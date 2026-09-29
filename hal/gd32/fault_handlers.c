/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Real NMI_Handler / HardFault_Handler / MemManage_Handler /
 * BusFault_Handler / UsageFault_Handler, replacing the vendor startup
 * file's weak aliases to Default_Handler (an unconditional `b .`).  See
 * https://github.com/alplabai/gd32-bridge-firmware/issues/36.
 *
 * Vendor confirmation (before writing anything here): all five names are
 * declared `.weak` in vendors/gd32_firmware_library/upstream/Firmware/
 * CMSIS/GD/GD32G5x3/Source/GCC/startup_gd32g5x3.S, each `.thumb_set` to
 * Default_Handler (lines 266-279), and the vector table at that same
 * file's __gVectors (lines 108-113) references them by exactly these
 * names -- an ODR-correct C definition here overrides the alias at link
 * time with no vendor-side change needed.
 *
 * ---- What each handler records, and why ----
 *
 * The stacked PC (the faulting instruction) and LR (the faulting
 * context's return address) are the useful minimum to locate a fault in
 * the field without a live debugger.  For the four CPU fault exceptions
 * that comes with SCB->CFSR / SCB->HFSR, and SCB->MMFAR / SCB->BFAR when
 * their CFSR validity bit says they're live (ARMv8-M ARM; see
 * fault_diag.h, which owns that decision as a pure, host-testable
 * function).  For NMI, CFSR/HFSR don't apply -- the useful minimum is
 * SYSCFG_STAT (tells the five armed sources apart: CKMNMIIF,
 * FLASHECCIF, SRAM0/1ECCMEIF, TCMSRAMECCMEIF) plus FMC_ECCCS when
 * FLASHECCIF fired, the one source issue #36 shows is concretely
 * reachable (an OTA rollback CRC walk over an interrupted flash program,
 * see the issue's "Failure" section). The SRAM0/1/TCMSRAM ECC address
 * fields in SYSCFG_CFG3/4/5 are deliberately NOT decoded here: whether
 * SRAM ECC is even armed on shipped modules depends on FMC_OBCTL's
 * SRAM_ECCEN option byte, which issue #36 says nobody has read yet on a
 * bench part -- decoding a field that may not be live is exactly the
 * "recording an address the architecture says is meaningless" trap this
 * file exists to avoid.
 *
 * ---- Where the record goes, and why ----
 *
 * RTC_BKP0..7 (backup domain 32-bit registers, GD32G553 User Manual
 * Rev1.2 p.568): "System reset: no effect", i.e. they survive exactly
 * the NVIC_SystemReset() this file issues.  Neither toolchain/
 * gd32g553_flash.ld nor gd32g553_app_slot.ld.in defines a no-init SRAM
 * region, and both are owned by open PR #94 -- adding one here would
 * both expand scope and step on that PR.  Plain SRAM was considered and
 * rejected outright, not just as the higher-effort option: Reset_Handler
 * (startup_gd32g5x3.S "SramInit") unconditionally zeroes RAM from
 * 0x20000000 for a length it reads out of a chip register BEFORE
 * anything in this firmware runs again, on every single reset including
 * the one this file triggers -- so .bss, .noinit, or any other
 * SRAM-resident record would already be gone by the time a person or a
 * debugger could read it back. Backup registers need PMU_CTL0_BKPWEN
 * (write access to the backup domain, p.145) which in turn needs
 * RCU_APB1EN_PMUEN; both are plain register writes here, no linker or
 * init.c change of any kind.
 *
 * ---- Reset vs. halt ----
 *
 * Default: RESET (fault_reset_loop_bump() permitting -- see below), with
 * one exception: a debugger already attached (DCB->DHCSR &
 * DCB_DHCSR_C_DEBUGEN_Msk) gets a halt instead, so attaching to chase
 * the fault doesn't lose the race against our own reboot, and so CFSR /
 * SYSCFG_STAT / FMC_ECCCS stay live in their real registers instead of
 * being cleared out from under the debugger.
 *
 * Reasoning: this is a supervisor on a deployed, normally-unattended
 * board (issue #36's whole point is that the host currently sees a dead
 * SPI/I2C link with zero diagnostic).  A reset the host can observe as
 * "the bridge came back" beats a silent hang every time nobody is
 * watching with a debugger, and the record above means the reset does
 * NOT destroy the evidence the way a bare reset would.  This is a real
 * trade-off, not a free win -- it can, even briefly, glitch whatever the
 * GD32 was driving (rails / reset lines it gates for the rest of the
 * SoM, per issue #36's own "Failure" section) while its own
 * Reset_Handler and bridge_hw_init() re-run.  What would change my mind:
 * if bench validation shows that glitch is real and matters (e.g. a
 * rail sags enough to reset something downstream), the fix is to flip
 * the default to halt, not to add sequencing here -- that's out of this
 * file's scope. HardFault/MemManage/BusFault/UsageFault are unlikely to
 * fire in the field at all on this bare-metal, no-RTOS, no-dynamic-
 * memory firmware (they need a real code/data bug, not just an ECC or
 * clock event) but get the same policy for one reason: five different
 * policies for five vectors that all land in the same "the supervisor
 * stopped" failure mode would be its own bug waiting to happen.
 *
 * ---- Bounding the reset loop ----
 *
 * fault_reset_loop_bump() persists a consecutive-fault count in
 * RTC_BKP7 (same backup domain, same survives-reset property).  Past
 * FAULT_RESET_LOOP_LIMIT consecutive fault-triggered resets, this file
 * halts instead of resetting again -- an unbounded "fault, reset, fault
 * again immediately" loop (the exact OTA/flash-ECC scenario issue #36
 * describes: the bootloader's CRC walk re-reads the same damaged
 * doubleword on every boot) is worse than the original hang, since it
 * now also thrashes the board's rails/reset lines on every cycle instead
 * of just once.
 *
 * Known limitation, stated plainly: nothing clears RTC_BKP7 on a
 * healthy boot, because that belongs in bridge_hw_init() (owned by open
 * PRs #75/#81/#84/#86 -- out of this file's scope per the task, and
 * genuinely: it needs to run after the application decides the boot is
 * healthy, not merely reached, which is a policy this file has no basis
 * to invent). Until a future change adds that, this counter is
 * effectively lifetime-of-backup-domain-power, not per-incident: four
 * total faults ever (not four in a row) trips the halt path. Recovering
 * from that halted state needs either a debugger write to RTC_BKP7 or
 * removing backup-domain power entirely (VBAT, if populated on this
 * board -- unconfirmed, bench item). That is a real cost of not editing
 * init.c; it is still strictly better than today, where every one of
 * these five sources hangs forever on the very first occurrence.
 */

#include <stdbool.h>
#include <stdint.h>

#include "gd32g5x3.h"

#include "fault_diag.h"

/* Past this many consecutive fault-triggered resets (RTC_BKP7, see file
 * header), stop resetting and halt instead. */
#define FAULT_RESET_LOOP_LIMIT 3u

/* Backup-domain write access: RCU_APB1EN_PMUEN clocks the PMU so
 * PMU_CTL0 is writable at all; PMU_CTL0_BKPWEN then gates writes to the
 * RTC_BKPx block itself (UM p.145).  Both are idempotent -- safe to call
 * unconditionally from every fault entry regardless of what bridge_hw_init()
 * already enabled. */
static void fault_backup_unlock(void)
{
	RCU_APB1EN |= RCU_APB1EN_PMUEN;
	PMU_CTL0 |= PMU_CTL0_BKPWEN;
}

static void fault_record_save(const fault_record_t *rec)
{
	RTC_BKP0 = rec->magic;
	RTC_BKP1 = rec->pc;
	RTC_BKP2 = rec->lr;
	RTC_BKP3 = rec->status_primary;
	RTC_BKP4 = rec->status_secondary;
	RTC_BKP5 = rec->mmfar;
	RTC_BKP6 = rec->bfar;
}

/* Bumps and returns the consecutive-fault counter.  Saturates instead of
 * wrapping so a pathological run can't wrap back through 0 and look
 * like a fresh start. */
static uint32_t fault_reset_loop_bump(void)
{
	uint32_t count = RTC_BKP7;

	if (count < 0xFFFFFFFFu) {
		count++;
	}
	RTC_BKP7 = count;
	return count;
}

/* Halts with interrupts off.  Used both for "a debugger is already
 * attached" and "the reset loop limit tripped" -- see file header. */
static void fault_halt(void)
{
	__disable_irq();
	for (;;) {
		__NOP();
	}
}

/* Common tail for all five vectors.  `frame` is the hardware-pushed
 * exception frame (r0,r1,r2,r3,r12,lr,pc,xpsr -- this firmware builds
 * with -mfloat-abi=soft, so there is no lazy-FP state to account for);
 * `fault_type` is the FAULT_TYPE_* tag the calling trampoline passed in
 * r1.  Reached only from the naked trampolines below, never called
 * directly -- not `static` so the trampolines' hand-written branch has a
 * stable external symbol to target regardless of inlining/whole-file
 * optimisation. */
_Noreturn void fault_common_handler(uint32_t *frame, uint32_t fault_type)
{
	fault_record_t rec;
	uint32_t       pc = frame[6];
	uint32_t       lr = frame[5];

	if (fault_type == (uint32_t)FAULT_TYPE_NMI) {
		uint32_t stat = SYSCFG_STAT;
		uint32_t fmc  = (stat & SYSCFG_STAT_FLASHECCIF) ? FMC_ECCCS : 0u;

		fault_diag_build_nmi(&rec, pc, lr, stat, fmc);
	} else {
		uint32_t cfsr  = SCB->CFSR;
		uint32_t hfsr  = SCB->HFSR;
		uint32_t mmfar = SCB->MMFAR;
		uint32_t bfar  = SCB->BFAR;

		fault_diag_build_cpu(&rec, (fault_type_t)fault_type, pc, lr, cfsr, hfsr, mmfar, bfar);
	}

	fault_backup_unlock();
	fault_record_save(&rec);

	uint32_t loop_count        = fault_reset_loop_bump();
	bool     debugger_attached = (DCB->DHCSR & DCB_DHCSR_C_DEBUGEN_Msk) != 0u;

	if (debugger_attached) {
		/* Leave CFSR / SYSCFG_STAT / FMC_ECCCS exactly as the fault left
		 * them -- the record above is already saved, but a live debugger
		 * gets more from the real registers than from our backup-register
		 * copy of them. */
		fault_halt();
	}

	if (loop_count > FAULT_RESET_LOOP_LIMIT) {
		fault_halt();
	}

	/* About to reset -- clear the source(s) so the very first instruction
	 * after reset doesn't walk straight back into a still-pending flag.
	 * (NVIC_SystemReset() resets SYSCFG/SCB to their power-on values too,
	 * which would make this redundant on THIS silicon, but the mechanism
	 * is documented per-source rather than assumed away.) */
	if (fault_type == (uint32_t)FAULT_TYPE_NMI) {
		if (rec.status_secondary != 0u) {
			/* w1c; UM p.94 Note 4: clearing ECCDET0 also clears
			 * SYSCFG_STAT's FLASHECCIF. */
			FMC_ECCCS = FMC_ECCCS_ECCDET0;
		}
		SYSCFG_STAT = rec.status_primary; /* rc_w1: write back what we read to clear it */
	} else {
		SCB->CFSR = rec.status_primary; /* rc_w1 sub-registers (MMFSR/BFSR/UFSR) */
	}

	NVIC_SystemReset();
	for (;;) {
		/* NVIC_SystemReset() never returns; kept only so this _Noreturn
		 * function has no implicit fall-through for the compiler to warn
		 * about. */
	}
}

/*
 * Naked trampolines: the C ABI prologue GCC would normally emit pushes
 * registers onto the SAME stack pointer the hardware exception frame
 * lives on, which would shift it out from under fault_common_handler
 * before that function could compute where the frame is.  Each
 * trampoline instead runs with NO prologue, resolves MSP vs. PSP off
 * EXC_RETURN bit 2 of LR (this firmware is bare-metal/no-RTOS and never
 * switches to PSP, so the MSP branch is the one that actually runs
 * today, but decoding it properly costs one extra instruction and is
 * correct regardless), and tail-branches into the shared handler with
 * the frame pointer in r0 and its fault-type tag in r1 -- the first two
 * AAPCS integer argument registers, matching fault_common_handler's
 * signature exactly.
 */
#define FAULT_TRAMPOLINE(name, tag) \
	__attribute__((naked)) void name(void) \
	{ \
		__asm volatile("movs r1, %0        \n" \
		               "movs r0, #4        \n" \
		               "mov  r2, lr        \n" \
		               "tst  r0, r2        \n" \
		               "beq  1f            \n" \
		               "mrs  r0, psp       \n" \
		               "b    2f            \n" \
		               "1: mrs  r0, msp    \n" \
		               "2: b    fault_common_handler \n" \
		               : \
		               : "i"(tag) \
		               : "r0", "r1", "r2"); \
	}

FAULT_TRAMPOLINE(NMI_Handler, FAULT_TYPE_NMI)
FAULT_TRAMPOLINE(HardFault_Handler, FAULT_TYPE_HARDFAULT)
FAULT_TRAMPOLINE(MemManage_Handler, FAULT_TYPE_MEMMANAGE)
FAULT_TRAMPOLINE(BusFault_Handler, FAULT_TYPE_BUSFAULT)
FAULT_TRAMPOLINE(UsageFault_Handler, FAULT_TYPE_USAGEFAULT)

#undef FAULT_TRAMPOLINE
