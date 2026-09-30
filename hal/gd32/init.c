/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 backend for the bridge HAL.  Selected by setting
 * BRIDGE_HAL_BACKEND=gd32 in CMakeLists.txt.  Links
 * against the GigaDevice firmware-library wrapper.  This repo does NOT
 * vendor that tree: pass -DGD32_VENDOR_DIR=<path> pointing at a checkout
 * of alp-sdk's vendors/gd32_firmware_library/ (a verbatim mirror of GD's
 * v1.5.0 release).  Left unset, the build falls back to
 * ../../vendors/gd32_firmware_library resolved against this source tree
 * -- the pre-split layout, when this tree was nested at
 * <alp-sdk>/firmware/gd32-bridge/.  See README.md "Build".
 *
 * Status:
 *   The hooks below have real bodies -- selecting this backend drives
 *   silicon.  The per-hook record that follows is the state AS LANDED,
 *   not a plan: DONE, PARTIAL (defaults accepted, the rest refused) or
 *   an unconditional sentinel where the SoM revision has no HW path.
 *   Whatever a hook refuses returns BRIDGE_HW_ERR_NOTIMPL, which
 *   reaches the wire as STATUS_NOSUPPORT (status_from_hw() in
 *   src/protocol.c).
 *
 * Per-hook state, in the order the bodies landed (increasing risk):
 *
 *   1. RESET_REASON          -- DONE: RCU_RSTSCK decode + RSTFC clear.
 *   2. GPIO_READ / WRITE     -- DONE: 21-pad map (18 E1M IO8..IO35 pads
 *                               + 2 Murata REG_ON sideband bits + 1
 *                               CAN_STBY sideband bit), boot configures
 *                               the 18 E1M pads as INPUT, high-Z (no
 *                               pull) so the carrier's own pulls define
 *                               the default -- see the boot-loop
 *                               comment below -- write auto-promotes
 *                               each to OUTPUT push-pull.  Bits 18/19
 *                               (BT_REG_ON/WL_REG_ON) instead boot
 *                               OUTPUT LOW -- module power is host
 *                               policy, this firmware never drives them
 *                               high on its own.  Bit 20 (CAN_STBY)
 *                               instead boots OUTPUT HIGH (standby) --
 *                               CAN-bus power-up is host policy, this
 *                               firmware never takes it live on its own.
 *   3. TRNG_READ             -- DONE: NIST SP800-90B mode init in
 *                               bridge_hw_init, DRDY-polled byte read
 *                               with bounded timeout.
 *   4. TMU_COMPUTE           -- DONE: 9 of 12 host functions via TMU
 *                               (tan / exp / tanh NOSUPPORT -- host
 *                               wrapper can libm-fallback in a future
 *                               commit).  F32 + Q31 paths.
 *   5. DAC_SET / GET         -- DONE: channel 0 -> DAC0_OUT0 / PA4,
 *                               channel 1 -> DAC1_OUT0 / PA6, mV<->12-bit
 *                               code at VREF=1800mV (V2N analog supply).
 *   6. PWM_SET / GET         -- DONE: 8 channels across TIMER0 + TIMER7
 *                               at 1 us LSB; pin AFs per datasheet Rev2.0.
 *   7. PWM_CONFIGURE         -- PARTIAL: defaults (edge-aligned, no
 *                               dead-time, no break input) accepted;
 *                               non-defaults NOSUPPORT pending follow-up.
 *   8. ADC_READ              -- DONE: 8-pad map across ADC0..3, single-
 *                               shot polling, mV<->code at VREF=1800mV.
 *   9. ADC_CONFIGURE         -- DONE: per-channel sample-cycle,
 *                               oversample-ratio and 6/8/10/12-bit
 *                               resolution settings cached + applied;
 *                               14/16-bit effective modes are NOSUPPORT.
 *   10. ADC_STREAM_*         -- DONE (§C.23): DMA0/1-backed
 *                               continuous acquisition.  Two parallel
 *                               streams; each owns a 1024-sample
 *                               circular ring buffer + a DMA channel
 *                               driving it peripheral-to-memory.
 *                               Host drains via polled stream_read;
 *                               write cursor recovered from the DMA
 *                               counter.
 *   11. QENC_READ / RESET    -- DONE: 4 encoders across TIMER1/2/3/4
 *                               in quadrature decoder mode 2 (X4).
 *   12. COUNTER_READ         -- DONE: Cortex-M33 DWT cycle counter
 *                               (32-bit free-running at core clock).
 *   13. PWM_CAPTURE_*        -- DONE (§C.15a): TIMERx input-capture
 *                               with polled drain.  V2N pad map binds
 *                               the COMPLEMENTARY (CHxN) outputs --
 *                               BEGIN reconfigures the channel as
 *                               input-capture but the captured edges
 *                               only land once a future hardware-
 *                               bring-up commit reworks the pad
 *                               routing onto the timer's main CHx
 *                               input mux.  Structural code path
 *                               (config + drain + ns conversion) is
 *                               complete and exercised end-to-end.
 *   14. PWM_SINGLE_PULSE     -- DONE: TIMERx OPM (one-pulse mode).
 *                               SPM and CAR are timer-wide, so a
 *                               one-shot answers STATUS_BUSY while a
 *                               sibling channel on the same timer has
 *                               a continuous PWM or capture claim
 *                               (#87); a PWM_SET flips back to
 *                               repetitive.
 *   15. TIMER_SYNC           -- DONE (§C.15b): master-slave SMC
 *                               config via timer_slave_mode_select
 *                               + timer_master_output0_trigger_source_select
 *                               + timer_master_slave_mode_config.
 *                               Wire `mode` byte translated to the
 *                               vendor's TIMER_SLAVE_MODE_* /
 *                               TIMER_QUAD_DECODER_MODE* encoding.
 *                               Slave ITIx looked up per (master, slave)
 *                               pair (timer_sync_iti_lookup, hal/gd32/
 *                               timer_sync_iti.c, UM p.570); an
 *                               unconnected pair returns INVAL.  Only
 *                               initialised TIMER0/TIMER7 are exposed;
 *                               TIMER19 id 2 returns RANGE (#142).
 *   16. POWER_MODE_SET       -- DONE (§C.15c + §C.25): mode 0/1
 *                               (run/sleep) accepted no-ops, mode 2
 *                               (deep-sleep) calls
 *                               pmu_to_deepsleepmode(LDO_LOWPOWER, WFI),
 *                               mode 3 (standby) calls
 *                               pmu_to_standbymode().  Wake-source
 *                               bitmap: GPIO enables PMU_WAKEUP_PIN0..4,
 *                               RTC + TIMER arm the RTC wakeup timer
 *                               at 0.5 ms LSB (IRC32K / DIV16); same
 *                               timer also honours wake_after_ms.
 *                               UART_RX / USB / ETH_LINK reject as
 *                               NOSUPPORT (no HW path on GD32G5).
 *   17. DA9292 fault forward -- 0xFF sentinel UNCONDITIONALLY: this
 *                               SoM rev wires the DA9292 fault nets
 *                               only to the Renesas (P37/P36); the
 *                               GD32 has no connection to them and no
 *                               I2C path to the PMIC.  Pin sampling
 *                               awaits a HW rev that mirrors the nets.
 *   18. ADC_DSP_*            -- DONE (§C.15d + §C.24): chain_open +
 *                               stage_push implemented as a 4-chain
 *                               pool with per-stage chunk reassembly
 *                               into a 260-byte buffer.  chain_bind
 *                               validates completeness + ordering
 *                               rules (FFT terminal, WINDOW preceding
 *                               FFT) and stores the binding on both
 *                               sides.  Runtime dispatch (#496) now
 *                               LANDED: a base-level pump routes bound
 *                               FIR/IIR chains through the FAC (filtered
 *                               STREAM_READ) and FFT chains through the
 *                               FFT block (CMD_ADC_SPECTRUM_READ; a plain
 *                               STREAM_READ answers NOSUPPORT).  One FAC
 *                               + one FFT block -> one filter + one FFT
 *                               stream at a time.
 *
 * A change that moves a hook between the states above updates this
 * header comment in the same commit.  Bench cadence is bounded by
 * maintainer access to the V2N EVK, so a hook can be code-complete
 * here and still carry the `needs-silicon` label until validated --
 * see docs/BENCH.md.
 *
 * Build assumptions:
 *   - arm-none-eabi-gcc on PATH (toolchain file
 *     toolchain/arm-none-eabi.cmake handles the rest).
 *   - the GigaDevice firmware-library tree reachable, either via
 *     -DGD32_VENDOR_DIR=<path> or the ../../vendors/gd32_firmware_library
 *     fallback (see the note at the top of this file).
 *   - Cortex-M33 + Thumb + soft-float ABI (matches the GigaDevice
 *     library's compile flags).
 *
 * Layout note (2026-06-05): this backend was split move-only into per-peripheral TUs under hal/gd32/ (fw v0.2.8 refactor); this file owns boot bring-up + board-level reads, shared tables/helpers are declared in gd32_common.h.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bridge_hw.h"
#include "bridge_board_config.h"

/* The wrapper's PUBLIC include directories expose the GigaDevice device
 * header.  It supplies the CMSIS/core definitions and pulls this project's
 * hal/gd32g5x3_libopt.h selector, which exposes the peripheral declarations
 * used by the real backend.  The vendor wrapper compiles its driver archive
 * independently; libopt controls declarations, not which driver units link. */
#include "gd32g5x3.h"
#include "gd32g5x3_dbg.h"
#include "gd32_common.h"
#include "reset_reason.h"

_Static_assert(RCU_RSTSCK_OBLRSTF == BRIDGE_RESET_RSTSCK_OBLRSTF, "vendor OBLRSTF mask changed");
_Static_assert(RCU_RSTSCK_RSTFC == BRIDGE_RESET_RSTSCK_RSTFC, "vendor RSTFC mask changed");
_Static_assert(RCU_RSTSCK_BORRSTF == BRIDGE_RESET_RSTSCK_BORRSTF, "vendor BORRSTF mask changed");
_Static_assert(RCU_RSTSCK_EPRSTF == BRIDGE_RESET_RSTSCK_EPRSTF, "vendor EPRSTF mask changed");
_Static_assert(RCU_RSTSCK_PORRSTF == BRIDGE_RESET_RSTSCK_PORRSTF, "vendor PORRSTF mask changed");
_Static_assert(RCU_RSTSCK_SWRSTF == BRIDGE_RESET_RSTSCK_SWRSTF, "vendor SWRSTF mask changed");
_Static_assert(RCU_RSTSCK_FWDGTRSTF == BRIDGE_RESET_RSTSCK_FWDGTRSTF,
               "vendor FWDGTRSTF mask changed");
_Static_assert(RCU_RSTSCK_WWDGTRSTF == BRIDGE_RESET_RSTSCK_WWDGTRSTF,
               "vendor WWDGTRSTF mask changed");
_Static_assert(RCU_RSTSCK_LPRSTF == BRIDGE_RESET_RSTSCK_LPRSTF, "vendor LPRSTF mask changed");

/* ----------------------------------------------------------------- */
/* Boot hooks (overrides of the weak defaults in src/main.c)         */
/* ----------------------------------------------------------------- */

/* Called once on entry to main() before the transport ISRs come online.
 * This brings up the backend's boot-global clocks, pads and peripheral
 * state; request-time operations live in the per-peripheral TUs.  Since
 * gh#54 a 50 ms SysTick retires the main loop's __WFI() on a fixed
 * cadence, so base-level housekeeping (bridge_hw_tick) advances whether
 * or not a transport interrupt arrives.  No DA9292 wiring exists on this
 * SoM rev; see bridge_hw_da9292_status_cached(). */
/* Sampled at the head of bridge_hw_init (#127); see gd32_common.h for
 * what reads them and why nothing acts on a mismatch yet.  Initialised to
 * the value the constants ASSUME so a debugger attaching before
 * bridge_hw_init has run does not read a spurious 0 and conclude the
 * clock tree is broken. */
uint32_t        bridge_core_clock_hz      = PWM_TIMER_CLK_HZ;
bool            bridge_core_clock_matches = true;
static uint32_t s_reset_reason_snapshot;

/* gh#146: the stack region's lower bound, provided by both linker
 * scripts (toolchain/gd32g553_flash.ld, gd32g553_app_slot.ld.in --
 * _stack_limit = _sp - __stack_size).  Programmed into MSPLIM at the
 * head of bridge_hw_init(). */
extern uint32_t _stack_limit[];

/* I/O compensation cell verdict (gh#66): true after bridge_hw_init()
 * iff SYSCFG_CPSCTL.CPS_RDY confirmed the cell ready within its
 * bounded spin.  Volatile so a bench probe can read it after boot;
 * no functional consumer today -- the cell is enable-and-forget. */
volatile bool gpio_compensation_ready;

void bridge_hw_init(void)
{
	/* --- Stack costing + MSPLIM limit (gh#146) -----------------------
	 *
	 * THE COSTING (measured, not asserted): a full -Os build of the
	 * gd32 backend with -fstack-usage (real vendor tree,
	 * BRIDGE_OTA_PARTITIONED on) reports the worst single frame in the
	 * firmware's own TUs at 248 bytes (adc_stream.c's FAC decode: the
	 * int16 taps[BRIDGE_DSP_MAX_FIR_TAPS] + float fv[] pair, #132's
	 * bounded stack), then 120 (transport_i2c), then a band of 88-104
	 * (protocol dispatch frames, adc_stream, tmu, gpio).  The deepest
	 * executable chains:
	 *
	 *   CS-EXTI (prio 1) -> protocol_dispatch -> worst opcode handler
	 *     -> adc_stream path .......... <= ~248 + 104 + 88 + frame ~700
	 *   + nested I2C-EV (prio 2) -> protocol_dispatch -> handler ~300
	 *   + 2 exception frames (no FPU context: 32 B each) ........ ~64
	 *   ---------------------------------------------------- approx 1.1 K
	 *
	 * against __stack_size = 2K: roughly 55% of the region on the
	 * worst legal nesting, which is margin, not slack -- the ring of
	 * #36's NMI_Handler on top costs another frame, and #132's taps[]
	 * bound is what keeps the 248 from doubling.  Remeasure after any
	 * change to adc_stream.c's decode (see the -fstack-usage recipe in
	 * the commit message: cmake with -DCMAKE_C_FLAGS="-fstack-usage
	 * -Os", then read the .su files; the bootloader image is measured
	 * the same way against the SAME 2K).
	 *
	 * The limit is enforced, not just measured: MSPLIM (ARMv8-M,
	 * ARMv8-M ARM B3.1) turns any excursion below _stack_limit into a
	 * fault instead of a silent overflow into .heap / .bss.  This
	 * firmware never sets SHCSR.USGFAULTENA (reset default: 0), so
	 * that fault does NOT land in UsageFault_Handler -- it escalates
	 * straight to HardFault_Handler, which fault_handlers.c's #36
	 * work already records and resets from (CFSR/HFSR are the same
	 * registers either way; see that file's header for what "record"
	 * means).  This is FAIL-STOP, not just a record: the corruption
	 * is caught before the write lands, not diagnosed after.
	 *
	 * That escalation is also why STKOFHFNMIGN is set right after
	 * MSPLIM.  HardFault's own exception-entry stacking is, by
	 * default, checked against the SAME limit it exists to report --
	 * so a caught overflow can escalate a SECOND time, and the
	 * ARMv8-M ARM (B1.5.3) defines a limit violation during
	 * HardFault's own entry as a lockup, not a further escalation:
	 * the core halts with no vector taken at all, invisible to
	 * fault_handlers.c and, on some implementations, to an attached
	 * debugger.  STKOFHFNMIGN (SCB->CCR bit 10) tells HardFault (and
	 * NMI) to ignore the stack limit on their OWN entry so they are
	 * guaranteed to actually run; the cost is that this one
	 * exception-frame push can land up to 32 B below _stack_limit,
	 * trading a sliver of the containment guarantee for a fault
	 * record and reset over a silent lockup.  Set first thing -- ahead
	 * of even the NVIC priority-group / SYSCFG-clock setup below -- so
	 * everything but the vendor startup's own frames (which are
	 * shallow, register-only) is guarded; neither write needs special
	 * care beyond running once before any nesting. */
	__set_MSPLIM((uint32_t)_stack_limit);
	SCB->CCR |= SCB_CCR_STKOFHFNMIGN_Msk;

	/* Reset-cause snapshot, taken ONCE, before any later boot work.  The
	 * BOOTLOADER stashes RCU_RSTSCK in RTC_BKP8 (backup domain, survives
	 * NVIC_SystemReset) and clears RSTFC itself; RTC_BKP8 == 0 means no
	 * bootloader ran (full-flash image or a pre-stash bootloader), so latch
	 * and clear the live register instead.  Either way the raw value is
	 * kept in s_reset_reason_snapshot and CMD_RESET_REASON decodes it
	 * non-destructively, so repeated reads agree. */
	RCU_APB1EN |= RCU_APB1EN_PMUEN;
	PMU_CTL0 |= PMU_CTL0_BKPWEN;
	s_reset_reason_snapshot = RTC_BKP8;
	if (s_reset_reason_snapshot != 0u) {
		RTC_BKP8 = 0u;
	} else {
		s_reset_reason_snapshot = bridge_reset_reason_latch_and_clear(&RCU_RSTSCK);
	}

	/* The priority numbers in bridge_board_config.h mean preemption levels
	 * only under PRE2_SUB2. A Path-A bootloader handoff preserves AIRCR, and
	 * the vendor helper otherwise retains a valid inherited grouping, so set
	 * the bridge policy before configuring any NVIC line or, on Path-A,
	 * unmasking IRQs. */
	nvic_priority_group_set(NVIC_PRIGROUP_PRE2_SUB2);

	/* SYSCFG hosts the TIMER quadrature-decoder mode fields
     * (SYSCFG_TIMERxCFG0.TSCFGy) that qenc_channel_init() programs
     * below, as well as the EXTI source mux that spi_cs_exti_init()
     * programs later.  Its APB2 clock gate (RCU_APB2EN bit 14,
     * SYSCFGEN) is off out of reset, and writes to a gated APB
     * block do not stick, so this must run before ANY SYSCFG write
     * -- in particular before the quadrature-encoder bring-up further
     * down this function.  spi_cs_exti_init() keeps its own
     * rcu_periph_clock_enable(RCU_SYSCFG) too (the call is
     * idempotent); it must not become dependent on init ordering for
     * its own SYSCFG writes. */
	rcu_periph_clock_enable(RCU_SYSCFG);

#if defined(BRIDGE_OTA_PARTITIONED) && defined(BRIDGE_APP_SLOT_BASE)
	/* OTA Path-A: the app runs from a flash slot, not 0x08000000, so move
     * the vector table off the vendor SystemInit default before any NVIC
     * use.  Runs first (main calls bridge_hw_init before the transports
     * enable interrupts). */
	SCB->VTOR = (uint32_t)(BRIDGE_APP_SLOT_BASE);
	__DSB();
	__ISB();
	/* The Path-A bootloader hands off with PRIMASK SET (it runs
     * __disable_irq() before swapping MSP/VTOR for the jump).  Do NOT
     * clear it here (gh#257): everything below this point through the
     * end of bridge_hw_init() is boot-only peripheral bring-up --
     * roughly two dozen bare rcu_periph_clock_enable() read-modify-
     * writes on the RCU enable registers -- and none of it needs
     * interrupts live.  Re-enabling here used to open exactly that
     * window: once PRIMASK is clear, a transport ISR (once its own
     * NVIC line is unmasked by transport_spi_init/transport_i2c_init,
     * which main() calls AFTER this function returns) could in
     * principle pre-empt one of these RMWs and have a bit it just set
     * silently erased -- the same hazard #224 closed for the RUNTIME
     * call sites.  __enable_irq() now runs once, at the very end of
     * bridge_hw_init(), after the last RCU write. */
#endif

	/* A breakpoint must preserve the state it is inspecting.  Hold the
	 * timer counters this backend owns (PWM 0/7, quadrature 1..4 and ADC
	 * pacing 5/6), both watchdog counters and I2C0's SMBus timeout while
	 * the CM33 is halted (#55).  These bits affect only debug halt, not
	 * normal execution.  Do not enable DBG_CTL0 low-power holds here: the
	 * manual changes their clock source/standby behaviour, so that remains
	 * an explicit SWD bench choice rather than shipped firmware policy. */
	DBG_CTL1 |= DBG_CTL1_TIMER1_HOLD | DBG_CTL1_TIMER2_HOLD | DBG_CTL1_TIMER3_HOLD |
	            DBG_CTL1_TIMER4_HOLD | DBG_CTL1_TIMER5_HOLD | DBG_CTL1_TIMER6_HOLD |
	            DBG_CTL1_WWDGT_HOLD | DBG_CTL1_FWDGT_HOLD | DBG_CTL1_I2C0_HOLD;
	DBG_CTL2 |= DBG_CTL2_TIMER0_HOLD | DBG_CTL2_TIMER7_HOLD;

	/* ORDERING (merge of #61's se_reset_init and #127's clock sample):
     * se_reset_init() goes FIRST and that is load-bearing -- see its
     * comment below on PC13 floating in ANALOG mode from reset.  The
     * clock sample that follows is a pure register read with no side
     * effects and derives nothing se_reset_init() needs (the SE reset is
     * a GPIO push-pull level, with no clock-derived timing anywhere in
     * hal/gd32/se_reset.c), so putting it second costs the #127 check
     * nothing: nothing between these two statements consumes
     * SystemCoreClock either. */
	/* Secure-element reset (SE_RST = PC13): promote it to a released-
     * level push-pull output FIRST, ahead of every other peripheral
     * bring-up below.  PC13 has no entry in `gpio_pad_map` (see
     * hal/gd32/gpio.c), so from reset until whichever statement first
     * touches it, it sits at its GPIO POR default -- ANALOG mode,
     * with the input buffer and both pull resistors disabled (UM
     * Rev1.2 p.270 §7.3.7): genuinely floating, not merely un-pulled.
     * The TRNG bring-up a few statements below this used to run first
     * and can alone spend up to ~1 ms in that state (hal/gd32/trng.c)
     * -- but that is NOT the dominant exposure: the pre-main() SramInit
     * loop floats PC13 for ~20-25 ms before bridge_hw_init() is ever
     * called (see the header comment in hal/gd32/se_reset.c for the
     * derivation).  That window is not fixable by moving this call any
     * earlier -- there is nowhere earlier in this backend to move it
     * to -- so gh#61 stays open after this reorder; this promotion
     * only bounds boot from main() onward and closes the separate
     * sub-microsecond OCTL-preload-vs-mode-promote gap inside
     * se_reset_configure() itself.  Needs only its own GPIOC clock;
     * the bulk AHB2 enable just below
     * re-enables it harmlessly for the rest of port C's pads. */
	rcu_periph_clock_enable(RCU_GPIOC);
	se_reset_init();

	/* #127: sample the clock the vendor's SystemInit() ACTUALLY left
     * running, before anything derived from it is programmed.  Until this
     * call the repo never referenced SystemCoreClock at all: every timing
     * constant in gd32_common.h -- PWM_TIMER_CLK_HZ, PWM_TIMER_PRESCALER,
     * BRIDGE_ADC_PACE_CLK_HZ, and the DWT "~4.63 ns LSB" claim in
     * counter.c -- was asserted against a number no code checked, set by a
     * SystemInit() that lives in another repository and whose variant is
     * chosen by a wrapper this repo does not own.
     *
     * SystemCoreClockUpdate() re-derives the value from the live RCU
     * registers rather than trusting the compile-time initialiser, so this
     * is the one place the firmware can find out it is running on the
     * wrong clock tree.  It is a pure register read plus arithmetic: no
     * side effects, safe this early.
     *
     * The result is recorded, not acted on.  See the declarations in
     * gd32_common.h for why refusing supervised outputs on a mismatch is
     * a follow-up rather than part of this change. */
	SystemCoreClockUpdate();
	bridge_core_clock_hz      = SystemCoreClock;
	bridge_core_clock_matches = (SystemCoreClock == PWM_TIMER_CLK_HZ);

	/* --- Explicit NMI-source arming (gh#36) ---------------------------
	 *
	 * Five NMI sources come up ARMED by the reset values of
	 * SYSCFG_CFG3 (0xXXXX X00F), CFG4 (0xXXX0 XX03) and CFG5
	 * (0xXXXX XX03) (UM Rev1.2 p.64-67): CKNMIIE (HXTAL clock
	 * failure), FLASHECCIE (flash double-bit ECC -- unconditional,
	 * UM p.93 "When two errors are detected, the ECCDET0 bit ... is
	 * set and a NMI is generated"), and the SRAM0/SRAM1/TCMSRAM
	 * multi-bit + single-bit ECC enables.  Until gh#36's handler
	 * work (hal/gd32/fault_handlers.c) every one of them vectored
	 * into the vendor Default_Handler's infinite loop with no
	 * watchdog armed -- a permanent, undiagnosable wedge.
	 *
	 * The handlers exist now, so the armed set is KEPT -- but written
	 * explicitly, read-modify-write on exactly the enable bits, so
	 * the set of live NMI sources is a decision recorded in this
	 * source rather than an accident of a reset value the manual
	 * prints with X digits.  The three single-bit-correction enables
	 * stay armed but inert: they route to the SYSCFG NVIC line,
	 * which this firmware never enables.  The SRAM multi-bit paths
	 * are option-byte dependent (FMC_OBCTL bit 24 SRAM_ECCEN, p.121
	 * -- unread on a bench part, see gh#36's verification list); the
	 * flash path needs no such confirmation and is live on every
	 * part.
	 *
	 * The BOOTLOADER runs on the same reset defaults and links the
	 * same handler set (#182), but does not write these registers
	 * yet; it arms the identical set by reset value.  Any future
	 * change here must be mirrored there -- both images must agree. */
	SYSCFG_CFG3 |= (SYSCFG_CFG3_CKMNMIIE | SYSCFG_CFG3_FLASHECCIE | SYSCFG_CFG3_SRAM0ECCSEIE |
	                SYSCFG_CFG3_SRAM0ECCMEIE);
	SYSCFG_CFG4 |= (SYSCFG_CFG4_SRAM1ECCSEIE | SYSCFG_CFG4_SRAM1ECCMEIE);
	SYSCFG_CFG5 |= (SYSCFG_CFG5_TCMSRAMECCSEIE | SYSCFG_CFG5_TCMSRAMECCMEIE);

	/* I/O compensation cell (gh#66): set SYSCFG_CPSCTL.CPS_EN (bit 0,
	 * UM Rev1.2 p.68-69 §1.7.15, offset 0x3C) and wait for CPS_RDY
	 * (bit 8, read-only) with a BOUNDED spin in the same shape as the
	 * VREFRDY wait below -- recording the verdict, never hanging.
	 * The cell controls output commutation slew (tfall/trise) across
	 * the whole I/O ring: the SPI pads' speed-10 class spans tR/tF
	 * 4.0-6.6 ns across supply alone (Datasheet Rev2.0 p.130 Table
	 * 4-30) -- 65% spread before process and temperature -- and a
	 * marginal unit at a corner has less edge margin against the
	 * master's CS-to-first-SCK window than a bench unit shows.
	 * RCU_SYSCFG is NOT re-enabled here; transport_hw_gd32.c already
	 * enabled it.  The compensation-code programming path UM p.271
	 * describes is deliberately not attempted: Rev1.2 defines no
	 * register for it and CPSCTL has no code field -- only the
	 * enable-and-let-hardware-compensate mode is reachable.
	 * Sequencing per gh#66: this is a PVT-robustness measure on
	 * links that already work, so it lands in the same change as the
	 * I2C drive-class drop, and BOTH need the bench re-validation
	 * the issue names (SPI setup window, PWM edges, I2C falls)
	 * before the verdict counts as settled. */
	{
		bool     comp_ready = false;
		uint32_t spins      = 0u;
		SYSCFG_CPSCTL |= SYSCFG_CPSCTL_CPS_EN;
		while (spins++ < 200000u) {
			if (0u != (SYSCFG_CPSCTL & SYSCFG_CPSCTL_CPS_RDY)) {
				comp_ready = true;
				break;
			}
		}
		/* Verdict recorded for a bench read (volatile: no other
		 * reader today; a never-ready cell must not hang boot). */
		gpio_compensation_ready = comp_ready;
	}

	/* Enable AHB2 clocks for every GPIO port the pad map references.
     * The chip's RCU keeps unused GPIO ports clock-gated to save
     * power; we enable A..F unconditionally because the E1M IO map
     * spans those ports.  Port G isn't referenced by any pad. */
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOD);
	rcu_periph_clock_enable(RCU_GPIOE);
	rcu_periph_clock_enable(RCU_GPIOF);

	/* Pad map parking (gh#66): leave every entry in `gpio_pad_map`
	 * at its CTLy = 0b11 ANALOG reset state -- input buffer and both
	 * pull resistors disabled (UM Rev1.2 p.270 §7.3.7), which is what
	 * all twenty-one pads already reset to (p.275).  The old INPUT +
	 * PULL_UP park sank 1.8 V / 40 kΩ = 45 µA per pad continuously
	 * from boot into every pad a carrier holds LOW (Datasheet
	 * Rev2.0 p.128 Table 4-28: RPU = 40 kΩ, "value guaranteed by
	 * design") -- on a part whose whole point in modes 2/3 is low
	 * standing current.  bridge_hw_gpio_read() now promotes a pad to
	 * INPUT + PULLUP lazily on the first CMD_GPIO_READ that names
	 * it (with a settling allowance for the pull-up charging the pad
	 * capacitance), mirroring the existing lazy OUTPUT promotion in
	 * bridge_hw_gpio_write(); gpio_is_output[] stays the write-side
	 * truth.  Tradeoff made deliberately per gh#66: an unconnected
	 * E1M IO reads 0 only while its pad is parked analog (ISTAT
	 * reads 0 in analog mode, UM p.270) -- the first READ names it,
	 * promotes it, and every later read is the pulled-up level.
	 *
	 * Also settles the SDIO mux regression bench-proven 2026-09-26 on
	 * E1M-V2M103 / E1M-X EVK: the carrier's SDIO mux (microSD vs M.2
	 * Wi-Fi) is steered by two of these pads (IO27 SDIO_MUX_SEL, IO29
	 * SDIO_MUX_EN) and the carrier's own pulls already select microSD
	 * when the pads are undriven -- but the GD32's old INPUT+PULLUP
	 * park used to win the moment this firmware ran, so both pads
	 * read 1 and the mux disabled / flipped away from microSD (Linux:
	 * `mmc1: tuning execution failed: -5`, `card aaaa removed`).  The
	 * analog park below carries no internal pull either, so the
	 * carrier's pulls own the default the same as the high-Z fix
	 * bench-validated (SDR104 @ 200 MHz, 3x1 GiB md5-identical
	 * transfers) confirmed they must.  The BT/WL_REG_ON pads (bits
	 * 18/19) and CAN_STBY (bit 20) are NOT skipped in the loop below --
	 * unlike a plain gpio_mode_set() park, leaving them at their
	 * analog reset state here is a no-op these pads' own OUTPUT
	 * promotion blocks further down safely override -- so this loop
	 * needs no special-casing for them. */
	for (size_t i = 0; i < GPIO_PAD_MAP_COUNT; ++i) {
		gpio_is_output[i] = false;
	}

	/* Murata LBEE5HY2FY-922 Wi-Fi/BT REG_ON lines (bits 18/19): boot
     * OUTPUT driven LOW = module OFF.  Power policy belongs to the
     * HOST, not this firmware -- a host turns the module on by
     * issuing CMD_GPIO_WRITE on these bits as part of its own
     * WiFi/BT bring-up; the IO-MCU only proxies the line and must
     * never assert it autonomously.  OUTPUT LOW (rather than left
     * as the default INPUT high-Z) gives a defined OFF state
     * instead of floating against the module's internal 50 k
     * pull-downs, and a clean low->high edge once the host asserts.
     * gpio_is_output[i] is set so the pad is not re-promoted (and
     * does not glitch) on the host's first GPIO_WRITE, and reads
     * report the driven level. */
	for (size_t i = GPIO_PAD_BT_REG_ON; i <= GPIO_PAD_WL_REG_ON; ++i) {
		gpio_bit_reset(gpio_pad_map[i].periph, gpio_pad_map[i].pin);
		gpio_output_options_set(
		    gpio_pad_map[i].periph, GPIO_OTYPE_PP, GPIO_OSPEED_12MHZ, gpio_pad_map[i].pin);
		gpio_mode_set(
		    gpio_pad_map[i].periph, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, gpio_pad_map[i].pin);
		gpio_is_output[i] = true;
	}

	/* CAN_STBY (bit 20): boot OUTPUT driven HIGH = both on-module
     * TCAN1044 transceivers in standby.  Power-up policy belongs to
     * the HOST, not this firmware -- a host takes the bus live by
     * issuing CMD_GPIO_WRITE on this bit as part of its own CAN
     * bring-up; the IO-MCU only proxies the line and must never drive
     * it low autonomously.  gpio_is_output[i] is set so the pad is not
     * re-promoted (and does not glitch) on the host's first
     * GPIO_WRITE, and reads report the driven level. */
	gpio_bit_set(gpio_pad_map[GPIO_PAD_CAN_STBY].periph, gpio_pad_map[GPIO_PAD_CAN_STBY].pin);
	gpio_output_options_set(gpio_pad_map[GPIO_PAD_CAN_STBY].periph,
	                        GPIO_OTYPE_PP,
	                        GPIO_OSPEED_12MHZ,
	                        gpio_pad_map[GPIO_PAD_CAN_STBY].pin);
	gpio_mode_set(gpio_pad_map[GPIO_PAD_CAN_STBY].periph,
	              GPIO_MODE_OUTPUT,
	              GPIO_PUPD_NONE,
	              gpio_pad_map[GPIO_PAD_CAN_STBY].pin);
	gpio_is_output[GPIO_PAD_CAN_STBY] = true;

	/* TRNG bring-up: configure + enable only.  The NIST pipeline's
     * first conditioned word can lag past any boot-time wait we are
     * willing to spin; readiness is promoted lazily by the first
     * TRNG_READ's short DRDY poll (by which point the host's boot
     * settle has given the analog source seconds, not milliseconds). */
	trng_started = trng_start();
	if (trng_started) {
		/* Boot-time readiness grace: the transports aren't up yet, so
         * spending a few milliseconds here is free and spares the
         * FIRST host TRNG_READ the one-time conditioning latency
         * (observed on the functional tier: the inaugural read paid
         * the promotion and answered STATUS_IO once).  Bounded; the
         * lazy per-read path remains the safety net. */
		for (unsigned round = 0u; round < 8u && trng_started && !trng_ready; ++round) {
			(void)trng_poll_ready();
		}
	}

	/* TMU clock enable.  Per-op configuration happens in
     * bridge_hw_tmu_compute because the mode + I/O width vary per
     * call. */
	rcu_periph_clock_enable(RCU_TMU);

	/* DAC bring-up.  Two channels per `dac_channels[]`:
     *   ch0 -> DAC0_OUT0 -> PA4
     *   ch1 -> DAC1_OUT0 -> PA6
     * Configure both pads as analog (GPIOA clock was enabled
     * above for the IO pad map), enable each DAC peripheral in
     * NORMAL_PIN_BUFFON mode so the output drives the pad
     * through the chip's built-in buffer. */
	gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_4 | GPIO_PIN_6);
	rcu_periph_clock_enable(RCU_DAC0);
	rcu_periph_clock_enable(RCU_DAC1);
	for (size_t i = 0; i < DAC_CHANNEL_COUNT; ++i) {
		dac_deinit(dac_channels[i].periph);
		dac_trigger_disable(dac_channels[i].periph, dac_channels[i].out);
		dac_wave_mode_config(dac_channels[i].periph, dac_channels[i].out, DAC_WAVE_DISABLE);
		dac_mode_config(dac_channels[i].periph, dac_channels[i].out, NORMAL_PIN_BUFFON);
		dac_enable(dac_channels[i].periph, dac_channels[i].out);
	}

	/* Free-running counter: enable the Cortex-M33 DWT cycle counter.
     * TRCENA in CoreDebug->DEMCR gates the entire DWT/ITM trace block;
     * setting CYCCNTENA in DWT->CTRL starts the 32-bit free-running
     * counter at the core clock rate (216 MHz on the GD32G553 in the
     * stock clock config -> ~19.9 s wrap, ~4.63 ns LSB).  The counter
     * is the source for bridge_hw_counter_read(). */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0u;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* PWM bring-up: configure the 8 PWM pads as alt-function outputs,
     * enable TIMER0 + TIMER7 clocks, run the per-timer + per-channel
     * init.  After this the timers are running at 1 us tick with 0%
     * duty on every channel; bridge_hw_pwm_set programs both the
     * channel compare register and the timer ARR per call. */
	for (size_t i = 0; i < PWM_CHANNEL_COUNT; ++i) {
		const gd32_pwm_ch_t *ch = &pwm_channels[i];
		gpio_mode_set(ch->gpio_port, GPIO_MODE_AF, GPIO_PUPD_NONE, ch->gpio_pin);
		gpio_output_options_set(ch->gpio_port, GPIO_OTYPE_PP, GPIO_OSPEED_12MHZ, ch->gpio_pin);
		gpio_af_set(ch->gpio_port, ch->gpio_af, ch->gpio_pin);
	}
	rcu_periph_clock_enable(RCU_TIMER0);
	rcu_periph_clock_enable(RCU_TIMER7);
	pwm_timer_init(TIMER0);
	pwm_timer_init(TIMER7);
	for (size_t i = 0; i < PWM_CHANNEL_COUNT; ++i) {
		pwm_channel_init(&pwm_channels[i]);
	}

	/* Analog REFERENCE bring-up -- MUST precede any ADC/DAC use.
     *
     * On this module revision the converters' reference node is served
     * by the GD32's ON-CHIP reference buffer -- there is no external
     * reference source (hardware rationale in the internal bench
     * notes).  At reset VREF_CS defaults to 0x02 (HIPM high-impedance):
     * the buffer is parked, the reference node is undriven, and EVERY
     * ADC channel + both DACs reference a dead node -- the entire
     * analog subsystem read garbage/zero (silently, since the old ADC
     * assertions were ceiling-only and DAC_GET echoes the digital hold
     * register, not the pad).
     *
     * Fix: enable the buffer.  Its three targets (2.048 / 2.5 /
     * 2.9 V) all exceed the module's 1.8 V VDDA, so the buffer
     * regulates as high as the rail allows (~VDDA); the lowest
     * target (2.048 V) is the closest fit and least headroom stress.
     * Bench-proven: VREFEN -> VREFRDY sets, and a DAC->ADC copper
     * loopback then tracks 1:1 (DAC 2730 -> ADC 2730).  The reference
     * cancels ratiometrically in that loop, so correctness is
     * independent of the exact railed reference value; the absolute mV
     * scale (ADC_VREF_MV / DAC_VREF_MV) tracks the railed VDDA.
     *
	 * VREFRDY wait is BOUNDED (boot-time; the gh#54 SysTick is armed
	 * only at the END of bridge_hw_init, after this block): a spin
	 * cap, not an unbounded poll -- a never-ready buffer must not hang
	 * the bridge before the transports come online. */
	rcu_periph_clock_enable(RCU_VREF);
	vref_voltage_select(VREF_VOLTAGE_SEL_2_048V); /* lowest target = closest under VDDA */
	/* CLEAR HIPM first.  VREF_CS resets to 0x02 (HIPM high-impedance),
     * and the SPL's vref_enable() is a read-modify-write that only
     * sets VREFEN -- it PRESERVES the reset HIPM bit, leaving the
     * buffer output high-Z so VREFRDY never sets and the reference
     * node stays dead (silicon-caught 2026-06-05: VREF_CS read 0x03 =
     * VREFEN|HIPM, ADC still zero).  HIPM must be cleared for the
     * buffer to drive the node. */
	vref_high_impedance_mode_disable();
	vref_enable();
	for (uint32_t vr = 0u; vr < 100000u; ++vr) {
		if (vref_status_get() == SET) break; /* VREFRDY -- buffer locked */
	}
	/* Latch the verdict.  A buffer that never locked leaves vref_ok
     * false and every ADC/DAC op answers IO instead of serving
     * garbage referenced to a dead node (the exact silent failure the
     * VREF bring-up exists to cure).  vref_ready_check() re-probes on
     * each analog op, so a late lock self-promotes. */
	vref_ok = (vref_status_get() == SET);

	/* ADC bring-up: configure 8 pads as analog, enable all four ADC
	 * peripheral clocks, reset every converter, set each shared clock
	 * domain once, then run the per-converter init.  Calibration
	 * inside adc_periph_boot_init now runs against a LIVE reference (it
     * previously self-calibrated against the undriven reference node,
     * baking in a bogus offset); the VREF bring-up above is the
     * prerequisite that makes that calibration meaningful. */
	for (size_t i = 0; i < ADC_CHANNEL_MAP_COUNT; ++i) {
		gpio_mode_set(adc_channels_map[i].gpio_port,
		              GPIO_MODE_ANALOG,
		              GPIO_PUPD_NONE,
		              adc_channels_map[i].gpio_pin);
	}
	rcu_periph_clock_enable(RCU_ADC0);
	rcu_periph_clock_enable(RCU_ADC1);
	rcu_periph_clock_enable(RCU_ADC2);
	rcu_periph_clock_enable(RCU_ADC3);
	/* Boot-time calibration result intentionally not latched: a
     * converter that fails here is also the converter every request-
     * path op re-times against (the read path's bounded EOC wait +
     * self-heal), so the failure surfaces loudly on first use instead
     * of wedging boot. */
	adc_periph_boot_reset_all();
	adc_shared_clock_init();
	(void)adc_periph_boot_init(ADC0);
	(void)adc_periph_boot_init(ADC1);
	(void)adc_periph_boot_init(ADC2);
	(void)adc_periph_boot_init(ADC3);
	for (size_t i = 0; i < ADC_CHANNEL_MAP_COUNT; ++i) {
		adc_sample_cycles_cache[i]    = ADC_DEFAULT_SAMPLE_CYCLES;
		adc_resolution_bits_cache[i]  = ADC_RES_BITS_DEFAULT;
		adc_oversample_ratio_cache[i] = 1u; /* no oversampling */
	}

	/* Quadrature encoder bring-up: TIMER1..4 host the four E1M
     * encoder pairs.  Each timer uses its CH0 + CH1 input-capture
     * units as the X / Y quadrature inputs (decoder mode 2 -> X4). */
	rcu_periph_clock_enable(RCU_TIMER1);
	rcu_periph_clock_enable(RCU_TIMER2);
	rcu_periph_clock_enable(RCU_TIMER3);
	rcu_periph_clock_enable(RCU_TIMER4);
	for (size_t i = 0; i < QENC_CHANNEL_COUNT; ++i) {
		qenc_channel_init(&qenc_map[i]);
	}

#if defined(BRIDGE_OTA_PARTITIONED) && defined(BRIDGE_APP_SLOT_BASE)
	/* gh#257: re-enable interrupts here, now that every boot-time RCU
     * enable/config write above has landed.  On a plain power-on boot
     * PRIMASK is already clear and this is a no-op; on a Path-A
     * bootloader handoff (PRIMASK left SET, see the VTOR block above)
     * this is what lets the transports' interrupts fire once
     * transport_spi_init()/transport_i2c_init() unmask their NVIC
     * lines after this function returns. */
	__enable_irq();
#endif

	/* --- Periodic tick (gh#54) ----------------------------------------
	 *
	 * The main loop is `for (;;) { __WFI(); bridge_hw_tick(); }` and
	 * used to have NO periodic wake source at all: nothing between
	 * transport interrupts ever retired the wfi, so an armed OTA slot
	 * erase stalled mid-slot the moment the host stopped polling, and
	 * a bound DSP pump stopped producing.  Arm SysTick so the tick
	 * advances on a 50 ms cadence regardless of host traffic.
	 *
	 * 216 MHz / 20 Hz = 10 800 000 -> reload 10 799 999, inside the
	 * 24-bit 0x00FFFFFF limit (max period 77.6 ms at 216 MHz).  The
	 * reload is derived from the LIVE SystemCoreClock sampled above
	 * (#127), not a hardcoded 216 MHz literal -- on a broken clock
	 * tree the tick is still 50 ms of real time, just fewer counts.
	 * Reload is clamped to the 24-bit field so a bogusly huge
	 * SystemCoreClock cannot push LOAD out of range.
	 *
	 * NVIC priority 15 (lowest, __NVIC_PRIO_BITS = 4 on this part):
	 * the handler body is empty -- waking __WFI() is its entire job
	 * -- and it must never delay BRIDGE_CS_IRQ_PRIO 1 or
	 * BRIDGE_I2C_IRQ_PRIO 2. */
	{
		uint32_t reload = (SystemCoreClock / 20u) - 1u; /* 50 ms */
		if (reload > SysTick_LOAD_RELOAD_Msk) reload = SysTick_LOAD_RELOAD_Msk;
		(void)SysTick_Config(reload + 1u);
		NVIC_SetPriority(SysTick_IRQn, (1u << __NVIC_PRIO_BITS) - 1u); /* 15 = lowest */
	}
}

/* Periodic tick handler (gh#54): only counts periods (#315).  Its job is
 * to retire the main loop's __WFI() so bridge_hw_tick() runs.  The
 * watchdog is NEVER fed from here: a wedged main loop would otherwise
 * keep being fed by a healthy tick interrupt.  Strong definition
 * overrides the weak Default_Handler alias in the vendor startup. */
volatile uint32_t bridge_systick_count; /* 50 ms periods; read by the I2C stuck-SDA poll */

void SysTick_Handler(void)
{
	bridge_systick_count++;
}

/* Called at base level after every main-loop wake.  This backend uses the
 * hook to advance work deliberately kept out of the transport ISRs.  The
 * DA9292 fault nets still reach only the Renesas, so no PMIC sampling is
 * performed here and bridge_hw_da9292_status_cached() retains its 0xFF
 * "no sample" sentinel. */
/* Base-level DSP pump (#496): drains each bound FIR/IIR stream's raw
 * samples through the FAC into its processed ring.  Runs here, off the
 * main loop -- never in the CS-EXTI stream_read path. */
extern void bridge_hw_dsp_pump(void);
/* Background OTA slot-erase pump (#770): advances a BEGIN-armed erase one
 * page-region per tick so BEGIN never blocks the SPI reply inline.  No-op
 * in the OTA-inert build. */
extern void ota_erase_tick(void);
/* Deferred low-power entry (gh#63): executes a mode 2/3 request latched
 * by bridge_hw_power_mode_set() on a quiet link, at base level -- never
 * from a transport ISR.  Strong impl in hal/gd32/power.c. */
extern void bridge_power_tick(void);

/* BRD_I2C stuck-SDA detector (gh#39, erratum 2.3.1): polls the SDA pad
 * and runs the documented I2C software reset when the line is confirmed
 * stuck low.  Weak no-op on the stub backend (src/transport_i2c.c);
 * strong impl in hal/transport_hw_gd32.c. */
extern void bridge_transport_i2c_stuck_poll(void);

/* OTA trial/confirm pump (bench fact 2026-09-26, E1M-V2M103): once the
 * wire has noted a frame during an unconfirmed trial, commits the slot
 * permanent and reboots.  No-op in the OTA-inert build, and a no-op on
 * every tick outside an active trial. */
extern void ota_confirm_tick(void);

/* Free watchdog (FWDGT, gh#54).  Armed from bridge_hw_tick() on the first
 * healthy pass and fed only from there, never from SysTick_Handler().
 *
 * Why not in bridge_hw_init(): a TRIAL image (src/ota.c) runs under the
 * bootloader's ~32.8 s FWDGT (OTA_TRIAL_FWDGT_RELOAD, DIV256), which IS
 * the confirm deadline -- it reverts a slot the host never confirms.  The
 * counter cannot be disarmed, so re-arming it here with a 501 ms window
 * that the tick keeps feeding would silently cancel that revert.  While
 * ota_trial_unconfirmed() the tick therefore leaves the watchdog alone;
 * once the slot is confirmed the part reboots and arms here.
 *
 * Window: RLD 500, /32 -> (RLD+1) ticks of IRC32K/32 = 501 ms nominal
 * (UM Rev1.2 p.525 Table 21-1), 445..573 ms across the 28..36 kHz
 * IRC32K spread (Datasheet Rev2.0 p.125 Table 4-23).  Sized against the
 * longest legitimate stretch between two feeds, i.e. one transport ISR
 * plus one bridge_hw_tick() pass, built -Os (CMakeLists.txt):
 *   - OTA_VERIFY / rollback CRC of a full 0x36000 slot: table-driven
 *     crc32 <= 16 instr/byte * 4 cycles (deliberately generous) *
 *     221184 B / 216 MHz ~= 66 ms (172 ms if built -O0);
 *   - ota_erase_tick(): one 2 KB region = 20 ms single-bank, 40 ms
 *     dual-bank (tERASE max 20 ms/page, Datasheet p.126);
 *   - COMMIT: CRC + one metadata page erase/program ~ 66 + 21 ms;
 *   - ADC calibrate wedged: ~200000 iterations (adc.c) ~ 63 ms at
 *     314.8 ns/iter (hal/fmc_ota.c derivation), plus the ~400000
 *     iteration EOC bound ~ 126 ms;
 *   - OTA_FMC_ERASE_TIMEOUT_ITERS (2.2 M) on a WEDGED FMC: 204 ms -Os /
 *     693 ms -O0 -- that case is meant to end in a reset.
 * Worst legitimate sum ~ 190 ms ISR + 40 ms erase step + pumps, under
 * half the 445 ms minimum window.
 *
 * Warm resets: like an IWDG, a started FWDGT is only stopped by a
 * power-on reset (bring-up item 4 on gh#54 confirms this per unit), so
 * after NVIC_SystemReset, a fault-loop reset, ota_confirm_tick()'s reboot
 * or a host NRST the 501 ms dog may still be counting through the whole
 * boot path, not just this tick.  The budget before the first feed here:
 *   - bootloader (src/boot/boot_main.c): one full-slot CRC per candidate,
 *     up to 2 x 66 ms at -Os (2 x 172 ms at -O0).  The bootloader feeds
 *     before each candidate CRC, before the jump and in its no-valid-image
 *     park, so each stretch is one CRC (<= 172 ms) and the park stays a
 *     diagnosable stop instead of a ~0.5 s reset loop;
 *   - bridge_hw_init(): a healthy boot is a few ms; every bounded wait
 *     wedged at once (compensation cell 200000 iterations ~ 63 ms, VREF
 *     100000 ~ 31 ms, four ADC calibrations ~ 4 x 63 ms) is ~ 350 ms,
 *     which is inside the 445 ms minimum only because the bootloader fed
 *     immediately before the jump.  A hardware wedge that long is meant to
 *     end in a reset;
 *   - the first bridge_hw_tick() then arms/feeds within one 50 ms tick.
 * Deep-sleep/Standby entry has its own feed + spacing rule in
 * hal/gd32/power.c, and refuses a wake longer than the window whenever the
 * FWDGSPD_STDBY option bit lets the counter run through Standby.
 *
 * IRC32K is switched on first with a BOUNDED stabilisation spin (the
 * vendor rcu_osci_stab_wait() polls forever, and the FWDGT is not yet
 * running to rescue that hang).  fwdgt_config() bounds its own PSC/RLD
 * update waits; an open-coded 0xCCCC sequence would spin with the dog
 * already live on its 512 ms reset defaults.  A failed PSC/RLD update is
 * retried once and, if it still fails, latched in fwdgt_config_failed
 * (SWD-readable next to gpio_compensation_ready): the counter is already
 * running on its reset defaults (~0.5 s) by then and cannot be stopped.
 * DBG_CTL1_FWDGT_HOLD is already set in bridge_hw_init(), so a halted core
 * does not reset.
 *
 * Not decided here (bring-up items on gh#54): the FWDGSPD_DPSLP option bit
 * (FMC_OBCTL 17; Deep-sleep entry is refused today).  nFWDG_HW must stay 1
 * so the bootloader's no-valid-image park cannot become a hardware-watchdog
 * reset loop. */
#define BRIDGE_FWDGT_RELOAD 500u

/* True if the FWDGT PSC/RLD update failed twice: the dog is running on its
 * reset defaults, not the 501 ms window.  Volatile so a bench probe can
 * read it (same shape as gpio_compensation_ready); not on the wire. */
volatile bool fwdgt_config_failed;

static void bridge_fwdgt_arm(void)
{
	rcu_osci_on(RCU_IRC32K);
	uint32_t to = 200000u;
	while (--to && RESET == rcu_flag_get(RCU_FLAG_IRC32KSTB)) {
		/* spin: typical < 50 us */
	}
	if (fwdgt_config(BRIDGE_FWDGT_RELOAD, FWDGT_PSC_DIV32) != SUCCESS &&
	    fwdgt_config(BRIDGE_FWDGT_RELOAD, FWDGT_PSC_DIV32) != SUCCESS) {
		fwdgt_config_failed = true;
	}
	fwdgt_enable();
}

/* OTA trial gate (src/ota.c): true while this boot runs an unconfirmed
 * TRIAL image under the bootloader's watchdog. */
extern bool ota_trial_unconfirmed(void);

void bridge_hw_tick(void)
{
	bridge_hw_dsp_pump();
	ota_erase_tick();
	ota_confirm_tick();
	bridge_transport_i2c_stuck_poll();
	bridge_power_tick();
	/* Feed the free watchdog LAST (gh#54), and only from this healthy
	 * base-level pass: a step above that wedges never reaches this line,
	 * the part resets ~0.5 s later and the host sees a reboot instead of
	 * a silent wedge.  See bridge_fwdgt_arm() for the window sizing and
	 * the TRIAL exception. */
	if (ota_trial_unconfirmed()) return;
	static bool fwdgt_armed;
	if (!fwdgt_armed) {
		bridge_fwdgt_arm();
		fwdgt_armed = true;
	} else {
		fwdgt_counter_reload();
	}
}

/* ----------------------------------------------------------------- */
/* Stub bodies -- shape mirrors bridge_hw_stub.c so the gd32 backend  */
/* is binary-compatible with the stub backend pending real impls.    */
/* ----------------------------------------------------------------- */

uint8_t bridge_hw_reset_reason(void)
{
	/* Snapshot taken in bridge_hw_init(); reading it never clears anything. */
	return bridge_reset_reason_decode(s_reset_reason_snapshot);
}

uint8_t bridge_hw_da9292_status_cached(void)
{
	/* 0xFF sentinel unconditionally: no DA9292 net reaches the GD32 on
     * this SoM rev (fault pins are Renesas P37/P36 inputs). */
	return 0xFFu;
}
