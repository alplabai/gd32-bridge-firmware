/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge firmware -- entry point.
 *
 * The GD32G553 supervisor runs this firmware as its sole user
 * application (no RTOS).  The runtime model is purely
 * interrupt-driven: SPI slave + I2C slave each have their own ISR
 * that hands complete request envelopes off to protocol_dispatch()
 * (in protocol.c) and stages the matching reply envelope back to
 * the transport.
 *
 * The main loop has no work of its own; it sleeps in WFI() so the
 * Cortex-M33 idles between interrupts.  bridge_hw_tick() runs on the
 * wakeups as a periodic housekeeping hook; it is a no-op on this SoM
 * revision -- the DA9292 fault nets (DA9292_INT/DA9292_TW) reach only
 * the Renesas (P37/P36), the GD32 has no pin to sample and no I2C
 * path to the PMIC, so CMD_DA9292_STATUS_FORWARD serves the 0xFF
 * "no sample" sentinel.  Register-level PMIC status (PMC_STATUS_00
 * etc.) is read by the host over BRD_I2C via the chips/da9292 driver.
 *
 * Backends: BRIDGE_HAL_BACKEND=gd32 drives real silicon (peripheral
 * HAL in the per-peripheral TUs under hal/gd32/, SPI1 + I2C0 slave
 * transports in hal/transport_hw_gd32.c).  BRIDGE_HAL_BACKEND=stub keeps everything
 * hardware-free for host-side protocol tests: PING / GET_VERSION /
 * GET_BUILD_ID / RESET_REASON round-trip and HW-touching ops return
 * NOSUPPORT.
 */

#include <stdbool.h>
#include <stdint.h>

#include "ota.h"
#include "protocol.h"
#include "timing_stats.h"
#include "transport.h"

/* Optional weak hooks the HAL layer can override.  Defaults to a
 * busy WFI loop -- behaviour-equivalent to a no-op for the
 * scaffold; the real HAL overrides these for peripheral bring-up
 * and periodic housekeeping. */
__attribute__((weak)) void bridge_hw_init(void)
{
}
__attribute__((weak)) void bridge_hw_tick(void)
{
}

/* The Cortex-M intrinsic.  This TU deliberately does not include
 * cmsis_gcc.h (CMSIS defines __WFI() as a macro, not a linkable
 * symbol -- `#ifndef __WFI` in a TU that never saw that header is
 * always true, so a guard keyed on the macro's definedness silently
 * always wins and every build, including the real hardware build,
 * got the empty stub below instead of a real `wfi`).  Key the guard
 * on the actual compile target instead: on Arm, emit the instruction
 * directly; the empty stub is kept only for a genuinely hosted
 * (non-Arm) build of this TU, which no target in this tree currently
 * performs. */
#if defined(__arm__)
static inline void __WFI(void)
{
	__asm volatile("wfi" ::: "memory");
}
/* PRIMASK only gates taking an interrupt; WFI still wakes on a pending one. */
static inline void main_irq_off(void)
{
	__asm volatile("cpsid i" ::: "memory");
}
static inline void main_irq_on(void)
{
	__asm volatile("cpsie i" ::: "memory");
}
#else
__attribute__((weak)) void __WFI(void)
{
}
static inline void main_irq_off(void)
{
}
static inline void main_irq_on(void)
{
}
#endif

/* True while base-level work an ISR queued (or a multi-step walk) still needs
 * bridge_hw_tick() before the core may sleep.  Weak default: none. */
__attribute__((weak)) bool bridge_hw_work_pending(void)
{
	return false;
}

int main(void)
{
	bridge_hw_init();
	TS_ONLY(timing_stats_init();)
	/* Reconcile the A/B metadata against the slot we are running BEFORE any
	 * frame can arrive: decides whether this is an unconfirmed TRIAL boot
	 * (wire gated BUSY, ota_confirm_tick() waits for a frame) and self-heals
	 * a rejected candidate.  Needs the FMC HAL from bridge_hw_init(). */
	ota_boot_init();
	transport_spi_init();
	transport_i2c_init();

	/* Housekeeping lives in bridge_hw_tick().  The loop sleeps in WFI until an
	 * interrupt (transport or SysTick) and then ticks; it skips the sleep only
	 * while bridge_hw_work_pending() says an ISR-queued job still needs a tick. */
	for (;;) {
		/* Check-then-sleep must be atomic against the transport ISRs: an ISR
		 * that queues work after the last bridge_hw_tick() passed its check
		 * but before WFI would otherwise sleep with the work pending until
		 * the next unrelated interrupt (a queued CMD_BOOT_CONFIG SET was only
		 * committed by the next bus transaction).  With IRQs masked, an ISR
		 * arriving after the check stays pending and WFI returns at once. */
		main_irq_off();
		if (!bridge_hw_work_pending()) {
			__WFI();
		}
		main_irq_on();
		bridge_hw_tick();
	}
	/* unreachable */
	return 0;
}
