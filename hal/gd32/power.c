/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 bridge HAL backend -- system power modes.
 * Split move-only from hal/bridge_hw_gd32.c (fw v0.2.8); see
 * hal/gd32/init.c for the backend-wide implementation notes.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bridge_hw.h"
#include "gd32g5x3.h"

#include "bridge_board_config.h" /* BRIDGE_I2C_PERIPH */
#include "gd32_common.h"
#include "transport.h" /* bridge_transport_i2c_hw_init() */

/* ----------------------------------------------------------------- */
/* v0.5 (§2B.3) -- system power-mode set                             */
/* ----------------------------------------------------------------- */

/* ALP_POWER_WAKE_* bits the firmware supports (mirrors the wire
 * encoding in <alp/power.h>).
 *
 * Mapping notes per V2N hardware reality:
 *
 *   - GPIO : routes through PMU_WAKEUP_PIN0..4 -- five fixed pads
 *            on the GD32G553 + board wires the desired triggers
 *            onto them.  Landed §C.15c.
 *   - RTC  : RTC alarm 0 fires on a scheduled wallclock; the
 *            wakeup timer also surfaces under this bit so the
 *            firmware uses the timer (simpler than absolute-time
 *            alarms in a partial bring-up).  Landed §C.25.
 *   - TIMER: any non-zero `wake_after_ms` -- same RTC wakeup-timer
 *            path as RTC.  The bit is redundant when wake_after_ms
 *            > 0 (the timer wakes the chip implicitly per the
 *            <alp/power.h> contract); honouring the bit explicitly
 *            lets a caller arm a "wake on next tick" without a
 *            specific deadline.  Landed §C.25.
 *   - UART_RX / USB / ETH_LINK : no hardware path on the GD32G5
 *            (no LPUART wake / no USB OTG / no MAC).  Future SoCs
 *            on the bridge slot could populate these via the same
 *            opcode; today the firmware rejects them so the host
 *            knows the request is moot.
 */
#define POWER_WAKE_RTC            0x00000001u
#define POWER_WAKE_GPIO           0x00000002u
#define POWER_WAKE_UART_RX        0x00000004u
#define POWER_WAKE_TIMER          0x00000008u
#define POWER_WAKE_USB            0x00000010u
#define POWER_WAKE_ETH_LINK       0x00000020u
#define POWER_WAKE_MASK_SUPPORTED (POWER_WAKE_RTC | POWER_WAKE_GPIO | POWER_WAKE_TIMER)
#define POWER_WAKE_MASK_HW_GATED  (POWER_WAKE_UART_RX | POWER_WAKE_USB | POWER_WAKE_ETH_LINK)

/* RTC wakeup timer LSB.  POWER_WAKE_LSB_HZ = 2000 Hz assumes an exact
 * 32000 Hz IRC32K with the /16 divider -- 0.5 ms/tick nominal, max
 * wake 65535/2000 = 32.7 s -- but IRC32K is NOT a fixed 32000 Hz.
 * LXTAL is unavailable on this SoM (PC14/PC15 are spent as host
 * E1M IO24/IO25) so IRC32K is the only legal RTC source, and it is
 * specified only as 28-36 kHz over this grade-7 part's -40..105 degC
 * range (GD32G553xx Datasheet Rev2.0 p.125 Table 4-23; full citation
 * + why LXTAL is unavailable is in hal/bridge_board_config.h).  The
 * real tick is therefore 444 us (36 kHz corner) to 571 us (28 kHz
 * corner) against the 500 us this constant assumes -- about
 * -11.1% / +14.3% on any requested wake_after_ms, and the 32.7 s
 * ceiling can land anywhere from ~29.1 s to ~37.4 s.  No firmware
 * change closes this: IRC32K has no trim register in the RCU, and
 * the RTC's own RTC_HRFC digital calibration corrects a fixed
 * offset against a reference clock, not IRC32K's drift over
 * temperature (see hal/bridge_board_config.h).  Longer waits than
 * the ceiling would need the CKSPRE_2EXP16 mode, which sits in a
 * future commit. */
#define POWER_WAKE_LSB_HZ       2000u
#define POWER_WAKE_TIMER_MAX_MS (65535u * 1000u / POWER_WAKE_LSB_HZ)

/* One-time RTC + LSI bring-up that arms the wakeup timer.  Idempotent
 * across multiple power_mode_set calls -- the LSI stays enabled, the
 * RTC source latches to IRC32K once.  Failure (LSI never stabilises,
 * write-protected register won't unlock) leaves rtc_wakeup_ready
 * false and bridge_hw_power_mode_set returns NOSUPPORT for any
 * timer-bearing call. */
static bool rtc_wakeup_ready = false;

static bool rtc_wakeup_init_once(void)
{
	if (rtc_wakeup_ready) return true;

	/* Bring up IRC32K (internal LSI) as the RTC clock source. */
	rcu_osci_on(RCU_IRC32K);
	/* Spin until IRC32K stabilises -- typical < 50 us, the upper
     * bound keeps a dead oscillator from hanging the bridge. */
	uint32_t to = 200000u;
	while (--to && RESET == rcu_flag_get(RCU_FLAG_IRC32KSTB)) {
		/* spin */
	}
	if (to == 0u) return false;

	rcu_periph_clock_enable(RCU_PMU);
	pmu_backup_write_enable();
	rcu_rtc_clock_config(RCU_RTCSRC_IRC32K);
	rcu_periph_clock_enable(RCU_RTC);

	rtc_wakeup_ready = true;
	return true;
}

static int rtc_wakeup_arm_ms(uint32_t wake_after_ms)
{
	if (!rtc_wakeup_init_once()) return BRIDGE_HW_ERR_IO;
	if (wake_after_ms > POWER_WAKE_TIMER_MAX_MS) return BRIDGE_HW_ERR_RANGE;

	/* Compute ticks (round up so a sub-LSB request still waits at
     * least one tick rather than zero). */
	uint32_t ticks = (wake_after_ms * POWER_WAKE_LSB_HZ + 999u) / 1000u;
	if (ticks == 0u) ticks = 1u;
	if (ticks > 65535u) ticks = 65535u;

	/* The vendor sequence: disable the wakeup timer, switch its
     * clock source, set the counter, re-enable.  rtc_wakeup_disable
     * may return ERROR if the WTWF flag never sets; treat as IO. */
	if (SUCCESS != rtc_wakeup_disable()) return BRIDGE_HW_ERR_IO;
	if (SUCCESS != rtc_wakeup_clock_set(WAKEUP_RTCCK_DIV16)) return BRIDGE_HW_ERR_IO;
	if (SUCCESS != rtc_wakeup_timer_set((uint16_t)(ticks - 1u))) return BRIDGE_HW_ERR_IO;
	rtc_wakeup_enable();
	return BRIDGE_HW_OK;
}

static void power_wake_pins_enable(uint32_t wake_bitmap)
{
	if ((wake_bitmap & POWER_WAKE_GPIO) == 0u) return;
	pmu_wakeup_pin_enable(PMU_WAKEUP_PIN0);
	pmu_wakeup_pin_enable(PMU_WAKEUP_PIN1);
	pmu_wakeup_pin_enable(PMU_WAKEUP_PIN2);
	pmu_wakeup_pin_enable(PMU_WAKEUP_PIN3);
	pmu_wakeup_pin_enable(PMU_WAKEUP_PIN4);
}

/* Deferred low-power request latched by bridge_hw_power_mode_set() in
 * a transport ISR and consumed by bridge_power_tick() at base level
 * (gh#63).  Non-zero `s_lp_pending_mode` means an accepted request is
 * waiting for a quiet-link window; wake sources were already armed at
 * latch time (register writes, ISR-safe). */
static volatile uint8_t s_lp_pending_mode;

/* Watchdog-aware low-power entry gate: feed the FWDGT and honour UM
 * Rev1.2 p.525's spacing rule ("more than 3 IRC32K clock intervals
 * must be inserted in the middle of reload and deepsleep/standby mode
 * commands") before the entry wfi.  3 IRC32K intervals at the 28 kHz
 * minimum (Datasheet Rev2.0 p.125) is 107 us; the spin below is
 * ~150-250 us at 216 MHz.  Mirrors the same-named helper on the gh#54
 * branch (codex/periodic-tick-and-fwdgt) -- identical by design, take
 * either copy when the two branches merge. */
static void power_fwdgt_settle_before_lp_entry(void)
{
	fwdgt_counter_reload();
	while (0u != (FWDGT_STAT & (FWDGT_STAT_PUD | FWDGT_STAT_RUD | FWDGT_STAT_WUD))) {
		/* bounded by construction: the only pending writes are
		 * bridge_hw_init()'s long-completed fwdgt_config() */
	}
	for (volatile uint32_t gap = 0u; gap < 8000u; ++gap) {
		/* >= 3 IRC32K intervals at 28 kHz min = 107 us; ~5 cycles
		 * per iteration at 216 MHz -> ~185 us */
	}
}

int bridge_hw_power_mode_set(uint8_t mode, uint32_t wake_bitmap, uint32_t wake_after_ms)
{
	/* Mode 0 (run) + mode 1 (sleep) are accepted no-ops -- main()'s
	 * `for (;;) { __WFI(); bridge_hw_tick(); }` already runs the CPU
	 * in WFI between transport interrupts, which IS "sleep" on the
	 * GD32G5.  Mode 2 (deep-sleep) + mode 3 (standby) are LATCHED
	 * here and executed from bridge_power_tick() at base level (gh#63)
	 * -- never entered from this call, because this call runs INSIDE a
	 * transport ISR (CS-EXTI prio 1 / I2C-EV prio 2) and:
	 *
	 *   - an ARMv8-M WFI only ends on an exception that can preempt
	 *     the current execution priority, so a deep-sleep entered
	 *     from the CS handler can never be ended by the CS line --
	 *     the wake source that is supposed to end it;
	 *   - UM Rev1.2 p.143: with any EXTI_PD bit set, "the program
	 *     will skip the entry process of Deep-sleep mode to continue
	 *     execute the following procedure" -- and EXTI_PD0/1 have
	 *     UNDEFINED reset values (UM p.216/p.219), so the skip was
	 *     the COMMON case, returning STATUS_OK for a sleep that
	 *     never happened;
	 *   - entering Standby from interrupt context leaves the calling
	 *     transport's state machine mid-transaction with no way to
	 *     unwind it if the entry aborts.
	 *
	 * The reply is staged and drained normally; the actual entry
	 * happens on the next bridge_power_tick(), on a quiet link.
	 *
	 * Wake-source semantics: `wake_bitmap` enumerates the explicit
	 * sources the host wants armed; `wake_after_ms` is a timed
	 * fallback that arms the RTC wakeup timer regardless of the
	 * bitmap (per the <alp/power.h> contract: the timer is implicit
	 * when wake_after_ms > 0).  Unsupported bits (UART_RX / USB /
	 * ETH_LINK on the GD32G5 baseline) reject so the host knows the
	 * request was not honoured. */
	if ((wake_bitmap & POWER_WAKE_MASK_HW_GATED) != 0u) return BRIDGE_HW_ERR_NOTIMPL;

	switch (mode) {
	case 0u: /* run -- no-op */
	case 1u: /* sleep -- already in WFI between transport ISRs */
		return BRIDGE_HW_OK;
	case 2u: /* deep-sleep */
		/* Deep-sleep can only be left through an EXTI line (UM
		 * Rev1.2 p.142), and this bridge's I2C slave has NO wake
		 * path: WUEN is never set, the I2C kernel clock is APB1
		 * not IRC8M (p.1279: without IRC8M, "IRC8M will not [be]
		 * switched on after receiving start signal"), and EXTI
		 * line 31 (I2C0 wakeup, p.213 Table 5-3) is never enabled.
		 * Disabling I2C0 across the entry (gh#41's UM p.1279
		 * requirement) therefore means a sleeping bridge can never
		 * be woken over BRD_I2C -- so while that link is up, mode 2
		 * is refused outright: the honest option gh#63 names.
		 * "While the link is up" is unconditionally true on this
		 * bridge today (bridge_transport_i2c_hw_init() runs at
		 * boot unconditionally), so mode 2 answers NOSUPPORT
		 * until the I2C-wake branch lands (BRIDGE_I2C_CK_SRC ->
		 * IRC8M + WUEN re-arm per entry + EXTI line 31 -- a
		 * bench-validated change of its own, NOT this PR).
		 * STATUS_NOSUPPORT (0x06) is the wire answer the host
		 * already handles for other unsupported wake sources. */
		return BRIDGE_HW_ERR_NOTIMPL;
	case 3u: /* standby */
		rcu_periph_clock_enable(RCU_PMU);
		/* Gate the entry: Standby's wake set is exactly five
		 * sources (UM Rev1.2 p.142 Table 3-1: "1. NRST pin 2. WKUP
		 * pins 3. FWDGT reset 4. RTC 5. LCKMD").  A request that
		 * arms neither a WKUP pin nor the RTC wakeup timer enters
		 * a mode only NRST can leave -- refuse before anything is
		 * latched (gh#40 fix 1). */
		{
			const bool wake_pins_armed = (wake_bitmap & POWER_WAKE_GPIO) != 0u;
			const bool rtc_armed =
			    wake_after_ms != 0u || (wake_bitmap & (POWER_WAKE_RTC | POWER_WAKE_TIMER)) != 0u;
			if (!wake_pins_armed && !rtc_armed) return BRIDGE_HW_ERR_INVAL;
		}
		power_wake_pins_enable(wake_bitmap);
		if (wake_after_ms != 0u || (wake_bitmap & (POWER_WAKE_RTC | POWER_WAKE_TIMER)) != 0u) {
			const uint32_t ms = (wake_after_ms != 0u) ? wake_after_ms : POWER_WAKE_TIMER_MAX_MS;
			int            rc = rtc_wakeup_arm_ms(ms);
			if (rc != BRIDGE_HW_OK) return rc;
		}
		/* Latch the request: the STATUS_OK reply drains normally,
		 * and bridge_power_tick() below performs the entry on a
		 * quiet link.  Wake sources were armed above (pure register
		 * writes, ISR-safe); the bitmap itself needs no deferral
		 * because nothing at tick time re-reads it.  Standby exits
		 * through a power-on reset, so the host's "rebooting, then
		 * re-init and probe" contract applies unchanged. */
		s_lp_pending_mode = 3u;
		return BRIDGE_HW_OK;
	default:
		return BRIDGE_HW_ERR_INVAL;
	}
	(void)POWER_WAKE_MASK_SUPPORTED;
}

/* Base-level low-power entry (gh#63): the deferred half of
 * bridge_hw_power_mode_set().  Runs from bridge_hw_tick() -- base level,
 * AFTER the accepted request's reply has drained onto the wire, with
 * no transport state machine mid-transaction.
 *
 * Steps 1-4 below NARROW the entry race but cannot close it -- a CS
 * falling edge landing between the gate and the wfi still defeats the
 * entry -- which is why every gate failure below re-latches the request
 * and retries on the NEXT tick rather than reporting a success that
 * did not happen or silently dropping the request. */
void bridge_power_tick(void)
{
	if (s_lp_pending_mode == 0u) return;

	/* 1. Only sleep on a quiet link: CS de-asserted (PA8 high).  An
	 *    asserted CS means a transaction is in flight (or about to
	 *    be) -- retry later. */
	if (SET != gpio_input_bit_get(BRIDGE_SPI_NSS_PORT, BRIDGE_SPI_NSS_PIN)) return;

	/* 2. Clear every EXTI pending bit (UM Rev1.2 p.143: "In order to
	 *    enter Deep-sleep mode smoothly, all EXTI line pending status
	 *    (in the EXTI_PD register) and related peripheral flags must
	 *    be reset. If not, the program will skip the entry process").
	 *    EXTI_PD0 at offset 0x14, bits 31:0 rc_w1; EXTI_PD1 at offset
	 *    0x2C, bits 6:0 rc_w1, bits 31:7 reserved (UM p.216-217,
	 *    p.219).  Today only line 8 is enabled so the collateral is
	 *    nil, but any future EXTI consumer (RTC wakeup line 19,
	 *    I2C0 line 31, LPTIMER line 35) must be re-examined against
	 *    this clear. */
	EXTI_PD0 = 0xFFFFFFFFu;
	EXTI_PD1 = 0x0000007Fu;
	__DSB();

	/* 3. Re-read and abort the attempt rather than reporting a sleep
	 *    that did not happen: a re-pended bit means an edge arrived
	 *    during the clear -- a transaction is starting. */
	if ((EXTI_PD0 != 0u) || ((EXTI_PD1 & 0x0000007Fu) != 0u)) return;

	/* 4. A WFI with an enabled interrupt already pending in the NVIC
	 *    is architecturally a no-op, so the pending set must be clear
	 *    too (gh#63 step 4). */
	if (NVIC_GetPendingIRQ(BRIDGE_SPI_CS_EXTI_IRQN) != 0u) return;
	if (NVIC_GetPendingIRQ(BRIDGE_I2C_EV_IRQN) != 0u) return;

	/* 5. Only now enter. */
	const uint8_t mode = s_lp_pending_mode;

	if (mode == 3u) {
		/* Watchdog-aware entry (gh#54): feed + >= 3 IRC32K
		 * intervals of gap before the wfi (UM p.525). */
		power_fwdgt_settle_before_lp_entry();
		pmu_to_standbymode();

		/* Reaching this line means the standby entry ABORTED: the
		 * part did not power down.  The vendor helper has already
		 * masked the NVIC down to IRQ 3 and IRQ 41 (ICER0 =
		 * 0xFFFFFFF7 / ICER1 = 0xFFFFFDFF / ICER2 = 0xFFFFFFFF),
		 * which disables all three transport vectors, and left
		 * SLEEPDEEP set in SCB->SCR.  Undo the damage so the
		 * bridge keeps serving its host instead of going
		 * permanently deaf with a deep-sleeping main loop; the
		 * failed request is dropped (the reply already said OK,
		 * but standby's exit is a reset in the intended case, so
		 * the host's re-init probe is the recovery either way).
		 * Priorities come from the same bridge_board_config.h
		 * macros the arming sites use, so the pair cannot drift
		 * (gh#40 fix 2). */
		nvic_irq_enable(BRIDGE_SPI_CS_EXTI_IRQN, BRIDGE_CS_IRQ_PRIO, BRIDGE_CS_IRQ_SUBPRIO);
		nvic_irq_enable(BRIDGE_I2C_EV_IRQN, BRIDGE_I2C_IRQ_PRIO, BRIDGE_I2C_IRQ_SUBPRIO);
		nvic_irq_enable(BRIDGE_I2C_ER_IRQN, BRIDGE_I2C_IRQ_PRIO, BRIDGE_I2C_IRQ_SUBPRIO);
		SCB->SCR &= ~SCB_SCR_SLEEPDEEP_Msk;
		s_lp_pending_mode = 0u; /* drop the failed request */
	}
	/* NOTE (future I2C-wake branch): the mode-2 entry lives here --
	 * quiet-link gate + EXTI_PD clear above, then i2c_disable(),
	 * pmu_to_deepsleepmode(PMU_LDO_LOWPOWER, WFI_CMD), and
	 * bridge_transport_i2c_hw_init() on the wake path (its live-clock
	 * timing derivation handles the post-wake IRC8M, gh#41).  It is
	 * not reachable until bridge_hw_power_mode_set() stops rejecting
	 * mode 2 -- see the mode-2 comment there. */
}
