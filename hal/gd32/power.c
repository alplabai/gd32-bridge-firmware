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
#include "bridge_critical.h"
#include "gd32_common.h"
#include "ota.h" /* ota_trial_unconfirmed() */
#include "power_policy.h"
#include "power_wake.h"
#include "transport.h" /* bridge_transport_i2c_hw_init() */

/* ----------------------------------------------------------------- */
/* v0.5 (§2B.3) -- system power-mode set                             */
/* ----------------------------------------------------------------- */

/* ALP_POWER_WAKE_* bits the firmware supports (mirrors the wire
 * encoding in <alp/power.h>).
 *
 * Mapping notes per V2N hardware reality:
 *
 *   - GPIO : no usable pad on this SoM, so the firmware rejects it
 *            (BRIDGE_HW_ERR_NOTIMPL).  The GD32G553 PMU wake pads are
 *            WKUP0=PA0 (encoder), WKUP1=PC13 (SE_RST), WKUP3=PA2 (ADC)
 *            and WKUP4=PC5 (PWM3), all claimed by live bridge functions;
 *            WKUP2=PE6 has no ball on the WLCSP81 package (#20).
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

	bridge_rcu_periph_clock_enable(RCU_PMU);
	pmu_backup_write_enable();
	/* gh#257: rcu_rtc_clock_config() is a bare RCU_BDCTL read-modify-
     * write, same exposure class as the RCU_*EN writes
     * bridge_rcu_periph_clock_enable() already protects -- and this
     * path runs at runtime (first armed power-mode-set call), with
     * interrupts live, not just at boot. */
	const uint32_t rtcsrc_primask_ = bridge_irq_lock();
	rcu_rtc_clock_config(RCU_RTCSRC_IRC32K);
	bridge_irq_unlock(rtcsrc_primask_);
	bridge_rcu_periph_clock_enable(RCU_RTC);

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
	 * may return ERROR if the WTWF flag never sets; treat as IO.
	 *
	 * UM Rev1.2 p.548 s22.3.18 prescribes THREE steps to make an RTC
	 * wakeup event reach the core -- "1. Configure and enable the
	 * corresponding interrupt line ... of EXTI and set the rising edge
	 * for triggering. 2. Configure and enable the RTC ... auto wakeup
	 * interrupt. 3. Configure and enable the RTC ... auto wakeup
	 * function."  rtc_wakeup_enable() below is step 3 ONLY; before
	 * gh#53 neither step 1 (EXTI line 19: "RTC wakeup timer", UM
	 * p.213 Table 5-3) nor step 2 (RTC_CTL.WTIE, bit 14, UM p.551)
	 * existed anywhere in the tree, so the timer counted down, set
	 * RTC_STAT.WTF ... and nothing reached the NVIC: a mode-2 entry
	 * armed with wake_after_ms never came back and both transport
	 * links died with it (no watchdog either, gh#54).
	 *
	 * Clear WTF BEFORE the timer is enabled: it is set by hardware
	 * on expiry and cleared only by software writing 0 (UM p.554
	 * s22.4.4 bit 10), RTC_STAT is NOT reset by a system reset
	 * ("Only INITM, INITF and RSYNF bits are set to 0. Others are
	 * not affected", UM p.553), so a WTF left by one firmware run
	 * survives every NRST into the next -- arming without clearing
	 * is not idempotent across resets.  The clear must also precede
	 * rtc_wakeup_enable() by at least 1.5 RTC clock periods (~47 us
	 * at 32 kHz, UM p.554) relative to the NEXT WTF set; the timer
	 * counts a whole wake_after_ms before that, so ordering here is
	 * free. */
	if (SUCCESS != rtc_wakeup_disable()) return BRIDGE_HW_ERR_IO;
	if (SUCCESS != rtc_wakeup_clock_set(WAKEUP_RTCCK_DIV16)) return BRIDGE_HW_ERR_IO;
	if (SUCCESS != rtc_wakeup_timer_set((uint16_t)(ticks - 1u))) return BRIDGE_HW_ERR_IO;

	/* Step 2: RTC_CTL.WTIE (UM p.551, bit 14; the vendor unlock/lock
	 * around the write is inside rtc_interrupt_enable()).  Table 22-3
	 * (UM p.548) carries the footnote "Only active when RTC clock
	 * source is LXTAL or IRC32K" -- satisfied by the
	 * rcu_rtc_clock_config(RCU_RTCSRC_IRC32K) above. */
	rtc_flag_clear(RTC_FLAG_WT);          /* p.554: WTF cleared by software */
	rtc_interrupt_enable(RTC_INT_WAKEUP); /* RTC_CTL.WTIE <- 1 */

	/* Step 1: EXTI line 19 = RTC wakeup timer (UM p.213 Table 5-3),
	 * rising edge (UM p.548 s22.3.18 step 1), IRQ 3 (UM p.208
	 * Table 5-2).  exti_init() is a per-bit RMW of INTEN0/RTEN0/FTEN0,
	 * so this cannot disturb the CS line 8 configuration owned by
	 * hal/transport_hw_gd32.c. */
	exti_flag_clear(EXTI_19);
	exti_init(EXTI_19, EXTI_INTERRUPT, EXTI_TRIG_RISING);

	/* Priority 0 is exactly the line the vendor pmu_to_standbymode()
	 * leaves unmasked (its ICER0 write preserves bit 3), so the same NVIC
	 * enable serves mode 3's RTC exit and mode 2's timed wake. */
	nvic_irq_enable(RTC_WKUP_IRQn, 0U, 0U);

	rtc_wakeup_enable();
	return BRIDGE_HW_OK;
}

/* RTC wakeup-timer ISR (IRQ 3).  A strong definition overrides the
 * weak Default_Handler alias in the vendor startup
 * (startup_gd32g5x3.S:302-303) -- without THIS handler, enabling
 * WTIE + EXTI 19 would convert a non-waking bridge into a hard-hung
 * one, because the vector resolves to Default_Handler's
 * `b Infinite_Loop` at priority 0 (gh#53's own warning: land the
 * handler in the same change, not a follow-up).
 *
 * Ordering: clear the EXTI pending bit FIRST, then the RTC flag.
 * WTF must be cleared before the timer's next expiry and is safe to
 * touch here -- the 1.5-RTC-clock spacing rule (UM p.554) is about
 * the gap between the clear and the NEXT WTF set, which is a whole
 * wake_after_ms away.  Do NOT also clear WTF from base level in the
 * same window; rtc_wakeup_arm_ms() clears it once, at arm time.  (The
 * deep-sleep wake path in bridge_power_tick() does clear it, but only
 * AFTER rtc_wakeup_disable() has stopped the timer, so no new WTF can
 * arrive inside that spacing window.) */
void RTC_WKUP_IRQHandler(void)
{
	exti_interrupt_flag_clear(EXTI_19);
	rtc_flag_clear(RTC_FLAG_WT);
}

/* Deferred low-power request latched by bridge_hw_power_mode_set() in
 * a transport ISR and consumed by bridge_power_tick() at base level
 * (gh#63).  Non-zero `s_lp_pending_mode` means an accepted request is
 * waiting for a quiet-link window; wake sources were already armed at
 * latch time (register writes, ISR-safe). */
static volatile uint8_t s_lp_pending_mode;

/* POWER_FLAG_* of the latched Deep-sleep request. */
static volatile uint8_t s_lp_flags;

/* The latched request armed the RTC timer (a bounded sleep). */
static volatile bool s_lp_timed;

/* SWD-readable low-power counters (see power_policy.h). */
volatile bridge_power_diag_t bridge_power_diag;

/* Everything the Deep-sleep / Standby gate looks at, sampled now. */
static power_activity_t power_activity_get(void)
{
	const power_activity_t a = {
		.adc_stream  = bridge_adc_streams_active(),
		.pwm         = bridge_pwm_claims_active(),
		.dac         = bridge_dac_driven(),
		.ota         = ota_session_active(),
		.boot_commit = ota_trial_unconfirmed(),
	};
	return a;
}

static bool power_entry_refused(uint8_t mode)
{
	const power_activity_t a = power_activity_get();
	return power_entry_blocked(mode, &a);
}

/* Drop a latched request and stop the RTC timer armed for it. */
static void power_cancel_pending(void)
{
	s_lp_pending_mode = 0u;
	s_lp_flags        = 0u;
	s_lp_timed        = false;
	if (rtc_wakeup_ready) {
		(void)rtc_wakeup_disable();
		rtc_flag_clear(RTC_FLAG_WT);
		exti_flag_clear(EXTI_19);
	}
}

/* Longest low-power wake this firmware accepts while the mode's FWDGSPD bit
 * (FMC_OBCTL bit 18 standby, bit 17 deep-sleep) is 1, i.e. while the armed
 * FWDGT keeps counting through the mode.
 * The window is 445 ms minimum (hal/gd32/init.c); the requested time can
 * run +14.3% long on a slow IRC32K (see POWER_WAKE_LSB_HZ), so 300 ms ->
 * <= 343 ms actual, plus the ~0.2 ms settle gap, stays inside it.  With the
 * bit at 0 the counter is frozen in that mode and any wake time is safe.  The
 * stock option-byte value is not printed in UM Rev1.2, so the bit is read
 * per request instead of assumed. */
#define POWER_LP_FWDGT_MAX_MS 300u

/* Watchdog-aware low-power entry gate: feed the FWDGT and honour UM
 * Rev1.2 p.525's spacing rule ("more than 3 IRC32K clock intervals
 * must be inserted in the middle of reload and deepsleep/standby mode
 * commands") before the entry wfi.  3 IRC32K intervals at the 28 kHz
 * minimum (Datasheet Rev2.0 p.125) is 107 us; the spin below is
 * ~150-250 us at 216 MHz.
 *
 * The feed is skipped while ota_trial_unconfirmed(): the running counter
 * is then the bootloader's ~32.8 s revert dog, and feeding it from an
 * aborted standby request would push the confirm deadline back.
 *
 * Whether the armed FWDGT keeps COUNTING through Standby / Deep-sleep is
 * decided by FWDGSPD_STDBY / FWDGSPD_DPSLP; bridge_hw_power_mode_set() refuses a wake longer than
 * POWER_LP_FWDGT_MAX_MS when it does. */
static void power_fwdgt_settle_before_lp_entry(void)
{
	if (!ota_trial_unconfirmed()) {
		fwdgt_counter_reload();
	}
	while (0u != (FWDGT_STAT & (FWDGT_STAT_PUD | FWDGT_STAT_RUD | FWDGT_STAT_WUD))) {
		/* bounded by construction: the only pending writes are
		 * bridge_hw_init()'s long-completed fwdgt_config() */
	}
	for (volatile uint32_t gap = 0u; gap < 8000u; ++gap) {
		/* >= 3 IRC32K intervals at 28 kHz min = 107 us; ~5 cycles
		 * per iteration at 216 MHz -> ~185 us */
	}
}

int bridge_hw_power_mode_set(uint8_t  mode,
                             uint32_t wake_bitmap,
                             uint32_t wake_after_ms,
                             uint8_t  flags)
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
	 * when wake_after_ms > 0).  Any bit outside the supported set rejects,
	 * including future bits this firmware does not know, so the host is never
	 * told that an unarmed source will wake the part. */
	if (!power_wake_bitmap_supported(wake_bitmap)) return BRIDGE_HW_ERR_NOTIMPL;
	if ((flags & ~POWER_FLAGS_SUPPORTED) != 0u) return BRIDGE_HW_ERR_INVAL;

	switch (mode) {
	case 0u: /* run -- cancels a latched, not yet executed Deep-sleep / Standby */
	case 1u: /* sleep -- already in WFI between transport ISRs */
		/* Neither mode arms any wake source -- a non-zero
		 * wake_after_ms here would report STATUS_OK for a timer
		 * that was never armed (gh#261), the same fail-open #107
		 * closed for the wake_bitmap axis.  Refuse before any
		 * hardware touch, matching the standby gate below. */
		if (wake_after_ms != 0u || flags != 0u) return BRIDGE_HW_ERR_INVAL;
		/* RUN and SLEEP both supersede a latched, not yet executed request. */
		if (s_lp_pending_mode != 0u) {
			++bridge_power_diag.cancelled;
			power_cancel_pending();
		}
		if (mode == 1u) {
			++bridge_power_diag.entries[1];
			bridge_power_diag.last_mode = 1u;
		}
		return BRIDGE_HW_OK;
	case 2u: /* deep-sleep */
	case 3u: /* standby */
		/* Deep-sleep wake sources (UM Rev1.2 p.142): any enabled EXTI
		 * line.  The host-reachable one is the SPI CS line (EXTI 8, both
		 * edges, always armed by the SPI transport); RTC wakeup (EXTI 19)
		 * is the bounded fallback.  The I2C slave wakes the part only when
		 * the request carries POWER_FLAG_WAKE_I2C: I2C0 then runs from IRC8M
		 * with WUEN set (APB1 is gated in Deep-sleep, UM p.1279) and EXTI 31
		 * armed.  Without that flag I2C0 is disabled across the entry, so
		 * mode 2, like mode 3, REQUIRES an armed RTC timer: the sleep is
		 * always bounded and a bridge whose host never toggles CS still
		 * comes back.  Host contract: the CS
		 * falling edge that wakes the part is served on the IRC8M clock,
		 * so the frame clocked during that transaction is lost; retry it
		 * (the reply-read retry / STATUS_IO path recovers).  */
		/* Standby is a reset: no peripheral, I2C included, wakes it. */
		if (mode == 3u && flags != 0u) return BRIDGE_HW_ERR_INVAL;
		/* Features that would be cut off (ADC pacing, PWM/capture, DAC,
		 * OTA, an unconfirmed trial) answer BUSY -- retry after stopping
		 * them, or use SLEEP, which keeps every clock running. */
		if (power_entry_refused(mode)) {
			++bridge_power_diag.refused_busy;
			return BRIDGE_HW_ERR_BUSY;
		}
		bridge_rcu_periph_clock_enable(RCU_PMU);
		/* Gate the entry: Standby's wake set is exactly five
		 * sources (UM Rev1.2 p.142 Table 3-1: "1. NRST pin 2. WKUP
		 * pins 3. FWDGT reset 4. RTC 5. LCKMD").  WKUP pins are
		 * unavailable on this SoM (POWER_WAKE_GPIO is rejected above,
		 * #20), so a request that does not arm the RTC wakeup timer
		 * enters a mode only NRST can leave -- refuse before anything
		 * is latched (gh#40 fix 1). */
		const bool no_timer =
		    (wake_after_ms == 0u && (wake_bitmap & (POWER_WAKE_RTC | POWER_WAKE_TIMER)) == 0u);
		/* Every sleep is bounded by the RTC timer.  WAKE_I2C only adds an early
		 * wake on a BRD_I2C address match: the I2C0 wakeup EXTI line is NOT yet
		 * confirmed against the GD32G5x3 user manual (the vendor header names
		 * NVIC IRQ 31 = I2C0_EV_WKUP but gives no EXTI line table, and the UM is
		 * not reachable from the tree), so an untimed WAKE_I2C -- which would
		 * leave CS as the only proven wake -- is refused until a bench run proves
		 * the line. */
		if (no_timer) {
			return ((flags & POWER_FLAG_WAKE_I2C) != 0u) ? BRIDGE_HW_ERR_RANGE
			                                             : BRIDGE_HW_ERR_INVAL;
		}
		if (!no_timer) {
			const uint32_t ms = (wake_after_ms != 0u) ? wake_after_ms : POWER_WAKE_TIMER_MAX_MS;
			/* The FWDGT keeps counting through the mode when its FWDGSPD
			 * bit (STDBY for 3, DPSLP for 2) is 1: a longer wake would end in a watchdog reset instead
			 * of the requested wake.  Refuse before arming anything. */
			const uint32_t fwdgspd =
			    (mode == 2u) ? FMC_OBCTL_FWDGSPD_DPSLP : FMC_OBCTL_FWDGSPD_STDBY;
			if ((FMC_OBCTL & fwdgspd) != 0u && ms > POWER_LP_FWDGT_MAX_MS) {
				return BRIDGE_HW_ERR_RANGE;
			}
			int rc = rtc_wakeup_arm_ms(ms);
			if (rc != BRIDGE_HW_OK) return rc;
		}
		/* Latch the request: the STATUS_OK reply drains normally,
		 * and bridge_power_tick() below performs the entry on a
		 * quiet link.  Wake sources were armed above (pure register
		 * writes, ISR-safe); the bitmap itself needs no deferral
		 * because nothing at tick time re-reads it.  Standby exits
		 * through a power-on reset, so the host's "rebooting, then
		 * re-init and probe" contract applies unchanged.  Deep-sleep instead
		 * resumes in place: bridge_power_tick() restores the PLL and I2C0
		 * on wake, and an RTC timer is mandatory so the sleep is bounded. */
		s_lp_flags        = flags;
		s_lp_timed        = !no_timer;
		s_lp_pending_mode = mode;
		return BRIDGE_HW_OK;
	default:
		return BRIDGE_HW_ERR_INVAL;
	}
}

/* Bounded spin budget for the PLL relock waits below.  The vendor
 * SystemInit() spins unbounded, but this runs at base level on a live
 * bridge: a PLL that never locks must leave the part on IRC8M and
 * report failure, not hang it. */
#define POWER_CLOCK_RESTORE_SPINS 100000u

/* True while a Deep-sleep wake left the part on IRC8M; bridge_power_tick()
 * retries the relock (like bridge_i2c_reinit_pending) until it holds. */
static volatile uint8_t s_clock_relock_pending;

/* Ticks since the last relock retry; see bridge_power_tick(). */
static uint8_t s_clock_relock_tick;

/* Deep-sleep exit clock restore (gh#12).  UM Rev1.2 p.142: on exit
 * "the IRC8M is selected as the system clock", so CK_SYS falls to
 * 8 MHz while the PWM/ADC/DWT constants in gd32_common.h assume 216 MHz.
 * SRAM and registers are preserved across Deep-sleep, so RCU_PLL keeps
 * the PLLP = (IRC8M / 2) * 108 / 2 configuration system_clock_216m_irc8m()
 * (vendor overrides/system_gd32g5x3.c, a static function in another
 * repo, so not callable here) programmed at boot; only the final steps
 * of that sequence need replaying: FMC wait states BEFORE the clock
 * rises, PLLEN, wait PLLSTB, SCS = PLLP, wait SCSS.  PMU_CTL0 (LDOVS
 * 1.15 V) is likewise preserved (UM p.145), so it is not rewritten.
 *
 * Returns false, on IRC8M, if the PLL does not lock or the switch does
 * not take within POWER_CLOCK_RESTORE_SPINS.  A timed-out switch is
 * backed out (SCS = IRC8M) so the part is never left half-switched, and
 * SystemCoreClock plus the bridge_core_clock_* telemetry are refreshed on
 * EVERY exit so they always describe the live clock.  Called by the mode-2
 * wake path and by the bridge_power_tick() relock retry, before the
 * PWM/ADC/DWT users run: PWM_TIMER_CLK_HZ,
 * BRIDGE_ADC_PACE_CLK_HZ and the DWT cycle conversions hardcode 216 MHz.
 * (The I2C timing is NOT one of them -- bridge_transport_i2c_hw_init()
 * derives it from rcu_clock_freq_get(CK_APB1) at run time, gh#41.) */
bool bridge_clock_restore_after_deepsleep(void)
{
	bool ok = false;

	FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT(7);
	RCU_CTL |= RCU_CTL_PLLEN;

	uint32_t to = POWER_CLOCK_RESTORE_SPINS;
	while (0u == (RCU_CTL & RCU_CTL_PLLSTB) && --to != 0u) {
	}

	if (0u != (RCU_CTL & RCU_CTL_PLLSTB)) {
		RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_SCS) | RCU_CKSYSSRC_PLLP;

		to = POWER_CLOCK_RESTORE_SPINS;
		while (RCU_SCSS_PLLP != (RCU_CFG0 & RCU_CFG0_SCSS) && --to != 0u) {
		}

		ok = (RCU_SCSS_PLLP == (RCU_CFG0 & RCU_CFG0_SCSS));
		if (!ok) {
			/* Back the switch out so hardware cannot finish it after we
			 * report failure. */
			RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_SCS) | RCU_CKSYSSRC_IRC8M;
		}
	}

	/* SysTick CTRL is preserved across Deep-sleep; LOAD is re-sized below.
	 * SystemCoreClock and its telemetry mirror are re-derived from the
	 * live RCU state. */
	SystemCoreClockUpdate();
	bridge_core_clock_hz      = SystemCoreClock;
	bridge_core_clock_matches = (SystemCoreClock == PWM_TIMER_CLK_HZ);

	/* SysTick LOAD was sized for the boot clock.  On a failed relock the
	 * part runs on IRC8M, where that reload stretches the 50 ms tick ~27x
	 * past the FWDGT window and the idle bridge would watchdog-reset.
	 * Re-derive the 50 ms period from the live clock (same math as init). */
	uint32_t reload = (SystemCoreClock / 20u) - 1u;
	if (reload > SysTick_LOAD_RELOAD_Msk) reload = SysTick_LOAD_RELOAD_Msk;
	SysTick->LOAD = reload;
	SysTick->VAL  = 0u;
	return ok;
}

/* True while an I2C transaction is in flight (i2c_disable() would cut it off). */
static bool mode_i2c_busy(void)
{
	return RESET != i2c_flag_get(BRIDGE_I2C_PERIPH, I2C_FLAG_I2CBSY);
}

/* Non-zero when the last deep-sleep wake could not bring I2C0 back
 * (bridge_transport_i2c_hw_init() refused, e.g. after a failed PLL
 * relock).  bridge_power_tick() retries it on every later tick, so a
 * transient refusal is not a permanent deaf I2C slave, and a host reading
 * this over SWD sees the failure instead of a silent NACK.
 * bridge_transport_i2c_hw_init() re-derives the timing and the stretch
 * timeout reload from the live APB1 clock on each call. */
volatile uint8_t bridge_i2c_reinit_pending;

/* Every condition that makes a low-power entry pointless or unsafe.  Shared
 * by the pre-entry gates (interrupts enabled) and by the re-check under
 * bridge_irq_lock(), so the two can never drift.  Returns true when the
 * link is quiet.  A WFI with any enabled pending interrupt returns at once
 * (architecturally a no-op), so a pending transport vector (CS-EXTI,
 * I2C0-EV, I2C0-ER -- #262) or the RTC wakeup vector means the sleep would
 * not happen.  Never clears anything: the under-lock call must not erase
 * the very edge it is looking for. */
static bool lp_link_quiet(void)
{
	/* CS de-asserted (PA8 high): an asserted CS means a transaction is in
	 * flight or about to be. */
	if (SET != gpio_input_bit_get(BRIDGE_SPI_NSS_PORT, BRIDGE_SPI_NSS_PIN)) return false;
	/* An in-flight I2C transaction would be cut off by i2c_disable(). */
	if (mode_i2c_busy()) return false;
	if ((EXTI_PD0 != 0u) || ((EXTI_PD1 & 0x0000007Fu) != 0u)) return false;
	if (NVIC_GetPendingIRQ(BRIDGE_SPI_CS_EXTI_IRQN) != 0u) return false;
	if (NVIC_GetPendingIRQ(BRIDGE_I2C_EV_IRQN) != 0u) return false;
	if (NVIC_GetPendingIRQ(BRIDGE_I2C_ER_IRQN) != 0u) return false;
	if (NVIC_GetPendingIRQ(RTC_WKUP_IRQn) != 0u) return false;
	return true;
}

/* The last look before a WFI, taken with PRIMASK set so no ISR can change
 * anything between it and the entry.  The settle spin before it runs with
 * interrupts ON, so a RUN / SLEEP request, a stream start or a CS edge can
 * land there.  Returns 0 = enter, 1 = link not quiet (request stays latched,
 * retry next tick), 2 = request gone (cancelled or overtaken; counted). */
static int lp_final_gate(uint8_t mode)
{
	if (s_lp_pending_mode != mode) {
		++bridge_power_diag.cancelled;
		return 2;
	}
	if (power_entry_refused(mode)) {
		++bridge_power_diag.refused_late;
		power_cancel_pending();
		return 2;
	}
	return lp_link_quiet() ? 0 : 1;
}

/* Base-level low-power entry (gh#63): the deferred half of
 * bridge_hw_power_mode_set().  Runs from bridge_hw_tick() -- base level,
 * AFTER the accepted request's reply has drained onto the wire, with
 * no transport state machine mid-transaction.
 *
 * The pre-entry gates below run with interrupts ENABLED and so only
 * narrow the entry race.  For Deep-sleep the race is closed by taking
 * bridge_irq_lock() first and re-running the whole gate set under it
 * (lp_link_quiet()): with PRIMASK set no transport ISR can start or
 * dispatch a transaction between the re-check and the WFI, and anything
 * pending at that point is visible in the NVIC / EXTI / GPIO state.  On a
 * failed re-check the entry is backed out (I2C0 re-enabled, SysTick
 * restored) and the request stays latched, so the next tick retries.  A
 * CS edge whose ISR already ran to completion before the lock is a
 * finished transaction, not an in-flight one.
 * Every gate failure re-latches the request and retries on the NEXT tick
 * rather than reporting a success that did not happen or silently dropping
 * the request. */
void bridge_power_tick(void)
{
	/* Relock first so the I2C re-init below runs in the same tick, on the
	 * final APB1 clock.  ponytail: a dead PLL costs up to 2 *
	 * POWER_CLOCK_RESTORE_SPINS spins per attempt, so retry every 8th tick
	 * (~400 ms); add an attempt cap if that still starves base-level work. */
	if (s_clock_relock_pending != 0u && (s_clock_relock_tick++ & 7u) == 0u &&
	    bridge_clock_restore_after_deepsleep()) {
		s_clock_relock_pending = 0u;
		/* APB1 moved: re-derive the I2C0 timing for the new clock. */
		/* In I2C-wake mode the kernel clock is IRC8M, so the PLL relock
		 * does not move it and a re-init would cut a live transaction. */
		if (!bridge_transport_i2c_wake_mode()) bridge_i2c_reinit_pending = 1u;
	}

	if (bridge_i2c_reinit_pending != 0u) {
		/* hw_init needs a disabled I2C0 (timeout-enable bits lock the reload
		 * fields, TIMING/OADDR are write-only while I2CEN=1).  Wait out an
		 * in-flight transaction instead of cutting it off. */
		const uint32_t primask = bridge_irq_lock();
		const bool     idle    = !mode_i2c_busy();
		if (idle) i2c_disable(BRIDGE_I2C_PERIPH);
		bridge_irq_unlock(primask);
		if (idle && bridge_transport_i2c_hw_init() == BRIDGE_HW_OK) {
			bridge_i2c_reinit_pending = 0u;
		}
	}

	if (s_lp_pending_mode == 0u) return;

	/* The reply is out; a stream / PWM / OTA session started since would be
	 * cut off by the entry.  Drop the request rather than sleep on it. */
	if (power_entry_refused(s_lp_pending_mode)) {
		++bridge_power_diag.refused_late;
		power_cancel_pending();
		return;
	}

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
	 *    this clear.  Done ONCE, here: the under-lock re-check must not
	 *    clear, or it would erase the edge it is looking for. */
	EXTI_PD0 = 0xFFFFFFFFu;
	EXTI_PD1 = 0x0000007Fu;
	__DSB();

	/* 3. Re-read and abort the attempt rather than reporting a sleep
	 *    that did not happen: a re-pended bit means an edge arrived
	 *    during the clear -- a transaction is starting.  Also covers
	 *    the NVIC pending set (gh#63 step 4) and I2CBSY (4b). */
	if (!lp_link_quiet()) return;

	/* 5. Only now enter. */
	const uint8_t mode = s_lp_pending_mode;

	/* Watchdog-aware entry (gh#54): feed + >= 3 IRC32K intervals of gap
	 * before the wfi (UM p.525).  The ~185 us spin runs with interrupts
	 * still on, which is why Deep-sleep re-checks under the lock below. */
	power_fwdgt_settle_before_lp_entry();

	/* Stop SysTick across the entry (gh#54): a tick pending or firing at
	 * the wfi would end it at once (WFI with an enabled interrupt pending
	 * is a no-op) and the vendor helpers only mask NVIC IRQs, not this
	 * core exception.  Restored on the abort / wake path. */
	const uint32_t systick_ctrl = SysTick->CTRL;
	SysTick->CTRL &= ~(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
	__DSB();

	if (mode == 2u) {
		/* Deep-sleep (gh#12).  PRIMASK is taken BEFORE I2C0 is touched and
		 * stays set through the wfi: an enabled pending interrupt still
		 * ends a WFI, but its handler must not run on the post-wake IRC8M
		 * clock -- restore the PLL first, THEN let the CS-EXTI / RTC
		 * handlers run. */
		const uint32_t primask = bridge_irq_lock();

		const bool want_i2c_wake = (s_lp_flags & POWER_FLAG_WAKE_I2C) != 0u;
		bool       i2c_wake_ok   = false;
		bool       go            = (lp_final_gate(2u) == 0);
		if (go) {
			/* I2C0 off while it is reconfigured (UM p.1279); re-enabled below
			 * or on back-out.  Re-check once more afterwards: an address match
			 * can land while the peripheral is being disabled. */
			bool i2c_off = true;
			i2c_disable(BRIDGE_I2C_PERIPH);
			bridge_transport_i2c_wake_mode_set(want_i2c_wake);
			if (want_i2c_wake) {
				/* Kernel clock -> IRC8M + WUEN, I2C0 re-enabled by the init.
				 * EXTI line 31 is ASSUMED to be the I2C0 wakeup line: NVIC IRQ
				 * 31 (I2C0_EV_WKUP) is a separate numbering, and the UM EXTI
				 * line table (the one that gives RTC wakeup = line 19, UM p.213
				 * Table 5-3) is not reachable from this tree, so the line, and
				 * whether it is configurable, are UNCONFIRMED -- bench item.
				 * Armed like line 19 (rising, interrupt); the wake path below
				 * disables it again. */
				i2c_wake_ok = (bridge_transport_i2c_hw_init() == BRIDGE_HW_OK);
				if (i2c_wake_ok) {
					i2c_off = false;
					exti_flag_clear(EXTI_31);
					exti_init(EXTI_31, EXTI_INTERRUPT, EXTI_TRIG_RISING);
				} else {
					/* Left half-configured: a later tick re-initialises it.  The
					 * request is timed (untimed WAKE_I2C is refused), so the
					 * sleep still ends; only the early I2C wake is lost. */
					bridge_transport_i2c_wake_mode_set(false);
					bridge_i2c_reinit_pending = 1u;
					i2c_off                   = false;
				}
			}
			if (go) go = lp_link_quiet();
			if (!go && i2c_off) i2c_enable(BRIDGE_I2C_PERIPH);
		}
		if (!go) {
			/* Back out: I2C0 is enabled again (or was never disabled),
			 * SysTick restarts its period, and s_lp_pending_mode stays
			 * latched so the next tick retries.  Whatever is pending runs
			 * the moment the lock is released. */
			SysTick->VAL  = 0u;
			SysTick->CTRL = systick_ctrl;
			bridge_irq_unlock(primask);
			return;
		}

		++bridge_power_diag.entries[2];
		bridge_power_diag.last_mode = 2u;
		pmu_to_deepsleepmode(PMU_LDO_LOWPOWER, WFI_CMD);
		const uint32_t wake_t0 = DWT->CYCCNT;
		const uint32_t wake_pd = EXTI_PD0; /* before anything clears it */
		/* A failed relock leaves the part on IRC8M (see the
		 * bridge_core_clock_matches telemetry); the restore already
		 * re-sized SysTick for that clock and later ticks retry it. */
		if (!bridge_clock_restore_after_deepsleep()) {
			s_clock_relock_pending = 1u;
			s_clock_relock_tick    = 0u; /* first retry on the next tick */
		}
		/* A successful init also calls fault_reset_loop_mark_healthy()
		 * (RTC_BKP7 = 0), so every wake resets the consecutive-fault
		 * counter.  Benign: the wake proves the transports came back. */
		/* I2C0 stayed enabled on IRC8M in wake mode: re-initialising it
		 * here would cut the very transaction that woke the part.  Otherwise
		 * (kernel clock = APB1, off across the sleep) bring it back. */
		if (!i2c_wake_ok) {
			/* nonzero = retried by later ticks */
			bridge_i2c_reinit_pending =
			    (bridge_transport_i2c_hw_init() == BRIDGE_HW_OK) ? (uint8_t)0u : (uint8_t)1u;
		}
		/* The timer keeps auto-reloading after a wake; stop it so it
		 * does not interrupt the run-mode bridge every period. */
		(void)rtc_wakeup_disable();
		rtc_flag_clear(RTC_FLAG_WT);
		exti_flag_clear(EXTI_19);
		if (i2c_wake_ok) {
			/* The line was only for the sleep.  I2C0 is still on IRC8M, possibly
			 * mid-transaction: hand it back to APB1 from the tick once idle. */
			exti_interrupt_disable(EXTI_31);
			exti_flag_clear(EXTI_31);
			bridge_transport_i2c_wake_mode_set(false);
			bridge_i2c_reinit_pending = 1u;
		}
		SysTick->VAL  = 0u;
		SysTick->CTRL = systick_ctrl;
		++bridge_power_diag.wakes[2];
		bridge_power_diag.last_wake_pd0         = wake_pd;
		bridge_power_diag.last_wake_source      = power_wake_source_from_pd0(wake_pd);
		bridge_power_diag.i2c_wake_armed        = i2c_wake_ok ? 1u : 0u;
		bridge_power_diag.last_wake_restore_cyc = DWT->CYCCNT - wake_t0;
		s_lp_pending_mode                       = 0u;
		s_lp_flags                              = 0u;
		s_lp_timed                              = false;
		bridge_irq_unlock(primask);
		return;
	}

	if (mode == 3u) {
		/* PRIMASK across the gate and the entry, as for Deep-sleep: a RUN
		 * request arriving in the settle spin must not be lost.  An aborted
		 * entry unwinds below with the lock still held. */
		const uint32_t primask = bridge_irq_lock();
		if (lp_final_gate(3u) != 0) {
			SysTick->VAL  = 0u;
			SysTick->CTRL = systick_ctrl;
			bridge_irq_unlock(primask);
			return;
		}
		++bridge_power_diag.entries[3];
		bridge_power_diag.last_mode = 3u;
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
		SysTick->VAL      = 0u; /* restart the 50 ms period */
		SysTick->CTRL     = systick_ctrl;
		s_lp_pending_mode = 0u; /* drop the failed request */
		s_lp_timed        = false;
		bridge_irq_unlock(primask);
	}
}
