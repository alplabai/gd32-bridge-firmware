/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Register-level ops for clock_source.c, plus the entry points the rest of the
 * HAL uses: boot select, the host-requested HXTAL switch, Deep-sleep relock
 * prepare, CKM NMI recovery, and the live core-clock readers.
 *
 * OSCOUT (PF1) is unused in HXTAL BYPASS mode (OSCIN is driven single-ended,
 * RCU_CTL_HXTALBPS), so it stays free as the E1M IO13 GPIO.  The vendor
 * header only says "external crystal oscillator clock bypass mode enable";
 * the OSCOUT-released statement is the standard bypass contract and is a
 * bench check (docs/BENCH.md), not something this file can prove.
 *
 * Frequency check.  The vendor library has no CTC, but the on-chip trigger
 * router offers HXTAL/32 (TRIGSEL_INPUT_HXTAL_DIV32_TRIG) as a source, routable
 * to TIMER14's ITI14.  Before the PLL is moved onto HXTAL, while SYSCLK is
 * still IRC8M-derived, TIMER14 counts those edges over a DWT-timed window and
 * the count is classified in clock_source.c.  TIMER14 is used by nothing else
 * in this firmware (PWM = TIMER0/7, quadrature = TIMER1..4, ADC pacing =
 * TIMER5/6, sync never touches 14/19); TIMER14, its SYSCFG routing and its
 * TRIGSEL target are released again afterwards.  Static review only: the
 * routing is taken from the vendor header and timer driver and has not run on
 * silicon.  If it counts nothing the result is the SAFE one (FB_HXTAL_FREQ,
 * stay on IRC8M), never a blind PLL switch.
 *
 * What is NOT detected: a clock that changes frequency after the switch (the
 * CKM monitor only sees a stopped clock).  The host must not reprogram SE2
 * once the PLL runs from it.
 */

#include "gd32g5x3.h"

#include "bridge_critical.h"
#include "gd32_common.h"

#include "clock_source.h"
#include "fault_handlers.h"

/* Cycle counter.  A host test replaces this to advance a simulated clock. */
#ifndef BRIDGE_CYCCNT
#define BRIDGE_CYCCNT() (DWT->CYCCNT)
#define BRIDGE_DWT_ENSURE() \
	do { \
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; \
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; \
	} while (0)
#endif

extern bool ota_trial_unconfirmed(void);

volatile uint32_t bridge_clock_hxtal_request; /* SWD / protocol-seam trigger */

/* ------------------------------------------------------------------ */
/* Live clock readers (both builds).                                   */
/* ------------------------------------------------------------------ */

static uint32_t ahb_shift(void)
{
	static const uint8_t exp[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9 };
	return exp[(RCU_CFG0 & RCU_CFG0_AHBPSC) >> 4];
}

/* SYSCLK from RCU, with the OSCIN frequency taken from bridge_clock_input_hz
 * rather than the vendor HXTAL_VALUE (8 MHz). */
static uint32_t live_sysclk_hz(void)
{
	uint32_t hz = BRIDGE_CLOCK_IRC8M_HZ;
	if ((RCU_CFG0 & RCU_CFG0_SCSS) == RCU_SCSS_PLLP) {
		uint32_t in = BRIDGE_CLOCK_IRC8M_HZ;
		if ((RCU_PLL & RCU_PLL_PLLSEL) == RCU_PLLSRC_HXTAL) {
			in = bridge_clock_input_hz != 0u ? bridge_clock_input_hz : BRIDGE_CLOCK_IRC8M_HZ;
		}
		const uint32_t psc = (RCU_PLL & RCU_PLL_PLLPSC) + 1u;
		const uint32_t n   = (RCU_PLL >> 6) & 0xFFu;
		const uint32_t p   = (((RCU_PLL >> 16) & 0x3u) + 1u) << 1;
		hz                 = in / psc * n / p;
	}
	return hz >> ahb_shift();
}

void bridge_clock_core_update(void)
{
	SystemCoreClock = live_sysclk_hz();
}

uint32_t bridge_clock_apb1_hz(void)
{
	const uint32_t f = (RCU_CFG0 & RCU_CFG0_APB1PSC) >> 10;
	return live_sysclk_hz() >> (f >= 4u ? f - 3u : 0u);
}

/* SYSCLK the active selection is meant to produce, for the telemetry flag. */
bool bridge_clock_core_matches(void)
{
	uint32_t want = BRIDGE_CLOCK_CORE_HZ;
	if (bridge_clock_source == BRIDGE_CLOCK_SRC_HXTAL) {
		const bridge_clock_ref_t *ref = bridge_clock_ref_for(bridge_clock_input_hz);
		if (ref != 0) want = ref->sysclk_hz;
	}
	return SystemCoreClock == want;
}

#ifndef BRIDGE_CLOCK_IRC8M_ONLY

/* Re-derive SystemCoreClock + telemetry + the 50 ms SysTick period from the
 * live RCU state (same as power.c does after a relock). */
static void resync_core_clock(void)
{
	bridge_clock_core_update();
	bridge_core_clock_hz      = SystemCoreClock;
	bridge_core_clock_matches = bridge_clock_core_matches();
	uint32_t reload           = (SystemCoreClock / 20u) - 1u;
	if (reload > SysTick_LOAD_RELOAD_Msk) reload = SysTick_LOAD_RELOAD_Msk;
	SysTick->LOAD = reload;
	SysTick->VAL  = 0u;
}

/* ------------------------------------------------------------------ */
/* Time-bounded waits.                                                 */
/* ------------------------------------------------------------------ */

/* True while the FWDGT is running under THIS firmware's window and may be fed
 * from the waits.  Under an OTA trial the bootloader's ~32.8 s dog is the
 * confirm deadline and must not be pushed back. */
static bool s_feed;

static uint32_t cycles_for_us(uint32_t us)
{
	return (live_sysclk_hz() / 1000000u) * us;
}

/* Poll `cond` until it holds or `budget` core cycles have passed.  A CYCCNT that
 * does not advance (debugger-disabled DWT) is detected -- consecutive reads
 * must differ -- and ends the wait at once instead of hanging under PRIMASK. */
static bool wait_cycles(bool (*cond)(void), uint32_t budget)
{
	const uint32_t t0   = BRIDGE_CYCCNT();
	uint32_t       last = t0, same = 0u;
	for (;;) {
		if (cond()) return true;
		if (s_feed) fwdgt_counter_reload();
		const uint32_t now = BRIDGE_CYCCNT();
		if ((uint32_t)(now - t0) >= budget) break;
		if (now == last) {
			if (++same >= 8u) break; /* DWT stalled */
		} else {
			same = 0u;
			last = now;
		}
	}
	return cond();
}

/* `hz` = the LOWEST core clock the wait can run at.  A wait that starts at one
 * clock and finishes at another (the SCS write that drops 216 MHz / AHB 4 to
 * IRC8M / AHB 4) must budget with the slower one, or the "2 ms" is 13 ms. */
static bool wait_for_at(bool (*cond)(void), uint32_t us, uint32_t hz)
{
	return wait_cycles(cond, (hz / 1000000u) * us);
}

static bool wait_for(bool (*cond)(void), uint32_t us)
{
	return wait_cycles(cond, cycles_for_us(us));
}

static void delay_us(uint32_t us)
{
	const uint32_t budget = cycles_for_us(us);
	const uint32_t t0     = BRIDGE_CYCCNT();
	uint32_t       last = t0, same = 0u;
	for (;;) {
		const uint32_t now = BRIDGE_CYCCNT();
		if ((uint32_t)(now - t0) >= budget) break;
		if (now == last) {
			if (++same >= 8u) break;
		} else {
			same = 0u;
			last = now;
		}
	}
}

static bool cond_hxtal_stable(void)
{
	return 0u != (RCU_CTL & RCU_CTL_HXTALSTB);
}
static bool cond_scss_irc8m(void)
{
	return RCU_SCSS_IRC8M == (RCU_CFG0 & RCU_CFG0_SCSS);
}
static bool cond_scss_pllp(void)
{
	return RCU_SCSS_PLLP == (RCU_CFG0 & RCU_CFG0_SCSS);
}
static bool cond_pll_locked(void)
{
	return 0u != (RCU_CTL & RCU_CTL_PLLSTB);
}
static bool cond_pll_off(void)
{
	return 0u == (RCU_CTL & RCU_CTL_PLLSTB);
}

/* ------------------------------------------------------------------ */
/* HXTAL ops.                                                          */
/* ------------------------------------------------------------------ */

static void hw_hxtal_bypass_enable(void)
{
	/* Monitor off first: CKMEN must not survive into a restart (it stays set
	 * through Deep-sleep), and bypass may only change with HXTALEN clear. */
	RCU_CTL &= ~RCU_CTL_CKMEN;
	/* Never clear HXTALEN under a PLL that is running from it (a WFI that
	 * returned without sleeping leaves exactly that): bypass is already set
	 * and the clock is live, so there is nothing to change. */
	if ((RCU_PLL & RCU_PLL_PLLSEL) == RCU_PLLSRC_HXTAL && 0u != (RCU_CTL & RCU_CTL_PLLEN)) return;
	RCU_CTL &= ~RCU_CTL_HXTALEN;
	RCU_CTL |= RCU_CTL_HXTALBPS;
}

static void hw_hxtal_enable(void)
{
	RCU_CTL |= RCU_CTL_HXTALEN;
}

static bool hw_hxtal_wait_stable(void)
{
	return wait_for(cond_hxtal_stable, BRIDGE_CLOCK_HXTAL_STARTUP_US);
}

static void hw_hxtal_stop(void)
{
	RCU_CTL &= ~RCU_CTL_CKMEN; /* monitor off first: no NMI from our own stop */
	RCU_CTL &= ~RCU_CTL_HXTALEN;
	RCU_CTL &= ~RCU_CTL_HXTALBPS;
}

static void hw_ckm_enable(void)
{
	RCU_INT |= RCU_INT_CKMIC; /* a stale flag would NMI at once */
	SYSCFG_STAT = SYSCFG_STAT_CKMNMIIF;
	RCU_CTL |= RCU_CTL_CKMEN;
}

static void hw_ckm_ack(void)
{
	RCU_CTL &= ~RCU_CTL_CKMEN;          /* cannot re-fire while we recover */
	RCU_INT |= RCU_INT_CKMIC;           /* the RCU source first ... */
	SYSCFG_STAT = SYSCFG_STAT_CKMNMIIF; /* ... then the SYSCFG mirror (rc_w1) */
}

/* HXTAL/32 -> TIMER14 ITI14 as an external clock (slave mode external 0).
 * Returns the count NORMALISED to `window_us` of core time: the window is
 * measured with the cycle counter between timer_enable() and the counter read
 * (the SYSCFG/TRIGSEL/timer calls cost real time, a large share at 8 MHz), so
 * f = counts * core_hz / elapsed_cycles. */
static uint32_t hw_hxtal_div32_count(uint32_t window_us)
{
	bridge_rcu_periph_clock_enable(RCU_TRIGSEL);
	bridge_rcu_periph_clock_enable(RCU_TIMER14);

	trigsel_init(TRIGSEL_OUTPUT_TIMER14_ITI14, TRIGSEL_INPUT_HXTAL_DIV32_TRIG);
	timer_deinit(TIMER14);
	/* timer_deinit() leaves CAR at its reset value; an explicit full-range
	 * reload keeps the counter from stopping at a stale/zero auto-reload. */
	timer_autoreload_value_config(TIMER14, 0xFFFFu);
	timer_input_trigger_source_select(TIMER14, TIMER_SMCFG_TRGSEL_ITI14);
	timer_slave_mode_select(TIMER14, TIMER_SLAVE_MODE_EXTERNAL0);
	timer_counter_value_config(TIMER14, 0u);
	timer_enable(TIMER14);
	const uint32_t c0 = BRIDGE_CYCCNT();

	delay_us(window_us);
	const uint32_t counts = timer_counter_read(TIMER14);
	const uint32_t c1     = BRIDGE_CYCCNT();

	/* Release everything this touched. */
	timer_disable(TIMER14);
	for (uint32_t i = 0u; i < 3u; i++) {
		REG32(SYSCFG_TIMERCFG(SYSCFG_TIMER14) + 4u * i) = 0u;
	}
	timer_deinit(TIMER14);
	trigsel_init(TRIGSEL_OUTPUT_TIMER14_ITI14, TRIGSEL_INPUT_0);
	{
		const uint32_t primask = bridge_irq_lock();
		rcu_periph_clock_disable(RCU_TIMER14);
		bridge_irq_unlock(primask);
	}

	const uint32_t elapsed = (uint32_t)(c1 - c0);
	if (elapsed == 0u) return 0u;
	return (uint32_t)((uint64_t)counts * live_sysclk_hz() * window_us /
	                  ((uint64_t)elapsed * 1000000u));
}

/* ------------------------------------------------------------------ */
/* PLL select, with the vendor's Vcore step.                           */
/* ------------------------------------------------------------------ */

/* Field encodings, same as the vendor override's 216M tuple (PLLN bits 6..13
 * is `n` itself). */
#define PLL_P_FIELD ((BRIDGE_CLOCK_PLL_P >> 1u) - 1u) /* PLLP   bits 16..17 */
#define PLL_Q_FIELD 2u                                /* PLLQ   bits 23..26 */
#define PLL_R_FIELD 2u                                /* PLLR   bits 27..31 */

/* system_gd32g5x3.c RCU_MODIFY_DE_2 / _soft_delay_: leaving 216 MHz without
 * stepping AHB down first fluctuates Vcore ("strongly recommended ... to avoid
 * issues caused by self-removal").  The vendor delays are ~1 us (0x50 loops)
 * between steps and ~3 us (200 loops) after the SCS write; these are time
 * based and a little longer. */
#define VCORE_STEP_US   2u
#define VCORE_SETTLE_US 5u

static void ahb_set(uint32_t ahbpsc)
{
	RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_AHBPSC) | ahbpsc;
}

static void vcore_step_down(void)
{
	delay_us(VCORE_STEP_US);
	ahb_set(RCU_AHB_CKSYS_DIV2);
	delay_us(VCORE_STEP_US);
	ahb_set(RCU_AHB_CKSYS_DIV4);
	delay_us(VCORE_STEP_US);
}

static void vcore_step_up(void)
{
	ahb_set(RCU_AHB_CKSYS_DIV2);
	delay_us(VCORE_STEP_US);
	ahb_set(RCU_AHB_CKSYS_DIV1);
	delay_us(VCORE_STEP_US);
}

static bool hw_pll_select(bridge_clock_source_t src, uint32_t input_hz)
{
	uint32_t psc = BRIDGE_CLOCK_IRC8M_HZ / BRIDGE_CLOCK_PLL_IN_HZ; /* IRC8M tuple */
	uint32_t n   = BRIDGE_CLOCK_PLL_N;
	if (src == BRIDGE_CLOCK_SRC_HXTAL) {
		const bridge_clock_ref_t *ref = bridge_clock_ref_for(input_hz);
		if (ref == 0) return false;
		psc = ref->pll_psc;
		n   = ref->pll_n;
	}
	const uint32_t psc_field = psc - 1u; /* PLLPSC bits 0..3 */

	/* Leave the PLL: SYSCLK to IRC8M (always running), PLL off.  From 216 MHz
	 * AHB steps /2 then /4 first, and is restored to /1 only once SYSCLK is on
	 * the 8 MHz IRC8M, so Vcore never sees the full step. */
	const bool leaving_pll = (RCU_CFG0 & RCU_CFG0_SCSS) == RCU_SCSS_PLLP;
	RCU_CTL |= RCU_CTL_IRC8MEN;
	if (leaving_pll) vcore_step_down();
	RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_SCS) | RCU_CKSYSSRC_IRC8M;
	delay_us(VCORE_SETTLE_US);
	/* The core drops to IRC8M (still AHB /4 when leaving the PLL) during this
	 * wait: budget with that clock, not the 216 MHz it starts at. */
	if (!wait_for_at(
	        cond_scss_irc8m, BRIDGE_CLOCK_SWITCH_US, BRIDGE_CLOCK_IRC8M_HZ >> ahb_shift())) {
		if (leaving_pll) vcore_step_up();
		return false;
	}
	ahb_set(RCU_AHB_CKSYS_DIV1);
	RCU_CTL &= ~RCU_CTL_PLLEN;
	if (!wait_for(cond_pll_off, BRIDGE_CLOCK_SWITCH_US)) return false;

	RCU_PLL &= ~(RCU_PLL_PLLSEL | RCU_PLL_PLLPSC | RCU_PLL_PLLN | RCU_PLL_PLLP | RCU_PLL_PLLQ |
	             RCU_PLL_PLLR);
	RCU_PLL |= ((src == BRIDGE_CLOCK_SRC_HXTAL ? RCU_PLLSRC_HXTAL : RCU_PLLSRC_IRC8M) |
	            (psc_field << 0) | (n << 6) | (PLL_P_FIELD << 16) | (PLL_Q_FIELD << 23) |
	            (PLL_R_FIELD << 27) | RCU_PLL_PLLPEN | RCU_PLL_PLLQEN | RCU_PLL_PLLREN);

	/* Same wait states as the IRC8M 216 MHz path; LDO voltage is untouched
	 * (PMU_CTL0 was set by SystemInit and survives Deep-sleep). */
	FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT(7);

	RCU_CTL |= RCU_CTL_PLLEN;
	if (!wait_for(cond_pll_locked, BRIDGE_CLOCK_PLL_LOCK_US)) goto back_out;
	RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_SCS) | RCU_CKSYSSRC_PLLP;
	if (!wait_for(cond_scss_pllp, BRIDGE_CLOCK_SWITCH_US)) goto back_out;
	return true;

back_out:
	RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_SCS) | RCU_CKSYSSRC_IRC8M;
	return false;
}

static uint32_t hw_marker_get(void)
{
	return RTC_BKP9;
}

static void hw_marker_set(uint32_t value)
{
	RTC_BKP9 = value;
}

static const bridge_clock_ops_t s_ops = {
	.hxtal_bypass_enable = hw_hxtal_bypass_enable,
	.hxtal_enable        = hw_hxtal_enable,
	.hxtal_wait_stable   = hw_hxtal_wait_stable,
	.hxtal_div32_count   = hw_hxtal_div32_count,
	.hxtal_stop          = hw_hxtal_stop,
	.pll_select          = hw_pll_select,
	.ckm_enable          = hw_ckm_enable,
	.ckm_ack             = hw_ckm_ack,
	.marker_get          = hw_marker_get,
	.marker_set          = hw_marker_set,
};

/* ------------------------------------------------------------------ */
/* Entry points.                                                       */
/* ------------------------------------------------------------------ */

void bridge_clock_init(bool fwdgt_running)
{
	s_feed = fwdgt_running;
	BRIDGE_DWT_ENSURE();
	/* RTC_BKP9 needs PMUEN + BKPWEN, both set at the head of bridge_hw_init().
	 * IRC8M stays the default; HXTAL at boot is opt-in (SE2 is not final yet
	 * on current units). */
#ifdef BRIDGE_CLOCK_HXTAL_AT_BOOT
	(void)bridge_clock_boot(&s_ops, true);
#else
	(void)bridge_clock_boot(&s_ops, false);
#endif
}

volatile bridge_clock_switch_status_t bridge_clock_switch_status = BRIDGE_CLOCK_SW_IDLE;

static uint32_t s_quiet_ticks, s_pending_ticks;

/* Supervised outputs must never see the ~27x period stretch of the PLL rebuild:
 * refuse while any PWM channel (output or capture) or ADC stream is active.
 * The quadrature encoders are not listed: they count external edges with no
 * SYSCLK-derived period (only the input filter time shifts by the clock ratio),
 * are enabled at boot, and have no session to wait for. */
static bool switch_blocked(void)
{
	return pwm_any_claimed() || adc_stream_any_active();
}

bool bridge_clock_try_hxtal(void)
{
	if (bridge_clock_source == BRIDGE_CLOCK_SRC_HXTAL) {
		bridge_clock_switch_status = BRIDGE_CLOCK_SW_DONE_HXTAL;
		return true;
	}
	if (switch_blocked()) {
		bridge_clock_switch_status = BRIDGE_CLOCK_SW_REFUSED_BUSY;
		return false; /* nothing touched, nothing latched */
	}
	s_feed = !ota_trial_unconfirmed();
	BRIDGE_DWT_ENSURE();
	bridge_clock_switch_status = BRIDGE_CLOCK_SW_RUNNING;
	/* PRIMASK across the whole sequence, as the Deep-sleep wake does: no
	 * transport ISR may run while the core is on a transient clock. */
	const uint32_t primask = bridge_irq_lock();
	(void)bridge_clock_attempt_hxtal(&s_ops);
	resync_core_clock();
	bridge_irq_unlock(primask);
	/* The truth is the active source: an NMI may have fallen back already. */
	const bool ok              = (bridge_clock_source == BRIDGE_CLOCK_SRC_HXTAL);
	bridge_clock_switch_status = ok ? BRIDGE_CLOCK_SW_DONE_HXTAL : BRIDGE_CLOCK_SW_DONE_FALLBACK;
	return ok;
}

void bridge_clock_request_hxtal(void)
{
	bridge_clock_hxtal_request = 1u;
}

void bridge_clock_tick(void)
{
	if (bridge_clock_hxtal_request != 0u) {
		bridge_clock_hxtal_request = 0u;
		if (bridge_clock_switch_status != BRIDGE_CLOCK_SW_PENDING &&
		    bridge_clock_switch_status != BRIDGE_CLOCK_SW_RUNNING) {
			if (bridge_clock_source == BRIDGE_CLOCK_SRC_HXTAL) {
				bridge_clock_switch_status = BRIDGE_CLOCK_SW_DONE_HXTAL;
			} else if (switch_blocked()) {
				bridge_clock_switch_status = BRIDGE_CLOCK_SW_REFUSED_BUSY;
			} else {
				bridge_clock_switch_status = BRIDGE_CLOCK_SW_PENDING;
				s_quiet_ticks              = 0u;
				s_pending_ticks            = 0u;
			}
		}
	}
	if (bridge_clock_switch_status == BRIDGE_CLOCK_SW_PENDING) {
		if (switch_blocked() || ++s_pending_ticks > BRIDGE_CLOCK_PENDING_MAX_TICKS) {
			bridge_clock_switch_status = BRIDGE_CLOCK_SW_REFUSED_BUSY; /* drop it */
			return;
		}
		s_quiet_ticks = bridge_link_quiet() ? s_quiet_ticks + 1u : 0u;
		if (s_quiet_ticks >= BRIDGE_CLOCK_QUIET_TICKS) {
			(void)bridge_clock_try_hxtal();
			return; /* the marker is cleared by the NEXT healthy tick */
		}
		return;
	}
	bridge_clock_mark_healthy(&s_ops);
}

void bridge_clock_pre_deepsleep(void)
{
	/* HXTAL stops in Deep-sleep; with CKMEN still set the wake would NMI
	 * before relock_prepare() runs. */
	if (bridge_clock_source == BRIDGE_CLOCK_SRC_HXTAL) RCU_CTL &= ~RCU_CTL_CKMEN;
}

bool bridge_clock_relock_prepare(void)
{
	if (bridge_clock_source != BRIDGE_CLOCK_SRC_HXTAL) return false;
	s_feed = !ota_trial_unconfirmed();
	BRIDGE_DWT_ENSURE();
	/* A WFI that returned without sleeping (an edge latched between the quiet
	 * check and the entry) leaves HXTAL and its PLL fully live: only the
	 * monitor, taken off before the entry, needs re-arming.  Restarting HXTAL
	 * here would clear HXTALEN under the running PLL. */
	if ((RCU_CFG0 & RCU_CFG0_SCSS) == RCU_SCSS_PLLP && 0u != (RCU_CTL & RCU_CTL_PLLSTB) &&
	    0u != (RCU_CTL & RCU_CTL_HXTALSTB)) {
		hw_ckm_enable();
		return true;
	}
	/* Deep-sleep stopped HXTAL; bring it back (or fall back) before the caller
	 * would replay PLLEN.  Runs at 8 MHz under PRIMASK: time-bounded, see
	 * clock_source.h for the ~23 ms worst case against the 445 ms FWDGT. */
	(void)bridge_clock_external_start(&s_ops, true);
	/* Both outcomes (HXTAL, or the rebuilt IRC8M PLL) end on PLLP with the PLL
	 * locked: then the caller must not replay it. */
	return (RCU_CFG0 & RCU_CFG0_SCSS) == RCU_SCSS_PLLP && 0u != (RCU_CTL & RCU_CTL_PLLSTB);
}

/* SYSCFG_STAT bits that are events, not NMI sources of their own (single-bit
 * ECC corrections): present alongside a CKM NMI they do not make it a fault. */
#define CKM_TOLERATED_STAT \
	(SYSCFG_STAT_CKMNMIIF | SYSCFG_STAT_SRAM0ECCSEIF | SYSCFG_STAT_SRAM1ECCSEIF | \
	 SYSCFG_STAT_TCMSRAMECCSEIF)

/* NMI-context recovery.  The CKM hardware has already moved SYSCLK to IRC8M
 * and dropped the PLL; rebuild 216 MHz from IRC8M and keep running.  Only a
 * CKM NMI with no multi-bit ECC, flash ECC or NMI-pin bit qualifies. */
bool bridge_clock_nmi_recover(uint32_t syscfg_stat)
{
	if (0u == (syscfg_stat & SYSCFG_STAT_CKMNMIIF)) return false;
	if (0u != (syscfg_stat & ~CKM_TOLERATED_STAT)) return false;
	BRIDGE_DWT_ENSURE();
	if (!bridge_clock_on_ckm_nmi(&s_ops)) return false;
	resync_core_clock();
	return true;
}

#else /* BRIDGE_CLOCK_IRC8M_ONLY */

void bridge_clock_init(bool fwdgt_running)
{
	(void)fwdgt_running;
	bridge_clock_source   = BRIDGE_CLOCK_SRC_IRC8M;
	bridge_clock_fallback = BRIDGE_CLOCK_FB_BUILD_DISABLED;
}

bool bridge_clock_try_hxtal(void)
{
	return false;
}
void bridge_clock_request_hxtal(void)
{
}
volatile bridge_clock_switch_status_t bridge_clock_switch_status = BRIDGE_CLOCK_SW_IDLE;

void bridge_clock_tick(void)
{
	if (bridge_clock_hxtal_request != 0u) {
		bridge_clock_hxtal_request = 0u;
		bridge_clock_switch_status = BRIDGE_CLOCK_SW_DONE_FALLBACK; /* FB_BUILD_DISABLED */
	}
}
void bridge_clock_pre_deepsleep(void)
{
}
bool bridge_clock_relock_prepare(void)
{
	return false;
}
bool bridge_clock_nmi_recover(uint32_t syscfg_stat)
{
	(void)syscfg_stat;
	return false;
}

#endif
