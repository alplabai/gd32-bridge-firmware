/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Register-level ops for clock_source.c, plus the three entry points the rest
 * of the HAL uses: boot select, Deep-sleep relock prepare, CKM NMI recovery.
 *
 * OSCOUT (PF1) is unused in HXTAL BYPASS mode (OSCIN is driven single-ended,
 * RCU_CTL_HXTALBPS), so it stays free as the E1M IO13 GPIO.  The vendor
 * header only says "external crystal oscillator clock bypass mode enable";
 * the OSCOUT-released statement is the standard bypass contract and is a
 * bench check (docs/BENCH.md), not something this file can prove.
 *
 * No clock-measure path exists on this part for this use: there is no CTC in
 * the vendor library and nothing independent of SYSCLK to count against.  The
 * only checks are HXTALSTB, PLLSTB, the SCSS read-back, and the CKM monitor.
 */

#include "gd32g5x3.h"

#include "gd32_common.h"

#include "clock_source.h"
#include "fault_handlers.h"

#ifndef BRIDGE_CLOCK_IRC8M_ONLY

#define CLOCK_SWITCH_SPINS 100000u

/* PLL tuple, same encoding as the vendor override's 216M tuple. */
#define PLL_PSC_FIELD (BRIDGE_CLOCK_PLL_PSC - 1u)       /* PLLPSC bits 0..3  */
#define PLL_N_FIELD   (BRIDGE_CLOCK_PLL_N)              /* PLLN   bits 6..13 */
#define PLL_P_FIELD   ((BRIDGE_CLOCK_PLL_P >> 1u) - 1u) /* PLLP   bits 16..17 */
#define PLL_Q_FIELD   2u                                /* PLLQ   bits 23..26 */
#define PLL_R_FIELD   2u                                /* PLLR   bits 27..31 */

static void hw_hxtal_bypass_enable(void)
{
	RCU_CTL &= ~RCU_CTL_HXTALEN; /* bypass may only change with HXTALEN clear */
	RCU_CTL |= RCU_CTL_HXTALBPS;
}

static void hw_hxtal_enable(void)
{
	RCU_CTL |= RCU_CTL_HXTALEN;
}

static bool hw_hxtal_stable(void)
{
	return 0u != (RCU_CTL & RCU_CTL_HXTALSTB);
}

static void hw_hxtal_stop(void)
{
	RCU_CTL &= ~RCU_CTL_CKMEN; /* monitor off first: no NMI from our own stop */
	RCU_CTL &= ~RCU_CTL_HXTALEN;
	RCU_CTL &= ~RCU_CTL_HXTALBPS;
}

static void hw_ckm_enable(void)
{
	RCU_CTL |= RCU_CTL_CKMEN;
}

static bool wait_scss(uint32_t scss)
{
	for (uint32_t to = CLOCK_SWITCH_SPINS; to != 0u; to--) {
		if (scss == (RCU_CFG0 & RCU_CFG0_SCSS)) return true;
	}
	return scss == (RCU_CFG0 & RCU_CFG0_SCSS);
}

static bool wait_pllstb(bool want)
{
	for (uint32_t to = CLOCK_SWITCH_SPINS; to != 0u; to--) {
		if ((0u != (RCU_CTL & RCU_CTL_PLLSTB)) == want) return true;
	}
	return (0u != (RCU_CTL & RCU_CTL_PLLSTB)) == want;
}

static bool hw_pll_select(bridge_clock_source_t src)
{
	/* Leave the PLL: SYSCLK to IRC8M (always running), PLL off. */
	RCU_CTL |= RCU_CTL_IRC8MEN;
	RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_SCS) | RCU_CKSYSSRC_IRC8M;
	if (!wait_scss(RCU_SCSS_IRC8M)) return false;
	RCU_CTL &= ~RCU_CTL_PLLEN;
	if (!wait_pllstb(false)) return false;

	RCU_PLL &= ~(RCU_PLL_PLLSEL | RCU_PLL_PLLPSC | RCU_PLL_PLLN | RCU_PLL_PLLP | RCU_PLL_PLLQ |
	             RCU_PLL_PLLR);
	RCU_PLL |=
	    ((src == BRIDGE_CLOCK_SRC_HXTAL ? RCU_PLLSRC_HXTAL : RCU_PLLSRC_IRC8M) |
	     (PLL_PSC_FIELD << 0) | (PLL_N_FIELD << 6) | (PLL_P_FIELD << 16) | (PLL_Q_FIELD << 23) |
	     (PLL_R_FIELD << 27) | RCU_PLL_PLLPEN | RCU_PLL_PLLQEN | RCU_PLL_PLLREN);

	/* Same wait states as the IRC8M 216 MHz path; LDO voltage is untouched
	 * (PMU_CTL0 was set by SystemInit and survives Deep-sleep). */
	FMC_WS = (FMC_WS & (~FMC_WS_WSCNT)) | WS_WSCNT(7);

	RCU_CTL |= RCU_CTL_PLLEN;
	if (!wait_pllstb(true)) goto back_out;
	RCU_CFG0 = (RCU_CFG0 & ~RCU_CFG0_SCS) | RCU_CKSYSSRC_PLLP;
	if (!wait_scss(RCU_SCSS_PLLP)) goto back_out;
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
	.hxtal_stable        = hw_hxtal_stable,
	.hxtal_stop          = hw_hxtal_stop,
	.pll_select          = hw_pll_select,
	.ckm_enable          = hw_ckm_enable,
	.marker_get          = hw_marker_get,
	.marker_set          = hw_marker_set,
};

/* Re-derive SystemCoreClock + telemetry + the 50 ms SysTick period from the
 * live RCU state (same as power.c does after a relock). */
static void resync_core_clock(void)
{
	SystemCoreClockUpdate();
	bridge_core_clock_hz      = SystemCoreClock;
	bridge_core_clock_matches = (SystemCoreClock == PWM_TIMER_CLK_HZ);
	uint32_t reload           = (SystemCoreClock / 20u) - 1u;
	if (reload > SysTick_LOAD_RELOAD_Msk) reload = SysTick_LOAD_RELOAD_Msk;
	SysTick->LOAD = reload;
	SysTick->VAL  = 0u;
}

void bridge_clock_init(void)
{
	/* RTC_BKP9 needs PMUEN + BKPWEN, both set at the head of bridge_hw_init(). */
	(void)bridge_clock_select(&s_ops, BRIDGE_CLOCK_HXTAL_SPINS);
}

void bridge_clock_relock_prepare(void)
{
	if (bridge_clock_source != BRIDGE_CLOCK_SRC_HXTAL) return;
	/* Deep-sleep stopped HXTAL; bring it back (or fall back) before the
	 * caller re-enables the PLL. */
	(void)bridge_clock_external_start(&s_ops, BRIDGE_CLOCK_HXTAL_SPINS, true);
}

void bridge_clock_healthy(void)
{
	bridge_clock_mark_healthy(&s_ops);
}

/* NMI-context recovery.  The CKM hardware has already moved SYSCLK to IRC8M
 * and dropped the PLL; rebuild 216 MHz from IRC8M and keep running. */
bool bridge_clock_nmi_recover(uint32_t syscfg_stat)
{
	if (syscfg_stat != SYSCFG_STAT_CKMNMIIF) return false;
	bridge_clock_on_ckm_failure(&s_ops);
	SYSCFG_STAT = SYSCFG_STAT_CKMNMIIF; /* rc_w1 */
	resync_core_clock();
	return true;
}

#else /* BRIDGE_CLOCK_IRC8M_ONLY */

void bridge_clock_init(void)
{
	bridge_clock_source   = BRIDGE_CLOCK_SRC_IRC8M;
	bridge_clock_fallback = BRIDGE_CLOCK_FB_BUILD_DISABLED;
}

void bridge_clock_relock_prepare(void)
{
}
void bridge_clock_healthy(void)
{
}
bool bridge_clock_nmi_recover(uint32_t syscfg_stat)
{
	(void)syscfg_stat;
	return false;
}

#endif
