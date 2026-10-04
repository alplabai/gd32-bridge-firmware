/* SPDX-License-Identifier: Apache-2.0
 *
 * Register-level host tests for hal/gd32/clock_hw.c + clock_source.c: the real
 * ops run against the RCU/FMC/SYSCFG/TIMER14 simulation in mock/mock_hw.c.
 * They pin the field encodings, the order bits are written in (AHB stepping,
 * bypass before enable, CKMEN handling, CKMIC before the SYSCFG clear), the
 * time bounds with simulated time, and the frequency check.  Static
 * simulation only: none of this proves the silicon behaves as modelled.
 */

#include <zephyr/ztest.h>

#include <stddef.h>
#include <string.h>

#include "gd32g5x3.h"

#include "clock_source.h"
#include "gd32_common.h"
#include "mock_hw.h"

#define MS_NS 1000000ull

static void reset(void)
{
	mock_hw_reset();
	bridge_clock_source        = BRIDGE_CLOCK_SRC_IRC8M;
	bridge_clock_fallback      = BRIDGE_CLOCK_FB_NONE;
	bridge_clock_input_hz      = 0u;
	bridge_clock_hxtal_request = 0u;
	bridge_clock_init(true); /* default build: stays on IRC8M, marker handled */
	mock_log_clear();
	mock_fwdgt_feeds = 0u;
}

static uint64_t t0;
static void     mark(void)
{
	t0 = mock_now_ns();
}
static uint64_t since(void)
{
	return mock_now_ns() - t0;
}

ZTEST_SUITE(gd32_bridge_clock_hw, NULL, NULL, NULL, NULL, NULL);

/* ---- default boot ---- */

ZTEST(gd32_bridge_clock_hw, test_default_boot_stays_on_the_irc8m_pll_and_touches_no_register)
{
	reset();
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_NOT_REQUESTED);
	zassert_equal(bridge_clock_input_hz, 0u);
	zassert_equal(RCU_CFG0 & RCU_CFG0_SCSS, RCU_SCSS_PLLP);
	zassert_equal(strlen(mock_log_text()), 0u, "no register written: %s", mock_log_text());
}

/* ---- the host-requested switch: encodings and order ---- */

ZTEST(gd32_bridge_clock_hw, test_8mhz_switch_register_encodings_and_write_order)
{
	reset();
	mark();
	zassert_true(bridge_clock_try_hxtal());
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_HXTAL);
	zassert_equal(bridge_clock_input_hz, BRIDGE_CLOCK_IN_8MHZ);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_NONE);

	/* RCU_PLL: PLLPSC /2 (field 1), PLLN 108, PLLP /2 (field 0), source HXTAL. */
	zassert_equal(RCU_PLL & RCU_PLL_PLLPSC, 1u);
	zassert_equal((RCU_PLL & RCU_PLL_PLLN) >> 6, 108u);
	zassert_equal(RCU_PLL & RCU_PLL_PLLP, 0u);
	zassert_equal(RCU_PLL & RCU_PLL_PLLSEL, RCU_PLLSRC_HXTAL);
	zassert_equal((RCU_PLL & RCU_PLL_PLLQ) >> 23, 2u);
	zassert_equal((RCU_PLL & RCU_PLL_PLLR) >> 27, 2u);
	zassert_true((RCU_PLL & (RCU_PLL_PLLPEN | RCU_PLL_PLLQEN | RCU_PLL_PLLREN)) ==
	             (RCU_PLL_PLLPEN | RCU_PLL_PLLQEN | RCU_PLL_PLLREN));
	zassert_equal(RCU_CTL & (RCU_CTL_HXTALEN | RCU_CTL_HXTALBPS | RCU_CTL_CKMEN | RCU_CTL_PLLEN),
	              RCU_CTL_HXTALEN | RCU_CTL_HXTALBPS | RCU_CTL_CKMEN | RCU_CTL_PLLEN);
	zassert_equal(RCU_CFG0 & RCU_CFG0_SCSS, RCU_SCSS_PLLP);
	zassert_equal(RCU_CFG0 & RCU_CFG0_AHBPSC, RCU_AHB_CKSYS_DIV1, "AHB restored to /1");
	zassert_equal(FMC_WS & FMC_WS_WSCNT, 7u);

	zassert_equal(SystemCoreClock, 216000000u);
	zassert_equal(bridge_core_clock_hz, 216000000u);
	zassert_true(bridge_core_clock_matches);
	zassert_equal(mock_systick.LOAD, 216000000u / 20u - 1u, "50 ms SysTick from the live clock");

	/* Order: bypass set while HXTALEN is clear, then enable; frequency check;
	 * AHB steps down BEFORE leaving the PLL and is restored on IRC8M; PLL off,
	 * reprogrammed, relocked, selected; the monitor armed last. */
	zassert_true(mock_in_order("BKP9=48545831",
	                           "BPS+",
	                           "HXEN+",
	                           "TRIG:88/a3",
	                           "ITI14",
	                           "EXT0",
	                           "T14ON",
	                           "T14OFF",
	                           "AHB8",
	                           "AHB9",
	                           "SCS0",
	                           "AHB0",
	                           "PLLEN-",
	                           "PLLCFG:2/108/2/H",
	                           "PLLEN+",
	                           "SCS3",
	                           "CKM+",
	                           NULL),
	             "write order wrong: %s",
	             mock_log_text());

	/* The frequency-check hardware is released again. */
	zassert_equal(mock_syscfg_timer14_cfg(), 0u, "SYSCFG TIMER14 routing cleared");
	zassert_equal(mock_trigsel_target_source(), 0u, "TRIGSEL target released");
	zassert_equal(mock_timer14_clk_enabled(), 0u, "TIMER14 clock off");
	zassert_true(mock_in_order("T14OFF", "T14DEINIT", "TRIG:88/0", "CLK2-", NULL));

	zassert_true(
	    since() < 6 * MS_NS, "typical switch is a few ms: %llu ns", (unsigned long long)since());
	zassert_true(mock_primask_max_ns > 0u, "PRIMASK held across the sequence");
	zassert_equal(mock_primask, 0u, "and released");
	zassert_true(mock_fwdgt_feeds > 0u, "FWDGT fed during the waits (not a trial)");
}

ZTEST(gd32_bridge_clock_hw, test_marker_is_set_before_and_cleared_after_a_healthy_tick)
{
	reset();
	zassert_true(bridge_clock_try_hxtal());
	zassert_equal(RTC_BKP9, BRIDGE_CLOCK_ATTEMPT_MAGIC);
	bridge_clock_tick(); /* the first healthy tick after the switch */
	zassert_equal(RTC_BKP9, 0u);
}

ZTEST(gd32_bridge_clock_hw, test_supported_references_program_their_own_pll_tuple)
{
	static const struct {
		uint32_t osc, psc, n, sys, ref;
	} t[] = {
		{ 12000000u, 3u, 108u, 216000000u, BRIDGE_CLOCK_IN_12MHZ },
		{ 16000000u, 4u, 108u, 216000000u, BRIDGE_CLOCK_IN_16MHZ },
		{ 20000000u, 5u, 108u, 216000000u, BRIDGE_CLOCK_IN_20MHZ },
		{ 24576000u, 6u, 105u, 215040000u, BRIDGE_CLOCK_IN_24P576 },
	};
	for (unsigned i = 0; i < sizeof t / sizeof t[0]; i++) {
		reset();
		mock_scn.oscin_hz = t[i].osc;
		zassert_true(bridge_clock_try_hxtal(), "osc %u", (unsigned)t[i].osc);
		zassert_equal(bridge_clock_input_hz, t[i].ref);
		zassert_equal((RCU_PLL & RCU_PLL_PLLPSC) + 1u, t[i].psc);
		zassert_equal((RCU_PLL & RCU_PLL_PLLN) >> 6, t[i].n);
		zassert_equal(SystemCoreClock, t[i].sys, "SYSCLK follows the tuple, not a constant");
		zassert_equal(mock_sim_sysclk_hz(), t[i].sys, "and the simulated hardware agrees");
		zassert_true(bridge_core_clock_matches);
	}
}

ZTEST(gd32_bridge_clock_hw, test_timer_prescalers_follow_the_live_clock)
{
	reset();
	mock_scn.oscin_hz = 24576000u;
	zassert_true(bridge_clock_try_hxtal());
	zassert_equal(bridge_core_clock_hz, 215040000u);
	zassert_equal(bridge_timer_prescaler(1000000u), 214u, "/215 -> 1.0002 MHz, not the stale /216");
	zassert_equal(bridge_timer_prescaler(10000u), 21503u);
	bridge_core_clock_hz = 216000000u;
	zassert_equal(bridge_timer_prescaler(1000000u), 215u);
	zassert_equal(bridge_timer_prescaler(10000u), 21599u);
	bridge_core_clock_hz = 8000000u;
	zassert_equal(bridge_timer_prescaler(1000000u), 7u);
}

ZTEST(gd32_bridge_clock_hw, test_24mhz_input_lands_on_the_lower_gain_tuple_and_telemetry_says_so)
{
	reset();
	mock_scn.oscin_hz = 24000000u;
	zassert_true(bridge_clock_try_hxtal());
	zassert_equal(bridge_clock_input_hz, BRIDGE_CLOCK_IN_24P576, "indistinguishable from 24.576");
	/* Software believes 24.576 MHz; the silicon runs 24.000 / 6 * 105 / 2 = 210 MHz, 2.4% LOW
	 * (the safe side).  Nothing in the part can tell the two references apart. */
	zassert_equal(SystemCoreClock, 215040000u, "what software derives from the classification");
	zassert_equal(mock_sim_sysclk_hz(), 210000000u, "what the hardware really runs");
}

/* ---- refusals ---- */

ZTEST(gd32_bridge_clock_hw, test_unsupported_frequency_is_refused_before_the_pll_moves)
{
	reset();
	mock_scn.oscin_hz = 28000000u;
	zassert_false(bridge_clock_try_hxtal());
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_HXTAL_FREQ);
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
	zassert_false(mock_log_has("SCS0"), "SYSCLK never left the IRC8M PLL");
	zassert_false(mock_log_has("PLLEN-"));
	zassert_false(mock_log_has("CKM+"));
	zassert_true(mock_in_order("HXEN+", "T14ON", "T14OFF", "HXEN-", "BPS-", NULL),
	             "stopped after the check: %s",
	             mock_log_text());
	zassert_equal(RCU_CTL & (RCU_CTL_HXTALEN | RCU_CTL_HXTALBPS | RCU_CTL_CKMEN), 0u);
	zassert_equal(RCU_CFG0 & RCU_CFG0_SCSS, RCU_SCSS_PLLP);
	zassert_equal(SystemCoreClock, 216000000u);
	zassert_equal(mock_syscfg_timer14_cfg(), 0u, "check hardware released on the reject path too");
	zassert_equal(RTC_BKP9, 0u);
}

ZTEST(gd32_bridge_clock_hw, test_32khz_free_run_is_never_taken_for_a_clock)
{
	reset();
	mock_scn.oscin_hz = 32768u; /* the shipped SE2 OTP image */
	/* If HXTALSTB did set (it needs ~4096 HXTAL cycles = 125 ms, so it should not),
	 * the count is ~2 and is refused too. */
	mock_scn.stb_delay_us = 0u;
	zassert_false(bridge_clock_try_hxtal());
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_HXTAL_FREQ);

	reset();
	mock_scn.oscin_hz     = 32768u;
	mock_scn.stb_delay_us = 125000u;
	mark();
	zassert_false(bridge_clock_try_hxtal());
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_HXTAL_TIMEOUT);
	zassert_true(since() <= 5300000ull,
	             "bounded by the 5 ms startup budget, got %llu ns",
	             (unsigned long long)since());
}

ZTEST(gd32_bridge_clock_hw, test_se2_off_costs_exactly_the_bounded_startup_wait)
{
	reset();
	mock_scn.oscin_hz = 0u;
	mark();
	zassert_false(bridge_clock_try_hxtal());
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_HXTAL_TIMEOUT);
	zassert_true(
	    since() >= 5000000ull, "it really waited the budget: %llu ns", (unsigned long long)since());
	zassert_true(since() <= 5300000ull, "and no longer: %llu ns", (unsigned long long)since());
	zassert_false(mock_log_has("CKM+"));
	zassert_false(mock_log_has("SCS0"), "PLL untouched");
	zassert_equal(RCU_CTL & (RCU_CTL_HXTALEN | RCU_CTL_HXTALBPS), 0u);
	zassert_equal(RTC_BKP9, 0u);
}

ZTEST(gd32_bridge_clock_hw, test_irc8m_tolerance_is_inside_the_band_but_not_beyond)
{
	reset();
	mock_scn.irc8m_err_permille = 20; /* IRC8M and its PLL 2% fast: fewer counts */
	zassert_true(bridge_clock_try_hxtal());

	reset();
	mock_scn.irc8m_err_permille = -20;
	zassert_true(bridge_clock_try_hxtal());

	reset();
	mock_scn.irc8m_err_permille = 40;
	zassert_false(bridge_clock_try_hxtal(), "4% is refused rather than risk an overclock");
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_HXTAL_FREQ);
}

ZTEST(gd32_bridge_clock_hw, test_pll_refusing_hxtal_rebuilds_the_irc8m_pll)
{
	reset();
	mock_scn.pll_locks_hxtal = false;
	mark();
	zassert_false(bridge_clock_try_hxtal());
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_PLL_FAIL);
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(RCU_PLL & RCU_PLL_PLLSEL, RCU_PLLSRC_IRC8M);
	zassert_equal((RCU_PLL & RCU_PLL_PLLPSC) + 1u, 2u);
	zassert_equal(RCU_CFG0 & RCU_CFG0_SCSS, RCU_SCSS_PLLP, "back on a 216 MHz PLL");
	zassert_equal(RCU_CFG0 & RCU_CFG0_AHBPSC, RCU_AHB_CKSYS_DIV1);
	zassert_equal(SystemCoreClock, 216000000u);
	zassert_equal(RCU_CTL & (RCU_CTL_HXTALEN | RCU_CTL_HXTALBPS | RCU_CTL_CKMEN), 0u);
	zassert_equal(RTC_BKP9, 0u);
}

ZTEST(gd32_bridge_clock_hw,
      test_irc8m_pll_failing_in_fall_back_is_recorded_and_clock_telemetry_follows)
{
	reset();
	mock_scn.pll_locks_hxtal = false;
	mock_scn.pll_locks_irc8m = false; /* nothing locks */
	mark();
	zassert_false(bridge_clock_try_hxtal());
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_IRC8M_PLL_FAIL);
	zassert_equal(RCU_CFG0 & RCU_CFG0_SCSS, RCU_SCSS_IRC8M, "left on the bare IRC8M");
	zassert_equal(SystemCoreClock, 8000000u);
	zassert_equal(bridge_core_clock_hz, 8000000u);
	zassert_false(bridge_core_clock_matches);
	zassert_equal(mock_systick.LOAD, 8000000u / 20u - 1u, "SysTick re-sized for 8 MHz");
	zassert_true(since() <= 23500000ull,
	             "worst bounded chain stays inside 23 ms: %llu ns",
	             (unsigned long long)since());
}

ZTEST(gd32_bridge_clock_hw,
      test_slowest_hxtal_start_plus_dead_plls_stays_inside_the_documented_bound)
{
	reset();
	mock_scn.stb_delay_us    = 4900u; /* HXTAL just inside its budget */
	mock_scn.pll_locks_hxtal = false;
	mock_scn.pll_locks_irc8m = false;
	mark();
	zassert_false(bridge_clock_try_hxtal());
	zassert_true(since() <= 23500000ull, "%llu ns", (unsigned long long)since());
	zassert_true(mock_primask_max_ns <= 23500000u);
}

ZTEST(gd32_bridge_clock_hw, test_a_wedged_switch_steps_ahb_back_up_and_gives_up_in_time)
{
	reset();
	mock_scn.scs_refuses_irc8m = true; /* SCSS never leaves the PLL */
	mark();
	zassert_false(bridge_clock_try_hxtal());
	zassert_true(mock_in_order("AHB8", "AHB9", "SCS0", "AHB8", "AHB0", NULL),
	             "stepped down, then back up in stages: %s",
	             mock_log_text());
	zassert_true(since() <= 23500000ull, "%llu ns", (unsigned long long)since());
	zassert_equal(RCU_CFG0 & RCU_CFG0_AHBPSC, RCU_AHB_CKSYS_DIV1);
}

ZTEST(gd32_bridge_clock_hw, test_ota_trial_does_not_feed_the_bootloader_dog)
{
	reset();
	mock_scn.trial = true;
	zassert_true(bridge_clock_try_hxtal());
	zassert_equal(mock_fwdgt_feeds, 0u, "the trial dog is the confirm deadline");
}

/* ---- Deep-sleep wake ---- */

ZTEST(gd32_bridge_clock_hw, test_pre_deepsleep_takes_the_monitor_off_a_running_hxtal)
{
	reset();
	zassert_true(bridge_clock_try_hxtal());
	mock_log_clear();
	bridge_clock_pre_deepsleep();
	zassert_equal(RCU_CTL & RCU_CTL_CKMEN, 0u);
	zassert_true(mock_log_has("CKM-"));
}

ZTEST(gd32_bridge_clock_hw, test_wake_relock_restarts_hxtal_with_ckmen_cleared_first)
{
	reset();
	zassert_true(bridge_clock_try_hxtal());
	/* No pre_deepsleep: CKMEN stays set through the sleep (the reviewed race). */
	mock_hw_deepsleep();
	zassert_true(RCU_CTL & RCU_CTL_CKMEN);
	mock_log_clear();
	mark();
	bridge_clock_relock_prepare();
	(void)RCU_CTL; /* flush: the last write is only logged by the next access */
	zassert_true(mock_in_order("CKM-",
	                           "HXEN+",
	                           "TRIG:88/a3",
	                           "T14ON",
	                           "T14OFF",
	                           "PLLCFG:2/108/2/H",
	                           "PLLEN+",
	                           "SCS3",
	                           "CKM+",
	                           NULL),
	             "wake order: %s",
	             mock_log_text());
	zassert_false(mock_log_has("AHB8"), "waking from IRC8M: no Vcore step needed");
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_HXTAL);
	zassert_equal(RCU_CFG0 & RCU_CFG0_SCSS, RCU_SCSS_PLLP);
	zassert_true(since() <= 6 * MS_NS,
	             "typical relock is a few ms even at 8 MHz: %llu ns",
	             (unsigned long long)since());
}

ZTEST(gd32_bridge_clock_hw, test_wake_with_se2_gone_is_time_bounded_not_spin_bounded_at_8mhz)
{
	reset();
	zassert_true(bridge_clock_try_hxtal());
	mock_hw_deepsleep();
	mock_scn.oscin_hz   = 0u; /* SE2 switched off during the sleep */
	mock_primask_max_ns = 0u;
	mark();
	mock_set_primask(1u); /* power.c holds PRIMASK across the wake */
	bridge_clock_relock_prepare();
	mock_set_primask(0u);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_HXTAL_TIMEOUT);
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
	zassert_true(since() >= 5000000ull && since() <= 12 * MS_NS,
	             "the old 1e6-spin budget would be ~1.2-1.9 s here; got %llu ns",
	             (unsigned long long)since());
	zassert_true(mock_primask_max_ns <= 12 * MS_NS);
	/* The PLL was re-pointed so power.c's PLLEN replay locks on IRC8M. */
	zassert_equal(RCU_PLL & RCU_PLL_PLLSEL, RCU_PLLSRC_IRC8M);
	bridge_clock_core_update();
	zassert_equal(SystemCoreClock, 216000000u);
}

ZTEST(gd32_bridge_clock_hw, test_wake_path_worst_case_is_bounded)
{
	reset();
	zassert_true(bridge_clock_try_hxtal());
	mock_hw_deepsleep();
	mock_scn.stb_delay_us    = 4900u;
	mock_scn.pll_locks_hxtal = false;
	mock_scn.pll_locks_irc8m = false;
	mark();
	bridge_clock_relock_prepare();
	zassert_true(since() <= 23500000ull, "%llu ns", (unsigned long long)since());
}

/* ---- CKM NMI ---- */

static void healthy_hxtal(void)
{
	reset();
	zassert_true(bridge_clock_try_hxtal());
	bridge_clock_tick();
}

ZTEST(gd32_bridge_clock_hw, test_ckm_nmi_acks_ckmic_before_the_syscfg_flag_then_rebuilds_216)
{
	healthy_hxtal();
	mock_hw_ckm_fire(); /* hardware: SYSCLK to IRC8M, PLL dropped, flag raised */
	zassert_equal(mock_stat_flags(), SYSCFG_STAT_CKMNMIIF);
	zassert_true(bridge_clock_nmi_recover(SYSCFG_STAT_CKMNMIIF));
	zassert_true(mock_in_order("CKM-",
	                           "CKMIC",
	                           "STATW:8",
	                           "HXEN-",
	                           "BPS-",
	                           "PLLCFG:2/108/2/I",
	                           "PLLEN+",
	                           "SCS3",
	                           NULL),
	             "NMI order: %s",
	             mock_log_text());
	zassert_equal(mock_stat_flags(), 0u, "rc_w1 cleared");
	zassert_equal(RCU_CTL & (RCU_CTL_CKMEN | RCU_CTL_HXTALEN | RCU_CTL_HXTALBPS), 0u);
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_CKM_FAILURE);
	zassert_equal(bridge_clock_input_hz, 0u);
	zassert_equal(SystemCoreClock, 216000000u);
	zassert_true(bridge_core_clock_matches);
	zassert_equal(mock_systick.LOAD, 216000000u / 20u - 1u);
}

ZTEST(gd32_bridge_clock_hw, test_second_ckm_nmi_is_a_storm_and_takes_the_fault_path_untouched)
{
	healthy_hxtal();
	mock_hw_ckm_fire();
	zassert_true(bridge_clock_nmi_recover(SYSCFG_STAT_CKMNMIIF));
	mock_set_stat_flags(SYSCFG_STAT_CKMNMIIF); /* it fires again */
	mock_log_clear();
	zassert_false(bridge_clock_nmi_recover(SYSCFG_STAT_CKMNMIIF), "already on IRC8M");
	zassert_equal(strlen(mock_log_text()), 0u, "no register touched: %s", mock_log_text());
	zassert_equal(mock_stat_flags(), SYSCFG_STAT_CKMNMIIF, "flag kept for the fault record");
}

ZTEST(gd32_bridge_clock_hw, test_ckm_nmi_with_hxtal_never_requested_is_not_recoverable)
{
	reset();
	mock_set_stat_flags(SYSCFG_STAT_CKMNMIIF);
	zassert_false(bridge_clock_nmi_recover(SYSCFG_STAT_CKMNMIIF));
	zassert_equal(strlen(mock_log_text()), 0u);
}

ZTEST(gd32_bridge_clock_hw, test_nmi_status_mask_ignores_single_bit_ecc_but_not_real_faults)
{
	static const uint32_t se_bits[]  = { SYSCFG_STAT_SRAM0ECCSEIF,
		                                 SYSCFG_STAT_SRAM1ECCSEIF,
		                                 SYSCFG_STAT_TCMSRAMECCSEIF };
	static const uint32_t bad_bits[] = { SYSCFG_STAT_SRAM0ECCMEIF,
		                                 SYSCFG_STAT_FLASHECCIF,
		                                 SYSCFG_STAT_NMIPINIF,
		                                 SYSCFG_STAT_SRAM1ECCMEIF,
		                                 SYSCFG_STAT_TCMSRAMECCMEIF };
	for (unsigned i = 0; i < sizeof se_bits / sizeof se_bits[0]; i++) {
		healthy_hxtal();
		mock_hw_ckm_fire();
		mock_set_stat_flags(SYSCFG_STAT_CKMNMIIF | se_bits[i]);
		zassert_true(bridge_clock_nmi_recover(SYSCFG_STAT_CKMNMIIF | se_bits[i]),
		             "CKM + single-bit ECC event 0x%x is still a CKM NMI",
		             (unsigned)se_bits[i]);
	}
	for (unsigned i = 0; i < sizeof bad_bits / sizeof bad_bits[0]; i++) {
		healthy_hxtal();
		mock_hw_ckm_fire();
		mock_set_stat_flags(SYSCFG_STAT_CKMNMIIF | bad_bits[i]);
		mock_log_clear();
		zassert_false(bridge_clock_nmi_recover(SYSCFG_STAT_CKMNMIIF | bad_bits[i]),
		              "CKM + 0x%x must keep the record-and-reset policy",
		              (unsigned)bad_bits[i]);
		zassert_equal(strlen(mock_log_text()), 0u, "and leave every flag for the record");
	}
	healthy_hxtal();
	zassert_false(bridge_clock_nmi_recover(SYSCFG_STAT_SRAM0ECCSEIF), "no CKM bit: not ours");
}

/* ---- the request seam ---- */

ZTEST(gd32_bridge_clock_hw,
      test_swd_trigger_variable_and_request_latch_run_the_switch_from_the_tick)
{
	reset();
	bridge_clock_tick();
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M, "no request, no switch");

	bridge_clock_request_hxtal(); /* the ISR-safe seam */
	zassert_equal(bridge_clock_hxtal_request, 1u);
	bridge_clock_tick();
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_HXTAL);
	zassert_equal(bridge_clock_hxtal_request, 0u, "consumed");
	zassert_equal(RTC_BKP9, BRIDGE_CLOCK_ATTEMPT_MAGIC, "healthy-tick clear is the NEXT tick");
	bridge_clock_tick();
	zassert_equal(RTC_BKP9, 0u);

	reset();
	mock_scn.oscin_hz          = 0u;
	bridge_clock_hxtal_request = 1u; /* the bench pokes the variable over SWD */
	bridge_clock_tick();
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_HXTAL_TIMEOUT);
	zassert_equal(bridge_clock_hxtal_request, 0u);
}

ZTEST(gd32_bridge_clock_hw, test_marker_left_by_an_unhealthy_attempt_blocks_the_next_boot_once)
{
	reset();
	RTC_BKP9 = BRIDGE_CLOCK_ATTEMPT_MAGIC; /* the previous attempt never got healthy */
	bridge_clock_init(true);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_PREV_BOOT_UNHEALTHY);
	zassert_equal(RTC_BKP9, 0u);
	mock_log_clear();
	zassert_false(bridge_clock_try_hxtal(), "refused this boot");
	zassert_false(mock_log_has("HXEN+"), "HXTAL not even started");
}

/* ---- live readers ---- */

ZTEST(gd32_bridge_clock_hw, test_live_clock_readers_use_the_measured_oscin_not_the_vendor_8mhz)
{
	reset();
	RCU_PLL               = RCU_PLLSRC_HXTAL | (5u << 0) | (105u << 6) | (0u << 16);
	RCU_CFG0              = RCU_CKSYSSRC_PLLP | RCU_SCSS_PLLP;
	bridge_clock_input_hz = BRIDGE_CLOCK_IN_24P576;
	bridge_clock_core_update();
	zassert_equal(
	    SystemCoreClock, 215040000u, "the vendor SystemCoreClockUpdate() would say 143.36 MHz");

	RCU_CFG0 = RCU_CKSYSSRC_PLLP | RCU_SCSS_PLLP | RCU_AHB_CKSYS_DIV4;
	bridge_clock_core_update();
	zassert_equal(SystemCoreClock, 215040000u / 4u);

	RCU_CFG0 = RCU_CKSYSSRC_PLLP | RCU_SCSS_PLLP | (4u << 10); /* APB1 /2 */
	zassert_equal(bridge_clock_apb1_hz(), 215040000u / 2u);
	RCU_CFG0 = RCU_CKSYSSRC_PLLP | RCU_SCSS_PLLP;
	zassert_equal(bridge_clock_apb1_hz(), 215040000u);

	RCU_CFG0 = RCU_CKSYSSRC_IRC8M | RCU_SCSS_IRC8M;
	bridge_clock_core_update();
	zassert_equal(SystemCoreClock, 8000000u);
}
