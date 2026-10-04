/* SPDX-License-Identifier: Apache-2.0
 *
 * Host tests for hal/gd32/clock_source.c: HXTAL-bypass / IRC8M-fallback
 * sequencing, driven through a fake bridge_clock_ops_t that models the RCU
 * bits the real ops touch (HXTALEN, HXTALBPS, HXTALSTB, CKMEN, PLL source).
 */

#include <zephyr/ztest.h>

#include <string.h>

#include "clock_source.h"

#define LOG_MAX 32

static struct {
	bool     en, bps, ckm;
	bool     stable_when_enabled; /* SE2 present: HXTALSTB follows HXTALEN */
	bool     pll_hxtal_ok;        /* PLL locks on HXTAL */
	bool     pll_on_hxtal;
	uint32_t marker;
	int      stable_polls;
	int      n;
	char     log[LOG_MAX];
	bool     bps_set_with_en_clear; /* HXTALBPS was written while HXTALEN was clear */
	bool     en_set_with_bps_set;   /* HXTALEN was written while HXTALBPS was set */
} f;

static void fake_reset(void)
{
	memset(&f, 0, sizeof(f));
	f.stable_when_enabled = true;
	f.pll_hxtal_ok        = true;
	bridge_clock_source   = BRIDGE_CLOCK_SRC_IRC8M;
	bridge_clock_fallback = BRIDGE_CLOCK_FB_NONE;
}

static void rec(char c)
{
	if (f.n < LOG_MAX) f.log[f.n++] = c;
}

static void op_bypass(void)
{
	f.en                    = false; /* real op clears HXTALEN first */
	f.bps                   = true;
	f.bps_set_with_en_clear = !f.en;
	rec('B');
}
static void op_enable(void)
{
	f.en_set_with_bps_set = f.bps;
	f.en                  = true;
	rec('E');
}
static bool op_stable(void)
{
	f.stable_polls++;
	return f.en && f.stable_when_enabled;
}
static void op_stop(void)
{
	f.ckm = f.en = f.bps = false;
	rec('X');
}
static bool op_pll(bridge_clock_source_t src)
{
	rec(src == BRIDGE_CLOCK_SRC_HXTAL ? 'H' : 'I');
	if (src == BRIDGE_CLOCK_SRC_HXTAL && !f.pll_hxtal_ok) return false;
	f.pll_on_hxtal = (src == BRIDGE_CLOCK_SRC_HXTAL);
	return true;
}
static void op_ckm(void)
{
	f.ckm = true;
	rec('K');
}
static uint32_t op_mget(void)
{
	return f.marker;
}
static void op_mset(uint32_t v)
{
	f.marker = v;
}

static const bridge_clock_ops_t ops = { op_bypass, op_enable, op_stable, op_stop,
	                                    op_pll,    op_ckm,    op_mget,   op_mset };

ZTEST_SUITE(gd32_bridge_clock_source, NULL, NULL, NULL, NULL, NULL);

ZTEST(gd32_bridge_clock_source, test_hxtal_ok_runs_pll_from_hxtal)
{
	fake_reset();
	zassert_equal(bridge_clock_select(&ops, 100u), BRIDGE_CLOCK_SRC_HXTAL);
	zassert_true(f.pll_on_hxtal);
	zassert_true(f.ckm, "clock monitor armed");
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_NONE);
	zassert_equal(f.marker, BRIDGE_CLOCK_ATTEMPT_MAGIC, "attempt marker held until healthy");
	bridge_clock_mark_healthy(&ops);
	zassert_equal(f.marker, 0u);
	zassert_mem_equal(f.log, "BEHK", 4, "bypass, enable, PLL on HXTAL, then CKM");
}

ZTEST(gd32_bridge_clock_source, test_bypass_set_before_enable_with_hxtalen_clear)
{
	fake_reset();
	(void)bridge_clock_select(&ops, 100u);
	zassert_true(f.bps_set_with_en_clear);
	zassert_true(f.en_set_with_bps_set);
	zassert_true(f.log[0] == 'B' && f.log[1] == 'E');
}

ZTEST(gd32_bridge_clock_source, test_stab_timeout_falls_back_to_irc8m_bounded)
{
	fake_reset();
	f.stable_when_enabled = false; /* SE2 off */
	zassert_equal(bridge_clock_select(&ops, 50u), BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(f.stable_polls, 50, "poll loop is bounded by the spin budget");
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_HXTAL_TIMEOUT);
	zassert_false(f.en || f.bps || f.ckm, "HXTAL stopped");
	zassert_false(f.pll_on_hxtal);
	zassert_equal(f.marker, 0u);
	zassert_true(memchr(f.log, 'H', (size_t)f.n) == NULL, "PLL never touched at boot");
}

ZTEST(gd32_bridge_clock_source, test_pll_failure_falls_back_to_irc8m_pll)
{
	fake_reset();
	f.pll_hxtal_ok = false;
	zassert_equal(bridge_clock_select(&ops, 100u), BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_PLL_FAIL);
	zassert_false(f.pll_on_hxtal);
	zassert_false(f.en);
	zassert_equal(f.marker, 0u);
}

ZTEST(gd32_bridge_clock_source, test_failure_nmi_switches_to_irc8m)
{
	fake_reset();
	(void)bridge_clock_select(&ops, 100u);
	bridge_clock_mark_healthy(&ops);
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_HXTAL);

	f.n = 0; /* external clock dies; CKM NMI */
	bridge_clock_on_ckm_failure(&ops);
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_CKM_FAILURE);
	zassert_false(f.pll_on_hxtal, "PLL re-pointed at IRC8M");
	zassert_false(f.ckm, "monitor disarmed so it cannot re-fire");
	zassert_mem_equal(f.log, "XI", 2);
}

ZTEST(gd32_bridge_clock_source, test_unhealthy_previous_boot_skips_hxtal)
{
	fake_reset();
	f.marker = BRIDGE_CLOCK_ATTEMPT_MAGIC;
	zassert_equal(bridge_clock_select(&ops, 100u), BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_PREV_BOOT_UNHEALTHY);
	zassert_equal(f.stable_polls, 0, "HXTAL not even started");
	zassert_equal(f.marker, 0u, "one boot only, then retried");
}

ZTEST(gd32_bridge_clock_source, test_deepsleep_relock_failure_repoints_pll)
{
	fake_reset();
	(void)bridge_clock_select(&ops, 100u);
	f.en = f.bps          = false; /* Deep-sleep stopped HXTAL */
	f.stable_when_enabled = false;
	zassert_false(bridge_clock_external_start(&ops, 10u, true));
	zassert_false(f.pll_on_hxtal);
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
}
