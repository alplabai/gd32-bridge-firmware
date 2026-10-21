/* SPDX-License-Identifier: Apache-2.0
 *
 * Host tests for hal/gd32/clock_source.c: HXTAL-bypass / IRC8M-fallback
 * sequencing, the frequency classifier and the CKM NMI handshake, driven
 * through a fake bridge_clock_ops_t.  The fake records WHAT the sequencer
 * calls and in which order; it deliberately does not "help" (it never clears
 * a bit on behalf of an op), so a mis-ordered or missing call shows.  The
 * register-level behaviour of the real ops is tests/unit/clock_hw.
 */

#include <zephyr/ztest.h>

#include <string.h>

#include "clock_source.h"

#define LOG_MAX 40

#define COUNTS_8MHZ(window_us) (250000u * (window_us) / 1000000u)

static struct {
	bool                  stable;       /* HXTALSTB would set (SE2 present) */
	uint32_t              counts;       /* what the HXTAL/32 counter returns */
	bool                  pll_hxtal_ok; /* PLL locks on HXTAL */
	bool                  pll_irc8m_ok; /* the IRC8M PLL relocks */
	uint32_t              marker;
	char                  log[LOG_MAX];
	int                   n;
	uint32_t              window_seen;
	uint32_t              pll_in_hz;
	bridge_clock_source_t source_when_ckm_armed;
	bridge_clock_source_t source_when_nmi_hook;
	int                   nmi_result;
	void (*hook_pll_hxtal)(void); /* NMI injection points */
	void (*hook_ckm_enable)(void);
} f;

static const bridge_clock_ops_t ops;

static void fake_reset(void)
{
	memset(&f, 0, sizeof(f));
	f.stable              = true;
	f.counts              = COUNTS_8MHZ(BRIDGE_CLOCK_FREQ_WINDOW_US);
	f.pll_hxtal_ok        = true;
	f.pll_irc8m_ok        = true;
	f.nmi_result          = -1;
	bridge_clock_source   = BRIDGE_CLOCK_SRC_IRC8M;
	bridge_clock_fallback = BRIDGE_CLOCK_FB_NONE;
	bridge_clock_input_hz = 0u;
	/* A boot also clears any blocked / in-flight state of a previous case. */
	(void)bridge_clock_boot(&ops, false);
	bridge_clock_fallback = BRIDGE_CLOCK_FB_NONE;
	f.n                   = 0;
	memset(f.log, 0, sizeof(f.log));
}

static void rec(char c)
{
	if (f.n < LOG_MAX - 1) f.log[f.n++] = c;
}

static void op_bypass(void)
{
	rec('B');
}
static void op_enable(void)
{
	rec('E');
}
static bool op_wait_stable(void)
{
	rec('S');
	return f.stable;
}
static uint32_t op_div32(uint32_t window_us)
{
	rec('F');
	f.window_seen = window_us;
	return f.counts;
}
static void op_stop(void)
{
	rec('X');
}
static bool op_pll(bridge_clock_source_t src, uint32_t in_hz)
{
	rec(src == BRIDGE_CLOCK_SRC_HXTAL ? 'H' : 'I');
	if (src == BRIDGE_CLOCK_SRC_HXTAL) {
		f.pll_in_hz = in_hz;
		if (f.hook_pll_hxtal) f.hook_pll_hxtal();
		return f.pll_hxtal_ok;
	}
	return f.pll_irc8m_ok;
}
static void op_ckm(void)
{
	rec('K');
	f.source_when_ckm_armed = bridge_clock_source;
	if (f.hook_ckm_enable) f.hook_ckm_enable();
}
static void op_ack(void)
{
	rec('A');
}
static uint32_t op_mget(void)
{
	return f.marker;
}
static void op_mset(uint32_t v)
{
	f.marker = v;
}

static const bridge_clock_ops_t ops = {
	.hxtal_bypass_enable = op_bypass,
	.hxtal_enable        = op_enable,
	.hxtal_wait_stable   = op_wait_stable,
	.hxtal_div32_count   = op_div32,
	.hxtal_stop          = op_stop,
	.pll_select          = op_pll,
	.ckm_enable          = op_ckm,
	.ckm_ack             = op_ack,
	.marker_get          = op_mget,
	.marker_set          = op_mset,
};

static void nmi_now(void)
{
	f.source_when_nmi_hook = bridge_clock_source;
	f.nmi_result           = bridge_clock_on_ckm_nmi(&ops) ? 1 : 0;
}

ZTEST_SUITE(gd32_bridge_clock_source, NULL, NULL, NULL, NULL, NULL);

/* ---- classifier ---------------------------------------------------- */

static uint32_t classify_hz(uint32_t oscin_hz, int permille_error)
{
	const uint32_t w = BRIDGE_CLOCK_FREQ_WINDOW_US;
	/* counts = (oscin / 32) * window, scaled by the IRC8M-timebase error. */
	const uint64_t c = (uint64_t)(oscin_hz / 32u) * w / 1000000u;
	const uint64_t e = c * (uint64_t)(1000 + permille_error) / 1000u;
	return bridge_clock_classify_div32((uint32_t)e, w);
}

ZTEST(gd32_bridge_clock_source, test_each_reference_is_recognised)
{
	zassert_equal(classify_hz(8000000u, 0), BRIDGE_CLOCK_IN_8MHZ);
	zassert_equal(classify_hz(12000000u, 0), BRIDGE_CLOCK_IN_12MHZ);
	zassert_equal(classify_hz(16000000u, 0), BRIDGE_CLOCK_IN_16MHZ);
	zassert_equal(classify_hz(20000000u, 0), BRIDGE_CLOCK_IN_20MHZ);
	zassert_equal(classify_hz(24576000u, 0), BRIDGE_CLOCK_IN_24P576);
}

ZTEST(gd32_bridge_clock_source, test_other_references_have_a_three_percent_band)
{
	zassert_equal(classify_hz(8000000u, 29), BRIDGE_CLOCK_IN_8MHZ, "+2.9% (slow IRC8M)");
	zassert_equal(classify_hz(8000000u, -29), BRIDGE_CLOCK_IN_8MHZ, "-2.9%");
	zassert_equal(classify_hz(8000000u, 40), 0u, "+4% is outside the band");
	zassert_equal(classify_hz(8000000u, -40), 0u);
	zassert_equal(classify_hz(16000000u, 29), BRIDGE_CLOCK_IN_16MHZ);
	zassert_equal(classify_hz(16000000u, 40), 0u);
}

ZTEST(gd32_bridge_clock_source, test_24p576_band_is_plus_minus_one_percent)
{
	zassert_equal(classify_hz(24576000u, -9), BRIDGE_CLOCK_IN_24P576, "-0.9% (fast IRC8M)");
	zassert_equal(classify_hz(24576000u, 9), BRIDGE_CLOCK_IN_24P576, "+0.9% (slow IRC8M)");
	zassert_equal(classify_hz(24576000u, -12), 0u, "-1.2% refused");
	zassert_equal(classify_hz(24576000u, 12), 0u, "+1.2% refused");
}

ZTEST(gd32_bridge_clock_source, test_unsupported_and_dead_clocks_are_refused)
{
	zassert_equal(classify_hz(28000000u, 0), 0u);
	zassert_equal(classify_hz(6000000u, 0), 0u);
	zassert_equal(classify_hz(32768u, 0), 0u, "SE2 free-run 32.768 kHz: ~2 counts");
	zassert_equal(classify_hz(10000000u, 0), 0u);
	zassert_equal(classify_hz(48000000u, 0), 0u);
	zassert_equal(bridge_clock_classify_div32(0u, BRIDGE_CLOCK_FREQ_WINDOW_US), 0u, "no edges");
	zassert_equal(bridge_clock_classify_div32(500u, 0u), 0u, "zero window");
}

ZTEST(gd32_bridge_clock_source, test_no_24mhz_entry_and_25mhz_is_refused_with_a_true_timebase)
{
	zassert_equal(classify_hz(24000000u, 0), 0u, "24.000 MHz is not a supported source (-2.3%)");
	zassert_true(bridge_clock_ref_for(24000000u) == NULL);
	zassert_equal(classify_hz(25000000u, 0), 0u, "25 MHz is 1.7% above 24.576: outside +-1%");
	zassert_equal(classify_hz(25000000u, 10), 0u);
	zassert_true(bridge_clock_ref_for(25000000u) == NULL, "no tuple");
}

/* 25 MHz is +1.7% on the count: it needs an IRC8M more than 0.7% FAST (counts
 * low by that much) to land inside +-1%.  25 MHz must never be fed (SE2 is only
 * programmed by the alp-sdk U-Boot fixup). */
ZTEST(gd32_bridge_clock_source, test_25mhz_aliases_only_when_the_irc8m_is_fast_by_over_0p7_percent)
{
	zassert_equal(classify_hz(25000000u, 0), 0u);
	zassert_equal(classify_hz(25000000u, -5), 0u, "0.5% fast: 25 MHz reads +1.2%, refused");
	zassert_equal(classify_hz(25000000u, -8), BRIDGE_CLOCK_IN_24P576, "0.8% fast: +0.9%");
}

ZTEST(gd32_bridge_clock_source, test_bands_are_disjoint_and_ordered)
{
	/* Scan every HXTAL/32 count the window can produce: a match must never go
	 * back to a lower reference, and every reference band must be one run. */
	uint32_t last = 0u;
	for (uint32_t c = 0u; c <= 4000u; c++) {
		const uint32_t r = bridge_clock_classify_div32(c, BRIDGE_CLOCK_FREQ_WINDOW_US);
		if (r == 0u) continue;
		zassert_true(r >= last, "bands interleave at %u counts", (unsigned)c);
		last = r;
	}
	zassert_equal(last, BRIDGE_CLOCK_IN_24P576);
	/* The closest pair: 20 MHz upper edge vs 24.576 MHz lower edge cannot meet. */
	zassert_true(20000000ull * 1030u / 1000u < 24576000ull * 990u / 1000u);
}

ZTEST(gd32_bridge_clock_source, test_table_tuples)
{
	static const struct {
		uint32_t in, psc, n, sys;
	} want[] = {
		{ 8000000u, 2u, 108u, 216000000u },  { 12000000u, 3u, 108u, 216000000u },
		{ 16000000u, 4u, 108u, 216000000u }, { 20000000u, 5u, 108u, 216000000u },
		{ 24576000u, 6u, 105u, 215040000u },
	};
	for (unsigned i = 0; i < sizeof want / sizeof want[0]; i++) {
		const bridge_clock_ref_t *r = bridge_clock_ref_for(want[i].in);
		zassert_true(r != NULL);
		zassert_equal(r->pll_psc, want[i].psc);
		zassert_equal(r->pll_n, want[i].n);
		zassert_equal(r->sysclk_hz, want[i].sys);
	}
	zassert_true(bridge_clock_ref_for(25000000u) == NULL);
	zassert_true(bridge_clock_ref_for(0u) == NULL);
}

/* ---- boot / attempt ------------------------------------------------ */

ZTEST(gd32_bridge_clock_source, test_boot_default_stays_on_irc8m_and_touches_nothing)
{
	fake_reset();
	zassert_equal(bridge_clock_boot(&ops, false), BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_NOT_REQUESTED);
	zassert_equal(f.n, 0, "no HXTAL op at boot unless asked");
}

ZTEST(gd32_bridge_clock_source, test_attempt_ok_runs_pll_from_hxtal_in_order)
{
	fake_reset();
	zassert_true(bridge_clock_attempt_hxtal(&ops));
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_HXTAL);
	zassert_equal(bridge_clock_input_hz, BRIDGE_CLOCK_IN_8MHZ);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_NONE);
	zassert_equal(f.pll_in_hz, BRIDGE_CLOCK_IN_8MHZ, "PLL tuple chosen from the measurement");
	zassert_equal(f.window_seen, BRIDGE_CLOCK_FREQ_WINDOW_US);
	zassert_mem_equal(f.log, "BESFHK", 6, "bypass, enable, wait, measure, PLL, then CKM");
	zassert_equal(f.source_when_ckm_armed,
	              BRIDGE_CLOCK_SRC_HXTAL,
	              "the state is written BEFORE the monitor is armed");
	zassert_equal(f.marker, BRIDGE_CLOCK_ATTEMPT_MAGIC, "held until healthy");
	bridge_clock_mark_healthy(&ops);
	zassert_equal(f.marker, 0u);
	zassert_true(bridge_clock_attempt_hxtal(&ops), "already on HXTAL: success, no work");
	zassert_equal(f.n, 6);
}

ZTEST(gd32_bridge_clock_source, test_pll_tuple_follows_the_measured_frequency)
{
	fake_reset();
	f.counts = 1536u; /* 24.576 MHz: 768 kHz over 2 ms */
	zassert_true(bridge_clock_attempt_hxtal(&ops));
	zassert_equal(f.pll_in_hz, BRIDGE_CLOCK_IN_24P576);
	zassert_equal(bridge_clock_input_hz, BRIDGE_CLOCK_IN_24P576);

	fake_reset();
	f.counts = COUNTS_8MHZ(BRIDGE_CLOCK_FREQ_WINDOW_US) * 2u; /* 16 MHz */
	zassert_true(bridge_clock_attempt_hxtal(&ops));
	zassert_equal(f.pll_in_hz, BRIDGE_CLOCK_IN_16MHZ);
}

ZTEST(gd32_bridge_clock_source, test_startup_timeout_falls_back_to_irc8m)
{
	fake_reset();
	f.stable = false; /* SE2 off, or the 32.768 kHz free-run */
	zassert_false(bridge_clock_attempt_hxtal(&ops));
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_HXTAL_TIMEOUT);
	zassert_equal(bridge_clock_input_hz, 0u);
	zassert_mem_equal(f.log, "BESX", 4, "stopped, PLL never touched");
	zassert_equal(f.n, 4);
	zassert_equal(f.marker, 0u);
}

ZTEST(gd32_bridge_clock_source, test_wrong_frequency_is_refused_before_the_pll_moves)
{
	fake_reset();
	f.counts = 1750u; /* 28 MHz: no table entry within the band */
	zassert_false(bridge_clock_attempt_hxtal(&ops));
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_HXTAL_FREQ);
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
	zassert_mem_equal(f.log, "BESFX", 5, "no PLL op at all: the IRC8M PLL keeps running");
	zassert_equal(f.n, 5);
	zassert_equal(f.marker, 0u);
	zassert_equal(bridge_clock_input_hz, 0u);
}

ZTEST(gd32_bridge_clock_source, test_pll_failure_rebuilds_the_irc8m_pll)
{
	fake_reset();
	f.pll_hxtal_ok = false;
	zassert_false(bridge_clock_attempt_hxtal(&ops));
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_PLL_FAIL);
	zassert_mem_equal(f.log, "BESFHXI", 7);
	zassert_equal(f.marker, 0u);
}

ZTEST(gd32_bridge_clock_source, test_irc8m_pll_failing_inside_fall_back_is_recorded)
{
	fake_reset();
	f.pll_hxtal_ok = false;
	f.pll_irc8m_ok = false;
	zassert_false(bridge_clock_attempt_hxtal(&ops));
	zassert_equal(bridge_clock_fallback,
	              BRIDGE_CLOCK_FB_IRC8M_PLL_FAIL,
	              "the worse state is reported over the original cause");
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
}

ZTEST(gd32_bridge_clock_source, test_deepsleep_relock_failure_repoints_the_pll)
{
	fake_reset();
	zassert_true(bridge_clock_attempt_hxtal(&ops));
	f.n      = 0;
	f.stable = false;
	zassert_false(bridge_clock_external_start(&ops, true));
	zassert_mem_equal(f.log, "BESXI", 5, "PLL was on HXTAL: it must be re-pointed");
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_HXTAL_TIMEOUT);
}

ZTEST(gd32_bridge_clock_source, test_previous_unhealthy_attempt_blocks_hxtal_for_one_boot)
{
	fake_reset();
	f.marker = BRIDGE_CLOCK_ATTEMPT_MAGIC;
	zassert_equal(bridge_clock_boot(&ops, true), BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_PREV_BOOT_UNHEALTHY);
	zassert_equal(f.marker, 0u, "one boot only");
	zassert_equal(f.n, 1, "only the stop; HXTAL not even started");
	zassert_false(bridge_clock_attempt_hxtal(&ops), "a host request this boot is refused too");
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_PREV_BOOT_UNHEALTHY);
	zassert_equal(f.n, 1);

	zassert_equal(bridge_clock_boot(&ops, false), BRIDGE_CLOCK_SRC_IRC8M, "next boot");
	zassert_true(bridge_clock_attempt_hxtal(&ops), "allowed again");
}

/* ---- CKM NMI handshake --------------------------------------------- */

ZTEST(gd32_bridge_clock_source, test_nmi_after_boot_acks_then_falls_back)
{
	fake_reset();
	zassert_true(bridge_clock_attempt_hxtal(&ops));
	bridge_clock_mark_healthy(&ops);
	f.n = 0;
	zassert_true(bridge_clock_on_ckm_nmi(&ops));
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_CKM_FAILURE);
	zassert_equal(bridge_clock_input_hz, 0u);
	zassert_mem_equal(f.log, "AXI", 3, "acknowledge (CKMEN off, CKMIC, STAT) first, then recover");
}

ZTEST(gd32_bridge_clock_source, test_second_ckm_nmi_while_on_irc8m_is_a_storm_and_not_handled)
{
	fake_reset();
	zassert_true(bridge_clock_attempt_hxtal(&ops));
	zassert_true(bridge_clock_on_ckm_nmi(&ops));
	f.n = 0;
	zassert_false(bridge_clock_on_ckm_nmi(&ops), "caller must take the fault path");
	zassert_equal(f.n, 0, "flags left alone so the fault record still shows CKMNMIIF");
}

ZTEST(gd32_bridge_clock_source, test_ckm_nmi_with_nothing_armed_is_not_handled)
{
	fake_reset();
	zassert_false(bridge_clock_on_ckm_nmi(&ops));
	zassert_equal(f.n, 0);
}

ZTEST(gd32_bridge_clock_source, test_nmi_inside_the_sequence_only_records_and_the_thread_falls_back)
{
	fake_reset();
	f.hook_pll_hxtal = nmi_now; /* NMI lands while the PLL switch is in flight */
	zassert_false(bridge_clock_attempt_hxtal(&ops), "the attempt is reported failed");
	zassert_equal(f.nmi_result, 1, "acknowledged");
	zassert_equal(
	    f.source_when_nmi_hook, BRIDGE_CLOCK_SRC_IRC8M, "the NMI saw no HXTAL state to overwrite");
	/* The NMI did not stop HXTAL or touch the PLL: A only, then the thread's own K, X, I. */
	zassert_mem_equal(f.log, "BESFHAKXI", 9);
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_CKM_FAILURE);
	zassert_equal(f.marker, 0u);
}

ZTEST(gd32_bridge_clock_source,
      test_nmi_between_ckm_enable_and_the_state_write_finds_hxtal_recorded)
{
	fake_reset();
	f.hook_ckm_enable = nmi_now; /* the earliest the monitor can fire */
	zassert_false(bridge_clock_attempt_hxtal(&ops));
	zassert_equal(f.source_when_nmi_hook,
	              BRIDGE_CLOCK_SRC_HXTAL,
	              "state written before ckm_enable: no window where the NMI sees IRC8M");
	zassert_equal(f.nmi_result, 1, "so it is not mistaken for a storm");
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_CKM_FAILURE);
	zassert_mem_equal(f.log, "BESFHKAXI", 9);
}

ZTEST(gd32_bridge_clock_source, test_nmi_after_the_sequence_ends_recovers_in_nmi_context)
{
	fake_reset();
	zassert_true(bridge_clock_attempt_hxtal(&ops));
	f.n = 0;
	nmi_now();
	zassert_equal(f.nmi_result, 1);
	zassert_mem_equal(f.log, "AXI", 3);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_CKM_FAILURE);
}
