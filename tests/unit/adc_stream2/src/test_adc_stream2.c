/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * The vendor-free arithmetic behind CMD_ADC_STREAM_BEGIN2 / _READ2
 * (hal/gd32/adc_stream2.c; docs/protocol-v0.15-design.md section 5):
 *
 *   - READ2 accounting: overrun skip, the GUARD band, the delivered index D,
 *     u32 wrap, the 0xFFFFFFFF discontinuity sentinel;
 *   - the pace timer's REALISED rate (period = floor(tick / rate));
 *   - the conversion-time model (ratio * (sample_cycles + 12.5) / 36 MHz);
 *   - the VREF_MEASURED flag.
 *
 * The hardware half (cursors, DMA registers, copy-out) runs against the
 * register mock in the adc_seq suite.
 */

#include <zephyr/ztest.h>

#include "adc_stream2.h"
#include "protocol.h"

ZTEST_SUITE(gd32_adc_stream2, NULL, NULL, NULL, NULL, NULL);

static adc_read2_plan_t plan(uint32_t backlog, uint16_t depth, uint8_t max, uint32_t d, bool disc)
{
	adc_read2_plan_t p;
	adc_read2_plan(backlog, depth, max, d, disc, false, &p);
	return p;
}

ZTEST(gd32_adc_stream2, test_empty_ring_reports_nothing)
{
	const adc_read2_plan_t p = plan(0u, 512u, 16u, 100u, false);
	zassert_equal(p.got, 0u);
	zassert_equal(p.dropped, 0u);
	zassert_equal(p.first_index, 100u, "first_index is the delivered index");
	zassert_equal(p.next_d, 100u);
	zassert_false(p.sentinel);
}

ZTEST(gd32_adc_stream2, test_normal_read_advances_the_delivered_index)
{
	const adc_read2_plan_t p = plan(10u, 512u, 4u, 100u, false);
	zassert_equal(p.skip, 0u);
	zassert_equal(p.got, 4u, "got = min(max_samples, backlog)");
	zassert_equal(p.remaining, 6u);
	zassert_equal(p.first_index, 100u);
	zassert_equal(p.next_d, 104u, "D = first_index + got");
	zassert_equal(p.dropped, 0u);
}

ZTEST(gd32_adc_stream2, test_short_backlog_returns_what_is_there)
{
	const adc_read2_plan_t p = plan(3u, 512u, 16u, 0u, false);
	zassert_equal(p.got, 3u);
	zassert_equal(p.remaining, 0u);
}

/* The overrun budget is ring_depth - GUARD: exactly that much backlog is not
 * an overrun, one more sample is. */
ZTEST(gd32_adc_stream2, test_guard_band_boundary)
{
	zassert_equal(GD32_BRIDGE_ADC_STREAM2_GUARD, 8u);

	adc_read2_plan_t p = plan(512u - 8u, 512u, 255u, 0u, false);
	zassert_equal(p.skip, 0u, "backlog == depth - GUARD is still lossless");
	zassert_equal(p.dropped, 0u);
	zassert_equal(p.got, 255u);

	p = plan(512u - 8u + 1u, 512u, 255u, 0u, false);
	zassert_equal(p.skip, 1u, "one past the budget skips the single oldest sample");
	zassert_equal(p.dropped, 1u);
	zassert_equal(p.first_index, 1u, "first_index = D + dropped");
}

/* Small rings (W = 16 -> depth 32): the guard band is a quarter of the ring. */
ZTEST(gd32_adc_stream2, test_smallest_ring_budget)
{
	const adc_read2_plan_t p = plan(32u, 32u, 255u, 0u, false);
	zassert_equal(p.skip, 32u - 24u, "keep = 32 - 8 = 24, skip the rest");
	zassert_equal(p.got, 24u);
	zassert_equal(p.remaining, 0u);
}

ZTEST(gd32_adc_stream2, test_overrun_serves_the_freshest_contiguous_samples)
{
	/* Backlog 700 of a 512 ring: keep the newest 504, drop the oldest 196. */
	const adc_read2_plan_t p = plan(700u, 512u, 100u, 1000u, false);
	zassert_equal(p.skip, 196u);
	zassert_equal(p.dropped, 196u, "dropped counts exactly what was skipped");
	zassert_equal(p.first_index, 1196u);
	zassert_equal(p.got, 100u);
	zassert_equal(p.remaining, 404u, "504 kept - 100 delivered");
	zassert_equal(p.next_d, 1296u);
	zassert_false(p.sentinel, "overrun is never a discontinuity and never BUSY");
}

/* The host invariant, over a mixed sequence of reads:
 * first_index(n) == first_index(n-1) + got(n-1) + dropped(n)  (mod 2^32). */
ZTEST(gd32_adc_stream2, test_host_invariant_over_a_read_sequence)
{
	static const uint32_t backlogs[] = { 0u, 5u, 40u, 600u, 12u, 504u, 505u, 3u, 1024u };
	uint32_t              d          = 0xFFFFFF00u; /* start near the u32 wrap */
	uint32_t              prev_first = 0u;
	uint32_t              prev_got   = 0u;
	bool                  have_prev  = false;

	for (unsigned i = 0u; i < sizeof(backlogs) / sizeof(backlogs[0]); i++) {
		const adc_read2_plan_t p = plan(backlogs[i], 1024u, 32u, d, false);
		if (have_prev) {
			zassert_equal(p.first_index,
			              (uint32_t)(prev_first + prev_got + p.dropped),
			              "read %u breaks the first_index invariant",
			              i);
		}
		prev_first = p.first_index;
		prev_got   = p.got;
		have_prev  = true;
		d          = p.next_d;
	}
}

ZTEST(gd32_adc_stream2, test_delivered_index_wraps_mod_2_32)
{
	const adc_read2_plan_t p = plan(40u, 1024u, 16u, 0xFFFFFFF8u, false);
	zassert_equal(p.first_index, 0xFFFFFFF8u);
	zassert_equal(p.next_d, 8u, "D wraps past 2^32");
}

ZTEST(gd32_adc_stream2, test_wrapped_overrun_skip)
{
	const adc_read2_plan_t p = plan(1024u, 1024u, 8u, 0xFFFFFFF0u, false);
	zassert_equal(p.skip, 1024u - (1024u - 8u));
	zassert_equal(p.first_index, 0xFFFFFFF0u + 8u);
}

ZTEST(gd32_adc_stream2, test_discontinuity_flag_answers_the_sentinel)
{
	const adc_read2_plan_t p = plan(300u, 512u, 16u, 777u, true);
	zassert_true(p.sentinel);
	zassert_equal(p.dropped, ADC_STREAM2_DISCONTINUITY);
	zassert_equal(p.dropped, 0xFFFFFFFFu);
	zassert_equal(p.got, 0u);
	zassert_equal(p.first_index, 777u, "first_index = D_prev");
	zassert_equal(p.next_d, 777u, "an unknown-length gap does not move D");
}

/* backlog > 2^31 is the third sentinel cause; exactly 2^31 is still a (huge)
 * overrun, and a backlog a ring or less BELOW zero is a transient undercount
 * (gh#149), which plans an empty read, not a loss. */
ZTEST(gd32_adc_stream2, test_absurd_backlog_is_a_discontinuity)
{
	adc_read2_plan_t p = plan(0x80000001u, 1024u, 16u, 5u, false);
	zassert_true(p.sentinel, "backlog > 2^31");
	zassert_equal(p.dropped, 0xFFFFFFFFu);
	zassert_equal(p.got, 0u);

	p = plan(0x80000000u, 1024u, 16u, 5u, false);
	zassert_false(p.sentinel, "backlog == 2^31 is still skip arithmetic");
	zassert_equal(p.skip, 0x80000000u - (1024u - 8u));
}

ZTEST(gd32_adc_stream2, test_transient_undercount_is_empty_not_lost)
{
	adc_read2_plan_t p = plan(0xFFFFFFFFu, 512u, 16u, 9u, false); /* one sample "negative" */
	zassert_false(p.sentinel);
	zassert_equal(p.got, 0u);
	zassert_equal(p.dropped, 0u);
	zassert_equal(p.next_d, 9u);

	zassert_true(p.undercount, "flagged so the next plan can tell a repeat");

	p = plan((uint32_t)(0u - 512u), 512u, 16u, 9u, false); /* a whole ring "negative" */
	zassert_false(p.sentinel);
	zassert_equal(p.got, 0u);
}

/* The empty read is allowed ONCE: a second consecutive undercount, or one more
 * than a ring below zero, is cursor disagreement -- the sentinel, not an empty
 * read that would hide lost samples. */
ZTEST(gd32_adc_stream2, test_undercount_is_one_shot_and_bounded)
{
	adc_read2_plan_t p;

	adc_read2_plan(0xFFFFFFFFu, 512u, 16u, 9u, false, true, &p);
	zassert_true(p.sentinel, "a second consecutive undercount");
	zassert_equal(p.dropped, 0xFFFFFFFFu);
	zassert_equal(p.next_d, 9u);

	p = plan((uint32_t)(0u - 513u), 512u, 16u, 9u, false);
	zassert_true(p.sentinel, "one past a ring below zero");
	zassert_false(p.undercount);

	/* a healthy read in between resets the one-shot (caller stores plan.undercount) */
	p = plan(4u, 512u, 16u, 9u, false);
	zassert_false(p.undercount);
	zassert_equal(p.got, 4u);
}

/* The ring spans at least ADC_STREAM2_LAP_MIN_US at the REALISED rate, so the
 * prio-3 lap ISR cannot be starved for a whole lap by prio-1/2 work; the granted
 * watermark is ring/2 and may exceed the request. */
static void ring_plan(uint32_t rate, uint16_t w, uint16_t *ring, uint16_t *granted)
{
	uint32_t tick, period;

	adc_stream2_pace(rate, &tick, &period);
	zassert_true(adc_stream2_ring_plan(w, tick, period, ring, granted), "rate %u", (unsigned)rate);
}

ZTEST(gd32_adc_stream2, test_ring_plan_sizes_the_ring_for_a_5ms_lap)
{
	uint16_t ring, granted;

	zassert_equal(ADC_STREAM2_LAP_MIN_US, 5000u);

	ring_plan(1000u, 16u, &ring, &granted); /* 5 samples per 5 ms: 2W wins */
	zassert_equal(ring, 32u);
	zassert_equal(granted, 16u);

	ring_plan(1000u, 256u, &ring, &granted);
	zassert_equal(ring, 512u);
	zassert_equal(granted, 256u);

	ring_plan(100000u, 16u, &ring, &granted); /* 500 samples per 5 ms */
	zassert_equal(ring, 512u, "smallest power of two >= 500");
	zassert_equal(granted, 256u, "granted watermark exceeds the requested 16");

	ring_plan(100000u, 512u, &ring, &granted);
	zassert_equal(ring, 1024u);
	zassert_equal(granted, 512u);

	ring_plan(10000u, 16u, &ring, &granted); /* 50 per 5 ms -> 64 */
	zassert_equal(ring, 64u);
	zassert_equal(granted, 32u);

	ring_plan(6000u, 16u, &ring, &granted); /* realised 6024 Hz: 31 per 5 ms <= 2W */
	zassert_equal(ring, 32u, "no growth beyond 2W");
	ring_plan(6400u, 16u, &ring, &granted); /* realised 6410 Hz: 33 per 5 ms > 2W */
	zassert_equal(ring, 64u);
}

ZTEST(gd32_adc_stream2, test_ring_plan_no_watermark_and_limits)
{
	uint16_t ring, granted;

	ring_plan(100000u, 0u, &ring, &granted);
	zassert_equal(ring, 1024u, "no events: the whole ring (10.24 ms at 100 kHz)");
	zassert_equal(granted, 0u);

	/* 204.8 kHz is where 1024 samples stop covering 5 ms; 100 kHz max never gets there. */
	zassert_false(adc_stream2_ring_plan(16u, 1000000u, 4u, &ring, &granted),
	              "250 kHz: OUT_OF_RANGE");
	zassert_true(adc_stream2_ring_plan(16u, 1000000u, 5u, &ring, &granted),
	             "200 kHz: 1000 samples");
	zassert_equal(ring, 1024u);

	/* a slow stream: the 5 ms floor is below 2W anyway */
	ring_plan(1u, 16u, &ring, &granted);
	zassert_equal(ring, 32u);
}

/* Realised rate = tick_hz / period_ticks exactly. */
ZTEST(gd32_adc_stream2, test_pace_realised_rate)
{
	uint32_t tick, period;

	adc_stream2_pace(1000u, &tick, &period);
	zassert_equal(tick, 1000000u);
	zassert_equal(period, 1000u, "1 kHz is exact");

	adc_stream2_pace(300u, &tick, &period);
	zassert_equal(period, 3333u, "300 Hz truncates to 3333 ticks = 300.03 Hz");
	zassert_equal(tick / period, 300u);

	adc_stream2_pace(16u, &tick, &period);
	zassert_equal(tick, 1000000u, "16 Hz still uses the 1 MHz tick");
	zassert_equal(period, 62500u);

	adc_stream2_pace(15u, &tick, &period);
	zassert_equal(tick, 10000u, "below 16 Hz the tick is 10 kHz");
	zassert_equal(period, 666u);

	adc_stream2_pace(1u, &tick, &period);
	zassert_equal(period, 10000u);

	adc_stream2_pace(100000u, &tick, &period);
	zassert_equal(period, 10u);
}

/* ratio * (sample_cycles + 12.5) / 36 MHz  <  period / tick */
ZTEST(gd32_adc_stream2, test_conversion_time_model)
{
	/* default 240-cycle sample, no oversampling: ~7.0 us, fits 1 kHz and 100 kHz */
	zassert_true(adc_stream2_conv_fits(1u, 240u, 1000u, 1000000u));
	zassert_true(adc_stream2_conv_fits(1u, 240u, 10u, 1000000u), "7.0 us < 10 us");

	/* the widest legal setting does not fit 1 kHz (4.6 ms > 1 ms) */
	zassert_false(adc_stream2_conv_fits(256u, 638u, 1000u, 1000000u));

	/* ... but fits a 1 Hz stream's 10 kHz-tick period of 10000 ticks = 1 s */
	zassert_true(adc_stream2_conv_fits(256u, 638u, 10000u, 10000u));
}

/* 256 * (2 + 12.5) = 3712 ADCCK = 103.11 us: 103 ticks of 1 us is too short,
 * 104 fits.  8 * (10 + 12.5) = 180 ADCCK = exactly 5 us: the check is
 * strict (>= period is refused), so 5 ticks is rejected and 6 fits. */
ZTEST(gd32_adc_stream2, test_conversion_time_boundaries)
{
	zassert_false(adc_stream2_conv_fits(256u, 2u, 103u, 1000000u));
	zassert_true(adc_stream2_conv_fits(256u, 2u, 104u, 1000000u));

	zassert_false(adc_stream2_conv_fits(8u, 10u, 5u, 1000000u), "equal is refused");
	zassert_true(adc_stream2_conv_fits(8u, 10u, 6u, 1000000u));
}

/* VREF_MEASURED: code != 0 and the derived reference inside [1700, 1900] mV.
 * The fallback 1800 does not count, even though 1800 is inside the window. */
ZTEST(gd32_adc_stream2, test_vref_measured_flag)
{
	zassert_false(adc_vref_is_measured(0u), "never measured / EOC timeout");
	zassert_true(adc_vref_is_measured(2730u), "healthy 1800 mV measurement");
	zassert_true(adc_vref_is_measured(2891u), "1700 mV edge");
	zassert_false(adc_vref_is_measured(2892u), "1699 mV: outside");
	zassert_true(adc_vref_is_measured(2586u), "1900 mV edge");
	zassert_false(adc_vref_is_measured(2585u), "1901 mV: outside");
	zassert_false(adc_vref_is_measured(2000u),
	              "an out-of-window code falls back to 1800 -- not measured");
}
