/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host-only regression for #69/#70: bridge_hw_adc_dsp_chain_bind() used to
 * validate a chain's SHAPE (ordering, blob well-formedness, contiguity) but
 * not the P1 runtime's CAPABILITY limits, which lived only in the FAC/FFT
 * config functions the pump calls -- so a chain those functions refuse
 * (multi-stage FIR/IIR, a cascaded IIR, a FIR ahead of a WINDOW+FFT
 * terminal, a second FFT-terminal bind against the single FFT block) bound
 * with BRIDGE_HW_OK and then starved silently forever.
 *
 * This links hal/gd32/adc_dsp_chain.c -- the chain pool + chain_bind split
 * out specifically because it is VENDOR-HEADER-FREE (touches no GD32
 * register) -- directly against a TEST-LOCAL definition of adc_streams[].
 * That is a feature, not a shortcut: driving `in_use` / `dsp_bound` /
 * `dsp_terminal` on the array by hand is exactly what is needed to reach
 * #70's double-bind and FAC/FFT contention paths without a running stream.
 *
 * What this suite CANNOT cover: adc_dsp_fac_config / adc_dsp_pump_stream /
 * adc_dsp_fft_config / adc_dsp_pump_fft stay in hal/gd32/adc_stream.c
 * because they touch RCU/FAC/FFT registers -- the pump interaction and the
 * `pump_raw_read` ownership-flip fix need a bench, not a host build.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "adc_dsp_chain.h"
#include "bridge_hw.h"
#include "gd32_common.h"

/* Test-local definition of the extern adc_streams[] gd32_common.h declares
 * -- the real definition lives in hal/gd32/adc_stream.c, which this suite
 * deliberately does NOT link (it pulls "gd32g5x3.h").  Tests set in_use /
 * dsp_bound / dsp_terminal directly rather than going through
 * bridge_hw_adc_stream_begin, which lives in the unlinked file. */
adc_stream_state_t adc_streams[BRIDGE_ADC_STREAM_COUNT];

/* ---- fixtures --------------------------------------------------------- */

/* Return every chain slot to the pool and zero the stream array, so each
 * case starts from a clean pool + clean streams regardless of run order. */
static void reset_all(void)
{
	for (uint8_t i = 0u; i < BRIDGE_DSP_MAX_CHAINS; ++i) {
		adc_dsp_chain_release(i);
	}
	memset(adc_streams, 0, sizeof(adc_streams));
}

/* Mark stream `sid` as a running, unbound ADC stream -- the precondition
 * chain_bind checks (`s->in_use` true, `s->dsp_bound` false) without
 * needing the real DMA/timer bring-up in bridge_hw_adc_stream_begin. */
static void stream_running(uint8_t sid)
{
	adc_streams[sid].in_use    = true;
	adc_streams[sid].dsp_bound = false;
}

static uint8_t open_chain(void)
{
	uint8_t chain_id = 0xFFu;

	zassert_equal(bridge_hw_adc_dsp_chain_open(&chain_id), BRIDGE_HW_OK, "chain_open failed");
	return chain_id;
}

static void
push_stage(uint8_t chain_id, uint8_t stage_index, uint8_t kind, const uint8_t *data, uint16_t len)
{
	int rv = bridge_hw_adc_dsp_stage_push(chain_id, stage_index, kind, 0u, len, data, len);
	zassert_equal(rv,
	              BRIDGE_HW_OK,
	              "stage_push(chain=%u idx=%u kind=%u) failed, got %d",
	              chain_id,
	              stage_index,
	              kind,
	              rv);
}

/* ---- per-kind blob builders (docs/gd32-bridge-protocol.md §3.x layout,
 * mirrored in hal/gd32/adc_dsp_chain.h's header comment) ---------------- */

/* FIR: format:u8 n_taps:u8 rsvd:u16 taps[n_taps*4].  `out` must be at
 * least 4 + n_taps*4 bytes; returns the blob's total size. */
static uint16_t fir_blob(uint8_t *out, uint8_t n_taps)
{
	out[0] = 1u; /* fmt = Q31 */
	out[1] = n_taps;
	out[2] = 0u;
	out[3] = 0u;
	for (uint8_t k = 0u; k < n_taps; ++k) {
		out[4u + k * 4u + 0u] = 0x00u;
		out[4u + k * 4u + 1u] = 0x00u;
		out[4u + k * 4u + 2u] = 0x00u;
		out[4u + k * 4u + 3u] = 0x10u; /* arbitrary nonzero Q31 tap */
	}
	return (uint16_t)(4u + (uint16_t)n_taps * 4u);
}

/* IIR: format:u8 n_sections:u8 rsvd:u16 coeffs[n_sec*5*4]. */
static uint16_t iir_blob(uint8_t *out, uint8_t n_sections)
{
	out[0] = 1u; /* fmt = Q31 */
	out[1] = n_sections;
	out[2] = 0u;
	out[3] = 0u;
	for (uint16_t k = 0u; k < (uint16_t)n_sections * 5u; ++k) {
		out[4u + k * 4u + 0u] = 0x00u;
		out[4u + k * 4u + 1u] = 0x00u;
		out[4u + k * 4u + 2u] = 0x00u;
		out[4u + k * 4u + 3u] = 0x10u;
	}
	return (uint16_t)(4u + (uint16_t)n_sections * 5u * 4u);
}

/* WINDOW: shape:u8 rsvd[3]. */
static uint16_t window_blob(uint8_t *out, uint8_t shape)
{
	out[0] = shape;
	out[1] = 0u;
	out[2] = 0u;
	out[3] = 0u;
	return 4u;
}

/* FFT: n_points:u16 out_fmt:u8 rsvd:u8. */
static uint16_t fft_blob(uint8_t *out, uint16_t n_points, uint8_t out_fmt)
{
	out[0] = (uint8_t)(n_points & 0xFFu);
	out[1] = (uint8_t)(n_points >> 8);
	out[2] = out_fmt;
	out[3] = 0u;
	return 4u;
}

ZTEST_SUITE(gd32_bridge_adc_dsp, NULL, NULL, NULL, NULL, NULL);

/* ===================================================================== *
 * #69 -- four shapes P1's FAC/FFT config functions refuse must now be
 * refused at BIND time (BRIDGE_HW_ERR_NOTIMPL), not accepted and left to
 * starve silently in the pump.
 * ===================================================================== */

ZTEST(gd32_bridge_adc_dsp, test_reject_two_stage_fir)
{
	reset_all();
	stream_running(0u);

	uint8_t  chain_id = open_chain();
	uint8_t  blob[8];
	uint16_t len = fir_blob(blob, 1u);
	push_stage(chain_id, 0u, 0u /* FIR */, blob, len);
	push_stage(chain_id, 1u, 0u /* FIR */, blob, len);

	int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
	zassert_equal(
	    rv, BRIDGE_HW_ERR_NOTIMPL, "two-stage FIR must be refused as NOTIMPL, got %d", rv);
}

ZTEST(gd32_bridge_adc_dsp, test_reject_fir_then_iir)
{
	reset_all();
	stream_running(0u);

	uint8_t  chain_id = open_chain();
	uint8_t  fir[8];
	uint16_t fir_len = fir_blob(fir, 1u);
	uint8_t  iir[24];
	uint16_t iir_len = iir_blob(iir, 1u);
	push_stage(chain_id, 0u, 0u /* FIR */, fir, fir_len);
	push_stage(chain_id, 1u, 1u /* IIR */, iir, iir_len);

	int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
	zassert_equal(rv, BRIDGE_HW_ERR_NOTIMPL, "FIR+IIR must be refused as NOTIMPL, got %d", rv);
}

ZTEST(gd32_bridge_adc_dsp, test_reject_iir_cascaded_sections)
{
	/* n_sections in 2..8 (P1 handles exactly 1) -- boundary + max. */
	const uint8_t cases[] = { 2u, 8u };

	for (size_t i = 0u; i < sizeof(cases) / sizeof(cases[0]); ++i) {
		reset_all();
		stream_running(0u);

		uint8_t  chain_id = open_chain();
		uint8_t  iir[4u + 8u * 5u * 4u];
		uint16_t len = iir_blob(iir, cases[i]);
		push_stage(chain_id, 0u, 1u /* IIR */, iir, len);

		int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
		zassert_equal(rv,
		              BRIDGE_HW_ERR_NOTIMPL,
		              "IIR n_sections=%u must be refused as NOTIMPL, got %d",
		              cases[i],
		              rv);
	}
}

ZTEST(gd32_bridge_adc_dsp, test_reject_fir_before_window_fft)
{
	reset_all();
	stream_running(0u);

	uint8_t  chain_id = open_chain();
	uint8_t  fir[8];
	uint16_t fir_len = fir_blob(fir, 1u);
	uint8_t  win[4];
	uint16_t win_len = window_blob(win, 0u);
	uint8_t  fft[4];
	uint16_t fft_len = fft_blob(fft, 32u, 0u);
	push_stage(chain_id, 0u, 0u /* FIR */, fir, fir_len);
	push_stage(chain_id, 1u, 2u /* WINDOW */, win, win_len);
	push_stage(chain_id, 2u, 3u /* FFT */, fft, fft_len);

	int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
	zassert_equal(rv,
	              BRIDGE_HW_ERR_NOTIMPL,
	              "FIR ahead of a WINDOW+FFT terminal must be refused as NOTIMPL, got %d",
	              rv);
}

/* ===================================================================== *
 * Non-regression: the four shapes P1 DOES support must still bind.  This
 * is the real risk of tightening a validator -- do not skip it.
 * ===================================================================== */

ZTEST(gd32_bridge_adc_dsp, test_accept_single_fir)
{
	reset_all();
	stream_running(0u);

	uint8_t  chain_id = open_chain();
	uint8_t  fir[8];
	uint16_t len = fir_blob(fir, 1u);
	push_stage(chain_id, 0u, 0u /* FIR */, fir, len);

	int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
	zassert_equal(rv, BRIDGE_HW_OK, "single FIR must still bind, got %d", rv);
}

ZTEST(gd32_bridge_adc_dsp, test_accept_single_section_iir)
{
	reset_all();
	stream_running(0u);

	uint8_t  chain_id = open_chain();
	uint8_t  iir[24];
	uint16_t len = iir_blob(iir, 1u);
	push_stage(chain_id, 0u, 1u /* IIR */, iir, len);

	int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
	zassert_equal(rv, BRIDGE_HW_OK, "1-section IIR must still bind, got %d", rv);
}

ZTEST(gd32_bridge_adc_dsp, test_accept_window_fft)
{
	reset_all();
	stream_running(0u);

	uint8_t  chain_id = open_chain();
	uint8_t  win[4];
	uint16_t win_len = window_blob(win, 1u);
	uint8_t  fft[4];
	uint16_t fft_len = fft_blob(fft, 64u, 1u);
	push_stage(chain_id, 0u, 2u /* WINDOW */, win, win_len);
	push_stage(chain_id, 1u, 3u /* FFT */, fft, fft_len);

	int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
	zassert_equal(rv, BRIDGE_HW_OK, "WINDOW+FFT must still bind, got %d", rv);
}

ZTEST(gd32_bridge_adc_dsp, test_accept_bare_fft)
{
	reset_all();
	stream_running(0u);

	uint8_t  chain_id = open_chain();
	uint8_t  fft[4];
	uint16_t len = fft_blob(fft, 128u, 2u);
	push_stage(chain_id, 0u, 3u /* FFT */, fft, len);

	int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
	zassert_equal(rv, BRIDGE_HW_OK, "bare FFT must still bind, got %d", rv);
}

/* ===================================================================== *
 * #70 -- a second FFT-terminal bind must be refused (the FFT block is
 * single-instance, like the FAC); FFT-on-one-stream + FIR-on-the-other
 * must still both succeed, because the FAC and FFT blocks ARE independent
 * HW and only same-class contention is refused.
 * ===================================================================== */

ZTEST(gd32_bridge_adc_dsp, test_reject_second_fft_bind)
{
	reset_all();
	stream_running(0u);
	stream_running(1u);

	uint8_t  c0 = open_chain();
	uint8_t  fft0[4];
	uint16_t fft0_len = fft_blob(fft0, 32u, 0u);
	push_stage(c0, 0u, 3u /* FFT */, fft0, fft0_len);
	int rv0 = bridge_hw_adc_dsp_chain_bind(c0, 0u);
	zassert_equal(rv0, BRIDGE_HW_OK, "first FFT bind must succeed, got %d", rv0);

	uint8_t  c1 = open_chain();
	uint8_t  fft1[4];
	uint16_t fft1_len = fft_blob(fft1, 32u, 0u);
	push_stage(c1, 0u, 3u /* FFT */, fft1, fft1_len);
	int rv1 = bridge_hw_adc_dsp_chain_bind(c1, 1u);
	zassert_equal(rv1,
	              BRIDGE_HW_ERR_NOTIMPL,
	              "second FFT bind to the OTHER stream must be refused, got %d",
	              rv1);
}

ZTEST(gd32_bridge_adc_dsp, test_fft_and_fir_independent_blocks)
{
	reset_all();
	stream_running(0u);
	stream_running(1u);

	uint8_t  c0 = open_chain();
	uint8_t  fft0[4];
	uint16_t fft0_len = fft_blob(fft0, 32u, 0u);
	push_stage(c0, 0u, 3u /* FFT */, fft0, fft0_len);
	int rv0 = bridge_hw_adc_dsp_chain_bind(c0, 0u);
	zassert_equal(rv0, BRIDGE_HW_OK, "FFT bind on stream 0 must succeed, got %d", rv0);

	uint8_t  c1 = open_chain();
	uint8_t  fir1[8];
	uint16_t fir1_len = fir_blob(fir1, 1u);
	push_stage(c1, 0u, 0u /* FIR */, fir1, fir1_len);
	int rv1 = bridge_hw_adc_dsp_chain_bind(c1, 1u);
	zassert_equal(rv1,
	              BRIDGE_HW_OK,
	              "FIR bind on the OTHER stream must still succeed (FAC/FFT independent), got %d",
	              rv1);
}

/* ===================================================================== *
 * Ordering / contiguity / blob-validity / pool-exhaustion rules pre-date
 * #69/#70 and must still behave exactly as before the split + fix.
 * ===================================================================== */

ZTEST(gd32_bridge_adc_dsp, test_gap_in_stage_list_rejected)
{
	reset_all();
	stream_running(0u);

	uint8_t  chain_id = open_chain();
	uint8_t  fir[8];
	uint16_t len = fir_blob(fir, 1u);
	push_stage(chain_id, 0u, 0u /* FIR */, fir, len);
	/* stage index 1 left empty -- stage 2 populated -> a gap. */
	push_stage(chain_id, 2u, 0u /* FIR */, fir, len);

	int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
	zassert_equal(rv, BRIDGE_HW_ERR_INVAL, "a gap in the stage list must reject, got %d", rv);
}

ZTEST(gd32_bridge_adc_dsp, test_fft_not_terminal_rejected)
{
	reset_all();
	stream_running(0u);

	/* A populated stage after the FFT that is NOT a WINDOW -- deliberately
	 * not FFT-then-WINDOW, which would also trip the separate "WINDOW must
	 * directly precede FFT" rule (`fft_index == 0u` in that check) and mask
	 * whether THIS check, "FFT must be terminal", is the one doing the
	 * rejecting. */
	uint8_t  chain_id = open_chain();
	uint8_t  fft[4];
	uint16_t fft_len = fft_blob(fft, 32u, 0u);
	uint8_t  fir[8];
	uint16_t fir_len = fir_blob(fir, 1u);
	push_stage(chain_id, 0u, 3u /* FFT */, fft, fft_len);
	push_stage(chain_id, 1u, 0u /* FIR */, fir, fir_len);

	int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
	zassert_equal(rv, BRIDGE_HW_ERR_INVAL, "FFT must be the terminal stage, got %d", rv);
}

ZTEST(gd32_bridge_adc_dsp, test_window_without_fft_rejected)
{
	reset_all();
	stream_running(0u);

	uint8_t  chain_id = open_chain();
	uint8_t  win[4];
	uint16_t len = window_blob(win, 0u);
	push_stage(chain_id, 0u, 2u /* WINDOW */, win, len);

	int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
	zassert_equal(
	    rv, BRIDGE_HW_ERR_INVAL, "WINDOW without a terminating FFT must reject, got %d", rv);
}

ZTEST(gd32_bridge_adc_dsp, test_malformed_fir_blob_rejected)
{
	reset_all();
	stream_running(0u);

	uint8_t chain_id = open_chain();
	uint8_t fir[4]   = { 1u, 0u, 0u, 0u }; /* n_taps = 0 -- invalid */
	push_stage(chain_id, 0u, 0u /* FIR */, fir, sizeof(fir));

	int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
	zassert_equal(rv, BRIDGE_HW_ERR_INVAL, "n_taps=0 FIR blob must reject, got %d", rv);
}

ZTEST(gd32_bridge_adc_dsp, test_pool_exhaustion_returns_notimpl)
{
	reset_all();

	uint8_t chain_id = 0xFFu;
	for (uint8_t i = 0u; i < BRIDGE_DSP_MAX_CHAINS; ++i) {
		int rv = bridge_hw_adc_dsp_chain_open(&chain_id);
		zassert_equal(rv, BRIDGE_HW_OK, "chain %u should still be available, got %d", i, rv);
	}

	int rv = bridge_hw_adc_dsp_chain_open(&chain_id);
	zassert_equal(rv, BRIDGE_HW_ERR_NOTIMPL, "pool exhaustion must return NOTIMPL, got %d", rv);
}

/* ===================================================================== *
 * Second-pass review coverage gaps -- six clauses of the chain_bind
 * contract that a mutation pass found completely unpinned: each of the
 * six can be deleted from hal/gd32/adc_dsp_chain.c and this suite stays
 * green.  Every case below is proven against the code with its named
 * guard removed, per-case, as it lands.
 * ===================================================================== */

/* T1 -- the FAC-busy gate (bridge_hw_adc_dsp_chain_bind's `else` arm,
 * `if (adc_dsp_filter_stream_busy(stream_id)) return
 * BRIDGE_HW_ERR_NOTIMPL;`) has zero coverage.  #70 restructured the
 * single busy check into an if/else over FFT-terminal vs.
 * filter-terminal chains; test_reject_second_fft_bind above pins only
 * the `if` (FFT) half.  Mirrors that case exactly but for the FAC. */
ZTEST(gd32_bridge_adc_dsp, test_reject_second_fir_bind)
{
	reset_all();
	stream_running(0u);
	stream_running(1u);

	uint8_t  c0 = open_chain();
	uint8_t  fir0[8];
	uint16_t fir0_len = fir_blob(fir0, 1u);
	push_stage(c0, 0u, 0u /* FIR */, fir0, fir0_len);
	int rv0 = bridge_hw_adc_dsp_chain_bind(c0, 0u);
	zassert_equal(rv0, BRIDGE_HW_OK, "first FIR bind must succeed, got %d", rv0);

	uint8_t  c1 = open_chain();
	uint8_t  fir1[8];
	uint16_t fir1_len = fir_blob(fir1, 1u);
	push_stage(c1, 0u, 0u /* FIR */, fir1, fir1_len);
	int rv1 = bridge_hw_adc_dsp_chain_bind(c1, 1u);
	zassert_equal(rv1,
	              BRIDGE_HW_ERR_NOTIMPL,
	              "second FIR bind to the OTHER stream must be refused (FAC busy), got %d",
	              rv1);
}

/* T2 -- adc_dsp_fft_stream_busy's SELECTIVITY (its `dsp_terminal == 3u`
 * term) is unpinned: test_fft_and_fir_independent_blocks above binds
 * FFT-then-FIR, and that order never calls adc_dsp_fft_stream_busy at
 * all (the FIR bind takes the FAC-busy branch instead).  This is the
 * reverse order -- FIR-then-FFT -- which DOES consult it: mutating the
 * predicate to `in_use && dsp_bound` (dropping the dsp_terminal check,
 * so any bound stream reads as FFT-busy) makes the second bind below
 * wrongly see stream 0's FIR bind as FFT contention. */
ZTEST(gd32_bridge_adc_dsp, test_fir_then_fft_independent_blocks)
{
	reset_all();
	stream_running(0u);
	stream_running(1u);

	uint8_t  c0 = open_chain();
	uint8_t  fir0[8];
	uint16_t fir0_len = fir_blob(fir0, 1u);
	push_stage(c0, 0u, 0u /* FIR */, fir0, fir0_len);
	int rv0 = bridge_hw_adc_dsp_chain_bind(c0, 0u);
	zassert_equal(rv0, BRIDGE_HW_OK, "FIR bind on stream 0 must succeed, got %d", rv0);

	uint8_t  c1 = open_chain();
	uint8_t  fft1[4];
	uint16_t fft1_len = fft_blob(fft1, 32u, 0u);
	push_stage(c1, 0u, 3u /* FFT */, fft1, fft1_len);
	int rv1 = bridge_hw_adc_dsp_chain_bind(c1, 1u);
	zassert_equal(rv1,
	              BRIDGE_HW_OK,
	              "FFT bind on the OTHER stream must still succeed (FAC/FFT independent), got %d",
	              rv1);
}

/* T3 -- `if (!st->complete) return BRIDGE_HW_ERR_INVAL;` is the FIRST
 * clause of bridge_hw_adc_dsp_chain_bind's documented contract in
 * hal/bridge_hw.h ("the chain has unfinished (mid-upload) stages"), and
 * CMD_ADC_DSP_STAGE_PUSH is chunked specifically so a host can hit this
 * mid-upload -- yet nothing here ever bound against a partial upload
 * before this case. */
ZTEST(gd32_bridge_adc_dsp, test_incomplete_stage_bind_rejected)
{
	reset_all();
	stream_running(0u);

	uint8_t chain_id = open_chain();
	uint8_t chunk[4] = { 1u, 1u, 0u, 0u }; /* fmt=Q31, n_taps=1 -- half of an 8-byte FIR blob */
	int     push_rv  = bridge_hw_adc_dsp_stage_push(chain_id, 0u, 0u /* FIR */, 0u, 8u, chunk, 4u);
	zassert_equal(push_rv, BRIDGE_HW_OK, "partial push itself must succeed, got %d", push_rv);

	int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
	zassert_equal(
	    rv, BRIDGE_HW_ERR_INVAL, "a mid-upload (incomplete) stage must reject bind, got %d", rv);
}

/* T4 -- `if (chunk_offset > chunk_total_size) return
 * BRIDGE_HW_ERR_RANGE;` in bridge_hw_adc_dsp_stage_push guards an
 * out-of-bounds write (F2, review of #69/#70): without it, `chunk_data_len
 * > (size_t)(chunk_total_size - chunk_offset)` promotes the subtraction
 * to `int`, which goes negative for chunk_offset > chunk_total_size and
 * then wraps to a huge size_t on the cast, so the guard silently passes
 * every out-of-range offset.  ASan reproduced the resulting OOB write in
 * st->data[] on exactly this sequence: chain_open, push offset=0
 * total=260 len=1, then push offset=60000 total=260 len=1. */
ZTEST(gd32_bridge_adc_dsp, test_chunk_offset_past_total_size_rejected)
{
	reset_all();
	stream_running(0u);

	uint8_t chain_id = open_chain();
	uint8_t byte     = 0xAAu;

	int rv0 = bridge_hw_adc_dsp_stage_push(chain_id, 0u, 0u /* FIR */, 0u, 260u, &byte, 1u);
	zassert_equal(rv0, BRIDGE_HW_OK, "first (opening) chunk must succeed, got %d", rv0);

	int rv1 = bridge_hw_adc_dsp_stage_push(chain_id, 0u, 0u /* FIR */, 60000u, 260u, &byte, 1u);
	zassert_equal(rv1,
	              BRIDGE_HW_ERR_RANGE,
	              "chunk_offset (60000) > chunk_total_size (260) must be RANGE, got %d",
	              rv1);
}

/* T5 -- multi-chunk assembly at a legitimate non-zero offset was
 * entirely uncovered: every push elsewhere in this suite goes through
 * push_stage(), which always uses chunk_offset == 0 -- the exact reason
 * the T4 chunk_offset > chunk_total_size defect survived as long as it
 * did.  Assemble one 8-byte FIR stage across two 4-byte chunks and check
 * both that bind succeeds and that the reassembled bytes are exactly
 * what was pushed -- a dropped `chunk_offset` in stage_push's copy loop
 * would overwrite bytes [0,4) twice instead of filling [4,8). */
ZTEST(gd32_bridge_adc_dsp, test_multi_chunk_assembly_succeeds)
{
	reset_all();
	stream_running(0u);

	uint8_t chain_id  = open_chain();
	uint8_t first[4]  = { 1u, 1u, 0u, 0u };             /* fmt=Q31 n_taps=1 rsvd=0 */
	uint8_t second[4] = { 0x00u, 0x00u, 0x00u, 0x10u }; /* tap[0], arbitrary nonzero Q31 */

	int rv0 = bridge_hw_adc_dsp_stage_push(chain_id, 0u, 0u /* FIR */, 0u, 8u, first, 4u);
	zassert_equal(rv0, BRIDGE_HW_OK, "first chunk push must succeed, got %d", rv0);
	int rv1 = bridge_hw_adc_dsp_stage_push(chain_id, 0u, 0u /* FIR */, 4u, 8u, second, 4u);
	zassert_equal(rv1, BRIDGE_HW_OK, "second chunk push must succeed, got %d", rv1);

	int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
	zassert_equal(rv, BRIDGE_HW_OK, "two-chunk FIR assembly must bind, got %d", rv);

	uint8_t expect[8];
	memcpy(expect, first, sizeof(first));
	memcpy(expect + sizeof(first), second, sizeof(second));
	zassert_mem_equal(adc_dsp_chains[chain_id].stages[0].data,
	                  expect,
	                  sizeof(expect),
	                  "assembled stage bytes must match the two pushed chunks");
}

/* T6 -- adc_dsp_chain_p1_capable()'s FFT-terminal loop rejects a FIR/IIR
 * stage ahead of an FFT terminal whether or not a WINDOW is present, but
 * test_reject_fir_before_window_fft above only exercises the FIR+WINDOW+
 * FFT form.  FIR directly ahead of a BARE FFT (no WINDOW stage anywhere
 * in the chain) is a distinct shape that same test cannot pin: a
 * predicate that only rejects a leading FIR/IIR when a WINDOW is present
 * elsewhere in the chain would still pass test_reject_fir_before_window_fft
 * while wrongly accepting this one. */
ZTEST(gd32_bridge_adc_dsp, test_reject_fir_before_bare_fft)
{
	reset_all();
	stream_running(0u);

	uint8_t  chain_id = open_chain();
	uint8_t  fir[8];
	uint16_t fir_len = fir_blob(fir, 1u);
	uint8_t  fft[4];
	uint16_t fft_len = fft_blob(fft, 32u, 0u);
	push_stage(chain_id, 0u, 0u /* FIR */, fir, fir_len);
	push_stage(chain_id, 1u, 3u /* FFT */, fft, fft_len);

	int rv = bridge_hw_adc_dsp_chain_bind(chain_id, 0u);
	zassert_equal(rv,
	              BRIDGE_HW_ERR_NOTIMPL,
	              "FIR ahead of a BARE FFT terminal must be refused as NOTIMPL, got %d",
	              rv);
}
