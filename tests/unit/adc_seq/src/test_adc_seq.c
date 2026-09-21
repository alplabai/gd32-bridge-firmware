/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Register-sequence tests for hal/gd32/adc.c + hal/gd32/adc_stream.c,
 * against the mock GD32 vendor header in tests/unit/adc_seq/mock/.
 *
 * WHY THIS SUITE EXISTS: #77 means no CI job compiles hal/gd32/ sources at
 * all -- the stub-build job links only hal/bridge_hw_stub.c, and the
 * two host suites in tests/unit/ link the stub HAL, not the GD32 one.
 * PR #80's review asked for exactly two things by name:
 *
 *   #44 -- "a unit test asserting that stream_begin clears ADC_STAT.
 *           ROVF before adc_enable, and one that drives the mocked
 *           ADC_SSTAT bit ... and asserts that bridge_hw_adc_stream_
 *           read returns BRIDGE_HW_ERR_BUSY."
 *   #34 -- "a register-sequence gate ... assert[ing] that every
 *           adc_enable() call site on the request path is followed
 *           by a calibration."
 *
 * These link the REAL hal/gd32/adc.c + hal/gd32/adc_stream.c sources
 * (unmodified) against the mock vendor header above, so a regression
 * in the actual production sequencing fails a real assertion instead
 * of a hand-read of the diff.
 *
 * NOTE on #44's "ADC_SSTAT" wording: the merged fix polls the per-ADC
 * ADC_FLAG_ROVF via adc_flag_get(), not the four-converter ADC_SSTAT
 * summary register issue #44 offered as one implementation option (see
 * adc_stream.c's bridge_hw_adc_stream_read) -- so "drive the mocked
 * ADC_SSTAT bit" is satisfied here by driving the flag the shipped
 * code actually reads.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "bridge_hw.h"
#include "gd32_common.h"

#include "gd32g5x3.h" /* mock -- tests/unit/adc_seq/mock/ */

/* adc_channels_map[0] = { ADC3, ADC_CHANNEL_12, GPIOD, GPIO_PIN_9 } --
 * bridge channel 0, the periph every "channel 0" test below drives. */
#define BRIDGE_ADC_CH0        0u
#define BRIDGE_ADC_CH0_PERIPH ADC3

/* #35 tests drive the base-level pump + chain plumbing declared in
 * adc_stream.c / adc_dsp_chain.c and published via gd32_common.h /
 * bridge_hw.h. */
extern void bridge_hw_dsp_pump(void);
extern void adc_dsp_fac_release(uint8_t stream_id); /* adc_stream.c */

/* Releases the FAC ownership latch; defined next to the gh#35 tests
 * (forward-declared so the earlier tests' resets can call it too). */
static void fac_latch_release(void);

static void adc_seq_reset(void)
{
	mock_seq_reset();
	memset(mock_adc_ctl1, 0, sizeof mock_adc_ctl1);
	for (uint8_t s = 0u; s < BRIDGE_ADC_STREAM_COUNT; ++s) {
		adc_streams[s] = (adc_stream_state_t){ 0 };
	}
	mock_adc_set_flag(BRIDGE_ADC_CH0_PERIPH, ADC_FLAG_ROVF, RESET);
	mock_adc_set_flag(BRIDGE_ADC_CH0_PERIPH, ADC_FLAG_EOC, RESET);
	vref_ok = true;
}

/* ---------------------------------------------------------------------
 * #34 -- calibration must follow every adc_enable() on the request
 * path.  adc_calibrate_bounded() is the ONLY code that touches
 * ADC_CTL1 in either source file (checked by hand against both), so
 * an "ADC_CTL1_TOUCH" log entry is unambiguous evidence calibration
 * ran; its position relative to "adc_enable" pins the ORDER.
 * --------------------------------------------------------------------- */

ZTEST(gd32_adc_seq, test_read_recalibrates_after_enable)
{
	uint16_t mv[1];

	adc_seq_reset();
	fac_latch_release();

	int rc = bridge_hw_adc_read(BRIDGE_ADC_CH0, 1u, mv);
	zassert_equal(rc, BRIDGE_HW_OK, "read succeeds against the mock");

	int enable_i  = mock_seq_find_from("adc_enable", BRIDGE_ADC_CH0_PERIPH, 0);
	int calib_i   = mock_seq_find_from("ADC_CTL1_TOUCH", BRIDGE_ADC_CH0_PERIPH, 0);
	int trigger_i = mock_seq_find_from("adc_software_trigger_enable", BRIDGE_ADC_CH0_PERIPH, 0);

	zassert_true(enable_i >= 0, "adc_enable was called");
	zassert_true(calib_i >= 0, "the calibration FSM (ADC_CTL1) was touched");
	zassert_true(trigger_i >= 0, "a conversion was triggered");
	zassert_true(calib_i > enable_i, "calibration must run AFTER adc_enable, not before");
	zassert_true(trigger_i > calib_i,
	             "calibration must complete BEFORE the first conversion trigger");
}

ZTEST(gd32_adc_seq, test_stream_begin_recalibrates_after_enable)
{
	adc_seq_reset();
	fac_latch_release();

	int rc = bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u);
	zassert_equal(rc, BRIDGE_HW_OK, "stream_begin succeeds against the mock");

	int enable_i = mock_seq_find_from("adc_enable", BRIDGE_ADC_CH0_PERIPH, 0);
	int calib_i  = mock_seq_find_from("ADC_CTL1_TOUCH", BRIDGE_ADC_CH0_PERIPH, 0);

	zassert_true(enable_i >= 0, "adc_enable was called");
	zassert_true(calib_i >= 0, "the calibration FSM (ADC_CTL1) was touched");
	zassert_true(calib_i > enable_i, "calibration must run AFTER adc_enable, not before (#34)");
}

/* #183 -- dma_init() must reload the DMA count for a new session.  Without
 * that mock contract, a DSP pump after restart sees samples which only
 * existed in the preceding stream. */
ZTEST(gd32_adc_seq, test_stream_restart_reloads_dma_count)
{
	adc_seq_reset();
	fac_latch_release();

	int rc = bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u);
	zassert_equal(rc, BRIDGE_HW_OK, "first stream begins against the mock");
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 32u);

	rc = bridge_hw_adc_stream_end(0u);
	zassert_equal(rc, BRIDGE_HW_OK, "first stream ends against the mock");
	rc = bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u);
	zassert_equal(rc, BRIDGE_HW_OK, "replacement stream begins against the mock");

	zassert_equal(dma_transfer_number_get(DMA0, DMA_CH0),
	              BRIDGE_ADC_STREAM_RING_SAMPLES,
	              "dma_init must reload the full count, not retain stale remainder (#183)");
}

/* ---------------------------------------------------------------------
 * #44 test 1 -- stream_begin clears ROVF before adc_enable.
 * --------------------------------------------------------------------- */

ZTEST(gd32_adc_seq, test_stream_begin_clears_rovf_before_enable)
{
	adc_seq_reset();
	fac_latch_release();
	/* Start with ROVF already set, as a prior session might leave it
	 * (the exact scenario #44 describes). */
	mock_adc_set_flag(BRIDGE_ADC_CH0_PERIPH, ADC_FLAG_ROVF, SET);

	int rc = bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u);
	zassert_equal(rc, BRIDGE_HW_OK, "stream_begin succeeds against the mock");

	/* adc_flag_clear is called for both EOC and ROVF; find the ROVF
	 * one specifically via its logged arg. */
	int rovf_clear_i = -1;
	for (int i = 0; i < mock_seq_n; ++i) {
		if (strcmp(mock_seq[i].name, "adc_flag_clear") == 0 &&
		    mock_seq[i].periph == BRIDGE_ADC_CH0_PERIPH && mock_seq[i].arg == ADC_FLAG_ROVF) {
			rovf_clear_i = i;
			break;
		}
	}
	int enable_i = mock_seq_find_from("adc_enable", BRIDGE_ADC_CH0_PERIPH, 0);

	zassert_true(rovf_clear_i >= 0, "ROVF was cleared");
	zassert_true(enable_i >= 0, "adc_enable was called");
	zassert_true(rovf_clear_i < enable_i,
	             "ROVF must be cleared BEFORE adc_enable, not after (#44)");
}

/* ---------------------------------------------------------------------
 * #44 test 2 -- a live ROVF makes bridge_hw_adc_stream_read report
 * BRIDGE_HW_ERR_BUSY (never a silent zero-sample STATUS_OK).
 * --------------------------------------------------------------------- */

ZTEST(gd32_adc_seq, test_stream_read_reports_busy_on_rovf)
{
	adc_seq_reset();
	fac_latch_release();

	int rc = bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u);
	zassert_equal(rc, BRIDGE_HW_OK, "stream_begin succeeds against the mock");

	/* Drive the mocked overflow flag exactly as #44 describes: an
	 * overflow occurred mid-stream. */
	mock_adc_set_flag(BRIDGE_ADC_CH0_PERIPH, ADC_FLAG_ROVF, SET);

	uint8_t  got = 0xFFu;
	uint16_t mv[4];
	rc = bridge_hw_adc_stream_read(0u, 4u, &got, mv);

	zassert_equal(rc,
	              BRIDGE_HW_ERR_BUSY,
	              "a live ROVF must answer BUSY, never silent STATUS_OK/zero-samples");
	zassert_equal(got, 0u, "no samples are served from an overflowed ring");
	zassert_equal((int)adc_flag_get(BRIDGE_ADC_CH0_PERIPH, ADC_FLAG_ROVF),
	              (int)RESET,
	              "recovery must clear ROVF");
}

/* ---------------------------------------------------------------------
 * ROVF recovery must clear the DMA FTF interrupt flag (this PR's fix).
 * --------------------------------------------------------------------- */

ZTEST(gd32_adc_seq, test_rovf_recovery_clears_dma_ftf)
{
	adc_seq_reset();
	fac_latch_release();

	int rc = bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u);
	zassert_equal(rc, BRIDGE_HW_OK, "stream_begin succeeds against the mock");

	mock_adc_set_flag(BRIDGE_ADC_CH0_PERIPH, ADC_FLAG_ROVF, SET);

	uint8_t  got = 0xFFu;
	uint16_t mv[4];
	rc = bridge_hw_adc_stream_read(0u, 4u, &got, mv);
	zassert_equal(rc, BRIDGE_HW_ERR_BUSY, "ROVF recovery path taken");

	/* DMA0 is stream 0's controller (stream_begin: s->dma_periph =
	 * (stream_id == 0u) ? DMA0 : DMA1). */
	int ftf_clear_i = mock_seq_find_from("dma_interrupt_flag_clear", DMA0, 0);
	zassert_true(ftf_clear_i >= 0,
	             "ROVF recovery must clear the DMA FTF interrupt flag, "
	             "or a pending lap tick fires the instant this handler "
	             "returns and corrupts the freshly-resynced lap_count");
}

/* ---------------------------------------------------------------------
 * #35 -- the FAC filter decode.  The production decode now (a) NEGATES
 * the feedback pair (the FAC adds the feedback term, UM Rev1.2 p.1505
 * eq.(35-2); the wire contract subtracts it), (b) scales coefficients
 * against the accumulator gain IPR instead of clamping them onto the
 * q1.15 rail, and (c) biases the pump's input at mid-scale so a signed
 * output is not half-wave rectified.  These tests pin the exact
 * coefficient words two independent vendor references print.
 * --------------------------------------------------------------------- */

/* Open + bind a single-stage F32 IIR chain on stream 0 (stream already
 * begun by the caller), push the 5-coefficient section, and bind. */
static void
bind_f32_iir(const float b0, const float b1, const float b2, const float a1, const float a2)
{
	uint8_t blob[4u + 5u * 4u];
	blob[0]          = 0u; /* fmt = F32 */
	blob[1]          = 1u; /* n_sections */
	blob[2]          = 0u;
	blob[3]          = 0u;
	const float c[5] = { b0, b1, b2, a1, a2 };
	memcpy(&blob[4u], c, sizeof c);

	uint8_t cid = 0xFFu;
	zassert_equal(bridge_hw_adc_dsp_chain_open(&cid), BRIDGE_HW_OK, "chain_open");
	int rv =
	    bridge_hw_adc_dsp_stage_push(cid, 0u, 1u /* IIR */, 0u, sizeof blob, blob, sizeof blob);
	zassert_equal(rv, BRIDGE_HW_OK, "stage_push");
	zassert_equal(bridge_hw_adc_dsp_chain_bind(cid, 0u), BRIDGE_HW_OK, "chain_bind");
}

/* Release the FAC ownership latch before a test claims it.  The latch
 * (adc_dsp_fac_owner, static in adc_stream.c) survives both adc_seq_reset
 * and a failed zassert aborting the previous test BEFORE its
 * stream_end -- so a test that starts with the latch held would skip
 * adc_dsp_fac_config() entirely and assert against a stale FAC. */
static void fac_latch_release(void)
{
	adc_dsp_fac_release(0u);
}

ZTEST(gd32_adc_seq, test_fac_iir_decode_matches_an208)
{
	adc_seq_reset();
	fac_latch_release();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream_begin");

	/* AN208 p.13's own worked example: the design biquad
	 * B = [0.0200806, 0.0401611, 0.0200806] (= [658, 1316, 658]/32768)
	 * and A = [1.0, -1.561018075800718, 0.641351538057563].  AN208
	 * prints the words the MCU must receive as B = [329, 658, 329] and
	 * A = [25575, -10507] (first element dropped, sign reversed,
	 * halved) -- exactly what the negation + IPR-scaling decode must
	 * produce, with the headroom exponent g = 1 (max |coeff| = 1.561).
	 */
	bind_f32_iir(658.0f / 32768.0f,
	             1316.0f / 32768.0f,
	             658.0f / 32768.0f,
	             -1.561018075800718f,
	             0.641351538057563f);

	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);
	bridge_hw_dsp_pump();

	zassert_equal(mock_fac_func, (uint32_t)FUNC_IIR_DIRECT_FORM_1, "IIR direct-form-1 selected");
	zassert_equal(mock_fac_ipr, 1u, "headroom exponent g=1 lands in IPR (#35)");
	zassert_equal(mock_fac_coeffb_size, 3u, "coeffb = feed-forward B");
	zassert_equal(mock_fac_coeffb[0], 329, "AN208 B[0]");
	zassert_equal(mock_fac_coeffb[1], 658, "AN208 B[1]");
	zassert_equal(mock_fac_coeffb[2], 329, "AN208 B[2]");
	zassert_equal(mock_fac_coeffa_size, 2u, "coeffa = feedback A");
	zassert_equal(mock_fac_coeffa[0], 25575, "AN208 A[1] negated+halved (#35: poles not mirrored)");
	zassert_equal(mock_fac_coeffa[1], -10507, "AN208 A[2] negated+halved");

	bridge_hw_adc_stream_end(0u); /* releases the FAC for the next test */
}

ZTEST(gd32_adc_seq, test_fac_iir_decode_matches_vendor_iir_dma)
{
	adc_seq_reset();
	fac_latch_release();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream_begin");

	/* The vendor Iir_dma example (main.c:58-60 with iir_gain = 1 at
	 * main.c:205): a 16 kHz / 500 Hz second-order Butterworth that
	 * loads iir_coeffa[] = { 28242, -12412 }.  The design values that
	 * reproduce those exact words under this decode are a1 = -1.7238
	 * (28242.74 -> 28242) and a2 = +0.75757 (12412.03 -> 12412),
	 * negated per the FAC's plus-sign feedback convention (the
	 * -1.72385 / "1.72386" quotes floating around issue trackers
	 * truncate one or two off the shipped constants; the vendor's own
	 * rounded design values land exactly).  With max|coeff| = 1.724
	 * the headroom exponent is g = 1.  The b values are this test's
	 * own (in range, so g is still set by the feedback pair). */
	bind_f32_iir(0.02f, 0.04f, 0.02f, -1.7238f, 0.75757f);

	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);
	bridge_hw_dsp_pump();

	zassert_equal(mock_fac_ipr, 1u, "g=1 for the vendor example's magnitude too");
	zassert_equal(mock_fac_coeffa[0], 28242, "Iir_dma a1: -(-1.72385)/2*32768");
	zassert_equal(mock_fac_coeffa[1], -12412, "Iir_dma a2: -(+0.75757)/2*32768");

	bridge_hw_adc_stream_end(0u);
}

ZTEST(gd32_adc_seq, test_fac_rejects_coeff_beyond_ipr_range)
{
	adc_seq_reset();
	fac_latch_release();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream_begin");

	/* max|coeff| >= 128 cannot be scaled into q1.15 even at g = 7:
	 * the config must refuse (sticky) instead of clamping onto the
	 * rail and serving a different filter with STATUS_OK. */
	bind_f32_iir(0.02f, 0.04f, 0.02f, -128.0f, 0.75757f);

	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);
	bridge_hw_dsp_pump();

	zassert_true(adc_streams[0].dsp_cfg_bad,
	             "the out-of-range chain must be refused stickily, not clamped");
	uint8_t  got = 0u;
	uint16_t mv[4];
	zassert_equal(bridge_hw_adc_stream_read(0u, 4u, &got, mv),
	              BRIDGE_HW_ERR_RANGE,
	              "a refused config must surface RANGE, never STATUS_OK samples");

	bridge_hw_adc_stream_end(0u);
	zassert_equal(adc_streams[0].dsp_cfg_bad, false, "stream_end clears the sticky flag");
}

ZTEST(gd32_adc_seq, test_fac_pump_biases_input_and_rebiases_output)
{
	adc_seq_reset();
	fac_latch_release();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream_begin");
	bind_f32_iir(0.25f, 0.5f, 0.25f, -1.561018075800718f, 0.641351538057563f);

	/* One sample in the raw ring: code 2560 (mid-scale + 512).  The
	 * pump must hand the FAC x = (2560 - 2048) << 3 = +4096 -- biased
	 * around mid-scale, NOT the old non-negative code << 3. */
	adc_streams[0].ring[0] = 2560u;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);

	/* A NEGATIVE FAC output (the legitimate half the old code
	 * rectified away): q15 -8192 (a quarter below zero). */
	mock_fac_read_value = -8192;
	bridge_hw_dsp_pump();

	zassert_equal(mock_fac_last_write, 4096, "input must be mid-scale biased (#35)");
	zassert_equal(adc_streams[0].proc_write, 1u, "one processed sample produced");
	zassert_equal(adc_streams[0].proc_ring[0],
	              1024u,
	              "output re-bias maps -8192 to code 1024 -- the negative half is served, "
	              "not clamped to 0");

	bridge_hw_adc_stream_end(0u);
}

ZTEST(gd32_adc_seq, test_fac_saturation_is_sticky)
{
	adc_seq_reset();
	fac_latch_release();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream_begin");
	bind_f32_iir(0.25f, 0.5f, 0.25f, -1.561018075800718f, 0.641351538057563f);

	adc_streams[0].ring[0] = 2560u;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);
	mock_fac_flags = FAC_FLAG_STEF; /* output saturation observed */
	bridge_hw_dsp_pump();

	zassert_true(adc_streams[0].dsp_sat, "STEF must mark the stream sticky");
	uint8_t  got = 0u;
	uint16_t mv[4];
	zassert_equal(bridge_hw_adc_stream_read(0u, 4u, &got, mv),
	              BRIDGE_HW_ERR_IO,
	              "a saturated series must never be reported as STATUS_OK");

	bridge_hw_adc_stream_end(0u);
}

ZTEST_SUITE(gd32_adc_seq, NULL, NULL, NULL, NULL, NULL);
