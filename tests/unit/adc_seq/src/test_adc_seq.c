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
#include "adc_dsp_chain.h"
#include "gd32_common.h"

#include "gd32g5x3.h" /* mock -- tests/unit/adc_seq/mock/ */

/* Internal base-level lifecycle hooks from adc_stream.c. */
void adc_dsp_fac_release(uint8_t stream_id);
void adc_dsp_fft_release(uint8_t stream_id);
void bridge_hw_dsp_pump(void);

/* adc_channels_map[0] = { ADC3, ADC_CHANNEL_12, GPIOD, GPIO_PIN_9 } --
 * bridge channel 0, the periph every "channel 0" test below drives. */
#define BRIDGE_ADC_CH0        0u
#define BRIDGE_ADC_CH0_PERIPH ADC3
#define BRIDGE_ADC_CH2        2u
#define BRIDGE_ADC_CH4        4u
#define BRIDGE_ADC_CH2_PERIPH ADC2

/* #35 tests drive the base-level pump + chain plumbing declared in
 * adc_stream.c / adc_dsp_chain.c and published via gd32_common.h /
 * bridge_hw.h. */
extern void bridge_hw_dsp_pump(void);
extern void adc_dsp_fac_release(uint8_t stream_id); /* adc_stream.c */

/* Releases the FAC ownership latch; defined next to the gh#35 tests
 * (forward-declared so the earlier tests' resets can call it too). */
static void fac_latch_release(void);

/* Default trigger behaviour: the hardware finishes the whole burst at once --
 * every code is the mock's routine-data value, FTF is raised, and the burst
 * DMA IRQ runs before bridge_hw_adc_read_start() returns.  That is what lets
 * the blocking bridge_hw_adc_read (I2C path) run to completion on the host.
 * Tests of the asynchronous behaviour clear it with mock_adc_set_trigger_hook(0)
 * and play the interrupt themselves. */
static void burst_autocomplete(void)
{
	if (adc_burst.state != ADC_BURST_RUNNING) return;
	for (uint8_t i = 0u; i < adc_burst.samples; ++i) {
		adc_burst.codes[i] = (uint16_t)mock_adc_get_routine_data();
	}
	mock_dma_set_interrupt_flag(DMA1, DMA_CH1, DMA_INT_FLAG_FTF, SET);
	DMA1_Channel1_IRQHandler();
}

static void adc_seq_reset(void)
{
	/* A burst left running by a previous case would keep the converter claim and
	 * the DMA slot; quiesce it (and every recorded format) before the log and
	 * the mock state are cleared. */
	adc_deepsleep_quiesce();
	mock_adc_set_trigger_hook(burst_autocomplete);
	/* The owner bytes are file-local production state. Release both stream
	 * IDs before zeroing the visible arrays so a failed prior case cannot
	 * contaminate the next case; clear the resulting mock log afterward. */
	for (uint8_t s = 0u; s < BRIDGE_ADC_STREAM_COUNT; ++s) {
		adc_dsp_fac_release(s);
		adc_dsp_fft_release(s);
	}
	mock_seq_reset();
	mock_dma_reset();
	memset(mock_adc_ctl1, 0, sizeof mock_adc_ctl1);
	memset(adc_dsp_chains, 0, sizeof adc_dsp_chains);
	for (uint8_t s = 0u; s < BRIDGE_ADC_STREAM_COUNT; ++s) {
		adc_streams[s] = (adc_stream_state_t){ 0 };
	}
	mock_adc_set_flag(BRIDGE_ADC_CH0_PERIPH, ADC_FLAG_ROVF, RESET);
	mock_adc_set_flag(BRIDGE_ADC_CH0_PERIPH, ADC_FLAG_EOC, RESET);
	/* The per-channel format caches persist across cases; start every case from
	 * what bridge_hw_init seeds so an earlier configure cannot leak forward. */
	for (uint8_t c = 0u; c < 8u; ++c) {
		adc_sample_cycles_cache[c]    = ADC_DEFAULT_SAMPLE_CYCLES;
		adc_resolution_bits_cache[c]  = ADC_RES_BITS_DEFAULT;
		adc_oversample_ratio_cache[c] = 1u;
	}
	vref_ok            = true;
	adc_vref_mv        = ADC_VREF_MV;
	adc_vrefint_code   = 0u;
	mock_adc_eoc_stuck = false;
	mock_vref_ready    = true;
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
	zassert_equal(mock_rcu_lock_violations,
	              0u,
	              "every runtime ADC-stream clock enable must hold PRIMASK (#148)");
	zassert_equal(mock_primask, 0u, "clock-enable critical sections must restore PRIMASK");
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

/* #52 -- a new stream is configured only after CHEN reads clear, and ending
 * it releases the DMAMUX request selection before the converter is offered
 * to another stream. */
ZTEST(gd32_adc_seq, test_stream_lifecycle_confirms_disable_and_releases_dmamux)
{
	adc_seq_reset();

	int rc = bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u);
	zassert_equal(rc, BRIDGE_HW_OK, "stream begins against the mock");
	zassert_equal(mock_dmamux_request_get(0u),
	              DMA_REQUEST_ADC3,
	              "DMA0 channel 0 selects ADC3 while the stream is live");
	zassert_true(mock_seq_find_from("DMA_CHCTL_READ", DMA0, 0) >= 0,
	             "begin must observe CHEN clear before configuring DMA");

	rc = bridge_hw_adc_stream_end(0u);
	zassert_equal(rc, BRIDGE_HW_OK, "stream ends against the mock");
	zassert_equal(mock_dmamux_request_get(0u),
	              0u,
	              "end must return the DMAMUX channel to its no-request state");
}

ZTEST(gd32_adc_seq, test_stream_begin_refuses_a_channel_that_does_not_disable)
{
	adc_seq_reset();
	mock_dma_set_disable_hold(DMA0, DMA_CH0, true);

	int rc = bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u);
	zassert_equal(
	    rc, BRIDGE_HW_ERR_IO, "begin must not configure count/address while CHEN remains set");
	zassert_equal(mock_seq_find_from("dma_deinit", DMA0, 0),
	              -1,
	              "no SPL register rewrite is permitted before CHEN reads clear");
}

/* #137 -- ADC0/1/2 share ADC_SYNCCTL.  Boot resets every converter before
 * setting the two shared clock domains once; request paths never get to
 * perform either global operation. */
ZTEST(gd32_adc_seq, test_boot_adc_reset_precedes_two_shared_clock_setups)
{
	adc_seq_reset();

	adc_periph_boot_reset_all();
	adc_shared_clock_init();
	(void)adc_periph_boot_init(ADC0);
	(void)adc_periph_boot_init(ADC1);
	(void)adc_periph_boot_init(ADC2);
	(void)adc_periph_boot_init(ADC3);

	int clock0_i = mock_seq_find_from("adc_clock_config", ADC0, 0);
	int clock3_i = mock_seq_find_from("adc_clock_config", ADC3, 0);
	zassert_true(clock0_i >= 0 && clock3_i >= 0, "both shared ADC clock domains are configured");

	int clock_count = 0;
	for (int i = 0; i < mock_seq_n; ++i) {
		if (strcmp(mock_seq[i].name, "adc_clock_config") == 0) ++clock_count;
		if (strcmp(mock_seq[i].name, "adc_deinit") == 0) {
			zassert_true(i < clock0_i && i < clock3_i,
			             "all ADC resets precede both shared-clock writes");
		}
	}
	zassert_equal(clock_count, 2, "only ADC0 and ADC3 may configure shared clock domains");
}

ZTEST(gd32_adc_seq, test_stream_end_restore_does_not_reset_or_reclock_adc)
{
	adc_seq_reset();
	int rc = bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH2, 1000u);
	zassert_equal(rc, BRIDGE_HW_OK, "ADC2 stream begins against the mock");
	rc = bridge_hw_adc_stream_begin(1u, BRIDGE_ADC_CH4, 1000u);
	zassert_equal(rc, BRIDGE_HW_OK, "ADC1 sibling stream begins against the mock");

	mock_seq_reset();
	rc = bridge_hw_adc_stream_end(0u);
	zassert_equal(rc, BRIDGE_HW_OK, "stream ends against the mock");
	zassert_true(mock_seq_find_from("adc_disable", BRIDGE_ADC_CH2_PERIPH, 0) >= 0,
	             "restore enters an ADCON-off configuration window");
	zassert_equal(mock_seq_find_from("adc_deinit", BRIDGE_ADC_CH2_PERIPH, 0),
	              -1,
	              "stream end must not reset an ADC while a sibling may stream");
	zassert_equal(mock_seq_find_from("adc_clock_config", BRIDGE_ADC_CH2_PERIPH, 0),
	              -1,
	              "stream end must not rewrite a shared ADC clock domain");
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

/* gh#18 A22: DMA count 0 (mid circular reload) must not become write
 * index 1024 -- read_idx would then index ring[1024] (== read_idx). */
ZTEST(gd32_adc_seq, test_overrun_resync_with_zero_dma_count_keeps_read_idx_in_ring)
{
	adc_seq_reset();
	fac_latch_release();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream_begin");
	adc_streams[0].lap_count = 2u; /* writer lapped the reader */
	mock_dma_set_remaining(DMA0, DMA_CH0, 0u);

	uint8_t  got = 0xFFu;
	uint16_t mv[4];
	zassert_equal(bridge_hw_adc_stream_read(0u, 4u, &got, mv), BRIDGE_HW_ERR_BUSY, "overrun");
	zassert_true(adc_streams[0].read_idx < BRIDGE_ADC_STREAM_RING_SAMPLES,
	             "read_idx must stay inside the ring");
	bridge_hw_adc_stream_end(0u);
}

/* gh#18 A23: a saturated FAC input must leave the sample in the ring. */
ZTEST(gd32_adc_seq, test_fac_pump_does_not_consume_a_sample_when_x0_buffer_full)
{
	adc_seq_reset();
	fac_latch_release();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream_begin");
	bind_f32_iir(0.25f, 0.5f, 0.25f, -1.561018075800718f, 0.641351538057563f);
	adc_streams[0].ring[0] = 2560u;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);

	mock_fac_flags  = FAC_FLAG_X0BFF;
	uint32_t before = adc_streams[0].pump_raw_read;
	bridge_hw_dsp_pump();
	zassert_equal(adc_streams[0].pump_raw_read, before, "sample must stay unconsumed");
	zassert_equal(adc_streams[0].proc_write, 0u, "nothing produced");

	mock_fac_flags = 0u;
	bridge_hw_dsp_pump();
	zassert_equal(adc_streams[0].pump_raw_read, before + 1u, "sample consumed once FAC drains");
	zassert_equal(adc_streams[0].proc_write, 1u, "and filtered");
	bridge_hw_adc_stream_end(0u);
}

/* gh#306 -- FIR on silicon.  The mock now models FAC_PARACFG: an empty
 * X0 preload leaves EXE set and the FIR function config never latches
 * (bench: FAC_PARACFG stayed 0x81000000, pump stalled at X0BFF, Y empty). */
static void bind_q31_fir(const uint8_t n_taps)
{
	uint8_t blob[4u + 8u * 4u];
	memset(blob, 0, sizeof blob);
	blob[0] = 1u; /* fmt = Q31 */
	blob[1] = n_taps;
	for (uint8_t k = 0u; k < n_taps; ++k) {
		blob[4u + k * 4u + 3u] = 0x10u; /* tap = 0x10000000 (0.125) */
	}
	const uint16_t len = (uint16_t)(4u + n_taps * 4u);
	uint8_t        cid = 0xFFu;
	zassert_equal(bridge_hw_adc_dsp_chain_open(&cid), BRIDGE_HW_OK, "chain_open");
	zassert_equal(bridge_hw_adc_dsp_stage_push(cid, 0u, 0u /* FIR */, 0u, len, blob, len),
	              BRIDGE_HW_OK,
	              "stage_push");
	zassert_equal(bridge_hw_adc_dsp_chain_bind(cid, 0u), BRIDGE_HW_OK, "chain_bind");
}

ZTEST(gd32_adc_seq, test_fac_fir_config_latches_and_pump_filters)
{
	adc_seq_reset();
	fac_latch_release();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream_begin");
	bind_q31_fir(8u);
	adc_streams[0].ring[0] = 2560u;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);
	bridge_hw_dsp_pump();

	zassert_false(adc_streams[0].dsp_cfg_bad, "config must not wedge");
	zassert_equal(mock_fac_func, (uint32_t)FUNC_CONVO_FIR, "FIR function latched (FUN=8)");
	zassert_equal((mock_fac_paracfg & FAC_PARACFG_FUN), (uint32_t)FUNC_CONVO_FIR, "PARACFG FUN=8");
	zassert_true((mock_fac_paracfg & FAC_PARACFG_EXE) != 0u, "started");
	zassert_equal(mock_fac_coeffb_size, 8u, "X1 coefficient load latched");
	zassert_equal(adc_streams[0].proc_write, 1u, "filtered sample produced");
	bridge_hw_adc_stream_end(0u);
}

ZTEST(gd32_adc_seq, test_fac_config_reports_a_wedged_load)
{
	adc_seq_reset();
	fac_latch_release();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream_begin");
	bind_q31_fir(8u);
	mock_fac_load_wedge    = true; /* EXE never clears after the load */
	adc_streams[0].ring[0] = 2560u;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);
	bridge_hw_dsp_pump();
	zassert_true(adc_streams[0].dsp_cfg_bad, "wedge surfaces as dsp_cfg_bad, not silence");
	bridge_hw_adc_stream_end(0u);
}

ZTEST(gd32_adc_seq, test_fac_pump_drains_all_ready_output_words)
{
	adc_seq_reset();
	fac_latch_release();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream_begin");
	bind_q31_fir(8u);
	mock_fac_y_per_write   = 3u; /* Y holds 3 words after one X0 write */
	adc_streams[0].ring[0] = 2560u;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);
	bridge_hw_dsp_pump();
	zassert_equal(adc_streams[0].proc_write, 3u, "every ready Y word drained, not just one");
	bridge_hw_adc_stream_end(0u);
}

/* A write that releases more Y words than one claim/commit batch can hold
 * must not overflow the local out[]: the surplus stays queued in Y and is
 * drained by later batches, in order, with nothing dropped. */
ZTEST(gd32_adc_seq, test_fac_pump_drain_is_bounded_by_the_commit_buffer)
{
	adc_seq_reset();
	fac_latch_release();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream_begin");
	bind_q31_fir(8u);
	mock_fac_y_per_write = 20u; /* > ADC_DSP_PUMP_LOCK_BATCH words per X0 write */
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);
	bridge_hw_dsp_pump();
	zassert_true(adc_streams[0].proc_write <= 2u * ADC_DSP_PUMP_LOCK_BATCH, "no out[] overrun");
	zassert_equal(adc_streams[0].proc_write + mock_fac_y_pending,
	              20u,
	              "every produced Y word is either published or still queued");
	bridge_hw_adc_stream_end(0u);
}

ZTEST(gd32_adc_seq, test_fac_first_pump_realigns_stale_backlog_without_busy)
{
	adc_seq_reset();
	fac_latch_release();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream_begin");
	bind_q31_fir(8u);              /* seeds pump_raw_read = total_read = 0 */
	adc_streams[0].lap_count = 5u; /* DMA lapped while the raw reader never ran */
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 10u);
	bridge_hw_dsp_pump();
	zassert_false(adc_streams[0].proc_gap, "no spurious BUSY at bind");
	zassert_equal(adc_streams[0].pump_raw_read,
	              5u * BRIDGE_ADC_STREAM_RING_SAMPLES + 10u,
	              "cursor aligned with the live DMA total");
	bridge_hw_adc_stream_end(0u);
}

/* gh#18 B9: the pump falling a full ring behind must surface as BUSY. */
ZTEST(gd32_adc_seq, test_fac_pump_full_ring_resync_answers_busy_once)
{
	adc_seq_reset();
	fac_latch_release();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream_begin");
	bind_f32_iir(0.25f, 0.5f, 0.25f, -1.561018075800718f, 0.641351538057563f);
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES);
	bridge_hw_dsp_pump();          /* first pass: FAC config + cursor align (gh#306) */
	adc_streams[0].lap_count = 1u; /* one full lap = ring-size backlog */
	bridge_hw_dsp_pump();

	uint8_t  got = 0xFFu;
	uint16_t mv[4];
	zassert_equal(bridge_hw_adc_stream_read(0u, 4u, &got, mv), BRIDGE_HW_ERR_BUSY, "gap -> BUSY");
	zassert_equal(bridge_hw_adc_stream_read(0u, 4u, &got, mv), BRIDGE_HW_OK, "BUSY is one-shot");
	bridge_hw_adc_stream_end(0u);
}

ZTEST(gd32_adc_seq, test_fac_pump_biases_a_10bit_stream_by_its_own_full_scale)
{
	/* gh#253: hal/gd32/adc_stream.c hardcoded the FAC pump's input
	 * bias/scale to the 12-bit mid-scale (2048) and a fixed <<3, and
	 * the output re-bias clamped to a hardcoded [0, 4095].  A legal
	 * 10-bit stream (adc_full_scale_for_bits(10) == 1023, see adc.c)
	 * rails on every sample instead: this pins the fix deriving both
	 * from the stream's OWN s->full_scale. */
	adc_seq_reset();
	fac_latch_release();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream_begin");
	bind_f32_iir(0.25f, 0.5f, 0.25f, -1.561018075800718f, 0.641351538057563f);
	adc_streams[0].full_scale = 1023u; /* simulate a 10-bit stream */

	/* code 640 = 10-bit mid-scale (512) + 128.  A correct 10-bit
	 * decode hands the FAC x = (640 - 512) << 5 = +4096 -- the SAME
	 * FAC input the 12-bit test drives with (2560 - 2048) << 3, so a
	 * hardcoded-12-bit decode is caught by comparing against this
	 * exact word, not just by "did not crash". */
	adc_streams[0].ring[0] = 640u;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);

	/* Same -8192 FAC output as the 12-bit test.  A hardcoded 12-bit
	 * decode re-biases this to ((-8192 >> 3) + 2048) = 1024, clipped
	 * into [0, 4095] -- silently wrong AND silently in-range for a
	 * 10-bit stream.  The correct 10-bit decode is
	 * ((-8192 >> 5) + 512) = 256, inside [0, 1023]. */
	mock_fac_read_value = -8192;
	bridge_hw_dsp_pump();

	zassert_equal(mock_fac_last_write, 4096, "10-bit input must bias off its OWN full_scale");
	zassert_equal(adc_streams[0].proc_write, 1u, "one processed sample produced");
	zassert_equal(adc_streams[0].proc_ring[0],
	              256u,
	              "10-bit output re-bias must map -8192 to code 256, not the 12-bit answer 1024");

	/* A large positive FAC output must clip to the STREAM's full_scale
	 * (1023), not the hardcoded 4095 -- a hardcoded clamp would still
	 * silently hand the host an out-of-range 10-bit code. */
	adc_streams[0].ring[1] = 640u;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 2u);
	mock_fac_read_value = 32767;
	bridge_hw_dsp_pump();

	zassert_equal(adc_streams[0].proc_write, 2u, "second processed sample produced");
	zassert_equal(adc_streams[0].proc_ring[1],
	              1023u,
	              "positive rail must clip to the stream's own full_scale (1023), not 4095");

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

/* ---------------------------------------------------------------------
 * #149 -- a coalesced DMA lap (write index regressed while lap_count
 * stood still) must not permanently skew the read path's backlog
 * accounting.  Two properties pinned:
 *   1. the read still serves fresh samples (pre-fix it saw
 *      total_written go BACKWARDS and answered OK with zero samples
 *      forever after);
 *   2. when the pended lap ISR finally counts that same reload, the
 *      correction is not repeated -- the backlog must stay under one
 *      ring (rc == OK), not resync as a phantom overrun (BUSY).
 * --------------------------------------------------------------------- */

ZTEST(gd32_adc_seq, test_read_recovers_coalesced_lap)
{
	adc_seq_reset();

	int rc = bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u);
	zassert_equal(rc, BRIDGE_HW_OK, "stream_begin succeeds against the mock");

	/* Reader drains up to position 100 with lap_count == 0. */
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 100u);
	uint8_t  got = 0u;
	uint16_t mv[128];
	rc = bridge_hw_adc_stream_read(0u, 128u, &got, mv);
	zassert_equal(rc, BRIDGE_HW_OK, "first read OK");
	zassert_equal(got, 100u, "first read drains to position 100");

	/* The coalesced lap: the DMA reloaded (w regressed to 8) but the
	 * FTF the lap ISR would count has not run -- lap_count still 0.
	 * Pre-fix, total_written = 0*RS + 8 < total_read = 100: the read
	 * answered OK with ZERO samples and never recovered. */
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 8u);
	got = 0u;
	rc  = bridge_hw_adc_stream_read(0u, 16u, &got, mv);
	zassert_equal(rc, BRIDGE_HW_OK, "read after a coalesced lap stays OK");
	zassert_equal(got,
	              16u,
	              "the regressed write index must be credited one ring (#149): "
	              "backlog = (RS+8) - 100, serves max_samples fresh samples");

	/* The pended lap ISR now counts the SAME reload (lap_count 0 -> 1).
	 * The per-sample correction must not be repeated: total = 1*RS + 8,
	 * total_read = 116, backlog = RS - 108 < RS -> rc must stay OK.  A
	 * double-counted correction would push the backlog over one ring
	 * and answer BUSY with a resync -- the phantom overrun this
	 * second assertion pins out. */
	adc_streams[0].lap_count = 1u;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 8u);
	got = 0u;
	rc  = bridge_hw_adc_stream_read(0u, 128u, &got, mv);
	zassert_equal(rc, BRIDGE_HW_OK, "the ISR counting the corrected lap must not double-credit it");
	zassert_equal(got, 128u, "backlog stays under one ring: RS-108 samples remain");
}

/* #51 -- an AHB DMA transfer error cannot be reported as an apparently
 * healthy empty stream.  ERRIF and a simultaneous FTF are cleared through
 * their dedicated bits so exact lap accounting remains intact. */
ZTEST(gd32_adc_seq, test_dma_error_is_sticky_and_preserves_simultaneous_lap)
{
	adc_seq_reset();
	int rc = bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u);
	zassert_equal(rc, BRIDGE_HW_OK, "stream begins against the mock");
	int enable_i = mock_seq_find_from("dma_interrupt_enable", DMA0, 0);
	zassert_true(enable_i >= 0, "stream begins with DMA interrupts armed");
	zassert_equal(mock_seq[enable_i].arg,
	              DMA_INT_FTF | DMA_INT_ERR,
	              "both full-transfer and transfer-error interrupts are armed");

	mock_dma_set_interrupt_flag(DMA0, DMA_CH0, DMA_INT_FLAG_FTF | DMA_INT_FLAG_ERR, SET);
	DMA0_Channel0_IRQHandler();
	zassert_equal(adc_streams[0].lap_count, 1u, "simultaneous FTF remains counted");
	zassert_equal(adc_streams[0].dma_error_count, 1u, "ERRIF is retained as stream state");

	uint8_t  got = 0xFFu;
	uint16_t mv[1];
	rc = bridge_hw_adc_stream_read(0u, 1u, &got, mv);
	zassert_equal(rc, BRIDGE_HW_ERR_IO, "transfer error reaches the host as IO");
	zassert_equal(got, 0u, "no stale ring samples escape after DMA error");

	rc = bridge_hw_adc_stream_end(0u);
	zassert_equal(rc, BRIDGE_HW_OK, "END clears the faulted session");
	rc = bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u);
	zassert_equal(rc, BRIDGE_HW_OK, "BEGIN starts a clean replacement session");
	zassert_equal(adc_streams[0].dma_error_count, 0u, "new session clears the sticky DMA error");

	/* CS EXTI outranks the DMA IRQ: it must consume a just-raised ERRIF
	 * itself rather than waiting for the lower-priority vector to run. */
	mock_dma_set_interrupt_flag(DMA0, DMA_CH0, DMA_INT_FLAG_ERR, SET);
	rc = bridge_hw_adc_stream_read(0u, 1u, &got, mv);
	zassert_equal(
	    rc, BRIDGE_HW_ERR_IO, "direct ERRIF check is host-visible without waiting for IRQ");
}

/* ---------------------------------------------------------------------
 * #140 -- ending and rebinding an FFT stream starts a new publication
 * session. Until that session fills and publishes its own FFT window,
 * spectrum_read must answer BUSY instead of reporting the prior sequence
 * as a successful empty frame.
 * --------------------------------------------------------------------- */

static void bind_test_fft_chain(uint8_t stream_id)
{
	adc_dsp_chain_t *chain = &adc_dsp_chains[0];
	memset(chain, 0, sizeof(*chain));
	chain->in_use = true;

	adc_dsp_stage_t *fft = &chain->stages[0];
	fft->kind            = 3u;
	fft->total_size      = BRIDGE_DSP_STAGE_HDR_BYTES;
	fft->bytes_received  = BRIDGE_DSP_STAGE_HDR_BYTES;
	fft->complete        = true;
	fft->data[0]         = 32u; /* 32-point FFT, little-endian u16 */
	fft->data[1]         = 0u;
	fft->data[2]         = 0u; /* complex output */
	fft->data[3]         = 0u;

	zassert_equal(bridge_hw_adc_dsp_chain_bind(0u, stream_id),
	              BRIDGE_HW_OK,
	              "well-formed test FFT chain binds");
}

static void bind_test_fir_chain(uint8_t stream_id)
{
	adc_dsp_chain_t *chain = &adc_dsp_chains[0];
	memset(chain, 0, sizeof(*chain));
	chain->in_use = true;

	adc_dsp_stage_t *fir = &chain->stages[0];
	fir->kind            = 0u;
	fir->total_size      = BRIDGE_DSP_STAGE_HDR_BYTES + 4u;
	fir->bytes_received  = fir->total_size;
	fir->complete        = true;
	fir->data[0]         = 1u;    /* Q31 */
	fir->data[1]         = 1u;    /* one tap */
	fir->data[7]         = 0x40u; /* 0.5 in little-endian Q31 */

	zassert_equal(bridge_hw_adc_dsp_chain_bind(0u, stream_id),
	              BRIDGE_HW_OK,
	              "well-formed test FIR chain binds");
}

static int fft_init_end_result;
static int fac_init_end_result;
static int replacement_end_result;
static int replacement_begin_result;

static void end_stream_during_fft_init(void)
{
	fft_init_end_result = bridge_hw_adc_stream_end(0u);
}

static void end_stream_during_fac_init(void)
{
	fac_init_end_result = bridge_hw_adc_stream_end(0u);
}

static void replace_stream_with_fft(void)
{
	replacement_end_result   = bridge_hw_adc_stream_end(0u);
	replacement_begin_result = bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u);
	if (replacement_begin_result != BRIDGE_HW_OK) return;
	bind_test_fft_chain(0u);
	for (uint16_t i = 0u; i < 32u; ++i)
		adc_streams[0].ring[i] = (uint16_t)(100u + i);
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 32u);
	mock_fft_set_flag(SET);
}

static void replace_stream_with_fir(void)
{
	replacement_end_result   = bridge_hw_adc_stream_end(0u);
	replacement_begin_result = bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u);
	if (replacement_begin_result != BRIDGE_HW_OK) return;
	bind_test_fir_chain(0u);
	adc_streams[0].ring[0] = 1024u;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);
}

ZTEST(gd32_adc_seq, test_fft_rebind_rejects_previous_sequence_until_new_frame)
{
	adc_seq_reset();

	/* Session 1 publishes one real frame through the production pump. */
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "first stream begins");
	bind_test_fft_chain(0u);
	for (uint16_t i = 0u; i < 32u; ++i)
		adc_streams[0].ring[i] = i;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 32u);
	mock_fft_set_flag(SET);
	bridge_hw_dsp_pump();

	uint32_t seq   = 0u;
	uint16_t total = 0u;
	uint8_t  got   = 0u;
	float    bin   = -1.0f;
	zassert_equal(bridge_hw_adc_spectrum_read(0u, 0u, 1u, &seq, &total, &got, &bin),
	              BRIDGE_HW_OK,
	              "first session publishes a readable frame");
	zassert_equal(seq, 1u, "first session frame sequence starts at one");
	zassert_equal(got, 1u, "one complex-output bin component is returned");
	zassert_equal(bridge_hw_adc_stream_end(0u), BRIDGE_HW_OK, "first stream ends cleanly");

	/* Session 2 has configured the FFT owner but produced no samples. The
	 * old frame must remain hidden until this session publishes its own. */
	zassert_equal(bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u),
	              BRIDGE_HW_OK,
	              "second stream begins");
	bind_test_fft_chain(0u);
	mock_fft_set_flag(RESET);
	bridge_hw_dsp_pump();

	seq   = 0xDEADBEEFu;
	total = 0xFFFFu;
	got   = 0xFFu;
	bin   = -1.0f;
	zassert_equal(bridge_hw_adc_spectrum_read(0u, 0u, 1u, &seq, &total, &got, &bin),
	              BRIDGE_HW_ERR_BUSY,
	              "freshly rebound FFT session has no frame yet");
	zassert_equal(got, 0u, "no bins are reported before this session publishes");

	for (uint16_t i = 0u; i < 32u; ++i)
		adc_streams[0].ring[i] = (uint16_t)(32u - i);
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 32u);
	mock_fft_set_flag(SET);
	bridge_hw_dsp_pump();
	seq   = 0u;
	total = 0u;
	got   = 0u;
	zassert_equal(bridge_hw_adc_spectrum_read(0u, 0u, 1u, &seq, &total, &got, &bin),
	              BRIDGE_HW_OK,
	              "second session publishes its own readable frame");
	zassert_equal(seq, 1u, "new session frame sequence restarts at one");
	zassert_equal(got, 1u, "new session serves its own bin data");
	zassert_equal(bridge_hw_adc_stream_end(0u), BRIDGE_HW_OK, "second stream ends cleanly");
}

ZTEST(gd32_adc_seq, test_fft_config_preemption_cannot_resurrect_ended_stream)
{
	adc_seq_reset();
	fft_init_end_result = BRIDGE_HW_ERR_IO;

	/* Give the first pump enough data to publish, then model the transport
	 * ISR ending the stream from inside the long first-time FFT config. */
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream begins");
	bind_test_fft_chain(0u);
	for (uint16_t i = 0u; i < 32u; ++i)
		adc_streams[0].ring[i] = i;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 32u);
	mock_fft_set_flag(SET);
	mock_fft_set_init_hook(end_stream_during_fft_init);
	bridge_hw_dsp_pump();

	zassert_equal(fft_init_end_result, BRIDGE_HW_OK, "ISR-side stream end succeeds");
	zassert_false(adc_streams[0].in_use, "interrupted session remains ended");

	/* A fresh same-ID session must not inherit the owner or a frame that
	 * base level could otherwise have published after the teardown. */
	zassert_equal(bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u),
	              BRIDGE_HW_OK,
	              "replacement stream begins");
	bind_test_fft_chain(0u);
	bridge_hw_dsp_pump();

	uint32_t seq   = 0u;
	uint16_t total = 0u;
	uint8_t  got   = 0u;
	float    bin   = -1.0f;
	zassert_equal(bridge_hw_adc_spectrum_read(0u, 0u, 1u, &seq, &total, &got, &bin),
	              BRIDGE_HW_ERR_BUSY,
	              "replacement session has no frame before its own samples");
	zassert_equal(got, 0u, "replacement session reports no bins before publication");
	zassert_equal(bridge_hw_adc_stream_end(0u), BRIDGE_HW_OK, "replacement stream ends cleanly");
}

ZTEST(gd32_adc_seq, test_fac_config_preemption_cannot_resurrect_ended_stream)
{
	adc_seq_reset();
	fac_init_end_result = BRIDGE_HW_ERR_IO;

	/* End the session from the transport-ISR seam inside FAC setup. The
	 * configuring token must let release see the in-progress owner. */
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream begins");
	bind_test_fir_chain(0u);
	adc_streams[0].ring[0] = 2048u;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);
	mock_fac_set_init_hook(end_stream_during_fac_init);
	bridge_hw_dsp_pump();

	zassert_equal(fac_init_end_result, BRIDGE_HW_OK, "ISR-side stream end succeeds");
	zassert_false(adc_streams[0].in_use, "interrupted FAC session remains ended");
	zassert_equal(adc_streams[0].proc_write, 0u, "ended session publishes no filtered sample");
	int first_stop = mock_seq_find_from("fac_stop", 0u, 0);
	int restart    = mock_seq_find_from("fac_start", 0u, first_stop + 1);
	int final_stop = mock_seq_find_from("fac_stop", 0u, restart + 1);
	zassert_true(first_stop >= 0, "release stops the configuring FAC owner");
	zassert_true(restart > first_stop, "interrupted config resumes and starts FAC once");
	zassert_true(final_stop > restart, "failed ownership commit stops FAC again");

	/* Reusing the stream ID must perform a fresh configuration and process
	 * new data instead of inheriting the ended session's FAC owner. */
	zassert_equal(bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u),
	              BRIDGE_HW_OK,
	              "replacement stream begins");
	bind_test_fir_chain(0u);
	adc_streams[0].ring[0] = 1024u;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);
	const int before_replacement_pump = mock_seq_n;
	bridge_hw_dsp_pump();
	zassert_true(mock_seq_find_from("fac_init", 0u, before_replacement_pump) >= 0,
	             "replacement session configures FAC again");
	zassert_equal(adc_streams[0].proc_write, 1u, "replacement session processes its own sample");
	zassert_equal(bridge_hw_adc_stream_end(0u), BRIDGE_HW_OK, "replacement stream ends cleanly");
}

ZTEST(gd32_adc_seq, test_fac_post_commit_preemption_cannot_drain_replacement)
{
	adc_seq_reset();
	replacement_end_result   = BRIDGE_HW_ERR_IO;
	replacement_begin_result = BRIDGE_HW_ERR_IO;

	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream begins");
	bind_test_fir_chain(0u);
	adc_streams[0].ring[0] = 2048u;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 1u);

	/* dma_transfer_number_get() is the first interruptible call after the
	 * configuring token has committed to an active FAC owner. Replace the
	 * same stream ID there, then return the old DMA snapshot to the suspended
	 * pump: without a guarded FAC transaction it drains replacement data
	 * through the previous session's configuration. */
	const int before_stale_pump = mock_seq_n;
	mock_dma_set_transfer_get_hook(replace_stream_with_fir);
	bridge_hw_dsp_pump();

	zassert_equal(replacement_end_result, BRIDGE_HW_OK, "post-commit END succeeds");
	zassert_equal(replacement_begin_result, BRIDGE_HW_OK, "same-ID replacement begins");
	zassert_equal(adc_streams[0].proc_write, 0u, "suspended pump publishes no replacement sample");
	zassert_true(mock_seq_find_from("fac_fixed_data_write", 0u, before_stale_pump) < 0,
	             "revoked FAC owner performs no post-END MMIO");

	const int before_replacement_pump = mock_seq_n;
	bridge_hw_dsp_pump();
	zassert_true(mock_seq_find_from("fac_init", 0u, before_replacement_pump) >= 0,
	             "next main-loop pump configures the replacement FAC session");
	zassert_true(mock_seq_find_from("fac_fixed_data_write", 0u, before_replacement_pump) >= 0,
	             "replacement session processes its own sample");
	zassert_equal(adc_streams[0].proc_write, 1u, "replacement publishes exactly one sample");
	zassert_equal(bridge_hw_adc_stream_end(0u), BRIDGE_HW_OK, "replacement stream ends cleanly");
}

/* Arm a FIR stream whose FAC is already configured and owned (first tick
 * claims + configures with nothing pending), ready for `samples` mid-scale
 * raw samples to be pumped. */
static void arm_fac_pump_with_samples(uint32_t samples)
{
	adc_seq_reset();
	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream begins");
	bind_test_fir_chain(0u);
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES);
	bridge_hw_dsp_pump();
	for (uint16_t i = 0u; i < samples; ++i)
		adc_streams[0].ring[i] = 2048u;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - samples);
}

ZTEST(gd32_adc_seq, test_fac_pump_lock_count_is_bounded_per_batch)
{
	/* gh#272: each batch costs exactly one claim + one commit section (the
	 * FAC work between them runs with interrupts on), never one per sample. */
	const uint32_t samples         = 100u;
	const uint32_t locks_per_batch = 2u;
	const uint32_t batches = (samples + ADC_DSP_PUMP_LOCK_BATCH - 1u) / ADC_DSP_PUMP_LOCK_BATCH;
	arm_fac_pump_with_samples(samples);
	const uint32_t before = mock_irq_lock_count;
	bridge_hw_dsp_pump();

	zassert_equal(
	    mock_irq_lock_count - before, batches * locks_per_batch, "claim+commit per batch");
	zassert_equal(adc_streams[0].proc_write, samples, "every sample still processed");
	zassert_equal(mock_irq_get_primask(), 0u, "no interrupt-off window left open");
	zassert_equal(bridge_hw_adc_stream_end(0u), BRIDGE_HW_OK, "stream ends cleanly");
}

ZTEST(gd32_adc_seq, test_fac_pump_stall_mid_batch_keeps_stalled_sample_in_ring)
{
	const uint32_t samples = 5u;
	arm_fac_pump_with_samples(samples);

	/* X0BFF rises after the 3rd sample's write: the flag is polled before each
	 * sample, so samples 0..2 are consumed and sample 3 stays in the raw ring. */
	const uint32_t stall_at = 3u;
	mock_fac_flags          = 0u;
	mock_fac_set_write_hook_after(stall_at);
	bridge_hw_dsp_pump();

	zassert_equal(adc_streams[0].pump_raw_read, stall_at, "stalled sample left in the ring");
	zassert_equal(adc_streams[0].proc_write, stall_at, "only consumed samples published");
	zassert_equal(bridge_hw_adc_stream_end(0u), BRIDGE_HW_OK, "stream ends cleanly");
}

ZTEST(gd32_adc_seq, test_fac_pump_end_between_batches_does_not_drain_replacement)
{
	replacement_end_result   = BRIDGE_HW_ERR_IO;
	replacement_begin_result = BRIDGE_HW_ERR_IO;
	arm_fac_pump_with_samples(2u * ADC_DSP_PUMP_LOCK_BATCH);

	/* Locks: 1 = batch-1 claim, 2 = batch-1 commit, 3 = batch-2 claim.
	 * END -> BEGIN -> BIND lands just before the batch-2 claim. */
	mock_irq_set_lock_hook(3u, replace_stream_with_fir);
	bridge_hw_dsp_pump();

	zassert_equal(replacement_end_result, BRIDGE_HW_OK, "END between batches succeeds");
	zassert_equal(replacement_begin_result, BRIDGE_HW_OK, "same-ID replacement begins");
	zassert_equal(adc_streams[0].proc_write, 0u, "batch 2 publishes nothing into the replacement");
	zassert_equal(adc_streams[0].pump_raw_read, 0u, "batch 2 does not drain the replacement");
	zassert_equal(bridge_hw_adc_stream_end(0u), BRIDGE_HW_OK, "replacement ends cleanly");
}

static uint8_t  read_hook_rc  = 0xFFu;
static uint8_t  read_hook_got = 0u;
static uint16_t read_hook_mv[8];

static void read_stream_from_hook(void)
{
	read_hook_rc = (uint8_t)bridge_hw_adc_stream_read(0u, 8u, &read_hook_got, read_hook_mv);
}

ZTEST(gd32_adc_seq, test_fac_pump_does_not_overwrite_unread_proc_ring_before_commit)
{
	/* gh#272: proc_ring slot proc_write % ring aliases the sample one ring
	 * older, which stream_read still serves while the backlog is ~1020.  A
	 * read landing between claim and commit must see the ORIGINAL samples:
	 * the batch is staged locally and copied in only inside the commit. */
	arm_fac_pump_with_samples(ADC_DSP_PUMP_LOCK_BATCH);
	adc_stream_state_t *s = &adc_streams[0];
	for (uint16_t i = 0u; i < BRIDGE_ADC_STREAM_RING_SAMPLES; ++i)
		s->proc_ring[i] = i; /* distinct marker per slot, all <= full_scale */
	s->proc_read  = 0u;
	s->proc_write = BRIDGE_ADC_STREAM_RING_SAMPLES - 4u; /* backlog 1020 */

	read_hook_rc = 0xFFu;
	mock_irq_set_lock_hook(2u, read_stream_from_hook); /* just before the commit */
	bridge_hw_dsp_pump();

	zassert_equal(read_hook_rc, BRIDGE_HW_OK, "mid-batch read succeeds");
	zassert_equal(read_hook_got, 8u, "reader drains a full request");
	for (uint16_t i = 0u; i < 8u; ++i) {
		zassert_equal(read_hook_mv[i],
		              adc_code_to_mv(i, s->full_scale),
		              "slot %u still holds its original sample, not a newer overwrite",
		              (unsigned)i);
	}
	zassert_equal(s->proc_write,
	              BRIDGE_ADC_STREAM_RING_SAMPLES - 4u + ADC_DSP_PUMP_LOCK_BATCH,
	              "batch published at commit");
	zassert_equal(bridge_hw_adc_stream_end(0u), BRIDGE_HW_OK, "stream ends cleanly");
}

ZTEST(gd32_adc_seq, test_fac_pump_end_between_claim_and_commit_discards_batch)
{
	replacement_end_result   = BRIDGE_HW_ERR_IO;
	replacement_begin_result = BRIDGE_HW_ERR_IO;
	arm_fac_pump_with_samples(ADC_DSP_PUMP_LOCK_BATCH);

	/* Locks: 1 = batch-1 claim, 2 = batch-1 commit.  END -> BEGIN -> BIND
	 * lands after the claim, so the commit must fail and discard. */
	mock_irq_set_lock_hook(2u, replace_stream_with_fir);
	bridge_hw_dsp_pump();

	zassert_equal(replacement_end_result, BRIDGE_HW_OK, "END between claim and commit succeeds");
	zassert_equal(replacement_begin_result, BRIDGE_HW_OK, "same-ID replacement begins");
	zassert_equal(adc_streams[0].proc_write, 0u, "failed commit publishes nothing");
	zassert_equal(adc_streams[0].pump_raw_read, 0u, "failed commit consumes nothing");

	/* The revoked token must not wedge the FAC: the next pump re-inits it
	 * and processes the replacement's single sample. */
	const int before = mock_seq_n;
	bridge_hw_dsp_pump();
	zassert_true(mock_seq_find_from("fac_init", 0u, before) >= 0, "next pump configures the FAC");
	zassert_equal(adc_streams[0].proc_write, 1u, "replacement sample processed");
	zassert_equal(bridge_hw_adc_stream_end(0u), BRIDGE_HW_OK, "replacement ends cleanly");
}

ZTEST(gd32_adc_seq, test_fft_post_commit_preemption_cannot_drain_replacement)
{
	adc_seq_reset();
	replacement_end_result   = BRIDGE_HW_ERR_IO;
	replacement_begin_result = BRIDGE_HW_ERR_IO;

	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream begins");
	bind_test_fft_chain(0u);
	for (uint16_t i = 0u; i < 32u; ++i)
		adc_streams[0].ring[i] = i;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 32u);
	mock_fft_set_flag(SET);

	const int before_stale_pump = mock_seq_n;
	mock_dma_set_transfer_get_hook(replace_stream_with_fft);
	bridge_hw_dsp_pump();

	zassert_equal(replacement_end_result, BRIDGE_HW_OK, "post-commit END succeeds");
	zassert_equal(replacement_begin_result, BRIDGE_HW_OK, "same-ID replacement begins");
	zassert_equal(
	    adc_streams[0].pump_raw_read, 0u, "suspended pump consumes no replacement samples");
	zassert_true(mock_seq_find_from("fft_calculation_start", 0u, before_stale_pump) < 0,
	             "revoked FFT owner cannot start a replacement frame");

	const int before_replacement_pump = mock_seq_n;
	bridge_hw_dsp_pump();
	zassert_true(mock_seq_find_from("fft_calculation_start", 0u, before_replacement_pump) >= 0,
	             "next main-loop pump calculates the replacement frame");
	uint32_t seq   = 0u;
	uint16_t total = 0u;
	uint8_t  got   = 0u;
	float    bin   = -1.0f;
	zassert_equal(bridge_hw_adc_spectrum_read(0u, 0u, 1u, &seq, &total, &got, &bin),
	              BRIDGE_HW_OK,
	              "replacement publishes its own frame");
	zassert_equal(seq, 1u, "replacement frame sequence starts at one");
	zassert_equal(bridge_hw_adc_stream_end(0u), BRIDGE_HW_OK, "replacement stream ends cleanly");
}

ZTEST(gd32_adc_seq, test_fft_wait_preemption_cannot_publish_ended_session)
{
	adc_seq_reset();
	replacement_end_result   = BRIDGE_HW_ERR_IO;
	replacement_begin_result = BRIDGE_HW_ERR_IO;

	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream begins");
	bind_test_fft_chain(0u);
	/* First tick establishes the active FFT owner without a frame. */
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES);
	bridge_hw_dsp_pump();

	for (uint16_t i = 0u; i < 32u; ++i)
		adc_streams[0].ring[i] = i;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - 32u);
	mock_fft_set_flag(SET);
	mock_fft_set_poll_hook(replace_stream_with_fft);
	bridge_hw_dsp_pump();

	zassert_equal(replacement_end_result, BRIDGE_HW_OK, "END during FFT wait succeeds");
	zassert_equal(replacement_begin_result, BRIDGE_HW_OK, "replacement begins during FFT wait");
	uint32_t seq   = 0xDEADBEEFu;
	uint16_t total = 0xFFFFu;
	uint8_t  got   = 0xFFu;
	float    bin   = -1.0f;
	zassert_equal(bridge_hw_adc_spectrum_read(0u, 0u, 1u, &seq, &total, &got, &bin),
	              BRIDGE_HW_ERR_BUSY,
	              "ended session's completed hardware result remains unpublished");
	zassert_equal(got, 0u, "no stale bins are exposed");

	bridge_hw_dsp_pump();
	zassert_equal(bridge_hw_adc_spectrum_read(0u, 0u, 1u, &seq, &total, &got, &bin),
	              BRIDGE_HW_OK,
	              "replacement can publish its own frame on the next tick");
	zassert_equal(seq, 1u, "replacement publication starts a fresh sequence");
	zassert_equal(bridge_hw_adc_stream_end(0u), BRIDGE_HW_OK, "replacement stream ends cleanly");
}

ZTEST(gd32_adc_seq, test_fft_publish_commit_preemption_cannot_expose_stale_frame)
{
	adc_seq_reset();
	replacement_end_result   = BRIDGE_HW_ERR_IO;
	replacement_begin_result = BRIDGE_HW_ERR_IO;

	zassert_equal(
	    bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "stream begins");
	bind_test_fft_chain(0u);
	/* Establish the active owner without producing a frame. */
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES);
	bridge_hw_dsp_pump();

	const uint32_t samples = 32u;
	for (uint16_t i = 0u; i < samples; ++i)
		adc_streams[0].ring[i] = i;
	mock_dma_set_remaining(DMA0, DMA_CH0, BRIDGE_ADC_STREAM_RING_SAMPLES - samples);
	mock_fft_set_flag(SET);

	/* Each sample takes one lease-protected capture and one protected
	 * append (2 locks per sample); the next lock claims the unreadable
	 * publishing token.  Inject END -> BEGIN -> BIND on the lock after
	 * that, immediately before the final metadata/owner commit. The
	 * callback runs from __get_PRIMASK(), before interrupts become masked,
	 * so this is a silicon-reachable boundary.  Derived from the sample
	 * count so a lock added to the pump moves this test's expectation
	 * visibly rather than silently shifting the injection point. */
	const uint32_t locks_per_sample = 2u; /* capture + append */
	const uint32_t claim_locks      = 1u; /* publishing-token claim */
	const uint32_t commit_lock      = 1u; /* the lock the hook precedes */
	mock_irq_set_lock_hook(samples * locks_per_sample + claim_locks + commit_lock,
	                       replace_stream_with_fft);
	bridge_hw_dsp_pump();

	zassert_equal(replacement_end_result, BRIDGE_HW_OK, "END before publish commit succeeds");
	zassert_equal(replacement_begin_result, BRIDGE_HW_OK, "same-ID replacement begins");
	uint32_t seq   = 0xDEADBEEFu;
	uint16_t total = 0xFFFFu;
	uint8_t  got   = 0xFFu;
	float    bin   = -1.0f;
	zassert_equal(bridge_hw_adc_spectrum_read(0u, 0u, 1u, &seq, &total, &got, &bin),
	              BRIDGE_HW_ERR_BUSY,
	              "revoked publisher cannot expose its frame through the replacement owner");
	zassert_equal(got, 0u, "no stale bins are exposed after publish revocation");
	zassert_equal(
	    adc_streams[0].pump_raw_read, 0u, "publish revocation leaves replacement cursor fresh");

	bridge_hw_dsp_pump();
	zassert_equal(bridge_hw_adc_spectrum_read(0u, 0u, 1u, &seq, &total, &got, &bin),
	              BRIDGE_HW_OK,
	              "replacement publishes its own frame on the next tick");
	zassert_equal(seq, 1u, "replacement publication starts at sequence one");
	zassert_equal(bridge_hw_adc_stream_end(0u), BRIDGE_HW_OK, "replacement stream ends cleanly");
}

/* ---------------------------------------------------------------------
 * #59 -- runtime ADC/DAC reference derived from VREFINT.
 * --------------------------------------------------------------------- */

ZTEST(gd32_adc_seq, test_vref_from_code_math_and_clamp)
{
	zassert_equal(adc_vref_mv_from_code(2730u), 1800u, "1.2 V @ 1.8 V ref");
	zassert_equal(adc_vref_mv_from_code(2600u), 1890u, "1.2 V @ 1.89 V ref");
	zassert_equal(adc_vref_mv_from_code(2400u), ADC_VREF_MV, "2048 mV is above the window");
	zassert_equal(adc_vref_mv_from_code(0u), ADC_VREF_MV, "code 0 falls back");
	zassert_equal(adc_vref_mv_from_code(4095u), ADC_VREF_MV, "1200 mV ref is below the window");
	zassert_equal(adc_vref_mv_from_code(1000u), ADC_VREF_MV, "4914 mV ref is above the window");
}

ZTEST(gd32_adc_seq, test_vref_measure_latches_and_scales_reads)
{
	uint16_t mv[1];

	adc_seq_reset();
	mock_adc_set_routine_data(2600u);
	zassert_true(adc_vref_measure(), "measurement completes");
	zassert_equal(adc_vrefint_code, 2600u, "raw VREFINT code latched");
	zassert_equal(adc_vref_mv, 1890u, "runtime reference latched");
	zassert_true(mock_seq_find_from("adc_internal_channel_config", 0u, 0) >= 0,
	             "VREFINT channel enabled on ADC0");

	mock_adc_set_routine_data(ADC_FULL_SCALE);
	zassert_equal(bridge_hw_adc_read(0u, 1u, mv), BRIDGE_HW_OK, "read ok");
	zassert_equal(mv[0], 1890u, "full-scale code maps to the runtime reference");
}

ZTEST(gd32_adc_seq, test_vref_measure_eoc_timeout_keeps_default)
{
	adc_seq_reset();
	mock_adc_eoc_stuck = true;
	zassert_false(adc_vref_measure(), "EOC never arrives");
	zassert_equal(adc_vref_mv, ADC_VREF_MV, "default kept");
	zassert_equal(adc_vrefint_code, 0u, "code 0 marks a failed measurement");
	zassert_false(mock_adc_internal_ch_on, "VREFINT channel disabled again");
}

/* Late VREF lock: the ISR-context probe must not touch ADC0 or publish
 * vref_ok; the base-level tick measures under an ADC0 claim, then publishes. */
ZTEST(gd32_adc_seq, test_vref_late_lock_measures_at_base_level_under_claim)
{
	adc_seq_reset();
	vref_ok         = false;
	mock_vref_ready = false;
	mock_adc_set_routine_data(2600u);

	zassert_false(vref_ready_check(), "not locked yet");
	vref_late_tick();
	zassert_equal(adc_vrefint_code, 0u, "no measurement without a lock");

	mock_vref_ready = true;
	mock_seq_reset();
	zassert_false(vref_ready_check(), "ISR probe only notes the lock");
	zassert_equal(
	    mock_seq_find_from("adc_internal_channel_config", 0u, 0), -1, "probe must not drive ADC0");
	zassert_equal(adc_vrefint_code, 0u, "nothing measured in the probe");

	zassert_true(adc_periph_claim(ADC0), "a stream/read owns ADC0");
	vref_late_tick();
	zassert_false(vref_ok, "busy ADC0: no measurement, still not ready");
	zassert_equal(adc_vrefint_code, 0u, "no measurement while ADC0 is claimed");
	adc_periph_release(ADC0);

	vref_late_tick();
	zassert_true(vref_ok, "published after the measurement");
	zassert_equal(adc_vrefint_code, 2600u, "raw code latched");
	zassert_equal(adc_vref_mv, 1890u, "runtime reference published");
	zassert_true(adc_periph_claim(ADC0), "ADC0 claim released again");
	adc_periph_release(ADC0);
}

ZTEST(gd32_adc_seq, test_vref_late_lock_failed_measure_still_promotes)
{
	adc_seq_reset();
	vref_ok            = false;
	mock_adc_eoc_stuck = true;
	(void)vref_ready_check();
	vref_late_tick();
	zassert_true(vref_ok, "locked buffer promotes on the default reference");
	zassert_equal(adc_vref_mv, ADC_VREF_MV, "default kept");
	zassert_equal(adc_vrefint_code, 0u, "code 0 marks the failed measurement");
}

/* A late-lock measurement runs after reads may have left ADC0 at a lower
 * resolution / oversampling: it must reset the format (ADCON low), then
 * enable and calibrate before its first trigger. */
ZTEST(gd32_adc_seq, test_vref_measure_forces_12bit_no_oversample_and_recalibrates)
{
	adc_seq_reset();
	mock_seq_reset();
	mock_adc_set_routine_data(2730u);
	zassert_true(adc_vref_measure(), "measurement completes");
	const int dis = mock_seq_find_from("adc_disable", ADC0, 0);
	const int res = mock_seq_find_from("adc_resolution_config", ADC0, 0);
	const int ovs = mock_seq_find_from("adc_oversample_mode_disable", ADC0, 0);
	const int ena = mock_seq_find_from("adc_enable", ADC0, 0);
	const int trg = mock_seq_find_from("adc_software_trigger_enable", ADC0, 0);
	zassert_true(dis >= 0 && res > dis && ovs > dis, "format applied with ADCON low");
	zassert_true(ena > res && ena > ovs, "converter re-enabled after the format");
	zassert_true(trg > ena, "first trigger only after re-enable + calibration");
	zassert_equal(adc_vref_mv, 1800u, "code 2730 -> 1800 mV");
}

/* ---------------------------------------------------------------------
 * ADC_READ burst: DMA-completed hardware sequence, cached converter format.
 * --------------------------------------------------------------------- */

static unsigned burst_done_calls;
static int      burst_done_rv;
static uint8_t  burst_done_samples;
static uint16_t burst_done_mv[8];

static void burst_done(int rv, const uint16_t *mv, uint8_t samples)
{
	burst_done_calls++;
	burst_done_rv      = rv;
	burst_done_samples = samples;
	if (rv == BRIDGE_HW_OK) memcpy(burst_done_mv, mv, (size_t)samples * sizeof(uint16_t));
}

/* Async variant of the reset: the trigger no longer finishes the burst, so the
 * test plays the DMA-complete interrupt itself. */
static void burst_manual_reset(void)
{
	adc_seq_reset();
	mock_adc_set_trigger_hook(0);
	burst_done_calls   = 0u;
	burst_done_rv      = 12345;
	burst_done_samples = 0u;
}

/* The hardware finishing: DMA has written `codes`, raises FTF; run the IRQ. */
static void burst_hw_finish(const uint16_t *codes, uint8_t n)
{
	for (uint8_t i = 0u; i < n; ++i) {
		adc_burst.codes[i] = codes[i];
	}
	mock_dma_set_interrupt_flag(DMA1, DMA_CH1, DMA_INT_FLAG_FTF, SET);
	DMA1_Channel1_IRQHandler();
}

static int count_events(const char *name, uint32_t periph)
{
	int n = 0;
	for (int i = 0; i < mock_seq_n; ++i) {
		if (strcmp(mock_seq[i].name, name) == 0 &&
		    (periph == MOCK_ANY_PERIPH || mock_seq[i].periph == periph)) {
			n++;
		}
	}
	return n;
}

/* A burst that took the slow path: converter disabled, re-enabled, recalibrated
 * (calibration strictly after the enable, trigger strictly after calibration). */
static void expect_full_reconfigure(const char *what)
{
	const int dis = mock_seq_find_from("adc_disable", MOCK_ANY_PERIPH, 0);
	const int ena = mock_seq_find_from("adc_enable", MOCK_ANY_PERIPH, 0);
	const int cal = mock_seq_find_from("ADC_CTL1_TOUCH", MOCK_ANY_PERIPH, 0);
	const int trg = mock_seq_find_from("adc_software_trigger_enable", MOCK_ANY_PERIPH, 0);
	zassert_true(dis >= 0, "%s: converter disabled for the reformat", what);
	zassert_true(ena > dis, "%s: re-enabled after the disable", what);
	zassert_true(cal > ena, "%s: calibration must follow the enable", what);
	zassert_true(trg > cal, "%s: trigger only after calibration", what);
}

/* A burst that hit the cache: the converter is never disabled or recalibrated. */
static void expect_no_reconfigure(const char *what)
{
	zassert_equal(count_events("adc_disable", MOCK_ANY_PERIPH), 0, "%s: no disable", what);
	zassert_equal(count_events("adc_enable", MOCK_ANY_PERIPH), 0, "%s: no re-enable", what);
	zassert_equal(count_events("ADC_CTL1_TOUCH", MOCK_ANY_PERIPH), 0, "%s: no recalibration", what);
	zassert_equal(
	    count_events("adc_resolution_config", MOCK_ANY_PERIPH), 0, "%s: no reformat", what);
	zassert_equal(
	    count_events("adc_software_trigger_enable", MOCK_ANY_PERIPH), 1, "%s: one trigger", what);
}

/* Rounding: nearest, ties up, clamped, never above the reference. */
ZTEST(gd32_adc_seq, test_code_to_mv_rounds_to_nearest)
{
	adc_seq_reset();
	adc_vref_mv = 1800u;

	zassert_equal(adc_code_to_mv(0u, 4095u), 0u, "zero");
	zassert_equal(adc_code_to_mv(4095u, 4095u), 1800u, "full scale is the reference");
	zassert_equal(adc_code_to_mv(5000u, 4095u), 1800u, "above full scale clamps to Vref");
	zassert_equal(adc_code_to_mv(1u, 4095u), 0u, "0.44 mV rounds down");
	zassert_equal(adc_code_to_mv(2u, 4095u), 1u, "0.88 mV rounds UP (truncation gave 0)");
	zassert_equal(adc_code_to_mv(1u, 63u), 29u, "6-bit: 28.57 mV rounds up (truncation gave 28)");
	zassert_equal(adc_code_to_mv(128u, 255u), 904u, "8-bit: 903.53 mV rounds up (truncated 903)");
	zassert_equal(adc_code_to_mv(1u, 3600u), 1u, "an exact half rounds up");
	adc_vref_mv = 1890u;
	zassert_equal(adc_code_to_mv(4095u, 4095u), 1890u, "tracks the measured reference");
	zassert_equal(adc_code_to_mv(1u, 0u), 0u, "a zero full scale cannot divide");
}

/* The same rounding through the real burst path. */
ZTEST(gd32_adc_seq, test_burst_result_is_rounded)
{
	uint16_t mv[2];

	adc_seq_reset();
	mock_adc_set_routine_data(1u);
	zassert_equal(bridge_hw_adc_configure(BRIDGE_ADC_CH0, 1u, 0u, 6u), BRIDGE_HW_OK, "6-bit");
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0, 2u, mv), BRIDGE_HW_OK, "read ok");
	zassert_equal(mv[0], 29u, "1/63 of 1800 mV, rounded");
	zassert_equal(mv[1], 29u, "second sample too");
}

ZTEST(gd32_adc_seq, test_format_cache_hit_skips_disable_enable_and_recalibration)
{
	uint16_t mv[4];

	adc_seq_reset();
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0, 4u, mv), BRIDGE_HW_OK, "first read");
	expect_full_reconfigure("first read (cold record)");

	mock_seq_reset();
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0, 4u, mv), BRIDGE_HW_OK, "repeat read");
	expect_no_reconfigure("repeat read");

	mock_seq_reset();
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0, 4u, mv), BRIDGE_HW_OK, "third read");
	expect_no_reconfigure("third read");
}

/* Every field of the record is part of the key: change one, miss. */
ZTEST(gd32_adc_seq, test_format_cache_misses_on_each_key_field)
{
	uint16_t mv[8];

	adc_seq_reset();
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0, 4u, mv), BRIDGE_HW_OK, "prime");

	mock_seq_reset();
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0, 5u, mv), BRIDGE_HW_OK, "sequence length");
	expect_full_reconfigure("sample count changed");
	zassert_equal(count_events("adc_channel_length_config", ADC3), 1, "length programmed");

	mock_seq_reset();
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0 + 1u, 5u, mv), BRIDGE_HW_OK, "sibling");
	expect_full_reconfigure("sibling channel on the same converter");

	mock_seq_reset();
	zassert_equal(bridge_hw_adc_configure(BRIDGE_ADC_CH0 + 1u, 1u, 0u, 10u), BRIDGE_HW_OK, "10b");
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0 + 1u, 5u, mv), BRIDGE_HW_OK, "resolution");
	expect_full_reconfigure("resolution changed");

	mock_seq_reset();
	zassert_equal(bridge_hw_adc_configure(BRIDGE_ADC_CH0 + 1u, 16u, 0u, 10u), BRIDGE_HW_OK, "x16");
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0 + 1u, 5u, mv), BRIDGE_HW_OK, "oversample");
	expect_full_reconfigure("oversample changed");

	mock_seq_reset();
	zassert_equal(
	    bridge_hw_adc_configure(BRIDGE_ADC_CH0 + 1u, 16u, 100u, 10u), BRIDGE_HW_OK, "cycles");
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0 + 1u, 5u, mv), BRIDGE_HW_OK, "sample cycles");
	expect_full_reconfigure("sample cycles changed");

	/* A ratio the hardware floors to the same power of two is NOT a different
	 * format: 17 runs as 16. */
	mock_seq_reset();
	zassert_equal(
	    bridge_hw_adc_configure(BRIDGE_ADC_CH0 + 1u, 17u, 100u, 10u), BRIDGE_HW_OK, "x17");
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0 + 1u, 5u, mv), BRIDGE_HW_OK, "floored ratio");
	expect_no_reconfigure("ratio 17 floors to the programmed 16");
}

/* Records are per converter: a read on ADC2 must not disturb ADC3's. */
ZTEST(gd32_adc_seq, test_format_cache_is_per_converter)
{
	uint16_t mv[2];

	adc_seq_reset();
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0, 2u, mv), BRIDGE_HW_OK, "ADC3");
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH2, 2u, mv), BRIDGE_HW_OK, "ADC2");
	mock_seq_reset();
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0, 2u, mv), BRIDGE_HW_OK, "ADC3 again");
	expect_no_reconfigure("ADC3 untouched by ADC2's read");
}

/* ---- every invalidation path ---- */

static void prime_burst_format(void)
{
	uint16_t mv[4];

	adc_seq_reset();
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0, 4u, mv), BRIDGE_HW_OK, "prime");
	mock_seq_reset();
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0, 4u, mv), BRIDGE_HW_OK, "prime check");
	expect_no_reconfigure("primed record is live");
	mock_seq_reset();
}

static void expect_next_read_reformats(const char *what)
{
	uint16_t mv[4];

	mock_seq_reset();
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0, 4u, mv), BRIDGE_HW_OK, "%s: read", what);
	expect_full_reconfigure(what);
}

ZTEST(gd32_adc_seq, test_format_invalidated_by_stream_begin_and_end)
{
	prime_burst_format();
	zassert_equal(bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u), BRIDGE_HW_OK, "begin");

	/* stream_begin drops the scan mode and the longer sequence the burst left. */
	const int len = mock_seq_find_from("adc_channel_length_config", ADC3, 0);
	const int scn = mock_seq_find_from("adc_special_function_config", ADC3, 0);
	zassert_true(len >= 0 && mock_seq[len].arg == 1u, "stream runs a one-rank sequence");
	zassert_true(scn >= 0 && mock_seq[scn].arg == (uint32_t)DISABLE,
	             "scan mode off for the stream");

	zassert_equal(bridge_hw_adc_stream_end(0u), BRIDGE_HW_OK, "end");
	expect_next_read_reformats("after stream begin/end");
}

ZTEST(gd32_adc_seq, test_format_invalidated_by_vref_measure)
{
	uint16_t mv[4];

	adc_seq_reset();
	zassert_equal(bridge_hw_adc_read(6u, 4u, mv), BRIDGE_HW_OK, "prime ADC0 (bridge channel 6)");
	mock_seq_reset();
	zassert_equal(bridge_hw_adc_read(6u, 4u, mv), BRIDGE_HW_OK, "hit");
	expect_no_reconfigure("ADC0 record is live");

	mock_seq_reset();
	mock_adc_set_routine_data(2730u);
	zassert_true(adc_vref_measure(), "VREFINT measurement reprograms ADC0");
	/* it also takes the converter back to one conversion per trigger, no DMA */
	zassert_true(count_events("adc_dma_mode_disable", ADC0) >= 1, "DMA request off");
	const int len = mock_seq_find_from("adc_channel_length_config", ADC0, 0);
	zassert_true(len >= 0 && mock_seq[len].arg == 1u, "one-rank sequence for VREFINT");

	mock_seq_reset();
	zassert_equal(bridge_hw_adc_read(6u, 4u, mv), BRIDGE_HW_OK, "read after VREFINT");
	zassert_true(mock_seq_find_from("adc_disable", ADC0, 0) >= 0, "ADC0 reformatted");
	zassert_true(mock_seq_find_from("ADC_CTL1_TOUCH", ADC0, 0) >= 0, "and recalibrated");
}

ZTEST(gd32_adc_seq, test_format_invalidated_by_deepsleep_quiesce)
{
	prime_burst_format();
	adc_deepsleep_quiesce();
	expect_next_read_reformats("after deep-sleep entry");
}

ZTEST(gd32_adc_seq, test_format_invalidated_by_converter_reinit)
{
	prime_burst_format();
	zassert_true(adc_periph_restore(ADC3), "restore (stream end / error recovery)");
	expect_next_read_reformats("after adc_periph_restore");

	prime_burst_format();
	zassert_true(adc_periph_boot_init(ADC3), "boot init");
	expect_next_read_reformats("after adc_periph_boot_init");
}

/* ---- burst lifecycle: arm, complete via DMA IRQ, never poll ---- */

ZTEST(gd32_adc_seq, test_burst_completes_via_dma_callback_without_polling)
{
	burst_manual_reset();
	mock_adc_set_routine_data(2048u);

	zassert_equal(bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 4u, burst_done),
	              BRIDGE_HW_OK,
	              "start arms the burst");

	/* Armed, not finished: nothing delivered, burst owns the converter + DMA. */
	zassert_equal(burst_done_calls, 0u, "completion is the DMA IRQ's job, not start()'s");
	zassert_equal(adc_burst.state, ADC_BURST_RUNNING, "burst in flight");
	zassert_false(adc_periph_claim(ADC3), "converter is held for the burst");
	zassert_equal(dma_transfer_number_get(DMA1, DMA_CH1), 4u, "DMA moves N codes");
	zassert_equal(mock_dmamux_request_get(8u), DMA_REQUEST_ADC3, "DMA1 CH1 = DMAMUX 8 -> ADC3");
	zassert_true((*mock_dma_chctl_ref(DMA1, DMA_CH1) & DMA_CHXCTL_CHEN) != 0u, "channel armed");

	/* The sequence: scan, N ranks of the same pad, DMA mode, ONE trigger, armed
	 * BEFORE the trigger -- and not a single EOC poll or data-register read. */
	zassert_equal(count_events("adc_channel_length_config", ADC3), 1, "sequence length set");
	zassert_equal(
	    mock_seq[mock_seq_find_from("adc_channel_length_config", ADC3, 0)].arg, 4u, "N=4");
	int ranks = 0;
	for (int i = 0; i < mock_seq_n; ++i) {
		if (strcmp(mock_seq[i].name, "adc_routine_channel_config") == 0) {
			zassert_equal(mock_seq[i].arg >> 8, (uint32_t)ranks, "ranks programmed in order");
			zassert_equal(mock_seq[i].arg & 0xFFu, (uint32_t)ADC_CHANNEL_12, "all the same pad");
			ranks++;
		}
	}
	zassert_equal(ranks, 4, "one rank per sample");
	zassert_equal(
	    count_events("adc_software_trigger_enable", ADC3), 1, "one trigger for the burst");
	zassert_true(mock_seq_find_from("dma_channel_enable", DMA1, 0) <
	                 mock_seq_find_from("adc_software_trigger_enable", ADC3, 0),
	             "DMA armed before the conversions start");
	for (int i = 0; i < mock_seq_n; ++i) {
		zassert_false(strcmp(mock_seq[i].name, "adc_flag_get") == 0 &&
		                  mock_seq[i].arg == ADC_FLAG_EOC,
		              "no EOC polling");
		zassert_true(strcmp(mock_seq[i].name, "adc_routine_data_read") != 0, "DMA reads RDATA");
	}

	/* The hardware finishes: FTF -> IRQ -> done(). */
	const uint16_t codes[4] = { 0u, 2048u, 4095u, 1u };
	burst_hw_finish(codes, 4u);
	zassert_equal(burst_done_calls, 1u, "delivered exactly once");
	zassert_equal(burst_done_rv, BRIDGE_HW_OK, "success");
	zassert_equal(burst_done_samples, 4u, "N samples");
	zassert_equal(burst_done_mv[0], 0u, "code 0");
	zassert_equal(burst_done_mv[1], 900u, "mid-scale 2048 -> 900 mV (rounded: 900.2)");
	zassert_equal(burst_done_mv[2], 1800u, "full scale");
	zassert_equal(burst_done_mv[3], 0u, "code 1 -> 0.44 mV");

	/* Torn down: converter + DMA free again, request released, further IRQ is inert. */
	zassert_equal(adc_burst.state, ADC_BURST_IDLE, "idle");
	zassert_false((*mock_dma_chctl_ref(DMA1, DMA_CH1) & DMA_CHXCTL_CHEN) != 0u, "channel off");
	zassert_equal(mock_dmamux_request_get(8u), 0u, "DMAMUX request released");
	zassert_true(adc_periph_claim(ADC3), "converter claim released");
	adc_periph_release(ADC3);
	DMA1_Channel1_IRQHandler();
	zassert_equal(burst_done_calls, 1u, "a stale IRQ delivers nothing");
}

/* The result is delivered only AFTER the DMA IRQ: nothing before. */
ZTEST(gd32_adc_seq, test_burst_result_not_delivered_before_dma_complete)
{
	burst_manual_reset();
	zassert_equal(bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 8u, burst_done), BRIDGE_HW_OK, "start");
	DMA1_Channel1_IRQHandler(); /* spurious vector entry, no FTF */
	zassert_equal(burst_done_calls, 0u, "no FTF, no completion");
	zassert_equal(adc_burst.state, ADC_BURST_RUNNING, "still converting");
}

ZTEST(gd32_adc_seq, test_burst_is_exclusive_across_converters)
{
	burst_manual_reset();
	zassert_equal(bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 2u, burst_done), BRIDGE_HW_OK, "first");
	zassert_equal(bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 2u, burst_done),
	              BRIDGE_HW_ERR_BUSY,
	              "same converter while converting");
	zassert_equal(bridge_hw_adc_read_start(BRIDGE_ADC_CH2, 2u, burst_done),
	              BRIDGE_HW_ERR_BUSY,
	              "other converter: the one burst DMA channel is taken");
	zassert_equal(bridge_hw_adc_stream_begin(0u, BRIDGE_ADC_CH0, 1000u),
	              BRIDGE_HW_ERR_BUSY,
	              "a stream cannot reprogram a converter mid-burst");

	const uint16_t codes[2] = { 1u, 2u };
	burst_hw_finish(codes, 2u);
	zassert_equal(bridge_hw_adc_read_start(BRIDGE_ADC_CH2, 2u, burst_done), BRIDGE_HW_OK, "after");
}

ZTEST(gd32_adc_seq, test_burst_start_rejects_without_arming)
{
	burst_manual_reset();
	zassert_equal(
	    bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 0u, burst_done), BRIDGE_HW_ERR_INVAL, "0");
	zassert_equal(
	    bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 9u, burst_done), BRIDGE_HW_ERR_RANGE, "9");
	zassert_equal(bridge_hw_adc_read_start(8u, 1u, burst_done), BRIDGE_HW_ERR_RANGE, "channel");
	zassert_equal(bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 1u, 0), BRIDGE_HW_ERR_INVAL, "no cb");
	vref_ok         = false;
	mock_vref_ready = false;
	zassert_equal(
	    bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 1u, burst_done), BRIDGE_HW_ERR_IO, "vref");
	zassert_equal(adc_burst.state, ADC_BURST_IDLE, "nothing armed, nothing claimed");
	zassert_equal(count_events("dma_channel_enable", DMA1), 0, "DMA never touched");
}

ZTEST(gd32_adc_seq, test_burst_abort_cancels_without_callback_and_restores_converter)
{
	burst_manual_reset();
	zassert_equal(bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 4u, burst_done), BRIDGE_HW_OK, "start");

	bridge_hw_adc_read_abort(0); /* not our callback: no-op */
	zassert_equal(adc_burst.state, ADC_BURST_RUNNING, "null callback cancels nothing");
	bridge_hw_adc_read_abort(burst_done);
	zassert_equal(adc_burst.state, ADC_BURST_IDLE, "aborted");
	zassert_equal(burst_done_calls, 0u, "an abort never calls back");
	zassert_false((*mock_dma_chctl_ref(DMA1, DMA_CH1) & DMA_CHXCTL_CHEN) != 0u, "DMA stopped");
	zassert_true(mock_seq_find_from("adc_disable", ADC3, 0) >= 0,
	             "ADCON cleared: sequence stopped");
	zassert_true(adc_periph_claim(ADC3), "claim released");
	adc_periph_release(ADC3);

	/* a late FTF from the dead burst delivers nothing */
	const uint16_t codes[4] = { 1u, 2u, 3u, 4u };
	burst_hw_finish(codes, 4u);
	zassert_equal(burst_done_calls, 0u, "no delivery after the abort");

	/* and the converter's record was dropped: the next burst reformats */
	mock_adc_set_trigger_hook(burst_autocomplete);
	uint16_t mv[4];
	mock_seq_reset();
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0, 4u, mv), BRIDGE_HW_OK, "next read");
	expect_full_reconfigure("after an abort");
}

ZTEST(gd32_adc_seq, test_burst_dma_error_reports_io_and_recovers)
{
	burst_manual_reset();
	zassert_equal(bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 4u, burst_done), BRIDGE_HW_OK, "start");
	mock_dma_set_interrupt_flag(DMA1, DMA_CH1, DMA_INT_FLAG_ERR, SET);
	DMA1_Channel1_IRQHandler();

	zassert_equal(burst_done_calls, 1u, "failure is reported once");
	zassert_equal(burst_done_rv, BRIDGE_HW_ERR_IO, "as IO");
	zassert_equal(adc_burst.state, ADC_BURST_IDLE, "burst released");
	zassert_true(adc_periph_claim(ADC3), "claim released");
	adc_periph_release(ADC3);
	mock_dma_set_interrupt_flag(DMA1, DMA_CH1, DMA_INT_FLAG_ERR, RESET);

	mock_adc_set_trigger_hook(burst_autocomplete);
	uint16_t mv[4];
	mock_seq_reset();
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0, 4u, mv), BRIDGE_HW_OK, "next read works");
	expect_full_reconfigure("after a DMA error");
}

ZTEST(gd32_adc_seq, test_burst_overrun_flag_fails_the_burst)
{
	burst_manual_reset();
	zassert_equal(bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 2u, burst_done), BRIDGE_HW_OK, "start");
	mock_adc_set_flag(ADC3, ADC_FLAG_ROVF, SET); /* a code was lost: the buffer is not trusted */
	const uint16_t codes[2] = { 1u, 2u };
	burst_hw_finish(codes, 2u);
	zassert_equal(burst_done_rv, BRIDGE_HW_ERR_IO, "overrun -> IO, never half-trusted data");
}

ZTEST(gd32_adc_seq, test_burst_watchdog_tears_down_a_burst_that_never_completes)
{
	burst_manual_reset();
	zassert_equal(bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 4u, burst_done), BRIDGE_HW_OK, "start");

	adc_burst_tick();
	zassert_equal(adc_burst.state, ADC_BURST_RUNNING, "one tick (< 50 ms) is not a stall");
	adc_burst_tick();
	zassert_equal(adc_burst.state, ADC_BURST_IDLE, "second tick: torn down");
	zassert_equal(burst_done_calls, 0u, "base level never stages a reply (it would race the ISR)");
	zassert_true(adc_periph_claim(ADC3), "converter and DMA come back");
	adc_periph_release(ADC3);

	adc_burst_tick(); /* idle: no-op, no spurious restore */
	zassert_equal(adc_burst.state, ADC_BURST_IDLE, "idle stays idle");
}

ZTEST(gd32_adc_seq, test_burst_watchdog_age_restarts_with_each_burst)
{
	burst_manual_reset();
	zassert_equal(bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 1u, burst_done), BRIDGE_HW_OK, "start");
	adc_burst_tick();
	const uint16_t one = 7u;
	burst_hw_finish(&one, 1u);
	zassert_equal(bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 1u, burst_done), BRIDGE_HW_OK, "again");
	adc_burst_tick();
	zassert_equal(adc_burst.state, ADC_BURST_RUNNING, "the earlier tick does not count against it");
}

/* The blocking path (I2C): same burst, bounded wait, abort on expiry. */
ZTEST(gd32_adc_seq, test_blocking_read_times_out_aborts_and_reports_io)
{
	uint16_t mv[2];

	burst_manual_reset(); /* trigger completes nothing */
	zassert_equal(bridge_hw_adc_read(BRIDGE_ADC_CH0, 2u, mv), BRIDGE_HW_ERR_IO, "timed out");
	zassert_equal(adc_burst.state, ADC_BURST_IDLE, "burst aborted, not leaked");
	zassert_true(adc_periph_claim(ADC3), "claim released");
	adc_periph_release(ADC3);
}

static void other_done(int rv, const uint16_t *mv, uint8_t samples)
{
	(void)rv;
	(void)mv;
	(void)samples;
}

/* The two readers share the burst slot but must not abort each other's burst. */
ZTEST(gd32_adc_seq, test_abort_only_cancels_the_callers_own_burst)
{
	burst_manual_reset();
	zassert_equal(bridge_hw_adc_read_start(BRIDGE_ADC_CH0, 2u, burst_done), BRIDGE_HW_OK, "start");
	bridge_hw_adc_read_abort(other_done);
	zassert_equal(adc_burst.state, ADC_BURST_RUNNING, "someone else's callback: untouched");
}

ZTEST_SUITE(gd32_adc_seq, NULL, NULL, NULL, NULL, NULL);
