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

static void adc_seq_reset(void)
{
	/* The owner bytes are file-local production state. Release both stream
	 * IDs before zeroing the visible arrays so a failed prior case cannot
	 * contaminate the next case; clear the resulting mock log afterward. */
	for (uint8_t s = 0u; s < BRIDGE_ADC_STREAM_COUNT; ++s) {
		adc_dsp_fac_release(s);
		adc_dsp_fft_release(s);
	}
	mock_seq_reset();
	memset(mock_adc_ctl1, 0, sizeof mock_adc_ctl1);
	memset(adc_dsp_chains, 0, sizeof adc_dsp_chains);
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
 * #140 -- ending and rebinding an FFT stream starts a new publication
 * session. Until that session fills and publishes its own FFT window,
 * spectrum_read must answer IO/BUSY instead of reporting the prior sequence
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

static void end_stream_during_fft_init(void)
{
	fft_init_end_result = bridge_hw_adc_stream_end(0u);
}

static void end_stream_during_fac_init(void)
{
	fac_init_end_result = bridge_hw_adc_stream_end(0u);
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
	              BRIDGE_HW_ERR_IO,
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
	              BRIDGE_HW_ERR_IO,
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

ZTEST_SUITE(gd32_adc_seq, NULL, NULL, NULL, NULL, NULL);
