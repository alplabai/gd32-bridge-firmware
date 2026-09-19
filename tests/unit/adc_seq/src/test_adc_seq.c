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

ZTEST_SUITE(gd32_adc_seq, NULL, NULL, NULL, NULL, NULL);
