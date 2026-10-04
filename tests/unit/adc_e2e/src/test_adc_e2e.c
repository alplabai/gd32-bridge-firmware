/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * End to end: CMD_ADC_READ over SPI through the REAL transport_spi.c +
 * protocol.c + hal/gd32/adc.c, with only the vendor register layer mocked.
 *
 * WHY.  The two halves are each tested against a fake of the other: adc_seq
 * drives adc.c with hand-built callbacks, adc_burst_reply drives protocol.c over
 * a fake HAL.  Neither can see the contract that lives BETWEEN them:
 * protocol_deferred_abort() cancels a burst by handing bridge_hw_adc_read_abort()
 * the SAME callback pointer that bridge_hw_adc_read_start() stored, and adc.c
 * honours the abort only on an exact pointer match.  A refactor that gives
 * protocol.c a second completion function (or wraps it) would make every abort a
 * silent no-op, leaving the burst running and the converter claimed until the
 * watchdog -- and both halves' own suites would stay green.  Here the pointer is
 * whatever protocol.c really registered, so that mismatch fails.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "bridge_hw.h"
#include "gd32_common.h"
#include "protocol.h"
#include "transport.h"

#include "gd32g5x3.h" /* mock -- tests/unit/adc_seq/mock/ */

#define FRAME_CAP (1u + 1u + GD32_BRIDGE_MAX_PAYLOAD_BYTES + 2u)

#define ADC3_SLOT 3u /* bridge channel 0 -> ADC3 */

/* The gd32 backend arms TX DMA here; this double drains the staged reply like it. */
static unsigned hook_calls;
static size_t   hook_len;
static uint8_t  hook_frame[FRAME_CAP];

void bridge_transport_spi_reply_staged(void)
{
	hook_calls++;
	hook_len = 0u;
	while (spi_slave_tx_pending() && hook_len < sizeof(hook_frame)) {
		hook_frame[hook_len++] = spi_slave_tx_next_byte();
	}
}

static void reset(void)
{
	adc_deepsleep_quiesce();
	mock_seq_reset();
	mock_dma_reset();
	mock_dwt_cycles             = 0u;
	mock_dwt_step               = 1u;
	bridge_core_clock_hz        = 216000000u;
	mock_dwt_ptr()->CTRL        = DWT_CTRL_CYCCNTENA_Msk;
	mock_coredebug_ptr()->DEMCR = CoreDebug_DEMCR_TRCENA_Msk;
	mock_adc_set_trigger_hook(0); /* the burst runs until the test plays the DMA IRQ */
	for (uint8_t c = 0u; c < 8u; ++c) {
		adc_sample_cycles_cache[c]    = ADC_DEFAULT_SAMPLE_CYCLES;
		adc_resolution_bits_cache[c]  = ADC_RES_BITS_DEFAULT;
		adc_oversample_ratio_cache[c] = 1u;
	}
	vref_ok         = true;
	adc_vref_mv     = ADC_VREF_MV;
	mock_vref_ready = true;
	mock_adc_set_flag(ADC3, ADC_FLAG_ROVF, RESET);
	mock_adc_set_flag(ADC3, ADC_FLAG_EOC, RESET);

	transport_spi_init();
	const uint8_t off[1] = { 0u };
	uint8_t       rp[GD32_BRIDGE_MAX_PAYLOAD_BYTES];
	size_t        rl = 0u;
	(void)protocol_dispatch(GD32_BRIDGE_LINK_SPI, CMD_LINK_FEATURES, off, 1u, rp, sizeof rp, &rl);
	hook_calls = 0u;
	hook_len   = 0u;
}

static void send_request(uint8_t cmd, const uint8_t *payload, size_t len)
{
	uint8_t f[FRAME_CAP];
	f[0] = GD32_BRIDGE_SOF;
	f[1] = cmd;
	if (len > 0u) memcpy(&f[2], payload, len);
	const uint16_t crc = crc16_ccitt_false(f, 2u + len);
	f[2u + len]        = (uint8_t)(crc & 0xFFu);
	f[3u + len]        = (uint8_t)(crc >> 8);

	spi_slave_cs_low();
	for (size_t i = 0u; i < 4u + len; ++i) {
		spi_slave_rx_byte(f[i]);
	}
	spi_slave_cs_high();
}

static void adc_read_request(uint8_t channel, uint8_t samples)
{
	const uint8_t p[2] = { channel, samples };
	send_request(CMD_ADC_READ, p, 2u);
}

/* The hardware finishing ADC3's burst: DMA wrote `codes`, FTF is up, run the IRQ. */
static void hw_finish(const uint16_t *codes, uint8_t n)
{
	for (uint8_t i = 0u; i < n; ++i) {
		adc_burst[ADC3_SLOT].codes[i] = codes[i];
	}
	mock_dma_set_interrupt_flag(DMA1, DMA_CH4, DMA_INT_FLAG_FTF, SET);
	DMA1_Channel4_IRQHandler();
}

static bool dma_armed(void)
{
	return (*mock_dma_chctl_ref(DMA1, DMA_CH4) & DMA_CHXCTL_CHEN) != 0u;
}

/* The request arms the real burst; its DMA-complete IRQ stages the real reply. */
ZTEST(gd32_adc_e2e, test_spi_adc_read_completes_through_the_real_burst)
{
	reset();
	adc_read_request(0u, 2u);
	zassert_equal(adc_burst[ADC3_SLOT].state, ADC_BURST_RUNNING, "armed by the request");
	zassert_equal(hook_calls, 0u, "no reply before the DMA completes");
	zassert_false(spi_slave_tx_pending(), "nothing staged");

	const uint16_t codes[2] = { 0u, 4095u };
	hw_finish(codes, 2u);

	zassert_equal(hook_calls, 1u, "the completion staged the reply");
	const uint8_t want[] = { GD32_BRIDGE_SOF, STATUS_OK, 2u, 0u, 0u, 0x08u, 0x07u };
	zassert_equal(hook_len, sizeof want + 2u, "SOF STATUS n 2xu16 CRC");
	zassert_mem_equal(hook_frame, want, sizeof want, "0 mV, 1800 mV");
	zassert_equal(adc_burst[ADC3_SLOT].state, ADC_BURST_IDLE, "burst released");
}

/* THE contract: a newer request cancels the burst through the callback protocol.c
 * registered.  If adc.c did not recognise it the burst would still be RUNNING. */
ZTEST(gd32_adc_e2e, test_a_newer_request_aborts_the_burst_by_the_registered_callback)
{
	reset();
	adc_read_request(0u, 2u);
	zassert_equal(adc_burst[ADC3_SLOT].state, ADC_BURST_RUNNING, "armed");
	zassert_true(dma_armed(), "DMA armed");

	send_request(CMD_PING, NULL, 0u);
	zassert_equal(
	    adc_burst[ADC3_SLOT].state, ADC_BURST_IDLE, "protocol_deferred_abort reached adc.c");
	zassert_false(dma_armed(), "DMA stopped");
	zassert_true(adc_periph_claim(ADC3), "converter claim released");
	adc_periph_release(ADC3);

	const uint16_t codes[2] = { 1u, 2u };
	hw_finish(codes, 2u); /* a late completion of the cancelled burst */
	zassert_equal(hook_calls, 0u, "never staged over the PING reply");
}

ZTEST(gd32_adc_e2e, test_a_transport_error_aborts_the_burst_by_the_registered_callback)
{
	reset();
	adc_read_request(0u, 2u);
	zassert_equal(adc_burst[ADC3_SLOT].state, ADC_BURST_RUNNING, "armed");

	spi_slave_transport_error();
	zassert_equal(
	    adc_burst[ADC3_SLOT].state, ADC_BURST_IDLE, "the burst did not outlive its request");
	zassert_false(dma_armed(), "DMA stopped");
	zassert_true(spi_slave_tx_pending(), "STATUS_IO staged");
}

/* Bursts on different converters coexist; the one-reply-in-flight rule of the
 * transport still holds: the newer request cancels the older, whatever converter
 * it was on. */
ZTEST(gd32_adc_e2e, test_newer_request_cancels_the_older_burst_on_another_converter)
{
	reset();
	adc_read_request(0u, 2u); /* ADC3 */
	adc_read_request(2u, 2u); /* ADC2: supersedes it */
	zassert_equal(adc_burst[ADC3_SLOT].state, ADC_BURST_IDLE, "the older request was cancelled");
	zassert_equal(adc_burst[2].state, ADC_BURST_RUNNING, "the newer one runs");
}

ZTEST_SUITE(gd32_adc_e2e, NULL, NULL, NULL, NULL, NULL);
