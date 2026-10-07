/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * The v0.15 canonical vectors (tests/protocol_vectors.txt section 16) that the
 * STUB-linked tests/unit/protocol_vectors suite cannot reach, because they
 * need a backend that implements ADC_STREAM2 / ATTN: the gd32-shaped 0x1F
 * grant, the ATTN refusal under a debugger, the BEGIN2 / READ2 replies and a
 * BATCH that actually executes.  Links the REAL transport_spi.c ->
 * protocol.c against the injectable fake HAL (tests/unit/fake), and replays
 * the vectors' own bytes through the CS seams exactly as the HAL's CS-EXTI
 * handler would.
 *
 * Replies that the vectors store UNSTAMPED are produced on a link negotiated
 * WITHOUT STATUS_SEQ (a hand-built extended LINK_FEATURES), so the committed
 * bytes are compared as they are.  Every case disarms the link again (the
 * 1-byte `features = 0` form clears the whole word) before returning.
 */

#include <stdbool.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "bridge_hw_fake.h"
#include "protocol.h"
#include "protocol_vectors_gen.h"
#include "transport.h"

ZTEST_SUITE(protocol_vectors_fake, NULL, NULL, NULL, NULL, NULL);

static const pv_vector_t *pv_find(const char *name)
{
	for (size_t i = 0; i < PV_VECTOR_COUNT; i++) {
		if (strcmp(pv_vectors[i].name, name) == 0) return &pv_vectors[i];
	}
	zassert_true(false, "vector %s not found in the generated header", name);
	return NULL;
}

static size_t send(const uint8_t *bytes, size_t len, uint8_t *reply, size_t cap)
{
	transport_spi_init();
	spi_slave_cs_low();
	for (size_t i = 0; i < len; i++) {
		spi_slave_rx_byte(bytes[i]);
	}
	spi_slave_cs_high();
	size_t n = 0;
	while (spi_slave_tx_pending() && n < cap) {
		reply[n++] = spi_slave_tx_next_byte();
	}
	return n;
}

static size_t send_vec(const pv_vector_t *v, uint8_t *reply, size_t cap)
{
	return send(v->bytes, v->len, reply, cap);
}

static void negotiate(uint32_t want, uint16_t mp)
{
	uint8_t f[10] = { GD32_BRIDGE_SOF, CMD_LINK_FEATURES, 0, 0, 0, 0, 0, 0, 0, 0 };
	uint8_t reply[32];

	f[2]               = (uint8_t)(want & 0xFFu);
	f[3]               = (uint8_t)((want >> 8) & 0xFFu);
	f[4]               = (uint8_t)((want >> 16) & 0xFFu);
	f[5]               = (uint8_t)((want >> 24) & 0xFFu);
	f[6]               = (uint8_t)(mp & 0xFFu);
	f[7]               = (uint8_t)(mp >> 8);
	const uint16_t crc = crc16_ccitt_false(f, 8u);
	f[8]               = (uint8_t)(crc & 0xFFu);
	f[9]               = (uint8_t)(crc >> 8);
	(void)send(f, sizeof f, reply, sizeof reply);
}

static void disarm(void)
{
	uint8_t        lf_off[5] = { GD32_BRIDGE_SOF, CMD_LINK_FEATURES, 0x00u, 0, 0 };
	const uint16_t crc       = crc16_ccitt_false(lf_off, 3u);
	uint8_t        reply[16];

	lf_off[3] = (uint8_t)(crc & 0xFFu);
	lf_off[4] = (uint8_t)(crc >> 8);
	(void)send(lf_off, sizeof lf_off, reply, sizeof reply);
}

static void expect_reply(const pv_vector_t *req, const pv_vector_t *want, const char *what)
{
	uint8_t reply[272];
	size_t  n = send_vec(req, reply, sizeof reply);
	zassert_equal(n, want->len, "%s: reply length", what);
	zassert_mem_equal(reply, want->bytes, want->len, "%s: reply bytes", what);
}

/* design 2.2: the gd32 backend grants all five bits. */
ZTEST(protocol_vectors_fake, test_ext_link_features_all_granted)
{
	bridge_hw_fake_reset();
	expect_reply(pv_find("spi_link_features_ext_request_all"),
	             pv_find("spi_link_features_ext_reply_all_seq1"),
	             "gd32-shaped grant");
	zassert_equal(protocol_link_features(GD32_BRIDGE_LINK_SPI), 0x1Fu);
	zassert_equal(protocol_link_max_payload(GD32_BRIDGE_LINK_SPI), 252u);
	zassert_true(bridge_hw_fake_attn_enabled(), "ATTN pin enabled as part of the grant");
	disarm();
	zassert_false(bridge_hw_fake_attn_enabled(), "the 1-byte form clears ATTN again");
}

/* design 2.2 step 3 / 4.2 F2: a debugger on PA14 refuses ATTN, grants the rest. */
ZTEST(protocol_vectors_fake, test_ext_link_features_attn_refused_with_a_debugger)
{
	bridge_hw_fake_reset();
	bridge_hw_fake_set_debugger_attached(1);
	expect_reply(pv_find("spi_link_features_ext_request_all"),
	             pv_find("spi_link_features_ext_reply_attn_refused_seq1"),
	             "debugger attached");
	zassert_false(bridge_hw_fake_attn_enabled(), "PA14 is never driven");
	zassert_equal(bridge_hw_fake_attn_enable_calls(), 0u, "no pin activity at all");
	disarm();
}

ZTEST(protocol_vectors_fake, test_begin2_replies_match_committed_vectors)
{
	bridge_hw_fake_reset();
	negotiate(GD32_BRIDGE_LINK_FEAT_ADC_STREAM2, 65u); /* no stamp: vectors are unstamped */

	bridge_hw_adc_stream2_info_t info = {
		.tick_hz      = 1000000u,
		.period_ticks = 1000u,
		.full_scale   = 4095u,
		.vref_mv      = 1800u,
		.watermark    = 256u,
		.ring_depth   = 512u,
		.flags        = BRIDGE_HW_ADC_STREAM2_FLAG_VREF_MEASURED,
	};
	bridge_hw_fake_begin2_set_info(&info);
	expect_reply(pv_find("spi_adc_stream_begin2_s0_ch0_1khz_w256_request"),
	             pv_find("spi_adc_stream_begin2_reply_1khz_w256"),
	             "BEGIN2 1 kHz W=256");

	uint8_t  stream, channel;
	uint32_t rate;
	uint16_t wm;
	bridge_hw_fake_begin2_get_last(&stream, &channel, &rate, &wm);
	zassert_equal(stream, 0u);
	zassert_equal(channel, 0u);
	zassert_equal(rate, 1000u);
	zassert_equal(wm, 256u);

	/* The 300 Hz example: the realised pace is reported, W = 0 -> ring 1024. */
	info.period_ticks = 3333u;
	info.watermark    = 0u;
	info.ring_depth   = 1024u;
	info.flags        = 0u;
	bridge_hw_fake_begin2_set_info(&info);
	expect_reply(pv_find("spi_adc_stream_begin2_s0_ch0_1khz_w256_request"),
	             pv_find("spi_adc_stream_begin2_reply_300hz_truncation"),
	             "BEGIN2 300 Hz truncation");
	disarm();
}

/* BEGIN2 / READ2 are refused until ADC_STREAM2 is granted on the link. */
ZTEST(protocol_vectors_fake, test_begin2_and_read2_need_the_grant)
{
	uint8_t reply[32];

	bridge_hw_fake_reset();
	disarm();
	const pv_vector_t *nosupp = pv_find("spi_reply_nosupport");
	size_t             n =
	    send_vec(pv_find("spi_adc_stream_begin2_s0_ch0_1khz_w256_request"), reply, sizeof reply);
	zassert_equal(n, nosupp->len);
	zassert_mem_equal(reply, nosupp->bytes, nosupp->len, "BEGIN2 without the grant");
	n = send_vec(pv_find("spi_adc_stream_read2_s0_max121_request"), reply, sizeof reply);
	zassert_equal(n, nosupp->len);
	zassert_mem_equal(reply, nosupp->bytes, nosupp->len, "READ2 without the grant");
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_ADC_STREAM_BEGIN2), 0u);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_ADC_STREAM_READ2), 0u);
}

/* READ2 replies: variable length, the CRC position derived from `got`.  The
 * request asks for max_samples = 121 (needs BIG_FRAME: ceiling (252-9)/2). */
ZTEST(protocol_vectors_fake, test_read2_replies_match_committed_vectors)
{
	const pv_vector_t *req = pv_find("spi_adc_stream_read2_s0_max121_request");

	bridge_hw_fake_reset();
	negotiate(GD32_BRIDGE_LINK_FEAT_ADC_STREAM2 | GD32_BRIDGE_LINK_FEAT_BIG_FRAME, 252u);

	static const uint16_t got3[] = { 0x0800u, 0x0801u, 0x0FFFu };
	bridge_hw_fake_read2_seed(0x100u, 0u, 3u, got3, 0);
	expect_reply(req, pv_find("spi_adc_stream_read2_reply_got3"), "READ2 got 3");
	zassert_equal(bridge_hw_fake_read2_last_max_samples(), 121u);

	bridge_hw_fake_read2_seed(0x103u, 0u, 0u, NULL, 0);
	expect_reply(req, pv_find("spi_adc_stream_read2_reply_empty"), "READ2 empty");

	static const uint16_t got2[] = { 0x0800u, 0x0801u };
	bridge_hw_fake_read2_seed(0x123u, 32u, 2u, got2, 0);
	expect_reply(req, pv_find("spi_adc_stream_read2_reply_overrun_dropped32"), "READ2 overrun");

	bridge_hw_fake_read2_seed(0x125u, 0xFFFFFFFFu, 0u, NULL, 0);
	expect_reply(req, pv_find("spi_adc_stream_read2_reply_discontinuity"), "READ2 discontinuity");
	disarm();
}

/* READ2's ceiling is link-scoped: 121 is above (65-9)/2 = 28 without BIG_FRAME. */
ZTEST(protocol_vectors_fake, test_read2_ceiling_follows_the_negotiated_max_payload)
{
	uint8_t reply[32];

	bridge_hw_fake_reset();
	negotiate(GD32_BRIDGE_LINK_FEAT_ADC_STREAM2, 65u);
	size_t n = send_vec(pv_find("spi_adc_stream_read2_s0_max121_request"), reply, sizeof reply);
	zassert_equal(n, 4u + 0u, "empty-payload envelope");
	zassert_equal(reply[1], STATUS_OUT_OF_RANGE, "max_samples 121 > 28 at max_payload 65");
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_ADC_STREAM_READ2),
	              0u,
	              "the HAL is never reached, nothing consumed");
	disarm();
}

/* A BATCH that executes: three ops, in order, replies packed in place. */
ZTEST(protocol_vectors_fake, test_batch_executes_in_order_and_packs_replies)
{
	uint8_t reply[272];

	bridge_hw_fake_reset();
	negotiate(GD32_BRIDGE_LINK_FEAT_BATCH | GD32_BRIDGE_LINK_FEAT_ADC_STREAM2 |
	              GD32_BRIDGE_LINK_FEAT_BIG_FRAME,
	          252u);
	static const uint16_t codes[] = { 0x0010u, 0x0020u };
	bridge_hw_fake_read2_seed(7u, 0u, 2u, codes, 0);

	size_t n = send_vec(pv_find("spi_batch_request_gpiow_pwmget_read2"), reply, sizeof reply);
	/* A5 | OK | executed=3 | {OK,0} | {OK,8,period+duty} | {OK,13,READ2} | CRC(2) */
	zassert_equal(reply[1], STATUS_OK, "outer OK");
	zassert_equal(reply[2], 3u, "all three executed");
	zassert_equal(reply[3], STATUS_OK);
	zassert_equal(reply[4], 0u, "GPIO_WRITE: no payload");
	zassert_equal(reply[5], STATUS_OK);
	zassert_equal(reply[6], 8u, "PWM_GET: period+duty");
	zassert_equal(reply[7 + 8], STATUS_OK, "READ2 sub-status");
	zassert_equal(reply[7 + 8 + 1], 9u + 2u * 2u, "READ2 sub-reply: 9-byte header + 2 codes");
	zassert_equal(n, 2u + 1u + 2u + (2u + 8u) + (2u + 13u) + 2u, "frame length");
	zassert_equal(bridge_hw_fake_gpio_get_pads() & 1u, 1u, "GPIO_WRITE really ran");
	disarm();
}

/* Execution stops at the first non-OK sub-status; later ops never run. */
ZTEST(protocol_vectors_fake, test_batch_stops_at_the_first_error_per_committed_vector)
{
	bridge_hw_fake_reset();
	negotiate(GD32_BRIDGE_LINK_FEAT_BATCH | GD32_BRIDGE_LINK_FEAT_ADC_STREAM2, 65u);
	bridge_hw_fake_force(FAKE_FN_ADC_STREAM_READ2, BRIDGE_HW_ERR_INVAL); /* stream 1 inactive */

	expect_reply(pv_find("spi_batch_request_gpiow_read2_inactive_ping"),
	             pv_find("spi_batch_reply_stop_at_first_error"),
	             "stop at first error");
	disarm();
}

/* The validation refusals leave NO side effects. */
ZTEST(protocol_vectors_fake, test_rejected_batch_executes_nothing)
{
	uint8_t reply[32];

	bridge_hw_fake_reset();
	negotiate(GD32_BRIDGE_LINK_FEAT_BATCH, 65u);
	const pv_vector_t *inval = pv_find("spi_reply_inval");

	size_t n = send_vec(pv_find("spi_batch_request_trailing_byte_rejected"), reply, sizeof reply);
	zassert_equal(n, inval->len);
	zassert_mem_equal(reply, inval->bytes, inval->len);
	n = send_vec(pv_find("spi_batch_request_nested_rejected"), reply, sizeof reply);
	zassert_equal(n, inval->len);
	zassert_mem_equal(reply, inval->bytes, inval->len);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_GPIO_WRITE), 0u);
	disarm();
}

/* The largest READ2 reply: 121 codes = 251 payload bytes = a 255-byte frame
 * that fits the 256-byte staging, with the CRC at offset 11 + 2*got. */
ZTEST(protocol_vectors_fake, test_read2_full_size_reply_fits_the_256_byte_frame)
{
	const pv_vector_t *req = pv_find("spi_adc_stream_read2_s0_max121_request");
	uint8_t            reply[272];
	uint16_t           codes[121];

	bridge_hw_fake_reset();
	negotiate(GD32_BRIDGE_LINK_FEAT_ADC_STREAM2 | GD32_BRIDGE_LINK_FEAT_BIG_FRAME, 252u);
	for (uint16_t i = 0; i < 121u; i++)
		codes[i] = (uint16_t)(0x0100u + i);
	bridge_hw_fake_read2_seed(1000u, 0u, 121u, codes, 0);

	size_t n = send_vec(req, reply, sizeof reply);
	zassert_equal(n, 255u, "SOF + STATUS + (9 + 2*121) + CRC(2)");
	zassert_equal(reply[1], STATUS_OK);
	zassert_equal(reply[10], 121u, "got @10 of the SPI frame");
	zassert_equal(reply[11] | (reply[12] << 8), 0x0100);
	zassert_equal(reply[11 + 2 * 120] | (reply[12 + 2 * 120] << 8), 0x0100 + 120);
	const uint16_t crc = crc16_ccitt_false(reply, 11u + 2u * 121u);
	zassert_equal(reply[11 + 2 * 121], (uint8_t)(crc & 0xFFu), "CRC follows the last code");
	zassert_equal(reply[12 + 2 * 121], (uint8_t)(crc >> 8));
	disarm();
}
