/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * CMD_ADC_READ over SPI is a DEFERRED command: the CS-rising path only arms the
 * conversion burst, and the burst's DMA-complete interrupt stages the reply.
 * This suite pins the transport half of that contract against the REAL
 * src/transport_spi.c + src/protocol.c and the injectable fake HAL, whose
 * bridge_hw_fake_adc_async_complete() plays the burst interrupt:
 *
 *   - the reply is not visible before completion, and neither is the PREVIOUS
 *     command's reply (a read that lands mid-conversion must see "not ready",
 *     never a stale CRC-valid frame);
 *   - once the interrupt stages it, it is whole and byte-identical to the
 *     synchronous (I2C) reply;
 *   - a newer request, a transport error, the drain bound and a HAL failure each
 *     leave exactly one coherent reply behind and no stranded burst.
 *
 * The HAL-side half (DMA arm, completion IRQ, format cache) is in
 * tests/unit/adc_seq.
 */

#include <string.h>
#include <zephyr/ztest.h>

#include "../../../../hal/bridge_hw.h"
#include "bridge_hw_fake.h"
#include "protocol.h"
#include "transport.h"

#define FRAME_CAP (1u + 1u + GD32_BRIDGE_MAX_PAYLOAD_BYTES + 2u)

/* The gd32 backend arms TX DMA here, draining the staged reply into its flat TX
 * buffer.  This test double does the same drain, so the cursor is spent exactly
 * as on silicon, and records what it was handed. */
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
	bridge_hw_fake_reset();
	transport_spi_init();
	/* Link features outlive transport_spi_init(): disarm STATUS_SEQ so a case that
	 * armed it cannot stamp the next case's replies. */
	const uint8_t off[1] = { 0u };
	uint8_t       rp[GD32_BRIDGE_MAX_PAYLOAD_BYTES];
	size_t        rl = 0u;
	(void)protocol_dispatch(GD32_BRIDGE_LINK_SPI, CMD_LINK_FEATURES, off, 1u, rp, sizeof rp, &rl);
	hook_calls = 0u;
	hook_len   = 0u;
	memset(hook_frame, 0, sizeof hook_frame);
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

/* A host reply-read transaction: the master clocks DUMMY zero bytes. */
static void send_drain(void)
{
	spi_slave_cs_low();
	for (unsigned i = 0u; i < 6u; ++i) {
		spi_slave_rx_byte(0x00u);
	}
	spi_slave_cs_high();
}

/* What the backend would clock out after a transaction: the whole staged reply. */
static size_t take_reply(uint8_t *out)
{
	size_t n = 0u;
	while (spi_slave_tx_pending() && n < FRAME_CAP) {
		out[n++] = spi_slave_tx_next_byte();
	}
	return n;
}

static void adc_read_request(uint8_t channel, uint8_t samples)
{
	const uint8_t p[2] = { channel, samples };
	send_request(CMD_ADC_READ, p, 2u);
}

static bool frame_crc_ok(const uint8_t *f, size_t n)
{
	if (n < 4u) return false;
	const uint16_t got = (uint16_t)f[n - 2u] | (uint16_t)((uint16_t)f[n - 1u] << 8);
	return got == crc16_ccitt_false(f, n - 2u);
}

/* PING first, so a CRC-valid reply for a DIFFERENT command is staged and has
 * been served once: the stale reply a mid-conversion read must not see. */
static void stage_and_serve_ping(void)
{
	uint8_t f[FRAME_CAP];
	send_request(CMD_PING, NULL, 0u);
	zassert_true(spi_slave_tx_pending(), "PING reply staged synchronously");
	zassert_true(take_reply(f) >= 4u, "PING reply clocked out");
}

ZTEST(adc_burst_reply, test_reply_is_not_visible_before_the_burst_completes)
{
	uint8_t f[FRAME_CAP];

	reset();
	stage_and_serve_ping();

	adc_read_request(0u, 3u);
	zassert_true(bridge_hw_fake_adc_async_inflight(), "burst armed by the request");
	zassert_false(spi_slave_tx_pending(), "no reply yet -- and not the previous command's either");
	zassert_equal(hook_calls, 0u, "nothing staged, nothing for the backend to arm");

	/* The host's first reply read lands mid-conversion: it must drain NOTHING
	 * (idle bytes, which the host retries on) rather than re-serve the PING
	 * reply the drain/rewind path would otherwise re-arm. */
	for (unsigned i = 0u; i < 5u; ++i) {
		send_drain();
		zassert_false(spi_slave_tx_pending(), "a drain during conversion serves nothing");
		zassert_equal(take_reply(f), 0u, "not ready");
	}
	zassert_true(bridge_hw_fake_adc_async_inflight(), "still converting");
	zassert_equal(bridge_hw_fake_adc_async_abort_count(), 0u, "drains alone never abort");
}

ZTEST(adc_burst_reply, test_burst_completion_stages_the_whole_reply_atomically)
{
	uint8_t f[FRAME_CAP];

	reset();
	stage_and_serve_ping();
	zassert_equal(bridge_hw_fake_adc_queue_push(100u), 0, "q");
	zassert_equal(bridge_hw_fake_adc_queue_push(1800u), 0, "q");
	zassert_equal(bridge_hw_fake_adc_queue_push(0x1234u), 0, "q");

	adc_read_request(0u, 3u);
	zassert_false(spi_slave_tx_pending(), "pending");

	bridge_hw_fake_adc_async_complete(BRIDGE_HW_OK); /* the DMA-complete interrupt */

	/* The completion path told the backend, and what it was handed is a whole,
	 * CRC-valid envelope -- never a half-written one. */
	zassert_equal(hook_calls, 1u, "the backend is told exactly once");
	const uint8_t want[] = { GD32_BRIDGE_SOF, STATUS_OK, 3u, 100u, 0u, 0x08u, 0x07u, 0x34u, 0x12u };
	zassert_equal(hook_len, sizeof want + 2u, "SOF STATUS n 3xu16 CRC");
	zassert_mem_equal(hook_frame, want, sizeof want, "reply bytes");
	zassert_true(frame_crc_ok(hook_frame, hook_len), "CRC valid");
	zassert_false(bridge_hw_fake_adc_async_inflight(), "burst released");

	/* The backend consumed the cursor into its TX DMA buffer (as on silicon);
	 * the host's reply-read transaction then takes the normal drain/rewind and
	 * is re-served the same frame. */
	zassert_false(spi_slave_tx_pending(), "cursor spent by the backend's drain");
	send_drain();
	zassert_equal(take_reply(f), hook_len, "re-armed in full");
	zassert_mem_equal(f, hook_frame, hook_len, "same reply on re-read");
}

/* The SPI frame carries the same bytes the synchronous (I2C) dispatch builds. */
ZTEST(adc_burst_reply, test_deferred_reply_matches_the_synchronous_reply)
{
	uint8_t       sync_reply[GD32_BRIDGE_MAX_PAYLOAD_BYTES];
	size_t        sync_len = 0u;
	const uint8_t req[2]   = { 2u, 4u };

	reset();
	for (uint16_t v = 10u; v < 50u; v += 10u) {
		zassert_equal(bridge_hw_fake_adc_queue_push(v), 0, "q");
	}
	zassert_equal(
	    protocol_dispatch(
	        GD32_BRIDGE_LINK_I2C, CMD_ADC_READ, req, 2u, sync_reply, sizeof sync_reply, &sync_len),
	    STATUS_OK,
	    "sync");
	zassert_equal(sync_len, 9u, "1 + 4*2");

	for (uint16_t v = 10u; v < 50u; v += 10u) {
		zassert_equal(bridge_hw_fake_adc_queue_push(v), 0, "q");
	}
	adc_read_request(2u, 4u);
	bridge_hw_fake_adc_async_complete(BRIDGE_HW_OK);
	zassert_equal(hook_len, 2u + sync_len + 2u, "envelope around the same payload");
	zassert_mem_equal(&hook_frame[2], sync_reply, sync_len, "identical payload bytes");
}

ZTEST(adc_burst_reply, test_a_newer_request_cancels_the_pending_burst)
{
	uint8_t f[FRAME_CAP];

	reset();
	adc_read_request(0u, 2u);
	zassert_true(bridge_hw_fake_adc_async_inflight(), "armed");

	send_request(CMD_PING, NULL, 0u);
	zassert_equal(bridge_hw_fake_adc_async_abort_count(), 1u, "superseded burst cancelled");
	zassert_false(bridge_hw_fake_adc_async_inflight(), "released");
	zassert_true(take_reply(f) >= 4u, "the PING reply is what is staged");
	zassert_equal(f[1], STATUS_OK, "PING ok");

	bridge_hw_fake_adc_async_complete(BRIDGE_HW_OK); /* nothing in flight: inert */
	zassert_equal(hook_calls, 0u, "no stray ADC reply is staged over the PING reply");
}

/* The drain bound: a burst that never completes must not hold the link silent. */
ZTEST(adc_burst_reply, test_drain_bound_replaces_a_missing_reply_with_io_and_cancels)
{
	uint8_t f[FRAME_CAP];

	reset();
	adc_read_request(0u, 2u);
	for (unsigned i = 0u; i < 12u; ++i) { /* SPI_DRAIN_REWIND_BOUND */
		send_drain();
		zassert_false(spi_slave_tx_pending(), "drain %u: still not ready", i);
	}
	zassert_equal(bridge_hw_fake_adc_async_abort_count(), 0u, "within the bound");
	send_drain(); /* one past the bound */
	zassert_equal(bridge_hw_fake_adc_async_abort_count(), 1u, "burst cancelled");
	zassert_true(spi_slave_tx_pending(), "STATUS_IO staged");
	const size_t n = take_reply(f);
	zassert_equal(n, 4u, "empty-payload envelope");
	zassert_equal(f[1], STATUS_IO, "the host's error-envelope fallback decodes it");
	zassert_true(frame_crc_ok(f, n), "CRC valid");
}

ZTEST(adc_burst_reply, test_transport_error_aborts_the_burst_and_wins_over_a_late_completion)
{
	uint8_t f[FRAME_CAP];

	reset();
	adc_read_request(0u, 2u);
	zassert_true(bridge_hw_fake_adc_async_inflight(), "armed");
	spi_slave_transport_error(); /* DMA ERRIF / RXORERR on the transaction */
	zassert_true(spi_slave_tx_pending(), "STATUS_IO staged");
	zassert_equal(
	    bridge_hw_fake_adc_async_abort_count(), 1u, "the burst no longer runs for nothing");
	zassert_false(bridge_hw_fake_adc_async_inflight(), "converter and DMA released");

	bridge_hw_fake_adc_async_complete(BRIDGE_HW_OK); /* inert: nothing in flight */
	zassert_equal(hook_calls, 0u, "no completion is staged over the error");
	const size_t n = take_reply(f);
	zassert_equal(f[1], STATUS_IO, "the error reply survives");
	zassert_true(frame_crc_ok(f, n), "CRC valid");
}

/* Every other path that replaces the pending reply with STATUS_IO cancels the
 * burst too: a corrupt request (bad CRC), a mangled one (leading byte neither SOF
 * nor the all-zero drain), and a truncated one. */
ZTEST(adc_burst_reply, test_every_error_reply_aborts_a_pending_burst)
{
	uint8_t f[FRAME_CAP];

	reset();
	adc_read_request(0u, 2u);
	spi_slave_cs_low();
	spi_slave_rx_byte(GD32_BRIDGE_SOF);
	spi_slave_rx_byte(CMD_PING);
	spi_slave_rx_byte(0x00u);
	spi_slave_rx_byte(0x00u); /* wrong CRC */
	spi_slave_cs_high();
	zassert_equal(bridge_hw_fake_adc_async_abort_count(), 1u, "bad CRC cancels the burst");
	zassert_false(bridge_hw_fake_adc_async_inflight(), "released");
	zassert_equal(take_reply(f) >= 4u ? f[1] : 0xEEu, STATUS_IO, "STATUS_IO");

	adc_read_request(0u, 2u);
	spi_slave_cs_low();
	spi_slave_rx_byte(0x5Au); /* neither SOF nor the all-zero reply drain */
	spi_slave_rx_byte(0x00u);
	spi_slave_cs_high();
	zassert_equal(bridge_hw_fake_adc_async_abort_count(), 2u, "mangled request cancels it");

	adc_read_request(0u, 2u);
	spi_slave_cs_low();
	spi_slave_rx_byte(GD32_BRIDGE_SOF);
	spi_slave_rx_byte(CMD_PING); /* too short for an envelope */
	spi_slave_cs_high();
	zassert_equal(bridge_hw_fake_adc_async_abort_count(), 3u, "truncated request cancels it");
	zassert_false(bridge_hw_fake_adc_async_inflight(), "nothing left running");
}

ZTEST(adc_burst_reply, test_hal_failure_is_reported_as_its_status)
{
	uint8_t f[FRAME_CAP];

	reset();
	adc_read_request(0u, 2u);
	bridge_hw_fake_adc_async_complete(BRIDGE_HW_ERR_IO);
	zassert_equal(hook_calls, 1u, "staged");
	zassert_equal(hook_frame[1], STATUS_IO, "burst failed -> IO");
	zassert_equal(hook_len, 4u, "no payload");
	(void)f;
}

/* Errors the synchronous read reported at request time are still reported at
 * request time, with no burst armed and nothing left pending. */
ZTEST(adc_burst_reply, test_start_time_errors_reply_immediately)
{
	uint8_t f[FRAME_CAP];

	reset();
	bridge_hw_fake_force(FAKE_FN_ADC_READ, BRIDGE_HW_ERR_BUSY);
	adc_read_request(0u, 2u);
	zassert_false(bridge_hw_fake_adc_async_inflight(), "nothing armed");
	zassert_true(spi_slave_tx_pending(), "reply staged synchronously");
	zassert_true(take_reply(f) >= 4u, "frame");
	zassert_equal(f[1], STATUS_BUSY, "BUSY, as before");

	bridge_hw_fake_force(FAKE_FN_ADC_READ, BRIDGE_HW_ERR_RANGE);
	adc_read_request(0u, 2u);
	take_reply(f);
	zassert_equal(f[1], STATUS_OUT_OF_RANGE, "RANGE, as before");

	bridge_hw_fake_force(FAKE_FN_ADC_READ, 0);
	adc_read_request(0u, 0u);
	take_reply(f);
	zassert_equal(f[1], STATUS_INVAL, "samples == 0 is rejected before any hardware");
	adc_read_request(0u, GD32_BRIDGE_ADC_MAX_SAMPLES + 1u);
	take_reply(f);
	zassert_equal(f[1], STATUS_OUT_OF_RANGE, "over the sample cap");
	zassert_false(bridge_hw_fake_adc_async_inflight(), "none of those armed a burst");
}

/* The wire stays stamped: STATUS_SEQ advances once, when the reply is staged. */
ZTEST(adc_burst_reply, test_status_seq_stamp_advances_once_at_completion)
{
	uint8_t f[FRAME_CAP];

	reset();
	/* arm STATUS_SEQ via CMD_LINK_FEATURES(features = 1) */
	const uint8_t feat[1] = { GD32_BRIDGE_LINK_FEAT_STATUS_SEQ };
	send_request(CMD_LINK_FEATURES, feat, 1u);
	zassert_true(take_reply(f) >= 4u, "granted");
	const uint8_t seq_before = (uint8_t)(f[1] >> GD32_BRIDGE_STATUS_SEQ_SHIFT);

	adc_read_request(0u, 1u);
	zassert_false(spi_slave_tx_pending(), "pending: no stamp consumed yet");
	for (unsigned i = 0u; i < 3u; ++i) {
		send_drain();
	}
	bridge_hw_fake_adc_async_complete(BRIDGE_HW_OK);
	zassert_equal((uint8_t)(hook_frame[1] >> GD32_BRIDGE_STATUS_SEQ_SHIFT),
	              (uint8_t)((seq_before + 1u) & 0x0Fu),
	              "one stamp for the one fresh reply");
	zassert_equal(
	    hook_frame[1] & GD32_BRIDGE_STATUS_CODE_MASK, STATUS_OK, "status in the low bits");
}

ZTEST_SUITE(adc_burst_reply, NULL, NULL, NULL, NULL, NULL);
