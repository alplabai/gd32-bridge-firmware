/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the gd32-bridge SPI-slave transport seams
 * (gd32-bridge-firmware:src/transport_spi.c) -- the CS-framed staging
 * layer between the byte-level HAL and protocol_dispatch().
 *
 * The regression that motivates this suite (silicon, 2026-06-04): the
 * gd32 HAL backend consumes the staged reply through
 * spi_slave_tx_next_byte() at stage time (it drains the bytes into its
 * TX DMA buffer), so when the host's reply read lands BEFORE a slow
 * handler has staged the reply, the resulting all-0x00 "drain"
 * transaction used to find the cursor already spent and re-armed a
 * zero-length reply -- every subsequent re-read clocked 0x00 forever
 * and the command failed 255-of-256 despite the correct reply sitting
 * in the staging buffer.  The drain gate now REWINDS the cursor; these
 * tests pin that contract from the HAL's point of view.
 */

#include <string.h>
#include <zephyr/ztest.h>

#include "protocol.h"
#include "transport.h"

/* PING request and its byte-identical reply: SOF | 0x00 | CRC(0xFF84)
 * emitted lo-byte first on the wire (silicon-verified). */
static const uint8_t ping_frame[] = { 0xA5u, 0x00u, 0x84u, 0xFFu };

/* Replays one CS transaction through the seams, as the HAL's CS-EXTI
 * rising path does: reset staging, feed the captured bytes, decode. */
static void transaction(const uint8_t *bytes, size_t len)
{
	spi_slave_cs_low();
	for (size_t i = 0; i < len; i++) {
		spi_slave_rx_byte(bytes[i]);
	}
	spi_slave_cs_high();
}

/* Drains the staged reply exactly the way hal/transport_hw_gd32.c
 * fills its TX DMA buffer: pull bytes while the transport reports
 * them pending.  Returns the number of bytes drained. */
static size_t hal_drain(uint8_t *out, size_t cap)
{
	size_t n = 0;
	while (spi_slave_tx_pending() && n < cap) {
		out[n++] = spi_slave_tx_next_byte();
	}
	return n;
}

ZTEST(gd32_bridge_transport, test_ping_stages_reply)
{
	uint8_t reply[80];

	transport_spi_init();
	transaction(ping_frame, sizeof ping_frame);

	size_t n = hal_drain(reply, sizeof reply);

	zassert_equal(n, sizeof ping_frame, "PING reply is 4 bytes");
	zassert_mem_equal(
	    reply, ping_frame, sizeof ping_frame, "PING reply is byte-identical to the request");
}

/* The silicon regression: after the HAL consumed the staged reply, an
 * all-0x00 drain transaction must REWIND the cursor so the HAL can
 * re-drain and re-arm the very same reply for the host's re-read. */
ZTEST(gd32_bridge_transport, test_drain_rewinds_consumed_reply)
{
	uint8_t       first[80], again[80];
	const uint8_t zeros[8] = { 0 };

	transport_spi_init();
	transaction(ping_frame, sizeof ping_frame);
	size_t n_first = hal_drain(first, sizeof first);
	zassert_equal(n_first, sizeof ping_frame, "reply staged");
	zassert_false(spi_slave_tx_pending(), "cursor consumed by the HAL drain");

	/* Host read that found nothing useful -> all-0x00 capture. */
	transaction(zeros, sizeof zeros);

	zassert_true(spi_slave_tx_pending(),
	             "drain gate must rewind the staged reply (silicon bug "
	             "2026-06-04: spent cursor disarmed every re-read)");
	size_t n_again = hal_drain(again, sizeof again);
	zassert_equal(n_again, n_first, "full reply re-armed");
	zassert_mem_equal(again, first, n_first, "identical bytes re-armed");
}

/* Repeated drains must be idempotent -- each one re-arms the same
 * reply (the host may re-read several times down its backoff ladder). */
ZTEST(gd32_bridge_transport, test_drain_rearm_is_idempotent)
{
	uint8_t       buf[80];
	const uint8_t zeros[4] = { 0 };

	transport_spi_init();
	transaction(ping_frame, sizeof ping_frame);
	(void)hal_drain(buf, sizeof buf);

	for (int i = 0; i < 3; i++) {
		transaction(zeros, sizeof zeros);
		size_t n = hal_drain(buf, sizeof buf);
		zassert_equal(n, sizeof ping_frame, "re-arm %d intact", i);
		zassert_mem_equal(buf, ping_frame, sizeof ping_frame, "re-arm %d byte-exact", i);
	}
}

/* An EMPTY transaction (CS toggled, zero bytes captured -- the edge-
 * coalescing case where a colliding read's bytes hit the mid-reset SPI
 * and are lost) must also rewind: the gd32 backend re-drains after
 * every rising edge, and a spent cursor would disarm TX and force a
 * guaranteed second miss (silicon 2026-06-04, the ADC double miss). */
ZTEST(gd32_bridge_transport, test_empty_transaction_rewinds)
{
	uint8_t buf[80];

	transport_spi_init();
	transaction(ping_frame, sizeof ping_frame);
	(void)hal_drain(buf, sizeof buf);
	zassert_false(spi_slave_tx_pending(), "cursor consumed");

	/* CS pulse with no captured bytes. */
	transaction(NULL, 0u);

	zassert_true(spi_slave_tx_pending(), "empty transaction must rewind");
	size_t n = hal_drain(buf, sizeof buf);
	zassert_equal(n, sizeof ping_frame, "full reply re-armed");
	zassert_mem_equal(buf, ping_frame, sizeof ping_frame, "byte-exact");
}

/* A fresh request replaces the staged reply -- the rewind must never
 * resurrect a PREVIOUS command's reply once a new one decodes. */
ZTEST(gd32_bridge_transport, test_new_request_replaces_staged_reply)
{
	uint8_t       buf[80];
	const uint8_t zeros[4] = { 0 };

	/* GET_VERSION request: SOF | 0x01 | CRC.  CRC over [A5 01]
     * computed by the linked production crc16_ccitt_false. */
	uint8_t        gv[4] = { 0xA5u, 0x01u, 0, 0 };
	const uint16_t crc   = crc16_ccitt_false(gv, 2u);

	gv[2] = (uint8_t)(crc & 0xFFu);
	gv[3] = (uint8_t)(crc >> 8);

	transport_spi_init();
	transaction(ping_frame, sizeof ping_frame);
	(void)hal_drain(buf, sizeof buf);

	transaction(gv, sizeof gv);
	size_t n = hal_drain(buf, sizeof buf);

	/* GET_VERSION reply: SOF | STATUS | MAJOR MINOR PATCH | CRC = 7 B. */
	zassert_equal(n, 7u, "GET_VERSION reply staged");
	zassert_equal(buf[0], 0xA5u, "SOF");
	zassert_equal(buf[1], 0x00u, "STATUS_OK");

	/* And the drain re-arms the NEW reply, not the old PING. */
	transaction(zeros, sizeof zeros);
	size_t n2 = hal_drain(buf, sizeof buf);
	zassert_equal(n2, 7u, "rewind re-arms the current reply");
}

/* A corrupted (non-SOF, non-zero) capture must stage a loud STATUS_IO
 * error reply -- not preserve the previous reply, not stay silent. */
ZTEST(gd32_bridge_transport, test_mangled_request_stages_io_error)
{
	uint8_t       buf[80];
	const uint8_t garbage[5] = { 0x00u, 0xA5u, 0x12u, 0x34u, 0x56u };

	transport_spi_init();
	transaction(ping_frame, sizeof ping_frame);
	(void)hal_drain(buf, sizeof buf);

	transaction(garbage, sizeof garbage);
	size_t n = hal_drain(buf, sizeof buf);

	zassert_equal(n, 4u, "error reply is the empty envelope");
	zassert_equal(buf[0], 0xA5u, "SOF");
	zassert_equal(buf[1], 0x05u, "STATUS_IO");
}

/* A hardware transport fault -- a DMA ERRIF or an SPI RXORERR overrun -- is
 * not a malformed request we can safely decode: bytes may be missing
 * entirely.  It must replace any previous reply with a fresh STATUS_IO
 * envelope, so the host retries instead of accepting a stale reply from the
 * transaction that preceded the fault. */
ZTEST(gd32_bridge_transport, test_hardware_transport_error_stages_io_error)
{
	uint8_t buf[80];

	transport_spi_init();
	transaction(ping_frame, sizeof ping_frame);
	(void)hal_drain(buf, sizeof buf);

	spi_slave_transport_error();
	const size_t n = hal_drain(buf, sizeof buf);

	zassert_equal(n, 4u, "error reply is the empty envelope");
	zassert_equal(buf[0], 0xA5u, "SOF");
	zassert_equal(buf[1], 0x05u, "STATUS_IO");
	const uint16_t crc = crc16_ccitt_false(buf, 2u);
	zassert_equal(buf[2], (uint8_t)(crc & 0xFFu), "CRC lo");
	zassert_equal(buf[3], (uint8_t)(crc >> 8), "CRC hi");
}

/* ------------------------------------------------------------------ */
/* v0.7 STATUS_SEQ -- the stale-reply kill (silicon-fingerprinted      */
/* 2026-06-06: byte-exact replays on back-to-back identical frames).   */
/* ------------------------------------------------------------------ */

/* Builds + replays a CMD_LINK_FEATURES transaction wanting `feat`. */
static void negotiate(uint8_t feat)
{
	uint8_t        lf[5] = { 0xA5u, 0x81u /* CMD_LINK_FEATURES */, feat, 0, 0 };
	const uint16_t crc   = crc16_ccitt_false(lf, 3u);

	lf[3] = (uint8_t)(crc & 0xFFu);
	lf[4] = (uint8_t)(crc >> 8);
	transaction(lf, sizeof lf);
}

/* The full feature contract in one walk: the negotiation reply itself
 * is stamped (baseline), every fresh decode advances the stamp, the
 * drain/rewind re-serve KEEPS the stamp (that is the stale signature
 * the host detects), error envelopes advance + stamp too, and a
 * disable returns the wire to the legacy unstamped framing. */
ZTEST(gd32_bridge_transport, test_status_seq_stamp_contract)
{
	uint8_t       buf[80];
	const uint8_t zeros[4] = { 0 };

	transport_spi_init();

	/* Negotiate ON: reply carries granted=0x01 and stamp 1 (the
     * feature arms BEFORE the reply stages). */
	negotiate(0x01u);
	size_t n = hal_drain(buf, sizeof buf);
	zassert_equal(n, 5u, "LINK_FEATURES reply: SOF STATUS granted CRC");
	zassert_equal(buf[1], 0x10u, "stamp=1, code=OK");
	zassert_equal(buf[2], 0x01u, "STATUS_SEQ granted");
	/* The CRC covers the STAMPED status byte. */
	{
		const uint16_t crc = crc16_ccitt_false(buf, 3u);
		zassert_equal(buf[3], (uint8_t)(crc & 0xFFu), "stamped CRC lo");
		zassert_equal(buf[4], (uint8_t)(crc >> 8), "stamped CRC hi");
	}

	/* Fresh decode -> stamp advances. */
	transaction(ping_frame, sizeof ping_frame);
	n = hal_drain(buf, sizeof buf);
	zassert_equal(n, 4u, "PING reply staged");
	zassert_equal(buf[1], 0x20u, "stamp=2, code=OK");

	/* Drain/rewind re-serves the SAME stamp -- the stale fingerprint
     * a host uses to know its next request was never decoded. */
	transaction(zeros, sizeof zeros);
	n = hal_drain(buf, sizeof buf);
	zassert_equal(n, 4u, "rewound reply re-armed");
	zassert_equal(buf[1], 0x20u, "stamp UNCHANGED across rewind");

	/* Another fresh decode advances again... */
	transaction(ping_frame, sizeof ping_frame);
	n = hal_drain(buf, sizeof buf);
	zassert_equal(buf[1], 0x30u, "stamp=3 after the next decode");

	/* ...and a transport-error envelope is fresh too. */
	spi_slave_transport_error();
	n = hal_drain(buf, sizeof buf);
	zassert_equal(n, 4u, "error envelope");
	zassert_equal(buf[1], 0x45u, "stamp=4, code=STATUS_IO");

	/* Negotiate OFF: the disable-ack itself is already unstamped
     * (feature disarmed before staging), and the wire returns to the
     * legacy framing. */
	negotiate(0x00u);
	n = hal_drain(buf, sizeof buf);
	zassert_equal(n, 5u, "LINK_FEATURES reply");
	zassert_equal(buf[1], 0x00u, "disable-ack unstamped");
	zassert_equal(buf[2], 0x00u, "nothing granted");

	transaction(ping_frame, sizeof ping_frame);
	n = hal_drain(buf, sizeof buf);
	zassert_mem_equal(
	    buf, ping_frame, sizeof ping_frame, "legacy framing restored (PING byte-identical)");
}

/* The 4-bit stamp wraps 15 -> 0 -> 1; stamp 0 is a VALID value mid-
 * session (host-side inequality still detects staleness across it). */
ZTEST(gd32_bridge_transport, test_status_seq_wraps_mod_16)
{
	uint8_t buf[80];

	transport_spi_init();
	negotiate(0x01u);
	(void)hal_drain(buf, sizeof buf); /* stamp 1 consumed */

	/* 14 more decodes take the stamp to 15. */
	for (int i = 0; i < 14; i++) {
		transaction(ping_frame, sizeof ping_frame);
		(void)hal_drain(buf, sizeof buf);
	}
	transaction(ping_frame, sizeof ping_frame);
	(void)hal_drain(buf, sizeof buf);
	zassert_equal(buf[1] >> 4, 0x0u, "stamp wrapped to 0");

	transaction(ping_frame, sizeof ping_frame);
	(void)hal_drain(buf, sizeof buf);
	zassert_equal(buf[1] >> 4, 0x1u, "and keeps counting");

	negotiate(0x00u); /* leave the suite in legacy framing */
	(void)hal_drain(buf, sizeof buf);
}

/* ------------------------------------------------------------------ */
/* v0.15 -- 256-byte frames, >65 enforcement, the fresh-stage signal   */
/* ------------------------------------------------------------------ */

/* Builds `SOF | cmd | payload | CRC` (lo byte first) into `out`; returns its length. */
static size_t build_frame(uint8_t *out, uint8_t cmd, const uint8_t *payload, size_t payload_len)
{
	out[0] = 0xA5u;
	out[1] = cmd;
	if (payload_len > 0u) memcpy(&out[2], payload, payload_len);
	const uint16_t crc    = crc16_ccitt_false(out, 2u + payload_len);
	out[2u + payload_len] = (uint8_t)(crc & 0xFFu);
	out[3u + payload_len] = (uint8_t)(crc >> 8);
	return 4u + payload_len;
}

/* Extended LINK_FEATURES (no STATUS_SEQ, so replies stay unstamped). */
static void negotiate_ext(uint32_t want, uint16_t mp)
{
	const uint8_t p[6] = { (uint8_t)want,         (uint8_t)(want >> 8), (uint8_t)(want >> 16),
		                   (uint8_t)(want >> 24), (uint8_t)mp,          (uint8_t)(mp >> 8) };
	uint8_t       f[10];
	uint8_t       sink[16];

	transaction(f, build_frame(f, 0x81u, p, sizeof p));
	(void)hal_drain(sink, sizeof sink);
}

static void negotiate_off(void)
{
	uint8_t sink[16];

	negotiate(0x00u);
	(void)hal_drain(sink, sizeof sink);
}

/* spi_slave_cs_high() reports whether a FRESH reply was staged -- what the
 * HAL turns into the ATTN rising edge.  Decoded requests, error envelopes
 * and the tar-pit breaker are fresh; the drain / empty rewinds are not. */
ZTEST(gd32_bridge_transport, test_cs_high_reports_a_fresh_stage)
{
	uint8_t       buf[80];
	const uint8_t zeros[4] = { 0 };

	transport_spi_init();

	spi_slave_cs_low();
	for (size_t i = 0; i < sizeof ping_frame; i++)
		spi_slave_rx_byte(ping_frame[i]);
	zassert_true(spi_slave_cs_high(), "a decoded request stages a fresh reply");
	(void)hal_drain(buf, sizeof buf);

	spi_slave_cs_low();
	for (size_t i = 0; i < sizeof zeros; i++)
		spi_slave_rx_byte(zeros[i]);
	zassert_false(spi_slave_cs_high(), "an all-0x00 reply-drain is a rewind, not a stage");
	(void)hal_drain(buf, sizeof buf);

	spi_slave_cs_low();
	zassert_false(spi_slave_cs_high(), "an empty transaction is a rewind, not a stage");
	(void)hal_drain(buf, sizeof buf);

	/* Corrupted request (non-SOF, not all-zero): a fresh STATUS_IO envelope. */
	const uint8_t junk[4] = { 0x12u, 0x34u, 0x56u, 0x78u };
	spi_slave_cs_low();
	for (size_t i = 0; i < sizeof junk; i++)
		spi_slave_rx_byte(junk[i]);
	zassert_true(spi_slave_cs_high(), "a mangled request stages the IO envelope");
	(void)hal_drain(buf, sizeof buf);

	/* Bad CRC: also fresh. */
	const uint8_t badcrc[4] = { 0xA5u, 0x00u, 0x00u, 0x00u };
	spi_slave_cs_low();
	for (size_t i = 0; i < sizeof badcrc; i++)
		spi_slave_rx_byte(badcrc[i]);
	zassert_true(spi_slave_cs_high(), "a CRC failure stages the IO envelope");
	(void)hal_drain(buf, sizeof buf);

	/* Too short to be an envelope. */
	const uint8_t shorty[2] = { 0xA5u, 0x00u };
	spi_slave_cs_low();
	for (size_t i = 0; i < sizeof shorty; i++)
		spi_slave_rx_byte(shorty[i]);
	zassert_true(spi_slave_cs_high(), "a runt frame stages the IO envelope");
	(void)hal_drain(buf, sizeof buf);
}

/* The tar-pit breaker (13th consecutive rewind) swaps in the IO envelope: fresh. */
ZTEST(gd32_bridge_transport, test_cs_high_tar_pit_breaker_is_a_fresh_stage)
{
	uint8_t       buf[80];
	const uint8_t zeros[4]   = { 0 };
	bool          fresh_seen = false;

	transport_spi_init();
	transaction(ping_frame, sizeof ping_frame);
	(void)hal_drain(buf, sizeof buf);
	for (int i = 0; i < 20; i++) {
		spi_slave_cs_low();
		for (size_t j = 0; j < sizeof zeros; j++)
			spi_slave_rx_byte(zeros[j]);
		const bool fresh = spi_slave_cs_high();
		(void)hal_drain(buf, sizeof buf);
		if (i < 12) zassert_false(fresh, "rewind %d is not fresh", i);
		if (fresh) fresh_seen = true;
	}
	zassert_true(fresh_seen, "past the rewind bound the breaker stages a fresh IO envelope");
}

/* Only BATCH may exceed 65 bytes: an unknown opcode with exactly 65 reaches the
 * dispatcher (NOSUPPORT); 66 is refused by the transport (INVAL), with or
 * without BIG_FRAME. */
ZTEST(gd32_bridge_transport, test_payload_over_65_is_refused_for_every_non_batch_opcode)
{
	uint8_t payload[260];
	uint8_t frame[260 + 4];
	uint8_t buf[80];

	memset(payload, 0x5A, sizeof payload);
	transport_spi_init();
	negotiate_off();

	for (int big = 0; big < 2; big++) {
		if (big) negotiate_ext(GD32_BRIDGE_LINK_FEAT_BIG_FRAME, 252u);

		size_t len = build_frame(frame, 0x99u, payload, 65u);
		transaction(frame, len);
		size_t n = hal_drain(buf, sizeof buf);
		zassert_equal(n, 4u);
		zassert_equal(
		    buf[1], 0x06u, "65 B reaches dispatch: unknown opcode -> NOSUPPORT (big=%d)", big);

		len = build_frame(frame, 0x99u, payload, 66u);
		transaction(frame, len);
		n = hal_drain(buf, sizeof buf);
		zassert_equal(n, 4u);
		zassert_equal(buf[1], 0x01u, "66 B -> INVAL by the transport (big=%d)", big);

		/* OTA keeps its limit even on a BIG link. */
		len = build_frame(frame, 0xF1u, payload, 66u);
		transaction(frame, len);
		n = hal_drain(buf, sizeof buf);
		zassert_equal(buf[1], 0x01u, "OTA WRITE_CHUNK 66 B -> INVAL (big=%d)", big);

		/* ADC_DSP_STAGE_PUSH too. */
		len = build_frame(frame, 0x38u, payload, 100u);
		transaction(frame, len);
		n = hal_drain(buf, sizeof buf);
		zassert_equal(buf[1], 0x01u, "DSP stage push over 65 -> INVAL (big=%d)", big);
	}
	negotiate_off();
}

/* The BATCH request ceiling is the negotiated max_payload: 65 until BIG_FRAME. */
ZTEST(gd32_bridge_transport, test_batch_request_ceiling_is_the_negotiated_max_payload)
{
	uint8_t payload[260];
	uint8_t frame[260 + 4];
	uint8_t buf[80];

	memset(payload, 0, sizeof payload);
	transport_spi_init();

	/* BATCH granted, BIG_FRAME not: 66 bytes is over max_payload 65 -> INVAL
	 * (not the NOSUPPORT / validation answer a <= 65 request would get). */
	negotiate_ext(GD32_BRIDGE_LINK_FEAT_BATCH, 65u);
	size_t len = build_frame(frame, 0x04u, payload, 66u);
	transaction(frame, len);
	size_t n = hal_drain(buf, sizeof buf);
	zassert_equal(n, 4u);
	zassert_equal(buf[1], 0x01u, "66 > max_payload 65");

	/* A 252-byte BATCH payload fits a BIG link's ceiling: it reaches the
	 * dispatcher (garbage count -> validation INVAL, but through dispatch). */
	negotiate_ext(GD32_BRIDGE_LINK_FEAT_BATCH | GD32_BRIDGE_LINK_FEAT_BIG_FRAME, 252u);
	payload[0] = 17u; /* count > 16 -> OUT_OF_RANGE, which only dispatch can answer */
	len        = build_frame(frame, 0x04u, payload, 252u);
	zassert_equal(len, 256u, "the largest SPI frame");
	transaction(frame, len);
	n = hal_drain(buf, sizeof buf);
	zassert_equal(n, 4u);
	zassert_equal(buf[1], 0x08u, "252 B accepted by the transport; dispatch says OUT_OF_RANGE");

	/* A negotiated max_payload below 252 is honoured. */
	negotiate_ext(GD32_BRIDGE_LINK_FEAT_BATCH | GD32_BRIDGE_LINK_FEAT_BIG_FRAME, 100u);
	len = build_frame(frame, 0x04u, payload, 101u);
	transaction(frame, len);
	n = hal_drain(buf, sizeof buf);
	zassert_equal(buf[1], 0x01u, "101 > negotiated 100");
	len = build_frame(frame, 0x04u, payload, 100u);
	transaction(frame, len);
	n = hal_drain(buf, sizeof buf);
	zassert_equal(buf[1], 0x08u, "100 <= negotiated 100 reaches dispatch");
	negotiate_off();
}

/* The RX/TX staging holds a full 256-byte frame: a 253-byte payload is one byte
 * too many, the trailing CRC byte is dropped, and the CRC check fails loud. */
ZTEST(gd32_bridge_transport, test_frame_one_byte_over_256_fails_the_crc)
{
	uint8_t payload[260];
	uint8_t frame[260 + 4];
	uint8_t buf[80];

	memset(payload, 0, sizeof payload);
	transport_spi_init();
	negotiate_ext(GD32_BRIDGE_LINK_FEAT_BATCH | GD32_BRIDGE_LINK_FEAT_BIG_FRAME, 252u);

	payload[0] = 17u;
	size_t len = build_frame(frame, 0x04u, payload, 253u);
	zassert_equal(len, 257u);
	transaction(frame, len);
	size_t n = hal_drain(buf, sizeof buf);
	zassert_equal(n, 4u);
	zassert_equal(buf[1], 0x05u, "STATUS_IO: the 257th byte was dropped");
	negotiate_off();
}

/* A real BATCH larger than any legacy frame executes through the 256-byte path:
 * 16 TMU_COMPUTE entries (12-byte args) = a 225-byte payload.  On the stub the
 * first TMU op answers NOSUPPORT, so execution stops there. */
ZTEST(gd32_bridge_transport, test_large_batch_is_decoded_and_stops_at_the_first_error)
{
	uint8_t payload[260];
	uint8_t frame[260 + 4];
	uint8_t buf[80];
	size_t  off = 0;

	memset(payload, 0, sizeof payload);
	transport_spi_init();
	negotiate_ext(GD32_BRIDGE_LINK_FEAT_BATCH | GD32_BRIDGE_LINK_FEAT_BIG_FRAME, 252u);

	payload[off++] = 16u;
	for (int i = 0; i < 16; i++) {
		payload[off++] = 0x90u; /* TMU_COMPUTE */
		payload[off++] = 12u;
		off += 12u;
	}
	zassert_equal(off, 225u);
	size_t len = build_frame(frame, 0x04u, payload, off);
	zassert_equal(len, 229u, "well past the legacy 69-byte envelope");
	transaction(frame, len);
	size_t n = hal_drain(buf, sizeof buf);
	zassert_equal(n, 4u + 1u + 2u, "SOF STATUS executed {status,len} CRC");
	zassert_equal(buf[1], 0x00u, "outer OK");
	zassert_equal(buf[2], 1u, "one op executed");
	zassert_equal(buf[3], 0x06u, "TMU_COMPUTE on the stub: NOSUPPORT");
	zassert_equal(buf[4], 0u, "non-OK sub-status has len 0");
	negotiate_off();
}

/* The static reply scratch is shared: a long reply followed by a short one must
 * not leak the earlier bytes into the later reply. */
ZTEST(gd32_bridge_transport, test_reply_scratch_does_not_leak_between_requests)
{
	uint8_t frame[16];
	uint8_t buf[80];
	uint8_t req[1] = { 0x00u };

	transport_spi_init();
	negotiate_off();
	transaction(frame, build_frame(frame, 0x02u, NULL, 0u)); /* GET_BUILD_ID: 20-byte reply */
	size_t n = hal_drain(buf, sizeof buf);
	zassert_equal(n, 24u);
	(void)req;
	transaction(ping_frame, sizeof ping_frame);
	n = hal_drain(buf, sizeof buf);
	zassert_equal(n, 4u, "a PING reply is exactly 4 bytes, whatever the scratch held");
	zassert_mem_equal(buf, ping_frame, sizeof ping_frame);
}

ZTEST_SUITE(gd32_bridge_transport, NULL, NULL, NULL, NULL, NULL);
