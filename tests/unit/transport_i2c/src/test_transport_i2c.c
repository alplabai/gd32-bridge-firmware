/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the gd32-bridge I2C-slave transport seams
 * (gd32-bridge-firmware:src/transport_i2c.c) -- the register-framed
 * staging layer between the byte-level HAL and protocol_dispatch().
 *
 * This suite links the REAL transport and dispatcher, not mocks:
 * src/transport_i2c.c, src/protocol.c, src/bootloader/bootloader.c,
 * src/ota.c, src/crc32.c and hal/bridge_hw_stub.c (see
 * tests/unit/CMakeLists.txt's test_transport_i2c target) -- the exact
 * same code paths the device runs, minus the GigaDevice I2C0 ISR
 * wiring in hal/transport_hw_gd32.c, which is out of scope for a
 * hardware-less host build.
 *
 * Per the project rule "one framing format, one command set, one set
 * of reply codes; only the transport layer differs" (src/protocol.c),
 * every twinned case below exercises the SAME protocol_dispatch() the
 * SPI suite (tests/unit/transport_spi/src/test_transport_spi.c) pins
 * -- only the wire framing around it changes.
 *
 * NOT-YET-WRITTEN cases (deliberate omissions, not silent gaps):
 *
 *   1. i2c_slave_write_end() driven a SECOND time off the same
 *      write_start()/rx_byte() sequence, mirroring the real HAL's
 *      call sites (hal/transport_hw_gd32.c fires write_end() from
 *      BOTH the ADDSEND-read branch and the STOP branch of one
 *      transaction).  Tracked as issue #4: write_end() has no
 *      once-only guard and never consumes/clears i2c_rx_buf, so the
 *      second call re-validates the same buffered bytes and
 *      re-dispatches protocol_dispatch() a second (or third) time for
 *      a command the host sent exactly once.  A case asserting
 *      CORRECT behaviour here would be red today; a case asserting
 *      the double-dispatch would cement the bug.  Once #4 is fixed,
 *      this case would assert that a second write_end() call with no
 *      intervening i2c_slave_write_start() returns false (or
 *      otherwise does not re-invoke protocol_dispatch()) instead of
 *      re-staging a fresh reply for the same command.
 *   2. The "stale re-dispatch" read-side twin of #1: an I2C read
 *      phase with NO new write since the last staged reply was
 *      consumed re-runs write_end() over the stale i2c_rx_buf/rx_len
 *      and returns a CRC-valid reply for a command the host never
 *      re-sent.  Same root cause as #4, same reason it is not written
 *      here.  Once fixed, this case would assert that a repeated
 *      read-phase after a fully-drained reply does NOT trigger a
 *      second protocol_dispatch() call (observable once the fix adds
 *      a dispatch-counting or one-shot-consumed seam) and instead
 *      returns the idle 0xFF pattern (see
 *      test_read_past_reply_returns_idle_0xff below, which pins the
 *      boundary of what IS safe to assert today: repeated
 *      i2c_slave_tx_next_byte() calls alone, without a second
 *      write_end(), are idle-safe).
 *   3. A real STATUS_OK maximum-length (GD32_BRIDGE_MAX_PAYLOAD_BYTES)
 *      REPLY is not exercised end-to-end.  Not an impossibility -- it is
 *      a SCOPING decision.  This suite deliberately links the STUB HAL
 *      (hal/bridge_hw_stub.c, see tests/unit/CMakeLists.txt's
 *      test_transport_i2c target), which answers every HW-backed
 *      handler with an unconditional BRIDGE_HW_ERR_NOTIMPL, so the only
 *      command whose reply could reach GD32_BRIDGE_MAX_PAYLOAD_BYTES
 *      (CMD_ADC_STREAM_READ at max_samples == GD32_BRIDGE_ADC_STREAM_READ_MAX)
 *      short-circuits to STATUS_NOSUPPORT before ever touching the reply
 *      buffer here.  tests/unit/fake/bridge_hw_fake.c -- a test-only fake
 *      with a real queued-sample backing store
 *      (bridge_hw_fake_adc_stream_queue_push() /
 *      bridge_hw_adc_stream_read()) -- CAN produce that data and already
 *      exists in this tree, but swapping this suite's link target from
 *      the stub to the fake is a build-target change (tests/unit/CMakeLists.txt)
 *      outside this file, which this suite does not own.  This suite
 *      instead pins the maximum-length REQUEST boundary
 *      (test_max_length_request_dispatches), which IS reachable without
 *      live hardware or a different link target.  The real STATUS_OK
 *      max-length REPLY case belongs in a fake-backed suite (see
 *      tests/unit/protocol/src/test_protocol.c's test_protocol target,
 *      which already links fake/bridge_hw_fake.c) -- not here.
 *   4. This suite deliberately CONTRADICTS the MSB-first CRC byte order
 *      committed in tests/protocol_vectors.txt:43 (`i2c_ping_write =
 *      0000E1F0`) and :45 (`i2c_ping_read_ok = 00E1F0`).  Every
 *      assertion below pins the CRC LSB-first, matching what
 *      src/transport_i2c.c:62-63,115-116 actually ships (and what the
 *      interoperating alp-sdk host driver parses) -- see issue #68,
 *      which is open precisely to decide which side (the shipping
 *      firmware/host pairing, or the vector-file generator + the
 *      alp-sdk spec sentence it followed) is normative, and holds that
 *      the firmware must NOT move.  Resolving #68 changes the
 *      generator and the committed vectors, not this suite's
 *      assertions; the follow-on step named in #68 is a
 *      tests/unit/protocol_vectors/ suite that replays
 *      tests/protocol_vectors.txt's i2c_* vectors through these same
 *      seams -- once that lands, this file's hand-built frames
 *      (build_write() and the hardcoded literals below) should be
 *      retired in favour of consuming that vector file directly, so
 *      the wire byte order can never again drift between the generator
 *      and the firmware unnoticed.
 */

#include <string.h>
#include <zephyr/ztest.h>

#include "protocol.h"
#include "transport.h"

/* Mirrors transport_i2c.c's private I2C_MAX_WRITE_BYTES (not exported):
 * reg(1) + CMD(1) + max payload + CRC(2).  Recomputed from the public
 * constants rather than hardcoded so the boundary test below tracks the
 * real capacity, not a number this file made up. */
#define I2C_TEST_MAX_WRITE_BYTES (1u + 1u + GD32_BRIDGE_MAX_PAYLOAD_BYTES + 2u)

/* Builds one WRITE-phase byte sequence exactly as the host driver does:
 * reg | CMD | PAYLOAD | CRC(CMD..PAYLOAD) -- the CRC deliberately excludes
 * the leading reg byte (transport_i2c.c: crc16_ccitt_false(&i2c_rx_buf[1],
 * ...)).  Returns the total envelope length written into `out`. */
static size_t build_write(uint8_t *out, uint8_t cmd, const uint8_t *payload, size_t payload_len)
{
	out[0] = GD32_BRIDGE_I2C_REG_CMD;
	out[1] = cmd;
	if (payload_len > 0u && payload != NULL) {
		memcpy(&out[2], payload, payload_len);
	}
	const uint16_t crc    = crc16_ccitt_false(&out[1], 1u + payload_len);
	out[2u + payload_len] = (uint8_t)(crc & 0xFFu);
	out[3u + payload_len] = (uint8_t)(crc >> 8);
	return 4u + payload_len;
}

/* Replays one WRITE phase through the seams, as the HAL's I2C0 ISR does on
 * an addressed write: reset RX staging, feed the captured bytes, then run
 * the single write_end() the STOP/ADDSEND-read edge fires.  Returns what
 * write_end() returned (true = a fresh request was validated and staged). */
static bool write_phase(const uint8_t *bytes, size_t len)
{
	i2c_slave_write_start();
	for (size_t i = 0; i < len; i++) {
		i2c_slave_rx_byte(bytes[i]);
	}
	return i2c_slave_write_end();
}

/* Clocks out exactly `n` reply bytes the way the HAL's read-phase ISR does:
 * one i2c_slave_tx_next_byte() call per SCL byte the host clocks. */
static void read_phase(uint8_t *out, size_t n)
{
	for (size_t i = 0; i < n; i++) {
		out[i] = i2c_slave_tx_next_byte();
	}
}

/* --------------------------------------------------------------- */
/* Independent CRC oracle -- every OTHER assertion in this file computes  */
/* its expected CRC by calling the SAME crc16_ccitt_false() the code      */
/* under test calls, so a wrong init/poly/xor-out in that one function    */
/* is invisible to them (see the mutation report's CRC-algorithm gap).    */
/* The two constants below are literal bytes, not recomputed here, so     */
/* they catch what the recomputed-CRC assertions structurally cannot.     */
/* --------------------------------------------------------------- */

/* PING write frame and its byte-identical reply, hardcoded byte-for-byte
 * rather than built from crc16_ccitt_false() at test time.  CRC over the
 * single CMD=0x00 byte (and, for the reply, the single STATUS=0x00 byte)
 * is 0xE1F0 -- independently checkable against the universally-cited
 * CRC-16/CCITT-FALSE self-test vector (crc16_ccitt_false_ref_string /
 * crc16_ccitt_false_ref_result = 29B1 in ../../../protocol_vectors.txt;
 * see test_crc16_matches_canonical_self_test_vector below) by running
 * the same poly/init/xor-out by hand over one 0x00 byte.  Byte order is
 * LSB first -- the firmware's SHIPPING order, which this suite pins on
 * purpose against tests/protocol_vectors.txt's MSB-first vectors; see
 * the file-header declared omission #4 (issue #68). */
static const uint8_t ping_write_frame[] = { GD32_BRIDGE_I2C_REG_CMD, CMD_PING, 0xF0u, 0xE1u };
static const uint8_t ping_reply_frame[] = { STATUS_OK, 0xF0u, 0xE1u };

/* --------------------------------------------------------------- */
/* Twins of the SPI suite's cases (same protocol_dispatch(), same     */
/* commands -- only the wire framing differs).                        */
/* --------------------------------------------------------------- */

ZTEST(gd32_bridge_transport_i2c, test_ping_stages_reply)
{
	uint8_t req[4];
	uint8_t reply[8];

	transport_i2c_init();
	size_t req_len = build_write(req, CMD_PING, NULL, 0u);

	/* Assert against the HARDCODED literal, not a recomputed CRC --
	 * closes the CRC-algorithm-mutation gap (see the block comment
	 * above ping_write_frame[]). */
	zassert_equal(req_len, sizeof ping_write_frame, "PING write frame is reg+CMD+CRC(2) = 4 bytes");
	zassert_mem_equal(
	    req,
	    ping_write_frame,
	    sizeof ping_write_frame,
	    "build_write() output matches the hardcoded literal (independent CRC oracle)");
	zassert_true(write_phase(req, req_len), "PING write stages a fresh reply");

	/* PING reply: STATUS | CRC(STATUS) -- 0-byte payload, 3 bytes total
	 * (no SOF on I2C; see transport_i2c.c's stage_reply()). */
	read_phase(reply, sizeof ping_reply_frame);

	zassert_mem_equal(
	    reply,
	    ping_reply_frame,
	    sizeof ping_reply_frame,
	    "PING reply is byte-identical to the hardcoded literal (independent CRC oracle)");
}

/* Closes the CRC-algorithm-mutation gap directly: crc16_ccitt_false()
 * over the universally-cited "123456789" self-test input must equal
 * 0x29B1 (crc16_ccitt_false_ref_string / crc16_ccitt_false_ref_result in
 * ../../../protocol_vectors.txt).  This is a LITERAL expected value, not
 * one recomputed through the function under test, so a wrong init value,
 * wrong polynomial, or a missing/extra final XOR (all of which every
 * other CRC assertion in this file is blind to, since they call the
 * same production crc16_ccitt_false() to build their own expectation)
 * fails HERE. */
ZTEST(gd32_bridge_transport_i2c, test_crc16_matches_canonical_self_test_vector)
{
	static const uint8_t ref_string[] = { '1', '2', '3', '4', '5', '6', '7', '8', '9' };

	zassert_equal(crc16_ccitt_false(ref_string, sizeof ref_string),
	              0x29B1u,
	              "CRC-16/CCITT-FALSE(\"123456789\") must be the canonical 0x29B1");
}

/* A fresh write+read cycle for a DIFFERENT command must replace the
 * staged reply, not append to or resurrect the previous command's. */
ZTEST(gd32_bridge_transport_i2c, test_new_request_replaces_staged_reply)
{
	uint8_t req[4];
	uint8_t reply[8];

	transport_i2c_init();

	size_t req_len = build_write(req, CMD_PING, NULL, 0u);
	zassert_true(write_phase(req, req_len), "PING write stages a fresh reply");
	read_phase(reply, 3u);
	zassert_equal(reply[0], STATUS_OK, "PING reply staged");

	req_len = build_write(req, CMD_GET_VERSION, NULL, 0u);
	zassert_true(write_phase(req, req_len), "GET_VERSION write stages a fresh reply");
	read_phase(reply, 6u); /* STATUS + MAJOR MINOR PATCH + CRC(2) = 6 B */

	zassert_equal(reply[0], STATUS_OK, "GET_VERSION replies STATUS_OK");
	zassert_equal(reply[1], PROTOCOL_VERSION_MAJOR, "MAJOR");
	zassert_equal(reply[2], PROTOCOL_VERSION_MINOR, "MINOR");
	zassert_equal(reply[3], PROTOCOL_VERSION_PATCH, "PATCH");

	const uint16_t crc = crc16_ccitt_false(reply, 4u);

	zassert_equal(reply[4], (uint8_t)(crc & 0xFFu), "CRC lo -- the NEW reply, not PING's");
	zassert_equal(reply[5], (uint8_t)(crc >> 8), "CRC hi");
}

/* SPI's test_mangled_request_stages_io_error twin: a corrupted request
 * must stage a loud error reply -- not preserve the previous reply, not
 * stay silent.  The staged STATUS CODE differs by design: I2C has no
 * SOF to distinguish "corrupted" from "nothing sent" the way SPI's
 * all-0x00 dummy-drain detection does, so every CRC/framing failure
 * here stages the I2C-only STATUS_NO_PENDING (0x80) sentinel rather
 * than SPI's STATUS_IO. */
ZTEST(gd32_bridge_transport_i2c, test_mangled_request_stages_no_pending)
{
	uint8_t req[4];
	uint8_t reply[8];

	transport_i2c_init();

	size_t req_len = build_write(req, CMD_PING, NULL, 0u);
	req[req_len - 1] ^= 0xFFu; /* corrupt the CRC hi byte */

	zassert_false(write_phase(req, req_len), "corrupted CRC must not dispatch");
	read_phase(reply, 3u);

	const uint16_t crc = crc16_ccitt_false(reply, 1u);

	zassert_equal(reply[0], STATUS_NO_PENDING, "corrupted request stages NO_PENDING, not IO");
	zassert_equal(reply[1], (uint8_t)(crc & 0xFFu), "CRC lo");
	zassert_equal(reply[2], (uint8_t)(crc >> 8), "CRC hi");
}

/* --------------------------------------------------------------- */
/* I2C-specific framing rules -- no SPI counterpart.                 */
/* --------------------------------------------------------------- */

/* A read that lands before any matching write since the last START must
 * stage the I2C-only STATUS_NO_PENDING (0x80) sentinel -- see
 * ../docs/gd32-bridge-protocol.md §5 and transport_i2c.c's
 * i2c_slave_tx_next_byte(). */
ZTEST(gd32_bridge_transport_i2c, test_read_before_write_stages_no_pending)
{
	uint8_t reply[8];

	transport_i2c_init();

	/* No write_phase() at all. */
	read_phase(reply, 3u);

	const uint16_t crc = crc16_ccitt_false(reply, 1u);

	zassert_equal(reply[0], STATUS_NO_PENDING, "read-before-write stages NO_PENDING");
	zassert_equal(reply[1], (uint8_t)(crc & 0xFFu), "CRC lo");
	zassert_equal(reply[2], (uint8_t)(crc >> 8), "CRC hi");
}

/* The write-phase CRC covers CMD..PAYLOAD and deliberately EXCLUDES the
 * leading reg byte (transport_i2c.c: crc16_ccitt_false(&i2c_rx_buf[1],
 * 1u + payload_len)) -- unlike SPI, whose CRC covers the whole envelope
 * including SOF.  Pin it by constructing a frame whose CRC covers reg+CMD
 * (the SPI-style span): it must be REJECTED, because the firmware's own
 * CRC only ever covers CMD..PAYLOAD and the two spans hash differently
 * for this content. */
ZTEST(gd32_bridge_transport_i2c, test_crc_excludes_leading_reg_byte)
{
	uint8_t req[4];
	uint8_t reply[8];

	transport_i2c_init();

	req[0]                   = GD32_BRIDGE_I2C_REG_CMD;
	req[1]                   = CMD_PING;
	const uint16_t wrong_crc = crc16_ccitt_false(req, 2u); /* covers reg+CMD -- wrong span */

	req[2] = (uint8_t)(wrong_crc & 0xFFu);
	req[3] = (uint8_t)(wrong_crc >> 8);

	zassert_false(write_phase(req, sizeof req), "reg-inclusive CRC must not match");
	read_phase(reply, 3u);
	zassert_equal(reply[0], STATUS_NO_PENDING, "reg-inclusive CRC rejected");

	/* The SAME command with the documented CMD..PAYLOAD-only CRC
	 * dispatches cleanly -- pinning the exact covered span. */
	size_t req_len = build_write(req, CMD_PING, NULL, 0u);

	zassert_true(write_phase(req, req_len), "CMD-only CRC dispatches");
	read_phase(reply, 3u);
	zassert_equal(reply[0], STATUS_OK, "PING replies STATUS_OK");
}

/* The complementary invariant to SPI's v0.7 STATUS_SEQ stamp
 * (test_status_seq_stamp_contract in the SPI suite): I2C replies must
 * NEVER carry the 4-bit stamp, even after negotiating STATUS_SEQ over
 * THIS transport.  Bit 7 of the I2C status byte is already owned by the
 * I2C-only STATUS_NO_PENDING sentinel, so transport_i2c.c has no
 * stamping logic at all -- protocol_link_features() is consulted only
 * by transport_spi.c's stage_reply() (see protocol.h's v0.7 note). */
ZTEST(gd32_bridge_transport_i2c, test_status_seq_never_stamps_i2c_reply)
{
	uint8_t req[8];
	uint8_t reply[8];
	uint8_t feat = GD32_BRIDGE_LINK_FEAT_STATUS_SEQ;

	transport_i2c_init();

	size_t req_len = build_write(req, CMD_LINK_FEATURES, &feat, 1u);

	zassert_true(write_phase(req, req_len), "LINK_FEATURES write dispatches");
	read_phase(reply, 4u); /* STATUS + granted(1) + CRC(2) */
	zassert_equal(reply[0], STATUS_OK, "LINK_FEATURES granted, unstamped");
	zassert_equal(reply[1], GD32_BRIDGE_LINK_FEAT_STATUS_SEQ, "STATUS_SEQ granted");

	/* Several fresh decodes afterward: every I2C status byte stays
	 * EXACTLY the bare status code, never OR'd with a sequence stamp. */
	for (int i = 0; i < 3; i++) {
		req_len = build_write(req, CMD_PING, NULL, 0u);
		zassert_true(write_phase(req, req_len), "PING write dispatches (%d)", i);
		read_phase(reply, 3u);
		zassert_equal(reply[0], STATUS_OK, "PING status byte carries no STATUS_SEQ stamp (%d)", i);
	}

	/* Disable again -- tidy, and matches the un-negotiated default. */
	feat    = 0u;
	req_len = build_write(req, CMD_LINK_FEATURES, &feat, 1u);
	zassert_true(write_phase(req, req_len), "LINK_FEATURES disable dispatches");
	read_phase(reply, 4u);
	zassert_equal(reply[1], 0x00u, "nothing granted");
}

/* The request-payload pointer i2c_slave_write_end() hands to
 * protocol_dispatch() (transport_i2c.c: `payload_len > 0u ?
 * &i2c_rx_buf[2] : NULL`) must point at the ACTUAL payload bytes, not an
 * off-by-one location such as the CRC low byte at i2c_rx_buf[3].
 * CMD_LINK_FEATURES is the only opcode this suite can use to pin this --
 * it is the sole handler that echoes request-payload CONTENT (masked by
 * GD32_BRIDGE_LINK_FEAT_STATUS_SEQ) straight into the reply, so an
 * off-by-one read is externally observable.
 *
 * Payload 0x10 is deliberately chosen over the more obvious 0/1 used by
 * test_status_seq_never_stamps_i2c_reply above: with those two values,
 * the LOW byte of crc16_ccitt_false(CMD_LINK_FEATURES, payload) happens
 * to share bit 0 with the payload itself (0x00 -> CRC lo 0xA6, bit0 0;
 * 0x01 -> CRC lo 0x87, bit0 1), so a pointer shifted one byte late reads
 * the CRC low byte instead of the payload and gets the SAME masked
 * answer by coincidence -- the mutation survives silently.  With 0x10
 * (bit 0 clear), crc16_ccitt_false(CMD_LINK_FEATURES, 0x10)'s low byte
 * is 0x97 (bit 0 SET), so a shifted read observably grants STATUS_SEQ
 * when the correct pointer must not. */
ZTEST(gd32_bridge_transport_i2c, test_request_payload_pointer_is_pinned)
{
	uint8_t req[8];
	uint8_t reply[8];
	uint8_t feat = 0x10u; /* bit 0 clear -- STATUS_SEQ must NOT be granted */

	transport_i2c_init();

	size_t req_len = build_write(req, CMD_LINK_FEATURES, &feat, 1u);

	zassert_true(write_phase(req, req_len), "LINK_FEATURES write dispatches");
	read_phase(reply, 4u); /* STATUS + granted(1) + CRC(2) */
	zassert_equal(reply[0], STATUS_OK, "LINK_FEATURES accepted");
	zassert_equal(reply[1],
	              0x00u,
	              "payload 0x10 (bit0 clear) must NOT grant STATUS_SEQ -- an off-by-one payload "
	              "pointer reading the CRC low byte (0x97, bit0 set) would");
}

/* Bytes read PAST the staged reply's end must return the 0xFF idle
 * pattern -- not garbage, not a wrapped repeat of the reply, and (see
 * the file header's omission #2) NOT a re-dispatch of the buffered
 * request.  This is exactly the boundary that IS safe to assert today:
 * repeated i2c_slave_tx_next_byte() calls alone, without a second
 * i2c_slave_write_end(), never touch protocol_dispatch() again. */
ZTEST(gd32_bridge_transport_i2c, test_read_past_reply_returns_idle_0xff)
{
	uint8_t req[4];
	uint8_t reply[8];

	transport_i2c_init();

	size_t req_len = build_write(req, CMD_PING, NULL, 0u);
	zassert_true(write_phase(req, req_len), "PING write stages a fresh reply");

	read_phase(reply, 3u);
	zassert_equal(reply[0], STATUS_OK, "PING reply staged");

	for (int i = 0; i < 5; i++) {
		uint8_t extra = i2c_slave_tx_next_byte();

		zassert_equal(extra, 0xFFu, "over-read %d returns idle 0xFF", i);
	}
}

/* Boundary: a maximum-length request whose payload is EXACTLY
 * GD32_BRIDGE_MAX_PAYLOAD_BYTES must be captured and handed to
 * protocol_dispatch() intact.  CMD_PING rejects any non-empty payload
 * (STATUS_INVAL) -- which is exactly the signal this case needs:
 * STATUS_INVAL can only come out of protocol_dispatch(), so seeing it
 * (rather than the transport's own NO_PENDING framing-error sentinel)
 * proves the full envelope was captured, CRC-validated and passed
 * through with its real length. */
ZTEST(gd32_bridge_transport_i2c, test_max_length_request_dispatches)
{
	uint8_t req[4 + GD32_BRIDGE_MAX_PAYLOAD_BYTES];
	uint8_t payload[GD32_BRIDGE_MAX_PAYLOAD_BYTES];
	uint8_t reply[8];

	memset(payload, 0x5Au, sizeof payload);

	transport_i2c_init();

	size_t req_len = build_write(req, CMD_PING, payload, sizeof payload);

	zassert_equal(req_len, sizeof req, "max envelope is reg+CMD+MAX_PAYLOAD+CRC");
	zassert_true(write_phase(req, req_len), "max-length write dispatches");
	read_phase(reply, 3u);
	zassert_equal(
	    reply[0], STATUS_INVAL, "PING rejects the oversized payload -- reached the dispatcher");
}

/* Boundary: a write phase clocked LONGER than the RX staging buffer's
 * capacity (I2C_MAX_WRITE_BYTES) must not overflow and must not
 * false-accept.  This pins only that an over-capacity write is
 * REJECTED -- it does NOT by itself pin how i2c_slave_rx_byte() handles
 * the trailing bytes (drop vs. e.g. overwrite the last accepted slot):
 * either way the buffered frame's CRC ends up mismatched against
 * garbage-filled bytes, so both behaviours reject here identically.
 * See test_bytes_past_capacity_are_dropped_not_overwritten below for the
 * case that DOES disambiguate drop from overwrite. */
ZTEST(gd32_bridge_transport_i2c, test_oversized_write_truncates_and_rejects)
{
	uint8_t big[I2C_TEST_MAX_WRITE_BYTES + 16u];
	uint8_t reply[8];

	big[0] = GD32_BRIDGE_I2C_REG_CMD;
	big[1] = CMD_PING;
	for (size_t i = 2; i < sizeof big; i++) {
		big[i] = (uint8_t)i;
	}

	transport_i2c_init();

	zassert_false(write_phase(big, sizeof big), "over-capacity write must not dispatch");
	read_phase(reply, 3u);
	zassert_equal(reply[0], STATUS_NO_PENDING, "truncated/CRC-mismatched write stages NO_PENDING");
}

/* Disambiguates DROP from OVERWRITE for bytes clocked past
 * i2c_rx_buf's capacity (test_oversized_write_truncates_and_rejects
 * above cannot: both behaviours reject an all-garbage over-long write
 * identically).  Send a VALID, exactly capacity-sized max-payload PING
 * frame -- byte-identical to test_max_length_request_dispatches' frame
 * -- immediately followed by extra junk bytes.  i2c_slave_rx_byte()'s
 * own comment says the junk is silently DROPPED ("the CRC validation at
 * end-of-write will fail because the trailing bytes never landed" --
 * i.e. never landed AT ALL, not landed-and-overwrote-something): under
 * that contract the accepted frame is untouched by the junk and still
 * dispatches (STATUS_INVAL, same as the no-junk case).  Under an
 * OVERWRITE-the-last-slot mutation, the junk corrupts the frame's
 * trailing CRC byte and the write is rejected (STATUS_NO_PENDING)
 * instead -- the two behaviours diverge here. */
ZTEST(gd32_bridge_transport_i2c, test_bytes_past_capacity_are_dropped_not_overwritten)
{
	uint8_t payload[GD32_BRIDGE_MAX_PAYLOAD_BYTES];
	uint8_t req[I2C_TEST_MAX_WRITE_BYTES + 8u];
	uint8_t reply[8];

	memset(payload, 0x5Au, sizeof payload);

	transport_i2c_init();

	size_t req_len = build_write(req, CMD_PING, payload, sizeof payload);

	zassert_equal(req_len, I2C_TEST_MAX_WRITE_BYTES, "sanity: frame exactly fills the RX buffer");

	/* Extra junk clocked in past capacity -- must be dropped, not folded
	 * into the last accepted byte. */
	for (size_t i = req_len; i < sizeof req; i++) {
		req[i] = (uint8_t)(0xC0u + i);
	}

	zassert_true(
	    write_phase(req, sizeof req),
	    "a capacity-sized frame plus trailing junk must still dispatch -- the junk must be "
	    "DROPPED, not overwrite the frame's trailing CRC byte");
	read_phase(reply, 3u);
	zassert_equal(
	    reply[0],
	    STATUS_INVAL,
	    "PING rejects the oversized payload -- the frame reached the dispatcher unmodified "
	    "by the trailing junk");
}

/* --------------------------------------------------------------- */
/* i2c_slave_write_end()'s framing guard (transport_i2c.c:110):        */
/*   if (i2c_rx_len < 4u || i2c_rx_buf[0] != GD32_BRIDGE_I2C_REG_CMD)   */
/* Every case above reaches this guard only via build_write(), which    */
/* always emits a >=4-byte envelope addressed to GD32_BRIDGE_I2C_REG_CMD*/
/* -- so neither term, the whole condition, nor the branch's own return */
/* value had a direct case pinning it before now.                      */
/* --------------------------------------------------------------- */

/* Pins the `i2c_rx_len < 4u` term: every length strictly below the
 * 4-byte minimum envelope (reg + cmd + 0-byte payload + CRC(2)) must be
 * rejected without ever reaching protocol_dispatch(). */
ZTEST(gd32_bridge_transport_i2c, test_short_write_below_minimum_rejected)
{
	uint8_t short_req[3] = { GD32_BRIDGE_I2C_REG_CMD, CMD_PING, 0x00u };
	uint8_t reply[8];

	for (size_t len = 1u; len < 4u; len++) {
		transport_i2c_init();
		zassert_false(write_phase(short_req, len),
		              "%zu-byte write below the 4-byte minimum must not dispatch",
		              len);
		read_phase(reply, 3u);
		zassert_equal(reply[0], STATUS_NO_PENDING, "%zu-byte write stages NO_PENDING", len);
	}
}

/* The zero-byte write phase -- i2c_slave_write_start() then
 * i2c_slave_write_end() with NO i2c_slave_rx_byte() calls in between --
 * is the I2C-only twin of the SPI suite's test_empty_transaction_rewinds
 * (tests/unit/transport_spi/src/test_transport_spi.c:117): writable and
 * reachable today, does NOT depend on issue #4 (the double-write_end()
 * bug the file header's omissions #1/#2 track).  I2C has no rewind
 * concept (nothing was ever staged to rewind to) -- the correct
 * response is the same NO_PENDING rejection as any other envelope short
 * of the 4-byte minimum. */
ZTEST(gd32_bridge_transport_i2c, test_zero_byte_write_stages_no_pending)
{
	uint8_t reply[8];

	transport_i2c_init();

	i2c_slave_write_start();
	zassert_false(i2c_slave_write_end(), "zero-byte write phase must not dispatch");

	read_phase(reply, 3u);
	zassert_equal(reply[0], STATUS_NO_PENDING, "zero-byte write phase stages NO_PENDING");
}

/* Pins the `i2c_rx_buf[0] != GD32_BRIDGE_I2C_REG_CMD` term: a
 * well-formed, correctly-CRC'd 4-byte envelope addressed to a register
 * OTHER than GD32_BRIDGE_I2C_REG_CMD must still be rejected.  Every
 * other case in this file goes through build_write(), which hardcodes
 * GD32_BRIDGE_I2C_REG_CMD as byte 0 -- so this register check has never
 * been exercised with a non-matching register until now. */
ZTEST(gd32_bridge_transport_i2c, test_write_to_non_cmd_register_rejected)
{
	uint8_t       req[4];
	uint8_t       reply[8];
	const uint8_t bogus_reg = (uint8_t)(GD32_BRIDGE_I2C_REG_CMD + 1u);

	transport_i2c_init();

	req[0] = bogus_reg;
	req[1] = CMD_PING;
	const uint16_t crc =
	    crc16_ccitt_false(&req[1], 1u); /* same CMD..PAYLOAD span build_write() uses */
	req[2] = (uint8_t)(crc & 0xFFu);
	req[3] = (uint8_t)(crc >> 8);

	zassert_false(write_phase(req, sizeof req), "a non-CMD register write must not dispatch");
	read_phase(reply, 3u);
	zassert_equal(reply[0], STATUS_NO_PENDING, "non-CMD register write stages NO_PENDING");
}

/* transport_i2c_init() must reset ALL staging state on its own -- not
 * rely on whatever residue a previous test happened to leave behind.
 * Every OTHER case in this file calls transport_i2c_init() first thing,
 * so a gutted (no-op) init is indistinguishable from a correct one for
 * whichever case happens to run FIRST in a given ordering (statics are
 * zero-initialised by C anyway), and only trips for cases running
 * later.  This case is self-contained: it manufactures its OWN dirty
 * state (a completed PING round-trip plus one partial read leaves
 * i2c_tx_cursor != 0, i2c_tx_len != 0 and pending_reply_valid == true)
 * immediately before the transport_i2c_init() under test, so the
 * assertion holds no matter what ran before this test, or whether this
 * test runs first, last, or alone. */
ZTEST(gd32_bridge_transport_i2c, test_init_resets_state_independent_of_order)
{
	uint8_t req[4];
	uint8_t reply[8];

	/* Deliberately dirty the module's static staging state. */
	transport_i2c_init();
	size_t req_len = build_write(req, CMD_PING, NULL, 0u);
	zassert_true(write_phase(req, req_len), "PING write stages a reply (setup)");
	uint8_t one_byte = i2c_slave_tx_next_byte();
	zassert_equal(one_byte, STATUS_OK, "sanity: reply cursor advanced past 0 (setup)");

	/* The init under test -- must clear i2c_rx_len, i2c_tx_len,
	 * i2c_tx_cursor and pending_reply_valid regardless of the dirty
	 * state left above. */
	transport_i2c_init();
	read_phase(reply, 3u);

	const uint16_t crc = crc16_ccitt_false(reply, 1u);

	zassert_equal(reply[0],
	              STATUS_NO_PENDING,
	              "post-init read-before-write is NO_PENDING, not a resurrected/half-drained PING "
	              "reply from the state dirtied before init");
	zassert_equal(reply[1], (uint8_t)(crc & 0xFFu), "CRC lo");
	zassert_equal(reply[2], (uint8_t)(crc >> 8), "CRC hi");
}

ZTEST_SUITE(gd32_bridge_transport_i2c, NULL, NULL, NULL, NULL, NULL);
