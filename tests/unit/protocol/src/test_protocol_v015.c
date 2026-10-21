/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Wire protocol v0.15 at the dispatcher (docs/protocol-v0.15-design.md):
 *
 *   - the extended CMD_LINK_FEATURES form and its grant algorithm (2.2)
 *   - the legacy 1-byte form clearing every 0.15 bit (2.1)
 *   - the unconditional I2C opcode allow-list and its SWD diagnostics (7)
 *   - the feature gates on BATCH / BEGIN2 / READ2
 *   - BEGIN2 request validation and READ2 validation + capacity-before-consume
 *   - CMD_BATCH: validation-before-execution, the allow-list, stop-at-first-error
 *
 * Same fake-HAL link as test_protocol.c.  protocol.c's per-link feature word
 * is process-lifetime state the fake cannot reset, so every case starts by
 * writing the 1-byte `features = 0` form on both links (reset_links()).
 */

#include <string.h>
#include <zephyr/ztest.h>

#include "bridge_hw_fake.h"
#include "protocol.h"

#define CAP 272u

static gd32_bridge_status_t disp(gd32_bridge_link_t link,
                                 uint8_t            cmd,
                                 const uint8_t     *req,
                                 size_t             len,
                                 uint8_t           *reply,
                                 size_t             cap,
                                 size_t            *rlen)
{
	*rlen = 0xDEADu;
	return protocol_dispatch(link, cmd, req, len, reply, cap, rlen);
}

static void put32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static uint32_t get32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* The 1-byte `features = 0` form on both links: process-state reset. */
static void reset_links(void)
{
	uint8_t       reply[CAP];
	size_t        n;
	const uint8_t off = 0u;

	bridge_hw_fake_reset();
	(void)disp(GD32_BRIDGE_LINK_SPI, CMD_LINK_FEATURES, &off, 1u, reply, sizeof reply, &n);
	(void)disp(GD32_BRIDGE_LINK_I2C, CMD_LINK_FEATURES, &off, 1u, reply, sizeof reply, &n);
	bridge_hw_fake_reset();
}

/* Extended negotiation on `link`; returns the status, fills granted/supported/mp. */
static gd32_bridge_status_t negotiate_ext(gd32_bridge_link_t link,
                                          uint32_t           want,
                                          uint16_t           mp_req,
                                          uint32_t          *granted,
                                          uint32_t          *supported,
                                          uint16_t          *mp)
{
	uint8_t req[6];
	uint8_t reply[CAP];
	size_t  n;

	put32(req, want);
	req[4]                        = (uint8_t)mp_req;
	req[5]                        = (uint8_t)(mp_req >> 8);
	const gd32_bridge_status_t st = disp(link, CMD_LINK_FEATURES, req, 6u, reply, sizeof reply, &n);
	if (st == STATUS_OK) {
		zassert_equal(n, 10u, "extended reply is 10 bytes");
		if (granted) *granted = get32(&reply[0]);
		if (supported) *supported = get32(&reply[4]);
		if (mp) *mp = (uint16_t)(reply[8] | (reply[9] << 8));
	} else {
		zassert_equal(n, 0u, "an error answers an empty payload");
	}
	return st;
}

#define ALL_FEATS \
	(GD32_BRIDGE_LINK_FEAT_STATUS_SEQ | GD32_BRIDGE_LINK_FEAT_BIG_FRAME | \
	 GD32_BRIDGE_LINK_FEAT_ATTN | GD32_BRIDGE_LINK_FEAT_ADC_STREAM2 | GD32_BRIDGE_LINK_FEAT_BATCH)

/* ------------------------------------------------------------------ */
/* Version                                                             */
/* ------------------------------------------------------------------ */

ZTEST(protocol, test_v015_get_version_reports_0_15_0)
{
	uint8_t reply[CAP];
	size_t  n;

	reset_links();
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_GET_VERSION, NULL, 0u, reply, sizeof reply, &n),
	              STATUS_OK);
	zassert_equal(n, 3u);
	zassert_equal(reply[0], 0u);
	zassert_equal(reply[1], 15u, "PROTOCOL_VERSION_MINOR 15");
	zassert_equal(reply[2], 0u);
	zassert_equal(CMD_BATCH, 0x04);
	zassert_equal(CMD_ADC_STREAM_BEGIN2, 0x3B);
	zassert_equal(CMD_ADC_STREAM_READ2, 0x3C);
}

/* ------------------------------------------------------------------ */
/* Link feature negotiation                                             */
/* ------------------------------------------------------------------ */

/* The constants are the wire contract: pin the bit values. */
ZTEST(protocol, test_v015_feature_bit_values)
{
	zassert_equal(GD32_BRIDGE_LINK_FEAT_STATUS_SEQ, 0x01u);
	zassert_equal(GD32_BRIDGE_LINK_FEAT_BIG_FRAME, 0x02u);
	zassert_equal(GD32_BRIDGE_LINK_FEAT_ATTN, 0x04u);
	zassert_equal(GD32_BRIDGE_LINK_FEAT_ADC_STREAM2, 0x08u);
	zassert_equal(GD32_BRIDGE_LINK_FEAT_BATCH, 0x10u);
	zassert_equal(GD32_BRIDGE_SPI_BIG_MAX_PAYLOAD_BYTES, 252u);
	zassert_equal(GD32_BRIDGE_MAX_PAYLOAD_BYTES, 65u, "the base envelope is unchanged");
	zassert_equal(GD32_BRIDGE_BATCH_MAX_OPS, 16u);
	zassert_equal(GD32_BRIDGE_ADC_STREAM2_GUARD, 8u);
	zassert_equal(GD32_BRIDGE_READ2_HDR_BYTES, 9u);
}

/* 2.2 step 1: the supported word depends on build and link. */
ZTEST(protocol, test_v015_supported_word_by_backend_and_link)
{
	uint32_t g, sup;
	uint16_t mp;

	reset_links();
	zassert_equal(negotiate_ext(GD32_BRIDGE_LINK_SPI, 0u, 65u, &g, &sup, &mp), STATUS_OK);
	zassert_equal(sup, 0x1Fu, "SPI on the gd32 backend");

	bridge_hw_fake_set_link_hw(0, 0);
	zassert_equal(negotiate_ext(GD32_BRIDGE_LINK_SPI, 0u, 65u, &g, &sup, &mp), STATUS_OK);
	zassert_equal(sup, 0x13u, "SPI on the stub backend: no ATTN, no ADC_STREAM2");

	zassert_equal(negotiate_ext(GD32_BRIDGE_LINK_I2C, ALL_FEATS, 252u, &g, &sup, &mp), STATUS_OK);
	zassert_equal(sup, 0x01u, "I2C only echoes STATUS_SEQ");
	zassert_equal(g, 0x01u, "and grants nothing else");
	zassert_equal(mp, 65u);
	reset_links();
}

ZTEST(protocol, test_v015_grant_all_on_the_gd32_backend)
{
	uint32_t g, sup;
	uint16_t mp;

	reset_links();
	zassert_equal(negotiate_ext(GD32_BRIDGE_LINK_SPI, ALL_FEATS, 252u, &g, &sup, &mp), STATUS_OK);
	zassert_equal(g, 0x1Fu);
	zassert_equal(sup, 0x1Fu);
	zassert_equal(mp, 252u);
	zassert_equal(protocol_link_features(GD32_BRIDGE_LINK_SPI), 0x1Fu, "armed before the reply");
	zassert_equal(protocol_link_max_payload(GD32_BRIDGE_LINK_SPI), 252u);
	zassert_true(bridge_hw_fake_attn_enabled(), "the ATTN pin was switched on");
	zassert_equal(protocol_link_features(GD32_BRIDGE_LINK_I2C), 0u, "per-link: I2C untouched");
	zassert_equal(protocol_link_max_payload(GD32_BRIDGE_LINK_I2C), 65u);
	reset_links();
}

/* Reserved bits 5..31 are never granted. */
ZTEST(protocol, test_v015_reserved_bits_are_never_granted)
{
	uint32_t g, sup;
	uint16_t mp;

	reset_links();
	zassert_equal(negotiate_ext(GD32_BRIDGE_LINK_SPI, 0xFFFFFFFFu, 252u, &g, &sup, &mp), STATUS_OK);
	zassert_equal(g, 0x1Fu);
	zassert_equal(protocol_link_features(GD32_BRIDGE_LINK_SPI) & ~0x1Fu, 0u);
	reset_links();
}

/* 2.2 step 3: ATTN needs STATUS_SEQ. */
ZTEST(protocol, test_v015_attn_is_dropped_without_status_seq)
{
	uint32_t g, sup;
	uint16_t mp;

	reset_links();
	zassert_equal(negotiate_ext(GD32_BRIDGE_LINK_SPI,
	                            GD32_BRIDGE_LINK_FEAT_ATTN | GD32_BRIDGE_LINK_FEAT_BATCH,
	                            65u,
	                            &g,
	                            &sup,
	                            &mp),
	              STATUS_OK);
	zassert_equal(g, GD32_BRIDGE_LINK_FEAT_BATCH, "ATTN refused without STATUS_SEQ");
	zassert_equal(sup, 0x1Fu, "but still listed as supported");
	zassert_false(bridge_hw_fake_attn_enabled());
	zassert_equal(bridge_hw_fake_attn_enable_calls(), 0u, "no pin activity for a refused bit");
	reset_links();
}

/* 2.2 step 3 / 4.2 F2: never grant ATTN while a debugger is attached. */
ZTEST(protocol, test_v015_attn_is_refused_while_a_debugger_is_attached)
{
	uint32_t g, sup;
	uint16_t mp;

	reset_links();
	bridge_hw_fake_set_debugger_attached(1);
	zassert_equal(negotiate_ext(GD32_BRIDGE_LINK_SPI, ALL_FEATS, 252u, &g, &sup, &mp), STATUS_OK);
	zassert_equal(g, 0x1Bu, "everything but ATTN");
	zassert_equal(sup, 0x1Fu);
	zassert_equal(bridge_hw_fake_attn_enable_calls(), 0u, "PA14 is never driven");
	reset_links();
}

/* 2.2 step 4: BIG_FRAME needs a max_payload_req above 65; the grant clamps to 252. */
ZTEST(protocol, test_v015_big_frame_max_payload_clamping)
{
	uint32_t       g, sup;
	uint16_t       mp;
	const uint32_t want = GD32_BRIDGE_LINK_FEAT_BIG_FRAME | GD32_BRIDGE_LINK_FEAT_STATUS_SEQ;

	reset_links();
	zassert_equal(negotiate_ext(GD32_BRIDGE_LINK_SPI, want, 65u, &g, &sup, &mp), STATUS_OK);
	zassert_equal(g, GD32_BRIDGE_LINK_FEAT_STATUS_SEQ, "65 clamps to 65: BIG_FRAME dropped");
	zassert_equal(mp, 65u);
	zassert_equal(negotiate_ext(GD32_BRIDGE_LINK_SPI, want, 0u, &g, &sup, &mp), STATUS_OK);
	zassert_equal(g, GD32_BRIDGE_LINK_FEAT_STATUS_SEQ, "0 clamps up to 65: dropped");
	zassert_equal(mp, 65u);
	zassert_equal(negotiate_ext(GD32_BRIDGE_LINK_SPI, want, 66u, &g, &sup, &mp), STATUS_OK);
	zassert_equal(g, want);
	zassert_equal(mp, 66u, "66 is the smallest BIG_FRAME ceiling");
	zassert_equal(negotiate_ext(GD32_BRIDGE_LINK_SPI, want, 100u, &g, &sup, &mp), STATUS_OK);
	zassert_equal(mp, 100u);
	zassert_equal(negotiate_ext(GD32_BRIDGE_LINK_SPI, want, 300u, &g, &sup, &mp), STATUS_OK);
	zassert_equal(mp, 252u, "clamped to 252");
	zassert_equal(
	    negotiate_ext(GD32_BRIDGE_LINK_SPI, GD32_BRIDGE_LINK_FEAT_STATUS_SEQ, 252u, &g, &sup, &mp),
	    STATUS_OK);
	zassert_equal(mp, 65u, "no BIG_FRAME bit: max_payload is 65 whatever was asked");
	reset_links();
}

/* The ATTN pin transition runs only when the bit changes. */
ZTEST(protocol, test_v015_attn_pin_transitions_only_on_a_change)
{
	uint32_t g, sup;
	uint16_t mp;

	reset_links();
	negotiate_ext(GD32_BRIDGE_LINK_SPI, ALL_FEATS, 252u, &g, &sup, &mp);
	zassert_equal(bridge_hw_fake_attn_enable_calls(), 1u, "enabled once");
	negotiate_ext(GD32_BRIDGE_LINK_SPI, ALL_FEATS, 252u, &g, &sup, &mp);
	zassert_equal(
	    bridge_hw_fake_attn_enable_calls(), 1u, "re-sending the same want: no pin activity");
	negotiate_ext(
	    GD32_BRIDGE_LINK_SPI, ALL_FEATS & ~GD32_BRIDGE_LINK_FEAT_ATTN, 252u, &g, &sup, &mp);
	zassert_equal(bridge_hw_fake_attn_enable_calls(), 2u, "removing ATTN disables the pin");
	zassert_false(bridge_hw_fake_attn_enabled());
	reset_links();
}

/* An ATTN enable the HAL cannot perform is dropped from the grant, not lied about. */
ZTEST(protocol, test_v015_attn_enable_failure_drops_the_bit)
{
	uint32_t g, sup;
	uint16_t mp;

	reset_links();
	bridge_hw_fake_force(FAKE_FN_ATTN_ENABLE, BRIDGE_HW_ERR_NOTIMPL);
	negotiate_ext(GD32_BRIDGE_LINK_SPI, ALL_FEATS, 252u, &g, &sup, &mp);
	zassert_equal(g & GD32_BRIDGE_LINK_FEAT_ATTN, 0u);
	zassert_equal(protocol_link_features(GD32_BRIDGE_LINK_SPI) & GD32_BRIDGE_LINK_FEAT_ATTN, 0u);
	reset_links();
}

/* 2.1: the 1-byte form keeps only STATUS_SEQ and clears every 0.15 bit and mp. */
ZTEST(protocol, test_v015_one_byte_form_clears_the_new_bits)
{
	uint32_t      g, sup;
	uint16_t      mp;
	uint8_t       reply[CAP];
	size_t        n;
	const uint8_t one = 0xFFu;

	reset_links();
	negotiate_ext(GD32_BRIDGE_LINK_SPI, ALL_FEATS, 252u, &g, &sup, &mp);
	zassert_equal(protocol_link_features(GD32_BRIDGE_LINK_SPI), 0x1Fu);

	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_LINK_FEATURES, &one, 1u, reply, sizeof reply, &n),
	              STATUS_OK);
	zassert_equal(n, 1u, "1-byte reply, byte-identical to 0.14");
	zassert_equal(reply[0], 0x01u, "features & 0x01");
	zassert_equal(
	    protocol_link_features(GD32_BRIDGE_LINK_SPI), 0x01u, "BIG/ATTN/STREAM2/BATCH cleared");
	zassert_equal(protocol_link_max_payload(GD32_BRIDGE_LINK_SPI), 65u, "max_payload back to 65");
	zassert_false(bridge_hw_fake_attn_enabled(), "ATTN pin released with its bit");
	reset_links();
}

/* Any other request length: INVAL, empty payload, link state unchanged. */
ZTEST(protocol, test_v015_bad_link_features_lengths_change_nothing)
{
	uint32_t            g, sup;
	uint16_t            mp;
	uint8_t             req[8] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00 };
	uint8_t             reply[CAP];
	size_t              n;
	static const size_t bad[] = { 0u, 2u, 3u, 4u, 5u, 7u, 8u };

	reset_links();
	negotiate_ext(GD32_BRIDGE_LINK_SPI, GD32_BRIDGE_LINK_FEAT_BATCH, 65u, &g, &sup, &mp);
	for (size_t i = 0; i < sizeof bad / sizeof bad[0]; i++) {
		zassert_equal(
		    disp(GD32_BRIDGE_LINK_SPI, CMD_LINK_FEATURES, req, bad[i], reply, sizeof reply, &n),
		    STATUS_INVAL,
		    "req_len %u",
		    (unsigned)bad[i]);
		zassert_equal(n, 0u);
		zassert_equal(protocol_link_features(GD32_BRIDGE_LINK_SPI),
		              GD32_BRIDGE_LINK_FEAT_BATCH,
		              "state unchanged after a bad length");
	}
	reset_links();
}

ZTEST(protocol, test_v015_extended_reply_needs_ten_bytes_of_capacity)
{
	uint8_t req[6] = { 0x01, 0, 0, 0, 65, 0 };
	uint8_t reply[CAP];
	size_t  n;

	reset_links();
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_LINK_FEATURES, req, 6u, reply, 9u, &n),
	              STATUS_NOMEM);
	reset_links();
}

/* ------------------------------------------------------------------ */
/* I2C opcode policy                                                    */
/* ------------------------------------------------------------------ */

static bool i2c_allowed(unsigned cmd)
{
	switch (cmd) {
	case 0x00:
	case 0x01:
	case 0x02:
	case 0x03:
	case 0x10:
	case 0x11:
	case 0x41:
	case 0x42:
	case 0x81:
		return true;
	default:
		return cmd >= 0xF0u;
	}
}

/* Every one of the 256 opcodes, against the allow-list of design section 7. */
ZTEST(protocol, test_v015_i2c_allow_list_exhaustive)
{
	uint8_t reply[CAP];
	size_t  n;
	uint8_t req[16] = { 0 };

	reset_links();
	for (unsigned cmd = 0u; cmd < 256u; cmd++) {
		const uint32_t before = bridge_i2c_denied_count;
		/* Empty payload is fine: a denied opcode never reaches its handler, an
		 * allowed one answers whatever its own validation says. */
		const gd32_bridge_status_t st =
		    disp(GD32_BRIDGE_LINK_I2C, (uint8_t)cmd, req, 0u, reply, sizeof reply, &n);
		if (i2c_allowed(cmd)) {
			zassert_equal(bridge_i2c_denied_count, before, "cmd 0x%02X must not be denied", cmd);
		} else {
			zassert_equal(st, STATUS_NOSUPPORT, "cmd 0x%02X denied with NOSUPPORT", cmd);
			zassert_equal(n, 0u, "cmd 0x%02X: empty payload", cmd);
			zassert_equal(bridge_i2c_denied_count, before + 1u, "cmd 0x%02X counted", cmd);
			zassert_equal(bridge_i2c_denied_last_cmd, (uint8_t)cmd);
		}
	}
	/* Nothing hardware-facing ran for any of the denied opcodes. */
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_ADC_READ), 0u);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_PWM_SET), 0u);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_TRNG_READ), 0u);
	reset_links();
}

/* The same opcodes are untouched on SPI (no per-link restriction there). */
ZTEST(protocol, test_v015_spi_link_is_unrestricted)
{
	uint8_t       reply[CAP];
	size_t        n;
	const uint8_t adc[2] = { 0u, 1u };

	reset_links();
	const uint32_t before = bridge_i2c_denied_count;
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_READ, adc, 2u, reply, sizeof reply, &n),
	              STATUS_OK);
	zassert_equal(bridge_i2c_denied_count, before, "SPI requests are never counted as I2C denials");
	zassert_equal(disp(GD32_BRIDGE_LINK_I2C, CMD_ADC_READ, adc, 2u, reply, sizeof reply, &n),
	              STATUS_NOSUPPORT);
	reset_links();
}

/* The allow-listed I2C opcodes still work, and a denial does not poison them. */
ZTEST(protocol, test_v015_i2c_allowed_opcodes_still_run)
{
	uint8_t       reply[CAP];
	size_t        n;
	const uint8_t mask[4] = { 1u, 0u, 0u, 0u };

	reset_links();
	zassert_equal(disp(GD32_BRIDGE_LINK_I2C, CMD_PING, NULL, 0u, reply, sizeof reply, &n),
	              STATUS_OK);
	zassert_equal(disp(GD32_BRIDGE_LINK_I2C, CMD_GET_VERSION, NULL, 0u, reply, sizeof reply, &n),
	              STATUS_OK);
	zassert_equal(reply[1], 15u);
	zassert_equal(disp(GD32_BRIDGE_LINK_I2C, CMD_GPIO_READ, mask, 4u, reply, sizeof reply, &n),
	              STATUS_OK);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_GPIO_READ), 1u);
	reset_links();
}

/* The I2C-only policy also covers the 0.15 opcodes: NOSUPPORT, counted. */
ZTEST(protocol, test_v015_new_opcodes_are_denied_on_i2c)
{
	uint8_t       reply[CAP];
	size_t        n;
	const uint8_t req[2] = { 0u, 1u };

	reset_links();
	const uint32_t before = bridge_i2c_denied_count;
	zassert_equal(disp(GD32_BRIDGE_LINK_I2C, CMD_BATCH, req, 2u, reply, sizeof reply, &n),
	              STATUS_NOSUPPORT);
	zassert_equal(
	    disp(GD32_BRIDGE_LINK_I2C, CMD_ADC_STREAM_READ2, req, 2u, reply, sizeof reply, &n),
	    STATUS_NOSUPPORT);
	zassert_equal(
	    disp(GD32_BRIDGE_LINK_I2C, CMD_ADC_STREAM_BEGIN2, req, 2u, reply, sizeof reply, &n),
	    STATUS_NOSUPPORT);
	zassert_equal(bridge_i2c_denied_count, before + 3u);
	reset_links();
}

/* ------------------------------------------------------------------ */
/* Feature gates                                                        */
/* ------------------------------------------------------------------ */

ZTEST(protocol, test_v015_new_opcodes_answer_nosupport_until_granted)
{
	uint8_t reply[CAP];
	size_t  n;
	uint8_t batch[3]   = { 1u, CMD_PING, 0u };
	uint8_t read2[2]   = { 0u, 4u };
	uint8_t begin2[12] = { 0, 0, 0, 0, 0xE8, 0x03, 0, 0, 0, 0, 0, 0 };

	reset_links();
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, batch, 3u, reply, sizeof reply, &n),
	              STATUS_NOSUPPORT);
	zassert_equal(n, 0u);
	zassert_equal(
	    disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, read2, 2u, reply, sizeof reply, &n),
	    STATUS_NOSUPPORT);
	zassert_equal(
	    disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_BEGIN2, begin2, 12u, reply, sizeof reply, &n),
	    STATUS_NOSUPPORT);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_ADC_STREAM_BEGIN2), 0u);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_ADC_STREAM_READ2), 0u);

	/* Each gate is its own bit: BATCH does not unlock the stream opcodes. */
	negotiate_ext(GD32_BRIDGE_LINK_SPI, GD32_BRIDGE_LINK_FEAT_BATCH, 65u, NULL, NULL, NULL);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, batch, 3u, reply, sizeof reply, &n),
	              STATUS_OK);
	zassert_equal(
	    disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, read2, 2u, reply, sizeof reply, &n),
	    STATUS_NOSUPPORT);

	/* Removing a bit takes effect from the next request. */
	negotiate_ext(GD32_BRIDGE_LINK_SPI, GD32_BRIDGE_LINK_FEAT_ADC_STREAM2, 65u, NULL, NULL, NULL);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, batch, 3u, reply, sizeof reply, &n),
	              STATUS_NOSUPPORT);
	zassert_equal(
	    disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, read2, 2u, reply, sizeof reply, &n),
	    STATUS_OK);
	reset_links();
}

/* ------------------------------------------------------------------ */
/* BEGIN2                                                               */
/* ------------------------------------------------------------------ */

static void begin2_req(uint8_t *r,
                       uint8_t  sid,
                       uint8_t  ch,
                       uint8_t  trig,
                       uint8_t  arg,
                       uint32_t rate,
                       uint16_t wm,
                       uint16_t rsv)
{
	r[0] = sid;
	r[1] = ch;
	r[2] = trig;
	r[3] = arg;
	put32(&r[4], rate);
	r[8]  = (uint8_t)wm;
	r[9]  = (uint8_t)(wm >> 8);
	r[10] = (uint8_t)rsv;
	r[11] = (uint8_t)(rsv >> 8);
}

ZTEST(protocol, test_v015_begin2_request_validation_table)
{
	uint8_t r[12];
	uint8_t reply[CAP];
	size_t  n;

	reset_links();
	negotiate_ext(GD32_BRIDGE_LINK_SPI, GD32_BRIDGE_LINK_FEAT_ADC_STREAM2, 65u, NULL, NULL, NULL);

	struct {
		uint8_t              sid, ch, trig, arg;
		uint32_t             rate;
		uint16_t             wm, rsv;
		gd32_bridge_status_t want;
		const char          *what;
	} t[] = {
		{ 0, 0, 0, 0, 1000, 256, 0, STATUS_OK, "valid" },
		{ 1, 7, 0, 0, 1, 0, 0, STATUS_OK, "stream 1 channel 7 rate 1 W 0" },
		{ 0, 0, 0, 0, 100000, 512, 0, STATUS_OK, "rate ceiling, W 512" },
		{ 2, 0, 0, 0, 1000, 0, 0, STATUS_INVAL, "stream_id 2" },
		{ 0, 0, 6, 0, 1000, 0, 0, STATUS_INVAL, "trigger_src 6 is malformed" },
		{ 0, 0, 0xFF, 0, 1000, 0, 0, STATUS_INVAL, "trigger_src 0xFF" },
		{ 0, 0, 1, 0, 1000, 0, 0, STATUS_NOSUPPORT, "TIMER0_CC reserved" },
		{ 0, 0, 2, 3, 1000, 0, 0, STATUS_NOSUPPORT, "TIMER7_CC reserved" },
		{ 0, 0, 3, 0, 1000, 0, 0, STATUS_NOSUPPORT, "TIMER5_TRGO reserved" },
		{ 0, 0, 4, 0, 1000, 0, 0, STATUS_NOSUPPORT, "TIMER6_TRGO reserved" },
		{ 0, 0, 5, 7, 1000, 0, 0, STATUS_NOSUPPORT, "EXTI reserved" },
		{ 0, 0, 0, 1, 1000, 0, 0, STATUS_INVAL, "trigger_arg != 0 for PACE_TIMER" },
		{ 0, 0, 0, 0, 1000, 0, 1, STATUS_INVAL, "reserved != 0" },
		{ 0, 0, 0, 0, 0, 0, 0, STATUS_INVAL, "rate 0" },
		{ 0, 0, 0, 0, 100001, 0, 0, STATUS_OUT_OF_RANGE, "rate above 100000" },
		{ 0, 0, 0, 0, 1000, 8, 0, STATUS_INVAL, "watermark 8 not allowed" },
		{ 0, 0, 0, 0, 1000, 17, 0, STATUS_INVAL, "watermark 17 not allowed" },
		{ 0, 0, 0, 0, 1000, 1024, 0, STATUS_INVAL, "watermark 1024 not allowed" },
	};
	for (size_t i = 0; i < sizeof t / sizeof t[0]; i++) {
		begin2_req(r, t[i].sid, t[i].ch, t[i].trig, t[i].arg, t[i].rate, t[i].wm, t[i].rsv);
		zassert_equal(
		    disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_BEGIN2, r, 12u, reply, sizeof reply, &n),
		    t[i].want,
		    "%s",
		    t[i].what);
	}

	/* The six accepted watermarks. */
	static const uint16_t good_wm[] = { 0, 16, 32, 64, 128, 256, 512 };
	for (size_t i = 0; i < sizeof good_wm / sizeof good_wm[0]; i++) {
		begin2_req(r, 0, 0, 0, 0, 1000, good_wm[i], 0);
		zassert_equal(
		    disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_BEGIN2, r, 12u, reply, sizeof reply, &n),
		    STATUS_OK,
		    "watermark %u",
		    (unsigned)good_wm[i]);
	}

	/* Wrong lengths. */
	zassert_equal(
	    disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_BEGIN2, r, 11u, reply, sizeof reply, &n),
	    STATUS_INVAL);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_BEGIN2, r, 0u, reply, sizeof reply, &n),
	              STATUS_INVAL);
	reset_links();
}

ZTEST(protocol, test_v015_begin2_reply_layout_and_hal_error_mapping)
{
	uint8_t r[12];
	uint8_t reply[CAP];
	size_t  n;

	reset_links();
	negotiate_ext(GD32_BRIDGE_LINK_SPI, GD32_BRIDGE_LINK_FEAT_ADC_STREAM2, 65u, NULL, NULL, NULL);
	const bridge_hw_adc_stream2_info_t info = {
		.tick_hz      = 1000000u,
		.period_ticks = 3333u,
		.full_scale   = 1023u,
		.vref_mv      = 1795u,
		.watermark    = 64u,
		.ring_depth   = 128u,
		.flags        = BRIDGE_HW_ADC_STREAM2_FLAG_VREF_MEASURED,
	};
	bridge_hw_fake_begin2_set_info(&info);
	begin2_req(r, 1, 3, 0, 0, 300, 64, 0);
	zassert_equal(
	    disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_BEGIN2, r, 12u, reply, sizeof reply, &n),
	    STATUS_OK);
	zassert_equal(n, 17u);
	zassert_equal(get32(&reply[0]), 1000000u, "tick_hz @0");
	zassert_equal(get32(&reply[4]), 3333u, "period_ticks @4");
	zassert_equal(reply[8] | (reply[9] << 8), 1023, "full_scale @8");
	zassert_equal(reply[10] | (reply[11] << 8), 1795, "vref_mv @10");
	zassert_equal(reply[12], 0x01u, "flags @12");
	zassert_equal(reply[13] | (reply[14] << 8), 64, "watermark @13");
	zassert_equal(reply[15] | (reply[16] << 8), 128, "ring_depth @15");

	uint8_t  sid, ch;
	uint32_t rate;
	uint16_t wm;
	bridge_hw_fake_begin2_get_last(&sid, &ch, &rate, &wm);
	zassert_equal(sid, 1u);
	zassert_equal(ch, 3u);
	zassert_equal(rate, 300u);
	zassert_equal(wm, 64u);

	/* BEGIN2 status summary: each HAL error reaches its wire code. */
	static const struct {
		int                  rv;
		gd32_bridge_status_t st;
	} map[] = {
		{ BRIDGE_HW_ERR_INVAL, STATUS_INVAL }, { BRIDGE_HW_ERR_RANGE, STATUS_OUT_OF_RANGE },
		{ BRIDGE_HW_ERR_BUSY, STATUS_BUSY },   { BRIDGE_HW_ERR_NOT_READY, STATUS_NOT_READY },
		{ BRIDGE_HW_ERR_IO, STATUS_IO },       { BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	};
	for (size_t i = 0; i < sizeof map / sizeof map[0]; i++) {
		bridge_hw_fake_force(FAKE_FN_ADC_STREAM_BEGIN2, map[i].rv);
		zassert_equal(
		    disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_BEGIN2, r, 12u, reply, sizeof reply, &n),
		    map[i].st,
		    "HAL %d",
		    map[i].rv);
		zassert_equal(n, 0u);
	}
	bridge_hw_fake_force(FAKE_FN_ADC_STREAM_BEGIN2, BRIDGE_HW_OK);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_BEGIN2, r, 12u, reply, 16u, &n),
	              STATUS_NOMEM,
	              "17-byte reply needs 17 bytes of capacity");
	reset_links();
}

/* ------------------------------------------------------------------ */
/* READ2                                                                */
/* ------------------------------------------------------------------ */

ZTEST(protocol, test_v015_read2_validation_and_ceiling)
{
	uint8_t reply[CAP];
	size_t  n;
	uint8_t r[2];

	reset_links();
	negotiate_ext(GD32_BRIDGE_LINK_SPI, GD32_BRIDGE_LINK_FEAT_ADC_STREAM2, 65u, NULL, NULL, NULL);

	r[0] = 0u;
	r[1] = 4u;
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, r, 1u, reply, sizeof reply, &n),
	              STATUS_INVAL);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, r, 3u, reply, sizeof reply, &n),
	              STATUS_INVAL);
	r[0] = 2u;
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, r, 2u, reply, sizeof reply, &n),
	              STATUS_INVAL,
	              "stream_id 2");
	r[0] = 0u;
	r[1] = 0u;
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, r, 2u, reply, sizeof reply, &n),
	              STATUS_INVAL,
	              "max_samples 0");
	r[1] = 29u;
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, r, 2u, reply, sizeof reply, &n),
	              STATUS_OUT_OF_RANGE,
	              "max_samples 29 > (65-9)/2 = 28 at max_payload 65");
	r[1] = 28u;
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, r, 2u, reply, sizeof reply, &n),
	              STATUS_OK,
	              "28 is the ceiling at 65");
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_ADC_STREAM_READ2),
	              1u,
	              "only the valid call reached the HAL");

	/* BIG_FRAME raises the ceiling to (252-9)/2 = 121. */
	negotiate_ext(GD32_BRIDGE_LINK_SPI,
	              GD32_BRIDGE_LINK_FEAT_ADC_STREAM2 | GD32_BRIDGE_LINK_FEAT_BIG_FRAME,
	              252u,
	              NULL,
	              NULL,
	              NULL);
	r[1] = 121u;
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, r, 2u, reply, sizeof reply, &n),
	              STATUS_OK);
	r[1] = 122u;
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, r, 2u, reply, sizeof reply, &n),
	              STATUS_OUT_OF_RANGE);
	reset_links();
}

/* The capacity check precedes the consume: a too-small reply buffer answers
 * NOMEM and the HAL (the ring) is never touched. */
ZTEST(protocol, test_v015_read2_capacity_is_checked_before_the_ring_is_consumed)
{
	uint8_t       reply[CAP];
	size_t        n;
	const uint8_t r[2] = { 0u, 20u }; /* needs 9 + 40 = 49 */

	reset_links();
	negotiate_ext(GD32_BRIDGE_LINK_SPI, GD32_BRIDGE_LINK_FEAT_ADC_STREAM2, 65u, NULL, NULL, NULL);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, r, 2u, reply, 48u, &n),
	              STATUS_NOMEM);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_ADC_STREAM_READ2), 0u, "nothing consumed");
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, r, 2u, reply, 49u, &n),
	              STATUS_OK);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_ADC_STREAM_READ2), 1u);
	reset_links();
}

ZTEST(protocol, test_v015_read2_reply_layout_and_error_mapping)
{
	uint8_t       reply[CAP];
	size_t        n;
	const uint8_t r[2] = { 1u, 8u };

	reset_links();
	negotiate_ext(GD32_BRIDGE_LINK_SPI, GD32_BRIDGE_LINK_FEAT_ADC_STREAM2, 65u, NULL, NULL, NULL);
	static const uint16_t codes[] = { 0x0123u, 0x0456u, 0x0FFFu };
	bridge_hw_fake_read2_seed(0xFFFFFFFEu, 17u, 3u, codes, 0);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, r, 2u, reply, sizeof reply, &n),
	              STATUS_OK);
	zassert_equal(n, 9u + 3u * 2u, "9 + 2*got");
	zassert_equal(get32(&reply[0]), 0xFFFFFFFEu, "first_index @0");
	zassert_equal(get32(&reply[4]), 17u, "dropped @4");
	zassert_equal(reply[8], 3u, "got @8");
	zassert_equal(reply[9] | (reply[10] << 8), 0x0123, "codes @9, u16 LE");
	zassert_equal(reply[13] | (reply[14] << 8), 0x0FFF);

	/* A HAL that reports more than was asked is a contract violation. */
	bridge_hw_fake_read2_seed(0u, 0u, 9u, NULL, 1);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, r, 2u, reply, sizeof reply, &n),
	              STATUS_IO,
	              "got > max_samples");

	static const struct {
		int                  rv;
		gd32_bridge_status_t st;
	} map[] = {
		{ BRIDGE_HW_ERR_INVAL, STATUS_INVAL },        /* stream inactive / started with BEGIN */
		{ BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },  /* FFT-bound */
		{ BRIDGE_HW_ERR_RANGE, STATUS_OUT_OF_RANGE }, /* dsp_cfg_bad */
		{ BRIDGE_HW_ERR_IO, STATUS_IO },              /* DMA ERRIF / ROVF recal / dsp_sat */
	};
	for (size_t i = 0; i < sizeof map / sizeof map[0]; i++) {
		bridge_hw_fake_force(FAKE_FN_ADC_STREAM_READ2, map[i].rv);
		zassert_equal(
		    disp(GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ2, r, 2u, reply, sizeof reply, &n),
		    map[i].st,
		    "HAL %d",
		    map[i].rv);
	}
	reset_links();
}

/* ------------------------------------------------------------------ */
/* CMD_BATCH                                                            */
/* ------------------------------------------------------------------ */

static size_t put_op(uint8_t *b, size_t off, uint8_t op, const uint8_t *args, uint8_t len)
{
	b[off]      = op;
	b[off + 1u] = len;
	for (uint8_t i = 0; i < len; i++)
		b[off + 2u + i] = args ? args[i] : 0u;
	return off + 2u + len;
}

static void batch_link(uint16_t mp)
{
	reset_links();
	negotiate_ext(GD32_BRIDGE_LINK_SPI,
	              GD32_BRIDGE_LINK_FEAT_BATCH | GD32_BRIDGE_LINK_FEAT_ADC_STREAM2 |
	                  (mp > 65u ? GD32_BRIDGE_LINK_FEAT_BIG_FRAME : 0u),
	              mp,
	              NULL,
	              NULL,
	              NULL);
}

ZTEST(protocol, test_v015_batch_rejects_two_read2_on_one_stream)
{
	uint8_t       b[CAP];
	uint8_t       reply[CAP];
	size_t        n, len;
	const uint8_t s0[2] = { 0u, 4u };
	const uint8_t s1[2] = { 1u, 4u };

	batch_link(252u);

	/* Same stream twice: refused at validation, nothing executes. */
	b[0] = 2u;
	len  = put_op(b, 1u, CMD_ADC_STREAM_READ2, s0, 2u);
	len  = put_op(b, len, CMD_ADC_STREAM_READ2, s0, 2u);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, len, reply, sizeof reply, &n),
	              STATUS_INVAL,
	              "two READ2 on stream 0");
	zassert_equal(n, 0u);

	/* One READ2 per stream passes validation (stream 0 and 1). */
	len = put_op(b, 1u, CMD_ADC_STREAM_READ2, s0, 2u);
	len = put_op(b, len, CMD_ADC_STREAM_READ2, s1, 2u);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, len, reply, sizeof reply, &n),
	              STATUS_OK,
	              "READ2 on streams 0 and 1");
}

ZTEST(protocol, test_v015_batch_request_validation)
{
	uint8_t b[CAP];
	uint8_t reply[CAP];
	size_t  n, len;

	batch_link(252u);

	/* Empty request, count 0. */
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, 0u, reply, sizeof reply, &n),
	              STATUS_INVAL);
	b[0] = 0u;
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, 1u, reply, sizeof reply, &n),
	              STATUS_INVAL,
	              "count 0");
	zassert_equal(n, 0u);

	/* count > 16 -> OUT_OF_RANGE (before the entries are even looked at). */
	b[0] = 17u;
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, 1u, reply, sizeof reply, &n),
	              STATUS_OUT_OF_RANGE);

	/* 16 PINGs is the most a batch carries. */
	b[0] = 16u;
	len  = 1u;
	for (int i = 0; i < 16; i++)
		len = put_op(b, len, CMD_PING, NULL, 0u);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, len, reply, sizeof reply, &n),
	              STATUS_OK);
	zassert_equal(reply[0], 16u, "executed 16");
	zassert_equal(n, 1u + 16u * 2u);

	/* A truncated entry header, a truncated entry body. */
	b[0] = 1u;
	b[1] = CMD_PING;
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, 2u, reply, sizeof reply, &n),
	              STATUS_INVAL,
	              "header cut");
	b[1] = CMD_GPIO_READ;
	b[2] = 4u;
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, 5u, reply, sizeof reply, &n),
	              STATUS_INVAL,
	              "body cut");
	/* Fewer entries than count. */
	b[0] = 2u;
	len  = put_op(b, 1u, CMD_PING, NULL, 0u);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, len, reply, sizeof reply, &n),
	              STATUS_INVAL,
	              "count > entries");
	/* A trailing byte. */
	b[0]   = 1u;
	len    = put_op(b, 1u, CMD_PING, NULL, 0u);
	b[len] = 0u;
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, len + 1u, reply, sizeof reply, &n),
	              STATUS_INVAL,
	              "trailing");
	/* A wrong fixed length for the op. */
	b[0] = 1u;
	len  = put_op(b, 1u, CMD_GPIO_READ, NULL, 3u);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, len, reply, sizeof reply, &n),
	              STATUS_INVAL,
	              "len != 4");
	len = put_op(b, 1u, CMD_PING, NULL, 1u);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, len, reply, sizeof reply, &n),
	              STATUS_INVAL,
	              "PING len 1");
	reset_links();
}

/* Exhaustive: only the allow-list's 14 opcodes are accepted at validation. */
ZTEST(protocol, test_v015_batch_allow_list_exhaustive)
{
	uint8_t b[CAP];
	uint8_t reply[CAP];
	size_t  n;
	static const struct {
		uint8_t op, len;
	} allowed[] = {
		{ 0x00, 0 }, { 0x10, 4 }, { 0x11, 8 }, { 0x20, 10 }, { 0x21, 1 }, { 0x24, 1 }, { 0x3C, 2 },
		{ 0x40, 0 }, { 0x50, 4 }, { 0x51, 1 }, { 0x60, 1 },  { 0x61, 1 }, { 0x70, 1 }, { 0x90, 12 },
	};

	batch_link(252u);
	for (unsigned op = 0u; op < 256u; op++) {
		int want_len = -1;
		for (size_t i = 0; i < sizeof allowed / sizeof allowed[0]; i++) {
			if (allowed[i].op == op) want_len = allowed[i].len;
		}
		/* Probe with a request whose entry length is the op's own allowed length
		 * (or 0 for ops that are not allowed). */
		const uint8_t len  = (want_len >= 0) ? (uint8_t)want_len : 0u;
		b[0]               = 1u;
		const size_t total = put_op(b, 1u, (uint8_t)op, NULL, len);
		if (op == 0x3C) b[3] = 0u, b[4] = 1u; /* READ2: stream 0, max 1 */
		const gd32_bridge_status_t st =
		    disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, total, reply, sizeof reply, &n);
		if (want_len >= 0) {
			zassert_equal(st, STATUS_OK, "op 0x%02X is on the allow-list", op);
		} else {
			zassert_equal(st,
			              STATUS_INVAL,
			              "op 0x%02X must be refused (nested 0x04, 0x81, 0x28, 0xF0..0xFF, ...)",
			              op);
			zassert_equal(n, 0u);
		}
	}
	reset_links();
}

/* The worst-case reply must fit the negotiated max_payload, else OUT_OF_RANGE. */
ZTEST(protocol, test_v015_batch_worst_case_reply_must_fit_max_payload)
{
	uint8_t       b[CAP];
	uint8_t       reply[CAP];
	size_t        n, len;
	const uint8_t read2_big[2] = { 0u, 120u }; /* worst reply 9 + 240 = 249 */

	/* On a 65-byte link READ2 120 would not even fit one entry... */
	batch_link(65u);
	b[0] = 1u;
	len  = put_op(b, 1u, CMD_ADC_STREAM_READ2, read2_big, 2u);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, len, reply, sizeof reply, &n),
	              STATUS_OUT_OF_RANGE);

	/* ... on a BIG link one such entry fits: 1 + 2 + 249 = 252. */
	batch_link(252u);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, len, reply, sizeof reply, &n),
	              STATUS_OK);
	/* Two do not (streams 0 and 1: one READ2 per stream is allowed). */
	const uint8_t read2_big_s1[2] = { 1u, 120u };
	b[0]                          = 2u;
	len                           = put_op(b, 1u, CMD_ADC_STREAM_READ2, read2_big, 2u);
	len                           = put_op(b, len, CMD_ADC_STREAM_READ2, read2_big_s1, 2u);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, len, reply, sizeof reply, &n),
	              STATUS_OUT_OF_RANGE);

	/* The caller's capacity is honoured too. */
	b[0] = 1u;
	len  = put_op(b, 1u, CMD_ADC_STREAM_READ2, read2_big, 2u);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, len, reply, 100u, &n), STATUS_NOMEM);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_ADC_STREAM_READ2), 1u, "NOMEM ran nothing");
	reset_links();
}

/* Ops run in order through the real handlers; every sub-reply is packed. */
ZTEST(protocol, test_v015_batch_executes_in_order_and_packs_sub_replies)
{
	uint8_t       b[CAP];
	uint8_t       reply[CAP];
	size_t        n, len;
	uint8_t       gw[8]  = { 0x03, 0, 0, 0, 0x02, 0, 0, 0 }; /* mask bits 0,1; levels bit 1 */
	const uint8_t dac[1] = { 1u };

	batch_link(252u);
	bridge_hw_fake_gpio_set_pads(0u);
	b[0] = 4u;
	len  = put_op(b, 1u, CMD_GPIO_WRITE, gw, 8u);
	len  = put_op(b, len, CMD_GPIO_READ, gw, 4u);
	len  = put_op(b, len, CMD_DAC_GET, dac, 1u);
	len  = put_op(b, len, CMD_PING, NULL, 0u);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, len, reply, sizeof reply, &n),
	              STATUS_OK);
	zassert_equal(reply[0], 4u, "executed");
	zassert_equal(reply[1], STATUS_OK);
	zassert_equal(reply[2], 0u, "GPIO_WRITE len 0");
	zassert_equal(reply[3], STATUS_OK);
	zassert_equal(reply[4], 4u, "GPIO_READ len 4");
	zassert_equal(
	    get32(&reply[5]), 0x02u, "the write that ran first is visible to the read after it");
	zassert_equal(reply[9], STATUS_OK);
	zassert_equal(reply[10], 2u, "DAC_GET len 2");
	zassert_equal(reply[13], STATUS_OK);
	zassert_equal(reply[14], 0u, "PING len 0");
	zassert_equal(n, 1u + 2u + (2u + 4u) + (2u + 2u) + 2u);
	reset_links();
}

/* Stop at the first non-OK; its sub-reply has len 0; later ops never run. */
ZTEST(protocol, test_v015_batch_stops_at_the_first_non_ok_sub_status)
{
	uint8_t       b[CAP];
	uint8_t       reply[CAP];
	size_t        n, len;
	const uint8_t pwm[1] = { 0u };
	const uint8_t cnt[1] = { 0u };

	batch_link(252u);
	bridge_hw_fake_force(FAKE_FN_PWM_GET, BRIDGE_HW_ERR_IO);
	b[0] = 3u;
	len  = put_op(b, 1u, CMD_PING, NULL, 0u);
	len  = put_op(b, len, CMD_PWM_GET, pwm, 1u);
	len  = put_op(b, len, CMD_COUNTER_READ, cnt, 1u);
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, len, reply, sizeof reply, &n),
	              STATUS_OK,
	              "the outer status is OK whenever the batch validated");
	zassert_equal(reply[0], 2u, "executed counts the failing op");
	zassert_equal(reply[1], STATUS_OK);
	zassert_equal(reply[2], 0u);
	zassert_equal(reply[3], STATUS_IO);
	zassert_equal(reply[4], 0u, "a non-OK sub-status always has len 0");
	zassert_equal(n, 5u);
	zassert_equal(
	    bridge_hw_fake_call_count(FAKE_FN_COUNTER_READ), 0u, "the op after the error never ran");
	reset_links();
}

/* A rejected batch has NO side effects (validation precedes execution). */
ZTEST(protocol, test_v015_rejected_batch_has_no_side_effects)
{
	uint8_t b[CAP];
	uint8_t reply[CAP];
	size_t  n, len;
	uint8_t gw[8] = { 1, 0, 0, 0, 1, 0, 0, 0 };

	batch_link(252u);
	bridge_hw_fake_gpio_set_pads(0u);
	b[0] = 2u;
	len  = put_op(b, 1u, CMD_GPIO_WRITE, gw, 8u); /* would set a pad */
	len  = put_op(b, len, CMD_BATCH, NULL, 0u);   /* then a nested BATCH: refuses the whole batch */
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_BATCH, b, len, reply, sizeof reply, &n),
	              STATUS_INVAL);
	zassert_equal(
	    bridge_hw_fake_gpio_get_pads(), 0u, "the GPIO_WRITE before the bad entry did not run");
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_GPIO_WRITE), 0u);
	reset_links();
}

/* ------------------------------------------------------------------ */
/* POWER_MODE_SET                                                       */
/* ------------------------------------------------------------------ */

ZTEST(protocol, test_v015_power_mode_transition_quiesces_attn)
{
	uint8_t reply[CAP];
	size_t  n;
	uint8_t req[10] = { 0 };

	reset_links();
	zassert_equal(bridge_hw_fake_attn_quiesce_calls(), 0u);
	req[0] = 0u; /* RUN */
	disp(GD32_BRIDGE_LINK_SPI, CMD_POWER_MODE_SET, req, 10u, reply, sizeof reply, &n);
	zassert_equal(bridge_hw_fake_attn_quiesce_calls(), 0u, "RUN is not a transition out of RUN");
	for (uint8_t mode = 1u; mode <= 3u; mode++) {
		req[0] = mode;
		disp(GD32_BRIDGE_LINK_SPI, CMD_POWER_MODE_SET, req, 10u, reply, sizeof reply, &n);
		zassert_equal(
		    bridge_hw_fake_attn_quiesce_calls(), (uint32_t)mode, "mode %u quiesces", mode);
	}
	req[0] = 9u; /* invalid mode: refused before any side effect */
	zassert_equal(disp(GD32_BRIDGE_LINK_SPI, CMD_POWER_MODE_SET, req, 10u, reply, sizeof reply, &n),
	              STATUS_INVAL);
	zassert_equal(bridge_hw_fake_attn_quiesce_calls(), 3u);
	reset_links();
}

/* Removing ADC_STREAM2 mutes the watermark events and clears the pending ones:
 * READ2 would answer NOSUPPORT and nothing else could clear them. */
ZTEST(protocol, test_v015_removing_adc_stream2_clears_and_mutes_events)
{
	uint32_t g, sup;
	uint16_t mp;

	reset_links();
	negotiate_ext(GD32_BRIDGE_LINK_SPI, ALL_FEATS, 252u, &g, &sup, &mp);
	zassert_true(bridge_hw_fake_attn_streams_enabled(), "granted: events live");
	const uint32_t c0 = bridge_hw_fake_attn_event_clear_calls(0u);
	const uint32_t c1 = bridge_hw_fake_attn_event_clear_calls(1u);

	negotiate_ext(
	    GD32_BRIDGE_LINK_SPI, ALL_FEATS & ~GD32_BRIDGE_LINK_FEAT_ADC_STREAM2, 252u, &g, &sup, &mp);
	zassert_false(bridge_hw_fake_attn_streams_enabled(), "muted");
	zassert_equal(bridge_hw_fake_attn_event_clear_calls(0u), c0 + 1u, "stream 0 event cleared");
	zassert_equal(bridge_hw_fake_attn_event_clear_calls(1u), c1 + 1u, "stream 1 event cleared");

	/* re-sending the same want changes nothing */
	negotiate_ext(
	    GD32_BRIDGE_LINK_SPI, ALL_FEATS & ~GD32_BRIDGE_LINK_FEAT_ADC_STREAM2, 252u, &g, &sup, &mp);
	zassert_equal(bridge_hw_fake_attn_event_clear_calls(0u), c0 + 1u);
	reset_links();
}
