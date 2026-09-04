/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * The firmware-side consumer of tests/protocol_vectors.txt that
 * CONTRIBUTING.md and the vector file's own banner claim exists (issue #31
 * finding E5).  Before this file, `grep -rn protocol_vectors -- '*.c' '*.h'`
 * under tests/unit/ returned nothing: the only automation touching the file
 * was tests/gen_protocol_vectors.py's regenerate-and-diff CI job, which
 * proves the generator is deterministic and proves NOTHING about whether the
 * firmware actually emits those bytes (see issue #68's postmortem: that
 * exact gate stayed green through a whole-file CRC byte-order inversion).
 *
 * This suite parses nothing at test runtime.  tests/protocol_vectors.txt is
 * turned into a generated header (protocol_vectors_gen.h, produced by
 * tests/gen_protocol_vectors_header.py -- see tests/unit/CMakeLists.txt's
 * protocol-vectors-header custom command) at BUILD time, never committed,
 * mirroring cmake/gen_build_id.cmake's generate-don't-parse precedent: no
 * cwd-relative file-IO dependency under ctest, and no hand-rolled hex parser
 * in C for -Wconversion to find a narrowing bug in.
 *
 * Links the REAL src/transport_spi.c, src/transport_i2c.c, src/protocol.c,
 * src/bootloader/bootloader.c, src/ota.c, src/crc32.c against
 * hal/bridge_hw_stub.c -- the SAME link set as test_transport_spi /
 * test_transport_i2c (tests/unit/CMakeLists.txt), unpartitioned OTA.  That
 * is a deliberate choice, not a default: the majority of the vectors' own
 * comments describe stub-backend behaviour verbatim ("the stub HAL backend
 * still answers STATUS_NOSUPPORT", "see spi_reply_nosupport for what the
 * stub backend answers today") -- this suite is what makes those comments
 * true by construction instead of merely asserted in a comment.
 *
 * COVERAGE
 *
 *   PV_EXACT   -- the request has its OWN named reply vector in the file,
 *                 reachable byte-for-byte on the stub: PING, GET_VERSION,
 *                 RESET_REASON, DA9292_STATUS_FORWARD, the I2C PING pair,
 *                 and (as two DEDICATED scenario cases below, not the main
 *                 table -- see why there) LINK_FEATURES's granted/stamp=1
 *                 reply and the stamp=5 example.
 *   PV_NOSUPP  -- STATUS_NOSUPPORT, 0-byte payload: every stub-linked
 *                 HAL-backed opcode whose BRIDGE_HW_ERR_NOTIMPL routes to
 *                 STATUS_NOSUPPORT (src/protocol.c, checked against this
 *                 tree at commit bbd5c99 handler-by-handler before writing
 *                 this table), the reserved 0x36 probe (default-case
 *                 dispatch), and the WHOLE 0xF0..0xFF OTA range (the
 *                 unpartitioned build takes src/ota.c's `#else` branch --
 *                 unconditional STATUS_NOSUPPORT, via src/bootloader/
 *                 bootloader.c's bl_dispatch_ota -- regardless of which OTA
 *                 opcode or payload was sent).  This reply is CONSTANT
 *                 (SOF + STATUS with a 0-byte payload has the same CRC every
 *                 time), so the file's own spi_reply_nosupport vector
 *                 legitimately stands in for the whole class.
 *   PV_IO      -- STATUS_IO, 0-byte payload: GPIO_READ/GPIO_WRITE, the one
 *                 HAL-backed pair whose handlers do NOT route through the
 *                 central status_from_hw() mapper (`if (rv < 0) return
 *                 STATUS_IO;`).  Same constant-reply reasoning, against
 *                 spi_reply_io.
 *
 * NOT COVERED (checked against this tree at commit bbd5c99; each reason is
 * why closing it is bigger than this bounded suite, not an oversight):
 *
 *   - spi_ota_begin_reply_slot_b, spi_ota_write_chunk_reply_8b,
 *     spi_ota_verify_reply_match, spi_ota_get_state_reply_ready and
 *     spi_ota_reply_not_ready all assume an ARMED -DBRIDGE_OTA_PARTITIONED
 *     session against a real (or host-buffer-overridden) FMC -- exactly the
 *     scope tests/unit/ota/src/test_ota.c already owns as its OWN, separate
 *     link target (it drives src/ota.c's state machine directly, not
 *     through protocol_dispatch()/a transport).  Reaching these five through
 *     THIS suite would need a partitioned link line stitched through the
 *     transport layer -- a second suite's worth of scope, not a table row.
 *   - spi_adc_spectrum_read_reply_example: its own comment says it is
 *     "REPRESENTATIVE of the wired gd32 HAL body, not the stub backend's
 *     STATUS_NOSUPPORT reply" -- i.e. the file itself documents that the
 *     stub cannot produce it.  Reaching it needs the fake HAL
 *     (tests/unit/fake/), a different link target again (tests/unit/
 *     protocol's), which does not go through the wire transport at all.
 *   - crc16_ccitt_false_test_string (§1): has no paired `_result` vector in
 *     the file (it exists for a human to walk truncation by hand, per its
 *     own comment) -- there is nothing to assert it against.
 *
 * §1's actual pair (crc16_ccitt_false_ref_string / _ref_result) IS driven
 * here, straight from the generated header rather than hand-copied, in
 * test_crc16_matches_committed_self_test_vector below -- the check issue
 * #31 finding E5's "Fix" section describes almost verbatim, except every
 * OTHER value in this file already comes from the vector file instead of
 * being retyped.
 */

#include <stdio.h>
#include <string.h>
#include <zephyr/ztest.h>

#include "protocol.h"
#include "protocol_vectors_gen.h"
#include "transport.h"

/* ---- vector lookup -------------------------------------------------- */

static const pv_vector_t *pv_find(const char *name)
{
	for (size_t i = 0; i < PV_VECTOR_COUNT; i++) {
		if (strcmp(pv_vectors[i].name, name) == 0) {
			return &pv_vectors[i];
		}
	}
	zassert_true(false, "vector %s not found in the generated protocol_vectors_gen.h", name);
	return NULL; /* unreachable: zassert_true(false, ...) longjmp()s out */
}

/* ---- SPI replay: one CS transaction, drained the way the HAL does ---- */

static size_t spi_roundtrip(const pv_vector_t *req, uint8_t *reply, size_t cap)
{
	transport_spi_init();
	spi_slave_cs_low();
	for (size_t i = 0; i < req->len; i++) {
		spi_slave_rx_byte(req->bytes[i]);
	}
	spi_slave_cs_high();

	size_t n = 0;
	while (spi_slave_tx_pending() && n < cap) {
		reply[n++] = spi_slave_tx_next_byte();
	}
	return n;
}

/* ---- PV_EXACT / PV_NOSUPP / PV_IO classification table --------------- */
/*
 * CMD_LINK_FEATURES is deliberately NOT a row here: granting it is a
 * side effect on protocol.c's own `static uint8_t link_features`, which
 * transport_spi_init() does NOT reset (it is transport staging state, not
 * protocol.c's) -- see test_protocol.c's file header for the same fact
 * documented from the fake-HAL suite's side.  Arming it here would stamp
 * every STATUS byte this loop checks AFTER that row, corrupting every
 * PV_EXACT/PV_NOSUPP/PV_IO comparison that follows in registration order
 * (tests/unit/ztest_shim.c runs cases in source order, but everything
 * WITHIN one ZTEST body runs in array order with no isolation).
 * LINK_FEATURES gets its own dedicated case below, positioned after this
 * loop for exactly that reason.
 */
typedef enum {
	PV_EXACT,
	PV_NOSUPP,
	PV_IO,
} pv_class_t;

typedef struct {
	const char *req_name;
	pv_class_t class;
	const char *reply_name; /* PV_EXACT only; NULL otherwise */
} pv_case_t;

/* The GET_VERSION reply vector is NAMED after the protocol triple --
 * gen_protocol_vectors.py emits `spi_get_version_reply_v<maj>_<min>_<pat>` --
 * so every version bump renames it.  Hardcoding that name here meant the
 * lookup broke the first time the triple moved: #69/#70 bumped MINOR 9 -> 10
 * and this table still asked for `..._v0_9_0`, which pv_find() cannot find,
 * failing a suite that was otherwise correct about the wire.  Built from the
 * macros instead, so the next bump cannot repeat it.  The pointer stored in
 * the table is constant; only the buffer's contents are filled at runtime,
 * before the table is ever walked. */
static char pv_get_version_reply_name[48];

static void pv_build_get_version_reply_name(void)
{
	(void)snprintf(pv_get_version_reply_name,
	               sizeof pv_get_version_reply_name,
	               "spi_get_version_reply_v%u_%u_%u",
	               (unsigned)PROTOCOL_VERSION_MAJOR,
	               (unsigned)PROTOCOL_VERSION_MINOR,
	               (unsigned)PROTOCOL_VERSION_PATCH);
}

/* clang-format off */
static const pv_case_t SPI_CASES[] = {
	/* PV_EXACT: the request has its own named reply vector. */
	{ "spi_ping_request",                               PV_EXACT,  "spi_ping_reply_ok" },
	{ "spi_get_version_request",                         PV_EXACT,  pv_get_version_reply_name },
	{ "spi_reset_reason_request",                        PV_EXACT,  "spi_reset_reason_reply_unknown" },
	{ "spi_da9292_status_forward_request",               PV_EXACT,  "spi_da9292_status_forward_reply_no_sample" },

	/* PV_IO: GPIO_READ/WRITE map BRIDGE_HW_ERR_NOTIMPL to STATUS_IO, not
	 * STATUS_NOSUPPORT (src/protocol.c: `if (rv < 0) return STATUS_IO;`). */
	{ "spi_gpio_read_mask_bit0_request",                 PV_IO,     NULL },
	{ "spi_gpio_write_mask_bit0_high_request",           PV_IO,     NULL },

	/* PV_NOSUPP: every other stub-backed HAL opcode, v0.2..v0.5, plus the
	 * reserved 0x36 probe. */
	{ "spi_dac_set_ch0_1650mv_request",                  PV_NOSUPP, NULL },
	{ "spi_dac_get_ch1_request",                         PV_NOSUPP, NULL },
	{ "spi_qenc_read_ch0_request",                       PV_NOSUPP, NULL },
	{ "spi_qenc_reset_ch3_request",                      PV_NOSUPP, NULL },
	{ "spi_counter_read_ch0_request",                    PV_NOSUPP, NULL },
	{ "spi_se_reset_assert_request",                     PV_NOSUPP, NULL },
	{ "spi_se_reset_release_request",                    PV_NOSUPP, NULL },
	{ "spi_pwm_configure_ch0_request",                   PV_NOSUPP, NULL },
	{ "spi_adc_configure_ch3_request",                   PV_NOSUPP, NULL },
	{ "spi_adc_stream_begin_stream0_ch0_1ksps_request",  PV_NOSUPP, NULL },
	{ "spi_adc_stream_read_stream0_max32_request",       PV_NOSUPP, NULL },
	{ "spi_adc_stream_end_stream0_request",              PV_NOSUPP, NULL },
	{ "spi_trng_read_16_request",                        PV_NOSUPP, NULL },
	{ "spi_tmu_compute_sqrt_f32_4p0_request",            PV_NOSUPP, NULL },
	{ "spi_adc_stream_configure_dsp_probe_request",      PV_NOSUPP, NULL },
	{ "spi_pwm_capture_begin_probe_request",             PV_NOSUPP, NULL },
	{ "spi_pwm_single_pulse_probe_request",              PV_NOSUPP, NULL },
	{ "spi_power_mode_set_probe_request",                PV_NOSUPP, NULL },
	{ "spi_adc_dsp_chain_open_probe_request",            PV_NOSUPP, NULL },
	{ "spi_adc_dsp_stage_push_window_hann_request",      PV_NOSUPP, NULL },
	{ "spi_adc_dsp_chain_bind_probe_request",            PV_NOSUPP, NULL },
	{ "spi_pwm_set_ch0_1ms_period_500us_duty_request",   PV_NOSUPP, NULL },
	{ "spi_pwm_get_ch0_request",                         PV_NOSUPP, NULL },
	{ "spi_adc_read_ch0_4samples_request",               PV_NOSUPP, NULL },
	{ "spi_pwm_capture_read_ch0_request",                PV_NOSUPP, NULL },
	{ "spi_pwm_capture_end_ch0_request",                 PV_NOSUPP, NULL },
	{ "spi_timer_sync_t0_master_t7_slave_request",       PV_NOSUPP, NULL },
	{ "spi_adc_spectrum_read_stream0_request",           PV_NOSUPP, NULL },

	/* PV_NOSUPP: the WHOLE 0xF0..0xFF OTA range, unpartitioned build. */
	{ "spi_ota_begin_request",                           PV_NOSUPP, NULL },
	{ "spi_ota_begin_request_v0_7",                      PV_NOSUPP, NULL },
	{ "spi_ota_write_chunk_off0_8b_request",             PV_NOSUPP, NULL },
	{ "spi_ota_verify_request",                          PV_NOSUPP, NULL },
	{ "spi_ota_commit_request",                          PV_NOSUPP, NULL },
	{ "spi_ota_rollback_request",                        PV_NOSUPP, NULL },
	{ "spi_ota_get_state_request",                       PV_NOSUPP, NULL },
	{ "spi_ota_abort_request",                           PV_NOSUPP, NULL },
};
/* clang-format on */

#define N_SPI_CASES (sizeof(SPI_CASES) / sizeof(SPI_CASES[0]))

ZTEST(protocol_vectors, test_spi_requests_match_committed_replies)
{
	uint8_t reply[96];

	/* Fill the version-derived vector name before the table is walked --
	 * SPI_CASES stores a pointer to this buffer, not a literal. */
	pv_build_get_version_reply_name();

	const pv_vector_t *nosupp = pv_find("spi_reply_nosupport");
	const pv_vector_t *io     = pv_find("spi_reply_io");

	for (size_t i = 0; i < N_SPI_CASES; i++) {
		const pv_case_t   *c   = &SPI_CASES[i];
		const pv_vector_t *req = pv_find(c->req_name);
		const pv_vector_t *want =
		    (c->class == PV_EXACT) ? pv_find(c->reply_name) : (c->class == PV_IO ? io : nosupp);

		size_t n = spi_roundtrip(req, reply, sizeof reply);
		zassert_equal(n, want->len, "%s: reply length", c->req_name);
		zassert_mem_equal(reply, want->bytes, want->len, "%s: reply bytes", c->req_name);
	}
}

/* GET_BUILD_ID: the file explicitly commits no reply vector for it (the
 * real 20-byte payload is baked at CMake build time into GD32_BRIDGE_BUILD_ID
 * -- protocol.c's cmake/gen_build_id.cmake target, which this host-test
 * build never runs, so protocol.c falls back to its own compiled-in
 * placeholder).  What IS static regardless of that: STATUS_OK and the fixed
 * SOF + STATUS + 20-byte payload + CRC envelope length -- pin that much,
 * which is everything the wire vector's own comment says is pin-able. */
ZTEST(protocol_vectors, test_get_build_id_request_accepted)
{
	const pv_vector_t *req = pv_find("spi_get_build_id_request");
	uint8_t            reply[32];

	size_t n = spi_roundtrip(req, reply, sizeof reply);

	zassert_equal(n, 1u + 1u + 20u + 2u, "GET_BUILD_ID reply: SOF STATUS payload(20) CRC");
	zassert_equal(reply[0], 0xA5u, "SOF");
	zassert_equal(reply[1], 0x00u, "STATUS_OK");
}

/* I2C: only two vectors exist in the file (i2c_ping_write / i2c_ping_read_ok
 * -- every other framed vector is spi_*, see the file's own §3 vs §4+
 * shape).  Complements, not duplicates, tests/unit/transport_i2c/src/
 * test_transport_i2c.c's hand-built ping_write_frame/ping_reply_frame: that
 * suite pins the SAME two frames as C literals it wrote by hand; this suite
 * pins them against the FILE, so the two can never silently drift apart
 * without one of them going red (see that file's issue #68 note about
 * exactly this suite being the intended eventual replacement for its
 * hand-built frames). */
ZTEST(protocol_vectors, test_i2c_ping_matches_committed_vectors)
{
	const pv_vector_t *req  = pv_find("i2c_ping_write");
	const pv_vector_t *want = pv_find("i2c_ping_read_ok");
	uint8_t            reply[16];

	transport_i2c_init();
	i2c_slave_write_start();
	for (size_t i = 0; i < req->len; i++) {
		i2c_slave_rx_byte(req->bytes[i]);
	}
	zassert_true(i2c_slave_write_end(), "i2c_ping_write must stage a fresh reply");

	for (size_t i = 0; i < want->len; i++) {
		reply[i] = i2c_slave_tx_next_byte();
	}
	zassert_mem_equal(reply, want->bytes, want->len, "i2c_ping_read_ok: reply bytes");
}

/* CMD_LINK_FEATURES: its own dedicated case, positioned AFTER the main
 * classification loop on purpose (see that table's header comment) --
 * granting the feature is a side effect on protocol.c's process-lifetime
 * `link_features` global that would stamp every reply checked after it. */
ZTEST(protocol_vectors, test_link_features_request_matches_committed_vector)
{
	const pv_vector_t *req  = pv_find("spi_link_features_request");
	const pv_vector_t *want = pv_find("spi_link_features_reply_granted_seq1");
	uint8_t            reply[16];

	size_t n = spi_roundtrip(req, reply, sizeof reply);

	zassert_equal(n, want->len, "spi_link_features_reply_granted_seq1: reply length");
	zassert_mem_equal(
	    reply, want->bytes, want->len, "spi_link_features_reply_granted_seq1: reply bytes");
}

/* spi_ping_reply_ok_seq5: the file's own worked example of the STATUS_SEQ
 * stamp reaching 5 (code = STATUS & 0x0F, stamp = STATUS >> 4).  Walks the
 * exact sequence the vector assumes -- negotiate ON (stamp 1, drained), four
 * more fresh PING decodes (stamps 2..5) -- and disables the feature again at
 * the end so this being the LAST case in the file (registration order,
 * tests/unit/ztest_shim.c) does not leave protocol.c's link_features global
 * armed for whatever case a future PR appends after it. */
ZTEST(protocol_vectors, test_spi_stamp5_matches_committed_vector)
{
	const pv_vector_t *lf_on = pv_find("spi_link_features_request");
	const pv_vector_t *ping  = pv_find("spi_ping_request");
	const pv_vector_t *want  = pv_find("spi_ping_reply_ok_seq5");
	uint8_t            reply[16];
	size_t             n = 0;

	transport_spi_init();

	spi_slave_cs_low();
	for (size_t i = 0; i < lf_on->len; i++) {
		spi_slave_rx_byte(lf_on->bytes[i]);
	}
	spi_slave_cs_high();
	while (spi_slave_tx_pending()) {
		(void)spi_slave_tx_next_byte(); /* drain the stamp=1 negotiation reply */
	}

	for (int i = 0; i < 4; i++) { /* four more fresh decodes -> stamps 2..5 */
		spi_slave_cs_low();
		for (size_t j = 0; j < ping->len; j++) {
			spi_slave_rx_byte(ping->bytes[j]);
		}
		spi_slave_cs_high();
		n = 0;
		while (spi_slave_tx_pending() && n < sizeof reply) {
			reply[n++] = spi_slave_tx_next_byte();
		}
	}

	zassert_equal(n, want->len, "spi_ping_reply_ok_seq5: reply length");
	zassert_mem_equal(reply, want->bytes, want->len, "spi_ping_reply_ok_seq5: reply bytes");

	/* Negotiate OFF: hand-built (no committed "disable" vector exists),
	 * same shape as test_transport_spi.c's own negotiate() helper. */
	{
		uint8_t        lf_off[5] = { GD32_BRIDGE_SOF, CMD_LINK_FEATURES, 0x00u, 0, 0 };
		const uint16_t crc       = crc16_ccitt_false(lf_off, 3u);

		lf_off[3] = (uint8_t)(crc & 0xFFu);
		lf_off[4] = (uint8_t)(crc >> 8);

		spi_slave_cs_low();
		for (size_t i = 0; i < sizeof lf_off; i++) {
			spi_slave_rx_byte(lf_off[i]);
		}
		spi_slave_cs_high();
		while (spi_slave_tx_pending()) {
			(void)spi_slave_tx_next_byte();
		}
	}
}

/* §1's foundational CRC self-test, driven from the vector file instead of a
 * hand-copied literal -- closes the exact gap issue #31 finding E5's "Fix"
 * section names: "A host-compiled test that links src/protocol.c's own
 * crc16_ccitt_false and asserts it against the canonical value." */
ZTEST(protocol_vectors, test_crc16_matches_committed_self_test_vector)
{
	const pv_vector_t *ref_string = pv_find("crc16_ccitt_false_ref_string");
	const pv_vector_t *ref_result = pv_find("crc16_ccitt_false_ref_result");

	zassert_equal(ref_result->len, 2u, "crc16_ccitt_false_ref_result: 2 bytes");
	const uint16_t want = (uint16_t)((ref_result->bytes[0] << 8) | ref_result->bytes[1]);

	zassert_equal(crc16_ccitt_false(ref_string->bytes, ref_string->len),
	              want,
	              "CRC-16/CCITT-FALSE(\"123456789\") must match the committed vector");
}

ZTEST_SUITE(protocol_vectors, NULL, NULL, NULL, NULL, NULL);
