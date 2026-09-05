/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for protocol_dispatch() (gd32-bridge-firmware:src/protocol.c)
 * driven directly -- not through a transport -- against the TEST-ONLY fake
 * HAL in tests/unit/fake/bridge_hw_fake.{h,c}.
 *
 * WHY this suite exists: hal/bridge_hw_stub.c answers BRIDGE_HW_ERR_NOTIMPL
 * for essentially every call, so the transport_spi/transport_i2c suites
 * (which link the stub) can only ever reach STATUS_OK on the opcodes with no
 * hardware dependency (CMD_PING, CMD_GET_VERSION, CMD_GET_BUILD_ID,
 * CMD_LINK_FEATURES).  Every HAL-backed opcode -- the large majority of
 * protocol.c's 1025 lines -- goes untouched.  Linking the fake instead of the
 * stub lets each opcode reach STATUS_OK, and lets a test force any single
 * BRIDGE_HW_ERR_* return to check what STATUS_* the dispatcher maps it to.
 *
 * Four axes, table-driven, for every opcode protocol_dispatch() switches on:
 *   1. req_payload_len: the documented exact (or, for CMD_ADC_DSP_STAGE_PUSH,
 *      minimum) length, and length +/- 1 byte.
 *   2. reply_payload_cap: exactly the bytes STATUS_OK needs, and one byte
 *      short of that.
 *   3. the BRIDGE_HW_ERR_* -> STATUS_* mapping, injected through the fake,
 *      for every HAL-backed opcode.
 *   4. argument decode: for the opcodes with no reply payload (or whose
 *      reply is independent of the request, like TMU_COMPUTE's forced
 *      result), a post-dispatch check against the fake's captured-call
 *      accessors that the ARGUMENTS protocol.c handed the HAL match the
 *      request -- axes 1-3 alone only prove the STATUS code and length are
 *      right, not that the payload fields landed in the right place
 *      (a channel/reserved-byte mixup, a swapped pair of fields, an
 *      inverted polarity bit all pass axes 1-3 unnoticed).
 * Axis 1+2 (+ a same-request "does the reply reflect fake state" content
 * check where the reply is deterministic, PLUS axis 4's verify hook where
 * one exists) live in one generic table + loop, FIXED_CASES /
 * test_fixed_opcode_ok_and_boundaries.  Axis 3 lives in a second generic
 * table + loop, HAL_MAP_CASES / test_hal_error_mapping.
 * Opcodes whose request or reply length is a function of a payload FIELD
 * (CMD_ADC_READ, CMD_ADC_STREAM_READ, CMD_ADC_SPECTRUM_READ,
 * CMD_ADC_DSP_STAGE_PUSH) do not fit either generic shape and get their own
 * dedicated ZTEST cases below the two tables, alongside CMD_LINK_FEATURES
 * (stateful, but its output is a pure function of its own request -- see the
 * link_features paragraph below) and the opcodes with no dispatch-table entry
 * at all (unknown default, the 0x36 tombstone, the 0xF0..0xFF OTA range).
 *
 * Every ZTEST case calls bridge_hw_fake_reset() FIRST, manually: the ztest
 * shim (tests/unit/ztest_shim.h) runs every case in one process with no
 * before/after hook (ZTEST_SUITE's five hook slots are unused everywhere in
 * this tree), so a fake left dirty by one case is a real hazard for the
 * next one.  protocol.c's OWN `link_features[]` (src/protocol.c) is NOT
 * part of the fake and has no reset seam this suite can reach -- but it
 * does not need one here: no handler this file dispatches into ever READS
 * protocol_link_features() (only the SPI transport's reply-staging path
 * does, which this suite never calls), and handle_link_features's own
 * output depends only on the CURRENT request, not on whatever a previous
 * case left armed.  So CMD_LINK_FEATURES is safe to fold into the same
 * generic, order-independent table as everything else.
 *
 * Every dispatch below passes GD32_BRIDGE_LINK_SPI (#130): the link tag
 * is consulted by exactly one handler, and the SPI link is the one whose
 * armed features the accessor test then reads back.
 *
 * This suite links src/protocol.c, src/bootloader/bootloader.c, src/ota.c,
 * src/crc32.c and THIS suite's fake -- NOT hal/bridge_hw_stub.c (see the
 * test_protocol CMake target in tests/unit/CMakeLists.txt).  It does not
 * define -DBRIDGE_OTA_PARTITIONED (matching transport_spi/transport_i2c, not
 * the standalone `ota` suite), so src/ota.c's OTA_ST_* state machine never
 * compiles in and ota_dispatch() takes the `#else` branch at src/ota.c:483 --
 * unconditional STATUS_NOSUPPORT, no flash touched, for the WHOLE 0xF0..0xFF
 * range -- which is what test_ota_range_is_inert_without_partitioning below
 * pins.
 *
 * KNOWN OPEN DEFECTS -- verified present against this tree at commit a951585
 * (via `gh issue view`) before writing this file, each of them left
 * deliberately UNCOVERED rather than pinned either way (a case asserting the
 * documented/corrected mapping would be red today; a case asserting today's
 * mapping would cement the bug and fight the eventual fix).  #23's B2/B3/B4
 * are now FIXED (see below); B5 is still open.
 *
 *   - #23 -- several handlers flattened distinct HAL errors to STATUS_IO
 *     instead of running them through the central status_from_hw() mapper
 *     every other handler uses.  B2/B3/B4 are fixed as of this change; B5
 *     is a separate, still-open defect this suite continues to leave
 *     uncovered:
 *       * handle_adc_read (src/protocol.c) now routes its whole
 *         BRIDGE_HW_ERR_* -> STATUS_* translation through status_from_hw()
 *         (issue's B2, FIXED).  RANGE, BUSY, INVAL and the generic IO
 *         catch-all are all pinned below.
 *       * handle_pwm_set and handle_pwm_get (src/protocol.c) both now route
 *         through status_from_hw() too (issue's B3, FIXED), so the two
 *         handlers can no longer drift against each other on the shared
 *         PWM_CHANNEL_COUNT range check.  RANGE, BUSY, INVAL, NOTIMPL and
 *         IO are pinned for both.
 *       * handle_adc_stream_read (src/protocol.c) keeps its explicit
 *         NOTIMPL and BUSY rows (the BUSY row carries a load-bearing
 *         "poll faster" comment referencing docs/gd32-bridge-protocol.md
 *         §3.10) and now routes everything else -- including the
 *         previously-swallowed INVAL and RANGE -- through status_from_hw()
 *         (issue's B4, FIXED).  NOTIMPL, BUSY, INVAL and RANGE are all
 *         pinned below.
 *       * handle_pwm_capture_read (src/protocol.c:691) routes through
 *         status_from_hw() -- correct plumbing -- but the HAL contract it
 *         is fed (hal/bridge_hw.h:279-281, hal/gd32/pwm_capture.c:248)
 *         overloads BRIDGE_HW_ERR_NOTIMPL to also mean "capture ring empty,
 *         poll again", the same code every OTHER opcode uses for "this HAL
 *         body doesn't exist" (issue's B5, STILL OPEN).  The correct wire
 *         code is STATUS_NOT_READY (0x02): <alp/pwm.h>'s
 *         alp_pwm_capture_read() ALREADY documents "if no edge has been
 *         seen since the last call, returns ALP_ERR_NOT_READY", which is
 *         exactly the status_from_wire() mapping of wire STATUS_NOT_READY
 *         -- so the host-side contract this firmware must satisfy is
 *         already written down and is NOT STATUS_BUSY.  Fixing this needs
 *         a HAL-side change (hal/gd32/pwm_capture.c:248's ring-empty return
 *         must stop reusing BRIDGE_HW_ERR_NOTIMPL, which hal/bridge_hw_stub.c
 *         also returns for "this build has no capture HAL at all" -- the
 *         two meanings need two different BRIDGE_HW_ERR_* codes so a live,
 *         merely-empty channel stays distinguishable on the wire from a
 *         stub build) plus a hal/bridge_hw.h doc update, neither of which
 *         is in scope for this change (hal/gd32/pwm_capture.c is owned by
 *         another in-flight branch; hal/bridge_hw.h is owned by several).
 *         This suite pins pwm_capture_read's INVAL, RANGE, BUSY and IO rows
 *         (all unambiguous) and leaves its NOTIMPL row untested rather than
 *         assert today's NOTIMPL->STATUS_NOSUPPORT only for it to need to
 *         become something->STATUS_NOT_READY later.
 *
 *   - Unnumbered "stub/contract mismatch": CMD_GPIO_READ and CMD_GPIO_WRITE
 *     never special-case BRIDGE_HW_ERR_NOTIMPL (src/protocol.c:157, :175)
 *     the way every OTHER HAL-backed handler does -- out of scope for #23
 *     (the issue names four PWM/ADC handlers, not GPIO) and left unfixed
 *     here.  Against hal/bridge_hw_stub.c -- which answers NOTIMPL
 *     unconditionally for everything -- a real host talking to an
 *     unimplemented GD32 GPIO backend gets STATUS_IO ("retry the framing")
 *     forever instead of STATUS_NOSUPPORT ("this capability doesn't
 *     exist").  Their bare `if (rv < 0) return STATUS_IO;` (src/protocol.c:
 *     157, :175) is not NOTIMPL-specific either -- it flattens
 *     BRIDGE_HW_ERR_INVAL, _RANGE and _BUSY to STATUS_IO too, with no
 *     explicit branch for any of them.  This suite injects
 *     BRIDGE_HW_ERR_NOTIMPL nowhere for gpio_read or gpio_write; each gets
 *     only the unambiguous generic BRIDGE_HW_ERR_IO -> STATUS_IO row --
 *     GPIO's INVAL/RANGE/BUSY/NOTIMPL rows are left undeclared-by-omission,
 *     same as before this change.
 *
 *   - Unnumbered "flattened-to-IO" pair (no tracked issue; found while
 *     auditing the above): CMD_ADC_STREAM_END (src/protocol.c:583-584) and
 *     CMD_ADC_SPECTRUM_READ (src/protocol.c:545-547) each explicitly branch
 *     BRIDGE_HW_ERR_NOTIMPL (and, for spectrum, the "no frame yet" IO ->
 *     STATUS_BUSY special-case, which IS pinned below) but flatten
 *     BRIDGE_HW_ERR_INVAL, _RANGE and _BUSY to the generic
 *     `if (rv < 0) return STATUS_IO;` catch-all -- the same shape as #23's
 *     bucket above, just not yet filed against it.  NOTIMPL (+ spectrum's
 *     IO->BUSY) are pinned in test_hal_error_mapping; INVAL/RANGE/BUSY for
 *     both opcodes are not.
 *
 *   - CMD_RESET_REASON's destructive-read contract (hal/bridge_hw.h:60-62,
 *     hal/gd32/init.c:440, src/protocol.c:139-140) IS pinned below
 *     (test_reset_reason_destructive_read) against this fixture's model --
 *     see bridge_hw_fake.h/.c and gh#56, which proposes replacing this
 *     contract with an idempotent boot-snapshot instead.  #56 is open and
 *     unimplemented in this tree, so the destructive-read case reflects
 *     current, correct behaviour, not a bug being cemented; it must move in
 *     lockstep with #56 if that lands.
 *
 *   - #69 / #70 (both open) -- bridge_hw_adc_dsp_chain_bind's real (gd32
 *     backend) implementation accepts chain shapes the FAC/FFT runtime
 *     cannot realise (#69: a second FIR/IIR-terminal or multi-section chain;
 *     #70: a second FFT-terminal chain against the single-instance FFT
 *     block), silently returning BRIDGE_HW_OK where the runtime later
 *     starves.  Both defects live entirely in hal/gd32/adc_stream.c, which
 *     THIS suite never links (it links the fake instead) -- the fake's
 *     bridge_hw_adc_dsp_chain_bind is pure forced-return with no shape
 *     model at all (see bridge_hw_fake.h), so there is no bind-time
 *     validation logic here to exercise either correctly or incorrectly.
 *     What IS covered below (test_hal_error_mapping's ADC_DSP_CHAIN_BIND
 *     rows) is protocol.c's OWN mapping of whatever the HAL returns via the
 *     central status_from_hw() -- orthogonal to #69/#70, and not itself
 *     defective.  Closing #69/#70 will need a suite that links
 *     hal/gd32/adc_stream.c against a host-safe register model, which does
 *     not exist yet; until then this is a documented gap, not a covered one.
 *     CMD_ADC_DSP_STAGE_PUSH's own argument decode (chain_id, stage_index,
 *     kind, chunk_offset, chunk_total_size, chunk_data) IS now covered
 *     (test_adc_dsp_stage_push_length_boundary, via
 *     bridge_hw_fake_dsp_stage_push_get_last_call) -- plain decode
 *     coverage, still orthogonal to the #69/#70 shape gap above.
 *
 *   - Argument-decode coverage (see the axis note above) reaches every
 *     zero-reply FIXED_CASES row that has a fake accessor to check against:
 *     GPIO_WRITE, PWM_SET, PWM_CONFIGURE, ADC_CONFIGURE, SE_RESET,
 *     POWER_MODE_SET, DAC_SET (via a direct bridge_hw_dac_get() call),
 *     QENC_RESET (via a direct bridge_hw_qenc_read() call) and TMU_COMPUTE
 *     (non-zero reply, but its forced result made the inputs unverifiable
 *     without this).  It does NOT reach PWM_CAPTURE_BEGIN/END,
 *     PWM_SINGLE_PULSE, TIMER_SYNC, ADC_STREAM_BEGIN/END or
 *     ADC_DSP_CHAIN_BIND: the fake deliberately does not model a captured
 *     last-call for these (see bridge_hw_fake.h's per-function comments --
 *     "only the status-code seam"), so STATUS_OK + reply_len is the extent
 *     of what this suite proves for their arguments.  Adding capture state
 *     for these is future work, not done here.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "protocol.h"
#include "bootloader/bootloader.h" /* CMD_OTA_* */
#include "bridge_hw.h"
#include "bridge_hw_fake.h"

/* Shared reply scratch.  80 B covers every reply this suite ever asks for
 * (the largest is CMD_ADC_SPECTRUM_READ's 7 + 4*4 = 23 B). */
#define REPLY_SCRATCH_CAP 80u

static void put_le16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void put_le32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
	p[2] = (uint8_t)((v >> 16) & 0xFFu);
	p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

/* ------------------------------------------------------------------ */
/* Canonical (valid) request payloads, one per fixed-shape opcode.      */
/* Field values are chosen to (a) pass every LOCAL protocol.c value      */
/* check (duty <= period, enum ranges, stream_id < COUNT, ...) and       */
/* (b) where a "prep" seeds fake state, be distinctive enough that a     */
/* content mismatch could not be mistaken for a zero-initialised reply. */
/* ------------------------------------------------------------------ */

static const uint8_t req_gpio_read[4] = { 0xFF, 0xFF, 0x00, 0x00 }; /* mask=0x0000FFFF */
/* mask=0x0000FFFF, levels=0xFFFF1234 -- levels carries bits BOTH inside
 * and outside mask so a bug that ignores the mask, or a bug that swaps
 * which argument is mask vs levels, changes the verified result (see
 * prep_gpio_write() + verify_gpio_write() below, F1). */
static const uint8_t req_gpio_write[8] = { 0xFF, 0xFF, 0x00, 0x00, 0x34, 0x12, 0xFF, 0xFF };
static const uint8_t req_pwm_set[10]   = {
	0x02, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x90, 0xD0, 0x03, 0x00
};
static const uint8_t req_pwm_get[1]            = { 0x03 };
static const uint8_t req_pwm_configure[7]      = { 0x01, 0x02, 0xE8, 0x03, 0x00, 0x00, 0x01 };
static const uint8_t req_pwm_capture_begin[2]  = { 0x00, 0x01 };
static const uint8_t req_pwm_capture_read[1]   = { 0x01 };
static const uint8_t req_pwm_capture_end[1]    = { 0x00 };
static const uint8_t req_pwm_single_pulse[8]   = { 0x00, 0x00, 0x00, 0x00, 0x20, 0xA1, 0x07, 0x00 };
static const uint8_t req_timer_sync[3]         = { 0x00, 0x01, 0x01 };
static const uint8_t req_power_mode_set[10]    = { 0x01, 0x00, 0x01, 0x00, 0x00,
	                                               0x00, 0xE8, 0x03, 0x00, 0x00 };
static const uint8_t req_adc_configure[7]      = { 0x00, 0x00, 0x10, 0x00, 0x5C, 0x00, 0x0C };
static const uint8_t req_adc_stream_begin[7]   = { 0x00, 0x00, 0x00, 0xE8, 0x03, 0x00, 0x00 };
static const uint8_t req_adc_stream_end[1]     = { 0x01 };
static const uint8_t req_adc_dsp_chain_bind[2] = { 0x00, 0x01 };
static const uint8_t req_se_reset[1]           = { 0x01 };
static const uint8_t req_dac_set[4]            = { 0x00, 0x00, 0x08, 0x07 };
static const uint8_t req_dac_get[1]            = { 0x02 };
static const uint8_t req_qenc_read[1]          = { 0x01 };
static const uint8_t req_qenc_reset[1]         = { 0x01 };
static const uint8_t req_counter_read[1]       = { 0x00 };
static const uint8_t req_trng_read[1]          = { 0x08 };
static const uint8_t req_tmu_compute[12]       = { 0x00, 0x01, 0x00, 0x00, 0x00, 0x00,
	                                               0x80, 0x3F, 0x00, 0x00, 0x00, 0x00 };
static const uint8_t req_link_features[1]      = { 0xFF };

/* HAL-mapping-only requests: same shape rules, used only from
 * HAL_MAP_CASES below (their opcodes have field-dependent reply lengths
 * and get their own dedicated length-boundary cases further down). */
static const uint8_t req_adc_read[2]           = { 0x00, 0x04 };
static const uint8_t req_adc_stream_read[2]    = { 0x00, 0x04 };
static const uint8_t req_adc_spectrum_read[4]  = { 0x00, 0x00, 0x00, 0x04 };
static const uint8_t req_adc_dsp_stage_push[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0xAA };

/* ------------------------------------------------------------------ */
/* Expected reply content for the entries whose output is deterministic */
/* given the prep below (or, for GET_VERSION/GET_BUILD_ID, is a fixed   */
/* build-time constant).                                                */
/* ------------------------------------------------------------------ */

static const uint8_t expect_get_version[3] = { PROTOCOL_VERSION_MAJOR,
	                                           PROTOCOL_VERSION_MINOR,
	                                           PROTOCOL_VERSION_PATCH };
/* The 20-ASCII-byte placeholder GD32_BRIDGE_BUILD_ID falls back to when
 * GD32_BRIDGE_HAVE_BUILD_ID_HEADER is not defined (src/protocol.c:57-63) --
 * this suite's CMake target does not define it, matching transport_spi /
 * transport_i2c. */
static const uint8_t expect_get_build_id[20] = { '0', '0', '0', '0', '0', '0', '0', '0', '0', '0',
	                                             '0', '0', '0', '0', '0', '0', 'a', 'b', 'c', 'd' };
static const uint8_t expect_reset_reason[1]  = { 0x03 };
static const uint8_t expect_gpio_read[4]     = { 0xA5, 0xA5, 0x00, 0x00 }; /* pads & mask */
static const uint8_t expect_pwm_get[8]       = { 0x20, 0x4E, 0x00, 0x00, 0x88, 0x13, 0x00, 0x00 };
static const uint8_t expect_pwm_capture_read[8] = {
	0x40, 0x42, 0x0F, 0x00, 0x90, 0xD0, 0x03, 0x00
};
static const uint8_t expect_dac_get[2]        = { 0xD2, 0x04 };
static const uint8_t expect_qenc_read[4]      = { 0xD6, 0xFF, 0xFF, 0xFF };
static const uint8_t expect_counter_read[4]   = { 0xEF, 0xBE, 0xAD, 0xDE };
static const uint8_t expect_trng_read[8]      = { 0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17 };
static const uint8_t expect_tmu_compute[4]    = { 0x78, 0x56, 0x34, 0x12 };
static const uint8_t expect_da9292[1]         = { 0x01 };
static const uint8_t expect_dsp_chain_open[1] = { 0x07 };
static const uint8_t expect_link_features[1]  = { 0x01 }; /* 0xFF masked to the one known bit */

/* ------------------------------------------------------------------ */
/* prep hooks -- seed the fake's state model right before dispatch so   */
/* the OK-path reply reflects something other than a zeroed default.    */
/* ------------------------------------------------------------------ */

static void prep_reset_reason(void)
{
	bridge_hw_fake_set_reset_reason(0x03u);
}
static void prep_gpio_read(void)
{
	bridge_hw_fake_gpio_set_pads(0xA5A5A5A5u);
}
static void prep_pwm_get(void)
{
	bridge_hw_fake_pwm_seed(0x03u, 20000u, 5000u);
}
static void prep_pwm_capture_read(void)
{
	bridge_hw_fake_pwm_capture_seed(0x01u, 1000000u, 250000u);
}
static void prep_dac_get(void)
{
	bridge_hw_dac_set(0x02u, 1234u); /* the fake's own bridge_hw_dac_set, not a dispatch */
}
static void prep_qenc_read(void)
{
	bridge_hw_fake_qenc_set_position(0x01u, -42);
}
static void prep_counter_read(void)
{
	bridge_hw_fake_counter_set_ticks(0x00u, 0xDEADBEEFu);
}
static void prep_trng_read(void)
{
	bridge_hw_fake_trng_set_seed(0x10u);
}
static void prep_tmu_compute(void)
{
	bridge_hw_fake_tmu_set_result(0x12345678u);
}
static void prep_da9292(void)
{
	bridge_hw_fake_set_da9292_status(0x01u);
}
static void prep_dsp_chain_open(void)
{
	bridge_hw_fake_dsp_chain_set_next_id(0x07u);
}
static void prep_gpio_write(void)
{
	/* Nonzero, asymmetric starting pads (see req_gpio_write's comment)
	 * so verify_gpio_write() can distinguish "wrote levels&mask" from
	 * "ignored mask" or "swapped mask/levels". */
	bridge_hw_fake_gpio_set_pads(0xFFFF0000u);
}
static void prep_qenc_reset(void)
{
	/* Nonzero starting position at the SAME encoder req_qenc_reset
	 * resets, so verify_qenc_reset() proves the reset actually landed
	 * on encoder 1 rather than being a no-op or hitting the wrong slot. */
	bridge_hw_fake_qenc_set_position(0x01u, -77);
}

/* ------------------------------------------------------------------ */
/* verify hooks -- called after a zero-reply (or forced-reply) row's    */
/* canonical STATUS_OK dispatch to assert the ARGUMENTS protocol.c      */
/* handed the HAL, via the fake's captured-call accessors.  STATUS_OK   */
/* + reply_len alone cannot catch an argument-decode bug on these rows  */
/* (there is no reply payload to check, or the reply is independent of  */
/* the inputs) -- see this file's header, F1.                           */
/* ------------------------------------------------------------------ */

static void verify_gpio_write(void)
{
	zassert_equal(bridge_hw_fake_gpio_get_pads(),
	              0xFFFF1234u,
	              "GPIO_WRITE: pads reflect levels&mask over the prepped pads");
}
static void verify_pwm_set(void)
{
	uint32_t period_ns = 0u, duty_ns = 0u;
	bridge_hw_fake_pwm_get_state(0x02u, &period_ns, &duty_ns, NULL, NULL, NULL);
	zassert_equal(period_ns, 1000000u, "PWM_SET: period_ns lands on the requested channel");
	zassert_equal(duty_ns, 250000u, "PWM_SET: duty_ns lands on the requested channel");
}
static void verify_pwm_configure(void)
{
	uint8_t  align_mode = 0u, break_cfg = 0u;
	uint32_t dead_time_ns = 0u;
	bridge_hw_fake_pwm_get_state(0x01u, NULL, NULL, &align_mode, &dead_time_ns, &break_cfg);
	zassert_equal(align_mode, 2u, "PWM_CONFIGURE: align_mode");
	zassert_equal(dead_time_ns, 1000u, "PWM_CONFIGURE: dead_time_ns");
	zassert_equal(break_cfg, 1u, "PWM_CONFIGURE: break_cfg");
}
static void verify_adc_configure(void)
{
	uint8_t  channel = 0xFFu, resolution_bits = 0xFFu;
	uint16_t oversample_ratio = 0u, sample_cycles = 0u;
	bridge_hw_fake_adc_get_last_configure(
	    &channel, &oversample_ratio, &sample_cycles, &resolution_bits);
	zassert_equal(channel, 0u, "ADC_CONFIGURE: channel");
	zassert_equal(oversample_ratio, 16u, "ADC_CONFIGURE: oversample_ratio (not swapped)");
	zassert_equal(sample_cycles, 92u, "ADC_CONFIGURE: sample_cycles (not swapped)");
	zassert_equal(resolution_bits, 12u, "ADC_CONFIGURE: resolution_bits");
}
static void verify_se_reset(void)
{
	zassert_equal(bridge_hw_fake_se_reset_call_count(), 1u, "SE_RESET: called exactly once");
	zassert_equal(bridge_hw_fake_se_reset_last_assert(),
	              1u,
	              "SE_RESET: assert forwarded verbatim (not polarity-inverted)");
}
static void verify_power_mode_set(void)
{
	uint8_t  mode        = 0xFFu;
	uint32_t wake_bitmap = 0u, wake_after_ms = 0u;
	bridge_hw_fake_power_mode_get_last_call(&mode, &wake_bitmap, &wake_after_ms);
	zassert_equal(mode, 1u, "POWER_MODE_SET: mode");
	zassert_equal(wake_bitmap, 1u, "POWER_MODE_SET: wake_bitmap (not swapped)");
	zassert_equal(wake_after_ms, 1000u, "POWER_MODE_SET: wake_after_ms (not swapped)");
}
static void verify_tmu_compute(void)
{
	uint8_t  function = 0xFFu, format = 0xFFu;
	uint32_t in_a = 0u, in_b = 0xFFFFFFFFu;
	bridge_hw_fake_tmu_get_last_call(&function, &format, &in_a, &in_b);
	zassert_equal(function, 0u, "TMU_COMPUTE: function");
	zassert_equal(format, 1u, "TMU_COMPUTE: format");
	zassert_equal(in_a, 0x3F800000u, "TMU_COMPUTE: in_a (1.0f, Q31/IEEE754 bit pattern)");
	zassert_equal(in_b, 0u, "TMU_COMPUTE: in_b");
}
static void verify_dac_set(void)
{
	/* Direct call into the fake's own bridge_hw_dac_get -- the real HAL
	 * entry point (bridge_hw.h), not a test-only accessor -- exactly
	 * mirroring prep_dac_get()'s use of bridge_hw_dac_set() above. */
	uint16_t value_mv = 0xFFFFu;
	zassert_equal(bridge_hw_dac_get(0x00u, &value_mv), BRIDGE_HW_OK, "DAC_SET: readback succeeds");
	zassert_equal(value_mv, 1800u, "DAC_SET: value_mv (0x0708) landed on channel 0");
}
static void verify_qenc_reset(void)
{
	int32_t position = -1;
	zassert_equal(
	    bridge_hw_qenc_read(0x01u, &position), BRIDGE_HW_OK, "QENC_RESET: readback succeeds");
	zassert_equal(position, 0, "QENC_RESET: encoder 1's position is zeroed, not left at -77");
}

/* ------------------------------------------------------------------ */
/* Axis 1 + 2 (+ content): fixed request/reply shapes.                  */
/* ------------------------------------------------------------------ */

typedef struct {
	const char    *name;
	uint8_t        cmd;
	const uint8_t *req;        /* NULL iff req_len == 0 */
	size_t         req_len;    /* exact length protocol.c requires */
	size_t         reply_need; /* exact STATUS_OK reply length (0 = none) */
	void (*prep)(void);        /* optional fake-state seed; NULL = none */
	const uint8_t *expect;     /* optional expected reply bytes; NULL = skip */
	void (*verify)(void);      /* optional post-dispatch argument-decode
	                            * assertion via the fake's captured-call
	                            * accessors; NULL = no seam exists yet for
	                            * this opcode (see this file's header, F1) */
} fixed_case_t;

/* clang-format off */
static const fixed_case_t FIXED_CASES[] = {
	{ "PING",              CMD_PING,              NULL,                 0u, 0u, NULL,                 NULL,                 NULL },
	{ "GET_VERSION",       CMD_GET_VERSION,       NULL,                 0u, 3u, NULL,                 expect_get_version,   NULL },
	{ "GET_BUILD_ID",      CMD_GET_BUILD_ID,      NULL,                 0u, 20u, NULL,                expect_get_build_id, NULL },
	{ "RESET_REASON",      CMD_RESET_REASON,      NULL,                 0u, 1u, prep_reset_reason,    expect_reset_reason, NULL },
	{ "GPIO_READ",         CMD_GPIO_READ,         req_gpio_read,        4u, 4u, prep_gpio_read,       expect_gpio_read,    NULL },
	{ "GPIO_WRITE",        CMD_GPIO_WRITE,        req_gpio_write,       8u, 0u, prep_gpio_write,      NULL,                 verify_gpio_write },
	{ "PWM_SET",           CMD_PWM_SET,           req_pwm_set,          10u, 0u, NULL,                NULL,                 verify_pwm_set },
	{ "PWM_GET",           CMD_PWM_GET,           req_pwm_get,          1u, 8u, prep_pwm_get,         expect_pwm_get,       NULL },
	{ "PWM_CONFIGURE",     CMD_PWM_CONFIGURE,     req_pwm_configure,    7u, 0u, NULL,                 NULL,                 verify_pwm_configure },
	{ "PWM_CAPTURE_BEGIN", CMD_PWM_CAPTURE_BEGIN, req_pwm_capture_begin, 2u, 0u, NULL,                NULL,                 NULL },
	{ "PWM_CAPTURE_READ",  CMD_PWM_CAPTURE_READ,  req_pwm_capture_read, 1u, 8u, prep_pwm_capture_read, expect_pwm_capture_read, NULL },
	{ "PWM_CAPTURE_END",   CMD_PWM_CAPTURE_END,   req_pwm_capture_end,  1u, 0u, NULL,                 NULL,                 NULL },
	{ "PWM_SINGLE_PULSE",  CMD_PWM_SINGLE_PULSE,  req_pwm_single_pulse, 8u, 0u, NULL,                 NULL,                 NULL },
	{ "TIMER_SYNC",        CMD_TIMER_SYNC,        req_timer_sync,       3u, 0u, NULL,                 NULL,                 NULL },
	{ "POWER_MODE_SET",    CMD_POWER_MODE_SET,    req_power_mode_set,   10u, 0u, NULL,                NULL,                 verify_power_mode_set },
	{ "ADC_CONFIGURE",     CMD_ADC_CONFIGURE,     req_adc_configure,    7u, 0u, NULL,                 NULL,                 verify_adc_configure },
	{ "ADC_STREAM_BEGIN",  CMD_ADC_STREAM_BEGIN,  req_adc_stream_begin, 7u, 0u, NULL,                 NULL,                 NULL },
	{ "ADC_STREAM_END",    CMD_ADC_STREAM_END,    req_adc_stream_end,   1u, 0u, NULL,                 NULL,                 NULL },
	{ "ADC_DSP_CHAIN_OPEN", CMD_ADC_DSP_CHAIN_OPEN, NULL,                0u, 1u, prep_dsp_chain_open, expect_dsp_chain_open, NULL },
	{ "ADC_DSP_CHAIN_BIND", CMD_ADC_DSP_CHAIN_BIND, req_adc_dsp_chain_bind, 2u, 0u, NULL,             NULL,                 NULL },
	{ "DA9292_STATUS_FORWARD", CMD_DA9292_STATUS_FORWARD, NULL,          0u, 1u, prep_da9292,         expect_da9292,        NULL },
	{ "SE_RESET",          CMD_SE_RESET,          req_se_reset,         1u, 0u, NULL,                 NULL,                 verify_se_reset },
	{ "DAC_SET",           CMD_DAC_SET,           req_dac_set,          4u, 0u, NULL,                 NULL,                 verify_dac_set },
	{ "DAC_GET",           CMD_DAC_GET,           req_dac_get,          1u, 2u, prep_dac_get,         expect_dac_get,       NULL },
	{ "QENC_READ",         CMD_QENC_READ,         req_qenc_read,        1u, 4u, prep_qenc_read,       expect_qenc_read,     NULL },
	{ "QENC_RESET",        CMD_QENC_RESET,        req_qenc_reset,       1u, 0u, prep_qenc_reset,      NULL,                 verify_qenc_reset },
	{ "COUNTER_READ",      CMD_COUNTER_READ,      req_counter_read,     1u, 4u, prep_counter_read,    expect_counter_read,  NULL },
	{ "TRNG_READ",         CMD_TRNG_READ,         req_trng_read,        1u, 8u, prep_trng_read,       expect_trng_read,     NULL },
	{ "TMU_COMPUTE",       CMD_TMU_COMPUTE,       req_tmu_compute,      12u, 4u, prep_tmu_compute,    expect_tmu_compute,   verify_tmu_compute },
	{ "LINK_FEATURES",     CMD_LINK_FEATURES,     req_link_features,    1u, 1u, NULL,                 expect_link_features, NULL },
};
/* clang-format on */

#define N_FIXED_CASES (sizeof(FIXED_CASES) / sizeof(FIXED_CASES[0]))

ZTEST(protocol, test_fixed_opcode_ok_and_boundaries)
{
	uint8_t reply[REPLY_SCRATCH_CAP];
	uint8_t padded[16]; /* >= longest FIXED_CASES req_len (12) + 1 */

	for (size_t i = 0; i < N_FIXED_CASES; i++) {
		const fixed_case_t *c = &FIXED_CASES[i];

		/* --- canonical request: STATUS_OK, exact reply length,
		 * and (where deterministic) exact reply content. --- */
		bridge_hw_fake_reset();
		if (c->prep) c->prep();
		size_t               reply_len = 0xDEADu; /* poisoned; must become c->reply_need --
		                            * for the reply_need==0 rows this is the
		                            * ONLY thing that can catch a handler
		                            * that never writes *reply_len (see this
		                            * file's header, F1/F3). */
		gd32_bridge_status_t st        = protocol_dispatch(
		    GD32_BRIDGE_LINK_SPI, c->cmd, c->req, c->req_len, reply, c->reply_need, &reply_len);
		zassert_equal(st, STATUS_OK, "%s: canonical request", c->name);
		zassert_equal(reply_len, c->reply_need, "%s: reply length", c->name);
		if (c->expect) {
			zassert_mem_equal(reply, c->expect, c->reply_need, "%s: reply content", c->name);
		}
		if (c->verify) c->verify();

		/* --- req_payload_len - 1: STATUS_INVAL. --- */
		if (c->req_len > 0u) {
			bridge_hw_fake_reset();
			if (c->prep) c->prep();
			reply_len = 0u;
			st        = protocol_dispatch(GD32_BRIDGE_LINK_SPI,
			                              c->cmd,
			                              c->req,
			                              c->req_len - 1u,
			                              reply,
			                              REPLY_SCRATCH_CAP,
			                              &reply_len);
			zassert_equal(st, STATUS_INVAL, "%s: req_len-1", c->name);
		}

		/* --- req_payload_len + 1: STATUS_INVAL. --- */
		bridge_hw_fake_reset();
		if (c->prep) c->prep();
		if (c->req_len > 0u) {
			memcpy(padded, c->req, c->req_len);
		}
		padded[c->req_len] = 0u; /* the stray extra byte; value is irrelevant */
		reply_len          = 0u;
		st                 = protocol_dispatch(GD32_BRIDGE_LINK_SPI,
		                                       c->cmd,
		                                       padded,
		                                       c->req_len + 1u,
		                                       reply,
		                                       REPLY_SCRATCH_CAP,
		                                       &reply_len);
		zassert_equal(st, STATUS_INVAL, "%s: req_len+1", c->name);

		/* --- reply_payload_cap one below the STATUS_OK need:
		 * STATUS_NOMEM.  Only meaningful when a reply is produced
		 * at all -- `reply_cap < 0` can never hold for size_t. --- */
		if (c->reply_need > 0u) {
			bridge_hw_fake_reset();
			if (c->prep) c->prep();
			reply_len = 0u;
			st        = protocol_dispatch(GD32_BRIDGE_LINK_SPI,
			                              c->cmd,
			                              c->req,
			                              c->req_len,
			                              reply,
			                              c->reply_need - 1u,
			                              &reply_len);
			zassert_equal(st, STATUS_NOMEM, "%s: reply_cap-1", c->name);
		}
	}
}

/* ------------------------------------------------------------------ */
/* Axis 3: injected BRIDGE_HW_ERR_* -> expected STATUS_* mapping.       */
/* Every row's request already satisfies its opcode's LOCAL protocol.c  */
/* value checks (reusing FIXED_CASES' canonical requests where the      */
/* opcode has one), so the ONLY thing under test is the mapping past    */
/* the HAL call.  Skipped combinations are the KNOWN OPEN DEFECTS       */
/* enumerated in this file's header comment.                            */
/* ------------------------------------------------------------------ */

typedef struct {
	const char          *name;
	uint8_t              cmd;
	const uint8_t       *req;
	size_t               req_len;
	bridge_hw_fake_fn_t  fn;
	int                  inject;
	gd32_bridge_status_t want;
} hal_map_case_t;

/* clang-format off */
static const hal_map_case_t HAL_MAP_CASES[] = {
	/* --- "stub/contract mismatch" bucket: only the unambiguous IO row. */
	{ "GPIO_READ/IO",  CMD_GPIO_READ,  req_gpio_read,  4u, FAKE_FN_GPIO_READ,  BRIDGE_HW_ERR_IO, STATUS_IO },
	{ "GPIO_WRITE/IO", CMD_GPIO_WRITE, req_gpio_write, 8u, FAKE_FN_GPIO_WRITE, BRIDGE_HW_ERR_IO, STATUS_IO },

	/* --- PWM_SET / PWM_GET: both now route through status_from_hw()
	 * (#23 B3, FIXED) so they can no longer disagree on the shared
	 * PWM_CHANNEL_COUNT range check -- full INVAL/RANGE/NOTIMPL/BUSY/IO
	 * coverage for both. */
	{ "PWM_SET/RANGE",   CMD_PWM_SET, req_pwm_set, 10u, FAKE_FN_PWM_SET, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "PWM_SET/INVAL",   CMD_PWM_SET, req_pwm_set, 10u, FAKE_FN_PWM_SET, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "PWM_SET/NOTIMPL", CMD_PWM_SET, req_pwm_set, 10u, FAKE_FN_PWM_SET, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "PWM_SET/BUSY",    CMD_PWM_SET, req_pwm_set, 10u, FAKE_FN_PWM_SET, BRIDGE_HW_ERR_BUSY,    STATUS_BUSY },
	{ "PWM_SET/IO",      CMD_PWM_SET, req_pwm_set, 10u, FAKE_FN_PWM_SET, BRIDGE_HW_ERR_IO,      STATUS_IO },
	{ "PWM_GET/RANGE",   CMD_PWM_GET, req_pwm_get, 1u, FAKE_FN_PWM_GET, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "PWM_GET/INVAL",   CMD_PWM_GET, req_pwm_get, 1u, FAKE_FN_PWM_GET, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "PWM_GET/NOTIMPL", CMD_PWM_GET, req_pwm_get, 1u, FAKE_FN_PWM_GET, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "PWM_GET/BUSY",    CMD_PWM_GET, req_pwm_get, 1u, FAKE_FN_PWM_GET, BRIDGE_HW_ERR_BUSY,    STATUS_BUSY },
	{ "PWM_GET/IO",      CMD_PWM_GET, req_pwm_get, 1u, FAKE_FN_PWM_GET, BRIDGE_HW_ERR_IO,      STATUS_IO },

	/* --- ADC_READ: now routes through status_from_hw() (#23 B2, FIXED)
	 * -- full INVAL/RANGE/NOTIMPL/BUSY/IO coverage. */
	{ "ADC_READ/RANGE",   CMD_ADC_READ, req_adc_read, 2u, FAKE_FN_ADC_READ, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "ADC_READ/INVAL",   CMD_ADC_READ, req_adc_read, 2u, FAKE_FN_ADC_READ, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "ADC_READ/NOTIMPL", CMD_ADC_READ, req_adc_read, 2u, FAKE_FN_ADC_READ, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "ADC_READ/BUSY",    CMD_ADC_READ, req_adc_read, 2u, FAKE_FN_ADC_READ, BRIDGE_HW_ERR_BUSY,    STATUS_BUSY },
	{ "ADC_READ/IO",      CMD_ADC_READ, req_adc_read, 2u, FAKE_FN_ADC_READ, BRIDGE_HW_ERR_IO,      STATUS_IO },

	/* --- Full INVAL/RANGE/NOTIMPL triple: correctly and explicitly
	 * branched, no known defect. */
	{ "PWM_CONFIGURE/INVAL",   CMD_PWM_CONFIGURE,   req_pwm_configure,   7u, FAKE_FN_PWM_CONFIGURE,   BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "PWM_CONFIGURE/RANGE",   CMD_PWM_CONFIGURE,   req_pwm_configure,   7u, FAKE_FN_PWM_CONFIGURE,   BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "PWM_CONFIGURE/NOTIMPL", CMD_PWM_CONFIGURE,   req_pwm_configure,   7u, FAKE_FN_PWM_CONFIGURE,   BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "ADC_CONFIGURE/INVAL",   CMD_ADC_CONFIGURE,   req_adc_configure,   7u, FAKE_FN_ADC_CONFIGURE,   BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "ADC_CONFIGURE/RANGE",   CMD_ADC_CONFIGURE,   req_adc_configure,   7u, FAKE_FN_ADC_CONFIGURE,   BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "ADC_CONFIGURE/NOTIMPL", CMD_ADC_CONFIGURE,   req_adc_configure,   7u, FAKE_FN_ADC_CONFIGURE,   BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "ADC_STREAM_BEGIN/INVAL",   CMD_ADC_STREAM_BEGIN, req_adc_stream_begin, 7u, FAKE_FN_ADC_STREAM_BEGIN, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "ADC_STREAM_BEGIN/RANGE",   CMD_ADC_STREAM_BEGIN, req_adc_stream_begin, 7u, FAKE_FN_ADC_STREAM_BEGIN, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "ADC_STREAM_BEGIN/NOTIMPL", CMD_ADC_STREAM_BEGIN, req_adc_stream_begin, 7u, FAKE_FN_ADC_STREAM_BEGIN, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "DAC_SET/INVAL",   CMD_DAC_SET, req_dac_set, 4u, FAKE_FN_DAC_SET, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "DAC_SET/RANGE",   CMD_DAC_SET, req_dac_set, 4u, FAKE_FN_DAC_SET, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "DAC_SET/NOTIMPL", CMD_DAC_SET, req_dac_set, 4u, FAKE_FN_DAC_SET, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "TMU_COMPUTE/INVAL",   CMD_TMU_COMPUTE, req_tmu_compute, 12u, FAKE_FN_TMU_COMPUTE, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "TMU_COMPUTE/RANGE",   CMD_TMU_COMPUTE, req_tmu_compute, 12u, FAKE_FN_TMU_COMPUTE, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "TMU_COMPUTE/NOTIMPL", CMD_TMU_COMPUTE, req_tmu_compute, 12u, FAKE_FN_TMU_COMPUTE, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },

	/* --- INVAL/NOTIMPL pair + generic IO catch-all (no RANGE branch
	 * in these handlers; not established as a defect, simply not
	 * part of this handler's explicit ladder). */
	{ "DAC_GET/INVAL",   CMD_DAC_GET, req_dac_get, 1u, FAKE_FN_DAC_GET, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "DAC_GET/NOTIMPL", CMD_DAC_GET, req_dac_get, 1u, FAKE_FN_DAC_GET, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "DAC_GET/IO",      CMD_DAC_GET, req_dac_get, 1u, FAKE_FN_DAC_GET, BRIDGE_HW_ERR_IO,      STATUS_IO },
	{ "QENC_READ/INVAL",   CMD_QENC_READ, req_qenc_read, 1u, FAKE_FN_QENC_READ, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "QENC_READ/NOTIMPL", CMD_QENC_READ, req_qenc_read, 1u, FAKE_FN_QENC_READ, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "QENC_READ/IO",      CMD_QENC_READ, req_qenc_read, 1u, FAKE_FN_QENC_READ, BRIDGE_HW_ERR_IO,      STATUS_IO },
	{ "QENC_RESET/INVAL",   CMD_QENC_RESET, req_qenc_reset, 1u, FAKE_FN_QENC_RESET, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "QENC_RESET/NOTIMPL", CMD_QENC_RESET, req_qenc_reset, 1u, FAKE_FN_QENC_RESET, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "QENC_RESET/IO",      CMD_QENC_RESET, req_qenc_reset, 1u, FAKE_FN_QENC_RESET, BRIDGE_HW_ERR_IO,      STATUS_IO },
	{ "COUNTER_READ/INVAL",   CMD_COUNTER_READ, req_counter_read, 1u, FAKE_FN_COUNTER_READ, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "COUNTER_READ/NOTIMPL", CMD_COUNTER_READ, req_counter_read, 1u, FAKE_FN_COUNTER_READ, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "COUNTER_READ/IO",      CMD_COUNTER_READ, req_counter_read, 1u, FAKE_FN_COUNTER_READ, BRIDGE_HW_ERR_IO,      STATUS_IO },
	{ "SE_RESET/INVAL",   CMD_SE_RESET, req_se_reset, 1u, FAKE_FN_SE_RESET, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "SE_RESET/NOTIMPL", CMD_SE_RESET, req_se_reset, 1u, FAKE_FN_SE_RESET, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "SE_RESET/IO",      CMD_SE_RESET, req_se_reset, 1u, FAKE_FN_SE_RESET, BRIDGE_HW_ERR_IO,      STATUS_IO },
	{ "ADC_STREAM_END/NOTIMPL", CMD_ADC_STREAM_END, req_adc_stream_end, 1u, FAKE_FN_ADC_STREAM_END, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "ADC_STREAM_END/IO",      CMD_ADC_STREAM_END, req_adc_stream_end, 1u, FAKE_FN_ADC_STREAM_END, BRIDGE_HW_ERR_IO,      STATUS_IO },

	/* --- Centralised status_from_hw() mapper: full, correct coverage
	 * (INVAL/RANGE/NOTIMPL/BUSY/IO) -- no known defect. */
	{ "PWM_CAPTURE_BEGIN/INVAL",   CMD_PWM_CAPTURE_BEGIN, req_pwm_capture_begin, 2u, FAKE_FN_PWM_CAPTURE_BEGIN, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "PWM_CAPTURE_BEGIN/RANGE",   CMD_PWM_CAPTURE_BEGIN, req_pwm_capture_begin, 2u, FAKE_FN_PWM_CAPTURE_BEGIN, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "PWM_CAPTURE_BEGIN/NOTIMPL", CMD_PWM_CAPTURE_BEGIN, req_pwm_capture_begin, 2u, FAKE_FN_PWM_CAPTURE_BEGIN, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "PWM_CAPTURE_BEGIN/BUSY",    CMD_PWM_CAPTURE_BEGIN, req_pwm_capture_begin, 2u, FAKE_FN_PWM_CAPTURE_BEGIN, BRIDGE_HW_ERR_BUSY,    STATUS_BUSY },
	{ "PWM_CAPTURE_BEGIN/IO",      CMD_PWM_CAPTURE_BEGIN, req_pwm_capture_begin, 2u, FAKE_FN_PWM_CAPTURE_BEGIN, BRIDGE_HW_ERR_IO,      STATUS_IO },
	/* PWM_CAPTURE_READ: NOTIMPL is the #23 B5 ring-empty overload --
	 * not injected here (see this file's header comment). */
	{ "PWM_CAPTURE_READ/INVAL", CMD_PWM_CAPTURE_READ, req_pwm_capture_read, 1u, FAKE_FN_PWM_CAPTURE_READ, BRIDGE_HW_ERR_INVAL, STATUS_INVAL },
	{ "PWM_CAPTURE_READ/RANGE", CMD_PWM_CAPTURE_READ, req_pwm_capture_read, 1u, FAKE_FN_PWM_CAPTURE_READ, BRIDGE_HW_ERR_RANGE, STATUS_OUT_OF_RANGE },
	{ "PWM_CAPTURE_READ/BUSY",  CMD_PWM_CAPTURE_READ, req_pwm_capture_read, 1u, FAKE_FN_PWM_CAPTURE_READ, BRIDGE_HW_ERR_BUSY,  STATUS_BUSY },
	{ "PWM_CAPTURE_READ/IO",    CMD_PWM_CAPTURE_READ, req_pwm_capture_read, 1u, FAKE_FN_PWM_CAPTURE_READ, BRIDGE_HW_ERR_IO,    STATUS_IO },
	{ "PWM_CAPTURE_END/INVAL",   CMD_PWM_CAPTURE_END, req_pwm_capture_end, 1u, FAKE_FN_PWM_CAPTURE_END, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "PWM_CAPTURE_END/RANGE",   CMD_PWM_CAPTURE_END, req_pwm_capture_end, 1u, FAKE_FN_PWM_CAPTURE_END, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "PWM_CAPTURE_END/NOTIMPL", CMD_PWM_CAPTURE_END, req_pwm_capture_end, 1u, FAKE_FN_PWM_CAPTURE_END, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "PWM_CAPTURE_END/BUSY",    CMD_PWM_CAPTURE_END, req_pwm_capture_end, 1u, FAKE_FN_PWM_CAPTURE_END, BRIDGE_HW_ERR_BUSY,    STATUS_BUSY },
	{ "PWM_CAPTURE_END/IO",      CMD_PWM_CAPTURE_END, req_pwm_capture_end, 1u, FAKE_FN_PWM_CAPTURE_END, BRIDGE_HW_ERR_IO,      STATUS_IO },
	{ "PWM_SINGLE_PULSE/INVAL",   CMD_PWM_SINGLE_PULSE, req_pwm_single_pulse, 8u, FAKE_FN_PWM_SINGLE_PULSE, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "PWM_SINGLE_PULSE/RANGE",   CMD_PWM_SINGLE_PULSE, req_pwm_single_pulse, 8u, FAKE_FN_PWM_SINGLE_PULSE, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "PWM_SINGLE_PULSE/NOTIMPL", CMD_PWM_SINGLE_PULSE, req_pwm_single_pulse, 8u, FAKE_FN_PWM_SINGLE_PULSE, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "PWM_SINGLE_PULSE/BUSY",    CMD_PWM_SINGLE_PULSE, req_pwm_single_pulse, 8u, FAKE_FN_PWM_SINGLE_PULSE, BRIDGE_HW_ERR_BUSY,    STATUS_BUSY },
	{ "PWM_SINGLE_PULSE/IO",      CMD_PWM_SINGLE_PULSE, req_pwm_single_pulse, 8u, FAKE_FN_PWM_SINGLE_PULSE, BRIDGE_HW_ERR_IO,      STATUS_IO },
	{ "TIMER_SYNC/INVAL",   CMD_TIMER_SYNC, req_timer_sync, 3u, FAKE_FN_TIMER_SYNC, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "TIMER_SYNC/RANGE",   CMD_TIMER_SYNC, req_timer_sync, 3u, FAKE_FN_TIMER_SYNC, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "TIMER_SYNC/NOTIMPL", CMD_TIMER_SYNC, req_timer_sync, 3u, FAKE_FN_TIMER_SYNC, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "TIMER_SYNC/BUSY",    CMD_TIMER_SYNC, req_timer_sync, 3u, FAKE_FN_TIMER_SYNC, BRIDGE_HW_ERR_BUSY,    STATUS_BUSY },
	{ "TIMER_SYNC/IO",      CMD_TIMER_SYNC, req_timer_sync, 3u, FAKE_FN_TIMER_SYNC, BRIDGE_HW_ERR_IO,      STATUS_IO },
	{ "POWER_MODE_SET/INVAL",   CMD_POWER_MODE_SET, req_power_mode_set, 10u, FAKE_FN_POWER_MODE_SET, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "POWER_MODE_SET/RANGE",   CMD_POWER_MODE_SET, req_power_mode_set, 10u, FAKE_FN_POWER_MODE_SET, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "POWER_MODE_SET/NOTIMPL", CMD_POWER_MODE_SET, req_power_mode_set, 10u, FAKE_FN_POWER_MODE_SET, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "POWER_MODE_SET/BUSY",    CMD_POWER_MODE_SET, req_power_mode_set, 10u, FAKE_FN_POWER_MODE_SET, BRIDGE_HW_ERR_BUSY,    STATUS_BUSY },
	{ "POWER_MODE_SET/IO",      CMD_POWER_MODE_SET, req_power_mode_set, 10u, FAKE_FN_POWER_MODE_SET, BRIDGE_HW_ERR_IO,      STATUS_IO },
	{ "ADC_DSP_CHAIN_OPEN/INVAL",   CMD_ADC_DSP_CHAIN_OPEN, NULL, 0u, FAKE_FN_ADC_DSP_CHAIN_OPEN, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "ADC_DSP_CHAIN_OPEN/RANGE",   CMD_ADC_DSP_CHAIN_OPEN, NULL, 0u, FAKE_FN_ADC_DSP_CHAIN_OPEN, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "ADC_DSP_CHAIN_OPEN/NOTIMPL", CMD_ADC_DSP_CHAIN_OPEN, NULL, 0u, FAKE_FN_ADC_DSP_CHAIN_OPEN, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "ADC_DSP_CHAIN_OPEN/BUSY",    CMD_ADC_DSP_CHAIN_OPEN, NULL, 0u, FAKE_FN_ADC_DSP_CHAIN_OPEN, BRIDGE_HW_ERR_BUSY,    STATUS_BUSY },
	{ "ADC_DSP_CHAIN_OPEN/IO",      CMD_ADC_DSP_CHAIN_OPEN, NULL, 0u, FAKE_FN_ADC_DSP_CHAIN_OPEN, BRIDGE_HW_ERR_IO,      STATUS_IO },
	/* ADC_DSP_CHAIN_BIND: this is protocol.c's OWN mapping, orthogonal
	 * to the #69/#70 bind-time SHAPE validation gap -- see this file's
	 * header comment. */
	{ "ADC_DSP_CHAIN_BIND/INVAL",   CMD_ADC_DSP_CHAIN_BIND, req_adc_dsp_chain_bind, 2u, FAKE_FN_ADC_DSP_CHAIN_BIND, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "ADC_DSP_CHAIN_BIND/RANGE",   CMD_ADC_DSP_CHAIN_BIND, req_adc_dsp_chain_bind, 2u, FAKE_FN_ADC_DSP_CHAIN_BIND, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "ADC_DSP_CHAIN_BIND/NOTIMPL", CMD_ADC_DSP_CHAIN_BIND, req_adc_dsp_chain_bind, 2u, FAKE_FN_ADC_DSP_CHAIN_BIND, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "ADC_DSP_CHAIN_BIND/BUSY",    CMD_ADC_DSP_CHAIN_BIND, req_adc_dsp_chain_bind, 2u, FAKE_FN_ADC_DSP_CHAIN_BIND, BRIDGE_HW_ERR_BUSY,    STATUS_BUSY },
	{ "ADC_DSP_CHAIN_BIND/IO",      CMD_ADC_DSP_CHAIN_BIND, req_adc_dsp_chain_bind, 2u, FAKE_FN_ADC_DSP_CHAIN_BIND, BRIDGE_HW_ERR_IO,      STATUS_IO },
	{ "TRNG_READ/INVAL",   CMD_TRNG_READ, req_trng_read, 1u, FAKE_FN_TRNG_READ, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "TRNG_READ/RANGE",   CMD_TRNG_READ, req_trng_read, 1u, FAKE_FN_TRNG_READ, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "TRNG_READ/NOTIMPL", CMD_TRNG_READ, req_trng_read, 1u, FAKE_FN_TRNG_READ, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "TRNG_READ/BUSY",    CMD_TRNG_READ, req_trng_read, 1u, FAKE_FN_TRNG_READ, BRIDGE_HW_ERR_BUSY,    STATUS_BUSY },
	{ "TRNG_READ/IO",      CMD_TRNG_READ, req_trng_read, 1u, FAKE_FN_TRNG_READ, BRIDGE_HW_ERR_IO,      STATUS_IO },
	{ "ADC_DSP_STAGE_PUSH/INVAL",   CMD_ADC_DSP_STAGE_PUSH, req_adc_dsp_stage_push, 8u, FAKE_FN_ADC_DSP_STAGE_PUSH, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "ADC_DSP_STAGE_PUSH/RANGE",   CMD_ADC_DSP_STAGE_PUSH, req_adc_dsp_stage_push, 8u, FAKE_FN_ADC_DSP_STAGE_PUSH, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },
	{ "ADC_DSP_STAGE_PUSH/NOTIMPL", CMD_ADC_DSP_STAGE_PUSH, req_adc_dsp_stage_push, 8u, FAKE_FN_ADC_DSP_STAGE_PUSH, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "ADC_DSP_STAGE_PUSH/BUSY",    CMD_ADC_DSP_STAGE_PUSH, req_adc_dsp_stage_push, 8u, FAKE_FN_ADC_DSP_STAGE_PUSH, BRIDGE_HW_ERR_BUSY,    STATUS_BUSY },
	{ "ADC_DSP_STAGE_PUSH/IO",      CMD_ADC_DSP_STAGE_PUSH, req_adc_dsp_stage_push, 8u, FAKE_FN_ADC_DSP_STAGE_PUSH, BRIDGE_HW_ERR_IO,      STATUS_IO },

	/* --- ADC_STREAM_READ: NOTIMPL/BUSY keep their explicit rows (BUSY's
	 * carries the "ring overrun -- poll faster" contract, docs
	 * §3.10); INVAL and RANGE, previously swallowed to STATUS_IO by the
	 * `if (rv < 0) return STATUS_IO;` catch-all, now route through
	 * status_from_hw() (#23 B4, FIXED). */
	{ "ADC_STREAM_READ/NOTIMPL", CMD_ADC_STREAM_READ, req_adc_stream_read, 2u, FAKE_FN_ADC_STREAM_READ, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "ADC_STREAM_READ/BUSY",    CMD_ADC_STREAM_READ, req_adc_stream_read, 2u, FAKE_FN_ADC_STREAM_READ, BRIDGE_HW_ERR_BUSY,    STATUS_BUSY },
	{ "ADC_STREAM_READ/INVAL",   CMD_ADC_STREAM_READ, req_adc_stream_read, 2u, FAKE_FN_ADC_STREAM_READ, BRIDGE_HW_ERR_INVAL,   STATUS_INVAL },
	{ "ADC_STREAM_READ/RANGE",   CMD_ADC_STREAM_READ, req_adc_stream_read, 2u, FAKE_FN_ADC_STREAM_READ, BRIDGE_HW_ERR_RANGE,   STATUS_OUT_OF_RANGE },

	/* --- ADC_SPECTRUM_READ: NOTIMPL explicit; IO is the documented
	 * "no frame yet" -> STATUS_BUSY special-case (src/protocol.c:546),
	 * distinct from every other opcode's IO handling -- worth pinning
	 * on its own. */
	{ "ADC_SPECTRUM_READ/NOTIMPL", CMD_ADC_SPECTRUM_READ, req_adc_spectrum_read, 4u, FAKE_FN_ADC_SPECTRUM_READ, BRIDGE_HW_ERR_NOTIMPL, STATUS_NOSUPPORT },
	{ "ADC_SPECTRUM_READ/IO_is_BUSY", CMD_ADC_SPECTRUM_READ, req_adc_spectrum_read, 4u, FAKE_FN_ADC_SPECTRUM_READ, BRIDGE_HW_ERR_IO, STATUS_BUSY },
};
/* clang-format on */

#define N_HAL_MAP_CASES (sizeof(HAL_MAP_CASES) / sizeof(HAL_MAP_CASES[0]))

ZTEST(protocol, test_hal_error_mapping)
{
	uint8_t reply[REPLY_SCRATCH_CAP];

	for (size_t i = 0; i < N_HAL_MAP_CASES; i++) {
		const hal_map_case_t *c = &HAL_MAP_CASES[i];

		bridge_hw_fake_reset();
		bridge_hw_fake_force(c->fn, c->inject);
		size_t               reply_len = 0u;
		gd32_bridge_status_t st        = protocol_dispatch(
		    GD32_BRIDGE_LINK_SPI, c->cmd, c->req, c->req_len, reply, REPLY_SCRATCH_CAP, &reply_len);
		zassert_equal(st, c->want, "%s", c->name);
	}
}

/* ------------------------------------------------------------------ */
/* Dedicated cases: field-dependent request/reply length.               */
/* ------------------------------------------------------------------ */

/* CMD_ADC_READ: reply is 1 + samples*2, samples itself is req[1].
 * Pins the STATUS_OK content path (against the fake's sample queue) plus
 * the two LOCAL value checks (samples == 0, samples > MAX). */
ZTEST(protocol, test_adc_read_variable_reply)
{
	uint8_t reply[REPLY_SCRATCH_CAP];
	uint8_t req[2];

	bridge_hw_fake_reset();
	bridge_hw_fake_adc_queue_push(1111u);
	bridge_hw_fake_adc_queue_push(2222u);
	bridge_hw_fake_adc_queue_push(3333u);
	req[0]                         = 0x00u; /* channel */
	req[1]                         = 0x03u; /* samples */
	size_t               reply_len = 0u;
	gd32_bridge_status_t st =
	    protocol_dispatch(GD32_BRIDGE_LINK_SPI, CMD_ADC_READ, req, 2u, reply, 7u, &reply_len);
	zassert_equal(st, STATUS_OK, "3-sample read");
	zassert_equal(reply_len, 7u, "1 + 3*2 reply bytes");
	zassert_equal(reply[0], 3u, "echoes back the validated sample count");
	uint8_t expect[7];
	expect[0] = 3u;
	put_le16(&expect[1], 1111u);
	put_le16(&expect[3], 2222u);
	put_le16(&expect[5], 3333u);
	zassert_mem_equal(reply, expect, 7u, "samples drain the fake's queue in order");

	/* samples == 0 -> STATUS_INVAL (local check, before the HAL call). */
	bridge_hw_fake_reset();
	req[0]    = 0x00u;
	req[1]    = 0x00u;
	reply_len = 0u;
	st        = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_ADC_READ, req, 2u, reply, REPLY_SCRATCH_CAP, &reply_len);
	zassert_equal(st, STATUS_INVAL, "samples == 0");

	/* samples > GD32_BRIDGE_ADC_MAX_SAMPLES -> STATUS_OUT_OF_RANGE. */
	bridge_hw_fake_reset();
	req[0]    = 0x00u;
	req[1]    = (uint8_t)(GD32_BRIDGE_ADC_MAX_SAMPLES + 1u);
	reply_len = 0u;
	st        = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_ADC_READ, req, 2u, reply, REPLY_SCRATCH_CAP, &reply_len);
	zassert_equal(st, STATUS_OUT_OF_RANGE, "samples > GD32_BRIDGE_ADC_MAX_SAMPLES");

	/* reply_cap one below the need for the 3-sample request. */
	bridge_hw_fake_reset();
	bridge_hw_fake_adc_queue_push(1u);
	req[0]    = 0x00u;
	req[1]    = 0x03u;
	reply_len = 0u;
	st = protocol_dispatch(GD32_BRIDGE_LINK_SPI, CMD_ADC_READ, req, 2u, reply, 6u, &reply_len);
	zassert_equal(st, STATUS_NOMEM, "reply_cap one short of 1 + 3*2");
}

/* CMD_ADC_STREAM_READ: reply is 1 + max_samples*2; also pins the
 * `got > max_samples` HAL-contract-violation guard (src/protocol.c:501). */
ZTEST(protocol, test_adc_stream_read_variable_reply)
{
	uint8_t reply[REPLY_SCRATCH_CAP];
	uint8_t req[2] = { 0x00u, 0x03u }; /* stream_id=0, max_samples=3 */

	bridge_hw_fake_reset();
	bridge_hw_fake_adc_stream_queue_push(0u, 10u);
	bridge_hw_fake_adc_stream_queue_push(0u, 20u);
	size_t               reply_len = 0u;
	gd32_bridge_status_t st        = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ, req, 2u, reply, 7u, &reply_len);
	zassert_equal(st, STATUS_OK, "drains what's queued, short of max_samples");
	zassert_equal(reply_len, 7u, "1 + 3*2 reply bytes regardless of `got`");
	zassert_equal(reply[0], 2u, "got == 2 (only 2 samples were queued)");
	uint8_t expect[7] = { 2u, 10u, 0u, 20u, 0u, 0u, 0u }; /* trailing slot zero-padded */
	zassert_mem_equal(reply, expect, 7u, "queued samples + zero padding");

	/* max_samples == 0 -> STATUS_INVAL; > READ_MAX -> STATUS_OUT_OF_RANGE
	 * (both local checks, before the HAL call). */
	bridge_hw_fake_reset();
	req[0]    = 0x00u;
	req[1]    = 0x00u;
	reply_len = 0u;
	st        = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ, req, 2u, reply, REPLY_SCRATCH_CAP, &reply_len);
	zassert_equal(st, STATUS_INVAL, "max_samples == 0");

	bridge_hw_fake_reset();
	req[0]    = 0x00u;
	req[1]    = (uint8_t)(GD32_BRIDGE_ADC_STREAM_READ_MAX + 1u);
	reply_len = 0u;
	st        = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ, req, 2u, reply, REPLY_SCRATCH_CAP, &reply_len);
	zassert_equal(st, STATUS_OUT_OF_RANGE, "max_samples > GD32_BRIDGE_ADC_STREAM_READ_MAX");

	/* stream_id >= GD32_BRIDGE_ADC_STREAM_COUNT -> STATUS_INVAL. */
	bridge_hw_fake_reset();
	req[0]    = (uint8_t)GD32_BRIDGE_ADC_STREAM_COUNT;
	req[1]    = 0x01u;
	reply_len = 0u;
	st        = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ, req, 2u, reply, REPLY_SCRATCH_CAP, &reply_len);
	zassert_equal(st, STATUS_INVAL, "stream_id out of range");

	/* HAL-contract violation: the fake reports got > max_samples --
	 * protocol.c must refuse to trust it rather than overrun `reply`. */
	bridge_hw_fake_reset();
	bridge_hw_fake_adc_stream_force_got(0u, 5u);
	req[0]    = 0x00u;
	req[1]    = 0x03u; /* max_samples = 3, HAL claims 5 */
	reply_len = 0u;
	st        = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ, req, 2u, reply, REPLY_SCRATCH_CAP, &reply_len);
	zassert_equal(st, STATUS_IO, "got > max_samples is a HAL contract violation, not STATUS_OK");

	/* reply_cap one below the need: STATUS_NOMEM -- but unlike
	 * CMD_ADC_READ's reply_cap check (src/protocol.c:239, BEFORE the HAL
	 * call), this one runs AFTER bridge_hw_adc_stream_read() already
	 * drained the ring (src/protocol.c:504) -- F4 item 9.  Prove it: seed
	 * 3 samples, take a reply_cap-1 NOMEM, then retry with enough room
	 * and see the ring already empty. */
	bridge_hw_fake_reset();
	bridge_hw_fake_adc_stream_queue_push(0u, 10u);
	bridge_hw_fake_adc_stream_queue_push(0u, 20u);
	bridge_hw_fake_adc_stream_queue_push(0u, 30u);
	req[0]    = 0x00u;
	req[1]    = 0x03u;
	reply_len = 0u;
	st        = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ, req, 2u, reply, 6u, &reply_len);
	zassert_equal(st, STATUS_NOMEM, "reply_cap one short of 1 + 3*2");

	reply_len = 0u;
	st        = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_ADC_STREAM_READ, req, 2u, reply, 7u, &reply_len);
	zassert_equal(st, STATUS_OK, "retry with enough room succeeds");
	zassert_equal(reply[0],
	              0u,
	              "the NOMEM call already drained the fake's ring -- the 3 samples are lost, "
	              "not requeued");
}

/* CMD_ADC_SPECTRUM_READ: reply is 7 + max_bins*4; also pins the
 * `got > max_bins` HAL-contract-violation guard (src/protocol.c:548). */
ZTEST(protocol, test_adc_spectrum_read_variable_reply)
{
	uint8_t reply[REPLY_SCRATCH_CAP];
	uint8_t req[4];
	float   bins[2] = { 1.5f, -2.5f };

	bridge_hw_fake_reset();
	bridge_hw_fake_adc_spectrum_seed(0u, 0xAABBCCDDu, 2u, bins, 2u);
	req[0] = 0x00u;                         /* stream_id */
	put_le16(&req[1], 0u);                  /* bin_offset */
	req[3]                         = 0x02u; /* max_bins */
	size_t               reply_len = 0u;
	gd32_bridge_status_t st        = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_ADC_SPECTRUM_READ, req, 4u, reply, 15u, &reply_len);
	zassert_equal(st, STATUS_OK, "2-bin spectrum read");
	zassert_equal(reply_len, 15u, "7 + 2*4 reply bytes");
	uint8_t expect[15];
	put_le32(&expect[0], 0xAABBCCDDu);
	put_le16(&expect[4], 2u);
	expect[6] = 2u;
	uint32_t w;
	memcpy(&w, &bins[0], sizeof(w));
	put_le32(&expect[7], w);
	memcpy(&w, &bins[1], sizeof(w));
	put_le32(&expect[11], w);
	zassert_mem_equal(reply, expect, 15u, "seq + total_bins + got + bin bytes");

	/* max_bins == 0 -> INVAL; > SPECTRUM_READ_MAX -> OUT_OF_RANGE. */
	bridge_hw_fake_reset();
	req[0] = 0x00u;
	put_le16(&req[1], 0u);
	req[3]    = 0x00u;
	reply_len = 0u;
	st        = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_ADC_SPECTRUM_READ, req, 4u, reply, REPLY_SCRATCH_CAP, &reply_len);
	zassert_equal(st, STATUS_INVAL, "max_bins == 0");

	bridge_hw_fake_reset();
	req[0] = 0x00u;
	put_le16(&req[1], 0u);
	req[3]    = (uint8_t)(GD32_BRIDGE_ADC_SPECTRUM_READ_MAX + 1u);
	reply_len = 0u;
	st        = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_ADC_SPECTRUM_READ, req, 4u, reply, REPLY_SCRATCH_CAP, &reply_len);
	zassert_equal(st, STATUS_OUT_OF_RANGE, "max_bins > GD32_BRIDGE_ADC_SPECTRUM_READ_MAX");

	/* HAL-contract violation: got > max_bins. */
	bridge_hw_fake_reset();
	bridge_hw_fake_adc_spectrum_force_got(0u, 9u);
	req[0] = 0x00u;
	put_le16(&req[1], 0u);
	req[3]    = 0x02u;
	reply_len = 0u;
	st        = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_ADC_SPECTRUM_READ, req, 4u, reply, REPLY_SCRATCH_CAP, &reply_len);
	zassert_equal(st, STATUS_IO, "got > max_bins is a HAL contract violation");

	/* reply_cap one below the need: STATUS_NOMEM -- runs AFTER the HAL
	 * call too (src/protocol.c:551), same shape as ADC_STREAM_READ's
	 * boundary above (F4 item 9).  Unlike that ring, THIS fixture's
	 * spectrum model is non-destructive (bridge_hw_adc_spectrum_read
	 * copies from a static seeded frame rather than draining a queue --
	 * see bridge_hw_fake.c), so a retry below still sees the same frame;
	 * this pins the boundary itself without overclaiming a "samples
	 * lost" property this particular fake body doesn't model. */
	bridge_hw_fake_reset();
	bridge_hw_fake_adc_spectrum_seed(0u, 0xAABBCCDDu, 2u, bins, 2u);
	req[0] = 0x00u;
	put_le16(&req[1], 0u);
	req[3]    = 0x02u;
	reply_len = 0u;
	st        = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_ADC_SPECTRUM_READ, req, 4u, reply, 14u, &reply_len);
	zassert_equal(st, STATUS_NOMEM, "reply_cap one short of 7 + 2*4");
}

/* CMD_ADC_DSP_STAGE_PUSH: req_payload_len is a MINIMUM (7-byte header),
 * not an exact match -- axis 1 reads differently here than everywhere
 * else in this file, so it gets its own boundary case instead of a
 * FIXED_CASES row. */
ZTEST(protocol, test_adc_dsp_stage_push_length_boundary)
{
	uint8_t reply[REPLY_SCRATCH_CAP];

	/* Below the 7-byte header: STATUS_INVAL regardless of content. */
	bridge_hw_fake_reset();
	size_t               reply_len = 0u;
	gd32_bridge_status_t st        = protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                                   CMD_ADC_DSP_STAGE_PUSH,
	                                                   req_adc_dsp_stage_push,
	                                                   6u,
	                                                   reply,
	                                                   REPLY_SCRATCH_CAP,
	                                                   &reply_len);
	zassert_equal(st, STATUS_INVAL, "6 bytes is below the 7-byte header minimum");

	/* Exactly the header, zero data bytes, chunk_total_size == 0:
	 * offset(0) + data_len(0) <= total(0), so this is a VALID (if
	 * pointless) push. */
	bridge_hw_fake_reset();
	uint8_t header_only[7] = { 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u };
	reply_len              = 0u;
	st                     = protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                           CMD_ADC_DSP_STAGE_PUSH,
	                                           header_only,
	                                           7u,
	                                           reply,
	                                           REPLY_SCRATCH_CAP,
	                                           &reply_len);
	zassert_equal(st, STATUS_OK, "7-byte header with an empty chunk is a valid minimum request");

	/* The canonical 8-byte request (header + 1 data byte, exactly
	 * filling chunk_total_size == 1) -- STATUS_OK, and ONE more byte
	 * (9 total) overflows chunk_total_size -> STATUS_OUT_OF_RANGE, NOT
	 * STATUS_INVAL: the length gate only enforces the 7-byte MINIMUM,
	 * so a request longer than the header is a shape violation the
	 * offset/length range check catches instead (src/protocol.c:820). */
	bridge_hw_fake_reset();
	reply_len = 0u;
	st        = protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                              CMD_ADC_DSP_STAGE_PUSH,
	                              req_adc_dsp_stage_push,
	                              8u,
	                              reply,
	                              REPLY_SCRATCH_CAP,
	                              &reply_len);
	zassert_equal(st, STATUS_OK, "8-byte canonical push (header + 1 in-bounds data byte)");

	/* Argument-decode coverage (F4 item 11): chain_id, stage_index, kind,
	 * chunk_offset, chunk_total_size and chunk_data must all have landed
	 * on the HAL exactly as req_adc_dsp_stage_push encoded them.  This is
	 * plain decode coverage, orthogonal to the still-open #69/#70 chain-
	 * shape validation gap (see this file's header). */
	uint8_t  cap_chain_id = 0xFFu, cap_stage_index = 0xFFu, cap_kind = 0xFFu;
	uint16_t cap_chunk_offset = 0xFFFFu, cap_chunk_total_size = 0xFFFFu;
	uint8_t  cap_chunk_data[GD32_BRIDGE_FAKE_STAGE_PUSH_DATA_CAP];
	size_t   cap_chunk_data_len = 0xFFu;
	bridge_hw_fake_dsp_stage_push_get_last_call(&cap_chain_id,
	                                            &cap_stage_index,
	                                            &cap_kind,
	                                            &cap_chunk_offset,
	                                            &cap_chunk_total_size,
	                                            cap_chunk_data,
	                                            &cap_chunk_data_len);
	zassert_equal(cap_chain_id, req_adc_dsp_stage_push[0], "ADC_DSP_STAGE_PUSH: chain_id");
	zassert_equal(cap_stage_index, req_adc_dsp_stage_push[1], "ADC_DSP_STAGE_PUSH: stage_index");
	zassert_equal(cap_kind, req_adc_dsp_stage_push[2], "ADC_DSP_STAGE_PUSH: kind");
	zassert_equal(cap_chunk_offset, 0u, "ADC_DSP_STAGE_PUSH: chunk_offset");
	zassert_equal(cap_chunk_total_size, 1u, "ADC_DSP_STAGE_PUSH: chunk_total_size");
	zassert_equal(cap_chunk_data_len, 1u, "ADC_DSP_STAGE_PUSH: chunk_data_len");
	zassert_equal(
	    cap_chunk_data[0], req_adc_dsp_stage_push[7], "ADC_DSP_STAGE_PUSH: chunk_data[0]");

	bridge_hw_fake_reset();
	uint8_t overlong[9];
	memcpy(overlong, req_adc_dsp_stage_push, 8u);
	overlong[8] = 0xBBu;
	reply_len   = 0u;
	st          = protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_ADC_DSP_STAGE_PUSH,
	                                overlong,
	                                9u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len);
	zassert_equal(st, STATUS_OUT_OF_RANGE, "chunk_offset + chunk_data_len > chunk_total_size");

	/* kind > 3 (not one of FIR/IIR/WINDOW/FFT) -> STATUS_INVAL (local
	 * check, before the HAL call). */
	bridge_hw_fake_reset();
	uint8_t bad_kind[7] = { 0x00u, 0x00u, 0x04u, 0x00u, 0x00u, 0x00u, 0x00u };
	reply_len           = 0u;
	st                  = protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                        CMD_ADC_DSP_STAGE_PUSH,
	                                        bad_kind,
	                                        7u,
	                                        reply,
	                                        REPLY_SCRATCH_CAP,
	                                        &reply_len);
	zassert_equal(st, STATUS_INVAL, "kind > 3");
}

/* ------------------------------------------------------------------ */
/* LOCAL value validation -- pure protocol.c logic, never reaching the  */
/* HAL, so none of these depend on (or need to force) the fake at all.  */
/* Grouped into one case since each is a single boundary on a           */
/* different opcode's already-covered request shape.                    */
/* ------------------------------------------------------------------ */

ZTEST(protocol, test_local_value_validation)
{
	uint8_t reply[REPLY_SCRATCH_CAP];
	size_t  reply_len;

	/* PWM_SET: duty_ns > period_ns -> STATUS_INVAL. */
	bridge_hw_fake_reset();
	uint8_t bad_pwm_set[10];
	memcpy(bad_pwm_set, req_pwm_set, sizeof(bad_pwm_set));
	put_le32(&bad_pwm_set[2], 1000u); /* period */
	put_le32(&bad_pwm_set[6], 2000u); /* duty > period */
	reply_len = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_PWM_SET,
	                                bad_pwm_set,
	                                10u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "duty_ns > period_ns");

	/* PWM_CONFIGURE: align_mode > 3 -> STATUS_INVAL. */
	bridge_hw_fake_reset();
	uint8_t bad_align[7];
	memcpy(bad_align, req_pwm_configure, sizeof(bad_align));
	bad_align[1] = 0x04u;
	reply_len    = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_PWM_CONFIGURE,
	                                bad_align,
	                                7u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "align_mode > 3");

	/* PWM_CAPTURE_BEGIN: edge > 2 -> STATUS_INVAL. */
	bridge_hw_fake_reset();
	uint8_t bad_edge[2] = { 0x00u, 0x03u };
	reply_len           = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_PWM_CAPTURE_BEGIN,
	                                bad_edge,
	                                2u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "edge > 2");

	/* PWM_SINGLE_PULSE: pulse_ns == 0 -> STATUS_INVAL. */
	bridge_hw_fake_reset();
	uint8_t zero_pulse[8] = { 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u };
	reply_len             = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_PWM_SINGLE_PULSE,
	                                zero_pulse,
	                                8u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "pulse_ns == 0");

	/* TIMER_SYNC has no local value check beyond length -- HAL owns
	 * master/slave/mode validity (see #24 B6 for why that matters,
	 * out of scope for the fake). Nothing to assert here.
	 *
	 * POWER_MODE_SET: mode > 3 -> STATUS_INVAL. */
	bridge_hw_fake_reset();
	uint8_t bad_mode[10];
	memcpy(bad_mode, req_power_mode_set, sizeof(bad_mode));
	bad_mode[0] = 0x04u;
	reply_len   = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_POWER_MODE_SET,
	                                bad_mode,
	                                10u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "mode > 3");

	/* ADC_CONFIGURE: resolution_bits not in {0,6,8,10,12,14,16} ->
	 * STATUS_INVAL. */
	bridge_hw_fake_reset();
	uint8_t bad_res[7];
	memcpy(bad_res, req_adc_configure, sizeof(bad_res));
	bad_res[6] = 0x07u;
	reply_len  = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_ADC_CONFIGURE,
	                                bad_res,
	                                7u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "resolution_bits == 7");

	/* ADC_STREAM_BEGIN: stream_id out of range, and sample_rate_hz == 0
	 * -> STATUS_INVAL. */
	bridge_hw_fake_reset();
	uint8_t bad_stream_id[7];
	memcpy(bad_stream_id, req_adc_stream_begin, sizeof(bad_stream_id));
	bad_stream_id[0] = (uint8_t)GD32_BRIDGE_ADC_STREAM_COUNT;
	reply_len        = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_ADC_STREAM_BEGIN,
	                                bad_stream_id,
	                                7u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "stream_id >= GD32_BRIDGE_ADC_STREAM_COUNT");

	bridge_hw_fake_reset();
	uint8_t zero_rate[7];
	memcpy(zero_rate, req_adc_stream_begin, sizeof(zero_rate));
	put_le32(&zero_rate[3], 0u);
	reply_len = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_ADC_STREAM_BEGIN,
	                                zero_rate,
	                                7u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "sample_rate_hz == 0");

	/* ADC_STREAM_END: stream_id out of range -> STATUS_INVAL. */
	bridge_hw_fake_reset();
	uint8_t bad_end[1] = { (uint8_t)GD32_BRIDGE_ADC_STREAM_COUNT };
	reply_len          = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_ADC_STREAM_END,
	                                bad_end,
	                                1u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "stream_id >= GD32_BRIDGE_ADC_STREAM_COUNT");

	/* ADC_DSP_CHAIN_BIND: stream_id out of range -> STATUS_INVAL. */
	bridge_hw_fake_reset();
	uint8_t bad_bind[2] = { 0x00u, (uint8_t)GD32_BRIDGE_ADC_STREAM_COUNT };
	reply_len           = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_ADC_DSP_CHAIN_BIND,
	                                bad_bind,
	                                2u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "stream_id >= GD32_BRIDGE_ADC_STREAM_COUNT");

	/* SE_RESET: assert > 1 -> STATUS_INVAL. */
	bridge_hw_fake_reset();
	uint8_t bad_assert[1] = { 0x02u };
	reply_len             = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_SE_RESET,
	                                bad_assert,
	                                1u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "assert > 1");

	/* TMU_COMPUTE: function >= BRIDGE_TMU_FN__COUNT, and separately
	 * format >= BRIDGE_TMU_FMT__COUNT -> STATUS_INVAL. */
	bridge_hw_fake_reset();
	uint8_t bad_fn[12];
	memcpy(bad_fn, req_tmu_compute, sizeof(bad_fn));
	bad_fn[0] = (uint8_t)BRIDGE_TMU_FN__COUNT;
	reply_len = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_TMU_COMPUTE,
	                                bad_fn,
	                                12u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "function >= BRIDGE_TMU_FN__COUNT");

	bridge_hw_fake_reset();
	uint8_t bad_fmt[12];
	memcpy(bad_fmt, req_tmu_compute, sizeof(bad_fmt));
	bad_fmt[1] = (uint8_t)BRIDGE_TMU_FMT__COUNT;
	reply_len  = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_TMU_COMPUTE,
	                                bad_fmt,
	                                12u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "format >= BRIDGE_TMU_FMT__COUNT");

	/* TRNG_READ: want == 0 or want > 32 -> STATUS_INVAL (local gate,
	 * src/protocol.c:601; previously uncovered in full -- F4 item 7). */
	bridge_hw_fake_reset();
	uint8_t zero_want[1] = { 0x00u };
	reply_len            = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_TRNG_READ,
	                                zero_want,
	                                1u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "want == 0");

	bridge_hw_fake_reset();
	uint8_t over_want[1] = { 33u };
	reply_len            = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_TRNG_READ,
	                                over_want,
	                                1u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "want > 32");

	/* ADC_SPECTRUM_READ: stream_id out of range -> STATUS_INVAL (mirrors
	 * its ADC_STREAM_READ twin above; previously uncovered -- an
	 * oversight per this file's header, F4 item 8). */
	bridge_hw_fake_reset();
	uint8_t bad_spectrum_stream[4];
	memcpy(bad_spectrum_stream, req_adc_spectrum_read, sizeof(bad_spectrum_stream));
	bad_spectrum_stream[0] = (uint8_t)GD32_BRIDGE_ADC_STREAM_COUNT;
	reply_len              = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_ADC_SPECTRUM_READ,
	                                bad_spectrum_stream,
	                                4u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "stream_id >= GD32_BRIDGE_ADC_STREAM_COUNT");
}

/* ------------------------------------------------------------------ */
/* GPIO_WRITE -> GPIO_READ round trip through protocol_dispatch() only  */
/* (no direct fake introspection) -- proves the write side, which the   */
/* generic table above cannot (its reply_need is 0). */
/* ------------------------------------------------------------------ */

ZTEST(protocol, test_gpio_write_then_read_roundtrip)
{
	uint8_t reply[REPLY_SCRATCH_CAP];
	uint8_t write_req[8] = {
		0xFF, 0x00, 0x00, 0x00, 0xA5, 0x00, 0x00, 0x00
	}; /* mask=0xFF, levels=0xA5 */
	uint8_t read_req[4] = { 0xFF, 0xFF, 0xFF, 0xFF }; /* mask = everything */

	bridge_hw_fake_reset();
	size_t reply_len = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_GPIO_WRITE,
	                                write_req,
	                                8u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_OK,
	              "GPIO_WRITE succeeds");

	reply_len               = 0u;
	gd32_bridge_status_t st = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, CMD_GPIO_READ, read_req, 4u, reply, REPLY_SCRATCH_CAP, &reply_len);
	zassert_equal(st, STATUS_OK, "GPIO_READ succeeds");
	zassert_equal(reply_len, 4u, "GPIO_READ reply is 4 bytes");
	uint8_t expect[4] = { 0xA5, 0x00, 0x00, 0x00 };
	zassert_mem_equal(reply, expect, 4u, "read-back reflects the just-written pads under mask");
}

/* ------------------------------------------------------------------ */
/* CMD_RESET_REASON: destructive-read contract (F2, gh#56).  A second    */
/* dispatch without a fresh bridge_hw_fake_set_reset_reason() in between */
/* must read back 0x00 (UNKNOWN), not the same cause twice -- proving    */
/* the fake's bridge_hw_reset_reason() actually latches, matching the    */
/* CURRENT documented hal/bridge_hw.h / hal/gd32/init.c / protocol.c     */
/* contract.  If/when gh#56 lands and flips that contract to an          */
/* idempotent boot-snapshot, this case (and bridge_hw_fake.c's latch)    */
/* must change in the SAME commit, not silently disagree with it.        */
/* ------------------------------------------------------------------ */

ZTEST(protocol, test_reset_reason_destructive_read)
{
	uint8_t reply[REPLY_SCRATCH_CAP];

	bridge_hw_fake_reset();
	bridge_hw_fake_set_reset_reason(0x03u);
	size_t reply_len = 0u;
	zassert_equal(
	    protocol_dispatch(
	        GD32_BRIDGE_LINK_SPI, CMD_RESET_REASON, NULL, 0u, reply, REPLY_SCRATCH_CAP, &reply_len),
	    STATUS_OK,
	    "first read succeeds");
	zassert_equal(reply[0], 0x03u, "first read reports the seeded cause");

	reply_len = 0u;
	zassert_equal(
	    protocol_dispatch(
	        GD32_BRIDGE_LINK_SPI, CMD_RESET_REASON, NULL, 0u, reply, REPLY_SCRATCH_CAP, &reply_len),
	    STATUS_OK,
	    "second read succeeds");
	zassert_equal(
	    reply[0], 0x00u, "second read (no re-seed in between) reports UNKNOWN -- the read latched");
}

/* ------------------------------------------------------------------ */
/* bridge_hw_se_reset() direct HAL call (F6, hal/bridge_hw.h:255):       */
/* out-of-range `assert` documents BRIDGE_HW_ERR_INVAL, mirrored by      */
/* hal/gd32/se_reset.c:73.  protocol.c:277 pre-rejects assert>1 before   */
/* protocol_dispatch() ever reaches the HAL (see HAL_MAP_CASES' SE_RESET */
/* rows), so only a direct call exercises this branch of the fake.       */
/* ------------------------------------------------------------------ */

ZTEST(protocol, test_se_reset_hal_contract_inval_direct)
{
	bridge_hw_fake_reset();
	zassert_equal(bridge_hw_se_reset(2u), BRIDGE_HW_ERR_INVAL, "assert > 1, direct HAL call");
	zassert_equal(bridge_hw_fake_se_reset_call_count(),
	              0u,
	              "the rejected call does not count as a real reset");
}

/* ------------------------------------------------------------------ */
/* PWM period is shared per timer group (F7, hal/bridge_hw.h:87):        */
/* setting one channel's period must change its TIMER SIBLINGS' reported */
/* period too, while leaving the other timer group's period untouched.   */
/* GD32_BRIDGE_FAKE_PWM_CHANNELS_PER_TIMER channels share one timer.     */
/* ------------------------------------------------------------------ */

ZTEST(protocol, test_pwm_period_shared_per_timer)
{
	uint8_t reply[REPLY_SCRATCH_CAP];

	bridge_hw_fake_reset();
	uint8_t set_req[10] = { 0x00, 0x00, 0x40, 0x42, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00 };
	/* channel=0, period=1,000,000 ns, duty=0 */
	size_t reply_len = 0u;
	zassert_equal(
	    protocol_dispatch(
	        GD32_BRIDGE_LINK_SPI, CMD_PWM_SET, set_req, 10u, reply, REPLY_SCRATCH_CAP, &reply_len),
	    STATUS_OK,
	    "PWM_SET on channel 0 succeeds");

	/* Channel 1 shares channel 0's timer group (both < CHANNELS_PER_TIMER
	 * apart) -- PWM_GET on it must report the SAME period, sight unseen. */
	uint8_t get_sibling[1] = { 0x01u };
	reply_len              = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_PWM_GET,
	                                get_sibling,
	                                1u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_OK,
	              "PWM_GET on channel 1 succeeds");
	zassert_equal(reply[0], 0x40u, "channel 1's period low byte reflects channel 0's PWM_SET");
	zassert_equal(reply[1], 0x42u, "channel 1's period byte 1 reflects channel 0's PWM_SET");
	zassert_equal(reply[2], 0x0Fu, "channel 1's period byte 2 reflects channel 0's PWM_SET");
	zassert_equal(reply[3], 0x00u, "channel 1's period byte 3 reflects channel 0's PWM_SET");

	/* Channel 4 is a DIFFERENT timer group -- it must still report the
	 * boot default (65.536 ms), untouched by channel 0's PWM_SET. */
	uint8_t get_other_timer[1] = { 0x04u };
	reply_len                  = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_PWM_GET,
	                                get_other_timer,
	                                1u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_OK,
	              "PWM_GET on channel 4 succeeds");
	uint8_t expect_boot[4] = { 0x00, 0x00, 0xE8, 0x03 }; /* 65,536,000 ns LE */
	zassert_mem_equal(reply,
	                  expect_boot,
	                  4u,
	                  "channel 4 (a different timer) still reports the untouched boot period");
}

/* ------------------------------------------------------------------ */
/* CMD_LINK_FEATURES: intersection-masking + idempotent disable.  Not   */
/* in FIXED_CASES because these two calls are chained on purpose (the   */
/* second call's correctness depends on the first, unlike every other   */
/* fixed-case entry). */
/* ------------------------------------------------------------------ */

ZTEST(protocol, test_link_features_grant_then_disable)
{
	uint8_t reply[REPLY_SCRATCH_CAP];
	uint8_t want_all[1]  = { 0xFFu };
	uint8_t want_none[1] = { 0x00u };

	bridge_hw_fake_reset();
	size_t reply_len = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_LINK_FEATURES,
	                                want_all,
	                                1u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_OK,
	              "grant request succeeds");
	zassert_equal(reply_len, 1u, "1-byte granted-features reply");
	zassert_equal(reply[0],
	              GD32_BRIDGE_LINK_FEAT_STATUS_SEQ,
	              "0xFF is masked down to the one bit this firmware implements");
	zassert_equal(protocol_link_features(GD32_BRIDGE_LINK_SPI),
	              GD32_BRIDGE_LINK_FEAT_STATUS_SEQ,
	              "armed immediately, observable via the accessor");

	reply_len = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_LINK_FEATURES,
	                                want_none,
	                                1u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_OK,
	              "disable request succeeds");
	zassert_equal(reply[0], 0x00u, "nothing granted");
	zassert_equal(protocol_link_features(GD32_BRIDGE_LINK_SPI), 0x00u, "disarmed immediately");
}

/* #130 -- CMD_LINK_FEATURES sits in the SHARED dispatch table and is
 * therefore reachable from both transport ISRs, but STATUS_SEQ is
 * declared SPI-only (src/protocol.h) and the SPI transport is its only
 * consumer.  With one process-wide feature byte, an I2C-side driver
 * open/close/reset issuing the documented idempotent `features = 0`
 * disarmed a live SPI STATUS_SEQ session with nothing telling the SPI
 * host -- switching off its ONLY detector for the stale-reply residual
 * hazard fingerprinted on silicon 2026-06-06.  The reverse was equally
 * reachable: an I2C-side `features = 1` re-framed the SPI wire for a
 * host that never negotiated it.
 *
 * Both directions are asserted here.  Before the per-link split the
 * first read-back below returned 0x00 and the second returned
 * GD32_BRIDGE_LINK_FEAT_STATUS_SEQ. */
ZTEST(protocol, test_link_features_are_per_transport)
{
	uint8_t reply[REPLY_SCRATCH_CAP];
	uint8_t want_all[1]  = { 0xFFu };
	uint8_t want_none[1] = { 0x00u };
	size_t  reply_len    = 0u;

	bridge_hw_fake_reset();

	/* SPI host negotiates STATUS_SEQ and is relying on the stamp. */
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_LINK_FEATURES,
	                                want_all,
	                                1u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_OK,
	              "SPI grant succeeds");
	zassert_equal(protocol_link_features(GD32_BRIDGE_LINK_SPI),
	              GD32_BRIDGE_LINK_FEAT_STATUS_SEQ,
	              "SPI link armed");

	/* An I2C-side open/close/reset path issues the idempotent zero. */
	reply_len = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_I2C,
	                                CMD_LINK_FEATURES,
	                                want_none,
	                                1u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_OK,
	              "I2C disable is answered honestly, not NOSUPPORT -- the documented "
	              "open/close path issues it unconditionally");
	zassert_equal(reply[0], 0x00u, "I2C link reports nothing granted");
	zassert_equal(protocol_link_features(GD32_BRIDGE_LINK_I2C), 0x00u, "I2C link disarmed");
	zassert_equal(protocol_link_features(GD32_BRIDGE_LINK_SPI),
	              GD32_BRIDGE_LINK_FEAT_STATUS_SEQ,
	              "the SPI session MUST survive an I2C-side features=0");

	/* And the reverse: an I2C-side grant must not re-frame the SPI wire
	 * for a host that never negotiated it. */
	reply_len = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                CMD_LINK_FEATURES,
	                                want_none,
	                                1u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_OK,
	              "SPI disarms itself");
	reply_len = 0u;
	zassert_equal(protocol_dispatch(GD32_BRIDGE_LINK_I2C,
	                                CMD_LINK_FEATURES,
	                                want_all,
	                                1u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_OK,
	              "I2C grant succeeds");
	zassert_equal(protocol_link_features(GD32_BRIDGE_LINK_I2C),
	              GD32_BRIDGE_LINK_FEAT_STATUS_SEQ,
	              "I2C link armed for its own host");
	zassert_equal(protocol_link_features(GD32_BRIDGE_LINK_SPI),
	              0x00u,
	              "the SPI wire MUST NOT be re-framed by an I2C-side grant");
}

/* An out-of-range link tag must not index the feature array. */
ZTEST(protocol, test_link_features_rejects_unknown_link)
{
	uint8_t reply[REPLY_SCRATCH_CAP];
	uint8_t want_all[1] = { 0xFFu };
	size_t  reply_len   = 0u;

	bridge_hw_fake_reset();
	zassert_equal(protocol_dispatch((gd32_bridge_link_t)GD32_BRIDGE_LINK_COUNT,
	                                CMD_LINK_FEATURES,
	                                want_all,
	                                1u,
	                                reply,
	                                REPLY_SCRATCH_CAP,
	                                &reply_len),
	              STATUS_INVAL,
	              "an unknown link tag is refused, not used as an array index");
	zassert_equal(protocol_link_features((gd32_bridge_link_t)GD32_BRIDGE_LINK_COUNT),
	              0u,
	              "the accessor answers 0 for an unknown link");
}

/* ------------------------------------------------------------------ */
/* No dispatch-table entry: the generic default, the 0x36 tombstone,    */
/* and the (inert, unpartitioned) 0xF0..0xFF OTA range.                 */
/* ------------------------------------------------------------------ */

ZTEST(protocol, test_unknown_opcode_default)
{
	uint8_t reply[REPLY_SCRATCH_CAP];
	size_t  reply_len = 0xDEADu; /* poisoned; must become 0 */

	bridge_hw_fake_reset();
	/* 0x99 is unassigned in gd32_bridge_cmd_t and below CMD_OTA_BEGIN
	 * (0xF0), so it must fall through to the plain STATUS_NOSUPPORT
	 * default rather than the OTA range. */
	gd32_bridge_status_t st = protocol_dispatch(
	    GD32_BRIDGE_LINK_SPI, 0x99u, NULL, 0u, reply, REPLY_SCRATCH_CAP, &reply_len);
	zassert_equal(st, STATUS_NOSUPPORT, "unassigned opcode");
	zassert_equal(reply_len, 0u, "no reply payload");
}

ZTEST(protocol, test_reserved_0x36_tombstone)
{
	uint8_t reply[REPLY_SCRATCH_CAP];
	size_t  reply_len = 0xDEADu; /* poisoned; must become 0 */

	bridge_hw_fake_reset();
	/* CMD_ADC_STREAM_CONFIGURE_DSP (0x36) has no case in
	 * protocol_dispatch()'s switch by design (src/protocol.c:994-997)
	 * -- the chunked CMD_ADC_DSP_CHAIN_* path replaced it before this
	 * opcode ever shipped. It must fall through to the same default as
	 * any other unassigned opcode. */
	gd32_bridge_status_t st = protocol_dispatch(GD32_BRIDGE_LINK_SPI,
	                                            CMD_ADC_STREAM_CONFIGURE_DSP,
	                                            NULL,
	                                            0u,
	                                            reply,
	                                            REPLY_SCRATCH_CAP,
	                                            &reply_len);
	zassert_equal(st, STATUS_NOSUPPORT, "0x36 tombstone");
	zassert_equal(reply_len, 0u, "no reply payload");
}

ZTEST(protocol, test_ota_range_is_inert_without_partitioning)
{
	uint8_t reply[REPLY_SCRATCH_CAP];

	/* This suite's CMake target does not define -DBRIDGE_OTA_PARTITIONED
	 * (see this file's header comment), so src/ota.c's ota_dispatch()
	 * takes the unconditional-NOSUPPORT branch at src/ota.c:483-499 for
	 * every opcode in 0xF0..0xFF, regardless of payload. Walk the whole
	 * documented range (src/bootloader/bootloader.h:27-33), not just the
	 * named CMD_OTA_* constants, since protocol.c's `cmd >=
	 * CMD_OTA_BEGIN` routes anything up to and including 0xFF here. */
	for (unsigned cmd = CMD_OTA_BEGIN; cmd <= 0xFFu; cmd++) {
		bridge_hw_fake_reset();
		size_t               reply_len = 0xDEADu; /* poisoned; must become 0 */
		gd32_bridge_status_t st        = protocol_dispatch(
		    GD32_BRIDGE_LINK_SPI, (uint8_t)cmd, NULL, 0u, reply, REPLY_SCRATCH_CAP, &reply_len);
		zassert_equal(st, STATUS_NOSUPPORT, "opcode 0x%02X inert without partitioning", cmd);
		zassert_equal(reply_len, 0u, "opcode 0x%02X: no reply payload", cmd);
	}
}

ZTEST_SUITE(protocol, NULL, NULL, NULL, NULL, NULL);
