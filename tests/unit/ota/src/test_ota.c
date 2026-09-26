/* SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the gd32-bridge OTA Path-A handler (ota.c, built with
 * -DBRIDGE_OTA_PARTITIONED).  Focus: the OTA_WRITE_CHUNK offset/length
 * bounds check must reject wrapped ranges (offset near UINT32_MAX) before
 * computing a flash address or calling ota_fmc_program.
 *
 * The firmware addresses flash as raw 32-bit constants cast to pointers,
 * which is not host-safe on native_sim.  The weak ota_fmc_* seams are
 * therefore overridden here to redirect every flash read/program/erase
 * into a host-side buffer, so the REAL state machine runs unmodified.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "ota.h"
#include "ota_layout.h"
#include "fmc_ota.h"
#include "crc32.h"
#include "protocol.h"              /* protocol_dispatch(), STATUS_BUSY, CMD_PING/... */
#include "bootloader/bootloader.h" /* CMD_OTA_* */

/* This suite is built TWICE (tests/unit/CMakeLists.txt: test_ota_slot_a,
 * test_ota_slot_b), each with a different -DBRIDGE_APP_SLOT_BASE, so the
 * #3 divergence tests below exercise ota.c's compile-time OTA_RUNNING_SLOT
 * derivation for BOTH values instead of just one.  TEST_RUNNING_SLOT /
 * TEST_OTHER_SLOT mirror that same derivation independently (this file
 * cannot see ota.c's static OTA_RUNNING_SLOT), so the tests can assert
 * against "the slot this binary runs from" without hardcoding which of
 * the two builds is currently running. */
#if !defined(BRIDGE_APP_SLOT_BASE)
#error "test_ota needs -DBRIDGE_APP_SLOT_BASE (see tests/unit/CMakeLists.txt)"
#elif (BRIDGE_APP_SLOT_BASE) == OTA_SLOT_A_BASE
#define TEST_RUNNING_SLOT OTA_SLOT_A
#define TEST_OTHER_SLOT   OTA_SLOT_B
#elif (BRIDGE_APP_SLOT_BASE) == OTA_SLOT_B_BASE
#define TEST_RUNNING_SLOT OTA_SLOT_B
#define TEST_OTHER_SLOT   OTA_SLOT_A
#else
#error "BRIDGE_APP_SLOT_BASE matches neither OTA_SLOT_A_BASE nor OTA_SLOT_B_BASE"
#endif

/* ---- Host flash model: mirror the whole device flash into a buffer ---- */

#define FL_BASE OTA_BOOTLOADER_BASE
#define FL_SIZE (OTA_FLASH_END - OTA_BOOTLOADER_BASE)

static uint8_t  g_flash[FL_SIZE];
static uint32_t g_program_calls;
static bool     g_program_fail; /* #74: models a PGERR-with-power-ON write
                                  * failure -- ota_fmc_program() returns
                                  * false having written nothing.  A REAL
                                  * power cut is a different EVENT (no
                                  * status ever reaches the host), but
                                  * leaves the SAME flash END STATE:
                                  * hal/fmc_ota.c programs ascending 8-byte
                                  * doublewords and bails on the first
                                  * non-FMC_READY, and rec_crc32 is the
                                  * LAST field (offsetof 40, sizeof 44), so
                                  * any ascending tear leaves the page
                                  * CRC-invalid regardless of which of the
                                  * two events stopped it. */
static bool     g_erase_fail;   /* meta_commit's ota_fmc_erase_range()-fails
                                * early return (ota.c) had no seam before
                                * this; mirrors g_program_fail but models
                                * the erase failing with nothing on the
                                * target page changed.  That is ONE of the
                                * real erase's three failure shapes -- the
                                * other two (erase started then failed, and
                                * the first of the region's two 1 KB pages
                                * erased before the second failed) leave the
                                * target torn instead, which degenerates to
                                * the power-cut end state the case above
                                * already pins.  The target is never the
                                * higher-ranked page, so the rank rule
                                * covers all three. */

static uint8_t *_host_ptr(uint32_t addr)
{
	zassert_true(addr >= FL_BASE && addr < FL_BASE + FL_SIZE,
	             "flash addr 0x%x out of model",
	             (unsigned)addr);
	return &g_flash[addr - FL_BASE];
}

/* ---- weak seam overrides -------------------------------------------- */

bool ota_fmc_supported(void)
{
	return true;
}

bool ota_fmc_erase_range(uint32_t base, uint32_t len)
{
	if (g_erase_fail) {
		/* Models the erase failing with the target untouched (e.g. a
		 * latched FMC error the caller cannot clear), matching
		 * hal/fmc_ota.c's erase_one_page() aborting before
		 * FMC_CTL_START on the first non-FMC_READY wait.  The shapes
		 * that tear the target instead are covered by the power-cut
		 * case -- see g_erase_fail's declaration. */
		return false;
	}
	memset(_host_ptr(base), 0xFF, len);
	return true;
}

bool ota_fmc_program(uint32_t addr, const uint8_t *data, size_t len)
{
	if (g_program_fail) {
		/* Models the PGERR-with-power-ON write failure (see
		 * g_program_fail above), not a real power cut: the target page
		 * is already erased (invalid) and stays that way, without
		 * copying the new record into it -- the same end state a real
		 * cut would leave. */
		return false;
	}
	memcpy(_host_ptr(addr), data, len);
	g_program_calls++;
	return true;
}

const void *ota_fmc_flash_ptr(uint32_t addr)
{
	return _host_ptr(addr);
}

static uint32_t g_reset_calls;

void ota_system_reset(void)
{
	g_reset_calls++;
}

/* C3 (adversarial-verify finding): ota_fault_loop_clear()'s weak default
 * is a no-op; override it here to prove ota_boot_init()'s self-heal path
 * and ota_confirm_tick()'s successful commit both call it. */
static uint32_t g_fault_loop_clear_calls;

void ota_fault_loop_clear(void)
{
	g_fault_loop_clear_calls++;
}

/* ---- helpers -------------------------------------------------------- */

static void wr_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static uint32_t rd_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

/* Open a fresh OTA session; leaves the state machine READY. */
static void begin_session(uint32_t img_len)
{
	uint8_t req[8];
	wr_u32(&req[0], img_len);
	wr_u32(&req[4], 0u); /* expected_crc (unused until VERIFY) */
	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof(req), reply, sizeof(reply), &rlen),
	              STATUS_OK);
	/* BEGIN now arms a BACKGROUND erase and acks immediately (#770): the
	 * slot is not erased inline, so pump ota_erase_tick() the way the main
	 * loop would until the erase drains and the state reaches READY. */
	for (unsigned i = 0u; i < (OTA_SLOT_SIZE / OTA_PAGE_SIZE) + 4u; ++i) {
		ota_erase_tick();
	}
}

static uint8_t ota_state_now(void)
{
	uint8_t reply[8] = { 0 };
	size_t  rlen     = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_GET_STATE, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_OK);
	return reply[0]; /* state:u8 */
}

/* Keep dlen and span_len independent: malformed-frame tests need to
 * exercise h_write's load-bearing cross-check instead of constructing a
 * self-consistent request by definition (#145). */
static gd32_bridge_status_t write_chunk_raw(uint32_t       off,
                                            const uint8_t *data,
                                            uint8_t        dlen,
                                            size_t         span_len,
                                            uint8_t       *reply,
                                            size_t        *rlen)
{
	uint8_t req[5 + 255] = { 0 };
	wr_u32(&req[0], off);
	req[4] = dlen;
	if (dlen > 0u) {
		memcpy(&req[5], data, dlen);
	}
	return ota_dispatch(CMD_OTA_WRITE_CHUNK, req, span_len, reply, 8u, rlen);
}

static gd32_bridge_status_t
write_chunk(uint32_t off, const uint8_t *data, uint8_t dlen, uint8_t *reply, size_t *rlen)
{
	return write_chunk_raw(off, data, dlen, (size_t)(5u + dlen), reply, rlen);
}

static void reset_model(void)
{
	memset(g_flash, 0, sizeof(g_flash)); /* zeroed meta -> no valid record */
	g_program_calls          = 0u;
	g_program_fail           = false;
	g_erase_fail             = false;
	g_reset_calls            = 0u;
	g_fault_loop_clear_calls = 0u;

	/* Reset src/ota.c's OWN state machine too, not just the flash model.
	 * Its statics (s_state, s_erasing, s_img_len, ...) are file-scope with
	 * no seam this suite can reach directly, ZTEST_SUITE's hook slots are
	 * unused throughout this tree, and every case runs in one process --
	 * so before this, a case that called begin_session() left the machine
	 * in OTA_ST_READY for whichever case ran next.  Nothing depended on
	 * that until h_rollback gained a state guard (#147), at which point
	 * two rollback cases that issue no BEGIN of their own started failing
	 * on inherited state.  CMD_OTA_ABORT is the documented exit from any
	 * in-flight session and is exactly this seam: it clears s_erasing,
	 * sets OTA_ST_IDLE and zeroes s_err.
	 *
	 * The return value is ignored on purpose: OTA_ABORT answers
	 * STATUS_NOSUPPORT in a build where ota_fmc_supported() is false, and
	 * that is not this fixture's business. */
	uint8_t abort_reply[8];
	size_t  abort_rlen = 0u;
	(void)ota_dispatch(CMD_OTA_ABORT, NULL, 0u, abort_reply, sizeof(abort_reply), &abort_rlen);
}

/* Write a CRC-valid metadata record directly into the flash model (#3
 * tests below): bypasses ota.c's meta_commit entirely so a test can plant
 * an arbitrary (including divergent) metadata state in one step, the way
 * an aborted BEGIN + ROLLBACK + a bootloader fallback would leave one
 * behind. img_len[] is indexed OTA_SLOT_A/OTA_SLOT_B, same as the struct. */
static void write_meta_record(uint32_t       addr,
                              uint32_t       counter,
                              uint8_t        active_slot,
                              uint8_t        slot_valid,
                              const uint32_t img_len[2])
{
	ota_meta_record_t rec;
	memset(&rec, 0, sizeof(rec));
	rec.magic          = OTA_META_MAGIC;
	rec.struct_version = OTA_META_STRUCT_VER;
	rec.counter        = counter;
	rec.active_slot    = active_slot;
	rec.slot_valid     = slot_valid;
	rec.img_len[0]     = img_len[0];
	rec.img_len[1]     = img_len[1];
	rec.rec_crc32 = ota_crc32(0u, (const uint8_t *)&rec, offsetof(ota_meta_record_t, rec_crc32));
	memcpy(_host_ptr(addr), &rec, sizeof(rec));
}

/* Trial/confirm variant of write_meta_record() (bench fact 2026-09-26):
 * every other byte is planted 0xFF garbage before the named fields are
 * set, so a test asserting "flags is exactly X after some op" proves the
 * op set it explicitly rather than inheriting old bytes -- see
 * meta_commit()'s "ALWAYS set explicitly" comment in src/ota.c. */
static void write_meta_record_flags(uint32_t       addr,
                                    uint32_t       counter,
                                    uint8_t        active_slot,
                                    uint8_t        slot_valid,
                                    const uint32_t img_len[2],
                                    uint8_t        flags)
{
	ota_meta_record_t rec;
	memset(&rec, 0xFF, sizeof(rec));
	rec.magic          = OTA_META_MAGIC;
	rec.struct_version = OTA_META_STRUCT_VER;
	rec.counter        = counter;
	rec.active_slot    = active_slot;
	rec.slot_valid     = slot_valid;
	rec.flags          = flags;
	rec.img_len[0]     = img_len[0];
	rec.img_len[1]     = img_len[1];
	rec.rec_crc32 = ota_crc32(0u, (const uint8_t *)&rec, offsetof(ota_meta_record_t, rec_crc32));
	memcpy(_host_ptr(addr), &rec, sizeof(rec));
}

/* Read a metadata page directly out of the flash model (#74 tests below):
 * mirrors meta_read()'s own validity check (magic, struct_version, CRC
 * over the record up to rec_crc32) so a test can observe meta_commit's
 * actual END STATE -- which page holds what, after the dispatch call
 * returns -- instead of intercepting the erase/program calls. */
static bool read_meta_at(uint32_t addr, ota_meta_record_t *out)
{
	ota_meta_record_t rec;
	memcpy(&rec, _host_ptr(addr), sizeof(rec));
	if (rec.magic != OTA_META_MAGIC || rec.struct_version != OTA_META_STRUCT_VER) {
		return false;
	}
	if (ota_crc32(0u, (const uint8_t *)&rec, offsetof(ota_meta_record_t, rec_crc32)) !=
	    rec.rec_crc32) {
		return false;
	}
	*out = rec;
	return true;
}

/* Plant a minimal bootable image (ota_image_bootable()'s MSP+reset head)
 * directly into the flash model at `base`, with or without a trial
 * marker (bench fact 2026-09-26 follow-up) -- used by the marker-based
 * downgrade-guard tests below to control what
 * ota_image_trial_capable() (src/ota_layout.h) finds at COMMIT/ROLLBACK
 * without going through a full BEGIN/WRITE/VERIFY wire cycle every time.
 * `with_marker`=false plants no marker at all (mirrors a pre-marker
 * image); `cap_bit`=false plants a marker whose capability_flags clears
 * OTA_TRIAL_CAP_CONFIRM (mirrors a future marker version that doesn't
 * implement the confirm handshake). */
/* Marker offset for every hand-built test image below: the real floor
 * (OTA_TRIAL_MARKER_MIN_OFFSET, src/ota_layout.h) is what the scan
 * itself enforces, so tests use exactly that floor rather than the real
 * device's ~616-byte vector table -- proves the scan's boundary
 * condition without a needlessly large buffer. */
#define TEST_MARKER_OFFSET OTA_TRIAL_MARKER_MIN_OFFSET
#define TEST_IMG_LEN       96u
static void plant_trial_image(uint32_t base, bool with_marker, bool cap_bit)
{
	uint8_t buf[TEST_IMG_LEN] = { 0 };
	wr_u32(&buf[0], 0x20010000u);      /* MSP into SRAM */
	wr_u32(&buf[4], (base + 8u) | 1u); /* reset, Thumb, in-image */
	if (with_marker) {
		const uint8_t magic[16] = OTA_TRIAL_MARKER_MAGIC_BYTES;
		memcpy(&buf[TEST_MARKER_OFFSET], magic, sizeof magic);
		const uint16_t sv             = OTA_TRIAL_MARKER_STRUCT_VER;
		const uint16_t cap            = cap_bit ? OTA_TRIAL_CAP_CONFIRM : 0u;
		buf[TEST_MARKER_OFFSET + 16u] = (uint8_t)(sv & 0xFFu);
		buf[TEST_MARKER_OFFSET + 17u] = (uint8_t)(sv >> 8);
		buf[TEST_MARKER_OFFSET + 18u] = (uint8_t)(cap & 0xFFu);
		buf[TEST_MARKER_OFFSET + 19u] = (uint8_t)(cap >> 8);
	}
	zassert_true(ota_fmc_program(base, buf, sizeof buf), "plant_trial_image: program failed");
}

/* Mirrors ota.c's meta_current()/meta_pick_newest() (highest-counter valid
 * record, REC0 wins a tie) without calling into it -- both are static,
 * file-scope helpers this suite cannot link against directly.  Used by the
 * trial/confirm tests below to inspect what a COMMIT/ROLLBACK/self-heal
 * actually wrote, on whichever page meta_commit's #74 rank rule chose. */
static bool meta_current_for_test(ota_meta_record_t *out, uint32_t *which_addr)
{
	ota_meta_record_t a, b;
	const bool        va = read_meta_at(OTA_META_REC0, &a);
	const bool        vb = read_meta_at(OTA_META_REC1, &b);
	if (va && vb) {
		if (a.counter >= b.counter) {
			*out        = a;
			*which_addr = OTA_META_REC0;
		} else {
			*out        = b;
			*which_addr = OTA_META_REC1;
		}
		return true;
	}
	if (va) {
		*out        = a;
		*which_addr = OTA_META_REC0;
		return true;
	}
	if (vb) {
		*out        = b;
		*which_addr = OTA_META_REC1;
		return true;
	}
	return false;
}

ZTEST_SUITE(gd32_bridge_ota, NULL, NULL, NULL, NULL, NULL);

ZTEST(gd32_bridge_ota, test_begin_then_normal_chunk_programs)
{
	reset_model();
	begin_session(64u);
	g_program_calls = 0u;

	const uint8_t        data[4] = { 0xDE, 0xAD, 0xBE, 0xEF };
	uint8_t              reply[8];
	size_t               rlen = 0u;
	gd32_bridge_status_t st   = write_chunk(0u, data, sizeof(data), reply, &rlen);
	zassert_equal(st, STATUS_OK, "normal chunk should program, got %d", st);
	zassert_equal(g_program_calls, 1u, "ota_fmc_program must be called once");
	zassert_equal(rlen, 4u);
	zassert_equal(rd_u32(reply), 4u, "high-water = received bytes");
}

/* ---- #145: embedded WRITE_CHUNK length must match dispatch span ----- */

ZTEST(gd32_bridge_ota, test_overlong_chunk_span_rejected_without_program)
{
	reset_model();
	begin_session(64u);
	g_program_calls            = 0u;
	const uint8_t state_before = ota_state_now();

	const uint8_t        data[4] = { 0x11, 0x22, 0x33, 0x44 };
	uint8_t              reply[8];
	size_t               rlen = 0u;
	gd32_bridge_status_t st =
	    write_chunk_raw(0u, data, sizeof(data), 5u + sizeof(data) + 1u, reply, &rlen);
	zassert_equal(st, STATUS_INVAL, "zero-extended chunk span must be rejected, got %d", st);
	zassert_equal(ota_state_now(), state_before, "zero-extended chunk must leave state untouched");
	zassert_equal(g_program_calls, 0u, "zero-extended chunk must not program flash");
}

ZTEST(gd32_bridge_ota, test_truncated_chunk_span_rejected_without_program)
{
	reset_model();
	begin_session(64u);
	g_program_calls            = 0u;
	const uint8_t state_before = ota_state_now();

	const uint8_t        data[4] = { 0x11, 0x22, 0x33, 0x44 };
	uint8_t              reply[8];
	size_t               rlen = 0u;
	gd32_bridge_status_t st =
	    write_chunk_raw(0u, data, sizeof(data), 5u + sizeof(data) - 1u, reply, &rlen);
	zassert_equal(st, STATUS_INVAL, "truncated chunk span must be rejected, got %d", st);
	zassert_equal(ota_state_now(), state_before, "truncated chunk must leave state untouched");
	zassert_equal(g_program_calls, 0u, "truncated chunk must not program flash");
}

ZTEST(gd32_bridge_ota, test_zero_chunk_length_rejected_without_program)
{
	reset_model();
	begin_session(64u);
	g_program_calls            = 0u;
	const uint8_t state_before = ota_state_now();

	uint8_t              reply[8];
	size_t               rlen = 0u;
	gd32_bridge_status_t st   = write_chunk_raw(0u, NULL, 0u, 5u, reply, &rlen);
	zassert_equal(st, STATUS_INVAL, "zero-length chunk must be rejected, got %d", st);
	zassert_equal(ota_state_now(), state_before, "zero-length chunk must leave state untouched");
	zassert_equal(g_program_calls, 0u, "zero-length chunk must not program flash");
}

ZTEST(gd32_bridge_ota, test_wrapped_offset_rejected_without_program)
{
	reset_model();
	begin_session(64u);
	g_program_calls = 0u;

	const uint8_t data[4] = { 0x5A, 0x5B, 0x5C, 0x5D };
	uint8_t       reply[8];
	size_t        rlen = 0u;
	/* off = UINT32_MAX - 1.  On a 32-bit target the old `off + dlen >
     * OTA_SLOT_SIZE` check wraps and wrongly accepts this; the
     * subtraction-style check rejects any off past the slot outright.  This
     * asserts the OUT_OF_RANGE contract and that no flash program happens. */
	gd32_bridge_status_t st = write_chunk(0xFFFFFFFEu, data, sizeof(data), reply, &rlen);
	zassert_equal(st, STATUS_OUT_OF_RANGE, "wrapped offset must be rejected, got %d", st);
	zassert_equal(g_program_calls, 0u, "ota_fmc_program must NOT be called on OOR");
}

ZTEST(gd32_bridge_ota, test_offset_past_slot_rejected_without_program)
{
	reset_model();
	begin_session(OTA_SLOT_SIZE);
	g_program_calls = 0u;

	const uint8_t data[4] = { 1, 2, 3, 4 };
	uint8_t       reply[8];
	size_t        rlen = 0u;
	/* Non-wrapping but out of range: off within slot, off + dlen past end. */
	gd32_bridge_status_t st = write_chunk(OTA_SLOT_SIZE - 2u, data, sizeof(data), reply, &rlen);
	zassert_equal(st, STATUS_OUT_OF_RANGE, "off+len past slot must be rejected, got %d", st);
	zassert_equal(g_program_calls, 0u);
}

ZTEST(gd32_bridge_ota, test_exact_end_chunk_accepted)
{
	reset_model();
	begin_session(OTA_SLOT_SIZE);
	g_program_calls = 0u;

	const uint8_t data[4] = { 1, 2, 3, 4 };
	uint8_t       reply[8];
	size_t        rlen = 0u;
	/* off + dlen == OTA_SLOT_SIZE is the last legal byte range. */
	gd32_bridge_status_t st = write_chunk(OTA_SLOT_SIZE - 4u, data, sizeof(data), reply, &rlen);
	zassert_equal(st, STATUS_OK, "exact-end chunk must be accepted, got %d", st);
	zassert_equal(g_program_calls, 1u);
}

ZTEST(gd32_bridge_ota, test_replay_is_idempotent)
{
	reset_model();
	begin_session(64u);

	const uint8_t data[4] = { 0x11, 0x22, 0x33, 0x44 };
	uint8_t       reply[8];
	size_t        rlen = 0u;

	g_program_calls = 0u;
	zassert_equal(write_chunk(0u, data, sizeof(data), reply, &rlen), STATUS_OK);
	zassert_equal(g_program_calls, 1u);

	/* Re-issue the identical chunk: below the high-water mark and byte
     * identical -> acked without re-programming (ECC re-write hard-faults). */
	g_program_calls = 0u;
	zassert_equal(write_chunk(0u, data, sizeof(data), reply, &rlen), STATUS_OK);
	zassert_equal(g_program_calls, 0u, "replayed chunk must not re-program");
}

/* #741: ota_slot_base_checked validates the slot instead of silently
 * mapping every non-B value to slot A. */
ZTEST(gd32_bridge_ota, test_slot_base_checked_validates_slot)
{
	uint32_t base = 0xDEADBEEFu;

	zassert_true(ota_slot_base_checked(OTA_SLOT_A, &base));
	zassert_equal(base, OTA_SLOT_A_BASE);
	zassert_true(ota_slot_base_checked(OTA_SLOT_B, &base));
	zassert_equal(base, OTA_SLOT_B_BASE);

	/* Representative invalid slots must be REJECTED, not resolved to a
	 * valid flash address -- the #741 defect. */
	base = 0xDEADBEEFu;
	zassert_false(ota_slot_base_checked(2u, &base), "slot 2 must reject, not map to A");
	zassert_false(ota_slot_base_checked(0xFFu, &base));
	zassert_equal(base, 0xDEADBEEFu, "base must be untouched on reject");
}

/* #755: ota_image_bootable rejects CRC-valid-but-unbootable images. */
static void put32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

ZTEST(gd32_bridge_ota, test_image_bootable_validates_vector_head)
{
	const uint32_t base    = OTA_SLOT_A_BASE;
	const uint32_t len     = 0x200u;
	uint8_t        img[16] = { 0 };

	/* Valid: MSP in SRAM + word-aligned, reset inside image + Thumb. */
	put32(&img[0], 0x20010000u);          /* MSP  */
	put32(&img[4], (base + 0x100u) | 1u); /* reset, Thumb */
	zassert_true(ota_image_bootable(base, img, len), "valid vector head must pass");

	/* One-byte / truncated image (below the MSP+reset head). */
	zassert_false(ota_image_bootable(base, img, 1u), "one-byte image must reject (#755)");
	zassert_false(ota_image_bootable(base, img, 7u), "truncated vector head must reject");

	/* MSP not word-aligned / outside SRAM. */
	put32(&img[0], 0x20010001u);
	zassert_false(ota_image_bootable(base, img, len), "unaligned MSP must reject");
	put32(&img[0], 0x08000000u);
	zassert_false(ota_image_bootable(base, img, len), "MSP outside SRAM must reject");
	put32(&img[0], 0x20010000u); /* restore */

	/* Reset vector without the Thumb bit, or outside the image. */
	put32(&img[4], base + 0x100u); /* even -> no Thumb */
	zassert_false(ota_image_bootable(base, img, len), "reset without Thumb bit must reject");
	put32(&img[4], (base + len) | 1u); /* past end */
	zassert_false(ota_image_bootable(base, img, len), "reset past image end must reject");
	put32(&img[4], (base - 4u) | 1u); /* before base */
	zassert_false(ota_image_bootable(base, img, len), "reset before image base must reject");
}

/* ---- #770: BEGIN arms a background erase, acks immediately ---------- */

/* BEGIN must NOT erase the slot inline (that stalled the SPI reply ~1 s and
 * hung the host's ota_begin -- #770).  It acks at once with state BUSY;
 * ota_erase_tick() drains the erase to READY; chunks are rejected until
 * then. */
ZTEST(gd32_bridge_ota, test_begin_arms_background_erase)
{
	reset_model();

	uint8_t req[8];
	wr_u32(&req[0], 64u); /* img_len */
	wr_u32(&req[4], 0u);  /* crc */
	uint8_t reply[8];
	size_t  rlen = 0u;
	/* BEGIN acks immediately -- no inline whole-slot erase. */
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof(req), reply, sizeof(reply), &rlen),
	              STATUS_OK);
	zassert_equal(ota_state_now(), 2u /* OTA_ST_BUSY */, "BEGIN must leave state BUSY (erasing)");

	/* A chunk before the erase finishes is refused (state not READY). */
	const uint8_t data[8] = { 1, 2, 3, 4, 5, 6, 7, 8 };
	uint8_t       wr[8];
	size_t        wrl = 0u;
	zassert_equal(write_chunk(0u, data, sizeof(data), wr, &wrl),
	              STATUS_NOT_READY,
	              "chunk before erase completes must be rejected");

	/* Pump the erase the way bridge_hw_tick would; state flips to READY. */
	for (unsigned i = 0u; i < (OTA_SLOT_SIZE / OTA_PAGE_SIZE) + 4u; ++i) {
		ota_erase_tick();
	}
	zassert_equal(ota_state_now(), 1u /* OTA_ST_READY */, "erase drain must reach READY");

	/* Now a chunk is accepted. */
	zassert_equal(
	    write_chunk(0u, data, sizeof(data), wr, &wrl), STATUS_OK, "chunk after READY must program");
}

/* ---- #733: on-flash layout / byte-representation guard --------------- */

/* The bootloader byte-copies a flash record and CRCs the raw bytes, so
 * ota_meta_record_t's in-memory image IS the on-flash format.  The
 * _Static_asserts in ota_layout.h pin size + offsets at compile time;
 * this test documents the intended LITTLE-ENDIAN byte representation and
 * proves the host toolchain lays it out the same way. */
ZTEST(gd32_bridge_ota, test_meta_record_layout_bytes)
{
	zassert_equal(sizeof(ota_meta_record_t), 44u, "record must be 44 bytes on flash");

	ota_meta_record_t m;
	memset(&m, 0, sizeof(m));
	m.magic          = 0x11223344u;
	m.struct_version = OTA_META_STRUCT_VER;
	m.counter        = 0xA1B2C3D4u;
	m.active_slot    = OTA_SLOT_B;
	m.slot_valid     = 0x03u;
	m.fw_version[0]  = 0x00010203u;
	m.img_len[1]     = 0x0000B000u;
	m.img_crc32[0]   = 0xDEADBEEFu;
	m.rec_crc32      = 0xFEEDFACEu;

	const uint8_t *b = (const uint8_t *)&m;
	zassert_equal(rd_u32(&b[0]), 0x11223344u, "magic @0");
	zassert_equal(rd_u32(&b[4]), OTA_META_STRUCT_VER, "struct_version @4");
	zassert_equal(rd_u32(&b[8]), 0xA1B2C3D4u, "counter @8");
	zassert_equal(b[12], (uint8_t)OTA_SLOT_B, "active_slot @12");
	zassert_equal(b[13], 0x03u, "slot_valid @13");
	zassert_equal(rd_u32(&b[16]), 0x00010203u, "fw_version[0] @16");
	zassert_equal(rd_u32(&b[28]), 0x0000B000u, "img_len[1] @28");
	zassert_equal(rd_u32(&b[32]), 0xDEADBEEFu, "img_crc32[0] @32");
	zassert_equal(rd_u32(&b[40]), 0xFEEDFACEu, "rec_crc32 @40");

	/* The CRC span the bootloader/app compute is everything up to
	 * rec_crc32; the offset is the documented span length. */
	zassert_equal(offsetof(ota_meta_record_t, rec_crc32), 40u, "CRC span = 40 bytes");
}

/* ---- #3: OTA_BEGIN must not erase the running slot on metadata/boot
 * divergence; GET_STATE must report the build-derived running slot;
 * ROLLBACK must keep using metadata -------------------------------- */

/* The five-step brick sequence from the issue, reproduced end-to-end:
 * BEGIN -> abort mid-erase -> ROLLBACK -> the bootloader's newest-first
 * fallback (#754, modelled by the metadata state ROLLBACK leaves behind,
 * NOT by re-running boot_main.c -- that file is out of scope) -> BEGIN
 * again.  Before the fix, the second BEGIN inverted metadata's
 * (divergent) active_slot and armed the erase against TEST_RUNNING_SLOT
 * -- the live image, vector table first. */
ZTEST(gd32_bridge_ota, test_divergence_second_begin_targets_non_running_slot)
{
	reset_model();

	uint32_t running_base = 0u, other_base = 0u;
	zassert_true(ota_slot_base_checked(TEST_RUNNING_SLOT, &running_base));
	zassert_true(ota_slot_base_checked(TEST_OTHER_SLOT, &other_base));

	/* Canary: stands in for the live vector table at the head of the
	 * slot THIS build runs from.  If BEGIN's erase ever reaches it, the
	 * test fails here -- the exact mechanism from #3. */
	uint8_t canary[16];
	memset(canary, 0xAAu, sizeof(canary));
	memcpy(_host_ptr(running_base), canary, sizeof(canary));

	/* Step 1: initial metadata, no divergence yet -- newest (only) record
	 * names the slot THIS build actually runs from. */
	const uint32_t len0[2] = { 4096u, 4096u };
	write_meta_record(OTA_META_REC0, 5u, TEST_RUNNING_SLOT, 0x03u, len0);

	/* Step 2: BEGIN (correctly targets the non-running slot -- sanity
	 * check on the harness, no divergence yet), then abandoned mid-erase
	 * (power loss / host abort).  slot_valid is left set: the erase-time
	 * clearing path is a separate, deferred slice (see the task notes;
	 * it would add a bank0 write per BEGIN and #37's read-while-write
	 * hazard is unmitigated), so this precondition is exactly what a
	 * real aborted BEGIN leaves behind today. */
	uint8_t req[8];
	uint8_t reply[8];
	size_t  rlen = 0u;
	wr_u32(&req[0], 4096u);
	wr_u32(&req[4], 0u);
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof(req), reply, sizeof(reply), &rlen),
	              STATUS_OK);
	zassert_equal(
	    reply[2], TEST_OTHER_SLOT, "pre-divergence BEGIN must target the non-running slot");
	ota_erase_tick();
	ota_erase_tick(); /* mid-erase, not drained to completion */
	zassert_equal(ota_dispatch(CMD_OTA_ABORT, NULL, 0u, reply, sizeof(reply), &rlen), STATUS_OK);

	/* Step 3: ROLLBACK succeeds (slot_valid/img_len for the other slot
	 * still assert valid) and commits a NEW highest-counter record with
	 * active_slot = TEST_OTHER_SLOT. */
	zassert_equal(ota_dispatch(CMD_OTA_ROLLBACK, NULL, 0u, reply, sizeof(reply), &rlen), STATUS_OK);

	/* Step 4: models the bootloader's fallback (boot_main.c:117-124,
	 * #754) rejecting TEST_OTHER_SLOT's (erased/invalid) image and
	 * booting the older record's slot instead -- i.e. this build,
	 * TEST_RUNNING_SLOT, keeps running while the newest valid metadata
	 * (committed in step 3) still names TEST_OTHER_SLOT.  That divergence
	 * is already in place; no further setup is needed. */

	/* Step 5: the assertion that matters.  Pre-fix, this inverted
	 * metadata's active_slot (TEST_OTHER_SLOT) to get TEST_RUNNING_SLOT
	 * -- the live slot -- as the erase target. */
	wr_u32(&req[0], 4096u);
	wr_u32(&req[4], 0u);
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof(req), reply, sizeof(reply), &rlen),
	              STATUS_OK);
	zassert_equal(
	    reply[2], TEST_OTHER_SLOT, "post-divergence BEGIN must still target the non-running slot");
	zassert_false(reply[2] == TEST_RUNNING_SLOT, "BEGIN must never target the running slot");

	/* Drain the erase and prove the running slot's canary was never
	 * touched, while the OTHER slot really was erased (0xFF) -- proving
	 * the erase ran, and against the correct target. */
	for (unsigned i = 0u; i < (OTA_SLOT_SIZE / OTA_PAGE_SIZE) + 4u; ++i) {
		ota_erase_tick();
	}
	zassert_mem_equal(_host_ptr(running_base),
	                  canary,
	                  sizeof(canary),
	                  "erase touched the running slot's live bytes");
	zassert_equal(((const uint8_t *)_host_ptr(other_base))[0],
	              0xFFu,
	              "erase must actually run, against the non-running slot");
}

/* Same setup as the divergence test above (metadata names TEST_OTHER_SLOT
 * active while this build runs TEST_RUNNING_SLOT), but proves a NARROWER
 * property than the name alone suggests -- correct that claim rather than
 * just re-stating it (#3 review).  The `reply[2]` equality below is what
 * actually kills the headline P1-reversion mutation (deleting
 * OTA_RUNNING_SLOT and going back to inverting active_slot_now()): that
 * mutation makes `reply[2]` wrong and the test dies there, so the canary
 * below is NEVER evaluated for it.  The `zassert_mem_equal` canary check
 * is load-bearing only for a NARROWER defect class where the erase range
 * (s_erase_at / s_erase_end) gets decoupled from the reported
 * `target_slot` -- e.g. a mutation that forces s_erase_at to the running
 * slot's base while leaving reply[2] correct (mutation m6) -- which is
 * the shape that would actually walk the running slot's flash while
 * telling the host it targeted the other one. */
ZTEST(gd32_bridge_ota, test_erase_range_never_intersects_running_slot)
{
	reset_model();

	uint32_t running_base = 0u;
	zassert_true(ota_slot_base_checked(TEST_RUNNING_SLOT, &running_base));
	uint8_t canary[16];
	memset(canary, 0x55u, sizeof(canary));
	memcpy(_host_ptr(running_base), canary, sizeof(canary));

	/* Highest-counter (only) record names TEST_OTHER_SLOT active -- the
	 * shape that, inverted, used to select TEST_RUNNING_SLOT. */
	const uint32_t len[2] = { 4096u, 4096u };
	write_meta_record(OTA_META_REC0, 9u, TEST_OTHER_SLOT, 0x03u, len);

	uint8_t req[8];
	uint8_t reply[8];
	size_t  rlen = 0u;
	wr_u32(&req[0], 4096u);
	wr_u32(&req[4], 0u);
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof(req), reply, sizeof(reply), &rlen),
	              STATUS_OK);
	zassert_equal(reply[2], TEST_OTHER_SLOT);

	for (unsigned i = 0u; i < (OTA_SLOT_SIZE / OTA_PAGE_SIZE) + 4u; ++i) {
		ota_erase_tick();
	}
	zassert_mem_equal(_host_ptr(running_base),
	                  canary,
	                  sizeof(canary),
	                  "s_erasing's range intersected the running slot");
}

/* #3: with NO metadata planted at all (factory-fresh / fully-erased
 * flash), meta_current() returns false.  Pre-fix, active_slot_now() fell
 * back to its silent OTA_SLOT_A default on that failure, so the old
 * `s_inactive = (active_slot_now() == OTA_SLOT_A) ? OTA_SLOT_B : OTA_SLOT_A`
 * resolved to OTA_SLOT_B unconditionally -- in a slot-B-resident build
 * that IS the running slot, with no abort/rollback/bootloader-fallback
 * narrative needed to reach it: a factory-fresh slot-B board bricks
 * itself on its very first OTA_BEGIN.  Mutation m2 (restoring that
 * pre-fix path) leaves every other zero-metadata case in this suite
 * green while this one targets the running slot. */
ZTEST(gd32_bridge_ota, test_begin_with_no_metadata_targets_non_running_slot)
{
	reset_model(); /* zeroed flash: meta_current() has nothing valid to read */

	uint8_t req[8];
	uint8_t reply[8];
	size_t  rlen = 0u;
	wr_u32(&req[0], 4096u);
	wr_u32(&req[4], 0u);
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof(req), reply, sizeof(reply), &rlen),
	              STATUS_OK);
	zassert_equal(reply[2],
	              TEST_OTHER_SLOT,
	              "BEGIN with no metadata at all must still target the non-running slot");
}

/* h_get_state's `active` byte must report the build-derived running slot
 * (OTA_RUNNING_SLOT), NOT metadata's active_slot, once metadata and the
 * running slot diverge -- this is what lets the host OBSERVE the
 * divergence instead of being told a comforting falsehood. */
ZTEST(gd32_bridge_ota, test_get_state_active_reports_running_slot_not_metadata)
{
	reset_model();

	const uint32_t len[2] = { 4096u, 4096u };
	write_meta_record(OTA_META_REC0, 3u, TEST_OTHER_SLOT, 0x03u, len);

	uint8_t reply[8] = { 0 };
	size_t  rlen     = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_GET_STATE, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_OK);
	zassert_equal(reply[1],
	              TEST_RUNNING_SLOT,
	              "GET_STATE.active must report the build-derived running slot, not metadata's");
}

/* h_rollback must NOT be broken by the P1/P2 split: it must keep using
 * the METADATA answer for "which slot is active", not the build-derived
 * OTA_RUNNING_SLOT -- ROLLBACK operates on the metadata state machine
 * itself and must read the source of truth it is about to write. */
ZTEST(gd32_bridge_ota, test_rollback_still_uses_metadata_active_slot)
{
	reset_model();

	/* Metadata names TEST_OTHER_SLOT active -- independent of which slot
	 * this build runs from.  If ROLLBACK had been switched to
	 * OTA_RUNNING_SLOT it would flip to TEST_OTHER_SLOT here (the SAME
	 * slot, a no-op / wrong target); reading metadata correctly flips to
	 * TEST_RUNNING_SLOT (metadata's "other" slot). */
	const uint32_t len[2] = { 4096u, 4096u };
	write_meta_record(OTA_META_REC0, 1u, TEST_OTHER_SLOT, 0x03u, len);

	uint8_t reply[8] = { 0 };
	size_t  rlen     = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_ROLLBACK, NULL, 0u, reply, sizeof(reply), &rlen), STATUS_OK);

	/* meta_commit alternates pages; REC0 was pre-populated directly above
	 * (bypassing meta_commit), so the committed record lands on REC1.
	 * Read it back off the flash model directly -- NOT via GET_STATE,
	 * which (per P2, this same change) now reports OTA_RUNNING_SLOT and
	 * would test the wrong thing here. */
	ota_meta_record_t rec1;
	memcpy(&rec1, _host_ptr(OTA_META_REC1), sizeof(rec1));
	zassert_equal(rec1.magic, OTA_META_MAGIC, "ROLLBACK must commit to REC1");
	zassert_equal(rec1.counter, 2u);
	zassert_equal(rec1.active_slot,
	              TEST_RUNNING_SLOT,
	              "ROLLBACK must flip metadata's active_slot, not OTA_RUNNING_SLOT's");
}

/* ---- #74: meta_commit must preserve the record that BOOTS THE PART, not
 * just the highest counter -- see the block comment above meta_commit in
 * ota.c for the full rationale ------------------------------------------ */

/* Drive a session to VERIFIED (BEGIN -> WRITE_CHUNK -> VERIFY) with a
 * minimal bootable image, so a #74 case below can dispatch CMD_OTA_COMMIT
 * itself and observe meta_commit through the real h_commit path -- not a
 * parallel harness.  BEGIN/WRITE/VERIFY never touch the metadata pages
 * (only the image slot), so a case may plant arbitrary metadata via
 * write_meta_record() BEFORE calling this and have it survive intact up
 * to the COMMIT dispatch the case makes afterwards.
 *
 * The staged image carries a confirm-capable trial marker (policy closed
 * on PR #246, src/ota.c's h_commit(): a markerless image is now REFUSED
 * at COMMIT, not committed CONFIRMED) -- the #74 page-selection cases
 * this feeds are about meta_commit()'s rank rule, independent of the
 * trial-marker downgrade guard, so they need COMMIT to actually reach
 * meta_commit() rather than being refused before it. */
static void drive_to_verified(void)
{
	uint32_t other_base = 0u;
	zassert_true(ota_slot_base_checked(TEST_OTHER_SLOT, &other_base),
	             "TEST_OTHER_SLOT must resolve to a flash base");

	/* MSP into SRAM, reset vector (Thumb bit set) at the slot's own base
	 * -- satisfies ota_image_bootable() (#755) so COMMIT's bootability
	 * guard doesn't itself reject the session before reaching
	 * meta_commit. */
	uint8_t img[TEST_IMG_LEN] = { 0 };
	put32(&img[0], 0x20010000u);
	put32(&img[4], other_base | 1u);
	const uint8_t  magic[16] = OTA_TRIAL_MARKER_MAGIC_BYTES;
	const uint16_t sv        = OTA_TRIAL_MARKER_STRUCT_VER;
	const uint16_t cap       = OTA_TRIAL_CAP_CONFIRM;
	memcpy(&img[TEST_MARKER_OFFSET], magic, sizeof magic);
	img[TEST_MARKER_OFFSET + 16u] = (uint8_t)(sv & 0xFFu);
	img[TEST_MARKER_OFFSET + 17u] = (uint8_t)(sv >> 8);
	img[TEST_MARKER_OFFSET + 18u] = (uint8_t)(cap & 0xFFu);
	img[TEST_MARKER_OFFSET + 19u] = (uint8_t)(cap >> 8);
	const uint32_t img_len = (uint32_t)sizeof(img);
	const uint32_t crc     = ota_crc32(0u, img, img_len);

	uint8_t req[8];
	wr_u32(&req[0], img_len);
	wr_u32(&req[4], crc);
	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof(req), reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "drive_to_verified: BEGIN must succeed");
	for (unsigned i = 0u; i < (OTA_SLOT_SIZE / OTA_PAGE_SIZE) + 4u; ++i) {
		ota_erase_tick();
	}

	uint8_t wr[8];
	size_t  wrl = 0u;
	zassert_equal(write_chunk(0u, img, (uint8_t)img_len, wr, &wrl),
	              STATUS_OK,
	              "drive_to_verified: WRITE_CHUNK must succeed");

	zassert_equal(ota_dispatch(CMD_OTA_VERIFY, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "drive_to_verified: VERIFY dispatch must return STATUS_OK");
	zassert_equal(reply[4], 1u, "drive_to_verified: image CRC must verify");
}

/* Case 1: divergent, newest on REC0.  REC0 (counter 9) names
 * TEST_OTHER_SLOT; REC1 (counter 8, older) names TEST_RUNNING_SLOT -- the
 * shape the bootloader's newest-first fallback (boot_main.c:117-124,
 * #754) leaves behind when REC0's slot fails validation and it boots
 * REC1's slot instead.  meta_commit must preserve REC1 (it is what is
 * keeping the part alive), not the higher counter.  This FAILS before the
 * #74 fix: the old rule erases the non-newest page (REC1) outright. */
ZTEST(gd32_bridge_ota, test_meta_commit_preserves_running_slot_record_on_rec0)
{
	reset_model();

	/* Distinguishable per-page descriptor tables (review #1): planting the
	 * SAME img_len[]/slot_valid on both pages cannot tell "carried forward
	 * from REC0 (the NEWEST record)" apart from "carried forward from
	 * REC1 (the erase survivor)" -- exactly the property that became new
	 * when the erase target could be the newest page, and exactly what
	 * this test exists to cover. */
	uint32_t rec0_len[2];
	rec0_len[TEST_RUNNING_SLOT] = 0x1000u;
	rec0_len[TEST_OTHER_SLOT]   = 0x2000u;
	uint32_t rec1_len[2];
	rec1_len[TEST_RUNNING_SLOT] = 0x3000u;
	rec1_len[TEST_OTHER_SLOT]   = 0x4000u;
	write_meta_record(OTA_META_REC0, 9u, TEST_OTHER_SLOT, 0x03u, rec0_len);
	write_meta_record(OTA_META_REC1, 8u, TEST_RUNNING_SLOT, 0x01u, rec1_len);

	drive_to_verified();
	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_COMMIT, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "COMMIT must succeed");

	ota_meta_record_t rec1;
	zassert_true(read_meta_at(OTA_META_REC1, &rec1), "REC1 must remain CRC-valid, untouched");
	zassert_equal(rec1.counter, 8u, "REC1's counter must be untouched");
	zassert_equal(rec1.active_slot,
	              TEST_RUNNING_SLOT,
	              "REC1 must still name the running slot -- it must not be the erase target");

	ota_meta_record_t rec0;
	zassert_true(read_meta_at(OTA_META_REC0, &rec0), "REC0 must hold the freshly committed record");
	zassert_equal(rec0.counter, 10u, "REC0's new record must carry counter = old max (9) + 1");
	/* The descriptor table must carry forward from REC0 (the NEWEST
	 * record), not from REC1 (the erase survivor) -- a mutant that reads
	 * img_len[]/slot_valid off the survivor page instead of the newest
	 * one must die here, even though it keeps the counter correct. */
	zassert_equal(rec0.img_len[TEST_RUNNING_SLOT],
	              0x1000u,
	              "committed record's img_len[TEST_RUNNING_SLOT] must carry from REC0 (the "
	              "newest), not REC1 (the survivor)");
	zassert_equal(rec0.slot_valid,
	              (uint8_t)(0x03u | (1u << TEST_OTHER_SLOT)),
	              "committed record's slot_valid must carry from REC0 (the newest), not REC1");
}

/* Case 2: mirror of case 1 -- the newest (divergent) record sits on REC1
 * instead of REC0.  Same property, opposite pages: REC0 (counter 8) names
 * TEST_RUNNING_SLOT and must survive; REC1 (counter 9) names
 * TEST_OTHER_SLOT and is the correct erase target. */
ZTEST(gd32_bridge_ota, test_meta_commit_preserves_running_slot_record_on_rec1)
{
	reset_model();

	const uint32_t len[2] = { 4096u, 4096u };
	write_meta_record(OTA_META_REC1, 9u, TEST_OTHER_SLOT, 0x03u, len);
	write_meta_record(OTA_META_REC0, 8u, TEST_RUNNING_SLOT, 0x03u, len);

	drive_to_verified();
	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_COMMIT, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "COMMIT must succeed");

	ota_meta_record_t rec0;
	zassert_true(read_meta_at(OTA_META_REC0, &rec0), "REC0 must remain CRC-valid, untouched");
	zassert_equal(rec0.counter, 8u, "REC0's counter must be untouched");
	zassert_equal(rec0.active_slot,
	              TEST_RUNNING_SLOT,
	              "REC0 must still name the running slot -- it must not be the erase target");

	ota_meta_record_t rec1;
	zassert_true(read_meta_at(OTA_META_REC1, &rec1), "REC1 must hold the freshly committed record");
	zassert_equal(rec1.counter, 10u, "REC1's new record must carry counter = old max (9) + 1");
}

/* Case 3: the actual brick #74 is about.  Same divergent state as case 1,
 * but the program step after the erase is made to fail (a modelled power
 * cut).  The page naming TEST_RUNNING_SLOT must still hold a CRC-valid
 * record afterwards -- if the erase target had been the running-slot page
 * instead, this is exactly the state that parks the part in
 * `for (;;) { __WFI(); }` with no over-the-wire recovery. */
ZTEST(gd32_bridge_ota, test_meta_commit_power_cut_leaves_running_slot_record_intact)
{
	reset_model();

	const uint32_t len[2] = { 4096u, 4096u };
	write_meta_record(OTA_META_REC0, 9u, TEST_OTHER_SLOT, 0x03u, len);
	write_meta_record(OTA_META_REC1, 8u, TEST_RUNNING_SLOT, 0x03u, len);

	drive_to_verified();
	g_program_fail = true;
	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_COMMIT, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_IO,
	              "COMMIT must report STATUS_IO on the PGERR-with-power-on path modelled by "
	              "g_program_fail (a real power cut reaches the host with no status at all, "
	              "but leaves the same flash end state -- see g_program_fail's comment)");

	ota_meta_record_t rec1;
	zassert_true(read_meta_at(OTA_META_REC1, &rec1),
	             "a CRC-valid record naming the running slot must survive the power cut");
	zassert_equal(
	    rec1.active_slot, TEST_RUNNING_SLOT, "the surviving record must name the running slot");

	ota_meta_record_t rec0_scratch;
	zassert_false(read_meta_at(OTA_META_REC0, &rec0_scratch),
	              "REC0 must have been erased (and left unprogrammed) by the simulated power cut");

	g_program_fail = false; /* hygiene: no before-hook clears this (#6) */
}

/* Case 4: normal alternation, unchanged.  Both records name
 * TEST_RUNNING_SLOT (equal rank) with different counters -- the
 * lower-counter page must still be the one rewritten, identical to
 * pre-#74 behaviour, since the tie-break preserves the newest. */
ZTEST(gd32_bridge_ota, test_meta_commit_tie_break_preserves_newest_both_running)
{
	reset_model();

	const uint32_t len[2] = { 4096u, 4096u };
	write_meta_record(OTA_META_REC0, 5u, TEST_RUNNING_SLOT, 0x03u, len);
	write_meta_record(OTA_META_REC1, 3u, TEST_RUNNING_SLOT, 0x03u, len);

	drive_to_verified();
	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_COMMIT, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "COMMIT must succeed");

	ota_meta_record_t rec0;
	zassert_true(read_meta_at(OTA_META_REC0, &rec0),
	             "REC0 (higher counter) must survive untouched");
	zassert_equal(rec0.counter, 5u, "REC0's counter must be untouched");

	ota_meta_record_t rec1;
	zassert_true(read_meta_at(OTA_META_REC1, &rec1), "REC1 must hold the freshly committed record");
	zassert_equal(rec1.counter, 6u, "REC1 (lower counter) must be the one rewritten");
}

/* Case 5: neither record names the running slot (both name
 * TEST_OTHER_SLOT, equal rank) -- the deliberate degradation to today's
 * preserve-newest behaviour when the running-slot proxy can't
 * distinguish the pages. */
ZTEST(gd32_bridge_ota, test_meta_commit_tie_break_preserves_newest_neither_running)
{
	reset_model();

	const uint32_t len[2] = { 4096u, 4096u };
	write_meta_record(OTA_META_REC0, 5u, TEST_OTHER_SLOT, 0x03u, len);
	write_meta_record(OTA_META_REC1, 3u, TEST_OTHER_SLOT, 0x03u, len);

	drive_to_verified();
	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_COMMIT, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "COMMIT must succeed");

	ota_meta_record_t rec0;
	zassert_true(read_meta_at(OTA_META_REC0, &rec0),
	             "REC0 (higher counter) must survive untouched");
	zassert_equal(rec0.counter, 5u, "REC0's counter must be untouched");

	ota_meta_record_t rec1;
	zassert_true(read_meta_at(OTA_META_REC1, &rec1), "REC1 must hold the freshly committed record");
	zassert_equal(rec1.counter, 6u, "REC1 (lower counter) must be the one rewritten");
}

/* Case 5b: EQUAL counters on both pages, same rank.  meta_commit can
 * never produce this itself (a new record is always max + 1), so it takes
 * a hand-flashed or externally-provisioned part to reach -- but it is the
 * one input that pins the `>=` in both meta_pick_newest() and the rank
 * tie-break, which decide REC0 wins a tie.  Without this case a `>=`-to-`>`
 * mutant in either place survives the whole suite, and the claim that the
 * tie-break reproduces pre-#74 selection bit for bit rests on reading the
 * code rather than on running it. */
ZTEST(gd32_bridge_ota, test_meta_commit_equal_counters_preserve_rec0)
{
	reset_model();

	/* The two records must differ in a CARRIED-FORWARD field, not just in
	 * which page they sit on.  With identical descriptors, picking REC1
	 * over REC0 yields a byte-identical record and the tie is
	 * unobservable -- a `>=`-to-`>` mutant in meta_pick_newest() then
	 * survives even though this case exists.  Distinct img_len[] makes
	 * the pick visible in the committed record. */
	uint32_t len0[2];
	uint32_t len1[2];
	len0[TEST_RUNNING_SLOT] = 0x1000u;
	len0[TEST_OTHER_SLOT]   = 0x2000u;
	len1[TEST_RUNNING_SLOT] = 0x3000u;
	len1[TEST_OTHER_SLOT]   = 0x4000u;
	write_meta_record(OTA_META_REC0, 7u, TEST_RUNNING_SLOT, 0x03u, len0);
	write_meta_record(OTA_META_REC1, 7u, TEST_RUNNING_SLOT, 0x03u, len1);

	drive_to_verified();
	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_COMMIT, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "COMMIT must succeed");

	ota_meta_record_t rec0;
	zassert_true(read_meta_at(OTA_META_REC0, &rec0), "REC0 must survive an equal-counter tie");
	zassert_equal(rec0.counter, 7u, "REC0's counter must be untouched -- REC0 wins the tie");

	ota_meta_record_t rec1;
	zassert_true(read_meta_at(OTA_META_REC1, &rec1), "REC1 must hold the freshly committed record");
	zassert_equal(rec1.counter, 8u, "REC1 must be the erase target on an equal-counter tie");
	zassert_equal(rec1.img_len[TEST_RUNNING_SLOT],
	              0x1000u,
	              "the committed record must carry REC0's descriptors -- REC0 wins the tie in "
	              "meta_pick_newest() too, not just in the erase-target choice");
}

/* Case 6: exactly one valid record (REC0 planted, REC1 left as
 * reset_model()'s zeroed/blank flash -- no valid record).  The invalid
 * page must be the erase target regardless of the valid page's rank. */
ZTEST(gd32_bridge_ota, test_meta_commit_targets_the_only_invalid_page)
{
	reset_model();

	const uint32_t len[2] = { 4096u, 4096u };
	write_meta_record(OTA_META_REC0, 1u, TEST_OTHER_SLOT, 0x03u, len);
	/* REC1 intentionally left unwritten: reset_model()'s zeroed flash has
	 * no valid magic/struct_version there. */

	drive_to_verified();
	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_COMMIT, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "COMMIT must succeed");

	ota_meta_record_t rec0;
	zassert_true(read_meta_at(OTA_META_REC0, &rec0),
	             "REC0 (the only valid page) must survive untouched");
	zassert_equal(rec0.counter, 1u, "REC0's counter must be untouched");

	ota_meta_record_t rec1;
	zassert_true(read_meta_at(OTA_META_REC1, &rec1),
	             "REC1 (the invalid page) must hold the freshly committed record");
	zassert_equal(rec1.counter, 2u, "REC1's new record must carry counter = old max (1) + 1");
}

/* Case 7: neither record is valid (factory-fresh / fully-erased flash).
 * meta_commit must fall back to today's initialiser, OTA_META_REC0. */
ZTEST(gd32_bridge_ota, test_meta_commit_targets_rec0_with_no_metadata)
{
	reset_model(); /* zeroed flash: neither page has a valid record */

	drive_to_verified();
	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_COMMIT, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "COMMIT must succeed");

	ota_meta_record_t rec0;
	zassert_true(read_meta_at(OTA_META_REC0, &rec0),
	             "with no prior metadata, REC0 must be the (default) target");
	zassert_equal(rec0.counter, 1u, "the first-ever record must carry counter 1");
}

/* Case 8: the ROLLBACK path in the divergent state.  Same metadata as
 * case 1, but driven through CMD_OTA_ROLLBACK (h_rollback) rather than
 * CMD_OTA_COMMIT -- h_rollback reaches meta_commit too, and is the more
 * likely real-world trigger for this state (see the narrative in
 * test_divergence_second_begin_targets_non_running_slot above).  The same
 * page must survive as in case 1. */
ZTEST(gd32_bridge_ota, test_meta_commit_rollback_preserves_running_slot_record)
{
	reset_model();

	/* Same distinguishable-descriptor rationale as
	 * test_meta_commit_preserves_running_slot_record_on_rec0 (review #1). */
	uint32_t rec0_len[2];
	rec0_len[TEST_RUNNING_SLOT] = 0x1000u;
	rec0_len[TEST_OTHER_SLOT]   = 0x2000u;
	uint32_t rec1_len[2];
	rec1_len[TEST_RUNNING_SLOT] = 0x3000u;
	rec1_len[TEST_OTHER_SLOT]   = 0x4000u;
	write_meta_record(OTA_META_REC0, 9u, TEST_OTHER_SLOT, 0x03u, rec0_len);
	write_meta_record(OTA_META_REC1, 8u, TEST_RUNNING_SLOT, 0x01u, rec1_len);

	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_ROLLBACK, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "ROLLBACK must succeed");

	ota_meta_record_t rec1;
	zassert_true(read_meta_at(OTA_META_REC1, &rec1), "REC1 must remain CRC-valid, untouched");
	zassert_equal(rec1.counter, 8u, "REC1's counter must be untouched");
	zassert_equal(rec1.active_slot,
	              TEST_RUNNING_SLOT,
	              "REC1 must still name the running slot -- it must not be the erase target");

	ota_meta_record_t rec0;
	zassert_true(read_meta_at(OTA_META_REC0, &rec0), "REC0 must hold the freshly committed record");
	zassert_equal(rec0.counter, 10u, "REC0's new record must carry counter = old max (9) + 1");
	/* ROLLBACK leaves the descriptor table untouched (update_entry=false),
	 * so it must carry forward from REC0 (the NEWEST record) exactly as
	 * COMMIT's carry-forward does above -- same mutant, same kill
	 * condition, this time via h_rollback's call path instead of
	 * h_commit's. */
	zassert_equal(rec0.img_len[TEST_RUNNING_SLOT],
	              0x1000u,
	              "committed record's img_len[TEST_RUNNING_SLOT] must carry from REC0 (the "
	              "newest), not REC1 (the survivor)");
	zassert_equal(rec0.slot_valid,
	              (uint8_t)(0x03u | (1u << TEST_RUNNING_SLOT)),
	              "committed record's slot_valid must carry from REC0 (the newest), not REC1");
}

/* Case 9 (review #6): meta_commit's own ota_fmc_erase_range()-fails early
 * return (ota.c: `if (!ota_fmc_erase_range(target, OTA_PAGE_SIZE)) { return
 * false; }`) had no seam to exercise -- g_program_fail only covers a
 * failure AFTER a successful erase.  Model the erase itself failing
 * outright (nothing touched, see g_erase_fail's comment) and prove
 * meta_commit leaves BOTH existing records exactly as they were, and the
 * command surfaces the real error status. */
ZTEST(gd32_bridge_ota, test_meta_commit_erase_fail_preserves_both_records)
{
	reset_model();

	const uint32_t len[2] = { 4096u, 4096u };
	write_meta_record(OTA_META_REC0, 5u, TEST_RUNNING_SLOT, 0x03u, len);
	write_meta_record(OTA_META_REC1, 3u, TEST_RUNNING_SLOT, 0x03u, len);

	drive_to_verified();
	g_erase_fail = true;
	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_COMMIT, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_IO,
	              "COMMIT must report STATUS_IO when the metadata erase itself fails");

	ota_meta_record_t rec0;
	zassert_true(read_meta_at(OTA_META_REC0, &rec0), "REC0 must remain CRC-valid, untouched");
	zassert_equal(rec0.counter, 5u, "REC0's counter must be untouched by a failed erase");
	zassert_equal(rec0.active_slot, TEST_RUNNING_SLOT, "REC0's active_slot must be untouched");

	ota_meta_record_t rec1;
	zassert_true(read_meta_at(OTA_META_REC1, &rec1), "REC1 must remain CRC-valid, untouched");
	zassert_equal(rec1.counter, 3u, "REC1's counter must be untouched by a failed erase");
	zassert_equal(rec1.active_slot, TEST_RUNNING_SLOT, "REC1's active_slot must be untouched");

	g_erase_fail = false; /* hygiene: no before-hook clears this (#6) */
}

/* ===================================================================== *
 * #131 -- a REJECTED OTA_BEGIN must not leave its unvalidated length in
 * the session.
 *
 * h_begin used to assign s_img_len / s_expected_crc / s_fw_version from
 * the wire and range-check afterwards, so a rejected BEGIN left the bad
 * length live.  The reject sets OTA_ST_ERROR -- but a background erase
 * armed by an EARLIER, valid BEGIN is still draining, and ota_erase_tick's
 * completion writeback overwrites that ERROR with OTA_ST_READY (#9's
 * mechanism).  h_verify's only guard is `s_state != OTA_ST_READY`, so it
 * then handed the rejected length to ota_crc32, which has no bound of its
 * own, and walked past the end of the slot.
 *
 * The bad length here is OTA_SLOT_SIZE + 1 rather than the 0xFFFFFFFF or
 * 0x00040000 of the report.  It is the smallest value h_begin rejects, so
 * it pins exactly the same defect while keeping the UNFIXED run's overread
 * to one byte past g_flash[] -- a host harness reading 20 KB or 4 GB past
 * a .bss array to prove a point is not a test, it is a crash.
 * ===================================================================== */
ZTEST(gd32_bridge_ota, test_rejected_begin_does_not_overwrite_session_length)
{
	reset_model();

	/* A valid 8-byte image, and the CRC the host would have supplied. */
	uint8_t img[8];
	put32(&img[0], 0x20010000u);
	put32(&img[4], 0x08000001u);
	const uint32_t img_len  = (uint32_t)sizeof(img);
	const uint32_t good_crc = ota_crc32(0u, img, img_len);

	uint8_t req[8];
	uint8_t reply[8];
	size_t  rlen = 0u;

	/* 1. A valid BEGIN arms the background erase; state is BUSY. */
	wr_u32(&req[0], img_len);
	wr_u32(&req[4], good_crc);
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof(req), reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "the first BEGIN must be accepted");

	/* 2. Two ticks only: the erase is mid-flight, exactly the multi-second
	 *    window the second BEGIN lands in on silicon. */
	ota_erase_tick();
	ota_erase_tick();

	/* 3. A second BEGIN with an out-of-range size is rejected on the wire. */
	wr_u32(&req[0], (uint32_t)(OTA_SLOT_SIZE + 1u));
	wr_u32(&req[4], 0xDEADBEEFu);
	rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof(req), reply, sizeof(reply), &rlen),
	              STATUS_OUT_OF_RANGE,
	              "an out-of-range BEGIN must be refused");

	/* 4. The armed erase drains and writes OTA_ST_READY over the ERROR the
	 *    reject just set.  This writeback is #9's mechanism and is NOT
	 *    fixed here -- it is the vehicle that makes the stale length
	 *    reachable, which is why it is reproduced rather than avoided. */
	for (unsigned i = 0u; i < (OTA_SLOT_SIZE / OTA_PAGE_SIZE) + 4u; ++i) {
		ota_erase_tick();
	}

	uint8_t wr[8];
	size_t  wrl = 0u;
	zassert_equal(write_chunk(0u, img, (uint8_t)img_len, wr, &wrl),
	              STATUS_OK,
	              "the erase writeback leaves the session writable");

	/* 5. VERIFY must walk the FIRST BEGIN's 8 bytes, not the rejected
	 *    length.  Before the fix this returns the CRC of OTA_SLOT_SIZE + 1
	 *    bytes -- one past the end of the slot -- and does not match. */
	rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_VERIFY, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "VERIFY dispatches");
	zassert_equal(rd_u32(&reply[0]),
	              good_crc,
	              "the rejected BEGIN's length must not have replaced the session's");
	zassert_equal(reply[4], 1u, "the image must still verify against the first BEGIN's CRC");
}

/* The rejected BEGIN must not have replaced the expected CRC either --
 * a separate field, a separate assignment, and a mismatch there would
 * fail the session in the opposite direction (a good image reported
 * unverified) rather than overreading. */
ZTEST(gd32_bridge_ota, test_rejected_begin_does_not_overwrite_expected_crc)
{
	reset_model();

	uint8_t img[8];
	put32(&img[0], 0x20010000u);
	put32(&img[4], 0x08000001u);
	const uint32_t img_len  = (uint32_t)sizeof(img);
	const uint32_t good_crc = ota_crc32(0u, img, img_len);

	uint8_t req[8];
	uint8_t reply[8];
	size_t  rlen = 0u;

	wr_u32(&req[0], img_len);
	wr_u32(&req[4], good_crc);
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof(req), reply, sizeof(reply), &rlen),
	              STATUS_OK);
	ota_erase_tick();

	/* Zero size is the other rejected shape h_begin checks. */
	wr_u32(&req[0], 0u);
	wr_u32(&req[4], 0xDEADBEEFu);
	rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof(req), reply, sizeof(reply), &rlen),
	              STATUS_OUT_OF_RANGE,
	              "a zero-size BEGIN must be refused");

	for (unsigned i = 0u; i < (OTA_SLOT_SIZE / OTA_PAGE_SIZE) + 4u; ++i) {
		ota_erase_tick();
	}
	uint8_t wr[8];
	size_t  wrl = 0u;
	zassert_equal(write_chunk(0u, img, (uint8_t)img_len, wr, &wrl), STATUS_OK);

	rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_VERIFY, NULL, 0u, reply, sizeof(reply), &rlen), STATUS_OK);
	zassert_equal(
	    reply[4], 1u, "the rejected BEGIN's 0xDEADBEEF must not have replaced the expected CRC");
}

/* ===================================================================== *
 * #147 -- h_rollback was the only OTA handler with no s_state guard, so
 * it could dispatch from a transport ISR into meta_commit -> the FMC
 * funnel while the BASE-level erase pump held the FMC unlocked mid-page-
 * walk.  The funnel now refuses that re-entry outright
 * (hal/fmc_ota.c's ownership interlock, not reachable from this host
 * suite); this is the other half, at the state machine, so the host gets
 * an accurate STATUS_BUSY instead of a STATUS_IO that reads like a flash
 * fault.
 *
 * ROLLBACK is allowed from IDLE and from ERROR (the recovery case) and
 * refused from BUSY / READY / VERIFIED.  CMD_OTA_ABORT is the documented
 * exit from an in-flight session.
 * ===================================================================== */
ZTEST(gd32_bridge_ota, test_rollback_refused_while_a_session_is_in_flight)
{
	reset_model();

	/* A valid fallback slot so the only thing that can refuse ROLLBACK
	 * below is the new state guard, not the metadata checks. */
	const uint32_t len[2] = { 4096u, 4096u };
	write_meta_record(OTA_META_REC0, 1u, TEST_OTHER_SLOT, 0x03u, len);

	uint8_t req[8];
	uint8_t reply[8] = { 0 };
	size_t  rlen     = 0u;

	/* Open a session: BEGIN arms the background erase and leaves BUSY. */
	wr_u32(&req[0], 4096u);
	wr_u32(&req[4], 0u);
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof(req), reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "BEGIN must open the session");

	/* Mid-erase: exactly the window in which an ISR-dispatched ROLLBACK
	 * used to re-enter the FMC underneath the base-level pump. */
	ota_erase_tick();
	rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_ROLLBACK, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_BUSY,
	              "ROLLBACK must be refused while an erase is in flight");

	/* And still refused once the erase drains to READY -- the session is
	 * open until the host closes it, not just while the FMC is hot. */
	for (unsigned i = 0u; i < (OTA_SLOT_SIZE / OTA_PAGE_SIZE) + 4u; ++i) {
		ota_erase_tick();
	}
	rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_ROLLBACK, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_BUSY,
	              "ROLLBACK must still be refused with the session open at READY");

	/* ABORT is the documented way out; ROLLBACK then works. */
	rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_ABORT, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "ABORT closes the session");
	rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_ROLLBACK, NULL, 0u, reply, sizeof(reply), &rlen),
	              STATUS_OK,
	              "ROLLBACK must succeed once the session is closed");
}

/* ---- trial/confirm + watchdog fallback (bench fact 2026-09-26,
 * E1M-V2M103) ------------------------------------------------------------
 * TEST_RUNNING_SLOT / TEST_OTHER_SLOT (top of file) mirror ota.c's own
 * compile-time OTA_RUNNING_SLOT for whichever of the two slot builds this
 * binary is; both are exercised since this suite is built twice. */

/* test_boot_candidate_ok_truth_table dropped (adversarial-verify
 * request): the composed ota_boot_select() tests below exercise every
 * cell of the same truth table (non-TRIAL/TRIAL x watchdog fired/not)
 * through the real composed decision, so the standalone unit-level
 * duplicate added nothing the composed tests didn't already cover. */

ZTEST(gd32_bridge_ota, test_boot_init_gates_a_fresh_trial)
{
	reset_model();
	const uint32_t len[2] = { 64u, 64u };
	write_meta_record_flags(OTA_META_REC0, 1u, TEST_RUNNING_SLOT, 0x03u, len, OTA_META_FLAG_TRIAL);

	ota_boot_init();
	zassert_true(ota_trial_unconfirmed(), "metadata names us TRIAL+active: gate the wire");
	zassert_equal(g_reset_calls, 0u, "recognising a trial boot must not itself reset");
}

ZTEST(gd32_bridge_ota, test_boot_init_confirmed_image_gate_is_false)
{
	reset_model();
	const uint32_t len[2] = { 64u, 64u };
	write_meta_record_flags(OTA_META_REC0, 1u, TEST_RUNNING_SLOT, 0x03u, len, 0u); /* confirmed */

	ota_boot_init();
	zassert_false(ota_trial_unconfirmed(), "a confirmed image must never be gated");
}

ZTEST(gd32_bridge_ota, test_boot_init_heals_from_rejected_trial)
{
	reset_model();
	/* Metadata still names the OTHER slot TRIAL+active -- as it would if
	 * the bootloader's watchdog gate rejected that candidate and fell
	 * back to TEST_RUNNING_SLOT, which is what THIS process is playing. */
	const uint32_t len[2] = { 64u, 64u };
	write_meta_record_flags(OTA_META_REC0, 1u, TEST_OTHER_SLOT, 0x03u, len, OTA_META_FLAG_TRIAL);

	ota_boot_init();
	zassert_false(ota_trial_unconfirmed(), "the running fallback image is not itself gated");
	zassert_equal(g_reset_calls, 0u, "self-heal must not reset -- we are already the safe slot");

	/* meta_commit's #74 rank rule (see ota.c) targets whichever page ranks
	 * LOWER: REC0 here is valid but names TEST_OTHER_SLOT (not the
	 * running slot) -> rank 1; REC1 is empty -> rank 0.  Rank 0 < rank 1,
	 * so REC1 is the erase target and the new (higher-counter) record
	 * lands there, becoming the newest. */
	ota_meta_record_t newest;
	uint32_t          which;
	zassert_true(meta_current_for_test(&newest, &which));
	zassert_equal(
	    newest.active_slot, TEST_RUNNING_SLOT, "self-heal must repoint active at the running slot");
	zassert_equal(newest.flags, 0u, "self-heal must clear TRIAL");
	zassert_equal(newest.counter, 2u, "self-heal commits a new (higher-counter) record");
	zassert_equal(newest.slot_valid,
	              (uint8_t)(1u << TEST_RUNNING_SLOT),
	              "self-heal must invalidate the rejected (TEST_OTHER_SLOT) slot so ROLLBACK "
	              "can never re-select it");
	zassert_equal(g_fault_loop_clear_calls,
	              1u,
	              "self-heal (a 'healthy boot' signal) must clear the fault-loop counter (C3)");
}

ZTEST(gd32_bridge_ota, test_boot_init_heals_with_both_pages_valid)
{
	reset_model();
	/* Both pages valid this time: REC0 = older CONFIRMED record naming
	 * TEST_RUNNING_SLOT (rank 2 -- matches the running slot); REC1 =
	 * newer TRIAL record naming TEST_OTHER_SLOT (rank 1 -- rejected).
	 * Ranks differ (2 vs 1), so the self-heal commit's rank rule must
	 * erase the LOWER-ranked page (REC1), preserving REC0 untouched --
	 * the confirmed record that is actually running. */
	uint32_t len0[2];
	len0[TEST_RUNNING_SLOT] = 64u;
	len0[TEST_OTHER_SLOT]   = 0u;
	write_meta_record_flags(
	    OTA_META_REC0, 1u, TEST_RUNNING_SLOT, (uint8_t)(1u << TEST_RUNNING_SLOT), len0, 0u);

	uint32_t len1[2];
	len1[TEST_RUNNING_SLOT] = 0u;
	len1[TEST_OTHER_SLOT]   = 64u;
	write_meta_record_flags(OTA_META_REC1,
	                        2u,
	                        TEST_OTHER_SLOT,
	                        (uint8_t)(1u << TEST_OTHER_SLOT),
	                        len1,
	                        OTA_META_FLAG_TRIAL);

	ota_boot_init();
	zassert_false(ota_trial_unconfirmed(), "the record naming us (REC0) is not TRIAL");
	zassert_equal(g_reset_calls, 0u, "self-heal must not reset");

	ota_meta_record_t rec0;
	zassert_true(read_meta_at(OTA_META_REC0, &rec0), "REC0 must survive the self-heal untouched");
	zassert_equal(rec0.counter, 1u);
	zassert_equal(rec0.active_slot, TEST_RUNNING_SLOT);

	ota_meta_record_t newest;
	uint32_t          which;
	zassert_true(meta_current_for_test(&newest, &which));
	zassert_equal(
	    which, OTA_META_REC1, "the heal commit must land on REC1 (rank 1), not REC0 (rank 2)");
	zassert_equal(newest.active_slot, TEST_RUNNING_SLOT);
	zassert_equal(newest.flags, 0u);
	zassert_equal(newest.slot_valid,
	              (uint8_t)(1u << TEST_RUNNING_SLOT),
	              "self-heal must invalidate the rejected TEST_OTHER_SLOT");
	zassert_equal(newest.counter, 3u, "counter must exceed both prior records");
}

ZTEST(gd32_bridge_ota, test_confirm_tick_commits_and_reboots)
{
	reset_model();
	const uint32_t len[2] = { 64u, 64u };
	write_meta_record_flags(OTA_META_REC0, 1u, TEST_RUNNING_SLOT, 0x03u, len, OTA_META_FLAG_TRIAL);

	ota_boot_init();
	zassert_true(ota_trial_unconfirmed());

	/* No frame noted yet: the tick must do nothing. */
	ota_confirm_tick();
	zassert_true(ota_trial_unconfirmed(), "no frame noted yet -- must not confirm early");
	zassert_equal(g_reset_calls, 0u);

	ota_note_frame();
	ota_confirm_tick();
	/* The gate stays CLOSED through the reset itself (item 6, bench fact
	 * 2026-09-26): s_trial is never cleared before ota_system_reset() --
	 * a real reset never returns, so there is no "confirmed but not yet
	 * rebooted" state on real silicon.  Proving confirmation landed means
	 * simulating the NEXT boot: call ota_boot_init() again and check that
	 * IT reads back the (now flags=0) flash record. */
	zassert_true(ota_trial_unconfirmed(), "the gate must stay closed through the reset itself");
	zassert_equal(g_reset_calls, 1u, "confirm must reboot exactly once");

	ota_meta_record_t newest;
	uint32_t          which;
	zassert_true(meta_current_for_test(&newest, &which));
	zassert_equal(newest.active_slot, TEST_RUNNING_SLOT);
	zassert_equal(newest.flags, 0u, "confirmed record must clear TRIAL");
	zassert_equal(newest.counter, 2u, "confirm must bump the counter");
	zassert_equal(g_fault_loop_clear_calls,
	              1u,
	              "a successful confirm (a 'healthy boot' signal) must clear the fault-loop "
	              "counter (C3)");

	ota_boot_init(); /* simulates the reboot ota_system_reset() causes */
	zassert_false(ota_trial_unconfirmed(),
	              "a fresh boot reading the now-confirmed record must not be gated");
}

ZTEST(gd32_bridge_ota, test_confirm_tick_failed_commit_preserves_older_confirmed_record)
{
	reset_model();
	/* Older, already-CONFIRMED record on REC0 -- this is what a failed
	 * or power-cut-interrupted confirm must leave standing (item 1,
	 * bench fact 2026-09-26: "reverted", not "only TRIAL is left"). */
	uint32_t len_old[2];
	len_old[TEST_OTHER_SLOT]   = 64u;
	len_old[TEST_RUNNING_SLOT] = 0u;
	write_meta_record_flags(
	    OTA_META_REC0, 1u, TEST_OTHER_SLOT, (uint8_t)(1u << TEST_OTHER_SLOT), len_old, 0u);

	/* Freshly-committed TRIAL record naming TEST_RUNNING_SLOT on REC1 --
	 * the page a real COMMIT's #74 rank rule would land it on here
	 * (REC0 ranks 1: valid, names the non-running slot; REC1 ranks 0:
	 * empty -- so REC1 is the lower-ranked erase target). */
	uint32_t len_new[2];
	len_new[TEST_OTHER_SLOT]   = 64u;
	len_new[TEST_RUNNING_SLOT] = 64u;
	write_meta_record_flags(
	    OTA_META_REC1, 2u, TEST_RUNNING_SLOT, 0x03u, len_new, OTA_META_FLAG_TRIAL);

	ota_boot_init();
	zassert_true(ota_trial_unconfirmed(), "the record naming us (REC1) is TRIAL: fresh trial boot");

	ota_note_frame();
	g_program_fail = true;
	ota_confirm_tick();
	g_program_fail = false;

	zassert_true(ota_trial_unconfirmed(),
	             "a failed confirm commit must leave TRIAL gating the wire");
	zassert_equal(g_reset_calls, 0u, "must not reset on a failed confirm commit");

	/* meta_commit's target_override forces the confirm onto REC1 (the
	 * TRIAL record's OWN page) -- so a failed program leaves REC1 erased
	 * (blank/invalid) but must NEVER have touched REC0. */
	ota_meta_record_t rec0;
	zassert_true(read_meta_at(OTA_META_REC0, &rec0),
	             "the OLDER confirmed record (REC0) must survive a failed confirm untouched");
	zassert_equal(rec0.active_slot, TEST_OTHER_SLOT);
	zassert_equal(rec0.flags, 0u);
	zassert_equal(rec0.counter, 1u);

	ota_meta_record_t rec1;
	zassert_false(
	    read_meta_at(OTA_META_REC1, &rec1),
	    "REC1 (the confirm's own erase target) is left erased/invalid by the failed program");

	/* Post-failure, meta_current() -- and so the NEXT real boot -- sees
	 * only the older confirmed record: "reverted", not "only TRIAL". */
	ota_meta_record_t newest;
	uint32_t          which;
	zassert_true(meta_current_for_test(&newest, &which));
	zassert_equal(which, OTA_META_REC0);
	zassert_equal(newest.active_slot, TEST_OTHER_SLOT);
	zassert_equal(newest.flags, 0u);
}

ZTEST(gd32_bridge_ota, test_confirm_tick_program_failure_keeps_trial_no_reset)
{
	reset_model();
	const uint32_t len[2] = { 64u, 64u };
	write_meta_record_flags(OTA_META_REC0, 1u, TEST_RUNNING_SLOT, 0x03u, len, OTA_META_FLAG_TRIAL);
	ota_boot_init();
	zassert_true(ota_trial_unconfirmed());

	ota_note_frame();
	g_program_fail = true;
	ota_confirm_tick();
	g_program_fail = false;

	zassert_true(ota_trial_unconfirmed(),
	             "a failed confirm commit must leave TRIAL gating the wire");
	zassert_equal(g_reset_calls, 0u, "must not reset on a failed confirm commit");
	zassert_equal(g_fault_loop_clear_calls,
	              0u,
	              "a failed commit is not a 'healthy boot' signal -- must not clear the "
	              "fault-loop counter");
}

ZTEST(gd32_bridge_ota, test_busy_gate_blocks_all_opcodes_until_confirmed)
{
	reset_model();
	const uint32_t len[2] = { 64u, 64u };
	write_meta_record_flags(OTA_META_REC0, 1u, TEST_RUNNING_SLOT, 0x03u, len, OTA_META_FLAG_TRIAL);
	ota_boot_init();
	zassert_true(ota_trial_unconfirmed());

	static const uint8_t cmds[] = {
		(uint8_t)CMD_PING,         (uint8_t)CMD_GET_VERSION, (uint8_t)CMD_GET_BUILD_ID,
		(uint8_t)CMD_RESET_REASON, (uint8_t)CMD_GPIO_READ,
	};
	for (size_t i = 0u; i < sizeof(cmds); ++i) {
		uint8_t              reply[8];
		size_t               rlen = 123u; /* poisoned -- must come back 0 */
		gd32_bridge_status_t st =
		    protocol_dispatch(GD32_BRIDGE_LINK_SPI, cmds[i], NULL, 0u, reply, sizeof reply, &rlen);
		zassert_equal(
		    st, STATUS_BUSY, "opcode 0x%02x must be gated BUSY while unconfirmed", cmds[i]);
		zassert_equal(rlen, 0u, "gated reply must carry an empty payload");
	}
	zassert_true(ota_trial_unconfirmed(), "merely dispatching frames doesn't confirm by itself");

	/* The gate noted every one of those frames; the confirm tick (as
	 * bridge_hw_tick would run it) now commits and reboots. The gate
	 * stays CLOSED through the reset itself (item 6) -- proving
	 * confirmation landed means simulating the next boot. */
	ota_confirm_tick();
	zassert_true(ota_trial_unconfirmed(), "the gate must stay closed through the reset itself");
	zassert_equal(g_reset_calls, 1u);

	ota_boot_init(); /* simulates the reboot ota_system_reset() causes */
	zassert_false(ota_trial_unconfirmed());

	/* Post-(simulated)-reboot, a normal opcode gets a real reply again. */
	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(
	    protocol_dispatch(GD32_BRIDGE_LINK_SPI, CMD_PING, NULL, 0u, reply, sizeof reply, &rlen),
	    STATUS_OK);
}

ZTEST(gd32_bridge_ota, test_commit_sets_trial_flag_no_pad_carry)
{
	reset_model();
	/* Stale record naming TEST_RUNNING_SLOT active, with 0xFF garbage
	 * planted across the whole record (including the byte the new
	 * struct-v2 `flags` field occupies) -- proves COMMIT sets `flags`
	 * explicitly rather than carrying the old record's raw byte
	 * forward. Ranks 2 (names the running slot), so it survives and its
	 * bytes get carried into the new record by meta_commit's `rec = a`
	 * assignment -- exactly the carry-forward hazard flags must not
	 * inherit. */
	const uint32_t len[2] = { 0u, 0u };
	write_meta_record_flags(
	    OTA_META_REC0, 1u, TEST_RUNNING_SLOT, (uint8_t)(1u << TEST_RUNNING_SLOT), len, 0xFFu);

	/* Image carries a confirm-capable trial marker (bench fact 2026-09-26
	 * follow-up) -- this test's whole point is to prove COMMIT sets
	 * `flags` explicitly to TRIAL, so it needs an image
	 * ota_image_trial_capable() actually accepts. */
	uint8_t img[TEST_IMG_LEN] = { 0 };
	put32(&img[0], 0x20010000u); /* MSP into SRAM */
	uint32_t other_base;
	zassert_true(ota_slot_base_checked(TEST_OTHER_SLOT, &other_base));
	put32(&img[4], (other_base + 8u) | 1u); /* reset, Thumb, in-image */
	{
		const uint8_t  magic[16] = OTA_TRIAL_MARKER_MAGIC_BYTES;
		const uint16_t sv        = OTA_TRIAL_MARKER_STRUCT_VER;
		const uint16_t cap       = OTA_TRIAL_CAP_CONFIRM;
		memcpy(&img[TEST_MARKER_OFFSET], magic, sizeof magic);
		img[TEST_MARKER_OFFSET + 16u] = (uint8_t)(sv & 0xFFu);
		img[TEST_MARKER_OFFSET + 17u] = (uint8_t)(sv >> 8);
		img[TEST_MARKER_OFFSET + 18u] = (uint8_t)(cap & 0xFFu);
		img[TEST_MARKER_OFFSET + 19u] = (uint8_t)(cap >> 8);
	}
	const uint32_t expected_crc = ota_crc32(0u, img, sizeof img);

	uint8_t req[11];
	wr_u32(&req[0], sizeof img);
	wr_u32(&req[4], expected_crc);
	req[8]  = 1u; /* fw_major */
	req[9]  = 2u; /* fw_minor */
	req[10] = 3u; /* fw_patch */
	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof req, reply, sizeof reply, &rlen),
	              STATUS_OK);
	for (unsigned i = 0u; i < (OTA_SLOT_SIZE / OTA_PAGE_SIZE) + 4u; ++i) {
		ota_erase_tick();
	}

	uint8_t wreply[8];
	size_t  wrl = 0u;
	zassert_equal(write_chunk(0u, img, sizeof img, wreply, &wrl), STATUS_OK);

	rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_VERIFY, NULL, 0u, reply, sizeof reply, &rlen), STATUS_OK);
	zassert_equal(reply[4], 1u, "verify must succeed (crc matches)");

	rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_COMMIT, NULL, 0u, reply, sizeof reply, &rlen), STATUS_OK);
	zassert_equal(g_reset_calls, 1u, "a successful COMMIT resets on real silicon");

	ota_meta_record_t newest;
	uint32_t          which;
	zassert_true(meta_current_for_test(&newest, &which), "a valid record must exist post-commit");
	zassert_equal(newest.active_slot, TEST_OTHER_SLOT, "commit must activate the inactive slot");
	zassert_equal(newest.flags,
	              (uint8_t)OTA_META_FLAG_TRIAL,
	              "flags must be exactly TRIAL, never the planted 0xFF garbage");
}

ZTEST(gd32_bridge_ota, test_rollback_sets_trial_flag_exactly)
{
	reset_model();
	/* TEST_RUNNING_SLOT active+confirmed; TEST_OTHER_SLOT has a valid
	 * descriptor to roll back to, and its OWN flash bytes carry a
	 * confirm-capable trial marker (bench fact 2026-09-26 follow-up) --
	 * NOT the 0xFF-garbage write_meta_record_flags() would otherwise
	 * leave in the metadata record, which is irrelevant here since the
	 * downgrade guard now reads the image, not the metadata. */
	uint32_t len[2];
	len[TEST_RUNNING_SLOT] = TEST_IMG_LEN;
	len[TEST_OTHER_SLOT]   = TEST_IMG_LEN;
	write_meta_record_flags(
	    OTA_META_REC0, 1u, TEST_RUNNING_SLOT, 0x03u /* both slots valid */, len, 0xFFu);
	uint32_t other_base;
	zassert_true(ota_slot_base_checked(TEST_OTHER_SLOT, &other_base));
	plant_trial_image(other_base, true /* with_marker */, true /* cap_bit */);

	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_ROLLBACK, NULL, 0u, reply, sizeof reply, &rlen), STATUS_OK);
	zassert_equal(g_reset_calls, 1u);

	ota_meta_record_t newest;
	uint32_t          which;
	zassert_true(meta_current_for_test(&newest, &which));
	zassert_equal(
	    newest.active_slot, TEST_OTHER_SLOT, "rollback must flip active to the other slot");
	zassert_equal(
	    newest.flags, (uint8_t)OTA_META_FLAG_TRIAL, "rollback marks the rolled-to slot TRIAL");
	zassert_equal(newest.img_len[TEST_OTHER_SLOT],
	              TEST_IMG_LEN,
	              "rollback must not touch the per-slot descriptors");
}

/* ---- downgrade guard: TRIAL only for an image that carries a
 * confirm-capable trial marker (bench fact 2026-09-26 follow-up).
 * Replaces the old fw_version-threshold guard: eligibility now comes
 * from the IMAGE ITSELF (ota_image_trial_capable(), src/ota_layout.h),
 * not the version the host declares in OTA_BEGIN. */

/* Runs a full BEGIN->WRITE->VERIFY->COMMIT cycle for a small bootable
 * image, with or without a trial marker (and, when present, with its
 * capability bit set or cleared) -- meta_commit() is static, so the
 * downgrade guard can only be driven through the real wire path, not by
 * poking it directly. */
static gd32_bridge_status_t commit_cycle_marker(bool with_marker, bool cap_bit)
{
	uint8_t img[TEST_IMG_LEN] = { 0 };
	put32(&img[0], 0x20010000u); /* MSP into SRAM */
	uint32_t other_base;
	zassert_true(ota_slot_base_checked(TEST_OTHER_SLOT, &other_base));
	put32(&img[4], (other_base + 8u) | 1u); /* reset, Thumb, in-image */
	if (with_marker) {
		const uint8_t  magic[16] = OTA_TRIAL_MARKER_MAGIC_BYTES;
		const uint16_t sv        = OTA_TRIAL_MARKER_STRUCT_VER;
		const uint16_t cap       = cap_bit ? OTA_TRIAL_CAP_CONFIRM : 0u;
		memcpy(&img[TEST_MARKER_OFFSET], magic, sizeof magic);
		img[TEST_MARKER_OFFSET + 16u] = (uint8_t)(sv & 0xFFu);
		img[TEST_MARKER_OFFSET + 17u] = (uint8_t)(sv >> 8);
		img[TEST_MARKER_OFFSET + 18u] = (uint8_t)(cap & 0xFFu);
		img[TEST_MARKER_OFFSET + 19u] = (uint8_t)(cap >> 8);
	}
	const uint32_t expected_crc = ota_crc32(0u, img, sizeof img);

	uint8_t req[8];
	wr_u32(&req[0], sizeof img);
	wr_u32(&req[4], expected_crc);
	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof req, reply, sizeof reply, &rlen),
	              STATUS_OK);
	for (unsigned i = 0u; i < (OTA_SLOT_SIZE / OTA_PAGE_SIZE) + 4u; ++i) {
		ota_erase_tick();
	}

	uint8_t wreply[8];
	size_t  wrl = 0u;
	zassert_equal(write_chunk(0u, img, sizeof img, wreply, &wrl), STATUS_OK);

	rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_VERIFY, NULL, 0u, reply, sizeof reply, &rlen), STATUS_OK);
	zassert_equal(reply[4], 1u, "verify must succeed (crc matches)");

	rlen = 0u;
	return ota_dispatch(CMD_OTA_COMMIT, NULL, 0u, reply, sizeof reply, &rlen);
}

ZTEST(gd32_bridge_ota, test_commit_no_marker_refused)
{
	/* Policy (closed on PR #246): a pre-marker (or otherwise markerless)
	 * image cannot generate the confirm handshake, so it is REFUSED at
	 * COMMIT -- STATUS_INVAL, metadata and active slot untouched, no
	 * reset -- rather than committed CONFIRMED and unprotected. See
	 * h_commit()'s comment (src/ota.c). */
	reset_model();
	uint32_t len[2];
	len[TEST_RUNNING_SLOT] = TEST_IMG_LEN;
	len[TEST_OTHER_SLOT]   = 0u;
	write_meta_record_flags(OTA_META_REC0, 5u, TEST_RUNNING_SLOT, 0x01u, len, 0u /* CONFIRMED */);

	zassert_equal(commit_cycle_marker(false, false), STATUS_INVAL);

	ota_meta_record_t after;
	uint32_t          which;
	zassert_true(meta_current_for_test(&after, &which));
	zassert_equal(which, OTA_META_REC0, "a refused COMMIT must not touch either metadata page");
	zassert_equal(after.counter, 5u, "a refused COMMIT must leave the metadata record untouched");
	zassert_equal(
	    after.active_slot, TEST_RUNNING_SLOT, "a refused COMMIT must not flip the active slot");
	zassert_equal(after.flags, 0u, "a refused COMMIT must not touch flags");
	zassert_equal(g_reset_calls, 0u, "a refused COMMIT must not reset -- no reboot on refusal");

	/* The very next OTA_BEGIN must still work: h_begin has no
	 * precondition on the OTA_ST_ERROR the refusal above left behind. */
	uint8_t  reply[8];
	size_t   rlen = 0u;
	uint8_t  req[8];
	wr_u32(&req[0], TEST_IMG_LEN);
	wr_u32(&req[4], 0u);
	zassert_equal(ota_dispatch(CMD_OTA_BEGIN, req, sizeof req, reply, sizeof reply, &rlen),
	              STATUS_OK,
	              "a fresh OTA_BEGIN after a refused COMMIT must still succeed");
}

ZTEST(gd32_bridge_ota, test_commit_marker_confirm_capable_sets_trial)
{
	reset_model();
	zassert_equal(commit_cycle_marker(true, true), STATUS_OK);

	ota_meta_record_t newest;
	uint32_t          which;
	zassert_true(meta_current_for_test(&newest, &which));
	zassert_equal(newest.flags,
	              (uint8_t)OTA_META_FLAG_TRIAL,
	              "a confirm-capable marker must get TRIAL protection");
}

ZTEST(gd32_bridge_ota, test_commit_marker_no_confirm_bit_refused)
{
	/* Marker present (so a future image COULD advertise other
	 * capabilities) but the confirm-capability bit is clear: this image
	 * still cannot generate the confirm handshake, so ota_image_trial_capable()
	 * reads it exactly like a markerless image and COMMIT refuses it too
	 * (policy closed on PR #246, see h_commit()'s comment, src/ota.c). */
	reset_model();
	zassert_equal(commit_cycle_marker(true, false), STATUS_INVAL);

	ota_meta_record_t newest;
	uint32_t          which;
	zassert_false(meta_current_for_test(&newest, &which),
	              "a refused COMMIT must leave metadata untouched (still no valid record)");
}

ZTEST(gd32_bridge_ota, test_rollback_target_no_marker_confirms_no_trial)
{
	reset_model();
	uint32_t len[2];
	len[TEST_RUNNING_SLOT] = TEST_IMG_LEN;
	len[TEST_OTHER_SLOT]   = TEST_IMG_LEN;
	write_meta_record_flags(OTA_META_REC0, 1u, TEST_RUNNING_SLOT, 0x03u, len, 0u);
	uint32_t other_base;
	zassert_true(ota_slot_base_checked(TEST_OTHER_SLOT, &other_base));
	plant_trial_image(other_base, false /* with_marker */, false);

	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_ROLLBACK, NULL, 0u, reply, sizeof reply, &rlen), STATUS_OK);

	ota_meta_record_t newest;
	uint32_t          which;
	zassert_true(meta_current_for_test(&newest, &which));
	zassert_equal(newest.active_slot, TEST_OTHER_SLOT);
	zassert_equal(newest.flags, 0u, "rollback to a markerless target must not arm TRIAL");
}

ZTEST(gd32_bridge_ota, test_rollback_target_marker_confirm_capable_sets_trial)
{
	reset_model();
	uint32_t len[2];
	len[TEST_RUNNING_SLOT] = TEST_IMG_LEN;
	len[TEST_OTHER_SLOT]   = TEST_IMG_LEN;
	write_meta_record_flags(OTA_META_REC0, 1u, TEST_RUNNING_SLOT, 0x03u, len, 0u);
	uint32_t other_base;
	zassert_true(ota_slot_base_checked(TEST_OTHER_SLOT, &other_base));
	plant_trial_image(other_base, true /* with_marker */, true /* cap_bit */);

	uint8_t reply[8];
	size_t  rlen = 0u;
	zassert_equal(ota_dispatch(CMD_OTA_ROLLBACK, NULL, 0u, reply, sizeof reply, &rlen), STATUS_OK);

	ota_meta_record_t newest;
	uint32_t          which;
	zassert_true(meta_current_for_test(&newest, &which));
	zassert_equal(newest.active_slot, TEST_OTHER_SLOT);
	zassert_equal(newest.flags,
	              (uint8_t)OTA_META_FLAG_TRIAL,
	              "rollback to a confirm-capable-marker target must arm TRIAL");
}

/* ---- ota_image_trial_capable() (src/ota_layout.h) in isolation: pure and
 * flash-free, so these drive it directly with hand-built byte buffers. */

ZTEST(gd32_bridge_ota, test_image_trial_capable_present_and_confirm_capable)
{
	uint8_t buf[TEST_IMG_LEN] = { 0 };
	{
		const uint8_t  magic[16] = OTA_TRIAL_MARKER_MAGIC_BYTES;
		const uint16_t sv        = OTA_TRIAL_MARKER_STRUCT_VER;
		const uint16_t cap       = OTA_TRIAL_CAP_CONFIRM;
		memcpy(&buf[TEST_MARKER_OFFSET], magic, sizeof magic);
		buf[TEST_MARKER_OFFSET + 16u] = (uint8_t)(sv & 0xFFu);
		buf[TEST_MARKER_OFFSET + 17u] = (uint8_t)(sv >> 8);
		buf[TEST_MARKER_OFFSET + 18u] = (uint8_t)(cap & 0xFFu);
		buf[TEST_MARKER_OFFSET + 19u] = (uint8_t)(cap >> 8);
	}
	zassert_true(ota_image_trial_capable(buf, sizeof buf));
}

ZTEST(gd32_bridge_ota, test_image_trial_capable_absent)
{
	uint8_t buf[TEST_IMG_LEN] = { 0 }; /* no marker anywhere */
	zassert_false(ota_image_trial_capable(buf, sizeof buf));
}

ZTEST(gd32_bridge_ota, test_image_trial_capable_corrupt_magic)
{
	uint8_t buf[TEST_IMG_LEN] = { 0 };
	{
		const uint8_t  magic[16] = OTA_TRIAL_MARKER_MAGIC_BYTES;
		const uint16_t sv        = OTA_TRIAL_MARKER_STRUCT_VER;
		const uint16_t cap       = OTA_TRIAL_CAP_CONFIRM;
		memcpy(&buf[TEST_MARKER_OFFSET], magic, sizeof magic);
		buf[TEST_MARKER_OFFSET] ^= 0xFFu; /* corrupt exactly one magic byte */
		buf[TEST_MARKER_OFFSET + 16u] = (uint8_t)(sv & 0xFFu);
		buf[TEST_MARKER_OFFSET + 17u] = (uint8_t)(sv >> 8);
		buf[TEST_MARKER_OFFSET + 18u] = (uint8_t)(cap & 0xFFu);
		buf[TEST_MARKER_OFFSET + 19u] = (uint8_t)(cap >> 8);
	}
	zassert_false(ota_image_trial_capable(buf, sizeof buf), "a corrupt magic must not match");
}

ZTEST(gd32_bridge_ota, test_image_trial_capable_unknown_struct_version_rejected)
{
	uint8_t buf[TEST_IMG_LEN] = { 0 };
	{
		const uint8_t  magic[16] = OTA_TRIAL_MARKER_MAGIC_BYTES;
		const uint16_t cap       = OTA_TRIAL_CAP_CONFIRM;
		memcpy(&buf[TEST_MARKER_OFFSET], magic, sizeof magic);
		buf[TEST_MARKER_OFFSET + 16u] = 0xFFu; /* struct_version = 0xFFFF, not recognised */
		buf[TEST_MARKER_OFFSET + 17u] = 0xFFu;
		buf[TEST_MARKER_OFFSET + 18u] = (uint8_t)(cap & 0xFFu);
		buf[TEST_MARKER_OFFSET + 19u] = (uint8_t)(cap >> 8);
	}
	zassert_false(ota_image_trial_capable(buf, sizeof buf),
	              "an unrecognised struct_version must not be trusted");
}

ZTEST(gd32_bridge_ota, test_image_trial_capable_before_min_offset_not_found)
{
	/* A byte-for-byte marker sitting inside the core vector-table region
	 * (< OTA_TRIAL_MARKER_MIN_OFFSET) must NOT be treated as the real
	 * one -- it would have to overlap the fixed ARMv8-M core exception
	 * vectors that precede any device-specific vector. */
	uint8_t buf[TEST_IMG_LEN] = { 0 };
	{
		const uint8_t  magic[16] = OTA_TRIAL_MARKER_MAGIC_BYTES;
		const uint16_t sv        = OTA_TRIAL_MARKER_STRUCT_VER;
		const uint16_t cap       = OTA_TRIAL_CAP_CONFIRM;
		memcpy(&buf[0], magic, sizeof magic);
		buf[16] = (uint8_t)(sv & 0xFFu);
		buf[17] = (uint8_t)(sv >> 8);
		buf[18] = (uint8_t)(cap & 0xFFu);
		buf[19] = (uint8_t)(cap >> 8);
	}
	zassert_false(ota_image_trial_capable(buf, sizeof buf),
	              "a marker before OTA_TRIAL_MARKER_MIN_OFFSET must not be found");
}

ZTEST(gd32_bridge_ota, test_image_trial_capable_outside_scan_window_not_found)
{
	uint8_t buf[OTA_TRIAL_SCAN_LIMIT + 32u];
	memset(buf, 0, sizeof buf);
	const uint8_t  magic[16] = OTA_TRIAL_MARKER_MAGIC_BYTES;
	const uint16_t sv        = OTA_TRIAL_MARKER_STRUCT_VER;
	const uint16_t cap       = OTA_TRIAL_CAP_CONFIRM;
	const uint32_t off       = OTA_TRIAL_SCAN_LIMIT; /* past the bounded window */
	memcpy(&buf[off], magic, sizeof magic);
	buf[off + 16u] = (uint8_t)(sv & 0xFFu);
	buf[off + 17u] = (uint8_t)(sv >> 8);
	buf[off + 18u] = (uint8_t)(cap & 0xFFu);
	buf[off + 19u] = (uint8_t)(cap >> 8);
	zassert_false(ota_image_trial_capable(buf, sizeof buf),
	              "a marker past OTA_TRIAL_SCAN_LIMIT must not be found");
}

/* ---- composed bootloader selection loop (item 1, bench fact 2026-09-26):
 * ota_boot_select() (src/ota_layout.h) is pure and flash-free, so these
 * drive it directly with hand-built records -- no flash model needed. */

ZTEST(gd32_bridge_ota, test_boot_select_strict_pass_picks_newest_when_both_pass)
{
	ota_meta_record_t a, b;
	memset(&a, 0, sizeof a);
	a.active_slot = TEST_RUNNING_SLOT;
	a.flags       = 0u;
	memset(&b, 0, sizeof b);
	b.active_slot                        = TEST_OTHER_SLOT;
	b.flags                              = 0u;
	const ota_meta_record_t *cands[2]    = { &a, &b };
	bool                     valid[2]    = { true, true };
	bool                     last_resort = true; /* poisoned */
	const int                sel         = ota_boot_select(cands, valid, 2, false, &last_resort);
	zassert_equal(sel, 0, "newest-first: cands[0] wins when both pass strictly");
	zassert_false(last_resort);
}

ZTEST(gd32_bridge_ota, test_boot_select_skips_invalid_and_watchdog_rejected)
{
	ota_meta_record_t a, b;
	memset(&a, 0, sizeof a);
	a.flags = OTA_META_FLAG_TRIAL; /* newest, TRIAL */
	memset(&b, 0, sizeof b);
	b.flags                              = 0u; /* older, confirmed */
	const ota_meta_record_t *cands[2]    = { &a, &b };
	bool                     valid[2]    = { true, true };
	bool                     last_resort = false;
	/* wdt_fired: cands[0] (TRIAL) fails the strict pass; cands[1] wins. */
	const int sel = ota_boot_select(cands, valid, 2, true, &last_resort);
	zassert_equal(sel, 1, "the watchdog-rejected TRIAL candidate must not win strictly");
	zassert_false(last_resort, "a strict-pass winner exists -- last resort must not fire");
}

ZTEST(gd32_bridge_ota, test_boot_select_last_resort_boots_rejected_trial_when_nothing_else_valid)
{
	ota_meta_record_t a;
	memset(&a, 0, sizeof a);
	a.flags                              = OTA_META_FLAG_TRIAL;
	const ota_meta_record_t *cands[1]    = { &a };
	bool                     valid[1]    = { true };
	bool                     last_resort = false;
	const int sel = ota_boot_select(cands, valid, 1, true /* wdt_fired */, &last_resort);
	zassert_equal(sel, 0, "the only valid candidate must still boot, even watchdog-rejected");
	zassert_true(last_resort, "must report that the last-resort pass fired");
}

ZTEST(gd32_bridge_ota, test_boot_select_returns_none_when_nothing_valid)
{
	ota_meta_record_t a, b;
	memset(&a, 0, sizeof a);
	memset(&b, 0, sizeof b);
	const ota_meta_record_t *cands[2]    = { &a, &b };
	bool                     valid[2]    = { false, false };
	bool                     last_resort = true; /* poisoned */
	const int                sel         = ota_boot_select(cands, valid, 2, false, &last_resort);
	zassert_equal(sel, -1, "no valid candidate at all -- must report nothing bootable");
	zassert_false(last_resort);
}

ZTEST(gd32_bridge_ota, test_boot_select_trial_candidate_wins_strictly_without_watchdog)
{
	/* Completes the truth table the dropped standalone test used to pin
	 * directly: a TRIAL candidate with the watchdog NOT fired is fully
	 * eligible and must win strictly (no last resort needed). */
	ota_meta_record_t a;
	memset(&a, 0, sizeof a);
	a.flags                              = OTA_META_FLAG_TRIAL;
	const ota_meta_record_t *cands[1]    = { &a };
	bool                     valid[1]    = { true };
	bool                     last_resort = true; /* poisoned */
	const int sel = ota_boot_select(cands, valid, 1, false /* wdt_fired */, &last_resort);
	zassert_equal(sel, 0, "TRIAL absent a watchdog fallback must win strictly");
	zassert_false(last_resort);
}

ZTEST(gd32_bridge_ota, test_boot_select_both_trial_rejected_picks_newest_last_resort)
{
	/* Both candidates are TRIAL and both get rejected by the watchdog
	 * gate strictly -- last resort must fire and pick the NEWEST
	 * (cands[0]) of the two, not the older one. */
	ota_meta_record_t a, b;
	memset(&a, 0, sizeof a);
	a.flags = OTA_META_FLAG_TRIAL; /* newest */
	memset(&b, 0, sizeof b);
	b.flags                              = OTA_META_FLAG_TRIAL; /* older */
	const ota_meta_record_t *cands[2]    = { &a, &b };
	bool                     valid[2]    = { true, true };
	bool                     last_resort = false;
	const int sel = ota_boot_select(cands, valid, 2, true /* wdt_fired */, &last_resort);
	zassert_equal(sel, 0, "last resort must pick the NEWEST of two equally-rejected candidates");
	zassert_true(last_resort);
}

ZTEST(gd32_bridge_ota, test_boot_select_newest_invalid_older_trial_rejected_picks_idx1)
{
	/* The newest candidate is not even valid (fails active_slot_valid
	 * upstream, e.g. a corrupted image) -- it must never be considered
	 * by either pass; the older, watchdog-rejected TRIAL candidate is
	 * the only bootable thing at all, so last resort must pick it. */
	ota_meta_record_t a, b;
	memset(&a, 0, sizeof a); /* newest -- invalid */
	memset(&b, 0, sizeof b);
	b.flags                           = OTA_META_FLAG_TRIAL; /* older -- valid, watchdog-rejected */
	const ota_meta_record_t *cands[2] = { &a, &b };
	bool                     valid[2] = { false, true };
	bool                     last_resort = false;
	const int sel = ota_boot_select(cands, valid, 2, true /* wdt_fired */, &last_resort);
	zassert_equal(sel, 1, "the invalid newest candidate must never be picked by either pass");
	zassert_true(last_resort);
}

/* ---- C7 (adversarial-verify finding): ota_confirm_tick() must target the
 * record naming OTA_RUNNING_SLOT, not meta_current()'s newest -------- */

ZTEST(gd32_bridge_ota, test_confirm_tick_targets_running_record_not_meta_current_newest)
{
	reset_model();
	/* #754-fallback shape: the NEWEST record (by counter) names the
	 * OTHER slot -- whatever knocked its image out of active_slot_valid()
	 * upstream (ota.c only re-validates the metadata record's own CRC,
	 * not the image, so a perfectly CRC-valid metadata record here is
	 * consistent with "the bootloader didn't boot it"). The OLDER record
	 * names OTA_RUNNING_SLOT and is TRIAL -- what ota_boot_init() must
	 * gate on and what ota_confirm_tick() must target, NOT
	 * meta_current()'s newest pick (which would hand confirm the WRONG
	 * page as its erase-target override). */
	uint32_t len_other[2];
	len_other[TEST_OTHER_SLOT]   = 64u;
	len_other[TEST_RUNNING_SLOT] = 0u;
	write_meta_record_flags(
	    OTA_META_REC0, 5u, TEST_OTHER_SLOT, (uint8_t)(1u << TEST_OTHER_SLOT), len_other, 0u);

	uint32_t len_run[2];
	len_run[TEST_OTHER_SLOT]   = 0u;
	len_run[TEST_RUNNING_SLOT] = 64u;
	write_meta_record_flags(OTA_META_REC1,
	                        3u,
	                        TEST_RUNNING_SLOT,
	                        (uint8_t)(1u << TEST_RUNNING_SLOT),
	                        len_run,
	                        OTA_META_FLAG_TRIAL);

	ota_boot_init();
	zassert_true(ota_trial_unconfirmed(),
	             "the record naming us (REC1, counter 3) is TRIAL, even though REC0 "
	             "(counter 5) is the overall newest");

	ota_note_frame();
	ota_confirm_tick();
	zassert_equal(g_reset_calls, 1u);

	/* REC0 (the newest, but names the OTHER slot) must survive UNTOUCHED
	 * -- confirm must never have targeted it via meta_current(). */
	ota_meta_record_t rec0;
	zassert_true(read_meta_at(OTA_META_REC0, &rec0), "REC0 must not be touched by confirm");
	zassert_equal(rec0.active_slot, TEST_OTHER_SLOT);
	zassert_equal(rec0.counter, 5u);
	zassert_equal(rec0.flags, 0u, "REC0 was planted CONFIRMED and confirm must never touch it");

	/* REC1 (the record confirm actually targeted) must now be CONFIRMED.
	 * Its new counter is seeded from meta_commit()'s carried-forward
	 * `rec` -- the OVERALL newest record (REC0, counter 5), +1 -- NOT
	 * from REC1's own prior counter (3), +1.  That is deliberate: it
	 * guarantees the just-confirmed record becomes the new
	 * highest-counter (meta_current()'s "newest") record across BOTH
	 * pages, so a later meta_current() lookup (h_get_state, h_rollback)
	 * stops seeing REC0's stale, rejected pointer as "current" at all. */
	ota_meta_record_t rec1;
	zassert_true(read_meta_at(OTA_META_REC1, &rec1));
	zassert_equal(rec1.active_slot, TEST_RUNNING_SLOT);
	zassert_equal(rec1.flags, 0u, "confirmed record must clear TRIAL");
	zassert_equal(rec1.counter, 6u, "confirm must out-count REC0 so it becomes the new newest");

	ota_meta_record_t newest;
	uint32_t          which;
	zassert_true(meta_current_for_test(&newest, &which));
	zassert_equal(which, OTA_META_REC1, "the confirmed record must now be meta_current()'s newest");
}

/* ---- composition guard: last-resort boot of a still-gated TRIAL image
 * must gate (s_trial) WITHOUT self-heal prematurely confirming it ----- */

ZTEST(gd32_bridge_ota, test_boot_init_last_resort_composition_gates_without_premature_heal)
{
	reset_model();
	/* Same shape as the C7 test above: newest (REC0) names the OTHER,
	 * rejected slot and is TRIAL; older (REC1) names OTA_RUNNING_SLOT and
	 * is ALSO still TRIAL (the composed "last-resort boot of a
	 * still-unconfirmed image" shape ota_boot_select()'s last-resort
	 * pass can now produce). ota_boot_init() must gate (s_trial=true) AND
	 * must NOT self-heal REC0 away this boot -- doing so would stamp
	 * flags=0 on a record nothing has actually confirmed yet. */
	uint32_t len_other[2];
	len_other[TEST_OTHER_SLOT]   = 64u;
	len_other[TEST_RUNNING_SLOT] = 0u;
	write_meta_record_flags(OTA_META_REC0,
	                        5u,
	                        TEST_OTHER_SLOT,
	                        (uint8_t)(1u << TEST_OTHER_SLOT),
	                        len_other,
	                        OTA_META_FLAG_TRIAL);

	uint32_t len_run[2];
	len_run[TEST_OTHER_SLOT]   = 0u;
	len_run[TEST_RUNNING_SLOT] = 64u;
	write_meta_record_flags(OTA_META_REC1,
	                        3u,
	                        TEST_RUNNING_SLOT,
	                        (uint8_t)(1u << TEST_RUNNING_SLOT),
	                        len_run,
	                        OTA_META_FLAG_TRIAL);

	ota_boot_init();
	zassert_true(ota_trial_unconfirmed(), "gate: the record naming us is TRIAL");
	zassert_equal(g_reset_calls, 0u, "gating alone must not reset");

	/* Composition guard: REC0 must be UNTOUCHED -- self-heal must have
	 * been deferred, not fired prematurely. */
	ota_meta_record_t rec0;
	zassert_true(read_meta_at(OTA_META_REC0, &rec0), "self-heal must not have touched REC0");
	zassert_equal(rec0.active_slot, TEST_OTHER_SLOT);
	zassert_equal(rec0.counter, 5u);
	zassert_equal(rec0.flags, (uint8_t)OTA_META_FLAG_TRIAL);

	/* The confirm dance still works normally from here: note a frame,
	 * confirm, and check REC1 (our own record) is what gets confirmed. */
	ota_note_frame();
	ota_confirm_tick();
	zassert_equal(g_reset_calls, 1u);
	ota_meta_record_t rec1;
	zassert_true(read_meta_at(OTA_META_REC1, &rec1));
	zassert_equal(rec1.active_slot, TEST_RUNNING_SLOT);
	zassert_equal(rec1.flags, 0u);
}
