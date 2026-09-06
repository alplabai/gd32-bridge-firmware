/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge OTA — Path A state machine.  See ota.h + ota_layout.h for
 * the public wire and flash-layout contract.
 *
 * Safe-by-default: without -DBRIDGE_OTA_PARTITIONED the dispatcher returns
 * STATUS_NOSUPPORT and touches no flash.  Even when partitioned, the real
 * flash ops come from hal/fmc_ota.c (gd32 backend); on the stub backend the
 * weak no-ops below report "unsupported".
 *
 * SILICON-VALIDATED 2026-06-04 (bench, protocol v0.6): full cycle
 * BEGIN -> length-checked chunk stream at 25 MHz -> VERIFY -> COMMIT ->
 * boot into the new slot -> ROLLBACK -> boot back, proven by wire
 * build-id reads + the A/B metadata generation history.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "ota.h"
#include "ota_layout.h"
#include "crc32.h"
#include "fmc_ota.h"
#include "bootloader/bootloader.h" /* CMD_OTA_* */

/* ---- Weak flash seam (overridden by hal/fmc_ota.c on the gd32 backend) - */
__attribute__((weak)) bool ota_fmc_supported(void)
{
	return false;
}
__attribute__((weak)) bool ota_fmc_erase_range(uint32_t base, uint32_t len)
{
	(void)base;
	(void)len;
	return false;
}
__attribute__((weak)) bool ota_fmc_program(uint32_t addr, const uint8_t *data, size_t len)
{
	(void)addr;
	(void)data;
	(void)len;
	return false;
}
__attribute__((weak)) void ota_system_reset(void)
{
}
__attribute__((weak)) const void *ota_fmc_flash_ptr(uint32_t addr)
{
	return (const void *)(uintptr_t)addr;
}

#if defined(BRIDGE_OTA_PARTITIONED)

/* OTA_GET_STATE states (wire-visible). */
enum {
	OTA_ST_IDLE     = 0u,
	OTA_ST_READY    = 1u,
	OTA_ST_BUSY     = 2u,
	OTA_ST_VERIFIED = 3u,
	OTA_ST_ERROR    = 4u
};

/* Compile-time derivation of the slot THIS BUILD runs from (#3).
 * BRIDGE_APP_SLOT_BASE is the same -D that hal/gd32/init.c uses to
 * relocate VTOR at boot; comparing it against the flash-layout constants
 * turns "which slot may h_begin erase" into a BUILD invariant instead of
 * a runtime metadata read -- it cannot go stale and needs no flash read
 * or CRC.  That matters because metadata's `active_slot` CAN legitimately
 * diverge from what's executing: the bootloader's newest-first fallback
 * (boot_main.c:117-124, #754, intentional) boots an OLDER record when the
 * newest record's slot fails validation, and once that happens the
 * newest metadata names a slot that is not running.
 *
 * The #error below is the point, not a formality: if issue #2's dual-bank
 * boundary fix ever relocates OTA_SLOT_A_BASE/OTA_SLOT_B_BASE without a
 * matching CMakeLists.txt update (or vice versa), the build fails loudly
 * here instead of this guard silently going dead or resolving to the
 * wrong slot. */
#if !defined(BRIDGE_APP_SLOT_BASE)
#error \
    "BRIDGE_OTA_PARTITIONED requires BRIDGE_APP_SLOT_BASE (see CMakeLists.txt: gd32-bridge-slot-a / gd32-bridge-slot-b)"
#elif (BRIDGE_APP_SLOT_BASE) == OTA_SLOT_A_BASE
#define OTA_RUNNING_SLOT OTA_SLOT_A
#elif (BRIDGE_APP_SLOT_BASE) == OTA_SLOT_B_BASE
#define OTA_RUNNING_SLOT OTA_SLOT_B
#else
#error \
    "BRIDGE_APP_SLOT_BASE matches neither OTA_SLOT_A_BASE nor OTA_SLOT_B_BASE (ota_layout.h) -- slot geometry has diverged from the build"
#endif

/* [base, base+size) of the slot this build runs from, for the P3
 * interval-intersection guard in h_begin (defence in depth, see there). */
#define OTA_RUNNING_SLOT_BASE ((uint32_t)(BRIDGE_APP_SLOT_BASE))
#define OTA_RUNNING_SLOT_END  (OTA_RUNNING_SLOT_BASE + OTA_SLOT_SIZE)

static uint8_t  s_state    = OTA_ST_IDLE;
static uint8_t  s_inactive = (OTA_RUNNING_SLOT == OTA_SLOT_A) ? OTA_SLOT_B : OTA_SLOT_A;
static uint32_t s_img_len;
static uint32_t s_last_off;
static uint32_t s_expected_crc; /* from OTA_BEGIN (host supplies CRC up front) */
static uint32_t s_img_crc;      /* computed at OTA_VERIFY, reused at COMMIT */
static uint32_t s_fw_version;   /* from OTA_BEGIN v0.7 form (packed
                                 * major<<16|minor<<8|patch); 0 = host
                                 * sent the legacy 8-byte form = unknown */
static uint8_t  s_err;

/* Background slot-erase progress (#770).  BEGIN must NOT erase the whole
 * 236 KB slot inline: that is a ~1 s RAMFUNC loop with the SPI slave
 * unserviced, so the BEGIN reply is lost and the host's ota_begin() hangs.
 * Instead BEGIN arms the erase (state=BUSY) and acks immediately; the main
 * loop's ota_erase_tick() erases ONE OTA_PAGE_SIZE region per tick (~8 ms
 * of blackout, which the host's reply re-read absorbs) and flips to READY
 * when done.  The host already polls GET_STATE for READY before streaming. */
static bool     s_erasing;   /* an erase is armed + in progress */
static uint32_t s_erase_at;  /* next flash address to erase */
static uint32_t s_erase_end; /* one past the last address to erase */

static uint32_t rd_u32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void wr_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

/* ---- A/B metadata --------------------------------------------------- */
static bool meta_read(uint32_t addr, ota_meta_record_t *r)
{
	const ota_meta_record_t *p = (const ota_meta_record_t *)ota_fmc_flash_ptr(addr);
	if (p->magic != OTA_META_MAGIC || p->struct_version != OTA_META_STRUCT_VER) {
		return false;
	}
	const uint32_t c = ota_crc32(0u, (const uint8_t *)p, offsetof(ota_meta_record_t, rec_crc32));
	if (c != p->rec_crc32) {
		return false;
	}
	*r = *p;
	return true;
}

/* Pick the newer of two candidate metadata records (REC0 wins on a tie).
 * `a`/`b` are the result of meta_read() on OTA_META_REC0/REC1; `va`/`vb`
 * their validity.  Returns true + the winning record and which page holds
 * it; false if neither is valid (factory / corrupt) -- `*out` is left
 * untouched in that case, so callers that pre-zero it get a defined
 * zeroed record rather than an indeterminate one.
 *
 * Shared by meta_current() (below -- its signature and selection rule are
 * a settled public surface, see its own comment) and meta_commit(), which
 * needs the identical rule to seed its working record without a second,
 * redundant pair of flash reads through meta_current() itself. */
static bool meta_pick_newest(const ota_meta_record_t *a,
                             bool                     va,
                             const ota_meta_record_t *b,
                             bool                     vb,
                             ota_meta_record_t       *out,
                             uint32_t                *which_addr)
{
	if (va && vb) {
		if (a->counter >= b->counter) {
			*out        = *a;
			*which_addr = OTA_META_REC0;
		} else {
			*out        = *b;
			*which_addr = OTA_META_REC1;
		}
		return true;
	}
	if (va) {
		*out        = *a;
		*which_addr = OTA_META_REC0;
		return true;
	}
	if (vb) {
		*out        = *b;
		*which_addr = OTA_META_REC1;
		return true;
	}
	return false;
}

/* Returns true + the winning record (highest valid counter) and which page
 * holds it; false if neither record is valid (factory / corrupt).  This
 * signature and selection rule are a settled public surface (PR #73):
 * h_get_state and h_rollback both call it and must keep seeing exactly
 * this contract. */
static bool meta_current(ota_meta_record_t *out, uint32_t *which_addr)
{
	ota_meta_record_t a, b;
	const bool        va = meta_read(OTA_META_REC0, &a);
	const bool        vb = meta_read(OTA_META_REC1, &b);
	return meta_pick_newest(&a, va, &b, vb, out, which_addr);
}

/* Rank a metadata page for meta_commit's erase-target choice (#74) --
 * see the block comment above meta_commit for the full rationale.
 * `valid`/`r` are the result of meta_read() on that page.  A page that
 * failed validation (blank / torn / wrong magic or struct_version) ranks
 * lowest, so it is always the erase target when the other page holds
 * anything usable at all. */
static uint8_t meta_page_rank(bool valid, const ota_meta_record_t *r)
{
	if (!valid) {
		return 0u;
	}
	return (r->active_slot == OTA_RUNNING_SLOT) ? 2u : 1u;
}

/* Choose which of the two meta pages to erase + overwrite (#74).  It is
 * NOT simply "the other page from the current record" (alternating) --
 * that rule preserves the HIGHEST COUNTER, and the counter is not always
 * what keeps the part alive.  The bootloader boots newest-first with
 * fallback (boot_main.c:117-124, #754): when the newest record's slot
 * fails validation, it boots the OLDER record instead -- so in that
 * window the older record is the only thing naming a slot the bootloader
 * will actually boot, and the old "erase the non-newest page" rule erased
 * precisely that page.  A power cut inside the erase-then-program window
 * then left one CRC-valid record naming a dead slot, with no over-the-
 * wire recovery.
 *
 * Fix: rank each page (meta_page_rank() above) and erase the LOWER-ranked
 * one.  Rank 0 = no CRC-valid record on the page; rank 1 = a valid record
 * whose active_slot is not OTA_RUNNING_SLOT; rank 2 = a valid record
 * whose active_slot IS OTA_RUNNING_SLOT.  Read that as a contrapositive,
 * not a promise: a record that does NOT name OTA_RUNNING_SLOT is
 * definitely not what the bootloader booted THIS boot, so rank 1 is a
 * safe erase target; a record that DOES name it is merely a CANDIDATE.
 * OTA_RUNNING_SLOT is a build-time fact (see its derivation above), so
 * rank 2 is a cheap NECESSARY condition for "this is the record keeping
 * the part alive" -- it is not a re-validation of the record itself.
 * active_slot_valid() (boot_main.c) validates a record against its OWN
 * slot_valid / img_len[slot] / img_crc32[slot], not slot identity, so two
 * rank-2 records naming the same slot with different descriptors are not
 * interchangeable; no reachable path constructs that pair today, but rank
 * 2 does not rule it out by itself.  Real re-validation would mean
 * recomputing the bootloader's whole-slot CRC here, which is #49's
 * territory and deliberately not done in this rank.
 *
 * Rank 2's meaning assumes the bootloader in flash implements #754's
 * newest-first-WITH-FALLBACK.  That assumption is worth stating because
 * the bootloader at OTA_BOOTLOADER_BASE is never OTA-updated: a fielded
 * part keeps whatever was bench-flashed into it, so the next change to
 * this selection rule will be reasoned against the in-tree bootloader and
 * run against an older one.  It is safe in the one direction that
 * matters -- under a hypothetical newest-only bootloader a running part's
 * newest record necessarily names the running slot (or a different build
 * would be executing), so the ranks tie and this rule collapses to the
 * old alternation.
 *
 * On a tie (equal rank, both pages valid) the LOWER-COUNTER page is
 * erased, i.e. the newest record is preserved -- identical to the pre-#74
 * alternation rule, so behaviour outside the divergent window is
 * unchanged.  When NEITHER page is valid (rank 0 on both) there is no
 * counter to compare: the target is unconditionally OTA_META_REC0
 * (today's factory-init behaviour, see the inline comment below).
 *
 * The per-slot image descriptors still carry forward from the current
 * (newest) record regardless of which page that turns out to be; only
 * the new active slot's entry is rewritten (and only when `update_entry`
 * -- a ROLLBACK flips `active_slot` without touching the descriptors, so
 * the rolled-to slot keeps the len/CRC recorded when it was last
 * written). */
static bool meta_commit(uint8_t  active_slot,
                        bool     update_entry,
                        uint32_t fw_ver,
                        uint32_t img_len,
                        uint32_t img_crc)
{
	ota_meta_record_t a, b;
	const bool        va = meta_read(OTA_META_REC0, &a);
	const bool        vb = meta_read(OTA_META_REC1, &b);

	ota_meta_record_t rec;
	uint32_t          newest_addr = 0u;
	memset(&rec, 0, sizeof rec);
	(void)meta_pick_newest(&a, va, &b, vb, &rec, &newest_addr);
	/* rec now holds the NEWEST record's counter + slot_valid + per-slot
	 * table (or stays the zeroed record above if neither page validated)
	 * -- carried forward regardless of which page the ranking below
	 * targets for erase (#74): `a`/`b` were read into RAM before either
	 * page is touched, and rec.counter += 1u below keeps the new record
	 * strictly above whatever page survives the erase. */

	const uint8_t rank_a = meta_page_rank(va, &a);
	const uint8_t rank_b = meta_page_rank(vb, &b);
	uint32_t      target;
	if (rank_a != rank_b) {
		target = (rank_a < rank_b) ? OTA_META_REC0 : OTA_META_REC1;
	} else if (rank_a != 0u) {
		/* Same rank, both valid: tie-break by counter, preserving the
		 * newest -- reproduces meta_pick_newest()'s own REC0-on-tie
		 * pick, so this matches meta_commit's pre-#74 selection bit for
		 * bit whenever the ranks agree. */
		target = (a.counter >= b.counter) ? OTA_META_REC1 : OTA_META_REC0;
	} else {
		/* Neither page holds a CRC-valid record: no counter to compare,
		 * so target the first page (today's factory-init behaviour). */
		target = OTA_META_REC0;
	}

	if (!ota_fmc_erase_range(target, OTA_PAGE_SIZE)) {
		return false;
	}
	rec.magic          = OTA_META_MAGIC;
	rec.struct_version = OTA_META_STRUCT_VER;
	rec.counter += 1u;
	rec.active_slot = active_slot;
	rec.slot_valid |= (uint8_t)(1u << active_slot);
	if (update_entry) {
		rec.fw_version[active_slot] = fw_ver;
		rec.img_len[active_slot]    = img_len;
		rec.img_crc32[active_slot]  = img_crc;
	}
	rec.rec_crc32 = ota_crc32(0u, (const uint8_t *)&rec, offsetof(ota_meta_record_t, rec_crc32));
	return ota_fmc_program(target, (const uint8_t *)&rec, sizeof rec);
}

/* Flash base of the in-flight (inactive) slot.  s_inactive is set to
 * OTA_SLOT_A/B at BEGIN (the trusted internal boundary), so the checked
 * derivation (#741) never fails here; validate anyway for defence. */
static uint32_t ota_inactive_base(void)
{
	uint32_t base = 0u;
	if (!ota_slot_base_checked(s_inactive, &base)) {
		base = OTA_SLOT_A_BASE; /* unreachable: s_inactive is always A/B */
	}
	return base;
}

/* ---- opcode handlers ------------------------------------------------ */
static gd32_bridge_status_t
h_begin(const uint8_t *req, size_t len, uint8_t *reply, size_t cap, size_t *rlen)
{
	/* Host OTA_BEGIN req: size:u32, expected_crc32:u32
     * [, fw_major:u8, fw_minor:u8, fw_patch:u8  -- v0.7 additive form].
     * The version triple is recorded into the meta record at COMMIT
     * (ota_meta_record_t.fw_version[slot], which reserved the field
     * with "0 = unknown" from day one).  Length-tolerant by design:
     * an 8-byte legacy BEGIN means version-unknown, and older firmware
     * ignores the 3 trailing bytes of the v0.7 form -- additive both
     * directions (wire MINOR). */
	if (len < 8u) {
		return STATUS_INVAL;
	}
	/* Read the wire fields into LOCALS and validate them there; the
     * session statics are committed only once every check below has
     * passed (#131).  Assigning first and range-checking afterwards left
     * a rejected BEGIN's length live in s_img_len: the reject sets
     * OTA_ST_ERROR, but a background erase armed by an EARLIER, valid
     * BEGIN is still draining, and its completion writeback below
     * overwrites that ERROR with OTA_ST_READY.  h_verify's only guard is
     * `s_state != OTA_ST_READY`, so it then handed the rejected length
     * to ota_crc32 -- which has no bound of its own (src/crc32.c) --
     * walking past OTA_FLASH_END into reserved space, and for a large
     * enough value across SRAM and the 0x40000000 peripheral aperture
     * where reads have side effects.  No fault handlers are installed
     * (#36) and there is no watchdog (#54), so the resulting BusFault
     * escalates to HardFault and parks in the vendor Default_Handler. */
	const uint32_t img_len      = rd_u32(&req[0]);
	const uint32_t expected_crc = rd_u32(&req[4]);
	const uint32_t fw_version =
	    (len >= 11u) ? (((uint32_t)req[8] << 16) | ((uint32_t)req[9] << 8) | (uint32_t)req[10])
	                 : 0u;
	if (img_len == 0u || img_len > OTA_SLOT_SIZE) {
		s_state = OTA_ST_ERROR;
		s_err   = 1u;
		return STATUS_OUT_OF_RANGE;
	}
	/* The slot to erase is "the one I am NOT executing from", answered by
     * OTA_RUNNING_SLOT (build-derived, see its definition above) -- NOT by
     * inverting metadata's active_slot (#3).  Metadata can legitimately
     * name the slot that IS running (the bootloader's newest-first
     * fallback, boot_main.c:117-124/#754), and inverting a stale answer
     * used to arm the erase against the live image, vector table first.
     * OTA_RUNNING_SLOT needs no flash read and cannot go stale, so this
     * self-heals the divergence instead of propagating it. */
	s_inactive               = (OTA_RUNNING_SLOT == OTA_SLOT_A) ? OTA_SLOT_B : OTA_SLOT_A;
	s_last_off               = 0u;
	const uint32_t erase_at  = ota_inactive_base();
	const uint32_t erase_end = erase_at + OTA_SLOT_SIZE;
	/* P3 (#3): defence in depth, NOT the primary guard -- P1 above already
     * makes s_inactive always the OTHER slot from OTA_RUNNING_SLOT by
     * construction, so this branch is UNREACHABLE today: no production
     * path drives s_inactive to anything but what P1 just computed, so
     * this is UNTESTED (a test would need a seam into s_inactive that
     * production code has no reason to have).  Checked as INTERVAL
     * INTERSECTION rather than base equality because intersection is the
     * actual safety property ("will this erase touch what I execute") and
     * would still catch a future geometry bug that makes the erase range
     * and the running range overlap without their bases changing.  This
     * is an ADDRESS-RANGE check ONLY -- it does NOT catch flash-BANK
     * aliasing.  Issue #2's hazard is exactly that: slot A's tail
     * (0x08040000..0x08045000) shares bank 1 with slot B while the two
     * slot RANGES stay adjacent and disjoint (A ends where B begins), so
     * this intersection test reads false and does not fire for #2's
     * situation; a bank-overlap guard belongs to #2's (and #37's) fix,
     * not this one.  If this ever trips, refuse: the erase has not been
     * armed yet at this point, so refusing here costs nothing new.  Note
     * s_err=7u on trip is currently UNREADABLE by the host: s_err is
     * written at several sites in this file but read nowhere in this
     * repo, so a tripped guard looks on the wire like any other
     * OTA_ST_ERROR; surfacing s_err is a protocol change, tracked
     * separately. */
	if (erase_at < OTA_RUNNING_SLOT_END && OTA_RUNNING_SLOT_BASE < erase_end) {
		s_state = OTA_ST_ERROR;
		s_err   = 7u;
		return STATUS_INVAL;
	}
	/* Every check has passed: NOW commit the wire fields to the session
     * (#131).  Nothing above this line may write s_img_len,
     * s_expected_crc or s_fw_version -- that ordering is the invariant
     * h_verify and h_commit re-assert at their heads.
     *
     * Deliberately NOT disarming a previously-armed erase on the reject
     * paths above.  All of them return before `s_erasing = true` below,
     * so a rejected BEGIN can never arm one -- it could only CANCEL an
     * erase armed by an earlier, valid BEGIN.  CMD_OTA_ABORT is the
     * explicit host-driven cancel; letting a truncated or out-of-range
     * frame do the same silently would add a remote erase-cancel
     * surface.  The ERROR-then-READY writeback those rejects can still
     * produce is #9's mechanism (the pump clobbering a state the
     * dispatcher set), and belongs to #9's fix. */
	s_img_len      = img_len;
	s_expected_crc = expected_crc;
	s_fw_version   = fw_version;

	/* Arm the background erase and ack NOW -- do NOT erase inline (#770).
     * ota_erase_tick() walks the slot a page-region per main-loop tick;
     * state stays BUSY until it finishes, then flips to READY.  The host
     * gets this reply immediately and polls GET_STATE for READY before it
     * streams the first chunk (h_write rejects anything but READY). */
	s_erasing   = true;
	s_erase_at  = erase_at;
	s_erase_end = erase_end;
	s_state     = OTA_ST_BUSY;
	/* Host OTA_BEGIN reply: chunk_max:u16 (LE), target_slot:u8.
     * chunk_max accounts for the offset:u32 + len:u8 header (v0.6). */
	if (cap >= 3u) {
		const uint16_t chunk_max = (uint16_t)(GD32_BRIDGE_MAX_PAYLOAD_BYTES - 5u);
		reply[0]                 = (uint8_t)(chunk_max & 0xFFu);
		reply[1]                 = (uint8_t)(chunk_max >> 8);
		reply[2]                 = s_inactive;
		*rlen                    = 3u;
	}
	return STATUS_OK;
}

static gd32_bridge_status_t
h_write(const uint8_t *req, size_t len, uint8_t *reply, size_t cap, size_t *rlen)
{
	/* Host OTA_WRITE_CHUNK req: offset:u32, len:u8, data[len].
     * Reply: received_bytes:u32.
     *
     * The explicit len byte is LOAD-BEARING (protocol v0.6,
     * silicon-caught 2026-06-04): the slave can capture a frame merged
     * with the following transaction's zero filler (the FMC program
     * window swallows CS edges), and for the ~1-in-256 chunk whose
     * frame CRC is byte-palindromic the zero-extended span still
     * passes the span CRC (CRC-CCITT self-consumption: frame + own
     * CRC + zeros hashes to 0x0000).  Cross-checking the embedded
     * length against the span-derived length rejects any such
     * extension regardless of CRC coincidences -- and without
     * poisoning the session (plain STATUS_INVAL, state untouched). */
	if (s_state != OTA_ST_READY) {
		/* No BEGIN-opened session: the inactive slot is not erased and
         * s_img_len/s_expected_crc are unset -- programming here would
         * corrupt the slot.  Host must (re-)issue OTA_BEGIN. */
		return STATUS_NOT_READY;
	}
	if (len < 5u) {
		return STATUS_INVAL;
	}
	const uint32_t off  = rd_u32(&req[0]);
	const size_t   dlen = req[4];
	if (dlen == 0u || dlen != len - 5u) {
		return STATUS_INVAL; /* extended/truncated capture: drop */
	}
	if (off > OTA_SLOT_SIZE || dlen > OTA_SLOT_SIZE - off) {
		s_state = OTA_ST_ERROR;
		s_err   = 3u;
		return STATUS_OUT_OF_RANGE;
	}
	/* The transport is AT-LEAST-ONCE: the slave can decode a request
     * twice (silicon-caught 2026-06-04: chunk #256 replayed
     * deterministically), and re-programming ECC flash hard-faults.
     * Make replays idempotent: a chunk entirely below the high-water
     * mark is compared against what's in flash -- identical bytes ack
     * without programming, different bytes are real corruption.  A
     * PARTIAL overlap still falls through to the program path (and
     * PGERRs) -- fixed-size streaming never produces one. */
	if (off + (uint32_t)dlen <= s_last_off) {
		const uint8_t *flash = (const uint8_t *)ota_fmc_flash_ptr(ota_inactive_base() + off);
		if (memcmp(flash, &req[5], dlen) == 0) {
			s_state = OTA_ST_READY;
			if (cap >= 4u) {
				wr_u32(&reply[0], s_last_off);
				*rlen = 4u;
			}
			return STATUS_OK;
		}
		s_state = OTA_ST_ERROR;
		s_err   = 4u;
		return STATUS_IO;
	}
	s_state = OTA_ST_BUSY;
	if (!ota_fmc_program(ota_inactive_base() + off, &req[5], dlen)) {
		s_state = OTA_ST_ERROR;
		s_err   = 4u;
		return STATUS_IO;
	}
	if (off + (uint32_t)dlen > s_last_off) {
		s_last_off = off + (uint32_t)dlen; /* cumulative high-water = received bytes */
	}
	s_state = OTA_ST_READY;
	if (cap >= 4u) {
		wr_u32(&reply[0], s_last_off);
		*rlen = 4u;
	}
	return STATUS_OK;
}

static gd32_bridge_status_t h_verify(uint8_t *reply, size_t cap, size_t *rlen)
{
	/* Host OTA_VERIFY req: empty (CRC was supplied at BEGIN).
     * Reply: computed_crc32:u32, verified:u8. */
	if (s_state != OTA_ST_READY) {
		/* Without a BEGIN-opened session s_img_len is 0: CRC over zero
         * bytes would "verify" trivially and a subsequent COMMIT would
         * write img_len=0 metadata that no bootloader accepts -- a
         * protocol-misuse brick.  Refuse instead. */
		return STATUS_NOT_READY;
	}
	/* Re-assert the length invariant at the POINT OF USE (#131).  A state
     * enum is not a length bound: OTA_ST_READY says "the slot is erased
     * and writable", not "s_img_len is in range".  Unreachable today --
     * h_begin commits s_img_len only after range-checking it -- but this
     * is the guard that has to hold if any future path writes
     * OTA_ST_READY without having gone through h_begin's validation,
     * which is exactly the shape the erase pump's unconditional
     * writeback already has (#9). */
	if (s_img_len == 0u || s_img_len > OTA_SLOT_SIZE) {
		s_state = OTA_ST_ERROR;
		s_err   = 1u;
		return STATUS_INVAL;
	}
	s_img_crc = ota_crc32(0u, (const uint8_t *)ota_fmc_flash_ptr(ota_inactive_base()), s_img_len);
	const bool ok = (s_img_crc == s_expected_crc);
	s_state       = ok ? OTA_ST_VERIFIED : OTA_ST_ERROR;
	if (!ok) {
		s_err = 5u;
	}
	if (cap >= 5u) {
		wr_u32(&reply[0], s_img_crc);
		reply[4] = ok ? 1u : 0u;
		*rlen    = 5u;
	}
	return STATUS_OK;
}

static gd32_bridge_status_t h_commit(void)
{
	if (s_state != OTA_ST_VERIFIED) {
		return STATUS_NOT_READY;
	}
	/* Same re-assertion as h_verify (#131): s_img_len is handed to
     * ota_image_bootable() and written into the meta record below, so the
     * bound is re-checked here rather than inherited from a state enum. */
	if (s_img_len == 0u || s_img_len > OTA_SLOT_SIZE) {
		s_state = OTA_ST_ERROR;
		s_err   = 1u;
		return STATUS_INVAL;
	}
	/* A verified (CRC-matching) image can still be unbootable -- a
	 * one-byte or truncated image with a matching host CRC (#755).
	 * Refuse to activate metadata that would brick the part on reboot. */
	if (!ota_image_bootable(ota_inactive_base(),
	                        (const uint8_t *)ota_fmc_flash_ptr(ota_inactive_base()),
	                        s_img_len)) {
		s_state = OTA_ST_ERROR;
		s_err   = 6u;
		return STATUS_INVAL;
	}
	if (!meta_commit(
	        s_inactive, true, s_fw_version /* 0 = legacy BEGIN, unknown */, s_img_len, s_img_crc)) {
		s_state = OTA_ST_ERROR;
		s_err   = 6u;
		return STATUS_IO;
	}
	ota_system_reset(); /* no return on real silicon */
	return STATUS_OK;
}

static gd32_bridge_status_t h_rollback(void)
{
	/* Host OTA_ROLLBACK: no payload either direction (status only). */

	/* #147: the state guard every sibling handler has and this one did
     * not (compare h_write, h_verify, h_commit).  Without it, ROLLBACK
     * dispatched from a transport ISR straight into meta_commit -> the
     * FMC funnel while the BASE-level erase pump was mid-page-walk with
     * the FMC unlocked -- the collision hal/fmc_ota.c's funnel interlock
     * now refuses outright.  This guard is the other half: refuse the
     * command at the state machine rather than let it reach the funnel
     * and fail there, so the host gets an accurate STATUS_BUSY instead
     * of a STATUS_IO that reads like a flash fault.
     *
     * ROLLBACK is only meaningful with no update in flight, so it is
     * allowed from IDLE and from ERROR (the recovery case) and refused
     * from BUSY / READY / VERIFIED.  CMD_OTA_ABORT is the documented way
     * out of an in-flight session and already cancels the erase, so a
     * host that genuinely wants to abandon an update and roll back
     * issues ABORT then ROLLBACK.
     *
     * Behaviour change, deliberate: ROLLBACK previously succeeded from
     * any state.  It ends in ota_system_reset(), so the in-flight
     * session died with the reset anyway -- what it did NOT do was
     * survive the FMC collision on the way there. */
	if (s_state != OTA_ST_IDLE && s_state != OTA_ST_ERROR) {
		return STATUS_BUSY;
	}

	ota_meta_record_t cur;
	uint32_t          which;
	if (!meta_current(&cur, &which)) {
		return STATUS_INVAL;
	}
	/* Deliberately METADATA's cur.active_slot -- already read above by the
     * meta_current() guard this function returns on -- not OTA_RUNNING_SLOT
     * (#3).  ROLLBACK is an operation ON the metadata state machine ("flip
     * active_slot to the other slot"), so it must read the same source of
     * truth it is about to write, from the SAME snapshot the slot_valid /
     * img_len checks just below use (a second, independent metadata read
     * here could race an ISR-dispatched OTA command and validate one
     * snapshot while flipping based on another). In the divergent state
     * (metadata names a slot the bootloader did not boot), this converges
     * metadata back toward what the bootloader actually chose; using the
     * build-derived running slot here instead would commit the part to the
     * slot that just failed validation and re-manufacture the exact
     * divergence P1/h_begin now self-heals. */
	const uint8_t other = (cur.active_slot == OTA_SLOT_A) ? OTA_SLOT_B : OTA_SLOT_A;
	if ((cur.slot_valid & (uint8_t)(1u << other)) == 0u || cur.img_len[other] == 0u ||
	    cur.img_len[other] > OTA_SLOT_SIZE) {
		return STATUS_INVAL; /* no valid fallback slot */
	}
	/* Flip active to `other` WITHOUT touching the per-slot descriptors
     * (update_entry=false): the bootloader validates the rolled-to slot
     * against the len/CRC recorded when that slot was last committed. */
	if (!meta_commit(other, false, 0u, 0u, 0u)) {
		return STATUS_IO;
	}
	ota_system_reset();
	return STATUS_OK;
}

static gd32_bridge_status_t h_get_state(uint8_t *reply, size_t cap, size_t *rlen)
{
	/* Host OTA_GET_STATE reply: state:u8, active:u8, pending:u8, boot_count:u16 (LE).
     * `boot_count` is mapped to the metadata update counter (generation).
     *
     * `active` reports OTA_RUNNING_SLOT (build-derived), NOT metadata's
     * active_slot (#3).  The two agree except in the divergent window the
     * bootloader's newest-first fallback can create (boot_main.c:117-124,
     * #754): there, metadata's answer is a LIE about what is executing,
     * while OTA_RUNNING_SLOT is a build-time fact.  This is what preserves
     * host observability of the divergence now that h_begin self-heals
     * around it instead of refusing outright -- without this the host
     * would see a comforting but false `active`, same wire byte, same
     * format, only the source changes. */
	ota_meta_record_t cur;
	uint32_t          which;
	uint16_t          gen = 0u;
	if (meta_current(&cur, &which)) {
		gen = (uint16_t)cur.counter;
	}
	const bool in_progress =
	    (s_state == OTA_ST_READY || s_state == OTA_ST_BUSY || s_state == OTA_ST_VERIFIED);
	if (cap >= 5u) {
		reply[0] = s_state;
		reply[1] = OTA_RUNNING_SLOT;
		reply[2] = in_progress ? s_inactive : 0xFFu; /* 0xFF = none pending */
		reply[3] = (uint8_t)(gen & 0xFFu);
		reply[4] = (uint8_t)(gen >> 8);
		*rlen    = 5u;
	}
	return STATUS_OK;
}

/* Background erase pump (#770): erase ONE OTA_PAGE_SIZE region per call
 * from the main loop (bridge_hw_tick).  Each call is a bounded ~8 ms
 * blackout the host's reply re-read absorbs -- unlike the old inline
 * whole-slot erase that stalled BEGIN's reply for ~1 s.  Flips the OTA
 * state machine to READY once the slot is fully erased, or ERROR on a
 * failed page.  No-op unless an erase is armed. */
void ota_erase_tick(void)
{
	if (!s_erasing) {
		return;
	}
	if (!ota_fmc_erase_range(s_erase_at, OTA_PAGE_SIZE)) {
		s_erasing = false;
		s_state   = OTA_ST_ERROR;
		s_err     = 2u;
		return;
	}
	s_erase_at += OTA_PAGE_SIZE;
	if (s_erase_at >= s_erase_end) {
		s_erasing = false;
		s_state   = OTA_ST_READY;
	}
}

gd32_bridge_status_t ota_dispatch(uint8_t        cmd,
                                  const uint8_t *req_payload,
                                  size_t         req_payload_len,
                                  uint8_t       *reply_payload,
                                  size_t         reply_payload_cap,
                                  size_t        *reply_payload_len)
{
	*reply_payload_len = 0u;
	if (!ota_fmc_supported()) {
		return STATUS_NOSUPPORT;
	}
	switch (cmd) {
	case CMD_OTA_BEGIN:
		return h_begin(
		    req_payload, req_payload_len, reply_payload, reply_payload_cap, reply_payload_len);
	case CMD_OTA_WRITE_CHUNK:
		return h_write(
		    req_payload, req_payload_len, reply_payload, reply_payload_cap, reply_payload_len);
	case CMD_OTA_VERIFY:
		return h_verify(reply_payload, reply_payload_cap, reply_payload_len);
	case CMD_OTA_COMMIT:
		return h_commit();
	case CMD_OTA_ROLLBACK:
		return h_rollback();
	case CMD_OTA_GET_STATE:
		return h_get_state(reply_payload, reply_payload_cap, reply_payload_len);
	case CMD_OTA_ABORT:
		s_erasing = false; /* cancel any in-flight background erase (#770) */
		s_state   = OTA_ST_IDLE;
		s_err     = 0u;
		return STATUS_OK;
	default:
		return STATUS_NOSUPPORT;
	}
}

#else /* !BRIDGE_OTA_PARTITIONED — OTA inert (safe default) */

gd32_bridge_status_t ota_dispatch(uint8_t        cmd,
                                  const uint8_t *req_payload,
                                  size_t         req_payload_len,
                                  uint8_t       *reply_payload,
                                  size_t         reply_payload_cap,
                                  size_t        *reply_payload_len)
{
	(void)cmd;
	(void)req_payload;
	(void)req_payload_len;
	(void)reply_payload;
	(void)reply_payload_cap;
	*reply_payload_len = 0u;
	return STATUS_NOSUPPORT;
}

/* OTA inert: no background erase to pump. */
void ota_erase_tick(void)
{
}

#endif /* BRIDGE_OTA_PARTITIONED */
