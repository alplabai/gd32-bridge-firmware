/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge OTA — flash layout + on-flash structures (Path A).
 *
 * OTA_PAGE_SIZE below is the LAYOUT granule (2 KB), not necessarily the
 * silicon erase page: the V2N's GD32G553 ships in DUAL-BANK mode
 * (OBCTL.DBS = 1, silicon-verified 2026-06-04), where the real page is
 * 1 KB (MAIN_FLASH_PAGE_SIZE_DBANK) and addresses >= 0x08040000 are
 * bank 1.  2 KB is a multiple of the page size in both bank modes, so
 * all region bases/sizes stay valid; hal/fmc_ota.c walks the REAL page
 * size and selects the bank per address.  Public slot map:
 *
 *   0x08000000  bootloader   32 KB   (never erased by OTA)
 *   0x08008000  metadata      8 KB   (A/B records, one per page)
 *   0x0800A000  slot A      236 KB
 *   0x08045000  slot B      236 KB
 *   0x08080000  end
 *
 * NOTE: OTA self-flashing needs the partitioned bootloader layout
 * (-DBRIDGE_OTA_PARTITIONED, paired with the bootloader image +
 * slot-linked app).  The default full-flash build leaves OTA inert
 * (ota_dispatch -> STATUS_NOSUPPORT).  Path A was silicon-validated
 * 2026-06-04 (bench, protocol v0.6) for the A->B update + rollback
 * direction; B->A has NOT been exercised.  It remains HIL-gated: a
 * bad bootloader bricks the part, and this HW revision has no
 * host-driven SWD reflash, so recovery needs a bench SWD probe on the
 * physical board (see src/bootloader/DESIGN.md).
 */
#ifndef GD32_BRIDGE_OTA_LAYOUT_H
#define GD32_BRIDGE_OTA_LAYOUT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#define OTA_PAGE_SIZE 0x00000800u /* 2 KB single-bank page */

#define OTA_BOOTLOADER_BASE 0x08000000u
#define OTA_BOOTLOADER_SIZE 0x00008000u /* 32 KB */
#define OTA_META_BASE       0x08008000u
#define OTA_META_SIZE       0x00002000u /* 8 KB = 4 pages */
#define OTA_SLOT_A_BASE     0x0800A000u
#define OTA_SLOT_B_BASE     0x08045000u
#define OTA_SLOT_SIZE       0x0003B000u /* 236 KB */
#define OTA_FLASH_END       0x08080000u

/* Two metadata records on separate pages -- a power-fail-safe A/B commit.
 * meta_commit() in ota.c chooses which page to erase + overwrite BY RANK
 * (see the block comment above meta_commit), not by simple alternation. */
#define OTA_META_REC0 (OTA_META_BASE + 0u * OTA_PAGE_SIZE)
#define OTA_META_REC1 (OTA_META_BASE + 1u * OTA_PAGE_SIZE)

enum { OTA_SLOT_A = 0u, OTA_SLOT_B = 1u };

/* An OTA image is a raw Cortex-M vector-table-first binary: initial MSP at
 * +0 and reset vector at +4.  Length, firmware version and CRC-32 live in
 * ota_meta_record_t below, outside the image.  Do not place a container
 * header at the slot base: ota_image_bootable() and the bootloader consume
 * those first two words directly.  This part also has no public-key or
 * message-hash engine for authenticated boot; see SECURITY.md (#50). */

/* A/B metadata record — the bootloader picks the highest `counter` with a
 * valid `rec_crc32`, then boots `active_slot` if the slot's metadata-recorded
 * CRC verifies over the slot contents.
 *
 * The image descriptors are PER-SLOT (struct v2): a ROLLBACK only flips
 * `active_slot` and must leave the rolled-to slot's len/CRC intact for the
 * bootloader to validate against — a single active-slot descriptor (v1)
 * left rollback with nothing valid to write, bricking the part on the
 * next boot. */
#define OTA_META_MAGIC      0x4F544D31u /* "OTM1" */
#define OTA_META_STRUCT_VER 2u

/* Trial/confirm + watchdog fallback (bench fact 2026-09-26, E1M-V2M103):
 * an OTA'd slot-B image passed VERIFY and COMMIT, then hung before
 * main() (stock SystemInit spinning on HXTALSTB) -- the bootloader had
 * no watchdog/confirm and jumped to the newest CRC-valid slot
 * unconditionally, bricking the bridge on both transports until an SWD
 * recovery.  `flags` marks a freshly-committed/rolled-back slot TRIAL
 * until the app notes a live wire frame and confirms it (src/ota.c
 * ota_boot_init/ota_trial_unconfirmed/ota_note_frame/ota_confirm_tick);
 * the bootloader (src/boot/boot_main.c) arms the FWDGT before jumping to
 * a TRIAL candidate so a hang before main() reverts to the previous slot
 * instead of repeating the bench incident. */
#define OTA_META_FLAG_TRIAL 0x01u

typedef struct {
	uint32_t magic;          /* OTA_META_MAGIC */
	uint32_t struct_version; /* OTA_META_STRUCT_VER */
	uint32_t counter;        /* monotonic; highest valid record wins */
	uint8_t  active_slot;    /* OTA_SLOT_A | OTA_SLOT_B */
	uint8_t  slot_valid;     /* bit0 = slot A valid, bit1 = slot B valid */
	uint8_t  flags;          /* OTA_META_FLAG_* -- TRIAL until confirmed */
	uint8_t  _pad1;
	uint32_t fw_version[2]; /* per-slot firmware semver (A, B); 0 = unknown */
	uint32_t img_len[2];    /* per-slot image length (A, B) */
	uint32_t img_crc32[2];  /* per-slot image CRC-32 (A, B) */
	uint32_t rec_crc32;     /* CRC-32 over this record excluding this field */
} ota_meta_record_t;

/* On-flash layout guards (#733).  The metadata struct is serialized by raw
 * byte access: the bootloader byte-copies a flash record and CRCs the raw
 * bytes, and ota.c programs it to flash verbatim.  Its in-memory layout
 * therefore IS the on-flash / CRC-covered format -- any compiler
 * padding change or field reorder silently invalidates every stored
 * record's CRC and can brick the boot path.  These are all naturally
 * aligned (u32/u8 + explicit pad), so the layout is deterministic without
 * packing (which would add unaligned-access hazards).  Assert the exact
 * size + every field offset so a struct edit fails the build instead of
 * the field on flash; rec_crc32's offset also anchors the CRC span the
 * bootloader/app compute (offsetof(..., rec_crc32)). */
_Static_assert(sizeof(ota_meta_record_t) == 44u, "ota_meta_record_t on-flash size drifted");
_Static_assert(offsetof(ota_meta_record_t, magic) == 0u, "meta.magic offset");
_Static_assert(offsetof(ota_meta_record_t, struct_version) == 4u, "meta.struct_version offset");
_Static_assert(offsetof(ota_meta_record_t, counter) == 8u, "meta.counter offset");
_Static_assert(offsetof(ota_meta_record_t, active_slot) == 12u, "meta.active_slot offset");
_Static_assert(offsetof(ota_meta_record_t, slot_valid) == 13u, "meta.slot_valid offset");
_Static_assert(offsetof(ota_meta_record_t, flags) == 14u, "meta.flags offset");
_Static_assert(offsetof(ota_meta_record_t, fw_version) == 16u, "meta.fw_version offset");
_Static_assert(offsetof(ota_meta_record_t, img_len) == 24u, "meta.img_len offset");
_Static_assert(offsetof(ota_meta_record_t, img_crc32) == 32u, "meta.img_crc32 offset");
_Static_assert(offsetof(ota_meta_record_t, rec_crc32) == 40u, "meta.rec_crc32 offset");

/* Derive a slot's flash base with EXPLICIT validation.  The old
 * `ota_slot_base` silently mapped every non-B value (incl. a corrupt
 * active_slot from persisted metadata) to slot A, so callers could not
 * tell a rejected slot from slot A and might erase / program / verify /
 * boot the wrong region (#741).  This returns false for any slot that
 * is not OTA_SLOT_A/B; callers MUST check before using *base_out. */
static inline bool ota_slot_base_checked(uint8_t slot, uint32_t *base_out)
{
	if (slot == OTA_SLOT_A) {
		*base_out = OTA_SLOT_A_BASE;
		return true;
	}
	if (slot == OTA_SLOT_B) {
		*base_out = OTA_SLOT_B_BASE;
		return true;
	}
	return false;
}

/* Boot-time trial/confirm gate: a TRIAL candidate that already ran a
 * watchdog reset on this power cycle must not be re-tried -- it hung
 * before confirming once and the fallback slot is the safe choice.
 * A non-TRIAL (confirmed) candidate is never rejected on this basis. */
static inline bool ota_boot_candidate_ok(const ota_meta_record_t *r, bool wdt_fired)
{
	return !(((r->flags & OTA_META_FLAG_TRIAL) != 0u) && wdt_fired);
}

/* Composed bootloader candidate-selection loop (#754's newest-first
 * fallback + the trial/watchdog gate + a last-resort pass) -- pure and
 * host-testable, so src/boot/boot_main.c and its test suite share the
 * exact same decision instead of a mirrored copy that can drift.  It does
 * NOT touch flash: `cands` are records already resident in RAM (read by
 * meta_read()/meta_candidates() on real hardware, or planted directly by
 * a test), newest-first; `valid[i]` is the caller's precomputed
 * active_slot_valid(cands[i]) (that check needs a real image CRC walk
 * over flash, which is exactly what a host test fakes -- keeping it out
 * of this function is what makes the function itself flash-free); `n` is
 * 0..2; `wdt_fired` is the reset cause read once by the caller.
 *
 * Pass 1 (strict, ordinary #754 behaviour): the first candidate that is
 * both valid and passes ota_boot_candidate_ok() wins.
 *
 * Pass 2 (last resort): a bootloader must never idle while a CRC-valid,
 * vector-valid image exists, even if every such candidate happens to be
 * an unconfirmed TRIAL that already burned a watchdog reset this power
 * cycle.  If pass 1 finds nothing, boot the newest candidate that is
 * valid but was rejected ONLY by the trial/watchdog gate -- under a
 * freshly-armed watchdog (the caller re-arms FWDGT for any TRIAL
 * candidate regardless of which pass picked it), so a transient failure
 * gets another confirm/revert cycle instead of a guaranteed brick.
 * `*last_resort_out` reports whether this pass had to fire.
 *
 * Returns the winning index into `cands`/`valid` (0..n-1), or -1 if
 * NOTHING is even valid -- the genuine "nothing to boot" case the
 * recovery WFI loop exists for. */
static inline int ota_boot_select(const ota_meta_record_t **cands,
                                  const bool               *valid,
                                  int                       n,
                                  bool                      wdt_fired,
                                  bool                     *last_resort_out)
{
	*last_resort_out = false;
	for (int i = 0; i < n; ++i) {
		if (valid[i] && ota_boot_candidate_ok(cands[i], wdt_fired)) {
			return i;
		}
	}
	for (int i = 0; i < n; ++i) {
		if (valid[i] && !ota_boot_candidate_ok(cands[i], wdt_fired)) {
			*last_resort_out = true;
			return i;
		}
	}
	return -1;
}

/* Minimum bootable image = at least the initial-MSP + reset-vector
 * words at the slot base (the boot path jumps via those two). */
#define OTA_IMG_MIN_LEN 8u

/* GD32G553 SRAM window a plausible initial MSP must fall in.  Upper
 * bound is generous (covers >128 KB parts) so a valid image is never
 * rejected; the point is to reject a garbage MSP, not size RAM exactly. */
#define OTA_SRAM_BASE 0x20000000u
#define OTA_SRAM_END  0x20040000u

/* Semantic bootability check for a slot's image, beyond the host CRC
 * (#755).  A CRC-valid one-byte / truncated / vector-less image would
 * otherwise pass COMMIT and boot -- the bootloader then reads incomplete
 * MSP/reset words and branches to an invalid address, bricking the part.
 * Require: length covers the MSP+reset head; the initial MSP points into
 * SRAM and is word-aligned; the reset vector lands inside the image and
 * has the Thumb bit set.
 *
 * @p base is the slot's ABSOLUTE flash base (for the reset-range check);
 * @p img is a readable pointer to the image bytes (identical to base on
 * memory-mapped silicon; a HAL-provided pointer under the OTA fake-flash
 * unit tests); @p len is the metadata-recorded, CRC-checked length. */
static inline bool ota_image_bootable(uint32_t base, const uint8_t *img, uint32_t len)
{
	if (len < OTA_IMG_MIN_LEN || img == 0) {
		return false;
	}
	uint32_t msp, reset;
	/* byte-wise load: img may be unaligned and const */
	msp   = (uint32_t)img[0] | ((uint32_t)img[1] << 8) | ((uint32_t)img[2] << 16) |
	        ((uint32_t)img[3] << 24);
	reset = (uint32_t)img[4] | ((uint32_t)img[5] << 8) | ((uint32_t)img[6] << 16) |
	        ((uint32_t)img[7] << 24);
	if ((msp & 3u) != 0u) {
		return false; /* MSP must be word-aligned */
	}
	if (msp < OTA_SRAM_BASE || msp > OTA_SRAM_END) {
		return false; /* MSP must point into SRAM */
	}
	if ((reset & 1u) == 0u) {
		return false; /* reset vector must have the Thumb bit set */
	}
	const uint32_t reset_addr = reset & ~1u;
	if (reset_addr < base || reset_addr >= base + len) {
		return false; /* reset must land inside the image */
	}
	return true;
}

/* Trial-capability marker (bench fact 2026-09-26 follow-up): the
 * downgrade guard that decides whether COMMIT/ROLLBACK marks a slot
 * OTA_META_FLAG_TRIAL used to trust the fw_version the HOST declared in
 * OTA_BEGIN (the now-removed OTA_TRIAL_MIN_FW_VERSION/
 * fw_version_trial_capable() in ota.c) -- but the 2026-09-26 bench
 * incident this whole trial/confirm dance responds to was the host
 * declaring the BAD image's TRUE, pre-fix version at OTA_BEGIN, which
 * that guard would have believed and committed the same image WITHOUT
 * trial protection. Eligibility must come from the IMAGE ITSELF: every
 * app image built from this branch onward plants this 20-byte struct
 * right after its vector table (toolchain/gd32g553_app_slot.ld.in's
 * `.trial_marker` section, src/trial_marker.c) and
 * ota_image_trial_capable() below scans a bounded window of the SLOT'S
 * OWN flash bytes for it at COMMIT/ROLLBACK -- not a numeric offset
 * shared with the linker script, so a future vector-table size change
 * (an added IRQ, a different startup file) does not need a matching
 * constant edited here. tools/check_trial_marker.py runs the identical
 * scan against the built .bin as a build-time gate: a slot image
 * produced without a findable marker fails the build.
 *
 * Policy, stated plainly (do not soften this on a future edit): ONLY an
 * image that carries this marker (with the confirm-capability bit set)
 * ever gets the FWDGT safety net. A markerless image -- any build older
 * than this change, INCLUDING the 2026-09-26 incident image itself
 * (built from a pre-marker tree) -- commits/rolls back CONFIRMED and is
 * NOT protected: a bad markerless image still needs a bench SWD
 * recovery, exactly as before this fix existed. That is a deliberate
 * trade-off, not an oversight: forcing TRIAL onto a markerless image
 * instead would make the FWDGT revert it (or, with no older CONFIRMED
 * record to fall back to, reset-loop it every ~32.8 s) even when that
 * image is perfectly healthy, because an old app has no confirm path at
 * all -- it never calls ota_note_frame()/ota_confirm_tick(), so nothing
 * would ever clear a TRIAL flag forced onto it. The previous cut's
 * "unknown fw_version defaults to TRIAL" policy is deliberately NOT
 * carried forward as "no marker defaults to TRIAL": that default was a
 * judgement call under a weaker signal (a declared version the host
 * could get wrong, as 2026-09-26 proved) and does not automatically
 * transfer to a signal read from the image's own bytes -- whether some
 * other opt-in default belongs here is a maintainer decision left open
 * on PR #246. */
#define OTA_TRIAL_MARKER_MAGIC_BYTES \
	{ 'G', 'D', '3', '2', 'B', 'R', 'I', 'D', 'G', 'E', '-', 'T', 'R', 'I', 'A', 'L' }
#define OTA_TRIAL_MARKER_STRUCT_VER 1u
/* bit0: the image implements the confirm handshake (ota_note_frame() /
 * ota_confirm_tick()) -- the only capability this marker records today. */
#define OTA_TRIAL_CAP_CONFIRM 0x0001u

typedef struct {
	uint8_t  magic[16];
	uint16_t struct_version;
	uint16_t capability_flags;
} ota_trial_marker_t;
_Static_assert(sizeof(ota_trial_marker_t) == 20u, "ota_trial_marker_t on-flash size drifted");

/* Bounded scan window: comfortably covers the GD32G553 vector table
 * (154 vectors * 4 B = 616 B, ALIGN(4)) plus the marker this build
 * places right after it, with slack for a future startup-file change --
 * generous on purpose so this reader needs no numeric offset kept in
 * lockstep with the linker script. */
#define OTA_TRIAL_SCAN_LIMIT 2048u

/* Minimum offset a genuine marker can ever sit at: the 16-word (64-byte)
 * ARMv8-M CORE exception vector block (initial SP + 15 core exceptions)
 * that precedes ANY device-specific IRQ vector, on every Cortex-M part,
 * regardless of how many device-specific vectors this or a future
 * silicon revision adds after it. A match before this offset cannot be
 * the planted marker -- it would have to overlap the core vector table
 * itself -- so the scan below refuses it rather than treat a coincidental
 * byte pattern there as a real marker. This is a FLOOR, not the real
 * (larger, GD32G553-specific) vector table size: see OTA_TRIAL_SCAN_LIMIT's
 * comment for why the reader deliberately does not hardcode that exact,
 * device-specific value either. */
#define OTA_TRIAL_MARKER_MIN_OFFSET 64u

static inline uint16_t ota_trial_rd_u16(const uint8_t *p)
{
	return (uint16_t)((uint32_t)p[0] | ((uint32_t)p[1] << 8));
}

/* True iff `img` (length `len`, the SLOT'S OWN recorded image length --
 * never the host-declared OTA_BEGIN version) carries a confirm-capable
 * trial marker within the bounded scan window, at or past
 * OTA_TRIAL_MARKER_MIN_OFFSET. An image with no marker at all (every
 * pre-marker build) or a struct_version this reader does not recognise
 * is NOT trial-capable -- see this file's policy comment above
 * ota_trial_marker_t for what that means (CONFIRMED, not a stuck gate). */
static inline bool ota_image_trial_capable(const uint8_t *img, uint32_t len)
{
	if (img == NULL) {
		return false;
	}
	const uint32_t limit = (len < OTA_TRIAL_SCAN_LIMIT) ? len : OTA_TRIAL_SCAN_LIMIT;
	if (limit < sizeof(ota_trial_marker_t)) {
		return false;
	}
	const uint8_t magic[16] = OTA_TRIAL_MARKER_MAGIC_BYTES;
	for (uint32_t off = OTA_TRIAL_MARKER_MIN_OFFSET; off + sizeof(ota_trial_marker_t) <= limit;
	     off += 4u) {
		if (memcmp(&img[off], magic, sizeof magic) == 0) {
			if (ota_trial_rd_u16(&img[off + 16u]) != OTA_TRIAL_MARKER_STRUCT_VER) {
				return false; /* unknown layout: don't guess at it */
			}
			return (ota_trial_rd_u16(&img[off + 18u]) & OTA_TRIAL_CAP_CONFIRM) != 0u;
		}
	}
	return false;
}

#endif /* GD32_BRIDGE_OTA_LAYOUT_H */
