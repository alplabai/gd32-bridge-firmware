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
 * build-id reads + the A/B metadata generation history, for the A->B
 * update + rollback direction.  A fresh B->A update has NOT been exercised.
 *
 * Trial/confirm + watchdog fallback (bench fact 2026-09-26, E1M-V2M103):
 * COMMIT/ROLLBACK mark the freshly-active slot OTA_META_FLAG_TRIAL; the
 * bootloader (src/boot/boot_main.c) arms the FWDGT before jumping to a
 * TRIAL candidate.  ota_boot_init() (called once from main(), before the
 * transports come up) reconciles the metadata against which slot is
 * actually running; protocol_dispatch() gates every opcode BUSY via
 * ota_trial_unconfirmed() until the first noted frame lets
 * ota_confirm_tick() (run from the base-level tick) clear the flag and
 * reboot into the now-permanent image.  See src/bootloader/DESIGN.md.
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
 * 236 KB slot inline: that can be a 4.72 s RAMFUNC loop in dual-bank mode
 * with the SPI slave unserviced, so the BEGIN reply is lost and the host's
 * ota_begin() hangs.  Instead BEGIN arms the erase (state=BUSY) and acks
 * immediately; the main loop's ota_erase_tick() erases ONE OTA_PAGE_SIZE
 * region per tick.  A tick is up to 20 ms single-bank or 40 ms dual-bank:
 * tERASE is 20 ms maximum per page (Datasheet Rev2.0 p.126), and a 2 KB
 * region spans two 1 KB pages in dual-bank mode.  The host polls GET_STATE
 * for READY before streaming. */
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
 * written).
 *
 * `target_override`: 0 uses the rank rule above (the COMMIT/ROLLBACK/
 * self-heal shape: establish a NEW active slot while preserving whichever
 * page best protects the part that is actually running).  A nonzero
 * value (OTA_META_REC0/REC1) instead forces erase+program onto that
 * EXACT page, bypassing the rank rule entirely -- the one caller that
 * needs this is ota_confirm_tick() (bench fact 2026-09-26): confirming a
 * TRIAL record is "flip THIS record's own flags to CONFIRMED", not
 * "establish a new active slot", so it must overwrite the SAME page the
 * TRIAL record already occupies (the page meta_current() names via
 * `which` at the point of confirming).  Using the rank rule there instead
 * would target the OTHER page -- which, in the exact scenario this fix
 * exists for, is the OLDER, already-CONFIRMED fallback record -- and a
 * failed or power-cut-interrupted confirm would then destroy that
 * fallback, leaving ONLY the (still-TRIAL) record behind instead of
 * degrading to "reverted" the way an interrupted write should.
 *
 * `invalidate_slot`: OTA_SLOT_A or OTA_SLOT_B clears that slot's
 * slot_valid bit in the written record (in addition to setting
 * `active_slot`'s bit); 0xFFu means "touch no additional bit".  Used by
 * ota_boot_init()'s self-heal path (bench fact 2026-09-26) to invalidate
 * a hung/rejected TRIAL slot so a later ROLLBACK can never re-select it
 * (h_rollback's own guard already refuses an invalid `other` slot -- this
 * is what makes that guard fire for a slot the bootloader just steered
 * away from). */
static bool meta_commit(uint8_t  active_slot,
                        bool     update_entry,
                        uint32_t fw_ver,
                        uint32_t img_len,
                        uint32_t img_crc,
                        uint8_t  flags,
                        uint32_t target_override,
                        uint8_t  invalidate_slot)
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

	uint32_t target;
	if (target_override != 0u) {
		target = target_override;
	} else {
		const uint8_t rank_a = meta_page_rank(va, &a);
		const uint8_t rank_b = meta_page_rank(vb, &b);
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
	}

	if (!ota_fmc_erase_range(target, OTA_PAGE_SIZE)) {
		return false;
	}
	rec.magic          = OTA_META_MAGIC;
	rec.struct_version = OTA_META_STRUCT_VER;
	rec.counter += 1u;
	rec.active_slot = active_slot;
	rec.slot_valid |= (uint8_t)(1u << active_slot);
	if (invalidate_slot == OTA_SLOT_A || invalidate_slot == OTA_SLOT_B) {
		rec.slot_valid &= (uint8_t)~(1u << invalidate_slot);
	}
	/* ALWAYS set explicitly -- never carry the surviving record's flags
	 * forward.  It describes a different boot's trial state (and on a
	 * record last written before this field existed, the old _pad bytes
	 * could be anything). */
	rec.flags = flags;
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

/* ---- trial/confirm + watchdog fallback (bench fact 2026-09-26,
 * E1M-V2M103) -------------------------------------------------------- */

/* Minimum firmware version (packed major<<16|minor<<8|patch, matching
 * OTA_BEGIN's v0.7 wire form) that KNOWS about this trial/confirm dance --
 * 0.2.14, the firmware-version.txt release this fix first shipped in.  An
 * image older than that, or one whose version is UNKNOWN (a legacy 8-byte
 * BEGIN packs 0), never calls ota_boot_init()/ota_confirm_tick(): nothing
 * in it will ever clear a TRIAL flag, so marking it TRIAL would gate the
 * wire BUSY forever instead of merely losing the watchdog protection.
 * Committing/rolling back to such an image CONFIRMED (flags=0) is the
 * conservative choice -- no different from before this fix, and no
 * permanent brick either. */
#define OTA_TRIAL_MIN_FW_VERSION 0x00020Eu /* 0.2.14 */

static bool fw_version_trial_capable(uint32_t packed_fw_version)
{
	return packed_fw_version != 0u && packed_fw_version >= OTA_TRIAL_MIN_FW_VERSION;
}

/* volatile: ota_confirm_tick() runs from the base-level tick and
 * ota_note_frame()/ota_trial_unconfirmed() run from (or are consulted by)
 * the transport ISR-driven protocol_dispatch() path -- these three are the
 * cross-context handshake between them, not just an optimiser hazard on
 * this single-core, no-RTOS target. */
static volatile bool s_trial;      /* this boot is running a not-yet-confirmed TRIAL image */
static volatile bool s_confirmed;  /* the confirm commit already landed this boot */
static volatile bool s_frame_seen; /* a wire frame arrived since boot (the confirm signal) */

void ota_boot_init(void)
{
	/* Real hardware zero-inits these on every cold/warm boot; reset
	 * explicitly too so host-side test re-entry into the same process
	 * behaves identically to a fresh boot. */
	s_trial      = false;
	s_confirmed  = false;
	s_frame_seen = false;
	if (!ota_fmc_supported()) {
		return;
	}
	ota_meta_record_t a, b;
	const bool        va = meta_read(OTA_META_REC0, &a);
	const bool        vb = meta_read(OTA_META_REC1, &b);
	if (!va && !vb) {
		return; /* factory / corrupt: nothing to reconcile */
	}

	/* The record that actually NAMES the slot we are running -- NOT
	 * simply meta_current()'s overall-newest pick (#754's fallback, and
	 * this fix's last-resort pass in ota_boot_select(), can both boot a
	 * candidate that is NOT the highest-counter record on flash; if an
	 * even-newer-but-rejected record exists on the other page,
	 * meta_current() would describe a slot nothing is executing). */
	const ota_meta_record_t *run_rec = NULL;
	if (va && a.active_slot == OTA_RUNNING_SLOT) {
		run_rec = &a;
	}
	if (vb && b.active_slot == OTA_RUNNING_SLOT &&
	    (run_rec == NULL || b.counter > run_rec->counter)) {
		run_rec = &b;
	}
	if (run_rec != NULL && (run_rec->flags & OTA_META_FLAG_TRIAL) != 0u) {
		s_trial = true; /* fresh (or re-armed last-resort) trial boot */
	}

	/* Separately: does the OVERALL newest record still name a DIFFERENT
	 * slot, and is it still marked TRIAL?  That is a lingering divergence
	 * -- that candidate was rejected (by validity or the watchdog gate)
	 * in favour of what we are actually running.  Self-heal it so a
	 * later boot never sees a stale trial pointing at a slot nothing will
	 * jump to again, AND clear that slot's slot_valid bit so a later
	 * ROLLBACK can never re-select the exact slot the bootloader just
	 * steered away from.  We ARE the safe (or last-resort, already-
	 * gated-by-s_trial-above) fallback already, so no reset. */
	ota_meta_record_t newest;
	uint32_t          which;
	if (meta_pick_newest(&a, va, &b, vb, &newest, &which) &&
	    newest.active_slot != OTA_RUNNING_SLOT && (newest.flags & OTA_META_FLAG_TRIAL) != 0u) {
		(void)meta_commit(OTA_RUNNING_SLOT, false, 0u, 0u, 0u, 0u, 0u, newest.active_slot);
	}
}

bool ota_trial_unconfirmed(void)
{
	return s_trial && !s_confirmed;
}

void ota_note_frame(void)
{
	s_frame_seen = true;
}

void ota_confirm_tick(void)
{
	if (!s_trial || s_confirmed || !s_frame_seen) {
		return;
	}
	/* Force the erase target onto the SAME page the current TRIAL record
	 * occupies (target_override=`which`) -- see meta_commit()'s own
	 * comment for why the #74 rank rule must NOT choose here. */
	ota_meta_record_t cur;
	uint32_t          which;
	if (!meta_current(&cur, &which)) {
		return; /* unreachable: ota_boot_init() already required a valid
		         * TRIAL record naming OTA_RUNNING_SLOT to set s_trial */
	}
	if (!meta_commit(OTA_RUNNING_SLOT, false, 0u, 0u, 0u, 0u, which, 0xFFu)) {
		return; /* leave TRIAL set -- the armed watchdog is the safety net */
	}
	/* Do NOT set s_confirmed here: the gate must stay closed for the
	 * remainder of THIS boot (a real ota_system_reset() never returns; a
	 * host-test seam that does could otherwise observe a one-instruction
	 * window where the commit landed but the reset hasn't -- s_confirmed
	 * only needs to exist for a NEXT boot's ota_boot_init() to read back
	 * flags=0 from flash, not for this one). */
	ota_system_reset(); /* reboot into the now-permanent (non-TRIAL) image */
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
	/* Downgrade guard (bench fact 2026-09-26): only mark TRIAL when the
	 * committed image itself KNOWS how to confirm -- see
	 * fw_version_trial_capable()'s own comment.  An unknown/pre-trial
	 * image commits CONFIRMED instead, same as before this fix. */
	const uint8_t commit_flags = fw_version_trial_capable(s_fw_version) ? OTA_META_FLAG_TRIAL : 0u;
	if (!meta_commit(s_inactive,
	                 true,
	                 s_fw_version /* 0 = legacy BEGIN, unknown */,
	                 s_img_len,
	                 s_img_crc,
	                 commit_flags,
	                 0u,
	                 0xFFu)) {
		s_state = OTA_ST_ERROR;
		s_err   = 6u;
		return STATUS_IO;
	}
	/* On silicon the reset happens before protocol_dispatch() returns to
	 * the transport, so STATUS_OK below is never staged on the wire.  The
	 * host must treat the missing/all-0x00 reply as "rebooting", then re-init
	 * the link and probe OTA_GET_STATE or CMD_GET_BUILD_ID.  The return is
	 * retained for the host-test reset seam, where ota_system_reset() returns. */
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
     * against the len/CRC recorded when that slot was last committed.
     * Downgrade guard (bench fact 2026-09-26): TRIAL only if the target
     * slot's OWN recorded fw_version knows how to confirm -- see
     * fw_version_trial_capable(). */
	const uint8_t rollback_flags =
	    fw_version_trial_capable(cur.fw_version[other]) ? OTA_META_FLAG_TRIAL : 0u;
	if (!meta_commit(other, false, 0u, 0u, 0u, rollback_flags, 0u, 0xFFu)) {
		return STATUS_IO;
	}
	/* Same reset-before-reply contract as h_commit(): STATUS_OK is not
	 * staged on silicon.  The host must treat a missing/all-0x00 reply as
	 * "rebooting", then re-init and confirm with OTA_GET_STATE or CMD_GET_BUILD_ID. */
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
 * from the main loop (bridge_hw_tick).  Each call can black out execution
 * for up to 20 ms single-bank or 40 ms dual-bank (Datasheet Rev2.0 p.126,
 * tERASE maximum 20 ms per page; dual-bank mode uses two 1 KB pages per
 * 2 KB region).  That is bounded per call, unlike the old inline whole-slot
 * erase that could stall BEGIN's reply for 4.72 s.  Flips the OTA state
 * machine to READY once the slot is fully erased, or ERROR on a failed page.
 * No-op unless an erase is armed. */
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

/* OTA inert: no trial ever gets armed, so nothing to reconcile/gate/confirm. */
void ota_boot_init(void)
{
}

bool ota_trial_unconfirmed(void)
{
	return false;
}

void ota_note_frame(void)
{
}

void ota_confirm_tick(void)
{
}

#endif /* BRIDGE_OTA_PARTITIONED */
