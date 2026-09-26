/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge OTA bootloader (Path A). Runs from the first 32 KB of flash
 * (0x08000000, never erased by OTA). Picks the active slot from the A/B
 * metadata, validates its image (CRC over the metadata-recorded length),
 * and jumps to it (MSP + VTOR + reset vector). If no slot validates it
 * idles so an external SWD probe can recover the part.
 *
 * Pairs with src/ota.c (the application-side OTA state machine that writes
 * the inactive slot + commits the metadata).
 *
 * Trial/confirm + watchdog fallback (bench fact 2026-09-26, E1M-V2M103): a
 * CRC-valid slot-B image once hung before main() (stock SystemInit spinning
 * on HXTALSTB), and this bootloader had no way to know -- it jumped to the
 * newest valid slot unconditionally, bricking the bridge on both transports
 * until an SWD recovery.  COMMIT/ROLLBACK now mark the freshly-active slot
 * TRIAL; a TRIAL candidate gets the FWDGT armed (~16 s, see
 * OTA_TRIAL_FWDGT_RELOAD below) before the jump, so a hang before the app
 * confirms (src/ota.c ota_confirm_tick()) reverts here to try the previous
 * slot instead.
 *
 * SILICON-VALIDATED 2026-06-04 (bench, protocol v0.6): boot/validate/jump,
 * slot relocation, dual-bank FMC-from-RAM, and the full stream → verify →
 * commit → boot-new-slot → rollback cycle were proven end-to-end over the
 * 25 MHz link, including two GD32 self-reboots through this bootloader, for
 * the A→B update + rollback direction.  A fresh B→A update has NOT been
 * exercised.  See src/bootloader/DESIGN.md for the bench evidence.
 *
 * STILL HANDLE WITH CARE: a bug here bricks the GD32 (no host-driven SWD
 * reflash this HW rev — recover via a bench SWD probe), so change this path
 * incrementally and re-run the bench cycle rather than trusting the gate.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "gd32g5x3.h"

#include "ota_layout.h"
#include "crc32.h"

/* Trial/confirm watchdog calibration knob (bench fact 2026-09-26,
 * E1M-V2M103): a slot-B image passed VERIFY/COMMIT then hung before
 * main() (stock SystemInit spinning on HXTALSTB) with no watchdog to
 * revert it -- the bridge stayed dead on both transports until an SWD
 * recovery.  IRC32K/256 ~= 125 Hz; reload 2000 counts -> ~16 s nominal
 * before a TRIAL candidate that never confirms falls back to the
 * previous slot.  IRC32K's factory trim drifts a few percent across
 * temperature (GD32G5x3 datasheet) -- this is nominal, not exact; retune
 * here if bench soak timing needs a different margin. */
#define OTA_TRIAL_FWDGT_RELOAD 2000u

static bool meta_read(uint32_t addr, ota_meta_record_t *r)
{
	const ota_meta_record_t *p = (const ota_meta_record_t *)addr;
	if (p->magic != OTA_META_MAGIC || p->struct_version != OTA_META_STRUCT_VER) {
		return false;
	}
	if (ota_crc32(0u, (const uint8_t *)p, offsetof(ota_meta_record_t, rec_crc32)) != p->rec_crc32) {
		return false;
	}
	*r = *p;
	return true;
}

/* Order the two CRC-valid metadata records newest-first into cands[]
 * and return how many are present (0..2).  The bootloader then tries
 * each in order (#754): a newer record that fails SEMANTIC validation
 * must not suppress an older bootable one, or the part drops into
 * permanent recovery with a perfectly good slot sitting unused. */
static int
meta_candidates(ota_meta_record_t *a, ota_meta_record_t *b, const ota_meta_record_t *cands[2])
{
	const bool va = meta_read(OTA_META_REC0, a);
	const bool vb = meta_read(OTA_META_REC1, b);
	int        n  = 0;
	if (va && vb) {
		if (a->counter >= b->counter) {
			cands[n++] = a;
			cands[n++] = b;
		} else {
			cands[n++] = b;
			cands[n++] = a;
		}
	} else if (va) {
		cands[n++] = a;
	} else if (vb) {
		cands[n++] = b;
	}
	return n;
}

static bool active_slot_valid(const ota_meta_record_t *m)
{
	const uint8_t slot = m->active_slot;
	uint32_t      base;
	/* A corrupt active_slot must NOT resolve to a valid flash base
	 * (#741): reject it here, before the slot indexes the [2] arrays. */
	if (!ota_slot_base_checked(slot, &base)) {
		return false;
	}
	const uint32_t len = m->img_len[slot];
	if ((m->slot_valid & (uint8_t)(1u << slot)) == 0u) {
		return false;
	}
	if (len == 0u || len > OTA_SLOT_SIZE) {
		return false;
	}
	if (ota_crc32(0u, (const uint8_t *)base, len) != m->img_crc32[slot]) {
		return false;
	}
	/* CRC integrity is necessary but not sufficient: a CRC-valid but
	 * truncated / vector-less image would boot into garbage MSP/reset
	 * words (#755).  Require a bootable vector head before jumping.
	 * Flash is memory-mapped, so the base doubles as the read pointer. */
	return ota_image_bootable(base, (const uint8_t *)base, len);
}

static void jump_to_slot(uint32_t slot_base)
{
	const uint32_t msp   = *(volatile uint32_t *)slot_base;
	const uint32_t reset = *(volatile uint32_t *)(slot_base + 4u);
	__disable_irq();
	SCB->VTOR = slot_base; /* relocate the vector table to the slot */
	__DSB();
	__set_MSP(msp);
	__ISB();
	((void (*)(void))reset)(); /* no return */
}

int main(void)
{
	ota_meta_record_t        a, b;
	const ota_meta_record_t *cands[2];
	const int                n = meta_candidates(&a, &b, cands);
	/* Read the reset cause ONCE, up front (bench fact 2026-09-26,
	 * E1M-V2M103): this bootloader must NOT clear RCU_RSTSCK -- the
	 * fallback app's CMD_RESET_REASON still needs to report WDT for a
	 * real watchdog event (the app clears it lazily, on that read; the
	 * one deliberate RSTSCK clear point is ota_system_reset(), hit
	 * only after a trial CONFIRMS). */
	const bool wdt_fired = (RCU_RSTSCK & RCU_RSTSCK_FWDGTRSTF) != 0u;
	/* Newest-first with fallback (#754): the first record whose active
	 * slot passes full semantic + image validation AND the trial/
	 * watchdog gate (a TRIAL candidate that already burned a watchdog
	 * reset this power cycle hung before confirming once -- don't
	 * re-try it) wins. */
	for (int i = 0; i < n; ++i) {
		if (!active_slot_valid(cands[i]) || !ota_boot_candidate_ok(cands[i], wdt_fired)) {
			continue;
		}
		uint32_t base;
		if (!ota_slot_base_checked(cands[i]->active_slot, &base)) {
			continue; /* unreachable: active_slot_valid already checked this */
		}
		if ((cands[i]->flags & OTA_META_FLAG_TRIAL) != 0u) {
			/* Arm the watchdog before jumping into an unconfirmed
			 * image: if it hangs before ota_confirm_tick() ever runs
			 * (the 2026-09-26 bench incident), FWDGT reverts to the
			 * previous slot instead of bricking the bridge on both
			 * transports.  Keep the watchdog running if a debugger
			 * halts the core, so a breakpointed bench session doesn't
			 * spuriously reset mid-investigation. */
			dbg_periph_enable(DBG_FWDGT_HOLD);
			fwdgt_config(OTA_TRIAL_FWDGT_RELOAD, FWDGT_PSC_DIV256);
		}
		jump_to_slot(base);
	}
	/* No valid image: recovery. A later build exposes the OTA opcodes here
     * to accept a reflash over the bridge; today, idle so a bench SWD probe
     * can take over. */
	for (;;) {
		__WFI();
	}
	/* unreachable */
	return 0;
}
