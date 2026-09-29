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
 * TRIAL; a TRIAL candidate gets the FWDGT armed (see OTA_TRIAL_FWDGT_RELOAD
 * below) before the jump, so a hang before the app confirms (src/ota.c
 * ota_confirm_tick()) reverts here to try the previous slot instead.  A
 * bootloader must never idle while SOME CRC-valid, vector-valid candidate
 * exists: ota_boot_select() (src/ota_layout.h) adds a last-resort pass for
 * exactly that -- see its own comment.
 *
 * Reset-cause ownership: this bootloader reads RCU_RSTSCK exactly ONCE per
 * boot, STASHES the raw value in RTC_BKP8 (backup-domain, survives
 * NVIC_SystemReset -- same rationale as hal/gd32/fault_handlers.c's use of
 * RTC_BKP0..7 for fault records; RTC_BKP8 is the next free register), then
 * clears RSTFC before jumping.  The application's bridge_hw_reset_reason()
 * (hal/gd32/init.c) decodes CMD_RESET_REASON from that stash, not from a
 * live RCU_RSTSCK read -- by the time the app runs, this bootloader has
 * already cleared the live register, so reading it directly there would
 * always see "no cause".  This also means a stale FWDGTRSTF from an
 * EARLIER, unrelated cycle can never survive past THIS boot to be
 * misread as a fresh trial's watchdog fallback on some LATER boot.
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
 * recovery.  Maximum reload (0xFFF, 12-bit counter) at the coarsest
 * prescaler (/256) -- the longest window this peripheral can express --
 * to give a genuinely slow-starting image (clock recovery, cold-flash
 * wait states) every chance to reach ota_confirm_tick() before reverting.
 * IRC32K/256 is NOMINALLY ~125 Hz, so 0xFFF (4095) counts is NOMINALLY
 * ~32.8 s -- stated as nominal, not exact: this file does not assert an
 * IRC32K tolerance number of its own.  The GD32G5x3 datasheet's IRC32K
 * accuracy section is the authority on the real spread and has not been
 * independently checked against this repo for this fix; treat the ~32.8 s
 * figure as an order-of-magnitude planning number until that check
 * happens, and retune the reload here if bench soak timing needs a
 * different margin (a WARM boot bench soak can also just measure the
 * ACTUAL fallback latency directly, sidestepping the datasheet number
 * entirely). */
#define OTA_TRIAL_FWDGT_RELOAD 0xFFFu

/* Backup-domain write access: RCU_APB1EN_PMUEN clocks the PMU so
 * PMU_CTL0 is writable at all; PMU_CTL0_BKPWEN then gates writes to the
 * RTC_BKPx block itself (UM p.145) -- same two-register unlock
 * hal/gd32/fault_handlers.c's fault_backup_unlock() uses for RTC_BKP0..7;
 * duplicated here rather than shared because the two are separate
 * translation units linked into separate images (bootloader vs. app) and
 * this is two idempotent register writes, not logic worth a shared
 * header over. */
static void backup_domain_unlock(void)
{
	RCU_APB1EN |= RCU_APB1EN_PMUEN;
	PMU_CTL0 |= PMU_CTL0_BKPWEN;
}

/* Bring up IRC32K -- the FWDGT's fixed clock source -- BEFORE arming the
 * watchdog (C2, adversarial-verify finding).  Without this, the very
 * first fwdgt_config() call on a cold boot can race the oscillator:
 * gd32g5x3_fwdgt.c's PSC/RLD writes each poll a ready flag (PUD/RUD) that
 * only advances on IRC32K edges, so a PSC/RLD write issued before IRC32K
 * has even started can legitimately time out -- and unlike a graceful
 * "retry later," fwdgt_config() writes FWDGT_CTL's KEY_ENABLE FIRST,
 * unconditionally, starting the counter immediately and IRREVOCABLY (no
 * register disarms a running FWDGT); see the call site below for what a
 * PSC/RLD timeout leaves armed in that case.  Same bring-up
 * hal/gd32/power.c's rtc_wakeup_init_once() uses for the identical
 * oscillator; returns false (best-effort) if IRC32K never stabilises, in
 * which case fwdgt_config() below still runs and its own bounded
 * software timeout is the final word regardless. */
static bool fwdgt_clock_ready(void)
{
	rcu_osci_on(RCU_IRC32K);
	uint32_t to = 200000u;
	while (--to && RESET == rcu_flag_get(RCU_FLAG_IRC32KSTB)) {
		/* spin */
	}
	return to != 0u;
}

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
	/* Read the reset cause ONCE, up front, then hand ownership of
	 * RCU_RSTSCK's cause bits to the stash: after this point they are
	 * cleared, so nothing later in this boot (or a future one) can
	 * mistake a stale bit for a fresh event.  See the file header's
	 * "Reset-cause ownership" section. */
	const uint32_t rstsck    = RCU_RSTSCK;
	const bool     wdt_fired = (rstsck & RCU_RSTSCK_FWDGTRSTF) != 0u;
	backup_domain_unlock();
	RTC_BKP8 = rstsck;
	RCU_RSTSCK |= RCU_RSTSCK_RSTFC;

	ota_meta_record_t        a, b;
	const ota_meta_record_t *cands[2];
	const int                n        = meta_candidates(&a, &b, cands);
	bool                     valid[2] = { false, false };
	for (int i = 0; i < n; ++i) {
		valid[i] = active_slot_valid(cands[i]);
	}
	bool      last_resort = false;
	const int sel         = ota_boot_select(cands, valid, n, wdt_fired, &last_resort);
	if (sel >= 0) {
		uint32_t base;
		if (ota_slot_base_checked(cands[sel]->active_slot, &base)) {
			if ((cands[sel]->flags & OTA_META_FLAG_TRIAL) != 0u) {
				/* Freeze the FWDGT counter while a debugger holds the
				 * core halted (DBG_FWDGT_HOLD) -- without this, a
				 * breakpointed bench session gets blown away by a
				 * spurious watchdog reset mid-investigation; the
				 * counter resumes counting from where it was once
				 * execution continues, so this does not extend the
				 * real (running) window. */
				dbg_periph_enable(DBG_FWDGT_HOLD);
				(void)fwdgt_clock_ready(); /* best-effort; see its own comment */
				if (fwdgt_config(OTA_TRIAL_FWDGT_RELOAD, FWDGT_PSC_DIV256) != SUCCESS) {
					/* CORRECTED (C2): fwdgt_config() already
					 * issued FWDGT_CTL's KEY_ENABLE write before
					 * this PSC/RLD failure -- the counter is
					 * running NOW and cannot be un-armed by any
					 * register in this peripheral.  A PSC/RLD
					 * timeout does NOT mean "some window applies,
					 * just maybe not the intended one": it means
					 * the counter is running under FWDGT's
					 * POWER-ON-RESET DEFAULTS (PSC=/4, RLD=0xFFF),
					 * which is roughly 0.5 s at nominal IRC32K --
					 * dangerously short compared to the intended
					 * ~32.8 s.  Retry exactly once (the
					 * fwdgt_clock_ready() call above already
					 * removes the likely root cause -- see its own
					 * comment); this is the bootloader arming its
					 * OWN safety net, so a retry LOOP here would
					 * defeat the point if the watchdog itself is
					 * the thing stuck.  If it fails AGAIN, that
					 * ~0.5 s default window is what is left armed
					 * and this bootloader has no further recourse
					 * short of re-implementing the raw PSC/RLD
					 * register writes with a longer PUD/RUD wait
					 * than gd32g5x3_fwdgt.c's own budget -- not
					 * done here. */
					(void)fwdgt_config(OTA_TRIAL_FWDGT_RELOAD, FWDGT_PSC_DIV256);
				}
				/* last_resort candidates are always TRIAL (see
				 * ota_boot_select()'s own comment), so arming here
				 * unconditionally covers that case too -- no
				 * separate branch needed. */
			}
			jump_to_slot(base);
		}
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
