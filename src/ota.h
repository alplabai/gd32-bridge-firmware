/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge OTA — Path A state machine (application bootloader over the
 * bridge).  Wired in from src/bootloader/bootloader.c (bl_dispatch_ota).
 *
 * Safe-by-default: unless built with -DBRIDGE_OTA_PARTITIONED (paired with
 * the partitioned bootloader layout + slot-linked app), ota_dispatch()
 * returns STATUS_NOSUPPORT and performs no flash writes.
 */
#ifndef GD32_BRIDGE_OTA_H
#define GD32_BRIDGE_OTA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "protocol.h"

/* OTA opcode handler (cmd in 0xF0..0xFF). */
gd32_bridge_status_t ota_dispatch(uint8_t        cmd,
                                  const uint8_t *req_payload,
                                  size_t         req_payload_len,
                                  uint8_t       *reply_payload,
                                  size_t         reply_payload_cap,
                                  size_t        *reply_payload_len);

/* Background OTA-erase pump (#770): call from the main loop
 * (bridge_hw_tick).  BEGIN arms a slot erase and acks immediately instead
 * of erasing inline (which stalled the SPI reply ~1 s); this advances the
 * erase one page-region per call and flips OTA to READY when done.  No-op
 * in the OTA-inert (non-partitioned) build. */
void ota_erase_tick(void);

/* Trial/confirm + watchdog fallback (bench fact 2026-09-26, E1M-V2M103).
 * Call once from main(), before the transports come up: reconciles the
 * A/B metadata against which slot is actually executing.  If the
 * metadata still marks the OTHER slot TRIAL+active, we are the
 * bootloader's safe fallback after a watchdog-rejected trial -- self-heal
 * the record (no reset).  If it marks US TRIAL+active, this is a fresh
 * trial boot: gate the wire until confirmed. */
void ota_boot_init(void);

/* True from a fresh trial boot until the confirm step below lands.
 * ISR-safe (a single flag read); protocol_dispatch() consults this
 * before touching any opcode. */
bool ota_trial_unconfirmed(void);

/* Note that a wire frame arrived (a single store) -- the confirm signal
 * the trial is waiting for.  Safe to call unconditionally; a no-op
 * outside a trial boot. */
void ota_note_frame(void);

/* Confirm step: call from the base-level tick (next to ota_erase_tick /
 * bridge_hw_tick).  If a frame was noted during an unconfirmed trial,
 * commits the slot as permanent (flags=0) and reboots into it; on a
 * failed commit, leaves TRIAL set and does NOT reset -- the armed
 * watchdog is the safety net. */
void ota_confirm_tick(void);

#endif /* GD32_BRIDGE_OTA_H */
