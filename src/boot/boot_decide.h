/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Vendor-free boot-slot decision seam.  The production bootloader maps
 * memory-mapped flash directly; host tests map the same absolute addresses
 * to buffers so they execute the real metadata and image validation logic.
 */

#ifndef GD32_BRIDGE_BOOT_DECIDE_H
#define GD32_BRIDGE_BOOT_DECIDE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* Return a readable span for [address, address + length), or NULL when the
 * requested flash range is unavailable. */
typedef const uint8_t *(*boot_flash_map_fn_t)(uint32_t address, size_t length, void *context);

/* Watchdog feed hook.  boot_decide_slot() calls it once before each candidate's
 * full-slot CRC, so a long validation never outlives a still-running FWDGT. */
typedef void (*boot_feed_fn_t)(void *context);

/* Outcome of the boot decision.  `trial` mirrors OTA_META_FLAG_TRIAL on the
 * selected record: the caller arms the FWDGT before jumping when it is set.
 * `last_resort` reports that ota_boot_select()'s second pass fired (a valid
 * candidate rejected only by the trial/watchdog gate). */
typedef struct {
	uint32_t slot_base;
	bool     trial;
	bool     last_resort;
} boot_decision_t;

/* Run the bootloader's candidate selection: CRC-valid metadata records
 * newest-first (REC0 wins equal counters), per-record image validation, then
 * ota_boot_select() with the reset-cause `wdt_fired`.  Returns true and fills
 * *out when a slot should be booted; returns false (and zeroes *out) for the
 * recovery case where nothing is bootable. */
bool boot_decide_slot(boot_flash_map_fn_t map,
                      boot_feed_fn_t      feed,
                      void               *context,
                      bool                wdt_fired,
                      boot_decision_t    *out);

#endif /* GD32_BRIDGE_BOOT_DECIDE_H */
