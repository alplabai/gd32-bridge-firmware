/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge OTA bootloader -- metadata/image selection decision.
 *
 * This is deliberately independent of Cortex-M registers and the final
 * MSP/VTOR jump so its exact newest-first/fallback decision can run against
 * a host flash model.  The boot entry point supplies the target's
 * memory-mapped flash resolver; tests supply an in-memory resolver.
 */

#ifndef GD32_BRIDGE_BOOT_DECIDE_H
#define GD32_BRIDGE_BOOT_DECIDE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef const uint8_t *(*boot_flash_map_fn)(uint32_t address, size_t length, void *context);

/* Select a CRC-valid and semantically bootable image from the two metadata
 * records, newest first.  If the newest record is syntactically valid but
 * names an invalid image, the older valid record remains a fallback.  On
 * success `*slot_base_out` is the image's flash base; false means recovery.
 *
 * The resolver must return a readable span of `length` bytes, or NULL if that
 * flash range is unavailable. */
bool boot_select_slot(boot_flash_map_fn map, void *context, uint32_t *slot_base_out);

#endif /* GD32_BRIDGE_BOOT_DECIDE_H */
