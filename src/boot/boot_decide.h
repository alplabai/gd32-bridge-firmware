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

/* Select the first bootable slot named by the CRC-valid metadata records,
 * ordered newest-first with REC0 winning equal counters.  Returns false and
 * writes 0 to slot_base_out when neither record names a bootable image. */
bool boot_decide_slot(boot_flash_map_fn_t map, void *context, uint32_t *slot_base_out);

#endif /* GD32_BRIDGE_BOOT_DECIDE_H */
