/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Boot-slot selection for the Path-A bootloader.  Keep this translation unit
 * free of GigaDevice headers: the decision is pure flash-data validation and
 * must execute under the host unit-test suite as well as on the Cortex-M33.
 */

#include "boot_decide.h"

#include <string.h>

#include "crc32.h"
#include "ota_layout.h"

static bool
meta_read(boot_flash_map_fn_t map, void *context, uint32_t address, ota_meta_record_t *record)
{
	const uint8_t *bytes = map(address, sizeof(*record), context);
	if (bytes == NULL) {
		return false;
	}

	/* Copy before interpreting fields so a host mapper may return an
	 * unaligned byte buffer.  On silicon the source is memory-mapped flash. */
	memcpy(record, bytes, sizeof(*record));
	if (record->magic != OTA_META_MAGIC || record->struct_version != OTA_META_STRUCT_VER) {
		return false;
	}
	return ota_crc32(0u, (const uint8_t *)record, offsetof(ota_meta_record_t, rec_crc32)) ==
	       record->rec_crc32;
}

/* Order the two CRC-valid metadata records newest-first into candidates and
 * return how many are present (0..2).  A newer record that fails semantic or
 * image validation must not suppress an older bootable record (#754). */
static int meta_candidates(boot_flash_map_fn_t      map,
                           void                    *context,
                           ota_meta_record_t       *a,
                           ota_meta_record_t       *b,
                           const ota_meta_record_t *candidates[2])
{
	const bool valid_a = meta_read(map, context, OTA_META_REC0, a);
	const bool valid_b = meta_read(map, context, OTA_META_REC1, b);
	int        count   = 0;

	if (valid_a && valid_b) {
		if (a->counter >= b->counter) {
			candidates[count++] = a;
			candidates[count++] = b;
		} else {
			candidates[count++] = b;
			candidates[count++] = a;
		}
	} else if (valid_a) {
		candidates[count++] = a;
	} else if (valid_b) {
		candidates[count++] = b;
	}
	return count;
}

static bool active_slot_valid(boot_flash_map_fn_t      map,
                              void                    *context,
                              const ota_meta_record_t *metadata,
                              uint32_t                *slot_base_out)
{
	const uint8_t slot = metadata->active_slot;
	uint32_t      base;

	/* Reject a corrupt active_slot before it indexes the per-slot arrays. */
	if (!ota_slot_base_checked(slot, &base)) {
		return false;
	}
	const uint32_t length = metadata->img_len[slot];
	if ((metadata->slot_valid & (uint8_t)(1u << slot)) == 0u) {
		return false;
	}
	if (length == 0u || length > OTA_SLOT_SIZE) {
		return false;
	}

	const uint8_t *image = map(base, (size_t)length, context);
	if (image == NULL) {
		return false;
	}
	if (ota_crc32(0u, image, length) != metadata->img_crc32[slot]) {
		return false;
	}
	if (!ota_image_bootable(base, image, length)) {
		return false;
	}

	*slot_base_out = base;
	return true;
}

bool boot_decide_slot(boot_flash_map_fn_t map, void *context, uint32_t *slot_base_out)
{
	if (slot_base_out == NULL) {
		return false;
	}
	*slot_base_out = 0u;
	if (map == NULL) {
		return false;
	}

	ota_meta_record_t        a, b;
	const ota_meta_record_t *candidates[2];
	const int                count = meta_candidates(map, context, &a, &b, candidates);
	for (int i = 0; i < count; ++i) {
		if (active_slot_valid(map, context, candidates[i], slot_base_out)) {
			return true;
		}
	}
	return false;
}
