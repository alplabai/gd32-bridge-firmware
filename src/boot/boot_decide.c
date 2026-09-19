/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stddef.h>

#include "boot_decide.h"
#include "../crc32.h"
#include "../ota_layout.h"

static bool
meta_read(boot_flash_map_fn map, void *context, uint32_t address, ota_meta_record_t *record)
{
	const ota_meta_record_t *flash =
	    (const ota_meta_record_t *)map(address, sizeof(ota_meta_record_t), context);
	if (flash == NULL || flash->magic != OTA_META_MAGIC ||
	    flash->struct_version != OTA_META_STRUCT_VER) {
		return false;
	}
	if (ota_crc32(0u, (const uint8_t *)flash, offsetof(ota_meta_record_t, rec_crc32)) !=
	    flash->rec_crc32) {
		return false;
	}
	*record = *flash;
	return true;
}

static int meta_candidates(boot_flash_map_fn        map,
                           void                    *context,
                           ota_meta_record_t        records[2],
                           const ota_meta_record_t *candidates[2])
{
	const bool first_valid  = meta_read(map, context, OTA_META_REC0, &records[0]);
	const bool second_valid = meta_read(map, context, OTA_META_REC1, &records[1]);
	int        count        = 0;
	if (first_valid && second_valid) {
		if (records[0].counter >= records[1].counter) {
			candidates[count++] = &records[0];
			candidates[count++] = &records[1];
		} else {
			candidates[count++] = &records[1];
			candidates[count++] = &records[0];
		}
	} else if (first_valid) {
		candidates[count++] = &records[0];
	} else if (second_valid) {
		candidates[count++] = &records[1];
	}
	return count;
}

static bool active_slot_valid(boot_flash_map_fn        map,
                              void                    *context,
                              const ota_meta_record_t *record,
                              uint32_t                *base_out)
{
	const uint8_t slot = record->active_slot;
	uint32_t      base;
	if (!ota_slot_base_checked(slot, &base)) {
		return false;
	}
	const uint32_t len = record->img_len[slot];
	if ((record->slot_valid & (uint8_t)(1u << slot)) == 0u || len == 0u || len > OTA_SLOT_SIZE) {
		return false;
	}
	const uint8_t *image = map(base, len, context);
	if (image == NULL || ota_crc32(0u, image, len) != record->img_crc32[slot] ||
	    !ota_image_bootable(base, image, len)) {
		return false;
	}
	*base_out = base;
	return true;
}

bool boot_select_slot(boot_flash_map_fn map, void *context, uint32_t *slot_base_out)
{
	if (map == NULL || slot_base_out == NULL) {
		return false;
	}
	ota_meta_record_t        records[2];
	const ota_meta_record_t *candidates[2];
	const int                count = meta_candidates(map, context, records, candidates);
	for (int i = 0; i < count; ++i) {
		if (active_slot_valid(map, context, candidates[i], slot_base_out)) {
			return true;
		}
	}
	return false;
}
