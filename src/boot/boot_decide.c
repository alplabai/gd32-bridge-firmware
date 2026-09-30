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
	/* Read only magic + struct_version first: a half-erased record can hold an
	 * uncorrectable-ECC doubleword past the header, and touching it raises the
	 * flash-ECC NMI (gh#36).  The rest is read only once the header matches. */
	const size_t   header_len = offsetof(ota_meta_record_t, counter);
	const uint8_t *bytes      = map(address, header_len, context);
	if (bytes == NULL) {
		return false;
	}
	uint32_t header[2];
	memcpy(header, bytes, sizeof(header));
	if (header[0] != OTA_META_MAGIC || header[1] != OTA_META_STRUCT_VER) {
		return false;
	}

	bytes = map(address, sizeof(*record), context);
	if (bytes == NULL) {
		return false;
	}
	/* Copy before interpreting fields so a host mapper may return an
	 * unaligned byte buffer.  On silicon the source is memory-mapped flash. */
	memcpy(record, bytes, sizeof(*record));
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

static bool
active_slot_valid(boot_flash_map_fn_t map, void *context, const ota_meta_record_t *metadata)
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
	return ota_image_bootable(base, image, length);
}

bool boot_decide_slot(boot_flash_map_fn_t map,
                      boot_feed_fn_t      feed,
                      void               *context,
                      bool                wdt_fired,
                      boot_decision_t    *out)
{
	if (out == NULL) {
		return false;
	}
	memset(out, 0, sizeof(*out));
	if (map == NULL) {
		return false;
	}

	ota_meta_record_t        a, b;
	const ota_meta_record_t *cands[2];
	const int                n        = meta_candidates(map, context, &a, &b, cands);
	bool                     valid[2] = { false, false };
	for (int i = 0; i < n; ++i) {
		if (feed != NULL) {
			feed(context);
		}
		valid[i] = active_slot_valid(map, context, cands[i]);
	}
	bool      last_resort = false;
	const int sel         = ota_boot_select(cands, valid, n, wdt_fired, &last_resort);
	if (sel < 0) {
		return false;
	}
	uint32_t base;
	if (!ota_slot_base_checked(cands[sel]->active_slot, &base)) {
		return false;
	}
	out->slot_base   = base;
	out->trial       = (cands[sel]->flags & OTA_META_FLAG_TRIAL) != 0u;
	out->last_resort = last_resort;
	return true;
}
