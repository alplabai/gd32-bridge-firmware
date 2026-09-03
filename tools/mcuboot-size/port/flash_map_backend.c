/*
 * SPDX-License-Identifier: Apache-2.0
 * Throwaway port glue for the mcuboot-size measurement. NOT production.
 *
 * This is NOT the real flash_area backend. It exists only so that
 * boot/bootutil/src/*.c has something to link against for a size probe --
 * it reports the same base/size numbers as src/ota_layout.h's two OTA
 * slots but never touches real flash. The real backend (over
 * hal/fmc_ota.c, ~200-400 lines: real GD32 FMC unlock/program/erase +
 * dual-bank page-size selection) is NOT included in this measurement --
 * see the workflow summary.
 */
#include <stddef.h>
#include <string.h>

#include "flash_map_backend/flash_map_backend.h"
#include "sysflash/sysflash.h"

/* Mirrors src/ota_layout.h's OTA_SLOT_A / OTA_SLOT_B (236 KB each) so the
 * reported sector count/geometry is representative; the bootloader and
 * metadata regions are irrelevant to this probe. */
#define PROBE_SLOT_SIZE   0x0003B000u
#define PROBE_SECTOR_SIZE 0x00000800u /* OTA_PAGE_SIZE, src/ota_layout.h */

static const struct flash_area probe_areas[2] = {
	{
	    .fa_id        = PRIMARY_ID,
	    .fa_device_id = 0,
	    .fa_off       = 0x0800A000u,
	    .fa_size      = PROBE_SLOT_SIZE,
	},
	{
	    .fa_id        = SECONDARY_ID,
	    .fa_device_id = 0,
	    .fa_off       = 0x08045000u,
	    .fa_size      = PROBE_SLOT_SIZE,
	},
};

int flash_area_open(uint8_t id, const struct flash_area **fapp)
{
	if (id != PRIMARY_ID && id != SECONDARY_ID) {
		return -1;
	}
	*fapp = &probe_areas[id];
	return 0;
}

void flash_area_close(const struct flash_area *fap)
{
	(void)fap;
}

int flash_area_read(const struct flash_area *fap, uint32_t off, void *dst, uint32_t len)
{
	(void)fap;
	(void)off;
	/* Link-time stub only -- this measurement never boots the image. */
	memset(dst, 0xFF, len);
	return 0;
}

int flash_area_write(const struct flash_area *fap, uint32_t off, const void *src, uint32_t len)
{
	(void)fap;
	(void)off;
	(void)src;
	(void)len;
	return 0;
}

int flash_area_erase(const struct flash_area *fap, uint32_t off, uint32_t len)
{
	(void)fap;
	(void)off;
	(void)len;
	return 0;
}

uint32_t flash_area_align(const struct flash_area *fap)
{
	(void)fap;
	return 8u; /* GD32G553 dual-bank write granularity (quad-word). */
}

uint8_t flash_area_erased_val(const struct flash_area *fap)
{
	(void)fap;
	return 0xFFu;
}

int flash_area_get_sectors(int fa_id, uint32_t *count, struct flash_sector *sectors)
{
	uint32_t n = PROBE_SLOT_SIZE / PROBE_SECTOR_SIZE;
	uint32_t i;

	(void)fa_id;
	if (n > *count) {
		n = *count;
	}
	for (i = 0; i < n; i++) {
		sectors[i].fs_off  = i * PROBE_SECTOR_SIZE;
		sectors[i].fs_size = PROBE_SECTOR_SIZE;
	}
	*count = n;
	return 0;
}

int flash_area_id_from_multi_image_slot(int image_index, int slot)
{
	(void)image_index;
	return (slot == 0) ? PRIMARY_ID : SECONDARY_ID;
}

int flash_area_id_from_image_slot(int slot)
{
	return flash_area_id_from_multi_image_slot(0, slot);
}

int flash_area_id_to_multi_image_slot(int image_index, int area_id)
{
	(void)image_index;
	if (area_id == PRIMARY_ID) {
		return 0;
	}
	if (area_id == SECONDARY_ID) {
		return 1;
	}
	return -1;
}

int flash_area_get_sector(const struct flash_area *fap, uint32_t off, struct flash_sector *fs)
{
	(void)fap;
	fs->fs_off  = (off / PROBE_SECTOR_SIZE) * PROBE_SECTOR_SIZE;
	fs->fs_size = PROBE_SECTOR_SIZE;
	return 0;
}
