/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Executes the real boot metadata/image selector against a bounded host flash
 * model.  It does not model the selector and intentionally leaves the
 * Cortex-M interrupt/VTOR/MSP jump to silicon validation.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "boot_decide.h"
#include "crc32.h"
#include "ota_layout.h"

#define FLASH_MODEL_BASE OTA_BOOTLOADER_BASE
#define FLASH_MODEL_SIZE (OTA_FLASH_END - OTA_BOOTLOADER_BASE)
#define IMAGE_LEN        8u

static uint8_t  flash_model[FLASH_MODEL_SIZE];
static uint32_t image_crc[2];

static const uint8_t *flash_map(uint32_t address, size_t length, void *context)
{
	(void)context;
	if (address < FLASH_MODEL_BASE || address > OTA_FLASH_END ||
	    length > (size_t)(OTA_FLASH_END - address)) {
		return NULL;
	}
	return &flash_model[address - FLASH_MODEL_BASE];
}

static uint8_t *flash_mut(uint32_t address)
{
	return &flash_model[address - FLASH_MODEL_BASE];
}

static uint32_t slot_base(uint8_t slot)
{
	uint32_t base = 0u;
	zassert_true(ota_slot_base_checked(slot, &base));
	return base;
}

static void put_u32(uint8_t *p, uint32_t value)
{
	p[0] = (uint8_t)value;
	p[1] = (uint8_t)(value >> 8);
	p[2] = (uint8_t)(value >> 16);
	p[3] = (uint8_t)(value >> 24);
}

static void write_bootable_image(uint8_t slot)
{
	uint8_t *image = flash_mut(slot_base(slot));
	put_u32(&image[0], 0x20010000u);
	put_u32(&image[4], slot_base(slot) | 1u);
	image_crc[slot] = ota_crc32(0u, image, IMAGE_LEN);
}

static void
write_record(uint32_t address, uint32_t counter, uint8_t active_slot, uint8_t valid_mask)
{
	ota_meta_record_t record = {
		.magic          = OTA_META_MAGIC,
		.struct_version = OTA_META_STRUCT_VER,
		.counter        = counter,
		.active_slot    = active_slot,
		.slot_valid     = valid_mask,
		.img_len        = { IMAGE_LEN, IMAGE_LEN },
		.img_crc32      = { image_crc[OTA_SLOT_A], image_crc[OTA_SLOT_B] },
	};
	record.rec_crc32 =
	    ota_crc32(0u, (const uint8_t *)&record, offsetof(ota_meta_record_t, rec_crc32));
	memcpy(flash_mut(address), &record, sizeof(record));
}

static void reseal_record(uint32_t address)
{
	ota_meta_record_t *record = (ota_meta_record_t *)flash_mut(address);
	record->rec_crc32 =
	    ota_crc32(0u, (const uint8_t *)record, offsetof(ota_meta_record_t, rec_crc32));
}

static void reset_model(void)
{
	memset(flash_model, 0, sizeof(flash_model));
	write_bootable_image(OTA_SLOT_A);
	write_bootable_image(OTA_SLOT_B);
}

static void assert_selected(uint32_t expected_base)
{
	uint32_t selected = 0u;
	zassert_true(boot_select_slot(flash_map, NULL, &selected));
	zassert_equal(selected, expected_base);
}

ZTEST(gd32_bridge_boot_decide, test_single_records_select_a_and_b)
{
	reset_model();
	write_record(OTA_META_REC0, 1u, OTA_SLOT_A, 0x03u);
	assert_selected(OTA_SLOT_A_BASE);

	reset_model();
	write_record(OTA_META_REC1, 1u, OTA_SLOT_B, 0x03u);
	assert_selected(OTA_SLOT_B_BASE);
}

ZTEST(gd32_bridge_boot_decide, test_newest_record_wins_in_both_directions_and_rec0_wins_ties)
{
	reset_model();
	write_record(OTA_META_REC0, 1u, OTA_SLOT_A, 0x03u);
	write_record(OTA_META_REC1, 2u, OTA_SLOT_B, 0x03u);
	assert_selected(OTA_SLOT_B_BASE);

	reset_model();
	write_record(OTA_META_REC0, 2u, OTA_SLOT_A, 0x03u);
	write_record(OTA_META_REC1, 1u, OTA_SLOT_B, 0x03u);
	assert_selected(OTA_SLOT_A_BASE);

	reset_model();
	write_record(OTA_META_REC0, 2u, OTA_SLOT_A, 0x03u);
	write_record(OTA_META_REC1, 2u, OTA_SLOT_B, 0x03u);
	assert_selected(OTA_SLOT_A_BASE);
}

ZTEST(gd32_bridge_boot_decide, test_newer_bad_image_falls_back_to_older_bootable_record)
{
	reset_model();
	write_record(OTA_META_REC0, 1u, OTA_SLOT_A, 0x03u);
	write_record(OTA_META_REC1, 2u, OTA_SLOT_B, 0x03u);
	/* Metadata remains CRC-valid, but its claimed B image no longer is. */
	flash_mut(OTA_SLOT_B_BASE)[0] ^= 1u;
	assert_selected(OTA_SLOT_A_BASE);

	reset_model();
	write_record(OTA_META_REC0, 1u, OTA_SLOT_A, 0x03u);
	write_bootable_image(OTA_SLOT_B);
	put_u32(flash_mut(OTA_SLOT_B_BASE), 0x20010002u); /* unaligned MSP */
	image_crc[OTA_SLOT_B] = ota_crc32(0u, flash_mut(OTA_SLOT_B_BASE), IMAGE_LEN);
	write_record(OTA_META_REC1, 2u, OTA_SLOT_B, 0x03u);
	assert_selected(OTA_SLOT_A_BASE);
}

ZTEST(gd32_bridge_boot_decide, test_invalid_active_or_descriptor_falls_back_to_older_record)
{
	reset_model();
	write_record(OTA_META_REC0, 1u, OTA_SLOT_A, 0x03u);
	write_record(OTA_META_REC1, 2u, 2u, 0x03u);
	assert_selected(OTA_SLOT_A_BASE);

	reset_model();
	write_record(OTA_META_REC0, 1u, OTA_SLOT_A, 0x03u);
	write_record(OTA_META_REC1, 2u, OTA_SLOT_B, 0x01u);
	assert_selected(OTA_SLOT_A_BASE);

	reset_model();
	write_record(OTA_META_REC0, 1u, OTA_SLOT_A, 0x03u);
	write_record(OTA_META_REC1, 2u, OTA_SLOT_B, 0x03u);
	((ota_meta_record_t *)flash_mut(OTA_META_REC1))->img_len[OTA_SLOT_B] = 0u;
	reseal_record(OTA_META_REC1);
	assert_selected(OTA_SLOT_A_BASE);

	reset_model();
	write_record(OTA_META_REC0, 1u, OTA_SLOT_A, 0x03u);
	write_record(OTA_META_REC1, 2u, OTA_SLOT_B, 0x03u);
	((ota_meta_record_t *)flash_mut(OTA_META_REC1))->img_len[OTA_SLOT_B] = OTA_SLOT_SIZE + 1u;
	reseal_record(OTA_META_REC1);
	assert_selected(OTA_SLOT_A_BASE);
}

ZTEST(gd32_bridge_boot_decide, test_torn_or_invalid_metadata_is_ignored)
{
	reset_model();
	write_record(OTA_META_REC0, 1u, OTA_SLOT_A, 0x03u);
	write_record(OTA_META_REC1, 2u, OTA_SLOT_B, 0x03u);
	flash_mut(OTA_META_REC1)[8] ^= 1u; /* counter tear: record CRC no longer matches */
	assert_selected(OTA_SLOT_A_BASE);

	reset_model();
	write_record(OTA_META_REC0, 1u, OTA_SLOT_A, 0x03u);
	write_record(OTA_META_REC1, 2u, OTA_SLOT_B, 0x03u);
	flash_mut(OTA_META_REC1)[0] ^= 1u; /* magic invalid */
	assert_selected(OTA_SLOT_A_BASE);

	reset_model();
	write_record(OTA_META_REC0, 1u, OTA_SLOT_A, 0x03u);
	write_record(OTA_META_REC1, 2u, OTA_SLOT_B, 0x03u);
	flash_mut(OTA_META_REC1)[4] ^= 1u; /* structure version invalid */
	assert_selected(OTA_SLOT_A_BASE);
}

ZTEST(gd32_bridge_boot_decide, test_no_usable_record_returns_recovery_without_mutating_output)
{
	reset_model();
	uint32_t selected = 0xDEADBEEFu;
	zassert_false(boot_select_slot(flash_map, NULL, &selected));
	zassert_equal(selected, 0xDEADBEEFu);
}

ZTEST_SUITE(gd32_bridge_boot_decide, NULL, NULL, NULL, NULL, NULL);
