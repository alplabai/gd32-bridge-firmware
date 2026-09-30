/* SPDX-License-Identifier: Apache-2.0
 *
 * Host execution of the real Path-A bootloader decision logic (#153).
 * Absolute flash addresses are mapped to typed metadata records and small
 * image buffers, so the tests cover the same address selection, CRC checks,
 * vector validation and newest-first fallback used on the Cortex-M33.
 */

#include <zephyr/ztest.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "boot_decide.h"
#include "crc32.h"
#include "ota_layout.h"

ZTEST_SUITE(gd32_bridge_boot_decide, NULL, NULL, NULL, NULL, NULL);

#define TEST_IMAGE_CAP 64u
#define TEST_IMAGE_LEN 32u

typedef struct {
	ota_meta_record_t metadata[2];
	uint8_t           image[2][TEST_IMAGE_CAP];
	bool              metadata_available[2];
	bool              image_available[2];
	bool              overread;   /* bytes past the header read of a bad-magic record */
	unsigned          feeds;      /* watchdog feed hook calls */
	unsigned          image_maps; /* full-slot image maps (each precedes one CRC) */
	bool              feed_late;  /* an image was mapped without a fresh feed first */
} flash_model_t;

static flash_model_t model;

static void put_u32(uint8_t *dst, uint32_t value)
{
	dst[0] = (uint8_t)value;
	dst[1] = (uint8_t)(value >> 8);
	dst[2] = (uint8_t)(value >> 16);
	dst[3] = (uint8_t)(value >> 24);
}

static uint32_t slot_base(uint8_t slot)
{
	return slot == OTA_SLOT_A ? OTA_SLOT_A_BASE : OTA_SLOT_B_BASE;
}

static void prepare_image(uint8_t slot, uint32_t length, uint8_t fill)
{
	zassert_true(slot <= OTA_SLOT_B);
	zassert_true(length <= TEST_IMAGE_CAP);
	memset(model.image[slot], fill, TEST_IMAGE_CAP);
	if (length >= OTA_IMG_MIN_LEN) {
		put_u32(&model.image[slot][0], OTA_SRAM_BASE + 0x1000u);
		put_u32(&model.image[slot][4], (slot_base(slot) + OTA_IMG_MIN_LEN) | 1u);
	}
	model.image_available[slot] = true;
}

static void reset_model(void)
{
	memset(&model, 0, sizeof(model));
	prepare_image(OTA_SLOT_A, TEST_IMAGE_LEN, 0xA5u);
	prepare_image(OTA_SLOT_B, TEST_IMAGE_LEN, 0x5Au);
}

static void finalize_metadata(unsigned index)
{
	ota_meta_record_t *record = &model.metadata[index];
	record->rec_crc32 =
	    ota_crc32(0u, (const uint8_t *)record, offsetof(ota_meta_record_t, rec_crc32));
}

static void write_metadata(unsigned index, uint32_t counter, uint8_t slot, uint32_t length)
{
	zassert_true(index < 2u);
	ota_meta_record_t *record = &model.metadata[index];
	memset(record, 0, sizeof(*record));
	record->magic          = OTA_META_MAGIC;
	record->struct_version = OTA_META_STRUCT_VER;
	record->counter        = counter;
	record->active_slot    = slot;
	if (slot <= OTA_SLOT_B) {
		record->slot_valid    = (uint8_t)(1u << slot);
		record->img_len[slot] = length;
		record->img_crc32[slot] =
		    length <= TEST_IMAGE_CAP ? ota_crc32(0u, model.image[slot], length) : 0xBAD0BAD0u;
	}
	finalize_metadata(index);
	model.metadata_available[index] = true;
}

static const uint8_t *map_flash(uint32_t address, size_t length, void *context)
{
	flash_model_t *flash = context;
	for (unsigned i = 0; i < 2u; i++) {
		if (address == (i == 0u ? OTA_META_REC0 : OTA_META_REC1) &&
		    length <= sizeof(ota_meta_record_t) && flash->metadata_available[i]) {
			if (length > offsetof(ota_meta_record_t, counter) &&
			    (flash->metadata[i].magic != OTA_META_MAGIC ||
			     flash->metadata[i].struct_version != OTA_META_STRUCT_VER)) {
				flash->overread = true;
			}
			return (const uint8_t *)&flash->metadata[i];
		}
	}
	if (address == OTA_SLOT_A_BASE || address == OTA_SLOT_B_BASE) {
		if (flash->image_maps >= flash->feeds) {
			flash->feed_late = true;
		}
		flash->image_maps++;
	}
	if (address == OTA_SLOT_A_BASE && length <= TEST_IMAGE_CAP && flash->image_available[0]) {
		return flash->image[0];
	}
	if (address == OTA_SLOT_B_BASE && length <= TEST_IMAGE_CAP && flash->image_available[1]) {
		return flash->image[1];
	}
	return NULL;
}

static void feed_hook(void *context)
{
	((flash_model_t *)context)->feeds++;
}

static boot_decision_t decide(bool wdt_fired, bool expect_boot)
{
	boot_decision_t decision;
	memset(&decision, 0xEE, sizeof(decision));
	zassert_equal(boot_decide_slot(map_flash, feed_hook, &model, wdt_fired, &decision),
	              expect_boot);
	zassert_false(model.feed_late, "an image CRC ran without a preceding watchdog feed");
	return decision;
}

static void expect_slot(uint32_t expected)
{
	const boot_decision_t d = decide(false, true);
	zassert_equal(d.slot_base, expected);
	zassert_false(d.trial);
	zassert_false(d.last_resort);
}

static void expect_recovery(void)
{
	const boot_decision_t d = decide(false, false);
	zassert_equal(d.slot_base, 0u, "a failed decision must not leak a stale slot base");
	zassert_false(d.trial);
	zassert_false(d.last_resort);
}

static void set_trial(unsigned index)
{
	model.metadata[index].flags |= OTA_META_FLAG_TRIAL;
	finalize_metadata(index);
}

/* Both directions and the equal-counter tie matter: REC0 wins an exact tie,
 * matching the bootloader's established deterministic rule. */
ZTEST(gd32_bridge_boot_decide, test_two_valid_records_choose_newest_and_rec0_wins_tie)
{
	reset_model();
	write_metadata(0u, 2u, OTA_SLOT_A, TEST_IMAGE_LEN);
	write_metadata(1u, 1u, OTA_SLOT_B, TEST_IMAGE_LEN);
	expect_slot(OTA_SLOT_A_BASE);

	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	write_metadata(1u, 2u, OTA_SLOT_B, TEST_IMAGE_LEN);
	expect_slot(OTA_SLOT_B_BASE);

	reset_model();
	write_metadata(0u, 7u, OTA_SLOT_B, TEST_IMAGE_LEN);
	write_metadata(1u, 7u, OTA_SLOT_A, TEST_IMAGE_LEN);
	expect_slot(OTA_SLOT_B_BASE);
}

ZTEST(gd32_bridge_boot_decide, test_single_valid_record_and_no_record_recovery)
{
	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	expect_slot(OTA_SLOT_A_BASE);

	reset_model();
	write_metadata(1u, 1u, OTA_SLOT_B, TEST_IMAGE_LEN);
	expect_slot(OTA_SLOT_B_BASE);

	reset_model();
	expect_recovery();
}

/* Header fields are checked before the record CRC, while any torn write to an
 * otherwise valid record is rejected by that CRC. */
ZTEST(gd32_bridge_boot_decide, test_metadata_magic_version_and_crc_are_required)
{
	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	model.metadata[0].magic ^= 1u;
	finalize_metadata(0u);
	expect_recovery();

	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	model.metadata[0].struct_version++;
	finalize_metadata(0u);
	expect_recovery();

	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	model.metadata[0].counter++; /* torn update: stored CRC is now stale */
	expect_recovery();
}

/* A half-erased record must never have bytes past magic + struct_version read
 * (uncorrectable ECC there raises the flash NMI, gh#36). */
ZTEST(gd32_bridge_boot_decide, test_bad_header_is_rejected_before_reading_the_rest)
{
	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	model.metadata[0].magic ^= 1u;
	write_metadata(1u, 1u, OTA_SLOT_B, TEST_IMAGE_LEN);
	model.metadata[1].struct_version++;
	expect_recovery();
	zassert_false(model.overread);
}

ZTEST(gd32_bridge_boot_decide, test_slot_number_valid_bit_and_length_are_required)
{
	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	model.metadata[0].active_slot = 2u;
	finalize_metadata(0u);
	expect_recovery();

	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	model.metadata[0].slot_valid = 0u;
	finalize_metadata(0u);
	expect_recovery();

	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, 0u);
	expect_recovery();

	reset_model();
	prepare_image(OTA_SLOT_A, OTA_IMG_MIN_LEN - 1u, 0x11u);
	write_metadata(0u, 1u, OTA_SLOT_A, OTA_IMG_MIN_LEN - 1u);
	expect_recovery();

	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, OTA_SLOT_SIZE + 1u);
	expect_recovery();
}

ZTEST(gd32_bridge_boot_decide, test_image_crc_mapping_and_vector_head_are_required)
{
	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	model.image[OTA_SLOT_A][12] ^= 1u;
	expect_recovery();

	reset_model();
	put_u32(&model.image[OTA_SLOT_A][0], 0u); /* MSP outside SRAM */
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	expect_recovery();

	reset_model();
	put_u32(&model.image[OTA_SLOT_A][4], OTA_SLOT_A_BASE + OTA_IMG_MIN_LEN); /* no Thumb */
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	expect_recovery();

	reset_model();
	put_u32(&model.image[OTA_SLOT_A][4], (OTA_SLOT_A_BASE + TEST_IMAGE_LEN) | 1u);
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	expect_recovery();

	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	model.image_available[OTA_SLOT_A] = false;
	expect_recovery();
}

/* This is the #754 divergent state in both directions: the newest CRC-valid
 * record names an unbootable image, while the older record still describes a
 * good slot.  Selection must continue rather than entering recovery. */
ZTEST(gd32_bridge_boot_decide, test_unbootable_newest_record_falls_back_in_both_directions)
{
	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	write_metadata(1u, 2u, OTA_SLOT_B, TEST_IMAGE_LEN);
	model.image[OTA_SLOT_B][12] ^= 1u;
	expect_slot(OTA_SLOT_A_BASE);

	reset_model();
	write_metadata(0u, 3u, OTA_SLOT_A, TEST_IMAGE_LEN);
	write_metadata(1u, 2u, OTA_SLOT_B, TEST_IMAGE_LEN);
	model.image[OTA_SLOT_A][12] ^= 1u;
	expect_slot(OTA_SLOT_B_BASE);

	/* A CRC-valid record can still be semantically unusable. The older
	 * record must remain a fallback when the newest slot is not committed. */
	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	write_metadata(1u, 2u, OTA_SLOT_B, TEST_IMAGE_LEN);
	model.metadata[1].slot_valid = 0u;
	finalize_metadata(1u);
	expect_slot(OTA_SLOT_A_BASE);
}

/* A TRIAL record is reported so the bootloader arms the FWDGT before the
 * jump; a confirmed record is not. */
ZTEST(gd32_bridge_boot_decide, test_trial_flag_is_reported_for_the_selected_record)
{
	reset_model();
	write_metadata(0u, 2u, OTA_SLOT_A, TEST_IMAGE_LEN);
	write_metadata(1u, 1u, OTA_SLOT_B, TEST_IMAGE_LEN);
	set_trial(0u);
	boot_decision_t d = decide(false, true);
	zassert_equal(d.slot_base, OTA_SLOT_A_BASE);
	zassert_true(d.trial);
	zassert_false(d.last_resort);

	set_trial(1u);
	model.metadata[0].flags = 0u;
	finalize_metadata(0u);
	d = decide(false, true);
	zassert_equal(d.slot_base, OTA_SLOT_A_BASE);
	zassert_false(d.trial, "trial state comes from the selected record only");
}

/* A TRIAL record that already burned a watchdog reset this power cycle is
 * skipped in favour of the older confirmed record; a confirmed newest record
 * is never rejected on that basis. */
ZTEST(gd32_bridge_boot_decide, test_watchdog_reset_skips_trial_record_for_confirmed_fallback)
{
	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	write_metadata(1u, 2u, OTA_SLOT_B, TEST_IMAGE_LEN);
	set_trial(1u);
	boot_decision_t d = decide(true, true);
	zassert_equal(d.slot_base, OTA_SLOT_A_BASE);
	zassert_false(d.trial);
	zassert_false(d.last_resort);

	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	write_metadata(1u, 2u, OTA_SLOT_B, TEST_IMAGE_LEN);
	d = decide(true, true);
	zassert_equal(d.slot_base, OTA_SLOT_B_BASE);
	zassert_false(d.last_resort);
}

/* Never idle while a CRC- and vector-valid image exists: when every valid
 * candidate is a TRIAL that already tripped the watchdog, the newest one is
 * booted as a last resort (the caller re-arms the FWDGT for it). */
ZTEST(gd32_bridge_boot_decide, test_watchdog_reset_last_resort_boots_newest_valid_trial)
{
	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	write_metadata(1u, 2u, OTA_SLOT_B, TEST_IMAGE_LEN);
	set_trial(0u);
	set_trial(1u);
	boot_decision_t d = decide(true, true);
	zassert_equal(d.slot_base, OTA_SLOT_B_BASE);
	zassert_true(d.trial);
	zassert_true(d.last_resort);

	/* Only a bootable image counts: invalid trial candidates stay recovery. */
	model.image[OTA_SLOT_A][12] ^= 1u;
	model.image[OTA_SLOT_B][12] ^= 1u;
	d = decide(true, false);
	zassert_equal(d.slot_base, 0u);
}

ZTEST(gd32_bridge_boot_decide, test_null_api_arguments_fail_closed)
{
	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	boot_decision_t d;
	memset(&d, 0xEE, sizeof(d));
	zassert_false(boot_decide_slot(NULL, feed_hook, &model, false, &d));
	zassert_equal(d.slot_base, 0u);
	zassert_false(d.trial);
	zassert_false(boot_decide_slot(map_flash, feed_hook, &model, false, NULL));
}

ZTEST(gd32_bridge_boot_decide, test_watchdog_is_fed_before_each_candidate_crc)
{
	reset_model();
	write_metadata(0u, 2u, OTA_SLOT_A, TEST_IMAGE_LEN);
	write_metadata(1u, 1u, OTA_SLOT_B, TEST_IMAGE_LEN);
	model.image[0][0] ^= 0xFFu; /* newest image fails its CRC -> older one is CRC'd too */
	expect_slot(OTA_SLOT_B_BASE);
	zassert_equal(model.feeds, 2u, "one feed per CRC'd candidate");
	zassert_equal(model.image_maps, 2u);

	reset_model();
	write_metadata(0u, 1u, OTA_SLOT_A, TEST_IMAGE_LEN);
	expect_slot(OTA_SLOT_A_BASE);
	zassert_equal(model.feeds, 1u);
	zassert_equal(model.image_maps, 1u);

	reset_model();
	expect_recovery();
	zassert_equal(model.feeds, 0u, "no candidates, nothing to feed for");
}
