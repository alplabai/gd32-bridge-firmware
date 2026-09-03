/* SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for hal/fmc_ota_guard.h's ota_fmc_range_forbidden() (#79).
 * This is the single funnel guard hal/fmc_ota.c's ota_fmc_erase_range()
 * and ota_fmc_program() both call before touching real flash; PR #73's
 * own review found that a caller-side guard could be deleted with every
 * existing test staying green, so its EXISTENCE, not just its logic, is
 * what this suite pins.
 *
 * Built with BRIDGE_OTA_PARTITIONED + a fixed BRIDGE_APP_SLOT_BASE (slot
 * A) so both halves of the guard are exercised -- the bootloader check
 * unconditionally, the running-slot check under the same build
 * conditions hal/fmc_ota.c compiles it in.
 */

#include <zephyr/ztest.h>

#include "fmc_ota_guard.h"

ZTEST(gd32_bridge_fmc_ota_guard, test_bootloader_rejected)
{
	/* Exact bootloader range. */
	zassert_true(ota_fmc_range_forbidden(OTA_BOOTLOADER_BASE, OTA_BOOTLOADER_SIZE),
	             "the bootloader's own range must be refused");
	/* A single page inside the bootloader. */
	zassert_true(ota_fmc_range_forbidden(OTA_BOOTLOADER_BASE, OTA_PAGE_SIZE),
	             "a bootloader-leading page must be refused");
	/* A range that only overlaps the bootloader's tail. */
	zassert_true(ota_fmc_range_forbidden(OTA_BOOTLOADER_BASE + OTA_BOOTLOADER_SIZE - OTA_PAGE_SIZE,
	                                     2u * OTA_PAGE_SIZE),
	             "a range straddling the bootloader/metadata boundary must be refused");
}

ZTEST(gd32_bridge_fmc_ota_guard, test_running_slot_rejected)
{
	/* This TU is built with BRIDGE_APP_SLOT_BASE = OTA_SLOT_A_BASE (see
	 * CMakeLists.txt), so slot A is the running slot. */
	zassert_true(ota_fmc_range_forbidden(OTA_SLOT_A_BASE, OTA_PAGE_SIZE),
	             "the running slot's own leading page must be refused");
	zassert_true(
	    ota_fmc_range_forbidden(OTA_SLOT_A_BASE + OTA_SLOT_SIZE - OTA_PAGE_SIZE, OTA_PAGE_SIZE),
	    "the running slot's trailing page must be refused");
}

ZTEST(gd32_bridge_fmc_ota_guard, test_legitimate_ranges_allowed)
{
	/* Metadata: neither bootloader nor (in this build) the running slot. */
	zassert_false(ota_fmc_range_forbidden(OTA_META_BASE, OTA_PAGE_SIZE),
	              "a metadata-record erase must not be refused");
	/* The INACTIVE slot (B, since this build runs from A) must stay open --
	 * this is the entire point of OTA: writing the other slot. */
	zassert_false(ota_fmc_range_forbidden(OTA_SLOT_B_BASE, OTA_SLOT_SIZE),
	              "the inactive slot must not be refused");
}

ZTEST_SUITE(gd32_bridge_fmc_ota_guard, NULL, NULL, NULL, NULL, NULL);
