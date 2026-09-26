/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Trial-capability marker planted in every OTA-slot app image (bench
 * fact 2026-09-26 follow-up, E1M-V2M103): the downgrade guard that gates
 * OTA_META_FLAG_TRIAL at COMMIT/ROLLBACK used to trust the fw_version the
 * HOST declared in OTA_BEGIN -- the 2026-09-26 incident that motivated
 * this whole trial/confirm dance was exactly the host declaring the BAD
 * image's TRUE, pre-fix version, which that guard would have believed and
 * committed the same image WITHOUT trial protection. Eligibility must
 * come from the image itself: `.trial_marker` (toolchain/
 * gd32g553_app_slot.ld.in) places this struct right after the vector
 * table, and ota_image_trial_capable() (src/ota_layout.h) scans the
 * SLOT'S OWN flash bytes for it at COMMIT/ROLLBACK. This translation unit
 * is only linked into the two OTA-slot targets (gd32-bridge-slot-a/b,
 * CMakeLists.txt) -- neither the non-partitioned full-flash image nor the
 * bootloader ever runs as a trial candidate, so neither needs the marker.
 *
 * tools/check_trial_marker.py runs the identical bounded scan against the
 * produced .bin as a build-time gate, so a slot image built without a
 * findable marker fails the build instead of silently shipping one.
 */
#include "ota_layout.h"

__attribute__((used, section(".trial_marker"))) const ota_trial_marker_t g_ota_trial_marker = {
	.magic            = OTA_TRIAL_MARKER_MAGIC_BYTES,
	.struct_version   = OTA_TRIAL_MARKER_STRUCT_VER,
	.capability_flags = OTA_TRIAL_CAP_CONFIRM,
};
