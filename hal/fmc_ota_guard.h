/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge OTA — flash-funnel address guard (#79).
 *
 * Split out of hal/fmc_ota.c into its own header so it is unit-testable on
 * the host: unlike the rest of hal/fmc_ota.c, this check has ZERO GD32
 * register dependencies -- it is pure arithmetic over ota_layout.h's
 * compile-time constants plus the BRIDGE_APP_SLOT_BASE build define, so it
 * needs no vendor gd32g5x3.h and no register mock to compile and run on a
 * host toolchain.  hal/fmc_ota.c includes this header and is the only
 * production caller; tests/unit/fmc_ota_guard/ includes it directly.
 */
#ifndef GD32_BRIDGE_FMC_OTA_GUARD_H
#define GD32_BRIDGE_FMC_OTA_GUARD_H

#include <stdbool.h>
#include <stdint.h>

#include "ota_layout.h"

#if defined(BRIDGE_OTA_PARTITIONED) && defined(BRIDGE_APP_SLOT_BASE)
/* [base, base+size) of the slot THIS BUILD executes from.  Same
 * compile-time fact src/ota.c's OTA_RUNNING_SLOT/OTA_RUNNING_SLOT_BASE
 * derive from BRIDGE_APP_SLOT_BASE -- the per-target -D CMakeLists.txt
 * already gives every partitioned build (:196-197, :201-202), and
 * hal/gd32/init.c already consumes for SCB->VTOR.  Re-derived here,
 * independently of src/ota.c, so the funnel's own guard does not depend
 * on what a caller computes as an erase target. */
#define OTA_RUNNING_SLOT_BASE ((uint32_t)(BRIDGE_APP_SLOT_BASE))
#define OTA_RUNNING_SLOT_END  (OTA_RUNNING_SLOT_BASE + OTA_SLOT_SIZE)
#endif

/* Single address-range guard for the erase/program funnels (#79).  Every
 * erase and program call passes through here, so one check covers all
 * present and future callers instead of relying on each caller to remember
 * one.  Rejects on INTERVAL INTERSECTION (does this span touch protected
 * flash), not base equality, so it still catches a range that merely
 * overlaps a protected region without starting on its base.
 *
 * Always rejects the bootloader (0x08000000..OTA_BOOTLOADER_BASE+SIZE):
 * there is no recovery from erasing it on this hardware revision short of
 * a bench SWD probe.  Additionally rejects the running slot when this
 * build knows which slot that is (BRIDGE_APP_SLOT_BASE, partitioned
 * builds only) -- on the default full-flash build OTA is inert
 * (src/ota.c's dispatcher is #if BRIDGE_OTA_PARTITIONED-gated) and this
 * function is never reached from that build's own request handling, so
 * skipping the running-slot half of the guard there costs nothing. */
static inline bool ota_fmc_range_forbidden(uint32_t base, uint32_t len)
{
	const uint32_t end = base + len;
	if (base < OTA_BOOTLOADER_BASE + OTA_BOOTLOADER_SIZE && OTA_BOOTLOADER_BASE < end) {
		return true;
	}
#if defined(OTA_RUNNING_SLOT_BASE)
	if (base < OTA_RUNNING_SLOT_END && OTA_RUNNING_SLOT_BASE < end) {
		return true;
	}
#endif
	return false;
}

#endif /* GD32_BRIDGE_FMC_OTA_GUARD_H */
