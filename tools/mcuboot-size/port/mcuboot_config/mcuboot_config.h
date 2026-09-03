/*
 * SPDX-License-Identifier: Apache-2.0
 * Throwaway port config for the mcuboot-size measurement. NOT production.
 *
 * Two build-time switches select the three configurations
 * .github/workflows/mcuboot-size.yml measures:
 *
 *   MCUBOOT_SIZE_PROBE_EC256   0 = hash-only integrity (SHA-256, no sig)
 *                               1 = SHA-256 + ECDSA-P256 verify (tinycrypt)
 *   MCUBOOT_SIZE_PROBE_REVERT  1 = also define MCUBOOT_DIRECT_XIP_REVERT
 *
 * All three configurations are direct-XIP (MCUBOOT_DIRECT_XIP always on) --
 * that is the mode gd32-bridge's real bootloader uses (SCB->VTOR relocation,
 * hal/gd32/init.c), never swap.
 */
#ifndef MCUBOOT_SIZE_PROBE_CONFIG_H
#define MCUBOOT_SIZE_PROBE_CONFIG_H

#ifndef MCUBOOT_SIZE_PROBE_EC256
#define MCUBOOT_SIZE_PROBE_EC256 0
#endif
#ifndef MCUBOOT_SIZE_PROBE_REVERT
#define MCUBOOT_SIZE_PROBE_REVERT 0
#endif

/* -- signature / hash algorithm ------------------------------------------ */
#if MCUBOOT_SIZE_PROBE_EC256
#define MCUBOOT_SIGN_EC256
#endif
#define MCUBOOT_SHA256

/* -- crypto backend -------------------------------------------------------
 * tinycrypt both for the hash-only SHA-256 path and for ECDSA-P256 verify:
 * the CAU on the real part (GD32G553) is symmetric-only (issue #50), so
 * asymmetric verify has to be software regardless -- this is what the task
 * asked to be sized. */
#define MCUBOOT_USE_TINYCRYPT

/* -- image layout: direct-XIP, one image, two slots, no scratch ---------- */
#define MCUBOOT_DIRECT_XIP
#if MCUBOOT_SIZE_PROBE_REVERT
#define MCUBOOT_DIRECT_XIP_REVERT
#endif
#define MCUBOOT_IMAGE_NUMBER 1

/* Validate the slot MCUboot is about to jump to on every boot (Kconfig
 * BOOT_VALIDATE_SLOT0 default in the real Zephyr port). */
#define MCUBOOT_VALIDATE_PRIMARY_SLOT

#define MCUBOOT_USE_FLASH_AREA_GET_SECTORS
#define MCUBOOT_MAX_IMG_SECTORS 128

/* No UART/RTT logging backend is stubbed -- BOOT_LOG_* compiles to nothing
 * (see bootutil/bootutil_log.h), which also matches the real bootloader
 * (32 KB has no room for a log formatter). */
/* #undef MCUBOOT_HAVE_LOGGING */

#define MCUBOOT_WATCHDOG_FEED() \
	do { \
	} while (0)
#define MCUBOOT_CPU_IDLE() \
	do { \
	} while (0)

#endif /* MCUBOOT_SIZE_PROBE_CONFIG_H */
