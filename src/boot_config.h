/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge persistent boot configuration (opt-in boot-time behaviour).
 *
 * One 16-byte record in its own flash page (OTA_CONFIG_BASE, ota_layout.h):
 * it survives power cycles and OTA slot swaps.  An erased or foreign page
 * reads as "no flags" -- every flag is OFF by default, so a unit that never
 * had the host set one behaves exactly as before.
 *
 * This flash is ECC: a doubleword half-written by a power cut can be
 * uncorrectable, and READING it raises the flash-ECC NMI (gh#36).  So the
 * record is laid out as two doublewords, payload first and magic LAST, and
 * the reader fetches the magic doubleword alone and touches the payload only
 * once the magic is there (the same shape as meta_read in boot_decide.c).
 * A cut between the two programs therefore reads as "no flags" without ever
 * reading the half-built payload.  Residual window: a cut during the page
 * erase or during the single magic-doubleword program can still leave that
 * doubleword uncorrectable; closing that needs an NMI-handler recovery.
 *
 * Today's only flag, BOOT_CONFIG_FLAG_SDMUX_EN_HIGH, drives PD11 (E1M IO29,
 * the EVK's SDIO_MUX_EN, active-LOW: low = SD connected, high = SD
 * disconnected) HIGH from bridge_hw_init(), so a provisioning run can keep
 * the microSD out across cold power cycles (alp-sdk #2697).  IO29's meaning
 * is carrier-specific, which is why this is a host-set opt-in and not a
 * firmware default.
 *
 * Header-only (same shape as hal/fmc_ota_guard.h) so protocol.c, the HAL
 * init and the host unit tests share one implementation without every test
 * target gaining a source file.  Flash access goes through the existing
 * weak ota_fmc_* seams and ota_crc32().
 */
#ifndef GD32_BRIDGE_BOOT_CONFIG_H
#define GD32_BRIDGE_BOOT_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "../hal/bridge_hw.h"
#include "crc32.h"
#include "fmc_ota.h"
#include "ota_layout.h"

/* bit0: drive PD11 (E1M IO29 / SDIO_MUX_EN) HIGH at boot = SD disconnected. */
#define BOOT_CONFIG_FLAG_SDMUX_EN_HIGH 0x00000001u
#define BOOT_CONFIG_KNOWN_FLAGS        BOOT_CONFIG_FLAG_SDMUX_EN_HIGH

/* GPIO_WRITE mask bit of E1M IO29 (PD11; gpio_pad_map[12] in hal/gd32/gpio.c). */
#define BOOT_CONFIG_SDMUX_EN_PAD_MASK ((uint32_t)1u << 12)

#define BOOT_CONFIG_MAGIC 0x31464342u /* "BCF1" little-endian */

/* Two flash doublewords: [0] payload, [8] commit (programmed last). */
typedef struct {
	uint32_t flags; /* BOOT_CONFIG_FLAG_* */
	uint32_t crc32; /* ota_crc32 over flags */
	uint32_t magic; /* BOOT_CONFIG_MAGIC: the commit marker */
	uint32_t _rsvd; /* 0 */
} boot_config_record_t;

_Static_assert(sizeof(boot_config_record_t) == 16u, "boot_config_record_t on-flash size");

#define BOOT_CONFIG_COMMIT_OFF offsetof(boot_config_record_t, magic)

static inline uint32_t boot_config_record_crc(const boot_config_record_t *r)
{
	return ota_crc32(0u, (const uint8_t *)r, offsetof(boot_config_record_t, crc32));
}

/* Stored flags; 0 (everything off) for an erased or foreign page, or a
 * record whose commit doubleword was never written.  The payload doubleword
 * is read only after the commit magic is seen. */
static inline uint32_t boot_config_flags(void)
{
	uint32_t magic;
	memcpy(&magic, ota_fmc_flash_ptr(OTA_CONFIG_BASE + BOOT_CONFIG_COMMIT_OFF), sizeof magic);
	if (magic != BOOT_CONFIG_MAGIC) return 0u;
	boot_config_record_t r = { 0u, 0u, magic, 0u };
	memcpy(&r, ota_fmc_flash_ptr(OTA_CONFIG_BASE), BOOT_CONFIG_COMMIT_OFF);
	if (r.crc32 != boot_config_record_crc(&r)) return 0u;
	return r.flags & BOOT_CONFIG_KNOWN_FLAGS;
}

/* Persist `flags` (erase + program payload + program commit + readback).
 * A request equal to the stored value returns OK without touching flash (the
 * transport is at-least-once: a replayed SET must not cost another erase
 * blackout and a wear cycle).  Returns OTA_FMC_RESULT_ERROR on a
 * funnel/flash fault or a failed readback. */
static inline ota_fmc_result_t boot_config_store(uint32_t flags)
{
	if (!ota_fmc_supported()) return OTA_FMC_RESULT_ERROR;
	flags &= BOOT_CONFIG_KNOWN_FLAGS;
	if (boot_config_flags() == flags) return OTA_FMC_RESULT_OK;
	boot_config_record_t r = { flags, 0u, BOOT_CONFIG_MAGIC, 0u };
	r.crc32                = boot_config_record_crc(&r);
	ota_fmc_result_t rv    = ota_fmc_erase_range(OTA_CONFIG_BASE, OTA_PAGE_SIZE);
	if (rv != OTA_FMC_RESULT_OK) return rv;
	rv = ota_fmc_program(OTA_CONFIG_BASE, (const uint8_t *)&r, BOOT_CONFIG_COMMIT_OFF);
	if (rv != OTA_FMC_RESULT_OK) return rv;
	rv = ota_fmc_program(OTA_CONFIG_BASE + BOOT_CONFIG_COMMIT_OFF,
	                     (const uint8_t *)&r + BOOT_CONFIG_COMMIT_OFF,
	                     sizeof r - BOOT_CONFIG_COMMIT_OFF);
	if (rv != OTA_FMC_RESULT_OK) return rv;
	return (boot_config_flags() == r.flags) ? OTA_FMC_RESULT_OK : OTA_FMC_RESULT_ERROR;
}

/* Apply the stored flags.  Called from bridge_hw_init() as early as the pad
 * map is usable; with no flag set it touches no pad at all.  The write goes
 * through bridge_hw_gpio_write(), which preloads the level before switching
 * the pad to push-pull (no low glitch), and the pad then stays driven until
 * the GD32 resets -- identical to a host GPIO_WRITE of IO29. */
static inline void boot_config_apply(void)
{
	if ((boot_config_flags() & BOOT_CONFIG_FLAG_SDMUX_EN_HIGH) != 0u) {
		(void)bridge_hw_gpio_write(BOOT_CONFIG_SDMUX_EN_PAD_MASK, BOOT_CONFIG_SDMUX_EN_PAD_MASK);
	}
}

#endif /* GD32_BRIDGE_BOOT_CONFIG_H */
