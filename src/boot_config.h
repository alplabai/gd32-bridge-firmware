/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge persistent boot configuration (opt-in boot-time behaviour).
 *
 * One 16-byte record in its own flash page (OTA_CONFIG_BASE, ota_layout.h):
 * it survives power cycles and OTA slot swaps, and an erased / torn /
 * foreign page reads as "no flags" -- every flag is OFF by default, so a
 * unit that never had the host set one behaves exactly as before.
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

typedef struct {
	uint32_t magic; /* BOOT_CONFIG_MAGIC */
	uint32_t flags; /* BOOT_CONFIG_FLAG_* */
	uint32_t crc32; /* ota_crc32 over magic + flags */
	uint32_t _rsvd; /* 0; pads the record to two flash doublewords */
} boot_config_record_t;

_Static_assert(sizeof(boot_config_record_t) == 16u, "boot_config_record_t on-flash size");

static inline uint32_t boot_config_record_crc(const boot_config_record_t *r)
{
	return ota_crc32(0u, (const uint8_t *)r, offsetof(boot_config_record_t, crc32));
}

/* Stored flags; 0 (everything off) for an erased, torn or foreign page. */
static inline uint32_t boot_config_flags(void)
{
	boot_config_record_t r;
	memcpy(&r, ota_fmc_flash_ptr(OTA_CONFIG_BASE), sizeof r);
	if (r.magic != BOOT_CONFIG_MAGIC || r.crc32 != boot_config_record_crc(&r)) return 0u;
	return r.flags & BOOT_CONFIG_KNOWN_FLAGS;
}

/* Persist `flags` (erase + program + readback).  A power failure between
 * the erase and the program leaves an erased page = all flags off, i.e. the
 * safe default.  Returns OTA_FMC_RESULT_ERROR on a funnel/flash fault or a
 * failed readback. */
static inline ota_fmc_result_t boot_config_store(uint32_t flags)
{
	if (!ota_fmc_supported()) return OTA_FMC_RESULT_ERROR;
	boot_config_record_t r = { BOOT_CONFIG_MAGIC, flags & BOOT_CONFIG_KNOWN_FLAGS, 0u, 0u };
	r.crc32                = boot_config_record_crc(&r);
	ota_fmc_result_t rv    = ota_fmc_erase_range(OTA_CONFIG_BASE, OTA_PAGE_SIZE);
	if (rv != OTA_FMC_RESULT_OK) return rv;
	rv = ota_fmc_program(OTA_CONFIG_BASE, (const uint8_t *)&r, sizeof r);
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
