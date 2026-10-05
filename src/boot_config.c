/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Persistent boot configuration: A/B record pages, ECC-safe reads, queued
 * base-level commit.  See boot_config.h for the power-loss rules.
 */

#include "boot_config.h"

#include <string.h>

#include "../hal/bridge_hw.h"
#include "crc32.h"

static uint32_t          s_stored;  /* RAM cache of the newest valid record's flags */
static volatile bool     s_pending; /* a SET is queued for boot_config_tick() */
static volatile uint32_t s_pending_flags;

static uint32_t record_crc(uint32_t counter, uint32_t flags)
{
	const uint32_t words[2] = { counter, flags };
	return ota_crc32(0u, (const uint8_t *)words, sizeof words);
}

bool boot_config_read_page(uint32_t base, boot_config_record_t *out)
{
	uint32_t commit[2];
	uint32_t payload[2];

	/* Commit doubleword first, payload only once the magic is there: a cut
	 * before the commit leaves a payload that may be uncorrectable and is
	 * never read. */
	if (!ota_fmc_read_safe(base + BOOT_CONFIG_COMMIT_OFF, commit, sizeof commit)) return false;
	if (commit[1] != BOOT_CONFIG_MAGIC) return false;
	if (!ota_fmc_read_safe(base, payload, sizeof payload)) return false;
	if (commit[0] != record_crc(payload[0], payload[1])) return false;
	if (payload[0] == 0u || payload[0] >= BOOT_CONFIG_COUNTER_LIMIT) return false;
	out->counter = payload[0];
	out->flags   = payload[1];
	out->crc32   = commit[0];
	out->magic   = commit[1];
	return true;
}

/* Newest valid record across both pages; *target = the page a new record
 * must go to (the page NOT holding it, REC0 when neither is valid). */
static bool newest_record(boot_config_record_t *newest, uint32_t *target)
{
	boot_config_record_t r0, r1;
	const bool           v0 = boot_config_read_page(OTA_CONFIG_REC0, &r0);
	const bool           v1 = boot_config_read_page(OTA_CONFIG_REC1, &r1);

	if (v0 && (!v1 || r0.counter > r1.counter)) {
		*newest = r0;
		*target = OTA_CONFIG_REC1;
		return true;
	}
	if (v1) {
		*newest = r1;
		*target = OTA_CONFIG_REC0;
		return true;
	}
	*target = OTA_CONFIG_REC0;
	return false;
}

ota_fmc_result_t boot_config_store(uint32_t flags)
{
	boot_config_record_t cur, rec, check;
	uint32_t             target;

	if (!ota_fmc_supported()) return OTA_FMC_RESULT_ERROR;
	flags &= BOOT_CONFIG_KNOWN_FLAGS;
	const bool have = newest_record(&cur, &target);
	if ((have ? cur.flags : 0u) == flags) return OTA_FMC_RESULT_OK; /* no-op SET */
	if (have && cur.counter + 1u >= BOOT_CONFIG_COUNTER_LIMIT) return OTA_FMC_RESULT_ERROR;

	rec.counter = have ? cur.counter + 1u : 1u;
	rec.flags   = flags;
	rec.crc32   = record_crc(rec.counter, rec.flags);
	rec.magic   = BOOT_CONFIG_MAGIC;

	ota_fmc_result_t rv = ota_fmc_erase_range(target, OTA_PAGE_SIZE);
	if (rv != OTA_FMC_RESULT_OK) return rv;
	rv = ota_fmc_program(target, (const uint8_t *)&rec, BOOT_CONFIG_COMMIT_OFF);
	if (rv != OTA_FMC_RESULT_OK) return rv;
	rv = ota_fmc_program(target + BOOT_CONFIG_COMMIT_OFF,
	                     (const uint8_t *)&rec + BOOT_CONFIG_COMMIT_OFF,
	                     sizeof rec - BOOT_CONFIG_COMMIT_OFF);
	if (rv != OTA_FMC_RESULT_OK) return rv;
	return (boot_config_read_page(target, &check) && check.counter == rec.counter &&
	        check.flags == rec.flags)
	           ? OTA_FMC_RESULT_OK
	           : OTA_FMC_RESULT_ERROR;
}

void boot_config_load(void)
{
	boot_config_record_t cur;
	uint32_t             target;

	s_pending = false;
	s_stored  = newest_record(&cur, &target) ? (cur.flags & BOOT_CONFIG_KNOWN_FLAGS) : 0u;
}

uint32_t boot_config_flags(void)
{
	return s_stored;
}

bool boot_config_pending(uint32_t *flags_out)
{
	if (!s_pending) return false;
	if (flags_out != NULL) *flags_out = s_pending_flags;
	return true;
}

bool boot_config_request(uint32_t flags)
{
	if (s_pending) return s_pending_flags == flags;
	if (flags == s_stored) return true;
	s_pending_flags = flags;
	s_pending       = true;
	return true;
}

void boot_config_tick(void)
{
	if (!s_pending) return;
	if (ota_fmc_funnel_busy()) return; /* OTA erase walk owns the FMC: retry next pass */
	const uint32_t flags = s_pending_flags;
	if (boot_config_store(flags) == OTA_FMC_RESULT_OK) s_stored = flags;
	s_pending = false; /* after the cache update: a SET racing in sees the new value */
}

void boot_config_apply(void)
{
	boot_config_load();
	if ((s_stored & BOOT_CONFIG_FLAG_SDMUX_EN_HIGH) != 0u) {
		(void)bridge_hw_gpio_write(BOOT_CONFIG_SDMUX_EN_PAD_MASK, BOOT_CONFIG_SDMUX_EN_PAD_MASK);
	}
}
