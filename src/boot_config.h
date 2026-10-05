/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge persistent boot configuration (opt-in boot-time behaviour).
 *
 * Power-loss rule: no cut, at any instant, may brick the unit or fault boot.
 * Three mechanisms enforce it.
 *
 * 1. A/B records (same pattern as the OTA metadata, src/ota_layout.h): the
 *    config lives in two dedicated 2 KB pages, OTA_CONFIG_REC0/REC1, outside
 *    every image's link range (linker ASSERTs in toolchain/).  A SET erases
 *    and programs only the page that does NOT hold the newest valid record,
 *    so the old value stays intact until the new record's commit doubleword
 *    lands.  A cut mid-erase or mid-program leaves one invalid page and one
 *    good page: the old value wins.  Records carry a monotonic counter; the
 *    highest valid counter wins; an erased, torn, foreign or CRC-bad page
 *    reads as "absent".  No valid page at all = every flag OFF (the factory
 *    default), so a unit that never had the host set a flag behaves exactly
 *    as before.
 *
 * 2. ECC-fault-safe reads.  This flash is ECC protected and a doubleword
 *    half-written by a power cut can be uncorrectable; reading it raises the
 *    flash-ECC NMI, which resets the part (gh#36).  Every read here goes
 *    through ota_fmc_read_safe(), which masks that NMI source around the
 *    read, checks FMC_ECCCS.ECCDET0/ECCDET1 (vendor API fmc_ecc_flag_get /
 *    fmc_ecc_flag_clear, UM p.93-94) and reports "uncorrectable" instead of
 *    faulting.  The commit doubleword (counter-independent magic) is read
 *    first and the payload only once the magic is present.  A torn
 *    doubleword is therefore "absent -> default off" or "fall back to the
 *    other page", never a fault.
 *
 * 3. No flash write in a transport ISR.  A SET is queued
 *    (boot_config_request) and committed from the main loop
 *    (boot_config_tick, bridge_hw_tick) -- the host gets its reply
 *    immediately and polls GET until the stored value equals what it asked
 *    for.  The commit erases 2 x 1 KB pages on dual-bank parts (DBS=1), each
 *    <= 20 ms with interrupts masked (hal/fmc_ota.c erase_one_page): up to
 *    2 x 20 ms of blackout, plus main-loop latency (boot_config_tick can
 *    start ~40 ms late behind an ota_erase_tick walk).  There is no fixed
 *    idle window: polls must simply be retry-tolerant (gd32g553_boot_config_set).
 *    A SET equal to the stored value is a no-op and never touches flash.
 *
 * Today's only flag, BOOT_CONFIG_FLAG_SDMUX_EN_HIGH, drives PD11 (E1M IO29,
 * the EVK's SDIO_MUX_EN, active-LOW: low = SD connected, high = SD
 * disconnected) HIGH from bridge_hw_init(), so a provisioning run can keep
 * the microSD out across cold power cycles (alp-sdk #2697).  IO29's meaning
 * is carrier-specific, which is why this is a host-set opt-in and not a
 * firmware default.
 *
 * Flash access goes through the weak ota_fmc_* seams and ota_crc32().
 */
#ifndef GD32_BRIDGE_BOOT_CONFIG_H
#define GD32_BRIDGE_BOOT_CONFIG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "fmc_ota.h"
#include "ota_layout.h"

/* bit0: drive PD11 (E1M IO29 / SDIO_MUX_EN) HIGH at boot = SD disconnected. */
#define BOOT_CONFIG_FLAG_SDMUX_EN_HIGH 0x00000001u
#define BOOT_CONFIG_KNOWN_FLAGS        BOOT_CONFIG_FLAG_SDMUX_EN_HIGH

/* GPIO_WRITE mask bit of E1M IO29 (PD11; gpio_pad_map[12] in hal/gd32/gpio.c). */
#define BOOT_CONFIG_SDMUX_EN_PAD_MASK ((uint32_t)1u << 12)

#define BOOT_CONFIG_MAGIC 0x32464342u /* "BCF2" little-endian */

/* A record never reaches this counter (no wrap: a wrap would rank below the
 * old record).  At one SET per provisioning run it is unreachable. */
#define BOOT_CONFIG_COUNTER_LIMIT 0xFFFFFFF0u

/* Two flash doublewords: [0] payload, [8] commit (programmed last). */
typedef struct {
	uint32_t counter; /* monotonic; highest valid record wins */
	uint32_t flags;   /* BOOT_CONFIG_FLAG_* */
	uint32_t crc32;   /* ota_crc32 over counter + flags */
	uint32_t magic;   /* BOOT_CONFIG_MAGIC: the commit marker */
} boot_config_record_t;

_Static_assert(sizeof(boot_config_record_t) == 16u, "boot_config_record_t on-flash size");

#define BOOT_CONFIG_COMMIT_OFF offsetof(boot_config_record_t, crc32)

/* Read one record page ECC-safely.  false = absent: erased, torn
 * (uncorrectable or half-written), foreign or CRC-bad. */
bool boot_config_read_page(uint32_t base, boot_config_record_t *out);

/* Persist `flags` into the page that does not hold the newest valid record
 * (erase + payload doubleword + commit doubleword + readback).  Equal to the
 * stored value = OK without touching flash.  BASE LEVEL ONLY (it blocks for
 * a page erase); the protocol handler never calls it. */
ota_fmc_result_t boot_config_store(uint32_t flags);

/* Re-read both pages into the RAM cache and drop any queued SET (what a
 * reset does).  boot_config_apply() calls it. */
void boot_config_load(void);

/* Stored flags as of the last load/commit (RAM cache; safe in an ISR). */
uint32_t boot_config_flags(void);

/* Queue a SET for boot_config_tick().  true = accepted (queued, or already
 * queued with this very value, or equal to the stored value with nothing
 * queued); false = a different SET is still pending (reply BUSY). */
bool boot_config_request(uint32_t flags);

/* The queued value, if any (for tests and the handler). */
bool boot_config_pending(uint32_t *flags_out);

/* Base level: commit the queued SET.  Waits (returns, retries next pass)
 * while another caller owns the FMC funnel.  A failed commit drops the
 * request; the host's GET poll then times out and it re-sends the SET. */
void boot_config_tick(void);

/* Load the stored flags and apply them.  Called from bridge_hw_init() as
 * early as the pad map is usable; with no flag set it touches no pad at all.
 * The write goes through bridge_hw_gpio_write(), which preloads the level
 * before switching the pad to push-pull (no low glitch), and the pad then
 * stays driven until the GD32 resets -- identical to a host GPIO_WRITE of
 * IO29. */
void boot_config_apply(void);

#endif /* GD32_BRIDGE_BOOT_CONFIG_H */
