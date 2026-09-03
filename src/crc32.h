/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge: CRC-32 (IEEE 802.3, reflected, init 0xFFFFFFFF, xorout
 * 0xFFFFFFFF).  Shared by the OTA state machine (src/ota.c) and the
 * bootloader (src/boot/boot_main.c).  Single-call use: seed with 0.
 *
 * Software-only, deliberately: the GD32G553's hardware CRC unit is a
 * single instance with no arbitration (no busy/lock/interrupt bit,
 * and its CRC_DATA state cannot be saved and restored across a
 * pre-emption -- see hal/bridge_hw.h and gh#58).  Do not route this
 * function, or the framing CRC in src/protocol.c, through that unit
 * without a single, provably non-pre-emptable owner.
 */
#ifndef GD32_BRIDGE_CRC32_H
#define GD32_BRIDGE_CRC32_H

#include <stddef.h>
#include <stdint.h>

uint32_t ota_crc32(uint32_t crc, const uint8_t *data, size_t len);

#endif /* GD32_BRIDGE_CRC32_H */
