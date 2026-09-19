/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Reset-cause snapshot/decode helpers.  This header deliberately has no
 * vendor-header dependency: the host tests exercise the same RCU_RSTSCK
 * bit layout that the production backend passes in.
 */

#ifndef GD32_BRIDGE_HAL_GD32_RESET_REASON_H
#define GD32_BRIDGE_HAL_GD32_RESET_REASON_H

#include <stdint.h>

/* GD32G553 RCU_RSTSCK reset-source bits (UM Rev. 1.2, pp. 201-202).
 * Keep these local copies so the decode and its tests do not need the vendor
 * device header.  init.c statically checks them against the vendor names. */
#define BRIDGE_RESET_RSTSCK_OBLRSTF   0x00800000u
#define BRIDGE_RESET_RSTSCK_RSTFC     0x01000000u
#define BRIDGE_RESET_RSTSCK_BORRSTF   0x02000000u
#define BRIDGE_RESET_RSTSCK_EPRSTF    0x04000000u
#define BRIDGE_RESET_RSTSCK_PORRSTF   0x08000000u
#define BRIDGE_RESET_RSTSCK_SWRSTF    0x10000000u
#define BRIDGE_RESET_RSTSCK_FWDGTRSTF 0x20000000u
#define BRIDGE_RESET_RSTSCK_WWDGTRSTF 0x40000000u
#define BRIDGE_RESET_RSTSCK_LPRSTF    0x80000000u

/* Converts a captured RCU_RSTSCK value into the existing wire values:
 * UNKNOWN=0, POWER_ON=1, NRST_PIN=2, SOFT=3, WDT=4, BROWNOUT=5,
 * LOWPOWER=6.  The enum has no option-byte-loader value yet, so OBLRSTF
 * alone remains UNKNOWN until the host API and wire protocol change together.
 *
 * Several flags may be set.  This is deterministic diagnostic precedence,
 * not a chronology claim: WDT, software, low-power, then power-on, external
 * reset, and brownout.  POR must precede EPR/BOR because the documented cold
 * reset value (0x0e000000) sets all three. */
uint8_t bridge_reset_reason_decode(uint32_t rstsck);

/* Reads the live register once and requests a clear by setting RSTFC.  The
 * returned value is the pre-clear snapshot; callers retain it for the boot
 * lifetime instead of reading the destructive hardware register again. */
uint32_t bridge_reset_reason_latch_and_clear(volatile uint32_t *rstsck);

#endif /* GD32_BRIDGE_HAL_GD32_RESET_REASON_H */
