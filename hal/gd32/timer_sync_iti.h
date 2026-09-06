/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure (slave, master) -> SYSCFG internal-trigger-router code lookup for
 * bridge_hw_timer_sync().  No vendor register access here on purpose --
 * kept in its own translation unit with no gd32g5x3.h include, so a HOST
 * build can link and test all six pairs plus the rejection cases directly
 * (issue #77: no CI job compiles hal/gd32/timer_sync.c itself, since it
 * needs the Cortex-M33 vendor headers).
 *
 * Each row is the TIMER0/TIMER7/TIMER19 line of the GD32G553 internal-
 * trigger connection table, GD32G553 User Manual Rev1.2 p.570:
 *
 *   TIMERx   ITI0          ITI1          ...  ITI5          ...  ITI9
 *   TIMER0   -             TIMER1_TRGO0  ...  TIMER7_TRGO0  ...  TIMER19_TRGO0
 *   TIMER7   TIMER0_TRGO0  TIMER1_TRGO0  ...  -             ...  TIMER19_TRGO0
 *   TIMER19  TIMER0_TRGO0  TIMER1_TRGO0  ...  TIMER7_TRGO0  ...  -
 *
 * i.e. the ITIx index that names a given upstream timer is NOT the same
 * index on every slave row -- selecting ITI0 unconditionally (the bug this
 * table replaces, alp-sdk gd32-bridge-firmware#42) hits the wrong master
 * on 4 of the 6 (master, slave) pairs the wire protocol can express, and
 * hits nothing at all on the two pairs whose ITI0 entry is "-".
 *
 * ITI0/ITI5/ITI9 encode to 0x01/0x0A/0x0E respectively (SYSCFG_TIMERxCFG2
 * TSCFG15[4:0], UM Rev1.2 p.83); those words match
 * TIMER_SMCFG_TRGSEL_ITI0/5/9 in gd32g5x3_syscfg.h, restated here as raw
 * constants rather than included so this module stays vendor-header-free.
 * 0 is not a valid encoding for any ITIx and marks a (master, slave) pair
 * absent from the table above -- there is no SMC/TRGS fallback field on
 * this part to fall back to (TIMERx_SMCFG bits 6:4 and 2:0 are Reserved,
 * UM Rev1.2 p.634/p.636), so that pair must be refused rather than
 * silently miswired.
 */

#ifndef GD32_BRIDGE_TIMER_SYNC_ITI_H
#define GD32_BRIDGE_TIMER_SYNC_ITI_H

#include <stdint.h>

/* Look up the SYSCFG_TIMERxCFG2 TSCFG15[4:0] code that routes @p slave's
 * ITIx from @p master's TRGO0, using the wire ids bridge_hw_timer_sync()
 * already validates (0 = TIMER0, 1 = TIMER7, 2 = TIMER19; master == slave
 * and out-of-range ids are the caller's job to reject first).
 *
 * Returns the nonzero TSCFG15[4:0] code on a covered pair, or 0 for any
 * pair the GD32G553 internal-trigger table does not connect (including an
 * out-of-range slave/master id, so a fourth-timer id added to the wire
 * protocol without a matching table row fails closed rather than reading
 * out of bounds). */
uint32_t timer_sync_iti_lookup(uint8_t slave, uint8_t master);

#endif /* GD32_BRIDGE_TIMER_SYNC_ITI_H */
