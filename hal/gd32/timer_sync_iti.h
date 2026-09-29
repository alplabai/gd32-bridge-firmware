/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure (slave, master) -> SYSCFG internal-trigger-router code lookup for
 * bridge_hw_timer_sync().  No vendor register access here on purpose --
 * kept in its own translation unit with no gd32g5x3.h include, so a host
 * build can test both supported pairs plus the rejection cases without a
 * register model.  The production bridge_hw_timer_sync() body is tested
 * separately against a minimal vendor-API mock (#142).
 *
 * Each row is the TIMER0/TIMER7 line of the GD32G553 internal-
 * trigger connection table, GD32G553 User Manual Rev1.2 p.570:
 *
 *   TIMERx   ITI0          ITI1          ...  ITI5
 *   TIMER0   -             TIMER1_TRGO0  ...  TIMER7_TRGO0
 *   TIMER7   TIMER0_TRGO0  TIMER1_TRGO0  ...  -
 *
 * i.e. the ITIx index that names a given upstream timer is NOT the same
 * index on every slave row -- selecting ITI0 unconditionally (the bug this
 * table replaces, alp-sdk gd32-bridge-firmware#42) misses the TIMER7 ->
 * TIMER0 route.  TIMER19's silicon routes are intentionally absent here:
 * the firmware does not initialise TIMER19 and rejects its old wire id 2
 * before calling this helper (#142).
 *
 * ITI0/ITI5 encode to 0x01/0x0A respectively (SYSCFG_TIMERxCFG2
 * TSCFG15[4:0], UM Rev1.2 p.83); those words match
 * TIMER_SMCFG_TRGSEL_ITI0/5 in gd32g5x3_syscfg.h, restated here as raw
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
 * already validates (0 = TIMER0, 1 = TIMER7; master == slave and
 * out-of-range ids, including the unsupported TIMER19 id 2, are the
 * caller's job to reject first).
 *
 * Returns the nonzero TSCFG15[4:0] code on a covered pair, or 0 for any
 * pair the GD32G553 internal-trigger table does not connect (including an
 * out-of-range slave/master id, so a fourth-timer id added to the wire
 * protocol without a matching table row fails closed rather than reading
 * out of bounds). */
uint32_t timer_sync_iti_lookup(uint8_t slave, uint8_t master);

#endif /* GD32_BRIDGE_TIMER_SYNC_ITI_H */
