/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Timer-sync master ownership, shared by timer_sync.c and pwm.c.
 */

#ifndef GD32_BRIDGE_TIMER_SYNC_MASTER_H
#define GD32_BRIDGE_TIMER_SYNC_MASTER_H

#include <stdbool.h>
#include <stdint.h>

/* A nonzero TIMER_SYNC mode makes @p timer_id (the compact wire id:
 * TIMER0 = 0, TIMER7 = 1) a TRGO0 master.  Callers must serialize a
 * query or update with bridge_irq_lock(): the state guards a forced UPG
 * against racing a newly configured slave route. */
bool timer_sync_master_active(uint8_t timer_id);
void timer_sync_master_set(uint8_t timer_id, bool active);

#endif /* GD32_BRIDGE_TIMER_SYNC_MASTER_H */
