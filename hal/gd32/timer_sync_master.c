/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 */

#include "timer_sync_master.h"

#define TIMER_SYNC_TIMER_COUNT 2u

/* Bit n records whether compact timer n currently broadcasts TRGO0 to an
 * enabled slave.  timer_sync.c and pwm.c access this only while PRIMASK is
 * held; keep this module vendor-header-free so its state transition can be
 * exercised by the host unit tests. */
static volatile uint8_t timer_sync_master_mask;

bool timer_sync_master_active(uint8_t timer_id)
{
	if (timer_id >= TIMER_SYNC_TIMER_COUNT) return false;
	return (timer_sync_master_mask & (uint8_t)(1u << timer_id)) != 0u;
}

void timer_sync_master_set(uint8_t timer_id, bool active)
{
	if (timer_id >= TIMER_SYNC_TIMER_COUNT) return;
	const uint8_t bit = (uint8_t)(1u << timer_id);
	if (active) {
		timer_sync_master_mask |= bit;
	} else {
		timer_sync_master_mask &= (uint8_t)~bit;
	}
}

bool timer_sync_forced_update_allowed(uint8_t timer_id)
{
	return !timer_sync_master_active(timer_id);
}
