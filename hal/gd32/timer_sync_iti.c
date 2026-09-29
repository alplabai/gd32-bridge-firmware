/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * See timer_sync_iti.h for the derivation of this table and why it lives
 * in its own vendor-header-free translation unit.
 */

#include "timer_sync_iti.h"

/* UM Rev1.2 p.83: SYSCFG_TIMERxCFG2 TSCFG15[4:0] codes for ITI0/ITI5.
 * Match TIMER_SMCFG_TRGSEL_ITI0/5 (gd32g5x3_syscfg.h:636,645). */
#define TIMER_SYNC_ITI0 0x01u
#define TIMER_SYNC_ITI5 0x0Au

/* Slave id -> master id -> TSCFG15[4:0] code, UM Rev1.2 p.570. 0 marks a
 * pair the table does not connect (see timer_sync_iti.h). */
static const uint32_t timer_sync_iti[2][2] = {
	/*                   master=TIMER0    master=TIMER7 */
	/* slave=TIMER0 */ { 0u, TIMER_SYNC_ITI5 },
	/* slave=TIMER7 */ { TIMER_SYNC_ITI0, 0u },
};

uint32_t timer_sync_iti_lookup(uint8_t slave, uint8_t master)
{
	if (slave >= 2u || master >= 2u) {
		return 0u;
	}
	return timer_sync_iti[slave][master];
}
