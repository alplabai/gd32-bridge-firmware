/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 bridge HAL backend -- master/slave timer sync.
 * Split move-only from hal/bridge_hw_gd32.c (fw v0.2.8); see
 * hal/gd32/init.c for the backend-wide implementation notes.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bridge_hw.h"
#include "gd32g5x3.h"

#include "gd32_common.h"

/* Master/slave id (host-side enum) -> GD32 peripheral base address.
 * The protocol-level contract numbers the three advanced timers
 * compactly as 0 = TIMER0, 1 = TIMER7, 2 = TIMER19; this keeps the
 * wire byte small without naming the underlying SoC peripheral. */
static uint32_t timer_sync_periph(uint8_t id)
{
	switch (id) {
	case 0u:
		return TIMER0;
	case 1u:
		return TIMER7;
	case 2u:
		return TIMER19;
	default:
		return 0u;
	}
}

/* Slave id -> master id -> SYSCFG internal-trigger-router code
 * (TIMER_SMCFG_TRGSEL_ITIn, gd32g5x3_syscfg.h) that makes the slave's
 * ITIx actually source from the requested master's TRGO0.
 *
 * Each row is the TIMER0/TIMER7/TIMER19 line of the GD32G553 internal-
 * trigger connection table, GD32G553 User Manual Rev1.2 p.570:
 *
 *   TIMERx   ITI0          ITI1          ...  ITI5          ...  ITI9
 *   TIMER0   -             TIMER1_TRGO0  ...  TIMER7_TRGO0  ...  TIMER19_TRGO0
 *   TIMER7   TIMER0_TRGO0  TIMER1_TRGO0  ...  -             ...  TIMER19_TRGO0
 *   TIMER19  TIMER0_TRGO0  TIMER1_TRGO0  ...  TIMER7_TRGO0  ...  -
 *
 * i.e. the ITIx index that names a given upstream timer is NOT the
 * same index on every slave row -- selecting ITI0 unconditionally
 * (the bug this table replaces, alp-sdk gd32-bridge-firmware#42) hits
 * the wrong master on 4 of the 6 (master, slave) pairs the wire
 * protocol can express, and hits nothing at all on the two pairs
 * whose ITI0 entry is "-".
 *
 * ITI0/ITI5/ITI9 encode to 0x01/0x0A/0x0E respectively (TSCFG15[4:0],
 * UM Rev1.2 p.83); 0 is not a valid encoding for any ITIx and marks a
 * (master, slave) pair absent from the table above -- there is no
 * SMC/TRGS fallback field on this part to fall back to (TIMERx_SMCFG
 * bits 6:4 and 2:0 are Reserved, UM Rev1.2 p.634/p.636), so that pair
 * must be refused rather than silently miswired. The diagonal
 * (master == slave) is rejected before this table is read. */
static const uint32_t timer_sync_iti[3][3] = {
	/*                    master=TIMER0            master=TIMER7            master=TIMER19          */
	/* slave=TIMER0  */ { 0u, TIMER_SMCFG_TRGSEL_ITI5, TIMER_SMCFG_TRGSEL_ITI9 },
	/* slave=TIMER7  */ { TIMER_SMCFG_TRGSEL_ITI0, 0u, TIMER_SMCFG_TRGSEL_ITI9 },
	/* slave=TIMER19 */ { TIMER_SMCFG_TRGSEL_ITI0, TIMER_SMCFG_TRGSEL_ITI5, 0u },
};

int bridge_hw_timer_sync(uint8_t master, uint8_t slave, uint8_t mode)
{
	if (master == slave) return BRIDGE_HW_ERR_INVAL;
	const uint32_t mp = timer_sync_periph(master);
	const uint32_t sp = timer_sync_periph(slave);
	if (mp == 0u || sp == 0u) return BRIDGE_HW_ERR_RANGE;

	/* Translate the wire `mode` byte (host-defined: 0 disabled,
     * 1 reset, 2 gated, 3 trigger, 4 external-clock, 5 encoder-mode-1)
     * to the vendor's SMC encoding (defined in gd32g5x3_syscfg.h:
     * TIMER_SLAVE_MODE_* + TIMER_QUAD_DECODER_MODE*).  The two
     * encodings differ in numeric value, so an explicit table is
     * needed here; anything outside the documented range returns
     * INVAL rather than silently selecting a default. */
	uint32_t slave_mode;
	switch (mode) {
	case 0u:
		slave_mode = TIMER_SLAVE_MODE_DISABLE;
		break;
	case 1u:
		slave_mode = TIMER_SLAVE_MODE_RESTART;
		break;
	case 2u:
		slave_mode = TIMER_SLAVE_MODE_PAUSE;
		break;
	case 3u:
		slave_mode = TIMER_SLAVE_MODE_EVENT;
		break;
	case 4u:
		slave_mode = TIMER_SLAVE_MODE_EXTERNAL0;
		break;
	case 5u:
		slave_mode = TIMER_QUAD_DECODER_MODE1;
		break;
	default:
		return BRIDGE_HW_ERR_INVAL;
	}

	/* Master side: emit the update event as TRGO0 (the canonical
     * "I just rolled over" pulse that synchronises a downstream
     * slave to the master's period) + flip the master-slave mode
     * bit so the master broadcasts its trigger.  Idempotent. */
	timer_master_output0_trigger_source_select(mp, TIMER_TRI_OUT0_SRC_UPDATE);
	timer_master_slave_mode_config(mp, TIMER_MASTER_SLAVE_MODE_ENABLE);

	/* Slave side: look up which SYSCFG-routed ITIx actually carries
     * the requested master's TRGO0 on this slave (timer_sync_iti
     * above); 0u marks a pair the GD32G553 internal-trigger table
     * does not connect at all.  Following the vendor pattern,
     * configure the slave-mode AFTER the input-trigger selection so
     * the slave doesn't act on a stale TRGI source. */
	const uint32_t iti = timer_sync_iti[slave][master];
	if (iti == 0u) return BRIDGE_HW_ERR_INVAL;
	timer_input_trigger_source_select(sp, iti);
	timer_slave_mode_select(sp, slave_mode);
	return BRIDGE_HW_OK;
}
