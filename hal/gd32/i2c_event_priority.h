/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure I2C-event priority selector.  Kept separate from the vendor-register
 * ISR so the simultaneous-flag ordering is host-testable.
 */

#ifndef GD32_BRIDGE_HAL_GD32_I2C_EVENT_PRIORITY_H
#define GD32_BRIDGE_HAL_GD32_I2C_EVENT_PRIORITY_H

#include <stdbool.h>

typedef enum {
	BRIDGE_I2C_EVENT_NONE = 0,
	BRIDGE_I2C_EVENT_RBNE,
	BRIDGE_I2C_EVENT_ADDSEND,
	BRIDGE_I2C_EVENT_STPDET,
	BRIDGE_I2C_EVENT_TI,
} bridge_i2c_event_t;

/* Selects the one event this ISR invocation services.  RBNE comes first:
 * on a combined write/repeated-START read, the final write byte can remain
 * pending at the same time as ADDSEND.  It must enter the framing seam before
 * ADDSEND calls i2c_slave_write_end(). */
bridge_i2c_event_t bridge_i2c_event_select(bool rbne, bool addsend, bool stpdet, bool ti);

#endif /* GD32_BRIDGE_HAL_GD32_I2C_EVENT_PRIORITY_H */
