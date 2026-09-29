/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 */

#include "i2c_event_priority.h"

bridge_i2c_event_t bridge_i2c_event_select(bool rbne, bool addsend, bool stpdet, bool ti)
{
	if (rbne) return BRIDGE_I2C_EVENT_RBNE;
	if (addsend) return BRIDGE_I2C_EVENT_ADDSEND;
	if (stpdet) return BRIDGE_I2C_EVENT_STPDET;
	if (ti) return BRIDGE_I2C_EVENT_TI;
	return BRIDGE_I2C_EVENT_NONE;
}
