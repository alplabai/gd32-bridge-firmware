/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 */

#include "i2c_timeout.h"

#define I2C_TIMEOUT_COUNTER_SCALE 2048u
#define I2C_TIMEOUT_RELOAD_MAX    4095u
#define I2C_TIMEOUT_MAX_US        35000u

bool bridge_i2c_stretch_timeout_reload(uint32_t i2c_clk_hz, uint16_t *reload_out)
{
	if (i2c_clk_hz == 0u || reload_out == 0) return false;

	/* Ceil the counter units: this makes the programmed timeout at least
	 * 25 ms. The products are at most ~1.5e17 for uint32_t clocks, safely
	 * within uint64_t. */
	uint64_t units = ((uint64_t)i2c_clk_hz * BRIDGE_I2C_STRETCH_TIMEOUT_US) /
	                 ((uint64_t)I2C_TIMEOUT_COUNTER_SCALE * 1000000u);
	if (((uint64_t)units * I2C_TIMEOUT_COUNTER_SCALE * 1000000u) <
	    ((uint64_t)i2c_clk_hz * BRIDGE_I2C_STRETCH_TIMEOUT_US)) {
		++units;
	}
	if (units == 0u) return false;
	if (units > (uint64_t)I2C_TIMEOUT_RELOAD_MAX + 1u) return false;
	if ((units * I2C_TIMEOUT_COUNTER_SCALE * 1000000u) >
	    ((uint64_t)i2c_clk_hz * I2C_TIMEOUT_MAX_US))
		return false;

	*reload_out = (uint16_t)(units - 1u);
	return true;
}
