/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Clock-derived I2C/SMBus stretch-timeout configuration.
 */

#ifndef GD32_BRIDGE_HAL_GD32_I2C_TIMEOUT_H
#define GD32_BRIDGE_HAL_GD32_I2C_TIMEOUT_H

#include <stdbool.h>
#include <stdint.h>

/* GD32G553 User Manual Rev1.2 §28.3.9 limits SMBus clock stretching to
 * 25--35 ms. Choose the earliest conforming limit: it detects a bridge
 * request handler that has starved I2C0_EV without needlessly extending the
 * time the shared BRD_I2C bus is unavailable to the PMIC or secure element.
 * The IP specifies TIMEOUT as a flag, not an automatic slave abort; the
 * recovery path remains responsible for proving pad-level release. */
#define BRIDGE_I2C_STRETCH_TIMEOUT_US 25000u

/* Convert an I2C kernel clock to the BUSTOA/BUSTOB reload value for a timeout
 * at least BRIDGE_I2C_STRETCH_TIMEOUT_US but no more than the manual's 35 ms
 * maximum. Both GD32 counters expire at (reload + 1) * 2048 * tI2CCLK.
 * Returns false when the 12-bit field cannot represent that window; the caller
 * must leave I2C disabled rather than silently configure a non-SMBus timeout. */
bool bridge_i2c_stretch_timeout_reload(uint32_t i2c_clk_hz, uint16_t *reload_out);

#endif /* GD32_BRIDGE_HAL_GD32_I2C_TIMEOUT_H */
