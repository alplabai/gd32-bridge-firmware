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

/* Fast-mode PSC / SCLDELY / SDADELY derived from the live I2C kernel clock
 * (hal/transport_hw_gd32.c; shared by the I2C0 slave and the I2C3 master
 * proxy, hal/gd32/i2cm.c).  Returns false -- outputs untouched -- below the
 * 4 MHz Fast-mode floor or if a field would not fit its 4 bits. */
bool bridge_i2c_timing_derive(uint32_t  apb1_hz,
                              uint32_t *psc,
                              uint32_t *scl_dely,
                              uint32_t *sda_dely);

#endif /* GD32_BRIDGE_HAL_GD32_I2C_TIMEOUT_H */
