/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C0 software-reset spin policy -- the vendor-header-free half of #251.
 */

#ifndef GD32_BRIDGE_HAL_GD32_I2C_RECOVERY_H
#define GD32_BRIDGE_HAL_GD32_I2C_RECOVERY_H

#include <stdbool.h>
#include <stdint.h>

/* Bound on the I2CEN=0 read-back spin in the documented I2C0 software reset
 * (UM Rev1.2 p.1262 SS28.3.5): "Write I2CEN = 0 / Check I2CEN = 0 / Write
 * I2CEN = 1", I2CEN held low for >= 3 APB clock cycles, which "releases SCL
 * and SDA". At any APB1 frequency this firmware configures (8-216 MHz, see
 * hal/gd32/i2c_timeout.c), 3 cycles clears in single-digit poll iterations;
 * 64 is generously above that floor and still small enough that a genuinely
 * dropped I2CEN=0 write escalates within a handful of microseconds instead
 * of spinning forever with no watchdog to recover it (#251). */
#define BRIDGE_I2C_EN_CLEAR_SPIN_LIMIT 64u

/* Pure poll-count policy, host-testable without the vendor I2C register
 * block: true once the caller's spin counter reaches the bound above,
 * meaning the documented I2CEN=0 software reset alone did not clear and the
 * caller must escalate to a full RCU peripheral reset instead of spinning
 * further. `spins_taken` is the count of polls performed so far, including
 * the one that just observed I2CEN still set. */
static inline bool bridge_i2c_en_clear_spin_exhausted(uint32_t spins_taken)
{
	return spins_taken >= BRIDGE_I2C_EN_CLEAR_SPIN_LIMIT;
}

#endif /* GD32_BRIDGE_HAL_GD32_I2C_RECOVERY_H */
