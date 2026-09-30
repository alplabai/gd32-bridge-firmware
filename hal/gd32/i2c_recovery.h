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

/* Register/reset access the timeout recovery needs, behind function pointers
 * so the sequence is host-testable with a mock.  On silicon
 * transport_hw_gd32.c passes a `static const` table of static functions, so
 * the always_inline helpers below fold to the same register accesses the ISR
 * used to spell out inline (no behaviour change, #150 follow-up). */
typedef struct {
	void (*en_clear)(void);        /* I2C_CTL0.I2CEN = 0 */
	bool (*en_is_set)(void);       /* I2C_CTL0.I2CEN reads back set */
	void (*en_set)(void);          /* I2C_CTL0.I2CEN = 1 */
	void (*rcu_reset)(void);       /* RCU_I2C0RST pulse */
	void (*reinit)(void);          /* bridge_transport_i2c_hw_init() */
	bool (*timeout_pending)(void); /* I2C_INT_FLAG_TIMEOUT set */
	void (*timeout_clear)(void);   /* clear I2C_INT_FLAG_TIMEOUT */
} bridge_i2c_recovery_ops_t;

/* Documented software reset; escalates to an RCU reset + re-init when the
 * I2CEN=0 read-back spin is exhausted.  Returns true when it escalated. */
static inline __attribute__((always_inline)) bool
bridge_i2c_bus_release(const bridge_i2c_recovery_ops_t *ops)
{
	ops->en_clear();
	uint32_t spins = 0u;
	while (ops->en_is_set()) {
		if (bridge_i2c_en_clear_spin_exhausted(++spins)) {
			ops->rcu_reset();
			ops->reinit();
			return true;
		}
	}
	ops->en_set();
	return false;
}

/* ER-vector TIMEOUT arm: if TIMEOUT is pending, clear it and force the bus
 * release.  Returns true when it ran, i.e. the caller must resynchronise the
 * slave framing.  Never touches the peripheral when no timeout is pending, so
 * a healthy in-flight transfer is left alone.  `recoveries` / `escalations`
 * are the SWD-readable counters. */
static inline __attribute__((always_inline)) bool
bridge_i2c_timeout_service(const bridge_i2c_recovery_ops_t *ops,
                           volatile uint32_t               *recoveries,
                           volatile uint32_t               *escalations)
{
	if (!ops->timeout_pending()) {
		return false;
	}
	ops->timeout_clear();
	(*recoveries)++;
	if (bridge_i2c_bus_release(ops)) {
		(*escalations)++;
	}
	return true;
}

#endif /* GD32_BRIDGE_HAL_GD32_I2C_RECOVERY_H */
