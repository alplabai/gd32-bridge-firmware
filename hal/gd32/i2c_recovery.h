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

/* Stuck-SDA confirmation policy (#39, #295, #315).  Vendor-header-free and
 * host-tested; the caller gathers the pad sample and the ISR-side facts.
 *
 * SDA low with SCL high is the erratum 2.3.1 wedge, but it is ALSO what a
 * live write looks like while the master is preempted with SCL high in the
 * ACK clock phase (this slave holds SDA low to ACK).  #315: that stall
 * tripped the detector ten times inside a healthy 65-byte OTA_WRITE_CHUNK
 * and it released I2C0 before the last byte.  A wedge is silent, so:
 *   - any I2C event ISR activity since the previous poll cancels the
 *     candidate (`activity` is a free-running count bumped per event);
 *   - while a transaction is open (ADDSEND seen, no STPDET yet, `txn_open`)
 *     the silence must also last BRIDGE_I2C_STUCK_LIVE_QUIET_TICKS SysTick
 *     periods (50 ms each), far above any byte + ACK + stretch at 100 kHz
 *     (~90 us), before the bus is judged wedged.  An idle-bus wedge (the
 *     #296 cold-boot case) still confirms on the second consecutive poll.
 * Ceiling: a master stalled mid-transfer for over 200 ms is released; it
 * would already have timed out. */
#define BRIDGE_I2C_STUCK_LIVE_QUIET_TICKS 5u

typedef struct {
	uint32_t activity_seen;
	uint32_t quiet_since;
	uint8_t  polls;
} bridge_i2c_stuck_t;

/* `sda_low_scl_high`: the whole sample burst read SDA low with SCL high.
 * `now`: SysTick period count.  True when the caller must release the bus. */
static inline bool bridge_i2c_stuck_confirm(bridge_i2c_stuck_t *s,
                                            bool                sda_low_scl_high,
                                            uint32_t            activity,
                                            bool                txn_open,
                                            uint32_t            now)
{
	if (!sda_low_scl_high || activity != s->activity_seen) {
		s->activity_seen = activity;
		s->quiet_since   = now;
		s->polls         = 0u;
		return false;
	}
	if (s->polls < 2u) s->polls++;
	if (s->polls < 2u) return false;
	if (txn_open && (uint32_t)(now - s->quiet_since) < BRIDGE_I2C_STUCK_LIVE_QUIET_TICKS) {
		return false;
	}
	s->polls       = 0u;
	s->quiet_since = now;
	return true;
}

#endif /* GD32_BRIDGE_HAL_GD32_I2C_RECOVERY_H */
