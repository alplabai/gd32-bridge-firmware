/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host proof for the vendor-header-free I2CEN=0 spin bound (#251): the
 * unbounded `while (I2CEN still set) {}` in
 * hal/transport_hw_gd32.c:bridge_transport_i2c_stuck_poll() hung forever on
 * a dropped I2CEN=0 write, with no watchdog configured on `dev` to recover
 * it. This pins the extracted policy that bounds it.
 */

#include <zephyr/ztest.h>

#include "i2c_recovery.h"

ZTEST(i2c_recovery, test_spin_limit_is_generously_above_the_documented_3_cycle_floor)
{
	/* UM Rev1.2 p.1262 SS28.3.5 requires only >= 3 APB cycles; the bound
	 * must stay well clear of that so real hardware never escalates. */
	zassert_true(BRIDGE_I2C_EN_CLEAR_SPIN_LIMIT >= 3u);
}

ZTEST(i2c_recovery, test_below_limit_does_not_escalate)
{
	zassert_false(bridge_i2c_en_clear_spin_exhausted(1u));
	zassert_false(bridge_i2c_en_clear_spin_exhausted(BRIDGE_I2C_EN_CLEAR_SPIN_LIMIT - 1u));
}

ZTEST(i2c_recovery, test_at_and_beyond_limit_escalates)
{
	zassert_true(bridge_i2c_en_clear_spin_exhausted(BRIDGE_I2C_EN_CLEAR_SPIN_LIMIT));
	zassert_true(bridge_i2c_en_clear_spin_exhausted(BRIDGE_I2C_EN_CLEAR_SPIN_LIMIT + 1u));
	/* The bug this pins: an unbounded spin never reaches this state at
	 * all -- UINT32_MAX polls in is still "not yet escalated" without a
	 * bound, which is exactly the unrecoverable hang #251 reported. */
	zassert_true(bridge_i2c_en_clear_spin_exhausted(0xFFFFFFFFu));
}

ZTEST_SUITE(i2c_recovery, NULL, NULL, NULL, NULL, NULL);
