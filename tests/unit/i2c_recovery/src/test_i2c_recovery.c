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

#include <string.h>

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

/* ---- Mock HAL for the TIMEOUT recovery sequence (#150) ---------------- */

static struct {
	bool     timeout;   /* TIMEOUT flag pending */
	bool     en;        /* I2CEN bit */
	uint32_t en_sticky; /* reads of I2CEN that stay set after en_clear (UINT32_MAX = forever) */
	int      seq[16];
	int      n;
} m;

enum { EN_CLEAR = 1, EN_SET, RCU, REINIT, TO_CLEAR };

static void mk_en_clear(void)
{
	m.seq[m.n++] = EN_CLEAR;
	m.en         = false;
}
static bool mk_en_is_set(void)
{
	if (m.en_sticky > 0u) {
		if (m.en_sticky != 0xFFFFFFFFu) {
			m.en_sticky--;
		}
		return true;
	}
	return m.en;
}
static void mk_en_set(void)
{
	m.seq[m.n++] = EN_SET;
	m.en         = true;
}
static void mk_rcu(void)
{
	m.seq[m.n++] = RCU;
}
static void mk_reinit(void)
{
	m.seq[m.n++] = REINIT;
}
static bool mk_pending(void)
{
	return m.timeout;
}
static void mk_to_clear(void)
{
	m.seq[m.n++] = TO_CLEAR;
	m.timeout    = false;
}

static const bridge_i2c_recovery_ops_t mock_ops = { mk_en_clear, mk_en_is_set, mk_en_set,  mk_rcu,
	                                                mk_reinit,   mk_pending,   mk_to_clear };

static volatile uint32_t recoveries, escalations;

static void mock_reset(void)
{
	memset(&m, 0, sizeof(m));
	m.en       = true;
	recoveries = escalations = 0u;
}

ZTEST(i2c_recovery, test_timeout_releases_bus_and_requests_resync)
{
	mock_reset();
	m.timeout = true;
	zassert_true(bridge_i2c_timeout_service(&mock_ops, &recoveries, &escalations));
	const int want[] = { TO_CLEAR, EN_CLEAR, EN_SET };
	zassert_equal(m.n, 3);
	zassert_mem_equal(m.seq, want, sizeof(want));
	zassert_true(m.en);
	zassert_equal(recoveries, 1u);
	zassert_equal(escalations, 0u);
}

ZTEST(i2c_recovery, test_slow_but_bounded_readback_does_not_escalate)
{
	mock_reset();
	m.timeout   = true;
	m.en_sticky = BRIDGE_I2C_EN_CLEAR_SPIN_LIMIT - 1u;
	zassert_true(bridge_i2c_timeout_service(&mock_ops, &recoveries, &escalations));
	zassert_equal(escalations, 0u);
	zassert_true(m.en);
}

ZTEST(i2c_recovery, test_readback_at_exactly_the_spin_limit_escalates)
{
	mock_reset();
	m.timeout   = true;
	m.en_sticky = BRIDGE_I2C_EN_CLEAR_SPIN_LIMIT; /* Nth read is still set: bound hit */
	zassert_true(bridge_i2c_timeout_service(&mock_ops, &recoveries, &escalations));
	const int want[] = { TO_CLEAR, EN_CLEAR, RCU, REINIT };
	zassert_equal(m.n, 4);
	zassert_mem_equal(m.seq, want, sizeof(want));
	zassert_equal(escalations, 1u);
}

ZTEST(i2c_recovery, test_spin_bound_hit_escalates_to_rcu_reset_and_reinit)
{
	mock_reset();
	m.timeout   = true;
	m.en_sticky = 0xFFFFFFFFu; /* I2CEN never reads back clear */
	zassert_true(bridge_i2c_timeout_service(&mock_ops, &recoveries, &escalations));
	const int want[] = { TO_CLEAR, EN_CLEAR, RCU, REINIT };
	zassert_equal(m.n, 4);
	zassert_mem_equal(m.seq, want, sizeof(want));
	zassert_equal(recoveries, 1u);
	zassert_equal(escalations, 1u);
}

ZTEST(i2c_recovery, test_no_timeout_never_touches_the_peripheral)
{
	mock_reset(); /* healthy transfer: no TIMEOUT flag */
	zassert_false(bridge_i2c_timeout_service(&mock_ops, &recoveries, &escalations));
	zassert_equal(m.n, 0);
	zassert_true(m.en);
	zassert_equal(recoveries, 0u);
	zassert_equal(escalations, 0u);
}

ZTEST_SUITE(i2c_recovery, NULL, NULL, NULL, NULL, NULL);
