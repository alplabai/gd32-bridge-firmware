/* SPDX-License-Identifier: Apache-2.0
 *
 * HOST unit tests for the (slave, master) -> SYSCFG ITIx router-code
 * lookup (hal/gd32/timer_sync_iti.c).  This is the one slice of
 * bridge_hw_timer_sync() with no vendor-API dependency, so it stays split
 * into its own translation unit and can be tested directly.  A sibling
 * suite links the production HAL body against a minimal vendor mock (#142).
 *
 * Covers both supported ordered (master, slave) pairs plus the rejection
 * cases: master == slave, the uninitialised TIMER19 id 2 (#142), and an
 * out-of-range id.  Expected codes per GD32G553
 * User Manual Rev1.2 p.570 (connection table) and p.83 (TSCFG15[4:0]
 * encoding) -- see hal/gd32/timer_sync_iti.h for the derivation.
 */

#include <zephyr/ztest.h>

#include "timer_sync_iti.h"

#define TIMER0  0u
#define TIMER7  1u
#define TIMER19 2u

#define ITI0 0x01u
#define ITI5 0x0Au

/* ---- the two ordered (master, slave) pairs the wire protocol can
 * express, UM p.570 ------------------------------------------------- */

ZTEST(timer_sync_iti, test_master_timer7_slave_timer0)
{
	/* TIMER0's ITI5 entry is TIMER7_TRGO0. */
	zassert_equal(timer_sync_iti_lookup(TIMER0, TIMER7), ITI5);
}

ZTEST(timer_sync_iti, test_master_timer0_slave_timer7)
{
	/* TIMER7's ITI0 entry is TIMER0_TRGO0 -- one of the two pairs that
	 * already worked before #42's fix. */
	zassert_equal(timer_sync_iti_lookup(TIMER7, TIMER0), ITI0);
}

/* ---- rejection cases ------------------------------------------------ */

ZTEST(timer_sync_iti, test_rejects_master_equals_slave)
{
	/* Each of TIMER0/TIMER0 and TIMER7/TIMER7 is the
	 * diagonal of the internal-trigger table ("-" in UM p.570) -- no
	 * timer is its own ITI source.  bridge_hw_timer_sync() also rejects
	 * master == slave before ever calling this lookup, but the lookup
	 * itself must fail closed if that caller-side guard is ever
	 * dropped or bypassed. */
	zassert_equal(timer_sync_iti_lookup(TIMER0, TIMER0), 0u);
	zassert_equal(timer_sync_iti_lookup(TIMER7, TIMER7), 0u);
}

ZTEST(timer_sync_iti, test_rejects_uninitialised_timer19)
{
	zassert_equal(timer_sync_iti_lookup(TIMER0, TIMER19), 0u);
	zassert_equal(timer_sync_iti_lookup(TIMER19, TIMER0), 0u);
	zassert_equal(timer_sync_iti_lookup(TIMER19, TIMER19), 0u);
}

ZTEST(timer_sync_iti, test_rejects_out_of_range_slave)
{
	/* id 3 names no timer today -- the fourth-timer case the header's
	 * doc comment calls out; must return 0, never index past the
	 * table. */
	zassert_equal(timer_sync_iti_lookup(3u, TIMER0), 0u);
}

ZTEST(timer_sync_iti, test_rejects_out_of_range_master)
{
	zassert_equal(timer_sync_iti_lookup(TIMER0, 3u), 0u);
}

ZTEST(timer_sync_iti, test_rejects_out_of_range_both)
{
	zassert_equal(timer_sync_iti_lookup(255u, 255u), 0u);
}

ZTEST_SUITE(timer_sync_iti, NULL, NULL, NULL, NULL, NULL);
