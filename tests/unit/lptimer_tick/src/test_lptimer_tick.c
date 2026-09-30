/* SPDX-License-Identifier: Apache-2.0
 *
 * Host tests for the LPTIMER tick arithmetic (gh#65).  Pins the trap from
 * the issue: a 2 ms tick is CARL = 3, not 3999 (which is 2 seconds).
 */

#include <zephyr/ztest.h>

#include "lptimer_tick.h"

ZTEST(lptimer_tick, test_two_ms_is_carl_3_not_3999)
{
	zassert_equal(lptimer_tick_carl(2000u), 3u);
	/* (3999 + 1) counts * 500 us = 2 s: what the wrong value would give. */
	zassert_equal(lptimer_tick_carl(2000000u), 3999u);
}

ZTEST(lptimer_tick, test_unrepresentable_requests_refused)
{
	zassert_equal(lptimer_tick_carl(0u), 0u);
	zassert_equal(lptimer_tick_carl(499u), 0u);
	zassert_equal(lptimer_tick_carl(500u), 0u); /* 1 count -> CARL 0 collides */
	zassert_equal(lptimer_tick_carl(750u), 0u); /* not a whole count */
	zassert_equal(lptimer_tick_carl(500u * 65537u), 0u); /* CARL 65536 > 16 bit */
	zassert_equal(lptimer_tick_carl(500u * 65536u), 65535u);
}

ZTEST(lptimer_tick, test_irc32k_tolerance_bounds)
{
	/* 50 ms nominal: 28 kHz -> 57142 us, 36 kHz -> 44444 us. */
	zassert_equal(lptimer_tick_worst_max_us(50000u), 57142u);
	zassert_equal(lptimer_tick_worst_min_us(50000u), 44444u);
}

ZTEST_SUITE(lptimer_tick, NULL, NULL, NULL, NULL, NULL);
