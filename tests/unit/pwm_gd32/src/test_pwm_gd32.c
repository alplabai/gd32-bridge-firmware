/* SPDX-License-Identifier: Apache-2.0
 *
 * Host integration tests for the real GD32 bridge_hw_pwm_set().
 */

#include <zephyr/ztest.h>

#include "bridge_hw.h"
#include "pwm_internal.h"

static bool     test_center_aligned;
static unsigned apply_call_count;
static uint8_t  applied_channel;
static uint32_t applied_arr;
static uint32_t applied_compare;

bool pwm_channel_center_aligned(uint8_t channel)
{
	zassert_equal(channel, 0u);
	return test_center_aligned;
}

int pwm_apply_counter_values(uint8_t channel, uint32_t arr, uint32_t compare)
{
	apply_call_count++;
	applied_channel = channel;
	applied_arr     = arr;
	applied_compare = compare;
	return BRIDGE_HW_OK;
}

static void reset_apply_seam(bool center_aligned)
{
	test_center_aligned = center_aligned;
	apply_call_count    = 0u;
	applied_channel     = UINT8_MAX;
	applied_arr         = 0u;
	applied_compare     = 0u;
}

static void expect_refused_without_apply(uint32_t period_ns, uint32_t duty_ns, bool center_aligned)
{
	reset_apply_seam(center_aligned);
	zassert_equal(bridge_hw_pwm_set(0u, period_ns, duty_ns), BRIDGE_HW_ERR_RANGE);
	zassert_equal(apply_call_count, 0u);
}

ZTEST(pwm_gd32, test_edge_accepts_exact_period_limit)
{
	reset_apply_seam(false);
	zassert_equal(bridge_hw_pwm_set(0u, 65536000u, 32768000u), BRIDGE_HW_OK);
	zassert_equal(apply_call_count, 1u);
	zassert_equal(applied_channel, 0u);
	zassert_equal(applied_arr, 65535u);
	zassert_equal(applied_compare, 32768u);
}

ZTEST(pwm_gd32, test_edge_rejects_above_period_limit_without_writes)
{
	expect_refused_without_apply(65536001u, 1000u, false);
	expect_refused_without_apply(100000000u, 50000000u, false);
}

ZTEST(pwm_gd32, test_edge_rejects_unrepresentable_full_duty_without_writes)
{
	expect_refused_without_apply(65536000u, 65536000u, false);
}

ZTEST(pwm_gd32, test_edge_accepts_full_duty_below_limit)
{
	reset_apply_seam(false);
	zassert_equal(bridge_hw_pwm_set(0u, 1000000u, 1000000u), BRIDGE_HW_OK);
	zassert_equal(apply_call_count, 1u);
	zassert_equal(applied_arr, 999u);
	zassert_equal(applied_compare, 1000u);
}

ZTEST(pwm_gd32, test_center_accepts_exact_period_limit)
{
	reset_apply_seam(true);
	zassert_equal(bridge_hw_pwm_set(0u, 131070000u, 131070000u), BRIDGE_HW_OK);
	zassert_equal(apply_call_count, 1u);
	zassert_equal(applied_arr, 65535u);
	zassert_equal(applied_compare, 65535u);
}

ZTEST(pwm_gd32, test_center_rejects_above_period_limit_without_writes)
{
	expect_refused_without_apply(131070001u, 1000u, true);
}

ZTEST(pwm_gd32, test_rejects_period_below_one_counter_cycle_without_writes)
{
	expect_refused_without_apply(999u, 0u, false);
	expect_refused_without_apply(1999u, 0u, true);
}

ZTEST(pwm_gd32, test_duty_above_period_is_invalid_without_writes)
{
	reset_apply_seam(false);
	zassert_equal(bridge_hw_pwm_set(0u, 1000000u, 1000001u), BRIDGE_HW_ERR_INVAL);
	zassert_equal(apply_call_count, 0u);
}

ZTEST(pwm_gd32, test_invalid_channel_is_rejected_without_writes)
{
	reset_apply_seam(false);
	zassert_equal(bridge_hw_pwm_set(UINT8_MAX, 1000000u, 500000u), BRIDGE_HW_ERR_RANGE);
	zassert_equal(apply_call_count, 0u);
}

ZTEST_SUITE(pwm_gd32, NULL, NULL, NULL, NULL, NULL);
