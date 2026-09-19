/* Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/ztest.h>

#include "pwm_ownership.h"

ZTEST(pwm_ownership, test_own_claim_does_not_block_one_shot)
{
	const uint8_t claims = pwm_timer_channel_bit(2u);
	zassert_false(pwm_timer_has_sibling_claim(claims, 2u), "own channel is allowed");
}

ZTEST(pwm_ownership, test_sibling_claim_blocks_one_shot)
{
	const uint8_t claims = (uint8_t)(pwm_timer_channel_bit(0u) | pwm_timer_channel_bit(3u));
	zassert_true(pwm_timer_has_sibling_claim(claims, 2u), "sibling channel is live");
}

ZTEST(pwm_ownership, test_timer_groups_reuse_four_channel_bit_positions)
{
	const uint8_t claims = pwm_timer_channel_bit(5u);
	zassert_false(pwm_timer_has_sibling_claim(claims, 5u), "own TIMER7 channel is allowed");
	zassert_true(pwm_timer_has_sibling_claim(claims, 7u), "TIMER7 sibling is live");
}

ZTEST_SUITE(pwm_ownership, NULL, NULL, NULL, NULL, NULL);
