/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <stdint.h>

#include "bridge_hw.h"
#include "gd32_common.h"
#include "pwm_internal.h"

static int pwm_counter_values(uint32_t  period_ns,
                              uint32_t  duty_ns,
                              bool      center_aligned,
                              uint32_t *arr_out,
                              uint32_t *compare_out)
{
	const uint32_t max_period_ticks =
	    center_aligned ? (2u * PWM_TIMER_ARR_MAX) : (PWM_TIMER_ARR_MAX + 1u);
	if (period_ns > max_period_ticks * PWM_TIMER_TICK_NS) return BRIDGE_HW_ERR_RANGE;

	/* Preserve the existing round-down to the fixed 1 us timer tick. */
	const uint32_t period_ticks = period_ns / PWM_TIMER_TICK_NS;
	const uint32_t duty_ticks   = duty_ns / PWM_TIMER_TICK_NS;
	if (period_ticks == 0u) return BRIDGE_HW_ERR_RANGE;

	uint32_t arr;
	uint32_t compare;
	if (!center_aligned) {
		arr     = period_ticks - 1u;
		compare = duty_ticks;
	} else {
		const uint32_t half_period = period_ticks / 2u;
		if (half_period == 0u) return BRIDGE_HW_ERR_RANGE;
		arr     = half_period;
		compare = duty_ticks / 2u;
	}

	/* CHxCV is also 16-bit.  At the maximum edge-aligned period,
	 * 100 % duty would produce compare == 65536 and wrap to zero (#16). */
	if (arr > PWM_TIMER_ARR_MAX || compare > PWM_TIMER_ARR_MAX) return BRIDGE_HW_ERR_RANGE;

	*arr_out     = arr;
	*compare_out = compare;
	return BRIDGE_HW_OK;
}

int bridge_hw_pwm_set(uint8_t channel, uint32_t period_ns, uint32_t duty_ns)
{
	if (channel >= PWM_CHANNEL_COUNT) return BRIDGE_HW_ERR_RANGE;
	if (duty_ns > period_ns) return BRIDGE_HW_ERR_INVAL;

	uint32_t  arr;
	uint32_t  compare;
	const int rv =
	    pwm_counter_values(period_ns, duty_ns, pwm_channel_center_aligned(channel), &arr, &compare);
	if (rv != BRIDGE_HW_OK) return rv;

	return pwm_apply_counter_values(channel, arr, compare);
}
