/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Per-timer PWM channel-claim helpers.  The caller passes one of the
 * bridge's eight logical PWM channels; bit positions deliberately repeat
 * across TIMER0 (0..3) and TIMER7 (4..7).
 */
#ifndef GD32_BRIDGE_PWM_OWNERSHIP_H
#define GD32_BRIDGE_PWM_OWNERSHIP_H

#include <stdbool.h>
#include <stdint.h>

static inline uint8_t pwm_timer_channel_bit(uint8_t channel)
{
	return (uint8_t)(1u << (channel & 0x03u));
}

static inline bool pwm_timer_has_sibling_claim(uint8_t claims, uint8_t channel)
{
	return (claims & (uint8_t)~pwm_timer_channel_bit(channel)) != 0u;
}

#endif /* GD32_BRIDGE_PWM_OWNERSHIP_H */
