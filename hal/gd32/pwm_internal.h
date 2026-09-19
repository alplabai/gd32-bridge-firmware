/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Private seam between vendor-free PWM command validation and the GD32
 * register-writing backend.
 */

#ifndef GD32_BRIDGE_HAL_GD32_PWM_INTERNAL_H
#define GD32_BRIDGE_HAL_GD32_PWM_INTERNAL_H

#include <stdbool.h>
#include <stdint.h>

bool pwm_channel_center_aligned(uint8_t channel);
int  pwm_apply_counter_values(uint8_t channel, uint32_t arr, uint32_t compare);

#endif /* GD32_BRIDGE_HAL_GD32_PWM_INTERNAL_H */
