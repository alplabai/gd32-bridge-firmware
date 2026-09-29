/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * One bounded PWM preload transaction.  The callback must only perform
 * register-only preload bookkeeping; it runs with update generation disabled
 * and transport interrupts briefly masked.
 */

#ifndef GD32_BRIDGE_HAL_GD32_PWM_PRELOAD_TRANSACTION_H
#define GD32_BRIDGE_HAL_GD32_PWM_PRELOAD_TRANSACTION_H

#include <stdint.h>

typedef void (*pwm_preload_transaction_fn)(void *context);

/* Preserve CTL0.UPDIS around fn().  Other CTL0 bits written by fn() are
 * retained, so restoring the caller's update-disable policy cannot revive a
 * stale CEN/SPM/CAM value. */
void pwm_preload_transaction(uint32_t periph, pwm_preload_transaction_fn fn, void *context);

#endif /* GD32_BRIDGE_HAL_GD32_PWM_PRELOAD_TRANSACTION_H */
