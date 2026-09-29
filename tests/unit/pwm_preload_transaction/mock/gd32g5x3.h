/* SPDX-License-Identifier: Apache-2.0 */
/* Minimal register/CMSIS mock for pwm_preload_transaction.c. */
#ifndef GD32_BRIDGE_PWM_PRELOAD_TRANSACTION_MOCK_GD32G5X3_H
#define GD32_BRIDGE_PWM_PRELOAD_TRANSACTION_MOCK_GD32G5X3_H

#include <stdint.h>

#define TIMER0 ((uint32_t)0u)
#define TIMER7 ((uint32_t)1u)

#define TIMER_CTL0_CEN   ((uint32_t)(1u << 0))
#define TIMER_CTL0_UPDIS ((uint32_t)(1u << 1))
#define TIMER_CTL0_SPM   ((uint32_t)(1u << 3))

extern uint32_t mock_timer_ctl0[2];
extern uint32_t mock_primask;
extern uint32_t mock_disable_calls;
extern uint32_t mock_set_primask_calls;

static inline uint32_t *mock_timer_ctl0_ref(uint32_t periph)
{
	return &mock_timer_ctl0[periph];
}

#define TIMER_CTL0(periph) (*mock_timer_ctl0_ref(periph))

static inline uint32_t __get_PRIMASK(void)
{
	return mock_primask;
}

static inline void __disable_irq(void)
{
	mock_disable_calls++;
	mock_primask = 1u;
}

static inline void __set_PRIMASK(uint32_t value)
{
	mock_set_primask_calls++;
	mock_primask = value;
}

#endif /* GD32_BRIDGE_PWM_PRELOAD_TRANSACTION_MOCK_GD32G5X3_H */
