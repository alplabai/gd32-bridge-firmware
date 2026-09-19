/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include "gd32g5x3.h"

#include "bridge_critical.h"
#include "pwm_preload_transaction.h"

void pwm_preload_transaction(uint32_t periph, pwm_preload_transaction_fn fn, void *context)
{
	/* UPDIS blocks the timer's independent update event; the PRIMASK
	 * claim additionally makes the register writes and capture-mirror
	 * handoff indivisible to the other transport ISR.  This deliberately
	 * covers only a fixed register-only callback, never a wait or a
	 * dispatcher call. */
	const uint32_t irq_state  = bridge_irq_lock();
	const uint32_t saved_ctl0 = TIMER_CTL0(periph);
	TIMER_CTL0(periph)        = saved_ctl0 | (uint32_t)TIMER_CTL0_UPDIS;

	fn(context);

	/* Do not restore the complete saved word: a callback can legitimately
	 * change CEN/SPM/CAM.  Return only UPDIS to the policy this call found. */
	uint32_t ctl0 = TIMER_CTL0(periph);
	ctl0 &= ~(uint32_t)TIMER_CTL0_UPDIS;
	ctl0 |= saved_ctl0 & (uint32_t)TIMER_CTL0_UPDIS;
	TIMER_CTL0(periph) = ctl0;
	bridge_irq_unlock(irq_state);
}
