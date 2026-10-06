/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 bridge HAL backend -- analog reference readiness latch.
 * Split move-only from hal/bridge_hw_gd32.c (fw v0.2.8); see
 * hal/gd32/init.c for the backend-wide implementation notes.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bridge_hw.h"
#include "gd32g5x3.h"

#include "gd32_common.h"

/* Analog-reference health latch.  bridge_hw_init arms the on-chip
 * reference buffer and records whether VREFRDY ever set; every
 * ADC/DAC entry point consults this before touching its converter.
 * A reference that never locked means EVERY conversion result is
 * garbage referenced to a dead node -- the v0.2.6 root cause -- and a
 * STATUS_OK reply carrying garbage millivolts is indistinguishable on
 * the wire from healthy analog (no GET_FAULT-style opcode exists).
 * Failing the analog ops with an IO error is the only honest signal
 * this protocol revision can give.
 *
 * The check self-heals: a buffer that locks LATE (after init's bounded
 * wait expired) is noticed by the first analog op that finds VREFRDY
 * set and promoted by the next base-level tick (vref_late_tick), which
 * also derives the runtime reference -- that op and any before the tick
 * still answer IO. */
bool vref_ok = false;

/* Set (from any context) when VREFRDY is seen after init's wait expired;
 * consumed by vref_late_tick() at base level. */
static volatile bool vref_remeasure_pending = false;

/* Request-path probe.  Runs in the transport ISR, so it only NOTES a late
 * lock: the VREFINT measurement drives ADC0 with long EOC waits and must
 * not run here.  vref_ok stays false (analog ops keep answering IO) until
 * vref_late_tick() has measured and published the reference. */
bool vref_ready_check(void)
{
	if (!vref_ok && vref_status_get() == SET) vref_remeasure_pending = true;
	return vref_ok;
}

bool vref_remeasure_pending_get(void)
{
	return vref_remeasure_pending;
}

/* Base level (bridge_hw_tick).  Measures under an ADC0 claim, then publishes
 * vref_ok.  A failed measurement still promotes: the buffer is locked and
 * adc_vref_mv keeps the ADC_VREF_MV default.  A busy ADC0 retries next tick. */
void vref_late_tick(void)
{
	if (!vref_remeasure_pending || !adc_periph_claim(ADC0)) return;
	(void)adc_vref_measure();
	adc_periph_release(ADC0);
	vref_remeasure_pending = false;
	vref_ok                = true;
}
