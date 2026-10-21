/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure arithmetic for BEGIN2 / READ2 -- see adc_stream2.h.
 */

#include "adc_stream2.h"

#include "gd32_common.h"
#include "protocol.h" /* GD32_BRIDGE_ADC_STREAM2_GUARD */

void adc_read2_plan(uint32_t          backlog,
                    uint16_t          ring_depth,
                    uint8_t           max_samples,
                    uint32_t          d_prev,
                    bool              discontinuity,
                    bool              undercount_prev,
                    adc_read2_plan_t *out)
{
	out->skip        = 0u;
	out->got         = 0u;
	out->first_index = d_prev;
	out->dropped     = 0u;
	out->remaining   = 0u;
	out->next_d      = d_prev;
	out->sentinel    = false;
	out->undercount  = false;

	if (discontinuity) {
		out->dropped  = ADC_STREAM2_DISCONTINUITY;
		out->sentinel = true;
		return;
	}
	/* Transient undercount (see adc_stream2.h): at most one ring below zero,
	 * once.  Empty, not lost. */
	if (backlog > ADC_STREAM2_BACKLOG_MAX && backlog >= (uint32_t)(0u - (uint32_t)ring_depth) &&
	    !undercount_prev) {
		out->undercount = true;
		return;
	}
	if (backlog > ADC_STREAM2_BACKLOG_MAX) {
		out->dropped  = ADC_STREAM2_DISCONTINUITY;
		out->sentinel = true;
		return;
	}

	uint32_t       remaining = backlog;
	const uint32_t keep      = (uint32_t)ring_depth - GD32_BRIDGE_ADC_STREAM2_GUARD;
	if (remaining > keep) {
		out->skip = remaining - keep;
		remaining = keep;
	}
	/* skip <= 2^31, so the 0xFFFFFFFE saturation of the wire field is
	 * unreachable; the sentinel value itself is never produced here. */
	out->dropped     = out->skip;
	out->first_index = d_prev + out->skip;
	out->got         = (remaining < (uint32_t)max_samples) ? remaining : (uint32_t)max_samples;
	out->remaining   = remaining - out->got;
	out->next_d      = out->first_index + out->got;
}

bool adc_stream2_ring_plan(uint16_t  watermark,
                           uint32_t  tick_hz,
                           uint32_t  period_ticks,
                           uint16_t *ring_depth,
                           uint16_t *granted_watermark)
{
	/* samples in one minimum lap = ceil(tick / period * LAP_MIN_US / 1e6) */
	const uint64_t num  = (uint64_t)tick_hz * ADC_STREAM2_LAP_MIN_US;
	const uint64_t den  = (uint64_t)period_ticks * 1000000u;
	const uint64_t need = (num + den - 1u) / den;
	if (need > BRIDGE_ADC_STREAM_RING_SAMPLES) return false;

	if (watermark == 0u) {
		*ring_depth        = (uint16_t)BRIDGE_ADC_STREAM_RING_SAMPLES; /* >= one lap (<= 500) */
		*granted_watermark = 0u;
		return true;
	}
	uint32_t target = 2u * (uint32_t)watermark;
	if (target < need) target = (uint32_t)need;
	uint32_t ring = 32u; /* 2 * the smallest watermark */
	while (ring < target)
		ring <<= 1;
	*ring_depth        = (uint16_t)ring;
	*granted_watermark = (uint16_t)(ring / 2u);
	return true;
}

void adc_stream2_pace(uint32_t rate_hz, uint32_t *tick_hz, uint32_t *period_ticks)
{
	const uint32_t tick = (rate_hz >= 16u) ? 1000000u : 10000u;
	*tick_hz            = tick;
	*period_ticks       = tick / rate_hz;
}

bool adc_stream2_conv_fits(uint16_t ratio,
                           uint16_t sample_cycles,
                           uint32_t period_ticks,
                           uint32_t tick_hz)
{
	/* ratio * (sample_cycles + 12.5) in half ADCCK cycles. */
	const uint64_t conv_half =
	    (uint64_t)ratio * ((uint64_t)sample_cycles * 2u + ADC_READ_CONV_HALF_CYCLES_12B);
	/* conv_half / (2 * ADCCK) < period_ticks / tick_hz, cross-multiplied. */
	const uint64_t lhs = conv_half * (uint64_t)tick_hz;
	const uint64_t rhs = (uint64_t)period_ticks * 2u * (uint64_t)ADC_READ_ADCCK_HZ;
	return lhs < rhs;
}

bool adc_vref_is_measured(uint16_t vrefint_code)
{
	if (vrefint_code == 0u) return false;
	/* The same derivation as adc_vref_mv_from_code(), but without its
	 * fallback to ADC_VREF_MV: an out-of-window result is "not measured". */
	const uint32_t mv =
	    ((uint32_t)ADC_VREFINT_TYP_MV * ADC_FULL_SCALE + (uint32_t)vrefint_code / 2u) /
	    (uint32_t)vrefint_code;
	return mv >= ADC_VREF_MIN_MV && mv <= ADC_VREF_MAX_MV;
}
