/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure (vendor-header-free) arithmetic behind CMD_ADC_STREAM_BEGIN2 /
 * CMD_ADC_STREAM_READ2 (docs/protocol-v0.15-design.md section 5).
 *
 * Kept out of hal/gd32/adc_stream.c for the same reason adc_dsp_chain.c and
 * tmu_q31_scale.c are: adc_stream.c pulls the vendor register header, so the
 * accounting rules -- the part a host test can actually prove -- live here
 * and tests/unit/adc_stream2 links this file alone.
 */

#ifndef GD32_BRIDGE_HAL_GD32_ADC_STREAM2_H
#define GD32_BRIDGE_HAL_GD32_ADC_STREAM2_H

#include <stdbool.h>
#include <stdint.h>

/* READ2 `dropped` value meaning "a discontinuity of unknown length".  The
 * reply then carries got = 0 and first_index = the previous delivered index. */
#define ADC_STREAM2_DISCONTINUITY 0xFFFFFFFFu

/* Largest unsigned backlog treated as a real (if overrun) backlog.  A
 * backlog above this cannot be skip arithmetic any more -- it means the
 * cursors disagree beyond recovery -- and answers the discontinuity
 * sentinel instead (design 5.4: "backlog > 2^31"). */
#define ADC_STREAM2_BACKLOG_MAX 0x80000000u

/* What one READ2 will do, computed from the ring state alone. */
typedef struct {
	uint32_t skip;        /* samples to discard before the first delivered one */
	uint32_t got;         /* samples to deliver: min(max_samples, remaining)   */
	uint32_t first_index; /* index of the first delivered sample (or D_prev)   */
	uint32_t dropped;     /* reply `dropped`: skip, or ADC_STREAM2_DISCONTINUITY */
	uint32_t remaining;   /* backlog still queued after this read              */
	uint32_t next_d;      /* the delivered index D after this read             */
	bool     sentinel;    /* discontinuity: deliver nothing, report the sentinel */
} adc_read2_plan_t;

/* Plan one READ2.
 *
 *   backlog        (total_written - total_read) mod 2^32
 *   ring_depth     the stream's own ring length, a power of two
 *   max_samples    the request's `max_samples`
 *   d_prev         delivered index D before this read
 *   discontinuity  a recovery event (ROVF recovery, DSP pump proc_gap) has
 *                  already invalidated the cursors: answer the sentinel
 *
 * Overrun is NEVER BUSY: when backlog > ring_depth - GUARD the oldest
 * `backlog - (ring_depth - GUARD)` samples are skipped, counted in `dropped`,
 * and the freshest ring_depth - GUARD are served with STATUS_OK.  The host
 * invariant first_index(n) == first_index(n-1) + got(n-1) + dropped(n)
 * (mod 2^32) holds for every non-sentinel read.
 *
 * A backlog within one ring "below zero" (the writer total momentarily one
 * lap short, gh#149) is a transient undercount and plans an empty read, not a
 * discontinuity. */
void adc_read2_plan(uint32_t          backlog,
                    uint16_t          ring_depth,
                    uint8_t           max_samples,
                    uint32_t          d_prev,
                    bool              discontinuity,
                    adc_read2_plan_t *out);

/* Ring length for a watermark W: 2*W, or the whole 1024-sample ring when
 * W == 0 (no events).  Always a power of two for the accepted watermarks. */
uint16_t adc_stream2_ring_depth(uint16_t watermark);

/* The pace timer's tick and period for a requested rate: a 1 MHz tick for
 * rates >= 16 Hz and a 10 kHz tick below, period = floor(tick / rate).  The
 * REALISED rate is exactly tick_hz / period_ticks (300 Hz -> 3333 ticks =
 * 300.03 Hz).  rate_hz must be 1..BRIDGE_ADC_STREAM_RATE_MAX_HZ. */
void adc_stream2_pace(uint32_t rate_hz, uint32_t *tick_hz, uint32_t *period_ticks);

/* True when one conversion fits inside one pacing period:
 *   ratio * (sample_cycles + 12.5) / ADCCK  <  period_ticks / tick_hz
 * with ADCCK = 36 MHz -- the ADC_READ residency model of gd32_common.h
 * (half-cycles in 64-bit integers, so the 12.5 stays exact).  BEGIN2 refuses
 * a rate for which this is false instead of silently degrading it. */
bool adc_stream2_conv_fits(uint16_t ratio,
                           uint16_t sample_cycles,
                           uint32_t period_ticks,
                           uint32_t tick_hz);

/* VREF_MEASURED flag: the VREFINT average code is non-zero AND the
 * reference derived from it lies inside [ADC_VREF_MIN_MV, ADC_VREF_MAX_MV],
 * i.e. adc_vref_mv is a measurement and not the ADC_VREF_MV 1800 fallback. */
bool adc_vref_is_measured(uint16_t vrefint_code);

#endif /* GD32_BRIDGE_HAL_GD32_ADC_STREAM2_H */
