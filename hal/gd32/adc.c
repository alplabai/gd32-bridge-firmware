/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 bridge HAL backend -- single-shot ADC.
 * Split move-only from hal/bridge_hw_gd32.c (fw v0.2.8); see
 * hal/gd32/init.c for the backend-wide implementation notes.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bridge_hw.h"
#include "gd32g5x3.h"

#include "bridge_board_config.h"
#include "bridge_critical.h"
#include "gd32_common.h"
#include "i2c_timeout.h"

/* ----------------------------------------------------------------- */
/* ADC channels.                                                      */
/* ----------------------------------------------------------------- */

/* E1M ADC0..7 -> (ADC peripheral, channel index, pad).  Sourced from
 * maintainer-confirmed alp-sdk `metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv`
 * with channel + peripheral assignments cross-checked against the
 * GD32G553xx datasheet pin alt-function summary:
 *
 *   E1M ADC0  PD9   ADC3  CH12
 *   E1M ADC1  PB12  ADC3  CH2
 *   E1M ADC2  PE13  ADC2  CH2
 *   E1M ADC3  PE11  ADC2  CH13
 *   E1M ADC4  PC4   ADC1  CH4
 *   E1M ADC5  PA5   ADC1  CH12
 *   E1M ADC6  PA2   ADC0  CH2
 *   E1M ADC7  PA3   ADC0  CH3
 *
 * All four ADC peripherals carry two of the eight channels; the
 * init loop brings up each ADC once and the read body reconfigures
 * the routine channel + triggers a single conversion on demand. */

const gd32_adc_ch_t adc_channels_map[] = {
	[0] = { ADC3, ADC_CHANNEL_12, GPIOD, GPIO_PIN_9 },
	[1] = { ADC3, ADC_CHANNEL_2, GPIOB, GPIO_PIN_12 },
	[2] = { ADC2, ADC_CHANNEL_2, GPIOE, GPIO_PIN_13 },
	[3] = { ADC2, ADC_CHANNEL_13, GPIOE, GPIO_PIN_11 },
	[4] = { ADC1, ADC_CHANNEL_4, GPIOC, GPIO_PIN_4 },
	[5] = { ADC1, ADC_CHANNEL_12, GPIOA, GPIO_PIN_5 },
	[6] = { ADC0, ADC_CHANNEL_2, GPIOA, GPIO_PIN_2 },
	[7] = { ADC0, ADC_CHANNEL_3, GPIOA, GPIO_PIN_3 },
};
_Static_assert(sizeof(adc_channels_map) / sizeof(adc_channels_map[0]) == ADC_CHANNEL_MAP_COUNT,
               "adc_channels_map size must match ADC_CHANNEL_MAP_COUNT");

/* Per-channel sticky conversion-format overrides applied by
 * bridge_hw_adc_configure.  All three are pure caches: configure
 * validates + stores, and the read / stream-begin paths program them
 * into the converter (inside the ADCON==0 window DRES/OVSAMPCTL
 * require).  Defaults: 240 sample cycles, 12-bit resolution, no
 * oversampling.  Zero-initialised at boot, so bridge_hw_init's ADC
 * bring-up loop (init.c) seeds the non-zero defaults. */
uint16_t adc_sample_cycles_cache[8];
uint8_t  adc_resolution_bits_cache[8];
uint16_t adc_oversample_ratio_cache[8];

/* Runtime reference in mV (gd32-bridge-firmware#59); see gd32_common.h. */
uint16_t adc_vref_mv = ADC_VREF_MV;

/* Raw VREFINT average code behind adc_vref_mv; 0 = never measured or EOC
 * timeout.  SWD-readable so a healthy 1800 mV (code ~2730) is
 * distinguishable from the fallback 1800. */
uint16_t adc_vrefint_code = 0u;

/* VREFINT code -> reference mV.  The ADC converts against the reference,
 * so code = VREFINT / Vref * 4095  =>  Vref = VREFINT * 4095 / code.
 * Rounded; anything outside the sane window (or code 0) falls back. */
uint16_t adc_vref_mv_from_code(uint32_t code)
{
	if (code == 0u) return ADC_VREF_MV;
	uint32_t mv = ((uint32_t)ADC_VREFINT_TYP_MV * ADC_FULL_SCALE + code / 2u) / code;
	if (mv < ADC_VREF_MIN_MV || mv > ADC_VREF_MAX_MV) return ADC_VREF_MV;
	return (uint16_t)mv;
}

/* Code -> mV, rounded.  See gd32_common.h. */
uint16_t adc_code_to_mv(uint32_t code, uint16_t full_scale)
{
	if (full_scale == 0u) return 0u; /* unreachable: callers pass a resolution's range */
	if (code > full_scale) code = full_scale;
	return (uint16_t)((code * (uint32_t)adc_vref_mv + (uint32_t)full_scale / 2u) / full_scale);
}

/* DMAMUX request id for a converter's data-ready line. */
uint32_t adc_dma_request_id(uint32_t periph)
{
	return (periph == ADC1)   ? DMA_REQUEST_ADC1
	       : (periph == ADC2) ? DMA_REQUEST_ADC2
	       : (periph == ADC3) ? DMA_REQUEST_ADC3
	                          : DMA_REQUEST_ADC0;
}

/* Publish the (code, mV) pair as one unit so a preempting reader never sees
 * a new mV paired with a stale code. */
static void adc_vref_publish(uint16_t code, uint16_t mv)
{
	const uint32_t st = bridge_irq_lock();
	adc_vrefint_code  = code;
	adc_vref_mv       = mv;
	bridge_irq_unlock(st);
}

/* Sample VREFINT (ADC0 channel 18) 16x and latch adc_vref_mv.  Runs at boot
 * (before any request path) or from base level via vref_late_tick() with
 * ADC0 claimed; never call it from an ISR -- the EOC waits are unbounded by
 * ADC_READ_ISR_BUDGET_US.  Returns false (adc_vref_mv left unchanged,
 * adc_vrefint_code = 0) if EOC never came. */
bool adc_vref_measure(void)
{
	enum { N = 16u };
	uint32_t sum = 0u;
	adc_internal_channel_config(ADC0, ADC_CHANNEL_INTERNAL_VREFINT, ENABLE);
	for (volatile uint32_t d = 0u; d < 4096u; ++d) {
		/* VREFINT settle */
	}
	/* Own the converter format: a late-lock run happens after single-shot
	 * reads may have left ADC0 at a cached 10/8/6-bit resolution or with
	 * oversampling on, which would skew the code against the 4095 full
	 * scale used below.  Format and rank latch only with ADCON == 0, and
	 * the ADCON toggle drops the calibration, so redo the tSTAB dwell and
	 * the bounded calibration exactly like the burst read's format miss.  Every
	 * later read/stream re-applies its own format, so nothing to restore. */
	adc_format_invalidate(ADC0); /* reprogrammed below: the burst must redo its format */
	adc_disable(ADC0);
	/* Single conversion per trigger, no DMA: a burst may have left scan mode,
	 * a longer sequence and the DMA request bits set on this converter. */
	adc_dma_mode_disable(ADC0);
	adc_dma_request_after_last_disable(ADC0);
	adc_special_function_config(ADC0, ADC_SCAN_MODE, DISABLE);
	adc_channel_length_config(ADC0, ADC_ROUTINE_CHANNEL, 1u);
	adc_resolution_config(ADC0, ADC_RESOLUTION_12B);
	adc_oversample_mode_disable(ADC0);
	adc_routine_channel_config(ADC0, 0u, ADC_CHANNEL_18, ADC_DEFAULT_SAMPLE_CYCLES);
	adc_enable(ADC0);
	for (volatile uint32_t d = 0u; d < 4096u; ++d) {
		/* tSTAB dwell after ADCON */
	}
	if (!adc_calibrate_bounded(ADC0)) {
		adc_internal_channel_config(ADC0, ADC_CHANNEL_INTERNAL_VREFINT, DISABLE);
		adc_vref_publish(0u, adc_vref_mv);
		return false;
	}
	adc_flag_clear(ADC0, ADC_FLAG_EOC); /* drop a stale EOC from an earlier timeout */
	for (uint32_t i = 0u; i < N; ++i) {
		adc_software_trigger_enable(ADC0, ADC_ROUTINE_CHANNEL);
		uint32_t to = 100000u;
		while (!adc_flag_get(ADC0, ADC_FLAG_EOC) && --to) {
		}
		if (to == 0u) {
			adc_internal_channel_config(ADC0, ADC_CHANNEL_INTERNAL_VREFINT, DISABLE);
			adc_vref_publish(0u, adc_vref_mv);
			return false;
		}
		adc_flag_clear(ADC0, ADC_FLAG_EOC);
		sum += adc_routine_data_read(ADC0) & ADC_FULL_SCALE;
	}
	adc_internal_channel_config(ADC0, ADC_CHANNEL_INTERNAL_VREFINT, DISABLE);
	adc_vref_publish((uint16_t)(sum / N), adc_vref_mv_from_code(sum / N));
	return true;
}

/* Resolution bits (12/10/8/6) -> ADC_RESOLUTION_* register value.
 * Returns false for any other width so bridge_hw_adc_configure can
 * reject it as NOSUPPORT rather than silently clamping. */
static bool adc_resolution_reg(uint8_t bits, uint32_t *reg_out)
{
	switch (bits) {
	case 12u:
		*reg_out = ADC_RESOLUTION_12B;
		return true;
	case 10u:
		*reg_out = ADC_RESOLUTION_10B;
		return true;
	case 8u:
		*reg_out = ADC_RESOLUTION_8B;
		return true;
	case 6u:
		*reg_out = ADC_RESOLUTION_6B;
		return true;
	default:
		return false;
	}
}

/* Right-aligned code range for a resolution: (1<<bits)-1.  Used by
 * both read paths to scale a code to millivolts.  Unknown widths fall
 * back to the 12-bit range (they never reach here -- configure gates
 * them -- but the mv math must not divide by a bogus value). */
uint16_t adc_full_scale_for_bits(uint8_t bits)
{
	switch (bits) {
	case 10u:
		return 1023u;
	case 8u:
		return 255u;
	case 6u:
		return 63u;
	default:
		return ADC_FULL_SCALE; /* 12-bit */
	}
}

/* Oversample ratio -> (OVSR register value, OVSS shift enum).  The
 * wire contract (alp-sdk docs/gd32-bridge-protocol.md §3.9) rounds a caller's
 * ratio DOWN to the nearest power of two in 1..256: 0 or 1 means "no
 * oversampling", and any value >1 is floored to 2^n.  The GD32 OVSR
 * field is the conversion count minus one, and a matching right-shift
 * of log2(ratio) normalises the accumulator back to the selected
 * resolution's full-scale (sum of `ratio` codes each <= full_scale,
 * shifted right by log2(ratio), is again <= full_scale) -- so the mv
 * math divides by the resolution's full-scale regardless of ratio.
 * enable_out == false tells the caller to disable the mode rather than
 * program a 1x accumulator. */
static void
adc_oversample_params(uint16_t ratio, bool *enable_out, uint16_t *ovsr_out, uint32_t *shift_out)
{
	if (ratio <= 1u) {
		*enable_out = false;
		return;
	}
	if (ratio > ADC_OVERSAMPLE_RATIO_MAX) ratio = ADC_OVERSAMPLE_RATIO_MAX;
	/* Largest power of two <= ratio, with its exponent. */
	uint32_t log2 = 0u;
	uint16_t pow2 = 1u;
	while ((uint16_t)(pow2 << 1) <= ratio) {
		pow2 <<= 1;
		++log2;
	}
	*enable_out = true;
	*ovsr_out   = (uint16_t)(pow2 - 1u);
	*shift_out  = OVSCR_OVSS(log2); /* ADC_OVERSAMPLING_SHIFT_<log2>B */
}

/* The oversample ratio the HARDWARE actually runs for a channel: the
 * cache floored to a power of two in [1, ADC_OVERSAMPLE_RATIO_MAX], which
 * is what adc_oversample_params programs.  One helper so the residency
 * budget (#135) and the EOC timeout bound cannot drift apart from each
 * other or from the register value -- the EOC bound previously scaled by
 * the CLAMPED cache rather than the floored one, so a ratio of 200 sized
 * its bound for 200 while the converter ran 128. */
static uint16_t adc_effective_ratio(uint8_t channel)
{
	bool     enable = false;
	uint16_t ovsr   = 0u;
	uint32_t shift  = 0u;

	adc_oversample_params(adc_oversample_ratio_cache[channel], &enable, &ovsr, &shift);
	return enable ? (uint16_t)(ovsr + 1u) : 1u;
}

/* Program a channel's cached resolution + oversample into its ADC.
 * The caller MUST have the converter disabled (DRES lives in CTL0 and
 * OVSAMPCTL only latches with ADCON==0 -- the vendor's own
 * ADC3_resolution_oversample example brackets every change with
 * adc_disable/adc_enable).  Shared by the single-shot read (which
 * wraps it in a disable/enable/tSTAB) and stream_begin (already inside
 * its own disable..enable window).  That disable/enable bracket is
 * also an ADCON power-off/on: both callers must recalibrate
 * (adc_calibrate_bounded) after their tSTAB dwell, not just re-apply
 * the format -- the calibration factor does not survive it (#34). */
void adc_apply_conv_format(uint32_t periph, uint8_t channel)
{
	uint32_t res_reg;
	if (adc_resolution_reg(adc_resolution_bits_cache[channel], &res_reg)) {
		adc_resolution_config(periph, res_reg);
	}

	bool     ovs_en;
	uint16_t ovsr;
	uint32_t shift;
	adc_oversample_params(adc_oversample_ratio_cache[channel], &ovs_en, &ovsr, &shift);
	if (ovs_en) {
		/* ALL_CONVERT: one trigger runs all `ratio` conversions and
		 * raises a single EOC with the shifted average -- the read
		 * loop's per-sample software trigger is unchanged. */
		adc_oversample_mode_config(periph, ADC_OVERSAMPLING_ALL_CONVERT, shift, ovsr);
		adc_oversample_mode_enable(periph);
	} else {
		adc_oversample_mode_disable(periph);
	}
}

/* Bounded reimplementation of the vendor's adc_calibration_enable().
 * The SPL body spins `while (RSTCLB)` then `while (CLB)` with NO
 * timeout -- and the calibration FSM only advances on a healthy,
 * clocked converter.  adc_periph_restore() is reachable from the CS-EXTI
 * request handler (bridge_hw_adc_stream_end's restore), where an unbounded spin on a
 * wedged ADC takes the WHOLE LINK down -- the exact failure class the
 * read path's own EOC bound was added to stop (silicon 2026-06-04).
 * Without the irony: the self-heal for a wedged converter must not
 * itself trust that converter to terminate a loop.  Same register
 * sequence as the vendor, same bound family as the other handler-safe
 * waits in this file; returns false if either phase never completes.
 * NOT static: adc_stream.c's stream_begin and ROVF recovery share it
 * (declared in gd32_common.h) -- every ADCON toggle needs the same
 * bounded recalibration, not just the boot setup. */
bool adc_calibrate_bounded(uint32_t periph)
{
	uint32_t to;

	ADC_CTL1(periph) |= (uint32_t)ADC_CTL1_RSTCLB;
	for (to = 100000u; to != 0u; --to) {
		if (RESET == (ADC_CTL1(periph) & ADC_CTL1_RSTCLB)) break;
	}
	if (to == 0u) return false;

	ADC_CTL1(periph) |= (uint32_t)ADC_CTL1_CLB;
	for (to = 100000u; to != 0u; --to) {
		if (RESET == (ADC_CTL1(periph) & ADC_CTL1_CLB)) break;
	}
	return to != 0u;
}

/* Configure one converter after its boot-only reset and shared-clock setup.
 * The group reset and clock setup intentionally live outside this helper:
 * ADC0/1/2 share ADC_SYNCCTL, and reprogramming it while a sibling streams
 * changes that sibling's clock. */
bool adc_periph_boot_init(uint32_t periph)
{
	/* Boot, adc_periph_restore() (stream end) and every
	 * other re-init funnel through here, so this is the one place that makes the
	 * "burst format is applied" record true to the hardware again. */
	adc_format_invalidate(periph);
	adc_data_alignment_config(periph, ADC_DATAALIGN_RIGHT);
	adc_special_function_config(periph, ADC_SCAN_MODE, DISABLE);
	adc_channel_length_config(periph, ADC_ROUTINE_CHANNEL, 1u);
	adc_external_trigger_config(periph, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_DISABLE);
	adc_enable(periph);

	/* Stabilisation + calibration.  The datasheet wants tSTAB after
     * ADCON before calibrating; a generous spin costs microseconds.
     * The bounded calibration then waits on RSTCLB/CLB -- without it
     * the converter runs uncalibrated for the whole session (linearity
     * offsets land directly in every reported millivolt).  A false
     * return means the calibration FSM never finished: the converter
     * is configured but its accuracy is unknown -- callers on the
     * request path surface that as an IO error rather than serving
     * readings from a converter in an unproven state. */
	for (volatile uint32_t stab = 0u; stab < 4096u; ++stab) {
	}
	return adc_calibrate_bounded(periph);
}

/* One GD32G553 boot sequence: reset every converter before setting either
 * shared clock domain, then initialise each converter separately.  The SPL
 * restricts adc_clock_config() to ADC0 (the ADC0/1/2 domain) and ADC3 (its
 * own domain); ADC1 and ADC2 must never rewrite ADC0's shared SYNCCTL. */
void adc_periph_boot_reset_all(void)
{
	adc_deinit(ADC0);
	adc_deinit(ADC1);
	adc_deinit(ADC2);
	adc_deinit(ADC3);
}

void adc_shared_clock_init(void)
{
	adc_clock_config(ADC0, ADC_CLK_SYNC_HCLK_DIV6);
	adc_clock_config(ADC3, ADC_CLK_SYNC_HCLK_DIV6);
}

/* Request-path restore after stream teardown or a single-shot timeout.  It
 * deliberately resets neither an ADC peripheral nor either shared clock
 * domain: another converter in ADC0/1/2 may be streaming.  Clearing ADCON
 * gives the same safe configuration window for alignment, routine length and
 * trigger selection, and the following bounded calibration replaces the
 * factor invalidated by that ADCON edge. */
bool adc_periph_restore(uint32_t periph)
{
	adc_disable(periph);
	adc_dma_request_after_last_disable(periph);
	adc_dma_mode_disable(periph);
	adc_flag_clear(periph, ADC_FLAG_EOC);
	adc_flag_clear(periph, ADC_FLAG_ROVF);
	return adc_periph_boot_init(periph);
}

/* ---- per-converter ownership interlock (#133) ------------------------ *
 *
 * Two bridge channels ride each ADC peripheral (0/1 -> ADC3, 2/3 -> ADC2,
 * 4/5 -> ADC1, 6/7 -> ADC0), and both bridge_hw_adc_read and
 * bridge_hw_adc_stream_begin reconfigure that shared converter -- from
 * INTERRUPT context, at two different NVIC group priorities.  The only
 * mutual exclusion either had was a scan of adc_streams[].in_use, which
 * says nothing about a single-shot read in flight.
 *
 * So: an I2C-side CMD_ADC_READ(channel=6) passes the stream scan, does
 * adc_disable(ADC0), points routine rank 0 at ADC_CHANNEL_2 / PA2, emits
 * two of four samples -- and the SPI CS-EXTI handler pre-empts with
 * CMD_ADC_READ(channel=7), passes the same scan, and re-points the same
 * converter at PA3.  The I2C side resumes and reads its remaining samples
 * from THE WRONG PAD, then answers STATUS_OK.  Nothing on the wire
 * distinguishes that reading from a good one.
 *
 * The flag is claimed for the WHOLE disable/reconfigure/enable/convert
 * sequence, but interrupts are masked only across the test-and-set --
 * see bridge_critical.h on why the section must stay that short.  The
 * loser is told BRIDGE_HW_ERR_BUSY and returns immediately; nothing here
 * spins waiting for the flag, so there is no deadlock to construct even
 * though the claim can be held for milliseconds (#135). */
#define ADC_PERIPH_COUNT 4u

/* Dense slot index for the four converters.  A switch rather than
 * `channel >> 1` so the mapping does not silently follow a future
 * re-ordering of adc_channels_map[]. */
static uint8_t adc_periph_slot(uint32_t periph)
{
	switch (periph) {
	case ADC0:
		return 0u;
	case ADC1:
		return 1u;
	case ADC2:
		return 2u;
	case ADC3:
		return 3u;
	default:
		return ADC_PERIPH_COUNT; /* unreachable: adc_channels_map has no other */
	}
}

/* `volatile` is load-bearing here, unlike most of this tree: the flag is
 * written at one NVIC priority and read at another, and the compiler has
 * no reason to reload it across the claim. */
static volatile bool adc_periph_busy[ADC_PERIPH_COUNT];

/* Test-and-set.  True = the caller now owns `periph` and MUST release it
 * on every return path.  False = someone else holds it; answer BUSY. */
bool adc_periph_claim(uint32_t periph)
{
	const uint8_t slot = adc_periph_slot(periph);

	if (slot >= ADC_PERIPH_COUNT) return false;

	const uint32_t st       = bridge_irq_lock();
	const bool     free_now = !adc_periph_busy[slot];
	if (free_now) {
		adc_periph_busy[slot] = true;
	}
	bridge_irq_unlock(st);
	return free_now;
}

void adc_periph_release(uint32_t periph)
{
	const uint8_t slot = adc_periph_slot(periph);

	if (slot >= ADC_PERIPH_COUNT) return;
	/* A single aligned store; no section needed to clear it. */
	adc_periph_busy[slot] = false;
}

/* ---- burst read: hardware sequence + DMA --------------------------------- *
 *
 * CMD_ADC_READ takes N <= 8 consecutive samples of one channel.  They run as
 * ONE software-triggered scan of an N-rank regular sequence (every rank the
 * same channel), each conversion moved to adc_burst.codes by DMA, with the
 * transfer-complete interrupt as the only completion event.  Nothing spins on
 * EOC and nothing waits for a conversion: the old path polled EOC (~18-20 us per
 * sample on the bench) inside the priority-1 CS EXTI handler and re-powered +
 * recalibrated the converter on every read.
 *
 *   start()   CS EXTI ISR:  claim -> format (cached) -> arm DMA -> trigger
 *   DMA IRQ   FTF:          stop DMA -> codes to mV -> done() -> release
 *
 * State machine, adc_burst[slot].state, one context per converter (each has its
 * own burst DMA channel, bridge_board_config.h, so only a same-converter request
 * is BUSY):
 *
 *   IDLE --claim--> RUNNING --FTF/ERR--> (finish) --> IDLE
 *                      |  \--abort / watchdog / sync timeout--> ABORTING --> IDLE
 *
 * The transition into RUNNING and into ABORTING happen under bridge_irq_lock();
 * the work (DMA stop, converter stand-down) happens outside it, in the context
 * that won the transition.  The completion IRQ acts only on RUNNING, so a burst
 * being torn down can never also complete.
 *
 * Interrupt priorities: the DMA IRQs have the CS EXTI's preemption priority
 * (BRIDGE_ADC_BURST_IRQ_PRIO == BRIDGE_CS_IRQ_PRIO), so start()/abort() called
 * from the CS ISR and finish() never preempt each other.  The I2C ISR (lower
 * priority) may be preempted by either; it only ever uses the blocking wrapper.
 *
 * Teardown cost.  start()/abort() run in the prio-1 CS EXTI and finish() in the
 * DMA IRQ, so an error/abort stand-down must be a handful of register writes:
 * stop the DMA, clear ADCON, drop the DMA-request bits and the format record.
 * It never recalibrates (tSTAB + a bounded calibration, up to 2 x 100000
 * iterations on a wedged converter); the next burst misses the format record and
 * pays that calibration on its own request instead.
 *
 * Time base.  The two time bounds below (the base-level watchdog and the I2C
 * blocking wait) count DWT cycles, which run at the core clock.  A burst's
 * duration is fixed in HCLK cycles (ADCCK = HCLK / 6, ADC_CLK_SYNC_HCLK_DIV6), so
 * a bound in DWT cycles tracks it whatever the core clock is, including after a
 * relock failure drops the part to IRC8M.  Counting main-loop wakes instead would
 * not be a time base at all: the loop runs after EVERY interrupt.
 *
 * ---- applied-format record -------------------------------------------------
 *
 * Resolution (DRES) and oversampling latch only with ADCON clear, and an ADCON
 * toggle drops the calibration (UM Rev1.2 17.4.1 p.424: the factor lasts "until
 * the next ADC power-off"; p.447), so changing them costs a power cycle, the
 * tSTAB dwell and a tCAL = 902 1/fADC = 25.06 us recalibration (GD32G553xx
 * Datasheet Rev2.0 Table 4-35, at this driver's 36 MHz ADCCK).  The previous code
 * paid all of that on EVERY read.  The record below remembers what each
 * converter was last programmed to -- channel, sequence length, resolution,
 * effective oversample ratio, sample cycles; the sequence ranks and length are
 * part of the format because they are written inside the same ADCON-clear
 * window -- so an unchanged repeat read skips straight to arming the DMA.
 *
 * A miss runs the full sequence INCLUDING the recalibration: calibration must
 * follow every disable, and it does, because the only way the converter is ever
 * disabled here is inside that sequence.
 *
 * INVALIDATION.  The record is only true while nothing else touches the
 * converter.  Every other path that does MUST drop it:
 *   - adc_periph_boot_init(): boot, adc_periph_restore() (stream end), and
 *     anything else that re-initialises a converter;
 *   - adc_burst_converter_stop() (burst error, abort, watchdog, sync timeout);
 *   - adc_vref_measure() (ADC0 reprogrammed to VREFINT);
 *   - bridge_hw_adc_stream_begin() and adc_stream_recover_rovf() (adc_stream.c);
 *   - adc_deepsleep_quiesce() (power.c).
 * bridge_hw_adc_configure() needs no hook: the key is compared against the live
 * caches at every start, so a configure simply makes the next burst miss. */
typedef struct {
	bool     valid;
	uint8_t  channel; /* bridge channel */
	uint8_t  samples; /* sequence length */
	uint8_t  res_bits;
	uint16_t ovs_ratio; /* effective (floored) ratio */
	uint16_t sample_cycles;
} adc_applied_fmt_t;

static adc_applied_fmt_t adc_applied_fmt[ADC_PERIPH_COUNT];

_Static_assert(ADC_BURST_COUNT == ADC_PERIPH_COUNT, "one burst context per converter");

adc_burst_t adc_burst[ADC_BURST_COUNT];

/* Per-converter burst DMA channel, DMAMUX multiplexer channel and IRQ, indexed by
 * converter slot (bridge_board_config.h has the map and the reasoning). */
static const dma_channel_enum adc_burst_dma_ch[ADC_BURST_COUNT] = BRIDGE_ADC_BURST_DMA_CHANNELS;
static const uint8_t   adc_burst_dmamux_ch[ADC_BURST_COUNT]     = BRIDGE_ADC_BURST_DMAMUX_CHANNELS;
static const IRQn_Type adc_burst_dma_irqn[ADC_BURST_COUNT]      = BRIDGE_ADC_BURST_DMA_IRQNS;

/* Time bounds, in DWT cycles (see the Time base note above).
 *
 * ADC_BURST_RESIDENCY_CYCLES is ADC_READ_ISR_BUDGET_US expressed in core cycles
 * at the nominal 216 MHz: the longest a burst that passed the occupancy check can
 * convert for, since ADCCK is a fixed HCLK divisor.  Both bounds are a multiple
 * of it so a healthy burst never trips them; they exist only to free a converter
 * whose conversion or DMA request was lost. */
#define ADC_BURST_RESIDENCY_CYCLES (ADC_READ_ISR_BUDGET_US * (PWM_TIMER_CLK_HZ / 1000000u))
#define ADC_BURST_DEADLINE_CYCLES  (4u * ADC_BURST_RESIDENCY_CYCLES)

void adc_format_invalidate(uint32_t periph)
{
	const uint8_t slot = adc_periph_slot(periph);

	if (slot < ADC_PERIPH_COUNT) adc_applied_fmt[slot].valid = false;
}

void adc_format_invalidate_all(void)
{
	for (uint8_t slot = 0u; slot < ADC_PERIPH_COUNT; ++slot) {
		adc_applied_fmt[slot].valid = false;
	}
}

/* Claim the burst for `periph`: the converter flag AND this converter's burst
 * context, in one critical section.  False = a burst or another claimant of THIS
 * converter is active; the caller answers BUSY.  Other converters are unaffected. */
static bool adc_burst_claim(uint32_t periph)
{
	const uint8_t slot = adc_periph_slot(periph);

	if (slot >= ADC_PERIPH_COUNT) return false;

	adc_burst_t   *b  = &adc_burst[slot];
	const uint32_t st = bridge_irq_lock();
	const bool     ok = (b->state == ADC_BURST_IDLE) && !adc_periph_busy[slot];
	if (ok) {
		adc_periph_busy[slot] = true;
		b->periph             = periph;
		b->done               = 0;
		b->start_cycles       = DWT->CYCCNT;
		b->state              = ADC_BURST_RUNNING;
	}
	bridge_irq_unlock(st);
	return ok;
}

static void adc_burst_release(uint8_t slot)
{
	adc_burst_t *b = &adc_burst[slot];

	b->done  = 0;
	b->state = ADC_BURST_IDLE;
	adc_periph_release(b->periph);
}

/* Bounded wait for CHEN to read clear (UM Rev1.2 s8.4.7: count/address may be
 * written only then); a few register reads, not a wait on conversions. */
#define ADC_BURST_DMA_DISABLE_SPINS 64u

static bool adc_burst_dma_disable_confirm(uint8_t slot)
{
	dma_channel_disable(BRIDGE_ADC_BURST_DMA, adc_burst_dma_ch[slot]);
	for (uint32_t spin = 0u; spin < ADC_BURST_DMA_DISABLE_SPINS; ++spin) {
		if ((DMA_CHCTL(BRIDGE_ADC_BURST_DMA, adc_burst_dma_ch[slot]) & DMA_CHXCTL_CHEN) == 0u) {
			return true;
		}
	}
	return false;
}

/* Program the converter's burst channel: peripheral-to-memory, 16-bit, `samples`
 * beats, normal (non-circular) mode, FTF + ERR interrupts on.  CHEN is set
 * last. */
static bool adc_burst_dma_arm(uint8_t slot, uint32_t periph, uint8_t samples)
{
	const dma_channel_enum ch = adc_burst_dma_ch[slot];

	bridge_rcu_periph_clock_enable(RCU_DMAMUX);
	bridge_rcu_periph_clock_enable(BRIDGE_ADC_BURST_DMA_RCU);
	if (!adc_burst_dma_disable_confirm(slot)) return false;
	dma_deinit(BRIDGE_ADC_BURST_DMA, ch);

	dma_parameter_struct init;
	dma_struct_para_init(&init);
	init.periph_addr  = (uint32_t)(uintptr_t)&ADC_RDATA(periph);
	init.memory_addr  = (uint32_t)(uintptr_t)adc_burst[slot].codes;
	init.direction    = DMA_PERIPHERAL_TO_MEMORY;
	init.number       = samples;
	init.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
	init.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
	init.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
	init.memory_width = DMA_MEMORY_WIDTH_16BIT;
	init.priority     = DMA_PRIORITY_HIGH;
	init.request      = adc_dma_request_id(periph);
	dma_init(BRIDGE_ADC_BURST_DMA, ch, &init);

	dma_flag_clear(BRIDGE_ADC_BURST_DMA, ch, DMA_FLAG_FTF | DMA_FLAG_ERR);
	dma_interrupt_enable(BRIDGE_ADC_BURST_DMA, ch, DMA_INT_FTF | DMA_INT_ERR);
	nvic_irq_enable(
	    adc_burst_dma_irqn[slot], BRIDGE_ADC_BURST_IRQ_PRIO, BRIDGE_ADC_BURST_IRQ_SUBPRIO);
	dma_channel_enable(BRIDGE_ADC_BURST_DMA, ch);
	return true;
}

/* Stand the converter's burst channel down: channel off, DMAMUX request released
 * (same discipline as adc_stream_end, so a later stream selecting this ADC
 * request on the other controller never sees it routed twice), interrupt sources
 * masked, flags cleared.  False = CHEN would not clear. */
static bool adc_burst_dma_stop(uint8_t slot)
{
	const dma_channel_enum ch      = adc_burst_dma_ch[slot];
	const bool             stopped = adc_burst_dma_disable_confirm(slot);

	DMAMUX_RM_CHXCFG(adc_burst_dmamux_ch[slot]) &= ~DMAMUX_RM_CHXCFG_MUXID;
	dma_interrupt_disable(BRIDGE_ADC_BURST_DMA, ch, DMA_INT_FTF | DMA_INT_ERR);
	dma_flag_clear(BRIDGE_ADC_BURST_DMA, ch, DMA_FLAG_FTF | DMA_FLAG_ERR);
	return stopped;
}

/* Cheap converter stand-down for an error or abort (CS EXTI / DMA IRQ context):
 * ADCON clear aborts anything still converting, the DMA-request bits and flags
 * go, and the format record is dropped.  No tSTAB dwell, no recalibration: the
 * next burst misses the record and redoes both inside its own request. */
static void adc_burst_converter_stop(uint32_t periph)
{
	adc_disable(periph);
	adc_dma_request_after_last_disable(periph);
	adc_dma_mode_disable(periph);
	adc_flag_clear(periph, ADC_FLAG_EOC);
	adc_flag_clear(periph, ADC_FLAG_ROVF);
	adc_format_invalidate(periph);
}

/* Make `periph` hold the burst format for (channel, samples).  True = ready to
 * trigger (cache hit, or reprogrammed + recalibrated); false = the calibration
 * FSM never finished (wedged converter) -- the caller reports IO. */
static bool adc_burst_format_apply(const gd32_adc_ch_t *ch, uint8_t channel, uint8_t samples)
{
	adc_applied_fmt_t *f      = &adc_applied_fmt[adc_periph_slot(ch->periph)];
	const uint16_t     ratio  = adc_effective_ratio(channel);
	const uint16_t     cycles = adc_sample_cycles_cache[channel];
	const uint8_t      bits   = adc_resolution_bits_cache[channel];

	if (f->valid && f->channel == channel && f->samples == samples && f->res_bits == bits &&
	    f->ovs_ratio == ratio && f->sample_cycles == cycles) {
		return true; /* hit: converter already holds exactly this format */
	}

	f->valid = false;
	adc_disable(ch->periph);
	adc_apply_conv_format(ch->periph, channel);
	adc_special_function_config(ch->periph, ADC_SCAN_MODE, ENABLE);
	adc_channel_length_config(ch->periph, ADC_ROUTINE_CHANNEL, samples);
	for (uint8_t rank = 0u; rank < samples; ++rank) {
		adc_routine_channel_config(ch->periph, rank, ch->channel, cycles);
	}
	/* DMA request on every conversion and kept alive past the DMA's last beat
	 * (DDM): the channel is re-armed per burst, the converter side is not touched
	 * again.  Exactly `samples` conversions follow each trigger, so the channel's
	 * count is consumed exactly. */
	adc_dma_request_after_last_enable(ch->periph);
	adc_dma_mode_enable(ch->periph);
	adc_enable(ch->periph);
	for (volatile uint32_t stab = 0u; stab < 4096u; ++stab) {
		/* tSTAB dwell after ADCON (a few us of hardware settling, not a wait on a
		 * conversion) */
	}
	if (!adc_calibrate_bounded(ch->periph)) return false;

	f->channel       = channel;
	f->samples       = samples;
	f->res_bits      = bits;
	f->ovs_ratio     = ratio;
	f->sample_cycles = cycles;
	f->valid         = true;
	return true;
}

/* Common tail for completion and error.  Runs in the DMA IRQ. */
static void adc_burst_finish(uint8_t slot, int rv)
{
	adc_burst_t   *b      = &adc_burst[slot];
	const uint32_t periph = b->periph;
	const bool     ok     = adc_burst_dma_stop(slot);

	if (rv == BRIDGE_HW_OK && (!ok || SET == adc_flag_get(periph, ADC_FLAG_ROVF))) {
		rv = BRIDGE_HW_ERR_IO; /* a stuck channel or an overrun: the codes are not trusted */
	}
	if (rv == BRIDGE_HW_OK) {
		for (uint8_t i = 0u; i < b->samples; ++i) {
			b->mv[i] = adc_code_to_mv(b->codes[i], b->full_scale);
		}
	} else {
		adc_burst_converter_stop(periph);
	}

	void (*const done)(int, const uint16_t *, uint8_t) = b->done;
	if (done != 0) done(rv, b->mv, b->samples);
	adc_burst_release(slot);
}

/* Burst DMA transfer-complete / error interrupt, one vector per converter.  The
 * state check makes a stale pending harmless: an abort followed by a NEW burst
 * started in the same CS ISR leaves the old vector pending while the state is
 * RUNNING again, but the new arm cleared the flags, so neither source is set
 * and nothing happens. */
static void adc_burst_irq(uint8_t slot)
{
	const dma_channel_enum ch = adc_burst_dma_ch[slot];
	const bool err = dma_interrupt_flag_get(BRIDGE_ADC_BURST_DMA, ch, DMA_INT_FLAG_ERR) != RESET;
	const bool ftf = dma_interrupt_flag_get(BRIDGE_ADC_BURST_DMA, ch, DMA_INT_FLAG_FTF) != RESET;

	if (adc_burst[slot].state != ADC_BURST_RUNNING) {
		/* Nothing owns the channel (a stale pending, or a teardown already
		 * took it): drop the flags so the vector does not re-enter. */
		dma_flag_clear(BRIDGE_ADC_BURST_DMA, ch, DMA_FLAG_FTF | DMA_FLAG_ERR);
		return;
	}
	if (err) {
		adc_burst_finish(slot, BRIDGE_HW_ERR_IO);
	} else if (ftf) {
		adc_burst_finish(slot, BRIDGE_HW_OK);
	}
}

/* Strong definitions overriding the vendor startup's weak aliases; see
 * bridge_board_config.h for the channel map and priority.  Converter slot n
 * (ADC<n>) owns DMA1 CH(n+1). */
void DMA1_Channel1_IRQHandler(void)
{
	adc_burst_irq(0u);
}

void DMA1_Channel2_IRQHandler(void)
{
	adc_burst_irq(1u);
}

void DMA1_Channel3_IRQHandler(void)
{
	adc_burst_irq(2u);
}

void DMA1_Channel4_IRQHandler(void)
{
	adc_burst_irq(3u);
}

int bridge_hw_adc_read_start(uint8_t channel, uint8_t samples, bridge_hw_adc_read_done_fn done)
{
	if (done == 0) return BRIDGE_HW_ERR_INVAL;
	if (samples == 0u) return BRIDGE_HW_ERR_INVAL;
	if (channel >= ADC_CHANNEL_MAP_COUNT) return BRIDGE_HW_ERR_RANGE;
	if (samples > BRIDGE_HW_ADC_READ_MAX_SAMPLES) return BRIDGE_HW_ERR_RANGE;
	if (!vref_ready_check()) return BRIDGE_HW_ERR_IO; /* dead reference -- fail loud */

	const gd32_adc_ch_t *ch = &adc_channels_map[channel];

	/* Converter-sharing guard, the read-side mirror of stream_begin's
	 * stream-vs-stream check: two bridge channels ride each ADC peripheral
	 * (ch0/1 -> ADC3, 2/3 -> ADC2, 4/5 -> ADC1, 6/7 -> ADC0), and a stream owns
	 * its converter outright between BEGIN and END (external-trigger + circular
	 * DMA + DDM).  A burst on the sibling channel would re-point the stream's
	 * rank 0 and consume the EOC/DMA requests it depends on.  Refuse honestly;
	 * the host retries after STREAM_END. */
	for (uint8_t si = 0u; si < BRIDGE_ADC_STREAM_COUNT; ++si) {
		if (adc_streams[si].in_use &&
		    adc_channels_map[adc_streams[si].channel].periph == ch->periph) {
			return BRIDGE_HW_ERR_BUSY;
		}
	}

	/* Occupancy budget (#135).  Bound the PRODUCT of the two host-settable
	 * multipliers -- and the sample window they multiply -- against
	 * ADC_READ_ISR_BUDGET_US BEFORE touching the converter.  The SPI link no
	 * longer sits in an ISR for this time, but the burst still holds the
	 * converter, its burst DMA channel and (for I2C) the caller; see
	 * ADC_READ_ISR_BUDGET_US in gd32_common.h.  Rejecting rather than capping is
	 * deliberate: silently halving an oversample ratio would return a reading
	 * whose noise floor is not what the caller asked for, with STATUS_OK. */
	const uint32_t half_cycles_per_conv =
	    (uint32_t)(2u * adc_sample_cycles_cache[channel]) + ADC_READ_CONV_HALF_CYCLES_12B;
	const uint32_t residency_half_cycles =
	    (uint32_t)samples * (uint32_t)adc_effective_ratio(channel) * half_cycles_per_conv;
	if (residency_half_cycles > ADC_READ_BUDGET_HALF_CYCLES) {
		return BRIDGE_HW_ERR_RANGE;
	}

	/* Claim the converter AND its burst context (#133): covers read-vs-read and
	 * read-vs-stream_begin on one converter.  A burst on a DIFFERENT converter has
	 * its own DMA channel and does not conflict.  Every return below must
	 * release. */
	if (!adc_burst_claim(ch->periph)) return BRIDGE_HW_ERR_BUSY;
	const uint8_t slot = adc_periph_slot(ch->periph);
	adc_burst_t  *b    = &adc_burst[slot];
	b->done            = done;
	b->samples         = samples;
	b->full_scale      = adc_full_scale_for_bits(adc_resolution_bits_cache[channel]);

	if (!adc_burst_format_apply(ch, channel, samples)) {
		/* Calibration never finished: report IO rather than serve readings from
		 * an unproven converter.  The record stays invalid, so the next burst
		 * redoes the whole sequence.  Release before bailing (#133 x #80): a
		 * stranded claim would answer BUSY until reboot. */
		adc_burst_release(slot);
		return BRIDGE_HW_ERR_IO;
	}
	if (!adc_burst_dma_arm(slot, ch->periph, samples)) {
		adc_burst_release(slot);
		return BRIDGE_HW_ERR_IO;
	}

	/* A stale EOC/ROVF (e.g. a stream END's in-flight conversion) must not be
	 * mistaken for ours or stall the sequence. */
	adc_flag_clear(ch->periph, ADC_FLAG_EOC);
	adc_flag_clear(ch->periph, ADC_FLAG_ROVF);

	/* Go.  From here the DMA-complete interrupt owns the burst; the watchdog's
	 * clock starts at the trigger, not at the claim (the format apply above is
	 * not part of the conversion). */
	b->start_cycles = DWT->CYCCNT;
	adc_software_trigger_enable(ch->periph, ADC_ROUTINE_CHANNEL);
	return BRIDGE_HW_OK;
}

/* Tear a burst down that has already been moved to ABORTING by the caller. */
static void adc_burst_teardown(uint8_t slot)
{
	(void)adc_burst_dma_stop(slot);
	adc_burst_converter_stop(adc_burst[slot].periph);
	adc_burst_release(slot);
}

void bridge_hw_adc_read_abort(bridge_hw_adc_read_done_fn done)
{
	if (done == 0) return;

	for (uint8_t slot = 0u; slot < ADC_BURST_COUNT; ++slot) {
		const uint32_t st = bridge_irq_lock();
		const bool     mine =
		    (adc_burst[slot].state == ADC_BURST_RUNNING) && (adc_burst[slot].done == done);
		if (mine) adc_burst[slot].state = ADC_BURST_ABORTING;
		bridge_irq_unlock(st);

		if (mine) {
			adc_burst_teardown(slot);
			return; /* one command defers at a time: a callback owns at most one burst */
		}
	}
}

/* Watchdog: a burst converts for at most ADC_BURST_RESIDENCY_CYCLES (the
 * occupancy budget), so one still RUNNING after ADC_BURST_DEADLINE_CYCLES of
 * elapsed time never completed -- a wedged converter or a lost DMA request.  Tear
 * it down so the converter, the DMA channel and the claim come back.  No
 * callback: the SPI transport bounds its own wait (SPI_DRAIN_REWIND_BOUND) and a
 * new request cancels it, and staging a reply from base level would race the CS
 * ISR.
 *
 * Aged by TIME, not by calls.  bridge_hw_tick() runs after EVERY wake of the main
 * loop (`__WFI(); bridge_hw_tick();`): a host reply-read CS edge, a stream lap,
 * an I2C event or SysTick each invoke it, so a per-call counter would kill a
 * healthy burst on the next unrelated interrupt.  The unsigned subtraction is
 * wrap-safe for any elapsed time under the 2^32-cycle DWT period (~19.9 s at
 * 216 MHz); the tick runs at least every 50 ms. */
void adc_burst_tick(void)
{
	for (uint8_t slot = 0u; slot < ADC_BURST_COUNT; ++slot) {
		bool expired = false;

		const uint32_t st = bridge_irq_lock();
		if (adc_burst[slot].state == ADC_BURST_RUNNING &&
		    (uint32_t)(DWT->CYCCNT - adc_burst[slot].start_cycles) >= ADC_BURST_DEADLINE_CYCLES) {
			adc_burst[slot].state = ADC_BURST_ABORTING;
			expired               = true;
		}
		bridge_irq_unlock(st);

		if (expired) adc_burst_teardown(slot);
	}
}

void adc_deepsleep_quiesce(void)
{
	for (uint8_t slot = 0u; slot < ADC_BURST_COUNT; ++slot) {
		bool running = false;

		const uint32_t st = bridge_irq_lock();
		if (adc_burst[slot].state == ADC_BURST_RUNNING) {
			adc_burst[slot].state = ADC_BURST_ABORTING;
			running               = true;
		}
		bridge_irq_unlock(st);

		if (running) adc_burst_teardown(slot);
	}
	adc_format_invalidate_all();
}

/* Blocking read for the I2C link, whose reply must exist before the read phase
 * starts (its EV ISR clock-stretches meanwhile).  Same burst, same DMA: it spins
 * on a flag the DMA-complete IRQ sets -- not on EOC -- which is possible because
 * that IRQ outranks the I2C ISR.
 *
 * INTERIM.  This wait sits in the priority-2 I2C ISR and is only tolerable for as
 * long as ADC_READ is reachable over I2C at all.  From protocol 0.15 the
 * maintainer's decision is that Linux on BRD_I2C is limited to GPIO + SE_RST +
 * OTA: ADC_READ over I2C then answers STATUS_NOSUPPORT, and this function and its
 * wait go away with it.  ADC_READ stays on SPI, deferred.
 *
 * The wait is bounded by ELAPSED TIME, read from the live clock (DWT), never by
 * an iteration count (a raw count is ~5-9 ms at 216 MHz but ~27x longer on the
 * IRC8M fallback).  The bound is the smaller of
 *   - ADC_BURST_DEADLINE_CYCLES: a multiple of the burst's own worst case, which
 *     is fixed in HCLK cycles and so valid at any core clock, and
 *   - half the SMBus clock-stretch window (BRIDGE_I2C_STRETCH_TIMEOUT_US) at the
 *     LIVE core clock, bridge_core_clock_hz, which relock/fallback keeps current:
 *     on a degraded clock the host gets STATUS_IO inside the window instead of a
 *     stretch timeout.
 * On expiry the burst is aborted (converter stood down) and IO reported.
 *
 * Never call this from the SPI CS ISR (or anything at the burst IRQ's priority):
 * the completion could not run. */
static uint32_t adc_sync_wait_cycles(void)
{
	const uint64_t stretch_cycles =
	    ((uint64_t)bridge_core_clock_hz * (BRIDGE_I2C_STRETCH_TIMEOUT_US / 2u)) / 1000000ull;

	return (stretch_cycles < ADC_BURST_DEADLINE_CYCLES) ? (uint32_t)stretch_cycles
	                                                    : ADC_BURST_DEADLINE_CYCLES;
}

static struct {
	volatile bool done;
	int           rv;
	uint8_t       n;
	uint16_t      mv[BRIDGE_HW_ADC_READ_MAX_SAMPLES];
} adc_sync;

static void adc_sync_done(int rv, const uint16_t *mv, uint8_t samples)
{
	adc_sync.rv = rv;
	adc_sync.n  = 0u;
	if (rv == BRIDGE_HW_OK) {
		for (uint8_t i = 0u; i < samples && i < BRIDGE_HW_ADC_READ_MAX_SAMPLES; ++i) {
			adc_sync.mv[i] = mv[i];
		}
		adc_sync.n = samples;
	}
	adc_sync.done = true;
}

int bridge_hw_adc_read(uint8_t channel, uint8_t samples, uint16_t *mv)
{
	if (mv == 0) return BRIDGE_HW_ERR_INVAL;

	adc_sync.done = false;
	const int rv  = bridge_hw_adc_read_start(channel, samples, adc_sync_done);
	if (rv != BRIDGE_HW_OK) return rv;

	const uint32_t t0    = DWT->CYCCNT;
	const uint32_t limit = adc_sync_wait_cycles();
	while (!adc_sync.done && (uint32_t)(DWT->CYCCNT - t0) < limit) {
		/* bounded wait for the DMA-complete IRQ's flag */
	}
	if (!adc_sync.done) {
		bridge_hw_adc_read_abort(adc_sync_done);
		if (!adc_sync.done) return BRIDGE_HW_ERR_IO; /* not a late completion: it timed out */
	}
	adc_sync.done = false;
	if (adc_sync.rv != BRIDGE_HW_OK) return adc_sync.rv;
	for (uint8_t i = 0u; i < adc_sync.n; ++i) {
		mv[i] = adc_sync.mv[i];
	}
	return BRIDGE_HW_OK;
}

int bridge_hw_adc_configure(uint8_t  channel,
                            uint16_t oversample_ratio,
                            uint16_t sample_cycles,
                            uint8_t  resolution_bits)
{
	if (channel >= ADC_CHANNEL_MAP_COUNT) return BRIDGE_HW_ERR_RANGE;

	/* Validate resolution BEFORE mutating any cache so a rejected field
	 * leaves the channel's format untouched.  All three fields are
	 * pure-cache here (like sample_cycles): the read / stream_begin
	 * paths program them into the converter inside their ADCON==0
	 * windows.  Deferring the register write also keeps configure from
	 * disturbing a converter that a sibling channel may be streaming
	 * on -- the new format takes effect on this channel's next read or
	 * stream_begin.
	 *
	 * Field semantics follow the wire contract (alp-sdk docs/gd32-bridge-
	 * protocol.md §3.9):
	 *   resolution_bits: 0 -> default (12).  6/8/10/12 map to the
	 *     hardware DRES field.  14/16 are not supported by the
	 *     hardware (DRES tops out at 12 bit) -> NOSUPPORT.  Any other width is invalid ->
	 *     INVAL.
	 *   oversample_ratio: 0/1 -> off; anything larger is floored to the
	 *     nearest power of two in 2..256 (adc_oversample_params, never
	 *     rejects -- matches the doc's "rounds down" wording).
	 * The stored resolution is normalised to 12 when 0 so the read
	 * path's full-scale lookup and register apply see a concrete
	 * width. */
	uint8_t  res_bits = (resolution_bits == 0u) ? ADC_RES_BITS_DEFAULT : resolution_bits;
	uint32_t res_reg;
	if (!adc_resolution_reg(res_bits, &res_reg)) {
		if (res_bits == 14u || res_bits == 16u) return BRIDGE_HW_ERR_NOTIMPL;
		return BRIDGE_HW_ERR_INVAL;
	}

	/* Sample cycles: 0 means "firmware default" (per the wire contract),
     * NOT the fastest window -- collapsing 240 -> 2 cycles on the
     * high-impedance divider inputs the default exists to serve would
     * leave the S/H cap unsettled and read systematically low (worse
     * still under oversampling).  Any non-zero value is a raw RSMP
     * register count (the sample_time parameter of
     * `adc_routine_channel_config`, ADCCK cycles with sample time = value + 2.5
     * cycles; not microseconds, not a rung selector) clamped into the
     * vendor's accepted 2..638 range. */
	uint16_t sc = (sample_cycles == 0u) ? ADC_DEFAULT_SAMPLE_CYCLES : sample_cycles;
	if (sc < 2u) sc = 2u;
	if (sc > 638u) sc = 638u;

	adc_sample_cycles_cache[channel]    = sc;
	adc_resolution_bits_cache[channel]  = res_bits;         /* 0 normalised to 12 above */
	adc_oversample_ratio_cache[channel] = oversample_ratio; /* apply-time floors to pow2 */
	return BRIDGE_HW_OK;
}
