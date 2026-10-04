/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 bridge HAL backend -- DMA-paced ADC streaming + the FAC/FFT
 * register-level DSP pump.  Split move-only from hal/bridge_hw_gd32.c
 * (fw v0.2.8); see hal/gd32/init.c for the backend-wide implementation
 * notes.  The DSP-chain POOL and bind-time validation moved out to
 * hal/gd32/adc_dsp_chain.c (#69/#70) -- see adc_dsp_chain.h.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bridge_hw.h"
#include "gd32g5x3.h"

#include "adc_dsp_chain.h"
#include "adc_stream2.h"
#include "bridge_board_config.h"
#include "bridge_critical.h"
#include "gd32_common.h"
#include "protocol.h" /* the wire-side stream constants asserted below */

/* The protocol layer validates BEGIN2 against its own copies of these. */
_Static_assert(BRIDGE_ADC_STREAM_RATE_MAX_HZ == GD32_BRIDGE_ADC_STREAM2_RATE_MAX_HZ,
               "BEGIN2's rate ceiling (protocol.h) must match the HAL's");
_Static_assert(BRIDGE_ADC_STREAM_RING_SAMPLES == GD32_BRIDGE_ADC_STREAM_RING_SAMPLES,
               "the host-visible ring depth (protocol.h) must match the HAL's");
_Static_assert(BRIDGE_ADC_STREAM_COUNT == GD32_BRIDGE_ADC_STREAM_COUNT,
               "stream count (protocol.h) must match the HAL's");

/* Stream slots; layout + sizing doc in gd32_common.h. */
adc_stream_state_t adc_streams[BRIDGE_ADC_STREAM_COUNT];

/* NVIC priority for the per-stream DMA "lap" ISR (full-transfer-
 * finish).  Below every transport ISR (SPI/CS = 1, I2C = 2 -- see
 * bridge_board_config.h): a lap tick fires once per ring period
 * (>= ~10 ms at the 100 kHz rate cap) and is pure bookkeeping, so it
 * must never delay the latency-sensitive link ISRs. */
#define ADC_STREAM_LAP_IRQ_PRIO    BRIDGE_ADC_STREAM_LAP_IRQ_PRIO
#define ADC_STREAM_LAP_IRQ_SUBPRIO 0u

/* UM Rev1.2 §8.4.7 permits writing the channel address/count registers
 * only after CHEN reads clear.  The SPL helpers are plain register writes,
 * so make that interlock explicit rather than assuming a preceding write
 * has already reached the DMA controller.  This path is command-driven,
 * not a sampling hot path; a bounded failure is therefore preferable to
 * reusing a possibly still-live channel configuration. */
#define ADC_STREAM_DMA_DISABLE_SPINS 64u

static bool adc_stream_dma_disable_confirm(uint32_t dma_periph, dma_channel_enum channel)
{
	dma_channel_disable(dma_periph, channel);
	for (uint32_t spin = 0u; spin < ADC_STREAM_DMA_DISABLE_SPINS; ++spin) {
		if ((DMA_CHCTL(dma_periph, channel) & DMA_CHXCTL_CHEN) == 0u) return true;
	}
	return false;
}

static uint32_t adc_stream_dmamux_channel(const adc_stream_state_t *s)
{
	return (s->dma_periph == DMA0) ? (uint32_t)s->dma_channel : (uint32_t)s->dma_channel + 7u;
}

/* ERRIF means this channel has stopped delivering a trustworthy ring.  The
 * latch is used by both its DMA IRQ and the higher-priority CS handler, which
 * polls it directly before interpreting stream state. */
static void adc_stream_latch_dma_error(uint8_t stream_id)
{
	adc_stream_state_t *s = &adc_streams[stream_id];
	if (dma_interrupt_flag_get(s->dma_periph, (dma_channel_enum)s->dma_channel, DMA_INT_FLAG_ERR) ==
	    RESET) {
		return;
	}
	/* ERRIFC only: a global clear would discard a concurrent FTF. */
	dma_interrupt_flag_clear(s->dma_periph, (dma_channel_enum)s->dma_channel, DMA_INT_FLAG_ERR);
	dma_interrupt_disable(
	    s->dma_periph, (dma_channel_enum)s->dma_channel, DMA_INT_FTF | DMA_INT_HTF | DMA_INT_ERR);
	dma_channel_disable(s->dma_periph, (dma_channel_enum)s->dma_channel);
	s->dma_error_count++;
}

/* A watermark boundary was crossed on stream `id` (half-transfer at W samples,
 * full-transfer at 2W of a 2W-deep ring).  Only a BEGIN2 stream with a
 * watermark whose data plane is the RAW ring raises ATTN events from here; a
 * DSP-bound stream raises them from the base-level pump when the PROCESSED
 * backlog reaches W (bridge_hw_dsp_pump). */
static void adc_stream_watermark_event(uint8_t id)
{
	const adc_stream_state_t *s = &adc_streams[id];
	if (s->v2 && s->watermark != 0u && !s->dsp_bound) bridge_hw_attn_event_set(id);
}

/* DMA full-transfer-finish "lap" ISRs -- one per stream (stream 0 ->
 * DMA0 CH0, stream 1 -> DMA1 CH0, fixed in stream_begin below).  The
 * circular channel raises FTF exactly once per ring reload, so
 * lap_count * RING_SAMPLES + the live write index is the TOTAL sample
 * count the DMA has ever deposited -- the writer half of the overrun
 * accounting in bridge_hw_adc_stream_read.  Strong definitions
 * override the vendor startup's weak Default_Handler aliases
 * (CMSIS/GD/GD32G5x3/Source/GCC/startup_gd32g5x3.S). */
void DMA0_Channel0_IRQHandler(void)
{
	if (dma_interrupt_flag_get(DMA0, DMA_CH0, DMA_INT_FLAG_FTF) != RESET) {
		dma_interrupt_flag_clear(DMA0, DMA_CH0, DMA_INT_FLAG_FTF);
		adc_streams[0].lap_count++;
		adc_stream_watermark_event(0u);
	}
	if (dma_interrupt_flag_get(DMA0, DMA_CH0, DMA_INT_FLAG_HTF) != RESET) {
		dma_interrupt_flag_clear(DMA0, DMA_CH0, DMA_INT_FLAG_HTF);
		adc_stream_watermark_event(0u);
	}
	adc_stream_latch_dma_error(0u);
}

void DMA1_Channel0_IRQHandler(void)
{
	if (dma_interrupt_flag_get(DMA1, DMA_CH0, DMA_INT_FLAG_FTF) != RESET) {
		dma_interrupt_flag_clear(DMA1, DMA_CH0, DMA_INT_FLAG_FTF);
		adc_streams[1].lap_count++;
		adc_stream_watermark_event(1u);
	}
	if (dma_interrupt_flag_get(DMA1, DMA_CH0, DMA_INT_FLAG_HTF) != RESET) {
		dma_interrupt_flag_clear(DMA1, DMA_CH0, DMA_INT_FLAG_HTF);
		adc_stream_watermark_event(1u);
	}
	adc_stream_latch_dma_error(1u);
}

/* TRIGSEL route target for an ADC peripheral's routine-group trigger. */
static trigsel_periph_enum adc_stream_routrg(uint32_t adc_periph)
{
	if (adc_periph == ADC1) return TRIGSEL_OUTPUT_ADC1_ROUTRG;
	if (adc_periph == ADC2) return TRIGSEL_OUTPUT_ADC2_ROUTRG;
	if (adc_periph == ADC3) return TRIGSEL_OUTPUT_ADC3_ROUTRG;
	return TRIGSEL_OUTPUT_ADC0_ROUTRG;
}

/* DMA write-cursor read.  The DMA channel counter counts DOWN from
 * the configured transfer length; converting to a write index uses
 * `ring_samples - remaining`.  Wraps naturally via the circular-mode
 * reload. */
static uint16_t adc_stream_write_index(const adc_stream_state_t *s)
{
	const uint32_t remaining =
	    dma_transfer_number_get(s->dma_periph, (dma_channel_enum)s->dma_channel);
	/* remaining == 0 (mid circular reload) is write index 0, never 1024: a
	 * 1024 stored into read_idx would index ring[1024], which aliases the
	 * read_idx member itself (gh#18 A22). */
	if (remaining == 0u || remaining > s->ring_depth) return 0u;
	return (uint16_t)(s->ring_depth - remaining);
}

/* Total samples the DMA has ever deposited, with gh#149's coalescing
 * recovery folded in.
 *
 * The raw arithmetic -- lap_count * RING_SAMPLES + write index -- is
 * exact ONLY while the lap ISR counts every FTF.  It can undercount:
 * the DMA's FTF latches once per circular reload, so a reload that
 * lands while its predecessor's FTF is still pending (or before the
 * pended prio-3 lap ISR gets to run) is counted at most once -- and
 * from THIS side of the NVIC there is no distinguishing "ISR pended,
 * will count" from "two reloads, one count".  The observable symptom
 * is a write index that REGRESSED (the counter reloaded top-down)
 * while lap_count stood still: that is one full ring the raw formula
 * silently drops, permanently skewing this consumer's backlog math so
 * the next "fresh" samples are one ring stale -- data corruption with
 * no error code, the failure mode gh#149 opened with.
 *
 * Recovery: each consumer tracks its own last-observed (laps, w) and
 * adds RING_SAMPLES to the total when it sees a regression with
 * lap_count unchanged.  The correction is per-sample (not persisted
 * into lap_count), so the lap ISR counting that same reload a moment
 * later cannot double-credit: the next sample sees lap_count moved
 * and needs no correction.  Two wraps between two samples of the same
 * consumer cannot be counted this way -- gh#265: this does NOT fall
 * back to the >= RING_SAMPLES overrun resync below.  If total_written
 * is short by one RING_SAMPLES, backlog = total_written - total_read
 * is short by the identical amount, so that branch cannot fire
 * either; the two errors are not independent.  What actually happens:
 * the shortfall self-heals the moment this consumer next observes
 * lap_count move, one ring period later -- until then it is served
 * one-ring-stale samples with BRIDGE_HW_OK, no BRIDGE_HW_ERR_BUSY.
 * That is bounded and self-healing, not silent corruption: for two
 * wraps to land between samples the prio-3 lap vector must be starved
 * for a whole ring period (>= ~10 ms at the 100 kHz cap), which no
 * bounded prio-1/2 work in this tree approaches (the longest is the
 * ROVF recovery's ~2 ms bounded recalibration spin).
 *
 * trk is the caller's own tracker (s->rd_pos for the prio-1 read
 * path, s->pump_pos for the base-level pump) -- never lap_count.  The
 * tracker type lives in gd32_common.h (adc_dma_pos_t). */

static uint32_t adc_stream_total_written(adc_stream_state_t *s, adc_dma_pos_t *trk)
{
	const uint32_t laps  = s->lap_count;
	const uint16_t w     = adc_stream_write_index(s);
	uint32_t       total = laps * s->ring_depth + (uint32_t)w;
	if (trk->valid && laps == trk->laps && w < trk->w) {
		total += s->ring_depth; /* one uncounted reload */
	}
	trk->laps  = laps;
	trk->w     = w;
	trk->valid = true;
	return total;
}

/* Shared body of the legacy BEGIN and BEGIN2 (v2 == true).  BEGIN2 adds, all
 * BEFORE any hardware is touched: a late-VREF-pending refusal (NOT_READY), a
 * core-clock check (IO), and the conversion-time check (RANGE) -- it never
 * reports a rate it cannot achieve -- and on success fills `info` with the
 * realised pace.  Everything else is the one proven bring-up sequence. */
static int adc_stream_begin_common(uint8_t                       stream_id,
                                   uint8_t                       channel,
                                   uint32_t                      sample_rate_hz,
                                   uint16_t                      watermark,
                                   bool                          v2,
                                   bridge_hw_adc_stream2_info_t *info)
{
	if (stream_id >= BRIDGE_ADC_STREAM_COUNT) return BRIDGE_HW_ERR_RANGE;
	if (channel >= ADC_CHANNEL_MAP_COUNT) return BRIDGE_HW_ERR_RANGE;
	if (sample_rate_hz == 0u) return BRIDGE_HW_ERR_INVAL;
	if (sample_rate_hz > BRIDGE_ADC_STREAM_RATE_MAX_HZ) return BRIDGE_HW_ERR_RANGE;
	if (!vref_ready_check()) {
		/* A late lock was just noted: the base-level re-measure will replace
		 * adc_vref_mv, so a BEGIN2 snapshot taken now would go stale.  Retry
		 * after the SysTick housekeeping period.  Otherwise the reference is
		 * simply dead -- fail loud. */
		return (v2 && vref_remeasure_pending_get()) ? BRIDGE_HW_ERR_NOT_READY : BRIDGE_HW_ERR_IO;
	}

	adc_stream_state_t *s = &adc_streams[stream_id];
	if (s->in_use) return BRIDGE_HW_ERR_INVAL; /* stream already running */

	const gd32_adc_ch_t *ch = &adc_channels_map[channel];

	/* One stream per ADC converter: both streams sharing a peripheral
     * would fight over routine rank 0 AND the TRIGSEL routine-trigger
     * route -- the second begin would silently re-pace and re-point
     * the first.  Refuse honestly instead. */
	for (uint8_t i = 0u; i < BRIDGE_ADC_STREAM_COUNT; ++i) {
		if (i != stream_id && adc_streams[i].in_use &&
		    adc_channels_map[adc_streams[i].channel].periph == ch->periph) {
			return BRIDGE_HW_ERR_INVAL;
		}
	}

	/* The realised pace and, for BEGIN2, the checks that need it. */
	uint32_t tick_hz, period_ticks;
	adc_stream2_pace(sample_rate_hz, &tick_hz, &period_ticks);
	const uint16_t ring_depth =
	    v2 ? adc_stream2_ring_depth(watermark) : (uint16_t)BRIDGE_ADC_STREAM_RING_SAMPLES;
	if (v2) {
		/* Every timing constant here assumes the 216 MHz core clock. */
		if (!bridge_core_clock_matches) return BRIDGE_HW_ERR_IO;
		if (!adc_stream2_conv_fits(adc_effective_ratio(channel),
		                           adc_sample_cycles_cache[channel],
		                           period_ticks,
		                           tick_hz)) {
			return BRIDGE_HW_ERR_RANGE;
		}
	}

	/* Stream 0 -> DMA0, stream 1 -> DMA1.  Channel 0 of each DMA
     * controller is the first free slot in the GD32G5x3 dma_channel
     * enum; bridge brings up no other DMA users today so collisions
     * are not a concern. */
	s->dma_periph  = (stream_id == 0u) ? DMA0 : DMA1;
	s->dma_channel = (uint8_t)DMA_CH0;
	s->pace_timer  = (stream_id == 0u) ? TIMER5 : TIMER6;

	/* The DMAMUX request routing dma_init() writes below lands on a
     * clock-gated register unless the mux clock is up.  The SPI
     * transport happens to enable it first at boot today -- own the
     * dependency here instead of relying on bring-up order (silicon
     * 2026-06-04 audit: an I2C-only build would stream zero samples). */
	bridge_rcu_periph_clock_enable(RCU_DMAMUX);
	bridge_rcu_periph_clock_enable((stream_id == 0u) ? RCU_DMA0 : RCU_DMA1);
	if (!adc_stream_dma_disable_confirm(s->dma_periph, (dma_channel_enum)s->dma_channel)) {
		return BRIDGE_HW_ERR_IO;
	}
	dma_deinit(s->dma_periph, (dma_channel_enum)s->dma_channel);

	dma_parameter_struct init;
	dma_struct_para_init(&init); /* all fields defined before the explicit set */
	init.periph_addr  = (uint32_t)(uintptr_t)&ADC_RDATA(ch->periph);
	init.memory_addr  = (uint32_t)(uintptr_t)s->ring;
	init.direction    = DMA_PERIPHERAL_TO_MEMORY;
	init.number       = ring_depth;
	init.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
	init.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
	init.periph_width = DMA_PERIPHERAL_WIDTH_16BIT;
	init.memory_width = DMA_MEMORY_WIDTH_16BIT;
	init.priority     = DMA_PRIORITY_MEDIUM;
	/* DMAMUX request: route the channel to this ADC instance.  Without
     * this the request id is left uninitialised and the channel triggers
     * on the wrong (or no) source. */
	init.request = (ch->periph == ADC1)   ? DMA_REQUEST_ADC1
	               : (ch->periph == ADC2) ? DMA_REQUEST_ADC2
	               : (ch->periph == ADC3) ? DMA_REQUEST_ADC3
	                                      : DMA_REQUEST_ADC0;
	dma_init(s->dma_periph, (dma_channel_enum)s->dma_channel, &init);

	/* Circular mode -- DMA reloads `number` after each cycle so the
     * channel keeps running without firmware re-arms.  Combined with
     * adc_dma_mode_enable below this produces a steady-state
     * peripheral-to-ring pipeline with no firmware in the hot path. */
	dma_circulation_enable(s->dma_periph, (dma_channel_enum)s->dma_channel);

	/* Reconfigure the converter for streaming with ADCON CLEAR, in
     * the vendor's proven order (Examples/ADC/ADC0_routine_channel_
     * with_DMA): mode + trigger + DMA controls all land BEFORE the
     * enable.  Programming CTL1 on an already-running converter is
     * exactly how the v0.2.3 stream silently produced zero samples.
     * Calibration IS redone below, after ADCON re-enables: an ADCON
     * toggle does NOT preserve the boot calibration from
     * the boot setup (UM Rev1.2 p.424: the factor is applied only
     * "until the next ADC power-off", and clearing ADCON IS that
     * power-off, p.447) -- and the recalibration is bounded
     * (adc_calibrate_bounded), so it is not the unbounded vendor spin
     * this comment used to worry about (#34). */
	/* Claim the shared converter for the reconfigure below (#133).  The
     * stream-vs-stream scan above says nothing about a single-shot
     * bridge_hw_adc_read in flight on the sibling bridge channel, and
     * that read holds this same flag for its whole convert loop -- so a
     * BEGIN that lands mid-read is told BUSY instead of re-pointing
     * routine rank 0 out from under it.  Long-term ownership stays with
     * the in_use publication; this flag only covers the window in which
     * the converter is being reprogrammed -- which now also spans the
     * bounded recalibration above, since that too runs on the shared
     * converter and must not race a sibling read. */
	if (!adc_periph_claim(ch->periph)) return BRIDGE_HW_ERR_BUSY;

	adc_disable(ch->periph);
	/* Apply the channel's cached resolution + oversample while the
	 * converter is disabled (DRES/OVSAMPCTL only latch with ADCON==0).
	 * In oversampling mode each pacing-timer trigger runs all `ratio`
	 * conversions before one DMA beat lands, so an over-asked rate
	 * degrades exactly as the un-oversampled path documents. */
	adc_apply_conv_format(ch->periph, channel);
	adc_routine_channel_config(ch->periph, 0u, ch->channel, adc_sample_cycles_cache[channel]);

	/* Each pacing-timer TRGO edge starts exactly ONE routine
     * conversion -- the honest realisation of `sample_rate_hz`.  No
     * continuous mode: the silicon ignores trigger edges that land
     * mid-conversion, so an over-asked rate degrades to the channel's
     * achievable rate instead of corrupting the ring. */
	adc_external_trigger_config(ch->periph, ADC_ROUTINE_CHANNEL, EXTERNAL_TRIGGER_RISING);

	/* THE v0.2.3 got==0 root cause: CTL1.DMA alone stops issuing
     * requests after one DMA run.  CTL1.DDM (request-after-last) keeps
     * the request line live so the circular channel refills forever --
     * the vendor reference enables BOTH, in this order. */
	adc_dma_request_after_last_enable(ch->periph);
	adc_dma_mode_enable(ch->periph);

	/* Clear any End-Of-Conversion left by a prior single-shot
     * bridge_hw_adc_read on this peripheral BEFORE the converter
     * re-enables -- a stale EOC otherwise fires one spurious DMA
     * beat the moment the request unmasks, depositing a phantom
     * zeroth sample and desynchronising the ring cursor.  ROVF gets
     * the same treatment: overflow detection is live the instant
     * ADCON sets (DMA is already enabled above), and a session-stale
     * ROVF left set would stall conversion before the first sample
     * (UM Rev1.2 17.4.12, p.431-432; #44). */
	adc_flag_clear(ch->periph, ADC_FLAG_EOC);
	adc_flag_clear(ch->periph, ADC_FLAG_ROVF);
	adc_enable(ch->periph);
	for (volatile uint32_t stab = 0u; stab < 4096u; ++stab) {
		/* tSTAB dwell after ADCON, same bound the boot setup uses */
	}
	/* Recalibrate after the ADCON toggle above -- see the disable/
     * enable comment at the top of this bracket (#34).  Bounded, cost
     * ~25 us (adc.c's bridge_hw_adc_read carries the full derivation);
     * a false return means the calibration FSM never finished, so
     * fail the begin rather than arm a stream on an unproven
     * converter -- the DMA channel + lap ISR are not armed yet at
     * this point, so there is no live stream state to unwind.
     *
     * DISCLOSURE (#34 review): this is one of three request-path call
     * sites (the others: bridge_hw_adc_read in adc.c, and
     * adc_stream_recover_rovf below) that now pay calibration's
     * ~200000-iteration wedged-FSM worst case on every invocation,
     * not just once at boot -- see adc.c's bridge_hw_adc_read for the
     * full disclosure and the decision not to shorten the bound. */
	if (!adc_calibrate_bounded(ch->periph)) {
		/* Release the converter claim before bailing (#133 x #80).
         * Neither change has this hazard alone -- #80 added this early
         * return, #133 added the claim above it, and the MERGE is what
         * puts a return inside the claimed window.  Leaking the claim
         * here would leave adc_periph_busy[] set forever: every later
         * bridge_hw_adc_read and stream_begin on this converter would
         * answer BRIDGE_HW_ERR_BUSY, reboot-only recovery, from a single
         * calibration failure. */
		adc_periph_release(ch->periph);
		return BRIDGE_HW_ERR_IO;
	}

	/* Reader-visible session fields are initialised BEFORE the DMA and
	 * pacing timer arm and before in_use publishes: protocol_dispatch runs
	 * in the transport ISRs and may preempt this call (gh#18 A21). */
	s->channel = channel;
	/* Snapshot the full-scale for the mv math so a mid-stream
	 * bridge_hw_adc_configure (which only rewrites the cache) can't
	 * change the divisor under a running stream -- the converter keeps
	 * the format this begin applied until stream_end. */
	s->full_scale   = adc_full_scale_for_bits(adc_resolution_bits_cache[channel]);
	s->ring_depth   = ring_depth;
	s->watermark    = watermark;
	s->v2           = v2;
	s->read2_d      = 0u; /* the delivered index starts at 0 at BEGIN2 */
	s->read_idx     = 0u;
	s->total_read   = 0u; /* lap_count zeroed above, pre-arm */
	s->dsp_chain_id = 0u;
	s->dsp_bound    = false;
	s->proc_gap     = false;
	s->dsp_cfg_bad  = false; /* gh#35 sticky flags: clean slate per session */
	s->dsp_sat      = false;

	/* Arm the lap counter BEFORE the channel starts: clear any stale
	 * full-transfer flag from a prior session on this controller, then
	 * enable the FTF interrupt + its NVIC line so EVERY ring reload is
	 * counted -- the overrun detection in stream_read is exact
	 * total-written-vs-read accounting, not a heuristic. */
	s->lap_count       = 0u;
	s->dma_error_count = 0u;
	/* gh#149: reset both consumers' position trackers so a new session
	 * starts from a clean baseline -- a stale tracker from a prior
	 * session would compare against a garbage (laps, w) and could add a
	 * phantom lap on the first read. */
	s->rd_pos.valid   = false;
	s->pump_pos.valid = false;
	dma_flag_clear(s->dma_periph,
	               (dma_channel_enum)s->dma_channel,
	               DMA_FLAG_FTF | DMA_FLAG_HTF | DMA_FLAG_ERR);
	/* HTF fires at W samples of the 2W ring, FTF at 2W: the two watermark
	 * boundaries.  A legacy / no-watermark stream keeps FTF (the lap counter)
	 * and ERR only. */
	dma_interrupt_enable(s->dma_periph,
	                     (dma_channel_enum)s->dma_channel,
	                     DMA_INT_FTF | DMA_INT_ERR | (watermark != 0u ? DMA_INT_HTF : 0u));
	nvic_irq_enable((s->dma_periph == DMA0) ? DMA0_Channel0_IRQn : DMA1_Channel0_IRQn,
	                ADC_STREAM_LAP_IRQ_PRIO,
	                ADC_STREAM_LAP_IRQ_SUBPRIO);

	dma_channel_enable(s->dma_periph, (dma_channel_enum)s->dma_channel);

	/* Route the pacing timer's update-event TRGO0 to this converter's
     * routine trigger, then run the timer at the requested rate.  Two
     * prescaler regimes keep the 16-bit period in range: a 1 MHz tick
     * covers 16 Hz..100 kHz exactly where it matters; below 16 Hz a
     * 10 kHz tick stretches to 1 Hz.  Division truncates -- worst-case
     * quantisation is one tick (documented in the protocol spec). */
	bridge_rcu_periph_clock_enable(RCU_TRIGSEL);
	trigsel_init(adc_stream_routrg(ch->periph),
	             (stream_id == 0u) ? TRIGSEL_INPUT_TIMER5_TRGO0 : TRIGSEL_INPUT_TIMER6_TRGO0);

	bridge_rcu_periph_clock_enable((stream_id == 0u) ? RCU_TIMER5 : RCU_TIMER6);
	timer_deinit(s->pace_timer);
	/* adc_stream2_pace(): 1 MHz tick (period 10..62500) at >= 16 Hz, else a
	 * 10 kHz tick (period 667..10000). */
	const uint32_t         psc = (BRIDGE_ADC_PACE_CLK_HZ / tick_hz) - 1u;
	timer_parameter_struct tp;
	timer_struct_para_init(&tp);
	tp.prescaler = (uint16_t)psc;
	tp.period    = period_ticks - 1u;
	timer_init(s->pace_timer, &tp);
	timer_master_output0_trigger_source_select(s->pace_timer, TIMER_TRI_OUT0_SRC_UPDATE);
	timer_enable(s->pace_timer);

	if (info != NULL) {
		info->tick_hz      = tick_hz;
		info->period_ticks = period_ticks;
		info->full_scale   = s->full_scale;
		info->vref_mv      = adc_vref_mv;
		info->watermark    = watermark;
		info->ring_depth   = ring_depth;
		info->flags        = adc_vref_is_measured(adc_vrefint_code)
		                         ? (uint8_t)BRIDGE_HW_ADC_STREAM2_FLAG_VREF_MEASURED
		                         : 0u;
	}

	/* Publish last: every reader-visible field was set before the DMA
	 * and pacing timer were armed (gh#18 A21). */
	s->in_use = true;
	/* Reconfigure done and in_use published: hand the converter's
     * short-term claim back (#133).  From here the in_use scan in
     * bridge_hw_adc_read is what keeps single-shot reads off this
     * converter, for as long as the stream runs. */
	adc_periph_release(ch->periph);
	return BRIDGE_HW_OK;
}

int bridge_hw_adc_stream_begin(uint8_t stream_id, uint8_t channel, uint32_t sample_rate_hz)
{
	return adc_stream_begin_common(stream_id, channel, sample_rate_hz, 0u, false, NULL);
}

int bridge_hw_adc_stream_begin2(uint8_t                       stream_id,
                                uint8_t                       channel,
                                uint32_t                      sample_rate_hz,
                                uint16_t                      watermark,
                                bridge_hw_adc_stream2_info_t *info)
{
	if (info == NULL) return BRIDGE_HW_ERR_INVAL;
	return adc_stream_begin_common(stream_id, channel, sample_rate_hz, watermark, true, info);
}

bool bridge_hw_adc_stream2_supported(void)
{
	return true;
}

/* Recover the ADC from a routine-data overflow (#44) -- the 9-step
 * sequence UM Rev1.2 17.4.12 (p.431-432) documents.  Steps 2 and 7
 * toggle ADCON, which invalidates the calibration factor the same way
 * stream_begin's disable/enable does (#34): recalibrate before
 * returning so the stream resumes on a proven converter, not merely
 * an unstalled one.  DDM (request-after-last) and the external-
 * trigger routine config are untouched by this sequence and the
 * pacing timer never stopped, so step 9 ("start conversion") needs no
 * explicit call here -- the next TRGO edge resumes conversion once
 * ADCON is back.  Returns false only if the recalibration's bounded
 * spin never completes (the converter itself stayed wedged).
 *
 * DISCLOSURE (#34 review): this is the third of three request-path
 * call sites that now pay adc_calibrate_bounded's ~200000-iteration
 * wedged-FSM worst case on every invocation rather than once at boot
 * -- see adc.c's bridge_hw_adc_read for the full disclosure and the
 * decision not to shorten the bound.
 *
 * The DMA full-transfer-finish (FTF) interrupt flag is cleared as
 * part of step 3's "reinit DMA module": ROVF and a ring-wrap FTF can
 * land in the same window (this handler runs at CS-EXTI priority 1,
 * which blocks the priority-3 lap ISR -- DMA0/1_Channel0_IRQHandler
 * above -- from running until this function returns), and a FTF left
 * pending here fires the instant this handler returns, bumping
 * lap_count against a total_read the caller is about to re-anchor to
 * "0 new samples since recovery" -- the next poll would then
 * misreport a full-ring loss it didn't actually see (the #18 phantom-
 * loss class).  UM Rev1.2 §8.4.8 (p.295): the flag lives in DMA_INTF,
 * cleared via the dedicated bit in DMA_INTC (FTFIFC) -- disabling
 * CHEN does not clear it, so it needs its own clear here. */
static bool adc_stream_recover_rovf(adc_stream_state_t *s, const gd32_adc_ch_t *ch)
{
	adc_dma_mode_disable(ch->periph); /* 1. Clear DMA bit of ADC_CTL1. */
	adc_disable(ch->periph);          /* 2. Clear ADCON bit of ADC_CTL1. */

	/* 3. Clear CHEN bit of DMA_CHxCTL, reinit the DMA module.  The
	 * count register is reloaded to the full ring length explicitly:
	 * an overflow almost certainly caught the channel mid-ring rather
	 * than exactly at a circular-reload boundary.  A stale FTF (see
	 * the function comment above) is cleared here too, alongside the
	 * rest of the reinit, before the channel comes back up. */
	if (!adc_stream_dma_disable_confirm(s->dma_periph, (dma_channel_enum)s->dma_channel)) {
		return false;
	}
	dma_transfer_number_config(s->dma_periph, (dma_channel_enum)s->dma_channel, s->ring_depth);
	dma_interrupt_flag_clear(s->dma_periph, (dma_channel_enum)s->dma_channel, DMA_INT_FLAG_FTF);
	dma_interrupt_flag_clear(s->dma_periph, (dma_channel_enum)s->dma_channel, DMA_INT_FLAG_HTF);

	adc_flag_clear(ch->periph, ADC_FLAG_ROVF); /* 4. Clear ROVF bit of ADC_STAT. */
	dma_channel_enable(s->dma_periph, (dma_channel_enum)s->dma_channel); /* 5. Set CHEN. */
	adc_dma_mode_enable(ch->periph); /* 6. Set DMA bit of ADC_CTL1. */
	adc_enable(ch->periph);          /* 7. Set ADCON bit of ADC_CTL1. */
	for (volatile uint32_t stab = 0u; stab < 4096u; ++stab) {
		/* 8. Wait T(setup) -- same bound stream_begin/the boot setup use. */
	}
	return adc_calibrate_bounded(ch->periph); /* ADCON edge above invalidated calibration. */
}

/* ROVF recovery plus the cursor re-anchor both read paths need.  Re-anchors
 * through the SAME corrected total the read path uses (gh#149): the recovery
 * 9-step can step the DMA, so both position trackers are re-based against the
 * raw post-recovery position -- valid=false makes the next sample a clean
 * baseline rather than a regression.  Returns false only if the bounded
 * recalibration never completed. */
static bool adc_stream_rovf_recover_and_reanchor(adc_stream_state_t *s, const gd32_adc_ch_t *ch)
{
	const bool     recal_ok = adc_stream_recover_rovf(s, ch);
	const uint16_t w        = adc_stream_write_index(s);
	s->rd_pos.valid         = false;
	s->pump_pos.valid       = false;
	s->read_idx             = (uint16_t)(w % s->ring_depth);
	s->total_read           = s->lap_count * s->ring_depth + (uint32_t)w;
	s->pump_raw_read        = s->total_read;
	return recal_ok;
}

int bridge_hw_adc_stream_read(uint8_t   stream_id,
                              uint8_t   max_samples,
                              uint8_t  *got_samples,
                              uint16_t *mv)
{
	if (got_samples == 0) return BRIDGE_HW_ERR_INVAL;
	*got_samples = 0u;
	if (mv == 0) return BRIDGE_HW_ERR_INVAL;
	if (stream_id >= BRIDGE_ADC_STREAM_COUNT) return BRIDGE_HW_ERR_RANGE;

	adc_stream_state_t *s = &adc_streams[stream_id];
	if (!s->in_use) return BRIDGE_HW_ERR_INVAL;
	/* A stream started with BEGIN2 answers only READ2. */
	if (s->v2) return BRIDGE_HW_ERR_INVAL;
	adc_stream_latch_dma_error(stream_id);
	if (s->dma_error_count != 0u) return BRIDGE_HW_ERR_IO;

	/* ROVF (routine-data overflow) recovery (#44) -- checked before
	 * EITHER data plane below, raw or DSP-filtered: both draw from
	 * this stream's DMA ring, and UM Rev1.2 17.4.12 (p.431-432) is
	 * explicit that "[t]he ADC conversion will be stalled until the
	 * ROVF bit is cleared" -- unrecovered, the write index freezes and
	 * every subsequent poll answers STATUS_OK with zero samples,
	 * forever, on both planes. */
	const gd32_adc_ch_t *ch = &adc_channels_map[s->channel];
	if (SET == adc_flag_get(ch->periph, ADC_FLAG_ROVF)) {
		const bool recal_ok = adc_stream_rovf_recover_and_reanchor(s, ch);
		/* Same wire contract as the ring-overrun branch below
		 * (alp-sdk docs/gd32-bridge-protocol.md §3.10): STATUS_BUSY, "poll
		 * faster".  A failed recalibration is the harder failure --
		 * report IO so the host doesn't keep polling a converter left
		 * in an unproven state. */
		return recal_ok ? BRIDGE_HW_ERR_BUSY : BRIDGE_HW_ERR_IO;
	}

	/* DSP data plane (#496): a bound FIR/IIR chain means the host reads
	 * FILTERED samples the base-level pump produced in proc_ring -- NOT
	 * the raw DMA ring.  A bound FFT chain has no stream data plane;
	 * the spectrum is pulled via CMD_ADC_SPECTRUM_READ, so a plain
	 * STREAM_READ answers NOSUPPORT (never silently raw).  proc_write
	 * is produced at base level (volatile); snapshot once and use the
	 * same exact-difference backlog accounting as the raw path. */
	if (s->dsp_bound) {
		if (s->dsp_terminal == 3u) return BRIDGE_HW_ERR_NOTIMPL; /* FFT */

		/* gh#35 sticky fault surfacing: a config refusal (coefficients
		 * out of the FAC's realisable range) answers RANGE; a
		 * saturated FAC (STEF/GSTEF, see the pump) answers IO.  In
		 * both cases the stream is never again reported as
		 * STATUS_OK-serving-clean-data until stream_end resets the
		 * flags. */
		if (s->dsp_cfg_bad) return BRIDGE_HW_ERR_RANGE;
		if (s->dsp_sat) return BRIDGE_HW_ERR_IO;

		if (s->proc_gap) {
			s->proc_gap  = false;
			s->proc_read = s->proc_write;
			return BRIDGE_HW_ERR_BUSY; /* pump resynced past a full ring */
		}
		const uint32_t pw       = s->proc_write;
		const int32_t  pbacklog = (int32_t)(pw - s->proc_read);
		if (pbacklog <= 0) return BRIDGE_HW_OK; /* pump hasn't produced yet */
		if ((uint32_t)pbacklog >= BRIDGE_ADC_STREAM_RING_SAMPLES) {
			s->proc_read = pw; /* pump lapped the reader -> resync, report loss */
			return BRIDGE_HW_ERR_BUSY;
		}
		const uint16_t pavail = (uint16_t)pbacklog;
		const uint16_t emit   = (pavail < max_samples) ? pavail : max_samples;
		for (uint16_t i = 0u; i < emit; ++i) {
			uint32_t code = s->proc_ring[s->proc_read % BRIDGE_ADC_STREAM_RING_SAMPLES];
			if (code > s->full_scale) code = s->full_scale;
			mv[i] = (uint16_t)((code * (uint32_t)adc_vref_mv) / s->full_scale);
			s->proc_read++;
		}
		*got_samples = (uint8_t)emit;
		return BRIDGE_HW_OK;
	}

	/* Drain as many fresh samples as the host asked for, capped by
	 * what the DMA has actually deposited since the last read.
	 * Overrun accounting is EXACT total-written-vs-read: the writer's
	 * lifetime deposit count is lap_count full rings (the FTF lap ISR
	 * above) plus the live write index, with gh#149's coalescing
	 * recovery folded in by adc_stream_total_written(); the reader's
	 * is total_read.  A backlog beyond one ring means the writer
	 * lapped the reader and overwrote samples the host never saw --
	 * mixed-lap data that must not be delivered as a contiguous
	 * stream.
	 *
	 * Snapshot lap_count BEFORE the write index: this read runs in
	 * the CS-EXTI handler (prio 1), which outprioritises the lap ISR
	 * (prio 3), so a reload landing mid-read leaves lap_count
	 * momentarily one short while w has already wrapped small.  The
	 * regression correction in adc_stream_total_written() absorbs
	 * exactly that snapshot (and the genuinely coalesced lap the ISR
	 * will never count), so the combined path can only ever
	 * UNDERcount by a lap it has already corrected once -- never a
	 * false overrun.  Unsigned uint32 wrap of the lifetime totals is
	 * harmless: the difference below stays small and modular
	 * arithmetic keeps it exact. */
	const uint32_t total_written = adc_stream_total_written(s, &s->rd_pos);
	const uint16_t w             = adc_stream_write_index(s);
	const int32_t  backlog       = (int32_t)(total_written - s->total_read);
	if (backlog <= 0) return BRIDGE_HW_OK; /* empty ring (or transient undercount) */

	if ((uint32_t)backlog >= s->ring_depth) {
		/* Lapped (or exactly full, where the oldest unread slot is the
	     * DMA's next landing zone -- reading it races the in-flight
	     * beat).  Drop the corrupt backlog and resynchronise the
	     * cursor to the live write position so the NEXT read returns
	     * fresh, gap-free samples; answer BUSY so the host learns
	     * samples were lost (alp-sdk docs/gd32-bridge-protocol.md §3.10: ring
	     * overrun -> STATUS_BUSY, "poll faster"). */
		s->read_idx   = (uint16_t)(w % s->ring_depth);
		s->total_read = total_written;
		return BRIDGE_HW_ERR_BUSY;
	}
	const uint16_t avail = (uint16_t)backlog;

	uint16_t to_emit = (avail < max_samples) ? avail : max_samples;
	for (uint16_t i = 0u; i < to_emit; ++i) {
		uint32_t code = s->ring[s->read_idx];
		if (code > s->full_scale) code = s->full_scale;
		mv[i]       = (uint16_t)((code * (uint32_t)adc_vref_mv) / s->full_scale);
		s->read_idx = (uint16_t)((s->read_idx + 1u) % s->ring_depth);
	}
	s->total_read += to_emit;
	*got_samples = (uint8_t)to_emit;
	return BRIDGE_HW_OK;
}

/* =====================================================================
 * v0.15 READ2 -- lossless-accounting read of a BEGIN2 stream.
 *
 * The planning arithmetic (overrun skip, GUARD band, delivered index D,
 * the discontinuity sentinel) is adc_read2_plan() in adc_stream2.c; this is
 * its hardware half: pick the data plane (raw DMA ring or the DSP-processed
 * ring), apply the plan to the cursors, copy the codes straight into the
 * caller's reply bytes, and keep the ATTN event bit honest.  Overrun is never
 * BUSY: the oldest samples are skipped and counted in `dropped`.
 * ===================================================================== */
int bridge_hw_adc_stream_read2(uint8_t   stream_id,
                               uint8_t   max_samples,
                               uint32_t *first_index,
                               uint32_t *dropped,
                               uint8_t  *got,
                               uint8_t  *codes_le)
{
	if (first_index == 0 || dropped == 0 || got == 0 || codes_le == 0) return BRIDGE_HW_ERR_INVAL;
	*first_index = 0u;
	*dropped     = 0u;
	*got         = 0u;
	if (stream_id >= BRIDGE_ADC_STREAM_COUNT) return BRIDGE_HW_ERR_RANGE;

	adc_stream_state_t *s = &adc_streams[stream_id];
	/* Not running, or started with the legacy BEGIN (which answers only
	 * STREAM_READ). */
	if (!s->in_use || !s->v2) return BRIDGE_HW_ERR_INVAL;
	adc_stream_latch_dma_error(stream_id);
	if (s->dma_error_count != 0u) return BRIDGE_HW_ERR_IO;

	adc_read2_plan_t plan;

	/* ROVF recovery (#44): the stalled conversion is restarted and the
	 * cursors re-anchored, but the samples lost in between are unknowable --
	 * the discontinuity sentinel, unless the recalibration itself failed. */
	const gd32_adc_ch_t *ch = &adc_channels_map[s->channel];
	if (SET == adc_flag_get(ch->periph, ADC_FLAG_ROVF)) {
		const bool recal_ok = adc_stream_rovf_recover_and_reanchor(s, ch);
		bridge_hw_attn_event_clear(stream_id);
		if (!recal_ok) return BRIDGE_HW_ERR_IO;
		adc_read2_plan(0u, s->ring_depth, max_samples, s->read2_d, true, &plan);
		*first_index = plan.first_index;
		*dropped     = plan.dropped;
		return BRIDGE_HW_OK;
	}

	uint32_t remaining;
	if (s->dsp_bound) {
		/* Filtered data plane: processed-ring codes in the same code space.
		 * An FFT chain has no stream data plane (spectrum is its own
		 * opcode); the sticky DSP faults keep their 0.14 meaning. */
		if (s->dsp_terminal == 3u) return BRIDGE_HW_ERR_NOTIMPL;
		if (s->dsp_cfg_bad) return BRIDGE_HW_ERR_RANGE;
		if (s->dsp_sat) return BRIDGE_HW_ERR_IO;

		const bool gap = s->proc_gap;
		if (gap) {
			s->proc_gap  = false; /* the pump resynced past a full ring */
			s->proc_read = s->proc_write;
		}
		const uint32_t backlog = (uint32_t)(s->proc_write - s->proc_read);
		adc_read2_plan(
		    backlog, (uint16_t)BRIDGE_ADC_STREAM_RING_SAMPLES, max_samples, s->read2_d, gap, &plan);
		s->proc_read += plan.skip;
		for (uint32_t i = 0u; i < plan.got; ++i) {
			uint32_t code = s->proc_ring[s->proc_read % BRIDGE_ADC_STREAM_RING_SAMPLES];
			if (code > s->full_scale) code = s->full_scale;
			codes_le[2u * i]      = (uint8_t)(code & 0xFFu);
			codes_le[2u * i + 1u] = (uint8_t)((code >> 8) & 0xFFu);
			s->proc_read++;
		}
	} else {
		/* Raw DMA ring: the same exact total-written-vs-read accounting the
		 * legacy read uses (gh#149 coalescing recovery included), now against
		 * this stream's own ring depth. */
		const uint32_t total_written = adc_stream_total_written(s, &s->rd_pos);
		const uint16_t w             = adc_stream_write_index(s);
		const uint32_t backlog       = total_written - s->total_read;
		adc_read2_plan(backlog, s->ring_depth, max_samples, s->read2_d, false, &plan);
		if (plan.sentinel) {
			/* Cursors disagree beyond recovery: resync to the live position. */
			s->read_idx   = (uint16_t)(w % s->ring_depth);
			s->total_read = total_written;
		} else {
			uint32_t idx = ((uint32_t)s->read_idx + (plan.skip % s->ring_depth)) % s->ring_depth;
			for (uint32_t i = 0u; i < plan.got; ++i) {
				uint32_t code = s->ring[idx];
				if (code > s->full_scale) code = s->full_scale;
				codes_le[2u * i]      = (uint8_t)(code & 0xFFu);
				codes_le[2u * i + 1u] = (uint8_t)((code >> 8) & 0xFFu);
				idx                   = (idx + 1u) % s->ring_depth;
			}
			s->read_idx = (uint16_t)idx;
			s->total_read += plan.skip + plan.got;
		}
	}

	remaining    = plan.remaining;
	s->read2_d   = plan.next_d;
	*first_index = plan.first_index;
	*dropped     = plan.dropped;
	*got         = (uint8_t)plan.got;

	/* ATTN event bookkeeping: this read consumed the event; it stays raised
	 * when a watermark's worth of backlog is still waiting. */
	bridge_hw_attn_event_clear(stream_id);
	if (!plan.sentinel && s->watermark != 0u && remaining >= s->watermark) {
		bridge_hw_attn_event_set(stream_id);
	}
	return BRIDGE_HW_OK;
}

/* adc_dsp_chain_release + adc_dsp_filter_stream_busy now live in
 * adc_dsp_chain.c (#69/#70 host-testability split) and are declared
 * in adc_dsp_chain.h, included above.
 *
 * DSP dispatch helpers still defined in the #496 pump section at end
 * of this file but referenced earlier by stream_end. */
void        adc_dsp_fac_release(uint8_t stream_id);
void        adc_dsp_fft_release(uint8_t stream_id);
static void adc_dsp_pump_fft(uint8_t sid);

int bridge_hw_adc_stream_end(uint8_t stream_id)
{
	if (stream_id >= BRIDGE_ADC_STREAM_COUNT) return BRIDGE_HW_ERR_RANGE;
	adc_stream_state_t *s = &adc_streams[stream_id];
	if (!s->in_use) return BRIDGE_HW_OK; /* idempotent */

	/* Stop the trigger SOURCE first (pacing timer), then disarm the
     * ADC's DMA request generation, then the DMA channel -- the other
     * order can leave one in-flight transfer landing after the
     * channel is disabled. */
	const gd32_adc_ch_t *ch = &adc_channels_map[s->channel];
	timer_disable(s->pace_timer);
	timer_deinit(s->pace_timer);
	adc_dma_request_after_last_disable(ch->periph);
	adc_dma_mode_disable(ch->periph);
	if (!adc_stream_dma_disable_confirm(s->dma_periph, (dma_channel_enum)s->dma_channel)) {
		return BRIDGE_HW_ERR_IO;
	}
	/* MUXID zero is the DMAMUX idle state.  Releasing it before clearing
	 * in_use prevents a later stream from selecting the same ADC request on
	 * the other controller's multiplexer channel (UM Rev1.2 §9.4.2). */
	DMAMUX_RM_CHXCFG(adc_stream_dmamux_channel(s)) &= ~DMAMUX_RM_CHXCFG_MUXID;

	/* Stand the lap counter down with the channel: mask the FTF
     * interrupt + NVIC line and clear a possibly-pending flag so a
     * later single-shot user of this DMA controller can't inherit a
     * stale lap tick. */
	dma_interrupt_disable(
	    s->dma_periph, (dma_channel_enum)s->dma_channel, DMA_INT_FTF | DMA_INT_HTF | DMA_INT_ERR);
	nvic_irq_disable((s->dma_periph == DMA0) ? DMA0_Channel0_IRQn : DMA1_Channel0_IRQn);
	dma_flag_clear(s->dma_periph,
	               (dma_channel_enum)s->dma_channel,
	               DMA_FLAG_FTF | DMA_FLAG_HTF | DMA_FLAG_ERR);

	/* A trigger edge may have started a conversion just before the
     * timer stopped.  Dwell past one conversion time (~6.3 us healthy;
     * the spin below is comfortably longer) so it lands, then clear
     * EOC unconditionally -- whether the last EOC went to the DMA or
     * is still latched, the converter must idle CLEAN.  A leftover
     * conversion/EOC straddling into the next single-shot read on the
     * same peripheral is what started the 2026-06-04 link-rot chain. */
	for (volatile uint32_t settle = 0u; settle < 8192u; ++settle) {
		/* fixed dwell, ~tens of microseconds */
	}

	/* Full single-shot restore: reconfigure + recalibrate (calibration
	 * BOUNDED -- this runs in the CS-EXTI handler).  This deliberately
	 * does not reset an ADC or reconfigure a shared clock domain, because
	 * ADC0/1/2 may have a sibling stream running.  It
     * puts EXTERNAL_TRIGGER_DISABLE, routine length 1 and a fresh
     * calibration back so a following bridge_hw_adc_read sees the
	 * exact converter state the boot setup promised it -- the same
     * self-heal shape the read path's timeout branch uses.  The stream
     * state clears regardless of the restore verdict (the stream IS
     * over); a calibration that never completed reports IO so the host
     * knows the converter came back in an unproven state. */
	const bool restored = adc_periph_restore(ch->periph);

	/* Release the DSP chain bound to this stream back to the pool.
     * chain_open is the ONLY allocator (sets in_use=true) and nothing
     * else clears it, so without this the 4-slot pool leaks one chain
     * per bind->end cycle -- after BRIDGE_DSP_MAX_CHAINS cycles
     * chain_open returns NOSUPPORT forever until a reboot (#496).
     * dsp_chain_id is only meaningful while dsp_bound, so gate on it. */
	if (s->dsp_bound) {
		adc_dsp_fac_release(stream_id); /* free the FAC if this stream owned it */
		adc_dsp_fft_release(stream_id); /* free the FFT if this stream owned it */
		adc_dsp_chain_release(s->dsp_chain_id);
	}

	s->in_use      = false;
	s->dsp_bound   = false;
	s->dsp_cfg_bad = false; /* gh#35: sticky flags live exactly one session */
	s->dsp_sat     = false;
	s->v2          = false;
	s->watermark   = 0u;
	bridge_hw_attn_event_clear(stream_id); /* the stream's pending watermark event dies with it */
	return restored ? BRIDGE_HW_OK : BRIDGE_HW_ERR_IO;
}

/* v0.5 (§2B wave-2) chunked DSP-chain upload -- the pool, chain_open,
 * stage_push, adc_dsp_stage_blob_valid, chain_release and chain_bind
 * (plus the shared adc_dsp_chain_p1_capable() capability predicate and
 * the FAC/FFT busy checks it and chain_bind both use) now live in
 * adc_dsp_chain.c (#69/#70 host-testability split -- see that file's
 * header comment).  adc_dsp_chain.h, included above, is this file's
 * only remaining dependency on that pool: the pump-side config
 * functions below index `adc_dsp_chains[]` directly to decode a bound
 * chain's stage blobs before programming the FAC/FFT registers. */

/* =====================================================================
 * #496 FAC FIR/IIR runtime dispatch -- the filtered data plane.
 *
 * A bound FIR/IIR chain routes the raw ADC stream through the GD32 FAC
 * hardware filter (first-lit 2026-07-13: coeffs in X1, DEEP X0 input
 * buffer, clip-enabled, batch or streaming).  The FILTER runs in the
 * base-level pump (bridge_hw_dsp_pump, from the main WFI loop) -- NEVER
 * in stream_read (the CS-EXTI transport handler, prio 1) where even a
 * modest FIR would add link latency (the 2026-06-04 link-rot mode).
 *
 * There is ONE FAC block, so ONE filter stream may be bound at a time;
 * chain_bind rejects a second filter chain with NOSUPPORT
 * (adc_dsp_chain.c: adc_dsp_filter_stream_busy()).
 * ===================================================================== */

/* The FAC/FFT owner byte is shared by the base-level DSP pump and the
 * ISR-side stream teardown. Negative per-stream values are deliberately
 * NOT readable owners: they let release revoke a long configuration or FFT
 * publication while spectrum/data readers treat the block as unavailable
 * (#184/#185/#187). Transitions to/from an active owner are liveness-checked
 * in short PRIMASK critical sections; long hardware work stays interruptible. */
#define ADC_DSP_OWNER_NONE ((int8_t)-1)

/* gh#271: this transitional token is a function of stream_id ALONE --
 * it carries no per-session generation, so a session that reused
 * stream_id N would compute the identical token an earlier session on
 * the same stream_id N once held.  That collision is unreachable
 * today ONLY because the token is never live across two overlapping
 * sessions: the only writers of a transitional token are
 * adc_dsp_owner_claim_config(), the FIR pump claim and the FFT publish,
 * and each writes it only while *owner is NONE or already this stream's
 * id (claim_config requires *owner == ADC_DSP_OWNER_NONE first),
 * and every path that ends a session (adc_dsp_fac_release /
 * adc_dsp_fft_release, both driven from stream_end) restores
 * ADC_DSP_OWNER_NONE before a replacement stream_begin on the same ID
 * can run -- stream_begin/stream_end themselves are serialised by the
 * single-threaded protocol dispatch that calls them.  If a second
 * writer of this token is ever introduced, this invariant -- and the
 * _Static_assert below it -- must move with it. */
_Static_assert(BRIDGE_ADC_STREAM_COUNT <= 126u,
               "adc_dsp_owner_transitional's -2-stream_id must stay representable in int8_t "
               "without wrapping onto ADC_DSP_OWNER_NONE (-1) at stream_id 255");

static int8_t adc_dsp_owner_transitional(uint8_t stream_id)
{
	return (int8_t)(-2 - (int8_t)stream_id);
}

static bool adc_dsp_owner_live_locked(const volatile int8_t    *owner,
                                      int8_t                    expected_owner,
                                      const adc_stream_state_t *stream,
                                      bool                      fft_terminal)
{
	const bool terminal_matches =
	    fft_terminal ? (stream->dsp_terminal == 3u) : (stream->dsp_terminal != 3u);
	return *owner == expected_owner && stream->in_use && stream->dsp_bound && terminal_matches;
}

static bool adc_dsp_owner_claim_config(volatile int8_t          *owner,
                                       uint8_t                   stream_id,
                                       const adc_stream_state_t *stream,
                                       bool                      fft_terminal)
{
	const uint32_t irq_state = bridge_irq_lock();
	bool           claimed   = false;
	if (*owner == ADC_DSP_OWNER_NONE &&
	    adc_dsp_owner_live_locked(owner, ADC_DSP_OWNER_NONE, stream, fft_terminal)) {
		*owner  = adc_dsp_owner_transitional(stream_id);
		claimed = true;
	}
	bridge_irq_unlock(irq_state);
	return claimed;
}

static bool adc_dsp_owner_commit_config(volatile int8_t          *owner,
                                        uint8_t                   stream_id,
                                        const adc_stream_state_t *stream,
                                        bool                      fft_terminal)
{
	const int8_t   transitional = adc_dsp_owner_transitional(stream_id);
	const uint32_t irq_state    = bridge_irq_lock();
	const bool     live = adc_dsp_owner_live_locked(owner, transitional, stream, fft_terminal);
	if (live) {
		*owner = (int8_t)stream_id;
	} else if (*owner == transitional) {
		/* END may have landed before the configuring claim was visible,
		 * so release could not clear it. Do not leave a negative claim
		 * wedged after the post-config liveness check fails. */
		*owner = ADC_DSP_OWNER_NONE;
	}
	bridge_irq_unlock(irq_state);
	return live;
}

static bool adc_dsp_owner_release(volatile int8_t *owner, uint8_t stream_id)
{
	const int8_t   transitional = adc_dsp_owner_transitional(stream_id);
	const uint32_t irq_state    = bridge_irq_lock();
	const bool     released     = *owner == (int8_t)stream_id || *owner == transitional;
	if (released) *owner = ADC_DSP_OWNER_NONE;
	bridge_irq_unlock(irq_state);
	return released;
}

/* stream_id currently loaded into the FAC, a negative configuring token,
 * or ADC_DSP_OWNER_NONE while the FAC is idle. */
static volatile int8_t adc_dsp_fac_owner = ADC_DSP_OWNER_NONE;

/* Release the FAC if this stream owned it (called from stream_end). */
void adc_dsp_fac_release(uint8_t stream_id)
{
	if (adc_dsp_owner_release(&adc_dsp_fac_owner, stream_id)) {
		fac_stop();
	}
}

/* Decode one wire coefficient (4 bytes little-endian, Q31 or F32) into
 * the FAC's Q15 fixed-point.  Q31 -> arithmetic >>16; F32 -> clamp to
 * [-1, +1) and scale by 2^15.
 *
 * gh#35 fix 2: `g` is the section's headroom exponent -- the FAC's
 * accumulator gain IPR multiplies the accumulator output by 2^IPR
 * (UM Rev1.2 p.1505 s35.3.6: "The parameter IPR is the gain, applied
 * to the accumulator output by multiplied 2IPR, where IPR is in the
 * range [0:7]"), so coefficients delivered to local memory are scaled
 * DOWN by 2^g and the gain buys the factor back.  This is exactly the
 * mechanism AN208 p.13 prescribes ("To make full use of these data,
 * the above parameters are scaled up by 16384") and the vendor
 * Iir_dma example uses with iir_gain = 1.  Without it, any |coeff| >=
 * 1.0 -- routine for a lightly damped biquad, whose a1 routinely sits
 * between -1 and -2 -- was silently clamped onto the q1.15 rail,
 * changing the filter's cutoff and Q with no error anywhere on the
 * wire.  g == 0 keeps this identical to the pre-gh#35 decode. */
static int16_t adc_dsp_f32_to_q15(float f, uint8_t g)
{
	f = f / (float)(1u << g);
	if (f >= 1.0f) f = 0.999969f;
	if (f <= -1.0f) f = -1.0f;
	return (int16_t)(f * 32768.0f);
}

/* Negate a q1.15 feedback coefficient.  gh#35 fix 1: the wire contract
 * (<alp/dsp.h>) is y[n] = b*x - a*y, but the FAC ADDS the feedback
 * term (UM Rev1.2 p.1505 eq.(35-2): "yn = 2IPR (sum(xn-k x bk) +
 * sum(yn-k x ak))"), so a1/a2 must be sign-reversed at decode or every
 * biquad's poles come out mirrored -- a filter designed stable can be
 * realised unstable.  Both the negation and the halving are confirmed
 * by the vendor Iir_dma example (main.c:58-60 + ipr=1 at :205).  The
 * INT16_MIN case clamps to INT16_MAX because -(-32768) is not
 * representable in int16_t.  Do NOT "fix" the sign back. */
static int16_t adc_dsp_neg_q15(int16_t v)
{
	return (v == INT16_MIN) ? INT16_MAX : (int16_t)(-v);
}

/* Largest |coefficient| across a section's F32 values, for the
 * headroom exponent.  Returns a negative value (-1.0f) to signal
 * "out of FAC range": max|coeff| >= 128 needs g = 7 and still does
 * not fit, which is where gh#35 draws the line -- refuse the chain
 * (sticky, surfaced through stream_read as RANGE) instead of silently
 * clamping onto the rail. */
#define ADC_DSP_FAC_COEFF_MAX 128.0f

static float adc_dsp_f32_max_abs(const float *v, uint8_t n)
{
	float max = 0.0f;
	for (uint8_t k = 0u; k < n; ++k) {
		const float a = (v[k] < 0.0f) ? -v[k] : v[k];
		if (a > max) max = a;
	}
	return (max >= ADC_DSP_FAC_COEFF_MAX) ? -1.0f : max;
}

/* Smallest g in [0,7] with max/2^g <= 1, so no coefficient lands on
 * the q1.15 rail (caller has already rejected max >= 128, so g = 7
 * always suffices; the max == 2^g edge clamps to 0.999969, a 3e-5
 * gain error that is the plane's own resolution). */
static uint8_t adc_dsp_headroom_exp(float max_abs)
{
	uint8_t g = 0u;
	while (g < 7u && max_abs > (float)(1u << g))
		++g;
	return g;
}

/* gh#253: bit-width-derived bias/scale for the FAC pump's input map
 * and output re-bias.  adc_full_scale_for_bits() only ever returns
 * (1 << res_bits) - 1 for res_bits in {12, 10, 8, 6} (adc.c), so
 * full_scale + 1 is always one of {4096, 1024, 256, 64} and the shift
 * that maps a code centred on mid-scale into (most of) the signed
 * q1.15 range is exactly 15 - res_bits.  The 12-bit case (shift 3,
 * mid 2048) is the original gh#35 constant; this generalises it
 * instead of assuming every stream is 12-bit. */
static uint8_t adc_dsp_bias_shift(uint16_t full_scale)
{
	switch (full_scale) {
	case 1023u:
		return 5u; /* 10-bit: 15-10 */
	case 255u:
		return 7u; /* 8-bit: 15-8 */
	case 63u:
		return 9u; /* 6-bit: 15-6 */
	default:
		return 3u; /* 12-bit: 15-12 */
	}
}

/* gh#306: run the vendor preload and prove it completed.
 *
 * fac_fixed_buffer_preload() writes FAC_PARACFG = IPP | LOAD_X0 | EXE
 * unconditionally, then streams `input_size` words into X0.  With
 * input_size == 0 no word ever completes the load, so EXE stays set on
 * silicon (FAC_PARACFG 0x81000000) and the LOAD_X1 coefficient load and
 * the FIR/IIR function config that follow never latch -- the pump then
 * stalls at X0BFF with Y empty.  The vendor Examples/FAC/Fir_polling
 * and Iir_dma preload a NON-EMPTY X0 (input_array_size samples), so we
 * prime X0 with `x0_zeros` zeros (n_taps - 1: the first real sample then
 * yields the first output, since FIR/IIR emit inputs - taps + 1 words).
 * A load is never issued with size 0.  EXE clearing is the hardware's
 * "load latched" signal: bound-wait for it so a wedge is reported
 * (-> dsp_cfg_bad) instead of silently starving the stream. */
#define ADC_DSP_FAC_LOAD_SPIN_MAX 10000u

static bool adc_dsp_fac_preload(fac_fixed_data_preload_struct *pl, uint8_t x0_zeros)
{
	static int16_t zeros[BRIDGE_DSP_MAX_FIR_TAPS]; /* .bss, never written */
	if (x0_zeros == 0u) x0_zeros = 1u;
	if (x0_zeros > BRIDGE_DSP_MAX_FIR_TAPS) return false;
	pl->input_ctx  = zeros;
	pl->input_size = x0_zeros;
	fac_fixed_buffer_preload(pl);
	for (uint32_t spin = 0u; spin < ADC_DSP_FAC_LOAD_SPIN_MAX; ++spin) {
		if ((FAC_PARACFG & FAC_PARACFG_EXE) == 0u) return true;
	}
	fac_deinit(); /* wedged load: reset the block rather than leave EXE stuck */
	return false;
}

/* Configure the FAC for stream s's bound chain (single FIR or single-
 * section IIR -- the only shapes chain_bind now lets through, see
 * adc_dsp_chain_p1_capable() in adc_dsp_chain.c).  Streaming mode:
 * coeffs preloaded into X1, X0 primed with zeros (gh#306, see
 * adc_dsp_fac_preload) -- the pump feeds X0 one sample at a time.  chain_bind is where a caller now learns a chain
 * is unrealisable (BRIDGE_HW_ERR_NOTIMPL, #69) -- the `return false`
 * paths below are unreachable in normal operation once bind enforces
 * the shared predicate; they stay only as a defence-in-depth guard
 * against the two sides drifting apart again, in which case the
 * stream is simply left unfiltered. */
static bool adc_dsp_fac_config(const adc_stream_state_t *s)
{
	const adc_dsp_chain_t *chain = &adc_dsp_chains[s->dsp_chain_id];

	/* Defence in depth: chain_bind already refused any chain P1 can't
	 * realise via this SAME predicate -- re-checking it here means a
	 * future capability lift landing on only one side can't silently
	 * reopen the #69 hang.  Since #132 the predicate also re-runs
	 * adc_dsp_stage_blob_valid() over every populated stage, so it
	 * covers the PAYLOAD fields decoded below (the FIR tap count, the
	 * IIR section count, the FFT out_fmt) and not just the chain's
	 * shape -- which is what this comment always claimed. */
	if (!adc_dsp_chain_p1_capable(chain)) return false;

	/* P1 handles exactly one populated non-FFT stage (guaranteed by
	 * the capability check above; find it to decode its blob). */
	const adc_dsp_stage_t *st = 0;
	for (uint8_t i = 0u; i < BRIDGE_DSP_MAX_STAGES; ++i) {
		if (chain->stages[i].total_size != 0u) {
			st = &chain->stages[i];
			break;
		}
	}
	if (st == 0) return false; /* unreachable post-bind; kept as a defensive guard */

	fac_deinit();
	bridge_rcu_periph_clock_enable(RCU_FAC);

	fac_parameter_struct p;
	fac_struct_para_init(&p);
	const uint8_t depth = 32u; /* X0 working depth beyond the tap window */

	if (st->kind == 0u) { /* FIR: format:u8 n_taps:u8 rsvd:u16 taps[] */
		const uint8_t fmt = st->data[0];
		const uint8_t nt  = st->data[1];
		/* Hard bound at the point of use (#132).  The predicate above
		 * already rejects nt == 0 or nt > BRIDGE_DSP_MAX_FIR_TAPS, so
		 * this cannot fire today -- it is here so the array index below
		 * is provably in range from THIS function alone, without the
		 * reader (or a future editor of adc_dsp_chain.c) having to
		 * carry the bound across a translation-unit boundary.  `taps`
		 * lives on the single 2 KB stack used by the transport ISR's
		 * protocol_dispatch().  Nested dispatch is refused by #19, but
		 * there is still no MSPLIM or stack painting: an overflow here
		 * is silent. */
		if (nt == 0u || nt > BRIDGE_DSP_MAX_FIR_TAPS) return false;
		if (st->total_size != (uint16_t)(BRIDGE_DSP_STAGE_HDR_BYTES + (uint16_t)nt * 4u))
			return false;
		/* gh#35 fix 2: decode every tap, derive the section's headroom
		 * exponent from the largest magnitude, scale, and set IPR to
		 * buy the factor back.  Q31 values are inside [-1, 1) by
		 * construction so g collapses to 0 and the >>16 decode is
		 * exact; only F32 taps can exceed the q1.15 range.  A max >=
		 * 128 cannot be scaled into range even at g = 7 -- refuse
		 * (sticky, surfaced via stream_read as RANGE) rather than
		 * clamping onto the rail and serving a different filter with
		 * STATUS_OK.
		 *
		 * gh#270: the int16 taps and the F32 decode scratch are never
		 * both LIVE at once -- one format populates only its own half
		 * of the loop below, and the later F32->q15 pass only ever
		 * needs taps_u.i16[k] AFTER it has read taps_u.f32[k], so the
		 * two can share one buffer instead of costing a second 256-
		 * byte frame on the 2 KB protocol_dispatch() stack.  The
		 * shrink-in-place write is safe because taps_u.i16[k] never
		 * occupies a byte taps_u.f32[j] for j >= k has not already
		 * been read (2 bytes/tap written can never catch up with the
		 * 4 bytes/tap already consumed). */
		union {
			int16_t i16[BRIDGE_DSP_MAX_FIR_TAPS];
			float   f32[BRIDGE_DSP_MAX_FIR_TAPS];
		} taps_u;
		uint8_t g = 0u;
		for (uint8_t k = 0u; k < nt; ++k) {
			const uint16_t off = (uint16_t)(BRIDGE_DSP_STAGE_HDR_BYTES + (uint16_t)k * 4u);
			const uint32_t w   = (uint32_t)st->data[off] | ((uint32_t)st->data[off + 1u] << 8) |
			                     ((uint32_t)st->data[off + 2u] << 16) |
			                     ((uint32_t)st->data[off + 3u] << 24);
			if (fmt == 1u) { /* Q31 */
				taps_u.i16[k] = (int16_t)((int32_t)w >> 16);
			} else { /* F32 */
				__builtin_memcpy(&taps_u.f32[k], &w, sizeof(taps_u.f32[k]));
			}
		}
		if (fmt != 1u) {
			const float max = adc_dsp_f32_max_abs(taps_u.f32, nt);
			if (max < 0.0f) return false; /* >= 128: out of FAC range */
			g = adc_dsp_headroom_exp(max);
			for (uint8_t k = 0u; k < nt; ++k)
				taps_u.i16[k] = adc_dsp_f32_to_q15(taps_u.f32[k], g);
		}
		p.coeff_addr       = 0u;
		p.coeff_size       = nt;
		p.input_addr       = nt;
		p.input_size       = (uint8_t)(nt + depth);
		p.output_addr      = (uint8_t)(nt + nt + depth);
		p.output_size      = depth;
		p.input_threshold  = FAC_THRESHOLD_1;
		p.output_threshold = FAC_THRESHOLD_1;
		p.clip             = FAC_CP_ENABLE;
		fac_init(&p);

		fac_fixed_data_preload_struct pl;
		fac_fixed_data_preload_init(&pl);
		pl.coeffb_ctx  = taps_u.i16;
		pl.coeffb_size = nt;
		pl.coeffa_ctx  = 0;
		pl.coeffa_size = 0u;
		pl.output_ctx  = 0;
		pl.output_size = 0u;
		if (!adc_dsp_fac_preload(&pl, (uint8_t)(nt - 1u))) return false;

		p.func = FUNC_CONVO_FIR;
		p.ipp  = nt;
		p.ipq  = 0u;
		p.ipr  = g; /* accumulator gain 2^g buys the decode's scaling back */
		fac_function_config(&p);
		fac_start();
		return true;
	}

	if (st->kind == 1u) { /* IIR direct-form-1, SINGLE biquad in P1 -- the
	                       * n_sections == 1 limit is enforced by
	                       * adc_dsp_chain_p1_capable() above, not here. */
		const uint8_t fmt = st->data[0];
		/* section = b0,b1,b2,a1,a2 (5 coeffs).  FAC coeffb = feed-
		 * forward B (b0,b1,b2), coeffa = feedback A (a1,a2).
		 *
		 * gh#35 fixes 1 + 2 (they must land together -- both change
		 * the words written into FAC local memory): the feedback pair
		 * is NEGATED at decode (the FAC adds the feedback term, UM
		 * p.1505 eq.(35-2); the wire contract subtracts it), and all
		 * five coefficients are scaled down by the section's headroom
		 * exponent g with p.ipr = g buying the factor back.  Q31 is
		 * inside [-1, 1) by construction so g = 0.  max|coeff| >= 128
		 * refuses the whole config (sticky, surfaced via stream_read
		 * as RANGE). */
		int16_t        b[3], a[2];
		uint8_t        g = 0u;
		float          fv[5];
		const uint8_t *c = &st->data[BRIDGE_DSP_STAGE_HDR_BYTES];
		for (uint8_t k = 0u; k < 5u; ++k) {
			const uint32_t w = (uint32_t)c[k * 4u] | ((uint32_t)c[k * 4u + 1u] << 8) |
			                   ((uint32_t)c[k * 4u + 2u] << 16) | ((uint32_t)c[k * 4u + 3u] << 24);
			if (fmt == 1u) { /* Q31 */
				const int16_t v = (int16_t)((int32_t)w >> 16);
				if (k < 3u) {
					b[k] = v;
				} else {
					a[k - 3u] = adc_dsp_neg_q15(v);
				}
			} else { /* F32 */
				__builtin_memcpy(&fv[k], &w, sizeof(fv[k]));
			}
		}
		if (fmt != 1u) {
			const float max = adc_dsp_f32_max_abs(fv, 5u);
			if (max < 0.0f) return false; /* >= 128: out of FAC range */
			g = adc_dsp_headroom_exp(max);
			for (uint8_t k = 0u; k < 3u; ++k)
				b[k] = adc_dsp_f32_to_q15(fv[k], g);
			for (uint8_t k = 0u; k < 2u; ++k)
				a[k] = adc_dsp_neg_q15(adc_dsp_f32_to_q15(fv[3u + k], g));
		}
		p.coeff_addr       = 0u;
		p.coeff_size       = 5u; /* b0..b2,a1,a2 */
		p.input_addr       = 5u;
		p.input_size       = (uint8_t)(3u + depth);
		p.output_addr      = (uint8_t)(5u + 3u + depth);
		p.output_size      = depth;
		p.input_threshold  = FAC_THRESHOLD_1;
		p.output_threshold = FAC_THRESHOLD_1;
		p.clip             = FAC_CP_ENABLE;
		fac_init(&p);

		fac_fixed_data_preload_struct pl;
		fac_fixed_data_preload_init(&pl);
		pl.coeffb_ctx  = b;
		pl.coeffb_size = 3u;
		pl.coeffa_ctx  = a;
		pl.coeffa_size = 2u;
		pl.output_ctx  = 0;
		pl.output_size = 0u;
		if (!adc_dsp_fac_preload(&pl, 2u)) return false; /* IPP - 1 */

		/* IPP = feed-forward count (3), IPQ = feedback count (2).
		 * IPR = g: the accumulator gain 2^g buys the decode scaling
		 * back (UM p.1505; vendor Iir_dma main.c:205 iir_gain = 1). */
		p.func = FUNC_IIR_DIRECT_FORM_1;
		p.ipp  = 3u;
		p.ipq  = 2u;
		p.ipr  = g;
		fac_function_config(&p);
		fac_start();
		return true;
	}

	return false; /* WINDOW/FFT are not filter terminals */
}

/* Drain stream sid's new raw samples through the FAC into its processed
 * ring.  Base-level producer of proc_ring + sole consumer of the raw
 * ring for this stream; mirrors stream_read's exact lap+write-index
 * backlog accounting so a mid-pump DMA reload can only UNDER-count. */
static void adc_dsp_pump_stream(uint8_t sid)
{
	adc_stream_state_t *s = &adc_streams[sid];
	if (s->dma_error_count != 0u) return;

	if (adc_dsp_fac_owner != (int8_t)sid) {
		if (!adc_dsp_owner_claim_config(&adc_dsp_fac_owner, sid, s, false)) return;
		if (!adc_dsp_fac_config(s)) {
			(void)adc_dsp_owner_release(&adc_dsp_fac_owner, sid);
			/* gh#35: a config refusal is no longer silent.  The
			 * chain passed bind's shape checks but its
			 * coefficients are out of the FAC's realisable range
			 * (max|coeff| >= 128).  Mark the stream so
			 * stream_read answers RANGE instead of letting the
			 * pump idle forever on a bound chain that will never
			 * produce -- the #69 silent-starvation shape.  Sticky
			 * until stream_end: no auto-retry, the coefficients
			 * cannot change without a new chain_open. */
			s->dsp_cfg_bad = true;
			return;
		}
		if (!adc_dsp_owner_commit_config(&adc_dsp_fac_owner, sid, s, false)) {
			/* Config may have resumed and called fac_start() after an ISR-side
			 * release stopped the block. Leave revoked/ended sessions stopped. */
			fac_stop();
			return;
		}
		/* gh#306: chain_bind seeds pump_raw_read from total_read, which is
		 * 0 when the raw reader never ran; the DMA has been lapping since
		 * stream_begin, so the first pass below would see a full-ring
		 * backlog and answer a spurious BUSY.  Nothing before this point
		 * was ever wanted by the filter: if a ring or more has piled up,
		 * silently jump to the live total (no proc_gap). */
		const uint32_t live = adc_stream_total_written(s, &s->pump_pos);
		if ((uint32_t)(live - s->pump_raw_read) >= s->ring_depth) {
			const uint32_t irq_state = bridge_irq_lock();
			if (adc_dsp_owner_live_locked(&adc_dsp_fac_owner, (int8_t)sid, s, false))
				s->pump_raw_read = live;
			bridge_irq_unlock(irq_state);
		}
	}

	/* Same corrected total the read path uses (gh#149): the pump is
	 * the raw ring's other consumer and owns its own position tracker
	 * (s->pump_pos), so a coalesced lap the prio-3 ISR never counted
	 * cannot silently stale the pump's backlog either.  The pump runs
	 * at base level and can be preempted by the lap ISR mid-call; the
	 * tracker update inside adc_stream_total_written() is safe under
	 * that preemption because lap_count is the only shared field read
	 * (volatile) and the tracker itself is pump-private. */
	const uint32_t total_written = adc_stream_total_written(s, &s->pump_pos);
	int32_t        avail         = (int32_t)(total_written - s->pump_raw_read);
	if (avail <= 0) return;
	if ((uint32_t)avail >= s->ring_depth) {
		/* The pump fell a full ring behind the DMA -- drop the corrupt
		 * backlog and resync so the next batch is gap-free (proc_gap makes
		 * stream_read answer BUSY, same as a raw overrun). */
		const uint32_t irq_state = bridge_irq_lock();
		if (adc_dsp_owner_live_locked(&adc_dsp_fac_owner, (int8_t)sid, s, false)) {
			s->pump_raw_read = total_written;
			s->proc_gap      = true; /* stream_read answers BUSY once (gh#18 B9) */
		}
		bridge_irq_unlock(irq_state);
		return;
	}

	/* gh#253: bias/scale generalised off the stream's OWN bit width
	 * (s->full_scale, snapshotted at stream_begin) instead of a
	 * hardcoded 12-bit mid-scale -- a 10/8/6-bit stream no longer
	 * rails on every sample. */
	const int32_t mid   = (int32_t)((s->full_scale + 1u) / 2u);
	const uint8_t shift = adc_dsp_bias_shift(s->full_scale);

	while (avail > 0) {
		/* gh#272: the FAC work runs with interrupts ON.  Each batch of at
		 * most ADC_DSP_PUMP_LOCK_BATCH samples takes two short sections,
		 * each a flag test-and-set (bridge_critical.h): CLAIM swaps the
		 * FAC owner to the transitional token (the FFT pump's pattern), and
		 * COMMIT publishes pump_raw_read / proc_write only if the token is
		 * still ours.  The interrupt-off window no longer scales with the
		 * FAC access count: the window is bounded and does not grow with the
		 * FAC work.
		 * END during the batch releases the owner (release clears the
		 * transitional token too); commit then fails and the batch is
		 * discarded, so a consumed-but-unfiltered or stale sample is never
		 * exposed (test_fac_post_commit_preemption_cannot_drain_replacement).
		 * Filtered samples go to a LOCAL buffer and are copied into proc_ring
		 * inside the commit section: slot proc_write % ring aliases sample
		 * proc_write - ring, which stream_read may still be serving, so it
		 * must not be overwritten before proc_write is published.  The batch
		 * size only amortises the two sections. */
		const int8_t   transitional = adc_dsp_owner_transitional(sid);
		const uint32_t claim_state  = bridge_irq_lock();
		const bool leased = adc_dsp_owner_live_locked(&adc_dsp_fac_owner, (int8_t)sid, s, false);
		if (leased) adc_dsp_fac_owner = transitional;
		bridge_irq_unlock(claim_state);
		if (!leased) return;

		int32_t batch = avail;
		if (batch > (int32_t)ADC_DSP_PUMP_LOCK_BATCH) batch = (int32_t)ADC_DSP_PUMP_LOCK_BATCH;
		uint32_t raw_read = s->pump_raw_read;
		uint16_t out[ADC_DSP_PUMP_LOCK_BATCH];
		uint32_t n_out   = 0u;
		bool     sat     = false;
		bool     stalled = false;
		int32_t  used    = 0;
		/* A write can release several Y words (gh#306 drains Y, not one
		 * word per write), so out[] can fill before the batch does: stop
		 * feeding when it is full, and never drain past its capacity.  Any
		 * word left in Y stays queued in order and is drained by the next
		 * batch; nothing is dropped or overflowed. */
		for (; used < batch && n_out < ADC_DSP_PUMP_LOCK_BATCH; ++used) {
			/* FAC input saturated: leave the sample in the ring for the
			 * next tick instead of consuming it un-filtered (gh#18 A23). */
			if (fac_flag_get(FAC_FLAG_X0BFF) == SET) {
				stalled = true;
				break;
			}
			const uint16_t ridx = (uint16_t)(raw_read % s->ring_depth);
			const uint16_t code = (uint16_t)(s->ring[ridx] & 0x0FFFu); /* 12-bit raw ring word */
			raw_read++;

			/* gh#35 fix 3 / gh#253: bias the input around mid-scale BEFORE
			 * the shift, so a high-pass / band-pass / DC-blocking biquad's
			 * legitimately negative half is not half-wave-rectified.  x is
			 * a signed q1.15 centred on 0; the mid re-bias below maps a
			 * mid-scale-centred swing back onto the unipolar code plane
			 * WITHOUT discarding the negative half. */
			const int16_t x = (int16_t)(((int32_t)code - mid) << shift);
			fac_fixed_data_write(x);

			/* gh#306: drain every ready output word (YBEF == RESET means Y
			 * is NOT empty), not one per write -- a primed/backed-up Y
			 * otherwise never catches up.  Bounded: Y is 32 deep, Y
			 * threshold 1, and out[] capacity. */
			for (uint8_t drain = 0u; drain < 32u && n_out < ADC_DSP_PUMP_LOCK_BATCH &&
			                         fac_flag_get(FAC_FLAG_YBEF) == RESET;
			     ++drain) {
				/* Re-bias the signed q1.15 output back onto the unipolar
				 * code plane.  Swings beyond one code half-range clip
				 * here; genuine FAC saturation is flagged via dsp_sat
				 * below, so a railed series is never reported as
				 * STATUS_OK. */
				int32_t c = (((int32_t)fac_fixed_data_read()) >> shift) + mid;
				if (c < 0) c = 0;
				if (c > (int32_t)s->full_scale) c = (int32_t)s->full_scale;
				/* gh#35: poll the FAC's sticky error flags (UM p.1515
				 * FAC_STAT STEF bit 10 = output saturation, GSTEF bit 11
				 * = gain saturation) rather than arming their interrupt
				 * enables (STEIE/GSTEIE, p.1514) -- the pump runs at base
				 * level, no vector is needed, and polling cannot preempt
				 * the transports.  Either flag means the served stream
				 * contains railed values: mark it sticky so stream_read
				 * stops answering STATUS_OK. */
				if (SET == fac_flag_get(FAC_FLAG_STEF) || SET == fac_flag_get(FAC_FLAG_GSTEF)) {
					sat = true;
				}
				out[n_out++] = (uint16_t)c;
			}
		}

		const uint32_t commit_state = bridge_irq_lock();
		const bool     live = adc_dsp_owner_live_locked(&adc_dsp_fac_owner, transitional, s, false);
		if (live) {
			uint32_t proc_wr = s->proc_write;
			for (uint32_t i = 0u; i < n_out; ++i)
				s->proc_ring[(proc_wr + i) % BRIDGE_ADC_STREAM_RING_SAMPLES] = out[i];
			s->pump_raw_read = raw_read;
			s->proc_write    = proc_wr + n_out;
			if (sat) s->dsp_sat = true;
			adc_dsp_fac_owner = (int8_t)sid;
		} else if (adc_dsp_fac_owner == transitional) {
			adc_dsp_fac_owner = ADC_DSP_OWNER_NONE; /* as commit_config: never wedge a claim */
		}
		bridge_irq_unlock(commit_state);
		if (!live || stalled) return;
		avail -= used;
	}
}

/* Base-level DSP pump -- called every main-loop tick (bridge_hw_tick).
 * Services every bound FIR/IIR stream; FFT-bound streams have no filter
 * data plane (spectrum is pulled separately). */
void bridge_hw_dsp_pump(void)
{
	for (uint8_t sid = 0u; sid < BRIDGE_ADC_STREAM_COUNT; ++sid) {
		const adc_stream_state_t *s = &adc_streams[sid];
		if (!s->in_use || !s->dsp_bound) continue;
		if (s->dsp_terminal == 3u) {
			adc_dsp_pump_fft(sid); /* spectrum path */
		} else {
			adc_dsp_pump_stream(sid); /* FIR/IIR path */
			/* A BEGIN2 stream's watermark event for a filtered data plane is
			 * "the PROCESSED backlog reached W" (the raw HTF/FTF are muted
			 * while a chain is bound).  A hint only: READ2 re-derives it. */
			if (s->v2 && s->watermark != 0u &&
			    (uint32_t)(s->proc_write - s->proc_read) >= s->watermark) {
				bridge_hw_attn_event_set(sid);
			}
		}
	}
}

/* =====================================================================
 * #496 FFT spectrum path -- WINDOW+FFT terminal chains.
 *
 * The GD32 FFT block (first-lit 2026-07-13; FLOAT real-in / complex-out)
 * transforms a full N-sample window of the ADC stream into a spectrum.
 * Like the FAC there is ONE block, so ONE FFT stream at a time.  The
 * base-level pump accumulates raw samples into a float window; when the
 * window fills it runs the HW FFT (HW-windowed if the chain has a WINDOW
 * stage), reduces to the requested output format, bumps a frame seq, and
 * refills.  The host pulls the latest frame with CMD_ADC_SPECTRUM_READ.
 * ===================================================================== */
/* F3 (review of #69/#70): this is a compile-time ALIAS of
 * BRIDGE_DSP_MAX_FFT_POINTS (adc_dsp_chain.h), not an independently-
 * maintained copy.  Before this fix the two were separately-defined
 * constants that happened to agree (both 1024) -- raising
 * BRIDGE_DSP_MAX_FFT_POINTS alone would have let chain_bind accept an
 * n_points the FFT buffers below are not sized for, so adc_dsp_fft_config
 * would then always fail post-bind and CMD_ADC_SPECTRUM_READ would
 * answer IO forever: the #69 silent-starvation shape, regenerated
 * through this one limit.  Aliasing makes that divergence impossible;
 * bumping the point cap is now a single-macro edit that resizes these
 * buffers automatically. */
#define ADC_DSP_FFT_MAX_POINTS BRIDGE_DSP_MAX_FFT_POINTS

/* Active stream ID, negative configuration/publication token, or
 * idle. */
static volatile int8_t   adc_dsp_fft_owner = ADC_DSP_OWNER_NONE;
static uint16_t          adc_dsp_fft_points;
static uint8_t           adc_dsp_fft_outfmt; /* 0 complex / 1 mag / 2 mag-onesided */
static uint16_t          adc_dsp_fft_fill;
static volatile uint32_t adc_dsp_fft_seq;   /* completed-frame counter */
static uint16_t          adc_dsp_fft_nbins; /* bins in the current frame */
static float             adc_dsp_fft_real[ADC_DSP_FFT_MAX_POINTS];
static float             adc_dsp_fft_out[ADC_DSP_FFT_MAX_POINTS * 2u]; /* re,im */
static float             adc_dsp_fft_wcoef[ADC_DSP_FFT_MAX_POINTS];
static float             adc_dsp_fft_bins[ADC_DSP_FFT_MAX_POINTS * 2u]; /* published */

/* Start a new FFT publication session. The bins may keep their old bytes,
 * but seq == 0 gates every read until the new session publishes a complete
 * frame; keeping these three fields together prevents a lifecycle path from
 * reporting an old sequence as a readable (but empty) new-session frame
 * (#140). */
static void adc_dsp_fft_session_reset(void)
{
	adc_dsp_fft_fill  = 0u;
	adc_dsp_fft_seq   = 0u;
	adc_dsp_fft_nbins = 0u;
}

static uint8_t adc_dsp_fft_point_enum(uint16_t n)
{
	switch (n) {
	case 32u:
		return FFT_POINT_32;
	case 64u:
		return FFT_POINT_64;
	case 128u:
		return FFT_POINT_128;
	case 256u:
		return FFT_POINT_256;
	default:
		break;
	}
	/* 512 / 1024 continue the enum (see gd32g5x3_fft.h). */
	if (n == 512u) return (uint8_t)(FFT_POINT_256 + 1u);
	return (uint8_t)(FFT_POINT_256 + 2u); /* 1024 */
}

/* Fill the HW window coefficient buffer for a shape (0 rect => flat 1.0,
 * 1 Hann, 2 Hamming, 3 Blackman).  Symmetric window over N points. */
static void adc_dsp_fft_make_window(uint8_t shape, uint16_t n)
{
	const float twopi = 6.28318530718f;
	for (uint16_t i = 0u; i < n; ++i) {
		const float t = (float)i / (float)(n - 1u);
		float       w;
		switch (shape) {
		case 1u:
			w = 0.5f - 0.5f * __builtin_cosf(twopi * t);
			break; /* Hann    */
		case 2u:
			w = 0.54f - 0.46f * __builtin_cosf(twopi * t);
			break; /* Hamming */
		case 3u:
			w = 0.42f - 0.5f * __builtin_cosf(twopi * t) + 0.08f * __builtin_cosf(2.0f * twopi * t);
			break; /* Blackman*/
		default:
			w = 1.0f;
			break; /* rect    */
		}
		adc_dsp_fft_wcoef[i] = w;
	}
}

/* Configure the FFT block + window for stream s's bound FFT chain.
 * chain_bind is now where a FIR/IIR-ahead-of-FFT shape is refused
 * (BRIDGE_HW_ERR_NOTIMPL, #69) -- the checks below are a defence-in-
 * depth re-check via the SAME shared predicate chain_bind uses, not
 * an independent copy of the "WINDOW+FFT only" limit. */
static bool adc_dsp_fft_config(const adc_stream_state_t *s)
{
	const adc_dsp_chain_t *chain = &adc_dsp_chains[s->dsp_chain_id];

	if (!adc_dsp_chain_p1_capable(chain)) return false;

	const adc_dsp_stage_t *fft_st = 0, *win_st = 0;
	for (uint8_t i = 0u; i < BRIDGE_DSP_MAX_STAGES; ++i) {
		if (chain->stages[i].total_size == 0u) continue;
		if (chain->stages[i].kind == 3u)
			fft_st = &chain->stages[i];
		else if (chain->stages[i].kind == 2u)
			win_st = &chain->stages[i];
		/* No other kind can be populated here -- guaranteed by the
		 * capability check above. */
	}
	if (fft_st == 0) return false; /* unreachable post-bind; kept as a defensive guard */

	const uint16_t n   = (uint16_t)(fft_st->data[0] | ((uint16_t)fft_st->data[1] << 8));
	const uint8_t  ofm = fft_st->data[2];
	/* F3 (review of #69/#70): adc_dsp_chain_p1_capable() above now folds
	 * in this same bound (BRIDGE_DSP_MIN_FFT_POINTS/MAX_FFT_POINTS,
	 * which ADC_DSP_FFT_MAX_POINTS aliases) -- unreachable post-bind,
	 * kept as a defensive guard against the two checks drifting apart. */
	if (n < BRIDGE_DSP_MIN_FFT_POINTS || n > ADC_DSP_FFT_MAX_POINTS) return false;

	adc_dsp_fft_points = n;
	adc_dsp_fft_outfmt = ofm;
	adc_dsp_fft_session_reset();

	const uint8_t shape = (win_st != 0) ? win_st->data[0] : 0u;

	fft_deinit();
	bridge_rcu_periph_clock_enable(RCU_FFT);
	fft_parameter_struct f;
	fft_struct_para_init(&f);
	f.mode_sel     = FFT_MODE;
	f.point_num    = adc_dsp_fft_point_enum(n);
	f.downsamp_sel = FFT_DOWNSAMPLE_1;
	f.image_source = FFT_IM_ZERO;
	f.real_addr    = (uint32_t)(uintptr_t)adc_dsp_fft_real;
	f.image_addr   = 0u;
	f.output_addr  = (uint32_t)(uintptr_t)adc_dsp_fft_out;
	if (win_st != 0 || shape != 0u) {
		adc_dsp_fft_make_window(shape, n);
		f.window_enable = FFT_WINDOW_ENABLE;
		f.window_addr   = (uint32_t)(uintptr_t)adc_dsp_fft_wcoef;
	} else {
		f.window_enable = FFT_WINDOW_DISABLE;
		f.window_addr   = 0u;
	}
	fft_init(&f);
	return true;
}

/* Reduce the FFT block's complex output into the pending bin buffer. The
 * caller publishes nbins + sequence only after revalidating the owner lease. */
static uint16_t adc_dsp_fft_prepare_bins(void)
{
	const uint16_t n = adc_dsp_fft_points;
	if (adc_dsp_fft_outfmt == 0u) { /* COMPLEX: re,im interleaved, 2N */
		for (uint16_t i = 0u; i < n * 2u; ++i)
			adc_dsp_fft_bins[i] = adc_dsp_fft_out[i];
		return (uint16_t)(n * 2u);
	} else { /* MAGNITUDE (N) or MAGNITUDE_ONESIDED (N/2+1) */
		const uint16_t nb = (adc_dsp_fft_outfmt == 2u) ? (uint16_t)(n / 2u + 1u) : n;
		for (uint16_t i = 0u; i < nb; ++i) {
			const float re      = adc_dsp_fft_out[i * 2u];
			const float im      = adc_dsp_fft_out[i * 2u + 1u];
			adc_dsp_fft_bins[i] = __builtin_sqrtf(re * re + im * im);
		}
		return nb;
	}
}

/* Pump the FFT path for stream sid: accumulate new raw samples into the
 * float window; on a full window run the HW FFT and publish. */
static void adc_dsp_pump_fft(uint8_t sid)
{
	adc_stream_state_t *s = &adc_streams[sid];
	if (s->dma_error_count != 0u) return;

	if (adc_dsp_fft_owner != (int8_t)sid) {
		/* A negative configuring token is visible to release but never to
		 * spectrum_read, so no stale sequence becomes readable before the
		 * config path resets this session (#140/#184). */
		if (!adc_dsp_owner_claim_config(&adc_dsp_fft_owner, sid, s, true)) return;
		if (!adc_dsp_fft_config(s)) {
			(void)adc_dsp_owner_release(&adc_dsp_fft_owner, sid);
			return;
		}
		if (!adc_dsp_owner_commit_config(&adc_dsp_fft_owner, sid, s, true)) return;
		/* pump_raw_read is deliberately NOT rewound here (#70).
		 * chain_bind already seeds it (adc_dsp_chain.c: `s->pump_raw_read
		 * = s->total_read;`) at the moment this stream's chain was
		 * bound; re-seeding it again on every ownership flip is what
		 * converted transient FAC/FFT contention into PERMANENT
		 * starvation -- total_read never advances for an FFT-bound
		 * stream (stream_read bails on dsp_terminal == 3u before
		 * touching it), so this rewind kept resetting the window to the
		 * same frozen value on every pump tick, which never let
		 * adc_dsp_fft_seq leave 0.  UNPROVEN HOST-SIDE: whether the
		 * rewind is needed for a genuine same-stream re-attach (a
		 * stream_end -> chain_release -> later chain_bind on the SAME
		 * sid, where chain_bind's own seed already covers it) is a
		 * question about DMA down-counter semantics and lap-ISR
		 * interleaving that no host harness reproduces -- confirm on a
		 * bench before relying on this. */
	}

	/* Same corrected total as the FIR/IIR pump (gh#149, see the
	 * comment there): the FFT pump is the raw ring's consumer for an
	 * FFT-bound stream and shares the pump-side position tracker. */
	const uint32_t total_written = adc_stream_total_written(s, &s->pump_pos);
	int32_t        avail         = (int32_t)(total_written - s->pump_raw_read);
	if (avail <= 0) return;
	if ((uint32_t)avail >= s->ring_depth) {
		const uint32_t irq_state = bridge_irq_lock();
		if (adc_dsp_owner_live_locked(&adc_dsp_fft_owner, (int8_t)sid, s, true)) {
			s->pump_raw_read = total_written; /* fell behind -> resync, drop partial window */
			adc_dsp_fft_fill = 0u;
		}
		bridge_irq_unlock(irq_state);
		return;
	}

	while (avail-- > 0) {
		/* Capture the raw code under the session lease, then perform the
		 * soft-float conversion with interrupts enabled. The production
		 * build lowers it to __aeabi_ui2f/__aeabi_fdiv calls, which are far
		 * too long for the bridge's short PRIMASK sections. */
		uint32_t irq_state = bridge_irq_lock();
		if (!adc_dsp_owner_live_locked(&adc_dsp_fft_owner, (int8_t)sid, s, true)) {
			bridge_irq_unlock(irq_state);
			return;
		}
		const uint16_t ridx = (uint16_t)(s->pump_raw_read % s->ring_depth);
		const uint16_t code = (uint16_t)(s->ring[ridx] & 0x0FFFu);
		bridge_irq_unlock(irq_state);
		const float sample = (float)code / 4096.0f;

		irq_state = bridge_irq_lock();
		if (!adc_dsp_owner_live_locked(&adc_dsp_fft_owner, (int8_t)sid, s, true)) {
			bridge_irq_unlock(irq_state);
			return;
		}
		s->pump_raw_read++;
		adc_dsp_fft_real[adc_dsp_fft_fill++] = sample;
		const bool frame_ready               = adc_dsp_fft_fill >= adc_dsp_fft_points;
		if (frame_ready) fft_calculation_start();
		bridge_irq_unlock(irq_state);
		if (!frame_ready) continue;

		/* The completion wait and bin reduction are intentionally
		 * interruptible. END revokes the owner first; the two lease checks
		 * below then prevent this suspended pump from publishing into a
		 * replacement session that reuses the same stream ID. */
		uint32_t g = 0u;
		while (fft_flag_get(FFT_FLAG_CCF) == RESET && ++g < 1000000u) {
		}
		const bool complete = fft_flag_get(FFT_FLAG_CCF) != RESET;

		uint32_t publish_irq_state = bridge_irq_lock();
		if (!complete) {
			const bool still_live =
			    adc_dsp_owner_live_locked(&adc_dsp_fft_owner, (int8_t)sid, s, true);
			if (still_live) adc_dsp_fft_fill = 0u;
			bridge_irq_unlock(publish_irq_state);
			if (!still_live) return;
			continue;
		}

		/* Make the single bin buffer unreadable before reducing into it.
		 * END can revoke this transitional token while the long copy/sqrt
		 * work remains interruptible; spectrum_read reports BUSY instead of
		 * returning old/new bins under one old sequence (#18/#187). */
		const int8_t publishing = adc_dsp_owner_transitional(sid);
		const bool   publishing_claimed =
		    adc_dsp_owner_live_locked(&adc_dsp_fft_owner, (int8_t)sid, s, true);
		if (publishing_claimed) adc_dsp_fft_owner = publishing;
		bridge_irq_unlock(publish_irq_state);
		if (!publishing_claimed) return;

		const uint16_t nbins  = adc_dsp_fft_prepare_bins();
		publish_irq_state     = bridge_irq_lock();
		const bool still_live = adc_dsp_owner_live_locked(&adc_dsp_fft_owner, publishing, s, true);
		if (still_live) {
			adc_dsp_fft_nbins = nbins;
			adc_dsp_fft_seq++;
			adc_dsp_fft_fill  = 0u;
			adc_dsp_fft_owner = (int8_t)sid;
		} else if (adc_dsp_fft_owner == publishing) {
			adc_dsp_fft_owner = ADC_DSP_OWNER_NONE;
		}
		bridge_irq_unlock(publish_irq_state);
		if (!still_live) return;
	}
}

/* Release the FFT block if this stream owned it (stream_end). */
void adc_dsp_fft_release(uint8_t stream_id)
{
	if (adc_dsp_owner_release(&adc_dsp_fft_owner, stream_id)) {
		adc_dsp_fft_session_reset();
	}
}

/* HAL: read spectrum bins (float32 LE) for a bound FFT stream.  Chunked:
 * the host asks for [bin_offset, bin_offset+max_bins); the reply carries
 * the frame seq so the host detects a frame roll mid-fetch.  Returns
 * NOSUPPORT if the stream isn't FFT-bound, BUSY before the first frame, and
 * IO after its DMA channel reports a transfer error. */
int bridge_hw_adc_spectrum_read(uint8_t   stream_id,
                                uint16_t  bin_offset,
                                uint8_t   max_bins,
                                uint32_t *seq_out,
                                uint16_t *total_bins_out,
                                uint8_t  *got_bins_out,
                                float    *bins_out)
{
	if (seq_out == 0 || total_bins_out == 0 || got_bins_out == 0 || bins_out == 0) {
		return BRIDGE_HW_ERR_INVAL;
	}
	*got_bins_out = 0u;
	if (stream_id >= BRIDGE_ADC_STREAM_COUNT) return BRIDGE_HW_ERR_RANGE;
	adc_stream_state_t *s = &adc_streams[stream_id];
	if (!s->in_use || !s->dsp_bound || s->dsp_terminal != 3u) return BRIDGE_HW_ERR_NOTIMPL;
	adc_stream_latch_dma_error(stream_id);
	if (s->dma_error_count != 0u) return BRIDGE_HW_ERR_IO;
	if (adc_dsp_fft_owner != (int8_t)stream_id || adc_dsp_fft_seq == 0u) {
		return BRIDGE_HW_ERR_BUSY; /* no frame yet */
	}

	*seq_out        = adc_dsp_fft_seq;
	*total_bins_out = adc_dsp_fft_nbins;
	if (bin_offset >= adc_dsp_fft_nbins) return BRIDGE_HW_OK; /* past the end */

	uint16_t remain = (uint16_t)(adc_dsp_fft_nbins - bin_offset);
	uint8_t  emit   = (remain < max_bins) ? (uint8_t)remain : max_bins;
	for (uint8_t i = 0u; i < emit; ++i)
		bins_out[i] = adc_dsp_fft_bins[bin_offset + i];
	*got_bins_out = emit;
	return BRIDGE_HW_OK;
}
