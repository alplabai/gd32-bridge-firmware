/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 bridge HAL backend -- PWM output.
 * Split move-only from hal/bridge_hw_gd32.c (fw v0.2.8); see
 * hal/gd32/init.c for the backend-wide implementation notes.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bridge_hw.h"
#include "gd32g5x3.h"

#include "gd32_common.h"

/* ----------------------------------------------------------------- */
/* PWM channels (TIMER0 + TIMER7).                                    */
/* ----------------------------------------------------------------- */

/* E1M PWM channel -> GD32 (timer, channel, output kind, pad, AF).
 * Sourced from `metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv` for the
 * pad column; AF + timer-channel from the GD32G553xx Datasheet Rev2.0
 * Tables 2-10..2-13 (pin alternate-function summary).
 *
 *   PWM0  PA11  TIMER0_MCH0 (complement.) AF6
 *   PWM1  PB1   TIMER0_MCH2 (complement.) AF6
 *   PWM2  PB14  TIMER0_MCH1 (complement.) AF6
 *   PWM3  PC5   TIMER0_MCH3 (complement.) AF6
 *   PWM4  PC10  TIMER7_MCH0               AF4
 *   PWM5  PC11  TIMER7_MCH1               AF4
 *   PWM6  PC12  TIMER7_MCH2               AF4
 *   PWM7  PD0   TIMER7_MCH3               AF6
 *
 * Every PWM rides a distinct TIMER channel's complementary output,
 * so per-channel duty cycles are fully independent.  Each channel's
 * main output (CHx) is unused on V2N -- only the complementary
 * (CHxN) pad sits on the E1M PWM connector.
 *
 * Periods are SHARED across all PWMs of the same timer (TIMER0:
 * PWM0..3; TIMER7: PWM4..7) because each TIMER has one ARR.  The
 * per-channel `bridge_hw_pwm_set` body updates the timer's ARR every
 * call -- last write wins.  In typical V2N use the host sets the
 * same period across each group so this doesn't surface. */

const gd32_pwm_ch_t pwm_channels[] = {
	[0] = { TIMER0, TIMER_CH_0, true, GPIOA, GPIO_PIN_11, GPIO_AF_6 },
	[1] = { TIMER0, TIMER_CH_2, true, GPIOB, GPIO_PIN_1, GPIO_AF_6 },
	[2] = { TIMER0, TIMER_CH_1, true, GPIOB, GPIO_PIN_14, GPIO_AF_6 },
	[3] = { TIMER0, TIMER_CH_3, true, GPIOC, GPIO_PIN_5, GPIO_AF_6 },
	[4] = { TIMER7, TIMER_CH_0, true, GPIOC, GPIO_PIN_10, GPIO_AF_4 },
	[5] = { TIMER7, TIMER_CH_1, true, GPIOC, GPIO_PIN_11, GPIO_AF_4 },
	[6] = { TIMER7, TIMER_CH_2, true, GPIOC, GPIO_PIN_12, GPIO_AF_4 },
	[7] = { TIMER7, TIMER_CH_3, true, GPIOD, GPIO_PIN_0, GPIO_AF_6 },
};
_Static_assert(sizeof(pwm_channels) / sizeof(pwm_channels[0]) == PWM_CHANNEL_COUNT,
               "pwm_channels size must match PWM_CHANNEL_COUNT");

/* No read-back cache: bridge_hw_pwm_get reads CAR/CHxCV straight from
 * the timer so the host sees what the silicon is actually generating,
 * never a software echo of its own request (silicon lesson 2026-06-04:
 * a cache echo "verified" PWM for weeks while the pads were idle). */

/* Per-timer init.  Called once per peripheral from bridge_hw_init();
 * Advanced timers need timer_primary_output_config(ENABLE) before any
 * output pin actually drives (vs basic timers, where the channel
 * enable is sufficient). */
void pwm_timer_init(uint32_t periph)
{
	timer_parameter_struct ip;
	timer_struct_para_init(&ip);
	ip.prescaler         = (uint16_t)PWM_TIMER_PRESCALER;
	ip.alignedmode       = TIMER_COUNTER_EDGE;
	ip.counterdirection  = TIMER_COUNTER_UP;
	ip.period            = PWM_TIMER_ARR_MAX; /* 65.5 ms default; per-set */
	ip.clockdivision     = TIMER_CKDIV_DIV1;
	ip.repetitioncounter = 0u;
	timer_deinit(periph);
	timer_init(periph, &ip);
	timer_primary_output_config(periph, ENABLE);
	timer_enable(periph);
}

/* #129: how a one-shot is armed, and why it needs tracking.
 *
 * Single-pulse mode halts the counter at the next update event, and that
 * update RELOADS CNT to 0.  In PWM0 (output active while CNT < CHxCV)
 * every CHxCV >= 1 therefore reads as active at CNT == 0, so the halted
 * one-shot parks its pad at the ACTIVE level and stays there until some
 * later bridge_hw_pwm_set -- the exact failure a one-shot exists to
 * prevent.  No choice of ARR/CHxCV fixes that in PWM0: CHxCV == 0 emits
 * no pulse at all, and any other value is active at CNT == 0.
 *
 * PWM1 inverts the comparison (inactive while CNT < CHxCV), so the same
 * halt-at-CNT-0 leaves the pad INACTIVE.  The pulse is then the window
 * between the compare match and the reload:
 *     delay  = CHxCV ticks (pad inactive)
 *     pulse  = ARR - CHxCV + 1 ticks (pad active)
 * With CHxCV fixed at PWM_ONE_SHOT_DELAY_TICKS and ARR = pulse_us, that
 * is a 1 us lead-in followed by exactly pulse_us of active output, then a
 * clean return to idle.
 *
 * The mode is per-channel state that outlives the call, so it is tracked:
 * bridge_hw_pwm_set restores PWM0 (alongside the SPM clear it already
 * does) and bridge_hw_pwm_get has to invert its duty arithmetic while a
 * channel is still in the one-shot mode, or it reports the lead-in as the
 * pulse width. */
#define PWM_ONE_SHOT_DELAY_TICKS 1u

static bool pwm_one_shot[PWM_CHANNEL_COUNT];

/* Per-channel init.  Sets PWM mode 0 (output high while counter <
 * compare) and 0 duty -- HW pad sits low until the host issues a
 * bridge_hw_pwm_set with a non-zero duty. */
void pwm_channel_init(const gd32_pwm_ch_t *ch)
{
	timer_oc_parameter_struct oc;
	timer_channel_output_struct_para_init(&oc);
	if (ch->complement) {
		oc.outputstate  = TIMER_CCX_DISABLE;
		oc.outputnstate = TIMER_CCXN_ENABLE;
	} else {
		oc.outputstate  = TIMER_CCX_ENABLE;
		oc.outputnstate = TIMER_CCXN_DISABLE;
	}
	oc.ocpolarity   = TIMER_OC_POLARITY_HIGH;
	oc.ocnpolarity  = TIMER_OCN_POLARITY_HIGH;
	oc.ocidlestate  = TIMER_OC_IDLE_STATE_LOW;
	oc.ocnidlestate = TIMER_OCN_IDLE_STATE_LOW;
	timer_channel_output_config(ch->periph, ch->channel, &oc);
	timer_channel_output_pulse_value_config(ch->periph, ch->channel, 0u);
	timer_channel_output_mode_config(ch->periph, ch->channel, TIMER_OC_MODE_PWM0);
	timer_channel_output_shadow_config(ch->periph, ch->channel, TIMER_OC_SHADOW_DISABLE);
	/* This function is the one place PWM0 is (re-)asserted from scratch,
     * so it owns clearing the one-shot mode shadow (#129).  Both callers
     * -- bridge_hw_init at boot and pwm_capture.c's full output-stage
     * restore at capture end -- pass &pwm_channels[i], so recovering the
     * index by pointer arithmetic keeps the invariant "hardware is in
     * PWM0 <=> pwm_one_shot[] is false" true without either caller having
     * to know the shadow exists.  Without this, a one-shot followed by a
     * capture session would leave the flag set while the hardware was
     * back in PWM0, and bridge_hw_pwm_get would invert its arithmetic
     * against a channel that is no longer inverted. */
	const size_t idx = (size_t)(ch - pwm_channels);
	if (idx < PWM_CHANNEL_COUNT) {
		pwm_one_shot[idx] = false;
	}
}

/* Sticky per-TIMER counter-alignment mode set by bridge_hw_pwm_configure.
 * Index 0 = TIMER0 (PWM0..3), index 1 = TIMER7 (PWM4..7); value is the
 * wire align_mode (0 edge / 1 center-up / 2 center-down / 3 center-both,
 * matching alp_pwm_align_t).  CAM is a timer-wide field shared by every
 * channel on the timer, so this is per-timer, not per-channel:
 * last-write-wins across sibling channels (documented in the protocol
 * spec).  bridge_hw_pwm_set / _get read it to convert between commanded
 * period/duty and the ARR/compare ticks, because a center-aligned
 * counter runs 0->ARR->0 (period == 2*ARR ticks) where an edge-aligned
 * counter runs 0->ARR (period == ARR+1 ticks). */
static uint8_t pwm_align_mode[2];

/* TIMER base -> pwm_align_mode index. */
static uint8_t pwm_timer_index(uint32_t periph)
{
	return (periph == TIMER0) ? 0u : 1u;
}

int bridge_hw_pwm_set(uint8_t channel, uint32_t period_ns, uint32_t duty_ns)
{
	if (channel >= PWM_CHANNEL_COUNT) return BRIDGE_HW_ERR_RANGE;
	if (duty_ns > period_ns) return BRIDGE_HW_ERR_INVAL;

	/* Clear OPM if a prior bridge_hw_pwm_single_pulse left the timer
     * in one-pulse mode -- per the contract, a subsequent PWM_SET
     * returns the channel (and any other channels on the same timer)
     * to continuous output. */
	timer_single_pulse_mode_config(pwm_channels[channel].periph, TIMER_SP_MODE_REPETITIVE);
	/* ...and restore PWM0 if that one-shot also left the channel in the
     * inverted compare mode it needs to return the pad to idle (#129).
     * Same contract clause as the SPM clear above: a PWM_SET returns the
     * channel to continuous output, which has to include the sense of the
     * comparison, not just the halt behaviour.  Unconditional rather than
     * guarded on pwm_one_shot[]: re-asserting the mode a channel is
     * already in is a plain register write with no edge, and it also
     * repairs a channel left in PWM1 by anything this array did not see. */
	timer_channel_output_mode_config(
	    pwm_channels[channel].periph, pwm_channels[channel].channel, TIMER_OC_MODE_PWM0);
	pwm_one_shot[channel] = false;

	/* Round period + duty to whole microseconds (the timer tick). */
	uint32_t period_us = period_ns / PWM_TIMER_TICK_NS;
	uint32_t duty_us   = duty_ns / PWM_TIMER_TICK_NS;
	if (period_us == 0u) return BRIDGE_HW_ERR_RANGE;

	const gd32_pwm_ch_t *ch = &pwm_channels[channel];

	/* Convert commanded period/duty to ARR + compare, honouring the
	 * timer's configured alignment (bridge_hw_pwm_configure).  The
	 * up-counter (edge) counts 0..ARR inclusive -> period == ARR+1
	 * ticks and high-time == compare ticks.  A center-aligned counter
	 * runs 0->ARR->0 -> period == 2*ARR ticks and high-time ==
	 * 2*compare ticks (compare on the up-ramp + compare on the
	 * down-ramp), so both ARR and compare are the commanded value
	 * halved.  ARR must fit in 16 bits either way; clamp on over-range
	 * so the timer never gets an invalid reload. */
	uint32_t arr, cmp;
	if (pwm_align_mode[pwm_timer_index(ch->periph)] == 0u) {
		if (period_us > PWM_TIMER_ARR_MAX + 1u) period_us = PWM_TIMER_ARR_MAX + 1u;
		if (duty_us > period_us) duty_us = period_us;
		arr = period_us - 1u;
		cmp = duty_us;
	} else {
		uint32_t half_period = period_us / 2u;
		uint32_t half_duty   = duty_us / 2u;
		if (half_period == 0u) return BRIDGE_HW_ERR_RANGE; /* period < 2 us */
		if (half_period > PWM_TIMER_ARR_MAX) half_period = PWM_TIMER_ARR_MAX;
		if (half_duty > half_period) half_duty = half_period;
		arr = half_period;
		cmp = half_duty;
	}

	/* Updates ALL channels of the same timer -- the contract documents
	 * this shared-ARR constraint. */
	timer_autoreload_value_config(ch->periph, arr);
	timer_channel_output_pulse_value_config(ch->periph, ch->channel, cmp);
	return BRIDGE_HW_OK;
}

int bridge_hw_pwm_get(uint8_t channel, uint32_t *period_ns, uint32_t *duty_ns)
{
	if (period_ns == 0 || duty_ns == 0) return BRIDGE_HW_ERR_INVAL;
	*period_ns = 0u;
	*duty_ns   = 0u;
	if (channel >= PWM_CHANNEL_COUNT) return BRIDGE_HW_ERR_RANGE;

	const gd32_pwm_ch_t *ch = &pwm_channels[channel];

	/* Hardware read-back: convert the live CAR/CHxCV ticks to ns at
     * the fixed 1 us tick.  This reports what the pad is actually
     * doing -- including the shared-period reality (one ARR per
     * timer: a PWM_SET on a sibling channel moves this channel's
     * reported period too) and the boot default (65.536 ms period,
     * 0 duty) before the first PWM_SET.  CHxCV can legitimately read
     * ARR + 1 (a pre-#129 single-pulse programmed compare past the
     * period); clamp so duty never reports > period.
     *
     * The tick->ns conversion mirrors bridge_hw_pwm_set's alignment
     * math: edge-aligned period is CAR+1 ticks, center-aligned is
     * 2*CAR ticks (and duty 2*CV), so a channel configured
     * center-aligned reads back the same period/duty it was
     * commanded. */
	const uint32_t car = TIMER_CAR(ch->periph) & PWM_TIMER_ARR_MAX;
	uint32_t       cv  = timer_channel_capture_value_register_read(ch->periph, ch->channel);
	if (cv > car + 1u) cv = car + 1u;
	if (pwm_one_shot[channel]) {
		/* One-shot channels sit in PWM1, where CHxCV is the INACTIVE
         * lead-in and the active width is CAR - CHxCV + 1 (#129).
         * Reading CHxCV as high-time here would report a 1 us pulse for
         * every one-shot regardless of what was commanded.  One-shots are
         * always edge-aligned -- bridge_hw_pwm_single_pulse refuses a
         * center-aligned timer outright -- so there is no center variant
         * to handle. */
		*period_ns = (car + 1u) * PWM_TIMER_TICK_NS;
		*duty_ns   = (car - cv + 1u) * PWM_TIMER_TICK_NS;
		return BRIDGE_HW_OK;
	}
	if (pwm_align_mode[pwm_timer_index(ch->periph)] == 0u) {
		*period_ns = (car + 1u) * PWM_TIMER_TICK_NS;
		*duty_ns   = cv * PWM_TIMER_TICK_NS;
	} else {
		*period_ns = 2u * car * PWM_TIMER_TICK_NS;
		*duty_ns   = 2u * cv * PWM_TIMER_TICK_NS;
		if (*duty_ns > *period_ns) *duty_ns = *period_ns;
	}
	return BRIDGE_HW_OK;
}

int bridge_hw_pwm_configure(uint8_t  channel,
                            uint8_t  align_mode,
                            uint32_t dead_time_ns,
                            uint8_t  break_cfg)
{
	if (channel >= PWM_CHANNEL_COUNT) return BRIDGE_HW_ERR_RANGE;

	/* align_mode 0..3 map to CTL0.CAM (0 edge, 1 center-up, 2
	 * center-down, 3 center-both -- alp_pwm_align_t).  Reject out of
	 * range before touching hardware so a bad value leaves the timer
	 * untouched. */
	if (align_mode > 3u) return BRIDGE_HW_ERR_INVAL;

	/* dead_time_ns and break_cfg are advanced-timer features the GD32
	 * TIMER0/7 silicon HAS, but that the V2N E1M routing does not
	 * physically expose, so honouring them would be an unverifiable
	 * register write (exactly the "cache echo verified PWM while the
	 * pads were idle" trap this file warns about above):
	 *   - Dead-time is the guard gap between a channel's main (CHx) and
	 *     complementary (CHxN) outputs, but only the CHxN pad sits on
	 *     the E1M PWM connector (see the channel-map comment) -- there
	 *     is no complementary PAIR on the wire for a dead-time to act
	 *     on, so any DTCFG value is physically inert.
	 *   - The break input needs a BRK pad, and the V2N gd32-io-mcu-map
	 *     routes none -- the break logic could be armed but nothing
	 *     could ever trigger it.
	 * Both therefore return NOSUPPORT on this board (a hardware-routing
	 * limit, not an unimplemented feature); a future carrier that
	 * routes the complementary pair / a BRK pad would lift this.
	 * align_mode == 0 with both zero is still the idempotent
	 * "set-to-defaults" call and succeeds. */
	if (dead_time_ns != 0u) return BRIDGE_HW_ERR_NOTIMPL;
	if (break_cfg != 0u) return BRIDGE_HW_ERR_NOTIMPL;

	const gd32_pwm_ch_t *ch  = &pwm_channels[channel];
	const uint8_t        idx = pwm_timer_index(ch->periph);
	if (pwm_align_mode[idx] == align_mode) return BRIDGE_HW_OK; /* no change */

	/* CAM is a shared timer-wide field and the GD32 (like most STM32-
	 * lineage advanced timers) latches it only while the counter is
	 * stopped, so bracket the change in disable/enable.  This briefly
	 * halts every channel on the timer -- acceptable at config time,
	 * and last-write-wins across the sibling channels is the documented
	 * contract.  The counter value + ARR/compare survive the toggle, so
	 * a running duty resumes with the new alignment on the next
	 * update.
	 *
	 * PRESERVE the run state: a prior bridge_hw_pwm_single_pulse on a
	 * sibling channel leaves the timer HALTED (hardware clears CEN at
	 * the one-shot's update event) with SP=SINGLE and a pulse-shaped
	 * ARR/compare still latched.  Unconditionally re-enabling here would
	 * re-arm that one-shot and fire a SECOND, uncommanded pulse on the
	 * sibling pad -- an actuation with no host command.  So only
	 * re-enable if the timer was actually running when we entered. */
	static const uint32_t cam_map[4] = {
		TIMER_COUNTER_EDGE,        /* 0: edge          */
		TIMER_COUNTER_CENTER_UP,   /* 1: center-up     */
		TIMER_COUNTER_CENTER_DOWN, /* 2: center-down   */
		TIMER_COUNTER_CENTER_BOTH, /* 3: center-both   */
	};
	const bool was_running = (TIMER_CTL0(ch->periph) & (uint32_t)TIMER_CTL0_CEN) != 0u;
	timer_disable(ch->periph);
	uint32_t ctl0 = TIMER_CTL0(ch->periph);
	ctl0 &= ~(uint32_t)TIMER_CTL0_CAM;
	ctl0 |= cam_map[align_mode];
	TIMER_CTL0(ch->periph) = ctl0;
	if (was_running) timer_enable(ch->periph);

	pwm_align_mode[idx] = align_mode;
	return BRIDGE_HW_OK;
}

int bridge_hw_pwm_single_pulse(uint8_t channel, uint32_t pulse_ns)
{
	if (channel >= PWM_CHANNEL_COUNT) return BRIDGE_HW_ERR_RANGE;

	const gd32_pwm_ch_t *ch = &pwm_channels[channel];

	/* One-pulse mode is edge-aligned only.  The ARR math below assumes
	 * an up-counter (period == ARR+1 ticks); on a center-aligned timer
	 * (bridge_hw_pwm_configure) the counter runs 0->ARR->0 with an
	 * update event at BOTH crest and trough, so OPM would halt at the
	 * wrong point and the emitted pulse width would be neither
	 * `pulse_us` nor even deterministic (and the output can freeze
	 * HIGH at the halt).  Refuse rather than fire a wrong-width pulse;
	 * the host must set align_mode back to edge (0) first. */
	if (pwm_align_mode[pwm_timer_index(ch->periph)] != 0u) return BRIDGE_HW_ERR_NOTIMPL;

	const uint32_t pulse_us = pulse_ns / PWM_TIMER_TICK_NS;
	if (pulse_us == 0u) return BRIDGE_HW_ERR_RANGE;
	/* ARR is the FULL one-shot window now (lead-in + pulse) and it is a
     * 16-bit register, so pulse_us itself must fit: the widest one-shot
     * this timer can emit is PWM_TIMER_ARR_MAX us.  The old code clamped
     * silently to PWM_TIMER_ARR_MAX + 1 and still answered STATUS_OK --
     * success for a pulse it did not emit.  Refuse instead: a caller
     * asking for 70 ms needs to be told, not to find out on a scope. */
	if (pulse_us > PWM_TIMER_ARR_MAX) return BRIDGE_HW_ERR_RANGE;

	/* Halt the counter before reprogramming.  A prior one-shot already
     * left CEN clear, but a running continuous PWM has not, and
     * rewriting OCxM/ARR/CHxCV underneath a live counter can emit a runt
     * edge on the pad on the way into the one-shot. */
	timer_disable(ch->periph);

	/* PWM1 -- inactive while CNT < CHxCV -- is what makes this a PULSE
     * rather than a latch; see the PWM_ONE_SHOT_DELAY_TICKS comment above
     * for the full argument.  Order is load-bearing: mode and values are
     * programmed while the counter is stopped, SPM is armed, and only
     * then is the counter released. */
	timer_channel_output_mode_config(ch->periph, ch->channel, TIMER_OC_MODE_PWM1);
	timer_counter_value_config(ch->periph, 0u);
	timer_autoreload_value_config(ch->periph, pulse_us);
	timer_channel_output_pulse_value_config(ch->periph, ch->channel, PWM_ONE_SHOT_DELAY_TICKS);
	pwm_one_shot[channel] = true;

	/* SPM is timer-wide (TIMERx_CTL0.SPM), not per-channel, so this also
     * arms every sibling channel on this timer for the same one-shot
     * halt -- a running sibling gets silently re-perioded and stopped.
     * Pre-existing, out of scope here; tracked as #87. */
	timer_single_pulse_mode_config(ch->periph, TIMER_SP_MODE_SINGLE);
	timer_enable(ch->periph);

	/* A follow-up bridge_hw_pwm_get reports the PULSE width, not the
     * lead-in: it inverts its duty arithmetic while pwm_one_shot[] is set
     * for the channel. */
	return BRIDGE_HW_OK;
}
