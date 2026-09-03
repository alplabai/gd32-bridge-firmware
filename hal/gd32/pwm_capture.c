/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 bridge HAL backend -- PWM input capture.
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
/* v0.5 (§2B.2) -- advanced timer extras                             */
/* ----------------------------------------------------------------- */

/* Per-channel input-capture state.  Polled model -- the bridge
 * dispatcher runs single-threaded between transport ISRs so the
 * host's bridge_hw_pwm_capture_read poll-loop naturally drains the
 * capture register.  Each entry caches the most-recent (period,
 * pulse-width) pair latched from the timer's CCxVAL register.
 *
 * V2N-specific caveat: the PWM map binds the COMPLEMENTARY output
 * pad (CHxN) of each advanced-timer channel.  The TIx input-capture
 * path reads the MAIN CHx pad, which is a physically different pin
 * on the GD32G5x3.  BEGIN below switches the channel from output
 * mode to input-capture mode -- the CHCTL field is rewritten so the
 * complementary output stops driving -- but the capture pad routing
 * has to be reworked by the maintainer in a hardware-bring-up commit
 * before READ delivers real edges.  Until then the firmware
 * structure (config + polled drain + correct unit conversion) is
 * exercised end-to-end and READ surfaces BRIDGE_HW_ERR_NOTIMPL
 * ("ring empty") as documented. */
typedef struct {
	uint32_t last_tick;         /* most-recent CCxVAL                  */
	uint32_t period_ticks;      /* drained delta -- same-edge to same  */
	uint32_t pulse_width_ticks; /* rising-to-falling delta             */
	uint8_t  edge;              /* 0 rising / 1 falling / 2 both       */
	uint8_t  state;             /* 0 waiting first / 1 have-rising /
                                   2 have-falling                       */
	bool     in_capture;        /* true between BEGIN..END             */
	bool     have_period;       /* a period sample is ready to drain   */
	bool     have_pulse;        /* a pulse-width sample is ready       */
} pwm_capture_state_t;

static pwm_capture_state_t pwm_capture[PWM_CHANNEL_COUNT];

/* The capture UNIT for a PWM channel.  Every E1M PWM rides an MCH
 * (complementary) pad, and on the GD32G5 the classic CHx input stage
 * (CIx) samples the CHx PRIMARY pin -- a different package pad
 * entirely.  Capturing what arrives on the PWM pad therefore needs
 * the MCH capture unit (MCHCTL0/1 + MCHxCV + MCHxIF), which the SPL
 * addresses with the TIMER_MCH_x channel ids (0x10 | index).  Routing
 * capture through TIMER_CH_x was the silicon-caught 2026-06-04 bug:
 * the loopback jumper drove the MCH pad while CIx listened on an
 * unconnected pin -- zero edges, forever. */
static uint16_t pwm_capture_unit(const gd32_pwm_ch_t *ch)
{
	return ch->complement ? (uint16_t)(TIMER_MCH_0 + ch->channel) : ch->channel;
}

/* Capture/compare interrupt flag for a classic OR multi-mode channel
 * unit id.  Mirrors timer.h's flag macros but exposed here so the
 * polled drain can pick the right one without a switch. */
static uint32_t pwm_capture_flag(uint16_t unit)
{
	switch (unit) {
	case TIMER_CH_0:
		return TIMER_FLAG_CH0;
	case TIMER_CH_1:
		return TIMER_FLAG_CH1;
	case TIMER_CH_2:
		return TIMER_FLAG_CH2;
	case TIMER_CH_3:
		return TIMER_FLAG_CH3;
	case TIMER_MCH_0:
		return TIMER_FLAG_MCH0;
	case TIMER_MCH_1:
		return TIMER_FLAG_MCH1;
	case TIMER_MCH_2:
		return TIMER_FLAG_MCH2;
	case TIMER_MCH_3:
		return TIMER_FLAG_MCH3;
	default:
		return 0u;
	}
}

/* Per-timer confirmed-ACTIVE reload mirror.  Index 0 = TIMER0
 * (PWM0..3), index 1 = TIMER7 (PWM4..7) -- same mapping as pwm.c's
 * private pwm_timer_index, duplicated here rather than shared because
 * this file only ever needs it for TIMER0/TIMER7 via pwm_channels.
 *
 * bridge_hw_pwm_set writes TIMERx_CAR through the ARSE shadow: the
 * write lands in the PRELOAD register only, and a raw TIMER_CAR read
 * always returns that preload content, never the separate,
 * host-inaccessible active copy the counter is actually comparing
 * against.  A first cut of this mirror (#82 review, first round)
 * re-read TIMER_CAR once TIMER_FLAG_UP confirmed a promotion had
 * happened -- but a second, un-promoted write can land in preload
 * between the promotion and the poll, so that raw re-read silently
 * commits the WRONG (not-yet-promoted) value (#82 review, second
 * round).  Registers alone can't disambiguate this: once a write
 * overwrites preload, the identity of what an earlier UPIF actually
 * promoted is gone.  So this mirror is now driven entirely by two
 * SOFTWARE-tracked values pwm.c hands over explicitly --
 * pwm_capture_pending_car (a write not yet known to be promoted) and
 * this array (the write last PROVEN promoted) -- rather than
 * reconstructed from the register after the fact.  See
 * pwm_car_shadow_defer / pwm_car_shadow_commit / pwm_capture_consume_
 * update below.  Boot default matches pwm_timer_init's initial ARR
 * (PWM_TIMER_ARR_MAX) before any PWM_SET has run. */
static uint32_t pwm_capture_active_car[2] = { PWM_TIMER_ARR_MAX, PWM_TIMER_ARR_MAX };

/* Per-timer outstanding write: the CAR value pwm_car_shadow_defer
 * last handed the ARSE preload without forcing its promotion.  The
 * preload register only ever holds ONE value at a time (a later
 * defer before this one promotes simply supersedes it, which is
 * correct -- hardware will promote whichever value was preload-
 * resident at its own next update event), so a single pending slot
 * per timer is sufficient; _valid distinguishes "nothing outstanding"
 * from a legitimately-zero CAR. */
static uint32_t pwm_capture_pending_car[2];
static bool     pwm_capture_pending_valid[2];

/* TIMER base -> pwm_capture_active_car / pending index. */
static uint8_t pwm_capture_timer_index(uint32_t periph)
{
	return (periph == TIMER0) ? 0u : 1u;
}

/* Consume ONE outstanding update event for periph's timer, if
 * TIMER_FLAG_UP evidences one landed since the last time this ran.
 * A single update event can promote at most one outstanding pending
 * write (only the latest write is ever preload-resident -- see the
 * pending-CAR comment above), so on a hit this commits pending_car
 * into the active mirror when there IS one, and always clears the
 * flag regardless -- a steady period with no intervening write still
 * fires UPIF and must not leave it sticky for the next observer.
 *
 * Called from BOTH sides that can otherwise lose track of which
 * write an update event belongs to: the write side
 * (pwm_car_shadow_defer, before it overwrites the pending slot with a
 * newer value) and the read side (pwm_capture_drain).  Draining the
 * flag at every point that could stomp the evidence is what closes
 * the #82-review race the raw-CAR-re-read design had.  Nothing else
 * in this backend reads or clears TIMER_FLAG_UP -- confirmed by grep
 * across hal/ and src/ -- so polling and clearing it here (and in
 * pwm_car_shadow_commit below) is safe. */
static void pwm_capture_consume_update(uint32_t periph)
{
	const uint8_t idx = pwm_capture_timer_index(periph);
	if (RESET == timer_flag_get(periph, TIMER_FLAG_UP)) return;
	timer_flag_clear(periph, TIMER_FLAG_UP);
	if (pwm_capture_pending_valid[idx]) {
		pwm_capture_active_car[idx]    = pwm_capture_pending_car[idx];
		pwm_capture_pending_valid[idx] = false;
	}
}

void pwm_car_shadow_defer(uint32_t periph, uint32_t car)
{
	const uint8_t idx = pwm_capture_timer_index(periph);
	/* Settle any earlier pending write FIRST: if it was already
	 * promoted (UPIF set), this is the last chance to commit it to
	 * the active mirror before its record is overwritten below. */
	pwm_capture_consume_update(periph);
	pwm_capture_pending_car[idx]   = car;
	pwm_capture_pending_valid[idx] = true;
}

void pwm_car_shadow_commit(uint32_t periph, uint32_t car)
{
	const uint8_t idx = pwm_capture_timer_index(periph);
	/* This call's forced UPG (bridge_hw_pwm_set's halted-recovery
	 * path, or bridge_hw_pwm_single_pulse) promotes `car` to active
	 * SYNCHRONOUSLY -- no need to consult or wait on TIMER_FLAG_UP,
	 * and any earlier still-pending write is superseded outright, not
	 * merely raced with. Clear the flag this write also sets so a
	 * later observer doesn't misread it as evidence about a write
	 * that no longer exists. */
	timer_flag_clear(periph, TIMER_FLAG_UP);
	pwm_capture_active_car[idx]    = car;
	pwm_capture_pending_valid[idx] = false;
}

/* Drain any newly-latched capture from the timer's CCxVAL into the
 * per-channel state.  Polled from bridge_hw_pwm_capture_read; safe to
 * call when no edge has occurred (clears nothing, leaves state). */
static void pwm_capture_drain(uint8_t channel)
{
	pwm_capture_state_t *s    = &pwm_capture[channel];
	const gd32_pwm_ch_t *ch   = &pwm_channels[channel];
	const uint16_t       unit = pwm_capture_unit(ch);
	const uint32_t       flag = pwm_capture_flag(unit);
	if (flag == 0u) return;
	if (RESET == timer_flag_get(ch->periph, flag)) return;

	const uint32_t now = timer_channel_capture_value_register_read(ch->periph, unit);
	timer_flag_clear(ch->periph, flag);
	pwm_capture_consume_update(ch->periph);

	/* WRAP-AWARE edge delta.  The capture counter is the timer's own
     * up-counter (0..CAR), so consecutive edges straddle the wrap and a
     * raw `now - last` underflows to a huge value.  Take the delta
     * modulo the counter period (CAR + 1) so it is always the true tick
     * span.  This matters most when the capture timer is SHARED with
     * the stimulus (the Tier-B PWM->PWM loopback: ch2 stimulus + ch3
     * capture both on TIMER0): there the period (same-edge delta) is
     * exactly one wrap and reads ~0 -- a documented degeneracy the host
     * must not assert -- while the pulse width (adjacent-edge delta)
     * stays meaningful.
     *
     * VALIDITY BOUND: the modulo arithmetic is single-wrap.  A true
     * inter-edge span of >= (CAR + 1) ticks -- a captured signal slower
     * than the capture timer's configured period, or an edge the
     * hardware overwrote because the drain polled too slowly -- ALIASES
     * to its remainder with no detection (the per-unit overcapture
     * flags, CHxOF/MCHxOF, are not read in this revision).  Callers
     * must keep the captured signal's edge spacing under the timer
     * period; BEGIN inherits whatever CAR the last pwm_set programmed
     * (boot default 65536 ticks = 65.5 ms at the 1 us tick).
     *
     * The modulus comes from pwm_capture_active_car, the software-
     * tracked PROVEN-promoted mirror, not a raw TIMER_CAR read -- see
     * the comment above that array for why re-reading the register
     * after the fact is ambiguous (#82 review). */
	const uint32_t mod   = pwm_capture_active_car[pwm_capture_timer_index(ch->periph)] + 1u;
	const uint32_t delta = (now + mod - (s->last_tick % mod)) % mod;

	if (s->edge == 2u) {
		/* Both-edge polarity.  Alternate the state machine: the
         * first edge after BEGIN seeds last_tick; the second is an
         * adjacent-edge delta (pulse_width); the third closes the
         * period.  After the third edge the cycle repeats.  Correct
         * pulse-width measurement requires the host to poll fast
         * enough to catch ADJACENT edges -- a slow poll samples
         * non-consecutive edges and the delta is meaningless. */
		switch (s->state) {
		case 0u: /* first edge -- seed */
			s->last_tick = now;
			s->state     = 1u;
			break;
		case 1u: /* second edge -- pulse_width sample */
			s->pulse_width_ticks = delta;
			s->have_pulse        = true;
			s->last_tick         = now;
			s->state             = 2u;
			break;
		case 2u: /* third edge -- period closer */
		default:
			s->period_ticks = delta + s->pulse_width_ticks;
			s->have_period  = true;
			s->last_tick    = now;
			s->state        = 1u;
			break;
		}
	} else {
		/* Single-edge polarity (rising OR falling).  Delta between
         * consecutive same-edge captures is the period; pulse_width
         * is not observable in this mode -- left at zero. */
		if (s->state == 0u) {
			s->last_tick = now;
			s->state     = 1u;
		} else {
			s->period_ticks = delta;
			s->have_period  = true;
			s->last_tick    = now;
		}
	}
}

int bridge_hw_pwm_capture_begin(uint8_t channel, uint8_t edge)
{
	if (channel >= PWM_CHANNEL_COUNT) return BRIDGE_HW_ERR_RANGE;
	if (edge > 2u) return BRIDGE_HW_ERR_INVAL;

	const gd32_pwm_ch_t *ch   = &pwm_channels[channel];
	const uint16_t       unit = pwm_capture_unit(ch);

	/* Stop the pad driving: the complementary output (CHxNEN) is what
     * feeds the MCH pad in COMPLEMENTARY mode.  The classic CHx output
     * unit keeps running for the other channels of this timer. */
	timer_channel_complementary_output_state_config(ch->periph, ch->channel, TIMER_CCXN_DISABLE);

	/* Free the MCH pad from the CHx output unit: in the reset-default
     * COMPLEMENTARY mode the MCH pin is slaved to CHx and its own
     * capture stage never sees the pad.  INDEPENDENT mode hands the
     * pin to the MCH unit (input capture lives there).  capture_end
     * restores COMPLEMENTARY. */
	if (ch->complement) {
		timer_multi_mode_channel_mode_config(ch->periph, unit, TIMER_MCH_MODE_INDEPENDENTLY);
	}

	/* Switch the pad's GPIO mode to AF input (pull-up keeps a
     * disconnected line at a defined level).  The pad still uses the
     * timer's AF -- on the GD32G5 the same AF carries both directions
     * for a given (port, pin) so we don't touch gpio_af_set. */
	gpio_mode_set(ch->gpio_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, ch->gpio_pin);

	/* Configure the input-capture parameters ON THE MCH UNIT (the SPL
     * routes TIMER_MCH_x ids to MCHCTL0/1 + MCHxEN; see
     * pwm_capture_unit's rationale).  Filter=0 (no debounce cycles) --
     * the host can ask for a fixed filter via a follow-up opcode if a
     * glitchy source needs it.  Prescaler=DIV1 captures every selected
     * edge. */
	timer_ic_parameter_struct ic;
	timer_channel_input_struct_para_init(&ic);
	ic.icpolarity  = (edge == 0u)   ? TIMER_IC_POLARITY_RISING
	                 : (edge == 1u) ? TIMER_IC_POLARITY_FALLING
	                                : TIMER_IC_POLARITY_BOTH_EDGE;
	ic.icselection = TIMER_IC_SELECTION_DIRECTTI;
	ic.icprescaler = TIMER_IC_PSC_DIV1;
	ic.icfilter    = 0u;
	timer_input_capture_config(ch->periph, unit, &ic);

	/* Clear any stale capture flag so the first drained edge after
     * BEGIN is genuinely the first new edge. */
	timer_flag_clear(ch->periph, pwm_capture_flag(unit));

	pwm_capture[channel].last_tick         = 0u;
	pwm_capture[channel].period_ticks      = 0u;
	pwm_capture[channel].pulse_width_ticks = 0u;
	pwm_capture[channel].edge              = edge;
	pwm_capture[channel].state             = 0u;
	pwm_capture[channel].in_capture        = true;
	pwm_capture[channel].have_period       = false;
	pwm_capture[channel].have_pulse        = false;
	return BRIDGE_HW_OK;
}

int bridge_hw_pwm_capture_read(uint8_t channel, uint32_t *period_ns, uint32_t *pulse_width_ns)
{
	if (period_ns == 0 || pulse_width_ns == 0) return BRIDGE_HW_ERR_INVAL;
	*period_ns      = 0u;
	*pulse_width_ns = 0u;
	if (channel >= PWM_CHANNEL_COUNT) return BRIDGE_HW_ERR_RANGE;

	pwm_capture_state_t *s = &pwm_capture[channel];
	if (!s->in_capture) return BRIDGE_HW_ERR_INVAL;

	/* Sample the timer's CCxVAL if a new edge has landed since the
     * last poll.  The drain updates `have_period` / `have_pulse`
     * when enough edges have arrived to compose a full tuple. */
	pwm_capture_drain(channel);
	if (!s->have_period) return BRIDGE_HW_ERR_NOTIMPL; /* ring empty */

	/* Convert ticks back to nanoseconds.  The PWM timers run at the
     * 1 us tick configured by pwm_timer_init (prescaler 216-1 against
     * the 216 MHz timer clock) -- NOT the unscaled ~4.63 ns core-clock
     * LSB the free-running counter peripheral uses; bridge_hw.h's
     * capture contract documents the same 1 us resolution. */
	*period_ns      = s->period_ticks * PWM_TIMER_TICK_NS;
	*pulse_width_ns = s->pulse_width_ticks * PWM_TIMER_TICK_NS;

	/* Mark the period drained.  pulse_width is consumed alongside
     * because a subsequent READ should not surface stale data; the
     * next pair of edges populates both fresh. */
	s->have_period = false;
	s->have_pulse  = false;
	return BRIDGE_HW_OK;
}

int bridge_hw_pwm_capture_end(uint8_t channel)
{
	if (channel >= PWM_CHANNEL_COUNT) return BRIDGE_HW_ERR_RANGE;

	const gd32_pwm_ch_t *ch   = &pwm_channels[channel];
	const uint16_t       unit = pwm_capture_unit(ch);
	timer_flag_clear(ch->periph, pwm_capture_flag(unit));

	/* FULL output-stage restore (silicon lesson 2026-06-04): BEGIN
     * disabled the complementary drive, handed the MCH pad to its own
     * capture unit (INDEPENDENT mode, MCHxMS=input, MCHxEN armed) and
     * flipped the pad to INPUT+PULLUP.  The old teardown left all of
     * that in place on the assumption that "bridge_hw_pwm_set
     * re-configures on the next call" -- it does NOT (pwm_set only
     * writes CAR + CHxCV), so one capture session left that PWM
     * channel output-dead until reboot: the HiL soak's capture row
     * (bridge ch2) silently killed PWM2 every run, caught by the
     * Tier-B loopback's zero-edge capture.  Restore mirrors boot:
     * the MCH unit back to compare mode + COMPLEMENTARY slaving
     * (timer_multi_mode_channel_output_config clears MCHxMS, drops
     * MCHxEN, and re-selects the mode in CTL2), the pad back to
     * AF-output push-pull, then pwm_channel_init() re-programs the
     * classic unit to PWM0-output / complementary-enabled / 0 duty. */
	if (ch->complement) {
		timer_omc_parameter_struct omc;
		timer_multi_mode_channel_output_parameter_struct_init(&omc);
		omc.outputmode  = TIMER_MCH_MODE_COMPLEMENTARY;
		omc.outputstate = TIMER_MCCX_DISABLE; /* pad drive comes via CHxNEN */
		omc.ocpolarity  = TIMER_OMC_POLARITY_HIGH;
		timer_multi_mode_channel_output_config(ch->periph, unit, &omc);
	}
	gpio_mode_set(ch->gpio_port, GPIO_MODE_AF, GPIO_PUPD_NONE, ch->gpio_pin);
	gpio_output_options_set(ch->gpio_port, GPIO_OTYPE_PP, GPIO_OSPEED_12MHZ, ch->gpio_pin);
	gpio_af_set(ch->gpio_port, ch->gpio_af, ch->gpio_pin);
	pwm_channel_init(ch);

	pwm_capture[channel].in_capture = false;
	return BRIDGE_HW_OK;
}
