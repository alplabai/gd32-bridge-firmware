/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/ztest.h>

#include <stdbool.h>
#include <stdint.h>

#include "gd32g5x3.h"
#include "pwm_preload_transaction.h"

uint32_t mock_timer_ctl0[2];
uint32_t mock_primask;
uint32_t mock_disable_calls;
uint32_t mock_set_primask_calls;

typedef struct {
	uint32_t ctl0_seen;
	uint32_t primask_seen;
} callback_observation_t;

/* Small timer/capture-mirror model for the race #177 found.  Its only
 * hardware rule is the relevant one: a natural update promotes both preload
 * registers unless CTL0.UPDIS is set. */
typedef struct {
	uint32_t active_car;
	uint32_t active_cmp;
	uint32_t preload_car;
	uint32_t preload_cmp;
	uint32_t mirror_active_car;
	uint32_t mirror_pending_car;
	bool     mirror_pending_valid;
	bool     upif;
	uint32_t promotions;
} pwm_model_t;

static void reset_mock(void)
{
	mock_timer_ctl0[0]     = 0u;
	mock_timer_ctl0[1]     = 0u;
	mock_primask           = 0u;
	mock_disable_calls     = 0u;
	mock_set_primask_calls = 0u;
}

static void model_natural_update(pwm_model_t *model)
{
	if ((TIMER_CTL0(TIMER0) & TIMER_CTL0_UPDIS) != 0u) return;
	model->active_car = model->preload_car;
	model->active_cmp = model->preload_cmp;
	model->upif       = true;
	model->promotions++;
}

static void model_capture_consume_update(pwm_model_t *model)
{
	if (!model->upif) return;
	model->upif = false;
	if (model->mirror_pending_valid) {
		model->mirror_active_car    = model->mirror_pending_car;
		model->mirror_pending_valid = false;
	}
}

/* The production callback has this exact CAR -> CHxCV -> defer order.  Try a
 * natural update after each adjacent operation; with the real transaction it
 * must never promote a partial pair. */
static void model_preload_callback(void *context)
{
	pwm_model_t *model = context;

	model->preload_car = 47u;
	model_natural_update(model);
	model->preload_cmp = 13u;
	model_natural_update(model);
	/* pwm_car_shadow_defer first settles a prior UPIF, then publishes B. */
	model_capture_consume_update(model);
	model->mirror_pending_car   = 47u;
	model->mirror_pending_valid = true;
	model_natural_update(model);
}

/* Stand-in for the running PWM_SET callback: it observes the guard then
 * changes timer state just as a future preload helper could. */
static void preload_callback(void *context)
{
	callback_observation_t *observation = context;
	observation->ctl0_seen              = TIMER_CTL0(TIMER0);
	observation->primask_seen           = mock_primask;

	/* Model legitimate callback-owned CTL0 changes.  The guard must not
	 * restore the earlier word and accidentally bring CEN back. */
	TIMER_CTL0(TIMER0) &= ~(uint32_t)TIMER_CTL0_CEN;
	TIMER_CTL0(TIMER0) |= (uint32_t)TIMER_CTL0_SPM;
}

ZTEST(pwm_preload_transaction, test_disables_updates_and_preserves_other_ctl0_changes)
{
	callback_observation_t observation = { 0 };

	reset_mock();
	mock_timer_ctl0[0] = TIMER_CTL0_CEN;
	pwm_preload_transaction(TIMER0, preload_callback, &observation);

	zassert_true((observation.ctl0_seen & TIMER_CTL0_UPDIS) != 0u,
	             "callback must run while natural updates are disabled");
	zassert_equal(
	    observation.primask_seen, 1u, "transport ISR cannot interleave the shadow handoff");
	zassert_equal(
	    mock_timer_ctl0[0], TIMER_CTL0_SPM, "restore only UPDIS; retain callback CTL0 changes");
	zassert_equal(mock_primask, 0u, "restore the caller's interrupt state");
	zassert_equal(mock_disable_calls, 1u);
	zassert_equal(mock_set_primask_calls, 1u);
}

ZTEST(pwm_preload_transaction, test_preserves_callers_existing_update_disable_policy)
{
	callback_observation_t observation = { 0 };

	reset_mock();
	mock_timer_ctl0[0] = TIMER_CTL0_CEN | TIMER_CTL0_UPDIS;
	pwm_preload_transaction(TIMER0, preload_callback, &observation);

	zassert_true((mock_timer_ctl0[0] & TIMER_CTL0_UPDIS) != 0u,
	             "caller-owned UPDIS must remain asserted");
	zassert_true((mock_timer_ctl0[0] & TIMER_CTL0_SPM) != 0u,
	             "callback CTL0 bits must survive restoration");
	zassert_true((mock_timer_ctl0[0] & TIMER_CTL0_CEN) == 0u,
	             "restoration must not revive stale CEN");
}

ZTEST(pwm_preload_transaction, test_blocks_each_injected_update_until_pair_and_mirror_are_pending)
{
	pwm_model_t model = {
		.active_car           = 31u,
		.active_cmp           = 7u,
		.preload_car          = 31u,
		.preload_cmp          = 7u,
		.mirror_active_car    = 31u,
		.mirror_pending_car   = 0u,
		.mirror_pending_valid = false,
	};

	reset_mock();
	mock_timer_ctl0[0] = TIMER_CTL0_CEN;
	pwm_preload_transaction(TIMER0, model_preload_callback, &model);

	zassert_equal(
	    model.promotions, 0u, "every injected update inside the guarded callback is suppressed");
	zassert_equal(model.active_car, 31u, "new CAR cannot promote before its compare");
	zassert_equal(model.active_cmp, 7u, "old compare remains active with old CAR");
	zassert_equal(model.mirror_active_car, 31u, "mirror stays with the active pair");
	zassert_true(model.mirror_pending_valid, "coherent new CAR is published as pending");

	/* Once UPDIS is restored, the next natural update promotes the COMPLETE
	 * pair, and the capture-side consume observes exactly that CAR. */
	model_natural_update(&model);
	model_capture_consume_update(&model);
	zassert_equal(model.promotions, 1u);
	zassert_equal(model.active_car, 47u);
	zassert_equal(model.active_cmp, 13u);
	zassert_equal(model.mirror_active_car, 47u);
	zassert_false(model.mirror_pending_valid);
}

ZTEST(pwm_preload_transaction, test_model_would_detect_the_unguarded_torn_pair_mutation)
{
	pwm_model_t model = {
		.active_car  = 31u,
		.active_cmp  = 7u,
		.preload_car = 31u,
		.preload_cmp = 7u,
	};

	reset_mock();
	/* This is the pre-fix CAR write followed by the injected update, with no
	 * UPDIS transaction.  It proves the injection model is sensitive to the
	 * precise torn-pair bug the guarded production path prevents. */
	model.preload_car = 47u;
	model_natural_update(&model);
	zassert_equal(model.active_car, 47u);
	zassert_equal(model.active_cmp, 7u);
}

ZTEST_SUITE(pwm_preload_transaction, NULL, NULL, NULL, NULL, NULL);
