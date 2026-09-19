/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Production-linked host tests for bridge_hw_timer_sync().  The tiny mock
 * header supplies only the four GigaDevice SPL calls this HAL body uses;
 * every call is logged here so rejection paths prove that no timer or
 * SYSCFG state was touched.
 */

#include <zephyr/ztest.h>

#include <stddef.h>
#include <string.h>

#include "bridge_hw.h"
#include "gd32g5x3.h"
#include "timer_sync_master.h"

enum mock_call_kind {
	MOCK_MASTER_TRIGGER,
	MOCK_MASTER_SLAVE,
	MOCK_INPUT_TRIGGER,
	MOCK_SLAVE_MODE,
};

typedef struct {
	enum mock_call_kind kind;
	uint32_t            periph;
	uint32_t            arg;
} mock_call_t;

static mock_call_t calls[4];
static size_t      call_count;

volatile uint32_t mock_timer_ctl0[2];
volatile uint32_t mock_timer_car[2];
volatile uint32_t mock_timer_cv[2][4];
volatile uint32_t mock_primask;

static unsigned pwm_mutation_calls;
static unsigned pwm_upg_calls;

static void log_call(enum mock_call_kind kind, uint32_t periph, uint32_t arg)
{
	zassert_true(call_count < 4u, "timer_sync emitted too many vendor calls");
	calls[call_count++] = (mock_call_t){ .kind = kind, .periph = periph, .arg = arg };
}

void timer_master_output0_trigger_source_select(uint32_t timer_periph, uint32_t outrigger)
{
	log_call(MOCK_MASTER_TRIGGER, timer_periph, outrigger);
}

void timer_master_slave_mode_config(uint32_t timer_periph, uint32_t masterslave)
{
	log_call(MOCK_MASTER_SLAVE, timer_periph, masterslave);
}

void timer_input_trigger_source_select(uint32_t timer_periph, uint32_t intrigger)
{
	log_call(MOCK_INPUT_TRIGGER, timer_periph, intrigger);
}

void timer_slave_mode_select(uint32_t timer_periph, uint32_t slavemode)
{
	log_call(MOCK_SLAVE_MODE, timer_periph, slavemode);
}

static size_t timer_index(uint32_t periph)
{
	return (periph == TIMER0) ? 0u : 1u;
}

void timer_struct_para_init(timer_parameter_struct *initpara)
{
	memset(initpara, 0, sizeof(*initpara));
}

void timer_deinit(uint32_t timer_periph)
{
	(void)timer_periph;
}

void timer_init(uint32_t timer_periph, const timer_parameter_struct *initpara)
{
	mock_timer_car[timer_index(timer_periph)] = initpara->period;
}

void timer_auto_reload_shadow_enable(uint32_t timer_periph)
{
	(void)timer_periph;
}

void timer_primary_output_config(uint32_t timer_periph, uint32_t newvalue)
{
	(void)timer_periph;
	(void)newvalue;
}

void timer_enable(uint32_t timer_periph)
{
	mock_timer_ctl0[timer_index(timer_periph)] |= TIMER_CTL0_CEN;
	pwm_mutation_calls++;
}

void timer_disable(uint32_t timer_periph)
{
	mock_timer_ctl0[timer_index(timer_periph)] &= ~TIMER_CTL0_CEN;
	pwm_mutation_calls++;
}

void timer_channel_output_struct_para_init(timer_oc_parameter_struct *ocpara)
{
	memset(ocpara, 0, sizeof(*ocpara));
}

void timer_channel_output_config(uint32_t                         timer_periph,
                                 uint16_t                         channel,
                                 const timer_oc_parameter_struct *ocpara)
{
	(void)timer_periph;
	(void)channel;
	(void)ocpara;
}

void timer_channel_output_pulse_value_config(uint32_t timer_periph,
                                             uint16_t channel,
                                             uint32_t pulse)
{
	mock_timer_cv[timer_index(timer_periph)][channel] = pulse;
	pwm_mutation_calls++;
}

void timer_channel_output_mode_config(uint32_t timer_periph, uint16_t channel, uint32_t oc_mode)
{
	(void)timer_periph;
	(void)channel;
	(void)oc_mode;
	pwm_mutation_calls++;
}

void timer_channel_output_shadow_config(uint32_t timer_periph, uint16_t channel, uint32_t oc_shadow)
{
	(void)timer_periph;
	(void)channel;
	(void)oc_shadow;
}

void timer_single_pulse_mode_config(uint32_t timer_periph, uint32_t sp_mode)
{
	(void)timer_periph;
	(void)sp_mode;
	pwm_mutation_calls++;
}

void timer_autoreload_value_config(uint32_t timer_periph, uint32_t autoreload)
{
	mock_timer_car[timer_index(timer_periph)] = autoreload;
	pwm_mutation_calls++;
}

void timer_event_software_generate(uint32_t timer_periph, uint32_t event)
{
	(void)timer_periph;
	zassert_equal(event, TIMER_EVENT_SRC_UPG);
	zassert_equal(mock_primask, 1u, "UPG must execute under the sync-state lock");
	pwm_upg_calls++;
}

void timer_counter_value_config(uint32_t timer_periph, uint32_t counter)
{
	(void)timer_periph;
	(void)counter;
	pwm_mutation_calls++;
}

uint32_t timer_channel_capture_value_register_read(uint32_t timer_periph, uint16_t channel)
{
	return mock_timer_cv[timer_index(timer_periph)][channel];
}

void pwm_car_shadow_defer(uint32_t periph, uint32_t car)
{
	(void)periph;
	(void)car;
}

void pwm_car_shadow_commit(uint32_t periph, uint32_t car)
{
	(void)periph;
	(void)car;
}

static void reset_calls(void)
{
	call_count         = 0u;
	pwm_mutation_calls = 0u;
	pwm_upg_calls      = 0u;
	mock_primask       = 0u;
	memset((void *)mock_timer_ctl0, 0, sizeof(mock_timer_ctl0));
	memset((void *)mock_timer_car, 0, sizeof(mock_timer_car));
	memset((void *)mock_timer_cv, 0, sizeof(mock_timer_cv));
	timer_sync_master_set(0u, false);
	timer_sync_master_set(1u, false);
	zassert_true(timer_sync_forced_update_allowed(0u));
	zassert_true(timer_sync_forced_update_allowed(1u));
}

/* Production-linked regression for #89. The sync route is configured by
 * the real TIMER_SYNC HAL, then the real pwm.c functions must refuse every
 * path that would force UPG while their timer broadcasts TRGO0. */
static void assert_sync_master_guard(uint8_t master, uint8_t slave, uint8_t pwm_channel)
{
	reset_calls();
	zassert_equal(bridge_hw_timer_sync(master, slave, 1u), BRIDGE_HW_OK);
	pwm_mutation_calls = 0u;
	pwm_upg_calls      = 0u;

	zassert_equal(bridge_hw_pwm_set(pwm_channel, 10000u, 5000u), BRIDGE_HW_ERR_BUSY);
	zassert_equal(pwm_mutation_calls, 0u, "refused PWM_SET must not touch timer state");
	zassert_equal(pwm_upg_calls, 0u, "refused PWM_SET must not emit UPG");
	zassert_equal(mock_primask, 0u, "refused PWM_SET must restore PRIMASK");
	zassert_equal(bridge_hw_pwm_single_pulse(pwm_channel, 5000u), BRIDGE_HW_ERR_BUSY);
	zassert_equal(pwm_mutation_calls, 0u, "refused one-shot must not touch timer state");
	zassert_equal(pwm_upg_calls, 0u, "refused one-shot must not emit UPG");
	zassert_equal(mock_primask, 0u, "refused one-shot must restore PRIMASK");

	call_count = 0u; /* the route-call log is not under test in this helper */
	zassert_equal(bridge_hw_timer_sync(master, slave, 0u), BRIDGE_HW_OK);
	pwm_mutation_calls = 0u;
	pwm_upg_calls      = 0u;
	zassert_equal(bridge_hw_pwm_set(pwm_channel, 10000u, 5000u), BRIDGE_HW_OK);
	zassert_equal(pwm_upg_calls, 1u, "disabled route restores halted PWM_SET's UPG");
	zassert_equal(mock_primask, 0u, "allowed PWM_SET must restore PRIMASK");

	call_count = 0u;
	zassert_equal(bridge_hw_timer_sync(master, slave, 1u), BRIDGE_HW_OK);
	mock_timer_ctl0[master] |= TIMER_CTL0_CEN;
	pwm_mutation_calls = 0u;
	pwm_upg_calls      = 0u;
	zassert_equal(bridge_hw_pwm_set(pwm_channel, 10000u, 5000u), BRIDGE_HW_OK);
	zassert_equal(pwm_upg_calls, 0u, "running PWM_SET must not force UPG");
	zassert_equal(mock_primask, 0u, "running PWM_SET must not retain PRIMASK");
}

ZTEST(timer_sync, test_pwm_sync_guard_covers_both_pwm_timers)
{
	assert_sync_master_guard(0u, 1u, 0u);
	assert_sync_master_guard(1u, 0u, 4u);
}

ZTEST(timer_sync, test_compact_ids_configure_timer0_to_timer7)
{
	reset_calls();
	zassert_equal(bridge_hw_timer_sync(0u, 1u, 1u), BRIDGE_HW_OK);
	zassert_equal(call_count, 4u);
	zassert_equal(calls[0].kind, MOCK_MASTER_TRIGGER);
	zassert_equal(calls[0].periph, TIMER0);
	zassert_equal(calls[0].arg, TIMER_TRI_OUT0_SRC_UPDATE);
	zassert_equal(calls[1].kind, MOCK_MASTER_SLAVE);
	zassert_equal(calls[1].periph, TIMER0);
	zassert_equal(calls[1].arg, TIMER_MASTER_SLAVE_MODE_ENABLE);
	zassert_equal(calls[2].kind, MOCK_INPUT_TRIGGER);
	zassert_equal(calls[2].periph, TIMER7);
	zassert_equal(calls[2].arg, 0x01u); /* TIMER7 ITI0 <- TIMER0_TRGO0 */
	zassert_equal(calls[3].kind, MOCK_SLAVE_MODE);
	zassert_equal(calls[3].periph, TIMER7);
	zassert_equal(calls[3].arg, TIMER_SLAVE_MODE_RESTART);
	zassert_true(timer_sync_master_active(0u));
	zassert_false(timer_sync_master_active(1u));
	zassert_false(timer_sync_forced_update_allowed(0u));
	zassert_true(timer_sync_forced_update_allowed(1u));
}

ZTEST(timer_sync, test_disable_writes_disabled_slave_mode)
{
	reset_calls();
	zassert_equal(bridge_hw_timer_sync(1u, 0u, 0u), BRIDGE_HW_OK);
	zassert_equal(call_count, 4u);
	zassert_equal(calls[2].kind, MOCK_INPUT_TRIGGER);
	zassert_equal(calls[2].periph, TIMER0);
	zassert_equal(calls[2].arg, 0x0Au); /* TIMER0 ITI5 <- TIMER7_TRGO0 */
	zassert_equal(calls[3].kind, MOCK_SLAVE_MODE);
	zassert_equal(calls[3].arg, TIMER_SLAVE_MODE_DISABLE);
	zassert_false(timer_sync_master_active(1u));
	zassert_true(timer_sync_forced_update_allowed(1u));
}

ZTEST(timer_sync, test_disabling_one_route_preserves_the_other_master)
{
	reset_calls();
	zassert_equal(bridge_hw_timer_sync(0u, 1u, 1u), BRIDGE_HW_OK);
	call_count = 0u; /* retain the state above; discard its vendor-call log */
	zassert_equal(bridge_hw_timer_sync(1u, 0u, 1u), BRIDGE_HW_OK);
	zassert_true(timer_sync_master_active(0u));
	zassert_true(timer_sync_master_active(1u));

	call_count = 0u; /* same: state persists, call log is per operation */
	zassert_equal(bridge_hw_timer_sync(0u, 1u, 0u), BRIDGE_HW_OK);
	zassert_false(timer_sync_master_active(0u));
	zassert_true(timer_sync_master_active(1u));
	zassert_true(timer_sync_forced_update_allowed(0u));
	zassert_false(timer_sync_forced_update_allowed(1u));
}

ZTEST(timer_sync, test_timer19_id_rejected_before_register_writes)
{
	static const uint8_t pairs[][2] = {
		{ 2u, 0u }, { 0u, 2u }, { 2u, 1u }, { 1u, 2u }, { 2u, 2u },
	};

	for (size_t i = 0u; i < sizeof pairs / sizeof pairs[0]; ++i) {
		reset_calls();
		zassert_equal(bridge_hw_timer_sync(pairs[i][0], pairs[i][1], 1u),
		              BRIDGE_HW_ERR_RANGE,
		              "TIMER19 pair %zu must fail as out of range",
		              i);
		zassert_equal(call_count, 0u, "rejected TIMER19 pair %zu touched hardware", i);
		zassert_false(timer_sync_master_active(0u));
		zassert_false(timer_sync_master_active(1u));
	}
}

ZTEST(timer_sync, test_other_invalid_inputs_fail_before_register_writes)
{
	reset_calls();
	zassert_equal(bridge_hw_timer_sync(0u, 0u, 1u), BRIDGE_HW_ERR_INVAL);
	zassert_equal(call_count, 0u);
	zassert_false(timer_sync_master_active(0u));
	zassert_false(timer_sync_master_active(1u));

	reset_calls();
	zassert_equal(bridge_hw_timer_sync(3u, 0u, 1u), BRIDGE_HW_ERR_RANGE);
	zassert_equal(call_count, 0u);

	reset_calls();
	zassert_equal(bridge_hw_timer_sync(0u, 7u, 1u), BRIDGE_HW_ERR_RANGE);
	zassert_equal(call_count, 0u, "peripheral number 7 is not TIMER7's compact wire id");

	reset_calls();
	zassert_equal(bridge_hw_timer_sync(0u, 1u, 6u), BRIDGE_HW_ERR_INVAL);
	zassert_equal(call_count, 0u);
}

ZTEST_SUITE(timer_sync, NULL, NULL, NULL, NULL, NULL);
