/* SPDX-License-Identifier: Apache-2.0 */

#include "gd32g5x3.h"

#include <stddef.h>
#include <stdbool.h>

#include "bridge_hw.h"

uint32_t       mock_power_hw_calls;
uint32_t       FMC_OBCTL;
uint32_t       mock_fwdgt_feeds;
uint32_t       mock_systick_ctrl_at_standby;
int            mock_trial_unconfirmed;
uint32_t       EXTI_PD0;
uint32_t       EXTI_PD1;
uint32_t       FWDGT_STAT;
uint32_t       FMC_WS, RCU_CTL, RCU_CFG0;
uint32_t       mock_system_core_clock_updates;
uint32_t       SystemCoreClock;
uint32_t       bridge_core_clock_hz;
bool           bridge_core_clock_matches;
mock_scb_t     mock_scb;
mock_systick_t mock_systick;
uint32_t       mock_primask;
uint32_t       mock_deepsleep_entries, mock_i2c_disables, mock_i2c_inits, mock_primask_at_deepsleep;
FlagStatus     mock_i2c_busy;
FlagStatus     mock_cs_level = SET;
uint64_t       mock_nvic_pending;
void (*mock_on_settle)(void);
void (*mock_on_i2c_disable)(void);
uint32_t   mock_i2c_enables, mock_rtc_disables, mock_rtc_flag_clears, mock_exti19_clears;
uint32_t   mock_seq, mock_seq_clock_restore, mock_seq_i2c_init, mock_seq_rtc_disable;
int        mock_i2c_init_rc;
int        mock_rtc_clock_config_saw_irq_masked;
mock_dwt_t mock_dwt;
int        mock_act_adc, mock_act_pwm, mock_act_dac, mock_act_ota;
uint32_t   mock_pd0_on_wake;
uint32_t   mock_exti31_enables, mock_exti31_disables, mock_wake_mode_sets, mock_i2c_init_wake_mode;
int        mock_i2c_wake_mode;
uint32_t   mock_i2cm_wakes;
uint32_t   mock_i2cm_busy;

bool bridge_hw_i2cm_busy(void)
{
	return mock_i2cm_busy;
}

void bridge_hw_i2cm_wake(void)
{
	mock_i2cm_wakes++;
}

void mock_power_reset(void)
{
	mock_power_hw_calls          = 0u;
	mock_primask                 = 0u;
	FMC_OBCTL                    = 0u;
	mock_fwdgt_feeds             = 0u;
	mock_systick_ctrl_at_standby = 0u;
	mock_trial_unconfirmed       = 0;
	mock_systick.CTRL            = 0u;
	mock_systick.LOAD            = 0u;
	mock_systick.VAL             = 0u;
	mock_scb.SCR                 = 0u;
	mock_scb.ICSR                = 0u;
	mock_deepsleep_entries       = 0u;
	mock_i2c_disables            = 0u;
	mock_i2c_inits               = 0u;
	mock_primask_at_deepsleep    = 0u;
	mock_i2c_busy                = RESET;
	mock_i2cm_busy               = 0u;
	mock_cs_level                = SET;
	mock_nvic_pending            = 0u;
	mock_on_settle               = NULL;
	mock_on_i2c_disable          = NULL;
	mock_i2c_enables = mock_rtc_disables = mock_rtc_flag_clears = mock_exti19_clears = 0u;
	mock_seq = mock_seq_clock_restore = mock_seq_i2c_init = mock_seq_rtc_disable = 0u;
	mock_i2c_init_rc                                                             = BRIDGE_HW_OK;
	mock_i2cm_wakes                                                              = 0u;
	EXTI_PD0                                                                     = 0u;
	EXTI_PD1                                                                     = 0u;
	mock_act_adc = mock_act_pwm = mock_act_dac = mock_act_ota = 0;
	mock_pd0_on_wake                                          = 0u;
	mock_exti31_disables                                      = 0u;
	mock_exti31_enables = mock_wake_mode_sets = mock_i2c_init_wake_mode = 0u;
	mock_i2c_wake_mode                                                  = 0;
	mock_dwt.CYCCNT                                                     = 0u;
}

void rcu_osci_on(uint32_t osci)
{
	(void)osci;
	++mock_power_hw_calls;
}

FlagStatus rcu_flag_get(uint32_t flag)
{
	(void)flag;
	++mock_power_hw_calls;
	return SET;
}

void rcu_periph_clock_enable(uint32_t periph)
{
	(void)periph;
	++mock_power_hw_calls;
}

void pmu_backup_write_enable(void)
{
	++mock_power_hw_calls;
}

void rcu_rtc_clock_config(uint32_t source)
{
	(void)source;
	/* Sticky: rtc_wakeup_init_once() latches, so only the FIRST arm in the
	 * process reaches here whatever the case order.  1 = every call so far
	 * ran masked, -1 = some call did not. */
	if (mock_primask == 0u) {
		mock_rtc_clock_config_saw_irq_masked = -1;
	} else if (mock_rtc_clock_config_saw_irq_masked == 0) {
		mock_rtc_clock_config_saw_irq_masked = 1;
	}
	++mock_power_hw_calls;
}

ErrStatus rtc_wakeup_disable(void)
{
	++mock_rtc_disables;
	mock_seq_rtc_disable = ++mock_seq;
	++mock_power_hw_calls;
	return SUCCESS;
}

ErrStatus rtc_wakeup_clock_set(uint32_t source)
{
	(void)source;
	++mock_power_hw_calls;
	return SUCCESS;
}

ErrStatus rtc_wakeup_timer_set(uint16_t count)
{
	(void)count;
	++mock_power_hw_calls;
	return SUCCESS;
}

void rtc_wakeup_enable(void)
{
	++mock_power_hw_calls;
}

void i2c_disable(uint32_t periph)
{
	(void)periph;
	++mock_i2c_disables;
	++mock_power_hw_calls;
	if (mock_on_i2c_disable != NULL) mock_on_i2c_disable();
}

void i2c_enable(uint32_t periph)
{
	(void)periph;
	++mock_i2c_enables;
	++mock_power_hw_calls;
}

void pmu_to_deepsleepmode(uint32_t ldo, uint32_t command)
{
	(void)ldo;
	(void)command;
	++mock_deepsleep_entries;
	EXTI_PD0 |= mock_pd0_on_wake;
	mock_dwt.CYCCNT += 100u; /* time spent "asleep" */
	mock_primask_at_deepsleep = mock_primask;
	++mock_power_hw_calls;
}

void pmu_to_standbymode(void)
{
	mock_systick_ctrl_at_standby = mock_systick.CTRL;
	++mock_power_hw_calls;
}

void rtc_flag_clear(uint32_t flag)
{
	(void)flag;
	++mock_rtc_flag_clears;
	++mock_power_hw_calls;
}

void rtc_interrupt_enable(uint32_t interrupt)
{
	(void)interrupt;
	++mock_power_hw_calls;
}

void exti_flag_clear(uint32_t linex)
{
	if (linex == EXTI_19) ++mock_exti19_clears;
	++mock_power_hw_calls;
}

void exti_init(uint32_t linex, uint32_t mode, uint32_t trig_type)
{
	if (linex == EXTI_31) ++mock_exti31_enables;
	(void)mode;
	(void)trig_type;
	++mock_power_hw_calls;
}

void exti_interrupt_flag_clear(uint32_t linex)
{
	(void)linex;
	++mock_power_hw_calls;
}

void nvic_irq_enable(int32_t nvic_irq, uint8_t pre_priority, uint8_t sub_priority)
{
	(void)nvic_irq;
	(void)pre_priority;
	(void)sub_priority;
	++mock_power_hw_calls;
}

FlagStatus i2c_flag_get(uint32_t periph, uint32_t flag)
{
	(void)periph;
	(void)flag;
	return mock_i2c_busy;
}

int bridge_transport_i2c_hw_init(void)
{
	++mock_i2c_inits;
	mock_i2c_init_wake_mode = (uint32_t)mock_i2c_wake_mode;
	mock_dwt.CYCCNT += 50u;
	mock_seq_i2c_init = ++mock_seq;
	++mock_power_hw_calls;
	return mock_i2c_init_rc;
}

void fwdgt_counter_reload(void)
{
	++mock_fwdgt_feeds;
	++mock_power_hw_calls;
	if (mock_on_settle != NULL) mock_on_settle();
}

bool ota_trial_unconfirmed(void)
{
	return mock_trial_unconfirmed != 0;
}

FlagStatus gpio_input_bit_get(uint32_t gpio_periph, uint32_t pin)
{
	(void)gpio_periph;
	(void)pin;
	return mock_cs_level;
}

uint32_t NVIC_GetPendingIRQ(int32_t irqn)
{
	return (uint32_t)((mock_nvic_pending >> (uint32_t)irqn) & 1u);
}

void SystemCoreClockUpdate(void)
{
	mock_seq_clock_restore = ++mock_seq;
	++mock_system_core_clock_updates;
	mock_dwt.CYCCNT += 10u;
}

void exti_interrupt_disable(uint32_t linex)
{
	if (linex == EXTI_31) ++mock_exti31_disables;
	++mock_power_hw_calls;
}

void bridge_transport_i2c_wake_mode_set(bool wake)
{
	mock_i2c_wake_mode = wake;
	++mock_wake_mode_sets;
}

bool bridge_transport_i2c_wake_mode(void)
{
	return mock_i2c_wake_mode != 0;
}

bool bridge_adc_streams_active(void)
{
	return mock_act_adc != 0;
}

bool bridge_pwm_claims_active(void)
{
	return mock_act_pwm != 0;
}

bool bridge_dac_driven(void)
{
	return mock_act_dac != 0;
}

bool ota_session_active(void)
{
	return mock_act_ota != 0;
}
