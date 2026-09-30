/* SPDX-License-Identifier: Apache-2.0 */

#include "gd32g5x3.h"

#include "bridge_hw.h"

uint32_t   mock_power_hw_calls;
uint32_t   EXTI_PD0;
uint32_t   EXTI_PD1;
uint32_t   FWDGT_STAT;
mock_scb_t mock_scb;
uint32_t   mock_primask;
int        mock_rtc_clock_config_saw_irq_masked;

void mock_power_reset(void)
{
	mock_power_hw_calls                  = 0u;
	mock_primask                         = 0u;
	mock_rtc_clock_config_saw_irq_masked = 0;
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
	mock_rtc_clock_config_saw_irq_masked = (mock_primask != 0u);
	++mock_power_hw_calls;
}

ErrStatus rtc_wakeup_disable(void)
{
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
	++mock_power_hw_calls;
}

void pmu_to_deepsleepmode(uint32_t ldo, uint32_t command)
{
	(void)ldo;
	(void)command;
	++mock_power_hw_calls;
}

void pmu_to_standbymode(void)
{
	++mock_power_hw_calls;
}

void rtc_flag_clear(uint32_t flag)
{
	(void)flag;
	++mock_power_hw_calls;
}

void rtc_interrupt_enable(uint32_t interrupt)
{
	(void)interrupt;
	++mock_power_hw_calls;
}

void exti_flag_clear(uint32_t linex)
{
	(void)linex;
	++mock_power_hw_calls;
}

void exti_init(uint32_t linex, uint32_t mode, uint32_t trig_type)
{
	(void)linex;
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

int bridge_transport_i2c_hw_init(void)
{
	++mock_power_hw_calls;
	return BRIDGE_HW_OK;
}

void fwdgt_counter_reload(void)
{
	++mock_power_hw_calls;
}

FlagStatus gpio_input_bit_get(uint32_t gpio_periph, uint32_t pin)
{
	(void)gpio_periph;
	(void)pin;
	return SET;
}

uint32_t NVIC_GetPendingIRQ(int32_t irqn)
{
	(void)irqn;
	return 0u;
}
