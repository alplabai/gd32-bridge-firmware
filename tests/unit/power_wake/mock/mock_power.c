/* SPDX-License-Identifier: Apache-2.0 */

#include "gd32g5x3.h"

#include "bridge_hw.h"

uint32_t mock_power_hw_calls;

void mock_power_reset(void)
{
	mock_power_hw_calls = 0u;
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

void pmu_wakeup_pin_enable(uint32_t pin)
{
	(void)pin;
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

int bridge_transport_i2c_hw_init(void)
{
	++mock_power_hw_calls;
	return BRIDGE_HW_OK;
}
