/* SPDX-License-Identifier: Apache-2.0 */

#include "gd32g5x3.h"

#include <stddef.h>
#include <stdbool.h>

#include "bridge_hw.h"
#include "gd32_common.h" /* PWM_TIMER_CLK_HZ */

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
uint32_t mock_i2c_enables, mock_rtc_disables, mock_rtc_flag_clears, mock_exti19_clears;
uint32_t mock_seq, mock_seq_clock_restore, mock_seq_i2c_init, mock_seq_rtc_disable;
int      mock_i2c_init_rc;
int      mock_rtc_clock_config_saw_irq_masked;

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
	mock_cs_level                = SET;
	mock_nvic_pending            = 0u;
	mock_on_settle               = NULL;
	mock_on_i2c_disable          = NULL;
	mock_hxtal_source            = 0;
	mock_hxtal_relock_ok         = 1;
	mock_pre_deepsleeps = mock_relock_prepares = 0u;
	mock_i2c_enables = mock_rtc_disables = mock_rtc_flag_clears = mock_exti19_clears = 0u;
	mock_seq = mock_seq_clock_restore = mock_seq_i2c_init = mock_seq_rtc_disable = 0u;
	mock_i2c_init_rc                                                             = BRIDGE_HW_OK;
	EXTI_PD0                                                                     = 0u;
	EXTI_PD1                                                                     = 0u;
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
	mock_seq_deepsleep        = ++mock_seq;
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

FlagStatus i2c_flag_get(uint32_t periph, uint32_t flag)
{
	(void)periph;
	(void)flag;
	return mock_i2c_busy;
}

int bridge_transport_i2c_hw_init(void)
{
	++mock_i2c_inits;
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
}

/* clock_hw.c is not linked here.  The three hooks power.c calls around the
 * Deep-sleep clock are modelled just far enough to pin the ORDER with the PLLEN
 * replay (see mock_hxtal_* in gd32g5x3.h); with mock_hxtal_source == 0 (the
 * IRC8M default) they are inert, so the replay is what those cases exercise. */
int      mock_hxtal_source;        /* 1 = the PLL was running from HXTAL before the sleep */
int      mock_hxtal_relock_ok = 1; /* the HXTAL restart succeeds (else: fall back to IRC8M) */
uint32_t mock_pre_deepsleeps, mock_relock_prepares;
uint32_t mock_seq_pre_deepsleep, mock_seq_relock_prepare, mock_seq_deepsleep;
uint32_t mock_ctl_pllen_at_relock_prepare;

void bridge_clock_pre_deepsleep(void)
{
	++mock_pre_deepsleeps;
	mock_seq_pre_deepsleep = ++mock_seq;
}

void bridge_clock_relock_prepare(void)
{
	if (!mock_hxtal_source) return;
	++mock_relock_prepares;
	mock_seq_relock_prepare          = ++mock_seq;
	mock_ctl_pllen_at_relock_prepare = RCU_CTL & RCU_CTL_PLLEN;
	/* HXTAL back (PLL re-pointed on it) or fallen back to IRC8M: either way the
	 * PLL is configured and DISABLED when this returns; the replay enables it. */
	(void)mock_hxtal_relock_ok;
}

/* The live-clock derivation is clock_hw.c's; here SystemCoreClock is set by the test. */
void bridge_clock_core_update(void)
{
	SystemCoreClockUpdate();
}

bool bridge_clock_core_matches(void)
{
	return SystemCoreClock == PWM_TIMER_CLK_HZ;
}
