/* SPDX-License-Identifier: Apache-2.0
 *
 * Minimal host double for the vendor PMU/RTC surface used by power.c.
 * Every function increments mock_power_hw_calls so rejection tests can
 * prove validation happens before any hardware side effect.
 */
#ifndef GD32_BRIDGE_POWER_WAKE_MOCK_GD32G5X3_H
#define GD32_BRIDGE_POWER_WAKE_MOCK_GD32G5X3_H

#include <stdint.h>

typedef enum { RESET = 0, SET = !RESET } FlagStatus;
typedef enum { ERROR = 0, SUCCESS = !ERROR } ErrStatus;

#define RCU_IRC32K         1u
#define RCU_FLAG_IRC32KSTB 2u
#define RCU_PMU            3u
#define RCU_RTCSRC_IRC32K  4u
#define RCU_RTC            5u
#define WAKEUP_RTCCK_DIV16 6u
#define I2C0               20u
#define PMU_LDO_LOWPOWER   30u
#define WFI_CMD            31u
#define RTC_FLAG_WT        40u
#define RTC_INT_WAKEUP     41u
#define EXTI_19            50u
#define EXTI_INTERRUPT     51u
#define EXTI_TRIG_RISING   52u
#define RTC_WKUP_IRQn      3

/* Symbols reached only by bridge_power_tick()'s deferred standby entry
 * (gh#63).  Plain storage, not counted: this suite pins what the ISR-side
 * bridge_hw_power_mode_set() does, not the base-level entry gate. */
#define GPIOA                 0x40u
#define GPIO_PIN_8            0x100u
#define EXTI5_9_IRQn          23
#define I2C0_EV_WKUP_IRQn     31
#define I2C0_ER_IRQn          32
#define FWDGT_STAT_PUD        0x1u
#define FWDGT_STAT_RUD        0x2u
#define FWDGT_STAT_WUD        0x4u
#define SCB_SCR_SLEEPDEEP_Msk 0x4u

typedef struct {
	uint32_t SCR;
} mock_scb_t;

extern uint32_t   EXTI_PD0;
extern uint32_t   EXTI_PD1;
extern uint32_t   FWDGT_STAT;
extern mock_scb_t mock_scb;
#define SCB (&mock_scb)

/* CMSIS PRIMASK intrinsics reached through hal/gd32/bridge_critical.h.
 * Real (if trivial) tracking via mock_primask -- gh#257's test needs to
 * observe whether a given mock hardware call landed inside a
 * bridge_irq_lock()/bridge_irq_unlock() section. */
extern uint32_t mock_primask;

static inline uint32_t __get_PRIMASK(void)
{
	return mock_primask;
}
static inline void __disable_irq(void)
{
	mock_primask = 1u;
}
static inline void __set_PRIMASK(uint32_t primask)
{
	mock_primask = primask;
}
static inline void __DSB(void)
{
}

extern uint32_t mock_power_hw_calls;
/* gh#257: true iff mock_primask was set (interrupts masked) the last
 * time rcu_rtc_clock_config() ran. */
extern int mock_rtc_clock_config_saw_irq_masked;

void mock_power_reset(void);

void       rcu_osci_on(uint32_t osci);
FlagStatus rcu_flag_get(uint32_t flag);
void       rcu_periph_clock_enable(uint32_t periph);
void       pmu_backup_write_enable(void);
void       rcu_rtc_clock_config(uint32_t source);
ErrStatus  rtc_wakeup_disable(void);
ErrStatus  rtc_wakeup_clock_set(uint32_t source);
ErrStatus  rtc_wakeup_timer_set(uint16_t count);
void       rtc_wakeup_enable(void);
void       i2c_disable(uint32_t periph);
void       pmu_to_deepsleepmode(uint32_t ldo, uint32_t command);
void       pmu_to_standbymode(void);
void       rtc_flag_clear(uint32_t flag);
void       rtc_interrupt_enable(uint32_t interrupt);
void       exti_flag_clear(uint32_t linex);
void       exti_init(uint32_t linex, uint32_t mode, uint32_t trig_type);
void       exti_interrupt_flag_clear(uint32_t linex);
void       nvic_irq_enable(int32_t nvic_irq, uint8_t pre_priority, uint8_t sub_priority);
void       fwdgt_counter_reload(void);
FlagStatus gpio_input_bit_get(uint32_t gpio_periph, uint32_t pin);
uint32_t   NVIC_GetPendingIRQ(int32_t irqn);

#endif /* GD32_BRIDGE_POWER_WAKE_MOCK_GD32G5X3_H */
