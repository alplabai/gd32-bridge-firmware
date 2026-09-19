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
#define PMU_WAKEUP_PIN0    10u
#define PMU_WAKEUP_PIN1    11u
#define PMU_WAKEUP_PIN2    12u
#define PMU_WAKEUP_PIN3    13u
#define PMU_WAKEUP_PIN4    14u
#define I2C0               20u
#define PMU_LDO_LOWPOWER   30u
#define WFI_CMD            31u

extern uint32_t mock_power_hw_calls;

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
void       pmu_wakeup_pin_enable(uint32_t pin);
void       i2c_disable(uint32_t periph);
void       pmu_to_deepsleepmode(uint32_t ldo, uint32_t command);
void       pmu_to_standbymode(void);

#endif /* GD32_BRIDGE_POWER_WAKE_MOCK_GD32G5X3_H */
