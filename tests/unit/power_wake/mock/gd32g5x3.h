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
#define GPIOA                    0x40u
#define GPIO_PIN_8               0x100u
#define EXTI5_9_IRQn             23
#define I2C0_EV_WKUP_IRQn        31
#define I2C0_ER_IRQn             32
#define FWDGT_STAT_PUD           0x1u
#define FWDGT_STAT_RUD           0x2u
#define FWDGT_STAT_WUD           0x4u
#define SCB_SCR_SLEEPDEEP_Msk    0x4u
#define SCB_ICSR_PENDSTCLR_Msk   0x02000000u
#define SysTick_CTRL_ENABLE_Msk  0x1u
#define SysTick_CTRL_TICKINT_Msk 0x2u

typedef struct {
	uint32_t SCR;
	uint32_t ICSR;
} mock_scb_t;

typedef struct {
	uint32_t CTRL;
	uint32_t VAL;
} mock_systick_t;

#define FMC_OBCTL_FWDGSPD_STDBY 0x00040000u /* BIT(18): FWDGT keeps counting in Standby */
#define FMC_OBCTL_FWDGSPD_DPSLP 0x00020000u /* BIT(17): FWDGT keeps counting in Deep-sleep */
#define I2C_FLAG_I2CBSY         0x8000u

extern uint32_t   FMC_OBCTL;
extern uint32_t   EXTI_PD0;
extern uint32_t   EXTI_PD1;
extern uint32_t   FWDGT_STAT;
extern mock_scb_t mock_scb;
#define SCB (&mock_scb)
extern mock_systick_t mock_systick;
#define SysTick (&mock_systick)

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
/* EXTI_PD0/1 are rc_w1 on silicon: the pending-clear write bridge_power_tick()
 * issues just before its __DSB() has landed by the time it re-reads them, so
 * model that here (a plain variable would keep the 1s and abort the entry). */
static inline void __DSB(void)
{
	EXTI_PD0 = 0u;
	EXTI_PD1 = 0u;
}

/* Clock-tree registers for bridge_clock_restore_after_deepsleep() (gh#12).
 * Plain variables; the test decides which status bits "hardware" reports. */
#define RCU_CTL_PLLEN      0x01000000u
#define RCU_CTL_PLLSTB     0x02000000u
#define RCU_CFG0_SCS       0x3u
#define RCU_CFG0_SCSS      0xCu
#define RCU_CKSYSSRC_PLLP  0x3u
#define RCU_CKSYSSRC_IRC8M 0x0u
#define RCU_SCSS_PLLP      0xCu
#define FMC_WS_WSCNT       0xFu
#define WS_WSCNT(regval)   ((uint32_t)(regval))
extern uint32_t FMC_WS, RCU_CTL, RCU_CFG0;
extern uint32_t mock_system_core_clock_updates;
extern uint32_t SystemCoreClock;
void            SystemCoreClockUpdate(void);

extern uint32_t mock_power_hw_calls;
/* fwdgt_counter_reload() calls, SysTick->CTRL as seen by pmu_to_standbymode(),
 * and the ota_trial_unconfirmed() answer the mock returns. */
extern uint32_t mock_fwdgt_feeds;
extern uint32_t mock_systick_ctrl_at_standby;
extern int      mock_trial_unconfirmed;
/* Deep-sleep entry observations (mode 2). */
extern uint32_t mock_deepsleep_entries, mock_i2c_disables, mock_i2c_inits,
    mock_primask_at_deepsleep;
extern FlagStatus mock_i2c_busy;
/* Race injection (gh#12 review): CS pin level, per-IRQ NVIC pending bits, and
 * hooks fired inside the pre-lock settle window (fwdgt_counter_reload) and
 * inside i2c_disable(), so a test can land an event exactly there.  Ordering
 * stamps record the order of the wake-path steps; counters record I2C
 * re-enables, RTC timer stops and pending-flag clears. */
extern FlagStatus mock_cs_level;
extern uint64_t   mock_nvic_pending;
extern void (*mock_on_settle)(void);
extern void (*mock_on_i2c_disable)(void);
extern uint32_t mock_i2c_enables, mock_rtc_disables, mock_rtc_flag_clears, mock_exti19_clears;
extern uint32_t mock_seq, mock_seq_clock_restore, mock_seq_i2c_init, mock_seq_rtc_disable;
extern int      mock_i2c_init_rc;
void            i2c_enable(uint32_t periph);
/* gh#257: 0 = rcu_rtc_clock_config() never ran, 1 = every call ran with
 * interrupts masked, -1 = some call ran unmasked.  Never reset: the call
 * is latched inside power.c, so it happens once per process. */
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
FlagStatus i2c_flag_get(uint32_t periph, uint32_t flag);
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
