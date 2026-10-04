/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Register-level host double for hal/gd32/clock_hw.c.  The RCU / FMC / SYSCFG
 * registers are reached through accessor functions, so every STATEMENT that
 * touches one lets mock_hw.c diff the register file: the order in which clock_hw.c
 * writes bits is recorded as event tokens, and the hardware reacts (HXTALSTB,
 * PLLSTB, SCSS follow, with simulated time).  Field positions and values are
 * the vendor's (gd32g5x3_rcu.h / _syscfg.h); only the surface clock_hw.c uses
 * is present.
 */
#ifndef GD32_BRIDGE_CLOCK_HW_MOCK_GD32G5X3_H
#define GD32_BRIDGE_CLOCK_HW_MOCK_GD32G5X3_H

#include <stdbool.h>
#include <stdint.h>

#define BIT(x)     ((uint32_t)1U << (x))
#define BITS(s, e) ((uint32_t)(0xFFFFFFFFU << (s)) & (0xFFFFFFFFU >> (31U - (uint32_t)(e))))

/* ---- registers (accessors; see mock_hw.c) ---- */
volatile uint32_t *mock_reg_rcu_ctl(void);
volatile uint32_t *mock_reg_rcu_cfg0(void);
volatile uint32_t *mock_reg_rcu_pll(void);
volatile uint32_t *mock_reg_rcu_int(void);
volatile uint32_t *mock_reg_fmc_ws(void);
volatile uint32_t *mock_reg_syscfg_stat(void);
volatile uint32_t *mock_reg_rtc_bkp9(void);
volatile uint32_t *mock_reg32(uint32_t addr);

#define RCU_CTL     (*mock_reg_rcu_ctl())
#define RCU_CFG0    (*mock_reg_rcu_cfg0())
#define RCU_PLL     (*mock_reg_rcu_pll())
#define RCU_INT     (*mock_reg_rcu_int())
#define FMC_WS      (*mock_reg_fmc_ws())
#define SYSCFG_STAT (*mock_reg_syscfg_stat())
#define RTC_BKP9    (*mock_reg_rtc_bkp9())
#define REG32(addr) (*mock_reg32((uint32_t)(addr)))

#define RCU_CTL_IRC8MEN  BIT(0)
#define RCU_CTL_HXTALEN  BIT(16)
#define RCU_CTL_HXTALSTB BIT(17)
#define RCU_CTL_HXTALBPS BIT(18)
#define RCU_CTL_CKMEN    BIT(19)
#define RCU_CTL_PLLEN    BIT(24)
#define RCU_CTL_PLLSTB   BIT(25)

#define RCU_PLL_PLLPSC BITS(0, 3)
#define RCU_PLL_PLLN   BITS(6, 13)
#define RCU_PLL_PLLP   BITS(16, 17)
#define RCU_PLL_PLLPEN BIT(19)
#define RCU_PLL_PLLQEN BIT(20)
#define RCU_PLL_PLLREN BIT(21)
#define RCU_PLL_PLLSEL BIT(22)
#define RCU_PLL_PLLQ   BITS(23, 26)
#define RCU_PLL_PLLR   BITS(27, 31)

#define RCU_PLLSRC_IRC8M ((uint32_t)0x00000000U)
#define RCU_PLLSRC_HXTAL RCU_PLL_PLLSEL

#define RCU_CFG0_SCS     BITS(0, 1)
#define RCU_CFG0_SCSS    BITS(2, 3)
#define RCU_CFG0_AHBPSC  BITS(4, 7)
#define RCU_CFG0_APB1PSC BITS(10, 12)

#define RCU_CKSYSSRC_IRC8M ((uint32_t)0U << 0)
#define RCU_CKSYSSRC_PLLP  ((uint32_t)3U << 0)
#define RCU_SCSS_IRC8M     ((uint32_t)0U << 2)
#define RCU_SCSS_PLLP      ((uint32_t)3U << 2)
#define RCU_AHB_CKSYS_DIV1 ((uint32_t)0U << 4)
#define RCU_AHB_CKSYS_DIV2 ((uint32_t)8U << 4)
#define RCU_AHB_CKSYS_DIV4 ((uint32_t)9U << 4)

#define RCU_INT_CKMIC BIT(23)
#define RCU_INT_CKMIF BIT(7)

#define SYSCFG_STAT_SRAM0ECCMEIF   BIT(0)
#define SYSCFG_STAT_SRAM0ECCSEIF   BIT(1)
#define SYSCFG_STAT_FLASHECCIF     BIT(2)
#define SYSCFG_STAT_CKMNMIIF       BIT(3)
#define SYSCFG_STAT_NMIPINIF       BIT(4)
#define SYSCFG_STAT_SRAM1ECCMEIF   BIT(5)
#define SYSCFG_STAT_SRAM1ECCSEIF   BIT(6)
#define SYSCFG_STAT_TCMSRAMECCMEIF BIT(7)
#define SYSCFG_STAT_TCMSRAMECCSEIF BIT(8)

#define FMC_WS_WSCNT     BITS(0, 3)
#define WS_WSCNT(regval) ((uint32_t)(regval))

/* ---- core ---- */
typedef struct {
	uint32_t CTRL, LOAD, VAL;
} mock_systick_t;
extern mock_systick_t mock_systick;
#define SysTick                 (&mock_systick)
#define SysTick_LOAD_RELOAD_Msk 0x00FFFFFFu
extern uint32_t SystemCoreClock;

extern uint32_t        mock_primask;
static inline uint32_t __get_PRIMASK(void)
{
	return mock_primask;
}
void               mock_set_primask(uint32_t v);
static inline void __disable_irq(void)
{
	mock_set_primask(1u);
}
static inline void __set_PRIMASK(uint32_t primask)
{
	mock_set_primask(primask);
}

/* clock_hw.c's cycle-counter seam: each read advances simulated time. */
uint32_t mock_cyccnt(void);
void     mock_dwt_ensure(void);
#define BRIDGE_CYCCNT()     mock_cyccnt()
#define BRIDGE_DWT_ENSURE() mock_dwt_ensure()

void fwdgt_counter_reload(void);

/* ---- TIMER14 / TRIGSEL / SYSCFG routing used by the frequency check ---- */
#define TIMER14                        ((uint32_t)0x40014000u)
#define RCU_TIMER14                    ((uint32_t)2u)
#define RCU_TRIGSEL                    ((uint32_t)1u)
#define TRIGSEL_OUTPUT_TIMER14_ITI14   ((uint32_t)0x88u)
#define TRIGSEL_INPUT_HXTAL_DIV32_TRIG ((uint32_t)0xA3u)
#define TRIGSEL_INPUT_0                ((uint32_t)0x00u)
#define TIMER_SMCFG_TRGSEL_ITI14       ((uint32_t)0x13u)
#define TIMER_SLAVE_MODE_EXTERNAL0     ((uint32_t)0x06u)
#define SYSCFG_TIMER14                 ((uint8_t)0x06U)
#define SYSCFG_TIMERCFG(syscfg_timerx) ((uint32_t)(0x40010000u + 0x100U + (syscfg_timerx) * 0x0CU))

void     rcu_periph_clock_enable(uint32_t periph);
void     rcu_periph_clock_disable(uint32_t periph);
void     trigsel_init(uint32_t target, uint32_t source);
void     timer_deinit(uint32_t timer);
void     timer_input_trigger_source_select(uint32_t timer, uint32_t intrigger);
void     timer_slave_mode_select(uint32_t timer, uint32_t slavemode);
void     timer_counter_value_config(uint32_t timer, uint32_t counter);
void     timer_enable(uint32_t timer);
void     timer_disable(uint32_t timer);
uint32_t timer_counter_read(uint32_t timer);

#endif /* GD32_BRIDGE_CLOCK_HW_MOCK_GD32G5X3_H */
