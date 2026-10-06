/* SPDX-License-Identifier: Apache-2.0
 *
 * Scenario + observation surface of the clock_hw.c register simulation
 * (mock_hw.c).  The simulation is the hardware: tests choose what is wired
 * (the OSCIN source, whether PLLs lock, how fast the IRC8M is) and read back
 * the registers, the time spent, and the ORDER in which bits were written.
 */
#ifndef GD32_BRIDGE_CLOCK_HW_MOCK_HW_H
#define GD32_BRIDGE_CLOCK_HW_MOCK_HW_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
	uint32_t oscin_hz;       /* what the SE2 net carries; 0 = SE2 off */
	bool     require_bypass; /* a driven clock only stabilises with HXTALBPS set */
	uint32_t stb_delay_us;   /* HXTALEN -> HXTALSTB */
	uint32_t lock_us;        /* PLLEN -> PLLSTB */
	bool     pll_locks_irc8m;
	bool     pll_locks_hxtal;
	bool     scs_refuses_irc8m;          /* SCSS never leaves the PLL (a wedged switch) */
	int      irc8m_err_permille;         /* IRC8M (and its PLL) fast by this much */
	uint32_t step_cycles;                /* core cycles that elapse per time/cycle-counter read */
	bool     trial;                      /* ota_trial_unconfirmed() */
	bool     pwm_active;                 /* pwm_any_claimed() */
	bool     adc_active;                 /* adc_stream_any_active() */
	bool     link_busy;                  /* bridge_link_quiet() == false */
	bool     cyccnt_stalled;             /* DWT->CYCCNT does not advance */
	uint32_t timer_read_overhead_cycles; /* core cycles burnt inside timer_counter_read() */
} mock_scn_t;

extern mock_scn_t mock_scn;

/* Boot state: the vendor SystemInit() result (IRC8M PLL 216 MHz, AHB /1). */
void mock_hw_reset(void);

uint64_t mock_now_ns(void);
uint32_t mock_sim_sysclk_hz(void); /* the simulated hardware's SYSCLK, independent of clock_hw.c */

/* The event log: space separated tokens in write order (see mock_hw.c). */
const char *mock_log_text(void);
void        mock_log_clear(void);
bool        mock_log_has(const char *token);
/* true when the NULL-terminated tokens appear in this order. */
bool mock_in_order(const char *first, ...);

extern uint32_t mock_fwdgt_feeds;
extern uint32_t mock_primask_max_ns; /* longest stretch spent with PRIMASK set */

/* Simulated hardware state the tests inspect. */
uint32_t mock_timer14_clk_enabled(void);
uint32_t mock_trigsel_target_source(void); /* source routed to TIMER14_ITI14 */
uint32_t mock_syscfg_timer14_cfg(void);    /* OR of the three SYSCFG TIMER14 config words */
void     mock_set_stat_flags(uint32_t flags);
void     mock_hw_ckm_fire(void);
void     mock_hw_deepsleep(void);
uint32_t mock_stat_flags(void);
uint32_t mock_timer14_car(void);

#endif
