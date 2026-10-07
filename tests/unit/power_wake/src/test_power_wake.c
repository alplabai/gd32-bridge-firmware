/* SPDX-License-Identifier: Apache-2.0
 *
 * Host tests for bridge_hw_power_mode_set(, 0u)'s wake-bitmap gate.  This suite
 * links the real power.c against a minimal PMU/RTC mock: return status and
 * zero hardware calls together prove a rejected request cannot enter a
 * low-power path before replying.
 */

#include <zephyr/ztest.h>

#include "bridge_hw.h"
#include "gd32_common.h"
#include "gd32g5x3.h"
#include "power_policy.h"
#include "power_wake.h"

#include <string.h>

extern void bridge_power_tick(void);

static void expect_rejected_without_hardware(uint32_t wake_bitmap)
{
	mock_power_reset();
	zassert_equal(bridge_hw_power_mode_set(2u, wake_bitmap, 0u, 0u), BRIDGE_HW_ERR_NOTIMPL);
	zassert_equal(mock_power_hw_calls, 0u);
}

ZTEST(power_wake, test_accepts_empty_and_supported_bits)
{
	/* Mode 0 is the side-effect-free way to prove each bitmap clears the
	 * production validation gate. */
	zassert_equal(bridge_hw_power_mode_set(0u, 0u, 0u, 0u), BRIDGE_HW_OK);
	zassert_equal(bridge_hw_power_mode_set(0u, POWER_WAKE_RTC, 0u, 0u), BRIDGE_HW_OK);
	zassert_equal(bridge_hw_power_mode_set(0u, POWER_WAKE_TIMER, 0u, 0u), BRIDGE_HW_OK);
	zassert_equal(bridge_hw_power_mode_set(0u, POWER_WAKE_MASK_SUPPORTED, 0u, 0u), BRIDGE_HW_OK);
}

ZTEST(power_wake, test_rejects_known_unsupported_bits)
{
	expect_rejected_without_hardware(POWER_WAKE_GPIO);
	expect_rejected_without_hardware(POWER_WAKE_UART_RX);
	expect_rejected_without_hardware(POWER_WAKE_USB);
	expect_rejected_without_hardware(POWER_WAKE_ETH_LINK);
}

ZTEST(power_wake, test_rejects_portable_comparator_and_brownout_bits)
{
	/* ALP_POWER_WAKE_COMPARATOR / ALP_POWER_WAKE_BROWNOUT are already
	 * public wire bits, but this GD32 backend has no path to arm them. */
	expect_rejected_without_hardware(0x00000040u);
	expect_rejected_without_hardware(0x00000080u);
}

ZTEST(power_wake, test_rejects_future_and_mixed_bits)
{
	expect_rejected_without_hardware(0x80000000u);
	expect_rejected_without_hardware(POWER_WAKE_RTC | 0x00000100u);
	expect_rejected_without_hardware(UINT32_MAX);
}

ZTEST(power_wake, test_deep_sleep_without_rtc_wake_is_refused)
{
	/* gh#12: the I2C slave has no wake path, so mode 2 must always carry
	 * an RTC timer -- an unbounded sleep for a BRD_I2C-only host would
	 * strand the bridge. */
	mock_power_reset();
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 0u, 0u), BRIDGE_HW_ERR_INVAL);
	zassert_equal(mock_deepsleep_entries, 0u);
}

ZTEST(power_wake, test_deep_sleep_long_wake_refused_when_fwdgt_runs_in_deepsleep)
{
	mock_power_reset();
	FMC_OBCTL = FMC_OBCTL_FWDGSPD_DPSLP;
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 301u, 0u), BRIDGE_HW_ERR_RANGE);
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 300u, 0u), BRIDGE_HW_OK);
	/* The standby bit alone does not restrict deep-sleep. */
	mock_power_reset();
	FMC_OBCTL = FMC_OBCTL_FWDGSPD_STDBY;
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 30000u, 0u), BRIDGE_HW_OK);
}

/* Latch a deep-sleep request and run the base-level entry; the mock's
 * pmu_to_deepsleepmode() returns at once, i.e. an immediate wake. */
ZTEST(power_wake, test_deep_sleep_entry_and_wake_sequence)
{
	mock_power_reset();
	mock_systick.CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
	mock_systick.VAL  = 1234u;
	RCU_CTL           = RCU_CTL_PLLSTB;
	RCU_CFG0          = RCU_SCSS_PLLP;
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, 0u), BRIDGE_HW_OK);
	zassert_equal(mock_deepsleep_entries, 0u, "entry is deferred to the base level");
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 1u);
	zassert_equal(mock_i2c_disables, 1u, "I2C0 off across the entry");
	zassert_equal(mock_primask_at_deepsleep, 1u, "handlers held off until the PLL is back");
	zassert_equal(mock_primask, 0u, "interrupts re-enabled after the wake");
	zassert_equal(mock_system_core_clock_updates > 0u, true, "clock restore ran");
	zassert_equal(mock_i2c_inits, 1u, "I2C re-initialised after the wake");
	zassert_equal(mock_fwdgt_feeds, 1u);
	zassert_equal(mock_systick.CTRL, SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
	zassert_equal(mock_systick.VAL, 0u);
	/* The request is consumed: a second tick is a no-op. */
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 1u);
}

ZTEST(power_wake, test_deep_sleep_waits_for_idle_i2c)
{
	mock_power_reset();
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, 0u), BRIDGE_HW_OK);
	mock_i2c_busy = SET;
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 0u, "no entry mid-I2C-transaction");
	mock_i2c_busy = RESET;
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 1u, "request retained and retried");
}

/* Order + timer-stop assertions for the wake path: PLL relock first, then
 * the I2C re-init, then the RTC timer stop and the WTF / EXTI-19 clears. */
ZTEST(power_wake, test_deep_sleep_wake_order_and_timer_stop)
{
	mock_power_reset();
	RCU_CTL  = RCU_CTL_PLLSTB;
	RCU_CFG0 = RCU_SCSS_PLLP;
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, 0u), BRIDGE_HW_OK);
	uint32_t clears0 = mock_rtc_flag_clears, exti0 = mock_exti19_clears, dis0 = mock_rtc_disables;
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 1u);
	zassert_true(mock_seq_clock_restore != 0u && mock_seq_clock_restore < mock_seq_i2c_init,
	             "PLL relock before the I2C re-init");
	zassert_true(mock_seq_i2c_init < mock_seq_rtc_disable, "I2C re-init before the timer stop");
	zassert_equal(mock_rtc_disables, dis0 + 1u, "RTC wakeup timer stopped");
	zassert_true(mock_rtc_flag_clears > clears0, "WTF cleared after the stop");
	zassert_true(mock_exti19_clears > exti0, "EXTI 19 pending cleared after the stop");
	zassert_equal(mock_i2c_enables, 0u, "no back-out on a clean entry");
}

/* Let a pending relock succeed and its I2C re-init run, so the file-static
 * latch in power.c does not leak into the next test. */
static void drain_pending_relock(void)
{
	mock_i2c_init_rc = BRIDGE_HW_OK;
	RCU_CTL          = RCU_CTL_PLLSTB;
	RCU_CFG0         = RCU_SCSS_PLLP;
	for (unsigned i = 0u; i < 8u; i++)
		bridge_power_tick();
}

/* Relock failure: the part stays on IRC8M, the I2C re-init refuses; the failure is
 * recorded and retried on later ticks, and the request is still consumed. */
ZTEST(power_wake, test_deep_sleep_wake_with_failed_relock_and_i2c_reinit_retries)
{
	extern volatile uint8_t bridge_i2c_reinit_pending;

	mock_power_reset();
	RCU_CTL          = 0u; /* PLLSTB never sets */
	RCU_CFG0         = 0u;
	mock_i2c_init_rc = BRIDGE_HW_ERR_RANGE;
	SystemCoreClock  = 8000000u; /* the live clock after a failed relock */
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, 0u), BRIDGE_HW_OK);
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 1u);
	zassert_false(bridge_core_clock_matches, "telemetry shows the failed relock");
	zassert_equal(bridge_i2c_reinit_pending, 1u, "I2C re-init failure recorded");
	zassert_equal(mock_primask, 0u);

	uint32_t inits = mock_i2c_inits;
	bridge_power_tick();
	zassert_equal(mock_i2c_inits, inits + 1u, "retried on the next tick");
	zassert_equal(bridge_i2c_reinit_pending, 1u, "still failing");
	mock_i2c_init_rc = BRIDGE_HW_OK;
	bridge_power_tick();
	zassert_equal(bridge_i2c_reinit_pending, 0u, "recovered");
	bridge_power_tick();
	zassert_equal(mock_i2c_inits, inits + 2u, "no further retries once recovered");
	drain_pending_relock();
}

/* A wake that never locks latches the relock: retried sparsely while it keeps
 * failing, and once it holds the I2C0 is disabled and re-initialised in the
 * same tick. */
ZTEST(power_wake, test_failed_relock_is_retried_then_arms_i2c_reinit)
{
	extern volatile uint8_t bridge_i2c_reinit_pending;

	mock_power_reset();
	RCU_CTL                        = 0u; /* PLLSTB never sets */
	RCU_CFG0                       = 0u;
	SystemCoreClock                = 8000000u;
	mock_system_core_clock_updates = 0u;
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, 0u), BRIDGE_HW_OK);
	bridge_power_tick();
	zassert_equal(mock_system_core_clock_updates, 1u, "the wake itself relocks once");

	bridge_power_tick();
	zassert_equal(mock_system_core_clock_updates, 2u, "first retry on the next tick");
	for (unsigned i = 0u; i < 7u; i++)
		bridge_power_tick();
	zassert_equal(mock_system_core_clock_updates, 2u, "backed off between retries");
	bridge_power_tick();
	zassert_equal(mock_system_core_clock_updates, 3u, "latched while the PLL stays dead");

	RCU_CTL           = RCU_CTL_PLLSTB;
	RCU_CFG0          = RCU_SCSS_PLLP;
	uint32_t inits    = mock_i2c_inits;
	uint32_t disables = mock_i2c_disables;
	for (unsigned i = 0u; i < 7u; i++)
		bridge_power_tick();
	zassert_equal(mock_system_core_clock_updates, 3u, "still backed off");
	bridge_power_tick();
	zassert_equal(mock_system_core_clock_updates, 4u, "retry succeeds");
	zassert_equal(mock_i2c_disables, disables + 1u, "I2C0 disabled before the re-init");
	zassert_equal(mock_i2c_inits, inits + 1u, "re-init in the same tick");
	zassert_equal(bridge_i2c_reinit_pending, 0u);
	bridge_power_tick();
	zassert_equal(mock_system_core_clock_updates, 4u, "latch cleared");
}

ZTEST(power_wake, test_deep_sleep_rtc_bit_without_time_refused_under_dpslp_cap)
{
	/* RTC bit + wake_after_ms == 0 arms POWER_WAKE_TIMER_MAX_MS, far above the cap. */
	mock_power_reset();
	FMC_OBCTL = FMC_OBCTL_FWDGSPD_DPSLP;
	zassert_equal(bridge_hw_power_mode_set(2u, POWER_WAKE_RTC, 0u, 0u), BRIDGE_HW_ERR_RANGE);
}

static void assert_backed_out_then_retries(uint32_t systick_ctrl, uint32_t expect_i2c_disables)
{
	zassert_equal(mock_deepsleep_entries, 0u, "no deep-sleep entry");
	zassert_equal(mock_i2c_disables, expect_i2c_disables);
	zassert_equal(mock_i2c_enables, expect_i2c_disables, "I2C0 re-enabled on back-out");
	zassert_equal(mock_primask, 0u, "lock released");
	zassert_equal(mock_systick.CTRL, systick_ctrl, "SysTick restored");
	zassert_equal(mock_i2c_inits, 0u, "wake path did not run");

	/* The request is still latched: with the event gone the next tick sleeps. */
	mock_cs_level     = SET;
	mock_nvic_pending = 0u;
	mock_on_settle = mock_on_i2c_disable = NULL;
	RCU_CTL                              = RCU_CTL_PLLSTB;
	RCU_CFG0                             = RCU_SCSS_PLLP;
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 1u, "request retained, retried");
}

static void inject_cs_low(void)
{
	mock_cs_level = RESET;
}

static void inject_i2c_ev_pending(void)
{
	mock_nvic_pending |= (uint64_t)1u << I2C0_EV_WKUP_IRQn;
}

static void inject_exti_pd(void)
{
	EXTI_PD0 = 1u << 8;
}

static void inject_rtc_pending(void)
{
	mock_nvic_pending |= (uint64_t)1u << RTC_WKUP_IRQn;
}

static void inject_i2c_busy(void)
{
	mock_i2c_busy = SET;
}

/* The ~185 us settle window has interrupts enabled: an event landing there
 * (after the entry gates passed) must be caught by the re-check under the
 * lock and abort the entry cleanly. */
ZTEST(power_wake, test_deep_sleep_backs_out_on_event_in_settle_window)
{
	static void (*const injectors[])(
	    void) = { inject_cs_low, inject_i2c_ev_pending, inject_rtc_pending, inject_i2c_busy };
	for (size_t i = 0; i < sizeof(injectors) / sizeof(injectors[0]); ++i) {
		mock_power_reset();
		const uint32_t ctrl = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
		mock_systick.CTRL   = ctrl;
		zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, 0u), BRIDGE_HW_OK);
		mock_on_settle = injectors[i];
		bridge_power_tick();
		/* Caught before I2C0 was ever disabled. */
		zassert_equal(mock_deepsleep_entries, 0u, "injector %u", (unsigned)i);
		zassert_equal(mock_i2c_disables, 0u, "I2C0 untouched, injector %u", (unsigned)i);
		zassert_equal(mock_primask, 0u);
		zassert_equal(mock_systick.CTRL, ctrl, "SysTick restored");
		mock_i2c_busy = RESET;
		assert_backed_out_then_retries(ctrl, 0u);
	}
}

/* An event landing while I2C0 is being disabled (after the lock, after the
 * first re-check) is caught by the second re-check: I2C0 comes back. */
ZTEST(power_wake, test_deep_sleep_backs_out_on_event_during_i2c_disable)
{
	static void (*const injectors[])(
	    void) = { inject_cs_low, inject_i2c_ev_pending, inject_exti_pd, inject_rtc_pending };
	for (size_t i = 0; i < sizeof(injectors) / sizeof(injectors[0]); ++i) {
		mock_power_reset();
		const uint32_t ctrl = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
		mock_systick.CTRL   = ctrl;
		zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, 0u), BRIDGE_HW_OK);
		mock_on_i2c_disable = injectors[i];
		bridge_power_tick();
		EXTI_PD0 = 0u;
		assert_backed_out_then_retries(ctrl, 1u);
	}
}

ZTEST(power_wake, test_standby_without_rtc_wake_is_refused)
{
	/* gh#63 / gh#40: standby with no RTC wake would need NRST to leave. */
	mock_power_reset();
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 0u, 0u), BRIDGE_HW_ERR_INVAL);
}

ZTEST(power_wake, test_rtc_clock_config_runs_masked)
{
	/* gh#257: rcu_rtc_clock_config() is a bare RCU_BDCTL read-modify-
	 * write reached at runtime (first armed standby request), with
	 * interrupts live -- it must run inside a
	 * bridge_irq_lock()/bridge_irq_unlock() section, same exposure
	 * class as the RCU_*EN writes bridge_rcu_periph_clock_enable()
	 * already protects. */
	mock_power_reset();
	zassert_equal(bridge_hw_power_mode_set(3u, POWER_WAKE_RTC, 0u, 0u), BRIDGE_HW_OK);
	zassert_equal(mock_rtc_clock_config_saw_irq_masked, 1, "rcu_rtc_clock_config() ran unmasked");
	/* The section must also restore: no net mask left behind. */
	zassert_equal(mock_primask, 0u);
}

ZTEST(power_wake, test_run_and_sleep_reject_nonzero_wake_after_ms)
{
	/* gh#261: modes 0 (run) and 1 (sleep) arm no wake source at all, so a
	 * non-zero wake_after_ms must be refused rather than answering
	 * STATUS_OK for a timer that was never armed -- and refused before any
	 * hardware touch, same fail-closed shape as the bitmap gate (#107). */
	mock_power_reset();
	zassert_equal(bridge_hw_power_mode_set(0u, 0u, 1u, 0u), BRIDGE_HW_ERR_INVAL);
	zassert_equal(mock_power_hw_calls, 0u);

	mock_power_reset();
	zassert_equal(bridge_hw_power_mode_set(1u, 0u, 500u, 0u), BRIDGE_HW_ERR_INVAL);
	zassert_equal(mock_power_hw_calls, 0u);

	/* wake_after_ms == 0 is still the accepted no-op for both modes. */
	mock_power_reset();
	zassert_equal(bridge_hw_power_mode_set(0u, 0u, 0u, 0u), BRIDGE_HW_OK);
	zassert_equal(bridge_hw_power_mode_set(1u, 0u, 0u, 0u), BRIDGE_HW_OK);
}

ZTEST(power_wake, test_standby_long_wake_refused_when_fwdgt_runs_in_standby)
{
	/* FWDGSPD_STDBY = 1: the ~501 ms FWDGT would reset the part out of a
	 * longer standby, so refuse before any hardware touch. */
	mock_power_reset();
	FMC_OBCTL = FMC_OBCTL_FWDGSPD_STDBY;
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 301u, 0u), BRIDGE_HW_ERR_RANGE);
	zassert_equal(bridge_hw_power_mode_set(3u, POWER_WAKE_RTC, 0u, 0u), BRIDGE_HW_ERR_RANGE);
	zassert_equal(mock_power_hw_calls, 2u, "one PMU clock enable per call, nothing armed");
	/* A wake inside the window is still accepted. */
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 300u, 0u), BRIDGE_HW_OK);
}

ZTEST(power_wake, test_standby_long_wake_accepted_when_fwdgt_frozen_in_standby)
{
	mock_power_reset();
	FMC_OBCTL = 0u;
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 30000u, 0u), BRIDGE_HW_OK);
}

/* Latch a standby request and run the base-level entry; the mock's
 * pmu_to_standbymode() returns, i.e. the entry ABORTS. */
static void run_aborted_standby(uint32_t systick_ctrl)
{
	mock_power_reset();
	mock_systick.CTRL = systick_ctrl;
	mock_systick.VAL  = 1234u;
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 100u, 0u), BRIDGE_HW_OK);
	bridge_power_tick();
}

ZTEST(power_wake, test_standby_abort_restores_systick)
{
	run_aborted_standby(SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
	zassert_equal(mock_systick_ctrl_at_standby, 0u, "SysTick must be stopped across the entry");
	zassert_equal(mock_systick.CTRL,
	              SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk,
	              "CTRL restored on abort");
	zassert_equal(mock_systick.VAL, 0u, "period restarted on abort");
	zassert_equal(mock_scb.ICSR, SCB_ICSR_PENDSTCLR_Msk, "pending tick cleared");
	zassert_equal(mock_scb.SCR & SCB_SCR_SLEEPDEEP_Msk, 0u);
}

ZTEST(power_wake, test_standby_entry_feeds_only_outside_a_trial)
{
	run_aborted_standby(0u);
	zassert_equal(mock_fwdgt_feeds, 1u, "normal boot: feed before the entry");

	mock_power_reset();
	mock_trial_unconfirmed = 1;
	/* A pending trial commit now refuses the entry outright (BUSY); the
	 * defensive no-feed guard still covers a trial that starts after the
	 * request was latched. */
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 100u, 0u), BRIDGE_HW_ERR_BUSY);
	mock_trial_unconfirmed = 0;
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 100u, 0u), BRIDGE_HW_OK);
	mock_trial_unconfirmed = 1;
	bridge_power_tick();
	zassert_equal(mock_fwdgt_feeds, 0u, "TRIAL: the counter is the bootloader's revert dog");
}

/* gh#12: Deep-sleep exit leaves CK_SYS on IRC8M; the restore must raise the
 * wait states, lock the PLL, select PLLP and re-derive SystemCoreClock.  The
 * mock "hardware" reports PLLSTB and SCSS=PLLP as already set. */
ZTEST(power_wake, test_clock_restore_selects_pll_and_updates_core_clock)
{
	mock_power_reset();
	FMC_WS                         = 0u;
	RCU_CTL                        = RCU_CTL_PLLSTB;
	mock_system_core_clock_updates = 0u;
	SystemCoreClock                = PWM_TIMER_CLK_HZ;
	/* SCSS reads back the selected source on silicon; model that. */
	RCU_CFG0 = RCU_SCSS_PLLP;

	zassert_true(bridge_clock_restore_after_deepsleep());
	zassert_equal(FMC_WS & FMC_WS_WSCNT, 7u, "wait states left at 7 for the 216 MHz clock");
	zassert_true((RCU_CTL & RCU_CTL_PLLEN) != 0u, "PLL re-enabled");
	zassert_equal(RCU_CFG0 & RCU_CFG0_SCS, RCU_CKSYSSRC_PLLP, "PLLP selected");
	zassert_equal(mock_system_core_clock_updates, 1u);
	zassert_equal(bridge_core_clock_hz, PWM_TIMER_CLK_HZ);
	zassert_true(bridge_core_clock_matches);
}

/* Every failure exit still re-derives SystemCoreClock and refreshes the
 * telemetry mirror, so it describes the live (8 MHz) clock. */
ZTEST(power_wake, test_clock_restore_bounded_when_pll_never_locks)
{
	mock_power_reset();
	RCU_CTL                        = 0u; /* PLLSTB never sets */
	RCU_CFG0                       = 0u;
	mock_system_core_clock_updates = 0u;
	SystemCoreClock                = 8000000u;

	zassert_false(bridge_clock_restore_after_deepsleep());
	zassert_equal(RCU_CFG0 & RCU_CFG0_SCS, 0u, "stays on IRC8M");
	zassert_equal(mock_system_core_clock_updates, 1u);
	zassert_equal(bridge_core_clock_hz, 8000000u);
	zassert_false(bridge_core_clock_matches);
	/* 50 ms at 8 MHz, not the boot-clock reload that would outlast FWDGT. */
	zassert_equal(SysTick->LOAD, 8000000u / 20u - 1u);
}

/* PLL locks but SCSS never reports PLLP: the switch must be backed out
 * (SCS = IRC8M) and the clock state refreshed, not left half-switched. */
ZTEST(power_wake, test_clock_restore_backs_out_when_scss_never_switches)
{
	mock_power_reset();
	RCU_CTL                        = RCU_CTL_PLLSTB;
	RCU_CFG0                       = 0u; /* SCSS never reads PLLP */
	mock_system_core_clock_updates = 0u;
	SystemCoreClock                = 8000000u;

	zassert_false(bridge_clock_restore_after_deepsleep());
	zassert_equal(RCU_CFG0 & RCU_CFG0_SCS, RCU_CKSYSSRC_IRC8M, "SCS reverted to IRC8M");
	zassert_equal(mock_system_core_clock_updates, 1u);
	zassert_equal(bridge_core_clock_hz, 8000000u);
	zassert_false(bridge_core_clock_matches);
}

/* ------------------------------------------------------------------ */
/* Low-power modes: BUSY gate, flags, I2C wake, diagnostics            */
/* ------------------------------------------------------------------ */

static void diag_reset(void)
{
	memset((void *)&bridge_power_diag, 0, sizeof bridge_power_diag);
}

/* Bring the mock clock tree to "PLL locked" so a wake restores cleanly. */
static void clock_ready(void)
{
	RCU_CTL  = RCU_CTL_PLLSTB;
	RCU_CFG0 = RCU_SCSS_PLLP;
}

ZTEST(power_wake, test_busy_gate_refuses_deep_sleep_and_standby_but_not_sleep)
{
	int *const acts[] = { &mock_act_adc, &mock_act_pwm, &mock_act_dac, &mock_act_ota };
	for (size_t i = 0; i <= sizeof acts / sizeof acts[0]; ++i) {
		mock_power_reset();
		diag_reset();
		if (i < sizeof acts / sizeof acts[0]) {
			*acts[i] = 1;
		} else {
			mock_trial_unconfirmed = 1; /* pending boot-config commit */
		}
		zassert_equal(
		    bridge_hw_power_mode_set(2u, 0u, 100u, 0u), BRIDGE_HW_ERR_BUSY, "%u", (unsigned)i);
		zassert_equal(
		    bridge_hw_power_mode_set(3u, 0u, 100u, 0u), BRIDGE_HW_ERR_BUSY, "%u", (unsigned)i);
		zassert_equal(mock_power_hw_calls, 0u, "refused before any hardware touch");
		zassert_equal(bridge_power_diag.refused_busy, 2u);
		/* SLEEP keeps every clock running; RUN is a no-op. */
		zassert_equal(bridge_hw_power_mode_set(1u, 0u, 0u, 0u), BRIDGE_HW_OK);
		zassert_equal(bridge_hw_power_mode_set(0u, 0u, 0u, 0u), BRIDGE_HW_OK);
		zassert_equal(bridge_power_diag.entries[1], 1u);
		/* Stopping the feature lifts the refusal. */
		if (i < sizeof acts / sizeof acts[0]) {
			*acts[i] = 0;
		} else {
			mock_trial_unconfirmed = 0;
		}
		zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, 0u), BRIDGE_HW_OK);
		bridge_hw_power_mode_set(0u, 0u, 0u, 0u); /* cancel the latch */
	}
}

ZTEST(power_wake, test_activity_started_after_the_reply_drops_the_request)
{
	mock_power_reset();
	diag_reset();
	clock_ready();
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, 0u), BRIDGE_HW_OK);
	mock_act_adc = 1; /* e.g. ADC_STREAM_BEGIN2 arrived after the accepted request */
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 0u);
	zassert_equal(bridge_power_diag.refused_late, 1u);
	mock_act_adc = 0;
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 0u, "the dropped request is not resurrected");
}

/* Bench 2026-10-07: entering before the host read the reply made its CS
 * falling edge the wake source and cost the reply. */
ZTEST(power_wake, test_entry_waits_for_the_reply_to_be_read)
{
	mock_power_reset();
	diag_reset();
	clock_ready();
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 250u, 0u), BRIDGE_HW_OK);
	SystemCoreClock      = PWM_TIMER_CLK_HZ;
	mock_reply_undrained = 1;
	/* Many ticks, little time: ticks are not a time base. */
	for (unsigned i = 0u; i < 50u; i++) {
		mock_dwt.CYCCNT += (SystemCoreClock / 1000u) * 5u; /* 5 ms */
		bridge_power_tick();
	}
	zassert_equal(mock_deepsleep_entries, 0u, "no entry while the reply is unread");
	zassert_equal(bridge_power_diag.refused_late, 0u);
	mock_reply_undrained = 0; /* host's reply read ended */
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 1u, "entry once the reply is out");
}

ZTEST(power_wake, test_unread_reply_times_out_without_entry)
{
	mock_power_reset();
	diag_reset();
	clock_ready();
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 250u, 0u), BRIDGE_HW_OK);
	SystemCoreClock      = PWM_TIMER_CLK_HZ;
	mock_reply_undrained = 1;
	bridge_power_tick(); /* wait starts */
	mock_dwt.CYCCNT += (SystemCoreClock / 1000u) * 499u;
	bridge_power_tick();
	zassert_equal(bridge_power_diag.refused_late, 0u, "still inside the bound");
	mock_dwt.CYCCNT += (SystemCoreClock / 1000u) * 1u;
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 0u);
	zassert_equal(bridge_power_diag.refused_late, 1u, "abandoned at 500 ms, counted once");
	mock_reply_undrained = 0;
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 0u, "the abandoned request is not resurrected");
}

ZTEST(power_wake, test_run_cancels_a_latched_request)
{
	mock_power_reset();
	clock_ready();
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, 0u), BRIDGE_HW_OK);
	zassert_equal(bridge_hw_power_mode_set(0u, 0u, 0u, 0u), BRIDGE_HW_OK);
	zassert_true(mock_rtc_disables > 0u, "the armed RTC timer is stopped");
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 0u);
}

ZTEST(power_wake, test_flag_validation)
{
	mock_power_reset();
	zassert_equal(
	    bridge_hw_power_mode_set(2u, 0u, 100u, 0x02u), BRIDGE_HW_ERR_INVAL, "unknown bit");
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 100u, POWER_FLAG_WAKE_I2C), BRIDGE_HW_ERR_INVAL);
	zassert_equal(bridge_hw_power_mode_set(1u, 0u, 0u, POWER_FLAG_WAKE_I2C), BRIDGE_HW_ERR_INVAL);
	zassert_equal(bridge_hw_power_mode_set(0u, 0u, 0u, POWER_FLAG_WAKE_I2C), BRIDGE_HW_ERR_INVAL);
	zassert_equal(mock_power_hw_calls, 0u);
	/* Untimed WAKE_I2C is refused until the I2C0 wake line is bench-proven;
	 * without the flag an untimed Deep-sleep is INVAL. */
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 0u, POWER_FLAG_WAKE_I2C), BRIDGE_HW_ERR_RANGE);
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 0u, 0u), BRIDGE_HW_ERR_INVAL);
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, POWER_FLAG_WAKE_I2C), BRIDGE_HW_OK);
	bridge_hw_power_mode_set(0u, 0u, 0u, 0u);
}

ZTEST(power_wake, test_i2c_wake_runs_i2c_from_irc8m_then_reverts_to_apb1_from_the_tick)
{
	mock_power_reset();
	diag_reset();
	clock_ready();
	mock_pd0_on_wake = 1u << 31; /* the I2C0 wakeup line ended the sleep */
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, POWER_FLAG_WAKE_I2C), BRIDGE_HW_OK);
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 1u);
	zassert_equal(mock_i2c_inits, 1u, "initialised once, before the entry; not cut after the wake");
	zassert_equal(mock_i2c_init_wake_mode, 1u, "that init ran with the IRC8M kernel clock + WUEN");
	zassert_equal(mock_exti31_enables, 1u, "line armed for the sleep");
	zassert_equal(mock_exti31_disables, 1u, "and disabled again after the wake");
	zassert_equal(mock_i2c_enables, 0u);
	zassert_true(mock_seq_i2c_init < mock_seq_clock_restore, "I2C armed first, PLL restored after");
	zassert_equal(mock_i2c_wake_mode, 0, "wake mode cleared for the revert");
	zassert_equal(bridge_power_diag.wakes[2], 1u);
	zassert_equal(bridge_power_diag.entries[2], 1u);
	zassert_equal(bridge_power_diag.last_wake_source, POWER_WAKE_SRC_I2C);
	zassert_equal(bridge_power_diag.i2c_wake_armed, 1u);
	zassert_true(bridge_power_diag.last_wake_restore_cyc > 0u);

	/* Base level, link idle: I2C0 goes back to APB1 (kernel clock + no WUEN). */
	mock_pd0_on_wake = 0u;
	EXTI_PD0         = 0u;
	mock_i2c_busy    = SET; /* the woken transaction is still running */
	bridge_power_tick();
	zassert_equal(mock_i2c_inits, 1u, "not while the transaction runs");
	mock_i2c_busy = RESET;
	bridge_power_tick();
	zassert_equal(mock_i2c_inits, 2u);
	zassert_equal(mock_i2c_init_wake_mode, 0u, "re-initialised on APB1");
}

ZTEST(power_wake, test_wake_source_classification)
{
	const struct {
		uint32_t pd0;
		uint8_t  src;
	} cases[] = {
		{ 1u << 8, POWER_WAKE_SRC_SPI_CS },
		{ 1u << 19, POWER_WAKE_SRC_RTC },
		{ (1u << 8) | (1u << 19), POWER_WAKE_SRC_SPI_CS },
		{ 0u, POWER_WAKE_SRC_NONE },
		{ 1u << 3, POWER_WAKE_SRC_OTHER },
	};
	for (size_t i = 0; i < sizeof cases / sizeof cases[0]; ++i) {
		mock_power_reset();
		diag_reset();
		clock_ready();
		mock_pd0_on_wake = cases[i].pd0;
		zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, 0u), BRIDGE_HW_OK);
		bridge_power_tick();
		zassert_equal(bridge_power_diag.last_wake_source, cases[i].src, "case %u", (unsigned)i);
		zassert_equal(bridge_power_diag.last_wake_pd0, cases[i].pd0);
	}
}

ZTEST(power_wake, test_i2c_wake_init_failure_still_sleeps_timer_only)
{
	mock_power_reset();
	diag_reset();
	clock_ready();
	mock_i2c_init_rc = BRIDGE_HW_ERR_RANGE;
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, POWER_FLAG_WAKE_I2C), BRIDGE_HW_OK);
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 1u);
	zassert_equal(mock_i2c_wake_mode, 0, "wake mode reverted");
	zassert_equal(bridge_power_diag.i2c_wake_armed, 0u);
	mock_i2c_init_rc = BRIDGE_HW_OK;
	drain_pending_relock();
}

ZTEST(power_wake, test_standby_entries_are_counted)
{
	mock_power_reset();
	diag_reset();
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 100u, 0u), BRIDGE_HW_OK);
	bridge_power_tick();
	zassert_equal(bridge_power_diag.entries[3], 1u);
	zassert_equal(bridge_power_diag.last_mode, 3u);
}

/* A RUN request arriving from the CS ISR during the watchdog-settle spin (which
 * runs with interrupts on) must stop the entry: previously Standby was entered
 * with the RTC timer already disabled, and Deep-sleep slept after the cancel. */
static void inject_run_cancel(void)
{
	zassert_equal(bridge_hw_power_mode_set(0u, 0u, 0u, 0u), BRIDGE_HW_OK);
}

ZTEST(power_wake, test_review_run_cancel_during_settle_deepsleep)
{
	mock_power_reset();
	diag_reset();
	clock_ready();
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, 0u), BRIDGE_HW_OK);
	mock_on_settle = inject_run_cancel;
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 0u, "no sleep after the cancel");
	zassert_equal(mock_i2c_disables, 0u);
	zassert_equal(mock_primask, 0u);
	zassert_equal(bridge_power_diag.cancelled, 2u, "RUN itself + the gate that saw it");
	mock_on_settle = NULL;
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 0u, "and it stays cancelled");
}

ZTEST(power_wake, test_review_run_cancel_during_settle_standby)
{
	mock_power_reset();
	diag_reset();
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 100u, 0u), BRIDGE_HW_OK);
	mock_systick.CTRL = SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk;
	mock_on_settle    = inject_run_cancel;
	bridge_power_tick();
	zassert_equal(mock_systick_ctrl_at_standby, 0u, "standby was never entered");
	zassert_equal(bridge_power_diag.entries[3], 0u);
	zassert_equal(mock_primask, 0u, "lock released");
	zassert_equal(mock_systick.CTRL, SysTick_CTRL_ENABLE_Msk | SysTick_CTRL_TICKINT_Msk);
	mock_on_settle = NULL;
	bridge_power_tick();
	zassert_equal(bridge_power_diag.entries[3], 0u, "and it stays cancelled");
}

ZTEST(power_wake, test_standby_aborted_entry_releases_the_lock)
{
	run_aborted_standby(SysTick_CTRL_ENABLE_Msk);
	zassert_equal(mock_primask, 0u);
}

ZTEST(power_wake, test_sleep_cancels_a_latched_request)
{
	mock_power_reset();
	diag_reset();
	clock_ready();
	zassert_equal(bridge_hw_power_mode_set(2u, 0u, 100u, 0u), BRIDGE_HW_OK);
	zassert_equal(bridge_hw_power_mode_set(1u, 0u, 0u, 0u), BRIDGE_HW_OK);
	bridge_power_tick();
	zassert_equal(mock_deepsleep_entries, 0u);
	zassert_equal(bridge_power_diag.cancelled, 1u);
}

ZTEST_SUITE(power_wake, NULL, NULL, NULL, NULL, NULL);
