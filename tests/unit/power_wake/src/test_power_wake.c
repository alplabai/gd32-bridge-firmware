/* SPDX-License-Identifier: Apache-2.0
 *
 * Host tests for bridge_hw_power_mode_set()'s wake-bitmap gate.  This suite
 * links the real power.c against a minimal PMU/RTC mock: return status and
 * zero hardware calls together prove a rejected request cannot enter a
 * low-power path before replying.
 */

#include <zephyr/ztest.h>

#include "bridge_hw.h"
#include "gd32_common.h"
#include "gd32g5x3.h"
#include "power_wake.h"

static void expect_rejected_without_hardware(uint32_t wake_bitmap)
{
	mock_power_reset();
	zassert_equal(bridge_hw_power_mode_set(2u, wake_bitmap, 0u), BRIDGE_HW_ERR_NOTIMPL);
	zassert_equal(mock_power_hw_calls, 0u);
}

ZTEST(power_wake, test_accepts_empty_and_supported_bits)
{
	/* Mode 0 is the side-effect-free way to prove each bitmap clears the
	 * production validation gate. */
	zassert_equal(bridge_hw_power_mode_set(0u, 0u, 0u), BRIDGE_HW_OK);
	zassert_equal(bridge_hw_power_mode_set(0u, POWER_WAKE_RTC, 0u), BRIDGE_HW_OK);
	zassert_equal(bridge_hw_power_mode_set(0u, POWER_WAKE_TIMER, 0u), BRIDGE_HW_OK);
	zassert_equal(bridge_hw_power_mode_set(0u, POWER_WAKE_MASK_SUPPORTED, 0u), BRIDGE_HW_OK);
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

ZTEST(power_wake, test_deep_sleep_refused_without_hardware)
{
	/* gh#63: mode 2 has no BRD_I2C wake path, so a supported bitmap is
	 * still refused (NOTIMPL -> STATUS_NOSUPPORT on the wire) before any
	 * PMU/RTC side effect. */
	mock_power_reset();
	zassert_equal(bridge_hw_power_mode_set(2u, POWER_WAKE_RTC, 100u), BRIDGE_HW_ERR_NOTIMPL);
	zassert_equal(mock_power_hw_calls, 0u);
}

ZTEST(power_wake, test_standby_without_rtc_wake_is_refused)
{
	/* gh#63 / gh#40: standby with no RTC wake would need NRST to leave. */
	mock_power_reset();
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 0u), BRIDGE_HW_ERR_INVAL);
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
	zassert_equal(bridge_hw_power_mode_set(3u, POWER_WAKE_RTC, 0u), BRIDGE_HW_OK);
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
	zassert_equal(bridge_hw_power_mode_set(0u, 0u, 1u), BRIDGE_HW_ERR_INVAL);
	zassert_equal(mock_power_hw_calls, 0u);

	mock_power_reset();
	zassert_equal(bridge_hw_power_mode_set(1u, 0u, 500u), BRIDGE_HW_ERR_INVAL);
	zassert_equal(mock_power_hw_calls, 0u);

	/* wake_after_ms == 0 is still the accepted no-op for both modes. */
	mock_power_reset();
	zassert_equal(bridge_hw_power_mode_set(0u, 0u, 0u), BRIDGE_HW_OK);
	zassert_equal(bridge_hw_power_mode_set(1u, 0u, 0u), BRIDGE_HW_OK);
}

extern void bridge_power_tick(void);

ZTEST(power_wake, test_standby_long_wake_refused_when_fwdgt_runs_in_standby)
{
	/* FWDGSPD_STDBY = 1: the ~501 ms FWDGT would reset the part out of a
	 * longer standby, so refuse before any hardware touch. */
	mock_power_reset();
	FMC_OBCTL = FMC_OBCTL_FWDGSPD_STDBY;
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 301u), BRIDGE_HW_ERR_RANGE);
	zassert_equal(bridge_hw_power_mode_set(3u, POWER_WAKE_RTC, 0u), BRIDGE_HW_ERR_RANGE);
	zassert_equal(mock_power_hw_calls, 2u, "one PMU clock enable per call, nothing armed");
	/* A wake inside the window is still accepted. */
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 300u), BRIDGE_HW_OK);
}

ZTEST(power_wake, test_standby_long_wake_accepted_when_fwdgt_frozen_in_standby)
{
	mock_power_reset();
	FMC_OBCTL = 0u;
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 30000u), BRIDGE_HW_OK);
}

/* Latch a standby request and run the base-level entry; the mock's
 * pmu_to_standbymode() returns, i.e. the entry ABORTS. */
static void run_aborted_standby(uint32_t systick_ctrl)
{
	mock_power_reset();
	mock_systick.CTRL = systick_ctrl;
	mock_systick.VAL  = 1234u;
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 100u), BRIDGE_HW_OK);
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
	zassert_equal(bridge_hw_power_mode_set(3u, 0u, 100u), BRIDGE_HW_OK);
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

ZTEST_SUITE(power_wake, NULL, NULL, NULL, NULL, NULL);
