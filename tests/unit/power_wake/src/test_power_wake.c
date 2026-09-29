/* SPDX-License-Identifier: Apache-2.0
 *
 * Host tests for bridge_hw_power_mode_set()'s wake-bitmap gate.  This suite
 * links the real power.c against a minimal PMU/RTC mock: return status and
 * zero hardware calls together prove a rejected request cannot enter a
 * low-power path before replying.
 */

#include <zephyr/ztest.h>

#include "bridge_hw.h"
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

ZTEST_SUITE(power_wake, NULL, NULL, NULL, NULL, NULL);
