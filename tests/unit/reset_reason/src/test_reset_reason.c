/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/ztest.h>

#include <stdint.h>

#include "reset_reason.h"

ZTEST_SUITE(gd32_bridge_reset_reason, NULL, NULL, NULL, NULL, NULL);

#define COLD_RESET_VALUE \
	(BRIDGE_RESET_RSTSCK_PORRSTF | BRIDGE_RESET_RSTSCK_EPRSTF | BRIDGE_RESET_RSTSCK_BORRSTF)

ZTEST(gd32_bridge_reset_reason, test_documented_cold_reset_decodes_power_on)
{
	zassert_equal(bridge_reset_reason_decode(COLD_RESET_VALUE), 1u);
}

ZTEST(gd32_bridge_reset_reason, test_cold_flags_outrank_watchdog)
{
	zassert_equal(bridge_reset_reason_decode(COLD_RESET_VALUE | BRIDGE_RESET_RSTSCK_FWDGTRSTF), 1u);
	zassert_equal(
	    bridge_reset_reason_decode(BRIDGE_RESET_RSTSCK_FWDGTRSTF | BRIDGE_RESET_RSTSCK_PORRSTF),
	    1u);
	zassert_equal(
	    bridge_reset_reason_decode(BRIDGE_RESET_RSTSCK_WWDGTRSTF | BRIDGE_RESET_RSTSCK_BORRSTF),
	    5u);
}

ZTEST(gd32_bridge_reset_reason, test_watchdog_outranks_nrst_pin)
{
	zassert_equal(
	    bridge_reset_reason_decode(BRIDGE_RESET_RSTSCK_FWDGTRSTF | BRIDGE_RESET_RSTSCK_EPRSTF), 4u);
	zassert_equal(
	    bridge_reset_reason_decode(BRIDGE_RESET_RSTSCK_WWDGTRSTF | BRIDGE_RESET_RSTSCK_EPRSTF), 4u);
}

ZTEST(gd32_bridge_reset_reason, test_nrst_and_lowpower_outrank_software)
{
	zassert_equal(
	    bridge_reset_reason_decode(BRIDGE_RESET_RSTSCK_SWRSTF | BRIDGE_RESET_RSTSCK_EPRSTF), 2u);
	zassert_equal(
	    bridge_reset_reason_decode(BRIDGE_RESET_RSTSCK_SWRSTF | BRIDGE_RESET_RSTSCK_LPRSTF), 6u);
	zassert_equal(bridge_reset_reason_decode(BRIDGE_RESET_RSTSCK_SWRSTF), 3u);
	zassert_equal(bridge_reset_reason_decode(BRIDGE_RESET_RSTSCK_LPRSTF), 6u);
	zassert_equal(bridge_reset_reason_decode(BRIDGE_RESET_RSTSCK_EPRSTF), 2u);
	zassert_equal(bridge_reset_reason_decode(BRIDGE_RESET_RSTSCK_BORRSTF), 5u);
}

ZTEST(gd32_bridge_reset_reason, test_unknown_and_option_byte_only_remain_unknown)
{
	zassert_equal(bridge_reset_reason_decode(0u), 0u);
	zassert_equal(bridge_reset_reason_decode(BRIDGE_RESET_RSTSCK_OBLRSTF), 0u);
}

ZTEST(gd32_bridge_reset_reason, test_latch_preserves_preclear_snapshot_and_requests_one_clear)
{
	volatile uint32_t mock_rstsck = COLD_RESET_VALUE | BRIDGE_RESET_RSTSCK_SWRSTF;
	const uint32_t    snapshot    = bridge_reset_reason_latch_and_clear(&mock_rstsck);

	zassert_equal(snapshot, COLD_RESET_VALUE | BRIDGE_RESET_RSTSCK_SWRSTF);
	zassert_true((mock_rstsck & BRIDGE_RESET_RSTSCK_RSTFC) != 0u,
	             "the RSTFC clear request must be written after capture");
	zassert_equal(bridge_reset_reason_decode(snapshot), 1u);
	zassert_equal(bridge_reset_reason_decode(snapshot),
	              1u,
	              "a retained snapshot must be idempotent within one boot");
}
