/* SPDX-License-Identifier: Apache-2.0
 *
 * The two non-default clock_hw.c builds, one source file, selected by a
 * compile definition:
 *   -DBRIDGE_CLOCK_HXTAL_AT_BOOT=1  the autonomous attempt in bridge_clock_init()
 *   -DBRIDGE_CLOCK_IRC8M_ONLY=1     the HXTAL path compiled out
 * The default build (host-requested only) is test_clock_hw.c.
 */

#include <zephyr/ztest.h>

#include <string.h>

#include "gd32g5x3.h"

#include "clock_source.h"
#include "gd32_common.h"
#include "mock_hw.h"

static void boot(bool fwdgt_running)
{
	bridge_clock_source        = BRIDGE_CLOCK_SRC_IRC8M;
	bridge_clock_fallback      = BRIDGE_CLOCK_FB_NONE;
	bridge_clock_input_hz      = 0u;
	bridge_clock_hxtal_request = 0u;
	bridge_clock_init(fwdgt_running);
}

ZTEST_SUITE(gd32_bridge_clock_variant, NULL, NULL, NULL, NULL, NULL);

#if defined(BRIDGE_CLOCK_IRC8M_ONLY)

ZTEST(gd32_bridge_clock_variant, test_irc8m_only_build_never_touches_the_clock_tree)
{
	mock_hw_reset();
	boot(true);
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_BUILD_DISABLED);
	zassert_false(bridge_clock_try_hxtal());
	bridge_clock_request_hxtal();
	bridge_clock_hxtal_request = 1u;
	bridge_clock_tick();
	zassert_equal(bridge_clock_hxtal_request, 0u, "a stale request does not linger");
	bridge_clock_pre_deepsleep();
	bridge_clock_relock_prepare();
	zassert_false(bridge_clock_nmi_recover(SYSCFG_STAT_CKMNMIIF), "a CKM NMI is a fault here");
	zassert_equal(strlen(mock_log_text()), 0u, "no register written: %s", mock_log_text());
}

ZTEST(gd32_bridge_clock_variant, test_irc8m_only_build_still_reads_the_live_clock)
{
	mock_hw_reset();
	boot(true);
	bridge_clock_core_update();
	zassert_equal(SystemCoreClock, 216000000u);
	zassert_true(bridge_clock_core_matches());
	zassert_equal(bridge_clock_apb1_hz(), 216000000u);
}

#elif defined(BRIDGE_CLOCK_HXTAL_AT_BOOT)

ZTEST(gd32_bridge_clock_variant, test_at_boot_attempt_succeeds_and_marker_clears_on_the_first_tick)
{
	mock_hw_reset();
	boot(true);
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_HXTAL);
	zassert_equal(bridge_clock_input_hz, BRIDGE_CLOCK_IN_8MHZ);
	zassert_equal(RTC_BKP9, BRIDGE_CLOCK_ATTEMPT_MAGIC);
	zassert_true(mock_fwdgt_feeds > 0u, "the dog is fed inside the waits when it is armed");
	bridge_clock_tick();
	zassert_equal(RTC_BKP9, 0u);
}

ZTEST(gd32_bridge_clock_variant, test_at_boot_with_se2_off_is_bounded_and_falls_back)
{
	mock_hw_reset();
	mock_scn.oscin_hz = 0u;
	boot(true);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_HXTAL_TIMEOUT);
	zassert_true(mock_now_ns() >= 5000000ull && mock_now_ns() <= 5300000ull,
	             "%llu ns",
	             (unsigned long long)mock_now_ns());
	zassert_equal(RCU_CFG0 & RCU_CFG0_SCSS, RCU_SCSS_PLLP, "the IRC8M PLL was never left");
}

ZTEST(gd32_bridge_clock_variant, test_at_boot_under_an_ota_trial_does_not_feed_the_bootloader_dog)
{
	mock_hw_reset();
	boot(false); /* init.c passes !ota_trial_peek() */
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_HXTAL);
	zassert_equal(mock_fwdgt_feeds, 0u);
}

ZTEST(gd32_bridge_clock_variant, test_at_boot_honours_the_unhealthy_marker)
{
	mock_hw_reset();
	RTC_BKP9 = BRIDGE_CLOCK_ATTEMPT_MAGIC;
	boot(true);
	zassert_equal(bridge_clock_source, BRIDGE_CLOCK_SRC_IRC8M);
	zassert_equal(bridge_clock_fallback, BRIDGE_CLOCK_FB_PREV_BOOT_UNHEALTHY);
	zassert_equal(RTC_BKP9, 0u);
	zassert_false(mock_log_has("HXEN+"));
}

#endif
