/* SPDX-License-Identifier: Apache-2.0 */
/* Host proof for the vendor-header-free I2C timeout arithmetic. */

#include <zephyr/ztest.h>

#include "i2c_timeout.h"

ZTEST(i2c_timeout, test_216mhz_apb1_rounds_up_to_smbus_minimum)
{
	uint16_t reload = 0u;
	zassert_true(bridge_i2c_stretch_timeout_reload(216000000u, &reload));
	zassert_equal(reload, 2636u);
}

ZTEST(i2c_timeout, test_irc8m_deep_sleep_clock_is_rederived)
{
	uint16_t reload = 0u;
	zassert_true(bridge_i2c_stretch_timeout_reload(8000000u, &reload));
	zassert_equal(reload, 97u);
}

ZTEST(i2c_timeout, test_clocks_outside_the_25_to_35ms_window_are_rejected)
{
	uint16_t reload = 0u;
	zassert_false(bridge_i2c_stretch_timeout_reload(58514u, &reload));
	zassert_false(bridge_i2c_stretch_timeout_reload(0u, &reload));
	zassert_false(bridge_i2c_stretch_timeout_reload(216000000u, NULL));
}

ZTEST(i2c_timeout, test_clock_too_fast_for_the_12bit_counter_is_rejected)
{
	uint16_t reload = 0u;
	zassert_false(bridge_i2c_stretch_timeout_reload(500000000u, &reload));
}

ZTEST_SUITE(i2c_timeout, NULL, NULL, NULL, NULL, NULL);
