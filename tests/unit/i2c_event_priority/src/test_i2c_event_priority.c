/* SPDX-License-Identifier: Apache-2.0 */

#include <zephyr/ztest.h>

#include "i2c_event_priority.h"

ZTEST_SUITE(gd32_bridge_i2c_event_priority, NULL, NULL, NULL, NULL, NULL);

ZTEST(gd32_bridge_i2c_event_priority, test_no_pending_event_selects_none)
{
	zassert_equal(bridge_i2c_event_select(false, false, false, false), BRIDGE_I2C_EVENT_NONE);
}

ZTEST(gd32_bridge_i2c_event_priority, test_each_event_selects_itself)
{
	zassert_equal(bridge_i2c_event_select(true, false, false, false), BRIDGE_I2C_EVENT_RBNE);
	zassert_equal(bridge_i2c_event_select(false, true, false, false), BRIDGE_I2C_EVENT_ADDSEND);
	zassert_equal(bridge_i2c_event_select(false, false, true, false), BRIDGE_I2C_EVENT_STPDET);
	zassert_equal(bridge_i2c_event_select(false, false, false, true), BRIDGE_I2C_EVENT_TI);
}

ZTEST(gd32_bridge_i2c_event_priority, test_final_write_byte_precedes_repeated_start)
{
	/* RDATA/RBNE and address-match/ADDSEND clear independently.  The final
	 * byte of a combined write/read must be consumed before the read phase
	 * validates and stages its reply. */
	zassert_equal(bridge_i2c_event_select(true, true, false, false), BRIDGE_I2C_EVENT_RBNE);
}

ZTEST(gd32_bridge_i2c_event_priority, test_stop_still_precedes_transmit)
{
	zassert_equal(bridge_i2c_event_select(false, false, true, true), BRIDGE_I2C_EVENT_STPDET);
}
