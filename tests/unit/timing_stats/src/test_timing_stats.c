/* SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the opt-in SPI timing stats (src/timing_stats.c, #104):
 * the min/max/last/count accumulator, wrap-safe deltas, and the hook wired
 * into transport_spi.c's decode path (built with -DBRIDGE_TIMING_STATS).
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "protocol.h"
#include "timing_stats.h"
#include "transport.h"

extern uint32_t timing_stats_test_cycles;
extern uint32_t timing_stats_test_step;

static void feed(const uint8_t *b, size_t n)
{
	spi_slave_cs_low();
	for (size_t i = 0; i < n; i++) {
		spi_slave_rx_byte(b[i]);
	}
	spi_slave_cs_high();
}

ZTEST(timing_stats, test_accumulator_min_max_last_count)
{
	timing_stat_t s = { 0 };
	timing_stat_add(&s, 50u);
	zassert_equal(s.count, 1u);
	zassert_equal(s.min, 50u);
	zassert_equal(s.max, 50u);
	timing_stat_add(&s, 20u);
	timing_stat_add(&s, 90u);
	timing_stat_add(&s, 60u);
	zassert_equal(s.count, 4u);
	zassert_equal(s.min, 20u);
	zassert_equal(s.max, 90u);
	zassert_equal(s.last, 60u);
}

ZTEST(timing_stats, test_zero_cycle_first_sample_sets_min)
{
	timing_stat_t s = { 0 };
	timing_stat_add(&s, 0u);
	timing_stat_add(&s, 7u);
	zassert_equal(s.min, 0u);
	zassert_equal(s.max, 7u);
}

ZTEST(timing_stats, test_record_wraps_and_splits_stages)
{
	timing_stats_init();
	timing_stats_test_step   = 0u;
	timing_stats_test_cycles = 0xFFFFFFF0u;
	timing_stats_mark_cs_edge();
	/* crc0/crc1/disp1/end straddle the 32-bit wrap: 4, 6, 10 cycles. */
	timing_stats_record(0xFFFFFFF4u, 0xFFFFFFF8u, 0xFFFFFFFEu, 0x00000008u, 0x01u);
	zassert_equal(bridge_timing_stats.req_crc.last, 4u);
	zassert_equal(bridge_timing_stats.dispatch.last, 6u);
	zassert_equal(bridge_timing_stats.reply_stage.last, 10u);
	zassert_equal(bridge_timing_stats.total.last, 24u); /* end - cs edge */
	zassert_equal(bridge_timing_stats.cmd_last, 0x01u);
}

ZTEST(timing_stats, test_spi_transaction_is_recorded)
{
	static const uint8_t ping[] = { 0xA5, 0x00, 0x84, 0xFF };
	timing_stats_init();
	transport_spi_init();
	timing_stats_test_step = 10u;
	timing_stats_mark_cs_edge(); /* the CS-rising ISR does this on silicon */
	feed(ping, sizeof ping);
	zassert_equal(bridge_timing_stats.total.count, 1u);
	zassert_equal(bridge_timing_stats.req_crc.last, 10u);
	zassert_equal(bridge_timing_stats.dispatch.last, 10u);
	zassert_equal(bridge_timing_stats.reply_stage.last, 10u);
	zassert_equal(bridge_timing_stats.total.last, 40u);
	zassert_equal(bridge_timing_stats.cmd_last, 0x00u);

	/* A CRC-failing request is not a sample. */
	static const uint8_t bad[] = { 0xA5, 0x00, 0x00, 0x00 };
	feed(bad, sizeof bad);
	zassert_equal(bridge_timing_stats.total.count, 1u);
}

ZTEST(timing_stats, test_layout_is_76_bytes)
{
	timing_stats_init();
	zassert_equal(sizeof(bridge_timing_stats_t), 76u);
	zassert_equal(bridge_timing_stats.magic, TIMING_STATS_MAGIC);
}

ZTEST_SUITE(timing_stats, NULL, NULL, NULL, NULL, NULL);
