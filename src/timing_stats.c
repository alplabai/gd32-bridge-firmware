/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge firmware: opt-in SPI round-trip timing stats (issue #104).
 * Compiled only with -DBRIDGE_TIMING_STATS=ON; see timing_stats.h.
 */

#include "timing_stats.h"

#ifdef BRIDGE_TIMING_STATS

#include <string.h>

/* Read over SWD; `used` keeps it even if a future LTO build sees no reader. */
__attribute__((used)) bridge_timing_stats_t bridge_timing_stats;

static uint32_t cs_edge_cycles;

#if defined(__arm__)
/* Cortex-M33 DWT CYCCNT by address (keeps this file free of vendor headers);
 * bridge_hw_init() already enables the counter. */
#define DWT_CYCCNT (*(volatile uint32_t *)0xE0001004u)

uint32_t timing_stats_now(void)
{
	return DWT_CYCCNT;
}

#else
/* Host build: a fake clock that advances by timing_stats_test_step on every
 * read, so a test gets distinct, predictable per-stage deltas. */
uint32_t timing_stats_test_cycles;
uint32_t timing_stats_test_step;

uint32_t timing_stats_now(void)
{
	timing_stats_test_cycles += timing_stats_test_step;
	return timing_stats_test_cycles;
}
#endif

void timing_stats_init(void)
{
	memset(&bridge_timing_stats, 0, sizeof(bridge_timing_stats));
	bridge_timing_stats.magic  = TIMING_STATS_MAGIC;
	bridge_timing_stats.layout = TIMING_STATS_LAYOUT;
}

void timing_stats_mark_cs_edge(void)
{
	cs_edge_cycles = timing_stats_now();
}

void timing_stat_add(timing_stat_t *s, uint32_t cycles)
{
	if (s->count == 0u || cycles < s->min) {
		s->min = cycles;
	}
	if (s->count == 0u || cycles > s->max) {
		s->max = cycles;
	}
	s->last = cycles;
	s->count++;
}

void timing_stats_record(uint32_t crc0, uint32_t crc1, uint32_t disp1, uint32_t end, uint8_t cmd)
{
	bridge_timing_stats_t *t = &bridge_timing_stats;
	timing_stat_add(&t->req_crc, crc1 - crc0);
	timing_stat_add(&t->dispatch, disp1 - crc1);
	timing_stat_add(&t->reply_stage, end - disp1);
	timing_stat_add(&t->total, end - cs_edge_cycles);
	t->cmd_last = cmd;
}

#endif /* BRIDGE_TIMING_STATS */
