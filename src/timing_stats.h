/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge firmware: opt-in SPI round-trip timing stats (issue #104).
 *
 * Built only with -DBRIDGE_TIMING_STATS=ON.  With the option OFF (the
 * default) every TS_ONLY() below expands to nothing and timing_stats.c is
 * not compiled, so the image is byte-identical to a build without this
 * feature.
 *
 * The DWT cycle counter is sampled around each stage of one SPI
 * transaction and folded into a RAM struct the host reads over SWD:
 *
 *   symbol: bridge_timing_stats   (see docs/timing-stats.md for the layout)
 *
 * Only transactions that decode, pass the request CRC and reach
 * protocol_dispatch() are recorded.  Zero only offsets 0x08..0x4B over SWD
 * to restart the measurement, keeping magic/layout (count == 0 means "no
 * sample yet").
 */
#ifndef GD32_BRIDGE_TIMING_STATS_H
#define GD32_BRIDGE_TIMING_STATS_H

#include <stdint.h>

#ifdef BRIDGE_TIMING_STATS

#define TS_ONLY(...) __VA_ARGS__

#define TIMING_STATS_MAGIC  0x41545354u /* "TSTA" little-endian */
#define TIMING_STATS_LAYOUT 1u

/* One accumulator: 16 bytes.  min/max/last are DWT cycles. */
typedef struct {
	uint32_t count;
	uint32_t last;
	uint32_t min;
	uint32_t max;
} timing_stat_t;

/* 8-byte header + 4 x 16-byte stats + cmd_last = 76 bytes, no padding. */
typedef struct {
	uint32_t      magic;  /* TIMING_STATS_MAGIC once initialised          */
	uint32_t      layout; /* TIMING_STATS_LAYOUT                          */
	timing_stat_t req_crc;
	timing_stat_t dispatch;
	timing_stat_t reply_stage;
	timing_stat_t total;
	uint32_t      cmd_last; /* opcode of the most recent recorded request */
} bridge_timing_stats_t;

extern bridge_timing_stats_t bridge_timing_stats;

/* Clear the struct and stamp magic/layout (called from main after
 * bridge_hw_init(), which enables the DWT cycle counter). */
void timing_stats_init(void);

/* Current DWT CYCCNT value (host builds: timing_stats_test_cycles). */
uint32_t timing_stats_now(void);

/* CS-rising-edge timestamp, taken at the CS-release branch entry (exception latency not counted). */
void timing_stats_mark_cs_edge(void);

/* Fold one accumulator; pure, exposed for the host test. */
void timing_stat_add(timing_stat_t *s, uint32_t cycles);

/* Record one transaction from its five timestamps (wrap-safe subtraction):
 * crc0/crc1 bracket the request CRC check, disp1 follows protocol_dispatch,
 * end follows stage_reply.  total = end - the last CS-edge mark. */
void timing_stats_record(uint32_t crc0, uint32_t crc1, uint32_t disp1, uint32_t end, uint8_t cmd);

#else

#define TS_ONLY(...)

#endif /* BRIDGE_TIMING_STATS */

#endif /* GD32_BRIDGE_TIMING_STATS_H */
