/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * LPTIMER low-power tick arithmetic (gh#65).  Vendor-header-free so the
 * period maths -- where a wrong auto-reload runs the housekeeping pump a
 * thousand times too slowly -- is host-testable.
 *
 * Source clock is IRC32K (RCU_CFG2 LPTIMERSEL = 0b01), prescaled /16 to a
 * nominal 2 kHz = 500 us per count.  The counter wraps after CARL + 1
 * counts, so the tick period is (CARL + 1) * 500 us.  IRC32K is only
 * specified as 28..36 kHz over -40..105 degC (GD32G553xx Datasheet Rev2.0
 * p.125, Table 4-23), so the real period is -12.5 % / +14.3 % of nominal.
 */
#ifndef GD32_BRIDGE_LPTIMER_TICK_H
#define GD32_BRIDGE_LPTIMER_TICK_H

#include <stdint.h>

#define LPTIMER_TICK_CLK_HZ     32000u /* nominal IRC32K               */
#define LPTIMER_TICK_PSC_DIV    16u    /* prescaler output = 2 kHz     */
#define LPTIMER_TICK_COUNT_US   500u   /* nominal us per LPTIMER count */
#define LPTIMER_TICK_CAR_MAX    0xFFFFu
#define LPTIMER_TICK_IRC32K_MIN 28000u
#define LPTIMER_TICK_IRC32K_MAX 36000u

_Static_assert(1000000u / (LPTIMER_TICK_CLK_HZ / LPTIMER_TICK_PSC_DIV) == LPTIMER_TICK_COUNT_US,
               "LPTIMER_TICK_COUNT_US does not match the prescaled clock");

/* Auto-reload value (CARL) for a nominal tick of @p tick_us.  Returns 0
 * when the request is not representable: not a whole number of counts, a
 * single count (CARL 0 would collide with the refuse value), or beyond the
 * 16-bit reload range.  Callers treat 0 as "refuse". */
static inline uint32_t lptimer_tick_carl(uint32_t tick_us)
{
	if (tick_us % LPTIMER_TICK_COUNT_US != 0u) return 0u;
	const uint32_t counts = tick_us / LPTIMER_TICK_COUNT_US;
	if (counts < 2u || counts - 1u > LPTIMER_TICK_CAR_MAX) return 0u;
	return counts - 1u;
}

/* Worst-case real period of a nominal @p tick_us over the IRC32K range. */
static inline uint32_t lptimer_tick_worst_max_us(uint32_t tick_us)
{
	return (uint32_t)(((uint64_t)tick_us * LPTIMER_TICK_CLK_HZ) / LPTIMER_TICK_IRC32K_MIN);
}

static inline uint32_t lptimer_tick_worst_min_us(uint32_t tick_us)
{
	return (uint32_t)(((uint64_t)tick_us * LPTIMER_TICK_CLK_HZ) / LPTIMER_TICK_IRC32K_MAX);
}

#endif /* GD32_BRIDGE_LPTIMER_TICK_H */
