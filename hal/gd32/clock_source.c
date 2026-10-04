/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure HXTAL-bypass / IRC8M-fallback sequencing.  See clock_source.h.
 */

#include "clock_source.h"

volatile bridge_clock_source_t   bridge_clock_source   = BRIDGE_CLOCK_SRC_IRC8M;
volatile bridge_clock_fallback_t bridge_clock_fallback = BRIDGE_CLOCK_FB_NONE;

static void fall_back(const bridge_clock_ops_t *ops, bridge_clock_fallback_t why, bool repoint_pll)
{
	ops->hxtal_stop();
	if (repoint_pll) {
		(void)ops->pll_select(BRIDGE_CLOCK_SRC_IRC8M);
	}
	bridge_clock_source   = BRIDGE_CLOCK_SRC_IRC8M;
	bridge_clock_fallback = why;
}

bool bridge_clock_external_start(const bridge_clock_ops_t *ops, uint32_t spins, bool pll_on_hxtal)
{
	/* Bypass first, with HXTALEN clear (the UM requires it), then enable. */
	ops->hxtal_bypass_enable();
	ops->hxtal_enable();

	bool stable = false;
	for (uint32_t n = spins; n != 0u; n--) {
		if (ops->hxtal_stable()) {
			stable = true;
			break;
		}
	}
	if (!stable) {
		fall_back(ops, BRIDGE_CLOCK_FB_HXTAL_TIMEOUT, pll_on_hxtal);
		return false;
	}

	if (!ops->pll_select(BRIDGE_CLOCK_SRC_HXTAL)) {
		fall_back(ops, BRIDGE_CLOCK_FB_PLL_FAIL, true);
		return false;
	}

	ops->ckm_enable();
	bridge_clock_source   = BRIDGE_CLOCK_SRC_HXTAL;
	bridge_clock_fallback = BRIDGE_CLOCK_FB_NONE;
	return true;
}

bridge_clock_source_t bridge_clock_select(const bridge_clock_ops_t *ops, uint32_t spins)
{
	if (ops->marker_get() == BRIDGE_CLOCK_ATTEMPT_MAGIC) {
		/* The previous boot switched to HXTAL and never got healthy
		 * (fault, hang + reset).  Do not repeat it. */
		ops->marker_set(0u);
		fall_back(ops, BRIDGE_CLOCK_FB_PREV_BOOT_UNHEALTHY, false);
		return bridge_clock_source;
	}

	/* The marker covers the window where the PLL runs from HXTAL; it is set
	 * before the attempt and cleared by bridge_clock_mark_healthy(). */
	ops->marker_set(BRIDGE_CLOCK_ATTEMPT_MAGIC);
	if (!bridge_clock_external_start(ops, spins, false)) {
		ops->marker_set(0u); /* failed cleanly on IRC8M: nothing to remember */
	}
	return bridge_clock_source;
}

void bridge_clock_on_ckm_failure(const bridge_clock_ops_t *ops)
{
	fall_back(ops, BRIDGE_CLOCK_FB_CKM_FAILURE, true);
}

void bridge_clock_mark_healthy(const bridge_clock_ops_t *ops)
{
	if (ops->marker_get() != 0u) {
		ops->marker_set(0u);
	}
}
