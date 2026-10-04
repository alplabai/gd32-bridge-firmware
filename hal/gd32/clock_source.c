/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure HXTAL-bypass / IRC8M-fallback sequencing.  See clock_source.h.
 */

#include "clock_source.h"

volatile bridge_clock_source_t   bridge_clock_source   = BRIDGE_CLOCK_SRC_IRC8M;
volatile bridge_clock_fallback_t bridge_clock_fallback = BRIDGE_CLOCK_FB_NONE;
volatile uint32_t                bridge_clock_input_hz = 0u;

/* NMI-vs-thread handshake.  The NMI cannot be masked, so a CKM NMI can land in
 * the middle of external_start().  While `s_busy` the NMI only acknowledges
 * and sets `s_ckm_pending`; external_start() performs the fallback itself once
 * its sequence is done.  Outside a sequence the NMI does the fallback. */
static volatile bool s_busy;
static volatile bool s_ckm_pending;

/* A boot that found the attempt marker refuses HXTAL until the next boot. */
static bool s_blocked;

#define REF(in, psc, n, band) { (in), (psc), (n), (in) / (psc) * (n) / BRIDGE_CLOCK_PLL_P, (band) }

static const bridge_clock_ref_t k_refs[] = {
	REF(BRIDGE_CLOCK_IN_8MHZ, 2u, 108u, BRIDGE_CLOCK_BAND_OTHER_PERMILLE),
	REF(BRIDGE_CLOCK_IN_12MHZ, 3u, 108u, BRIDGE_CLOCK_BAND_OTHER_PERMILLE),
	REF(BRIDGE_CLOCK_IN_16MHZ, 4u, 108u, BRIDGE_CLOCK_BAND_OTHER_PERMILLE),
	REF(BRIDGE_CLOCK_IN_20MHZ, 5u, 108u, BRIDGE_CLOCK_BAND_OTHER_PERMILLE),
	REF(BRIDGE_CLOCK_IN_24P576, 6u, 105u, BRIDGE_CLOCK_BAND_24P576_PERMILLE),
};

/* Field ranges from gd32g5x3_rcu.h (PLLPSC /1../16, PLLN 8..180) and the
 * documented SYSCLK of each entry; the 216 MHz ones are exact. */
_Static_assert(BRIDGE_CLOCK_IN_8MHZ / 2u * 108u / 2u == BRIDGE_CLOCK_CORE_HZ, "8 MHz");
_Static_assert(BRIDGE_CLOCK_IN_12MHZ / 3u * 108u / 2u == BRIDGE_CLOCK_CORE_HZ, "12 MHz");
_Static_assert(BRIDGE_CLOCK_IN_16MHZ / 4u * 108u / 2u == BRIDGE_CLOCK_CORE_HZ, "16 MHz");
_Static_assert(BRIDGE_CLOCK_IN_20MHZ / 5u * 108u / 2u == BRIDGE_CLOCK_CORE_HZ, "20 MHz");
_Static_assert(BRIDGE_CLOCK_IN_24P576 / 6u * 105u / 2u == 215040000u, "24.576 MHz");
_Static_assert(BRIDGE_CLOCK_IN_24P576 % 6u == 0u, "24.576 MHz divides exactly by PLLPSC 6");

const bridge_clock_ref_t *bridge_clock_ref_for(uint32_t in_hz)
{
	for (unsigned i = 0u; i < sizeof k_refs / sizeof k_refs[0]; i++) {
		if (k_refs[i].in_hz == in_hz) return &k_refs[i];
	}
	return 0;
}

uint32_t bridge_clock_classify_div32(uint32_t counts, uint32_t window_us)
{
	if (window_us == 0u) return 0u;
	const uint64_t f = (uint64_t)counts * 1000000u / window_us; /* HXTAL/32, Hz */
	/* The bands do not overlap (tests/unit/clock_source), so the first match wins. */
	for (unsigned i = 0u; i < sizeof k_refs / sizeof k_refs[0]; i++) {
		const uint64_t nominal = k_refs[i].in_hz / BRIDGE_CLOCK_DIV32_PRESC;
		const uint64_t dev     = f > nominal ? f - nominal : nominal - f;
		if (dev * 1000u <= nominal * k_refs[i].band_permille) return k_refs[i].in_hz;
	}
	return 0u;
}

static void fall_back(const bridge_clock_ops_t *ops, bridge_clock_fallback_t why, bool repoint_pll)
{
	ops->hxtal_stop();
	if (repoint_pll && !ops->pll_select(BRIDGE_CLOCK_SRC_IRC8M, BRIDGE_CLOCK_IRC8M_HZ)) {
		/* Not even the IRC8M PLL relocks: SYSCLK is left on the bare IRC8M.
		 * Report that over the original cause -- it is the worse state. */
		why = BRIDGE_CLOCK_FB_IRC8M_PLL_FAIL;
	}
	bridge_clock_input_hz = 0u;
	bridge_clock_source   = BRIDGE_CLOCK_SRC_IRC8M;
	bridge_clock_fallback = why;
}

bool bridge_clock_external_start(const bridge_clock_ops_t *ops, bool pll_on_hxtal)
{
	s_ckm_pending = false;
	s_busy        = true;

	/* Bypass first, with HXTALEN (and CKMEN) clear -- the UM requires it --
	 * then enable. */
	ops->hxtal_bypass_enable();
	ops->hxtal_enable();

	bridge_clock_fallback_t why = BRIDGE_CLOCK_FB_NONE;
	uint32_t                hz  = 0u;
	if (!ops->hxtal_wait_stable()) {
		why = BRIDGE_CLOCK_FB_HXTAL_TIMEOUT;
	} else {
		hz = bridge_clock_classify_div32(ops->hxtal_div32_count(BRIDGE_CLOCK_FREQ_WINDOW_US),
		                                 BRIDGE_CLOCK_FREQ_WINDOW_US);
		if (hz == 0u) {
			why = BRIDGE_CLOCK_FB_HXTAL_FREQ;
		} else if (!ops->pll_select(BRIDGE_CLOCK_SRC_HXTAL, hz)) {
			why = BRIDGE_CLOCK_FB_PLL_FAIL;
		}
	}

	if (why != BRIDGE_CLOCK_FB_NONE) {
		/* A PLL failure always leaves it needing a rebuild; earlier failures
		 * only when the PLL was already on HXTAL (deep-sleep relock). */
		fall_back(ops, why, pll_on_hxtal || why == BRIDGE_CLOCK_FB_PLL_FAIL);
		s_busy = false;
		return false;
	}

	/* The state is written BEFORE the monitor is armed: a CKM NMI right after
	 * ckm_enable() then finds HXTAL recorded as the active source. */
	bridge_clock_input_hz = hz;
	bridge_clock_source   = BRIDGE_CLOCK_SRC_HXTAL;
	bridge_clock_fallback = BRIDGE_CLOCK_FB_NONE;
	ops->ckm_enable();
	s_busy = false;

	if (s_ckm_pending) { /* an NMI landed inside the sequence: act on it here */
		s_ckm_pending = false;
		fall_back(ops, BRIDGE_CLOCK_FB_CKM_FAILURE, true);
		return false;
	}
	return true;
}

bool bridge_clock_attempt_hxtal(const bridge_clock_ops_t *ops)
{
	if (bridge_clock_source == BRIDGE_CLOCK_SRC_HXTAL) return true;
	if (s_blocked) return false; /* reason already recorded at boot */

	/* The marker covers the window where the PLL runs from HXTAL; it is set
	 * before the attempt and cleared by bridge_clock_mark_healthy(). */
	ops->marker_set(BRIDGE_CLOCK_ATTEMPT_MAGIC);
	if (!bridge_clock_external_start(ops, false)) {
		ops->marker_set(0u); /* failed cleanly on IRC8M: nothing to remember */
		return false;
	}
	return true;
}

bridge_clock_source_t bridge_clock_boot(const bridge_clock_ops_t *ops, bool attempt_hxtal)
{
	s_blocked = false;
	if (ops->marker_get() == BRIDGE_CLOCK_ATTEMPT_MAGIC) {
		/* A previous attempt switched to HXTAL and never got healthy
		 * (fault, hang + reset).  Do not repeat it this boot. */
		ops->marker_set(0u);
		s_blocked = true;
		fall_back(ops, BRIDGE_CLOCK_FB_PREV_BOOT_UNHEALTHY, false);
		return bridge_clock_source;
	}
	if (!attempt_hxtal) {
		bridge_clock_fallback = BRIDGE_CLOCK_FB_NOT_REQUESTED;
		return bridge_clock_source;
	}
	(void)bridge_clock_attempt_hxtal(ops);
	return bridge_clock_source;
}

bool bridge_clock_on_ckm_nmi(const bridge_clock_ops_t *ops)
{
	/* Storm guard: a CKM NMI while nothing is armed and nothing is in flight
	 * means the acknowledge below did not hold -- do not loop on it, let the
	 * caller reset through the fault path (flags left for the record). */
	if (!s_busy && bridge_clock_source != BRIDGE_CLOCK_SRC_HXTAL) return false;

	ops->ckm_ack();
	if (s_busy) {
		s_ckm_pending = true; /* the thread side owns the fallback */
		return true;
	}
	fall_back(ops, BRIDGE_CLOCK_FB_CKM_FAILURE, true);
	return true;
}

void bridge_clock_mark_healthy(const bridge_clock_ops_t *ops)
{
	if (ops->marker_get() != 0u) {
		ops->marker_set(0u);
	}
}
