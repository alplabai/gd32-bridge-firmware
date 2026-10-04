/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * System-clock source selection: external 8 MHz (HXTAL, bypass mode) with a
 * guaranteed IRC8M fallback.  This header and clock_source.c are pure logic
 * with no vendor-header dependency; every register access goes through
 * bridge_clock_ops_t so the host tests drive the exact production sequence
 * against a fake.  The real ops live in clock_hw.c.
 *
 * The part has NO crystal.  OSCIN (PF0) is driven single-ended by the
 * 5L35023B SE2 output (nominally 8 MHz, net GD32_OSC), so HXTAL must run in
 * BYPASS mode (RCU_CTL_HXTALBPS) -- the stock vendor path never sets it,
 * which is why HXTALSTB never set and the shipped image runs from IRC8M.
 * SE2 is host-configured and may be off: nothing here may hang or brick a
 * unit in that state.
 */

#ifndef GD32_BRIDGE_HAL_GD32_CLOCK_SOURCE_H
#define GD32_BRIDGE_HAL_GD32_CLOCK_SOURCE_H

#include <stdbool.h>
#include <stdint.h>

/* Active PLL input.  SWD-readable via bridge_clock_source; not on the wire
 * (a protocol 0.15 field will carry it). */
typedef enum {
	BRIDGE_CLOCK_SRC_IRC8M = 0,
	BRIDGE_CLOCK_SRC_HXTAL = 1,
} bridge_clock_source_t;

/* Why the part is on IRC8M.  NONE while on HXTAL (and also before init). */
typedef enum {
	BRIDGE_CLOCK_FB_NONE                = 0,
	BRIDGE_CLOCK_FB_BUILD_DISABLED      = 1, /* built with BRIDGE_CLOCK_IRC8M_ONLY */
	BRIDGE_CLOCK_FB_HXTAL_TIMEOUT       = 2, /* HXTALSTB never set (SE2 off?) */
	BRIDGE_CLOCK_FB_PLL_FAIL            = 3, /* PLL would not lock/switch on HXTAL */
	BRIDGE_CLOCK_FB_CKM_FAILURE         = 4, /* HXTAL died after boot (CKM NMI) */
	BRIDGE_CLOCK_FB_PREV_BOOT_UNHEALTHY = 5, /* last HXTAL boot never reached a healthy tick */
} bridge_clock_fallback_t;

/* 8 MHz / PLLPSC 2 * PLLN 108 / PLLP 2 = 216 MHz, identical to the IRC8M
 * PLL tuple, so PWM/ADC/DWT constants and flash wait states are unchanged. */
#define BRIDGE_CLOCK_HXTAL_HZ 8000000u
#define BRIDGE_CLOCK_PLL_PSC  2u
#define BRIDGE_CLOCK_PLL_N    108u
#define BRIDGE_CLOCK_PLL_P    2u
_Static_assert(BRIDGE_CLOCK_HXTAL_HZ / BRIDGE_CLOCK_PLL_PSC * BRIDGE_CLOCK_PLL_N /
                       BRIDGE_CLOCK_PLL_P ==
                   216000000u,
               "HXTAL PLL tuple must yield the 216 MHz the timer constants assume");

/* Bounded HXTALSTB poll (loop iterations, not time: ~3-5 cycles each, so
 * roughly 15-25 ms at 216 MHz). */
#define BRIDGE_CLOCK_HXTAL_SPINS 1000000u

/* Backup-register marker (RTC_BKP9) set before the first HXTAL PLL switch
 * and cleared on the first healthy tick; found set at boot = skip HXTAL. */
#define BRIDGE_CLOCK_ATTEMPT_MAGIC 0x48545831u /* "HTX1" */

typedef struct {
	void (*hxtal_bypass_enable)(void); /* clear HXTALEN, then set HXTALBPS */
	void (*hxtal_enable)(void);        /* set HXTALEN (bypass already set) */
	bool (*hxtal_stable)(void);        /* RCU_CTL_HXTALSTB */
	void (*hxtal_stop)(void);          /* CKMEN, HXTALEN, HXTALBPS all cleared */
	/* Move SYSCLK to IRC8M, reprogram the PLL from `src`, lock, switch back
	 * to PLLP.  false = failed, SYSCLK left on IRC8M. */
	bool (*pll_select)(bridge_clock_source_t src);
	void (*ckm_enable)(void); /* RCU_CTL_CKMEN */
	uint32_t (*marker_get)(void);
	void (*marker_set)(uint32_t value);
} bridge_clock_ops_t;

/* Active source / fallback reason.  Non-static so a bench SWD read sees them. */
extern volatile bridge_clock_source_t   bridge_clock_source;
extern volatile bridge_clock_fallback_t bridge_clock_fallback;

/* Boot-time selection (uses the attempt marker).  Returns the active source. */
bridge_clock_source_t bridge_clock_select(const bridge_clock_ops_t *ops, uint32_t spins);

/* Start HXTAL (bypass before enable), wait bounded, move the PLL onto it and
 * arm the clock monitor.  On failure the part is left on the IRC8M PLL with
 * HXTAL stopped.  `pll_on_hxtal` = the PLL may currently be sourced from HXTAL
 * (deep-sleep relock), so it must be re-pointed at IRC8M on failure. */
bool bridge_clock_external_start(const bridge_clock_ops_t *ops, uint32_t spins, bool pll_on_hxtal);

/* HXTAL clock-monitor failure: stop HXTAL, put the PLL back on IRC8M, record. */
void bridge_clock_on_ckm_failure(const bridge_clock_ops_t *ops);

/* First healthy tick: clear the attempt marker. */
void bridge_clock_mark_healthy(const bridge_clock_ops_t *ops);

/* Real-hardware entry points (clock_hw.c). */
void bridge_clock_init(void);           /* boot select; call before SystemCoreClockUpdate() */
void bridge_clock_relock_prepare(void); /* Deep-sleep exit: restore HXTAL or fall back */
void bridge_clock_healthy(void);        /* first healthy tick */
bool bridge_clock_nmi_recover(uint32_t syscfg_stat); /* true = CKM-only NMI handled, resume */

#endif /* GD32_BRIDGE_HAL_GD32_CLOCK_SOURCE_H */
