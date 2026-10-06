/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * System-clock source selection: an external clock into OSCIN (HXTAL, bypass
 * mode) with a guaranteed IRC8M fallback.  This header and clock_source.c are
 * pure logic with no vendor-header dependency; every register access goes
 * through bridge_clock_ops_t so the host tests drive the exact production
 * sequence against a fake.  The real ops live in clock_hw.c.
 *
 * The part has NO crystal.  OSCIN (PF0) is driven single-ended by the
 * 5L35023B SE2 output (net GD32_OSC).  SE2 is programmable and, on current
 * units, only reaches its final configuration when U-Boot writes the clock
 * generator's volatile registers -- AFTER the GD32 has booted.  So the part
 * boots on the IRC8M PLL exactly as before, and moves to HXTAL only when the
 * host asks (bridge_clock_try_hxtal(), see clock_hw.c); an autonomous attempt
 * at boot exists behind -DBRIDGE_CLOCK_HXTAL_AT_BOOT=ON for an SoM revision
 * whose SE2 is right from POR.  HXTAL runs in BYPASS mode (RCU_CTL_HXTALBPS)
 * -- the stock vendor path never sets it, which is why HXTALSTB never set.
 *
 * The OSCIN frequency is not assumed: HXTAL/32 is counted against the core
 * clock and classified against the table of supported references below.
 * Anything else is refused.  The clock monitor only detects a STOPPED clock,
 * not a CHANGED one: once the PLL is on HXTAL the host must never reprogram
 * SE2 (the U-Boot sequence comes first; Linux and userspace must not touch
 * it), or the PLL follows the new frequency.
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
	BRIDGE_CLOCK_FB_CKM_FAILURE         = 4, /* HXTAL died after the switch (CKM NMI) */
	BRIDGE_CLOCK_FB_PREV_BOOT_UNHEALTHY = 5, /* last HXTAL attempt never reached a healthy tick */
	BRIDGE_CLOCK_FB_HXTAL_FREQ          = 6, /* HXTAL/32 matches no supported reference */
	BRIDGE_CLOCK_FB_IRC8M_PLL_FAIL      = 7, /* the IRC8M PLL itself would not relock: the part
	                                            * is left on the bare 8 MHz IRC8M */
	BRIDGE_CLOCK_FB_NOT_REQUESTED       = 8, /* booted on IRC8M; no HXTAL switch asked for yet */
} bridge_clock_fallback_t;

/* Supported OSCIN references and their PLL tuples (bridge_clock_ref_t table in
 * clock_source.c).  PLL input = OSCIN / PLLPSC, VCO = input * PLLN, SYSCLK =
 * VCO / PLLP with PLLP /2 (field = PLLPSC - 1, PLLP field 0).  The vendor
 * header gives only the field ranges (PLLPSC /1../16, PLLN 8..180, PLLP /2../8),
 * not the input/VCO limits, so every tuple stays at the proven IRC8M-path
 * neighbourhood (4 MHz in / 432 MHz VCO) except 24.576 MHz, whose 4.096 MHz /
 * 430.08 MHz is the closest point to it.
 *
 *   OSCIN        HXTAL/32   PLLPSC (field)  PLLN  PLL in     VCO         SYSCLK     band
 *    8     MHz   250   kHz  /2  (1)        108   4     MHz   432    MHz  216     MHz   +-3%
 *   12     MHz   375   kHz  /3  (2)        108   4     MHz   432    MHz  216     MHz   +-3%
 *   16     MHz   500   kHz  /4  (3)        108   4     MHz   432    MHz  216     MHz   +-3%
 *   20     MHz   625   kHz  /5  (4)        108   4     MHz   432    MHz  216     MHz   +-3%
 *   24.576 MHz   768   kHz  /6  (5)        105   4.096 MHz   430.08 MHz  215.04  MHz   +-1%
 *                                                                        (-0.44%)
 *
 * There is NO 24.000 MHz entry: every board feeds 24.576 MHz (SE2 DIV4 route,
 * and the same setting burned into the clock generator OTP).  The bands do not
 * overlap (the closest pair, 20 and 24.576 MHz, is 23% apart), so a match is
 * unambiguous.  25 MHz has no tuple (PLLN * 25 MHz / PLLPSC = 216 MHz * PLLP
 * needs PLLPSC * PLLP to be a multiple of 25, impossible with PLLPSC <= 16 and
 * PLLP in {2,4,6,8}) and MUST NEVER BE FED.  It is 1.7% above 24.576 MHz, so
 * whether it is refused depends on the IRC8M timebase error e: with the +-1%
 * band it passes as 24.576 MHz (SYSCLK 218.75 MHz, +1.3% over 216) only when
 * the IRC8M reads more than 0.7% fast, and is refused otherwise.  The IRC8M
 * datasheet tolerance is not in this tree, so this cannot be made absolute.
 * A 32.768 kHz OSCIN (the SE2 free-run default) never matches: HXTAL/32 =
 * 1.024 kHz is ~2 counts in the window, and HXTALSTB normally does not set
 * inside the startup budget either.
 *
 * SYSCLK therefore is NOT always 216 MHz.  Nothing may assume it: every timing
 * derivation reads the live clock (SystemCoreClock / bridge_core_clock_hz,
 * fed by bridge_clock_core_update()); the 216 MHz literals that remain
 * (PWM_TIMER_CLK_HZ, BRIDGE_ADC_PACE_CLK_HZ, ADC_READ_ADCCK_HZ) are NOMINAL
 * values for the IRC8M path, not used to program hardware. */
#define BRIDGE_CLOCK_IN_8MHZ     8000000u
#define BRIDGE_CLOCK_IN_12MHZ    12000000u
#define BRIDGE_CLOCK_IN_16MHZ    16000000u
#define BRIDGE_CLOCK_IN_20MHZ    20000000u
#define BRIDGE_CLOCK_IN_24P576   24576000u
#define BRIDGE_CLOCK_PLL_IN_HZ   4000000u /* the IRC8M path's PLL input */
#define BRIDGE_CLOCK_PLL_N       108u     /* the IRC8M path's PLLN */
#define BRIDGE_CLOCK_PLL_P       2u       /* PLLP, every tuple */
#define BRIDGE_CLOCK_IRC8M_HZ    8000000u
#define BRIDGE_CLOCK_CORE_HZ     216000000u /* IRC8M-path SYSCLK; the exact-216 entries too */
#define BRIDGE_CLOCK_DIV32_PRESC 32u        /* the on-chip HXTAL/32 trigger source */
_Static_assert(BRIDGE_CLOCK_PLL_IN_HZ *BRIDGE_CLOCK_PLL_N / BRIDGE_CLOCK_PLL_P ==
                   BRIDGE_CLOCK_CORE_HZ,
               "the IRC8M-path PLL tuple must yield 216 MHz");

typedef struct {
	uint32_t in_hz;            /* OSCIN */
	uint32_t pll_psc;          /* PLLPSC divider (field = pll_psc - 1) */
	uint32_t pll_n;            /* PLLN */
	uint32_t sysclk_hz;        /* in_hz / pll_psc * pll_n / BRIDGE_CLOCK_PLL_P */
	uint32_t band_lo_permille; /* accepted this far BELOW in_hz / 32 (per mille) */
	uint32_t band_hi_permille; /* accepted this far ABOVE in_hz / 32 (per mille) */
} bridge_clock_ref_t;

/* The table entry for an OSCIN frequency returned by the classifier, or NULL. */
const bridge_clock_ref_t *bridge_clock_ref_for(uint32_t in_hz);

/* Acceptance band per table entry (band_lo/hi_permille, measured on the
 * HXTAL/32 COUNT).  The count is taken against the core clock, i.e. the IRC8M
 * (or its PLL): an IRC8M that is fast by e gives a count LOW by e, so the low
 * side of the band absorbs a fast IRC8M and the high side a slow one.  The
 * IRC8M datasheet figure is NOT in this tree and must be checked; the bands are
 * a safety choice, not a measurement.
 *  - 24.576 MHz (the only source on real boards) is symmetric, +-1%: an IRC8M
 *    within 1% either way switches, one further off is refused (FB_HXTAL_FREQ,
 *    stay on IRC8M: the safe direction).  25 MHz (+1.7%, no tuple) needs an IRC8M
 *    more than 0.7% fast to be taken for 24.576 MHz.  25 MHz cannot come from any
 *    alp-sdk path: SE2 is only ever programmed by the alp-sdk U-Boot fixup
 *    (DIV4 route, 24.576 MHz), and it must never be fed.
 *  - 8/12/16/20 MHz (exact 216, no board feeds them) keep +-3%.
 *  - A true OSCIN deviation of up to band + IRC8M error can pass, so the core
 *    can run that much above its nominal SYSCLK in the worst case.
 * SystemCoreClock is derived from the CLASSIFIED reference. */
#define BRIDGE_CLOCK_BAND_24P576_LO_PERMILLE 10u
#define BRIDGE_CLOCK_BAND_24P576_HI_PERMILLE 10u
#define BRIDGE_CLOCK_BAND_OTHER_PERMILLE     30u

/* Every HXTAL/PLL wait is bounded by TIME (DWT CYCCNT against the live core
 * clock, clock_hw.c), never by an iteration count: the same loop is 27x slower
 * at 8 MHz than at 216 MHz, and the Deep-sleep wake runs at 8 MHz under
 * PRIMASK inside the 445 ms FWDGT window.
 *
 * Typical cost with a good clock is ~3 ms (HXTAL start well under 1 ms + the
 * 2 ms count window + PLL lock); SE2 off costs the 5 ms startup budget.  The
 * worst bounded case is HXTAL up but the PLL refusing it, then the IRC8M PLL
 * rebuilt: 5 + 2 + 8 + 8 = 23 ms (+ a few us of Vcore-step delays). */
#define BRIDGE_CLOCK_HXTAL_STARTUP_US 5000u /* HXTALSTB after HXTALEN */
#define BRIDGE_CLOCK_SWITCH_US        2000u /* SCSS follows an SCS write; PLLSTB drops */
#define BRIDGE_CLOCK_PLL_LOCK_US      2000u /* PLLSTB after PLLEN */
#define BRIDGE_CLOCK_FREQ_WINDOW_US   2000u /* HXTAL/32 count window */

/* Backup-register marker (RTC_BKP9) set before an HXTAL PLL switch and cleared
 * on the first healthy tick after it (an OTA trial included); found set at
 * boot = refuse HXTAL for that boot.
 *
 * Limits: it needs the backup domain to survive the reset, so it does NOT
 * help across a power cycle where VBAT is not retained -- a clock that wedges
 * the part on every cold boot is not caught.  A misclock that still reaches a
 * healthy tick is not caught by it either; the HXTAL/32 frequency check
 * before the PLL switch is what covers that. */
#define BRIDGE_CLOCK_ATTEMPT_MAGIC 0x48545831u /* "HTX1" */

typedef struct {
	void (*hxtal_bypass_enable)(void); /* CKMEN off, HXTALEN off, then HXTALBPS */
	void (*hxtal_enable)(void);        /* set HXTALEN (bypass already set) */
	bool (*hxtal_wait_stable)(void);   /* HXTALSTB within BRIDGE_CLOCK_HXTAL_STARTUP_US */
	/* HXTAL/32 edges counted over `window_us` (timebase: the core clock).
	 * Called with the PLL NOT yet on HXTAL. */
	uint32_t (*hxtal_div32_count)(uint32_t window_us);
	void (*hxtal_stop)(void); /* CKMEN, HXTALEN, HXTALBPS all cleared, in that order */
	/* Move SYSCLK to IRC8M (stepping AHB down first when leaving the PLL),
	 * reprogram the PLL from `src` (`input_hz` = a table OSCIN frequency, only
	 * read for HXTAL; its tuple comes from bridge_clock_ref_for()), lock,
	 * restore AHB /1, switch back to PLLP.
	 * false = failed, SYSCLK left on IRC8M. */
	bool (*pll_select)(bridge_clock_source_t src, uint32_t input_hz);
	void (*ckm_enable)(void); /* clear stale CKM flags, then RCU_CTL_CKMEN */
	/* NMI acknowledge: CKMEN off first (it cannot re-fire), then
	 * RCU_INT.CKMIC, then SYSCFG_STAT.CKMNMIIF -- in that order. */
	void (*ckm_ack)(void);
	uint32_t (*marker_get)(void);
	void (*marker_set)(uint32_t value);
} bridge_clock_ops_t;

/* Active source / fallback reason / detected OSCIN frequency (one of the
 * BRIDGE_CLOCK_IN_* values, or 0 = unknown / HXTAL not in use).  Non-static so
 * a bench SWD read sees them. */
extern volatile bridge_clock_source_t   bridge_clock_source;
extern volatile bridge_clock_fallback_t bridge_clock_fallback;
extern volatile uint32_t                bridge_clock_input_hz;

/* HXTAL/32 count over `window_us` -> the matching BRIDGE_CLOCK_IN_* frequency
 * when within that entry band (no two bands overlap), else 0. */
uint32_t bridge_clock_classify_div32(uint32_t counts, uint32_t window_us);

/* Boot-time selection.  An attempt marker left by a previous HXTAL attempt
 * that never got healthy refuses HXTAL for this boot (FB_PREV_BOOT_UNHEALTHY).
 * Otherwise `attempt_hxtal` false leaves the part on IRC8M (FB_NOT_REQUESTED)
 * and true runs bridge_clock_attempt_hxtal().  Returns the active source. */
bridge_clock_source_t bridge_clock_boot(const bridge_clock_ops_t *ops, bool attempt_hxtal);

/* The host-requested switch: marker, then the full sequence below.  true when
 * the PLL runs from HXTAL afterwards (including "already was").  false with
 * bridge_clock_fallback saying why; a boot that found the marker refuses. */
bool bridge_clock_attempt_hxtal(const bridge_clock_ops_t *ops);

/* Start HXTAL (bypass before enable), wait bounded, check the frequency, move
 * the PLL onto it and arm the clock monitor.  On failure the part is left on
 * the IRC8M PLL with HXTAL stopped.  `pll_on_hxtal` = the PLL may currently be
 * sourced from HXTAL (deep-sleep relock), so it must be re-pointed at IRC8M on
 * failure.  The whole call is a "sequence in flight": a CKM NMI landing in it
 * only records, and this thread-side code performs the fallback. */
bool bridge_clock_external_start(const bridge_clock_ops_t *ops, bool pll_on_hxtal);

/* CKM NMI (NMI context).  false = not recoverable (a second CKM NMI while
 * already on IRC8M -- a storm -- or nothing to recover): the caller must take
 * the fault path.  true = acknowledged; the fallback was either done here or
 * deferred to the sequence in flight. */
bool bridge_clock_on_ckm_nmi(const bridge_clock_ops_t *ops);

/* First healthy tick: clear the attempt marker. */
void bridge_clock_mark_healthy(const bridge_clock_ops_t *ops);

/* ---- Real-hardware entry points (clock_hw.c) ---- */

/* Boot: marker check, then (BRIDGE_CLOCK_HXTAL_AT_BOOT only) the HXTAL
 * attempt.  `fwdgt_running` = the FWDGT is armed and may be fed during the
 * waits.  Call before bridge_clock_core_update(). */
void bridge_clock_init(bool fwdgt_running);

/* Switch status, SWD-readable now and the field the protocol 0.15 opcode will
 * report later (alplabai/gd32-bridge-firmware#330 tracks that wire opcode and
 * the fields).  `bridge_clock_fallback` carries the reason once DONE_*.
 *   IDLE           nothing requested (or the last result was consumed by a new request)
 *   PENDING        request latched; waiting for both transports to be quiet
 *   RUNNING        the sequence is executing (PRIMASK held, <= ~23 ms)
 *   DONE_HXTAL     the PLL runs from HXTAL
 *   DONE_FALLBACK  refused by the sequence (see bridge_clock_fallback); on IRC8M
 *   REFUSED_BUSY   not attempted: a PWM channel or ADC stream is active, or the
 *                  buses were never quiet; NOTHING stays latched
 * Bus-quiet contract: the switch runs only after the request has been latched
 * AND both transports have been idle (CS high, no I2C transaction or address
 * match pending) for BRIDGE_CLOCK_QUIET_TICKS consecutive 50 ms ticks.  The host
 * must keep BOTH buses idle from the request until the status leaves RUNNING:
 * the core clock stretches ~27x while the PLL is rebuilt and a transfer in flight
 * can fail its CRC.  A request that cannot get quiet buses within
 * BRIDGE_CLOCK_PENDING_MAX_TICKS ends REFUSED_BUSY. */
typedef enum {
	BRIDGE_CLOCK_SW_IDLE          = 0,
	BRIDGE_CLOCK_SW_PENDING       = 1,
	BRIDGE_CLOCK_SW_RUNNING       = 2,
	BRIDGE_CLOCK_SW_DONE_HXTAL    = 3,
	BRIDGE_CLOCK_SW_DONE_FALLBACK = 4,
	BRIDGE_CLOCK_SW_REFUSED_BUSY  = 5,
} bridge_clock_switch_status_t;
extern volatile bridge_clock_switch_status_t bridge_clock_switch_status;
#define BRIDGE_CLOCK_QUIET_TICKS       3u
#define BRIDGE_CLOCK_PENDING_MAX_TICKS 200u /* ~10 s */

/* The internal seam the protocol layer will call for the host's "switch to the
 * external clock" command (the wire opcode is part of the protocol 0.15 work,
 * not here).  Base-level, NOT from a transport ISR: it holds PRIMASK for up to
 * ~23 ms.  It REFUSES (REFUSED_BUSY, false, no register touched) while any PWM
 * channel or ADC stream is active: supervised outputs must never see the period
 * stretch.  It does not wait for quiet buses; that is the tick path.  The
 * ISR-safe form is bridge_clock_request_hxtal(): it latches, and
 * bridge_clock_tick() (from bridge_hw_tick()) applies the bus-quiet contract
 * above and then runs it.  Writing 1 to the SWD variable
 * bridge_clock_hxtal_request does the same for the bench.  Returns true when the
 * PLL runs from HXTAL afterwards. */
bool bridge_clock_try_hxtal(void);
void bridge_clock_request_hxtal(void);
void bridge_clock_tick(void); /* advance a latched request, else clear the attempt marker */
extern volatile uint32_t bridge_clock_hxtal_request;

void bridge_clock_pre_deepsleep(void); /* before Deep-sleep entry: CKMEN off (HXTAL stops) */
/* Deep-sleep exit: restore HXTAL or fall back.  true = the PLL is already
 * running and selected (the caller must NOT replay PLLEN/SCS); false = the
 * caller replays the PLL as for IRC8M.  A WFI that returned without sleeping
 * leaves HXTAL and the PLL live: that is just a CKM re-arm, never a restart. */
bool bridge_clock_relock_prepare(void);
bool bridge_clock_nmi_recover(uint32_t syscfg_stat); /* true = CKM-only NMI handled, resume */

/* SystemCoreClock from the LIVE RCU state.  The vendor SystemCoreClockUpdate()
 * hardcodes HXTAL_VALUE (8 MHz) as the PLL input, which is wrong for any other
 * OSCIN, so nothing in this firmware calls it. */
void bridge_clock_core_update(void);

/* Telemetry for bridge_core_clock_matches: SYSCLK equals what the active
 * clock selection is meant to produce (the table entry's sysclk_hz on HXTAL,
 * 216 MHz on the IRC8M PLL).  false on a bare 8 MHz IRC8M after a failed PLL. */
bool bridge_clock_core_matches(void);
uint32_t
bridge_clock_apb1_hz(void); /* live CK_APB1 (the vendor rcu_clock_freq_get has the same flaw) */

#endif /* GD32_BRIDGE_HAL_GD32_CLOCK_SOURCE_H */
