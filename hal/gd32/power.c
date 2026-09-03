/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 bridge HAL backend -- system power modes.
 * Split move-only from hal/bridge_hw_gd32.c (fw v0.2.8); see
 * hal/gd32/init.c for the backend-wide implementation notes.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bridge_hw.h"
#include "gd32g5x3.h"

#include "bridge_board_config.h" /* BRIDGE_I2C_PERIPH */
#include "gd32_common.h"
#include "transport.h" /* bridge_transport_i2c_hw_init() */

/* ----------------------------------------------------------------- */
/* v0.5 (§2B.3) -- system power-mode set                             */
/* ----------------------------------------------------------------- */

/* ALP_POWER_WAKE_* bits the firmware supports (mirrors the wire
 * encoding in <alp/power.h>).
 *
 * Mapping notes per V2N hardware reality:
 *
 *   - GPIO : routes through PMU_WAKEUP_PIN0..4 -- five fixed pads
 *            on the GD32G553 + board wires the desired triggers
 *            onto them.  Landed §C.15c.
 *   - RTC  : RTC alarm 0 fires on a scheduled wallclock; the
 *            wakeup timer also surfaces under this bit so the
 *            firmware uses the timer (simpler than absolute-time
 *            alarms in a partial bring-up).  Landed §C.25.
 *   - TIMER: any non-zero `wake_after_ms` -- same RTC wakeup-timer
 *            path as RTC.  The bit is redundant when wake_after_ms
 *            > 0 (the timer wakes the chip implicitly per the
 *            <alp/power.h> contract); honouring the bit explicitly
 *            lets a caller arm a "wake on next tick" without a
 *            specific deadline.  Landed §C.25.
 *   - UART_RX / USB / ETH_LINK : no hardware path on the GD32G5
 *            (no LPUART wake / no USB OTG / no MAC).  Future SoCs
 *            on the bridge slot could populate these via the same
 *            opcode; today the firmware rejects them so the host
 *            knows the request is moot.
 */
#define POWER_WAKE_RTC            0x00000001u
#define POWER_WAKE_GPIO           0x00000002u
#define POWER_WAKE_UART_RX        0x00000004u
#define POWER_WAKE_TIMER          0x00000008u
#define POWER_WAKE_USB            0x00000010u
#define POWER_WAKE_ETH_LINK       0x00000020u
#define POWER_WAKE_MASK_SUPPORTED (POWER_WAKE_RTC | POWER_WAKE_GPIO | POWER_WAKE_TIMER)
#define POWER_WAKE_MASK_HW_GATED  (POWER_WAKE_UART_RX | POWER_WAKE_USB | POWER_WAKE_ETH_LINK)

/* RTC wakeup timer LSB.  POWER_WAKE_LSB_HZ = 2000 Hz assumes an exact
 * 32000 Hz IRC32K with the /16 divider -- 0.5 ms/tick nominal, max
 * wake 65535/2000 = 32.7 s -- but IRC32K is NOT a fixed 32000 Hz.
 * LXTAL is unavailable on this SoM (PC14/PC15 are spent as host
 * E1M IO24/IO25) so IRC32K is the only legal RTC source, and it is
 * specified only as 28-36 kHz over this grade-7 part's -40..105 degC
 * range (GD32G553xx Datasheet Rev2.0 p.125 Table 4-23; full citation
 * + why LXTAL is unavailable is in hal/bridge_board_config.h).  The
 * real tick is therefore 444 us (36 kHz corner) to 571 us (28 kHz
 * corner) against the 500 us this constant assumes -- about
 * -11.1% / +14.3% on any requested wake_after_ms, and the 32.7 s
 * ceiling can land anywhere from ~29.1 s to ~37.4 s.  No firmware
 * change closes this: IRC32K has no trim register in the RCU, and
 * the RTC's own RTC_HRFC digital calibration corrects a fixed
 * offset against a reference clock, not IRC32K's drift over
 * temperature (see hal/bridge_board_config.h).  Longer waits than
 * the ceiling would need the CKSPRE_2EXP16 mode, which sits in a
 * future commit. */
#define POWER_WAKE_LSB_HZ       2000u
#define POWER_WAKE_TIMER_MAX_MS (65535u * 1000u / POWER_WAKE_LSB_HZ)

/* One-time RTC + LSI bring-up that arms the wakeup timer.  Idempotent
 * across multiple power_mode_set calls -- the LSI stays enabled, the
 * RTC source latches to IRC32K once.  Failure (LSI never stabilises,
 * write-protected register won't unlock) leaves rtc_wakeup_ready
 * false and bridge_hw_power_mode_set returns NOSUPPORT for any
 * timer-bearing call. */
static bool rtc_wakeup_ready = false;

static bool rtc_wakeup_init_once(void)
{
	if (rtc_wakeup_ready) return true;

	/* Bring up IRC32K (internal LSI) as the RTC clock source. */
	rcu_osci_on(RCU_IRC32K);
	/* Spin until IRC32K stabilises -- typical < 50 us, the upper
     * bound keeps a dead oscillator from hanging the bridge. */
	uint32_t to = 200000u;
	while (--to && RESET == rcu_flag_get(RCU_FLAG_IRC32KSTB)) {
		/* spin */
	}
	if (to == 0u) return false;

	rcu_periph_clock_enable(RCU_PMU);
	pmu_backup_write_enable();
	rcu_rtc_clock_config(RCU_RTCSRC_IRC32K);
	rcu_periph_clock_enable(RCU_RTC);

	rtc_wakeup_ready = true;
	return true;
}

static int rtc_wakeup_arm_ms(uint32_t wake_after_ms)
{
	if (!rtc_wakeup_init_once()) return BRIDGE_HW_ERR_IO;
	if (wake_after_ms > POWER_WAKE_TIMER_MAX_MS) return BRIDGE_HW_ERR_RANGE;

	/* Compute ticks (round up so a sub-LSB request still waits at
     * least one tick rather than zero). */
	uint32_t ticks = (wake_after_ms * POWER_WAKE_LSB_HZ + 999u) / 1000u;
	if (ticks == 0u) ticks = 1u;
	if (ticks > 65535u) ticks = 65535u;

	/* The vendor sequence: disable the wakeup timer, switch its
     * clock source, set the counter, re-enable.  rtc_wakeup_disable
     * may return ERROR if the WTWF flag never sets; treat as IO. */
	if (SUCCESS != rtc_wakeup_disable()) return BRIDGE_HW_ERR_IO;
	if (SUCCESS != rtc_wakeup_clock_set(WAKEUP_RTCCK_DIV16)) return BRIDGE_HW_ERR_IO;
	if (SUCCESS != rtc_wakeup_timer_set((uint16_t)(ticks - 1u))) return BRIDGE_HW_ERR_IO;
	rtc_wakeup_enable();
	return BRIDGE_HW_OK;
}

static void power_wake_pins_enable(uint32_t wake_bitmap)
{
	if ((wake_bitmap & POWER_WAKE_GPIO) == 0u) return;
	pmu_wakeup_pin_enable(PMU_WAKEUP_PIN0);
	pmu_wakeup_pin_enable(PMU_WAKEUP_PIN1);
	pmu_wakeup_pin_enable(PMU_WAKEUP_PIN2);
	pmu_wakeup_pin_enable(PMU_WAKEUP_PIN3);
	pmu_wakeup_pin_enable(PMU_WAKEUP_PIN4);
}

int bridge_hw_power_mode_set(uint8_t mode, uint32_t wake_bitmap, uint32_t wake_after_ms)
{
	/* Mode 0 (run) + mode 1 (sleep) are accepted no-ops -- main()'s
     * `for (;;) { __WFI(); bridge_hw_tick(); }` already runs the CPU
     * in WFI between transport interrupts, which IS "sleep" on the
     * GD32G5.  Mode 2 (deep-sleep) + mode 3 (standby) call into the
     * vendor's PMU helpers below.
     *
     * Wake-source semantics: `wake_bitmap` enumerates the explicit
     * sources the host wants armed; `wake_after_ms` is a timed
     * fallback that arms the RTC wakeup timer regardless of the
     * bitmap (per the <alp/power.h> contract: the timer is implicit
     * when wake_after_ms > 0).  Unsupported bits (UART_RX / USB /
     * ETH_LINK on the GD32G5 baseline) reject so the host knows the
     * request was not honoured. */
	if ((wake_bitmap & POWER_WAKE_MASK_HW_GATED) != 0u) return BRIDGE_HW_ERR_NOTIMPL;

	switch (mode) {
	case 0u: /* run -- no-op */
	case 1u: /* sleep -- already in WFI between transport ISRs */
		return BRIDGE_HW_OK;
	case 2u: /* deep-sleep */
		rcu_periph_clock_enable(RCU_PMU);
		power_wake_pins_enable(wake_bitmap);
		if (wake_after_ms != 0u || (wake_bitmap & (POWER_WAKE_RTC | POWER_WAKE_TIMER)) != 0u) {
			const uint32_t ms = (wake_after_ms != 0u) ? wake_after_ms : POWER_WAKE_TIMER_MAX_MS;
			int            rc = rtc_wakeup_arm_ms(ms);
			if (rc != BRIDGE_HW_OK) return rc;
		}
		/* I2C0 is one of the modules the PMU keeps powered across
         * Deep-sleep (UM Rev1.2 p.132, Fig 3-2: "Deep-sleep power on
         * modules: ... I2C0 ..."), so I2CEN stays live across this
         * mode unless software disables it first.  UM Rev1.2 SS28.3.11
         * (p.1279): "Before entering power saving mode (I2CEN = 0), the
         * I2C peripheral must be disabled if wakeup from power saving
         * mode is disabled (WUEN = 0)."  This firmware never sets WUEN
         * (bridge_transport_i2c_hw_init() does not touch it), so every
         * prior Deep-sleep entry ran I2C0 outside its documented
         * operating procedure.  Disable it here and bring it fully back
         * up on the way out. */
		i2c_disable(BRIDGE_I2C_PERIPH);
		/* PMU_LDO_LOWPOWER drops the core LDO into its low-power
         * regulation point during deepsleep (saves a few hundred
         * uA at the cost of a slightly slower wakeup); WFI_CMD
         * issues the actual `wfi` instruction that suspends the
         * core.  Returns here once a wakeup source fires. */
		pmu_to_deepsleepmode(PMU_LDO_LOWPOWER, WFI_CMD);
		/* Wake path.  UM Rev1.2 p.142 "Deep-sleep mode": "all of IRC8M,
         * HXTAL and PLL are disabled ... When exiting the Deep-sleep
         * mode, the IRC8M is selected as the system clock" -- and this
         * firmware has no PLL-relock step anywhere (gh#12, tracked
         * separately), so CK_APB1 can still be running on ~8 MHz IRC8M
         * rather than the normal 216 MHz PLL when this line executes.
         * bridge_transport_i2c_hw_init() redoes the GPIO/timing/address/
         * IRQ setup and re-runs i2c_enable(), so the peripheral comes
         * back up in a known state and a half-finished transaction from
         * before sleep cannot resume against it -- that part is correct
         * regardless of clock.  Its I2C_TIMING fields are now derived
         * from the LIVE CK_APB1 on every call (not a 216-MHz-only
         * constant), so this call is correctly timed at whatever clock
         * CK_APB1 is actually running on here -- including 8 MHz IRC8M,
         * with no PLL relock required for I2C0 specifically.  If the
         * derivation still refuses (BRIDGE_HW_ERR_RANGE: the live
         * apb1_hz is below the Fast-mode floor or produced fields that
         * don't fit their 4-bit registers), I2C0 is left disabled by
         * that function rather than brought up mistimed -- surface that
         * to the caller instead of claiming BRIDGE_HW_OK for a deep-sleep
         * transition that left the I2C bridge transport down. */
		return bridge_transport_i2c_hw_init();
	case 3u: /* standby */
		rcu_periph_clock_enable(RCU_PMU);
		power_wake_pins_enable(wake_bitmap);
		if (wake_after_ms != 0u || (wake_bitmap & (POWER_WAKE_RTC | POWER_WAKE_TIMER)) != 0u) {
			const uint32_t ms = (wake_after_ms != 0u) ? wake_after_ms : POWER_WAKE_TIMER_MAX_MS;
			int            rc = rtc_wakeup_arm_ms(ms);
			if (rc != BRIDGE_HW_OK) return rc;
		}
		/* Standby powers down the core + SRAM (except backup) and
         * wakes via reset -- pmu_to_standbymode() never returns;
         * the SoC re-runs Reset_Handler when a wakeup source fires.
         * The caller's host link will see the bridge re-issue its
         * handshake on the next transport packet, which is the
         * documented contract. */
		pmu_to_standbymode();
		/* Unreachable in normal operation; keep the return so the
         * compiler doesn't warn about a missing terminator. */
		return BRIDGE_HW_OK;
	default:
		return BRIDGE_HW_ERR_INVAL;
	}
	(void)POWER_WAKE_MASK_SUPPORTED;
}
