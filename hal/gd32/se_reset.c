/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 bridge HAL backend -- secure-element reset line.
 *
 * SE_RST = GD32 PC13 drives the on-module OPTIGA Trust M's reset
 * (the SE is a slave on the shared BRD_I2C management bus, per
 * alp-sdk metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv).  Before this TU the
 * pin was left at its GPIO power-on-reset default -- ANALOG mode, per
 * GD32G553_CTL reset value 0xFFFF FFFF for port C (UM Rev1.2 p.275
 * §7.4.1, CTL13[1:0] = 11), not a floating INPUT -- so the host had no
 * way to reset a secure element that had clock-stretched SCL low and
 * wedged BRD_I2C -- the failure mode that blocks reading the 5L35023B
 * PCIe-refclk generator on the V2N-M1 DEEPX bring-up.
 *
 * se_reset_init() (called from bridge_hw_init) parks the pin DEASSERTED
 * at boot so the SE runs and the line is no longer analog/floating.
 * se_reset_configure() preloads GPIOC_OCTL13 to the released level
 * BEFORE flipping CTL13 to OUTPUT, so the pad never transits its
 * asserted (reset) level on the way there (gh#61) -- OCTL resets to
 * 0 (UM Rev1.2 p.282 §7.4.6), so promoting to OUTPUT first would drive
 * PC13 LOW, the SE's active-LOW asserted level, for the gap until the
 * level write caught up.  The host pulses SE_RST via CMD_SE_RESET to
 * recover the bus.
 *
 * PC13 is also a power-switch pad sharing a 3 mA / 2 MHz / 30 pF
 * budget with PC14/PC15 (E1M IO24/IO25, hal/gd32/gpio.c) -- see the
 * block comment there and gh#60.
 *
 * REMAINING WINDOW (gh#61, not closed by this TU): se_reset_init()
 * being the first thing bridge_hw_init() does only bounds the part of
 * boot from main() onward.  Before main() runs, Reset_Handler
 * (vendors/gd32_firmware_library upstream
 * .../CMSIS/GD/GD32G5x3/Source/GCC/startup_gd32g5x3.S, SramInit: loop)
 * reads SRAM_DENSITY[15:0] at 0x1FFFB3E0 (UM Rev1.2 p.89 §1.8.1,
 * value in KBytes) and zeroes that many bytes of SRAM one word at a
 * time BEFORE calling SystemInit or main.  GD32G553MEY7TR reports 128
 * KB total on-chip SRAM (Datasheet Rev2.0 p.11 part-number table) =
 * 32768 four-instruction loop iterations (stm/subs/cmp/bne), and the
 * loop runs at the RCU reset-default clock, not the 216 MHz
 * SystemInit() later programs: RCU_CFG0 resets to 0x0000_0000 (UM
 * Rev1.2 p.164 §4.3.3), SCS[1:0] = 00 selects CK_IRC8M = 8 MHz
 * (p.166), AHBPSC[3:0] = 0000 applies no division.  At roughly 5-6
 * cycles/iteration that is ~164k-197k cycles, i.e. ~20-25 ms at 8
 * MHz -- not the sub-microsecond OCTL-preload gap gh#61 closed, and
 * not the ~1 ms TRNG figure this backend's comments cite elsewhere;
 * this window is roughly TWENTY TIMES that.  PC13 sits ANALOG/floating
 * (UM Rev1.2 p.270 §7.3.7: pull-up/down disabled, output buffer
 * disabled) for the entire ~20-25 ms.
 *
 * This is NOT fixable from bridge_hw_init(), or from anything else
 * that runs after main() is reached -- by the time se_reset_init()
 * is the first bridge_hw_init() statement, the SramInit loop is
 * already 20+ ms in the past.  A SystemInit() hook or a C++-style
 * constructor could in principle run earlier, but SramInit executes
 * before either of those too (it precedes __libc_init_array, which is
 * what runs constructors, and precedes the bl SystemInit call itself)
 * -- so no firmware hook available in this toolchain's reset sequence
 * can win the race.  Left unaddressed, the honest fix is external:
 * a pull resistor on PC13 at the board level, sized so the SE reads a
 * defined (released) level while the pin is ANALOG.  Tracked on gh#61;
 * do not read that issue as closed by this file.
 */

#include <stdbool.h>
#include <stdint.h>

#include "bridge_hw.h"
#include "gd32g5x3.h"

#include "gd32_common.h"

/* OPTIGA Trust M RST is active-LOW: drive the pin LOW to hold the SE
 * in reset, HIGH to release it.  This is the OPTIGA Trust M datasheet
 * convention; VERIFY against the E1M-X schematic on first bench
 * bring-up.  If the SE does not recover on a pulse, flip this to 0. */
#define SE_RST_ACTIVE_LOW 1

#define SE_RST_PORT GPIOC
#define SE_RST_PIN  GPIO_PIN_13

static bool se_rst_configured;

/* Drive SE_RST to its asserted (held-in-reset) or released level,
 * resolving the active-low polarity into a physical pin level. */
static void se_reset_drive(uint8_t assert)
{
#if SE_RST_ACTIVE_LOW
	const bool pin_high = (assert == 0u); /* release = HIGH, assert = LOW */
#else
	const bool pin_high = (assert != 0u);
#endif
	if (pin_high) {
		gpio_bit_set(SE_RST_PORT, SE_RST_PIN);
	} else {
		gpio_bit_reset(SE_RST_PORT, SE_RST_PIN);
	}
}

/* Promote PC13 to a push-pull output.  RCU_GPIOC is already clocked by
 * bridge_hw_init(); 12 MHz is the slowest output speed (low EMI, ample
 * for a reset control line; PC13 is also a power-switch pad capped at
 * 2 MHz output toggle rate regardless -- see the gpio.c pad-map block
 * comment and gh#60).
 *
 * Preload OCTL13 to the RELEASED level via BOP (offset 0x18, UM
 * Rev1.2 p.283 §7.4.7) BEFORE flipping CTL13 to OUTPUT: OCTL resets
 * to 0 (UM Rev1.2 p.282 §7.4.6), so promoting to OUTPUT first would
 * drive PC13 to its active-LOW asserted level for the gap until the
 * level write caught up, pulsing the OPTIGA Trust M into reset on
 * every boot (gh#61). Writing OCTL while the pad is still ANALOG is
 * harmless -- push-pull mode is not active yet. */
static void se_reset_configure(void)
{
#if SE_RST_ACTIVE_LOW
	gpio_bit_set(SE_RST_PORT, SE_RST_PIN); /* GPIOC_BOP BOP13: OCTL13 <- 1 (released) */
#else
	gpio_bit_reset(SE_RST_PORT, SE_RST_PIN); /* GPIOC_BOP CR13: OCTL13 <- 0 (released) */
#endif
	gpio_output_options_set(SE_RST_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_12MHZ, SE_RST_PIN);
	gpio_mode_set(SE_RST_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, SE_RST_PIN);
	se_rst_configured = true;
}

void se_reset_init(void)
{
	se_reset_configure();
	se_reset_drive(0u); /* boot default: SE released (running), not floating */
}

int bridge_hw_se_reset(uint8_t assert)
{
	if (assert > 1u) return BRIDGE_HW_ERR_INVAL;
	if (!se_rst_configured) se_reset_configure(); /* lazy guard if init skipped */
	se_reset_drive(assert);
	return BRIDGE_HW_OK;
}
