/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 bridge HAL backend -- GPIO pads.
 * Split move-only from hal/bridge_hw_gd32.c (fw v0.2.8); see
 * hal/gd32/init.c for the backend-wide implementation notes.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bridge_hw.h"
#include "gd32g5x3.h"

#include "gd32_common.h"

/* ----------------------------------------------------------------- */
/* GPIO pad map -- E1M IO logical-index to GD32 (port, pin) lookup.   */
/* Sourced from `metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv`        */
/* (the "E1M IO*" rows).  Wire-side `mask` bit i selects entry i in   */
/* this table; numbering is compact (0..17) rather than matching the  */
/* physical E1M IO numbering, which has gaps at 15 / 17..23 / 26 / 33 */
/* because those positions are assigned to other peripherals on the   */
/* board.  Host-side translation table lives in                     */
/* `chips/gd32g553/gd32g553.c`.                                       */
/*                                                                    */
/* Bits 8/9 (PC14/PC15, E1M IO24/IO25) are NOT ordinary pads: they    */
/* are supplied through the backup-domain power switch together with */
/* SE_RST (PC13, hal/gd32/se_reset.c), sharing a typical 3 mA source  */
/* budget, capped at 2 MHz output toggle rate with a 30 pF max load,  */
/* and excluded from the datasheet's output voltage characterisation */
/* (GD32G553xx Datasheet Rev2.0 p.130 Table 4-29 footnote 2; GD32G553 */
/* User Manual Rev1.2 p.133 §3.3.1 -- both confirmed unchanged in     */
/* Rev1.6/Rev1.3).  `GPIO_OSPEED_12MHZ` below is already OSPD = 00,   */
/* the slowest class the part offers (UM Rev1.2 p.280 §7.4.3), so the */
/* 2 MHz cap cannot be met by a register change -- the HOST must not  */
/* toggle IO24/IO25 faster than 2 MHz or load them beyond 30 pF, and  */
/* current drawn through them competes with the milliamps holding    */
/* SE_RST released.  See gh#60.  PC14/PC15 land here as plain GPIO    */
/* rather than LXTAL (OSC32_IN/OSC32_OUT) because this backend never  */
/* selects LXTAL as the RTC clock source -- hal/gd32/power.c hardcodes*/
/* rcu_rtc_clock_config(RCU_RTCSRC_IRC32K) -- so the pins are free for*/
/* E1M IO use by construction.  See gh#64 for that reasoning.         */
/* ----------------------------------------------------------------- */
/*                                                                    */
/* Nothing below cross-checks this table against the pins already    */
/* claimed by other peripheral tables in this backend: dac_channels[] */
/* (hal/gd32/dac.c: PA4, PA6, analog mode), adc_channels_map[]        */
/* (hal/gd32/adc.c: PD9, PB12, PE13, PE11, PC4, PA5, PA2, PA3, analog */
/* mode), pwm_channels[] (hal/gd32/pwm.c: PA11, PB1, PB14, PC5, PC10, */
/* PC11, PC12, PD0, AF mode), qenc_map[] (hal/gd32/qenc.c: PA0, PB3,  */
/* PC6, PC7, PB6, PB7, PB2, PA1, AF mode).  None overlap today -- a   */
/* C _Static_assert can't compare these against each other, since     */
/* each table is a separate TU's array literal and there is no       */
/* single translation unit that sees all of them at once.  But if a   */
/* FUTURE entry here ever reuses an ADC or DAC pin, the reused pad's   */
/* ISTAT bit is hardware-forced to 0 whenever the pin is in Analog    */
/* mode (UM Rev1.2 p.270 §7.3.7: "The port input status register of  */
/* this I/O port bit is '0'"), so bridge_hw_gpio_read()'s unconditional*/
/* gpio_input_bit_get() would silently and permanently report LOW for */
/* that bit -- the exact class of silent wrong answer gh#62 just      */
/* fixed for OCTL, relocated to ISTAT.  Grep the four tables above     */
/* before adding a pad here. */
/* ----------------------------------------------------------------- */

const gd32_gpio_pad_t gpio_pad_map[] = {
	{ GPIOB, GPIO_PIN_10 }, /* bit  0 = E1M IO8  */
	{ GPIOA, GPIO_PIN_7 },  /* bit  1 = E1M IO9  */
	{ GPIOA, GPIO_PIN_12 }, /* bit  2 = E1M IO10 */
	{ GPIOB, GPIO_PIN_0 },  /* bit  3 = E1M IO11 */
	{ GPIOC, GPIO_PIN_1 },  /* bit  4 = E1M IO12 */
	{ GPIOF, GPIO_PIN_1 },  /* bit  5 = E1M IO13 */
	{ GPIOB, GPIO_PIN_5 },  /* bit  6 = E1M IO14 */
	{ GPIOC, GPIO_PIN_0 },  /* bit  7 = E1M IO16 */
	{ GPIOC, GPIO_PIN_14 }, /* bit  8 = E1M IO24 -- power-switch pad, see block comment above */
	{ GPIOC, GPIO_PIN_15 }, /* bit  9 = E1M IO25 -- power-switch pad, see block comment above */
	{ GPIOB, GPIO_PIN_11 }, /* bit 10 = E1M IO27 */
	{ GPIOC, GPIO_PIN_2 },  /* bit 11 = E1M IO28 */
	{ GPIOD, GPIO_PIN_11 }, /* bit 12 = E1M IO29 */
	{ GPIOD, GPIO_PIN_10 }, /* bit 13 = E1M IO30 */
	{ GPIOE, GPIO_PIN_12 }, /* bit 14 = E1M IO31 */
	{ GPIOD, GPIO_PIN_2 },  /* bit 15 = E1M IO32 */
	{ GPIOD, GPIO_PIN_8 },  /* bit 16 = E1M IO34 */
	{ GPIOD, GPIO_PIN_1 },  /* bit 17 = E1M IO35 */
};
_Static_assert(sizeof(gpio_pad_map) / sizeof(gpio_pad_map[0]) == GPIO_PAD_MAP_COUNT,
               "gpio_pad_map size must match GPIO_PAD_MAP_COUNT");

/* Per-pad direction tracking.  Boot configures every pad as INPUT +
 * PULL_UP; bridge_hw_gpio_write() flips an entry to OUTPUT push-pull
 * on first call (sticky until the next chip reset).  Avoids the
 * need for a separate `CMD_GPIO_CONFIGURE` opcode.  Used ONLY by
 * bridge_hw_gpio_write() to decide whether a pad still needs
 * promoting -- bridge_hw_gpio_read() below always reads the measured
 * pad level regardless of this flag (gh#62). */
bool gpio_is_output[GPIO_PAD_MAP_COUNT];

int bridge_hw_gpio_read(uint32_t mask, uint32_t *levels)
{
	if (levels == 0) return BRIDGE_HW_ERR_INVAL;
	*levels = 0u;
	/* Bits above `GPIO_PAD_MAP_COUNT` are silently ignored -- the
     * host header documents the mapping as opaque, so out-of-range
     * bits are treated as "no pad selected" rather than an error. */
	for (size_t i = 0; i < GPIO_PAD_MAP_COUNT; ++i) {
		if ((mask & ((uint32_t)1u << i)) == 0u) continue;
		/* GPIOx_ISTAT (offset 0x10) is read-only, hardware-updated
         * every AHB cycle, and stays valid in output mode: UM
         * Rev1.2 p.269 §7.3.6 "A read access to the port input
         * status register gets the I/O state."  Always report the
         * MEASURED pad level here, never GPIOx_OCTL (offset 0x14,
         * "the last written value") -- a pad the host has promoted
         * to output but that is shorted, contended, or open on the
         * carrier must read back what the pad actually does, not
         * what CMD_GPIO_WRITE last commanded (gh#62). */
		const FlagStatus s = gpio_input_bit_get(gpio_pad_map[i].periph, gpio_pad_map[i].pin);
		if (s == SET) {
			*levels |= ((uint32_t)1u << i);
		}
	}
	return BRIDGE_HW_OK;
}

int bridge_hw_gpio_write(uint32_t mask, uint32_t levels)
{
	/* Out-of-range bits silently ignored, same policy as
     * bridge_hw_gpio_read(). */
	for (size_t i = 0; i < GPIO_PAD_MAP_COUNT; ++i) {
		if ((mask & ((uint32_t)1u << i)) == 0u) continue;
		if (!gpio_is_output[i]) {
			/* First write to this pad since boot: promote
             * INPUT+PULL_UP to OUTPUT push-pull.  12 MHz is the
             * GD32G5's slowest output speed (datasheet §7.4.1);
             * adequate for control lines, low EMI.  The bridge
             * dispatcher is single-threaded so no locking is
             * needed around the mode flip + the flag write.
             *
             * Preload the commanded level into GPIOx_OCTL via BOP
             * (offset 0x18, write-only set/clear -- UM Rev1.2 p.283
             * §7.4.7) BEFORE flipping the direction bits, while the
             * pad is still INPUT+PULL_UP.  GPIOx_OCTL resets to
             * 0x0000 0000 (UM Rev1.2 p.282 §7.4.6), so promoting the
             * pad to OUTPUT first -- as this code used to -- drives
             * it LOW for the gap until the level write below caught
             * up, glitching every pad on its first commanded HIGH
             * (gh#61). Writing OCTL while the pad is still an input
             * is harmless (push-pull mode is not active yet), so
             * this preload is a pure reordering with no new
             * register access. */
			if (levels & ((uint32_t)1u << i)) {
				gpio_bit_set(gpio_pad_map[i].periph, gpio_pad_map[i].pin);
			} else {
				gpio_bit_reset(gpio_pad_map[i].periph, gpio_pad_map[i].pin);
			}
			gpio_output_options_set(
			    gpio_pad_map[i].periph, GPIO_OTYPE_PP, GPIO_OSPEED_12MHZ, gpio_pad_map[i].pin);
			gpio_mode_set(
			    gpio_pad_map[i].periph, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, gpio_pad_map[i].pin);
			gpio_is_output[i] = true;
		}
		/* Unconditional on every call (not just the first): a
         * no-op immediately after the preload above, and the only
         * level write on every subsequent call to an already-output
         * pad. */
		if (levels & ((uint32_t)1u << i)) {
			gpio_bit_set(gpio_pad_map[i].periph, gpio_pad_map[i].pin);
		} else {
			gpio_bit_reset(gpio_pad_map[i].periph, gpio_pad_map[i].pin);
		}
	}
	return BRIDGE_HW_OK;
}
