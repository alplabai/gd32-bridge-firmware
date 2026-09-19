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
/* Sourced from `alp-sdk metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv` */
/* (the "E1M IO*" rows).  Wire-side `mask` bit i selects entry i in   */
/* this table; numbering is compact (0..17) rather than matching the  */
/* physical E1M IO numbering, which has gaps at 15 / 17..23 / 26 / 33 */
/* because those positions are assigned to other peripherals on the   */
/* board.  Host-side translation table lives in                     */
/* https://github.com/alplabai/alp-sdk/blob/main/chips/gd32g553/gd32g553.c. */
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
/* PC6, PC7, PB6, PB7, PB2, PA1, AF mode).  None overlap today.  All   */
/* five tables ARE visible together at compile time -- gd32_common.h  */
/* externs each one and init.c includes it -- so TU visibility is not */
/* why a C _Static_assert can't do this cross-check.  The real reason */
/* is that a _Static_assert condition must be an integer constant     */
/* expression, and subscripting a `const`-qualified array object --   */
/* gpio_pad_map[i] et al -- is not one in C, visible or not; a        */
/* compile-time check would have to hand-unroll every cross-table     */
/* pair as literal macro comparisons instead of walking the arrays.   */
/* A boot-time runtime overlap check over the ~40 combined entries    */
/* would be cheap and is worth adding -- not done here, out of scope  */
/* for this PR.                                                       */
/*                                                                    */
/* If a FUTURE entry here ever reuses an ADC or DAC pin, the reused   */
/* pad's ISTAT bit is hardware-forced to 0 whenever the pin is in     */
/* Analog mode (UM Rev1.2 p.270 §7.3.7: "The port input status        */
/* register of this I/O port bit is '0'"), so bridge_hw_gpio_read()'s */
/* unconditional gpio_input_bit_get() would silently and permanently  */
/* report LOW for that bit -- the exact class of silent wrong answer  */
/* gh#62 just fixed for OCTL, relocated to ISTAT.  Grep the four      */
/* tables above before adding a pad here.                             */
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

#define GPIO_MAPPED_PORT_COUNT 6u

static const uint32_t gpio_mapped_ports[GPIO_MAPPED_PORT_COUNT] = {
	GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF,
};

static size_t gpio_mapped_port_index(uint32_t periph)
{
	for (size_t i = 0; i < GPIO_MAPPED_PORT_COUNT; ++i) {
		if (gpio_mapped_ports[i] == periph) return i;
	}
	return GPIO_MAPPED_PORT_COUNT;
}

int bridge_hw_gpio_read(uint32_t mask, uint32_t *levels)
{
	if (levels == 0) return BRIDGE_HW_ERR_INVAL;
	*levels                                       = 0u;
	uint16_t port_inputs[GPIO_MAPPED_PORT_COUNT]  = { 0u };
	bool     port_sampled[GPIO_MAPPED_PORT_COUNT] = { false };

	/* Bits above `GPIO_PAD_MAP_COUNT` are silently ignored -- the
     * host header documents the mapping as opaque, so out-of-range
     * bits are treated as "no pad selected" rather than an error. */
	for (size_t i = 0; i < GPIO_PAD_MAP_COUNT; ++i) {
		if ((mask & ((uint32_t)1u << i)) == 0u) continue;
		const size_t port = gpio_mapped_port_index(gpio_pad_map[i].periph);
		if (port == GPIO_MAPPED_PORT_COUNT) return BRIDGE_HW_ERR_IO;

		/* GPIOx_ISTAT (offset 0x10) is read-only, hardware-updated
         * every AHB cycle, and stays valid in output mode: UM
         * Rev1.2 p.269 §7.3.6 "A read access to the port input
         * status register gets the I/O state."  Always report the
         * MEASURED pad level here, never GPIOx_OCTL (offset 0x14,
         * "the last written value") -- a pad the host has promoted
         * to output but that is shorted, contended, or open on the
         * carrier must read back what the pad actually does, not
         * what CMD_GPIO_WRITE last commanded (gh#62). */
		if (!port_sampled[port]) {
			port_inputs[port]  = gpio_input_port_get(gpio_pad_map[i].periph);
			port_sampled[port] = true;
		}
		if ((port_inputs[port] & gpio_pad_map[i].pin) != 0u) {
			*levels |= ((uint32_t)1u << i);
		}
	}
	return BRIDGE_HW_OK;
}

int bridge_hw_gpio_write(uint32_t mask, uint32_t levels)
{
	uint32_t bop_words[GPIO_MAPPED_PORT_COUNT]    = { 0u };
	uint32_t promote_pins[GPIO_MAPPED_PORT_COUNT] = { 0u };

	/* Out-of-range bits silently ignored, same policy as
     * bridge_hw_gpio_read(). */
	for (size_t i = 0; i < GPIO_PAD_MAP_COUNT; ++i) {
		if ((mask & ((uint32_t)1u << i)) == 0u) continue;
		const size_t port = gpio_mapped_port_index(gpio_pad_map[i].periph);
		if (port == GPIO_MAPPED_PORT_COUNT) return BRIDGE_HW_ERR_IO;

		if ((levels & ((uint32_t)1u << i)) != 0u) {
			bop_words[port] |= gpio_pad_map[i].pin;
		} else {
			bop_words[port] |= gpio_pad_map[i].pin << 16;
		}
		if (!gpio_is_output[i]) {
			promote_pins[port] |= gpio_pad_map[i].pin;
		}
	}

	/* Promote newly written pads by physical port.  Preload only those
     * pins through GPIOx_BOP before changing their mode, preserving the
     * no-low-glitch rule from gh#61.  The SPL accepts a pin mask, so every
     * new pin on a port changes output options and direction together. */
	for (size_t port = 0; port < GPIO_MAPPED_PORT_COUNT; ++port) {
		if (promote_pins[port] == 0u) continue;
		const uint32_t preload_mask       = promote_pins[port] | (promote_pins[port] << 16);
		GPIO_BOP(gpio_mapped_ports[port]) = bop_words[port] & preload_mask;
		gpio_output_options_set(
		    gpio_mapped_ports[port], GPIO_OTYPE_PP, GPIO_OSPEED_12MHZ, promote_pins[port]);
		gpio_mode_set(
		    gpio_mapped_ports[port], GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, promote_pins[port]);
	}

	for (size_t i = 0; i < GPIO_PAD_MAP_COUNT; ++i) {
		if ((mask & ((uint32_t)1u << i)) != 0u) gpio_is_output[i] = true;
	}

	/* GPIOx_BOP atomically sets its low-half mask and clears its high-half
     * mask (UM Rev1.2 p.283).  One final write per touched port prevents
     * same-port pins from exposing the per-pad intermediate states that
     * the old gpio_bit_set()/gpio_bit_reset() walk produced (gh#198).
     * Atomicity is per physical port, not across all six ports; the
     * pre-existing cross-transport dispatch interleaving is tracked by
     * gh#19 and its protocol-wide serialization fix. */
	for (size_t port = 0; port < GPIO_MAPPED_PORT_COUNT; ++port) {
		if (bop_words[port] != 0u) GPIO_BOP(gpio_mapped_ports[port]) = bop_words[port];
	}
	return BRIDGE_HW_OK;
}
