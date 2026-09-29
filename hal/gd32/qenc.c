/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 bridge HAL backend -- quadrature encoders.
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
/* Quadrature encoder channels.                                       */
/* ----------------------------------------------------------------- */

/* E1M ENC0..3 -> (TIMER peripheral, X pad, Y pad, AF).  Each encoder
 * binds the underlying timer's CH0 + CH1 input-capture units as the
 * X / Y quadrature pair (decoder mode 2 -> X4 counts on both edges
 * of both inputs).  Maintainer-confirmed mapping:
 *
 *   E1M ENC0  X=PA0  Y=PB3  TIMER1  CH0/CH1  AF1
 *   E1M ENC1  X=PC6  Y=PC7  TIMER2  CH0/CH1  AF2
 *   E1M ENC2  X=PB6  Y=PB7  TIMER3  CH0/CH1  AF2
 *   E1M ENC3  X=PB2  Y=PA1  TIMER4  CH0/CH1  AF2
 *
 * TIMER1 + TIMER4 are 32-bit counters on the GD32G5x3; TIMER2 +
 * TIMER3 are 16-bit.  bridge_hw_qenc_read() sign-extends the narrow
 * (16-bit) timers from int16_t so a count backwards reads as a small
 * negative value instead of wrapping to +65535; see the `wide` field
 * below and gd32_qenc_t's comment in gd32_common.h. */

const gd32_qenc_t qenc_map[] = {
	[0] = { TIMER1, GPIOA, GPIO_PIN_0, GPIOB, GPIO_PIN_3, GPIO_AF_1, true },
	[1] = { TIMER2, GPIOC, GPIO_PIN_6, GPIOC, GPIO_PIN_7, GPIO_AF_2, false },
	[2] = { TIMER3, GPIOB, GPIO_PIN_6, GPIOB, GPIO_PIN_7, GPIO_AF_2, false },
	[3] = { TIMER4, GPIOB, GPIO_PIN_2, GPIOA, GPIO_PIN_1, GPIO_AF_2, true },
};
_Static_assert(sizeof(qenc_map) / sizeof(qenc_map[0]) == QENC_CHANNEL_COUNT,
               "qenc_map size must match QENC_CHANNEL_COUNT");

/* Per-encoder init.  Configures the X / Y pads as alt-function
 * inputs (pull-up so a disconnected encoder doesn't float), enables
 * the timer's clock, sets it up in quadrature decoder mode 2 (X4
 * counts on both edges of both inputs), and starts the counter. */
void qenc_channel_init(const gd32_qenc_t *e)
{
	gpio_mode_set(e->gpio_x_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, e->gpio_x_pin);
	gpio_mode_set(e->gpio_y_port, GPIO_MODE_AF, GPIO_PUPD_PULLUP, e->gpio_y_pin);
	gpio_af_set(e->gpio_x_port, e->gpio_af, e->gpio_x_pin);
	gpio_af_set(e->gpio_y_port, e->gpio_af, e->gpio_y_pin);

	timer_parameter_struct ip;
	timer_struct_para_init(&ip);
	ip.prescaler         = 0u; /* count every encoder edge   */
	ip.alignedmode       = TIMER_COUNTER_EDGE;
	ip.counterdirection  = TIMER_COUNTER_UP;
	ip.period            = 0xFFFFFFFFu; /* 16-bit timers truncate     */
	ip.clockdivision     = TIMER_CKDIV_DIV1;
	ip.repetitioncounter = 0u;
	timer_deinit(e->timer_periph);
	timer_init(e->timer_periph, &ip);
	timer_quadrature_decoder_mode_config(e->timer_periph,
	                                     TIMER_QUAD_DECODER_MODE2,
	                                     TIMER_IC_POLARITY_RISING,
	                                     TIMER_IC_POLARITY_RISING);
	timer_enable(e->timer_periph);
}

int bridge_hw_qenc_read(uint8_t encoder, int32_t *position)
{
	if (position == 0) return BRIDGE_HW_ERR_INVAL;
	*position = 0;
	if (encoder >= QENC_CHANNEL_COUNT) return BRIDGE_HW_ERR_RANGE;
	/* TIMER1 and TIMER4 (encoders 0, 3) are 32-bit counters -- the
     * raw uint32_t IS the signed count and casting straight to
     * int32_t is correct.  TIMER2 and TIMER3 (encoders 1, 2) are
     * 16-bit counters, so their upper 16 bits always read zero; a
     * plain (int32_t) cast would report a count backwards from zero
     * as +65535 instead of -1.  Route the 16-bit timers through an
     * (int32_t)(int16_t)(uint16_t) chain instead, which sign-extends
     * the 16-bit two's-complement value the way the timer intends. */
	const uint32_t raw = timer_counter_read(qenc_map[encoder].timer_periph);
	*position          = qenc_map[encoder].wide ? (int32_t)raw : (int32_t)(int16_t)(uint16_t)raw;
	return BRIDGE_HW_OK;
}

int bridge_hw_qenc_reset(uint8_t encoder)
{
	if (encoder >= QENC_CHANNEL_COUNT) return BRIDGE_HW_ERR_RANGE;
	timer_counter_value_config(qenc_map[encoder].timer_periph, 0u);
	return BRIDGE_HW_OK;
}
