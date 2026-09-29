/* SPDX-License-Identifier: Apache-2.0 */
/* Minimal host-only GD32G5x3 GPIO surface for hal/gd32/gpio.c. */
#ifndef GD32_BRIDGE_GPIO_GROUPED_MOCK_GD32G5X3_H
#define GD32_BRIDGE_GPIO_GROUPED_MOCK_GD32G5X3_H

#include <stdint.h>

#define GPIOA ((uint32_t)0x50000000u)
#define GPIOB ((uint32_t)0x50000400u)
#define GPIOC ((uint32_t)0x50000800u)
#define GPIOD ((uint32_t)0x50000C00u)
#define GPIOE ((uint32_t)0x50001000u)
#define GPIOF ((uint32_t)0x50001400u)

#define GPIO_PIN_0  ((uint32_t)1u << 0)
#define GPIO_PIN_1  ((uint32_t)1u << 1)
#define GPIO_PIN_2  ((uint32_t)1u << 2)
#define GPIO_PIN_5  ((uint32_t)1u << 5)
#define GPIO_PIN_7  ((uint32_t)1u << 7)
#define GPIO_PIN_8  ((uint32_t)1u << 8)
#define GPIO_PIN_10 ((uint32_t)1u << 10)
#define GPIO_PIN_11 ((uint32_t)1u << 11)
#define GPIO_PIN_12 ((uint32_t)1u << 12)
#define GPIO_PIN_14 ((uint32_t)1u << 14)
#define GPIO_PIN_15 ((uint32_t)1u << 15)

#define GPIO_OTYPE_PP     ((uint32_t)0u)
#define GPIO_OSPEED_12MHZ ((uint32_t)0u)
#define GPIO_MODE_OUTPUT  ((uint32_t)1u)
#define GPIO_PUPD_NONE    ((uint32_t)0u)

uint32_t *mock_gpio_bop_lvalue(uint32_t gpio_periph);
#define GPIO_BOP(gpio_periph) (*mock_gpio_bop_lvalue(gpio_periph))

uint16_t gpio_input_port_get(uint32_t gpio_periph);
void gpio_output_options_set(uint32_t gpio_periph, uint32_t otype, uint32_t speed, uint32_t pin);
void gpio_mode_set(uint32_t gpio_periph, uint32_t mode, uint32_t pull_up_down, uint32_t pin);

#endif /* GD32_BRIDGE_GPIO_GROUPED_MOCK_GD32G5X3_H */
