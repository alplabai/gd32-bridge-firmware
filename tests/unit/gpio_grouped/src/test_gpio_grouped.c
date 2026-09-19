/* SPDX-License-Identifier: Apache-2.0 */
/* Production-linked tests for grouped GPIO reads, writes, and promotion. */

#include <zephyr/ztest.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "bridge_hw.h"
#include "gd32_common.h"
#include "gd32g5x3.h"

#define MOCK_PORT_COUNT 6u
#define MOCK_EVENT_MAX  24u

enum mock_event_kind {
	MOCK_EVENT_INPUT_READ,
	MOCK_EVENT_BOP,
	MOCK_EVENT_OPTIONS,
	MOCK_EVENT_MODE,
};

typedef struct {
	enum mock_event_kind kind;
	uint32_t             port;
	uint32_t             pins;
} mock_event_t;

static const uint32_t mock_ports[MOCK_PORT_COUNT] = {
	GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF,
};

static uint16_t     mock_inputs[MOCK_PORT_COUNT];
static uint32_t     mock_bop[MOCK_PORT_COUNT];
static uint32_t     mock_bop_previous[MOCK_PORT_COUNT];
static unsigned     mock_read_count[MOCK_PORT_COUNT];
static unsigned     mock_bop_count[MOCK_PORT_COUNT];
static mock_event_t mock_events[MOCK_EVENT_MAX];
static size_t       mock_event_count;

static size_t mock_port_index(uint32_t port)
{
	for (size_t i = 0; i < MOCK_PORT_COUNT; ++i) {
		if (mock_ports[i] == port) return i;
	}
	zassert_true(false, "unknown GPIO port 0x%08x", (unsigned)port);
	return 0u;
}

static void mock_log(enum mock_event_kind kind, uint32_t port, uint32_t pins)
{
	zassert_true(mock_event_count < MOCK_EVENT_MAX);
	mock_events[mock_event_count++] = (mock_event_t){ .kind = kind, .port = port, .pins = pins };
}

static void mock_reset(void)
{
	memset(mock_inputs, 0, sizeof mock_inputs);
	memset(mock_bop, 0, sizeof mock_bop);
	memset(mock_bop_previous, 0, sizeof mock_bop_previous);
	memset(mock_read_count, 0, sizeof mock_read_count);
	memset(mock_bop_count, 0, sizeof mock_bop_count);
	memset(mock_events, 0, sizeof mock_events);
	mock_event_count = 0u;
	memset(gpio_is_output, 0, sizeof(bool) * GPIO_PAD_MAP_COUNT);
}

uint32_t *mock_gpio_bop_lvalue(uint32_t gpio_periph)
{
	const size_t port = mock_port_index(gpio_periph);
	if (mock_bop_count[port] != 0u) mock_bop_previous[port] = mock_bop[port];
	++mock_bop_count[port];
	mock_log(MOCK_EVENT_BOP, gpio_periph, 0u);
	return &mock_bop[port];
}

uint16_t gpio_input_port_get(uint32_t gpio_periph)
{
	const size_t port = mock_port_index(gpio_periph);
	++mock_read_count[port];
	mock_log(MOCK_EVENT_INPUT_READ, gpio_periph, 0u);
	return mock_inputs[port];
}

void gpio_output_options_set(uint32_t gpio_periph, uint32_t otype, uint32_t speed, uint32_t pin)
{
	zassert_equal(otype, GPIO_OTYPE_PP);
	zassert_equal(speed, GPIO_OSPEED_12MHZ);
	mock_log(MOCK_EVENT_OPTIONS, gpio_periph, pin);
}

void gpio_mode_set(uint32_t gpio_periph, uint32_t mode, uint32_t pull_up_down, uint32_t pin)
{
	zassert_equal(mode, GPIO_MODE_OUTPUT);
	zassert_equal(pull_up_down, GPIO_PUPD_NONE);
	mock_log(MOCK_EVENT_MODE, gpio_periph, pin);
}

ZTEST(gpio_grouped, test_read_samples_each_touched_port_once)
{
	mock_reset();
	mock_inputs[mock_port_index(GPIOB)] = GPIO_PIN_10;
	mock_inputs[mock_port_index(GPIOC)] = GPIO_PIN_1;
	mock_inputs[mock_port_index(GPIOF)] = GPIO_PIN_1;

	uint32_t       levels = UINT32_MAX;
	const uint32_t mask   = (1u << 0) | (1u << 3) | (1u << 4) | (1u << 5) | (1u << 31);
	zassert_equal(bridge_hw_gpio_read(mask, &levels), BRIDGE_HW_OK);
	zassert_equal(levels, (1u << 0) | (1u << 4) | (1u << 5));
	zassert_equal(mock_read_count[mock_port_index(GPIOB)], 1u);
	zassert_equal(mock_read_count[mock_port_index(GPIOC)], 1u);
	zassert_equal(mock_read_count[mock_port_index(GPIOF)], 1u);
	zassert_equal(mock_read_count[mock_port_index(GPIOA)], 0u);
	zassert_equal(mock_read_count[mock_port_index(GPIOD)], 0u);
	zassert_equal(mock_read_count[mock_port_index(GPIOE)], 0u);
}

ZTEST(gpio_grouped, test_write_groups_set_and_clear_by_port)
{
	mock_reset();
	for (size_t i = 0; i < GPIO_PAD_MAP_COUNT; ++i)
		gpio_is_output[i] = true;

	const uint32_t mask   = (1u << 0) | (1u << 3) | (1u << 4) | (1u << 8) | (1u << 15);
	const uint32_t levels = (1u << 0) | (1u << 4) | (1u << 15);
	zassert_equal(bridge_hw_gpio_write(mask, levels), BRIDGE_HW_OK);

	zassert_equal(mock_bop_count[mock_port_index(GPIOB)], 1u);
	zassert_equal(mock_bop[mock_port_index(GPIOB)], GPIO_PIN_10 | (GPIO_PIN_0 << 16));
	zassert_equal(mock_bop_count[mock_port_index(GPIOC)], 1u);
	zassert_equal(mock_bop[mock_port_index(GPIOC)], GPIO_PIN_1 | (GPIO_PIN_14 << 16));
	zassert_equal(mock_bop_count[mock_port_index(GPIOD)], 1u);
	zassert_equal(mock_bop[mock_port_index(GPIOD)], GPIO_PIN_2);
	zassert_equal(mock_event_count, 3u);
}

ZTEST(gpio_grouped, test_first_write_preloads_before_grouped_promotion)
{
	mock_reset();

	const uint32_t mask   = (1u << 0) | (1u << 3);
	const uint32_t levels = (1u << 0);
	zassert_equal(bridge_hw_gpio_write(mask, levels), BRIDGE_HW_OK);

	const size_t port = mock_port_index(GPIOB);
	zassert_equal(mock_bop_count[port], 2u);
	zassert_equal(mock_bop_previous[port], GPIO_PIN_10 | (GPIO_PIN_0 << 16));
	zassert_equal(mock_bop[port], GPIO_PIN_10 | (GPIO_PIN_0 << 16));
	zassert_equal(mock_event_count, 4u);
	zassert_equal(mock_events[0].kind, MOCK_EVENT_BOP);
	zassert_equal(mock_events[1].kind, MOCK_EVENT_OPTIONS);
	zassert_equal(mock_events[1].pins, GPIO_PIN_10 | GPIO_PIN_0);
	zassert_equal(mock_events[2].kind, MOCK_EVENT_MODE);
	zassert_equal(mock_events[2].pins, GPIO_PIN_10 | GPIO_PIN_0);
	zassert_equal(mock_events[3].kind, MOCK_EVENT_BOP);
	zassert_true(gpio_is_output[0]);
	zassert_true(gpio_is_output[3]);
}

ZTEST(gpio_grouped, test_mixed_port_preloads_only_new_pin)
{
	mock_reset();
	gpio_is_output[0] = true; /* PB10 already drives; PB0 (logical 3) is still input. */

	const uint32_t mask   = (1u << 0) | (1u << 3);
	const uint32_t levels = (1u << 0);
	zassert_equal(bridge_hw_gpio_write(mask, levels), BRIDGE_HW_OK);

	const size_t port = mock_port_index(GPIOB);
	zassert_equal(mock_bop_count[port], 2u);
	zassert_equal(mock_bop_previous[port], GPIO_PIN_0 << 16);
	zassert_equal(mock_bop[port], GPIO_PIN_10 | (GPIO_PIN_0 << 16));
	zassert_equal(mock_events[1].pins, GPIO_PIN_0);
	zassert_equal(mock_events[2].pins, GPIO_PIN_0);
}

ZTEST(gpio_grouped, test_ignored_high_bits_touch_nothing)
{
	mock_reset();
	uint32_t levels = UINT32_MAX;
	zassert_equal(bridge_hw_gpio_read(1u << 31, &levels), BRIDGE_HW_OK);
	zassert_equal(levels, 0u);
	zassert_equal(bridge_hw_gpio_write(1u << 31, UINT32_MAX), BRIDGE_HW_OK);
	zassert_equal(mock_event_count, 0u);
}

ZTEST(gpio_grouped, test_null_read_output_is_rejected_without_access)
{
	mock_reset();
	zassert_equal(bridge_hw_gpio_read(1u, NULL), BRIDGE_HW_ERR_INVAL);
	zassert_equal(mock_event_count, 0u);
}

ZTEST_SUITE(gpio_grouped, NULL, NULL, NULL, NULL, NULL);
