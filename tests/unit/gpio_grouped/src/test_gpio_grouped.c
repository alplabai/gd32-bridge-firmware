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
	/* gh#66 lazy INPUT promotion state is production state too. */
	memset(gpio_input_promoted, 0, sizeof(bool) * GPIO_PAD_MAP_COUNT);
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
	/* gh#66 added the read path's lazy INPUT promotion, so this mock
	 * sees both (OUTPUT, PUPD_NONE) from the write path and (INPUT,
	 * PUPD_NONE) from the read path.  A read must never add a pull: a
	 * pull-up on a carrier net (e.g. EVK SDIO_MUX_EN) flips it. */
	zassert_true(mode == GPIO_MODE_OUTPUT || mode == GPIO_MODE_INPUT,
	             "unexpected GPIO mode %u",
	             (unsigned)mode);
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

	const uint32_t mask   = (1u << 0) | (1u << 3) | (1u << 4) | (1u << 7) | (1u << 15);
	const uint32_t levels = (1u << 0) | (1u << 4) | (1u << 15);
	zassert_equal(bridge_hw_gpio_write(mask, levels), BRIDGE_HW_OK);

	zassert_equal(mock_bop_count[mock_port_index(GPIOB)], 1u);
	zassert_equal(mock_bop[mock_port_index(GPIOB)], GPIO_PIN_10 | (GPIO_PIN_0 << 16));
	zassert_equal(mock_bop_count[mock_port_index(GPIOC)], 1u);
	zassert_equal(mock_bop[mock_port_index(GPIOC)], GPIO_PIN_1 | (GPIO_PIN_0 << 16));
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

ZTEST(gpio_grouped, test_reg_on_bits_route_to_gpioe_14_15)
{
	/* Bits 18/19 (BT_REG_ON/WL_REG_ON) are sideband, not E1M pads, but
	 * they still route through the same generic mask path -- prove
	 * they land on GPIOE PIN_14/PIN_15 and nowhere else. */
	mock_reset();
	mock_inputs[mock_port_index(GPIOE)] = GPIO_PIN_14 | GPIO_PIN_15;

	uint32_t       levels = 0u;
	const uint32_t mask   = (1u << 18) | (1u << 19);
	zassert_equal(bridge_hw_gpio_read(mask, &levels), BRIDGE_HW_OK);
	zassert_equal(levels, mask);
	zassert_equal(mock_read_count[mock_port_index(GPIOE)], 1u);

	mock_reset();
	/* Simulate post-promotion state directly -- this test does not exercise
	 * init.c's boot loop. */
	for (size_t i = 0; i < GPIO_PAD_MAP_COUNT; ++i)
		gpio_is_output[i] = true;
	zassert_equal(bridge_hw_gpio_write(mask, (1u << 18)), BRIDGE_HW_OK);
	zassert_equal(mock_bop[mock_port_index(GPIOE)], GPIO_PIN_14 | (GPIO_PIN_15 << 16));
}

ZTEST(gpio_grouped, test_can_stby_bit_routes_to_gpiob_13)
{
	/* Bit 20 (CAN_STBY) is sideband, not an E1M pad, but it still
	 * routes through the same generic mask path -- prove it lands on
	 * GPIOB PIN_13 and nowhere else. */
	mock_reset();
	mock_inputs[mock_port_index(GPIOB)] = GPIO_PIN_13;

	uint32_t       levels = 0u;
	const uint32_t mask   = (1u << 20);
	zassert_equal(bridge_hw_gpio_read(mask, &levels), BRIDGE_HW_OK);
	zassert_equal(levels, mask);
	zassert_equal(mock_read_count[mock_port_index(GPIOB)], 1u);

	mock_reset();
	/* Simulate post-promotion state directly -- this test does not exercise
	 * init.c's boot loop. */
	for (size_t i = 0; i < GPIO_PAD_MAP_COUNT; ++i)
		gpio_is_output[i] = true;
	zassert_equal(bridge_hw_gpio_write(mask, 0u), BRIDGE_HW_OK);
	zassert_equal(mock_bop[mock_port_index(GPIOB)], GPIO_PIN_13 << 16);
}

ZTEST(gpio_grouped, test_io15_io26_bits_route_to_pb4_pc2)
{
	/* Bit 21 = E1M IO15 = GPIOB PIN_4, bit 22 = E1M IO26 = GPIOC PIN_2. */
	mock_reset();
	mock_inputs[mock_port_index(GPIOB)] = GPIO_PIN_4;
	mock_inputs[mock_port_index(GPIOC)] = GPIO_PIN_2;

	uint32_t       levels = 0u;
	const uint32_t mask   = (1u << 21) | (1u << 22);
	zassert_equal(bridge_hw_gpio_read(mask, &levels), BRIDGE_HW_OK);
	zassert_equal(levels, mask);

	mock_reset();
	for (size_t i = 0; i < GPIO_PAD_MAP_COUNT; ++i)
		gpio_is_output[i] = true;
	zassert_equal(bridge_hw_gpio_write(mask, 1u << 21), BRIDGE_HW_OK);
	zassert_equal(mock_bop[mock_port_index(GPIOB)], GPIO_PIN_4);
	zassert_equal(mock_bop[mock_port_index(GPIOC)], GPIO_PIN_2 << 16);
}

ZTEST(gpio_grouped, test_null_read_output_is_rejected_without_access)
{
	mock_reset();
	zassert_equal(bridge_hw_gpio_read(1u, NULL), BRIDGE_HW_ERR_INVAL);
	zassert_equal(mock_event_count, 0u);
}

/* gh#66: pads are parked at their analog reset state at boot; the FIRST
 * read that names a pad promotes it to floating INPUT, no pull (lazily), and
 * exactly once -- a second read must not re-promote. */
ZTEST(gpio_grouped, test_first_read_promotes_input_once)
{
	mock_reset();
	mock_inputs[mock_port_index(GPIOB)] = GPIO_PIN_10;

	uint32_t levels = 0u;
	zassert_equal(bridge_hw_gpio_read(1u << 0, &levels), BRIDGE_HW_OK);
	zassert_equal(levels, 1u << 0);
	/* MODE (promotion) logged before the port INPUT_READ. */
	zassert_equal(mock_event_count, 2u);
	zassert_equal(mock_events[0].kind, MOCK_EVENT_MODE);
	zassert_equal(mock_events[0].port, GPIOB);
	zassert_equal(mock_events[0].pins, GPIO_PIN_10);
	zassert_equal(mock_events[1].kind, MOCK_EVENT_INPUT_READ);
	zassert_true(gpio_input_promoted[0]);

	/* Second read of the same pad: promotion is sticky, so only the
	 * port INPUT_READ happens.  Clear ONLY the event log here -- the
	 * promotion state must persist across reads (it is production
	 * state, and mock_reset would wipe it). */
	memset(mock_events, 0, sizeof mock_events);
	mock_event_count = 0u;
	memset(mock_read_count, 0, sizeof mock_read_count);
	levels = 0u;
	zassert_equal(bridge_hw_gpio_read(1u << 0, &levels), BRIDGE_HW_OK);
	zassert_equal(mock_event_count, 1u);
	zassert_equal(mock_events[0].kind, MOCK_EVENT_INPUT_READ);
}

/* gh#255: a pad the host promoted to OUTPUT via CMD_GPIO_WRITE must
 * never be lazily re-promoted to INPUT by a later CMD_GPIO_READ that
 * happens to name the same bit -- that strands the pad as a pulled-up
 * input until reset while the host still believes it is driving. */
ZTEST(gpio_grouped, test_read_does_not_demote_a_driven_output_pad)
{
	mock_reset();
	zassert_equal(bridge_hw_gpio_write(1u << 0, 1u << 0), BRIDGE_HW_OK);
	zassert_true(gpio_is_output[0]);

	memset(mock_events, 0, sizeof mock_events);
	mock_event_count = 0u;

	uint32_t levels = 0u;
	zassert_equal(bridge_hw_gpio_read(1u << 0, &levels), BRIDGE_HW_OK);
	for (size_t i = 0; i < mock_event_count; ++i) {
		zassert_true(mock_events[i].kind != MOCK_EVENT_MODE,
		             "read must not reconfigure an already-driven output pad");
	}
	zassert_false(gpio_input_promoted[0]);
	zassert_true(gpio_is_output[0]);

	/* A subsequent write must still land as a plain BOP write -- no
	 * dangling gpio_input_promoted state should force a re-promotion. */
	memset(mock_events, 0, sizeof mock_events);
	mock_event_count = 0u;
	zassert_equal(bridge_hw_gpio_write(1u << 0, 0u), BRIDGE_HW_OK);
	zassert_equal(mock_event_count, 1u);
	zassert_equal(mock_events[0].kind, MOCK_EVENT_BOP);
}

ZTEST_SUITE(gpio_grouped, NULL, NULL, NULL, NULL, NULL);

ZTEST(gpio_grouped, test_unrouted_io24_bit_is_rejected_and_touches_nothing)
{
	/* gh#298: E1M IO24 is not routed to the GD32 on the SoM.  Naming bit 8
	 * must answer NOTIMPL (wire STATUS_NOSUPPORT) and sample/drive no port,
	 * even when other valid bits are named alongside it. */
	mock_reset();
	uint32_t levels = 0u;
	zassert_equal(bridge_hw_gpio_read((1u << 8) | (1u << 0), &levels), BRIDGE_HW_ERR_NOTIMPL);
	for (size_t p = 0; p < 6; ++p)
		zassert_equal(mock_read_count[p], 0u);
	zassert_equal(bridge_hw_gpio_write((1u << 8) | (1u << 0), UINT32_MAX), BRIDGE_HW_ERR_NOTIMPL);
	for (size_t p = 0; p < 6; ++p)
		zassert_equal(mock_bop_count[p], 0u);
	zassert_equal(mock_event_count, 0u);
}

ZTEST(gpio_grouped, test_io28_bit_routes_to_gpioe_9)
{
	/* gh#298: E1M IO28 is GD32 PE9 on SoM rev 2625-R2 (PC2 is E1M IO26). */
	mock_reset();
	mock_inputs[mock_port_index(GPIOE)] = GPIO_PIN_9;
	uint32_t levels                     = 0u;
	zassert_equal(bridge_hw_gpio_read(1u << 11, &levels), BRIDGE_HW_OK);
	zassert_equal(levels, 1u << 11);
	zassert_equal(mock_read_count[mock_port_index(GPIOE)], 1u);
	zassert_equal(mock_read_count[mock_port_index(GPIOC)], 0u);
}
