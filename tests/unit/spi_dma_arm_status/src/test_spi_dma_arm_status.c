/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Host-only regression for gh#268: spi_dma_arm_rx()/spi_dma_arm_tx()
 * (hal/transport_hw_gd32.c) used to return void, so a
 * spi_dma_disable_confirm() CHEN-confirmation timeout at arm time left the
 * channel unarmed with no counter and no status -- indistinguishable on
 * the bench from "no traffic".
 *
 * This pins the one piece of that fix that is vendor-header-free and
 * host-testable without a GD32 register mock: spi_dma_arm_record_confirm_
 * fail() (hal/gd32/spi_dma_arm_status.c), the shared "arm failed" side
 * effect both call sites now invoke -- it must bump the counter it is
 * given EXACTLY once and set the pending flag, so a real confirm timeout
 * is never silent and never double-counted.
 */

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/ztest.h>

#include "spi_dma_arm_status.h"

ZTEST_SUITE(spi_dma_arm_status, NULL, NULL, NULL, NULL, NULL);

ZTEST(spi_dma_arm_status, test_confirm_fail_bumps_counter_and_flags_pending)
{
	volatile uint32_t fail_count    = 0u;
	volatile bool     error_pending = false;

	spi_dma_arm_record_confirm_fail(&fail_count, &error_pending);

	zassert_equal(fail_count, 1u, "a single confirm timeout must count exactly once");
	zassert_true(error_pending, "a confirm timeout must flag spi_dma_error_pending");
}

ZTEST(spi_dma_arm_status, test_confirm_fail_is_sticky_across_calls)
{
	volatile uint32_t fail_count    = 5u;
	volatile bool     error_pending = false;

	spi_dma_arm_record_confirm_fail(&fail_count, &error_pending);
	spi_dma_arm_record_confirm_fail(&fail_count, &error_pending);

	zassert_equal(fail_count, 7u, "the counter is sticky: it accumulates, never resets itself");
	zassert_true(error_pending, "pending stays set across repeated failures");
}
