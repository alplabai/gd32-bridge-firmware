/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * See spi_dma_arm_status.h.
 */
#include "spi_dma_arm_status.h"

void spi_dma_arm_record_confirm_fail(volatile uint32_t *fail_count, volatile bool *error_pending)
{
	(*fail_count)++;
	*error_pending = true;
}
