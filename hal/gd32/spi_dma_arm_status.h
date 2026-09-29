/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gh#268: spi_dma_arm_rx()/spi_dma_arm_tx() (hal/transport_hw_gd32.c) used
 * to return void, so a spi_dma_disable_confirm() CHEN-confirmation timeout
 * at arm time left the channel silently unarmed -- no counter, no status,
 * indistinguishable on the bench from "no traffic" (see #207/#208, which
 * hardened the disable-confirm gate itself but left this edge open).
 *
 * This one function is the shared "arm failed" side effect both call
 * sites need: bump a sticky counter and flag spi_dma_error_pending so the
 * next CS-rising decode routes the transaction through the same STATUS_IO
 * seam a DMA ERRIF already uses.  Split out vendor-header-free (the
 * adc_dsp_chain.c precedent, tests/unit/CMakeLists.txt) so it is
 * host-testable without a GD32 register mock.
 */
#ifndef GD32_BRIDGE_SPI_DMA_ARM_STATUS_H
#define GD32_BRIDGE_SPI_DMA_ARM_STATUS_H

#include <stdbool.h>
#include <stdint.h>

void spi_dma_arm_record_confirm_fail(volatile uint32_t *fail_count, volatile bool *error_pending);

#endif /* GD32_BRIDGE_SPI_DMA_ARM_STATUS_H */
