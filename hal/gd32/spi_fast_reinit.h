/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * SPI end-of-transaction fast re-init (gh#165).
 *
 * The CS-rising handler used to pulse RCU_SPI1RST on every transaction (~28 us)
 * because the GD32G5x3 SPI has no FIFO flush bit and stale TX FIFO bytes
 * cannot otherwise be discarded.  When SPI_STAT proves the peripheral is idle
 * and empty the reset has nothing to clean up, so it is skipped.
 */
#ifndef GD32_BRIDGE_SPI_FAST_REINIT_H
#define GD32_BRIDGE_SPI_FAST_REINIT_H

#include <stdbool.h>
#include <stdint.h>

/* Build option (CMake BRIDGE_SPI_FAST_REINIT).  0 = always take the full reset. */
#ifndef BRIDGE_SPI_FAST_REINIT
#define BRIDGE_SPI_FAST_REINIT 1
#endif

/* Re-init path counters, SWD-readable as `bridge_spi_reinit_stats`. */
typedef struct {
	uint32_t fast; /* no peripheral reset: SPI_STAT showed nothing stale */
	uint32_t slow; /* full RCU_SPI1RST reset + reconfigure */
} bridge_spi_reinit_stats_t;

extern volatile bridge_spi_reinit_stats_t bridge_spi_reinit_stats;

/* True when `stat` (SPI_STAT, read AFTER the RX drain) shows no stale state:
 * TX FIFO empty, not transmitting, RX FIFO empty, and no sticky error flag.
 * Anything else must take the full reset. */
static inline bool bridge_spi_fast_reinit_ok(uint32_t stat)
{
	return (stat & (SPI_STAT_TXLVL | SPI_STAT_TRANS | SPI_STAT_RXLVL | SPI_STAT_CRCERR |
	                SPI_STAT_CONFERR | SPI_STAT_RXORERR | SPI_STAT_FERR)) == 0u;
}

#endif
