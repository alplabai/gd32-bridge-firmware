/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Every exit path of the CS-edge EXTI handler must clear the EXTI_PD0 group
 * bits it owns (gh#66).  The handler used to `return` early on two error
 * paths (DMA quiesce timeout, DMA/SPI error seam) and skip the final
 * EXTI_PD0 clear, leaving EXTI line 8 pending: the handler re-entered
 * immediately at BRIDGE_CS_IRQ_PRIO and starved base level.
 *
 * The REAL hal/transport_hw_gd32.c is #included so the test drives the
 * production handler body against the vendor mock in mock/gd32g5x3.h.  The
 * handler reads EXTI_PD0 once at entry (snapshot) and stores the clear once
 * on exit, so exactly two accesses must be counted on a correct exit.
 */

#include <stdbool.h>
#include <stdint.h>

#include <zephyr/ztest.h>

#include "../../../../hal/transport_hw_gd32.c"

/* ---- mock backing store ------------------------------------------------ */
uint32_t mock_scratch;
uint32_t mock_dma_chctl;
uint32_t mock_spi_stat;

static uint32_t pd0;
static unsigned pd0_accesses;
static bool     cs_level_high;
static bool     dma_err_flag;

/* SPI reset model: the RCU pulse empties both FIFOs and clears TRANS and the
 * sticky error flags -- the only way this silicon discards stale TX bytes. */
static unsigned spi_resets;
void            rcu_periph_reset_enable(rcu_periph_reset_enum r)
{
	(void)r;
	spi_resets++;
	mock_spi_stat &= ~(SPI_STAT_TXLVL | SPI_STAT_RXLVL | SPI_STAT_TRANS | SPI_STAT_CRCERR |
	                   SPI_STAT_CONFERR | SPI_STAT_RXORERR | SPI_STAT_FERR);
}
void rcu_periph_reset_disable(rcu_periph_reset_enum r)
{
	(void)r;
}

uint32_t *mock_exti_pd0(void)
{
	pd0_accesses++;
	return &pd0;
}

FlagStatus gpio_input_bit_get(uint32_t port, uint32_t pin)
{
	(void)port;
	(void)pin;
	return cs_level_high ? SET : RESET;
}
FlagStatus dma_interrupt_flag_get(uint32_t d, uint32_t ch, uint32_t f)
{
	(void)d;
	(void)ch;
	(void)f;
	return dma_err_flag ? SET : RESET;
}
FlagStatus spi_flag_get(uint32_t p, uint32_t f)
{
	(void)p;
	(void)f;
	return RESET;
}
FlagStatus i2c_flag_get(uint32_t p, uint32_t f)
{
	(void)p;
	(void)f;
	return RESET;
}
FlagStatus i2c_interrupt_flag_get(uint32_t p, uint32_t f)
{
	(void)p;
	(void)f;
	return RESET;
}
/* TX/RX DMA model knobs (gh#165): bytes the master clocked this transaction and
 * TX DMA transfers still pending.  Defaults = nothing clocked, TX drained. */
static uint32_t mock_clocked;
static uint32_t mock_tx_remaining;
static uint32_t mock_reply_left; /* staged reply bytes spi_slave_tx_*() hands out */
uint32_t        dma_transfer_number_get(uint32_t d, uint32_t ch)
{
	(void)d;
	return ch == BRIDGE_SPI_TX_DMA_CH ? mock_tx_remaining : BRIDGE_SPI_DMA_BUF_LEN - mock_clocked;
}
uint32_t spi_data_receive(uint32_t p)
{
	(void)p;
	return 0u;
}
uint32_t i2c_data_receive(uint32_t p)
{
	(void)p;
	return 0u;
}
uint32_t rcu_clock_freq_get(uint32_t c)
{
	(void)c;
	return 0u;
}

/* ---- seams / cross-TU symbols the HAL file references ------------------- */
void spi_slave_cs_low(void)
{
}
void spi_slave_rx_byte(uint8_t b)
{
	(void)b;
}
void spi_slave_cs_high(void)
{
}
void spi_slave_transport_error(void)
{
}
uint8_t spi_slave_tx_next_byte(void)
{
	if (mock_reply_left > 0u) mock_reply_left--;
	return 0xFFu;
}
bool spi_slave_tx_pending(void)
{
	return mock_reply_left > 0u;
}
void i2c_slave_write_start(void)
{
}
bool i2c_slave_write_end(void)
{
	return false;
}
void i2c_slave_rx_byte(uint8_t b)
{
	(void)b;
}
void i2c_slave_tx_abort(void)
{
}
uint8_t i2c_slave_tx_next_byte(void)
{
	return 0xFFu;
}
void fault_reset_loop_mark_healthy(void)
{
}

/* ---- harness ----------------------------------------------------------- */
/* The shim runs no suite hooks, so each case calls this itself. */
static void reset_state(void)
{
	/* The HAL remembers the armed TX length across calls: prime it to 0 with a
	 * no-reply transaction so every case starts from "no TX armed". */
	pd0               = 1u << 8;
	cs_level_high     = true;
	dma_err_flag      = false;
	mock_dma_chctl    = 0u;
	mock_spi_stat     = 0u;
	mock_clocked      = 0u;
	mock_tx_remaining = 0u;
	mock_reply_left   = 0u;
	BRIDGE_SPI_CS_EXTI_HANDLER();
	pd0                          = 1u << 8; /* line 8 (CS) pending at entry */
	pd0_accesses                 = 0u;
	cs_level_high                = true;
	dma_err_flag                 = false;
	mock_dma_chctl               = 0u; /* CHEN clear: channels quiesce */
	mock_spi_stat                = 0u;
	mock_clocked                 = 0u;
	mock_tx_remaining            = 0u;
	mock_reply_left              = 0u;
	spi_resets                   = 0u;
	bridge_spi_reinit_stats.fast = 0u;
	bridge_spi_reinit_stats.slow = 0u;
	spi_dma_error_pending        = false;
}

ZTEST_SUITE(transport_hw_cs_exti, NULL, NULL, NULL, NULL, NULL);

/* Entry read + exit clear == 2 accesses; the stored value is the entry
 * snapshot, i.e. the pending line-8 bit is written back to clear it. */
static void expect_group_cleared(const char *what)
{
	zassert_equal(pd0_accesses, 2u, "%s: EXTI_PD0 must be read once and cleared once", what);
	zassert_equal(pd0 & (1u << 8), 1u << 8, "%s: line 8 clear was not stored", what);
}

ZTEST(transport_hw_cs_exti, test_cs_falling_clears_group)
{
	reset_state();
	cs_level_high = false;
	BRIDGE_SPI_CS_EXTI_HANDLER();
	expect_group_cleared("cs falling");
}

ZTEST(transport_hw_cs_exti, test_cs_rising_clean_decode_clears_group)
{
	reset_state();
	BRIDGE_SPI_CS_EXTI_HANDLER();
	expect_group_cleared("cs rising, clean");
}

ZTEST(transport_hw_cs_exti, test_cs_rising_quiesce_timeout_clears_group)
{
	reset_state();
	mock_dma_chctl = DMA_CHXCTL_CHEN; /* CHEN never reads clear */
	BRIDGE_SPI_CS_EXTI_HANDLER();
	expect_group_cleared("quiesce timeout");
}

ZTEST(transport_hw_cs_exti, test_cs_rising_dma_error_clears_group)
{
	reset_state();
	dma_err_flag = true; /* ERRIF -> STATUS_IO error seam */
	BRIDGE_SPI_CS_EXTI_HANDLER();
	expect_group_cleared("dma error seam");
}

ZTEST(transport_hw_cs_exti, test_cs_rising_spi_overrun_clears_group)
{
	reset_state();
	mock_spi_stat = SPI_STAT_RXORERR;
	BRIDGE_SPI_CS_EXTI_HANDLER();
	expect_group_cleared("spi overrun seam");
}

/* ---- SPI fast re-init (gh#165, opt-in BRIDGE_SPI_FAST_REINIT) ---------------- */

#if BRIDGE_SPI_FAST_REINIT

#define TXLVL(n) ((uint32_t)(n) << 11)
#define RXLVL(n) ((uint32_t)(n) << 9)

/* Run one CS-rising with the given SPI_STAT; the model's reset clears it. */
static void rising_with_stat(uint32_t stat)
{
	reset_state();
	mock_spi_stat = stat;
	BRIDGE_SPI_CS_EXTI_HANDLER();
}

/* No byte may be left in the TX FIFO once the handler returns: a stale one
 * would be shifted out as the first byte of the NEXT reply. */
static void expect_no_stale_tx(const char *what)
{
	zassert_equal(mock_spi_stat & SPI_STAT_TXLVL, 0u, "%s: stale TX byte survives", what);
	zassert_equal(mock_spi_stat & SPI_STAT_TRANS, 0u, "%s: still transmitting", what);
}

ZTEST(transport_hw_cs_exti, test_fast_reinit_taken_when_spi_idle)
{
	rising_with_stat(0u);
	zassert_equal(spi_resets, 0u, "no peripheral reset when TX empty and idle");
	zassert_equal(bridge_spi_reinit_stats.fast, 1u);
	zassert_equal(bridge_spi_reinit_stats.slow, 0u);
	expect_no_stale_tx("fast");
	expect_group_cleared("fast path");
}

ZTEST(transport_hw_cs_exti, test_slow_reinit_when_tx_fifo_not_empty)
{
	for (unsigned lvl = 1u; lvl <= 3u; lvl++) {
		rising_with_stat(TXLVL(lvl));
		zassert_equal(spi_resets, 1u, "TXLVL=%u must reset", lvl);
		zassert_equal(bridge_spi_reinit_stats.slow, 1u);
		zassert_equal(bridge_spi_reinit_stats.fast, 0u);
		expect_no_stale_tx("slow, TXLVL");
	}
}

ZTEST(transport_hw_cs_exti, test_slow_reinit_when_trans_set)
{
	rising_with_stat(SPI_STAT_TRANS);
	zassert_equal(spi_resets, 1u);
	zassert_equal(bridge_spi_reinit_stats.slow, 1u);
	expect_no_stale_tx("slow, TRANS");
}

ZTEST(transport_hw_cs_exti, test_slow_reinit_when_rx_not_drained_or_flag_sticky)
{
	static const uint32_t bad[] = { RXLVL(1), SPI_STAT_CRCERR, SPI_STAT_CONFERR, SPI_STAT_FERR };
	for (unsigned i = 0u; i < sizeof(bad) / sizeof(bad[0]); i++) {
		rising_with_stat(bad[i]);
		zassert_equal(spi_resets, 1u, "stat 0x%x must reset", bad[i]);
		zassert_equal(bridge_spi_reinit_stats.fast, 0u);
	}
}

/* Shift-register model.  A reply of `armed` bytes is staged by one transaction;
 * the next one clocks `clocked` bytes.  The first `armed-clocked` unsent bytes
 * sit in: 1 shift register + up to 3 FIFO slots + the rest still in the TX DMA.
 * TXLVL sees only the FIFO, so clocked == armed-1 shows TXLVL=0, TRANS=0 while
 * one byte is stuck in the shift register -- it would lead the NEXT reply. */
static void run_reply_then_read(uint32_t armed, uint32_t clocked)
{
	reset_state();
	mock_reply_left = armed;
	BRIDGE_SPI_CS_EXTI_HANDLER(); /* stages + arms the reply */
	pd0                          = 1u << 8;
	pd0_accesses                 = 0u;
	spi_resets                   = 0u;
	bridge_spi_reinit_stats.fast = 0u;
	bridge_spi_reinit_stats.slow = 0u;
	const uint32_t unsent        = armed > clocked ? armed - clocked : 0u;
	const uint32_t queued        = unsent > 0u ? unsent - 1u : 0u; /* behind the shift register */
	const uint32_t fifo          = queued > 3u ? 3u : queued;
	mock_clocked                 = clocked;
	mock_tx_remaining            = queued - fifo;
	mock_spi_stat                = TXLVL(fifo);
	BRIDGE_SPI_CS_EXTI_HANDLER();
}

/* Equivalence with the always-reset behaviour: whenever the fast path is taken
 * no reply byte may remain unsent (shift register, FIFO or DMA), and it is taken
 * exactly when the master clocked the whole armed reply.  Includes the
 * clocked == armed-1 case. */
ZTEST(transport_hw_cs_exti, test_fast_reinit_never_leaves_a_stale_tx_byte)
{
	for (uint32_t armed = 1u; armed <= 12u; armed++) {
		for (uint32_t clocked = 0u; clocked <= armed + 2u; clocked++) {
			run_reply_then_read(armed, clocked);
			const bool all_sent = clocked >= armed;
			zassert_equal(spi_resets, all_sent ? 0u : 1u, "armed=%u clocked=%u", armed, clocked);
			zassert_equal(bridge_spi_reinit_stats.fast,
			              all_sent ? 1u : 0u,
			              "armed=%u clocked=%u",
			              armed,
			              clocked);
		}
	}
}

ZTEST(transport_hw_cs_exti, test_short_read_by_one_byte_takes_slow_path)
{
	run_reply_then_read(8u, 7u); /* TXLVL=0 TRANS=0 yet one byte in the shift register */
	zassert_equal(mock_spi_stat & (SPI_STAT_TXLVL | SPI_STAT_TRANS), 0u);
	zassert_equal(spi_resets, 1u, "short read must reset");
	zassert_equal(bridge_spi_reinit_stats.slow, 1u);
}

/* SPI_STAT combinations still force the slow path on their own. */
ZTEST(transport_hw_cs_exti, test_every_fifo_state_resets_unless_idle)
{
	for (uint32_t tx = 0u; tx <= 3u; tx++) {
		for (uint32_t rx = 0u; rx <= 3u; rx++) {
			for (uint32_t trans = 0u; trans <= 1u; trans++) {
				const uint32_t stat = TXLVL(tx) | RXLVL(rx) | (trans ? SPI_STAT_TRANS : 0u);
				rising_with_stat(stat);
				expect_no_stale_tx("equivalence");
				const bool clean = (stat == 0u);
				zassert_equal(spi_resets, clean ? 0u : 1u, "tx=%u rx=%u trans=%u", tx, rx, trans);
				zassert_equal(bridge_spi_reinit_stats.fast, clean ? 1u : 0u);
			}
		}
	}
}

/* The overrun / error-seam / quiesce-timeout exits keep their unconditional
 * reset: the fast path must never apply to a transaction with a fault. */
ZTEST(transport_hw_cs_exti, test_fault_paths_still_reset)
{
	rising_with_stat(SPI_STAT_RXORERR);
	zassert_equal(spi_resets, 1u, "overrun");
	reset_state();
	dma_err_flag = true;
	BRIDGE_SPI_CS_EXTI_HANDLER();
	zassert_equal(spi_resets, 1u, "dma error");
	reset_state();
	mock_dma_chctl = DMA_CHXCTL_CHEN;
	BRIDGE_SPI_CS_EXTI_HANDLER();
	zassert_equal(spi_resets, 1u, "quiesce timeout");
}

#else /* default build: every transaction takes the full reset */

ZTEST(transport_hw_cs_exti, test_default_build_always_resets_the_spi)
{
	for (uint32_t armed = 0u; armed <= 4u; armed++) {
		reset_state();
		mock_reply_left = armed;
		BRIDGE_SPI_CS_EXTI_HANDLER(); /* stages + arms the reply */
		pd0           = 1u << 8;
		spi_resets    = 0u;
		mock_clocked  = armed; /* whole reply clocked, SPI_STAT idle */
		mock_spi_stat = 0u;
		BRIDGE_SPI_CS_EXTI_HANDLER();
		zassert_equal(spi_resets, 1u, "armed=%u: reset must not be skipped by default", armed);
		zassert_equal(bridge_spi_reinit_stats.fast, 0u);
	}
}

#endif /* BRIDGE_SPI_FAST_REINIT */
