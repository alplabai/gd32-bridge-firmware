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
 *
 * v0.15 adds the ATTN cases at the bottom: the GPIO mock records every drive /
 * mode call on PA14 so the enable / disable sequences and the drive points
 * (CS falling, CS-rising entry and exit, watermark event) are asserted in
 * order.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "../../../../hal/transport_hw_gd32.c"

/* ---- mock backing store ------------------------------------------------ */
uint32_t mock_scratch;
uint32_t mock_dma_chctl;
uint32_t mock_spi_stat;
uint32_t mock_dhcsr;

static uint32_t pd0;
static unsigned pd0_accesses;
static bool     cs_level_high;
static bool     dma_err_flag;
static int      cs_flip_low_after; /* 0 = off */
static bool     cs_high_fresh;     /* what the stubbed spi_slave_cs_high() reports */
static bool     transport_err_called;

/* ---- ATTN (PA14) pin-op log ---------------------------------------------- */
typedef enum {
	OP_LOW = 1,     /* gpio_bit_reset (GPIO_BC)                         */
	OP_HIGH,        /* gpio_bit_set                                     */
	OP_OTYPE_PP_12, /* push-pull, slowest speed class (12 MHz)          */
	OP_MODE_OUT,    /* output mode, no pull                             */
	OP_AF0,         /* alternate function 0                             */
	OP_MODE_AF_PD,  /* alternate-function mode with pull-down           */
	OP_OTHER,       /* anything unexpected on PA14                      */
} attn_op_t;
static attn_op_t attn_log[16];
static unsigned  attn_log_n;

static void attn_record(uint32_t pin, attn_op_t op)
{
	if (pin != GPIO_PIN_14) return; /* only PA14 is of interest */
	if (attn_log_n < sizeof(attn_log) / sizeof(attn_log[0])) attn_log[attn_log_n++] = op;
}

void gpio_bit_set(uint32_t port, uint32_t pin)
{
	(void)port;
	attn_record(pin, OP_HIGH);
}
void gpio_bit_reset(uint32_t port, uint32_t pin)
{
	(void)port;
	attn_record(pin, OP_LOW);
}
void gpio_mode_set(uint32_t port, uint32_t mode, uint32_t pupd, uint32_t pin)
{
	(void)port;
	if (mode == GPIO_MODE_OUTPUT && pupd == GPIO_PUPD_NONE)
		attn_record(pin, OP_MODE_OUT);
	else if (mode == GPIO_MODE_AF && pupd == GPIO_PUPD_PULLDOWN)
		attn_record(pin, OP_MODE_AF_PD);
	else
		attn_record(pin, OP_OTHER);
}
void gpio_output_options_set(uint32_t port, uint32_t otype, uint32_t speed, uint32_t pin)
{
	(void)port;
	attn_record(pin,
	            (otype == GPIO_OTYPE_PP && speed == GPIO_OSPEED_12MHZ) ? OP_OTYPE_PP_12 : OP_OTHER);
}
void gpio_af_set(uint32_t port, uint32_t af, uint32_t pin)
{
	(void)port;
	attn_record(pin, af == GPIO_AF_0 ? OP_AF0 : OP_OTHER);
}

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
	const bool level = cs_level_high;
	/* Model a CS edge landing mid-handler: after N reads the line goes low. */
	if (cs_flip_low_after > 0 && --cs_flip_low_after == 0) cs_level_high = false;
	return level ? SET : RESET;
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
uint32_t dma_transfer_number_get(uint32_t d, uint32_t ch)
{
	(void)d;
	(void)ch;
	return BRIDGE_SPI_DMA_BUF_LEN; /* nothing received */
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
bool spi_slave_cs_high(void)
{
	return cs_high_fresh;
}
void spi_slave_transport_error(void)
{
	transport_err_called = true;
}
uint8_t spi_slave_tx_next_byte(void)
{
	return 0xFFu;
}
bool spi_slave_tx_pending(void)
{
	return false;
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
	pd0                          = 1u << 8; /* line 8 (CS) pending at entry */
	pd0_accesses                 = 0u;
	cs_level_high                = true;
	dma_err_flag                 = false;
	mock_dma_chctl               = 0u; /* CHEN clear: channels quiesce */
	mock_spi_stat                = 0u;
	spi_resets                   = 0u;
	bridge_spi_reinit_stats.fast = 0u;
	bridge_spi_reinit_stats.slow = 0u;
	spi_dma_error_pending        = false;
	cs_high_fresh                = true;
	transport_err_called         = false;
	mock_dhcsr                   = 0u;
	attn_log_n                   = 0u;
	memset(attn_log, 0, sizeof(attn_log));
	cs_flip_low_after = 0;
	attn_quiesced     = false;
	bridge_hw_attn_enable(false); /* ATTN off, event bits clear, between cases */
	bridge_hw_attn_streams_enable(true);
	attn_log_n = 0u;
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

/* ---- SPI fast re-init (gh#165) --------------------------------------------- */

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

/* Equivalence: for EVERY TXLVL/TRANS/RXLVL combination the post-handler state
 * is the same as the always-reset behaviour -- empty TX FIFO, not busy -- and
 * the fast path is taken exactly when that was already true on entry. */
ZTEST(transport_hw_cs_exti, test_fast_reinit_equivalent_to_full_reset_for_every_fifo_state)
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

/* ---- ATTN (v0.15) --------------------------------------------------------- */

static void expect_log(const attn_op_t *want, unsigned n, const char *what)
{
	zassert_equal(attn_log_n, n, "%s: PA14 op count", what);
	for (unsigned i = 0u; i < n; i++) {
		zassert_equal(attn_log[i], want[i], "%s: PA14 op #%u", what, i);
	}
}

/* F3 enable: output latch LOW first, then push-pull / slowest speed / no pull,
 * then output mode -- PA14 never glitches high. */
ZTEST(transport_hw_cs_exti, test_attn_enable_sequence)
{
	static const attn_op_t want[] = { OP_LOW, OP_OTYPE_PP_12, OP_MODE_OUT };
	reset_state();
	zassert_equal(bridge_hw_attn_enable(true), BRIDGE_HW_OK);
	expect_log(want, 3u, "enable");
	zassert_true(attn_on);
}

/* F3 disable: drive low, restore AF0, then AF mode with the reset pull-down. */
ZTEST(transport_hw_cs_exti, test_attn_disable_sequence_restores_swd)
{
	static const attn_op_t want[] = { OP_LOW, OP_AF0, OP_MODE_AF_PD };
	reset_state();
	bridge_hw_attn_enable(true);
	attn_log_n = 0u;
	zassert_equal(bridge_hw_attn_enable(false), BRIDGE_HW_OK);
	expect_log(want, 3u, "disable");
	zassert_false(attn_on);
	/* Disabled twice / enabled twice is idempotent: no pin activity. */
	attn_log_n = 0u;
	bridge_hw_attn_enable(false);
	zassert_equal(attn_log_n, 0u, "second disable touches nothing");
}

/* F1: a link that never enabled ATTN never has PA14 driven, on any edge. */
ZTEST(transport_hw_cs_exti, test_attn_off_never_drives_pa14)
{
	reset_state();
	cs_level_high = false;
	BRIDGE_SPI_CS_EXTI_HANDLER();
	reset_state();
	attn_log_n = 0u;
	BRIDGE_SPI_CS_EXTI_HANDLER(); /* rising, fresh stage */
	bridge_hw_attn_event_set(0u);
	zassert_equal(attn_log_n, 0u, "no PA14 activity while ATTN is not enabled");
}

/* CS falling: deassert. */
ZTEST(transport_hw_cs_exti, test_attn_cs_falling_drives_low)
{
	static const attn_op_t want[] = { OP_LOW };
	reset_state();
	bridge_hw_attn_enable(true);
	attn_log_n    = 0u;
	cs_level_high = false;
	BRIDGE_SPI_CS_EXTI_HANDLER();
	expect_log(want, 1u, "cs falling");
}

/* CS rising with a FRESH reply armed: low at entry, high at exit. */
ZTEST(transport_hw_cs_exti, test_attn_fresh_stage_rises_after_arm)
{
	static const attn_op_t want[] = { OP_LOW, OP_HIGH };
	reset_state();
	bridge_hw_attn_enable(true);
	attn_log_n    = 0u;
	cs_high_fresh = true;
	BRIDGE_SPI_CS_EXTI_HANDLER();
	expect_log(want, 2u, "fresh stage");
	expect_group_cleared("attn fresh stage");
}

/* A drain / empty transaction stages nothing: stay low unless an event waits. */
ZTEST(transport_hw_cs_exti, test_attn_drain_without_event_stays_low)
{
	static const attn_op_t want[] = { OP_LOW };
	reset_state();
	bridge_hw_attn_enable(true);
	attn_log_n    = 0u;
	cs_high_fresh = false;
	BRIDGE_SPI_CS_EXTI_HANDLER();
	expect_log(want, 1u, "drain, no event");
}

ZTEST(transport_hw_cs_exti, test_attn_drain_with_pending_event_rises)
{
	static const attn_op_t want[] = { OP_LOW, OP_HIGH };
	reset_state();
	bridge_hw_attn_enable(true);
	cs_level_high = false; /* an event arrives while CS is asserted: latched, not driven */
	bridge_hw_attn_event_set(1u);
	zassert_equal(attn_ev, 2u, "event bit latched for stream 1");
	attn_log_n    = 0u;
	cs_level_high = true;
	pd0           = 1u << 8;
	cs_high_fresh = false;
	BRIDGE_SPI_CS_EXTI_HANDLER();
	expect_log(want, 2u, "drain, event pending");
}

/* A quiesce failure stages nothing either: same rule as a drain. */
ZTEST(transport_hw_cs_exti, test_attn_quiesce_failure_follows_drain_rule)
{
	static const attn_op_t none[] = { OP_LOW };
	reset_state();
	bridge_hw_attn_enable(true);
	attn_log_n     = 0u;
	mock_dma_chctl = DMA_CHXCTL_CHEN; /* CHEN never clears */
	cs_high_fresh  = true;            /* must be ignored: nothing was staged */
	BRIDGE_SPI_CS_EXTI_HANDLER();
	expect_log(none, 1u, "quiesce failure, no event");
	expect_group_cleared("attn quiesce failure");
}

/* The DMA/SPI error seam stages the STATUS_IO envelope: a fresh reply. */
ZTEST(transport_hw_cs_exti, test_attn_error_seam_envelope_is_a_fresh_stage)
{
	static const attn_op_t want[] = { OP_LOW, OP_HIGH };
	reset_state();
	bridge_hw_attn_enable(true);
	attn_log_n    = 0u;
	cs_high_fresh = false; /* irrelevant on this path */
	dma_err_flag  = true;
	BRIDGE_SPI_CS_EXTI_HANDLER();
	zassert_true(transport_err_called);
	expect_log(want, 2u, "error seam");
}

/* Watermark event: drive high only while CS is idle AND no CS edge is pending. */
ZTEST(transport_hw_cs_exti, test_attn_event_gated_on_cs_idle_and_no_pending_edge)
{
	reset_state();
	bridge_hw_attn_enable(true);

	attn_log_n    = 0u;
	cs_level_high = true;
	pd0           = 0u;
	bridge_hw_attn_event_set(0u);
	zassert_equal(attn_log_n, 1u, "idle CS, no pending edge: rises");
	zassert_equal(attn_log[0], OP_HIGH);

	attn_log_n    = 0u;
	cs_level_high = false; /* CS asserted */
	bridge_hw_attn_event_set(0u);
	zassert_equal(attn_log_n, 0u, "CS asserted: latched, not driven");

	attn_log_n    = 0u;
	cs_level_high = true;
	pd0           = 1u << 8; /* an edge on line 8 has not been serviced */
	bridge_hw_attn_event_set(0u);
	zassert_equal(attn_log_n, 0u, "pending EXTI line-8 bit: latched, not driven");
	zassert_equal(attn_ev, 1u);
}

ZTEST(transport_hw_cs_exti, test_attn_event_clear_drops_the_bit)
{
	reset_state();
	bridge_hw_attn_enable(true);
	cs_level_high = false;
	bridge_hw_attn_event_set(0u);
	bridge_hw_attn_event_set(1u);
	bridge_hw_attn_event_clear(0u);
	zassert_equal(attn_ev, 2u);
	bridge_hw_attn_enable(false);
	zassert_equal(attn_ev, 0u, "disabling ATTN discards pending events");
}

/* POWER_MODE_SET: low before entering. */
ZTEST(transport_hw_cs_exti, test_attn_quiesce_drives_low_only_when_enabled)
{
	reset_state();
	attn_log_n = 0u;
	bridge_hw_attn_quiesce();
	zassert_equal(attn_log_n, 0u, "disabled: nothing");
	bridge_hw_attn_enable(true);
	attn_log_n = 0u;
	bridge_hw_attn_quiesce();
	zassert_equal(attn_log_n, 1u);
	zassert_equal(attn_log[0], OP_LOW);
}

/* F2 probe + F4: PA14 is never in a GPIOA lock mask. */
ZTEST(transport_hw_cs_exti, test_attn_debugger_probe_and_lock_mask)
{
	reset_state();
	mock_dhcsr = 0u;
	zassert_false(bridge_hw_debugger_attached());
	mock_dhcsr = 1u;
	zassert_true(bridge_hw_debugger_attached(), "C_DEBUGEN set = debugger attached");
	zassert_equal(BRIDGE_GPIOA_LOCK_MASK & (1u << 14), 0u, "PA14 stays out of the lock mask");
	zassert_equal(BRIDGE_GPIOA_LOCK_MASK, 0x8700u);
}

/* A fresh reply is armed but the host already re-asserted CS (a coalesced next
 * transaction): the exit does NOT raise ATTN (same CS-idle gate as the event
 * assert); the next CS-rising exit re-evaluates. */
ZTEST(transport_hw_cs_exti, test_attn_exit_stays_low_when_cs_is_asserted_again)
{
	static const attn_op_t want[] = { OP_LOW };
	reset_state();
	bridge_hw_attn_enable(true);
	attn_log_n        = 0u;
	cs_high_fresh     = true;
	cs_flip_low_after = 1; /* the handler's entry read sees high; the exit read sees low */
	BRIDGE_SPI_CS_EXTI_HANDLER();
	expect_log(want, 1u, "fresh stage but CS low at exit");
}

/* POWER_MODE_SET: ATTN stays low through the sleep -- a watermark IRQ latches
 * the event but cannot drive the pin -- until the next CS edge. */
ZTEST(transport_hw_cs_exti, test_attn_quiesce_holds_low_until_the_next_cs_edge)
{
	reset_state();
	bridge_hw_attn_enable(true);
	cs_level_high = true;
	pd0           = 0u;
	attn_log_n    = 0u;
	bridge_hw_attn_quiesce();
	bridge_hw_attn_event_set(0u);
	zassert_equal(attn_log_n, 1u, "only the quiesce's own low");
	zassert_equal(attn_log[0], OP_LOW);
	zassert_equal(attn_ev, 1u, "the event is still latched");

	pd0           = 1u << 8;
	cs_level_high = false; /* host wakes the link: CS falling ends the quiesce */
	BRIDGE_SPI_CS_EXTI_HANDLER();
	zassert_false(attn_quiesced);

	attn_log_n    = 0u;
	cs_level_high = true;
	pd0           = 0u;
	bridge_hw_attn_event_set(1u);
	zassert_equal(attn_log_n, 1u, "events drive again after the edge");
	zassert_equal(attn_log[0], OP_HIGH);
}

/* With ADC_STREAM2 not granted there is nothing able to clear a watermark
 * event, so none is latched and ATTN is not driven for it. */
ZTEST(transport_hw_cs_exti, test_attn_events_ignored_while_streams_are_disabled)
{
	reset_state();
	bridge_hw_attn_enable(true);
	bridge_hw_attn_streams_enable(false);
	cs_level_high = true;
	pd0           = 0u;
	attn_log_n    = 0u;
	bridge_hw_attn_event_set(0u);
	zassert_equal(attn_log_n, 0u);
	zassert_equal(attn_ev, 0u);

	bridge_hw_attn_streams_enable(true);
	bridge_hw_attn_event_set(0u);
	zassert_equal(attn_log_n, 1u);
	bridge_hw_attn_streams_enable(false);
	zassert_equal(attn_ev, 0u, "disabling discards what was pending");
}
