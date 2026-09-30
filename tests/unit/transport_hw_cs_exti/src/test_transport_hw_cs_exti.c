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

static uint32_t   pd0;
static unsigned   pd0_accesses;
static bool       cs_level_high;
static bool       dma_err_flag;
static bool       sda_high;
static bool       scl_high;
static uint32_t   ev_flags;
static unsigned   releases;
volatile uint32_t bridge_systick_count;

uint32_t *mock_exti_pd0(void)
{
	pd0_accesses++;
	return &pd0;
}

FlagStatus gpio_input_bit_get(uint32_t port, uint32_t pin)
{
	if (port == BRIDGE_I2C_SDA_PORT && pin == BRIDGE_I2C_SDA_PIN) return sda_high ? SET : RESET;
	if (port == BRIDGE_I2C_SCL_PORT && pin == BRIDGE_I2C_SCL_PIN) return scl_high ? SET : RESET;
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
	return (ev_flags & f) ? SET : RESET;
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
void spi_slave_cs_high(void)
{
}
void spi_slave_transport_error(void)
{
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
	releases++; /* only the stuck poll / ER bus-error arm call this */
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
	pd0                   = 1u << 8; /* line 8 (CS) pending at entry */
	pd0_accesses          = 0u;
	cs_level_high         = true;
	dma_err_flag          = false;
	mock_dma_chctl        = 0u; /* CHEN clear: channels quiesce */
	mock_spi_stat         = 0u;
	spi_dma_error_pending = false;
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

/* ---- #315: the stuck-SDA poll must never act on a live transfer --------- */
/* One base-level tick, `periods` 50 ms SysTick periods after the last. */
static void poll_after(uint32_t periods)
{
	bridge_systick_count += periods;
	bridge_transport_i2c_stuck_poll();
}

/* The wedge signature: SDA held low, SCL released high. */
static void wedge_pads(void)
{
	sda_high = false;
	scl_high = true;
	ev_flags = 0u;
	releases = 0u;
}

/* One serviced I2C0 event, as the real ISR would see it. */
static void isr_event(uint32_t flag)
{
	ev_flags = flag;
	BRIDGE_I2C_EV_HANDLER();
	ev_flags = 0u;
}

/* #296 cold-boot case: idle bus (no transaction), SDA held low, SCL high,
 * no ISR activity -> released on the second consecutive tick, as before. */
ZTEST(transport_hw_cs_exti, test_stuck_poll_idle_wedge_still_releases)
{
	wedge_pads();
	poll_after(1u);
	zassert_equal(releases, 0u, "one candidate tick must not act");
	poll_after(1u);
	zassert_equal(releases, 1u, "two silent candidate ticks must release the idle wedge");
}

/* #315: a write whose last byte's ACK phase has SCL high / SDA low while
 * the master is preempted.  The transaction is open (ADDSEND, bytes, no
 * STPDET).  Ticks arrive for 150 ms of that stall; none may release. */
ZTEST(transport_hw_cs_exti, test_stuck_poll_does_not_release_a_live_write)
{
	wedge_pads();
	isr_event(I2C_INT_FLAG_ADDSEND);
	for (int i = 0; i < 64; i++)
		isr_event(I2C_INT_FLAG_RBNE);
	for (int t = 0; t < 4; t++) {
		poll_after(1u);
		zassert_equal(releases, 0u, "released a live transfer (tick %d)", t);
	}
}

/* Each byte cancels the candidate: a slow but steady write never releases,
 * however long it runs. */
ZTEST(transport_hw_cs_exti, test_stuck_poll_activity_cancels_the_candidate)
{
	wedge_pads();
	isr_event(I2C_INT_FLAG_ADDSEND);
	for (int t = 0; t < 40; t++) {
		isr_event(I2C_INT_FLAG_RBNE);
		poll_after(1u);
	}
	zassert_equal(releases, 0u, "a transfer making progress was released");
}

/* An open transaction that goes silent for good (the erratum wedge can
 * leave ADDSEND without a STOP) is still recovered, after the live-quiet
 * window rather than after two ticks. */
ZTEST(transport_hw_cs_exti, test_stuck_poll_releases_a_silent_open_transaction)
{
	wedge_pads();
	isr_event(I2C_INT_FLAG_ADDSEND);
	for (int t = 0; t < 4; t++)
		poll_after(1u);
	zassert_equal(releases, 0u, "released inside the live-quiet window");
	for (int t = 0; t < 3; t++)
		poll_after(1u);
	zassert_equal(releases, 1u, "silent open transaction was never recovered");
}

/* A completed transaction (STPDET) is an idle bus again. */
ZTEST(transport_hw_cs_exti, test_stuck_poll_stop_returns_to_idle_rule)
{
	wedge_pads();
	isr_event(I2C_INT_FLAG_ADDSEND);
	isr_event(I2C_INT_FLAG_STPDET);
	poll_after(1u); /* activity seen: candidate restarts */
	poll_after(1u);
	poll_after(1u);
	zassert_equal(releases, 1u, "post-STOP wedge must use the two-tick rule");
}
