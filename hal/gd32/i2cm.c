/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 bridge HAL backend -- I2C3 master proxy (protocol v0.17,
 * CMD_I2CM_CONFIG / _XFER / _RESULT), register driver half.  The job
 * state machine lives in i2cm_core.c; this file is the polled I2C3 master
 * behind its i2cm_ops_t plus the bridge_hw_i2cm_*() HAL entry points.
 *
 * Wiring (SoM 2625-R2 + X-EVK V2): GD32 U41 PC8 = I2C3_SCL (E1M-X pad
 * A24), PC9 = I2C3_SDA (pad A23).  No pull-ups on the SoM: the carrier /
 * module provides them.  The pads are hi-Z (analog) until the first
 * CMD_I2CM_CONFIG and again after a Deep-sleep wake.
 *
 * UNVERIFIED (no board access when this was written -- bench before
 * relying on it):
 *   - the alternate-function number for I2C3_SCL/SDA on PC8/PC9
 *     (BRIDGE_I2CM_GPIO_AF, hal/bridge_board_config.h): no GD32G553
 *     datasheet AF table was available, so the driver refuses CONFIG with
 *     BRIDGE_HW_ERR_NOTIMPL unless the build defines it;
 *   - the master state-machine details (automatic STOP after a NACK, TC
 *     handling, NBYTES = 0 address-only probe) follow the vendor SPL
 *     (gd32g5x3_i2c.h, Examples/I2C) and the I2C-bus spec, not a silicon
 *     run.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bridge_hw.h"
#include "gd32g5x3.h"

#include "bridge_board_config.h"
#include "bridge_critical.h"
#include "gd32_common.h"
#include "i2c_timeout.h"
#include "i2cm_core.h"

/* Job deadline and SCL-low (clock-stretch) limit.  The SCL-low limit is
 * the hardware TIMEOUTA counter (UM Rev1.2 SS28.3.9, (reload + 1) * 2048 *
 * tI2CCLK); the job deadline is measured on SysTick. */
#define I2CM_JOB_DEADLINE_MS 20u
#define I2CM_SCL_LOW_MS      10u
/* Recovery: up to 9 SCL pulses at ~100 kHz (5 us low / 5 us high). */
#define I2CM_RECOVERY_PULSES  9u
#define I2CM_RECOVERY_HALF_US 5u

/* ----------------------------------------------------------------- */
/* Time bases                                                         */
/* ----------------------------------------------------------------- */

/* Busy-wait ~us microseconds.  ~5 cycles per volatile iteration, same
 * estimate hal/gd32/gpio.c uses; scales with the live core clock. */
static void i2cm_delay_us(uint32_t us)
{
	uint32_t per_us = SystemCoreClock / 5000000u;
	if (per_us == 0u) per_us = 1u;
	for (volatile uint32_t i = 0u; i < us * per_us; ++i) {
	}
}

/* Elapsed core cycles since @p start (a SysTick->VAL sample).  SysTick
 * counts down over a 50 ms period (hal/gd32/init.c), so this is valid for
 * any interval shorter than one period -- the 20 ms job deadline is. */
static uint32_t systick_elapsed(uint32_t start)
{
	const uint32_t now = SysTick->VAL;
	return (now <= start) ? (start - now) : (start + (SysTick->LOAD + 1u - now));
}

typedef struct {
	uint32_t start;
	uint32_t limit; /* cycles */
} i2cm_deadline_t;

static i2cm_deadline_t deadline_start(uint32_t ms)
{
	i2cm_deadline_t d = { SysTick->VAL, (SystemCoreClock / 1000u) * ms };
	return d;
}

static bool deadline_expired(const i2cm_deadline_t *d)
{
	return systick_elapsed(d->start) >= d->limit;
}

/* ----------------------------------------------------------------- */
/* Pads                                                               */
/* ----------------------------------------------------------------- */

static void pads_hiz(void)
{
	gpio_mode_set(BRIDGE_I2CM_SCL_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, BRIDGE_I2CM_SCL_PIN);
	gpio_mode_set(BRIDGE_I2CM_SDA_PORT, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, BRIDGE_I2CM_SDA_PIN);
}

/* Both pads GPIO open-drain, released high; lets the recovery pulse SCL
 * and read the real pad level (ISTAT stays valid in output mode). */
static void pads_gpio_od(void)
{
	GPIO_BOP(BRIDGE_I2CM_SCL_PORT) = BRIDGE_I2CM_SCL_PIN;
	GPIO_BOP(BRIDGE_I2CM_SDA_PORT) = BRIDGE_I2CM_SDA_PIN;
	gpio_output_options_set(
	    BRIDGE_I2CM_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_12MHZ, BRIDGE_I2CM_SCL_PIN);
	gpio_output_options_set(
	    BRIDGE_I2CM_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_12MHZ, BRIDGE_I2CM_SDA_PIN);
	gpio_mode_set(BRIDGE_I2CM_SCL_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BRIDGE_I2CM_SCL_PIN);
	gpio_mode_set(BRIDGE_I2CM_SDA_PORT, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, BRIDGE_I2CM_SDA_PIN);
}

static void pads_af(void)
{
	/* Open-drain, slowest drive class (same reasoning as I2C0, see
	 * i2c_gpio_init() in hal/transport_hw_gd32.c); no internal pull. */
	gpio_output_options_set(
	    BRIDGE_I2CM_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_12MHZ, BRIDGE_I2CM_SCL_PIN);
	gpio_output_options_set(
	    BRIDGE_I2CM_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_12MHZ, BRIDGE_I2CM_SDA_PIN);
	gpio_af_set(BRIDGE_I2CM_SCL_PORT, BRIDGE_I2CM_GPIO_AF, BRIDGE_I2CM_SCL_PIN);
	gpio_af_set(BRIDGE_I2CM_SDA_PORT, BRIDGE_I2CM_GPIO_AF, BRIDGE_I2CM_SDA_PIN);
	gpio_mode_set(BRIDGE_I2CM_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BRIDGE_I2CM_SCL_PIN);
	gpio_mode_set(BRIDGE_I2CM_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BRIDGE_I2CM_SDA_PIN);
}

static bool sda_high(void)
{
	return (GPIO_ISTAT(BRIDGE_I2CM_SDA_PORT) & BRIDGE_I2CM_SDA_PIN) != 0u;
}

static bool scl_high(void)
{
	return (GPIO_ISTAT(BRIDGE_I2CM_SCL_PORT) & BRIDGE_I2CM_SCL_PIN) != 0u;
}

/* 9-clock bus recovery: pulse SCL (PC8 as GPIO open-drain, ~100 kHz) until
 * a slave that was mid-byte releases SDA, then a STOP.  Leaves the pads in
 * GPIO mode; the caller restores the AF.  Returns true when SDA and SCL are
 * both high afterwards (false = BUS_STUCK). */
static bool bus_recover(void)
{
	pads_gpio_od();
	i2cm_delay_us(I2CM_RECOVERY_HALF_US);
	for (uint32_t i = 0u; i < I2CM_RECOVERY_PULSES && !sda_high(); ++i) {
		GPIO_BC(BRIDGE_I2CM_SCL_PORT) = BRIDGE_I2CM_SCL_PIN;
		i2cm_delay_us(I2CM_RECOVERY_HALF_US);
		GPIO_BOP(BRIDGE_I2CM_SCL_PORT) = BRIDGE_I2CM_SCL_PIN;
		i2cm_delay_us(I2CM_RECOVERY_HALF_US);
	}
	/* STOP: SDA low while SCL low, SCL high, then SDA high. */
	GPIO_BC(BRIDGE_I2CM_SCL_PORT) = BRIDGE_I2CM_SCL_PIN;
	i2cm_delay_us(I2CM_RECOVERY_HALF_US);
	GPIO_BC(BRIDGE_I2CM_SDA_PORT) = BRIDGE_I2CM_SDA_PIN;
	i2cm_delay_us(I2CM_RECOVERY_HALF_US);
	GPIO_BOP(BRIDGE_I2CM_SCL_PORT) = BRIDGE_I2CM_SCL_PIN;
	i2cm_delay_us(I2CM_RECOVERY_HALF_US);
	GPIO_BOP(BRIDGE_I2CM_SDA_PORT) = BRIDGE_I2CM_SDA_PIN;
	i2cm_delay_us(I2CM_RECOVERY_HALF_US);
	return sda_high() && scl_high();
}

/* ----------------------------------------------------------------- */
/* Peripheral                                                         */
/* ----------------------------------------------------------------- */

/* Documented software reset: PE = 0 for >= 3 APB cycles, then PE = 1
 * (same sequence as the I2C0 slave, hal/gd32/i2c_recovery.h).  Clears the
 * status flags and the internal state machine; timing/timeout registers
 * are kept. */
static void periph_soft_reset(void)
{
	i2c_disable(BRIDGE_I2CM_PERIPH);
	for (uint32_t spins = 0u; (I2C_CTL0(BRIDGE_I2CM_PERIPH) & I2C_CTL0_I2CEN) != 0u && spins < 64u;
	     ++spins) {
	}
	i2c_enable(BRIDGE_I2CM_PERIPH);
}

static void clear_errors(void)
{
	i2c_flag_clear(BRIDGE_I2CM_PERIPH,
	               I2C_FLAG_NACK | I2C_FLAG_STPDET | I2C_FLAG_BERR | I2C_FLAG_LOSTARB |
	                   I2C_FLAG_TIMEOUT);
}

static int i2cm_bus_enable(uint16_t bus_khz)
{
	/* TODO(unverified): BRIDGE_I2CM_GPIO_AF -- see bridge_board_config.h.
	 * Refuse rather than mux PC8/PC9 to a guessed function. */
	if (!BRIDGE_I2CM_AF_VERIFIED) return BRIDGE_HW_ERR_NOTIMPL;

	const uint32_t primask = bridge_irq_lock();
	rcu_i2c_clock_config(BRIDGE_I2CM_RCU_IDX, BRIDGE_I2CM_CK_SRC);
	bridge_irq_unlock(primask);
	bridge_rcu_periph_clock_enable(BRIDGE_I2CM_RCU);

	i2c_disable(BRIDGE_I2CM_PERIPH);

	/* Best-effort recovery before taking the pads: a slave left mid-byte by
	 * a previous owner is released here.  A bus that stays stuck still
	 * configures; the first job reports BUS_STUCK. */
	(void)bus_recover();
	pads_af();

	/* Fast-mode-derived PSC / SCLDELY / SDADELY (shared with the I2C0
	 * slave) plus SCLH / SCLL for the requested speed. */
	const uint32_t i2cclk_hz = rcu_clock_freq_get(CK_APB1);
	uint32_t       psc, scl_dely, sda_dely, sclh, scll;
	uint16_t       scl_low_reload;
	if (!bridge_i2c_timing_derive(i2cclk_hz, &psc, &scl_dely, &sda_dely) ||
	    !i2cm_scl_counts(i2cclk_hz, psc, bus_khz, &sclh, &scll)) {
		pads_hiz();
		return BRIDGE_HW_ERR_RANGE;
	}
	/* (reload + 1) * 2048 * tI2CCLK >= I2CM_SCL_LOW_MS */
	const uint64_t reload = (((uint64_t)i2cclk_hz * I2CM_SCL_LOW_MS) / 1000u + 2047u) / 2048u;
	if (reload == 0u || reload > 0x1000u) {
		pads_hiz();
		return BRIDGE_HW_ERR_RANGE;
	}
	scl_low_reload = (uint16_t)(reload - 1u);

	i2c_timing_config(BRIDGE_I2CM_PERIPH, psc, scl_dely, sda_dely);
	i2c_master_clock_config(BRIDGE_I2CM_PERIPH, sclh, scll);
	i2c_analog_noise_filter_enable(BRIDGE_I2CM_PERIPH);
	/* SCL held low longer than the limit raises TIMEOUT (flag only; the job
	 * polls it and recovers). */
	i2c_bus_timeout_a_config(BRIDGE_I2CM_PERIPH, scl_low_reload);
	i2c_clock_timeout_enable(BRIDGE_I2CM_PERIPH);
	i2c_enable(BRIDGE_I2CM_PERIPH);
	clear_errors();
	return BRIDGE_HW_OK;
}

static void i2cm_bus_release(void)
{
	if (BRIDGE_I2CM_AF_VERIFIED) i2c_disable(BRIDGE_I2CM_PERIPH);
	pads_hiz();
}

/* Wait until @p flags (any) is set; returns the flag set seen, 0 on the
 * job deadline.  Error flags are always reported ahead of the awaited
 * ones. */
#define I2CM_ERR_FLAGS (I2C_FLAG_BERR | I2C_FLAG_LOSTARB | I2C_FLAG_TIMEOUT)

static uint32_t wait_flags(uint32_t flags, const i2cm_deadline_t *dl)
{
	for (;;) {
		const uint32_t stat = I2C_STAT(BRIDGE_I2CM_PERIPH);
		if ((stat & I2CM_ERR_FLAGS) != 0u) return stat & I2CM_ERR_FLAGS;
		if ((stat & flags) != 0u) return stat & flags;
		if (deadline_expired(dl)) return 0u;
	}
}

static uint8_t error_to_result(uint32_t stat)
{
	if ((stat & I2C_FLAG_LOSTARB) != 0u) return I2CM_RES_ARB_LOST;
	if ((stat & I2C_FLAG_TIMEOUT) != 0u) return I2CM_RES_TIMEOUT;
	return I2CM_RES_BUS_ERROR;
}

/* Finish a failed job: reset the peripheral state machine, then make sure
 * the bus is free.  BUS_STUCK overrides @p res when SDA/SCL stay low after
 * the recovery pulses. */
static uint8_t fail_and_recover(uint8_t res)
{
	periph_soft_reset();
	clear_errors();
	if (!sda_high() || !scl_high()) {
		i2c_disable(BRIDGE_I2CM_PERIPH);
		const bool freed = bus_recover();
		pads_af();
		i2c_enable(BRIDGE_I2CM_PERIPH);
		clear_errors();
		if (!freed) return I2CM_RES_BUS_STUCK;
	}
	return res;
}

/* After a NACK the master generates a STOP by itself; wait for it (bounded)
 * and clear the flags.  Force a STOP if it does not show up. */
static void settle_after_nack(const i2cm_deadline_t *dl)
{
	i2cm_deadline_t grace = deadline_start(1u);
	while ((I2C_STAT(BRIDGE_I2CM_PERIPH) & I2C_FLAG_STPDET) == 0u) {
		if (deadline_expired(&grace) || deadline_expired(dl)) {
			i2c_stop_on_bus(BRIDGE_I2CM_PERIPH);
			break;
		}
	}
	i2cm_deadline_t stop_wait = deadline_start(1u);
	while ((I2C_STAT(BRIDGE_I2CM_PERIPH) & I2C_FLAG_STPDET) == 0u &&
	       !deadline_expired(&stop_wait)) {
	}
	clear_errors();
}

static uint8_t i2cm_run(uint8_t        addr7,
                        const uint8_t *wdata,
                        uint8_t        wlen,
                        uint8_t       *rdata,
                        uint8_t        rlen,
                        uint8_t       *nread)
{
	const uint32_t  addr = (uint32_t)addr7 << 1;
	i2cm_deadline_t dl   = deadline_start(I2CM_JOB_DEADLINE_MS);
	uint32_t        got;

	*nread = 0u;

	/* Never start on a bus that is already held; a peripheral that thinks it
	 * is busy with lines idle is cleared by the soft reset. */
	if ((I2C_STAT(BRIDGE_I2CM_PERIPH) & I2C_FLAG_I2CBSY) != 0u) periph_soft_reset();
	clear_errors();
	if (!sda_high() || !scl_high()) return fail_and_recover(I2CM_RES_BUS_STUCK);

	/* ---- write phase (or the address-only quick probe) ---- */
	if (wlen != 0u || rlen == 0u) {
		i2c_master_addressing(BRIDGE_I2CM_PERIPH, addr, I2C_MASTER_TRANSMIT);
		i2c_transfer_byte_number_config(BRIDGE_I2CM_PERIPH, wlen);
		i2c_reload_disable(BRIDGE_I2CM_PERIPH);
		if (rlen == 0u) {
			i2c_automatic_end_enable(BRIDGE_I2CM_PERIPH);
		} else {
			i2c_automatic_end_disable(BRIDGE_I2CM_PERIPH);
		}
		i2c_start_on_bus(BRIDGE_I2CM_PERIPH);

		uint8_t sent = 0u;
		while (sent < wlen) {
			got = wait_flags(I2C_FLAG_TI | I2C_FLAG_NACK, &dl);
			if (got == 0u) return fail_and_recover(I2CM_RES_TIMEOUT);
			if ((got & I2CM_ERR_FLAGS) != 0u) return fail_and_recover(error_to_result(got));
			if ((got & I2C_FLAG_NACK) != 0u) {
				settle_after_nack(&dl);
				return (sent == 0u) ? I2CM_RES_NACK_ADDR : I2CM_RES_NACK_DATA;
			}
			i2c_data_transmit(BRIDGE_I2CM_PERIPH, wdata[sent]);
			sent++;
		}
		if (rlen == 0u) {
			/* AUTOEND: the STOP follows the last byte (or the address, for
			 * the quick probe); a NACK ends the job first. */
			got = wait_flags(I2C_FLAG_STPDET | I2C_FLAG_NACK, &dl);
			if (got == 0u) return fail_and_recover(I2CM_RES_TIMEOUT);
			if ((got & I2CM_ERR_FLAGS) != 0u) return fail_and_recover(error_to_result(got));
			if ((got & I2C_FLAG_NACK) != 0u) {
				settle_after_nack(&dl);
				return (wlen == 0u) ? I2CM_RES_NACK_ADDR : I2CM_RES_NACK_DATA;
			}
			clear_errors();
			return I2CM_RES_OK;
		}
		/* S W.. Sr R..: wait for transfer complete, no STOP yet. */
		got = wait_flags(I2C_FLAG_TC | I2C_FLAG_NACK, &dl);
		if (got == 0u) return fail_and_recover(I2CM_RES_TIMEOUT);
		if ((got & I2CM_ERR_FLAGS) != 0u) return fail_and_recover(error_to_result(got));
		if ((got & I2C_FLAG_NACK) != 0u) {
			settle_after_nack(&dl);
			return I2CM_RES_NACK_DATA;
		}
	}

	/* ---- read phase ---- */
	i2c_master_addressing(BRIDGE_I2CM_PERIPH, addr, I2C_MASTER_RECEIVE);
	i2c_transfer_byte_number_config(BRIDGE_I2CM_PERIPH, rlen);
	i2c_reload_disable(BRIDGE_I2CM_PERIPH);
	i2c_automatic_end_enable(BRIDGE_I2CM_PERIPH);
	i2c_start_on_bus(BRIDGE_I2CM_PERIPH); /* START, or the repeated START */

	uint8_t received = 0u;
	while (received < rlen) {
		got = wait_flags(I2C_FLAG_RBNE | I2C_FLAG_NACK, &dl);
		if (got == 0u) return fail_and_recover(I2CM_RES_TIMEOUT);
		if ((got & I2CM_ERR_FLAGS) != 0u) return fail_and_recover(error_to_result(got));
		if ((got & I2C_FLAG_RBNE) != 0u) {
			rdata[received++] = (uint8_t)i2c_data_receive(BRIDGE_I2CM_PERIPH);
		} else {
			/* NACK before any data = address NACK (the slave never NACKs
			 * a read byte; the master NACKs the last one itself). */
			settle_after_nack(&dl);
			return (received == 0u) ? I2CM_RES_NACK_ADDR : I2CM_RES_NACK_DATA;
		}
	}
	got = wait_flags(I2C_FLAG_STPDET, &dl);
	if (got == 0u) return fail_and_recover(I2CM_RES_TIMEOUT);
	if ((got & I2CM_ERR_FLAGS) != 0u) return fail_and_recover(error_to_result(got));
	clear_errors();
	*nread = received;
	return I2CM_RES_OK;
}

static const i2cm_ops_t i2cm_ops = {
	.bus_enable  = i2cm_bus_enable,
	.bus_release = i2cm_bus_release,
	.run         = i2cm_run,
};

/* ----------------------------------------------------------------- */
/* HAL entry points                                                   */
/* ----------------------------------------------------------------- */

/* Called from bridge_hw_init(): pads stay parked (hi-Z) until CONFIG. */
void bridge_hw_i2cm_init(void)
{
	i2cm_core_init(&i2cm_ops);
}

int bridge_hw_i2cm_config(uint16_t bus_khz)
{
	return i2cm_core_config(bus_khz);
}

int bridge_hw_i2cm_xfer(uint8_t        tag,
                        uint8_t        addr7,
                        const uint8_t *wdata,
                        uint8_t        wlen,
                        uint8_t        rlen)
{
	return i2cm_core_xfer(tag, addr7, wdata, wlen, rlen);
}

int bridge_hw_i2cm_result(uint8_t *tag, uint8_t *result, uint8_t *nread, uint8_t *rdata)
{
	return i2cm_core_result(tag, result, nread, rdata);
}

bool bridge_hw_i2cm_busy(void)
{
	return i2cm_core_busy();
}

/* Base level, from bridge_hw_tick(). */
void bridge_hw_i2cm_tick(void)
{
	i2cm_core_tick();
}

/* Deep-sleep wake (hal/gd32/power.c): drop to unconfigured. */
void bridge_hw_i2cm_wake(void)
{
	i2cm_core_mark_unconfigured();
}
