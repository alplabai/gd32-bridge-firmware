/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C3 master proxy -- job state machine.  See i2cm_core.h.
 */

#include <stddef.h>
#include <string.h>

#include "bridge_hw.h"
#include "i2cm_core.h"

typedef enum {
	I2CM_UNCONF = 0, /* pads hi-Z, no CONFIG since boot / wake / CONFIG(0) */
	I2CM_IDLE,       /* configured, no job since CONFIG                    */
	I2CM_QUEUED,     /* XFER accepted, tick has not started it             */
	I2CM_RUNNING,    /* tick is on the bus                                 */
	I2CM_DONE,       /* result valid until the next XFER / CONFIG          */
} i2cm_state_t;

static volatile uint8_t  s_state; /* i2cm_state_t */
static const i2cm_ops_t *s_ops;

/* Job (owned by the handler until QUEUED, then by the tick). */
static uint8_t s_job_tag, s_job_addr, s_job_wlen, s_job_rlen;
static uint8_t s_job_w[I2CM_MAX_WRITE];

/* Result (owned by the tick until DONE, then readable by the handler). */
static uint8_t s_res_tag, s_res_code, s_res_nread;
static uint8_t s_res_data[I2CM_MAX_READ];

/* Speed of the last successful CONFIG, 0 = none (SWD-readable). */
static volatile uint16_t i2cm_khz;

void i2cm_core_init(const i2cm_ops_t *ops)
{
	s_ops    = ops;
	s_state  = I2CM_UNCONF;
	i2cm_khz = 0u;
}

bool i2cm_core_busy(void)
{
	const uint8_t st = s_state;
	return st == I2CM_QUEUED || st == I2CM_RUNNING;
}

int i2cm_core_config(uint16_t bus_khz)
{
	if (bus_khz != 0u && bus_khz != 100u && bus_khz != 400u) return BRIDGE_HW_ERR_INVAL;
	if (i2cm_core_busy()) return BRIDGE_HW_ERR_BUSY;
	if (bus_khz == 0u) {
		s_ops->bus_release();
		s_state  = I2CM_UNCONF;
		i2cm_khz = 0u;
		return BRIDGE_HW_OK;
	}
	const int rv = s_ops->bus_enable(bus_khz);
	if (rv != BRIDGE_HW_OK) {
		s_ops->bus_release();
		s_state  = I2CM_UNCONF;
		i2cm_khz = 0u;
		return rv;
	}
	i2cm_khz = bus_khz;
	s_state  = I2CM_IDLE; /* also discards an uncollected result */
	return BRIDGE_HW_OK;
}

int i2cm_core_xfer(uint8_t tag, uint8_t addr7, const uint8_t *wdata, uint8_t wlen, uint8_t rlen)
{
	if (wlen > I2CM_MAX_WRITE || rlen > I2CM_MAX_READ || addr7 > 0x7Fu) return BRIDGE_HW_ERR_INVAL;
	if (wlen != 0u && wdata == NULL) return BRIDGE_HW_ERR_INVAL;
	const uint8_t st = s_state;
	if (st == I2CM_UNCONF) return BRIDGE_HW_ERR_NOT_READY;
	if (st == I2CM_QUEUED || st == I2CM_RUNNING) return BRIDGE_HW_ERR_BUSY;
	s_job_tag  = tag;
	s_job_addr = addr7;
	s_job_wlen = wlen;
	s_job_rlen = rlen;
	if (wlen != 0u) memcpy(s_job_w, wdata, wlen);
	/* The tick runs from base level while this runs from an ISR: the job
	 * fields must be visible before the state that hands them over. */
	__atomic_thread_fence(__ATOMIC_RELEASE);
	s_state = I2CM_QUEUED; /* publish last; discards an uncollected result */
	return BRIDGE_HW_OK;
}

int i2cm_core_result(uint8_t *tag, uint8_t *result, uint8_t *nread, uint8_t *rdata)
{
	const uint8_t st = s_state;
	if (st == I2CM_QUEUED || st == I2CM_RUNNING) return BRIDGE_HW_ERR_BUSY;
	if (st != I2CM_DONE) return BRIDGE_HW_ERR_NOT_READY;
	__atomic_thread_fence(__ATOMIC_ACQUIRE);
	*tag    = s_res_tag;
	*result = s_res_code;
	*nread  = s_res_nread;
	if (s_res_nread != 0u) memcpy(rdata, s_res_data, s_res_nread);
	return BRIDGE_HW_OK;
}

void i2cm_core_tick(void)
{
	if (s_state != I2CM_QUEUED) return;
	__atomic_thread_fence(__ATOMIC_ACQUIRE); /* job fields published before QUEUED */
	s_state = I2CM_RUNNING;
	/* ponytail: the transfer blocks base level for up to its 20 ms
	 * deadline (polled driver, no DMA/IRQ).  The transport ISRs keep
	 * running and the FWDGT budget (hal/gd32/init.c) covers it; if a longer
	 * job or a second blocking pump ever lands, move the polling into an
	 * IRQ-driven state machine. */
	uint8_t nread = 0u;
	s_res_code    = s_ops->run(s_job_addr, s_job_w, s_job_wlen, s_res_data, s_job_rlen, &nread);
	if (s_res_code != (uint8_t)I2CM_RES_OK || nread > s_job_rlen) nread = 0u;
	s_res_tag   = s_job_tag;
	s_res_nread = nread;
	__atomic_thread_fence(__ATOMIC_RELEASE); /* result visible before DONE */
	s_state = I2CM_DONE;
}

void i2cm_core_mark_unconfigured(void)
{
	if (i2cm_core_busy()) return;
	s_ops->bus_release();
	s_state  = I2CM_UNCONF;
	i2cm_khz = 0u;
}

/* Nominal SCL times (ns), sized so tLOW + tHIGH >= 1 / fSCLmax even with
 * zero rise/fall time, i.e. fSCL never exceeds 100 / 400 kHz: 100 kHz =
 * 4700 + 5300 = 10 us; 400 kHz = 1400 + 1100 = 2.5 us.  Both phases stay at
 * or above the I2C-bus specification (UM10204, timing table) minimums:
 * Standard-mode tLOW >= 4700 / tHIGH >= 4000; Fast-mode tLOW >= 1300 /
 * tHIGH >= 600.  Rise/fall and synchronisation only lengthen the period
 * further.  Calibration knobs: measure SCL on the carrier and trim these. */
#define I2CM_SCLL_NS_100K 4700u
#define I2CM_SCLH_NS_100K 5300u
#define I2CM_SCLL_NS_400K 1400u
#define I2CM_SCLH_NS_400K 1100u

static bool counts_for(uint64_t t_psc_ps, uint32_t ns, uint32_t *out)
{
	/* ceil(ns / tPSC) - 1, never below 0 */
	const uint64_t ticks = ((uint64_t)ns * 1000u + t_psc_ps - 1u) / t_psc_ps;
	const uint64_t v     = ticks > 0u ? ticks - 1u : 0u;
	if (v > 255u) return false;
	*out = (uint32_t)v;
	return true;
}

bool i2cm_scl_counts(uint32_t  i2cclk_hz,
                     uint32_t  psc,
                     uint16_t  bus_khz,
                     uint32_t *sclh,
                     uint32_t *scll)
{
	if (i2cclk_hz == 0u || psc > 15u) return false;
	uint32_t low_ns, high_ns;
	if (bus_khz == 100u) {
		low_ns  = I2CM_SCLL_NS_100K;
		high_ns = I2CM_SCLH_NS_100K;
	} else if (bus_khz == 400u) {
		low_ns  = I2CM_SCLL_NS_400K;
		high_ns = I2CM_SCLH_NS_400K;
	} else {
		return false;
	}
	const uint64_t t_psc_ps = (uint64_t)(psc + 1u) * (1000000000000ULL / i2cclk_hz);
	return counts_for(t_psc_ps, low_ns, scll) && counts_for(t_psc_ps, high_ns, sclh);
}
