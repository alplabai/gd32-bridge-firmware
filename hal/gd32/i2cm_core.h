/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C3 master proxy -- vendor-header-free job state machine (protocol v0.17,
 * CMD_I2CM_CONFIG / _XFER / _RESULT).  The register driver sits behind
 * i2cm_ops_t so the whole queue/result contract runs in host CI.
 *
 * Concurrency: the request handlers (transport ISR context, serialised by
 * protocol_dispatch()) and i2cm_core_tick() (base level) share `s_state`.
 * Cortex-M33 byte stores are atomic and every transition has ONE owner:
 * a handler moves UNCONF/IDLE/DONE -> QUEUED (after filling the job
 * buffer), the tick moves QUEUED -> RUNNING -> DONE (after filling the
 * result buffer).  A handler that preempts the tick sees QUEUED/RUNNING and
 * answers BUSY, so neither side ever touches a buffer the other owns.
 */

#ifndef GD32_BRIDGE_HAL_GD32_I2CM_CORE_H
#define GD32_BRIDGE_HAL_GD32_I2CM_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "bridge_hw.h" /* I2CM_DIAG_LEN */
#include "protocol.h" /* I2CM_MAX_WRITE / I2CM_MAX_READ, gd32_bridge_i2cm_result_t */

typedef struct {
	/* Take the pads + peripheral at @p bus_khz (100 or 400): run the 9-clock
	 * bus recovery, mux the pads to I2C3, program timing, enable.  Returns
	 * BRIDGE_HW_OK or a BRIDGE_HW_ERR_* (e.g. NOTIMPL / RANGE). */
	int (*bus_enable)(uint16_t bus_khz);
	/* Peripheral off, PC8/PC9 back to hi-Z. */
	void (*bus_release)(void);
	/* Run one blocking transfer; returns a gd32_bridge_i2cm_result_t.
	 * Must honour the SCL-low / job deadlines itself and leave the bus
	 * recovered after a failure. */
	uint8_t (*run)(uint8_t        addr7,
	               const uint8_t *wdata,
	               uint8_t        wlen,
	               uint8_t       *rdata,
	               uint8_t        rlen,
	               uint8_t       *nread);
} i2cm_ops_t;

/* ---- CMD_I2CM_DIAG (bench only) reply layout, all little-endian ----------
 * off  size  field
 *   0   u32  GPIO_CTL(GPIOC)      2 bits/pin; PC8 = bits 17:16, PC9 = 19:18
 *   4   u32  GPIO_OMODE(GPIOC)    1 = open-drain
 *   8   u32  GPIO_PUD(GPIOC)
 *  12   u32  GPIO_ISTAT(GPIOC)    PC8 = bit 8, PC9 = bit 9
 *  16   u32  GPIO_OCTL(GPIOC)
 *  20   u32  GPIO_AFSEL1(GPIOC)   PC8 = bits 3:0, PC9 = bits 7:4
 *  24   u32  RCU_AHB2EN           bit 19 = GPIOC clock
 *  28   u32  RCU_APB1EN           bit 23 = I2C2 clock
 *  32   u32  RCU_CFG2             bits 5:4 = I2C2SEL
 *  36   u32  I2C_CTL0(I2C2)       0 while the I2C2 clock is off
 *  40   u32  I2C_STAT(I2C2)       0 while the I2C2 clock is off
 *  44   u32  I2C_TIMING(I2C2)     0 while the I2C2 clock is off
 *  48   u32  GPIO_LOCK(GPIOC)     bit 16 = LKK, 15:0 = locked pins
 *  52   u8   capture 1: i2cm_run, right before the first BUS_STUCK decision
 *  53   u8   capture 2: fail_and_recover, right before its BUS_STUCK decision
 *            capture byte: b0 = PC8 ISTAT, b1 = PC9 ISTAT, b3:2 = PC8 CTL
 *            mode, b5:4 = PC9 CTL mode, b7 = valid (0 = never captured)
 *  54   u8   i2cm_core state (0 UNCONF 1 IDLE 2 QUEUED 3 RUNNING 4 DONE)
 *  55   u8   last job result (gd32_bridge_i2cm_result_t, 0 before any job)
 *  56   8*u8 zero
 * ------------------------------------------------------------------------ */
#define I2CM_DIAG_NREGS 13u

/* Pure serialiser behind the register reads (host-testable). */
void i2cm_diag_pack(uint8_t        out[I2CM_DIAG_LEN],
                    const uint32_t regs[I2CM_DIAG_NREGS],
                    uint8_t        cap1,
                    uint8_t        cap2,
                    uint8_t        state,
                    uint8_t        last_result);

/* Current state byte and last job result, for the diag reply. */
void i2cm_core_diag_state(uint8_t *state, uint8_t *last_result);

/* Resets the state machine to "unconfigured, no job".  Does not touch pads. */
void i2cm_core_init(const i2cm_ops_t *ops);

int  i2cm_core_config(uint16_t bus_khz);
int  i2cm_core_xfer(uint8_t tag, uint8_t addr7, const uint8_t *wdata, uint8_t wlen, uint8_t rlen);
int  i2cm_core_result(uint8_t *tag, uint8_t *result, uint8_t *nread, uint8_t *rdata);
bool i2cm_core_busy(void);

/* Base level: run a queued job to completion. */
void i2cm_core_tick(void);

/* Deep-sleep wake: the peripheral and pads are not trusted across the mode,
 * so drop to "unconfigured" (Linux re-CONFIGs on NOT_READY).  Releases the
 * pads.  No-op while a job runs (the power-mode request is refused BUSY
 * then, so this cannot happen on the real path). */
void i2cm_core_mark_unconfigured(void);

/* SCL high/low counts (the I2C_TIMING SCLH / SCLL fields) for @p bus_khz at
 * the prescaled tick tPSC = (psc + 1) / i2cclk_hz:
 *   tSCLL = (SCLL + 1) * tPSC,  tSCLH = (SCLH + 1) * tPSC.
 * Nominal low/high times are the I2C-bus spec minimums plus margin (see
 * i2cm_core.c); real rise/fall and the 2-3 tI2CCLK input sync add a few
 * percent, so the achieved rate sits slightly under nominal.  Returns false
 * for an unsupported speed or a count that does not fit the 8-bit fields. */
bool i2cm_scl_counts(uint32_t  i2cclk_hz,
                     uint32_t  psc,
                     uint16_t  bus_khz,
                     uint32_t *sclh,
                     uint32_t *scll);

#endif /* GD32_BRIDGE_HAL_GD32_I2CM_CORE_H */
