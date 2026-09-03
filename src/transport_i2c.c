/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge firmware: I2C-slave transport.
 *
 * Wire framing (see ../docs/gd32-bridge-protocol.md §5):
 *
 *   WRITE: [S+ADDR+W] reg=0x00 CMD PAYLOAD CRC(CMD..PAYLOAD)
 *   READ : [S+ADDR+R] STATUS  PAYLOAD CRC(STATUS..PAYLOAD)
 *
 * On the V2N module the GD32 sits as a slave on I2C0 (PA15/PB9) at
 * 7-bit address GD32_BRIDGE_DEFAULT_I2C_ADDR (0x70 by default).
 * The slave-side ISR accumulates bytes during the write phase;
 * when the host issues a repeated-start read, the ISR clock-stretches
 * (holds SCL low) just long enough to run protocol_dispatch() before
 * clocking out the reply bytes.
 *
 * THIS FILE is SILICON-FREE: framing, CRC, staging and the
 * protocol_dispatch() hand-off only.  The byte-level GigaDevice
 * hookups (I2C0 slave init, address ACK, clock-stretching, ISR wiring)
 * live in the gd32 backend at hal/transport_hw_gd32.c and drive the
 * i2c_slave_*() seams below; the stub backend leaves the bring-up hook
 * a no-op so a unit-test mock can feed the same seams directly.
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "../hal/bridge_hw.h" /* BRIDGE_HW_OK */
#include "protocol.h"
#include "transport.h"

/* Weak default: the stub backend links this no-op so it needs no vendor
 * library.  The gd32 backend's hal/transport_hw_gd32.c overrides it with
 * the real I2C0 slave bring-up (and its real BRIDGE_HW_OK /
 * BRIDGE_HW_ERR_RANGE return). */
__attribute__((weak)) int bridge_transport_i2c_hw_init(void)
{
	return BRIDGE_HW_OK;
}

#define I2C_MAX_WRITE_BYTES \
	(1u /* reg */ + 1u /* CMD */ + GD32_BRIDGE_MAX_PAYLOAD_BYTES + 2u /* CRC */)
#define I2C_MAX_READ_BYTES (1u /* STATUS */ + GD32_BRIDGE_MAX_PAYLOAD_BYTES + 2u /* CRC */)

static uint8_t i2c_rx_buf[I2C_MAX_WRITE_BYTES];
static size_t  i2c_rx_len;

static uint8_t i2c_tx_buf[I2C_MAX_READ_BYTES];
static size_t  i2c_tx_len;
static size_t  i2c_tx_cursor;

/* "No pending command since last START" sentinel -- see §5 of the
 * protocol doc.  Set when a read transaction comes in before any
 * matching write completed. */
static bool pending_reply_valid;

static void stage_no_pending(void)
{
	i2c_tx_buf[0] = STATUS_NO_PENDING;
	/* Empty payload; CRC covers just the status byte. */
	const uint16_t crc = crc16_ccitt_false(i2c_tx_buf, 1u);
	i2c_tx_buf[1]      = (uint8_t)(crc & 0xFFu);
	i2c_tx_buf[2]      = (uint8_t)((crc >> 8) & 0xFFu);
	i2c_tx_len         = 3u;
	i2c_tx_cursor      = 0u;
}

static void stage_reply(uint8_t status, const uint8_t *payload, size_t payload_len)
{
	i2c_tx_buf[0] = status;
	if (payload_len > 0u && payload != NULL) {
		memcpy(&i2c_tx_buf[1], payload, payload_len);
	}
	const size_t   crc_covered   = 1u + payload_len;
	const uint16_t crc           = crc16_ccitt_false(i2c_tx_buf, crc_covered);
	i2c_tx_buf[crc_covered]      = (uint8_t)(crc & 0xFFu);
	i2c_tx_buf[crc_covered + 1u] = (uint8_t)((crc >> 8) & 0xFFu);
	i2c_tx_len                   = crc_covered + 2u;
	i2c_tx_cursor                = 0u;
}

/* --------------------------------------------------------------- */
/* ISR hooks (testable; HW-side wiring lives in the GigaDevice HAL)  */
/* --------------------------------------------------------------- */

/* Call on START + addressed-write: resets the RX staging buffer. */
void i2c_slave_write_start(void)
{
	i2c_rx_len          = 0u;
	pending_reply_valid = false;
}

/* Call on a bus-error resync (BRIDGE_I2C_ER_HANDLER's bus_error arm):
 * drops the tx-side staging alongside i2c_slave_write_start()'s rx-side
 * reset.  Without this, a bus error mid-reply-transmission left
 * i2c_tx_buf/i2c_tx_len/i2c_tx_cursor untouched, so a retried read
 * resumed from the half-consumed cursor (replaying the tail of the old
 * reply) instead of getting a clean, freshly-decided answer.  Setting
 * i2c_tx_cursor == i2c_tx_len (both zeroed) makes the next
 * i2c_slave_tx_next_byte() call land on the same "already fully
 * drained" path i2c_slave_write_end()'s once-only guard now stages
 * NO_PENDING from. */
void i2c_slave_tx_abort(void)
{
	i2c_tx_len    = 0u;
	i2c_tx_cursor = 0u;
}

/* Call per received byte during the write phase. */
void i2c_slave_rx_byte(uint8_t b)
{
	if (i2c_rx_len < sizeof(i2c_rx_buf)) {
		i2c_rx_buf[i2c_rx_len++] = b;
	}
	/* Else: overrun.  The CRC validation at end-of-write will fail
     * because the trailing bytes never landed. */
}

/* Call on STOP or repeated-START after a write transaction: validates
 * the buffered request and stages the reply.  Returns true on success
 * (reply staged), false on framing/CRC failure (NO_PENDING sentinel
 * staged in its place so the next read still gets a valid envelope). */
bool i2c_slave_write_end(void)
{
	if (pending_reply_valid) {
		/* Already decoded + dispatched for this addressed write.  The
	     * EV ISR calls this function unconditionally from both the
	     * repeated-START read arm and the STPDET arm (see the module
	     * banner), so a second or third entry against the same
	     * untouched i2c_rx_buf must be a no-op rather than a re-decode
	     * -- pending_reply_valid is cleared only in
	     * i2c_slave_write_start(), so it is false exactly once per
	     * addressed write and true for every repeat entry within the
	     * same transaction.  Falling through instead would re-run
	     * protocol_dispatch() on identical bytes (non-idempotent
	     * opcodes execute twice, ring-consuming opcodes drop a batch,
	     * OTA erase/program steps replay) and, on the framing-failure
	     * path, would call stage_no_pending() and clobber the reply
	     * already staged for this write before the host clocks it out.
	     *
	     * One exception: if the staged reply has ALREADY been fully
	     * clocked out (i2c_tx_cursor caught up with i2c_tx_len) by the
	     * time this repeat call lands, re-stage NO_PENDING instead of a
	     * bare no-op.  Two call sites land here with that true: the
	     * STOP-time tail call after a read that finished draining (no
	     * further byte ever reads tx_buf this transaction either way,
	     * so re-staging is unobservable there), and a genuinely NEW,
	     * separate read transaction with no intervening write -- which
	     * is the case that matters, since without this a bare repeated
	     * read after the reply drained would return the idle 0xFF
	     * pattern forever instead of the documented STATUS_NO_PENDING
	     * sentinel.  A reply that has only been PARTIALLY drained is
	     * left alone (falls through to the plain no-op below) -- an
	     * early-STOP mid-read is not the case this guards. */
		if (i2c_tx_cursor >= i2c_tx_len) {
			stage_no_pending();
		}
		return true;
	}

	/* Smallest valid envelope: reg(1) + cmd(1) + 0-byte payload + crc(2). */
	if (i2c_rx_len < 4u || i2c_rx_buf[0] != GD32_BRIDGE_I2C_REG_CMD) {
		stage_no_pending();
		return false;
	}
	const size_t   payload_len = i2c_rx_len - 4u; /* reg + cmd + ... + crc(2) */
	const uint16_t got_crc =
	    (uint16_t)i2c_rx_buf[2u + payload_len] | (uint16_t)i2c_rx_buf[2u + payload_len + 1u] << 8;
	/* CRC covers CMD..PAYLOAD (not the leading reg byte). */
	const uint16_t expect_crc = crc16_ccitt_false(&i2c_rx_buf[1], 1u + payload_len);
	if (got_crc != expect_crc) {
		stage_no_pending();
		return false;
	}

	uint8_t                    reply_pl[GD32_BRIDGE_MAX_PAYLOAD_BYTES];
	size_t                     reply_pl_len = 0u;
	const gd32_bridge_status_t st = protocol_dispatch(i2c_rx_buf[1],
	                                                  payload_len > 0u ? &i2c_rx_buf[2] : NULL,
	                                                  payload_len,
	                                                  reply_pl,
	                                                  sizeof(reply_pl),
	                                                  &reply_pl_len);
	stage_reply((uint8_t)st, reply_pl, reply_pl_len);
	pending_reply_valid = true;
	return true;
}

/* Call on addressed-read: returns the next byte to clock out.  When
 * the reply is exhausted, returns 0xFF as an idle pattern. */
uint8_t i2c_slave_tx_next_byte(void)
{
	if (!pending_reply_valid && i2c_tx_cursor == 0u) {
		/* Read before any matching write since the last START. */
		stage_no_pending();
	}
	if (i2c_tx_cursor < i2c_tx_len) {
		return i2c_tx_buf[i2c_tx_cursor++];
	}
	return 0xFFu;
}

void transport_i2c_init(void)
{
	i2c_rx_len          = 0u;
	i2c_tx_len          = 0u;
	i2c_tx_cursor       = 0u;
	pending_reply_valid = false;
	/* I2C0 slave bring-up (PA15/PB9, addr GD32_BRIDGE_DEFAULT_I2C_ADDR)
     * lives in the gd32 HAL backend (hal/transport_hw_gd32.c); the stub
     * backend's weak no-op keeps this hardware-free for host tests.
     * Return value ignored here on purpose: at boot CK_APB1 is whatever
     * SystemInit already brought up (216 MHz on this board) before
     * main() runs, so the out-of-range refusal
     * (hal/transport_hw_gd32.c's i2c_timing_derive()) is not a live
     * boot-time concern the way it is on the Deep-sleep wake path
     * (hal/gd32/power.c, which DOES propagate it) -- and there is no
     * host link yet at boot to report a failure to. */
	(void)bridge_transport_i2c_hw_init();
}
