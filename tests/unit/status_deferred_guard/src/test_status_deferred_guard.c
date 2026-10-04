/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * STATUS_DEFERRED (0xFE) is the dispatcher's internal "the reply comes later"
 * marker.  It must never reach the wire: both transports turn it into a
 * CRC-valid STATUS_IO with an empty payload at the single staging site.
 *
 * Neither real dispatcher path can produce it where it would be staged (the SPI
 * decode path returns before staging a DEFERRED, and the I2C link never defers),
 * so the guard is only reachable through a test seam: this suite links the REAL
 * src/transport_spi.c and src/transport_i2c.c against a fake protocol layer that
 * returns 0xFE on demand and hands back the deferred-reply sink the SPI
 * transport registered.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "protocol.h"
#include "transport.h"

/* ---- fake protocol layer --------------------------------------------------- */

static gd32_bridge_status_t       fake_status;
static protocol_deferred_reply_fn captured_sink;

uint16_t crc16_ccitt_false(const uint8_t *buf, size_t len)
{
	uint16_t crc = 0xFFFFu;
	for (size_t i = 0u; i < len; ++i) {
		crc ^= (uint16_t)((uint16_t)buf[i] << 8);
		for (unsigned b = 0u; b < 8u; ++b) {
			crc = (uint16_t)((crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
			                                 : (uint16_t)(crc << 1));
		}
	}
	return crc;
}

uint8_t protocol_link_features(gd32_bridge_link_t link)
{
	(void)link;
	return 0u;
}

void protocol_deferred_attach(protocol_deferred_reply_fn sink)
{
	captured_sink = sink;
}

void protocol_deferred_abort(void)
{
}

gd32_bridge_status_t protocol_dispatch(gd32_bridge_link_t link,
                                       uint8_t            cmd,
                                       const uint8_t     *req_payload,
                                       size_t             req_payload_len,
                                       uint8_t           *reply_payload,
                                       size_t             reply_payload_cap,
                                       size_t            *reply_payload_len)
{
	(void)link;
	(void)cmd;
	(void)req_payload;
	(void)req_payload_len;
	(void)reply_payload_cap;
	/* A non-empty payload alongside the marker: the guard must drop it too. */
	reply_payload[0]   = 0xAAu;
	reply_payload[1]   = 0xBBu;
	reply_payload[2]   = 0xCCu;
	*reply_payload_len = 3u;
	return fake_status;
}

/* ---- SPI -------------------------------------------------------------------- */

#define SPI_FRAME_CAP (1u + 1u + GD32_BRIDGE_MAX_PAYLOAD_BYTES + 2u)

static size_t spi_take_reply(uint8_t *out)
{
	size_t n = 0u;
	while (spi_slave_tx_pending() && n < SPI_FRAME_CAP) {
		out[n++] = spi_slave_tx_next_byte();
	}
	return n;
}

ZTEST(status_deferred_guard, test_spi_never_stages_deferred_on_the_wire)
{
	uint8_t        f[SPI_FRAME_CAP];
	const uint8_t  body[2] = { GD32_BRIDGE_SOF, CMD_ADC_READ };
	const uint16_t crc     = crc16_ccitt_false(body, 2u);

	transport_spi_init();
	zassert_true(captured_sink != NULL, "the SPI transport registered its deferred sink");

	/* A request the dispatcher defers: pending, nothing staged. */
	fake_status = STATUS_DEFERRED;
	spi_slave_cs_low();
	spi_slave_rx_byte(body[0]);
	spi_slave_rx_byte(body[1]);
	spi_slave_rx_byte((uint8_t)(crc & 0xFFu));
	spi_slave_rx_byte((uint8_t)(crc >> 8));
	spi_slave_cs_high();
	zassert_false(spi_slave_tx_pending(), "decode path stages nothing for a deferred command");

	/* The completion path is handed the marker (and a payload) by mistake. */
	const uint8_t pl[3] = { 1u, 2u, 3u };
	captured_sink(STATUS_DEFERRED, pl, sizeof pl);

	const size_t n = spi_take_reply(f);
	zassert_equal(n, 4u, "empty-payload envelope: SOF STATUS CRC(2)");
	zassert_equal(f[0], GD32_BRIDGE_SOF, "SOF");
	zassert_equal(f[1], STATUS_IO, "0xFE became STATUS_IO");
	const uint16_t got = (uint16_t)f[2] | (uint16_t)((uint16_t)f[3] << 8);
	zassert_equal(got, crc16_ccitt_false(f, 2u), "CRC valid");
}

/* ---- I2C -------------------------------------------------------------------- */

ZTEST(status_deferred_guard, test_i2c_never_stages_deferred_on_the_wire)
{
	const uint8_t  cmd = CMD_PING;
	const uint16_t crc = crc16_ccitt_false(&cmd, 1u);

	transport_i2c_init();
	fake_status = STATUS_DEFERRED;

	i2c_slave_write_start();
	i2c_slave_rx_byte(GD32_BRIDGE_I2C_REG_CMD);
	i2c_slave_rx_byte(cmd);
	i2c_slave_rx_byte((uint8_t)(crc & 0xFFu));
	i2c_slave_rx_byte((uint8_t)(crc >> 8));
	zassert_true(i2c_slave_write_end(), "request dispatched");

	uint8_t f[3];
	for (unsigned i = 0u; i < sizeof f; ++i) {
		f[i] = i2c_slave_tx_next_byte();
	}
	zassert_equal(f[0], STATUS_IO, "0xFE became STATUS_IO");
	const uint16_t got = (uint16_t)f[1] | (uint16_t)((uint16_t)f[2] << 8);
	zassert_equal(got, crc16_ccitt_false(f, 1u), "CRC valid over STATUS alone: empty payload");
	zassert_equal(i2c_slave_tx_next_byte(), 0xFFu, "and nothing follows the 3-byte envelope");
}

ZTEST_SUITE(status_deferred_guard, NULL, NULL, NULL, NULL, NULL);
