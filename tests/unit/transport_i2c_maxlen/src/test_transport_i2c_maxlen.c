/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * The I2C transport's staging buffers hold a full 65-byte reply (status +
 * 65 + CRC).  v0.15's I2C opcode allow-list means no real opcode produces a
 * reply that long on I2C any more, so the old end-to-end case (an
 * ADC_STREAM_READ over I2C) cannot reach it through protocol.c.  This suite
 * links src/transport_i2c.c against a stand-in dispatcher that fabricates a
 * maximum-length reply, to keep that buffer-capacity property pinned.
 */

#include <string.h>
#include <zephyr/ztest.h>

#include "protocol.h"
#include "transport.h"

/* ---- stand-ins for the symbols transport_i2c.c pulls from protocol.c ---- */

uint16_t crc16_ccitt_false(const uint8_t *buf, size_t len)
{
	uint16_t crc = 0xFFFFu; /* bit-serial reference, independent of protocol.c */
	for (size_t i = 0u; i < len; i++) {
		crc ^= (uint16_t)((uint16_t)buf[i] << 8);
		for (int b = 0; b < 8; b++) {
			crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
		}
	}
	return crc;
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
	if (reply_payload_cap < GD32_BRIDGE_MAX_PAYLOAD_BYTES) return STATUS_NOMEM;
	for (size_t i = 0u; i < GD32_BRIDGE_MAX_PAYLOAD_BYTES; i++) {
		reply_payload[i] = (uint8_t)(0x40u + i);
	}
	*reply_payload_len = GD32_BRIDGE_MAX_PAYLOAD_BYTES;
	return STATUS_OK;
}

ZTEST_SUITE(gd32_bridge_transport_i2c_maxlen, NULL, NULL, NULL, NULL, NULL);

ZTEST(gd32_bridge_transport_i2c_maxlen, test_max_length_reply_is_staged_intact)
{
	uint8_t req[4];
	uint8_t reply[1u + GD32_BRIDGE_MAX_PAYLOAD_BYTES + 2u];

	transport_i2c_init();

	req[0]             = GD32_BRIDGE_I2C_REG_CMD;
	req[1]             = CMD_PING;
	const uint16_t crc = crc16_ccitt_false(&req[1], 1u);
	req[2]             = (uint8_t)(crc & 0xFFu);
	req[3]             = (uint8_t)(crc >> 8);

	i2c_slave_write_start();
	for (size_t i = 0u; i < sizeof(req); i++) {
		i2c_slave_rx_byte(req[i]);
	}
	zassert_true(i2c_slave_write_end(), "request dispatches and stages");
	for (size_t i = 0u; i < sizeof(reply); i++) {
		reply[i] = i2c_slave_tx_next_byte();
	}

	zassert_equal(reply[0], STATUS_OK);
	for (size_t i = 0u; i < GD32_BRIDGE_MAX_PAYLOAD_BYTES; i++) {
		zassert_equal(reply[1u + i], (uint8_t)(0x40u + i), "payload byte %u", (unsigned)i);
	}
	const size_t   crc_covered = 1u + GD32_BRIDGE_MAX_PAYLOAD_BYTES;
	const uint16_t rcrc        = crc16_ccitt_false(reply, crc_covered);
	zassert_equal(reply[crc_covered], (uint8_t)(rcrc & 0xFFu));
	zassert_equal(reply[crc_covered + 1u], (uint8_t)(rcrc >> 8));
}
