/* SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the gd32-bridge framing CRC-16 (src/protocol.c's
 * crc16_ccitt_false(), #26 / #104).
 *
 * crc16_ccitt_false() is on the FRAMING path -- both transports call it
 * TWICE per round trip (verify the request, compute the reply), inside
 * the SPI CS EXTI handler / the I2C ISR.  A wrong table entry means every
 * frame's CRC either always fails (link down) or, worse, some frames
 * pass with a corrupted payload.  This suite pins the table-driven
 * implementation bit-identical to a REFERENCE bit-serial implementation
 * kept local to this file -- a byte-for-byte copy of what src/protocol.c
 * contained before the table-driven rewrite, so the comparison is
 * against the ORIGINAL behaviour, not merely against itself.
 *
 * Coverage: the standard CRC-16/CCITT-FALSE check value for "123456789"
 * (0x29B1), all 256 single-byte inputs, the empty input, and a set of
 * multi-byte vectors.  crc16_ccitt_false() takes no seed parameter (it
 * always starts fresh at 0xFFFF), so unlike crc32's suite there is no
 * streaming/seed contract to pin here.
 *
 * Links src/protocol.c itself (not a standalone crc16.c/.h): this
 * function deliberately stays IN protocol.c -- see the comment ahead of
 * crc16_table[] there -- so this suite pulls in protocol.c's other
 * dependencies (bootloader.c, ota.c, crc32.c, the stub HAL) the same way
 * tests/unit/transport_spi and tests/unit/transport_i2c do.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "protocol.h"

/* ---- reference: the bit-serial implementation protocol.c used to be ----
 *
 * Deliberately NOT shared code with src/protocol.c: an independent
 * implementation of the same algorithm is what makes the comparison
 * below a real equivalence proof rather than a tautology.
 */
static uint16_t crc16_ref(const uint8_t *buf, size_t len)
{
	uint16_t crc = 0xFFFFu;
	for (size_t i = 0; i < len; ++i) {
		crc ^= (uint16_t)buf[i] << 8;
		for (unsigned b = 0; b < 8; ++b) {
			if (crc & 0x8000u) {
				crc = (uint16_t)((crc << 1) ^ 0x1021u);
			} else {
				crc <<= 1;
			}
		}
	}
	return crc;
}

ZTEST_SUITE(crc16, NULL, NULL, NULL, NULL, NULL);

/* The standard CRC-16/CCITT-FALSE check value for "123456789". */
ZTEST(crc16, test_check_value)
{
	const uint8_t *v = (const uint8_t *)"123456789";

	zassert_equal(crc16_ref(v, 9u), 0x29B1u, "reference impl must be standard CRC-16/CCITT-FALSE");
	zassert_equal(crc16_ccitt_false(v, 9u), 0x29B1u, "table-driven impl must match the standard");
}

/* Empty input: both forms must return the init value 0xFFFF un-mutated
 * (this CRC has no post-invert, unlike CRC-32). */
ZTEST(crc16, test_empty_input)
{
	zassert_equal(crc16_ccitt_false(NULL, 0u), 0xFFFFu);
	zassert_equal(crc16_ref(NULL, 0u), 0xFFFFu);
}

/* Exhaustive: every possible single-byte input, table-driven vs reference. */
ZTEST(crc16, test_all_256_single_byte_inputs)
{
	for (int n = 0; n < 256; n++) {
		uint8_t        b   = (uint8_t)n;
		const uint16_t ref = crc16_ref(&b, 1u);
		const uint16_t got = crc16_ccitt_false(&b, 1u);
		zassert_equal(got, ref, "byte 0x%02x: table=0x%04x ref=0x%04x", n, got, ref);
	}
}

/* A handful of multi-byte vectors, including lengths that don't divide
 * evenly and a run of identical bytes (exercises repeated table
 * indices, which an off-by-one in the table itself would not catch
 * from single-byte cases alone). */
ZTEST(crc16, test_multi_byte_vectors)
{
	static const struct {
		const char *data;
		size_t      len;
	} vecs[] = {
		{ "A", 1u },
		{ "Alp Lab", 7u },
		{ "The quick brown fox jumps over the lazy dog", 44u },
		{ "\x00\x00\x00\x00\x00\x00\x00\x00", 8u },
		{ "\xff\xff\xff\xff\xff\xff\xff\xff", 8u },
	};

	for (size_t i = 0u; i < sizeof(vecs) / sizeof(vecs[0]); i++) {
		const uint8_t *d   = (const uint8_t *)vecs[i].data;
		const uint16_t ref = crc16_ref(d, vecs[i].len);
		const uint16_t got = crc16_ccitt_false(d, vecs[i].len);
		zassert_equal(
		    got, ref, "vector %zu (len %zu): table=0x%04x ref=0x%04x", i, vecs[i].len, got, ref);
	}
}
