/* SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for the gd32-bridge CRC-32 (src/crc32.c, #26 C2 / #104).
 *
 * ota_crc32() is on the OTA image-acceptance path: a wrong table
 * silently accepts a corrupt image or rejects a good one, on a path
 * with no host-driven SWD recovery.  This suite pins the table-driven
 * implementation bit-identical to a REFERENCE bit-serial implementation
 * kept local to this file -- a byte-for-byte copy of what src/crc32.c
 * contained before the table-driven rewrite, so the comparison is
 * against the ORIGINAL behaviour, not merely against itself.
 *
 * Coverage: the standard CRC-32 check value for "123456789"
 * (0xCBF43926), all 256 single-byte inputs, the empty input, a set of
 * multi-byte vectors, and a streaming/chunked case that chains the
 * `crc` seed the way src/ota.c's incremental use depends on -- proving
 * that contract still holds under the table-driven rewrite.
 */

#include <string.h>

#include <zephyr/ztest.h>

#include "crc32.h"

/* ---- reference: the bit-serial implementation crc32.c used to be ----
 *
 * Deliberately NOT shared code with src/crc32.c: an independent
 * implementation of the same algorithm is what makes the comparison
 * below a real equivalence proof rather than a tautology.
 */
static uint32_t crc32_ref(uint32_t crc, const uint8_t *data, size_t len)
{
	crc ^= 0xFFFFFFFFu;
	for (size_t i = 0u; i < len; i++) {
		crc ^= data[i];
		for (int b = 0; b < 8; b++) {
			crc = (crc >> 1) ^ (0xEDB88320u & (uint32_t)(-(int32_t)(crc & 1u)));
		}
	}
	return crc ^ 0xFFFFFFFFu;
}

ZTEST_SUITE(crc32, NULL, NULL, NULL, NULL, NULL);

/* The standard CRC-32/ISO-HDLC check value for "123456789". */
ZTEST(crc32, test_check_value)
{
	const uint8_t *v = (const uint8_t *)"123456789";

	zassert_equal(crc32_ref(0u, v, 9u), 0xCBF43926u, "reference impl must be standard CRC-32");
	zassert_equal(ota_crc32(0u, v, 9u), 0xCBF43926u, "table-driven impl must match the standard");
}

/* Empty input: both forms must return the seed itself un-mutated (the
 * pre/post inversion is self-cancelling with nothing in between). */
ZTEST(crc32, test_empty_input)
{
	zassert_equal(ota_crc32(0u, NULL, 0u), 0u);
	zassert_equal(ota_crc32(0xDEADBEEFu, NULL, 0u), 0xDEADBEEFu);
	zassert_equal(crc32_ref(0xDEADBEEFu, NULL, 0u), 0xDEADBEEFu);
}

/* Exhaustive: every possible single-byte input, table-driven vs reference. */
ZTEST(crc32, test_all_256_single_byte_inputs)
{
	for (int n = 0; n < 256; n++) {
		uint8_t        b   = (uint8_t)n;
		const uint32_t ref = crc32_ref(0u, &b, 1u);
		const uint32_t got = ota_crc32(0u, &b, 1u);
		zassert_equal(got, ref, "byte 0x%02x: table=0x%08x ref=0x%08x", n, got, ref);
	}
}

/* A handful of multi-byte vectors, including lengths that don't divide
 * evenly and a run of identical bytes (exercises repeated table
 * indices, which an off-by-one in the table itself would not catch
 * from single-byte cases alone). */
ZTEST(crc32, test_multi_byte_vectors)
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
		const uint32_t ref = crc32_ref(0u, d, vecs[i].len);
		const uint32_t got = ota_crc32(0u, d, vecs[i].len);
		zassert_equal(
		    got, ref, "vector %zu (len %zu): table=0x%08x ref=0x%08x", i, vecs[i].len, got, ref);
	}
}

/* Streaming/chunked: src/ota.c's h_write composes a running CRC by
 * feeding the previous call's return value back in as the next call's
 * seed.  Split "123456789" into three unevenly-sized chunks fed through
 * SEPARATE calls and confirm the chained result still equals the
 * single-shot check value -- proves the seed contract survives the
 * table-driven rewrite, at both odd chunk boundaries and a
 * single-byte-at-a-time extreme. */
ZTEST(crc32, test_streaming_seed_contract)
{
	const uint8_t *v = (const uint8_t *)"123456789";

	/* Chunked 3+3+3. */
	uint32_t crc = 0u;
	crc          = ota_crc32(crc, &v[0], 3u);
	crc          = ota_crc32(crc, &v[3], 3u);
	crc          = ota_crc32(crc, &v[6], 3u);
	zassert_equal(crc, 0xCBF43926u, "chunked 3+3+3 must match the single-shot check value");

	/* Chunked 1+8. */
	crc = ota_crc32(0u, &v[0], 1u);
	crc = ota_crc32(crc, &v[1], 8u);
	zassert_equal(crc, 0xCBF43926u, "chunked 1+8 must match the single-shot check value");

	/* One byte at a time -- the extreme h_write could in principle hit. */
	crc = 0u;
	for (size_t i = 0u; i < 9u; i++) {
		crc = ota_crc32(crc, &v[i], 1u);
	}
	zassert_equal(
	    crc, 0xCBF43926u, "byte-at-a-time streaming must match the single-shot check value");

	/* Chunked must also match crc32_ref chunked the same way (not just
     * the literal check-value constant), so this isn't pinned to one
     * magic number. */
	uint32_t ref_crc   = 0u;
	uint32_t table_crc = 0u;
	ref_crc            = crc32_ref(ref_crc, &v[0], 4u);
	table_crc          = ota_crc32(table_crc, &v[0], 4u);
	ref_crc            = crc32_ref(ref_crc, &v[4], 5u);
	table_crc          = ota_crc32(table_crc, &v[4], 5u);
	zassert_equal(table_crc,
	              ref_crc,
	              "chunked 4+5 vs reference: table=0x%08x ref=0x%08x",
	              table_crc,
	              ref_crc);
}
