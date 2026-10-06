/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge firmware: shared command-handler table.
 *
 * Per the project memory rule "don't fork the protocol -- one framing
 * format, one command set, one set of reply codes; only the transport
 * layer differs", both the SPI and I2C transports call into this
 * file's protocol_dispatch().
 *
 * Handlers that need actual hardware (PWM channel programming, ADC
 * sampling, GPIO output, DA9292 INT/TW pin sampling) call into the
 * `bridge_hw_*` HAL shims declared in hal/bridge_hw.h.  The HAL is a
 * separate compile unit; which implementation links is the build-time
 * BRIDGE_HAL_BACKEND choice (CMakeLists.txt):
 *
 *   stub -- hal/bridge_hw_stub.c: every shim returns
 *           BRIDGE_HW_ERR_NOTIMPL, which status_from_hw() below maps to
 *           STATUS_NOSUPPORT on the wire, so the protocol round-trip is
 *           exercisable host-side with no silicon.
 *   gd32 -- the per-peripheral translation units under hal/gd32/,
 *           implemented against the GigaDevice firmware library.
 *
 * PING + GET_VERSION + GET_BUILD_ID answer identically under both:
 * they need no peripheral I/O.
 */

#include <stdatomic.h>
#include <string.h>

#include "protocol.h"
#include "../hal/bridge_hw.h"
#include "boot_config.h"
#include "bootloader/bootloader.h"
#include "ota.h"

/* --------------------------------------------------------------- */
/* CRC-16 / CCITT-FALSE -- shared with transports.                   */
/*                                                                    */
/* Table-driven (#26, #104): this is the FRAMING CRC -- both          */
/* transports call it TWICE per round trip (verify the request,      */
/* compute the reply), on the path inside the SPI CS EXTI handler /   */
/* the I2C ISR.  The previous bit-serial form cost 8 shift-and-branch */
/* iterations per byte.  crc16_table[] below is the standard Sarwate  */
/* byte-wise table, adapted for this CRC's non-reflected / MSB-first  */
/* shape (poly 0x1021, init 0xFFFF, xor-out 0x0000) rather than       */
/* copied from CRC-32's reflected construction (see crc32.c): the     */
/* incoming byte combines with the crc's TOP byte, and the table is   */
/* indexed by `(crc >> 8) ^ byte`.  Measured (arm-none-eabi-gcc       */
/* 13.3.1, -mcpu=cortex-m33 -mthumb, disassembly of just this         */
/* function): table-driven crc16_ccitt_false() is ~8.4x fewer         */
/* instructions executed per byte at -Os than the bit-serial form it  */
/* replaces (9 vs 76 -- PR #115's CRC-32 table win was ~5.75x, smaller */
/* because that inner loop carries less per-bit overhead), at a cost  */
/* of 512 B of .rodata.                                               */
/*                                                                    */
/* The table is GENERATED, not hand-typed: tools/gen_crc16_table.py   */
/* produces the exact block between the clang-format fence below from */
/* the polynomial alone, and its --check mode proves this checked-in  */
/* copy still matches that output --                                  */
/*     python3 tools/gen_crc16_table.py --check src/protocol.c        */
/* -- so nobody has to eyeball 256 magic numbers and trust them.      */
/* tools/test_gen_crc16_table.py runs that check under `python3 -m    */
/* compileall`'s neighbourhood (byte-compiled by the python-lint CI   */
/* job) and additionally as its own assertion.                        */
/*                                                                    */
/* Bit-identical to the bit-serial routine it replaces, proved         */
/* exhaustively by tests/unit/crc16/src/test_crc16.c: the standard    */
/* CRC-16/CCITT-FALSE check value 0x29B1 for "123456789", all 256     */
/* single-byte inputs, and a set of multi-byte vectors -- against a   */
/* reference implementation kept local to that test file, independent */
/* of this one.  tests/protocol_vectors.txt (regenerated + diffed in  */
/* CI) is the end-to-end proof: every pinned frame carries a real CRC */
/* computed by tests/gen_protocol_vectors.py's own bit-serial Python  */
/* copy, so a wrong table entry would show up there too.              */
/*                                                                    */
/* Kept IN this file rather than split into its own crc16.c/.h        */
/* (unlike ota_crc32, which lives in crc32.c): alp-sdk's frame-fuzz   */
/* harness (tests/fuzz/gd32_bridge_frame_fuzz.c, see this repo's      */
/* README.md) links crc16_ccitt_false by compiling src/protocol.c     */
/* directly, by hardcoded path, from a sibling repo this change does  */
/* not touch -- moving the symbol out of protocol.c would silently    */
/* break that build.                                                  */
/*                                                                    */
/* Checked in rather than computed at first use, for the same reason  */
/* as crc32_table: this is ROM-resident .rodata with no boot-time     */
/* budget to spend building a table before the first frame arrives.   */
/*                                                                    */
/* (The fence-post survives clang-format only because of the          */
/* "clang-format off/on" markers -- see crc32.c for why.)             */
/* --------------------------------------------------------------- */

/* clang-format off */
static const uint16_t crc16_table[256] = {
	0x0000u, 0x1021u, 0x2042u, 0x3063u, 0x4084u, 0x50A5u, 0x60C6u, 0x70E7u,
	0x8108u, 0x9129u, 0xA14Au, 0xB16Bu, 0xC18Cu, 0xD1ADu, 0xE1CEu, 0xF1EFu,
	0x1231u, 0x0210u, 0x3273u, 0x2252u, 0x52B5u, 0x4294u, 0x72F7u, 0x62D6u,
	0x9339u, 0x8318u, 0xB37Bu, 0xA35Au, 0xD3BDu, 0xC39Cu, 0xF3FFu, 0xE3DEu,
	0x2462u, 0x3443u, 0x0420u, 0x1401u, 0x64E6u, 0x74C7u, 0x44A4u, 0x5485u,
	0xA56Au, 0xB54Bu, 0x8528u, 0x9509u, 0xE5EEu, 0xF5CFu, 0xC5ACu, 0xD58Du,
	0x3653u, 0x2672u, 0x1611u, 0x0630u, 0x76D7u, 0x66F6u, 0x5695u, 0x46B4u,
	0xB75Bu, 0xA77Au, 0x9719u, 0x8738u, 0xF7DFu, 0xE7FEu, 0xD79Du, 0xC7BCu,
	0x48C4u, 0x58E5u, 0x6886u, 0x78A7u, 0x0840u, 0x1861u, 0x2802u, 0x3823u,
	0xC9CCu, 0xD9EDu, 0xE98Eu, 0xF9AFu, 0x8948u, 0x9969u, 0xA90Au, 0xB92Bu,
	0x5AF5u, 0x4AD4u, 0x7AB7u, 0x6A96u, 0x1A71u, 0x0A50u, 0x3A33u, 0x2A12u,
	0xDBFDu, 0xCBDCu, 0xFBBFu, 0xEB9Eu, 0x9B79u, 0x8B58u, 0xBB3Bu, 0xAB1Au,
	0x6CA6u, 0x7C87u, 0x4CE4u, 0x5CC5u, 0x2C22u, 0x3C03u, 0x0C60u, 0x1C41u,
	0xEDAEu, 0xFD8Fu, 0xCDECu, 0xDDCDu, 0xAD2Au, 0xBD0Bu, 0x8D68u, 0x9D49u,
	0x7E97u, 0x6EB6u, 0x5ED5u, 0x4EF4u, 0x3E13u, 0x2E32u, 0x1E51u, 0x0E70u,
	0xFF9Fu, 0xEFBEu, 0xDFDDu, 0xCFFCu, 0xBF1Bu, 0xAF3Au, 0x9F59u, 0x8F78u,
	0x9188u, 0x81A9u, 0xB1CAu, 0xA1EBu, 0xD10Cu, 0xC12Du, 0xF14Eu, 0xE16Fu,
	0x1080u, 0x00A1u, 0x30C2u, 0x20E3u, 0x5004u, 0x4025u, 0x7046u, 0x6067u,
	0x83B9u, 0x9398u, 0xA3FBu, 0xB3DAu, 0xC33Du, 0xD31Cu, 0xE37Fu, 0xF35Eu,
	0x02B1u, 0x1290u, 0x22F3u, 0x32D2u, 0x4235u, 0x5214u, 0x6277u, 0x7256u,
	0xB5EAu, 0xA5CBu, 0x95A8u, 0x8589u, 0xF56Eu, 0xE54Fu, 0xD52Cu, 0xC50Du,
	0x34E2u, 0x24C3u, 0x14A0u, 0x0481u, 0x7466u, 0x6447u, 0x5424u, 0x4405u,
	0xA7DBu, 0xB7FAu, 0x8799u, 0x97B8u, 0xE75Fu, 0xF77Eu, 0xC71Du, 0xD73Cu,
	0x26D3u, 0x36F2u, 0x0691u, 0x16B0u, 0x6657u, 0x7676u, 0x4615u, 0x5634u,
	0xD94Cu, 0xC96Du, 0xF90Eu, 0xE92Fu, 0x99C8u, 0x89E9u, 0xB98Au, 0xA9ABu,
	0x5844u, 0x4865u, 0x7806u, 0x6827u, 0x18C0u, 0x08E1u, 0x3882u, 0x28A3u,
	0xCB7Du, 0xDB5Cu, 0xEB3Fu, 0xFB1Eu, 0x8BF9u, 0x9BD8u, 0xABBBu, 0xBB9Au,
	0x4A75u, 0x5A54u, 0x6A37u, 0x7A16u, 0x0AF1u, 0x1AD0u, 0x2AB3u, 0x3A92u,
	0xFD2Eu, 0xED0Fu, 0xDD6Cu, 0xCD4Du, 0xBDAAu, 0xAD8Bu, 0x9DE8u, 0x8DC9u,
	0x7C26u, 0x6C07u, 0x5C64u, 0x4C45u, 0x3CA2u, 0x2C83u, 0x1CE0u, 0x0CC1u,
	0xEF1Fu, 0xFF3Eu, 0xCF5Du, 0xDF7Cu, 0xAF9Bu, 0xBFBAu, 0x8FD9u, 0x9FF8u,
	0x6E17u, 0x7E36u, 0x4E55u, 0x5E74u, 0x2E93u, 0x3EB2u, 0x0ED1u, 0x1EF0u,
};
/* clang-format on */

uint16_t crc16_ccitt_false(const uint8_t *buf, size_t len)
{
	uint16_t crc = 0xFFFFu;
	for (size_t i = 0; i < len; ++i) {
		crc = (uint16_t)((crc << 8) ^ crc16_table[((crc >> 8) ^ buf[i]) & 0xFFu]);
	}
	return crc;
}

/* --------------------------------------------------------------- */
/* Build identifier: "<fw-version>+<git-sha-prefix>", 20 ASCII chars. */
/* Baked at build time into the generated gd32_bridge_build_id.h by  */
/* cmake/gen_build_id.cmake (re-run every build, so HEAD moves and   */
/* version bumps land in the next incremental build).  Non-CMake     */
/* consumers (the SDK-side transport unit tests compile this file    */
/* directly) keep the hermetic placeholder below.                     */
/* --------------------------------------------------------------- */

#ifdef GD32_BRIDGE_HAVE_BUILD_ID_HEADER
#include "gd32_bridge_build_id.h"
#endif

#ifndef GD32_BRIDGE_BUILD_ID
#define GD32_BRIDGE_BUILD_ID "0000000000000000abcd"
#endif

_Static_assert(sizeof(GD32_BRIDGE_BUILD_ID) - 1u == GD32_BRIDGE_BUILD_ID_LEN,
               "build id must be 20 ASCII bytes");

/* --------------------------------------------------------------- */
/* LE-int helpers (firmware side, parallel to the host driver).      */
/* --------------------------------------------------------------- */

static uint32_t get_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static uint16_t get_le16(const uint8_t *p)
{
	return (uint16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static void put_le16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
}

static void put_le32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFFu);
	p[1] = (uint8_t)((v >> 8) & 0xFFu);
	p[2] = (uint8_t)((v >> 16) & 0xFFu);
	p[3] = (uint8_t)((v >> 24) & 0xFFu);
}

/* --------------------------------------------------------------- */
/* Per-opcode handlers                                                */
/* --------------------------------------------------------------- */

/* Translate a BRIDGE_HW_ERR_* return into a STATUS_*.  Centralised so
 * every handler below reports the same code for the same HAL error --
 * forward-declared here (defined with the v0.5 handler set) so the
 * legacy handlers earlier in this file can route through it instead
 * of hand-rolling their own STATUS_IO-flattening ladder (#23). */
static gd32_bridge_status_t status_from_hw(int rv);

static gd32_bridge_status_t
handle_ping(const uint8_t *req, size_t req_len, uint8_t *reply, size_t reply_cap, size_t *reply_len)
{
	(void)req;
	(void)reply;
	(void)reply_cap;
	if (req_len != 0u) return STATUS_INVAL;
	*reply_len = 0u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_get_version(const uint8_t *req,
                                               size_t         req_len,
                                               uint8_t       *reply,
                                               size_t         reply_cap,
                                               size_t        *reply_len)
{
	(void)req;
	if (req_len != 0u) return STATUS_INVAL;
	if (reply_cap < 3u) return STATUS_NOMEM;
	reply[0]   = PROTOCOL_VERSION_MAJOR;
	reply[1]   = PROTOCOL_VERSION_MINOR;
	reply[2]   = PROTOCOL_VERSION_PATCH;
	*reply_len = 3u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_get_build_id(const uint8_t *req,
                                                size_t         req_len,
                                                uint8_t       *reply,
                                                size_t         reply_cap,
                                                size_t        *reply_len)
{
	(void)req;
	if (req_len != 0u) return STATUS_INVAL;
	if (reply_cap < GD32_BRIDGE_BUILD_ID_LEN) return STATUS_NOMEM;
	memcpy(reply, GD32_BRIDGE_BUILD_ID, GD32_BRIDGE_BUILD_ID_LEN);
	*reply_len = GD32_BRIDGE_BUILD_ID_LEN;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_reset_reason(const uint8_t *req,
                                                size_t         req_len,
                                                uint8_t       *reply,
                                                size_t         reply_cap,
                                                size_t        *reply_len)
{
	(void)req;
	if (req_len != 0u) return STATUS_INVAL;
	if (reply_cap < 1u) return STATUS_NOMEM;
	/* The real HAL captures and clears sticky hardware flags during boot, so
     * this reply is repeatable for every reader in that boot. */
	reply[0]   = bridge_hw_reset_reason();
	*reply_len = 1u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_gpio_read(const uint8_t *req,
                                             size_t         req_len,
                                             uint8_t       *reply,
                                             size_t         reply_cap,
                                             size_t        *reply_len)
{
	if (req_len != 4u) return STATUS_INVAL;
	if (reply_cap < 4u) return STATUS_NOMEM;
	const uint32_t mask   = get_le32(req);
	uint32_t       levels = 0u;
	const int      rv     = bridge_hw_gpio_read(mask, &levels);
	if (rv != BRIDGE_HW_OK) return status_from_hw(rv);
	put_le32(reply, levels);
	*reply_len = 4u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_gpio_write(const uint8_t *req,
                                              size_t         req_len,
                                              uint8_t       *reply,
                                              size_t         reply_cap,
                                              size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 8u) return STATUS_INVAL;
	const uint32_t mask   = get_le32(&req[0]);
	const uint32_t levels = get_le32(&req[4]);
	const int      rv     = bridge_hw_gpio_write(mask, levels);
	if (rv != BRIDGE_HW_OK) return status_from_hw(rv);
	*reply_len = 0u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_pwm_set(const uint8_t *req,
                                           size_t         req_len,
                                           uint8_t       *reply,
                                           size_t         reply_cap,
                                           size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 10u) return STATUS_INVAL;
	const uint8_t channel = req[0];
	/* req[1] is the reserved padding byte; ignore */
	const uint32_t period_ns = get_le32(&req[2]);
	const uint32_t duty_ns   = get_le32(&req[6]);
	if (duty_ns > period_ns) return STATUS_INVAL;
	const int rv = bridge_hw_pwm_set(channel, period_ns, duty_ns);
	if (rv != BRIDGE_HW_OK) return status_from_hw(rv);
	*reply_len = 0u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_pwm_get(const uint8_t *req,
                                           size_t         req_len,
                                           uint8_t       *reply,
                                           size_t         reply_cap,
                                           size_t        *reply_len)
{
	if (req_len != 1u) return STATUS_INVAL;
	if (reply_cap < 8u) return STATUS_NOMEM;
	uint32_t  period_ns = 0u;
	uint32_t  duty_ns   = 0u;
	const int rv        = bridge_hw_pwm_get(req[0], &period_ns, &duty_ns);
	if (rv != BRIDGE_HW_OK) return status_from_hw(rv);
	put_le32(&reply[0], period_ns);
	put_le32(&reply[4], duty_ns);
	*reply_len = 8u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_adc_read(const uint8_t *req,
                                            size_t         req_len,
                                            uint8_t       *reply,
                                            size_t         reply_cap,
                                            size_t        *reply_len)
{
	if (req_len != 2u) return STATUS_INVAL;
	uint8_t channel = req[0];
	uint8_t samples = req[1];
	if (samples == 0u) return STATUS_INVAL;
	/* Reject out-of-range sample counts outright rather than silently
     * capping -- the host driver compares `reply[0]` against the
     * originally-requested count and treats a mismatch as a wire
     * error, so silent capping would turn a documented OUT_OF_RANGE
     * caller error into a misleading ALP_ERR_IO. */
	if (samples > GD32_BRIDGE_ADC_MAX_SAMPLES) return STATUS_OUT_OF_RANGE;

	const size_t need = 1u + (size_t)samples * 2u;
	if (reply_cap < need) return STATUS_NOMEM;

	reply[0] = samples; /* echoes back the (validated) value */
	uint16_t  mv[GD32_BRIDGE_ADC_MAX_SAMPLES];
	const int rv = bridge_hw_adc_read(channel, samples, mv);
	if (rv != BRIDGE_HW_OK) return status_from_hw(rv);
	for (uint8_t i = 0u; i < samples; ++i) {
		reply[1u + i * 2u]      = (uint8_t)(mv[i] & 0xFFu);
		reply[1u + i * 2u + 1u] = (uint8_t)((mv[i] >> 8) & 0xFFu);
	}
	*reply_len = need;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_da9292_forward(const uint8_t *req,
                                                  size_t         req_len,
                                                  uint8_t       *reply,
                                                  size_t         reply_cap,
                                                  size_t        *reply_len)
{
	(void)req;
	if (req_len != 0u) return STATUS_INVAL;
	if (reply_cap < 1u) return STATUS_NOMEM;
	reply[0]   = bridge_hw_da9292_status_cached();
	*reply_len = 1u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_se_reset(const uint8_t *req,
                                            size_t         req_len,
                                            uint8_t       *reply,
                                            size_t         reply_cap,
                                            size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 1u) return STATUS_INVAL;
	if (req[0] > 1u) return STATUS_INVAL; /* assert ∈ {0 = release, 1 = hold} */
	const int rv = bridge_hw_se_reset(req[0]);
	if (rv == BRIDGE_HW_ERR_INVAL) return STATUS_INVAL;
	if (rv == BRIDGE_HW_ERR_NOTIMPL) return STATUS_NOSUPPORT;
	if (rv < 0) return STATUS_IO;
	*reply_len = 0u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_boot_config(const uint8_t *req,
                                               size_t         req_len,
                                               uint8_t       *reply,
                                               size_t         reply_cap,
                                               size_t        *reply_len)
{
	if (req_len != 5u) return STATUS_INVAL;
	if (reply_cap < 4u) return STATUS_NOMEM;
	if (req[0] > 1u) return STATUS_INVAL;              /* op: 0 = GET, 1 = SET */
	if (!ota_fmc_supported()) return STATUS_NOSUPPORT; /* no FMC HAL = no config page */
	if (req[0] == 1u) {
		/* Never flash from here (transport ISR): queue the SET for the main
		 * loop (boot_config_tick) and reply with the CURRENT stored value.
		 * The host polls GET until it equals the request.  A SET equal to
		 * the stored value is accepted without queueing (no-op). */
		const uint32_t flags = get_le32(&req[1]);
		if ((flags & ~BOOT_CONFIG_KNOWN_FLAGS) != 0u) return STATUS_INVAL;
		if (flags != boot_config_flags() && !ota_fmc_config_write_safe()) return STATUS_NOSUPPORT;
		if (flags != boot_config_flags() && ota_session_active())
			return STATUS_BUSY;                              /* OTA owns the FMC */
		if (!boot_config_request(flags)) return STATUS_BUSY; /* a different SET is in flight */
	}
	put_le32(reply, boot_config_flags());
	*reply_len = 4u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_dac_set(const uint8_t *req,
                                           size_t         req_len,
                                           uint8_t       *reply,
                                           size_t         reply_cap,
                                           size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 4u) return STATUS_INVAL;
	const uint8_t channel = req[0];
	/* req[1] reserved padding -- ignore */
	const uint16_t value_mv = (uint16_t)req[2] | ((uint16_t)req[3] << 8);
	const int      rv       = bridge_hw_dac_set(channel, value_mv);
	if (rv == BRIDGE_HW_ERR_INVAL) return STATUS_INVAL;
	if (rv == BRIDGE_HW_ERR_RANGE) return STATUS_OUT_OF_RANGE;
	if (rv == BRIDGE_HW_ERR_NOTIMPL) return STATUS_NOSUPPORT;
	if (rv < 0) return STATUS_IO;
	*reply_len = 0u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_dac_get(const uint8_t *req,
                                           size_t         req_len,
                                           uint8_t       *reply,
                                           size_t         reply_cap,
                                           size_t        *reply_len)
{
	if (req_len != 1u) return STATUS_INVAL;
	if (reply_cap < 2u) return STATUS_NOMEM;
	uint16_t  value_mv = 0u;
	const int rv       = bridge_hw_dac_get(req[0], &value_mv);
	if (rv == BRIDGE_HW_ERR_INVAL) return STATUS_INVAL;
	if (rv == BRIDGE_HW_ERR_NOTIMPL) return STATUS_NOSUPPORT;
	if (rv < 0) return STATUS_IO;
	reply[0]   = (uint8_t)(value_mv & 0xFFu);
	reply[1]   = (uint8_t)((value_mv >> 8) & 0xFFu);
	*reply_len = 2u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_qenc_read(const uint8_t *req,
                                             size_t         req_len,
                                             uint8_t       *reply,
                                             size_t         reply_cap,
                                             size_t        *reply_len)
{
	if (req_len != 1u) return STATUS_INVAL;
	if (reply_cap < 4u) return STATUS_NOMEM;
	int32_t   position = 0;
	const int rv       = bridge_hw_qenc_read(req[0], &position);
	if (rv == BRIDGE_HW_ERR_INVAL) return STATUS_INVAL;
	if (rv == BRIDGE_HW_ERR_NOTIMPL) return STATUS_NOSUPPORT;
	if (rv < 0) return STATUS_IO;
	put_le32(reply, (uint32_t)position);
	*reply_len = 4u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_qenc_reset(const uint8_t *req,
                                              size_t         req_len,
                                              uint8_t       *reply,
                                              size_t         reply_cap,
                                              size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 1u) return STATUS_INVAL;
	const int rv = bridge_hw_qenc_reset(req[0]);
	if (rv == BRIDGE_HW_ERR_INVAL) return STATUS_INVAL;
	if (rv == BRIDGE_HW_ERR_NOTIMPL) return STATUS_NOSUPPORT;
	if (rv < 0) return STATUS_IO;
	*reply_len = 0u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_counter_read(const uint8_t *req,
                                                size_t         req_len,
                                                uint8_t       *reply,
                                                size_t         reply_cap,
                                                size_t        *reply_len)
{
	if (req_len != 1u) return STATUS_INVAL;
	if (reply_cap < 4u) return STATUS_NOMEM;
	uint32_t  ticks = 0u;
	const int rv    = bridge_hw_counter_read(req[0], &ticks);
	if (rv == BRIDGE_HW_ERR_INVAL) return STATUS_INVAL;
	if (rv == BRIDGE_HW_ERR_NOTIMPL) return STATUS_NOSUPPORT;
	if (rv < 0) return STATUS_IO;
	put_le32(reply, ticks);
	*reply_len = 4u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_pwm_configure(const uint8_t *req,
                                                 size_t         req_len,
                                                 uint8_t       *reply,
                                                 size_t         reply_cap,
                                                 size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 7u) return STATUS_INVAL;
	const uint8_t  channel      = req[0];
	const uint8_t  align_mode   = req[1];
	const uint32_t dead_time_ns = get_le32(&req[2]);
	const uint8_t  break_cfg    = req[6];
	if (align_mode > 3u) return STATUS_INVAL;
	const int rv = bridge_hw_pwm_configure(channel, align_mode, dead_time_ns, break_cfg);
	if (rv == BRIDGE_HW_ERR_INVAL) return STATUS_INVAL;
	if (rv == BRIDGE_HW_ERR_RANGE) return STATUS_OUT_OF_RANGE;
	if (rv == BRIDGE_HW_ERR_NOTIMPL) return STATUS_NOSUPPORT;
	if (rv < 0) return STATUS_IO;
	*reply_len = 0u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_adc_configure(const uint8_t *req,
                                                 size_t         req_len,
                                                 uint8_t       *reply,
                                                 size_t         reply_cap,
                                                 size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 7u) return STATUS_INVAL;
	const uint8_t channel = req[0];
	/* req[1] reserved padding */
	const uint16_t oversample_ratio = (uint16_t)req[2] | ((uint16_t)req[3] << 8);
	const uint16_t sample_cycles    = (uint16_t)req[4] | ((uint16_t)req[5] << 8);
	const uint8_t  resolution_bits  = req[6];
	/* The GD32G5 effective-resolution table also lists 14/16 bits.
	 * Accept those values here so the HAL can report their deliberate
	 * lack of implementation as STATUS_NOSUPPORT; reject all other
	 * values rather than silently rounding them. */
	switch (resolution_bits) {
	case 0u: /* "use the firmware default" */
	case 6u:
	case 8u:
	case 10u:
	case 12u:
	case 14u:
	case 16u:
		break;
	default:
		return STATUS_INVAL;
	}
	const int rv =
	    bridge_hw_adc_configure(channel, oversample_ratio, sample_cycles, resolution_bits);
	if (rv == BRIDGE_HW_ERR_INVAL) return STATUS_INVAL;
	if (rv == BRIDGE_HW_ERR_RANGE) return STATUS_OUT_OF_RANGE;
	if (rv == BRIDGE_HW_ERR_NOTIMPL) return STATUS_NOSUPPORT;
	if (rv < 0) return STATUS_IO;
	*reply_len = 0u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_adc_stream_begin(const uint8_t *req,
                                                    size_t         req_len,
                                                    uint8_t       *reply,
                                                    size_t         reply_cap,
                                                    size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 7u) return STATUS_INVAL;
	const uint8_t stream_id = req[0];
	const uint8_t channel   = req[1];
	/* req[2] reserved padding */
	const uint32_t sample_rate_hz = get_le32(&req[3]);
	if (stream_id >= GD32_BRIDGE_ADC_STREAM_COUNT) return STATUS_INVAL;
	if (sample_rate_hz == 0u) return STATUS_INVAL;
	const int rv = bridge_hw_adc_stream_begin(stream_id, channel, sample_rate_hz);
	if (rv == BRIDGE_HW_ERR_INVAL) return STATUS_INVAL;
	if (rv == BRIDGE_HW_ERR_RANGE) return STATUS_OUT_OF_RANGE;
	if (rv == BRIDGE_HW_ERR_NOTIMPL) return STATUS_NOSUPPORT;
	if (rv < 0) return STATUS_IO;
	*reply_len = 0u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_adc_stream_read(const uint8_t *req,
                                                   size_t         req_len,
                                                   uint8_t       *reply,
                                                   size_t         reply_cap,
                                                   size_t        *reply_len)
{
	if (req_len != 2u) return STATUS_INVAL;
	const uint8_t stream_id   = req[0];
	uint8_t       max_samples = req[1];
	if (stream_id >= GD32_BRIDGE_ADC_STREAM_COUNT) return STATUS_INVAL;
	/* The wire-framing layer needs a fixed reply length so the host
     * can clock the right number of bytes.  Reply is always
     * `1 + max_samples*2` bytes: byte 0 = `got` (the valid sample
     * count, 0..max_samples); subsequent bytes are the sample data,
     * with trailing slots zero-padded when the firmware ring had
     * fewer than `max_samples` samples ready. */
	if (max_samples == 0u) return STATUS_INVAL;
	if (max_samples > GD32_BRIDGE_ADC_STREAM_READ_MAX) {
		return STATUS_OUT_OF_RANGE;
	}

	/* Reply scratch on the firmware stack -- avoid pulling in the
     * full GD32_BRIDGE_MAX_PAYLOAD_BYTES buffer.  Sized to the
     * stream-read ceiling. */
	uint16_t  mv[GD32_BRIDGE_ADC_STREAM_READ_MAX];
	uint8_t   got = 0u;
	const int rv  = bridge_hw_adc_stream_read(stream_id, max_samples, &got, mv);
	if (rv == BRIDGE_HW_ERR_NOTIMPL) return STATUS_NOSUPPORT;
	/* Ring overrun (the DMA writer lapped the host's read cursor):
     * the documented wire answer is STATUS_BUSY -- "host should poll
     * faster" (alp-sdk docs/gd32-bridge-protocol.md §3.10).  The HAL has
     * already dropped the corrupt backlog and resynced its cursor,
     * so the next READ returns fresh samples. */
	if (rv == BRIDGE_HW_ERR_BUSY) return STATUS_BUSY;
	if (rv != BRIDGE_HW_OK) return status_from_hw(rv);
	if (got > max_samples) return STATUS_IO; /* HAL contract violation */

	const size_t need = 1u + (size_t)max_samples * 2u;
	if (reply_cap < need) return STATUS_NOMEM;
	reply[0] = got;
	for (uint8_t i = 0u; i < got; ++i) {
		reply[1u + i * 2u]      = (uint8_t)(mv[i] & 0xFFu);
		reply[1u + i * 2u + 1u] = (uint8_t)((mv[i] >> 8) & 0xFFu);
	}
	/* Pad with zeros so the on-wire envelope length is deterministic
     * (host pre-committed to clocking 1 + max_samples*2 reply bytes
     * + CRC).  Filler bytes are zero so the CRC is reproducible. */
	for (uint8_t i = got; i < max_samples; ++i) {
		reply[1u + i * 2u]      = 0u;
		reply[1u + i * 2u + 1u] = 0u;
	}
	*reply_len = need;
	return STATUS_OK;
}

/* CMD_ADC_SPECTRUM_READ (#496): one chunk of the latest FFT frame for an
 * FFT-terminal chain.  req: stream_id:u8 bin_offset:u16(LE) max_bins:u8.
 * reply (fixed 7 + max_bins*4): seq:u32 total_bins:u16 got:u8 bins[]
 * (float32 LE, zero-padded past `got`). */
static gd32_bridge_status_t handle_adc_spectrum_read(const uint8_t *req,
                                                     size_t         req_len,
                                                     uint8_t       *reply,
                                                     size_t         reply_cap,
                                                     size_t        *reply_len)
{
	if (req_len != 4u) return STATUS_INVAL;
	const uint8_t  stream_id  = req[0];
	const uint16_t bin_offset = (uint16_t)(req[1] | ((uint16_t)req[2] << 8));
	const uint8_t  max_bins   = req[3];
	if (stream_id >= GD32_BRIDGE_ADC_STREAM_COUNT) return STATUS_INVAL;
	if (max_bins == 0u) return STATUS_INVAL;
	if (max_bins > GD32_BRIDGE_ADC_SPECTRUM_READ_MAX) return STATUS_OUT_OF_RANGE;

	float     bins[GD32_BRIDGE_ADC_SPECTRUM_READ_MAX];
	uint32_t  seq   = 0u;
	uint16_t  total = 0u;
	uint8_t   got   = 0u;
	const int rv =
	    bridge_hw_adc_spectrum_read(stream_id, bin_offset, max_bins, &seq, &total, &got, bins);
	if (rv == BRIDGE_HW_ERR_NOTIMPL) return STATUS_NOSUPPORT; /* not FFT-bound */
	if (rv == BRIDGE_HW_ERR_BUSY) return STATUS_BUSY;         /* no frame yet -> poll */
	if (rv < 0) return STATUS_IO;
	if (got > max_bins) return STATUS_IO; /* HAL contract violation */

	const size_t need = 7u + (size_t)max_bins * 4u;
	if (reply_cap < need) return STATUS_NOMEM;
	reply[0] = (uint8_t)(seq & 0xFFu);
	reply[1] = (uint8_t)((seq >> 8) & 0xFFu);
	reply[2] = (uint8_t)((seq >> 16) & 0xFFu);
	reply[3] = (uint8_t)((seq >> 24) & 0xFFu);
	reply[4] = (uint8_t)(total & 0xFFu);
	reply[5] = (uint8_t)((total >> 8) & 0xFFu);
	reply[6] = got;
	for (uint8_t i = 0u; i < max_bins; ++i) {
		uint32_t w = 0u;
		if (i < got) memcpy(&w, &bins[i], sizeof(w)); /* float32 -> LE bytes */
		reply[7u + i * 4u]      = (uint8_t)(w & 0xFFu);
		reply[7u + i * 4u + 1u] = (uint8_t)((w >> 8) & 0xFFu);
		reply[7u + i * 4u + 2u] = (uint8_t)((w >> 16) & 0xFFu);
		reply[7u + i * 4u + 3u] = (uint8_t)((w >> 24) & 0xFFu);
	}
	*reply_len = need;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_adc_stream_end(const uint8_t *req,
                                                  size_t         req_len,
                                                  uint8_t       *reply,
                                                  size_t         reply_cap,
                                                  size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 1u) return STATUS_INVAL;
	const uint8_t stream_id = req[0];
	if (stream_id >= GD32_BRIDGE_ADC_STREAM_COUNT) return STATUS_INVAL;
	const int rv = bridge_hw_adc_stream_end(stream_id);
	if (rv == BRIDGE_HW_ERR_NOTIMPL) return STATUS_NOSUPPORT;
	if (rv < 0) return STATUS_IO;
	*reply_len = 0u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_trng_read(const uint8_t *req,
                                             size_t         req_len,
                                             uint8_t       *reply,
                                             size_t         reply_cap,
                                             size_t        *reply_len)
{
	if (req_len != 1u) return STATUS_INVAL;
	const uint8_t want = req[0];
	if (want == 0u || want > 32u) return STATUS_INVAL;
	if (reply_cap < want) return STATUS_NOMEM;
	const int rv = bridge_hw_trng_read(reply, (size_t)want);
	if (rv != BRIDGE_HW_OK) return status_from_hw(rv);
	*reply_len = want;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_tmu_compute(const uint8_t *req,
                                               size_t         req_len,
                                               uint8_t       *reply,
                                               size_t         reply_cap,
                                               size_t        *reply_len)
{
	if (req_len != 12u) return STATUS_INVAL;
	if (reply_cap < 4u) return STATUS_NOMEM;
	const uint8_t function = req[0];
	const uint8_t format   = req[1];
	/* req[2..3] are the reserved padding -- ignore. */
	const uint32_t in_a = get_le32(&req[4]);
	const uint32_t in_b = get_le32(&req[8]);
	/* Range-check function + format on the firmware side so the host
     * gets a precise STATUS_INVAL rather than a misleading STATUS_IO
     * out of the HAL backend.  The host driver already pre-validates;
     * this is the defence-in-depth pass. */
	if (function >= (uint8_t)BRIDGE_TMU_FN__COUNT) return STATUS_INVAL;
	if (format >= (uint8_t)BRIDGE_TMU_FMT__COUNT) return STATUS_INVAL;

	uint32_t  result = 0u;
	const int rv     = bridge_hw_tmu_compute(function, format, in_a, in_b, &result);
	if (rv == BRIDGE_HW_ERR_INVAL) return STATUS_INVAL;
	if (rv == BRIDGE_HW_ERR_RANGE) return STATUS_OUT_OF_RANGE;
	if (rv == BRIDGE_HW_ERR_NOTIMPL) return STATUS_NOSUPPORT;
	if (rv < 0) return STATUS_IO;
	put_le32(reply, result);
	*reply_len = 4u;
	return STATUS_OK;
}

/* ----------------------------------------------------------------- */
/* v0.5 (§2B.2) -- advanced timer extras                              */
/*                                                                    */
/* Wire frames per alp-sdk docs/gd32-bridge-protocol.md §3.y.  Every  */
/* handler validates the request payload length, calls into the HAL  */
/* hook, and maps the BRIDGE_HW_ERR_* return into the on-wire        */
/* STATUS_* code.  On the stub backend (bridge_hw_stub.c) all of     */
/* these return BRIDGE_HW_ERR_NOTIMPL -- which protocol_dispatch     */
/* maps to STATUS_NOSUPPORT on the wire -- so host code sees a       */
/* precise NOSUPPORT contract; the gd32 backend's real bodies live   */
/* in the per-peripheral TUs under hal/gd32/.                        */
/* ----------------------------------------------------------------- */

/* Translate a BRIDGE_HW_ERR_* return into a STATUS_*.  Centralised so
 * the new handler set has identical mapping. */
static gd32_bridge_status_t status_from_hw(int rv)
{
	if (rv == BRIDGE_HW_OK) return STATUS_OK;
	if (rv == BRIDGE_HW_ERR_INVAL) return STATUS_INVAL;
	if (rv == BRIDGE_HW_ERR_RANGE) return STATUS_OUT_OF_RANGE;
	if (rv == BRIDGE_HW_ERR_NOTIMPL) return STATUS_NOSUPPORT;
	if (rv == BRIDGE_HW_ERR_BUSY) return STATUS_BUSY;
	if (rv == BRIDGE_HW_ERR_NOT_READY) return STATUS_NOT_READY;
	return STATUS_IO;
}

static gd32_bridge_status_t handle_pwm_capture_begin(const uint8_t *req,
                                                     size_t         req_len,
                                                     uint8_t       *reply,
                                                     size_t         reply_cap,
                                                     size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 2u) return STATUS_INVAL;
	if (req[1] > 2u) return STATUS_INVAL; /* edge ∈ {RISING, FALLING, BOTH} */
	const int rv = bridge_hw_pwm_capture_begin(req[0], req[1]);
	*reply_len   = 0u;
	return status_from_hw(rv);
}

static gd32_bridge_status_t handle_pwm_capture_read(const uint8_t *req,
                                                    size_t         req_len,
                                                    uint8_t       *reply,
                                                    size_t         reply_cap,
                                                    size_t        *reply_len)
{
	if (req_len != 1u) return STATUS_INVAL;
	if (reply_cap < 8u) return STATUS_NOMEM;
	uint32_t                   period_ns = 0u;
	uint32_t                   pulse_ns  = 0u;
	const int                  rv = bridge_hw_pwm_capture_read(req[0], &period_ns, &pulse_ns);
	const gd32_bridge_status_t s  = status_from_hw(rv);
	if (s != STATUS_OK) {
		*reply_len = 0u;
		return s;
	}
	put_le32(&reply[0], period_ns);
	put_le32(&reply[4], pulse_ns);
	*reply_len = 8u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_pwm_capture_end(const uint8_t *req,
                                                   size_t         req_len,
                                                   uint8_t       *reply,
                                                   size_t         reply_cap,
                                                   size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 1u) return STATUS_INVAL;
	const int rv = bridge_hw_pwm_capture_end(req[0]);
	*reply_len   = 0u;
	return status_from_hw(rv);
}

static gd32_bridge_status_t handle_pwm_single_pulse(const uint8_t *req,
                                                    size_t         req_len,
                                                    uint8_t       *reply,
                                                    size_t         reply_cap,
                                                    size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 8u) return STATUS_INVAL;
	/* req[1..3] are reserved padding -- ignored by the firmware so
     * future-rev hosts can repurpose without an opcode bump. */
	const uint32_t pulse_ns = get_le32(&req[4]);
	if (pulse_ns == 0u) return STATUS_INVAL; /* zero-width pulse is a caller bug */
	const int rv = bridge_hw_pwm_single_pulse(req[0], pulse_ns);
	*reply_len   = 0u;
	return status_from_hw(rv);
}

static gd32_bridge_status_t handle_timer_sync(const uint8_t *req,
                                              size_t         req_len,
                                              uint8_t       *reply,
                                              size_t         reply_cap,
                                              size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 3u) return STATUS_INVAL;
	const int rv = bridge_hw_timer_sync(req[0], req[1], req[2]);
	*reply_len   = 0u;
	return status_from_hw(rv);
}

/* ----------------------------------------------------------------- */
/* v0.5 (§2B.3) -- system power-mode set                              */
/* ----------------------------------------------------------------- */

static gd32_bridge_status_t handle_power_mode_set(const uint8_t *req,
                                                  size_t         req_len,
                                                  uint8_t       *reply,
                                                  size_t         reply_cap,
                                                  size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 10u) return STATUS_INVAL;
	if (req[0] > 3u) return STATUS_INVAL; /* mode in {RUN, SLEEP, DEEP_SLEEP, STANDBY} */
	/* An I2C3 proxy job runs at base level and must not be cut off by a
	 * low-power entry. */
	if (bridge_hw_i2cm_busy()) {
		*reply_len = 0u;
		return STATUS_BUSY;
	}
	/* req[1] = POWER_FLAG_* (0 from a host that predates the flags). */
	const uint32_t wake_bitmap   = get_le32(&req[2]);
	const uint32_t wake_after_ms = get_le32(&req[6]);
	const int      rv = bridge_hw_power_mode_set(req[0], wake_bitmap, wake_after_ms, req[1]);
	/* ATTN is driven low once a transition out of RUN is ACCEPTED (a refused
	 * request, e.g. BUSY, leaves the link as it was; the entry itself runs
	 * later at base level, after this reply drains).  STANDBY is a reset,
	 * which returns PA14 to SWCLK on its own. */
	if (rv == BRIDGE_HW_OK && req[0] != 0u) bridge_hw_attn_quiesce();
	*reply_len = 0u;
	return status_from_hw(rv);
}

/* ----------------------------------------------------------------- */
/* v0.17 -- I2C3 master proxy (I2C link only; see CMD_I2CM_* in       */
/* protocol.h).  The handlers only validate and queue: the transfer   */
/* itself runs at base level from bridge_hw_tick().                   */
/* ----------------------------------------------------------------- */

static gd32_bridge_status_t handle_i2cm_config(const uint8_t *req,
                                               size_t         req_len,
                                               uint8_t       *reply,
                                               size_t         reply_cap,
                                               size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 2u) return STATUS_INVAL;
	const int rv = bridge_hw_i2cm_config(get_le16(req));
	*reply_len   = 0u;
	return status_from_hw(rv);
}

static gd32_bridge_status_t handle_i2cm_xfer(const uint8_t *req,
                                             size_t         req_len,
                                             uint8_t       *reply,
                                             size_t         reply_cap,
                                             size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	/* tag, addr7, flags, wlen, rlen, wdata[wlen] */
	if (req_len < 5u) return STATUS_INVAL;
	const uint8_t wlen = req[3];
	const uint8_t rlen = req[4];
	if (req[2] != 0u || wlen > I2CM_MAX_WRITE || rlen > I2CM_MAX_READ || req[1] > 0x7Fu ||
	    req_len != 5u + (size_t)wlen) {
		return STATUS_INVAL;
	}
	const int rv = bridge_hw_i2cm_xfer(req[0], req[1], &req[5], wlen, rlen);
	*reply_len   = 0u;
	return status_from_hw(rv);
}

static gd32_bridge_status_t handle_i2cm_result(const uint8_t *req,
                                               size_t         req_len,
                                               uint8_t       *reply,
                                               size_t         reply_cap,
                                               size_t        *reply_len)
{
	(void)req;
	if (req_len != 0u) return STATUS_INVAL;
	if (reply_cap < 3u + I2CM_MAX_READ) return STATUS_NOMEM;
	uint8_t   nread = 0u;
	const int rv    = bridge_hw_i2cm_result(&reply[0], &reply[1], &nread, &reply[3]);
	if (rv != BRIDGE_HW_OK) {
		*reply_len = 0u;
		return status_from_hw(rv);
	}
	reply[2]   = nread;
	*reply_len = 3u + (size_t)nread;
	return STATUS_OK;
}

/* ----------------------------------------------------------------- */
/* v0.5 (§2B wave-2) -- chunked DSP-chain upload                      */
/*                                                                    */
/* Wire frames per alp-sdk docs/gd32-bridge-protocol.md §3.x.         */
/* ----------------------------------------------------------------- */

static gd32_bridge_status_t handle_adc_dsp_chain_open(const uint8_t *req,
                                                      size_t         req_len,
                                                      uint8_t       *reply,
                                                      size_t         reply_cap,
                                                      size_t        *reply_len)
{
	(void)req;
	if (req_len != 0u) return STATUS_INVAL;
	if (reply_cap < 1u) return STATUS_NOMEM;
	uint8_t                    chain_id = 0u;
	const int                  rv       = bridge_hw_adc_dsp_chain_open(&chain_id);
	const gd32_bridge_status_t s        = status_from_hw(rv);
	if (s != STATUS_OK) {
		*reply_len = 0u;
		return s;
	}
	reply[0]   = chain_id;
	*reply_len = 1u;
	return STATUS_OK;
}

static gd32_bridge_status_t handle_adc_dsp_stage_push(const uint8_t *req,
                                                      size_t         req_len,
                                                      uint8_t       *reply,
                                                      size_t         reply_cap,
                                                      size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	/* Header is 7 bytes: chain_id, stage_index, kind, chunk_offset:u16,
     * chunk_total_size:u16.  chunk_data follows at req[7..req_len). */
	if (req_len < 7u) return STATUS_INVAL;
	if (req[2] > 3u) return STATUS_INVAL; /* kind ∈ {FIR, IIR, WINDOW, FFT} */
	const uint8_t  chain_id         = req[0];
	const uint8_t  stage_index      = req[1];
	const uint8_t  kind             = req[2];
	const uint16_t chunk_offset     = (uint16_t)req[3] | ((uint16_t)req[4] << 8);
	const uint16_t chunk_total_size = (uint16_t)req[5] | ((uint16_t)req[6] << 8);
	const size_t   chunk_data_len   = req_len - 7u;
	const uint8_t *chunk_data       = (chunk_data_len > 0u) ? &req[7] : (const uint8_t *)0;
	/* Defence-in-depth: chunk_data + chunk_offset must stay within the
     * declared chunk_total_size.  The host helper already enforces
     * this, but firmware re-checks so a malformed wire request can't
     * walk past the per-stage scratch buffer. */
	if ((uint32_t)chunk_offset + (uint32_t)chunk_data_len > (uint32_t)chunk_total_size) {
		return STATUS_OUT_OF_RANGE;
	}
	const int rv = bridge_hw_adc_dsp_stage_push(
	    chain_id, stage_index, kind, chunk_offset, chunk_total_size, chunk_data, chunk_data_len);
	*reply_len = 0u;
	return status_from_hw(rv);
}

static gd32_bridge_status_t handle_adc_dsp_chain_bind(const uint8_t *req,
                                                      size_t         req_len,
                                                      uint8_t       *reply,
                                                      size_t         reply_cap,
                                                      size_t        *reply_len)
{
	(void)reply;
	(void)reply_cap;
	if (req_len != 2u) return STATUS_INVAL;
	if (req[1] >= GD32_BRIDGE_ADC_STREAM_COUNT) return STATUS_INVAL;
	const int rv = bridge_hw_adc_dsp_chain_bind(req[0], req[1]);
	*reply_len   = 0u;
	return status_from_hw(rv);
}

/* --------------------------------------------------------------- */
/* v0.7 -- link-feature negotiation (v0.15: u32 word + max_payload)   */
/* --------------------------------------------------------------- */

/* Armed link features (GD32_BRIDGE_LINK_FEAT_*) and effective payload
 * ceiling, ONE ENTRY PER LINK.  The negotiation command can arrive over
 * either transport; the SPI transport consults the accessors when it stages
 * replies (the I2C transport never stamps -- STATUS_NO_PENDING owns bit 7
 * there).  Reset default: everything off and max_payload 65 = the pre-v0.7
 * wire, so an un-negotiated link is byte-identical to older firmware.
 *
 * #130: this was a single process-wide byte.  CMD_LINK_FEATURES sits in
 * the shared dispatch table and is therefore reachable from BOTH
 * transport ISRs, but only the SPI transport consumes the result -- so
 * an I2C-side driver open/close/reset issuing the documented idempotent
 * `features = 0` disarmed an active SPI STATUS_SEQ session with nothing
 * telling the SPI host, switching off its only detector for the
 * stale-reply residual hazard (src/transport_spi.c stale-reply note,
 * silicon 2026-06-06).  The reverse was equally reachable: an I2C-side
 * `features = 1` re-framed the SPI wire for a host that never
 * negotiated it.
 *
 * `volatile` because the two writers run at different NVIC preemption
 * levels -- CS-EXTI at BRIDGE_CS_IRQ_PRIO (1) can preempt I2C0_EV at
 * BRIDGE_I2C_IRQ_PRIO (2) mid-handler (hal/bridge_board_config.h).
 * Indexing by link means the two levels no longer touch the same word,
 * so the qualifier is now belt-and-braces rather than load-bearing --
 * but the array is still read from one level and written from another
 * for its own link, and this build sets no -O at all (#26), which is
 * the only reason the pre-#130 miss was not already observable. */
static volatile uint32_t link_features[GD32_BRIDGE_LINK_COUNT];
static volatile uint16_t link_max_payload[GD32_BRIDGE_LINK_COUNT] = {
	GD32_BRIDGE_MAX_PAYLOAD_BYTES,
	GD32_BRIDGE_MAX_PAYLOAD_BYTES,
};

uint32_t protocol_link_features(gd32_bridge_link_t link)
{
	if ((unsigned)link >= (unsigned)GD32_BRIDGE_LINK_COUNT) return 0u;
	return link_features[link];
}

uint16_t protocol_link_max_payload(gd32_bridge_link_t link)
{
	if ((unsigned)link >= (unsigned)GD32_BRIDGE_LINK_COUNT) return GD32_BRIDGE_MAX_PAYLOAD_BYTES;
	return link_max_payload[link];
}

/* Feature bits this build implements on `link` (design 2.2 step 1).  I2C
 * only ever echoes STATUS_SEQ, preserving the idempotent `features = 0`
 * open/close path; SPI adds BIG_FRAME + BATCH everywhere, and ATTN +
 * ADC_STREAM2 where the HAL backs them. */
static uint32_t link_supported(gd32_bridge_link_t link)
{
	if (link != GD32_BRIDGE_LINK_SPI) return GD32_BRIDGE_LINK_FEAT_STATUS_SEQ;
	uint32_t supported = GD32_BRIDGE_LINK_FEAT_STATUS_SEQ | GD32_BRIDGE_LINK_FEAT_BIG_FRAME |
	                     GD32_BRIDGE_LINK_FEAT_BATCH;
	if (bridge_hw_attn_supported()) supported |= GD32_BRIDGE_LINK_FEAT_ATTN;
	if (bridge_hw_adc_stream2_supported()) supported |= GD32_BRIDGE_LINK_FEAT_ADC_STREAM2;
	return supported;
}

/* Arm `granted`/`mp` on `link`.  The ATTN pin transition runs only when the
 * ATTN bit actually changes, so re-sending the same `want` causes no pin
 * activity.  Returns the word actually armed: ATTN is dropped if the HAL
 * could not switch the pin on. */
static uint32_t link_arm(gd32_bridge_link_t link, uint32_t granted, uint16_t mp)
{
	const bool attn_was = (link_features[link] & GD32_BRIDGE_LINK_FEAT_ATTN) != 0u;
	const bool attn_now = (granted & GD32_BRIDGE_LINK_FEAT_ATTN) != 0u;
	if (attn_was != attn_now) {
		if (bridge_hw_attn_enable(attn_now) != BRIDGE_HW_OK && attn_now) {
			granted &= ~GD32_BRIDGE_LINK_FEAT_ATTN;
		}
	}
	/* Watermark events exist only while READ2 is reachable.  On removal clear
	 * what is pending and stop new ones: nothing could clear them afterwards
	 * (READ2 answers NOSUPPORT) and ATTN would keep rising. */
	const bool s2_was = (link_features[link] & GD32_BRIDGE_LINK_FEAT_ADC_STREAM2) != 0u;
	const bool s2_now = (granted & GD32_BRIDGE_LINK_FEAT_ADC_STREAM2) != 0u;
	if (link == GD32_BRIDGE_LINK_SPI && s2_was != s2_now) {
		bridge_hw_attn_streams_enable(s2_now);
		if (!s2_now) {
			for (uint8_t sid = 0u; sid < GD32_BRIDGE_ADC_STREAM_COUNT; ++sid) {
				bridge_hw_attn_event_clear(sid);
			}
		}
	}
	link_features[link]    = granted;
	link_max_payload[link] = mp;
	return granted;
}

/* The one link-SCOPED handler, so it does not match cmd_handler_t and is
 * called directly from protocol_dispatch's switch instead of going
 * through the table (#130).  Passing the tag as an argument -- rather
 * than parking a "current link" in a file-scope variable the whole table
 * could read -- is deliberate: CS-EXTI at priority 1 preempts I2C0_EV at
 * priority 2, so any such variable would need save/restore discipline at
 * every dispatch to survive nesting.  An argument needs none.
 *
 * Two request forms, chosen by length (design section 2.1):
 *   1 byte  `features:u8`  legacy.  The link's whole word becomes
 *           `features & STATUS_SEQ` and max_payload 65, so 0 still means
 *           "disable everything".  Reply `granted:u8`.
 *   6 bytes `want:u32 max_payload_req:u16`  v0.15.  Reply 10 bytes
 *           `granted:u32 supported:u32 max_payload:u16`.
 * Either way the new state is armed BEFORE the reply is staged -- the
 * reply to this very command already rides the new framing (the host
 * treats its stamp as the sequence baseline, and an ATTN-granting reply is
 * the first ATTN-signalled one).  Armed for THIS link only; STATUS_SEQ has
 * no effect on the I2C wire either way (that transport never stamps), so
 * the I2C answer stays truthful rather than STATUS_NOSUPPORT -- which
 * would break the documented idempotent open/close path that issues
 * `features = 0` unconditionally. */
static gd32_bridge_status_t handle_link_features(gd32_bridge_link_t link,
                                                 const uint8_t     *req,
                                                 size_t             req_len,
                                                 uint8_t           *reply,
                                                 size_t             reply_cap,
                                                 size_t            *reply_len)
{
	if (req_len != 1u && req_len != 6u) return STATUS_INVAL;
	if ((unsigned)link >= (unsigned)GD32_BRIDGE_LINK_COUNT) return STATUS_INVAL;

	if (req_len == 1u) {
		if (reply_cap < 1u) return STATUS_NOMEM;
		const uint32_t granted = link_arm(link,
		                                  (uint32_t)req[0] & GD32_BRIDGE_LINK_FEAT_STATUS_SEQ,
		                                  GD32_BRIDGE_MAX_PAYLOAD_BYTES);
		reply[0]               = (uint8_t)granted;
		*reply_len             = 1u;
		return STATUS_OK;
	}

	if (reply_cap < 10u) return STATUS_NOMEM;
	const uint32_t want      = get_le32(&req[0]);
	const uint16_t mp_req    = get_le16(&req[4]);
	const uint32_t supported = link_supported(link);
	uint32_t       g         = want & supported;

	/* ATTN rides the STATUS_SEQ stamp, and PA14 is SWCLK: never drive it
	 * while a debugger may be driving it too (design F2). */
	if ((g & GD32_BRIDGE_LINK_FEAT_ATTN) != 0u &&
	    ((g & GD32_BRIDGE_LINK_FEAT_STATUS_SEQ) == 0u || bridge_hw_debugger_attached())) {
		g &= ~GD32_BRIDGE_LINK_FEAT_ATTN;
	}

	uint16_t mp = GD32_BRIDGE_MAX_PAYLOAD_BYTES;
	if ((g & GD32_BRIDGE_LINK_FEAT_BIG_FRAME) != 0u) {
		mp = mp_req;
		if (mp > GD32_BRIDGE_SPI_BIG_MAX_PAYLOAD_BYTES) mp = GD32_BRIDGE_SPI_BIG_MAX_PAYLOAD_BYTES;
		if (mp <= GD32_BRIDGE_MAX_PAYLOAD_BYTES) {
			mp = GD32_BRIDGE_MAX_PAYLOAD_BYTES;
			g &= ~GD32_BRIDGE_LINK_FEAT_BIG_FRAME;
		}
	}

	g = link_arm(link, g, mp);
	put_le32(&reply[0], g);
	put_le32(&reply[4], supported);
	put_le16(&reply[8], mp);
	*reply_len = 10u;
	return STATUS_OK;
}

/* --------------------------------------------------------------- */
/* v0.15 -- BEGIN2 / READ2 (ADC stream with lossless accounting)       */
/* --------------------------------------------------------------- */

/* Watermark values BEGIN2 accepts (design 5.1).  0 = no events. */
static bool begin2_watermark_valid(uint16_t w)
{
	return w == 0u || w == 16u || w == 32u || w == 64u || w == 128u || w == 256u || w == 512u;
}

static gd32_bridge_status_t handle_adc_stream_begin2(const uint8_t *req,
                                                     size_t         req_len,
                                                     uint8_t       *reply,
                                                     size_t         reply_cap,
                                                     size_t        *reply_len)
{
	if (req_len != GD32_BRIDGE_BEGIN2_REQ_BYTES) return STATUS_INVAL;
	const uint8_t  stream_id   = req[0];
	const uint8_t  channel     = req[1];
	const uint8_t  trigger_src = req[2];
	const uint8_t  trigger_arg = req[3];
	const uint32_t rate_hz     = get_le32(&req[4]);
	const uint16_t watermark   = get_le16(&req[8]);
	const uint16_t reserved    = get_le16(&req[10]);
	if (stream_id >= GD32_BRIDGE_ADC_STREAM_COUNT) return STATUS_INVAL;
	/* 0x00 PACE_TIMER is the only trigger this firmware implements.  0x01..0x05
	 * are reserved (a later minor may implement them) and answer NOSUPPORT;
	 * anything higher is malformed. */
	if (trigger_src >= 0x06u) return STATUS_INVAL;
	if (trigger_src != 0x00u) return STATUS_NOSUPPORT;
	if (trigger_arg != 0u || reserved != 0u) return STATUS_INVAL;
	if (rate_hz == 0u) return STATUS_INVAL;
	if (!begin2_watermark_valid(watermark)) return STATUS_INVAL;
	if (rate_hz > GD32_BRIDGE_ADC_STREAM2_RATE_MAX_HZ) return STATUS_OUT_OF_RANGE;
	if (reply_cap < GD32_BRIDGE_BEGIN2_REPLY_BYTES) return STATUS_NOMEM;

	bridge_hw_adc_stream2_info_t info;
	memset(&info, 0, sizeof(info));
	const int rv = bridge_hw_adc_stream_begin2(stream_id, channel, rate_hz, watermark, &info);
	if (rv != BRIDGE_HW_OK) return status_from_hw(rv);

	put_le32(&reply[0], info.tick_hz);
	put_le32(&reply[4], info.period_ticks);
	put_le16(&reply[8], info.full_scale);
	put_le16(&reply[10], info.vref_mv);
	reply[12] = info.flags;
	put_le16(&reply[13], info.watermark);
	put_le16(&reply[15], info.ring_depth);
	*reply_len = GD32_BRIDGE_BEGIN2_REPLY_BYTES;
	return STATUS_OK;
}

/* READ2: reply `first_index:u32 dropped:u32 got:u8 codes[got]` (variable
 * length, 9 + 2*got).  The capacity check precedes the ring consume: a
 * request whose worst-case reply does not fit answers NOMEM with the ring
 * untouched (the legacy handler consumes first). */
static gd32_bridge_status_t handle_adc_stream_read2(gd32_bridge_link_t link,
                                                    const uint8_t     *req,
                                                    size_t             req_len,
                                                    uint8_t           *reply,
                                                    size_t             reply_cap,
                                                    size_t            *reply_len)
{
	if (req_len != 2u) return STATUS_INVAL;
	const uint8_t stream_id   = req[0];
	const uint8_t max_samples = req[1];
	if (stream_id >= GD32_BRIDGE_ADC_STREAM_COUNT) return STATUS_INVAL;
	if (max_samples == 0u) return STATUS_INVAL;
	const unsigned ceiling =
	    ((unsigned)protocol_link_max_payload(link) - GD32_BRIDGE_READ2_HDR_BYTES) / 2u;
	if ((unsigned)max_samples > ceiling) return STATUS_OUT_OF_RANGE;
	const size_t need = GD32_BRIDGE_READ2_HDR_BYTES + (size_t)max_samples * 2u;
	if (reply_cap < need) return STATUS_NOMEM;

	uint32_t  first_index = 0u;
	uint32_t  dropped     = 0u;
	uint8_t   got         = 0u;
	const int rv          = bridge_hw_adc_stream_read2(
	    stream_id, max_samples, &first_index, &dropped, &got, &reply[GD32_BRIDGE_READ2_HDR_BYTES]);
	if (rv != BRIDGE_HW_OK) return status_from_hw(rv);
	if (got > max_samples) return STATUS_IO; /* HAL contract violation */

	put_le32(&reply[0], first_index);
	put_le32(&reply[4], dropped);
	reply[8]   = got;
	*reply_len = GD32_BRIDGE_READ2_HDR_BYTES + (size_t)got * 2u;
	return STATUS_OK;
}

/* --------------------------------------------------------------- */
/* v0.15 -- CMD_BATCH                                                   */
/* --------------------------------------------------------------- */

typedef struct {
	uint8_t op;
	uint8_t req_len;   /* fixed request payload length */
	uint8_t max_reply; /* fixed max reply payload; ignored for READ2 */
} batch_op_t;

/* The allow-list: bounded, side-effect-local handlers only (design 6.3).
 * Left out on purpose: ADC_READ (up to 1 ms each), stream BEGIN/END
 * (calibration can spin ~200000 iterations), TRNG_READ, SE_RESET (needs
 * host-timed gaps), DSP chain opcodes (setup-time), LINK_FEATURES (would
 * reframe mid-reply), POWER_MODE_SET, OTA and nested BATCH. */
static const batch_op_t batch_ops[] = {
	{ CMD_PING, 0u, 0u },
	{ CMD_GPIO_READ, 4u, 4u },
	{ CMD_GPIO_WRITE, 8u, 0u },
	{ CMD_PWM_SET, 10u, 0u },
	{ CMD_PWM_GET, 1u, 8u },
	{ CMD_PWM_CAPTURE_READ, 1u, 8u },
	{ CMD_ADC_STREAM_READ2, 2u, 0u },
	{ CMD_DA9292_STATUS_FORWARD, 0u, 1u },
	{ CMD_DAC_SET, 4u, 0u },
	{ CMD_DAC_GET, 1u, 2u },
	{ CMD_QENC_READ, 1u, 4u },
	{ CMD_QENC_RESET, 1u, 0u },
	{ CMD_COUNTER_READ, 1u, 4u },
	{ CMD_TMU_COMPUTE, 12u, 4u },
};

static const batch_op_t *batch_op_find(uint8_t op)
{
	for (size_t i = 0u; i < sizeof(batch_ops) / sizeof(batch_ops[0]); ++i) {
		if (batch_ops[i].op == op) return &batch_ops[i];
	}
	return NULL;
}

/* Worst-case reply payload of one sub-op; READ2's depends on its
 * `max_samples` argument (args[1]; the entry length was already checked). */
static size_t batch_op_max_reply(const batch_op_t *d, const uint8_t *args)
{
	if (d->op == CMD_ADC_STREAM_READ2) {
		return GD32_BRIDGE_READ2_HDR_BYTES + (size_t)args[1] * 2u;
	}
	return d->max_reply;
}

static gd32_bridge_status_t protocol_dispatch_inner(gd32_bridge_link_t link,
                                                    uint8_t            cmd,
                                                    const uint8_t     *req_payload,
                                                    size_t             req_payload_len,
                                                    uint8_t           *reply_payload,
                                                    size_t             reply_payload_cap,
                                                    size_t            *reply_payload_len);

/* Request  : count:u8 (1..16), then count entries {op:u8, len:u8, args[len]}.
 * Reply    : executed:u8, then per executed op {status:u8, len:u8, payload[len]}.
 *
 * Validation runs to completion BEFORE anything executes, so a rejected
 * batch has no side effects and answers with an empty payload.  The request
 * length must equal 1 + sum(2 + len_i) exactly: a trailing byte is the
 * signature of a zero-extended capture and is refused.  Execution then
 * stops at the first non-OK sub-status (which always has len = 0), and
 * every sub-op runs through the same handler a standalone request would,
 * writing its reply in place after the entry header -- no temporary copy.
 * The outer status is OK whenever validation passed. */
static gd32_bridge_status_t handle_batch(gd32_bridge_link_t link,
                                         const uint8_t     *req,
                                         size_t             req_len,
                                         uint8_t           *reply,
                                         size_t             reply_cap,
                                         size_t            *reply_len)
{
	if (req_len < 1u) return STATUS_INVAL;
	const uint8_t count = req[0];
	if (count == 0u) return STATUS_INVAL;
	if (count > GD32_BRIDGE_BATCH_MAX_OPS) return STATUS_OUT_OF_RANGE;

	size_t  off        = 1u;
	size_t  worst      = 1u; /* executed:u8 */
	uint8_t read2_seen = 0u; /* stream ids already read in this batch */
	for (uint8_t i = 0u; i < count; ++i) {
		if (off + 2u > req_len) return STATUS_INVAL;
		const uint8_t op  = req[off];
		const uint8_t len = req[off + 1u];
		if (off + 2u + (size_t)len > req_len) return STATUS_INVAL;
		const batch_op_t *d = batch_op_find(op);
		if (d == NULL || len != d->req_len) return STATUS_INVAL;
		/* One READ2 per stream: repeated reads of a stream inside one CS ISR
		 * see a stale lap count (the lap ISR cannot run here) and would trip
		 * the undercount sentinel on the third read. */
		if (op == CMD_ADC_STREAM_READ2 && req[off + 2u] < 8u) {
			const uint8_t bit = (uint8_t)(1u << req[off + 2u]);
			if (read2_seen & bit) return STATUS_INVAL;
			read2_seen |= bit;
		}
		worst += 2u + batch_op_max_reply(d, &req[off + 2u]);
		off += 2u + (size_t)len;
	}
	if (off != req_len) return STATUS_INVAL;
	if (worst > (size_t)protocol_link_max_payload(link)) return STATUS_OUT_OF_RANGE;
	if (reply_cap < worst) return STATUS_NOMEM;

	size_t  in       = 1u;
	size_t  out      = 1u;
	uint8_t executed = 0u;
	for (uint8_t i = 0u; i < count; ++i) {
		const uint8_t        op     = req[in];
		const uint8_t        len    = req[in + 1u];
		const uint8_t       *args   = &req[in + 2u];
		const batch_op_t    *d      = batch_op_find(op);
		const size_t         maxrep = batch_op_max_reply(d, args);
		size_t               rl     = 0u;
		gd32_bridge_status_t st =
		    protocol_dispatch_inner(link, op, args, len, &reply[out + 2u], maxrep, &rl);
		if (st == STATUS_OK && rl > maxrep) st = STATUS_IO; /* handler overran its cap */
		++executed;
		if (st != STATUS_OK) {
			reply[out]      = (uint8_t)st;
			reply[out + 1u] = 0u;
			out += 2u;
			break;
		}
		reply[out]      = (uint8_t)STATUS_OK;
		reply[out + 1u] = (uint8_t)rl;
		out += 2u + rl;
		in += 2u + (size_t)len;
	}
	reply[0]   = executed;
	*reply_len = out;
	return STATUS_OK;
}

/* --------------------------------------------------------------- */
/* Dispatch                                                          */
/* --------------------------------------------------------------- */

typedef gd32_bridge_status_t (*cmd_handler_t)(const uint8_t *, size_t, uint8_t *, size_t, size_t *);

/* Both transport ISRs dispatch synchronously, and SPI's CS EXTI can
 * pre-empt the I2C event handler.  A lock-free atomic flag makes the
 * higher-priority nested request fail fast with STATUS_BUSY before it
 * reaches shared protocol or HAL state (#19). */
static atomic_flag dispatch_in_flight = ATOMIC_FLAG_INIT;

/* Opcodes the I2C link may carry (v0.15 policy; design section 7).  The
 * kernel gpio-gd32-bridge driver, tools/gd32-ota-host and the BRD_I2C
 * bring-up example use these, plus PWM_SET / PWM_GET (v0.17: the Linux
 * host needs a PWM provider, e.g. a panel backlight; both handlers are
 * short and ownership is unchanged -- the CM33 keeps using SPI). */
static bool i2c_opcode_allowed(uint8_t cmd)
{
	switch (cmd) {
	case CMD_PING:
	case CMD_GET_VERSION:
	case CMD_GET_BUILD_ID:
	case CMD_RESET_REASON:
	case CMD_GPIO_READ:
	case CMD_GPIO_WRITE:
	case CMD_PWM_SET:
	case CMD_PWM_GET:
	case CMD_SE_RESET:
	case CMD_BOOT_CONFIG:
	case CMD_LINK_FEATURES:
	case CMD_I2CM_CONFIG:
	case CMD_I2CM_XFER:
	case CMD_I2CM_RESULT:
		return true;
	default:
		return cmd >= CMD_OTA_BEGIN;
	}
}

volatile uint32_t bridge_i2c_denied_count;
volatile uint8_t  bridge_i2c_denied_last_cmd;

static bool link_feature_granted(gd32_bridge_link_t link, uint32_t bit)
{
	return (protocol_link_features(link) & bit) != 0u;
}

/* Two-tier dispatch: a sparse switch on opcode keeps the table size
 * small (vs a dense 256-entry array) without losing the "one handler
 * table" property. */
static gd32_bridge_status_t protocol_dispatch_inner(gd32_bridge_link_t link,
                                                    uint8_t            cmd,
                                                    const uint8_t     *req_payload,
                                                    size_t             req_payload_len,
                                                    uint8_t           *reply_payload,
                                                    size_t             reply_payload_cap,
                                                    size_t            *reply_payload_len)
{
	/* Trial/confirm gate (bench fact 2026-09-26, E1M-V2M103): a
	 * not-yet-confirmed OTA trial must never let the host believe a
	 * session is stable -- it can still revert on the armed watchdog.
	 * Note the frame (the confirm signal the base-level tick is
	 * waiting for) and answer BUSY with an empty payload for EVERY
	 * opcode, on EITHER link, ahead of the normal dispatch. */
	if (ota_trial_unconfirmed()) {
		ota_note_frame();
		*reply_payload_len = 0u;
		return STATUS_BUSY;
	}
	/* I2C opcode policy (v0.15, unconditional): the I2C link carries only
	 * the identity / GPIO / SE-reset / negotiation opcodes plus OTA.  Every
	 * other opcode is refused here, before its handler can run, with an
	 * empty NOSUPPORT.  The trial gate above stays first so any CRC-valid
	 * frame on either link still confirms a trial.  SPI is unrestricted. */
	if (link == GD32_BRIDGE_LINK_I2C && !i2c_opcode_allowed(cmd)) {
		bridge_i2c_denied_count++;
		bridge_i2c_denied_last_cmd = cmd;
		*reply_payload_len         = 0u;
		return STATUS_NOSUPPORT;
	}
	cmd_handler_t h = NULL;
	switch (cmd) {
	case CMD_PING:
		h = handle_ping;
		break;
	case CMD_GET_VERSION:
		h = handle_get_version;
		break;
	case CMD_GET_BUILD_ID:
		h = handle_get_build_id;
		break;
	case CMD_RESET_REASON:
		h = handle_reset_reason;
		break;
	case CMD_GPIO_READ:
		h = handle_gpio_read;
		break;
	case CMD_GPIO_WRITE:
		h = handle_gpio_write;
		break;
	case CMD_PWM_SET:
		h = handle_pwm_set;
		break;
	case CMD_PWM_GET:
		h = handle_pwm_get;
		break;
	case CMD_PWM_CONFIGURE:
		h = handle_pwm_configure;
		break;
	case CMD_ADC_READ:
		h = handle_adc_read;
		break;
	case CMD_ADC_CONFIGURE:
		h = handle_adc_configure;
		break;
	case CMD_ADC_STREAM_BEGIN:
		h = handle_adc_stream_begin;
		break;
	case CMD_ADC_STREAM_READ:
		h = handle_adc_stream_read;
		break;
	case CMD_ADC_STREAM_END:
		h = handle_adc_stream_end;
		break;
	case CMD_ADC_SPECTRUM_READ:
		h = handle_adc_spectrum_read;
		break;
	case CMD_TRNG_READ:
		h = handle_trng_read;
		break;
	case CMD_TMU_COMPUTE:
		h = handle_tmu_compute;
		break;
	case CMD_DA9292_STATUS_FORWARD:
		h = handle_da9292_forward;
		break;
	case CMD_SE_RESET:
		h = handle_se_reset;
		break;
	case CMD_BOOT_CONFIG:
		h = handle_boot_config;
		break;
	case CMD_DAC_SET:
		h = handle_dac_set;
		break;
	case CMD_DAC_GET:
		h = handle_dac_get;
		break;
	case CMD_QENC_READ:
		h = handle_qenc_read;
		break;
	case CMD_QENC_RESET:
		h = handle_qenc_reset;
		break;
	case CMD_COUNTER_READ:
		h = handle_counter_read;
		break;
	/* v0.5 (§2B.2) advanced timer extras + (§2B.3) power-mode set.
     * Real bodies in hal/gd32/ (pwm_capture.c / pwm.c / timer_sync.c /
     * power.c); the stub backend answers STATUS_NOSUPPORT. */
	case CMD_PWM_CAPTURE_BEGIN:
		h = handle_pwm_capture_begin;
		break;
	case CMD_PWM_CAPTURE_READ:
		h = handle_pwm_capture_read;
		break;
	case CMD_PWM_CAPTURE_END:
		h = handle_pwm_capture_end;
		break;
	case CMD_PWM_SINGLE_PULSE:
		h = handle_pwm_single_pulse;
		break;
	case CMD_TIMER_SYNC:
		h = handle_timer_sync;
		break;
	case CMD_POWER_MODE_SET:
		h = handle_power_mode_set;
		break;
	/* v0.15: each of these needs its link feature granted on THIS link and
	 * is link-scoped (the per-link max_payload shapes the reply). */
	case CMD_BATCH:
		if (!link_feature_granted(link, GD32_BRIDGE_LINK_FEAT_BATCH)) {
			*reply_payload_len = 0u;
			return STATUS_NOSUPPORT;
		}
		*reply_payload_len = 0u;
		return handle_batch(link,
		                    req_payload,
		                    req_payload_len,
		                    reply_payload,
		                    reply_payload_cap,
		                    reply_payload_len);
	case CMD_ADC_STREAM_BEGIN2:
		if (!link_feature_granted(link, GD32_BRIDGE_LINK_FEAT_ADC_STREAM2)) {
			*reply_payload_len = 0u;
			return STATUS_NOSUPPORT;
		}
		h = handle_adc_stream_begin2;
		break;
	case CMD_ADC_STREAM_READ2:
		if (!link_feature_granted(link, GD32_BRIDGE_LINK_FEAT_ADC_STREAM2)) {
			*reply_payload_len = 0u;
			return STATUS_NOSUPPORT;
		}
		*reply_payload_len = 0u;
		return handle_adc_stream_read2(link,
		                               req_payload,
		                               req_payload_len,
		                               reply_payload,
		                               reply_payload_cap,
		                               reply_payload_len);
	/* v0.17: I2C3 proxy -- I2C link only, never batched. */
	case CMD_I2CM_CONFIG:
	case CMD_I2CM_XFER:
	case CMD_I2CM_RESULT:
		if (link != GD32_BRIDGE_LINK_I2C) {
			*reply_payload_len = 0u;
			return STATUS_NOSUPPORT;
		}
		h = (cmd == CMD_I2CM_CONFIG) ? handle_i2cm_config
		    : (cmd == CMD_I2CM_XFER) ? handle_i2cm_xfer
		                             : handle_i2cm_result;
		break;
	case CMD_LINK_FEATURES:
		/* Link-scoped: called directly, not through the table (#130). */
		return handle_link_features(link,
		                            req_payload,
		                            req_payload_len,
		                            reply_payload,
		                            reply_payload_cap,
		                            reply_payload_len);
	/* v0.5 (§2B wave-2) chunked DSP-chain upload (CHAIN_OPEN /
     * STAGE_PUSH / CHAIN_BIND).  The 0x36 tombstone stays in the
     * default branch -- host code SHOULD NOT call it, and any
     * residual caller gets STATUS_NOSUPPORT from the fallthrough. */
	case CMD_ADC_DSP_CHAIN_OPEN:
		h = handle_adc_dsp_chain_open;
		break;
	case CMD_ADC_DSP_STAGE_PUSH:
		h = handle_adc_dsp_stage_push;
		break;
	case CMD_ADC_DSP_CHAIN_BIND:
		h = handle_adc_dsp_chain_bind;
		break;
	default:
		/* Route the reserved OTA opcode range (0xF0..0xFF) through
         * the application bootloader's dispatcher.  Commands 0xF0..0xF6
         * execute only in a partitioned build with an FMC backend; the
         * full-flash/stub builds return STATUS_NOSUPPORT without flash I/O,
         * as do reserved 0xF7..0xFF.  `cmd` is uint8_t so the upper bound
         * 0xFFu is implicit; an explicit check would trip -Wtype-limits. */
		if (cmd >= CMD_OTA_BEGIN) {
			return bl_dispatch_ota(cmd,
			                       req_payload,
			                       req_payload_len,
			                       reply_payload,
			                       reply_payload_cap,
			                       reply_payload_len);
		}
		*reply_payload_len = 0u;
		return STATUS_NOSUPPORT;
	}
	return h(req_payload, req_payload_len, reply_payload, reply_payload_cap, reply_payload_len);
}

gd32_bridge_status_t protocol_dispatch(gd32_bridge_link_t link,
                                       uint8_t            cmd,
                                       const uint8_t     *req_payload,
                                       size_t             req_payload_len,
                                       uint8_t           *reply_payload,
                                       size_t             reply_payload_cap,
                                       size_t            *reply_payload_len)
{
	if (atomic_flag_test_and_set_explicit(&dispatch_in_flight, memory_order_acquire)) {
		*reply_payload_len = 0u;
		return STATUS_BUSY;
	}

	/* Only a handler that succeeds writes a length; every error answers an
	 * empty payload, whatever the caller's `*reply_payload_len` held. */
	*reply_payload_len                = 0u;
	const gd32_bridge_status_t status = protocol_dispatch_inner(link,
	                                                            cmd,
	                                                            req_payload,
	                                                            req_payload_len,
	                                                            reply_payload,
	                                                            reply_payload_cap,
	                                                            reply_payload_len);
	atomic_flag_clear_explicit(&dispatch_in_flight, memory_order_release);
	return status;
}
