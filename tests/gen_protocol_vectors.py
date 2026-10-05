#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""
Regenerate tests/protocol_vectors.txt.

This script is the authoritative source-of-truth for the canonical
wire vectors consumed by BOTH sides of the bridge protocol, which now
live in two repositories:

  * the firmware-side tests in this repo, tests/protocol_vectors.txt
  * the host-side driver tests in alp-sdk, under
    tests/zephyr/chips/gd32g553/

That split is why regenerating matters: a wire change made on one side
has no local consumer that would notice the other side drifting. CI
(.github/workflows/ci.yml) runs this script and fails if the committed
tests/protocol_vectors.txt is not what it produces.

It computes the CRC bytes natively (no external dependency) and
emits exactly the format the consumers expect: one `<name> = <hex>`
vector per line, comment lines start with `#`.

Run from anywhere -- the output path is resolved relative to this
file, not to the working directory:

    python3 tests/gen_protocol_vectors.py

The output file is fully regenerated -- diff against git to spot
unexpected wire changes.
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys


def crc16_ccitt_false(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, non-reflected, xor-out 0)."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def spi_frame(framing_byte: int, op_or_status: int, payload: bytes = b"") -> bytes:
    """Build a SPI envelope: `SOF | CMD-or-STATUS | PAYLOAD | CRC(LSB,MSB)`.

    The CRC is the one field in the envelope that is NOT big-endian --
    both transports (transport_spi.c, transport_i2c.c) and the host
    driver (alp-sdk chips/gd32g553/gd32g553.c) emit and parse it low
    byte first, silicon-verified.  See issue #68.
    """
    body = bytes([framing_byte, op_or_status]) + payload
    crc = crc16_ccitt_false(body)
    return body + crc.to_bytes(2, "little")


def i2c_write(cmd: int, payload: bytes = b"") -> bytes:
    """Build an I2C write envelope.

    Wire layout per alp-sdk docs/gd32-bridge-protocol.md §5: the host clocks
    `<reg-addr=0x00> CMD PAYLOAD CRC(2)` after the I2C
    `S | ADDR | W` envelope.  CRC covers `CMD | PAYLOAD` only and is
    transmitted low byte first (see spi_frame's docstring, issue #68).
    """
    body = bytes([cmd]) + payload
    crc = crc16_ccitt_false(body)
    return bytes([0x00]) + body + crc.to_bytes(2, "little")


def i2c_read(status: int, payload: bytes = b"") -> bytes:
    """Build an I2C read envelope: `STATUS PAYLOAD CRC(2)` (CRC over `STATUS | PAYLOAD`, low byte first)."""
    body = bytes([status]) + payload
    crc = crc16_ccitt_false(body)
    return body + crc.to_bytes(2, "little")


# ---------------------------------------------------------------------
# Protocol constants -- parsed from src/protocol.h, the wire source of truth.
# ---------------------------------------------------------------------

SOF = 0xA5
PROTOCOL_HEADER = pathlib.Path(__file__).resolve().parent.parent / "src" / "protocol.h"
BOOTLOADER_HEADER = PROTOCOL_HEADER.parent / "bootloader" / "bootloader.h"

_REQUIRED_CMD_NAMES = (
    "CMD_PING",
    "CMD_GET_VERSION",
    "CMD_GET_BUILD_ID",
    "CMD_RESET_REASON",
    "CMD_GPIO_READ",
    "CMD_GPIO_WRITE",
    "CMD_PWM_SET",
    "CMD_PWM_GET",
    "CMD_PWM_CONFIGURE",
    "CMD_ADC_READ",
    "CMD_ADC_CONFIGURE",
    "CMD_ADC_STREAM_BEGIN",
    "CMD_ADC_STREAM_READ",
    "CMD_ADC_STREAM_END",
    "CMD_ADC_STREAM_CONFIGURE_DSP",
    "CMD_ADC_DSP_CHAIN_OPEN",
    "CMD_ADC_DSP_STAGE_PUSH",
    "CMD_ADC_DSP_CHAIN_BIND",
    "CMD_ADC_SPECTRUM_READ",
    "CMD_PWM_CAPTURE_BEGIN",
    "CMD_PWM_CAPTURE_READ",
    "CMD_PWM_CAPTURE_END",
    "CMD_PWM_SINGLE_PULSE",
    "CMD_TIMER_SYNC",
    "CMD_POWER_MODE_SET",
    "CMD_TRNG_READ",
    "CMD_TMU_COMPUTE",
    "CMD_DAC_SET",
    "CMD_DAC_GET",
    "CMD_QENC_READ",
    "CMD_QENC_RESET",
    "CMD_COUNTER_READ",
    "CMD_SE_RESET",
    "CMD_DA9292_STATUS_FORWARD",
    "CMD_LINK_FEATURES",
    "CMD_BATCH",
    "CMD_ADC_STREAM_BEGIN2",
    "CMD_ADC_STREAM_READ2",
)
_REQUIRED_STATUS_NAMES = (
    "STATUS_OK",
    "STATUS_INVAL",
    "STATUS_NOT_READY",
    "STATUS_IO",
    "STATUS_NOSUPPORT",
)
_REQUIRED_OTA_CMD_NAMES = (
    "CMD_OTA_BEGIN",
    "CMD_OTA_WRITE_CHUNK",
    "CMD_OTA_VERIFY",
    "CMD_OTA_COMMIT",
    "CMD_OTA_ROLLBACK",
    "CMD_OTA_GET_STATE",
    "CMD_OTA_ABORT",
)


def _strip_c_comments(text: str) -> str:
    """Remove C block and line comments before parsing enum declarations."""
    return re.sub(r"/\*.*?\*/|//[^\r\n]*", "", text, flags=re.DOTALL)


def _required_enum_values(
    header_text: str,
    header_path: pathlib.Path,
    enum_name: str,
    required_names: tuple[str, ...],
) -> dict[str, int]:
    """Return required direct-integer enumerators from one named typedef enum."""
    uncommented = _strip_c_comments(header_text)
    enum_match = re.search(
        rf"\btypedef\s+enum(?:\s+[A-Za-z_]\w*)?\s*\{{(?P<body>[^}}]*)\}}\s*{re.escape(enum_name)}\s*;",
        uncommented,
        flags=re.DOTALL,
    )
    if enum_match is None:
        sys.exit(
            f"gen_protocol_vectors.py: could not find typedef enum {enum_name} "
            f"in {header_path}"
        )

    expressions: dict[str, str | None] = {}
    for entry in enum_match.group("body").split(","):
        entry = entry.strip()
        if not entry:
            continue
        item_match = re.fullmatch(
            r"(?P<name>[A-Za-z_]\w*)(?:\s*=\s*(?P<expression>.*))?",
            entry,
            flags=re.DOTALL,
        )
        if item_match is not None:
            name = item_match.group("name")
            if name in expressions:
                sys.exit(
                    f"gen_protocol_vectors.py: duplicate enumerator {enum_name}.{name} "
                    f"in {header_path}"
                )
            expressions[name] = item_match.group("expression")

    missing = [name for name in required_names if name not in expressions]
    if missing:
        sys.exit(
            f"gen_protocol_vectors.py: {enum_name} in {header_path} is missing "
            f"required enumerator(s): {', '.join(missing)}"
        )

    values: dict[str, int] = {}
    integer_literal = re.compile(
        r"(?P<value>0[xX][0-9A-Fa-f]+|0[bB][01]+|0[0-7]*|[1-9][0-9]*)(?:[uUlL]+)?"
    )
    for name in required_names:
        expression = expressions[name]
        literal_match = integer_literal.fullmatch(expression or "")
        if literal_match is None:
            rendered = expression.strip() if expression is not None else "<implicit>"
            sys.exit(
                f"gen_protocol_vectors.py: {enum_name}.{name} in {header_path} "
                f"must be a simple integer enumerator, got {rendered!r}"
            )
        literal = literal_match.group("value")
        base = (
            8
            if len(literal) > 1
            and literal.startswith("0")
            and not literal.lower().startswith(("0x", "0b"))
            else 0
        )
        values[name] = int(literal, base)
    return values


def _required_macro_values(
    header_text: str,
    header_path: pathlib.Path,
    required_names: tuple[str, ...],
) -> dict[str, int]:
    """Return required direct-integer object macros from one header."""
    uncommented = _strip_c_comments(header_text)
    definitions: dict[str, str] = {}
    required = set(required_names)
    for match in re.finditer(
        r"^\s*#\s*define\s+(?P<name>[A-Za-z_]\w*)\s+(?P<expression>[^\r\n]+)",
        uncommented,
        flags=re.MULTILINE,
    ):
        name = match.group("name")
        if name not in required:
            continue
        if name in definitions:
            sys.exit(f"gen_protocol_vectors.py: duplicate macro {name} in {header_path}")
        definitions[name] = match.group("expression").strip()

    missing = [name for name in required_names if name not in definitions]
    if missing:
        sys.exit(
            f"gen_protocol_vectors.py: {header_path} is missing required macro(s): "
            f"{', '.join(missing)}"
        )

    values: dict[str, int] = {}
    integer_literal = re.compile(
        r"(?P<value>0[xX][0-9A-Fa-f]+|0[bB][01]+|0[0-7]*|[1-9][0-9]*)(?:[uUlL]+)?"
    )
    for name in required_names:
        expression = definitions[name]
        literal_match = integer_literal.fullmatch(expression)
        if literal_match is None:
            sys.exit(
                f"gen_protocol_vectors.py: {name} in {header_path} must be a simple "
                f"integer macro, got {expression!r}"
            )
        literal = literal_match.group("value")
        base = (
            8
            if len(literal) > 1
            and literal.startswith("0")
            and not literal.lower().startswith(("0x", "0b"))
            else 0
        )
        values[name] = int(literal, base)
    return values


def _protocol_constants_from_headers() -> dict[str, int]:
    try:
        header_text = PROTOCOL_HEADER.read_text(encoding="utf-8")
    except OSError as exc:
        sys.exit(f"gen_protocol_vectors.py: cannot read {PROTOCOL_HEADER}: {exc}")
    try:
        bootloader_text = BOOTLOADER_HEADER.read_text(encoding="utf-8")
    except OSError as exc:
        sys.exit(f"gen_protocol_vectors.py: cannot read {BOOTLOADER_HEADER}: {exc}")

    return {
        **_required_enum_values(
            header_text, PROTOCOL_HEADER, "gd32_bridge_cmd_t", _REQUIRED_CMD_NAMES
        ),
        **_required_enum_values(
            header_text, PROTOCOL_HEADER, "gd32_bridge_status_t", _REQUIRED_STATUS_NAMES
        ),
        # OTA is deliberately owned by the bootloader header rather than
        # gd32_bridge_cmd_t; parse that real source instead of copying it.
        **_required_macro_values(bootloader_text, BOOTLOADER_HEADER, _REQUIRED_OTA_CMD_NAMES),
    }


globals().update(_protocol_constants_from_headers())

# Firmware-declared version triple. Parsed out of src/protocol.h at run
# time -- not a private literal -- because a hard-coded copy is exactly
# what let this drift from 0.9.0 to 0.8.0 unnoticed (#22): the
# regenerate-and-diff gate compares this file against itself and stays
# green even when the constant is stale. Do not "simplify" this back
# into a tuple; that reopens #22.
def _fw_version_from_header() -> tuple[int, int, int]:
    try:
        text = PROTOCOL_HEADER.read_text(encoding="utf-8")
    except OSError as exc:
        sys.exit(f"gen_protocol_vectors.py: cannot read {PROTOCOL_HEADER}: {exc}")
    parts = []
    for field in ("MAJOR", "MINOR", "PATCH"):
        m = re.search(rf"#define\s+PROTOCOL_VERSION_{field}\s+(\d+)[uU]?\b", text)
        if m is None:
            sys.exit(
                f"gen_protocol_vectors.py: could not parse "
                f"PROTOCOL_VERSION_{field} out of {PROTOCOL_HEADER}"
            )
        parts.append(int(m.group(1)))
    return (parts[0], parts[1], parts[2])


FW_VERSION = _fw_version_from_header()


HEADER = """# gd32-bridge canonical wire-test vectors
#
# Consumed by this repo's own firmware-side consumer,
# tests/unit/protocol_vectors/ -- which drives MOST (not all) of these
# vectors through the real transport_spi.c / transport_i2c.c -> protocol.c,
# on the stub HAL backend, and asserts the emitted bytes.  See
# tests/unit/protocol_vectors/src/test_protocol_vectors.c's file header for
# exactly which vectors it reaches, and which it does not and why (five
# armed-OTA reply vectors + one fake-HAL-only spectrum reply, all needing a
# link target this suite deliberately does not stitch together).
#
# There is no alp-sdk test consumer of this file; alp-sdk's own
# docs/gd32-bridge-protocol.md says as much.  A wire change still needs a
# matching alp-sdk change -- the GD32G553_OTA_MIN_PROTOCOL_MINOR /
# GD32G553_REG_ON_MIN_PROTOCOL_MINOR gates and the per-opcode version notes
# in that doc -- see CONTRIBUTING.md's "A wire change still needs a matching
# alp-sdk change" section.
#
# Format: one vector per non-comment line, `<name> = <hex>` where
# <hex> is a sequence of byte values with no separators.  Whitespace
# is ignored; lines beginning with `#` are comments.
#
# Vectors below are regenerated by `python3 tests/gen_protocol_vectors.py`
# any time the framing layer changes.  Re-run the generator and commit
# both this file and the corresponding driver / firmware updates in
# the same change.
"""


class _Section:
    """A section boundary carried in the vector stream by build_vectors().

    Holding the boundary in the stream is the whole point: section membership
    can no longer be derived from a vector's position, so inserting or removing
    a vector cannot silently move a later section's start.
    """

    __slots__ = ("header",)

    def __init__(self, header: list[str]) -> None:
        self.header = header


def build_vectors() -> list[tuple[str, str, str | None] | _Section]:
    """Return [_Section | (name, hex_value, comment_or_none)] in emission order."""
    out: list[tuple[str, str, str | None] | _Section] = []

    # ----- §1. Foundational CRC-16/CCITT-FALSE vector ----------------
    out.append(_Section([
        "§1. Foundational CRC-16/CCITT-FALSE vector",
    ]))
    out.append((
        "crc16_ccitt_false_ref_string",
        b"123456789".hex().upper(),
        '"123456789" (ASCII) -- the canonical CRC-16/CCITT-FALSE test input',
    ))
    crc_ref = crc16_ccitt_false(b"123456789")
    out.append((
        "crc16_ccitt_false_ref_result",
        f"{crc_ref:04X}",
        "expected output: 0x29B1 (universally cited)",
    ))
    out.append((
        "crc16_ccitt_false_test_string",
        b"1234567890".hex().upper(),
        "helper: extra trailing '0' catches truncated walks",
    ))

    # ----- §2. SPI envelopes -----------------------------------------
    out.append(_Section([
        "§2. SPI envelopes -- two-transaction request / reply pattern",
    ]))
    out.append((
        "spi_ping_request",
        spi_frame(SOF, CMD_PING).hex().upper(),
        "SOF | CMD=0x00 (PING) | CRC -- empty payload",
    ))
    out.append((
        "spi_ping_reply_ok",
        spi_frame(SOF, STATUS_OK).hex().upper(),
        "SOF | STATUS=0x00 (OK) | CRC -- empty payload",
    ))
    out.append((
        "spi_get_version_request",
        spi_frame(SOF, CMD_GET_VERSION).hex().upper(),
        "SOF | CMD=0x01 (GET_VERSION) | CRC -- empty payload",
    ))
    out.append((
        f"spi_get_version_reply_v{FW_VERSION[0]}_{FW_VERSION[1]}_{FW_VERSION[2]}",
        spi_frame(SOF, STATUS_OK, bytes(FW_VERSION)).hex().upper(),
        f"SOF | STATUS=0x00 | major={FW_VERSION[0]} | minor={FW_VERSION[1]}"
        f" | patch={FW_VERSION[2]} | CRC",
    ))

    # ----- §3. I2C envelopes -----------------------------------------
    out.append(_Section([
        "§3. I2C envelopes -- register-style framing",
    ]))
    out.append((
        "i2c_ping_write",
        i2c_write(CMD_PING).hex().upper(),
        "regaddr=0x00 | CMD=0x00 | CRC -- CRC excludes the regaddr byte",
    ))
    out.append((
        "i2c_ping_read_ok",
        i2c_read(STATUS_OK).hex().upper(),
        "STATUS=0x00 | CRC -- empty payload",
    ))

    # ----- §4. v0.2 additions: DAC / QENC / COUNTER ------------------
    out.append(_Section([
        "§4. v0.2 additions -- DAC, quadrature encoder, free-running counter",
    ]))
    # Request envelopes (host -> firmware).  Stub firmware replies
    # NOSUPPORT (0x06) for every body; the gd32 backend's real
    # bridge_hw_*_set / *_read / *_reset live under hal/gd32/.  The
    # vectors let both sides assert that the framing layer is
    # byte-for-byte locked independently of the HAL bodies.
    out.append((
        "spi_dac_set_ch0_1650mv_request",
        spi_frame(SOF, CMD_DAC_SET,
                  bytes([0x00, 0x00, 0x72, 0x06])).hex().upper(),
        "SOF | CMD=0x50 | channel=0 | resv=0 | value_mv=1650 (LE 0x0672) | CRC",
    ))
    out.append((
        "spi_dac_get_ch1_request",
        spi_frame(SOF, CMD_DAC_GET, bytes([0x01])).hex().upper(),
        "SOF | CMD=0x51 | channel=1 | CRC",
    ))
    out.append((
        "spi_qenc_read_ch0_request",
        spi_frame(SOF, CMD_QENC_READ, bytes([0x00])).hex().upper(),
        "SOF | CMD=0x60 | encoder=0 | CRC",
    ))
    out.append((
        "spi_qenc_reset_ch3_request",
        spi_frame(SOF, CMD_QENC_RESET, bytes([0x03])).hex().upper(),
        "SOF | CMD=0x61 | encoder=3 | CRC",
    ))
    out.append((
        "spi_counter_read_ch0_request",
        spi_frame(SOF, CMD_COUNTER_READ, bytes([0x00])).hex().upper(),
        "SOF | CMD=0x70 | counter=0 | CRC",
    ))
    out.append((
        "spi_se_reset_assert_request",
        spi_frame(SOF, CMD_SE_RESET, bytes([0x01])).hex().upper(),
        "SOF | CMD=0x41 | assert=1 (hold OPTIGA Trust M in reset) | CRC",
    ))
    out.append((
        "spi_se_reset_release_request",
        spi_frame(SOF, CMD_SE_RESET, bytes([0x00])).hex().upper(),
        "SOF | CMD=0x41 | assert=0 (release the SE) | CRC",
    ))
    out.append((
        "spi_reply_nosupport",
        spi_frame(SOF, STATUS_NOSUPPORT).hex().upper(),
        "SOF | STATUS=0x06 (NOSUPPORT) | empty payload | CRC --"
        " stub-firmware reply for any v0.2+ opcode whose HAL body is"
        " not yet wired",
    ))

    # ----- §5. v0.3 additions: GD32G5 HW knobs -----------------------
    out.append(_Section([
        "§5. v0.3 additions -- GD32G5 HW knobs (PWM_CONFIGURE, ADC_CONFIGURE,",
        "                      ADC_STREAM_BEGIN / READ / END, TRNG_READ)",
    ]))
    # Sticky-configure opcodes + DMA-backed ADC streaming.  Firmware
    # auto-selects HRPWM when achievable, so no separate
    # CMD_PWM_SET_HIGHRES -- the existing CMD_PWM_SET routes there
    # when the period demands sub-4-ns precision.
    out.append((
        "spi_pwm_configure_ch0_request",
        spi_frame(SOF, CMD_PWM_CONFIGURE,
                  bytes([0x00,             # channel
                         0x01,             # align_mode = CENTER_UP
                         0xD0, 0x07,       # dead_time_ns = 2000 (LE 0x000007D0)
                         0x00, 0x00,
                         0x01,             # break_cfg bit 0 = enable
                  ])).hex().upper(),
        "SOF | CMD=0x22 | channel=0 | align=CENTER_UP | dead_time_ns=2000 | break_en | CRC",
    ))
    out.append((
        "spi_adc_configure_ch3_request",
        spi_frame(SOF, CMD_ADC_CONFIGURE,
                  bytes([0x03,             # channel
                         0x00,             # reserved
                         0x10, 0x00,       # oversample_ratio = 16 (LE 0x0010)
                         0x5F, 0x00,       # sample_cycles = 95 (~92.5 cycles, LE 0x005F)
                         0x10,             # resolution = 16 bits
                  ])).hex().upper(),
        "SOF | CMD=0x32 | channel=3 | oversample=16 | sample_cycles~92 | resolution=16 | CRC",
    ))
    out.append((
        "spi_adc_stream_begin_stream0_ch0_1ksps_request",
        spi_frame(SOF, CMD_ADC_STREAM_BEGIN,
                  bytes([0x00,             # stream_id
                         0x00,             # channel
                         0x00,             # reserved
                         0xE8, 0x03, 0x00, 0x00,  # sample_rate_hz = 1000 (LE)
                  ])).hex().upper(),
        "SOF | CMD=0x33 | stream_id=0 | channel=0 | rate_hz=1000 | CRC",
    ))
    out.append((
        "spi_adc_stream_read_stream0_max32_request",
        spi_frame(SOF, CMD_ADC_STREAM_READ,
                  bytes([0x00, 0x20])).hex().upper(),
        "SOF | CMD=0x34 | stream_id=0 | max_samples=32 | CRC --"
        " reply length is 1 + max_samples*2 + 2 (CRC) bytes regardless"
        " of how many samples the firmware actually returns",
    ))
    out.append((
        "spi_adc_stream_end_stream0_request",
        spi_frame(SOF, CMD_ADC_STREAM_END, bytes([0x00])).hex().upper(),
        "SOF | CMD=0x35 | stream_id=0 | CRC",
    ))
    out.append((
        "spi_trng_read_16_request",
        spi_frame(SOF, CMD_TRNG_READ, bytes([0x10])).hex().upper(),
        "SOF | CMD=0x80 | request 16 bytes of true random | CRC --"
        " host receives 16 bytes of GD32G5 TRNG (NIST SP800-90B)",
    ))

    # ----- §6. v0.4 additions: TMU (CORDIC) math accelerator --------
    out.append(_Section([
        "§6. v0.4 additions -- GD32G5 TMU (CORDIC) math accelerator",
    ]))
    # Single representative vector: alp_tmu_sqrt(4.0f) -> 2.0f as
    # encoded on the wire.  Function = SQRT (5); format = IEEE-754
    # single (1); in_a = 0x40800000 (float bits for 4.0f); in_b = 0.
    # Firmware stub replies STATUS_NOSUPPORT; the gd32 backend's TMU
    # body lives in hal/gd32/tmu.c, mirroring the existing v0.3 pattern.
    out.append((
        "spi_tmu_compute_sqrt_f32_4p0_request",
        spi_frame(SOF, CMD_TMU_COMPUTE,
                  bytes([0x05,                           # function = SQRT
                         0x01,                           # format = IEEE-754
                         0x00, 0x00,                     # reserved
                         0x00, 0x00, 0x80, 0x40,         # in_a = 4.0f (LE)
                         0x00, 0x00, 0x00, 0x00,         # in_b = 0
                  ])).hex().upper(),
        "SOF | CMD=0x90 | function=SQRT | format=F32 | in_a=4.0f (LE 0x40800000) |"
        " in_b=0 | CRC -- bridge replies 2.0f as 4 reply payload bytes"
        " + STATUS",
    ))

    # ----- §7. v0.5 additions: ADC-stream DSP pipeline (reserved) ---
    out.append(_Section([
        "§7. v0.5 additions -- ADC-stream DSP pipeline (reserved opcode)",
    ]))
    # The CMD_ADC_STREAM_CONFIGURE_DSP opcode is RESERVED at v0.5.0
    # for the wave-2 bridge-wired surfaces alp_adc_filter_t /
    # alp_adc_spectrum_t (see <alp/adc.h>, ships in v0.5.x).  The
    # firmware dispatcher returns STATUS_NOSUPPORT for this opcode
    # today via the default branch.  The standalone <alp/dsp.h> API
    # (chain.open / .apply_samples / .apply_bins) ships in v0.5.0
    # without using this opcode -- it runs the chain locally.
    # The wire vector below carries an empty payload (host probing
    # the opcode); the eventual wave-2 wire format will land with
    # the v0.5.x sub-commits.
    out.append((
        "spi_adc_stream_configure_dsp_probe_request",
        spi_frame(SOF, CMD_ADC_STREAM_CONFIGURE_DSP).hex().upper(),
        "SOF | CMD=0x36 | (no payload yet) | CRC -- v0.5 reserved opcode;"
        " firmware replies STATUS_NOSUPPORT via the default-case dispatch"
        " until the wave-2 wire payload format finalises in v0.5.x",
    ))

    # ----- §8. v0.5 additions (§2B.2): advanced timer extras --------
    out.append(_Section([
        "§8. v0.5 additions (§2B.2) -- advanced timer extras",
    ]))
    # CMD_PWM_CAPTURE_{BEGIN, READ, END}, CMD_PWM_SINGLE_PULSE, and
    # CMD_TIMER_SYNC are implemented by the GD32 HAL.  Representative
    # vectors below pin their request framing; the protocol-vector host
    # suite deliberately links the generic stub HAL, so its round-trip
    # checks still expect STATUS_NOSUPPORT for these hardware operations.
    out.append((
        "spi_pwm_capture_begin_probe_request",
        spi_frame(SOF, CMD_PWM_CAPTURE_BEGIN,
                  bytes([0x00,                            # channel = 0
                         0x02,                            # edge = BOTH
                  ])).hex().upper(),
        "SOF | CMD=0x23 | channel=0 | edge=BOTH | CRC -- dispatched to"
        " handle_pwm_capture_begin() (protocol.c); production body in"
        " hal/gd32/pwm_capture.c (bridge_hw_pwm_capture_begin).  The"
        " stub HAL backend still answers STATUS_NOSUPPORT"
        " (BRIDGE_HW_ERR_NOTIMPL)",
    ))
    out.append((
        "spi_pwm_single_pulse_probe_request",
        spi_frame(SOF, CMD_PWM_SINGLE_PULSE,
                  bytes([0x00,                            # channel = 0
                         0x00, 0x00, 0x00,                # reserved
                         0xE8, 0x03, 0x00, 0x00,          # pulse_ns = 1000 (LE)
                  ])).hex().upper(),
        "SOF | CMD=0x26 | channel=0 | pulse_ns=1000 (LE 0x000003E8) | CRC"
        " -- dispatched to handle_pwm_single_pulse() (protocol.c);"
        " production body in hal/gd32/pwm.c (bridge_hw_pwm_single_pulse)."
        "  The stub HAL backend still answers STATUS_NOSUPPORT"
        " (BRIDGE_HW_ERR_NOTIMPL)",
    ))

    # ----- §9. v0.5 additions (§2B.3): system power-mode set ---------
    out.append(_Section([
        "§9. v0.5 additions (§2B.3) -- system power-mode set",
    ]))
    # CMD_POWER_MODE_SET is RESERVED at v0.5 for the host->supervisor
    # sleep-transition request.  Portable surface lives in
    # <alp/power.h>; firmware HAL body lands in a follow-up drop.
    # Representative probe payload encodes the four-field request
    # shape that the eventual firmware-side handler will decode:
    # mode (DEEP_SLEEP), reserved, wake_bitmap (RTC | GPIO), and a
    # wake_after_ms ceiling.
    out.append((
        "spi_power_mode_set_probe_request",
        spi_frame(SOF, CMD_POWER_MODE_SET,
                  bytes([0x02,                            # mode = DEEP_SLEEP
                         0x00,                            # reserved
                         0x03, 0x00, 0x00, 0x00,          # wake_bitmap = RTC|GPIO (LE)
                         0x10, 0x27, 0x00, 0x00,          # wake_after_ms = 10000 (LE)
                  ])).hex().upper(),
        "SOF | CMD=0x28 | mode=DEEP_SLEEP | wake_bitmap=RTC|GPIO |"
        " wake_after_ms=10000 (LE 0x00002710) | CRC -- dispatched to"
        " handle_power_mode_set() (protocol.c); production body in"
        " hal/gd32/power.c (bridge_hw_power_mode_set).  The stub HAL"
        " backend still answers STATUS_NOSUPPORT (BRIDGE_HW_ERR_NOTIMPL)",
    ))

    # Request byte 1 is the POWER_FLAG_* byte (0 from a host that predates
    # it).  WAKE_I2C (0x01) adds an early wake on a BRD_I2C address match to a
    # TIMED Deep-sleep (an untimed one answers STATUS_OUT_OF_RANGE).
    out.append((
        "spi_power_mode_set_deepsleep_wake_i2c_request",
        spi_frame(SOF, CMD_POWER_MODE_SET,
                  bytes([0x02,                            # mode = DEEP_SLEEP
                         0x01,                            # flags = WAKE_I2C
                         0x00, 0x00, 0x00, 0x00,          # wake_bitmap = none
                         0x64, 0x00, 0x00, 0x00,          # wake_after_ms = 100 (LE)
                  ])).hex().upper(),
        "SOF | CMD=0x28 | mode=DEEP_SLEEP | flags=WAKE_I2C | wake_bitmap=0 |"
        " wake_after_ms=100 | CRC -- answers STATUS_BUSY while an ADC stream,"
        " PWM, DAC or OTA session is live; an untimed WAKE_I2C request answers"
        " STATUS_OUT_OF_RANGE",
    ))

    # ----- §10. v0.5 additions (§2B wave-2): chunked DSP-chain upload -
    out.append(_Section([
        "§10. v0.5 additions (§2B wave-2) -- chunked DSP-chain upload",
        "      (CHAIN_OPEN / STAGE_PUSH / CHAIN_BIND)",
    ]))
    # CMD_ADC_DSP_{CHAIN_OPEN, STAGE_PUSH, CHAIN_BIND} are RESERVED at
    # v0.5 for the wave-2 bridge-wired DSP pipeline that lets raw ADC
    # samples never traverse the wire when filtered or spectral data
    # is what the customer wants.  Portable surfaces in <alp/adc.h>
    # (alp_adc_filter_t / alp_adc_spectrum_t) and <alp/dsp.h>
    # (alp_dsp_chain_t).  Wire format documented in
    # alp-sdk docs/gd32-bridge-protocol.md §3.x.  Firmware default-case
    # dispatch returns STATUS_NOSUPPORT for all three opcodes until
    # the bridge_hw_adc_dsp_* HAL bodies land.  Representative probe
    # vectors below: CHAIN_OPEN with no payload, STAGE_PUSH carrying
    # one WINDOW stage (shape=Hann) in a single 4-byte chunk, and
    # CHAIN_BIND attaching chain_id 0 to stream_id 0.
    out.append((
        "spi_adc_dsp_chain_open_probe_request",
        spi_frame(SOF, CMD_ADC_DSP_CHAIN_OPEN).hex().upper(),
        "SOF | CMD=0x37 | (no payload) | CRC -- dispatched to"
        " handle_adc_dsp_chain_open() (protocol.c); production body in"
        " hal/gd32/adc_stream.c (bridge_hw_adc_dsp_chain_open).  Reply"
        " payload is chain_id:u8.  The stub HAL backend still answers"
        " STATUS_NOSUPPORT (BRIDGE_HW_ERR_NOTIMPL)",
    ))
    out.append((
        "spi_adc_dsp_stage_push_window_hann_request",
        spi_frame(SOF, CMD_ADC_DSP_STAGE_PUSH,
                  bytes([0x00,                            # chain_id = 0
                         0x00,                            # stage_index = 0
                         0x02,                            # kind = WINDOW
                         0x00, 0x00,                      # chunk_offset = 0 (LE)
                         0x04, 0x00,                      # chunk_total_size = 4 (LE)
                         0x01,                            # shape = Hann
                         0x00, 0x00, 0x00,                # reserved[3]
                  ])).hex().upper(),
        "SOF | CMD=0x38 | chain_id=0 | stage_index=0 | kind=WINDOW(2) |"
        " chunk_offset=0 | chunk_total_size=4 | shape=Hann |"
        " reserved[3] | CRC -- dispatched to handle_adc_dsp_stage_push()"
        " (protocol.c); production body in hal/gd32/adc_stream.c"
        " (bridge_hw_adc_dsp_stage_push).  The stub HAL backend still"
        " answers STATUS_NOSUPPORT (BRIDGE_HW_ERR_NOTIMPL)",
    ))
    out.append((
        "spi_adc_dsp_chain_bind_probe_request",
        spi_frame(SOF, CMD_ADC_DSP_CHAIN_BIND,
                  bytes([0x00,                            # chain_id = 0
                         0x00,                            # stream_id = 0
                  ])).hex().upper(),
        "SOF | CMD=0x39 | chain_id=0 | stream_id=0 | CRC -- dispatched"
        " to handle_adc_dsp_chain_bind() (protocol.c); production body"
        " in hal/gd32/adc_stream.c (bridge_hw_adc_dsp_chain_bind).  The"
        " stub HAL backend still answers STATUS_NOSUPPORT"
        " (BRIDGE_HW_ERR_NOTIMPL)",
    ))

    # ----- §11. OTA Path-A opcodes (0xF0..0xF6) ----------------------
    out.append(_Section([
        "§11. OTA Path-A opcodes (0xF0..0xF6) -- in-system upgrade over the",
        "      bridge (alp-sdk docs/gd32-bridge-protocol.md §10 Path A).  Unarmed",
        "      firmware replies STATUS_NOSUPPORT to every OTA opcode.",
    ]))
    # Payload layouts per alp-sdk docs/gd32-bridge-protocol.md §10 / src/ota.c.
    # Unarmed firmware (no -DBRIDGE_OTA_PARTITIONED) replies
    # STATUS_NOSUPPORT to every OTA opcode; the vectors lock the
    # request framing and the armed-firmware reply layouts.
    out.append((
        "spi_ota_begin_request",
        spi_frame(SOF, CMD_OTA_BEGIN,
                  bytes([0xF8, 0xA3, 0x00, 0x00,          # size = 41976 (LE)
                         0xEF, 0xBE, 0xAD, 0xDE,          # expected_crc32 (LE)
                  ])).hex().upper(),
        "SOF | CMD=0xF0 | size=41976 (LE 0x0000A3F8) |"
        " expected_crc32=0xDEADBEEF (LE) | CRC",
    ))
    out.append((
        "spi_ota_begin_request_v0_7",
        spi_frame(SOF, CMD_OTA_BEGIN,
                  bytes([0xF8, 0xA3, 0x00, 0x00,          # size = 41976 (LE)
                         0xEF, 0xBE, 0xAD, 0xDE,          # expected_crc32 (LE)
                         0x00, 0x02, 0x09,                # fw_version 0.2.9
                  ])).hex().upper(),
        "SOF | CMD=0xF0 | size=41976 (LE) | expected_crc32=0xDEADBEEF (LE)"
        " | fw_major=0 | fw_minor=2 | fw_patch=9 | CRC -- the v0.7"
        " ADDITIVE form: the triple lands in the A/B metadata at COMMIT"
        " (fw_version[slot], 0 = unknown); pre-v0.7 firmware ignores the"
        " 3 trailing bytes, and the 8-byte legacy form stays valid",
    ))
    out.append((
        "spi_ota_begin_reply_slot_b",
        spi_frame(SOF, STATUS_OK,
                  bytes([0x38, 0x00,                      # chunk_max = 56 (LE)
                         0x01,                            # target_slot = B (this
                                                           # example is a slot-A
                                                           # -resident build)
                  ])).hex().upper(),
        "SOF | STATUS=0x00 | chunk_max=56 (LE 0x0038) | target_slot=B(1) |"
        " CRC -- 56 = (MAX_PAYLOAD(65) - offset:u32 - len:u8 header) rounded down to the 8-byte flash program granule."
        " target_slot is ota.c's compile-time OTA_RUNNING_SLOT complement"
        " (#3), a BUILD property: this vector's value B is only what a"
        " slot-A-resident build replies (its own non-running slot); a"
        " slot-B-resident build replies A for the identical request",
    ))
    out.append((
        "spi_ota_write_chunk_off0_8b_request",
        spi_frame(SOF, CMD_OTA_WRITE_CHUNK,
                  bytes([0x00, 0x00, 0x00, 0x00,          # offset = 0 (LE)
                         0x08,                            # len = 8 (v0.6 cross-check)
                         0x01, 0x02, 0x03, 0x04,          # data[8] (8-byte
                         0x05, 0x06, 0x07, 0x08,          # doubleword granule)
                  ])).hex().upper(),
        "SOF | CMD=0xF1 | offset=0 (LE) | len=8 | data=01..08 (one FMC"
        " doubleword) | CRC -- the len byte rejects transaction-merged"
        " zero-extended captures that survive the span CRC via the"
        " palindromic-CRC self-consumption hole; offsets pace on 8-byte"
        " boundaries",
    ))
    out.append((
        "spi_ota_write_chunk_reply_8b",
        spi_frame(SOF, STATUS_OK,
                  bytes([0x08, 0x00, 0x00, 0x00])).hex().upper(),
        "SOF | STATUS=0x00 | received_bytes=8 (LE, cumulative high-water) | CRC",
    ))
    out.append((
        "spi_ota_verify_request",
        spi_frame(SOF, CMD_OTA_VERIFY).hex().upper(),
        "SOF | CMD=0xF2 | (no payload -- CRC was supplied at BEGIN) | CRC",
    ))
    out.append((
        "spi_ota_verify_reply_match",
        spi_frame(SOF, STATUS_OK,
                  bytes([0xEF, 0xBE, 0xAD, 0xDE,          # computed_crc32 (LE)
                         0x01,                            # verified = 1
                  ])).hex().upper(),
        "SOF | STATUS=0x00 | computed_crc32=0xDEADBEEF (LE) | verified=1 | CRC",
    ))
    out.append((
        "spi_ota_commit_request",
        spi_frame(SOF, CMD_OTA_COMMIT).hex().upper(),
        "SOF | CMD=0xF3 | (no payload; firmware resets on success) | CRC",
    ))
    out.append((
        "spi_ota_rollback_request",
        spi_frame(SOF, CMD_OTA_ROLLBACK).hex().upper(),
        "SOF | CMD=0xF4 | (no payload; firmware resets on success) | CRC",
    ))
    out.append((
        "spi_ota_get_state_request",
        spi_frame(SOF, CMD_OTA_GET_STATE).hex().upper(),
        "SOF | CMD=0xF5 | (no payload) | CRC",
    ))
    out.append((
        "spi_ota_get_state_reply_ready",
        spi_frame(SOF, STATUS_OK,
                  bytes([0x01,                            # state = READY
                         0x00,                            # active = A (build-
                                                           # derived running
                                                           # slot, see below)
                         0x01,                            # pending_slot = B
                         0x01, 0x00,                      # boot_count = 1 (LE)
                         0x00,                            # err = NONE (gh#101)
                  ])).hex().upper(),
        "SOF | STATUS=0x00 | state=READY(1) | active=A(0) | pending=B(1) |"
        " boot_count=1 (LE, metadata generation) | err=NONE(0) (gh#101) | CRC"
        " -- pending=0xFF when"
        " no session is open. active is ota.c's compile-time OTA_RUNNING_SLOT"
        " (#3), a BUILD property reporting the slot THIS firmware executes"
        " from -- NOT metadata's active_slot field.  The two normally agree,"
        " but diverge across the bootloader's newest-first fallback"
        " (boot_main.c), where metadata can legitimately name a slot that"
        " is not running; this byte is what lets the host observe that"
        " divergence.  This example value (A) is a slot-A-resident build",
    ))
    out.append((
        "spi_ota_get_state_reply_error_verify_crc",
        spi_frame(SOF, STATUS_OK,
                  bytes([0x04,          # state = ERROR
                         0x00,          # active = A
                         0xFF,          # pending = none
                         0x01, 0x00,    # boot_count = 1
                         0x05,          # err = VERIFY_CRC (gh#101)
                  ])).hex().upper(),
        "SOF | STATUS=0x00 | state=ERROR(4) | active=A(0) | pending=none(0xFF) |"
        " boot_count=1 | err=VERIFY_CRC(0x05) (gh#101: the failure cause that"
        " used to be written in nine places and read in none) | CRC.  The err"
        " byte is BRIDGE_OTA_ERR_* in protocol.h; cleared by OTA_ABORT.",
    ))
    out.append((
        "spi_ota_abort_request",
        spi_frame(SOF, CMD_OTA_ABORT).hex().upper(),
        "SOF | CMD=0xF6 | (no payload) | CRC",
    ))
    out.append((
        "spi_ota_reply_not_ready",
        spi_frame(SOF, STATUS_NOT_READY).hex().upper(),
        "SOF | STATUS=0x02 (NOT_READY) | CRC -- WRITE_CHUNK / VERIFY"
        " without a BEGIN-opened session, or COMMIT before VERIFY",
    ))

    # ----- §12. v0.7 additions: link-feature negotiation -------------
    out.append(_Section([
        "§12. v0.7 additions -- link-feature negotiation (CMD_LINK_FEATURES)",
        "      + the STATUS_SEQ stamped-reply framing (SPI only)",
    ]))
    # CMD_LINK_FEATURES (0x81) + the STATUS_SEQ stamped-reply framing.
    # The stamp value is per-session state, so the canonical vectors fix
    # an EXAMPLE stamp; the unstamped reply is simultaneously the exact
    # I2C wire shape (I2C never stamps -- STATUS_NO_PENDING owns bit 7).
    out.append((
        "spi_link_features_request",
        spi_frame(SOF, CMD_LINK_FEATURES, bytes([0x01])).hex().upper(),
        "SOF | CMD=0x81 (LINK_FEATURES) | features=0x01 (STATUS_SEQ"
        " wanted) | CRC -- pre-v0.7 firmware answers STATUS_NOSUPPORT",
    ))
    out.append((
        "spi_link_features_reply_granted_seq1",
        spi_frame(SOF, 0x10 | STATUS_OK, bytes([0x01])).hex().upper(),
        "SOF | STATUS=0x10 (code OK, stamp=1 -- the firmware arms the"
        " feature BEFORE staging, so the negotiation reply itself is"
        " stamped and the host baselines from it) | granted=0x01 | CRC",
    ))
    out.append((
        "spi_ping_reply_ok_seq5",
        spi_frame(SOF, 0x50 | STATUS_OK).hex().upper(),
        "SOF | STATUS=0x50 (code OK, stamp=5) | CRC -- example stamped"
        " reply: code = STATUS & 0x0F, stamp = STATUS >> 4; a reply whose"
        " stamp equals the previously accepted one is a STALE re-serve",
    ))

    # ----- §13. Day-one opcode coverage (#31 E4) ----------------------
    out.append(_Section([
        "§13. Day-one opcode coverage (#31 E4) -- GET_BUILD_ID, RESET_REASON,",
        "      GPIO_READ/WRITE, legacy PWM_SET/GET, ADC_READ,",
        "      DA9292_STATUS_FORWARD",
    ]))
    # CMD_GET_BUILD_ID, CMD_RESET_REASON, CMD_GPIO_{READ,WRITE},
    # CMD_PWM_{SET,GET}, CMD_ADC_READ and CMD_DA9292_STATUS_FORWARD
    # predate the versioned §4+ additions above but had no wire vector
    # until now.  GPIO_READ/WRITE still share one reply vector
    # (spi_reply_io): handle_gpio_read/write (protocol.c) are out of
    # scope for #23 and still fall through to
    # `if (rv < 0) return STATUS_IO;` on any BRIDGE_HW_ERR, so on the
    # stub HAL backend (the only one CI compiles, #31 E1) they answer
    # STATUS_IO, not NOSUPPORT.  PWM_SET/GET and ADC_READ used to share
    # that same STATUS_IO reply but were fixed under #23 to route
    # through status_from_hw() like the rest of the v0.5+ handlers, so
    # their stub-backend reply is spi_reply_nosupport instead (see
    # §14's status_from_hw() note for the pattern).  Their real
    # success-reply payloads carry live GPIO/PWM/ADC state and are not
    # a wire-format constant, so only the request framing is
    # vectorized here.
    out.append((
        "spi_get_build_id_request",
        spi_frame(SOF, CMD_GET_BUILD_ID).hex().upper(),
        "SOF | CMD=0x02 (GET_BUILD_ID) | CRC -- empty payload.  The"
        " success reply is GD32_BRIDGE_BUILD_ID_LEN=20 ASCII bytes"
        " \"<fw-version>+<git-sha-prefix>\", baked at CMake build time"
        " by cmake/gen_build_id.cmake into a generated header (protocol.c)"
        " -- not a fixed wire constant, so no reply vector is given",
    ))
    out.append((
        "spi_reset_reason_request",
        spi_frame(SOF, CMD_RESET_REASON).hex().upper(),
        "SOF | CMD=0x03 (RESET_REASON) | CRC -- empty payload",
    ))
    out.append((
        "spi_reset_reason_reply_unknown",
        spi_frame(SOF, STATUS_OK, bytes([0x00])).hex().upper(),
        "SOF | STATUS=0x00 | reason=0(UNKNOWN) | CRC -- the STUB HAL's"
        " bridge_hw_reset_reason() hardcodes 0u; on real hardware this"
        " reports the reset cause captured at boot, so this"
        " vector pins only the stub-backend value, not a representative"
        " live one",
    ))
    out.append((
        "spi_gpio_read_mask_bit0_request",
        spi_frame(SOF, CMD_GPIO_READ, bytes([0x01, 0x00, 0x00, 0x00])).hex().upper(),
        "SOF | CMD=0x10 | mask=0x00000001 (LE) | CRC -- reply (on the"
        " gd32 backend) is levels:u32(LE); see spi_reply_io for what"
        " the stub backend answers today",
    ))
    out.append((
        "spi_gpio_write_mask_bit0_high_request",
        spi_frame(SOF, CMD_GPIO_WRITE,
                  bytes([0x01, 0x00, 0x00, 0x00,   # mask = bit0 (LE)
                         0x01, 0x00, 0x00, 0x00,   # levels = bit0 high (LE)
                  ])).hex().upper(),
        "SOF | CMD=0x11 | mask=0x00000001 (LE) | levels=0x00000001 (LE)"
        " | CRC -- empty-payload reply on success; see spi_reply_io for"
        " what the stub backend answers today",
    ))
    out.append((
        "spi_pwm_set_ch0_1ms_period_500us_duty_request",
        spi_frame(SOF, CMD_PWM_SET,
                  bytes([0x00, 0x00,               # channel=0, reserved
                         0x40, 0x42, 0x0F, 0x00,   # period_ns = 1_000_000 (LE)
                         0x20, 0xA1, 0x07, 0x00,   # duty_ns   =   500_000 (LE)
                  ])).hex().upper(),
        "SOF | CMD=0x20 | channel=0 | period_ns=1000000 (LE) |"
        " duty_ns=500000 (LE, 50%) | CRC -- empty-payload reply on"
        " success; see spi_reply_nosupport for what the stub backend"
        " answers today (status_from_hw(), #23)",
    ))
    out.append((
        "spi_pwm_get_ch0_request",
        spi_frame(SOF, CMD_PWM_GET, bytes([0x00])).hex().upper(),
        "SOF | CMD=0x21 | channel=0 | CRC -- reply (on the gd32 backend)"
        " is period_ns:u32(LE) duty_ns:u32(LE); see spi_reply_nosupport"
        " for what the stub backend answers today (status_from_hw(),"
        " #23)",
    ))
    out.append((
        "spi_adc_read_ch0_4samples_request",
        spi_frame(SOF, CMD_ADC_READ, bytes([0x00, 0x04])).hex().upper(),
        "SOF | CMD=0x30 | channel=0 | samples=4 | CRC -- reply (on the"
        " gd32 backend) is samples:u8 (echoed) + samples*mv:u16(LE);"
        " see spi_reply_nosupport for what the stub backend answers"
        " today (status_from_hw(), #23)",
    ))
    out.append((
        "spi_da9292_status_forward_request",
        spi_frame(SOF, CMD_DA9292_STATUS_FORWARD).hex().upper(),
        "SOF | CMD=0x40 (DA9292_STATUS_FORWARD) | CRC -- empty payload",
    ))
    out.append((
        "spi_da9292_status_forward_reply_no_sample",
        spi_frame(SOF, STATUS_OK, bytes([0xFF])).hex().upper(),
        "SOF | STATUS=0x00 | status=0xFF (\"no sample available\") | CRC"
        " -- bridge_hw_da9292_status_cached() returns this sentinel on"
        " both the stub HAL and this SoM revision's real hardware (no"
        " DA9292 net reaches the GD32 on this SoM rev; see"
        " hal/bridge_hw_stub.c)",
    ))
    out.append((
        "spi_reply_io",
        spi_frame(SOF, STATUS_IO).hex().upper(),
        "SOF | STATUS=0x05 (IO) | empty payload | CRC -- the reply"
        " handle_gpio_read/write (protocol.c) give on the STUB HAL"
        " backend for any BRIDGE_HW_ERR (they don't special-case"
        " BRIDGE_HW_ERR_NOTIMPL the way the status_from_hw()-routed"
        " handlers do -- see spi_reply_nosupport for that family's stub"
        " reply instead).  handle_pwm_set/get and handle_adc_read used"
        " to share this reply too until #23 routed them through"
        " status_from_hw(); GPIO_READ/WRITE are out of scope for #23"
        " and still land here",
    ))

    # ----- §14. v0.5 additions (§2B.2), continued (#31 E4) ------------
    out.append(_Section([
        "§14. v0.5 additions (§2B.2), continued (#31 E4) -- PWM_CAPTURE_READ/",
        "      END, TIMER_SYNC",
    ]))
    # CMD_PWM_CAPTURE_READ / CMD_PWM_CAPTURE_END / CMD_TIMER_SYNC round
    # out the advanced-timer-extras trio §8 already introduces
    # (CMD_PWM_CAPTURE_BEGIN, CMD_PWM_SINGLE_PULSE); their handlers
    # route through status_from_hw(), which maps the stub HAL's
    # BRIDGE_HW_ERR_NOTIMPL to STATUS_NOSUPPORT -- the existing
    # spi_reply_nosupport vector is their stub-backend reply too.
    out.append((
        "spi_pwm_capture_read_ch0_request",
        spi_frame(SOF, CMD_PWM_CAPTURE_READ, bytes([0x00])).hex().upper(),
        "SOF | CMD=0x24 | channel=0 | CRC -- dispatched to"
        " handle_pwm_capture_read() (protocol.c); reply on the gd32"
        " backend is period_ns:u32(LE) pulse_width_ns:u32(LE).  The"
        " stub HAL backend answers STATUS_NOSUPPORT"
        " (BRIDGE_HW_ERR_NOTIMPL via status_from_hw); see"
        " spi_reply_nosupport",
    ))
    out.append((
        "spi_pwm_capture_end_ch0_request",
        spi_frame(SOF, CMD_PWM_CAPTURE_END, bytes([0x00])).hex().upper(),
        "SOF | CMD=0x25 | channel=0 | CRC -- dispatched to"
        " handle_pwm_capture_end() (protocol.c); empty-payload reply on"
        " success.  The stub HAL backend answers STATUS_NOSUPPORT; see"
        " spi_reply_nosupport",
    ))
    out.append((
        "spi_timer_sync_t0_master_t7_slave_request",
        spi_frame(SOF, CMD_TIMER_SYNC, bytes([0x00, 0x01, 0x00])).hex().upper(),
        "SOF | CMD=0x27 | master=0(TIMER0) | slave=1(TIMER7) | mode=0"
        " | CRC -- dispatched to handle_timer_sync() (protocol.c); the"
        " mode encoding beyond master/slave linkage is HAL-internal and"
        " undocumented at the wire layer, so this pins only the 3-byte"
        " request framing.  The stub HAL backend answers"
        " STATUS_NOSUPPORT; see spi_reply_nosupport",
    ))

    # ----- §15. CMD_ADC_SPECTRUM_READ (#496, #31 E2) -------------------
    out.append(_Section([
        "§15. CMD_ADC_SPECTRUM_READ (#496, #31 E2) -- FFT-chain bin readback",
    ]))
    # req: stream_id:u8 bin_offset:u16(LE) max_bins:u8 (protocol.c:530).
    # reply: seq:u32(LE) total_bins:u16(LE) got:u8 bins[max_bins*4]
    # (float32 LE, zero-padded past `got`; protocol.c:552-566).  The
    # reply below is REPRESENTATIVE of the wired gd32 HAL body with
    # got(2) < max_bins(4) so the zero-pad tail is pinned, reusing the
    # float-pattern convention already established by
    # spi_tmu_compute_sqrt_f32_4p0_request (§6) -- it is not what the
    # stub HAL backend answers today, which is STATUS_NOSUPPORT
    # (handle_adc_spectrum_read special-cases BRIDGE_HW_ERR_NOTIMPL
    # explicitly; see spi_reply_nosupport).
    out.append((
        "spi_adc_spectrum_read_stream0_request",
        spi_frame(SOF, CMD_ADC_SPECTRUM_READ,
                  bytes([0x00,             # stream_id = 0
                         0x00, 0x00,       # bin_offset = 0 (LE)
                         0x04,             # max_bins = 4
                  ])).hex().upper(),
        "SOF | CMD=0x3A | stream_id=0 | bin_offset=0 | max_bins=4 | CRC",
    ))
    out.append((
        "spi_adc_spectrum_read_reply_example",
        spi_frame(SOF, STATUS_OK,
                  bytes([0x01, 0x00, 0x00, 0x00,   # seq = 1 (LE)
                         0x0A, 0x00,               # total_bins = 10 (LE)
                         0x02,                     # got = 2 (< max_bins=4)
                         0x00, 0x00, 0x80, 0x40,   # bins[0] = 4.0f (LE 0x40800000)
                         0x00, 0x00, 0x00, 0x41,   # bins[1] = 8.0f (LE 0x41000000)
                         0x00, 0x00, 0x00, 0x00,   # bins[2] = 0 (zero-pad, i >= got)
                         0x00, 0x00, 0x00, 0x00,   # bins[3] = 0 (zero-pad, i >= got)
                  ])).hex().upper(),
        "SOF | STATUS=0x00 | seq=1 (LE) | total_bins=10 (LE) | got=2 |"
        " bins[0]=4.0f | bins[1]=8.0f | bins[2..3]=0 (zero-padded, i >="
        " got) | CRC -- REPRESENTATIVE of the wired gd32 HAL body, not"
        " the stub backend's STATUS_NOSUPPORT reply",
    ))

    # ----- §16. v0.15 additions ---------------------------------------
    out.append(_Section([
        "§16. v0.15 additions -- extended LINK_FEATURES, I2C opcode policy,",
        "      ADC_STREAM_BEGIN2 / READ2, CMD_BATCH, the >65 B enforcement",
        "      (docs/protocol-v0.15-design.md section 9)",
    ]))

    def le32(v: int) -> bytes:
        return v.to_bytes(4, "little")

    def le16(v: int) -> bytes:
        return v.to_bytes(2, "little")

    # -- extended CMD_LINK_FEATURES (6-byte request, 10-byte reply) --
    lf_req = le32(0x1F) + le16(252)
    out.append((
        "spi_link_features_ext_request_all",
        spi_frame(SOF, CMD_LINK_FEATURES, lf_req).hex().upper(),
        "SOF | CMD=0x81 | want=0x0000001F (STATUS_SEQ|BIG_FRAME|ATTN|"
        "ADC_STREAM2|BATCH, u32 LE) | max_payload_req=252 (u16 LE) | CRC --"
        " the 6-byte v0.15 form; pre-v0.15 firmware answers spi_reply_inval",
    ))
    out.append((
        "spi_link_features_ext_reply_all_seq1",
        spi_frame(SOF, 0x10 | STATUS_OK, le32(0x1F) + le32(0x1F) + le16(252)).hex().upper(),
        "SOF | STATUS=0x10 (OK, stamp 1) | granted=0x1F | supported=0x1F |"
        " max_payload=252 | CRC -- gd32 backend, nothing refused",
    ))
    out.append((
        "spi_link_features_ext_reply_attn_refused_seq1",
        spi_frame(SOF, 0x10 | STATUS_OK, le32(0x1B) + le32(0x1F) + le16(252)).hex().upper(),
        "SOF | STATUS=0x10 | granted=0x1B | supported=0x1F | max_payload=252"
        " | CRC -- ATTN (bit 2) is in `supported` but not `granted`: a"
        " debugger is attached (DHCSR.C_DEBUGEN), PA14 is SWCLK",
    ))
    out.append((
        "spi_link_features_ext_reply_stub_seq1",
        spi_frame(SOF, 0x10 | STATUS_OK, le32(0x13) + le32(0x13) + le16(252)).hex().upper(),
        "SOF | STATUS=0x10 | granted=0x13 | supported=0x13 | max_payload=252"
        " | CRC -- stub backend: no ATTN, no ADC_STREAM2",
    ))
    out.append((
        "spi_reply_inval",
        spi_frame(SOF, STATUS_INVAL).hex().upper(),
        "SOF | STATUS=0x01 (INVAL) | empty payload | CRC -- also the"
        " v0.7..v0.14 answer to the 6-byte LINK_FEATURES form, a BATCH"
        " that fails validation, and a non-BATCH request over 65 bytes",
    ))

    # -- I2C: extended negotiation is echoed, opcode policy is enforced --
    out.append((
        "i2c_link_features_ext_write",
        i2c_write(CMD_LINK_FEATURES, lf_req).hex().upper(),
        "reg=0x00 | CMD=0x81 | want=0x1F | max_payload_req=252 | CRC(CMD..)"
        " -- the I2C link supports only STATUS_SEQ",
    ))
    out.append((
        "i2c_link_features_ext_read",
        i2c_read(STATUS_OK, le32(0x01) + le32(0x01) + le16(65)).hex().upper(),
        "STATUS=0x00 | granted=0x00000001 | supported=0x00000001 |"
        " max_payload=65 | CRC -- I2C never grants BIG_FRAME / ATTN /"
        " ADC_STREAM2 / BATCH, and never stamps",
    ))
    out.append((
        "i2c_adc_read_ch0_4_write_denied",
        i2c_write(CMD_ADC_READ, bytes([0x00, 0x04])).hex().upper(),
        "reg=0x00 | CMD=0x30 (ADC_READ) | channel=0 | samples=4 | CRC --"
        " ADC_READ is not on the I2C allow-list",
    ))
    out.append((
        "i2c_adc_read_ch0_4_read_denied",
        i2c_read(STATUS_NOSUPPORT).hex().upper(),
        "STATUS=0x06 (NOSUPPORT) | empty payload | CRC -- the handler never"
        " ran; bridge_i2c_denied_count++ and bridge_i2c_denied_last_cmd=0x30",
    ))
    out.append((
        "i2c_ota_get_state_write_allowed",
        i2c_write(CMD_OTA_GET_STATE).hex().upper(),
        "reg=0x00 | CMD=0xF5 (OTA_GET_STATE) | CRC -- 0xF0..0xFF stay"
        " reachable over I2C (Linux tools/gd32-ota-host)",
    ))

    # -- BEGIN2 / READ2 --
    out.append((
        "spi_adc_stream_begin2_s0_ch0_1khz_w256_request",
        spi_frame(SOF, CMD_ADC_STREAM_BEGIN2,
                  bytes([0x00, 0x00, 0x00, 0x00]) + le32(1000) + le16(256) + le16(0)
                  ).hex().upper(),
        "SOF | CMD=0x3B | stream_id=0 | channel=0 | trigger_src=0 (PACE_TIMER)"
        " | trigger_arg=0 | sample_rate_hz=1000 | watermark=256 | reserved=0"
        " | CRC (12-byte payload)",
    ))
    out.append((
        "spi_adc_stream_begin2_reply_1khz_w256",
        spi_frame(SOF, STATUS_OK,
                  le32(1000000) + le32(1000) + le16(0x0FFF) + le16(1800)
                  + bytes([0x01]) + le16(256) + le16(512)).hex().upper(),
        "SOF | STATUS=0x00 | tick_hz=1000000 | period_ticks=1000 |"
        " full_scale=4095 | vref_mv=1800 | flags=0x01 (VREF_MEASURED) |"
        " watermark=256 | ring_depth=512 (2*W) | CRC (17-byte payload)",
    ))
    out.append((
        "spi_adc_stream_begin2_reply_300hz_truncation",
        spi_frame(SOF, STATUS_OK,
                  le32(1000000) + le32(3333) + le16(0x0FFF) + le16(1800)
                  + bytes([0x00]) + le16(0) + le16(1024)).hex().upper(),
        "SOF | STATUS=0x00 | tick_hz=1000000 | period_ticks=3333 (floor of"
        " 1000000/300: the REALISED rate is tick_hz/period_ticks = 300.03 Hz)"
        " | full_scale=4095 | vref_mv=1800 | flags=0x00 (the 1800 mV"
        " fallback, not measured) | watermark=0 | ring_depth=1024 | CRC",
    ))
    out.append((
        "spi_adc_stream_read2_s0_max121_request",
        spi_frame(SOF, CMD_ADC_STREAM_READ2, bytes([0x00, 121])).hex().upper(),
        "SOF | CMD=0x3C | stream_id=0 | max_samples=121 (the ceiling at"
        " max_payload 252: (252-9)/2) | CRC",
    ))
    out.append((
        "spi_adc_stream_read2_reply_got3",
        spi_frame(SOF, STATUS_OK,
                  le32(0x100) + le32(0) + bytes([3])
                  + le16(0x0800) + le16(0x0801) + le16(0x0FFF)).hex().upper(),
        "SOF | STATUS=0x00 | first_index=256 | dropped=0 | got=3 |"
        " codes=0x0800,0x0801,0x0FFF (u16 LE, right-aligned raw) | CRC at"
        " offset 11+2*got -- the host clocks 13+2*max_samples bytes and"
        " finds the CRC from `got`",
    ))
    out.append((
        "spi_adc_stream_read2_reply_empty",
        spi_frame(SOF, STATUS_OK, le32(0x103) + le32(0) + bytes([0])).hex().upper(),
        "SOF | STATUS=0x00 | first_index=259 | dropped=0 | got=0 | CRC --"
        " an empty ring is STATUS_OK, not an error",
    ))
    out.append((
        "spi_adc_stream_read2_reply_overrun_dropped32",
        spi_frame(SOF, STATUS_OK,
                  le32(0x123) + le32(32) + bytes([2]) + le16(0x0800) + le16(0x0801)
                  ).hex().upper(),
        "SOF | STATUS=0x00 | first_index=291 | dropped=32 | got=2 | codes |"
        " CRC -- overrun is STATUS_OK with `dropped` set, never BUSY",
    ))
    out.append((
        "spi_adc_stream_read2_reply_discontinuity",
        spi_frame(SOF, STATUS_OK, le32(0x125) + le32(0xFFFFFFFF) + bytes([0])).hex().upper(),
        "SOF | STATUS=0x00 | first_index=293 | dropped=0xFFFFFFFF (a"
        " discontinuity of UNKNOWN length: ROVF recovery, a DSP pump gap)"
        " | got=0 | CRC",
    ))

    # -- CMD_BATCH --
    batch_ok = bytes([3]) \
        + bytes([CMD_GPIO_WRITE, 8]) + le32(1) + le32(1) \
        + bytes([CMD_PWM_GET, 1, 0]) \
        + bytes([CMD_ADC_STREAM_READ2, 2, 0, 16])
    out.append((
        "spi_batch_request_gpiow_pwmget_read2",
        spi_frame(SOF, CMD_BATCH, batch_ok).hex().upper(),
        "SOF | CMD=0x04 | count=3 | {0x11 len 8 mask=1 levels=1} |"
        " {0x21 len 1 ch=0} | {0x3C len 2 stream=0 max=16} | CRC",
    ))
    batch_stop = bytes([3]) \
        + bytes([CMD_GPIO_WRITE, 8]) + le32(1) + le32(1) \
        + bytes([CMD_ADC_STREAM_READ2, 2, 1, 16]) \
        + bytes([CMD_PING, 0])
    out.append((
        "spi_batch_request_gpiow_read2_inactive_ping",
        spi_frame(SOF, CMD_BATCH, batch_stop).hex().upper(),
        "SOF | CMD=0x04 | count=3 | GPIO_WRITE | READ2 on stream 1 (not"
        " running) | PING | CRC -- execution stops at the INVAL",
    ))
    out.append((
        "spi_batch_reply_stop_at_first_error",
        spi_frame(SOF, STATUS_OK, bytes([2, 0x00, 0x00, 0x01, 0x00])).hex().upper(),
        "SOF | STATUS=0x00 (outer OK: the batch validated) | executed=2 |"
        " {status=0x00 len=0} | {status=0x01 (INVAL) len=0} | CRC -- the"
        " third op (PING) never ran; a non-OK sub-status always has len 0",
    ))
    out.append((
        "spi_batch_request_nested_rejected",
        spi_frame(SOF, CMD_BATCH, bytes([1, CMD_BATCH, 0])).hex().upper(),
        "SOF | CMD=0x04 | count=1 | {0x04 len 0} | CRC -- a nested BATCH is"
        " not on the allow-list; reply = spi_reply_inval, nothing executed",
    ))
    out.append((
        "spi_batch_request_trailing_byte_rejected",
        spi_frame(SOF, CMD_BATCH, bytes([1, CMD_PING, 0, 0])).hex().upper(),
        "SOF | CMD=0x04 | count=1 | {0x00 len 0} | one TRAILING 0x00 | CRC"
        " -- the request length must equal 1 + sum(2+len) exactly; reply ="
        " spi_reply_inval (the defence against zero-extended captures)",
    ))

    # -- >65 B enforcement --
    out.append((
        "spi_ota_write_chunk_over_65_rejected_on_big_link",
        spi_frame(SOF, CMD_OTA_WRITE_CHUNK,
                  le32(0) + bytes([61]) + bytes([0xA5] * 61)).hex().upper(),
        "SOF | CMD=0xF1 | offset=0 | len=61 | 61 data bytes (66-byte"
        " payload) | CRC -- only BATCH and READ2 may exceed 65 bytes, even"
        " on a BIG_FRAME link, so OTA keeps its 56-byte chunks; reply ="
        " spi_reply_inval",
    ))

    return out


def emit(vectors: list[tuple[str, str, str | None] | _Section]) -> str:
    """Render the vector list back to the on-disk format.

    Sections come from the _Section markers build_vectors() placed in the
    stream.  This used to slice `vectors` by hard-coded index ranges held in
    this function, a second copy of the layout build_vectors() already owned;
    adding a vector shifted every later boundary and dropped the last one from
    the file without any error.  A vector outside a section, or a section with
    no vectors, is now a hard error rather than a quietly missing line in a
    file that a second repository reads.
    """
    rule = "# " + "-" * 69
    chunks: list[str] = [HEADER]

    in_section = False
    empty_tail = True

    for item in vectors:
        if isinstance(item, _Section):
            in_section = True
            empty_tail = True
            chunks.append("")
            chunks.append(rule)
            chunks.extend("# " + line for line in item.header)
            chunks.append(rule)
            continue

        if not in_section:
            raise ValueError(f"vector {item[0]!r} precedes any section marker")

        name, value, comment = item
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")
        empty_tail = False

    if in_section and empty_tail:
        raise ValueError("a section marker is followed by no vectors")

    chunks.append("")  # final newline
    return "\n".join(chunks)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--check",
        action="store_true",
        help="exit 1 if the on-disk file does not match the generated content",
    )
    parser.add_argument(
        "--out",
        type=pathlib.Path,
        default=pathlib.Path(__file__).parent / "protocol_vectors.txt",
        help="path to the vectors file (default: alongside this script)",
    )
    args = parser.parse_args(argv)

    rendered = emit(build_vectors())

    if args.check:
        if not args.out.exists():
            print(f"missing: {args.out}", file=sys.stderr)
            return 1
        on_disk = args.out.read_text(encoding="utf-8")
        if on_disk != rendered:
            print(
                f"DRIFT: {args.out} does not match generator output. "
                f"Rerun this script without --check.",
                file=sys.stderr,
            )
            return 1
        print(f"OK: {args.out} matches generator output.")
        return 0

    args.out.write_text(rendered, encoding="utf-8", newline="")
    print(f"wrote {len(rendered)} bytes to {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
