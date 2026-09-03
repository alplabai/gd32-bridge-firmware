#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""
Regenerate firmware/gd32-bridge/tests/protocol_vectors.txt.

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

    Wire layout per docs/gd32-bridge-protocol.md §5: the host clocks
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
# Constants -- keep aligned with firmware/gd32-bridge/src/protocol.h.
# ---------------------------------------------------------------------

SOF = 0xA5
CMD_PING                     = 0x00
CMD_GET_VERSION              = 0x01
CMD_GET_BUILD_ID             = 0x02
CMD_RESET_REASON             = 0x03
CMD_GPIO_READ                = 0x10
CMD_GPIO_WRITE               = 0x11
CMD_PWM_SET                  = 0x20
CMD_PWM_GET                  = 0x21
CMD_PWM_CONFIGURE            = 0x22
CMD_ADC_READ                 = 0x30
CMD_ADC_CONFIGURE            = 0x32
CMD_ADC_STREAM_BEGIN         = 0x33
CMD_ADC_STREAM_READ          = 0x34
CMD_ADC_STREAM_END           = 0x35
CMD_ADC_STREAM_CONFIGURE_DSP = 0x36
CMD_ADC_DSP_CHAIN_OPEN       = 0x37
CMD_ADC_DSP_STAGE_PUSH       = 0x38
CMD_ADC_DSP_CHAIN_BIND       = 0x39
CMD_ADC_SPECTRUM_READ        = 0x3A
CMD_PWM_CAPTURE_BEGIN        = 0x23
CMD_PWM_CAPTURE_READ         = 0x24
CMD_PWM_CAPTURE_END          = 0x25
CMD_PWM_SINGLE_PULSE         = 0x26
CMD_TIMER_SYNC               = 0x27
CMD_POWER_MODE_SET           = 0x28
CMD_TRNG_READ                = 0x80
CMD_TMU_COMPUTE              = 0x90
CMD_DAC_SET                  = 0x50
CMD_DAC_GET                  = 0x51
CMD_QENC_READ                = 0x60
CMD_QENC_RESET               = 0x61
CMD_COUNTER_READ             = 0x70
CMD_SE_RESET                 = 0x41
CMD_DA9292_STATUS_FORWARD    = 0x40
CMD_LINK_FEATURES            = 0x81
CMD_OTA_BEGIN                = 0xF0
CMD_OTA_WRITE_CHUNK          = 0xF1
CMD_OTA_VERIFY               = 0xF2
CMD_OTA_COMMIT               = 0xF3
CMD_OTA_ROLLBACK             = 0xF4
CMD_OTA_GET_STATE            = 0xF5
CMD_OTA_ABORT                = 0xF6
STATUS_OK                    = 0x00
STATUS_NOT_READY             = 0x02
STATUS_IO                    = 0x05
STATUS_NOSUPPORT             = 0x06

# Firmware-declared version triple. Parsed out of src/protocol.h at run
# time -- not a private literal -- because a hard-coded copy is exactly
# what let this drift from 0.9.0 to 0.8.0 unnoticed (#22): the
# regenerate-and-diff gate compares this file against itself and stays
# green even when the constant is stale. Do not "simplify" this back
# into a tuple; that reopens #22.
def _fw_version_from_header() -> tuple[int, int, int]:
    header = pathlib.Path(__file__).resolve().parent.parent / "src" / "protocol.h"
    try:
        text = header.read_text()
    except OSError as exc:
        sys.exit(f"gen_protocol_vectors.py: cannot read {header}: {exc}")
    parts = []
    for field in ("MAJOR", "MINOR", "PATCH"):
        m = re.search(rf"#define\s+PROTOCOL_VERSION_{field}\s+(\d+)[uU]?\b", text)
        if m is None:
            sys.exit(
                f"gen_protocol_vectors.py: could not parse "
                f"PROTOCOL_VERSION_{field} out of {header}"
            )
        parts.append(int(m.group(1)))
    return (parts[0], parts[1], parts[2])


FW_VERSION = _fw_version_from_header()


HEADER = """\
# gd32-bridge canonical wire-test vectors
#
# Both the host-side driver tests (tests/zephyr/chips/gd32g553/) and
# the firmware-side unit tests (firmware/gd32-bridge/tests/) consume this file
# so the two implementations cannot diverge.
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


def build_vectors() -> list[tuple[str, str, str | None]]:
    """Return [(name, hex_value, comment_or_none)] in emission order."""
    out: list[tuple[str, str, str | None]] = []

    # ----- §1. Foundational CRC-16/CCITT-FALSE vector ----------------
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
    # CMD_PWM_CAPTURE_{BEGIN, READ, END}, CMD_PWM_SINGLE_PULSE, and
    # CMD_TIMER_SYNC are all RESERVED at v0.5.  Portable surfaces in
    # <alp/pwm.h> (alp_pwm_capture_t + alp_pwm_single_pulse) declare
    # the API today with NOSUPPORT-with-INVAL-pre-checks stubs; the
    # GD32 bridge_hw_* HAL bodies land in a follow-up firmware drop.
    # Representative probe vectors below: PWM_CAPTURE_BEGIN with a
    # minimal payload (channel + edge) and PWM_SINGLE_PULSE with the
    # caller's intended pulse width.  Firmware replies
    # STATUS_NOSUPPORT today via the default branch.
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

    # ----- §10. v0.5 additions (§2B wave-2): chunked DSP-chain upload -
    # CMD_ADC_DSP_{CHAIN_OPEN, STAGE_PUSH, CHAIN_BIND} are RESERVED at
    # v0.5 for the wave-2 bridge-wired DSP pipeline that lets raw ADC
    # samples never traverse the wire when filtered or spectral data
    # is what the customer wants.  Portable surfaces in <alp/adc.h>
    # (alp_adc_filter_t / alp_adc_spectrum_t) and <alp/dsp.h>
    # (alp_dsp_chain_t).  Wire format documented in
    # docs/gd32-bridge-protocol.md §3.x.  Firmware default-case
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
    # Payload layouts per docs/gd32-bridge-protocol.md §10 / src/ota.c.
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
                  bytes([0x3C, 0x00,                      # chunk_max = 60 (LE)
                         0x01,                            # target_slot = B
                  ])).hex().upper(),
        "SOF | STATUS=0x00 | chunk_max=60 (LE 0x003C) | target_slot=B(1) |"
        " CRC -- 60 = MAX_PAYLOAD(65) - offset:u32 - len:u8 header",
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
                         0x00,                            # active_slot = A
                         0x01,                            # pending_slot = B
                         0x01, 0x00,                      # boot_count = 1 (LE)
                  ])).hex().upper(),
        "SOF | STATUS=0x00 | state=READY(1) | active=A(0) | pending=B(1) |"
        " boot_count=1 (LE, metadata generation) | CRC -- pending=0xFF when"
        " no session is open",
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
    # CMD_GET_BUILD_ID, CMD_RESET_REASON, CMD_GPIO_{READ,WRITE},
    # CMD_PWM_{SET,GET}, CMD_ADC_READ and CMD_DA9292_STATUS_FORWARD
    # predate the versioned §4+ additions above but had no wire vector
    # until now.  GPIO_READ/WRITE, PWM_SET/GET and ADC_READ share one
    # reply vector (spi_reply_io): their handlers
    # (handle_gpio_read/write, handle_pwm_set/get, handle_adc_read in
    # protocol.c) do NOT special-case BRIDGE_HW_ERR_NOTIMPL the way
    # status_from_hw()-routed handlers do, so on the stub HAL backend
    # (the only one CI compiles, #31 E1) they fall through to
    # `if (rv < 0) return STATUS_IO;` -- STATUS_IO, not NOSUPPORT.
    # Their real success-reply payloads carry live GPIO/PWM/ADC state
    # and are not a wire-format constant, so only the request framing
    # is vectorized here.
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
        " reads (and clears) the live MCU reset-cause flags, so this"
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
        " success; see spi_reply_io for what the stub backend answers"
        " today",
    ))
    out.append((
        "spi_pwm_get_ch0_request",
        spi_frame(SOF, CMD_PWM_GET, bytes([0x00])).hex().upper(),
        "SOF | CMD=0x21 | channel=0 | CRC -- reply (on the gd32 backend)"
        " is period_ns:u32(LE) duty_ns:u32(LE); see spi_reply_io for"
        " what the stub backend answers today",
    ))
    out.append((
        "spi_adc_read_ch0_4samples_request",
        spi_frame(SOF, CMD_ADC_READ, bytes([0x00, 0x04])).hex().upper(),
        "SOF | CMD=0x30 | channel=0 | samples=4 | CRC -- reply (on the"
        " gd32 backend) is samples:u8 (echoed) + samples*mv:u16(LE);"
        " see spi_reply_io for what the stub backend answers today",
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
        " handle_gpio_read/write, handle_pwm_set/get and handle_adc_read"
        " (protocol.c) give on the STUB HAL backend for any BRIDGE_HW_ERR"
        " (they don't special-case BRIDGE_HW_ERR_NOTIMPL the way the"
        " status_from_hw()-routed handlers do -- see spi_reply_nosupport"
        " for that family's stub reply instead)",
    ))

    # ----- §14. v0.5 additions (§2B.2), continued (#31 E4) ------------
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
        spi_frame(SOF, CMD_TIMER_SYNC, bytes([0x00, 0x07, 0x00])).hex().upper(),
        "SOF | CMD=0x27 | master=0(TIMER0) | slave=7(TIMER7) | mode=0"
        " | CRC -- dispatched to handle_timer_sync() (protocol.c); the"
        " mode encoding beyond master/slave linkage is HAL-internal and"
        " undocumented at the wire layer, so this pins only the 3-byte"
        " request framing.  The stub HAL backend answers"
        " STATUS_NOSUPPORT; see spi_reply_nosupport",
    ))

    # ----- §15. CMD_ADC_SPECTRUM_READ (#496, #31 E2) -------------------
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

    return out


def emit(vectors: list[tuple[str, str, str | None]]) -> str:
    """Render the vector list back to the on-disk format."""
    chunks: list[str] = [HEADER]

    # ----- §1 block --------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §1. Foundational CRC-16/CCITT-FALSE vector")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[:3]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

    # ----- §2 block --------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §2. SPI envelopes -- two-transaction request / reply pattern")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[3:7]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

    # ----- §3 block --------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §3. I2C envelopes -- register-style framing")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[7:9]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

    # ----- §4 block --------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §4. v0.2 additions -- DAC, quadrature encoder, free-running counter")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[9:17]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

    # ----- §5 block --------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §5. v0.3 additions -- GD32G5 HW knobs (PWM_CONFIGURE, ADC_CONFIGURE,")
    chunks.append("#                       ADC_STREAM_BEGIN / READ / END, TRNG_READ)")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[17:23]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

    # ----- §6 block --------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §6. v0.4 additions -- GD32G5 TMU (CORDIC) math accelerator")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[23:24]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

    # ----- §7 block --------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §7. v0.5 additions -- ADC-stream DSP pipeline (reserved opcode)")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[24:25]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

    # ----- §8 block --------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §8. v0.5 additions (§2B.2) -- advanced timer extras")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[25:27]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

    # ----- §9 block --------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §9. v0.5 additions (§2B.3) -- system power-mode set")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[27:28]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

    # ----- §10 block -------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §10. v0.5 additions (§2B wave-2) -- chunked DSP-chain upload")
    chunks.append("#       (CHAIN_OPEN / STAGE_PUSH / CHAIN_BIND)")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[28:31]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

    # ----- §11 block -------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §11. OTA Path-A opcodes (0xF0..0xF6) -- in-system upgrade over the")
    chunks.append("#       bridge (docs/gd32-bridge-protocol.md §10 Path A).  Unarmed")
    chunks.append("#       firmware replies STATUS_NOSUPPORT to every OTA opcode.")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[31:44]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

    # ----- §12 block -------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §12. v0.7 additions -- link-feature negotiation (CMD_LINK_FEATURES)")
    chunks.append("#       + the STATUS_SEQ stamped-reply framing (SPI only)")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[44:47]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

    # ----- §13 block -------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §13. Day-one opcode coverage (#31 E4) -- GET_BUILD_ID, RESET_REASON,")
    chunks.append("#       GPIO_READ/WRITE, legacy PWM_SET/GET, ADC_READ,")
    chunks.append("#       DA9292_STATUS_FORWARD")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[47:58]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

    # ----- §14 block -------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §14. v0.5 additions (§2B.2), continued (#31 E4) -- PWM_CAPTURE_READ/")
    chunks.append("#       END, TIMER_SYNC")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[58:61]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

    # ----- §15 block -------------------------------------------------
    chunks.append("\n# ---------------------------------------------------------------------")
    chunks.append("# §15. CMD_ADC_SPECTRUM_READ (#496, #31 E2) -- FFT-chain bin readback")
    chunks.append("# ---------------------------------------------------------------------")
    for name, value, comment in vectors[61:63]:
        if comment:
            chunks.append(f"# {comment}")
        chunks.append(f"{name:<30} = {value}")

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
