#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""
Generate the 256-entry byte-wise CRC-16/CCITT-FALSE lookup table checked
into src/protocol.c, and (--check) prove that the checked-in table still
matches what this script produces.

Why this exists (#26, #104): crc16_ccitt_false() -- the framing CRC both
transports run TWICE per round trip (verify the request, compute the
reply) -- used to be bit-serial, 8 inner iterations per byte.  Measured at
~76 instructions/byte at -Os (arm-none-eabi-gcc 13.3.1); PR #115 gave
ota_crc32() the same table treatment first and measured ~5.75x fewer
instructions/byte, so this mirrors that construction rather than
reinventing it.  See tools/gen_crc32_table.py for the CRC-32 sibling.

Algorithm: CRC-16/CCITT-FALSE, poly 0x1021, init 0xFFFF, NON-reflected
(MSB-first), xor-out 0x0000.  This is a DIFFERENT shape of table from
CRC-32's: CRC-32 is reflected (LSB-first, poly 0xEDB88320) and its table
recurrence is `crc = table[(crc ^ byte) & 0xFF] ^ (crc >> 8)`.
CRC-16/CCITT-FALSE is non-reflected -- the byte enters at the TOP of the
16-bit register, and the table recurrence is:

    crc = (crc << 8) ^ table[((crc >> 8) ^ byte) & 0xFF]

table[n] is the bit-serial routine's inner 8-iteration loop applied to
(n << 8) alone -- i.e. what crc16_ccitt_false's original loop would leave
in the top byte after `crc ^= byte << 8` with the bottom byte held at 0.
This is standard Sarwate table-driven CRC construction (same technique as
CRC-32's, adapted for a non-reflected/MSB-first polynomial), not a new
algorithm.  Derivation, for anyone re-deriving instead of trusting this
comment: shift8() (the 8-iteration inner loop) is XOR-linear over GF(2),
so shift8(crc_in ^ (byte<<8)) = shift8(crc_in) ^ shift8(byte<<8).  Writing
crc_in as (h<<8)|l for byte-halves h, l: shift8(crc_in) = shift8(h<<8) ^
shift8(l), and shift8(l) = l<<8 exactly (an 8-bit value's bit 15 can never
be set within 8 left-shifts, so no XOR ever triggers -- it is a pure
shift).  shift8(h<<8) is table[h] by definition.  Substituting back gives
exactly the recurrence above.

Regenerate and diff by hand:
    python3 tools/gen_crc16_table.py > /tmp/table16.txt
    # then compare the lines against src/protocol.c's fenced table

Or just run the check:
    python3 tools/gen_crc16_table.py --check src/protocol.c
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys

CRC16_POLY = 0x1021

BEGIN_MARK = "/* clang-format off */"
END_MARK = "/* clang-format on */"

_TABLE_LINE_RE = re.compile(r"0x([0-9A-Fa-f]{4})u")


def generate_table(poly: int = CRC16_POLY) -> list[int]:
    """The Sarwate byte-wise table: table[n] = 8 MSB-first bit-serial
    steps over (n << 8), the top-byte-only case the derivation above
    reduces every byte's contribution to."""
    table = []
    for n in range(256):
        c = (n << 8) & 0xFFFF
        for _ in range(8):
            c = ((c << 1) ^ poly) & 0xFFFF if (c & 0x8000) else (c << 1) & 0xFFFF
        table.append(c)
    return table


def render_c_table(table: list[int], per_line: int = 8) -> str:
    lines = [BEGIN_MARK, "static const uint16_t crc16_table[256] = {"]
    for i in range(0, len(table), per_line):
        row = ", ".join(f"0x{v:04X}u" for v in table[i:i + per_line])
        lines.append(f"\t{row},")
    lines.append("};")
    lines.append(END_MARK)
    return "\n".join(lines) + "\n"


def extract_table_from_file(path: pathlib.Path) -> list[int]:
    """Pull the crc16_table[] literal out of a checked-in C file."""
    text = path.read_text()
    try:
        start = text.index(BEGIN_MARK)
        end = text.index(END_MARK, start)
    except ValueError as exc:
        raise ValueError(
            f"{path}: no {BEGIN_MARK} .. {END_MARK} fenced block found -- "
            f"is this the generated table?") from exc
    body = text[start:end]
    values = [int(m, 16) for m in _TABLE_LINE_RE.findall(body)]
    if len(values) != 256:
        raise ValueError(f"{path}: found {len(values)} table entries, expected 256")
    return values


def table_driven_crc16(table: list[int], data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc = ((crc << 8) & 0xFFFF) ^ table[((crc >> 8) ^ b) & 0xFF]
    return crc


def self_check_value() -> int:
    """The canonical CRC-16/CCITT-FALSE check value for "123456789"."""
    table = generate_table()
    return table_driven_crc16(table, b"123456789")


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", type=pathlib.Path, metavar="FILE",
                        help="verify FILE's checked-in table matches this script's output")
    args = parser.parse_args(argv)

    table = generate_table()

    computed_check_value = self_check_value()
    if computed_check_value != 0x29B1:
        print(f"error: generated table gives check value 0x{computed_check_value:04X}, "
              f"expected the standard CRC-16/CCITT-FALSE 0x29B1",
              file=sys.stderr)
        return 1

    if args.check:
        try:
            checked_in = extract_table_from_file(args.check)
        except ValueError as exc:
            print(f"error: {exc}", file=sys.stderr)
            return 1
        if checked_in != table:
            mismatches = [i for i in range(256) if checked_in[i] != table[i]]
            print(f"error: {args.check} table drifted from gen_crc16_table.py at "
                  f"{len(mismatches)} entr{'y' if len(mismatches) == 1 else 'ies'}, "
                  f"first at index {mismatches[0]}: "
                  f"checked-in=0x{checked_in[mismatches[0]]:04X} "
                  f"generated=0x{table[mismatches[0]]:04X}",
                  file=sys.stderr)
            return 1
        print(f"{args.check}: table matches (256/256 entries, check value 0x{computed_check_value:04X})")
        return 0

    sys.stdout.write(render_c_table(table))
    return 0


if __name__ == "__main__":
    sys.exit(main())
