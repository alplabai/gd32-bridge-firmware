#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""
Generate the 256-entry byte-wise CRC-32 lookup table checked into
src/crc32.c, and (--check) prove that the checked-in table still matches
what this script produces.

Why this exists (#26 C2, #104): ota_crc32() used to be bit-serial, 8
inner iterations per byte -- ~1.93M iterations for a full OTA_SLOT_SIZE
(0x0003B000 = 241664 byte) verify.  The table trades that for one lookup
per byte, at a cost of 1 KB of .rodata.  Nobody should have to eyeball
256 magic numbers and trust they are the right ones, so the table is
generated here from the algorithm alone, and this script's own --check
mode is what tools/test_gen_crc32_table.py (and a human, by hand) uses
to prove src/crc32.c's checked-in copy is exactly this output -- not
something hand-edited afterward.

Algorithm: standard reflected CRC-32 (IEEE 802.3 / zlib), polynomial
0xEDB88320.  table[n] is the bit-serial routine's inner 8-iteration loop
applied to n alone -- byte-at-a-time use folds in the running crc's low
byte first:

    crc = table[(crc ^ byte) & 0xFF] ^ (crc >> 8)

which is mathematically identical to running the original bit-serial
loop byte-by-byte (the outer XOR distributes over the 8 inner XOR/shift
steps -- this is the standard Sarwate table-driven CRC construction, not
a new algorithm).  Pre-invert with 0xFFFFFFFF, post-invert the same way:
that part of ota_crc32() is UNCHANGED, so the running-crc SEED contract
(src/ota.c chains a verify across multiple calls) is preserved exactly.

Regenerate and diff by hand:
    python3 tools/gen_crc32_table.py > /tmp/table.txt
    # then compare the 64 lines against src/crc32.c's fenced table

Or just run the check:
    python3 tools/gen_crc32_table.py --check src/crc32.c
"""

from __future__ import annotations

import argparse
import pathlib
import re
import sys
import zlib

CRC32_POLY = 0xEDB88320

BEGIN_MARK = "/* clang-format off */"
END_MARK = "/* clang-format on */"

_TABLE_LINE_RE = re.compile(r"0x([0-9A-Fa-f]{8})u")


def generate_table(poly: int = CRC32_POLY) -> list[int]:
    """The Sarwate byte-wise table: table[n] = 8 bit-serial steps over n."""
    table = []
    for n in range(256):
        c = n
        for _ in range(8):
            c = (poly ^ (c >> 1)) if (c & 1) else (c >> 1)
        table.append(c & 0xFFFFFFFF)
    return table


def render_c_table(table: list[int], per_line: int = 8) -> str:
    lines = [BEGIN_MARK, "static const uint32_t crc32_table[256] = {"]
    for i in range(0, len(table), per_line):
        row = ", ".join(f"0x{v:08X}u" for v in table[i:i + per_line])
        lines.append(f"\t{row},")
    lines.append("};")
    lines.append(END_MARK)
    return "\n".join(lines) + "\n"


def extract_table_from_file(path: pathlib.Path) -> list[int]:
    """Pull the crc32_table[] literal out of a checked-in C file."""
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


def self_check_value() -> int:
    """The standard CRC-32 check value, cross-checked against zlib.crc32
    (which implements the same reflected/init/xorout convention)."""
    table = generate_table()
    crc = 0xFFFFFFFF
    for b in b"123456789":
        crc = table[(crc ^ b) & 0xFF] ^ (crc >> 8)
    return crc ^ 0xFFFFFFFF


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", type=pathlib.Path, metavar="FILE",
                        help="verify FILE's checked-in table matches this script's output")
    args = parser.parse_args(argv)

    table = generate_table()

    computed_check_value = self_check_value()
    zlib_check_value = zlib.crc32(b"123456789") & 0xFFFFFFFF
    if computed_check_value != zlib_check_value or computed_check_value != 0xCBF43926:
        print(f"error: generated table gives check value 0x{computed_check_value:08X}, "
              f"expected the standard 0xCBF43926 (zlib agrees: 0x{zlib_check_value:08X})",
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
            print(f"error: {args.check} table drifted from gen_crc32_table.py at "
                  f"{len(mismatches)} entr{'y' if len(mismatches) == 1 else 'ies'}, "
                  f"first at index {mismatches[0]}: "
                  f"checked-in=0x{checked_in[mismatches[0]]:08X} "
                  f"generated=0x{table[mismatches[0]]:08X}",
                  file=sys.stderr)
            return 1
        print(f"{args.check}: table matches (256/256 entries, check value 0x{computed_check_value:08X})")
        return 0

    sys.stdout.write(render_c_table(table))
    return 0


if __name__ == "__main__":
    sys.exit(main())
