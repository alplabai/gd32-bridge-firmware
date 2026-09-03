#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""
Host tests for gen_crc16_table.py (#26, #104).

Stdlib-only (unittest), no test framework -- same pattern as
tools/test_gen_crc32_table.py.  Run directly:
    python3 tools/test_gen_crc16_table.py -v
"""

from __future__ import annotations

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_crc16_table as gct  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
PROTOCOL_C = REPO_ROOT / "src" / "protocol.c"


def bit_serial_crc16_ccitt_false(data: bytes) -> int:
    """Independent oracle: a from-scratch bit-serial CRC-16/CCITT-FALSE,
    written without reference to gen_crc16_table.py's table-precompute
    construction (poly 0x1021, init 0xFFFF, non-reflected, xor-out 0) --
    a byte-at-a-time transliteration of the algorithm's textbook
    definition, not a copy of the table generator's per-byte shortcut."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class TestGenCrc16Table(unittest.TestCase):
    def test_table_has_256_entries(self):
        table = gct.generate_table()
        self.assertEqual(len(table), 256)

    def test_check_value_matches_standard(self):
        # The canonical CRC-16/CCITT-FALSE check value for "123456789".
        self.assertEqual(gct.self_check_value(), 0x29B1)
        self.assertEqual(bit_serial_crc16_ccitt_false(b"123456789"), 0x29B1)

    def test_table_matches_bit_serial_oracle_byte_by_byte(self):
        # Cross-check every single-byte crc16(seed=0xFFFF, [n]) against the
        # independent bit-serial oracle above.
        table = gct.generate_table()
        for n in range(256):
            got = gct.table_driven_crc16(table, bytes([n]))
            want = bit_serial_crc16_ccitt_false(bytes([n]))
            self.assertEqual(got, want, f"byte {n:#04x}")

    def test_table_matches_bit_serial_oracle_multi_byte(self):
        vectors = [b"A", b"Alp Lab", b"The quick brown fox jumps over the lazy dog",
                   bytes(8), b"\xff" * 8]
        table = gct.generate_table()
        for v in vectors:
            got = gct.table_driven_crc16(table, v)
            want = bit_serial_crc16_ccitt_false(v)
            self.assertEqual(got, want, f"vector {v!r}")

    def test_checked_in_src_protocol_c_matches_generator(self):
        # This is the guard against 256 hand-typed (or hand-edited) magic
        # numbers drifting from the algorithm that is supposed to have
        # produced them: fails loudly if src/protocol.c's table and this
        # script's output ever diverge by even one entry.
        self.assertTrue(PROTOCOL_C.is_file(), f"{PROTOCOL_C} not found")
        checked_in = gct.extract_table_from_file(PROTOCOL_C)
        generated = gct.generate_table()
        self.assertEqual(checked_in, generated)

    def test_check_cli_mode_exits_zero_on_the_real_file(self):
        rc = gct.main(["--check", str(PROTOCOL_C)])
        self.assertEqual(rc, 0)

    def test_check_cli_mode_detects_a_corrupted_entry(self):
        # Mutation check: flip one table entry in a scratch copy and prove
        # --check catches it (and does NOT touch the real file).
        import tempfile
        text = PROTOCOL_C.read_text()
        corrupted = text.replace("0x1021u, 0x2042u", "0x1021u, 0x2043u", 1)
        self.assertNotEqual(corrupted, text, "fixture string not found in src/protocol.c")
        with tempfile.NamedTemporaryFile("w", suffix=".c", delete=False) as f:
            f.write(corrupted)
            tmp_path = Path(f.name)
        try:
            rc = gct.main(["--check", str(tmp_path)])
            self.assertEqual(rc, 1)
        finally:
            tmp_path.unlink()


if __name__ == "__main__":
    unittest.main()
