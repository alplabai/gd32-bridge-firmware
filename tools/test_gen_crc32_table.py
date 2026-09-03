#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""
Host tests for gen_crc32_table.py (#26 C2, #104).

Stdlib-only (unittest + zlib), no test framework -- same pattern as
tools/test_gen_ota_metadata.py.  Run directly:
    python3 tools/test_gen_crc32_table.py -v
"""

from __future__ import annotations

import sys
import unittest
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_crc32_table as gct  # noqa: E402

REPO_ROOT = Path(__file__).resolve().parent.parent
CRC32_C = REPO_ROOT / "src" / "crc32.c"


class TestGenCrc32Table(unittest.TestCase):
    def test_table_has_256_entries(self):
        table = gct.generate_table()
        self.assertEqual(len(table), 256)

    def test_check_value_matches_standard_and_zlib(self):
        # The canonical CRC-32 check value for "123456789".
        self.assertEqual(gct.self_check_value(), 0xCBF43926)
        self.assertEqual(zlib.crc32(b"123456789") & 0xFFFFFFFF, 0xCBF43926)

    def test_table_matches_zlib_byte_by_byte(self):
        # Cross-check every single-byte crc32(seed=0, [n]) against zlib,
        # an independent implementation of the same algorithm.
        table = gct.generate_table()
        for n in range(256):
            crc = 0xFFFFFFFF
            crc = table[(crc ^ n) & 0xFF] ^ (crc >> 8)
            got = crc ^ 0xFFFFFFFF
            want = zlib.crc32(bytes([n])) & 0xFFFFFFFF
            self.assertEqual(got, want, f"byte {n:#04x}")

    def test_checked_in_src_crc32_c_matches_generator(self):
        # This is the guard against 256 hand-typed (or hand-edited) magic
        # numbers drifting from the algorithm that is supposed to have
        # produced them: fails loudly if src/crc32.c's table and this
        # script's output ever diverge by even one entry.
        self.assertTrue(CRC32_C.is_file(), f"{CRC32_C} not found")
        checked_in = gct.extract_table_from_file(CRC32_C)
        generated = gct.generate_table()
        self.assertEqual(checked_in, generated)

    def test_check_cli_mode_exits_zero_on_the_real_file(self):
        rc = gct.main(["--check", str(CRC32_C)])
        self.assertEqual(rc, 0)

    def test_check_cli_mode_detects_a_corrupted_entry(self):
        # Mutation check: flip one table entry in a scratch copy and prove
        # --check catches it (and does NOT touch the real file).
        import tempfile
        text = CRC32_C.read_text()
        corrupted = text.replace("0x77073096u", "0x77073097u", 1)
        self.assertNotEqual(corrupted, text, "fixture string not found in src/crc32.c")
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
