#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""
Host tests for check_trial_marker.py (bench fact 2026-09-26 follow-up).

Stdlib-only (unittest), following tools/test_gen_ota_metadata.py's pattern.

Run directly:
    python3 tools/test_check_trial_marker.py -v
"""

from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_trial_marker as ctm  # noqa: E402


def _marker(struct_ver: int = ctm.STRUCT_VER, cap: int = ctm.CAP_CONFIRM) -> bytes:
    return ctm.MAGIC + struct.pack("<HH", struct_ver, cap)


class FindMarkerTests(unittest.TestCase):
    def test_present_at_start(self):
        image = _marker() + b"\x00" * 100
        self.assertEqual(ctm.find_marker(image), (0, ctm.STRUCT_VER, ctm.CAP_CONFIRM))

    def test_present_after_vector_table(self):
        # Real images carry the marker right after a ~616-byte vector
        # table (GD32G553, 154 vectors); the scan must find it at any
        # 4-byte-aligned offset, not just 0.
        prefix = b"\xAA" * 616
        image = prefix + _marker() + b"\x00" * 100
        self.assertEqual(ctm.find_marker(image), (616, ctm.STRUCT_VER, ctm.CAP_CONFIRM))

    def test_absent(self):
        image = b"\x00" * 200
        self.assertIsNone(ctm.find_marker(image))

    def test_absent_outside_scan_window(self):
        # Marker present, but past SCAN_LIMIT -- must not be found (mirrors
        # the firmware's bounded scan, which never reads this far).
        image = b"\x00" * ctm.SCAN_LIMIT + _marker()
        self.assertIsNone(ctm.find_marker(image))

    def test_corrupt_magic_not_found(self):
        corrupt = bytearray(_marker())
        corrupt[0] ^= 0xFF
        self.assertIsNone(ctm.find_marker(bytes(corrupt) + b"\x00" * 20))


class CheckTests(unittest.TestCase):
    def _write(self, tmp_path: Path, data: bytes) -> Path:
        p = tmp_path / "slot.bin"
        p.write_bytes(data)
        return p

    def test_check_ok(self, tmp_path=None):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            p = self._write(Path(d), b"\xAA" * 32 + _marker() + b"\x00" * 32)
            self.assertEqual(ctm.check(p), 0)

    def test_check_missing_marker_fails(self):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            p = self._write(Path(d), b"\x00" * 64)
            self.assertEqual(ctm.check(p), 1)

    def test_check_wrong_struct_version_fails(self):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            p = self._write(Path(d), _marker(struct_ver=99) + b"\x00" * 32)
            self.assertEqual(ctm.check(p), 1)

    def test_check_capability_bit_unset_fails(self):
        import tempfile

        with tempfile.TemporaryDirectory() as d:
            p = self._write(Path(d), _marker(cap=0) + b"\x00" * 32)
            self.assertEqual(ctm.check(p), 1)


if __name__ == "__main__":
    unittest.main()
