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

import re
import struct
import sys
import tempfile
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import check_trial_marker as ctm  # noqa: E402

OTA_LAYOUT_HEADER = Path(__file__).resolve().parent.parent / "src" / "ota_layout.h"


def _marker(struct_ver: int = ctm.STRUCT_VER, cap: int = ctm.CAP_CONFIRM) -> bytes:
    return ctm.MAGIC + struct.pack("<HH", struct_ver, cap)


def _padded(prefix_len: int, marker: bytes, tail: int = 32) -> bytes:
    return b"\x00" * prefix_len + marker + b"\x00" * tail


class ParityWithHeaderTests(unittest.TestCase):
    """Parity check (bench fact 2026-09-26 follow-up, item 4): every
    constant this script parses out of src/ota_layout.h must match what a
    plain #define regex finds there too -- guards against the parser
    silently picking up the wrong macro (e.g. a same-named constant in a
    comment the strip missed) rather than genuinely tracking the header."""

    @classmethod
    def setUpClass(cls):
        cls.text = OTA_LAYOUT_HEADER.read_text(encoding="utf-8")

    def test_magic_matches_header_literal(self):
        m = re.search(
            r"#define\s+OTA_TRIAL_MARKER_MAGIC_BYTES\s+\\\s*\{([^}]*)\}", self.text, re.DOTALL
        )
        self.assertIsNotNone(m, "OTA_TRIAL_MARKER_MAGIC_BYTES not found in ota_layout.h")
        chars = re.findall(r"'(.)'", m.group(1))
        self.assertEqual(bytes(ord(c) for c in chars), ctm.MAGIC)

    def test_struct_version_matches_header_literal(self):
        m = re.search(r"#define\s+OTA_TRIAL_MARKER_STRUCT_VER\s+(\d+)u?\b", self.text)
        self.assertIsNotNone(m)
        self.assertEqual(int(m.group(1)), ctm.STRUCT_VER)

    def test_cap_confirm_matches_header_literal(self):
        m = re.search(r"#define\s+OTA_TRIAL_CAP_CONFIRM\s+(0x[0-9A-Fa-f]+)u?\b", self.text)
        self.assertIsNotNone(m)
        self.assertEqual(int(m.group(1), 16), ctm.CAP_CONFIRM)

    def test_scan_limit_matches_header_literal(self):
        m = re.search(r"#define\s+OTA_TRIAL_SCAN_LIMIT\s+(\d+)u?\b", self.text)
        self.assertIsNotNone(m)
        self.assertEqual(int(m.group(1)), ctm.SCAN_LIMIT)

    def test_min_offset_matches_header_literal(self):
        m = re.search(r"#define\s+OTA_TRIAL_MARKER_MIN_OFFSET\s+(\d+)u?\b", self.text)
        self.assertIsNotNone(m)
        self.assertEqual(int(m.group(1)), ctm.MIN_OFFSET)

    def test_marker_len_matches_struct_static_assert(self):
        m = re.search(
            r"_Static_assert\(sizeof\(ota_trial_marker_t\)\s*==\s*(\d+)u", self.text
        )
        self.assertIsNotNone(m)
        self.assertEqual(int(m.group(1)), ctm.MARKER_LEN)


class FindMarkerTests(unittest.TestCase):
    def test_present_at_min_offset(self):
        image = _padded(ctm.MIN_OFFSET, _marker())
        self.assertEqual(
            ctm.find_marker(image), (ctm.MIN_OFFSET, ctm.STRUCT_VER, ctm.CAP_CONFIRM)
        )

    def test_present_after_vector_table(self):
        # Real images carry the marker right after a ~616-byte vector
        # table (GD32G553, 154 vectors); the scan must find it at any
        # 4-byte-aligned offset >= MIN_OFFSET, not just at the floor.
        image = _padded(616, _marker())
        self.assertEqual(ctm.find_marker(image), (616, ctm.STRUCT_VER, ctm.CAP_CONFIRM))

    def test_before_min_offset_not_found(self):
        # A byte-for-byte marker sitting inside the core vector-table
        # region (< MIN_OFFSET) must NOT be treated as the real one.
        image = _marker() + b"\x00" * 32
        self.assertIsNone(ctm.find_marker(image))

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
        self.assertIsNone(ctm.find_marker(_padded(ctm.MIN_OFFSET, bytes(corrupt))))


class CheckTests(unittest.TestCase):
    def _write(self, tmp_path: Path, data: bytes) -> Path:
        p = tmp_path / "slot.bin"
        p.write_bytes(data)
        return p

    def test_check_ok(self):
        with tempfile.TemporaryDirectory() as d:
            p = self._write(Path(d), _padded(ctm.MIN_OFFSET, _marker()))
            self.assertEqual(ctm.check(p, None, None), 0)

    def test_check_missing_marker_fails(self):
        with tempfile.TemporaryDirectory() as d:
            p = self._write(Path(d), b"\x00" * 96)
            self.assertEqual(ctm.check(p, None, None), 1)

    def test_check_marker_before_min_offset_fails(self):
        with tempfile.TemporaryDirectory() as d:
            p = self._write(Path(d), _marker() + b"\x00" * 32)
            self.assertEqual(ctm.check(p, None, None), 1)

    def test_check_wrong_struct_version_fails(self):
        with tempfile.TemporaryDirectory() as d:
            p = self._write(Path(d), _padded(ctm.MIN_OFFSET, _marker(struct_ver=99)))
            self.assertEqual(ctm.check(p, None, None), 1)

    def test_check_capability_bit_unset_fails(self):
        with tempfile.TemporaryDirectory() as d:
            p = self._write(Path(d), _padded(ctm.MIN_OFFSET, _marker(cap=0)))
            self.assertEqual(ctm.check(p, None, None), 1)


if __name__ == "__main__":
    unittest.main()
