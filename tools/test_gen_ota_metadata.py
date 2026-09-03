#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""
Host tests for gen_ota_metadata.py (#25 B12/B13).

Stdlib-only (unittest + subprocess), no test framework: this is the same
tree that byte-compiles `tests` and `tools` in CI (.github/workflows/ci.yml,
job `python-lint`), and this file lives next to its subject rather than in
`tests/unit/`, which is a C/ztest suite (see tests/unit/CMakeLists.txt) with
no Python test of its own to pattern-match. There was no existing Python
test home in this repo; `tools/` is it now.

Run directly:
    python3 tools/test_gen_ota_metadata.py -v
"""

from __future__ import annotations

import shutil
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import gen_ota_metadata as gom  # noqa: E402

TOOL = Path(__file__).resolve().parent / "gen_ota_metadata.py"


def _image(msp: int, reset: int, pad_to: int = 8) -> bytes:
    """Build a minimal synthetic image: header word0=msp, word1=reset,
    zero-padded to `pad_to` bytes. Real slot images are much larger; only
    the first 8 bytes are load-bearing for ota_image_bootable()."""
    body = struct.pack("<II", msp, reset)
    return body + b"\x00" * (pad_to - len(body))


# Issue #25 / PR #90's review cites 0x08011621 as a genuine slot-A reset
# vector on a real, much larger image; these tests use small synthetic
# images, so the reset vector here only needs the same two properties
# (Thumb bit set, inside the slot) at an address the short image covers.
GOOD_MSP = 0x20010000  # inside [0x20000000, 0x20040000], word-aligned
GOOD_RESET = gom.OTA_SLOT_A_BASE | 1  # Thumb bit set, base itself


class CheckBootable(unittest.TestCase):
    def test_good_image_accepted(self) -> None:
        img = _image(GOOD_MSP, GOOD_RESET, pad_to=64)
        gom.check_bootable(0, img)  # must not raise

    def test_too_short_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "at least 8 bytes"):
            gom.check_bootable(0, b"\x00" * 4)

    def test_msp_outside_sram_rejected(self) -> None:
        img = _image(0x08000000, GOOD_RESET, pad_to=64)  # flash, not SRAM
        with self.assertRaisesRegex(ValueError, "initial MSP"):
            gom.check_bootable(0, img)

    def test_msp_misaligned_rejected(self) -> None:
        img = _image(GOOD_MSP | 1, GOOD_RESET, pad_to=64)
        with self.assertRaisesRegex(ValueError, "initial MSP"):
            gom.check_bootable(0, img)

    def test_thumb_bit_clear_rejected(self) -> None:
        img = _image(GOOD_MSP, GOOD_RESET & ~1, pad_to=64)  # even address
        with self.assertRaisesRegex(ValueError, "Thumb bit"):
            gom.check_bootable(0, img)

    def test_reset_outside_slot_rejected(self) -> None:
        # Valid Thumb address, but linked for slot B while asked for slot A.
        img = _image(GOOD_MSP, gom.OTA_SLOT_B_BASE | 1, pad_to=64)
        with self.assertRaisesRegex(ValueError, "outside"):
            gom.check_bootable(0, img)

    def test_oversized_rejected_by_build_record(self) -> None:
        img = _image(GOOD_MSP, GOOD_RESET, pad_to=64)
        img += b"\xff" * gom.OTA_SLOT_SIZE  # push length past OTA_SLOT_SIZE
        with self.assertRaisesRegex(ValueError, "must be within"):
            gom.build_record(0, 1, img, 0)


class BuildRecordCrc(unittest.TestCase):
    """A generator that mis-packs the CRC is the B7-flavoured mistake for
    metadata: verify both CRC fields the bootloader checks, not just that
    packing didn't raise."""

    def test_record_crc_fields_are_correct(self) -> None:
        img = _image(GOOD_MSP, GOOD_RESET, pad_to=64)
        rec = gom.build_record(0, 7, img, 0x00010203)
        self.assertEqual(len(rec), 44)
        magic, ver, counter, active, valid = struct.unpack_from("<III BB", rec, 0)
        fw_ver = struct.unpack_from("<2I", rec, 16)
        img_len = struct.unpack_from("<2I", rec, 24)
        img_crc = struct.unpack_from("<2I", rec, 32)
        (rec_crc,) = struct.unpack_from("<I", rec, 40)

        self.assertEqual(magic, gom.OTA_META_MAGIC)
        self.assertEqual(ver, gom.OTA_META_STRUCT_VER)
        self.assertEqual(counter, 7)
        self.assertEqual(active, 0)
        self.assertEqual(valid, 0b01)
        self.assertEqual(fw_ver[0], 0x00010203)
        self.assertEqual(img_len[0], len(img))
        self.assertEqual(img_crc[0], zlib.crc32(img) & 0xFFFFFFFF)
        # rec_crc32 covers every byte before it -- this is what
        # active_slot_valid()/meta_read() in the firmware recompute.
        self.assertEqual(rec_crc, zlib.crc32(rec[:40]) & 0xFFFFFFFF)


class CliExitStatus(unittest.TestCase):
    """Distinct exit statuses: 0 = wrote a record, 1 = a bootability
    ValueError, 2 = argparse rejected the arguments outright."""

    def _run(self, img: bytes, extra_args: list[str] | None = None):
        # NOT a `with` block: the caller inspects out_path after this
        # returns, so the tempdir must outlive the call. addCleanup tears
        # it down once the test finishes either way.
        td = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, td, True)
        img_path = Path(td) / "slot.bin"
        out_path = Path(td) / "rec.bin"
        img_path.write_bytes(img)
        cmd = [sys.executable, str(TOOL), "--slot-image", str(img_path),
               "--out", str(out_path)]
        cmd += extra_args or []
        proc = subprocess.run(cmd, capture_output=True, text=True)
        return proc, out_path

    def test_good_image_exits_zero_and_writes_output(self) -> None:
        proc, out_path = self._run(_image(GOOD_MSP, GOOD_RESET, pad_to=64))
        self.assertEqual(proc.returncode, 0, proc.stderr)
        self.assertTrue(out_path.exists())
        self.assertEqual(out_path.stat().st_size, 44)

    def test_thumb_bit_clear_exits_one(self) -> None:
        proc, out_path = self._run(_image(GOOD_MSP, GOOD_RESET & ~1, pad_to=64))
        self.assertEqual(proc.returncode, 1)
        self.assertIn("Thumb bit", proc.stderr)
        self.assertFalse(out_path.exists())

    def test_oversized_exits_one(self) -> None:
        img = _image(GOOD_MSP, GOOD_RESET, pad_to=64) + b"\xff" * gom.OTA_SLOT_SIZE
        proc, out_path = self._run(img)
        self.assertEqual(proc.returncode, 1)
        self.assertIn("must be within", proc.stderr)
        self.assertFalse(out_path.exists())

    def test_bad_counter_exits_two(self) -> None:
        proc, out_path = self._run(_image(GOOD_MSP, GOOD_RESET, pad_to=64),
                                    extra_args=["--counter", "-1"])
        self.assertEqual(proc.returncode, 2)
        self.assertFalse(out_path.exists())


if __name__ == "__main__":
    unittest.main()
