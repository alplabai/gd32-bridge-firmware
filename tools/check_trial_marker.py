#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""
Build-time gate: fail if a built OTA-slot .bin does not carry a findable,
confirm-capable trial marker (bench fact 2026-09-26 follow-up, E1M-V2M103).

The downgrade guard that decides whether COMMIT/ROLLBACK marks a slot
OTA_META_FLAG_TRIAL used to trust the fw_version the HOST declared in
OTA_BEGIN -- the 2026-09-26 incident this whole trial/confirm dance
responds to was the host declaring the BAD image's TRUE, pre-fix version,
which that guard would have believed and committed the same image WITHOUT
trial protection. Eligibility now comes from the image itself: every
OTA-slot app image plants a 20-byte marker right after its vector table
(toolchain/gd32g553_app_slot.ld.in's `.trial_marker` section,
src/trial_marker.c), and ota_image_trial_capable() (src/ota_layout.h)
scans a bounded window of the slot's own flash bytes for it at
COMMIT/ROLLBACK.

This script runs the IDENTICAL bounded scan against the just-built .bin
(wired in as a POST_BUILD step in CMakeLists.txt for the gd32 backend) --
not a trust that the linker script placed the section correctly, an
actual check of the produced artifact. A slot image built without a
findable marker fails the build instead of shipping one that would
silently commit CONFIRMED (no safety net) if this policy is ever
regressed.

Constants below mirror src/ota_layout.h -- keep them in lockstep with
that file.

Run directly:
    python3 tools/check_trial_marker.py build/gd32-bridge-slot-a.bin
"""

from __future__ import annotations

import argparse
import struct
import sys
from pathlib import Path

MAGIC = b"GD32BRIDGE-TRIAL"  # OTA_TRIAL_MARKER_MAGIC_BYTES, 16 bytes
STRUCT_VER = 1  # OTA_TRIAL_MARKER_STRUCT_VER
CAP_CONFIRM = 0x0001  # OTA_TRIAL_CAP_CONFIRM (bit0)
MARKER_LEN = 20  # sizeof(ota_trial_marker_t): 16-byte magic + 2 u16 fields
SCAN_LIMIT = 2048  # OTA_TRIAL_SCAN_LIMIT


def find_marker(image: bytes, limit: int = SCAN_LIMIT) -> tuple[int, int, int] | None:
    """Return (offset, struct_version, capability_flags) for the first
    marker found in image[:limit], or None -- mirrors
    ota_image_trial_capable()'s scan exactly (same window, same 4-byte
    stride, same field layout)."""
    window = image[:limit]
    off = 0
    while off + MARKER_LEN <= len(window):
        if window[off : off + 16] == MAGIC:
            struct_ver, cap = struct.unpack_from("<HH", window, off + 16)
            return off, struct_ver, cap
        off += 4
    return None


def check(bin_path: Path) -> int:
    image = bin_path.read_bytes()
    found = find_marker(image)
    if found is None:
        print(
            f"error: {bin_path}: no trial-capability marker found in the first "
            f"{SCAN_LIMIT} bytes -- this image would commit/rollback CONFIRMED "
            "(no watchdog safety net). See src/trial_marker.c / "
            "toolchain/gd32g553_app_slot.ld.in.",
            file=sys.stderr,
        )
        return 1
    offset, struct_ver, cap = found
    hexbytes = image[offset : offset + MARKER_LEN].hex()
    print(f"{bin_path}: trial marker found at offset 0x{offset:04x}: {hexbytes}")
    if struct_ver != STRUCT_VER:
        print(
            f"error: {bin_path}: marker struct_version={struct_ver}, "
            f"expected {STRUCT_VER}",
            file=sys.stderr,
        )
        return 1
    if not (cap & CAP_CONFIRM):
        print(
            f"error: {bin_path}: marker capability_flags=0x{cap:04x} does not set "
            f"CAP_CONFIRM (0x{CAP_CONFIRM:04x}) -- image is not trial-confirm-capable",
            file=sys.stderr,
        )
        return 1
    print(f"{bin_path}: OK (struct_version={struct_ver}, capability_flags=0x{cap:04x})")
    return 0


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("bin_path", type=Path, help="built slot .bin to check")
    args = ap.parse_args(argv)
    return check(args.bin_path)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
