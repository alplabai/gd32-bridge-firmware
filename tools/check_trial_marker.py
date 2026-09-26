#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""
Build-time gate: fail if a built OTA-slot .bin does not carry a findable,
confirm-capable trial marker (bench fact 2026-09-26 follow-up, E1M-V2M103).

Policy this gate enforces (see src/ota_layout.h's own policy comment,
just above ota_trial_marker_t, for the full rationale -- state it
identically here, do not soften it): ONLY an image that carries this
marker (with the confirm-capability bit set) ever gets the FWDGT safety
net. A markerless image commits/rolls back CONFIRMED and is NOT
protected -- that is the deliberate trade-off (forcing TRIAL onto a
markerless image would revert, or reset-loop, even a healthy one, since
an old app has no confirm path). This script exists so a slot image that
was SUPPOSED to carry the marker (anything built from this tree) but
doesn't -- a linker-script regression, a missing translation unit, a
corrupted build -- fails at build time instead of silently shipping
unprotected.

The marker itself (magic + struct version + capability bitfield) mirrors
`toolchain/gd32g553_app_slot.ld.in`'s `.trial_marker` section and
`src/trial_marker.c`. Every constant below is PARSED OUT OF
src/ota_layout.h at run time (the same approach
tests/gen_protocol_vectors.py uses for src/protocol.h) rather than
hand-mirrored, so the two can never independently drift.

Run directly:
    python3 tools/check_trial_marker.py build/gd32-bridge-slot-a.bin

With the adjacency check (requires the linked ELF and the slot's flash
base -- this is how CMakeLists.txt's POST_BUILD step invokes it):
    python3 tools/check_trial_marker.py build/gd32-bridge-slot-a.bin \
        --elf build/gd32-bridge-slot-a.elf --slot-base 0x0800A000
"""

from __future__ import annotations

import argparse
import re
import shutil
import struct
import subprocess
import sys
from pathlib import Path

OTA_LAYOUT_HEADER = Path(__file__).resolve().parent.parent / "src" / "ota_layout.h"

# sizeof(ota_trial_marker_t): 16-byte magic + two u16 fields. This is
# struct-layout arithmetic (asserted in C by ota_layout.h's own
# _Static_assert), not an independent policy value, so it is not parsed
# out of the header the way the constants below are.
MARKER_LEN = 20


def _strip_comments(text: str) -> str:
    return re.sub(r"/\*.*?\*/|//[^\r\n]*", "", text, flags=re.DOTALL)


def _join_continuations(text: str) -> str:
    """Collapse a backslash-newline-continued #define onto one line, so
    the single-line regexes below can match OTA_TRIAL_MARKER_MAGIC_BYTES
    (a multi-line macro in ota_layout.h)."""
    return re.sub(r"\\\r?\n", " ", text)


def _read_header() -> str:
    try:
        text = OTA_LAYOUT_HEADER.read_text(encoding="utf-8")
    except OSError as exc:
        sys.exit(f"check_trial_marker.py: cannot read {OTA_LAYOUT_HEADER}: {exc}")
    return _join_continuations(_strip_comments(text))


def _parse_int_macro(text: str, name: str) -> int:
    m = re.search(rf"#define\s+{name}\s+(0[xX][0-9A-Fa-f]+|\d+)[uU]?\b", text)
    if m is None:
        sys.exit(f"check_trial_marker.py: could not parse {name} out of {OTA_LAYOUT_HEADER}")
    return int(m.group(1), 0)


def _parse_magic_macro(text: str) -> bytes:
    m = re.search(r"#define\s+OTA_TRIAL_MARKER_MAGIC_BYTES\s+\{([^}]*)\}", text)
    if m is None:
        sys.exit(
            "check_trial_marker.py: could not parse OTA_TRIAL_MARKER_MAGIC_BYTES out of "
            f"{OTA_LAYOUT_HEADER}"
        )
    chars = re.findall(r"'(.)'", m.group(1))
    return bytes(ord(c) for c in chars)


_HEADER_TEXT = _read_header()
MAGIC = _parse_magic_macro(_HEADER_TEXT)
STRUCT_VER = _parse_int_macro(_HEADER_TEXT, "OTA_TRIAL_MARKER_STRUCT_VER")
CAP_CONFIRM = _parse_int_macro(_HEADER_TEXT, "OTA_TRIAL_CAP_CONFIRM")
SCAN_LIMIT = _parse_int_macro(_HEADER_TEXT, "OTA_TRIAL_SCAN_LIMIT")
MIN_OFFSET = _parse_int_macro(_HEADER_TEXT, "OTA_TRIAL_MARKER_MIN_OFFSET")

if len(MAGIC) != 16:
    sys.exit(
        f"check_trial_marker.py: parsed OTA_TRIAL_MARKER_MAGIC_BYTES has "
        f"{len(MAGIC)} bytes, expected 16 -- {OTA_LAYOUT_HEADER} parse went wrong"
    )


def find_marker(image: bytes, limit: int = SCAN_LIMIT, min_offset: int = MIN_OFFSET):
    """Return (offset, struct_version, capability_flags) for the first
    marker found in image[:limit] at or past min_offset, or None --
    mirrors ota_image_trial_capable()'s scan exactly (same window, same
    floor, same 4-byte stride, same field layout)."""
    window = image[:limit]
    off = min_offset
    while off + MARKER_LEN <= len(window):
        if window[off : off + 16] == MAGIC:
            struct_ver, cap = struct.unpack_from("<HH", window, off + 16)
            return off, struct_ver, cap
        off += 4
    return None


def _trial_marker_vma(elf_path: Path) -> int:
    """Read the `.trial_marker` section's link-time VMA out of the ELF via
    arm-none-eabi-objdump -- the adjacency check's source of truth for
    "where the linker script actually put it", independent of this
    script's own bounded scan (which only proves "somewhere findable")."""
    objdump = shutil.which("arm-none-eabi-objdump")
    if objdump is None:
        sys.exit("check_trial_marker.py: arm-none-eabi-objdump not found on PATH")
    try:
        out = subprocess.run(
            [objdump, "-h", str(elf_path)], capture_output=True, text=True, check=True
        ).stdout
    except subprocess.CalledProcessError as exc:
        sys.exit(f"check_trial_marker.py: {objdump} -h {elf_path} failed: {exc.stderr}")
    for line in out.splitlines():
        parts = line.split()
        # Section header line: "<idx> <name> <size> <vma> <lma> <file off> <algn>"
        if len(parts) >= 4 and parts[1] == ".trial_marker":
            return int(parts[3], 16)
    sys.exit(f"check_trial_marker.py: no .trial_marker section found in {elf_path}")


def check(bin_path: Path, elf_path: Path | None, slot_base: int | None) -> int:
    image = bin_path.read_bytes()
    found = find_marker(image)
    if found is None:
        print(
            f"error: {bin_path}: no trial-capability marker found at or past offset "
            f"0x{MIN_OFFSET:x} in the first {SCAN_LIMIT} bytes -- this image would "
            "commit/rollback CONFIRMED (no watchdog safety net). See src/trial_marker.c / "
            "toolchain/gd32g553_app_slot.ld.in.",
            file=sys.stderr,
        )
        return 1
    offset, struct_ver, cap = found
    hexbytes = image[offset : offset + MARKER_LEN].hex()
    print(f"{bin_path}: trial marker found at offset 0x{offset:04x}: {hexbytes}")

    if elf_path is not None and slot_base is not None:
        vma = _trial_marker_vma(elf_path)
        expected_offset = vma - slot_base
        if expected_offset != offset:
            print(
                f"error: {bin_path}: marker found at offset 0x{offset:x} in the .bin, "
                f"but the linker placed `.trial_marker` at VMA 0x{vma:x} (slot base "
                f"0x{slot_base:x} -> expected offset 0x{expected_offset:x}) in {elf_path}. "
                "The marker is not sitting where toolchain/gd32g553_app_slot.ld.in's "
                "adjacency to `.vectors` should put it.",
                file=sys.stderr,
            )
            return 1
        print(f"{bin_path}: marker offset matches .trial_marker's linked VMA (adjacency OK)")

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
    ap.add_argument(
        "--elf", type=Path, default=None, help="linked slot .elf (enables the adjacency check)"
    )
    ap.add_argument(
        "--slot-base",
        type=lambda s: int(s, 0),
        default=None,
        help="slot's absolute flash base, e.g. 0x0800A000 (enables the adjacency check)",
    )
    args = ap.parse_args(argv)
    if (args.elf is None) != (args.slot_base is None):
        ap.error("--elf and --slot-base must be given together")
    return check(args.bin_path, args.elf, args.slot_base)


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
