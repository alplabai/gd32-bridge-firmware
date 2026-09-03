#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""
Generate an initial OTA A/B metadata record for the GD32 bridge
partitioned layout (factory provisioning).

The Path-A bootloader (src/boot/boot_main.c) boots ONLY a slot that a
CRC-valid metadata record marks active+valid -- a freshly partitioned
part with erased metadata pages idles in the recovery WFI loop.  This
tool emits the record binary that an external SWD probe flashes to
OTA_META_REC0 (0x08008000) alongside the bootloader (0x08000000) and
the slot-A application image (0x0800A000):

    python3 gen_ota_metadata.py --slot-image gd32-bridge-slot-a.bin \
        --out ota-meta-rec0.bin
    # J-Link: erase 0x08008000 0x0800A000
    # J-Link: loadbin ota-meta-rec0.bin,0x08008000

The erase MUST cover both OTA_META_REC0 (0x08008000) and OTA_META_REC1
(0x08008800): both the bootloader and the application pick the record
with the highest `counter`, so a stale REC1 left over from a returned
board silently keeps the old slot active even after REC0 is rewritten
(#25 B13).

Layout mirrors ota_meta_record_t (src/ota_layout.h, struct v2 with
PER-SLOT image descriptors) and the CRC-32 mirrors src/crc32.c
(IEEE 802.3 reflected == zlib.crc32).  Keep all three in lockstep.
"""

from __future__ import annotations

import argparse
import pathlib
import struct
import sys
import zlib

OTA_META_MAGIC = 0x4F544D31  # "OTM1"
OTA_META_STRUCT_VER = 2
OTA_SLOT_SIZE = 0x0003B000

# Mirrors src/ota_layout.h -- keep these four in lockstep with that file.
OTA_SLOT_A_BASE = 0x0800A000
OTA_SLOT_B_BASE = 0x08045000
OTA_IMG_MIN_LEN = 8
OTA_SRAM_BASE = 0x20000000
OTA_SRAM_END = 0x20040000

SLOT_NAMES = {"a": 0, "b": 1}
SLOT_BASES = {0: OTA_SLOT_A_BASE, 1: OTA_SLOT_B_BASE}


def check_bootable(active_slot: int, img: bytes) -> None:
    """Reject an image the bootloader is guaranteed not to boot.

    Mirrors ota_image_bootable() in src/ota_layout.h byte-for-byte (keep
    the two in step): length covers the MSP+reset head, the initial MSP
    is a word-aligned address inside SRAM, and the reset vector has the
    Thumb bit set and lands inside the image.  The bootloader's
    active_slot_valid() (src/boot/boot_main.c) applies exactly this test
    before it will jump to a slot -- a record that passes here but not
    there is the generator drifting from the firmware, not a new class
    of bug, so raise loudly rather than let it happen quietly.
    """
    if len(img) < OTA_IMG_MIN_LEN:
        raise ValueError(f"slot image is {len(img)} bytes; "
                         f"must be at least {OTA_IMG_MIN_LEN} bytes "
                         f"(the initial-MSP + reset-vector head the "
                         f"bootloader reads before it jumps)")

    base = SLOT_BASES[active_slot]
    msp, reset = struct.unpack_from("<II", img, 0)

    if (msp & 3) != 0 or not (OTA_SRAM_BASE <= msp <= OTA_SRAM_END):
        raise ValueError(
            f"initial MSP {msp:#010x} (image bytes[0:4]) is not a "
            f"word-aligned address in [{OTA_SRAM_BASE:#010x}, "
            f"{OTA_SRAM_END:#010x}] -- this is not a valid application "
            f"image for slot {'A' if active_slot == 0 else 'B'} "
            f"(wrong file, an ELF instead of a raw .bin, or an image "
            f"linked for a different target)")

    reset_addr = reset & ~1
    if (reset & 1) == 0:
        raise ValueError(
            f"reset vector {reset:#010x} (image bytes[4:8]) has the "
            f"Thumb bit clear -- Cortex-M requires bit 0 set on every "
            f"code address; the bootloader will hard-fault on entry. "
            f"Expected an odd address, e.g. {reset | 1:#010x}")
    if not (base <= reset_addr < base + len(img)):
        raise ValueError(
            f"reset vector {reset:#010x} points to {reset_addr:#010x}, "
            f"which is outside [{base:#010x}, {base + len(img):#010x}) "
            f"-- the image is not linked for slot "
            f"{'A' if active_slot == 0 else 'B'} at {base:#010x}")


def build_record(active_slot: int, counter: int, img: bytes,
                 fw_version: int) -> bytes:
    """Pack one ota_meta_record_t marking `active_slot` valid."""
    if not (1 <= len(img) <= OTA_SLOT_SIZE):
        raise ValueError(f"slot image is {len(img)} bytes; "
                         f"must be within (0, {OTA_SLOT_SIZE:#x}]")
    check_bootable(active_slot, img)

    fw_ver = [0, 0]
    img_len = [0, 0]
    img_crc = [0, 0]
    fw_ver[active_slot] = fw_version
    img_len[active_slot] = len(img)
    img_crc[active_slot] = zlib.crc32(img) & 0xFFFFFFFF

    body = struct.pack(
        "<III BB 2x 2I 2I 2I",
        OTA_META_MAGIC,
        OTA_META_STRUCT_VER,
        counter,
        active_slot,
        1 << active_slot,          # slot_valid bitmask
        *fw_ver,
        *img_len,
        *img_crc,
    )
    rec_crc = zlib.crc32(body) & 0xFFFFFFFF
    return body + struct.pack("<I", rec_crc)


def _u32_counter(s: str) -> int:
    """argparse type for --counter: a plain range error, not a struct.error
    traceback out of struct.pack() at the bottom of build_record()."""
    v = int(s, 0)
    if not (0 <= v <= 0xFFFFFFFF):
        raise argparse.ArgumentTypeError(
            f"--counter {v} is out of range; must fit a u32 "
            f"(0..{0xFFFFFFFF})")
    return v


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--slot-image", type=pathlib.Path, required=True,
                        help="the application image that will occupy the active slot")
    parser.add_argument("--active-slot", choices=("a", "b"), default="a",
                        help="which slot the image is flashed to (default: a)")
    parser.add_argument("--counter", type=_u32_counter, default=1,
                        help="metadata generation counter (default: 1 = factory)")
    parser.add_argument("--fw-version", type=lambda s: int(s, 0), default=0,
                        help="packed firmware version for the record (default: 0 = unknown)")
    parser.add_argument("--out", type=pathlib.Path, required=True,
                        help="output record binary (flash to 0x08008000)")
    args = parser.parse_args(argv)

    img = args.slot_image.read_bytes()
    active_slot = SLOT_NAMES[args.active_slot]
    try:
        rec = build_record(active_slot, args.counter, img, args.fw_version)
    except ValueError as exc:
        # Fail loudly with the reason, not a traceback: this is the sole
        # factory path onto a new part, and a bad image here means a
        # part that idles at the recovery WFI on first boot (#25 B12).
        print(f"error: refusing to write an unbootable metadata record: "
              f"{exc}", file=sys.stderr)
        return 1

    args.out.write_bytes(rec)
    msp, reset = struct.unpack_from("<II", img, 0)
    print(f"wrote {len(rec)}-byte metadata record: active=slot-"
          f"{args.active_slot.upper()} counter={args.counter} "
          f"img_len={len(img)} img_crc32=0x{zlib.crc32(img) & 0xFFFFFFFF:08X} "
          f"msp={msp:#010x} reset={reset:#010x}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
