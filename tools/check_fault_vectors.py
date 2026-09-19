#!/usr/bin/env python3
# Copyright 2026 Alp Lab AB
# SPDX-License-Identifier: Apache-2.0
"""Verify that every real image owns its core-fault vectors.

The vendor startup exposes weak aliases for the five exception handlers. A
successful link therefore does not prove that the firmware's diagnostic
handlers won. This check requires strong text symbols and verifies that the
corresponding words in the emitted .vectors section name those symbols.
"""

from __future__ import annotations

import argparse
import pathlib
import struct
import subprocess
import tempfile


HANDLER_SLOTS = {
    "NMI_Handler": 2,
    "HardFault_Handler": 3,
    "MemManage_Handler": 4,
    "BusFault_Handler": 5,
    "UsageFault_Handler": 6,
}


def read_symbols(nm: str, elf: pathlib.Path) -> dict[str, tuple[int, str]]:
    output = subprocess.run(
        [nm, "--defined-only", "--format=posix", str(elf)],
        check=True,
        capture_output=True,
        text=True,
    ).stdout
    symbols: dict[str, tuple[int, str]] = {}
    for line in output.splitlines():
        fields = line.split()
        if len(fields) >= 3:
            name, kind, value = fields[:3]
            try:
                symbols[name] = (int(value, 16), kind)
            except ValueError:
                continue
    return symbols


def read_vectors(objcopy: str, elf: pathlib.Path) -> bytes:
    with tempfile.TemporaryDirectory() as temp_dir:
        output = pathlib.Path(temp_dir) / "vectors.bin"
        subprocess.run(
            [objcopy, "-O", "binary", "--only-section=.vectors", str(elf), str(output)],
            check=True,
        )
        return output.read_bytes()


def check_image(nm: str, objcopy: str, elf: pathlib.Path) -> list[str]:
    errors: list[str] = []
    if not elf.is_file():
        return [f"{elf}: image does not exist"]

    symbols = read_symbols(nm, elf)
    default = symbols.get("Default_Handler")
    vectors = read_vectors(objcopy, elf)

    for handler, slot in HANDLER_SLOTS.items():
        symbol = symbols.get(handler)
        if symbol is None:
            errors.append(f"{elf}: {handler} is undefined")
            continue
        address, kind = symbol
        if kind != "T":
            errors.append(f"{elf}: {handler} is not a strong text symbol (nm type {kind})")
        if default is not None and address == default[0]:
            errors.append(f"{elf}: {handler} still aliases Default_Handler")

        offset = slot * 4
        if len(vectors) < offset + 4:
            errors.append(f"{elf}: .vectors is too short for slot {slot} ({handler})")
            continue
        vector = struct.unpack_from("<I", vectors, offset)[0]
        expected = address | 1
        if vector != expected:
            errors.append(
                f"{elf}: vector slot {slot} is {vector:#010x}, "
                f"expected {handler}|1 ({expected:#010x})"
            )
    return errors


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--nm", default="arm-none-eabi-nm")
    parser.add_argument("--objcopy", default="arm-none-eabi-objcopy")
    parser.add_argument("elf", nargs="+", type=pathlib.Path)
    args = parser.parse_args()

    errors: list[str] = []
    for elf in args.elf:
        image_errors = check_image(args.nm, args.objcopy, elf)
        if image_errors:
            errors.extend(image_errors)
        else:
            print(f"{elf}: fault symbols and vectors OK")

    for error in errors:
        print(f"ERROR: {error}")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
