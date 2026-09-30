#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fail unless a linked GD32 image was built with the IRC8M clock override.

The stock vendor SystemInit() waits forever on HXTALSTB, which never sets on
the E1M-V2M101 SoM, so an image built against it hangs before main().  Two
independent checks (either failing fails the release):

  1. compile_commands.json compiled system_gd32g5x3.c from an overrides/ path
     and never from the vendor upstream tree.
  2. the linked ELF's SystemInit() never sets HXTALEN (RCU_CTL bit 16, i.e. an
     `orr ... #65536`), which the stock HXTAL path does first.

Usage: check_clock_override.py COMPILE_COMMANDS.json ELF [ELF...]
"""
import json
import pathlib
import re
import subprocess
import sys

NAME = "system_gd32g5x3.c"


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    cdb, elfs = argv[1], argv[2:]
    srcs = [e["file"].replace("\\", "/") for e in json.loads(pathlib.Path(cdb).read_text())
            if e["file"].replace("\\", "/").endswith("/" + NAME)]
    if not srcs or any("/overrides/" not in s for s in srcs):
        print(f"FAIL: {NAME} not built from an overrides/ path: {srcs}")
        return 1
    for elf in elfs:
        dis = subprocess.run(["arm-none-eabi-objdump", "-d", "--disassemble=SystemInit", elf],
                             check=True, capture_output=True, text=True).stdout
        if "<SystemInit>:" not in dis:
            print(f"FAIL: {elf}: no SystemInit() found")
            return 1
        if re.search(r"\borr(\.w)?\b.*#(65536|0x10000)\b", dis):
            print(f"FAIL: {elf}: SystemInit() enables HXTAL (stock clock init)")
            return 1
        print(f"ok: {elf}: IRC8M clock override linked")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
