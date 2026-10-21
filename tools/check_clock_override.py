#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Fail unless a linked GD32 image was built with the IRC8M clock override.

The stock vendor SystemInit() waits forever on HXTALSTB, which never sets on
the E1M-V2M101 SoM, so an image built against it hangs before main().  Two
independent checks, plus a third for the HXTAL-bypass clock path
(either failing fails the release):

  1. compile_commands.json compiled system_gd32g5x3.c from an overrides/ path
     and never from the vendor upstream tree.
  2. the linked ELF's SystemInit() never sets HXTALEN (RCU_CTL bit 16, i.e. an
     `orr ... #65536`), which the stock HXTAL path does first.

  3. the external-clock path (hal/gd32/clock_hw.c, run from bridge_hw_init()
     and NOT from SystemInit(), so check 2 stays valid) is linked, and sets
     HXTALBPS (RCU_CTL bit 18, `orr ... #262144`) -- the bit the stock path
     never set.  With --irc8m-only (the -DBRIDGE_CLOCK_HXTAL=OFF build) the
     inverse holds: the HXTAL bring-up code must be absent.

Usage: check_clock_override.py [--irc8m-only] [--bootloader ELF]...
                               COMPILE_COMMANDS.json [APP_ELF...]

The bootloader is named explicitly with --bootloader (repeatable), never
guessed from a file name: it stays on the SystemInit() clock and must not link
the HXTAL bring-up in EITHER build, so it is checked for that instead of for the
HXTAL-bypass path the application images must have.
"""
import json
import pathlib
import re
import subprocess
import sys

NAME = "system_gd32g5x3.c"


BYPASS_FN = "hw_hxtal_bypass_enable"


def main(argv):
    irc8m_only = False
    bootloaders = []
    rest = []
    it = iter(argv[1:])
    for a in it:
        if a == "--irc8m-only":
            irc8m_only = True
        elif a == "--bootloader":
            bootloaders.append(next(it, None))
        else:
            rest.append(a)
    if None in bootloaders or len(rest) < 1 or (len(rest) < 2 and not bootloaders):
        print(__doc__)
        return 2
    cdb, elfs = rest[0], rest[1:]
    srcs = [e["file"].replace("\\", "/") for e in json.loads(pathlib.Path(cdb).read_text())
            if e["file"].replace("\\", "/").endswith("/" + NAME)]
    if not srcs or any("/overrides/" not in s for s in srcs):
        print(f"FAIL: {NAME} not built from an overrides/ path: {srcs}")
        return 1
    for elf in bootloaders + elfs:
        is_bootloader = elf in bootloaders
        dis = subprocess.run(["arm-none-eabi-objdump", "-d", "--disassemble=SystemInit", elf],
                             check=True, capture_output=True, text=True).stdout
        if "<SystemInit>:" not in dis:
            print(f"FAIL: {elf}: no SystemInit() found")
            return 1
        if re.search(r"\borr(\.w)?\b.*#(65536|0x10000)\b", dis):
            print(f"FAIL: {elf}: SystemInit() enables HXTAL (stock clock init)")
            return 1
        byp = subprocess.run(["arm-none-eabi-objdump", "-d", f"--disassemble={BYPASS_FN}", elf],
                             check=True, capture_output=True, text=True).stdout
        has_fn = f"<{BYPASS_FN}>:" in byp
        if is_bootloader:
            # The bootloader stays on the SystemInit() clock; it must not link
            # the HXTAL bring-up at all, in either build.
            if has_fn:
                print(f"FAIL: {elf}: bootloader links the HXTAL bring-up ({BYPASS_FN})")
                return 1
            print(f"ok: {elf}: IRC8M override linked, bootloader has no HXTAL path")
            continue
        if irc8m_only and has_fn:
            print(f"FAIL: {elf}: IRC8M-only build still links the HXTAL bring-up ({BYPASS_FN})")
            return 1
        if not irc8m_only:
            if not has_fn:
                print(f"FAIL: {elf}: HXTAL-bypass clock path ({BYPASS_FN}) not linked")
                return 1
            if not re.search(r"\borr(\.w)?\b.*#(262144|0x40000)\b", byp):
                print(f"FAIL: {elf}: {BYPASS_FN} never sets HXTALBPS (RCU_CTL bit 18)")
                return 1
        print(f"ok: {elf}: IRC8M override linked, " +
              ("IRC8M-only build" if irc8m_only else "HXTAL-bypass path present"))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
