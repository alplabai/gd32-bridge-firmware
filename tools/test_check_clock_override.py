#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Unit tests for check_clock_override.py: objdump output is faked, so every
verdict (override compiled, HXTAL-bypass path present/absent per shape, the
bootloader named explicitly) is exercised without a toolchain."""
import contextlib
import io
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent))
import check_clock_override as cco  # noqa: E402

GOOD_INIT = "08000000 <SystemInit>:\n 8000002:\tmov r0, r1\n"
STOCK_INIT = "08000000 <SystemInit>:\n 8000002:\torr.w r3, r3, #65536\t@ 0x10000\n"
BYPASS_OK = ("08001000 <hw_hxtal_bypass_enable>:\n"
             " 8001004:\torr.w r2, r2, #262144\t@ 0x40000\n")
BYPASS_NO_ORR = "08001000 <hw_hxtal_bypass_enable>:\n 8001004:\tbic.w r2, r2, #65536\n"
NO_BYPASS = ""


class Fake:
    """Maps an ELF path to (SystemInit disassembly, bypass-function disassembly)."""

    def __init__(self, table):
        self.table = table

    def __call__(self, cmd, **kw):
        elf = cmd[-1]
        init, byp = self.table[elf]
        out = init if "--disassemble=SystemInit" in cmd else byp
        return subprocess.CompletedProcess(cmd, 0, stdout=out, stderr="")


def run(argv, table, override=True):
    with tempfile.TemporaryDirectory() as d:
        cdb = pathlib.Path(d) / "compile_commands.json"
        src = "/x/overrides/system_gd32g5x3.c" if override else "/x/upstream/system_gd32g5x3.c"
        cdb.write_text(json.dumps([{"file": src}]))
        out = io.StringIO()
        with contextlib.redirect_stdout(out), mock.patch.object(
                cco.subprocess, "run", Fake(table)):
            rc = cco.main(["check_clock_override.py"] + [a.replace("CDB", str(cdb)) for a in argv])
        return rc, out.getvalue()


class CheckClockOverride(unittest.TestCase):
    def test_default_shape_passes_with_bootloader_named_explicitly(self):
        rc, out = run(["--bootloader", "boot.elf", "CDB", "app.elf"],
                      {"boot.elf": (GOOD_INIT, NO_BYPASS), "app.elf": (GOOD_INIT, BYPASS_OK)})
        self.assertEqual(rc, 0, out)

    def test_bootloader_is_not_guessed_from_its_file_name(self):
        # A file called gd32-bootloader.elf given as a plain image is an app: it
        # must carry the HXTAL path.
        rc, _ = run(["CDB", "gd32-bootloader.elf"],
                    {"gd32-bootloader.elf": (GOOD_INIT, NO_BYPASS)})
        self.assertEqual(rc, 1)

    def test_bootloader_linking_the_hxtal_path_fails(self):
        rc, out = run(["--bootloader", "boot.elf", "CDB"],
                      {"boot.elf": (GOOD_INIT, BYPASS_OK)})
        self.assertEqual(rc, 1)
        self.assertIn("bootloader links the HXTAL bring-up", out)

    def test_app_without_the_bypass_path_fails_in_the_default_shape(self):
        rc, out = run(["CDB", "app.elf"], {"app.elf": (GOOD_INIT, NO_BYPASS)})
        self.assertEqual(rc, 1)
        self.assertIn("not linked", out)

    def test_bypass_function_that_never_sets_hxtalbps_fails(self):
        rc, out = run(["CDB", "app.elf"], {"app.elf": (GOOD_INIT, BYPASS_NO_ORR)})
        self.assertEqual(rc, 1)
        self.assertIn("never sets HXTALBPS", out)

    def test_irc8m_only_shape(self):
        rc, _ = run(["--irc8m-only", "CDB", "app.elf"], {"app.elf": (GOOD_INIT, NO_BYPASS)})
        self.assertEqual(rc, 0)
        rc, out = run(["--irc8m-only", "CDB", "app.elf"], {"app.elf": (GOOD_INIT, BYPASS_OK)})
        self.assertEqual(rc, 1)
        self.assertIn("IRC8M-only build still links", out)

    def test_stock_system_init_fails(self):
        rc, out = run(["CDB", "app.elf"], {"app.elf": (STOCK_INIT, BYPASS_OK)})
        self.assertEqual(rc, 1)
        self.assertIn("enables HXTAL", out)

    def test_override_not_compiled_fails(self):
        rc, out = run(["CDB", "app.elf"], {"app.elf": (GOOD_INIT, BYPASS_OK)}, override=False)
        self.assertEqual(rc, 1)
        self.assertIn("overrides/", out)

    def test_bad_arguments_print_usage(self):
        for argv in ([], ["--bootloader"], ["CDB"]):
            with self.subTest(argv=argv):
                rc, out = run(argv, {})
                self.assertEqual(rc, 2)
                self.assertIn("Usage", out)


if __name__ == "__main__":
    unittest.main()
