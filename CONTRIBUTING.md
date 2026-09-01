<!--
Copyright (c) 2026 Alp Lab AB
SPDX-License-Identifier: Apache-2.0
-->

# Contributing to the GD32 bridge firmware

This firmware runs on the GD32G553 supervisor on an E1M-X V2N SoM and speaks a
wire protocol to a host driver in
[`alp-sdk`](https://github.com/alplabai/alp-sdk). Two things shape everything
below: **this chip is a supervisor** — it drives rails, resets and PWMs on real
hardware — and **the wire contract has a second half in another repository**.

## Branch strategy

`main` is the only long-lived branch, and it is protected. Every change —
including a one-line fix, a revert of your own work, and a docs typo — starts on
its own branch and lands through a pull request.

```sh
git checkout main && git pull
git checkout -b fix/<topic>        # or feat/, docs/, chore/
# ... work, run the gates below ...
git push -u origin fix/<topic>
gh pr create --base main
```

Prefixes: `fix/`, `feat/`, `docs/`, `chore/`, `perf/`. Reference the alp-sdk
issue where one exists (`alp-sdk#1730`) — most work here is tracked there,
because the defect is usually visible from the host side first.

**Why no `dev` branch**, unlike alp-sdk: this repo has a single long-lived
artifact line and a bench that serialises anyway. An integration branch would
add a merge step without adding a place for anything to be integrated. If that
changes, revisit it; until then `main` plus short-lived branches is the honest
shape.

Squash-merge is the default, so branch churn and the "try it on the bench"
commits die with the branch instead of becoming permanent history.

## Before you open a PR

```sh
# stub build -- the same one CI runs. This repo does not vendor the GD32
# firmware library, so the stub backend is the compile coverage available
# without one; with an alp-sdk checkout you can also build the gd32 backend by
# adding -DBRIDGE_HAL_BACKEND=gd32 -DGD32_VENDOR_DIR=<path> (see README.md).
# CMAKE_TOOLCHAIN_FILE is not optional: this is Cortex-M33 firmware linked
# against a device linker script. Without it CMake configures against the host
# compiler and the BUILD then dies on the first source file, which rejects
# -mcpu=cortex-m33 -- the configure step itself still succeeds, so the failure
# arrives later than you expect.
cmake -B build/stub -S . -DCMAKE_TOOLCHAIN_FILE=toolchain/arm-none-eabi.cmake -DBRIDGE_HAL_BACKEND=stub
cmake --build build/stub

# host unit tests -- the same ones CI runs, against the real firmware sources
# (ota.c, transport_spi.c, crc32.c, protocol.c, bootloader.c), not mocks, so
# they cover the OTA bounds checks and the SPI CS-framing seams the device
# actually runs.
cmake -S tests/unit -B build-tests -DCMAKE_BUILD_TYPE=Debug
cmake --build build-tests --parallel
ctest --test-dir build-tests --output-on-failure

# wire vectors -- must leave no diff
python3 tests/gen_protocol_vectors.py && git diff --exit-code tests/protocol_vectors.txt

# formatting, on the files you touched (clang-format 22.x)
clang-format -i <your files> && clang-format --dry-run --Werror <your files>
```

### The wire vectors are shared across repositories

`tests/gen_protocol_vectors.py` is the authoritative source for the canonical
vectors, and they have **two** consumers: the firmware tests here, and the
host-side driver tests in alp-sdk under `tests/zephyr/chips/gd32g553/`.

Nothing in this repo fails when the alp-sdk side drifts. So a wire change is not
finished when CI here is green — it needs the matching alp-sdk change, and the
two should land close together. Say in your PR which alp-sdk PR pairs with it.

## Bench evidence is part of the change

A patch that touches a transport, the OTA path, or a supervised output is **not
reviewable without a bench run**. CI builds the stub backend; it cannot tell you
whether the link survives, and it cannot tell you what a rail did.

Paste the actual console output, not a summary:

- Quote the counters from **both** sides where the change has two. Measuring one
  side alone has produced confident wrong conclusions on this bridge before.
- State what you did **not** verify. "SPI tested, I2C not" and "built but not
  flashed" are useful results; a claim of PASS a reader cannot check is not.

### Supervised outputs deserve a sentence of their own

If a change can alter a rail, a reset line or a PWM, say which and what the safe
state is. A wrong duty cycle or an unexpected reset is not a failed test — it is
hardware doing something, and the bench boards are shared.

## OTA

`BRIDGE_OTA_PARTITIONED` is **default-OFF** and HIL-gated. If you are changing
the OTA path, check whether the code you are editing is even compiled into a
shipped image, and bump `firmware-version.txt` when you ship one.

## What not to commit

Build output (`build*/` — that covers `build/`, `build/stub` and `build-tests`
alike), Python `__pycache__/`, MSYS `*.stackdump` files, and anything from the GD32
firmware library. CI rejects tracked `.out`, `.map`, `.o`, `.elf`, `.bin` and
`.hex` files — the sibling cc3501e repo had a 14 MB `.out` and several crash
dumps swept in by a `git add -A` on the Windows bench.

## Attribution

Commits and PR bodies are attributed to Alp Lab AB and the human author. No
AI-assistant footers, co-author trailers, or session links.
