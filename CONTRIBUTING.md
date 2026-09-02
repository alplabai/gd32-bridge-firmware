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

There are two long-lived branches. **`dev` is the integration target** — every
change, including a one-line fix, a revert of your own work, and a docs typo,
branches off `dev` and lands back on `dev` through a pull request. **`main` is
the release line**, and it only ever receives `dev`.

```sh
git checkout dev && git pull
git checkout -b fix/<topic>        # or feat/, docs/, chore/
# ... work, run the gates below ...
git push -u origin fix/<topic>
gh pr create --base dev
```

Prefixes: `fix/`, `feat/`, `docs/`, `chore/`, `perf/`. Reference the issue in this
repository where one exists, and the alp-sdk issue where the defect is tracked
there instead (`alp-sdk#1730`) — much of this work is visible from the host side
first, so the two backlogs both carry parts of it.

**Never target `main` directly.** A PR based on `main` skips the integration step,
and if it was cut from `main` while `dev` is ahead it also carries a stale base.
Check before opening one:

```sh
git fetch origin dev main
git log --oneline origin/dev..HEAD   # every commit the PR would carry -- eyeball it
```

If that list contains commits you did not write for this change, the branch was
cut from the wrong place.

**`Closes #N` does not auto-close on a `dev` merge.** GitHub auto-closes a linked
issue only when the merge lands on the repository's *default* branch, which is
`main`. In a PR targeting `dev` the keyword still records intent and links the
issue, but the issue has to be closed by hand after the merge. Use `Refs #N` when
the change does not fully resolve the issue.

**Why `dev` exists**, having previously not: this tree grew a host test suite and
a set of cross-repo wire obligations, so changes now accumulate that are complete
and gate-green but not yet release-ready. `dev` is where they accumulate. The
bench still serialises, and that has not changed — what changed is that there is
now something to integrate.

Squash-merge is the default, so branch churn and the "try it on the bench"
commits die with the branch instead of becoming permanent history.

## Before you open a PR

```sh
# stub build -- the same one CI runs. The gd32 backend needs the GD32 firmware
# library, which this repo does not vendor, so this is the compile coverage
# available to you.
cmake -B build/stub -S . -DBRIDGE_HAL_BACKEND=stub
cmake --build build/stub

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

Build output (`build/`), MSYS `*.stackdump` files, and anything from the GD32
firmware library. CI rejects tracked `.out`, `.map`, `.o`, `.elf`, `.bin` and
`.hex` files — the sibling cc3501e repo had a 14 MB `.out` and several crash
dumps swept in by a `git add -A` on the Windows bench.

## Attribution

Commits and PR bodies are attributed to Alp Lab AB and the human author. No
AI-assistant footers, co-author trailers, or session links.
