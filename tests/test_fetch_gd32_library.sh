#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Alp Lab AB
#
# Safety tests for tools/fetch_gd32_library.sh.  Uses the already-fetched
# vendor/gd32_firmware_library/upstream as a seed (run the fetch script first).
set -euo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
script="$root/tools/fetch_gd32_library.sh"
seed="$root/vendor/gd32_firmware_library/upstream"
tmp="$(mktemp -d)"; trap 'rm -rf "$tmp"' EXIT
fail() { echo "FAIL: $*" >&2; exit 1; }

# 1. A non-empty foreign directory is refused and NOT deleted.
mkdir -p "$tmp/foreign" && echo keep > "$tmp/foreign/precious.txt"
if "$script" "$tmp/foreign" >/dev/null 2>&1; then fail "foreign dir accepted"; fi
[ -f "$tmp/foreign/precious.txt" ] || fail "foreign dir was deleted"

# 2. A verified checkout is reused (no network).
[ -d "$seed/.git" ] || fail "seed missing: run tools/fetch_gd32_library.sh first"
cp -a "$seed" "$tmp/good"
"$script" "$tmp/good" | grep -q "already present" || fail "verified checkout not reused"

# 3. An extra untracked .c in a cached checkout fails verification and is kept.
cp -a "$seed" "$tmp/extra"
echo 'int evil;' > "$tmp/extra/Firmware/GD32G5x3_standard_peripheral/Source/gd32g5x3_evil.c"
if "$script" "$tmp/extra" >/dev/null 2>&1; then fail "extra .c accepted"; fi
[ -f "$tmp/extra/Firmware/GD32G5x3_standard_peripheral/Source/gd32g5x3_evil.c" ] || fail "dirty checkout was deleted"

# 4. A clone of a different origin is refused, not deleted.
cp -a "$seed" "$tmp/other"
git -C "$tmp/other" remote set-url origin https://example.invalid/x.git
if "$script" "$tmp/other" >/dev/null 2>&1; then fail "foreign origin accepted"; fi
[ -d "$tmp/other/.git" ] || fail "foreign-origin clone was deleted"
echo "ok: fetch_gd32_library.sh safety tests passed"
