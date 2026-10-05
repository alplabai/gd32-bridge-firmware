#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Alp Lab AB
#
# Fetch the GigaDevice GD32G5x3 firmware library from GigaDevice's official
# repository into vendor/gd32_firmware_library/upstream (gitignored).  The
# library is GigaDevice's code under GigaDevice's own terms and is never
# committed here; see THIRD_PARTY_NOTICES.md.
#
# Trust anchor: the clone must land on PIN_COMMIT *and* its Firmware/ tree must
# hash to PIN_TREE (a git tree id is a content hash of every file below it).
# A moved tag or a tampered mirror fails closed.  Idempotent: an existing,
# verified checkout is reused, so a CI/local cache of `upstream/` works offline.
# A non-empty dest that does not verify is never deleted; the script refuses.
#
# Usage: tools/fetch_gd32_library.sh [dest]
#   GD32_LIBRARY_URL  override the clone URL (local mirror/cache); the pins
#                     are still enforced.
set -euo pipefail

URL="${GD32_LIBRARY_URL:-https://github.com/GigaDevice-GD32-MCU/GD32G5x3_Firmware_Library.git}"
TAG="V1.5.0"
PIN_COMMIT="2d4ac55768261800d06d59a45963623fc74adaa2"
PIN_TREE="3807e194d8de55e24fc8160ad9accdd84c503651"   # tree of Firmware/

root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
dest="${1:-$root/vendor/gd32_firmware_library/upstream}"

# Deterministic, byte-exact verification of an existing checkout: the commit and
# Firmware/ tree must match the pins, the checkout must BE the git root of
# $dest (not a parent repo), and nothing may be modified, untracked or ignored
# (an extra .c would be globbed straight into the build).
verify() {
  local top c t
  top="$(git -C "$dest" rev-parse --show-toplevel 2>/dev/null)" || return 1
  [ "$(cd "$top" && pwd -P)" = "$(cd "$dest" && pwd -P)" ] || return 1
  git -C "$dest" update-index --really-refresh >/dev/null 2>&1 || true
  c="$(git -C "$dest" rev-parse HEAD)"
  t="$(git -C "$dest" rev-parse HEAD:Firmware)"
  if [ "$c" != "$PIN_COMMIT" ] || [ "$t" != "$PIN_TREE" ]; then
    echo "error: $dest is commit $c / Firmware tree $t; expected $PIN_COMMIT / $PIN_TREE" >&2
    return 1
  fi
  if [ -n "$(git -C "$dest" status --porcelain --untracked-files=all --ignored)" ]; then
    echo "error: $dest has modified, untracked or ignored files" >&2
    return 1
  fi
}

# Never delete anything we did not create: an existing non-empty dest is only
# reused when it verifies; otherwise refuse and leave it alone.
if [ -e "$dest" ] && [ -n "$(ls -A "$dest" 2>/dev/null)" ]; then
  if [ -d "$dest/.git" ] && [ "$(git -C "$dest" config --get remote.origin.url 2>/dev/null)" = "$URL" ] && verify; then
    echo "GD32 library already present and verified ($PIN_COMMIT)"
    exit 0
  fi
  echo "error: $dest exists and is not a clean, verified clone of $URL; refusing to delete it. Remove it yourself and re-run." >&2
  exit 1
fi

mkdir -p "$(dirname "$dest")"
# autocrlf=false: byte-exact checkout so the clock patch applies on every OS.
git -c core.autocrlf=false -c advice.detachedHead=false \
  clone --quiet --branch "$TAG" --depth 1 "$URL" "$dest"
if ! verify; then
  echo "error: fresh clone failed verification; leaving $dest for inspection" >&2
  exit 1
fi
echo "GD32 library fetched and verified ($PIN_COMMIT, Firmware tree $PIN_TREE)"
