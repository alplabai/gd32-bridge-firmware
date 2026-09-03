## What this changes

<!-- One paragraph. If it fixes an alp-sdk issue, link it: alp-sdk#NNNN -->

## Bench evidence

<!--
REQUIRED for any change touching a transport, the OTA path, or a supervised
output (rail, reset, PWM). CI's `gd32 backend build` job compiles and links
the real gd32 backend against the real GD32 firmware library, but that is
compile-and-link only -- it cannot tell you whether the link survives on
silicon, and it certainly cannot tell you what a rail did.

Paste the actual console output, not a summary. Include BOTH sides where the
change has two (host counters and firmware counters together).
-->

- [ ] Ran on a real E1M-X V2N module; output pasted above
- [ ] Not applicable (docs / build / stub-only change)

## What I did NOT verify

<!-- "Built but not flashed", "SPI only, I2C untested" are useful. A PASS a
     reader cannot check is not. -->

## Wire protocol

- [ ] Unchanged
- [ ] Changed — `tests/gen_protocol_vectors.py` re-run and
      `tests/protocol_vectors.txt` committed in the SAME change. Note the other
      consumer of these vectors is alp-sdk's host-side driver tests, in a
      different repository: nothing here fails if that side is not updated too.

## Supervised outputs

- [ ] This change cannot alter a rail, reset or PWM
- [ ] It can — the affected outputs and their safe states are described above

## Checks

- [ ] `cmake -DBRIDGE_HAL_BACKEND=stub` builds
- [ ] `python3 tests/gen_protocol_vectors.py` leaves no diff
- [ ] `clang-format --dry-run --Werror` silent on changed C/H
