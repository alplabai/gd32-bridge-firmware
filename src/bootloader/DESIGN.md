# GD32 application bootloader

Status: **Path-A implemented, gated — silicon-validated 2026-06-04 for the
A→B update + rollback direction; B→A not yet exercised.** The OTA opcode
range `0xF0..0xFF` reserved in
[`docs/gd32-bridge-protocol.md` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/docs/gd32-bridge-protocol.md)
§10 routes through `bl_dispatch_ota` into the state machine in
[`../ota.c`](../ota.c) (BEGIN/WRITE_CHUNK/VERIFY/COMMIT/ROLLBACK/GET_STATE/
ABORT, CRC-32, A/B metadata; FMC backend in `hal/fmc_ota.c`).

**Safe-by-default:** destructive flashing is armed only with
`-DBRIDGE_OTA_PARTITIONED` (default **OFF**); otherwise every handler still
returns `STATUS_NOSUPPORT` and no flash is touched, so the full-flash image
cannot brick itself. When armed, the CMake build emits the partitioned set:
a 32 KB bootloader (`src/boot/boot_main.c` + `toolchain/gd32g553_bootloader.ld`)
that validates the active A/B slot and jumps (MSP/VTOR), plus the app linked
for each slot (`toolchain/gd32g553_app_slot.ld.in`, `.ramfunc` in RAM; the app
sets `SCB->VTOR` to its slot base). The host OTA opcodes already exist in
[`chips/gd32g553/` (alp-sdk)](https://github.com/alplabai/alp-sdk/tree/main/chips/gd32g553/)
and the firmware payloads are reconciled to them.

**SILICON-VALIDATED 2026-06-04** (bench, protocol v0.6): boot/validate/jump,
slot relocation, dual-bank FMC-from-RAM, and the full stream → verify →
commit → boot-new-slot → rollback cycle proven end-to-end over the 25 MHz
link, including two GD32 self-reboots through this bootloader.  This run
exercised the **A→B direction and rollback only** — the B→A direction has
**not** been exercised, which matters because the slot map's overlap with
the dual-bank boundary at `0x08040000` bites in that direction.  Still
HIL-gated: a bad bootloader bricks the part, and this HW revision has no
host-driven SWD reflash, so recovery needs a bench SWD probe on the
physical board; the default (unarmed) image remains the
cannot-brick-itself configuration.  First-flash of a partitioned part
needs the factory metadata record from
[`../../tools/gen_ota_metadata.py`](../../tools/gen_ota_metadata.py) at
`0x08008000`.

`OTA_COMMIT` and `OTA_ROLLBACK` both reset the MCU (`ota_system_reset()`,
`src/ota.c`) **before** the reply is staged — the reply's `STATUS_OK` is
never observed on the wire; the host must treat an all-`0x00` drain
following either opcode as "rebooting", not as an error, and re-probe with
`OTA_GET_STATE` / `CMD_GET_BUILD_ID` once the link returns to confirm which
image is now active. This was already exercised, unremarked, by the
2026-06-04 bench cycle above.

Integration detail beyond the public flash-layout and wire contract is tracked
outside this repo; this file documents the recoverable public bootloader path.

## OTA opcode contract

| Opcode                | Code |
|-----------------------|------|
| `CMD_OTA_BEGIN`       | 0xF0 |
| `CMD_OTA_WRITE_CHUNK` | 0xF1 |
| `CMD_OTA_VERIFY`      | 0xF2 |
| `CMD_OTA_COMMIT`      | 0xF3 |
| `CMD_OTA_ROLLBACK`    | 0xF4 |
| `CMD_OTA_GET_STATE`   | 0xF5 |
| `CMD_OTA_ABORT`       | 0xF6 |

Host driver code that talks to the bridge can call these opcodes
today; the scaffold will reply `STATUS_NOSUPPORT` against any
firmware build that lacks the body.  This is the same degradation
path the protocol uses for any reserved-but-unimplemented opcode
(see alp-sdk `docs/gd32-bridge-protocol.md` §6).

## See also

* [`docs/gd32-bridge-protocol.md` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/docs/gd32-bridge-protocol.md)
  §10 -- protocol-level reservation of the OTA opcode range.
* [`docs/gd32-bridge.md` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/docs/gd32-bridge.md)
  "Flashing" -- end-user-visible upgrade paths.
* [`chips/gd32_swd/` (alp-sdk)](https://github.com/alplabai/alp-sdk/tree/main/chips/gd32_swd/) --
  host-driven SWD recovery path (Path B in the protocol spec).
