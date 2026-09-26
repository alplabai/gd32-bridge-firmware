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

## Trial/confirm + watchdog fallback

**Bench fact (2026-09-26, E1M-V2M103):** an OTA'd slot-B image passed VERIFY
and COMMIT, then hung *before* `main()` (stock `SystemInit` spinning on
`HXTALSTB`). This bootloader had no watchdog and no confirm step -- it jumped
to the newest CRC-valid slot unconditionally, so the bridge was dead on both
transports (SPI + I2C) until an external SWD recovery.

The fix adds a **TRIAL** flag to the A/B metadata record
(`ota_meta_record_t.flags`, `OTA_META_FLAG_TRIAL`) and a free watchdog
(FWDGT) fallback:

1. `CMD_OTA_COMMIT` / `CMD_OTA_ROLLBACK` mark the slot they just made active
   **TRIAL** (never CONFIRMED) -- `src/ota.c`'s `meta_commit()`.
2. This bootloader reads `RCU_RSTSCK` **once**, up front, to see whether the
   *previous* boot ended in a watchdog reset (`FWDGTRSTF`) -- and never
   clears it (see "reset-cause ownership" below). For each candidate record
   (newest-first, #754's existing fallback order), `ota_boot_candidate_ok()`
   (`src/ota_layout.h`) rejects a TRIAL candidate if a watchdog already fired
   this power cycle -- it hung before confirming once; don't re-try it, fall
   back to the next candidate (the previous, already-confirmed slot).
3. Before jumping into a TRIAL candidate, the bootloader arms the FWDGT:
   IRC32K/256 prescaler, reload 2000 (`OTA_TRIAL_FWDGT_RELOAD` in
   `src/boot/boot_main.c`) -- **~16 s nominal** (IRC32K's factory trim
   drifts a few percent across temperature per the GD32G5x3 datasheet; this
   is a calibration knob, not an exact bound). The debug-hold bit
   (`DBG_FWDGT_HOLD`) is set first so a breakpointed bench session doesn't
   spuriously reset mid-investigation.
4. If the jumped-to image comes up, `main()` calls `ota_boot_init()`
   (`src/ota.c`) *before* the transports start. It reconciles the metadata
   against which slot is **actually** executing (`OTA_RUNNING_SLOT`, the
   same compile-time fact #3's fixes already derive from
   `BRIDGE_APP_SLOT_BASE`):
   - Metadata names **us** TRIAL+active: a fresh trial boot.
     `ota_trial_unconfirmed()` gates the wire -- every opcode
     `protocol_dispatch()` sees, on either link, answers `STATUS_BUSY` with
     an empty payload (never `STATUS_OK` from a session that might still
     revert), and notes that a live frame arrived.
   - Metadata still names the **other** (rejected) slot TRIAL+active: we are
     the bootloader's watchdog fallback. Self-heal the record (repoint
     `active_slot` at us, clear TRIAL) with **no reset** -- we are already
     the safe slot.
5. Once a frame has been noted during an unconfirmed trial, the base-level
   tick (`ota_confirm_tick()`, called next to `ota_erase_tick()` from
   `bridge_hw_tick()`) commits the slot **CONFIRMED** (`flags = 0`) and
   reboots. The extra reset is deliberate: on a fresh, non-TRIAL boot this
   bootloader never arms the FWDGT, so rebooting is the only way to stop the
   watchdog that's been running since the trial boot. If that commit fails
   (flash error), the TRIAL flag is left set and **no reset happens** -- the
   already-armed watchdog is the fallback safety net.

**Reset-cause ownership:** this bootloader only *reads* `RCU_RSTSCK`; it
never clears it. The one deliberate clear point is `ota_system_reset()`
(`hal/fmc_ota.c`), which sets `RCU_RSTSCK.RSTFC` immediately before
`NVIC_SystemReset()` -- so a stale `FWDGTRSTF` left over from an earlier,
unrelated cycle can't masquerade as *this* trial's watchdog fallback. The
app's `CMD_RESET_REASON` handler (`hal/gd32/init.c`) still clears the same
register lazily, on demand, for a real watchdog event a host queries about.

**Cost on fielded units with the OLD (pre-trial) bootloader:** none of this
protects them -- an old bootloader doesn't know about `flags` or
`ota_boot_candidate_ok()` and will still jump unconditionally to the newest
valid slot. Only a bootloader rebuilt with this change gets the watchdog
fallback. On any bootloader that DOES have it, the visible cost is one extra
GD32 self-reboot per successful OTA (the confirm step's deliberate reset).

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

`bl_dispatch_ota()` forwards these commands to the implemented OTA state
machine. They execute only in a partitioned build with an FMC backend;
the default full-flash/non-partitioned image, a build without FMC support,
and the reserved `0xF7..0xFF` range return `STATUS_NOSUPPORT` without
touching flash. This is the same degradation path the protocol uses for
reserved-but-unimplemented opcodes (see alp-sdk
`docs/gd32-bridge-protocol.md` §6).

## See also

* [`docs/gd32-bridge-protocol.md` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/docs/gd32-bridge-protocol.md)
  §10 -- protocol-level reservation of the OTA opcode range.
* [`docs/gd32-bridge.md` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/docs/gd32-bridge.md)
  "Flashing" -- end-user-visible upgrade paths.
* [`chips/gd32_swd/` (alp-sdk)](https://github.com/alplabai/alp-sdk/tree/main/chips/gd32_swd/) --
  host-driven SWD recovery path (Path B in the protocol spec).
