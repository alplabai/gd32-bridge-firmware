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
   **TRIAL**, but only when that slot's own recorded firmware version is
   **downgrade-safe**: `>= 0.2.14` (`OTA_TRIAL_MIN_FW_VERSION`, `src/ota.c`),
   the release this dance first shipped in. An older or version-unknown
   image (a legacy 8-byte `OTA_BEGIN`) has no `ota_boot_init()` /
   `ota_confirm_tick()` of its own to ever clear the flag, so marking it
   TRIAL would gate the wire BUSY forever; such an image commits/rolls back
   **CONFIRMED** (`flags = 0`) instead -- no watchdog protection, same as
   before this fix, but no permanent brick either.
2. This bootloader reads `RCU_RSTSCK` **once**, up front, to see whether the
   *previous* boot ended in a watchdog reset (`FWDGTRSTF`), stashes the raw
   value, and clears `RSTFC` (see "reset-cause ownership" below). For each
   candidate record (newest-first, #754's existing fallback order),
   `ota_boot_candidate_ok()` (`src/ota_layout.h`) rejects a TRIAL candidate
   if a watchdog already fired this power cycle -- it hung before
   confirming once. **Last-resort pass:** a bootloader must never idle
   while some CRC-valid, vector-valid image exists at all. If every
   candidate that passed `active_slot_valid()` was rejected ONLY by the
   watchdog gate (i.e. there is no other, already-confirmed fallback to
   fall back to), `ota_boot_select()` (`src/ota_layout.h`, a pure,
   host-tested function shared by this bootloader and its test suite)
   boots the newest such candidate anyway, re-arming the FWDGT for another
   attempt, rather than dropping into the recovery `WFI` loop with a
   perfectly bootable (if unconfirmed) image sitting unused.
3. Before jumping into a TRIAL candidate, the bootloader arms the FWDGT at
   its **maximum window**: reload `0xFFF` (12-bit counter) at the coarsest
   prescaler (`FWDGT_PSC_DIV256`) -- IRC32K/256 is *nominally* ~125 Hz, so
   this is *nominally* ~32.8 s, stated as an order-of-magnitude planning
   number, not an asserted tolerance: the GD32G5x3 datasheet's IRC32K
   accuracy section is the authority on the real spread and has not been
   independently re-checked against this repo for this fix. `fwdgt_config()`'s
   return is checked and retried exactly once on failure (a transient
   PSC/RLD-write timeout right after boot is the plausible failure mode;
   see `src/boot/boot_main.c`'s own comment) -- not looped, since this is
   the bootloader arming its own safety net. The debug-hold bit
   (`DBG_FWDGT_HOLD`) is set first: it **freezes** the FWDGT counter while a
   debugger holds the core halted, so a breakpointed bench session isn't
   blown away by a spurious watchdog reset mid-investigation (the counter
   resumes counting once execution continues -- this does not extend the
   real, running window).
4. If the jumped-to image comes up, `main()` calls `ota_boot_init()`
   (`src/ota.c`) *before* the transports start. It derives trial status from
   the record that actually **names the slot we are running**
   (`OTA_RUNNING_SLOT`, the same compile-time fact #3's fixes already
   derive from `BRIDGE_APP_SLOT_BASE`) -- deliberately NOT simply
   `meta_current()`'s overall-newest pick, because the last-resort pass
   above (and #754's ordinary fallback) can both boot a candidate that is
   *not* the highest-counter record on flash; if an even-newer-but-rejected
   record sits on the other page, `meta_current()` alone would describe a
   slot nothing is executing.
   - The record naming us is TRIAL: a fresh (or re-armed last-resort) trial
     boot. `ota_trial_unconfirmed()` gates the wire -- every opcode
     `protocol_dispatch()` sees, on either link, answers `STATUS_BUSY` with
     an empty payload (never `STATUS_OK` from a session that might still
     revert), and notes that a live frame arrived.
   - Separately, if the OVERALL newest record still names a *different*
     slot and is still TRIAL, that candidate was rejected (by validity or
     the watchdog gate) in favour of what we are actually running:
     self-heal it (repoint `active_slot` at us, clear TRIAL) with **no
     reset** -- we are already the safe (or last-resort) fallback -- and
     also **clear that slot's `slot_valid` bit**, so a later `ROLLBACK` can
     never re-select the exact slot the bootloader just steered away from.
5. Once a frame has been noted during an unconfirmed trial, the base-level
   tick (`ota_confirm_tick()`, called next to `ota_erase_tick()` from
   `bridge_hw_tick()`) commits the slot **CONFIRMED** (`flags = 0`) and
   reboots. This commit forces its erase target onto the **same metadata
   page the TRIAL record already occupies** (`meta_commit()`'s
   `target_override` parameter) rather than the usual #74 rank rule: a
   confirm is "flip this record's own flags", not "establish a new active
   slot", so it must not touch the *other* page -- which, in the scenario
   this fix exists for, holds the OLDER, already-CONFIRMED fallback
   record. A failed or power-cut-interrupted confirm then degrades to
   "reverted" (the older confirmed record survives, untouched, and the
   next boot uses it) rather than "only the TRIAL record is left". The
   extra reset on a successful confirm is deliberate: on a fresh, non-TRIAL
   boot this bootloader never arms the FWDGT, so rebooting is the only way
   to stop the watchdog that's been running since the trial boot. The
   confirmed-in-RAM flag is deliberately **not** set before this reset (a
   real `ota_system_reset()` never returns; keeping the gate closed through
   it means a host-test seam that DOES return can't observe a
   confirmed-but-not-yet-rebooted state that never exists on real
   silicon -- a test wanting to prove confirmation landed calls
   `ota_boot_init()` again, simulating the reboot, and checks that FRESH
   read of the now-flags=0 record).

**Reset-cause ownership:** this bootloader reads `RCU_RSTSCK` exactly ONCE
per boot, **stashes** the raw value in `RTC_BKP8` (an RTC backup-domain
register -- survives `NVIC_SystemReset()`, same rationale as
`hal/gd32/fault_handlers.c`'s use of `RTC_BKP0..7` for fault records;
`RTC_BKP8` is the next free one), then **clears** `RSTFC` before jumping --
unconditionally, on every single boot, not just the ones a trial confirm
triggers. The application's `CMD_RESET_REASON` handler
(`bridge_hw_reset_reason()`, `hal/gd32/init.c`) decodes from that stash, not
from a live `RCU_RSTSCK` read (which would only ever see "no cause" -- the
bootloader already cleared it by the time app code runs), reorders
`FWDGTRSTF` **before** `EPRSTF` in its priority ladder (the G5 can latch
`EPRSTF` alongside an internally-generated watchdog reset), and clears the
stash on read, preserving the original clear-on-read contract.
`ota_system_reset()` (`hal/fmc_ota.c`) no longer touches `RCU_RSTSCK` at all
-- the bootloader's unconditional per-boot stash+clear makes that
redundant. This closes two gaps the earlier (single-clear-point) design
had: a stale `FWDGTRSTF` from an unrelated EARLIER cycle could no longer be
distinguished from a live one if the app never happened to call
`ota_system_reset()` in between, and an OLD APP that never queries
`CMD_RESET_REASON` at all would carry a stale cause forward indefinitely
under the old design; the bootloader now owns the read-once/clear-always
step regardless of what the app does afterward.

**Cost on fielded units with the OLD (pre-fix) bootloader:** none of this
protects them -- an old bootloader doesn't know about `flags`,
`ota_boot_candidate_ok()`, the last-resort pass, or the reset-cause stash,
and will still jump unconditionally to the newest valid slot with no
watchdog. **An old bootloader paired with a NEW app**: the app still writes
`OTA_META_FLAG_TRIAL` on commit/rollback (subject to the version gate
above) and still gates the wire BUSY via `ota_boot_init()` -- but nothing
ever arms a watchdog to revert a hang, so the trial/confirm dance adds an
extra reboot with **no actual safety net** if the new image hangs before
confirming. **A new bootloader paired with an OLD (pre-fix) app**: the old
app never calls `ota_boot_init()`/`ota_confirm_tick()` and carries whatever
byte was in the metadata record's old `_pad[2]` forward as `flags` on its
own commits (this struct-v2 field existed as padding before this fix, so an
old app's `meta_commit()` never wrote it deliberately) -- in practice a
factory-provisioned or previously-confirmed record has that byte zeroed
already (`tools/gen_ota_metadata.py` always writes `flags = 0`), so the
common case is CONFIRMED-equivalent behaviour with no trial dance, but this
is NOT a guarantee for a part with an already-nonzero stray byte there;
**migrating a fielded part from an old app to a new one via OTA gets no
trial/confirm protection for that transition** -- the new bootloader's
watchdog only ever arms for a record the new app itself marked TRIAL. Only
a bootloader AND app rebuilt together with this fix get the full watchdog
fallback. On a fully-updated pair, the visible cost is one extra GD32
self-reboot per successful OTA (the confirm step's deliberate reset).

**Host contract:** after a successful `CMD_OTA_COMMIT` / `CMD_OTA_ROLLBACK`,
the host must send **some** valid frame (a `CMD_PING` is enough) within the
watchdog window (see item 3 above) so the confirm tick has a live frame to
notice -- without one, the image never confirms and the watchdog reverts it
even though it came up healthy. The host must also treat `STATUS_BUSY`
returned to ANY opcode in that window as **retryable**, not as an error:
it means "this session may still revert", and a later retry of the exact
same frame either succeeds (post-confirm) or fails the same way (still
gated) -- never a wire fault.

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
