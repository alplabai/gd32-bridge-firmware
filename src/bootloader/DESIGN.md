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
   **TRIAL** iff that slot's OWN FLASH BYTES carry a confirm-capable trial
   marker -- `ota_image_trial_capable()` (`src/ota_layout.h`) scans a
   bounded window of the target slot's own image for a 20-byte struct
   (magic + struct version + a capability bitfield, bit0 = implements the
   confirm handshake) that every app image built from this branch onward
   plants right after its vector table (`toolchain/gd32g553_app_slot.ld.in`'s
   `.trial_marker` section, `src/trial_marker.c`).
   **Bench fact 2026-09-26 follow-up:** this replaced an earlier cut of
   this guard that trusted the fw_version the HOST declared in
   `OTA_BEGIN` (`OTA_TRIAL_MIN_FW_VERSION`, now removed). That guard was
   defeated by the exact incident it existed to protect against: on
   2026-09-26 the host declared the BAD image's TRUE, pre-fix version at
   `OTA_BEGIN`, which the declared-version guard believed and would have
   committed WITHOUT trial protection -- the FWDGT would never have saved
   it. Eligibility has to come from the image itself, not from what the
   host claims about it. `s_fw_version` is still recorded into the
   metadata record (informational only; no longer decides trial
   eligibility or commit-ability).

   **Policy, stated plainly (CLOSED on PR #246): a markerless image --
   no marker at all, or a marker whose confirm-capability bit is clear --
   has no confirm path at all**, so forcing TRIAL onto it would make the
   FWDGT revert it (or, with no older CONFIRMED record to fall back to,
   reset-loop it every ~32.8 s) even when it is perfectly healthy, since
   an old app never calls `ota_note_frame()`/`ota_confirm_tick()` and
   nothing would ever clear a TRIAL flag forced onto it -- while
   committing it **CONFIRMED, unprotected** is exactly the 2026-09-26
   incident shape again. `CMD_OTA_COMMIT`'s handler (`h_commit()`,
   `src/ota.c`) therefore **REFUSES the request outright** for a
   markerless image: `STATUS_INVAL`, a dedicated `s_err` code, the active
   slot left completely untouched, and the next `OTA_BEGIN` unaffected --
   a valid, well-formed request policy declines, not a silently-chosen
   unsafe default. A pre-marker image remains installable only via
   SWD/factory programming.
   `CMD_OTA_ROLLBACK`'s handler (`h_rollback()`) is **unchanged** by this:
   rolling back to a markerless target still commits it **CONFIRMED**,
   because that slot's metadata record already proves it booted
   successfully on this unit before -- it is not the fresh, unproven case
   the refuse-at-COMMIT policy exists to guard against.
   `tools/check_trial_marker.py` is a build-time gate (wired into
   `CMakeLists.txt` for the two OTA-slot targets): it runs the identical
   bounded scan against the just-built `.bin`, so a slot image produced
   without a findable marker fails the build instead of shipping one.
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
   independently re-checked against this repo for this fix. `fwdgt_clock_ready()`
   brings up IRC32K (`rcu_osci_on(RCU_IRC32K)` + a bounded stabilisation
   wait, mirroring `hal/gd32/power.c`'s `rtc_wakeup_init_once()` for the
   identical oscillator) BEFORE arming (C2, adversarial-verify finding):
   without it, the very first `fwdgt_config()` call on a cold boot can
   race the oscillator and time out its PSC/RLD write. `fwdgt_config()`'s
   return is checked and retried exactly once on failure -- not looped,
   since this is the bootloader arming its own safety net. **A PSC/RLD
   timeout is not benign:** `fwdgt_config()` issues `FWDGT_CTL`'s
   `KEY_ENABLE` write FIRST, unconditionally, so the counter is running
   and CANNOT be un-armed by any register in this peripheral by the time a
   PSC/RLD failure is even detected; a timeout leaves the counter running
   under FWDGT's power-on-reset defaults (`PSC=/4`, `RLD=0xFFF`, roughly
   0.5 s at nominal IRC32K) instead of the intended ~32.8 s. The debug-hold bit
   (`DBG_FWDGT_HOLD`) is set first: it **freezes** the FWDGT counter while a
   debugger holds the core halted, so a breakpointed bench session isn't
   blown away by a spurious watchdog reset mid-investigation (the counter
   resumes counting once execution continues -- this does not extend the
   real, running window).
4. If the jumped-to image comes up, `main()` calls `ota_boot_init()`
   (`src/ota.c`) *before* the transports start. It derives trial status from
   `find_running_slot_record()` -- the CRC-valid record whose `active_slot`
   actually **names the slot we are running** (`OTA_RUNNING_SLOT`, the same
   compile-time fact #3's fixes already derive from `BRIDGE_APP_SLOT_BASE`)
   -- deliberately NOT simply `meta_current()`'s overall-newest pick,
   because the last-resort pass above (and #754's ordinary fallback) can
   both boot a candidate that is *not* the highest-counter record on flash;
   if an even-newer-but-rejected record sits on the other page,
   `meta_current()` alone would describe a slot nothing is executing.
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
     **Composition guard:** self-heal is deliberately SKIPPED whenever the
     record naming us is *itself* still an unconfirmed TRIAL (the
     composed "last-resort boot of a still-gated image" shape) -- writing
     `flags = 0` there would prematurely confirm a record
     `ota_confirm_tick()` has not actually confirmed, bypassing the
     frame-gated dance entirely. Deferring is safe: `ota_confirm_tick()`
     (via the same `find_running_slot_record()` lookup) targets and fixes
     up our own record once a frame arrives, and the other page's stale
     pointer gets cleaned up by this self-heal on a LATER boot once we are
     no longer gated.
   - Both this self-heal path and a successful confirm (item 5) call
     `ota_fault_loop_clear()`, which zeroes `hal/gd32/fault_handlers.c`'s
     consecutive-fault counter (`RTC_BKP7`). Without this, a TRIAL image
     that fault-loops (hits `FAULT_RESET_LOOP_LIMIT` before the FWDGT ever
     gets a chance to revert it) permanently consumes that counter --
     `fault_handlers.c`'s own header documents "nothing clears `RTC_BKP7`
     on a healthy boot" as an accepted, standing limitation -- so the
     confirmed fallback slot's very first UNRELATED fault would then halt
     instead of reset. Reaching either call site here IS the "healthy
     boot" signal that limitation was waiting on.
5. Once a frame has been noted during an unconfirmed trial, the base-level
   tick (`ota_confirm_tick()`, called next to `ota_erase_tick()` from
   `bridge_hw_tick()`) commits the slot **CONFIRMED** (`flags = 0`) and
   reboots. This commit forces its erase target onto the **page of the
   record that names `OTA_RUNNING_SLOT` with TRIAL** -- the SAME
   `find_running_slot_record()` lookup item 4 used, **not**
   `meta_current()` -- via `meta_commit()`'s `target_override` parameter,
   rather than the usual #74 rank rule. This distinction matters
   precisely in the #754-fallback shape: if the OVERALL newest record
   names a *different*, rejected slot, `meta_current()` would hand
   confirm the WRONG page to force as its erase target, corrupting the
   very record confirm is supposed to leave alone. A confirm is "flip
   THIS record's own flags", not "establish a new active slot", so it
   must not touch the *other* page -- which, in the scenario this fix
   exists for, holds the OLDER, already-CONFIRMED fallback record. A
   failed or power-cut-interrupted confirm then degrades to "reverted"
   (the older confirmed record survives, untouched, and the next boot
   uses it) rather than "only the TRIAL record is left". The extra reset
   on a successful confirm is deliberate: on a fresh, non-TRIAL boot this
   bootloader never arms the FWDGT, so rebooting is the only way to stop
   the watchdog that's been running since the trial boot. The
   confirmed-in-RAM flag is deliberately **not** set before this reset (a
   real `ota_system_reset()` never returns; keeping the gate closed through
   it means a host-test seam that DOES return can't observe a
   confirmed-but-not-yet-rebooted state that never exists on real
   silicon -- a test wanting to prove confirmation landed calls
   `ota_boot_init()` again, simulating the reboot, and checks that FRESH
   read of the now-flags=0 record).

   **A frame-confirmed image can still be reverted.** Noting a frame
   (`ota_note_frame()`) only starts the confirm commit on the NEXT
   base-level tick -- it does not stop the FWDGT. If power is lost, or the
   watchdog itself fires, inside the narrow erase-then-program window the
   confirm commit runs in, the image reverts exactly as if no frame had
   ever arrived: a healthy, wire-responsive image is not exempt from this
   window merely for having proven itself once.

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

**Fallback source, by configuration (C1, adversarial-verify finding):**
`RTC_BKP8` is written ONLY by THIS fix's bootloader, so
`bridge_hw_reset_reason()` treats a `RTC_BKP8 == 0` read as "no stash was
ever written" and falls back to a LIVE `RCU_RSTSCK` read (same priority
order, clearing `RSTFC` itself on that path) rather than reporting UNKNOWN
unconditionally:

| Configuration | `CMD_RESET_REASON` source |
|---|---|
| New bootloader + new app (the fully-updated pair) | `RTC_BKP8` stash |
| Full-flash, non-partitioned image (no bootloader runs at all) | live `RCU_RSTSCK` fallback |
| Old (pre-fix) bootloader + new app | live `RCU_RSTSCK` fallback (the old bootloader never stashed anything) |

The live-read fallback inherits the ORIGINAL single-reader limitation the
stash exists to remove (this app is the only thing that will ever read or
clear those bits), but that is strictly better than a permanent UNKNOWN on
two real, reachable configurations.

**Cost on fielded units with the OLD (pre-fix) bootloader:** none of this
protects them -- an old bootloader doesn't know about `flags`,
`ota_boot_candidate_ok()`, the last-resort pass, or the reset-cause stash,
and will still jump unconditionally to the newest valid slot with no
watchdog. **An old bootloader paired with a NEW app**: the app still writes
`OTA_META_FLAG_TRIAL` on commit/rollback (subject to the trial-marker guard
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
gated) -- never a wire fault. This retry contract also covers the rarer
case item 5 states plainly: a frame the host already sent can still be
reverted by a power loss or watchdog fault landing inside the confirm
commit's own erase-then-program window, so "I already got a non-BUSY
reply once" is not itself proof the image survived -- re-probe with
`OTA_GET_STATE` / `CMD_GET_BUILD_ID` after any suspected reset, the same
as the missing-reply case above.

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
