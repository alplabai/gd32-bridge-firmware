<!--
Copyright (c) 2026 Alp Lab AB
SPDX-License-Identifier: Apache-2.0
-->

# Bench validation runbook — the fifteen `needs-silicon` PRs

Every one of #73, #75, #80, #81, #82, #83, #84, #85, #86, #90, #91, #92, #99,
#106, #113 carries its own "Bench validation required" section. This
document collects them into one ordered plan. It does not repeat each PR's
full reasoning — read the PR for that — it gives the bench operator the
procedure, the pass/fail criterion, what a failure would falsify, and the
brick risk, in the order that keeps the board alive.

**This is a plan, not a certification.** Nothing below has been run. Where a
PR's own bench section was vague, that is called out in place rather than
invented into false precision. Where a figure could not be independently
confirmed from the datasheet/user-manual PDFs on hand, that is stated rather
than guessed.

## How to read an entry

Each numbered item gives:

- **Proves** — the one claim this step settles.
- **Procedure** — what to flash, drive, and capture.
- **PASS** / **FAIL** — the observable that decides it.
- **Falsifies** — which PR's claim dies if this comes out wrong.
- **Brick risk** — whether a wrong answer here can strand the part, and the
  recovery.

## 0. Pre-flight

**Equipment:**

- SWD probe (J-Link or equivalent) wired directly to the GD32G553's own SWD
  pins on the E1M-X V2N carrier — this is the *only* recovery path for
  anything in Phase 1/2 below. `src/boot/boot_main.c:20-22` states it
  plainly: this hardware revision has no host-driven (RZ/V2N-initiated) SWD
  reflash, so "recoverable" everywhere in this document means "recoverable
  with a probe physically on the board," never "recoverable over the wire."
- Programmable bench PSU with a current-sense point on the GD32's supply
  rail, and the ability to cut power on command (needed for #99's flash-ECC
  provocation and #91's current-draw comparison).
- Oscilloscope (2+ channels) for PWM/timer/GPIO edge capture, and a DMM for
  static voltage/current reads (VREFP, DAC pads, PC13 budget).
- A bus/logic analyser on the BRD_I2C and SPI lines, capable of capturing
  full frame payloads, not just framing/CRC (#92's corrupted-reply-content
  check specifically needs payload diffing).
- An RZ/V2N host (or host-side test harness) able to drive the wire protocol
  opcodes named below and to pace OTA chunks at an arbitrary offset/size —
  several steps need chunk offsets the current advertised `chunk_max`
  cannot produce cleanly (see the OTA phase and the `chunk_max` open
  question).
- Build environment for all four firmware targets (`gd32-bridge`,
  `gd32-bridge-slot-a`, `gd32-bridge-slot-b`, `gd32-bootloader`) at both the
  tree's current default optimisation (unpinned, effectively `-O0`) and at
  `-Os` (PR #106, not yet merged as of this writing) — Phase 1 requires
  booting at both.

**Recovery procedure (the one used repeatedly below):** attach the SWD
probe, halt the core, reflash the known-good image for the slot/target in
question via the probe's own loader (not the OTA path — the OTA path is
exactly what may be broken). This is always available; it is the "no
recovery" language throughout this document that means "not recoverable
*without* the probe."

## 1. Ordering rationale

The order below is not PR number order. Four things drove it, all stated in
the task and each traced back to a source in the tree:

1. **`src/boot/boot_main.c:20-22`** — a bug in boot, OTA, or FMC bricks the
   part with no host-driven recovery. Everything touching those three areas
   (#90, #113, #106, #92, #73) is therefore validated *before* anything
   that assumes a working, field-flashable image, and is validated *on its
   own* rather than bundled with unrelated changes, so a bad result has one
   suspect, not three.
2. **#90 and #113 change the boot mechanism itself** — #90 fixes the
   MSP-write-then-branch ordering in `jump_to_slot()`, #113 fixes the
   `.ramfunc` section identity that the FMC masked-window invariant depends
   on. Both must be individually proven before OTA (#92/#73) or the
   optimisation pin (#106) are trusted, because OTA and `-Os` both build on
   top of "the part boots the slot it's told to."
3. **#106 pins `-Os` globally.** Every generated instruction changes.
   Its own bench section asks for a boot of all four images specifically
   *because* a failure there invalidates the object-level evidence every
   later PR's disassembly-based claims (#90, #92, #113 all re-checked their
   invariants at `-Os` in review, but none of those re-checks ran on
   silicon) rest on.
4. **#81 is a hard blocker, not a validation.** Whether VREFP is actually
   board-tied to VDDA is unknown in both this repo and alp-sdk. Until a
   meter settles it, `#80` (ADC calibration) and any TMU/DAC reading that
   assumes a known reference are unfalsifiable — a "PASS" would mean
   nothing. It costs no flash and no boot, so it runs first, before
   anything else touches the board.

Two further pairings the task calls out explicitly are honoured below: #92
(FMC guard) and #73 (OTA erase targeting) are validated together as "the OTA
path," and issues #2 / #28 establish that the 2026-06-04 bench run proved
only the A→B direction and rollback — B→A is not "the same test in the other
direction," it is the **first time** that code path has ever executed on
silicon, because slot A straddles the dual-bank boundary `0x08040000` and a
B→A update is the direction that erases across it. That distinction is
carried into the OTA phase's procedure, not glossed over.

---

## Phase 0 — before anything is flashed: the VREFP hard blocker

### 0.1 — VREFP board-tie confirmation (#81 / issue #59)

**Proves:** whether VREFP is actually tied to VDDA (1.8 V) on this SoM
revision, or floating — the fact `hal/gd32/init.c`'s ADC/DAC millivolt
conversion has always assumed and never measured.

**Procedure:** with the *current, unmodified* firmware still on the part
(no flash required for this step — it can run before any other change in
this batch lands), meter the VREFP ball (J6) against VDDA with a DMM. Then
flash #81 and halt the part under SWD; read `VREF_CS` back and require it
reads `0x0000 0002` (POR value — `VREFEN=0, HIPM=1`, external-reference
mode per User Manual Rev1.2 p.520 Table 20-1, as cited in #81's own body).

**PASS:** VREFP tracks VDDA (reads ~1.8 V, moves with VDDA if VDDA is
perturbed) **and** `VREF_CS` reads back `0x0000 0002` post-fix.

**FAIL:** VREFP floats, reads an unexpected fixed voltage, or does not
track VDDA. This does not mean #81's *code* is wrong — leaving the internal
buffer off is still the only branch that's correct regardless of the
board-tie answer, per #81's own scope note — but it means `ADC_VREF_MV`
`/DAC_VREF_MV`'s 1800 mV assumption is **unknowable**, not merely unverified,
and every downstream millivolt figure (#80's calibration check, any DAC
sweep) has no ground truth to compare against until a schematic/board
fact resolves it.

**Falsifies:** #81's own "external reference mode is correct for this rail"
conclusion is falsified only if VREFP does *not* track VDDA — in which case
neither the old (internal buffer, out of spec) nor the new (external,
unconfirmed) code path produces a defined ADC/DAC reference, and the fix
becomes "replace a known-wrong reading with an undefined one," exactly the
risk one of the PR's own review comments states.

**Brick risk:** none. This is a passive meter read plus one SWD register
read-back; nothing here writes flash or touches a rail the GD32 controls.

**Vague-in-source note:** the PR does not say what voltage tolerance counts
as "tracks VDDA" — treat a reading within the DMM's own accuracy of VDDA as
a pass and anything else as inconclusive pending a schematic check, since
neither PR states a numeric band.

---

## Phase 1 — boot mechanics, on their own

Nothing in this phase should be combined with another change in the same
flash cycle. Each of the three needs to be individually attributable if the
part fails to come back up.

### 1.1 — `jump_to_slot()` MSP-then-branch ordering (#90 / issue #14)

**Proves:** that the single `volatile asm` block folding `msr MSP` / `isb` /
`bx` is what makes the slot jump safe, not codegen luck (the pre-fix code
only worked by two *different* accidents at `-O0` and `-Os`, per the
disassembly in the PR body).

**Procedure:**
1. Build `gd32-bridge-slot-a`, `gd32-bridge-slot-b`, `gd32-bootloader` with
   `-DBRIDGE_OTA_PARTITIONED=ON -DBRIDGE_HAL_BACKEND=gd32` at the tree's
   current default (unpinned/`-O0`).
2. Flash slot A via the bench SWD probe.
3. Power-cycle. Confirm the bootloader boots slot A: the bridge answers a
   protocol opcode over SPI/I2C (per `src/bootloader/DESIGN.md`'s
   2026-06-04 procedure — e.g. `CMD_PING`).
4. Repeat for slot B (`BRIDGE_APP_SLOT_BASE=0x08045000u`).
5. Repeat the full two-slot boot cycle against images built at `-Os`.

**PASS:** all four boots (slot A/B × `-O0`/`-Os`) answer a protocol opcode.

**FAIL:** silent hang — no `HardFault_Handler`, no watchdog on this path, so
"failure" looks identical to "board not powered." The only observable is
absence of a reply within a reasonable timeout.

**Falsifies:** #90's claim that the fix is optimiser-proof "by construction."
A failure at either optimisation level means the asm block does not in fact
guarantee the property the PR argues from disassembly alone.

**Brick risk:** yes, unconditionally — this is the exact function
`src/boot/boot_main.c:20-22` warns about. Recovery: bench SWD probe only.

### 1.2 — `.ramfunc` output-section split (#113 / issue #27 hygiene 1)

**Proves:** that giving `.ramfunc` its own linker output section (instead of
folding it into `.data`) is a pure section-identity fix — `erase_one_page`
and `program_one_dword` are still RAM-resident, `_sdata..._edata` still
covers the full copied span, and no `bl`/`blx` appears inside either
function at either optimisation level (the invariant #92's masked-window
fix depends on).

**Procedure:** build all four targets with #113 applied, at both `-O0` and
`-Os`. Boot slot A and slot B as in 1.1 (this is the same boot test, run
again on top of the section-layout change — do not skip it because 1.1
already passed on a different tree). Additionally, over SWD on a halted
part, read back `_sdata`/`_edata` and confirm the span matches the
pre-computed `0x20000148..0x20000564` (or the build's actual addresses if
they differ) and that `erase_one_page`/`program_one_dword` load in that
range, not in `.text`.

**PASS:** both slots boot at both optimisation levels, **and** the RAM
placement read-back matches the object-level evidence in the PR
(`arm-none-eabi-nm`/`objdump` output quoted in the PR body).

**FAIL:** either slot fails to boot, or the RAM-resident functions land
outside the expected span (would indicate the startup copy loop no longer
covers them — the PR's own "most important check").

**Falsifies:** #113's claim that this is "not a correctness bug — a
section-identity/hygiene defect." If the boot fails where it did not before
#113, the split broke something the object-level check missed.

**Brick risk:** yes, same class as 1.1 — this touches the RAM-resident FMC
path on the one image with no host-driven SWD recovery on this HW rev.
Recovery: bench SWD probe.

**Vague-in-source note:** #113 explicitly declines "hygiene 2" (the
`.heap_stack`/96K SRAM question) after determining the issue's own framing
did not hold up on inspection. Nothing to validate here for hygiene 2 —
it was not shipped.

### 1.3 — global `-Os` pin, all four images (#106 / issues #26, #104)

**Proves:** that every firmware image still boots and runs once `-Os` is
pinned unconditionally in `CMakeLists.txt`, not merely that the
disassembly of three *other* PRs' invariants looks right at `-Os` (which is
all #106 itself verified — object-level only).

**Procedure, run only after 1.1 and 1.2 both pass at `-Os`:**
1. Flash and boot `gd32-bridge` (monolithic, non-partitioned) on real
   silicon; confirm it answers protocol opcodes normally.
2. Flash `gd32-bootloader` + `gd32-bridge-slot-a`/`-b`; confirm the
   boot → validate → jump sequence still works end to end (same procedure
   as 1.1, now on the tree with `-Os` pinned by default rather than passed
   as a one-off build flag).
3. Run one full OTA cycle (this anticipates Phase 2 below, but #106's own
   bench section calls it out specifically as the path its FMC timeout
   budget — `OTA_FMC_ERASE_TIMEOUT_ITERS` sized off the `-Os` loop, ≈204 ms
   against the datasheet `tERASE` max of 20 ms — exists to protect). If
   Phase 2 has not yet run, defer this sub-step until it does; do not
   invent a shortcut OTA cycle here.

**PASS:** all boots succeed; the OTA cycle (once run per Phase 2) completes
inside the timeout budget on real hardware.

**FAIL:** any image fails to boot at `-Os` where it booted at the tree's
prior default, or the FMC timeout trips on a legitimate near-max-spec
erase/program (would mean the 10x margin computed from disassembled
instruction counts does not hold on real silicon — the derivation assumes
a cycle-per-instruction bound and an AHB wait-state bound that PR #92's own
review flags as unmeasured).

**Falsifies:** the entire batch's disassembly-based `-Os` re-verification
(#90, #92, #113 all re-checked their own invariants at `-Os` by
disassembly only). A boot failure here means at least one of those
re-checks missed something codegen does at `-Os` that it does not do at
the default.

**Brick risk:** yes, for the same reason as 1.1/1.2 — `-Os` changes the
generated instruction stream on `boot_main.c` and the RAM-resident FMC
routines. Recovery: bench SWD probe.

---

## Phase 2 — the OTA path (#92 + #73)

Validated together because they are the same erase/program funnel from two
sides: #92 guards *what range* the funnel will act on and bounds the
interrupt-masked window; #73 fixes *which slot* the erase targets. Both
inherit the boot-path brick warning, and both are gated on Phase 1 having
passed — an OTA cycle that lands on a part with an unproven boot mechanism
conflates two failure classes.

### 2.1 — address guard refuses the bootloader and the running slot (#92 finding #79)

**Proves:** `ota_fmc_range_forbidden()` actually refuses a bootloader- or
running-slot-overlapping erase/program on real hardware, not just in the
host unit test (`tests/unit/fmc_ota_guard/`).

**Procedure:** with the guard in place, attempt (via a debug/test hook, or
by constructing a deliberately-bad `OTA_BEGIN`/write sequence if the wire
protocol allows addressing outside the normal flow) an erase/program
targeting `0x08000000..0x08008000` (the bootloader) and a range overlapping
the currently-running slot. Confirm both are refused (`false` from the
funnel, surfacing as `STATUS_IO`/`OTA_ST_ERROR` on the wire per the PR's
own trace of each caller) and that flash content at those addresses is
unchanged after the attempt.

**PASS:** both refusals observed; a read-back (SWD or a subsequent boot) of
the bootloader and running-slot flash content is byte-identical to before
the attempt.

**FAIL:** either range gets erased/programmed. This is the address-guard
"fails open" scenario #92's own bench section names as brick-class.

**Falsifies:** #92's `ota_fmc_range_forbidden()` guard and #73's
compile-time `OTA_RUNNING_SLOT` derivation both, since #73's own body notes
the guard from #92 is what makes the interval-intersection check
reachable-in-principle at all.

**Brick risk:** yes — a guard that fails open erases the bootloader with no
recovery but a bench SWD probe. This is the highest-consequence single
check in this entire runbook.

### 2.2 — masked-window timing and SPI/I2C corruption, both directions (#92 findings #5, and the review-added corrupted-content check)

**Proves:** (a) the PRIMASK-masked busy window around `erase_one_page`/
`program_one_dword` bounds the interrupt blackout to roughly one page's
`tERASE`/`tPROG`, not the whole erase sweep; (b) a CS edge or I2C
transaction landing inside that window does not silently hand the host
**corrupted reply content** (not merely "no fault") — the review-round
correction to this PR's own bench section, because a mask spanning up to
~20 ms nominal covers roughly 62,500 SPI byte-periods at 25 MHz SCK.

**Procedure:**
1. Drive continuous SPI request/reply traffic from the RZ/V2N host across
   a full erase + program + commit cycle, **in both directions — A→B and
   B→A** (see 2.4 below for why B→A is not optional here). Diff every
   reply frame's *payload* against the expected echo/status, not just its
   CRC/framing, to catch a well-formed-but-stale reply.
2. Repeat with concurrent I2C traffic across the same window. I2C is
   better protected (`i2c_stretch_scl_low_enable()`, hardware SCL stretch,
   not ISR-dependent per the PR) but check specifically whether a ~20 ms
   stretch trips the RZ/V2N master's own I2C bus timeout — a second,
   separate failure mode from corrupted content.
3. With a logic analyser or DWT cycle counter, measure the maximum
   contiguous interrupt-masked interval during `ota_fmc_erase_range()`/
   `ota_fmc_program()` and compare it against the SPI RX DMA window
   `src/ota.c:81-83` documents (~8 ms blackout the host's reply re-read is
   designed to absorb).
4. Confirm a normal erase/program completes well inside the
   `OTA_FMC_ERASE_TIMEOUT_ITERS`/`OTA_FMC_PROGRAM_TIMEOUT_ITERS` budget
   (2,200,000 / sized similarly for program, per the PR's derivation from
   the `-Os` disassembly and the datasheet `tERASE`/`tPROG` max — this
   repo's own Datasheet Rev2.0 p.126 table gives `tERASE` max **20 ms**,
   `tPROG` max **80 μs**, both confirmed directly against the PDF for this
   runbook). If a fault can be induced safely, confirm the masked window
   actually exits on timeout rather than hanging.

**PASS:** every captured reply frame's payload matches the expected
value (no stale-data replies); no I2C bus-timeout trips; the measured
masked interval is on the order of one page's erase/program time, not the
full sweep; a healthy erase/program completes well inside the timeout
budget.

**FAIL:** any reply frame carries stale/corrupted payload content (a CRC-
or framing-only check would miss this, which is exactly why the review
round rewrote this step); an I2C transaction times out on the host side;
the masked interval exceeds a single page's worth; or a normal
erase/program approaches the timeout budget.

**Falsifies:** #92's own disclosed-but-unmeasured claims — the PR is
explicit that "the aggregate interrupt-masked interval across a full erase
sweep is not re-measured against the SPI RX DMA window" and that the
timeout derivation "assumes a Cortex-M33 cycle-per-instruction bound and an
AHB wait-state bound this repo has no bench trace to check." This step is
that trace.

**Brick risk:** the guard failing open (2.1) is the brick-class failure;
this step's own failure modes (corrupted reply, bus timeout) are
transport-level, not brick-class, but they run inside the same OTA cycle,
so use the same recovery posture — probe attached, ready to reflash — for
the whole session.

### 2.3 — compile-time erase-target derivation (#73 / issue #3)

**Proves:** `OTA_RUNNING_SLOT` (derived at compile time from
`BRIDGE_APP_SLOT_BASE`) makes `h_begin` target the *other* slot directly,
closing the divergent-metadata brick sequence #73 describes (begin → abort
mid-erase → rollback on a stale `slot_valid` bit → reset → next begin
targets the slot it's executing from).

**Procedure:** on a slot-A-resident build, issue `CMD_OTA_BEGIN` and
confirm the reply's `target_slot` names slot B (not a metadata read); read
`GET_STATE`'s `active` byte and confirm it reports the *running* slot.
Repeat on a slot-B-resident build, confirm the mirror image. Then attempt
to reproduce the original brick sequence from #73's own description
(begin → abort mid-erase → rollback → reset → begin again) and confirm the
second begin still targets the non-running slot rather than re-targeting
the slot it's executing from.

**PASS:** `target_slot`/`active` are correct in both directions, and the
reproduced divergence sequence no longer erases the running slot.

**FAIL:** either slot build reports the wrong target, or the divergence
sequence still lands on the running slot.

**Falsifies:** #73's central claim — that deriving the erase target from
`BRIDGE_APP_SLOT_BASE` rather than reading metadata removes the brick
sequence "with zero new flash writes."

**Brick risk:** yes, directly — this is the defect #73 exists to close.
Recovery: bench SWD probe.

**Vague-in-source note:** #73's own body flags that convergence after
`COMMIT` "is verified by reading, not by a test" and that a power cut
during the one narrow window the fix newly makes reachable (an alternation
to the non-booting metadata record) is a real, disclosed, but
un-benched trade. If time allows, add a power-cut-during-commit capture to
this step; it is not in #73's own required list, so its absence here is
not a gap this runbook is silently accepting — it is an acknowledged open
edge the PR itself names.

### 2.4 — A→B vs. B→A: why the second direction is not optional

Issues #2 and #28 establish, from the code and the 2026-06-04 bench record,
that only A→B (plus rollback) has ever run on real silicon. Slot A
(`0x0800A000..0x08045000`) straddles the dual-bank boundary
`0x08040000` — its tail 20 pages live in bank 1, the same bank slot B lives
in entirely. A B→A update erases slot A, including that bank-1 tail, while
the *running* image (slot B) executes from bank 1. That is precisely the
same-bank read-while-write condition `hal/fmc_ota.c:9-13`'s own comment
says faults, and it is not exercised by an A→B run at all — A→B's erase
target (slot B, wholly in bank 1) does not overlap the bank the *running*
slot-A image occupies (bank 0, plus the same 20-page tail, but the running
image there is what's being erased *into*, not fetched from, during an
A→B cycle... the exposure is asymmetric by construction).

**Run 2.2's full procedure in the B→A direction as a first-class test, not
a "for completeness" repeat of A→B.** It is the first time this code path
has ever executed on this hardware. If it fails, the failure is not a
regression — nothing regressed, because nothing ran there before.

**Brick risk:** the highest single risk in this document. Recovery: bench
SWD probe, and be ready to reflash both slots plus the metadata records if
the erase corrupts the running image mid-cycle.

---

## Phase 3 — I2C transport (#83)

### 3.1 — dispatch-once, TDATA flush, error recovery, Fast-mode timing, deep-sleep guard

**Proves:** five defects together — a command no longer dispatches 2-3
times per transaction (#4), a read no longer leaves a stray `0xFF` in
TDATA (#10), a bus error no longer level-holds the NVIC line (#7),
`I2C_TIMING` meets the Fast-mode `SCLDELY`/`SDADELY` minimums and is
re-derived (not hardcoded) across the Deep-sleep wake clock change (#38 +
the wake-path interaction fixed in review), and I2C0 is disabled with
`WUEN=0` across Deep-sleep entry (#41).

**Procedure (per the PR's own bench section):**
1. Capture a write-then-repeated-START-read `CMD_PING` (or
   `CMD_PWM_SINGLE_PULSE`) transaction on a bus analyser. Confirm exactly
   one dispatch (one pulse on the pad for the PWM case). Confirm a
   *second*, separate read afterward returns `STATUS_NO_PENDING` (`0x80`),
   not a replayed reply or idle `0xFF`. Run a sustained PING loop and
   confirm the host-side CRC never fails from the second read onward.
2. Inject a bus error on BRD_I2C while the bridge is addressed (a second
   bit-banged master asserting START mid-data-phase). Confirm the error
   ISR fires a bounded number of times, the next transaction completes
   normally, and a retried read after an error mid-reply gets
   `STATUS_NO_PENDING` rather than a stale byte.
3. Halt the part after `bridge_transport_i2c_hw_init()` and read back
   `I2C_TIMING` at normal run clock (216 MHz): require `PSC[3:0]`=`0xF`,
   `SCLDELY[3:0]`=`0x5`, `SDADELY[3:0]`=`0x4`. Scope SCL/SDA at the RZ/V2N
   end of BRD_I2C under worst-case production loading and confirm
   Fast-mode setup/hold margins.
4. Halt the part in Deep-sleep and read `I2C_CTL0` bit 0 (`I2CEN`) — must
   be 0. Wake, halt again, read `I2C_TIMING` — with no PLL relock in this
   firmware, `CK_APB1` should still read ~8 MHz (IRC8M) and `I2C_TIMING`
   should show the re-derived values for that clock (`PSC[3:0]`=`0x0`,
   `SCLDELY[3:0]`=`0x3`, `SDADELY[3:0]`=`0x0`), not the 216 MHz values.
   Confirm a BRD_I2C transaction issued after wake completes correctly at
   address `0x70`.

**PASS/FAIL:** as stated per sub-step above — this is one of the more
completely specified bench sections in the batch.

**Falsifies:** per sub-step — (1) falsifies the #4+#10 once-only-dispatch
fix; (2) falsifies #7's error-ISR recovery; (3) falsifies #38's timing
derivation; (4) falsifies #41's Deep-sleep guard *and* the review-round
wake-path re-derivation (#38/#41's interaction) — a wrong reading here
means the two fixes are still fighting each other, just less obviously.

**Brick risk:** none of this touches flash or boot. A wedged I2C transport
recovers with a bus reset / power cycle, not a probe-only recovery.

**Left out, flagged not resolved:** issue #39 (errata 2.3.1 SDA-stuck-low
detector) is explicitly not in this PR — it needs a periodic tick this
firmware does not have anywhere (no `SysTick_Config` in the tree). Nothing
to bench here for #39; it is out of scope for this batch entirely.

---

## Phase 4 — GPIO / SE-RST (#84)

### 4.1 — SE_RST glitch on boot, GPIO read-back source, PC13-15 power-switch budget (#61, #62, #60)

**Proves:** the OCTL-preload-before-CTL-flip ordering removes the
sub-instruction LOW glitch on promoted output pads (including PC13/SE_RST,
which resets the OPTIGA Trust M on every glitch); `CMD_GPIO_READ` now
reads `GPIOx_ISTAT` (measured pad level) instead of `GPIOx_OCTL` (an echo
of the last write), so contention is now observable; and the shared
3 mA / 2 MHz / 30 pF PC13-15 power-switch budget is documented, not
changed (it cannot be changed by register write — `GPIO_OSPEED_12MHZ` is
already the slowest class the part offers).

**Procedure (per the PR's own bench section, both review rounds):**
1. **SE_RST, full window.** Scope PC13 across a supervisor reset with the
   OPTIGA Trust M populated, **triggering and capturing from power-on
   (VDD rising / NRST release), not from `se_reset_init()` or `main()`**.
   The pre-`main()` `SramInit` loop alone is on the order of 20-25 ms
   during which PC13 sits ANALOG/floating by construction — this window is
   **not closed by this PR** (confirmed unfixable in firmware; the fix, if
   one is wanted, is an external pull resistor, per the PR's own review
   round 2). What the capture should confirm in addition to the
   pre-`main()` interval: from the first instruction of `bridge_hw_init()`
   onward, PC13 goes straight from ANALOG to released-HIGH output with no
   LOW excursion. Also halt over SWD immediately after
   `se_reset_configure()` returns and confirm `GPIOC_OCTL` bit 13 already
   reads 1 before `GPIOC_CTL` field `CTL13[1:0]` reads `01`.
2. **E1M IO pads.** Scope one E1M IO pad (e.g. IO8) across the host's
   first `CMD_GPIO_WRITE` commanding it HIGH; confirm the pad rises
   monotonically from the pull-up level with no LOW dip.
3. **GPIO read reports the pad, not the write.** Command an E1M IO pad
   HIGH via `CMD_GPIO_WRITE`, then pull it toward ground through a
   resistor sized to cross the input threshold against the push-pull
   driver (largest resistor that still crosses it; keep the duration
   bounded). Confirm `CMD_GPIO_READ` now reports LOW where it previously
   reported HIGH.
4. **PC13-15 shared budget.** With SE_RST held HIGH on PC13, load E1M IO24
   and IO25 to the carrier's worst case and measure the PC13 output level
   and the OPTIGA Trust M reset input while all three pads are driven —
   this establishes how much of the shared **3 mA** a carrier may take
   before SE_RST loses margin (the datasheet gives no VOH figure for these
   specific pads under Power-Switch supply). Separately, scope E1M IO24/
   IO25 under the fastest toggle sequence the host can issue over the
   25 MHz SPI link and confirm the edge rate stays within **2 MHz**, net
   capacitance measured against **30 pF**. The exact text — "PC13 to PC15
   are supplied through the Power Switch, which can only be obtained by a
   small current (typical source capability: 3 mA shared between these
   IOs...), the speed of GPIOs PC13 to PC15 should not exceed 2 MHz when
   they are in output mode (maximum load: 30 pF)" — was opened and
   confirmed for this runbook directly against **GD32G553xx Datasheet
   Rev2.0, p.130, footnote (2) attached to Table 4-29** (the footnote sits
   on the page immediately before the Table 4-30 heading; PDF text
   extraction confirms the wording verbatim).

**PASS/FAIL:** per sub-step, as described above.

**Falsifies:** (1) falsifies #61's PC13 fix for the *post*-`bridge_hw_init()`
window specifically — the pre-`main()` window is already known-open and
not what this step tests; (3) falsifies #62's read-source fix; (4) is a
measurement, not a pass/fail against this PR's code (the code cannot
change the 3 mA / 2 MHz / 30 pF limit) — it establishes carrier-level
margin, which is what #60 asks for.

**Brick risk:** none of #84 touches flash or boot. A wedged SE-RST state
recovers with a power cycle; it does not need a probe.

**Vague-in-source note:** the PR itself flags the pre-`main()` PC13 window
as open and unfixable from this firmware — the bench step above is written
to confirm that window's *existence and duration*, not to attempt to close
it. Do not read a positive capture here as "issue #61 is closed"; only the
post-`bridge_hw_init()` half is.

---

## Phase 5 — timers and PWM (#86 + #82)

Grouped because both PRs touch the same TIMER0/TIMER7/TIMER19 hardware and
because review on both surfaced a shared, unresolved cross-PR hazard
(issue #89: forcing a timer-wide update event for one purpose can glitch a
sync-slave relationship set up by the other). Run #86 first — it is the
prerequisite for a meaningful multi-timer capture in #82's sibling-isolation
step.

### 5.1 — internal-trigger routing, all six ordered pairs (#86 / issue #42)

**Proves:** the per-(master, slave) `SYSCFG_TIMERxCFG2` lookup table
replaces the old hardwired-to-ITI0 routing, which was wrong for four of the
six reachable (master, slave) pairs.

**Procedure:** for each of the six pairs, send `CMD_TIMER_SYNC` (`0x27`)
in reset mode (mode=1), then scope both timers' PWM outputs and confirm
the slave's period locks to and resets in phase with the master's, at the
master's rate — not free-running, not locked to a different timer. A
cheaper first check before the full scope capture: read back
`SYSCFG_TIMERxCFG2` `TSCFG15[20:16]` on the slave after each call and
confirm it matches the table in the PR body.

| Master | Slave | What proves it |
|---|---|---|
| TIMER7 | TIMER0 | Locks to TIMER7's period (was: free-running or dead) |
| TIMER19 | TIMER0 | Locks to TIMER19's period (was: free-running or dead) |
| TIMER0 | TIMER7 | Still locks to TIMER0 (regression check — was already correct) |
| TIMER19 | TIMER7 | Locks to TIMER19, not TIMER0 (was: wrong master) |
| TIMER0 | TIMER19 | Still locks to TIMER0 (regression check) |
| TIMER7 | TIMER19 | Locks to TIMER7, not TIMER0 (was: wrong master) |

**PASS:** all six lock as tabled. **FAIL:** any pair free-runs, locks to
the wrong master, or reads back the wrong `TSCFG15` value.

**Falsifies:** #86's lookup-table fix, per pair.

**Brick risk:** none — a wrong timer route is a functional bug, not a
flash/boot hazard.

### 5.2 — PWM truncation reject, shadow-safe writes, halted-timer recovery (#82 / issues #16, #43, #8)

**Proves:** an unrepresentable compare is rejected (`STATUS_OUT_OF_RANGE`)
rather than silently truncated to 0; an already-running channel's duty
change lands glitch-free at the next natural update event with phase
*preserved* (the corrected behaviour — the pre-review version of this PR
forced a phase restart on every write, which was itself a regression, now
fixed); a `PWM_SET` recovers a timer halted by a prior `PWM_SINGLE_PULSE`.

**Procedure (per the PR's own, review-revised bench section):**
1. **#16 boundary, single-pulse.** `CMD_PWM_SINGLE_PULSE` (`0x26`) with
   `pulse_ns = 65536000` → expect `STATUS_OUT_OF_RANGE`, no pulse. Then
   `pulse_ns = 65535000` → expect a single ~65.535 ms pulse. Repeat on
   both TIMER0 (PWM0-3) and TIMER7 (PWM4-7) groups.
2. **#16 boundary, continuous.** `CMD_PWM_SET` (`0x20`) with
   `period_ns = 65536000, duty_ns = 65536000` → expect
   `STATUS_OUT_OF_RANGE`, pad unchanged; then `duty_ns = 65535000` →
   ~100% duty at the pad.
3. **#43, already-running channel.** With a PWM channel already running a
   steady period, scope the pad and issue repeated `CMD_PWM_SET` duty
   changes as fast as the link allows. Confirm every observed period is
   either the old or the new duty, no runt width, **and confirm phase is
   preserved** (no period restart) — this is the corrected behaviour; a
   visible phase restart here is a regression against the pre-review fix.
4. **#43, sibling isolation.** With PWM0 and PWM1 (both TIMER0) running
   steady, distinct periods, issue repeated `CMD_PWM_SET` duty changes on
   PWM0 only. Confirm PWM1's period and phase are undisturbed.
5. **#8.** `CMD_PWM_SINGLE_PULSE` on PWM0 (PA11, TIMER0_MCH0), confirm the
   one-shot, then `CMD_PWM_SET` on a sibling on the same timer (e.g. PWM3,
   PC5, TIMER0_MCH3) — confirm a continuous waveform reappears (this
   recovery path is still expected to restart phase on the whole timer
   group, since the halted timer has no natural update event otherwise).
   Repeat on the TIMER7 group.
6. **`pwm_capture.c` coalescing (the case the pre-review PR could not have
   caught).** With a capture channel bound to a pad fed by a sibling PWM
   channel on the same timer running continuously: issue `CMD_PWM_SET`
   with period A, wait past the old period so A is confirmed physically
   running; within less than one period of A, issue a second
   `CMD_PWM_SET` with period B **without** polling
   `CMD_PWM_CAPTURE_READ` in between; only then poll capture-read and
   confirm it reports **A**, not B (B is not yet promoted); let a further
   period elapse without another write and re-read — confirm it now
   reports B.

**PASS/FAIL:** per sub-step as stated.

**Falsifies:** (1)/(2) falsify the reject-not-clamp decision; (3)/(4)
falsify the shadow-register (ARSE/CHxCOMSEN) fix and its phase-preservation
property specifically; (5) falsifies the `CEN` re-enable fix; (6) falsifies
the `pwm_car_shadow_defer`/`_commit` mirror design that replaced the
raw-`CAR`-re-read the PR's own second review round found wrong.

**Brick risk:** none — PWM misbehaviour is a functional/output-safety
concern (see Supervised outputs below), not a flash/boot hazard.

**Cross-PR hazard, not fixed by either #82 or #86 (issue #89):** forcing a
timer-wide update event (`UPG`) on a sync-master timer also fires `TRGO0`,
which can glitch a slave timer synced off it via #86's routing. #86's own
review concluded no `TIMER_TRI_OUT0_SRC_*` source is immune to a forced
`UPG` without sacrificing a PWM channel — the likelier fix lives in #82
(make the forced update conditional on the timer not currently being a
sync master), not implemented in either PR in this batch. **Nothing in
this runbook validates #89** — flag it as a known gap if #86 and #82 are
both armed on the same board with `CMD_TIMER_SYNC` also in use.

---

## Phase 6 — quadrature encoders (#75 / issues #33, #15)

**Proves:** `RCU_SYSCFG` is enabled ahead of decoder-mode configuration
(without it, `SYSCFG_TIMERxCFG0.TSCFG2[14:10]` writes never stick, and
every encoder free-runs on its APB clock while still reporting
`STATUS_OK`), and TIMER2/TIMER3 (16-bit, encoders 1 and 2) are correctly
sign-extended rather than reporting `+65535` for a one-step-backwards
count.

**Procedure (per the PR's own bench section):**
1. On a halted part, after `bridge_hw_init()` returns, read
   `SYSCFG_TIMER1CFG0` (SYSCFG base + `0x10C`) and require
   `TSCFG2[14:10] != 00000` (expect `0x0000 7C00` per the SPL's all-zero
   trigger branch). Repeat at `0x118` (TIMER2), `0x124` (TIMER3), `0x130`
   (TIMER4).
2. End-to-end: turn one encoder a known number of detents; confirm
   `CMD_QENC_READ` (`0x60`) reports the expected X4 edge count and holds
   steady while the shaft is not moving.
3. For encoder 1 (TIMER2, X=PC6, Y=PC7) and encoder 2 (TIMER3, X=PB6,
   Y=PB7): issue `CMD_QENC_RESET` (`0x61`), step one X4 count backwards,
   issue `CMD_QENC_READ` (`0x60`), require **-1**; step forward two
   counts, require **+1**. Repeat on encoder 0 (TIMER1) and encoder 3
   (TIMER4) to confirm the `wide` flag did not regress the 32-bit timers.
   Drive a full backwards wrap on encoder 1 past `0x8000` and confirm the
   reported position decreases monotonically with no 65536-count step.

**PASS/FAIL:** as stated per sub-step.

**Falsifies:** (1)/(2) falsify #33's SYSCFG-clock-gating fix; (3) falsifies
#15's sign-extension fix, specifically the wrap-boundary case.

**Brick risk:** none — this is purely a timer-peripheral bring-up
correctness question.

---

## Phase 7 — ADC (#80 / issues #44, #34, #17), after Phase 0

Run only after Phase 0 (#81) has produced a VREFP answer — #80's own
calibration bench step ("compare reported millivolts before/after this
change") is meaningless without a known reference to compare against.

### 7.1 — ROVF clear/recover, calibration after every `ADCON` toggle, EOC bound

**Proves:** a DMA-starved stream no longer answers `STATUS_OK` with
`got_samples == 0` forever (ROVF was never cleared/checked); every
`CMD_ADC_READ`/`STREAM_BEGIN` recalibrates after the `ADCON` toggle that
destroys the calibration factor (previously every reading since boot was
uncalibrated); and the EOC busy-wait's abort bound is a fixed iteration
ceiling rather than one that scaled multiplicatively with the oversample
ratio.

**Procedure (per the PR's own bench section):**
1. **#44.** Start an `adc_stream` at a rate the DMA cannot sustain (or
   otherwise starve the DMA mid-stream) and confirm `ADC_STAT` bit 5 sets.
   Poll `CMD_ADC_STREAM_READ` and confirm the host observes `STATUS_BUSY`
   (not permanent `STATUS_OK`/zero-samples) and that samples resume on the
   following poll, with `DMA_CHxCNT` advancing again.
2. **#34.** Drive a stable known voltage into one ADC channel (now that
   Phase 0 has established what "known" means against VREFP), issue
   repeated `CMD_ADC_READ`, and compare reported millivolts before/after
   this change — pre-fix readings should drift from the reference by the
   converter's uncorrected offset/gain error from the second request
   onward; post-fix they should not. Repeat for `CMD_ADC_STREAM_BEGIN` and
   for the sibling channel sharing the same converter.
3. **#17.** With `oversample_ratio = 256`, provoke the wedged-converter
   state from the 2026-06-04 incident (an `adc_stream` cycle followed by a
   single-shot read) and measure CS-EXTI handler dwell with a scope on the
   CS pin or a DWT cycle counter — must fall to tens of milliseconds, and
   the link must survive. Separately confirm a healthy 256x-oversampled
   `CMD_ADC_READ` still returns `STATUS_OK` without hitting the timeout
   branch.

**PASS/FAIL:** as stated per sub-step. Note the PR's own explicit caveat on
(3): the millisecond figures it quotes (~4.6 ms healthy dwell, ~18.4 ms
wedged-cap estimate) are **estimates pending this measurement, not settled
facts** — treat the bench numbers as authoritative once available, not the
PR's arithmetic.

**Falsifies:** (1) falsifies the ROVF clear+recovery fix; (2) falsifies the
calibration-after-toggle fix — and, if Phase 0 found VREFP undetermined,
(2) cannot be run meaningfully at all (see the Phase 0 dependency above);
(3) falsifies the new additive-plus-ceiling EOC bound and, separately,
settles whether the wall-clock estimate in the PR (explicitly flagged as
unverified per-iteration cost) is close or far off.

**Brick risk:** none — ADC misbehaviour is a data-correctness/liveness
concern, not a flash/boot hazard.

---

## Phase 8 — TMU Q31 band representability (#85 / issue #46)

**Proves:** Q31 `SQRT` now always answers (previously refused wholesale by
an earlier, over-corrected draft of this PR); Q31 `SINH`/`LN` answer within
their representable input bands and reject with `STATUS_OUT_OF_RANGE`
outside them; Q31 `COSH` remains correctly refused (`cosh(x) >= 1` for
every `x`, so no Q31 input is ever representable).

**Procedure (per the PR's own bench section — all inputs/outputs are exact
Q31 hex values computed independently in the PR, reproduced here
verbatim):**

| # | Opcode / mode | format | in_a | Expect |
|---|---|---|---|---|
| 1 | `CMD_TMU_COMPUTE`, SQRT(5) | Q31(0) | `0x20000000` (0.25) | `STATUS_OK`, reply ≈ `0x40000000` (0.5) |
| 2 | `CMD_TMU_COMPUTE`, SQRT(5) | Q31(0) | `0x73333333` (0.9) | `STATUS_OK`, reply ≈ `0x796E744E` |
| 3 | `CMD_TMU_COMPUTE`, SINH(8) | Q31(0) | `0x40000000` (0.5) | `STATUS_OK`, reply ≈ `0x42B34040` |
| 4 | `CMD_TMU_COMPUTE`, SINH(8) | Q31(0) | `0x70D0D986` (asinh(1) boundary) | `STATUS_OUT_OF_RANGE` (`0x08`) |
| 5 | `CMD_TMU_COMPUTE`, LOG(6) | Q31(0) | `0x40000000` (0.5) | `STATUS_OK`, reply ≈ `0xA746F404` |
| 6 | `CMD_TMU_COMPUTE`, LOG(6) | Q31(0) | `0x1999999A` (0.2) | `STATUS_OUT_OF_RANGE` |
| 7 | `CMD_TMU_COMPUTE`, COSH(9) | Q31(0) | any | `STATUS_NOSUPPORT` (`0x06`), unchanged |
| 8 | `CMD_TMU_COMPUTE`, SQRT(5) | F32(1) | `4.0f` (`0x40800000`) | `STATUS_OK`, reply `2.0f` — regression check, confirms F32 untouched |

**PASS:** all eight match. **FAIL:** any mismatch — in particular, cases 4
and 6 returning `STATUS_OK` with a wrong number would mean the
representability band is wider on real hardware than the manual's tables
say (this fix has zero dependency on GD32 register semantics for the
band arithmetic itself — `hal/gd32/tmu_q31_scale.c` is host-tested — so a
bench mismatch here would point at the actual TMU hardware register
writes in `hal/gd32/tmu.c`, not the band logic).

**Falsifies:** the band-representability tables (UM Rev1.2 Tables 14-16,
14-18, 14-23, 14-26, as reproduced and cross-checked against the vendor
SPL in the PR body) if any Q31-in-band case fails, or the refusal logic if
an out-of-band case returns a plausible-looking wrong answer instead of
`STATUS_OUT_OF_RANGE`.

**Brick risk:** none.

---

## Phase 9 — main-loop `wfi` (#91 / issues #13, #54)

**Proves:** the main loop's `__WFI()` now emits a real `wfi` instruction
(`bf30`) rather than an empty stub — the pre-fix `#ifndef __WFI` guard
never distinguished anything, since CMSIS's `__WFI()` is a macro this file
never included, so both build configs in this tree got the empty stub,
including the real hardware build, and the core spun at full clock
forever.

**Procedure (per the PR's own bench section):**
1. Probe the GD32's supply rail (or the on-EVK current-sense point) and
   confirm a visible average-current drop between an idle bridge (no
   transport traffic, spinning in `for (;;) { __WFI(); bridge_hw_tick(); }`)
   before vs. after this change. A clean disassembly (already done,
   object-level, in the PR) proves the instruction is emitted; it does not
   prove the part draws less current — that is what this step is for.
2. Confirm the bridge still answers `PING`/transport transactions normally
   after idling in real WFI for an extended period (rules out a wake-
   latency regression from actually sleeping vs. spinning).

**PASS:** measurable average-current drop; normal transport response after
extended idle.

**FAIL:** no current drop (would mean something elsewhere keeps the core
awake despite the `wfi` opcode being present — e.g. a pending interrupt
immediately re-waking it every cycle) or a wake-latency regression.

**Falsifies:** the entire premise that this fix changes power behaviour,
not just the disassembly.

**Brick risk:** none.

**Watchdog / periodic tick (#54): explicitly not implemented, and not
benched here.** The PR's own assessment declines to implement either a
watchdog or a periodic tick in this change, for two independently-stated
reasons: sizing a safe FWDGT timeout needs measurements this batch does
not yet have (the OTA erase pump's per-page bound, the ADC calibration/EOC
spins from Phase 7), and a live SysTick's interaction with Deep-sleep entry
(`hal/gd32/power.c`, owned by PR #83 in this batch) is not resolved by the
manual text alone and was explicitly left for a follow-up that sequences
with #83. **Nothing to bench for #54 in this runbook** — it is out of
scope by the PR's own explicit statement, not an oversight here.

---

## Phase 10 — fault handlers (#99 / issue #36)

Run last among the "everything else" phases: several of its own steps
deliberately provoke a fault, which is a controlled, self-recovering
disruption (via the very reset/halt behaviour under test) but is still
disruptive to whatever else is running on the bus, and its flash-ECC
provocation step reuses the OTA path from Phase 2 — run it after Phase 2 so
a Phase-2 regression is not mistaken for a Phase-10 finding or vice versa.

**Proves:** all five NMI/fault vectors (NMI, HardFault, MemManage,
BusFault, UsageFault) now record diagnostic state to `RTC_BKP0..7` and
reset (or halt, if a debugger is attached) instead of hanging forever with
no diagnostic — previously all five were weakly aliased to the vendor's
`Default_Handler` (`b .`, an unconditional infinite loop).

**Procedure (per the PR's own bench section, one sub-step per source):**
1. **Flash ECC (NMI, `FLASHECCIF`)** — the reachable path from issue #36:
   start an OTA update (Phase 2 must already be validated), cut power with
   the programmable PSU mid-program (interrupting one 72-bit doubleword),
   then boot and let the bootloader's CRC walk hit the damaged doubleword.
   **Caveat stated in the PR itself:** this scenario is fully exercisable
   only once `fault_handlers.c` is *also* linked into the bootloader — a
   follow-up not in this PR (`src/boot/boot_main.c` links a disjoint
   source list that does not pull in `hal/gd32/*.c`). Today, exercise it
   via a direct read of a deliberately-corrupted flash location from
   application code instead, and expect `RTC_BKP0` to read
   `FAULT_RECORD_MAGIC | FAULT_TYPE_NMI`, `RTC_BKP3` (`SYSCFG_STAT`) to
   have `FLASHECCIF` set, `RTC_BKP4` (`FMC_ECCCS`) to have `ECCDET0` set
   with `ECCADDR`/`BK_ECC` matching the damaged doubleword.
2. **HXTAL clock failure (NMI, `CKMNMIIF`)** — pull the HXTAL
   crystal/oscillator input while `CKNMIIE` is enabled. Expect `RTC_BKP3`
   bit 3 set, `RTC_BKP4` = 0.
3. **SRAM ECC (NMI, `SRAM*ECCMEIF`)** — **first confirm `FMC_OBCTL` bit 24
   `SRAM_ECCEN`** on the bench part; issue #36 established this bit's
   state is unconfirmed and nothing in the repo reads it. If armed, write
   a corrupted 2-bit pattern via the debugger into an untouched SRAM
   location and read it back to trip the multi-bit detector.
4. **HardFault (forced)** — jump to an unaligned/invalid PC, or trigger an
   escalated bus fault. Expect `RTC_BKP3` (`CFSR`) with the relevant
   `MMFSR`/`BFSR`/`UFSR` bit(s) and `HFSR.FORCED`, `RTC_BKP1`/`RTC_BKP2`
   (PC/LR) at the faulting call site.
5. **MemManage** — needs the MPU configured with a protected region, which
   this firmware does not do today; otherwise MemManage cannot fire and
   escalates straight to HardFault. **Note this in the bench log rather
   than forcing it artificially**, per the PR's own instruction.
6. **BusFault (precise)** — read/write an address outside any configured
   memory region (e.g. `0x60000000`). Expect `RTC_BKP3` `BFARVALID` set
   and the recorded `BFAR` equal to the faulting address.
7. **UsageFault** — execute an integer divide by zero (if `SCB->CCR`
   enables the trap). Expect `RTC_BKP3` `DIVBYZERO` set.
8. **Reset-loop bound** — repeat any one of the above 4+ times without an
   intervening manual clear of `RTC_BKP7`. Confirm the 4th occurrence
   halts (core stopped per `DCB->DHCSR`) instead of resetting again.
9. **Debugger-attached skip** — attach the probe *before* provoking any of
   the above; confirm the handler halts instead of resetting, with
   `CFSR`/`SYSCFG_STAT`/`FMC_ECCCS` still live/readable, not cleared.

**PASS/FAIL:** per sub-step, as stated.

**Falsifies:** each sub-step falsifies the corresponding fault source's
record-and-recover behaviour; (8) falsifies the consecutive-fault counter;
(9) falsifies the debugger-attached halt-instead-of-reset branch.

**Brick risk:** low but not zero. The PR's own "what would change my mind"
note: resetting can glitch whatever the GD32 is driving (rails/reset lines
it gates for the rest of the SoM) while it re-runs its own `Reset_Handler`
and `bridge_hw_init()`. If step 8's reset loop is triggered accidentally
during an unrelated bench session elsewhere in this runbook, it is
self-recovering (halts after 3 consecutive resets) but will leave
`RTC_BKP7` non-zero — clear it via debugger write before the next fault
test, or the counter reads as a false "already near the limit."
Recovery for a genuinely wedged fault handler: bench SWD probe (same as
everywhere else), though this is the one phase in the batch whose entire
purpose is to make that less necessary.

---

## Open questions the bench can answer (not PR validations)

These are not covered by any of the fifteen PRs' own bench sections — they
are questions the issues raise that only a bench measurement settles, and
that this batch has an opportunity to answer while the board is already up.

### The `chunk_max` question (issue #25)

`OTA_BEGIN`'s advertised `chunk_max` is 60 bytes
(`GD32_BRIDGE_MAX_PAYLOAD_BYTES - 5`), and 60 is not a multiple of 8 — the
flash program granule `ota_fmc_program()` requires. A host that took the
advertised value at face value and streamed fixed 60-byte chunks would
poison its OTA session on the second chunk (offset 60 is not 8-aligned).
The 2026-06-04 bench run (issue #28) proved an A→B cycle worked — which
means the host used on that date did **not** pace at the literal advertised
60. What is not established anywhere in this repo or in either issue is
**what pacing that host actually used**: an 8-aligned size chosen
independently of the (broken) advertised value, or something else. This
matters for triage, not just tidiness: if the 2026-06-04 host already paced
at an 8-aligned size, `chunk_max = 60` is a **latent** bug — real, but
never actually exercised by the one host that has run this path. If it
somehow paced at 60 and got lucky (e.g. by never hitting an offset that
mattered, or by a host-side retry/re-chunk that isn't visible in this
firmware's history), it is closer to a **live regression waiting to
happen** on the next host build. **The bench can settle this directly**:
check the 2026-06-04 host build's chunking logic (if the artifact or its
source is still available) for the literal chunk size it used, or — more
robust than archaeology — issue `CMD_OTA_BEGIN` from the current host,
inspect the offsets its `OTA_WRITE_CHUNK` calls actually use, and check
them against 8-byte alignment directly, independent of what `chunk_max`
currently advertises.

### #53's effect on #64: does the RTC wake reach the core at all?

Issue #64 (RTC wakeup-timer timing accuracy: IRC32K is only specified to
28-36 kHz, so a nominal 0.5 ms tick is actually 444-571 μs, ±11%/+14%) is
not benchable as written, because issue #53 establishes that the RTC
wakeup interrupt **never reaches the core in the first place** —
`RTC_CTL.WTIE` is never set, EXTI line 19 is never enabled, `RTC_WKUP_IRQn`
is never enabled at the NVIC, and `RTC_STAT.WTF` is never cleared. A host
that requests a timed Deep-sleep or Standby wake today gets `STATUS_OK` and
then permanently loses the bridge — both transports decode inside ISRs on
a core parked in `__WFI()` with no interrupt configured to end it. **#64's
own timing-accuracy question cannot be measured until #53 is fixed**: there
is currently no wake edge to time. Neither #53 nor #64 is among the fifteen
PRs in this batch, so this is flagged here as a dependency for whoever
picks either issue up next, not as a step in the fifteen above. If #53
lands first, #64's own verification section already specifies the
measurement: request a 1000 ms wake, measure the wake edge on a scope,
accept anywhere in 889-1143 ms (and up to 37.45 s at the 32.767 s ceiling)
— this runbook does not repeat that derivation, only flags that it is
gated on #53, not on anything in this batch.

---

## What this does not cover

Fifteen PRs' worth of bench validation is not full coverage. Specifically,
this runbook does **not** exercise:

- **Standby mode (power mode 3) at all**, beyond what #99's fault-loop
  bound touches incidentally. #61's own issue states Standby's pad state
  during that mode is "not proven" by the manual and needs a schematic
  check this batch does not have. #53/#64 (RTC wake) are Deep-sleep- and
  Standby-adjacent but are explicitly out of the fifteen PRs, per above.
- **Issue #39** (I2C errata 2.3.1, SDA-stuck-low detector) — needs
  infrastructure (`SysTick`) this firmware does not have anywhere; #83
  explicitly leaves it out.
- **Issue #54** (watchdog) — explicitly declined in #91's own scope, for
  reasons stated there (unmeasured safe-timeout inputs, an unresolved
  Deep-sleep/SysTick interaction).
- **Issue #89** (cross-PR TRGO0 hazard between #82 and #86) — named by
  both PRs' review rounds, fixed by neither. Not validated here.
- **Statistical/population testing.** Every step above is a single-unit
  bench proof of existence, not a distribution. The IRC32K, PC13-15 budget
  and TRNG figures in particular carry datasheet footnotes marking them
  "guaranteed by characterization" or "guaranteed by design, not 100%
  tested in production" — a single board's reading confirms behaviour on
  that board, not the specification's own margin.
- **Thermal corners.** Every electrical figure quoted above (IRC32K
  28-36 kHz, the PC13-15 budget, `tERASE`/`tPROG`) is a datasheet limit
  across a temperature range; nothing in this runbook proposes measuring
  at temperature, only at bench-ambient.
- **#47's FIPS discard/repeat TRNG checks and #36's SRAM-ECC arm state**
  are both flagged in their own PRs as needing a bench answer this
  runbook includes (Phase 10.3, and #81's own bench section for TRNG) but
  does not resolve in advance — they are provoked-fault tests whose result
  is genuinely unknown going in.
- **Anything outside these fifteen PRs' diffs.** USB, the BRD_I2C EEPROM/
  RTC path already characterized elsewhere, the DA9292 forwarding path,
  and any peripheral not named above are untouched by this batch and
  unvalidated by this document.
- **A combined, simultaneous-load soak.** Every phase above validates one
  cluster of PRs largely in isolation. None of this runs, say, PWM + I2C +
  OTA + fault injection concurrently for an extended period, which is
  closer to the field load than any single phase's isolated procedure.

## Supervised outputs

Per each PR's own "Supervised outputs" checkbox: #82 (PWM) and #84 (GPIO,
including SE_RST) are the two PRs in this batch that can alter a rail,
reset line, or PWM output as part of normal operation, not just as a bench
provocation. Treat both phases (5.2 and 4.1) as requiring the same care a
live rail/reset change always does — know the safe state of whatever is
attached before commanding an edge.

## PR cross-reference

| Phase | PRs | Issues | Brick class |
|---|---|---|---|
| 0 | #81 | #59, #45, #47 | No |
| 1 | #90, #113, #106 | #14, #27 (hygiene 1), #26, #104 | **Yes** |
| 2 | #92, #73 | #79, #5, #3, #2, #28 | **Yes** |
| 3 | #83 | #4, #10, #7, #38, #41 | No |
| 4 | #84 | #61, #62, #60 | No |
| 5 | #86, #82 | #42, #16, #43, #8, #89 (unresolved) | No |
| 6 | #75 | #33, #15 | No |
| 7 | #80 | #44, #34, #17 | No |
| 8 | #85 | #46 | No |
| 9 | #91 | #13, #54 (declined) | No |
| 10 | #99 | #36 | Low, self-recovering |
| — | (not in batch) | #25 (chunk_max), #53→#64 (RTC wake) | — |
