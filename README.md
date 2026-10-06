@page firmware_gd32_bridge_index GD32 bridge firmware

# gd32-bridge

> **This repository holds the GD32G553 bridge FIRMWARE only.**
> The wire contract, the host-side driver and all hardware metadata stay in
> [alplabai/alp-sdk](https://github.com/alplabai/alp-sdk) — the dependency runs
> firmware -> alp-sdk, never the other way, so alp-sdk owns the contract it
> publishes. Extracted from `alp-sdk:firmware/gd32-bridge` with history intact
> (alp-sdk#1370): this is product firmware programmed on Alp Lab's line, on a
> different cadence and under different authority from the SDK.
>
> The frame-parser fuzz harness deliberately stays in alp-sdk
> (`tests/fuzz/gd32_bridge_frame_fuzz.c`): it consumes peer-supplied data and
> cross-checks its CRC against this firmware's `crc16_ccitt_false`, so it must
> keep linking the real `src/protocol.c`. Point it here with
> `-DALP_GD32_BRIDGE_FIRMWARE_DIR=<this checkout>`.

Firmware that runs on the **GigaDevice GD32G553MEY7TR** supervisor
MCU on the E1M-X V2N / V2N-M1 SoMs.  Serves the Renesas RZ/V2N host
over the **hybrid SPI + I2C bridge** documented in
[`docs/gd32-bridge-protocol.md` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/docs/gd32-bridge-protocol.md).

This tree is a **separate compile artifact** with its own toolchain
(ARM-GCC for Cortex-M33) and its own flash binary.  It is **not**
linked into the Zephyr-side `alp-sdk` library; the matching
host-side driver lives at [`chips/gd32g553/` (alp-sdk)](https://github.com/alplabai/alp-sdk/tree/main/chips/gd32g553/).

## Tree layout

```
gd32-bridge-firmware/
├── CMakeLists.txt          ← top-level build entry (host-built, cross-compiled)
├── README.md               ← this file
├── vendor/                 ← Alp-authored build glue for the GigaDevice library (wrapper
│                              CMake + clock patch); the library itself is fetched, not committed
├── toolchain/              ← ARM-GCC + linker script for GD32G553MEY7TR
├── hal/                    ← thin shims around the GigaDevice firmware library
├── src/
│   ├── main.c              ← startup + dispatch loop
│   ├── protocol.c          ← shared command-handler table  ← single source
│   ├── protocol.h          ← internal header
│   ├── transport_spi.c     ← SPI-slave receive + reply staging
│   ├── transport_i2c.c     ← I2C-slave receive + reply staging
│   ├── ota.c               ← OTA Path-A state machine
│   ├── crc32.c             ← CRC-32 (OTA image/metadata)
│   └── boot/, bootloader/  ← application bootloader
└── tests/
    └── protocol_vectors.txt  ← canonical CRC + wire vectors (shared with host tests)
```

The transport layer is **per-bus**, but every transport calls the
**same** `protocol_dispatch()` entry point so that adding an opcode
is a one-place change — never "fork the protocol".  This mirrors
the project-memory rule:

> Same command frame on both transports … one framing format, one
> command set, one set of reply codes; only the transport layer
> differs.

## Build

The CMake build runs **outside** the Zephyr build (the Renesas
side's `west build` does not descend here).  Invoke directly:

```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=toolchain/arm-none-eabi.cmake
cmake --build build
```

`BRIDGE_HAL_BACKEND` defaults to `stub`, so this command emits only
`build/gd32-bridge.elf` — the stub backend links no vendor
`Reset_Handler`, so there's nothing for objcopy to extract into
`.hex`/`.bin`. For a flashable image, select the `gd32` backend. The
GigaDevice firmware library is **not** in this repository (it is GigaDevice's
code under its own terms, see [`THIRD_PARTY_NOTICES.md`](THIRD_PARTY_NOTICES.md));
fetch it once from GigaDevice's official repository:

```bash
tools/fetch_gd32_library.sh        # pinned commit + tree-hash verified
cmake -B build -DCMAKE_TOOLCHAIN_FILE=toolchain/arm-none-eabi.cmake \
    -DBRIDGE_HAL_BACKEND=gd32
cmake --build build
```

The script clones `GigaDevice-GD32-MCU/GD32G5x3_Firmware_Library` tag `V1.5.0`
into `vendor/gd32_firmware_library/upstream/` (gitignored) and fails closed
unless the commit and the `Firmware/` tree hash match its pins. A verified
checkout is reused, so a cached `upstream/` works offline;
`GD32_LIBRARY_URL` points it at a local mirror (pins still enforced).
`GD32_VENDOR_DIR` is empty by default and only needed to substitute your own
wrapper directory. The build emits the monolithic
`build/gd32-bridge.elf` + `.hex` + `.bin` (OTA inert — the whole
`0xF0..0xFF` range answers `STATUS_NOSUPPORT`, so the image cannot
brick itself).

**A flashable image needs the IRC8M clock patch.** The stock vendor
`system_gd32g5x3.c` selects `__SYSTEM_CLOCK_216M_PLL_HXTAL`, whose startup
spins `while(1){}` waiting for `HXTALSTB` — which never sets on this SoM, so the
part hangs before `main()` and the flashed board looks bricked, with SPI and I2C
never coming up. The in-repo wrapper (`vendor/gd32_firmware_library/`) applies
`patches/system_gd32g5x3-irc8m.patch` to a build-directory copy of that file
(selects `__SYSTEM_CLOCK_216M_PLL_IRC8M` instead). A custom `GD32_VENDOR_DIR`
without `patches/system_gd32g5x3-irc8m.patch` fails configure with a clear
message. **`-DBRIDGE_ALLOW_STOCK_SYSTEM_INIT=ON`** silences that failure and links
the stock, hanging `SystemInit()` — compile-and-link coverage only; never pass
it for an image you intend to flash.

### System clock (IRC8M by default, HXTAL bypass on request)

The part has no crystal: OSCIN (PF0, pin E9) is driven single-ended by the
5L35023B SE2 output (22 ohm series, net `GD32_OSC`, test point TP88); OSCOUT
(PF1) is the E1M IO13 pad and is meant to stay a GPIO, because bypass mode
leaves it unused (a Phase 11 bench check, not something the vendor header
states). The shipped OTP image has SE2 free-running 32.768 kHz, which is why
`HXTALSTB` never set under the stock path; the plan is to route SE2 from a PLL
divider (DIV4, 24.576 MHz) via U-Boot volatile register writes, and the next
build burns the same setting into the clock generator OTP. 24.576 MHz is the
only source on any board.

**Boot is always the IRC8M 216 MHz PLL, exactly as before.** On current units
SE2 only reaches its final frequency when U-Boot configures the clock
generator, after the GD32 has booted, so locking the PLL to HXTAL
autonomously could be followed by a frequency change (e.g. locked at 8 MHz x
108, then SE2 moves to 24.576 MHz = a ~664 MHz overclock). The switch to the
external clock therefore happens only when the host asks:

- `bridge_clock_try_hxtal()` (`hal/gd32/clock_hw.c`) is the internal seam the
  protocol layer will call; the wire opcode (and the status fields) belong to
  the protocol 0.15 work, tracked in alplabai/gd32-bridge-firmware#330, and are
  deliberately not invented here. It is base-level only (it holds PRIMASK for
  up to ~23 ms). `bridge_clock_request_hxtal()` is the ISR-safe latch and
  `bridge_clock_tick()` (from `bridge_hw_tick()`) advances it. For the bench,
  writing 1 to the SWD variable `bridge_clock_hxtal_request` does the same.
- **Bus-quiet contract.** The switch runs only after the request has been
  latched AND both transports have been idle (CS high, no I2C transaction or
  address match pending) for 3 consecutive 50 ms ticks; a request that never
  gets quiet buses ends after ~10 s. The host must keep BOTH buses idle from the
  request until the status leaves RUNNING: the core clock stretches ~27x while
  the PLL is rebuilt and a transfer in flight can fail its CRC.
  `bridge_clock_switch_status` (SWD-readable now, the field the 0.15 opcode will
  report) is 0 IDLE, 1 PENDING, 2 RUNNING, 3 DONE_HXTAL, 4 DONE_FALLBACK (reason
  in `bridge_clock_fallback`), 5 REFUSED_BUSY.
- **Refused while supervised outputs run.** The switch is refused (status 5,
  nothing latched, no register touched) while any PWM channel (output or
  capture) or ADC stream is active, so no supervised output ever sees the
  period stretch; anything started after a successful switch uses the live
  prescaler. The quadrature encoders are not a blocker: they count external
  edges with no SYSCLK-derived period (only the input-filter time scales with
  the clock), are enabled at boot and have no session.
- `-DBRIDGE_CLOCK_HXTAL_AT_BOOT=ON` (default OFF) attempts it in
  `bridge_hw_init()` instead, for an SoM whose SE2 is right from POR via the
  clock generator OTP image. That attempt runs with the FWDGT already armed
  (except under an OTA trial, where the bootloader dog is already counting).

The sequence (`clock_hw.c`, sequencing in `clock_source.c`):

1. `CKMEN` off, `HXTALEN` off, then `RCU_CTL_HXTALBPS`, then `HXTALEN`;
2. wait for `HXTALSTB`, bounded by TIME (5 ms, DWT cycle counter against the
   live core clock; never an iteration count: the Deep-sleep wake runs this at
   8 MHz under PRIMASK);
3. **frequency check**: the on-chip HXTAL/32 trigger source
   (`TRIGSEL_INPUT_HXTAL_DIV32_TRIG`) is routed to TIMER14 `ITI14` and counted
   over a 2 ms DWT-timed window while SYSCLK is still IRC8M-derived; TIMER14,
   its SYSCFG routing and its TRIGSEL target are released afterwards (TIMER14 is
   used by nothing else in this firmware). The count is matched to the
   reference table below. This routing is taken from the vendor header and
   timer driver and has not run on silicon; if it counts nothing the result is
   the safe one (`FB_HXTAL_FREQ`, stay on IRC8M);
4. the PLL is moved onto HXTAL with that entry tuple: AHB steps /2 then /4
   before leaving the 216 MHz PLL and returns to /1 once SYSCLK is on IRC8M
   (the vendor Vcore step, `RCU_MODIFY_DE_2`), PLL off, reprogrammed, relocked,
   SYSCLK back to PLLP, then `CKMEN` armed;
5. on any failure HXTAL is stopped and the IRC8M 216 MHz PLL is rebuilt.

| OSCIN | HXTAL/32 | PLLPSC (field) | PLLN | PLL in | VCO | SYSCLK | band |
|---|---|---|---|---|---|---|---|
| 8 MHz | 250 kHz | /2 (1) | 108 | 4 MHz | 432 MHz | 216 MHz | +-3% |
| 12 MHz | 375 kHz | /3 (2) | 108 | 4 MHz | 432 MHz | 216 MHz | +-3% |
| 16 MHz | 500 kHz | /4 (3) | 108 | 4 MHz | 432 MHz | 216 MHz | +-3% |
| 20 MHz | 625 kHz | /5 (4) | 108 | 4 MHz | 432 MHz | 216 MHz | +-3% |
| 24.576 MHz | 768 kHz | /6 (5) | 105 | 4.096 MHz | 430.08 MHz | 215.04 MHz (-0.44%) | +-1% |

PLLP is /2 (field 0) throughout. There is no 24.000 MHz entry (no board feeds
it). Every tuple stays at (or, for 24.576 MHz, next to) the IRC8M path proven
4 MHz / 432 MHz input and VCO: the vendor header gives only the field ranges
(PLLPSC /1../16, PLLN 8..180, PLLP /2../8), not input/VCO limits. A 32.768 kHz
OSCIN never matches (`HXTALSTB` normally does not set inside the budget; if it
did, the count is ~2).

**Tolerance.** The count is taken against the IRC8M-derived core clock, so a
band has to hold the IRC8M own error plus quantisation (<= 0.2%). The IRC8M
datasheet figure is not in this tree and must be checked, so the bands are a
safety choice, not a measurement. The band is on the HXTAL/32 COUNT: a fast
IRC8M gives a LOW count and a slow one a HIGH count. 24.576 MHz, the only real
source, is +-1%: an IRC8M within 1% either way switches, one further off
refuses (`FB_HXTAL_FREQ`, stays on IRC8M; the safe direction). The unused
8/12/16/20 MHz entries keep +-3%. The bands do not overlap (closest pair, 20 and
24.576 MHz, is 23% apart), so a match is unambiguous.

**25 MHz must never be fed.** It has no tuple (it needs PLLPSC * PLLP to be a
multiple of 25) and is 1.7% above 24.576 MHz. With a true timebase it is
refused; it takes an IRC8M that is MORE THAN 0.7% FAST to pull it into the +-1%
band, and it would then be taken for 24.576 MHz (SYSCLK 218.75 MHz, +1.3% over
216). The part cannot prevent that, because the only reference is the IRC8M, so
SE2 must only ever be programmed by the alp-sdk U-Boot fixup (DIV4 route,
24.576 MHz; the next build burns the same setting into the OTP): no other path
may write the clock generator.

**SYSCLK is not always 216 MHz.** Everything derives from the live clock:
`SystemCoreClock` comes from `bridge_clock_core_update()` (the vendor
`SystemCoreClockUpdate()` hardcodes `HXTAL_VALUE` = 8 MHz as the PLL input and is
no longer called), the I2C timing reads `bridge_clock_apb1_hz()`, SysTick is
re-sized after every change, and the PWM / ADC-pacing timer prescalers use
`bridge_timer_prescaler()` (`/215` at 215.04 MHz, not a stale `/216`).
`bridge_core_clock_matches` now means "SYSCLK is what the active selection is
meant to produce". The 216 MHz literals left in `gd32_common.h`
(`PWM_TIMER_CLK_HZ`, `BRIDGE_ADC_PACE_CLK_HZ`, `ADC_READ_ADCCK_HZ`) are nominal
IRC8M-path values and program no hardware. A timer configured before a switch
keeps its old prescaler, which is why the switch is refused while PWM or ADC
streaming is active. `bridge_hw_counter_read()` returns raw core-clock ticks;
the core clock is not on the wire yet (#330), so a host that converts them
assumes 216 MHz and is 0.44% off after a 24.576 MHz switch.

**The host must never reprogram SE2 after the switch.** The clock monitor only
detects a STOPPED clock, not a changed one. The U-Boot sequence comes first;
Linux and userspace must not touch SE2 once the PLL runs from it.

**Boot / wake timing.** Waits are bounded by time, not spins. Typical cost with a
good clock is ~3 ms (HXTAL start well under 1 ms + the 2 ms count window + PLL
lock); SE2 off costs the 5 ms startup budget; the worst bounded chain (HXTAL up,
PLL refusing it, IRC8M PLL rebuilt) is 23 ms. The earlier 15-25 ms figure came
from a 1e6-iteration spin, which at the 8 MHz of a Deep-sleep wake is 1.2-1.9 s
under PRIMASK against the 445 ms minimum FWDGT window.

**Clock monitor.** If it reports HXTAL failure, `fault_nmi_entry` (only
`SYSCFG_STAT_CKMNMIIF`, tolerating the single-bit-ECC event bits) acknowledges
(`CKMEN` off, then `RCU_INT.CKMIC`, then `SYSCFG_STAT.CKMNMIIF`: without the RCU
clear the NMI re-fires forever), stops HXTAL, rebuilds the 216 MHz PLL from
IRC8M and RESUMES; any other NMI source, and a second CKM NMI while already on
IRC8M, keeps the record-and-reset policy. The part runs at 8 MHz inside that NMI
until the (bounded) PLL relock completes, and a transfer in flight at that
instant can fail its CRC. A CKM NMI that lands while a switch is in flight only
records; the thread side performs the fallback. `CKMEN` is cleared before
Deep-sleep entry and at the start of every restart (never HXTALEN, though,
while the PLL runs from it: a WFI that returns without sleeping leaves HXTAL and
its PLL live and only re-arms the monitor, and a restart that does happen leaves
the PLL locked and selected so the wake path skips its PLLEN replay), and the active source is
recorded before `CKMEN` is armed.

**Attempt marker.** `RTC_BKP9` holds `0x48545831` from just before an HXTAL PLL
switch until the first healthy `bridge_hw_tick()` after it (an OTA trial
included; arming the FWDGT is separate). A boot that finds it set refuses HXTAL
for that boot (`FB_PREV_BOOT_UNHEALTHY`). Limits: it does not help across a power
cycle where the backup domain is not retained, and a misclock that still reaches
a healthy tick is not caught by it (the frequency check covers part of that).

**Active source.** SWD-readable RAM symbols `bridge_clock_source`
(`BRIDGE_CLOCK_SRC_IRC8M` = 0, `BRIDGE_CLOCK_SRC_HXTAL` = 1),
`bridge_clock_input_hz` (0 = unknown / not in use, else the classified
reference: 8000000, 12000000, 16000000, 20000000 or 24576000) and `bridge_clock_fallback` (0 none,
1 build-disabled, 2 `HXTALSTB` timeout, 3 PLL fail, 4 CKM failure, 5 previous
attempt unhealthy, 6 frequency matches no reference, 7 the IRC8M PLL itself
would not relock (left on bare 8 MHz), 8 not requested yet). Nothing is on the
wire yet: no existing reply has a spare field, and a new field needs a protocol
bump, so a protocol 0.15 field will carry it.

**IRC8M-only build.** `-DBRIDGE_CLOCK_HXTAL=OFF` compiles the HXTAL path out
(`BRIDGE_CLOCK_IRC8M_ONLY`); the active source is then always IRC8M with
fallback 1. `tools/check_clock_override.py` checks both shapes (`--irc8m-only`
for the OFF build) and that the bootloader (named with `--bootloader`) never
links the HXTAL path; CI `clock-images` job runs it on the default and OFF
images.

**`-DBRIDGE_TIMING_STATS=ON`** (bench only, default OFF) records per-SPI-transaction
DWT cycle counts in a RAM struct read over SWD; see
[`docs/timing-stats.md`](docs/timing-stats.md).  OFF adds nothing to the image.

**`-DBRIDGE_OTA_PARTITIONED=ON`** (requires `BRIDGE_HAL_BACKEND=gd32`)
arms the in-system upgrade path and emits the partitioned set instead:
`gd32-bootloader` (32 KB at flash base), `gd32-bridge-slot-a` and
`gd32-bridge-slot-b` (the app linked per A/B slot, `.ramfunc` FMC loop
in RAM, `SCB->VTOR` relocated).  First-flash of a partitioned part also
needs the factory A/B metadata record —
[`tools/gen_ota_metadata.py`](tools/gen_ota_metadata.py) generates it:

```bash
python3 tools/gen_ota_metadata.py --slot-image build/gd32-bridge-slot-a.bin --out ota-meta-rec0.bin
```

flash the result to `0x08008000` (`OTA_META_REC0`) alongside the
bootloader (`0x08000000`) and the slot-A image (`0x0800A000`); without
it the bootloader idles in its recovery loop.  The full Path-A wire
contract is
[`docs/gd32-bridge-protocol.md` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/docs/gd32-bridge-protocol.md) §10.

Validated on silicon 2026-06-04 (bench, protocol v0.6) for the A→B
update + rollback direction only — B→A has **not** been exercised.
Still HIL-gated: a bad bootloader bricks the part, and this HW
revision's recovery paths (prebuilt release images, external SWD
probe, host-driven SWD from the V2N, OTA) are in
[`docs/RECOVERY.md`](docs/RECOVERY.md); no toolchain is needed.

Development flashing uses an external SWD probe on `GD32_SWDIO` /
`GD32_SWCLK` (J-Link, ST-Link, OpenOCD).

> **Status:** Both backends build clean.  The gd32 backend drives the
> real peripheral HAL (per-peripheral TUs under `hal/gd32/`), the
> **SPI1 + I2C0 slave transports** (`hal/transport_hw_gd32.c`), and
> the **OTA Path-A state machine** (`src/ota.c` + `hal/fmc_ota.c` —
> silicon-validated 2026-06-04 for A→B update + rollback; B→A not yet
> exercised; armed only with `BRIDGE_OTA_PARTITIONED`).  The fw
> v0.2.3–v0.2.7 campaign cleared the soak-quarantined HAL defects
> (`pwm_capture`, `adc_stream`,
> `qenc`, `tmu` — silicon-validated; the analog subsystem additionally
> needed the v0.2.6 internal-VREF bring-up).  The ADC DSP-chain runtime
> dispatch (FIR/IIR via the FAC, FFT via `CMD_ADC_SPECTRUM_READ`) is
> wired in `hal/gd32/adc_stream.c`; the chain pool and `chain_bind`
> capability validation (#69, #70) live in the vendor-header-free
> `hal/gd32/adc_dsp_chain.c`, which a host suite links directly
> (`tests/unit/adc_dsp/`).  The stub backend stays HW-free for host
> protocol round-trip tests.

## Protocol majorset

The firmware ships with a build-time `PROTOCOL_VERSION_MAJOR`
constant in [`src/protocol.h`](src/protocol.h).  Bumping the major
breaks every host that has not been rebuilt against the matching
[`<alp/chips/gd32g553.h>`](https://github.com/alplabai/alp-sdk/blob/main/include/alp/chips/gd32g553.h) -- treat
it as a wire-incompatible change and stage carefully.

## GPIO + PWM channel maps

The wire-side **logical** ids that the protocol uses do **not**
match GD32 silicon pad indices.  `src/protocol.c` passes channel ids
to the HAL untranslated; all pad translation happens in the gd32
backend, in [`hal/gd32/gpio.c`](hal/gd32/gpio.c) (`gpio_pad_map[]`)
and [`hal/gd32/pwm.c`](hal/gd32/pwm.c) (`pwm_channels[]`), with the
ADC and encoder maps in `hal/gd32/adc.c` (`adc_channels_map[]`) and
`hal/gd32/qenc.c` (`qenc_map[]`); all four are declared `extern` in
`hal/gd32/gd32_common.h`.  Sourced from
[`metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv).
Host code reaches a channel by its logical id; the firmware
translates internally.

`gpio_pad_map[]` is 21 entries: 18 E1M IO pads (bits 0-17) plus three
sideband bits (18, 19, 20) that are not E1M pads at all -- `BT_REG_ON`
(GD32 `PE14`) and `WL_REG_ON` (GD32 `PE15`), the Murata
LBEE5HY2FY-922 Wi-Fi/BT module's power enables, and `CAN_STBY` (GD32
`PB13`), the shared standby line for the two on-module TCAN1044
CAN-FD transceivers (U15/U16). Unlike the E1M pads,
which boot INPUT, high-Z (no internal pull -- the carrier's own
pulls define the default; see the boot-loop comment in
[`hal/gd32/init.c`](hal/gd32/init.c)), these two boot **OUTPUT driven
LOW** (module off); the module has internal 50 k pull-downs on both,
so an input pad would leave the module's power state indeterminate.
Module power is host policy, not a firmware default: a host powers
the module by writing bits 18/19 high via `CMD_GPIO_WRITE`. `CAN_STBY`
instead boots **OUTPUT driven HIGH** (both transceivers held in
standby -- STB HIGH = standby, STB LOW = normal); a host takes the CAN
bus live by writing bit 20 low via `CMD_GPIO_WRITE`. This table
(source of truth: `gpio_pad_map[]` in
[`hal/gd32/gpio.c`](hal/gd32/gpio.c)) is the owner of the bit layout --
`docs/gd32-bridge-protocol.md` (alp-sdk) links back here instead of
repeating it:

| Bit | GD32 pad | Signal      |
|----:|----------|-------------|
|   0 | PB10     | E1M IO8     |
|   1 | PA7      | E1M IO9     |
|   2 | PA12     | E1M IO10    |
|   3 | PB0      | E1M IO11    |
|   4 | PC1      | E1M IO12    |
|   5 | PF1      | E1M IO13    |
|   6 | PB5      | E1M IO14    |
|   7 | PC0      | E1M IO16    |
|   8 | PC14     | E1M IO24    |
|   9 | PC15     | E1M IO25    |
|  10 | PB11     | E1M IO27    |
|  11 | PC2      | E1M IO28    |
|  12 | PD11     | E1M IO29    |
|  13 | PD10     | E1M IO30    |
|  14 | PE12     | E1M IO31    |
|  15 | PD2      | E1M IO32    |
|  16 | PD8      | E1M IO34    |
|  17 | PD1      | E1M IO35    |
|  18 | PE14     | BT_REG_ON   |
|  19 | PE15     | WL_REG_ON   |
|  20 | PB13     | CAN_STBY    |

Bits 8/9 (`PC14`/`PC15`, E1M IO24/IO25) are not ordinary pads: they are
supplied through the backup-domain power switch together with SE_RST
(`PC13`, the OPTIGA Trust M reset line), sharing a typical 3 mA source
budget, capped at 2 MHz output toggle rate with a 30 pF max load
(GD32G553xx Datasheet Rev2.0 p.130 Table 4-29 footnote 2; GD32G553 User
Manual Rev1.2 p.133 §3.3.1). `GPIO_OSPEED_12MHZ` is already the slowest
speed class the part offers, so the 2 MHz cap cannot be met by a
firmware register change -- the host must not toggle IO24/IO25 faster
than 2 MHz or load them beyond 30 pF, and current drawn through them
competes with the milliamps holding SE_RST released. A carrier or host
that ignores this budget can sag SE_RST below the OPTIGA's released
threshold without either side seeing why.

Any GD32 reset (WDT, fault, OTA A/B swap, SE reset) drops both REG_ON
lines low and drives CAN_STBY high again -- the boot-time defaults in
[`hal/gd32/init.c`](hal/gd32/init.c) apply on every reset, not just
cold power-on, so the Wi-Fi/BT module gets power-cycled and the CAN
bus goes back to standby along with it. The host must re-assert bits
18/19/20 after any GD32 reset; `CMD_RESET_REASON` is how a host
detects one happened. Firmware v0.10 and earlier silently ignore
writes to bits 18/19 and still return `STATUS_OK` (bits did not exist
yet); firmware v0.12 and earlier do the same for bit 20. A host
relying on bits 18/19 must require `PROTOCOL_VERSION_MINOR >= 11`,
and on bit 20 must require `PROTOCOL_VERSION_MINOR >= 13` (both via
`GET_VERSION`), before trusting that the write actually took effect.

## Cross-link

* Protocol wire spec: [`docs/gd32-bridge-protocol.md` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/docs/gd32-bridge-protocol.md).
* Host-side driver public API: [`include/alp/chips/gd32g553.h` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/include/alp/chips/gd32g553.h).
* Host-side driver implementation: [`chips/gd32g553/gd32g553.c` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/chips/gd32g553/gd32g553.c).
* GD32 pad map: [`metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv).
