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
├── ci/                     ← CI-only glue (e.g. the vendor-library wrapper CMakeLists
│                              staged by the `gd32 backend build` job; not GigaDevice IP)
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
`.hex`/`.bin`. For a flashable image, select the `gd32` backend and
point `GD32_VENDOR_DIR` at a checkout of alp-sdk's
`vendors/gd32_firmware_library/` (this repo does not vendor the
GigaDevice SDK):

```bash
cmake -B build -DCMAKE_TOOLCHAIN_FILE=toolchain/arm-none-eabi.cmake \
    -DBRIDGE_HAL_BACKEND=gd32 \
    -DGD32_VENDOR_DIR=<path to alp-sdk checkout>/vendors/gd32_firmware_library
cmake --build build
```

`GD32_VENDOR_DIR` is empty by default.  Left empty, the build falls
back to `../../vendors/gd32_firmware_library` **resolved against this
source tree** — the pre-split layout, when this tree was nested at
`<alp-sdk>/firmware/gd32-bridge/`.  A standalone clone has no such
parent tree, so pass the flag explicitly; a relative value you pass is
resolved against your shell's working directory, not the source tree,
so prefer an absolute path. That build emits the monolithic
`build/gd32-bridge.elf` + `.hex` + `.bin` (OTA inert — the whole
`0xF0..0xFF` range answers `STATUS_NOSUPPORT`, so the image cannot
brick itself).

**A flashable image needs the IRC8M clock override, not just a vendor
tree.** The stock vendor `system_gd32g5x3.c` selects
`__SYSTEM_CLOCK_216M_PLL_HXTAL`, whose startup spins `while(1){}`
waiting for `HXTALSTB` — which never sets on this SoM, so the part
hangs before `main()` and the flashed board looks bricked, with SPI
and I2C never coming up. `GD32_VENDOR_DIR` must therefore point at a
tree that also carries `overrides/system_gd32g5x3.c` (selects
`__SYSTEM_CLOCK_216M_PLL_IRC8M` instead) — an alp-sdk checkout's
`vendors/gd32_firmware_library/` carries this override; the public
[`gd32g5x3-firmware-library`](https://github.com/alplabai/gd32g5x3-firmware-library)
mirror alone does not. Configure fails fast with a clear message if
the override is missing. **`-DBRIDGE_ALLOW_STOCK_SYSTEM_INIT=ON`**
silences that failure and links the stock, hanging `SystemInit()`
instead — it exists only for compile-and-link coverage (CI's `gd32
backend build` job, which never runs on silicon); never pass it for an
image you intend to flash.

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

## Protocol v0.15 (negotiated)

`GET_VERSION` reports `0.15.0`. The full design -- wire layouts, the grant
algorithm, the ATTN pin rules -- is
[`docs/protocol-v0.15-design.md`](docs/protocol-v0.15-design.md); the points a
firmware reader needs:

* **Everything new is opt-in per link.** The 6-byte form of `CMD_LINK_FEATURES`
  (`want:u32`, `max_payload_req:u16`; 10-byte reply `granted`, `supported`,
  `max_payload`) negotiates `STATUS_SEQ`, `BIG_FRAME`, `ATTN`, `ADC_STREAM2` and
  `BATCH` on the SPI link. The legacy 1-byte form still keeps only `STATUS_SEQ`
  and clears the rest, so an un-negotiated SPI link is byte-identical to v0.14
  in both directions.
* **`BIG_FRAME`** lifts the payload ceiling to at most 252 bytes (a 256-byte SPI
  frame) for exactly two things: `CMD_BATCH` (`0x04`) requests and replies, and
  `CMD_ADC_STREAM_READ2` (`0x3C`) replies. Every other opcode -- OTA
  `0xF0..0xFF` included -- keeps the 65-byte limit, which
  `src/transport_spi.c` enforces after the CRC check. `OTA_CHUNK_MAX` stays 56
  (asserted in `src/ota.c`).
* **`ATTN`** is a level on GD32 `PA14` (`P71` on the RZ/V2N): high means a reply
  is armed or a watermark event is pending. `PA14` is also SWCLK, so the
  firmware drives it only while `ATTN` is in the link's feature word, never
  while a debugger is attached (`DHCSR.C_DEBUGEN`), and never adds it to a GPIO
  lock mask. Any reset returns it to SWCLK. The drive points live in
  `hal/transport_hw_gd32.c`; the pin is `BRIDGE_ATTN_*` in
  [`hal/bridge_board_config.h`](hal/bridge_board_config.h).
* **`CMD_ADC_STREAM_BEGIN2` / `_READ2`** report the *realised* sample rate
  (`tick_hz / period_ticks`), refuse a rate whose conversion time does not fit
  the pacing period, size the ring at `2 x watermark`, and report overrun as
  `dropped` on an `OK` reply (never `BUSY`) with a `first_index` delivered
  index; `dropped = 0xFFFFFFFF` means a gap of unknown length. The
  vendor-header-free arithmetic is `hal/gd32/adc_stream2.c`, host-tested in
  `tests/unit/adc_stream2/`.
* **`CMD_BATCH`** runs up to 16 allow-listed operations through the same
  handlers as standalone requests, validates the whole request before executing
  anything, and stops at the first non-`OK` sub-status.
* **The one unconditional change is the I2C opcode allow-list**: the I2C link
  carries only `0x00..0x03`, `0x10`, `0x11`, `0x41`, `0x81` and `0xF0..0xFF`;
  every other opcode answers an empty `STATUS_NOSUPPORT` and never reaches its
  handler. `bridge_i2c_denied_count` / `bridge_i2c_denied_last_cmd` (SWD-readable)
  record the refusals. SPI is unrestricted.

The cost: `.bss` grows by about 1 KB (256-byte SPI buffers, a 252-byte static
reply scratch in place of a 65-byte stack buffer), and `BRIDGE_SPI_DMA_BUF_LEN`
is 260. Bench items that remain open are in
[`docs/BENCH.md`](docs/BENCH.md) Phase 11.

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
