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

**`-DBRIDGE_OTA_PARTITIONED=ON`** (requires `BRIDGE_HAL_BACKEND=gd32`)
arms the in-system upgrade path and emits the partitioned set instead:
`gd32-bootloader` (32 KB at flash base), `gd32-bridge-slot-a` and
`gd32-bridge-slot-b` (the app linked per A/B slot, `.ramfunc` FMC loop
in RAM, `SCB->VTOR` relocated).  First-flash of a partitioned part also
needs the factory A/B metadata record —
[`tools/gen_ota_metadata.py`](tools/gen_ota_metadata.py) generates it
(flash to `0x08008000`); without it the bootloader idles in its
recovery loop.  The full Path-A wire contract is
[`docs/gd32-bridge-protocol.md` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/docs/gd32-bridge-protocol.md) §10.

Validated on silicon 2026-06-04 (bench, protocol v0.6) for the A→B
update + rollback direction only — B→A has **not** been exercised.
Still HIL-gated: a bad bootloader bricks the part, and this HW
revision has no host-driven SWD reflash, so recovery needs a bench SWD
probe on the physical board.

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
> wired in `hal/gd32/adc_stream.c`.  The stub backend stays HW-free for
> host protocol round-trip tests.

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

## Cross-link

* Protocol wire spec: [`docs/gd32-bridge-protocol.md` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/docs/gd32-bridge-protocol.md).
* Host-side driver public API: [`include/alp/chips/gd32g553.h` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/include/alp/chips/gd32g553.h).
* Host-side driver implementation: [`chips/gd32g553/gd32g553.c` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/chips/gd32g553/gd32g553.c).
* GD32 pad map: [`metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv` (alp-sdk)](https://github.com/alplabai/alp-sdk/blob/main/metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv).
