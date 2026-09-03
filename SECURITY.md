<!--
Copyright (c) 2026 Alp Lab AB
SPDX-License-Identifier: Apache-2.0
-->

# Security policy

## Reporting a vulnerability

**Do not open a public issue for a vulnerability.**

Use GitHub's private vulnerability reporting on this repository
(Security → Report a vulnerability), or email **security@alplab.ai**.

Include the firmware version (`firmware-version.txt`) and how the bridge was
reached — a fault reachable only over SWD with physical access is a different
severity from one reachable over the SPI or I2C link from the host SoC.

We will acknowledge within 5 working days.

## Scope

This repository is the **GD32 bridge firmware**: the image running on the
GD32G553 supervisor on an E1M-X V2N SoM, its wire protocol, and its OTA path.

In scope here:

- The wire protocol in `src/` — framing, dispatch, and any handler that parses
  bytes supplied by the host SoC. Both transports carry it: SPI and the
  `BRD_I2C` slave path.
- The OTA path (`hal/fmc_ota.c`, `src/boot/`) and the metadata tooling in
  `tools/gen_ota_metadata.py`.

**Not in scope here, report against the right repository:**

- The *host* driver and the portable `<alp/*>` API in
  [`alplabai/alp-sdk`](https://github.com/alplabai/alp-sdk) — including
  `chips/gd32g553/`, the host half of this same protocol.
- The CC3501E bridge firmware, in
  [`alplabai/cc3501e-bridge-firmware`](https://github.com/alplabai/cc3501e-bridge-firmware).
- The GD32 firmware library, which this repo does not vendor — `CMakeLists.txt`
  resolves it outside the tree.

## Things worth knowing before you report

Documented, deliberate properties of the current hardware revision. Reporting
them is not wasted, but the response will point here:

- **Neither bridge transport is authenticated.** SPI and `BRD_I2C` are
  inter-chip links on one module; an attacker with physical access to those
  traces has more direct options.
- **The bridge is a supervisor.** By design it drives rails, resets and PWMs on
  behalf of the host. A host that can talk to the bridge can therefore do
  physical things — that is the feature, not a privilege-escalation bug. A path
  that lets a *non-host* party reach it is very much in scope.
- **OTA Path-A is default-OFF** (`BRIDGE_OTA_PARTITIONED`). If you are looking
  at the OTA code, check whether the path you are reading is even compiled into
  a shipped image.
- **The GD32 has no public-key crypto hardware.** Its only crypto engine is
  the CAU (`0x4802 1000`), which is symmetric-only — DES, TDES and AES. There
  is no PKA, HMAC, CMAC or message-hash engine on the GD32G553 (GD32G553 User
  Manual Rev1.2 p.350 §13.1; confirmed absent from the peripheral memory map
  at Datasheet Rev2.0 p.19). The reserved 64-byte `signature` field in
  `ota_img_header_t` (`src/ota_layout.h`) can never be verified on this part
  as specified — nothing in this repo populates or checks it today; only
  `body_crc32` is enforced. The on-module OPTIGA Trust M cannot stand in as
  the GD32's root of trust either, as currently wired: `BRD_I2C`'s `I2C0` is
  configured slave-only on the GD32 (`hal/transport_hw_gd32.c`) and the
  RZ/V2N masters that bus, so the GD32's only interaction with the SE is
  holding its reset line (`PC13`, `CMD_SE_RESET` = `0x41`). Any bridge-side
  image verification would have to run in the 32 KB bootloader before the
  host is even up.

## Disclosure

We aim to ship a fix or a documented mitigation within 90 days of a confirmed
report, and will credit reporters who want it. Because this firmware is flashed
over SWD onto modules, a fix may also require a release artifact and a
field-update path, not only a source change — we will say so explicitly rather
than closing on the commit.
