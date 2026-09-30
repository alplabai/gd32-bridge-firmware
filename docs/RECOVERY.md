# Recovering a GD32 bridge without a toolchain

The GD32G553 bridge on an E1M-X V2N / V2N-M1 SoM is flashed by Alp at the
factory. If its image is lost or corrupt, restore it from the prebuilt images
attached to a GitHub Release of this repository; no compiler or vendor library
is needed. This document is the public recovery path. Factory provisioning
(option bytes, per-unit records, manufacturing fixtures) is separate and
internal.

## 1. Get and verify the images

Download from the Release page for the tag you want (`vX.Y.Z`):

| Asset | Flash address | Use |
|---|---|---|
| `gd32-bridge-vX.Y.Z-bootloader.bin` | `0x08000000` | 32 KB partitioned bootloader |
| `gd32-bridge-vX.Y.Z-ota-meta-rec0.bin` | `0x08008000` | factory A/B record, slot A active |
| `gd32-bridge-vX.Y.Z-slot-a.bin` | `0x0800A000` | application, slot A |
| `gd32-bridge-vX.Y.Z-slot-b.bin` | `0x08040000` | application, slot B (bank-aligned) |
| `gd32-bridge-vX.Y.Z-full-flash.bin` | `0x08000000` | non-partitioned single image (no OTA) |
| `SHA256SUMS` | | checksums of every `.bin` |
| `*.sig` | | detached signatures (only on signed releases) |

Use the partitioned set (bootloader + metadata + slot A, optionally slot B) for
a normal recovery. The full-flash image is a fallback with OTA disabled.

```sh
sha256sum -c SHA256SUMS
# signed releases only; PUBKEY is Alp's published release-signing public key
# (keys/alp_release_signing_ecdsa_p256.pub.pem in the alp-sdk repository)
openssl dgst -sha256 -verify PUBKEY -signature SHA256SUMS.sig SHA256SUMS
```

The release notes state `UNSIGNED` when no signing key was configured for that
build. Then only the checksums apply, and they prove integrity, not who built it.

## 2. Path A: external SWD probe (J-Link or OpenOCD)

Connect the probe to the GD32 SWDIO/SWCLK on the module's programming header
and power the module. Metadata must be erased before it is written: both
records (`0x08008000` and `0x08008800`) must be cleared, because the bootloader
and application pick the record with the highest counter and a stale second
record would keep an old slot active.

J-Link Commander (device: GD32G553):

```
erase 0x08008000 0x0800A000
loadbin gd32-bridge-vX.Y.Z-bootloader.bin,0x08000000
loadbin gd32-bridge-vX.Y.Z-ota-meta-rec0.bin,0x08008000
loadbin gd32-bridge-vX.Y.Z-slot-a.bin,0x0800A000
loadbin gd32-bridge-vX.Y.Z-slot-b.bin,0x08040000
r
g
```

Slot B is optional for boot; flash it so a later OTA has a known-good peer.
OpenOCD: use `program <file> <address>` per line for the same files and
addresses, after an explicit erase of `0x08008000..0x0800A000`.

Loading the images does not set option bytes (write protection, BOOTLK, SPC).
A recovered board is functional without them; they are a factory step.

## 3. Path B: host-driven SWD from the V2N A55

The SoM routes the GD32 SWDIO/SWCLK (and NRST) back to SoC pads, so the Linux
side can act as the SWD probe with no external hardware. Generically:

1. Copy the verified `.bin` files to the V2N.
2. Run a host-side SWD master (for example OpenOCD with a sysfs/libgpiod
   bit-bang adapter) on the SoC pads wired to GD32 SWDIO/SWCLK, with NRST
   under host control.
3. Perform the same erase-then-program sequence and addresses as Path A.
4. Release NRST and let the GD32 boot.

The SoC pad names, pin-mux setup and the ready-made scripts used on Alp's
bench are internal and are not part of this repository.

## 4. Verify

After reset, read the protocol version over BRD_I2C at 7-bit address `0x70`.
`CMD_GET_VERSION` is opcode `0x01`. Frame (see `src/transport_i2c.c` and the
alp-sdk `docs/gd32-bridge-protocol.md`):

- write: register `0x00`, `CMD`, payload, CRC over `CMD..payload`
- repeated-start read: `STATUS`, payload, CRC over `STATUS..payload`

A healthy bridge returns `STATUS_OK` and a 3-byte payload
`PROTOCOL_VERSION_MAJOR, MINOR, PATCH`. `CMD_GET_BUILD_ID` identifies the
exact build; compare it with the release you flashed. If the read gets no ACK
at `0x70`, re-check the erase step and that the metadata record was written.
