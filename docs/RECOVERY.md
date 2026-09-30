# Recovering a GD32 bridge without a toolchain

The GD32G553 bridge on an E1M-X V2N / V2N-M1 SoM is flashed by Alp at the
factory. If its image is lost or corrupt, restore it from the prebuilt images
attached to a GitHub Release of this repository; no compiler or vendor library
is needed. This document is the public recovery path. Factory provisioning
(option bytes, per-unit records, manufacturing fixtures) is separate and
internal; a factory-provisioned unit needs the extra step in section 2.

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

`SHA256SUMS` plus its signature is enough: `sha256sum -c` then covers every
`.bin`. The per-file `.sig` files are extra and need not be checked one by one.

The release notes state `UNSIGNED` when no signing key was configured for that
build. Then only the checksums apply, and they prove integrity, not who built it.
Treat an UNSIGNED release as unattested: use it only for a bench or development
unit you accept the risk on, and for a unit you must trust either wait for a
signed release or rebuild from the tagged source (`README.md`). A `.sig` that
fails to verify, or a `sha256sum -c` failure, means stop: re-download, and if it
still fails do not flash. Never flash an image whose checksum you have not run.

## 2. Path A: external SWD probe (J-Link or OpenOCD)

Connect the probe to the GD32 SWDIO/SWCLK on the module's programming header
and power the module. Metadata must be erased before it is written: both
records (`0x08008000` and `0x08008800`) must be cleared, because the bootloader
and application pick the record with the highest counter and a stale second
record would keep an old slot active.

A factory-provisioned unit may have a write-protection (WP) area over the
bootloader and a security level (SPC) set. Then mass erase is refused and SWD
cannot write main flash, so `loadbin` below fails. First clear the WP area,
then demote SPC from low to no protection with a full mass erase, then
power-cycle the module, and only then follow the steps below. NEVER set SPC to
high level protection (`0xCC33`): it cannot be undone. A part that was never
provisioned needs none of this.

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
side can act as the SWD probe with no external hardware. The public alp-sdk
repository documents the pin routing and carries a working SWD master:
`chips/gd32_swd/` (driver) and `examples/v2n/v2n-gd32-swd-flash/` (example and
README with the resolved pads). Use it, or any SWD master on those pads, to:

1. Copy the verified `.bin` files to the V2N.
2. Halt the GD32 and erase `0x08008000..0x0800A000` (then sectors as needed).
3. Program the same files at the same addresses as Path A.
4. Release NRST and let the GD32 boot.

A generic OpenOCD sysfs/libgpiod bit-bang adapter on those pads is untested
here; prefer the alp-sdk example.

## 4. Path C: over the bridge's own OTA (bridge still answers)

If the bridge still boots and answers on BRD_I2C or SPI, no SWD is needed: use
the OTA opcodes `0xF0..0xF6` (`OTA_BEGIN`, `OTA_WRITE_CHUNK`, `OTA_VERIFY`,
`OTA_COMMIT`, `OTA_ROLLBACK`, `OTA_GET_STATE`, `OTA_ABORT`; wire contract in the
alp-sdk `docs/gd32-bridge-protocol.md` section 10). Stream the release
`slot-a.bin` or `slot-b.bin` into the inactive slot, then `OTA_VERIFY` and
`OTA_COMMIT`. Constraints:

- Only a partitioned build has OTA; a `full-flash` image answers
  `STATUS_NOSUPPORT` for `0xF0..0xFF`.
- Send the image built for the slot that `OTA_BEGIN` reports as `target_slot`.
- Anti-rollback: an image whose version is below the newest recorded in the
  metadata is refused (`OTA_GET_STATE` reports the error cause). To go to an
  older release, use Path A or B, which rewrites the metadata record.
- OTA cannot repair the bootloader or a bridge that no longer boots; use Path A
  or B for those.

## 5. Verify

After reset, read the protocol version over BRD_I2C at 7-bit address `0x70`.
`CMD_GET_VERSION` is opcode `0x01`. Frame (see `src/transport_i2c.c` and the
alp-sdk `docs/gd32-bridge-protocol.md`):

- write: register `0x00`, `CMD`, payload, CRC over `CMD..payload`
- repeated-start read: `STATUS`, payload, CRC over `STATUS..payload`

A healthy bridge returns `STATUS_OK` and a 3-byte payload
`PROTOCOL_VERSION_MAJOR, MINOR, PATCH`. `CMD_GET_BUILD_ID` identifies the
exact build; compare it with the release you flashed. If the read gets no ACK
at `0x70`, re-check the erase step and that the metadata record was written.
