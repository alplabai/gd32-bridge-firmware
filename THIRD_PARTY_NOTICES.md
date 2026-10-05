# Third-party notices

## Our sources

Everything committed to this repository is Copyright 2026 Alp Lab AB and
licensed under the Apache License, Version 2.0 (see `LICENSE`). That includes
`vendor/gd32_firmware_library/CMakeLists.txt` and
`vendor/gd32_firmware_library/patches/system_gd32g5x3-irc8m.patch` (our
changes only, no GigaDevice code).

## GigaDevice GD32G5x3 Firmware Library (separate component, not relicensed)

- Component: GD32G5x3 Firmware Library V1.5.0 (CMSIS device files, startup code,
  standard peripheral drivers), Copyright (c) 2026 GigaDevice Semiconductor Inc.
  Some bundled CMSIS files are Copyright (c) Arm Limited.
- Source: <https://github.com/GigaDevice-GD32-MCU/GD32G5x3_Firmware_Library>,
  tag `V1.5.0`, commit `2d4ac55768261800d06d59a45963623fc74adaa2`
  (`Firmware/` tree `3807e194d8de55e24fc8160ad9accdd84c503651`).
- It is **not part of this repository.** `tools/fetch_gd32_library.sh` clones it
  from GigaDevice at build time and refuses to proceed unless both ids match.
- It is **not under Apache-2.0** as a whole and Alp Lab does not relicense it.
  Its terms are GigaDevice's: the per-file notices in the library (BSD-3-Clause
  for the standard peripheral library, Apache-2.0 for the CMSIS system and
  startup files and Arm CMSIS content) and the GigaDevice Software License
  Agreement SLA-GD0001 v1.1 that accompanies GigaDevice's firmware-library
  archive. In particular SLA-GD0001 limits use of the licensed code to GigaDevice
  devices and restricts making it subject to open-source licence terms. The
  authoritative text is GigaDevice's; read it before redistributing anything
  that contains the library.- The released `.bin` images link the library (unmodified, except the clock
  patch below). Binaries are redistributed with this notice, as the
  BSD-3-Clause terms below require.
- The image's `SystemInit()` is the library's `system_gd32g5x3.c` with
  `vendor/gd32_firmware_library/patches/system_gd32g5x3-irc8m.patch` applied to
  a build-directory copy (selects the internal 8 MHz RC clock instead of HXTAL).
  The modified copy is never committed or distributed in source form.

### BSD-3-Clause notice carried by the GigaDevice standard peripheral library

```
Copyright (c) 2026, GigaDevice Semiconductor Inc.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its contributors
   may be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
OF SUCH DAMAGE.
```

The Apache-2.0 files in the library (CMSIS system and startup files) carry their
own Apache-2.0 notices; the Apache-2.0 text is in this repository's `LICENSE`.
