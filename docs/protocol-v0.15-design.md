# GD32 bridge wire protocol v0.15.0 (negotiated MINOR over v0.14.0)

## 0. Scope and compatibility contract

- `GET_VERSION` returns `0.15.0` (`PROTOCOL_VERSION_MINOR 15u`).
- Everything new except the I2C opcode policy (§7) stays off until the host enables it on that link with the 6-byte `CMD_LINK_FEATURES` form (§2). An SPI link that has not negotiated is byte-identical to v0.14.0. This holds in all four directions: old host + new firmware, and new host + old firmware.
- The I2C opcode policy (§7) applies unconditionally. An audit of in-tree I2C callers found they use only allow-listed opcodes:
  - the kernel `gpio-gd32-bridge` driver (`0005-gpio-add-gd32-bridge-expander-driver.patch`: `0x00/0x01/0x10/0x11/0x41`)
  - `tools/gd32-ota-host` (`gd32g553_init` + OTA `0xF0..0xF6`)
  - `examples/v2n/v2n-brd-i2c-bringup` (`PING`/`GET_VERSION`)
- Unchanged from v0.14:
  - Little-endian integers.
  - CRC-16/CCITT-FALSE (poly `0x1021`, init `0xFFFF`, xor-out `0x0000`, non-reflected), low byte on the wire first.
  - SPI CRC covers `SOF|CMD|PAYLOAD` (request) and `SOF|STATUS|PAYLOAD` (reply).
  - I2C CRC covers `CMD|PAYLOAD` (write) and `STATUS|PAYLOAD` (read). The `0x00` register byte and the address are excluded.
  - SPI STATUS bits `[3:0]` hold the code and `[7:4]` the `STATUS_SEQ` stamp.
- No new status codes. Every SPI-visible code stays ≤ `0x0F`; the set used is `0x00` OK, `0x01` INVAL, `0x02` NOT_READY, `0x03` BUSY, `0x04` TIMEOUT, `0x05` IO, `0x06` NOSUPPORT, `0x07` NOMEM, `0x08` OUT_OF_RANGE, plus I2C-only `0x80` NO_PENDING.
- OTA opcodes `0xF0..0xFF`: request/reply layouts, `OTA_CHUNK_MAX = 56` and dispatch through `bl_dispatch_ota()` are all unchanged. They stay reachable on both links. No feature bit affects them.

## 1. New constants

| Name (firmware / host) | Value | Meaning |
|---|---|---|
| `CMD_BATCH` / `GD32G553_CMD_BATCH` | `0x04` | §6 |
| `CMD_ADC_STREAM_BEGIN2` / `GD32G553_CMD_ADC_STREAM_BEGIN2` | `0x3B` | §5 |
| `CMD_ADC_STREAM_READ2` / `GD32G553_CMD_ADC_STREAM_READ2` | `0x3C` | §5 |
| `GD32_BRIDGE_LINK_FEAT_STATUS_SEQ` | `0x00000001` | existing (v0.7) |
| `GD32_BRIDGE_LINK_FEAT_BIG_FRAME` | `0x00000002` | §3; SPI only |
| `GD32_BRIDGE_LINK_FEAT_ATTN` | `0x00000004` | §4; SPI only; needs STATUS_SEQ |
| `GD32_BRIDGE_LINK_FEAT_ADC_STREAM2` | `0x00000008` | gates `0x3B`/`0x3C`; SPI only |
| `GD32_BRIDGE_LINK_FEAT_BATCH` | `0x00000010` | gates `0x04`; SPI only |
| bits 5..31 | reserved | never granted; host sends 0 |
| `GD32_BRIDGE_MAX_PAYLOAD_BYTES` | `65` (unchanged) | base envelope; `OTA_CHUNK_MAX` derives from it |
| `GD32_BRIDGE_SPI_BIG_MAX_PAYLOAD_BYTES` / `GD32G553_BIG_MAX_PAYLOAD_BYTES` | `252` | BIG_FRAME ceiling → 256-B SPI frame |
| `GD32_BRIDGE_READ2_HDR_BYTES` | `9` | §5 |
| `GD32_BRIDGE_BATCH_MAX_OPS` | `16` | §6 |
| `GD32_BRIDGE_ADC_STREAM2_GUARD` | `8` samples | §5.4 |
| `GD32G553_V015_MIN_PROTOCOL_MINOR` (host) | `15` | §8 |

## 2. `CMD_LINK_FEATURES` (`0x81`) extension

### 2.1 Request: two accepted forms, chosen by request payload length

| `req_len` | Layout | Semantics |
|---|---|---|
| 1 (legacy, v0.7+) | `features:u8` | The link's whole feature word becomes `features & 0x01`. All other bits (BIG_FRAME, ATTN, ADC_STREAM2, BATCH) are **cleared** on that link, so `features = 0` still "disables everything". The link's `max_payload` becomes 65. Reply `granted:u8` (1 B), byte-identical to 0.14. |
| 6 (extended, v0.15) | `want:u32` @0, `max_payload_req:u16` @4 | §2.2. Reply is 10 B (§2.3). |
| anything else | n/a | `STATUS_INVAL`, empty payload, link state unchanged. |

### 2.2 Grant algorithm (extended form, arriving on link L)

1. `supported(L)` depends on the build and the link:

   | Link / backend | `supported(L)` |
   |---|---|
   | SPI, gd32 backend | `0x0000001F` |
   | SPI, stub backend (no ATTN, no ADC_STREAM2) | `0x00000013` |
   | I2C, any backend | `0x00000001`: STATUS_SEQ is echoed with no wire effect, preserving the 0.14 idempotent `features = 0` path |

2. `g = want & supported(L)`.
3. Drop ATTN from `g` if STATUS_SEQ ∉ `g`, or if `DHCSR.C_DEBUGEN` (address `0xE000EDF0`, bit 0) reads 1 (debugger attached, §4.2 F2).
4. If BIG_FRAME ∈ `g`, set `mp = clamp(max_payload_req, 65, 252)`; if that gives 65, drop BIG_FRAME. If BIG_FRAME ∉ `g`, `mp = 65`.
5. **Arm before staging the reply**, as v0.7 already does:
   - write L's feature word = `g` and L's `max_payload = mp`;
   - apply the ATTN pin transition (§4.2 F3) only if the ATTN bit actually changed. Re-sending the same `want` causes no pin activity.
6. Stage the reply:
   - It is stamped if STATUS_SEQ ∈ `g`; the host takes this stamp as its baseline, unchanged from v0.7.
   - If ATTN ∈ `g`, this reply is the first ATTN-signalled reply (§4.5 self-test).
   - If ATTN was just removed, this reply is **not** ATTN-signalled; the host reads it with the 0.14 drain rule.

Removing a bit takes effect from the next request:
- Removing ADC_STREAM2 does not stop running streams. `0x3B`/`0x3C` then answer NOSUPPORT, while `0x35` still ends a stream.
- Removing BIG_FRAME restores the 65-B limit.
- Removing ATTN returns PA14 to SWCLK (§4.2).

### 2.3 Extended reply (10 B)

| Off | Field | Type | Meaning |
|---|---|---|---|
| 0 | `granted` | u32 | bits now armed on L |
| 4 | `supported` | u32 | bits this build implements on L. A bit in `supported` but not in `granted` was refused (missing STATUS_SEQ, debugger attached, `mp` = 65). |
| 8 | `max_payload` | u16 | effective payload ceiling on L (65 or 66..252) |

### 2.4 Older firmware

| Firmware | Response to the 6-byte form |
|---|---|
| v0.7..v0.14 | `STATUS_INVAL` (`req_len != 1`, `src/protocol.c:987`) as a 4-byte error envelope; stamped if STATUS_SEQ was already armed; no state change. |
| < v0.7 | `STATUS_NOSUPPORT` via the dispatch default. |

The host never sends the 6-byte form when `GET_VERSION` minor < 15 (§8). INVAL is only a safety net.

## 3. BIG_FRAME

### 3.1 Limits
- `max_payload` ≤ 252, so the SPI frame is ≤ 256 B in either direction (`1 SOF + 1 CMD/STATUS + 252 + 2 CRC`).
- BIG_FRAME exists on SPI only. I2C stays at 65.
- On a BIG link, only these may exceed 65 B:
  - the `CMD_BATCH` (`0x04`) request and reply;
  - the `CMD_ADC_STREAM_READ2` (`0x3C`) reply.
- Every other opcode keeps the 0.14 limit of 65 for both request and reply. That includes OTA `0xF0..0xFF` (so OTA really is unchanged) and `ADC_DSP_STAGE_PUSH` (`0x38`).

### 3.2 Firmware enforcement (SPI decode, after the CRC check passes)
- `payload_len > 65` and `cmd != 0x04` → `STATUS_INVAL`.
- `cmd == 0x04` and `payload_len > mp(SPI)` → `STATUS_INVAL`.
- `reply_cap` passed to dispatch = `mp(SPI)` for `0x04`/`0x3C`, and 65 for everything else.

Why this is needed: 0.14's 72-B RX buffer truncated over-long captures. A capture merged by CS-edge coalescing with the following transaction's zero filler can pass the CRC whenever the frame CRC is byte-palindromic (the hole `src/ota.c:826-835` documents). Holding legacy opcodes to 65 B restores 0.14's exposure exactly.

### 3.3 Host rules
- Never clock more than 256 B in one CS window. The RX DMA count is 260; beyond that the 4-frame RX FIFO overflows, `RXORERR` sets, and the error seam replaces the staged reply with `STATUS_IO`.
- Never send a request over 65 B before BIG_FRAME is granted.

### 3.4 GD32 memory and timing
- `BRIDGE_SPI_DMA_BUF_LEN 72u` → `260u` (`hal/transport_hw_gd32.c:111`). That is the 256-B frame plus 4 B margin, the same margin 72 gave over 69. Add a static assert `BRIDGE_SPI_DMA_BUF_LEN >= SPI_MAX_FRAME_BYTES`.
- `src/transport_spi.c`: size `SPI_MAX_FRAME_BYTES` from the BIG constant (256) for `spi_rx_buf`/`spi_tx_buf`.
- Move the stack-resident `reply_pl[GD32_BRIDGE_MAX_PAYLOAD_BYTES]` in `decode_and_dispatch()` (`src/transport_spi.c:204`) to a static 252-B buffer. This is safe because the SPI path is non-reentrant: one EXTI5_9 vector at NVIC prio 1. The CS-EXTI stack chain shrinks by 65 B.
- `READ2` writes codes directly into the reply buffer (no local sample array). `BATCH` writes sub-replies in place (no temporary copy).
- `.bss` grows by about 1 KB. MSP stays 2K (`toolchain/*.ld`, about 1.1 K worst case measured per `hal/gd32/init.c:221-245`); re-run `-fstack-usage` and confirm no new frame enters the 88-248 B band.
- Leave I2C buffers at 65. `GD32_BRIDGE_MAX_PAYLOAD_BYTES` stays 65; add a static assert pinning `OTA_CHUNK_MAX == 56u` in `src/ota.c`.
- CPU cost (estimate, verify with `timing_stats`): the table-driven CRC is about 9 instructions per byte, so a 256-B frame takes about 11 µs at 216 MHz, twice per round trip, plus copies of up to 260 B. The non-dispatch part of the CS-rising handler on maximum frames should be ≤ about 30 µs.
- Without ATTN, the host's 35 µs staging gap (`chips/gd32g553/gd32g553.c:128`) should grow by about 45 ns per byte of request plus reply. This is an estimate; the re-read ladder covers the rest.

## 4. ATTN (data-ready / attention)

### 4.1 Signal
- GD32 `PA14` (net `GD32_SWCLK`, `metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv`) drives RZ/V2N `P71` (`renesas-peripheral-map.tsv`).
- Active HIGH, push-pull, **level** (not a pulse). Low means deasserted.
- GD32 drive: slowest speed class (`GPIO_OSPEED_12MHZ`), no pull.
- Fail-safe: whenever ATTN is not enabled, PA14 is SWCLK with its reset pull-down, so the host reads it as deasserted.

### 4.2 Ownership and enable/disable handshake (the GD32 never drives PA14 while a debugger may drive SWCLK)

Host rules:
- **H1.** In every boot stage, including A55 pinctrl and CM33 init, `P71` defaults to input (not driven).
- **H2.** The host sets `P71` to input with a rising-edge IRQ *before* it requests ATTN.
- **H3.** The host may drive `P70`/`P71` as outputs (SWD bit-bang) only if one of these holds:
  - (a) its most recent CRC-valid `LINK_FEATURES` reply on SPI showed ATTN ∉ `granted`, and it has sent no `LINK_FEATURES` since; or
  - (b) `GD32_NRST` (`P74`, open-drain, shared with ACT88760 GPIO4) is held asserted, i.e. connect-under-reset.
  
  The recovery procedure is always: first assert `P74`, then drive `P70`/`P71`. Reset puts PA13/PA14 back to SWD, and the bootloader never touches PA14.
- **H4.** Bench probes attach connect-under-reset only.

Firmware rules:
- **F1.** The firmware drives PA14 only while ATTN is in the SPI link's feature word.
- **F2.** ATTN is never granted while `DHCSR.C_DEBUGEN` (`0xE000EDF0` bit 0) = 1.
- **F3.** Pin transitions:
  - Enable: write `GPIO_BC` for PA14 (output low) → push-pull, slowest speed, no pull → mode = output. This order avoids a high glitch.
  - Disable: drive low → set `GPIO_AF_0` → mode = AF with pull-down. This restores PA14's reset SWD configuration; confirm against the UM Rev1.2 GPIOA reset values.
- **F4.** PA14 must never be added to the GPIOA lock mask. Today's mask is `0x8700` (PA15/PA10/PA9/PA8, `hal/transport_hw_gd32.c:806`), and the lock is irreversible until reset.
- **F5.** Any reset clears every link feature and returns PA14 to SWCLK: NRST, FWDGT, OTA `COMMIT`/`ROLLBACK`/trial confirm, fault reset, or STANDBY wake. During the OTA trial window every opcode, `LINK_FEATURES` included, answers BUSY, so ATTN cannot be enabled there.

### 4.3 Semantics (firmware state: `attn_on`, per-stream event bits `ev[0..1]`)

| Trigger | ATTN action |
|---|---|
| CS falling (PA8, EXTI8 handler, low-level branch) | drive LOW |
| CS-rising handler entry | drive LOW. This covers a coalesced falling edge and guarantees a low→high edge for every fresh reply. |
| CS-rising handler exit, after a **fresh** reply was staged and `spi_dma_arm_tx()` armed it | drive HIGH. "Fresh" means any call to `stage_reply()` in `src/transport_spi.c`: a decoded request, an error envelope, the `spi_slave_transport_error()` IO, or the tar-pit breaker. |
| CS-rising handler exit with no fresh stage (drain / empty transaction / quiesce failure) | HIGH if any `ev[s]` is set, otherwise stay LOW |
| Watermark reached (raw ring: DMA HTF/FTF in `DMA0_Channel0_IRQHandler` / `DMA1_Channel0_IRQHandler`, prio 3; FIR/IIR-filtered ring: base-level pump) | set `ev[s]`. Then, inside a PRIMASK section (`bridge_irq_lock()`): drive HIGH only if PA8 reads high **and** `EXTI_PD0` bit 8 is clear. Otherwise leave it; the next CS-rising exit re-evaluates. |
| `READ2` on stream `s` | clear `ev[s]`; set it again if the remaining backlog is still ≥ watermark |
| `STREAM_END` on `s`, or ATTN disabled | clear `ev[s]`; ATTN disable drives LOW and then releases the pin (F3) |
| Any `POWER_MODE_SET` transition | drive LOW before entering. STANDBY is a reset (F5). |

What the host should infer:
- ATTN goes high after its own request transaction → that reply is armed.
- ATTN goes high with no request outstanding → events are pending.

### 4.4 Timing (provisional; bench-verify with `timing_stats` plus a scope on P71/P97)

| Parameter | Guarantee |
|---|---|
| CS falling → ATTN low | ≤ 2 µs while the CS-EXTI vector is idle (prio 1, the highest configured) |
| ATTN rise | Only after the TX DMA is armed with the complete reply. The host may start the reply transaction on the edge with no staging gap. |
| CS rising → ATTN high | `t_frame` (≤ ~30 µs on 256-B frames) + `t_dispatch(op)`. `ADC_READ` is bounded by `ADC_READ_ISR_BUDGET_US` = 1000. |
| Low time around each fresh stage | ≥ 1 µs (deassert at handler entry to assert at exit) |
| Host ATTN timeout `T_ATTN` | `GD32G553_BRIDGE_REPLY_TIMEOUT_MS` (10 ms) |
| CS setup (assert → first SCK) | Unchanged from today. The slave needs about 3 µs (provisional). |
| Reply-read CS rising → next request CS falling | ≥ 10 µs (provisional). A request falling edge that coalesces with the drain handler re-arms RX over bytes already captured. |

### 4.5 Host procedure

**Reply path:**
1. Clock the request.
2. Clear the P71 edge latch as the last action before releasing CS.
3. Release CS.
4. Wait (IRQ plus semaphore; no polling) for a rising edge, up to `T_ATTN`.
5. Clock the reply.
6. Validate with STATUS_SEQ (§4.7).

**Self-test at enable:** the `LINK_FEATURES` reply that grants ATTN must arrive on an edge. If it does not, immediately re-send the extended form with ATTN cleared, and treat ATTN as unusable until the next `gd32g553_init()`.

**Event path:** on an edge while idle, issue `READ2` for every stream with a watermark armed. Use one `CMD_BATCH` when two streams are armed.

### 4.6 Lost or stuck ATTN
- **Lost** (no edge within `T_ATTN`): continue that command with the 0.14 drain rule (25 µs → 1.6 ms re-read ladder) plus STATUS_SEQ. After 3 consecutive timeouts, stop waiting on ATTN and re-negotiate without the ATTN bit, which also returns PA14 to SWCLK.
- **Stuck high:** a violation is ATTN reading high when the host releases CS on a request whose CS-low time was ≥ 20 µs. After 3 consecutive violations, or 8 consecutive idle-path edges where every armed stream returns `got = 0`, `dropped = 0`, take the same action as for lost ATTN.

### 4.7 Interaction with STATUS_SEQ
- ATTN is a hint; the stamp decides.
- ATTN is only granted together with STATUS_SEQ.
- A stale stamp under ATTN triggers the same single re-send as `chips/gd32g553/gd32g553.c:240-250`. Example: the request was lost whole, the rising handler took the empty-drain path, and pending events raised ATTN.
- The RESET signature (stamp 0 after a non-zero baseline) means: drop all negotiated state including ATTN, keep `P71` as input, and re-run §8.

## 5. `CMD_ADC_STREAM_BEGIN2` (`0x3B`) / `CMD_ADC_STREAM_READ2` (`0x3C`)

Both opcodes are SPI-only and need ADC_STREAM2 granted on the link; otherwise they answer `STATUS_NOSUPPORT`.

### 5.1 BEGIN2 request (12 B)

| Off | Field | Type | Rules |
|---|---|---|---|
| 0 | `stream_id` | u8 | 0..1, else INVAL |
| 1 | `channel` | u8 | 0..7. An unmapped channel → OUT_OF_RANGE (same as legacy). |
| 2 | `trigger_src` | u8 | `0x00` PACE_TIMER: the firmware-programmed "software timer", TIMER5 for stream 0 and TIMER6 for stream 1 (today's behaviour). Reserved: `0x01` TIMER0_CC, `0x02` TIMER7_CC (arg = CC channel 0..3); `0x03` TIMER5_TRGO shared, `0x04` TIMER6_TRGO shared (simultaneous two-stream pacing); `0x05` EXTI (arg = line 0..15). `0x01..0x05` → NOSUPPORT; `≥0x06` → INVAL. |
| 3 | `trigger_arg` | u8 | must be 0 for PACE_TIMER, else INVAL |
| 4 | `sample_rate_hz` | u32 | PACE_TIMER: 1..100000. 0 → INVAL; >100000 → OUT_OF_RANGE. |
| 8 | `watermark` | u16 | one of {0, 16, 32, 64, 128, 256, 512}, else INVAL. 0 = no events. This is the *requested* watermark: the firmware may grant a larger one (§5.2). |
| 10 | `reserved` | u16 | must be 0, else INVAL |

### 5.2 BEGIN2 reply (17 B)

| Off | Field | Type | Meaning |
|---|---|---|---|
| 0 | `tick_hz` | u32 | pace-timer tick: 1000000 when rate ≥ 16, otherwise 10000; 0 for non-timer triggers |
| 4 | `period_ticks` | u32 | `floor(tick_hz / sample_rate_hz)`, as programmed in `hal/gd32/adc_stream.c:390-396`. **Realised rate = `tick_hz / period_ticks` exactly** (for example 300 Hz → 3333 ticks = 300.03 Hz). |
| 8 | `full_scale` | u16 | `(1 << res_bits) − 1` captured at BEGIN2 (4095/1023/255/63); oversampling does not change it |
| 10 | `vref_mv` | u16 | `adc_vref_mv` captured at BEGIN2 |
| 12 | `flags` | u8 | bit0 `VREF_MEASURED` = `adc_vrefint_code ≠ 0` **and** the derived value lies in [1700, 1900] mV, i.e. it is not the `ADC_VREF_MV` 1800 fallback. Bits 1..7 = 0. |
| 13 | `watermark` | u16 | **granted** watermark = `ring_depth / 2` (≥ the requested one; 0 stays 0). The host must use this value, not the one it asked for. |
| 15 | `ring_depth` | u16 | the smallest power of two ≥ max(`2 × watermark`, samples produced in 5 ms at the realised rate), capped at 1024; 1024 when watermark = 0. This is the overrun budget. |

BEGIN2 rules beyond the legacy BEGIN checks (vref dead → IO, slot in use or shared converter → INVAL, converter claimed → BUSY, DMA/calibration failure → IO):
- **Conversion-time check.** Answer `STATUS_OUT_OF_RANGE` if `ratio × (sample_cycles + 12.5) / 36 MHz ≥ period_ticks / tick_hz`. This is the `ADC_READ` residency model from `hal/gd32/gd32_common.h`. Compute it in 64-bit integer half-cycles using the channel's cached `ADC_CONFIGURE` values. Unlike legacy BEGIN, which silently degrades, BEGIN2 never reports a rate it cannot achieve.
- `bridge_core_clock_matches == false` → `STATUS_IO` (the realised rate would be wrong).
- A late VREF re-measure is pending (`vref.c`) → `STATUS_NOT_READY`. Retry after ≥ 50 ms, the SysTick housekeeping period. This keeps the `vref_mv` snapshot from being replaced while the stream runs.
- A stream started with BEGIN2 answers only READ2: legacy `0x34` on it → INVAL. Likewise READ2 on a legacy-started stream → INVAL. `0x35` STREAM_END ends either kind. `0x39` CHAIN_BIND and `0x3A` SPECTRUM_READ behave as in 0.14.

### 5.3 READ2

Request (2 B): `stream_id:u8`, `max_samples:u8`.
- `max_samples` must be 1..`READ2_MAX(L) = floor((mp − 9)/2)`: 28 at mp = 65, 121 at mp = 252.
- 0 → INVAL; above the ceiling → OUT_OF_RANGE.

Reply (variable, `9 + 2·got` B):

| Off | Field | Type |
|---|---|---|
| 0 | `first_index` | u32: index of `codes[0]` in this stream's sample sequence since BEGIN2; wraps mod 2^32 |
| 4 | `dropped` | u32: samples discarded immediately before `codes[0]` since the previous READ2 reply |
| 8 | `got` | u8: 0..`max_samples` |
| 9 | `codes[got]` | u16 LE each: right-aligned raw ADC codes, clamped to `full_scale` |

Framing:
- The CRC follows the last code directly. In the SPI frame it sits at offset `11 + 2·got` (`SOF` @0, `STATUS` @1, `got` @10).
- The host clocks `13 + 2·max_samples` bytes, reads `got`, and ignores bytes after the CRC (TX-underrun filler).
- The host checks STATUS first. A non-OK STATUS means the 4-byte error envelope, with the CRC at offset 2.
- `got > max_samples` → host returns IO.
- A pure function of the stream state; CRC position is derived from `got`.

### 5.4 READ2 accounting (normative)
- Each stream keeps a delivered index `D` (u32), 0 at BEGIN2.
- Backlog = (`total_written − total_read`) mod 2^32. Ring index arithmetic uses the stream's own `ring_depth` (a power of two, so mod-2^32 wraparound stays exact), replacing every `BRIDGE_ADC_STREAM_RING_SAMPLES` in the raw-ring index maths.
- **Lap safety (ring sizing).** The raw ring's lap count comes from a prio-3 ISR counting each DMA reload. Prio-1/2 work (ROVF recalibration ≈ 2 ms, `ADC_READ` ≤ 1 ms, a ≈ 350 µs BATCH, I2C OTA programming) can keep it off the CPU long enough that a reload is counted once instead of twice; a lost lap would make `READ2` report `dropped = 0` with a consistent `first_index` over wrong data. So BEGIN2 sizes `ring_depth` so that one lap lasts at least 5 ms (`ADC_STREAM2_LAP_MIN_US`, twice the ROVF figure) at the *realised* rate: `ring_depth = pow2ceil(max(2W, ceil(realised_rate × 5 ms)))`. At 100 kHz that is 500 samples, so `W = 16` yields `ring_depth = 512` and a granted watermark of 256; at 1 kHz `W = 16` stays at ring 32 and watermark 16. The events fire at HTF/FTF = `ring_depth / 2`, which is the granted watermark. `W = 0` uses the whole 1024 ring (≥ 10.24 ms at 100 kHz) and raises no events. If even 1024 samples cannot span 5 ms (rates above 204.8 kHz, unreachable since the maximum is 100 kHz) BEGIN2 answers `OUT_OF_RANGE`.
- **Transient undercount.** A backlog at most one ring "below zero" is the single window in which one CS-EXTI handler's lap snapshot and write-index read straddle a reload; it plans one empty read. A second consecutive undercount, or any backlog further below zero, answers the `0xFFFFFFFF` sentinel and resyncs the read cursor to the live total.
- **Overrun:** if backlog > `ring_depth − GUARD` (GUARD = 8), then:
  - `skip = backlog − (ring_depth − GUARD)`;
  - advance the read cursor by `skip`;
  - `dropped += skip`;
  - answer `STATUS_OK` with the freshest contiguous samples.
  
  **READ2 never answers BUSY for overrun.**
- Then `first_index = D_prev + dropped`, `got = min(max_samples, remaining backlog)`, and `D = first_index + got`.
- Host invariant: `first_index(n) == first_index(n−1) + got(n−1) + dropped(n)` (mod 2^32), except when `dropped` is the sentinel.
- `dropped` saturates at `0xFFFFFFFE`. `0xFFFFFFFF` means a **discontinuity of unknown length**, with `got = 0` and `first_index = D_prev`. Causes:
  - ROVF recovery (the existing 9-step `adc_stream_recover_rovf()`; a recalibration failure answers `STATUS_IO` instead);
  - a DSP pump `proc_gap`;
  - backlog > 2^31.
- Check the reply capacity **before** consuming the ring: `9 + 2·max_samples > cap` → NOMEM, nothing consumed. (The legacy handler consumes first; see `src/protocol.c:586` vs `:598`.)
- FIR/IIR-bound stream: READ2 returns processed-ring codes in the same code space. FFT-bound stream: `STATUS_NOSUPPORT`. Sticky DSP faults keep their 0.14 mapping: `dsp_cfg_bad` → OUT_OF_RANGE, `dsp_sat` → IO.
- Watermark events come from HTF (W samples) and FTF (2W) on a raw ring of length `2W`, or from the base-level pump when the processed backlog reaches W. Size W with `W ≥ rate × host round-trip`; the slack before overrun is `W − GUARD` sample periods.
- Host conversion to mV is `(min(code, full_scale) × vref_mv) / full_scale` with integer truncation. This is bit-identical to the legacy `STREAM_READ` maths (`hal/gd32/adc_stream.c:606`).

### 5.5 Status summary

| | BEGIN2 | READ2 |
|---|---|---|
| NOSUPPORT | feature not granted; reserved trigger; stub HAL | feature not granted; FFT-bound; stub |
| INVAL | length; stream_id; slot in use / shared converter; trigger ≥ `0x06`; arg / reserved ≠ 0; rate 0; watermark | length; stream_id; stream inactive or started with legacy BEGIN |
| OUT_OF_RANGE | rate > 100000; unmapped channel; conversion time ≥ period | `max_samples` > ceiling; `dsp_cfg_bad` |
| NOT_READY | VREF re-measure pending | n/a |
| BUSY | converter claimed (#133) | n/a (dispatcher-level BUSY only) |
| IO | vref dead; DMA disable or calibration failure; clock mismatch | DMA ERRIF; ROVF recalibration failure; `dsp_sat` |
| NOMEM | n/a | reply exceeds capacity (nothing consumed) |

## 6. `CMD_BATCH` (`0x04`)

SPI-only; requires BATCH granted; returns NOSUPPORT otherwise.

### 6.1 Request
`count:u8` (1..16), followed by `count` entries `{op:u8, len:u8, args[len]}`.

`req_payload_len` must equal `1 + Σ(2 + len_i)` exactly. Any trailing byte → INVAL. This is the defence against zero-extended captures.

### 6.2 Validation pass (no side effects; batch-level error = empty-payload envelope, nothing executes)

| Condition | Status |
|---|---|
| `count == 0`, length mismatch, op not on the allow-list (includes nested `0x04`, `0x81`, `0x28`, any `0xF0..0xFF`), `len_i` ≠ the op's fixed request length, or a second `0x3C` READ2 on the same `stream_id` (repeated reads inside one CS ISR see a stale lap count) | INVAL |
| `count > 16`, or worst-case reply `1 + Σ(2 + maxreply_i) > mp` | OUT_OF_RANGE |

### 6.3 Allow-list (bounded, side-effect-local handlers)

| Op | Request len | Max reply |
|---|---|---|
| `0x00` PING | 0 | 0 |
| `0x10` GPIO_READ | 4 | 4 |
| `0x11` GPIO_WRITE | 8 | 0 |
| `0x20` PWM_SET | 10 | 0 |
| `0x21` PWM_GET | 1 | 8 |
| `0x24` PWM_CAPTURE_READ | 1 | 8 |
| `0x3C` ADC_STREAM_READ2 | 2 | 9 + 2·`max_samples` |
| `0x40` DA9292_STATUS_FORWARD | 0 | 1 |
| `0x50` DAC_SET | 4 | 0 |
| `0x51` DAC_GET | 1 | 2 |
| `0x60` QENC_READ | 1 | 4 |
| `0x61` QENC_RESET | 1 | 0 |
| `0x70` COUNTER_READ | 1 | 4 |
| `0x90` TMU_COMPUTE | 12 | 4 |

Excluded, with reasons:
- `ADC_READ`: up to 1 ms each.
- stream BEGIN/END variants: calibration can spin for up to about 200000 iterations.
- `TRNG_READ`: budgeted DRDY polls.
- `SE_RESET`: needs host-timed gaps between steps.
- DSP chain opcodes: setup-time only.
- `LINK_FEATURES`: would change framing in the middle of a reply.
- `POWER_MODE_SET`, OTA, nested BATCH.

### 6.4 Execution
- Ops run in order through the same handlers as standalone ops, via `protocol_dispatch_inner()` with the in-flight flag already held. Each gets `reply_cap = maxreply_i`.
- Execution **stops at the first non-OK** sub-status.
- **Residency budget:** at most 16 ops × at most 20 µs worst case per allow-listed handler (each to be measured with `timing_stats` before the list is frozen), plus framing ≤ 30 µs, gives ≤ about 350 µs in the CS-EXTI handler. That is below `ADC_READ_ISR_BUDGET_US` (1000). I2C0 (prio 2) and base level stall for that window, and BRD_I2C may be clock-stretched by up to about 350 µs.

### 6.5 Reply
Outer STATUS is `OK` whenever the batch passed validation. It carries the one STATUS_SEQ stamp; sub-statuses are never stamped.

Payload: `executed:u8` (number of ops attempted, including a failing one), then per executed op `{status:u8, len:u8, payload[len]}`. A non-OK sub-status always has `len = 0`.

The length is self-delimiting, and its maximum can be computed from the request. The host clocks the worst case, parses entries to find the CRC, and returns IO if:
- `executed > count`, or
- `len_i > maxreply_i`, or
- the length of an op with a fixed reply ≠ its expected value.

## 7. I2C link policy (unconditional)

Enforced in `protocol_dispatch_inner()` **after** the trial gate (unchanged: any CRC-valid frame on either link confirms a trial) and before the opcode switch.

- **Allowed on `GD32_BRIDGE_LINK_I2C`:** `0x00` PING, `0x01` GET_VERSION, `0x02` GET_BUILD_ID, `0x03` RESET_REASON, `0x10` GPIO_READ, `0x11` GPIO_WRITE, `0x41` SE_RESET, `0x42` BOOT_CONFIG, `0x81` LINK_FEATURES (I2C grants only STATUS_SEQ, `mp` = 65), and `0xF0..0xFF` OTA.
- Any other opcode: `STATUS_NOSUPPORT` (`0x06`) with an empty payload; the handler never runs.
- Add SWD-readable diagnostics `bridge_i2c_denied_count:u32` and `bridge_i2c_denied_last_cmd:u8`, in the same style as `bridge_i2c_rx_diag`.
- SPI is unrestricted. No per-pad GPIO ownership is enforced (§11 Q9).

## 8. Discovery order and host fallback matrix

`gd32g553_init()` steps:
1. `PING`, using the existing BUSY-gated retry ladder.
2. `GET_VERSION`: major must equal 0 (`chips/gd32g553/gd32g553.c:574-576`); record `m` = minor.
3. If SPI is open:
   - (a) If `m ≥ 15`: send the 6-byte `LINK_FEATURES` with `want` = STATUS_SEQ|BIG_FRAME|ADC_STREAM2|BATCH, plus ATTN only if the backend registered an ATTN hook with `P71` as input + IRQ, and `max_payload_req = 252`. Store `granted`/`mp`; take the stamp baseline from the reply; run the ATTN self-test (§4.5). If the reply is INVAL or NOSUPPORT, go to (b).
   - (b) If `m < 15`: send the 1-byte STATUS_SEQ form, as today.
4. If I2C is open: nothing is required. Optionally, when `m ≥ 15`, send the 6-byte form on I2C to read `supported`; expect `0x00000001`, `mp` 65.
5. On the RESET signature or any reply showing an OTA COMMIT/ROLLBACK reset: drop every negotiated item (sequence, BIG, ATTN, STREAM2, BATCH, stream handles) and repeat from step 1.

| Host | Firmware | Result |
|---|---|---|
| 0.14 | 0.14 | Unchanged. |
| 0.14 | 0.15 | 1-byte negotiation, so the wire is byte-identical to 0.14 on SPI. On I2C, opcodes outside §7 now answer NOSUPPORT; no in-tree caller is affected. |
| 0.15 | 0.14 (or older) | Legacy 1-byte STATUS_SEQ; legacy `0x33`/`0x34` (mV, BUSY on overrun, report `dropped` as unknown); no BATCH; 65-B frames; `P71` may stay input or SWD; ATTN never driven. |
| 0.15 | 0.15, ATTN not granted (not requested, debugger attached, stub, self-test failed) | BIG + STREAM2 + BATCH. Reply timing uses the 0.14 staging gap + ladder + STATUS_SEQ. Reads are scheduled by a host timer at `W / realised_rate`, not by busy-polling. |
| 0.15 | 0.15, ATTN granted | Everything. Per-command fallback to the drain rule on timeout; disable after 3 consecutive faults (§4.6). |

`docs/gd32-bridge-protocol.md` §8's "pre-1.0 lockstep including minor" paragraph must be changed: mixing 0.14 and 0.15 is supported, and 0.15 features are gated on negotiation, not on minor alone.

## 9. Test vectors to add to `tests/protocol_vectors.txt`

Hex for all of these must come from `tests/gen_protocol_vectors.py`; none of the CRCs below were computed by hand. The fields up to the CRC are listed.

Unless noted, `STATUS` is shown un-stamped (`0x00`), with a stamped variant where named.

| Name | Bytes before CRC |
|---|---|
| `spi_get_version_reply_v0_15_0` | `A5 00 00 0F 00` |
| `spi_link_features_ext_request_all` | `A5 81 1F000000 FC00` |
| `spi_link_features_ext_reply_all_seq1` | `A5 10 1F000000 1F000000 FC00` |
| `spi_link_features_ext_reply_attn_refused_seq1` | `A5 10 1B000000 1F000000 FC00` (debugger attached) |
| `spi_link_features_ext_reply_stub_seq1` | `A5 10 13000000 13000000 FC00` |
| `spi_reply_inval` | `A5 01`. Also the 0.14 answer to the 6-byte form. |
| `i2c_link_features_ext_write` | `00 81 1F000000 FC00` (CRC over `81..`) |
| `i2c_link_features_ext_read` | `00 01000000 01000000 4100` |
| `i2c_adc_read_ch0_4_write_denied` | `00 30 00 04`, read `06` |
| `i2c_ota_get_state_write_allowed` | `00 F5` |
| `spi_adc_stream_begin2_s0_ch0_1khz_w256_request` | `A5 3B 00 00 00 00 E8030000 0001 0000` |
| `spi_adc_stream_begin2_reply_1khz_w256` | `A5 00 40420F00 E8030000 FF0F 0807 01 0001 0002` |
| `spi_adc_stream_begin2_reply_300hz_truncation` | `A5 00 40420F00 050D0000 FF0F 0807 00 0000 0004` (VREF fallback, W = 0) |
| `spi_adc_stream_read2_s0_max121_request` | `A5 3C 00 79` |
| `spi_adc_stream_read2_reply_got3` | `A5 00 00010000 00000000 03 0008 0108 FF0F` |
| `spi_adc_stream_read2_reply_empty` | `A5 00 03010000 00000000 00` |
| `spi_adc_stream_read2_reply_overrun_dropped32` | `A5 00 23010000 20000000 02 0008 0108` |
| `spi_adc_stream_read2_reply_discontinuity` | `A5 00 25010000 FFFFFFFF 00` |
| `spi_batch_request_gpiow_pwmget_read2` | `A5 04 03 11 08 01000000 01000000 21 01 00 3C 02 00 10` |
| `spi_batch_reply_stop_at_first_error` | `A5 00 02 00 00 01 00` (request: GPIO_WRITE, READ2 on inactive stream 1, PING) |
| `spi_batch_request_nested_rejected` | `A5 04 01 04 00`, reply = `spi_reply_inval` |
| `spi_batch_request_trailing_byte_rejected` | `A5 04 01 00 00 00`, reply = `spi_reply_inval` |
| `spi_ota_write_chunk_over_65_rejected_on_big_link` | `A5 F1` + 66-B payload, reply = `spi_reply_inval` |

Unit-test consumers to extend:
- `tests/unit/protocol_vectors`
- `tests/unit/protocol`: 1-byte form clears the 0.15 bits; grant algorithm; I2C allow-list and denial counter
- `tests/unit/transport_spi`: 256-B frames; >65 enforcement; fresh-vs-drain signal
- `tests/unit/transport_hw_cs_exti`: ATTN drive points
- a new host-testable READ2 accounting suite: overrun skip arithmetic, sentinel, u32 wrap, GUARD

Each new regression check must be shown to fail against the 0.14 build first.

## 10. Files to change

**Firmware:**
- `src/protocol.h`: MINOR 15 plus version note; new opcodes, feature bits and size constants; per-link feature word widened to u32; fix the stale `GD32_BRIDGE_ADC_STREAM_RING_SAMPLES 128u` comment (the real ring is `BRIDGE_ADC_STREAM_RING_SAMPLES 1024u`, `hal/gd32/gd32_common.h:85`).
- `src/protocol.c`: 6-byte `handle_link_features`; per-link `mp`; I2C allow-list; feature gating of `0x04`/`0x3B`/`0x3C`; BEGIN2/READ2/BATCH handlers.
- `src/transport_spi.c`, `src/transport.h`: 256-B buffers; static reply scratch; >65 enforcement; a "fresh stage" result from `spi_slave_cs_high()`.
- `hal/transport_hw_gd32.c`: `BRIDGE_SPI_DMA_BUF_LEN 260u`; ATTN enable/disable and drive points; event-assert helper with the PA8 / `EXTI_PD0` bit-8 gate.
- `hal/bridge_board_config.h`: ATTN = `GPIOA`/`GPIO_PIN_14`; note forbidding PA14 in the lock mask.
- `hal/bridge_hw.h`, `hal/bridge_hw_stub.c`: BEGIN2/READ2/ATTN HAL entry points (stub: NOTIMPL, ATTN unsupported).
- `hal/gd32/adc_stream.c`, `hal/gd32/gd32_common.h`: per-stream `ring_depth`, HTF events, `D`/`dropped` accounting, realised rate, conversion-time check, snapshot fields.
- `hal/gd32/vref.c`, `hal/gd32/adc.c`: expose `VREF_MEASURED` and remeasure-pending.
- `src/ota.c`: static assert only.
- `tests/gen_protocol_vectors.py`, `tests/protocol_vectors.txt`, `tests/unit/**`; firmware CHANGELOG / README.

**alp-sdk:**
- `docs/gd32-bridge-protocol.md`: §3 table, §3.10, §3.14, new §3.16 BATCH / §3.17 ATTN / §3.18 STREAM2, §4 variable-length CRC position, §4.1 ATTN timing, §5 I2C policy, §6 note on I2C NOSUPPORT, §8 version history and lockstep text, §9.
- `include/alp/chips/gd32g553.h`: new opcodes, bits and constants; context fields (granted, `mp`, ATTN state and counters); new API for `begin2`/`read2`/`batch`; ATTN hook registration. The driver stays Zephyr-agnostic: the backend supplies a wait-for-edge hook built on `alp_gpio_irq_enable()` plus its own semaphore.
- `chips/gd32g553/gd32g553.c`: §8 negotiation; variable-length reply parsing; ATTN wait replacing the staging gap; fallback counters. `GD32G553_MAX_PAYLOAD_BYTES` stays 65 (used for the OTA limit at `:1348` and the I2C buffers). 256-B SPI buffers move off the caller stack into the context, or the CM33 thread stacks must be checked.
- `src/zephyr/v2n_supervisor.c` + board DT: `P71` as input + IRQ, ATTN hook.
- `src/zephyr/peripheral_adc.c`: use BEGIN2/READ2 when granted.
- `metadata/chips/gd32g553.yaml`: ATTN / PA14 dual-role note.
- `tests/zephyr/chips/src/test_gd32_bridge.c` + I2C emulator; `examples/v2n/v2n-gd32-bridge-functional`, `examples/v2n/v2n-gd32-bridge-hil-soak`; `changelog.d/`.
- **No change needed:** the kernel patch `0005-gpio-add-gd32-bridge-expander-driver.patch` and `tools/gd32-ota-host`.

## 11. Open questions
1. **OTA ownership.** The maintainer's note says "CM33/SPI owns OTA", but `tools/gd32-ota-host` is I2C-only. This spec keeps OTA reachable on I2C (no lockstep). Confirm this, or plan a migration of the tool before tightening §7.
2. **P71 as an interrupt and its owner.** Can `P71` raise a GPIO interrupt (TINT) routed to the CM33? Which side owns its pinctrl (A55 DT vs CM33)? Is its default high-impedance in every boot stage (H1)?
3. **Contention on the SWCLK net.** Is there a series resistor? Confirm PA14's reset state (AF0, pull-down) against UM Rev1.2 before F3 is final.
4. **ATTN timing numbers.** All of §4.4 is provisional and needs bench measurement.
5. **Watermark mechanism.** Ring = 2W shrinks the overrun budget at small W. Is a timer that counts samples per stream (keeping the 1024-deep ring) feasible on spare timers? TIMER0/7 drive PWM and TIMER1..4 drive QENC.
6. **Discontinuity signal.** `dropped = 0xFFFFFFFF` was chosen to keep the requested layout. A 1-byte `flags` field would be cleaner if the maintainer accepts a 10-B header.
7. **Conversion-time model.** `src/protocol.h:144-146` says sample time = RSMP + 2.5 cycles; `hal/gd32/gd32_common.h` models `sample_cycles + 12.5`. Agree on one model before the BEGIN2 check is frozen.
8. **BATCH per-op cost.** The 20 µs per-op ceiling must be measured with `timing_stats` for each allow-listed op.
9. **GPIO pad ownership.** Pad-level ownership between Linux (I2C) and the CM33 (SPI) is not enforced.
10. **`C_DEBUGEN` after detach.** If it stays set after a probe disconnects, ATTN is refused until reset. Is that acceptable as fail-safe?
11. **Portable API.** How should overrun-with-data (`dropped > 0`) surface in `<alp/adc.h>`, which returns `ALP_ERR_BUSY` today?

## 12. Maintainer decisions (2026-10-04) resolving §11

- Q1 OTA: stays reachable on I2C (Linux `tools/gd32-ota-host`); SWD is for factory flashing and recovery. §7 allow-list keeps `0xF0..0xFF`.
- Q2 P71: TINT-capable (ICU TSSEL `0x31`, inferred from the RZ/V2N TINT table; bench-verify), no core owns P70/P71 today and Linux has no claim. The CM33 owns P71 as input + IRQ.
- Q3/Q4/Q8/Q10: bench items. Implement per spec; the timing numbers stay provisional constants in one place.
- Q5 watermark: ring = 2W as the floor, grown to the smallest power of two spanning a 5 ms lap at the realised rate (lap-safety, §5.4); the granted watermark (`ring_depth / 2`) is echoed in the BEGIN2 reply and may exceed the requested one. Revisit only if bench shows the overrun budget is too small.
- Q6 discontinuity: keep the `dropped = 0xFFFFFFFF` sentinel (9-byte header).
- Q7 conversion model: use the code's model in `hal/gd32/gd32_common.h` (`sample_cycles + 12.5`); fix the `src/protocol.h` comment to match.
- Q9: no per-pad ownership enforcement in 0.15.
- Q11 portable API: `<alp/adc.h>` is unchanged in this step; the chip API (`gd32g553_adc_stream_read2`) reports `first_index` / `dropped`. The V2N ADC backend uses READ2 when granted and still returns `ALP_ERR_BUSY` only on the discontinuity sentinel.
- Host SPI engine: DMA only; no interim FIFO-interrupt engine. ATTN replaces the staging-gap busy-wait when granted (IRQ + semaphore, no polling).


## 13. `CMD_BOOT_CONFIG` (0x42): persistent opt-in boot behaviour (alp-sdk #2697)

Request `op:u8 flags:u32 LE` (5 B); reply `flags:u32 LE` (the STORED value; on a SET, the value before the commit). `op` 0 = GET (`flags` ignored), 1 = SET. Allowed on I2C and SPI (provisioning runs from Linux). Added inside the unreleased 0.15 line, so no `PROTOCOL_VERSION` bump; a host sees `STATUS_NOSUPPORT` from firmware that predates it.

- `flags` bit0 `SDMUX_EN_HIGH`: from `bridge_hw_init()` the GD32 drives PD11 (E1M IO29, the EVK's `SDIO_MUX_EN`, active-low: low = SD connected, high = SD disconnected) HIGH, so a provisioning run keeps the microSD out across cold power cycles. Other bits -> `STATUS_INVAL`.
- **Default off.** IO29's meaning is carrier-specific, so a unit that never received a SET behaves as before: the pad is not touched (hi-Z/analog park). The flag is host-set, never a firmware default.
- **Storage (power-loss safe):** two A/B record pages, `OTA_CONFIG_REC0` (0x08076000) and `OTA_CONFIG_REC1` (0x08076800), outside every image's link range (`src/ota_layout.h`; linker ASSERTs in `toolchain/gd32g553_flash.ld` and `gd32g553_app_slot.ld.in`). The record is 16 B, two flash doublewords: payload (`counter`, `flags`), then the commit doubleword (`crc32` over the payload, `magic "BCF2"`), programmed last. It is the same A/B-with-counter pattern as the OTA metadata (`ota_meta_record_t`); the metadata record itself cannot carry the flag because the deployed bootloader rejects any other layout. A SET erases and programs only the page that does NOT hold the newest valid record, so the old value stays intact until the new commit doubleword lands; the highest valid counter wins; an erased, torn, foreign or CRC-bad page is "absent"; no valid page = all flags off. The pages survive power cycles and OTA slot swaps.
- **ECC-fault-safe read.** This flash is ECC: a doubleword torn by a power cut can be uncorrectable, and reading it raises the flash-ECC NMI (gh#36, a reset). Every read goes through `ota_fmc_read_safe()` (`hal/fmc_ota.c`): it masks the single NMI source (`syscfg_interrupt_disable(SYSCFG_INT_FLASHECC)`), clears `FMC_ECCCS.ECCDET0/ECCDET1` (`fmc_ecc_flag_clear`), does the aligned loads, tests the flags (`fmc_ecc_flag_get`) and reports "uncorrectable" instead of faulting; the syscfg flag is cleared before the NMI enable is restored. Source: GD32G5x3 User Manual Rev1.2 FMC ECC section (p.93-94: a double-bit error sets ECCDET0 and generates an NMI; Note 4: clearing ECCDET0 also clears `SYSCFG_STAT.FLASHECCIF`) and the vendor library API above (GigaDevice firmware library V1.5.0, `gd32g5x3_fmc.c`). The commit doubleword is read first and the payload only once the magic is present. Torn page = absent, so a cut at any step of a SET boots with the old or the new value and never faults. **Unproven on silicon:** that an NMI-masked read of a real uncorrectable doubleword completes with the flag set rather than a bus fault; the bench plan is to tear a record on purpose (cut power mid-SET, 100+ cycles) and confirm the unit boots with the old value and `FMC_ECCCS` is clean.
- **Asynchronous SET.** The transport ISR never touches flash. A SET is queued and answered at once with the CURRENT stored value; `bridge_hw_tick()` calls `boot_config_tick()` which commits it at base level (page erase <= 20 ms with interrupts masked for the busy window, two programs, readback). The host leaves the link idle for ~30 ms and polls GET until the stored value equals the request (`gd32g553_boot_config_set`). A SET equal to the stored value replies OK without queueing or touching flash (the transport is at-least-once); a different SET while one is queued answers `STATUS_BUSY`; the tick waits while an OTA erase walk owns the FMC funnel (#266). If the commit fails the GET poll never matches and the host re-sends. `STATUS_NOSUPPORT` when the build has no FMC HAL, when `FMC_OBCTL.DBS` = 0 (the full-flash build runs the erase from flash, which is only safe in dual-bank mode with the whole image in bank 0; the linker asserts the image ends below 0x08040000), or when the running image is in bank 1 (slot B: a bank-1 erase from bank-1 code is not bench-validated; set the flag while running slot A).
- **SET never moves a pad.** It takes effect at the next GD32 reset, so setting it on a unit running from the SD cannot pull its rootfs out. A host that wants the SD out now also writes IO29 high with `GPIO_WRITE`; clearing the flag does not release an already-driven pad (that needs a GD32 reset).
- **Boot timing (estimate, not measured):** the pad is driven right after the pad map is parked in `bridge_hw_init()`, i.e. after the bootloader's slot check (~72 ms at -Os for a full-slot CRC, `src/boot/boot_main.c`) and the pre-`main()` SRAM init (~20-25 ms, `hal/gd32/se_reset.c`) plus a few ms of clock/NVIC/SYSCFG setup: roughly 100-150 ms after GD32 reset release. RZ/V2N U-Boot scans mmc1 seconds after power-up (TF-A + DDR training + U-Boot SPL first), so the margin is one order of magnitude or more. Bench item: scope PD11 vs the SoM supply on 2026W38-0008 to confirm.
- `GPIO_READ` is unaffected by this feature and still must not reconfigure a pad (alp-sdk #2701).
