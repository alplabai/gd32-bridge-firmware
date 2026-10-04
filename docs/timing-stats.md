# SPI round-trip timing stats (opt-in, issue #104)

Configure with `-DBRIDGE_TIMING_STATS=ON` (default OFF; OFF adds no code or
data, the image is byte-identical to a build without the feature).  The DWT
cycle counter is enabled by `bridge_hw_init()` and sampled around one SPI transaction.
The results live in the RAM symbol `bridge_timing_stats`, read over SWD.

Only transactions that pass the request CRC and reach `protocol_dispatch()` are
recorded (drains, malformed and CRC-failing frames are not).  I2C is not
instrumented.  Reset the measurement by writing zeros over offsets `0x08..0x4B` only (68
bytes; keep `magic`/`layout` so a later read still proves the struct is
initialised).  `count == 0` means no sample; `timing_stats_init()` clears the
whole struct and stamps the header at boot.

`CMD_ADC_READ` on the SPI link is a **deferred** command: `protocol_dispatch()`
only arms the conversion burst and returns, and the burst's DMA-complete
interrupt stages the reply later.  Such a transaction is therefore **not
recorded** (no `reply_stage`/`total` sample is taken, because there is no staged
reply at the end of the CS-release branch), and the conversion and staging time
is not in any stat.  For the interval from the request's CS release to the reply
being armed, measure on the wire instead.  Every other opcode is unchanged.

## Layout (`bridge_timing_stats_t`, 76 bytes, little-endian, no padding)

| Offset | Field | Meaning |
|--------|-------|---------|
| 0x00 | `magic` | `0x41545354` once initialised |
| 0x04 | `layout` | `1` |
| 0x08 | `req_crc` | cycles for the request CRC-16 check |
| 0x18 | `dispatch` | cycles in `protocol_dispatch()` |
| 0x28 | `reply_stage` | cycles in `stage_reply()` (copy + reply CRC) |
| 0x38 | `total` | cycles from CS-release branch entry (in the CS EXTI handler) to reply staged |
| 0x48 | `cmd_last` | opcode of the most recent recorded request |

Each stat (`timing_stat_t`, 16 bytes) is four u32: `+0 count`, `+4 last`,
`+8 min`, `+12 max`, all in DWT cycles.  Cycles / `SystemCoreClock` (216 MHz)
gives seconds: 216 cycles = 1 us.

`total` starts at the CS-release branch of the CS EXTI handler (after the EXTI
pending and NSS reads; exception latency is not counted) (`hal/transport_hw_gd32.c`), so it includes DMA quiesce, the SPI
reset/re-init and the byte copy, and ends when `stage_reply()` returns.  The
short loop that copies the reply into the TX DMA buffer and arms TX (<= 69 B)
follows and is not included.  The 32-bit counter wraps every ~19.9 s; deltas are
wrap-safe for any single stage.

## Bench frames (SPI, CRC-16/CCITT-FALSE, lo-byte first on the wire)

Request frame is `SOF(A5) | CMD | PAYLOAD | CRC_lo CRC_hi`; the reply is read on
the next CS assertion.  Both opcodes below leave flash untouched.

| Purpose | Request frame (hex) | Reply frame |
|---------|---------------------|-------------|
| `GET_VERSION` (0x01) | `A5 01 A5 EF` | `A5 00 <maj min patch> CRC` |
| `ADC_STREAM_BEGIN` (0x33): stream 0, channel 0, pad 0, 10 Hz | `A5 33 00 00 00 0A 00 00 00 F2 91` | `A5 00 CRC` |
| `ADC_STREAM_READ` (0x34): stream 0, 32 samples (max reply) | `A5 34 00 20 F4 EF` | `A5 00 <got> <32 x u16 LE mV> CRC` = 69 B |
| `ADC_STREAM_END` (0x35): stream 0 | `A5 35 00 90 60` | `A5 00 CRC` |

`ADC_STREAM_READ` with 32 samples is the largest reply the bridge produces
(`GD32_BRIDGE_MAX_PAYLOAD_BYTES` = 65), so it bounds `reply_stage`.  It needs a
begun stream (an un-begun stream answers `STATUS_INVAL` with no payload, which
would measure an early-out), hence the BEGIN first and END last.  BEGIN at 10 Hz on purpose: the ring holds
1024 samples, so it covers about 100 s at 10 Hz but only about 1 s at 1000 Hz, and
a longer gap between reads would overrun it; an overrun makes the read answer a
status-only `STATUS_BUSY` frame, which would pull the reply timings down.  Check
that the reply status byte (byte 1) is `0x00` on every read.  The request
frames are short (4 to 11 B), so `req_crc` reflects a small request.  Do not use
`OTA_WRITE_CHUNK` (0xF1) for this: it needs `OTA_BEGIN`, which erases the
inactive (fallback) slot, and flash programming swallows CS edges.
