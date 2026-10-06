/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal header for the gd32-bridge firmware.  Declares the
 * protocol opcodes + the dispatcher entry that both transports
 * (SPI + I2C) feed into.
 *
 * The wire-side spec is in alp-sdk docs/gd32-bridge-protocol.md; opcode
 * numbering MUST stay in sync with <alp/chips/gd32g553.h> on the
 * host side.
 */

#ifndef GD32_BRIDGE_PROTOCOL_H
#define GD32_BRIDGE_PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* --------------------------------------------------------------- */
/* Wire constants -- keep in sync with the host header.              */
/* --------------------------------------------------------------- */

#define GD32_BRIDGE_SOF         0xA5u
#define GD32_BRIDGE_I2C_REG_CMD 0x00u
/* Default 7-bit I2C slave address (host: GD32G553_BRIDGE_DEFAULT_I2C_ADDR;
 * alp-sdk metadata/chips/gd32g553.yaml default_address_7bit). */
#define GD32_BRIDGE_DEFAULT_I2C_ADDR 0x70u
#define GD32_BRIDGE_ADC_MAX_SAMPLES  8u
#define GD32_BRIDGE_BUILD_ID_LEN     20u

/* WIRE-PROTOCOL version (compatibility gate) -- GET_VERSION returns this
 * triple and the host driver refuses a mismatched MAJOR.  This is NOT
 * the firmware *release* version: that is its own semver in
 * firmware-version.txt, surfaced via GET_BUILD_ID ("<ver>+<sha>").  The
 * two axes move independently. */
#define PROTOCOL_VERSION_MAJOR 0u
/* v0.12 (bench fact 2026-09-26): the trial/confirm watchdog fallback makes
 * protocol_dispatch() answer STATUS_BUSY for EVERY opcode -- not just the
 * handful that already documented a BUSY case -- for the whole window
 * between a TRIAL boot and its confirm.  That is new, wire-observable
 * behaviour a host must be ready for, so this is a MINOR bump per
 * extending-the-gd32-bridge-protocol's own rule ("adding an opcode = MINOR
 * bump"): no opcode/payload actually changed, but a host built against an
 * OLDER MINOR has no reason to expect BUSY from e.g. CMD_PING, so it is
 * exactly the same "older hosts don't need it, newer ones should know"
 * shape that rule exists for.  0.11 is already taken by the REG_ON PR off
 * dev; this uses 0.12 to avoid a collision. */
/* v0.13: the GPIO mask grew from 20 to 21 bits -- bit 20 is CAN_STBY
 * (see CMD_GPIO_READ/CMD_GPIO_WRITE below). */
/* v0.14 (gh#101): CMD_OTA_GET_STATE's reply widens 5 -> 6 bytes, adding
 * an `err` byte (gd32_bridge_ota_err_t) that attributes an OTA_ST_ERROR
 * to its cause instead of leaving every failure indistinguishable.
 * Additive per the opcode-derived-length rule (no length is carried on
 * the wire; the reply already decodes by opcode) -- an older host that
 * only reads the first 5 bytes keeps working unchanged, so this is a
 * MINOR bump ("adding an opcode/payload field = MINOR"), not MAJOR. */
/* v0.15: negotiated extensions over v0.14 (docs/protocol-v0.15-design.md).
 * Everything new -- BIG_FRAME, ATTN, ADC_STREAM2, BATCH -- stays OFF until
 * the host enables it on a link with the 6-byte CMD_LINK_FEATURES form, so
 * an un-negotiated SPI link is byte-identical to v0.14.  The one
 * unconditional change is the I2C opcode allow-list (see
 * protocol_dispatch_inner): opcodes outside it answer STATUS_NOSUPPORT on
 * the I2C link.
 *
 * v0.16: the GPIO mask grew from 21 to 23 bits -- bit 21 is E1M IO15
 * (GD32 PB4), bit 22 is E1M IO26 (GD32 PC2).  Hosts relying on bits
 * 21/22 must require MINOR >= 16; 0.15 firmware ignores them and
 * still answers STATUS_OK. */
#define PROTOCOL_VERSION_MINOR 16u
#define PROTOCOL_VERSION_PATCH 0u

/* v0.7: opt-in link features negotiated via CMD_LINK_FEATURES.
 *
 * STATUS_SEQ ("sequence echo"): once granted, every SPI reply's STATUS
 * byte carries a 4-bit slave-side sequence stamp in bits [7:4] (the
 * status code keeps bits [3:0] -- all SPI-visible codes fit, see
 * gd32_bridge_status_t).  The stamp advances by one (mod 16) each time
 * the slave DECODES a fresh request and stages a new reply; the
 * drain/rewind re-serves of the SAME staged reply keep the same stamp.
 * A host that observes a reply whose stamp has not advanced past the
 * previous accepted reply knows the slave never consumed its request
 * -- the stale-reply residual hazard (transport_spi.c) becomes
 * DETECTABLE instead of silently masquerading as a fresh success
 * (silicon-fingerprinted 2026-06-06 on back-to-back COUNTER_READs).
 * I2C replies are NEVER stamped: the I2C-only STATUS_NO_PENDING
 * (0x80) owns bit 7 there, and the hazard is SPI-specific. */
#define GD32_BRIDGE_LINK_FEAT_STATUS_SEQ 0x00000001u
#define GD32_BRIDGE_STATUS_CODE_MASK     0x0Fu
#define GD32_BRIDGE_STATUS_SEQ_SHIFT     4u

/* v0.15 link features (the per-link feature word is a u32 from v0.15; the
 * 1-byte CMD_LINK_FEATURES form only ever carries STATUS_SEQ).  Bits 5..31
 * are reserved and never granted.  All four are SPI-only. */
/* BIG_FRAME: BATCH requests/replies and READ2 replies may exceed 65 bytes,
 * up to the negotiated per-link max_payload (<= 252 -> 256-byte SPI frame). */
#define GD32_BRIDGE_LINK_FEAT_BIG_FRAME 0x00000002u
/* ATTN: PA14 drives a level data-ready / attention line (needs STATUS_SEQ). */
#define GD32_BRIDGE_LINK_FEAT_ATTN 0x00000004u
/* ADC_STREAM2: gates CMD_ADC_STREAM_BEGIN2 / CMD_ADC_STREAM_READ2. */
#define GD32_BRIDGE_LINK_FEAT_ADC_STREAM2 0x00000008u
/* BATCH: gates CMD_BATCH. */
#define GD32_BRIDGE_LINK_FEAT_BATCH 0x00000010u

/* BIG_FRAME ceiling: 1 SOF + 1 CMD/STATUS + 252 payload + 2 CRC = 256 bytes. */
#define GD32_BRIDGE_SPI_BIG_MAX_PAYLOAD_BYTES 252u
/* READ2 reply header: first_index:u32 dropped:u32 got:u8. */
#define GD32_BRIDGE_READ2_HDR_BYTES 9u
/* Most sub-operations one CMD_BATCH may carry. */
#define GD32_BRIDGE_BATCH_MAX_OPS 16u
/* READ2 overrun guard band, in samples (docs/protocol-v0.15-design.md 5.4). */
#define GD32_BRIDGE_ADC_STREAM2_GUARD 8u
/* BEGIN2 PACE_TIMER sample-rate ceiling (BRIDGE_ADC_STREAM_RATE_MAX_HZ in
 * hal/gd32/gd32_common.h is the same value; adc_stream.c asserts it). */
#define GD32_BRIDGE_ADC_STREAM2_RATE_MAX_HZ 100000u
/* CMD_ADC_STREAM_BEGIN2 request / reply sizes. */
#define GD32_BRIDGE_BEGIN2_REQ_BYTES   12u
#define GD32_BRIDGE_BEGIN2_REPLY_BYTES 17u

/* Number of concurrent DMA-backed ADC streams the firmware supports.
 * Bounded by the GD32G553's two DMA controllers (DMA0 + DMA1 with
 * 7 channels each, per the datasheet).  Stream 0 binds to a DMA0
 * channel; stream 1 binds to a DMA1 channel; both can run at
 * different sample rates against different ADC channels. */
#define GD32_BRIDGE_ADC_STREAM_COUNT 2u

/* Host-visible ring depth ceiling per stream (firmware-side DMA
 * destination; BRIDGE_ADC_STREAM_RING_SAMPLES in hal/gd32/gd32_common.h is
 * the same 1024).  A legacy CMD_ADC_STREAM_BEGIN stream uses the whole
 * ring; a BEGIN2 stream uses the smallest power of two >= max(2*W, 5 ms of samples).  Host polls
 * CMD_ADC_STREAM_READ / _READ2 for batches; a non-empty ring lets the
 * firmware decouple DMA cadence from host poll cadence.  Slots are u16. */
#define GD32_BRIDGE_ADC_STREAM_RING_SAMPLES 1024u

/* Maximum samples returned by a single CMD_ADC_STREAM_READ reply.
 * Bounded by the wire's MAX_PAYLOAD_BYTES; tuned to keep the SPI
 * read transaction under ~1 ms at 10 MHz. */
#define GD32_BRIDGE_ADC_STREAM_READ_MAX 32u

/* Max FFT magnitude/complex bins per CMD_ADC_SPECTRUM_READ chunk.  Bins
 * are float32 (4 B); with the 7-byte reply header (seq+total+got) this
 * keeps the reply (7 + 14*4 = 63 B) inside the STREAM_READ envelope. */
#define GD32_BRIDGE_ADC_SPECTRUM_READ_MAX 14u

/* Maximum wire payload either direction.  Driven by the larger of
 * CMD_ADC_READ (1 + N*2 bytes) and CMD_ADC_STREAM_READ (1 + N*2
 * bytes), so the same formula applies; sample-count ceiling is
 * the larger of the two opcode-side limits. */
#define GD32_BRIDGE_MAX_PAYLOAD_BYTES \
	(1u + ((GD32_BRIDGE_ADC_STREAM_READ_MAX > GD32_BRIDGE_ADC_MAX_SAMPLES \
	            ? GD32_BRIDGE_ADC_STREAM_READ_MAX \
	            : GD32_BRIDGE_ADC_MAX_SAMPLES) * \
	       2u))

typedef enum {
	CMD_PING         = 0x00,
	CMD_GET_VERSION  = 0x01,
	CMD_GET_BUILD_ID = 0x02,
	CMD_RESET_REASON = 0x03,
	/* v0.15: up to GD32_BRIDGE_BATCH_MAX_OPS allow-listed sub-operations in
	 * one transaction.  SPI only; needs the BATCH link feature. */
	CMD_BATCH      = 0x04,
	CMD_GPIO_READ  = 0x10,
	CMD_GPIO_WRITE = 0x11,
	/* v0.11: the GPIO mask these two opcodes address grew from 18 to
     * 20 bits -- bits 18/19 are BT_REG_ON/WL_REG_ON, the Murata
     * LBEE5HY2FY-922 Wi-Fi/BT module's power enables (sideband, not
     * an E1M pad; GPIO_PAD_BT_REG_ON/GPIO_PAD_WL_REG_ON in
     * hal/gd32/gd32_common.h).  Older hosts addressing only bits
     * 0..17 are unaffected.
     * v0.13: grew again, 20 to 21 bits -- bit 20 is CAN_STBY, the
     * shared standby line for the two on-module TCAN1044 CAN-FD
     * transceivers (sideband, not an E1M pad; GPIO_PAD_CAN_STBY).
     * Hosts relying on bit 20 must require MINOR >= 13.
     * v0.16: grew again, 21 to 23 bits -- bits 21/22 are E1M IO15
     * (PB4) and IO26 (PC2).  Hosts relying on them must require
     * MINOR >= 16 (v0.16; 0.15 firmware ignores them). */
	/* PWM_SET with period_ns == 0 (duty_ns must be 0, else STATUS_INVAL) is
     * STOP: the channel's pad goes to its idle low level and its timer
     * claim is released, so a sibling PWM_SINGLE_PULSE is accepted again.
     * No protocol bump: firmware before this change answers period 0 with
     * STATUS_OUT_OF_RANGE, which a host can use to detect it. */
	CMD_PWM_SET = 0x20,
	CMD_PWM_GET = 0x21,
	/* v0.3: sticky per-channel PWM tuning (align mode, dead time, fault
     * inputs).  On V2N every E1M PWM channel rides one of the GD32's
     * 16-bit advanced timers (PWM0..3 -> TIMER0 channels MCH0..MCH3,
     * PWM4..7 -> TIMER7 channels MCH0..MCH3 per
     * alp-sdk `metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv`).  The firmware
     * prescales the 216 MHz timer clock to a 1 us tick: the 16-bit limit is
     * 65.536 ms edge-aligned or 131.070 ms center-aligned, and longer
     * periods return STATUS_OUT_OF_RANGE.  CMD_PWM_GET reports the actual
     * programmed value so callers can see the round-down to whole ticks. */
	CMD_PWM_CONFIGURE = 0x22,
	CMD_ADC_READ      = 0x30,
	/* v0.3: sticky per-channel ADC tuning -- oversampling ratio,
     * sample-and-hold count (a raw RSMP value in ADCCK cycles; one conversion
     * takes value + 12.5 ADCCK cycles, the model hal/gd32/gd32_common.h uses for
     * the ADC_READ residency budget and the BEGIN2 rate check; not microseconds,
     * not a rung selector),
     * resolution (6/8/10/12-bit; the GD32G5 DRES field has no 14/16-bit
     * mode, so those widths reply STATUS_NOSUPPORT).  CMD_ADC_READ honours
     * the configured tuning on the next call. */
	CMD_ADC_CONFIGURE = 0x32,
	/* v0.3: DMA-backed streaming.  Two streams can run concurrently
     * (one per GD32 DMA controller).  STREAM_BEGIN binds a stream_id
     * to a channel + sample rate; STREAM_READ drains the ring;
     * STREAM_END releases the DMA + ring.  Stream id encoded in the
     * payload of each opcode so a single handler covers both
     * controllers. */
	CMD_ADC_STREAM_BEGIN      = 0x33,
	CMD_ADC_STREAM_READ       = 0x34,
	CMD_ADC_STREAM_END        = 0x35,
	CMD_DA9292_STATUS_FORWARD = 0x40,
	/* v0.8: secure-element reset line.  SE_RST = GD32 PC13 drives the
     * OPTIGA Trust M's reset (the SE sits on the shared BRD_I2C bus per
     * alp-sdk metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv).  Request payload
     * `assert:u8` -- 0 = release (SE runs), 1 = hold the SE in reset;
     * empty reply.  Pulsing it (1 -> wait -> 0) is the recovery for a
     * wedged BRD_I2C where the SE clock-stretches SCL low; the host
     * sequences the pulse + post-reset settle (the SPI transport reaches
     * the GD32 even while BRD_I2C is held low).  The active level is
     * firmware-owned (OPTIGA RST is active-low; see hal/gd32/se_reset.c). */
	CMD_SE_RESET = 0x41,
	/* v0.15: persistent boot configuration (src/boot_config.h).  Request
	 * `op:u8 flags:u32` (5 B): op 0 = GET (flags ignored), op 1 = SET (store
	 * `flags`, bit0 = drive PD11 / E1M IO29 SDIO_MUX_EN high at every boot;
	 * unknown bits -> STATUS_INVAL).  Reply `flags:u32` = the STORED value.
	 * SET is asynchronous: it is accepted at once (reply = the old stored
	 * value) and committed from the main loop, never in the transport ISR;
	 * the host polls GET until the stored value equals what it asked for.
	 * The commit erases two 1 KB pages on dual-bank parts (each <= 20 ms,
	 * interrupts masked), so the link can black out for 2 x 20 ms plus
	 * main-loop latency; there is no fixed idle window, so GET polls must be
	 * retry-tolerant.  A SET while an OTA session is active answers BUSY.  A SET equal to the stored value is a no-op; a
	 * different SET while one is queued answers BUSY.  SET takes effect at
	 * the next GD32 reset and does NOT touch a pad now.  NOSUPPORT on a
	 * build without the FMC HAL, with OBCTL.DBS = 0, or running from bank 1.
	 * Allowed on the I2C link (provisioning runs from Linux).  Added inside
	 * the unreleased 0.15 line: no version bump. */
	CMD_BOOT_CONFIG = 0x42,
	/* v0.2 additions -- the GD32 carries every E1M-standard analog
	 * and counter peripheral on V2N (per alp-sdk gd32-io-mcu-map.tsv); the
     * SDK's portable surface routes through these. */
	CMD_DAC_SET      = 0x50,
	CMD_DAC_GET      = 0x51,
	CMD_QENC_READ    = 0x60,
	CMD_QENC_RESET   = 0x61,
	CMD_COUNTER_READ = 0x70,
	/* v0.3: GD32G5 security block.  TRNG is the NIST SP800-90B
     * pre-certified true-random generator -- 32-bit pull per op (or
     * 128-bit on the NIST path; firmware-internal choice).  The CAU
     * (0x4802 1000) is SYMMETRIC-ONLY -- DES/TDES/AES; the GD32G553 has
     * no PKA, HMAC, CMAC or message-hash engine (GD32G553 User Manual
     * Rev1.2 p.350 §13.1; confirmed absent from the peripheral memory
     * map, Datasheet Rev2.0 p.19).  There is no image-signing path on
     * this part -- see SECURITY.md. */
	CMD_TRNG_READ = 0x80,
	/* v0.4: GD32G5 TMU (CORDIC) math accelerator.  General-purpose
     * fixed-function trig / sqrt / log / exp / vector-magnitude block
     * with a 12-input function table.  Request payload
     * `function:u8 format:u8 reserved:u16 in_a:u32 in_b:u32` (12 B);
     * reply `result:u32 status:u8` (5 B).  Format 0 = Q31 fixed-point;
     * format 1 = IEEE-754 single (the firmware's default plumbing). */
	CMD_TMU_COMPUTE = 0x90,
	/* v0.5: ADC-stream DSP pipeline configuration -- attaches a chain
     * of FIR / IIR / WINDOW / FFT stages to a streaming ADC source so
     * raw samples never leave the GD32 when the customer's intent is
     * filtered or spectral data (the link bandwidth wins more from
     * pushing the post-processing onto the bridge than from sending
     * the raw samples).  RESERVED + tombstoned at v0.5: the original
     * single-shot configure payload won't fit one FIR stage's 256-byte
     * Q31-tap blob inside the 65-byte wire envelope, so the actual
     * upload path is the three chunked sub-opcodes CMD_ADC_DSP_CHAIN_*
     * (0x37/0x38/0x39) below.  The 0x36 opcode keeps its slot to
     * avoid renumbering across the v0.5.x line; firmware default-case
     * dispatch returns STATUS_NOSUPPORT for it. */
	CMD_ADC_STREAM_CONFIGURE_DSP = 0x36,
	/* v0.5 (§2B): chunked DSP-chain upload path.  Three sub-opcodes:
     *   CHAIN_OPEN   allocates a firmware-side chain handle, returns
     *                its u8 id in the reply payload.
     *   STAGE_PUSH   uploads one chunk of one stage's per-kind params
     *                (FIR taps / IIR sections / WINDOW shape / FFT
     *                size + output format) into the named chain at
     *                the named stage_index + chunk_offset.  Variable
     *                request payload up to GD32_BRIDGE_MAX_PAYLOAD_BYTES;
     *                empty reply.  Repeated as many times as needed
     *                to assemble the full per-kind blob.
     *   CHAIN_BIND   attaches the chain to an existing streaming ADC
     *                source by stream_id.  From then on the stream's
     *                samples flow through the chain instead of straight
     *                to the host -- the v0.5.x wire format flip from
     *                "host-side DSP" to "GD32-side DSP" that's the real
     *                wave-2 value.
     * All three are RESERVED at protocol v0.5; firmware default-case
     * dispatch returns STATUS_NOSUPPORT until the bridge_hw_adc_dsp_*
     * HAL bodies land in the GD32 firmware tree.  Host helpers in
     * alp-sdk chips/gd32g553/ honour the same NOSUPPORT contract by routing
     * the wire dispatch through cmd_send unchanged. */
	CMD_ADC_DSP_CHAIN_OPEN = 0x37,
	CMD_ADC_DSP_STAGE_PUSH = 0x38,
	CMD_ADC_DSP_CHAIN_BIND = 0x39,
	/* Read one chunk of the latest FFT frame for an FFT-terminal
	 * chain bound to a stream (#496).  Request: stream_id:u8
	 * bin_offset:u16(LE) max_bins:u8.  Reply: seq:u32(LE)
	 * total_bins:u16(LE) got:u8 bins[max_bins*4] (float32 LE,
	 * zero-padded past `got`).  seq lets the host detect a frame
	 * roll mid-fetch.  STREAM_READ on an FFT-bound stream answers
	 * NOSUPPORT; a filter (FIR/IIR) chain uses STREAM_READ instead. */
	CMD_ADC_SPECTRUM_READ = 0x3A,
	/* v0.15: BEGIN2 / READ2 -- hardware-realised rate reported back,
	 * watermark events, and lossless-accounting reads (first_index +
	 * dropped).  SPI only; need the ADC_STREAM2 link feature. */
	CMD_ADC_STREAM_BEGIN2 = 0x3B,
	CMD_ADC_STREAM_READ2  = 0x3C,
	/* v0.5 (§2B.2): advanced timer extras.  PWM_CAPTURE turns a
     * PWM channel's pin into an input-capture source for frequency
     * / pulse-width measurement; PWM_SINGLE_PULSE drives a one-shot
     * pulse of caller-specified duration on a PWM channel then
     * stops; TIMER_SYNC links the initialised TIMER0 / TIMER7 groups
     * (wire ids 0 / 1) in master-slave configuration for synchronised
     * multi-channel output.  TIMER19's former id 2 is rejected because
     * this firmware never clocks or initialises it (#142).  The GD32 HAL
     * implements these opcodes; builds without the corresponding HAL
     * body retain the STATUS_NOSUPPORT contract. */
	CMD_PWM_CAPTURE_BEGIN = 0x23,
	CMD_PWM_CAPTURE_READ  = 0x24,
	CMD_PWM_CAPTURE_END   = 0x25,
	CMD_PWM_SINGLE_PULSE  = 0x26,
	CMD_TIMER_SYNC        = 0x27,
	/* v0.5 (§2B.3): system-wide power-mode transition.  Host requests
     * SLEEP / DEEP_SLEEP / STANDBY via the supervisor, which prepares
     * the GD32 + signals the Renesas SoC to enter the matching mode,
     * then re-runs the bridge handshake on wakeup so the host can
     * resume bridge calls.  RESERVED at protocol v0.5; firmware
     * dispatcher returns STATUS_NOSUPPORT until the HAL body lands.
     * Portable surface in <alp/power.h>. */
	CMD_POWER_MODE_SET = 0x28,
	/* v0.7: link-feature negotiation.  Request `features:u8` = the set
     * the host wants (GD32_BRIDGE_LINK_FEAT_*); reply `features:u8` =
     * the subset the firmware granted (and immediately armed).  A
     * request of 0 disables everything (idempotent).  Older firmware
     * answers STATUS_NOSUPPORT via the dispatch default -- the host
     * degrades to the legacy framing.  The reply to THIS command is
     * already stamped when STATUS_SEQ is granted; the host uses that
     * stamp as its sequence baseline.
     * v0.15: a second, 6-byte request form `want:u32 max_payload_req:u16`
     * negotiates the whole u32 feature word and the link's max_payload;
     * its reply is `granted:u32 supported:u32 max_payload:u16` (10 B).
     * The legacy 1-byte form keeps only STATUS_SEQ and clears every 0.15
     * bit on that link. */
	CMD_LINK_FEATURES = 0x81,
} gd32_bridge_cmd_t;

/* TMU function index sent in CMD_TMU_COMPUTE's request payload byte 0.
 * Mirrors @ref gd32g553_tmu_function_t on the host side. */
typedef enum {
	BRIDGE_TMU_FN_SIN    = 0u,
	BRIDGE_TMU_FN_COS    = 1u,
	BRIDGE_TMU_FN_TAN    = 2u,
	BRIDGE_TMU_FN_ATAN   = 3u,
	BRIDGE_TMU_FN_ATAN2  = 4u,
	BRIDGE_TMU_FN_SQRT   = 5u,
	BRIDGE_TMU_FN_LOG    = 6u,
	BRIDGE_TMU_FN_EXP    = 7u,
	BRIDGE_TMU_FN_SINH   = 8u,
	BRIDGE_TMU_FN_COSH   = 9u,
	BRIDGE_TMU_FN_TANH   = 10u,
	BRIDGE_TMU_FN_HYPOT  = 11u,
	BRIDGE_TMU_FN__COUNT = 12u, /**< sentinel; not a valid function. */
} gd32_bridge_tmu_function_t;

/* TMU operand / result format byte 1 of CMD_TMU_COMPUTE.  Mirrors
 * @ref gd32g553_tmu_format_t on the host side. */
typedef enum {
	BRIDGE_TMU_FMT_Q31    = 0u, /**< Q31 fixed-point (signed).  */
	BRIDGE_TMU_FMT_F32    = 1u, /**< IEEE-754 single precision. */
	BRIDGE_TMU_FMT__COUNT = 2u, /**< sentinel.                   */
} gd32_bridge_tmu_format_t;

/* Wire-side status byte; mirrors the table in alp-sdk docs/gd32-bridge-protocol.md §6.
 * Note the unsigned magnitude vs the host's negative alp_status_t -- the
 * firmware doesn't depend on the host's signed enum. */
typedef enum {
	STATUS_OK           = 0x00,
	STATUS_INVAL        = 0x01,
	STATUS_NOT_READY    = 0x02,
	STATUS_BUSY         = 0x03,
	STATUS_TIMEOUT      = 0x04,
	STATUS_IO           = 0x05,
	STATUS_NOSUPPORT    = 0x06,
	STATUS_NOMEM        = 0x07,
	STATUS_OUT_OF_RANGE = 0x08,
	STATUS_NO_PENDING   = 0x80, /* I2C-only: read before any matching write */
} gd32_bridge_status_t;

/* --------------------------------------------------------------- */
/* Dispatcher                                                         */
/* --------------------------------------------------------------- */

/* Which transport a request arrived on.  protocol_dispatch() takes this
 * so a handler whose effect is scoped to ONE link cannot reach across to
 * the other: the command table is shared by design, but state armed by a
 * command is not always shareable.  CMD_LINK_FEATURES is the first such
 * command (#130) -- STATUS_SEQ is declared SPI-only above, and the SPI
 * transport is its only consumer.
 *
 * Values are a dense index into protocol.c's per-link feature array; do
 * not renumber without updating it. */
typedef enum {
	GD32_BRIDGE_LINK_SPI = 0,
	GD32_BRIDGE_LINK_I2C = 1,
	GD32_BRIDGE_LINK_COUNT
} gd32_bridge_link_t;

/*
 * OTA failure causes (gh#101) -- the `err` byte of CMD_OTA_GET_STATE's
 * reply.  Nine distinct non-zero causes used to collapse into
 * `state = ERROR` with nothing else on the wire, so a failed session
 * was unattributable for the host and indistinguishable on the bench
 * (the 2026-06-04 campaign's seven silicon bugs all presented the
 * same).  The values are the same bare integers ota.c always assigned
 * (s_err was written in nine places and read in none); they are now a
 * documented enum so the next write site cannot collide by accident.
 * Do NOT renumber: the wire pins them, and the host driver decodes
 * them by value.  0 = no error recorded (idle / clean session).
 */
typedef enum {
	BRIDGE_OTA_ERR_NONE               = 0x00,
	BRIDGE_OTA_ERR_SESSION_RANGE      = 0x01, /* BEGIN/VERIFY/COMMIT image size
	                                     * out of range */
	BRIDGE_OTA_ERR_ERASE_FAILED       = 0x02, /* background page erase failed */
	BRIDGE_OTA_ERR_CHUNK_RANGE        = 0x03, /* chunk offset / length rejected */
	BRIDGE_OTA_ERR_PROGRAM_FAILED     = 0x04, /* flash program failed (PGERR/PGSERR) */
	BRIDGE_OTA_ERR_VERIFY_CRC         = 0x05, /* VERIFY's CRC comparison failed */
	BRIDGE_OTA_ERR_COMMIT_FAILED      = 0x06, /* COMMIT: bootability check or
	                                     * metadata commit failed */
	BRIDGE_OTA_ERR_ERASE_TARGET       = 0x07, /* erase target would intersect
	                                     * the running slot (#3 guard) */
	BRIDGE_OTA_ERR_NOT_TRIAL_CAPABLE  = 0x08, /* COMMIT refused: the candidate
	                                     * image has no valid trial marker,
	                                     * so it cannot be confirm-gated */
	BRIDGE_OTA_ERR_META_DEMOTE_FAILED = 0x09, /* BEGIN: the metadata commit
	                                     * that demotes the stale target
	                                     * slot's valid bit before erase
	                                     * failed */
	BRIDGE_OTA_ERR_BELOW_FLOOR        = 0x0A, /* COMMIT/ROLLBACK refused: the
	                                     * image's version is below the
	                                     * anti-rollback floor (#49) */
} gd32_bridge_ota_err_t;

/*
 * protocol_dispatch -- called by either transport when a complete
 * request envelope has been validated (CRC OK, framing OK).
 *
 * Inputs:
 *   link           -- the transport this request arrived on
 *                     (GD32_BRIDGE_LINK_SPI / _I2C).  Only link-scoped
 *                     handlers consult it; the shared command table is
 *                     otherwise identical on both links.
 *   cmd            -- opcode (one of CMD_*).
 *   req_payload    -- pointer to N request payload bytes (may be
 *                     NULL when req_payload_len == 0).
 *   req_payload_len-- length of req_payload.
 *
 * Outputs (caller-supplied):
 *   reply_payload      -- buffer for M reply payload bytes.
 *   reply_payload_cap  -- capacity of reply_payload.
 *   reply_payload_len  -- [out] M (bytes actually written).
 *
 * Only one dispatch may execute at a time across both transport ISRs.
 * A nested request returns STATUS_BUSY with a zero-length payload before
 * entering any command handler; the host may retry it after the active
 * request completes.
 *
 * Return:  STATUS_OK on success; STATUS_NOSUPPORT for unknown
 *          opcodes; STATUS_INVAL on bad payload lengths /
 *          out-of-range args; STATUS_TIMEOUT / STATUS_IO for
 *          downstream peripheral errors (e.g. an ADC or timer
 *          peripheral fault).
 */
gd32_bridge_status_t protocol_dispatch(gd32_bridge_link_t link,
                                       uint8_t            cmd,
                                       const uint8_t     *req_payload,
                                       size_t             req_payload_len,
                                       uint8_t           *reply_payload,
                                       size_t             reply_payload_cap,
                                       size_t            *reply_payload_len);

/* Link features currently armed ON `link` (GD32_BRIDGE_LINK_FEAT_* bits,
 * set by a CMD_LINK_FEATURES that arrived on that same link).  Consulted
 * by the SPI transport when staging replies; 0 = legacy framing.  u32
 * from v0.15.
 *
 * Per-link since #132's sibling #130: the feature set used to be one
 * process-wide byte, so an I2C-side negotiation re-framed the SPI wire
 * for a host that never asked -- and an I2C-side `features = 0` silently
 * disarmed an active SPI STATUS_SEQ session mid-flight, switching off the
 * SPI host's ONLY detector for the stale-reply residual hazard
 * fingerprinted on silicon 2026-06-06. */
uint32_t protocol_link_features(gd32_bridge_link_t link);

/* Effective payload ceiling on `link` (65, or 66..252 once BIG_FRAME is
 * granted on the SPI link).  Only CMD_BATCH requests/replies and
 * CMD_ADC_STREAM_READ2 replies may use more than 65 (the SPI transport
 * enforces that for requests; protocol_dispatch() is handed the matching
 * reply capacity). */
uint16_t protocol_link_max_payload(gd32_bridge_link_t link);

/* I2C opcode-policy diagnostics (SWD-readable, same style as
 * bridge_i2c_rx_diag): how many I2C requests were refused because their
 * opcode is outside the I2C allow-list, and the last such opcode. */
extern volatile uint32_t bridge_i2c_denied_count;
extern volatile uint8_t  bridge_i2c_denied_last_cmd;

/* --------------------------------------------------------------- */
/* CRC-16 / CCITT-FALSE -- shared between transports.                */
/* --------------------------------------------------------------- */

uint16_t crc16_ccitt_false(const uint8_t *buf, size_t len);

#endif /* GD32_BRIDGE_PROTOCOL_H */
