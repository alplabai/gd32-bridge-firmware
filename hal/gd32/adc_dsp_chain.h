/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * DSP-chain pool + bind-time validation -- declarations shared between
 * hal/gd32/adc_dsp_chain.c (the pool + chain_bind, vendor-header-free
 * and therefore host-testable) and hal/gd32/adc_stream.c (the FAC/FFT
 * register-level pump, which decodes a bound chain's stage blobs).
 *
 * Type-light like gd32_common.h: this header pulls in only the ISO C
 * headers below and NEVER "gd32g5x3.h" -- see adc_dsp_chain.c's own
 * header comment for why that matters (#69/#70 host regression).
 */

#ifndef GD32_BRIDGE_HAL_GD32_ADC_DSP_CHAIN_H
#define GD32_BRIDGE_HAL_GD32_ADC_DSP_CHAIN_H

#include <stdbool.h>
#include <stdint.h>

/* Pool sizing -- mirrors the constants in `<alp/chips/gd32g553.h>`
 * so the host's view of "what fits" agrees with the firmware's
 * actual buffer reservation.  These local copies avoid pulling the
 * SDK header into the firmware tree (which would drag in alp_status_t
 * + supplementary ALP types the firmware doesn't otherwise consume).
 * Bumping any of them requires a coordinated edit on both sides --
 * see `docs/gd32-bridge-protocol.md` §3.x (alp-sdk --
 * https://github.com/alplabai/alp-sdk/blob/main/docs/gd32-bridge-protocol.md)
 * for the wire-format implications.  That file lives in the OTHER
 * repository; this one has no docs/ tree. */
#define BRIDGE_DSP_MAX_CHAINS      4u
#define BRIDGE_DSP_MAX_STAGES      4u
#define BRIDGE_DSP_MAX_STAGE_BYTES 260u

/* Valid `kind` byte range -- alp_dsp_stage_kind_t mirrors the wire
 * encoding: 0 FIR, 1 IIR, 2 WINDOW, 3 FFT.  Anything outside this
 * range rejects at stage_push so a typo from the host is caught
 * before any bytes hit the per-stage buffer. */
#define BRIDGE_DSP_KIND_MAX 3u

/* Per-kind parameter bounds -- mirror the ALP_DSP_MAX_* macros in
 * `<alp/dsp.h>` so the firmware's assembled-blob validation agrees
 * with the host's construction limits.  See the reassembled blob
 * layout in `<alp/chips/gd32g553.h>` (gd32g553_adc_dsp_stage_push):
 *   FIR    : format:u8 n_taps:u8    rsvd:u16  taps[n_taps*4]  (Q31/F32)
 *   IIR    : format:u8 n_sections:u8 rsvd:u16 coeffs[n_sec*5*4]
 *   WINDOW : shape:u8  rsvd[3]                                  (4 B)
 *   FFT    : n_points:u16 out_fmt:u8 rsvd:u8                    (4 B) */
#define BRIDGE_DSP_MAX_FIR_TAPS     64u
#define BRIDGE_DSP_MAX_IIR_SECTIONS 8u
#define BRIDGE_DSP_MIN_FFT_POINTS   32u
#define BRIDGE_DSP_MAX_FFT_POINTS   1024u
#define BRIDGE_DSP_COEFF_FMT_MAX    1u /* 0 F32, 1 Q31 */
#define BRIDGE_DSP_WINDOW_SHAPE_MAX 3u /* rect/hann/hamming/blackman */
#define BRIDGE_DSP_FFT_OUT_FMT_MAX  2u /* complex/magnitude/magnitude-onesided */
#define BRIDGE_DSP_STAGE_HDR_BYTES  4u /* every kind's fixed 4-byte header */

typedef struct {
	uint8_t  kind;           /* alp_dsp_stage_kind_t (valid when total_size > 0) */
	uint16_t total_size;     /* declared in first chunk; locks for the stage    */
	uint16_t bytes_received; /* running count toward total_size                  */
	bool     complete;       /* bytes_received == total_size                     */
	uint8_t  data[BRIDGE_DSP_MAX_STAGE_BYTES];
} adc_dsp_stage_t;

typedef struct {
	bool            in_use;
	bool            bound;
	adc_dsp_stage_t stages[BRIDGE_DSP_MAX_STAGES];
} adc_dsp_chain_t;

/* 4 chains x 4 stages x 260 B = 4160 bytes of stage-data RAM + ~80
 * bytes of metadata; well inside the GD32G553's 128 KB SRAM. */
extern adc_dsp_chain_t adc_dsp_chains[BRIDGE_DSP_MAX_CHAINS]; /* adc_dsp_chain.c */

/* Return a chain slot to the pool.  Idempotent-safe for an out-of-range
 * id (a no-op).
 *
 * LIFECYCLE (F1, review of #69/#70) -- who calls this, and when, is
 * otherwise implicit; spelled out here because getting it wrong either
 * leaks a slot (a bus-reachable, reboot-only DoS of the DSP feature --
 * the pool is only BRIDGE_DSP_MAX_CHAINS entries) or frees a chain_id
 * a host still legitimately owns.  There are exactly THREE release
 * sites, all in the tree today:
 *
 *   1. bridge_hw_adc_dsp_chain_bind() (below), on a CAPABILITY refusal
 *      (adc_dsp_chain_p1_capable() false -> BRIDGE_HW_ERR_NOTIMPL).
 *      This chain's SHAPE can never become realisable by retrying --
 *      P1-capability depends only on the chain's own stages, not on
 *      stream_id -- so a conforming host that heeds NOTIMPL and
 *      abandons the chain must not be the one leaking the slot: bind
 *      releases it itself.  chain_id is INVALID after this return; a
 *      host must not push more stages to it or retry bind with it.
 *
 *   2. bridge_hw_adc_dsp_chain_bind() does NOT release on a BUSY
 *      refusal (adc_dsp_filter_stream_busy() / adc_dsp_fft_stream_busy()
 *      -> BRIDGE_HW_ERR_NOTIMPL): this chain IS realisable, just
 *      contended for a single-instance HW block (the FAC or the FFT
 *      unit) another stream currently owns.  chain_id stays OPEN and
 *      BOUND == false; the supported recovery is retrying the SAME
 *      chain_id (typically after the contending stream's STREAM_END).
 *
 *   3. bridge_hw_adc_stream_end() (adc_stream.c), on a stream that WAS
 *      dsp_bound (gated on s->dsp_bound -- dsp_chain_id is only
 *      meaningful while bound).  This is the ONLY release path for a
 *      chain that made it all the way to a successful bind: there is
 *      no host-facing CHAIN_CLOSE opcode, so a bound chain's lifetime
 *      is tied 1:1 to the stream it is bound to, and ending the stream
 *      is the only way to free it.
 *
 * A chain that is OPENed but never BOUND and never hits site 1 above
 * (e.g. the host simply stops talking to it, or a structural INVAL at
 * bind, which the FIX above the capability check deliberately mirrors
 * by NOT releasing) has no automatic release path today -- same as
 * before #69/#70; out of scope here (would need a CHAIN_CLOSE opcode,
 * a wire addition). */
void adc_dsp_chain_release(uint8_t chain_id); /* adc_dsp_chain.c */

/* Is a FIR/IIR (filter, not FFT) chain already bound to some OTHER
 * stream?  The single FAC can serve only one at a time. */
bool adc_dsp_filter_stream_busy(uint8_t except_stream); /* adc_dsp_chain.c */

/* Is an FFT-terminal chain already bound to some OTHER stream?  Like
 * the FAC, the FFT block is single-instance -- ONE FFT stream at a
 * time (adc_stream.c's own header comment says so; nothing enforced
 * it until #70).  Mirrors adc_dsp_filter_stream_busy but selects the
 * FFT terminal instead of excluding it. */
bool adc_dsp_fft_stream_busy(uint8_t except_stream); /* adc_dsp_chain.c */

/* The shared "does the P1 runtime support this chain's shape"
 * predicate (#69, #70).  P1's FAC/FFT config functions
 * (adc_dsp_fac_config / adc_dsp_fft_config in adc_stream.c) and
 * chain_bind's capability check must agree on these limits by
 * CONSTRUCTION -- one function, called from both sides -- rather than
 * by two independently-maintained copies; a capability lift landing
 * on only one side is exactly how #69/#70 happened (chain_bind
 * validated shape/ordering but not runtime capability, so an
 * unrealisable chain bound cleanly and the pump silently refused it
 * forever after).
 *
 * Assumes the chain already passed chain_bind's STRUCTURAL checks
 * (every populated stage complete + blob-valid, no gaps, FFT terminal
 * with WINDOW immediately preceding it if present) -- it only judges
 * REALISABILITY under P1's capability limits:
 *   - a non-FFT-terminal chain: exactly one populated stage (P1's FAC
 *     handles a single filter, never a multi-stage chain),
 *   - a populated IIR stage: exactly one biquad section (P1 has no
 *     cascaded-section support),
 *   - an FFT-terminal chain: only WINDOW/FFT populated stages (P1's
 *     FFT block has no upstream filter -- a FIR/IIR ahead of the
 *     transform is not a P1 spectrum chain, bare or WINDOW-preceded),
 *     AND its declared n_points within
 *     [BRIDGE_DSP_MIN_FFT_POINTS, BRIDGE_DSP_MAX_FFT_POINTS] (F3,
 *     review of #69/#70 -- adc_stream.c's FFT buffers are sized by
 *     ADC_DSP_FFT_MAX_POINTS, a compile-time alias of the MAX bound
 *     here, so this check and that sizing can't drift apart). */
bool adc_dsp_chain_p1_capable(const adc_dsp_chain_t *chain); /* adc_dsp_chain.c */

#endif /* GD32_BRIDGE_HAL_GD32_ADC_DSP_CHAIN_H */
