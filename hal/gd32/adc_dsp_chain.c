/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * DSP-chain pool + bind-time validation.  Split out of adc_stream.c
 * (#69, #70): this file is VENDOR-HEADER-FREE -- it never includes
 * "gd32g5x3.h" and touches no GD32 register -- so a host test can
 * link it directly and drive CHAIN_OPEN -> STAGE_PUSH -> CHAIN_BIND
 * without a Cortex-M33 toolchain or the vendor firmware library.  See
 * tests/unit/adc_dsp/.
 *
 * What stays in adc_stream.c instead, because it touches RCU/FAC/FFT
 * registers: adc_dsp_fac_config, adc_dsp_pump_stream, adc_dsp_fft_config,
 * adc_dsp_pump_fft.  Those call adc_dsp_chain_p1_capable() below as a
 * defence-in-depth re-check (see that function's doc comment) but own
 * no capability limit themselves anymore -- this file is the ONE place
 * "what P1 can realise" is encoded.  That includes the FFT point-count
 * cap (F3, review of #69/#70): adc_dsp_chain_p1_capable() enforces
 * BRIDGE_DSP_MIN_FFT_POINTS/BRIDGE_DSP_MAX_FFT_POINTS below, and
 * adc_stream.c's ADC_DSP_FFT_MAX_POINTS -- which sizes its static FFT
 * buffers -- is a compile-time ALIAS of BRIDGE_DSP_MAX_FFT_POINTS, not
 * an independently-maintained copy, so the two limits cannot drift
 * apart the way they did before this fix (raising one macro alone used
 * to let a chain bind that the pump then silently refused forever --
 * the exact #69 starvation shape, regenerated through this one limit).
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bridge_hw.h"

#include "adc_dsp_chain.h"
#include "gd32_common.h"

/* 4 chains x 4 stages x 260 B = 4160 bytes of stage-data RAM + ~80
 * bytes of metadata; well inside the GD32G553's 128 KB SRAM.  Non-
 * static (adc_dsp_chain.h) because adc_stream.c's pump-side config
 * functions index it directly to decode a bound chain's stage blobs. */
adc_dsp_chain_t adc_dsp_chains[BRIDGE_DSP_MAX_CHAINS];

/* Return a chain slot to the pool.  The counterpart to chain_open's
 * first-fit allocation: there is no host-facing close opcode, so a
 * chain's lifetime is tied to the stream it binds -- stream_end
 * (adc_stream.c) calls this on the bound chain.  Idempotent-safe for
 * an out-of-range id. */
void adc_dsp_chain_release(uint8_t chain_id)
{
	if (chain_id >= BRIDGE_DSP_MAX_CHAINS) return;
	adc_dsp_chains[chain_id].bound  = false;
	adc_dsp_chains[chain_id].in_use = false;
}

/* Validate one completed stage's reassembled blob against its declared
 * `kind`.  stage_push only bounds the byte COUNT (<= total_size) and
 * the kind range; it never looks at the payload.  This runs at bind --
 * the last point before the chain goes live -- so a filter with a bad
 * tap count, an out-of-range FFT size, or a header/length mismatch is
 * rejected here rather than mis-programming the FAC/FFT block later.
 * The 4-byte header is present for every kind (guaranteed because bind
 * only inspects populated stages, and total_size >= 1 for those --
 * but we re-check to keep the field reads in-bounds). */
static bool adc_dsp_stage_blob_valid(const adc_dsp_stage_t *st)
{
	if (st->total_size < BRIDGE_DSP_STAGE_HDR_BYTES) return false;
	const uint8_t *d = st->data;

	switch (st->kind) {
	case 0u: { /* FIR: format:u8 n_taps:u8 rsvd:u16 taps[n_taps*4] */
		const uint8_t fmt    = d[0];
		const uint8_t n_taps = d[1];
		if (fmt > BRIDGE_DSP_COEFF_FMT_MAX) return false;
		if (n_taps == 0u || n_taps > BRIDGE_DSP_MAX_FIR_TAPS) return false;
		return st->total_size == (uint16_t)(BRIDGE_DSP_STAGE_HDR_BYTES + (uint16_t)n_taps * 4u);
	}
	case 1u: { /* IIR: format:u8 n_sections:u8 rsvd:u16 coeffs[n_sec*5*4] */
		const uint8_t fmt   = d[0];
		const uint8_t n_sec = d[1];
		if (fmt > BRIDGE_DSP_COEFF_FMT_MAX) return false;
		if (n_sec == 0u || n_sec > BRIDGE_DSP_MAX_IIR_SECTIONS) return false;
		return st->total_size == (uint16_t)(BRIDGE_DSP_STAGE_HDR_BYTES + (uint16_t)n_sec * 5u * 4u);
	}
	case 2u: /* WINDOW: shape:u8 rsvd[3] */
		if (d[0] > BRIDGE_DSP_WINDOW_SHAPE_MAX) return false;
		return st->total_size == BRIDGE_DSP_STAGE_HDR_BYTES;
	case 3u: { /* FFT: n_points:u16 out_fmt:u8 rsvd:u8 */
		const uint16_t n_points = (uint16_t)(d[0] | ((uint16_t)d[1] << 8));
		const uint8_t  out_fmt  = d[2];
		if (out_fmt > BRIDGE_DSP_FFT_OUT_FMT_MAX) return false;
		if (n_points < BRIDGE_DSP_MIN_FFT_POINTS || n_points > BRIDGE_DSP_MAX_FFT_POINTS)
			return false;
		if ((n_points & (uint16_t)(n_points - 1u)) != 0u) return false; /* pow2 */
		return st->total_size == BRIDGE_DSP_STAGE_HDR_BYTES;
	}
	default:
		return false;
	}
}

int bridge_hw_adc_dsp_chain_open(uint8_t *chain_id)
{
	if (chain_id == 0) return BRIDGE_HW_ERR_INVAL;
	*chain_id = 0u;

	/* First-fit search over the chain pool.  The pool is small (4
     * entries today) so the linear scan is comfortably faster than
     * any free-list bookkeeping would be; if the pool grows, this
     * function is the natural place to add a free-list head. */
	for (uint8_t i = 0u; i < BRIDGE_DSP_MAX_CHAINS; ++i) {
		if (!adc_dsp_chains[i].in_use) {
			/* Zero the chain state so a previously-released chain
             * doesn't leak stale stage data into the new allocation. */
			for (uint8_t s = 0u; s < BRIDGE_DSP_MAX_STAGES; ++s) {
				adc_dsp_chains[i].stages[s].kind           = 0u;
				adc_dsp_chains[i].stages[s].total_size     = 0u;
				adc_dsp_chains[i].stages[s].bytes_received = 0u;
				adc_dsp_chains[i].stages[s].complete       = false;
			}
			adc_dsp_chains[i].in_use = true;
			adc_dsp_chains[i].bound  = false;
			*chain_id                = i;
			return BRIDGE_HW_OK;
		}
	}
	/* Pool exhaustion.  Protocol layer maps BRIDGE_HW_ERR_NOTIMPL to
     * STATUS_NOSUPPORT (0x06) today.  STATUS_NOMEM (0x07) has 17
     * defensive reply_cap guards in protocol.c, but both transports
     * pass a 65-byte reply buffer so none of them can fire on the
     * wire -- what's missing is a NOMEM-equivalent BRIDGE_HW_ERR_*
     * here in hal/bridge_hw.h for this path. */
	return BRIDGE_HW_ERR_NOTIMPL;
}

int bridge_hw_adc_dsp_stage_push(uint8_t        chain_id,
                                 uint8_t        stage_index,
                                 uint8_t        kind,
                                 uint16_t       chunk_offset,
                                 uint16_t       chunk_total_size,
                                 const uint8_t *chunk_data,
                                 size_t         chunk_data_len)
{
	if (chain_id >= BRIDGE_DSP_MAX_CHAINS) return BRIDGE_HW_ERR_RANGE;
	if (stage_index >= BRIDGE_DSP_MAX_STAGES) return BRIDGE_HW_ERR_RANGE;
	if (kind > BRIDGE_DSP_KIND_MAX) return BRIDGE_HW_ERR_INVAL;
	if (chunk_total_size == 0u) return BRIDGE_HW_ERR_INVAL;
	if (chunk_total_size > BRIDGE_DSP_MAX_STAGE_BYTES) return BRIDGE_HW_ERR_RANGE;
	if (chunk_data_len == 0u || chunk_data == 0) return BRIDGE_HW_ERR_INVAL;
	/* `chunk_offset + chunk_data_len <= chunk_total_size`, checked
     * WITHOUT computing `chunk_offset + chunk_data_len` (F2, review of
     * #69/#70): chunk_offset and chunk_total_size are both uint16_t, so
     * `chunk_total_size - chunk_offset` promotes to `int` (NOT the
     * "overflow-safe" subtraction a since-corrected comment here used to
     * claim) -- for chunk_offset > chunk_total_size that int goes
     * negative, and casting a negative int to size_t wraps to a huge
     * unsigned value against which chunk_data_len always compares
     * smaller, so the guard silently PASSED every out-of-range offset.
     * ASan confirmed the resulting OOB write in st->data[] below
     * (chain_open, push offset=0 total=260 len=1, then push
     * offset=60000 total=260 len=1).  The explicit ordering check first
     * makes the subtraction's operands well-ordered before it runs; the
     * wire path (src/protocol.c's handle_adc_dsp_stage_push) has always
     * had an independent, correctly-widened uint32_t check, which is
     * why this was unreachable from the wire despite being directly
     * linkable and reachable from a host test. */
	if (chunk_offset > chunk_total_size) return BRIDGE_HW_ERR_RANGE;
	if (chunk_data_len > (size_t)(chunk_total_size - chunk_offset)) return BRIDGE_HW_ERR_RANGE;

	adc_dsp_chain_t *chain = &adc_dsp_chains[chain_id];
	if (!chain->in_use) return BRIDGE_HW_ERR_INVAL;
	if (chain->bound) return BRIDGE_HW_ERR_INVAL; /* mutation after bind */

	adc_dsp_stage_t *st = &chain->stages[stage_index];

	if (chunk_offset == 0u) {
		/* First chunk of this stage.  Seed `kind` + `total_size`;
         * any subsequent chunks must agree with these values so a
         * mid-upload re-target of the stage is caught as INVAL. */
		st->kind           = kind;
		st->total_size     = chunk_total_size;
		st->bytes_received = 0u;
		st->complete       = false;
	} else {
		/* Continuation chunk.  The host must keep the same kind +
         * total_size as the first chunk of this (chain, stage)
         * pair -- otherwise the buffer would be a mix of two
         * different stage payloads. */
		if (st->total_size == 0u) return BRIDGE_HW_ERR_INVAL; /* stage not yet opened */
		if (st->kind != kind) return BRIDGE_HW_ERR_INVAL;
		if (st->total_size != chunk_total_size) return BRIDGE_HW_ERR_INVAL;
		if (st->complete) return BRIDGE_HW_ERR_INVAL; /* already done */
	}

	for (size_t i = 0u; i < chunk_data_len; ++i) {
		st->data[chunk_offset + i] = chunk_data[i];
	}
	st->bytes_received += (uint16_t)chunk_data_len;
	if (st->bytes_received == st->total_size) {
		st->complete = true;
	}
	return BRIDGE_HW_OK;
}

/* Is a FIR/IIR (filter, not FFT) chain already bound to some OTHER
 * stream?  The single FAC can serve only one at a time. */
bool adc_dsp_filter_stream_busy(uint8_t except_stream)
{
	for (uint8_t i = 0u; i < BRIDGE_ADC_STREAM_COUNT; ++i) {
		if (i == except_stream) continue;
		if (adc_streams[i].in_use && adc_streams[i].dsp_bound &&
		    adc_streams[i].dsp_terminal != 3u /* not FFT */) {
			return true;
		}
	}
	return false;
}

/* Is an FFT-terminal chain already bound to some OTHER stream?  Like
 * the FAC, the FFT block is single-instance (adc_stream.c's own
 * header comment: "Like the FAC there is ONE block, so ONE FFT stream
 * at a time") -- nothing enforced that until #70, which let two
 * streams both bind an FFT chain and neither ever complete a frame. */
bool adc_dsp_fft_stream_busy(uint8_t except_stream)
{
	for (uint8_t i = 0u; i < BRIDGE_ADC_STREAM_COUNT; ++i) {
		if (i == except_stream) continue;
		if (adc_streams[i].in_use && adc_streams[i].dsp_bound &&
		    adc_streams[i].dsp_terminal == 3u /* FFT */) {
			return true;
		}
	}
	return false;
}

/* The shared "does the P1 runtime support this chain's shape"
 * predicate (#69, #70) -- see the doc comment in adc_dsp_chain.h for
 * why this is ONE function called from both chain_bind (below) and
 * adc_stream.c's pump-side config functions, not two copies.
 *
 * Assumes the chain already passed chain_bind's structural checks
 * (every populated stage complete + blob-valid, no gaps, FFT terminal
 * with WINDOW immediately preceding it if present); this only judges
 * realisability under P1's capability limits. */
bool adc_dsp_chain_p1_capable(const adc_dsp_chain_t *chain)
{
	uint8_t populated = 0u;
	uint8_t fft_index = BRIDGE_DSP_MAX_STAGES;

	for (uint8_t i = 0u; i < BRIDGE_DSP_MAX_STAGES; ++i) {
		const adc_dsp_stage_t *st = &chain->stages[i];
		if (st->total_size == 0u) continue;
		++populated;
		if (st->kind == 3u /* FFT */) fft_index = i;
	}

	/* #132: the payload check belongs HERE, not only in chain_bind.
     * adc_stream.c's pump-side config functions (adc_dsp_fac_config,
     * adc_dsp_fft_config) re-check ONLY this predicate as their
     * defence-in-depth guard, so a per-kind payload field they then
     * decode -- the FIR tap count above all -- was never re-validated
     * on that path.  adc_dsp_fac_config sizes `taps[]` from
     * BRIDGE_DSP_MAX_FIR_TAPS but indexes it with the raw `n_taps`
     * byte; with the bound living only in chain_bind's separate
     * blob_valid loop, any future path that reaches the pump without
     * having gone through THIS predicate's caller overflows a 2 KB
     * stack shared with two ISRs' reply buffers, silently (no MSPLIM,
     * no painting, no linker ASSERT).  Folding blob_valid in makes
     * "re-checking this predicate" actually mean what adc_stream.c's
     * comment already claims it means -- and it closes the IIR
     * `data[1]` and FFT `out_fmt` reads on the same path for free.
     *
     * Cheap and idempotent: chain_bind runs this a second time after
     * its own blob_valid loop, over at most 4 populated stages. */
	for (uint8_t i = 0u; i < BRIDGE_DSP_MAX_STAGES; ++i) {
		if (chain->stages[i].total_size == 0u) continue;
		if (!adc_dsp_stage_blob_valid(&chain->stages[i])) return false;
	}

	if (fft_index != BRIDGE_DSP_MAX_STAGES) {
		/* FFT-terminal chain: P1's FFT block has no upstream filter --
         * only WINDOW may sit ahead of the transform (mirrors the old
         * adc_dsp_fft_config() rejection at what was adc_stream.c:1015,
         * "FIR/IIR before an FFT: not a P1 spectrum chain"). */
		for (uint8_t i = 0u; i < BRIDGE_DSP_MAX_STAGES; ++i) {
			const adc_dsp_stage_t *st = &chain->stages[i];
			if (st->total_size == 0u) continue;
			if (st->kind != 3u /* FFT */ && st->kind != 2u /* WINDOW */) return false;
		}
		/* F3 (review of #69/#70): the FFT point-count bound is a P1
         * REALISABILITY limit too -- fold it into this predicate rather
         * than leaving it solely to adc_dsp_stage_blob_valid()'s
         * structural check (still run first, at bind, as the wire's
         * own well-formedness gate) and adc_stream.c's independent
         * ADC_DSP_FFT_MAX_POINTS re-check (now a compile-time alias of
         * BRIDGE_DSP_MAX_FFT_POINTS below, not a second copy of the
         * number).  chain_bind's structural loop already guarantees
         * fft_st->total_size >= BRIDGE_DSP_STAGE_HDR_BYTES, so the
         * header-byte read here is in-bounds. */
		const adc_dsp_stage_t *fft_st = &chain->stages[fft_index];
		const uint16_t n_points = (uint16_t)(fft_st->data[0] | ((uint16_t)fft_st->data[1] << 8));
		if (n_points < BRIDGE_DSP_MIN_FFT_POINTS || n_points > BRIDGE_DSP_MAX_FFT_POINTS) {
			return false;
		}
		return true;
	}

	/* Non-FFT (filter) terminal: P1's FAC handles exactly one
     * populated stage (mirrors adc_dsp_fac_config's old
     * `populated != 1u` check), and a populated IIR stage must be a
     * single biquad section (mirrors its old `n_sec != 1u` check). */
	if (populated != 1u) return false;
	for (uint8_t i = 0u; i < BRIDGE_DSP_MAX_STAGES; ++i) {
		const adc_dsp_stage_t *st = &chain->stages[i];
		if (st->total_size == 0u) continue;
		if (st->kind == 1u /* IIR */) {
			const uint8_t n_sec = st->data[1]; /* format:u8 n_sections:u8 ... */
			if (n_sec != 1u) return false;
		}
	}
	return true;
}

int bridge_hw_adc_dsp_chain_bind(uint8_t chain_id, uint8_t stream_id)
{
	if (chain_id >= BRIDGE_DSP_MAX_CHAINS) return BRIDGE_HW_ERR_RANGE;
	if (stream_id >= BRIDGE_ADC_STREAM_COUNT) return BRIDGE_HW_ERR_RANGE;

	adc_dsp_chain_t *chain = &adc_dsp_chains[chain_id];
	if (!chain->in_use) return BRIDGE_HW_ERR_INVAL;
	if (chain->bound) return BRIDGE_HW_ERR_INVAL; /* already attached */

	/* Validate every populated stage is complete + the chain
     * follows the ordering rules documented in
     * `bridge_hw_adc_dsp_chain_bind`'s contract:
     *   - FFT must be the terminal stage (no stage after it),
     *   - WINDOW must immediately precede FFT,
     *   - empty stages (total_size == 0) are allowed only at
     *     contiguous tail positions -- not interleaved with
     *     populated stages. */
	uint8_t fft_index            = BRIDGE_DSP_MAX_STAGES;
	uint8_t window_index         = BRIDGE_DSP_MAX_STAGES;
	uint8_t last_populated_index = BRIDGE_DSP_MAX_STAGES;
	for (uint8_t i = 0u; i < BRIDGE_DSP_MAX_STAGES; ++i) {
		adc_dsp_stage_t *st = &chain->stages[i];
		if (st->total_size == 0u) continue;
		if (!st->complete) return BRIDGE_HW_ERR_INVAL; /* mid-upload */
		/* Payload well-formed for its kind?  stage_push checked only the
         * byte count + kind range; this is where a malformed FIR/IIR/
         * WINDOW/FFT blob is caught, before it can mis-program the HW. */
		if (!adc_dsp_stage_blob_valid(st)) return BRIDGE_HW_ERR_INVAL;
		if (last_populated_index != BRIDGE_DSP_MAX_STAGES &&
		    (uint8_t)(i - last_populated_index) != 1u) {
			return BRIDGE_HW_ERR_INVAL; /* gap in stage list */
		}
		last_populated_index = i;
		if (st->kind == 3u /* FFT */) {
			if (fft_index != BRIDGE_DSP_MAX_STAGES) return BRIDGE_HW_ERR_INVAL;
			fft_index = i;
		} else if (st->kind == 2u /* WINDOW */) {
			if (window_index != BRIDGE_DSP_MAX_STAGES) return BRIDGE_HW_ERR_INVAL;
			window_index = i;
		}
	}
	if (last_populated_index == BRIDGE_DSP_MAX_STAGES) {
		return BRIDGE_HW_ERR_INVAL; /* empty chain */
	}
	if (fft_index != BRIDGE_DSP_MAX_STAGES) {
		/* FFT must be terminal -- no populated stage after it. */
		if (fft_index != last_populated_index) return BRIDGE_HW_ERR_INVAL;
		/* WINDOW (if present) must directly precede the FFT. */
		if (window_index != BRIDGE_DSP_MAX_STAGES &&
		    (fft_index == 0u || window_index != fft_index - 1u)) {
			return BRIDGE_HW_ERR_INVAL;
		}
	} else if (window_index != BRIDGE_DSP_MAX_STAGES) {
		/* WINDOW without a terminating FFT has no defined meaning in
         * the filtered-samples path -- reject per alp-sdk
         * docs/gd32-bridge-protocol.md §3.x. */
		return BRIDGE_HW_ERR_INVAL;
	}

	/* Stream validity BEFORE capability (F5, second review pass on
     * #69/#70): a non-existent or not-yet-running stream is the more
     * fundamental error and must win over "this chain's shape is
     * unrealisable" -- check it here, ahead of the capability gate
     * below, rather than after it. */
	adc_stream_state_t *s = &adc_streams[stream_id];
	if (!s->in_use) return BRIDGE_HW_ERR_INVAL;   /* stream not running */
	if (s->dsp_bound) return BRIDGE_HW_ERR_INVAL; /* stream already has a chain */

	/* Chain shape is STRUCTURALLY valid.  Whether P1 can actually
     * realise it is a separate, narrower question (#69) -- a
     * well-formed chain can still be beyond the FAC's single-stage /
     * single-biquad limit or ahead of a WINDOW+FFT terminal.  Refuse
     * those here, at bind, rather than letting them through to the
     * pump where the refusal used to be silently swallowed.
     *
     * F1 (review of #69/#70): release the slot back to the pool on
     * this refusal.  There is no host-facing close opcode -- see
     * adc_dsp_chain_release's doc comment -- so a chain a conforming
     * host abandons after NOTIMPL would otherwise never come back:
     * BRIDGE_DSP_MAX_CHAINS refused-and-abandoned capability binds
     * exhaust the whole pool and chain_open then answers NOTIMPL
     * forever (reboot-only recovery -- a bus-reachable permanent DoS
     * of the DSP feature).  Safe to release unconditionally here:
     * P1-capability is a property of the CHAIN's shape alone, not of
     * the stream_id it was bound against, so this chain can never
     * become realisable by retrying against a different stream --
     * releasing it costs no legitimate retry path. */
	if (!adc_dsp_chain_p1_capable(chain)) {
		adc_dsp_chain_release(chain_id);
		return BRIDGE_HW_ERR_NOTIMPL;
	}

	/* One FAC block -> one filter (FIR/IIR) stream at a time; one FFT
     * block -> one FFT stream at a time (#70 -- the FFT terminal used
     * to be exempted from any busy check on the wrong premise that
     * the FFT block wasn't itself single-instance; it is).
     *
     * Deliberately NOT released on either busy refusal below (unlike
     * the capability refusal above): this chain IS realisable, just
     * contended right now -- a host that waits for the other stream's
     * stream_end and retries the SAME chain_id is the expected,
     * supported recovery path, and releasing here would invalidate
     * that chain_id out from under a legitimately-retrying host. */
	const uint8_t terminal_kind = chain->stages[last_populated_index].kind;
	if (terminal_kind == 3u /* FFT */) {
		if (adc_dsp_fft_stream_busy(stream_id)) {
			return BRIDGE_HW_ERR_NOTIMPL; /* FFT already serving another stream */
		}
	} else {
		if (adc_dsp_filter_stream_busy(stream_id)) {
			return BRIDGE_HW_ERR_NOTIMPL; /* FAC already serving another stream */
		}
	}

	/* Attachment is a state flip on both halves.  The terminal stage
     * kind decides the data plane: FIR/IIR -> the base-level pump
     * filters raw samples through the FAC into this stream's processed
     * ring and stream_read serves filtered mV; FFT -> stream_read
     * answers NOSUPPORT (spectrum is read via CMD_ADC_SPECTRUM_READ).
     * Reset the processed-ring cursors so the pump starts clean. */
	s->dsp_terminal  = chain->stages[last_populated_index].kind;
	s->proc_write    = 0u;
	s->proc_read     = 0u;
	s->pump_raw_read = s->total_read; /* pump picks up where the raw reader is */
	s->dsp_chain_id  = chain_id;
	s->dsp_bound     = true;
	chain->bound     = true;
	return BRIDGE_HW_OK;
}
