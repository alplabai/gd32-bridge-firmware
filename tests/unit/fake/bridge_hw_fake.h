/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * TEST-ONLY fake implementation of hal/bridge_hw.h.  Lives under
 * tests/unit/, NOT under hal/ -- it must have no chance of being picked
 * up by a firmware build (CMakeLists.txt in the top-level tree never
 * globs this directory; only tests/unit/CMakeLists.txt references
 * these files by explicit path).
 *
 * hal/bridge_hw_stub.c answers BRIDGE_HW_ERR_NOTIMPL for essentially
 * every call, so only PING-class opcodes can ever reach STATUS_OK
 * through it.  This fake instead gives every bridge_hw_* symbol a small
 * resettable, host-side model (GPIO pad levels, PWM per-channel
 * records, an ADC sample queue, ...) PLUS a per-function forced-return
 * override, so a test can drive protocol_dispatch() through both its
 * success path and every BRIDGE_HW_ERR_* -> STATUS_* mapping the real
 * firmware exercises against real silicon.
 *
 * State model, one line each:
 *   - Reset reason: one uint8_t, DESTRUCTIVE READ -- bridge_hw_reset_reason()
 *                   returns the captured value and latches it to 0
 *                   (UNKNOWN), mirroring the CURRENT documented contract
 *                   (hal/bridge_hw.h:60-62, hal/gd32/init.c:440,
 *                   src/protocol.c:139-140).  gh#56 (open) proposes
 *                   replacing this with a boot-snapshot, idempotent-within-
 *                   a-boot contract instead; if/when #56 lands, this fake
 *                   and this comment must flip in the SAME change, not
 *                   silently drift from whichever bridge_hw.h says.
 *   - GPIO:        one 32-bit pad-level word (bridge_hw_gpio_write sets
 *                   under mask; bridge_hw_gpio_read reads back under mask).
 *   - PWM:         one { duty_ns, align_mode, dead_time_ns, break_cfg,
 *                   capture edge/period/pulse } record per channel slot,
 *                   PLUS one period_ns per TIMER GROUP (channels 0-3 share
 *                   one period, channels 4-7 share another -- mirrors
 *                   hal/bridge_hw.h:87 "the period is shared per underlying
 *                   timer" and hal/gd32/pwm.c's TIMER0/TIMER7 grouping).
 *   - ADC:         one FIFO queue of millivolt samples (bridge_hw_adc_read
 *                   drains up to `samples` from the front, zero-padding
 *                   once the queue empties) + a captured last-configure
 *                   call.
 *   - ADC stream:  one small ring per stream_id (begin/read/end), plus a
 *                   spectrum-frame model (seq, total_bins, bins[]).
 *   - QENC:        one int32_t position per encoder (reset zeroes it).
 *   - Counter:     one uint32_t tick value per counter.
 *   - TRNG:        deterministic fill -- dest[i] = (uint8_t)(seed + i),
 *                   seed settable, default 0.
 *   - DAC:         one uint16_t mV latch per channel.
 *   - TMU:         captured last (function, format, in_a, in_b); result
 *                   defaults to in_a (identity) and is overridable.
 *   - DA9292:      one settable status byte (default 0xFF, matching the
 *                   real "no sample available" sentinel).
 *   - SE reset:    captured last `assert` argument + a call count; an
 *                   out-of-range `assert` (>1) returns BRIDGE_HW_ERR_INVAL
 *                   WITHOUT touching either, mirroring hal/gd32/se_reset.c:73
 *                   (protocol.c:277 pre-rejects assert>1 before the HAL is
 *                   ever reached through protocol_dispatch(), so only a
 *                   direct bridge_hw_se_reset() call exercises this).
 *   - Power mode:  captured last (mode, wake_bitmap, wake_after_ms).
 *   - DSP chain:   settable next chain_id; chain_bind is pure forced-return
 *                   (no shape validation -- see the #69/#70 note in
 *                   test_protocol.c's file header for why this fixture
 *                   deliberately does not model the FAC/FFT bind-time
 *                   acceptance rules).  stage_push CAPTURES its last call's
 *                   arguments (chain_id/stage_index/kind/chunk_offset/
 *                   chunk_total_size/chunk_data) for a test to assert
 *                   against -- that capture is plain argument-decode
 *                   coverage, orthogonal to the still-unmodelled #69/#70
 *                   shape rules.
 *
 * Every one of the above resets to its zero/default value from
 * bridge_hw_fake_reset(void), which every ZTEST case in test_protocol.c
 * calls FIRST (there is no ztest before/after hook in the shim -- see
 * tests/unit/ztest_shim.h -- so this reset is manual, not automatic).
 * protocol.c's own `static uint8_t link_features` is NOT part of this
 * fake and is NOT reset by it: it lives in the unit under test, not the
 * HAL, and this fixture has no seam to reach it.  Tests that care about
 * link_features re-arm it explicitly via a CMD_LINK_FEATURES request
 * (0 = disable, idempotent) rather than relying on a fresh-process
 * default.
 */

#ifndef GD32_BRIDGE_TEST_FAKE_BRIDGE_HW_FAKE_H
#define GD32_BRIDGE_TEST_FAKE_BRIDGE_HW_FAKE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* One slot per bridge_hw_* entry point that returns an `int` status
 * (BRIDGE_HW_OK / BRIDGE_HW_ERR_*).  bridge_hw_reset_reason() and
 * bridge_hw_da9292_status_cached() return a bare uint8_t with no error
 * channel, so they are not in this list -- see the dedicated
 * bridge_hw_fake_set_reset_reason() / bridge_hw_fake_set_da9292_status()
 * below instead. */
typedef enum {
	FAKE_FN_GPIO_READ = 0,
	FAKE_FN_GPIO_WRITE,
	FAKE_FN_PWM_SET,
	FAKE_FN_PWM_GET,
	FAKE_FN_PWM_CONFIGURE,
	FAKE_FN_ADC_READ,
	FAKE_FN_ADC_CONFIGURE,
	FAKE_FN_ADC_STREAM_BEGIN,
	FAKE_FN_ADC_STREAM_READ,
	FAKE_FN_ADC_STREAM_END,
	FAKE_FN_ADC_SPECTRUM_READ,
	FAKE_FN_TRNG_READ,
	FAKE_FN_TMU_COMPUTE,
	FAKE_FN_DAC_SET,
	FAKE_FN_DAC_GET,
	FAKE_FN_QENC_READ,
	FAKE_FN_QENC_RESET,
	FAKE_FN_COUNTER_READ,
	FAKE_FN_SE_RESET,
	FAKE_FN_PWM_CAPTURE_BEGIN,
	FAKE_FN_PWM_CAPTURE_READ,
	FAKE_FN_PWM_CAPTURE_END,
	FAKE_FN_PWM_SINGLE_PULSE,
	FAKE_FN_TIMER_SYNC,
	FAKE_FN_POWER_MODE_SET,
	FAKE_FN_ADC_DSP_CHAIN_OPEN,
	FAKE_FN_ADC_DSP_STAGE_PUSH,
	FAKE_FN_ADC_DSP_CHAIN_BIND,
	FAKE_FN_COUNT,
} bridge_hw_fake_fn_t;

/* Zeroes every subsystem model AND every forced-return slot back to
 * BRIDGE_HW_OK (i.e. "run the normal fake logic").  Call this FIRST in
 * every ZTEST case that touches the fake, directly or via
 * protocol_dispatch() -- the shim runs every case in one process with
 * no isolation between them. */
void bridge_hw_fake_reset(void);

/* Forces the NEXT (and every subsequent, until changed) call to `fn`
 * to return `rv` without touching that function's state model.
 * `rv == BRIDGE_HW_OK` (0) clears the override and resumes normal fake
 * behaviour -- it is never itself an "error" a test needs to force. */
void bridge_hw_fake_force(bridge_hw_fake_fn_t fn, int rv);

/* --------------------------------------------------------------- */
/* Reset-cause + DA9292 -- no error channel, so a plain setter each. */
/* --------------------------------------------------------------- */

/* Arms the value the NEXT bridge_hw_reset_reason() call returns.  That
 * call is DESTRUCTIVE (see this header's state-model block and gh#56) --
 * it latches back to 0 (UNKNOWN) as it returns, so a SECOND call without
 * an intervening bridge_hw_fake_set_reset_reason() reports 0, not the
 * value seeded here. */
void bridge_hw_fake_set_reset_reason(uint8_t reason);
void bridge_hw_fake_set_da9292_status(uint8_t status);

/* --------------------------------------------------------------- */
/* GPIO -- one pad-level word.                                       */
/* --------------------------------------------------------------- */

void     bridge_hw_fake_gpio_set_pads(uint32_t levels);
uint32_t bridge_hw_fake_gpio_get_pads(void);

/* --------------------------------------------------------------- */
/* PWM -- per-channel record, period shared per timer group.          */
/* GD32_BRIDGE_FAKE_PWM_CHANNELS mirrors the V2N map (PWM0..7);       */
/* out-of-range channel ids are accepted by the fake and WRAP (modulo */
/* GD32_BRIDGE_FAKE_PWM_CHANNELS) rather than clamping to slot 0 --   */
/* range validation is the real backend's job, not this fixture's:   */
/* hal/gd32/pwm.c returns BRIDGE_HW_ERR_RANGE for                    */
/* channel >= PWM_CHANNEL_COUNT (gd32_common.h: 8u), which this fake  */
/* does not reproduce.                                                */
/* --------------------------------------------------------------- */

#define GD32_BRIDGE_FAKE_PWM_CHANNELS 8u

/* Channels this many apart share one timer (mirrors hal/gd32/pwm.c's
 * TIMER0 (PWM0..3) / TIMER7 (PWM4..7) grouping) and therefore one
 * period_ns -- see bridge_hw_fake_pwm_get_state's period_ns output. */
#define GD32_BRIDGE_FAKE_PWM_CHANNELS_PER_TIMER 4u

/* Seeds channel `ch`'s reported PWM_GET boot default (period/duty) --
 * bridge_hw_pwm_get documents 65.536 ms / 0 ns before the first
 * bridge_hw_pwm_set, which bridge_hw_fake_reset() already establishes;
 * this setter exists for tests that want a DIFFERENT starting point
 * without going through PWM_SET first.  `period_ns` lands on `ch`'s whole
 * TIMER GROUP (see GD32_BRIDGE_FAKE_PWM_CHANNELS_PER_TIMER), so seeding
 * one channel's period changes what its group siblings report too --
 * `duty_ns` stays per-channel. */
void bridge_hw_fake_pwm_seed(uint8_t ch, uint32_t period_ns, uint32_t duty_ns);
void bridge_hw_fake_pwm_get_state(uint8_t   ch,
                                  uint32_t *period_ns,
                                  uint32_t *duty_ns,
                                  uint8_t  *align_mode,
                                  uint32_t *dead_time_ns,
                                  uint8_t  *break_cfg);

/* PWM input-capture readback the fake will hand back on the next
 * bridge_hw_pwm_capture_read for `ch` (independent of capture_begin --
 * this fixture does not model the begin/end state machine, only the
 * status-code seam CMD_PWM_CAPTURE_* dispatch through). */
void bridge_hw_fake_pwm_capture_seed(uint8_t ch, uint32_t period_ns, uint32_t pulse_width_ns);

/* --------------------------------------------------------------- */
/* ADC -- a FIFO queue of mV samples shared by bridge_hw_adc_read.    */
/* --------------------------------------------------------------- */

#define GD32_BRIDGE_FAKE_ADC_QUEUE_CAP 64u

/* Appends one sample; asserts (via a return-value the caller should
 * check with a plain if, not a zassert -- this header has no ztest
 * dependency) rather than silently dropping on overflow. */
int  bridge_hw_fake_adc_queue_push(uint16_t mv);
void bridge_hw_fake_adc_get_last_configure(uint8_t  *channel,
                                           uint16_t *oversample_ratio,
                                           uint16_t *sample_cycles,
                                           uint8_t  *resolution_bits);

/* --------------------------------------------------------------- */
/* ADC streaming + spectrum.                                         */
/* --------------------------------------------------------------- */

/* Queues one raw sample for `stream_id`'s next bridge_hw_adc_stream_read
 * drain.  Independent of stream_begin/end bookkeeping (this fixture
 * does not model ADC/DMA ownership -- only the status-code seam). */
int bridge_hw_fake_adc_stream_queue_push(uint8_t stream_id, uint16_t mv);

/* Forces the NEXT bridge_hw_adc_stream_read for `stream_id` to report
 * `got` regardless of how many samples are actually queued or what
 * `max_samples` the caller asked for -- the deliberate HAL-contract
 * violation protocol.c's `if (got > max_samples) return STATUS_IO;`
 * guards against (src/protocol.c:501). */
void bridge_hw_fake_adc_stream_force_got(uint8_t stream_id, uint8_t got);

/* Fixture-local bin storage cap, independent of protocol.h's
 * GD32_BRIDGE_ADC_SPECTRUM_READ_MAX (this file has no protocol.h
 * dependency by design -- it only promises the hal/bridge_hw.h
 * contract) but sized >= it so every wire-reachable request fits. */
#define GD32_BRIDGE_FAKE_SPECTRUM_BINS_CAP 32u

/* Seeds the frame bridge_hw_adc_spectrum_read serves for `stream_id`:
 * `seq`/`total_bins` are echoed verbatim; up to
 * GD32_BRIDGE_FAKE_SPECTRUM_BINS_CAP bins are copied from `bins`. */
void bridge_hw_fake_adc_spectrum_seed(uint8_t      stream_id,
                                      uint32_t     seq,
                                      uint16_t     total_bins,
                                      const float *bins,
                                      uint16_t     bins_len);

/* Same HAL-contract-violation override as the stream-read one above,
 * for handle_adc_spectrum_read's `if (got > max_bins) return
 * STATUS_IO;` (src/protocol.c:548). */
void bridge_hw_fake_adc_spectrum_force_got(uint8_t stream_id, uint8_t got);

/* --------------------------------------------------------------- */
/* TRNG -- deterministic fill dest[i] = (uint8_t)(seed + i).          */
/* --------------------------------------------------------------- */

void bridge_hw_fake_trng_set_seed(uint8_t seed);

/* --------------------------------------------------------------- */
/* TMU -- captured last call + overridable result.                   */
/* --------------------------------------------------------------- */

void bridge_hw_fake_tmu_set_result(uint32_t result);
void bridge_hw_fake_tmu_get_last_call(uint8_t  *function,
                                      uint8_t  *format,
                                      uint32_t *in_a,
                                      uint32_t *in_b);

/* --------------------------------------------------------------- */
/* DAC -- per-channel mV latch.                                      */
/* --------------------------------------------------------------- */

#define GD32_BRIDGE_FAKE_DAC_CHANNELS 4u

/* --------------------------------------------------------------- */
/* QENC + counter.                                                   */
/* --------------------------------------------------------------- */

#define GD32_BRIDGE_FAKE_QENC_COUNT    4u
#define GD32_BRIDGE_FAKE_COUNTER_COUNT 4u

void bridge_hw_fake_qenc_set_position(uint8_t encoder, int32_t position);
void bridge_hw_fake_counter_set_ticks(uint8_t counter, uint32_t ticks);

/* --------------------------------------------------------------- */
/* SE reset + power mode -- captured last-call arguments.             */
/* --------------------------------------------------------------- */

/* Returns the number of bridge_hw_se_reset() calls since the last
 * reset, and writes the most recent `assert` argument to *last_assert
 * (untouched if there has been no call yet). */
uint32_t bridge_hw_fake_se_reset_call_count(void);
uint8_t  bridge_hw_fake_se_reset_last_assert(void);

void bridge_hw_fake_power_mode_get_last_call(uint8_t  *mode,
                                             uint32_t *wake_bitmap,
                                             uint32_t *wake_after_ms);

/* --------------------------------------------------------------- */
/* DSP chain-open id + stage-push argument capture.                  */
/* --------------------------------------------------------------- */

void bridge_hw_fake_dsp_chain_set_next_id(uint8_t chain_id);

/* Fixture-local capture-buffer cap for bridge_hw_fake_dsp_stage_push_get_
 * last_call's chunk_data output -- independent of protocol.h (this file
 * has no protocol.h dependency by design, see the file header) but sized
 * comfortably above any wire-reachable chunk (protocol.h's
 * GD32_BRIDGE_MAX_PAYLOAD_BYTES is 65; a stage-push chunk is at most that
 * minus its 7-byte header). */
#define GD32_BRIDGE_FAKE_STAGE_PUSH_DATA_CAP 64u

/* Captures bridge_hw_adc_dsp_stage_push's last call so a test can assert
 * the decoded chain_id/stage_index/kind/chunk_offset/chunk_total_size and
 * chunk_data landed exactly as the request encoded them.  This is plain
 * argument-decode coverage (the class of bug this fixture's dead-accessor
 * review flagged); it is deliberately NOT the #69/#70 chain-shape
 * validation (see this header's state-model block and test_protocol.c's
 * file header), which stays undone.  *chunk_data_len_out is the call's
 * real length (0 if chunk_data was NULL); chunk_data_out receives up to
 * GD32_BRIDGE_FAKE_STAGE_PUSH_DATA_CAP bytes of it, truncated silently if
 * the real call was longer (no wire-reachable request is). */
void bridge_hw_fake_dsp_stage_push_get_last_call(uint8_t  *chain_id,
                                                 uint8_t  *stage_index,
                                                 uint8_t  *kind,
                                                 uint16_t *chunk_offset,
                                                 uint16_t *chunk_total_size,
                                                 uint8_t  *chunk_data_out,
                                                 size_t   *chunk_data_len_out);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /* GD32_BRIDGE_TEST_FAKE_BRIDGE_HW_FAKE_H */
