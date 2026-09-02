/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * TEST-ONLY fake for hal/bridge_hw.h -- see bridge_hw_fake.h for the
 * state model and why this lives under tests/unit/ rather than hal/.
 */

#include "bridge_hw_fake.h"

#include <string.h>

#include "../../../hal/bridge_hw.h"

#define FAKE_STREAM_COUNT 2u

/* ---- forced-return table ------------------------------------------- */

static int s_force_rv[FAKE_FN_COUNT];

void bridge_hw_fake_force(bridge_hw_fake_fn_t fn, int rv)
{
	s_force_rv[fn] = rv;
}

/* Returns true (and writes *out) when `fn` has a non-OK override
 * armed -- the one check every fake body below performs before
 * touching its state model. */
static int forced(bridge_hw_fake_fn_t fn, int *out)
{
	if (s_force_rv[fn] != BRIDGE_HW_OK) {
		*out = s_force_rv[fn];
		return 1;
	}
	return 0;
}

/* ---- reset-cause + DA9292 -------------------------------------------- */

static uint8_t s_reset_reason;
static uint8_t s_da9292_status;

void bridge_hw_fake_set_reset_reason(uint8_t reason)
{
	s_reset_reason = reason;
}

void bridge_hw_fake_set_da9292_status(uint8_t status)
{
	s_da9292_status = status;
}

/* ---- GPIO -------------------------------------------------------------- */

static uint32_t s_gpio_pads;

void bridge_hw_fake_gpio_set_pads(uint32_t levels)
{
	s_gpio_pads = levels;
}

uint32_t bridge_hw_fake_gpio_get_pads(void)
{
	return s_gpio_pads;
}

/* ---- PWM ----------------------------------------------------------------*/

typedef struct {
	uint32_t duty_ns;
	uint8_t  align_mode;
	uint32_t dead_time_ns;
	uint8_t  break_cfg;
	uint32_t capture_period_ns;
	uint32_t capture_pulse_ns;
} fake_pwm_t;

static fake_pwm_t s_pwm[GD32_BRIDGE_FAKE_PWM_CHANNELS];

/* Period is per TIMER GROUP, not per channel (hal/bridge_hw.h:87: "the
 * period is shared per underlying timer"; hal/gd32/pwm.c groups PWM0..3 on
 * TIMER0 and PWM4..7 on TIMER7).  GD32_BRIDGE_FAKE_PWM_CHANNELS_PER_TIMER
 * channels share one slot here. */
#define GD32_BRIDGE_FAKE_PWM_TIMER_GROUPS \
	(GD32_BRIDGE_FAKE_PWM_CHANNELS / GD32_BRIDGE_FAKE_PWM_CHANNELS_PER_TIMER)
static uint32_t s_pwm_timer_period_ns[GD32_BRIDGE_FAKE_PWM_TIMER_GROUPS];

/* Boot default per bridge_hw_pwm_get's docblock: 65.536 ms period, 0 ns
 * duty, before the first bridge_hw_pwm_set. */
#define FAKE_PWM_BOOT_PERIOD_NS 65536000u

/* `ch` WRAPS (modulo GD32_BRIDGE_FAKE_PWM_CHANNELS) rather than clamping
 * to slot 0 -- see bridge_hw_fake.h's PWM section.  The real backend
 * (hal/gd32/pwm.c) instead returns BRIDGE_HW_ERR_RANGE for
 * channel >= PWM_CHANNEL_COUNT; this fixture does not reproduce that. */
static uint8_t pwm_slot(uint8_t ch)
{
	return (uint8_t)(ch % GD32_BRIDGE_FAKE_PWM_CHANNELS);
}

static uint8_t pwm_timer_group(uint8_t slot)
{
	return (uint8_t)(slot / GD32_BRIDGE_FAKE_PWM_CHANNELS_PER_TIMER);
}

void bridge_hw_fake_pwm_seed(uint8_t ch, uint32_t period_ns, uint32_t duty_ns)
{
	const uint8_t slot                           = pwm_slot(ch);
	s_pwm_timer_period_ns[pwm_timer_group(slot)] = period_ns;
	s_pwm[slot].duty_ns                          = duty_ns;
}

void bridge_hw_fake_pwm_get_state(uint8_t   ch,
                                  uint32_t *period_ns,
                                  uint32_t *duty_ns,
                                  uint8_t  *align_mode,
                                  uint32_t *dead_time_ns,
                                  uint8_t  *break_cfg)
{
	const uint8_t     slot = pwm_slot(ch);
	const fake_pwm_t *p    = &s_pwm[slot];
	if (period_ns) *period_ns = s_pwm_timer_period_ns[pwm_timer_group(slot)];
	if (duty_ns) *duty_ns = p->duty_ns;
	if (align_mode) *align_mode = p->align_mode;
	if (dead_time_ns) *dead_time_ns = p->dead_time_ns;
	if (break_cfg) *break_cfg = p->break_cfg;
}

void bridge_hw_fake_pwm_capture_seed(uint8_t ch, uint32_t period_ns, uint32_t pulse_width_ns)
{
	fake_pwm_t *p        = &s_pwm[pwm_slot(ch)];
	p->capture_period_ns = period_ns;
	p->capture_pulse_ns  = pulse_width_ns;
}

/* ---- ADC single-shot ------------------------------------------------- */

static uint16_t s_adc_queue[GD32_BRIDGE_FAKE_ADC_QUEUE_CAP];
static size_t   s_adc_queue_len;
static size_t   s_adc_queue_head;

static struct {
	uint8_t  channel;
	uint16_t oversample_ratio;
	uint16_t sample_cycles;
	uint8_t  resolution_bits;
} s_adc_last_configure;

int bridge_hw_fake_adc_queue_push(uint16_t mv)
{
	if (s_adc_queue_len >= GD32_BRIDGE_FAKE_ADC_QUEUE_CAP) return -1;
	s_adc_queue[(s_adc_queue_head + s_adc_queue_len) % GD32_BRIDGE_FAKE_ADC_QUEUE_CAP] = mv;
	s_adc_queue_len++;
	return 0;
}

static uint16_t adc_queue_pop_or_zero(void)
{
	if (s_adc_queue_len == 0u) return 0u;
	const uint16_t v = s_adc_queue[s_adc_queue_head];
	s_adc_queue_head = (s_adc_queue_head + 1u) % GD32_BRIDGE_FAKE_ADC_QUEUE_CAP;
	s_adc_queue_len--;
	return v;
}

void bridge_hw_fake_adc_get_last_configure(uint8_t  *channel,
                                           uint16_t *oversample_ratio,
                                           uint16_t *sample_cycles,
                                           uint8_t  *resolution_bits)
{
	if (channel) *channel = s_adc_last_configure.channel;
	if (oversample_ratio) *oversample_ratio = s_adc_last_configure.oversample_ratio;
	if (sample_cycles) *sample_cycles = s_adc_last_configure.sample_cycles;
	if (resolution_bits) *resolution_bits = s_adc_last_configure.resolution_bits;
}

/* ---- ADC streaming + spectrum ------------------------------------------ */

typedef struct {
	uint16_t queue[GD32_BRIDGE_FAKE_ADC_QUEUE_CAP];
	size_t   len;
	size_t   head;
	int      force_got_armed;
	uint8_t  force_got;
} fake_adc_stream_t;

typedef struct {
	uint32_t seq;
	uint16_t total_bins;
	uint16_t bins_len;
	float    bins[GD32_BRIDGE_FAKE_SPECTRUM_BINS_CAP];
	int      force_got_armed;
	uint8_t  force_got;
} fake_adc_spectrum_t;

static fake_adc_stream_t   s_stream[FAKE_STREAM_COUNT];
static fake_adc_spectrum_t s_spectrum[FAKE_STREAM_COUNT];

/* WRAPS (modulo FAKE_STREAM_COUNT) -- see dac_slot() above.  protocol.c's
 * own `stream_id >= GD32_BRIDGE_ADC_STREAM_COUNT` local check pre-rejects
 * out-of-range ids before any dispatch-driven call reaches here, so this
 * only matters for a test calling the fake's HAL entry points directly. */
static uint8_t stream_slot(uint8_t stream_id)
{
	return (uint8_t)(stream_id % FAKE_STREAM_COUNT);
}

static uint16_t stream_pop_or_zero(fake_adc_stream_t *s)
{
	if (s->len == 0u) return 0u;
	const uint16_t v = s->queue[s->head];
	s->head          = (s->head + 1u) % GD32_BRIDGE_FAKE_ADC_QUEUE_CAP;
	s->len--;
	return v;
}

int bridge_hw_fake_adc_stream_queue_push(uint8_t stream_id, uint16_t mv)
{
	fake_adc_stream_t *s = &s_stream[stream_slot(stream_id)];
	if (s->len >= GD32_BRIDGE_FAKE_ADC_QUEUE_CAP) return -1;
	s->queue[(s->head + s->len) % GD32_BRIDGE_FAKE_ADC_QUEUE_CAP] = mv;
	s->len++;
	return 0;
}

void bridge_hw_fake_adc_stream_force_got(uint8_t stream_id, uint8_t got)
{
	fake_adc_stream_t *s = &s_stream[stream_slot(stream_id)];
	s->force_got_armed   = 1;
	s->force_got         = got;
}

void bridge_hw_fake_adc_spectrum_seed(uint8_t      stream_id,
                                      uint32_t     seq,
                                      uint16_t     total_bins,
                                      const float *bins,
                                      uint16_t     bins_len)
{
	fake_adc_spectrum_t *sp = &s_spectrum[stream_slot(stream_id)];
	uint16_t             n  = bins_len;
	if (n > GD32_BRIDGE_FAKE_SPECTRUM_BINS_CAP) n = GD32_BRIDGE_FAKE_SPECTRUM_BINS_CAP;
	sp->seq        = seq;
	sp->total_bins = total_bins;
	sp->bins_len   = n;
	if (n > 0u && bins != NULL) {
		memcpy(sp->bins, bins, (size_t)n * sizeof(bins[0]));
	}
}

void bridge_hw_fake_adc_spectrum_force_got(uint8_t stream_id, uint8_t got)
{
	fake_adc_spectrum_t *sp = &s_spectrum[stream_slot(stream_id)];
	sp->force_got_armed     = 1;
	sp->force_got           = got;
}

/* ---- TRNG -------------------------------------------------------------- */

static uint8_t s_trng_seed;

void bridge_hw_fake_trng_set_seed(uint8_t seed)
{
	s_trng_seed = seed;
}

/* ---- TMU ----------------------------------------------------------------*/

static struct {
	uint8_t  function;
	uint8_t  format;
	uint32_t in_a;
	uint32_t in_b;
} s_tmu_last;

static uint32_t s_tmu_result;
static int      s_tmu_result_armed;

void bridge_hw_fake_tmu_set_result(uint32_t result)
{
	s_tmu_result       = result;
	s_tmu_result_armed = 1;
}

void bridge_hw_fake_tmu_get_last_call(uint8_t  *function,
                                      uint8_t  *format,
                                      uint32_t *in_a,
                                      uint32_t *in_b)
{
	if (function) *function = s_tmu_last.function;
	if (format) *format = s_tmu_last.format;
	if (in_a) *in_a = s_tmu_last.in_a;
	if (in_b) *in_b = s_tmu_last.in_b;
}

/* ---- DAC ----------------------------------------------------------------*/

static uint16_t s_dac_mv[GD32_BRIDGE_FAKE_DAC_CHANNELS];

/* WRAPS (modulo GD32_BRIDGE_FAKE_DAC_CHANNELS), like pwm_slot() above --
 * range validation is the real backend's job, not this fixture's. */
static uint8_t dac_slot(uint8_t ch)
{
	return (uint8_t)(ch % GD32_BRIDGE_FAKE_DAC_CHANNELS);
}

/* ---- QENC + counter ----------------------------------------------------*/

static int32_t  s_qenc_position[GD32_BRIDGE_FAKE_QENC_COUNT];
static uint32_t s_counter_ticks[GD32_BRIDGE_FAKE_COUNTER_COUNT];

/* WRAPS (modulo GD32_BRIDGE_FAKE_QENC_COUNT) -- see dac_slot() above. */
static uint8_t qenc_slot(uint8_t e)
{
	return (uint8_t)(e % GD32_BRIDGE_FAKE_QENC_COUNT);
}

/* WRAPS (modulo GD32_BRIDGE_FAKE_COUNTER_COUNT) -- see dac_slot() above. */
static uint8_t counter_slot(uint8_t c)
{
	return (uint8_t)(c % GD32_BRIDGE_FAKE_COUNTER_COUNT);
}

void bridge_hw_fake_qenc_set_position(uint8_t encoder, int32_t position)
{
	s_qenc_position[qenc_slot(encoder)] = position;
}

void bridge_hw_fake_counter_set_ticks(uint8_t counter, uint32_t ticks)
{
	s_counter_ticks[counter_slot(counter)] = ticks;
}

/* ---- SE reset + power mode ---------------------------------------------*/

static uint32_t s_se_reset_calls;
static uint8_t  s_se_reset_last_assert;

uint32_t bridge_hw_fake_se_reset_call_count(void)
{
	return s_se_reset_calls;
}

uint8_t bridge_hw_fake_se_reset_last_assert(void)
{
	return s_se_reset_last_assert;
}

static struct {
	uint8_t  mode;
	uint32_t wake_bitmap;
	uint32_t wake_after_ms;
} s_power_last;

void bridge_hw_fake_power_mode_get_last_call(uint8_t  *mode,
                                             uint32_t *wake_bitmap,
                                             uint32_t *wake_after_ms)
{
	if (mode) *mode = s_power_last.mode;
	if (wake_bitmap) *wake_bitmap = s_power_last.wake_bitmap;
	if (wake_after_ms) *wake_after_ms = s_power_last.wake_after_ms;
}

/* ---- DSP chain-open id --------------------------------------------------*/

static uint8_t s_dsp_next_chain_id;

void bridge_hw_fake_dsp_chain_set_next_id(uint8_t chain_id)
{
	s_dsp_next_chain_id = chain_id;
}

/* ---- DSP stage-push argument capture ------------------------------------*/

static struct {
	uint8_t  chain_id;
	uint8_t  stage_index;
	uint8_t  kind;
	uint16_t chunk_offset;
	uint16_t chunk_total_size;
	uint8_t  chunk_data[GD32_BRIDGE_FAKE_STAGE_PUSH_DATA_CAP];
	size_t   chunk_data_len;
} s_dsp_stage_last;

void bridge_hw_fake_dsp_stage_push_get_last_call(uint8_t  *chain_id,
                                                 uint8_t  *stage_index,
                                                 uint8_t  *kind,
                                                 uint16_t *chunk_offset,
                                                 uint16_t *chunk_total_size,
                                                 uint8_t  *chunk_data_out,
                                                 size_t   *chunk_data_len_out)
{
	if (chain_id) *chain_id = s_dsp_stage_last.chain_id;
	if (stage_index) *stage_index = s_dsp_stage_last.stage_index;
	if (kind) *kind = s_dsp_stage_last.kind;
	if (chunk_offset) *chunk_offset = s_dsp_stage_last.chunk_offset;
	if (chunk_total_size) *chunk_total_size = s_dsp_stage_last.chunk_total_size;
	if (chunk_data_len_out) *chunk_data_len_out = s_dsp_stage_last.chunk_data_len;
	if (chunk_data_out && s_dsp_stage_last.chunk_data_len > 0u) {
		memcpy(chunk_data_out, s_dsp_stage_last.chunk_data, s_dsp_stage_last.chunk_data_len);
	}
}

/* ---- reset --------------------------------------------------------------*/

void bridge_hw_fake_reset(void)
{
	memset(s_force_rv, 0, sizeof(s_force_rv));

	s_reset_reason  = 0u;
	s_da9292_status = 0xFFu;

	s_gpio_pads = 0u;

	for (unsigned i = 0; i < GD32_BRIDGE_FAKE_PWM_CHANNELS; i++) {
		s_pwm[i].duty_ns           = 0u;
		s_pwm[i].align_mode        = 0u;
		s_pwm[i].dead_time_ns      = 0u;
		s_pwm[i].break_cfg         = 0u;
		s_pwm[i].capture_period_ns = 0u;
		s_pwm[i].capture_pulse_ns  = 0u;
	}
	for (unsigned g = 0; g < GD32_BRIDGE_FAKE_PWM_TIMER_GROUPS; g++) {
		s_pwm_timer_period_ns[g] = FAKE_PWM_BOOT_PERIOD_NS;
	}

	s_adc_queue_len  = 0u;
	s_adc_queue_head = 0u;
	memset(&s_adc_last_configure, 0, sizeof(s_adc_last_configure));

	memset(s_stream, 0, sizeof(s_stream));
	memset(s_spectrum, 0, sizeof(s_spectrum));

	s_trng_seed = 0u;

	memset(&s_tmu_last, 0, sizeof(s_tmu_last));
	s_tmu_result       = 0u;
	s_tmu_result_armed = 0;

	memset(s_dac_mv, 0, sizeof(s_dac_mv));
	memset(s_qenc_position, 0, sizeof(s_qenc_position));
	memset(s_counter_ticks, 0, sizeof(s_counter_ticks));

	s_se_reset_calls       = 0u;
	s_se_reset_last_assert = 0xFFu; /* not-yet-called sentinel */
	memset(&s_power_last, 0, sizeof(s_power_last));

	s_dsp_next_chain_id = 0u;
	memset(&s_dsp_stage_last, 0, sizeof(s_dsp_stage_last));
}

/* --------------------------------------------------------------- */
/* hal/bridge_hw.h implementation                                     */
/* --------------------------------------------------------------- */

uint8_t bridge_hw_reset_reason(void)
{
	/* DESTRUCTIVE READ, mirroring the CURRENT documented contract
	 * (hal/bridge_hw.h:60-62, hal/gd32/init.c:440, src/protocol.c:139-140,
	 * gh#56): capture, then latch back to 0 (UNKNOWN) so a second reader
	 * in the same boot sees UNKNOWN, not the same event twice. */
	const uint8_t v = s_reset_reason;
	s_reset_reason  = 0u;
	return v;
}

int bridge_hw_gpio_read(uint32_t mask, uint32_t *levels)
{
	int rv = 0;
	if (forced(FAKE_FN_GPIO_READ, &rv)) return rv;
	if (levels) *levels = s_gpio_pads & mask;
	return BRIDGE_HW_OK;
}

int bridge_hw_gpio_write(uint32_t mask, uint32_t levels)
{
	int rv = 0;
	if (forced(FAKE_FN_GPIO_WRITE, &rv)) return rv;
	s_gpio_pads = (s_gpio_pads & ~mask) | (levels & mask);
	return BRIDGE_HW_OK;
}

int bridge_hw_pwm_set(uint8_t channel, uint32_t period_ns, uint32_t duty_ns)
{
	int rv = 0;
	if (forced(FAKE_FN_PWM_SET, &rv)) return rv;
	const uint8_t slot                           = pwm_slot(channel);
	s_pwm_timer_period_ns[pwm_timer_group(slot)] = period_ns;
	s_pwm[slot].duty_ns                          = duty_ns;
	return BRIDGE_HW_OK;
}

int bridge_hw_pwm_get(uint8_t channel, uint32_t *period_ns, uint32_t *duty_ns)
{
	int rv = 0;
	if (forced(FAKE_FN_PWM_GET, &rv)) return rv;
	const uint8_t     slot = pwm_slot(channel);
	const fake_pwm_t *p    = &s_pwm[slot];
	if (period_ns) *period_ns = s_pwm_timer_period_ns[pwm_timer_group(slot)];
	if (duty_ns) *duty_ns = p->duty_ns;
	return BRIDGE_HW_OK;
}

int bridge_hw_pwm_configure(uint8_t  channel,
                            uint8_t  align_mode,
                            uint32_t dead_time_ns,
                            uint8_t  break_cfg)
{
	int rv = 0;
	if (forced(FAKE_FN_PWM_CONFIGURE, &rv)) return rv;
	fake_pwm_t *p   = &s_pwm[pwm_slot(channel)];
	p->align_mode   = align_mode;
	p->dead_time_ns = dead_time_ns;
	p->break_cfg    = break_cfg;
	return BRIDGE_HW_OK;
}

int bridge_hw_adc_read(uint8_t channel, uint8_t samples, uint16_t *mv)
{
	(void)channel;
	int rv = 0;
	if (forced(FAKE_FN_ADC_READ, &rv)) return rv;
	for (uint8_t i = 0; i < samples; i++) {
		const uint16_t v = adc_queue_pop_or_zero();
		if (mv) mv[i] = v;
	}
	return BRIDGE_HW_OK;
}

int bridge_hw_adc_configure(uint8_t  channel,
                            uint16_t oversample_ratio,
                            uint16_t sample_cycles,
                            uint8_t  resolution_bits)
{
	int rv = 0;
	if (forced(FAKE_FN_ADC_CONFIGURE, &rv)) return rv;
	s_adc_last_configure.channel          = channel;
	s_adc_last_configure.oversample_ratio = oversample_ratio;
	s_adc_last_configure.sample_cycles    = sample_cycles;
	s_adc_last_configure.resolution_bits  = resolution_bits;
	return BRIDGE_HW_OK;
}

int bridge_hw_adc_stream_begin(uint8_t stream_id, uint8_t channel, uint32_t sample_rate_hz)
{
	(void)channel;
	(void)sample_rate_hz;
	int rv = 0;
	if (forced(FAKE_FN_ADC_STREAM_BEGIN, &rv)) return rv;
	(void)stream_id;
	return BRIDGE_HW_OK;
}

int bridge_hw_adc_stream_read(uint8_t   stream_id,
                              uint8_t   max_samples,
                              uint8_t  *got_samples,
                              uint16_t *mv)
{
	int rv = 0;
	if (forced(FAKE_FN_ADC_STREAM_READ, &rv)) return rv;
	fake_adc_stream_t *s = &s_stream[stream_slot(stream_id)];
	if (s->force_got_armed) {
		if (got_samples) *got_samples = s->force_got;
		/* Deliberately does NOT clamp to max_samples -- this branch
		 * exists so a test can construct the "HAL reported more
		 * samples than the caller's buffer can hold" contract
		 * violation src/protocol.c:501 guards against. */
		for (uint8_t i = 0; i < s->force_got && mv != NULL; i++) {
			mv[i] = stream_pop_or_zero(s);
		}
		return BRIDGE_HW_OK;
	}
	uint8_t got = (uint8_t)(s->len < max_samples ? s->len : max_samples);
	if (got_samples) *got_samples = got;
	for (uint8_t i = 0; i < got; i++) {
		const uint16_t v = stream_pop_or_zero(s);
		if (mv) mv[i] = v;
	}
	return BRIDGE_HW_OK;
}

int bridge_hw_adc_stream_end(uint8_t stream_id)
{
	int rv = 0;
	if (forced(FAKE_FN_ADC_STREAM_END, &rv)) return rv;
	fake_adc_stream_t *s = &s_stream[stream_slot(stream_id)];
	s->len               = 0u;
	s->head              = 0u;
	s->force_got_armed   = 0;
	return BRIDGE_HW_OK;
}

int bridge_hw_adc_spectrum_read(uint8_t   stream_id,
                                uint16_t  bin_offset,
                                uint8_t   max_bins,
                                uint32_t *seq_out,
                                uint16_t *total_bins_out,
                                uint8_t  *got_bins_out,
                                float    *bins_out)
{
	int rv = 0;
	if (forced(FAKE_FN_ADC_SPECTRUM_READ, &rv)) return rv;
	fake_adc_spectrum_t *sp = &s_spectrum[stream_slot(stream_id)];
	if (seq_out) *seq_out = sp->seq;
	if (total_bins_out) *total_bins_out = sp->total_bins;

	uint8_t got;
	if (sp->force_got_armed) {
		got = sp->force_got; /* deliberately unclamped -- see the
		                      * stream-read force_got comment. */
	} else {
		uint16_t remaining =
		    (bin_offset < sp->bins_len) ? (uint16_t)(sp->bins_len - bin_offset) : 0u;
		got = (uint8_t)(remaining < max_bins ? remaining : max_bins);
	}
	if (got_bins_out) *got_bins_out = got;
	for (uint8_t i = 0; i < got && bins_out != NULL; i++) {
		const uint16_t idx = (uint16_t)(bin_offset + i);
		bins_out[i]        = (idx < sp->bins_len) ? sp->bins[idx] : 0.0f;
	}
	return BRIDGE_HW_OK;
}

int bridge_hw_trng_read(uint8_t *dest, size_t len)
{
	int rv = 0;
	if (forced(FAKE_FN_TRNG_READ, &rv)) return rv;
	for (size_t i = 0; i < len && dest != NULL; i++) {
		dest[i] = (uint8_t)(s_trng_seed + (uint8_t)i);
	}
	return BRIDGE_HW_OK;
}

int bridge_hw_tmu_compute(uint8_t   function,
                          uint8_t   format,
                          uint32_t  in_a,
                          uint32_t  in_b,
                          uint32_t *result_out)
{
	int rv = 0;
	if (forced(FAKE_FN_TMU_COMPUTE, &rv)) return rv;
	s_tmu_last.function = function;
	s_tmu_last.format   = format;
	s_tmu_last.in_a     = in_a;
	s_tmu_last.in_b     = in_b;
	if (result_out) *result_out = s_tmu_result_armed ? s_tmu_result : in_a;
	return BRIDGE_HW_OK;
}

int bridge_hw_dac_set(uint8_t channel, uint16_t value_mv)
{
	int rv = 0;
	if (forced(FAKE_FN_DAC_SET, &rv)) return rv;
	s_dac_mv[dac_slot(channel)] = value_mv;
	return BRIDGE_HW_OK;
}

int bridge_hw_dac_get(uint8_t channel, uint16_t *value_mv)
{
	int rv = 0;
	if (forced(FAKE_FN_DAC_GET, &rv)) return rv;
	if (value_mv) *value_mv = s_dac_mv[dac_slot(channel)];
	return BRIDGE_HW_OK;
}

int bridge_hw_qenc_read(uint8_t encoder, int32_t *position)
{
	int rv = 0;
	if (forced(FAKE_FN_QENC_READ, &rv)) return rv;
	if (position) *position = s_qenc_position[qenc_slot(encoder)];
	return BRIDGE_HW_OK;
}

int bridge_hw_qenc_reset(uint8_t encoder)
{
	int rv = 0;
	if (forced(FAKE_FN_QENC_RESET, &rv)) return rv;
	s_qenc_position[qenc_slot(encoder)] = 0;
	return BRIDGE_HW_OK;
}

int bridge_hw_counter_read(uint8_t counter, uint32_t *ticks)
{
	int rv = 0;
	if (forced(FAKE_FN_COUNTER_READ, &rv)) return rv;
	if (ticks) *ticks = s_counter_ticks[counter_slot(counter)];
	return BRIDGE_HW_OK;
}

uint8_t bridge_hw_da9292_status_cached(void)
{
	return s_da9292_status;
}

int bridge_hw_se_reset(uint8_t assert)
{
	/* Mirrors hal/gd32/se_reset.c:73's own range check (hal/bridge_hw.h:255
	 * documents BRIDGE_HW_ERR_INVAL for an out-of-range `assert`).
	 * protocol.c:277 pre-rejects assert>1 before protocol_dispatch() ever
	 * reaches this call, so no dispatch-driven test exercises this branch
	 * -- see test_protocol.c's direct bridge_hw_se_reset() call. */
	if (assert > 1u) return BRIDGE_HW_ERR_INVAL;
	int rv = 0;
	if (forced(FAKE_FN_SE_RESET, &rv)) return rv;
	s_se_reset_calls++;
	s_se_reset_last_assert = assert;
	return BRIDGE_HW_OK;
}

int bridge_hw_pwm_capture_begin(uint8_t channel, uint8_t edge)
{
	(void)channel;
	(void)edge;
	int rv = 0;
	if (forced(FAKE_FN_PWM_CAPTURE_BEGIN, &rv)) return rv;
	return BRIDGE_HW_OK;
}

int bridge_hw_pwm_capture_read(uint8_t channel, uint32_t *period_ns, uint32_t *pulse_width_ns)
{
	int rv = 0;
	if (forced(FAKE_FN_PWM_CAPTURE_READ, &rv)) return rv;
	const fake_pwm_t *p = &s_pwm[pwm_slot(channel)];
	if (period_ns) *period_ns = p->capture_period_ns;
	if (pulse_width_ns) *pulse_width_ns = p->capture_pulse_ns;
	return BRIDGE_HW_OK;
}

int bridge_hw_pwm_capture_end(uint8_t channel)
{
	(void)channel;
	int rv = 0;
	if (forced(FAKE_FN_PWM_CAPTURE_END, &rv)) return rv;
	return BRIDGE_HW_OK;
}

int bridge_hw_pwm_single_pulse(uint8_t channel, uint32_t pulse_ns)
{
	(void)channel;
	(void)pulse_ns;
	int rv = 0;
	if (forced(FAKE_FN_PWM_SINGLE_PULSE, &rv)) return rv;
	return BRIDGE_HW_OK;
}

int bridge_hw_timer_sync(uint8_t master, uint8_t slave, uint8_t mode)
{
	(void)master;
	(void)slave;
	(void)mode;
	int rv = 0;
	if (forced(FAKE_FN_TIMER_SYNC, &rv)) return rv;
	return BRIDGE_HW_OK;
}

int bridge_hw_power_mode_set(uint8_t mode, uint32_t wake_bitmap, uint32_t wake_after_ms)
{
	int rv = 0;
	if (forced(FAKE_FN_POWER_MODE_SET, &rv)) return rv;
	s_power_last.mode          = mode;
	s_power_last.wake_bitmap   = wake_bitmap;
	s_power_last.wake_after_ms = wake_after_ms;
	return BRIDGE_HW_OK;
}

int bridge_hw_adc_dsp_chain_open(uint8_t *chain_id)
{
	int rv = 0;
	if (forced(FAKE_FN_ADC_DSP_CHAIN_OPEN, &rv)) return rv;
	if (chain_id) *chain_id = s_dsp_next_chain_id;
	return BRIDGE_HW_OK;
}

int bridge_hw_adc_dsp_stage_push(uint8_t        chain_id,
                                 uint8_t        stage_index,
                                 uint8_t        kind,
                                 uint16_t       chunk_offset,
                                 uint16_t       chunk_total_size,
                                 const uint8_t *chunk_data,
                                 size_t         chunk_data_len)
{
	int rv = 0;
	if (forced(FAKE_FN_ADC_DSP_STAGE_PUSH, &rv)) return rv;
	/* Argument-decode capture only -- see bridge_hw_fake_dsp_stage_push_
	 * get_last_call's doc for why this is NOT the #69/#70 shape model. */
	s_dsp_stage_last.chain_id         = chain_id;
	s_dsp_stage_last.stage_index      = stage_index;
	s_dsp_stage_last.kind             = kind;
	s_dsp_stage_last.chunk_offset     = chunk_offset;
	s_dsp_stage_last.chunk_total_size = chunk_total_size;
	size_t n                          = chunk_data_len;
	if (n > GD32_BRIDGE_FAKE_STAGE_PUSH_DATA_CAP) n = GD32_BRIDGE_FAKE_STAGE_PUSH_DATA_CAP;
	s_dsp_stage_last.chunk_data_len = n;
	if (n > 0u && chunk_data != NULL) {
		memcpy(s_dsp_stage_last.chunk_data, chunk_data, n);
	}
	return BRIDGE_HW_OK;
}

int bridge_hw_adc_dsp_chain_bind(uint8_t chain_id, uint8_t stream_id)
{
	(void)chain_id;
	(void)stream_id;
	int rv = 0;
	if (forced(FAKE_FN_ADC_DSP_CHAIN_BIND, &rv)) return rv;
	return BRIDGE_HW_OK;
}
