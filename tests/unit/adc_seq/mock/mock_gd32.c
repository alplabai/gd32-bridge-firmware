/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Implementation of the host test double declared in mock/gd32g5x3.h.
 * See that header's top comment for what this does and does not model.
 */
#include "gd32g5x3.h"

#include <string.h>

mock_seq_evt_t    mock_seq[MOCK_SEQ_MAX];
int               mock_seq_n;
volatile uint32_t mock_primask;
uint32_t          mock_rcu_lock_violations;

uint32_t        mock_irq_lock_count;
static uint32_t mock_fac_stall_after_writes;

static mock_hook_t mock_irq_lock_hook;
static uint32_t    mock_irq_locks_until_hook;

void mock_irq_set_lock_hook(uint32_t locks_until_hook, mock_hook_t hook)
{
	mock_irq_locks_until_hook = locks_until_hook;
	mock_irq_lock_hook        = hook;
}

uint32_t mock_irq_get_primask(void)
{
	mock_irq_lock_count++;
	if (mock_irq_lock_hook != 0 && mock_irq_locks_until_hook > 0u &&
	    --mock_irq_locks_until_hook == 0u) {
		mock_hook_t hook = mock_irq_lock_hook;
		mock_irq_set_lock_hook(0u, 0);
		hook();
	}
	return mock_primask;
}

void mock_seq_reset(void)
{
	mock_seq_n = 0;
	memset(mock_seq, 0, sizeof mock_seq);
	/* gh#35 FAC captures reset with the rest of the mock. */
	memset(mock_fac_coeffb, 0, sizeof mock_fac_coeffb);
	memset(mock_fac_coeffa, 0, sizeof mock_fac_coeffa);
	mock_fac_coeffb_size        = 0u;
	mock_fac_coeffa_size        = 0u;
	mock_fac_func               = 0u;
	mock_fac_ipr                = 0u;
	mock_fac_last_write         = 0;
	mock_fac_read_value         = 0;
	mock_fac_flags              = 0u;
	mock_fac_paracfg            = 0u;
	mock_fac_y_pending          = 0u;
	mock_fac_y_per_write        = 1u;
	mock_fac_load_wedge         = false;
	mock_fac_stall_after_writes = 0u;
	mock_primask                = 0u;
	mock_rcu_lock_violations    = 0u;
	mock_irq_lock_count         = 0u;
	mock_irq_set_lock_hook(0u, 0);
	mock_dma_set_transfer_get_hook(0);
	mock_fac_set_init_hook(0);
	mock_fft_set_flag(RESET);
	mock_fft_set_init_hook(0);
	mock_fft_set_poll_hook(0);
}

/* --- gh#35 FAC capture surface ------------------------------------ */

int16_t  mock_fac_coeffb[MOCK_FAC_MAX_COEFFS];
uint8_t  mock_fac_coeffb_size;
int16_t  mock_fac_coeffa[MOCK_FAC_MAX_COEFFS];
uint8_t  mock_fac_coeffa_size;
uint32_t mock_fac_func;
uint8_t  mock_fac_ipr;
int16_t  mock_fac_last_write;
int16_t  mock_fac_read_value;
uint32_t mock_fac_flags;
uint32_t mock_fac_paracfg;
uint32_t mock_fac_y_pending;
uint32_t mock_fac_y_per_write;
bool     mock_fac_load_wedge;

void mock_seq_log(const char *name, uint32_t periph, uint32_t arg)
{
	if (mock_seq_n >= MOCK_SEQ_MAX) return; /* test bug if this ever trips */
	mock_seq[mock_seq_n].name   = name;
	mock_seq[mock_seq_n].periph = periph;
	mock_seq[mock_seq_n].arg    = arg;
	mock_seq_n++;
}

int mock_seq_find_from(const char *name, uint32_t periph, int from)
{
	for (int i = (from < 0) ? 0 : from; i < mock_seq_n; ++i) {
		if (strcmp(mock_seq[i].name, name) != 0) continue;
		if (periph != MOCK_ANY_PERIPH && mock_seq[i].periph != periph) continue;
		return i;
	}
	return -1;
}

/* --- ADC_CTL1 register file + logging accessor ----------------------- */

uint32_t mock_adc_ctl1[MOCK_ADC_COUNT];
uint32_t mock_adc_rdata_dummy[MOCK_ADC_COUNT] = { 0x1000u, 0x1100u, 0x1200u, 0x1300u };

static uint32_t mock_ctl1_touch[MOCK_ADC_COUNT];

/* Every ADC_CTL1(periph) use in the code under test lives inside
 * adc_calibrate_bounded (checked: it is the only place either file
 * touches this macro).  Log the touch, then simulate a healthy
 * RSTCLB/CLB FSM completing after a few polls -- real hardware
 * finishes in ~tCAL (datasheet Table 4-35), a small, fixed number of
 * CK_ADC cycles, nowhere near the ~100000-iteration abort bound; this
 * mock just needs "eventually clears", not a cycle-accurate model. */
uint32_t *mock_adc_ctl1_ref(uint32_t periph)
{
	mock_seq_log("ADC_CTL1_TOUCH", periph, mock_adc_ctl1[periph]);
	if (++mock_ctl1_touch[periph] > 3u) {
		mock_adc_ctl1[periph] &= ~(ADC_CTL1_RSTCLB | ADC_CTL1_CLB);
	}
	return &mock_adc_ctl1[periph];
}

/* --- ADC ---------------------------------------------------------------*/

static uint32_t mock_adc_flags[MOCK_ADC_COUNT];
static uint32_t mock_adc_routine_data = 2048u; /* mid-scale default */

void adc_deinit(uint32_t adc_periph)
{
	mock_seq_log("adc_deinit", adc_periph, 0u);
}
void adc_clock_config(uint32_t adc_periph, uint32_t prescaler)
{
	(void)prescaler;
	mock_seq_log("adc_clock_config", adc_periph, 0u);
}
void adc_data_alignment_config(uint32_t adc_periph, uint32_t data_alignment)
{
	(void)data_alignment;
	mock_seq_log("adc_data_alignment_config", adc_periph, 0u);
}
void adc_enable(uint32_t adc_periph)
{
	mock_seq_log("adc_enable", adc_periph, 0u);
}
void adc_internal_channel_config(uint32_t adc_periph, uint32_t internal_channel, ControlStatus s)
{
	(void)internal_channel;
	mock_adc_internal_ch_on = (s == ENABLE);
	mock_seq_log("adc_internal_channel_config", adc_periph, (uint32_t)s);
}
void adc_disable(uint32_t adc_periph)
{
	mock_seq_log("adc_disable", adc_periph, 0u);
}
void adc_resolution_config(uint32_t adc_periph, uint32_t resolution)
{
	(void)resolution;
	mock_seq_log("adc_resolution_config", adc_periph, 0u);
}
void adc_dma_mode_enable(uint32_t adc_periph)
{
	mock_seq_log("adc_dma_mode_enable", adc_periph, 0u);
}
void adc_dma_mode_disable(uint32_t adc_periph)
{
	mock_seq_log("adc_dma_mode_disable", adc_periph, 0u);
}
void adc_dma_request_after_last_enable(uint32_t adc_periph)
{
	mock_seq_log("adc_dma_request_after_last_enable", adc_periph, 0u);
}
void adc_dma_request_after_last_disable(uint32_t adc_periph)
{
	mock_seq_log("adc_dma_request_after_last_disable", adc_periph, 0u);
}
void adc_channel_length_config(uint32_t adc_periph, uint8_t adc_sequence, uint32_t length)
{
	(void)adc_sequence;
	(void)length;
	mock_seq_log("adc_channel_length_config", adc_periph, 0u);
}
void adc_routine_channel_config(uint32_t adc_periph,
                                uint8_t  rank,
                                uint8_t  adc_channel,
                                uint32_t sample_time)
{
	(void)rank;
	(void)adc_channel;
	(void)sample_time;
	mock_seq_log("adc_routine_channel_config", adc_periph, 0u);
}
void adc_external_trigger_config(uint32_t adc_periph, uint8_t adc_sequence, uint32_t trigger_mode)
{
	(void)adc_sequence;
	mock_seq_log("adc_external_trigger_config", adc_periph, trigger_mode);
}
bool mock_adc_eoc_stuck;
bool mock_adc_internal_ch_on;

void adc_software_trigger_enable(uint32_t adc_periph, uint8_t adc_sequence)
{
	(void)adc_sequence;
	mock_seq_log("adc_software_trigger_enable", adc_periph, 0u);
	/* A software-triggered conversion completes instantly in this
	 * mock: mark EOC so the polling loop in bridge_hw_adc_read sees
	 * a completed conversion on its very first check. */
	if (!mock_adc_eoc_stuck) mock_adc_flags[adc_periph] |= ADC_FLAG_EOC;
}
uint32_t adc_routine_data_read(uint32_t adc_periph)
{
	mock_seq_log("adc_routine_data_read", adc_periph, 0u);
	return mock_adc_routine_data;
}
void adc_oversample_mode_config(uint32_t adc_periph, uint32_t mode, uint32_t shift, uint16_t ratio)
{
	(void)mode;
	(void)shift;
	(void)ratio;
	mock_seq_log("adc_oversample_mode_config", adc_periph, 0u);
}
void adc_oversample_mode_enable(uint32_t adc_periph)
{
	mock_seq_log("adc_oversample_mode_enable", adc_periph, 0u);
}
void adc_oversample_mode_disable(uint32_t adc_periph)
{
	mock_seq_log("adc_oversample_mode_disable", adc_periph, 0u);
}
FlagStatus adc_flag_get(uint32_t adc_periph, uint32_t flag)
{
	mock_seq_log("adc_flag_get", adc_periph, flag);
	return (mock_adc_flags[adc_periph] & flag) ? SET : RESET;
}
void adc_flag_clear(uint32_t adc_periph, uint32_t flag)
{
	mock_seq_log("adc_flag_clear", adc_periph, flag);
	mock_adc_flags[adc_periph] &= ~flag;
}

void mock_adc_set_flag(uint32_t periph, uint32_t flag, FlagStatus state)
{
	if (state == SET) {
		mock_adc_flags[periph] |= flag;
	} else {
		mock_adc_flags[periph] &= ~flag;
	}
}
void mock_adc_set_routine_data(uint32_t code)
{
	mock_adc_routine_data = code;
}

/* --- DMA -----------------------------------------------------------------*/

static uint32_t mock_dma_remaining[2][1]; /* [dma_periph][channel] */
static uint32_t mock_dma_chctl[2][1];
static uint32_t mock_dma_interrupt_flags[2][1];
static bool     mock_dma_disable_hold[2][1];
static uint32_t mock_dmamux_chcfg[14];

uint32_t *mock_dma_chctl_ref(uint32_t dma_periph, dma_channel_enum channelx)
{
	mock_seq_log("DMA_CHCTL_READ", dma_periph, channelx);
	return &mock_dma_chctl[dma_periph][channelx];
}

uint32_t *mock_dmamux_chcfg_ref(uint32_t channel)
{
	return &mock_dmamux_chcfg[channel];
}
static mock_hook_t mock_dma_transfer_get_hook;

void dma_deinit(uint32_t dma_periph, dma_channel_enum channelx)
{
	mock_dma_chctl[dma_periph][channelx] &= ~DMA_CHXCTL_CHEN;
	mock_seq_log("dma_deinit", dma_periph, 0u);
}
void dma_struct_para_init(dma_parameter_struct *init_struct)
{
	memset(init_struct, 0, sizeof *init_struct);
}
void dma_init(uint32_t dma_periph, dma_channel_enum channelx, dma_parameter_struct *init_struct)
{
	/* The real SPL dma_init() writes init_struct->number to DMA_CHCNT.
	 * Mirror that observable contract so a new stream cannot inherit the
	 * previous test session's remaining-count state. */
	mock_dma_remaining[dma_periph][channelx] = init_struct->number;
	mock_dmamux_chcfg[(dma_periph == DMA0) ? (uint32_t)channelx : (uint32_t)channelx + 7u] =
	    init_struct->request;
	mock_seq_log("dma_init", dma_periph, init_struct->number);
}
void dma_circulation_enable(uint32_t dma_periph, dma_channel_enum channelx)
{
	(void)channelx;
	mock_seq_log("dma_circulation_enable", dma_periph, 0u);
}
void dma_channel_enable(uint32_t dma_periph, dma_channel_enum channelx)
{
	mock_dma_chctl[dma_periph][channelx] |= DMA_CHXCTL_CHEN;
	mock_seq_log("dma_channel_enable", dma_periph, 0u);
}
void dma_channel_disable(uint32_t dma_periph, dma_channel_enum channelx)
{
	if (!mock_dma_disable_hold[dma_periph][channelx]) {
		mock_dma_chctl[dma_periph][channelx] &= ~DMA_CHXCTL_CHEN;
	}
	mock_seq_log("dma_channel_disable", dma_periph, 0u);
}
void dma_transfer_number_config(uint32_t dma_periph, dma_channel_enum channelx, uint32_t number)
{
	mock_seq_log("dma_transfer_number_config", dma_periph, number);
	mock_dma_remaining[dma_periph][channelx] = number; /* just configured: nothing sent yet */
}
uint32_t dma_transfer_number_get(uint32_t dma_periph, dma_channel_enum channelx)
{
	mock_seq_log("dma_transfer_number_get", dma_periph, 0u);
	const uint32_t remaining = mock_dma_remaining[dma_periph][channelx];
	if (mock_dma_transfer_get_hook != 0) {
		mock_hook_t hook           = mock_dma_transfer_get_hook;
		mock_dma_transfer_get_hook = 0;
		hook();
	}
	return remaining;
}
void dma_flag_clear(uint32_t dma_periph, dma_channel_enum channelx, uint32_t flag)
{
	(void)channelx;
	mock_seq_log("dma_flag_clear", dma_periph, flag);
}
void dma_interrupt_enable(uint32_t dma_periph, dma_channel_enum channelx, uint32_t source)
{
	(void)channelx;
	mock_seq_log("dma_interrupt_enable", dma_periph, source);
}
void dma_interrupt_disable(uint32_t dma_periph, dma_channel_enum channelx, uint32_t source)
{
	(void)channelx;
	mock_seq_log("dma_interrupt_disable", dma_periph, source);
}
FlagStatus dma_interrupt_flag_get(uint32_t dma_periph, dma_channel_enum channelx, uint32_t int_flag)
{
	return (mock_dma_interrupt_flags[dma_periph][channelx] & int_flag) ? SET : RESET;
}
void dma_interrupt_flag_clear(uint32_t dma_periph, dma_channel_enum channelx, uint32_t int_flag)
{
	mock_dma_interrupt_flags[dma_periph][channelx] &= ~int_flag;
	mock_seq_log("dma_interrupt_flag_clear", dma_periph, int_flag);
}

void mock_dma_set_remaining(uint32_t dma_periph, dma_channel_enum channelx, uint32_t remaining)
{
	mock_dma_remaining[dma_periph][channelx] = remaining;
}

void mock_dma_set_interrupt_flag(uint32_t         dma_periph,
                                 dma_channel_enum channelx,
                                 uint32_t         flag,
                                 FlagStatus       state)
{
	if (state == SET) {
		mock_dma_interrupt_flags[dma_periph][channelx] |= flag;
	} else {
		mock_dma_interrupt_flags[dma_periph][channelx] &= ~flag;
	}
}

void mock_dma_reset(void)
{
	memset(mock_dma_remaining, 0, sizeof mock_dma_remaining);
	memset(mock_dma_chctl, 0, sizeof mock_dma_chctl);
	memset(mock_dma_interrupt_flags, 0, sizeof mock_dma_interrupt_flags);
	memset(mock_dma_disable_hold, 0, sizeof mock_dma_disable_hold);
	memset(mock_dmamux_chcfg, 0, sizeof mock_dmamux_chcfg);
}

void mock_dma_set_disable_hold(uint32_t dma_periph, dma_channel_enum channelx, bool hold)
{
	mock_dma_disable_hold[dma_periph][channelx] = hold;
	if (hold) mock_dma_chctl[dma_periph][channelx] |= DMA_CHXCTL_CHEN;
}

uint32_t mock_dmamux_request_get(uint32_t channel)
{
	return mock_dmamux_chcfg[channel] & DMAMUX_RM_CHXCFG_MUXID;
}

void mock_dma_set_transfer_get_hook(mock_hook_t hook)
{
	mock_dma_transfer_get_hook = hook;
}

/* --- RCU / TRIGSEL / TIMER / NVIC: logged no-ops --------------------------*/

void rcu_periph_clock_enable(uint32_t periph_clk)
{
	if (mock_primask != 1u) mock_rcu_lock_violations++;
	mock_seq_log("rcu_periph_clock_enable", periph_clk, 0u);
}
void trigsel_init(trigsel_periph_enum target_periph, trigsel_source_enum trigger_source)
{
	mock_seq_log("trigsel_init", (uint32_t)target_periph, (uint32_t)trigger_source);
}
void timer_deinit(uint32_t timer_periph)
{
	mock_seq_log("timer_deinit", timer_periph, 0u);
}
void timer_struct_para_init(timer_parameter_struct *initpara)
{
	memset(initpara, 0, sizeof *initpara);
}
void timer_init(uint32_t timer_periph, timer_parameter_struct *initpara)
{
	(void)initpara;
	mock_seq_log("timer_init", timer_periph, 0u);
}
void timer_master_output0_trigger_source_select(uint32_t timer_periph, uint32_t outrigger)
{
	(void)outrigger;
	mock_seq_log("timer_master_output0_trigger_source_select", timer_periph, 0u);
}
void timer_enable(uint32_t timer_periph)
{
	mock_seq_log("timer_enable", timer_periph, 0u);
}
void timer_disable(uint32_t timer_periph)
{
	mock_seq_log("timer_disable", timer_periph, 0u);
}
void nvic_irq_enable(IRQn_Type nvic_irq, uint8_t pre, uint8_t sub)
{
	(void)pre;
	(void)sub;
	mock_seq_log("nvic_irq_enable", (uint32_t)nvic_irq, 0u);
}
void nvic_irq_disable(IRQn_Type nvic_irq)
{
	mock_seq_log("nvic_irq_disable", (uint32_t)nvic_irq, 0u);
}

/* --- FAC / FFT: lightweight lifecycle stubs ------------------------------*/

static mock_dsp_init_hook_t mock_fac_init_hook;

void fac_deinit(void)
{
	mock_fac_paracfg = 0u;
}
void fac_struct_para_init(fac_parameter_struct *p)
{
	memset(p, 0, sizeof *p);
}
void fac_init(fac_parameter_struct *p)
{
	(void)p;
	mock_seq_log("fac_init", 0u, 0u);
	if (mock_fac_init_hook != 0) {
		mock_dsp_init_hook_t hook = mock_fac_init_hook;
		mock_fac_init_hook        = 0;
		hook();
	}
}
void fac_fixed_data_preload_init(fac_fixed_data_preload_struct *p)
{
	memset(p, 0, sizeof *p);
}
void fac_fixed_buffer_preload(fac_fixed_data_preload_struct *p)
{
	/* gh#35: capture the decoded coefficient vectors the production
	 * decode hands to the FAC, so the AN208 / Iir_dma numbers can be
	 * asserted directly. */
	if (p->coeffb_size > MOCK_FAC_MAX_COEFFS || p->coeffa_size > MOCK_FAC_MAX_COEFFS) return;
	/* gh#306: model the vendor sequence on silicon.  PARACFG = IPP | LOAD_X0
	 * | EXE, then input_size words complete the load.  A load of size 0
	 * NEVER completes: EXE stays set and the X1 load below is never
	 * latched (FAC_PARACFG stayed 0x81000000 on the bench). */
	mock_fac_paracfg = (uint32_t)p->input_size | FUNC_LOAD_X0 | FAC_PARACFG_EXE;
	if (p->input_size == 0u) {
		mock_seq_log("fac_fixed_buffer_preload", 0u, 0u);
		return;
	}
	mock_fac_paracfg &= ~FAC_PARACFG_EXE;
	mock_fac_paracfg = (uint32_t)p->coeffb_size | FUNC_LOAD_X1 | FAC_PARACFG_EXE;
	if (!mock_fac_load_wedge) mock_fac_paracfg &= ~FAC_PARACFG_EXE;
	memset(mock_fac_coeffb, 0, sizeof mock_fac_coeffb);
	memset(mock_fac_coeffa, 0, sizeof mock_fac_coeffa);
	for (uint8_t k = 0u; k < p->coeffb_size; ++k)
		mock_fac_coeffb[k] = p->coeffb_ctx[k];
	for (uint8_t k = 0u; k < p->coeffa_size; ++k)
		mock_fac_coeffa[k] = p->coeffa_ctx[k];
	mock_fac_coeffb_size = p->coeffb_size;
	mock_fac_coeffa_size = p->coeffa_size;
	mock_seq_log("fac_fixed_buffer_preload", 0u, 0u);
}
void fac_function_config(fac_parameter_struct *p)
{
	/* Function config only latches when no load is in flight (EXE clear). */
	if (mock_fac_paracfg & FAC_PARACFG_EXE) {
		mock_seq_log("fac_function_config", p->func, p->ipr);
		return;
	}
	mock_fac_func    = p->func;
	mock_fac_ipr     = p->ipr; /* gh#35: the headroom exponent must land here */
	mock_fac_paracfg = (mock_fac_paracfg & ~FAC_PARACFG_FUN) | p->func;
	mock_seq_log("fac_function_config", p->func, p->ipr);
}
void fac_start(void)
{
	mock_fac_paracfg |= FAC_PARACFG_EXE;
	mock_seq_log("fac_start", 0u, 0u);
}
void fac_stop(void)
{
	mock_fac_paracfg &= ~FAC_PARACFG_EXE;
	mock_seq_log("fac_stop", 0u, 0u);
}
void fac_fixed_data_write(int16_t data)
{
	mock_fac_last_write = data; /* gh#35 bias test */
	/* Y only fills once a filter function is latched AND running. */
	if ((mock_fac_paracfg & FAC_PARACFG_EXE) &&
	    ((mock_fac_paracfg & FAC_PARACFG_FUN) == FUNC_CONVO_FIR ||
	     (mock_fac_paracfg & FAC_PARACFG_FUN) == FUNC_IIR_DIRECT_FORM_1))
		mock_fac_y_pending += mock_fac_y_per_write;
	if (mock_fac_stall_after_writes != 0u && --mock_fac_stall_after_writes == 0u)
		mock_fac_flags |= FAC_FLAG_X0BFF;
	mock_seq_log("fac_fixed_data_write", 0u, (uint32_t)(uint16_t)data);
}
int16_t fac_fixed_data_read(void)
{
	if (mock_fac_y_pending != 0u) mock_fac_y_pending--;
	return mock_fac_read_value;
}
FlagStatus fac_flag_get(uint32_t flag)
{
	/* YBEF must read RESET for the pump's output path to run; the
	 * saturation flags are settable per test (gh#35). */
	if (flag == FAC_FLAG_YBEF) return (mock_fac_y_pending == 0u) ? SET : RESET;
	return (mock_fac_flags & flag) ? SET : RESET;
}
void mock_fac_set_write_hook_after(uint32_t writes)
{
	mock_fac_stall_after_writes = writes;
}
void mock_fac_set_init_hook(mock_dsp_init_hook_t hook)
{
	mock_fac_init_hook = hook;
}

void fft_deinit(void)
{
}
void fft_struct_para_init(fft_parameter_struct *p)
{
	memset(p, 0, sizeof *p);
}
static mock_dsp_init_hook_t mock_fft_init_hook;

void fft_init(fft_parameter_struct *p)
{
	(void)p;
	if (mock_fft_init_hook != 0) {
		mock_dsp_init_hook_t hook = mock_fft_init_hook;
		mock_fft_init_hook        = 0;
		hook();
	}
}
void fft_calculation_start(void)
{
	mock_seq_log("fft_calculation_start", 0u, 0u);
}
static FlagStatus  mock_fft_status;
static mock_hook_t mock_fft_poll_hook;

FlagStatus fft_flag_get(uint32_t flag)
{
	(void)flag;
	const FlagStatus status = mock_fft_status;
	if (mock_fft_poll_hook != 0) {
		mock_hook_t hook   = mock_fft_poll_hook;
		mock_fft_poll_hook = 0;
		hook();
	}
	return status;
}
void mock_fft_set_flag(FlagStatus status)
{
	mock_fft_status = status;
}
void mock_fft_set_init_hook(mock_dsp_init_hook_t hook)
{
	mock_fft_init_hook = hook;
}

void mock_fft_set_poll_hook(mock_hook_t hook)
{
	mock_fft_poll_hook = hook;
}

/* --- gd32_common.h externs the driver needs but this suite doesn't use ---*/

/* hal/gd32/vref.c is linked for real; this is its VREFRDY source. */
bool       mock_vref_ready = true;
FlagStatus vref_status_get(void)
{
	return mock_vref_ready ? SET : RESET;
}
