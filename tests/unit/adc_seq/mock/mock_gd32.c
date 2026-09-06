/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Implementation of the host test double declared in mock/gd32g5x3.h.
 * See that header's top comment for what this does and does not model.
 */
#include "gd32g5x3.h"

#include <string.h>

mock_seq_evt_t mock_seq[MOCK_SEQ_MAX];
int            mock_seq_n;

void mock_seq_reset(void)
{
	mock_seq_n = 0;
	memset(mock_seq, 0, sizeof mock_seq);
}

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
void adc_software_trigger_enable(uint32_t adc_periph, uint8_t adc_sequence)
{
	(void)adc_sequence;
	mock_seq_log("adc_software_trigger_enable", adc_periph, 0u);
	/* A software-triggered conversion completes instantly in this
	 * mock: mark EOC so the polling loop in bridge_hw_adc_read sees
	 * a completed conversion on its very first check. */
	mock_adc_flags[adc_periph] |= ADC_FLAG_EOC;
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

void dma_deinit(uint32_t dma_periph, dma_channel_enum channelx)
{
	(void)channelx;
	mock_seq_log("dma_deinit", dma_periph, 0u);
}
void dma_struct_para_init(dma_parameter_struct *init_struct)
{
	memset(init_struct, 0, sizeof *init_struct);
}
void dma_init(uint32_t dma_periph, dma_channel_enum channelx, dma_parameter_struct *init_struct)
{
	(void)channelx;
	(void)init_struct;
	mock_seq_log("dma_init", dma_periph, 0u);
}
void dma_circulation_enable(uint32_t dma_periph, dma_channel_enum channelx)
{
	(void)channelx;
	mock_seq_log("dma_circulation_enable", dma_periph, 0u);
}
void dma_channel_enable(uint32_t dma_periph, dma_channel_enum channelx)
{
	(void)channelx;
	mock_seq_log("dma_channel_enable", dma_periph, 0u);
}
void dma_channel_disable(uint32_t dma_periph, dma_channel_enum channelx)
{
	(void)channelx;
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
	return mock_dma_remaining[dma_periph][channelx];
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
	(void)dma_periph;
	(void)channelx;
	(void)int_flag;
	return RESET;
}
void dma_interrupt_flag_clear(uint32_t dma_periph, dma_channel_enum channelx, uint32_t int_flag)
{
	(void)channelx;
	mock_seq_log("dma_interrupt_flag_clear", dma_periph, int_flag);
}

void mock_dma_set_remaining(uint32_t dma_periph, dma_channel_enum channelx, uint32_t remaining)
{
	mock_dma_remaining[dma_periph][channelx] = remaining;
}

/* --- RCU / TRIGSEL / TIMER / NVIC: logged no-ops --------------------------*/

void rcu_periph_clock_enable(uint32_t periph_clk)
{
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

/* --- FAC / FFT: link-only stubs, never exercised by these tests ----------*/

void fac_deinit(void)
{
}
void fac_struct_para_init(fac_parameter_struct *p)
{
	memset(p, 0, sizeof *p);
}
void fac_init(fac_parameter_struct *p)
{
	(void)p;
}
void fac_fixed_data_preload_init(fac_fixed_data_preload_struct *p)
{
	memset(p, 0, sizeof *p);
}
void fac_fixed_buffer_preload(fac_fixed_data_preload_struct *p)
{
	(void)p;
}
void fac_function_config(fac_parameter_struct *p)
{
	(void)p;
}
void fac_start(void)
{
}
void fac_stop(void)
{
}
void fac_fixed_data_write(int16_t data)
{
	(void)data;
}
int16_t fac_fixed_data_read(void)
{
	return 0;
}
FlagStatus fac_flag_get(uint32_t flag)
{
	(void)flag;
	return RESET;
}

void fft_deinit(void)
{
}
void fft_struct_para_init(fft_parameter_struct *p)
{
	memset(p, 0, sizeof *p);
}
void fft_init(fft_parameter_struct *p)
{
	(void)p;
}
void fft_calculation_start(void)
{
}
FlagStatus fft_flag_get(uint32_t flag)
{
	(void)flag;
	return RESET;
}

/* --- gd32_common.h externs the driver needs but this suite doesn't use ---*/

bool vref_ok = true;
bool vref_ready_check(void)
{
	return vref_ok;
}
