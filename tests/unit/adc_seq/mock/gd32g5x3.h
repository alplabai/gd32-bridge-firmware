/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Host-only test double for the GD32G5x3 vendor SPL header.
 *
 * This is NOT a general-purpose GD32 mock -- it exists to let
 * hal/gd32/adc.c and hal/gd32/adc_stream.c compile and run, UNMODIFIED,
 * on the host, so a small set of register-level SEQUENCE assertions
 * (adc_enable -> calibrate, ROVF-clear-before-enable, DMA FTF clear on
 * ROVF recovery) can be pinned without hardware (#77: no CI job compiles
 * hal/gd32/ sources today).  Function names/signatures mirror the real
 * vendor headers (vendors/gd32_firmware_library/upstream/.../
 * gd32g5x3_adc.h, gd32g5x3_dma.h, gd32g5x3_timer.h, gd32g5x3_trigsel.h,
 * gd32g5x3_fac.h, gd32g5x3_fft.h, gd32g5x3_misc.h) exactly, checked by
 * hand against those files -- ONLY the call sites the two source files
 * under test actually use are given a real behaviour; everything else
 * (FAC/FFT programming, timers, TRIGSEL, NVIC, RCU) is a harmless no-op
 * stub, present purely so the translation unit links.
 */
#ifndef GD32_BRIDGE_MOCK_GD32G5X3_H
#define GD32_BRIDGE_MOCK_GD32G5X3_H

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Event log -- the register-sequence gate's data source.              */
/* ------------------------------------------------------------------ */

#define MOCK_SEQ_MAX 256

typedef struct {
	const char *name;   /* event tag, e.g. "adc_enable"            */
	uint32_t    periph; /* the peripheral/channel argument, if any */
	uint32_t    arg;    /* a secondary argument (e.g. a flag mask) */
} mock_seq_evt_t;

extern mock_seq_evt_t mock_seq[MOCK_SEQ_MAX];
extern int            mock_seq_n;

void mock_seq_reset(void);
void mock_seq_log(const char *name, uint32_t periph, uint32_t arg);
/* First/last index of an event matching name (+ periph if periph !=
 * MOCK_ANY_PERIPH), searched over [from, mock_seq_n).  Returns -1 if
 * not found. */
#define MOCK_ANY_PERIPH 0xFFFFFFFFu
int mock_seq_find_from(const char *name, uint32_t periph, int from);

/* ------------------------------------------------------------------ */
/* Flag status + peripheral base "addresses" (plain small integers --  */
/* used only as array indices by this mock, never dereferenced as      */
/* real memory).                                                       */
/* ------------------------------------------------------------------ */

typedef enum { RESET = 0, SET = !RESET } FlagStatus;

#define ADC0           ((uint32_t)0u)
#define ADC1           ((uint32_t)1u)
#define ADC2           ((uint32_t)2u)
#define ADC3           ((uint32_t)3u)
#define MOCK_ADC_COUNT 4u

#define DMA0 ((uint32_t)0u)
#define DMA1 ((uint32_t)1u)

#define GPIOA       ((uint32_t)0u)
#define GPIOB       ((uint32_t)1u)
#define GPIOC       ((uint32_t)2u)
#define GPIOD       ((uint32_t)3u)
#define GPIOE       ((uint32_t)4u)
#define GPIOF       ((uint32_t)5u)
#define GPIO_PIN_2  ((uint32_t)(1u << 2))
#define GPIO_PIN_3  ((uint32_t)(1u << 3))
#define GPIO_PIN_4  ((uint32_t)(1u << 4))
#define GPIO_PIN_5  ((uint32_t)(1u << 5))
#define GPIO_PIN_9  ((uint32_t)(1u << 9))
#define GPIO_PIN_11 ((uint32_t)(1u << 11))
#define GPIO_PIN_12 ((uint32_t)(1u << 12))
#define GPIO_PIN_13 ((uint32_t)(1u << 13))

#define ADC_CHANNEL_2  ((uint8_t)2u)
#define ADC_CHANNEL_3  ((uint8_t)3u)
#define ADC_CHANNEL_4  ((uint8_t)4u)
#define ADC_CHANNEL_12 ((uint8_t)12u)
#define ADC_CHANNEL_13 ((uint8_t)13u)

#define ADC_RESOLUTION_12B 0u
#define ADC_RESOLUTION_10B 1u
#define ADC_RESOLUTION_8B  2u
#define ADC_RESOLUTION_6B  3u

#define ADC_CLK_SYNC_HCLK_DIV6   0u
#define ADC_DATAALIGN_RIGHT      0u
#define ADC_ROUTINE_CHANNEL      0u
#define EXTERNAL_TRIGGER_DISABLE 0u
#define EXTERNAL_TRIGGER_RISING  1u

#define ADC_FLAG_EOC  ((uint32_t)(1u << 0))
#define ADC_FLAG_ROVF ((uint32_t)(1u << 1))

#define ADC_OVERSAMPLING_ALL_CONVERT 0u
#define OVSCR_OVSS(n)                (n)

/* Only these two bits are ever touched via ADC_CTL1() in the code
 * under test (both inside adc_calibrate_bounded) -- real values,
 * matching gd32g5x3_adc.h's ADC_CTL1_RSTCLB/ADC_CTL1_CLB bit
 * positions, so a stray extra use elsewhere would still behave
 * sanely instead of aliasing another field. */
#define ADC_CTL1_RSTCLB ((uint32_t)(1u << 3))
#define ADC_CTL1_CLB    ((uint32_t)(1u << 2))

/* ADC_CTL1(periph): the ONLY register the code under test manipulates
 * directly (both uses live inside adc_calibrate_bounded).  Modelled as
 * a real 4-entry register file, accessed through a logging function so
 * calibration activity shows up in the sequence log -- and made to
 * "complete" after a few touches, simulating a healthy RSTCLB/CLB FSM,
 * so the bounded loop returns quickly instead of spinning its full
 * ~100000-iteration ceiling (still exercised, just not for real time). */
extern uint32_t mock_adc_ctl1[MOCK_ADC_COUNT];
uint32_t       *mock_adc_ctl1_ref(uint32_t periph);
#define ADC_CTL1(periph) (*mock_adc_ctl1_ref(periph))

/* ADC_RDATA is only READ into a struct field (init.periph_addr = &ADC_
 * RDATA(...)) in the code under test, never dereferenced by this mock
 * -- a distinct placeholder number per peripheral is enough. */
#define ADC_RDATA(periph) (mock_adc_rdata_dummy[(periph)])
extern uint32_t mock_adc_rdata_dummy[MOCK_ADC_COUNT];

/* ------------------------------------------------------------------ */
/* ADC function surface.                                               */
/* ------------------------------------------------------------------ */

void adc_deinit(uint32_t adc_periph);
void adc_clock_config(uint32_t adc_periph, uint32_t prescaler);
void adc_data_alignment_config(uint32_t adc_periph, uint32_t data_alignment);
void adc_enable(uint32_t adc_periph);
void adc_disable(uint32_t adc_periph);
void adc_resolution_config(uint32_t adc_periph, uint32_t resolution);
void adc_dma_mode_enable(uint32_t adc_periph);
void adc_dma_mode_disable(uint32_t adc_periph);
void adc_dma_request_after_last_enable(uint32_t adc_periph);
void adc_dma_request_after_last_disable(uint32_t adc_periph);
void adc_channel_length_config(uint32_t adc_periph, uint8_t adc_sequence, uint32_t length);
void adc_routine_channel_config(uint32_t adc_periph,
                                uint8_t  rank,
                                uint8_t  adc_channel,
                                uint32_t sample_time);
void adc_external_trigger_config(uint32_t adc_periph, uint8_t adc_sequence, uint32_t trigger_mode);
void adc_software_trigger_enable(uint32_t adc_periph, uint8_t adc_sequence);
uint32_t adc_routine_data_read(uint32_t adc_periph);
void adc_oversample_mode_config(uint32_t adc_periph, uint32_t mode, uint32_t shift, uint16_t ratio);
void adc_oversample_mode_enable(uint32_t adc_periph);
void adc_oversample_mode_disable(uint32_t adc_periph);
FlagStatus adc_flag_get(uint32_t adc_periph, uint32_t flag);
void       adc_flag_clear(uint32_t adc_periph, uint32_t flag);

/* Test-only hooks (not part of any real vendor header) -- let a test
 * drive the mocked ADC_FLAG_ROVF bit and control adc_routine_data_read's
 * return value without reaching into the driver's private state. */
void mock_adc_set_flag(uint32_t periph, uint32_t flag, FlagStatus state);
void mock_adc_set_routine_data(uint32_t code);

/* ------------------------------------------------------------------ */
/* DMA function surface.                                               */
/* ------------------------------------------------------------------ */

typedef enum { DMA_CH0 = 0 } dma_channel_enum;

#define DMA_PERIPHERAL_TO_MEMORY    0u
#define DMA_PERIPH_INCREASE_DISABLE 0u
#define DMA_MEMORY_INCREASE_ENABLE  1u
#define DMA_PERIPHERAL_WIDTH_16BIT  0u
#define DMA_MEMORY_WIDTH_16BIT      0u
#define DMA_PRIORITY_MEDIUM         0u
#define DMA_REQUEST_ADC0            0u
#define DMA_REQUEST_ADC1            1u
#define DMA_REQUEST_ADC2            2u
#define DMA_REQUEST_ADC3            3u
#define DMA_FLAG_FTF                ((uint32_t)(1u << 0))
#define DMA_INT_FTF                 ((uint32_t)(1u << 0))
#define DMA_INT_FLAG_FTF            ((uint32_t)(1u << 0))

typedef struct {
	uint32_t periph_addr;
	uint32_t periph_width;
	uint32_t memory_addr;
	uint32_t memory_width;
	uint32_t number;
	uint32_t priority;
	uint8_t  periph_inc;
	uint8_t  memory_inc;
	uint8_t  direction;
	uint32_t request;
} dma_parameter_struct;

void dma_deinit(uint32_t dma_periph, dma_channel_enum channelx);
void dma_struct_para_init(dma_parameter_struct *init_struct);
void dma_init(uint32_t dma_periph, dma_channel_enum channelx, dma_parameter_struct *init_struct);
void dma_circulation_enable(uint32_t dma_periph, dma_channel_enum channelx);
void dma_channel_enable(uint32_t dma_periph, dma_channel_enum channelx);
void dma_channel_disable(uint32_t dma_periph, dma_channel_enum channelx);
void dma_transfer_number_config(uint32_t dma_periph, dma_channel_enum channelx, uint32_t number);
uint32_t dma_transfer_number_get(uint32_t dma_periph, dma_channel_enum channelx);
void     dma_flag_clear(uint32_t dma_periph, dma_channel_enum channelx, uint32_t flag);
void     dma_interrupt_enable(uint32_t dma_periph, dma_channel_enum channelx, uint32_t source);
void     dma_interrupt_disable(uint32_t dma_periph, dma_channel_enum channelx, uint32_t source);
FlagStatus
dma_interrupt_flag_get(uint32_t dma_periph, dma_channel_enum channelx, uint32_t int_flag);
void dma_interrupt_flag_clear(uint32_t dma_periph, dma_channel_enum channelx, uint32_t int_flag);

/* Test-only hook: set the value dma_transfer_number_get returns (the
 * "remaining" countdown), so the write-index math in adc_stream_write_
 * index resolves to a safe, in-range value without a real DMA. */
void mock_dma_set_remaining(uint32_t dma_periph, dma_channel_enum channelx, uint32_t remaining);

/* ------------------------------------------------------------------ */
/* RCU -- every clock-gate call is a no-op tag; only logged.           */
/* ------------------------------------------------------------------ */

#define RCU_DMAMUX  0u
#define RCU_DMA0    1u
#define RCU_DMA1    2u
#define RCU_TRIGSEL 3u
#define RCU_TIMER5  4u
#define RCU_TIMER6  5u
#define RCU_FAC     6u
#define RCU_FFT     7u
void rcu_periph_clock_enable(uint32_t periph_clk);

/* ------------------------------------------------------------------ */
/* TRIGSEL -- routing only, no register semantics needed by the tests. */
/* ------------------------------------------------------------------ */

typedef enum {
	TRIGSEL_OUTPUT_ADC0_ROUTRG,
	TRIGSEL_OUTPUT_ADC1_ROUTRG,
	TRIGSEL_OUTPUT_ADC2_ROUTRG,
	TRIGSEL_OUTPUT_ADC3_ROUTRG
} trigsel_periph_enum;

typedef enum { TRIGSEL_INPUT_TIMER5_TRGO0, TRIGSEL_INPUT_TIMER6_TRGO0 } trigsel_source_enum;

void trigsel_init(trigsel_periph_enum target_periph, trigsel_source_enum trigger_source);

/* ------------------------------------------------------------------ */
/* TIMER -- pacing only, no register semantics needed by the tests.    */
/* ------------------------------------------------------------------ */

#define TIMER5                    0u
#define TIMER6                    1u
#define TIMER_TRI_OUT0_SRC_UPDATE 0u

typedef struct {
	uint16_t prescaler;
	uint16_t period;
} timer_parameter_struct;

void timer_deinit(uint32_t timer_periph);
void timer_struct_para_init(timer_parameter_struct *initpara);
void timer_init(uint32_t timer_periph, timer_parameter_struct *initpara);
void timer_master_output0_trigger_source_select(uint32_t timer_periph, uint32_t outrigger);
void timer_enable(uint32_t timer_periph);
void timer_disable(uint32_t timer_periph);

/* ------------------------------------------------------------------ */
/* NVIC -- logged only.                                                */
/* ------------------------------------------------------------------ */

typedef enum { DMA0_Channel0_IRQn = 0, DMA1_Channel0_IRQn = 1 } IRQn_Type;

void nvic_irq_enable(IRQn_Type nvic_irq,
                     uint8_t   nvic_irq_pre_priority,
                     uint8_t   nvic_irq_sub_priority);
void nvic_irq_disable(IRQn_Type nvic_irq);

/* ------------------------------------------------------------------ */
/* FAC -- the DSP filter block.  adc_stream.c's #496 pump code must    */
/* link; none of the tests below exercise it, so every hook is inert.  */
/* ------------------------------------------------------------------ */

#define FAC_THRESHOLD_1        0u
#define FAC_CP_ENABLE          1u
#define FUNC_CONVO_FIR         0u
#define FUNC_IIR_DIRECT_FORM_1 1u
#define FAC_FLAG_X0BFF         ((uint32_t)(1u << 0))
#define FAC_FLAG_YBEF          ((uint32_t)(1u << 1))

typedef struct {
	uint8_t  coeff_addr;
	uint8_t  coeff_size;
	uint8_t  input_addr;
	uint8_t  input_size;
	uint8_t  output_addr;
	uint8_t  output_size;
	uint32_t input_threshold;
	uint32_t output_threshold;
	uint32_t clip;
	uint32_t func;
	uint8_t  ipp;
	uint8_t  ipq;
	uint8_t  ipr;
} fac_parameter_struct;

typedef struct {
	const int16_t *coeffb_ctx;
	uint8_t        coeffb_size;
	const int16_t *coeffa_ctx;
	uint8_t        coeffa_size;
	const int16_t *input_ctx;
	uint8_t        input_size;
	const int16_t *output_ctx;
	uint8_t        output_size;
} fac_fixed_data_preload_struct;

void       fac_deinit(void);
void       fac_struct_para_init(fac_parameter_struct *fac_parameter);
void       fac_init(fac_parameter_struct *fac_parameter);
void       fac_fixed_data_preload_init(fac_fixed_data_preload_struct *init_struct);
void       fac_fixed_buffer_preload(fac_fixed_data_preload_struct *init_struct);
void       fac_function_config(fac_parameter_struct *fac_parameter);
void       fac_start(void);
void       fac_stop(void);
void       fac_fixed_data_write(int16_t data);
int16_t    fac_fixed_data_read(void);
FlagStatus fac_flag_get(uint32_t flag);

/* ------------------------------------------------------------------ */
/* FFT -- the spectrum block.  Same story as FAC: link-only for these  */
/* tests.                                                               */
/* ------------------------------------------------------------------ */

#define FFT_MODE           0u
#define FFT_DOWNSAMPLE_1   0u
#define FFT_IM_ZERO        0u
#define FFT_WINDOW_ENABLE  1u
#define FFT_WINDOW_DISABLE 0u
#define FFT_POINT_32       0u
#define FFT_POINT_64       1u
#define FFT_POINT_128      2u
#define FFT_POINT_256      3u
#define FFT_FLAG_CCF       ((uint32_t)(1u << 0))

typedef struct {
	uint32_t mode_sel;
	uint32_t point_num;
	uint32_t downsamp_sel;
	uint32_t image_source;
	uint32_t real_addr;
	uint32_t image_addr;
	uint32_t output_addr;
	uint32_t window_enable;
	uint32_t window_addr;
} fft_parameter_struct;

void       fft_deinit(void);
void       fft_struct_para_init(fft_parameter_struct *fft_parameter);
void       fft_init(fft_parameter_struct *fft_parameter);
void       fft_calculation_start(void);
FlagStatus fft_flag_get(uint32_t flag);

#endif /* GD32_BRIDGE_MOCK_GD32G5X3_H */
