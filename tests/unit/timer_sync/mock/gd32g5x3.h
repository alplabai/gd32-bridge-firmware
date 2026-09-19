/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Minimal host-only GD32G5x3 SPL surface for the production timer-sync and
 * PWM HAL bodies. Names and signatures match the GigaDevice SPL. TIMER19 is
 * intentionally absent: production must not reference the uninitialised
 * peripheral after #142.
 */
#ifndef GD32_BRIDGE_TIMER_SYNC_MOCK_GD32G5X3_H
#define GD32_BRIDGE_TIMER_SYNC_MOCK_GD32G5X3_H

#include <stdbool.h>
#include <stdint.h>

#define TIMER0 ((uint32_t)0x40012C00u)
#define TIMER7 ((uint32_t)0x40013400u)

#define TIMER_TRI_OUT0_SRC_UPDATE      ((uint32_t)0x20u)
#define TIMER_MASTER_SLAVE_MODE_ENABLE ((uint32_t)0x80u)

#define ENABLE  ((uint32_t)1u)
#define DISABLE ((uint32_t)0u)

#define GPIOA       ((uint32_t)0x40020000u)
#define GPIOB       ((uint32_t)0x40020400u)
#define GPIOC       ((uint32_t)0x40020800u)
#define GPIOD       ((uint32_t)0x40020C00u)
#define GPIO_PIN_0  ((uint32_t)1u << 0)
#define GPIO_PIN_1  ((uint32_t)1u << 1)
#define GPIO_PIN_5  ((uint32_t)1u << 5)
#define GPIO_PIN_10 ((uint32_t)1u << 10)
#define GPIO_PIN_11 ((uint32_t)1u << 11)
#define GPIO_PIN_12 ((uint32_t)1u << 12)
#define GPIO_PIN_14 ((uint32_t)1u << 14)
#define GPIO_AF_4   ((uint32_t)4u)
#define GPIO_AF_6   ((uint32_t)6u)

#define TIMER_CH_0 ((uint16_t)0u)
#define TIMER_CH_1 ((uint16_t)1u)
#define TIMER_CH_2 ((uint16_t)2u)
#define TIMER_CH_3 ((uint16_t)3u)

#define TIMER_COUNTER_EDGE        ((uint32_t)0u)
#define TIMER_COUNTER_CENTER_UP   ((uint32_t)0x10u)
#define TIMER_COUNTER_CENTER_DOWN ((uint32_t)0x20u)
#define TIMER_COUNTER_CENTER_BOTH ((uint32_t)0x30u)
#define TIMER_COUNTER_UP          ((uint32_t)0u)
#define TIMER_CKDIV_DIV1          ((uint32_t)0u)
#define TIMER_CCX_DISABLE         ((uint32_t)0u)
#define TIMER_CCX_ENABLE          ((uint32_t)1u)
#define TIMER_CCXN_DISABLE        ((uint32_t)0u)
#define TIMER_CCXN_ENABLE         ((uint32_t)1u)
#define TIMER_OC_POLARITY_HIGH    ((uint32_t)0u)
#define TIMER_OCN_POLARITY_HIGH   ((uint32_t)0u)
#define TIMER_OC_IDLE_STATE_LOW   ((uint32_t)0u)
#define TIMER_OCN_IDLE_STATE_LOW  ((uint32_t)0u)
#define TIMER_OC_MODE_PWM0        ((uint32_t)6u)
#define TIMER_OC_MODE_PWM1        ((uint32_t)7u)
#define TIMER_OC_SHADOW_ENABLE    ((uint32_t)1u)
#define TIMER_SP_MODE_REPETITIVE  ((uint32_t)0u)
#define TIMER_SP_MODE_SINGLE      ((uint32_t)1u)
#define TIMER_EVENT_SRC_UPG       ((uint32_t)1u)
#define TIMER_CTL0_CEN            ((uint32_t)1u)
#define TIMER_CTL0_CAM            ((uint32_t)0x60u)

typedef struct {
	uint16_t prescaler;
	uint32_t alignedmode;
	uint32_t counterdirection;
	uint32_t period;
	uint32_t clockdivision;
	uint32_t repetitioncounter;
} timer_parameter_struct;

typedef struct {
	uint32_t outputstate;
	uint32_t outputnstate;
	uint32_t ocpolarity;
	uint32_t ocnpolarity;
	uint32_t ocidlestate;
	uint32_t ocnidlestate;
} timer_oc_parameter_struct;

extern volatile uint32_t mock_timer_ctl0[2];
extern volatile uint32_t mock_timer_car[2];
extern volatile uint32_t mock_timer_cv[2][4];
extern volatile uint32_t mock_primask;

#define TIMER_CTL0(periph) (mock_timer_ctl0[((periph) == TIMER0) ? 0u : 1u])
#define TIMER_CAR(periph)  (mock_timer_car[((periph) == TIMER0) ? 0u : 1u])

#define TIMER_QUAD_DECODER_MODE1   ((uint8_t)0x01u)
#define TIMER_SLAVE_MODE_RESTART   ((uint8_t)0x03u)
#define TIMER_SLAVE_MODE_PAUSE     ((uint8_t)0x04u)
#define TIMER_SLAVE_MODE_EVENT     ((uint8_t)0x05u)
#define TIMER_SLAVE_MODE_EXTERNAL0 ((uint8_t)0x06u)
#define TIMER_SLAVE_MODE_DISABLE   ((uint8_t)0x0Fu)

void timer_master_output0_trigger_source_select(uint32_t timer_periph, uint32_t outrigger);
void timer_master_slave_mode_config(uint32_t timer_periph, uint32_t masterslave);
void timer_input_trigger_source_select(uint32_t timer_periph, uint32_t intrigger);
void timer_slave_mode_select(uint32_t timer_periph, uint32_t slavemode);
void timer_struct_para_init(timer_parameter_struct *initpara);
void timer_deinit(uint32_t timer_periph);
void timer_init(uint32_t timer_periph, const timer_parameter_struct *initpara);
void timer_auto_reload_shadow_enable(uint32_t timer_periph);
void timer_primary_output_config(uint32_t timer_periph, uint32_t newvalue);
void timer_enable(uint32_t timer_periph);
void timer_disable(uint32_t timer_periph);
void timer_channel_output_struct_para_init(timer_oc_parameter_struct *ocpara);
void timer_channel_output_config(uint32_t                         timer_periph,
                                 uint16_t                         channel,
                                 const timer_oc_parameter_struct *ocpara);
void timer_channel_output_pulse_value_config(uint32_t timer_periph,
                                             uint16_t channel,
                                             uint32_t pulse);
void timer_channel_output_mode_config(uint32_t timer_periph, uint16_t channel, uint32_t oc_mode);
void timer_channel_output_shadow_config(uint32_t timer_periph,
                                        uint16_t channel,
                                        uint32_t oc_shadow);
void timer_single_pulse_mode_config(uint32_t timer_periph, uint32_t sp_mode);
void timer_autoreload_value_config(uint32_t timer_periph, uint32_t autoreload);
void timer_event_software_generate(uint32_t timer_periph, uint32_t event);
void timer_counter_value_config(uint32_t timer_periph, uint32_t counter);
uint32_t timer_channel_capture_value_register_read(uint32_t timer_periph, uint16_t channel);

static inline uint32_t __get_PRIMASK(void)
{
	return mock_primask;
}

static inline void __disable_irq(void)
{
	mock_primask = 1u;
}

static inline void __set_PRIMASK(uint32_t primask)
{
	mock_primask = primask;
}

#endif /* GD32_BRIDGE_TIMER_SYNC_MOCK_GD32G5X3_H */
