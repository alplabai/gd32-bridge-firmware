/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Minimal host-only GD32G5x3 SPL surface for hal/gd32/timer_sync.c.
 * Names and signatures match GigaDevice's gd32g5x3_timer.h and
 * gd32g5x3_syscfg.h.  TIMER19 is intentionally absent: production must not
 * reference the uninitialised peripheral after #142.
 */
#ifndef GD32_BRIDGE_TIMER_SYNC_MOCK_GD32G5X3_H
#define GD32_BRIDGE_TIMER_SYNC_MOCK_GD32G5X3_H

#include <stdint.h>

#define TIMER0 ((uint32_t)0x40012C00u)
#define TIMER7 ((uint32_t)0x40013400u)

#define TIMER_TRI_OUT0_SRC_UPDATE      ((uint32_t)0x20u)
#define TIMER_MASTER_SLAVE_MODE_ENABLE ((uint32_t)0x80u)

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

static inline uint32_t __get_PRIMASK(void)
{
	return 0u;
}

static inline void __disable_irq(void)
{
}

static inline void __set_PRIMASK(uint32_t primask)
{
	(void)primask;
}

#endif /* GD32_BRIDGE_TIMER_SYNC_MOCK_GD32G5X3_H */
