/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * The firmware's ONE critical-section primitive.
 *
 * Why this file exists (#19 and its children #133/#134/#147): this is a
 * bare-metal, purely interrupt-driven application in which
 * protocol_dispatch() runs synchronously inside BOTH transport ISRs --
 * BRIDGE_SPI_CS_EXTI_HANDLER at BRIDGE_CS_IRQ_PRIO (1) and
 * BRIDGE_I2C_EV_HANDLER at BRIDGE_I2C_IRQ_PRIO (2), see
 * hal/bridge_board_config.h.  Group priority 1 pre-empts group priority
 * 2, so any handler reached from I2C can be suspended mid-sequence and
 * resumed after the SPI side has rewritten the same peripheral.  Until
 * this header there was no critical section anywhere in the application:
 * the only __disable_irq() in the tree is src/boot/boot_main.c's, in the
 * bootloader, on the way out of the bootloader.
 *
 * Why PRIMASK and not BASEPRI: BASEPRI is the finer instrument, but the
 * only thing this application ever needs to exclude is the CS-EXTI
 * handler, which is already the highest-priority interrupt it owns.
 * Masking everything is therefore equivalent in effect here, and PRIMASK
 * is one register with no priority-encoding subtleties to get wrong.
 * PRIMASK does not mask HardFault or NMI, so a fault raised inside a
 * section is still delivered.
 *
 * Why save/restore rather than a bare __enable_irq() at the end: these
 * sections nest -- an ADC claim inside a dispatch that is itself inside
 * an ISR -- and a bare __enable_irq() at the inner exit would unmask
 * interrupts the outer section was relying on being masked.  Restoring
 * the caller's PRIMASK makes nesting correct by construction.
 *
 * HOW TO USE IT, and the rule that matters:
 *
 *     const uint32_t st = bridge_irq_lock();
 *     ... a handful of instructions ...
 *     bridge_irq_unlock(st);
 *
 * Keep the body to a test-and-set or a single register read-modify-write.
 * A section is not a place to do work: every cycle spent inside one is a
 * cycle the SPI CS-EXTI handler cannot run, and that handler has a hard
 * deadline (the host clocks the reply out).  Long mutual exclusion is
 * what the ownership FLAGS this primitive protects are for -- claim the
 * flag in a short section, do the work with interrupts ON, release it in
 * another short section.
 *
 * This header includes the vendor device header, so it must NOT be
 * included from gd32_common.h: that header is deliberately vendor-free
 * (adc_dsp_chain.c links into a host test through it).  Include this
 * directly from the .c files that already pull "gd32g5x3.h".
 */

#ifndef GD32_BRIDGE_HAL_GD32_BRIDGE_CRITICAL_H
#define GD32_BRIDGE_HAL_GD32_BRIDGE_CRITICAL_H

#include <stdint.h>

#include "gd32g5x3.h"

/* Enter a critical section.  Returns the PRIMASK value to hand back to
 * bridge_irq_unlock().  Safe to nest. */
static inline uint32_t bridge_irq_lock(void)
{
	const uint32_t primask = __get_PRIMASK();

	__disable_irq();
	return primask;
}

/* Leave a critical section, restoring the interrupt-enable state the
 * matching bridge_irq_lock() found -- NOT unconditionally enabling. */
static inline void bridge_irq_unlock(uint32_t primask)
{
	__set_PRIMASK(primask);
}

#endif /* GD32_BRIDGE_HAL_GD32_BRIDGE_CRITICAL_H */
