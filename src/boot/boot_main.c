/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge OTA bootloader (Path A). Runs from the first 32 KB of flash
 * (0x08000000, never erased by OTA). Picks the active slot from the A/B
 * metadata, validates its image (CRC over the metadata-recorded length),
 * and jumps to it (MSP + VTOR + reset vector). If no slot validates it
 * idles so an external SWD probe can recover the part.
 *
 * Pairs with src/ota.c (the application-side OTA state machine that writes
 * the inactive slot + commits the metadata).
 *
 * SILICON-VALIDATED 2026-06-04 (bench, protocol v0.6): boot/validate/jump,
 * slot relocation, dual-bank FMC-from-RAM, and the full stream → verify →
 * commit → boot-new-slot → rollback cycle were proven end-to-end over the
 * 25 MHz link, including two GD32 self-reboots through this bootloader, for
 * the A→B update + rollback direction.  A fresh B→A update has NOT been
 * exercised.  See src/bootloader/DESIGN.md for the bench evidence.
 *
 * STILL HANDLE WITH CARE: a bug here bricks the GD32 (no host-driven SWD
 * reflash this HW rev — recover via a bench SWD probe), so change this path
 * incrementally and re-run the bench cycle rather than trusting the gate.
 */

#include <stdint.h>
#include <stddef.h>

#include "gd32g5x3.h"

#include "boot_decide.h"

static const uint8_t *flash_map(uint32_t address, size_t length, void *context)
{
	(void)length;
	(void)context;
	return (const uint8_t *)(uintptr_t)address;
}

static void jump_to_slot(uint32_t slot_base)
{
	const uint32_t msp   = *(volatile uint32_t *)slot_base;
	const uint32_t reset = *(volatile uint32_t *)(slot_base + 4u);
	__disable_irq();
	SCB->VTOR = slot_base; /* relocate the vector table to the slot */
	__DSB();
	__set_MSP(msp);
	__ISB();
	((void (*)(void))reset)(); /* no return */
}

int main(void)
{
	uint32_t base;
	/* Newest-first with fallback (#754): a newer record that fails image
	 * validation must not suppress an older bootable image. */
	if (boot_select_slot(flash_map, NULL, &base)) {
		jump_to_slot(base);
	}
	/* No valid image: recovery. A later build exposes the OTA opcodes here
     * to accept a reflash over the bridge; today, idle so a bench SWD probe
     * can take over. */
	for (;;) {
		__WFI();
	}
	/* unreachable */
	return 0;
}
