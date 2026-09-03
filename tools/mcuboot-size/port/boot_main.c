/*
 * SPDX-License-Identifier: Apache-2.0
 * Throwaway entry point for the mcuboot-size measurement. NOT production.
 *
 * This is deliberately NOT a real bootloader: there is no vector table, no
 * clock/GPIO init, no SCB->VTOR handoff into the chosen slot -- Reset_Handler
 * calls boot_go() and spins. It exists so arm-none-eabi-gcc/ld have a real
 * entry point and boot_go()'s call graph is reachable (so --gc-sections
 * keeps it), which is what makes the size number mean something. The real
 * bootloader's startup/vector-table code is NOT included in this measurement
 * -- see the workflow summary.
 */
#include <stdint.h>

#include "bootutil/bootutil.h"
#include "bootutil/fault_injection_hardening.h"

#if defined(MCUBOOT_USE_TINYCRYPT) && defined(MCUBOOT_SIGN_EC256)
#include <tinycrypt/ecc.h>
#endif

static void probe_boot(void)
{
	struct boot_rsp rsp;
	FIH_DECLARE(fih_rc, FIH_FAILURE);

#if defined(MCUBOOT_USE_TINYCRYPT) && defined(MCUBOOT_SIGN_EC256)
	/* Verify-only: never exercised, but mirrors the real init call so the
	 * symbol is reachable the same way a real port would reach it. */
	uECC_set_rng(0);
#endif

	FIH_CALL(boot_go, fih_rc, &rsp);
	if (FIH_NOT_EQ(fih_rc, FIH_SUCCESS)) {
		/* A real bootloader would drop to a recovery/WFI loop here
		 * (see toolchain/gd32g553_bootloader.ld's header comment). */
	}
}

void Reset_Handler(void)
{
	probe_boot();
	for (;;) {
		/* Size probe only -- never actually boots an image. */
	}
}

/* Minimal vector table: just enough for a valid Cortex-M reset entry (word 0
 * = initial MSP, word 1 = Reset_Handler). Real NVIC/fault vectors are part
 * of the excluded startup code. */
__attribute__((section(".isr_vector"), used)) static const uint32_t probe_vectors[2] = {
	0x20018000u, /* initial MSP -- arbitrary, never runs */
	(uint32_t)(void (*)(void))Reset_Handler,
};
