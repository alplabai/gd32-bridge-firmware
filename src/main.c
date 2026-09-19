/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge firmware -- entry point.
 *
 * The GD32G553 supervisor runs this firmware as its sole user
 * application (no RTOS).  The SPI CS-EXTI and I2C interrupt paths
 * hand complete request envelopes to protocol_dispatch() (in
 * protocol.c) and stage the matching reply envelope back to the
 * transport.
 *
 * The main loop calls the __WFI seam, then bridge_hw_tick().  The
 * current ARM image still resolves __WFI to the empty weak fallback
 * below (#13), so it does not yet idle between interrupts.  The gd32
 * tick override pumps DSP streams and advances background OTA erases.
 * The DA9292 fault nets
 * (DA9292_INT/DA9292_TW) reach only the Renesas (P37/P36), so the GD32
 * still has no PMIC sample to collect and CMD_DA9292_STATUS_FORWARD
 * serves the 0xFF "no sample" sentinel.  Register-level PMIC status
 * (PMC_STATUS_00 etc.) is read by the host over BRD_I2C via the alp-sdk
 * chips/da9292 driver.
 *
 * Backends: BRIDGE_HAL_BACKEND=gd32 drives real silicon (peripheral
 * HAL in the per-peripheral TUs under hal/gd32/, SPI1 + I2C0 slave
 * transports in hal/transport_hw_gd32.c).  BRIDGE_HAL_BACKEND=stub keeps everything
 * hardware-free for host-side protocol tests: PING / GET_VERSION /
 * GET_BUILD_ID / RESET_REASON round-trip and HW-touching ops return
 * NOSUPPORT.
 */

#include <stdint.h>

#include "protocol.h"
#include "transport.h"

/* Optional weak hooks the HAL layer can override.  Both defaults are
 * no-ops; the real backend overrides them for peripheral bring-up and
 * base-level housekeeping. */
__attribute__((weak)) void bridge_hw_init(void)
{
}
__attribute__((weak)) void bridge_hw_tick(void)
{
}

/* Weak fallback for hosted toolchains where the Cortex-M intrinsic is
 * unavailable.  It currently also resolves in the ARM image; #13 tracks
 * replacing that no-op with a real wait-for-interrupt instruction. */
#ifndef __WFI
__attribute__((weak)) void __WFI(void)
{
}
#endif

int main(void)
{
	bridge_hw_init();
	transport_spi_init();
	transport_i2c_init();

	/* The gd32 override pumps DSP streams and background OTA erase work;
     * the weak stub default remains a no-op. */
	for (;;) {
		__WFI();
		bridge_hw_tick();
	}
	/* unreachable */
	return 0;
}
