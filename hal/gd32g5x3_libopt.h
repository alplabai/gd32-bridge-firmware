/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge's project-level standard-peripheral selector.  Both the
 * GD32G5x3 CMSIS header (`gd32g5x3.h`) and any peripheral source that
 * includes it pull in this file by name.  This selector controls which
 * peripheral declarations are visible to firmware translation units; it
 * does not select vendor source files or determine the linked image size.
 * The vendor wrapper compiles every driver into a sectioned static library,
 * and the linker discards unreferenced sections.
 *
 * Pattern: each time a `hal/gd32/` TU (or any peer that uses
 * the standard-peripheral library) starts calling into a new
 * peripheral driver, add the matching `gd32g5x3_<periph>.h` include
 * here so the driver's prototype + register layouts are visible to
 * the translation unit.
 *
 * Today the file lists only headers referenced by the real backend.  Add a
 * driver header when a firmware TU starts using it; doing so exposes the
 * API but still does not pull that driver's code into the final image.
 */

#ifndef GD32G5X3_LIBOPT_H
#define GD32G5X3_LIBOPT_H

/* Core / clock tree / interrupts / power. */
#include "gd32g5x3_rcu.h"
#include "gd32g5x3_misc.h"
#include "gd32g5x3_pmu.h"
#include "gd32g5x3_fwdgt.h"
#include "gd32g5x3_dbg.h"

/* GPIO + EXTI for the bridge's IO surface. */
#include "gd32g5x3_gpio.h"
#include "gd32g5x3_exti.h"
#include "gd32g5x3_syscfg.h"

/* Timers (advanced + general purpose) for PWM / capture / single-pulse
 * / encoder.  The free-running counter uses the Cortex-M DWT directly. */
#include "gd32g5x3_timer.h"

/* Analog: ADC + DAC + reference. */
#include "gd32g5x3_adc.h"
#include "gd32g5x3_dac.h"
#include "gd32g5x3_vref.h"

/* DMA + trigger routing for ADC streaming + FFT/FAC pipelines. */
#include "gd32g5x3_dma.h"
#include "gd32g5x3_trigsel.h"

/* DSP acceleration: TMU (CORDIC), FFT, FAC. */
#include "gd32g5x3_tmu.h"
#include "gd32g5x3_fft.h"
#include "gd32g5x3_fac.h"

/* Security: TRNG. */
#include "gd32g5x3_trng.h"

/* Serial buses for the host bridge. */
#include "gd32g5x3_spi.h"
#include "gd32g5x3_i2c.h"

/* Flash controller for OTA + factory provisioning. */
#include "gd32g5x3_fmc.h"

/* RTC wake source. */
#include "gd32g5x3_rtc.h"

#endif /* GD32G5X3_LIBOPT_H */
