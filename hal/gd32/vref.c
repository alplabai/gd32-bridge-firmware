/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 bridge HAL backend -- analog reference readiness latch.
 * Split move-only from hal/bridge_hw_gd32.c (fw v0.2.8); see
 * hal/gd32/init.c for the backend-wide implementation notes.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bridge_hw.h"
#include "gd32g5x3.h"

#include "gd32_common.h"

/* Analog-reference availability latch.
 *
 * Through v0.2.8 this polled VREFRDY, the ready bit for the on-chip
 * VREF buffer's INTERNAL voltage-reference mode.  hal/gd32/init.c no
 * longer drives that mode -- running the buffer's 2.048 V target on
 * the module's 1.8 V VDDA violated Datasheet Rev2.0 p.109 Table 4-3
 * (VDDA min = VREFP+0.3 when VREFBUF is used, i.e. 2.348 V), so VREF_CS
 * is left at its reset value 0x0000 0002 (VREFEN=0, HIPM=1 -- external
 * voltage reference mode, User Manual Rev1.2 p.520 Table 20-1; see the
 * VREF bring-up comment in init.c, alp-sdk-internal
 * gd32-bridge-firmware#59).
 *
 * VREFRDY's only documented meaning (User Manual Rev1.2 p.520) is
 * scoped to internal voltage-reference mode, so it says nothing about
 * external-reference mode's state -- there is no on-chip bit left for
 * this firmware to poll.  vref_ok is therefore a constant true: ADC
 * and DAC ops proceed unconditionally, same as before this fix but now
 * without pretending a meaningless flag is a health signal.  Whether
 * VREFP is actually board-tied to VDDA -- the fact ADC_VREF_MV /
 * DAC_VREF_MV assume -- is a hardware question this firmware cannot
 * observe; it needs a meter on the VREFP ball (J6). */
bool vref_ok = true;

bool vref_ready_check(void)
{
	return vref_ok;
}
