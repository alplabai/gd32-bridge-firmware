/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 */

#include "reset_reason.h"

uint8_t bridge_reset_reason_decode(uint32_t rstsck)
{
	if (rstsck & (BRIDGE_RESET_RSTSCK_FWDGTRSTF | BRIDGE_RESET_RSTSCK_WWDGTRSTF)) {
		return 4u; /* WDT */
	}
	if (rstsck & BRIDGE_RESET_RSTSCK_SWRSTF) return 3u;  /* SOFT */
	if (rstsck & BRIDGE_RESET_RSTSCK_LPRSTF) return 6u;  /* LOWPOWER */
	if (rstsck & BRIDGE_RESET_RSTSCK_PORRSTF) return 1u; /* POWER_ON */
	if (rstsck & BRIDGE_RESET_RSTSCK_EPRSTF) return 2u;  /* NRST_PIN */
	if (rstsck & BRIDGE_RESET_RSTSCK_BORRSTF) return 5u; /* BROWNOUT */
	return 0u; /* UNKNOWN, including option-byte-loader-only resets for now */
}

uint32_t bridge_reset_reason_latch_and_clear(volatile uint32_t *rstsck)
{
	const uint32_t snapshot = *rstsck;

	*rstsck |= BRIDGE_RESET_RSTSCK_RSTFC;
	return snapshot;
}
