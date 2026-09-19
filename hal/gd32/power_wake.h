/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G553 power-wake bitmap validation.  Kept vendor-header-free so the
 * exact fail-closed predicate used by power.c can run in host CI.
 */
#ifndef GD32_BRIDGE_POWER_WAKE_H
#define GD32_BRIDGE_POWER_WAKE_H

#include <stdbool.h>
#include <stdint.h>

/* Mirrors the ALP_POWER_WAKE_* wire encoding in <alp/power.h>. */
#define POWER_WAKE_RTC      0x00000001u
#define POWER_WAKE_GPIO     0x00000002u
#define POWER_WAKE_UART_RX  0x00000004u
#define POWER_WAKE_TIMER    0x00000008u
#define POWER_WAKE_USB      0x00000010u
#define POWER_WAKE_ETH_LINK 0x00000020u

/* This GD32 backend can arm only RTC/timed and fixed-PMU-pad wakeups.
 * Reject every other bit, including future bits unknown to this firmware,
 * so a low-power request never reports success without arming the wake
 * source the host requested (#107). */
#define POWER_WAKE_MASK_SUPPORTED (POWER_WAKE_RTC | POWER_WAKE_GPIO | POWER_WAKE_TIMER)

static inline bool power_wake_bitmap_supported(uint32_t wake_bitmap)
{
	return (wake_bitmap & ~POWER_WAKE_MASK_SUPPORTED) == 0u;
}

#endif /* GD32_BRIDGE_POWER_WAKE_H */
