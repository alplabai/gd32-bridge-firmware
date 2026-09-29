/* SPDX-License-Identifier: Apache-2.0 */
/* Minimal host double for bridge_critical.h. The OTA suite is
 * single-threaded; its erase callback models the asynchronous ISR. */
#ifndef GD32_BRIDGE_OTA_MOCK_GD32G5X3_H
#define GD32_BRIDGE_OTA_MOCK_GD32G5X3_H

#include <stdint.h>

extern volatile uint32_t g_ota_mock_primask;

static inline uint32_t __get_PRIMASK(void)
{
	return g_ota_mock_primask;
}

static inline void __disable_irq(void)
{
	g_ota_mock_primask = 1u;
}

static inline void __set_PRIMASK(uint32_t primask)
{
	g_ota_mock_primask = primask;
}

#endif /* GD32_BRIDGE_OTA_MOCK_GD32G5X3_H */
