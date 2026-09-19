/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Internal lifecycle seam for the GD32 fault reset-loop guard.  Exception
 * entry points remain owned by the vendor vector-table symbol names in
 * fault_handlers.c; application bring-up needs only this health marker.
 */

#ifndef GD32_BRIDGE_HAL_GD32_FAULT_HANDLERS_H
#define GD32_BRIDGE_HAL_GD32_FAULT_HANDLERS_H

/* Clear only the consecutive-fault counter (RTC_BKP7).  Call after every
 * required transport has completed hardware initialisation successfully;
 * RTC_BKP0..6 retain the last diagnostic record for post-reset inspection. */
void fault_reset_loop_mark_healthy(void);

#endif /* GD32_BRIDGE_HAL_GD32_FAULT_HANDLERS_H */
