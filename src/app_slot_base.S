/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Link-time witness for the slot base the firmware compiler received.
 * The slot linker script compares this absolute symbol with ORIGIN(FLASH),
 * so a legal-but-swapped BRIDGE_APP_SLOT_BASE cannot silently move VTOR and
 * the OTA running-slot guard away from the image being executed (#78).
 */

#ifndef BRIDGE_APP_SLOT_BASE
#error "app_slot_base.S requires BRIDGE_APP_SLOT_BASE"
#endif

	.global __bridge_compiled_app_slot_base
	.equ __bridge_compiled_app_slot_base, BRIDGE_APP_SLOT_BASE
