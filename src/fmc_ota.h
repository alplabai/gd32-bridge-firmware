/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge OTA — flash HW seam.
 *
 * Real implementation in hal/fmc_ota.c (gd32 backend; the erase/program
 * inner loop runs from RAM so the FMC stall doesn't fault the executing
 * core).  Weak no-op defaults live in ota.c so the stub backend links
 * without the vendor FMC driver — there ota_fmc_supported() returns false
 * and the OTA state machine degrades to STATUS_NOSUPPORT.
 */
#ifndef GD32_BRIDGE_FMC_OTA_H
#define GD32_BRIDGE_FMC_OTA_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* false on a build without real FMC support (stub backend) -> OTA inert. */
bool ota_fmc_supported(void);

/* gh#281: erase_range/program used to collapse every failure -- a real
 * FMC error (PGERR/WPERR/...) and a bounded ota_fmc_wait_ready() timeout
 * (FMC_TOERR, hal/fmc_ota.c) alike -- into a single bool, so the host
 * could not tell a stuck FMC from any other flash fault; both read as
 * STATUS_IO.  This tri-state preserves the distinction erase_one_page() /
 * program_one_dword() already compute internally, through to the callers
 * that can usefully report it on the wire (STATUS_TIMEOUT already exists
 * in gd32_bridge_status_t -- no new wire value, no PROTOCOL_VERSION
 * bump). */
typedef enum {
	OTA_FMC_RESULT_OK = 0,
	OTA_FMC_RESULT_ERROR,
	OTA_FMC_RESULT_TIMEOUT, /* terminal state was FMC_TOERR, not another FMC error */
} ota_fmc_result_t;

/* Erase [base, base+len): base + len must be page-aligned (OTA_PAGE_SIZE).
 * Returns non-OK on FMC error (ERROR) or a bounded wait_ready timeout
 * (TIMEOUT -- gh#281), or on the funnel guard (hal/fmc_ota_guard.h, #79)
 * refusing the range (ERROR) -- it always refuses the bootloader, and, in
 * a partitioned build that knows its own BRIDGE_APP_SLOT_BASE, this
 * build's own running slot. */
ota_fmc_result_t ota_fmc_erase_range(uint32_t base, uint32_t len);

/* Program `len` bytes at `addr` (flash). `addr` and `len` honour the
 * device's program granularity (handled inside). Returns non-OK on FMC
 * error (ERROR), a bounded wait_ready timeout (TIMEOUT -- gh#281), or on
 * the same funnel-guard refusal ota_fmc_erase_range() documents above
 * (ERROR). */
ota_fmc_result_t ota_fmc_program(uint32_t addr, const uint8_t *data, size_t len);

/* Resolve a flash address to a readable pointer.  Default (target) is a
 * plain cast; the seam exists so hardware-less unit tests can redirect the
 * metadata/slot read paths into a host buffer instead of dereferencing a
 * raw 32-bit flash address on a 64-bit host. */
const void *ota_fmc_flash_ptr(uint32_t addr);

/* True while another caller currently owns the FMC funnel (#147) -- i.e. an
 * ota_fmc_erase_range()/ota_fmc_program() call is mid-flight elsewhere and
 * the next one would lose the race and fail outright.  Query-only: never
 * claims or releases.  Always false on the stub backend (weak default in
 * ota.c), matching ota_fmc_supported() -- there is no funnel to contend
 * for.  See #266 / ota_fmc_funnel_busy()'s definition in hal/fmc_ota.c for
 * why h_begin (src/ota.c) needs this ahead of its metadata-demote call. */
bool ota_fmc_funnel_busy(void);

#endif /* GD32_BRIDGE_FMC_OTA_H */
