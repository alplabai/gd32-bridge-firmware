/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * See fault_diag.h for why this is a separate, vendor-header-free TU.
 */

#include "fault_diag.h"

void fault_diag_build_cpu(fault_record_t *rec,
                          fault_type_t    fault_type,
                          uint32_t        pc,
                          uint32_t        lr,
                          uint32_t        cfsr,
                          uint32_t        hfsr,
                          uint32_t        mmfar_raw,
                          uint32_t        bfar_raw)
{
	rec->magic            = FAULT_RECORD_MAGIC | (uint32_t)fault_type;
	rec->pc               = pc;
	rec->lr               = lr;
	rec->status_primary   = cfsr;
	rec->status_secondary = hfsr;
	rec->mmfar            = (cfsr & FAULT_DIAG_CFSR_MMARVALID_MSK) ? mmfar_raw : 0u;
	rec->bfar             = (cfsr & FAULT_DIAG_CFSR_BFARVALID_MSK) ? bfar_raw : 0u;
}

void fault_diag_build_nmi(fault_record_t *rec,
                          uint32_t        pc,
                          uint32_t        lr,
                          uint32_t        syscfg_stat,
                          uint32_t        fmc_eccs_raw)
{
	rec->magic          = FAULT_RECORD_MAGIC | (uint32_t)FAULT_TYPE_NMI;
	rec->pc             = pc;
	rec->lr             = lr;
	rec->status_primary = syscfg_stat;
	rec->status_secondary =
	    (syscfg_stat & FAULT_DIAG_SYSCFG_STAT_FLASHECCIF_MSK) ? fmc_eccs_raw : 0u;
	rec->mmfar = 0u; /* not applicable to NMI */
	rec->bfar  = 0u; /* not applicable to NMI */
}
