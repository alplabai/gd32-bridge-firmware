/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure fault-record decode logic, split out of fault_handlers.c so it is
 * host-testable (tests/unit/fault_diag/) the same way hal/gd32/tmu_q31_scale.c
 * is: no vendor "gd32g5x3.h" include, no register access, no side effects.
 * fault_handlers.c does the actual SCB / SYSCFG / FMC register reads and
 * hands the raw values in here.
 *
 * The two things this module has to get right are exactly the two things a
 * hand-rolled version gets wrong under pressure:
 *
 *   1. SCB->MMFAR / SCB->BFAR are only meaningful when CFSR's MMARVALID /
 *      BFARVALID bit says so (ARMv8-M ARM, CFSR bits 7 and 15 of MMFSR/BFSR)
 *      -- an unconditional capture records a stale address left over from a
 *      PREVIOUS fault, which reads as "found it" while pointing nowhere near
 *      the real one.
 *   2. The Flash-ECC path in FMC_ECCCS is the one reachable NMI source (see
 *      issue #36); the other four SYSCFG_STAT bits are recorded as raw
 *      status so a person can tell CKMNMIIF/SRAM*ECCMEIF apart, but FMC_ECCCS
 *      is only valid, and only worth reading, when FLASHECCIF actually
 *      fired -- reading it unconditionally records a stale ECCADDR/BK_ECC
 *      from whatever flash access last happened to trip it.
 */

#ifndef GD32_BRIDGE_HAL_GD32_FAULT_DIAG_H
#define GD32_BRIDGE_HAL_GD32_FAULT_DIAG_H

#include <stdint.h>

/* Exception tag, one per overridden vector.  Values are wire-stable (baked
 * into fault_record_t.magic low byte, read back across a reset) -- append,
 * never renumber. */
typedef enum {
	FAULT_TYPE_NMI        = 0,
	FAULT_TYPE_HARDFAULT  = 1,
	FAULT_TYPE_MEMMANAGE  = 2,
	FAULT_TYPE_BUSFAULT   = 3,
	FAULT_TYPE_USAGEFAULT = 4,
} fault_type_t;

/* FAULT_RECORD_MAGIC | fault_type occupies fault_record_t.magic; a reader
 * that finds anything else in RTC_BKP0 knows the backup registers hold
 * whatever they powered up with, not a real record. */
#define FAULT_RECORD_MAGIC 0xFA170000u

typedef struct {
	uint32_t magic;          /* FAULT_RECORD_MAGIC | fault_type */
	uint32_t pc;             /* stacked PC: the faulting instruction */
	uint32_t lr;             /* stacked LR: return address of the faulting context */
	uint32_t status_primary; /* CFSR (CPU faults) or SYSCFG_STAT (NMI) */
	uint32_t
	    status_secondary; /* HFSR (CPU faults) or FMC_ECCCS (NMI, 0 if flash ECC didn't fire) */
	uint32_t mmfar;       /* valid iff CFSR.MMARVALID, else 0 */
	uint32_t bfar;        /* valid iff CFSR.BFARVALID, else 0 */
} fault_record_t;

/* CFSR bit positions/masks this module needs, duplicated from the CMSIS
 * core_cm33.h SCB_CFSR_* names (not included here -- see file header) so a
 * host build has no vendor dependency at all. */
#define FAULT_DIAG_CFSR_MMARVALID_MSK 0x00000080u /* SCB_CFSR_MMARVALID_Msk */
#define FAULT_DIAG_CFSR_BFARVALID_MSK 0x00008000u /* SCB_CFSR_BFARVALID_Msk */

/* Builds a record for one of the four CPU fault exceptions (HardFault,
 * MemManage, BusFault, UsageFault).  mmfar_raw/bfar_raw are whatever SCB-
 * >MMFAR/BFAR currently read; this function is the ONLY thing that decides
 * whether they are meaningful, via the CFSR validity bits above. */
void fault_diag_build_cpu(fault_record_t *rec,
                          fault_type_t    fault_type,
                          uint32_t        pc,
                          uint32_t        lr,
                          uint32_t        cfsr,
                          uint32_t        hfsr,
                          uint32_t        mmfar_raw,
                          uint32_t        bfar_raw);

/* Builds a record for NMI.  fmc_eccs_raw is whatever FMC_ECCCS currently
 * reads; recorded only when syscfg_stat's FLASHECCIF bit (BIT(2), see
 * SYSCFG_STAT_FLASHECCIF) is set, else status_secondary comes back 0. */
#define FAULT_DIAG_SYSCFG_STAT_FLASHECCIF_MSK 0x00000004u /* SYSCFG_STAT_FLASHECCIF */

void fault_diag_build_nmi(fault_record_t *rec,
                          uint32_t        pc,
                          uint32_t        lr,
                          uint32_t        syscfg_stat,
                          uint32_t        fmc_eccs_raw);

#endif /* GD32_BRIDGE_HAL_GD32_FAULT_DIAG_H */
