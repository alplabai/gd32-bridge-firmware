/* SPDX-License-Identifier: Apache-2.0
 *
 * Host unit tests for hal/gd32/fault_diag.c -- the pure CFSR/SYSCFG_STAT ->
 * fault_record_t decode logic behind the fault handlers (issue #36).  No
 * vendor register header involved: these drive the module with raw
 * register-shaped uint32_t values and check the two decisions that matter --
 * whether MMFAR/BFAR get recorded or zeroed, and whether FMC_ECCCS gets
 * recorded or zeroed -- against the CFSR / SYSCFG_STAT validity bits.
 */

#include <zephyr/ztest.h>

#include "fault_diag.h"

ZTEST_SUITE(gd32_bridge_fault_diag, NULL, NULL, NULL, NULL, NULL);

/* ---- CPU faults (HardFault / MemManage / BusFault / UsageFault) ------ */

ZTEST(gd32_bridge_fault_diag, test_cpu_magic_encodes_fault_type)
{
	fault_record_t rec;

	fault_diag_build_cpu(&rec, FAULT_TYPE_BUSFAULT, 0x08001234u, 0x08005678u, 0u, 0u, 0u, 0u);
	zassert_equal(rec.magic,
	              FAULT_RECORD_MAGIC | (uint32_t)FAULT_TYPE_BUSFAULT,
	              "magic must encode the fault type in its low byte");
	zassert_equal(rec.pc, 0x08001234u);
	zassert_equal(rec.lr, 0x08005678u);
}

ZTEST(gd32_bridge_fault_diag, test_mmfar_recorded_only_when_mmarvalid_set)
{
	fault_record_t rec;

	/* MMARVALID (CFSR bit 7) set: MMFAR is meaningful, must be recorded. */
	fault_diag_build_cpu(
	    &rec, FAULT_TYPE_MEMMANAGE, 0u, 0u, FAULT_DIAG_CFSR_MMARVALID_MSK, 0u, 0xDEADBEEFu, 0u);
	zassert_equal(rec.mmfar, 0xDEADBEEFu, "MMARVALID set: MMFAR must be captured");

	/* MMARVALID clear: whatever MMFAR currently reads is a stale leftover
	 * from a previous fault -- must come back as 0, not the raw value. */
	fault_diag_build_cpu(&rec, FAULT_TYPE_MEMMANAGE, 0u, 0u, 0u, 0u, 0xDEADBEEFu, 0u);
	zassert_equal(rec.mmfar, 0u, "MMARVALID clear: stale MMFAR must be dropped, not recorded");
}

ZTEST(gd32_bridge_fault_diag, test_bfar_recorded_only_when_bfarvalid_set)
{
	fault_record_t rec;

	fault_diag_build_cpu(
	    &rec, FAULT_TYPE_BUSFAULT, 0u, 0u, FAULT_DIAG_CFSR_BFARVALID_MSK, 0u, 0u, 0xCAFEF00Du);
	zassert_equal(rec.bfar, 0xCAFEF00Du, "BFARVALID set: BFAR must be captured");

	fault_diag_build_cpu(&rec, FAULT_TYPE_BUSFAULT, 0u, 0u, 0u, 0u, 0u, 0xCAFEF00Du);
	zassert_equal(rec.bfar, 0u, "BFARVALID clear: stale BFAR must be dropped, not recorded");
}

ZTEST(gd32_bridge_fault_diag, test_cpu_status_registers_recorded_raw)
{
	fault_record_t rec;

	fault_diag_build_cpu(&rec, FAULT_TYPE_HARDFAULT, 0u, 0u, 0x00020000u, 0x40000000u, 0u, 0u);
	zassert_equal(rec.status_primary, 0x00020000u, "status_primary must be the raw CFSR");
	zassert_equal(rec.status_secondary, 0x40000000u, "status_secondary must be the raw HFSR");
}

/* ---- NMI ---------------------------------------------------------------
 *
 * Five sources land on this one vector (issue #36): CKMNMIIF, FLASHECCIF,
 * SRAM0/1ECCMEIF, TCMSRAMECCMEIF, all bits of SYSCFG_STAT.  Only FLASHECCIF
 * has a second, concretely reachable status register (FMC_ECCCS) worth
 * decoding here.
 */

ZTEST(gd32_bridge_fault_diag, test_nmi_fmc_eccs_recorded_only_when_flashECCIF_set)
{
	fault_record_t rec;

	/* FLASHECCIF (SYSCFG_STAT bit 2) set: FMC_ECCCS is the live fault
	 * source and must be captured. */
	fault_diag_build_nmi(
	    &rec, 0x08001000u, 0x08002000u, FAULT_DIAG_SYSCFG_STAT_FLASHECCIF_MSK, 0x800A1234u);
	zassert_equal(rec.status_primary, FAULT_DIAG_SYSCFG_STAT_FLASHECCIF_MSK);
	zassert_equal(rec.status_secondary, 0x800A1234u, "FLASHECCIF set: FMC_ECCCS must be captured");
	zassert_equal(rec.magic, FAULT_RECORD_MAGIC | (uint32_t)FAULT_TYPE_NMI);

	/* A different source fired (e.g. CKMNMIIF, bit 3) -- FMC_ECCCS is
	 * whatever it happened to read last and must NOT be recorded. */
	fault_diag_build_nmi(&rec, 0u, 0u, 0x00000008u /* CKMNMIIF */, 0x800A1234u);
	zassert_equal(rec.status_primary, 0x00000008u);
	zassert_equal(rec.status_secondary,
	              0u,
	              "FLASHECCIF clear: stale FMC_ECCCS must be dropped, not recorded");
}

ZTEST(gd32_bridge_fault_diag, test_nmi_pc_lr_recorded)
{
	fault_record_t rec;

	fault_diag_build_nmi(&rec, 0x08001000u, 0x08002000u, 0u, 0u);
	zassert_equal(rec.pc, 0x08001000u);
	zassert_equal(rec.lr, 0x08002000u);
}
