/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Pure Q31 FACTOR[2:0] band selection for the GD32G5x3 TMU's SQRT and LN
 * CORDIC modes (mode9 / mode8), plus the SINH representability check
 * (mode6's FACTOR is manual-fixed, no band lookup needed).  No vendor
 * register access here on purpose -- kept in its own translation unit
 * with no gd32g5x3.h include, so a HOST build can link and test the band
 * boundaries directly (issue #77: no CI job compiles hal/gd32/tmu.c
 * itself, since it needs the Cortex-M33 vendor headers).
 *
 * Every threshold here is the Q31 (signed 1.31, value = word / 2^31) word
 * for a boundary derived from GD32G553 User Manual Rev1.2 p.393 Table
 * 14-23 and p.394 Tables 14-25/14-26, restricted to the domain this
 * bridge's wire protocol can actually encode: format=0 (Q31) is
 * full-scale +-1.0 for BOTH inputs and outputs
 * (docs/gd32-bridge-protocol.md SS3.12), with no exponent/factor field on
 * the wire, so an operand or a post-scaled result that needs
 * |value| >= 1.0 cannot be encoded at all -- regardless of the manual's
 * own illustrative worked-example inputs (8, 2, 1.0), which are
 * real-world values that get software pre-scaled before conversion
 * precisely because they don't fit fixed point in the first place.
 */

#ifndef GD32_BRIDGE_TMU_Q31_SCALE_H
#define GD32_BRIDGE_TMU_Q31_SCALE_H

#include <stdint.h>

/* SQRT (mode9), UM Table 14-26, restricted to the Q31-reachable
 * sub-range x in [0,1): the table's third band (1.75 <= x < 2.341,
 * f=2) needs an operand >= 1 and is unreachable in this wire format.
 *
 *   0.027 < x < 0.75   -> FACTOR 3'b000 (f=0)
 *   0.75 <= x < 1.75   -> FACTOR 3'b001 (f=1)   (only the < 1 slice reachable)
 *
 * sqrt(x) for x in [0,1) is itself in [0,1) (sqrt is monotone increasing
 * and sqrt(1)=1), so every legally encodable operand is representable --
 * this never needs to refuse.  Negative operands are a separate,
 * pre-existing hardware-detected error (TMU_FLAG_OVRF), unrelated to
 * FACTOR selection; this function returns f=0 for a negative-looking
 * word, matching the unconditional FACTOR this driver used for every
 * mode before the FACTOR fix, so that already-verified path is
 * unaffected. */
unsigned tmu_q31_sqrt_factor(uint32_t q31_operand);

/* LN (mode8), UM Table 14-23's only Q31-reachable band is
 * 0.107 <= x < 1 (FACTOR 3'b001/f=1; the f=2..4 bands all need an
 * operand >= 1, unreachable here).  But ln(x) over that band ranges
 * (-2.235, 0], and Q31 can only hold down to -1.0 -- representable iff
 * x >= e^-1 (~0.3679), the real cutoff for this wire format and
 * stricter than the table's own 0.107 floor.
 *
 * Returns nonzero (representable -- FACTOR is always 3'b001/f=1 in
 * this band) or 0 (refuse -- x is <= 0 or below e^-1, either an
 * invalid ln() domain or a real result that would not fit in signed
 * Q31). */
int tmu_q31_ln_representable(uint32_t q31_operand);

/* SINH (mode6), UM Table 14-18: FACTOR is manual-mandated fixed
 * 3'b001 (f=1), not operand-dependent -- no band lookup needed, only
 * a representability check.  sinh is monotone increasing and odd;
 * sinh(x) fits signed Q31 [-1,1) iff |x| < asinh(1) (~0.8814), i.e.
 * iff |sinh(x)| < 1.
 *
 * Returns nonzero (representable) or 0 (refuse). */
int tmu_q31_sinh_representable(uint32_t q31_operand);

#endif /* GD32_BRIDGE_TMU_Q31_SCALE_H */
