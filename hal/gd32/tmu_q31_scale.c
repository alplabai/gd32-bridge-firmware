/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * See tmu_q31_scale.h for the derivation of the domain this bridge's Q31
 * wire format can actually reach and why each threshold sits where it
 * does.
 */

#include "tmu_q31_scale.h"

/* UM p.394 Table 14-26 band boundary: real x = 0.75.
 * Q31 word = round(0.75 * 2^31). */
#define TMU_Q31_SQRT_BAND1_THRESHOLD 0x60000000

unsigned tmu_q31_sqrt_factor(uint32_t q31_operand)
{
	const int32_t x = (int32_t)q31_operand;
	return (x < (int32_t)TMU_Q31_SQRT_BAND1_THRESHOLD) ? 0u : 1u;
}

/* UM p.393 Table 14-23's own floor is x=0.107 (Q31 word 0x0DB22D0F),
 * but the real cutoff for this wire format is e^-1 -- ln(x) = -1.0
 * there, the most negative value signed Q31 can hold.  Q31 word =
 * round(e^-1 * 2^31), computed once offline in double precision
 * (Python: round(math.exp(-1) * 2**31) == 0x2F16AC6C). */
#define TMU_Q31_LN_MIN_THRESHOLD 0x2F16AC6C

int tmu_q31_ln_representable(uint32_t q31_operand)
{
	const int32_t x = (int32_t)q31_operand;
	return x > (int32_t)TMU_Q31_LN_MIN_THRESHOLD;
}

/* Q31 word = round(asinh(1) * 2^31), computed once offline in double
 * precision (Python: round(math.asinh(1.0) * 2**31) == 0x70D0D986). */
#define TMU_Q31_SINH_LIMIT 0x70D0D986

int tmu_q31_sinh_representable(uint32_t q31_operand)
{
	const int32_t x = (int32_t)q31_operand;
	return x > -(int32_t)TMU_Q31_SINH_LIMIT && x < (int32_t)TMU_Q31_SINH_LIMIT;
}
