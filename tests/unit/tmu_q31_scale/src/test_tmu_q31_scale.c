/* SPDX-License-Identifier: Apache-2.0
 *
 * HOST unit tests for the Q31 TMU FACTOR band-selection logic
 * (hal/gd32/tmu_q31_scale.c).  This is the one slice of the SQRT/LN/SINH
 * fix (alplabai/gd32-bridge-firmware#46, PR #85) that host-testable at
 * all: hal/gd32/tmu.c itself needs the Cortex-M33 vendor register headers
 * and is compiled by no CI job (issue #77), but the band arithmetic has
 * no hardware dependency, so it was split into its own translation unit
 * purely so this suite could link and exercise it directly.
 *
 * Every boundary word below is independently derived in Python (double
 * precision, not the firmware) -- see the comments at each case and
 * hal/gd32/tmu_q31_scale.c's derivation comments for the matching
 * one-liners.
 */

#include <zephyr/ztest.h>

#include "tmu_q31_scale.h"

/* ---- SQRT (UM p.394 Table 14-26) ------------------------------------- */

ZTEST(tmu_q31_scale, test_sqrt_band0_low)
{
	/* x=0.25 (0x20000000): the alp-sdk functional regression case
	 * (v2n-gd32-bridge-functional t_tmu_q31_sqrt) -- 0.25 < 0.75, so
	 * FACTOR must stay f=0, exactly what the driver wrote
	 * unconditionally before this fix broke it. */
	zassert_equal(tmu_q31_sqrt_factor(0x20000000u), 0u);
}

ZTEST(tmu_q31_scale, test_sqrt_band0_zero)
{
	/* x=0: below the table's 0.027 accuracy floor, but nothing in the
	 * general constraint (x/2^f < 1-1/2^(f+2)) sets a lower bound --
	 * f=0 stays correct, sqrt(0)=0 stays representable. */
	zassert_equal(tmu_q31_sqrt_factor(0x00000000u), 0u);
}

ZTEST(tmu_q31_scale, test_sqrt_band0_upper_boundary)
{
	/* x just below 0.75 (0x5FFFFFFF ~= 0.749999999767) is still band 0. */
	zassert_equal(tmu_q31_sqrt_factor(0x5FFFFFFFu), 0u);
}

ZTEST(tmu_q31_scale, test_sqrt_band1_at_boundary)
{
	/* x=0.75 (0x60000000) is the INCLUSIVE start of band 1 per Table
	 * 14-26 ("0.75 <= x < 1.75"). */
	zassert_equal(tmu_q31_sqrt_factor(0x60000000u), 1u);
}

ZTEST(tmu_q31_scale, test_sqrt_band1_near_one)
{
	/* x=0x7FFFFFFF (~0.9999999995, Q31's max positive value) is the
	 * highest operand this wire format can ever present -- still
	 * squarely inside band 1 (band 1 covers up to x<1.75). */
	zassert_equal(tmu_q31_sqrt_factor(0x7FFFFFFFu), 1u);
}

ZTEST(tmu_q31_scale, test_sqrt_negative_defaults_to_band0)
{
	/* A negative operand is an invalid sqrt() domain -- caught by the
	 * TMU's hardware OVRF flag in bridge_hw_tmu_compute(), not here.
	 * This function must still return SOME factor rather than crash;
	 * f=0 matches the unconditional FACTOR this driver wrote for every
	 * mode before the FACTOR fix, so the already-verified negative-
	 * input rejection path is unaffected by this change. */
	zassert_equal(tmu_q31_sqrt_factor(0x80000000u), 0u); /* -1.0 */
}

/* ---- LN (UM p.393 Table 14-23) ---------------------------------------- */

ZTEST(tmu_q31_scale, test_ln_representable_half)
{
	/* x=0.5 (0x40000000): ln(0.5) = -0.6931... > -1.0, representable. */
	zassert_true(tmu_q31_ln_representable(0x40000000u));
}

ZTEST(tmu_q31_scale, test_ln_representable_near_one)
{
	/* x=0x7FFFFFFF (~0.9999999995): ln(x) ~= -4.7e-10, representable. */
	zassert_true(tmu_q31_ln_representable(0x7FFFFFFFu));
}

ZTEST(tmu_q31_scale, test_ln_refuses_at_e_inv_boundary)
{
	/* x = round(e^-1 * 2^31) = 0x2F16AC6C: ln(x) rounds to just past
	 * -1.0 once quantized to a Q31 word (Python:
	 * math.log(0x2F16AC6C / 2**31) == -1.000000000444) -- refuse at
	 * the boundary itself, only accept strictly above it. */
	zassert_false(tmu_q31_ln_representable(0x2F16AC6Cu));
}

ZTEST(tmu_q31_scale, test_ln_representable_just_above_boundary)
{
	/* One Q31 ULP above the e^-1 threshold: representable. */
	zassert_true(tmu_q31_ln_representable(0x2F16AC6Du));
}

ZTEST(tmu_q31_scale, test_ln_refuses_below_e_inv)
{
	/* x=0.3 (0x26666666): below e^-1 (~0.3679) -- inside the table's
	 * own 0.107 operand floor, but the post-scaled ln(0.3)=-1.204 does
	 * not fit signed Q31 [-1,1). */
	zassert_false(tmu_q31_ln_representable(0x26666666u));
}

ZTEST(tmu_q31_scale, test_ln_refuses_zero_and_negative)
{
	/* x<=0 is outside ln()'s domain entirely. */
	zassert_false(tmu_q31_ln_representable(0x00000000u));
	zassert_false(tmu_q31_ln_representable(0x80000000u)); /* -1.0 */
}

/* ---- SINH (UM p.391 Table 14-18) --------------------------------------- */

ZTEST(tmu_q31_scale, test_sinh_representable_at_zero)
{
	zassert_true(tmu_q31_sinh_representable(0x00000000u));
}

ZTEST(tmu_q31_scale, test_sinh_representable_half)
{
	/* x=0.5 (0x40000000): sinh(0.5) = 0.5211..., representable. */
	zassert_true(tmu_q31_sinh_representable(0x40000000u));
}

ZTEST(tmu_q31_scale, test_sinh_representable_negative_half)
{
	/* x=-0.7: sinh is odd, sinh(-0.7) = -0.7586..., representable. */
	zassert_true(tmu_q31_sinh_representable(0xA6666666u));
}

ZTEST(tmu_q31_scale, test_sinh_refuses_at_asinh1_boundary)
{
	/* x = round(asinh(1) * 2^31) = 0x70D0D986: sinh(x) rounds to just
	 * past 1.0 once quantized (Python: math.sinh(0x70D0D986 / 2**31)
	 * == 1.000000000064) -- refuse at the boundary, only accept
	 * strictly inside it. */
	zassert_false(tmu_q31_sinh_representable(0x70D0D986u));
	zassert_false(tmu_q31_sinh_representable(0x8F2F267Au)); /* -asinh(1) */
}

ZTEST(tmu_q31_scale, test_sinh_representable_just_inside_boundary)
{
	zassert_true(tmu_q31_sinh_representable(0x70D0D985u));
	zassert_true(tmu_q31_sinh_representable(0x8F2F267Bu));
}

ZTEST(tmu_q31_scale, test_sinh_refuses_near_one)
{
	/* x=0x7FFFFFFF (~0.9999999995): sinh(x) = 1.1752..., well past 1.0. */
	zassert_false(tmu_q31_sinh_representable(0x7FFFFFFFu));
}

ZTEST_SUITE(tmu_q31_scale, NULL, NULL, NULL, NULL, NULL);
