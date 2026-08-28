/* SPDX-License-Identifier: Apache-2.0 */
/*
 * A stand-in for the six Zephyr ztest macros these suites use.
 *
 * WHY: the suites came from alp-sdk, where they ran under twister on
 * native_sim.  The firmware they exercise now lives here (ADR 0031), and the
 * code under test -- ota.c, crc32.c, transport_spi.c, protocol.c,
 * bootloader.c -- is plain freestanding C with no Zephyr dependency at all.
 * Only the TEST HARNESS was Zephyr.  Pulling a Zephyr checkout into this
 * repo's CI purely to provide six macros would cost minutes per job and couple
 * a bare-metal firmware repo to a west manifest; a host compiler already has
 * everything else these tests need.
 *
 * The suites are byte-identical to their alp-sdk originals, so this header has
 * to match ztest's signatures and SEMANTICS exactly, not a tidier equivalent.
 */
#ifndef ZTEST_SHIM_H
#define ZTEST_SHIM_H

#include <setjmp.h>
#include <stdio.h>
#include <string.h>

struct ztest_case {
	const char *name;
	void (*fn)(void);
	struct ztest_case *next;
};

void ztest_register(struct ztest_case *c);
void ztest_fail(const char *file, int line, const char *detail, ...);

/*
 * A failed assert ABORTS THE WHOLE CASE -- ztest's semantic -- not merely the
 * function the assert sits in.  That distinction is load-bearing: these suites
 * call zassert_* from inside NON-VOID helpers (test_ota.c's _host_ptr and
 * ota_state_now), where a plain return is both a compile error under
 * -Werror=return-type and the wrong behaviour, since the caller would carry on
 * with an indeterminate value after the failure.
 */
extern jmp_buf ztest_jmp;
void           ztest_abort(void);

/* Set by the runner so a failure can name the case it came from. */
extern const char *ztest_current;
extern int         ztest_failures;

/*
 * ztest's assert macros take an OPTIONAL trailing message + printf args.  The
 * empty-literal concatenation below makes the zero-argument case work without
 * relying on the comma-swallowing GNU extension: with no message it collapses
 * to the empty string, and with one it concatenates onto the leading literal.
 * Every ztest message in these suites is a string literal, which is what makes
 * that legal.
 */
#define ZTEST_NOTE(...) ztest_note("" __VA_ARGS__)
void ztest_note(const char *fmt, ...);

/*
 * ZTEST_SUITE(name, predicate, setup, before, after, teardown) -- every use in
 * these suites passes NULL for all five hooks, so there is nothing to run.  It
 * expands to a forward declaration so the trailing semicolon at file scope
 * stays a declaration rather than a stray semicolon.
 */
#define ZTEST_SUITE(suite, ...) struct ztest_suite_##suite##_unused

#define ZTEST(suite, name) \
	static void                              suite##_##name(void); \
	static struct ztest_case                 ztest_case_##suite##_##name = { #suite "." #name, \
		                                                                     suite##_##name, \
		                                                                     NULL }; \
	__attribute__((constructor)) static void ztest_reg_##suite##_##name(void) \
	{ \
		ztest_register(&ztest_case_##suite##_##name); \
	} \
	static void suite##_##name(void)

#define zassert_true(cond, ...) \
	do { \
		if (!(cond)) { \
			ztest_fail(__FILE__, __LINE__, "expected true: %s", #cond); \
			ZTEST_NOTE(__VA_ARGS__); \
			ztest_abort(); \
		} \
	} while (0)

#define zassert_false(cond, ...) \
	do { \
		if ((cond)) { \
			ztest_fail(__FILE__, __LINE__, "expected false: %s", #cond); \
			ZTEST_NOTE(__VA_ARGS__); \
			ztest_abort(); \
		} \
	} while (0)

/*
 * Both sides are widened to long long once.  ztest compares arithmetically, so
 * a signed status against an unsigned literal must not hit the usual-arithmetic
 * -conversions trap that a bare == between the two would.
 */
#define zassert_equal(a, b, ...) \
	do { \
		long long za_ = (long long)(a); \
		long long zb_ = (long long)(b); \
		if (za_ != zb_) { \
			ztest_fail(__FILE__, __LINE__, "%s != %s (%lld vs %lld)", #a, #b, za_, zb_); \
			ZTEST_NOTE(__VA_ARGS__); \
			ztest_abort(); \
		} \
	} while (0)

#define zassert_mem_equal(a, b, sz, ...) \
	do { \
		if (memcmp((a), (b), (size_t)(sz)) != 0) { \
			ztest_fail(__FILE__, __LINE__, "memcmp(%s, %s, %s) != 0", #a, #b, #sz); \
			ZTEST_NOTE(__VA_ARGS__); \
			ztest_abort(); \
		} \
	} while (0)

#endif /* ZTEST_SHIM_H */
