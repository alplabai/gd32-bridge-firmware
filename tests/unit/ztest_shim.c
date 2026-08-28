/* SPDX-License-Identifier: Apache-2.0 */
/* Runner for the ztest shim -- see ztest_shim.h for why this exists. */
#include "ztest_shim.h"

#include <stdarg.h>

static struct ztest_case *cases;
jmp_buf                   ztest_jmp;
const char               *ztest_current = "(none)";
int                       ztest_failures;

void ztest_register(struct ztest_case *c)
{
	/*
	 * Append rather than push-front so cases run in registration order.
	 * Constructor order within one translation unit is source order, which
	 * keeps the output reading top-to-bottom like the suite file.
	 */
	struct ztest_case **tail = &cases;

	while (*tail != NULL) {
		tail = &(*tail)->next;
	}
	*tail   = c;
	c->next = NULL;
}

void ztest_fail(const char *file, int line, const char *detail, ...)
{
	va_list ap;

	ztest_failures++;
	fprintf(stderr, "  FAIL %s (%s:%d): ", ztest_current, file, line);
	va_start(ap, detail);
	vfprintf(stderr, detail, ap);
	va_end(ap);
	fputc('\n', stderr);
}

void ztest_abort(void)
{
	longjmp(ztest_jmp, 1);
}

void ztest_note(const char *fmt, ...)
{
	va_list ap;

	if (fmt == NULL || fmt[0] == '\0') {
		return;
	}
	fputs("       note: ", stderr);
	va_start(ap, fmt);
	vfprintf(stderr, fmt, ap);
	va_end(ap);
	fputc('\n', stderr);
}

int main(void)
{
	int total        = 0;
	int failed_cases = 0;

	for (struct ztest_case *c = cases; c != NULL; c = c->next) {
		int before = ztest_failures;

		ztest_current = c->name;
		total++;
		if (setjmp(ztest_jmp) == 0) {
			c->fn();
		}
		if (ztest_failures > before) {
			failed_cases++;
		} else {
			printf("  PASS %s\n", c->name);
		}
	}

	printf("%d/%d cases passed\n", total - failed_cases, total);
	/*
	 * Zero registered cases is a FAILURE, not a pass: it means the
	 * constructors never ran (the linker dropped the objects, or the suite
	 * was excluded from the build).  A harness that reports success when it
	 * ran nothing is exactly how coverage disappears unnoticed.
	 */
	if (total == 0) {
		fprintf(stderr, "no test cases registered -- harness is broken\n");
		return 2;
	}
	return failed_cases == 0 ? 0 : 1;
}
