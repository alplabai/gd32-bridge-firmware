/* SPDX-License-Identifier: Apache-2.0
 *
 * gh#304: main() must call ota_boot_init() after bridge_hw_init() (it needs
 * the FMC HAL) and before either transport starts (no frame may reach
 * protocol_dispatch() before s_trial is decided).  Compiles the REAL
 * src/main.c with main renamed, records the call order, and escapes the
 * infinite loop from the first bridge_hw_tick().
 */

#include <setjmp.h>
#include <stdbool.h>
#include <string.h>

#include <zephyr/ztest.h>

#include "ota.h"
#include "transport.h"

int fw_main(void);

static jmp_buf escape;
static int     n;
static char    seq[8];

static void note(char c)
{
	seq[n++] = c;
}

void bridge_hw_init(void)
{
	note('H');
}
static bool work_pending;
static int  wfi_calls;
bool        bridge_hw_work_pending(void)
{
	return work_pending;
}
void __WFI(void)
{
	wfi_calls++;
}
void bridge_hw_tick(void)
{
	longjmp(escape, 1);
}
void ota_boot_init(void)
{
	note('O');
}
void transport_spi_init(void)
{
	note('S');
}
void transport_i2c_init(void)
{
	note('I');
}

ZTEST_SUITE(main_boot_order, NULL, NULL, NULL, NULL, NULL);

ZTEST(main_boot_order, test_ota_boot_init_between_hw_and_transports)
{
	n = 0;
	memset(seq, 0, sizeof(seq));
	if (setjmp(escape) == 0) {
		(void)fw_main();
	}
	zassert_true(strcmp(seq, "HOSI") == 0, "want bridge_hw_init, ota_boot_init, spi, i2c");
}

/* Host note: main_irq_off()/main_irq_on() are no-ops in a hosted build, so this
 * proves only the skip-WFI branch.  That the mask surrounds the check (the
 * lost-wakeup fix proper) is a bench property, proven on 2026W38-0009.
 *
 * A SET queued by an ISR after the last tick's check must not be left behind a
 * WFI: with work pending the loop goes straight to the tick; with none it
 * sleeps first. */
ZTEST(main_boot_order, test_no_sleep_while_deferred_work_is_pending)
{
	work_pending = true;
	wfi_calls    = 0;
	if (setjmp(escape) == 0) {
		(void)fw_main();
	}
	zassert_equal(wfi_calls, 0, "pending work: tick runs, no WFI");

	work_pending = false;
	if (setjmp(escape) == 0) {
		(void)fw_main();
	}
	zassert_equal(wfi_calls, 1, "idle: sleeps once, then ticks");
}
