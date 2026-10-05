/* SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for src/boot_config.[ch] and CMD_BOOT_CONFIG (alp-sdk #2697): the
 * opt-in "drive PD11 / E1M IO29 SDIO_MUX_EN high at boot" flag, persisted in
 * two A/B record pages.
 *
 * Flash is two host pages behind the weak ota_fmc_* seams, with a per-
 * doubleword "uncorrectable ECC" model: ota_fmc_read_safe() reports false for
 * such a doubleword instead of returning data (the real HAL does the same
 * rather than raising the flash-ECC NMI).  A "power cycle" is
 * bridge_hw_fake_reset() (HAL/pad state lost) + boot_config_apply(), the call
 * bridge_hw_init() makes, with the flash buffers kept.
 */

#include <zephyr/ztest.h>

#include "bridge_hw_fake.h"
#include "boot_config.h"
#include "protocol.h"

#define NDW (OTA_PAGE_SIZE / 8u)

static uint8_t          g_page[2][OTA_PAGE_SIZE];
static bool             g_bad[2][NDW]; /* doubleword reads as uncorrectable */
static uint32_t         g_erase_calls;
static uint32_t         g_bad_reads; /* read_safe calls that returned false */
static uint32_t         g_payload_reads;
static bool             g_supported, g_busy, g_write_safe;
static ota_fmc_result_t g_erase_rv, g_program_rv;
static int  g_ops_before_cut; /* flash ops that complete before a power cut; <0 = none */
static bool g_cut;            /* the cut happened: every later op fails */

bool ota_fmc_funnel_busy(void)
{
	return g_busy;
}
bool ota_fmc_config_write_safe(void)
{
	return g_write_safe;
}
bool ota_fmc_supported(void)
{
	return g_supported;
}

static int page_of(uint32_t base)
{
	zassert_true(base == OTA_CONFIG_REC0 || base == OTA_CONFIG_REC1, "only config pages: %x", base);
	return base == OTA_CONFIG_REC1;
}

/* Returns true when this op is cut by power loss (the page is left torn). */
static bool power_cut_now(void)
{
	if (g_cut) return true;
	if (g_ops_before_cut == 0) {
		g_cut = true;
		return true;
	}
	if (g_ops_before_cut > 0) g_ops_before_cut--;
	return false;
}

ota_fmc_result_t ota_fmc_erase_range(uint32_t base, uint32_t len)
{
	const int p = page_of(base);
	zassert_equal(len, OTA_PAGE_SIZE);
	g_erase_calls++;
	if (power_cut_now()) { /* cut mid-erase: the page is garbage + uncorrectable */
		memset(g_page[p], 0x5A, OTA_PAGE_SIZE);
		for (uint32_t i = 0; i < NDW; i++)
			g_bad[p][i] = true;
		return OTA_FMC_RESULT_ERROR;
	}
	if (g_erase_rv != OTA_FMC_RESULT_OK) return g_erase_rv;
	memset(g_page[p], 0xFF, OTA_PAGE_SIZE);
	memset(g_bad[p], 0, sizeof g_bad[p]);
	return OTA_FMC_RESULT_OK;
}

ota_fmc_result_t ota_fmc_program(uint32_t addr, const uint8_t *data, size_t len)
{
	const uint32_t base = addr & ~(OTA_PAGE_SIZE - 1u);
	const int      p    = page_of(base);
	const uint32_t off  = addr - base;
	zassert_equal(off % 8u, 0u, "doubleword program");
	zassert_true(off + len <= 16u, "records live in the first 16 bytes");
	if (power_cut_now()) { /* cut mid-program: the doubleword is torn and uncorrectable */
		g_bad[p][off / 8u] = true;
		return OTA_FMC_RESULT_ERROR;
	}
	if (g_program_rv != OTA_FMC_RESULT_OK) return g_program_rv;
	memcpy(&g_page[p][off], data, len);
	return OTA_FMC_RESULT_OK;
}

bool ota_fmc_read_safe(uint32_t addr, void *dst, size_t len)
{
	const uint32_t base = addr & ~(OTA_PAGE_SIZE - 1u);
	const int      p    = page_of(base);
	const uint32_t off  = addr - base;
	zassert_equal(off % 8u, 0u);
	zassert_equal(len % 8u, 0u);
	for (size_t i = 0; i < len / 8u; i++) {
		if (g_bad[p][off / 8u + i]) {
			g_bad_reads++;
			return false;
		}
	}
	if (off == 0u) g_payload_reads++;
	memcpy(dst, &g_page[p][off], len);
	return true;
}

static void fresh_unit(void) /* factory-new: both pages erased, HAL reset */
{
	memset(g_page, 0xFF, sizeof g_page);
	memset(g_bad, 0, sizeof g_bad);
	g_erase_calls = g_bad_reads = g_payload_reads = 0u;
	g_supported                                   = true;
	g_busy                                        = false;
	g_write_safe                                  = true;
	g_ops_before_cut                              = -1;
	g_cut                                         = false;
	g_erase_rv = g_program_rv = OTA_FMC_RESULT_OK;
	bridge_hw_fake_reset();
	boot_config_load();
}

static void power_cycle(void) /* HAL/pad state gone, flash kept, then the boot-time apply */
{
	g_ops_before_cut = -1;
	g_cut            = false;
	bridge_hw_fake_reset();
	boot_config_apply();
}

static gd32_bridge_status_t cfg(gd32_bridge_link_t link, uint8_t op, uint32_t flags, uint32_t *out)
{
	const uint8_t req[5] = {
		op, (uint8_t)flags, (uint8_t)(flags >> 8), (uint8_t)(flags >> 16), (uint8_t)(flags >> 24)
	};
	uint8_t              reply[8];
	size_t               n = 0u;
	gd32_bridge_status_t st =
	    protocol_dispatch(link, CMD_BOOT_CONFIG, req, sizeof req, reply, sizeof reply, &n);
	if (st == STATUS_OK) {
		zassert_equal(n, 4u);
		*out = (uint32_t)reply[0] | ((uint32_t)reply[1] << 8) | ((uint32_t)reply[2] << 16) |
		       ((uint32_t)reply[3] << 24);
	}
	return st;
}

/* SET over the wire, then the main loop's tick commits it. */
static gd32_bridge_status_t set_and_commit(uint32_t flags, uint32_t *out)
{
	const gd32_bridge_status_t st = cfg(GD32_BRIDGE_LINK_SPI, 1u, flags, out);
	boot_config_tick();
	return st;
}

#define PD11 BOOT_CONFIG_SDMUX_EN_PAD_MASK
#define F1   BOOT_CONFIG_FLAG_SDMUX_EN_HIGH

ZTEST(boot_config, test_default_off_touches_no_pad)
{
	fresh_unit();
	boot_config_apply();
	zassert_equal(boot_config_flags(), 0u);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_GPIO_WRITE), 0u, "no flag = pad untouched");
}

ZTEST(boot_config, test_set_is_queued_never_flashed_in_dispatch)
{
	uint32_t v = 9u;
	fresh_unit();
	zassert_equal(cfg(GD32_BRIDGE_LINK_SPI, 1u, F1, &v), STATUS_OK, "accepted at once");
	zassert_equal(v, 0u, "reply = the still-stored (old) value");
	zassert_equal(g_erase_calls, 0u, "no flash op inside the transport ISR");
	zassert_equal(cfg(GD32_BRIDGE_LINK_SPI, 0u, 0u, &v), STATUS_OK);
	zassert_equal(v, 0u, "GET shows the old value until the tick commits");

	boot_config_tick(); /* main loop */
	zassert_equal(g_erase_calls, 1u);
	zassert_equal(cfg(GD32_BRIDGE_LINK_SPI, 0u, 0u, &v), STATUS_OK);
	zassert_equal(v, F1, "poll sees the stored value");
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_GPIO_WRITE), 0u, "SET never moves the pad");
}

ZTEST(boot_config, test_flag_persists_and_is_applied_at_boot)
{
	uint32_t v;
	fresh_unit();
	zassert_equal(set_and_commit(F1, &v), STATUS_OK);
	power_cycle();
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_GPIO_WRITE), 1u);
	zassert_equal(bridge_hw_fake_gpio_get_pads() & PD11, PD11, "PD11 (IO29) driven high");
	zassert_equal(bridge_hw_fake_gpio_get_pads() & ~PD11, 0u, "no other pad driven");
}

ZTEST(boot_config, test_records_alternate_pages_and_clear_restores_default)
{
	uint32_t             v;
	boot_config_record_t r;
	fresh_unit();
	set_and_commit(F1, &v);
	zassert_true(boot_config_read_page(OTA_CONFIG_REC0, &r), "first record in REC0");
	zassert_false(boot_config_read_page(OTA_CONFIG_REC1, &r));
	set_and_commit(0u, &v);
	zassert_true(boot_config_read_page(OTA_CONFIG_REC1, &r), "second record in REC1");
	zassert_equal(r.counter, 2u);
	power_cycle();
	zassert_equal(boot_config_flags(), 0u);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_GPIO_WRITE), 0u);
	set_and_commit(F1, &v);
	zassert_true(boot_config_read_page(OTA_CONFIG_REC0, &r), "third record back in REC0");
	zassert_equal(r.counter, 3u);
	power_cycle();
	zassert_equal(boot_config_flags(), F1);
}

ZTEST(boot_config, test_get_never_writes_flash_and_set_allowed_on_i2c)
{
	uint32_t v = 9u;
	fresh_unit();
	zassert_equal(cfg(GD32_BRIDGE_LINK_I2C, 0u, 0xFFFFFFFFu, &v), STATUS_OK, "GET ignores flags");
	zassert_equal(v, 0u);
	zassert_equal(g_erase_calls, 0u);
	zassert_equal(cfg(GD32_BRIDGE_LINK_I2C, 1u, F1, &v), STATUS_OK, "SET allowed on I2C");
	boot_config_tick();
	zassert_equal(boot_config_flags(), F1);
}

ZTEST(boot_config, test_unchanged_set_skips_flash)
{
	uint32_t v;
	fresh_unit();
	zassert_equal(set_and_commit(0u, &v), STATUS_OK, "default -> default");
	zassert_equal(g_erase_calls, 0u);
	zassert_equal(set_and_commit(F1, &v), STATUS_OK);
	zassert_equal(g_erase_calls, 1u);
	zassert_equal(set_and_commit(F1, &v), STATUS_OK, "replayed SET");
	zassert_equal(g_erase_calls, 1u, "same value: no second erase");
	g_busy       = true; /* an unchanged SET is fine while the funnel is busy / bank unsafe */
	g_write_safe = false;
	zassert_equal(set_and_commit(F1, &v), STATUS_OK);
	zassert_equal(g_erase_calls, 1u);
	/* store() itself also refuses to rewrite an equal value */
	zassert_equal(boot_config_store(F1), OTA_FMC_RESULT_OK);
	zassert_equal(g_erase_calls, 1u);
}

ZTEST(boot_config, test_busy_while_a_different_set_is_pending)
{
	uint32_t v, p;
	fresh_unit();
	zassert_equal(cfg(GD32_BRIDGE_LINK_SPI, 1u, F1, &v), STATUS_OK);
	zassert_equal(
	    cfg(GD32_BRIDGE_LINK_SPI, 1u, F1, &v), STATUS_OK, "same value replayed: accepted");
	zassert_true(boot_config_pending(&p));
	zassert_equal(cfg(GD32_BRIDGE_LINK_SPI, 1u, 0u, &v), STATUS_BUSY, "different SET while queued");
	boot_config_tick();
	zassert_false(boot_config_pending(&p));
	zassert_equal(cfg(GD32_BRIDGE_LINK_SPI, 1u, 0u, &v), STATUS_OK);
	boot_config_tick();
	zassert_equal(boot_config_flags(), 0u);
}

ZTEST(boot_config, test_tick_waits_for_the_fmc_funnel)
{
	uint32_t v;
	fresh_unit();
	cfg(GD32_BRIDGE_LINK_SPI, 1u, F1, &v);
	g_busy = true; /* OTA erase walk owns the FMC */
	boot_config_tick();
	zassert_equal(g_erase_calls, 0u, "not committed while the funnel is busy");
	zassert_equal(boot_config_flags(), 0u);
	g_busy = false;
	boot_config_tick();
	zassert_equal(boot_config_flags(), F1, "committed once the funnel is free");
}

ZTEST(boot_config, test_set_refused_when_bank_unsafe_or_no_fmc)
{
	uint32_t v;
	fresh_unit();
	g_write_safe = false; /* DBS = 0 or running from bank 1 */
	zassert_equal(cfg(GD32_BRIDGE_LINK_SPI, 1u, F1, &v), STATUS_NOSUPPORT);
	zassert_equal(cfg(GD32_BRIDGE_LINK_SPI, 0u, 0u, &v), STATUS_OK, "GET unaffected");
	g_write_safe = true;
	g_supported  = false;
	zassert_equal(cfg(GD32_BRIDGE_LINK_SPI, 1u, F1, &v), STATUS_NOSUPPORT);
	zassert_equal(g_erase_calls, 0u);
}

ZTEST(boot_config, test_validation)
{
	uint32_t v;
	uint8_t  reply[8];
	size_t   n;
	fresh_unit();
	zassert_equal(cfg(GD32_BRIDGE_LINK_SPI, 1u, 0x2u, &v), STATUS_INVAL, "unknown flag bit");
	zassert_equal(cfg(GD32_BRIDGE_LINK_SPI, 2u, 0u, &v), STATUS_INVAL, "unknown op");
	zassert_equal(protocol_dispatch(
	                  GD32_BRIDGE_LINK_SPI, CMD_BOOT_CONFIG, reply, 4u, reply, sizeof reply, &n),
	              STATUS_INVAL,
	              "short request");
	zassert_false(boot_config_pending(NULL), "rejected requests are never queued");
}

ZTEST(boot_config, test_failed_commit_keeps_old_value_and_allows_resend)
{
	uint32_t v;
	fresh_unit();
	g_erase_rv = OTA_FMC_RESULT_ERROR;
	cfg(GD32_BRIDGE_LINK_SPI, 1u, F1, &v);
	boot_config_tick();
	zassert_false(boot_config_pending(NULL), "failed request is dropped");
	zassert_equal(boot_config_flags(), 0u, "host's GET poll never matches");
	g_erase_rv   = OTA_FMC_RESULT_OK;
	g_program_rv = OTA_FMC_RESULT_TIMEOUT;
	set_and_commit(F1, &v);
	zassert_equal(boot_config_flags(), 0u);
	g_program_rv = OTA_FMC_RESULT_OK;
	set_and_commit(F1, &v);
	zassert_equal(boot_config_flags(), F1, "re-sent SET lands");
}

/* Power cut at EVERY flash operation of a SET (erase, payload program, commit
 * program): after the reset the unit boots with the old value, the torn page
 * is read as absent (the ECC model fails any read of a torn doubleword, so a
 * fault would show as a wrong value), and the next SET still works.  The A/B
 * rule keeps the old record intact until the new commit doubleword lands. */
ZTEST(boot_config, test_power_cut_at_every_step_leaves_old_or_new)
{
	for (uint32_t old = 0u; old <= F1; old++) {
		for (uint32_t prior_sets = 0u; prior_sets < 3u; prior_sets++) {
			for (int cut = 0; cut <= 3; cut++) {
				uint32_t       v;
				const uint32_t want = old ^ F1;
				fresh_unit();
				for (uint32_t i = 0; i < prior_sets; i++)
					set_and_commit((i & 1u) ? F1 : 0u, &v);
				set_and_commit(old, &v);
				power_cycle();
				zassert_equal(boot_config_flags(), old);
				g_ops_before_cut =
				    cut; /* 0: erase torn, 1: payload torn, 2: commit torn, 3: done */
				set_and_commit(want, &v);
				power_cycle();
				const uint32_t got = boot_config_flags();
				if (cut < 3) {
					zassert_equal(got, old, "cut %d before the commit lands: old wins", cut);
				} else {
					zassert_equal(got, want, "no cut: new value");
				}
				/* and the next SET still works from this state */
				set_and_commit(want, &v);
				power_cycle();
				zassert_equal(boot_config_flags(), want);
			}
		}
	}
}

ZTEST(boot_config, test_uncorrectable_pages_read_as_absent_not_fault)
{
	uint32_t v;
	fresh_unit();
	set_and_commit(F1, &v);
	/* payload doubleword of the only record turns uncorrectable */
	g_bad[0][0] = true;
	boot_config_load();
	zassert_equal(boot_config_flags(), 0u, "no valid page = default off");
	zassert_true(g_bad_reads > 0u);
	/* both doublewords bad on both pages */
	for (uint32_t p = 0; p < 2u; p++) {
		g_bad[p][0] = g_bad[p][1] = true;
	}
	boot_config_apply();
	zassert_equal(boot_config_flags(), 0u);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_GPIO_WRITE), 0u, "default off: pad untouched");
	/* a SET still recovers the unit: store() erases the absent target page */
	set_and_commit(F1, &v);
	zassert_equal(boot_config_flags(), F1);
}

ZTEST(boot_config, test_payload_is_never_read_before_the_commit_magic)
{
	fresh_unit();
	/* payload doubleword half-written (uncorrectable on silicon), no commit */
	g_page[0][0]    = 0x01u;
	g_bad[0][0]     = true;
	g_payload_reads = g_bad_reads = 0u;
	boot_config_load();
	zassert_equal(boot_config_flags(), 0u);
	zassert_equal(g_payload_reads, 0u);
	zassert_equal(g_bad_reads, 0u, "the torn payload doubleword was never touched");
}

ZTEST(boot_config, test_foreign_or_crc_bad_page_reads_as_absent)
{
	uint32_t v;
	fresh_unit();
	set_and_commit(F1, &v);
	g_page[0][4] ^= 0x01u; /* flags bit flipped: CRC no longer matches */
	boot_config_load();
	zassert_equal(boot_config_flags(), 0u);
	memset(g_page, 0u, sizeof g_page); /* foreign content */
	boot_config_load();
	zassert_equal(boot_config_flags(), 0u);
}

ZTEST(boot_config, test_newest_counter_wins_when_both_pages_are_valid)
{
	uint32_t v;
	fresh_unit();
	set_and_commit(F1, &v); /* REC0 counter 1 */
	set_and_commit(0u, &v); /* REC1 counter 2 */
	boot_config_load();
	zassert_equal(boot_config_flags(), 0u);
	g_bad[1][1] = true; /* newest page's commit goes bad: the older record is the fallback */
	boot_config_load();
	zassert_equal(boot_config_flags(), F1);
}

ZTEST_SUITE(boot_config, NULL, NULL, NULL, NULL, NULL);
