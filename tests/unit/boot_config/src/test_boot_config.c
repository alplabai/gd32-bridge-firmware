/* SPDX-License-Identifier: Apache-2.0
 *
 * Unit tests for src/boot_config.h and CMD_BOOT_CONFIG (alp-sdk #2697): the
 * opt-in "drive PD11 / E1M IO29 SDIO_MUX_EN high at boot" flag.
 *
 * Flash is one host page behind the weak ota_fmc_* seams.  A "power cycle" is
 * bridge_hw_fake_reset() (the HAL/pad state is lost) with the flash buffer
 * kept, then the same boot_config_apply() bridge_hw_init() runs.
 */

#include <zephyr/ztest.h>

#include "bridge_hw_fake.h"
#include "boot_config.h"
#include "protocol.h"

static uint8_t          g_page[OTA_PAGE_SIZE];
static uint32_t         g_erase_calls;
static bool             g_supported;
static ota_fmc_result_t g_erase_rv, g_program_rv;
static bool             g_busy, g_write_safe;
static uint32_t         g_payload_reads;    /* reads of the payload doubleword */
static int              g_program_calls_ok; /* programs that succeed before a tear; <0 = no tear */

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
ota_fmc_result_t ota_fmc_erase_range(uint32_t base, uint32_t len)
{
	zassert_equal(base, OTA_CONFIG_BASE, "only the config page may be erased");
	zassert_equal(len, OTA_PAGE_SIZE);
	g_erase_calls++;
	if (g_erase_rv == OTA_FMC_RESULT_OK) memset(g_page, 0xFF, sizeof g_page);
	return g_erase_rv;
}
ota_fmc_result_t ota_fmc_program(uint32_t addr, const uint8_t *data, size_t len)
{
	zassert_true(addr >= OTA_CONFIG_BASE && addr + len <= OTA_CONFIG_BASE + 16u);
	zassert_equal(addr % 8u, 0u, "doubleword program");
	if (g_program_calls_ok == 0) return OTA_FMC_RESULT_ERROR; /* power cut here */
	if (g_program_calls_ok > 0) g_program_calls_ok--;
	if (g_program_rv == OTA_FMC_RESULT_OK) memcpy(&g_page[addr - OTA_CONFIG_BASE], data, len);
	return g_program_rv;
}
const void *ota_fmc_flash_ptr(uint32_t addr)
{
	zassert_true(addr == OTA_CONFIG_BASE || addr == OTA_CONFIG_BASE + 8u);
	if (addr == OTA_CONFIG_BASE) g_payload_reads++;
	return &g_page[addr - OTA_CONFIG_BASE];
}

static void fresh_unit(void) /* factory-new: erased page, HAL reset */
{
	memset(g_page, 0xFF, sizeof g_page);
	g_erase_calls      = 0u;
	g_supported        = true;
	g_busy             = false;
	g_write_safe       = true;
	g_payload_reads    = 0u;
	g_program_calls_ok = -1;
	g_erase_rv = g_program_rv = OTA_FMC_RESULT_OK;
	bridge_hw_fake_reset();
}

static gd32_bridge_status_t
set_cfg(gd32_bridge_link_t link, uint8_t op, uint32_t flags, uint32_t *out)
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

#define PD11 BOOT_CONFIG_SDMUX_EN_PAD_MASK

ZTEST(boot_config, test_default_off_touches_no_pad)
{
	fresh_unit();
	zassert_equal(boot_config_flags(), 0u);
	boot_config_apply();
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_GPIO_WRITE), 0u, "no flag = pad untouched");
}

ZTEST(boot_config, test_flag_persists_and_is_applied_at_boot)
{
	uint32_t v = 0u;
	fresh_unit();
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, BOOT_CONFIG_FLAG_SDMUX_EN_HIGH, &v), STATUS_OK);
	zassert_equal(v, BOOT_CONFIG_FLAG_SDMUX_EN_HIGH);
	/* SET itself must not move the pad (a running SD root would drop out). */
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_GPIO_WRITE), 0u);

	bridge_hw_fake_reset(); /* power cycle: HAL state gone, flash kept */
	boot_config_apply();
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_GPIO_WRITE), 1u);
	zassert_equal(bridge_hw_fake_gpio_get_pads() & PD11, PD11, "PD11 (IO29) driven high");
	zassert_equal(bridge_hw_fake_gpio_get_pads() & ~PD11, 0u, "no other pad driven");
}

ZTEST(boot_config, test_clear_restores_default)
{
	uint32_t v = 9u;
	fresh_unit();
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 1u, &v), STATUS_OK);
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 0u, &v), STATUS_OK);
	zassert_equal(v, 0u);
	bridge_hw_fake_reset();
	boot_config_apply();
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_GPIO_WRITE), 0u);
}

ZTEST(boot_config, test_get_never_writes_flash_and_allowed_on_i2c)
{
	uint32_t v = 9u;
	fresh_unit();
	zassert_equal(
	    set_cfg(GD32_BRIDGE_LINK_I2C, 0u, 0xFFFFFFFFu, &v), STATUS_OK, "GET ignores flags");
	zassert_equal(v, 0u);
	zassert_equal(g_erase_calls, 0u);
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_I2C, 1u, 1u, &v), STATUS_OK, "SET allowed on I2C");
	zassert_equal(v, 1u);
}

ZTEST(boot_config, test_torn_or_foreign_page_reads_as_off)
{
	uint32_t v;
	fresh_unit();
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 1u, &v), STATUS_OK);
	g_page[0] ^= 0x01u; /* flip a flags bit: CRC no longer matches */
	zassert_equal(boot_config_flags(), 0u);
	memset(g_page, 0u, sizeof g_page); /* foreign content */
	zassert_equal(boot_config_flags(), 0u);
}

ZTEST(boot_config, test_torn_first_doubleword_is_never_read)
{
	uint32_t v;
	fresh_unit();
	g_program_calls_ok = 0; /* cut before the payload doubleword lands */
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 1u, &v), STATUS_IO);
	g_page[0]       = 0x01u; /* payload doubleword half-written: would be ECC-bad on silicon */
	g_page[3]       = 0x5Au;
	g_payload_reads = 0u;
	zassert_equal(boot_config_flags(), 0u);
	zassert_equal(g_payload_reads, 0u, "payload read only after the commit magic");

	fresh_unit();
	g_program_calls_ok = 1; /* payload lands, cut before the commit magic */
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 1u, &v), STATUS_IO);
	g_payload_reads = 0u;
	zassert_equal(boot_config_flags(), 0u, "no commit magic = all off");
	zassert_equal(g_payload_reads, 0u);
}

ZTEST(boot_config, test_unchanged_set_skips_flash)
{
	uint32_t v;
	fresh_unit();
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 0u, &v), STATUS_OK, "default -> default");
	zassert_equal(g_erase_calls, 0u);
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 1u, &v), STATUS_OK);
	zassert_equal(g_erase_calls, 1u);
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 1u, &v), STATUS_OK, "replayed SET");
	zassert_equal(g_erase_calls, 1u, "same value: no second erase");
	g_busy       = true; /* an unchanged SET is still fine while the funnel is busy */
	g_write_safe = false;
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 1u, &v), STATUS_OK);
}

ZTEST(boot_config, test_set_refused_when_funnel_busy_or_bank_unsafe)
{
	uint32_t v;
	fresh_unit();
	g_busy = true;
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 1u, &v), STATUS_BUSY);
	zassert_equal(g_erase_calls, 0u);
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 0u, 0u, &v), STATUS_OK, "GET ignores the funnel");
	g_busy       = false;
	g_write_safe = false; /* DBS = 0 or running from bank 1 */
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 1u, &v), STATUS_NOSUPPORT);
	zassert_equal(g_erase_calls, 0u);
}

ZTEST(boot_config, test_validation_and_fault_mapping)
{
	uint32_t v;
	uint8_t  reply[8];
	size_t   n;
	fresh_unit();
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 0x2u, &v), STATUS_INVAL, "unknown flag bit");
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 2u, 0u, &v), STATUS_INVAL, "unknown op");
	zassert_equal(protocol_dispatch(
	                  GD32_BRIDGE_LINK_SPI, CMD_BOOT_CONFIG, reply, 4u, reply, sizeof reply, &n),
	              STATUS_INVAL,
	              "short request");
	zassert_equal(g_erase_calls, 0u, "rejected requests never reach flash");

	g_supported = false;
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 1u, &v), STATUS_NOSUPPORT);
	g_supported = true;
	g_erase_rv  = OTA_FMC_RESULT_ERROR;
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 1u, &v), STATUS_IO);
	g_erase_rv   = OTA_FMC_RESULT_OK;
	g_program_rv = OTA_FMC_RESULT_TIMEOUT;
	zassert_equal(set_cfg(GD32_BRIDGE_LINK_SPI, 1u, 1u, &v), STATUS_TIMEOUT);
	zassert_equal(boot_config_flags(), 0u, "failed store leaves the safe default");
}

ZTEST_SUITE(boot_config, NULL, NULL, NULL, NULL, NULL);
