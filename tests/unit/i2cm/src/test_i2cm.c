/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * I2C3 master proxy (protocol v0.17: CMD_I2CM_CONFIG / _XFER / _RESULT).
 *
 * Two layers, one suite:
 *   - the job state machine (hal/gd32/i2cm_core.c) driven against fake bus
 *     ops, which is where the CONFIG / XFER / RESULT contract lives;
 *   - the wire handlers in src/protocol.c over the injectable fake HAL
 *     (link policy, payload validation, status mapping, POWER_MODE_SET
 *     refusal), which is where the opcode-level contract lives.
 * The polled register driver (hal/gd32/i2cm.c) needs the vendor headers
 * and a bus, so it is covered by the gd32 backend build and the bench,
 * not here.
 */

#include <string.h>
#include <zephyr/ztest.h>

#include "bridge_hw.h"
#include "bridge_hw_fake.h"
#include "i2cm_core.h"
#include "protocol.h"

/* ------------------------------------------------------------------ */
/* Fake bus ops for the state machine                                  */
/* ------------------------------------------------------------------ */

static struct {
	int      enable_rv;
	uint16_t enable_khz;
	unsigned enable_calls, release_calls, run_calls;
	uint8_t  run_result;
	uint8_t  run_nread;
	uint8_t  last_addr, last_wlen, last_rlen;
	uint8_t  last_w[I2CM_MAX_WRITE];
} bus;

static int fake_enable(uint16_t khz)
{
	bus.enable_calls++;
	bus.enable_khz = khz;
	return bus.enable_rv;
}

static void fake_release(void)
{
	bus.release_calls++;
}

static uint8_t
fake_run(uint8_t addr7, const uint8_t *w, uint8_t wlen, uint8_t *r, uint8_t rlen, uint8_t *nread)
{
	bus.run_calls++;
	bus.last_addr = addr7;
	bus.last_wlen = wlen;
	bus.last_rlen = rlen;
	if (wlen != 0u) memcpy(bus.last_w, w, wlen);
	for (uint8_t i = 0u; i < bus.run_nread && i < rlen; i++)
		r[i] = (uint8_t)(0xA0u + i);
	*nread = bus.run_nread;
	return bus.run_result;
}

static const i2cm_ops_t ops = { fake_enable, fake_release, fake_run };

static void core_reset(void)
{
	memset(&bus, 0, sizeof bus);
	i2cm_core_init(&ops);
}

static void core_configure(void)
{
	zassert_equal(i2cm_core_config(100u), BRIDGE_HW_OK);
}

/* ------------------------------------------------------------------ */
/* State machine                                                       */
/* ------------------------------------------------------------------ */

ZTEST(i2cm, test_pads_untouched_and_not_ready_until_config)
{
	uint8_t tag, res, n, buf[I2CM_MAX_READ];
	core_reset();
	zassert_equal(bus.enable_calls + bus.release_calls, 0u, "init never touches the pads");
	zassert_equal(i2cm_core_xfer(1u, 0x50u, NULL, 0u, 0u), BRIDGE_HW_ERR_NOT_READY);
	zassert_equal(i2cm_core_result(&tag, &res, &n, buf), BRIDGE_HW_ERR_NOT_READY);
	zassert_false(i2cm_core_busy());
}

ZTEST(i2cm, test_config_speeds)
{
	core_reset();
	zassert_equal(i2cm_core_config(100u), BRIDGE_HW_OK);
	zassert_equal(bus.enable_khz, 100u);
	zassert_equal(i2cm_core_config(400u), BRIDGE_HW_OK);
	zassert_equal(bus.enable_khz, 400u);
	const unsigned before = bus.enable_calls;
	zassert_equal(i2cm_core_config(1000u), BRIDGE_HW_ERR_INVAL);
	zassert_equal(i2cm_core_config(50u), BRIDGE_HW_ERR_INVAL);
	zassert_equal(bus.enable_calls, before, "an invalid speed never reaches the bus");
}

ZTEST(i2cm, test_config_zero_releases_pads_and_unconfigures)
{
	core_reset();
	core_configure();
	zassert_equal(i2cm_core_config(0u), BRIDGE_HW_OK);
	zassert_equal(bus.release_calls, 1u);
	zassert_equal(i2cm_core_xfer(1u, 0x50u, NULL, 0u, 0u), BRIDGE_HW_ERR_NOT_READY);
}

ZTEST(i2cm, test_config_failure_leaves_unconfigured)
{
	core_reset();
	bus.enable_rv = BRIDGE_HW_ERR_NOTIMPL;
	zassert_equal(i2cm_core_config(100u), BRIDGE_HW_ERR_NOTIMPL);
	zassert_equal(bus.release_calls, 1u, "pads released again");
	zassert_equal(i2cm_core_xfer(1u, 0x50u, NULL, 0u, 0u), BRIDGE_HW_ERR_NOT_READY);
}

ZTEST(i2cm, test_result_not_ready_until_a_job_ran)
{
	uint8_t tag, res, n, buf[I2CM_MAX_READ];
	core_reset();
	core_configure();
	zassert_equal(i2cm_core_result(&tag, &res, &n, buf), BRIDGE_HW_ERR_NOT_READY);
}

ZTEST(i2cm, test_xfer_queues_busy_until_tick_then_result)
{
	uint8_t       tag, res, n, buf[I2CM_MAX_READ];
	const uint8_t w[2] = { 0x12u, 0x34u };

	core_reset();
	core_configure();
	bus.run_nread = 3u;
	zassert_equal(i2cm_core_xfer(7u, 0x50u, w, 2u, 3u), BRIDGE_HW_OK);
	zassert_equal(bus.run_calls, 0u, "XFER only queues");
	zassert_true(i2cm_core_busy());
	zassert_equal(i2cm_core_result(&tag, &res, &n, buf), BRIDGE_HW_ERR_BUSY);
	zassert_equal(i2cm_core_xfer(8u, 0x51u, NULL, 0u, 0u), BRIDGE_HW_ERR_BUSY, "no second job");
	zassert_equal(i2cm_core_config(400u), BRIDGE_HW_ERR_BUSY, "no CONFIG under a job");

	i2cm_core_tick();
	zassert_equal(bus.run_calls, 1u);
	zassert_equal(bus.last_addr, 0x50u);
	zassert_equal(bus.last_wlen, 2u);
	zassert_equal(bus.last_rlen, 3u);
	zassert_mem_equal(bus.last_w, w, 2u);
	zassert_false(i2cm_core_busy());

	zassert_equal(i2cm_core_result(&tag, &res, &n, buf), BRIDGE_HW_OK);
	zassert_equal(tag, 7u);
	zassert_equal(res, I2CM_RES_OK);
	zassert_equal(n, 3u);
	zassert_equal(buf[0], 0xA0u);
	zassert_equal(buf[2], 0xA2u);

	/* The result stays readable (a lost I2C read can be repeated). */
	n = 0u;
	zassert_equal(i2cm_core_result(&tag, &res, &n, buf), BRIDGE_HW_OK);
	zassert_equal(n, 3u);

	/* A tick with nothing queued does not rerun the job. */
	i2cm_core_tick();
	zassert_equal(bus.run_calls, 1u);
}

ZTEST(i2cm, test_new_xfer_discards_uncollected_result)
{
	uint8_t tag, res, n, buf[I2CM_MAX_READ];
	core_reset();
	core_configure();
	zassert_equal(i2cm_core_xfer(1u, 0x50u, NULL, 0u, 0u), BRIDGE_HW_OK);
	i2cm_core_tick();
	/* never collected */
	zassert_equal(i2cm_core_xfer(2u, 0x51u, NULL, 0u, 0u), BRIDGE_HW_OK);
	zassert_equal(i2cm_core_result(&tag, &res, &n, buf), BRIDGE_HW_ERR_BUSY);
	i2cm_core_tick();
	zassert_equal(i2cm_core_result(&tag, &res, &n, buf), BRIDGE_HW_OK);
	zassert_equal(tag, 2u, "only the newest job's result survives");
}

ZTEST(i2cm, test_config_after_a_job_clears_the_result)
{
	uint8_t tag, res, n, buf[I2CM_MAX_READ];
	core_reset();
	core_configure();
	zassert_equal(i2cm_core_xfer(1u, 0x50u, NULL, 0u, 0u), BRIDGE_HW_OK);
	i2cm_core_tick();
	core_configure();
	zassert_equal(i2cm_core_result(&tag, &res, &n, buf), BRIDGE_HW_ERR_NOT_READY);
}

ZTEST(i2cm, test_failed_job_reports_code_and_no_data)
{
	uint8_t tag, res, n = 9u, buf[I2CM_MAX_READ];
	core_reset();
	core_configure();
	bus.run_result = I2CM_RES_NACK_ADDR;
	bus.run_nread  = 4u; /* a failing run must not leak data */
	zassert_equal(i2cm_core_xfer(5u, 0x10u, NULL, 0u, 4u), BRIDGE_HW_OK);
	i2cm_core_tick();
	zassert_equal(i2cm_core_result(&tag, &res, &n, buf), BRIDGE_HW_OK);
	zassert_equal(res, I2CM_RES_NACK_ADDR);
	zassert_equal(n, 0u);
	zassert_equal(tag, 5u);
}

ZTEST(i2cm, test_xfer_argument_limits)
{
	uint8_t w[I2CM_MAX_WRITE + 1u] = { 0 };
	core_reset();
	core_configure();
	zassert_equal(i2cm_core_xfer(0u, 0x80u, NULL, 0u, 0u), BRIDGE_HW_ERR_INVAL, "addr7 > 0x7F");
	zassert_equal(i2cm_core_xfer(0u, 0x50u, w, I2CM_MAX_WRITE + 1u, 0u), BRIDGE_HW_ERR_INVAL);
	zassert_equal(i2cm_core_xfer(0u, 0x50u, NULL, 0u, I2CM_MAX_READ + 1u), BRIDGE_HW_ERR_INVAL);
	zassert_equal(i2cm_core_xfer(0u, 0x50u, w, I2CM_MAX_WRITE, I2CM_MAX_READ), BRIDGE_HW_OK);
}

ZTEST(i2cm, test_wake_marks_unconfigured_but_never_under_a_job)
{
	core_reset();
	core_configure();
	zassert_equal(i2cm_core_xfer(1u, 0x50u, NULL, 0u, 0u), BRIDGE_HW_OK);
	i2cm_core_mark_unconfigured(); /* job queued: left alone */
	zassert_true(i2cm_core_busy());
	i2cm_core_tick();
	i2cm_core_mark_unconfigured();
	zassert_equal(bus.release_calls, 1u);
	zassert_equal(i2cm_core_xfer(1u, 0x50u, NULL, 0u, 0u), BRIDGE_HW_ERR_NOT_READY);
}

ZTEST(i2cm, test_scl_counts)
{
	uint32_t h, l;
	/* 216 MHz kernel clock, PSC = 15: tPSC = 74.07 ns (the live boot clock). */
	zassert_true(i2cm_scl_counts(216000000u, 15u, 100u, &h, &l));
	zassert_equal(l, 63u, "4700 ns / 74 ns, minus 1");
	zassert_equal(h, 54u, "4000 ns / 74 ns, minus 1");
	zassert_true(i2cm_scl_counts(216000000u, 15u, 400u, &h, &l));
	zassert_equal(l, 18u);
	zassert_equal(h, 10u);
	/* 8 MHz (IRC8M after a failed relock), PSC = 0: tPSC = 125 ns. */
	zassert_true(i2cm_scl_counts(8000000u, 0u, 100u, &h, &l));
	zassert_equal(l, 37u);
	zassert_equal(h, 31u);
	/* unsupported speed, bad prescaler, field overflow */
	zassert_false(i2cm_scl_counts(216000000u, 15u, 1000u, &h, &l));
	zassert_false(i2cm_scl_counts(216000000u, 16u, 100u, &h, &l));
	zassert_false(i2cm_scl_counts(216000000u, 0u, 100u, &h, &l), "tPSC 4.6 ns: >255 counts");
}

/* ------------------------------------------------------------------ */
/* Wire handlers over the fake HAL                                     */
/* ------------------------------------------------------------------ */

#define CAP 272u

static gd32_bridge_status_t disp(gd32_bridge_link_t link,
                                 uint8_t            cmd,
                                 const uint8_t     *req,
                                 size_t             len,
                                 uint8_t           *reply,
                                 size_t            *rlen)
{
	*rlen = 0xDEADu;
	return protocol_dispatch(link, cmd, req, len, reply, CAP, rlen);
}

#define I2C GD32_BRIDGE_LINK_I2C
#define SPI GD32_BRIDGE_LINK_SPI

ZTEST(i2cm, test_wire_opcodes_and_version)
{
	uint8_t reply[CAP];
	size_t  n;
	zassert_equal(CMD_I2CM_CONFIG, 0xA0);
	zassert_equal(CMD_I2CM_XFER, 0xA1);
	zassert_equal(CMD_I2CM_RESULT, 0xA2);
	zassert_equal(PROTOCOL_VERSION_MINOR, 17u);
	bridge_hw_fake_reset();
	zassert_equal(disp(I2C, CMD_GET_VERSION, NULL, 0u, reply, &n), STATUS_OK);
	zassert_equal(reply[1], 17u);
}

ZTEST(i2cm, test_i2c_link_only)
{
	uint8_t       reply[CAP];
	size_t        n;
	const uint8_t cfg[2]  = { 100u, 0u };
	const uint8_t xfer[5] = { 1u, 0x50u, 0u, 0u, 0u };

	bridge_hw_fake_reset();
	zassert_equal(disp(SPI, CMD_I2CM_CONFIG, cfg, 2u, reply, &n), STATUS_NOSUPPORT);
	zassert_equal(n, 0u);
	zassert_equal(disp(SPI, CMD_I2CM_XFER, xfer, 5u, reply, &n), STATUS_NOSUPPORT);
	zassert_equal(disp(SPI, CMD_I2CM_RESULT, NULL, 0u, reply, &n), STATUS_NOSUPPORT);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_I2CM_CONFIG), 0u);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_I2CM_XFER), 0u);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_I2CM_RESULT), 0u);

	zassert_equal(disp(I2C, CMD_I2CM_CONFIG, cfg, 2u, reply, &n), STATUS_OK);
	zassert_equal(n, 0u);
	zassert_equal(bridge_hw_fake_i2cm_last_config_khz(), 100u);
}

ZTEST(i2cm, test_not_in_batch)
{
	uint8_t reply[CAP];
	size_t  n;
	/* BATCH needs its link feature; with it off the whole opcode is
	 * NOSUPPORT, and an I2CM sub-op is not on the batch allow-list either
	 * (covered by the dispatcher's batch_ops[]).  Here: the SPI link refuses
	 * the opcode outright, so a batch can never reach the handler. */
	const uint8_t sub[] = { 1u, CMD_I2CM_RESULT, 0u };
	bridge_hw_fake_reset();
	zassert_equal(disp(SPI, CMD_BATCH, sub, sizeof sub, reply, &n), STATUS_NOSUPPORT);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_I2CM_RESULT), 0u);
}

ZTEST(i2cm, test_config_wire)
{
	uint8_t       reply[CAP];
	size_t        n;
	const uint8_t k400[2] = { 0x90u, 0x01u }; /* 400 LE */
	const uint8_t k0[2]   = { 0u, 0u };

	bridge_hw_fake_reset();
	zassert_equal(disp(I2C, CMD_I2CM_CONFIG, k400, 2u, reply, &n), STATUS_OK);
	zassert_equal(bridge_hw_fake_i2cm_last_config_khz(), 400u);
	zassert_equal(disp(I2C, CMD_I2CM_CONFIG, k0, 2u, reply, &n), STATUS_OK);
	zassert_equal(bridge_hw_fake_i2cm_last_config_khz(), 0u);
	/* length */
	zassert_equal(disp(I2C, CMD_I2CM_CONFIG, k400, 1u, reply, &n), STATUS_INVAL);
	zassert_equal(disp(I2C, CMD_I2CM_CONFIG, k400, 3u, reply, &n), STATUS_INVAL);
	zassert_equal(disp(I2C, CMD_I2CM_CONFIG, NULL, 0u, reply, &n), STATUS_INVAL);
	/* HAL status mapping: bad speed INVAL, stub-style NOSUPPORT, running job BUSY */
	bridge_hw_fake_force(FAKE_FN_I2CM_CONFIG, BRIDGE_HW_ERR_INVAL);
	zassert_equal(disp(I2C, CMD_I2CM_CONFIG, k400, 2u, reply, &n), STATUS_INVAL);
	bridge_hw_fake_force(FAKE_FN_I2CM_CONFIG, BRIDGE_HW_ERR_NOTIMPL);
	zassert_equal(disp(I2C, CMD_I2CM_CONFIG, k400, 2u, reply, &n), STATUS_NOSUPPORT);
	bridge_hw_fake_force(FAKE_FN_I2CM_CONFIG, BRIDGE_HW_ERR_BUSY);
	zassert_equal(disp(I2C, CMD_I2CM_CONFIG, k400, 2u, reply, &n), STATUS_BUSY);
	bridge_hw_fake_reset();
}

ZTEST(i2cm, test_xfer_wire)
{
	uint8_t reply[CAP];
	size_t  n;
	uint8_t req[5 + I2CM_MAX_WRITE];
	uint8_t tag, addr, w[60], wlen, rlen;

	bridge_hw_fake_reset();
	memset(req, 0, sizeof req);

	/* write-then-read */
	const uint8_t wr[] = { 7u, 0x50u, 0u, 2u, 4u, 0x12u, 0x34u };
	zassert_equal(disp(I2C, CMD_I2CM_XFER, wr, sizeof wr, reply, &n), STATUS_OK);
	zassert_equal(n, 0u);
	bridge_hw_fake_i2cm_last_xfer(&tag, &addr, w, &wlen, &rlen);
	zassert_equal(tag, 7u);
	zassert_equal(addr, 0x50u);
	zassert_equal(wlen, 2u);
	zassert_equal(rlen, 4u);
	zassert_equal(w[0], 0x12u);
	zassert_equal(w[1], 0x34u);

	/* quick write probe: wlen = rlen = 0 */
	const uint8_t quick[] = { 1u, 0x20u, 0u, 0u, 0u };
	zassert_equal(disp(I2C, CMD_I2CM_XFER, quick, sizeof quick, reply, &n), STATUS_OK);

	/* limits: wlen 60 / rlen 62 accepted, one more is INVAL */
	req[0] = 1u;
	req[1] = 0x50u;
	req[3] = I2CM_MAX_WRITE;
	req[4] = I2CM_MAX_READ;
	zassert_equal(disp(I2C, CMD_I2CM_XFER, req, 5u + I2CM_MAX_WRITE, reply, &n), STATUS_OK);
	req[3] = I2CM_MAX_WRITE + 1u;
	zassert_equal(disp(I2C, CMD_I2CM_XFER, req, 5u + I2CM_MAX_WRITE + 1u, reply, &n), STATUS_INVAL);
	req[3] = 0u;
	req[4] = I2CM_MAX_READ + 1u;
	zassert_equal(disp(I2C, CMD_I2CM_XFER, req, 5u, reply, &n), STATUS_INVAL);

	/* flags must be 0, addr7 <= 0x7F, wlen must match the payload exactly */
	const uint8_t bad_flags[] = { 1u, 0x50u, 1u, 0u, 0u };
	zassert_equal(disp(I2C, CMD_I2CM_XFER, bad_flags, sizeof bad_flags, reply, &n), STATUS_INVAL);
	const uint8_t bad_addr[] = { 1u, 0x80u, 0u, 0u, 0u };
	zassert_equal(disp(I2C, CMD_I2CM_XFER, bad_addr, sizeof bad_addr, reply, &n), STATUS_INVAL);
	const uint8_t short_w[] = { 1u, 0x50u, 0u, 3u, 0u, 0xAAu };
	zassert_equal(disp(I2C, CMD_I2CM_XFER, short_w, sizeof short_w, reply, &n), STATUS_INVAL);
	const uint8_t long_w[] = { 1u, 0x50u, 0u, 1u, 0u, 0xAAu, 0xBBu };
	zassert_equal(disp(I2C, CMD_I2CM_XFER, long_w, sizeof long_w, reply, &n), STATUS_INVAL);
	zassert_equal(disp(I2C, CMD_I2CM_XFER, quick, 4u, reply, &n), STATUS_INVAL);

	/* HAL status mapping: unconfigured NOT_READY, running job BUSY */
	bridge_hw_fake_force(FAKE_FN_I2CM_XFER, BRIDGE_HW_ERR_NOT_READY);
	zassert_equal(disp(I2C, CMD_I2CM_XFER, quick, sizeof quick, reply, &n), STATUS_NOT_READY);
	bridge_hw_fake_force(FAKE_FN_I2CM_XFER, BRIDGE_HW_ERR_BUSY);
	zassert_equal(disp(I2C, CMD_I2CM_XFER, quick, sizeof quick, reply, &n), STATUS_BUSY);
	zassert_equal(n, 0u);
	bridge_hw_fake_force(FAKE_FN_I2CM_XFER, BRIDGE_HW_ERR_NOTIMPL);
	zassert_equal(disp(I2C, CMD_I2CM_XFER, quick, sizeof quick, reply, &n), STATUS_NOSUPPORT);
	bridge_hw_fake_reset();
}

ZTEST(i2cm, test_result_wire)
{
	uint8_t       reply[CAP];
	size_t        n;
	const uint8_t data[4] = { 0xDEu, 0xADu, 0xBEu, 0xEFu };

	bridge_hw_fake_reset();
	bridge_hw_fake_i2cm_set_result(7u, I2CM_RES_OK, data, 4u);
	zassert_equal(disp(I2C, CMD_I2CM_RESULT, NULL, 0u, reply, &n), STATUS_OK);
	zassert_equal(n, 7u);
	zassert_equal(reply[0], 7u, "tag");
	zassert_equal(reply[1], 0u, "result");
	zassert_equal(reply[2], 4u, "nread");
	zassert_mem_equal(&reply[3], data, 4u);

	/* a failed job: outer STATUS stays OK, result carries the bus outcome */
	bridge_hw_fake_i2cm_set_result(9u, I2CM_RES_NACK_ADDR, NULL, 0u);
	zassert_equal(disp(I2C, CMD_I2CM_RESULT, NULL, 0u, reply, &n), STATUS_OK);
	zassert_equal(n, 3u);
	zassert_equal(reply[1], I2CM_RES_NACK_ADDR);

	/* a maximum read fits the 65-byte I2C payload */
	uint8_t big[I2CM_MAX_READ];
	memset(big, 0x5Au, sizeof big);
	bridge_hw_fake_i2cm_set_result(1u, I2CM_RES_OK, big, I2CM_MAX_READ);
	zassert_equal(disp(I2C, CMD_I2CM_RESULT, NULL, 0u, reply, &n), STATUS_OK);
	zassert_equal(n, 65u);

	/* BUSY / NOT_READY: empty reply */
	bridge_hw_fake_force(FAKE_FN_I2CM_RESULT, BRIDGE_HW_ERR_BUSY);
	zassert_equal(disp(I2C, CMD_I2CM_RESULT, NULL, 0u, reply, &n), STATUS_BUSY);
	zassert_equal(n, 0u);
	bridge_hw_fake_force(FAKE_FN_I2CM_RESULT, BRIDGE_HW_ERR_NOT_READY);
	zassert_equal(disp(I2C, CMD_I2CM_RESULT, NULL, 0u, reply, &n), STATUS_NOT_READY);
	zassert_equal(n, 0u);

	/* payload must be empty; a short reply buffer is NOMEM, not an overrun */
	bridge_hw_fake_force(FAKE_FN_I2CM_RESULT, BRIDGE_HW_OK);
	zassert_equal(disp(I2C, CMD_I2CM_RESULT, data, 1u, reply, &n), STATUS_INVAL);
	n = 0xDEADu;
	zassert_equal(protocol_dispatch(I2C, CMD_I2CM_RESULT, NULL, 0u, reply, 10u, &n), STATUS_NOMEM);
	bridge_hw_fake_reset();
}

ZTEST(i2cm, test_power_mode_refused_busy_while_a_job_runs)
{
	uint8_t       reply[CAP];
	size_t        n;
	const uint8_t req[10] = { 2u, 0u, 8u, 0u, 0u, 0u, 0x10u, 0x27u, 0u, 0u }; /* deep-sleep, 10 s */

	bridge_hw_fake_reset();
	bridge_hw_fake_i2cm_set_busy(true);
	zassert_equal(disp(SPI, CMD_POWER_MODE_SET, req, sizeof req, reply, &n), STATUS_BUSY);
	zassert_equal(n, 0u);
	zassert_equal(bridge_hw_fake_call_count(FAKE_FN_POWER_MODE_SET), 0u, "HAL never asked");
	zassert_equal(bridge_hw_fake_attn_quiesce_calls(), 0u, "ATTN left alone");
	bridge_hw_fake_i2cm_set_busy(false);
	zassert_equal(disp(SPI, CMD_POWER_MODE_SET, req, sizeof req, reply, &n), STATUS_OK);
	bridge_hw_fake_reset();
}

ZTEST_SUITE(i2cm, NULL, NULL, NULL, NULL, NULL);
