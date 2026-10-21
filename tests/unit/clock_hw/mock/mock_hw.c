/* SPDX-License-Identifier: Apache-2.0
 *
 * Register-level simulation behind clock_hw.c's host test.  See mock_hw.h.
 *
 * Every accessor of a register (one per C statement that names it) and every
 * cycle-counter read runs sync(): it diffs the register file against the last
 * snapshot, logs what software changed as ordered tokens, then lets the
 * "hardware" react (HXTALSTB, PLLSTB, SCSS follow with simulated time).
 * Rest values of the write-1-to-clear registers are 0, so any non-zero value
 * seen is a write.
 */

#include "gd32g5x3.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "mock_hw.h"

mock_scn_t     mock_scn;
mock_systick_t mock_systick;
uint32_t       mock_primask;
uint32_t       SystemCoreClock;
uint32_t       bridge_core_clock_hz;
bool           bridge_core_clock_matches;
uint32_t       mock_fwdgt_feeds;
uint32_t       mock_primask_max_ns;

static uint32_t r_ctl, r_cfg0, r_pll, r_int, r_ws, r_stat, r_bkp9;
static uint32_t s_ctl, s_cfg0, s_pll, s_ws, s_bkp9;
static uint32_t stat_flags;
static uint64_t now_ps, hxtal_en_ns, pll_en_ns, timer_en_ns, primask_start_ns;
#define now_ns (now_ps / 1000u) /* read-only view */
static uint32_t cycles;
static bool     in_sync;

static char   log_buf[8192];
static size_t log_len;

static uint32_t syscfg_t14[3];
static uint32_t clk_mask;
static uint32_t trigsel_src;
static bool     t14_trgsel_iti14, t14_slave_ext0, t14_enabled;
static uint32_t t14_car; /* reset value 0: a timer with CAR 0 does not count */

static void logf_(const char *fmt, ...)
{
	char    tok[48];
	va_list ap;
	va_start(ap, fmt);
	(void)vsnprintf(tok, sizeof tok, fmt, ap);
	va_end(ap);
	const size_t n = strlen(tok);
	if (log_len + n + 2u >= sizeof log_buf) return;
	memcpy(log_buf + log_len, tok, n);
	log_len += n;
	log_buf[log_len++] = ' ';
	log_buf[log_len]   = '\0';
}

const char *mock_log_text(void)
{
	return log_buf;
}
void mock_log_clear(void)
{
	log_len    = 0u;
	log_buf[0] = '\0';
}
bool mock_log_has(const char *token)
{
	char pat[64];
	(void)snprintf(pat, sizeof pat, "%s ", token);
	const char *p = strstr(log_buf, pat);
	return p != NULL && (p == log_buf || p[-1] == ' ');
}
bool mock_in_order(const char *first, ...)
{
	va_list     ap;
	const char *from = log_buf;
	const char *tok  = first;
	va_start(ap, first);
	while (tok != NULL) {
		char pat[64];
		(void)snprintf(pat, sizeof pat, "%s ", tok);
		const char *p = strstr(from, pat);
		while (p != NULL && p != log_buf && p[-1] != ' ')
			p = strstr(p + 1, pat);
		if (p == NULL) {
			va_end(ap);
			return false;
		}
		from = p + strlen(pat);
		tok  = va_arg(ap, const char *);
	}
	va_end(ap);
	return true;
}

/* ---- the simulated SYSCLK ---- */

static uint32_t irc8m_hz(void)
{
	return (uint32_t)(8000000ull * (uint64_t)(1000 + mock_scn.irc8m_err_permille) / 1000u);
}

uint32_t mock_sim_sysclk_hz(void)
{
	uint32_t hz = irc8m_hz();
	if (((r_cfg0 >> 2) & 3u) == 3u) {
		const uint32_t in  = (r_pll & RCU_PLL_PLLSEL) ? mock_scn.oscin_hz : irc8m_hz();
		const uint32_t psc = (r_pll & RCU_PLL_PLLPSC) + 1u;
		const uint32_t n   = (r_pll >> 6) & 0xFFu;
		const uint32_t p   = (((r_pll >> 16) & 3u) + 1u) << 1;
		hz                 = in / psc * n / p;
		if (hz == 0u) hz = irc8m_hz(); /* a dead HXTAL: the monitor would have moved SYSCLK */
	}
	static const uint8_t exp[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9 };
	return hz >> exp[(r_cfg0 >> 4) & 0xFu];
}

uint64_t mock_now_ns(void)
{
	return now_ns;
}

/* ---- sync: log software changes, then react as the hardware ---- */

static bool pll_can_lock(void)
{
	const uint32_t psc = (r_pll & RCU_PLL_PLLPSC) + 1u;
	const uint32_t n   = (r_pll >> 6) & 0xFFu;
	if (n < 8u || n > 180u) return false;
	if (r_pll & RCU_PLL_PLLSEL) {
		if (!mock_scn.pll_locks_hxtal || !(r_ctl & RCU_CTL_HXTALSTB)) return false;
		return mock_scn.oscin_hz / psc * n <=
		       440000000u; /* VCO ceiling of the proven tuple + margin */
	}
	return mock_scn.pll_locks_irc8m;
}

static void sync(void)
{
	if (in_sync) return;
	in_sync = true;

	const uint32_t dc = r_ctl ^ s_ctl;
	if (dc & RCU_CTL_IRC8MEN) logf_(r_ctl & RCU_CTL_IRC8MEN ? "IRC+" : "IRC-");
	if (dc & RCU_CTL_HXTALEN) {
		logf_(r_ctl & RCU_CTL_HXTALEN ? "HXEN+" : "HXEN-");
		if (r_ctl & RCU_CTL_HXTALEN) hxtal_en_ns = now_ns;
	}
	if (dc & RCU_CTL_HXTALBPS) logf_(r_ctl & RCU_CTL_HXTALBPS ? "BPS+" : "BPS-");
	if (dc & RCU_CTL_CKMEN) logf_(r_ctl & RCU_CTL_CKMEN ? "CKM+" : "CKM-");
	if (dc & RCU_CTL_PLLEN) {
		logf_(r_ctl & RCU_CTL_PLLEN ? "PLLEN+" : "PLLEN-");
		if (r_ctl & RCU_CTL_PLLEN) pll_en_ns = now_ns;
	}
	if ((r_cfg0 ^ s_cfg0) & RCU_CFG0_SCS) logf_("SCS%u", (unsigned)(r_cfg0 & 3u));
	if ((r_cfg0 ^ s_cfg0) & RCU_CFG0_AHBPSC) logf_("AHB%u", (unsigned)((r_cfg0 >> 4) & 0xFu));
	if (r_pll != s_pll && ((r_pll >> 6) & 0xFFu) != 0u) {
		logf_("PLLCFG:%u/%u/%u/%c",
		      (unsigned)((r_pll & RCU_PLL_PLLPSC) + 1u),
		      (unsigned)((r_pll >> 6) & 0xFFu),
		      (unsigned)(((r_pll >> 16) & 3u) + 1u) << 1,
		      (r_pll & RCU_PLL_PLLSEL) ? 'H' : 'I');
	}
	if ((r_ws ^ s_ws) & FMC_WS_WSCNT) logf_("WS%u", (unsigned)(r_ws & FMC_WS_WSCNT));
	if (r_bkp9 != s_bkp9) logf_("BKP9=%x", (unsigned)r_bkp9);
	if (r_int != 0u) { /* write-only clear bits */
		if (r_int & RCU_INT_CKMIC) logf_("CKMIC");
		r_int = 0u;
	}
	if (r_stat != 0u) { /* write-1-to-clear */
		logf_("STATW:%x", (unsigned)r_stat);
		stat_flags &= ~r_stat;
		r_stat = 0u;
	}

	/* --- the hardware --- */
	if (!(r_ctl & RCU_CTL_HXTALEN) || mock_scn.oscin_hz == 0u ||
	    (mock_scn.require_bypass && !(r_ctl & RCU_CTL_HXTALBPS))) {
		r_ctl &= ~RCU_CTL_HXTALSTB;
	} else if (now_ns - hxtal_en_ns >= (uint64_t)mock_scn.stb_delay_us * 1000u) {
		r_ctl |= RCU_CTL_HXTALSTB;
	}
	if (!(r_ctl & RCU_CTL_PLLEN)) {
		r_ctl &= ~RCU_CTL_PLLSTB;
	} else if (now_ns - pll_en_ns >= (uint64_t)mock_scn.lock_us * 1000u && pll_can_lock()) {
		r_ctl |= RCU_CTL_PLLSTB;
	}
	uint32_t target = r_cfg0 & 3u;
	uint32_t cur    = (r_cfg0 >> 2) & 3u;
	if (target != cur) {
		if (target == 3u) {
			if (r_ctl & RCU_CTL_PLLSTB) cur = 3u;
		} else if (!(cur == 3u && mock_scn.scs_refuses_irc8m)) {
			cur = target;
		}
	}
	r_cfg0 = (r_cfg0 & ~RCU_CFG0_SCSS) | (cur << 2);

	s_ctl   = r_ctl;
	s_cfg0  = r_cfg0;
	s_pll   = r_pll;
	s_ws    = r_ws;
	s_bkp9  = r_bkp9;
	in_sync = false;
}

volatile uint32_t *mock_reg_rcu_ctl(void)
{
	sync();
	return &r_ctl;
}
volatile uint32_t *mock_reg_rcu_cfg0(void)
{
	sync();
	return &r_cfg0;
}
volatile uint32_t *mock_reg_rcu_pll(void)
{
	sync();
	return &r_pll;
}
volatile uint32_t *mock_reg_rcu_int(void)
{
	sync();
	return &r_int;
}
volatile uint32_t *mock_reg_fmc_ws(void)
{
	sync();
	return &r_ws;
}
volatile uint32_t *mock_reg_syscfg_stat(void)
{
	sync();
	return &r_stat;
}
volatile uint32_t *mock_reg_rtc_bkp9(void)
{
	sync();
	return &r_bkp9;
}
volatile uint32_t *mock_reg32(uint32_t addr)
{
	static uint32_t dummy;
	const uint32_t  base = SYSCFG_TIMERCFG(SYSCFG_TIMER14);
	if (addr >= base && addr < base + 12u) return &syscfg_t14[(addr - base) / 4u];
	return &dummy;
}

void mock_set_stat_flags(uint32_t flags)
{
	stat_flags = flags;
}
uint32_t mock_stat_flags(void)
{
	sync();
	return stat_flags;
}

/* ---- time ---- */

static void burn(uint32_t n)
{
	sync();
	now_ps += (uint64_t)n * 1000000000000ull / mock_sim_sysclk_hz();
	cycles += n;
	sync();
}

uint32_t mock_cyccnt(void)
{
	if (mock_scn.cyccnt_stalled) {
		sync();
		return cycles; /* the counter is not running */
	}
	burn(mock_scn.step_cycles);
	return cycles;
}

bool pwm_any_claimed(void)
{
	return mock_scn.pwm_active;
}
bool adc_stream_any_active(void)
{
	return mock_scn.adc_active;
}
bool bridge_link_quiet(void)
{
	return !mock_scn.link_busy;
}

void mock_dwt_ensure(void)
{
}

static void primask_changed(uint32_t from, uint32_t to)
{
	if (from == 0u && to != 0u) primask_start_ns = now_ns;
	if (from != 0u && to == 0u) {
		const uint64_t d = now_ns - primask_start_ns;
		if (d > mock_primask_max_ns) mock_primask_max_ns = (uint32_t)d;
	}
}

void mock_set_primask(uint32_t v)
{
	primask_changed(mock_primask, v);
	mock_primask = v;
}

void fwdgt_counter_reload(void)
{
	mock_fwdgt_feeds++;
}

bool ota_trial_unconfirmed(void)
{
	return mock_scn.trial;
}

/* ---- TIMER14 / TRIGSEL ---- */

void rcu_periph_clock_enable(uint32_t periph)
{
	clk_mask |= periph;
	logf_("CLK%u+", (unsigned)periph);
}
void rcu_periph_clock_disable(uint32_t periph)
{
	clk_mask &= ~periph;
	logf_("CLK%u-", (unsigned)periph);
}
uint32_t mock_timer14_clk_enabled(void)
{
	return clk_mask & RCU_TIMER14;
}
void trigsel_init(uint32_t target, uint32_t source)
{
	logf_("TRIG:%x/%x", (unsigned)target, (unsigned)source);
	if (target == TRIGSEL_OUTPUT_TIMER14_ITI14) trigsel_src = source;
}
uint32_t mock_trigsel_target_source(void)
{
	return trigsel_src;
}
void timer_deinit(uint32_t timer)
{
	(void)timer;
	t14_trgsel_iti14 = t14_slave_ext0 = t14_enabled = false;
	t14_car                                         = 0u;
	logf_("T14DEINIT");
}
void timer_input_trigger_source_select(uint32_t timer, uint32_t intrigger)
{
	(void)timer;
	t14_trgsel_iti14 = (intrigger == TIMER_SMCFG_TRGSEL_ITI14);
	syscfg_t14[2]    = intrigger << 16;
	logf_("ITI14");
}
void timer_slave_mode_select(uint32_t timer, uint32_t slavemode)
{
	(void)timer;
	t14_slave_ext0 = (slavemode == TIMER_SLAVE_MODE_EXTERNAL0);
	logf_("EXT0");
}
void timer_autoreload_value_config(uint32_t timer, uint32_t autoreload)
{
	(void)timer;
	t14_car = autoreload;
	logf_("CAR14");
}
uint32_t mock_timer14_car(void)
{
	return t14_car;
}
void timer_counter_value_config(uint32_t timer, uint32_t counter)
{
	(void)timer;
	(void)counter;
}
void timer_enable(uint32_t timer)
{
	(void)timer;
	t14_enabled = true;
	timer_en_ns = now_ns;
	logf_("T14ON");
}
void timer_disable(uint32_t timer)
{
	(void)timer;
	t14_enabled = false;
	logf_("T14OFF");
}
uint32_t timer_counter_read(uint32_t timer)
{
	(void)timer;
	burn(mock_scn.timer_read_overhead_cycles); /* the call itself costs time before the latch */
	sync();
	const bool routed = (trigsel_src == TRIGSEL_INPUT_HXTAL_DIV32_TRIG) && t14_trgsel_iti14 &&
	                    t14_slave_ext0 && t14_enabled && (clk_mask & RCU_TIMER14) &&
	                    (clk_mask & RCU_TRIGSEL);
	const bool hxtal_running = (r_ctl & RCU_CTL_HXTALEN) && mock_scn.oscin_hz != 0u &&
	                           (!mock_scn.require_bypass || (r_ctl & RCU_CTL_HXTALBPS));
	if (!routed || !hxtal_running || t14_car == 0u) return 0u;
	const uint64_t d_ns = now_ns - timer_en_ns;
	const uint64_t n    = d_ns * (mock_scn.oscin_hz / 32u) / 1000000000ull;
	return (uint32_t)(n > t14_car ? t14_car : n);
}
uint32_t mock_syscfg_timer14_cfg(void)
{
	return syscfg_t14[0] | syscfg_t14[1] | syscfg_t14[2];
}

/* The clock monitor firing: HXTAL died, the hardware moves SYSCLK to IRC8M and
 * drops the PLL, and the NMI flag is raised (the NMI itself is the test's call). */
void mock_hw_ckm_fire(void)
{
	sync();
	mock_scn.oscin_hz = 0u;
	r_ctl &= ~(RCU_CTL_PLLEN | RCU_CTL_PLLSTB | RCU_CTL_HXTALSTB);
	r_cfg0 = (r_cfg0 & ~(RCU_CFG0_SCS | RCU_CFG0_SCSS)) | RCU_CKSYSSRC_IRC8M | RCU_SCSS_IRC8M;
	stat_flags |= SYSCFG_STAT_CKMNMIIF;
	s_ctl  = r_ctl;
	s_cfg0 = r_cfg0;
	mock_log_clear();
}

/* Deep-sleep: HXTAL and the PLL stop and SYSCLK reverts to IRC8M; CKMEN, HXTALBPS
 * and the PLL configuration registers are retained (the very point of finding 5). */
void mock_hw_deepsleep(void)
{
	sync();
	r_ctl &= ~(RCU_CTL_HXTALEN | RCU_CTL_HXTALSTB | RCU_CTL_PLLEN | RCU_CTL_PLLSTB);
	r_cfg0 = (r_cfg0 & ~(RCU_CFG0_SCS | RCU_CFG0_SCSS)) | RCU_CKSYSSRC_IRC8M | RCU_SCSS_IRC8M;
	s_ctl  = r_ctl;
	s_cfg0 = r_cfg0;
}

/* ---- reset ---- */

void mock_hw_reset(void)
{
	memset(&mock_scn, 0, sizeof mock_scn);
	mock_scn.oscin_hz        = 8000000u;
	mock_scn.require_bypass  = true;
	mock_scn.stb_delay_us    = 300u;
	mock_scn.lock_us         = 100u;
	mock_scn.pll_locks_irc8m = true;
	mock_scn.pll_locks_hxtal = true;
	mock_scn.step_cycles     = 50u;

	/* The vendor SystemInit() result: IRC8M PLL at 216 MHz, AHB /1. */
	r_ctl  = RCU_CTL_IRC8MEN | RCU_CTL_PLLEN | RCU_CTL_PLLSTB;
	r_cfg0 = RCU_CKSYSSRC_PLLP | RCU_SCSS_PLLP;
	r_pll  = 1u | (108u << 6) | RCU_PLL_PLLPEN | RCU_PLL_PLLQEN | RCU_PLL_PLLREN | (2u << 23) |
	         (2u << 27);
	r_int = r_stat = r_bkp9 = 0u;
	r_ws                    = 7u;
	s_ctl                   = r_ctl;
	s_cfg0                  = r_cfg0;
	s_pll                   = r_pll;
	s_ws                    = r_ws;
	s_bkp9                  = r_bkp9;
	stat_flags              = 0u;
	now_ps = hxtal_en_ns = pll_en_ns = timer_en_ns = 0u;
	cycles                                         = 0u;
	in_sync                                        = false;
	memset(syscfg_t14, 0, sizeof syscfg_t14);
	clk_mask         = 0u;
	trigsel_src      = 0u;
	t14_trgsel_iti14 = t14_slave_ext0 = t14_enabled = false;
	t14_car                                         = 0u;
	mock_primask                                    = 0u;
	mock_primask_max_ns                             = 0u;
	mock_fwdgt_feeds                                = 0u;
	memset(&mock_systick, 0, sizeof mock_systick);
	SystemCoreClock           = 216000000u;
	bridge_core_clock_hz      = 216000000u;
	bridge_core_clock_matches = true;
	mock_log_clear();
}
