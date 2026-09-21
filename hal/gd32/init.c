/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * GD32G5x3 backend for the bridge HAL.  Selected by setting
 * BRIDGE_HAL_BACKEND=gd32 in CMakeLists.txt.  Links
 * against the GigaDevice firmware-library wrapper.  This repo does NOT
 * vendor that tree: pass -DGD32_VENDOR_DIR=<path> pointing at a checkout
 * of alp-sdk's vendors/gd32_firmware_library/ (a verbatim mirror of GD's
 * v1.5.0 release).  Left unset, the build falls back to
 * ../../vendors/gd32_firmware_library resolved against this source tree
 * -- the pre-split layout, when this tree was nested at
 * <alp-sdk>/firmware/gd32-bridge/.  See README.md "Build".
 *
 * Status:
 *   The hooks below have real bodies -- selecting this backend drives
 *   silicon.  The per-hook record that follows is the state AS LANDED,
 *   not a plan: DONE, PARTIAL (defaults accepted, the rest refused) or
 *   an unconditional sentinel where the SoM revision has no HW path.
 *   Whatever a hook refuses returns BRIDGE_HW_ERR_NOTIMPL, which
 *   reaches the wire as STATUS_NOSUPPORT (status_from_hw() in
 *   src/protocol.c).
 *
 * Per-hook state, in the order the bodies landed (increasing risk):
 *
 *   1. RESET_REASON          -- DONE: RCU_RSTSCK decode + RSTFC clear.
 *   2. GPIO_READ / WRITE     -- DONE: 18-pad map (E1M IO8..IO35),
 *                               boot configures all as INPUT + PULL_UP,
 *                               write auto-promotes to OUTPUT push-pull.
 *   3. TRNG_READ             -- DONE: NIST SP800-90B mode init in
 *                               bridge_hw_init, DRDY-polled byte read
 *                               with bounded timeout.
 *   4. TMU_COMPUTE           -- DONE: 9 of 12 host functions via TMU
 *                               (tan / exp / tanh NOSUPPORT -- host
 *                               wrapper can libm-fallback in a future
 *                               commit).  F32 + Q31 paths.
 *   5. DAC_SET / GET         -- DONE: channel 0 -> DAC0_OUT0 / PA4,
 *                               channel 1 -> DAC1_OUT0 / PA6, mV<->12-bit
 *                               code at VREF=1800mV (V2N analog supply).
 *   6. PWM_SET / GET         -- DONE: 8 channels across TIMER0 + TIMER7
 *                               at 1 us LSB; pin AFs per datasheet Rev2.0.
 *   7. PWM_CONFIGURE         -- PARTIAL: defaults (edge-aligned, no
 *                               dead-time, no break input) accepted;
 *                               non-defaults NOSUPPORT pending follow-up.
 *   8. ADC_READ              -- DONE: 8-pad map across ADC0..3, single-
 *                               shot polling, mV<->code at VREF=1800mV.
 *   9. ADC_CONFIGURE         -- DONE: per-channel sample-cycle,
 *                               oversample-ratio and 6/8/10/12-bit
 *                               resolution settings cached + applied;
 *                               14/16-bit effective modes are NOSUPPORT.
 *   10. ADC_STREAM_*         -- DONE (§C.23): DMA0/1-backed
 *                               continuous acquisition.  Two parallel
 *                               streams; each owns a 1024-sample
 *                               circular ring buffer + a DMA channel
 *                               driving it peripheral-to-memory.
 *                               Host drains via polled stream_read;
 *                               write cursor recovered from the DMA
 *                               counter.
 *   11. QENC_READ / RESET    -- DONE: 4 encoders across TIMER1/2/3/4
 *                               in quadrature decoder mode 2 (X4).
 *   12. COUNTER_READ         -- DONE: Cortex-M33 DWT cycle counter
 *                               (32-bit free-running at core clock).
 *   13. PWM_CAPTURE_*        -- DONE (§C.15a): TIMERx input-capture
 *                               with polled drain.  V2N pad map binds
 *                               the COMPLEMENTARY (CHxN) outputs --
 *                               BEGIN reconfigures the channel as
 *                               input-capture but the captured edges
 *                               only land once a future hardware-
 *                               bring-up commit reworks the pad
 *                               routing onto the timer's main CHx
 *                               input mux.  Structural code path
 *                               (config + drain + ns conversion) is
 *                               complete and exercised end-to-end.
 *   14. PWM_SINGLE_PULSE     -- DONE: TIMERx OPM (one-pulse mode).
 *                               Switches the timer's whole SP-bit so
 *                               other channels on the same timer also
 *                               run as single-pulse until a PWM_SET
 *                               flips back to repetitive.
 *   15. TIMER_SYNC           -- DONE (§C.15b): master-slave SMC
 *                               config via timer_slave_mode_select
 *                               + timer_master_output0_trigger_source_select
 *                               + timer_master_slave_mode_config.
 *                               Wire `mode` byte translated to the
 *                               vendor's TIMER_SLAVE_MODE_* /
 *                               TIMER_QUAD_DECODER_MODE* encoding.
 *                               Slave ITIx looked up per (master, slave)
 *                               pair (timer_sync_iti_lookup, hal/gd32/
 *                               timer_sync_iti.c, UM p.570); an
 *                               unconnected pair returns INVAL.  Only
 *                               initialised TIMER0/TIMER7 are exposed;
 *                               TIMER19 id 2 returns RANGE (#142).
 *   16. POWER_MODE_SET       -- DONE (§C.15c + §C.25): mode 0/1
 *                               (run/sleep) accepted no-ops, mode 2
 *                               (deep-sleep) calls
 *                               pmu_to_deepsleepmode(LDO_LOWPOWER, WFI),
 *                               mode 3 (standby) calls
 *                               pmu_to_standbymode().  Wake-source
 *                               bitmap: GPIO enables PMU_WAKEUP_PIN0..4,
 *                               RTC + TIMER arm the RTC wakeup timer
 *                               at 0.5 ms LSB (IRC32K / DIV16); same
 *                               timer also honours wake_after_ms.
 *                               UART_RX / USB / ETH_LINK reject as
 *                               NOSUPPORT (no HW path on GD32G5).
 *   17. DA9292 fault forward -- 0xFF sentinel UNCONDITIONALLY: this
 *                               SoM rev wires the DA9292 fault nets
 *                               only to the Renesas (P37/P36); the
 *                               GD32 has no connection to them and no
 *                               I2C path to the PMIC.  Pin sampling
 *                               awaits a HW rev that mirrors the nets.
 *   18. ADC_DSP_*            -- DONE (§C.15d + §C.24): chain_open +
 *                               stage_push implemented as a 4-chain
 *                               pool with per-stage chunk reassembly
 *                               into a 260-byte buffer.  chain_bind
 *                               validates completeness + ordering
 *                               rules (FFT terminal, WINDOW preceding
 *                               FFT) and stores the binding on both
 *                               sides.  Runtime dispatch (#496) now
 *                               LANDED: a base-level pump routes bound
 *                               FIR/IIR chains through the FAC (filtered
 *                               STREAM_READ) and FFT chains through the
 *                               FFT block (CMD_ADC_SPECTRUM_READ; a plain
 *                               STREAM_READ answers NOSUPPORT).  One FAC
 *                               + one FFT block -> one filter + one FFT
 *                               stream at a time.
 *
 * A change that moves a hook between the states above updates this
 * header comment in the same commit.  Bench cadence is bounded by
 * maintainer access to the V2N EVK, so a hook can be code-complete
 * here and still carry the `needs-silicon` label until validated --
 * see docs/BENCH.md.
 *
 * Build assumptions:
 *   - arm-none-eabi-gcc on PATH (toolchain file
 *     toolchain/arm-none-eabi.cmake handles the rest).
 *   - the GigaDevice firmware-library tree reachable, either via
 *     -DGD32_VENDOR_DIR=<path> or the ../../vendors/gd32_firmware_library
 *     fallback (see the note at the top of this file).
 *   - Cortex-M33 + Thumb + soft-float ABI (matches the GigaDevice
 *     library's compile flags).
 *
 * Layout note (2026-06-05): this backend was split move-only into per-peripheral TUs under hal/gd32/ (fw v0.2.8 refactor); this file owns boot bring-up + board-level reads, shared tables/helpers are declared in gd32_common.h.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "bridge_hw.h"

/* The wrapper's PUBLIC include directories expose the GigaDevice device
 * header.  It supplies the CMSIS/core definitions and pulls this project's
 * hal/gd32g5x3_libopt.h selector, which exposes the peripheral declarations
 * used by the real backend.  The vendor wrapper compiles its driver archive
 * independently; libopt controls declarations, not which driver units link. */
#include "gd32g5x3.h"
#include "gd32_common.h"

/* ----------------------------------------------------------------- */
/* Boot hooks (overrides of the weak defaults in src/main.c)         */
/* ----------------------------------------------------------------- */

/* Called once on entry to main() before the transport ISRs come online.
 * This brings up the backend's boot-global clocks, pads and peripheral
 * state; request-time operations live in the per-peripheral TUs.  Since
 * gh#54 a 50 ms SysTick retires the main loop's __WFI() on a fixed
 * cadence, so base-level housekeeping (bridge_hw_tick) advances whether
 * or not a transport interrupt arrives.  No DA9292 wiring exists on this
 * SoM rev; see bridge_hw_da9292_status_cached(). */
/* Sampled at the head of bridge_hw_init (#127); see gd32_common.h for
 * what reads them and why nothing acts on a mismatch yet.  Initialised to
 * the value the constants ASSUME so a debugger attaching before
 * bridge_hw_init has run does not read a spurious 0 and conclude the
 * clock tree is broken. */
uint32_t bridge_core_clock_hz      = PWM_TIMER_CLK_HZ;
bool     bridge_core_clock_matches = true;

void bridge_hw_init(void)
{
#if defined(BRIDGE_OTA_PARTITIONED) && defined(BRIDGE_APP_SLOT_BASE)
	/* OTA Path-A: the app runs from a flash slot, not 0x08000000, so move
     * the vector table off the vendor SystemInit default before any NVIC
     * use.  Runs first (main calls bridge_hw_init before the transports
     * enable interrupts). */
	SCB->VTOR = (uint32_t)(BRIDGE_APP_SLOT_BASE);
	__DSB();
	__ISB();
	/* The Path-A bootloader hands off with PRIMASK SET (it runs
     * __disable_irq() before swapping MSP/VTOR for the jump) -- without
     * clearing it here no interrupt ever fires in the slot app and the
     * transports go silent (silicon-caught 2026-06-04: CM33 retried init
     * 1600+ times against a slot app whose CS-EXTI could never run).
     * Re-enable now that OUR vector table is live; on a plain power-on
     * boot PRIMASK is already clear and this is a no-op. */
	__enable_irq();
#endif

	/* ORDERING (merge of #61's se_reset_init and #127's clock sample):
     * se_reset_init() goes FIRST and that is load-bearing -- see its
     * comment below on PC13 floating in ANALOG mode from reset.  The
     * clock sample that follows is a pure register read with no side
     * effects and derives nothing se_reset_init() needs (the SE reset is
     * a GPIO push-pull level, with no clock-derived timing anywhere in
     * hal/gd32/se_reset.c), so putting it second costs the #127 check
     * nothing: nothing between these two statements consumes
     * SystemCoreClock either. */
	/* Secure-element reset (SE_RST = PC13): promote it to a released-
     * level push-pull output FIRST, ahead of every other peripheral
     * bring-up below.  PC13 has no entry in `gpio_pad_map` (see
     * hal/gd32/gpio.c), so from reset until whichever statement first
     * touches it, it sits at its GPIO POR default -- ANALOG mode,
     * with the input buffer and both pull resistors disabled (UM
     * Rev1.2 p.270 §7.3.7): genuinely floating, not merely un-pulled.
     * The TRNG bring-up a few statements below this used to run first
     * and can alone spend up to ~1 ms in that state (hal/gd32/trng.c)
     * -- but that is NOT the dominant exposure: the pre-main() SramInit
     * loop floats PC13 for ~20-25 ms before bridge_hw_init() is ever
     * called (see the header comment in hal/gd32/se_reset.c for the
     * derivation).  That window is not fixable by moving this call any
     * earlier -- there is nowhere earlier in this backend to move it
     * to -- so gh#61 stays open after this reorder; this promotion
     * only bounds boot from main() onward and closes the separate
     * sub-microsecond OCTL-preload-vs-mode-promote gap inside
     * se_reset_configure() itself.  Needs only its own GPIOC clock;
     * the bulk AHB2 enable just below
     * re-enables it harmlessly for the rest of port C's pads. */
	rcu_periph_clock_enable(RCU_GPIOC);
	se_reset_init();

	/* #127: sample the clock the vendor's SystemInit() ACTUALLY left
     * running, before anything derived from it is programmed.  Until this
     * call the repo never referenced SystemCoreClock at all: every timing
     * constant in gd32_common.h -- PWM_TIMER_CLK_HZ, PWM_TIMER_PRESCALER,
     * BRIDGE_ADC_PACE_CLK_HZ, and the DWT "~4.63 ns LSB" claim in
     * counter.c -- was asserted against a number no code checked, set by a
     * SystemInit() that lives in another repository and whose variant is
     * chosen by a wrapper this repo does not own.
     *
     * SystemCoreClockUpdate() re-derives the value from the live RCU
     * registers rather than trusting the compile-time initialiser, so this
     * is the one place the firmware can find out it is running on the
     * wrong clock tree.  It is a pure register read plus arithmetic: no
     * side effects, safe this early.
     *
     * The result is recorded, not acted on.  See the declarations in
     * gd32_common.h for why refusing supervised outputs on a mismatch is
     * a follow-up rather than part of this change. */
	SystemCoreClockUpdate();
	bridge_core_clock_hz      = SystemCoreClock;
	bridge_core_clock_matches = (SystemCoreClock == PWM_TIMER_CLK_HZ);

	/* Enable AHB2 clocks for every GPIO port the pad map references.
     * The chip's RCU keeps unused GPIO ports clock-gated to save
     * power; we enable A..F unconditionally because the E1M IO map
     * spans those ports.  Port G isn't referenced by any pad. */
	rcu_periph_clock_enable(RCU_GPIOA);
	rcu_periph_clock_enable(RCU_GPIOB);
	rcu_periph_clock_enable(RCU_GPIOC);
	rcu_periph_clock_enable(RCU_GPIOD);
	rcu_periph_clock_enable(RCU_GPIOE);
	rcu_periph_clock_enable(RCU_GPIOF);

	/* Configure every entry in `gpio_pad_map` as INPUT + PULL_UP.
     * Safe default per the GPIO direction policy: no driven
     * contention with whatever the board might pull / drive on
     * those pads.  bridge_hw_gpio_write() promotes individual
     * pads to OUTPUT on demand. */
	for (size_t i = 0; i < GPIO_PAD_MAP_COUNT; ++i) {
		gpio_mode_set(
		    gpio_pad_map[i].periph, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, gpio_pad_map[i].pin);
		gpio_is_output[i] = false;
	}

	/* TRNG bring-up: configure + enable only.  The NIST pipeline's
     * first conditioned word can lag past any boot-time wait we are
     * willing to spin; readiness is promoted lazily by the first
     * TRNG_READ's short DRDY poll (by which point the host's boot
     * settle has given the analog source seconds, not milliseconds). */
	trng_started = trng_start();
	if (trng_started) {
		/* Boot-time readiness grace: the transports aren't up yet, so
         * spending a few milliseconds here is free and spares the
         * FIRST host TRNG_READ the one-time conditioning latency
         * (observed on the functional tier: the inaugural read paid
         * the promotion and answered STATUS_IO once).  Bounded; the
         * lazy per-read path remains the safety net. */
		for (unsigned round = 0u; round < 8u && trng_started && !trng_ready; ++round) {
			(void)trng_poll_ready();
		}
	}

	/* TMU clock enable.  Per-op configuration happens in
     * bridge_hw_tmu_compute because the mode + I/O width vary per
     * call. */
	rcu_periph_clock_enable(RCU_TMU);

	/* DAC bring-up.  Two channels per `dac_channels[]`:
     *   ch0 -> DAC0_OUT0 -> PA4
     *   ch1 -> DAC1_OUT0 -> PA6
     * Configure both pads as analog (GPIOA clock was enabled
     * above for the IO pad map), enable each DAC peripheral in
     * NORMAL_PIN_BUFFON mode so the output drives the pad
     * through the chip's built-in buffer. */
	gpio_mode_set(GPIOA, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO_PIN_4 | GPIO_PIN_6);
	rcu_periph_clock_enable(RCU_DAC0);
	rcu_periph_clock_enable(RCU_DAC1);
	for (size_t i = 0; i < DAC_CHANNEL_COUNT; ++i) {
		dac_deinit(dac_channels[i].periph);
		dac_trigger_disable(dac_channels[i].periph, dac_channels[i].out);
		dac_wave_mode_config(dac_channels[i].periph, dac_channels[i].out, DAC_WAVE_DISABLE);
		dac_mode_config(dac_channels[i].periph, dac_channels[i].out, NORMAL_PIN_BUFFON);
		dac_enable(dac_channels[i].periph, dac_channels[i].out);
	}

	/* Free-running counter: enable the Cortex-M33 DWT cycle counter.
     * TRCENA in CoreDebug->DEMCR gates the entire DWT/ITM trace block;
     * setting CYCCNTENA in DWT->CTRL starts the 32-bit free-running
     * counter at the core clock rate (216 MHz on the GD32G553 in the
     * stock clock config -> ~19.9 s wrap, ~4.63 ns LSB).  The counter
     * is the source for bridge_hw_counter_read(). */
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CYCCNT = 0u;
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

	/* PWM bring-up: configure the 8 PWM pads as alt-function outputs,
     * enable TIMER0 + TIMER7 clocks, run the per-timer + per-channel
     * init.  After this the timers are running at 1 us tick with 0%
     * duty on every channel; bridge_hw_pwm_set programs both the
     * channel compare register and the timer ARR per call. */
	for (size_t i = 0; i < PWM_CHANNEL_COUNT; ++i) {
		const gd32_pwm_ch_t *ch = &pwm_channels[i];
		gpio_mode_set(ch->gpio_port, GPIO_MODE_AF, GPIO_PUPD_NONE, ch->gpio_pin);
		gpio_output_options_set(ch->gpio_port, GPIO_OTYPE_PP, GPIO_OSPEED_12MHZ, ch->gpio_pin);
		gpio_af_set(ch->gpio_port, ch->gpio_af, ch->gpio_pin);
	}
	rcu_periph_clock_enable(RCU_TIMER0);
	rcu_periph_clock_enable(RCU_TIMER7);
	pwm_timer_init(TIMER0);
	pwm_timer_init(TIMER7);
	for (size_t i = 0; i < PWM_CHANNEL_COUNT; ++i) {
		pwm_channel_init(&pwm_channels[i]);
	}

	/* Analog REFERENCE bring-up -- MUST precede any ADC/DAC use.
     *
     * On this module revision the converters' reference node is served
     * by the GD32's ON-CHIP reference buffer -- there is no external
     * reference source (hardware rationale in the internal bench
     * notes).  At reset VREF_CS defaults to 0x02 (HIPM high-impedance):
     * the buffer is parked, the reference node is undriven, and EVERY
     * ADC channel + both DACs reference a dead node -- the entire
     * analog subsystem read garbage/zero (silently, since the old ADC
     * assertions were ceiling-only and DAC_GET echoes the digital hold
     * register, not the pad).
     *
     * Fix: enable the buffer.  Its three targets (2.048 / 2.5 /
     * 2.9 V) all exceed the module's 1.8 V VDDA, so the buffer
     * regulates as high as the rail allows (~VDDA); the lowest
     * target (2.048 V) is the closest fit and least headroom stress.
     * Bench-proven: VREFEN -> VREFRDY sets, and a DAC->ADC copper
     * loopback then tracks 1:1 (DAC 2730 -> ADC 2730).  The reference
     * cancels ratiometrically in that loop, so correctness is
     * independent of the exact railed reference value; the absolute mV
     * scale (ADC_VREF_MV / DAC_VREF_MV) tracks the railed VDDA.
     *
	 * VREFRDY wait is BOUNDED (boot-time; the gh#54 SysTick is armed
	 * only at the END of bridge_hw_init, after this block): a spin
	 * cap, not an unbounded poll -- a never-ready buffer must not hang
	 * the bridge before the transports come online. */
	rcu_periph_clock_enable(RCU_VREF);
	vref_voltage_select(VREF_VOLTAGE_SEL_2_048V); /* lowest target = closest under VDDA */
	/* CLEAR HIPM first.  VREF_CS resets to 0x02 (HIPM high-impedance),
     * and the SPL's vref_enable() is a read-modify-write that only
     * sets VREFEN -- it PRESERVES the reset HIPM bit, leaving the
     * buffer output high-Z so VREFRDY never sets and the reference
     * node stays dead (silicon-caught 2026-06-05: VREF_CS read 0x03 =
     * VREFEN|HIPM, ADC still zero).  HIPM must be cleared for the
     * buffer to drive the node. */
	vref_high_impedance_mode_disable();
	vref_enable();
	for (uint32_t vr = 0u; vr < 100000u; ++vr) {
		if (vref_status_get() == SET) break; /* VREFRDY -- buffer locked */
	}
	/* Latch the verdict.  A buffer that never locked leaves vref_ok
     * false and every ADC/DAC op answers IO instead of serving
     * garbage referenced to a dead node (the exact silent failure the
     * VREF bring-up exists to cure).  vref_ready_check() re-probes on
     * each analog op, so a late lock self-promotes. */
	vref_ok = (vref_status_get() == SET);

	/* ADC bring-up: configure 8 pads as analog, enable all four ADC
     * peripheral clocks, run the per-peripheral init.  Calibration
     * inside adc_periph_init now runs against a LIVE reference (it
     * previously self-calibrated against the undriven reference node,
     * baking in a bogus offset); the VREF bring-up above is the
     * prerequisite that makes that calibration meaningful. */
	for (size_t i = 0; i < ADC_CHANNEL_MAP_COUNT; ++i) {
		gpio_mode_set(adc_channels_map[i].gpio_port,
		              GPIO_MODE_ANALOG,
		              GPIO_PUPD_NONE,
		              adc_channels_map[i].gpio_pin);
	}
	rcu_periph_clock_enable(RCU_ADC0);
	rcu_periph_clock_enable(RCU_ADC1);
	rcu_periph_clock_enable(RCU_ADC2);
	rcu_periph_clock_enable(RCU_ADC3);
	/* Boot-time calibration result intentionally not latched: a
     * converter that fails here is also the converter every request-
     * path op re-times against (the read path's bounded EOC wait +
     * self-heal), so the failure surfaces loudly on first use instead
     * of wedging boot. */
	(void)adc_periph_init(ADC0);
	(void)adc_periph_init(ADC1);
	(void)adc_periph_init(ADC2);
	(void)adc_periph_init(ADC3);
	for (size_t i = 0; i < ADC_CHANNEL_MAP_COUNT; ++i) {
		adc_sample_cycles_cache[i]    = ADC_DEFAULT_SAMPLE_CYCLES;
		adc_resolution_bits_cache[i]  = ADC_RES_BITS_DEFAULT;
		adc_oversample_ratio_cache[i] = 1u; /* no oversampling */
	}

	/* Quadrature encoder bring-up: TIMER1..4 host the four E1M
     * encoder pairs.  Each timer uses its CH0 + CH1 input-capture
     * units as the X / Y quadrature inputs (decoder mode 2 -> X4). */
	rcu_periph_clock_enable(RCU_TIMER1);
	rcu_periph_clock_enable(RCU_TIMER2);
	rcu_periph_clock_enable(RCU_TIMER3);
	rcu_periph_clock_enable(RCU_TIMER4);
	for (size_t i = 0; i < QENC_CHANNEL_COUNT; ++i) {
		qenc_channel_init(&qenc_map[i]);
	}

	/* --- Periodic tick (gh#54) ----------------------------------------
	 *
	 * The main loop is `for (;;) { __WFI(); bridge_hw_tick(); }` and
	 * used to have NO periodic wake source at all: nothing between
	 * transport interrupts ever retired the wfi, so an armed OTA slot
	 * erase stalled mid-slot the moment the host stopped polling, and
	 * a bound DSP pump stopped producing.  Arm SysTick so the tick
	 * advances on a 50 ms cadence regardless of host traffic.
	 *
	 * 216 MHz / 20 Hz = 10 800 000 -> reload 10 799 999, inside the
	 * 24-bit 0x00FFFFFF limit (max period 77.6 ms at 216 MHz).  The
	 * reload is derived from the LIVE SystemCoreClock sampled above
	 * (#127), not a hardcoded 216 MHz literal -- on a broken clock
	 * tree the tick is still 50 ms of real time, just fewer counts.
	 * Reload is clamped to the 24-bit field so a bogusly huge
	 * SystemCoreClock cannot push LOAD out of range.
	 *
	 * NVIC priority 15 (lowest, __NVIC_PRIO_BITS = 4 on this part):
	 * the handler body is empty -- waking __WFI() is its entire job
	 * -- and it must never delay BRIDGE_CS_IRQ_PRIO 1 or
	 * BRIDGE_I2C_IRQ_PRIO 2. */
	{
		uint32_t reload = (SystemCoreClock / 20u) - 1u; /* 50 ms */
		if (reload > SysTick_LOAD_RELOAD_Msk) reload = SysTick_LOAD_RELOAD_Msk;
		(void)SysTick_Config(reload + 1u);
		NVIC_SetPriority(SysTick_IRQn, (1u << __NVIC_PRIO_BITS) - 1u); /* 15 = lowest */
	}

	/* --- Free watchdog (FWDGT, gh#54) ---------------------------------
	 *
	 * No watchdog existed anywhere in the tree (the only FWDGT trace
	 * was the after-the-fact RCU_RSTSCK decode in
	 * bridge_hw_reset_reason), so every permanent wedge -- the
	 * vendor Default_Handler infinite loop, a stalled pump, a hung
	 * flash wait -- stayed wedged until someone power-cycled the SoM.
	 *
	 * IRC32K first (defensive, from the vendor FWDGT example rather
	 * than the UM: Rev1.2 chapter 21 never says writing 0xCCCC
	 * auto-enables IRC32K, but the SPL touches no RCU register while
	 * the vendor's own FWDGT_key example brings the oscillator up
	 * first -- do not present this as a manual requirement).
	 *
	 * fwdgt_config() is preferred over an open-coded 0xCCCC sequence
	 * because it bounds its own PSC/RLD register-update waits
	 * (FWDGT_PSC_TIMEOUT / FWDGT_RLD_TIMEOUT) -- an open-coded version
	 * that writes 0xCCCC before spinning on FWDGT_STAT is spinning
	 * with the watchdog already live on its 1/4-prescaler / 0x0FFF
	 * reset defaults (512 ms) and an unbounded spin resets the part
	 * instead of reporting a fault.
	 *
	 * Window: (RLD+1) ticks at IRC32K/32 = 501 ms nominal
	 * (UM Rev1.2 p.525 Table 21-1 row 1/32), and 445 ms to 573 ms
	 * across the 28-36 kHz grade-7 IRC32K spread (Datasheet Rev2.0
	 * p.125 Table 4-23).  Sized against the longest per-tick blocking
	 * step: ota_erase_tick()'s up-to-40 ms dual-bank page-pair erase
	 * (Datasheet p.126 tERASE max 20 ms/page) -- an order of margin,
	 * and only the individual STEP must fit, because the feed below
	 * runs at the end of every bridge_hw_tick() and a pump that stops
	 * making progress stops feeding.  A full-slot walk is many such
	 * steps, each well inside the window.
	 *
	 * nFWDG_HW must stay 1 (software free watchdog, FMC_OBCTL bit 16)
	 * so the FWDGT does NOT start at power-on before firmware runs:
	 * the bootloader's no-valid-image park (src/boot/boot_main.c) is
	 * a deliberate, diagnosable stop, and a hardware-started watchdog
	 * turns it into a reset loop a bench probe has to race.  Whether
	 * a software-started FWDGT survives its own system reset is NOT
	 * documented (FWDGT_CTL's reset value has no reset-domain
	 * qualifier, UM p.526) -- that, and the FWDGSPD_DPSLP/STDBY
	 * option bits (FMC_OBCTL bits 17/18) that decide whether an armed
	 * FWDGT keeps counting through deep-sleep/standby, are bring-up
	 * gates on gh#54: read them per unit before shipping.
	 *
	 * DBG_CTL1_FWDGT_HOLD so a halted core under SWD does not reset
	 * itself out of the debugger (UM Rev1.2 p.419, DBG_CTL1 bit 12);
	 * overlaps the same-bit change in PR #211 (debug holds) -- keep
	 * both in sync, the OR is idempotent. */
	rcu_osci_on(RCU_IRC32K);
	/* Bounded stabilisation spin, same shape as
	 * rtc_wakeup_init_once() in hal/gd32/power.c -- the vendor's
	 * rcu_osci_stab_wait() is an unbounded poll, and an unbounded
	 * poll at boot is exactly the hang the FWDGT exists to prevent
	 * (and cannot, being not yet armed).  Typical < 50 us. */
	{
		uint32_t to = 200000u;
		while (--to && RESET == rcu_flag_get(RCU_FLAG_IRC32KSTB)) {
			/* spin */
		}
	}
	(void)fwdgt_config(500u, FWDGT_PSC_DIV32);
	fwdgt_enable();
	dbg_periph_enable(DBG_FWDGT_HOLD);
}

/* Periodic tick handler (gh#54): empty by design.  Its entire job is
 * to retire the main loop's __WFI() so bridge_hw_tick() runs -- the
 * tick itself must stay out of interrupt context.  Strong definition
 * overrides the weak Default_Handler alias in the vendor startup. */
void SysTick_Handler(void)
{
}

/* Called at base level after every main-loop wake.  This backend uses the
 * hook to advance work deliberately kept out of the transport ISRs.  The
 * DA9292 fault nets still reach only the Renesas, so no PMIC sampling is
 * performed here and bridge_hw_da9292_status_cached() retains its 0xFF
 * "no sample" sentinel. */
/* Base-level DSP pump (#496): drains each bound FIR/IIR stream's raw
 * samples through the FAC into its processed ring.  Runs here, off the
 * main loop -- never in the CS-EXTI stream_read path. */
extern void bridge_hw_dsp_pump(void);
/* Background OTA slot-erase pump (#770): advances a BEGIN-armed erase one
 * page-region per tick so BEGIN never blocks the SPI reply inline.  No-op
 * in the OTA-inert build. */
extern void ota_erase_tick(void);

void bridge_hw_tick(void)
{
	bridge_hw_dsp_pump();
	ota_erase_tick();
	/* Feed the free watchdog LAST (gh#54): a pump above that wedges
	 * in an unbounded loop never reaches this line, which is the
	 * entire recovery property -- the part resets ~500 ms later and
	 * the host sees the bridge reboot instead of a silent wedge.
	 * Each individual step above is bounded well inside the window
	 * (the binding one is ota_erase_tick's up-to-40 ms dual-bank
	 * erase pair), so a normally-progressing tick always feeds. */
	fwdgt_counter_reload();
}

/* ----------------------------------------------------------------- */
/* Stub bodies -- shape mirrors bridge_hw_stub.c so the gd32 backend  */
/* is binary-compatible with the stub backend pending real impls.    */
/* ----------------------------------------------------------------- */

uint8_t bridge_hw_reset_reason(void)
{
	/* Read RCU_RSTSCK (reset/clock control status register, GD32G5xx
     * Reference Manual §6.6.13) and decode the sticky reset-cause
     * flags in the high byte: PORRSTF (bit 27), BORRSTF (25),
     * EPRSTF (26, NRST pin), SWRSTF (28), FWDGTRSTF (29),
     * WWDGTRSTF (30), LPRSTF (31).
     *
     * The hardware can latch multiple flags across nested resets, so
     * we decode in coldest-first priority order: a power-on event
     * dominates a brownout, which dominates an external-pin reset,
     * which dominates a watchdog or software trigger.  Encoded byte
     * matches the host's `gd32g553_reset_cause_t` in
     * <alp/chips/gd32g553.h>:
     *
     *   0 = UNKNOWN, 1 = POWER_ON, 2 = NRST_PIN, 3 = SOFT,
     *   4 = WDT, 5 = BROWNOUT, 6 = LOWPOWER.
     *
     * RSTFC (bit 24) clears every cause flag in one write; the vendor
     * helper `rcu_all_reset_flag_clear()` is functionally identical
     * but we keep the access inline to avoid pulling rcu.c stages we
     * don't otherwise need.  After the write the next reader sees
     * UNKNOWN unless something resets the chip again. */
	const uint32_t rstsck = RCU_RSTSCK;
	uint8_t        cause  = 0u; /* UNKNOWN */

	if (rstsck & RCU_RSTSCK_PORRSTF) {
		cause = 1u; /* POWER_ON */
	} else if (rstsck & RCU_RSTSCK_BORRSTF) {
		cause = 5u; /* BROWNOUT */
	} else if (rstsck & RCU_RSTSCK_EPRSTF) {
		cause = 2u; /* NRST_PIN */
	} else if (rstsck & RCU_RSTSCK_LPRSTF) {
		cause = 6u; /* LOWPOWER */
	} else if (rstsck & (RCU_RSTSCK_FWDGTRSTF | RCU_RSTSCK_WWDGTRSTF)) {
		cause = 4u; /* WDT */
	} else if (rstsck & RCU_RSTSCK_SWRSTF) {
		cause = 3u; /* SOFT */
	}

	RCU_RSTSCK |= RCU_RSTSCK_RSTFC;
	return cause;
}

uint8_t bridge_hw_da9292_status_cached(void)
{
	/* 0xFF sentinel unconditionally: no DA9292 net reaches the GD32 on
     * this SoM rev (fault pins are Renesas P37/P36 inputs). */
	return 0xFFu;
}
