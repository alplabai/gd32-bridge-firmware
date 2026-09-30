/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge firmware: GigaDevice silicon bring-up for the SPI + I2C
 * slave transports (gd32 HAL backend only).
 *
 * The portable transports (src/transport_spi.c, src/transport_i2c.c)
 * own framing/CRC/dispatch and expose byte-level seams.  This file
 * provides:
 *   - the strong bridge_transport_{spi,i2c}_hw_init() implementations
 *     (the stub backend keeps the weak no-op defaults), and
 *   - the SPI1 / EXTI / I2C0 interrupt handlers that override the weak
 *     vector-table symbols and feed the seams.
 *
 * Pin map + the UNCONFIRMED AF/timing facts live in
 * hal/bridge_board_config.h (see the warnings there).  Define
 * BRIDGE_WARN_UNCONFIRMED at compile time to surface them as #warnings.
 *
 * ── SPI model (DMA, 25 MHz) ────────────────────────────────────────
 * SPI1 slave, mode 0, MSB-first, 8-bit frames, hardware NSS.  The link
 * runs at 25 MHz SCK (datasheet slave max 27 MHz): a byte lands every
 * 320 ns, far inside interrupt latency, so the data path is pure DMA --
 * RX DMA captures every clocked byte into a staging buffer and TX DMA
 * streams the staged reply out (the TBE request prefills the TX FIFO
 * the moment the channel is armed, replacing the old CS-falling
 * preload).  The only SPI-side interrupt left is the CS (PA8) edge pair
 * on EXTI8, which frames transactions:
 *
 *   CS falling: reset the portable RX staging (spi_slave_cs_low).
 *   CS rising:  read the RX residue (count = buffer - DMA remaining),
 *               flush + re-init the peripheral (RCU reset -- the only
 *               reliable FIFO flush), feed the received bytes through
 *               the byte seams, decode + stage the reply, then re-arm
 *               RX DMA (full buffer) and TX DMA (exact reply length).
 *
 * Request and reply ride separate CS transactions (../src/transport_spi.c);
 * the portable framing/CRC layer is untouched -- DMA is a drop-in
 * replacement for the per-byte RBNE interrupt at the seam boundary.
 *
 * ── I2C model ──────────────────────────────────────────────────────
 * I2C0 slave at GD32_BRIDGE_DEFAULT_I2C_ADDR.  Write phase accumulates
 * bytes (RBNE); the reply is computed at the repeated-START read
 * (ADDSEND with TR=transmitter) or at STOP after a write, with SCL
 * clock-stretching covering protocol_dispatch().  TI clocks the reply.
 *
 * NOTE: the byte-level interrupt timing (SPI reply preload latency, I2C
 * clock-stretch window) needs validation on real silicon — flash
 * externally (host-driven SWD reflash is not wired in this HW rev).
 */

#include <stdbool.h>
#include <stdint.h>

#include "gd32g5x3.h"

#include "bridge_board_config.h"
#include "gd32/bridge_critical.h"
#include "bridge_hw.h" /* BRIDGE_HW_OK / BRIDGE_HW_ERR_RANGE */
#include "gd32/fault_handlers.h"
#include "gd32/i2c_event_priority.h"
#include "gd32/i2c_recovery.h"
#include "gd32/i2c_timeout.h"
#include "gd32/spi_dma_arm_status.h"
#include "protocol.h"  /* GD32_BRIDGE_DEFAULT_I2C_ADDR */
#include "transport.h" /* the seams we drive */

/* =================================================================== */
/* SPI slave bring-up                                                   */
/* =================================================================== */

static void spi_gpio_init(void)
{
	/* SCK / MISO / MOSI / NSS all to alternate-function, push-pull.
     * 85 MHz drive class: at 25 MHz SCK the slave's MISO must be valid
     * within tV(SO)=9 ns of the sampling edge -- GigaDevice's own SPI DMA
     * examples use the 85 MHz class for exactly this reason (only MISO is
     * GD32-driven; the inputs' speed class is then a don't-care, set
     * uniformly for consistency).  NSS stays in AF for the SPI
     * hardware-NSS input; EXTI taps the same pin for CS-edge detection. */
	const uint32_t af = BRIDGE_SPI_GPIO_AF;

	gpio_mode_set(BRIDGE_SPI_SCK_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BRIDGE_SPI_SCK_PIN);
	gpio_mode_set(BRIDGE_SPI_MISO_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BRIDGE_SPI_MISO_PIN);
	gpio_mode_set(BRIDGE_SPI_MOSI_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BRIDGE_SPI_MOSI_PIN);
	gpio_mode_set(BRIDGE_SPI_NSS_PORT, GPIO_MODE_AF, GPIO_PUPD_PULLUP, BRIDGE_SPI_NSS_PIN);

	gpio_output_options_set(
	    BRIDGE_SPI_SCK_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, BRIDGE_SPI_SCK_PIN);
	gpio_output_options_set(
	    BRIDGE_SPI_MISO_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, BRIDGE_SPI_MISO_PIN);
	gpio_output_options_set(
	    BRIDGE_SPI_MOSI_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, BRIDGE_SPI_MOSI_PIN);
	gpio_output_options_set(
	    BRIDGE_SPI_NSS_PORT, GPIO_OTYPE_PP, GPIO_OSPEED_85MHZ, BRIDGE_SPI_NSS_PIN);

	gpio_af_set(BRIDGE_SPI_SCK_PORT, af, BRIDGE_SPI_SCK_PIN);
	gpio_af_set(BRIDGE_SPI_MISO_PORT, af, BRIDGE_SPI_MISO_PIN);
	gpio_af_set(BRIDGE_SPI_MOSI_PORT, af, BRIDGE_SPI_MOSI_PIN);
	gpio_af_set(BRIDGE_SPI_NSS_PORT, af, BRIDGE_SPI_NSS_PIN);
}

/* ── SPI slave DMA plumbing ─────────────────────────────────────────── */

/* HAL-side DMA staging.  RX captures up to one max wire envelope
 * (1 SOF + 1 CMD/STATUS + 65 payload + 2 CRC = 69 B; padded for margin --
 * anything the master over-clocks beyond this simply stops being captured
 * and the CRC check fails loud).  TX holds the staged reply drained from
 * the portable seams at decode time so the DMA has a stable flat buffer. */
#define BRIDGE_SPI_DMA_BUF_LEN         72u
#define BRIDGE_SPI_DMA_DISABLE_SPINS   64u
#define BRIDGE_SPI_RX_FIFO_FRAMES      4u
#define BRIDGE_SPI_DMA_ERR_IRQ_PRIO    3u
#define BRIDGE_SPI_DMA_ERR_IRQ_SUBPRIO 0u
static uint8_t           spi_rx_dma_buf[BRIDGE_SPI_DMA_BUF_LEN];
static uint8_t           spi_tx_dma_buf[BRIDGE_SPI_DMA_BUF_LEN];
static volatile uint32_t spi_dma_rx_error_count;
static volatile uint32_t spi_dma_tx_error_count;
static volatile bool     spi_dma_error_pending;
/* CHEN-confirmation failures at arm time (gh#268): spi_dma_disable_confirm()
 * timing out before spi_dma_arm_{rx,tx}() reload the channel leaves that
 * channel unarmed -- RX silently deaf, or a staged reply silently never
 * sent.  Sticky counters make the failure observable over SWD even though
 * neither arm site has anywhere else to report it (init runs before any
 * host exists, and the CS EXTI handler is void); spi_dma_error_pending is
 * also set so the very next CS-rising decode routes the transaction through
 * the same STATUS_IO seam as a DMA ERRIF, instead of silently returning a
 * truncated/empty capture the host can't tell apart from "no traffic". */
static volatile uint32_t spi_dma_rx_arm_fail_count;
static volatile uint32_t spi_dma_tx_arm_fail_count;

/* DMA error IRQs deliberately run below CS EXTI (priority 1).  The CS-rising
 * handler also samples ERRIF directly, so an error that arrives just before
 * CS release cannot be hidden behind the pending lower-priority IRQ. */
static void spi_dma_latch_error(dma_channel_enum channel)
{
	if (dma_interrupt_flag_get(BRIDGE_SPI_DMA, channel, DMA_INT_FLAG_ERR) == RESET) return;
	dma_interrupt_flag_clear(BRIDGE_SPI_DMA, channel, DMA_INT_FLAG_ERR);
	if (channel == BRIDGE_SPI_RX_DMA_CH) {
		spi_dma_rx_error_count++;
	} else {
		spi_dma_tx_error_count++;
	}
	spi_dma_error_pending = true;
}

void DMA0_Channel2_IRQHandler(void)
{
	spi_dma_latch_error(BRIDGE_SPI_TX_DMA_CH);
}

void DMA0_Channel3_IRQHandler(void)
{
	spi_dma_latch_error(BRIDGE_SPI_RX_DMA_CH);
}

static bool spi_dma_error_consume(void)
{
	spi_dma_latch_error(BRIDGE_SPI_RX_DMA_CH);
	spi_dma_latch_error(BRIDGE_SPI_TX_DMA_CH);
	if (!spi_dma_error_pending) return false;
	spi_dma_error_pending = false;
	return true;
}

/* The SPL's dma_channel_disable() is one CHEN write.  The manual requires
 * observing CHEN clear before MADDR/CNT are written, so never reload a
 * channel merely because that write was issued. */
static bool spi_dma_disable_confirm(dma_channel_enum channel)
{
	dma_channel_disable(BRIDGE_SPI_DMA, channel);
	for (uint32_t spin = 0u; spin < BRIDGE_SPI_DMA_DISABLE_SPINS; ++spin) {
		if ((DMA_CHCTL(BRIDGE_SPI_DMA, channel) & DMA_CHXCTL_CHEN) == 0u) return true;
	}
	return false;
}

/* One-time channel configuration (clocks, DMAMUX routing, widths).  The
 * per-transaction address/count reloads live in the arm helpers below;
 * everything here survives both the per-transaction SPI RCU reset (DMA and
 * DMAMUX are separate peripherals) and channel disable/enable cycles. */
static void spi_dma_init(void)
{
	dma_parameter_struct d;

	bridge_rcu_periph_clock_enable(BRIDGE_SPI_DMA_RCU);
	bridge_rcu_periph_clock_enable(RCU_DMAMUX);

	/* RX: SPI1 DATA -> spi_rx_dma_buf, byte-by-byte (BYTEN makes one 8-bit
     * peripheral access == one frame), memory incrementing. */
	dma_deinit(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH);
	dma_struct_para_init(&d);
	d.request      = BRIDGE_SPI_RX_DMA_REQ;
	d.direction    = DMA_PERIPHERAL_TO_MEMORY;
	d.periph_addr  = (uint32_t)&SPI_DATA(BRIDGE_SPI_PERIPH);
	d.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
	d.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
	d.memory_addr  = (uint32_t)spi_rx_dma_buf;
	d.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
	d.memory_width = DMA_MEMORY_WIDTH_8BIT;
	d.number       = BRIDGE_SPI_DMA_BUF_LEN;
	d.priority     = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH, &d);
	dma_circulation_disable(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH);
	dma_memory_to_memory_disable(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH);
	dmamux_synchronization_disable(BRIDGE_SPI_RX_DMAMUX_CH);
	dma_flag_clear(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH, DMA_FLAG_ERR);
	dma_interrupt_enable(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH, DMA_INT_ERR);

	/* TX: spi_tx_dma_buf -> SPI1 DATA.  Armed per-reply with the exact
     * staged length; the SPI's TBE request prefills the TX FIFO the moment
     * the channel enables, so the first reply byte is ready before CS. */
	dma_deinit(BRIDGE_SPI_DMA, BRIDGE_SPI_TX_DMA_CH);
	dma_struct_para_init(&d);
	d.request      = BRIDGE_SPI_TX_DMA_REQ;
	d.direction    = DMA_MEMORY_TO_PERIPHERAL;
	d.periph_addr  = (uint32_t)&SPI_DATA(BRIDGE_SPI_PERIPH);
	d.periph_inc   = DMA_PERIPH_INCREASE_DISABLE;
	d.periph_width = DMA_PERIPHERAL_WIDTH_8BIT;
	d.memory_addr  = (uint32_t)spi_tx_dma_buf;
	d.memory_inc   = DMA_MEMORY_INCREASE_ENABLE;
	d.memory_width = DMA_MEMORY_WIDTH_8BIT;
	d.number       = 0;
	d.priority     = DMA_PRIORITY_ULTRA_HIGH;
	dma_init(BRIDGE_SPI_DMA, BRIDGE_SPI_TX_DMA_CH, &d);
	dma_circulation_disable(BRIDGE_SPI_DMA, BRIDGE_SPI_TX_DMA_CH);
	dma_memory_to_memory_disable(BRIDGE_SPI_DMA, BRIDGE_SPI_TX_DMA_CH);
	dmamux_synchronization_disable(BRIDGE_SPI_TX_DMAMUX_CH);
	dma_flag_clear(BRIDGE_SPI_DMA, BRIDGE_SPI_TX_DMA_CH, DMA_FLAG_ERR);
	dma_interrupt_enable(BRIDGE_SPI_DMA, BRIDGE_SPI_TX_DMA_CH, DMA_INT_ERR);
	nvic_irq_enable(
	    DMA0_Channel2_IRQn, BRIDGE_SPI_DMA_ERR_IRQ_PRIO, BRIDGE_SPI_DMA_ERR_IRQ_SUBPRIO);
	nvic_irq_enable(
	    DMA0_Channel3_IRQn, BRIDGE_SPI_DMA_ERR_IRQ_PRIO, BRIDGE_SPI_DMA_ERR_IRQ_SUBPRIO);
}

/* Re-arm RX for a fresh transaction: full staging buffer.  CHCNT may only
 * be written while the channel is disabled. */
static bool spi_dma_arm_rx(void)
{
	if (!spi_dma_disable_confirm(BRIDGE_SPI_RX_DMA_CH)) {
		spi_dma_arm_record_confirm_fail(&spi_dma_rx_arm_fail_count, &spi_dma_error_pending);
		return false;
	}
	dma_memory_address_config(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH, (uint32_t)spi_rx_dma_buf);
	dma_transfer_number_config(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH, BRIDGE_SPI_DMA_BUF_LEN);
	dma_channel_enable(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH);
	return true;
}

/* Arm TX with exactly the staged reply (never more: the GD32 SPI has no
 * TX-underrun error and no FIFO flush, so over-queued bytes would stick --
 * the same invariant the old per-byte path enforced via tx_pending()). */
static bool spi_dma_arm_tx(uint32_t len)
{
	if (!spi_dma_disable_confirm(BRIDGE_SPI_TX_DMA_CH)) {
		spi_dma_arm_record_confirm_fail(&spi_dma_tx_arm_fail_count, &spi_dma_error_pending);
		return false;
	}
	if (len == 0u) {
		return true;
	}
	dma_memory_address_config(BRIDGE_SPI_DMA, BRIDGE_SPI_TX_DMA_CH, (uint32_t)spi_tx_dma_buf);
	dma_transfer_number_config(BRIDGE_SPI_DMA, BRIDGE_SPI_TX_DMA_CH, len);
	dma_channel_enable(BRIDGE_SPI_DMA, BRIDGE_SPI_TX_DMA_CH);
	return true;
}

static void spi_cs_exti_init(void)
{
	bridge_rcu_periph_clock_enable(RCU_SYSCFG);
	syscfg_exti_line_config(BRIDGE_SPI_CS_EXTI_PORT, BRIDGE_SPI_CS_EXTI_PIN);
	exti_init(BRIDGE_SPI_CS_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_BOTH);
	exti_interrupt_flag_clear(BRIDGE_SPI_CS_EXTI_LINE);
	nvic_irq_enable(BRIDGE_SPI_CS_EXTI_IRQN, BRIDGE_CS_IRQ_PRIO, BRIDGE_CS_IRQ_SUBPRIO);
}

/* (Re)configure the SPI1 slave peripheral: mode-0, 8-bit, hardware-NSS,
 * full-duplex, RX + error interrupts on.  Called at init AND as the per-
 * transaction FIFO flush (after a peripheral reset -- see the CS-rising
 * handler).  GPIO/EXTI/NVIC are set up once and survive a peripheral reset, so
 * they stay in bridge_transport_spi_hw_init(). */
static void bridge_spi_periph_config(void)
{
	spi_parameter_struct sp;
	spi_struct_para_init(&sp);
	sp.device_mode          = SPI_SLAVE;
	sp.trans_mode           = SPI_TRANSMODE_FULLDUPLEX;
	sp.frame_size           = SPI_FRAMESIZE_8BIT;
	sp.nss                  = SPI_NSS_HARD;
	sp.endian               = SPI_ENDIAN_MSB;
	sp.clock_polarity_phase = SPI_CK_PL_LOW_PH_1EDGE; /* mode 0 */
	spi_init(BRIDGE_SPI_PERIPH, &sp);

	/* Byte-access the FIFO so one CPU access == one 8-bit frame.  At reset the
     * GD32 SPI FIFO is in HALF-WORD access (BYTEN=0): a 16-bit spi_data_receive()
     * then pops TWO frames per call, which the old reply path tried to unpack --
     * but on an odd-length residue (any non-even frame count, e.g. the 4-byte
     * PING split across interrupts, or the 7-byte GET_VERSION reply) the second
     * half was a phantom 0x00 that shifted the request -> CRC fail -> STATUS_IO.
     * GigaDevice's own SPI slave examples (Examples/SPI/.../slave_*) use
     * SPI_BYTE_ACCESS + one spi_data_receive()/spi_data_transmit() per frame;
     * the SPL routes byte access to the 8-bit DATA alias (gd32g5x3_spi.c), so a
     * byte access DOES advance the FIFO -- the stale "8-bit jams / 16-bit
     * required" note was wrong.  Re-applied here on every (re)config because the
     * per-transaction RCU_SPI1RST reset (CS-rising handler) clears BYTEN. */
	spi_fifo_access_size_config(BRIDGE_SPI_PERIPH, SPI_BYTE_ACCESS);

	/* DMA-driven data path: no SPI data interrupts at all (at 25 MHz a byte
     * lands every 320 ns -- interrupt service cannot keep up).  Raise the DMA
     * request lines for both directions; the actual flow is gated by the DMA
     * channel enables (the arm helpers), so a raised TBE request with the TX
     * channel disabled moves nothing.  Both CTL1 bits are cleared by the
     * per-transaction RCU_SPI1RST flush, so they are re-applied here, exactly
     * like BYTEN above. */
	spi_dma_enable(BRIDGE_SPI_PERIPH, SPI_DMA_RECEIVE);
	spi_dma_enable(BRIDGE_SPI_PERIPH, SPI_DMA_TRANSMIT);
	spi_enable(BRIDGE_SPI_PERIPH);
}

void bridge_transport_spi_hw_init(void)
{
	bridge_rcu_periph_clock_enable(BRIDGE_SPI_RCU);
	spi_gpio_init();
	spi_dma_rx_error_count    = 0u;
	spi_dma_tx_error_count    = 0u;
	spi_dma_error_pending     = false;
	spi_dma_rx_arm_fail_count = 0u;
	spi_dma_tx_arm_fail_count = 0u;
	spi_dma_init();
	bridge_spi_periph_config();
	spi_dma_arm_rx();
	/* No TX arm yet: nothing is staged until the first request decodes.
     * No SPI NVIC interrupt either -- the DMA channels carry the data and
     * the CS EXTI below carries the framing. */
	spi_cs_exti_init();
}

/* CS edge: PA8 on EXTI8.  Falling = select (reset RX, preload the staged
 * reply); rising = end of transaction (decode + stage the next reply).
 *
 * LEVEL-AWARE RECONCILIATION: EXTI pending bits coalesce, so when the
 * host paces tightly a whole CS pulse can land while this handler is
 * still busy and only ONE invocation fires -- the branch below then
 * runs whichever path matches the CURRENT pin level, and the missed
 * edge's work must not leave stale state behind.  Two rules deliver
 * that: (1) the rising path is SELF-CONTAINED -- it resets the
 * portable staging itself before feeding bytes, so a missed falling
 * edge cannot concatenate the previous frame into this decode (the
 * zero-extension/merge corruption class, silicon-caught 2026-06-04);
 * (2) the falling path (re)arms RX idempotently, so a missed rising
 * edge cannot leave the new transaction capturing into a stale DMA
 * window.  A swallowed edge still loses at most that one transaction
 * -- the host driver's reply re-read / retry recovers it. */
void BRIDGE_SPI_CS_EXTI_HANDLER(void)
{
	/* Own the WHOLE group this vector serves (gh#66): EXTI5_9_IRQn is
	 * ONE vector for lines 5..9 (UM Rev1.2 p.209), and the old
	 * single-line test + clear left lines 5/6/7/9 pending forever if
	 * any of them ever got enabled -- unbounded re-entry of this
	 * handler.  Today spi_cs_exti_init() touches only EXTI_8 and the
	 * vendor exti_init() is a per-bit RMW, so lines 5..7 and 9 keep
	 * their reset INTEN0/RTEN0/FTEN0 = 0 and cannot assert; this
	 * clear is what keeps that "latent" instead of "live" the moment
	 * any other line in the group is enabled.
	 *
	 * The specific hazard this must never paper over: line 9's default
	 * GPIO source in this design is PA9 = SPI1_SCK
	 * (hal/bridge_board_config.h:58-59).  An enabled EXTI line 9 at
	 * the 25 MHz link rate would assert on every SCK edge into this
	 * vector at BRIDGE_CS_IRQ_PRIO 1 -- not unbounded re-entry, a hard
	 * LIVELOCK that takes the bridge off the bus.  PA9 must never be
	 * given an EXTI line in any configuration that ships.  (If the
	 * CRC-and-dispatch work is ever deferred out of this ISR, use
	 * PendSV or NVIC_SetPendingIRQ on an unused vector -- never
	 * EXTI_SWIEV0/1, which burns a pin's line number and this same
	 * shared IRQ 23 vector.)
	 *
	 * EXTI_PD0 at offset 0x14 is write-1-to-clear (UM Rev1.2 p.217);
	 * 0x000003E0 = lines 5..9.  Clear exactly what was pending AT
	 * ENTRY: an edge arriving between the read and the clear must keep
	 * its pending bit and re-enter the handler, never be swallowed. */
	const uint32_t group_pd = EXTI_PD0 & 0x000003E0u;
	if ((group_pd & (1u << 8)) != 0u) {
		if (RESET == gpio_input_bit_get(BRIDGE_SPI_NSS_PORT, BRIDGE_SPI_NSS_PIN)) {
			/* CS asserted (active-low): reset the portable RX staging and
             * make sure RX capture is armed for THIS transaction even if
             * the previous rising edge was swallowed (re-arming an armed
             * channel just reloads its count -- idempotent, and CS-fall +
             * the master's setup window leaves time before the first SCK).
             * The TX DMA armed at the previous CS-rising stays untouched:
             * it holds the staged reply this transaction may be reading. */
			spi_slave_cs_low();
			spi_dma_arm_rx();
		} else {
			/* CS released: end of transaction.
             *
             * 1. Quiesce RX DMA, wait for CHEN to read clear, and execute a
             *    DSB before taking the residue.  A pending AHB beat must be
             *    visible in memory/count before the snapshot.  Then drain the
             *    (at most four-frame) byte-mode RX FIFO into the DMA tail.
             * 2. Quiesce TX DMA.  Read the SPI peripheral's RXORERR flag
             *    HERE, before anything resets it: an overrun on the
             *    just-finished receive makes the captured byte run just as
             *    untrustworthy as a DMA ERRIF, so it feeds the SAME error
             *    seam below rather than a second one.
             * 3. FLUSH + re-init the SPI via the RCU reset (the only
             *    reliable FIFO flush; it also clears BYTEN/DMAREN/DMATEN,
             *    which bridge_spi_periph_config re-applies) so the
             *    peripheral is reception-ready while the heavier decode
             *    below runs.
             * 4. Feed the captured bytes through the byte seams, then
             *    re-arm RX BEFORE decoding.  The portable layer has copied
             *    them, so a following transaction can safely reuse the DMA
             *    buffer while protocol_dispatch() handles this one -- it
             *    can perform ADC, FMC, or image-validation work, which
             *    must not leave SPI deaf to the next transaction (#152).
             * 5. Drain the staged reply into the flat TX DMA buffer and
             *    arm TX for exactly that reply.
             *
             * Budget: steps 1-4 are register writes + CRC over <=69 B at
             * 216 MHz -- single-digit microseconds, well inside the master's
             * inter-transaction gap (its CS setup window alone is 60 us);
             * protocol_dispatch() in step 4 is deliberately outside that
             * budget (see above). */
			const bool rx_quiesced = spi_dma_disable_confirm(BRIDGE_SPI_RX_DMA_CH);
			const bool tx_quiesced = spi_dma_disable_confirm(BRIDGE_SPI_TX_DMA_CH);
			if (!rx_quiesced || !tx_quiesced) {
				/* Do not decode a count from a channel which may still be
				 * transferring.  Reset the SPI state and let the host retry the
				 * dropped transaction; the next CS falling edge retries the arm. */
				rcu_periph_reset_enable(RCU_SPI1RST);
				rcu_periph_reset_disable(RCU_SPI1RST);
				bridge_spi_periph_config();
				spi_slave_cs_low();
				goto clear_group;
			}

			const bool rx_overrun = (SPI_STAT(BRIDGE_SPI_PERIPH) & SPI_STAT_RXORERR) != 0u;

			/* ONE error seam for every hardware-side fault that makes the
			 * captured byte run untrustworthy -- a DMA ERRIF (a DMA-side
			 * transfer error) OR an SPI-side RXORERR overrun -- both route
			 * through spi_slave_transport_error() so the host sees a single,
			 * consistent STATUS_IO envelope regardless of which layer
			 * caught the fault.  spi_dma_error_consume() is called
			 * unconditionally (never short-circuited away) so its latched
			 * DMA error state is always drained even when rx_overrun alone
			 * would already trip this branch. */
			const bool dma_error = spi_dma_error_consume();
			if (dma_error || rx_overrun) {
				/* The byte run is incomplete, corrupt, or the staged reply
				 * was not sent.  Reset it and stage a definite STATUS_IO for
				 * the host's next reply-read instead of decoding a truncated
				 * or overrun frame.  Re-arm RX before returning -- same
				 * arm-before-dispatch posture as the success path below,
				 * even though this path never reaches protocol_dispatch(). */
				rcu_periph_reset_enable(RCU_SPI1RST);
				rcu_periph_reset_disable(RCU_SPI1RST);
				bridge_spi_periph_config();
				spi_slave_transport_error();
				uint32_t reply_len = 0u;
				while (spi_slave_tx_pending() && reply_len < BRIDGE_SPI_DMA_BUF_LEN) {
					spi_tx_dma_buf[reply_len++] = spi_slave_tx_next_byte();
				}
				spi_dma_arm_rx();
				spi_dma_arm_tx(reply_len);
				goto clear_group;
			}

			__DSB();
			uint32_t remaining = dma_transfer_number_get(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH);
			uint32_t received =
			    (remaining <= BRIDGE_SPI_DMA_BUF_LEN) ? (BRIDGE_SPI_DMA_BUF_LEN - remaining) : 0u;
			for (uint32_t frame = 0u;
			     frame < BRIDGE_SPI_RX_FIFO_FRAMES && received < BRIDGE_SPI_DMA_BUF_LEN &&
			     spi_flag_get(BRIDGE_SPI_PERIPH, SPI_FLAG_RBNE) != RESET;
			     ++frame) {
				spi_rx_dma_buf[received++] = (uint8_t)spi_data_receive(BRIDGE_SPI_PERIPH);
			}

			rcu_periph_reset_enable(RCU_SPI1RST);
			rcu_periph_reset_disable(RCU_SPI1RST);
			bridge_spi_periph_config();

			/* Self-contained decode: reset the portable staging FIRST so a
             * swallowed falling edge cannot prepend the previous frame's
             * bytes to this one (see the handler comment). */
			spi_slave_cs_low();
			for (uint32_t i = 0; i < received; i++) {
				spi_slave_rx_byte(spi_rx_dma_buf[i]);
			}
			/* The portable staging owns a copy now. Do not defer this arm until
			 * after dispatch: a new frame arriving during a slow command must
			 * be captured, not dropped at SPI1.  rx_overrun was already ruled
			 * out above (it took the error-seam return), so this is always
			 * the clean-decode path. */
			spi_dma_arm_rx();
			spi_slave_cs_high();

			uint32_t reply_len = 0;
			while (spi_slave_tx_pending() && (reply_len < BRIDGE_SPI_DMA_BUF_LEN)) {
				spi_tx_dma_buf[reply_len++] = spi_slave_tx_next_byte();
			}

			spi_dma_arm_tx(reply_len);
		}
	}

clear_group:
	/* Group clear LAST, with the entry snapshot (gh#66): every line
	 * this vector owns that was pending at entry is cleared here.
	 * rc_w1 pending bits have no read/clear race protection: an edge
	 * landing mid-handler re-sets a bit this write then clears, so a
	 * swallowed edge can cost the dispatch that edge -- for line 8
	 * that is exactly the swallowed-edge case the idempotent re-arm
	 * above already tolerates (see the handler header, point 2), and
	 * the host driver's reply re-read / retry recovers the
	 * transaction.  Writing 0 to a rc_w1 pending bit is a no-op, so
	 * the untouched lines' zeros cost nothing.
	 *
	 * EVERY exit reaches this clear (the two error paths above jump here,
	 * never `return`): a bare return left EXTI line 8 pending and the
	 * handler re-entered immediately, spinning at BRIDGE_CS_IRQ_PRIO and
	 * starving base level. */
	EXTI_PD0 = group_pd;
}

/* =================================================================== */
/* I2C slave bring-up                                                   */
/* =================================================================== */

/* Apply the GPIOx_LOCK key sequence for the given LKy mask (gh#66):
 * "Write 1 -> Write 0 -> Write 1 -> Read 0 -> Read 1", LKK at bit 16,
 * LKy held constant across the whole sequence (UM Rev1.2 p.284
 * §7.4.8).  After the final read the port's configuration registers
 * (CTL/OMODE/OSPD/PUD/AFSEL) are frozen for the masked pins until the
 * next MCU reset; OCTL/BOP/BC/TG stay writable.  The dummy reads are
 * required steps of the documented sequence -- volatile-free hardware
 * register reads cannot be elided by the compiler anyway, but the
 * (void) casts state the intent. */
static void gpio_lock_port(uint32_t gpiox, uint16_t lky_mask)
{
	GPIO_LOCK(gpiox) = 0x00010000u | (uint32_t)lky_mask; /* LKK = 1 */
	GPIO_LOCK(gpiox) = (uint32_t)lky_mask;               /* LKK = 0 */
	GPIO_LOCK(gpiox) = 0x00010000u | (uint32_t)lky_mask; /* LKK = 1 */
	(void)GPIO_LOCK(gpiox);                              /* reads 0 */
	(void)GPIO_LOCK(gpiox);                              /* reads 1 */
}

static void i2c_gpio_init(void)
{
	const uint32_t af = BRIDGE_I2C_GPIO_AF;

	/* Open-drain; rely on the BRD_I2C bus pull-ups. */
	gpio_mode_set(BRIDGE_I2C_SCL_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BRIDGE_I2C_SCL_PIN);
	gpio_mode_set(BRIDGE_I2C_SDA_PORT, GPIO_MODE_AF, GPIO_PUPD_NONE, BRIDGE_I2C_SDA_PIN);
	/* Slowest drive class, deliberately (gh#66): OSPD = 0b00 gives
	 * tR/tF <= 14.1 ns at 2.5-3.6 V / <= 21.7 ns at 1.71-2.5 V into
	 * 30 pF (Datasheet Rev2.0 p.130 Table 4-30, speed 00) -- still
	 * 5x+ inside the 300 ns Fast-mode fall-time budget (p.141
	 * Table 4-48) -- while cutting the di/dt the 60 MHz class (9.1 ns)
	 * drives into the shared multi-drop BRD_I2C bus carrying the
	 * OPTIGA Trust M and the 5L35023B.  Marginal ringing on SCL/SDA
	 * is exactly what produces the SE clock-stretch wedge CMD_SE_RESET
	 * exists to recover from.  If the bus is ever pushed to Fast-mode
	 * plus at 1 MHz on a heavily loaded backplane, re-measure tf
	 * before assuming this class still clears 120 ns. */
	gpio_output_options_set(
	    BRIDGE_I2C_SCL_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_12MHZ, BRIDGE_I2C_SCL_PIN);
	gpio_output_options_set(
	    BRIDGE_I2C_SDA_PORT, GPIO_OTYPE_OD, GPIO_OSPEED_12MHZ, BRIDGE_I2C_SDA_PIN);
	gpio_af_set(BRIDGE_I2C_SCL_PORT, af, BRIDGE_I2C_SCL_PIN);
	gpio_af_set(BRIDGE_I2C_SDA_PORT, af, BRIDGE_I2C_SDA_PIN);
}

/* I2C_TIMING (PSC/SCLDELY/SDADELY) fields are 4 bits each (UM Rev1.2
 * p.1287-1288) -- 0-15. */
#define I2C_TIMING_FIELD_MAX 15u

/* Fast-mode floor for the I2C kernel clock: Datasheet Rev2.0 p.141
 * Table 4-48, footnote (2), "To ensure the fast mode I2C frequency,
 * fPCLK1 must be at least 4 MHz."  BRIDGE_I2C_CK_SRC feeds I2C0 from
 * CK_APB1 (RCU_I2CSRC_APB1, hal/bridge_board_config.h), i.e. fPCLK1 IS
 * apb1_hz here -- this is the acceptable I2CCLK range's documented
 * floor. */
#define I2C_APB1_MIN_HZ_FAST_MODE 4000000u

/* Derive PSC/SCLDELY/SDADELY from the LIVE APB1 kernel clock (UM Rev1.2
 * p.1261 SS28.3.4 SCLDELY/SDADELY inequalities; Datasheet Rev2.0 p.141
 * Table 4-48 Fast-mode AC timing) rather than trusting a constant tied
 * to one clock configuration.
 *
 * Why this exists (gh#12, gh#38, gh#41): the Deep-sleep wake path
 * (hal/gd32/power.c) calls bridge_transport_i2c_hw_init() while CK_APB1
 * can still be running off ~8 MHz IRC8M -- UM Rev1.2 p.142, "Deep-sleep
 * mode": "all of IRC8M, HXTAL and PLL are disabled ... When exiting the
 * Deep-sleep mode, the IRC8M is selected as the system clock", and this
 * firmware has no PLL-relock step anywhere (gh#12, still open).  A PSC/
 * SCLDELY/SDADELY set computed for 216 MHz and reused verbatim at 8 MHz
 * (an earlier revision of this function did exactly that) makes tPSC
 * ~27x coarser than intended -- multiple microseconds of granularity
 * against sub-microsecond Fast-mode delay targets -- which is a real
 * synchronisation failure, not just imprecise timing.  Re-deriving from
 * the live clock on every call (what this function did before it was
 * replaced with fixed constants) fixes the wake path directly, because
 * the inequalities below are just as solvable at 8 MHz as at 216 MHz;
 * see the worked example below.
 *
 *   tI2CCLK = 1 / apb1_hz
 *   PSC: target ~8 MHz tick (apb1_hz/8MHz, clamped to the 4-bit field);
 *        a coarser divider keeps SCLDELY/SDADELY small at high apb1_hz
 *        instead of needing near-max field values.
 *   tPSC = (PSC+1) * tI2CCLK
 *   SCLDELY >= [tr(max)+tSU;DAT(min)] / tPSC - 1
 *   SDADELY >= {tf(max)+tHD;DAT(min)-tAF(min)-[(DNF+3)*tI2CCLK]} / tPSC
 *   (tAF(min) unspecified in either document, taken as 0 -- the
 *   conservative direction for a lower bound; DNF = 0000, the reset
 *   value, since I2C_CTL0's digital-filter field is never written; all
 *   Fast-mode column values from Datasheet Rev2.0 p.141 Table 4-48:
 *   tr(max)=300ns, tSU;DAT(min)=100ns, tf(max)=300ns, tHD;DAT(min)=0ns.)
 *
 *   At apb1_hz = 216,000,000 (this board's normal run clock): PSC=15,
 *   tPSC=74.07ns, SCLDELY=5, SDADELY=4 -- the exact constants this
 *   function hardcoded before, re-derived rather than assumed.
 *   At apb1_hz = 8,000,000 (IRC8M, the Deep-sleep wake case): PSC=0,
 *   tPSC=125ns, SCLDELY=3, SDADELY=0 -- all in range; the 216 MHz
 *   constants were never valid here, but a live re-derivation is.
 *
 * Returns false -- and leaves *psc, *scl_dely, *sda_dely untouched --
 * if apb1_hz is below the Fast-mode kernel-clock floor, or if the
 * derived fields would not fit their 4-bit registers (defensive: not
 * reachable at any clock this SoC can actually run I2C0's kernel from,
 * per the two data points above, but a mis-read RCU register must
 * refuse loudly, not silently clamp into a wrong-but-plausible value --
 * that silent-clamp failure mode is exactly what gh#38 fixed once
 * already, for PSC alone). */
static bool
i2c_timing_derive(uint32_t apb1_hz, uint32_t *psc, uint32_t *scl_dely, uint32_t *sda_dely)
{
	if (apb1_hz < I2C_APB1_MIN_HZ_FAST_MODE) {
		return false;
	}

	uint32_t p = apb1_hz / 8000000u;
	if (p > 0u) {
		p -= 1u;
	}
	if (p > I2C_TIMING_FIELD_MAX) {
		p = I2C_TIMING_FIELD_MAX;
	}

	/* Picosecond fixed-point: apb1_hz up to ~500 MHz and (p+1) <= 16
     * keep every intermediate well inside 64 bits. */
	const uint64_t t_i2cclk_ps = 1000000000000ULL / apb1_hz;
	const uint64_t t_psc_ps    = (uint64_t)(p + 1u) * t_i2cclk_ps;

	const uint64_t tr_max_ps      = 300000ULL;
	const uint64_t tsu_dat_min_ps = 100000ULL;
	const uint64_t tf_max_ps      = 300000ULL;
	const uint64_t dnf3_term_ps   = 3ULL * t_i2cclk_ps; /* (DNF+3)*tI2CCLK, DNF=0 */

	/* SCLDELY >= [tr(max)+tSU;DAT(min)]/tPSC - 1, ceiling division so
     * the inequality holds for the chosen integer field value. */
	const uint64_t scldely_q = (tr_max_ps + tsu_dat_min_ps + t_psc_ps - 1u) / t_psc_ps;
	const uint32_t scl       = (scldely_q > 0u) ? (uint32_t)(scldely_q - 1u) : 0u;

	/* SDADELY >= {tf(max)-tAF(min)-[(DNF+3)*tI2CCLK]}/tPSC (tHD;DAT(min)=0
     * drops out).  Can go non-positive at a low enough apb1_hz once the
     * DNF term dominates -- SDADELY=0 satisfies that case. */
	uint32_t sda = 0u;
	if (tf_max_ps > dnf3_term_ps) {
		const uint64_t sdadely_num = tf_max_ps - dnf3_term_ps;
		sda                        = (uint32_t)((sdadely_num + t_psc_ps - 1u) / t_psc_ps);
	}

	if (p > I2C_TIMING_FIELD_MAX || scl > I2C_TIMING_FIELD_MAX || sda > I2C_TIMING_FIELD_MAX) {
		return false;
	}

	*psc      = p;
	*scl_dely = scl;
	*sda_dely = sda;
	return true;
}

int bridge_transport_i2c_hw_init(void)
{
	/* gh#257: bare RCU_CFG3 read-modify-write, same exposure class as
     * the RCU_*EN writes bridge_rcu_periph_clock_enable() already
     * protects.  bridge_transport_i2c_hw_init() also runs on the wake
     * path (hal/gd32/power.c), with interrupts live, not just at
     * boot. */
	const uint32_t i2csrc_primask_ = bridge_irq_lock();
	rcu_i2c_clock_config(BRIDGE_I2C_RCU_IDX, BRIDGE_I2C_CK_SRC);
	bridge_irq_unlock(i2csrc_primask_);
	bridge_rcu_periph_clock_enable(BRIDGE_I2C_RCU);
	i2c_gpio_init();

	const uint32_t apb1_hz = rcu_clock_freq_get(CK_APB1);
	uint32_t       psc, scl_dely, sda_dely;
	uint16_t       stretch_timeout_reload;
	if (!i2c_timing_derive(apb1_hz, &psc, &scl_dely, &sda_dely) ||
	    !bridge_i2c_stretch_timeout_reload(apb1_hz, &stretch_timeout_reload)) {
		/* Refuse rather than clamp: no i2c_timing_config()/i2c_enable()
         * below, so I2C0 stays disabled and every access on the bus
         * gets a hard failure the host/analyser can see, instead of a
         * peripheral that answers with silently wrong timing.  See
         * i2c_timing_derive()'s banner (gh#12/gh#38/gh#41) for why a
         * silent clamp is exactly the defect this refusal avoids. */
		return BRIDGE_HW_ERR_RANGE;
	}
	i2c_timing_config(BRIDGE_I2C_PERIPH, psc, scl_dely, sda_dely);
	i2c_analog_noise_filter_enable(BRIDGE_I2C_PERIPH);

	/* UM Rev1.2 §28.3.9/§28.4.6: both counters use
	 * (reload + 1) * 2048 * tI2CCLK. A normal low-SCL timeout covers a
	 * continuously stretched clock; the extended counter covers cumulative
	 * slave extension. Program both before their enable bits lock the reload
	 * fields, then let the already-enabled ERRIE path clear TIMEOUT and
	 * resynchronise the framing. The manual specifies TIMEOUT as a flag, not
	 * an automatic slave abort or SCL release; a stalled pad needs an explicit
	 * disable/reinitialise recovery path, verified on silicon (#150). */
	i2c_bus_timeout_a_config(BRIDGE_I2C_PERIPH, stretch_timeout_reload);
	i2c_bus_timeout_b_config(BRIDGE_I2C_PERIPH, stretch_timeout_reload);
	i2c_clock_timeout_enable(BRIDGE_I2C_PERIPH);
	i2c_extented_clock_timeout_enable(BRIDGE_I2C_PERIPH);

	i2c_address_config(
	    BRIDGE_I2C_PERIPH, (uint32_t)GD32_BRIDGE_DEFAULT_I2C_ADDR << 1, I2C_ADDFORMAT_7BITS);
	i2c_stretch_scl_low_enable(BRIDGE_I2C_PERIPH);

	/* Address-match, receive, stop and error always on; the transmit
     * interrupt is enabled only while serving a read.
     *
     * I2C_INT_NACK is deliberately NOT in this mask (#128).  On this IP
     * NACK is gated by its own CTL0.NACKIE, which makes it an EVENT-line
     * source -- it raises I2C0_EV, not I2C0_ER.  The EV handler below has
     * no NACK arm, so the terminating NACK the master sends before STOP at
     * the end of EVERY read latched, was never cleared, and re-entered
     * BRIDGE_I2C_EV_HANDLER immediately and permanently at group priority
     * 2.  Base level never ran again: bridge_hw_dsp_pump() and
     * ota_erase_tick() (hal/gd32/init.c) both stall there, so a bound DSP
     * chain stops producing and an armed OTA erase stops advancing.
     *
     * The clear site that looked like it covered this is in
     * BRIDGE_I2C_ER_HANDLER, on a vector NACK never raises -- see the
     * comment there.
     *
     * A slave has nothing to do with a master's end-of-read NACK: it is
     * the normal, correct end of every read, not an error, and every
     * vendor slave example leaves it masked.  NACKF still SETS in I2C_STAT
     * with NACKIE clear, it just raises no interrupt; it is cleared at the
     * end of the transaction in the STPDET arm below so it cannot
     * accumulate across transfers. */
	i2c_interrupt_enable(BRIDGE_I2C_PERIPH,
	                     I2C_INT_ADDM | I2C_INT_RBNE | I2C_INT_STPDET | I2C_INT_ERR);
	nvic_irq_enable(BRIDGE_I2C_EV_IRQN, BRIDGE_I2C_IRQ_PRIO, BRIDGE_I2C_IRQ_SUBPRIO);
	nvic_irq_enable(BRIDGE_I2C_ER_IRQN, BRIDGE_I2C_IRQ_PRIO, BRIDGE_I2C_IRQ_SUBPRIO);

	i2c_enable(BRIDGE_I2C_PERIPH);

	/* GPIOx_LOCK on the transport pads + SE_RST (gh#66).  main() runs
	 * transport_i2c_init() LAST of the two transports, and
	 * se_reset_init() ran first inside bridge_hw_init(), so this is
	 * the single point where PA8/PA9/PA10/PB15 (SPI), PA15/PB9
	 * (I2C0) and PC13 (SE_RST) are all configured.  The lock
	 * protects GPIOx_CTL/OMODE/OSPD/PUD/AFSEL (UM Rev1.2 p.271
	 * §7.3.9) against any wild write that would retarget a
	 * transport pad out of alternate function -- defence in depth,
	 * not a closed hole: a wedged supervisor with locked pads is
	 * still wedged (the issue's own framing).  OCTL/BOP/BC/TG are
	 * NOT in the protected set, so CMD_SE_RESET can still pulse PC13
	 * and the transports keep driving data.
	 *
	 * Sequence per UM p.284: Write 1 -> Write 0 -> Write 1 -> Read 0
	 * -> Read 1, LKy held constant.  GPIOA 0x8700 = PA15/PA10/PA9/
	 * PA8; GPIOB 0x8200 = PB15/PB9; GPIOC 0x2000 = PC13.
	 *
	 * Audited reconfigure paths (the issue demands this before
	 * enabling): the per-transaction SPI flush above resets SPI1
	 * only (RCU_SPI1RST) and never touches GPIO config; the
	 * Deep-sleep wake path (power.c) calls THIS function again, whose
	 * gpio_mode_set/gpio_af_set on PA15/PB9 become harmless no-ops
	 * under the lock -- GPIO config survives Deep-sleep (GPIO is in
	 * the Deep-sleep power-on module set, UM p.132 Fig 3-2), so the
	 * skipped rewrite loses nothing; i2c_timing_config and the
	 * address/enable writes touch I2C registers, not GPIO.  Nothing
	 * else in the tree reconfigures a locked pad.  The lock is
	 * irreversible until the next MCU reset -- that is the point. */
	gpio_lock_port(GPIOA, 0x8700u);
	gpio_lock_port(GPIOB, 0x8200u);
	gpio_lock_port(GPIOC, 0x2000u);

	/* SPI initialisation runs before I2C during cold boot. Reaching this
	 * successful tail therefore marks the whole transport layer healthy;
	 * the range-error exit above deliberately leaves the fault-loop count. */
	fault_reset_loop_mark_healthy();
	return BRIDGE_HW_OK;
}

/* Documented I2C0 software reset (UM Rev1.2 p.1262 SS28.3.5): "Write
 * I2CEN = 0 / Check I2CEN = 0 / Write I2CEN = 1", I2CEN held low for
 * >= 3 APB clock cycles, which "releases SCL and SDA" while leaving
 * I2C_TIMING/I2C_SADDR0/configuration bits intact.  The read-back spin
 * IS that >= 3 cycle hold and normally clears within single-digit
 * iterations.
 *
 * #251: that spin was unbounded, so a dropped I2CEN=0 write (or any
 * fault that keeps the peripheral from ever reading it back clear) hung
 * this call forever at whatever level called it, and dev has no
 * watchdog to recover the resulting hang.  Bound it with
 * bridge_i2c_en_clear_spin_exhausted() (hal/gd32/i2c_recovery.h,
 * vendor-header-free and host-tested) and escalate to a full RCU_I2C0RST
 * pulse plus a from-scratch re-init on exhaustion -- the same per-
 * peripheral reset bridge_transport_spi_hw_init()'s CS handler already
 * relies on as its own "only reliable FIFO flush" (see :~305 above), and
 * the alternative this issue named explicitly.
 *
 * #252: clearing a bus-error/timeout status flag records that the IP saw
 * a wedge, but UM SS28.3.9 documents TIMEOUT as a flag only, not an
 * automatic SCL/SDA release -- clearing it alone leaves a genuinely
 * stalled pad exactly as stalled with the evidence erased.  This helper
 * is the slave's own bounded way to force the physical release, shared
 * by the stuck-SDA poll (#39, below) and the ER-vector bus-error path
 * (below) so neither one merely reports the wedge. */
static void bridge_i2c_force_bus_release(void)
{
	I2C_CTL0(BRIDGE_I2C_PERIPH) &= ~I2C_CTL0_I2CEN; /* Write I2CEN = 0 */
	uint32_t spins = 0u;
	while (0u != (I2C_CTL0(BRIDGE_I2C_PERIPH) & I2C_CTL0_I2CEN)) {
		/* Check I2CEN = 0 -- the read-back IS the >= 3 APB cycle hold */
		if (bridge_i2c_en_clear_spin_exhausted(++spins)) {
			/* I2CEN never read back clear: the documented software
			 * reset alone cannot recover this peripheral.  Force it
			 * with the RCU peripheral reset and bring I2C0 back up
			 * from scratch rather than spin any further. */
			rcu_periph_reset_enable(RCU_I2C0RST);
			rcu_periph_reset_disable(RCU_I2C0RST);
			(void)bridge_transport_i2c_hw_init();
			return;
		}
	}
	I2C_CTL0(BRIDGE_I2C_PERIPH) |= I2C_CTL0_I2CEN; /* Write I2CEN = 1 */
}

/* Live-transfer facts for the stuck-SDA detector (#315).  Written only by
 * the I2C0 EV ISR; the base-level poll reads them, so a torn read at worst
 * delays or cancels one confirmation. */
static volatile uint32_t i2c_activity; /* bumped once per serviced event */
static volatile bool     i2c_txn_open; /* ADDSEND seen, STPDET not yet */

/* I2C0 event ISR: address match (direction-aware), RX during a write,
 * STOP, and TX during a read.
 *
 * RBNE is tested AHEAD of ADDSEND.  At a combined write/repeated-START
 * read boundary, the final write byte can still be pending in RDATA while
 * the new address match is pending.  Drain that byte before ADDSEND calls
 * i2c_slave_write_end(), so the staged reply validates the full frame.
 *
 * STPDET is tested AHEAD of TI.  At the end of a normal read, the last
 * envelope byte drains I2C_TDATA (setting TI) and the master then NACKs
 * and issues STOP (setting STPDET) essentially back-to-back, so both
 * flags are typically pending together on the interrupt that follows.
 * Servicing STPDET first disables I2C_INT_TI before the TI arm below can
 * run, so no orphan byte is ever written into TDATA for a transaction
 * that has already stopped (see the TDATA-flush comment on the read arm
 * for what happens if one gets written anyway). */
void BRIDGE_I2C_EV_HANDLER(void)
{
	const bridge_i2c_event_t event = bridge_i2c_event_select(
	    RESET != i2c_interrupt_flag_get(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_RBNE),
	    RESET != i2c_interrupt_flag_get(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_ADDSEND),
	    RESET != i2c_interrupt_flag_get(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_STPDET),
	    RESET != i2c_interrupt_flag_get(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_TI));

	if (event != BRIDGE_I2C_EVENT_NONE) i2c_activity++;
	if (event == BRIDGE_I2C_EVENT_RBNE) {
		i2c_slave_rx_byte((uint8_t)i2c_data_receive(BRIDGE_I2C_PERIPH));
	} else if (event == BRIDGE_I2C_EVENT_ADDSEND) {
		i2c_txn_open              = true;
		const bool is_transmitter = (RESET != i2c_flag_get(BRIDGE_I2C_PERIPH, I2C_FLAG_TR));
		i2c_interrupt_flag_clear(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_ADDSEND);
		if (is_transmitter) {
			/* Repeated-START read: flush TDATA FIRST -- a prior read
             * can leave an orphan byte behind (UM Rev1.2 p.1292: TBE,
             * bit 0 of I2C_STAT, is hardware-set when TDATA is empty
             * and is also software-writable to empty TDATA; it is NOT
             * one of the flags I2C_STATC can clear, see i2c_flag_clear()'s
             * documented argument list in the SPL, so this is a direct
             * write to I2C_STAT itself).  Without the flush, that orphan
             * is shifted out first on this read and displaces every
             * field of the reply envelope by one byte, permanently
             * failing the CRC the host recomputes.  Then make sure the
             * reply for the just-received write is staged, and start
             * clocking it out. */
			I2C_STAT(BRIDGE_I2C_PERIPH) |= I2C_STAT_TBE;
			(void)i2c_slave_write_end();
			i2c_interrupt_enable(BRIDGE_I2C_PERIPH, I2C_INT_TI);
		} else {
			i2c_slave_write_start();
		}
	} else if (event == BRIDGE_I2C_EVENT_STPDET) {
		i2c_txn_open = false;
		i2c_interrupt_flag_clear(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_STPDET);
		/* The master NACKs the last byte of every read before STOP, so
         * NACKF is routinely set here.  It raises no interrupt now that
         * NACKIE is masked (#128), but clear it at the transaction
         * boundary anyway so it never carries into the next transfer and
         * cannot be observed as a stale error by anything that starts
         * polling I2C_STAT later. */
		i2c_interrupt_flag_clear(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_NACK);
		i2c_interrupt_disable(BRIDGE_I2C_PERIPH, I2C_INT_TI);
		/* Belt-and-suspenders: flush here too, so a byte written by a
         * TI race that slipped in before this STPDET was serviced
         * cannot strand itself across into the next transaction. */
		I2C_STAT(BRIDGE_I2C_PERIPH) |= I2C_STAT_TBE;
		/* STOP after a write with no read: stage the reply so a later
         * separate read transaction can fetch it. */
		(void)i2c_slave_write_end();
	} else if (event == BRIDGE_I2C_EVENT_TI) {
		i2c_data_transmit(BRIDGE_I2C_PERIPH, i2c_slave_tx_next_byte());
	} else {
		/* Terminating arm (#128).  An ISR that can return having cleared
         * NOTHING is a latent permanent lockup regardless of which flag
         * caused it: the NVIC line stays asserted and the handler is
         * re-entered immediately, forever, at a priority that starves
         * base level.  That is exactly how the unmasked NACK behaved
         * before it was dropped from the enable mask, and nothing about
         * the shape was specific to NACK.
         *
         * ADDSEND, STPDET and TI cannot reach here -- the arms above test
         * them first -- and RBNE/TI are cleared by the data-register
         * access, not by a status write.  So the only software-clearable
         * flag that can land here today is NACK, and only if a future
         * change puts I2C_INT_NACK back in the enable mask.  Clearing it
         * unconditionally costs one register write on a path that should
         * never execute, and turns "someone re-enabled NACK" from a dead
         * bridge into a no-op. */
		i2c_interrupt_flag_clear(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_NACK);
	}
}

/* I2C0 error ISR: clear every bus error the enabled group (I2C_INT_ERR,
 * see bridge_transport_i2c_hw_init()) can raise, then resynchronise the
 * slave framing so the transport recovers instead of merely no longer
 * re-interrupting.  That is #7.
 *
 * LOSTARB is a master-mode condition and unreachable on this pure slave,
 * so it is not handled here.
 *
 * The NACK arm below is NOT what clears a NACK, and never was (#128).
 * NACK is gated by its own CTL0.NACKIE, which makes it an EVENT-line
 * source: it raises BRIDGE_I2C_EV_HANDLER and never this vector, so this
 * arm has never once executed for a NACK -- which is why "the NACK is
 * cleared somewhere" read as true on inspection and was false in
 * execution.  The terminating NACK of every read is handled by masking
 * NACKIE in bridge_transport_i2c_hw_init() and clearing NACKF at the
 * transaction boundary in the EV handler's STPDET arm above.  This arm
 * stays only as a cheap safety net in case a future part or vendor-header
 * revision routes NACKF differently. */
void BRIDGE_I2C_ER_HANDLER(void)
{
	const uint32_t stat      = I2C_STAT(BRIDGE_I2C_PERIPH); /* snapshot before clearing */
	bool           bus_error = false;

	if (RESET != i2c_interrupt_flag_get(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_NACK)) {
		i2c_interrupt_flag_clear(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_NACK);
	}
	if (RESET != i2c_interrupt_flag_get(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_BERR)) {
		i2c_interrupt_flag_clear(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_BERR);
		bus_error = true;
	}
	if (RESET != i2c_interrupt_flag_get(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_OUERR)) {
		i2c_interrupt_flag_clear(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_OUERR);
		bus_error = true;
	}
	if (RESET != i2c_interrupt_flag_get(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_TIMEOUT)) {
		/* A timeout leaves the request/reply framing untrustworthy even
		 * though the IP only exposes it as a status flag. Clear it and use
		 * the same portable-side resynchronisation as a bus error; otherwise
		 * the next address match can append to a transaction that timed out
		 * while this handler was pre-empted. */
		i2c_interrupt_flag_clear(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_TIMEOUT);
		/* #252: clearing the flag only records that the IP saw the
		 * wedge -- UM SS28.3.9 specifies TIMEOUT as a flag, not an
		 * automatic slave abort or SCL/SDA release, so a genuinely
		 * stalled pad stays stalled with the evidence now erased.
		 * Force the documented release so this vector is the slave's
		 * own bounded way out, instead of waiting on whatever else
		 * next touches the bus. */
		bridge_i2c_force_bus_release();
		bus_error = true;
	}

	/* Catch-all: blanket-clear every software-clearable ERROR-domain
     * status bit the snapshot shows set -- the safety net for a source
     * this handler does not model by name (LOSTARB/PECERR/TIMEOUT/
     * SMBALT).  I2C_STATC mirrors I2C_STAT's bit positions (UM Rev1.2
     * p.1291-1292 vs p.1292-1293), so writing the snapshot masked to
     * these bits back into I2C_STATC clears exactly the set ones in one
     * access.  This must not be the only thing that keeps the vector
     * from level-holding; the explicit arms above are the contract.
     *
     * ADDSEND and STPDET are deliberately EXCLUDED from this mask, even
     * though UM p.1292-1293 lists them as clearable the same way.  Both
     * are EV-domain events (serviced by BRIDGE_I2C_EV_HANDLER, never
     * raised by anything this ER handler itself does), but I2C_STAT is
     * one shared register: if either was pending in the `stat` snapshot
     * above because the EV IRQ simply hadn't run yet, including them
     * here would clear them out from under BRIDGE_I2C_EV_HANDLER and
     * lose an address match or a STOP.  An earlier revision of this
     * mask included them and only held because BRIDGE_I2C_EV_IRQN (31)
     * sorts below BRIDGE_I2C_ER_IRQN (32) at the same NVIC priority
     * (hal/bridge_board_config.h) -- correct per the Cortex-M tie-break
     * rule (lower IRQn serviced first among equal-priority pendings),
     * but an unstated dependency an unrelated IRQn/priority change could
     * silently break.  Narrowing the mask to bits this handler actually
     * owns removes that dependency instead of merely documenting it. */
	I2C_STATC(BRIDGE_I2C_PERIPH) =
	    stat & (I2C_STAT_NACK | I2C_STAT_BERR | I2C_STAT_LOSTARB | I2C_STAT_OUERR |
	            I2C_STAT_PECERR | I2C_STAT_TIMEOUT | I2C_STAT_SMBALT);

	if (bus_error) {
		i2c_txn_open = false;
		i2c_interrupt_disable(BRIDGE_I2C_PERIPH, I2C_INT_TI);
		i2c_slave_write_start();
		/* Drop the tx side too -- see i2c_slave_tx_abort()'s banner.
         * Without this a retried read resumes from (or exhausts past)
         * a half-consumed reply instead of getting a clean answer. */
		i2c_slave_tx_abort();
	}
}

/* =====================================================================
 * BRD_I2C stuck-SDA detector (gh#39) -- erratum 2.3.1 workaround.
 *
 * "Device limitations of GD32G5x3 Rev1.0" s2.3.1: a 7-bit-address I2C
 * slave "will enter an error state, causing it to malfunction and the
 * SDA line to remain low" when a master that simulates I2C via IO
 * (an i2c-gpio adapter, a bench probe, an `i2cdetect -a` sweep of the
 * reserved head addresses) sends "Start + 10-bit Match Head Address +
 * Start + 7-bit Address Read + Wait ACK + Start".  This bridge runs
 * I2C0 exactly in that configuration (I2C_ADDFORMAT_7BITS).  The SDA
 * pad held low means no master on the shared multi-drop BRD_I2C bus
 * (OPTIGA Trust M, 5L35023B) can issue a START -- a bus-wide outage,
 * while the SPI link keeps answering, which is the least diagnosable
 * form this fault can take.
 *
 * Vendor workaround, verbatim: "Software periodically checks the status
 * of the SDA line. If SDA is detected to be stuck low, reinitialize
 * the I2C module."  Called from bridge_hw_tick() at base level, which
 * runs after each main-loop wake, and at least every 50 ms via the
 * periodic SysTick (gh#54) even with no other interrupt traffic.
 *
 * Detector design: a single instantaneous pad read is ambiguous -- a
 * legitimate in-flight byte holds SDA low ~half the bit times, so two
 * bare tick samples could both land on data bits and tear down a
 * healthy transfer (continuous back-to-back I2C traffic, e.g. an OTA
 * streamed over this bus, would eventually hit that pair by chance).
 * Instead each tick takes a BURST of samples ~1 ms apart, and a sample
 * only counts as stuck when SDA is low WHILE SCL IS HIGH.  SDA low on
 * its own is legal for as long as the master likes: a master that
 * pauses between bytes holds SCL low, and after it ACKs a byte (or
 * while this slave shifts a 0 bit) SDA is low too.  A Linux RIIC master
 * early in a cold boot pauses for milliseconds, and the earlier
 * SDA-only test fired mid-read, reset the slave and tore the reply
 * (kernel probe -EBADMSG, reply "00 00 0e 00 ff ff", #295).  The
 * erratum wedge is the other case: SCL released high, SDA held low, so
 * no master can issue a START.  BRIDGE_I2C_STUCK_SAMPLES consecutive
 * SDA-low/SCL-high readings spanning ~3 ms is ~1200 bit times --
 * unreachable in correct traffic (SDA only changes while SCL is low,
 * apart from START/STOP).  Only then does the tick count as a "low
 * candidate", and only TWO consecutive candidate ticks act (the
 * issue's confirmation rule).
 *
 * #315: that was still not enough.  A master preempted with SCL high in
 * the ACK clock phase of a byte (this slave drives the ACK, so SDA is
 * low) satisfies the burst, and the detector released I2C0 ten times
 * inside a healthy 65-byte OTA_WRITE_CHUNK write, one byte short of the
 * end.  A real wedge is SILENT: no event ISR runs.  So any I2C event
 * since the previous poll cancels the candidate, and while a transaction
 * is open (ADDSEND, no STPDET) the silence must also outlast
 * BRIDGE_I2C_STUCK_LIVE_QUIET_TICKS SysTick periods.  The decision is
 * bridge_i2c_stuck_confirm() in hal/gd32/i2c_recovery.h (host-tested).
 *
 * AF-mode pads still report the live line state (UM Rev1.2 p.270
 * s7.3.8: "A read access to the port input status register gets the
 * I/O state"), so gpio_input_bit_get() on PB9 is a valid detector.
 *
 * Recovery is the documented I2C software reset (UM Rev1.2 p.1262
 * s28.3.5): "Write I2CEN = 0 / Check I2CEN = 0 / Write I2CEN = 1",
 * I2CEN held low >= 3 APB clock cycles, which "releases SCL and SDA"
 * and leaves I2C_TIMING / I2C_SADDR0 / configuration bits intact --
 * safe to run from a tick, and preferable to re-running
 * bridge_transport_i2c_hw_init().  The portable staging is resynced
 * with i2c_slave_tx_abort() so a half-consumed reply from before the
 * wedge is dropped rather than resumed against a fresh peripheral. */
#define BRIDGE_I2C_STUCK_SAMPLES 4u

/* 50 ms SysTick period count (hal/gd32/init.c). */
extern volatile uint32_t bridge_systick_count;

static bridge_i2c_stuck_t i2c_stuck;

void bridge_transport_i2c_stuck_poll(void)
{
	/* Burst sample: SDA low with SCL high across the whole burst is the
	 * candidate. */
	bool all_low = true;
	for (uint32_t k = 0u; k < BRIDGE_I2C_STUCK_SAMPLES; ++k) {
		if (RESET != gpio_input_bit_get(BRIDGE_I2C_SDA_PORT, BRIDGE_I2C_SDA_PIN) ||
		    RESET == gpio_input_bit_get(BRIDGE_I2C_SCL_PORT, BRIDGE_I2C_SCL_PIN)) {
			/* SDA high, or SCL held low by a master mid-transfer: not
			 * the erratum wedge (see the banner). */
			all_low = false;
			break;
		}
		if (k + 1u < BRIDGE_I2C_STUCK_SAMPLES) {
			/* ~1 ms gap: 216000 cycles at 216 MHz, ~5 cycles per
			 * volatile iteration -> 43200 iterations.  Only paid
			 * on the stuck path; a healthy line exits at the first
			 * sample. */
			for (volatile uint32_t gap = 0u; gap < 43200u; ++gap) {
				/* spread samples ~1 ms apart */
			}
		}
	}
	/* Activity is read AFTER the burst so an event during it cancels. */
	if (!bridge_i2c_stuck_confirm(
	        &i2c_stuck, all_low, i2c_activity, i2c_txn_open, bridge_systick_count)) {
		return;
	}
	i2c_txn_open = false;

	/* Documented software reset (UM Rev1.2 p.1262 s28.3.5), bounded and
	 * shared with the ER-vector path -- see bridge_i2c_force_bus_release()
	 * above (#251). */
	bridge_i2c_force_bus_release();
	i2c_slave_tx_abort(); /* drop a half-consumed staged reply */
}
