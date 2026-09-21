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
#include "bridge_hw.h" /* BRIDGE_HW_OK / BRIDGE_HW_ERR_RANGE */
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
#define BRIDGE_SPI_DMA_BUF_LEN 72u
static uint8_t spi_rx_dma_buf[BRIDGE_SPI_DMA_BUF_LEN];
static uint8_t spi_tx_dma_buf[BRIDGE_SPI_DMA_BUF_LEN];

/* One-time channel configuration (clocks, DMAMUX routing, widths).  The
 * per-transaction address/count reloads live in the arm helpers below;
 * everything here survives both the per-transaction SPI RCU reset (DMA and
 * DMAMUX are separate peripherals) and channel disable/enable cycles. */
static void spi_dma_init(void)
{
	dma_parameter_struct d;

	rcu_periph_clock_enable(BRIDGE_SPI_DMA_RCU);
	rcu_periph_clock_enable(RCU_DMAMUX);

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
	dmamux_synchronization_disable(DMAMUX_MULTIPLEXER_CH3);

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
	dmamux_synchronization_disable(DMAMUX_MULTIPLEXER_CH2);
}

/* Re-arm RX for a fresh transaction: full staging buffer.  CHCNT may only
 * be written while the channel is disabled. */
static void spi_dma_arm_rx(void)
{
	dma_channel_disable(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH);
	dma_memory_address_config(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH, (uint32_t)spi_rx_dma_buf);
	dma_transfer_number_config(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH, BRIDGE_SPI_DMA_BUF_LEN);
	dma_channel_enable(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH);
}

/* Arm TX with exactly the staged reply (never more: the GD32 SPI has no
 * TX-underrun error and no FIFO flush, so over-queued bytes would stick --
 * the same invariant the old per-byte path enforced via tx_pending()). */
static void spi_dma_arm_tx(uint32_t len)
{
	dma_channel_disable(BRIDGE_SPI_DMA, BRIDGE_SPI_TX_DMA_CH);
	if (len == 0u) {
		return;
	}
	dma_memory_address_config(BRIDGE_SPI_DMA, BRIDGE_SPI_TX_DMA_CH, (uint32_t)spi_tx_dma_buf);
	dma_transfer_number_config(BRIDGE_SPI_DMA, BRIDGE_SPI_TX_DMA_CH, len);
	dma_channel_enable(BRIDGE_SPI_DMA, BRIDGE_SPI_TX_DMA_CH);
}

static void spi_cs_exti_init(void)
{
	rcu_periph_clock_enable(RCU_SYSCFG);
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
	rcu_periph_clock_enable(BRIDGE_SPI_RCU);
	spi_gpio_init();
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
             * 1. Snapshot the RX residue FIRST: bytes captured by RX DMA =
             *    buffer length minus the remaining transfer count.
             * 2. Quiesce both DMA channels, then FLUSH + re-init the SPI via
             *    the RCU reset (the only reliable FIFO flush; it also clears
             *    BYTEN/DMAREN/DMATEN, which bridge_spi_periph_config
             *    re-applies) so the peripheral is reception-ready while the
             *    heavier decode below runs.
             * 3. Feed the captured bytes through the byte seams and decode
             *    (spi_slave_cs_high stages the reply; the all-0x00 reply-
             *    drain gate in the portable layer is unchanged).
             * 4. Drain the staged reply into the flat TX DMA buffer and
             *    re-arm: RX for a full buffer, TX for exactly the reply.
             *
             * Budget: steps 1-4 are register writes + CRC over <=69 B at
             * 216 MHz -- single-digit microseconds, well inside the master's
             * inter-transaction gap (its CS setup window alone is 60 us). */
			uint32_t remaining = dma_transfer_number_get(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH);
			uint32_t received =
			    (remaining <= BRIDGE_SPI_DMA_BUF_LEN) ? (BRIDGE_SPI_DMA_BUF_LEN - remaining) : 0u;

			dma_channel_disable(BRIDGE_SPI_DMA, BRIDGE_SPI_RX_DMA_CH);
			dma_channel_disable(BRIDGE_SPI_DMA, BRIDGE_SPI_TX_DMA_CH);

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
			spi_slave_cs_high();

			uint32_t reply_len = 0;
			while (spi_slave_tx_pending() && (reply_len < BRIDGE_SPI_DMA_BUF_LEN)) {
				spi_tx_dma_buf[reply_len++] = spi_slave_tx_next_byte();
			}

			spi_dma_arm_rx();
			spi_dma_arm_tx(reply_len);
		}
	}

	/* Group clear LAST, with the entry snapshot (gh#66): every line
	 * this vector owns that was pending at entry is cleared here.
	 * rc_w1 pending bits have no read/clear race protection: an edge
	 * landing mid-handler re-sets a bit this write then clears, so a
	 * swallowed edge can cost the dispatch that edge -- for line 8
	 * that is exactly the swallowed-edge case the idempotent re-arm
	 * above already tolerates (see the handler header, point 2), and
	 * the host driver's reply re-read / retry recovers the
	 * transaction.  Writing 0 to a rc_w1 pending bit is a no-op, so
	 * the untouched lines' zeros cost nothing. */
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
	rcu_i2c_clock_config(BRIDGE_I2C_RCU_IDX, BRIDGE_I2C_CK_SRC);
	rcu_periph_clock_enable(BRIDGE_I2C_RCU);
	i2c_gpio_init();

	uint32_t psc, scl_dely, sda_dely;
	if (!i2c_timing_derive(rcu_clock_freq_get(CK_APB1), &psc, &scl_dely, &sda_dely)) {
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

	return BRIDGE_HW_OK;
}

/* I2C0 event ISR: address match (direction-aware), RX during a write,
 * STOP, and TX during a read.
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
	if (RESET != i2c_interrupt_flag_get(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_ADDSEND)) {
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
	} else if (RESET != i2c_interrupt_flag_get(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_RBNE)) {
		i2c_slave_rx_byte((uint8_t)i2c_data_receive(BRIDGE_I2C_PERIPH));
	} else if (RESET != i2c_interrupt_flag_get(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_STPDET)) {
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
	} else if (RESET != i2c_interrupt_flag_get(BRIDGE_I2C_PERIPH, I2C_INT_FLAG_TI)) {
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
		i2c_interrupt_disable(BRIDGE_I2C_PERIPH, I2C_INT_TI);
		i2c_slave_write_start();
		/* Drop the tx side too -- see i2c_slave_tx_abort()'s banner.
         * Without this a retried read resumes from (or exhausts past)
         * a half-consumed reply instead of getting a clean answer. */
		i2c_slave_tx_abort();
	}
}
