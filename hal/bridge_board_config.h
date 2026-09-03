/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * gd32-bridge firmware: board/silicon configuration for the transport
 * peripherals on the E1M-X V2N module's GD32G553MEY7TR supervisor.
 *
 * This header is included ONLY by the gd32 HAL backend
 * (hal/transport_hw_gd32.c); it references GigaDevice register macros.
 *
 * SOURCE OF TRUTH for the pin map (alp-sdk):
 *   metadata/e1m_modules/v2n/gd32-io-mcu-map.tsv
 *   metadata/chips/gd32g553.yaml
 * Peripheral + pin assignments below are CONFIRMED against the
 * GD32G553xx datasheet (Rev2.0) pin-definition table and the team
 * "GD32 Pin Config" sheet: PA8/PA9/PA10/PB15 = SPI1 NSS/SCK/MISO/MOSI,
 * PA15 = I2C0_SCL, PB9 = I2C0_SDA.  AF numbers follow the GD32G5 scheme
 * (SPI→AF5, I2C→AF4), matching the vendor SPI/I2C driver examples.
 *
 * I2C timing is derived at runtime from the live APB1 clock (in
 * bridge_transport_i2c_hw_init), so there are no unconfirmed silicon
 * facts left here.  SPI runs on RX+TX DMA (armed per transaction from the
 * CS-EXTI); the on-silicon validation point is the master's CS-to-first-SCK
 * setup vs the EXTI+DMA-arm latency, plus the I2C clock-stretch window.
 * Flash externally (host-driven SWD reflash is not wired in this HW revision).
 */
#ifndef GD32_BRIDGE_BOARD_CONFIG_H
#define GD32_BRIDGE_BOARD_CONFIG_H

#include "gd32g5x3.h"

/* =================================================================== */
/* SPI slave transport — dedicated point-to-point link to the Renesas. */
/* Renesas master P76(MOSI)/P77(MISO)/P96(SCLK)/P97(CS) <-> GD32 below. */
/* Mode 0 (CPOL=0,CPHA=0), MSB-first, 8-bit, hardware NSS.             */
/* =================================================================== */
#define BRIDGE_SPI_PERIPH      SPI1
#define BRIDGE_SPI_RCU         RCU_SPI1
#define BRIDGE_SPI_IRQN        SPI1_IRQn
#define BRIDGE_SPI_IRQ_HANDLER SPI1_IRQHandler

/* SPI1 slave DMA (25 MHz link: one byte every 320 ns -- an interrupt per
 * byte cannot keep up even at the 216 MHz core, so RX and TX stream over
 * DMA and the only SPI-side interrupts left are the CS edges on EXTI8).
 * DMA0 CH2(TX)/CH3(RX) mirror GigaDevice's own
 * SPI_master_slave_fullduplex_dma reference; DMA0 CH0 is owned by ADC
 * stream 0 (hal/gd32/adc_stream.c) -- keep clear of it.  Requests route through
 * the DMAMUX (SPI1_RX = mux id 12, SPI1_TX = mux id 13). */
#define BRIDGE_SPI_DMA        DMA0
#define BRIDGE_SPI_DMA_RCU    RCU_DMA0
#define BRIDGE_SPI_TX_DMA_CH  DMA_CH2
#define BRIDGE_SPI_RX_DMA_CH  DMA_CH3
#define BRIDGE_SPI_TX_DMA_REQ DMA_REQUEST_SPI1_TX
#define BRIDGE_SPI_RX_DMA_REQ DMA_REQUEST_SPI1_RX

/* GD32-side pins (gd32-io-mcu-map.tsv): SCLK PA9, MISO PA10,
 * MOSI PB15, NSS/CS PA8. */
#define BRIDGE_SPI_SCK_PORT  GPIOA
#define BRIDGE_SPI_SCK_PIN   GPIO_PIN_9
#define BRIDGE_SPI_MISO_PORT GPIOA
#define BRIDGE_SPI_MISO_PIN  GPIO_PIN_10
#define BRIDGE_SPI_MOSI_PORT GPIOB
#define BRIDGE_SPI_MOSI_PIN  GPIO_PIN_15
#define BRIDGE_SPI_NSS_PORT  GPIOA
#define BRIDGE_SPI_NSS_PIN   GPIO_PIN_8

/* CS/NSS edge detection: SPI-slave hardware NSS gives NO CS-edge
 * interrupt, so the CS line is mirrored onto EXTI to drive the
 * cs_low/cs_high seams.  PA8 -> EXTI line 8 -> EXTI[9:5] vector. */
#define BRIDGE_SPI_CS_EXTI_LINE    EXTI_8
#define BRIDGE_SPI_CS_EXTI_PORT    EXTI_SOURCE_GPIOA
#define BRIDGE_SPI_CS_EXTI_PIN     EXTI_SOURCE_PIN8
#define BRIDGE_SPI_CS_EXTI_IRQN    EXTI5_9_IRQn
#define BRIDGE_SPI_CS_EXTI_HANDLER EXTI5_9_IRQHandler

/* SPI1 alternate function = AF5 (GD32G5 AF scheme; vendor SPI examples).
 * The datasheet lists SPI1 NSS/SCK/MISO/MOSI on PA8/PA9/PA10/PB15
 * (these pins also offer I2C1/I2C2 on other AFs — not used here). */
#define BRIDGE_SPI_GPIO_AF GPIO_AF_5

/* =================================================================== */
/* I2C slave transport — shared BRD_I2C management bus.                */
/* 7-bit slave address from the protocol header (default 0x70).        */
/* =================================================================== */
#define BRIDGE_I2C_PERIPH     I2C0
#define BRIDGE_I2C_RCU        RCU_I2C0
#define BRIDGE_I2C_RCU_IDX    IDX_I2C0
#define BRIDGE_I2C_CK_SRC     RCU_I2CSRC_APB1
#define BRIDGE_I2C_EV_IRQN    I2C0_EV_WKUP_IRQn
#define BRIDGE_I2C_EV_HANDLER I2C0_EV_WKUP_IRQHandler
#define BRIDGE_I2C_ER_IRQN    I2C0_ER_IRQn
#define BRIDGE_I2C_ER_HANDLER I2C0_ER_IRQHandler

/* GD32-side pins (gd32-io-mcu-map.tsv): SCL PA15, SDA PB9. */
#define BRIDGE_I2C_SCL_PORT GPIOA
#define BRIDGE_I2C_SCL_PIN  GPIO_PIN_15
#define BRIDGE_I2C_SDA_PORT GPIOB
#define BRIDGE_I2C_SDA_PIN  GPIO_PIN_9

/* I2C0 alternate function = AF4 (GD32G5 AF scheme; vendor I2C examples).
 * Datasheet lists I2C0_SCL on PA15 and I2C0_SDA on PB9. */
#define BRIDGE_I2C_GPIO_AF GPIO_AF_4

/* I2C timing (PSC / SCLDEL / SDADEL) is computed at runtime in
 * bridge_transport_i2c_hw_init() from the live APB1 kernel clock, so it
 * needs no static value here.  Slave timing only needs setup/hold +
 * prescaler; the 400 kHz bus rate is the master's SCLH/SCLL, not ours. */

/* NVIC priorities (preemption, sub).  Transport ISRs sit above the
 * SysTick PMIC poll; SPI (point-to-point, latency-sensitive) above I2C. */
#define BRIDGE_SPI_IRQ_PRIO    1u
#define BRIDGE_SPI_IRQ_SUBPRIO 0u
#define BRIDGE_CS_IRQ_PRIO     1u
#define BRIDGE_CS_IRQ_SUBPRIO  1u
#define BRIDGE_I2C_IRQ_PRIO    2u
#define BRIDGE_I2C_IRQ_SUBPRIO 0u

/* =================================================================== */
/* RTC / wakeup-timer clock source -- LXTAL is not available on this   */
/* SoM, so every timed wake the bridge offers is IRC32K-bound.         */
/* =================================================================== */
/* LXTAL needs OSC32IN/OSC32OUT, i.e. PC14/PC15 (GD32G553xx Datasheet
 * Rev2.0, p.54, WLCSP81 balls C9/D9).  Both pads are already spent as
 * host-addressable E1M IO24/IO25 (hal/gd32/gpio.c gpio_pad_map, bits
 * 8/9) -- enabling the crystal would break the wire contract.  IRC32K
 * is therefore the only legal RTC clock source (RCU_BDCTL RTCSRC =
 * 0b10, GD32G553 User Manual Rev1.2 p.199; selected in
 * hal/gd32/power.c rtc_wakeup_init_once()).
 *
 * IRC32K is NOT 32.768 kHz and is specified only as a range, never a
 * fixed value.  GD32G553xx Datasheet Rev2.0, p.125, Table 4-23
 * (identical in Rev1.6 and Rev1.5, p.125; Rev1.3, p.127, still
 * carried "TBD" in the Min/Max cells at that point in the part's
 * characterization -- the 28-36 kHz bound was not committed until
 * Rev1.5/1.6):
 *
 *   fIRC32K, TA = -40 to 105 degC (grade-7 -- matches the
 *   GD32G553MEY7TR on this SoM, Datasheet Rev2.0 p.175 Table 6-1):
 *   Min 28 kHz, Max 36 kHz (no Typ given at this TA range).
 *   At TA = 25 degC only: Min 30 kHz, Typ 32.0 kHz, Max 34 kHz.
 *   Both Min/Max columns are footnoted "guaranteed by
 *   characterization, not 100% tested in production".
 *
 * Over the part's full operating range the RTC wakeup-timer LSB
 * (RTC/16 divider; POWER_WAKE_LSB_HZ in hal/gd32/power.c) is
 * therefore accurate to about -11.1% / +14.3% of any requested
 * wake_after_ms -- not the fixed 0.5 ms/tick that constant alone
 * would suggest.  This cannot be trimmed away: the RCU has no
 * IRC32K trim/calibration register (only IRC32KEN/IRC32KSTB enable
 * and stability-flag bits, UM Rev1.2 p.201-202), and the RTC's own
 * RTC_HRFC digital calibration (UM Rev1.2 p.543, range -487.1 to
 * +488.5 ppm) corrects a fixed offset against a known reference --
 * it has no temperature input and cannot track IRC32K's drift over
 * -40..105 degC. */

#endif /* GD32_BRIDGE_BOARD_CONFIG_H */
