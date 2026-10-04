/* SPDX-License-Identifier: Apache-2.0 */
/*
 * Minimal host-only GD32G5x3 SPL surface, just enough to compile the REAL
 * hal/transport_hw_gd32.c so its CS-edge EXTI handler can be driven on the
 * host.  Void vendor calls are no-op macros; the handful whose result steers
 * the handler (pin level, DMA CHEN, DMA error flag, SPI flags) are backed by
 * the mock_* knobs below.  Nothing here models timing.
 */
#ifndef GD32_BRIDGE_CS_EXTI_MOCK_GD32G5X3_H
#define GD32_BRIDGE_CS_EXTI_MOCK_GD32G5X3_H

#include <stdbool.h>
#include <stdint.h>

#define __get_PRIMASK()  0u
#define __set_PRIMASK(x) ((void)(x))
#define __disable_irq()  ((void)0)
#define __DSB()          ((void)0)

#define RESET 0u
#define SET   1u
typedef uint32_t FlagStatus;
typedef uint32_t dma_channel_enum;
typedef uint32_t rcu_periph_enum;
typedef uint32_t rcu_periph_reset_enum;
typedef uint32_t IRQn_Type;

typedef struct {
	uint32_t device_mode, trans_mode, frame_size, nss, endian, clock_polarity_phase, prescale;
} spi_parameter_struct;
typedef struct {
	uint32_t request, direction, periph_addr, periph_inc, periph_width, memory_addr, memory_inc,
	    memory_width, number, priority;
} dma_parameter_struct;

/* Register accesses: one shared scratch word, except the ones the handler
 * decides on.  EXTI_PD0 goes through a counting accessor so the test can see
 * the write-1-to-clear store happened (see mock_exti_pd0()). */
extern uint32_t mock_scratch;
extern uint32_t mock_dma_chctl;
extern uint32_t mock_spi_stat;
uint32_t       *mock_exti_pd0(void);
#define EXTI_PD0 (*mock_exti_pd0())
/* DHCSR backing word (bridge_hw_debugger_attached): bit 0 = C_DEBUGEN. */
extern uint32_t mock_dhcsr;
#define BRIDGE_DHCSR     (mock_dhcsr)
#define DMA_CHCTL(d, ch) mock_dma_chctl
#define DMA_CHXCTL_CHEN  1u
#define SPI_STAT(p)      mock_spi_stat
#define SPI_STAT_RXORERR 0x40u
#define SPI_DATA(p)      mock_scratch
#define I2C_CTL0(p)      mock_scratch
#define GPIO_LOCK(p)     mock_scratch
#define I2C_STAT(p)      mock_scratch
#define I2C_STATC(p)     mock_scratch

/* Value-returning vendor calls, defined in the test. */
FlagStatus gpio_input_bit_get(uint32_t port, uint32_t pin);
FlagStatus dma_interrupt_flag_get(uint32_t d, uint32_t ch, uint32_t f);
FlagStatus spi_flag_get(uint32_t p, uint32_t f);
FlagStatus i2c_flag_get(uint32_t p, uint32_t f);
FlagStatus i2c_interrupt_flag_get(uint32_t p, uint32_t f);
uint32_t   dma_transfer_number_get(uint32_t d, uint32_t ch);
uint32_t   spi_data_receive(uint32_t p);
uint32_t   i2c_data_receive(uint32_t p);
uint32_t   rcu_clock_freq_get(uint32_t c);

/* GPIO calls, defined in the test: the ATTN pin's drive and mode sequence is
 * the thing the ATTN cases pin, so these record rather than swallow. */
void gpio_bit_set(uint32_t port, uint32_t pin);
void gpio_bit_reset(uint32_t port, uint32_t pin);
void gpio_mode_set(uint32_t port, uint32_t mode, uint32_t pupd, uint32_t pin);
void gpio_output_options_set(uint32_t port, uint32_t otype, uint32_t speed, uint32_t pin);
void gpio_af_set(uint32_t port, uint32_t af, uint32_t pin);

/* Void vendor calls: swallowed. */
#define dma_channel_disable(...)               ((void)0)
#define dma_channel_enable(...)                ((void)0)
#define dma_circulation_disable(...)           ((void)0)
#define dma_deinit(...)                        ((void)0)
#define dma_flag_clear(...)                    ((void)0)
#define dma_init(...)                          ((void)0)
#define dma_interrupt_enable(...)              ((void)0)
#define dma_interrupt_flag_clear(...)          ((void)0)
#define dma_memory_address_config(...)         ((void)0)
#define dma_memory_to_memory_disable(...)      ((void)0)
#define dma_struct_para_init(...)              ((void)0)
#define dma_transfer_number_config(...)        ((void)0)
#define dmamux_synchronization_disable(...)    ((void)0)
#define exti_init(...)                         ((void)0)
#define exti_interrupt_flag_clear(...)         ((void)0)
#define i2c_address_config(...)                ((void)0)
#define i2c_analog_noise_filter_enable(...)    ((void)0)
#define i2c_bus_timeout_a_config(...)          ((void)0)
#define i2c_bus_timeout_b_config(...)          ((void)0)
#define i2c_clock_timeout_enable(...)          ((void)0)
#define i2c_data_transmit(...)                 ((void)0)
#define i2c_enable(...)                        ((void)0)
#define i2c_extented_clock_timeout_enable(...) ((void)0)
#define i2c_flag_clear(...)                    ((void)0)
#define i2c_interrupt_disable(...)             ((void)0)
#define i2c_interrupt_enable(...)              ((void)0)
#define i2c_interrupt_flag_clear(...)          ((void)0)
#define i2c_stretch_scl_low_enable(...)        ((void)0)
#define i2c_timing_config(...)                 ((void)0)
#define nvic_irq_enable(...)                   ((void)0)
#define rcu_i2c_clock_config(...)              ((void)0)
#define rcu_periph_clock_enable(...)           ((void)0)
#define rcu_periph_reset_disable(...)          ((void)0)
#define rcu_periph_reset_enable(...)           ((void)0)
#define spi_data_transmit(...)                 ((void)0)
#define spi_dma_enable(...)                    ((void)0)
#define spi_enable(...)                        ((void)0)
#define spi_fifo_access_size_config(...)       ((void)0)
#define spi_init(...)                          ((void)0)
#define spi_struct_para_init(...)              ((void)0)
#define syscfg_exti_line_config(...)           ((void)0)

/* Vendor constants the handler/bring-up code names but never decides on. */
#define CK_APB1                     0u
#define DMA0                        0u
#define DMA_CH2                     0u
#define DMA_CH3                     0u
#define DMA_INT_FLAG_ERR            0u
#define DMA_MEMORY_INCREASE_ENABLE  0u
#define DMA_MEMORY_TO_PERIPHERAL    0u
#define DMA_MEMORY_WIDTH_8BIT       0u
#define DMA_PERIPHERAL_TO_MEMORY    0u
#define DMA_PERIPHERAL_WIDTH_8BIT   0u
#define DMA_PERIPH_INCREASE_DISABLE 0u
#define DMA_PRIORITY_ULTRA_HIGH     0u
#define DMA_REQUEST_SPI1_RX         0u
#define DMA_REQUEST_SPI1_TX         0u
#define GPIOA                       0u
#define GPIO_AF_4                   0u
#define GPIO_AF_5                   0u
#define GPIOB                       0u
#define GPIOC                       0u
#define GPIO_PIN_14                 (1u << 14)
#define GPIO_PIN_10                 0u
#define GPIO_PIN_15                 0u
#define GPIO_PIN_8                  0u
#define GPIO_PIN_9                  0u
/* Distinct values so the ATTN cases can tell the mode / pull / speed apart. */
#define GPIO_MODE_AF             1u
#define GPIO_MODE_OUTPUT         2u
#define GPIO_PUPD_NONE           0u
#define GPIO_PUPD_PULLUP         1u
#define GPIO_PUPD_PULLDOWN       2u
#define GPIO_OTYPE_PP            0u
#define GPIO_OTYPE_OD            1u
#define GPIO_OSPEED_12MHZ        0u
#define GPIO_OSPEED_85MHZ        3u
#define GPIO_AF_0                0u
#define I2C0                     0u
#define I2C_CTL0_I2CEN           0u
#define I2C_FLAG_TR              0u
#define I2C_INT_FLAG_ADDSEND     0u
#define I2C_INT_FLAG_BERR        0u
#define I2C_INT_FLAG_NACK        0u
#define I2C_INT_FLAG_OUERR       0u
#define I2C_INT_FLAG_RBNE        0u
#define I2C_INT_FLAG_STPDET      0u
#define I2C_INT_FLAG_TI          0u
#define I2C_INT_FLAG_TIMEOUT     0u
#define I2C_STAT_BERR            0u
#define I2C_STAT_LOSTARB         0u
#define I2C_STAT_NACK            0u
#define I2C_STAT_OUERR           0u
#define I2C_STAT_PECERR          0u
#define I2C_STAT_SMBALT          0u
#define I2C_STAT_TBE             0u
#define I2C_STAT_TIMEOUT         0u
#define SPI1                     0u
#define SPI_CK_PL_LOW_PH_1EDGE   0u
#define SPI_ENDIAN_MSB           0u
#define SPI_FLAG_RBNE            0u
#define SPI_FRAMESIZE_8BIT       0u
#define SPI_NSS_HARD             0u
#define SPI_SLAVE                0u
#define SPI_TRANSMODE_FULLDUPLEX 0u

#endif
