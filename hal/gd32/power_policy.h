/*
 * Copyright 2026 Alp Lab AB
 * SPDX-License-Identifier: Apache-2.0
 *
 * Vendor-free low-power policy: POWER_MODE_SET flags, the "what blocks a
 * Deep-sleep / Standby entry" predicate, wake-source classification and the
 * SWD-readable diagnostics block.  Kept free of vendor headers so the exact
 * decisions used by power.c run in host CI.
 */
#ifndef GD32_BRIDGE_POWER_POLICY_H
#define GD32_BRIDGE_POWER_POLICY_H

#include <stdbool.h>
#include <stdint.h>

/* POWER_MODE_SET request byte 1 (reserved/0 on every pre-flags host, so a
 * 0.14 host is unaffected).  Unknown bits are refused (STATUS_INVAL). */
#define POWER_FLAG_WAKE_I2C 0x01u /* BRD_I2C address match ends Deep-sleep */
#define POWER_FLAGS_SUPPORTED (POWER_FLAG_WAKE_I2C)

/* Everything that must keep running, or whose state a Deep-sleep / Standby
 * would silently corrupt.  SLEEP (mode 1) keeps every clock and is never
 * refused on these grounds. */
typedef struct {
	bool adc_stream; /* an ADC stream (STREAM_BEGIN/BEGIN2) is paced */
	bool pwm;        /* a continuous PWM output or PWM input-capture is claimed */
	bool dac;        /* a DAC output has been driven by the host */
	bool ota;        /* an OTA session is open (BEGIN..COMMIT) or an erase runs */
	bool boot_commit; /* a trial image / boot-config commit is unconfirmed */
} power_activity_t;

static inline bool power_entry_blocked(uint8_t mode, const power_activity_t *a)
{
	if (mode != 2u && mode != 3u) return false;
	return a->adc_stream || a->pwm || a->dac || a->ota || a->boot_commit;
}

/* Last wake source (bridge_power_diag.last_wake_source). */
enum {
	POWER_WAKE_SRC_NONE   = 0,
	POWER_WAKE_SRC_SPI_CS = 1, /* EXTI 8 (PA8) */
	POWER_WAKE_SRC_I2C    = 2, /* EXTI 31 (I2C0 address match) */
	POWER_WAKE_SRC_RTC    = 3, /* EXTI 19 (RTC wakeup timer) */
	POWER_WAKE_SRC_OTHER  = 4
};

/* EXTI_PD0 sampled straight after the WFI returns.  Host traffic outranks
 * the timer when both are pending. */
static inline uint8_t power_wake_source_from_pd0(uint32_t pd0)
{
	if ((pd0 & (1u << 8)) != 0u) return POWER_WAKE_SRC_SPI_CS;
	if ((pd0 & (1u << 31)) != 0u) return POWER_WAKE_SRC_I2C;
	if ((pd0 & (1u << 19)) != 0u) return POWER_WAKE_SRC_RTC;
	return (pd0 != 0u) ? POWER_WAKE_SRC_OTHER : POWER_WAKE_SRC_NONE;
}

/* SWD-readable counters (symbol bridge_power_diag).  Index = POWER_MODE_SET
 * mode 0..3.  SRAM, so a Standby wake (a reset) zeroes it; the reset cause
 * is in bridge_reset_reason.  SLEEP is WFI between interrupts, so only its
 * entries are countable. */
typedef struct {
	uint32_t entries[4];            /* requests accepted / executed per mode */
	uint32_t wakes[4];              /* completed wakes per mode (2 only) */
	uint32_t refused_busy;          /* STATUS_BUSY: activity blocked the entry */
	uint32_t refused_late;          /* latched request dropped, activity started after the reply */
	uint32_t last_wake_pd0;         /* raw EXTI_PD0 at the last wake */
	uint32_t last_wake_restore_cyc; /* DWT cycles, WFI return -> clock/I2C restored */
	uint8_t  last_mode;
	uint8_t  last_wake_source; /* POWER_WAKE_SRC_* */
	uint8_t  i2c_wake_armed;   /* the last Deep-sleep armed WUEN */
	uint8_t  reserved;
} bridge_power_diag_t;

extern volatile bridge_power_diag_t bridge_power_diag;

#endif /* GD32_BRIDGE_POWER_POLICY_H */
