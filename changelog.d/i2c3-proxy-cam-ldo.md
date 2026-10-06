### Added

- Wire protocol 0.17: the E1M-X I2C3 master proxy, `CMD_I2CM_CONFIG` /
  `CMD_I2CM_XFER` / `CMD_I2CM_RESULT` (`0xA0`..`0xA2`, I2C link only), on the
  GD32 I2C2 peripheral, `PC8` / `PC9` (AF8).
  The transfer runs at base level from `bridge_hw_tick()`; `POWER_MODE_SET`
  answers `STATUS_BUSY` while a job is queued or running. Host tests in
  `tests/unit/i2cm/`.
- GPIO pad bits 23..26: `CAM_EN_LDO0..3` (`PC3`, `PE8`, `PE7`, `PE10`), output-only,
  boot OUTPUT LOW (camera LDOs off). Hosts relying on them must require
  `PROTOCOL_VERSION_MINOR >= 17`.

### Changed

- `bridge_i2c_timing_derive()` (formerly the file-static `i2c_timing_derive()`)
  is shared by the I2C0 slave and the I2C3 master.
- The E1M-X I2C3 proxy uses the GD32 I2C2 peripheral (`PC8` AF8 = I2C2_SCL,
  `PC9` AF8 = I2C2_SDA; GD32G553xx Datasheet Rev1.5 pin alternate-function
  table), so `BRIDGE_I2CM_GPIO_AF` defaults to `GPIO_AF_8`. Not yet run on silicon.
- SCL high/low times are sized so `fSCL` never exceeds 100/400 kHz (tLOW/tHIGH
  4700/5300 ns and 1400/1100 ns), at or above the I2C spec minimums.
- `bridge_hw_i2cm_tick()` runs first in `bridge_hw_tick()`.
- `CMD_PWM_SET` / `CMD_PWM_GET` (`0x20`/`0x21`) are now allowed on the I2C link
  (Linux PWM provider); the SPI path is unchanged.
- Wire protocol 0.17 must not ship before alp-sdk raises its minimum protocol
  gates to 17u (paired alp-sdk change).
