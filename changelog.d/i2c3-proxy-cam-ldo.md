### Added

- Wire protocol 0.17: the I2C3 master proxy, `CMD_I2CM_CONFIG` / `CMD_I2CM_XFER`
  / `CMD_I2CM_RESULT` (`0xA0`..`0xA2`, I2C link only) on GD32 `PC8` / `PC9`.
  The transfer runs at base level from `bridge_hw_tick()`; `POWER_MODE_SET`
  answers `STATUS_BUSY` while a job is queued or running. Host tests in
  `tests/unit/i2cm/`.
- GPIO pad bits 23..26: `CAM_EN_LDO0..3` (`PC3`, `PE8`, `PE7`, `PE10`), output-only,
  boot OUTPUT LOW (camera LDOs off). Hosts relying on them must require
  `PROTOCOL_VERSION_MINOR >= 17`.

### Changed

- `bridge_i2c_timing_derive()` (formerly the file-static `i2c_timing_derive()`)
  is shared by the I2C0 slave and the I2C3 master.

### Known gaps

- The I2C3 alternate-function number for `PC8` / `PC9` is unverified (no
  GD32G553 datasheet at hand): `BRIDGE_I2CM_GPIO_AF` must be defined by the
  build, otherwise `CMD_I2CM_CONFIG` answers `STATUS_NOSUPPORT`. Not run on silicon.
- Wire protocol 0.17 must not ship before alp-sdk raises its minimum protocol
  gates to 17u (paired alp-sdk change).
