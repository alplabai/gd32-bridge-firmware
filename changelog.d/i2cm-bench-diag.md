### Added

- Bench-only `CMD_I2CM_DIAG` (`0xAF`, I2C link only), compiled only with
  `-DBRIDGE_BENCH_DIAG=1` (default off, never in a release build): a 64-byte
  dump of the GPIOC, RCU and I2C2 registers behind the I2C3 proxy plus the
  pad state captured right before a `BUS_STUCK` decision.
