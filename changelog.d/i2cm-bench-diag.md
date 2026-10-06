### Added

- Bench-only `CMD_I2CM_DIAG` (`0xAF`, I2C link only), compiled only with
  `-DBRIDGE_BENCH_DIAG=1` (default off, never in a release build): a 64-byte
  dump of the GPIOC, RCU and I2C2 registers behind the I2C3 proxy plus the
  pad state captured right before a `BUS_STUCK` decision.
- Bench-only `CMD_I2CM_PADTEST` (`0xAE`, I2C link only, same flag): request
  `mode, pc8, pc9` (0 release to I2CM AF/OD, 1 push-pull outputs, 2 inputs),
  20-byte reply of GPIOC `CTL, OMODE, PUD, ISTAT, OCTL` (u32 LE).
