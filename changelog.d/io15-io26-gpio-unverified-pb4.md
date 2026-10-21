### Changed

- PB4 (GPIO pad bit 21, E1M IO15) reset state (JTAG NJTRST, AF + pull-up) and the
  SWD-only (PA13/PA14) claim are marked unverified in `hal/gd32/init.c`,
  `hal/gd32/gpio.c` and `README.md` pending a UM/datasheet citation or a bench
  read of GPIOB CTL/PUD.
- Wire protocol 0.16 (GPIO pads 21/22) must not ship before alp-sdk raises
  `GD32G553_IO15_IO26_MIN_PROTOCOL_MINOR` to 16u (paired alp-sdk change).
