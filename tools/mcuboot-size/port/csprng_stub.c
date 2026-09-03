/*
 * SPDX-License-Identifier: Apache-2.0
 * Throwaway port glue for the mcuboot-size measurement. NOT production.
 *
 * tinycrypt's ecc.c links against default_CSPRNG() regardless of whether
 * the build path ever calls uECC_sign() -- verify-only (which is all a
 * bootloader does) never touches the RNG. Every non-Zephyr MCUboot port
 * that uses tinycrypt for EC256 (e.g. boot/mbed/mcuboot_main.cpp) stubs
 * this the same way for link purposes only. A real signing tool needs a
 * real CSPRNG; this bootloader never signs.
 */
#include <stdint.h>

int default_CSPRNG(uint8_t *dest, unsigned int size);

int default_CSPRNG(uint8_t *dest, unsigned int size)
{
	(void)dest;
	(void)size;
	return 0;
}
