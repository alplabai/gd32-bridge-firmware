/*
 * SPDX-License-Identifier: Apache-2.0
 * Throwaway linker script for the mcuboot-size measurement. NOT production.
 *
 * FLASH/RAM here are deliberately oversized (1 MB / 256 KB): this script's
 * job is to let every configuration link so arm-none-eabi-size can report a
 * real number, not to enforce the 32 KB bootloader budget -- the workflow
 * compares the reported text+data against 32768 bytes itself, after the
 * fact. This is NOT toolchain/gd32g553_bootloader.ld and must not be
 * confused with it.
 */
MEMORY
{
	FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1M
	RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 256K
}

ENTRY(Reset_Handler)

SECTIONS
{
	.isr_vector :
	{
		. = ALIGN(4);
		KEEP(*(.isr_vector))
		. = ALIGN(4);
	} > FLASH

	.text :
	{
		. = ALIGN(4);
		*(.text)
		*(.text*)
		*(.rodata)
		*(.rodata*)
		. = ALIGN(4);
		_etext = .;
	} > FLASH

	_sidata = LOADADDR(.data);
	.data :
	{
		. = ALIGN(4);
		_sdata = .;
		*(.data)
		*(.data*)
		. = ALIGN(4);
		_edata = .;
	} > RAM AT > FLASH

	.bss :
	{
		. = ALIGN(4);
		_sbss = .;
		*(.bss)
		*(.bss*)
		*(COMMON)
		. = ALIGN(4);
		_ebss = .;
		PROVIDE(end = .); /* libnosys.a's _sbrk() wants this (CMSIS convention). */
	} > RAM

	.heap_stack (NOLOAD) :
	{
		. = ALIGN(8);
		. = . + 4K;
	} > RAM
}
