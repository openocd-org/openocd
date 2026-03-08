/* SPDX-License-Identifier: GPL-2.0-or-later */

/******************************************************************************
 *
 * Copyright (C) 2026 Texas Instruments Incorporated - https://www.ti.com/
 *
 * Linker script for AM13E230X flash loader algorithm.
 * Everything runs from SRAM. Params and buffers are at fixed addresses
 * that the OpenOCD driver knows about (see flash.h).
 *
 *****************************************************************************/

ENTRY( entry )

MEMORY
{
	/* Code, data, bss, stack */
	PROGRAM (RWX) : ORIGIN = 0x20000000, LENGTH = 0x2000
	/* Params and data buffers at known addresses */
	BUFFERS (RWX) : ORIGIN = 0x20002000, LENGTH = 0x1100
}

SECTIONS
{
	.text :
	{
		_text = .;
		*(.entry*)
		*(.text*)
		_etext = .;
	} > PROGRAM

	.data :
	{
		_data = .;
		*(.rodata*)
		*(.data*)
		_edata = .;
	} > PROGRAM

	.bss (NOLOAD) :
	{
		__bss_start__ = .;
		_bss = .;
		*(.bss*)
		*(COMMON)
		_ebss = .;
		__bss_end__ = .;
	} > PROGRAM

	.stack (NOLOAD) :
	{
		_stack = .;
		*(.stack*)
		_estack = .;
	} > PROGRAM

	.buffers (NOLOAD) :
	{
		_buffers = .;
		*(.buffers.params)
		. = 0x100;
		*(.buffers.buf0)
		*(.buffers.buf1)
		_ebuffers = .;
	} > BUFFERS
}
