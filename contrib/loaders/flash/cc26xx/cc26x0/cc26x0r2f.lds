/* SPDX-License-Identifier: BSD-3-Clause */

/******************************************************************************
*
* Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
*
******************************************************************************/

/* Entry Point */
ENTRY( entry )

/* System memory map */
MEMORY
{
	/* Application is stored in and executes from SRAM */
	PROGRAM (RWX) : ORIGIN = 0x20000000, LENGTH = 0x1BD8
	BUFFERS (RWX) : ORIGIN = 0x20001BD8, LENGTH = 0x3028
}

/* Section allocation in memory */
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
	{	_data = .;
		*(.rodata*)
		*(.data*)
		_edata = .;
	}

	.bss :
	{
		__bss_start__ = .;
		_bss = .;
		*(.bss*)
		*(COMMON)
		_ebss = .;
		__bss_end__ = .;
	} > PROGRAM

	.stack :
	{
		_stack = .;
		*(.stack*)
		_estack = .;
	} > PROGRAM

	.buffers :
	{
		_buffers = .;
		*(.buffers.g_cfg)
		*(.buffers.g_buf1)
		*(.buffers.g_buf2)
		*(.buffers*)
		_ebuffers = .;
	} > BUFFERS
}
