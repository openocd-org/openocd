/* SPDX-License-Identifier: GPL-2.0-or-later */

#include "npcx_flash_config.h"

/* Application memory map */
MEMORY {
	/* buffer + parameters */
    BUFFER (RWX)  : ORIGIN = NPCX_FLASH_LOADER_PARAMS_ADDR,
                      LENGTH = NPCX_FLASH_LOADER_PARAMS_SIZE + NPCX_FLASH_LOADER_BUFFER_SIZE

    PROGRAM (RWX)  : ORIGIN = NPCX_FLASH_LOADER_PROGRAM_ADDR,
                     LENGTH = NPCX_FLASH_LOADER_PROGRAM_SIZE
}

/* Sections used for flashing */
SECTIONS
{
	.buffers (NOLOAD) :
	{
		_buffers = .;
		*(.buffers.g_cfg)
		*(.buffers.g_buf)
		*(.buffers*)
		_ebuffers = .;
	} > BUFFER

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
	} > PROGRAM

	.bss :
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
}
