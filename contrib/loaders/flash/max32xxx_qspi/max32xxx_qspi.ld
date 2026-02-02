/* SPDX-License-Identifier: GPL-2.0-or-later */

ENTRY(algo_entry)

SECTIONS
{
	. = 0x20005000;

	.text : {
		KEEP(*(.algo_entry))
		*(.text*)
		*(.rodata*)
		KEEP(*(.algo_exit))
	}

	/DISCARD/ : {
		*(.comment*)
		*(.note*)
		*(.eh_frame*)
	}
}
