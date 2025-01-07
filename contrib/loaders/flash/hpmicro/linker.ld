/* SPDX-License-Identifier: BSD-3-Clause */
/*
 * Copyright (c) 2021 HPMicro
 */
ENTRY(_init)

SECTIONS
{
	.text : {
		*(.func_table)
		KEEP(*(.flash_algo.text*))
		KEEP(*(.rodata))
		KEEP(*(.rodata*))
		KEEP(*(.flash_algo.data*))
		*(.text)
		*(.text*)
		__etext = .;
	}
	.discard : {
		__noncacheable_start__ = .;
		__noncacheable_bss_start__ = .;
		__bss_start__ = .;
		__bss_end__ = .;
		__noncacheable_bss_end__ = .;
		_end = .;
		__noncacheable_init_start__ = .;
		__data_start__ = .;
		__data_end__ = .;
		__noncacheable_init_end__ = .;
		__noncacheable_end__ = .;
		__heap_start__ = .;
		__heap_end__ = .;
		__ramfunc_start__ = .;
		__ramfunc_end__ = .;
		__noncacheable_bss_start__ = .;
		__noncacheable_bss_end__ = .;
		__noncacheable_init_start__ = .;
		__noncacheable_init_end__ = .;
		__tdata_start__ = .;
		__tdata_end__ = .;
		__tbss_start__ = .;
		__tbss_end__ = .;
		__data_load_addr__ = .;
		__fast_load_addr__ = .;
		__tdata_load_addr__ = .;
		__noncacheable_init_load_addr__ = .;
	}
}
