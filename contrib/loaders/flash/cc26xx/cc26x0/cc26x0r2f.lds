/******************************************************************************
*
* Copyright (C) 2018 Texas Instruments Incorporated - http://www.ti.com/
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*  Redistributions of source code must retain the above copyright
*  notice, this list of conditions and the following disclaimer.
*
*  Redistributions in binary form must reproduce the above copyright
*  notice, this list of conditions and the following disclaimer in the
*  documentation and/or other materials provided with the
*  distribution.
*
*  Neither the name of Texas Instruments Incorporated nor the names of
*  its contributors may be used to endorse or promote products derived
*  from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
