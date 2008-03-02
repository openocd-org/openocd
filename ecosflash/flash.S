/*
#####ECOSGPLCOPYRIGHTBEGIN####
## -------------------------------------------
## This file is part of eCos, the Embedded Configurable Operating System.
## Copyright (C) 2008 Øyvind Harboe
##
## eCos is free software; you can redistribute it and/or modify it under
## the terms of the GNU General Public License as published by the Free
## Software Foundation; either version 2 or (at your option) any later version.
##
## eCos is distributed in the hope that it will be useful, but WITHOUT ANY
## WARRANTY; without even the implied warranty of MERCHANTABILITY or
## FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
## for more details.
##
## You should have received a copy of the GNU General Public License along
## with eCos; if not, write to the Free Software Foundation, Inc.,
## 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
##
## As a special exception, if other files instantiate templates or use macros
## or inline functions from this file, or you compile this file and link it
## with other works to produce a work based on this file, this file does not
## by itself cause the resulting work to be covered by the GNU General Public
## License. However the source code for this file must still be made available
## in accordance with section (3) of the GNU General Public License.
##
## This exception does not invalidate any other reasons why a work based on
## this file might be covered by the GNU General Public License.
## -------------------------------------------
#####ECOSGPLCOPYRIGHTEND####
*/

/*
	Jump table for flash driver
	
	Registers in ARM callling convention is to place args in registers 
	starting at r0.
	
	So for:
	
	void foo(int a, int b, int c).
	
	a=r0
	b=r1
	c=r2
	
	
*/
	.global _stack_base
	.global _stack_start
	.global _workarea
	.global _start
	.global _start_bss_clear
_start:
	// offset=0
	// int erase(void *address, int len)
	ldr     sp,=_stack_start                
	bl erase
	nop // Stop CPU here using hw breakpoint
	
	// offset=0xc
	// int program(void *buffer, void *address, int len)	
	ldr     sp,=_stack_start              
	bl program
	nop // Stop CPU here using hw breakpoint
	
	// offset=0x18
	ldr     r0,=_workarea                
	nop // Stop CPU here using hw breakpoint
	
	// offset=0x20
	// int init() - returns error message if the flash chip can't be detected	
	ldr     sp,=_stack_start              
	bl init
	nop // Stop CPU here using hw breakpoint

    .section ".bss"
    .balign 4
_stack_base:
        .rept 4096
        .byte 0
        .endr
_stack_start:
    .balign 4
_workarea:
        .rept 8192
        .byte 0
        .endr
	// NB!!! we clear bss while the stack is in use, so we start BSS clearing here !!! :-)
_start_bss_clear:
