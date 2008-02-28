/***************************************************************************
 *   Copyright (C) 2008 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
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
