/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

	.text
	.syntax unified
	.cpu cortex-m3
	.thumb
	.thumb_func
	.align	2

/*
	Call with :	
	r0 = buffer address
	r1 = destination address
	r2 = bytecount (in) - endaddr (work) 
	
	Used registers:	
	r3 = pFLASH_CTRL_BASE
	r4 = FLASHWRITECMD
	r5 = #1
	r6 = bytes written
	r7 = temp reg
*/

write:
	ldr 	r3,pFLASH_CTRL_BASE
	ldr 	r4,FLASHWRITECMD
	movs 	r5, 1
	movs 	r6, #0
mainloop:
	str		r1, [r3, #0]
	ldr		r7, [r0, r6]
	str		r7, [r3, #4]
	str		r4, [r3, #8]
waitloop:
	ldr		r7, [r3, #8]
	tst		r7, r5
	bne		waitloop
	adds	r1, r1, #4
	adds	r6, r6, #4
	cmp		r6, r2
	bne		mainloop
	bkpt 	#0

pFLASH_CTRL_BASE: .word 0x400FD000
FLASHWRITECMD: .word 0xA4420001
