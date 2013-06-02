/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

/*
	r0 - address in - crc out
	r1 - char count
*/

	.text
	.arm

_start:
main:
	mov		r2, r0
	mov		r0, #0xffffffff	/* crc */
	mov		r3, r1
	mov		r4, #0
	b		ncomp
nbyte:
	ldrb	r1, [r2, r4]
	ldr		r7, CRC32XOR
	eor		r0, r0, r1, asl #24
	mov		r5, #0
loop:
	cmp		r0, #0
	mov		r6, r0, asl #1
	add		r5, r5, #1
	mov		r0, r6
	eorlt	r0, r6, r7
	cmp		r5, #8
	bne		loop
	add		r4, r4, #1
ncomp:
	cmp		r4, r3
	bne		nbyte
end:
	bkpt	#0

CRC32XOR:	.word	0x04c11db7

	.end
