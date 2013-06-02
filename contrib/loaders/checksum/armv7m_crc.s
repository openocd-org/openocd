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
	parameters:
	r0 - address in - crc out
	r1 - char count
*/

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb
	.thumb_func

	.align	2

_start:
main:
	mov		r2, r0
	movs	r0, #0
	mvns	r0, r0
	ldr		r6, CRC32XOR
	mov		r3, r1
	movs	r4, #0
	b		ncomp
nbyte:
	ldrb	r1, [r2, r4]
	lsls	r1, r1, #24
	eors	r0, r0, r1
	movs	r5, #0
loop:
	cmp		r0, #0
	bge		notset
	lsls	r0, r0, #1
	eors	r0, r0, r6
	b		cont
notset:
	lsls	r0, r0, #1
cont:
	adds	r5, r5, #1
	cmp		r5, #8
	bne		loop
	adds	r4, r4, #1
ncomp:
	cmp		r4, r3
	bne		nbyte
	bkpt	#0

	.align	2

CRC32XOR:	.word	0x04c11db7

	.end
