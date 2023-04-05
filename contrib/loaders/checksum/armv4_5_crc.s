/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
