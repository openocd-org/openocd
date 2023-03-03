/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
