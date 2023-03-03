/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

	.global main
	.text
	.set noreorder

/* params:
 * $a0 address in
 * $a1 byte count
 * vars
 * $a0 crc
 * $a1 crc data byte
 * temps:
 * t3 v0 a3 a2 t0 v1
 */

.ent main
main:
	addiu	$t4, $a0, 0		/* address in */
	addiu	$t2, $a1, 0		/* count */

	addiu	$a0, $zero, 0xffffffff /* a0 crc - result */

	beq		$zero, $zero, ncomp
	addiu	$t3, $zero, 0	/* clear bytes read */

nbyte:
	lb		$a1, ($t4)		/* load byte from source address */
	addi	$t4, $t4, 1		/* inc byte count */

crc:
	sll		$a1, $a1, 24
	lui		$v0, 0x04c1
	xor		$a0, $a0, $a1
	ori		$a3, $v0, 0x1db7
	addu	$a2, $zero, $zero /* clear bit count */
loop:
	sll		$t0, $a0, 1
	addiu	$a2, $a2, 1		/* inc bit count */
	slti	$a0, $a0, 0
	xor		$t1, $t0, $a3
	movn	$t0, $t1, $a0
	slti	$v1, $a2, 8		/* 8bits processed */
	bne		$v1, $zero, loop
	addu	$a0, $t0, $zero

ncomp:
	bne		$t2, $t3, nbyte	/* all bytes processed */
	addiu	$t3, $t3, 1

wait:
	sdbbp

.end main
