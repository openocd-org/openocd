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
