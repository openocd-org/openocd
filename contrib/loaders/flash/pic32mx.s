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

	.text
	.arch m4k
	.set noreorder
	.set noat

/* params:
 * $a0 src adr - ram + result
 * $a1 dest adr - flash
 * $a2 count (32bit words)
 * vars
 *
 * temps:
 * $t0, $t1, $t2, $t3, $t4, $t5
 * $s0, $s1, $s3, $s4, $s5
 */

	.type main, @function
	.global main

.ent main
main:
	/* setup constants */
	lui		$t0, 0xaa99
	ori		$t0, 0x6655				/* NVMKEY1 */
	lui		$t1, 0x5566
	ori		$t1, 0x99AA				/* NVMKEY2 */
	lui		$t2, 0xBF80
	ori		$t2, 0xF400				/* NVMCON */
	ori		$t3, $zero, 0x4003		/* NVMCON row write cmd */
	ori		$t4, $zero, 0x8000		/* NVMCON start cmd */

write_row:
	/* can we perform a row write: 128 32bit words */
	sltiu	$s3, $a2, 128
	bne		$s3, $zero, write_word
	ori		$t5, $zero, 0x4000		/* NVMCON clear cmd */

	/* perform row write 512 bytes */
	sw		$a1, 32($t2)	/* set NVMADDR with dest addr - real addr */
	sw		$a0, 64($t2)	/* set NVMSRCADDR with src addr - real addr */

	bal		progflash
	addiu	$a0, $a0, 512
	addiu	$a1, $a1, 512
	beq		$zero, $zero, write_row
	addiu	$a2, $a2, -128

write_word:
	/* write 32bit words */
	lui		$s5, 0xa000
	ori		$s5, 0x0000
	or		$a0, $a0, $s5			/* convert to virtual addr */

	beq		$zero, $zero, next_word
	ori		$t3, $zero, 0x4001		/* NVMCON word write cmd */

prog_word:
	lw		$s4, 0($a0)		/* load data - from virtual addr */
	sw		$s4, 48($t2)	/* set NVMDATA with data */
	sw		$a1, 32($t2)	/* set NVMADDR with dest addr - real addr */

	bal		progflash
	addiu	$a0, $a0, 4
	addiu	$a1, $a1, 4
	addiu	$a2, $a2, -1
next_word:
	bne		$a2, $zero, prog_word
	nop

done:
	beq		$zero, $zero, exit
	addiu	$a0, $zero, 0

error:
	/* save result to $a0 */
	addiu	$a0, $s1, 0

exit:
	sdbbp
.end main

	.type progflash, @function
	.global progflash

.ent progflash
progflash:
	sw		$t3, 0($t2)		/* set NVMWREN */
	sw		$t0, 16($t2)	/* write NVMKEY1 */
	sw		$t1, 16($t2)	/* write NVMKEY2 */
	sw		$t4, 8($t2)		/* start operation */

waitflash:
	lw		$s0, 0($t2)
	and		$s0, $s0, $t4
	bne		$s0, $zero, waitflash
	nop

	/* following is to comply with errata #34
	 * 500ns delay required */
	nop
	nop
	nop
	nop
	/* check for errors */
	lw		$s1, 0($t2)
	andi	$s1, $zero, 0x3000
	bne		$s1, $zero, error
	sw		$t5, 4($t2)		/* clear NVMWREN */
	jr		$ra
	nop

.end progflash
