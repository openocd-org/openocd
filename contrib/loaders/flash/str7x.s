/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

	.text
	.arm
	.arch armv4t

	.section .init
/*
	r0 source address
	r1 address
	r2 FLASH_CR0
	r3 dword count
	r4 result
	r5 busy mask
*/

write:
	mov		r4, #0x10000000			/* set DWPG bit */
	str		r4, [r2, #0x0]			/* FLASH_CR0 */
	str		r1, [r2, #0x10]			/* FLASH_AR */
	ldr		r4, [r0], #4			/* load data */
	str		r4, [r2, #0x8]			/* FLASH_DR0 */
	ldr		r4, [r0], #4			/* load data */
	str		r4, [r2, #0xc]			/* FLASH_DR1 */
	mov		r4, #0x90000000			/* set DWPG and WMS bits */
	str		r4, [r2, #0x0]			/* FLASH_CR0 */
busy:
	ldr		r4, [r2, #0x0]			/* FLASH_CR0 */
	tst		r4, r5
	bne		busy
	ldr		r4, [r2, #0x14]			/* FLASH_ER */
	tst		r4, #0xff				/* do we have errors */
	tsteq	r4, #0x100			/* write protection set */
	bne		exit
	add		r1, r1, #0x8			/* next 8 bytes */
	subs	r3, r3, #1				/* decrement dword count */
	bne		write
exit:
	b		exit

	.end
