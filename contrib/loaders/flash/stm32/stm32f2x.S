/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 Øyvind Harboe                                      *
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
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m3
	.thumb

/*
 * Params :
 * r0 = workarea start, status (out)
 * r1 = workarea end
 * r2 = target address
 * r3 = count (16bit words)
 * r4 = flash base
 *
 * Clobbered:
 * r6 - temp
 * r7 - rp
 * r8 - wp, tmp
 */

#define STM32_FLASH_CR_OFFSET	0x10			/* offset of CR register in FLASH struct */
#define STM32_FLASH_SR_OFFSET	0x0c			/* offset of SR register in FLASH struct */

#define STM32_PROG16		0x101			/* PG | PSIZE_16*/

	.thumb_func
	.global	_start
_start:
wait_fifo:
	ldr 	r8, [r0, #0]	/* read wp */
	cmp 	r8, #0			/* abort if wp == 0 */
	beq 	exit
	ldr 	r7, [r0, #4]	/* read rp */
	cmp 	r7, r8			/* wait until rp != wp */
	beq 	wait_fifo

	ldr		r6, =STM32_PROG16
	str		r6, [r4, #STM32_FLASH_CR_OFFSET]
	ldrh 	r6, [r7], #0x02						/* read one half-word from src, increment ptr */
	strh 	r6, [r2], #0x02						/* write one half-word from src, increment ptr */
	dsb
busy:
	ldr 	r6, [r4, #STM32_FLASH_SR_OFFSET]
	tst 	r6, #0x10000						/* BSY (bit16) == 1 => operation in progress */
	bne 	busy								/* wait more... */
	tst		r6, #0xf0							/* PGSERR | PGPERR | PGAERR | WRPERR */
	bne		error								/* fail... */

	cmp 	r7, r1			/* wrap rp at end of buffer */
	it  	cs
	addcs	r7, r0, #8		/* skip loader args */
	str 	r7, [r0, #4]	/* store rp */
	subs	r3, r3, #1		/* decrement halfword count */
	cbz 	r3, exit		/* loop if not done */
	b		wait_fifo
error:
	movs	r1, #0
	str		r1, [r0, #4]	/* set rp = 0 on error */
exit:
	mov		r0, r6			/* return status in r0 */
	bkpt	#0x00

	.pool
