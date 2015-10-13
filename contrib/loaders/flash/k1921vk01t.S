/***************************************************************************
 *   Copyright (C) 2015 by Bogdan Kolbov                                   *
 *   kolbov@niiet.ru                                                       *
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
 *   Free Software Foundation, Inc.                                        *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m4
	.thumb
	.thumb_func

/* K1921VK01T has 128-bitwidth flash, so it`s able to load 4x32-bit words at the time.
 * And only after all words loaded we can start write
 */

/* Registers addresses */
#define FLASH_FMA	0x00		/* Address reg */
#define FLASH_FMD1	0x04		/* Data1 reg */
#define FLASH_FMC	0x08		/* Command reg */
#define FLASH_FCIS	0x0C		/* Operation Status reg */
#define FLASH_FCIC	0x14		/* Operation Status Clear reg */
#define FLASH_FMD2	0x50		/* Data2 reg */
#define FLASH_FMD3	0x54		/* Data3 reg */
#define FLASH_FMD4	0x58		/* Data4 reg*/

	/* Params:
	 * r0 - write cmd (in), status (out)
	 * r1 - count
	 * r2 - workarea start
	 * r3 - workarea end
	 * r4 - target address
	 * Clobbered:
	 * r5 - rp
	 * r6 - wp, tmp
	 * r7 - flash base
	 */

ldr     r7, =#0xA001C000  /* Flash reg base*/

wait_fifo:
	ldr		r6, [r2, #0]	/* read wp */
	cmp		r6, #0			/* abort if wp == 0 */
	beq		exit
	ldr		r5, [r2, #4]	/* read rp */
	cmp		r5, r6			/* wait until rp != wp */
	beq		wait_fifo


load_data:
	ldr r6, [r5]			/* read data1 */
	str r6, [r7, #FLASH_FMD1]
	adds	r5, #4

	ldr r6, [r5]			/* read data2 */
	str r6, [r7, #FLASH_FMD2]
	adds	r5, #4

	ldr r6, [r5]			/* read data3 */
	str r6, [r7, #FLASH_FMD3]
	adds	r5, #4

	ldr r6, [r5]			/* read data4 */
	str r6, [r7, #FLASH_FMD4]
	adds	r5, #4

start_write:
	str r4, [r7, #FLASH_FMA]		/* set addr */
	adds	r4, #16
	str r0, [r7, #FLASH_FMC]		/* write cmd */

busy:
	ldr		r6, [r7, #FLASH_FCIS]	/* wait until flag set */
	cmp		r6, #0x0
	beq		busy

	cmp		r6, #2			/* check the error bit */
	beq		error

	movs	r6, #1			/* clear flags */
	str r6, [r7, #FLASH_FCIC]

	cmp 	r5, r3			/* wrap rp at end of buffer */
	bcc	no_wrap
	mov	r5, r2
	adds	r5, #8
no_wrap:
	str 	r5, [r2, #4]	/* store rp */
	subs	r1, r1, #1		/* decrement 16-byte block count */
	cmp     r1, #0
	beq     exit		/* loop if not done */
	b	wait_fifo

error:
	movs	r0, #0
	str		r0, [r2, #4]	/* set rp = 0 on error */
exit:
	mov		r0, r6			/* return status in r0 */
	bkpt	#0
