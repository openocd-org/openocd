/***************************************************************************
 *   Copyright (C) 2015 by Ivan Meleca                                     *
 *   ivan@artekit.eu                                                       *
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

	/* Params:
	 * r0 = flash destination address, status
	 * r1 = longword count
	 * r2 = workarea start address
	 * r3 = workarea end address
	 */

	.text
	.cpu cortex-m0plus
	.code 16
	.thumb_func

	.align	2

	/* r5 = rp
	 * r6 = wp, tmp
	 * r7 = tmp
	 */

wait_fifo:
	ldr 	r6, [r2, #0]	/* read wp */
	cmp 	r6, #0			/* abort if wp == 0 */
	beq 	exit
	ldr 	r5, [r2, #4]	/* read rp */
	cmp 	r5, r6			/* wait until rp != wp */
	beq 	wait_fifo

	ldr		r6, fstat		/* Clear error flags */
	mov		r7, #48
	strb	r7, [r6]

	ldr		r6, fccobix		/* FCCOBIX = 0 */
	mov		r7, #0
	strb	r7, [r6]

	ldr 	r6, fccobhi		/* Program FLASH command */
	mov		r7, #6			/* FCCOBHI = 6 */
	strb	r7, [r6]

	lsr		r7, r0, #16		/* FCCOBLO = flash destination address >> 16 */
	ldr		r6, fccoblo
	strb	r7, [r6]

	ldr		r6, fccobix		/* Index for lower byte address bits[15:0] */
	mov		r7, #1
	strb	r7, [r6]		/* FCCOBIX = 1*/

	uxtb	r7, r0			/* Memory address bits[15:0] */
	ldr 	r6, fccoblo
	strb	r7, [r6]		/* FCCOBLO = flash destination address */

	lsr		r7, r0, #8
	ldr		r6, fccobhi
	strb	r7, [r6]		/* FCCOBHI = flash destination address >> 8 */

	ldr		r6, fccobix		/* FCCOBIX = 2 */
	mov		r7, #2
	strb	r7, [r6]

	ldrb	r7, [r5, #1]	/* FCCOBHI = rp >> 8 */
	ldr		r6, fccobhi
	strb	r7, [r6]

	ldrb	r7, [r5]		/* FCCOBLO = rp */
	ldr 	r6, fccoblo
	strb	r7, [r6]

	ldr		r6, fccobix		/* FCCOBIX = 3 */
	mov		r7, #3
	strb	r7, [r6]

	ldrb	r7, [r5, #3]	/* FCCOBHI = rp >> 24 */
	ldr		r6, fccobhi
	strb	r7, [r6]

	ldrb	r7, [r5, #2]	/* FCCOBLO = rp >> 16 */
	ldr		r6, fccoblo
	strb	r7, [r6]

	sub		r1, r1, #1		/* Two words (4 bytes) queued, decrement counter */
	add		r0, r0, #4		/* flash address += 4 */
	add		r5, r5, #4		/* rp += 4 */

	cmp     r5, r3			/* Wrap? */
	bcc     no_wrap
	mov     r5, r2
	add   	r5, r5, #8

no_wrap:
	cmp		r1, #0			/* Done? */
	beq		execute

	ldr 	r6, [r2, #0]	/* read wp */
	cmp 	r6, r5
	beq		execute			/* execute if rp == wp */

	ldr		r6, fccobix		/* FCCOBIX = 4 */
	mov		r7, #4
	strb	r7, [r6]

	ldrb	r7, [r5, #1]	/* FCCOBHI = rp >> 8 */
	ldr		r6, fccobhi
	strb	r7, [r6]

	ldrb	r7, [r5]		/* FCCOBLO = rp */
	ldr 	r6, fccoblo
	strb	r7, [r6]

	ldr		r6, fccobix		/* FCCOBIX = 5 */
	mov		r7, #5
	strb	r7, [r6]

	ldrb	r7, [r5, #3]	/* FCCOBHI = rp >> 24 */
	ldr		r6, fccobhi
	strb	r7, [r6]

	ldrb	r7, [r5, #2]	/* FCCOBLO = rp >> 16 */
	ldr		r6, fccoblo
	strb	r7, [r6]

	sub		r1, r1, #1		/* Two words (4 bytes) queued, decrement counter */
	add		r0, r0, #4		/* flash address += 4 */
	add		r5, r5, #4		/* rp += 4 */

	cmp     r5, r3			/* Wrap? */
	bcc     execute
	mov     r5, r2
	add   	r5, r5, #8

execute:
	ldr		r6, fstat		/* Launch the command */
	mov		r7, #128
	strb	r7, [r6]

wait_busy:
	ldr		r6, fstat
	ldrb	r6, [r6]		/* Wait until finished */
	tst		r6, r7
	beq		wait_busy

	mov		r7, #48			/* Check error */
	tst		r6, r7
	bne		error

	mov		r6, #0			/* Clear error */

	str     r5, [r2, #4]	/* Store rp */

	cmp		r1, #0			/* Done? */
	beq		done
	b		wait_fifo

error:
	mov		r0, #0
	str     r0, [r2, #4]    /* set rp = 0 on error */

done:
	mov		r0, r6			/* Set result code */
	bkpt    #0

	.align	2
fstat:
	.word	0
fccobix:
	.word	0
fccobhi:
	.word	0
fccoblo:
	.word	0
