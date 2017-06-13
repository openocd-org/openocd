/***************************************************************************
 *   Copyright (C) 2015 by Ivan Meleca                                     *
 *   ivan@artekit.eu                                                       *
 *                                                                         *
 *   Copyright (C) 2016 by Tomas Vanek                                     *
 *   vanekt@fbl.cz                                                         *
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
	 * r0 = flash destination address in/out
	 * r1 = longword count
	 * r2 = workarea start address
	 * r3 = workarea end address
	 * r4 = FTFx base
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

	/* old longword algo: 6.680 KiB/s @ adapter_khz 2000
	 * this async algo: 19.808 KiB/s @ adapter_khz 2000
	 */

FTFx_FSTAT =	0
FTFx_FCCOB3 =	4
FTFx_FCCOB0 =	7
FTFx_FCCOB7 =	8

wait_fifo:
	ldr 	r6, [r2, #0]	/* read wp */
	cmp 	r6, #0		/* abort if wp == 0 */
	beq 	exit

	ldr 	r5, [r2, #4]	/* read rp */
	cmp 	r5, r6		/* wait until rp != wp */
	beq 	wait_fifo

	str	r0, [r4, #FTFx_FCCOB3] /* set flash address */
	mov	r7, #6
	strb	r7, [r4, #FTFx_FCCOB0] /* flash command */

	ldr	r7, [r5]	/* set longword data = *rp */
	str	r7, [r4, #FTFx_FCCOB7]

	mov	r7, #128
	strb	r7, [r4, #FTFx_FSTAT]

	add	r5, #4		/* rp += 4 */
	cmp     r5, r3		/* Wrap? */
	bcc     no_wrap
	mov     r5, r2
	add   	r5, #8

no_wrap:
	str     r5, [r2, #4]	/* Store rp */

wait_ccif:
	ldr     r6, [r2, #0]    /* read wp */
	cmp     r6, #0          /* abort if wp == 0 */
	beq     exit

	ldrb	r6, [r4, #FTFx_FSTAT]
	tst	r6, r7
	beq	wait_ccif

	mov	r7, #0x70
	tst	r6, r7
	bne	error

	add	r0, #4		/* flash address += 4, do not increment before err check */

	sub	r1, #1		/* word_count-- */
	cmp	r1, #0
	bne	wait_fifo
	b	exit

error:
	mov	r5, #0
	str     r5, [r2, #4]    /* set rp = 0 on error */

exit:
	bkpt    #0
