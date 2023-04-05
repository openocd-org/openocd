/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

	.text
	.arm
	.arch armv5t

	.section .init
/*
	r0 source address (in)
	r1 target address (in)
	r2 word count (in)
	r3 result (out)
*/

write:
	bic		r4, r1, #3			/* word address */
	mov		r3, #0x40			/* write command */
	strh	r3, [r4, #0]
	ldrh 	r3, [r0], #2		/* read data */
	strh	r3, [r1], #2		/* write data */
	mov		r3, #0x70			/* status command */
	strh	r3, [r4, #0]
busy:
	ldrb	r3, [r4, #0]		/* status */
	tst 	r3, #0x80
	beq 	busy
	mov		r5, #0x50			/* clear status command */
	strh	r5, [r4, #0]
	mov		r5, #0xFF			/* read array */
	strh	r5, [r4, #0]
	tst		r3, #0x12
	bne		exit
	subs 	r2, r2, #1			/* decrement word count */
	bne 	write
exit:
	bkpt	#0

	.end
