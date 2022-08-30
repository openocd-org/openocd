/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2013 by Henrik Nilsson                                  *
 *   henrik.nilsson@bytequest.se                                           *
 ***************************************************************************/

	.text
	.syntax unified
	.arch armv7-m
	.thumb
	.thumb_func

	.align 4

/* Inputs:
 *  r0	buffer address
 *  r1	NAND data address (byte wide)
 *  r2	buffer length
 */
read:
	ldrb	r3, [r1]
	strb	r3, [r0], #1
	subs	r2, r2, #1
	bne		read

done_read:
	bkpt #0

	.align 4

/* Inputs:
 *  r0	NAND data address (byte wide)
 *  r1	buffer address
 *  r2	buffer length
 */
write:
	ldrb	r3, [r1], #1
	strb	r3, [r0]
	subs	r2, r2, #1
	bne		write

done_write:
	bkpt #0

	.end
