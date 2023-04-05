/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005, 2007 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *   Copyright (C) 2010 Spencer Oliver                                     *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

	.text
	.syntax unified
	.arch armv7-m
	.thumb
	.thumb_func

	.align 2

/* input parameters - */
/*	R0 = source address */
/*	R1 = destination address */
/*	R2 = number of writes */
/*	R3 = flash write command */
/*	R4 = constant to mask DQ7 bits (also used for Dq5 with shift) */
/* output parameters - */
/*	R5 = 0x80 ok 0x00 bad */
/* temp registers - */
/*	R6 = value read from flash to test status */
/*	R7 = holding register */
/* unlock registers - */
/*  R8 = unlock1_addr */
/*  R9 = unlock1_cmd */
/*  R10 = unlock2_addr */
/*  R11 = unlock2_cmd */

code:
	ldrh	r5, [r0], #2
	strh	r9, [r8]
	strh	r11, [r10]
	strh	r3, [r8]
	strh	r5, [r1]
	nop
busy:
	ldrh	r6, [r1]
	eor		r7, r5, r6
	ands	r7, r4, r7
	beq		cont			/* b if DQ7 == Data7 */
	ands	r6, r6, r4, lsr #2
	beq		busy			/* b if DQ5 low */
	ldrh	r6, [r1]
	eor		r7, r5, r6
	ands	r7, r4, r7
	beq		cont			/* b if DQ7 == Data7 */
	mov		r5, #0			/* 0x0 - return 0x00, error */
	bne		done
cont:
	subs	r2, r2, #1		/* 0x1 */
	beq 	success
	add		r1, r1, #2		/* 0x2 */
	b		code

success:
	mov 	r5, #128		/* 0x80 */
	b 	done

done:
	bkpt #0

	.end
