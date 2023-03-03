/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005, 2007 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *   Copyright (C) 2010 Spencer Oliver                                     *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

	.text
	.arm
	.arch armv4

	.section .init

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
	ldr		r5, [r0], #4
	str		r9, [r8]
	str		r11, [r10]
	str		r3, [r8]
	str		r5, [r1]
	nop
busy:
	ldr		r6, [r1]
	eor		r7, r5, r6
	ands	r7, r4, r7
	beq		cont			/* b if DQ7 == Data7 */
	ands	r6, r6, r4, lsr #2
	beq		busy			/* b if DQ5 low */
	ldr		r6, [r1]
	eor		r7, r5, r6
	ands	r7, r4, r7
	beq		cont			/* b if DQ7 == Data7 */
	mov		r5, #0			/* 0x0 - return 0x00, error */
	bne		done
cont:
	subs	r2, r2, #1		/* 0x1 */
	moveq	r5, #128		/* 0x80 */
	beq		done
	add		r1, r1, #4		/* 0x4 */
	b		code
done:
	b		done

	.end
