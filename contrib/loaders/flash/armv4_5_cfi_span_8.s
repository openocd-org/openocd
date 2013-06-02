/***************************************************************************
 *   Copyright (C) 2005, 2007 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *   Copyright (C) 2010 Spencer Oliver                                     *
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
	ldrb	r5, [r0], #1
	strb	r9, [r8]
	strb	r11, [r10]
	strb	r3, [r8]
	strb	r5, [r1]
	nop
busy:
	ldrb	r6, [r1]
	eor		r7, r5, r6
	ands	r7, r4, r7
	beq		cont			/* b if DQ7 == Data7 */
	ands	r6, r6, r4, lsr #2
	beq		busy			/* b if DQ5 low */
	ldrb	r6, [r1]
	eor		r7, r5, r6
	ands	r7, r4, r7
	beq		cont			/* b if DQ7 == Data7 */
	mov		r5, #0			/* 0x0 - return 0x00, error */
	bne		done
cont:
	subs	r2, r2, #1		/* 0x1 */
	moveq	r5, #128		/* 0x80 */
	beq		done
	add		r1, r1, #1		/* 0x1 */
	b		code
done:
	b		done

	.end
