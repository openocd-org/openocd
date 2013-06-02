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
	ldrh	r5, [r0], #2
	strh	r9, [r8]
	strh	r11, [r10]
	strh	r3, [r8]
	strh	r5, [r1]
	nop
busy:
	ldrh	r6, [r1]
	eor		r7, r5, r6
	ands	r7, #0x80
	bne		busy
	subs	r2, r2, #1	/* 0x1 */
	moveq	r5, #128	/* 0x80 */
	beq		done
	add		r1, r1, #2	/* 0x2 */
	b		code
done:
	b		done

	.end
