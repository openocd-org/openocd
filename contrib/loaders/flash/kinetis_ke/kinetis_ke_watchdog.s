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

	.text
	.cpu cortex-m0plus
	.code 16
	.thumb_func

	.align	2

	ldr		r3, wdog_cs1
	mov		r2, #127
	ldrb	r5, [r3]
	ldrb	r4, [r3, #1]
	and		r2, r5
	ldr		r5, unlock1
	ldrh	r0, [r3, #4]
	ldrh	r1, [r3, #6]
	strh	r5, [r3, #2]
	ldr		r5, unlock2
	strh	r5, [r3, #2]
	strb	r4, [r3, #1]
	strh	r0, [r3, #4]
	strh	r1, [r3, #6]
	strb	r2, [r3]
	bkpt	#0

	.align	2

wdog_cs1:
	.word	0x40052000	// Watchdog Control and Status Register 1
unlock1:
	.word	0x20C5		// 1st unlock word
unlock2:
	.word	0x28D9		// 2nd unlock word
