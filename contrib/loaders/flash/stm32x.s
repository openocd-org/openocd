/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

	.text
	.arm

/*
	r0 - source address
	r1 - target address
	r2 - count (halfword-16bit)
	r3 - result
	r4 - temp
	r5 - temp
*/

write:
	ldr		r4, STM32_FLASH_CR
	ldr		r5, STM32_FLASH_SR
	mov		r3, #1
	str		r3, [r4, #0]
	ldrh 	r3, [r0], #2
	strh 	r3, [r1], #2
busy:
	ldr 	r3, [r5, #0]
	tst 	r3, #0x01
	beq 	busy
	tst		r3, #0x14
	bne		exit
	subs	r2, r2, #1
	bne		write
exit:
	bkpt	#0

STM32_FLASH_CR: .word 0x40022010
STM32_FLASH_SR:	.word 0x4002200C
