/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
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


// Build : arm-eabi-gcc -c stm32f2xxx.S
	.text
	.syntax unified
	.cpu cortex-m3
	.thumb
	.thumb_func
	.global write

/*
	r0 - source address
	r1 - target address
	r2 - count (halfword-16bit)
	r3 - result out
	r4 - flash base
*/

#define STM32_FLASH_CR_OFFSET	0x10			/* offset of CR register in FLASH struct */
#define STM32_FLASH_SR_OFFSET	0x0c			/* offset of CR register in FLASH struct */

write:

write_half_word:
	ldr		r3, STM32_PROG16
	str		r3, [r4, #STM32_FLASH_CR_OFFSET]
	ldrh 	r3, [r0], #0x02						/* read one half-word from src, increment ptr */
	strh 	r3, [r1], #0x02						/* write one half-word from src, increment ptr */
busy:
	ldr 	r3, [r4, #STM32_FLASH_SR_OFFSET]
	tst 	r3, #0x10000						/* BSY (bit0) == 1 => operation in progress */
	beq 	busy								/* wait more... */
	tst		r3, #0xf0							/* PGSERR | PGPERR | PGAERR | WRPERR */
	bne		exit								/* fail... */
	subs	r2, r2, #0x01						/* decrement counter */
	bne		write_half_word						/* write next half-word if anything left */
exit:
	bkpt	#0x00


STM32_PROG16: .word 0x101 						/* PG | PSIZE_16*/
