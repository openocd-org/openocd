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
	.syntax unified
	.cpu cortex-m3
	.thumb
	.thumb_func
	.global write

/*
	r0 - source address
	r1 - target address
	r2 - count (halfword-16bit)
	r3 - sector offet in : result out
	r4 - flash base
*/

#define STM32_FLASH_CR_OFFSET	0x10			/* offset of CR register in FLASH struct */
#define STM32_FLASH_SR_OFFSET	0x0c			/* offset of CR register in FLASH struct */

write:
	ldr		r4, STM32_FLASH_BASE
	add		r4, r3								/* add offset 0x00 for sector 0 : 0x40 for sector 1 */
write_half_word:
	movs	r3, #0x01
	str		r3, [r4, #STM32_FLASH_CR_OFFSET]	/* PG (bit0) == 1 => flash programming enabled */
	ldrh 	r3, [r0], #0x02						/* read one half-word from src, increment ptr */
	strh 	r3, [r1], #0x02						/* write one half-word from src, increment ptr */
busy:
	ldr 	r3, [r4, #STM32_FLASH_SR_OFFSET]
	tst 	r3, #0x01							/* BSY (bit0) == 1 => operation in progress */
	beq 	busy								/* wait more... */
	tst		r3, #0x14							/* PGERR (bit2) == 1 or WRPRTERR (bit4) == 1 => error */
	bne		exit								/* fail... */
	subs	r2, r2, #0x01						/* decrement counter */
	bne		write_half_word						/* write next half-word if anything left */
exit:
	bkpt	#0x00

STM32_FLASH_BASE: .word 0x40022000				/* base address of FLASH struct */
