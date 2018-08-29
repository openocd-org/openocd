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
 ***************************************************************************/

/*
	parameters:
	r0 - pointer to struct { uint32_t size_in_result_out, uint32_t addr }
	r1 - value to check
*/

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb
	.thumb_func

	.align	2

BLOCK_SIZE_RESULT	= 0
BLOCK_ADDRESS		= 4
SIZEOF_STRUCT_BLOCK	= 8

start:
block_loop:
	ldr	r2, [r0, #BLOCK_SIZE_RESULT]	/* get size */
	tst	r2, r2
	beq	done

	ldr	r3, [r0, #BLOCK_ADDRESS]	/* get address */

word_loop:
	ldr	r4, [r3]	/* read word */
	adds	r3, #4

	cmp	r4, r1
	bne	not_erased

	subs	r2, #1
	bne	word_loop

	movs	r4, #1		/* block is erased */
save_result:
	str	r4, [r0, #BLOCK_SIZE_RESULT]
	adds	r0, #SIZEOF_STRUCT_BLOCK
	b	block_loop

not_erased:
	movs	r4, #0
	b	save_result

/* Avoid padding at .text segment end. Otherwise exit point check fails. */
        .skip   ( . - start + 2) & 2, 0

done:
	bkpt	#0

	.end
