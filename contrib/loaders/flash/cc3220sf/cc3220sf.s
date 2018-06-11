/***************************************************************************
 *   Copyright (C) 2017 by Texas Instruments, Inc.                         *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

	/* Params:
	 * r0 = buffer start address (in)
	 * r1 = flash destination address (in)
	 * r2 = number of words to write (in/out)
	 */

	.text
	.cpu cortex-m4
	.code 16
	.thumb
	.syntax unified

	.align 2

	/* r3 = scratchpad
	 * r4 = buffer word counter
	 * r10 = flash programming key
	 * r11 = base FWB address
	 * r12 = base flash regs address
	 */

start:
	ldr 	r10, =0xa4420001	/* flash programming key */
	ldr 	r11, =0x400fd100	/* base of FWB */
	ldr 	r12, =0x400fd000	/* base of flash regs */
	and 	r3, r1, #0x7f		/* is the dest address 32 word aligned? */
	cmp 	r3, #0
	bne 	program_word		/* if not aligned do one word at a time */

	/* program using the write buffers */
program_buffer:
	mov 	r4, #0				/* start the buffer word counter at 0 */
	str 	r1, [r12]			/* store the dest addr in FMA */
fill_buffer:
	ldr 	r3, [r0]			/* get the word to write to FWB */
	str 	r3, [r11]			/* store the word in the FWB */
	add 	r11, r11, #4		/* increment the FWB pointer */
	add 	r0, r0, #4			/* increment the source pointer */
	sub 	r2, r2, #1			/* decrement the total word counter */
	add 	r4, r4, #1			/* increment the buffer word counter */
	add 	r1, r1, #4			/* increment the dest pointer */
	cmp 	r2, #0				/* is the total word counter now 0? */
	beq 	buffer_ready		/* go to end if total word counter is 0 */
	cmp 	r4, #32				/* is the buffer word counter now 32? */
	bne 	fill_buffer			/* go to continue to fill buffer */
buffer_ready:
	str 	r10, [r12, #0x20]	/* store the key and write bit to FMC2 */
wait_buffer_done:
	ldr 	r3, [r12, #0x20]	/* read FMC2 */
	tst 	r3, #1				/* see if the write bit is cleared */
	bne 	wait_buffer_done	/* go to read FMC2 if bit not cleared */
	cmp 	r2, #0				/* is the total word counter now 0? */
	bne 	start				/* go if there is more to program */
	b   	exit

	/* program just one word */
program_word:
	str 	r1, [r12]			/* store the dest addr in FMA */
	ldr 	r3, [r0]			/* get the word to write to FMD */
	str 	r3, [r12, #0x4]		/* store the word in FMD */
	str 	r10, [r12, #0x8]	/* store the key and write bit to FMC */
wait_word_done:
	ldr 	r3, [r12, #0x8]		/* read FMC */
	tst 	r3, #1				/* see if the write bit is cleared */
	bne 	wait_word_done		/* go to read FMC if bit not cleared */
	sub 	r2, r2, #1			/* decrement the total word counter */
	add 	r0, r0, #4			/* increment the source pointer */
	add 	r1, r1, #4			/* increment the dest pointer */
	cmp 	r2, #0				/* is the total word counter now 0 */
	bne 	start				/* go if there is more to program */

	/* end */
exit:
	bkpt	#0
	bkpt	#1
	b   	exit
