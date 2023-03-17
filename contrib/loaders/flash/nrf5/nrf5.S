/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2014 Angus Gratton                                      *
 *   gus@projectgus.com                                                    *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb

/*
 * Params :
 * r0 = byte count
 * r1 = buffer start
 * r2 = buffer end
 * r3 = target address
 * r6 = watchdog refresh value
 * r7 = watchdog refresh register address
 */

	.thumb_func
	.global _start
_start:
wait_fifo:
	// Kick the watchdog
	str	r6, [r7, #0]
	// Load write pointer
	ldr	r5, [r1, #0]
	// Abort if it is NULL
	cmp	r5, #0
	beq.n	exit
	// Load read pointer
	ldr	r4, [r1, #4]
	// Continue waiting if it equals the write pointer
	cmp	r4, r5
	beq.n	wait_fifo
	// Copy one word from buffer to target, and increment pointers
	ldmia	r4!, {r5}
	stmia	r3!, {r5}
	// If at end of buffer, wrap back to buffer start
	cmp	r4, r2
	bcc.n   no_wrap
	mov	r4, r1
	adds	r4, #8
no_wrap:
	// Update read pointer inside the buffer
	str	r4, [r1, #4]
	// Deduce the word transferred from the byte count
	subs	r0, #4
	// Start again
	bne.n   wait_fifo
exit:
	// Wait for OpenOCD
	bkpt	#0x00

	.pool
