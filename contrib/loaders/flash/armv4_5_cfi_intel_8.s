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

/* algorithm register usage:
 * r0: source address (in RAM)
 * r1: target address (in Flash)
 * r2: count
 * r3: flash write command
 * r4: status byte (returned to host)
 * r5: busy test pattern
 * r6: error test pattern
 */

loop:
	ldrb	r4, [r0], #1
	strb	r3, [r1]
	strb	r4, [r1]
busy:
	ldrb	r4, [r1]
	and		r7, r4, r5
	cmp		r7, r5
	bne		busy
	tst		r4, r6
	bne		done
	subs	r2, r2, #1
	beq		done
	add		r1, r1, #1
	b		loop
done:
	b		done

	.end
