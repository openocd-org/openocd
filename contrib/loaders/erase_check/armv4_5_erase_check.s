/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

/*
	parameters:
	r0 - address in
	r1 - byte count
	r2 - mask - result out
*/

	.text
	.arm

loop:
	ldrb r3, [r0], #1
	and r2, r2, r3
	subs r1, r1, #1
	bne loop
end:
	bkpt	#0

	.end
