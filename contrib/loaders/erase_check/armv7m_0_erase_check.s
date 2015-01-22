/***************************************************************************
 *   Copyright (C) 2014 by Jeff Ciesielski                                 *
 *   jeffciesielski@gmail.com                                              *
 *                                                                         *
 *   Based on the armv7m erase checker by:                                 *
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
 ***************************************************************************/

/*
	parameters:
	r0 - address in
	r1 - byte count
	r2 - mask - result out
*/

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb
	.thumb_func

	.align	2

loop:
	ldrb	r3, [r0]
	adds	r0, #1
	orrs	r2, r2, r3
	subs	r1, r1, #1
	bne		loop
end:
	bkpt	#0

	.end
