/***************************************************************************
 *   Copyright (C) 2005, 2007 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *   Copyright (C) 2010 Spencer Oliver                                     *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
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
	ldrh	r4, [r0], #2
	strh	r3, [r1]
	strh	r4, [r1]
busy:
	ldrh	r4, [r1]
	and		r7, r4, r5
	cmp		r7, r5
	bne		busy
	tst		r4, r6
	bne		done
	subs	r2, r2, #1
	beq		done
	add		r1, r1, #2
	b		loop
done:
	b		done

	.end
