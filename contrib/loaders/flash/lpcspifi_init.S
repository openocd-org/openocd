/***************************************************************************
 *   Copyright (C) 2012 by George Harris  		                           *
 *   george@luminairecoffee.com                                            *
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

/***************************************************************************
*	This is an algorithm for the LPC43xx family (and probably the LPC18xx  *
*	family as well, though they have not been tested) that will initialize *
*	memory-mapped SPI flash accesses. Unfortunately NXP has published      *
*	neither the ROM source code that performs this initialization nor the  *
*	register descriptions necessary to do so, so this code is necessary to *
*	call into the ROM SPIFI API.                                           *
***************************************************************************/

	.text
	.syntax unified
	.arch armv7-m
	.thumb
	.thumb_func

	.align 2

/*
 * Params :
 * r0 = spifi clock speed
 */

#define IOCONFIG_BASE_HIGH          0x4008
#define IOCONFIG_BASE_LOW           0x6000
#define IOCONFIG_SCK_OFFSET         0x18c
#define IOCONFIG_HOLD_OFFSET        0x190
#define IOCONFIG_WP_OFFSET          0x194
#define IOCONFIG_MISO_OFFSET        0x198
#define IOCONFIG_MOSI_OFFSET        0x19c
#define IOCONFIG_CS_OFFSET          0x1a0

#define SPIFI_ROM_TABLE_BASE_HIGH   0x1040
#define SPIFI_ROM_TABLE_BASE_LOW    0x0118

code:
	mov.w 	r8, r0
	sub		sp, #0x84
	add		r7, sp, #0x0
	/* Initialize SPIFI pins */
	mov.w	r3, #IOCONFIG_BASE_LOW
	movt	r3, #IOCONFIG_BASE_HIGH
	mov.w	r2, #0xf3
	str.w 	r2, [r3, #IOCONFIG_SCK_OFFSET]
	mov.w	r3, #IOCONFIG_BASE_LOW
	movt	r3, #IOCONFIG_BASE_HIGH
	mov.w	r2, #IOCONFIG_BASE_LOW
	movt	r2, #IOCONFIG_BASE_HIGH
	mov.w	r1, #IOCONFIG_BASE_LOW
	movt	r1, #IOCONFIG_BASE_HIGH
	mov.w	r0, #IOCONFIG_BASE_LOW
	movt	r0, #IOCONFIG_BASE_HIGH
	mov.w	r4, #0xd3
	str.w	r4, [r0, #IOCONFIG_MOSI_OFFSET]
	mov	r0, r4
	str.w	r0, [r1, #IOCONFIG_MISO_OFFSET]
	mov	r1, r0
	str.w	r1, [r2, #IOCONFIG_WP_OFFSET]
	str.w	r1, [r3, #IOCONFIG_HOLD_OFFSET]
	mov.w	r3, #IOCONFIG_BASE_LOW
	movt	r3, #IOCONFIG_BASE_HIGH
	mov.w	r2, #0x13
	str.w	r2, [r3, #IOCONFIG_CS_OFFSET]

	/* Perform SPIFI init. See spifi_rom_api.h (in NXP lpc43xx driver package) for details */
	/* on initialization arguments. */
	movw 	r3, #SPIFI_ROM_TABLE_BASE_LOW      /* The ROM API table is located @ 0x10400118, and			*/
	movt 	r3, #SPIFI_ROM_TABLE_BASE_HIGH     /* the first pointer in the struct is to the init function. */
	ldr 	r3, [r3, #0x0]
	ldr 	r4, [r3, #0x0]		                 /* Grab the init function pointer from the table */
	/* Set up function arguments */
	movw 	r0, #0x3b4
	movt 	r0, #0x1000		    	              /* Pointer to a SPIFI data struct that we don't care about */
	mov.w 	r1, #0x3                        /* "csHigh". Not 100% sure what this does. */
	mov.w 	r2, #0xc0 			              /* The configuration word: S_RCVCLOCK | S_FULLCLK */
	mov.w 	r3, r8 				              /* SPIFI clock speed (12MHz) */
	blx 	r4					                    /* Call the init function */
	b 		done

done:
	bkpt 	#0

	.end
