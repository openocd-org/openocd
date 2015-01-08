/***************************************************************************
 *   Copyright (C) 2014 by Ladislav Bábel                                  *
 *   ladababel@seznam.cz                                                   *
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

#define INITIAL_UNLOCK    0x5A
#define MULTIPLE_UNLOCK   0xF2

#define FLASHCTRL_KEY     0x4002E0C0
#define FLASHCTRL_CONFIG  0x4002E000
#define FLASHCTRL_WRADDR  0x4002E0A0
#define FLASHCTRL_WRDATA  0x4002E0B0
#define BUSYF             0x00100000


		/* Write the initial unlock value to KEY (0xA5) */
		movs    r6, #INITIAL_UNLOCK
		str     r6, [r0, #FLASHCTRL_KEY]

		/* Write the multiple unlock value to KEY (0xF2) */
		movs    r6, #MULTIPLE_UNLOCK
		str     r6, [r0, #FLASHCTRL_KEY]

wait_fifo:
		ldr     r6, [r2, #0]
		cmp	    r6, #0
		beq     exit
		ldr     r5, [r2, #4]
		cmp     r5, r6
		beq     wait_fifo

		/* wait for BUSYF flag */
wait_busy1:
		ldr     r6, [r0, #FLASHCTRL_CONFIG]
		tst     r6, #BUSYF
		bne     wait_busy1

		/* Write the destination address to WRADDR */
		str     r4, [r0, #FLASHCTRL_WRADDR]

		/* Write the data half-word to WRDATA in right-justified format */
		ldrh    r6, [r5]
		str     r6, [r0, #FLASHCTRL_WRDATA]

		adds    r5, #2
		adds    r4, #2

		/* wrap rp at end of buffer */
		cmp     r5, r3
		bcc     no_wrap
		mov     r5, r2
		adds    r5, #8

no_wrap:
		str     r5, [r2, #4]
		subs    r1, r1, #1
		cmp     r1, #0
		beq     exit
		b       wait_fifo

exit:
		movs    r6, #MULTIPLE_LOCK
		str     r6, [r0, #FLASHCTRL_KEY]

		/* wait for BUSYF flag */
wait_busy2:
		ldr     r6, [r0, #FLASHCTRL_CONFIG]
		tst     r6, #BUSYF
		bne     wait_busy2

		bkpt    #0
