/***************************************************************************
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   Copyright (C) 2013 by Paul Fertser                                    *
 *   fercerpav@gmail.com                                                   *
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
	.syntax unified
	.cpu cortex-m0
	.thumb
	.thumb_func
	.global write

	/* Params:
	 * r0 - flash base (in), status (out)
	 * r1 - count (32bit)
	 * r2 - workarea start
	 * r3 - workarea end
	 * r4 - target address
	 * Clobbered:
	 * r5 - rp
	 * r6 - wp, tmp
	 * r7 - current FLASH_CMD
	 */

#define FLASH_CMD	0x00
#define FLASH_ADR	0x04
#define FLASH_DI	0x08

#define FLASH_NVSTR	(1 << 13)
#define FLASH_PROG	(1 << 12)
#define FLASH_MAS1	(1 << 11)
#define FLASH_ERASE	(1 << 10)
#define FLASH_SE	(1 << 8)
#define FLASH_YE	(1 << 7)
#define FLASH_XE	(1 << 6)

	ldr	r7, [r0, #FLASH_CMD]
wait_fifo:
	ldr 	r6, [r2, #0]	/* read wp */
	cmp 	r6, #0			/* abort if wp == 0 */
	beq 	exit
	ldr 	r5, [r2, #4]	/* read rp */
	cmp 	r5, r6			/* wait until rp != wp */
	beq 	wait_fifo

	ldr	r6, [r5]	/* "*target_address++ = *rp++" */
	str	r4, [r0, #FLASH_ADR]
	str	r6, [r0, #FLASH_DI]

	ldr	r6, =(FLASH_XE | FLASH_PROG)
	orrs	r7, r7, r6
	str	r7, [r0, #FLASH_CMD]
	# wait 5us
	movs	r6, #5
	bl	delay
	ldr	r6, =#FLASH_NVSTR
	orrs	r7, r7, r6
	str	r7, [r0, #FLASH_CMD]
	# wait 10us
	movs	r6, #13
	bl	delay
	movs	r6, #FLASH_YE
	orrs	r7, r7, r6
	str	r7, [r0, #FLASH_CMD]
	# wait 40us
	movs	r6, #61
	bl	delay
	movs	r6, #FLASH_YE
	bics	r7, r7, r6
	str	r7, [r0, #FLASH_CMD]
	ldr	r6, =#FLASH_PROG
	bics	r7, r7, r6
	str	r7, [r0, #FLASH_CMD]
	# wait 5us
	movs	r6, #5
	bl	delay
	ldr	r6, =#(FLASH_XE | FLASH_NVSTR)
	bics	r7, r7, r6
	str	r7, [r0, #FLASH_CMD]

	adds	r5, #4
	adds	r4, #4

	cmp 	r5, r3			/* wrap rp at end of buffer */
	bcc	no_wrap
	mov	r5, r2
	adds	r5, #8
no_wrap:
	str 	r5, [r2, #4]	/* store rp */
	subs	r1, r1, #1		/* decrement word count */
	cmp     r1, #0
	beq     exit		/* loop if not done */
	b	wait_fifo
exit:
	mov		r0, r6			/* return status in r0 */
	bkpt	#0

	/* r6 - in
	 * for r6 == 1 it'll take:
	 *  1 (prepare operand) + 4 (bl) + 2 (subs+cmp) + 1 (bne) + 3 (b) ->
	 *  11 tacts == 1.4us with 8MHz
	 * every extra iteration will take 5 tacts == 0.6us */
delay:
	subs	r6, r6, #1
	cmp	r6, #0
	bne	delay
	bx	lr
