/***************************************************************************
 *   Copyright (C) 2014 by Angus Gratton                                   *
 *   Derived from stm32f1x.S:
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *   Copyright (C) 2013 by Roman Dmitrienko                                *
 *   me@iamroman.org                                                       *
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
	.text
	.syntax unified
	.cpu cortex-m0
	.thumb
	.thumb_func

/* Written for NRF51822 (src/flash/nor/nrf51.c) however the NRF NVMC is
 * very generic (CPU blocks during flash writes), so this is actually
 * just a generic word-oriented copy routine for Cortex-M0 (also
 * suitable for Cortex-M0+/M3/M4.)
 *
 * To assemble:
 * arm-none-eabi-gcc -c cortex-m0.S
 *
 * To disassemble:
 * arm-none-eabi-objdump -o cortex-m0.o
 *
 * Thanks to Jens Bauer for providing advice on some of the tweaks.
 */

	/* Params:
	 * r0 - byte count (in)
	 * r1 - workarea start
	 * r2 - workarea end
	 * r3 - target address
	 * Clobbered:
	 * r4 - rp
	 * r5 - wp, tmp
	 */

wait_fifo:
	ldr 	r5, [r1, #0]	/* read wp */
	cmp 	r5, #0	        /* abort if wp == 0 */
	beq 	exit
	ldr 	r4, [r1, #4]	/* read rp */
	cmp 	r4, r5		/* wait until rp != wp */
	beq 	wait_fifo

	ldmia	r4!, {r5}	/* "*target_address++ = *rp++" */
        stmia   r3!, {r5}

        cmp 	r4, r2		/* wrap rp at end of work area buffer */
	bcc	no_wrap
	mov	r4, r1
	adds	r4, #8          /* skip rp,wp at start of work area */
no_wrap:
	str 	r4, [r1, #4]	/* write back rp */
	subs	r0, #4          /* decrement byte count */
	bne     wait_fifo	/* loop if not done */
exit:
	bkpt    #0
