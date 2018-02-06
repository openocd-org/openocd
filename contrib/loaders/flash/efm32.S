/***************************************************************************
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb
	.thumb_func

	/* Params:
	 * r0 - flash base (in), status (out)
	 * r1 - count (word-32bit)
	 * r2 - workarea start
	 * r3 - workarea end
	 * r4 - target address
	 * Clobbered:
	 * r5 - rp
	 * r6 - wp, tmp
	 * r7 - tmp
	 */

/* offsets of registers from flash reg base */
#define EFM32_MSC_WRITECTRL_OFFSET      0x008
#define EFM32_MSC_WRITECMD_OFFSET       0x00c
#define EFM32_MSC_ADDRB_OFFSET          0x010
#define EFM32_MSC_WDATA_OFFSET          0x018
#define EFM32_MSC_STATUS_OFFSET         0x01c

	/* set WREN to 1 */
	movs    r6, #1
	str     r6, [r0, #EFM32_MSC_WRITECTRL_OFFSET]

wait_fifo:
	ldr     r6, [r2, #0]    /* read wp */
	cmp     r6, #0          /* abort if wp == 0 */
	beq     exit
	ldr     r5, [r2, #4]    /* read rp */
	cmp     r5, r6          /* wait until rp != wp */
	beq     wait_fifo

	/* store address in MSC_ADDRB */
	str     r4, [r0, #EFM32_MSC_ADDRB_OFFSET]
	/* set LADDRIM bit */
	movs    r6, #1
	str     r6, [r0, #EFM32_MSC_WRITECMD_OFFSET]
	/* check status for INVADDR and/or LOCKED */
	ldr     r6, [r0, #EFM32_MSC_STATUS_OFFSET]
	movs    r7, #6
	tst     r6, r7
	bne     error

	/* wait for WDATAREADY */
wait_wdataready:
	ldr     r6, [r0, #EFM32_MSC_STATUS_OFFSET]
	movs    r7, #8
	tst     r6, r7
	beq     wait_wdataready

	/* load data to WDATA */
	ldr     r6, [r5]
	str     r6, [r0, #EFM32_MSC_WDATA_OFFSET]
	/* set WRITEONCE bit */
	movs    r6, #8
	str     r6, [r0, #EFM32_MSC_WRITECMD_OFFSET]

	adds    r5, #4          /* rp++ */
	adds    r4, #4          /* target_address++ */

	/* wait until BUSY flag is reset */
busy:
	ldr     r6, [r0, #EFM32_MSC_STATUS_OFFSET]
	movs    r7, #1
	tst     r6, r7
	bne     busy

	cmp     r5, r3          /* wrap rp at end of buffer */
	bcc     no_wrap
	mov     r5, r2
	adds    r5, #8
no_wrap:
	str     r5, [r2, #4]    /* store rp */
	subs    r1, r1, #1      /* decrement word count */
	cmp     r1, #0
	beq     exit            /* loop if not done */
	b       wait_fifo
error:
	movs    r0, #0
	str     r0, [r2, #4]    /* set rp = 0 on error */
exit:
	mov     r0, r6          /* return status in r0 */
	bkpt    #0
