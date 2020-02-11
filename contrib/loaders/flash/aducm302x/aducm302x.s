/***************************************************************************
 *   Modifed from stellaris.s                                              *
 *   Copyright (C) 2016 - 2018 Analog Devices, Inc.                        *
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
  .cpu cortex-m3
  .thumb
  .thumb_func

/*
 * Params :
 * r0 = workarea start
 * r1 = workarea end
 * r2 = target address
 * r3 = count (64bit double words)
 * r4 = FLASH_CTRL_BASE
 * r5 = FLASHWRITECMD
 *
 * Clobbered:
 * r6 - tmp
 * r7 - rp
 * r8 - wp, tmp
 */

wait_fifo:
  ldr   r8, [r0, #0]  /* read wp */
  cmp   r8, #0      /* abort if wp == 0 */
  beq   exit
  ldr   r7, [r0, #4]  /* read rp */
  cmp   r7, r8      /* wait until rp != wp */
  beq   wait_fifo

mainloop:
  str   r2, [r4, #0xc]  /* KH_ADDR - write address */
  add   r2, r2, #8    /* increment target address */
  ldr   r6, [r7], #4
  ldr   r8, [r7], #4
  str   r6, [r4, #0x10] /* KH_DATA0 - write data */
  str   r8, [r4, #0x14] /* KH_DATA1 - write data */
  str   r5, [r4, #8]  /* CMD - enable write */
busy:
  ldr   r8, [r4, #0]
  tst   r8, #4
  beq   busy

  cmp   r7, r1      /* wrap rp at end of buffer */
  it    cs
  addcs r7, r0, #8    /* skip loader args */
  str   r7, [r0, #4]  /* store rp */
  subs  r3, r3, #1    /* decrement word count */
  cbz   r3, exit    /* loop if not done */
  b   wait_fifo
exit:
  bkpt  #0