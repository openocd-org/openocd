/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2015 Nemui Trinomius                                    *
 *   nemuisan_kawausogasuki@live.jp                                        *
 *                                                                         *
 *   Copyright (C) 2017 Zale Yu                                            *
 *   CYYU@nuvoton.com                                                      *
 *                                                                         *
 *   Copyright (C) 2022 Jian-Hong Pan                                      *
 *   chienhung.pan@gmail.com                                               *
 ***************************************************************************/

	.text
	.cpu cortex-m0
	.thumb

	/* Params:
	 * r0 - workarea buffer / result
	 * r1 - target address
	 * r2 - wordcount
	 * Clobbered:
	 * r4 - tmp
	 * r5 - tmp
	 * r6 - tmp
	 * r7 - tmp
	 */

.L1:
	/* for(register uint32_t i=0;i<wcount;i++){ */
	mov	r4, r0
	mov	r3, #0
.L2:
	sub	r5, r1, r0
	add	r7, r4, r5
	cmp	r3, r2
	beq	.L7
.L4:
	/* NUMICRO_FLASH_ISPADR = faddr; */
	ldr	r6, .L8
	str	r7, [r6]
	/* NUMICRO_FLASH_ISPDAT = *pLW; */
	ldmia	r4!, {r7}
	ldr	r5, .L8+4
	str	r7, [r5]
	/* faddr += 4; */
	/* pLW++; */
	/*  Trigger write action  */
	/* NUMICRO_FLASH_ISPTRG = ISPTRG_ISPGO; */
	ldr	r5, .L8+8
	mov	r6, #1
	str	r6, [r5]
.L3:
	/* while((NUMICRO_FLASH_ISPTRG & ISPTRG_ISPGO) == ISPTRG_ISPGO){}; */
	ldr	r7, [r5]
	lsl	r7, r7, #31
	bmi	.L3

	add	r3, r3, #1
	b	.L2
.L7:
	/* return (NUMICRO_FLASH_ISPCON & ISPCON_ISPFF); */
	ldr	r3, .L8+12
	ldr	r0, [r3]
	mov	r1, #64
	and	r0, r1
.L9:
	bkpt	#0
.L8:
	.word	0x5000C004
	.word	0x5000C008
	.word	0x5000C010
	.word	0x5000C000
