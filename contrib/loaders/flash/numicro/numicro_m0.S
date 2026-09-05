/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2015 Nemui Trinomius                                    *
 *   nemuisan_kawausogasuki@live.jp                                        *
 *                                                                         *
 *   Copyright (C) 2017 Zale Yu                                            *
 *   CYYU@nuvoton.com                                                      *
 *                                                                         *
 *   Copyright (C) 2022-2026 Jian-Hong Pan                                 *
 *   chienhung.pan@gmail.com                                               *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb

	/* Params:
	 * r0 - workarea buffer pointer / return result
	 * r1 - target flash address
	 * r2 - wordcount
	 * r3 - FMC / ISP register base address (0x5000C000 or 0x4000C000)
	 *
	 * Clobbered:
	 * r4 - tmp
	 * r5 - tmp
	 */

	.thumb_func
	.global _start
_start:
.L_loop:
	cmp	r2, #0
	beq	.L_done

	/* NUMICRO_FLASH_ISPADR (offset 0x04) = faddr */
	str	r1, [r3, #4]

	/* NUMICRO_FLASH_ISPDAT (offset 0x08) = *pLW++ */
	ldmia	r0!, {r5}
	str	r5, [r3, #8]

	/* NUMICRO_FLASH_ISPTRG (offset 0x10) = ISPTRG_ISPGO (1) */
	movs	r4, #1
	str	r4, [r3, #16]

.L_busy_wait:
	/* while ((NUMICRO_FLASH_ISPTRG & ISPTRG_ISPGO) == ISPTRG_ISPGO) */
	ldr	r5, [r3, #16]
	lsls	r5, r5, #31
	bmi	.L_busy_wait

	adds	r1, r1, #4
	subs	r2, r2, #1
	b	.L_loop

.L_done:
	/* return (NUMICRO_FLASH_ISPCON & ISPCON_ISPFF) */
	ldr	r0, [r3, #0]
	movs	r4, #64
	ands	r0, r4

	bkpt	#0
