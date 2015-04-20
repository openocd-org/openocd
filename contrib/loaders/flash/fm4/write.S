/*
 * Spansion FM4 flash write algorithm
 *
 * Copyright (c) 2015 Andreas Färber
 *
 * Based on S6E2CC_MN709-00007 for S6E2CC/C5/C4/C3/C2/C1 series
 */

#include "fm4.h"

#define RESULT_OKAY	0
#define RESULT_NONE	1
#define RESULT_TIMEOUT	2

	.macro busy_wait, res, addr, data, tmp1, tmp2, tmp3

	ldrb	\tmp1, [\addr] /* ignore */
	and	\tmp2, \data, #FLASH_DPOL
1001:
	ldrb	\tmp1, [\addr]
	and	\tmp3, \tmp1, #FLASH_DPOL
	cmp	\tmp3, \tmp2
	beq	1010f

	and	\tmp3, \tmp1, #FLASH_TLOV
	cmp	\tmp3, #0
	beq	1001b

	ldrb	\tmp1, [\addr]
	and	\tmp3, \tmp1, #FLASH_DPOL
	cmp	\tmp3, \tmp2
	beq	1010f

	mov	\res, #RESULT_TIMEOUT
	bkpt	#0
1010:
	.endm


	.macro write_one, res, cmdseqaddr1, cmdseqaddr2, pa, pd, tmp1, tmp2, tmp3

	mov	\tmp1, #0xAA
	strh	\tmp1, [\cmdseqaddr1]
	mov	\tmp1, #0x55
	strh	\tmp1, [\cmdseqaddr2]
	mov	\tmp1, #0xA0
	strh	\tmp1, [\cmdseqaddr1]
	strh	\pd, [\pa]

	busy_wait \res, \pa, \pd, \tmp1, \tmp2, \tmp3

	.endm


	.macro write, cmdseqaddr1, cmdseqaddr2, dest, src, cnt, res, tmp1, tmp2, tmp3, tmp4

	mov	\res, #RESULT_NONE
2001:
	cbz	\cnt, 2010f

	ldrh	\tmp1, [\src]
	write_one \res, \cmdseqaddr1, \cmdseqaddr2, \dest, \tmp1, \tmp2, \tmp3, \tmp4

	sub	\cnt, \cnt, #1
	add	\dest, \dest, #2
	add	\src, \src, #2
	b	2001b
2010:
	mov	\res, #RESULT_OKAY
	.endm


	/* r0 = 0xAA8
	 * r1 = 0x554
	 * r2 = dest
	 * r3 = src
	 * r4 = cnt
	 * r5 = result
	 */
write:
	write r0, r1, r2, r3, r4, r5, r6, r7, r8, r9

	bkpt	#0

data:
