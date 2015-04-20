/*
 * Spansion FM4 flash sector erase algorithm
 *
 * Copyright (c) 2015 Andreas Färber
 *
 * Based on S6E2CC_MN709-00007 for S6E2CC/C5/C4/C3/C2/C1 series
 */

#include "fm4.h"

#define RESULT_OKAY	0
#define RESULT_NONE	1
#define RESULT_TIMEOUT	2

	.macro busy_wait, res, addr, tmp1, tmp2, tmp3

	ldrb	\tmp1, [\addr] /* ignore */
1001:
	ldrb	\tmp1, [\addr]
	ldrb	\tmp2, [\addr]

	and	\tmp3, \tmp1, #FLASH_TOGG
	and	\tmp2, \tmp2, #FLASH_TOGG
	cmp	\tmp3, \tmp2
	beq	1010f

	and	\tmp2, \tmp1, #FLASH_TLOV
	cmp	\tmp2, #0
	beq	1001b

	ldrb	\tmp1, [\addr]
	ldrb	\tmp2, [\addr]

	and	\tmp3, \tmp1, #FLASH_TOGG
	and	\tmp2, \tmp2, #FLASH_TOGG
	cmp	\tmp3, \tmp2
	beq	1010f

	mov	\res, #RESULT_TIMEOUT
	bkpt	#0
1010:
	mov	\res, #RESULT_OKAY

	.endm


	.macro erase, cmdseqaddr1, cmdseqaddr2, sa, res, tmp1, tmp2, tmp3

	mov	\res, #RESULT_NONE

	mov	\tmp1, #0xAA
	strh	\tmp1, [\cmdseqaddr1]
	mov	\tmp2, #0x55
	strh	\tmp2, [\cmdseqaddr2]
	mov	\tmp3, #0x80
	strh	\tmp3, [\cmdseqaddr1]
	strh	\tmp1, [\cmdseqaddr1]
	strh	\tmp2, [\cmdseqaddr2]
	mov	\tmp3, #0x30
	strh	\tmp3, [\sa]

	busy_wait \res, \sa, \tmp1, \tmp2, \tmp3

	.endm


	/* r0 = 0xAA8
	 * r1 = 0x554
	 * r2 = SA
	 * r3 = result
	 */
erase:
	erase r0, r1, r2, r3, r4, r5, r6

	bkpt	#0

data:
