/*
 * Infineon XMC1000 flash write
 *
 * Copyright (c) 2016 Andreas Färber
 *
 * Based on XMC1100 AA-Step Reference Manual
 *
 * License: GPL-2.0+
 */

#include "xmc1xxx.S"

	.macro write_block, nvmbase, dest, src, tmp, tmp2

	ldr	\tmp, [\src,  #0x0]
	str	\tmp, [\dest, #0x0]
	ldr	\tmp, [\src,  #0x4]
	str	\tmp, [\dest, #0x4]
	ldr	\tmp, [\src,  #0x8]
	str	\tmp, [\dest, #0x8]
	ldr	\tmp, [\src,  #0xc]
	str	\tmp, [\dest, #0xc]

	busy_wait \nvmbase, \tmp, \tmp2

	.endm


	.macro write, nvmbase, dest, src, count, tmp, tmp2

	movs	\tmp, #NVMPROG_ACTION_WRITE_CONTINUOUS
	strh	\tmp, [\nvmbase, #NVMPROG]
1001:
	write_block \nvmbase, \dest, \src, \tmp, \tmp2

	adds	\dest, \dest, #NVM_BLOCK_SIZE
	adds	\src, \src, #NVM_BLOCK_SIZE
	subs	\count, \count, #1
	cmp	\count, #0
	bgt	1001b

	movs	\tmp, #NVMPROG_ACTION_IDLE
	strh	\tmp, [\nvmbase, #NVMPROG]

	.endm


	/*
	 * r0 = 0x40050000
	 * r1 = e.g. 0x10001000
	 * r2 = e.g. 0x20000000
	 * r3 = e.g. 1
	 * NVMPROG.ACTION = 0x00
	 */
write:
	write r0, r1, r2, r3, r4, r5

	bkpt	#0
