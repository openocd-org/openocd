/*
 * Infineon XMC1000 flash sectors erase
 *
 * Copyright (c) 2016 Andreas Färber
 *
 * Based on XMC1100 AA-Step Reference Manual
 *
 * License: GPL-2.0+
 */

#include "xmc1xxx.S"

#define DUMMY_VALUE 0x42

	.macro erase_page, nvmbase, addr, tmp, tmp2

	movs	\tmp, #DUMMY_VALUE
	str	\tmp, [\addr]

	busy_wait \nvmbase, \tmp, \tmp2

	.endm


	.macro erase, nvmbase, addr, end, tmp, tmp2

	movs	\tmp, #NVMPROG_ACTION_PAGE_ERASE_CONTINUOUS
	strh	\tmp, [\nvmbase, #NVMPROG]
2001:
	erase_page \nvmbase, \addr, \tmp, \tmp2

	movs	\tmp, #(NVM_PAGE_SIZE - 1)
	adds	\tmp, \tmp, #1
	add	\addr, \addr, \tmp
	cmp	\addr, \end
	blt	2001b

	movs	\tmp, #NVMPROG_ACTION_IDLE
	strh	\tmp, [\nvmbase, #NVMPROG]

	.endm


	/*
	 * r0 = 0x40050000
	 * r1 = e.g. 0x10001000
	 * r2 = e.g. 0x10011000
	 * NVMPROG.ACTION = 0x00
	 */
erase:
	erase r0, r1, r2, r3, r4

	bkpt	#0
