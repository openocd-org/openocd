/*
 * Infineon XMC1000 flash sector erase check
 *
 * Copyright (c) 2016 Andreas Färber
 *
 * Based on XMC1100 AA-Step Reference Manual
 *
 * License: GPL-2.0+
 */

#include "xmc1xxx.S"

	.macro verify_block, nvmbase, addr, tmp, tmp2

	movs	\tmp, #0x00
	mvns	\tmp, \tmp
	str	\tmp, [\addr, #0x0]
	str	\tmp, [\addr, #0x4]
	str	\tmp, [\addr, #0x8]
	str	\tmp, [\addr, #0xC]

	busy_wait \nvmbase, \tmp, \tmp2

	.endm


	.macro erase_check, nvmbase, addr, end, tmp, tmp2

	ldrh	\tmp, [\nvmbase, #NVMCONF]
	movs	\tmp2, #NVMCONF_HRLEV_MASK
	mvns	\tmp2, \tmp2
	ands	\tmp, \tmp, \tmp2
	movs	\tmp2, #NVMCONF_HRLEV_HRE
	orrs	\tmp, \tmp, \tmp2
	strh	\tmp, [\nvmbase, #NVMCONF]

	movs	\tmp, #NVMPROG_ACTION_VERIFY_CONTINUOUS
	strh	\tmp, [\nvmbase, #NVMPROG]
2001:
	verify_block \nvmbase, \addr, \tmp, \tmp2

	ldrh	\tmp, [\nvmbase, #NVMSTATUS]
	movs	\tmp2, #NVMSTATUS_VERR_MASK
	ands	\tmp, \tmp, \tmp2
	cmp	\tmp, #NVMSTATUS_VERR_NOFAIL
	bne	2010f

	adds	\addr, \addr, #NVM_BLOCK_SIZE
	cmp	\addr, \end
	blt	2001b
2010:
	movs	\tmp, #NVMPROG_ACTION_IDLE
	strh	\tmp, [\nvmbase, #NVMPROG]

	.endm


	/*
	 * r0 = 0x40050000
	 * r1 = e.g. 0x10001000
	 * r2 = e.g. 0x10002000
	 * NVMPROG.ACTION = 0x00
	 */
erase_check:
	erase_check r0, r1, r2, r3, r4

	bkpt	#0
