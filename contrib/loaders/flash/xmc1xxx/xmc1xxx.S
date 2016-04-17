/*
 * Infineon XMC1000 flash
 *
 * Copyright (c) 2016 Andreas Färber
 *
 * Based on XMC1100 AA-Step Reference Manual
 *
 * License: GPL-2.0+
 */

	.text
	.syntax unified
	.cpu cortex-m0
	.thumb
	.thumb_func

#define NVMSTATUS	0x00
#define NVMPROG		0x04
#define NVMCONF		0x08

#define NVMSTATUS_BUSY		(1 << 0)
#define NVMSTATUS_VERR_NOFAIL	(0x0 << 2)
#define NVMSTATUS_VERR_MASK	(0x3 << 2)

#define NVMPROG_ACTION_IDLE			0x00
#define NVMPROG_ACTION_WRITE_CONTINUOUS		0xA1
#define NVMPROG_ACTION_PAGE_ERASE_CONTINUOUS	0xA2
#define NVMPROG_ACTION_VERIFY_CONTINUOUS	0xE0

#define NVMCONF_HRLEV_NR	(0x0 << 1)
#define NVMCONF_HRLEV_HRE	(0x2 << 1)
#define NVMCONF_HRLEV_MASK	(0x3 << 1)

#define NVM_WORD_SIZE	4
#define NVM_BLOCK_SIZE	(4 * NVM_WORD_SIZE)
#define NVM_PAGE_SIZE	(16 * NVM_BLOCK_SIZE)

	.macro busy_wait, nvmbase, tmp, tmp2
1:
	ldrh	\tmp, [\nvmbase, #NVMSTATUS]
	movs	\tmp2, #NVMSTATUS_BUSY
	ands	\tmp, \tmp, \tmp2
	cmp	\tmp, \tmp2
	beq	1b

	.endm
