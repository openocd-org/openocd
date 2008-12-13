/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef STM32X_H
#define STM32X_H

#include "flash.h"
#include "target.h"

typedef struct stm32x_options_s
{
	u16 RDP;
	u16 user_options;
	u16 protection[4];
} stm32x_options_t;

typedef struct stm32x_flash_bank_s
{
	stm32x_options_t option_bytes;
	working_area_t *write_algorithm;
	int ppage_size;
	int probed;
} stm32x_flash_bank_t;

/* stm32x register locations */

#define STM32_FLASH_ACR		0x40022000
#define STM32_FLASH_KEYR	0x40022004
#define STM32_FLASH_OPTKEYR	0x40022008
#define STM32_FLASH_SR		0x4002200C
#define STM32_FLASH_CR		0x40022010
#define STM32_FLASH_AR		0x40022014
#define STM32_FLASH_OBR		0x4002201C
#define STM32_FLASH_WRPR	0x40022020

/* option byte location */

#define STM32_OB_RDP		0x1FFFF800
#define STM32_OB_USER		0x1FFFF802
#define STM32_OB_DATA0		0x1FFFF804
#define STM32_OB_DATA1		0x1FFFF806
#define STM32_OB_WRP0		0x1FFFF808
#define STM32_OB_WRP1		0x1FFFF80A
#define STM32_OB_WRP2		0x1FFFF80C
#define STM32_OB_WRP3		0x1FFFF80E

/* FLASH_CR register bits */

#define FLASH_PG		(1<<0)
#define FLASH_PER		(1<<1)
#define FLASH_MER   	(1<<2)
#define FLASH_OPTPG		(1<<4)
#define FLASH_OPTER		(1<<5)
#define FLASH_STRT		(1<<6)
#define FLASH_LOCK		(1<<7)
#define FLASH_OPTWRE	(1<<9)

/* FLASH_SR regsiter bits */

#define FLASH_BSY		(1<<0)
#define FLASH_PGERR   	(1<<2)
#define FLASH_WRPRTERR	(1<<4)
#define FLASH_EOP		(1<<5)

/* STM32_FLASH_OBR bit definitions (reading) */

#define OPT_ERROR		0
#define OPT_READOUT		1
#define OPT_RDWDGSW		2
#define OPT_RDRSTSTOP	3
#define OPT_RDRSTSTDBY	4

/* register unlock keys */

#define KEY1			0x45670123
#define KEY2			0xCDEF89AB

typedef struct stm32x_mem_layout_s {
	u32 sector_start;
	u32 sector_size;
} stm32x_mem_layout_t;

#endif /* STM32X_H */
