/***************************************************************************
 *   Copyright (C) 2015 by Uwe Bonnes                                      *
 *   bon@elektron.ikp.physik.tu-darmstadt.de                               *
 *
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

#ifndef OPENOCD_FLASH_NOR_STM32L4X
#define OPENOCD_FLASH_NOR_STM32L4X

/* IMPORTANT: this file is included by stm32l4x driver and flashloader,
 * so please when changing this file, do not forget to check the flashloader */

/* FIXME: #include "helper/bits.h" cause build errors when compiling
 * the flashloader, for now just redefine the needed 'BIT 'macro */

#ifndef BIT
#define BIT(nr)                 (1UL << (nr))
#endif

/* FLASH_CR register bits */
#define FLASH_PG				BIT(0)
#define FLASH_PER				BIT(1)
#define FLASH_MER1				BIT(2)
#define FLASH_PAGE_SHIFT		3
#define FLASH_BKER				BIT(11)
#define FLASH_BKER_G0			BIT(13)
#define FLASH_MER2				BIT(15)
#define FLASH_STRT				BIT(16)
#define FLASH_OPTSTRT			BIT(17)
#define FLASH_EOPIE				BIT(24)
#define FLASH_ERRIE				BIT(25)
#define FLASH_OBL_LAUNCH		BIT(27)
#define FLASH_OPTLOCK			BIT(30)
#define FLASH_LOCK				BIT(31)

/* FLASH_SR register bits */
#define FLASH_BSY				BIT(16)
#define FLASH_BSY2				BIT(17)

/* Fast programming not used => related errors not used*/
#define FLASH_PGSERR			BIT(7) /* Programming sequence error */
#define FLASH_SIZERR			BIT(6) /* Size error */
#define FLASH_PGAERR			BIT(5) /* Programming alignment error */
#define FLASH_WRPERR			BIT(4) /* Write protection error */
#define FLASH_PROGERR			BIT(3) /* Programming error */
#define FLASH_OPERR				BIT(1) /* Operation error */
#define FLASH_EOP				BIT(0) /* End of operation */
#define FLASH_ERROR				(FLASH_PGSERR | FLASH_SIZERR | FLASH_PGAERR | \
								FLASH_WRPERR | FLASH_PROGERR | FLASH_OPERR)

/* register unlock keys */
#define KEY1					0x45670123
#define KEY2					0xCDEF89AB

/* option register unlock key */
#define OPTKEY1					0x08192A3B
#define OPTKEY2					0x4C5D6E7F

/* FLASH_OPTR register bits */
#define FLASH_RDP_MASK			0xFF
#define FLASH_TZEN				BIT(31)

/* FLASH secure block based bank 1/2 register offsets */
#define FLASH_SECBB1(X) (0x80 + 4 * (X - 1))
#define FLASH_SECBB2(X) (0xA0 + 4 * (X - 1))

#define FLASH_SECBB_SECURE      0xFFFFFFFF
#define FLASH_SECBB_NON_SECURE  0

/* other registers */
#define DBGMCU_IDCODE_G0		0x40015800
#define DBGMCU_IDCODE_L4_G4		0xE0042000
#define DBGMCU_IDCODE_L5		0xE0044000
#define UID64_DEVNUM			0x1FFF7580
#define UID64_IDS				0x1FFF7584
#define UID64_IDS_STM32WL		0x0080E115

#define STM32_FLASH_BANK_BASE	0x08000000
#define STM32_FLASH_S_BANK_BASE	0x0C000000

#define STM32L5_REGS_SEC_OFFSET 0x10000000

#endif
