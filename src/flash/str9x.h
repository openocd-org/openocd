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
#ifndef STR9X_H
#define STR9X_H

#include "flash.h"
#include "target.h"

typedef struct str9x_flash_bank_s
{
	u32 *sector_bits;
	int variant;
	int bank1;
	working_area_t *write_algorithm;
} str9x_flash_bank_t;

enum str9x_status_codes
{
	STR9X_CMD_SUCCESS = 0,
	STR9X_INVALID_COMMAND = 1,
	STR9X_SRC_ADDR_ERROR = 2,
	STR9X_DST_ADDR_ERROR = 3,
	STR9X_SRC_ADDR_NOT_MAPPED = 4,
	STR9X_DST_ADDR_NOT_MAPPED = 5,
	STR9X_COUNT_ERROR = 6,
	STR9X_INVALID_SECTOR = 7,
	STR9X_SECTOR_NOT_BLANK = 8,
	STR9X_SECTOR_NOT_PREPARED = 9,
	STR9X_COMPARE_ERROR = 10,
	STR9X_BUSY = 11
};

/* Flash registers */

#define FLASH_BBSR		0x54000000		/* Boot Bank Size Register                */
#define FLASH_NBBSR		0x54000004		/* Non-Boot Bank Size Register            */
#define FLASH_BBADR		0x5400000C		/* Boot Bank Base Address Register        */
#define FLASH_NBBADR	0x54000010		/* Non-Boot Bank Base Address Register    */
#define FLASH_CR		0x54000018		/* Control Register                       */
#define FLASH_SR		0x5400001C		/* Status Register                        */
#define FLASH_BCE5ADDR	0x54000020		/* BC Fifth Entry Target Address Register */

#endif /* STR9X_H */
