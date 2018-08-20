/***************************************************************************
 *   Copyright (C) 2007 by Pavel Chromy                                    *
 *   chromy@asix.cz                                                        *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/
#ifndef samflashH
#define samflashH

#include "platform.h"

#define FLASH_AREA_ADDR 0x100000

#define FLASH_STAT_OK 0
#define FLASH_STAT_PROGE 1
#define FLASH_STAT_LOCKE 2
#define FLASH_STAT_VERIFE 3
#define FLASH_STAT_INITE 4

extern unsigned int flash_page_count;
extern unsigned int flash_page_size; /* words */

/* detect chip and set loader parameters */
int flash_init(void);

/* program single flash page */
int flash_page_program(uint32 *data, int page_num);

/* erase whole chip */
int flash_erase_all(void);

/* verify written data */
int flash_verify(uint32 adr, unsigned int len, uint8 *src);

#endif
