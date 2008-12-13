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
#ifndef STR7X_H
#define STR7X_H

#include "flash.h"
#include "target.h"

typedef struct str7x_flash_bank_s
{
	u32 *sector_bits;
	u32 disable_bit;
	u32 busy_bits;
	u32 register_base;
	working_area_t *write_algorithm;
} str7x_flash_bank_t;

enum str7x_status_codes
{
	STR7X_CMD_SUCCESS = 0,
	STR7X_INVALID_COMMAND = 1,
	STR7X_SRC_ADDR_ERROR = 2,
	STR7X_DST_ADDR_ERROR = 3,
	STR7X_SRC_ADDR_NOT_MAPPED = 4,
	STR7X_DST_ADDR_NOT_MAPPED = 5,
	STR7X_COUNT_ERROR = 6,
	STR7X_INVALID_SECTOR = 7,
	STR7X_SECTOR_NOT_BLANK = 8,
	STR7X_SECTOR_NOT_PREPARED = 9,
	STR7X_COMPARE_ERROR = 10,
	STR7X_BUSY = 11
};

/*  Flash registers */

#define FLASH_CR0		0x00000000
#define FLASH_CR1		0x00000004
#define FLASH_DR0		0x00000008
#define FLASH_DR1		0x0000000C
#define FLASH_AR		0x00000010
#define FLASH_ER		0x00000014
#define FLASH_NVWPAR	0x0000DFB0
#define FLASH_NVAPR0 	0x0000DFB8
#define FLASH_NVAPR1 	0x0000DFBC

/* FLASH_CR0 register bits */

#define FLASH_WMS		0x80000000
#define FLASH_SUSP		0x40000000
#define FLASH_WPG   	0x20000000
#define FLASH_DWPG		0x10000000
#define FLASH_SER		0x08000000
#define FLASH_SPR		0x01000000
#define FLASH_BER		0x04000000
#define FLASH_MER		0x02000000
#define FLASH_LOCK		0x00000010
#define FLASH_BSYA1		0x00000004
#define FLASH_BSYA0		0x00000002

/* FLASH_CR1 regsiter bits */

#define FLASH_B1S		0x02000000
#define FLASH_B0S		0x01000000
#define FLASH_B1F1		0x00020000
#define FLASH_B1F0		0x00010000
#define FLASH_B0F7		0x00000080
#define FLASH_B0F6		0x00000040
#define FLASH_B0F5		0x00000020
#define FLASH_B0F4		0x00000010
#define FLASH_B0F3		0x00000008
#define FLASH_B0F2		0x00000004
#define FLASH_B0F1		0x00000002
#define FLASH_B0F0		0x00000001

/* FLASH_ER register bits */

#define FLASH_WPF		0x00000100
#define FLASH_RESER		0x00000080
#define FLASH_SEQER		0x00000040
#define FLASH_10ER		0x00000008
#define FLASH_PGER		0x00000004
#define FLASH_ERER		0x00000002
#define FLASH_ERR		0x00000001

typedef struct str7x_mem_layout_s {
	u32 sector_start;
	u32 sector_size;
	u32 sector_bit;
} str7x_mem_layout_t;

#endif /* STR7X_H */
