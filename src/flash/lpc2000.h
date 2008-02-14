/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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
#ifndef LPC2000_H
#define LPC2000_H

#include "flash.h"
#include "target.h"

typedef struct lpc2000_flash_bank_s
{
	int variant;
	struct working_area_s *iap_working_area;
	u32 cclk;
	int cmd51_dst_boundary;
	int cmd51_can_256b;
	int cmd51_can_8192b;
	int calc_checksum;
	int cmd51_max_buffer;
} lpc2000_flash_bank_t;

enum lpc2000_status_codes
{
	LPC2000_CMD_SUCCESS = 0,
	LPC2000_INVALID_COMMAND = 1,
	LPC2000_SRC_ADDR_ERROR = 2,
	LPC2000_DST_ADDR_ERROR = 3,
	LPC2000_SRC_ADDR_NOT_MAPPED = 4,
	LPC2000_DST_ADDR_NOT_MAPPED = 5,
	LPC2000_COUNT_ERROR = 6,
	LPC2000_INVALID_SECTOR = 7,
	LPC2000_SECTOR_NOT_BLANK = 8,
	LPC2000_SECTOR_NOT_PREPARED = 9,
	LPC2000_COMPARE_ERROR = 10,
	LPC2000_BUSY = 11
};

#endif /* LPC2000_H */
