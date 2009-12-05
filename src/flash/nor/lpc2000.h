/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   LPC1700 support Copyright (C) 2009 by Audrius Urmanavicius            *
 *   didele.deze@gmail.com                                                 *
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

typedef enum
{
	lpc2000_v1,
	lpc2000_v2,
	lpc1700
} lpc2000_variant;

struct lpc2000_flash_bank
{
	lpc2000_variant variant;
	struct working_area *iap_working_area;
	uint32_t cclk;
	int cmd51_dst_boundary;
	int cmd51_can_256b;
	int cmd51_can_8192b;
	int calc_checksum;
	uint32_t cmd51_max_buffer;
	int checksum_vector;
};

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
	LPC2000_BUSY = 11,
	LPC2000_PARAM_ERROR = 12,
	LPC2000_ADDR_ERROR = 13,
	LPC2000_ADDR_NOT_MAPPED = 14,
	LPC2000_CMD_NOT_LOCKED = 15,
	LPC2000_INVALID_CODE = 16,
	LPC2000_INVALID_BAUD_RATE = 17,
	LPC2000_INVALID_STOP_BIT = 18,
	LPC2000_CRP_ENABLED = 19

};

#endif /* LPC2000_H */
