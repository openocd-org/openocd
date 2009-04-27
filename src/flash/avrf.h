/***************************************************************************
 *   Copyright (C) 2009 by Simon Qian                                      *
 *   SimonQian@SimonQian.com                                               *
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
#ifndef AVRF_H
#define AVRF_H

typedef struct avrf_type_s
{
	char name[15];
	u16 chip_id;
	int flash_page_size;
	int flash_page_num;
	int eeprom_page_size;
	int eeprom_page_num;
} avrf_type_t;

typedef struct avrf_flash_bank_s
{
	int ppage_size;
	int probed;
} avrf_flash_bank_t;

#endif /* AVRF_H */
