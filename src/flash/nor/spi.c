/***************************************************************************
 *   Copyright (C) 2012 by George Harris                                   *
 *   george@luminairecoffee.com                                            *
 *                                                                         *
 *   Copyright (C) 2010 by Antonio Borneo                                  *
 *   borneo.antonio@gmail.com                                              *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <jtag/jtag.h>

 /* Shared table of known SPI flash devices for SPI-based flash drivers. Taken
  * from device datasheets and Linux SPI flash drivers. */
const struct flash_device flash_devices[] = {
	/* name, erase_cmd, chip_erase_cmd, device_id, pagesize, sectorsize, size_in_bytes */
	FLASH_ID("st m25p05",      0xd8, 0xc7, 0x00102020, 0x80,  0x8000,  0x10000),
	FLASH_ID("st m25p10",      0xd8, 0xc7, 0x00112020, 0x80,  0x8000,  0x20000),
	FLASH_ID("st m25p20",      0xd8, 0xc7, 0x00122020, 0x100, 0x10000, 0x40000),
	FLASH_ID("st m25p40",      0xd8, 0xc7, 0x00132020, 0x100, 0x10000, 0x80000),
	FLASH_ID("st m25p80",      0xd8, 0xc7, 0x00142020, 0x100, 0x10000, 0x100000),
	FLASH_ID("st m25p16",      0xd8, 0xc7, 0x00152020, 0x100, 0x10000, 0x200000),
	FLASH_ID("st m25p32",      0xd8, 0xc7, 0x00162020, 0x100, 0x10000, 0x400000),
	FLASH_ID("st m25p64",      0xd8, 0xc7, 0x00172020, 0x100, 0x10000, 0x800000),
	FLASH_ID("st m25p128",     0xd8, 0xc7, 0x00182020, 0x100, 0x40000, 0x1000000),
	FLASH_ID("st m45pe10",     0xd8, 0xd8, 0x00114020, 0x100, 0x10000, 0x20000),
	FLASH_ID("st m45pe20",     0xd8, 0xd8, 0x00124020, 0x100, 0x10000, 0x40000),
	FLASH_ID("st m45pe40",     0xd8, 0xd8, 0x00134020, 0x100, 0x10000, 0x80000),
	FLASH_ID("st m45pe80",     0xd8, 0xd8, 0x00144020, 0x100, 0x10000, 0x100000),
	FLASH_ID("sp s25fl004",    0xd8, 0xc7, 0x00120201, 0x100, 0x10000, 0x80000),
	FLASH_ID("sp s25fl008",    0xd8, 0xc7, 0x00130201, 0x100, 0x10000, 0x100000),
	FLASH_ID("sp s25fl016",    0xd8, 0xc7, 0x00140201, 0x100, 0x10000, 0x200000),
	FLASH_ID("sp s25fl032",    0xd8, 0xc7, 0x00150201, 0x100, 0x10000, 0x400000),
	FLASH_ID("sp s25fl064",    0xd8, 0xc7, 0x00160201, 0x100, 0x10000, 0x800000),
	FLASH_ID("sp s25fl128",    0xd8, 0xC7, 0x00182001, 0x100, 0x10000, 0x1000000),
	FLASH_ID("sp s25fl256",    0xd8, 0xC7, 0x00190201, 0x100, 0x10000, 0x2000000),
	FLASH_ID("atmel 25f512",   0x52, 0xc7, 0x0065001f, 0x80,  0x8000,  0x10000),
	FLASH_ID("atmel 25f1024",  0x52, 0x62, 0x0060001f, 0x100, 0x8000,  0x20000),
	FLASH_ID("atmel 25f2048",  0x52, 0x62, 0x0063001f, 0x100, 0x10000, 0x40000),
	FLASH_ID("atmel 25f4096",  0x52, 0x62, 0x0064001f, 0x100, 0x10000, 0x80000),
	FLASH_ID("atmel 25fs040",  0xd7, 0xc7, 0x0004661f, 0x100, 0x10000, 0x80000),
	FLASH_ID("mac 25l512",     0xd8, 0xc7, 0x001020c2, 0x010, 0x10000, 0x10000),
	FLASH_ID("mac 25l1005",    0xd8, 0xd8, 0x001120c2, 0x010, 0x10000, 0x20000),
	FLASH_ID("mac 25l2005",    0xd8, 0xc7, 0x001220c2, 0x010, 0x10000, 0x40000),
	FLASH_ID("mac 25l4005",    0xd8, 0xc7, 0x001320c2, 0x010, 0x10000, 0x80000),
	FLASH_ID("mac 25l8005",    0xd8, 0xc7, 0x001420c2, 0x010, 0x10000, 0x100000),
	FLASH_ID("mac 25l1605",    0xd8, 0xc7, 0x001520c2, 0x100, 0x10000, 0x200000),
	FLASH_ID("mac 25l3205",    0xd8, 0xc7, 0x001620c2, 0x100, 0x10000, 0x400000),
	FLASH_ID("mac 25l6405",    0xd8, 0xc7, 0x001720c2, 0x100, 0x10000, 0x800000),
	FLASH_ID("mcr n25q064",    0xd8, 0xc7, 0x0017ba20, 0x100, 0x10000, 0x800000),
	FLASH_ID("win w25q80bv",   0xd8, 0xc7, 0x001440ef, 0x100, 0x10000, 0x100000),
	FLASH_ID("win w25q32dw",   0xd8, 0xc7, 0x001660ef, 0x100, 0x10000, 0x400000),
	FLASH_ID("win w25q64cv",   0xd8, 0xc7, 0x001740ef, 0x100, 0x10000, 0x800000),
	FLASH_ID(NULL,             0,    0,	   0,          0,     0,       0)
};
