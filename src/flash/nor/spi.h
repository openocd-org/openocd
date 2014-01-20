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

/* data structure to maintain flash ids from different vendors */
struct flash_device {
	char *name;
	uint8_t erase_cmd;
	uint8_t chip_erase_cmd;
	uint32_t device_id;
	uint32_t pagesize;
	unsigned long sectorsize;
	unsigned long size_in_bytes;
};

#define FLASH_ID(n, es, ces, id, psize, ssize, size) \
{	                        \
	.name = n,              \
	.erase_cmd = es,        \
	.chip_erase_cmd = ces,  \
	.device_id = id,        \
	.pagesize = psize,      \
	.sectorsize = ssize,    \
	.size_in_bytes = size   \
}

extern const struct flash_device flash_devices[];

/* fields in SPI flash status register */
#define SPIFLASH_BSY_BIT		0x00000001 /* WIP Bit of SPI SR on SMI SR */
#define SPIFLASH_WE_BIT			0x00000002 /* WEL Bit of SPI SR on SMI SR */

/* SPI Flash Commands */
#define SPIFLASH_READ_ID		0x9F /* Read Flash Identification */
#define SPIFLASH_READ_STATUS	0x05 /* Read Status Register */
#define SPIFLASH_WRITE_ENABLE	0x06 /* Write Enable */
#define SPIFLASH_PAGE_PROGRAM	0x02 /* Page Program */
#define SPIFLASH_FAST_READ		0x0B /* Fast Read */
#define SPIFLASH_READ			0x03 /* Normal Read */
