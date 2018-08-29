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
#include "samflash.h"


unsigned int flash_page_count = 1024;
unsigned int flash_page_size = 256;

/* pages per lock bit */
unsigned int flash_lock_pages = 1024/16;


/* detect chip and set loader parameters */
int flash_init(void)
{
	unsigned int nvpsiz;

	nvpsiz = (inr(DBGU_CIDR) >> 8)&0xf;

	switch (nvpsiz) {
		case 3:
			/* AT91SAM7x32 */
			flash_page_count = 256;
			flash_page_size = 128;
			flash_lock_pages = 256/8;
			break;
		case 5:
			/* AT91SAM7x64 */
			flash_page_count = 512;
			flash_page_size = 128;
			flash_lock_pages = 512/16;
			break;
		case 7:
			/* AT91SAM7x128*/
			flash_page_count = 512;
			flash_page_size = 256;
			flash_lock_pages = 512/8;
			break;
		case 9:
			/* AT91SAM7x256 */
			flash_page_count = 1024;
			flash_page_size = 256;
			flash_lock_pages = 1024/16;
			break;
		case 10:
			/* AT91SAM7x512 */
			flash_page_count = 2048;
			flash_page_size = 256;
			flash_lock_pages = 2048/32;
			break;
		default:
			return FLASH_STAT_INITE;
	}
	return FLASH_STAT_OK;
}


/* program single flash page */
int flash_page_program(uint32 *data, int page_num)
{
	int i;
	int efc_ofs;

	uint32 *flash_ptr;
	uint32 *data_ptr;

	/* select proper controller */
	if (page_num >= 1024) efc_ofs = 0x10;
	else efc_ofs = 0;

	/* wait until FLASH is ready, just for sure */
	while ((inr(MC_FSR + efc_ofs)&MC_FRDY) == 0);

	/* calculate page address, only lower 8 bits are used to address the latch,
		 but the upper part of address is needed for writing to proper EFC */
	flash_ptr = (uint32 *)(FLASH_AREA_ADDR + (page_num*flash_page_size));
	data_ptr = data;

	/* copy data to latch */
	for (i = flash_page_size/4; i; i--) {
		/* we do not use memcpy to be sure that only 32 bit access is used */
		*(flash_ptr++)=*(data_ptr++);
	}

	/* page number and page write command to FCR */
	outr(MC_FCR + efc_ofs, ((page_num&0x3ff) << 8) | MC_KEY | MC_FCMD_WP);

	/* wait until it's done */
	while ((inr(MC_FSR + efc_ofs)&MC_FRDY) == 0);

	/* check for errors */
	if ((inr(MC_FSR + efc_ofs)&MC_PROGE)) return FLASH_STAT_PROGE;
	if ((inr(MC_FSR + efc_ofs)&MC_LOCKE)) return FLASH_STAT_LOCKE;

#if 0
	/* verify written data */
	flash_ptr = (uint32 *)(FLASH_AREA_ADDR + (page_num*flash_page_size));
	data_ptr = data;

	for (i = flash_page_size/4; i; i--) {
		if (*(flash_ptr++)!=*(data_ptr++)) return FLASH_STAT_VERIFE;
	}
#endif

	return FLASH_STAT_OK;
}


int flash_erase_plane(int efc_ofs)
{
	unsigned int lockbits;
	int page_num;

	page_num = 0;
	lockbits = inr(MC_FSR + efc_ofs) >> 16;
	while (lockbits) {
		if (lockbits&1) {

			/* wait until FLASH is ready, just for sure */
			while ((inr(MC_FSR + efc_ofs)&MC_FRDY) == 0);

			outr(MC_FCR + efc_ofs, ((page_num&0x3ff) << 8) | 0x5a000004);

			/* wait until it's done */
			while ((inr(MC_FSR + efc_ofs)&MC_FRDY) == 0);

			/* check for errors */
			if ((inr(MC_FSR + efc_ofs)&MC_PROGE)) return FLASH_STAT_PROGE;
			if ((inr(MC_FSR + efc_ofs)&MC_LOCKE)) return FLASH_STAT_LOCKE;

		}
		if ((page_num += flash_lock_pages) > flash_page_count) break;
		lockbits>>=1;
	}

	/* wait until FLASH is ready, just for sure */
	while ((inr(MC_FSR + efc_ofs)&MC_FRDY) == 0);

	/* erase all command to FCR */
	outr(MC_FCR + efc_ofs, 0x5a000008);

	/* wait until it's done */
	while ((inr(MC_FSR + efc_ofs)&MC_FRDY) == 0);

	/* check for errors */
	if ((inr(MC_FSR + efc_ofs)&MC_PROGE)) return FLASH_STAT_PROGE;
	if ((inr(MC_FSR + efc_ofs)&MC_LOCKE)) return FLASH_STAT_LOCKE;

	/* set no erase before programming */
	outr(MC_FMR + efc_ofs, inr(MC_FMR + efc_ofs) | 0x80);

	return FLASH_STAT_OK;
}


/* erase whole chip */
int flash_erase_all(void)
{
	int result;

	if ((result = flash_erase_plane(0)) != FLASH_STAT_OK) return result;

	/* the second flash controller, if any */
	if (flash_page_count > 1024) result = flash_erase_plane(0x10);

	return result;
}


int flash_verify(uint32 adr, unsigned int len, uint8 *src)
{
	unsigned char *flash_ptr;

	flash_ptr = (uint8 *)FLASH_AREA_ADDR + adr;
	for (;len; len--) {
		if (*(flash_ptr++)!=*(src++)) return FLASH_STAT_VERIFE;
	}
	return FLASH_STAT_OK;
}
