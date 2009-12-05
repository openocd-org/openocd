/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2006 by Gheorghe Guran (atlas)                          *
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

#ifndef AT91SAM7_H
#define AT91SAM7_H

struct at91sam7_flash_bank
{
	/* chip id register */
	uint32_t cidr;
	uint16_t cidr_ext;
	uint16_t cidr_nvptyp;
	uint16_t cidr_arch;
	uint16_t cidr_sramsiz;
	uint16_t cidr_nvpsiz;
	uint16_t cidr_nvpsiz2;
	uint16_t cidr_eproc;
	uint16_t cidr_version;
	char *target_name;

	/* flash auto-detection */
	uint8_t  flash_autodetection;

	/* flash geometry */
	uint16_t pages_per_sector;
	uint16_t pagesize;
	uint16_t pages_in_lockregion;

	/* nv memory bits */
	uint16_t num_lockbits_on;
	uint16_t lockbits;
	uint16_t num_nvmbits;
	uint16_t num_nvmbits_on;
	uint16_t nvmbits;
	uint8_t  securitybit;

	/* 0: not init
	 * 1: fmcn for nvbits (1uS)
	 * 2: fmcn for flash (1.5uS) */
	uint8_t  flashmode;

	/* main clock status */
	uint8_t  mck_valid;
	uint32_t mck_freq;

	/* external clock frequency */
	uint32_t ext_freq;

};


/* AT91SAM7 control registers */
#define DBGU_CIDR			0xFFFFF240
#define CKGR_MCFR			0xFFFFFC24
#define CKGR_MOR			0xFFFFFC20
#define CKGR_MCFR_MAINRDY	0x10000
#define CKGR_PLLR			0xFFFFFC2c
#define CKGR_PLLR_DIV		0xff
#define CKGR_PLLR_MUL		0x07ff0000
#define PMC_MCKR			0xFFFFFC30
#define PMC_MCKR_CSS		0x03
#define PMC_MCKR_PRES		0x1c

/* Flash Controller Commands */
#define WP		0x01
#define SLB		0x02
#define WPL		0x03
#define CLB		0x04
#define EA		0x08
#define SGPB	0x0B
#define CGPB	0x0D
#define SSB		0x0F

/* MC_FSR bit definitions */
#define MC_FSR_FRDY			1
#define MC_FSR_EOL			2

/* AT91SAM7 constants */
#define RC_FREQ				32000

/* Flash timing modes */
#define FMR_TIMING_NONE		0
#define FMR_TIMING_NVBITS	1
#define FMR_TIMING_FLASH	2

/* Flash size constants */
#define FLASH_SIZE_8KB		1
#define FLASH_SIZE_16KB		2
#define FLASH_SIZE_32KB		3
#define FLASH_SIZE_64KB		5
#define FLASH_SIZE_128KB	7
#define FLASH_SIZE_256KB	9
#define FLASH_SIZE_512KB	10
#define FLASH_SIZE_1024KB	12
#define FLASH_SIZE_2048KB	14

#endif /* AT91SAM7_H */
