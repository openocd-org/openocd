/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundinªmlu.mine.nu                                                    *
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

#include "flash.h"
#include "target.h"

typedef struct at91sam7_flash_bank_s
{
	struct target_s *target;
	u32 working_area;
	u32 working_area_size;

	/* chip id register */
	u32 cidr;
	u16 cidr_ext;
	u16 cidr_nvptyp;
	u16 cidr_arch;
	u16 cidr_sramsiz;
	u16 cidr_nvpsiz;
	u16 cidr_nvpsiz2;
	u16 cidr_eproc;
	u16 cidr_version;
	char * target_name;

	/* flash geometry */
	u16 num_pages;
	u16 pagesize;
	u16 pages_in_lockregion;
	u8 num_erase_regions;
	u32 *erase_region_info;

	/* nv memory bits */
	u16 num_lockbits;
	u16 lockbits;
	u16 num_nvmbits;
	u16 nvmbits;
	u8  securitybit;
	u8  flashmode;         /* 0: not init, 1: fmcn for nvbits (1uS), 2: fmcn for flash (1.5uS) */

	/* main clock status */
	u8  mck_valid;
	u32 mck_freq;
	
} at91sam7_flash_bank_t;

/* AT91SAM7 control registers */
#define DBGU_CIDR 0xFFFFF240
#define CKGR_MCFR 0xFFFFFC24
#define CKGR_MCFR_MAINRDY  0x10000
#define CKGR_PLLR 0xFFFFFC2c
#define CKGR_PLLR_DIV 0xff
#define CKGR_PLLR_MUL 0x07ff0000
#define PMC_MCKR  0xFFFFFC30
#define PMC_MCKR_CSS  0x03
#define PMC_MCKR_PRES 0x1c
#define MC_FMR	0xFFFFFF60
#define MC_FCR	0xFFFFFF64
#define MC_FSR	0xFFFFFF68

/* Flash Controller Commands */
#define  WP   0x01
#define  SLB  0x02
#define  WPL  0x03
#define  CLB  0x04
#define  EA   0x08
#define  SGPB 0x0B
#define  CGPB 0x0D
#define  SSB  0x0F

/* MC_FSR bit definitions */
#define        MC_FSR_FRDY 1
#define        MC_FSR_EOL 2

/* AT91SAM7 constants */
#define RC_FREQ  32000

/*  FLASH_TIMING_MODES */
#define  FMR_TIMING_NONE    0
#define  FMR_TIMING_NVBITS  1
#define  FMR_TIMING_FLASH   2

#endif /* AT91SAM7_H */
