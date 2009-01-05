/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by John McCarthy                                   *
 *   jgmcc@magma.ca                                                        *
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
#ifndef PIC32MX_H
#define PIC32MX_H

#include "flash.h"
#include "target.h"

typedef struct pic32mx_flash_bank_s
{
	working_area_t *write_algorithm;
	int devid;
	int ppage_size;
	int probed;
} pic32mx_flash_bank_t;

#define PIC32MX_MANUF_ID	0x029

/* pic32mx memory locations */

#define PIC32MX_KUSEG_PGM_FLASH		0x7D000000
#define PIC32MX_KUSEG_RAM			0x7F000000

#define PIC32MX_KSEG0_RAM			0x80000000
#define PIC32MX_KSEG0_PGM_FLASH		0x9D000000
#define PIC32MX_KSEG0_BOOT_FLASH	0x9FC00000

#define PIC32MX_KSEG1_RAM			0xA0000000
#define PIC32MX_KSEG1_PGM_FLASH		0xBD000000
#define PIC32MX_KSEG1_PERIPHERAL	0xBF800000
#define PIC32MX_KSEG1_BOOT_FLASH	0xBFC00000

#define PIC32MX_PHYS_RAM			0x00000000
#define PIC32MX_PHYS_PGM_FLASH		0x1D000000
#define PIC32MX_PHYS_PERIPHERALS	0x1F800000
#define PIC32MX_PHYS_BOOT_FLASH		0x1FC00000

/*
 * Translate Virtual and Physical addresses.
 * Note: These macros only work for KSEG0/KSEG1 addresses.
 */
#define KS1Virt2Phys(vaddr)			((vaddr)-0xA0000000)
#define Phys2KS1Virt(paddr)			((paddr)+0xA0000000)
#define KS0Virt2Phys(vaddr)			((vaddr)-0x80000000)
#define Phys2KS0Virt(paddr)			((paddr)+0x80000000)

/* pic32mx configuration register locations */

#define PIC32MX_DEVCFG0		0xBFC02FFC
#define PIC32MX_DEVCFG1		0xBFC02FF8
#define PIC32MX_DEVCFG2		0xBFC02FF4
#define PIC32MX_DEVCFG3		0XBFC02FF0
#define PIC32MX_DEVID		0xBF80F220

/* pic32mx flash controller register locations */

#define PIC32MX_NVMCON		0xBF80F400
#define PIC32MX_NVMCONCLR	0xBF80F404
#define PIC32MX_NVMCONSET	0xBF80F408
#define PIC32MX_NVMCONINV	0xBF80F40C
#define NVMCON_NVMWR		(1<<15)
#define NVMCON_NVMWREN		(1<<14)
#define NVMCON_NVMERR		(1<<13)
#define NVMCON_LVDERR		(1<<12)
#define NVMCON_LVDSTAT		(1<<11)
#define NVMCON_OP_PFM_ERASE		0x5
#define NVMCON_OP_PAGE_ERASE	0x4
#define NVMCON_OP_ROW_PROG		0x3
#define NVMCON_OP_WORD_PROG		0x1
#define NVMCON_OP_NOP			0x0

#define PIC32MX_NVMKEY		0xBF80F410
#define PIC32MX_NVMADDR		0xBF80F420
#define PIC32MX_NVMADDRCLR	0xBF80F424
#define PIC32MX_NVMADDRSET	0xBF80F428
#define PIC32MX_NVMADDRINV	0xBF80F42C
#define PIC32MX_NVMDATA		0xBF80F430
#define PIC32MX_NVMSRCADDR	0xBF80F440

/* flash unlock keys */

#define NVMKEY1			0xAA996655
#define NVMKEY2			0x556699AA

typedef struct pic32mx_mem_layout_s {
	u32 sector_start;
	u32 sector_size;
} pic32mx_mem_layout_t;

#endif /* PIC32MX_H */

