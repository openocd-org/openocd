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
#ifndef CFI_H
#define CFI_H

#include "flash.h"
#include "target.h"

typedef struct cfi_flash_bank_s
{
	working_area_t *write_algorithm;
	

	int x16_as_x8;
	int jedec_probe;
	int not_cfi;
	int probed;

	u16 manufacturer;
	u16 device_id;

	char qry[3];

	/* identification string */
	u16 pri_id;
	u16 pri_addr;
	u16 alt_id;
	u16 alt_addr;

	/* device-system interface */
	u8 vcc_min;
	u8 vcc_max;
	u8 vpp_min;
	u8 vpp_max;
	u8 word_write_timeout_typ;
	u8 buf_write_timeout_typ;
	u8 block_erase_timeout_typ;
	u8 chip_erase_timeout_typ;
	u8 word_write_timeout_max;
	u8 buf_write_timeout_max;
	u8 block_erase_timeout_max;
	u8 chip_erase_timeout_max;

	/* flash geometry */
	u8 dev_size;
	u16 interface_desc;
	u16 max_buf_write_size;
	u8 num_erase_regions;
	u32 *erase_region_info;

	void *pri_ext;
	void *alt_ext;
} cfi_flash_bank_t;

/* Intel primary extended query table
 * as defined for the Advanced+ Boot Block Flash Memory (C3)
 * and used by the linux kernel cfi driver (as of 2.6.14)
 */
typedef struct cfi_intel_pri_ext_s
{
	char pri[3];
	u8 major_version;
	u8 minor_version;
	u32 feature_support;
	u8 suspend_cmd_support;
	u16 blk_status_reg_mask;
	u8 vcc_optimal;
	u8 vpp_optimal;
	u8 num_protection_fields;
	u16 prot_reg_addr;
	u8 fact_prot_reg_size;
	u8 user_prot_reg_size;
	u8 extra[0];
} cfi_intel_pri_ext_t;

/* Spansion primary extended query table as defined for and used by
 * the linux kernel cfi driver (as of 2.6.15)
 */
typedef struct cfi_spansion_pri_ext_s
{
	u8  pri[3];
	u8  major_version;
	u8  minor_version;
	u8  SiliconRevision; /* bits 1-0: Address Sensitive Unlock */
	u8  EraseSuspend;
	u8  BlkProt;
	u8  TmpBlkUnprotect;
	u8  BlkProtUnprot;
	u8  SimultaneousOps;
	u8  BurstMode;
	u8  PageMode;
	u8  VppMin;
	u8  VppMax;
	u8  TopBottom;
	int _reversed_geometry;
	u32 _unlock1;
	u32 _unlock2;
} cfi_spansion_pri_ext_t;

/* Atmel primary extended query table as defined for and used by
 * the linux kernel cfi driver (as of 2.6.20+)
 */
typedef struct cfi_atmel_pri_ext_s
{
	u8 pri[3];
	u8 major_version;
	u8 minor_version;
	u8 features;
	u8 bottom_boot;
	u8 burst_mode;
	u8 page_mode;
} cfi_atmel_pri_ext_t;

enum {
	CFI_UNLOCK_555_2AA,
	CFI_UNLOCK_5555_2AAA,
};

typedef struct cfi_unlock_addresses_s
{
	u32 unlock1;
	u32 unlock2;
} cfi_unlock_addresses_t;

typedef struct cfi_fixup_s
{
	u16 mfr;
	u16 id;
	void (*fixup)(flash_bank_t *flash, void *param);
	void *param;
} cfi_fixup_t;

#define CFI_MFR_AMD		0x0001
#define CFI_MFR_FUJITSU	0x0004
#define CFI_MFR_ATMEL	0x001F
#define CFI_MFR_ST		0x0020	/* STMicroelectronics */
#define CFI_MFR_AMIC	0x0037
#define CFI_MFR_SST		0x00BF
#define CFI_MFR_MX		0x00C2

#define CFI_MFR_ANY		0xffff
#define CFI_ID_ANY		0xffff

#endif /* CFI_H */
