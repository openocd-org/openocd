/***************************************************************************
 *   Copyright (C) 2009 by Alexei Babich                                   *
 *   Rezonans plc., Chelyabinsk, Russia                                    *
 *   impatt@mail.ru                                                        *
 *                                                                         *
 *   Copyright (C) 2011 by Erik Ahlen                                      *
 *   Avalon Innovation, Sweden                                             *
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

#ifndef OPENOCD_FLASH_NAND_MXC_H
#define OPENOCD_FLASH_NAND_MXC_H

/*
 * Freescale iMX OpenOCD NAND Flash controller support.
 * based on Freescale iMX2* and iMX3* OpenOCD NAND Flash controller support.
 *
 * Many thanks to Ben Dooks for writing s3c24xx driver.
 */

#define		MXC_NF_BUFSIZ				(mxc_nf_info->mxc_regs_addr + 0x00)
#define		MXC_NF_BUFADDR				(mxc_nf_info->mxc_regs_addr + 0x04)
#define		MXC_NF_FADDR				(mxc_nf_info->mxc_regs_addr + 0x06)
#define		MXC_NF_FCMD					(mxc_nf_info->mxc_regs_addr + 0x08)
#define		MXC_NF_BUFCFG				(mxc_nf_info->mxc_regs_addr + 0x0a)
#define		MXC_NF_ECCSTATUS			(mxc_nf_info->mxc_regs_addr + 0x0c)
#define		MXC_NF_ECCMAINPOS			(mxc_nf_info->mxc_regs_addr + 0x0e)
#define		MXC_NF_V1_ECCSPAREPOS		(mxc_nf_info->mxc_regs_addr + 0x10)
#define		MXC_NF_V2_SPAS				(mxc_nf_info->mxc_regs_addr + 0x10)
#define		MXC_NF_FWP					(mxc_nf_info->mxc_regs_addr + 0x12)
#define		MXC_NF_V1_UNLOCKSTART		(mxc_nf_info->mxc_regs_addr + 0x14)
#define		MXC_NF_V1_UNLOCKEND			(mxc_nf_info->mxc_regs_addr + 0x16)
#define		MXC_NF_V2_UNLOCKSTART0		(mxc_nf_info->mxc_regs_addr + 0x20)
#define		MXC_NF_V2_UNLOCKSTART1		(mxc_nf_info->mxc_regs_addr + 0x24)
#define		MXC_NF_V2_UNLOCKSTART2		(mxc_nf_info->mxc_regs_addr + 0x28)
#define		MXC_NF_V2_UNLOCKSTART3		(mxc_nf_info->mxc_regs_addr + 0x2c)
#define		MXC_NF_V2_UNLOCKEND0		(mxc_nf_info->mxc_regs_addr + 0x22)
#define		MXC_NF_V2_UNLOCKEND1		(mxc_nf_info->mxc_regs_addr + 0x26)
#define		MXC_NF_V2_UNLOCKEND2		(mxc_nf_info->mxc_regs_addr + 0x2a)
#define		MXC_NF_V2_UNLOCKEND3		(mxc_nf_info->mxc_regs_addr + 0x2e)
#define		MXC_NF_FWPSTATUS			(mxc_nf_info->mxc_regs_addr + 0x18)
 /*
  * all bits not marked as self-clearing bit
  */
#define		MXC_NF_CFG1					(mxc_nf_info->mxc_regs_addr + 0x1a)
#define		MXC_NF_CFG2					(mxc_nf_info->mxc_regs_addr + 0x1c)

#define		MXC_NF_MAIN_BUFFER0			(mxc_nf_info->mxc_base_addr + 0x0000)
#define		MXC_NF_MAIN_BUFFER1			(mxc_nf_info->mxc_base_addr + 0x0200)
#define		MXC_NF_MAIN_BUFFER2			(mxc_nf_info->mxc_base_addr + 0x0400)
#define		MXC_NF_MAIN_BUFFER3			(mxc_nf_info->mxc_base_addr + 0x0600)
#define		MXC_NF_V1_SPARE_BUFFER0		(mxc_nf_info->mxc_base_addr + 0x0800)
#define		MXC_NF_V1_SPARE_BUFFER1		(mxc_nf_info->mxc_base_addr + 0x0810)
#define		MXC_NF_V1_SPARE_BUFFER2		(mxc_nf_info->mxc_base_addr + 0x0820)
#define		MXC_NF_V1_SPARE_BUFFER3		(mxc_nf_info->mxc_base_addr + 0x0830)
#define		MXC_NF_V2_MAIN_BUFFER4		(mxc_nf_info->mxc_base_addr + 0x0800)
#define		MXC_NF_V2_MAIN_BUFFER5		(mxc_nf_info->mxc_base_addr + 0x0a00)
#define		MXC_NF_V2_MAIN_BUFFER6		(mxc_nf_info->mxc_base_addr + 0x0c00)
#define		MXC_NF_V2_MAIN_BUFFER7		(mxc_nf_info->mxc_base_addr + 0x0e00)
#define		MXC_NF_V2_SPARE_BUFFER0		(mxc_nf_info->mxc_base_addr + 0x1000)
#define		MXC_NF_V2_SPARE_BUFFER1		(mxc_nf_info->mxc_base_addr + 0x1040)
#define		MXC_NF_V2_SPARE_BUFFER2		(mxc_nf_info->mxc_base_addr + 0x1080)
#define		MXC_NF_V2_SPARE_BUFFER3		(mxc_nf_info->mxc_base_addr + 0x10c0)
#define		MXC_NF_V2_SPARE_BUFFER4		(mxc_nf_info->mxc_base_addr + 0x1100)
#define		MXC_NF_V2_SPARE_BUFFER5		(mxc_nf_info->mxc_base_addr + 0x1140)
#define		MXC_NF_V2_SPARE_BUFFER6		(mxc_nf_info->mxc_base_addr + 0x1180)
#define		MXC_NF_V2_SPARE_BUFFER7		(mxc_nf_info->mxc_base_addr + 0x11c0)
#define		MXC_NF_MAIN_BUFFER_LEN		512
#define		MXC_NF_SPARE_BUFFER_LEN		16
#define		MXC_NF_SPARE_BUFFER_MAX		64
#define		MXC_NF_V1_LAST_BUFFADDR		((MXC_NF_V1_SPARE_BUFFER3) + \
	MXC_NF_SPARE_BUFFER_LEN - 2)
#define		MXC_NF_V2_LAST_BUFFADDR		((MXC_NF_V2_SPARE_BUFFER7) + \
	MXC_NF_SPARE_BUFFER_LEN - 2)

/* bits in MXC_NF_CFG1 register */
#define		MXC_NF_BIT_ECC_4BIT			(1<<0)
#define		MXC_NF_BIT_SPARE_ONLY_EN	(1<<2)
#define		MXC_NF_BIT_ECC_EN			(1<<3)
#define		MXC_NF_BIT_INT_DIS			(1<<4)
#define		MXC_NF_BIT_BE_EN			(1<<5)
#define		MXC_NF_BIT_RESET_EN			(1<<6)
#define		MXC_NF_BIT_FORCE_CE			(1<<7)
#define		MXC_NF_V2_CFG1_PPB(x)		(((x) & 0x3) << 9)

/* bits in MXC_NF_CFG2 register */

/*Flash Command Input*/
#define		MXC_NF_BIT_OP_FCI			(1<<0)
 /*
  * Flash Address Input
  */
#define		MXC_NF_BIT_OP_FAI			(1<<1)
 /*
  * Flash Data Input
  */
#define		MXC_NF_BIT_OP_FDI			(1<<2)

/* see "enum mx_dataout_type" below */
#define		MXC_NF_BIT_DATAOUT_TYPE(x)	((x)<<3)
#define		MXC_NF_BIT_OP_DONE			(1<<15)

#define		MXC_CCM_CGR2				0x53f80028
#define		MXC_GPR						0x43fac008
#define		MX2_FMCR					0x10027814
#define		MX2_FMCR_NF_16BIT_SEL		(1<<4)
#define		MX2_FMCR_NF_FMS				(1<<5)
#define		MX25_RCSR					0x53f80018
#define		MX25_RCSR_NF_16BIT_SEL		(1<<14)
#define		MX25_RCSR_NF_FMS			(1<<8)
#define		MX25_RCSR_NF_4K				(1<<9)
#define		MX3_PCSR					0x53f8000c
#define		MX3_PCSR_NF_16BIT_SEL		(1<<31)
#define		MX3_PCSR_NF_FMS				(1<<30)
#define		MX35_RCSR					0x53f80018
#define		MX35_RCSR_NF_16BIT_SEL		(1<<14)
#define		MX35_RCSR_NF_FMS			(1<<8)
#define		MX35_RCSR_NF_4K				(1<<9)

enum mxc_version {
	MXC_VERSION_UKWN = 0,
	MXC_VERSION_MX25 = 1,
	MXC_VERSION_MX27 = 2,
	MXC_VERSION_MX31 = 3,
	MXC_VERSION_MX35 = 4
};

enum mxc_dataout_type {
	MXC_NF_DATAOUT_PAGE = 1,
	MXC_NF_DATAOUT_NANDID = 2,
	MXC_NF_DATAOUT_NANDSTATUS = 4,
};

enum mxc_nf_finalize_action {
	MXC_NF_FIN_NONE,
	MXC_NF_FIN_DATAOUT,
};

struct mxc_nf_flags {
	unsigned target_little_endian:1;
	unsigned nand_readonly:1;
	unsigned one_kb_sram:1;
	unsigned hw_ecc_enabled:1;
	unsigned biswap_enabled:1;
};

struct mxc_nf_controller {
	enum mxc_version mxc_version;
	uint32_t mxc_base_addr;
	uint32_t mxc_regs_addr;
	enum mxc_dataout_type optype;
	enum mxc_nf_finalize_action fin;
	struct mxc_nf_flags flags;
};

#endif /* OPENOCD_FLASH_NAND_MXC_H */
