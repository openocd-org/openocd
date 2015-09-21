/***************************************************************************
 *   Copyright (C) 2009 by Alexei Babich                                   *
 *   Rezonans plc., Chelyabinsk, Russia                                    *
 *   impatt@mail.ru                                                        *
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

#ifndef OPENOCD_FLASH_NAND_MX3_H
#define OPENOCD_FLASH_NAND_MX3_H

/*
 * Freescale iMX3* OpenOCD NAND Flash controller support.
 *
 * Many thanks to Ben Dooks for writing s3c24xx driver.
 */

#define MX3_NF_BASE_ADDR				0xb8000000
#define MX3_NF_BUFSIZ					(MX3_NF_BASE_ADDR + 0xe00)
#define MX3_NF_BUFADDR					(MX3_NF_BASE_ADDR + 0xe04)
#define MX3_NF_FADDR					(MX3_NF_BASE_ADDR + 0xe06)
#define MX3_NF_FCMD						(MX3_NF_BASE_ADDR + 0xe08)
#define MX3_NF_BUFCFG					(MX3_NF_BASE_ADDR + 0xe0a)
#define MX3_NF_ECCSTATUS				(MX3_NF_BASE_ADDR + 0xe0c)
#define MX3_NF_ECCMAINPOS				(MX3_NF_BASE_ADDR + 0xe0e)
#define MX3_NF_ECCSPAREPOS				(MX3_NF_BASE_ADDR + 0xe10)
#define MX3_NF_FWP						(MX3_NF_BASE_ADDR + 0xe12)
#define MX3_NF_LOCKSTART				(MX3_NF_BASE_ADDR + 0xe14)
#define MX3_NF_LOCKEND					(MX3_NF_BASE_ADDR + 0xe16)
#define MX3_NF_FWPSTATUS				(MX3_NF_BASE_ADDR + 0xe18)
/*
 * all bits not marked as self-clearing bit
 */
#define MX3_NF_CFG1						(MX3_NF_BASE_ADDR + 0xe1a)
#define MX3_NF_CFG2						(MX3_NF_BASE_ADDR + 0xe1c)

#define MX3_NF_MAIN_BUFFER0				(MX3_NF_BASE_ADDR + 0x0000)
#define MX3_NF_MAIN_BUFFER1				(MX3_NF_BASE_ADDR + 0x0200)
#define MX3_NF_MAIN_BUFFER2				(MX3_NF_BASE_ADDR + 0x0400)
#define MX3_NF_MAIN_BUFFER3				(MX3_NF_BASE_ADDR + 0x0600)
#define MX3_NF_SPARE_BUFFER0			(MX3_NF_BASE_ADDR + 0x0800)
#define MX3_NF_SPARE_BUFFER1			(MX3_NF_BASE_ADDR + 0x0810)
#define MX3_NF_SPARE_BUFFER2			(MX3_NF_BASE_ADDR + 0x0820)
#define MX3_NF_SPARE_BUFFER3			(MX3_NF_BASE_ADDR + 0x0830)
#define MX3_NF_MAIN_BUFFER_LEN			512
#define MX3_NF_SPARE_BUFFER_LEN			16
#define MX3_NF_LAST_BUFFER_ADDR			((MX3_NF_SPARE_BUFFER3) + MX3_NF_SPARE_BUFFER_LEN - 2)

/* bits in MX3_NF_CFG1 register */
#define MX3_NF_BIT_SPARE_ONLY_EN		(1<<2)
#define MX3_NF_BIT_ECC_EN				(1<<3)
#define MX3_NF_BIT_INT_DIS				(1<<4)
#define MX3_NF_BIT_BE_EN				(1<<5)
#define MX3_NF_BIT_RESET_EN				(1<<6)
#define MX3_NF_BIT_FORCE_CE				(1<<7)

/* bits in MX3_NF_CFG2 register */

/*Flash Command Input*/
#define MX3_NF_BIT_OP_FCI				(1<<0)
/*
 * Flash Address Input
 */
#define MX3_NF_BIT_OP_FAI				(1<<1)
/*
 * Flash Data Input
 */
#define MX3_NF_BIT_OP_FDI				(1<<2)

/* see "enum mx_dataout_type" below */
#define MX3_NF_BIT_DATAOUT_TYPE(x)		((x)<<3)
#define MX3_NF_BIT_OP_DONE				(1<<15)

#define MX3_CCM_CGR2					0x53f80028
#define MX3_GPR							0x43fac008
#define MX3_PCSR						0x53f8000c

enum mx_dataout_type {
	MX3_NF_DATAOUT_PAGE = 1,
	MX3_NF_DATAOUT_NANDID = 2,
	MX3_NF_DATAOUT_NANDSTATUS = 4,
};
enum mx_nf_finalize_action {
	MX3_NF_FIN_NONE,
	MX3_NF_FIN_DATAOUT,
};

struct mx3_nf_flags {
	unsigned target_little_endian:1;
	unsigned nand_readonly:1;
	unsigned one_kb_sram:1;
	unsigned hw_ecc_enabled:1;
};

struct mx3_nf_controller {
	enum mx_dataout_type optype;
	enum mx_nf_finalize_action fin;
	struct mx3_nf_flags flags;
};

#endif /* OPENOCD_FLASH_NAND_MX3_H */
