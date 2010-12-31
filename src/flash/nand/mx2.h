
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/*
 * Freescale iMX2* OpenOCD NAND Flash controller support.
 * based on Freescale iMX3* OpenOCD NAND Flash controller support.
 *
 * Many thanks to Ben Dooks for writing s3c24xx driver.
 */

#define		MX2_NF_BASE_ADDR		0xd8000000
#define		MX2_NF_BUFSIZ			(MX2_NF_BASE_ADDR + 0xe00)
#define		MX2_NF_BUFADDR			(MX2_NF_BASE_ADDR + 0xe04)
#define		MX2_NF_FADDR			(MX2_NF_BASE_ADDR + 0xe06)
#define		MX2_NF_FCMD				(MX2_NF_BASE_ADDR + 0xe08)
#define		MX2_NF_BUFCFG			(MX2_NF_BASE_ADDR + 0xe0a)
#define		MX2_NF_ECCSTATUS			(MX2_NF_BASE_ADDR + 0xe0c)
#define		MX2_NF_ECCMAINPOS			(MX2_NF_BASE_ADDR + 0xe0e)
#define		MX2_NF_ECCSPAREPOS			(MX2_NF_BASE_ADDR + 0xe10)
#define		MX2_NF_FWP			(MX2_NF_BASE_ADDR + 0xe12)
#define		MX2_NF_LOCKSTART			(MX2_NF_BASE_ADDR + 0xe14)
#define		MX2_NF_LOCKEND			(MX2_NF_BASE_ADDR + 0xe16)
#define		MX2_NF_FWPSTATUS			(MX2_NF_BASE_ADDR + 0xe18)
 /*
  * all bits not marked as self-clearing bit
  */
#define		MX2_NF_CFG1			(MX2_NF_BASE_ADDR + 0xe1a)
#define		MX2_NF_CFG2			(MX2_NF_BASE_ADDR + 0xe1c)

#define		MX2_NF_MAIN_BUFFER0		(MX2_NF_BASE_ADDR + 0x0000)
#define		MX2_NF_MAIN_BUFFER1		(MX2_NF_BASE_ADDR + 0x0200)
#define		MX2_NF_MAIN_BUFFER2		(MX2_NF_BASE_ADDR + 0x0400)
#define		MX2_NF_MAIN_BUFFER3		(MX2_NF_BASE_ADDR + 0x0600)
#define		MX2_NF_SPARE_BUFFER0	(MX2_NF_BASE_ADDR + 0x0800)
#define		MX2_NF_SPARE_BUFFER1	(MX2_NF_BASE_ADDR + 0x0810)
#define		MX2_NF_SPARE_BUFFER2	(MX2_NF_BASE_ADDR + 0x0820)
#define		MX2_NF_SPARE_BUFFER3	(MX2_NF_BASE_ADDR + 0x0830)
#define		MX2_NF_MAIN_BUFFER_LEN	512
#define		MX2_NF_SPARE_BUFFER_LEN	16
#define		MX2_NF_LAST_BUFFER_ADDR	((MX2_NF_SPARE_BUFFER3) + MX2_NF_SPARE_BUFFER_LEN - 2)

/* bits in MX2_NF_CFG1 register */
#define		MX2_NF_BIT_SPARE_ONLY_EN	(1<<2)
#define		MX2_NF_BIT_ECC_EN			(1<<3)
#define		MX2_NF_BIT_INT_DIS			(1<<4)
#define		MX2_NF_BIT_BE_EN			(1<<5)
#define		MX2_NF_BIT_RESET_EN			(1<<6)
#define		MX2_NF_BIT_FORCE_CE			(1<<7)

/* bits in MX2_NF_CFG2 register */

/*Flash Command Input*/
#define		MX2_NF_BIT_OP_FCI			(1<<0)
 /*
  * Flash Address Input
  */
#define		MX2_NF_BIT_OP_FAI			(1<<1)
 /*
  * Flash Data Input
  */
#define		MX2_NF_BIT_OP_FDI			(1<<2)

/* see "enum mx_dataout_type" below */
#define		MX2_NF_BIT_DATAOUT_TYPE(x)	((x)<<3)
#define		MX2_NF_BIT_OP_DONE			(1<<15)

#define		MX2_CCM_CGR2		0x53f80028
#define		MX2_GPR				0x43fac008
//#define		MX2_PCSR			0x53f8000c
#define		MX2_FMCR			0x10027814
#define		MX2_FMCR_NF_16BIT_SEL		(1<<4)
#define		MX2_FMCR_NF_FMS			(1<<5)

enum mx_dataout_type
{
	MX2_NF_DATAOUT_PAGE = 1,
	MX2_NF_DATAOUT_NANDID = 2,
	MX2_NF_DATAOUT_NANDSTATUS = 4,
};
enum mx_nf_finalize_action
{
	MX2_NF_FIN_NONE,
	MX2_NF_FIN_DATAOUT,
};

struct mx2_nf_flags
{
	unsigned host_little_endian:1;
	unsigned target_little_endian:1;
	unsigned nand_readonly:1;
	unsigned one_kb_sram:1;
	unsigned hw_ecc_enabled:1;
};

struct mx2_nf_controller
{
	enum mx_dataout_type optype;
	enum mx_nf_finalize_action fin;
	struct mx2_nf_flags flags;
};
