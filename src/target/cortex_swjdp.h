/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
#ifndef CORTEX_SWJDP_H
#define CORTEX_SWJDP_H

#include "target.h"
#include "register.h"
#include "arm_jtag.h"

#define SWJDP_IR_DPACC	0xA
#define SWJDP_IR_APACC	0xB

#define DPAP_WRITE		0
#define DPAP_READ		1
#define DP_ZERO			0
#define DP_CTRL_STAT	0x4
#define DP_SELECT		0x8
#define DP_RDBUFF		0xC

#define CORUNDETECT		(1<<0)
#define SSTICKYORUN		(1<<1)
#define SSTICKYERR		(1<<5)
#define CDBGRSTREQ		(1<<26)
#define CDBGRSTACK		(1<<27)
#define CDBGPWRUPREQ	(1<<28)
#define CDBGPWRUPACK	(1<<29)
#define CSYSPWRUPREQ	(1<<30)
#define CSYSPWRUPACK	(1<<31)

#define	AHBAP_CSW		0x00
#define AHBAP_TAR		0x04
#define AHBAP_DRW		0x0C
#define AHBAP_BD0		0x10
#define AHBAP_BD1		0x14
#define AHBAP_BD2		0x18
#define AHBAP_BD3		0x1C
#define AHBAP_DBGROMA	0xF8
#define AHBAP_IDR		0xFC

#define CSW_8BIT		0
#define CSW_16BIT		1
#define CSW_32BIT		2

#define CSW_ADDRINC_MASK	(3<<4)
#define CSW_ADDRINC_OFF		0
#define CSW_ADDRINC_SINGLE	(1<<4)
#define CSW_ADDRINC_PACKED	(2<<4)
#define CSW_HPROT			(1<<25)
#define CSW_MASTER_DEBUG	(1<<29)
#define CSW_DBGSWENABLE		(1<<31)

/* transaction mode */
#define TRANS_MODE_NONE			0
/* Transaction waits for previous to complete */
#define TRANS_MODE_ATOMIC		1
/* Freerunning transactions with delays and overrun checking */
#define TRANS_MODE_COMPOSITE	2

typedef struct swjdp_reg_s
{
	int addr;
	arm_jtag_t *jtag_info;
} swjdp_reg_t;

typedef struct swjdp_common_s
{
	arm_jtag_t *jtag_info;
	/* Control config */
	u32 dp_ctrl_stat;
	/* Register select cache */
	u32 dp_select_value;
	u32 ap_csw_value;
	u32 ap_tar_value;
	/* information about current pending SWjDP-AHBAP transaction */
	u8  trans_mode;
	u8  trans_rw;
	u8  ack;
} swjdp_common_t;

/* Internal functions used in the module, partial transactions, use with caution */
extern int swjdp_write_dpacc(swjdp_common_t *swjdp, u32 value, u8 reg_addr);
/* extern int swjdp_write_apacc(swjdp_common_t *swjdp, u32 value, u8 reg_addr); */
extern int swjdp_read_dpacc(swjdp_common_t *swjdp, u32 *value, u8 reg_addr);
/* extern int swjdp_read_apacc(swjdp_common_t *swjdp, u32 *value, u8 reg_addr); */
extern int ahbap_write_reg(swjdp_common_t *swjdp, u32 reg_addr, u8* out_value_buf);
extern int ahbap_read_reg(swjdp_common_t *swjdp, u32 reg_addr, u8 *in_value_buf);

/* External interface, partial operations must be completed with swjdp_transaction_endcheck() */
extern int ahbap_read_system_u32(swjdp_common_t *swjdp, u32 address, u32 *value);
extern int ahbap_write_system_u32(swjdp_common_t *swjdp, u32 address, u32 value);
extern int swjdp_transaction_endcheck(swjdp_common_t *swjdp);

/* External interface, complete atomic operations  */
/* Host endian word transfer of single memory and system registers */
extern int ahbap_read_system_atomic_u32(swjdp_common_t *swjdp, u32 address, u32 *value);
extern int ahbap_write_system_atomic_u32(swjdp_common_t *swjdp, u32 address, u32 value);

/* Host endian word transfers of processor core registers */
extern int ahbap_read_coreregister_u32(swjdp_common_t *swjdp, u32 *value, int regnum);
extern int ahbap_write_coreregister_u32(swjdp_common_t *swjdp, u32 value, int regnum);

extern int ahbap_read_buf_u8(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address);
extern int ahbap_read_buf_u16(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address);
extern int ahbap_read_buf_u32(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address);

extern int ahbap_write_buf_u8(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address);
extern int ahbap_write_buf_u16(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address);
extern int ahbap_write_buf_u32(swjdp_common_t *swjdp, u8 *buffer, int count, u32 address);

/* Initialisation of the debug system, power domains and registers */
extern int ahbap_debugport_init(swjdp_common_t *swjdp);

#endif
