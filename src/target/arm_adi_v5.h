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
#ifndef ARM_ADI_V5_H
#define ARM_ADI_V5_H

#include "target.h"
#include "register.h"
#include "arm_jtag.h"

#define DAP_IR_DPACC	0xA
#define DAP_IR_APACC	0xB

#define DPAP_WRITE		0
#define DPAP_READ		1
#define DP_ZERO			0
#define DP_CTRL_STAT	0x4
#define DP_SELECT		0x8
#define DP_RDBUFF		0xC

#define CORUNDETECT		(1 << 0)
#define SSTICKYORUN		(1 << 1)
#define SSTICKYERR		(1 << 5)
#define CDBGRSTREQ		(1 << 26)
#define CDBGRSTACK		(1 << 27)
#define CDBGPWRUPREQ	(1 << 28)
#define CDBGPWRUPACK	(1 << 29)
#define CSYSPWRUPREQ	(1 << 30)
#define CSYSPWRUPACK	(1 << 31)

#define	AP_REG_CSW		0x00
#define AP_REG_TAR		0x04
#define AP_REG_DRW		0x0C
#define AP_REG_BD0		0x10
#define AP_REG_BD1		0x14
#define AP_REG_BD2		0x18
#define AP_REG_BD3		0x1C
#define AP_REG_DBGROMA	0xF8
#define AP_REG_IDR		0xFC

#define CSW_8BIT		0
#define CSW_16BIT		1
#define CSW_32BIT		2

#define CSW_ADDRINC_MASK	(3 << 4)
#define CSW_ADDRINC_OFF		0
#define CSW_ADDRINC_SINGLE	(1 << 4)
#define CSW_ADDRINC_PACKED	(2 << 4)
#define CSW_HPROT			(1 << 25)
#define CSW_MASTER_DEBUG	(1 << 29)
#define CSW_DBGSWENABLE		(1 << 31)

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
	uint32_t dp_ctrl_stat;
	/* Support for several AP's in one DAP */
	uint32_t apsel;
	/* Register select cache */
	uint32_t dp_select_value;
	uint32_t ap_csw_value;
	uint32_t ap_tar_value;
	/* information about current pending SWjDP-AHBAP transaction */
	uint8_t  trans_mode;
	uint8_t  trans_rw;
	uint8_t  ack;
	/* extra tck clocks for memory bus access */
	uint32_t	memaccess_tck;
	/* Size of TAR autoincrement block, ARM ADI Specification requires at least 10 bits */
	uint32_t tar_autoincr_block;

} swjdp_common_t;

/* Accessor function for currently selected DAP-AP number */
static inline uint8_t dap_ap_get_select(swjdp_common_t *swjdp)
{
	return (uint8_t)(swjdp ->apsel >> 24);
}

/* Internal functions used in the module, partial transactions, use with caution */
int dap_dp_write_reg(swjdp_common_t *swjdp, uint32_t value, uint8_t reg_addr);
/* int swjdp_write_apacc(swjdp_common_t *swjdp, uint32_t value, uint8_t reg_addr); */
int dap_dp_read_reg(swjdp_common_t *swjdp, uint32_t *value, uint8_t reg_addr);
/* int swjdp_read_apacc(swjdp_common_t *swjdp, uint32_t *value, uint8_t reg_addr); */
int dap_setup_accessport(swjdp_common_t *swjdp, uint32_t csw, uint32_t tar);
int dap_ap_select(swjdp_common_t *swjdp,uint8_t apsel);

int dap_ap_write_reg(swjdp_common_t *swjdp, uint32_t addr, uint8_t* out_buf);
int dap_ap_write_reg_u32(swjdp_common_t *swjdp, uint32_t addr, uint32_t value);
int dap_ap_read_reg(swjdp_common_t *swjdp, uint32_t addr, uint8_t *in_buf);
int dap_ap_read_reg_u32(swjdp_common_t *swjdp, uint32_t addr, uint32_t *value);

/* External interface, partial operations must be completed with swjdp_transaction_endcheck() */
int swjdp_transaction_endcheck(swjdp_common_t *swjdp);

/* MEM-AP memory mapped bus single uint32_t register transfers, without endcheck */
int mem_ap_read_u32(swjdp_common_t *swjdp, uint32_t address, uint32_t *value);
int mem_ap_write_u32(swjdp_common_t *swjdp, uint32_t address, uint32_t value);

/* MEM-AP memory mapped bus transfers, single registers, complete transactions */
int mem_ap_read_atomic_u32(swjdp_common_t *swjdp,
		uint32_t address, uint32_t *value);
int mem_ap_write_atomic_u32(swjdp_common_t *swjdp,
		uint32_t address, uint32_t value);

/* MEM-AP memory mapped bus block transfers */
int mem_ap_read_buf_u8(swjdp_common_t *swjdp,
		uint8_t *buffer, int count, uint32_t address);
int mem_ap_read_buf_u16(swjdp_common_t *swjdp,
		uint8_t *buffer, int count, uint32_t address);
int mem_ap_read_buf_u32(swjdp_common_t *swjdp,
		uint8_t *buffer, int count, uint32_t address);

int mem_ap_write_buf_u8(swjdp_common_t *swjdp,
		uint8_t *buffer, int count, uint32_t address);
int mem_ap_write_buf_u16(swjdp_common_t *swjdp,
		uint8_t *buffer, int count, uint32_t address);
int mem_ap_write_buf_u32(swjdp_common_t *swjdp,
		uint8_t *buffer, int count, uint32_t address);

/* Initialisation of the debug system, power domains and registers */
int ahbap_debugport_init(swjdp_common_t *swjdp);


/* Commands for user dap access */
int dap_info_command(struct command_context_s *cmd_ctx,
		swjdp_common_t *swjdp, int apsel);
int dap_baseaddr_command(struct command_context_s *cmd_ctx,
		swjdp_common_t *swjdp, char **args, int argc);
int dap_memaccess_command(struct command_context_s *cmd_ctx,
		swjdp_common_t *swjdp, char **args, int argc);
int dap_apsel_command(struct command_context_s *cmd_ctx,
		swjdp_common_t *swjdp, char **args, int argc);
int dap_apid_command(struct command_context_s *cmd_ctx,
		swjdp_common_t *swjdp, char **args, int argc);

#endif
