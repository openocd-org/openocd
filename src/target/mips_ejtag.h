/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef MIPS_EJTAG
#define MIPS_EJTAG

#include <jtag/jtag.h>

/* tap instructions */
#define EJTAG_INST_IDCODE		0x01
#define EJTAG_INST_IMPCODE		0x03
#define EJTAG_INST_ADDRESS		0x08
#define EJTAG_INST_DATA			0x09
#define EJTAG_INST_CONTROL		0x0A
#define EJTAG_INST_ALL			0x0B
#define EJTAG_INST_EJTAGBOOT	0x0C
#define EJTAG_INST_NORMALBOOT	0x0D
#define EJTAG_INST_FASTDATA		0x0E
#define EJTAG_INST_TCBCONTROLA	0x10
#define EJTAG_INST_TCBCONTROLB	0x11
#define EJTAG_INST_TCBDATA		0x12
#define EJTAG_INST_BYPASS		0xFF

/* microchip PIC32MX specific instructions */
#define MTAP_SW_MTAP			0x04
#define MTAP_SW_ETAP			0x05
#define MTAP_COMMAND			0x07

/* microchip specific cmds */
#define MCHP_ASERT_RST			0xd1
#define MCHP_DE_ASSERT_RST		0xd0
#define MCHP_ERASE				0xfc
#define MCHP_STATUS				0x00

/* ejtag control register bits ECR */
#define EJTAG_CTRL_TOF			(1 << 1)
#define EJTAG_CTRL_TIF			(1 << 2)
#define EJTAG_CTRL_BRKST		(1 << 3)
#define EJTAG_CTRL_DLOCK		(1 << 5)
#define EJTAG_CTRL_DRWN			(1 << 9)
#define EJTAG_CTRL_DERR			(1 << 10)
#define EJTAG_CTRL_DSTRT		(1 << 11)
#define EJTAG_CTRL_JTAGBRK		(1 << 12)
#define EJTAG_CTRL_SETDEV		(1 << 14)
#define EJTAG_CTRL_PROBEN		(1 << 15)
#define EJTAG_CTRL_PRRST		(1 << 16)
#define EJTAG_CTRL_DMAACC		(1 << 17)
#define EJTAG_CTRL_PRACC		(1 << 18)
#define EJTAG_CTRL_PRNW			(1 << 19)
#define EJTAG_CTRL_PERRST		(1 << 20)
#define EJTAG_CTRL_SYNC			(1 << 23)
#define EJTAG_CTRL_DNM			(1 << 28)
#define EJTAG_CTRL_ROCC			(1 << 31)

/* Debug Register (CP0 Register 23, Select 0) */

#define EJTAG_DEBUG_DSS			(1 << 0)
#define EJTAG_DEBUG_DBP			(1 << 1)
#define EJTAG_DEBUG_DDBL		(1 << 2)
#define EJTAG_DEBUG_DDBS		(1 << 3)
#define EJTAG_DEBUG_DIB			(1 << 4)
#define EJTAG_DEBUG_DINT		(1 << 5)
#define EJTAG_DEBUG_OFFLINE		(1 << 7)
#define EJTAG_DEBUG_SST			(1 << 8)
#define EJTAG_DEBUG_NOSST		(1 << 9)
#define EJTAG_DEBUG_DDBLIMPR	(1 << 18)
#define EJTAG_DEBUG_DDBSIMPR	(1 << 19)
#define EJTAG_DEBUG_IEXI		(1 << 20)
#define EJTAG_DEBUG_DBUSEP		(1 << 21)
#define EJTAG_DEBUG_CACHEEP		(1 << 22)
#define EJTAG_DEBUG_MCHECKP		(1 << 23)
#define EJTAG_DEBUG_IBUSEP		(1 << 24)
#define EJTAG_DEBUG_COUNTDM		(1 << 25)
#define EJTAG_DEBUG_HALT		(1 << 26)
#define EJTAG_DEBUG_DOZE		(1 << 27)
#define EJTAG_DEBUG_LSNM		(1 << 28)
#define EJTAG_DEBUG_NODCR		(1 << 29)
#define EJTAG_DEBUG_DM			(1 << 30)
#define EJTAG_DEBUG_DBD			(1 << 31)

/* implementaion register bits */
#define EJTAG_IMP_R3K			(1 << 28)
#define EJTAG_IMP_DINT			(1 << 24)
#define EJTAG_IMP_NODMA			(1 << 14)
#define EJTAG_IMP_MIPS16		(1 << 16)
#define EJTAG_DCR_MIPS64		(1 << 0)

/* Debug Control Register DCR */
#define EJTAG_DCR				0xFF300000
#define EJTAG_DCR_ENM			(1 << 29)
#define EJTAG_DCR_DB			(1 << 17)
#define EJTAG_DCR_IB			(1 << 16)
#define EJTAG_DCR_INTE			(1 << 4)
#define EJTAG_DCR_MP			(1 << 2)

/* breakpoint support */
/* EJTAG_V20_* was tested on Broadcom BCM7401
 * and may or will differ with other hardware. For example EZ4021-FC. */
#define EJTAG_V20_IBS			0xFF300004
#define EJTAG_V20_IBA0			0xFF300100
#define EJTAG_V20_IBC_OFFS		0x4	/* IBC Offset */
#define EJTAG_V20_IBM_OFFS		0x8
#define EJTAG_V20_IBAn_STEP		0x10	/* Offset for next channel */
#define EJTAG_V20_DBS			0xFF30008
#define EJTAG_V20_DBA0			0xFF300200
#define EJTAG_V20_DBC_OFFS		0x4
#define EJTAG_V20_DBM_OFFS		0x8
#define EJTAG_V20_DBV_OFFS		0xc
#define EJTAG_V20_DBAn_STEP		0x10

#define EJTAG_V25_IBS			0xFF301000
#define EJTAG_V25_IBA0			0xFF301100
#define EJTAG_V25_IBM_OFFS		0x8
#define EJTAG_V25_IBASID_OFFS		0x10
#define EJTAG_V25_IBC_OFFS		0x18
#define EJTAG_V25_IBAn_STEP		0x100
#define EJTAG_V25_DBS			0xFF302000
#define EJTAG_V25_DBA0			0xFF302100
#define EJTAG_V25_DBM_OFFS		0x8
#define EJTAG_V25_DBASID_OFFS		0x10
#define EJTAG_V25_DBC_OFFS		0x18
#define EJTAG_V25_DBV_OFFS		0x20
#define EJTAG_V25_DBAn_STEP		0x100

#define	EJTAG_DBCn_NOSB			(1 << 13)
#define	EJTAG_DBCn_NOLB			(1 << 12)
#define	EJTAG_DBCn_BLM_MASK		0xff
#define	EJTAG_DBCn_BLM_SHIFT	4
#define	EJTAG_DBCn_BE			(1 << 0)

#define EJTAG_VERSION_20		0
#define EJTAG_VERSION_25		1
#define EJTAG_VERSION_26		2
#define EJTAG_VERSION_31		3
#define EJTAG_VERSION_41		4
#define EJTAG_VERSION_51		5

struct mips_ejtag {
	struct jtag_tap *tap;
	uint32_t impcode;
	uint32_t idcode;
	uint32_t ejtag_ctrl;
	int fast_access_save;
	uint32_t reg8;
	uint32_t reg9;
	unsigned scan_delay;
	int mode;
	unsigned int ejtag_version;

	/* Memory-Mapped Registers. This addresses are not same on different
	 * EJTAG versions. */
	uint32_t ejtag_ibs_addr;	/* Instruction Address Break Status */
	uint32_t ejtag_iba0_addr;	/* IAB channel 0 */
	uint32_t ejtag_ibc_offs;	/* IAB Control offset */
	uint32_t ejtag_ibm_offs;	/* IAB Mask offset */
	uint32_t ejtag_ibasid_offs;	/* IAB ASID (4Kc) */

	uint32_t ejtag_dbs_addr;	/* Data Address Break Status Register */
	uint32_t ejtag_dba0_addr;	/* DAB channel 0 */
	uint32_t ejtag_dbc_offs;	/* DAB Control offset */
	uint32_t ejtag_dbm_offs;	/* DAB Mask offset */
	uint32_t ejtag_dbv_offs;	/* DAB Value offset */
	uint32_t ejtag_dbasid_offs;	/* DAB ASID (4Kc) */

	uint32_t ejtag_iba_step_size;
	uint32_t ejtag_dba_step_size;	/* siez of step till next
					 * *DBAn register. */
};

void mips_ejtag_set_instr(struct mips_ejtag *ejtag_info,
		int new_instr);
int mips_ejtag_enter_debug(struct mips_ejtag *ejtag_info);
int mips_ejtag_exit_debug(struct mips_ejtag *ejtag_info);
int mips_ejtag_get_idcode(struct mips_ejtag *ejtag_info, uint32_t *idcode);
void mips_ejtag_add_scan_96(struct mips_ejtag *ejtag_info,
			    uint32_t ctrl, uint32_t data, uint8_t *in_scan_buf);
void mips_ejtag_drscan_32_out(struct mips_ejtag *ejtag_info, uint32_t data);
int mips_ejtag_drscan_32(struct mips_ejtag *ejtag_info, uint32_t *data);
void mips_ejtag_drscan_8_out(struct mips_ejtag *ejtag_info, uint8_t data);
int mips_ejtag_drscan_8(struct mips_ejtag *ejtag_info, uint32_t *data);
int mips_ejtag_fastdata_scan(struct mips_ejtag *ejtag_info, int write_t, uint32_t *data);

int mips_ejtag_init(struct mips_ejtag *ejtag_info);
int mips_ejtag_config_step(struct mips_ejtag *ejtag_info, int enable_step);

static inline void mips_le_to_h_u32(jtag_callback_data_t arg)
{
	uint8_t *in = (uint8_t *)arg;
	*((uint32_t *)arg) = le_to_h_u32(in);
}

#endif /* MIPS_EJTAG */
