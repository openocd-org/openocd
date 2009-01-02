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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef MIPS_EJTAG
#define MIPS_EJTAG

#include "types.h"
#include "jtag.h"

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

/* debug control register bits ECR */
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
#define EJTAG_IMP_NODMA			(1 << 14)
#define EJTAG_IMP_MIPS16		(1 << 16)

/* breakpoint support */
#define EJTAG_DCR				0xFF300000
#define EJTAG_IBS				0xFF301000
#define EJTAG_IBA1				0xFF301100
#define EJTAG_DBS				0xFF302000
#define EJTAG_DBA1				0xFF302100

typedef struct mips_ejtag_s
{
	jtag_tap_t *tap;
	u32 impcode;
	u32 idcode;
	/*int use_dma;*/
	u32 ejtag_ctrl;
} mips_ejtag_t;

extern int mips_ejtag_set_instr(mips_ejtag_t *ejtag_info, int new_instr, in_handler_t handler);
extern int mips_ejtag_enter_debug(mips_ejtag_t *ejtag_info);
extern int mips_ejtag_exit_debug(mips_ejtag_t *ejtag_info, int enable_interrupts);
extern int mips_ejtag_get_impcode(mips_ejtag_t *ejtag_info, u32 *impcode, in_handler_t handler);
extern int mips_ejtag_get_idcode(mips_ejtag_t *ejtag_info, u32 *idcode, in_handler_t handler);
extern int mips_ejtag_drscan_32(mips_ejtag_t *ejtag_info, u32 *data);

extern int mips_ejtag_init(mips_ejtag_t *ejtag_info);
extern int mips_ejtag_config_step(mips_ejtag_t *ejtag_info, int enable_step);
extern int mips_ejtag_read_debug(mips_ejtag_t *ejtag_info, u32* debug_reg);

#endif /* MIPS_EJTAG */
