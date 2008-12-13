/***************************************************************************
 *   Copyright (C) 2005, 2006 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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
#ifndef EMBEDDED_ICE_H
#define EMBEDDED_ICE_H

#include "target.h"
#include "register.h"
#include "arm_jtag.h"
#include "arm7_9_common.h"

enum
{
	EICE_DBG_CTRL = 0,
	EICE_DBG_STAT = 1,
	EICE_COMMS_CTRL = 2,
	EICE_COMMS_DATA = 3,
	EICE_W0_ADDR_VALUE = 4,
	EICE_W0_ADDR_MASK = 5,
	EICE_W0_DATA_VALUE  = 6,
	EICE_W0_DATA_MASK = 7,
	EICE_W0_CONTROL_VALUE = 8,
	EICE_W0_CONTROL_MASK = 9,
	EICE_W1_ADDR_VALUE = 10,
	EICE_W1_ADDR_MASK = 11,
	EICE_W1_DATA_VALUE = 12,
	EICE_W1_DATA_MASK = 13,
	EICE_W1_CONTROL_VALUE = 14,
	EICE_W1_CONTROL_MASK = 15,
	EICE_VEC_CATCH = 16
};

enum
{
	EICE_DBG_CONTROL_ICEDIS = 5,
	EICE_DBG_CONTROL_MONEN = 4,
	EICE_DBG_CONTROL_INTDIS = 2,
	EICE_DBG_CONTROL_DBGRQ = 1,
	EICE_DBG_CONTROL_DBGACK = 0,
};

enum
{
	EICE_DBG_STATUS_IJBIT = 5,
	EICE_DBG_STATUS_ITBIT = 4,
	EICE_DBG_STATUS_SYSCOMP = 3,
	EICE_DBG_STATUS_IFEN = 2,
	EICE_DBG_STATUS_DBGRQ = 1,
	EICE_DBG_STATUS_DBGACK = 0
};

enum
{
	EICE_W_CTRL_ENABLE = 0x100,
	EICE_W_CTRL_RANGE = 0x80,
	EICE_W_CTRL_CHAIN = 0x40,
	EICE_W_CTRL_EXTERN = 0x20,
	EICE_W_CTRL_nTRANS = 0x10,
	EICE_W_CTRL_nOPC = 0x8,
	EICE_W_CTRL_MAS = 0x6,
	EICE_W_CTRL_ITBIT = 0x2,
	EICE_W_CTRL_nRW = 0x1
};

enum
{
	EICE_COMM_CTRL_WBIT = 1,
	EICE_COMM_CTRL_RBIT = 0
};

typedef struct embeddedice_reg_s
{
	int addr;
	arm_jtag_t *jtag_info;
} embeddedice_reg_t;

extern reg_cache_t* embeddedice_build_reg_cache(target_t *target, arm7_9_common_t *arm7_9);
extern int embeddedice_setup(target_t *target);
extern int embeddedice_read_reg(reg_t *reg);
extern void embeddedice_write_reg(reg_t *reg, u32 value);
extern int embeddedice_read_reg_w_check(reg_t *reg, u8* check_value, u8* check_mask);
extern void embeddedice_store_reg(reg_t *reg);
extern void embeddedice_set_reg(reg_t *reg, u32 value);
extern int embeddedice_set_reg_w_exec(reg_t *reg, u8 *buf);
extern int embeddedice_receive(arm_jtag_t *jtag_info, u32 *data, u32 size);
extern int embeddedice_send(arm_jtag_t *jtag_info, u32 *data, u32 size);
extern int embeddedice_handshake(arm_jtag_t *jtag_info, int hsbit, u32 timeout);

/* If many embeddedice_write_reg() follow eachother, then the >1 invocations can be this faster version of
 * embeddedice_write_reg
 */
static const int embeddedice_num_bits[]={32,5,1};
static __inline__ void embeddedice_write_reg_inner( jtag_tap_t *tap, int reg_addr, u32 value)
{
	u32 values[3];

	values[0]=value;
	values[1]=reg_addr;
	values[2]=1;

	jtag_add_dr_out( tap,
			3,
			embeddedice_num_bits,
			values,
			-1);
}

void embeddedice_write_dcc(jtag_tap_t *tap, int reg_addr, u8 *buffer, int little, int count);

#endif /* EMBEDDED_ICE_H */
