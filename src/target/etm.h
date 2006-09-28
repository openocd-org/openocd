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
#ifndef ETM_H
#define ETM_H

#include "target.h"
#include "register.h"
#include "arm_jtag.h"

// ETM registers (V1.2 protocol)
enum
{
	ETM_CTRL = 0x00,
	ETM_CONFIG = 0x01,
	ETM_TRIG_EVENT = 0x02,
	ETM_MMD_CTRL = 0x03,
	ETM_STATUS = 0x04,
	ETM_SYS_CONFIG = 0x05,
	ETM_TRACE_RESOURCE_CTRL = 0x06,
	ETM_TRACE_EN_CTRL2 = 0x07,
	ETM_TRACE_EN_EVENT = 0x08,
	ETM_TRACE_EN_CTRL1 = 0x09,
	ETM_FIFOFULL_REGION = 0x0a,
	ETM_FIFOFULL_LEVEL = 0x0b,
	ETM_VIEWDATA_EVENT = 0x0c,
	ETM_VIEWDATA_CTRL1 = 0x0d,
	ETM_VIEWDATA_CTRL2 = 0x0e,
	ETM_VIEWDATA_CTRL3 = 0x0f,
	ETM_ADDR_COMPARATOR_VALUE = 0x10,
	ETM_ADDR_ACCESS_TYPE = 0x20,
	ETM_DATA_COMPARATOR_VALUE = 0x30,
	ETM_DATA_COMPARATOR_MASK = 0x40,
	ETM_COUNTER_INITAL_VALUE = 0x50,
	ETM_COUNTER_ENABLE = 0x54,
	ETM_COUNTER_RELOAD_VALUE = 0x58,
	ETM_COUNTER_VALUE = 0x5c,
	ETM_SEQUENCER_CTRL = 0x60,
	ETM_SEQUENCER_STATE = 0x67,
	ETM_EXTERNAL_OUTPUT = 0x68,
	ETM_CONTEXTID_COMPARATOR_VALUE = 0x6c,
	ETM_CONTEXTID_COMPARATOR_MASK = 0x6f,	
};


typedef struct etm_reg_s
{
	int addr;
	arm_jtag_t *jtag_info;
} etm_reg_t;

extern reg_cache_t* etm_build_reg_cache(target_t *target, arm_jtag_t *jtag_info, int extra_reg);
extern int etm_read_reg(reg_t *reg);
extern int etm_write_reg(reg_t *reg, u32 value);
extern int etm_read_reg_w_check(reg_t *reg, u8* check_value, u8* check_mask);
extern int etm_store_reg(reg_t *reg);
extern int etm_set_reg(reg_t *reg, u32 value);
extern int etm_set_reg_w_exec(reg_t *reg, u8 *buf);

#endif /* ETM_H */
