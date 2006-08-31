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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "embeddedice.h"

#include "armv4_5.h"
#include "arm7_9_common.h"

#include "log.h"
#include "arm_jtag.h"
#include "types.h"
#include "binarybuffer.h"
#include "target.h"
#include "register.h"
#include "jtag.h"

#include <stdlib.h>

bitfield_desc_t embeddedice_comms_ctrl_bitfield_desc[] = 
{
	{"R", 1},
	{"W", 1},
	{"reserved", 26},
	{"version", 4}
};

int embeddedice_reg_arch_info[] =
{
	0x0, 0x1, 0x4, 0x5,
	0x8, 0x9, 0xa, 0xb, 0xc, 0xd,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15
};

char* embeddedice_reg_list[] =
{
	"debug_ctrl",
	"debug_status",
	
	"comms_ctrl",
	"comms_data",
	
	"watch 0 addr value",
	"watch 0 addr mask",
	"watch 0 data value",
	"watch 0 data mask",
	"watch 0 control value",
	"watch 0 control mask",
	
	"watch 1 addr value",
	"watch 1 addr mask",
	"watch 1 data value",
	"watch 1 data mask",
	"watch 1 control value",
	"watch 1 control mask"
};

int embeddedice_reg_arch_type = -1;

int embeddedice_get_reg(reg_t *reg);
int embeddedice_set_reg(reg_t *reg, u32 value);

int embeddedice_write_reg(reg_t *reg, u32 value);
int embeddedice_read_reg(reg_t *reg);

reg_cache_t* embeddedice_build_reg_cache(target_t *target, arm_jtag_t *jtag_info, int extra_reg)
{
	reg_cache_t *reg_cache = malloc(sizeof(reg_cache_t));
	reg_t *reg_list = NULL;
	embeddedice_reg_t *arch_info = NULL;
	int num_regs = 16 + extra_reg;
	int i;
	
	/* register a register arch-type for EmbeddedICE registers only once */
	if (embeddedice_reg_arch_type == -1)
		embeddedice_reg_arch_type = register_reg_arch_type(embeddedice_get_reg, embeddedice_set_reg_w_exec);
	
	/* the actual registers are kept in two arrays */
	reg_list = calloc(num_regs, sizeof(reg_t));
	arch_info = calloc(num_regs, sizeof(embeddedice_reg_t));
	
	/* fill in values for the reg cache */
	reg_cache->name = "EmbeddedICE registers";
	reg_cache->next = NULL;
	reg_cache->reg_list = reg_list;
	reg_cache->num_regs = num_regs;
	
	/* set up registers */
	for (i = 0; i < num_regs - extra_reg; i++)
	{
		reg_list[i].name = embeddedice_reg_list[i];
		reg_list[i].size = 32;
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].bitfield_desc = NULL;
		reg_list[i].num_bitfields = 0;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].arch_info = &arch_info[i];
		reg_list[i].arch_type = embeddedice_reg_arch_type;
		arch_info[i].addr = embeddedice_reg_arch_info[i];
		arch_info[i].jtag_info = jtag_info;
	}
	
	/* there may be one extra reg (Abort status (ARM7 rev4) or Vector catch (ARM9)) */
	if (extra_reg)
	{
		reg_list[num_regs - 1].arch_info = &arch_info[num_regs - 1];
		reg_list[num_regs - 1].arch_type = embeddedice_reg_arch_type;
		arch_info[num_regs - 1].jtag_info = jtag_info;
	}
	
	return reg_cache;
}

int embeddedice_get_reg(reg_t *reg)
{
	if (embeddedice_read_reg(reg) != ERROR_OK)
	{
		ERROR("BUG: error scheduling EmbeddedICE register read");
		exit(-1);
	}
	
	if (jtag_execute_queue() != ERROR_OK)
	{
		ERROR("register read failed");
	}
	
	return ERROR_OK;
}

int embeddedice_read_reg_w_check(reg_t *reg, u8* check_value, u8* check_mask)
{
	embeddedice_reg_t *ice_reg = reg->arch_info;
	u8 reg_addr = ice_reg->addr & 0x1f;
	scan_field_t fields[3];
	
	DEBUG("%i", ice_reg->addr);

	jtag_add_end_state(TAP_RTI);
	arm_jtag_scann(ice_reg->jtag_info, 0x2);
	arm_jtag_set_instr(ice_reg->jtag_info, ice_reg->jtag_info->intest_instr);
	
	fields[0].device = ice_reg->jtag_info->chain_pos;
	fields[0].num_bits = 32;
	fields[0].out_value = reg->value;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
	
	fields[1].device = ice_reg->jtag_info->chain_pos;
	fields[1].num_bits = 5;
	fields[1].out_value = malloc(1);
	buf_set_u32(fields[1].out_value, 0, 5, reg_addr);
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].device = ice_reg->jtag_info->chain_pos;
	fields[2].num_bits = 1;
	fields[2].out_value = malloc(1);
	buf_set_u32(fields[2].out_value, 0, 1, 0);
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = NULL;
	fields[2].in_check_mask = NULL;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;
	
	jtag_add_dr_scan(3, fields, -1);
	
	fields[0].in_value = reg->value;
	fields[0].in_check_value = check_value;
	fields[0].in_check_mask = check_mask;
	
	/* when reading the DCC data register, leaving the address field set to
	 * EICE_COMMS_DATA would read the register twice
	 * reading the control register is safe
	 */
	buf_set_u32(fields[1].out_value, 0, 5, embeddedice_reg_arch_info[EICE_COMMS_CTRL]);
	
	jtag_add_dr_scan(3, fields, -1);

	free(fields[1].out_value);
	free(fields[2].out_value);
	
	return ERROR_OK;
}

int embeddedice_read_reg(reg_t *reg)
{
	return embeddedice_read_reg_w_check(reg, NULL, NULL);	
}

int embeddedice_set_reg(reg_t *reg, u32 value)
{
	if (embeddedice_write_reg(reg, value) != ERROR_OK)
	{
		ERROR("BUG: error scheduling EmbeddedICE register write");
		exit(-1);
	}
	
	buf_set_u32(reg->value, 0, reg->size, value);
	reg->valid = 1;
	reg->dirty = 0;
	
	return ERROR_OK;
}

int embeddedice_set_reg_w_exec(reg_t *reg, u32 value)
{
	embeddedice_set_reg(reg, value);
	
	if (jtag_execute_queue() != ERROR_OK)
	{
		ERROR("register write failed");
		exit(-1);
	}
	return ERROR_OK;
}

int embeddedice_write_reg(reg_t *reg, u32 value)
{
	embeddedice_reg_t *ice_reg = reg->arch_info;
	u8 reg_addr = ice_reg->addr & 0x1f;
	scan_field_t fields[3];
	
	DEBUG("%i: 0x%8.8x", ice_reg->addr, value);
	
	jtag_add_end_state(TAP_RTI);
	arm_jtag_scann(ice_reg->jtag_info, 0x2);
	arm_jtag_set_instr(ice_reg->jtag_info, ice_reg->jtag_info->intest_instr);
	
	fields[0].device = ice_reg->jtag_info->chain_pos;
	fields[0].num_bits = 32;
	fields[0].out_value = malloc(4);
	buf_set_u32(fields[0].out_value, 0, 32, value);
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
	
	fields[1].device = ice_reg->jtag_info->chain_pos;
	fields[1].num_bits = 5;
	fields[1].out_value = malloc(1);
	buf_set_u32(fields[1].out_value, 0, 5, reg_addr);
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].device = ice_reg->jtag_info->chain_pos;
	fields[2].num_bits = 1;
	fields[2].out_value = malloc(1);
	buf_set_u32(fields[2].out_value, 0, 1, 1);
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = NULL;
	fields[2].in_check_mask = NULL;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;
	
	jtag_add_dr_scan(3, fields, -1);
	
	free(fields[0].out_value);
	free(fields[1].out_value);
	free(fields[2].out_value);
	
	return ERROR_OK;
}

int embeddedice_store_reg(reg_t *reg)
{
	return embeddedice_write_reg(reg, buf_get_u32(reg->value, 0, reg->size));
}

