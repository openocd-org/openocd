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

#include "etm.h"

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

bitfield_desc_t etm_comms_ctrl_bitfield_desc[] = 
{
	{"R", 1},
	{"W", 1},
	{"reserved", 26},
	{"version", 4}
};

int etm_reg_arch_info[] =
{
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17, 
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e, 0x1f, 
	0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x27,
	0x28, 0x29, 0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x2f, 
	0x30, 0x31, 0x32, 0x33, 0x34, 0x35, 0x36, 0x37,
	0x38, 0x39, 0x3a, 0x3b, 0x3c, 0x3d, 0x3e, 0x3f,
	0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47, 
	0x48, 0x49, 0x4a, 0x4b, 0x4c, 0x4d, 0x4e, 0x4f, 
	0x50, 0x51, 0x52, 0x53, 0x54, 0x55, 0x56, 0x57,
	0x58, 0x59, 0x5a, 0x5b, 0x5c, 0x5d, 0x5e, 0x5f,
	0x60, 0x61, 0x62, 0x63, 0x64, 0x65, 0x67, 
	0x68, 0x69, 0x6a, 0x6b, 0x6c, 0x6d, 0x6e, 0x6f, 
};

int etm_reg_arch_size_info[] =
{
	32, 32, 17, 8, 3, 9, 32, 17,
	26, 16, 25, 8, 17, 32, 32, 17,
	32, 32, 32, 32, 32, 32, 32, 32, 
	32, 32, 32, 32, 32, 32, 32, 32, 
	7, 7, 7, 7, 7, 7, 7, 7, 
	7, 7, 7, 7, 7, 7, 7, 7, 
	32, 32, 32, 32, 32, 32, 32, 32, 
	32, 32, 32, 32, 32, 32, 32, 32, 
	32, 32, 32, 32, 32, 32, 32, 32, 
	32, 32, 32, 32, 32, 32, 32, 32, 
	16, 16, 16, 16, 18, 18, 18, 18,
	17, 17, 17, 17, 16, 16, 16, 16,
	17, 17, 17, 17, 17, 17, 2, 
	17, 17, 17, 17, 32, 32, 32, 32 
};

char* etm_reg_list[] =
{
	"ETM_CTRL",
	"ETM_CONFIG",
	"ETM_TRIG_EVENT",
	"ETM_MMD_CTRL",
	"ETM_STATUS",
	"ETM_SYS_CONFIG",
	"ETM_TRACE_RESOURCE_CTRL",
	"ETM_TRACE_EN_CTRL2",
	"ETM_TRACE_EN_EVENT",
	"ETM_TRACE_EN_CTRL1",
	"ETM_FIFOFULL_REGION",
	"ETM_FIFOFULL_LEVEL",
	"ETM_VIEWDATA_EVENT",
	"ETM_VIEWDATA_CTRL1",
	"ETM_VIEWDATA_CTRL2",
	"ETM_VIEWDATA_CTRL3",
	"ETM_ADDR_COMPARATOR_VALUE1",
	"ETM_ADDR_COMPARATOR_VALUE2",
	"ETM_ADDR_COMPARATOR_VALUE3",
	"ETM_ADDR_COMPARATOR_VALUE4",
	"ETM_ADDR_COMPARATOR_VALUE5",
	"ETM_ADDR_COMPARATOR_VALUE6",
	"ETM_ADDR_COMPARATOR_VALUE7",
	"ETM_ADDR_COMPARATOR_VALUE8",
	"ETM_ADDR_COMPARATOR_VALUE9",
	"ETM_ADDR_COMPARATOR_VALUE10",
	"ETM_ADDR_COMPARATOR_VALUE11",
	"ETM_ADDR_COMPARATOR_VALUE12",
	"ETM_ADDR_COMPARATOR_VALUE13",
	"ETM_ADDR_COMPARATOR_VALUE14",
	"ETM_ADDR_COMPARATOR_VALUE15",
	"ETM_ADDR_COMPARATOR_VALUE16",
	"ETM_ADDR_ACCESS_TYPE1",
	"ETM_ADDR_ACCESS_TYPE2",
	"ETM_ADDR_ACCESS_TYPE3",
	"ETM_ADDR_ACCESS_TYPE4",
	"ETM_ADDR_ACCESS_TYPE5",
	"ETM_ADDR_ACCESS_TYPE6",
	"ETM_ADDR_ACCESS_TYPE7",
	"ETM_ADDR_ACCESS_TYPE8",
	"ETM_ADDR_ACCESS_TYPE9",
	"ETM_ADDR_ACCESS_TYPE10",
	"ETM_ADDR_ACCESS_TYPE11",
	"ETM_ADDR_ACCESS_TYPE12",
	"ETM_ADDR_ACCESS_TYPE13",
	"ETM_ADDR_ACCESS_TYPE14",
	"ETM_ADDR_ACCESS_TYPE15",
	"ETM_ADDR_ACCESS_TYPE16",
	"ETM_DATA_COMPARATOR_VALUE1",
	"ETM_DATA_COMPARATOR_VALUE2",
	"ETM_DATA_COMPARATOR_VALUE3",
	"ETM_DATA_COMPARATOR_VALUE4",
	"ETM_DATA_COMPARATOR_VALUE5",
	"ETM_DATA_COMPARATOR_VALUE6",
	"ETM_DATA_COMPARATOR_VALUE7",
	"ETM_DATA_COMPARATOR_VALUE8",
	"ETM_DATA_COMPARATOR_VALUE9",
	"ETM_DATA_COMPARATOR_VALUE10",
	"ETM_DATA_COMPARATOR_VALUE11",
	"ETM_DATA_COMPARATOR_VALUE12",
	"ETM_DATA_COMPARATOR_VALUE13",
	"ETM_DATA_COMPARATOR_VALUE14",
	"ETM_DATA_COMPARATOR_VALUE15",
	"ETM_DATA_COMPARATOR_VALUE16",
	"ETM_DATA_COMPARATOR_MASK1",
	"ETM_DATA_COMPARATOR_MASK2",
	"ETM_DATA_COMPARATOR_MASK3",
	"ETM_DATA_COMPARATOR_MASK4",
	"ETM_DATA_COMPARATOR_MASK5",
	"ETM_DATA_COMPARATOR_MASK6",
	"ETM_DATA_COMPARATOR_MASK7",
	"ETM_DATA_COMPARATOR_MASK8",
	"ETM_DATA_COMPARATOR_MASK9",
	"ETM_DATA_COMPARATOR_MASK10",
	"ETM_DATA_COMPARATOR_MASK11",
	"ETM_DATA_COMPARATOR_MASK12",
	"ETM_DATA_COMPARATOR_MASK13",
	"ETM_DATA_COMPARATOR_MASK14",
	"ETM_DATA_COMPARATOR_MASK15",
	"ETM_DATA_COMPARATOR_MASK16",
	"ETM_COUNTER_INITAL_VALUE1",
	"ETM_COUNTER_INITAL_VALUE2",
	"ETM_COUNTER_INITAL_VALUE3",
	"ETM_COUNTER_INITAL_VALUE4",
	"ETM_COUNTER_ENABLE1",
	"ETM_COUNTER_ENABLE2",
	"ETM_COUNTER_ENABLE3",
	"ETM_COUNTER_ENABLE4",
	"ETM_COUNTER_RELOAD_VALUE1",
	"ETM_COUNTER_RELOAD_VALUE2",
	"ETM_COUNTER_RELOAD_VALUE3",
	"ETM_COUNTER_RELOAD_VALUE4",
	"ETM_COUNTER_VALUE1",
	"ETM_COUNTER_VALUE2",
	"ETM_COUNTER_VALUE3",
	"ETM_COUNTER_VALUE4",
	"ETM_SEQUENCER_CTRL1",
	"ETM_SEQUENCER_CTRL2",
	"ETM_SEQUENCER_CTRL3",
	"ETM_SEQUENCER_CTRL4",
	"ETM_SEQUENCER_CTRL5",
	"ETM_SEQUENCER_CTRL6",
	"ETM_SEQUENCER_STATE",
	"ETM_EXTERNAL_OUTPUT1",
	"ETM_EXTERNAL_OUTPUT2",
	"ETM_EXTERNAL_OUTPUT3",
	"ETM_EXTERNAL_OUTPUT4",
	"ETM_CONTEXTID_COMPARATOR_VALUE1",
	"ETM_CONTEXTID_COMPARATOR_VALUE2",
	"ETM_CONTEXTID_COMPARATOR_VALUE3",
	"ETM_CONTEXTID_COMPARATOR_MASK"
};  

int etm_reg_arch_type = -1;

int etm_get_reg(reg_t *reg);
int etm_set_reg(reg_t *reg, u32 value);
int etm_set_reg_w_exec(reg_t *reg, u8 *buf);

int etm_write_reg(reg_t *reg, u32 value);
int etm_read_reg(reg_t *reg);

reg_cache_t* etm_build_reg_cache(target_t *target, arm_jtag_t *jtag_info, int extra_reg)
{
	reg_cache_t *reg_cache = malloc(sizeof(reg_cache_t));
	reg_t *reg_list = NULL;
	etm_reg_t *arch_info = NULL;
	int num_regs = sizeof(etm_reg_arch_info)/sizeof(int);
	int i;
	
	/* register a register arch-type for etm registers only once */
	if (etm_reg_arch_type == -1)
		etm_reg_arch_type = register_reg_arch_type(etm_get_reg, etm_set_reg_w_exec);
	
	/* the actual registers are kept in two arrays */
	reg_list = calloc(num_regs, sizeof(reg_t));
	arch_info = calloc(num_regs, sizeof(etm_reg_t));
	
	/* fill in values for the reg cache */
	reg_cache->name = "etm registers";
	reg_cache->next = NULL;
	reg_cache->reg_list = reg_list;
	reg_cache->num_regs = num_regs;
	
	/* set up registers */
	for (i = 0; i < num_regs; i++)
	{
		reg_list[i].name = etm_reg_list[i];
		reg_list[i].size = 32;
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].bitfield_desc = NULL;
		reg_list[i].num_bitfields = 0;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].arch_info = &arch_info[i];
		reg_list[i].arch_type = etm_reg_arch_type;
		reg_list[i].size = etm_reg_arch_size_info[i];
		arch_info[i].addr = etm_reg_arch_info[i];
		arch_info[i].jtag_info = jtag_info;
	}
	return reg_cache;
}

int etm_get_reg(reg_t *reg)
{
	if (etm_read_reg(reg) != ERROR_OK)
	{
		ERROR("BUG: error scheduling etm register read");
		exit(-1);
	}
	
	if (jtag_execute_queue() != ERROR_OK)
	{
		ERROR("register read failed");
	}
	
	return ERROR_OK;
}

int etm_read_reg_w_check(reg_t *reg, u8* check_value, u8* check_mask)
{
	etm_reg_t *etm_reg = reg->arch_info;
	u8 reg_addr = etm_reg->addr & 0x7f;
	scan_field_t fields[3];
	
	DEBUG("%i", etm_reg->addr);

	jtag_add_end_state(TAP_RTI);
	arm_jtag_scann(etm_reg->jtag_info, 0x6);
	arm_jtag_set_instr(etm_reg->jtag_info, etm_reg->jtag_info->intest_instr);
	
	fields[0].device = etm_reg->jtag_info->chain_pos;
	fields[0].num_bits = 32;
	fields[0].out_value = reg->value;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
	
	fields[1].device = etm_reg->jtag_info->chain_pos;
	fields[1].num_bits = 7;
	fields[1].out_value = malloc(1);
	buf_set_u32(fields[1].out_value, 0, 7, reg_addr);
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].device = etm_reg->jtag_info->chain_pos;
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
		
	jtag_add_dr_scan(3, fields, -1);

	free(fields[1].out_value);
	free(fields[2].out_value);
	
	return ERROR_OK;
}

int etm_read_reg(reg_t *reg)
{
	return etm_read_reg_w_check(reg, NULL, NULL);	
}

int etm_set_reg(reg_t *reg, u32 value)
{
	if (etm_write_reg(reg, value) != ERROR_OK)
	{
		ERROR("BUG: error scheduling etm register write");
		exit(-1);
	}
	
	buf_set_u32(reg->value, 0, reg->size, value);
	reg->valid = 1;
	reg->dirty = 0;
	
	return ERROR_OK;
}

int etm_set_reg_w_exec(reg_t *reg, u8 *buf)
{
	etm_set_reg(reg, buf_get_u32(buf, 0, reg->size));
	
	if (jtag_execute_queue() != ERROR_OK)
	{
		ERROR("register write failed");
		exit(-1);
	}
	return ERROR_OK;
}

int etm_write_reg(reg_t *reg, u32 value)
{
	etm_reg_t *etm_reg = reg->arch_info;
	u8 reg_addr = etm_reg->addr & 0x7f;
	scan_field_t fields[3];
	
	DEBUG("%i: 0x%8.8x", etm_reg->addr, value);
	
	jtag_add_end_state(TAP_RTI);
	arm_jtag_scann(etm_reg->jtag_info, 0x6);
	arm_jtag_set_instr(etm_reg->jtag_info, etm_reg->jtag_info->intest_instr);
	
	fields[0].device = etm_reg->jtag_info->chain_pos;
	fields[0].num_bits = 32;
	fields[0].out_value = malloc(4);
	buf_set_u32(fields[0].out_value, 0, 32, value);
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
	
	fields[1].device = etm_reg->jtag_info->chain_pos;
	fields[1].num_bits = 7;
	fields[1].out_value = malloc(1);
	buf_set_u32(fields[1].out_value, 0, 7, reg_addr);
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].device = etm_reg->jtag_info->chain_pos;
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

int etm_store_reg(reg_t *reg)
{
	return etm_write_reg(reg, buf_get_u32(reg->value, 0, reg->size));
}

