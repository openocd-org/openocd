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

#include "etb.h"

#include "log.h"
#include "types.h"
#include "binarybuffer.h"
#include "target.h"
#include "register.h"
#include "jtag.h"

#include <stdlib.h>

char* etb_reg_list[] =
{
	"ETB_identification",
	"ETB_ram_depth",
	"ETB_ram_width",
	"ETB_status",
	"ETB_ram_data",
	"ETB_ram_read_pointer",
	"ETB_ram_write_pointer",
	"ETB_trigger_counter",
	"ETB_control",
};

int etb_reg_arch_type = -1;

int etb_get_reg(reg_t *reg);
int etb_set_reg(reg_t *reg, u32 value);
int etb_set_reg_w_exec(reg_t *reg, u8 *buf);

int etb_write_reg(reg_t *reg, u32 value);
int etb_read_reg(reg_t *reg);

int etb_set_instr(etb_t *etb, u32 new_instr)
{
	jtag_device_t *device = jtag_get_device(etb->chain_pos);
	
	if (buf_get_u32(device->cur_instr, 0, device->ir_length) != new_instr)
	{
		scan_field_t field;
	
		field.device = etb->chain_pos;
		field.num_bits = device->ir_length;
		field.out_value = calloc(CEIL(field.num_bits, 8), 1);
		buf_set_u32(field.out_value, 0, field.num_bits, new_instr);
		field.out_mask = NULL;
		field.in_value = NULL;
		field.in_check_value = NULL;
		field.in_check_mask = NULL;
		field.in_handler = NULL;
		field.in_handler_priv = NULL;
		
		jtag_add_ir_scan(1, &field, -1);
		
		free(field.out_value);
	}
	
	return ERROR_OK;
}

int etb_scann(etb_t *etb, u32 new_scan_chain)
{
	if(etb->cur_scan_chain != new_scan_chain)
	{
		scan_field_t field;
		
		field.device = etb->chain_pos;
		field.num_bits = 5;
		field.out_value = calloc(CEIL(field.num_bits, 8), 1);
		buf_set_u32(field.out_value, 0, field.num_bits, new_scan_chain);
		field.out_mask = NULL;
		field.in_value = NULL;
		field.in_check_value = NULL;
		field.in_check_mask = NULL;
		field.in_handler = NULL;
		field.in_handler_priv = NULL;
		
		/* select INTEST instruction */
		etb_set_instr(etb, 0x2);
		jtag_add_dr_scan(1, &field, -1);
		
		etb->cur_scan_chain = new_scan_chain;
		
		free(field.out_value);
	}

	return ERROR_OK;
}

reg_cache_t* etb_build_reg_cache(etb_t *etb)
{
	reg_cache_t *reg_cache = malloc(sizeof(reg_cache_t));
	reg_t *reg_list = NULL;
	etb_reg_t *arch_info = NULL;
	int num_regs = 9;
	int i;
	
	/* register a register arch-type for etm registers only once */
	if (etb_reg_arch_type == -1)
		etb_reg_arch_type = register_reg_arch_type(etb_get_reg, etb_set_reg_w_exec);
	
	/* the actual registers are kept in two arrays */
	reg_list = calloc(num_regs, sizeof(reg_t));
	arch_info = calloc(num_regs, sizeof(etb_reg_t));
	
	/* fill in values for the reg cache */
	reg_cache->name = "etb registers";
	reg_cache->next = NULL;
	reg_cache->reg_list = reg_list;
	reg_cache->num_regs = num_regs;
	
	/* set up registers */
	for (i = 0; i < num_regs; i++)
	{
		reg_list[i].name = etb_reg_list[i];
		reg_list[i].size = 32;
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].bitfield_desc = NULL;
		reg_list[i].num_bitfields = 0;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].arch_info = &arch_info[i];
		reg_list[i].arch_type = etb_reg_arch_type;
		reg_list[i].size = 32;
		arch_info[i].addr = i;
		arch_info[i].etb = etb;
	}
	
	return reg_cache;
}

int etb_get_reg(reg_t *reg)
{
	if (etb_read_reg(reg) != ERROR_OK)
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

int etb_read_reg_w_check(reg_t *reg, u8* check_value, u8* check_mask)
{
	etb_reg_t *etb_reg = reg->arch_info;
	u8 reg_addr = etb_reg->addr & 0x7f;
	scan_field_t fields[3];
	
	DEBUG("%i", etb_reg->addr);

	jtag_add_end_state(TAP_RTI);
	etb_scann(etb_reg->etb, 0x0);
	etb_set_instr(etb_reg->etb, 0xc);
	
	fields[0].device = etb_reg->etb->chain_pos;
	fields[0].num_bits = 32;
	fields[0].out_value = reg->value;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
	
	fields[1].device = etb_reg->etb->chain_pos;
	fields[1].num_bits = 7;
	fields[1].out_value = malloc(1);
	buf_set_u32(fields[1].out_value, 0, 7, reg_addr);
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].device = etb_reg->etb->chain_pos;
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

int etb_read_reg(reg_t *reg)
{
	return etb_read_reg_w_check(reg, NULL, NULL);	
}

int etb_set_reg(reg_t *reg, u32 value)
{
	if (etb_write_reg(reg, value) != ERROR_OK)
	{
		ERROR("BUG: error scheduling etm register write");
		exit(-1);
	}
	
	buf_set_u32(reg->value, 0, reg->size, value);
	reg->valid = 1;
	reg->dirty = 0;
	
	return ERROR_OK;
}

int etb_set_reg_w_exec(reg_t *reg, u8 *buf)
{
	etb_set_reg(reg, buf_get_u32(buf, 0, reg->size));
	
	if (jtag_execute_queue() != ERROR_OK)
	{
		ERROR("register write failed");
		exit(-1);
	}
	return ERROR_OK;
}

int etb_write_reg(reg_t *reg, u32 value)
{
	etb_reg_t *etb_reg = reg->arch_info;
	u8 reg_addr = etb_reg->addr & 0x7f;
	scan_field_t fields[3];
	
	DEBUG("%i: 0x%8.8x", etb_reg->addr, value);
	
	jtag_add_end_state(TAP_RTI);
	etb_scann(etb_reg->etb, 0x0);
	etb_set_instr(etb_reg->etb, 0xc);
	
	fields[0].device = etb_reg->etb->chain_pos;
	fields[0].num_bits = 32;
	fields[0].out_value = malloc(4);
	buf_set_u32(fields[0].out_value, 0, 32, value);
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
	
	fields[1].device = etb_reg->etb->chain_pos;
	fields[1].num_bits = 7;
	fields[1].out_value = malloc(1);
	buf_set_u32(fields[1].out_value, 0, 7, reg_addr);
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].device = etb_reg->etb->chain_pos;
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

int etb_store_reg(reg_t *reg)
{
	return etb_write_reg(reg, buf_get_u32(reg->value, 0, reg->size));
}

