/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
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

#include "arm7_9_common.h"
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

int handle_arm7_9_etb_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_arm7_9_etb_dump_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

char *etmv1_branch_reason_string[] =
{
	"normal pc change", "tracing enabled", "restart after FIFO overflow",
	"exit from debug state", "peridoic synchronization point",
	"reserved", "reserved", "reserved"
};


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
				
		jtag_add_ir_scan(1, &field, -1, NULL);
		
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
		jtag_add_dr_scan(1, &field, -1, NULL);
		
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
	
	jtag_add_dr_scan(3, fields, -1, NULL);
	
	/* read the identification register in the second run, to make sure we
	 * don't read the ETB data register twice, skipping every second entry
	 */
	buf_set_u32(fields[1].out_value, 0, 7, 0x0);
	fields[0].in_value = reg->value;
	fields[0].in_check_value = check_value;
	fields[0].in_check_mask = check_mask;
		
	jtag_add_dr_scan(3, fields, -1, NULL);

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
	
	jtag_add_dr_scan(3, fields, -1, NULL);
	
	free(fields[0].out_value);
	free(fields[1].out_value);
	free(fields[2].out_value);
	
	return ERROR_OK;
}

int etb_store_reg(reg_t *reg)
{
	return etb_write_reg(reg, buf_get_u32(reg->value, 0, reg->size));
}

int etb_register_commands(struct command_context_s *cmd_ctx, command_t *arm7_9_cmd)
{
	register_command(cmd_ctx, arm7_9_cmd, "etb", handle_arm7_9_etb_command, COMMAND_CONFIG, NULL);

	register_command(cmd_ctx, arm7_9_cmd, "etb_dump", handle_arm7_9_etb_dump_command, COMMAND_EXEC, "dump current ETB content");

	return ERROR_OK;
}

#define PIPESTAT(x) ((x) & 0x7)
#define TRACEPKT(x) (((x) & 0x7fff8) >> 3)
#define TRACESYNC(x) (((x) & 0x80000) >> 19)

int etmv1_next_packet(int trace_depth, u32 *trace_data, int frame, int *port_half, int apo, u8 *packet)
{
	while (frame < trace_depth)
	{
		if (apo > 0)
		{
			if (TRACESYNC(trace_data[frame]))
				apo--;
		}
		else
		{
			/* we're looking for a branch address, skip if TRACESYNC isn't set */
			if ((apo == 0) && (!TRACESYNC(trace_data[frame])))
			{
				frame++;
				continue;
			}
				
			/* TRACEPKT is valid if this isn't a TD nor a TRIGGER cycle */
			if (((PIPESTAT(trace_data[frame]) != 0x7) && (PIPESTAT(trace_data[frame]) != 0x6))
				&& !((apo == 0) && (!TRACESYNC(trace_data[frame]))))
			{
				if (*port_half == 0)
				{
					*packet = TRACEPKT(trace_data[frame]) & 0xff;
					*port_half = 1;
				}
				else
				{
					*packet = (TRACEPKT(trace_data[frame]) & 0xff00) >> 8;
					*port_half = 0;
					frame++;
				}
				return frame;
			}
		}
		frame++;
	}
	
	/* we reached the end of the trace without finding the packet we're looking for
	 * tracing is finished
	 */
	return -1;
}

int handle_arm7_9_etb_dump_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	int i, j, k;
	int first_frame = 0;
	int last_frame;
	int addressbits_valid = 0;
	u32 address = 0x0;
	u32 *trace_data;
	int port_half = 0;
	int last_instruction = -1;
	u8 branch_reason;
	u8 packet;
	char trace_output[256];
	int trace_output_len;
	u8 apo;

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}
	
	if (!arm7_9->etb)
	{
		command_print(cmd_ctx, "no ETB configured for current target");
		return ERROR_OK;
	}
	
	if (!(arm7_9->etb->RAM_depth && arm7_9->etb->RAM_width))
	{
		/* identify ETB RAM depth and width */
		etb_read_reg(&arm7_9->etb->reg_cache->reg_list[ETB_RAM_DEPTH]);
		etb_read_reg(&arm7_9->etb->reg_cache->reg_list[ETB_RAM_WIDTH]);
		jtag_execute_queue();
	
		arm7_9->etb->RAM_depth = buf_get_u32(arm7_9->etb->reg_cache->reg_list[ETB_RAM_DEPTH].value, 0, 32);
		arm7_9->etb->RAM_width = buf_get_u32(arm7_9->etb->reg_cache->reg_list[ETB_RAM_WIDTH].value, 0, 32);
	}
	
	trace_data = malloc(sizeof(u32) * arm7_9->etb->RAM_depth);
	
	etb_read_reg(&arm7_9->etb->reg_cache->reg_list[ETB_STATUS]);
	etb_read_reg(&arm7_9->etb->reg_cache->reg_list[ETB_RAM_WRITE_POINTER]);
	jtag_execute_queue();
	
	/* check if we overflowed, and adjust first and last frame of the trace accordingly */
	if (buf_get_u32(arm7_9->etb->reg_cache->reg_list[ETB_STATUS].value, 1, 1))
	{
		first_frame = buf_get_u32(arm7_9->etb->reg_cache->reg_list[ETB_RAM_WRITE_POINTER].value, 0, 32);
	}
	
	last_frame = buf_get_u32(arm7_9->etb->reg_cache->reg_list[ETB_RAM_WRITE_POINTER].value, 0, 32) - 1;
	
	etb_write_reg(&arm7_9->etb->reg_cache->reg_list[ETB_RAM_READ_POINTER], first_frame);

	/* read trace data from ETB */
	i = first_frame;
	j = 0;
	do {
		etb_read_reg(&arm7_9->etb->reg_cache->reg_list[ETB_RAM_DATA]);
		jtag_execute_queue();
		trace_data[j++] = buf_get_u32(arm7_9->etb->reg_cache->reg_list[ETB_RAM_DATA].value, 0, 32);
		i++;
	} while ((i % arm7_9->etb->RAM_depth) != (first_frame % arm7_9->etb->RAM_depth));
	
	for (i = 0, j = 0; i < arm7_9->etb->RAM_depth; i++)
	{
		int trigger = 0;
		
		trace_output_len = 0;
		
		/* catch trigger, actual PIPESTAT is encoded in TRACEPKT[2:0] */
		if (PIPESTAT(trace_data[i]) == 0x6)
		{
			trigger = 1;
			trace_data[i] &= ~0x7;
			trace_data[i] |= TRACEPKT(trace_data[i]) & 0x7;
		}
	
		if (addressbits_valid == 32)
		{
			trace_output_len += snprintf(trace_output + trace_output_len, 256 - trace_output_len,
				"%i: 0x%8.8x %s", i, address, (trigger) ? "(TRIGGER) " : "");
		}
		else if (addressbits_valid != 0)
		{
			trace_output_len += snprintf(trace_output + trace_output_len, 256 - trace_output_len,
				"%i: 0x...%x %s", i, address, (trigger) ? "(TRIGGER) " : "");
		}
		else
		{
			trace_output_len += snprintf(trace_output + trace_output_len, 256 - trace_output_len,
				"%i: 0xUNK %s", i, (trigger) ? "(TRIGGER) " : "");
		}
		
		switch (PIPESTAT(trace_data[i]))
		{
			case 0x0:
				trace_output_len += snprintf(trace_output + trace_output_len, 256 - trace_output_len,
					"IE");
				break;
			case 0x1:
				trace_output_len += snprintf(trace_output + trace_output_len, 256 - trace_output_len,
					"ID");
				break;
			case 0x2:
				/* Instruction exectued - TRACEPKT might be valid, but belongs to another cycle */
				trace_output_len += snprintf(trace_output + trace_output_len, 256 - trace_output_len,
					"IN");
				break;
			case 0x3:
				/* WAIT cycle - TRACEPKT is valid, but belongs to another cycle */
				trace_output_len += snprintf(trace_output + trace_output_len, 256 - trace_output_len,
					"WT");
				break;
			case 0x4:
				/* following a branch two APO cycles are output on PIPESTAT[1:0]
				 * but another BE/BD could overwrite the current branch,
				 * or a trigger could cause the APO to be output on TRACEPKT[1:0]
				 */
				if ((PIPESTAT(trace_data[i + 1]) == 0x4)
					|| (PIPESTAT(trace_data[i + 1]) == 0x5))
				{
					/* another branch occured, we ignore this one */
					j = (j < i + 1) ? i + 1 : j;
					break;
				}
				else if (PIPESTAT(trace_data[i + 1]) == 0x6)
				{
					apo = TRACEPKT(trace_data[i + 1]) & 0x3;
				}
				else
				{
					apo = PIPESTAT(trace_data[i + 1]) & 0x3;
				}

				if ((PIPESTAT(trace_data[i + 2]) == 0x4)
					|| (PIPESTAT(trace_data[i + 2]) == 0x5))
				{
					j = (j < i + 2) ? i + 1 : j;
					i = i + 1;
					break;
				}
				else if (PIPESTAT(trace_data[i + 2]) == 0x6)
				{
					apo |= (TRACEPKT(trace_data[i + 2]) & 0x3) << 2;
				}
				else
				{
					apo = (PIPESTAT(trace_data[i + 1]) & 0x3) << 2;
				}
				
				branch_reason = -1;
				k = 0;
				do
				{
					if ((j = etmv1_next_packet(arm7_9->etb->RAM_depth, trace_data, j, &port_half, apo, &packet)) != -1)
					{
						address &= ~(0x7f << (k * 7));
						address |= (packet & 0x7f) << (k * 7);
					}
					else
					{
						break;
					}
					k++;
				} while ((k < 5) && (packet & 0x80));
				
				if (addressbits_valid < ((k * 7 > 32) ? 32 : k * 7))
					addressbits_valid = (k * 7 > 32) ? 32 : k * 7;
				
				if (k == 5)
				{
					branch_reason = (packet & 0x7) >> 4;
					trace_output_len += snprintf(trace_output + trace_output_len, 256 - trace_output_len,
						"BE 0x%x (/%i) (%s)", address, addressbits_valid, etmv1_branch_reason_string[branch_reason]);
				}
				else
				{
					trace_output_len += snprintf(trace_output + trace_output_len, 256 - trace_output_len,
						"BE 0x%x (/%i)", address, addressbits_valid);
				}
				
				break;
			case 0x5:
				trace_output_len += snprintf(trace_output + trace_output_len, 256 - trace_output_len,
					"BD");
				break;
			case 0x6:
				/* We catch the trigger event before we get here */
				ERROR("TR pipestat should have been caught earlier");
				trace_output_len += snprintf(trace_output + trace_output_len, 256 - trace_output_len,
					"--");
				break;
			case 0x7:
				/* TRACE disabled - TRACEPKT = invalid */
				trace_output_len += snprintf(trace_output + trace_output_len, 256 - trace_output_len,
					"TD");
				break;
		}
		
		/* PIPESTAT other than WT (b011) and TD (b111) mean we executed an instruction */
		if ((PIPESTAT(trace_data[i]) & 0x3) != 0x3)
		{
			last_instruction = i;
			address += 4;
		}

		/* The group of packets for a particular instruction cannot start on or before any
		 * previous functional PIPESTAT (IE, IN, ID, BE, or BD)
		 */
		if (j < last_instruction)
		{
			j = last_instruction + 1;
		}

		/* restore trigger PIPESTAT to ensure TRACEPKT is ignored */		
		if (trigger == 1)
		{
			trace_data[i] &= ~0x7;
			trace_data[i] |= 0x6;	
		}
		
		command_print(cmd_ctx, "%s (raw: 0x%8.8x)", trace_output, trace_data[i]);
	}
	
	return ERROR_OK;
}
