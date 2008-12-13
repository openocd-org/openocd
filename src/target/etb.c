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

#include <string.h>

#include "arm7_9_common.h"
#include "etb.h"
#include "etm.h"

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

int handle_etb_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int etb_set_instr(etb_t *etb, u32 new_instr)
{
	jtag_tap_t *tap;
	tap = etb->tap;
	if (tap==NULL)
		return ERROR_FAIL;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != new_instr)
	{
		scan_field_t field;

		field.tap = tap;
		field.num_bits = tap->ir_length;
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

		field.tap = etb->tap;
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
	int retval;
	if ((retval = etb_read_reg(reg)) != ERROR_OK)
	{
		LOG_ERROR("BUG: error scheduling etm register read");
		return retval;
	}

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("register read failed");
		return retval;
	}

	return ERROR_OK;
}

int etb_read_ram(etb_t *etb, u32 *data, int num_frames)
{
	scan_field_t fields[3];
	int i;

	jtag_add_end_state(TAP_IDLE);
	etb_scann(etb, 0x0);
	etb_set_instr(etb, 0xc);

	fields[0].tap = etb->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].tap = etb->tap;
	fields[1].num_bits = 7;
	fields[1].out_value = malloc(1);
	buf_set_u32(fields[1].out_value, 0, 7, 4);
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].tap = etb->tap;
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

	fields[0].in_handler = buf_to_u32_handler;

	for (i = 0; i < num_frames; i++)
	{
		/* ensure nR/W reamins set to read */
		buf_set_u32(fields[2].out_value, 0, 1, 0);

		/* address remains set to 0x4 (RAM data) until we read the last frame */
		if (i < num_frames - 1)
			buf_set_u32(fields[1].out_value, 0, 7, 4);
		else
			buf_set_u32(fields[1].out_value, 0, 7, 0);

		fields[0].in_handler_priv = &data[i];
		jtag_add_dr_scan(3, fields, -1);
	}

	jtag_execute_queue();

	free(fields[1].out_value);
	free(fields[2].out_value);

	return ERROR_OK;
}

int etb_read_reg_w_check(reg_t *reg, u8* check_value, u8* check_mask)
{
	etb_reg_t *etb_reg = reg->arch_info;
	u8 reg_addr = etb_reg->addr & 0x7f;
	scan_field_t fields[3];

	LOG_DEBUG("%i", etb_reg->addr);

	jtag_add_end_state(TAP_IDLE);
	etb_scann(etb_reg->etb, 0x0);
	etb_set_instr(etb_reg->etb, 0xc);

	fields[0].tap = etb_reg->etb->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = reg->value;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].tap = etb_reg->etb->tap;
	fields[1].num_bits = 7;
	fields[1].out_value = malloc(1);
	buf_set_u32(fields[1].out_value, 0, 7, reg_addr);
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].tap = etb_reg->etb->tap;
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

	/* read the identification register in the second run, to make sure we
	 * don't read the ETB data register twice, skipping every second entry
	 */
	buf_set_u32(fields[1].out_value, 0, 7, 0x0);
	fields[0].in_value = reg->value;

	jtag_set_check_value(fields+0, check_value, check_mask, NULL);

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
	int retval;
	if ((retval = etb_write_reg(reg, value)) != ERROR_OK)
	{
		LOG_ERROR("BUG: error scheduling etm register write");
		return retval;
	}

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->valid = 1;
	reg->dirty = 0;

	return ERROR_OK;
}

int etb_set_reg_w_exec(reg_t *reg, u8 *buf)
{
	int retval;
	etb_set_reg(reg, buf_get_u32(buf, 0, reg->size));

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("register write failed");
		return retval;
	}
	return ERROR_OK;
}

int etb_write_reg(reg_t *reg, u32 value)
{
	etb_reg_t *etb_reg = reg->arch_info;
	u8 reg_addr = etb_reg->addr & 0x7f;
	scan_field_t fields[3];

	LOG_DEBUG("%i: 0x%8.8x", etb_reg->addr, value);

	jtag_add_end_state(TAP_IDLE);
	etb_scann(etb_reg->etb, 0x0);
	etb_set_instr(etb_reg->etb, 0xc);

	fields[0].tap = etb_reg->etb->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = malloc(4);
	buf_set_u32(fields[0].out_value, 0, 32, value);
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].tap = etb_reg->etb->tap;
	fields[1].num_bits = 7;
	fields[1].out_value = malloc(1);
	buf_set_u32(fields[1].out_value, 0, 7, reg_addr);
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].tap = etb_reg->etb->tap;
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

int etb_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *etb_cmd;

	etb_cmd = register_command(cmd_ctx, NULL, "etb", NULL, COMMAND_ANY, "Embedded Trace Buffer");

	register_command(cmd_ctx, etb_cmd, "config", handle_etb_config_command, COMMAND_CONFIG, NULL);

	return ERROR_OK;
}

int handle_etb_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	jtag_tap_t *tap;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;

	if (argc != 2)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	target = get_target_by_num(strtoul(args[0], NULL, 0));

	if (!target)
	{
		LOG_ERROR("target number '%s' not defined", args[0]);
		return ERROR_FAIL;
	}

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_FAIL;
	}

	tap = jtag_TapByString( args[1] );
	if( tap == NULL ){
		command_print(cmd_ctx, "Tap: %s does not exist", args[1] );
		return ERROR_FAIL;
	}


	if (arm7_9->etm_ctx)
	{
		etb_t *etb = malloc(sizeof(etb_t));

		arm7_9->etm_ctx->capture_driver_priv = etb;

		etb->tap  = tap;
		etb->cur_scan_chain = -1;
		etb->reg_cache = NULL;
		etb->ram_width = 0;
		etb->ram_depth = 0;
	}
	else
	{
		LOG_ERROR("target has no ETM defined, ETB left unconfigured");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int etb_init(etm_context_t *etm_ctx)
{
	etb_t *etb = etm_ctx->capture_driver_priv;

	etb->etm_ctx = etm_ctx;

	/* identify ETB RAM depth and width */
	etb_read_reg(&etb->reg_cache->reg_list[ETB_RAM_DEPTH]);
	etb_read_reg(&etb->reg_cache->reg_list[ETB_RAM_WIDTH]);
	jtag_execute_queue();

	etb->ram_depth = buf_get_u32(etb->reg_cache->reg_list[ETB_RAM_DEPTH].value, 0, 32);
	etb->ram_width = buf_get_u32(etb->reg_cache->reg_list[ETB_RAM_WIDTH].value, 0, 32);

	return ERROR_OK;
}

trace_status_t etb_status(etm_context_t *etm_ctx)
{
	etb_t *etb = etm_ctx->capture_driver_priv;

	etb->etm_ctx = etm_ctx;

	/* if tracing is currently idle, return this information */
	if (etm_ctx->capture_status == TRACE_IDLE)
	{
		return etm_ctx->capture_status;
	}
	else if (etm_ctx->capture_status & TRACE_RUNNING)
	{
		reg_t *etb_status_reg = &etb->reg_cache->reg_list[ETB_STATUS];
		int etb_timeout = 100;

		/* trace is running, check the ETB status flags */
		etb_get_reg(etb_status_reg);

		/* check Full bit to identify an overflow */
		if (buf_get_u32(etb_status_reg->value, 0, 1) == 1)
			etm_ctx->capture_status |= TRACE_OVERFLOWED;

		/* check Triggered bit to identify trigger condition */
		if (buf_get_u32(etb_status_reg->value, 1, 1) == 1)
			etm_ctx->capture_status |= TRACE_TRIGGERED;

		/* check AcqComp to identify trace completion */
		if (buf_get_u32(etb_status_reg->value, 2, 1) == 1)
		{
			while (etb_timeout-- && (buf_get_u32(etb_status_reg->value, 3, 1) == 0))
			{
				/* wait for data formatter idle */
				etb_get_reg(etb_status_reg);
			}

			if (etb_timeout == 0)
			{
				LOG_ERROR("AcqComp set but DFEmpty won't go high, ETB status: 0x%x",
					buf_get_u32(etb_status_reg->value, 0, etb_status_reg->size));
			}

			if (!(etm_ctx->capture_status && TRACE_TRIGGERED))
			{
				LOG_ERROR("trace completed, but no trigger condition detected");
			}

			etm_ctx->capture_status &= ~TRACE_RUNNING;
			etm_ctx->capture_status |= TRACE_COMPLETED;
		}
	}

	return etm_ctx->capture_status;
}

int etb_read_trace(etm_context_t *etm_ctx)
{
	etb_t *etb = etm_ctx->capture_driver_priv;
	int first_frame = 0;
	int num_frames = etb->ram_depth;
	u32 *trace_data = NULL;
	int i, j;

	etb_read_reg(&etb->reg_cache->reg_list[ETB_STATUS]);
	etb_read_reg(&etb->reg_cache->reg_list[ETB_RAM_WRITE_POINTER]);
	jtag_execute_queue();

	/* check if we overflowed, and adjust first frame of the trace accordingly
	 * if we didn't overflow, read only up to the frame that would be written next,
	 * i.e. don't read invalid entries
	 */
	if (buf_get_u32(etb->reg_cache->reg_list[ETB_STATUS].value, 0, 1))
	{
		first_frame = buf_get_u32(etb->reg_cache->reg_list[ETB_RAM_WRITE_POINTER].value, 0, 32);
	}
	else
	{
		num_frames = buf_get_u32(etb->reg_cache->reg_list[ETB_RAM_WRITE_POINTER].value, 0, 32);
	}

	etb_write_reg(&etb->reg_cache->reg_list[ETB_RAM_READ_POINTER], first_frame);

	/* read data into temporary array for unpacking */
	trace_data = malloc(sizeof(u32) * num_frames);
	etb_read_ram(etb, trace_data, num_frames);

	if (etm_ctx->trace_depth > 0)
	{
		free(etm_ctx->trace_data);
	}

	if ((etm_ctx->portmode & ETM_PORT_WIDTH_MASK) == ETM_PORT_4BIT)
		etm_ctx->trace_depth = num_frames * 3;
	else if ((etm_ctx->portmode & ETM_PORT_WIDTH_MASK) == ETM_PORT_8BIT)
		etm_ctx->trace_depth = num_frames * 2;
	else
		etm_ctx->trace_depth = num_frames;

	etm_ctx->trace_data = malloc(sizeof(etmv1_trace_data_t) * etm_ctx->trace_depth);

	for (i = 0, j = 0; i < num_frames; i++)
	{
		if ((etm_ctx->portmode & ETM_PORT_WIDTH_MASK) == ETM_PORT_4BIT)
		{
			/* trace word j */
			etm_ctx->trace_data[j].pipestat = trace_data[i] & 0x7;
			etm_ctx->trace_data[j].packet = (trace_data[i] & 0x78) >> 3;
			etm_ctx->trace_data[j].flags = 0;
			if ((trace_data[i] & 0x80) >> 7)
			{
				etm_ctx->trace_data[j].flags |= ETMV1_TRACESYNC_CYCLE;
			}
			if (etm_ctx->trace_data[j].pipestat == STAT_TR)
			{
				etm_ctx->trace_data[j].pipestat = etm_ctx->trace_data[j].packet & 0x7;
				etm_ctx->trace_data[j].flags |= ETMV1_TRIGGER_CYCLE;
			}

			/* trace word j+1 */
			etm_ctx->trace_data[j+1].pipestat = (trace_data[i] & 0x100) >> 8;
			etm_ctx->trace_data[j+1].packet = (trace_data[i] & 0x7800) >> 11;
			etm_ctx->trace_data[j+1].flags = 0;
			if ((trace_data[i] & 0x8000) >> 15)
			{
				etm_ctx->trace_data[j+1].flags |= ETMV1_TRACESYNC_CYCLE;
			}
			if (etm_ctx->trace_data[j+1].pipestat == STAT_TR)
			{
				etm_ctx->trace_data[j+1].pipestat = etm_ctx->trace_data[j+1].packet & 0x7;
				etm_ctx->trace_data[j+1].flags |= ETMV1_TRIGGER_CYCLE;
			}

			/* trace word j+2 */
			etm_ctx->trace_data[j+2].pipestat = (trace_data[i] & 0x10000) >> 16;
			etm_ctx->trace_data[j+2].packet = (trace_data[i] & 0x780000) >> 19;
			etm_ctx->trace_data[j+2].flags = 0;
			if ((trace_data[i] & 0x800000) >> 23)
			{
				etm_ctx->trace_data[j+2].flags |= ETMV1_TRACESYNC_CYCLE;
			}
			if (etm_ctx->trace_data[j+2].pipestat == STAT_TR)
			{
				etm_ctx->trace_data[j+2].pipestat = etm_ctx->trace_data[j+2].packet & 0x7;
				etm_ctx->trace_data[j+2].flags |= ETMV1_TRIGGER_CYCLE;
			}

			j += 3;
		}
		else if ((etm_ctx->portmode & ETM_PORT_WIDTH_MASK) == ETM_PORT_8BIT)
		{
			/* trace word j */
			etm_ctx->trace_data[j].pipestat = trace_data[i] & 0x7;
			etm_ctx->trace_data[j].packet = (trace_data[i] & 0x7f8) >> 3;
			etm_ctx->trace_data[j].flags = 0;
			if ((trace_data[i] & 0x800) >> 11)
			{
				etm_ctx->trace_data[j].flags |= ETMV1_TRACESYNC_CYCLE;
			}
			if (etm_ctx->trace_data[j].pipestat == STAT_TR)
			{
				etm_ctx->trace_data[j].pipestat = etm_ctx->trace_data[j].packet & 0x7;
				etm_ctx->trace_data[j].flags |= ETMV1_TRIGGER_CYCLE;
			}

			/* trace word j+1 */
			etm_ctx->trace_data[j+1].pipestat = (trace_data[i] & 0x7000) >> 12;
			etm_ctx->trace_data[j+1].packet = (trace_data[i] & 0x7f8000) >> 15;
			etm_ctx->trace_data[j+1].flags = 0;
			if ((trace_data[i] & 0x800000) >> 23)
			{
				etm_ctx->trace_data[j+1].flags |= ETMV1_TRACESYNC_CYCLE;
			}
			if (etm_ctx->trace_data[j+1].pipestat == STAT_TR)
			{
				etm_ctx->trace_data[j+1].pipestat = etm_ctx->trace_data[j+1].packet & 0x7;
				etm_ctx->trace_data[j+1].flags |= ETMV1_TRIGGER_CYCLE;
			}

			j += 2;
		}
		else
		{
			/* trace word j */
			etm_ctx->trace_data[j].pipestat = trace_data[i] & 0x7;
			etm_ctx->trace_data[j].packet = (trace_data[i] & 0x7fff8) >> 3;
			etm_ctx->trace_data[j].flags = 0;
			if ((trace_data[i] & 0x80000) >> 19)
			{
				etm_ctx->trace_data[j].flags |= ETMV1_TRACESYNC_CYCLE;
			}
			if (etm_ctx->trace_data[j].pipestat == STAT_TR)
			{
				etm_ctx->trace_data[j].pipestat = etm_ctx->trace_data[j].packet & 0x7;
				etm_ctx->trace_data[j].flags |= ETMV1_TRIGGER_CYCLE;
			}

			j += 1;
		}
	}

	free(trace_data);

	return ERROR_OK;
}

int etb_start_capture(etm_context_t *etm_ctx)
{
	etb_t *etb = etm_ctx->capture_driver_priv;
	u32 etb_ctrl_value = 0x1;
	u32 trigger_count;

	if ((etm_ctx->portmode & ETM_PORT_MODE_MASK) == ETM_PORT_DEMUXED)
	{
		if ((etm_ctx->portmode & ETM_PORT_WIDTH_MASK) != ETM_PORT_8BIT)
		{
			LOG_ERROR("ETB can't run in demultiplexed mode with a 4 or 16 bit port");
			return ERROR_ETM_PORTMODE_NOT_SUPPORTED;
		}
		etb_ctrl_value |= 0x2;
	}

	if ((etm_ctx->portmode & ETM_PORT_MODE_MASK) == ETM_PORT_MUXED)
		return ERROR_ETM_PORTMODE_NOT_SUPPORTED;

	trigger_count = (etb->ram_depth * etm_ctx->trigger_percent) / 100;

	etb_write_reg(&etb->reg_cache->reg_list[ETB_TRIGGER_COUNTER], trigger_count);
	etb_write_reg(&etb->reg_cache->reg_list[ETB_RAM_WRITE_POINTER], 0x0);
	etb_write_reg(&etb->reg_cache->reg_list[ETB_CTRL], etb_ctrl_value);
	jtag_execute_queue();

	/* we're starting a new trace, initialize capture status */
	etm_ctx->capture_status = TRACE_RUNNING;

	return ERROR_OK;
}

int etb_stop_capture(etm_context_t *etm_ctx)
{
	etb_t *etb = etm_ctx->capture_driver_priv;
	reg_t *etb_ctrl_reg = &etb->reg_cache->reg_list[ETB_CTRL];

	etb_write_reg(etb_ctrl_reg, 0x0);
	jtag_execute_queue();

	/* trace stopped, just clear running flag, but preserve others */
	etm_ctx->capture_status &= ~TRACE_RUNNING;

	return ERROR_OK;
}

etm_capture_driver_t etb_capture_driver =
{
	.name = "etb",
	.register_commands = etb_register_commands,
	.init = etb_init,
	.status = etb_status,
	.start_capture = etb_start_capture,
	.stop_capture = etb_stop_capture,
	.read_trace = etb_read_trace,
};
