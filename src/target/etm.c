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

#include <string.h>

#include "etm.h"
#include "etb.h"

#include "armv4_5.h"
#include "arm7_9_common.h"
#include "arm_disassembler.h"
#include "arm_simulator.h"

#include "log.h"
#include "arm_jtag.h"
#include "types.h"
#include "binarybuffer.h"
#include "target.h"
#include "register.h"
#include "jtag.h"
#include "fileio.h"

#include <stdlib.h>

/* ETM register access functionality
 *
 */

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
	32, 32, 17, 8, 3, 9, 32, 16,
	17, 26, 25, 8, 17, 32, 32, 17,
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

command_t *etm_cmd = NULL;

reg_cache_t* etm_build_reg_cache(target_t *target, arm_jtag_t *jtag_info, etm_context_t *etm_ctx)
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

	/* the ETM might have an ETB connected */
	if (strcmp(etm_ctx->capture_driver->name, "etb") == 0)
	{
		etb_t *etb = etm_ctx->capture_driver_priv;

		if (!etb)
		{
			LOG_ERROR("etb selected as etm capture driver, but no ETB configured");
			return ERROR_OK;
		}

		reg_cache->next = etb_build_reg_cache(etb);

		etb->reg_cache = reg_cache->next;
	}


	return reg_cache;
}

int etm_setup(target_t *target)
{
	int retval;
	u32 etm_ctrl_value;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	etm_context_t *etm_ctx = arm7_9->etm_ctx;
	reg_t *etm_ctrl_reg = &arm7_9->etm_ctx->reg_cache->reg_list[ETM_CTRL];
	/* initialize some ETM control register settings */
	etm_get_reg(etm_ctrl_reg);
	etm_ctrl_value = buf_get_u32(etm_ctrl_reg->value, 0, etm_ctrl_reg->size);

	/* clear the ETM powerdown bit (0) */
	etm_ctrl_value &= ~0x1;

	/* configure port width (6:4), mode (17:16) and clocking (13) */
	etm_ctrl_value = (etm_ctrl_value &
		~ETM_PORT_WIDTH_MASK & ~ETM_PORT_MODE_MASK & ~ETM_PORT_CLOCK_MASK)
		| etm_ctx->portmode;

	buf_set_u32(etm_ctrl_reg->value, 0, etm_ctrl_reg->size, etm_ctrl_value);
	etm_store_reg(etm_ctrl_reg);

	if ((retval=jtag_execute_queue())!=ERROR_OK)
		return retval;

	if ((retval=etm_ctx->capture_driver->init(etm_ctx)) != ERROR_OK)
	{
		LOG_ERROR("ETM capture driver initialization failed");
		return retval;
	}
	return ERROR_OK;
}

int etm_get_reg(reg_t *reg)
{
	int retval;
	if ((retval = etm_read_reg(reg)) != ERROR_OK)
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

int etm_read_reg_w_check(reg_t *reg, u8* check_value, u8* check_mask)
{
	etm_reg_t *etm_reg = reg->arch_info;
	u8 reg_addr = etm_reg->addr & 0x7f;
	scan_field_t fields[3];

	LOG_DEBUG("%i", etm_reg->addr);

	jtag_add_end_state(TAP_IDLE);
	arm_jtag_scann(etm_reg->jtag_info, 0x6);
	arm_jtag_set_instr(etm_reg->jtag_info, etm_reg->jtag_info->intest_instr, NULL);

	fields[0].tap = etm_reg->jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = reg->value;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].tap = etm_reg->jtag_info->tap;
	fields[1].num_bits = 7;
	fields[1].out_value = malloc(1);
	buf_set_u32(fields[1].out_value, 0, 7, reg_addr);
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].tap = etm_reg->jtag_info->tap;
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
	jtag_set_check_value(fields+0, check_value, check_mask, NULL);

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
	int retval;
	if ((retval = etm_write_reg(reg, value)) != ERROR_OK)
	{
		LOG_ERROR("BUG: error scheduling etm register write");
		return retval;
	}

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->valid = 1;
	reg->dirty = 0;

	return ERROR_OK;
}

int etm_set_reg_w_exec(reg_t *reg, u8 *buf)
{
	int retval;
	etm_set_reg(reg, buf_get_u32(buf, 0, reg->size));

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("register write failed");
		return retval;
	}
	return ERROR_OK;
}

int etm_write_reg(reg_t *reg, u32 value)
{
	etm_reg_t *etm_reg = reg->arch_info;
	u8 reg_addr = etm_reg->addr & 0x7f;
	scan_field_t fields[3];

	LOG_DEBUG("%i: 0x%8.8x", etm_reg->addr, value);

	jtag_add_end_state(TAP_IDLE);
	arm_jtag_scann(etm_reg->jtag_info, 0x6);
	arm_jtag_set_instr(etm_reg->jtag_info, etm_reg->jtag_info->intest_instr, NULL);

	fields[0].tap = etm_reg->jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = malloc(4);
	buf_set_u32(fields[0].out_value, 0, 32, value);
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].tap = etm_reg->jtag_info->tap;
	fields[1].num_bits = 7;
	fields[1].out_value = malloc(1);
	buf_set_u32(fields[1].out_value, 0, 7, reg_addr);
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].tap = etm_reg->jtag_info->tap;
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

/* ETM trace analysis functionality
 *
 */
extern etm_capture_driver_t etb_capture_driver;
extern etm_capture_driver_t etm_dummy_capture_driver;
#if BUILD_OOCD_TRACE == 1
extern etm_capture_driver_t oocd_trace_capture_driver;
#endif

etm_capture_driver_t *etm_capture_drivers[] =
{
	&etb_capture_driver,
	&etm_dummy_capture_driver,
#if BUILD_OOCD_TRACE == 1
	&oocd_trace_capture_driver,
#endif
	NULL
};

char *etmv1v1_branch_reason_strings[] =
{
	"normal PC change",
	"tracing enabled",
	"trace restarted after overflow",
	"exit from debug",
	"periodic synchronization",
	"reserved",
	"reserved",
	"reserved",
};

int etm_read_instruction(etm_context_t *ctx, arm_instruction_t *instruction)
{
	int i;
	int section = -1;
	u32 size_read;
	u32 opcode;
	int retval;

	if (!ctx->image)
		return ERROR_TRACE_IMAGE_UNAVAILABLE;

	/* search for the section the current instruction belongs to */
	for (i = 0; i < ctx->image->num_sections; i++)
	{
		if ((ctx->image->sections[i].base_address <= ctx->current_pc) &&
			(ctx->image->sections[i].base_address + ctx->image->sections[i].size > ctx->current_pc))
		{
			section = i;
			break;
		}
	}

	if (section == -1)
	{
		/* current instruction couldn't be found in the image */
		return ERROR_TRACE_INSTRUCTION_UNAVAILABLE;
	}

	if (ctx->core_state == ARMV4_5_STATE_ARM)
	{
		u8 buf[4];
		if ((retval = image_read_section(ctx->image, section,
			ctx->current_pc - ctx->image->sections[section].base_address,
			4, buf, &size_read)) != ERROR_OK)
		{
			LOG_ERROR("error while reading instruction: %i", retval);
			return ERROR_TRACE_INSTRUCTION_UNAVAILABLE;
		}
		opcode = target_buffer_get_u32(ctx->target, buf);
		arm_evaluate_opcode(opcode, ctx->current_pc, instruction);
	}
	else if (ctx->core_state == ARMV4_5_STATE_THUMB)
	{
		u8 buf[2];
		if ((retval = image_read_section(ctx->image, section,
			ctx->current_pc - ctx->image->sections[section].base_address,
			2, buf, &size_read)) != ERROR_OK)
		{
			LOG_ERROR("error while reading instruction: %i", retval);
			return ERROR_TRACE_INSTRUCTION_UNAVAILABLE;
		}
		opcode = target_buffer_get_u16(ctx->target, buf);
		thumb_evaluate_opcode(opcode, ctx->current_pc, instruction);
	}
	else if (ctx->core_state == ARMV4_5_STATE_JAZELLE)
	{
		LOG_ERROR("BUG: tracing of jazelle code not supported");
		exit(-1);
	}
	else
	{
		LOG_ERROR("BUG: unknown core state encountered");
		exit(-1);
	}

	return ERROR_OK;
}

int etmv1_next_packet(etm_context_t *ctx, u8 *packet, int apo)
{
	while (ctx->data_index < ctx->trace_depth)
	{
		/* if the caller specified an address packet offset, skip until the
		 * we reach the n-th cycle marked with tracesync */
		if (apo > 0)
		{
			if (ctx->trace_data[ctx->data_index].flags & ETMV1_TRACESYNC_CYCLE)
				apo--;

			if (apo > 0)
			{
				ctx->data_index++;
				ctx->data_half = 0;
			}
			continue;
		}

		/* no tracedata output during a TD cycle
		 * or in a trigger cycle */
		if ((ctx->trace_data[ctx->data_index].pipestat == STAT_TD)
			|| (ctx->trace_data[ctx->data_index].flags & ETMV1_TRIGGER_CYCLE))
		{
			ctx->data_index++;
			ctx->data_half = 0;
			continue;
		}

		if ((ctx->portmode & ETM_PORT_WIDTH_MASK) == ETM_PORT_16BIT)
		{
			if (ctx->data_half == 0)
			{
				*packet = ctx->trace_data[ctx->data_index].packet & 0xff;
				ctx->data_half = 1;
			}
			else
			{
				*packet = (ctx->trace_data[ctx->data_index].packet & 0xff00) >> 8;
				ctx->data_half = 0;
				ctx->data_index++;
			}
		}
		else if ((ctx->portmode & ETM_PORT_WIDTH_MASK) == ETM_PORT_8BIT)
		{
			*packet = ctx->trace_data[ctx->data_index].packet & 0xff;
			ctx->data_index++;
		}
		else
		{
			/* on a 4-bit port, a packet will be output during two consecutive cycles */
			if (ctx->data_index > (ctx->trace_depth - 2))
				return -1;

			*packet = ctx->trace_data[ctx->data_index].packet & 0xf;
			*packet |= (ctx->trace_data[ctx->data_index + 1].packet & 0xf) << 4;
			ctx->data_index += 2;
		}

		return 0;
	}

	return -1;
}

int etmv1_branch_address(etm_context_t *ctx)
{
	int retval;
	u8 packet;
	int shift = 0;
	int apo;
	int i;

	/* quit analysis if less than two cycles are left in the trace
	 * because we can't extract the APO */
	if (ctx->data_index > (ctx->trace_depth - 2))
		return -1;

	/* a BE could be output during an APO cycle, skip the current
	 * and continue with the new one */
	if (ctx->trace_data[ctx->pipe_index + 1].pipestat & 0x4)
		return 1;
	if (ctx->trace_data[ctx->pipe_index + 2].pipestat & 0x4)
		return 2;

	/* address packet offset encoded in the next two cycles' pipestat bits */
	apo = ctx->trace_data[ctx->pipe_index + 1].pipestat & 0x3;
	apo |= (ctx->trace_data[ctx->pipe_index + 2].pipestat & 0x3) << 2;

	/* count number of tracesync cycles between current pipe_index and data_index
	 * i.e. the number of tracesyncs that data_index already passed by
	 * to subtract them from the APO */
	for (i = ctx->pipe_index; i < ctx->data_index; i++)
	{
		if (ctx->trace_data[ctx->pipe_index + 1].pipestat & ETMV1_TRACESYNC_CYCLE)
			apo--;
	}

	/* extract up to four 7-bit packets */
	do {
		if ((retval = etmv1_next_packet(ctx, &packet, (shift == 0) ? apo + 1 : 0)) != 0)
			return -1;
		ctx->last_branch &= ~(0x7f << shift);
		ctx->last_branch |= (packet & 0x7f) << shift;
		shift += 7;
	} while ((packet & 0x80) && (shift < 28));

	/* one last packet holding 4 bits of the address, plus the branch reason code */
	if ((shift == 28) && (packet & 0x80))
	{
		if ((retval = etmv1_next_packet(ctx, &packet, 0)) != 0)
			return -1;
		ctx->last_branch &= 0x0fffffff;
		ctx->last_branch |= (packet & 0x0f) << 28;
		ctx->last_branch_reason = (packet & 0x70) >> 4;
		shift += 4;
	}
	else
	{
		ctx->last_branch_reason = 0;
	}

	if (shift == 32)
	{
		ctx->pc_ok = 1;
	}

	/* if a full address was output, we might have branched into Jazelle state */
	if ((shift == 32) && (packet & 0x80))
	{
		ctx->core_state = ARMV4_5_STATE_JAZELLE;
	}
	else
	{
		/* if we didn't branch into Jazelle state, the current processor state is
		 * encoded in bit 0 of the branch target address */
		if (ctx->last_branch & 0x1)
		{
			ctx->core_state = ARMV4_5_STATE_THUMB;
			ctx->last_branch &= ~0x1;
		}
		else
		{
			ctx->core_state = ARMV4_5_STATE_ARM;
			ctx->last_branch &= ~0x3;
		}
	}

	return 0;
}

int etmv1_data(etm_context_t *ctx, int size, u32 *data)
{
	int j;
	u8 buf[4];
	int retval;

	for (j = 0; j < size; j++)
	{
		if ((retval = etmv1_next_packet(ctx, &buf[j], 0)) != 0)
			return -1;
	}

	if (size == 8)
	{
		LOG_ERROR("TODO: add support for 64-bit values");
		return -1;
	}
	else if (size == 4)
		*data = target_buffer_get_u32(ctx->target, buf);
	else if (size == 2)
		*data = target_buffer_get_u16(ctx->target, buf);
	else if (size == 1)
		*data = buf[0];
	else
		return -1;

	return 0;
}

int etmv1_analyze_trace(etm_context_t *ctx, struct command_context_s *cmd_ctx)
{
	int retval;
	arm_instruction_t instruction;

	/* read the trace data if it wasn't read already */
	if (ctx->trace_depth == 0)
		ctx->capture_driver->read_trace(ctx);

	/* start at the beginning of the captured trace */
	ctx->pipe_index = 0;
	ctx->data_index = 0;
	ctx->data_half = 0;

	/* neither the PC nor the data pointer are valid */
	ctx->pc_ok = 0;
	ctx->ptr_ok = 0;

	while (ctx->pipe_index < ctx->trace_depth)
	{
		u8 pipestat = ctx->trace_data[ctx->pipe_index].pipestat;
		u32 next_pc = ctx->current_pc;
		u32 old_data_index = ctx->data_index;
		u32 old_data_half = ctx->data_half;
		u32 old_index = ctx->pipe_index;
		u32 last_instruction = ctx->last_instruction;
		u32 cycles = 0;
		int current_pc_ok = ctx->pc_ok;

		if (ctx->trace_data[ctx->pipe_index].flags & ETMV1_TRIGGER_CYCLE)
		{
			command_print(cmd_ctx, "--- trigger ---");
		}

		/* instructions execute in IE/D or BE/D cycles */
		if ((pipestat == STAT_IE) || (pipestat == STAT_ID))
			ctx->last_instruction = ctx->pipe_index;

		/* if we don't have a valid pc skip until we reach an indirect branch */
		if ((!ctx->pc_ok) && (pipestat != STAT_BE))
		{
			ctx->pipe_index++;
			continue;
		}

		/* any indirect branch could have interrupted instruction flow
		 * - the branch reason code could indicate a trace discontinuity
		 * - a branch to the exception vectors indicates an exception
		 */
		if ((pipestat == STAT_BE) || (pipestat == STAT_BD))
		{
			/* backup current data index, to be able to consume the branch address
			 * before examining data address and values
			 */
			old_data_index = ctx->data_index;
			old_data_half = ctx->data_half;

			ctx->last_instruction = ctx->pipe_index;

			if ((retval = etmv1_branch_address(ctx)) != 0)
			{
				/* negative return value from etmv1_branch_address means we ran out of packets,
				 * quit analysing the trace */
				if (retval < 0)
					break;

				/* a positive return values means the current branch was abandoned,
				 * and a new branch was encountered in cycle ctx->pipe_index + retval;
				 */
				LOG_WARNING("abandoned branch encountered, correctnes of analysis uncertain");
				ctx->pipe_index += retval;
				continue;
			}

			/* skip over APO cycles */
			ctx->pipe_index += 2;

			switch (ctx->last_branch_reason)
			{
				case 0x0:	/* normal PC change */
					next_pc = ctx->last_branch;
					break;
				case 0x1:	/* tracing enabled */
					command_print(cmd_ctx, "--- tracing enabled at 0x%8.8x ---", ctx->last_branch);
					ctx->current_pc = ctx->last_branch;
					ctx->pipe_index++;
					continue;
					break;
				case 0x2:	/* trace restarted after FIFO overflow */
					command_print(cmd_ctx, "--- trace restarted after FIFO overflow at 0x%8.8x ---", ctx->last_branch);
					ctx->current_pc = ctx->last_branch;
					ctx->pipe_index++;
					continue;
					break;
				case 0x3:	/* exit from debug state */
					command_print(cmd_ctx, "--- exit from debug state at 0x%8.8x ---", ctx->last_branch);
					ctx->current_pc = ctx->last_branch;
					ctx->pipe_index++;
					continue;
					break;
				case 0x4:	/* periodic synchronization point */
					next_pc = ctx->last_branch;
					/* if we had no valid PC prior to this synchronization point,
					 * we have to move on with the next trace cycle
					 */
					if (!current_pc_ok)
					{
						command_print(cmd_ctx, "--- periodic synchronization point at 0x%8.8x ---", next_pc);
						ctx->current_pc = next_pc;
						ctx->pipe_index++;
						continue;
					}
					break;
				default:	/* reserved */
					LOG_ERROR("BUG: branch reason code 0x%x is reserved", ctx->last_branch_reason);
					exit(-1);
					break;
			}

			/* if we got here the branch was a normal PC change
			 * (or a periodic synchronization point, which means the same for that matter)
			 * if we didn't accquire a complete PC continue with the next cycle
			 */
			if (!ctx->pc_ok)
				continue;

			/* indirect branch to the exception vector means an exception occured */
			if (((ctx->last_branch >= 0x0) && (ctx->last_branch <= 0x20))
				|| ((ctx->last_branch >= 0xffff0000) && (ctx->last_branch <= 0xffff0020)))
			{
				if ((ctx->last_branch & 0xff) == 0x10)
				{
					command_print(cmd_ctx, "data abort");
				}
				else
				{
					command_print(cmd_ctx, "exception vector 0x%2.2x", ctx->last_branch);
					ctx->current_pc = ctx->last_branch;
					ctx->pipe_index++;
					continue;
				}
			}
		}

		/* an instruction was executed (or not, depending on the condition flags)
		 * retrieve it from the image for displaying */
		if (ctx->pc_ok && (pipestat != STAT_WT) && (pipestat != STAT_TD) &&
			!(((pipestat == STAT_BE) || (pipestat == STAT_BD)) &&
				((ctx->last_branch_reason != 0x0) && (ctx->last_branch_reason != 0x4))))
		{
			if ((retval = etm_read_instruction(ctx, &instruction)) != ERROR_OK)
			{
				/* can't continue tracing with no image available */
				if (retval == ERROR_TRACE_IMAGE_UNAVAILABLE)
				{
					return retval;
				}
				else if (retval == ERROR_TRACE_INSTRUCTION_UNAVAILABLE)
				{
					/* TODO: handle incomplete images
					 * for now we just quit the analsysis*/
					return retval;
				}
			}

			cycles = old_index - last_instruction;
		}

		if ((pipestat == STAT_ID) || (pipestat == STAT_BD))
		{
			u32 new_data_index = ctx->data_index;
			u32 new_data_half = ctx->data_half;

			/* in case of a branch with data, the branch target address was consumed before
			 * we temporarily go back to the saved data index */
			if (pipestat == STAT_BD)
			{
				ctx->data_index = old_data_index;
				ctx->data_half = old_data_half;
			}

			if (ctx->tracemode & ETMV1_TRACE_ADDR)
			{
				u8 packet;
				int shift = 0;

				do {
					if ((retval = etmv1_next_packet(ctx, &packet, 0)) != 0)
						return ERROR_ETM_ANALYSIS_FAILED;
					ctx->last_ptr &= ~(0x7f << shift);
					ctx->last_ptr |= (packet & 0x7f) << shift;
					shift += 7;
				} while ((packet & 0x80) && (shift < 32));

				if (shift >= 32)
					ctx->ptr_ok = 1;

				if (ctx->ptr_ok)
				{
					command_print(cmd_ctx, "address: 0x%8.8x", ctx->last_ptr);
				}
			}

			if (ctx->tracemode & ETMV1_TRACE_DATA)
			{
				if ((instruction.type == ARM_LDM) || (instruction.type == ARM_STM))
				{
					int i;
					for (i = 0; i < 16; i++)
					{
						if (instruction.info.load_store_multiple.register_list & (1 << i))
						{
							u32 data;
							if (etmv1_data(ctx, 4, &data) != 0)
								return ERROR_ETM_ANALYSIS_FAILED;
							command_print(cmd_ctx, "data: 0x%8.8x", data);
						}
					}
				}
				else if ((instruction.type >= ARM_LDR) && (instruction.type <= ARM_STRH))
				{
					u32 data;
					if (etmv1_data(ctx, arm_access_size(&instruction), &data) != 0)
						return ERROR_ETM_ANALYSIS_FAILED;
					command_print(cmd_ctx, "data: 0x%8.8x", data);
				}
			}

			/* restore data index after consuming BD address and data */
			if (pipestat == STAT_BD)
			{
				ctx->data_index = new_data_index;
				ctx->data_half = new_data_half;
			}
		}

		/* adjust PC */
		if ((pipestat == STAT_IE) || (pipestat == STAT_ID))
		{
			if (((instruction.type == ARM_B) ||
				(instruction.type == ARM_BL) ||
				(instruction.type == ARM_BLX)) &&
				(instruction.info.b_bl_bx_blx.target_address != -1))
			{
				next_pc = instruction.info.b_bl_bx_blx.target_address;
			}
			else
			{
				next_pc += (ctx->core_state == ARMV4_5_STATE_ARM) ? 4 : 2;
			}
		}
		else if (pipestat == STAT_IN)
		{
			next_pc += (ctx->core_state == ARMV4_5_STATE_ARM) ? 4 : 2;
		}

		if ((pipestat != STAT_TD) && (pipestat != STAT_WT))
		{
			char cycles_text[32] = "";

			/* if the trace was captured with cycle accurate tracing enabled,
			 * output the number of cycles since the last executed instruction
			 */
			if (ctx->tracemode & ETMV1_CYCLE_ACCURATE)
			{
				snprintf(cycles_text, 32, " (%i %s)",
					cycles,
					(cycles == 1) ? "cycle" : "cycles");
			}

			command_print(cmd_ctx, "%s%s%s",
				instruction.text,
				(pipestat == STAT_IN) ? " (not executed)" : "",
				cycles_text);

			ctx->current_pc = next_pc;

			/* packets for an instruction don't start on or before the preceding
			 * functional pipestat (i.e. other than WT or TD)
			 */
			if (ctx->data_index <= ctx->pipe_index)
			{
				ctx->data_index = ctx->pipe_index + 1;
				ctx->data_half = 0;
			}
		}

		ctx->pipe_index += 1;
	}

	return ERROR_OK;
}

int handle_etm_tracemode_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etmv1_tracemode_t tracemode;

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!arm7_9->etm_ctx)
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	tracemode = arm7_9->etm_ctx->tracemode;

	if (argc == 4)
	{
		if (strcmp(args[0], "none") == 0)
		{
			tracemode = ETMV1_TRACE_NONE;
		}
		else if (strcmp(args[0], "data") == 0)
		{
			tracemode = ETMV1_TRACE_DATA;
		}
		else if (strcmp(args[0], "address") == 0)
		{
			tracemode = ETMV1_TRACE_ADDR;
		}
		else if (strcmp(args[0], "all") == 0)
		{
			tracemode = ETMV1_TRACE_DATA | ETMV1_TRACE_ADDR;
		}
		else
		{
			command_print(cmd_ctx, "invalid option '%s'", args[0]);
			return ERROR_OK;
		}

		switch (strtol(args[1], NULL, 0))
		{
			case 0:
				tracemode |= ETMV1_CONTEXTID_NONE;
				break;
			case 8:
				tracemode |= ETMV1_CONTEXTID_8;
				break;
			case 16:
				tracemode |= ETMV1_CONTEXTID_16;
				break;
			case 32:
				tracemode |= ETMV1_CONTEXTID_32;
				break;
			default:
				command_print(cmd_ctx, "invalid option '%s'", args[1]);
				return ERROR_OK;
		}

		if (strcmp(args[2], "enable") == 0)
		{
			tracemode |= ETMV1_CYCLE_ACCURATE;
		}
		else if (strcmp(args[2], "disable") == 0)
		{
			tracemode |= 0;
		}
		else
		{
			command_print(cmd_ctx, "invalid option '%s'", args[2]);
			return ERROR_OK;
		}

		if (strcmp(args[3], "enable") == 0)
		{
			tracemode |= ETMV1_BRANCH_OUTPUT;
		}
		else if (strcmp(args[3], "disable") == 0)
		{
			tracemode |= 0;
		}
		else
		{
			command_print(cmd_ctx, "invalid option '%s'", args[2]);
			return ERROR_OK;
		}
	}
	else if (argc != 0)
	{
		command_print(cmd_ctx, "usage: configure trace mode <none|data|address|all> <context id bits> <cycle accurate> <branch output>");
		return ERROR_OK;
	}

	command_print(cmd_ctx, "current tracemode configuration:");

	switch (tracemode & ETMV1_TRACE_MASK)
	{
		case ETMV1_TRACE_NONE:
			command_print(cmd_ctx, "data tracing: none");
			break;
		case ETMV1_TRACE_DATA:
			command_print(cmd_ctx, "data tracing: data only");
			break;
		case ETMV1_TRACE_ADDR:
			command_print(cmd_ctx, "data tracing: address only");
			break;
		case ETMV1_TRACE_DATA | ETMV1_TRACE_ADDR:
			command_print(cmd_ctx, "data tracing: address and data");
			break;
	}

	switch (tracemode & ETMV1_CONTEXTID_MASK)
	{
		case ETMV1_CONTEXTID_NONE:
			command_print(cmd_ctx, "contextid tracing: none");
			break;
		case ETMV1_CONTEXTID_8:
			command_print(cmd_ctx, "contextid tracing: 8 bit");
			break;
		case ETMV1_CONTEXTID_16:
			command_print(cmd_ctx, "contextid tracing: 16 bit");
			break;
		case ETMV1_CONTEXTID_32:
			command_print(cmd_ctx, "contextid tracing: 32 bit");
			break;
	}

	if (tracemode & ETMV1_CYCLE_ACCURATE)
	{
		command_print(cmd_ctx, "cycle-accurate tracing enabled");
	}
	else
	{
		command_print(cmd_ctx, "cycle-accurate tracing disabled");
	}

	if (tracemode & ETMV1_BRANCH_OUTPUT)
	{
		command_print(cmd_ctx, "full branch address output enabled");
	}
	else
	{
		command_print(cmd_ctx, "full branch address output disabled");
	}

	/* only update ETM_CTRL register if tracemode changed */
	if (arm7_9->etm_ctx->tracemode != tracemode)
	{
		reg_t *etm_ctrl_reg = &arm7_9->etm_ctx->reg_cache->reg_list[ETM_CTRL];

		etm_get_reg(etm_ctrl_reg);

		buf_set_u32(etm_ctrl_reg->value, 2, 2, tracemode & ETMV1_TRACE_MASK);
		buf_set_u32(etm_ctrl_reg->value, 14, 2, (tracemode & ETMV1_CONTEXTID_MASK) >> 4);
		buf_set_u32(etm_ctrl_reg->value, 12, 1, (tracemode & ETMV1_CYCLE_ACCURATE) >> 8);
		buf_set_u32(etm_ctrl_reg->value, 8, 1, (tracemode & ETMV1_BRANCH_OUTPUT) >> 9);
		etm_store_reg(etm_ctrl_reg);

		arm7_9->etm_ctx->tracemode = tracemode;

		/* invalidate old trace data */
		arm7_9->etm_ctx->capture_status = TRACE_IDLE;
		if (arm7_9->etm_ctx->trace_depth > 0)
		{
			free(arm7_9->etm_ctx->trace_data);
			arm7_9->etm_ctx->trace_data = NULL;
		}
		arm7_9->etm_ctx->trace_depth = 0;
	}

	return ERROR_OK;
}

int handle_etm_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_portmode_t portmode = 0x0;
	etm_context_t *etm_ctx = malloc(sizeof(etm_context_t));
	int i;

	if (argc != 5)
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

	switch (strtoul(args[1], NULL, 0))
	{
		case 4:
			portmode |= ETM_PORT_4BIT;
			break;
		case 8:
			portmode |= ETM_PORT_8BIT;
			break;
		case 16:
			portmode |= ETM_PORT_16BIT;
			break;
		default:
			command_print(cmd_ctx, "unsupported ETM port width '%s', must be 4, 8 or 16", args[1]);
			return ERROR_FAIL;
	}

	if (strcmp("normal", args[2]) == 0)
	{
		portmode |= ETM_PORT_NORMAL;
	}
	else if (strcmp("multiplexed", args[2]) == 0)
	{
		portmode |= ETM_PORT_MUXED;
	}
	else if (strcmp("demultiplexed", args[2]) == 0)
	{
		portmode |= ETM_PORT_DEMUXED;
	}
	else
	{
		command_print(cmd_ctx, "unsupported ETM port mode '%s', must be 'normal', 'multiplexed' or 'demultiplexed'", args[2]);
		return ERROR_FAIL;
	}

	if (strcmp("half", args[3]) == 0)
	{
		portmode |= ETM_PORT_HALF_CLOCK;
	}
	else if (strcmp("full", args[3]) == 0)
	{
		portmode |= ETM_PORT_FULL_CLOCK;
	}
	else
	{
		command_print(cmd_ctx, "unsupported ETM port clocking '%s', must be 'full' or 'half'", args[3]);
		return ERROR_FAIL;
	}

	for (i=0; etm_capture_drivers[i]; i++)
	{
		if (strcmp(args[4], etm_capture_drivers[i]->name) == 0)
		{
			int retval;
			if ((retval=etm_capture_drivers[i]->register_commands(cmd_ctx)) != ERROR_OK)
			{
				free(etm_ctx);
				return retval;
			}

			etm_ctx->capture_driver = etm_capture_drivers[i];

			break;
		}
	}

	if (!etm_capture_drivers[i])
	{
		/* no supported capture driver found, don't register an ETM */
		free(etm_ctx);
		LOG_ERROR("trace capture driver '%s' not found", args[4]);
		return ERROR_FAIL;
	}

	etm_ctx->target = target;
	etm_ctx->trigger_percent = 50;
	etm_ctx->trace_data = NULL;
	etm_ctx->trace_depth = 0;
	etm_ctx->portmode = portmode;
	etm_ctx->tracemode = 0x0;
	etm_ctx->core_state = ARMV4_5_STATE_ARM;
	etm_ctx->image = NULL;
	etm_ctx->pipe_index = 0;
	etm_ctx->data_index = 0;
	etm_ctx->current_pc = 0x0;
	etm_ctx->pc_ok = 0;
	etm_ctx->last_branch = 0x0;
	etm_ctx->last_branch_reason = 0x0;
	etm_ctx->last_ptr = 0x0;
	etm_ctx->ptr_ok = 0x0;
	etm_ctx->context_id = 0x0;
	etm_ctx->last_instruction = 0;

	arm7_9->etm_ctx = etm_ctx;

	return etm_register_user_commands(cmd_ctx);
}

int handle_etm_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	reg_t *etm_config_reg;
	reg_t *etm_sys_config_reg;

	int max_port_size;

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!arm7_9->etm_ctx)
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	etm_config_reg = &arm7_9->etm_ctx->reg_cache->reg_list[ETM_CONFIG];
	etm_sys_config_reg = &arm7_9->etm_ctx->reg_cache->reg_list[ETM_SYS_CONFIG];

	etm_get_reg(etm_config_reg);
	command_print(cmd_ctx, "pairs of address comparators: %i", buf_get_u32(etm_config_reg->value, 0, 4));
	command_print(cmd_ctx, "pairs of data comparators: %i", buf_get_u32(etm_config_reg->value, 4, 4));
	command_print(cmd_ctx, "memory map decoders: %i", buf_get_u32(etm_config_reg->value, 8, 5));
	command_print(cmd_ctx, "number of counters: %i", buf_get_u32(etm_config_reg->value, 13, 3));
	command_print(cmd_ctx, "sequencer %spresent",
			(buf_get_u32(etm_config_reg->value, 16, 1) == 1) ? "" : "not ");
	command_print(cmd_ctx, "number of ext. inputs: %i", buf_get_u32(etm_config_reg->value, 17, 3));
	command_print(cmd_ctx, "number of ext. outputs: %i", buf_get_u32(etm_config_reg->value, 20, 3));
	command_print(cmd_ctx, "FIFO full %spresent",
			(buf_get_u32(etm_config_reg->value, 23, 1) == 1) ? "" : "not ");
	command_print(cmd_ctx, "protocol version: %i", buf_get_u32(etm_config_reg->value, 28, 3));

	etm_get_reg(etm_sys_config_reg);

	switch (buf_get_u32(etm_sys_config_reg->value, 0, 3))
	{
		case 0:
			max_port_size = 4;
			break;
		case 1:
			max_port_size = 8;
			break;
		case 2:
			max_port_size = 16;
			break;
		default:
			LOG_ERROR("Illegal max_port_size");
			exit(-1);
	}
	command_print(cmd_ctx, "max. port size: %i", max_port_size);

	command_print(cmd_ctx, "half-rate clocking %ssupported",
			(buf_get_u32(etm_sys_config_reg->value, 3, 1) == 1) ? "" : "not ");
	command_print(cmd_ctx, "full-rate clocking %ssupported",
			(buf_get_u32(etm_sys_config_reg->value, 4, 1) == 1) ? "" : "not ");
	command_print(cmd_ctx, "normal trace format %ssupported",
			(buf_get_u32(etm_sys_config_reg->value, 5, 1) == 1) ? "" : "not ");
	command_print(cmd_ctx, "multiplex trace format %ssupported",
			(buf_get_u32(etm_sys_config_reg->value, 6, 1) == 1) ? "" : "not ");
	command_print(cmd_ctx, "demultiplex trace format %ssupported",
			(buf_get_u32(etm_sys_config_reg->value, 7, 1) == 1) ? "" : "not ");
	command_print(cmd_ctx, "FIFO full %ssupported",
			(buf_get_u32(etm_sys_config_reg->value, 8, 1) == 1) ? "" : "not ");

	return ERROR_OK;
}

int handle_etm_status_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	trace_status_t trace_status;

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!arm7_9->etm_ctx)
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	trace_status = arm7_9->etm_ctx->capture_driver->status(arm7_9->etm_ctx);

	if (trace_status == TRACE_IDLE)
	{
		command_print(cmd_ctx, "tracing is idle");
	}
	else
	{
		static char *completed = " completed";
		static char *running = " is running";
		static char *overflowed = ", trace overflowed";
		static char *triggered = ", trace triggered";

		command_print(cmd_ctx, "trace collection%s%s%s",
			(trace_status & TRACE_RUNNING) ? running : completed,
			(trace_status & TRACE_OVERFLOWED) ? overflowed : "",
			(trace_status & TRACE_TRIGGERED) ? triggered : "");

		if (arm7_9->etm_ctx->trace_depth > 0)
		{
			command_print(cmd_ctx, "%i frames of trace data read", arm7_9->etm_ctx->trace_depth);
		}
	}

	return ERROR_OK;
}

int handle_etm_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm_ctx;

	if (argc < 1)
	{
		command_print(cmd_ctx, "usage: etm image <file> [base address] [type]");
		return ERROR_OK;
	}

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!(etm_ctx = arm7_9->etm_ctx))
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	if (etm_ctx->image)
	{
		image_close(etm_ctx->image);
		free(etm_ctx->image);
		command_print(cmd_ctx, "previously loaded image found and closed");
	}

	etm_ctx->image = malloc(sizeof(image_t));
	etm_ctx->image->base_address_set = 0;
	etm_ctx->image->start_address_set = 0;

	/* a base address isn't always necessary, default to 0x0 (i.e. don't relocate) */
	if (argc >= 2)
	{
		etm_ctx->image->base_address_set = 1;
		etm_ctx->image->base_address = strtoul(args[1], NULL, 0);
	}
	else
	{
		etm_ctx->image->base_address_set = 0;
	}

	if (image_open(etm_ctx->image, args[0], (argc >= 3) ? args[2] : NULL) != ERROR_OK)
	{
		free(etm_ctx->image);
		etm_ctx->image = NULL;
		return ERROR_OK;
	}

	return ERROR_OK;
}

int handle_etm_dump_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	fileio_t file;
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm_ctx;
	int i;

	if (argc != 1)
	{
		command_print(cmd_ctx, "usage: etm dump <file>");
		return ERROR_OK;
	}

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!(etm_ctx = arm7_9->etm_ctx))
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	if (etm_ctx->capture_driver->status == TRACE_IDLE)
	{
		command_print(cmd_ctx, "trace capture wasn't enabled, no trace data captured");
		return ERROR_OK;
	}

	if (etm_ctx->capture_driver->status(etm_ctx) & TRACE_RUNNING)
	{
		/* TODO: if on-the-fly capture is to be supported, this needs to be changed */
		command_print(cmd_ctx, "trace capture not completed");
		return ERROR_OK;
	}

	/* read the trace data if it wasn't read already */
	if (etm_ctx->trace_depth == 0)
		etm_ctx->capture_driver->read_trace(etm_ctx);

	if (fileio_open(&file, args[0], FILEIO_WRITE, FILEIO_BINARY) != ERROR_OK)
	{
		return ERROR_OK;
	}

	fileio_write_u32(&file, etm_ctx->capture_status);
	fileio_write_u32(&file, etm_ctx->portmode);
	fileio_write_u32(&file, etm_ctx->tracemode);
	fileio_write_u32(&file, etm_ctx->trace_depth);

	for (i = 0; i < etm_ctx->trace_depth; i++)
	{
		fileio_write_u32(&file, etm_ctx->trace_data[i].pipestat);
		fileio_write_u32(&file, etm_ctx->trace_data[i].packet);
		fileio_write_u32(&file, etm_ctx->trace_data[i].flags);
	}

	fileio_close(&file);

	return ERROR_OK;
}

int handle_etm_load_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	fileio_t file;
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm_ctx;
	int i;

	if (argc != 1)
	{
		command_print(cmd_ctx, "usage: etm load <file>");
		return ERROR_OK;
	}

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!(etm_ctx = arm7_9->etm_ctx))
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	if (etm_ctx->capture_driver->status(etm_ctx) & TRACE_RUNNING)
	{
		command_print(cmd_ctx, "trace capture running, stop first");
		return ERROR_OK;
	}

	if (fileio_open(&file, args[0], FILEIO_READ, FILEIO_BINARY) != ERROR_OK)
	{
		return ERROR_OK;
	}

	if (file.size % 4)
	{
		command_print(cmd_ctx, "size isn't a multiple of 4, no valid trace data");
		fileio_close(&file);
		return ERROR_OK;
	}

	if (etm_ctx->trace_depth > 0)
	{
		free(etm_ctx->trace_data);
		etm_ctx->trace_data = NULL;
	}

	fileio_read_u32(&file, &etm_ctx->capture_status);
	fileio_read_u32(&file, &etm_ctx->portmode);
	fileio_read_u32(&file, &etm_ctx->tracemode);
	fileio_read_u32(&file, &etm_ctx->trace_depth);

	etm_ctx->trace_data = malloc(sizeof(etmv1_trace_data_t) * etm_ctx->trace_depth);
	if(etm_ctx->trace_data == NULL)
	{
		command_print(cmd_ctx, "not enough memory to perform operation");
		fileio_close(&file);
		return ERROR_OK;
	}

	for (i = 0; i < etm_ctx->trace_depth; i++)
	{
		u32 pipestat, packet, flags;
		fileio_read_u32(&file, &pipestat);
		fileio_read_u32(&file, &packet);
		fileio_read_u32(&file, &flags);
		etm_ctx->trace_data[i].pipestat = pipestat & 0xff;
		etm_ctx->trace_data[i].packet = packet & 0xffff;
		etm_ctx->trace_data[i].flags = flags;
	}

	fileio_close(&file);

	return ERROR_OK;
}

int handle_etm_trigger_percent_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm_ctx;

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!(etm_ctx = arm7_9->etm_ctx))
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	if (argc > 0)
	{
		u32 new_value = strtoul(args[0], NULL, 0);

		if ((new_value < 2) || (new_value > 100))
		{
			command_print(cmd_ctx, "valid settings are 2% to 100%");
		}
		else
		{
			etm_ctx->trigger_percent = new_value;
		}
	}

	command_print(cmd_ctx, "%i percent of the tracebuffer reserved for after the trigger", etm_ctx->trigger_percent);

	return ERROR_OK;
}

int handle_etm_start_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm_ctx;
	reg_t *etm_ctrl_reg;

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!(etm_ctx = arm7_9->etm_ctx))
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	/* invalidate old tracing data */
	arm7_9->etm_ctx->capture_status = TRACE_IDLE;
	if (arm7_9->etm_ctx->trace_depth > 0)
	{
		free(arm7_9->etm_ctx->trace_data);
		arm7_9->etm_ctx->trace_data = NULL;
	}
	arm7_9->etm_ctx->trace_depth = 0;

	etm_ctrl_reg = &arm7_9->etm_ctx->reg_cache->reg_list[ETM_CTRL];
	etm_get_reg(etm_ctrl_reg);

	/* Clear programming bit (10), set port selection bit (11) */
	buf_set_u32(etm_ctrl_reg->value, 10, 2, 0x2);

	etm_store_reg(etm_ctrl_reg);
	jtag_execute_queue();

	etm_ctx->capture_driver->start_capture(etm_ctx);

	return ERROR_OK;
}

int handle_etm_stop_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm_ctx;
	reg_t *etm_ctrl_reg;

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!(etm_ctx = arm7_9->etm_ctx))
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	etm_ctrl_reg = &arm7_9->etm_ctx->reg_cache->reg_list[ETM_CTRL];
	etm_get_reg(etm_ctrl_reg);

	/* Set programming bit (10), clear port selection bit (11) */
	buf_set_u32(etm_ctrl_reg->value, 10, 2, 0x1);

	etm_store_reg(etm_ctrl_reg);
	jtag_execute_queue();

	etm_ctx->capture_driver->stop_capture(etm_ctx);

	return ERROR_OK;
}

int handle_etm_analyze_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm_ctx;
	int retval;

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!(etm_ctx = arm7_9->etm_ctx))
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	if ((retval = etmv1_analyze_trace(etm_ctx, cmd_ctx)) != ERROR_OK)
	{
		switch(retval)
		{
			case ERROR_ETM_ANALYSIS_FAILED:
				command_print(cmd_ctx, "further analysis failed (corrupted trace data or just end of data");
				break;
			case ERROR_TRACE_INSTRUCTION_UNAVAILABLE:
				command_print(cmd_ctx, "no instruction for current address available, analysis aborted");
				break;
			case ERROR_TRACE_IMAGE_UNAVAILABLE:
				command_print(cmd_ctx, "no image available for trace analysis");
				break;
			default:
				command_print(cmd_ctx, "unknown error: %i", retval);
		}
	}

	return ERROR_OK;
}

int etm_register_commands(struct command_context_s *cmd_ctx)
{
	etm_cmd = register_command(cmd_ctx, NULL, "etm", NULL, COMMAND_ANY, "Embedded Trace Macrocell");

	register_command(cmd_ctx, etm_cmd, "config", handle_etm_config_command, COMMAND_CONFIG, "etm config <target> <port_width> <port_mode> <clocking> <capture_driver>");

	return ERROR_OK;
}

int etm_register_user_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, etm_cmd, "tracemode", handle_etm_tracemode_command,
		COMMAND_EXEC, "configure trace mode <none|data|address|all> <context id bits> <cycle accurate> <branch output");

	register_command(cmd_ctx, etm_cmd, "info", handle_etm_info_command,
		COMMAND_EXEC, "display info about the current target's ETM");

	register_command(cmd_ctx, etm_cmd, "trigger_percent <percent>", handle_etm_trigger_percent_command,
		COMMAND_EXEC, "amount (<percent>) of trace buffer to be filled after the trigger occured");
	register_command(cmd_ctx, etm_cmd, "status", handle_etm_status_command,
		COMMAND_EXEC, "display current target's ETM status");
	register_command(cmd_ctx, etm_cmd, "start", handle_etm_start_command,
		COMMAND_EXEC, "start ETM trace collection");
	register_command(cmd_ctx, etm_cmd, "stop", handle_etm_stop_command,
		COMMAND_EXEC, "stop ETM trace collection");

	register_command(cmd_ctx, etm_cmd, "analyze", handle_etm_analyze_command,
		COMMAND_EXEC, "anaylze collected ETM trace");

	register_command(cmd_ctx, etm_cmd, "image", handle_etm_image_command,
		COMMAND_EXEC, "load image from <file> [base address]");

	register_command(cmd_ctx, etm_cmd, "dump", handle_etm_dump_command,
		COMMAND_EXEC, "dump captured trace data <file>");
	register_command(cmd_ctx, etm_cmd, "load", handle_etm_load_command,
		COMMAND_EXEC, "load trace data for analysis <file>");

	return ERROR_OK;
}
