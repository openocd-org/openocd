/***************************************************************************
 *   Copyright (C) 2006, 2007 by Dominic Rath                              *
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

#include "replacements.h"

#include "xscale.h"

#include "register.h"
#include "target.h"
#include "armv4_5.h"
#include "arm_simulator.h"
#include "arm_disassembler.h"
#include "log.h"
#include "jtag.h"
#include "binarybuffer.h"
#include "time_support.h"
#include "breakpoints.h"
#include "fileio.h"

#include <stdlib.h>
#include <string.h>

#include <sys/types.h>
#include <unistd.h>
#include <errno.h>


/* cli handling */
int xscale_register_commands(struct command_context_s *cmd_ctx);

/* forward declarations */
int xscale_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct target_s *target);
int xscale_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int xscale_quit();

int xscale_arch_state(struct target_s *target, char *buf, int buf_size);
enum target_state xscale_poll(target_t *target);
int xscale_halt(target_t *target);
int xscale_resume(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution);
int xscale_step(struct target_s *target, int current, u32 address, int handle_breakpoints);
int xscale_debug_entry(target_t *target);
int xscale_restore_context(target_t *target);

int xscale_assert_reset(target_t *target);
int xscale_deassert_reset(target_t *target);
int xscale_soft_reset_halt(struct target_s *target);
int xscale_prepare_reset_halt(struct target_s *target);

int xscale_set_reg_u32(reg_t *reg, u32 value);

int xscale_read_core_reg(struct target_s *target, int num, enum armv4_5_mode mode);
int xscale_write_core_reg(struct target_s *target, int num, enum armv4_5_mode mode, u32 value);

int xscale_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int xscale_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int xscale_bulk_write_memory(target_t *target, u32 address, u32 count, u8 *buffer);

int xscale_add_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
int xscale_remove_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
int xscale_set_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
int xscale_unset_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
int xscale_add_watchpoint(struct target_s *target, watchpoint_t *watchpoint);
int xscale_remove_watchpoint(struct target_s *target, watchpoint_t *watchpoint);
void xscale_enable_watchpoints(struct target_s *target);
void xscale_enable_breakpoints(struct target_s *target);

target_type_t xscale_target =
{
	.name = "xscale",

	.poll = xscale_poll,
	.arch_state = xscale_arch_state,

	.halt = xscale_halt,
	.resume = xscale_resume,
	.step = xscale_step,

	.assert_reset = xscale_assert_reset,
	.deassert_reset = xscale_deassert_reset,
	.soft_reset_halt = xscale_soft_reset_halt,
	.prepare_reset_halt = xscale_prepare_reset_halt,

	.get_gdb_reg_list = armv4_5_get_gdb_reg_list,
	
	.read_memory = xscale_read_memory,
	.write_memory = xscale_write_memory,
	.bulk_write_memory = xscale_bulk_write_memory,

	.run_algorithm = armv4_5_run_algorithm,
	
	.add_breakpoint = xscale_add_breakpoint,
	.remove_breakpoint = xscale_remove_breakpoint,
	.add_watchpoint = xscale_add_watchpoint,
	.remove_watchpoint = xscale_remove_watchpoint,

	.register_commands = xscale_register_commands,
	.target_command = xscale_target_command,
	.init_target = xscale_init_target,
	.quit = xscale_quit
};

char* xscale_reg_list[] =
{
	"XSCALE_MAINID",		/* 0 */
	"XSCALE_CACHETYPE",
	"XSCALE_CTRL",
	"XSCALE_AUXCTRL",
	"XSCALE_TTB",
	"XSCALE_DAC",
	"XSCALE_FSR",
	"XSCALE_FAR",
	"XSCALE_PID",
	"XSCALE_CPACCESS",
	"XSCALE_IBCR0",			/* 10 */
	"XSCALE_IBCR1",
	"XSCALE_DBR0",
	"XSCALE_DBR1",
	"XSCALE_DBCON",
	"XSCALE_TBREG",
	"XSCALE_CHKPT0",		
	"XSCALE_CHKPT1",
	"XSCALE_DCSR",
	"XSCALE_TX",
	"XSCALE_RX",			/* 20 */
	"XSCALE_TXRXCTRL",
};

xscale_reg_t xscale_reg_arch_info[] =
{
	{XSCALE_MAINID, NULL},
	{XSCALE_CACHETYPE, NULL},
	{XSCALE_CTRL, NULL},
	{XSCALE_AUXCTRL, NULL},
	{XSCALE_TTB, NULL},
	{XSCALE_DAC, NULL},
	{XSCALE_FSR, NULL},
	{XSCALE_FAR, NULL},
	{XSCALE_PID, NULL},
	{XSCALE_CPACCESS, NULL},
	{XSCALE_IBCR0, NULL},
	{XSCALE_IBCR1, NULL},
	{XSCALE_DBR0, NULL},
	{XSCALE_DBR1, NULL},
	{XSCALE_DBCON, NULL},
	{XSCALE_TBREG, NULL},
	{XSCALE_CHKPT0, NULL},
	{XSCALE_CHKPT1, NULL},
	{XSCALE_DCSR, NULL}, /* DCSR accessed via JTAG or SW */
	{-1, NULL}, /* TX accessed via JTAG */
	{-1, NULL}, /* RX accessed via JTAG */
	{-1, NULL}, /* TXRXCTRL implicit access via JTAG */
};

int xscale_reg_arch_type = -1;

int xscale_get_reg(reg_t *reg);
int xscale_set_reg(reg_t *reg, u8 *buf);

int xscale_get_arch_pointers(target_t *target, armv4_5_common_t **armv4_5_p, xscale_common_t **xscale_p)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	if (armv4_5->common_magic != ARMV4_5_COMMON_MAGIC)
	{
		return -1;
	}
	
	if (xscale->common_magic != XSCALE_COMMON_MAGIC)
	{
		return -1;
	}
	
	*armv4_5_p = armv4_5;
	*xscale_p = xscale;
	
	return ERROR_OK;
}

int xscale_jtag_set_instr(int chain_pos, u32 new_instr)
{
	jtag_device_t *device = jtag_get_device(chain_pos);
	
	if (buf_get_u32(device->cur_instr, 0, device->ir_length) != new_instr)
	{
		scan_field_t field;
	
		field.device = chain_pos;
		field.num_bits = device->ir_length;
		field.out_value = calloc(CEIL(field.num_bits, 8), 1);
		buf_set_u32(field.out_value, 0, field.num_bits, new_instr);
		field.out_mask = NULL;
		field.in_value = NULL;
		field.in_check_value = device->expected;
		field.in_check_mask = device->expected_mask;
		field.in_handler = NULL;
		field.in_handler_priv = NULL;
		
		jtag_add_ir_scan(1, &field, -1, NULL);
		
		free(field.out_value);
	}
	
	return ERROR_OK;
}

int xscale_jtag_callback(enum jtag_event event, void *priv)
{
	switch (event)
	{
		case JTAG_TRST_ASSERTED:
			break;
		case JTAG_TRST_RELEASED:
			break;
		case JTAG_SRST_ASSERTED:
			break;
		case JTAG_SRST_RELEASED:
			break;
		default:
			WARNING("unhandled JTAG event");
	}
	
	return ERROR_OK;
}

int xscale_read_dcsr(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	int retval;
	
	scan_field_t fields[3];
	u8 field0 = 0x0;
	u8 field0_check_value = 0x2;
	u8 field0_check_mask = 0x7;
	u8 field2 = 0x0;
	u8 field2_check_value = 0x0;
	u8 field2_check_mask = 0x1;

	jtag_add_end_state(TAP_PD);
	xscale_jtag_set_instr(xscale->jtag_info.chain_pos, xscale->jtag_info.dcsr);
	
	buf_set_u32(&field0, 1, 1, xscale->hold_rst);
	buf_set_u32(&field0, 2, 1, xscale->external_debug_break);
	
	fields[0].device = xscale->jtag_info.chain_pos;
	fields[0].num_bits = 3;
	fields[0].out_value = &field0;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = &field0_check_value;
	fields[0].in_check_mask = &field0_check_mask;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
		
	fields[1].device = xscale->jtag_info.chain_pos;
	fields[1].num_bits = 32;
	fields[1].out_value = NULL;
	fields[1].out_mask = NULL;
	fields[1].in_value = xscale->reg_cache->reg_list[XSCALE_DCSR].value;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;

	fields[2].device = xscale->jtag_info.chain_pos;
	fields[2].num_bits = 1;
	fields[2].out_value = &field2;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = &field2_check_value;
	fields[2].in_check_mask = &field2_check_mask;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;
	
	jtag_add_dr_scan(3, fields, -1, NULL);

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		ERROR("JTAG error while reading DCSR");
		exit(-1);
	}
	
	xscale->reg_cache->reg_list[XSCALE_DCSR].dirty = 0;
	xscale->reg_cache->reg_list[XSCALE_DCSR].valid = 1;
	
	/* write the register with the value we just read
	 * on this second pass, only the first bit of field0 is guaranteed to be 0)
	 */
	field0_check_mask = 0x1;
	fields[1].out_value = xscale->reg_cache->reg_list[XSCALE_DCSR].value;
	fields[1].in_value = NULL;
	
	jtag_add_end_state(TAP_RTI);
	
	jtag_add_dr_scan(3, fields, -1, NULL);
	
	return ERROR_OK;
}

int xscale_receive(target_t *target, u32 *buffer, int num_words)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	enum tap_state path[3];
	scan_field_t fields[3];
	
	u8 *field0 = malloc(num_words * 1);
	u8 field0_check_value = 0x2;
	u8 field0_check_mask = 0x6;
	u32 *field1 = malloc(num_words * 4);
	u8 field2_check_value = 0x0;
	u8 field2_check_mask = 0x1;
	int words_done = 0;
	int words_scheduled = 0;
	
	int i;
	int retval;

	path[0] = TAP_SDS;
	path[1] = TAP_CD;
	path[2] = TAP_SD;
	
	fields[0].device = xscale->jtag_info.chain_pos;
	fields[0].num_bits = 3;
	fields[0].out_value = NULL;
	fields[0].out_mask = NULL;
	/* fields[0].in_value = field0; */
	fields[0].in_check_value = &field0_check_value;
	fields[0].in_check_mask = &field0_check_mask;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
		
	fields[1].device = xscale->jtag_info.chain_pos;
	fields[1].num_bits = 32;
	fields[1].out_value = NULL;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;

	fields[2].device = xscale->jtag_info.chain_pos;
	fields[2].num_bits = 1;
	fields[2].out_value = NULL;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = &field2_check_value;
	fields[2].in_check_mask = &field2_check_mask;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;

	jtag_add_end_state(TAP_RTI);
	xscale_jtag_set_instr(xscale->jtag_info.chain_pos, xscale->jtag_info.dbgtx);
	jtag_add_runtest(1, -1);
	
	/* repeat until all words have been collected */
	while (words_done < num_words)
	{
		/* schedule reads */
		words_scheduled = 0;
		for (i = words_done; i < num_words; i++)
		{
			fields[0].in_value = &field0[i];
			fields[1].in_handler = buf_to_u32_handler;
			fields[1].in_handler_priv = (u8*)&field1[i];
			
			jtag_add_pathmove(3, path);
			jtag_add_dr_scan(3, fields, TAP_RTI, NULL);
			words_scheduled++;
		}
		
		if ((retval = jtag_execute_queue()) != ERROR_OK)
		{
			ERROR("JTAG error while receiving data from debug handler");
			exit(-1);
		}
		
		/* examine results */
		for (i = words_done; i < num_words; i++)
		{
			if (!(field0[0] & 1))
			{
				/* move backwards if necessary */
				int j;
				for (j = i; j < num_words - 1; j++)
				{
					field0[j] = field0[j+1];
					field1[j] = field1[j+1];
				}
				words_scheduled--;
			}
		}
		words_done += words_scheduled;
	}
	
	for (i = 0; i < num_words; i++)
		*(buffer++) = buf_get_u32((u8*)&field1[i], 0, 32);
	
	free(field1);
	
	return ERROR_OK;
}

int xscale_read_tx(target_t *target, int consume)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	enum tap_state path[3];
	
	int retval;
	struct timeval timeout, now;
	
	scan_field_t fields[3];
	u8 field0_in = 0x0;
	u8 field0_check_value = 0x2;
	u8 field0_check_mask = 0x6;
	u8 field2_check_value = 0x0;
	u8 field2_check_mask = 0x1;
	
	jtag_add_end_state(TAP_RTI);
	
	xscale_jtag_set_instr(xscale->jtag_info.chain_pos, xscale->jtag_info.dbgtx);
	
	path[0] = TAP_SDS;
	path[1] = TAP_CD;
	path[2] = TAP_SD;
	
	fields[0].device = xscale->jtag_info.chain_pos;
	fields[0].num_bits = 3;
	fields[0].out_value = NULL;
	fields[0].out_mask = NULL;
	fields[0].in_value = &field0_in;
	fields[0].in_check_value = &field0_check_value;
	fields[0].in_check_mask = &field0_check_mask;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
		
	fields[1].device = xscale->jtag_info.chain_pos;
	fields[1].num_bits = 32;
	fields[1].out_value = NULL;
	fields[1].out_mask = NULL;
	fields[1].in_value = xscale->reg_cache->reg_list[XSCALE_TX].value;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;

	fields[2].device = xscale->jtag_info.chain_pos;
	fields[2].num_bits = 1;
	fields[2].out_value = NULL;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = &field2_check_value;
	fields[2].in_check_mask = &field2_check_mask;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;
	
	gettimeofday(&timeout, NULL);
	timeval_add_time(&timeout, 5, 0);
	
	do
	{
		/* if we want to consume the register content (i.e. clear TX_READY),
		 * we have to go straight from Capture-DR to Shift-DR
		 * otherwise, we go from Capture-DR to Exit1-DR to Pause-DR
		*/
		if (consume)
			jtag_add_pathmove(3, path);
		else
			jtag_add_statemove(TAP_PD);
		
		jtag_add_dr_scan(3, fields, TAP_RTI, NULL);
	
		if ((retval = jtag_execute_queue()) != ERROR_OK)
		{
			ERROR("JTAG error while reading TX");
			exit(-1);
		}
		
		gettimeofday(&now, NULL);
		if ((now.tv_sec > timeout.tv_sec) && (now.tv_usec > timeout.tv_usec))
		{
			ERROR("time out reading TX register");
			return ERROR_TARGET_TIMEOUT;
		}
	} while ((!(field0_in & 1)) && consume);
	
	if (!(field0_in & 1))
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		
	return ERROR_OK;
}

int xscale_write_rx(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	int retval;
	struct timeval timeout, now;
	
	scan_field_t fields[3];
	u8 field0_out = 0x0;
	u8 field0_in = 0x0;
	u8 field0_check_value = 0x2;
	u8 field0_check_mask = 0x6;
	u8 field2 = 0x0;
	u8 field2_check_value = 0x0;
	u8 field2_check_mask = 0x1;
	
	jtag_add_end_state(TAP_RTI);
	
	xscale_jtag_set_instr(xscale->jtag_info.chain_pos, xscale->jtag_info.dbgrx);
	
	fields[0].device = xscale->jtag_info.chain_pos;
	fields[0].num_bits = 3;
	fields[0].out_value = &field0_out;
	fields[0].out_mask = NULL;
	fields[0].in_value = &field0_in;
	fields[0].in_check_value = &field0_check_value;
	fields[0].in_check_mask = &field0_check_mask;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
		
	fields[1].device = xscale->jtag_info.chain_pos;
	fields[1].num_bits = 32;
	fields[1].out_value = xscale->reg_cache->reg_list[XSCALE_RX].value;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;

	fields[2].device = xscale->jtag_info.chain_pos;
	fields[2].num_bits = 1;
	fields[2].out_value = &field2;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = &field2_check_value;
	fields[2].in_check_mask = &field2_check_mask;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;
	
	gettimeofday(&timeout, NULL);
	timeval_add_time(&timeout, 5, 0);
	
	/* poll until rx_read is low */
	do
	{
		DEBUG("polling RX");
		jtag_add_dr_scan(3, fields, TAP_RTI, NULL);
	
		if ((retval = jtag_execute_queue()) != ERROR_OK)
		{
			ERROR("JTAG error while writing RX");
			exit(-1);
		}
		
		gettimeofday(&now, NULL);
		if ((now.tv_sec > timeout.tv_sec) && (now.tv_usec > timeout.tv_usec))
		{
			ERROR("time out writing RX register");
			return ERROR_TARGET_TIMEOUT;
		}
	} while (field0_in & 1);
	
	/* set rx_valid */
	field2 = 0x1;
	jtag_add_dr_scan(3, fields, TAP_RTI, NULL);
	
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		ERROR("JTAG error while writing RX");
		exit(-1);
	}
	
	return ERROR_OK;
}

/* send count elements of size byte to the debug handler */
int xscale_send(target_t *target, u8 *buffer, int count, int size)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
		
	int retval;
	
	int done_count = 0;
	u8 output[4] = {0, 0, 0, 0};
	
	scan_field_t fields[3];
	u8 field0_out = 0x0;
	u8 field0_in = 0x0;
	u8 field0_check_value = 0x2;
	u8 field0_check_mask = 0x6;
	u8 field2 = 0x1;
	u8 field2_check_value = 0x0;
	u8 field2_check_mask = 0x1;
	
	jtag_add_end_state(TAP_RTI);
	
	xscale_jtag_set_instr(xscale->jtag_info.chain_pos, xscale->jtag_info.dbgrx);
	
	fields[0].device = xscale->jtag_info.chain_pos;
	fields[0].num_bits = 3;
	fields[0].out_value = &field0_out;
	fields[0].out_mask = NULL;
	fields[0].in_value = &field0_in;
	fields[0].in_check_value = &field0_check_value;
	fields[0].in_check_mask = &field0_check_mask;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
		
	fields[1].device = xscale->jtag_info.chain_pos;
	fields[1].num_bits = 32;
	fields[1].out_value = output;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;

	fields[2].device = xscale->jtag_info.chain_pos;
	fields[2].num_bits = 1;
	fields[2].out_value = &field2;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = &field2_check_value;
	fields[2].in_check_mask = &field2_check_mask;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;
	
	while (done_count++ < count)
	{
		/* extract sized element from target-endian buffer, and put it 
		 * into little-endian output buffer
		 */
		switch (size)
		{
			case 4:
				buf_set_u32(output, 0, 32, target_buffer_get_u32(target, buffer));
				break;
			case 2:
				buf_set_u32(output, 0, 32, target_buffer_get_u16(target, buffer));
				break;
			case 1:
				output[0] = *buffer;
				break;
			default:
				ERROR("BUG: size neither 4, 2 nor 1");
				exit(-1); 
		}

		jtag_add_dr_scan(3, fields, TAP_RTI, NULL);
		buffer += size;
	}
	
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		ERROR("JTAG error while sending data to debug handler");
		exit(-1);
	}
	
	return ERROR_OK;
}

int xscale_send_u32(target_t *target, u32 value)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	buf_set_u32(xscale->reg_cache->reg_list[XSCALE_RX].value, 0, 32, value);
	return xscale_write_rx(target);
}

int xscale_write_dcsr(target_t *target, int hold_rst, int ext_dbg_brk)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	int retval;
	
	scan_field_t fields[3];
	u8 field0 = 0x0;
	u8 field0_check_value = 0x2;
	u8 field0_check_mask = 0x7;
	u8 field2 = 0x0;
	u8 field2_check_value = 0x0;
	u8 field2_check_mask = 0x1;
	
	if (hold_rst != -1)
		xscale->hold_rst = hold_rst;
	
	if (ext_dbg_brk != -1)
		xscale->external_debug_break = ext_dbg_brk;

	jtag_add_end_state(TAP_RTI);
	xscale_jtag_set_instr(xscale->jtag_info.chain_pos, xscale->jtag_info.dcsr);
	
	buf_set_u32(&field0, 1, 1, xscale->hold_rst);
	buf_set_u32(&field0, 2, 1, xscale->external_debug_break);
	
	fields[0].device = xscale->jtag_info.chain_pos;
	fields[0].num_bits = 3;
	fields[0].out_value = &field0;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = &field0_check_value;
	fields[0].in_check_mask = &field0_check_mask;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
		
	fields[1].device = xscale->jtag_info.chain_pos;
	fields[1].num_bits = 32;
	fields[1].out_value = xscale->reg_cache->reg_list[XSCALE_DCSR].value;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;

	fields[2].device = xscale->jtag_info.chain_pos;
	fields[2].num_bits = 1;
	fields[2].out_value = &field2;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = &field2_check_value;
	fields[2].in_check_mask = &field2_check_mask;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;
	
	jtag_add_dr_scan(3, fields, -1, NULL);
	
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		ERROR("JTAG error while writing DCSR");
		exit(-1);
	}
	
	xscale->reg_cache->reg_list[XSCALE_DCSR].dirty = 0;
	xscale->reg_cache->reg_list[XSCALE_DCSR].valid = 1;
	
	return ERROR_OK;
}

/* parity of the number of bits 0 if even; 1 if odd. for 32 bit words */
unsigned int parity (unsigned int v)
{
	unsigned int ov = v;
	v ^= v >> 16;
	v ^= v >> 8;
	v ^= v >> 4;
	v &= 0xf;
	DEBUG("parity of 0x%x is %i", ov, (0x6996 >> v) & 1);
	return (0x6996 >> v) & 1;
}

int xscale_load_ic(target_t *target, int mini, u32 va, u32 buffer[8])
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	u8 packet[4];
	u8 cmd;
	int word;
	
	scan_field_t fields[2];

	DEBUG("loading miniIC at 0x%8.8x", va);

	jtag_add_end_state(TAP_RTI);
	xscale_jtag_set_instr(xscale->jtag_info.chain_pos, xscale->jtag_info.ldic); /* LDIC */
	
	/* CMD is b010 for Main IC and b011 for Mini IC */
	if (mini)
		buf_set_u32(&cmd, 0, 3, 0x3);
	else
		buf_set_u32(&cmd, 0, 3, 0x2);
	
	buf_set_u32(&cmd, 3, 3, 0x0);
		
	/* virtual address of desired cache line */
	buf_set_u32(packet, 0, 27, va >> 5);
	
	fields[0].device = xscale->jtag_info.chain_pos;
	fields[0].num_bits = 6;
	fields[0].out_value = &cmd;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].device = xscale->jtag_info.chain_pos;
	fields[1].num_bits = 27;
	fields[1].out_value = packet;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;
	
	jtag_add_dr_scan(2, fields, -1, NULL);

	fields[0].num_bits = 32;
	fields[0].out_value = packet;
	
	fields[1].num_bits = 1;
	fields[1].out_value = &cmd;
	
	for (word = 0; word < 8; word++)
	{
		buf_set_u32(packet, 0, 32, buffer[word]);
		cmd = parity(*((u32*)packet));
		jtag_add_dr_scan(2, fields, -1, NULL);
	}
	
	jtag_execute_queue();
	
	return ERROR_OK;
}

int xscale_invalidate_ic_line(target_t *target, u32 va)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	u8 packet[4];
	u8 cmd;
	
	scan_field_t fields[2];

	jtag_add_end_state(TAP_RTI);
	xscale_jtag_set_instr(xscale->jtag_info.chain_pos, xscale->jtag_info.ldic); /* LDIC */
	
	/* CMD for invalidate IC line b000, bits [6:4] b000 */
	buf_set_u32(&cmd, 0, 6, 0x0);
	
	/* virtual address of desired cache line */
	buf_set_u32(packet, 0, 27, va >> 5);
	
	fields[0].device = xscale->jtag_info.chain_pos;
	fields[0].num_bits = 6;
	fields[0].out_value = &cmd;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].device = xscale->jtag_info.chain_pos;
	fields[1].num_bits = 27;
	fields[1].out_value = packet;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;
	
	jtag_add_dr_scan(2, fields, -1, NULL);
	
	return ERROR_OK;
}

int xscale_update_vectors(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	int i;
	
	u32 low_reset_branch, high_reset_branch;

	for (i = 1; i < 8; i++)
	{
		/* if there's a static vector specified for this exception, override */
		if (xscale->static_high_vectors_set & (1 << i))
		{
			xscale->high_vectors[i] = xscale->static_high_vectors[i];
		}
		else
		{
			if (target_read_u32(target, 0xffff0000 + 4*i, &xscale->high_vectors[i]) != ERROR_OK)
			{
				xscale->high_vectors[i] = ARMV4_5_B(0xfffffe, 0);
			}
		}
	}

	for (i = 1; i < 8; i++)
	{
		if (xscale->static_low_vectors_set & (1 << i))
		{
			xscale->low_vectors[i] = xscale->static_low_vectors[i];
		}
		else
		{
			if (target_read_u32(target, 0x0 + 4*i, &xscale->low_vectors[i]) != ERROR_OK)
			{
				xscale->low_vectors[i] = ARMV4_5_B(0xfffffe, 0);
			}
		}
	}
	
	/* calculate branches to debug handler */
	low_reset_branch = (xscale->handler_address + 0x20 - 0x0 - 0x8) >> 2;
	high_reset_branch = (xscale->handler_address + 0x20 - 0xffff0000 - 0x8) >> 2;
	
	xscale->low_vectors[0] = ARMV4_5_B((low_reset_branch & 0xffffff), 0);
	xscale->high_vectors[0] = ARMV4_5_B((high_reset_branch & 0xffffff), 0);
	
	/* invalidate and load exception vectors in mini i-cache */
	xscale_invalidate_ic_line(target, 0x0);
	xscale_invalidate_ic_line(target, 0xffff0000);
	
	xscale_load_ic(target, 1, 0x0, xscale->low_vectors);
	xscale_load_ic(target, 1, 0xffff0000, xscale->high_vectors);
	
	return ERROR_OK;
}

int xscale_arch_state(struct target_s *target, char *buf, int buf_size)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	char *state[] = 
	{
		"disabled", "enabled"
	};
	
	char *arch_dbg_reason[] =
	{
		"", "\n(processor reset)", "\n(trace buffer full)"
	};
	
	if (armv4_5->common_magic != ARMV4_5_COMMON_MAGIC)
	{
		ERROR("BUG: called for a non-ARMv4/5 target");
		exit(-1);
	}
	
	snprintf(buf, buf_size,
			"target halted in %s state due to %s, current mode: %s\n"
			"cpsr: 0x%8.8x pc: 0x%8.8x\n"
			"MMU: %s, D-Cache: %s, I-Cache: %s"
			"%s",
			 armv4_5_state_strings[armv4_5->core_state],
			 target_debug_reason_strings[target->debug_reason],
			 armv4_5_mode_strings[armv4_5_mode_to_number(armv4_5->core_mode)],
			 buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32),
			 buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32),
			 state[xscale->armv4_5_mmu.mmu_enabled],
			 state[xscale->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled], 
			 state[xscale->armv4_5_mmu.armv4_5_cache.i_cache_enabled],
			 arch_dbg_reason[xscale->arch_debug_reason]);
	
	return ERROR_OK;
}

enum target_state xscale_poll(target_t *target)
{
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	if ((target->state == TARGET_RUNNING) || (target->state == TARGET_DEBUG_RUNNING))
	{
		if ((retval = xscale_read_tx(target, 0)) == ERROR_OK)
		{
			enum target_state previous_state = target->state;
			
			/* there's data to read from the tx register, we entered debug state */
			xscale->handler_running = 1;

			target->state = TARGET_HALTED;
			
			/* process debug entry, fetching current mode regs */
			if ((retval = xscale_debug_entry(target)) != ERROR_OK)
				return retval;
			
			/* debug_entry could have overwritten target state (i.e. immediate resume)
			 * don't signal event handlers in that case
			 */
			if (target->state != TARGET_HALTED)
				return target->state;
			
			/* if target was running, signal that we halted
			 * otherwise we reentered from debug execution */
			if (previous_state == TARGET_RUNNING)
				target_call_event_callbacks(target, TARGET_EVENT_HALTED);
			else
				target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		}
		else if (retval != ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
		{
			ERROR("error while polling TX register");
			exit(-1);
		}
	}
	
	return target->state;
}

int xscale_debug_entry(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	u32 pc;
	u32 *buffer = malloc(4 * 10);
	int i;
	
	u32 moe;
	
	/* clear external dbg break (will be written on next DCSR read) */
	xscale->external_debug_break = 0;
	xscale_read_dcsr(target);
	
	/* get r0, pc, r1 to r7 and cpsr */
	xscale_receive(target, buffer, 10);
	
	/* move r0 from buffer to register cache */
	buf_set_u32(armv4_5->core_cache->reg_list[0].value, 0, 32, buffer[0]);
	armv4_5->core_cache->reg_list[15].dirty = 1;
	armv4_5->core_cache->reg_list[15].valid = 1;
	DEBUG("r0: 0x%8.8x", buffer[0]);
	
	/* move pc from buffer to register cache */
	buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, buffer[1]);
	armv4_5->core_cache->reg_list[15].dirty = 1;
	armv4_5->core_cache->reg_list[15].valid = 1;
	DEBUG("pc: 0x%8.8x", buffer[1]);
	
	/* move data from buffer to register cache */
	for (i = 1; i <= 7; i++)
	{
		buf_set_u32(armv4_5->core_cache->reg_list[i].value, 0, 32, buffer[1 + i]);
		armv4_5->core_cache->reg_list[i].dirty = 1;
		armv4_5->core_cache->reg_list[i].valid = 1;
		DEBUG("r%i: 0x%8.8x", i, buffer[i + 1]);
	}
	
	buf_set_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32, buffer[9]);
	armv4_5->core_cache->reg_list[ARMV4_5_CPSR].dirty = 1;
	armv4_5->core_cache->reg_list[ARMV4_5_CPSR].valid = 1;
	DEBUG("cpsr: 0x%8.8x", buffer[9]);
	
	armv4_5->core_mode = buffer[9] & 0x1f;
	if (armv4_5_mode_to_number(armv4_5->core_mode) == -1)
	{
		target->state = TARGET_UNKNOWN;
		ERROR("cpsr contains invalid mode value - communication failure");
		return ERROR_TARGET_FAILURE;
	}
	DEBUG("target entered debug state in %s mode", armv4_5_mode_strings[armv4_5_mode_to_number(armv4_5->core_mode)]);
	
	if (buffer[9] & 0x20)
		armv4_5->core_state = ARMV4_5_STATE_THUMB;
	else
		armv4_5->core_state = ARMV4_5_STATE_ARM;
	
	/* get banked registers, r8 to r14, and spsr if not in USR/SYS mode */
	if ((armv4_5->core_mode != ARMV4_5_MODE_USR) && (armv4_5->core_mode != ARMV4_5_MODE_SYS))
	{
		xscale_receive(target, buffer, 8);
		buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 16).value, 0, 32, buffer[7]);
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 16).dirty = 0;
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 16).valid = 1;
	}
	else
	{
		/* r8 to r14, but no spsr */
		xscale_receive(target, buffer, 7);
	}
	
	/* move data from buffer to register cache */
	for (i = 8; i <= 14; i++)
	{
		buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).value, 0, 32, buffer[i - 8]);
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).dirty = 0;
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).valid = 1;
	}
	
	/* examine debug reason */
	xscale_read_dcsr(target);
	moe = buf_get_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 2, 3);
	
	/* stored PC (for calculating fixup) */
	pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);
	
	switch (moe)
	{
		case 0x0: /* Processor reset */
			target->debug_reason = DBG_REASON_DBGRQ;
			xscale->arch_debug_reason = XSCALE_DBG_REASON_RESET;
			pc -= 4;
			break;
		case 0x1: /* Instruction breakpoint hit */
			target->debug_reason = DBG_REASON_BREAKPOINT;
			xscale->arch_debug_reason = XSCALE_DBG_REASON_GENERIC;
			pc -= 4;
			break;
		case 0x2: /* Data breakpoint hit */
			target->debug_reason = DBG_REASON_WATCHPOINT;
			xscale->arch_debug_reason = XSCALE_DBG_REASON_GENERIC;
			pc -= 4;
			break;
		case 0x3: /* BKPT instruction executed */
			target->debug_reason = DBG_REASON_BREAKPOINT;
			xscale->arch_debug_reason = XSCALE_DBG_REASON_GENERIC;
			pc -= 4;
			break;
		case 0x4: /* Ext. debug event */
			target->debug_reason = DBG_REASON_DBGRQ;
			xscale->arch_debug_reason = XSCALE_DBG_REASON_GENERIC;
			pc -= 4;
			break;
		case 0x5: /* Vector trap occured */
			target->debug_reason = DBG_REASON_BREAKPOINT;
			xscale->arch_debug_reason = XSCALE_DBG_REASON_GENERIC;
			pc -= 4;
			break;
		case 0x6: /* Trace buffer full break */
			target->debug_reason = DBG_REASON_DBGRQ;
			xscale->arch_debug_reason = XSCALE_DBG_REASON_TB_FULL;
			pc -= 4;
			break;
		case 0x7: /* Reserved */
		default:
			ERROR("Method of Entry is 'Reserved'");
			exit(-1);
			break;
	}
	
	/* apply PC fixup */
	buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, pc); 
	
	/* on the first debug entry, identify cache type */
	if (xscale->armv4_5_mmu.armv4_5_cache.ctype == -1)
	{
		u32 cache_type_reg;
		
		/* read cp15 cache type register */
		xscale_get_reg(&xscale->reg_cache->reg_list[XSCALE_CACHETYPE]);
		cache_type_reg = buf_get_u32(xscale->reg_cache->reg_list[XSCALE_CACHETYPE].value, 0, 32);
		
		armv4_5_identify_cache(cache_type_reg, &xscale->armv4_5_mmu.armv4_5_cache);
	}
	
	/* examine MMU and Cache settings */
	/* read cp15 control register */
	xscale_get_reg(&xscale->reg_cache->reg_list[XSCALE_CTRL]);
	xscale->cp15_control_reg = buf_get_u32(xscale->reg_cache->reg_list[XSCALE_CTRL].value, 0, 32);
	xscale->armv4_5_mmu.mmu_enabled = (xscale->cp15_control_reg & 0x1U) ? 1 : 0;
	xscale->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = (xscale->cp15_control_reg & 0x4U) ? 1 : 0;
	xscale->armv4_5_mmu.armv4_5_cache.i_cache_enabled = (xscale->cp15_control_reg & 0x1000U) ? 1 : 0;
	
	/* tracing enabled, read collected trace data */
	if (xscale->trace.buffer_enabled)
	{
		xscale_read_trace(target);
		xscale->trace.buffer_fill--;
		
		/* resume if we're still collecting trace data */
		if ((xscale->arch_debug_reason == XSCALE_DBG_REASON_TB_FULL)
			&& (xscale->trace.buffer_fill > 0))
		{
			xscale_resume(target, 1, 0x0, 1, 0);
		}
		else
		{
			xscale->trace.buffer_enabled = 0;
		}
	}
	
	return ERROR_OK;
}

int xscale_halt(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	DEBUG("target->state: %s", target_state_strings[target->state]);
	
	if (target->state == TARGET_HALTED)
	{
		WARNING("target was already halted");
		return ERROR_TARGET_ALREADY_HALTED;
	} 
	else if (target->state == TARGET_UNKNOWN)
	{
		/* this must not happen for a xscale target */
		ERROR("target was in unknown state when halt was requested");
		exit(-1);
	}
	else if (target->state == TARGET_RESET)
	{
		DEBUG("target->state == TARGET_RESET");
	}
	else
	{
		/* assert external dbg break */
		xscale->external_debug_break = 1;
		xscale_read_dcsr(target);
	
		target->debug_reason = DBG_REASON_DBGRQ;
	}
	
	return ERROR_OK;
}

int xscale_enable_single_step(struct target_s *target, u32 next_pc)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale= armv4_5->arch_info;
	reg_t *ibcr0 = &xscale->reg_cache->reg_list[XSCALE_IBCR0];
	
	if (xscale->ibcr0_used)
	{
		breakpoint_t *ibcr0_bp = breakpoint_find(target, buf_get_u32(ibcr0->value, 0, 32) & 0xfffffffe);
		
		if (ibcr0_bp)
		{
			xscale_unset_breakpoint(target, ibcr0_bp);
		}
		else
		{
			ERROR("BUG: xscale->ibcr0_used is set, but no breakpoint with that address found");
			exit(-1);
		}
	}
	
	xscale_set_reg_u32(ibcr0, next_pc | 0x1);
	
	return ERROR_OK;
}

int xscale_disable_single_step(struct target_s *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale= armv4_5->arch_info;
	reg_t *ibcr0 = &xscale->reg_cache->reg_list[XSCALE_IBCR0];
	
	xscale_set_reg_u32(ibcr0, 0x0);
	
	return ERROR_OK;
}

int xscale_resume(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale= armv4_5->arch_info;
	breakpoint_t *breakpoint = target->breakpoints;
	
	u32 current_pc;
	
	int retval;
	int i;
	
	DEBUG("-");
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (!debug_execution)
	{
		target_free_all_working_areas(target);
	}
	
	/* update vector tables */
	xscale_update_vectors(target);
	
	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
		buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, address);

	current_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);
	
	/* if we're at the reset vector, we have to simulate the branch */
	if (current_pc == 0x0)
	{
		arm_simulate_step(target, NULL);
		current_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);
	}
	
	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints)
	{
		if ((breakpoint = breakpoint_find(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32))))
		{
			u32 next_pc;
			
			/* there's a breakpoint at the current PC, we have to step over it */
			DEBUG("unset breakpoint at 0x%8.8x", breakpoint->address);
			xscale_unset_breakpoint(target, breakpoint);
			
			/* calculate PC of next instruction */
			if ((retval = arm_simulate_step(target, &next_pc)) != ERROR_OK)
			{
				u32 current_opcode;
				target_read_u32(target, current_pc, &current_opcode);
				ERROR("BUG: couldn't calculate PC of next instruction, current opcode was 0x%8.8x", current_opcode);
			}
			
			DEBUG("enable single-step");
			xscale_enable_single_step(target, next_pc);
			
			/* restore banked registers */
			xscale_restore_context(target);
			
			/* send resume request (command 0x30 or 0x31)
			 * clean the trace buffer if it is to be enabled (0x62) */
			if (xscale->trace.buffer_enabled)
			{
				xscale_send_u32(target, 0x62);
				xscale_send_u32(target, 0x31);
			}
			else
				xscale_send_u32(target, 0x30);
							
			/* send CPSR */
			xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32));
			DEBUG("writing cpsr with value 0x%8.8x", buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32));
				
			for (i = 7; i >= 0; i--)
			{
				/* send register */
				xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[i].value, 0, 32));
				DEBUG("writing r%i with value 0x%8.8x", i, buf_get_u32(armv4_5->core_cache->reg_list[i].value, 0, 32));
			}

			/* send PC */
			xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));
			DEBUG("writing PC with value 0x%8.8x", buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));
			
			/* wait for and process debug entry */
			xscale_debug_entry(target);
			
			DEBUG("disable single-step");
			xscale_disable_single_step(target);
			
			DEBUG("set breakpoint at 0x%8.8x", breakpoint->address);
			xscale_set_breakpoint(target, breakpoint);
		}
	}
	
	/* enable any pending breakpoints and watchpoints */
	xscale_enable_breakpoints(target);
	xscale_enable_watchpoints(target);
	
	/* restore banked registers */
	xscale_restore_context(target);
	
	/* send resume request (command 0x30 or 0x31)
	 * clean the trace buffer if it is to be enabled (0x62) */
	if (xscale->trace.buffer_enabled)
	{
		xscale_send_u32(target, 0x62);
		xscale_send_u32(target, 0x31);
	}
	else
		xscale_send_u32(target, 0x30);
	
	/* send CPSR */
	xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32));
	DEBUG("writing cpsr with value 0x%8.8x", buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32));
		
	for (i = 7; i >= 0; i--)
	{
		/* send register */
		xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[i].value, 0, 32));
		DEBUG("writing r%i with value 0x%8.8x", i, buf_get_u32(armv4_5->core_cache->reg_list[i].value, 0, 32));
	}

	/* send PC */
	xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));
	DEBUG("writing PC with value 0x%8.8x", buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));
		
	target->debug_reason = DBG_REASON_NOTHALTED;
		
	if (!debug_execution)
	{
		/* registers are now invalid */
		armv4_5_invalidate_core_regs(target);
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	}
	else
	{
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
	}
	
	DEBUG("target resumed");
	
	xscale->handler_running = 1;
	
	return ERROR_OK;
}

int xscale_step(struct target_s *target, int current, u32 address, int handle_breakpoints)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	breakpoint_t *breakpoint = target->breakpoints;
	
	u32 current_pc, next_pc;
	int i;
	int retval;

	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
		buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, address);
	
	current_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);
	
	/* if we're at the reset vector, we have to simulate the step */
	if (current_pc == 0x0)
	{
		arm_simulate_step(target, NULL);
		current_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);
		
		target->debug_reason = DBG_REASON_SINGLESTEP;
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		
		return ERROR_OK;
	}
	
	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints)
		if ((breakpoint = breakpoint_find(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32))))
		{
			xscale_unset_breakpoint(target, breakpoint);
		}
	
	target->debug_reason = DBG_REASON_SINGLESTEP;

	/* calculate PC of next instruction */
	if ((retval = arm_simulate_step(target, &next_pc)) != ERROR_OK)
	{
		u32 current_opcode;
		target_read_u32(target, current_pc, &current_opcode);
		ERROR("BUG: couldn't calculate PC of next instruction, current opcode was 0x%8.8x", current_opcode);
	}
	
	DEBUG("enable single-step");
	xscale_enable_single_step(target, next_pc);
	
	/* restore banked registers */
	xscale_restore_context(target);
	
	/* send resume request (command 0x30 or 0x31)
	 * clean the trace buffer if it is to be enabled (0x62) */
	if (xscale->trace.buffer_enabled)
	{
		xscale_send_u32(target, 0x62);
		xscale_send_u32(target, 0x31);
	}
	else
		xscale_send_u32(target, 0x30);
	
	/* send CPSR */
	xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32));
	DEBUG("writing cpsr with value 0x%8.8x", buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32));
		
	for (i = 7; i >= 0; i--)
	{
		/* send register */
		xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[i].value, 0, 32));
		DEBUG("writing r%i with value 0x%8.8x", i, buf_get_u32(armv4_5->core_cache->reg_list[i].value, 0, 32));
	}

	/* send PC */
	xscale_send_u32(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));
	DEBUG("writing PC with value 0x%8.8x", buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	/* registers are now invalid */
	armv4_5_invalidate_core_regs(target);
	
	/* wait for and process debug entry */
	xscale_debug_entry(target);
	
	DEBUG("disable single-step");
	xscale_disable_single_step(target);
		
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	if (breakpoint)
	{
		xscale_set_breakpoint(target, breakpoint);
	}
		
	DEBUG("target stepped");

	return ERROR_OK;

}

int xscale_assert_reset(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	DEBUG("target->state: %s", target_state_strings[target->state]);

	/* select DCSR instruction (set endstate to R-T-I to ensure we don't 
	 * end up in T-L-R, which would reset JTAG
	 */ 
	jtag_add_end_state(TAP_RTI);
	xscale_jtag_set_instr(xscale->jtag_info.chain_pos, xscale->jtag_info.dcsr);
	
	/* set Hold reset, Halt mode and Trap Reset */
	buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 30, 1, 0x1);
	buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 16, 1, 0x1);
	xscale_write_dcsr(target, 1, 0);

	/* select BYPASS, because having DCSR selected caused problems on the PXA27x */
	xscale_jtag_set_instr(xscale->jtag_info.chain_pos, 0x7f);
	jtag_execute_queue();
		
	/* assert reset */	
	jtag_add_reset(0, 1);
	
	/* sleep 1ms, to be sure we fulfill any requirements */
	jtag_add_sleep(1000);
	jtag_execute_queue();
	
	target->state = TARGET_RESET;
	
	return ERROR_OK;
}

int xscale_deassert_reset(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	fileio_t debug_handler;
	u32 address;
	u32 binary_size;

	u32 buf_cnt;
	int i;
	int retval;
	
	breakpoint_t *breakpoint = target->breakpoints;
	
	DEBUG("-");
	
	xscale->ibcr_available = 2;
	xscale->ibcr0_used = 0;
	xscale->ibcr1_used = 0;
		
	xscale->dbr_available = 2;
	xscale->dbr0_used = 0;
	xscale->dbr1_used = 0;
	
	/* mark all hardware breakpoints as unset */
	while (breakpoint)
	{
		if (breakpoint->type == BKPT_HARD)
		{
			breakpoint->set = 0;
		}
		breakpoint = breakpoint->next;
	}
	
	if (!xscale->handler_installed)
	{
		/* release SRST */
		jtag_add_reset(0, 0);
		
		/* wait 300ms; 150 and 100ms were not enough */
		jtag_add_sleep(3000000);

		jtag_add_runtest(2030, TAP_RTI);
		jtag_execute_queue();

		/* set Hold reset, Halt mode and Trap Reset */
		buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 30, 1, 0x1);
		buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 16, 1, 0x1);
		xscale_write_dcsr(target, 1, 0);

		if (fileio_open(&debug_handler, "target/xscale/debug_handler.bin", FILEIO_READ, FILEIO_BINARY) != ERROR_OK)
		{
			ERROR("file open error: %s", debug_handler.error_str);
			return ERROR_OK;
		}
	
		if ((binary_size = debug_handler.size) % 4)
		{
			ERROR("debug_handler.bin: size not a multiple of 4");
			exit(-1);
		}
			
		if (binary_size > 0x800)
		{
			ERROR("debug_handler.bin: larger than 2kb");
			exit(-1);
		}
		
		binary_size = CEIL(binary_size, 32) * 32;
		
		address = xscale->handler_address;
		while (binary_size > 0)
		{
			u32 cache_line[8];
			u8 buffer[32];
			
			if ((retval = fileio_read(&debug_handler, 32, buffer, &buf_cnt)) != ERROR_OK)
			{
				ERROR("reading debug handler failed: %s", debug_handler.error_str);
			}
			
			for (i = 0; i < buf_cnt; i += 4)
			{
				/* convert LE buffer to host-endian u32 */
				cache_line[i / 4] = le_to_h_u32(&buffer[i]);
			}
			
			for (; buf_cnt < 32; buf_cnt += 4)
			{
					cache_line[buf_cnt / 4] = 0xe1a08008;
			}
			
			/* only load addresses other than the reset vectors */
			if ((address % 0x400) != 0x0)
			{
				xscale_load_ic(target, 1, address, cache_line);
			}
			
			address += buf_cnt;
			binary_size -= buf_cnt;
		};
		
		xscale_load_ic(target, 1, 0x0, xscale->low_vectors);
		xscale_load_ic(target, 1, 0xffff0000, xscale->high_vectors);
	
		jtag_add_runtest(30, TAP_RTI);

		jtag_add_sleep(100000);
		
		/* set Hold reset, Halt mode and Trap Reset */
		buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 30, 1, 0x1);
		buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 16, 1, 0x1);
		xscale_write_dcsr(target, 1, 0);

		/* clear Hold reset to let the target run (should enter debug handler) */
		xscale_write_dcsr(target, 0, 1);
		target->state = TARGET_RUNNING;

		if ((target->reset_mode != RESET_HALT) && (target->reset_mode != RESET_INIT))
		{
			jtag_add_sleep(10000);
			
			/* we should have entered debug now */
			xscale_debug_entry(target);
			target->state = TARGET_HALTED;
			
			/* resume the target */
			xscale_resume(target, 1, 0x0, 1, 0);
		}
	}
	else
	{
		jtag_add_reset(0, 0);
	}
		
		
	return ERROR_OK;
}

int xscale_soft_reset_halt(struct target_s *target)
{
	
	return ERROR_OK;
}

int xscale_prepare_reset_halt(struct target_s *target)
{
	/* nothing to be done for reset_halt on XScale targets
	 * we always halt after a reset to upload the debug handler
	 */
	return ERROR_OK;
}

int xscale_read_core_reg(struct target_s *target, int num, enum armv4_5_mode mode)
{

	return ERROR_OK;
}

int xscale_write_core_reg(struct target_s *target, int num, enum armv4_5_mode mode, u32 value)
{
	
	return ERROR_OK;
}

int xscale_full_context(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	
	u32 *buffer;
	
	int i, j;
	
	DEBUG("-");
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	buffer = malloc(4 * 8);
	
	/* iterate through processor modes (FIQ, IRQ, SVC, ABT, UND and SYS)
	 * we can't enter User mode on an XScale (unpredictable),
	 * but User shares registers with SYS
	 */
	for(i = 1; i < 7; i++)
	{
		int valid = 1;
		
		/* check if there are invalid registers in the current mode 
		 */
		for (j = 0; j <= 16; j++)
		{
			if (ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), j).valid == 0)
				valid = 0;
		}
		
		if (!valid)
		{
			u32 tmp_cpsr;
			
			/* request banked registers */
			xscale_send_u32(target, 0x0);
			
			tmp_cpsr = 0x0;
			tmp_cpsr |= armv4_5_number_to_mode(i);
			tmp_cpsr |= 0xc0; /* I/F bits */
			
			/* send CPSR for desired mode */
			xscale_send_u32(target, tmp_cpsr);

			/* get banked registers, r8 to r14, and spsr if not in USR/SYS mode */
			if ((armv4_5_number_to_mode(i) != ARMV4_5_MODE_USR) && (armv4_5_number_to_mode(i) != ARMV4_5_MODE_SYS))
			{
				xscale_receive(target, buffer, 8);
				buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 16).value, 0, 32, buffer[7]);
				ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), 16).dirty = 0;
				ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), 16).valid = 1;
			}
			else
			{
				xscale_receive(target, buffer, 7);
			}
	
			/* move data from buffer to register cache */
			for (j = 8; j <= 14; j++)
			{
				buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), j).value, 0, 32, buffer[j - 8]);
				ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), j).dirty = 0;
				ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), j).valid = 1;
			}
		}
	}
	
	free(buffer);
	
	return ERROR_OK;
}

int xscale_restore_context(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	
	int i, j;
	
	DEBUG("-");
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	/* iterate through processor modes (FIQ, IRQ, SVC, ABT, UND and SYS)
	* we can't enter User mode on an XScale (unpredictable),
	* but User shares registers with SYS
	*/
	for(i = 1; i < 7; i++)
	{
		int dirty = 0;
		
		/* check if there are invalid registers in the current mode 
		*/
		for (j = 8; j <= 14; j++)
		{
			if (ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), j).dirty == 1)
				dirty = 1;
		}
		
		/* if not USR/SYS, check if the SPSR needs to be written */
		if ((armv4_5_number_to_mode(i) != ARMV4_5_MODE_USR) && (armv4_5_number_to_mode(i) != ARMV4_5_MODE_SYS))
		{
			if (ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), 16).dirty == 1)
				dirty = 1;
		}
		
		if (dirty)
		{
			u32 tmp_cpsr;
			
			/* send banked registers */
			xscale_send_u32(target, 0x1);
			
			tmp_cpsr = 0x0;
			tmp_cpsr |= armv4_5_number_to_mode(i);
			tmp_cpsr |= 0xc0; /* I/F bits */
			
			/* send CPSR for desired mode */
			xscale_send_u32(target, tmp_cpsr);

			/* send banked registers, r8 to r14, and spsr if not in USR/SYS mode */
			for (j = 8; j <= 14; j++)
			{
				xscale_send_u32(target, buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, j).value, 0, 32));
				ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), j).dirty = 0;
			}
			
			if ((armv4_5_number_to_mode(i) != ARMV4_5_MODE_USR) && (armv4_5_number_to_mode(i) != ARMV4_5_MODE_SYS))
			{
				xscale_send_u32(target, buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 16).value, 0, 32));
				ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), 16).dirty = 0;
			}
		}
	}
	
	return ERROR_OK;
}

int xscale_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	u32 *buf32;
	int i;
	
	DEBUG("address: 0x%8.8x, size: 0x%8.8x, count: 0x%8.8x", address, size, count);

	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_INVALID_ARGUMENTS;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;
	
	/* send memory read request (command 0x1n, n: access size) */
	xscale_send_u32(target, 0x10 | size);
	
	/* send base address for read request */
	xscale_send_u32(target, address);
	
	/* send number of requested data words */
	xscale_send_u32(target, count);
	
	/* receive data from target (count times 32-bit words in host endianness) */
	buf32 = malloc(4 * count);
	xscale_receive(target, buf32, count);
	
	/* extract data from host-endian buffer into byte stream */
	for (i = 0; i < count; i++)
	{
		switch (size)
		{
			case 4:
				target_buffer_set_u32(target, buffer, buf32[i]);
				buffer += 4;
				break;
			case 2:
				target_buffer_set_u16(target, buffer, buf32[i] & 0xffff);
				buffer += 2;
				break;
			case 1:
				*buffer++ = buf32[i] & 0xff;
				break;
			default:
				ERROR("should never get here");
				exit(-1);
		}
	}

	free(buf32);
	
	/* examine DCSR, to see if Sticky Abort (SA) got set */ 
	xscale_read_dcsr(target);
	if (buf_get_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 5, 1) == 1)
	{
		/* clear SA bit */
		xscale_send_u32(target, 0x60);
		
		return ERROR_TARGET_DATA_ABORT;
	}
	
	return ERROR_OK;
}

int xscale_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	DEBUG("address: 0x%8.8x, size: 0x%8.8x, count: 0x%8.8x", address, size, count);

	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_INVALID_ARGUMENTS;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;
	
	/* send memory write request (command 0x2n, n: access size) */
	xscale_send_u32(target, 0x20 | size);
	
	/* send base address for read request */
	xscale_send_u32(target, address);
	
	/* send number of requested data words to be written*/
	xscale_send_u32(target, count);
	
	/* extract data from host-endian buffer into byte stream */
#if 0
	for (i = 0; i < count; i++)
	{
		switch (size)
		{
			case 4:
				value = target_buffer_get_u32(target, buffer);
				xscale_send_u32(target, value);
				buffer += 4;
				break;
			case 2:
				value = target_buffer_get_u16(target, buffer);
				xscale_send_u32(target, value);
				buffer += 2;
				break;
			case 1:
				value = *buffer;
				xscale_send_u32(target, value);
				buffer += 1; 
				break;
			default:
				ERROR("should never get here");
				exit(-1);
		}
	}
#endif
	xscale_send(target, buffer, count, size);
	
	/* examine DCSR, to see if Sticky Abort (SA) got set */ 
	xscale_read_dcsr(target);
	if (buf_get_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 5, 1) == 1)
	{
		/* clear SA bit */
		xscale_send_u32(target, 0x60);
		
		return ERROR_TARGET_DATA_ABORT;
	}
	
	return ERROR_OK;
}

int xscale_bulk_write_memory(target_t *target, u32 address, u32 count, u8 *buffer)
{
	xscale_write_memory(target, address, 4, count, buffer);
	
	return ERROR_OK;
}

u32 xscale_get_ttb(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	u32 ttb;

	xscale_get_reg(&xscale->reg_cache->reg_list[XSCALE_TTB]);
	ttb = buf_get_u32(xscale->reg_cache->reg_list[XSCALE_TTB].value, 0, 32);

	return ttb;
}

void xscale_disable_mmu_caches(target_t *target, int mmu, int d_u_cache, int i_cache)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	u32 cp15_control;

	/* read cp15 control register */
	xscale_get_reg(&xscale->reg_cache->reg_list[XSCALE_CTRL]);
	cp15_control = buf_get_u32(xscale->reg_cache->reg_list[XSCALE_CTRL].value, 0, 32);
	
	if (mmu)
		cp15_control &= ~0x1U;
	
	if (d_u_cache)
	{
		/* clean DCache */
		xscale_send_u32(target, 0x50);
		xscale_send_u32(target, xscale->cache_clean_address);
		
		/* invalidate DCache */
		xscale_send_u32(target, 0x51);
		
		cp15_control &= ~0x4U;
	}
	
	if (i_cache)
	{
		/* invalidate ICache */
		xscale_send_u32(target, 0x52);
		cp15_control &= ~0x1000U;
	}

	/* write new cp15 control register */
	xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_CTRL], cp15_control);
	
	/* execute cpwait to ensure outstanding operations complete */
	xscale_send_u32(target, 0x53);
}

void xscale_enable_mmu_caches(target_t *target, int mmu, int d_u_cache, int i_cache)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	u32 cp15_control;

	/* read cp15 control register */
	xscale_get_reg(&xscale->reg_cache->reg_list[XSCALE_CTRL]);
	cp15_control = buf_get_u32(xscale->reg_cache->reg_list[XSCALE_CTRL].value, 0, 32);
			
	if (mmu)
		cp15_control |= 0x1U;
	
	if (d_u_cache)
		cp15_control |= 0x4U;
	
	if (i_cache)
		cp15_control |= 0x1000U;
	
	/* write new cp15 control register */
	xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_CTRL], cp15_control);
	
	/* execute cpwait to ensure outstanding operations complete */
	xscale_send_u32(target, 0x53);
}

int xscale_set_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (xscale->force_hw_bkpts)
		breakpoint->type = BKPT_HARD;

	if (breakpoint->set)
	{
		WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		u32 value = breakpoint->address | 1;
		if (!xscale->ibcr0_used)
		{
			xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_IBCR0], value); 
			xscale->ibcr0_used = 1;
			breakpoint->set = 1;	/* breakpoint set on first breakpoint register */
		}
		else if (!xscale->ibcr1_used)
		{
			xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_IBCR1], value); 
			xscale->ibcr1_used = 1;
			breakpoint->set = 2;	/* breakpoint set on second breakpoint register */
		}
		else
		{
			ERROR("BUG: no hardware comparator available");
			return ERROR_OK;
		}
	}
	else if (breakpoint->type == BKPT_SOFT)
	{
		if (breakpoint->length == 4)
		{
			/* keep the original instruction in target endianness */
			target->type->read_memory(target, breakpoint->address, 4, 1, breakpoint->orig_instr);
			/* write the original instruction in target endianness (arm7_9->arm_bkpt is host endian) */
			target_write_u32(target, breakpoint->address, xscale->arm_bkpt);
		}
		else
		{
			/* keep the original instruction in target endianness */
			target->type->read_memory(target, breakpoint->address, 2, 1, breakpoint->orig_instr);
			/* write the original instruction in target endianness (arm7_9->arm_bkpt is host endian) */
			target_write_u32(target, breakpoint->address, xscale->thumb_bkpt);
		}
		breakpoint->set = 1;
	}

	return ERROR_OK;

}

int xscale_add_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (xscale->force_hw_bkpts)
	{
		DEBUG("forcing use of hardware breakpoint at address 0x%8.8x", breakpoint->address);
		breakpoint->type = BKPT_HARD;
	}
	
	if ((breakpoint->type == BKPT_HARD) && (xscale->ibcr_available < 1))
	{
		INFO("no breakpoint unit available for hardware breakpoint");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	else
	{
		xscale->ibcr_available--;
	}
	
	if ((breakpoint->length != 2) && (breakpoint->length != 4))
	{
		INFO("only breakpoints of two (Thumb) or four (ARM) bytes length supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	
	return ERROR_OK;
}

int xscale_unset_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!breakpoint->set)
	{
		WARNING("breakpoint not set");
		return ERROR_OK;
	}
	
	if (breakpoint->type == BKPT_HARD)
	{
		if (breakpoint->set == 1)
		{
			xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_IBCR0], 0x0); 
			xscale->ibcr0_used = 0;
		}
		else if (breakpoint->set == 2)
		{
			xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_IBCR1], 0x0); 
			xscale->ibcr1_used = 0;
		}
		breakpoint->set = 0;
	}
	else
	{
		/* restore original instruction (kept in target endianness) */
		if (breakpoint->length == 4)
		{
			target->type->write_memory(target, breakpoint->address, 4, 1, breakpoint->orig_instr);
		}
		else
		{
			target->type->write_memory(target, breakpoint->address, 2, 1, breakpoint->orig_instr);
		}
		breakpoint->set = 0;
	}

	return ERROR_OK;
}

int xscale_remove_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (breakpoint->set)
	{
		xscale_unset_breakpoint(target, breakpoint);
	}
	
	if (breakpoint->type == BKPT_HARD)
		xscale->ibcr_available++;
	
	return ERROR_OK;
}

int xscale_set_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	u8 enable;
	reg_t *dbcon = &xscale->reg_cache->reg_list[XSCALE_DBCON];
	u32 dbcon_value = buf_get_u32(dbcon->value, 0, 32);
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	xscale_get_reg(dbcon);
	
	switch (watchpoint->rw)
	{
		case WPT_READ:
			enable = 0x3;
			break;
		case WPT_ACCESS:
			enable = 0x2;
			break;
		case WPT_WRITE:
			enable = 0x1;
			break;
		default:
			ERROR("BUG: watchpoint->rw neither read, write nor access");	
	}

	if (!xscale->dbr0_used)
	{
		xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_DBR0], watchpoint->address);
		dbcon_value |= enable;
		xscale_set_reg_u32(dbcon, dbcon_value);
		watchpoint->set = 1;
		xscale->dbr0_used = 1;
	}
	else if (!xscale->dbr1_used)
	{
		xscale_set_reg_u32(&xscale->reg_cache->reg_list[XSCALE_DBR1], watchpoint->address);
		dbcon_value |= enable << 2;
		xscale_set_reg_u32(dbcon, dbcon_value);
		watchpoint->set = 2;
		xscale->dbr1_used = 1;
	}
	else
	{
		ERROR("BUG: no hardware comparator available");
		return ERROR_OK;
	}
	
	return ERROR_OK;
}

int xscale_add_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (xscale->dbr_available < 1)
	{
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	
	if ((watchpoint->length != 1) && (watchpoint->length != 2) && (watchpoint->length != 4))
	{
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	
	xscale->dbr_available--;
		
	return ERROR_OK;
}

int xscale_unset_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	reg_t *dbcon = &xscale->reg_cache->reg_list[XSCALE_DBCON];
	u32 dbcon_value = buf_get_u32(dbcon->value, 0, 32);
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (!watchpoint->set)
	{
		WARNING("breakpoint not set");
		return ERROR_OK;
	}
	
	if (watchpoint->set == 1)
	{
		dbcon_value &= ~0x3;
		xscale_set_reg_u32(dbcon, dbcon_value);
		xscale->dbr0_used = 0;
	}
	else if (watchpoint->set == 2)
	{
		dbcon_value &= ~0xc;
		xscale_set_reg_u32(dbcon, dbcon_value);
		xscale->dbr1_used = 0;
	}
	watchpoint->set = 0;

	return ERROR_OK;
}

int xscale_remove_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (watchpoint->set)
	{
		xscale_unset_watchpoint(target, watchpoint);
	}
		
	xscale->dbr_available++;
	
	return ERROR_OK;
}

void xscale_enable_watchpoints(struct target_s *target)
{
	watchpoint_t *watchpoint = target->watchpoints;
	
	while (watchpoint)
	{
		if (watchpoint->set == 0)
			xscale_set_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}
}

void xscale_enable_breakpoints(struct target_s *target)
{
	breakpoint_t *breakpoint = target->breakpoints;
	
	/* set any pending breakpoints */
	while (breakpoint)
	{
		if (breakpoint->set == 0)
			xscale_set_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}
}

int xscale_get_reg(reg_t *reg)
{
	xscale_reg_t *arch_info = reg->arch_info;
	target_t *target = arch_info->target;
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
		
	/* DCSR, TX and RX are accessible via JTAG */
	if (strcmp(reg->name, "XSCALE_DCSR") == 0)
	{
		return xscale_read_dcsr(arch_info->target);
	}
	else if (strcmp(reg->name, "XSCALE_TX") == 0)
	{
		/* 1 = consume register content */
		return xscale_read_tx(arch_info->target, 1);
	}
	else if (strcmp(reg->name, "XSCALE_RX") == 0)
	{
		/* can't read from RX register (host -> debug handler) */
		return ERROR_OK;
	}
	else if (strcmp(reg->name, "XSCALE_TXRXCTRL") == 0)
	{
		/* can't (explicitly) read from TXRXCTRL register */
		return ERROR_OK;
	}
	else /* Other DBG registers have to be transfered by the debug handler */
	{
		/* send CP read request (command 0x40) */
		xscale_send_u32(target, 0x40);
		
		/* send CP register number */
		xscale_send_u32(target, arch_info->dbg_handler_number);

		/* read register value */
		xscale_read_tx(target, 1);
		buf_cpy(xscale->reg_cache->reg_list[XSCALE_TX].value, reg->value, 32);

		reg->dirty = 0;
		reg->valid = 1;
	}
	
	return ERROR_OK;
}

int xscale_set_reg(reg_t *reg, u8* buf)
{
	xscale_reg_t *arch_info = reg->arch_info;
	target_t *target = arch_info->target;
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	u32 value = buf_get_u32(buf, 0, 32);
	
	/* DCSR, TX and RX are accessible via JTAG */
	if (strcmp(reg->name, "XSCALE_DCSR") == 0)
	{
		buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 0, 32, value);
		return xscale_write_dcsr(arch_info->target, -1, -1);
	}
	else if (strcmp(reg->name, "XSCALE_RX") == 0)
	{
		buf_set_u32(xscale->reg_cache->reg_list[XSCALE_RX].value, 0, 32, value);
		return xscale_write_rx(arch_info->target);
	}
	else if (strcmp(reg->name, "XSCALE_TX") == 0)
	{
		/* can't write to TX register (debug-handler -> host) */
		return ERROR_OK;
	}
	else if (strcmp(reg->name, "XSCALE_TXRXCTRL") == 0)
	{
		/* can't (explicitly) write to TXRXCTRL register */
		return ERROR_OK;
	}
	else /* Other DBG registers have to be transfered by the debug handler */
	{
		/* send CP write request (command 0x41) */
		xscale_send_u32(target, 0x41);
		
		/* send CP register number */
		xscale_send_u32(target, arch_info->dbg_handler_number);
		
		/* send CP register value */
		xscale_send_u32(target, value);
		buf_set_u32(reg->value, 0, 32, value);
	}
	
	return ERROR_OK;
}

/* convenience wrapper to access XScale specific registers */
int xscale_set_reg_u32(reg_t *reg, u32 value)
{
	u8 buf[4];
	
	buf_set_u32(buf, 0, 32, value);
	
	return xscale_set_reg(reg, buf);
}

int xscale_write_dcsr_sw(target_t *target, u32 value)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	reg_t *dcsr = &xscale->reg_cache->reg_list[XSCALE_DCSR];
	xscale_reg_t *dcsr_arch_info = dcsr->arch_info;
	
	/* send CP write request (command 0x41) */
	xscale_send_u32(target, 0x41);
		
	/* send CP register number */
	xscale_send_u32(target, dcsr_arch_info->dbg_handler_number);
		
	/* send CP register value */
	xscale_send_u32(target, value);
	buf_set_u32(dcsr->value, 0, 32, value);
	
	return ERROR_OK;
}

int xscale_read_trace(target_t *target)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	xscale_trace_data_t **trace_data_p;
	
	/* 258 words from debug handler
	 * 256 trace buffer entries
	 * 2 checkpoint addresses
	 */ 
	u32 trace_buffer[258];
	int is_address[256];
	int i, j;
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target must be stopped to read trace data");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* send read trace buffer command (command 0x61) */
	xscale_send_u32(target, 0x61);
	
	/* receive trace buffer content */
	xscale_receive(target, trace_buffer, 258);
	
	/* parse buffer backwards to identify address entries */
	for (i = 255; i >= 0; i--)
	{
		is_address[i] = 0;
		if (((trace_buffer[i] & 0xf0) == 0x90) ||
			((trace_buffer[i] & 0xf0) == 0xd0)) 
		{
			if (i >= 3)
				is_address[--i] = 1;
			if (i >= 2)
				is_address[--i] = 1;
			if (i >= 1)
				is_address[--i] = 1;
			if (i >= 0)
				is_address[--i] = 1;
		}
	}

	
	/* search first non-zero entry */
	for (j = 0; (j < 256) && (trace_buffer[j] == 0) && (!is_address[j]); j++)
		;

	if (j == 256)
	{
		DEBUG("no trace data collected");
		return ERROR_XSCALE_NO_TRACE_DATA;
	}
		
	for (trace_data_p = &xscale->trace.data; *trace_data_p; trace_data_p = &(*trace_data_p)->next)
		;

	*trace_data_p = malloc(sizeof(xscale_trace_data_t));
	(*trace_data_p)->next = NULL;
	(*trace_data_p)->chkpt0 = trace_buffer[256];
	(*trace_data_p)->chkpt1 = trace_buffer[257];
	(*trace_data_p)->last_instruction = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);
	(*trace_data_p)->entries = malloc(sizeof(xscale_trace_entry_t) * (256 - j));
	(*trace_data_p)->depth = 256 - j;
		
	for (i = j; i < 256; i++)
	{
		(*trace_data_p)->entries[i - j].data = trace_buffer[i];
		if (is_address[i])
			(*trace_data_p)->entries[i - j].type = XSCALE_TRACE_ADDRESS;
		else
			(*trace_data_p)->entries[i - j].type = XSCALE_TRACE_MESSAGE;
	}
	
	return ERROR_OK;
}

int xscale_read_instruction(target_t *target, arm_instruction_t *instruction)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	int i;
	int section = -1;
	u32 size_read;
	u32 opcode;
	int retval;
	
	if (!xscale->trace.image)
		return ERROR_TRACE_IMAGE_UNAVAILABLE;
	
	/* search for the section the current instruction belongs to */	
	for (i = 0; i < xscale->trace.image->num_sections; i++)
	{
		if ((xscale->trace.image->sections[i].base_address <= xscale->trace.current_pc) &&
			(xscale->trace.image->sections[i].base_address + xscale->trace.image->sections[i].size > xscale->trace.current_pc))
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
	
	if (xscale->trace.core_state == ARMV4_5_STATE_ARM)
	{
		u8 buf[4];
		if ((retval = image_read_section(xscale->trace.image, section, 
			xscale->trace.current_pc - xscale->trace.image->sections[section].base_address,
			4, buf, &size_read)) != ERROR_OK)
		{
			ERROR("error while reading instruction: %i", retval);
			return ERROR_TRACE_INSTRUCTION_UNAVAILABLE;
		}
		opcode = target_buffer_get_u32(target, buf);
		arm_evaluate_opcode(opcode, xscale->trace.current_pc, instruction);
	}
	else if (xscale->trace.core_state == ARMV4_5_STATE_THUMB)
	{
		u8 buf[2];
		if ((retval = image_read_section(xscale->trace.image, section, 
			xscale->trace.current_pc - xscale->trace.image->sections[section].base_address,
			2, buf, &size_read)) != ERROR_OK)
		{
			ERROR("error while reading instruction: %i", retval);
			return ERROR_TRACE_INSTRUCTION_UNAVAILABLE;
		}
		opcode = target_buffer_get_u16(target, buf);
		thumb_evaluate_opcode(opcode, xscale->trace.current_pc, instruction);
	}
	else
	{
		ERROR("BUG: unknown core state encountered");
		exit(-1);
	}
	
	return ERROR_OK;
}

int xscale_branch_address(xscale_trace_data_t *trace_data, int i, u32 *target)
{
	/* if there are less than four entries prior to the indirect branch message
	 * we can't extract the address */
	if (i < 4)
	{
		return -1;
	}
	
	*target = (trace_data->entries[i-1].data) | (trace_data->entries[i-2].data << 8) |
				(trace_data->entries[i-3].data << 16) | (trace_data->entries[i-4].data << 24);
	
	return 0;
}

int xscale_analyze_trace(target_t *target, command_context_t *cmd_ctx)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;
	int next_pc_ok = 0;
	u32 next_pc = 0x0;
	xscale_trace_data_t *trace_data = xscale->trace.data;
	int retval;
	
	while (trace_data)
	{
		int i, chkpt;
		int rollover;
		int branch;
		int exception;
		xscale->trace.core_state = ARMV4_5_STATE_ARM;

		chkpt = 0;
		rollover = 0;
		
		for (i = 0; i < trace_data->depth; i++)
		{
			next_pc_ok = 0;
			branch = 0;
			exception = 0;
			
			if (trace_data->entries[i].type == XSCALE_TRACE_ADDRESS)
				continue;
			
			switch ((trace_data->entries[i].data & 0xf0) >> 4)
			{
				case 0:		/* Exceptions */
				case 1:
				case 2:
				case 3:
				case 4:
				case 5:
				case 6:
				case 7:
					exception = (trace_data->entries[i].data & 0x70) >> 4;
					next_pc_ok = 1;
					next_pc = (trace_data->entries[i].data & 0xf0) >> 2;
					command_print(cmd_ctx, "--- exception %i ---", (trace_data->entries[i].data & 0xf0) >> 4);
					break;
				case 8:		/* Direct Branch */
					branch = 1;
					break;
				case 9:		/* Indirect Branch */
					branch = 1;
					if (xscale_branch_address(trace_data, i, &next_pc) == 0)
					{
						next_pc_ok = 1;
					}
					break;
				case 13:	/* Checkpointed Indirect Branch */
					if (xscale_branch_address(trace_data, i, &next_pc) == 0)
					{
						next_pc_ok = 1;
						if (((chkpt == 0) && (next_pc != trace_data->chkpt0))
							|| ((chkpt == 1) && (next_pc != trace_data->chkpt1)))
							WARNING("checkpointed indirect branch target address doesn't match checkpoint");
					}
					/* explicit fall-through */
				case 12:	/* Checkpointed Direct Branch */
					branch = 1;
					if (chkpt == 0)
					{
						next_pc_ok = 1;
						next_pc = trace_data->chkpt0;
						chkpt++;
					}
					else if (chkpt == 1)
					{
						next_pc_ok = 1;
						next_pc = trace_data->chkpt0;
						chkpt++;
					}
					else
					{
						WARNING("more than two checkpointed branches encountered");
					}
					break;
				case 15:	/* Roll-over */
					rollover++;
					continue;
				default:	/* Reserved */
					command_print(cmd_ctx, "--- reserved trace message ---");
					ERROR("BUG: trace message %i is reserved", (trace_data->entries[i].data & 0xf0) >> 4);
					return ERROR_OK;
			}
			
			if (xscale->trace.pc_ok)
			{
				int executed = (trace_data->entries[i].data & 0xf) + rollover * 16;
				arm_instruction_t instruction;
				
				if ((exception == 6) || (exception == 7))
				{
					/* IRQ or FIQ exception, no instruction executed */ 
					executed -= 1;
				}
				
				while (executed-- >= 0)
				{
					if ((retval = xscale_read_instruction(target, &instruction)) != ERROR_OK)
					{
						/* can't continue tracing with no image available */
						if (retval == ERROR_TRACE_IMAGE_UNAVAILABLE)
						{
							return retval;
						}
						else if (retval == ERROR_TRACE_INSTRUCTION_UNAVAILABLE)
						{
							/* TODO: handle incomplete images */
						}
					}
					
					/* a precise abort on a load to the PC is included in the incremental
					 * word count, other instructions causing data aborts are not included
					 */
					if ((executed == 0) && (exception == 4)
						&& ((instruction.type >= ARM_LDR) && (instruction.type <= ARM_LDM)))
					{
						if ((instruction.type == ARM_LDM)
							&& ((instruction.info.load_store_multiple.register_list & 0x8000) == 0))
						{
							executed--;
						}
						else if (((instruction.type >= ARM_LDR) && (instruction.type <= ARM_LDRSH))
							&& (instruction.info.load_store.Rd != 15))
						{
							executed--;
						}
					}

					/* only the last instruction executed
					 * (the one that caused the control flow change)
					 * could be a taken branch
					 */
					if (((executed == -1) && (branch == 1)) &&
						(((instruction.type == ARM_B) ||
							(instruction.type == ARM_BL) ||
							(instruction.type == ARM_BLX)) &&
							(instruction.info.b_bl_bx_blx.target_address != -1)))
					{
						xscale->trace.current_pc = instruction.info.b_bl_bx_blx.target_address;
					}
					else
					{
						xscale->trace.current_pc += (xscale->trace.core_state == ARMV4_5_STATE_ARM) ? 4 : 2;
					}
					command_print(cmd_ctx, "%s", instruction.text);
				}
				
				rollover = 0;
			}
			
			if (next_pc_ok)
			{
				xscale->trace.current_pc = next_pc;
				xscale->trace.pc_ok = 1;
			}
		}
		
		for (; xscale->trace.current_pc < trace_data->last_instruction; xscale->trace.current_pc += (xscale->trace.core_state == ARMV4_5_STATE_ARM) ? 4 : 2)
		{
			arm_instruction_t instruction;
			if ((retval = xscale_read_instruction(target, &instruction)) != ERROR_OK)
			{
				/* can't continue tracing with no image available */
				if (retval == ERROR_TRACE_IMAGE_UNAVAILABLE)
				{
					return retval;
				}
				else if (retval == ERROR_TRACE_INSTRUCTION_UNAVAILABLE)
				{
					/* TODO: handle incomplete images */
				}
			}
			command_print(cmd_ctx, "%s", instruction.text);
		}
		
		trace_data = trace_data->next;
	}
	
	return ERROR_OK;
}

void xscale_build_reg_cache(target_t *target)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	xscale_common_t *xscale = armv4_5->arch_info;

	reg_cache_t **cache_p = register_get_last_cache_p(&target->reg_cache);
	xscale_reg_t *arch_info = malloc(sizeof(xscale_reg_arch_info));
	int i;
	int num_regs = sizeof(xscale_reg_arch_info) / sizeof(xscale_reg_t);
	
	(*cache_p) = armv4_5_build_reg_cache(target, armv4_5);
	armv4_5->core_cache = (*cache_p);
	
	/* register a register arch-type for XScale dbg registers only once */
	if (xscale_reg_arch_type == -1)
		xscale_reg_arch_type = register_reg_arch_type(xscale_get_reg, xscale_set_reg);
	
	(*cache_p)->next = malloc(sizeof(reg_cache_t));
	cache_p = &(*cache_p)->next;
	
	/* fill in values for the xscale reg cache */
	(*cache_p)->name = "XScale registers";
	(*cache_p)->next = NULL;
	(*cache_p)->reg_list = malloc(num_regs * sizeof(reg_t));
	(*cache_p)->num_regs = num_regs;
	
	for (i = 0; i < num_regs; i++)
	{
		(*cache_p)->reg_list[i].name = xscale_reg_list[i];
		(*cache_p)->reg_list[i].value = calloc(4, 1);
		(*cache_p)->reg_list[i].dirty = 0;
		(*cache_p)->reg_list[i].valid = 0;
		(*cache_p)->reg_list[i].size = 32;
		(*cache_p)->reg_list[i].bitfield_desc = NULL;
		(*cache_p)->reg_list[i].num_bitfields = 0;
		(*cache_p)->reg_list[i].arch_info = &arch_info[i];
		(*cache_p)->reg_list[i].arch_type = xscale_reg_arch_type;
		arch_info[i] = xscale_reg_arch_info[i];
		arch_info[i].target = target;
	}
	
	xscale->reg_cache = (*cache_p);
}

int xscale_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	if (startup_mode != DAEMON_RESET)
	{
		ERROR("XScale target requires a reset");
		ERROR("Reset target to enable debug");
	}
	
	/* assert TRST once during startup */
	jtag_add_reset(1, 0);
	jtag_add_sleep(5000);
	jtag_add_reset(0, 0);
	jtag_execute_queue();
	
	return ERROR_OK;
}

int xscale_quit()
{
	
	return ERROR_OK;
}

int xscale_init_arch_info(target_t *target, xscale_common_t *xscale, int chain_pos, char *variant)
{
	armv4_5_common_t *armv4_5;
	u32 high_reset_branch, low_reset_branch;
	int i;
	
	armv4_5 = &xscale->armv4_5_common;
	
	/* store architecture specfic data (none so far) */
	xscale->arch_info = NULL;
	xscale->common_magic = XSCALE_COMMON_MAGIC;
	
	/* remember the variant (PXA25x, PXA27x, IXP42x, ...) */
	xscale->variant = strdup(variant);
	
	/* prepare JTAG information for the new target */
	xscale->jtag_info.chain_pos = chain_pos;
	jtag_register_event_callback(xscale_jtag_callback, target);

	xscale->jtag_info.dbgrx = 0x02;
	xscale->jtag_info.dbgtx = 0x10;
	xscale->jtag_info.dcsr = 0x09;
	xscale->jtag_info.ldic = 0x07;	

	if ((strcmp(xscale->variant, "pxa250") == 0) ||
		(strcmp(xscale->variant, "pxa255") == 0) ||
		(strcmp(xscale->variant, "pxa26x") == 0))
	{
		xscale->jtag_info.ir_length = 5;
	}
	else if ((strcmp(xscale->variant, "pxa27x") == 0) ||
		(strcmp(xscale->variant, "ixp42x") == 0) ||
		(strcmp(xscale->variant, "ixp45x") == 0) ||
		(strcmp(xscale->variant, "ixp46x") == 0))
	{
		xscale->jtag_info.ir_length = 7;
	}
	
	/* the debug handler isn't installed (and thus not running) at this time */
	xscale->handler_installed = 0;
	xscale->handler_running = 0;
	xscale->handler_address = 0xfe000800;
	
	/* clear the vectors we keep locally for reference */
	memset(xscale->low_vectors, 0, sizeof(xscale->low_vectors));
	memset(xscale->high_vectors, 0, sizeof(xscale->high_vectors));

	/* no user-specified vectors have been configured yet */
	xscale->static_low_vectors_set = 0x0;
	xscale->static_high_vectors_set = 0x0;

	/* calculate branches to debug handler */
	low_reset_branch = (xscale->handler_address + 0x20 - 0x0 - 0x8) >> 2;
	high_reset_branch = (xscale->handler_address + 0x20 - 0xffff0000 - 0x8) >> 2;
	
	xscale->low_vectors[0] = ARMV4_5_B((low_reset_branch & 0xffffff), 0);
	xscale->high_vectors[0] = ARMV4_5_B((high_reset_branch & 0xffffff), 0);
	
	for (i = 1; i <= 7; i++)
	{
		xscale->low_vectors[i] = ARMV4_5_B(0xfffffe, 0);
		xscale->high_vectors[i] = ARMV4_5_B(0xfffffe, 0);
	}
	
	/* 64kB aligned region used for DCache cleaning */ 
	xscale->cache_clean_address = 0xfffe0000;	
	
	xscale->hold_rst = 0;
	xscale->external_debug_break = 0;
	
	xscale->force_hw_bkpts = 1;
	
	xscale->ibcr_available = 2;
	xscale->ibcr0_used = 0;
	xscale->ibcr1_used = 0;
		
	xscale->dbr_available = 2;
	xscale->dbr0_used = 0;
	xscale->dbr1_used = 0;
	
	xscale->arm_bkpt = ARMV5_BKPT(0x0);
	xscale->thumb_bkpt = ARMV5_T_BKPT(0x0) & 0xffff;
	
	xscale->vector_catch = 0x1;
	
	xscale->trace.capture_status = TRACE_IDLE;
	xscale->trace.data = NULL;
	xscale->trace.image = NULL;
	xscale->trace.buffer_enabled = 0;
	xscale->trace.buffer_fill = 0;
	
	/* prepare ARMv4/5 specific information */
	armv4_5->arch_info = xscale;
	armv4_5->read_core_reg = xscale_read_core_reg;
	armv4_5->write_core_reg = xscale_write_core_reg;
	armv4_5->full_context = xscale_full_context;

	armv4_5_init_arch_info(target, armv4_5);

	xscale->armv4_5_mmu.armv4_5_cache.ctype = -1;
	xscale->armv4_5_mmu.get_ttb = xscale_get_ttb;
	xscale->armv4_5_mmu.read_memory = xscale_read_memory;
	xscale->armv4_5_mmu.write_memory = xscale_write_memory;
	xscale->armv4_5_mmu.disable_mmu_caches = xscale_disable_mmu_caches;
	xscale->armv4_5_mmu.enable_mmu_caches = xscale_enable_mmu_caches;
	xscale->armv4_5_mmu.has_tiny_pages = 1;
	xscale->armv4_5_mmu.mmu_enabled = 0;
	
	return ERROR_OK;
}

/* target xscale <endianess> <startup_mode> <chain_pos> <variant> */
int xscale_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct target_s *target)
{
	int chain_pos;
	char *variant = NULL;
	xscale_common_t *xscale = malloc(sizeof(xscale_common_t));

	if (argc < 5)
	{
		ERROR("'target xscale' requires four arguments: <endianess> <startup_mode> <chain_pos> <variant>");
		exit(-1);
	}
	
	chain_pos = strtoul(args[3], NULL, 0);
	
	variant = args[4];
	
	xscale_init_arch_info(target, xscale, chain_pos, variant);
	xscale_build_reg_cache(target);
	
	return ERROR_OK;
}

int xscale_handle_debug_handler_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = NULL;
	armv4_5_common_t *armv4_5;
	xscale_common_t *xscale;

	u32 handler_address;

	if (argc < 2)
	{
		ERROR("'xscale debug_handler <target#> <address>' command takes two required operands");
		return ERROR_OK;
	}
	
	if ((target = get_target_by_num(strtoul(args[0], NULL, 0))) == NULL)
	{
		ERROR("no target '%s' configured", args[0]);
		return ERROR_OK;
	}
	
	if (xscale_get_arch_pointers(target, &armv4_5, &xscale) != ERROR_OK)
	{
		command_print(cmd_ctx, "target isn't an ARM920t target");
		return ERROR_OK;
	}
	
	handler_address = strtoul(args[1], NULL, 0);
	
	if (((handler_address >= 0x800) && (handler_address <= 0x1fef800)) ||
		((handler_address >= 0xfe000800) && (handler_address <= 0xfffff800)))
	{
		xscale->handler_address = handler_address;
	}
	else
	{
		ERROR("xscale debug_handler <address> must be between 0x800 and 0x1fef800 or between 0xfe000800 and 0xfffff800");
	}
	
	return ERROR_OK;
}

int xscale_handle_cache_clean_address_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = NULL;
	armv4_5_common_t *armv4_5;
	xscale_common_t *xscale;

	u32 cache_clean_address;
	
	if (argc < 2)
	{
		ERROR("'xscale cache_clean_address <target#> <address>' command takes two required operands");
		return ERROR_OK;
	}
	
	if ((target = get_target_by_num(strtoul(args[0], NULL, 0))) == NULL)
	{
		ERROR("no target '%s' configured", args[0]);
		return ERROR_OK;
	}
	
	if (xscale_get_arch_pointers(target, &armv4_5, &xscale) != ERROR_OK)
	{
		command_print(cmd_ctx, "target isn't an XScale target");
		return ERROR_OK;
	}
	
	cache_clean_address = strtoul(args[1], NULL, 0);
	
	if (cache_clean_address & 0xffff)
	{
		ERROR("xscale cache_clean_address <address> must be 64kb aligned");
	}
	else
	{
		xscale->cache_clean_address = cache_clean_address;
	}
	
	return ERROR_OK;
}

int xscale_handle_cache_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	xscale_common_t *xscale;
	
	if (xscale_get_arch_pointers(target, &armv4_5, &xscale) != ERROR_OK)
	{
		command_print(cmd_ctx, "target isn't an XScale target");
		return ERROR_OK;
	}
	
	return armv4_5_handle_cache_info_command(cmd_ctx, &xscale->armv4_5_mmu.armv4_5_cache);
}

int xscale_handle_virt2phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc)
{	
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	xscale_common_t *xscale;
	
	if (xscale_get_arch_pointers(target, &armv4_5, &xscale) != ERROR_OK)
	{
		command_print(cmd_ctx, "target isn't an XScale target");
		return ERROR_OK;
	}
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}
		
	return armv4_5_mmu_handle_virt2phys_command(cmd_ctx, cmd, args, argc, target, &xscale->armv4_5_mmu);
}

int xscale_handle_mmu_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc)
{	
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	xscale_common_t *xscale;
	
	if (xscale_get_arch_pointers(target, &armv4_5, &xscale) != ERROR_OK)
	{
		command_print(cmd_ctx, "target isn't an XScale target");
		return ERROR_OK;
	}
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}
	
	if (argc >= 1)
	{
		if (strcmp("enable", args[0]) == 0)
		{
			xscale_enable_mmu_caches(target, 1, 0, 0);
			xscale->armv4_5_mmu.mmu_enabled = 1;
		}
		else if (strcmp("disable", args[0]) == 0)
		{
			xscale_disable_mmu_caches(target, 1, 0, 0);
			xscale->armv4_5_mmu.mmu_enabled = 0;
		}
	}
		
	command_print(cmd_ctx, "mmu %s", (xscale->armv4_5_mmu.mmu_enabled) ? "enabled" : "disabled");
	
	return ERROR_OK;
}

int xscale_handle_idcache_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc)
{	
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	xscale_common_t *xscale;
	int icache = 0, dcache = 0;
	
	if (xscale_get_arch_pointers(target, &armv4_5, &xscale) != ERROR_OK)
	{
		command_print(cmd_ctx, "target isn't an XScale target");
		return ERROR_OK;
	}
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}
	
	if (strcmp(cmd, "icache") == 0)
		icache = 1;
	else if (strcmp(cmd, "dcache") == 0)
		dcache = 1;
	
	if (argc >= 1)
	{
		if (strcmp("enable", args[0]) == 0)
		{
			xscale_enable_mmu_caches(target, 0, dcache, icache);
			
			if (icache)
				xscale->armv4_5_mmu.armv4_5_cache.i_cache_enabled = 1;
			else if (dcache)
				xscale->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = 1;
		}
		else if (strcmp("disable", args[0]) == 0)
		{
			xscale_disable_mmu_caches(target, 0, dcache, icache);

			if (icache)
				xscale->armv4_5_mmu.armv4_5_cache.i_cache_enabled = 0;
			else if (dcache)
				xscale->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = 0;
		}
	}
	
	if (icache)
		command_print(cmd_ctx, "icache %s", (xscale->armv4_5_mmu.armv4_5_cache.i_cache_enabled) ? "enabled" : "disabled");
	
	if (dcache)
	 	command_print(cmd_ctx, "dcache %s", (xscale->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled) ? "enabled" : "disabled");
	 	
	return ERROR_OK;
}

int xscale_handle_vector_catch_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc)
{	
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	xscale_common_t *xscale;
	
	if (xscale_get_arch_pointers(target, &armv4_5, &xscale) != ERROR_OK)
	{
		command_print(cmd_ctx, "target isn't an XScale target");
		return ERROR_OK;
	}
	
	if (argc < 1)
	{
		command_print(cmd_ctx, "usage: xscale vector_catch [mask]");
	}
	else
	{
		xscale->vector_catch = strtoul(args[0], NULL, 0);
		buf_set_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 16, 8, xscale->vector_catch);
		xscale_write_dcsr(target, -1, -1);
	}
	
	command_print(cmd_ctx, "vector catch mask: 0x%2.2x", xscale->vector_catch);
	
	return ERROR_OK;
}

int xscale_handle_force_hw_bkpts_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	xscale_common_t *xscale;
	
	if (xscale_get_arch_pointers(target, &armv4_5, &xscale) != ERROR_OK)
	{
		command_print(cmd_ctx, "target isn't an XScale target");
		return ERROR_OK;
	}
	
	if ((argc >= 1) && (strcmp("enable", args[0]) == 0))
	{
		xscale->force_hw_bkpts = 1;
	}
	else if ((argc >= 1) && (strcmp("disable", args[0]) == 0))
	{
		xscale->force_hw_bkpts = 0;
	}
	else
	{
		command_print(cmd_ctx, "usage: xscale force_hw_bkpts <enable|disable>");
	}
		
	command_print(cmd_ctx, "force hardware breakpoints %s", (xscale->force_hw_bkpts) ? "enabled" : "disabled");

	return ERROR_OK;
}

int xscale_handle_trace_buffer_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	xscale_common_t *xscale;
	u32 dcsr_value;
	
	if (xscale_get_arch_pointers(target, &armv4_5, &xscale) != ERROR_OK)
	{
		command_print(cmd_ctx, "target isn't an XScale target");
		return ERROR_OK;
	}
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}
	
	if ((argc >= 1) && (strcmp("enable", args[0]) == 0))
	{
		xscale_trace_data_t *td, *next_td;
		xscale->trace.buffer_enabled = 1;
		
		/* free old trace data */
		td = xscale->trace.data;
		while (td)
		{
			next_td = td->next;
			
			if (td->entries)
				free(td->entries);
			free(td);
			td = next_td;
		}
		xscale->trace.data = NULL;
	}
	else if ((argc >= 1) && (strcmp("disable", args[0]) == 0))
	{
		xscale->trace.buffer_enabled = 0;
	}

	if ((argc >= 2) && (strcmp("fill", args[1]) == 0))
	{
		if (argc >= 3)
			xscale->trace.buffer_fill = strtoul(args[2], NULL, 0);
		else
			xscale->trace.buffer_fill = 1;
	}
	else if ((argc >= 2) && (strcmp("wrap", args[1]) == 0))
	{
		xscale->trace.buffer_fill = -1;
	}
	
	command_print(cmd_ctx, "trace buffer %s (%s)", 
		(xscale->trace.buffer_enabled) ? "enabled" : "disabled",
		(xscale->trace.buffer_fill > 0) ? "fill" : "wrap");

	dcsr_value = buf_get_u32(xscale->reg_cache->reg_list[XSCALE_DCSR].value, 0, 32);
	if (xscale->trace.buffer_fill >= 0)
		xscale_write_dcsr_sw(target, (dcsr_value & 0xfffffffc) | 2);
	else
		xscale_write_dcsr_sw(target, dcsr_value & 0xfffffffc);
		
	return ERROR_OK;
}

int xscale_handle_trace_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	xscale_common_t *xscale;

	if (argc < 1)
	{
		command_print(cmd_ctx, "usage: xscale trace_image <file> [base address] [type]");
		return ERROR_OK;
	}
	
	target = get_current_target(cmd_ctx);
	
	if (xscale_get_arch_pointers(target, &armv4_5, &xscale) != ERROR_OK)
	{
		command_print(cmd_ctx, "target isn't an XScale target");
		return ERROR_OK;
	}
	
	if (xscale->trace.image)
	{
		image_close(xscale->trace.image);
		free(xscale->trace.image);
		command_print(cmd_ctx, "previously loaded image found and closed");
	}
	
	xscale->trace.image = malloc(sizeof(image_t));
	xscale->trace.image->base_address_set = 0;
	xscale->trace.image->start_address_set = 0;
	
	/* a base address isn't always necessary, default to 0x0 (i.e. don't relocate) */
	if (argc >= 2)
	{
		xscale->trace.image->base_address_set = 1;
		xscale->trace.image->base_address = strtoul(args[1], NULL, 0);
	}
	else
	{
		xscale->trace.image->base_address_set = 0;
	}
		
	if (image_open(xscale->trace.image, args[0], (argc >= 3) ? args[2] : NULL) != ERROR_OK)
	{
		command_print(cmd_ctx, "image opening error: %s", xscale->trace.image->error_str);
		free(xscale->trace.image);
		xscale->trace.image = NULL;
		return ERROR_OK;
	}
	
	return ERROR_OK;
}

int xscale_handle_dump_trace_buffer_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	xscale_common_t *xscale;

	if (xscale_get_arch_pointers(target, &armv4_5, &xscale) != ERROR_OK)
	{
		command_print(cmd_ctx, "target isn't an XScale target");
		return ERROR_OK;
	}
	
	return ERROR_OK;	
}

int xscale_handle_analyze_trace_buffer_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	xscale_common_t *xscale;

	if (xscale_get_arch_pointers(target, &armv4_5, &xscale) != ERROR_OK)
	{
		command_print(cmd_ctx, "target isn't an XScale target");
		return ERROR_OK;
	}
	
	xscale_analyze_trace(target, cmd_ctx);
	
	return ERROR_OK;	
}

int xscale_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *xscale_cmd;

	xscale_cmd = register_command(cmd_ctx, NULL, "xscale", NULL, COMMAND_ANY, "xscale specific commands");
	
	register_command(cmd_ctx, xscale_cmd, "debug_handler", xscale_handle_debug_handler_command, COMMAND_CONFIG, NULL);
	register_command(cmd_ctx, xscale_cmd, "cache_clean_address", xscale_handle_cache_clean_address_command, COMMAND_ANY, NULL);

	register_command(cmd_ctx, xscale_cmd, "cache_info", xscale_handle_cache_info_command, COMMAND_EXEC, NULL);
	register_command(cmd_ctx, xscale_cmd, "virt2phys", xscale_handle_virt2phys_command, COMMAND_EXEC, NULL);
	register_command(cmd_ctx, xscale_cmd, "mmu", xscale_handle_mmu_command, COMMAND_EXEC, "['enable'|'disable'] the MMU");
	register_command(cmd_ctx, xscale_cmd, "icache", xscale_handle_idcache_command, COMMAND_EXEC, "['enable'|'disable'] the ICache");
	register_command(cmd_ctx, xscale_cmd, "dcache", xscale_handle_idcache_command, COMMAND_EXEC, "['enable'|'disable'] the DCache");
	
	register_command(cmd_ctx, xscale_cmd, "vector_catch", xscale_handle_idcache_command, COMMAND_EXEC, "<mask> of vectors that should be catched");
	
	register_command(cmd_ctx, xscale_cmd, "trace_buffer", xscale_handle_trace_buffer_command, COMMAND_EXEC, "<enable|disable> ['fill'|'wrap']");

	register_command(cmd_ctx, xscale_cmd, "dump_trace_buffer", xscale_handle_dump_trace_buffer_command, COMMAND_EXEC, "dump content of trace buffer");
	register_command(cmd_ctx, xscale_cmd, "analyze_trace", xscale_handle_analyze_trace_buffer_command, COMMAND_EXEC, "analyze content of trace buffer");
	register_command(cmd_ctx, xscale_cmd, "trace_image", xscale_handle_trace_image_command,
		COMMAND_EXEC, "load image from <file> [base address]");
		
	armv4_5_register_commands(cmd_ctx);
	
	return ERROR_OK;
}
