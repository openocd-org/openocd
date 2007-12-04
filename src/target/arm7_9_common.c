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

#include "replacements.h"

#include "embeddedice.h"
#include "target.h"
#include "target_request.h"
#include "armv4_5.h"
#include "arm_jtag.h"
#include "jtag.h"
#include "log.h"
#include "arm7_9_common.h"
#include "breakpoints.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <errno.h>

int arm7_9_debug_entry(target_t *target);
int arm7_9_enable_sw_bkpts(struct target_s *target);

/* command handler forward declarations */
int handle_arm7_9_write_xpsr_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_arm7_9_write_xpsr_im8_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_arm7_9_read_core_reg_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_arm7_9_write_core_reg_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_arm7_9_sw_bkpts_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_arm7_9_force_hw_bkpts_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_arm7_9_dbgrq_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_arm7_9_fast_memory_access_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_arm7_9_dcc_downloads_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_arm7_9_etm_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int arm7_9_reinit_embeddedice(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
	breakpoint_t *breakpoint = target->breakpoints;
	
	arm7_9->wp_available = 2;
	arm7_9->wp0_used = 0;
	arm7_9->wp1_used = 0;
		
	/* mark all hardware breakpoints as unset */
	while (breakpoint)
	{
		if (breakpoint->type == BKPT_HARD)
		{
			breakpoint->set = 0;
		}
		breakpoint = breakpoint->next;
	}
		
	if (arm7_9->sw_bkpts_enabled && arm7_9->sw_bkpts_use_wp)
	{
		arm7_9->sw_bkpts_enabled = 0;
		arm7_9_enable_sw_bkpts(target);
	}
	
	arm7_9->reinit_embeddedice = 0;
	
	return ERROR_OK;
}

int arm7_9_jtag_callback(enum jtag_event event, void *priv)
{
	target_t *target = priv;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
	/* a test-logic reset occured
	 * the EmbeddedICE registers have been reset
	 * hardware breakpoints have been cleared
	 */
	if (event == JTAG_TRST_ASSERTED)
	{
		arm7_9->reinit_embeddedice = 1;
	}
	
	return ERROR_OK;
}

int arm7_9_get_arch_pointers(target_t *target, armv4_5_common_t **armv4_5_p, arm7_9_common_t **arm7_9_p)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
	if (armv4_5->common_magic != ARMV4_5_COMMON_MAGIC)
	{
		return -1;
	}
	
	if (arm7_9->common_magic != ARM7_9_COMMON_MAGIC)
	{
		return -1;
	}
	
	*armv4_5_p = armv4_5;
	*arm7_9_p = arm7_9;
	
	return ERROR_OK;
}

int arm7_9_set_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (arm7_9->force_hw_bkpts)
		breakpoint->type = BKPT_HARD;

	if (breakpoint->set)
	{
		WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		/* either an ARM (4 byte) or Thumb (2 byte) breakpoint */
		u32 mask = (breakpoint->length == 4) ? 0x3u : 0x1u;
		if (!arm7_9->wp0_used)
		{
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_VALUE], breakpoint->address);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK], mask);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK], 0xffffffffu);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK], ~EICE_W_CTRL_nOPC & 0xff);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], EICE_W_CTRL_ENABLE);

			jtag_execute_queue();
			arm7_9->wp0_used = 1;
			breakpoint->set = 1;
		}
		else if (!arm7_9->wp1_used)
		{
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_VALUE], breakpoint->address);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_MASK], mask);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_DATA_MASK], 0xffffffffu);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_MASK], ~EICE_W_CTRL_nOPC & 0xff);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE], EICE_W_CTRL_ENABLE);

			jtag_execute_queue();
			arm7_9->wp1_used = 1;
			breakpoint->set = 2;
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
			/* write the breakpoint instruction in target endianness (arm7_9->arm_bkpt is host endian) */
			target_write_u32(target, breakpoint->address, arm7_9->arm_bkpt);
		}
		else
		{
			/* keep the original instruction in target endianness */
			target->type->read_memory(target, breakpoint->address, 2, 1, breakpoint->orig_instr);
			/* write the breakpoint instruction in target endianness (arm7_9->thumb_bkpt is host endian) */
			target_write_u16(target, breakpoint->address, arm7_9->thumb_bkpt);
		}
		breakpoint->set = 1;
	}

	return ERROR_OK;

}

int arm7_9_unset_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
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
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], 0x0);
			jtag_execute_queue();
			arm7_9->wp0_used = 0;
		}
		else if (breakpoint->set == 2)
		{
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE], 0x0);
			jtag_execute_queue();
			arm7_9->wp1_used = 0;
		}
		breakpoint->set = 0;
	}
	else
	{
		/* restore original instruction (kept in target endianness) */
		if (breakpoint->length == 4)
		{
			u32 current_instr;
			/* check that user program as not modified breakpoint instruction */
			target->type->read_memory(target, breakpoint->address, 4, 1, (u8*)&current_instr);
			if (current_instr==arm7_9->arm_bkpt)
				target->type->write_memory(target, breakpoint->address, 4, 1, breakpoint->orig_instr);
		}
		else
		{
			u16 current_instr;
			/* check that user program as not modified breakpoint instruction */
			target->type->read_memory(target, breakpoint->address, 2, 1, (u8*)&current_instr);
			if (current_instr==arm7_9->thumb_bkpt)
				target->type->write_memory(target, breakpoint->address, 2, 1, breakpoint->orig_instr);
		}
		breakpoint->set = 0;
	}

	return ERROR_OK;
}

int arm7_9_add_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (arm7_9->force_hw_bkpts)
	{
		DEBUG("forcing use of hardware breakpoint at address 0x%8.8x", breakpoint->address);
		breakpoint->type = BKPT_HARD;
	}
	
	if ((breakpoint->type == BKPT_SOFT) && (arm7_9->sw_bkpts_enabled == 0))
	{
		INFO("sw breakpoint requested, but software breakpoints not enabled");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	
	if ((breakpoint->type == BKPT_HARD) && (arm7_9->wp_available < 1))
	{
		INFO("no watchpoint unit available for hardware breakpoint");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	
	if ((breakpoint->length != 2) && (breakpoint->length != 4))
	{
		INFO("only breakpoints of two (Thumb) or four (ARM) bytes length supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	
	if (breakpoint->type == BKPT_HARD)
		arm7_9->wp_available--;
	
	return ERROR_OK;
}

int arm7_9_remove_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (breakpoint->set)
	{
		arm7_9_unset_breakpoint(target, breakpoint);
	}
	
	if (breakpoint->type == BKPT_HARD)
		arm7_9->wp_available++;
	
	return ERROR_OK;
}

int arm7_9_set_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	int rw_mask = 1;
	u32 mask;
	
	mask = watchpoint->length - 1;
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (watchpoint->rw == WPT_ACCESS)
		rw_mask = 0;
	else
		rw_mask = 1;
	
	if (!arm7_9->wp0_used)
	{
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_VALUE], watchpoint->address);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK], mask);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK], watchpoint->mask);
		if( watchpoint->mask != 0xffffffffu )
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_VALUE], watchpoint->value);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK], 0xff & ~EICE_W_CTRL_nOPC & ~rw_mask);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], EICE_W_CTRL_ENABLE | EICE_W_CTRL_nOPC | (watchpoint->rw & 1));

		jtag_execute_queue();
		watchpoint->set = 1;
		arm7_9->wp0_used = 2;
	}
	else if (!arm7_9->wp1_used)
	{
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_VALUE], watchpoint->address);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_MASK], mask);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_DATA_MASK], watchpoint->mask);
		if( watchpoint->mask != 0xffffffffu )
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_DATA_VALUE], watchpoint->value);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_MASK], 0xff & ~EICE_W_CTRL_nOPC & ~rw_mask);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE], EICE_W_CTRL_ENABLE | EICE_W_CTRL_nOPC | (watchpoint->rw & 1));

		jtag_execute_queue();
		watchpoint->set = 2;
		arm7_9->wp1_used = 2;
	} 
	else
	{
		ERROR("BUG: no hardware comparator available");
		return ERROR_OK;
	}
	
	return ERROR_OK;
}

int arm7_9_unset_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
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
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], 0x0);
		jtag_execute_queue();
		arm7_9->wp0_used = 0;
	}
	else if (watchpoint->set == 2)
	{
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE], 0x0);
		jtag_execute_queue();
		arm7_9->wp1_used = 0;
	}
	watchpoint->set = 0;

	return ERROR_OK;
}

int arm7_9_add_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (arm7_9->wp_available < 1)
	{
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	
	if ((watchpoint->length != 1) && (watchpoint->length != 2) && (watchpoint->length != 4))
	{
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	
	arm7_9->wp_available--;
		
	return ERROR_OK;
}

int arm7_9_remove_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (watchpoint->set)
	{
		arm7_9_unset_watchpoint(target, watchpoint);
	}
		
	arm7_9->wp_available++;
	
	return ERROR_OK;
}

int arm7_9_enable_sw_bkpts(struct target_s *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	int retval;
	
	if (arm7_9->sw_bkpts_enabled)
		return ERROR_OK;
	
	if (arm7_9->wp_available < 1)
	{
		WARNING("can't enable sw breakpoints with no watchpoint unit available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	arm7_9->wp_available--;
	
	if (!arm7_9->wp0_used)
	{
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_VALUE], arm7_9->arm_bkpt);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK], 0x0);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK], 0xffffffffu);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK], ~EICE_W_CTRL_nOPC & 0xff);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], EICE_W_CTRL_ENABLE);
		arm7_9->sw_bkpts_enabled = 1;
		arm7_9->wp0_used = 3;
	}
	else if (!arm7_9->wp1_used)
	{
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_DATA_VALUE], arm7_9->arm_bkpt);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_DATA_MASK], 0x0);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_MASK], 0xffffffffu);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_MASK], ~EICE_W_CTRL_nOPC & 0xff);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE], EICE_W_CTRL_ENABLE);
		arm7_9->sw_bkpts_enabled = 2;
		arm7_9->wp1_used = 3;
	}
	else
	{
		ERROR("BUG: both watchpoints used, but wp_available >= 1");
		exit(-1);
	}
	
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		ERROR("error writing EmbeddedICE registers to enable sw breakpoints");
		exit(-1);
	};
	
	return ERROR_OK;
}

int arm7_9_disable_sw_bkpts(struct target_s *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
	if (!arm7_9->sw_bkpts_enabled)
		return ERROR_OK;
	
	if (arm7_9->sw_bkpts_enabled == 1)
	{
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], 0x0);
		arm7_9->sw_bkpts_enabled = 0;
		arm7_9->wp0_used = 0;
		arm7_9->wp_available++;
	}
	else if (arm7_9->sw_bkpts_enabled == 2)
	{
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE], 0x0);
		arm7_9->sw_bkpts_enabled = 0;
		arm7_9->wp1_used = 0;
		arm7_9->wp_available++;
	}

	return ERROR_OK;
}

int arm7_9_execute_sys_speed(struct target_s *target)
{
	int timeout;
	int retval;
	
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	reg_t *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];
				
	/* set RESTART instruction */
	jtag_add_end_state(TAP_RTI);
	arm_jtag_set_instr(jtag_info, 0x4, NULL);
	
	for (timeout=0; timeout<50; timeout++)
	{
		/* read debug status register */
		embeddedice_read_reg(dbg_stat);
		if ((retval = jtag_execute_queue()) != ERROR_OK)
			return retval;
		if ((buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_DBGACK, 1))
				   && (buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_SYSCOMP, 1)))
			break;
		usleep(100000);	
	}
	if (timeout == 50)
	{
		ERROR("timeout waiting for SYSCOMP & DBGACK, last DBG_STATUS: %x", buf_get_u32(dbg_stat->value, 0, dbg_stat->size));
		return ERROR_TARGET_TIMEOUT;
	}
	
	return ERROR_OK;
}

int arm7_9_execute_fast_sys_speed(struct target_s *target)
{
	u8 check_value[4], check_mask[4];
	
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	reg_t *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];
				
	/* set RESTART instruction */
	jtag_add_end_state(TAP_RTI);
	arm_jtag_set_instr(jtag_info, 0x4, NULL);
	
	/* check for DBGACK and SYSCOMP set (others don't care) */
	buf_set_u32(check_value, 0, 32, 0x9);
	buf_set_u32(check_mask, 0, 32, 0x9);
	
	/* read debug status register */
	embeddedice_read_reg_w_check(dbg_stat, check_value, check_value);

	return ERROR_OK;
}

int arm7_9_target_request_data(target_t *target, u32 size, u8 *buffer)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	u32 *data;
	int i;
	
	data = malloc(size * (sizeof(u32)));
	
	embeddedice_receive(jtag_info, data, size);
	
	for (i = 0; i < size; i++)
	{
		h_u32_to_le(buffer + (i * 4), data[i]);
	}
	
	free(data);
	
	return ERROR_OK;
}

int arm7_9_handle_target_request(void *priv)
{
	target_t *target = priv;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info; 
	reg_t *dcc_control = &arm7_9->eice_cache->reg_list[EICE_COMMS_CTRL];
	
	if (target->state == TARGET_RUNNING)
	{
		/* read DCC control register */
		embeddedice_read_reg(dcc_control);
		jtag_execute_queue();
		
		/* check W bit */
		if (buf_get_u32(dcc_control->value, 1, 1) == 1)
		{
			u32 request;
			
			embeddedice_receive(jtag_info, &request, 1);
			target_request(target, request);
		}
	}
	
	return ERROR_OK;
}

enum target_state arm7_9_poll(target_t *target)
{
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	reg_t *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];

	if (arm7_9->reinit_embeddedice)
	{
		arm7_9_reinit_embeddedice(target);
	}
	
	/* read debug status register */
	embeddedice_read_reg(dbg_stat);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		switch (retval)
		{
			case ERROR_JTAG_QUEUE_FAILED:
				ERROR("JTAG queue failed while reading EmbeddedICE status register");
				exit(-1);
				break;
			default:
				break;
		}
	}
	
	if (buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_DBGACK, 1))
	{
		DEBUG("DBGACK set, dbg_state->value: 0x%x", buf_get_u32(dbg_stat->value, 0, 32));
		if ((target->state == TARGET_UNKNOWN))
		{
			WARNING("DBGACK set while target was in unknown state. Reset or initialize target before resuming");
			target->state = TARGET_RUNNING;
		}
		if ((target->state == TARGET_RUNNING) || (target->state == TARGET_RESET))
		{
			target->state = TARGET_HALTED;
			if ((retval = arm7_9_debug_entry(target)) != ERROR_OK)
				return retval;
			
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
		if (target->state == TARGET_DEBUG_RUNNING)
		{
			target->state = TARGET_HALTED;
			if ((retval = arm7_9_debug_entry(target)) != ERROR_OK)
				return retval;
			
			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		}
	}
	else
	{
		if (target->state != TARGET_DEBUG_RUNNING)
			target->state = TARGET_RUNNING;
	}
	
	return target->state;
}

int arm7_9_assert_reset(target_t *target)
{
	int retval;
	
	DEBUG("target->state: %s", target_state_strings[target->state]);
	
	if (target->state == TARGET_HALTED || target->state == TARGET_UNKNOWN)
	{
		/* if the target wasn't running, there might be working areas allocated */
		target_free_all_working_areas(target);
		
		/* assert SRST and TRST */
		/* system would get ouf sync if we didn't reset test-logic, too */
		if ((retval = jtag_add_reset(1, 1)) != ERROR_OK)
		{
			if (retval == ERROR_JTAG_RESET_CANT_SRST)
			{
				WARNING("can't assert srst");
				return retval;
			}
			else
			{
				ERROR("unknown error");
				exit(-1);
			}
		}
		jtag_add_sleep(5000);
		if ((retval = jtag_add_reset(0, 1)) != ERROR_OK)
		{
			if (retval == ERROR_JTAG_RESET_WOULD_ASSERT_TRST)
			{
				WARNING("srst resets test logic, too");
				retval = jtag_add_reset(1, 1);
			}
		}
	}
	else
	{
		if ((retval = jtag_add_reset(0, 1)) != ERROR_OK)
		{
			if (retval == ERROR_JTAG_RESET_WOULD_ASSERT_TRST)
			{
				WARNING("srst resets test logic, too");
				retval = jtag_add_reset(1, 1);
			}
			
			if (retval == ERROR_JTAG_RESET_CANT_SRST)
			{
				WARNING("can't assert srst");
				return retval;
			}
			else if (retval != ERROR_OK)
			{
				ERROR("unknown error");
				exit(-1);
			}
		}
	}
	
	target->state = TARGET_RESET;
	jtag_add_sleep(50000);
	
	armv4_5_invalidate_core_regs(target);

	return ERROR_OK;

}

int arm7_9_deassert_reset(target_t *target)
{
	DEBUG("target->state: %s", target_state_strings[target->state]);
	
	/* deassert reset lines */
	jtag_add_reset(0, 0);
	
	return ERROR_OK;
}

int arm7_9_clear_halt(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	reg_t *dbg_ctrl = &arm7_9->eice_cache->reg_list[EICE_DBG_CTRL];
	
	/* we used DBGRQ only if we didn't come out of reset */
	if (!arm7_9->debug_entry_from_reset && arm7_9->use_dbgrq)
	{
		/* program EmbeddedICE Debug Control Register to deassert DBGRQ
		 */
		buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGRQ, 1, 0);	
		embeddedice_store_reg(dbg_ctrl);
	}
	else
	{
		if (arm7_9->debug_entry_from_reset && arm7_9->has_vector_catch)
		{
			/* if we came out of reset, and vector catch is supported, we used
			 * vector catch to enter debug state
			 * restore the register in that case
			 */
			embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_VEC_CATCH]);
		}
		else
		{
			/* restore registers if watchpoint unit 0 was in use
			 */
			if (arm7_9->wp0_used)
			{
				embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK]);
				embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK]);
				embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK]);
			}
			/* control value always has to be restored, as it was either disabled, 
			 * or enabled with possibly different bits
			 */
			embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE]);
		}
	}
	
	return ERROR_OK;
}

int arm7_9_soft_reset_halt(struct target_s *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	reg_t *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];
	reg_t *dbg_ctrl = &arm7_9->eice_cache->reg_list[EICE_DBG_CTRL];
	int i;
	
	if (target->state == TARGET_RUNNING)
	{
		target->type->halt(target);
	}
	
	while (buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_DBGACK, 1) == 0)
	{
		embeddedice_read_reg(dbg_stat);
		jtag_execute_queue();
	}
	target->state = TARGET_HALTED;
	
	/* program EmbeddedICE Debug Control Register to assert DBGACK and INTDIS
	 * ensure that DBGRQ is cleared
	 */
	buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGACK, 1, 1);
	buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGRQ, 1, 0);
	buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_INTDIS, 1, 1);
	embeddedice_store_reg(dbg_ctrl);
	
	arm7_9_clear_halt(target);
	
	/* if the target is in Thumb state, change to ARM state */
	if (buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_ITBIT, 1))
	{
		u32 r0_thumb, pc_thumb;
		DEBUG("target entered debug from Thumb state, changing to ARM");
		/* Entered debug from Thumb mode */
		armv4_5->core_state = ARMV4_5_STATE_THUMB;
		arm7_9->change_to_arm(target, &r0_thumb, &pc_thumb);
	}
	
	/* all register content is now invalid */
	armv4_5_invalidate_core_regs(target);
	
	/* SVC, ARM state, IRQ and FIQ disabled */
	buf_set_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 8, 0xd3);
	armv4_5->core_cache->reg_list[ARMV4_5_CPSR].dirty = 1;
	armv4_5->core_cache->reg_list[ARMV4_5_CPSR].valid = 1;
	
	/* start fetching from 0x0 */
	buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, 0x0);
	armv4_5->core_cache->reg_list[15].dirty = 1;
	armv4_5->core_cache->reg_list[15].valid = 1;
	
	armv4_5->core_mode = ARMV4_5_MODE_SVC;
	armv4_5->core_state = ARMV4_5_STATE_ARM;
	
	/* reset registers */
	for (i = 0; i <= 14; i++)
	{	
		buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).value, 0, 32, 0xffffffff);
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).dirty = 1;
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).valid = 1;
	}
	
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	
	return ERROR_OK;
}

int arm7_9_prepare_reset_halt(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
	/* poll the target, and resume if it was currently halted */
	arm7_9_poll(target);
	if (target->state == TARGET_HALTED)
	{
		arm7_9_resume(target, 1, 0x0, 0, 1);
	}
	
	if (arm7_9->has_vector_catch)
	{
		/* program vector catch register to catch reset vector */
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_VEC_CATCH], 0x1);
	}
	else
	{
		/* program watchpoint unit to match on reset vector address */
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK], 0x3);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK], 0x0);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], 0x100);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK], 0xf7);
	}
	
	return ERROR_OK;
}

int arm7_9_halt(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	reg_t *dbg_ctrl = &arm7_9->eice_cache->reg_list[EICE_DBG_CTRL];
	
	DEBUG("target->state: %s", target_state_strings[target->state]);
	
	if (target->state == TARGET_HALTED)
	{
		WARNING("target was already halted");
		return ERROR_TARGET_ALREADY_HALTED;
	}
	
	if (target->state == TARGET_UNKNOWN)
	{
		WARNING("target was in unknown state when halt was requested");
	}
	
	if (target->state == TARGET_RESET) 
	{
		if ((jtag_reset_config & RESET_SRST_PULLS_TRST) && jtag_srst)
		{
			ERROR("can't request a halt while in reset if nSRST pulls nTRST");
			return ERROR_TARGET_FAILURE;
		}
		else
		{
			/* we came here in a reset_halt or reset_init sequence
			 * debug entry was already prepared in arm7_9_prepare_reset_halt()
			 */
			target->debug_reason = DBG_REASON_DBGRQ;
			
			return ERROR_OK; 
		}
	}

	if (arm7_9->use_dbgrq)
	{
		/* program EmbeddedICE Debug Control Register to assert DBGRQ
		 */
		buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGRQ, 1, 1);	
		embeddedice_store_reg(dbg_ctrl);
	}
	else
	{
		/* program watchpoint unit to match on any address
		 */
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK], 0xffffffff);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK], 0xffffffff);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], 0x100);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK], 0xf7);
	}

	target->debug_reason = DBG_REASON_DBGRQ;
	
	return ERROR_OK;
}

int arm7_9_debug_entry(target_t *target)
{
	int i;
	u32 context[16];
	u32* context_p[16];
	u32 r0_thumb, pc_thumb;
	u32 cpsr;
	int retval;
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	reg_t *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];
	reg_t *dbg_ctrl = &arm7_9->eice_cache->reg_list[EICE_DBG_CTRL];

#ifdef _DEBUG_ARM7_9_
	DEBUG("-");
#endif

	if (arm7_9->pre_debug_entry)
		arm7_9->pre_debug_entry(target);

	/* program EmbeddedICE Debug Control Register to assert DBGACK and INTDIS
	 * ensure that DBGRQ is cleared
	 */
	buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGACK, 1, 1);
	buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGRQ, 1, 0);
	buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_INTDIS, 1, 1);
	embeddedice_store_reg(dbg_ctrl);
	
	arm7_9_clear_halt(target);
	
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		switch (retval)
		{
			case ERROR_JTAG_QUEUE_FAILED:
				ERROR("JTAG queue failed while writing EmbeddedICE control register");
				exit(-1);
				break;
			default:
				break;
		}
	}

	if ((retval = arm7_9->examine_debug_reason(target)) != ERROR_OK)
		return retval;


	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	/* if the target is in Thumb state, change to ARM state */
	if (buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_ITBIT, 1))
	{
		DEBUG("target entered debug from Thumb state");
		/* Entered debug from Thumb mode */
		armv4_5->core_state = ARMV4_5_STATE_THUMB;
		arm7_9->change_to_arm(target, &r0_thumb, &pc_thumb);
		DEBUG("r0_thumb: 0x%8.8x, pc_thumb: 0x%8.8x", r0_thumb, pc_thumb);
	}
	else
	{
		DEBUG("target entered debug from ARM state");
		/* Entered debug from ARM mode */
		armv4_5->core_state = ARMV4_5_STATE_ARM;
	}
	
	for (i = 0; i < 16; i++)
		context_p[i] = &context[i];
	/* save core registers (r0 - r15 of current core mode) */
	arm7_9->read_core_regs(target, 0xffff, context_p);

	arm7_9->read_xpsr(target, &cpsr, 0);
	
	if ((retval = jtag_execute_queue()) != ERROR_OK)
		return retval;
	
	/* if the core has been executing in Thumb state, set the T bit */
	if (armv4_5->core_state == ARMV4_5_STATE_THUMB)
		cpsr |= 0x20;	
	
	buf_set_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32, cpsr);
	armv4_5->core_cache->reg_list[ARMV4_5_CPSR].dirty = 0;
	armv4_5->core_cache->reg_list[ARMV4_5_CPSR].valid = 1;
	
	armv4_5->core_mode = cpsr & 0x1f;
	
	if (armv4_5_mode_to_number(armv4_5->core_mode) == -1)
	{
		target->state = TARGET_UNKNOWN;
		ERROR("cpsr contains invalid mode value - communication failure");
		return ERROR_TARGET_FAILURE;
	}

	DEBUG("target entered debug state in %s mode", armv4_5_mode_strings[armv4_5_mode_to_number(armv4_5->core_mode)]);
	
	if (armv4_5->core_state == ARMV4_5_STATE_THUMB)
	{
		DEBUG("thumb state, applying fixups");
		context[0] = r0_thumb;
		context[15] = pc_thumb;
	} else if (armv4_5->core_state == ARMV4_5_STATE_ARM)
	{
		/* adjust value stored by STM */
		context[15] -= 3 * 4;
	}

	if ((target->debug_reason == DBG_REASON_BREAKPOINT)
			|| (target->debug_reason == DBG_REASON_SINGLESTEP)
			|| (target->debug_reason == DBG_REASON_WATCHPOINT)
			|| (target->debug_reason == DBG_REASON_WPTANDBKPT)
			|| ((target->debug_reason == DBG_REASON_DBGRQ) && (arm7_9->use_dbgrq == 0)))
		context[15] -= 3 * ((armv4_5->core_state == ARMV4_5_STATE_ARM) ? 4 : 2);
	else if (target->debug_reason == DBG_REASON_DBGRQ)
		context[15] -= arm7_9->dbgreq_adjust_pc * ((armv4_5->core_state == ARMV4_5_STATE_ARM) ? 4 : 2);
	else
	{
		ERROR("unknown debug reason: %i", target->debug_reason);
	}

	
	for (i=0; i<=15; i++)
	{
		DEBUG("r%i: 0x%8.8x", i, context[i]);
		buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).value, 0, 32, context[i]);
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).dirty = 0;
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).valid = 1;
	}
	
	DEBUG("entered debug state at PC 0x%x", context[15]);

	/* exceptions other than USR & SYS have a saved program status register */
	if ((armv4_5_mode_to_number(armv4_5->core_mode) != ARMV4_5_MODE_USR) && (armv4_5_mode_to_number(armv4_5->core_mode) != ARMV4_5_MODE_SYS))
	{
		u32 spsr;
		arm7_9->read_xpsr(target, &spsr, 1);
		jtag_execute_queue();
		buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 16).value, 0, 32, spsr);
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 16).dirty = 0;
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 16).valid = 1;
	}

	/* r0 and r15 (pc) have to be restored later */
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 0).dirty = 1;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 15).dirty = 1;

	if ((retval = jtag->execute_queue()) != ERROR_OK)
		return retval;

	if (arm7_9->post_debug_entry)
		arm7_9->post_debug_entry(target);

	return ERROR_OK;
}

int arm7_9_full_context(target_t *target)
{
	int i;
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	DEBUG("-");
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* iterate through processor modes (User, FIQ, IRQ, SVC, ABT, UND)
	 * SYS shares registers with User, so we don't touch SYS
	 */
	for(i = 0; i < 6; i++)
	{
		u32 mask = 0;
		u32* reg_p[16];
		int j;
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
			
			/* change processor mode (and mask T bit) */
			tmp_cpsr = buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 8) & 0xE0;
			tmp_cpsr |= armv4_5_number_to_mode(i);
			tmp_cpsr &= ~0x20;
			arm7_9->write_xpsr_im8(target, tmp_cpsr & 0xff, 0, 0);

			for (j = 0; j < 15; j++)
			{
				if (ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), j).valid == 0)
				{	
					reg_p[j] = (u32*)ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), j).value;
					mask |= 1 << j;
					ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), j).valid = 1;
					ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), j).dirty = 0;
				}
			}
			
			/* if only the PSR is invalid, mask is all zeroes */
			if (mask)
				arm7_9->read_core_regs(target, mask, reg_p);
			
			/* check if the PSR has to be read */
			if (ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), 16).valid == 0)
			{
				arm7_9->read_xpsr(target, (u32*)ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), 16).value, 1);
				ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), 16).valid = 1;
				ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), 16).dirty = 0;
			}
		}
	}

	/* restore processor mode (mask T bit) */
	arm7_9->write_xpsr_im8(target, buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 8) & ~0x20, 0, 0);
	
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		ERROR("JTAG failure");
		exit(-1);
	}
	return ERROR_OK;
}

int arm7_9_restore_context(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	reg_t *reg; 
	armv4_5_core_reg_t *reg_arch_info;
	enum armv4_5_mode current_mode = armv4_5->core_mode;
	int i, j;
	int dirty;
	int mode_change;
	
	DEBUG("-");
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (arm7_9->pre_restore_context)
		arm7_9->pre_restore_context(target);
	
	/* iterate through processor modes (User, FIQ, IRQ, SVC, ABT, UND)
	 * SYS shares registers with User, so we don't touch SYS
	 */
	for (i = 0; i < 6; i++)
	{
		DEBUG("examining %s mode", armv4_5_mode_strings[i]);
		dirty = 0;
		mode_change = 0;
		/* check if there are dirty registers in the current mode 
		*/
		for (j = 0; j <= 16; j++)
		{
			reg = &ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), j);
			reg_arch_info = reg->arch_info;
			if (reg->dirty == 1)
			{
				if (reg->valid == 1)
				{
					dirty = 1;
					DEBUG("examining dirty reg: %s", reg->name);
					if ((reg_arch_info->mode != ARMV4_5_MODE_ANY)
						&& (reg_arch_info->mode != current_mode)
						&& !((reg_arch_info->mode == ARMV4_5_MODE_USR) && (armv4_5->core_mode == ARMV4_5_MODE_SYS)) 
						&& !((reg_arch_info->mode == ARMV4_5_MODE_SYS) && (armv4_5->core_mode == ARMV4_5_MODE_USR)))
					{
						mode_change = 1;
						DEBUG("require mode change");
					}
				}
				else
				{
					ERROR("BUG: dirty register '%s', but no valid data", reg->name);
					exit(-1);
				}
			}
		}
		
		if (dirty)
		{
			u32 mask = 0x0;
			int num_regs = 0;
			u32 regs[16];

			if (mode_change)
			{
				u32 tmp_cpsr;
			
				/* change processor mode (mask T bit) */
				tmp_cpsr = buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 8) & 0xE0;
				tmp_cpsr |= armv4_5_number_to_mode(i);
				tmp_cpsr &= ~0x20;
				arm7_9->write_xpsr_im8(target, tmp_cpsr & 0xff, 0, 0);
				current_mode = armv4_5_number_to_mode(i);
			}
			
			for (j = 0; j <= 14; j++)
			{
				reg = &ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), j);
				reg_arch_info = reg->arch_info;
				
				
				if (reg->dirty == 1)
				{
					regs[j] = buf_get_u32(reg->value, 0, 32);
					mask |= 1 << j;
					num_regs++;
					reg->dirty = 0;
					reg->valid = 1;
					DEBUG("writing register %i of mode %s with value 0x%8.8x", j, armv4_5_mode_strings[i], regs[j]);
				}
			}
			
			if (mask)
			{
				arm7_9->write_core_regs(target, mask, regs);
			}
			
			reg = &ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_number_to_mode(i), 16);
			reg_arch_info = reg->arch_info;
			if ((reg->dirty) && (reg_arch_info->mode != ARMV4_5_MODE_ANY))
			{
				DEBUG("writing SPSR of mode %i with value 0x%8.8x", i, buf_get_u32(reg->value, 0, 32));
				arm7_9->write_xpsr(target, buf_get_u32(reg->value, 0, 32), 1);
			}
		}
	}
	
	if ((armv4_5->core_cache->reg_list[ARMV4_5_CPSR].dirty == 0) && (armv4_5->core_mode != current_mode))
	{
		/* restore processor mode (mask T bit) */
		u32 tmp_cpsr;
			
		tmp_cpsr = buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 8) & 0xE0;
		tmp_cpsr |= armv4_5_number_to_mode(i);
		tmp_cpsr &= ~0x20;
		DEBUG("writing lower 8 bit of cpsr with value 0x%2.2x", tmp_cpsr);
		arm7_9->write_xpsr_im8(target, tmp_cpsr & 0xff, 0, 0);
	}
	else if (armv4_5->core_cache->reg_list[ARMV4_5_CPSR].dirty == 1)
	{
		/* CPSR has been changed, full restore necessary (mask T bit) */
		DEBUG("writing cpsr with value 0x%8.8x", buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32));
		arm7_9->write_xpsr(target, buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32) & ~0x20, 0);
		armv4_5->core_cache->reg_list[ARMV4_5_CPSR].dirty = 0;
		armv4_5->core_cache->reg_list[ARMV4_5_CPSR].valid = 1;
	}
	
	/* restore PC */
	DEBUG("writing PC with value 0x%8.8x", buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));
	arm7_9->write_pc(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));
	armv4_5->core_cache->reg_list[15].dirty = 0;
			
	if (arm7_9->post_restore_context)
		arm7_9->post_restore_context(target);

	return ERROR_OK;
}

int arm7_9_restart_core(struct target_s *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	
	/* set RESTART instruction */
	jtag_add_end_state(TAP_RTI);
	arm_jtag_set_instr(jtag_info, 0x4, NULL);
	
	jtag_add_runtest(1, TAP_RTI);
	if ((jtag_execute_queue()) != ERROR_OK)
	{
		exit(-1);
	}
	
	return ERROR_OK;
}

void arm7_9_enable_watchpoints(struct target_s *target)
{
	watchpoint_t *watchpoint = target->watchpoints;
	
	while (watchpoint)
	{
		if (watchpoint->set == 0)
			arm7_9_set_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}
}

void arm7_9_enable_breakpoints(struct target_s *target)
{
	breakpoint_t *breakpoint = target->breakpoints;
	
	/* set any pending breakpoints */
	while (breakpoint)
	{
		if (breakpoint->set == 0)
			arm7_9_set_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}
}

void arm7_9_disable_bkpts_and_wpts(struct target_s *target)
{
	breakpoint_t *breakpoint = target->breakpoints;
	watchpoint_t *watchpoint = target->watchpoints;

	/* set any pending breakpoints */
	while (breakpoint)
	{
		if (breakpoint->set != 0)
			arm7_9_unset_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}
	
	while (watchpoint)
	{
		if (watchpoint->set != 0)
			arm7_9_unset_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}
}

int arm7_9_resume(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	breakpoint_t *breakpoint = target->breakpoints;
	reg_t *dbg_ctrl = &arm7_9->eice_cache->reg_list[EICE_DBG_CTRL];
	
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
	
	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
		buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, address);
	
	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints)
	{
		if ((breakpoint = breakpoint_find(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32))))
		{
			DEBUG("unset breakpoint at 0x%8.8x", breakpoint->address);
			arm7_9_unset_breakpoint(target, breakpoint);
			
			DEBUG("enable single-step");
			arm7_9->enable_single_step(target);
			
			target->debug_reason = DBG_REASON_SINGLESTEP;

			arm7_9_restore_context(target);
			
			if (armv4_5->core_state == ARMV4_5_STATE_ARM)
				arm7_9->branch_resume(target);
			else if (armv4_5->core_state == ARMV4_5_STATE_THUMB)
			{
				arm7_9->branch_resume_thumb(target);
			}
			else
			{
				ERROR("unhandled core state");
				exit(-1);
			}
				
			buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGACK, 1, 0);
			embeddedice_write_reg(dbg_ctrl, buf_get_u32(dbg_ctrl->value, 0, dbg_ctrl->size));
			arm7_9_execute_sys_speed(target);
			
			DEBUG("disable single-step");
			arm7_9->disable_single_step(target);
			
			arm7_9_debug_entry(target);
			DEBUG("new PC after step: 0x%8.8x", buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));
		
			DEBUG("set breakpoint at 0x%8.8x", breakpoint->address);
			arm7_9_set_breakpoint(target, breakpoint);
		}
	}
	
	/* enable any pending breakpoints and watchpoints */
	arm7_9_enable_breakpoints(target);
	arm7_9_enable_watchpoints(target);
	
	arm7_9_restore_context(target);
	
	if (armv4_5->core_state == ARMV4_5_STATE_ARM)
	{
		arm7_9->branch_resume(target);
	}
	else if (armv4_5->core_state == ARMV4_5_STATE_THUMB)
	{
		arm7_9->branch_resume_thumb(target);
	}
	else
	{
		ERROR("unhandled core state");
		exit(-1);
	}
	
	/* deassert DBGACK and INTDIS */
	buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGACK, 1, 0);
	/* INTDIS only when we really resume, not during debug execution */
	if (!debug_execution)
		buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_INTDIS, 1, 0);
	embeddedice_write_reg(dbg_ctrl, buf_get_u32(dbg_ctrl->value, 0, dbg_ctrl->size));
	
	arm7_9_restart_core(target);
	
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
	
	return ERROR_OK;
}

void arm7_9_enable_eice_step(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
	/* setup an inverse breakpoint on the current PC
	* - comparator 1 matches the current address
	* - rangeout from comparator 1 is connected to comparator 0 rangein
	* - comparator 0 matches any address, as long as rangein is low */
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK], 0xffffffff);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK], 0xffffffff);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], 0x100);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK], 0x77);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_VALUE], buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_MASK], 0);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_DATA_MASK], 0xffffffff);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE], 0x0);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_MASK], 0xf7);
}

void arm7_9_disable_eice_step(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_VALUE]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_MASK]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W1_DATA_MASK]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_MASK]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE]);
}

int arm7_9_step(struct target_s *target, int current, u32 address, int handle_breakpoints)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	breakpoint_t *breakpoint = NULL;

	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
		buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, address);
	
	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints)
		if ((breakpoint = breakpoint_find(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32))))
			arm7_9_unset_breakpoint(target, breakpoint);
	
	target->debug_reason = DBG_REASON_SINGLESTEP;

	arm7_9_restore_context(target);
	
	arm7_9->enable_single_step(target);
	
	if (armv4_5->core_state == ARMV4_5_STATE_ARM)
	{
		arm7_9->branch_resume(target);
	}
	else if (armv4_5->core_state == ARMV4_5_STATE_THUMB)
	{
		arm7_9->branch_resume_thumb(target);
	}
	else
	{
		ERROR("unhandled core state");
		exit(-1);
	}
	
	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	arm7_9_execute_sys_speed(target);
	arm7_9->disable_single_step(target);
	
	/* registers are now invalid */
	armv4_5_invalidate_core_regs(target);
	
	arm7_9_debug_entry(target);
	
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	if (breakpoint)
		arm7_9_set_breakpoint(target, breakpoint);
	
	DEBUG("target stepped");

	return ERROR_OK;

}

int arm7_9_read_core_reg(struct target_s *target, int num, enum armv4_5_mode mode)
{
	u32* reg_p[16];
	u32 value;
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	enum armv4_5_mode reg_mode = ((armv4_5_core_reg_t*)ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, mode, num).arch_info)->mode;
	
	if ((num < 0) || (num > 16))
		return ERROR_INVALID_ARGUMENTS;
	
	if ((mode != ARMV4_5_MODE_ANY)
			&& (mode != armv4_5->core_mode)
			&& (reg_mode != ARMV4_5_MODE_ANY))
	{
		u32 tmp_cpsr;
			
		/* change processor mode (mask T bit) */
		tmp_cpsr = buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 8) & 0xE0;
		tmp_cpsr |= mode;
		tmp_cpsr &= ~0x20;
		arm7_9->write_xpsr_im8(target, tmp_cpsr & 0xff, 0, 0);
	}
	
	if ((num >= 0) && (num <= 15))
	{
		/* read a normal core register */
		reg_p[num] = &value;
		
		arm7_9->read_core_regs(target, 1 << num, reg_p);
	}
	else
	{
		/* read a program status register
		 * if the register mode is MODE_ANY, we read the cpsr, otherwise a spsr
		 */
		armv4_5_core_reg_t *arch_info = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, mode, num).arch_info;
		int spsr = (arch_info->mode == ARMV4_5_MODE_ANY) ? 0 : 1;
		
		arm7_9->read_xpsr(target, &value, spsr);
	}
	
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		ERROR("JTAG failure");
		exit(-1);
	}
		
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, mode, num).valid = 1;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, mode, num).dirty = 0;
	buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, mode, num).value, 0, 32, value);
		
	if ((mode != ARMV4_5_MODE_ANY)
			&& (mode != armv4_5->core_mode)
			&& (reg_mode != ARMV4_5_MODE_ANY))	{
		/* restore processor mode (mask T bit) */
		arm7_9->write_xpsr_im8(target, buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 8) & ~0x20, 0, 0);
	}
	
	return ERROR_OK;
	
}

int arm7_9_write_core_reg(struct target_s *target, int num, enum armv4_5_mode mode, u32 value)
{
	u32 reg[16];
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	enum armv4_5_mode reg_mode = ((armv4_5_core_reg_t*)ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, mode, num).arch_info)->mode;

	if ((num < 0) || (num > 16))
		return ERROR_INVALID_ARGUMENTS;
	
	if ((mode != ARMV4_5_MODE_ANY)
			&& (mode != armv4_5->core_mode)
			&& (reg_mode != ARMV4_5_MODE_ANY))	{
		u32 tmp_cpsr;
			
		/* change processor mode (mask T bit) */
		tmp_cpsr = buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 8) & 0xE0;
		tmp_cpsr |= mode;
		tmp_cpsr &= ~0x20;
		arm7_9->write_xpsr_im8(target, tmp_cpsr & 0xff, 0, 0);
	}
	
	if ((num >= 0) && (num <= 15))
	{
		/* write a normal core register */
		reg[num] = value;
		
		arm7_9->write_core_regs(target, 1 << num, reg);
	}
	else
	{
		/* write a program status register
		* if the register mode is MODE_ANY, we write the cpsr, otherwise a spsr
		*/
		armv4_5_core_reg_t *arch_info = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, mode, num).arch_info;
		int spsr = (arch_info->mode == ARMV4_5_MODE_ANY) ? 0 : 1;
		
		/* if we're writing the CPSR, mask the T bit */
		if (!spsr)
			value &= ~0x20;
		
		arm7_9->write_xpsr(target, value, spsr);
	}
	
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, mode, num).valid = 1;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, mode, num).dirty = 0;
		
	if ((mode != ARMV4_5_MODE_ANY)
			&& (mode != armv4_5->core_mode)
			&& (reg_mode != ARMV4_5_MODE_ANY))	{
		/* restore processor mode (mask T bit) */
		arm7_9->write_xpsr_im8(target, buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 8) & ~0x20, 0, 0);
	}
	
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		ERROR("JTAG failure");
		exit(-1);
	}
	
	return ERROR_OK;
	
}

int arm7_9_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
	u32 reg[16];
	int num_accesses = 0;
	int thisrun_accesses;
	int i;
	u32 cpsr;
	int retval;
	int last_reg = 0;
	
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
	
	/* load the base register with the address of the first word */
	reg[0] = address;
	arm7_9->write_core_regs(target, 0x1, reg);
	
	switch (size)
	{
		case 4:
			while (num_accesses < count)
			{
				u32 reg_list;
				thisrun_accesses = ((count - num_accesses) >= 14) ? 14 : (count - num_accesses);
				reg_list = (0xffff >> (15 - thisrun_accesses)) & 0xfffe;
				
				if (last_reg <= thisrun_accesses)
					last_reg = thisrun_accesses;
				
				arm7_9->load_word_regs(target, reg_list);
				
				/* fast memory reads are only safe when the target is running
				 * from a sufficiently high clock (32 kHz is usually too slow)
				 */
				if (arm7_9->fast_memory_access)
					arm7_9_execute_fast_sys_speed(target);
				else
					arm7_9_execute_sys_speed(target);
									
				arm7_9->read_core_regs_target_buffer(target, reg_list, buffer, 4);
				
				/* advance buffer, count number of accesses */
				buffer += thisrun_accesses * 4;
				num_accesses += thisrun_accesses;
			}	
			break;
		case 2:
			while (num_accesses < count)
			{
				u32 reg_list;
				thisrun_accesses = ((count - num_accesses) >= 14) ? 14 : (count - num_accesses);
				reg_list = (0xffff >> (15 - thisrun_accesses)) & 0xfffe;
				
				for (i = 1; i <= thisrun_accesses; i++)
				{
					if (i > last_reg)
						last_reg = i;
					arm7_9->load_hword_reg(target, i);
					/* fast memory reads are only safe when the target is running
					 * from a sufficiently high clock (32 kHz is usually too slow)
					 */
					if (arm7_9->fast_memory_access)
						arm7_9_execute_fast_sys_speed(target);
					else
						arm7_9_execute_sys_speed(target);
				}
				
				arm7_9->read_core_regs_target_buffer(target, reg_list, buffer, 2);
				
				/* advance buffer, count number of accesses */
				buffer += thisrun_accesses * 2;
				num_accesses += thisrun_accesses;
			}	
			break;
		case 1:
			while (num_accesses < count)
			{
				u32 reg_list;
				thisrun_accesses = ((count - num_accesses) >= 14) ? 14 : (count - num_accesses);
				reg_list = (0xffff >> (15 - thisrun_accesses)) & 0xfffe;
				
				for (i = 1; i <= thisrun_accesses; i++)
				{
					if (i > last_reg)
						last_reg = i;
					arm7_9->load_byte_reg(target, i);
					/* fast memory reads are only safe when the target is running
					 * from a sufficiently high clock (32 kHz is usually too slow)
					 */
					if (arm7_9->fast_memory_access)
						arm7_9_execute_fast_sys_speed(target);
					else
						arm7_9_execute_sys_speed(target);
				}
				
				arm7_9->read_core_regs_target_buffer(target, reg_list, buffer, 1);
				
				/* advance buffer, count number of accesses */
				buffer += thisrun_accesses * 1;
				num_accesses += thisrun_accesses;
			}	
			break;
		default:
			ERROR("BUG: we shouldn't get here");
			exit(-1);
			break;
	}
	
	for (i=0; i<=last_reg; i++)
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).dirty = 1;

	arm7_9->read_xpsr(target, &cpsr, 0);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		ERROR("JTAG error while reading cpsr");
		exit(-1);
	}

	if (((cpsr & 0x1f) == ARMV4_5_MODE_ABT) && (armv4_5->core_mode != ARMV4_5_MODE_ABT))
	{
		WARNING("memory read caused data abort (address: 0x%8.8x, size: 0x%x, count: 0x%x)", address, size, count);

		arm7_9->write_xpsr_im8(target, buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 8) & ~0x20, 0, 0);

		return ERROR_TARGET_DATA_ABORT;
	}
	
	return ERROR_OK;
}

int arm7_9_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	reg_t *dbg_ctrl = &arm7_9->eice_cache->reg_list[EICE_DBG_CTRL];
	
	u32 reg[16];
	int num_accesses = 0;
	int thisrun_accesses;
	int i;
	u32 cpsr;
	int retval;
	int last_reg = 0;

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
	
	/* load the base register with the address of the first word */
	reg[0] = address;
	arm7_9->write_core_regs(target, 0x1, reg);
	
	/* Clear DBGACK, to make sure memory fetches work as expected */
	buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGACK, 1, 0);
	embeddedice_store_reg(dbg_ctrl);
	
	switch (size)
	{
		case 4:
			while (num_accesses < count)
			{
				u32 reg_list;
				thisrun_accesses = ((count - num_accesses) >= 14) ? 14 : (count - num_accesses);
				reg_list = (0xffff >> (15 - thisrun_accesses)) & 0xfffe;
				
				for (i = 1; i <= thisrun_accesses; i++)
				{
					if (i > last_reg)
						last_reg = i;
					reg[i] = target_buffer_get_u32(target, buffer);
					buffer += 4;
				}
				
				arm7_9->write_core_regs(target, reg_list, reg);
				
				arm7_9->store_word_regs(target, reg_list);
				
				/* fast memory writes are only safe when the target is running
				 * from a sufficiently high clock (32 kHz is usually too slow)
				 */
				if (arm7_9->fast_memory_access)
					arm7_9_execute_fast_sys_speed(target);
				else
					arm7_9_execute_sys_speed(target);
				
				num_accesses += thisrun_accesses;
			}	
			break;
		case 2:
			while (num_accesses < count)
			{
				u32 reg_list;
				thisrun_accesses = ((count - num_accesses) >= 14) ? 14 : (count - num_accesses);
				reg_list = (0xffff >> (15 - thisrun_accesses)) & 0xfffe;
				
				for (i = 1; i <= thisrun_accesses; i++)
				{
					if (i > last_reg)
						last_reg = i;
					reg[i] = target_buffer_get_u16(target, buffer) & 0xffff;
					buffer += 2;
				}
				
				arm7_9->write_core_regs(target, reg_list, reg);
				
				for (i = 1; i <= thisrun_accesses; i++)
				{
					arm7_9->store_hword_reg(target, i);
					
					/* fast memory writes are only safe when the target is running
					 * from a sufficiently high clock (32 kHz is usually too slow)
					 */
					if (arm7_9->fast_memory_access)
						arm7_9_execute_fast_sys_speed(target);
					else
						arm7_9_execute_sys_speed(target);
				}
				
				num_accesses += thisrun_accesses;
			}	
			break;
		case 1:
			while (num_accesses < count)
			{
				u32 reg_list;
				thisrun_accesses = ((count - num_accesses) >= 14) ? 14 : (count - num_accesses);
				reg_list = (0xffff >> (15 - thisrun_accesses)) & 0xfffe;
				
				for (i = 1; i <= thisrun_accesses; i++)
				{
					if (i > last_reg)
						last_reg = i;
					reg[i] = *buffer++ & 0xff;
				}
				
				arm7_9->write_core_regs(target, reg_list, reg);
				
				for (i = 1; i <= thisrun_accesses; i++)
				{
					arm7_9->store_byte_reg(target, i);
					/* fast memory writes are only safe when the target is running
					 * from a sufficiently high clock (32 kHz is usually too slow)
					 */
					if (arm7_9->fast_memory_access)
						arm7_9_execute_fast_sys_speed(target);
					else
						arm7_9_execute_sys_speed(target);
				}
				
				num_accesses += thisrun_accesses;
			}	
			break;
		default:
			ERROR("BUG: we shouldn't get here");
			exit(-1);
			break;
	}
	
	/* Re-Set DBGACK */
	buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGACK, 1, 1);
	embeddedice_store_reg(dbg_ctrl);
	
	for (i=0; i<=last_reg; i++)
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).dirty = 1;

	arm7_9->read_xpsr(target, &cpsr, 0);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		ERROR("JTAG error while reading cpsr");
		exit(-1);
	}

	if (((cpsr & 0x1f) == ARMV4_5_MODE_ABT) && (armv4_5->core_mode != ARMV4_5_MODE_ABT))
	{
		WARNING("memory write caused data abort (address: 0x%8.8x, size: 0x%x, count: 0x%x)", address, size, count);

		arm7_9->write_xpsr_im8(target, buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 8) & ~0x20, 0, 0);

		return ERROR_TARGET_DATA_ABORT;
	}
	
	return ERROR_OK;
}

int arm7_9_bulk_write_memory(target_t *target, u32 address, u32 count, u8 *buffer)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	enum armv4_5_state core_state = armv4_5->core_state;
	u32 r0 = buf_get_u32(armv4_5->core_cache->reg_list[0].value, 0, 32);
	u32 r1 = buf_get_u32(armv4_5->core_cache->reg_list[1].value, 0, 32);
	u32 pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);
	int i;
	
	u32 dcc_code[] = 
	{
		/* MRC      TST         BNE         MRC         STR         B */
		0xee101e10, 0xe3110001, 0x0afffffc, 0xee111e10, 0xe4801004, 0xeafffff9
	};
	
	if (!arm7_9->dcc_downloads)
		return target->type->write_memory(target, address, 4, count, buffer);

	/* regrab previously allocated working_area, or allocate a new one */
	if (!arm7_9->dcc_working_area)
	{
		u8 dcc_code_buf[6 * 4];
		
		/* make sure we have a working area */
		if (target_alloc_working_area(target, 24, &arm7_9->dcc_working_area) != ERROR_OK)
		{
			INFO("no working area available, falling back to memory writes");
			return target->type->write_memory(target, address, 4, count, buffer);
		}
		
		/* copy target instructions to target endianness */
		for (i = 0; i < 6; i++)
		{
			target_buffer_set_u32(target, dcc_code_buf + i*4, dcc_code[i]);
		}
		
		/* write DCC code to working area */
		target->type->write_memory(target, arm7_9->dcc_working_area->address, 4, 6, dcc_code_buf);
	}
	
	buf_set_u32(armv4_5->core_cache->reg_list[0].value, 0, 32, address);
	armv4_5->core_cache->reg_list[0].valid = 1;
	armv4_5->core_cache->reg_list[0].dirty = 1;
	armv4_5->core_state = ARMV4_5_STATE_ARM;

	arm7_9_resume(target, 0, arm7_9->dcc_working_area->address, 1, 1);
	
	for (i = 0; i < count; i++)
	{
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_COMMS_DATA], target_buffer_get_u32(target, buffer));
		buffer += 4;
	}
	
	target->type->halt(target);
	
	while (target->state != TARGET_HALTED)
		target->type->poll(target);
	
	/* restore target state */
	buf_set_u32(armv4_5->core_cache->reg_list[0].value, 0, 32, r0);
	armv4_5->core_cache->reg_list[0].valid = 1;
	armv4_5->core_cache->reg_list[0].dirty = 1;
	buf_set_u32(armv4_5->core_cache->reg_list[1].value, 0, 32, r1);
	armv4_5->core_cache->reg_list[1].valid = 1;
	armv4_5->core_cache->reg_list[1].dirty = 1;
	buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, pc);
	armv4_5->core_cache->reg_list[15].valid = 1;
	armv4_5->core_cache->reg_list[15].dirty = 1;
	armv4_5->core_state = core_state;
	
	return ERROR_OK;
}

int arm7_9_checksum_memory(struct target_s *target, u32 address, u32 count, u32* checksum)
{
	working_area_t *crc_algorithm;
	armv4_5_algorithm_t armv4_5_info;
	reg_param_t reg_params[2];
	int retval;
	
	u32 arm7_9_crc_code[] = {
		0xE1A02000,				/* mov		r2, r0 */
		0xE3E00000,				/* mov		r0, #0xffffffff */
		0xE1A03001,				/* mov		r3, r1 */
		0xE3A04000,				/* mov		r4, #0 */
		0xEA00000B,				/* b		ncomp */
								/* nbyte: */
		0xE7D21004,				/* ldrb	r1, [r2, r4] */
		0xE59F7030,				/* ldr		r7, CRC32XOR */
		0xE0200C01,				/* eor		r0, r0, r1, asl 24 */
		0xE3A05000,				/* mov		r5, #0 */
								/* loop: */
		0xE3500000,				/* cmp		r0, #0 */
		0xE1A06080,				/* mov		r6, r0, asl #1 */
		0xE2855001,				/* add		r5, r5, #1 */
		0xE1A00006,				/* mov		r0, r6 */
		0xB0260007,				/* eorlt	r0, r6, r7 */
		0xE3550008,				/* cmp		r5, #8 */
		0x1AFFFFF8,				/* bne		loop */
		0xE2844001,				/* add		r4, r4, #1 */
								/* ncomp: */
		0xE1540003,				/* cmp		r4, r3 */
		0x1AFFFFF1,				/* bne		nbyte */
								/* end: */
		0xEAFFFFFE,				/* b		end */
		0x04C11DB7				/* CRC32XOR:	.word 0x04C11DB7 */
	};
	
	int i;
	
	if (target_alloc_working_area(target, sizeof(arm7_9_crc_code), &crc_algorithm) != ERROR_OK)
	{
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	
	/* convert flash writing code into a buffer in target endianness */
	for (i = 0; i < (sizeof(arm7_9_crc_code)/sizeof(u32)); i++)
		target_write_u32(target, crc_algorithm->address + i*sizeof(u32), arm7_9_crc_code[i]);
	
	armv4_5_info.common_magic = ARMV4_5_COMMON_MAGIC;
	armv4_5_info.core_mode = ARMV4_5_MODE_SVC;
	armv4_5_info.core_state = ARMV4_5_STATE_ARM;
	
	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	
	buf_set_u32(reg_params[0].value, 0, 32, address);
	buf_set_u32(reg_params[1].value, 0, 32, count);
		
	if ((retval = target->type->run_algorithm(target, 0, NULL, 2, reg_params,
		crc_algorithm->address, crc_algorithm->address + (sizeof(arm7_9_crc_code) - 8), 20000, &armv4_5_info)) != ERROR_OK)
	{
		ERROR("error executing arm7_9 crc algorithm");
		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		target_free_working_area(target, crc_algorithm);
		return retval;
	}
	
	*checksum = buf_get_u32(reg_params[0].value, 0, 32);
	
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	
	target_free_working_area(target, crc_algorithm);
	
	return ERROR_OK;
}

int arm7_9_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *arm7_9_cmd;
	
	arm7_9_cmd = register_command(cmd_ctx, NULL, "arm7_9", NULL, COMMAND_ANY, "arm7/9 specific commands");

	register_command(cmd_ctx, arm7_9_cmd, "write_xpsr", handle_arm7_9_write_xpsr_command, COMMAND_EXEC, "write program status register <value> <not cpsr|spsr>");
	register_command(cmd_ctx, arm7_9_cmd, "write_xpsr_im8", handle_arm7_9_write_xpsr_im8_command, COMMAND_EXEC, "write program status register <8bit immediate> <rotate> <not cpsr|spsr>");
	
	register_command(cmd_ctx, arm7_9_cmd, "write_core_reg", handle_arm7_9_write_core_reg_command, COMMAND_EXEC, "write core register <num> <mode> <value>");	
	
	register_command(cmd_ctx, arm7_9_cmd, "sw_bkpts", handle_arm7_9_sw_bkpts_command, COMMAND_EXEC, "support for software breakpoints <enable|disable>");
	register_command(cmd_ctx, arm7_9_cmd, "force_hw_bkpts", handle_arm7_9_force_hw_bkpts_command, COMMAND_EXEC, "use hardware breakpoints for all breakpoints (disables sw breakpoint support) <enable|disable>");
	register_command(cmd_ctx, arm7_9_cmd, "dbgrq", handle_arm7_9_dbgrq_command,
		COMMAND_ANY, "use EmbeddedICE dbgrq instead of breakpoint for target halt requests <enable|disable>");
	register_command(cmd_ctx, arm7_9_cmd, "fast_writes", handle_arm7_9_fast_memory_access_command,
		 COMMAND_ANY, "(deprecated, see: arm7_9 fast_memory_access)");
	register_command(cmd_ctx, arm7_9_cmd, "fast_memory_access", handle_arm7_9_fast_memory_access_command,
		 COMMAND_ANY, "use fast memory accesses instead of slower but potentially unsafe slow accesses <enable|disable>");
	register_command(cmd_ctx, arm7_9_cmd, "dcc_downloads", handle_arm7_9_dcc_downloads_command,
		COMMAND_ANY, "use DCC downloads for larger memory writes <enable|disable>");

	armv4_5_register_commands(cmd_ctx);
	
	etm_register_commands(cmd_ctx);
	
	return ERROR_OK;
}

int handle_arm7_9_write_xpsr_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u32 value;
	int spsr;
	int retval;
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "can't write registers while running");
		return ERROR_OK;
	}
	
	if (argc < 2)
	{
		command_print(cmd_ctx, "usage: write_xpsr <value> <not cpsr|spsr>");
		return ERROR_OK;
	}
	
	value = strtoul(args[0], NULL, 0);
	spsr = strtol(args[1], NULL, 0);
	
	/* if we're writing the CPSR, mask the T bit */
	if (!spsr)
		value &= ~0x20;
	
	arm7_9->write_xpsr(target, value, spsr);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		ERROR("JTAG error while writing to xpsr");
		exit(-1);
	}
	
	return ERROR_OK;
}

int handle_arm7_9_write_xpsr_im8_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u32 value;
	int rotate;
	int spsr;
	int retval;
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "can't write registers while running");
		return ERROR_OK;
	}
	
	if (argc < 3)
	{
		command_print(cmd_ctx, "usage: write_xpsr_im8 <im8> <rotate> <not cpsr|spsr>");
		return ERROR_OK;
	}
	
	value = strtoul(args[0], NULL, 0);
	rotate = strtol(args[1], NULL, 0);
	spsr = strtol(args[2], NULL, 0);
		
	arm7_9->write_xpsr_im8(target, value, rotate, spsr);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		ERROR("JTAG error while writing 8-bit immediate to xpsr");
		exit(-1);
	}
	
	return ERROR_OK;
}

int handle_arm7_9_write_core_reg_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u32 value;
	u32 mode;
	int num;
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
		
	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "can't write registers while running");
		return ERROR_OK;
	}
	
	if (argc < 3)
	{
		command_print(cmd_ctx, "usage: write_core_reg <num> <mode> <value>");
		return ERROR_OK;
	}
	
	num = strtol(args[0], NULL, 0);
	mode = strtoul(args[1], NULL, 0);
	value = strtoul(args[2], NULL, 0);
	
	arm7_9_write_core_reg(target, num, mode, value);
	
	return ERROR_OK;
}

int handle_arm7_9_sw_bkpts_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	
	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}
	
	if (argc == 0)
	{
		command_print(cmd_ctx, "software breakpoints %s", (arm7_9->sw_bkpts_enabled) ? "enabled" : "disabled");
		return ERROR_OK;
	}
	
	if (strcmp("enable", args[0]) == 0)
	{
		if (arm7_9->sw_bkpts_use_wp)
		{
			arm7_9_enable_sw_bkpts(target);
		}
		else
		{
			arm7_9->sw_bkpts_enabled = 1;
		}
	}
	else if (strcmp("disable", args[0]) == 0)
	{
		if (arm7_9->sw_bkpts_use_wp)
		{
			arm7_9_disable_sw_bkpts(target);
		}
		else
		{
			arm7_9->sw_bkpts_enabled = 0;
		}
	}
	else
	{
		command_print(cmd_ctx, "usage: arm7_9 sw_bkpts <enable|disable>");
	}
	
	command_print(cmd_ctx, "software breakpoints %s", (arm7_9->sw_bkpts_enabled) ? "enabled" : "disabled");
	
	return ERROR_OK;
}

int handle_arm7_9_force_hw_bkpts_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	
	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}
	
	if ((argc >= 1) && (strcmp("enable", args[0]) == 0))
	{
		arm7_9->force_hw_bkpts = 1;
		if (arm7_9->sw_bkpts_use_wp)
		{
			arm7_9_disable_sw_bkpts(target);
		}
	}
	else if ((argc >= 1) && (strcmp("disable", args[0]) == 0))
	{
		arm7_9->force_hw_bkpts = 0;
	}
	else
	{
		command_print(cmd_ctx, "usage: arm7_9 force_hw_bkpts <enable|disable>");
	}
		
	command_print(cmd_ctx, "force hardware breakpoints %s", (arm7_9->force_hw_bkpts) ? "enabled" : "disabled");

	return ERROR_OK;
}

int handle_arm7_9_dbgrq_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	
	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}
	
	if (argc > 0)
	{
		if (strcmp("enable", args[0]) == 0)
		{
			arm7_9->use_dbgrq = 1;
		}
		else if (strcmp("disable", args[0]) == 0)
		{
			arm7_9->use_dbgrq = 0;
		}
		else
		{
			command_print(cmd_ctx, "usage: arm7_9 dbgrq <enable|disable>");
		}
	}
		
	command_print(cmd_ctx, "use of EmbeddedICE dbgrq instead of breakpoint for target halt %s", (arm7_9->use_dbgrq) ? "enabled" : "disabled");

	return ERROR_OK;
}

int handle_arm7_9_fast_memory_access_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	
	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}
	
	if (argc > 0)
	{
		if (strcmp("enable", args[0]) == 0)
		{
			arm7_9->fast_memory_access = 1;
		}
		else if (strcmp("disable", args[0]) == 0)
		{
			arm7_9->fast_memory_access = 0;
		}
		else
		{
			command_print(cmd_ctx, "usage: arm7_9 fast_memory_access <enable|disable>");
		}
	}
		
	command_print(cmd_ctx, "fast memory access is %s", (arm7_9->fast_memory_access) ? "enabled" : "disabled");

	return ERROR_OK;
}

int handle_arm7_9_dcc_downloads_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	
	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}
	
	if (argc > 0)
	{
		if (strcmp("enable", args[0]) == 0)
		{
			arm7_9->dcc_downloads = 1;
		}
		else if (strcmp("disable", args[0]) == 0)
		{
			arm7_9->dcc_downloads = 0;
		}
		else
		{
			command_print(cmd_ctx, "usage: arm7_9 dcc_downloads <enable|disable>");
		}
	}
		
	command_print(cmd_ctx, "dcc downloads are %s", (arm7_9->dcc_downloads) ? "enabled" : "disabled");

	return ERROR_OK;
}

int arm7_9_init_arch_info(target_t *target, arm7_9_common_t *arm7_9)
{
	armv4_5_common_t *armv4_5 = &arm7_9->armv4_5_common;
	
	arm7_9->common_magic = ARM7_9_COMMON_MAGIC;
	
	arm_jtag_setup_connection(&arm7_9->jtag_info);
	arm7_9->wp_available = 2;
	arm7_9->wp0_used = 0;
	arm7_9->wp1_used = 0;
	arm7_9->force_hw_bkpts = 0;
	arm7_9->use_dbgrq = 0;
	
	arm7_9->etm_ctx = NULL;
	arm7_9->has_single_step = 0;
	arm7_9->has_monitor_mode = 0;
	arm7_9->has_vector_catch = 0;
	
	arm7_9->reinit_embeddedice = 0;
	
	arm7_9->debug_entry_from_reset = 0;
	
	arm7_9->dcc_working_area = NULL;
	
	arm7_9->fast_memory_access = 0;
	arm7_9->dcc_downloads = 0;

	jtag_register_event_callback(arm7_9_jtag_callback, target);

	armv4_5->arch_info = arm7_9;
	armv4_5->read_core_reg = arm7_9_read_core_reg;
	armv4_5->write_core_reg = arm7_9_write_core_reg;
	armv4_5->full_context = arm7_9_full_context;
	
	armv4_5_init_arch_info(target, armv4_5);
	
	target_register_timer_callback(arm7_9_handle_target_request, 1, 1, target);
	
	return ERROR_OK;
}
