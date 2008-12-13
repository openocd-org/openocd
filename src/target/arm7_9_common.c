/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by Hongtao Zheng                                   *
 *   hontor@126.com                                                        *
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
#include "time_support.h"
#include "arm_simulator.h"

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
int handle_arm7_9_dbgrq_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_arm7_9_fast_memory_access_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_arm7_9_dcc_downloads_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_arm7_9_etm_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

static int arm7_9_clear_watchpoints(arm7_9_common_t *arm7_9)
{
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], 0x0);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE], 0x0);
	arm7_9->sw_breakpoints_added = 0;
	arm7_9->wp0_used = 0;
	arm7_9->wp1_used = arm7_9->wp1_used_default;
	arm7_9->wp_available = arm7_9->wp_available_max;

	return jtag_execute_queue();
}

/* set up embedded ice registers */
static int arm7_9_set_software_breakpoints(arm7_9_common_t *arm7_9)
{
	if (arm7_9->sw_breakpoints_added)
	{
		return ERROR_OK;
	}
	if (arm7_9->wp_available < 1)
	{
		LOG_WARNING("can't enable sw breakpoints with no watchpoint unit available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	arm7_9->wp_available--;

	/* pick a breakpoint unit */
	if (!arm7_9->wp0_used)
	{
		arm7_9->sw_breakpoints_added=1;
		arm7_9->wp0_used = 3;
	} else if (!arm7_9->wp1_used)
	{
		arm7_9->sw_breakpoints_added=2;
		arm7_9->wp1_used = 3;
	}
	else
	{
		LOG_ERROR("BUG: both watchpoints used, but wp_available >= 1");
		return ERROR_FAIL;
	}

	if (arm7_9->sw_breakpoints_added==1)
	{
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_VALUE], arm7_9->arm_bkpt);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK], 0x0);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK], 0xffffffffu);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK], ~EICE_W_CTRL_nOPC & 0xff);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], EICE_W_CTRL_ENABLE);
	}
	else if (arm7_9->sw_breakpoints_added==2)
	{
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_DATA_VALUE], arm7_9->arm_bkpt);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_DATA_MASK], 0x0);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_MASK], 0xffffffffu);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_MASK], ~EICE_W_CTRL_nOPC & 0xff);
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE], EICE_W_CTRL_ENABLE);
	}
	else
	{
		LOG_ERROR("BUG: both watchpoints used, but wp_available >= 1");
		return ERROR_FAIL;
	}

	return jtag_execute_queue();
}

/* set things up after a reset / on startup */
int arm7_9_setup(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	return arm7_9_clear_watchpoints(arm7_9);
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

/* we set up the breakpoint even if it is already set. Some action, e.g. reset
 * might have erased the values in embedded ice
 */
int arm7_9_set_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	int retval=ERROR_OK;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		/* either an ARM (4 byte) or Thumb (2 byte) breakpoint */
		u32 mask = (breakpoint->length == 4) ? 0x3u : 0x1u;
		if (breakpoint->set==1)
		{
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_VALUE], breakpoint->address);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK], mask);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK], 0xffffffffu);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK], ~EICE_W_CTRL_nOPC & 0xff);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], EICE_W_CTRL_ENABLE);
		}
		else if (breakpoint->set==2)
		{
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_VALUE], breakpoint->address);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_MASK], mask);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_DATA_MASK], 0xffffffffu);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_MASK], ~EICE_W_CTRL_nOPC & 0xff);
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE], EICE_W_CTRL_ENABLE);
		}
		else
		{
			LOG_ERROR("BUG: no hardware comparator available");
			return ERROR_OK;
		}

		retval=jtag_execute_queue();
	}
	else if (breakpoint->type == BKPT_SOFT)
	{
		if ((retval=arm7_9_set_software_breakpoints(arm7_9))!=ERROR_OK)
			return retval;

		/* did we already set this breakpoint? */
		if (breakpoint->set)
			return ERROR_OK;

		if (breakpoint->length == 4)
		{
			u32 verify = 0xffffffff;
			/* keep the original instruction in target endianness */
			if ((retval = target->type->read_memory(target, breakpoint->address, 4, 1, breakpoint->orig_instr)) != ERROR_OK)
			{
				return retval;
			}
			/* write the breakpoint instruction in target endianness (arm7_9->arm_bkpt is host endian) */
			if ((retval = target_write_u32(target, breakpoint->address, arm7_9->arm_bkpt)) != ERROR_OK)
			{
				return retval;
			}

			if ((retval = target_read_u32(target, breakpoint->address, &verify)) != ERROR_OK)
			{
				return retval;
			}
			if (verify != arm7_9->arm_bkpt)
			{
				LOG_ERROR("Unable to set 32 bit software breakpoint at address %08x - check that memory is read/writable", breakpoint->address);
				return ERROR_OK;
			}
		}
		else
		{
			u16 verify = 0xffff;
			/* keep the original instruction in target endianness */
			if ((retval = target->type->read_memory(target, breakpoint->address, 2, 1, breakpoint->orig_instr)) != ERROR_OK)
			{
				return retval;
			}
			/* write the breakpoint instruction in target endianness (arm7_9->thumb_bkpt is host endian) */
			if ((retval = target_write_u16(target, breakpoint->address, arm7_9->thumb_bkpt)) != ERROR_OK)
			{
				return retval;
			}

			if ((retval = target_read_u16(target, breakpoint->address, &verify)) != ERROR_OK)
			{
				return retval;
			}
			if (verify != arm7_9->thumb_bkpt)
			{
				LOG_ERROR("Unable to set thumb software breakpoint at address %08x - check that memory is read/writable", breakpoint->address);
				return ERROR_OK;
			}
		}
		breakpoint->set = 1;
	}

	return retval;
}

int arm7_9_unset_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	int retval = ERROR_OK;

	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	if (!breakpoint->set)
	{
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		if (breakpoint->set == 1)
		{
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], 0x0);
			arm7_9->wp0_used = 0;
		}
		else if (breakpoint->set == 2)
		{
			embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE], 0x0);
			arm7_9->wp1_used = 0;
		}
		retval = jtag_execute_queue();
		breakpoint->set = 0;
	}
	else
	{
		/* restore original instruction (kept in target endianness) */
		if (breakpoint->length == 4)
		{
			u32 current_instr;
			/* check that user program as not modified breakpoint instruction */
			if ((retval = target->type->read_memory(target, breakpoint->address, 4, 1, (u8*)&current_instr)) != ERROR_OK)
			{
				return retval;
			}
			if (current_instr==arm7_9->arm_bkpt)
				if ((retval = target->type->write_memory(target, breakpoint->address, 4, 1, breakpoint->orig_instr)) != ERROR_OK)
				{
					return retval;
				}
		}
		else
		{
			u16 current_instr;
			/* check that user program as not modified breakpoint instruction */
			if ((retval = target->type->read_memory(target, breakpoint->address, 2, 1, (u8*)&current_instr)) != ERROR_OK)
			{
				return retval;
			}
			if (current_instr==arm7_9->thumb_bkpt)
				if ((retval = target->type->write_memory(target, breakpoint->address, 2, 1, breakpoint->orig_instr)) != ERROR_OK)
				{
					return retval;
				}
		}
		breakpoint->set = 0;
	}

	return retval;
}

int arm7_9_add_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (arm7_9->breakpoint_count==0)
	{
		/* make sure we don't have any dangling breakpoints. This is vital upon
		 * GDB connect/disconnect
		 */
		arm7_9_clear_watchpoints(arm7_9);
	}

	if ((breakpoint->type == BKPT_HARD) && (arm7_9->wp_available < 1))
	{
		LOG_INFO("no watchpoint unit available for hardware breakpoint");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if ((breakpoint->length != 2) && (breakpoint->length != 4))
	{
		LOG_INFO("only breakpoints of two (Thumb) or four (ARM) bytes length supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		arm7_9->wp_available--;

		if (!arm7_9->wp0_used)
		{
			arm7_9->wp0_used = 1;
			breakpoint->set = 1;
		}
		else if (!arm7_9->wp1_used)
		{
			arm7_9->wp1_used = 1;
			breakpoint->set = 2;
		}
		else
		{
			LOG_ERROR("BUG: no hardware comparator available");
		}
	}

	arm7_9->breakpoint_count++;

	return arm7_9_set_breakpoint(target, breakpoint);
}

int arm7_9_remove_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	int retval = ERROR_OK;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	if((retval = arm7_9_unset_breakpoint(target, breakpoint)) != ERROR_OK)
	{
		return retval;
	}

	if (breakpoint->type == BKPT_HARD)
		arm7_9->wp_available++;

	arm7_9->breakpoint_count--;
	if (arm7_9->breakpoint_count==0)
	{
		/* make sure we don't have any dangling breakpoints */
		if((retval = arm7_9_clear_watchpoints(arm7_9)) != ERROR_OK)
		{
			return retval;
		}
	}

	return ERROR_OK;
}

int arm7_9_set_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	int retval = ERROR_OK;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	int rw_mask = 1;
	u32 mask;

	mask = watchpoint->length - 1;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
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

		if((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}
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

		if((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}
		watchpoint->set = 2;
		arm7_9->wp1_used = 2;
	}
	else
	{
		LOG_ERROR("BUG: no hardware comparator available");
		return ERROR_OK;
	}

	return ERROR_OK;
}

int arm7_9_unset_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	int retval = ERROR_OK;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!watchpoint->set)
	{
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (watchpoint->set == 1)
	{
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], 0x0);
		if((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}
		arm7_9->wp0_used = 0;
	}
	else if (watchpoint->set == 2)
	{
		embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE], 0x0);
		if((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}
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
		LOG_WARNING("target not halted");
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
	int retval = ERROR_OK;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	if (watchpoint->set)
	{
		if((retval = arm7_9_unset_watchpoint(target, watchpoint)) != ERROR_OK)
		{
			return retval;
		}
	}

	arm7_9->wp_available++;

	return ERROR_OK;
}

int arm7_9_execute_sys_speed(struct target_s *target)
{
	int retval;

	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	reg_t *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];

	/* set RESTART instruction */
	jtag_add_end_state(TAP_IDLE);
	if (arm7_9->need_bypass_before_restart) {
		arm7_9->need_bypass_before_restart = 0;
		arm_jtag_set_instr(jtag_info, 0xf, NULL);
	}
	arm_jtag_set_instr(jtag_info, 0x4, NULL);

	long long then=timeval_ms();
	int timeout;
	while (!(timeout=((timeval_ms()-then)>1000)))
	{
		/* read debug status register */
		embeddedice_read_reg(dbg_stat);
		if ((retval = jtag_execute_queue()) != ERROR_OK)
			return retval;
		if ((buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_DBGACK, 1))
				   && (buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_SYSCOMP, 1)))
			break;
		if (debug_level>=3)
		{
			alive_sleep(100);
		} else
		{
			keep_alive();
		}
	}
	if (timeout)
	{
		LOG_ERROR("timeout waiting for SYSCOMP & DBGACK, last DBG_STATUS: %x", buf_get_u32(dbg_stat->value, 0, dbg_stat->size));
		return ERROR_TARGET_TIMEOUT;
	}

	return ERROR_OK;
}

int arm7_9_execute_fast_sys_speed(struct target_s *target)
{
	static int set=0;
	static u8 check_value[4], check_mask[4];

	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	reg_t *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];

	/* set RESTART instruction */
	jtag_add_end_state(TAP_IDLE);
	if (arm7_9->need_bypass_before_restart) {
		arm7_9->need_bypass_before_restart = 0;
		arm_jtag_set_instr(jtag_info, 0xf, NULL);
	}
	arm_jtag_set_instr(jtag_info, 0x4, NULL);

	if (!set)
	{
		/* check for DBGACK and SYSCOMP set (others don't care) */

		/* NB! These are constants that must be available until after next jtag_execute() and
		 * we evaluate the values upon first execution in lieu of setting up these constants
		 * during early setup.
		 * */
		buf_set_u32(check_value, 0, 32, 0x9);
		buf_set_u32(check_mask, 0, 32, 0x9);
		set=1;
	}

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
	int i, retval = ERROR_OK;

	data = malloc(size * (sizeof(u32)));

	retval = embeddedice_receive(jtag_info, data, size);

	for (i = 0; i < size; i++)
	{
		h_u32_to_le(buffer + (i * 4), data[i]);
	}

	free(data);

	return retval;
}

int arm7_9_handle_target_request(void *priv)
{
	int retval = ERROR_OK;
	target_t *target = priv;
	if (!target->type->examined)
		return ERROR_OK;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	reg_t *dcc_control = &arm7_9->eice_cache->reg_list[EICE_COMMS_CTRL];

	if (!target->dbg_msg_enabled)
		return ERROR_OK;

	if (target->state == TARGET_RUNNING)
	{
		/* read DCC control register */
		embeddedice_read_reg(dcc_control);
		if ((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}

		/* check W bit */
		if (buf_get_u32(dcc_control->value, 1, 1) == 1)
		{
			u32 request;

			if ((retval = embeddedice_receive(jtag_info, &request, 1)) != ERROR_OK)
			{
				return retval;
			}
			if ((retval = target_request(target, request)) != ERROR_OK)
			{
				return retval;
			}
		}
	}

	return ERROR_OK;
}

int arm7_9_poll(target_t *target)
{
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	reg_t *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];

	/* read debug status register */
	embeddedice_read_reg(dbg_stat);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return retval;
	}

	if (buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_DBGACK, 1))
	{
/*		LOG_DEBUG("DBGACK set, dbg_state->value: 0x%x", buf_get_u32(dbg_stat->value, 0, 32));*/
		if (target->state == TARGET_UNKNOWN)
		{
			target->state = TARGET_RUNNING;
			LOG_WARNING("DBGACK set while target was in unknown state. Reset or initialize target.");
		}
		if ((target->state == TARGET_RUNNING) || (target->state == TARGET_RESET))
		{
			int check_pc=0;
			if (target->state == TARGET_RESET)
			{
				if (target->reset_halt)
				{
					if ((jtag_reset_config & RESET_SRST_PULLS_TRST)==0)
					{
						check_pc = 1;
					}
				}
			}

			target->state = TARGET_HALTED;

			if ((retval = arm7_9_debug_entry(target)) != ERROR_OK)
				return retval;

			if (check_pc)
			{
				reg_t *reg = register_get_by_name(target->reg_cache, "pc", 1);
				u32 t=*((u32 *)reg->value);
				if (t!=0)
				{
					LOG_ERROR("PC was not 0. Does this target need srst_pulls_trst?");
				}
			}

			if ((retval = target_call_event_callbacks(target, TARGET_EVENT_HALTED)) != ERROR_OK)
			{
				return retval;
			}
		}
		if (target->state == TARGET_DEBUG_RUNNING)
		{
			target->state = TARGET_HALTED;
			if ((retval = arm7_9_debug_entry(target)) != ERROR_OK)
				return retval;

			if ((retval = target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED)) != ERROR_OK)
			{
				return retval;
			}
		}
		if (target->state != TARGET_HALTED)
		{
			LOG_WARNING("DBGACK set, but the target did not end up in the halted stated %d", target->state);
		}
	}
	else
	{
		if (target->state != TARGET_DEBUG_RUNNING)
			target->state = TARGET_RUNNING;
	}

	return ERROR_OK;
}

/*
  Some -S targets (ARM966E-S in the STR912 isn't affected, ARM926EJ-S
  in the LPC3180 and AT91SAM9260 is affected) completely stop the JTAG clock
  while the core is held in reset(SRST). It isn't possible to program the halt
  condition once reset was asserted, hence a hook that allows the target to set
  up its reset-halt condition prior to asserting reset.
*/

int arm7_9_assert_reset(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	LOG_DEBUG("target->state: %s",
		  Jim_Nvp_value2name_simple( nvp_target_state,target->state)->name);

	if (!(jtag_reset_config & RESET_HAS_SRST))
	{
		LOG_ERROR("Can't assert SRST");
		return ERROR_FAIL;
	}

	if (target->reset_halt)
	{
		/*
		 * Some targets do not support communication while SRST is asserted. We need to
		 * set up the reset vector catch here.
		 *
		 * If TRST is asserted, then these settings will be reset anyway, so setting them
		 * here is harmless.
		 */
		if (arm7_9->has_vector_catch)
		{
			/* program vector catch register to catch reset vector */
			embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_VEC_CATCH], 0x1);
		}
		else
		{
			/* program watchpoint unit to match on reset vector address */
			embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_VALUE], 0x0);
			embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK], 0x3);
			embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK], 0xffffffff);
			embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], EICE_W_CTRL_ENABLE);
			embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK], ~EICE_W_CTRL_nOPC & 0xff);
		}
	}

	/* here we should issue a srst only, but we may have to assert trst as well */
	if (jtag_reset_config & RESET_SRST_PULLS_TRST)
	{
		jtag_add_reset(1, 1);
	} else
	{
		jtag_add_reset(0, 1);
	}

	target->state = TARGET_RESET;
	jtag_add_sleep(50000);

	armv4_5_invalidate_core_regs(target);

	if ((target->reset_halt)&&((jtag_reset_config & RESET_SRST_PULLS_TRST)==0))
	{
		/* debug entry was already prepared in arm7_9_assert_reset() */
		target->debug_reason = DBG_REASON_DBGRQ;
	}

	return ERROR_OK;
}

int arm7_9_deassert_reset(target_t *target)
{
	int retval=ERROR_OK;
	LOG_DEBUG("target->state: %s",
		Jim_Nvp_value2name_simple( nvp_target_state,target->state)->name);

	/* deassert reset lines */
	jtag_add_reset(0, 0);

	if (target->reset_halt&&(jtag_reset_config & RESET_SRST_PULLS_TRST)!=0)
	{
		LOG_WARNING("srst pulls trst - can not reset into halted mode. Issuing halt after reset.");
		/* set up embedded ice registers again */
		if ((retval=target->type->examine(target))!=ERROR_OK)
			return retval;

		if ((retval=target_poll(target))!=ERROR_OK)
		{
			return retval;
		}

		if ((retval=target_halt(target))!=ERROR_OK)
		{
			return retval;
		}

	}
	return retval;
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
				if (arm7_9->debug_entry_from_reset)
				{
					embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_VALUE]);
				}
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
	int retval;

	if ((retval=target_halt(target))!=ERROR_OK)
		return retval;

	long long then=timeval_ms();
	int timeout;
	while (!(timeout=((timeval_ms()-then)>1000)))
	{
		if (buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_DBGACK, 1) != 0)
			break;
		embeddedice_read_reg(dbg_stat);
		if ((retval=jtag_execute_queue())!=ERROR_OK)
			return retval;
		if (debug_level>=3)
		{
			alive_sleep(100);
		} else
		{
			keep_alive();
		}
	}
	if (timeout)
	{
		LOG_ERROR("Failed to halt CPU after 1 sec");
		return ERROR_TARGET_TIMEOUT;
	}
	target->state = TARGET_HALTED;

	/* program EmbeddedICE Debug Control Register to assert DBGACK and INTDIS
	 * ensure that DBGRQ is cleared
	 */
	buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGACK, 1, 1);
	buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGRQ, 1, 0);
	buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_INTDIS, 1, 1);
	embeddedice_store_reg(dbg_ctrl);

	if ((retval = arm7_9_clear_halt(target)) != ERROR_OK)
	{
		return retval;
	}

	/* if the target is in Thumb state, change to ARM state */
	if (buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_ITBIT, 1))
	{
		u32 r0_thumb, pc_thumb;
		LOG_DEBUG("target entered debug from Thumb state, changing to ARM");
		/* Entered debug from Thumb mode */
		armv4_5->core_state = ARMV4_5_STATE_THUMB;
		arm7_9->change_to_arm(target, &r0_thumb, &pc_thumb);
	}

	/* all register content is now invalid */
	if ((retval = armv4_5_invalidate_core_regs(target)) != ERROR_OK)
	{
		return retval;
	}

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

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

	/* reset registers */
	for (i = 0; i <= 14; i++)
	{
		buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).value, 0, 32, 0xffffffff);
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).dirty = 1;
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).valid = 1;
	}

	if ((retval = target_call_event_callbacks(target, TARGET_EVENT_HALTED)) != ERROR_OK)
	{
		return retval;
	}

	return ERROR_OK;
}

int arm7_9_halt(target_t *target)
{
	if (target->state==TARGET_RESET)
	{
		LOG_ERROR("BUG: arm7/9 does not support halt during reset. This is handled in arm7_9_assert_reset()");
		return ERROR_OK;
	}

	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	reg_t *dbg_ctrl = &arm7_9->eice_cache->reg_list[EICE_DBG_CTRL];

	LOG_DEBUG("target->state: %s",
		  Jim_Nvp_value2name_simple( nvp_target_state,target->state)->name);

	if (target->state == TARGET_HALTED)
	{
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
	{
		LOG_WARNING("target was in unknown state when halt was requested");
	}

	if (arm7_9->use_dbgrq)
	{
		/* program EmbeddedICE Debug Control Register to assert DBGRQ
		 */
		if (arm7_9->set_special_dbgrq) {
			arm7_9->set_special_dbgrq(target);
		} else {
			buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGRQ, 1, 1);
			embeddedice_store_reg(dbg_ctrl);
		}
	}
	else
	{
		/* program watchpoint unit to match on any address
		 */
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK], 0xffffffff);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK], 0xffffffff);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], EICE_W_CTRL_ENABLE);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK], ~EICE_W_CTRL_nOPC & 0xff);
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
	LOG_DEBUG("-");
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

	if ((retval = arm7_9_clear_halt(target)) != ERROR_OK)
	{
		return retval;
	}

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return retval;
	}

	if ((retval = arm7_9->examine_debug_reason(target)) != ERROR_OK)
		return retval;


	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* if the target is in Thumb state, change to ARM state */
	if (buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_ITBIT, 1))
	{
		LOG_DEBUG("target entered debug from Thumb state");
		/* Entered debug from Thumb mode */
		armv4_5->core_state = ARMV4_5_STATE_THUMB;
		arm7_9->change_to_arm(target, &r0_thumb, &pc_thumb);
		LOG_DEBUG("r0_thumb: 0x%8.8x, pc_thumb: 0x%8.8x", r0_thumb, pc_thumb);
	}
	else
	{
		LOG_DEBUG("target entered debug from ARM state");
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
		LOG_ERROR("cpsr contains invalid mode value - communication failure");
		return ERROR_TARGET_FAILURE;
	}

	LOG_DEBUG("target entered debug state in %s mode", armv4_5_mode_strings[armv4_5_mode_to_number(armv4_5->core_mode)]);

	if (armv4_5->core_state == ARMV4_5_STATE_THUMB)
	{
		LOG_DEBUG("thumb state, applying fixups");
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
		LOG_ERROR("unknown debug reason: %i", target->debug_reason);
	}

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

	for (i=0; i<=15; i++)
	{
		LOG_DEBUG("r%i: 0x%8.8x", i, context[i]);
		buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).value, 0, 32, context[i]);
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).dirty = 0;
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).valid = 1;
	}

	LOG_DEBUG("entered debug state at PC 0x%x", context[15]);

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

	/* exceptions other than USR & SYS have a saved program status register */
	if ((armv4_5->core_mode != ARMV4_5_MODE_USR) && (armv4_5->core_mode != ARMV4_5_MODE_SYS))
	{
		u32 spsr;
		arm7_9->read_xpsr(target, &spsr, 1);
		if ((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}
		buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 16).value, 0, 32, spsr);
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 16).dirty = 0;
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 16).valid = 1;
	}

	/* r0 and r15 (pc) have to be restored later */
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 0).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 0).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 15).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 15).valid;

	if ((retval = jtag_execute_queue()) != ERROR_OK)
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

	LOG_DEBUG("-");

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

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
		return retval;
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

	LOG_DEBUG("-");

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (arm7_9->pre_restore_context)
		arm7_9->pre_restore_context(target);

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

	/* iterate through processor modes (User, FIQ, IRQ, SVC, ABT, UND)
	 * SYS shares registers with User, so we don't touch SYS
	 */
	for (i = 0; i < 6; i++)
	{
		LOG_DEBUG("examining %s mode", armv4_5_mode_strings[i]);
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
					LOG_DEBUG("examining dirty reg: %s", reg->name);
					if ((reg_arch_info->mode != ARMV4_5_MODE_ANY)
						&& (reg_arch_info->mode != current_mode)
						&& !((reg_arch_info->mode == ARMV4_5_MODE_USR) && (armv4_5->core_mode == ARMV4_5_MODE_SYS))
						&& !((reg_arch_info->mode == ARMV4_5_MODE_SYS) && (armv4_5->core_mode == ARMV4_5_MODE_USR)))
					{
						mode_change = 1;
						LOG_DEBUG("require mode change");
					}
				}
				else
				{
					LOG_ERROR("BUG: dirty register '%s', but no valid data", reg->name);
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
					LOG_DEBUG("writing register %i of mode %s with value 0x%8.8x", j, armv4_5_mode_strings[i], regs[j]);
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
				LOG_DEBUG("writing SPSR of mode %i with value 0x%8.8x", i, buf_get_u32(reg->value, 0, 32));
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
		LOG_DEBUG("writing lower 8 bit of cpsr with value 0x%2.2x", tmp_cpsr);
		arm7_9->write_xpsr_im8(target, tmp_cpsr & 0xff, 0, 0);
	}
	else if (armv4_5->core_cache->reg_list[ARMV4_5_CPSR].dirty == 1)
	{
		/* CPSR has been changed, full restore necessary (mask T bit) */
		LOG_DEBUG("writing cpsr with value 0x%8.8x", buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32));
		arm7_9->write_xpsr(target, buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32) & ~0x20, 0);
		armv4_5->core_cache->reg_list[ARMV4_5_CPSR].dirty = 0;
		armv4_5->core_cache->reg_list[ARMV4_5_CPSR].valid = 1;
	}

	/* restore PC */
	LOG_DEBUG("writing PC with value 0x%8.8x", buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));
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
	jtag_add_end_state(TAP_IDLE);
	if (arm7_9->need_bypass_before_restart) {
		arm7_9->need_bypass_before_restart = 0;
		arm_jtag_set_instr(jtag_info, 0xf, NULL);
	}
	arm_jtag_set_instr(jtag_info, 0x4, NULL);

	jtag_add_runtest(1, TAP_IDLE);
	return jtag_execute_queue();
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
		arm7_9_set_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}
}

int arm7_9_resume(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	breakpoint_t *breakpoint = target->breakpoints;
	reg_t *dbg_ctrl = &arm7_9->eice_cache->reg_list[EICE_DBG_CTRL];
	int err, retval = ERROR_OK;

	LOG_DEBUG("-");

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!debug_execution)
	{
		target_free_all_working_areas(target);
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
		buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, address);

	u32 current_pc;
	current_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints)
	{
		if ((breakpoint = breakpoint_find(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32))))
		{
			LOG_DEBUG("unset breakpoint at 0x%8.8x", breakpoint->address);
			if ((retval = arm7_9_unset_breakpoint(target, breakpoint)) != ERROR_OK)
			{
				return retval;
			}

			/* calculate PC of next instruction */
			u32 next_pc;
			if ((retval = arm_simulate_step(target, &next_pc)) != ERROR_OK)
			{
				u32 current_opcode;
				target_read_u32(target, current_pc, &current_opcode);
				LOG_ERROR("BUG: couldn't calculate PC of next instruction, current opcode was 0x%8.8x", current_opcode);
				return retval;
			}

			LOG_DEBUG("enable single-step");
			arm7_9->enable_single_step(target, next_pc);

			target->debug_reason = DBG_REASON_SINGLESTEP;

			if ((retval = arm7_9_restore_context(target)) != ERROR_OK)
			{
				return retval;
			}

			if (armv4_5->core_state == ARMV4_5_STATE_ARM)
				arm7_9->branch_resume(target);
			else if (armv4_5->core_state == ARMV4_5_STATE_THUMB)
			{
				arm7_9->branch_resume_thumb(target);
			}
			else
			{
				LOG_ERROR("unhandled core state");
				return ERROR_FAIL;
			}

			buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGACK, 1, 0);
			embeddedice_write_reg(dbg_ctrl, buf_get_u32(dbg_ctrl->value, 0, dbg_ctrl->size));
			err = arm7_9_execute_sys_speed(target);

			LOG_DEBUG("disable single-step");
			arm7_9->disable_single_step(target);

			if (err != ERROR_OK)
			{
				if ((retval = arm7_9_set_breakpoint(target, breakpoint)) != ERROR_OK)
				{
					return retval;
				}
				target->state = TARGET_UNKNOWN;
				return err;
			}

			arm7_9_debug_entry(target);
			LOG_DEBUG("new PC after step: 0x%8.8x", buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));

			LOG_DEBUG("set breakpoint at 0x%8.8x", breakpoint->address);
			if ((retval = arm7_9_set_breakpoint(target, breakpoint)) != ERROR_OK)
			{
				return retval;
			}
		}
	}

	/* enable any pending breakpoints and watchpoints */
	arm7_9_enable_breakpoints(target);
	arm7_9_enable_watchpoints(target);

	if ((retval = arm7_9_restore_context(target)) != ERROR_OK)
	{
		return retval;
	}

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
		LOG_ERROR("unhandled core state");
		return ERROR_FAIL;
	}

	/* deassert DBGACK and INTDIS */
	buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGACK, 1, 0);
	/* INTDIS only when we really resume, not during debug execution */
	if (!debug_execution)
		buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_INTDIS, 1, 0);
	embeddedice_write_reg(dbg_ctrl, buf_get_u32(dbg_ctrl->value, 0, dbg_ctrl->size));

	if ((retval = arm7_9_restart_core(target)) != ERROR_OK)
	{
		return retval;
	}

	target->debug_reason = DBG_REASON_NOTHALTED;

	if (!debug_execution)
	{
		/* registers are now invalid */
		armv4_5_invalidate_core_regs(target);
		target->state = TARGET_RUNNING;
		if ((retval = target_call_event_callbacks(target, TARGET_EVENT_RESUMED)) != ERROR_OK)
		{
			return retval;
		}
	}
	else
	{
		target->state = TARGET_DEBUG_RUNNING;
		if ((retval = target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED)) != ERROR_OK)
		{
			return retval;
		}
	}

	LOG_DEBUG("target resumed");

	return ERROR_OK;
}

void arm7_9_enable_eice_step(target_t *target, u32 next_pc)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	u32 current_pc;
	current_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);

	if(next_pc != current_pc)
	{
		/* setup an inverse breakpoint on the current PC
		* - comparator 1 matches the current address
		* - rangeout from comparator 1 is connected to comparator 0 rangein
		* - comparator 0 matches any address, as long as rangein is low */
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK], 0xffffffff);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK], 0xffffffff);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], EICE_W_CTRL_ENABLE);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK], ~(EICE_W_CTRL_RANGE|EICE_W_CTRL_nOPC) & 0xff);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_VALUE], current_pc);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_MASK], 0);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_DATA_MASK], 0xffffffff);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE], 0x0);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_MASK], ~EICE_W_CTRL_nOPC & 0xff);
	}
	else
	{
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK], 0xffffffff);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK], 0xffffffff);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], 0x0);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK], 0xff);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_VALUE], next_pc);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_ADDR_MASK], 0);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_DATA_MASK], 0xffffffff);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_VALUE], EICE_W_CTRL_ENABLE);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W1_CONTROL_MASK], ~EICE_W_CTRL_nOPC & 0xff);
	}
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
	int err, retval;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
		buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, address);

	u32 current_pc;
	current_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints)
		if ((breakpoint = breakpoint_find(target, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32))))
			if ((retval = arm7_9_unset_breakpoint(target, breakpoint)) != ERROR_OK)
			{
				return retval;
			}

	target->debug_reason = DBG_REASON_SINGLESTEP;

	/* calculate PC of next instruction */
	u32 next_pc;
	if ((retval = arm_simulate_step(target, &next_pc)) != ERROR_OK)
	{
		u32 current_opcode;
		target_read_u32(target, current_pc, &current_opcode);
		LOG_ERROR("BUG: couldn't calculate PC of next instruction, current opcode was 0x%8.8x", current_opcode);
		return retval;
	}

	if ((retval = arm7_9_restore_context(target)) != ERROR_OK)
	{
		return retval;
	}

	arm7_9->enable_single_step(target, next_pc);

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
		LOG_ERROR("unhandled core state");
		return ERROR_FAIL;
	}

	if ((retval = target_call_event_callbacks(target, TARGET_EVENT_RESUMED)) != ERROR_OK)
	{
		return retval;
	}

	err = arm7_9_execute_sys_speed(target);
	arm7_9->disable_single_step(target);

	/* registers are now invalid */
	armv4_5_invalidate_core_regs(target);

	if (err != ERROR_OK)
	{
		target->state = TARGET_UNKNOWN;
	} else {
		arm7_9_debug_entry(target);
		if ((retval = target_call_event_callbacks(target, TARGET_EVENT_HALTED)) != ERROR_OK)
		{
			return retval;
		}
		LOG_DEBUG("target stepped");
	}

	if (breakpoint)
		if ((retval = arm7_9_set_breakpoint(target, breakpoint)) != ERROR_OK)
		{
			return retval;
		}

	return err;
}

int arm7_9_read_core_reg(struct target_s *target, int num, enum armv4_5_mode mode)
{
	u32* reg_p[16];
	u32 value;
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

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
		return retval;
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
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

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

	return jtag_execute_queue();
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

	LOG_DEBUG("address: 0x%8.8x, size: 0x%8.8x, count: 0x%8.8x", address, size, count);

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
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

	int j=0;

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
					retval = arm7_9_execute_fast_sys_speed(target);
				else
					retval = arm7_9_execute_sys_speed(target);
				if (retval != ERROR_OK)
					return retval;

				arm7_9->read_core_regs_target_buffer(target, reg_list, buffer, 4);

				/* advance buffer, count number of accesses */
				buffer += thisrun_accesses * 4;
				num_accesses += thisrun_accesses;

				if ((j++%1024)==0)
				{
					keep_alive();
				}
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
						retval = arm7_9_execute_fast_sys_speed(target);
					else
						retval = arm7_9_execute_sys_speed(target);
					if(retval != ERROR_OK)
					{
						return retval;
					}

				}

				arm7_9->read_core_regs_target_buffer(target, reg_list, buffer, 2);

				/* advance buffer, count number of accesses */
				buffer += thisrun_accesses * 2;
				num_accesses += thisrun_accesses;

				if ((j++%1024)==0)
				{
					keep_alive();
				}
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
						retval = arm7_9_execute_fast_sys_speed(target);
					else
						retval = arm7_9_execute_sys_speed(target);
					if(retval != ERROR_OK)
					{
						return retval;
					}
				}

				arm7_9->read_core_regs_target_buffer(target, reg_list, buffer, 1);

				/* advance buffer, count number of accesses */
				buffer += thisrun_accesses * 1;
				num_accesses += thisrun_accesses;

				if ((j++%1024)==0)
				{
					keep_alive();
				}
			}
			break;
		default:
			LOG_ERROR("BUG: we shouldn't get here");
			exit(-1);
			break;
	}

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

	for (i=0; i<=last_reg; i++)
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).valid;

	arm7_9->read_xpsr(target, &cpsr, 0);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("JTAG error while reading cpsr");
		return ERROR_TARGET_DATA_ABORT;
	}

	if (((cpsr & 0x1f) == ARMV4_5_MODE_ABT) && (armv4_5->core_mode != ARMV4_5_MODE_ABT))
	{
		LOG_WARNING("memory read caused data abort (address: 0x%8.8x, size: 0x%x, count: 0x%x)", address, size, count);

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

#ifdef _DEBUG_ARM7_9_
	LOG_DEBUG("address: 0x%8.8x, size: 0x%8.8x, count: 0x%8.8x", address, size, count);
#endif

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
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
					retval = arm7_9_execute_fast_sys_speed(target);
				else
					retval = arm7_9_execute_sys_speed(target);
				if(retval != ERROR_OK)
				{
					return retval;
				}

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
						retval = arm7_9_execute_fast_sys_speed(target);
					else
						retval = arm7_9_execute_sys_speed(target);
					if(retval != ERROR_OK)
					{
						return retval;
					}
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
						retval = arm7_9_execute_fast_sys_speed(target);
					else
						retval = arm7_9_execute_sys_speed(target);
					if(retval != ERROR_OK)
					{
						return retval;
					}

				}

				num_accesses += thisrun_accesses;
			}
			break;
		default:
			LOG_ERROR("BUG: we shouldn't get here");
			exit(-1);
			break;
	}

	/* Re-Set DBGACK */
	buf_set_u32(dbg_ctrl->value, EICE_DBG_CONTROL_DBGACK, 1, 1);
	embeddedice_store_reg(dbg_ctrl);

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

	for (i=0; i<=last_reg; i++)
		ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).valid;

	arm7_9->read_xpsr(target, &cpsr, 0);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("JTAG error while reading cpsr");
		return ERROR_TARGET_DATA_ABORT;
	}

	if (((cpsr & 0x1f) == ARMV4_5_MODE_ABT) && (armv4_5->core_mode != ARMV4_5_MODE_ABT))
	{
		LOG_WARNING("memory write caused data abort (address: 0x%8.8x, size: 0x%x, count: 0x%x)", address, size, count);

		arm7_9->write_xpsr_im8(target, buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 8) & ~0x20, 0, 0);

		return ERROR_TARGET_DATA_ABORT;
	}

	return ERROR_OK;
}

static int dcc_count;
static u8 *dcc_buffer;

static int arm7_9_dcc_completion(struct target_s *target, u32 exit_point, int timeout_ms, void *arch_info)
{
	int retval = ERROR_OK;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	if ((retval=target_wait_state(target, TARGET_DEBUG_RUNNING, 500))!=ERROR_OK)
		return retval;

	int little=target->endianness==TARGET_LITTLE_ENDIAN;
	int count=dcc_count;
	u8 *buffer=dcc_buffer;
	if (count>2)
	{
		/* Handle first & last using standard embeddedice_write_reg and the middle ones w/the
		 * core function repeated. */
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_COMMS_DATA], fast_target_buffer_get_u32(buffer, little));
		buffer+=4;

		embeddedice_reg_t *ice_reg = arm7_9->eice_cache->reg_list[EICE_COMMS_DATA].arch_info;
		u8 reg_addr = ice_reg->addr & 0x1f;
		jtag_tap_t *tap;
		tap = ice_reg->jtag_info->tap;

		embeddedice_write_dcc(tap, reg_addr, buffer, little, count-2);
		buffer += (count-2)*4;

		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_COMMS_DATA], fast_target_buffer_get_u32(buffer, little));
	} else
	{
		int i;
		for (i = 0; i < count; i++)
		{
			embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_COMMS_DATA], fast_target_buffer_get_u32(buffer, little));
			buffer += 4;
		}
	}

	if((retval = target_halt(target))!= ERROR_OK)
	{
		return retval;
	}
	return target_wait_state(target, TARGET_HALTED, 500);
}

static const u32 dcc_code[] =
{
	/* MRC      TST         BNE         MRC         STR         B */
	0xee101e10, 0xe3110001, 0x0afffffc, 0xee111e10, 0xe4801004, 0xeafffff9
};

int armv4_5_run_algorithm_inner(struct target_s *target, int num_mem_params, mem_param_t *mem_params, int num_reg_params, reg_param_t *reg_params, u32 entry_point, u32 exit_point, int timeout_ms, void *arch_info, int (*run_it)(struct target_s *target, u32 exit_point, int timeout_ms, void *arch_info));

int arm7_9_bulk_write_memory(target_t *target, u32 address, u32 count, u8 *buffer)
{
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	int i;

	if (!arm7_9->dcc_downloads)
		return target->type->write_memory(target, address, 4, count, buffer);

	/* regrab previously allocated working_area, or allocate a new one */
	if (!arm7_9->dcc_working_area)
	{
		u8 dcc_code_buf[6 * 4];

		/* make sure we have a working area */
		if (target_alloc_working_area(target, 24, &arm7_9->dcc_working_area) != ERROR_OK)
		{
			LOG_INFO("no working area available, falling back to memory writes");
			return target->type->write_memory(target, address, 4, count, buffer);
		}

		/* copy target instructions to target endianness */
		for (i = 0; i < 6; i++)
		{
			target_buffer_set_u32(target, dcc_code_buf + i*4, dcc_code[i]);
		}

		/* write DCC code to working area */
		if ((retval = target->type->write_memory(target, arm7_9->dcc_working_area->address, 4, 6, dcc_code_buf)) != ERROR_OK)
		{
			return retval;
		}
	}

	armv4_5_algorithm_t armv4_5_info;
	reg_param_t reg_params[1];

	armv4_5_info.common_magic = ARMV4_5_COMMON_MAGIC;
	armv4_5_info.core_mode = ARMV4_5_MODE_SVC;
	armv4_5_info.core_state = ARMV4_5_STATE_ARM;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, address);

	dcc_count=count;
	dcc_buffer=buffer;
	retval = armv4_5_run_algorithm_inner(target, 0, NULL, 1, reg_params,
			arm7_9->dcc_working_area->address, arm7_9->dcc_working_area->address+6*4, 20*1000, &armv4_5_info, arm7_9_dcc_completion);

	if (retval==ERROR_OK)
	{
		u32 endaddress=buf_get_u32(reg_params[0].value, 0, 32);
		if (endaddress!=(address+count*4))
		{
			LOG_ERROR("DCC write failed, expected end address 0x%08x got 0x%0x", (address+count*4), endaddress);
			retval=ERROR_FAIL;
		}
	}

	destroy_reg_param(&reg_params[0]);

	return retval;
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
	{
		if ((retval=target_write_u32(target, crc_algorithm->address + i*sizeof(u32), arm7_9_crc_code[i]))!=ERROR_OK)
		{
			return retval;
		}
	}

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
		LOG_ERROR("error executing arm7_9 crc algorithm");
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

int arm7_9_blank_check_memory(struct target_s *target, u32 address, u32 count, u32* blank)
{
	working_area_t *erase_check_algorithm;
	reg_param_t reg_params[3];
	armv4_5_algorithm_t armv4_5_info;
	int retval;
	int i;

	u32 erase_check_code[] =
	{
						/* loop: */
		0xe4d03001,		/* ldrb r3, [r0], #1	*/
		0xe0022003,		/* and r2, r2, r3 		*/
		0xe2511001, 	/* subs r1, r1, #1		*/
		0x1afffffb,		/* bne loop				*/
						/* end: */
		0xeafffffe		/* b end				*/
	};

	/* make sure we have a working area */
	if (target_alloc_working_area(target, sizeof(erase_check_code), &erase_check_algorithm) != ERROR_OK)
	{
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* convert flash writing code into a buffer in target endianness */
	for (i = 0; i < (sizeof(erase_check_code)/sizeof(u32)); i++)
		if ((retval = target_write_u32(target, erase_check_algorithm->address + i*sizeof(u32), erase_check_code[i])) != ERROR_OK)
		{
			return retval;
		}

	armv4_5_info.common_magic = ARMV4_5_COMMON_MAGIC;
	armv4_5_info.core_mode = ARMV4_5_MODE_SVC;
	armv4_5_info.core_state = ARMV4_5_STATE_ARM;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, address);

	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, count);

	init_reg_param(&reg_params[2], "r2", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, 0xff);

	if ((retval = target->type->run_algorithm(target, 0, NULL, 3, reg_params,
			erase_check_algorithm->address, erase_check_algorithm->address + (sizeof(erase_check_code) - 4), 10000, &armv4_5_info)) != ERROR_OK)
	{
		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		target_free_working_area(target, erase_check_algorithm);
		return 0;
	}

	*blank = buf_get_u32(reg_params[2].value, 0, 32);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	target_free_working_area(target, erase_check_algorithm);

	return ERROR_OK;
}

int arm7_9_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *arm7_9_cmd;

	arm7_9_cmd = register_command(cmd_ctx, NULL, "arm7_9", NULL, COMMAND_ANY, "arm7/9 specific commands");

	register_command(cmd_ctx, arm7_9_cmd, "write_xpsr", handle_arm7_9_write_xpsr_command, COMMAND_EXEC, "write program status register <value> <not cpsr|spsr>");
	register_command(cmd_ctx, arm7_9_cmd, "write_xpsr_im8", handle_arm7_9_write_xpsr_im8_command, COMMAND_EXEC, "write program status register <8bit immediate> <rotate> <not cpsr|spsr>");

	register_command(cmd_ctx, arm7_9_cmd, "write_core_reg", handle_arm7_9_write_core_reg_command, COMMAND_EXEC, "write core register <num> <mode> <value>");

	register_command(cmd_ctx, arm7_9_cmd, "dbgrq", handle_arm7_9_dbgrq_command,
		COMMAND_ANY, "use EmbeddedICE dbgrq instead of breakpoint for target halt requests <enable|disable>");
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
		LOG_ERROR("JTAG error while writing to xpsr");
		return retval;
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
		LOG_ERROR("JTAG error while writing 8-bit immediate to xpsr");
		return retval;
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

	return arm7_9_write_core_reg(target, num, mode, value);
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
	int retval = ERROR_OK;
	armv4_5_common_t *armv4_5 = &arm7_9->armv4_5_common;

	arm7_9->common_magic = ARM7_9_COMMON_MAGIC;

	if((retval = arm_jtag_setup_connection(&arm7_9->jtag_info)) != ERROR_OK)
	{
		return retval;
	}

	arm7_9->wp_available = 0; /* this is set up in arm7_9_clear_watchpoints() */
	arm7_9->wp_available_max = 2;
	arm7_9->sw_breakpoints_added = 0;
	arm7_9->breakpoint_count = 0;
	arm7_9->wp0_used = 0;
	arm7_9->wp1_used = 0;
	arm7_9->wp1_used_default = 0;
	arm7_9->use_dbgrq = 0;

	arm7_9->etm_ctx = NULL;
	arm7_9->has_single_step = 0;
	arm7_9->has_monitor_mode = 0;
	arm7_9->has_vector_catch = 0;

	arm7_9->debug_entry_from_reset = 0;

	arm7_9->dcc_working_area = NULL;

	arm7_9->fast_memory_access = fast_and_dangerous;
	arm7_9->dcc_downloads = fast_and_dangerous;

	arm7_9->need_bypass_before_restart = 0;

	armv4_5->arch_info = arm7_9;
	armv4_5->read_core_reg = arm7_9_read_core_reg;
	armv4_5->write_core_reg = arm7_9_write_core_reg;
	armv4_5->full_context = arm7_9_full_context;

	if((retval = armv4_5_init_arch_info(target, armv4_5)) != ERROR_OK)
	{
		return retval;
	}

	if((retval = target_register_timer_callback(arm7_9_handle_target_request, 1, 1, target)) != ERROR_OK)
	{
		return retval;
	}

	return ERROR_OK;
}
