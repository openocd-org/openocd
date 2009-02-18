/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
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

#include "mips32.h"
#include "mips_m4k.h"
#include "mips32_dmaacc.h"
#include "jtag.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>

/* cli handling */

/* forward declarations */
int mips_m4k_poll(target_t *target);
int mips_m4k_halt(struct target_s *target);
int mips_m4k_soft_reset_halt(struct target_s *target);
int mips_m4k_resume(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution);
int mips_m4k_step(struct target_s *target, int current, u32 address, int handle_breakpoints);
int mips_m4k_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int mips_m4k_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int mips_m4k_register_commands(struct command_context_s *cmd_ctx);
int mips_m4k_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int mips_m4k_quit(void);
int mips_m4k_target_create(struct target_s *target, Jim_Interp *interp);

int mips_m4k_examine(struct target_s *target);
int mips_m4k_assert_reset(target_t *target);
int mips_m4k_deassert_reset(target_t *target);
int mips_m4k_checksum_memory(target_t *target, u32 address, u32 size, u32 *checksum);

target_type_t mips_m4k_target =
{
	.name = "mips_m4k",

	.poll = mips_m4k_poll,
	.arch_state = mips32_arch_state,

	.target_request_data = NULL,

	.halt = mips_m4k_halt,
	.resume = mips_m4k_resume,
	.step = mips_m4k_step,

	.assert_reset = mips_m4k_assert_reset,
	.deassert_reset = mips_m4k_deassert_reset,
	.soft_reset_halt = mips_m4k_soft_reset_halt,

	.get_gdb_reg_list = mips32_get_gdb_reg_list,

	.read_memory = mips_m4k_read_memory,
	.write_memory = mips_m4k_write_memory,
	.bulk_write_memory = mips_m4k_bulk_write_memory,
	.checksum_memory = mips_m4k_checksum_memory,
	.blank_check_memory = NULL,

	.run_algorithm = mips32_run_algorithm,

	.add_breakpoint = mips_m4k_add_breakpoint,
	.remove_breakpoint = mips_m4k_remove_breakpoint,
	.add_watchpoint = mips_m4k_add_watchpoint,
	.remove_watchpoint = mips_m4k_remove_watchpoint,

	.register_commands = mips_m4k_register_commands,
	.target_create = mips_m4k_target_create,
	.init_target = mips_m4k_init_target,
	.examine = mips_m4k_examine,
	.quit = mips_m4k_quit
};

int mips_m4k_examine_debug_reason(target_t *target)
{
	u32 break_status;
	int retval;

	if ((target->debug_reason != DBG_REASON_DBGRQ)
		&& (target->debug_reason != DBG_REASON_SINGLESTEP))
	{
		/* get info about inst breakpoint support */
		if ((retval = target_read_u32(target, EJTAG_IBS, &break_status)) != ERROR_OK)
			return retval;
		if (break_status & 0x1f)
		{
			/* we have halted on a  breakpoint */
			if ((retval = target_write_u32(target, EJTAG_IBS, 0)) != ERROR_OK)
				return retval;
			target->debug_reason = DBG_REASON_BREAKPOINT;
		}

		/* get info about data breakpoint support */
		if ((retval = target_read_u32(target, 0xFF302000, &break_status)) != ERROR_OK)
			return retval;
		if (break_status & 0x1f)
		{
			/* we have halted on a  breakpoint */
			if ((retval = target_write_u32(target, 0xFF302000, 0)) != ERROR_OK)
				return retval;
			target->debug_reason = DBG_REASON_WATCHPOINT;
		}
	}

	return ERROR_OK;
}

int mips_m4k_debug_entry(target_t *target)
{
	mips32_common_t *mips32 = target->arch_info;
	mips_ejtag_t *ejtag_info = &mips32->ejtag_info;
	u32 debug_reg;

	/* read debug register */
	mips_ejtag_read_debug(ejtag_info, &debug_reg);

	/* make sure break uit configured */
	mips32_configure_break_unit(target);

	/* attempt to find halt reason */
	mips_m4k_examine_debug_reason(target);

	/* clear single step if active */
	if (debug_reg & EJTAG_DEBUG_DSS)
	{
		/* stopped due to single step - clear step bit */
		mips_ejtag_config_step(ejtag_info, 0);
	}

	mips32_save_context(target);

	LOG_DEBUG("entered debug state at PC 0x%x, target->state: %s",
		*(u32*)(mips32->core_cache->reg_list[MIPS32_PC].value),
		  Jim_Nvp_value2name_simple( nvp_target_state, target->state )->name);

	return ERROR_OK;
}

int mips_m4k_poll(target_t *target)
{
	int retval;
	mips32_common_t *mips32 = target->arch_info;
	mips_ejtag_t *ejtag_info = &mips32->ejtag_info;
	u32 ejtag_ctrl = ejtag_info->ejtag_ctrl;

	/* read ejtag control reg */
	jtag_add_end_state(TAP_IDLE);
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* clear this bit before handling polling
	 * as after reset registers will read zero */
	if (ejtag_ctrl & EJTAG_CTRL_ROCC)
	{
		/* we have detected a reset, clear flag
		 * otherwise ejtag will not work */
		jtag_add_end_state(TAP_IDLE);
		ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_ROCC;

		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
		mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
		LOG_DEBUG("Reset Detected");
	}

	/* check for processor halted */
	if (ejtag_ctrl & EJTAG_CTRL_BRKST)
	{
		if ((target->state == TARGET_RUNNING) || (target->state == TARGET_RESET))
		{
			jtag_add_end_state(TAP_IDLE);
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_NORMALBOOT, NULL);

			target->state = TARGET_HALTED;

			if ((retval = mips_m4k_debug_entry(target)) != ERROR_OK)
				return retval;

			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
		else if (target->state == TARGET_DEBUG_RUNNING)
		{
			target->state = TARGET_HALTED;

			if ((retval = mips_m4k_debug_entry(target)) != ERROR_OK)
				return retval;

			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		}
	}
	else
	{
		target->state = TARGET_RUNNING;
	}

//	LOG_DEBUG("ctrl=0x%08X", ejtag_ctrl);

	return ERROR_OK;
}

int mips_m4k_halt(struct target_s *target)
{
	mips32_common_t *mips32 = target->arch_info;
	mips_ejtag_t *ejtag_info = &mips32->ejtag_info;

	LOG_DEBUG("target->state: %s",
		  Jim_Nvp_value2name_simple( nvp_target_state, target->state )->name);

	if (target->state == TARGET_HALTED)
	{
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
	{
		LOG_WARNING("target was in unknown state when halt was requested");
	}

	if (target->state == TARGET_RESET)
	{
		if ((jtag_reset_config & RESET_SRST_PULLS_TRST) && jtag_srst)
		{
			LOG_ERROR("can't request a halt while in reset if nSRST pulls nTRST");
			return ERROR_TARGET_FAILURE;
		}
		else
		{
			/* we came here in a reset_halt or reset_init sequence
			 * debug entry was already prepared in mips32_prepare_reset_halt()
			 */
			target->debug_reason = DBG_REASON_DBGRQ;

			return ERROR_OK;
		}
	}

	/* break processor */
	mips_ejtag_enter_debug(ejtag_info);

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

int mips_m4k_assert_reset(target_t *target)
{
	mips32_common_t *mips32 = target->arch_info;
	mips_ejtag_t *ejtag_info = &mips32->ejtag_info;

	LOG_DEBUG("target->state: %s",
		Jim_Nvp_value2name_simple( nvp_target_state, target->state )->name);

	if (!(jtag_reset_config & RESET_HAS_SRST))
	{
		LOG_ERROR("Can't assert SRST");
		return ERROR_FAIL;
	}

	if (target->reset_halt)
	{
		/* use hardware to catch reset */
		jtag_add_end_state(TAP_IDLE);
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_EJTAGBOOT, NULL);
	}
	else
	{
		jtag_add_end_state(TAP_IDLE);
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_NORMALBOOT, NULL);
	}

	if (strcmp(target->variant, "ejtag_srst") == 0)
	{
		u32 ejtag_ctrl = ejtag_info->ejtag_ctrl | EJTAG_CTRL_PRRST | EJTAG_CTRL_PERRST;
		LOG_DEBUG("Using EJTAG reset (PRRST) to reset processor...");
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);
		mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	}
	else
	{
		/* here we should issue a srst only, but we may have to assert trst as well */
		if (jtag_reset_config & RESET_SRST_PULLS_TRST)
		{
			jtag_add_reset(1, 1);
		}
		else
		{
			jtag_add_reset(0, 1);
		}
	}

	target->state = TARGET_RESET;
	jtag_add_sleep(50000);

	mips32_invalidate_core_regs(target);

	if (target->reset_halt)
	{
		int retval;
		if ((retval = target_halt(target))!=ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int mips_m4k_deassert_reset(target_t *target)
{
	LOG_DEBUG("target->state: %s",
		Jim_Nvp_value2name_simple( nvp_target_state, target->state )->name);

	/* deassert reset lines */
	jtag_add_reset(0, 0);

	return ERROR_OK;
}

int mips_m4k_soft_reset_halt(struct target_s *target)
{
	/* TODO */
	return ERROR_OK;
}

int mips_m4k_single_step_core(target_t *target)
{
	mips32_common_t *mips32 = target->arch_info;
	mips_ejtag_t *ejtag_info = &mips32->ejtag_info;

	/* configure single step mode */
	mips_ejtag_config_step(ejtag_info, 1);

	/* exit debug mode */
	mips_ejtag_exit_debug(ejtag_info, 1);

	mips_m4k_debug_entry(target);

	return ERROR_OK;
}

int mips_m4k_resume(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution)
{
	mips32_common_t *mips32 = target->arch_info;
	mips_ejtag_t *ejtag_info = &mips32->ejtag_info;
	breakpoint_t *breakpoint = NULL;
	u32 resume_pc;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!debug_execution)
	{
		target_free_all_working_areas(target);
		mips_m4k_enable_breakpoints(target);
		mips_m4k_enable_watchpoints(target);
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
	{
		buf_set_u32(mips32->core_cache->reg_list[MIPS32_PC].value, 0, 32, address);
		mips32->core_cache->reg_list[MIPS32_PC].dirty = 1;
		mips32->core_cache->reg_list[MIPS32_PC].valid = 1;
	}

	resume_pc = buf_get_u32(mips32->core_cache->reg_list[MIPS32_PC].value, 0, 32);

	mips32_restore_context(target);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints)
	{
		/* Single step past breakpoint at current address */
		if ((breakpoint = breakpoint_find(target, resume_pc)))
		{
			LOG_DEBUG("unset breakpoint at 0x%8.8x", breakpoint->address);
			mips_m4k_unset_breakpoint(target, breakpoint);
			mips_m4k_single_step_core(target);
			mips_m4k_set_breakpoint(target, breakpoint);
		}
	}

	/* exit debug mode - enable interrupts if required */
	mips_ejtag_exit_debug(ejtag_info, !debug_execution);
	target->debug_reason = DBG_REASON_NOTHALTED;

	/* registers are now invalid */
	mips32_invalidate_core_regs(target);

	if (!debug_execution)
	{
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("target resumed at 0x%x", resume_pc);
	}
	else
	{
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("target debug resumed at 0x%x", resume_pc);
	}

	return ERROR_OK;
}

int mips_m4k_step(struct target_s *target, int current, u32 address, int handle_breakpoints)
{
	/* get pointers to arch-specific information */
	mips32_common_t *mips32 = target->arch_info;
	mips_ejtag_t *ejtag_info = &mips32->ejtag_info;
	breakpoint_t *breakpoint = NULL;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
		buf_set_u32(mips32->core_cache->reg_list[MIPS32_PC].value, 0, 32, address);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints)
		if ((breakpoint = breakpoint_find(target, buf_get_u32(mips32->core_cache->reg_list[MIPS32_PC].value, 0, 32))))
			mips_m4k_unset_breakpoint(target, breakpoint);

	/* restore context */
	mips32_restore_context(target);

	/* configure single step mode */
	mips_ejtag_config_step(ejtag_info, 1);

	target->debug_reason = DBG_REASON_SINGLESTEP;

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	/* exit debug mode */
	mips_ejtag_exit_debug(ejtag_info, 1);

	/* registers are now invalid */
	mips32_invalidate_core_regs(target);

	if (breakpoint)
		mips_m4k_set_breakpoint(target, breakpoint);

	LOG_DEBUG("target stepped ");

	mips_m4k_debug_entry(target);
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	return ERROR_OK;
}

void mips_m4k_enable_breakpoints(struct target_s *target)
{
	breakpoint_t *breakpoint = target->breakpoints;

	/* set any pending breakpoints */
	while (breakpoint)
	{
		if (breakpoint->set == 0)
			mips_m4k_set_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}
}

int mips_m4k_set_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	mips32_common_t *mips32 = target->arch_info;
	mips32_comparator_t * comparator_list = mips32->inst_break_list;
	int retval;
	
	if (breakpoint->set)
	{
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		int bp_num = 0;

		while(comparator_list[bp_num].used && (bp_num < mips32->num_inst_bpoints))
			bp_num++;
		if (bp_num >= mips32->num_inst_bpoints)
		{
			LOG_DEBUG("ERROR Can not find free FP Comparator");
			LOG_WARNING("ERROR Can not find free FP Comparator");
			exit(-1);
		}
		breakpoint->set = bp_num + 1;
		comparator_list[bp_num].used = 1;
		comparator_list[bp_num].bp_value = breakpoint->address;
		target_write_u32(target, comparator_list[bp_num].reg_address, comparator_list[bp_num].bp_value);
		target_write_u32(target, comparator_list[bp_num].reg_address + 0x08, 0x00000000);
		target_write_u32(target, comparator_list[bp_num].reg_address + 0x18, 1);
		LOG_DEBUG("bp_num %i bp_value 0x%x", bp_num, comparator_list[bp_num].bp_value);
	}
	else if (breakpoint->type == BKPT_SOFT)
	{
		if (breakpoint->length == 4)
		{
			u32 verify = 0xffffffff;
			
			if((retval = target->type->read_memory(target, breakpoint->address, breakpoint->length, 1, breakpoint->orig_instr)) != ERROR_OK)
			{
				return retval;
			}
			if ((retval = target_write_u32(target, breakpoint->address, MIPS32_SDBBP)) != ERROR_OK)
			{
				return retval;
			}
			
			if ((retval = target_read_u32(target, breakpoint->address, &verify)) != ERROR_OK)
			{
				return retval;
			}
			if (verify != MIPS32_SDBBP)
			{
				LOG_ERROR("Unable to set 32bit breakpoint at address %08x - check that memory is read/writable", breakpoint->address);
				return ERROR_OK;
			}
		}
		else
		{
			u16 verify = 0xffff;
			
			if((retval = target->type->read_memory(target, breakpoint->address, breakpoint->length, 1, breakpoint->orig_instr)) != ERROR_OK)
			{
				return retval;
			}
			if ((retval = target_write_u16(target, breakpoint->address, MIPS16_SDBBP)) != ERROR_OK)
			{
				return retval;
			}
			
			if ((retval = target_read_u16(target, breakpoint->address, &verify)) != ERROR_OK)
			{
				return retval;
			}
			if (verify != MIPS16_SDBBP)
			{
				LOG_ERROR("Unable to set 16bit breakpoint at address %08x - check that memory is read/writable", breakpoint->address);
				return ERROR_OK;
			}
		}
		
		breakpoint->set = 20; /* Any nice value but 0 */
	}

	return ERROR_OK;
}

int mips_m4k_unset_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	/* get pointers to arch-specific information */
	mips32_common_t *mips32 = target->arch_info;
	mips32_comparator_t * comparator_list = mips32->inst_break_list;
	int retval;
	
	if (!breakpoint->set)
	{
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		int bp_num = breakpoint->set - 1;
		if ((bp_num < 0) || (bp_num >= mips32->num_inst_bpoints))
		{
			LOG_DEBUG("Invalid FP Comparator number in breakpoint");
			return ERROR_OK;
		}
		comparator_list[bp_num].used = 0;
		comparator_list[bp_num].bp_value = 0;
		target_write_u32(target, comparator_list[bp_num].reg_address + 0x18, 0);
	}
	else
	{
		/* restore original instruction (kept in target endianness) */
		if (breakpoint->length == 4)
		{
			u32 current_instr;
			
			/* check that user program has not modified breakpoint instruction */
			if ((retval = target->type->read_memory(target, breakpoint->address, 4, 1, (u8*)&current_instr)) != ERROR_OK)
			{
				return retval;
			}
			if (current_instr == MIPS32_SDBBP)
			{
				if((retval = target->type->write_memory(target, breakpoint->address, 4, 1, breakpoint->orig_instr)) != ERROR_OK)
				{
					return retval;
				}
			}
		}
		else
		{
			u16 current_instr;
			
			/* check that user program has not modified breakpoint instruction */
			if ((retval = target->type->read_memory(target, breakpoint->address, 2, 1, (u8*)&current_instr)) != ERROR_OK)
			{
				return retval;
			}
			
			if (current_instr == MIPS16_SDBBP)
			{
				if((retval = target->type->write_memory(target, breakpoint->address, 2, 1, breakpoint->orig_instr)) != ERROR_OK)
				{
					return retval;
				}
			}
		}
	}
	breakpoint->set = 0;

	return ERROR_OK;
}

int mips_m4k_add_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	mips32_common_t *mips32 = target->arch_info;

	if (breakpoint->type == BKPT_HARD)
	{
		if (mips32->num_inst_bpoints_avail < 1)
		{
			LOG_INFO("no hardware breakpoint available");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		
		mips32->num_inst_bpoints_avail--;
	}	

	mips_m4k_set_breakpoint(target, breakpoint);

	return ERROR_OK;
}

int mips_m4k_remove_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	/* get pointers to arch-specific information */
	mips32_common_t *mips32 = target->arch_info;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (breakpoint->set)
	{
		mips_m4k_unset_breakpoint(target, breakpoint);
	}

	if (breakpoint->type == BKPT_HARD)
		mips32->num_inst_bpoints_avail++;

	return ERROR_OK;
}

int mips_m4k_set_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	/* TODO */
	return ERROR_OK;
}

int mips_m4k_unset_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	/* TODO */
	return ERROR_OK;
}

int mips_m4k_add_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	/* TODO */
	return ERROR_OK;
}

int mips_m4k_remove_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	/* TODO */
	return ERROR_OK;
}

void mips_m4k_enable_watchpoints(struct target_s *target)
{
	watchpoint_t *watchpoint = target->watchpoints;

	/* set any pending watchpoints */
	while (watchpoint)
	{
		if (watchpoint->set == 0)
			mips_m4k_set_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}
}

int mips_m4k_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	mips32_common_t *mips32 = target->arch_info;
	mips_ejtag_t *ejtag_info = &mips32->ejtag_info;

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

	switch (size)
	{
		case 4:
		case 2:
		case 1:
			/* if noDMA off, use DMAACC mode for memory read */
			if(ejtag_info->impcode & EJTAG_IMP_NODMA)
				return mips32_pracc_read_mem(ejtag_info, address, size, count, (void *)buffer);
			else
				return mips32_dmaacc_read_mem(ejtag_info, address, size, count, (void *)buffer);
		default:
			LOG_ERROR("BUG: we shouldn't get here");
			exit(-1);
			break;
	}

	return ERROR_OK;
}

int mips_m4k_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	mips32_common_t *mips32 = target->arch_info;
	mips_ejtag_t *ejtag_info = &mips32->ejtag_info;

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

	switch (size)
	{
		case 4:
		case 2:
		case 1:
			/* if noDMA off, use DMAACC mode for memory write */
			if(ejtag_info->impcode & EJTAG_IMP_NODMA)
				mips32_pracc_write_mem(ejtag_info, address, size, count, (void *)buffer);
			else
				mips32_dmaacc_write_mem(ejtag_info, address, size, count, (void *)buffer);
			break;
		default:
			LOG_ERROR("BUG: we shouldn't get here");
			exit(-1);
			break;
	}

	return ERROR_OK;
}

int mips_m4k_register_commands(struct command_context_s *cmd_ctx)
{
	int retval;

	retval = mips32_register_commands(cmd_ctx);
	return retval;
}

int mips_m4k_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	mips32_build_reg_cache(target);

	return ERROR_OK;
}

int mips_m4k_quit(void)
{
	return ERROR_OK;
}

int mips_m4k_init_arch_info(target_t *target, mips_m4k_common_t *mips_m4k, jtag_tap_t *tap)
{
	mips32_common_t *mips32 = &mips_m4k->mips32_common;

	mips_m4k->common_magic = MIPSM4K_COMMON_MAGIC;

	/* initialize mips4k specific info */
	mips32_init_arch_info(target, mips32, tap);
	mips32->arch_info = mips_m4k;

	return ERROR_OK;
}

int mips_m4k_target_create(struct target_s *target, Jim_Interp *interp)
{
	mips_m4k_common_t *mips_m4k = calloc(1,sizeof(mips_m4k_common_t));

	mips_m4k_init_arch_info(target, mips_m4k, target->tap);

	return ERROR_OK;
}

int mips_m4k_examine(struct target_s *target)
{
	int retval;
	mips32_common_t *mips32 = target->arch_info;
	mips_ejtag_t *ejtag_info = &mips32->ejtag_info;
	u32 idcode = 0;

	if (!target->type->examined)
	{
		mips_ejtag_get_idcode(ejtag_info, &idcode, NULL);
		ejtag_info->idcode = idcode;
		
		if (((idcode >> 1) & 0x7FF) == 0x29)
		{
			/* we are using a pic32mx so select ejtag port
			 * as it is not selected by default */
			mips_ejtag_set_instr(ejtag_info, 0x05, NULL);
			LOG_DEBUG("PIC32MX Detected - using EJTAG Interface");
		}
	}

	/* init rest of ejtag interface */
	if ((retval = mips_ejtag_init(ejtag_info)) != ERROR_OK)
		return retval;

	if ((retval = mips32_examine(target)) != ERROR_OK)
		return retval;

	return ERROR_OK;
}

int mips_m4k_bulk_write_memory(target_t *target, u32 address, u32 count, u8 *buffer)
{
	return mips_m4k_write_memory(target, address, 4, count, buffer);
}

int mips_m4k_checksum_memory(target_t *target, u32 address, u32 size, u32 *checksum)
{
	return ERROR_FAIL; /* use bulk read method */
}
