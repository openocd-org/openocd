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
#ifndef ARM7_9_COMMON_H
#define ARM7_9_COMMON_H

#include "armv4_5.h"
#include "arm_jtag.h"
#include "breakpoints.h"
#include "target.h"

#include "etm.h"

#define	ARM7_9_COMMON_MAGIC 0x0a790a79

typedef struct arm7_9_common_s
{
	int common_magic;
	
	arm_jtag_t jtag_info;
	reg_cache_t *eice_cache;
	
	u32 arm_bkpt;
	u16 thumb_bkpt;
	int sw_breakpoints_added;
	int breakpoint_count;
	int wp_available;
	int wp_available_max;
	int wp0_used;
	int wp1_used;
	int wp1_used_default;
	int force_hw_bkpts;
	int dbgreq_adjust_pc;
	int use_dbgrq;
	int need_bypass_before_restart;
	
	etm_context_t *etm_ctx;
	
	int has_single_step;
	int has_monitor_mode;
	int has_vector_catch;
	
	int debug_entry_from_reset;
	
	struct working_area_s *dcc_working_area;
	
	int fast_memory_access;
	int dcc_downloads;

	int (*examine_debug_reason)(target_t *target);
	
	void (*change_to_arm)(target_t *target, u32 *r0, u32 *pc);
	
	void (*read_core_regs)(target_t *target, u32 mask, u32 *core_regs[16]);
	void (*read_core_regs_target_buffer)(target_t *target, u32 mask, void *buffer, int size);
	void (*read_xpsr)(target_t *target, u32 *xpsr, int spsr);
	
	void (*write_xpsr)(target_t *target, u32 xpsr, int spsr);
	void (*write_xpsr_im8)(target_t *target, u8 xpsr_im, int rot, int spsr);
	void (*write_core_regs)(target_t *target, u32 mask, u32 core_regs[16]);
	
	void (*load_word_regs)(target_t *target, u32 mask);
	void (*load_hword_reg)(target_t *target, int num);
	void (*load_byte_reg)(target_t *target, int num);

	void (*store_word_regs)(target_t *target, u32 mask);
	void (*store_hword_reg)(target_t *target, int num);
	void (*store_byte_reg)(target_t *target, int num);
	
	void (*write_pc)(target_t *target, u32 pc);
	void (*branch_resume)(target_t *target);
	void (*branch_resume_thumb)(target_t *target);
	
	void (*enable_single_step)(target_t *target, u32 next_pc);
	void (*disable_single_step)(target_t *target);
	
	void (*set_special_dbgrq)(target_t *target);

	void (*pre_debug_entry)(target_t *target);
	void (*post_debug_entry)(target_t *target);
	
	void (*pre_restore_context)(target_t *target);
	void (*post_restore_context)(target_t *target);
	
	armv4_5_common_t armv4_5_common;
	void *arch_info;

} arm7_9_common_t;

int arm7_9_register_commands(struct command_context_s *cmd_ctx);

int arm7_9_poll(target_t *target);

int arm7_9_target_request_data(target_t *target, u32 size, u8 *buffer);

int arm7_9_setup(target_t *target);
int arm7_9_assert_reset(target_t *target);
int arm7_9_deassert_reset(target_t *target);
int arm7_9_reset_request_halt(target_t *target);
int arm7_9_early_halt(target_t *target);
int arm7_9_soft_reset_halt(struct target_s *target);
int arm7_9_prepare_reset_halt(struct target_s *target);

int arm7_9_halt(target_t *target);
int arm7_9_debug_entry(target_t *target);
int arm7_9_full_context(target_t *target);
int arm7_9_restore_context(target_t *target);
int arm7_9_resume(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution);
int arm7_9_step(struct target_s *target, int current, u32 address, int handle_breakpoints);
int arm7_9_read_core_reg(struct target_s *target, int num, enum armv4_5_mode mode);
int arm7_9_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int arm7_9_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int arm7_9_bulk_write_memory(target_t *target, u32 address, u32 count, u8 *buffer);
int arm7_9_checksum_memory(struct target_s *target, u32 address, u32 count, u32* checksum);
int arm7_9_blank_check_memory(struct target_s *target, u32 address, u32 count, u32* blank);

int arm7_9_run_algorithm(struct target_s *target, int num_mem_params, mem_param_t *mem_params, int num_reg_prams, reg_param_t *reg_param, u32 entry_point, void *arch_info);

int arm7_9_add_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
int arm7_9_remove_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
int arm7_9_add_watchpoint(struct target_s *target, watchpoint_t *watchpoint);
int arm7_9_remove_watchpoint(struct target_s *target, watchpoint_t *watchpoint);

void arm7_9_enable_eice_step(target_t *target, u32 next_pc);
void arm7_9_disable_eice_step(target_t *target);

int arm7_9_execute_sys_speed(struct target_s *target);

int arm7_9_init_arch_info(target_t *target, arm7_9_common_t *arm7_9);
int arm7_9_get_arch_pointers(target_t *target, armv4_5_common_t **armv4_5_p, arm7_9_common_t **arm7_9_p);

#endif /* ARM7_9_COMMON_H */
