/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2010 by Drasko DRASKOVIC                                *
 *   drasko.draskovic@gmail.com                                            *
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

#include "arm946e.h"
#include "target_type.h"
#include "arm_opcodes.h"

#include "breakpoints.h"

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

#define NB_CACHE_WAYS 4

static uint32_t dc;
static uint32_t ic;

/**
 * flag to give info about cache manipulation during debug :
 * "0"	-	cache lines are invalidated "on the fly", for affected addresses.
 *			This is prefered from performance point of view.
 * "1"	-	cache is invalidated and switched off on debug_entry, and switched back on on restore.
 *			It is kept off during debugging.
 */
static uint8_t arm946e_preserve_cache;

int arm946e_post_debug_entry(struct target *target);
void arm946e_pre_restore_context(struct target *target);
static int arm946e_read_cp15(struct target *target, int reg_addr, uint32_t *value);

int arm946e_init_arch_info(struct target *target,
	struct arm946e_common *arm946e,
	struct jtag_tap *tap)
{
	struct arm7_9_common *arm7_9 = &arm946e->arm7_9_common;

	/* initialize arm7/arm9 specific info (including armv4_5) */
	arm9tdmi_init_arch_info(target, arm7_9, tap);

	arm946e->common_magic = ARM946E_COMMON_MAGIC;

	/**
	 * The ARM946E-S implements the ARMv5TE architecture which
	 * has the BKPT instruction, so we don't have to use a watchpoint comparator
	 */
	arm7_9->arm_bkpt = ARMV5_BKPT(0x0);
	arm7_9->thumb_bkpt = ARMV5_T_BKPT(0x0) & 0xffff;


	arm7_9->post_debug_entry = arm946e_post_debug_entry;
	arm7_9->pre_restore_context = arm946e_pre_restore_context;

	/**
	 * disabling linefills leads to lockups, so keep them enabled for now
	 * this doesn't affect correctness, but might affect timing issues, if
	 * important data is evicted from the cache during the debug session
	 */
	arm946e_preserve_cache = 0;

	/* override hw single-step capability from ARM9TDMI */
	/* arm7_9->has_single_step = 1; */

	return ERROR_OK;
}

static int arm946e_target_create(struct target *target, Jim_Interp *interp)
{
	struct arm946e_common *arm946e = calloc(1, sizeof(struct arm946e_common));

	arm946e_init_arch_info(target, arm946e, target->tap);

	return ERROR_OK;
}

static int arm946e_verify_pointer(struct command_context *cmd_ctx,
	struct arm946e_common *arm946e)
{
	if (arm946e->common_magic != ARM946E_COMMON_MAGIC) {
		command_print(cmd_ctx, "target is not an ARM946");
		return ERROR_TARGET_INVALID;
	}
	return ERROR_OK;
}

/*
 * REVISIT:  The "read_cp15" and "write_cp15" commands could hook up
 * to eventual mrc() and mcr() routines ... the reg_addr values being
 * constructed (for CP15 only) from Opcode_1, Opcode_2, and CRn values.
 * See section 7.3 of the ARM946E-S TRM.
 */
static int arm946e_read_cp15(struct target *target, int reg_addr, uint32_t *value)
{
	int retval = ERROR_OK;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	struct scan_field fields[3];
	uint8_t reg_addr_buf = reg_addr & 0x3f;
	uint8_t nr_w_buf = 0;

	retval = arm_jtag_scann(jtag_info, 0xf, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 32;
	/* REVISIT: table 7-2 shows that bits 31-31 need to be
	 * specified for accessing BIST registers ...
	 */
	fields[0].out_value = NULL;
	fields[0].in_value = NULL;

	fields[1].num_bits = 6;
	fields[1].out_value = &reg_addr_buf;
	fields[1].in_value = NULL;

	fields[2].num_bits = 1;
	fields[2].out_value = &nr_w_buf;
	fields[2].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_IDLE);

	fields[0].in_value = (uint8_t *)value;
	jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_IDLE);

	jtag_add_callback(arm_le_to_h_u32, (jtag_callback_data_t)value);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	LOG_DEBUG("addr: 0x%x value: %8.8x", reg_addr, *value);
#endif

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

int arm946e_write_cp15(struct target *target, int reg_addr, uint32_t value)
{
	int retval = ERROR_OK;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	struct scan_field fields[3];
	uint8_t reg_addr_buf = reg_addr & 0x3f;
	uint8_t nr_w_buf = 1;
	uint8_t value_buf[4];

	buf_set_u32(value_buf, 0, 32, value);

	retval = arm_jtag_scann(jtag_info, 0xf, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 32;
	fields[0].out_value = value_buf;
	fields[0].in_value = NULL;

	fields[1].num_bits = 6;
	fields[1].out_value = &reg_addr_buf;
	fields[1].in_value = NULL;

	fields[2].num_bits = 1;
	fields[2].out_value = &nr_w_buf;
	fields[2].in_value = NULL;

	jtag_add_dr_scan(jtag_info->tap, 3, fields, TAP_IDLE);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	LOG_DEBUG("addr: 0x%x value: %8.8x", reg_addr, value);
#endif

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

uint32_t arm946e_invalidate_whole_dcache(struct target *target)
{

	uint32_t csize = 0;
	uint32_t shift = 0;
	uint32_t cp15_idx, seg, dtag;
	int nb_idx, idx = 0;
	int retval;

	/* Get cache type */
	arm946e_read_cp15(target, 0x01, (uint32_t *) &csize);

	csize = (csize >> 18) & 0x0F;

	if (csize == 0)
		shift = 0;
	else
		shift = csize - 0x3;	/* Now 0 = 4KB, 1 = 8KB, ... */

	/* Cache size, given in bytes */
	csize = 1 << (12 + shift);
	/* One line (index) is 32 bytes (8 words) long */
	nb_idx = (csize / 32);	/* gives nb of lines (indexes) in the cache */

	/* Loop for all segmentde (i.e. ways) */
	for (seg = 0; seg < NB_CACHE_WAYS; seg++) {
		/* Loop for all indexes */
		for (idx = 0; idx < nb_idx; idx++) {
			/* Form and write cp15 index (segment + line idx) */
			cp15_idx = seg << 30 | idx << 5;
			retval = arm946e_write_cp15(target, 0x3a, cp15_idx);
			if (retval != ERROR_OK) {
				LOG_DEBUG("ERROR writing index");
				return retval;
			}

			/* Read dtag */
			arm946e_read_cp15(target, 0x16, (uint32_t *) &dtag);

			/* Check cache line VALID bit */
			if (!(dtag >> 4 & 0x1))
				continue;

			/* Clean data cache line */
			retval = arm946e_write_cp15(target, 0x35, 0x1);
			if (retval != ERROR_OK) {
				LOG_DEBUG("ERROR cleaning cache line");
				return retval;
			}

			/* Flush data cache line */
			retval = arm946e_write_cp15(target, 0x1a, 0x1);
			if (retval != ERROR_OK) {
				LOG_DEBUG("ERROR flushing cache line");
				return retval;
			}
		}
	}

	return ERROR_OK;
}

uint32_t arm946e_invalidate_whole_icache(struct target *target)
{
	int retval;

	LOG_DEBUG("FLUSHING I$");

	/**
	 *  Invalidate (flush) I$
	 *  mcr	15, 0, r0, cr7, cr5, {0}
	 */
	retval = arm946e_write_cp15(target, 0x0f, 0x1);
	if (retval != ERROR_OK) {
		LOG_DEBUG("ERROR flushing I$");
		return retval;
	}

	return ERROR_OK;
}

int arm946e_post_debug_entry(struct target *target)
{
	uint32_t ctr_reg = 0x0;
	uint32_t retval = ERROR_OK;

	/* See if CACHES are enabled, and save that info
	 * in the global vars, so that arm946e_pre_restore_context() can use them */
	arm946e_read_cp15(target, 0x02, (uint32_t *) &ctr_reg);
	dc = (ctr_reg >> 2) & 0x01;
	ic = (ctr_reg >> 12) & 0x01;

	if (arm946e_preserve_cache) {
		if (dc == 1) {
			/* Clean and flush D$ */
			arm946e_invalidate_whole_dcache(target);

			/* Disable D$ */
			ctr_reg &= ~(1 << 2);
		}

		if (ic == 1) {
			/* Flush I$ */
			arm946e_invalidate_whole_icache(target);

			/* Disable I$ */
			ctr_reg &= ~(1 << 12);
		}

		/* Write the new configuration */
		retval = arm946e_write_cp15(target, 0x02, ctr_reg);
		if (retval != ERROR_OK) {
			LOG_DEBUG("ERROR disabling cache");
			return retval;
		}
	}	/* if preserve_cache */

	return ERROR_OK;
}

void arm946e_pre_restore_context(struct target *target)
{
	uint32_t ctr_reg = 0x0;
	uint32_t retval;

	if (arm946e_preserve_cache) {
		/* Get the contents of the CTR reg */
		arm946e_read_cp15(target, 0x02, (uint32_t *) &ctr_reg);

		/**
		 * Read-modify-write CP15 test state register
		 * to reenable I/D-cache linefills
		 */
		if (dc == 1) {
			/* Enable D$ */
			ctr_reg |= 1 << 2;
		}

		if (ic == 1) {
			/* Enable I$ */
			ctr_reg |= 1 << 12;
		}

		/* Write the new configuration */
		retval = arm946e_write_cp15(target, 0x02, ctr_reg);
		if (retval != ERROR_OK)
			LOG_DEBUG("ERROR enabling cache");
	}	/* if preserve_cache */
}

uint32_t arm946e_invalidate_dcache(struct target *target, uint32_t address,
	uint32_t size, uint32_t count)
{
	uint32_t csize = 0x0;
	uint32_t shift = 0;
	uint32_t cur_addr = 0x0;
	uint32_t cp15_idx, set, way, dtag;
	uint32_t i = 0;
	int retval;

	for (i = 0; i < count*size; i++) {
		cur_addr = address + i;

		/* Get cache type */
		arm946e_read_cp15(target, 0x01, (uint32_t *) &csize);

		/* Conclude cache size to find number of lines */
		csize = (csize >> 18) & 0x0F;

		if (csize == 0)
			shift = 0;
		else
			shift = csize - 0x3;	/* Now 0 = 4KB, 1 = 8KB, ... */

		csize = 1 << (12 + shift);

		set = (cur_addr >> 5) & 0xff;	/* set field is 8 bits long */

		for (way = 0; way < NB_CACHE_WAYS; way++) {
			/**
			 * Find if the affected address is kept in the cache.
			 * Because JTAG Scan Chain 15 offers limited approach,
			 * we have to loop through all cache ways (segments) and
			 * read cache tags, then compare them with with address.
			 */

			/* Form and write cp15 index (segment + line idx) */
			cp15_idx = way << 30 | set << 5;
			retval = arm946e_write_cp15(target, 0x3a, cp15_idx);
			if (retval != ERROR_OK) {
				LOG_DEBUG("ERROR writing index");
				return retval;
			}

			/* Read dtag */
			arm946e_read_cp15(target, 0x16, (uint32_t *) &dtag);

			/* Check cache line VALID bit */
			if (!(dtag >> 4 & 0x1))
				continue;

			/* If line is valid and corresponds to affected address - invalidate it */
			if (dtag >> 5 == cur_addr >> 5) {
				/* Clean data cache line */
				retval = arm946e_write_cp15(target, 0x35, 0x1);
				if (retval != ERROR_OK) {
					LOG_DEBUG("ERROR cleaning cache line");
					return retval;
				}

				/* Flush data cache line */
				retval = arm946e_write_cp15(target, 0x1c, 0x1);
				if (retval != ERROR_OK) {
					LOG_DEBUG("ERROR flushing cache line");
					return retval;
				}

				break;
			}
		}	/* loop through all 4 ways */
	}	/* loop through all addresses */

	return ERROR_OK;
}

uint32_t arm946e_invalidate_icache(struct target *target, uint32_t address,
	uint32_t size, uint32_t count)
{
	uint32_t cur_addr = 0x0;
	uint32_t cp15_idx, set, way, itag;
	uint32_t i = 0;
	int retval;

	for (i = 0; i < count*size; i++) {
		cur_addr = address + i;

		set = (cur_addr >> 5) & 0xff;	/* set field is 8 bits long */

		for (way = 0; way < NB_CACHE_WAYS; way++) {
			/* Form and write cp15 index (segment + line idx) */
			cp15_idx = way << 30 | set << 5;
			retval = arm946e_write_cp15(target, 0x3a, cp15_idx);
			if (retval != ERROR_OK) {
				LOG_DEBUG("ERROR writing index");
				return retval;
			}

			/* Read itag */
			arm946e_read_cp15(target, 0x17, (uint32_t *) &itag);

			/* Check cache line VALID bit */
			if (!(itag >> 4 & 0x1))
				continue;

			/* If line is valid and corresponds to affected address - invalidate it */
			if (itag >> 5 == cur_addr >> 5) {
				/* Flush I$ line */
				retval = arm946e_write_cp15(target, 0x1d, 0x0);
				if (retval != ERROR_OK) {
					LOG_DEBUG("ERROR flushing cache line");
					return retval;
				}

				break;
			}
		}	/* way loop */
	}	/* addr loop */

	return ERROR_OK;
}

/** Writes a buffer, in the specified word size, with current MMU settings. */
int arm946e_write_memory(struct target *target, uint32_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int retval;

	LOG_DEBUG("-");

	/* Invalidate D$ if it is ON */
	if (!arm946e_preserve_cache && dc == 1)
		arm946e_invalidate_dcache(target, address, size, count);

	/**
	 * Write memory
	 */
	retval = arm7_9_write_memory(target, address, size, count, buffer);
	if (retval != ERROR_OK)
		return retval;

	/* *
	 * Invalidate I$ if it is ON.
	 *
	 * D$ has been cleaned and flushed before mem write thus forcing it to behave like write-through,
	 * because arm7_9_write_memory() has seen non-valid bit in D$
	 * and wrote data into physical RAM (without touching or allocating the cache line).
	 * From ARM946ES Technical Reference Manual we can see that it uses "allocate on read-miss"
	 * policy for both I$ and D$ (Chapter 3.2 and 3.3)
	 *
	 * Explanation :
	 * "ARM system developer's guide: designing and optimizing system software" by
	 * Andrew N. Sloss, Dominic Symes and Chris Wright,
	 * Chapter 12.3.3 Allocating Policy on a Cache Miss :
	 * A read allocate on cache miss policy allocates a cache line only during a read from main memory.
	 * If the victim cache line contains valid data, then it is written to main memory before the cache line
	 * is filled with new data.
	 * Under this strategy, a write of new data to memory does not update the contents of the cache memory
	 * unless a cache line was allocated on a previous read from main memory.
	 * If the cache line contains valid data, then the write updates the cache and may update the main memory if
	 * the cache write policy is write-through.
	 * If the data is not in the cache, the controller writes to main memory only.
	 */
	if (!arm946e_preserve_cache && ic == 1)
		arm946e_invalidate_icache(target, address, size, count);

	return ERROR_OK;

}

int arm946e_read_memory(struct target *target, uint32_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	int retval;

	LOG_DEBUG("-");

	retval = arm7_9_read_memory(target, address, size, count, buffer);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}


COMMAND_HANDLER(arm946e_handle_cp15_command)
{
	int retval;
	struct target *target = get_current_target(CMD_CTX);
	struct arm946e_common *arm946e = target_to_arm946(target);

	retval = arm946e_verify_pointer(CMD_CTX, arm946e);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED) {
		command_print(CMD_CTX, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

	/* one or more argument, access a single register (write if second argument is given */
	if (CMD_ARGC >= 1) {
		uint32_t address;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);

		if (CMD_ARGC == 1) {
			uint32_t value;
			retval = arm946e_read_cp15(target, address, &value);
			if (retval != ERROR_OK) {
				command_print(CMD_CTX, "couldn't access reg %" PRIi32, address);
				return ERROR_OK;
			}
			retval = jtag_execute_queue();
			if (retval != ERROR_OK)
				return retval;

			command_print(CMD_CTX, "%" PRIi32 ": %8.8" PRIx32, address, value);
		} else if (CMD_ARGC == 2) {
			uint32_t value;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);
			retval = arm946e_write_cp15(target, address, value);
			if (retval != ERROR_OK) {
				command_print(CMD_CTX, "couldn't access reg %" PRIi32, address);
				return ERROR_OK;
			}
			command_print(CMD_CTX, "%" PRIi32 ": %8.8" PRIx32, address, value);
		}
	}

	return ERROR_OK;
}

static const struct command_registration arm946e_exec_command_handlers[] = {
	{
		.name = "cp15",
		.handler = arm946e_handle_cp15_command,
		.mode = COMMAND_EXEC,
		.usage = "regnum [value]",
		.help = "display/modify cp15 register",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration arm946e_command_handlers[] = {
	{
		.chain = arm9tdmi_command_handlers,
	},
	{
		.name = "arm946e",
		.mode = COMMAND_ANY,
		.help = "arm946e command group",
		.usage = "",
		.chain = arm946e_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

/** Holds methods for ARM946 targets. */
struct target_type arm946e_target = {
	.name = "arm946e",

	.poll = arm7_9_poll,
	.arch_state = arm_arch_state,

	.target_request_data = arm7_9_target_request_data,

	.halt = arm7_9_halt,
	.resume = arm7_9_resume,
	.step = arm7_9_step,

	.assert_reset = arm7_9_assert_reset,
	.deassert_reset = arm7_9_deassert_reset,
	.soft_reset_halt = arm7_9_soft_reset_halt,

	.get_gdb_reg_list = arm_get_gdb_reg_list,

	/* .read_memory = arm7_9_read_memory, */
	/* .write_memory = arm7_9_write_memory, */
	.read_memory = arm946e_read_memory,
	.write_memory = arm946e_write_memory,

	.bulk_write_memory = arm7_9_bulk_write_memory,

	.checksum_memory = arm_checksum_memory,
	.blank_check_memory = arm_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = arm7_9_add_breakpoint,
	.remove_breakpoint = arm7_9_remove_breakpoint,
	/* .add_breakpoint = arm946e_add_breakpoint, */
	/* .remove_breakpoint = arm946e_remove_breakpoint, */

	.add_watchpoint = arm7_9_add_watchpoint,
	.remove_watchpoint = arm7_9_remove_watchpoint,

	.commands = arm946e_command_handlers,
	.target_create = arm946e_target_create,
	.init_target = arm9tdmi_init_target,
	.examine = arm7_9_examine,
	.check_reset = arm7_9_check_reset,
};
