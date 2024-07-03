// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2018 by Liviu Ionescu                                   *
 *   ilg@livius.net                                                        *
 *                                                                         *
 *   Copyright (C) 2009 by Marvell Technology Group Ltd.                   *
 *   Written by Nicolas Pitre <nico@marvell.com>                           *
 *                                                                         *
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2016 by Square, Inc.                                    *
 *   Steven Stallion <stallion@squareup.com>                               *
 ***************************************************************************/

/**
 * @file
 * Hold RISC-V semihosting support.
 *
 * The RISC-V code is inspired from ARM semihosting.
 *
 * Details can be found in chapter 8 of DUI0203I_rvct_developer_guide.pdf
 * from ARM Ltd.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>

#include "target/target.h"
#include "riscv.h"
#include "riscv_reg.h"

static int riscv_semihosting_setup(struct target *target, int enable);
static int riscv_semihosting_post_result(struct target *target);

/**
 * Initialize RISC-V semihosting. Use common ARM code.
 */
void riscv_semihosting_init(struct target *target)
{
	semihosting_common_init(target, riscv_semihosting_setup,
		riscv_semihosting_post_result);
}

/**
 * Check for and process a semihosting request using the ARM protocol). This
 * is meant to be called when the target is stopped due to a debug mode entry.
 *
 * @param target Pointer to the target to process.
 * @param retval Pointer to a location where the return code will be stored
 * @return non-zero value if a request was processed or an error encountered
 */
enum semihosting_result riscv_semihosting(struct target *target, int *retval)
{
	struct semihosting *semihosting = target->semihosting;
	if (!semihosting) {
		LOG_TARGET_DEBUG(target, "   -> NONE (!semihosting)");
		return SEMIHOSTING_NONE;
	}

	if (!semihosting->is_active) {
		LOG_TARGET_DEBUG(target, "   -> NONE (!semihosting->is_active)");
		return SEMIHOSTING_NONE;
	}

	riscv_reg_t pc;
	int result = riscv_reg_get(target, &pc, GDB_REGNO_PC);
	if (result != ERROR_OK)
		return SEMIHOSTING_ERROR;

	/*
	 * The instructions that trigger a semihosting call,
	 * always uncompressed, should look like:
	 */
	uint32_t magic[] = {
		0x01f01013,	/* slli    zero,zero,0x1f */
		0x00100073,	/* ebreak */
		0x40705013	/* srai    zero,zero,0x7 */
	};

	/* Read three uncompressed instructions: The previous, the current one (pointed to by PC) and the next one */
	for (int i = 0; i < 3; i++) {
		uint8_t buf[4];
		/* Instruction memories may not support arbitrary read size. Use any size that will work. */
		target_addr_t address = (pc - 4) + 4 * i;
		*retval = riscv_read_by_any_size(target, address, 4, buf);
		if (*retval != ERROR_OK)
			return SEMIHOSTING_ERROR;
		uint32_t value = target_buffer_get_u32(target, buf);
		LOG_TARGET_DEBUG(target, "compare 0x%08x from 0x%" PRIx64 " against 0x%08x",
			value, address, magic[i]);
		if (value != magic[i]) {
			LOG_TARGET_DEBUG(target, "   -> NONE (no magic)");
			return SEMIHOSTING_NONE;
		}
	}

	/*
	 * Perform semihosting call if we are not waiting on a fileio
	 * operation to complete.
	 */
	if (!semihosting->hit_fileio) {
		/* RISC-V uses A0 and A1 to pass function arguments */
		riscv_reg_t r0;
		riscv_reg_t r1;

		result = riscv_reg_get(target, &r0, GDB_REGNO_A0);
		if (result != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Could not read semihosting operation code (register a0)");
			return SEMIHOSTING_ERROR;
		}

		result = riscv_reg_get(target, &r1, GDB_REGNO_A1);
		if (result != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Could not read semihosting operation code (register a1)");
			return SEMIHOSTING_ERROR;
		}

		semihosting->op = r0;
		semihosting->param = r1;
		semihosting->word_size_bytes = riscv_xlen(target) / 8;

		/* Check for ARM operation numbers. */
		if ((semihosting->op >= 0 && semihosting->op <= 0x31) ||
			(semihosting->op >= 0x100 && semihosting->op <= 0x107)) {

			*retval = semihosting_common(target);
			if (*retval != ERROR_OK) {
				LOG_TARGET_ERROR(target, "Failed semihosting operation (0x%02X)", semihosting->op);
				return SEMIHOSTING_ERROR;
			}
		} else {
			/* Unknown operation number, not a semihosting call. */
			LOG_TARGET_ERROR(target, "Unknown semihosting operation requested (op = 0x%x)", semihosting->op);
			return SEMIHOSTING_NONE;
		}
	}

	/* Resume right after the EBREAK 4 bytes instruction. */
	*retval = riscv_reg_set(target, GDB_REGNO_PC, pc + 4);
	if (*retval != ERROR_OK)
		return SEMIHOSTING_ERROR;

	/*
	 * Resume target if we are not waiting on a fileio
	 * operation to complete.
	 */
	if (semihosting->is_resumable && !semihosting->hit_fileio) {
		LOG_TARGET_DEBUG(target, "   -> HANDLED");
		return SEMIHOSTING_HANDLED;
	}

	LOG_TARGET_DEBUG(target, "   -> WAITING");
	return SEMIHOSTING_WAITING;
}

/* -------------------------------------------------------------------------
 * Local functions. */

/**
 * Called via semihosting->setup() later, after the target is known,
 * usually on the first semihosting command.
 */
static int riscv_semihosting_setup(struct target *target, int enable)
{
	LOG_TARGET_DEBUG(target, "enable=%d", enable);

	struct semihosting *semihosting = target->semihosting;
	if (semihosting)
		semihosting->setup_time = clock();

	return ERROR_OK;
}

static int riscv_semihosting_post_result(struct target *target)
{
	struct semihosting *semihosting = target->semihosting;
	if (!semihosting) {
		/* If not enabled, silently ignored. */
		return 0;
	}

	LOG_TARGET_DEBUG(target, "Result: 0x%" PRIx64, semihosting->result);
	riscv_reg_set(target, GDB_REGNO_A0, semihosting->result);
	return 0;
}
