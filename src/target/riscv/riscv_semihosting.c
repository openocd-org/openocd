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

static int riscv_semihosting_detect_magic_sequence(struct target *target,
	const target_addr_t pc, bool *sequence_found)
{
	assert(sequence_found);

	/* The semihosting "magic" sequence must be the exact three instructions
	 * listed below. All these instructions, including the ebreak, must be
	 * uncompressed (4 bytes long). */
	const uint32_t magic[] = {
		0x01f01013,	/* slli    zero,zero,0x1f */
		0x00100073,	/* ebreak */
		0x40705013	/* srai    zero,zero,0x7 */
	};

	LOG_TARGET_DEBUG(target, "Checking for RISC-V semihosting sequence "
		"at PC = 0x%" TARGET_PRIxADDR, pc);

	/* Read three uncompressed instructions:
	 * The previous, the current one (pointed to by PC) and the next one. */
	const target_addr_t sequence_start_address = pc - 4;
	for (int i = 0; i < 3; i++) {
		uint8_t buf[4];

		/* Instruction memories may not support arbitrary read size.
		 * Use any size that will work. */
		const target_addr_t address = sequence_start_address + (4 * i);
		int result = riscv_read_by_any_size(target, address, 4, buf);
		if (result != ERROR_OK) {
			*sequence_found = false;
			return result;
		}

		/* RISC-V instruction layout in memory is always little endian,
		 * regardless of the endianness of the whole system. */
		const uint32_t value = le_to_h_u32(buf);

		LOG_TARGET_DEBUG(target, "compare 0x%08" PRIx32 " from 0x%" PRIx64 " against 0x%08" PRIx32,
			value, address, magic[i]);
		if (value != magic[i]) {
			LOG_TARGET_DEBUG(target, "Not a RISC-V semihosting sequence");
			*sequence_found = false;
			return ERROR_OK;
		}
	}

	LOG_TARGET_DEBUG(target, "RISC-V semihosting sequence found "
		"at PC = 0x%" TARGET_PRIxADDR, pc);
	*sequence_found = true;
	return ERROR_OK;
}

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
	assert(semihosting);

	riscv_reg_t pc;
	int result = riscv_reg_get(target, &pc, GDB_REGNO_PC);
	if (result != ERROR_OK) {
		LOG_TARGET_DEBUG(target, "Semihosting outcome: ERROR (failed to read PC)");
		return SEMIHOSTING_ERROR;
	}

	bool sequence_found = false;
	*retval = riscv_semihosting_detect_magic_sequence(target, pc, &sequence_found);
	if (*retval != ERROR_OK) {
		LOG_TARGET_DEBUG(target, "Semihosting outcome: ERROR (during magic seq. detection)");
		return SEMIHOSTING_ERROR;
	}

	if (!semihosting->is_active) {
		if (sequence_found) {
			// If semihositing is encountered but disabled, provide an additional hint to the user.
			LOG_TARGET_WARNING(target, "RISC-V semihosting call encountered in the program "
				"but semihosting is disabled!");
			LOG_TARGET_WARNING(target, "The target will remain halted (PC = 0x%" TARGET_PRIxADDR ").", pc);
			LOG_TARGET_WARNING(target, "Hint: Restart your debug session and enable semihosting "
				"by command 'arm semihosting enable'.");
			// TODO: This can be improved: The ebreak halt cause detection and riscv_semihosting() call
			// can be added also to "arm semihosting enable", which would allow the user to continue
			// without restart of the debug session.
		}

		LOG_TARGET_DEBUG(target, "Semihosting outcome: NONE (semihosting not enabled)");
		return SEMIHOSTING_NONE;
	}

	if (!sequence_found) {
		LOG_TARGET_DEBUG(target, "Semihosting outcome: NONE (no magic sequence)");
		return SEMIHOSTING_NONE;
	}

	/* Otherwise we have a semihosting call (and semihosting is enabled).
	 * Proceed with the handling of semihosting. */

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
			LOG_TARGET_DEBUG(target, "Semihosting outcome: ERROR (failed to read a0)");
			return SEMIHOSTING_ERROR;
		}

		result = riscv_reg_get(target, &r1, GDB_REGNO_A1);
		if (result != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Could not read semihosting operation code (register a1)");
			LOG_TARGET_DEBUG(target, "Semihosting outcome: ERROR (failed to read a1)");
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
				LOG_TARGET_DEBUG(target, "Semihosting outcome: ERROR (error during semihosting processing)");
				return SEMIHOSTING_ERROR;
			}
		} else {
			/* Unknown operation number, not a semihosting call. */
			LOG_TARGET_ERROR(target, "Unknown semihosting operation requested (op = 0x%x)", semihosting->op);
			LOG_TARGET_DEBUG(target, "Semihosting outcome: NONE (unknown semihosting opcode)");
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
		LOG_TARGET_DEBUG(target, "Semihosting outcome: HANDLED");
		return SEMIHOSTING_HANDLED;
	}

	LOG_TARGET_DEBUG(target, "Semihosting outcome: WAITING");
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
	assert(semihosting);

	semihosting->setup_time = clock();
	return ERROR_OK;
}

static int riscv_semihosting_post_result(struct target *target)
{
	struct semihosting *semihosting = target->semihosting;
	assert(semihosting);

	LOG_TARGET_DEBUG(target, "Result: 0x%" PRIx64, semihosting->result);
	riscv_reg_set(target, GDB_REGNO_A0, semihosting->result);
	return 0;
}
