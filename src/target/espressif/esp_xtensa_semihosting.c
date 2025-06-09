// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (c) 2020 Espressif Systems (Shanghai) Co. Ltd.              *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/semihosting_common.h>
#include <target/xtensa/xtensa_regs.h>
#include <target/xtensa/xtensa.h>
#include "esp_xtensa.h"
#include "esp_xtensa_semihosting.h"

#define ESP_XTENSA_SYSCALL              0x41E0	/* XT_INS_BREAK(1, 14) */
#define ESP_XTENSA_SYSCALL_SZ           3

#define XTENSA_SYSCALL_OP_REG           XT_REG_IDX_A2
#define XTENSA_SYSCALL_RETVAL_REG       XT_REG_IDX_A2
#define XTENSA_SYSCALL_ERRNO_REG        XT_REG_IDX_A3

static int esp_xtensa_semihosting_setup(struct target *target, int enable)
{
	LOG_TARGET_DEBUG(target, "semihosting enable=%d", enable);

	return ERROR_OK;
}

static int esp_xtensa_semihosting_post_result(struct target *target)
{
	/* Even with the v2 and later, errno will not retrieved from A3 reg, it is safe to set */
	xtensa_reg_set(target, XTENSA_SYSCALL_RETVAL_REG, target->semihosting->result);
	xtensa_reg_set(target, XTENSA_SYSCALL_ERRNO_REG, target->semihosting->sys_errno);
	return ERROR_OK;
}

/**
 * Checks and processes an ESP Xtensa semihosting request. This is meant
 * to be called when the target is stopped due to a debug mode entry.
 * If the value 0 is returned then there was nothing to process. A non-zero
 * return value signifies that a request was processed and the target resumed,
 * or an error was encountered, in which case the caller must return immediately.
 *
 * @param target Pointer to the ESP Xtensa target to process.
 * @param retval Pointer to a location where the return code will be stored
 * @return SEMIHOSTING_HANDLED if a request was processed or SEMIHOSTING_NONE with the proper retval
 */
int esp_xtensa_semihosting(struct target *target, int *retval)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);

	xtensa_reg_val_t dbg_cause = xtensa_reg_get(target, XT_REG_IDX_DEBUGCAUSE);
	if ((dbg_cause & (DEBUGCAUSE_BI | DEBUGCAUSE_BN)) == 0)
		return SEMIHOSTING_NONE;

	uint8_t brk_insn_buf[sizeof(uint32_t)] = { 0 };
	xtensa_reg_val_t pc = xtensa_reg_get(target, XT_REG_IDX_PC);
	*retval = target_read_memory(target, pc, ESP_XTENSA_SYSCALL_SZ, 1, brk_insn_buf);
	if (*retval != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read break instruction!");
		return SEMIHOSTING_NONE;
	}

	uint32_t syscall_ins = buf_get_u32(brk_insn_buf, 0, 32);
	if (syscall_ins != ESP_XTENSA_SYSCALL) {
		*retval = ERROR_OK;
		return SEMIHOSTING_NONE;
	}

	if (esp_xtensa->semihost.ops && esp_xtensa->semihost.ops->prepare)
		esp_xtensa->semihost.ops->prepare(target);

	xtensa_reg_val_t a2 = xtensa_reg_get(target, XT_REG_IDX_A2);
	xtensa_reg_val_t a3 = xtensa_reg_get(target, XT_REG_IDX_A3);
	LOG_TARGET_DEBUG(target, "Semihosting call 0x%" PRIx32 " 0x%" PRIx32 " Base dir '%s'",
		a2,
		a3,
		target->semihosting->basedir ? target->semihosting->basedir : "");

	target->semihosting->op = a2;
	target->semihosting->param = a3;

	*retval = semihosting_common(target);

	/* Most operations are resumable, except the two exit calls. */
	if (*retval != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Semihosting operation (op: 0x%x) error! Code: %d",
			target->semihosting->op,
			*retval);
	}

	/* Resume if target it is resumable and we are not waiting on a fileio operation to complete. */
	if (target->semihosting->is_resumable && !target->semihosting->hit_fileio)
		target_to_esp_xtensa(target)->semihost.need_resume = true;

	return SEMIHOSTING_HANDLED;
}

static int xtensa_semihosting_init(struct target *target)
{
	return semihosting_common_init(target, esp_xtensa_semihosting_setup, esp_xtensa_semihosting_post_result);
}

int esp_xtensa_semihosting_init(struct target *target)
{
	int retval = xtensa_semihosting_init(target);
	if (retval != ERROR_OK)
		return retval;
	target->semihosting->word_size_bytes = 4;			/* 32 bits */
	target->semihosting->user_command_extension = esp_semihosting_common;
	return ERROR_OK;
}
