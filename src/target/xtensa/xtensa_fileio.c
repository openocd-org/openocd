// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Xtensa Target File-I/O Support for OpenOCD                            *
 *   Copyright (C) 2020-2023 Cadence Design Systems, Inc.                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "xtensa_chip.h"
#include "xtensa_fileio.h"
#include "xtensa.h"

#define XTENSA_SYSCALL(x)		XT_INS_BREAK(x, 1, 14)
#define XTENSA_SYSCALL_SZ		3
#define XTENSA_SYSCALL_LEN_MAX	255


int xtensa_fileio_init(struct target *target)
{
	char *idmem = malloc(XTENSA_SYSCALL_LEN_MAX + 1);
	target->fileio_info = malloc(sizeof(struct gdb_fileio_info));
	if (!idmem || !target->fileio_info) {
		LOG_TARGET_ERROR(target, "Out of memory!");
		free(idmem);
		free(target->fileio_info);
		return ERROR_FAIL;
	}
	target->fileio_info->identifier = idmem;
	return ERROR_OK;
}

/**
 * Checks for and processes an Xtensa File-IO request.
 *
 * Return ERROR_OK if request was found and handled; or
 * return ERROR_FAIL if no request was detected.
 */
int xtensa_fileio_detect_proc(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int retval;

	xtensa_reg_val_t dbg_cause = xtensa_cause_get(target);
	if ((dbg_cause & (DEBUGCAUSE_BI | DEBUGCAUSE_BN)) == 0 || xtensa->halt_request)
		return ERROR_FAIL;

	uint8_t brk_insn_buf[sizeof(uint32_t)] = {0};
	xtensa_reg_val_t pc = xtensa_reg_get(target, XT_REG_IDX_PC);
	retval = target_read_memory(target,
		pc,
		XTENSA_SYSCALL_SZ,
		1,
		(uint8_t *)brk_insn_buf);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read break instruction!");
		return ERROR_FAIL;
	}
	if (buf_get_u32(brk_insn_buf, 0, 32) != XTENSA_SYSCALL(xtensa))
		return ERROR_FAIL;

	LOG_TARGET_DEBUG(target, "File-I/O: syscall breakpoint found at 0x%x", pc);
	xtensa->proc_syscall = true;
	return ERROR_OK;
}

int xtensa_get_gdb_fileio_info(struct target *target, struct gdb_fileio_info *fileio_info)
{
	/* fill syscall parameters to file-I/O info */
	if (!fileio_info) {
		LOG_ERROR("File-I/O data structure uninitialized");
		return ERROR_FAIL;
	}

	struct xtensa *xtensa = target_to_xtensa(target);
	if (!xtensa->proc_syscall)
		return ERROR_FAIL;

	xtensa_reg_val_t syscall = xtensa_reg_get(target, XTENSA_SYSCALL_OP_REG);
	xtensa_reg_val_t arg0 = xtensa_reg_get(target, XT_REG_IDX_A6);
	xtensa_reg_val_t arg1 = xtensa_reg_get(target, XT_REG_IDX_A3);
	xtensa_reg_val_t arg2 = xtensa_reg_get(target, XT_REG_IDX_A4);
	xtensa_reg_val_t arg3 = xtensa_reg_get(target, XT_REG_IDX_A5);
	int retval = ERROR_OK;

	LOG_TARGET_DEBUG(target, "File-I/O: syscall 0x%x 0x%x 0x%x 0x%x 0x%x",
		syscall, arg0, arg1, arg2, arg3);

	switch (syscall) {
	case XTENSA_SYSCALL_OPEN:
		snprintf(fileio_info->identifier, XTENSA_SYSCALL_LEN_MAX, "open");
		fileio_info->param_1 = arg0;	// pathp
		fileio_info->param_2 = arg3;	// len
		fileio_info->param_3 = arg1;	// flags
		fileio_info->param_4 = arg2;	// mode
		break;
	case XTENSA_SYSCALL_CLOSE:
		snprintf(fileio_info->identifier, XTENSA_SYSCALL_LEN_MAX, "close");
		fileio_info->param_1 = arg0;	// fd
		break;
	case XTENSA_SYSCALL_READ:
		snprintf(fileio_info->identifier, XTENSA_SYSCALL_LEN_MAX, "read");
		fileio_info->param_1 = arg0;	// fd
		fileio_info->param_2 = arg1;	// bufp
		fileio_info->param_3 = arg2;	// count
		break;
	case XTENSA_SYSCALL_WRITE:
		snprintf(fileio_info->identifier, XTENSA_SYSCALL_LEN_MAX, "write");
		fileio_info->param_1 = arg0;	// fd
		fileio_info->param_2 = arg1;	// bufp
		fileio_info->param_3 = arg2;	// count
		break;
	case XTENSA_SYSCALL_LSEEK:
		snprintf(fileio_info->identifier, XTENSA_SYSCALL_LEN_MAX, "lseek");
		fileio_info->param_1 = arg0;	// fd
		fileio_info->param_2 = arg1;	// offset
		fileio_info->param_3 = arg2;	// flags
		break;
	case XTENSA_SYSCALL_RENAME:
		snprintf(fileio_info->identifier, XTENSA_SYSCALL_LEN_MAX, "rename");
		fileio_info->param_1 = arg0;	// old pathp
		fileio_info->param_2 = arg3;	// old len
		fileio_info->param_3 = arg1;	// new pathp
		fileio_info->param_4 = arg2;	// new len
		break;
	case XTENSA_SYSCALL_UNLINK:
		snprintf(fileio_info->identifier, XTENSA_SYSCALL_LEN_MAX, "unlink");
		fileio_info->param_1 = arg0;	// pathnamep
		fileio_info->param_2 = arg1;	// len
		break;
	case XTENSA_SYSCALL_STAT:
		snprintf(fileio_info->identifier, XTENSA_SYSCALL_LEN_MAX, "stat");
		fileio_info->param_1 = arg0;	// pathnamep
		fileio_info->param_2 = arg2;	// len
		fileio_info->param_3 = arg1;	// bufp
		break;
	case XTENSA_SYSCALL_FSTAT:
		snprintf(fileio_info->identifier, XTENSA_SYSCALL_LEN_MAX, "fstat");
		fileio_info->param_1 = arg0;	// fd
		fileio_info->param_2 = arg1;	// bufp
		break;
	case XTENSA_SYSCALL_GETTIMEOFDAY:
		snprintf(fileio_info->identifier, XTENSA_SYSCALL_LEN_MAX, "gettimeofday");
		fileio_info->param_1 = arg0;	// tvp
		fileio_info->param_2 = arg1;	// tzp
		break;
	case XTENSA_SYSCALL_ISATTY:
		snprintf(fileio_info->identifier, XTENSA_SYSCALL_LEN_MAX, "isatty");
		fileio_info->param_1 = arg0;	// fd
		break;
	case XTENSA_SYSCALL_SYSTEM:
		snprintf(fileio_info->identifier, XTENSA_SYSCALL_LEN_MAX, "system");
		fileio_info->param_1 = arg0;	// cmdp
		fileio_info->param_2 = arg1;	// len
		break;
	default:
		snprintf(fileio_info->identifier, XTENSA_SYSCALL_LEN_MAX, "unknown");
		LOG_TARGET_DEBUG(target, "File-I/O: syscall unknown (%d), pc=0x%08X",
			syscall, xtensa_reg_get(target, XT_REG_IDX_PC));
		LOG_INFO("File-I/O: syscall unknown (%d), pc=0x%08X",
			syscall, xtensa_reg_get(target, XT_REG_IDX_PC));
		retval = ERROR_FAIL;
		break;
	}

	return retval;
}

int xtensa_gdb_fileio_end(struct target *target, int retcode, int fileio_errno, bool ctrl_c)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	if (!xtensa->proc_syscall)
		return ERROR_FAIL;

	LOG_TARGET_DEBUG(target, "File-I/O: syscall return code: 0x%x, errno: 0x%x , ctrl_c: %s",
		retcode, fileio_errno, ctrl_c ? "true" : "false");

	/* If interrupt was requested before FIO completion (ERRNO==4), halt and repeat
	 * syscall. Otherwise, set File-I/O Ax and underlying ARx registers, increment PC.
	 * NOTE: sporadic cases of ((ERRNO==4) && !ctrl_c) were observed; most have ctrl_c.
	 */
	if (fileio_errno != 4) {
		xtensa_reg_set_deep_relgen(target, XTENSA_SYSCALL_RETVAL_REG, retcode);
		xtensa_reg_set_deep_relgen(target, XTENSA_SYSCALL_ERRNO_REG, fileio_errno);

		xtensa_reg_val_t pc = xtensa_reg_get(target, XT_REG_IDX_PC);
		xtensa_reg_set(target, XT_REG_IDX_PC, pc + XTENSA_SYSCALL_SZ);
	}

	xtensa->proc_syscall = false;
	xtensa->halt_request = true;
	return ctrl_c ? ERROR_FAIL : ERROR_OK;
}
