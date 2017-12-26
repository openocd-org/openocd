/***************************************************************************
 *   Copyright (C) 2009 by Marvell Technology Group Ltd.                   *
 *   Written by Nicolas Pitre <nico@marvell.com>                           *
 *                                                                         *
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2016 by Square, Inc.                                    *
 *   Steven Stallion <stallion@squareup.com>                               *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

/**
 * @file
 * Hold ARM semihosting support.
 *
 * Semihosting enables code running on an ARM target to use the I/O
 * facilities on the host computer. The target application must be linked
 * against a library that forwards operation requests by using the SVC
 * instruction trapped at the Supervisor Call vector by the debugger.
 * Details can be found in chapter 8 of DUI0203I_rvct_developer_guide.pdf
 * from ARM Ltd.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm.h"
#include "armv4_5.h"
#include "arm7_9_common.h"
#include "armv7m.h"
#include "armv7a.h"
#include "cortex_m.h"
#include "register.h"
#include "arm_opcodes.h"
#include "target_type.h"
#include "arm_semihosting.h"
#include <helper/binarybuffer.h>
#include <helper/log.h>
#include <sys/stat.h>

static const int open_modeflags[12] = {
	O_RDONLY,
	O_RDONLY | O_BINARY,
	O_RDWR,
	O_RDWR | O_BINARY,
	O_WRONLY | O_CREAT | O_TRUNC,
	O_WRONLY | O_CREAT | O_TRUNC | O_BINARY,
	O_RDWR | O_CREAT | O_TRUNC,
	O_RDWR | O_CREAT | O_TRUNC | O_BINARY,
	O_WRONLY | O_CREAT | O_APPEND,
	O_WRONLY | O_CREAT | O_APPEND | O_BINARY,
	O_RDWR | O_CREAT | O_APPEND,
	O_RDWR | O_CREAT | O_APPEND | O_BINARY
};

static int post_result(struct target *target)
{
	struct arm *arm = target_to_arm(target);

	/* REVISIT this looks wrong ... ARM11 and Cortex-A8
	 * should work this way at least sometimes.
	 */
	if (is_arm7_9(target_to_arm7_9(target)) ||
	    is_armv7a(target_to_armv7a(target))) {
		uint32_t spsr;

		/* return value in R0 */
		buf_set_u32(arm->core_cache->reg_list[0].value, 0, 32, arm->semihosting_result);
		arm->core_cache->reg_list[0].dirty = 1;

		/* LR --> PC */
		buf_set_u32(arm->core_cache->reg_list[15].value, 0, 32,
			buf_get_u32(arm_reg_current(arm, 14)->value, 0, 32));
		arm->core_cache->reg_list[15].dirty = 1;

		/* saved PSR --> current PSR */
		spsr = buf_get_u32(arm->spsr->value, 0, 32);

		/* REVISIT should this be arm_set_cpsr(arm, spsr)
		 * instead of a partially unrolled version?
		 */

		buf_set_u32(arm->cpsr->value, 0, 32, spsr);
		arm->cpsr->dirty = 1;
		arm->core_mode = spsr & 0x1f;
		if (spsr & 0x20)
			arm->core_state = ARM_STATE_THUMB;

	} else {
		/* resume execution, this will be pc+2 to skip over the
		 * bkpt instruction */

		/* return result in R0 */
		buf_set_u32(arm->core_cache->reg_list[0].value, 0, 32, arm->semihosting_result);
		arm->core_cache->reg_list[0].dirty = 1;
	}

	return ERROR_OK;
}

static int do_semihosting(struct target *target)
{
	struct arm *arm = target_to_arm(target);
	struct gdb_fileio_info *fileio_info = target->fileio_info;
	uint32_t r0 = buf_get_u32(arm->core_cache->reg_list[0].value, 0, 32);
	uint32_t r1 = buf_get_u32(arm->core_cache->reg_list[1].value, 0, 32);
	uint8_t params[16];
	int retval;

	/*
	 * TODO: lots of security issues are not considered yet, such as:
	 * - no validation on target provided file descriptors
	 * - no safety checks on opened/deleted/renamed file paths
	 * Beware the target app you use this support with.
	 *
	 * TODO: unsupported semihosting fileio operations could be
	 * implemented if we had a small working area at our disposal.
	 */
	switch ((arm->semihosting_op = r0)) {
	case 0x01:	/* SYS_OPEN */
		retval = target_read_memory(target, r1, 4, 3, params);
		if (retval != ERROR_OK)
			return retval;
		else {
			uint32_t a = target_buffer_get_u32(target, params+0);
			uint32_t m = target_buffer_get_u32(target, params+4);
			uint32_t l = target_buffer_get_u32(target, params+8);
			uint8_t fn[256];
			retval = target_read_memory(target, a, 1, l, fn);
			if (retval != ERROR_OK)
				return retval;
			fn[l] = 0;
			if (arm->is_semihosting_fileio) {
				if (strcmp((char *)fn, ":tt") == 0)
					arm->semihosting_result = 0;
				else {
					arm->semihosting_hit_fileio = true;
					fileio_info->identifier = "open";
					fileio_info->param_1 = a;
					fileio_info->param_2 = l;
					fileio_info->param_3 = open_modeflags[m];
					fileio_info->param_4 = 0644;
				}
			} else {
				if (l <= 255 && m <= 11) {
					if (strcmp((char *)fn, ":tt") == 0) {
						if (m < 4)
							arm->semihosting_result = dup(STDIN_FILENO);
						else
							arm->semihosting_result = dup(STDOUT_FILENO);
					} else {
						/* cygwin requires the permission setting
						 * otherwise it will fail to reopen a previously
						 * written file */
						arm->semihosting_result = open((char *)fn, open_modeflags[m], 0644);
					}
					arm->semihosting_errno =  errno;
				} else {
					arm->semihosting_result = -1;
					arm->semihosting_errno = EINVAL;
				}
			}
		}
		break;

	case 0x02:	/* SYS_CLOSE */
		retval = target_read_memory(target, r1, 4, 1, params);
		if (retval != ERROR_OK)
			return retval;
		else {
			int fd = target_buffer_get_u32(target, params+0);
			if (arm->is_semihosting_fileio) {
				arm->semihosting_hit_fileio = true;
				fileio_info->identifier = "close";
				fileio_info->param_1 = fd;
			} else {
				arm->semihosting_result = close(fd);
				arm->semihosting_errno = errno;
			}
		}
		break;

	case 0x03:	/* SYS_WRITEC */
		if (arm->is_semihosting_fileio) {
			arm->semihosting_hit_fileio = true;
			fileio_info->identifier = "write";
			fileio_info->param_1 = 1;
			fileio_info->param_2 = r1;
			fileio_info->param_3 = 1;
		} else {
			unsigned char c;
			retval = target_read_memory(target, r1, 1, 1, &c);
			if (retval != ERROR_OK)
				return retval;
			putchar(c);
			arm->semihosting_result = 0;
		}
		break;

	case 0x04:	/* SYS_WRITE0 */
		if (arm->is_semihosting_fileio) {
			size_t count = 0;
			for (uint32_t a = r1;; a++) {
				unsigned char c;
				retval = target_read_memory(target, a, 1, 1, &c);
				if (retval != ERROR_OK)
					return retval;
				if (c == '\0')
					break;
				count++;
			}
			arm->semihosting_hit_fileio = true;
			fileio_info->identifier = "write";
			fileio_info->param_1 = 1;
			fileio_info->param_2 = r1;
			fileio_info->param_3 = count;
		} else {
			do {
				unsigned char c;
				retval = target_read_memory(target, r1++, 1, 1, &c);
				if (retval != ERROR_OK)
					return retval;
				if (!c)
					break;
				putchar(c);
			} while (1);
			arm->semihosting_result = 0;
		}
		break;

	case 0x05:	/* SYS_WRITE */
		retval = target_read_memory(target, r1, 4, 3, params);
		if (retval != ERROR_OK)
			return retval;
		else {
			int fd = target_buffer_get_u32(target, params+0);
			uint32_t a = target_buffer_get_u32(target, params+4);
			size_t l = target_buffer_get_u32(target, params+8);
			if (arm->is_semihosting_fileio) {
				arm->semihosting_hit_fileio = true;
				fileio_info->identifier = "write";
				fileio_info->param_1 = fd;
				fileio_info->param_2 = a;
				fileio_info->param_3 = l;
			} else {
				uint8_t *buf = malloc(l);
				if (!buf) {
					arm->semihosting_result = -1;
					arm->semihosting_errno = ENOMEM;
				} else {
					retval = target_read_buffer(target, a, l, buf);
					if (retval != ERROR_OK) {
						free(buf);
						return retval;
					}
					arm->semihosting_result = write(fd, buf, l);
					arm->semihosting_errno = errno;
					if (arm->semihosting_result >= 0)
						arm->semihosting_result = l - arm->semihosting_result;
					free(buf);
				}
			}
		}
		break;

	case 0x06:	/* SYS_READ */
		retval = target_read_memory(target, r1, 4, 3, params);
		if (retval != ERROR_OK)
			return retval;
		else {
			int fd = target_buffer_get_u32(target, params+0);
			uint32_t a = target_buffer_get_u32(target, params+4);
			ssize_t l = target_buffer_get_u32(target, params+8);
			if (arm->is_semihosting_fileio) {
				arm->semihosting_hit_fileio = true;
				fileio_info->identifier = "read";
				fileio_info->param_1 = fd;
				fileio_info->param_2 = a;
				fileio_info->param_3 = l;
			} else {
				uint8_t *buf = malloc(l);
				if (!buf) {
					arm->semihosting_result = -1;
					arm->semihosting_errno = ENOMEM;
				} else {
					arm->semihosting_result = read(fd, buf, l);
					arm->semihosting_errno = errno;
					if (arm->semihosting_result >= 0) {
						retval = target_write_buffer(target, a, arm->semihosting_result, buf);
						if (retval != ERROR_OK) {
							free(buf);
							return retval;
						}
						arm->semihosting_result = l - arm->semihosting_result;
					}
					free(buf);
				}
			}
		}
		break;

	case 0x07:	/* SYS_READC */
		if (arm->is_semihosting_fileio) {
			LOG_ERROR("SYS_READC not supported by semihosting fileio");
			return ERROR_FAIL;
		}
		arm->semihosting_result = getchar();
		break;

	case 0x08:	/* SYS_ISERROR */
		retval = target_read_memory(target, r1, 4, 1, params);
		if (retval != ERROR_OK)
			return retval;
		arm->semihosting_result = (target_buffer_get_u32(target, params+0) != 0);
		break;

	case 0x09:	/* SYS_ISTTY */
		if (arm->is_semihosting_fileio) {
			arm->semihosting_hit_fileio = true;
			fileio_info->identifier = "isatty";
			fileio_info->param_1 = r1;
		} else {
			retval = target_read_memory(target, r1, 4, 1, params);
			if (retval != ERROR_OK)
				return retval;
			arm->semihosting_result = isatty(target_buffer_get_u32(target, params+0));
		}
		break;

	case 0x0a:	/* SYS_SEEK */
		retval = target_read_memory(target, r1, 4, 2, params);
		if (retval != ERROR_OK)
			return retval;
		else {
			int fd = target_buffer_get_u32(target, params+0);
			off_t pos = target_buffer_get_u32(target, params+4);
			if (arm->is_semihosting_fileio) {
				arm->semihosting_hit_fileio = true;
				fileio_info->identifier = "lseek";
				fileio_info->param_1 = fd;
				fileio_info->param_2 = pos;
				fileio_info->param_3 = SEEK_SET;
			} else {
				arm->semihosting_result = lseek(fd, pos, SEEK_SET);
				arm->semihosting_errno = errno;
				if (arm->semihosting_result == pos)
					arm->semihosting_result = 0;
			}
		}
		break;

	case 0x0c:	/* SYS_FLEN */
		if (arm->is_semihosting_fileio) {
			LOG_ERROR("SYS_FLEN not supported by semihosting fileio");
			return ERROR_FAIL;
		}
		retval = target_read_memory(target, r1, 4, 1, params);
		if (retval != ERROR_OK)
			return retval;
		else {
			int fd = target_buffer_get_u32(target, params+0);
			struct stat buf;
			arm->semihosting_result = fstat(fd, &buf);
			if (arm->semihosting_result == -1) {
				arm->semihosting_errno = errno;
				arm->semihosting_result = -1;
				break;
			}
			arm->semihosting_result = buf.st_size;
		}
		break;

	case 0x0e:	/* SYS_REMOVE */
		retval = target_read_memory(target, r1, 4, 2, params);
		if (retval != ERROR_OK)
			return retval;
		else {
			uint32_t a = target_buffer_get_u32(target, params+0);
			uint32_t l = target_buffer_get_u32(target, params+4);
			if (arm->is_semihosting_fileio) {
				arm->semihosting_hit_fileio = true;
				fileio_info->identifier = "unlink";
				fileio_info->param_1 = a;
				fileio_info->param_2 = l;
			} else {
				if (l <= 255) {
					uint8_t fn[256];
					retval = target_read_memory(target, a, 1, l, fn);
					if (retval != ERROR_OK)
						return retval;
					fn[l] = 0;
					arm->semihosting_result = remove((char *)fn);
					arm->semihosting_errno =  errno;
				} else {
					arm->semihosting_result = -1;
					arm->semihosting_errno = EINVAL;
				}
			}
		}
		break;

	case 0x0f:	/* SYS_RENAME */
		retval = target_read_memory(target, r1, 4, 4, params);
		if (retval != ERROR_OK)
			return retval;
		else {
			uint32_t a1 = target_buffer_get_u32(target, params+0);
			uint32_t l1 = target_buffer_get_u32(target, params+4);
			uint32_t a2 = target_buffer_get_u32(target, params+8);
			uint32_t l2 = target_buffer_get_u32(target, params+12);
			if (arm->is_semihosting_fileio) {
				arm->semihosting_hit_fileio = true;
				fileio_info->identifier = "rename";
				fileio_info->param_1 = a1;
				fileio_info->param_2 = l1;
				fileio_info->param_3 = a2;
				fileio_info->param_4 = l2;
			} else {
				if (l1 <= 255 && l2 <= 255) {
					uint8_t fn1[256], fn2[256];
					retval = target_read_memory(target, a1, 1, l1, fn1);
					if (retval != ERROR_OK)
						return retval;
					retval = target_read_memory(target, a2, 1, l2, fn2);
					if (retval != ERROR_OK)
						return retval;
					fn1[l1] = 0;
					fn2[l2] = 0;
					arm->semihosting_result = rename((char *)fn1, (char *)fn2);
					arm->semihosting_errno =  errno;
				} else {
					arm->semihosting_result = -1;
					arm->semihosting_errno = EINVAL;
				}
			}
		}
		break;

	case 0x11:	/* SYS_TIME */
		arm->semihosting_result = time(NULL);
		break;

	case 0x13:	/* SYS_ERRNO */
		arm->semihosting_result = arm->semihosting_errno;
		break;

	case 0x15:	/* SYS_GET_CMDLINE */
		retval = target_read_memory(target, r1, 4, 2, params);
		if (retval != ERROR_OK)
			return retval;
		else {
			uint32_t a = target_buffer_get_u32(target, params+0);
			uint32_t l = target_buffer_get_u32(target, params+4);
			char *arg = arm->semihosting_cmdline != NULL ? arm->semihosting_cmdline : "";
			uint32_t s = strlen(arg) + 1;
			if (l < s)
				arm->semihosting_result = -1;
			else {
				retval = target_write_buffer(target, a, s, (uint8_t *)arg);
				if (retval != ERROR_OK)
					return retval;
				arm->semihosting_result = 0;
			}
		}
		break;

	case 0x16:	/* SYS_HEAPINFO */
		retval = target_read_memory(target, r1, 4, 1, params);
		if (retval != ERROR_OK)
			return retval;
		else {
			uint32_t a = target_buffer_get_u32(target, params+0);
			/* tell the remote we have no idea */
			memset(params, 0, 4*4);
			retval = target_write_memory(target, a, 4, 4, params);
			if (retval != ERROR_OK)
				return retval;
			arm->semihosting_result = 0;
		}
		break;

	case 0x18:	/* angel_SWIreason_ReportException */
		switch (r1) {
		case 0x20026:	/* ADP_Stopped_ApplicationExit */
			fprintf(stderr, "semihosting: *** application exited ***\n");
			break;
		case 0x20000:	/* ADP_Stopped_BranchThroughZero */
		case 0x20001:	/* ADP_Stopped_UndefinedInstr */
		case 0x20002:	/* ADP_Stopped_SoftwareInterrupt */
		case 0x20003:	/* ADP_Stopped_PrefetchAbort */
		case 0x20004:	/* ADP_Stopped_DataAbort */
		case 0x20005:	/* ADP_Stopped_AddressException */
		case 0x20006:	/* ADP_Stopped_IRQ */
		case 0x20007:	/* ADP_Stopped_FIQ */
		case 0x20020:	/* ADP_Stopped_BreakPoint */
		case 0x20021:	/* ADP_Stopped_WatchPoint */
		case 0x20022:	/* ADP_Stopped_StepComplete */
		case 0x20023:	/* ADP_Stopped_RunTimeErrorUnknown */
		case 0x20024:	/* ADP_Stopped_InternalError */
		case 0x20025:	/* ADP_Stopped_UserInterruption */
		case 0x20027:	/* ADP_Stopped_StackOverflow */
		case 0x20028:	/* ADP_Stopped_DivisionByZero */
		case 0x20029:	/* ADP_Stopped_OSSpecific */
		default:
			fprintf(stderr, "semihosting: exception %#x\n",
					(unsigned) r1);
		}
		return target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	case 0x12:	/* SYS_SYSTEM */
		/* Provide SYS_SYSTEM functionality.  Uses the
		 * libc system command, there may be a reason *NOT*
		 * to use this, but as I can't think of one, I
		 * implemented it this way.
		 */
		retval = target_read_memory(target, r1, 4, 2, params);
		if (retval != ERROR_OK)
			return retval;
		else {
			uint32_t len = target_buffer_get_u32(target, params+4);
			uint32_t c_ptr = target_buffer_get_u32(target, params);
			if (arm->is_semihosting_fileio) {
				arm->semihosting_hit_fileio = true;
				fileio_info->identifier = "system";
				fileio_info->param_1 = c_ptr;
				fileio_info->param_2 = len;
			} else {
				uint8_t cmd[256];
				if (len > 255) {
					arm->semihosting_result = -1;
					arm->semihosting_errno = EINVAL;
				} else {
					memset(cmd, 0x0, 256);
					retval = target_read_memory(target, c_ptr, 1, len, cmd);
					if (retval != ERROR_OK)
						return retval;
					else
						arm->semihosting_result = system((const char *)cmd);
				}
			}
		}
		break;
	case 0x0d:	/* SYS_TMPNAM */
	case 0x10:	/* SYS_CLOCK */
	case 0x17:	/* angel_SWIreason_EnterSVC */
	case 0x30:	/* SYS_ELAPSED */
	case 0x31:	/* SYS_TICKFREQ */
	default:
		fprintf(stderr, "semihosting: unsupported call %#x\n",
				(unsigned) r0);
		arm->semihosting_result = -1;
		arm->semihosting_errno = ENOTSUP;
	}

	return ERROR_OK;
}

static int get_gdb_fileio_info(struct target *target, struct gdb_fileio_info *fileio_info)
{
	struct arm *arm = target_to_arm(target);

	/* To avoid uneccessary duplication, semihosting prepares the
	 * fileio_info structure out-of-band when the target halts. See
	 * do_semihosting for more detail.
	 */
	if (!arm->is_semihosting_fileio || !arm->semihosting_hit_fileio)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int gdb_fileio_end(struct target *target, int result, int fileio_errno, bool ctrl_c)
{
	struct arm *arm = target_to_arm(target);
	struct gdb_fileio_info *fileio_info = target->fileio_info;

	/* clear pending status */
	arm->semihosting_hit_fileio = false;

	arm->semihosting_result = result;
	arm->semihosting_errno = fileio_errno;

	/* Some fileio results do not match up with what the semihosting
	 * operation expects; for these operations, we munge the results
	 * below:
	 */
	switch (arm->semihosting_op) {
	case 0x05:	/* SYS_WRITE */
		if (result < 0)
			arm->semihosting_result = fileio_info->param_3;
		else
			arm->semihosting_result = 0;
		break;

	case 0x06:	/* SYS_READ */
		if (result == (int)fileio_info->param_3)
			arm->semihosting_result = 0;
		if (result <= 0)
			arm->semihosting_result = fileio_info->param_3;
		break;

	case 0x0a:	/* SYS_SEEK */
		if (result > 0)
			arm->semihosting_result = 0;
		break;
	}

	return post_result(target);
}

/**
 * Initialize ARM semihosting support.
 *
 * @param target Pointer to the ARM target to initialize.
 * @return An error status if there is a problem during initialization.
 */
int arm_semihosting_init(struct target *target)
{
	target->fileio_info = malloc(sizeof(*target->fileio_info));
	if (target->fileio_info == NULL) {
		LOG_ERROR("out of memory");
		return ERROR_FAIL;
	}

	target->type->get_gdb_fileio_info = get_gdb_fileio_info;
	target->type->gdb_fileio_end = gdb_fileio_end;

	return ERROR_OK;
}

/**
 * Checks for and processes an ARM semihosting request.  This is meant
 * to be called when the target is stopped due to a debug mode entry.
 * If the value 0 is returned then there was nothing to process. A non-zero
 * return value signifies that a request was processed and the target resumed,
 * or an error was encountered, in which case the caller must return
 * immediately.
 *
 * @param target Pointer to the ARM target to process.  This target must
 *	not represent an ARMv6-M or ARMv7-M processor.
 * @param retval Pointer to a location where the return code will be stored
 * @return non-zero value if a request was processed or an error encountered
 */
int arm_semihosting(struct target *target, int *retval)
{
	struct arm *arm = target_to_arm(target);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	uint32_t pc, lr, spsr;
	struct reg *r;

	if (!arm->is_semihosting)
		return 0;

	if (is_arm7_9(target_to_arm7_9(target)) ||
	    is_armv7a(armv7a)) {
		uint32_t vbar = 0x00000000;

		if (arm->core_mode != ARM_MODE_SVC)
			return 0;

		if (is_armv7a(armv7a)) {
			struct arm_dpm *dpm = armv7a->arm.dpm;

			*retval = dpm->prepare(dpm);
			if (*retval == ERROR_OK) {
				*retval = dpm->instr_read_data_r0(dpm,
								 ARMV4_5_MRC(15, 0, 0, 12, 0, 0),
								 &vbar);

				dpm->finish(dpm);

				if (*retval != ERROR_OK)
					return 1;
			} else {
				return 1;
			}
		}

		/* Check for PC == 0x00000008 or 0xffff0008: Supervisor Call vector. */
		r = arm->pc;
		pc = buf_get_u32(r->value, 0, 32);
		if (pc != (vbar + 0x00000008) && pc != 0xffff0008)
			return 0;

		r = arm_reg_current(arm, 14);
		lr = buf_get_u32(r->value, 0, 32);

		/* Core-specific code should make sure SPSR is retrieved
		 * when the above checks pass...
		 */
		if (!arm->spsr->valid) {
			LOG_ERROR("SPSR not valid!");
			*retval = ERROR_FAIL;
			return 1;
		}

		spsr = buf_get_u32(arm->spsr->value, 0, 32);

		/* check instruction that triggered this trap */
		if (spsr & (1 << 5)) {
			/* was in Thumb (or ThumbEE) mode */
			uint8_t insn_buf[2];
			uint16_t insn;

			*retval = target_read_memory(target, lr-2, 2, 1, insn_buf);
			if (*retval != ERROR_OK)
				return 1;
			insn = target_buffer_get_u16(target, insn_buf);

			/* SVC 0xab */
			if (insn != 0xDFAB)
				return 0;
		} else if (spsr & (1 << 24)) {
			/* was in Jazelle mode */
			return 0;
		} else {
			/* was in ARM mode */
			uint8_t insn_buf[4];
			uint32_t insn;

			*retval = target_read_memory(target, lr-4, 4, 1, insn_buf);
			if (*retval != ERROR_OK)
				return 1;
			insn = target_buffer_get_u32(target, insn_buf);

			/* SVC 0x123456 */
			if (insn != 0xEF123456)
				return 0;
		}
	} else if (is_armv7m(target_to_armv7m(target))) {
		uint16_t insn;

		if (target->debug_reason != DBG_REASON_BREAKPOINT)
			return 0;

		r = arm->pc;
		pc = buf_get_u32(r->value, 0, 32);

		pc &= ~1;
		*retval = target_read_u16(target, pc, &insn);
		if (*retval != ERROR_OK)
			return 1;

		/* bkpt 0xAB */
		if (insn != 0xBEAB)
			return 0;
	} else {
		LOG_ERROR("Unsupported semi-hosting Target");
		return 0;
	}

	/* Perform semihosting if we are not waiting on a fileio
	 * operation to complete.
	 */
	if (!arm->semihosting_hit_fileio) {
		*retval = do_semihosting(target);
		if (*retval != ERROR_OK) {
			LOG_ERROR("Failed semihosting operation");
			return 0;
		}
	}

	/* Post result to target if we are not waiting on a fileio
	 * operation to complete:
	 */
	if (!arm->semihosting_hit_fileio) {
		*retval = post_result(target);
		if (*retval != ERROR_OK) {
			LOG_ERROR("Failed to post semihosting result");
			return 0;
		}

		*retval = target_resume(target, 1, 0, 0, 0);
		if (*retval != ERROR_OK) {
			LOG_ERROR("Failed to resume target");
			return 0;
		}

		return 1;
	}

	return 0;
}
