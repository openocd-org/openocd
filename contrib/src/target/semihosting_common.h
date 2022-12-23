/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2018 by Liviu Ionescu                                   *
 *   <ilg@livius.net>                                                      *
 *                                                                         *
 *   Copyright (C) 2009 by Marvell Technology Group Ltd.                   *
 *   Written by Nicolas Pitre <nico@marvell.com>                           *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_SEMIHOSTING_COMMON_H
#define OPENOCD_TARGET_SEMIHOSTING_COMMON_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "helper/replacements.h"
#include <server/server.h>

/*
 * According to:
 * "Semihosting for AArch32 and AArch64, Release 2.0"
 * https://static.docs.arm.com/100863/0200/semihosting.pdf
 * from ARM Ltd.
 *
 * The available semihosting operation numbers passed in R0 are allocated
 * as follows:
 * - 0x00-0x31 Used by ARM.
 * - 0x32-0xFF Reserved for future use by ARM.
 * - 0x100-0x1FF Reserved for user applications. These are not used by ARM.
 *   However, if you are writing your own SVC operations, you are advised
 *   to use a different SVC number rather than using the semihosted
 *   SVC number and these operation type numbers.
 * - 0x200-0xFFFFFFFF Undefined and currently unused. It is recommended
 *   that you do not use these.
 */

enum semihosting_operation_numbers {
	/*
	 * ARM semihosting operations, in lexicographic order.
	 */
	SEMIHOSTING_ENTER_SVC = 0x17,	/* DEPRECATED */

	SEMIHOSTING_SYS_CLOSE = 0x02,
	SEMIHOSTING_SYS_CLOCK = 0x10,
	SEMIHOSTING_SYS_ELAPSED = 0x30,
	SEMIHOSTING_SYS_ERRNO = 0x13,
	SEMIHOSTING_SYS_EXIT = 0x18,
	SEMIHOSTING_SYS_EXIT_EXTENDED = 0x20,
	SEMIHOSTING_SYS_FLEN = 0x0C,
	SEMIHOSTING_SYS_GET_CMDLINE = 0x15,
	SEMIHOSTING_SYS_HEAPINFO = 0x16,
	SEMIHOSTING_SYS_ISERROR = 0x08,
	SEMIHOSTING_SYS_ISTTY = 0x09,
	SEMIHOSTING_SYS_OPEN = 0x01,
	SEMIHOSTING_SYS_READ = 0x06,
	SEMIHOSTING_SYS_READC = 0x07,
	SEMIHOSTING_SYS_REMOVE = 0x0E,
	SEMIHOSTING_SYS_RENAME = 0x0F,
	SEMIHOSTING_SYS_SEEK = 0x0A,
	SEMIHOSTING_SYS_SYSTEM = 0x12,
	SEMIHOSTING_SYS_TICKFREQ = 0x31,
	SEMIHOSTING_SYS_TIME = 0x11,
	SEMIHOSTING_SYS_TMPNAM = 0x0D,
	SEMIHOSTING_SYS_WRITE = 0x05,
	SEMIHOSTING_SYS_WRITEC = 0x03,
	SEMIHOSTING_SYS_WRITE0 = 0x04,
	SEMIHOSTING_USER_CMD_0X100 = 0x100, /* First user cmd op code */
	SEMIHOSTING_USER_CMD_0X107 = 0x107, /* Last supported user cmd op code */
	SEMIHOSTING_USER_CMD_0X1FF = 0x1FF, /* Last user cmd op code */
};

/** Maximum allowed Tcl command segment length in bytes*/
#define SEMIHOSTING_MAX_TCL_COMMAND_FIELD_LENGTH (1024 * 1024)

/*
 * Codes used by SEMIHOSTING_SYS_EXIT (formerly
 * SEMIHOSTING_REPORT_EXCEPTION).
 * On 64-bits, the exit code is passed explicitly.
 */
enum semihosting_reported_exceptions {
	/* On 32 bits, use it for exit(0) */
	ADP_STOPPED_APPLICATION_EXIT = ((2 << 16) + 38),
	/* On 32 bits, use it for exit(1) */
	ADP_STOPPED_RUN_TIME_ERROR = ((2 << 16) + 35),
};

enum semihosting_redirect_config {
	SEMIHOSTING_REDIRECT_CFG_NONE,
	SEMIHOSTING_REDIRECT_CFG_DEBUG,
	SEMIHOSTING_REDIRECT_CFG_STDIO,
	SEMIHOSTING_REDIRECT_CFG_ALL,
};

enum semihosting_result {
	SEMIHOSTING_NONE,		/* Not halted for a semihosting call. */
	SEMIHOSTING_HANDLED,	/* Call handled, and target was resumed. */
	SEMIHOSTING_WAITING,	/* Call handled, target is halted waiting until we can resume. */
	SEMIHOSTING_ERROR		/* Something went wrong. */
};

struct target;

/*
 * A pointer to this structure was added to the target structure.
 */
struct semihosting {

	/** A flag reporting whether semihosting is active. */
	bool is_active;

	/** Semihosting STDIO file descriptors */
	int stdin_fd, stdout_fd, stderr_fd;

	/** redirection configuration, NONE by default */
	enum semihosting_redirect_config redirect_cfg;

	/** Handle to redirect semihosting print via tcp */
	struct connection *tcp_connection;

	/** A flag reporting whether semihosting fileio is active. */
	bool is_fileio;

	/** A flag reporting whether semihosting fileio operation is active. */
	bool hit_fileio;

	/** Most are resumable, except the two exit calls. */
	bool is_resumable;

	/**
	 * When SEMIHOSTING_SYS_EXIT is called outside a debug session,
	 * things are simple, the openocd process calls exit() and passes
	 * the value returned by the target.
	 * When SEMIHOSTING_SYS_EXIT is called during a debug session,
	 * by default execution returns to the debugger, leaving the
	 * debugger in a HALT state, similar to the state entered when
	 * encountering a break.
	 * In some use cases, it is useful to have SEMIHOSTING_SYS_EXIT
	 * return normally, as any semihosting call, and do not break
	 * to the debugger.
	 * The standard allows this to happen, but the condition
	 * to trigger it is a bit obscure ("by performing an RDI_Execute
	 * request or equivalent").
	 *
	 * To make the SEMIHOSTING_SYS_EXIT call return normally, enable
	 * this variable via the dedicated command (default: disabled).
	 */
	bool has_resumable_exit;

	/** The Target (hart) word size; 8 for 64-bits targets. */
	size_t word_size_bytes;

	/** The current semihosting operation (R0 on ARM). */
	int op;

	/** The current semihosting parameter (R1 or ARM). */
	uint64_t param;

	/**
	 * The current semihosting result to be returned to the application.
	 * Usually 0 for success, -1 for error,
	 * but sometimes a useful value, even a pointer.
	 */
	int64_t result;

	/** The value to be returned by semihosting SYS_ERRNO request. */
	int sys_errno;

	/** The semihosting command line to be passed to the target. */
	char *cmdline;

	/** The current time when 'execution starts' */
	clock_t setup_time;

	/** Base directory for semihosting I/O operations. */
	char *basedir;

	/**
	 * Target's extension of semihosting user commands.
	 * @returns ERROR_NOT_IMPLEMENTED when user command is not handled, otherwise
	 * sets semihosting->result and semihosting->sys_errno and returns ERROR_OK.
	 */
	int (*user_command_extension)(struct target *target);

	int (*setup)(struct target *target, int enable);
	int (*post_result)(struct target *target);
};

int semihosting_common_init(struct target *target, void *setup,
	void *post_result);
int semihosting_common(struct target *target);

/* utility functions which may also be used by semihosting extensions (custom vendor-defined syscalls) */
int semihosting_read_fields(struct target *target, size_t number,
	uint8_t *fields);
int semihosting_write_fields(struct target *target, size_t number,
	uint8_t *fields);
uint64_t semihosting_get_field(struct target *target, size_t index,
	uint8_t *fields);
void semihosting_set_field(struct target *target, uint64_t value,
	size_t index,
	uint8_t *fields);

extern const struct command_registration semihosting_common_handlers[];

#endif	/* OPENOCD_TARGET_SEMIHOSTING_COMMON_H */
