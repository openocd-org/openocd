// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Semihosting API for Espressif chips                                   *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <target/target.h>
#include <target/semihosting_common.h>
#include "esp_semihosting.h"
#include "esp_xtensa.h"

static struct esp_semihost_data __attribute__((unused)) *target_to_esp_semihost_data(struct target *target)
{
	const char *arch = target_get_gdb_arch(target);
	if (arch) {
		if (strncmp(arch, "xtensa", 6) == 0)
			return &target_to_esp_xtensa(target)->semihost;
		/* TODO: add riscv */
	}
	LOG_ERROR("Unknown target arch!");
	return NULL;
}

static int esp_semihosting_sys_seek(struct target *target, uint64_t fd, uint32_t pos, size_t whence)
{
	struct semihosting *semihosting = target->semihosting;

	semihosting->result = lseek(fd, pos, whence);
	semihosting->sys_errno = errno;
	LOG_TARGET_DEBUG(target, "lseek(%" PRIx64 ", %" PRIu32 " %" PRId64 ")=%d", fd, pos, semihosting->result, errno);
	return ERROR_OK;
}

int esp_semihosting_common(struct target *target)
{
	struct semihosting *semihosting = target->semihosting;
	if (!semihosting)
		/* Silently ignore if the semihosting field was not set. */
		return ERROR_OK;

	int retval = ERROR_NOT_IMPLEMENTED;

	/* Enough space to hold 4 long words. */
	uint8_t fields[4 * 8];

	/*
	 * By default return an error.
	 * The actual result must be set by each function
	 */
	semihosting->result = -1;
	semihosting->sys_errno = EIO;

	LOG_TARGET_DEBUG(target, "op=0x%x, param=0x%" PRIx64, semihosting->op, semihosting->param);

	switch (semihosting->op) {
	case ESP_SEMIHOSTING_SYS_DRV_INFO:
		/* Return success to make esp-idf application happy */
		retval = ERROR_OK;
		semihosting->result = 0;
		semihosting->sys_errno = 0;
		break;

	case ESP_SEMIHOSTING_SYS_SEEK:
		retval = semihosting_read_fields(target, 3, fields);
		if (retval == ERROR_OK) {
			uint64_t fd = semihosting_get_field(target, 0, fields);
			uint32_t pos = semihosting_get_field(target, 1, fields);
			size_t whence = semihosting_get_field(target, 2, fields);
			retval = esp_semihosting_sys_seek(target, fd, pos, whence);
		}
		break;

	case ESP_SEMIHOSTING_SYS_APPTRACE_INIT:
	case ESP_SEMIHOSTING_SYS_DEBUG_STUBS_INIT:
	case ESP_SEMIHOSTING_SYS_BREAKPOINT_SET:
	case ESP_SEMIHOSTING_SYS_WATCHPOINT_SET:
		/* For the time being only riscv chips support these commands
		 * TODO: invoke riscv custom command handler */
		break;
	}

	return retval;
}

int esp_semihosting_basedir_command(struct command_invocation *cmd)
{
	struct target *target = get_current_target(CMD_CTX);

	if (!target) {
		LOG_ERROR("No target selected");
		return ERROR_FAIL;
	}

	struct semihosting *semihosting = target->semihosting;
	if (!semihosting) {
		command_print(CMD, "semihosting not supported for current target");
		return ERROR_FAIL;
	}

	if (!semihosting->is_active) {
		if (semihosting->setup(target, true) != ERROR_OK) {
			LOG_ERROR("Failed to Configure semihosting");
			return ERROR_FAIL;
		}
		semihosting->is_active = true;
	}

	if (CMD_ARGC > 0) {
		free(semihosting->basedir);
		semihosting->basedir = strdup(CMD_ARGV[0]);
		if (!semihosting->basedir) {
			command_print(CMD, "semihosting failed to allocate memory for basedir!");
			return ERROR_FAIL;
		}
	}

	command_print(CMD, "DEPRECATED! semihosting base dir: %s",
		semihosting->basedir ? semihosting->basedir : "");

	return ERROR_OK;
}
