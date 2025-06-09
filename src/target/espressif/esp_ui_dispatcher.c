// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2023 Espressif Systems (Shanghai) Co. Ltd.              *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/command.h>
#include <helper/log.h>

#include <target/target.h>

#include "ui.h"

int examine_failed_ui_handler(struct command_invocation *cmd)
{
	struct target *target = get_current_target(CMD_CTX);
	char str[128] = {0};

	sprintf(str, "%s examination failed! Check your wire connection or target configuration scripts",
		target->cmd_name);
	ui_show_info_screen(str);

	return ERROR_OK;
}
