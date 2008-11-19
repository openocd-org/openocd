/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#include <string.h>

#include "etm_dummy.h"
#include "etm.h"

#include "arm7_9_common.h"
#include "log.h"
#include "types.h"
#include "binarybuffer.h"
#include "target.h"
#include "register.h"
#include "jtag.h"

#include <stdlib.h>

int handle_etm_dummy_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;

	target = get_target_by_num(strtoul(args[0], NULL, 0));

	if (!target)
	{
		LOG_ERROR("target number '%s' not defined", args[0]);
		return ERROR_FAIL;
	}

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_FAIL;
	}

	if (arm7_9->etm_ctx)
	{
		arm7_9->etm_ctx->capture_driver_priv = NULL;
	}
	else
	{
		LOG_ERROR("target has no ETM defined, ETM dummy left unconfigured");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int etm_dummy_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *etm_dummy_cmd;

	etm_dummy_cmd = register_command(cmd_ctx, NULL, "etm_dummy", NULL, COMMAND_ANY, "Dummy ETM capture driver");

	register_command(cmd_ctx, etm_dummy_cmd, "config", handle_etm_dummy_config_command, COMMAND_CONFIG, NULL);

	return ERROR_OK;
}

int etm_dummy_init(etm_context_t *etm_ctx)
{
	return ERROR_OK;
}

trace_status_t etm_dummy_status(etm_context_t *etm_ctx)
{
	return TRACE_IDLE;
}

int etm_dummy_read_trace(etm_context_t *etm_ctx)
{
	return ERROR_OK;
}

int etm_dummy_start_capture(etm_context_t *etm_ctx)
{
	return ERROR_ETM_PORTMODE_NOT_SUPPORTED;
}

int etm_dummy_stop_capture(etm_context_t *etm_ctx)
{
	return ERROR_OK;
}

etm_capture_driver_t etm_dummy_capture_driver =
{
	.name = "dummy",
	.register_commands = etm_dummy_register_commands,
	.init = etm_dummy_init,
	.status = etm_dummy_status,
	.start_capture = etm_dummy_start_capture,
	.stop_capture = etm_dummy_stop_capture,
	.read_trace = etm_dummy_read_trace,
};
