// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Espressif Xtensa target API for OpenOCD                               *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdbool.h>
#include <stdint.h>
#include <target/smp.h>
#include <target/register.h>
#include "esp_xtensa.h"
#include "esp_semihosting.h"

int esp_xtensa_init_arch_info(struct target *target,
	struct esp_xtensa_common *esp_xtensa,
	struct xtensa_debug_module_config *dm_cfg,
	const struct esp_semihost_ops *semihost_ops)
{
	int ret = xtensa_init_arch_info(target, &esp_xtensa->xtensa, dm_cfg);
	if (ret != ERROR_OK)
		return ret;
	esp_xtensa->semihost.ops = (struct esp_semihost_ops *)semihost_ops;
	return ERROR_OK;
}

int esp_xtensa_target_init(struct command_context *cmd_ctx, struct target *target)
{
	return xtensa_target_init(cmd_ctx, target);
}

void esp_xtensa_target_deinit(struct target *target)
{
	LOG_DEBUG("start");

	xtensa_target_deinit(target);
	free(target_to_esp_xtensa(target));	/* same as free(xtensa) */
}

int esp_xtensa_arch_state(struct target *target)
{
	return ERROR_OK;
}

int esp_xtensa_poll(struct target *target)
{
	return xtensa_poll(target);
}

int esp_xtensa_breakpoint_add(struct target *target, struct breakpoint *breakpoint)
{
	return xtensa_breakpoint_add(target, breakpoint);
	/* flash breakpoints will be handled in another patch */
}

int esp_xtensa_breakpoint_remove(struct target *target, struct breakpoint *breakpoint)
{
	return xtensa_breakpoint_remove(target, breakpoint);
	/* flash breakpoints will be handled in another patch */
}
