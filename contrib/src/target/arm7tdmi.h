/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARM7TDMI_H
#define OPENOCD_TARGET_ARM7TDMI_H

#include "embeddedice.h"

int arm7tdmi_init_arch_info(struct target *target,
		struct arm7_9_common *arm7_9, struct jtag_tap *tap);
int arm7tdmi_init_target(struct command_context *cmd_ctx,
		struct target *target);
void arm7tdmi_deinit_target(struct target *target);

#endif /* OPENOCD_TARGET_ARM7TDMI_H */
