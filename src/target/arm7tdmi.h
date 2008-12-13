/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
#ifndef ARM7TDMI_H
#define ARM7TDMI_H

#include "target.h"
#include "register.h"
#include "armv4_5.h"
#include "embeddedice.h"
#include "arm_jtag.h"
#include "arm7_9_common.h"

#define	ARM7TDMI_COMMON_MAGIC 0x00a700a7

typedef struct arm7tdmi_common_s
{
	int common_magic;
	void *arch_info;
	arm7_9_common_t arm7_9_common;
} arm7tdmi_common_t;

int arm7tdmi_register_commands(struct command_context_s *cmd_ctx);
int arm7tdmi_init_arch_info(target_t *target, arm7tdmi_common_t *arm7tdmi, jtag_tap_t *tap);
int arm7tdmi_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int arm7tdmi_examine(struct target_s *target);

#endif /* ARM7TDMI_H */
