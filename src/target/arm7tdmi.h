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

#include "embeddedice.h"

/* FIXME we don't really need a separate arm7tdmi struct any more...
 * remove it, the arm7/arm9 common struct suffices.
 */
typedef struct arm7tdmi_common_s
{
	struct arm7_9_common arm7_9_common;
} arm7tdmi_common_t;

int arm7tdmi_init_arch_info(target_t *target, arm7tdmi_common_t *arm7tdmi, struct jtag_tap *tap);
int arm7tdmi_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int arm7tdmi_examine(struct target_s *target);

#endif /* ARM7TDMI_H */
