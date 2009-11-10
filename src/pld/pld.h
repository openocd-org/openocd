/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
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
#ifndef PLD_H
#define PLD_H

#include "command.h"

struct pld_device_s;

#define __PLD_DEVICE_COMMAND(name) \
		COMMAND_HELPER(name, struct pld_device_s *pld)

typedef struct pld_driver_s
{
	char *name;
	__PLD_DEVICE_COMMAND((*pld_device_command));
	int (*register_commands)(struct command_context_s *cmd_ctx);
	int (*load)(struct pld_device_s *pld_device, const char *filename);
} pld_driver_t;

#define PLD_DEVICE_COMMAND_HANDLER(name) static __PLD_DEVICE_COMMAND(name)

typedef struct pld_device_s
{
	pld_driver_t *driver;
	void *driver_priv;
	struct pld_device_s *next;
} pld_device_t;

int pld_register_commands(struct command_context_s *cmd_ctx);

int pld_init(struct command_context_s *cmd_ctx);

pld_device_t *get_pld_device_by_num(int num);

#define ERROR_PLD_DEVICE_INVALID	(-1000)
#define ERROR_PLD_FILE_LOAD_FAILED	(-1001)

#endif /* PLD_H */
