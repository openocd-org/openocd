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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_PLD_PLD_H
#define OPENOCD_PLD_PLD_H

#include <helper/command.h>

struct pld_device;

#define __PLD_DEVICE_COMMAND(name) \
	COMMAND_HELPER(name, struct pld_device *pld)

struct pld_driver {
	const char *name;
	__PLD_DEVICE_COMMAND((*pld_device_command));
	const struct command_registration *commands;
	int (*load)(struct pld_device *pld_device, const char *filename);
};

#define PLD_DEVICE_COMMAND_HANDLER(name) \
	static __PLD_DEVICE_COMMAND(name)

struct pld_device {
	struct pld_driver *driver;
	void *driver_priv;
	struct pld_device *next;
};

int pld_register_commands(struct command_context *cmd_ctx);

struct pld_device *get_pld_device_by_num(int num);

#define ERROR_PLD_DEVICE_INVALID        (-1000)
#define ERROR_PLD_FILE_LOAD_FAILED      (-1001)

#endif /* OPENOCD_PLD_PLD_H */
