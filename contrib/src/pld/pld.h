/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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
