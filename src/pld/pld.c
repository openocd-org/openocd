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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "replacements.h"

#include "pld.h"

#include "jtag.h"
#include "command.h"
#include "log.h"
#include "time_support.h"

#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include <sys/time.h>
#include <time.h>

/* pld drivers
 */
extern pld_driver_t virtex2_pld;

pld_driver_t *pld_drivers[] =
{
	&virtex2_pld,
	NULL,
};

pld_device_t *pld_devices;
static command_t *pld_cmd;

int handle_pld_devices_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_pld_device_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_pld_load_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int pld_init(struct command_context_s *cmd_ctx)
{
	if (pld_devices)
	{
		register_command(cmd_ctx, pld_cmd, "devices", handle_pld_devices_command, COMMAND_EXEC,
						"list configured pld devices");
		register_command(cmd_ctx, pld_cmd, "load", handle_pld_load_command, COMMAND_EXEC,
						"load configuration <file> into programmable logic device");
	}
	
	return ERROR_OK;
}

pld_device_t *get_pld_device_by_num(int num)
{
	pld_device_t *p;
	int i = 0;

	for (p = pld_devices; p; p = p->next)
	{
		if (i++ == num)
		{
			return p;
		}
	}
	
	return NULL;
}

/* pld device <driver> [driver_options ...]
 */
int handle_pld_device_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int i;
	int found = 0;
		
	if (argc < 1)
	{
		LOG_WARNING("incomplete 'pld bank' configuration");
		return ERROR_OK;
	}
	
	for (i = 0; pld_drivers[i]; i++)
	{
		if (strcmp(args[0], pld_drivers[i]->name) == 0)
		{
			pld_device_t *p, *c;
			
			/* register pld specific commands */
			if (pld_drivers[i]->register_commands(cmd_ctx) != ERROR_OK)
			{
				LOG_ERROR("couldn't register '%s' commands", args[0]);
				exit(-1);
			}
			
			c = malloc(sizeof(pld_device_t));
			c->driver = pld_drivers[i];
			c->next = NULL;
			
			if (pld_drivers[i]->pld_device_command(cmd_ctx, cmd, args, argc, c) != ERROR_OK)
			{
				LOG_ERROR("'%s' driver rejected pld device", args[0]);
				free(c);
				return ERROR_OK;
			}
			
			/* put pld device in linked list */
			if (pld_devices)
			{
				/* find last pld device */
				for (p = pld_devices; p && p->next; p = p->next);
				if (p)
					p->next = c;
			}
			else
			{
				pld_devices = c;
			}
			
			found = 1;
		}
	}
		
	/* no matching pld driver found */
	if (!found)
	{
		LOG_ERROR("pld driver '%s' not found", args[0]);
		exit(-1);
	}
	
	return ERROR_OK;
}

int handle_pld_devices_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	pld_device_t *p;
	int i = 0;
	
	if (!pld_devices)
	{
		command_print(cmd_ctx, "no pld devices configured");
		return ERROR_OK;
	}
	
	for (p = pld_devices; p; p = p->next)
	{
		command_print(cmd_ctx, "#%i: %s", i++, p->driver->name);
	}
	
	return ERROR_OK;
}

int handle_pld_load_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	struct timeval start, end, duration;
	pld_device_t *p;
	
	gettimeofday(&start, NULL);
		
	if (argc < 2)
	{
		command_print(cmd_ctx, "usage: pld load <device#> <file>");
		return ERROR_OK;
	}
	
	p = get_pld_device_by_num(strtoul(args[0], NULL, 0));
	if (!p)
	{
		command_print(cmd_ctx, "pld device '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}
	
	if ((retval = p->driver->load(p, args[1])) != ERROR_OK)
	{
		command_print(cmd_ctx, "failed loading file %s to pld device %i",
			args[1], strtoul(args[0], NULL, 0));
		switch (retval)
		{
		}
	}
	else
	{
		gettimeofday(&end, NULL);	
		timeval_subtract(&duration, &end, &start);
		
		command_print(cmd_ctx, "loaded file %s to pld device %i in %is %ius", 
			args[1], strtoul(args[0], NULL, 0), duration.tv_sec, duration.tv_usec);
	}
	
	return ERROR_OK;
}

int pld_register_commands(struct command_context_s *cmd_ctx)
{
	pld_cmd = register_command(cmd_ctx, NULL, "pld", NULL, COMMAND_ANY, "programmable logic device commands");
	
	register_command(cmd_ctx, pld_cmd, "device", handle_pld_device_command, COMMAND_CONFIG, NULL);
	
	return ERROR_OK;
}
