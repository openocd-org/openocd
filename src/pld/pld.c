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

#include "pld.h"
#include "log.h"
#include "time_support.h"


/* pld drivers
 */
extern struct pld_driver virtex2_pld;

static struct pld_driver *pld_drivers[] =
{
	&virtex2_pld,
	NULL,
};

static struct pld_device *pld_devices;
static struct command *pld_cmd;

struct pld_device *get_pld_device_by_num(int num)
{
	struct pld_device *p;
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
COMMAND_HANDLER(handle_pld_device_command)
{
	int i;
	int found = 0;

	if (CMD_ARGC < 1)
	{
		LOG_WARNING("incomplete 'pld device' command");
		return ERROR_OK;
	}

	for (i = 0; pld_drivers[i]; i++)
	{
		if (strcmp(CMD_ARGV[0], pld_drivers[i]->name) == 0)
		{
			struct pld_device *p, *c;

			/* register pld specific commands */
			if (pld_drivers[i]->register_commands(CMD_CTX) != ERROR_OK)
			{
				LOG_ERROR("couldn't register '%s' commands", CMD_ARGV[0]);
				exit(-1);
			}

			c = malloc(sizeof(struct pld_device));
			c->driver = pld_drivers[i];
			c->next = NULL;

			int retval = CALL_COMMAND_HANDLER(pld_drivers[i]->pld_device_command, c);
			if (ERROR_OK != retval)
			{
				LOG_ERROR("'%s' driver rejected pld device", CMD_ARGV[0]);
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
		LOG_ERROR("pld driver '%s' not found", CMD_ARGV[0]);
		exit(-1);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_pld_devices_command)
{
	struct pld_device *p;
	int i = 0;

	if (!pld_devices)
	{
		command_print(CMD_CTX, "no pld devices configured");
		return ERROR_OK;
	}

	for (p = pld_devices; p; p = p->next)
	{
		command_print(CMD_CTX, "#%i: %s", i++, p->driver->name);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_pld_load_command)
{
	int retval;
	struct timeval start, end, duration;
	struct pld_device *p;

	gettimeofday(&start, NULL);

	if (CMD_ARGC < 2)
	{
		command_print(CMD_CTX, "usage: pld load <device#> <file>");
		return ERROR_OK;
	}

	unsigned dev_id;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], dev_id);
	p = get_pld_device_by_num(dev_id);
	if (!p)
	{
		command_print(CMD_CTX, "pld device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_OK;
	}

	if ((retval = p->driver->load(p, CMD_ARGV[1])) != ERROR_OK)
	{
		command_print(CMD_CTX, "failed loading file %s to pld device %u",
			CMD_ARGV[1], dev_id);
		switch (retval)
		{
		}
		return retval;
	}
	else
	{
		gettimeofday(&end, NULL);
		timeval_subtract(&duration, &end, &start);

		command_print(CMD_CTX, "loaded file %s to pld device %u in %jis %jius",
			CMD_ARGV[1], dev_id,
			(intmax_t)duration.tv_sec, (intmax_t)duration.tv_usec);
	}

	return ERROR_OK;
}

int pld_init(struct command_context *cmd_ctx)
{
	if (!pld_devices)
		return ERROR_OK;

	COMMAND_REGISTER(cmd_ctx, pld_cmd, "devices",
			handle_pld_devices_command, COMMAND_EXEC,
			"list configured pld devices");
	COMMAND_REGISTER(cmd_ctx, pld_cmd, "load",
			handle_pld_load_command, COMMAND_EXEC,
			"load configuration <file> into programmable logic device");

	return ERROR_OK;
}

int pld_register_commands(struct command_context *cmd_ctx)
{
	pld_cmd = COMMAND_REGISTER(cmd_ctx, NULL, "pld", NULL, COMMAND_ANY, "programmable logic device commands");

	COMMAND_REGISTER(cmd_ctx, pld_cmd, "device", handle_pld_device_command, COMMAND_CONFIG, NULL);

	return ERROR_OK;
}
