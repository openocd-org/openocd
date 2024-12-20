// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "pld.h"
#include <sys/stat.h>
#include <helper/log.h>
#include <helper/replacements.h>
#include <helper/time_support.h>


static struct pld_driver *pld_drivers[] = {
	&efinix_pld,
	&gatemate_pld,
	&gowin_pld,
	&intel_pld,
	&lattice_pld,
	&virtex2_pld,
	NULL,
};

static struct pld_device *pld_devices;

static struct pld_device *get_pld_device_by_num(int num)
{
	struct pld_device *p;
	int i = 0;

	for (p = pld_devices; p; p = p->next) {
		if (i++ == num) {
			LOG_WARNING("DEPRECATED: use pld name \"%s\" instead of number %d", p->name, num);
			return p;
		}
	}

	return NULL;
}

struct pld_device *get_pld_device_by_name(const char *name)
{
	for (struct pld_device *p = pld_devices; p; p = p->next) {
		if (strcmp(p->name, name) == 0)
			return p;
	}

	return NULL;
}

struct pld_device *get_pld_device_by_name_or_numstr(const char *str)
{
	struct pld_device *dev = get_pld_device_by_name(str);
	if (dev)
		return dev;

	char *end;
	unsigned long dev_num = strtoul(str, &end, 0);
	if (*end || dev_num > INT_MAX) {
		LOG_ERROR("Invalid argument");
		return NULL;
	}

	return get_pld_device_by_num(dev_num);
}


int pld_has_jtagspi_instruction(struct pld_device *pld_device, bool *has_instruction)
{
	*has_instruction = false; /* default is using a proxy bitstream */

	if (!pld_device)
		return ERROR_FAIL;

	struct pld_driver *pld_driver = pld_device->driver;
	if (!pld_driver) {
		LOG_ERROR("pld device has no associated driver");
		return ERROR_FAIL;
	}

	if (pld_driver->has_jtagspi_instruction)
		return pld_driver->has_jtagspi_instruction(pld_device, has_instruction);
    /* else, take the default (proxy bitstream) */
	return ERROR_OK;
}

int pld_get_jtagspi_userircode(struct pld_device *pld_device, unsigned int *ir)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct pld_driver *pld_driver = pld_device->driver;
	if (!pld_driver) {
		LOG_ERROR("pld device has no associated driver");
		return ERROR_FAIL;
	}

	if (pld_driver->get_jtagspi_userircode)
		return pld_driver->get_jtagspi_userircode(pld_device, ir);

	return ERROR_FAIL;
}

int pld_get_jtagspi_stuff_bits(struct pld_device *pld_device, unsigned int *facing_read_bits,
							unsigned int *trailing_write_bits)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct pld_driver *pld_driver = pld_device->driver;
	if (!pld_driver) {
		LOG_ERROR("pld device has no associated driver");
		return ERROR_FAIL;
	}

	if (pld_driver->get_stuff_bits)
		return pld_driver->get_stuff_bits(pld_device, facing_read_bits, trailing_write_bits);

	return ERROR_OK;
}

int pld_connect_spi_to_jtag(struct pld_device *pld_device)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct pld_driver *pld_driver = pld_device->driver;
	if (!pld_driver) {
		LOG_ERROR("pld device has no associated driver");
		return ERROR_FAIL;
	}

	if (pld_driver->connect_spi_to_jtag)
		return pld_driver->connect_spi_to_jtag(pld_device);

	return ERROR_FAIL;
}

int pld_disconnect_spi_from_jtag(struct pld_device *pld_device)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct pld_driver *pld_driver = pld_device->driver;
	if (!pld_driver) {
		LOG_ERROR("pld device has no associated driver");
		return ERROR_FAIL;
	}

	if (pld_driver->disconnect_spi_from_jtag)
		return pld_driver->disconnect_spi_from_jtag(pld_device);

	return ERROR_FAIL;
}

COMMAND_HANDLER(handle_pld_create_command)
{
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct pld_driver *pld_driver = NULL;

	for (int i = 0; pld_drivers[i]; i++) {
		if (strcmp(CMD_ARGV[1], pld_drivers[i]->name) == 0) {
			pld_driver = pld_drivers[i];
			break;
		}
	}

	if (!pld_driver) {
		LOG_ERROR("pld driver '%s' not found", CMD_ARGV[1]);
		return ERROR_FAIL; /* exit(-1); */
	}

	if (get_pld_device_by_name(CMD_ARGV[0])) {
		LOG_ERROR("pld device with name '%s' already exists", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	struct pld_device *pld_device = malloc(sizeof(struct pld_device));
	if (!pld_device) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	pld_device->driver = pld_driver;
	pld_device->next = NULL;

	int retval = CALL_COMMAND_HANDLER(pld_driver->pld_create_command, pld_device);
	if (retval != ERROR_OK) {
		LOG_ERROR("'%s' driver rejected pld device",
			CMD_ARGV[1]);
		free(pld_device);
		return ERROR_OK;
	}
	pld_device->name = strdup(CMD_ARGV[0]);
	if (!pld_device->name) {
		LOG_ERROR("Out of memory");
		free(pld_device);
		return ERROR_FAIL;
	}

	/* register pld specific commands */
	if (pld_driver->commands) {
		retval = register_commands(CMD_CTX, NULL, pld_driver->commands);
		if (retval != ERROR_OK) {
			LOG_ERROR("couldn't register '%s' commands", CMD_ARGV[1]);
			free(pld_device->name);
			free(pld_device);
			return ERROR_FAIL;
		}
	}

	if (pld_devices) {
		/* find last pld device */
		struct pld_device *p = pld_devices;
		for (; p && p->next; p = p->next)
			;
		if (p)
			p->next = pld_device;
	} else {
		pld_devices = pld_device;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_pld_devices_command)
{
	struct pld_device *p;
	int i = 0;

	if (!pld_devices) {
		command_print(CMD, "no pld devices configured");
		return ERROR_OK;
	}

	for (p = pld_devices; p; p = p->next)
		command_print(CMD, "#%i: %s (driver: %s)", i++, p->name, p->driver->name);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_pld_load_command)
{
	int retval;
	struct timeval start, end, duration;
	struct pld_device *p;

	gettimeofday(&start, NULL);

	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	p = get_pld_device_by_name_or_numstr(CMD_ARGV[0]);
	if (!p) {
		command_print(CMD, "pld device '#%s' is out of bounds or unknown", CMD_ARGV[0]);
		return ERROR_OK;
	}

	struct stat input_stat;
	if (stat(CMD_ARGV[1], &input_stat) == -1) {
		LOG_ERROR("couldn't stat() %s: %s", CMD_ARGV[1], strerror(errno));
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	if (S_ISDIR(input_stat.st_mode)) {
		LOG_ERROR("%s is a directory", CMD_ARGV[1]);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	if (input_stat.st_size == 0) {
		LOG_ERROR("Empty file %s", CMD_ARGV[1]);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	retval = p->driver->load(p, CMD_ARGV[1]);
	if (retval != ERROR_OK) {
		command_print(CMD, "failed loading file %s to pld device %s",
			CMD_ARGV[1], CMD_ARGV[0]);
		return retval;
	} else {
		gettimeofday(&end, NULL);
		timeval_subtract(&duration, &end, &start);

		command_print(CMD, "loaded file %s to pld device %s in %jis %jius",
			CMD_ARGV[1], CMD_ARGV[0],
			(intmax_t)duration.tv_sec, (intmax_t)duration.tv_usec);
	}

	return ERROR_OK;
}

static const struct command_registration pld_exec_command_handlers[] = {
	{
		.name = "devices",
		.handler = handle_pld_devices_command,
		.mode = COMMAND_EXEC,
		.help = "list configured pld devices",
		.usage = "",
	},
	{
		.name = "load",
		.handler = handle_pld_load_command,
		.mode = COMMAND_EXEC,
		.help = "load configuration file into PLD",
		.usage = "pld_name filename",
	},
	COMMAND_REGISTRATION_DONE
};

static int pld_init(struct command_context *cmd_ctx)
{
	if (!pld_devices)
		return ERROR_OK;

	return register_commands(cmd_ctx, "pld", pld_exec_command_handlers);
}

COMMAND_HANDLER(handle_pld_init_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	static bool pld_initialized;
	if (pld_initialized) {
		LOG_INFO("'pld init' has already been called");
		return ERROR_OK;
	}
	pld_initialized = true;

	LOG_DEBUG("Initializing PLDs...");
	return pld_init(CMD_CTX);
}

static const struct command_registration pld_config_command_handlers[] = {
	{
		.name = "create",
		.mode = COMMAND_CONFIG,
		.handler = handle_pld_create_command,
		.help = "create a PLD device",
		.usage = "name.pld driver_name [driver_args ... ]",
	},
	{
		.name = "init",
		.mode = COMMAND_CONFIG,
		.handler = handle_pld_init_command,
		.help = "initialize PLD devices",
		.usage = ""
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration pld_command_handler[] = {
	{
		.name = "pld",
		.mode = COMMAND_ANY,
		.help = "programmable logic device commands",
		.usage = "",
		.chain = pld_config_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
int pld_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, pld_command_handler);
}
