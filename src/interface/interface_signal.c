/*
 * Copyright (C) 2011-2012 Tomasz Boleslaw CEDRO
 * cederom@tlen.pl, http://www.tomek.cedro.info
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/** @file: Generic OpenOCD interface. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <interface/interface.h>
#include <helper/log.h>

extern struct jtag_interface *jtag_interface;

/******************************************************************************
 * SIGNAL INFRASTRUCTURE AND OPERATIONS
 ******************************************************************************/

/** Check if specified signal is already defined (case insensitive) and return
 * its pointer if defined.
 * \param *name signal name to check
 * \return pointer to signal structure in memory if found, NULL otherwise.
 */
oocd_interface_signal_t *oocd_interface_signal_find(char *name)
{
	/* LOG_DEBUG("Searching for interface signal \"%s\"", name); */
	/* Check if interface already exists */
	if (!jtag_interface) {
		LOG_ERROR("Interface does not yet exist!");
		return NULL;
	}
	/* Check if interface signal to already exists */
	if (!jtag_interface->signal) {
		LOG_DEBUG("No interface signals defined (yet?).");
		return NULL;
	}
	/* Check if signal name is correct */
	if (!name || strncmp(name, " ", 1) == 0) {
		LOG_ERROR("Interface signal name cannot be empty.");
		return NULL;
	}
	/* Check if signal name already exist */
	oocd_interface_signal_t *sig;
	sig = jtag_interface->signal;
	while (sig) {
		if (!strncasecmp(sig->name, name, 32)) {
			LOG_DEBUG("Interface signal %s found.", sig->name);
			return sig;
		}
		sig = sig->next;
	}
	/* If signal is not found return null pointer. */
	LOG_WARNING("Interface signal %s not found.", name);
	return NULL;
}
/** Add new signal to the interface.
 * Signal will be allocated in memory with provided name and mask.
 * There is no sense for giving value field at this time because signal create
 * can take place during initialization where interface is not yet ready, also
 * they can be used for read and write, so this is higher level script task
 * to initialize their default value with appropriate 'bitbang' call.
 * The default value for new signal equals provided mask to maintain Hi-Z.
 *
 * \param *name is the signal name (max 32 char).
 * \param mask is the signal mask (unsigned int).
 * \param value is the initial value for signal to set.
 * \return ERROR_OK on success or ERROR_FAIL on failure.
 */
int oocd_interface_signal_add(char *name, unsigned int mask)
{
	LOG_DEBUG("Adding signal \"%s\"", name);
	/* Check if interface already exists */
	if (!jtag_interface) {
		LOG_ERROR("Interface does not yet exist!");
		return ERROR_FAIL;
	}

	/* Check if name is correct string */
	if (!name || strncmp(name, " ", 1) == 0) {
		LOG_ERROR("Signal name cannot be empty");
		return ERROR_FAIL;
	}

	oocd_interface_signal_t *newsignal, *lastsignal;
	int snlen;

	/* Check signal length (min=1, max=32 characters) */
	snlen = strnlen(name, 32);
	if (snlen < 1 || snlen > 32) {
		LOG_ERROR("Signal name too short or too long!");
		return ERROR_FAIL;
	}

	/* Check if signal name already exist and return error if so */
	if (oocd_interface_signal_find(name)) {
		LOG_ERROR("Specified signal already exist!");
		return ERROR_FAIL;
	}

	/* Allocate memory for new signal structure */
	newsignal = (oocd_interface_signal_t *)calloc(1, sizeof(oocd_interface_signal_t));
	if (!newsignal) {
		LOG_ERROR("cannot allocate memory for new signal: %s", name);
		return ERROR_FAIL;
	}
	newsignal->name = (char *)calloc(1, snlen + 1);
	if (!newsignal->name) {
		LOG_ERROR("cannot allocate memory for signal %s name", name);
		return ERROR_FAIL;
	}

	/* Initialize structure data and return or break on error */
	for (;;) {
		if (!strncpy(newsignal->name, name, snlen)) {
			LOG_ERROR("cannot copy signal %s name!", name);
			break;
		}

		newsignal->mask = mask;
		newsignal->value = mask;

		if (!jtag_interface->signal) {
			jtag_interface->signal = newsignal;
		} else {
			lastsignal = jtag_interface->signal;
			while (lastsignal->next)
				lastsignal = lastsignal->next;
			lastsignal->next = newsignal;
		}
		LOG_DEBUG("Signal \"%s\" added.", name);
		return ERROR_OK;
	}

	/* If there was an error free up resources and return error */
	free(newsignal->name);
	free(newsignal);
	return ERROR_FAIL;
}

/** Delete interface signal.
 * Removes signal from singly linked list of interface signals and free memory.
 * \param name is the name of the signal to remove.
 * \return ERROR_OK on success, ERROR_FAIL on failure.
 */
int oocd_interface_signal_del(char *name)
{
	LOG_DEBUG("Deleting signal \"%s\"", name);
	/* Check if interface already exists */
	if (!jtag_interface) {
		LOG_ERROR("Interface does not yet exist!");
		return ERROR_FAIL;
	}
	/* Check if interface any signal exist */
	if (!jtag_interface->signal) {
		LOG_ERROR("Signal list is empty!");
		return ERROR_FAIL;
	}

	/* Check if signal name is correct */
	if (!name || strncmp(name, " ", 1) == 0) {
		LOG_ERROR("Signal name cannot be empty.");
		return ERROR_FAIL;
	}

	oocd_interface_signal_t *delsig = NULL, *prevsig = NULL;

	/* look for the signal name on the list */
	delsig = oocd_interface_signal_find(name);

	/* return error if signal is not on the list */
	if (!delsig) {
		LOG_ERROR("Signal not found!");
		return ERROR_FAIL;
	}

	/* detach signal to be removed from the list */
	prevsig = jtag_interface->signal;
	if (prevsig == delsig) {
		/* we need to detach first signal on the list */
		jtag_interface->signal = jtag_interface->signal->next;
	} else {
		for (; prevsig->next; prevsig = prevsig->next) {
			if (prevsig->next == delsig) {
				prevsig->next = prevsig->next->next;
				break;
			}
		}
	}

	/* now free memory of detached element */
	free(delsig->name);
	free(delsig);
	LOG_DEBUG("Signal \"%s\" removed.", name);
	return ERROR_OK;
}

/******************************************************************************
 * TCL INTERFACE TO INTERFACE_SIGNAL INFRASTRUCTURE AND OPERATIONS
 ******************************************************************************/

/** Interface signals handling routine that can add, delete and list signals.
 * Signal ADD requires signal name string and 32-bit mask, optionally a value.
 * Values are read as HEX. Signal DEL requires only signal name to delete.
 * Signal LIST will show marvelous table wits signal names, masks and values.
 * Interfaces should be defined in configuration file by TCL interface.
 * Parameters are checked before function execution.
 */
COMMAND_HANDLER(handle_oocd_interface_signal_command)
{
	LOG_DEBUG("entering function...");

	if (!jtag_interface) {
		command_print(CMD_CTX, "You must initialize interface first!");
		return ERROR_FAIL;
	}

	if (CMD_ARGC < 1 || CMD_ARGC > 3) {
		command_print(CMD_CTX, "Bad syntax!");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	int sigmask;
	char signame[32];

	if (!strncasecmp(CMD_ARGV[0], "add", 3)) {
		if (CMD_ARGC < 3) {
			LOG_ERROR("Usage: interface_signal add signal_name signal_mask");
			return ERROR_FAIL;
		}
		if (!strncpy(signame, CMD_ARGV[1], 32)) {
			LOG_ERROR("Unable to copy signal name from parameter list!");
			return ERROR_FAIL;
		}
		/* We are going to add interface signal. */
		/* Check the mask parameter. */
		if (!sscanf(CMD_ARGV[2], "%x", &sigmask)) {
			LOG_ERROR("Bad signal mask parameter! Use HEX value.");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		/* Now add the inetrface signal with specified parameters. */
		return oocd_interface_signal_add(signame, sigmask);

	} else if (!strncasecmp(CMD_ARGV[0], "del", 3)) {
		if (CMD_ARGC < 2) {
			LOG_ERROR("Usage: interface_signal del signal_name");
			return ERROR_FAIL;
		}
		/* We are going to delete specified signal. */
		return oocd_interface_signal_del((char *)CMD_ARGV[1]);

	} else if (!strncasecmp(CMD_ARGV[0], "list", 4)) {
		/* We are going to list available signals. */
		oocd_interface_signal_t *sig;
		sig = jtag_interface->signal;
		command_print(CMD_CTX, "      Interface Signal Name      |    Mask    |   Value   ");
		command_print(CMD_CTX, "----------------------------------------------------------");
		while (sig) {
			command_print(CMD_CTX, "%32s | 0x%08X | 0x%08X", sig->name, sig->mask, sig->value);
			sig = sig->next;
		}
		/* Also print warning if interface driver does not support bit-baning. */
		if (!jtag_interface->bitbang)
			command_print(CMD_CTX, "WARNING: This interface does not support bit-baning!");
		return ERROR_OK;

	} else if (!strncasecmp(CMD_ARGV[0], "find", 4)) {
		if (CMD_ARGC < 2) {
			LOG_ERROR("Usage: interface_signal find signal_name");
			return ERROR_FAIL;
		}
		/* Find the signal and print its details. */
		oocd_interface_signal_t *sig = oocd_interface_signal_find((char *)CMD_ARGV[1]);
		if (sig != NULL) {
			command_print(CMD_CTX, "%s: mask=0x%08X value=0x%08X", sig->name, sig->mask, sig->value);
			return ERROR_OK;
		}
		/* Or return information and error if not found. */
		command_print(CMD_CTX, "Signal not found!");
		return ERROR_FAIL;
	}
	/* For unknown parameter print error and return error code. */
	command_print(CMD_CTX, "Unknown parameter!");
	return ERROR_COMMAND_SYNTAX_ERROR;
}

static const struct command_registration oocd_interface_signal_commands[] = {
	{
		.name = "interface_signal",
		.handler = handle_oocd_interface_signal_command,
		.mode = COMMAND_ANY,
		.help = "List, Find, Remove and Add interface signal mask",
		.usage = "(add|del|find|list) signal_name [mask]",
	},
	COMMAND_REGISTRATION_DONE
};

int oocd_interface_signal_register_commands(struct command_context *ctx)
{
	return register_commands(ctx, NULL, oocd_interface_signal_commands);
}
