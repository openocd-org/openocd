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
#include <jtag/interface.h>
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
	if (snlen < OOCD_INTERFACE_SIGNAL_NAME_MINLEN || snlen > OOCD_INTERFACE_SIGNAL_NAME_MAXLEN) {
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
	newsignal->name = (char *)calloc(1, snlen+1);
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

