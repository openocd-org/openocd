// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (c) 2010 by David Brownell
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/** @file
 * Infrastructure for specifying and managing the transport protocol
 * used in a given debug or programming session.
 *
 * Examples of "debug-capable" transports are JTAG or SWD.
 * Additionally, JTAG supports boundary scan testing.
 *
 * Examples of "programming-capable" transports include SPI or UART;
 * those are used (often mediated by a ROM bootloader) for ISP style
 * programming, to perform an initial load of code into flash, or
 * sometimes into SRAM.  Target code could use "variant" options to
 * decide how to use such protocols.  For example, Cortex-M3 cores
 * from TI/Luminary and from NXP use different protocols for for
 * UART or SPI based firmware loading.
 *
 * As a rule, there are protocols layered on top of the transport.
 * For example, different chip families use JTAG in different ways
 * for debugging.  Also, each family that supports programming over
 * a UART link for initial firmware loading tends to define its own
 * messaging and error handling.
 */

#include <helper/align.h>
#include <helper/bits.h>
#include <helper/list.h>
#include <helper/log.h>
#include <helper/replacements.h>
#include <transport/transport.h>

extern struct command_context *global_cmd_ctx;

/*-----------------------------------------------------------------------*/

/*
 * Infrastructure internals
 */

/** List of transports known to OpenOCD. */
static const struct {
	unsigned int id;
	const char *name;
	const char *full_name;
	const char *deprecated_name;
} transport_names[] = {
	{ TRANSPORT_JTAG,           "jtag", "jtag",             NULL,             },
	{ TRANSPORT_SWD,            "swd",  "swd",              NULL,             },
	{ TRANSPORT_HLA_JTAG,       "jtag", "jtag (hla)",       "hla_jtag",       },
	{ TRANSPORT_HLA_SWD,        "swd",  "swd (hla)",        "hla_swd",        },
	{ TRANSPORT_DAPDIRECT_JTAG, "jtag", "jtag (dapdirect)", "dapdirect_jtag", },
	{ TRANSPORT_DAPDIRECT_SWD,  "swd",  "swd (dapdirect)",  "dapdirect_swd",  },
	{ TRANSPORT_SWIM,           "swim", "swim",             NULL,             },
};

/** List of transports registered in OpenOCD, alphabetically sorted per name. */
static OOCD_LIST_HEAD(transport_list);

/**
 * Bitmask of transport IDs which the currently selected debug adapter supports.
 * This is declared by the time that adapter is fully set up.
 */
static unsigned int allowed_transports;

/**
 * Transport ID auto-selected when not specified by the user.
 */
static unsigned int preferred_transport;

/**
 * Adapter supports a single transport; it has been auto-selected
 */
static bool transport_single_is_autoselected;

/** * The transport being used for the current OpenOCD session.  */
static struct transport *session;

const char *transport_name(unsigned int id)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(transport_names); i++)
		if (id == transport_names[i].id)
			return transport_names[i].name;

	return NULL;
}

static const char *transport_full_name(unsigned int id)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(transport_names); i++)
		if (id == transport_names[i].id)
			return transport_names[i].full_name;

	return NULL;
}

static const char *transport_deprecated_name(unsigned int id)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(transport_names); i++)
		if (id == transport_names[i].id)
			return transport_names[i].deprecated_name;

	return NULL;
}

static bool is_transport_id_valid(unsigned int id)
{
	return (id != 0) && ((id & ~TRANSPORT_VALID_MASK) == 0) && IS_PWR_OF_2(id);
}

static int transport_select(struct command_context *ctx, unsigned int transport_id)
{
	/* name may only identify a known transport;
	 * caller guarantees session's transport isn't yet set.*/
	struct transport *t;
	list_for_each_entry(t, &transport_list, lh) {
		if (t->id == transport_id) {
			int retval = t->select(ctx);
			/* select() registers commands specific to this
			 * transport, and may also reset the link, e.g.
			 * forcing it to JTAG or SWD mode.
			 */
			if (retval == ERROR_OK)
				session = t;
			else
				LOG_ERROR("Error selecting '%s' as transport",
					transport_full_name(transport_id));
			return retval;
		}
	}

	LOG_ERROR("No transport named '%s' is available.",
		transport_full_name(transport_id));
	return ERROR_FAIL;
}

/**
 * Called by debug adapter drivers, or affiliated Tcl config scripts,
 * to declare the set of transports supported by an adapter.  When
 * there is only one member of that set, it is automatically selected.
 */
int allow_transports(struct command_context *ctx, unsigned int transport_ids,
	unsigned int transport_preferred_id)
{
	if (allowed_transports || session) {
		LOG_ERROR("Can't modify the set of allowed transports.");
		return ERROR_FAIL;
	}

	/* validate the values in transport_ids and transport_preferred_id */
	if (transport_ids == 0 || (transport_ids & ~TRANSPORT_VALID_MASK) != 0) {
		LOG_ERROR("BUG: Unknown transport IDs %lu", transport_ids & ~TRANSPORT_VALID_MASK);
		return ERROR_FAIL;
	}

	if ((transport_ids & transport_preferred_id) == 0
		|| !IS_PWR_OF_2(transport_preferred_id)) {
		LOG_ERROR("BUG: Invalid adapter transport_preferred_id");
		return ERROR_FAIL;
	}

	unsigned int mask = transport_ids &
		(TRANSPORT_JTAG | TRANSPORT_HLA_JTAG | TRANSPORT_DAPDIRECT_JTAG);
	if (mask && !IS_PWR_OF_2(mask)) {
		LOG_ERROR("BUG: Multiple JTAG transports");
		return ERROR_FAIL;
	}

	mask = transport_ids &
		(TRANSPORT_SWD | TRANSPORT_HLA_SWD | TRANSPORT_DAPDIRECT_SWD);
	if (mask && !IS_PWR_OF_2(mask)) {
		LOG_ERROR("BUG: Multiple SWD transports");
		return ERROR_FAIL;
	}

	allowed_transports = transport_ids;
	preferred_transport = transport_preferred_id;

	/* autoselect if there's no choice ... */
	if (IS_PWR_OF_2(transport_ids)) {
		LOG_DEBUG("only one transport option; autoselecting '%s'", transport_name(transport_ids));
		transport_single_is_autoselected = true;
		return transport_select(ctx, transport_ids);
	}

	return ERROR_OK;
}

/**
 * Registers a transport.  There are general purpose transports
 * (such as JTAG), as well as relatively proprietary ones which are
 * specific to a given chip (or chip family).
 *
 * Code implementing a transport needs to register it before it can
 * be selected and then activated.  This is a dynamic process, so
 * that chips (and families) can define transports as needed (without
 * needing error-prone static tables).
 *
 * @param new_transport the transport being registered.  On a
 * successful return, this memory is owned by the transport framework.
 *
 * @returns ERROR_OK on success, else a fault code.
 */
int transport_register(struct transport *new_transport)
{
	struct transport *t;

	if (!is_transport_id_valid(new_transport->id)) {
		LOG_ERROR("invalid transport ID 0x%x", new_transport->id);
		return ERROR_FAIL;
	}

	list_for_each_entry(t, &transport_list, lh) {
		if (t->id == new_transport->id) {
			LOG_ERROR("transport '%s' already registered",
					  transport_name(t->id));
			return ERROR_FAIL;
		}
	}

	if (!new_transport->select || !new_transport->init)
		LOG_ERROR("invalid transport %s",
				  transport_full_name(new_transport->id));

	/* splice this into the list, sorted in alphabetic order */
	list_for_each_entry(t, &transport_list, lh) {
		if (strcmp(transport_name(t->id),
				   transport_name(new_transport->id)) >= 0)
			break;
	}
	list_add_tail(&new_transport->lh, &t->lh);

	LOG_DEBUG("register '%s' (ID %d)",
			  transport_full_name(new_transport->id), new_transport->id);

	return ERROR_OK;
}

/**
 * Returns the transport currently being used by this debug or
 * programming session.
 *
 * @returns handle to the read-only transport entity.
 */
struct transport *get_current_transport(void)
{
	/* REVISIT -- constify */
	return session;
}

const char *get_current_transport_name(void)
{
	if (!session || !is_transport_id_valid(session->id))
		return NULL;

	return transport_full_name(session->id);
}

/*-----------------------------------------------------------------------*/

/*
 * Infrastructure for Tcl interface to transports.
 */

COMMAND_HANDLER(handle_transport_init)
{
	LOG_DEBUG("%s", __func__);
	if (!session) {
		LOG_ERROR("session transport was not selected. Use 'transport select <transport>'");

		/* no session transport configured, print transports then fail */
		LOG_ERROR("Transports available:");
		for (unsigned int i = BIT(0); i & TRANSPORT_VALID_MASK; i <<= 1) {
			if (i & allowed_transports)
				LOG_ERROR("%s", transport_full_name(i));
		}
		return ERROR_FAIL;
	}

	if (transport_single_is_autoselected)
		LOG_WARNING("DEPRECATED: auto-selecting transport \"%s\". "
			"Use 'transport select %s' to suppress this message.",
			transport_full_name(session->id), transport_name(session->id));

	return session->init(CMD_CTX);
}

COMMAND_HANDLER(handle_transport_list)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(CMD, "The following transports are available:");

	struct transport *t;
	const char *prev_name = NULL;
	/* list is sorted, don't print duplicated transport names */
	list_for_each_entry(t, &transport_list, lh) {
		const char *name = transport_name(t->id);
		if (!prev_name || strcmp(prev_name, name))
			command_print(CMD, "\t%s", name);
		prev_name = name;
	}

	return ERROR_OK;
}

/**
 * Implements the Tcl "transport select" command, choosing the
 * transport to be used in this debug session from among the
 * set supported by the debug adapter being used.  Return value
 * is scriptable (allowing "if swd then..." etc).
 */
COMMAND_HANDLER(handle_transport_select)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 0) {
		/* autoselect if necessary, then return/display current config */
		if (!session) {
			if (!allowed_transports) {
				command_print(CMD, "Debug adapter does not support any transports? Check config file order.");
				return ERROR_FAIL;
			}
			LOG_WARNING("DEPRECATED: auto-selecting transport \"%s\". "
				"Use 'transport select %s' to suppress this message.",
				transport_full_name(preferred_transport),
				transport_name(preferred_transport));
			int retval = transport_select(CMD_CTX, preferred_transport);
			if (retval != ERROR_OK)
				return retval;
		}
		command_print(CMD, "%s", transport_full_name(session->id));
		return ERROR_OK;
	}

	/* assign transport */
	if (session) {
		if (!strcmp(transport_name(session->id), CMD_ARGV[0])
			|| (transport_deprecated_name(session->id)
				&& !strcmp(transport_deprecated_name(session->id), CMD_ARGV[0]))) {
			if (transport_single_is_autoselected) {
				/* Nothing to do, but also nothing to complain */
				transport_single_is_autoselected = false;
				return ERROR_OK;
			}
			LOG_WARNING("Transport \"%s\" was already selected", CMD_ARGV[0]);
			return ERROR_OK;
		}
		command_print(CMD, "Can't change session's transport after the initial selection was made");
		return ERROR_FAIL;
	}

	/* Is this transport supported by our debug adapter?
	 * Example, "JTAG-only" means SWD is not supported.
	 *
	 * NOTE:  requires adapter to have been set up, with
	 * transports declared via C.
	 */
	if (!allowed_transports) {
		command_print(CMD, "Debug adapter doesn't support any transports?");
		return ERROR_FAIL;
	}

	for (unsigned int i = BIT(0); i & TRANSPORT_VALID_MASK; i <<= 1) {
		if (!(i & allowed_transports))
			continue;

		if (!strcmp(transport_name(i), CMD_ARGV[0]))
			return transport_select(CMD_CTX, i);

		if (transport_deprecated_name(i)
			&& !strcmp(transport_deprecated_name(i), CMD_ARGV[0])) {
			LOG_WARNING("DEPRECATED! use 'transport select %s', not 'transport select %s'",
				transport_name(i), transport_deprecated_name(i));
			return transport_select(CMD_CTX, i);
		}
	}

	command_print(CMD, "Debug adapter doesn't support '%s' transport", CMD_ARGV[0]);
	return ERROR_FAIL;
}

static const struct command_registration transport_commands[] = {
	{
		.name = "init",
		.handler = handle_transport_init,
		/* this would be COMMAND_CONFIG ... except that
		 * it needs to trigger event handlers that may
		 * require COMMAND_EXEC ...
		 */
		.mode = COMMAND_ANY,
		.help = "Initialize this session's transport",
		.usage = ""
	},
	{
		.name = "list",
		.handler = handle_transport_list,
		.mode = COMMAND_ANY,
		.help = "list all built-in transports",
		.usage = ""
	},
	{
		.name = "select",
		.handler = handle_transport_select,
		.mode = COMMAND_ANY,
		.help = "Select this session's transport",
		.usage = "[transport_name]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration transport_group[] = {
	{
		.name = "transport",
		.mode = COMMAND_ANY,
		.help = "Transport command group",
		.chain = transport_commands,
		.usage = ""
	},
	COMMAND_REGISTRATION_DONE
};

int transport_register_commands(struct command_context *ctx)
{
	return register_commands(ctx, NULL, transport_group);
}
