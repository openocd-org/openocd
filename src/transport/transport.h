/*
 * Transport subsystem header file for OpenOCD.
 *
 * Copyright (c) 2010 by David Brownell
 *
 * Copyright (C) 2011-2012 Tomasz Boleslaw CEDRO
 * cederom@tlen.pl, http://www.tomek.cedro.info
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef OOCD_TRANSPORT_H
#define OOCD_TRANSPORT_H

/**
 * Wrapper for transport lifecycle operations.
 *
 * OpenOCD talks to targets through some kind of debugging
 * or programming adapter, using some protocol that probably
 * has target-specific aspects.
 *
 * A "transport" reflects electrical protocol to the target,
 * e..g jtag, swd, spi, uart, ... NOT the messaging protocols
 * layered over it (e.g. JTAG has eICE, CoreSight, Nexus, OnCE,
 * and more).
 *
 * In addition to the lifecycle operations packaged by this
 * structure, a transport also involves  an interface supported
 * by debug adapters and used by components such as debug targets.
 * For non-debug transports,  there may be interfaces used to
 * write to flash chips.
 */
typedef struct oocd_transport {
	/**
	 * Each transport has a unique name, used to select it
	 * from among the alternatives.  Examples might include
	 * "jtag", * "swd", "AVR_ISP" and more.
	 */
	const char *name;

	/**
	 * When a transport is selected, this method registers
	 * its commands and activates the transport (e.g. resets
	 * the link).
	 *
	 * After those commands are registered, they will often
	 * be used for further configuration of the debug link.
	 */
	int (*select)(struct command_context *ctx);

	/**
	 * server startup uses this method to validate transport
	 * configuration.  (For example, with JTAG this interrogates
	 * the scan chain against the list of expected TAPs.)
	 */
	int (*init)(struct command_context *ctx);

	/**
	 * Quit makes resources free and destroys transport.
	 */
	int (*quit)(struct command_context *ctx);

	/**
	 * Transports are stored in a singly linked list.
	 */
	struct oocd_transport *next;
} oocd_transport_t;

int oocd_transport_select(struct command_context *ctx, const char *name);
int oocd_transport_allow(struct command_context *ctx, const char **vector);
bool oocd_transport_declared(void);
int oocd_transport_register(oocd_transport_t *new_transport);
oocd_transport_t *oocd_transport_current_get(void);
int oocd_transport_register_commands(struct command_context *ctx);
int oocd_transport_select_jim(Jim_Interp *interp, int argc, Jim_Obj * const *argv);

COMMAND_HELPER(oocd_transport_list_parse, char ***vector);

#endif
