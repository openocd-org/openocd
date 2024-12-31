/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Copyright (c) 2010 by David Brownell
 * Copyright (C) 2011 Tomasz Boleslaw CEDRO (http://www.tomek.cedro.info)
 */

#ifndef OPENOCD_TRANSPORT_TRANSPORT_H
#define OPENOCD_TRANSPORT_TRANSPORT_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "helper/bits.h"
#include "helper/command.h"
#include "helper/list.h"

#define TRANSPORT_JTAG                  BIT(0)
#define TRANSPORT_SWD                   BIT(1)
#define TRANSPORT_HLA_JTAG              BIT(2)
#define TRANSPORT_HLA_SWD               BIT(3)
#define TRANSPORT_DAPDIRECT_JTAG        BIT(4)
#define TRANSPORT_DAPDIRECT_SWD         BIT(5)
#define TRANSPORT_SWIM                  BIT(6)

/* mask for valid ID */
#define TRANSPORT_VALID_MASK            \
	(TRANSPORT_JTAG |                   \
	 TRANSPORT_SWD |                    \
	 TRANSPORT_HLA_JTAG |               \
	 TRANSPORT_HLA_SWD |                \
	 TRANSPORT_DAPDIRECT_JTAG |         \
	 TRANSPORT_DAPDIRECT_SWD |          \
	 TRANSPORT_SWIM)

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
struct transport {
	/**
	 * Each transport has a unique ID, used to select it
	 * from among the alternatives.
	 */
	unsigned int id;

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
	 * Optional. If defined, allows transport to override target
	 * name prior to initialisation.
	 *
	 * @returns ERROR_OK on success, or an error code on failure.
	 */
	int (*override_target)(const char **targetname);

	/**
	 * Transports are stored in a linked list.
	 */
	struct list_head lh;
};

int transport_register(struct transport *new_transport);

struct transport *get_current_transport(void);

const char *get_current_transport_name(void);

const char *transport_name(unsigned int id);

int transport_register_commands(struct command_context *ctx);

int allow_transports(struct command_context *ctx, unsigned int transport_ids,
	unsigned int transport_preferred_id);

bool transport_is_jtag(void);
bool transport_is_swd(void);
bool transport_is_dapdirect_jtag(void);
bool transport_is_dapdirect_swd(void);
bool transport_is_swim(void);

#if BUILD_HLADAPTER
bool transport_is_hla(void);
#else
static inline bool transport_is_hla(void)
{
	return false;
}
#endif

#endif /* OPENOCD_TRANSPORT_TRANSPORT_H */
