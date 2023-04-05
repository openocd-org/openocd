/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *                                                                         *
 * Copyright (C) ST-Ericsson SA 2011                                       *
 * Author: Michel Jaouen <michel.jaouen@stericsson.com> for ST-Ericsson.   *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_SMP_H
#define OPENOCD_TARGET_SMP_H

#include <helper/list.h>
#include "server/server.h"

#define foreach_smp_target(pos, head) \
	list_for_each_entry(pos, head, lh)

#define foreach_smp_target_direction(forward, pos, head) \
	list_for_each_entry_direction(forward, pos, head, lh)

extern const struct command_registration smp_command_handlers[];

/* DEPRECATED */
int gdb_read_smp_packet(struct connection *connection,
		char const *packet, int packet_size);
/* DEPRECATED */
int gdb_write_smp_packet(struct connection *connection,
		char const *packet, int packet_size);

#endif /* OPENOCD_TARGET_SMP_H */
