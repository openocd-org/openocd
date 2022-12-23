/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Copyright (C) 2016-2017 by Marc Schink <dev@zapb.de>
 */

#ifndef OPENOCD_SERVER_RTT_SERVER_H
#define OPENOCD_SERVER_RTT_SERVER_H

#include <helper/command.h>

int rtt_server_register_commands(struct command_context *ctx);

#endif /* OPENOCD_SERVER_RTT_SERVER_H */
