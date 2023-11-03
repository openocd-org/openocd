/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_PLD_VIRTEX2_H
#define OPENOCD_PLD_VIRTEX2_H

#include <jtag/jtag.h>

#define VIRTEX2_MAX_USER_INSTRUCTIONS 4

struct virtex2_command_set {
	uint64_t cfg_out;
	uint64_t cfg_in;
	uint64_t jprog_b;
	uint64_t jstart;
	uint64_t jshutdown;
	uint64_t bypass;
	uint64_t user[VIRTEX2_MAX_USER_INSTRUCTIONS];
	unsigned int num_user;
};

struct virtex2_pld_device {
	struct jtag_tap *tap;
	int no_jstart;
	struct virtex2_command_set command_set;
};

#endif /* OPENOCD_PLD_VIRTEX2_H */
