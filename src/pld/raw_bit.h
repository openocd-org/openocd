/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2022 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 ***************************************************************************/

#ifndef OPENOCD_PLD_RAW_BIN_H
#define OPENOCD_PLD_RAW_BIN_H

#include <stddef.h>
#include <stdint.h>

struct raw_bit_file {
	size_t length;
	uint8_t *data;
};

int cpld_read_raw_bit_file(struct raw_bit_file *bit_file, const char *filename);

#endif /* OPENOCD_PLD_RAW_BIN_H */
