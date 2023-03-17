/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_PLD_XILINX_BIT_H
#define OPENOCD_PLD_XILINX_BIT_H

#include "helper/types.h"

struct xilinx_bit_file {
	uint8_t unknown_header[13];
	uint8_t *source_file;
	uint8_t *part_name;
	uint8_t *date;
	uint8_t *time;
	uint32_t length;
	uint8_t *data;
};

int xilinx_read_bit_file(struct xilinx_bit_file *bit_file, const char *filename);

#endif /* OPENOCD_PLD_XILINX_BIT_H */
