/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2022 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 ***************************************************************************/

#ifndef OPENOCD_PLD_LATTICE_BIT_H
#define OPENOCD_PLD_LATTICE_BIT_H

#include "helper/types.h"
#include "raw_bit.h"


struct lattice_bit_file {
	struct raw_bit_file raw_bit;
	size_t offset;
	uint32_t idcode;
	const char *part; /* reuses memory in raw_bit_file */
	bool has_id;
};

enum lattice_family_e {
	LATTICE_ECP2,
	LATTICE_ECP3,
	LATTICE_ECP5,
	LATTICE_CERTUS,
	LATTICE_UNKNOWN,
};

int lattice_read_file(struct lattice_bit_file *bit_file, const char *filename, enum lattice_family_e family);

#endif /* OPENOCD_PLD_LATTICE_BIT_H */
