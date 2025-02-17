/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2022 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 ***************************************************************************/

#ifndef OPENOCD_PLD_LATTICE_H
#define OPENOCD_PLD_LATTICE_H

#include <jtag/jtag.h>
#include "pld.h"
#include "lattice_bit.h"

#define BYPASS 0xFF

struct lattice_pld_device {
	struct jtag_tap *tap;
	size_t preload_length;
	enum lattice_family_e family;
};

int lattice_set_instr(struct jtag_tap *tap, uint8_t new_instr, enum tap_state endstate);
int lattice_read_u32_register(struct jtag_tap *tap, uint8_t cmd, uint32_t *in_val,
							uint32_t out_val, bool do_idle);
int lattice_read_u64_register(struct jtag_tap *tap, uint8_t cmd, uint64_t *in_val,
							uint64_t out_val);
int lattice_verify_usercode(struct lattice_pld_device *lattice_device, uint32_t out,
						uint32_t expected, uint32_t mask);
int lattice_verify_status_register_u32(struct lattice_pld_device *lattice_device, uint32_t out,
						uint32_t expected, uint32_t mask, bool do_idle);
int lattice_verify_status_register_u64(struct lattice_pld_device *lattice_device, uint64_t out,
						uint64_t expected, uint64_t mask);
int lattice_preload(struct lattice_pld_device *lattice_device);

#endif /* OPENOCD_PLD_LATTICE_H */
