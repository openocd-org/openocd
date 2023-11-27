/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2022 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 ***************************************************************************/

#ifndef OPENOCD_PLD_ECP2_3_H
#define OPENOCD_PLD_ECP2_3_H

#include "lattice.h"

int lattice_ecp2_3_read_status(struct jtag_tap *tap, uint32_t *status, uint32_t out, bool do_idle);
int lattice_ecp2_3_read_usercode(struct jtag_tap *tap, uint32_t *usercode, uint32_t out);
int lattice_ecp2_3_write_usercode(struct lattice_pld_device *lattice_device, uint32_t usercode);
int lattice_ecp2_load(struct lattice_pld_device *lattice_device, struct lattice_bit_file *bit_file);
int lattice_ecp3_load(struct lattice_pld_device *lattice_device, struct lattice_bit_file *bit_file);
int lattice_ecp2_3_connect_spi_to_jtag(struct lattice_pld_device *pld_device_info);
int lattice_ecp2_3_disconnect_spi_from_jtag(struct lattice_pld_device *pld_device_info);
int lattice_ecp2_3_get_facing_read_bits(struct lattice_pld_device *pld_device_info, unsigned int *facing_read_bits);
int lattice_ecp2_3_refresh(struct lattice_pld_device *lattice_device);

#endif /* OPENOCD_PLD_ECP2_3_H */
