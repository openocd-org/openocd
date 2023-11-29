/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2022 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 ***************************************************************************/

#ifndef OPENOCD_PLD_CERTUS_H
#define OPENOCD_PLD_CERTUS_H

#include "lattice.h"

int lattice_certus_read_status(struct jtag_tap *tap, uint64_t *status, uint64_t out);
int lattice_certus_read_usercode(struct jtag_tap *tap, uint32_t *usercode, uint32_t out);
int lattice_certus_write_usercode(struct lattice_pld_device *lattice_device, uint32_t usercode);
int lattice_certus_load(struct lattice_pld_device *lattice_device, struct lattice_bit_file *bit_file);
int lattice_certus_connect_spi_to_jtag(struct lattice_pld_device *pld_device_info);
int lattice_certus_disconnect_spi_from_jtag(struct lattice_pld_device *pld_device_info);
int lattice_certus_get_facing_read_bits(struct lattice_pld_device *pld_device_info, unsigned int *facing_read_bits);
int lattice_certus_refresh(struct lattice_pld_device *lattice_device);

#endif /* OPENOCD_PLD_CERTUS_H */
