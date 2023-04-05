/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2013 by Andes Technology                                *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_AICE_AICE_PIPE_H
#define OPENOCD_JTAG_AICE_AICE_PIPE_H

#include <helper/types.h>

#define set_u32(buffer, value) h_u32_to_le((uint8_t *)buffer, value)
#define set_u16(buffer, value) h_u16_to_le((uint8_t *)buffer, value)
#define get_u32(buffer) le_to_h_u32((const uint8_t *)buffer)
#define get_u16(buffer) le_to_h_u16((const uint8_t *)buffer)

extern struct aice_port_api_s aice_pipe;

#endif /* OPENOCD_JTAG_AICE_AICE_PIPE_H */
