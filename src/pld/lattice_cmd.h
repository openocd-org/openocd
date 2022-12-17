/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2022 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 ***************************************************************************/

#ifndef OPENOCD_PLD_LATTICE_CMD_H
#define OPENOCD_PLD_LATTICE_CMD_H

#define ISC_ERASE            0x0E
#define ISC_DISABLE          0x26
#define PROGRAM_SPI          0x3A
#define LSC_READ_STATUS      0x3C
#define LSC_INIT_ADDRESS     0x46
#define LSC_REFRESH          0x79
#define LSC_BITSTREAM_BURST  0x7A
#define READ_USERCODE        0xC0
#define ISC_ENABLE           0xC6

#endif /* OPENOCD_PLD_LATTICE_CMD_H */
