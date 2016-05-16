/***************************************************************************
 *   Copyright (C) 2011 by Martin Schmoelzer                               *
 *   <martin.schmoelzer@student.tuwien.ac.at>                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

/**
 * @file
 * Definition of the commands supported by the OpenULINK firmware.
 *
 * Basically, two types of commands can be distinguished:
 *  - Commands with fixed payload size
 *  - Commands with variable payload size
 *
 * SCAN commands (in all variations) carry payloads of variable size, all
 * other commands carry payloads of fixed size.
 *
 * In the case of SCAN commands, the payload size (n) is calculated by
 * dividing the scan_size_bits variable by 8, rounding up the result.
 *
 * Offset zero always contains the command ID.
 *
 ****************************************************************************
 * CMD_SCAN_IN, CMD_SLOW_SCAN_IN:                                           *
 *                                                                          *
 * OUT:                                                                     *
 * offset 1: scan_size_bytes                                                *
 * offset 2: bits_last_byte                                                 *
 * offset 3: tms_count_start + tms_count_end                                *
 * offset 4: tms_sequence_start                                             *
 * offset 5: tms_sequence_end                                               *
 *                                                                          *
 * IN:                                                                      *
 * offset 0..n: TDO data                                                    *
 ****************************************************************************
 * CMD_SCAN_OUT, CMD_SLOW_SCAN_OUT:                                         *
 *                                                                          *
 * OUT:                                                                     *
 * offset 1: scan_size_bytes                                                *
 * offset 2: bits_last_byte                                                 *
 * offset 3: tms_count_start + tms_count_end                                *
 * offset 4: tms_sequence_start                                             *
 * offset 5: tms_sequence_end                                               *
 * offset 6..x: TDI data                                                    *
 ****************************************************************************
 * CMD_SCAN_IO, CMD_SLOW_SCAN_IO:                                           *
 *                                                                          *
 * OUT:                                                                     *
 * offset 1: scan_size_bytes                                                *
 * offset 2: bits_last_byte                                                 *
 * offset 3: tms_count_start + tms_count_end                                *
 * offset 4: tms_sequence_start                                             *
 * offset 5: tms_sequence_end                                               *
 * offset 6..x: TDI data                                                    *
 *                                                                          *
 * IN:                                                                      *
 * offset 0..n: TDO data                                                    *
 ****************************************************************************
 * CMD_CLOCK_TMS, CMD_SLOW_CLOCK_TMS:                                       *
 *                                                                          *
 * OUT:                                                                     *
 * offset 1: tms_count                                                      *
 * offset 2: tms_sequence                                                   *
 ****************************************************************************
 * CMD_CLOCK_TCK, CMD_SLOW_CLOCK_TCK:                                       *
 *                                                                          *
 * OUT:                                                                     *
 * offset 1: low byte of tck_count                                          *
 * offset 2: high byte of tck_count                                         *
 ****************************************************************************
 * CMD_CLOCK_SLEEP_US:                                                      *
 *                                                                          *
 * OUT:                                                                     *
 * offset 1: low byte of sleep_us                                           *
 * offset 2: high byte of sleep_us                                          *
 ****************************************************************************
 * CMD_CLOCK_SLEEP_MS:                                                      *
 *                                                                          *
 * OUT:                                                                     *
 * offset 1: low byte of sleep_ms                                           *
 * offset 2: high byte of sleep_ms                                          *
 ****************************************************************************
 * CMD_GET_SIGNALS:                                                         *
 *                                                                          *
 * IN:                                                                      *
 * offset 0: current state of input signals                                 *
 * offset 1: current state of output signals                                *
 ****************************************************************************
 * CMD_SET_SIGNALS:                                                         *
 *                                                                          *
 * OUT:                                                                     *
 * offset 1: signals that should be de-asserted                             *
 * offset 2: signals that should be asserted                                *
 ****************************************************************************
 * CMD_CONFIGURE_TCK_FREQ:                                                  *
 *                                                                          *
 * OUT:                                                                     *
 * offset 1: delay value for scan_in function                               *
 * offset 2: delay value for scan_out function                              *
 * offset 3: delay value for scan_io function                               *
 * offset 4: delay value for clock_tck function                             *
 * offset 5: delay value for clock_tms function                             *
 ****************************************************************************
 * CMD_SET_LEDS:                                                            *
 *                                                                          *
 * OUT:                                                                     *
 * offset 1: LED states:                                                    *
 *           Bit 0: turn COM LED on                                         *
 *           Bit 1: turn RUN LED on                                         *
 *           Bit 2: turn COM LED off                                        *
 *           Bit 3: turn RUN LED off                                        *
 *           Bits 7..4: Reserved                                            *
 ****************************************************************************
 * CMD_TEST:                                                                *
 *                                                                          *
 * OUT:                                                                     *
 * offset 1: unused dummy value                                             *
 ****************************************************************************
 */

#ifndef __MSGTYPES_H
#define __MSGTYPES_H

/*
 * Command IDs:
 *
 * Bits 7..6: Reserved, should always be zero
 * Bits 5..0: Command ID. There are 62 usable IDs. Of this 63 available IDs,
 *            the IDs 0x00..0x1F are commands with variable payload size,
 *            the IDs 0x20..0x3F are commands with fixed payload size.
 */

#define CMD_ID_MASK             0x3F

/* Commands with variable payload size */
#define CMD_SCAN_IN             0x00
#define CMD_SLOW_SCAN_IN        0x01
#define CMD_SCAN_OUT            0x02
#define CMD_SLOW_SCAN_OUT       0x03
#define CMD_SCAN_IO             0x04
#define CMD_SLOW_SCAN_IO        0x05

/* Commands with fixed payload size */
#define CMD_CLOCK_TMS           0x20
#define CMD_SLOW_CLOCK_TMS      0x21
#define CMD_CLOCK_TCK           0x22
#define CMD_SLOW_CLOCK_TCK      0x23
#define CMD_SLEEP_US            0x24
#define CMD_SLEEP_MS            0x25
#define CMD_GET_SIGNALS         0x26
#define CMD_SET_SIGNALS         0x27
#define CMD_CONFIGURE_TCK_FREQ  0x28
#define CMD_SET_LEDS            0x29
#define CMD_TEST                0x2A

/* JTAG signal definition for jtag_get_signals() -- Input signals! */
#define SIGNAL_TDO      (1<<0)
#define SIGNAL_BRKOUT   (1<<1)
#define SIGNAL_TRAP     (1<<2)
#define SIGNAL_RTCK     (1<<3)

/* JTAG signal definition for jtag_get_signals() -- Output signals! */
#define SIGNAL_TDI      (1<<0)
#define SIGNAL_TMS      (1<<1)
#define SIGNAL_TCK      (1<<2)
#define SIGNAL_TRST     (1<<3)
#define SIGNAL_BRKIN    (1<<4)
#define SIGNAL_RESET    (1<<5)
#define SIGNAL_OCDSE    (1<<6)

/* LED definitions for CMD_SET_LEDS and CMD_CLEAR_LEDS commands */
#define COM_LED_ON      (1<<0)
#define RUN_LED_ON      (1<<1)
#define COM_LED_OFF     (1<<2)
#define RUN_LED_OFF     (1<<3)

#endif
