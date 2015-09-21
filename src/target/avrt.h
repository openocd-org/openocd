/***************************************************************************
 *   Copyright (C) 2009 by Simon Qian                                      *
 *   SimonQian@SimonQian.com                                               *
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

#ifndef OPENOCD_TARGET_AVRT_H
#define OPENOCD_TARGET_AVRT_H

#include <jtag/jtag.h>

struct mcu_jtag {
	struct jtag_tap *tap;
};

struct avr_common {
	struct mcu_jtag jtag_info;
};

int mcu_execute_queue(void);
int avr_jtag_sendinstr(struct jtag_tap *tap, uint8_t *ir_in, uint8_t ir_out);
int avr_jtag_senddat(struct jtag_tap *tap, uint32_t *dr_in, uint32_t dr_out,
		int len);

#endif /* OPENOCD_TARGET_AVRT_H */
