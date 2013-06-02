/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef ETB_H
#define ETB_H

/* ETB registers */
enum {
	ETB_ID = 0x00,
	ETB_RAM_DEPTH = 0x01,
	ETB_RAM_WIDTH = 0x02,
	ETB_STATUS = 0x03,
	ETB_RAM_DATA = 0x04,
	ETB_RAM_READ_POINTER = 0x05,
	ETB_RAM_WRITE_POINTER = 0x06,
	ETB_TRIGGER_COUNTER = 0x07,
	ETB_CTRL = 0x08,
};

struct etb {
	struct etm_context *etm_ctx;
	struct jtag_tap *tap;
	uint32_t cur_scan_chain;
	struct reg_cache *reg_cache;

	/* ETB parameters */
	uint32_t ram_depth;
	uint32_t ram_width;

	/** how much trace buffer to fill after trigger */
	unsigned trigger_percent;
};

struct etb_reg {
	uint32_t addr;
	struct etb *etb;
};

extern struct etm_capture_driver etb_capture_driver;

struct reg_cache *etb_build_reg_cache(struct etb *etb);

#endif /* ETB_H */
