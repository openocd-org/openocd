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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef ETB_H
#define ETB_H

#include "command.h"
#include "target.h"
#include "register.h"
#include "arm_jtag.h"

#include "etb.h"
#include "etm.h"

/* ETB registers */
enum
{
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

typedef struct etb_s
{
	etm_context_t *etm_ctx;
	jtag_tap_t *tap;
	int cur_scan_chain;
	reg_cache_t *reg_cache;
	
	/* ETB parameters */
	int ram_depth;
	int ram_width;
} etb_t;

typedef struct etb_reg_s
{
	int addr;
	etb_t *etb;
} etb_reg_t;

extern etm_capture_driver_t etb_capture_driver;

extern reg_cache_t* etb_build_reg_cache(etb_t *etb);
extern int etb_read_reg(reg_t *reg);
extern int etb_write_reg(reg_t *reg, u32 value);
extern int etb_read_reg_w_check(reg_t *reg, u8* check_value, u8* check_mask);
extern int etb_store_reg(reg_t *reg);
extern int etb_set_reg(reg_t *reg, u32 value);
extern int etb_set_reg_w_exec(reg_t *reg, u8 *buf);

extern int etb_register_commands(struct command_context_s *cmd_ctx);

#endif /* ETB_H */
