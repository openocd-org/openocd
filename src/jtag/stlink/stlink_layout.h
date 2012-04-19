/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
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

#ifndef _STLINK_LAYOUT_H_
#define _STLINK_LAYOUT_H_

/** */
struct stlink_interface_s;
struct stlink_interface_param_s;

/** */
extern struct stlink_layout_api_s stlink_usb_layout_api;

/** */
struct stlink_layout_api_s {
	/** */
	int (*open) (struct stlink_interface_param_s *param, void **fd);
	/** */
	int (*close) (void *fd);
	/** */
	int (*reset) (void *fd);
	/** */
	int (*assert_srst) (void *fd, int srst);
	/** */
	int (*run) (void *fd);
	/** */
	int (*halt) (void *fd);
	/** */
	int (*step) (void *fd);
	/** */
	int (*read_regs) (void *fd);
	/** */
	int (*read_reg) (void *fd, int num, uint32_t *val);
	/** */
	int (*write_reg) (void *fd, int num, uint32_t val);
	/** */
	int (*read_mem8) (void *handle, uint32_t addr, uint16_t len,
			   uint8_t *buffer);
	/** */
	int (*write_mem8) (void *handle, uint32_t addr, uint16_t len,
			    const uint8_t *buffer);
	/** */
	int (*read_mem32) (void *handle, uint32_t addr, uint16_t len,
			   uint8_t *buffer);
	/** */
	int (*write_mem32) (void *handle, uint32_t addr, uint16_t len,
			    const uint8_t *buffer);
	/** */
	int (*write_debug_reg) (void *handle, uint32_t addr, uint32_t val);
	/** */
	int (*idcode) (void *fd, uint32_t *idcode);
	/** */
	enum target_state (*state) (void *fd);
};

/** */
struct stlink_layout {
	/** */
	char *name;
	/** */
	int type;
	/** */
	int (*open) (struct stlink_interface_s *stlink_if);
	/** */
	int (*close) (struct stlink_interface_s *stlink_if);
	/** */
	struct stlink_layout_api_s *api;
};

/** */
const struct stlink_layout *stlink_layout_get_list(void);
/** */
int stlink_layout_init(struct stlink_interface_s *stlink_if);

#endif
