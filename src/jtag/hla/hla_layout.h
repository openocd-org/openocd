/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

#ifndef _HL_LAYOUT_H
#define _HL_LAYOUT_H

/** */
struct hl_interface_s;
struct hl_interface_param_s;

/** */
extern struct hl_layout_api_s stlink_usb_layout_api;
extern struct hl_layout_api_s icdi_usb_layout_api;

/** */
struct hl_layout_api_s {
	/** */
	int (*open) (struct hl_interface_param_s *param, void **handle);
	/** */
	int (*close) (void *handle);
	/** */
	int (*reset) (void *handle);
	/** */
	int (*assert_srst) (void *handle, int srst);
	/** */
	int (*run) (void *handle);
	/** */
	int (*halt) (void *handle);
	/** */
	int (*step) (void *handle);
	/** */
	int (*read_regs) (void *handle);
	/** */
	int (*read_reg) (void *handle, int num, uint32_t *val);
	/** */
	int (*write_reg) (void *handle, int num, uint32_t val);
	/** */
	int (*read_mem) (void *handle, uint32_t addr, uint32_t size,
			uint32_t count, uint8_t *buffer);
	/** */
	int (*write_mem) (void *handle, uint32_t addr, uint32_t size,
			uint32_t count, const uint8_t *buffer);
	/** */
	int (*write_debug_reg) (void *handle, uint32_t addr, uint32_t val);
	/**
	 * Read the idcode of the target connected to the adapter
	 *
	 * If the adapter doesn't support idcode retrieval, this callback should
	 * store 0 to indicate a wildcard match.
	 *
	 * @param handle A pointer to the device-specific handle
	 * @param idcode Storage for the detected idcode
	 * @returns ERROR_OK on success, or an error code on failure.
	 */
	int (*idcode) (void *handle, uint32_t *idcode);
	/** */
	enum target_state (*state) (void *handle);
};

/** */
struct hl_layout {
	/** */
	char *name;
	/** */
	int (*open) (struct hl_interface_s *adapter);
	/** */
	int (*close) (struct hl_interface_s *adapter);
	/** */
	struct hl_layout_api_s *api;
};

/** */
const struct hl_layout *hl_layout_get_list(void);
/** */
int hl_layout_init(struct hl_interface_s *adapter);

#endif /* _HL_LAYOUT_H */
