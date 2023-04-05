/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NAND_LPC3180_H
#define OPENOCD_FLASH_NAND_LPC3180_H

enum lpc3180_selected_controller {
	LPC3180_NO_CONTROLLER,
	LPC3180_MLC_CONTROLLER,
	LPC3180_SLC_CONTROLLER,
};

struct lpc3180_nand_controller {
	int osc_freq;
	enum lpc3180_selected_controller selected_controller;
	int is_bulk;
	int sw_write_protection;
	uint32_t sw_wp_lower_bound;
	uint32_t sw_wp_upper_bound;
};

#endif /* OPENOCD_FLASH_NAND_LPC3180_H */
