/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NAND_LPC32XX_H
#define OPENOCD_FLASH_NAND_LPC32XX_H

enum lpc32xx_selected_controller {
	LPC32XX_NO_CONTROLLER,
	LPC32XX_MLC_CONTROLLER,
	LPC32XX_SLC_CONTROLLER,
};

struct lpc32xx_nand_controller {
	int osc_freq;
	enum lpc32xx_selected_controller selected_controller;
	int sw_write_protection;
	uint32_t sw_wp_lower_bound;
	uint32_t sw_wp_upper_bound;
};

#endif /* OPENOCD_FLASH_NAND_LPC32XX_H */
