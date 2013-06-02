/***************************************************************************
 *   Copyright (C) 2009 - 2010 by Simon Qian <SimonQian@SimonQian.com>     *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>

#include "../versaloon_include.h"
#include "../versaloon.h"
#include "../versaloon_internal.h"
#include "usbtoxxx.h"
#include "usbtoxxx_internal.h"

RESULT usbtogpio_init(uint8_t interface_index)
{
	return usbtoxxx_init_command(USB_TO_GPIO, interface_index);
}

RESULT usbtogpio_fini(uint8_t interface_index)
{
	return usbtoxxx_fini_command(USB_TO_GPIO, interface_index);
}

RESULT usbtogpio_config(uint8_t interface_index, uint32_t mask,
	uint32_t dir_mask, uint32_t pull_en_mask,
	uint32_t input_pull_mask)
{
	uint8_t conf[8];

#if PARAM_CHECK
	if (interface_index > 7) {
		LOG_BUG(ERRMSG_INVALID_INTERFACE_NUM, interface_index);
		return ERROR_FAIL;
	}
#endif

	dir_mask &= mask;
	SET_LE_U16(&conf[0], mask);
	SET_LE_U16(&conf[2], dir_mask);
	SET_LE_U16(&conf[4], pull_en_mask);
	SET_LE_U16(&conf[6], input_pull_mask);

	return usbtoxxx_conf_command(USB_TO_GPIO, interface_index, conf,
		sizeof(conf));
}

RESULT usbtogpio_in(uint8_t interface_index, uint32_t mask, uint32_t *value)
{
	uint8_t buf[2];

#if PARAM_CHECK
	if (interface_index > 7) {
		LOG_BUG(ERRMSG_INVALID_INTERFACE_NUM, interface_index);
		return ERROR_FAIL;
	}
#endif

	SET_LE_U16(&buf[0], mask);

	return usbtoxxx_in_command(USB_TO_GPIO, interface_index, buf, 2, 2,
		(uint8_t *)value, 0, 2, 0);
}

RESULT usbtogpio_out(uint8_t interface_index, uint32_t mask, uint32_t value)
{
	uint8_t buf[4];

#if PARAM_CHECK
	if (interface_index > 7) {
		LOG_BUG(ERRMSG_INVALID_INTERFACE_NUM, interface_index);
		return ERROR_FAIL;
	}
#endif

	SET_LE_U16(&buf[0], mask);
	SET_LE_U16(&buf[2], value);

	return usbtoxxx_out_command(USB_TO_GPIO, interface_index, buf, 4, 0);
}
