// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2009 - 2010 by Simon Qian <SimonQian@SimonQian.com>     *
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

RESULT usbtopwr_init(uint8_t interface_index)
{
	return usbtoxxx_init_command(USB_TO_POWER, interface_index);
}

RESULT usbtopwr_fini(uint8_t interface_index)
{
	return usbtoxxx_fini_command(USB_TO_POWER, interface_index);
}

RESULT usbtopwr_config(uint8_t interface_index)
{
#if PARAM_CHECK
	if (interface_index > 7) {
		LOG_BUG(ERRMSG_INVALID_INTERFACE_NUM, interface_index);
		return ERROR_FAIL;
	}
#endif

	return usbtoxxx_conf_command(USB_TO_POWER, interface_index, NULL, 0);
}

RESULT usbtopwr_output(uint8_t interface_index, uint16_t millivolt)
{
#if PARAM_CHECK
	if (interface_index > 7) {
		LOG_BUG(ERRMSG_INVALID_INTERFACE_NUM, interface_index);
		return ERROR_FAIL;
	}
#endif

	return usbtoxxx_out_command(USB_TO_POWER, interface_index, (uint8_t *)&millivolt,
		2, 0);
}
