// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2013 by Andes Technology                                *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include "aice_usb.h"
#include "aice_pipe.h"
#include "aice_port.h"

static const struct aice_port aice_ports[] = {
	{
		.name = "aice_usb",
		.type = AICE_PORT_AICE_USB,
		.api = &aice_usb_api,
	},
	{
		.name = "aice_pipe",
		.type = AICE_PORT_AICE_PIPE,
		.api = &aice_pipe,
	},
	{.name = NULL, /* END OF TABLE */ },
};

/** */
const struct aice_port *aice_port_get_list(void)
{
	return aice_ports;
}
