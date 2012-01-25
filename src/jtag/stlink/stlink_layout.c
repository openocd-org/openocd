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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <jtag/interface.h>
#include <transport/transport.h>
#include <helper/time_support.h>

#include <jtag/stlink/stlink_layout.h>
#include <jtag/stlink/stlink_tcl.h>
#include <jtag/stlink/stlink_transport.h>
#include <jtag/stlink/stlink_interface.h>

#define STLINK_LAYOUT_UNKNOWN	0
#define STLINK_LAYOUT_SG	1
#define STLINK_LAYOUT_USB	2

static int stlink_layout_open(struct stlink_interface_s *stlink_if)
{
	int res;

	LOG_DEBUG("stlink_layout_open");

	stlink_if->fd = NULL;

	res = stlink_if->layout->api->open(&stlink_if->param, &stlink_if->fd);

	if (res != ERROR_OK) {
		LOG_DEBUG("failed");
		return res;
	}

	return ERROR_OK;
}

static int stlink_layout_close(struct stlink_interface_s *stlink_if)
{
	return ERROR_OK;
}

static const struct stlink_layout stlink_layouts[] = {
	{
	 .name = "usb",
	 .type = STLINK_LAYOUT_USB,
	 .open = stlink_layout_open,
	 .close = stlink_layout_close,
	 .api = &stlink_usb_layout_api,
	 },
	{
	 .name = "sg",
	 .type = STLINK_LAYOUT_SG,
	 .open = stlink_layout_open,
	 .close = stlink_layout_close,
	 .api = &stlink_usb_layout_api,
	 },
	{.name = NULL, /* END OF TABLE */ },
};

/** */
const struct stlink_layout *stlink_layout_get_list(void)
{
	return stlink_layouts;
}

int stlink_layout_init(struct stlink_interface_s *stlink_if)
{
	LOG_DEBUG("stlink_layout_init");

	if (stlink_if->layout == NULL) {
		LOG_ERROR("no layout specified");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}
