// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <jtag/interface.h>
#include <transport/transport.h>
#include <helper/time_support.h>

#include <jtag/hla/hla_layout.h>
#include <jtag/hla/hla_tcl.h>
#include <jtag/hla/hla_transport.h>
#include <jtag/hla/hla_interface.h>

static int hl_layout_open(struct hl_interface_s *adapter)
{
	int res;

	LOG_DEBUG("hl_layout_open");

	adapter->handle = NULL;

	res = adapter->layout->api->open(&adapter->param, &adapter->handle);

	if (res != ERROR_OK) {
		LOG_DEBUG("failed");
		return res;
	}

	return ERROR_OK;
}

static int hl_layout_close(struct hl_interface_s *adapter)
{
	return ERROR_OK;
}

static const struct hl_layout hl_layouts[] = {
#if BUILD_HLADAPTER_STLINK
	{
	 .name = "stlink",
	 .open = hl_layout_open,
	 .close = hl_layout_close,
	 .api = &stlink_usb_layout_api,
	 },
#endif
#if BUILD_HLADAPTER_ICDI
	{
	 .name = "ti-icdi",
	 .open = hl_layout_open,
	 .close = hl_layout_close,
	 .api = &icdi_usb_layout_api,
	},
#endif
#if BUILD_HLADAPTER_NULINK
	{
	 .name = "nulink",
	 .open = hl_layout_open,
	 .close = hl_layout_close,
	 .api = &nulink_usb_layout_api,
	},
#endif
	{.name = NULL, /* END OF TABLE */ },
};

/** */
const struct hl_layout *hl_layout_get_list(void)
{
	return hl_layouts;
}

int hl_layout_init(struct hl_interface_s *adapter)
{
	LOG_DEBUG("hl_layout_init");

	if (!adapter->layout) {
		LOG_ERROR("no layout specified");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}
