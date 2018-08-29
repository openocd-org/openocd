/*
 * This file is part of the libjaylink project.
 *
 * Copyright (C) 2014-2016 Marc Schink <jaylink-dev@marcschink.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdlib.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "libjaylink.h"
#include "libjaylink-internal.h"

/**
 * @file
 *
 * Device discovery.
 */

static void clear_discovery_list(struct jaylink_context *ctx)
{
	struct list *item;
	struct list *tmp;
	struct jaylink_device *dev;

	item = ctx->discovered_devs;

	while (item) {
		dev = (struct jaylink_device *)item->data;
		jaylink_unref_device(dev);

		tmp = item;
		item = item->next;
		free(tmp);
	}

	ctx->discovered_devs = NULL;
}

/**
 * Scan for devices.
 *
 * @param[in,out] ctx libjaylink context.
 * @param[in] ifaces Host interfaces to scan for devices. Use bitwise OR to
 *                   specify multiple interfaces, or 0 to use all available
 *                   interfaces. See #jaylink_host_interface for a description
 *                   of the interfaces.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @see jaylink_get_devices()
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_discovery_scan(struct jaylink_context *ctx,
		uint32_t ifaces)
{
	int ret;

	if (!ctx)
		return JAYLINK_ERR_ARG;

	if (!ifaces)
		ifaces = JAYLINK_HIF_USB | JAYLINK_HIF_TCP;

	clear_discovery_list(ctx);

#ifdef HAVE_LIBUSB
	if (ifaces & JAYLINK_HIF_USB) {
		ret = discovery_usb_scan(ctx);

		if (ret != JAYLINK_OK) {
			log_err(ctx, "USB device discovery failed.");
			return ret;
		}
	}
#endif

	if (ifaces & JAYLINK_HIF_TCP) {
		ret = discovery_tcp_scan(ctx);

		if (ret != JAYLINK_OK) {
			log_err(ctx, "TCP/IP device discovery failed.");
			return ret;
		}
	}

	return JAYLINK_OK;
}
