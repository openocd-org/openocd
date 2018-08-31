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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <stdbool.h>
#ifdef _WIN32
#include <winsock2.h>
#endif
#ifdef HAVE_LIBUSB
#include <libusb.h>
#endif

#include "libjaylink.h"
#include "libjaylink-internal.h"

/**
 * @mainpage
 *
 * @section sec_intro Introduction
 *
 * This document describes the API of libjaylink.
 *
 * libjaylink is a shared library written in C to access SEGGER J-Link and
 * compatible devices.
 *
 * @section sec_error Error handling
 *
 * The libjaylink functions which can fail use the return value to indicate an
 * error. The functions typically return an error code of #jaylink_error.
 * For each function, all possible error codes and their detailed descriptions
 * are documented. As the possible error codes returned by a function may
 * change it is recommended to also always cover unexpected values when
 * checking for error codes to be compatible with later versions of libjaylink.
 *
 * There are a few exceptions where a function directly returns the result
 * instead of an error code because it is more convenient from an API
 * perspective and because there is only a single reason for failure which is
 * clearly distinguishable from the result.
 *
 * @section sec_license Copyright and license
 *
 * libjaylink is licensed under the terms of the GNU General Public
 * License (GPL), version 2 or later.
 *
 * @section sec_website Website
 *
 * <a href="http://git.zapb.de/libjaylink.git">git.zapb.de/libjaylink.git</a>
 */

/**
 * @file
 *
 * Core library functions.
 */

/**
 * Initialize libjaylink.
 *
 * This function must be called before any other libjaylink function is called.
 *
 * @param[out] ctx Newly allocated libjaylink context on success, and undefined
 *                 on failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_MALLOC Memory allocation error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_init(struct jaylink_context **ctx)
{
	int ret;
	struct jaylink_context *context;
#ifdef _WIN32
	WSADATA wsa_data;
#endif

	if (!ctx)
		return JAYLINK_ERR_ARG;

	context = malloc(sizeof(struct jaylink_context));

	if (!context)
		return JAYLINK_ERR_MALLOC;

#ifdef HAVE_LIBUSB
	if (libusb_init(&context->usb_ctx) != LIBUSB_SUCCESS) {
		free(context);
		return JAYLINK_ERR;
	}
#endif

#ifdef _WIN32
	ret = WSAStartup(MAKEWORD(2, 2), &wsa_data);

	if (ret != 0) {
#ifdef HAVE_LIBUSB
		libusb_exit(context->usb_ctx);
#endif
		free(context);
		return JAYLINK_ERR;
	}

	if (LOBYTE(wsa_data.wVersion) != 2 || HIBYTE(wsa_data.wVersion) != 2) {
#ifdef HAVE_LIBUSB
		libusb_exit(context->usb_ctx);
#endif
		free(context);
		return JAYLINK_ERR;
	}
#endif

	context->devs = NULL;
	context->discovered_devs = NULL;

	/* Show error and warning messages by default. */
	context->log_level = JAYLINK_LOG_LEVEL_WARNING;

	context->log_callback = &log_vprintf;
	context->log_callback_data = NULL;

	ret = jaylink_log_set_domain(context, JAYLINK_LOG_DOMAIN_DEFAULT);

	if (ret != JAYLINK_OK) {
#ifdef HAVE_LIBUSB
		libusb_exit(context->usb_ctx);
#endif
#ifdef _WIN32
		WSACleanup();
#endif
		free(context);
		return ret;
	}

	*ctx = context;

	return JAYLINK_OK;
}

/**
 * Shutdown libjaylink.
 *
 * @param[in,out] ctx libjaylink context.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_exit(struct jaylink_context *ctx)
{
	struct list *item;

	if (!ctx)
		return JAYLINK_ERR_ARG;

	item = ctx->discovered_devs;

	while (item) {
		jaylink_unref_device((struct jaylink_device *)item->data);
		item = item->next;
	}

	list_free(ctx->discovered_devs);
	list_free(ctx->devs);

#ifdef HAVE_LIBUSB
	libusb_exit(ctx->usb_ctx);
#endif
#ifdef _WIN32
	WSACleanup();
#endif
	free(ctx);

	return JAYLINK_OK;
}

/**
 * Check for a capability of libjaylink.
 *
 * @param[in] cap Capability to check for.
 *
 * @retval true Capability is supported.
 * @retval false Capability is not supported or invalid argument.
 *
 * @since 0.1.0
 */
JAYLINK_API bool jaylink_library_has_cap(enum jaylink_capability cap)
{
	switch (cap) {
#ifdef HAVE_LIBUSB
	case JAYLINK_CAP_HIF_USB:
		return true;
#endif
	default:
		return false;
	}
}
