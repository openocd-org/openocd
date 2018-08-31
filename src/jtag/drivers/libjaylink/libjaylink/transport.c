/*
 * This file is part of the libjaylink project.
 *
 * Copyright (C) 2014-2015 Marc Schink <jaylink-dev@marcschink.de>
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "libjaylink.h"
#include "libjaylink-internal.h"

/**
 * @file
 *
 * Transport abstraction layer.
 */

/**
 * Open a device.
 *
 * This function must be called before any other function of the transport
 * abstraction layer for the given device handle is called.
 *
 * @param[in,out] devh Device handle.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 */
JAYLINK_PRIV int transport_open(struct jaylink_device_handle *devh)
{
	int ret;

	switch (devh->dev->iface) {
#ifdef HAVE_LIBUSB
	case JAYLINK_HIF_USB:
		ret = transport_usb_open(devh);
		break;
#endif
	case JAYLINK_HIF_TCP:
		ret = transport_tcp_open(devh);
		break;
	default:
		log_err(devh->dev->ctx, "BUG: Invalid host interface: %u.",
			devh->dev->iface);
		return JAYLINK_ERR;
	}

	return ret;
}

/**
 * Close a device.
 *
 * After this function has been called no other function of the transport
 * abstraction layer for the given device handle must be called.
 *
 * @param[in,out] devh Device handle.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR Other error conditions.
 */
JAYLINK_PRIV int transport_close(struct jaylink_device_handle *devh)
{
	int ret;

	switch (devh->dev->iface) {
#ifdef HAVE_LIBUSB
	case JAYLINK_HIF_USB:
		ret = transport_usb_close(devh);
		break;
#endif
	case JAYLINK_HIF_TCP:
		ret = transport_tcp_close(devh);
		break;
	default:
		log_err(devh->dev->ctx, "BUG: Invalid host interface: %u.",
			devh->dev->iface);
		return JAYLINK_ERR;
	}

	return ret;
}

/**
 * Start a write operation for a device.
 *
 * The data of a write operation must be written with at least one call of
 * transport_write(). It is required that all data of a write operation is
 * written before an other write and/or read operation is started.
 *
 * @param[in,out] devh Device handle.
 * @param[in] length Number of bytes of the write operation.
 * @param[in] has_command Determines whether the data of the write operation
 *                        contains the protocol command.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 */
JAYLINK_PRIV int transport_start_write(struct jaylink_device_handle *devh,
		size_t length, bool has_command)
{
	int ret;

	switch (devh->dev->iface) {
#ifdef HAVE_LIBUSB
	case JAYLINK_HIF_USB:
		ret = transport_usb_start_write(devh, length, has_command);
		break;
#endif
	case JAYLINK_HIF_TCP:
		ret = transport_tcp_start_write(devh, length, has_command);
		break;
	default:
		log_err(devh->dev->ctx, "BUG: Invalid host interface: %u.",
			devh->dev->iface);
		return JAYLINK_ERR;
	}

	return ret;
}

/**
 * Start a read operation for a device.
 *
 * The data of a read operation must be read with at least one call of
 * transport_read(). It is required that all data of a read operation is read
 * before an other write and/or read operation is started.
 *
 * @param[in,out] devh Device handle.
 * @param[in] length Number of bytes of the read operation.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 */
JAYLINK_PRIV int transport_start_read(struct jaylink_device_handle *devh,
		size_t length)
{
	int ret;

	switch (devh->dev->iface) {
#ifdef HAVE_LIBUSB
	case JAYLINK_HIF_USB:
		ret = transport_usb_start_read(devh, length);
		break;
#endif
	case JAYLINK_HIF_TCP:
		ret = transport_tcp_start_read(devh, length);
		break;
	default:
		log_err(devh->dev->ctx, "BUG: Invalid host interface: %u.",
			devh->dev->iface);
		return JAYLINK_ERR;
	}

	return ret;
}

/**
 * Start a write and read operation for a device.
 *
 * This function starts a write and read operation as the consecutive call of
 * transport_start_write() and transport_start_read() but has a different
 * meaning from the protocol perspective and can therefore not be replaced by
 * these functions and vice versa.
 *
 * @note The write operation must be completed first before the read operation
 *       must be processed.
 *
 * @param[in,out] devh Device handle.
 * @param[in] write_length Number of bytes of the write operation.
 * @param[in] read_length Number of bytes of the read operation.
 * @param[in] has_command Determines whether the data of the write operation
 *                        contains the protocol command.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 */
JAYLINK_PRIV int transport_start_write_read(struct jaylink_device_handle *devh,
		size_t write_length, size_t read_length, bool has_command)
{
	int ret;

	switch (devh->dev->iface) {
#ifdef HAVE_LIBUSB
	case JAYLINK_HIF_USB:
		ret = transport_usb_start_write_read(devh, write_length,
			read_length, has_command);
		break;
#endif
	case JAYLINK_HIF_TCP:
		ret = transport_tcp_start_write_read(devh, write_length,
			read_length, has_command);
		break;
	default:
		log_err(devh->dev->ctx, "BUG: Invalid host interface: %u.",
			devh->dev->iface);
		return JAYLINK_ERR;
	}

	return ret;
}

/**
 * Write data to a device.
 *
 * Before this function is used transport_start_write() or
 * transport_start_write_read() must be called to start a write operation. The
 * total number of written bytes must not exceed the number of bytes of the
 * write operation.
 *
 * @note A write operation will be performed and the data will be sent to the
 *       device when the number of written bytes reaches the number of bytes of
 *       the write operation. Before that the data will be written into a
 *       buffer.
 *
 * @param[in,out] devh Device handle.
 * @param[in] buffer Buffer to write data from.
 * @param[in] length Number of bytes to write.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 */
JAYLINK_PRIV int transport_write(struct jaylink_device_handle *devh,
		const uint8_t *buffer, size_t length)
{
	int ret;

	switch (devh->dev->iface) {
#ifdef HAVE_LIBUSB
	case JAYLINK_HIF_USB:
		ret = transport_usb_write(devh, buffer, length);
		break;
#endif
	case JAYLINK_HIF_TCP:
		ret = transport_tcp_write(devh, buffer, length);
		break;
	default:
		log_err(devh->dev->ctx, "BUG: Invalid host interface: %u.",
			devh->dev->iface);
		return JAYLINK_ERR;
	}

	return ret;
}

/**
 * Read data from a device.
 *
 * Before this function is used transport_start_read() or
 * transport_start_write_read() must be called to start a read operation. The
 * total number of read bytes must not exceed the number of bytes of the read
 * operation.
 *
 * @param[in,out] devh Device handle.
 * @param[out] buffer Buffer to read data into on success. Its content is
 *                    undefined on failure.
 * @param[in] length Number of bytes to read.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 */
JAYLINK_PRIV int transport_read(struct jaylink_device_handle *devh,
		uint8_t *buffer, size_t length)
{
	int ret;

	switch (devh->dev->iface) {
#ifdef HAVE_LIBUSB
	case JAYLINK_HIF_USB:
		ret = transport_usb_read(devh, buffer, length);
		break;
#endif
	case JAYLINK_HIF_TCP:
		ret = transport_tcp_read(devh, buffer, length);
		break;
	default:
		log_err(devh->dev->ctx, "BUG: Invalid host interface: %u.",
			devh->dev->iface);
		return JAYLINK_ERR;
	}

	return ret;
}
