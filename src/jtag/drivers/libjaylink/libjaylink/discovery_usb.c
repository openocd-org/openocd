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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <sys/types.h>

#include "libjaylink.h"
#include "libjaylink-internal.h"

/*
 * libusb.h includes windows.h and therefore must be included after anything
 * that includes winsock2.h.
 */
#include <libusb.h>

/**
 * @file
 *
 * Device discovery (USB).
 */

/** @cond PRIVATE */
/** USB Vendor ID (VID) of SEGGER products. */
#define USB_VENDOR_ID			0x1366

/* USB Product IDs (PID) and their corresponding USB addresses. */
static const uint16_t pids[][2] = {
	{0x0101, 0},
	{0x0102, 1},
	{0x0103, 2},
	{0x0104, 3},
	{0x0105, 0},
	{0x0107, 0},
	{0x0108, 0},
	{0x1010, 0},
	{0x1011, 0},
	{0x1012, 0},
	{0x1013, 0},
	{0x1014, 0},
	{0x1015, 0},
	{0x1016, 0},
	{0x1017, 0},
	{0x1018, 0},
	{0x1051, 0},
	{0x1061, 0}
};

/** Maximum length of the USB string descriptor for the serial number. */
#define USB_SERIAL_NUMBER_LENGTH	12

/**
 * Maximum number of digits in a serial number
 *
 * The serial number of a device consists of at most 9 digits but user defined
 * serial numbers are allowed with up to 10 digits.
 */
#define MAX_SERIAL_NUMBER_DIGITS	10
/** @endcond */

static bool parse_serial_number(const char *str, uint32_t *serial_number)
{
	size_t length;

	length = strlen(str);

	/*
	 * Skip the first digits which are not part of a valid serial number.
	 * This is necessary because some devices erroneously use random digits
	 * instead of zeros for padding.
	 */
	if (length > MAX_SERIAL_NUMBER_DIGITS)
		str = str + (length - MAX_SERIAL_NUMBER_DIGITS);

	if (jaylink_parse_serial_number(str, serial_number) != JAYLINK_OK)
		return false;

	return true;
}

static bool compare_devices(const void *a, const void *b)
{
	const struct jaylink_device *dev;
	const struct libusb_device *usb_dev;

	dev = a;
	usb_dev = b;

	if (dev->iface != JAYLINK_HIF_USB)
		return false;

	if (dev->usb_dev == usb_dev)
		return true;

	return false;
}

static struct jaylink_device *find_device(const struct jaylink_context *ctx,
		const struct libusb_device *usb_dev)
{
	struct list *item;

	item = list_find_custom(ctx->devs, &compare_devices, usb_dev);

	if (item)
		return item->data;

	return NULL;
}

static struct jaylink_device *probe_device(struct jaylink_context *ctx,
		struct libusb_device *usb_dev)
{
	int ret;
	struct libusb_device_descriptor desc;
	struct libusb_device_handle *usb_devh;
	struct jaylink_device *dev;
	char buf[USB_SERIAL_NUMBER_LENGTH + 1];
	uint8_t usb_address;
	uint32_t serial_number;
	bool valid_serial_number;
	bool found_device;
	size_t i;

	ret = libusb_get_device_descriptor(usb_dev, &desc);

	if (ret != LIBUSB_SUCCESS) {
		log_warn(ctx, "Failed to get device descriptor: %s.",
			libusb_error_name(ret));
		return NULL;
	}

	if (desc.idVendor != USB_VENDOR_ID)
		return NULL;

	found_device = false;

	for (i = 0; i < sizeof(pids) / sizeof(pids[0]); i++) {
		if (pids[i][0] == desc.idProduct) {
			found_device = true;
			usb_address = pids[i][1];
			break;
		}
	}

	if (!found_device)
		return NULL;

	log_dbg(ctx, "Found device (VID:PID = %04x:%04x, bus:address = "
		"%03u:%03u).", desc.idVendor, desc.idProduct,
		libusb_get_bus_number(usb_dev),
		libusb_get_device_address(usb_dev));

	/*
	 * Search for an already allocated device instance for this device and
	 * if found return a reference to it.
	 */
	dev = find_device(ctx, usb_dev);

	if (dev) {
		log_dbg(ctx, "Device: USB address = %u.", dev->usb_address);

		if (dev->valid_serial_number)
			log_dbg(ctx, "Device: Serial number = %u.",
				dev->serial_number);
		else
			log_dbg(ctx, "Device: Serial number = N/A.");

		log_dbg(ctx, "Using existing device instance.");
		return jaylink_ref_device(dev);
	}

	/* Open the device to be able to retrieve its serial number. */
	ret = libusb_open(usb_dev, &usb_devh);

	if (ret != LIBUSB_SUCCESS) {
		log_warn(ctx, "Failed to open device: %s.",
			libusb_error_name(ret));
		return NULL;
	}

	serial_number = 0;
	valid_serial_number = true;

	ret = libusb_get_string_descriptor_ascii(usb_devh, desc.iSerialNumber,
		(unsigned char *)buf, USB_SERIAL_NUMBER_LENGTH + 1);

	libusb_close(usb_devh);

	if (ret < 0) {
		log_warn(ctx, "Failed to retrieve serial number: %s.",
			libusb_error_name(ret));
		valid_serial_number = false;
	}

	if (valid_serial_number) {
		if (!parse_serial_number(buf, &serial_number)) {
			log_warn(ctx, "Failed to parse serial number.");
			return NULL;
		}
	}

	log_dbg(ctx, "Device: USB address = %u.", usb_address);

	if (valid_serial_number)
		log_dbg(ctx, "Device: Serial number = %u.", serial_number);
	else
		log_dbg(ctx, "Device: Serial number = N/A.");

	log_dbg(ctx, "Allocating new device instance.");

	dev = device_allocate(ctx);

	if (!dev) {
		log_warn(ctx, "Device instance malloc failed.");
		return NULL;
	}

	dev->iface = JAYLINK_HIF_USB;
	dev->usb_dev = libusb_ref_device(usb_dev);
	dev->usb_address = usb_address;
	dev->serial_number = serial_number;
	dev->valid_serial_number = valid_serial_number;

	return dev;
}

JAYLINK_PRIV int discovery_usb_scan(struct jaylink_context *ctx)
{
	ssize_t ret;
	struct libusb_device **devs;
	struct jaylink_device *dev;
	size_t num;
	size_t i;

	ret = libusb_get_device_list(ctx->usb_ctx, &devs);

	if (ret == LIBUSB_ERROR_IO) {
		log_err(ctx, "Failed to retrieve device list: input/output "
			"error.");
		return JAYLINK_ERR_IO;
	} else if (ret < 0) {
		log_err(ctx, "Failed to retrieve device list: %s.",
			libusb_error_name(ret));
		return JAYLINK_ERR;
	}

	num = 0;

	for (i = 0; devs[i]; i++) {
		dev = probe_device(ctx, devs[i]);

		if (!dev)
			continue;

		ctx->discovered_devs = list_prepend(ctx->discovered_devs, dev);
		num++;
	}

	libusb_free_device_list(devs, true);
	log_dbg(ctx, "Found %zu USB device(s).", num);

	return JAYLINK_OK;
}
