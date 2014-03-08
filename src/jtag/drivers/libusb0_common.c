/***************************************************************************
 *   Copyright (C) 2009 by Zachary T Welch <zw@superlucidity.net>          *
 *                                                                         *
 *   Copyright (C) 2011 by Mauro Gamba <maurillo71@gmail.com>              *
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
#include "log.h"
#include "libusb0_common.h"

static bool jtag_libusb_match(struct jtag_libusb_device *dev,
		const uint16_t vids[], const uint16_t pids[])
{
	for (unsigned i = 0; vids[i]; i++) {
		if (dev->descriptor.idVendor == vids[i] &&
			dev->descriptor.idProduct == pids[i]) {
			return true;
		}
	}
	return false;
}

int jtag_libusb_open(const uint16_t vids[], const uint16_t pids[],
		struct jtag_libusb_device_handle **out)
{
	usb_init();

	usb_find_busses();
	usb_find_devices();

	struct usb_bus *busses = usb_get_busses();
	for (struct usb_bus *bus = busses; bus; bus = bus->next) {
		for (struct usb_device *dev = bus->devices;
				dev; dev = dev->next) {
			if (!jtag_libusb_match(dev, vids, pids))
				continue;

			*out = usb_open(dev);
			if (NULL == *out)
				return -errno;
			return 0;
		}
	}
	return -ENODEV;
}

void jtag_libusb_close(jtag_libusb_device_handle *dev)
{
	/* Close device */
	usb_close(dev);
}

int jtag_libusb_control_transfer(jtag_libusb_device_handle *dev, uint8_t requestType,
		uint8_t request, uint16_t wValue, uint16_t wIndex, char *bytes,
		uint16_t size, unsigned int timeout)
{
	int transferred = 0;

	transferred = usb_control_msg(dev, requestType, request, wValue, wIndex,
				bytes, size, timeout);

	if (transferred < 0)
		transferred = 0;

	return transferred;
}

int jtag_libusb_bulk_write(jtag_libusb_device_handle *dev, int ep, char *bytes,
		int size, int timeout)
{
	return usb_bulk_write(dev, ep, bytes, size, timeout);
}

int jtag_libusb_bulk_read(jtag_libusb_device_handle *dev, int ep, char *bytes,
		int size, int timeout)
{
	return usb_bulk_read(dev, ep, bytes, size, timeout);
}

int jtag_libusb_set_configuration(jtag_libusb_device_handle *devh,
		int configuration)
{
	struct jtag_libusb_device *udev = jtag_libusb_get_device(devh);

	return usb_set_configuration(devh,
			udev->config[configuration].bConfigurationValue);
}

int jtag_libusb_get_endpoints(struct jtag_libusb_device *udev,
		unsigned int *usb_read_ep,
		unsigned int *usb_write_ep)
{
	struct usb_interface *iface = udev->config->interface;
	struct usb_interface_descriptor *desc = iface->altsetting;

	for (int i = 0; i < desc->bNumEndpoints; i++) {
		uint8_t epnum = desc->endpoint[i].bEndpointAddress;
		bool is_input = epnum & 0x80;
		LOG_DEBUG("usb ep %s %02x", is_input ? "in" : "out", epnum);
		if (is_input)
			*usb_read_ep = epnum;
		else
			*usb_write_ep = epnum;
	}

	return 0;
}

int jtag_libusb_get_pid(struct jtag_libusb_device *dev, uint16_t *pid)
{
	struct libusb_device_descriptor dev_desc;

	if (libusb_get_device_descriptor(dev, &dev_desc) == 0) {
		*pid = dev_desc.idProduct;

		return 0;
	}

	return -ENODEV;
}
