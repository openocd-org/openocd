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
#include "libusb1_common.h"

static struct libusb_context *jtag_libusb_context; /**< Libusb context **/
static libusb_device **devs; /**< The usb device list **/

static bool jtag_libusb_match(struct jtag_libusb_device *dev,
		const uint16_t vids[], const uint16_t pids[])
{
	struct libusb_device_descriptor dev_desc;

	for (unsigned i = 0; vids[i]; i++) {
		if (libusb_get_device_descriptor(dev, &dev_desc) == 0) {
			if (dev_desc.idVendor == vids[i] &&
				dev_desc.idProduct == pids[i])
				return true;
		}
	}
	return false;
}

int jtag_libusb_open(const uint16_t vids[], const uint16_t pids[],
		struct jtag_libusb_device_handle **out)
{
	int cnt, idx, errCode;

	if (libusb_init(&jtag_libusb_context) < 0)
		return -ENODEV;

	cnt = libusb_get_device_list(jtag_libusb_context, &devs);

	for (idx = 0; idx < cnt; idx++) {
		if (!jtag_libusb_match(devs[idx], vids, pids))
			continue;

		errCode = libusb_open(devs[idx], out);

		/** Free the device list **/
		libusb_free_device_list(devs, 1);

		if (errCode) {
			LOG_ERROR("libusb_open() failed with %s",
				  libusb_error_name(errCode));
			return errCode;
		}
		return 0;
	}
	return -ENODEV;
}

void jtag_libusb_close(jtag_libusb_device_handle *dev)
{
	/* Close device */
	libusb_close(dev);

	libusb_exit(jtag_libusb_context);
}

int jtag_libusb_control_transfer(jtag_libusb_device_handle *dev, uint8_t requestType,
		uint8_t request, uint16_t wValue, uint16_t wIndex, char *bytes,
		uint16_t size, unsigned int timeout)
{
	int transferred = 0;

	transferred = libusb_control_transfer(dev, requestType, request, wValue, wIndex,
				(unsigned char *)bytes, size, timeout);

	if (transferred < 0)
		transferred = 0;

	return transferred;
}

int jtag_libusb_bulk_write(jtag_libusb_device_handle *dev, int ep, char *bytes,
		int size, int timeout)
{
	int transferred = 0;

	libusb_bulk_transfer(dev, ep, (unsigned char *)bytes, size,
			     &transferred, timeout);
	return transferred;
}

int jtag_libusb_bulk_read(jtag_libusb_device_handle *dev, int ep, char *bytes,
		int size, int timeout)
{
	int transferred = 0;

	libusb_bulk_transfer(dev, ep, (unsigned char *)bytes, size,
			     &transferred, timeout);
	return transferred;
}

int jtag_libusb_set_configuration(jtag_libusb_device_handle *devh,
		int configuration)
{
	struct jtag_libusb_device *udev = jtag_libusb_get_device(devh);
	int retCode = -99;

	struct libusb_config_descriptor *config = NULL;

	libusb_get_config_descriptor(udev, configuration, &config);
	retCode = libusb_set_configuration(devh, config->bConfigurationValue);

	libusb_free_config_descriptor(config);

	return retCode;
}

int jtag_libusb_get_endpoints(struct jtag_libusb_device *udev,
		unsigned int *usb_read_ep,
		unsigned int *usb_write_ep)
{
	const struct libusb_interface *inter;
	const struct libusb_interface_descriptor *interdesc;
	const struct libusb_endpoint_descriptor *epdesc;
	struct libusb_config_descriptor *config;

	libusb_get_config_descriptor(udev, 0, &config);
	for (int i = 0; i < (int)config->bNumInterfaces; i++) {
		inter = &config->interface[i];

		for (int j = 0; j < inter->num_altsetting; j++) {
			interdesc = &inter->altsetting[j];
			for (int k = 0;
				k < (int)interdesc->bNumEndpoints; k++) {
				epdesc = &interdesc->endpoint[k];

				uint8_t epnum = epdesc->bEndpointAddress;
				bool is_input = epnum & 0x80;
				LOG_DEBUG("usb ep %s %02x",
					is_input ? "in" : "out", epnum);

				if (is_input)
					*usb_read_ep = epnum;
				else
					*usb_write_ep = epnum;
			}
		}
	}
	libusb_free_config_descriptor(config);

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
