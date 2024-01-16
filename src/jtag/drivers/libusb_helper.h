/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2009 by Zachary T Welch <zw@superlucidity.net>          *
 *                                                                         *
 *   Copyright (C) 2011 by Mauro Gamba <maurillo71@gmail.com>              *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_DRIVERS_LIBUSB_HELPER_H
#define OPENOCD_JTAG_DRIVERS_LIBUSB_HELPER_H

#include <libusb.h>

/* When we debug a target that works as a USB device, halting the target causes the
 * USB communication with the USB host to become unresponsive. The host will try
 * to reconnect/reset/setup the unresponsive device during which communication
 * with other devices on the same USB bus can get stalled for several seconds.
 * If the JTAG adapter is on the same bus, we need to make sure openOCD will wait
 * for packets at least as long as the host USB stack. Otherwise the USB stack
 * might deliver a valid packet, but openOCD would ignore it due to the timeout.
 * The xHCI spec uses 5 sec timeouts, so let's use that in openOCD with some margin.
 *
 * Use this value in all libusb calls. HID API might have a libusb backend and
 * would probably be victim to the same bug, so it should use this timeout, too.
 */
#define LIBUSB_TIMEOUT_MS	(6000)

/* this callback should return a non NULL value only when the serial could not
 * be retrieved by the standard 'libusb_get_string_descriptor_ascii' */
typedef char * (*adapter_get_alternate_serial_fn)(struct libusb_device_handle *device,
		struct libusb_device_descriptor *dev_desc);

bool jtag_libusb_match_ids(struct libusb_device_descriptor *dev_desc,
		const uint16_t vids[], const uint16_t pids[]);
int jtag_libusb_open(const uint16_t vids[], const uint16_t pids[],
		const char *product, struct libusb_device_handle **out,
		adapter_get_alternate_serial_fn adapter_get_alternate_serial);
void jtag_libusb_close(struct libusb_device_handle *dev);
int jtag_libusb_control_transfer(struct libusb_device_handle *dev,
		uint8_t request_type, uint8_t request, uint16_t value,
		uint16_t index, char *bytes, uint16_t size, unsigned int timeout,
		int *transferred);
int jtag_libusb_bulk_write(struct libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout, int *transferred);
int jtag_libusb_bulk_read(struct libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout, int *transferred);
int jtag_libusb_set_configuration(struct libusb_device_handle *devh,
		int configuration);
/**
 * Find the first interface optionally matching class, subclass and
 * protocol and claim it.
 * @param devh _libusb_ device handle.
 * @param usb_read_ep A pointer to a variable where the _IN_ endpoint
 *	number will be stored.
 * @param usb_write_ep A pointer to a variable where the _OUT_ endpoint
 *	number will be stored.
 * @param bclass `bInterfaceClass` to match, or -1 to ignore this field.
 * @param subclass `bInterfaceSubClass` to match, or -1 to ignore this field.
 * @param protocol `bInterfaceProtocol` to match, or -1 to ignore this field.
 * @param trans_type `bmAttributes Bits 0..1 Transfer type` to match, or -1 to ignore this field.
 * @returns Returns ERROR_OK on success, ERROR_FAIL otherwise.
 */
int jtag_libusb_choose_interface(struct libusb_device_handle *devh,
		unsigned int *usb_read_ep,
		unsigned int *usb_write_ep,
		int bclass, int subclass, int protocol, int trans_type);
int jtag_libusb_get_pid(struct libusb_device *dev, uint16_t *pid);
int jtag_libusb_handle_events_completed(int *completed);

/**
 * Attempts to allocate a block of persistent DMA memory suitable for transfers
 * against the USB device. Fall-back to the ordinary heap malloc()
 * if the first libusb_dev_mem_alloc() call fails.
 * @param devh _libusb_ device handle.
 * @param length size of desired data buffer
 * @returns a pointer to the newly allocated memory, or NULL on failure
 */
uint8_t *oocd_libusb_dev_mem_alloc(libusb_device_handle *devh,
			size_t length);

/**
 * Free device memory allocated with oocd_libusb_dev_mem_alloc().
 * Uses either libusb_dev_mem_free() or free() consistently with
 * the used method of allocation.
 * @param devh _libusb_ device handle.
 * @param buffer pointer to the previously allocated memory
 * @param length size of desired data buffer
 * @returns Returns ERROR_OK on success, ERROR_FAIL otherwise.
 */
int oocd_libusb_dev_mem_free(libusb_device_handle *devh,
			uint8_t *buffer, size_t length);

#endif /* OPENOCD_JTAG_DRIVERS_LIBUSB_HELPER_H */
