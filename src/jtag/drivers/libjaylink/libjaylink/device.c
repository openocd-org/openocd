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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#ifdef _WIN32
#include <winsock2.h>
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#endif
#ifdef HAVE_LIBUSB
#include <libusb.h>
#endif

#include "libjaylink.h"
#include "libjaylink-internal.h"

/**
 * @file
 *
 * Device enumeration and handling.
 */

/** @cond PRIVATE */
#define CMD_GET_VERSION		0x01
#define CMD_GET_HW_STATUS	0x07
#define CMD_REGISTER		0x09
#define CMD_GET_HW_INFO		0xc1
#define CMD_GET_COUNTERS	0xc2
#define CMD_GET_FREE_MEMORY	0xd4
#define CMD_GET_CAPS		0xe8
#define CMD_GET_EXT_CAPS	0xed
#define CMD_GET_HW_VERSION	0xf0
#define CMD_READ_CONFIG		0xf2
#define CMD_WRITE_CONFIG	0xf3

#define REG_CMD_REGISTER	0x64
#define REG_CMD_UNREGISTER	0x65

/** Size of the registration header in bytes. */
#define REG_HEADER_SIZE		8
/** Minimum registration information size in bytes. */
#define REG_MIN_SIZE		0x4c
/** Maximum registration information size in bytes. */
#define REG_MAX_SIZE		0x200
/** Size of a connection entry in bytes. */
#define REG_CONN_INFO_SIZE	16

/* The maximum path depth according to the USB 3.0 specification. */
#define MAX_USB_PATH_DEPTH	7
/** @endcond */

/** @private */
JAYLINK_PRIV struct jaylink_device *device_allocate(
		struct jaylink_context *ctx)
{
	struct jaylink_device *dev;
	struct list *list;

	dev = malloc(sizeof(struct jaylink_device));

	if (!dev)
		return NULL;

	list = list_prepend(ctx->devs, dev);

	if (!list) {
		free(dev);
		return NULL;
	}

	ctx->devs = list;

	dev->ctx = ctx;
	dev->ref_count = 1;

	return dev;
}

static struct jaylink_device **allocate_device_list(size_t length)
{
	struct jaylink_device **list;

	list = malloc(sizeof(struct jaylink_device *) * (length + 1));

	if (!list)
		return NULL;

	list[length] = NULL;

	return list;
}

/**
 * Get available devices.
 *
 * @param[in,out] ctx libjaylink context.
 * @param[out] devs Newly allocated array which contains instances of available
 *                  devices on success, and undefined on failure. The array is
 *                  NULL-terminated and must be free'd by the caller with
 *                  jaylink_free_devices().
 * @param[out] count Number of available devices on success, and undefined on
 *                   failure. Can be NULL.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_MALLOC Memory allocation error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @see jaylink_discovery_scan()
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_get_devices(struct jaylink_context *ctx,
		struct jaylink_device ***devs, size_t *count)
{
	size_t num;
	struct list *item;
	struct jaylink_device **tmp;
	struct jaylink_device *dev;
	size_t i;

	if (!ctx || !devs)
		return JAYLINK_ERR_ARG;

	num = list_length(ctx->discovered_devs);
	tmp = allocate_device_list(num);

	if (!tmp) {
		log_err(ctx, "Failed to allocate device list.");
		return JAYLINK_ERR_MALLOC;
	}

	item = ctx->discovered_devs;

	for (i = 0; i < num; i++) {
		dev = (struct jaylink_device *)item->data;
		tmp[i] = jaylink_ref_device(dev);
		item = item->next;
	}

	if (count)
		*count = num;

	*devs = tmp;

	return JAYLINK_OK;
}

/**
 * Free devices.
 *
 * @param[in,out] devs Array of device instances. Must be NULL-terminated.
 * @param[in] unref Determines whether the device instances should be
 *                  unreferenced.
 *
 * @see jaylink_get_devices()
 *
 * @since 0.1.0
 */
JAYLINK_API void jaylink_free_devices(struct jaylink_device **devs, bool unref)
{
	size_t i;

	if (!devs)
		return;

	if (unref) {
		for (i = 0; devs[i]; i++)
			jaylink_unref_device(devs[i]);
	}

	free(devs);
}

/**
 * Get the host interface of a device.
 *
 * @param[in] dev Device instance.
 * @param[out] iface Host interface of the device on success, and undefined on
 *                   failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_device_get_host_interface(
		const struct jaylink_device *dev,
		enum jaylink_host_interface *iface)
{
	if (!dev || !iface)
		return JAYLINK_ERR_ARG;

	*iface = dev->iface;

	return JAYLINK_OK;
}

/**
 * Get the serial number of a device.
 *
 * @note This serial number is for enumeration purpose only and might differ
 *       from the real serial number of the device.
 *
 * @param[in] dev Device instance.
 * @param[out] serial_number Serial number of the device on success, and
 *                           undefined on failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_NOT_AVAILABLE Serial number is not available.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_device_get_serial_number(
		const struct jaylink_device *dev, uint32_t *serial_number)
{
	if (!dev || !serial_number)
		return JAYLINK_ERR_ARG;

	if (!dev->valid_serial_number)
		return JAYLINK_ERR_NOT_AVAILABLE;

	*serial_number = dev->serial_number;

	return JAYLINK_OK;
}

/**
 * Get the USB address of a device.
 *
 * @note Identification of a device with the USB address is deprecated and the
 *       serial number should be used instead.
 *
 * @param[in] dev Device instance.
 * @param[out] address USB address of the device on success, and undefined on
 *                     failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_NOT_SUPPORTED Supported for devices with host interface
 *                                   #JAYLINK_HIF_USB only.
 *
 * @see jaylink_device_get_serial_number()
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_device_get_usb_address(
		const struct jaylink_device *dev,
		enum jaylink_usb_address *address)
{
	if (!dev || !address)
		return JAYLINK_ERR_ARG;

	if (dev->iface != JAYLINK_HIF_USB)
		return JAYLINK_ERR_NOT_SUPPORTED;

#ifdef HAVE_LIBUSB
	*address = dev->usb_address;

	return JAYLINK_OK;
#else
	return JAYLINK_ERR_NOT_SUPPORTED;
#endif
}

/**
 * Get the USB bus and port numbers of a device.
 *
 * @param[in] dev Device instance.
 * @param[out] bus The bus number of the device on success and undefined on
 *                 failure.
 * @param[out] ports Newly allocated array which contains the port numbers on
 *                   success and is undefined on failure. The array must be
 *                   free'd by the caller.
 * @param[out] length Length of the port array on success and undefined on
 *                    failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_MALLOC Memory allocation error.
 * @retval JAYLINK_ERR_NOT_SUPPORTED Supported for devices with host interface
 *                                   #JAYLINK_HIF_USB only.
 *
 * @since 0.2.0
 */
JAYLINK_API int jaylink_device_get_usb_bus_ports(
		const struct jaylink_device *dev, uint8_t *bus,
		uint8_t **ports, size_t *length)
{
	if (!dev || !bus || !ports || !length)
		return JAYLINK_ERR_ARG;

	if (dev->iface != JAYLINK_HIF_USB)
		return JAYLINK_ERR_NOT_SUPPORTED;

#ifdef HAVE_LIBUSB
	struct jaylink_context *ctx = dev->ctx;
	int ret;

	*ports = malloc(MAX_USB_PATH_DEPTH * sizeof(uint8_t));

	if (!*ports) {
		return JAYLINK_ERR_MALLOC;
	}

	ret = libusb_get_port_numbers(dev->usb_dev, *ports,
		MAX_USB_PATH_DEPTH);

	if (ret == LIBUSB_ERROR_OVERFLOW) {
		log_err(ctx, "Failed to get port numbers: %s.",
			libusb_error_name(ret));
		return JAYLINK_ERR_ARG;
	}

	*length = ret;
	*bus = libusb_get_bus_number(dev->usb_dev);

	return JAYLINK_OK;
#else
	return JAYLINK_ERR_NOT_SUPPORTED;
#endif
}

/**
 * Get the IPv4 address string of a device.
 *
 * @param[in] dev Device instance.
 * @param[out] address IPv4 address string in quad-dotted decimal format of the
 *                     device on success and undefined on failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_NOT_SUPPORTED Supported for devices with host interface
 *                                   #JAYLINK_HIF_TCP only.
 *
 * @since 0.2.0
 */
JAYLINK_API int jaylink_device_get_ipv4_address(
		const struct jaylink_device *dev, char *address)
{
	if (!dev || !address)
		return JAYLINK_ERR_ARG;

	if (dev->iface != JAYLINK_HIF_TCP)
		return JAYLINK_ERR_NOT_SUPPORTED;

	memcpy(address, dev->ipv4_address, sizeof(dev->ipv4_address));

	return JAYLINK_OK;
}

/**
 * Get the MAC address of a device.
 *
 * @param[in] dev Device instance.
 * @param[out] address MAC address of the device on success and undefined on
 *                     failure. The length of the MAC address is
 *                     #JAYLINK_MAC_ADDRESS_LENGTH bytes.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_NOT_SUPPORTED Supported for devices with host interface
 *                                   #JAYLINK_HIF_TCP only.
 * @retval JAYLINK_ERR_NOT_AVAILABLE MAC address is not available.
 *
 * @since 0.2.0
 */
JAYLINK_API int jaylink_device_get_mac_address(
		const struct jaylink_device *dev, uint8_t *address)
{
	if (!dev || !address)
		return JAYLINK_ERR_ARG;

	if (dev->iface != JAYLINK_HIF_TCP)
		return JAYLINK_ERR_NOT_SUPPORTED;

	if (!dev->has_mac_address)
		return JAYLINK_ERR_NOT_AVAILABLE;

	memcpy(address, dev->mac_address, sizeof(dev->mac_address));

	return JAYLINK_OK;
}

/**
 * Get the hardware version of a device.
 *
 * @note The hardware type can not be obtained by this function, use
 *       jaylink_get_hardware_version() instead.
 *
 * @param[in] dev Device instance.
 * @param[out] version Hardware version of the device on success and undefined
 *                     on failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_NOT_SUPPORTED Supported for devices with host interface
 *                                   #JAYLINK_HIF_TCP only.
 * @retval JAYLINK_ERR_NOT_AVAILABLE Hardware version is not available.
 *
 * @since 0.2.0
 */
JAYLINK_API int jaylink_device_get_hardware_version(
		const struct jaylink_device *dev,
		struct jaylink_hardware_version *version)
{
	if (!dev || !version)
		return JAYLINK_ERR_ARG;

	if (dev->iface != JAYLINK_HIF_TCP)
		return JAYLINK_ERR_NOT_SUPPORTED;

	if (!dev->has_hw_version)
		return JAYLINK_ERR_NOT_AVAILABLE;

	*version = dev->hw_version;

	return JAYLINK_OK;
}

/**
 * Get the product name of a device.
 *
 * @param[in] dev Device instance.
 * @param[out] name Product name of the device on success and undefined on
 *                  failure. The maximum length of the product name is
 *                  #JAYLINK_PRODUCT_NAME_MAX_LENGTH bytes.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_NOT_SUPPORTED Supported for devices with host interface
 *                                   #JAYLINK_HIF_TCP only.
 * @retval JAYLINK_ERR_NOT_AVAILABLE Product name is not available.
 *
 * @since 0.2.0
 */
JAYLINK_API int jaylink_device_get_product_name(
		const struct jaylink_device *dev, char *name)
{
	if (!dev || !name)
		return JAYLINK_ERR_ARG;

	if (dev->iface != JAYLINK_HIF_TCP)
		return JAYLINK_ERR_NOT_SUPPORTED;

	if (!dev->has_product_name)
		return JAYLINK_ERR_NOT_AVAILABLE;

	memcpy(name, dev->product_name, sizeof(dev->product_name));

	return JAYLINK_OK;
}

/**
 * Get the nickname of a device.
 *
 * @param[in] dev Device instance.
 * @param[out] nickname Nickname of the device on success and undefined on
 *                      failure. The maximum length of the nickname is
 *                      #JAYLINK_NICKNAME_MAX_LENGTH bytes.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_NOT_SUPPORTED Supported for devices with host interface
 *                                   #JAYLINK_HIF_TCP only.
 * @retval JAYLINK_ERR_NOT_AVAILABLE Nickname is not available.
 *
 * @since 0.2.0
 */
JAYLINK_API int jaylink_device_get_nickname(const struct jaylink_device *dev,
		char *nickname)
{
	if (!dev || !nickname)
		return JAYLINK_ERR_ARG;

	if (dev->iface != JAYLINK_HIF_TCP)
		return JAYLINK_ERR_NOT_SUPPORTED;

	if (!dev->has_nickname)
		return JAYLINK_ERR_NOT_AVAILABLE;

	memcpy(nickname, dev->nickname, sizeof(dev->nickname));

	return JAYLINK_OK;
}

/**
 * Increment the reference count of a device.
 *
 * @param[in,out] dev Device instance.
 *
 * @return The given device instance on success, or NULL on invalid argument.
 *
 * @since 0.1.0
 */
JAYLINK_API struct jaylink_device *jaylink_ref_device(
		struct jaylink_device *dev)
{
	if (!dev)
		return NULL;

	dev->ref_count++;

	return dev;
}

/**
 * Decrement the reference count of a device.
 *
 * @param[in,out] dev Device instance.
 *
 * @since 0.1.0
 */
JAYLINK_API void jaylink_unref_device(struct jaylink_device *dev)
{
	struct jaylink_context *ctx;

	if (!dev)
		return;

	dev->ref_count--;

	if (!dev->ref_count) {
		ctx = dev->ctx;
		ctx->devs = list_remove(dev->ctx->devs, dev);

		if (dev->iface == JAYLINK_HIF_USB) {
#ifdef HAVE_LIBUSB
			log_dbg(ctx, "Device destroyed (bus:address = "
				"%03u:%03u).",
				libusb_get_bus_number(dev->usb_dev),
				libusb_get_device_address(dev->usb_dev));

			libusb_unref_device(dev->usb_dev);
#endif
		} else if (dev->iface == JAYLINK_HIF_TCP) {
			log_dbg(ctx, "Device destroyed (IPv4 address = %s).",
				dev->ipv4_address);
		} else {
			log_err(ctx, "BUG: Invalid host interface: %u.",
				dev->iface);
		}

		free(dev);
	}
}

static struct jaylink_device_handle *allocate_device_handle(
		struct jaylink_device *dev)
{
	struct jaylink_device_handle *devh;

	devh = malloc(sizeof(struct jaylink_device_handle));

	if (!devh)
		return NULL;

	devh->dev = jaylink_ref_device(dev);

	return devh;
}

static void free_device_handle(struct jaylink_device_handle *devh)
{
	jaylink_unref_device(devh->dev);
	free(devh);
}

/**
 * Open a device.
 *
 * @param[in,out] dev Device instance.
 * @param[out] devh Newly allocated handle for the opened device on success,
 *                  and undefined on failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_MALLOC Memory allocation error.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_open(struct jaylink_device *dev,
		struct jaylink_device_handle **devh)
{
	int ret;
	struct jaylink_device_handle *handle;

	if (!dev || !devh)
		return JAYLINK_ERR_ARG;

	handle = allocate_device_handle(dev);

	if (!handle) {
		log_err(dev->ctx, "Device handle malloc failed.");
		return JAYLINK_ERR_MALLOC;
	}

	ret = transport_open(handle);

	if (ret != JAYLINK_OK) {
		free_device_handle(handle);
		return ret;
	}

	*devh = handle;

	return JAYLINK_OK;
}

/**
 * Close a device.
 *
 * @param[in,out] devh Device instance.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_close(struct jaylink_device_handle *devh)
{
	int ret;

	if (!devh)
		return JAYLINK_ERR_ARG;

	ret = transport_close(devh);
	free_device_handle(devh);

	return ret;
}

/**
 * Get the device instance from a device handle.
 *
 * @note The reference count of the device instance is not increased.
 *
 * @param[in] devh Device handle.
 *
 * @return The device instance on success, or NULL on invalid argument.
 *
 * @since 0.1.0
 */
JAYLINK_API struct jaylink_device *jaylink_get_device(
		struct jaylink_device_handle *devh)
{
	if (!devh)
		return NULL;

	return devh->dev;
}

/**
 * Retrieve the firmware version of a device.
 *
 * @param[in,out] devh Device handle.
 * @param[out] version Newly allocated string which contains the firmware
 *                     version  on success, and undefined if @p length is zero
 *                     or on failure. The string is null-terminated and must be
 *                     free'd by the caller.
 * @param[out] length Length of the firmware version string including trailing
 *                    null-terminator on success, and undefined on failure.
 *                    Zero if no firmware version string is available.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_MALLOC Memory allocation error.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_get_firmware_version(
		struct jaylink_device_handle *devh, char **version,
		size_t *length)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[2];
	uint16_t dummy;
	char *tmp;

	if (!devh || !version || !length)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write_read(devh, 1, 2, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_GET_VERSION;

	ret = transport_write(devh, buf, 1);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, buf, 2);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	dummy = buffer_get_u16(buf, 0);
	*length = dummy;

	if (!dummy)
		return JAYLINK_OK;

	ret = transport_start_read(devh, dummy);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	tmp = malloc(dummy);

	if (!tmp) {
		log_err(ctx, "Firmware version string malloc failed.");
		return JAYLINK_ERR_MALLOC;
	}

	ret = transport_read(devh, (uint8_t *)tmp, dummy);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		free(tmp);
		return ret;
	}

	/* Last byte is reserved for null-terminator. */
	tmp[dummy - 1] = 0;
	*version = tmp;

	return JAYLINK_OK;
}

/**
 * Retrieve the hardware information of a device.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_GET_HW_INFO capability.
 *
 * @param[in,out] devh Device handle.
 * @param[in] mask A bit field where each set bit represents hardware
 *                 information to request. See #jaylink_hardware_info for a
 *                 description of the hardware information and their bit
 *                 positions.
 * @param[out] info Array to store the hardware information on success. Its
 *                  content is undefined on failure. The array must be large
 *                  enough to contain at least as many elements as bits set in
 *                  @a mask.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_get_hardware_info(struct jaylink_device_handle *devh,
		uint32_t mask, uint32_t *info)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[5];
	unsigned int i;
	unsigned int num;
	unsigned int length;

	if (!devh || !mask || !info)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	num = 0;

	for (i = 0; i < 32; i++) {
		if (mask & (1 << i))
			num++;
	}

	length = num * sizeof(uint32_t);

	ret = transport_start_write_read(devh, 5, length, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_GET_HW_INFO;
	buffer_set_u32(buf, mask, 1);

	ret = transport_write(devh, buf, 5);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, (uint8_t *)info, length);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	for (i = 0; i < num; i++)
		info[i] = buffer_get_u32((uint8_t *)info,
			i * sizeof(uint32_t));

	return JAYLINK_OK;
}

/**
 * Retrieve the counter values of a device.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_GET_COUNTERS capability.
 *
 * @param[in,out] devh Device handle.
 * @param[in] mask A bit field where each set bit represents a counter value to
 *                 request. See #jaylink_counter for a description of the
 *                 counters and their bit positions.
 * @param[out] values Array to store the counter values on success. Its content
 *                    is undefined on failure. The array must be large enough
 *                    to contain at least as many elements as bits set in @p
 *                    mask.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.2.0
 */
JAYLINK_API int jaylink_get_counters(struct jaylink_device_handle *devh,
		uint32_t mask, uint32_t *values)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[5];
	unsigned int i;
	unsigned int num;
	unsigned int length;

	if (!devh || !mask || !values)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	num = 0;

	for (i = 0; i < 32; i++) {
		if (mask & (1 << i))
			num++;
	}

	length = num * sizeof(uint32_t);
	ret = transport_start_write_read(devh, 5, length, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_GET_COUNTERS;
	buffer_set_u32(buf, mask, 1);

	ret = transport_write(devh, buf, 5);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, (uint8_t *)values, length);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	for (i = 0; i < num; i++)
		values[i] = buffer_get_u32((uint8_t *)values,
			i * sizeof(uint32_t));

	return JAYLINK_OK;
}

/**
 * Retrieve the hardware version of a device.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_GET_HW_VERSION capability.
 *
 * @warning This function may return a value for @p version where
 *          #jaylink_hardware_version::type is not covered by
 *          #jaylink_hardware_type.
 *
 * @param[in,out] devh Device handle.
 * @param[out] version Hardware version on success, and undefined on failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_get_hardware_version(
		struct jaylink_device_handle *devh,
		struct jaylink_hardware_version *version)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[4];
	uint32_t tmp;

	if (!devh || !version)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write_read(devh, 1, 4, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_GET_HW_VERSION;

	ret = transport_write(devh, buf, 1);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, buf, 4);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	tmp = buffer_get_u32(buf, 0);

	version->type = (tmp / 1000000) % 100;
	version->major = (tmp / 10000) % 100;
	version->minor = (tmp / 100) % 100;
	version->revision = tmp % 100;

	return JAYLINK_OK;
}

/**
 * Retrieve the hardware status of a device.
 *
 * @param[in,out] devh Device handle.
 * @param[out] status Hardware status on success, and undefined on failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_get_hardware_status(struct jaylink_device_handle *devh,
		struct jaylink_hardware_status *status)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[8];

	if (!devh || !status)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write_read(devh, 1, 8, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_GET_HW_STATUS;

	ret = transport_write(devh, buf, 1);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, buf, 8);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	status->target_voltage = buffer_get_u16(buf, 0);
	status->tck = buf[2];
	status->tdi = buf[3];
	status->tdo = buf[4];
	status->tms = buf[5];
	status->tres = buf[6];
	status->trst = buf[7];

	return JAYLINK_OK;
}

/**
 * Retrieve the capabilities of a device.
 *
 * The capabilities are stored in a 32-bit bit array consisting of
 * #JAYLINK_DEV_CAPS_SIZE bytes where each individual bit represents a
 * capability. The first bit of this array is the least significant bit of the
 * first byte and the following bits are sequentially numbered in order of
 * increasing bit significance and byte index. A set bit indicates a supported
 * capability. See #jaylink_device_capability for a description of the
 * capabilities and their bit positions.
 *
 * @param[in,out] devh Device handle.
 * @param[out] caps Buffer to store capabilities on success. Its content is
 *                  undefined on failure. The buffer must be large enough to
 *                  contain at least #JAYLINK_DEV_CAPS_SIZE bytes.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @see jaylink_get_extended_caps()
 * @see jaylink_has_cap()
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_get_caps(struct jaylink_device_handle *devh,
		uint8_t *caps)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[1];

	if (!devh || !caps)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write_read(devh, 1, JAYLINK_DEV_CAPS_SIZE, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_GET_CAPS;

	ret = transport_write(devh, buf, 1);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, caps, JAYLINK_DEV_CAPS_SIZE);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	return JAYLINK_OK;
}

/**
 * Retrieve the extended capabilities of a device.
 *
 * The extended capabilities are stored in a 256-bit bit array consisting of
 * #JAYLINK_DEV_EXT_CAPS_SIZE bytes. See jaylink_get_caps() for a further
 * description of how the capabilities are represented in this bit array. For a
 * description of the capabilities and their bit positions, see
 * #jaylink_device_capability.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_GET_EXT_CAPS capability.
 *
 * @param[in,out] devh Device handle.
 * @param[out] caps Buffer to store capabilities on success. Its content is
 *                  undefined on failure. The buffer must be large enough to
 *                  contain at least #JAYLINK_DEV_EXT_CAPS_SIZE bytes.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @see jaylink_get_caps()
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_get_extended_caps(struct jaylink_device_handle *devh,
		uint8_t *caps)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[1];

	if (!devh || !caps)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write_read(devh, 1, JAYLINK_DEV_EXT_CAPS_SIZE,
		true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_GET_EXT_CAPS;

	ret = transport_write(devh, buf, 1);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, caps, JAYLINK_DEV_EXT_CAPS_SIZE);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	return JAYLINK_OK;
}

/**
 * Retrieve the size of free memory of a device.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_GET_FREE_MEMORY capability.
 *
 * @param[in,out] devh Device handle.
 * @param[out] size Size of free memory in bytes on success, and undefined on
 *                  failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_get_free_memory(struct jaylink_device_handle *devh,
		uint32_t *size)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[4];

	if (!devh || !size)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write_read(devh, 1, 4, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_GET_FREE_MEMORY;

	ret = transport_write(devh, buf, 1);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, buf, 4);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	*size = buffer_get_u32(buf, 0);

	return JAYLINK_OK;
}

/**
 * Read the raw configuration data of a device.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_READ_CONFIG capability.
 *
 * @param[in,out] devh Device handle.
 * @param[out] config Buffer to store configuration data on success. Its
 *                    content is undefined on failure. The buffer must be large
 *                    enough to contain at least
 *                    #JAYLINK_DEV_CONFIG_SIZE bytes.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_read_raw_config(struct jaylink_device_handle *devh,
		uint8_t *config)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[1];

	if (!devh || !config)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write_read(devh, 1, JAYLINK_DEV_CONFIG_SIZE,
		true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_READ_CONFIG;

	ret = transport_write(devh, buf, 1);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, config, JAYLINK_DEV_CONFIG_SIZE);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	return JAYLINK_OK;
}

/**
 * Write the raw configuration data of a device.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_WRITE_CONFIG capability.
 *
 * @param[in,out] devh Device handle.
 * @param[in] config Buffer to write configuration data from. The size of the
 *                   configuration data is expected to be
 *                   #JAYLINK_DEV_CONFIG_SIZE bytes.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_write_raw_config(struct jaylink_device_handle *devh,
		const uint8_t *config)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[1];

	if (!devh || !config)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write(devh, 1 + JAYLINK_DEV_CONFIG_SIZE, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_WRITE_CONFIG;

	ret = transport_write(devh, buf, 1);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_write(devh, config, JAYLINK_DEV_CONFIG_SIZE);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	return JAYLINK_OK;
}

static void parse_conn_table(struct jaylink_connection *conns,
		const uint8_t *buffer, uint16_t num, uint16_t entry_size)
{
	unsigned int i;
	size_t offset;
	struct in_addr in;

	offset = 0;

	for (i = 0; i < num; i++) {
		conns[i].pid = buffer_get_u32(buffer, offset);

		in.s_addr = buffer_get_u32(buffer, offset + 4);
		/*
		 * Use inet_ntoa() instead of inet_ntop() because the latter
		 * requires at least Windows Vista.
		 */
		strcpy(conns[i].hid, inet_ntoa(in));

		conns[i].iid = buffer[offset + 8];
		conns[i].cid = buffer[offset + 9];
		conns[i].handle = buffer_get_u16(buffer, offset + 10);
		conns[i].timestamp = buffer_get_u32(buffer, offset + 12);
		offset = offset + entry_size;
	}
}

static bool _inet_pton(const char *str, struct in_addr *in)
{
#ifdef _WIN32
	int ret;
	struct sockaddr_in sock_in;
	int length;

	length = sizeof(sock_in);

	/*
	 * Use WSAStringToAddress() instead of inet_pton() because the latter
	 * requires at least Windows Vista.
	 */
	ret = WSAStringToAddress((LPTSTR)str, AF_INET, NULL,
		(LPSOCKADDR)&sock_in, &length);

	if (ret != 0)
		return false;

	*in = sock_in.sin_addr;
#else
	if (inet_pton(AF_INET, str, in) != 1)
		return false;
#endif

	return true;
}

/**
 * Register a connection on a device.
 *
 * A connection can be registered by using 0 as handle. Additional information
 * about the connection can be attached whereby the timestamp is a read-only
 * value and therefore ignored for registration. On success, a new handle
 * greater than 0 is obtained from the device.
 *
 * However, if an obtained handle does not appear in the list of device
 * connections, the connection was not registered because the maximum number of
 * connections on the device is reached.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_REGISTER capability.
 *
 * Example code:
 * @code{.c}
 * static bool register_connection(struct jaylink_device_handle *devh,
 *                 struct jaylink_connection *conn)
 * {
 *         int ret;
 *         struct jaylink_connection conns[JAYLINK_MAX_CONNECTIONS];
 *         bool found_handle;
 *         size_t count;
 *         size_t i;
 *
 *         conn->handle = 0;
 *         conn->pid = 0;
 *         strcpy(conn->hid, "0.0.0.0");
 *         conn->iid = 0;
 *         conn->cid = 0;
 *
 *         ret = jaylink_register(devh, conn, conns, &count);
 *
 *         if (ret != JAYLINK_OK) {
 *                 printf("jaylink_register() failed: %s.\n",
 *                         jaylink_strerror(ret));
 *                 return false;
 *         }
 *
 *         found_handle = false;
 *
 *         for (i = 0; i < count; i++) {
 *                 if (conns[i].handle == conn->handle) {
 *                         found_handle = true;
 *                         break;
 *                 }
 *         }
 *
 *         if (!found_handle) {
 *                 printf("Maximum number of connections reached.\n");
 *                 return false;
 *         }
 *
 *         printf("Connection successfully registered.\n");
 *
 *         return true;
 * }
 * @endcode
 *
 * @param[in,out] devh Device handle.
 * @param[in,out] connection Connection to register on the device.
 * @param[out] connections Array to store device connections on success.
 *                         Its content is undefined on failure. The array must
 *                         be large enough to contain at least
 *                         #JAYLINK_MAX_CONNECTIONS elements.
 * @param[out] count Number of device connections on success, and undefined on
 *                   failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_PROTO Protocol violation.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @see jaylink_unregister()
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_register(struct jaylink_device_handle *devh,
		struct jaylink_connection *connection,
		struct jaylink_connection *connections, size_t *count)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[REG_MAX_SIZE];
	uint16_t handle;
	uint16_t num;
	uint16_t entry_size;
	uint32_t size;
	uint32_t table_size;
	uint16_t info_size;
	struct in_addr in;

	if (!devh || !connection || !connections || !count)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;

	buf[0] = CMD_REGISTER;
	buf[1] = REG_CMD_REGISTER;
	buffer_set_u32(buf, connection->pid, 2);

	if (!_inet_pton(connection->hid, &in))
		return JAYLINK_ERR_ARG;

	buffer_set_u32(buf, in.s_addr, 6);

	buf[10] = connection->iid;
	buf[11] = connection->cid;
	buffer_set_u16(buf, connection->handle, 12);

	ret = transport_start_write_read(devh, 14, REG_MIN_SIZE, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_write(devh, buf, 14);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, buf, REG_MIN_SIZE);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	handle = buffer_get_u16(buf, 0);
	num = buffer_get_u16(buf, 2);
	entry_size = buffer_get_u16(buf, 4);
	info_size = buffer_get_u16(buf, 6);

	if (num > JAYLINK_MAX_CONNECTIONS) {
		log_err(ctx, "Maximum number of device connections exceeded: "
			"%u.", num);
		return JAYLINK_ERR_PROTO;
	}

	if (entry_size != REG_CONN_INFO_SIZE) {
		log_err(ctx, "Invalid connection entry size: %u bytes.",
			entry_size);
		return JAYLINK_ERR_PROTO;
	}

	table_size = num * entry_size;
	size = REG_HEADER_SIZE + table_size + info_size;

	if (size > REG_MAX_SIZE) {
		log_err(ctx, "Maximum registration information size exceeded: "
			"%u bytes.", size);
		return JAYLINK_ERR_PROTO;
	}

	if (size > REG_MIN_SIZE) {
		ret = transport_start_read(devh, size - REG_MIN_SIZE);

		if (ret != JAYLINK_OK) {
			log_err(ctx, "transport_start_read() failed: %s.",
				jaylink_strerror(ret));
			return JAYLINK_ERR;
		}

		ret = transport_read(devh, buf + REG_MIN_SIZE,
			size - REG_MIN_SIZE);

		if (ret != JAYLINK_OK) {
			log_err(ctx, "transport_read() failed: %s.",
				jaylink_strerror(ret));
			return JAYLINK_ERR;
		}
	}

	if (!handle) {
		log_err(ctx, "Obtained invalid connection handle.");
		return JAYLINK_ERR_PROTO;
	}

	connection->handle = handle;
	parse_conn_table(connections, buf + REG_HEADER_SIZE, num, entry_size);

	*count = num;

	return JAYLINK_OK;
}

/**
 * Unregister a connection from a device.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_REGISTER capability.
 *
 * @param[in,out] devh Device handle.
 * @param[in,out] connection Connection to unregister from the device.
 * @param[out] connections Array to store device connections on success.
 *                         Its content is undefined on failure. The array must
 *                         be large enough to contain at least
 *                         #JAYLINK_MAX_CONNECTIONS elements.
 * @param[out] count Number of device connections on success, and undefined on
 *                   failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_PROTO Protocol violation.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @see jaylink_register()
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_unregister(struct jaylink_device_handle *devh,
		const struct jaylink_connection *connection,
		struct jaylink_connection *connections, size_t *count)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[REG_MAX_SIZE];
	uint16_t num;
	uint16_t entry_size;
	uint32_t size;
	uint32_t table_size;
	uint16_t info_size;
	struct in_addr in;

	if (!devh || !connection || !connections || !count)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;

	buf[0] = CMD_REGISTER;
	buf[1] = REG_CMD_UNREGISTER;
	buffer_set_u32(buf, connection->pid, 2);

	if (!_inet_pton(connection->hid, &in))
		return JAYLINK_ERR_ARG;

	buffer_set_u32(buf, in.s_addr, 6);

	buf[10] = connection->iid;
	buf[11] = connection->cid;
	buffer_set_u16(buf, connection->handle, 12);

	ret = transport_start_write_read(devh, 14, REG_MIN_SIZE, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_write(devh, buf, 14);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, buf, REG_MIN_SIZE);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	num = buffer_get_u16(buf, 2);
	entry_size = buffer_get_u16(buf, 4);
	info_size = buffer_get_u16(buf, 6);

	if (num > JAYLINK_MAX_CONNECTIONS) {
		log_err(ctx, "Maximum number of device connections exceeded: "
			"%u.", num);
		return JAYLINK_ERR_PROTO;
	}

	if (entry_size != REG_CONN_INFO_SIZE) {
		log_err(ctx, "Invalid connection entry size: %u bytes.",
			entry_size);
		return JAYLINK_ERR_PROTO;
	}

	table_size = num * entry_size;
	size = REG_HEADER_SIZE + table_size + info_size;

	if (size > REG_MAX_SIZE) {
		log_err(ctx, "Maximum registration information size exceeded: "
			"%u bytes.", size);
		return JAYLINK_ERR_PROTO;
	}

	if (size > REG_MIN_SIZE) {
		ret = transport_start_read(devh, size - REG_MIN_SIZE);

		if (ret != JAYLINK_OK) {
			log_err(ctx, "transport_start_read() failed: %s.",
				jaylink_strerror(ret));
			return JAYLINK_ERR;
		}

		ret = transport_read(devh, buf + REG_MIN_SIZE,
			size - REG_MIN_SIZE);

		if (ret != JAYLINK_OK) {
			log_err(ctx, "transport_read() failed: %s.",
				jaylink_strerror(ret));
			return JAYLINK_ERR;
		}
	}

	parse_conn_table(connections, buf + REG_HEADER_SIZE, num, entry_size);

	*count = num;

	return JAYLINK_OK;
}
