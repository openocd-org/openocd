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

#ifndef LIBJAYLINK_LIBJAYLINK_INTERNAL_H
#define LIBJAYLINK_LIBJAYLINK_INTERNAL_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#include <sys/types.h>
#ifdef _WIN32
#include <ws2tcpip.h>
#else
#include <sys/socket.h>
#include <arpa/inet.h>
#endif

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_LIBUSB
#include <libusb.h>
#endif

#include "libjaylink.h"

/**
 * @file
 *
 * Internal libjaylink header file.
 */

/** Macro to mark private libjaylink symbol. */
#if defined(_WIN32) || defined(__MSYS__) || defined(__CYGWIN__)
#define JAYLINK_PRIV
#else
#define JAYLINK_PRIV __attribute__ ((visibility ("hidden")))
#endif

/** Calculate the minimum of two numeric values. */
#define MIN(a, b) (((a) < (b)) ? (a) : (b))

struct jaylink_context {
#ifdef HAVE_LIBUSB
	/** libusb context. */
	struct libusb_context *usb_ctx;
#endif
	/**
	 * List of allocated device instances.
	 *
	 * Used to prevent multiple device instances for the same device.
	 */
	struct list *devs;
	/** List of recently discovered devices. */
	struct list *discovered_devs;
	/** Current log level. */
	enum jaylink_log_level log_level;
	/** Log callback function. */
	jaylink_log_callback log_callback;
	/** User data to be passed to the log callback function. */
	void *log_callback_data;
	/** Log domain. */
	char log_domain[JAYLINK_LOG_DOMAIN_MAX_LENGTH + 1];
};

struct jaylink_device {
	/** libjaylink context. */
	struct jaylink_context *ctx;
	/** Number of references held on this device instance. */
	size_t ref_count;
	/** Host interface. */
	enum jaylink_host_interface iface;
	/**
	 * Serial number of the device.
	 *
	 * This number is for enumeration purpose only and can differ from the
	 * real serial number of the device.
	 */
	uint32_t serial_number;
	/** Indicates whether the serial number is valid. */
	bool valid_serial_number;
#ifdef HAVE_LIBUSB
	/** libusb device instance. */
	struct libusb_device *usb_dev;
	/** USB address of the device. */
	uint8_t usb_address;
#endif
	/**
	 * IPv4 address.
	 *
	 * The address is encoded as string in quad-dotted decimal format.
	 *
	 * This field is used for devices with host interface #JAYLINK_HIF_TCP
	 * only.
	 */
	char ipv4_address[INET_ADDRSTRLEN];
	/**
	 * Media Access Control (MAC) address.
	 *
	 * This field is used for devices with host interface #JAYLINK_HIF_TCP
	 * only.
	 */
	uint8_t mac_address[JAYLINK_MAC_ADDRESS_LENGTH];
	/** Indicates whether the MAC address is available. */
	bool has_mac_address;
	/**
	 * Product name.
	 *
	 * This field is used for devices with host interface #JAYLINK_HIF_TCP
	 * only.
	 */
	char product_name[JAYLINK_PRODUCT_NAME_MAX_LENGTH];
	/** Indicates whether the product name is available. */
	bool has_product_name;
	/**
	 * Nickname.
	 *
	 * This field is used for devices with host interface #JAYLINK_HIF_TCP
	 * only.
	 */
	char nickname[JAYLINK_NICKNAME_MAX_LENGTH];
	/** Indicates whether the nickname is available. */
	bool has_nickname;
	/**
	 * Hardware version.
	 *
	 * This field is used for devices with host interface #JAYLINK_HIF_TCP
	 * only.
	 */
	struct jaylink_hardware_version hw_version;
	/** Indicates whether the hardware version is available. */
	bool has_hw_version;
};

struct jaylink_device_handle {
	/** Device instance. */
	struct jaylink_device *dev;
	/**
	 * Buffer for write and read operations.
	 *
	 * Note that write and read operations are always processed
	 * consecutively and therefore the same buffer can be used for both.
	 */
	uint8_t *buffer;
	/** Buffer size. */
	size_t buffer_size;
	/** Number of bytes left for the read operation. */
	size_t read_length;
	/** Number of bytes available in the buffer to be read. */
	size_t bytes_available;
	/** Current read position in the buffer. */
	size_t read_pos;
	/**
	 * Number of bytes left to be written before the write operation will
	 * be performed.
	 */
	size_t write_length;
	/**
	 * Current write position in the buffer.
	 *
	 * This is equivalent to the number of bytes in the buffer and used for
	 * write operations only.
	 */
	size_t write_pos;
#ifdef HAVE_LIBUSB
	/** libusb device handle. */
	struct libusb_device_handle *usb_devh;
	/** USB interface number of the device. */
	uint8_t interface_number;
	/** USB interface IN endpoint of the device. */
	uint8_t endpoint_in;
	/** USB interface OUT endpoint of the device. */
	uint8_t endpoint_out;
#endif
	/**
	 * Socket descriptor.
	 *
	 * This field is used for devices with host interface #JAYLINK_HIF_TCP
	 * only.
	 */
	int sock;
};

struct list {
	void *data;
	struct list *next;
};

typedef bool (*list_compare_callback)(const void *data, const void *user_data);

/*--- buffer.c --------------------------------------------------------------*/

JAYLINK_PRIV void buffer_set_u16(uint8_t *buffer, uint16_t value,
		size_t offset);
JAYLINK_PRIV uint16_t buffer_get_u16(const uint8_t *buffer, size_t offset);
JAYLINK_PRIV void buffer_set_u32(uint8_t *buffer, uint32_t value,
		size_t offset);
JAYLINK_PRIV uint32_t buffer_get_u32(const uint8_t *buffer, size_t offset);

/*--- device.c --------------------------------------------------------------*/

JAYLINK_PRIV struct jaylink_device *device_allocate(
		struct jaylink_context *ctx);

/*--- discovery_tcp.c -------------------------------------------------------*/

JAYLINK_PRIV int discovery_tcp_scan(struct jaylink_context *ctx);

/*--- discovery_usb.c -------------------------------------------------------*/

JAYLINK_PRIV int discovery_usb_scan(struct jaylink_context *ctx);

/*--- list.c ----------------------------------------------------------------*/

JAYLINK_PRIV struct list *list_prepend(struct list *list, void *data);
JAYLINK_PRIV struct list *list_remove(struct list *list, const void *data);
JAYLINK_PRIV struct list *list_find_custom(struct list *list,
		list_compare_callback callback, const void *user_data);
JAYLINK_PRIV size_t list_length(struct list *list);
JAYLINK_PRIV void list_free(struct list *list);

/*--- log.c -----------------------------------------------------------------*/

JAYLINK_PRIV int log_vprintf(const struct jaylink_context *ctx,
		enum jaylink_log_level level, const char *format, va_list args,
		void *user_data);
JAYLINK_PRIV void log_err(const struct jaylink_context *ctx,
		const char *format, ...);
JAYLINK_PRIV void log_warn(const struct jaylink_context *ctx,
		const char *format, ...);
JAYLINK_PRIV void log_info(const struct jaylink_context *ctx,
		const char *format, ...);
JAYLINK_PRIV void log_dbg(const struct jaylink_context *ctx,
		const char *format, ...);
JAYLINK_PRIV void log_dbgio(const struct jaylink_context *ctx,
		const char *format, ...);

/*--- socket.c --------------------------------------------------------------*/

JAYLINK_PRIV bool socket_close(int sock);
JAYLINK_PRIV bool socket_bind(int sock, const struct sockaddr *address,
		size_t length);
JAYLINK_PRIV bool socket_send(int sock, const void *buffer, size_t *length,
		int flags);
JAYLINK_PRIV bool socket_recv(int sock, void *buffer, size_t *length,
		int flags);
JAYLINK_PRIV bool socket_sendto(int sock, const void *buffer, size_t *length,
		int flags, const struct sockaddr *address,
		size_t address_length);
JAYLINK_PRIV bool socket_recvfrom(int sock, void *buffer, size_t *length,
		int flags, struct sockaddr *address, size_t *address_length);
JAYLINK_PRIV bool socket_set_option(int sock, int level, int option,
		const void *value, size_t length);

/*--- transport.c -----------------------------------------------------------*/

JAYLINK_PRIV int transport_open(struct jaylink_device_handle *devh);
JAYLINK_PRIV int transport_close(struct jaylink_device_handle *devh);
JAYLINK_PRIV int transport_start_write_read(struct jaylink_device_handle *devh,
		size_t write_length, size_t read_length, bool has_command);
JAYLINK_PRIV int transport_start_write(struct jaylink_device_handle *devh,
		size_t length, bool has_command);
JAYLINK_PRIV int transport_start_read(struct jaylink_device_handle *devh,
		size_t length);
JAYLINK_PRIV int transport_write(struct jaylink_device_handle *devh,
		const uint8_t *buffer, size_t length);
JAYLINK_PRIV int transport_read(struct jaylink_device_handle *devh,
		uint8_t *buffer, size_t length);

/*--- transport_usb.c -------------------------------------------------------*/

JAYLINK_PRIV int transport_usb_open(struct jaylink_device_handle *devh);
JAYLINK_PRIV int transport_usb_close(struct jaylink_device_handle *devh);
JAYLINK_PRIV int transport_usb_start_write_read(
		struct jaylink_device_handle *devh, size_t write_length,
		size_t read_length, bool has_command);
JAYLINK_PRIV int transport_usb_start_write(struct jaylink_device_handle *devh,
		size_t length, bool has_command);
JAYLINK_PRIV int transport_usb_start_read(struct jaylink_device_handle *devh,
		size_t length);
JAYLINK_PRIV int transport_usb_write(struct jaylink_device_handle *devh,
		const uint8_t *buffer, size_t length);
JAYLINK_PRIV int transport_usb_read(struct jaylink_device_handle *devh,
		uint8_t *buffer, size_t length);

/*--- transport_tcp.c -------------------------------------------------------*/

JAYLINK_PRIV int transport_tcp_open(struct jaylink_device_handle *devh);
JAYLINK_PRIV int transport_tcp_close(struct jaylink_device_handle *devh);
JAYLINK_PRIV int transport_tcp_start_write_read(
		struct jaylink_device_handle *devh, size_t write_length,
		size_t read_length, bool has_command);
JAYLINK_PRIV int transport_tcp_start_write(struct jaylink_device_handle *devh,
		size_t length, bool has_command);
JAYLINK_PRIV int transport_tcp_start_read(struct jaylink_device_handle *devh,
		size_t length);
JAYLINK_PRIV int transport_tcp_write(struct jaylink_device_handle *devh,
		const uint8_t *buffer, size_t length);
JAYLINK_PRIV int transport_tcp_read(struct jaylink_device_handle *devh,
		uint8_t *buffer, size_t length);

#endif /* LIBJAYLINK_LIBJAYLINK_INTERNAL_H */
