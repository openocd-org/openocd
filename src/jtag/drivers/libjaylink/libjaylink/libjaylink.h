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

#ifndef LIBJAYLINK_LIBJAYLINK_H
#define LIBJAYLINK_LIBJAYLINK_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdarg.h>
#ifdef _WIN32
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#endif

/**
 * @file
 *
 * Public libjaylink header file to be used by applications.
 */

/** Error codes returned by libjaylink functions. */
enum jaylink_error {
	/** No error. */
	JAYLINK_OK = 0,
	/** Unspecified error. */
	JAYLINK_ERR = -1,
	/** Invalid argument. */
	JAYLINK_ERR_ARG = -2,
	/** Memory allocation error. */
	JAYLINK_ERR_MALLOC = -3,
	/** Timeout occurred. */
	JAYLINK_ERR_TIMEOUT = -4,
	/** Protocol violation. */
	JAYLINK_ERR_PROTO = -5,
	/** Entity not available. */
	JAYLINK_ERR_NOT_AVAILABLE = -6,
	/** Operation not supported. */
	JAYLINK_ERR_NOT_SUPPORTED = -7,
	/** Input/output error. */
	JAYLINK_ERR_IO = -8,
	/** Device: unspecified error. */
	JAYLINK_ERR_DEV = -1000,
	/** Device: operation not supported. */
	JAYLINK_ERR_DEV_NOT_SUPPORTED = -1001,
	/** Device: entity not available. */
	JAYLINK_ERR_DEV_NOT_AVAILABLE = -1002,
	/** Device: not enough memory to perform operation. */
	JAYLINK_ERR_DEV_NO_MEMORY = -1003
};

/** libjaylink log levels. */
enum jaylink_log_level {
	/** Output no messages. */
	JAYLINK_LOG_LEVEL_NONE = 0,
	/** Output error messages. */
	JAYLINK_LOG_LEVEL_ERROR = 1,
	/** Output warnings. */
	JAYLINK_LOG_LEVEL_WARNING = 2,
	/** Output informational messages. */
	JAYLINK_LOG_LEVEL_INFO = 3,
	/** Output debug messages. */
	JAYLINK_LOG_LEVEL_DEBUG = 4,
	/** Output I/O debug messages. */
	JAYLINK_LOG_LEVEL_DEBUG_IO = 5
};

/** Default libjaylink log domain. */
#define JAYLINK_LOG_DOMAIN_DEFAULT	"jaylink: "

/** Maximum length of a libjaylink log domain in bytes. */
#define JAYLINK_LOG_DOMAIN_MAX_LENGTH	32

/** libjaylink capabilities. */
enum jaylink_capability {
	/** Library supports USB as host interface. */
	JAYLINK_CAP_HIF_USB = 0
};

/** Host interfaces. */
enum jaylink_host_interface {
	/** Universal Serial Bus (USB). */
	JAYLINK_HIF_USB = (1 << 0),
	/** Transmission Control Protocol (TCP). */
	JAYLINK_HIF_TCP = (1 << 1)
};

/**
 * USB addresses.
 *
 * The USB address is a way to identify USB devices and is related to the USB
 * Product ID (PID) of a device.
 */
enum jaylink_usb_address {
	/** USB address 0 (Product ID 0x0101). */
	JAYLINK_USB_ADDRESS_0 = 0,
	/** USB address 1 (Product ID 0x0102). */
	JAYLINK_USB_ADDRESS_1 = 1,
	/** USB address 2 (Product ID 0x0103). */
	JAYLINK_USB_ADDRESS_2 = 2,
	/** USB address 3 (Product ID 0x0104). */
	JAYLINK_USB_ADDRESS_3 = 3
};

/** Device capabilities. */
enum jaylink_device_capability {
	/** Device supports retrieval of the hardware version. */
	JAYLINK_DEV_CAP_GET_HW_VERSION = 1,
	/** Device supports adaptive clocking. */
	JAYLINK_DEV_CAP_ADAPTIVE_CLOCKING = 3,
	/** Device supports reading configuration data. */
	JAYLINK_DEV_CAP_READ_CONFIG = 4,
	/** Device supports writing configuration data. */
	JAYLINK_DEV_CAP_WRITE_CONFIG = 5,
	/** Device supports retrieval of target interface speeds. */
	JAYLINK_DEV_CAP_GET_SPEEDS = 9,
	/** Device supports retrieval of free memory size. */
	JAYLINK_DEV_CAP_GET_FREE_MEMORY = 11,
	/** Device supports retrieval of hardware information. */
	JAYLINK_DEV_CAP_GET_HW_INFO = 12,
	/** Device supports the setting of the target power supply. */
	JAYLINK_DEV_CAP_SET_TARGET_POWER = 13,
	/** Device supports target interface selection. */
	JAYLINK_DEV_CAP_SELECT_TIF = 17,
	/** Device supports retrieval of counter values. */
	JAYLINK_DEV_CAP_GET_COUNTERS = 19,
	/** Device supports capturing of SWO trace data. */
	JAYLINK_DEV_CAP_SWO = 23,
	/** Device supports file I/O operations. */
	JAYLINK_DEV_CAP_FILE_IO = 26,
	/** Device supports registration of connections. */
	JAYLINK_DEV_CAP_REGISTER = 27,
	/** Device supports retrieval of extended capabilities. */
	JAYLINK_DEV_CAP_GET_EXT_CAPS = 31,
	/** Device supports EMUCOM. */
	JAYLINK_DEV_CAP_EMUCOM = 33,
	/** Device supports ethernet connectivity. */
	JAYLINK_DEV_CAP_ETHERNET = 38
};

/** Hardware information. */
enum jaylink_hardware_info {
	/**
	 * Status of the target power supply.
	 *
	 * This indicates whether the target power supply on pin 19 of the
	 * 20-pin JTAG / SWD connector is enabled or disabled.
	 *
	 * @see jaylink_set_target_power()
	 */
	JAYLINK_HW_INFO_TARGET_POWER = (1 << 0),
	/** Current consumption of the target in mA. */
	JAYLINK_HW_INFO_ITARGET = (1 << 2),
	/** Peak current consumption of the target in mA. */
	JAYLINK_HW_INFO_ITARGET_PEAK = (1 << 3),
	/**
	 * Device's IPv4 address in network byte order.
	 *
	 * If the address is 0.0.0.0 and DHCP is enabled, no address is
         * assigned (yet).
	 *
	 * @note The value is valid only if the device is configured in DHCP
	 *       mode.
	 */
	JAYLINK_HW_INFO_IPV4_ADDRESS = (1 << 16),
	/**
	 * IPv4 netmask in network byte order.
	 *
	 * @note The value is valid only if the device is configured in DHCP
	 *       mode.
	 */
	JAYLINK_HW_INFO_IPV4_NETMASK = (1 << 17),
	/**
	 * Gateway IPv4 address in network byte order.
	 *
	 * @note The value is valid only if the device is configured in DHCP
	 *       mode.
	 */
	JAYLINK_HW_INFO_IPV4_GATEWAY = (1 << 18),
	/**
	 * DNS server IPv4 address in network byte order.
	 *
	 * @note The value is valid only if the device is configured in DHCP
	 *       mode.
	 */
	JAYLINK_HW_INFO_IPV4_DNS = (1 << 19)
};

/** Device counters. */
enum jaylink_counter {
	/** Time the device is connected to a target in milliseconds. */
	JAYLINK_COUNTER_TARGET_TIME = (1 << 0),
	/**
	 * Number of times the device was connected or disconnected from a
	 * target.
	 */
	JAYLINK_COUNTER_TARGET_CONNECTIONS = (1 << 1)
};

/** Device hardware types. */
enum jaylink_hardware_type {
	/** J-Link. */
	JAYLINK_HW_TYPE_JLINK = 0,
	/** Flasher. */
	JAYLINK_HW_TYPE_FLASHER = 2,
	/** J-Link Pro. */
	JAYLINK_HW_TYPE_JLINK_PRO = 3
};

/** Target interfaces. */
enum jaylink_target_interface {
	/** Joint Test Action Group, IEEE 1149.1 (JTAG). */
	JAYLINK_TIF_JTAG = 0,
	/** Serial Wire Debug (SWD). */
	JAYLINK_TIF_SWD = 1,
	/** Background Debug Mode 3 (BDM3). */
	JAYLINK_TIF_BDM3 = 2,
	/** Renesas’ single-wire debug interface (FINE). */
	JAYLINK_TIF_FINE = 3,
	/** 2-wire JTAG for PIC32 compliant devices. */
	JAYLINK_TIF_2W_JTAG_PIC32 = 4,
};

/**
 * JTAG command versions.
 *
 * The JTAG command version only affects the device and the communication
 * protocol. The behaviour of a JTAG operation is not affected at all.
 */
enum jaylink_jtag_version {
	/**
	 * JTAG command version 2.
	 *
	 * This version is obsolete for major hardware version 5 and above. Use
	 * #JAYLINK_JTAG_VERSION_3 for these versions instead.
	 */
	JAYLINK_JTAG_VERSION_2 = 1,
	/** JTAG command version 3. */
	JAYLINK_JTAG_VERSION_3 = 2
};

/** Serial Wire Output (SWO) capture modes. */
enum jaylink_swo_mode {
	/** Universal Asynchronous Receiver Transmitter (UART). */
	JAYLINK_SWO_MODE_UART = 0
};

/** Target interface speed information. */
struct jaylink_speed {
	/** Base frequency in Hz. */
	uint32_t freq;
	/** Minimum frequency divider. */
	uint16_t div;
};

/** Serial Wire Output (SWO) speed information. */
struct jaylink_swo_speed {
	/** Base frequency in Hz. */
	uint32_t freq;
	/** Minimum frequency divider. */
	uint32_t min_div;
	/** Maximum frequency divider. */
	uint32_t max_div;
	/** Minimum prescaler. */
	uint32_t min_prescaler;
	/** Maximum prescaler. */
	uint32_t max_prescaler;
};

/** Device hardware version. */
struct jaylink_hardware_version {
	/** Hardware type. */
	enum jaylink_hardware_type type;
	/** Major version. */
	uint8_t major;
	/** Minor version. */
	uint8_t minor;
	/** Revision number. */
	uint8_t revision;
};

/** Device hardware status. */
struct jaylink_hardware_status {
	/** Target reference voltage in mV. */
	uint16_t target_voltage;
	/** TCK pin state. */
	bool tck;
	/** TDI pin state. */
	bool tdi;
	/** TDO pin state. */
	bool tdo;
	/** TMS pin state. */
	bool tms;
	/** TRES pin state. */
	bool tres;
	/** TRST pin state. */
	bool trst;
};

/** Device connection. */
struct jaylink_connection {
	/** Handle. */
	uint16_t handle;
	/**
	 * Process ID (PID).
	 *
	 * Identification of the client process. Usually this is the
	 * Process ID (PID) of the client process in an arbitrary format.
	 */
	uint32_t pid;
	/**
	 * Host ID (HID).
	 *
	 * IPv4 address string of the client in quad-dotted decimal format
	 * (e.g. 192.0.2.235). The address 0.0.0.0 should be used for the
	 * registration of an USB connection.
	 */
	char hid[INET_ADDRSTRLEN];
	/** IID. */
	uint8_t iid;
	/** CID. */
	uint8_t cid;
	/**
	 * Timestamp of the last registration in milliseconds.
	 *
	 * The timestamp is relative to the time the device was powered up.
	 */
	uint32_t timestamp;
};

/** Target interface speed value for adaptive clocking. */
#define JAYLINK_SPEED_ADAPTIVE_CLOCKING		0xffff

/** Size of the device configuration data in bytes. */
#define JAYLINK_DEV_CONFIG_SIZE			256

/** Number of bytes required to store device capabilities. */
#define JAYLINK_DEV_CAPS_SIZE			4

/** Number of bytes required to store extended device capabilities. */
#define JAYLINK_DEV_EXT_CAPS_SIZE		32

/** Maximum number of connections that can be registered on a device. */
#define JAYLINK_MAX_CONNECTIONS			16

/** Media Access Control (MAC) address length in bytes. */
#define JAYLINK_MAC_ADDRESS_LENGTH		6

/**
 * Maximum length of a device's nickname including trailing null-terminator in
 * bytes.
 */
#define JAYLINK_NICKNAME_MAX_LENGTH		32

/**
 * Maximum length of a device's product name including trailing null-terminator
 * in bytes.
 */
#define JAYLINK_PRODUCT_NAME_MAX_LENGTH		32

/** Maximum length of a filename in bytes. */
#define JAYLINK_FILE_NAME_MAX_LENGTH		255

/** Maximum transfer size for a file in bytes. */
#define JAYLINK_FILE_MAX_TRANSFER_SIZE		0x100000

/**
 * EMUCOM channel with the system time of the device in milliseconds.
 *
 * The channel is read-only and the time is encoded in 4 bytes. The byte order
 * is little-endian.
 */
#define JAYLINK_EMUCOM_CHANNEL_TIME	0x0

/**
 * Offset of EMUCOM user channels.
 *
 * User channels are available to implement vendor and/or device specific
 * functionalities. All channels below are reserved.
 */
#define JAYLINK_EMUCOM_CHANNEL_USER	0x10000

/**
 * @struct jaylink_context
 *
 * Opaque structure representing a libjaylink context.
 */
struct jaylink_context;

/**
 * @struct jaylink_device
 *
 * Opaque structure representing a device.
 */
struct jaylink_device;

/**
 * @struct jaylink_device_handle
 *
 * Opaque structure representing a handle of a device.
 */
struct jaylink_device_handle;

/** Macro to mark public libjaylink API symbol. */
#ifdef _WIN32
#define JAYLINK_API
#else
#define JAYLINK_API __attribute__ ((visibility ("default")))
#endif

/**
 * Log callback function type.
 *
 * @param[in] ctx libjaylink context.
 * @param[in] level Log level.
 * @param[in] format Message format in printf()-style.
 * @param[in] args Message arguments.
 * @param[in,out] user_data User data passed to the callback function.
 *
 * @return Number of characters printed on success, or a negative error code on
 *         failure.
 */
typedef int (*jaylink_log_callback)(const struct jaylink_context *ctx,
		enum jaylink_log_level level, const char *format, va_list args,
		void *user_data);

/*--- core.c ----------------------------------------------------------------*/

JAYLINK_API int jaylink_init(struct jaylink_context **ctx);
JAYLINK_API int jaylink_exit(struct jaylink_context *ctx);
JAYLINK_API bool jaylink_library_has_cap(enum jaylink_capability cap);

/*--- device.c --------------------------------------------------------------*/

JAYLINK_API int jaylink_get_devices(struct jaylink_context *ctx,
		struct jaylink_device ***devs, size_t *count);
JAYLINK_API void jaylink_free_devices(struct jaylink_device **devs,
		bool unref);
JAYLINK_API int jaylink_device_get_host_interface(
		const struct jaylink_device *dev,
		enum jaylink_host_interface *iface);
JAYLINK_API int jaylink_device_get_serial_number(
		const struct jaylink_device *dev, uint32_t *serial_number);
JAYLINK_API int jaylink_device_get_usb_address(
		const struct jaylink_device *dev,
		enum jaylink_usb_address *address);
JAYLINK_API int jaylink_device_get_usb_bus_ports(
		const struct jaylink_device *dev, uint8_t *bus,
		uint8_t **ports, size_t *length);
JAYLINK_API int jaylink_device_get_ipv4_address(
		const struct jaylink_device *dev, char *address);
JAYLINK_API int jaylink_device_get_mac_address(
		const struct jaylink_device *dev, uint8_t *address);
JAYLINK_API int jaylink_device_get_hardware_version(
		const struct jaylink_device *dev,
		struct jaylink_hardware_version *version);
JAYLINK_API int jaylink_device_get_product_name(
		const struct jaylink_device *dev, char *name);
JAYLINK_API int jaylink_device_get_nickname(const struct jaylink_device *dev,
		char *nickname);
JAYLINK_API struct jaylink_device *jaylink_ref_device(
		struct jaylink_device *dev);
JAYLINK_API void jaylink_unref_device(struct jaylink_device *dev);
JAYLINK_API int jaylink_open(struct jaylink_device *dev,
		struct jaylink_device_handle **devh);
JAYLINK_API int jaylink_close(struct jaylink_device_handle *devh);
JAYLINK_API struct jaylink_device *jaylink_get_device(
		struct jaylink_device_handle *devh);
JAYLINK_API int jaylink_get_firmware_version(
		struct jaylink_device_handle *devh, char **version,
		size_t *length);
JAYLINK_API int jaylink_get_hardware_info(struct jaylink_device_handle *devh,
		uint32_t mask, uint32_t *info);
JAYLINK_API int jaylink_get_counters(struct jaylink_device_handle *devh,
		uint32_t mask, uint32_t *values);
JAYLINK_API int jaylink_get_hardware_version(
		struct jaylink_device_handle *devh,
		struct jaylink_hardware_version *version);
JAYLINK_API int jaylink_get_hardware_status(struct jaylink_device_handle *devh,
		struct jaylink_hardware_status *status);
JAYLINK_API int jaylink_get_caps(struct jaylink_device_handle *devh,
		uint8_t *caps);
JAYLINK_API int jaylink_get_extended_caps(struct jaylink_device_handle *devh,
		uint8_t *caps);
JAYLINK_API int jaylink_get_free_memory(struct jaylink_device_handle *devh,
		uint32_t *size);
JAYLINK_API int jaylink_read_raw_config(struct jaylink_device_handle *devh,
		uint8_t *config);
JAYLINK_API int jaylink_write_raw_config(struct jaylink_device_handle *devh,
		const uint8_t *config);
JAYLINK_API int jaylink_register(struct jaylink_device_handle *devh,
		struct jaylink_connection *connection,
		struct jaylink_connection *connections, size_t *count);
JAYLINK_API int jaylink_unregister(struct jaylink_device_handle *devh,
		const struct jaylink_connection *connection,
		struct jaylink_connection *connections, size_t *count);

/*--- discovery.c -----------------------------------------------------------*/

JAYLINK_API int jaylink_discovery_scan(struct jaylink_context *ctx,
		uint32_t ifaces);

/*--- emucom.c --------------------------------------------------------------*/

JAYLINK_API int jaylink_emucom_read(struct jaylink_device_handle *devh,
		uint32_t channel, uint8_t *buffer, uint32_t *length);
JAYLINK_API int jaylink_emucom_write(struct jaylink_device_handle *devh,
		uint32_t channel, const uint8_t *buffer, uint32_t *length);

/*--- error.c ---------------------------------------------------------------*/

JAYLINK_API const char *jaylink_strerror(int error_code);
JAYLINK_API const char *jaylink_strerror_name(int error_code);

/*--- fileio.c --------------------------------------------------------------*/

JAYLINK_API int jaylink_file_read(struct jaylink_device_handle *devh,
		const char *filename, uint8_t *buffer, uint32_t offset,
		uint32_t *length);
JAYLINK_API int jaylink_file_write(struct jaylink_device_handle *devh,
		const char *filename, const uint8_t *buffer, uint32_t offset,
		uint32_t *length);
JAYLINK_API int jaylink_file_get_size(struct jaylink_device_handle *devh,
		const char *filename, uint32_t *size);
JAYLINK_API int jaylink_file_delete(struct jaylink_device_handle *devh,
		const char *filename);

/*--- jtag.c ----------------------------------------------------------------*/

JAYLINK_API int jaylink_jtag_io(struct jaylink_device_handle *devh,
		const uint8_t *tms, const uint8_t *tdi, uint8_t *tdo,
		uint16_t length, enum jaylink_jtag_version version);
JAYLINK_API int jaylink_jtag_clear_trst(struct jaylink_device_handle *devh);
JAYLINK_API int jaylink_jtag_set_trst(struct jaylink_device_handle *devh);

/*--- log.c -----------------------------------------------------------------*/

JAYLINK_API int jaylink_log_set_level(struct jaylink_context *ctx,
		enum jaylink_log_level level);
JAYLINK_API int jaylink_log_get_level(const struct jaylink_context *ctx,
		enum jaylink_log_level *level);
JAYLINK_API int jaylink_log_set_callback(struct jaylink_context *ctx,
		jaylink_log_callback callback, void *user_data);
JAYLINK_API int jaylink_log_set_domain(struct jaylink_context *ctx,
		const char *domain);
JAYLINK_API const char *jaylink_log_get_domain(
		const struct jaylink_context *ctx);

/*--- strutil.c -------------------------------------------------------------*/

JAYLINK_API int jaylink_parse_serial_number(const char *str,
		uint32_t *serial_number);

/*--- swd.c -----------------------------------------------------------------*/

JAYLINK_API int jaylink_swd_io(struct jaylink_device_handle *devh,
		const uint8_t *direction, const uint8_t *out, uint8_t *in,
		uint16_t length);

/*--- swo.c -----------------------------------------------------------------*/

JAYLINK_API int jaylink_swo_start(struct jaylink_device_handle *devh,
		enum jaylink_swo_mode mode, uint32_t baudrate, uint32_t size);
JAYLINK_API int jaylink_swo_stop(struct jaylink_device_handle *devh);
JAYLINK_API int jaylink_swo_read(struct jaylink_device_handle *devh,
		uint8_t *buffer, uint32_t *length);
JAYLINK_API int jaylink_swo_get_speeds(struct jaylink_device_handle *devh,
		enum jaylink_swo_mode mode, struct jaylink_swo_speed *speed);

/*--- target.c --------------------------------------------------------------*/

JAYLINK_API int jaylink_set_speed(struct jaylink_device_handle *devh,
		uint16_t speed);
JAYLINK_API int jaylink_get_speeds(struct jaylink_device_handle *devh,
		struct jaylink_speed *speed);
JAYLINK_API int jaylink_select_interface(struct jaylink_device_handle *devh,
		enum jaylink_target_interface iface,
		enum jaylink_target_interface *prev_iface);
JAYLINK_API int jaylink_get_available_interfaces(
		struct jaylink_device_handle *devh, uint32_t *ifaces);
JAYLINK_API int jaylink_get_selected_interface(
		struct jaylink_device_handle *devh,
		enum jaylink_target_interface *iface);
JAYLINK_API int jaylink_clear_reset(struct jaylink_device_handle *devh);
JAYLINK_API int jaylink_set_reset(struct jaylink_device_handle *devh);
JAYLINK_API int jaylink_set_target_power(struct jaylink_device_handle *devh,
		bool enable);

/*--- util.c ----------------------------------------------------------------*/

JAYLINK_API bool jaylink_has_cap(const uint8_t *caps, uint32_t cap);

/*--- version.c -------------------------------------------------------------*/

JAYLINK_API int jaylink_version_package_get_major(void);
JAYLINK_API int jaylink_version_package_get_minor(void);
JAYLINK_API int jaylink_version_package_get_micro(void);
JAYLINK_API const char *jaylink_version_package_get_string(void);
JAYLINK_API int jaylink_version_library_get_current(void);
JAYLINK_API int jaylink_version_library_get_revision(void);
JAYLINK_API int jaylink_version_library_get_age(void);
JAYLINK_API const char *jaylink_version_library_get_string(void);

#include "version.h"

#endif /* LIBJAYLINK_LIBJAYLINK_H */
