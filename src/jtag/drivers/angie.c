// SPDX-License-Identifier: GPL-2.0-or-later
/****************************************************************************
 * File : angie.c
 * Contents : Driver code for NanoXplore USB-JTAG ANGIE
 * adapter hardware.
 * Based on FT232R driver code by: Serge Vakulenko
 *
 * Copyright 2024, Ahmed BOUDJELIDA, NanoXplore SAS.
 * <aboudjelida@nanoxplore.com>
 ****************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if IS_CYGWIN == 1
#include "windows.h"
#undef LOG_ERROR
#endif

// project specific includes
#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <helper/time_support.h>
#include "libusb_helper.h"
#include <target/image.h>
#include <libusb.h>

// system includes
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

/*
 * Sync bit bang mode is implemented as described in FTDI Application
 * Note AN232R-01: "Bit Bang Modes for the ANGIE and FT245R".
 */

/**
 * USB endpoints.
 */
#define IN_EP	0x84
#define OUT_EP	0x02

/**
 * Address of EZ-USB ANGIE CPU Control & Status register.
 * This register can be written by issuing a Control EP0 vendor request.
 */
#define CPUCS_REG		0xE600

/** USB Control EP0 bRequest: "Firmware Load". */
#define REQUEST_FIRMWARE_LOAD	0xA0

/** Value to write into CPUCS to put EZ-USB ANGIE into reset. */
#define CPU_RESET		0x01

/** Value to write into CPUCS to put EZ-USB ANGIE out of reset. */
#define CPU_START		0x00

/** Base address of firmware in EZ-USB ANGIE code space. */
#define FIRMWARE_ADDR	0x0000

/** Delay (in microseconds) to wait while EZ-USB performs ReNumeration. */
#define ANGIE_RENUMERATION_DELAY_US	800000

/** Default location of ANGIE firmware image. */
#define ANGIE_FIRMWARE_FILE	PKGDATADIR "/angie/angie_firmware.bin"

/** Default location of ANGIE firmware image. */
#define ANGIE_BITSTREAM_FILE	PKGDATADIR "/angie/angie_bitstream.bit"

/**
 * Maximum size of a single firmware section.
 * Entire EZ-USB ANGIE code space = 16kB
 */
#define ANGIE_FW_SECTION_SIZE	16384

/** Vendor Requests */
#define VR_CFGOPEN				0xB0
#define VR_DATAOUTOPEN			0xB2

#define ANGIE_VID			    0x584E /* NX Vendor id */
#define ANGIE_NPROG_PID			0x424E /* ANGIE non programmed */
#define ANGIE_PROG_OOCD_PID		0x414F /* ANGIE programmed OpenOCD */
#define ANGIE_PROG_NXB2_PID		0x4a55 /* ANGIE programmed Nxbase2 */

#define TCK_GPIO		0
#define TDI_GPIO		1
#define TDO_GPIO		2
#define TMS_GPIO		3
#define NTRST_GPIO		4
#define NSYSRST_GPIO	6

#define ANGIE_XFER_BUFFER_TOTAL_SIZE (16 * 1024)
#define ANGIE_USB_BULK_SIZE 512

/** USB timeout delay in milliseconds */
#define ANGIE_USB_TIMEOUT_MS	1000

/**
 * List of elements used in a multiple commands reply.
 */
struct read_queue {
	struct list_head list;
};

/**
 * Entry element used to forge a reply buffer for openocd JTAG core.
 */
struct read_queue_entry {
	const struct scan_command *cmd;
	int reply_buffer_offset;
	uint8_t *buffer;
	struct list_head list;
};

/**
 * Angie device main context
 */
struct angie {
	struct libusb_device_handle *usbdev;
	uint8_t xfer_buffer[ANGIE_XFER_BUFFER_TOTAL_SIZE];
	size_t xfer_buffer_len;
	uint8_t reply_buffer[ANGIE_XFER_BUFFER_TOTAL_SIZE];
	size_t  reply_buffer_len;
	struct read_queue read_queue;
};

/**
 * Angie device singleton
 */
struct angie *angie_handle;

/**
 * Init read queue list
 *
 * @param queue: pointer on the read queue head
 */
static void angie_read_queue_init(struct read_queue *queue)
{
	INIT_LIST_HEAD(&queue->list);
}


/**
 * Add a single entry to the read queue
 *
 * @param queue: read queue list
 * @param entry to append
 */
static void angie_read_queue_add(struct read_queue *queue,
								 struct read_queue_entry *entry)
{
	list_add_tail(&entry->list, &queue->list);
}

/**
 * Execute elements enqueued in the read queue list
 *
 * @param queue: read queue list
 * @param device: pointer on the angie device
 */
static void angie_read_queue_execute(struct read_queue *queue,
									 struct angie *device)
{
	struct read_queue_entry *entry;
	struct read_queue_entry *tmp;

	list_for_each_entry_safe(entry, tmp, &queue->list, list) {
		int scan_size = jtag_scan_size(entry->cmd);

		// iterate over each bit in scan data
		for (int bit_cnt = 0; bit_cnt < scan_size; bit_cnt++) {
			// calculate byte index
			int bytec = bit_cnt / 8;
			// calculate bit mask: isolate the specific bit in corresponding byte
			int bcval = 1 << (bit_cnt % 8);
			// extract tdo value using index: "bit0_index + bit_cnt*2 + 1"
			int val = device->reply_buffer[entry->reply_buffer_offset + bit_cnt * 2 + 1];
			if (val & (1 << TDO_GPIO))
				entry->buffer[bytec] |= bcval;
			else
				entry->buffer[bytec] &= ~bcval;
		}

		jtag_read_buffer(entry->buffer, entry->cmd);

		list_del(&entry->list);
		free(entry->buffer);
		free(entry);
	}
}

/**
 * Clear the read queue list
 *
 * @param queue: read queue list
 */
static void angie_read_queue_clean(struct read_queue *queue)
{
	struct read_queue_entry *entry;
	struct read_queue_entry *tmp;

	list_for_each_entry_safe(entry, tmp, &queue->list, list) {
		list_del(&entry->list);
		free(entry->buffer);
		free(entry);
	}
}

/**
 * Flush a chunk of Angie's buffer
 *
 * USB write is done by configuring GPIF register on the target and calling
 * a USB bulk transfer.
 * Sequentially a USB read transferred is issued of the same size.
 * All the operation are synchronous.
 * Then the read queue list is executed once the read buffer has been retrieved.
 *
 * @param device: Angie device pointer
 * @param xfer_size: amount of bytes to transfer
 * @param offset: total bytes already sent during this transfer, this will
 *                offset the receive buffer accordingly
 * @param bytes_sent: will contain the amount of bytes sent
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_buffer_flush_chunk(struct angie *device,
									int xfer_size,
									int offset,
									int *bytes_sent)
{
	uint8_t gpifcnt[4];
	int sent_chunk_size = 0, bytes_received = 0;

	if (bytes_sent)
		*bytes_sent = 0;

	h_u32_to_be(gpifcnt, xfer_size);

	int ret = jtag_libusb_control_transfer(device->usbdev,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		VR_DATAOUTOPEN, 0, 0, (char *)gpifcnt, sizeof(gpifcnt),
		ANGIE_USB_TIMEOUT_MS, NULL);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to send GPIF count to target");
		return ret;
	}

	ret = jtag_libusb_bulk_write(device->usbdev, OUT_EP,
							(char *)device->xfer_buffer + offset,
							xfer_size, ANGIE_USB_TIMEOUT_MS, &sent_chunk_size);
	if (ret != ERROR_OK) {
		LOG_ERROR("USB bulk transfer failed");
		return ret;
	}

	ret = jtag_libusb_bulk_read(device->usbdev, IN_EP,
						(char *)device->reply_buffer + offset,
						sent_chunk_size, ANGIE_USB_TIMEOUT_MS, &bytes_received);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to read USB reply");
		return ret;
	}

	if (sent_chunk_size == xfer_size && bytes_received == xfer_size) {
		device->reply_buffer_len += xfer_size;
		device->xfer_buffer_len -= xfer_size;
		if (bytes_sent)
			*bytes_sent += sent_chunk_size;
	} else {
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/**
 * Flush Angie transfer buffer
 *
 * Flush is done by chunks of 512 bytes to match hardware internal FIFOs.
 * Then the read queue list is executed once the read buffer has been retrieved.
 *
 * @param device: Angie device pointer
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_buffer_flush(struct angie *device)
{
	if (device->xfer_buffer_len == 0)
		return ERROR_OK;

	int total_bytes_sent = 0;
	device->reply_buffer_len = 0;

	do {
		int sent_chunk_size;
		size_t xfer_size = MIN(device->xfer_buffer_len, ANGIE_USB_BULK_SIZE);
		int ret = angie_buffer_flush_chunk(device, xfer_size,
				total_bytes_sent, &sent_chunk_size);
		if (ret != ERROR_OK)
			return ret;
		total_bytes_sent += sent_chunk_size;
	} while (device->xfer_buffer_len > 0);

	angie_read_queue_execute(&device->read_queue, device);

	return ERROR_OK;
}

/**
 * Check if transfer buffer has enough remaining space for a given size.
 * If the buffer is not large enough, flush it.
 *
 * @param device: Angie device pointer
 * @param size to check
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_buffer_flush_check(struct angie *device, size_t size)
{
	if (device->xfer_buffer_len + size >= ANGIE_XFER_BUFFER_TOTAL_SIZE)
		return angie_buffer_flush(device);
	return ERROR_OK;
}

/**
 * Append a single byte value to the transfer buffer
 *
 * @param device: Angie device pointer
 * @param value to append
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_buffer_append_simple(struct angie *device, uint8_t value)
{
	if (device->xfer_buffer_len >= ANGIE_XFER_BUFFER_TOTAL_SIZE) {
		int ret = angie_buffer_flush(device);
		if (ret != ERROR_OK)
			return ret;
	}
	device->xfer_buffer[device->xfer_buffer_len++] = value;
	return ERROR_OK;
}

/**
 * Append a bit-bang JTAG value to the transfer buffer.
 *
 * @param device: Angie device pointer
 * @param tck value
 * @param tms value
 * @param tdi value
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_buffer_append(struct angie *device, int tck, int tms, int tdi)
{
	uint8_t val = (1 << NTRST_GPIO) | (1 << NSYSRST_GPIO);
	if (tck)
		val |= (1 << TCK_GPIO);
	if (tms)
		val |= (1 << TMS_GPIO);
	if (tdi)
		val |= (1 << TDI_GPIO);

	return angie_buffer_append_simple(device, val);
}

/**
 * Open Angie USB interface
 *
 * @param device: Angie device pointer
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_usb_open(struct angie *device)
{
	uint16_t avids[] = {
		ANGIE_VID,
		ANGIE_VID,
		ANGIE_VID,
		0,
	};
	uint16_t apids[] = {
		ANGIE_NPROG_PID,
		ANGIE_PROG_OOCD_PID,
		ANGIE_PROG_NXB2_PID,
		0,
	};
	struct libusb_device_handle *usb_dev;

	int ret = jtag_libusb_open(avids, apids, NULL, &usb_dev, NULL);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to open ANGIE USB interface");
		return ret;
	}

	device->usbdev = usb_dev;

	return ERROR_OK;
}

/**
 * Releases the ANGIE interface and closes the USB device handle.
 *
 * @param device: Angie device pointer
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_usb_close(struct angie *device)
{
	int ret = ERROR_OK;

	if (device->usbdev) {
		if (libusb_release_interface(device->usbdev, 0) != LIBUSB_SUCCESS) {
			LOG_ERROR("Could not release interface 0");
			ret = ERROR_FAIL;
		}

		jtag_libusb_close(device->usbdev);
		device->usbdev = NULL;
	}

	return ret;
}

/**
 * Writes '0' or '1' to the CPUCS register, putting the EZ-USB CPU into reset
 * or out of reset.
 *
 * @param device: Angie device pointer
 * @param reset_bit 0 to put CPU into reset, 1 to put CPU out of reset.
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_cpu_reset(struct angie *device, char reset_bit)
{
	return jtag_libusb_control_transfer(device->usbdev,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		REQUEST_FIRMWARE_LOAD, CPUCS_REG, 0, &reset_bit, 1,
		ANGIE_USB_TIMEOUT_MS, NULL);
}

/**
 * Send one contiguous firmware section to the ANGIE's EZ-USB microcontroller
 * over the USB bus.
 *
 * @param device: Angie device pointer
 * @param address: address of the firmware section
 * @param data: pointer to the data to be sent
 * @param size: size of the data
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_write_firmware_section(struct angie *device, uint16_t address,
										uint8_t *data, size_t size)
{
	int bytes_remaining = size;

	// Send section data in chunks of up to 64 bytes to ANGIE
	while (bytes_remaining > 0) {
		int chunk_size;
		int transferred;

		if (bytes_remaining > 64)
			chunk_size = 64;
		else
			chunk_size = bytes_remaining;

		int ret = jtag_libusb_control_transfer(device->usbdev,
			LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
			REQUEST_FIRMWARE_LOAD, address, FIRMWARE_ADDR, (char *)data,
			chunk_size, ANGIE_USB_TIMEOUT_MS, &transferred);

		if (ret != ERROR_OK)
			return ret;

		if (transferred != chunk_size) {
			// Abort if libusb sent less data than requested
			return ERROR_FAIL;
		}

		bytes_remaining -= chunk_size;
		address += chunk_size;
		data += chunk_size;
	}

	return ERROR_OK;
}

/**
 * Downloads a firmware image to the ANGIE's EZ-USB microcontroller
 * over the USB bus.
 *
 * @param device: Angie device pointer
 * @param filename: an absolute or relative path to the Intel HEX file
 *  containing the firmware image.
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_load_firmware(struct angie *device, const char *filename)
{
	struct image angie_firmware_image;

	int ret = angie_cpu_reset(device, CPU_RESET);
	if (ret != ERROR_OK) {
		LOG_ERROR("Could not halt ANGIE CPU");
		return ret;
	}

	angie_firmware_image.base_address = 0;
	angie_firmware_image.base_address_set = false;

	ret = image_open(&angie_firmware_image, filename, "bin");
	if (ret != ERROR_OK) {
		LOG_ERROR("Could not load firmware image");
		return ret;
	}

	uint8_t *data = malloc(ANGIE_FW_SECTION_SIZE);
	if (!data) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	// Download all sections in the image to ANGIE
	for (unsigned int i = 0; i < angie_firmware_image.num_sections; i++) {
		size_t size_read;
		uint32_t size = angie_firmware_image.sections[i].size;
		int addr = angie_firmware_image.sections[i].base_address;

		LOG_DEBUG("section %02i at addr 0x%04x (size 0x%04" PRIx32 ")",
			i, addr, size);

		ret = image_read_section(&angie_firmware_image, i, 0,
						size, data, &size_read);
		if (ret != ERROR_OK)
			goto exit;
		if (size_read != size) {
			ret = ERROR_FAIL;
			goto exit;
		}

		ret = angie_write_firmware_section(device, addr, data, size);
		if (ret != ERROR_OK) {
			LOG_ERROR("Could not write firmware section");
			goto exit;
		}
	}

	image_close(&angie_firmware_image);

	ret = angie_cpu_reset(device, CPU_START);
	if (ret != ERROR_OK) {
		LOG_ERROR("Could not restart ANGIE CPU");
		goto exit;
	}

exit:
	free(data);
	return ret;
}

/**
 * Puts the ANGIE's EZ-USB microcontroller into reset state, downloads
 * the firmware image, resumes the microcontroller and re-enumerates
 * USB devices.
 *
 * @param device: Angie device pointer
 *  The usb_handle member will be modified during re-enumeration.
 * @param filename: path to the Intel HEX file containing the firmware image.
 * @param delay_us: the delay to wait for the device to re-enumerate.
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_load_firmware_and_renumerate(struct angie *device,
											  const char *filename,
											  uint32_t delay_us)
{
	/*
	 * Basic process: After downloading the firmware, the ANGIE will disconnect
	 * itself and re-connect after a short amount of time so we have to close
	 * the handle and re-enumerate USB devices.
	 */

	int ret = angie_load_firmware(device, filename);
	if (ret != ERROR_OK)
		return ret;

	ret = angie_usb_close(device);
	if (ret != ERROR_OK)
		return ret;

	usleep(delay_us);

	ret = angie_usb_open(device);
	if (ret != ERROR_OK)
		return ret;

	if (libusb_claim_interface(angie_handle->usbdev, 0) != LIBUSB_SUCCESS)
		return ERROR_FAIL;

	return ERROR_OK;
}

/**
 * Downloads a bitstream file to the ANGIE's FPGA through the EZ-USB microcontroller
 * over the USB bus.
 *
 * @param device: Angie device pointer
 * @param filename: an absolute or relative path to the Xilinx .bit file
 *  containing the bitstream data.
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_load_bitstream(struct angie *device, const char *filename)
{
	int ret = ERROR_OK, transferred;
	const char *bitstream_file_path = filename;
	FILE *bitstream_file = NULL;
	char *bitstream_data = NULL;
	uint8_t gpifcnt[4];

	// Open the bitstream file
	bitstream_file = fopen(bitstream_file_path, "rb");
	if (!bitstream_file) {
		LOG_ERROR("Failed to open bitstream file: %s\n", bitstream_file_path);
		ret = ERROR_FAIL;
		goto exit;
	}

	// Get the size of the bitstream file
	fseek(bitstream_file, 0, SEEK_END);
	size_t bitstream_size = ftell(bitstream_file);
	fseek(bitstream_file, 0, SEEK_SET);

	// Allocate memory for the bitstream data
	bitstream_data = malloc(bitstream_size);
	if (!bitstream_data) {
		LOG_ERROR("Failed to allocate memory for bitstream data.");
		ret = ERROR_FAIL;
		goto exit;
	}

	// Read the bitstream data from the file
	if (fread(bitstream_data, 1, bitstream_size, bitstream_file) != bitstream_size) {
		LOG_ERROR("Failed to read bitstream data.");
		ret = ERROR_FAIL;
		goto exit;
	}

	// CFG Open
	h_u32_to_be(gpifcnt, bitstream_size);
	ret = jtag_libusb_control_transfer(device->usbdev,
		LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		VR_CFGOPEN, 0, 0, (char *)gpifcnt, sizeof(gpifcnt),
		ANGIE_USB_TIMEOUT_MS, &transferred);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed opencfg");
		goto exit;
	}

	// Send the bitstream data to the microcontroller
	int actual_length = 0;
	ret = jtag_libusb_bulk_write(device->usbdev, OUT_EP, bitstream_data,
			bitstream_size, ANGIE_USB_TIMEOUT_MS, &actual_length);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to send bitstream data: %s", libusb_strerror(ret));
		goto exit;
	}

	LOG_INFO("Bitstream sent successfully.");

exit:
	free(bitstream_data);
	if (bitstream_file)
		fclose(bitstream_file);

	return ret;
}

/**
 * Check if Angie firmware must be updated
 *
 * @param device: Angie device pointer
 * @return true if update is needed, false otherwise
 */
static bool angie_is_firmware_needed(struct angie *device)
{
	struct libusb_device_descriptor desc;

	// Get String Descriptor to determine if firmware needs to be loaded
	int ret = libusb_get_device_descriptor(libusb_get_device(angie_handle->usbdev),
										&desc);
	if (ret != LIBUSB_SUCCESS)
		// Could not get descriptor -> Unconfigured or original Keil firmware
		return true;
	else if (desc.idProduct != ANGIE_PROG_OOCD_PID)
		return true;

	return false;
}

/**
 * Set TAP end state
 *
 * @param state
 */
static void angie_set_end_state(enum tap_state state)
{
	if (tap_is_state_stable(state)) {
		tap_set_end_state(state);
	} else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

/**
 * Move TAP to given state
 *
 * @param device: Angie device pointer
 * @param skip: number of state to skip during move
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_state_move(struct angie *device, int skip)
{
	int ret;
	int tms = 0;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	// tms_scan has 8 bits that we bitbang one by one
	for (int i = skip; i < tms_count; i++) {
		tms = (tms_scan >> i) & 1;
		ret = angie_buffer_append(device, 0, tms, 0);
		if (ret != ERROR_OK)
			return ret;
		ret = angie_buffer_append(device, 1, tms, 0);
		if (ret != ERROR_OK)
			return ret;
	}
	ret = angie_buffer_append(device, 0, tms, 0);
	if (ret != ERROR_OK)
		return ret;

	tap_set_state(tap_get_end_state());

	return ERROR_OK;
}

/**
 * Return JTAG SCAN command size in bytes
 *
 * @param device: Angie device pointer
 * @param cmd: SCAN command
 * @return size of command in the transfer buffer in bytes
 */
static int angie_jtag_scan_size(struct angie *device,
								const struct scan_command *cmd)
{
	int cmd_size = 0;
	int count = 0;

	// move to TAP_IRSHIFT or TAP_DRSHIFT state
	if (cmd->ir_scan)
		count = tap_get_tms_path_len(tap_get_state(), TAP_IRSHIFT);
	else
		count = tap_get_tms_path_len(tap_get_state(), TAP_DRSHIFT);
	cmd_size += count * 2 + 1;

	// add scan size
	cmd_size += jtag_scan_size(cmd) * 2;

	/*
	 * move to cmd specified end state
	 * Also, see below function
	 * we *KNOW* the above loop transitioned out of
	 * the shift state, so we skip the first state
	 * and move directly to the end state.
	 */
	if (cmd->ir_scan)
		count = tap_get_tms_path_len(TAP_IRSHIFT, cmd->end_state) - 1;
	else
		count = tap_get_tms_path_len(TAP_DRSHIFT, cmd->end_state) - 1;
	cmd_size += count * 2 + 1;

	return cmd_size;
}

/**
 * Execute JTAG SCAN command
 *
 * @param device: Angie device pointer
 * @param cmd: SCAN command
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_jtag_execute_scan(struct angie *device,
								   const struct scan_command *cmd)
{
	uint8_t *buffer = NULL;
	LOG_DEBUG_IO("SCAN: size=%d %s scan end in %s", jtag_scan_size(cmd),
			(cmd->ir_scan) ? "IR" : "DR", tap_state_name(cmd->end_state));

	if (cmd->ir_scan) {
		if (tap_get_state() != TAP_IRSHIFT)
			angie_set_end_state(TAP_IRSHIFT);
	} else {
		if (tap_get_state() != TAP_DRSHIFT)
			angie_set_end_state(TAP_DRSHIFT);
	}
	int ret = angie_state_move(device, 0);
	if (ret != ERROR_OK)
		return ret;
	angie_set_end_state(cmd->end_state);

	// Execute scan
	int scan_size = jtag_build_buffer(cmd, &buffer);
	enum scan_type type = jtag_scan_type(cmd);

	// starting byte index
	int start_offset = device->xfer_buffer_len;

	// iterate over each bit in all scan data
	int tms = 0;
	int tdi = 0;
	for (int i = 0; i < scan_size; i++) {
		// calculate tms
		// if we finish shifting tdi bits : '1' , else '0'
		tms = (i == scan_size - 1) ? 1 : 0;
		// calculate byte index
		int bytec = i / 8;
		// calculate bit mask: isolate the specific bit in corresponding byte
		int bcval = 1 << (i % 8);
		// if type is not SCAN_IN (not just reading data)
		// and the bit masked is '1' then tdi = '1'
		tdi = 0;
		if (type != SCAN_IN && (buffer[bytec] & bcval))
			tdi = 1;
		// write tdi and tms twice in tck=0 and tck=1
		ret = angie_buffer_append(device, 0, tms, tdi);
		if (ret != ERROR_OK)
			return ret;
		ret = angie_buffer_append(device, 1, tms, tdi);
		if (ret != ERROR_OK)
			return ret;
	}

	angie_set_end_state(cmd->end_state);
	if (tap_get_state() != tap_get_end_state()) {
		/*
		 * We *KNOW* the above loop transitioned out of
		 * the shift state, so we skip the first state
		 * and move directly to the end state.
		 */
		ret = angie_state_move(device, 1);
		if (ret != ERROR_OK)
			return ret;
	}

	if (jtag_scan_type(cmd) != SCAN_OUT) {
		// queue read back buffer for further processing
		struct read_queue_entry *entry = malloc(sizeof(*entry));
		if (!entry) {
			LOG_ERROR("Out of memory");
			free(buffer);
			return ERROR_FAIL;
		}

		entry->reply_buffer_offset = start_offset;
		entry->cmd = cmd;
		entry->buffer = buffer;
		angie_read_queue_add(&device->read_queue, entry);
	} else {
		// built buffer won't be of later use
		free(buffer);
	}

	return ERROR_OK;
}

/**
 * Return JTAG RUNTEST command size in bytes
 *
 * @param device: Angie device pointer
 * @param cmd: RUNTEST command
 * @return size of command in the transfer buffer in bytes
 */
static int angie_jtag_runtest_size(struct angie *device,
								   const struct runtest_command *cmd)
{
	int cmd_size = 0;

	if (tap_get_state() != TAP_IDLE)
		cmd_size += tap_get_tms_path_len(tap_get_state(), TAP_IDLE) * 2 + 1;
	cmd_size += cmd->num_cycles * 2 + 1;
	if (tap_get_end_state() != TAP_IDLE)
		cmd_size += tap_get_tms_path_len(TAP_IDLE, tap_get_end_state()) * 2 + 1;

	return cmd_size;
}

/**
 * Execute JTAG RUNTEST command
 *
 * @param device: Angie device pointer
 * @param cmd: SCAN command
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_jtag_execute_runtest(struct angie *device,
									  const struct runtest_command *cmd)
{
	int ret;
	enum tap_state saved_end_state = tap_get_end_state();

	LOG_DEBUG_IO("RUNTEST: %d cycles", cmd->num_cycles);

	// only do a state_move when we're not already in IDLE
	if (tap_get_state() != TAP_IDLE) {
		angie_set_end_state(TAP_IDLE);
		ret = angie_state_move(device, 0);
		if (ret != ERROR_OK)
			return ret;
	}

	// execute num_cycles
	for (unsigned int i = 0; i < cmd->num_cycles; i++) {
		ret = angie_buffer_append(device, 0, 0, 0);
		if (ret != ERROR_OK)
			return ret;
		ret = angie_buffer_append(device, 1, 0, 0);
		if (ret != ERROR_OK)
			return ret;
	}
	ret = angie_buffer_append(device, 0, 0, 0);
	if (ret != ERROR_OK)
		return ret;

	// finish in end_state
	angie_set_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state()) {
		ret = angie_state_move(device, 0);
		if (ret != ERROR_OK)
			return ret;
	}

	return ERROR_OK;
}

/**
 * Execute JTAG TMS command
 * Clock a bunch of TMS transitions, to change the JTAG
 * state machine.
 *
 * @param device: Angie device pointer
 * @param cmd: TMS command
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_jtag_execute_tms(struct angie *device,
								  const struct tms_command *cmd)
{
	unsigned int num_bits = cmd->num_bits;
	const uint8_t *bits = cmd->bits;

	LOG_DEBUG_IO("TMS: %d bits", num_bits);

	int tms = 0;
	for (unsigned int i = 0; i < num_bits; i++) {
		tms = ((bits[i / 8] >> (i % 8)) & 1);
		int ret = angie_buffer_append(device, 0, tms, 0);
		if (ret != ERROR_OK)
			return ret;
		ret = angie_buffer_append(device, 1, tms, 0);
		if (ret != ERROR_OK)
			return ret;
	}

	return angie_buffer_append(device, 0, tms, 0);
}

/**
 * Execute JTAG RESET command
 * Control /TRST and /SYSRST pins.
 * Perform immediate bitbang transaction.
 *
 * @param device: Angie device pointer
 * @param cmd: RESET command
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_jtag_execute_reset(struct angie *device,
									const struct reset_command *cmd)
{
	LOG_DEBUG_IO("RESET: trst %i srst %i", cmd->trst, cmd->srst);

	uint8_t out_value = (1 << NTRST_GPIO) | (1 << NSYSRST_GPIO);
	if (cmd->trst == 1 ||
		(cmd->srst && (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
		tap_set_state(TAP_RESET);

	if (cmd->trst == 1)
		out_value &= ~(1 << NTRST_GPIO);		// switch /TRST low
	else if (cmd->trst == 0)
		out_value |= (1 << NTRST_GPIO);			// switch /TRST high

	if (cmd->srst == 1)
		out_value &= ~(1 << NSYSRST_GPIO);		// switch /SYSRST low
	else if (cmd->srst == 0)
		out_value |= (1 << NSYSRST_GPIO);		// switch /SYSRST high

	return angie_buffer_append_simple(device, out_value);
}

/**
 * Execute JTAG STABLECLOCKS command
 * Issues a number of clock cycles while staying in a stable state.
 * Because the TMS value required to stay in the RESET state is a 1, whereas
 * the TMS value required to stay in any of the other stable states is a 0,
 * this function checks the current stable state to decide on the value of TMS
 * to use.
 *
 * @param device: Angie device pointer
 * @param cmd: STABLECLOCKS command
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_jtag_execute_stableclocks(struct angie *device,
										   const struct stableclocks_command *cmd)
{
	int tms = (tap_get_state() == TAP_RESET ? 1 : 0);

	// send num_cycles clocks onto the cable
	for (unsigned int i = 0; i < cmd->num_cycles; i++) {
		int ret = angie_buffer_append(device, 1, tms, 0);
		if (ret != ERROR_OK)
			return ret;
		ret = angie_buffer_append(device, 0, tms, 0);
		if (ret != ERROR_OK)
			return ret;
	}

	LOG_DEBUG_IO("clocks %i while in %s", cmd->num_cycles,
		tap_state_name(tap_get_state()));

	return ERROR_OK;
}

/**
 * Execute JTAG STATEMOVE command
 *
 * @param device: Angie device pointer
 * @param cmd: STATEMOVE command
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_jtag_execute_statemove(struct angie *device,
										const struct statemove_command *cmd)
{
	LOG_DEBUG_IO("statemove end in %s", tap_state_name(cmd->end_state));
	angie_set_end_state(cmd->end_state);
	return angie_state_move(device, 0);
}

/**
 * Execute JTAG PATHMOVE command
 *
 * @param device: Angie device pointer
 * @param cmd: PATHMOVE command
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_jtag_execute_pathmove(struct angie *device,
									   const struct pathmove_command *cmd)
{
	int ret;
	int num_states = cmd->num_states;
	int tms = 0;

	LOG_DEBUG_IO("pathmove: %i states, end in %s", cmd->num_states,
			  tap_state_name(cmd->path[cmd->num_states - 1]));

	int state_count = 0;
	while (num_states) {
		if (tap_state_transition(tap_get_state(), false) == cmd->path[state_count]) {
			tms = 0;
		} else if (tap_state_transition(tap_get_state(), true) == cmd->path[state_count]) {
			tms = 1;
		} else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
			 tap_state_name(tap_get_state()),
			 tap_state_name(cmd->path[state_count]));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		ret = angie_buffer_append(device, 0, tms, 0);
		if (ret != ERROR_OK)
			return ret;
		ret = angie_buffer_append(device, 1, tms, 0);
		if (ret != ERROR_OK)
			return ret;

		tap_set_state(cmd->path[state_count]);
		state_count++;
		num_states--;
	}
	ret = angie_buffer_append(device, 0, tms, 0);
	if (ret != ERROR_OK)
		return ret;

	tap_set_end_state(tap_get_state());

	return ERROR_OK;
}

/**
 * Process command size in bytes
 *
 * @param device: Angie device pointer
 * @param cmd: JTAG command
 * @return command size in the transfer buffer in bytes
 */
static size_t angie_cmd_size(struct angie *device, const struct jtag_command *cmd)
{
	switch (cmd->type) {
	case JTAG_SCAN:
		return angie_jtag_scan_size(device, cmd->cmd.scan);
	case JTAG_TMS:
		return cmd->cmd.tms->num_bits + 2 + 1;
	case JTAG_RESET:
		return 1;
	case JTAG_RUNTEST:
		return angie_jtag_runtest_size(device, cmd->cmd.runtest);
	case JTAG_STABLECLOCKS:
		return cmd->cmd.stableclocks->num_cycles * 2;
	case JTAG_TLR_RESET: // renamed from JTAG_STATEMOVE
		return tap_get_tms_path_len(tap_get_state(),
							cmd->cmd.statemove->end_state) * 2 + 1;
	case JTAG_PATHMOVE:
		return cmd->cmd.pathmove->num_states * 2 + 1;
	case JTAG_SLEEP:
		LOG_DEBUG_IO("sleep %" PRIu32, cmd->cmd.sleep->us);
		return 0;
	default:
		LOG_ERROR("BUG: unknown JTAG command type encountered");
		return 0;
	}
}

/**
 * Execute JTAG commands queue
 *
 * @param cmd_queue to execute
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_jtag_execute_queue(struct jtag_command *cmd_queue)
{
	int retval = ERROR_OK;
	struct jtag_command *cmd = cmd_queue;
	struct angie *device = angie_handle;

	while (cmd) {
		retval = angie_buffer_flush_check(device, angie_cmd_size(device, cmd));
		if (retval != ERROR_OK)
			return retval;

		switch (cmd->type) {
		case JTAG_SCAN:
			retval = angie_jtag_execute_scan(device, cmd->cmd.scan);
			if (retval != ERROR_OK)
				return retval;
			break;
		case JTAG_TMS:
			retval = angie_jtag_execute_tms(device, cmd->cmd.tms);
			if (retval != ERROR_OK)
				return retval;
			retval = angie_buffer_flush(device);
			if (retval != ERROR_OK)
				return retval;
			break;
		case JTAG_RESET:
			angie_jtag_execute_reset(device, cmd->cmd.reset);
			retval = angie_buffer_flush(device);
			if (retval != ERROR_OK)
				return retval;
			break;
		case JTAG_RUNTEST:
			retval = angie_jtag_execute_runtest(device, cmd->cmd.runtest);
			if (retval != ERROR_OK)
				return retval;
			break;
		case JTAG_STABLECLOCKS:
			/* this is only allowed while in a stable state.  A check for a stable
			 * state was done in jtag_add_clocks()
			 */
			retval = angie_jtag_execute_stableclocks(device, cmd->cmd.stableclocks);
			if (retval != ERROR_OK)
				return retval;
			retval = angie_buffer_flush(device);
			if (retval != ERROR_OK)
				return retval;
			break;
		case JTAG_TLR_RESET: // renamed from JTAG_STATEMOVE
			retval = angie_jtag_execute_statemove(device, cmd->cmd.statemove);
			if (retval != ERROR_OK)
				return retval;
			retval = angie_buffer_flush(device);
			if (retval != ERROR_OK)
				return retval;
			break;
		case JTAG_PATHMOVE:
			retval = angie_jtag_execute_pathmove(device, cmd->cmd.pathmove);
			if (retval != ERROR_OK)
				return retval;
			retval = angie_buffer_flush(device);
			if (retval != ERROR_OK)
				return retval;
			break;
		case JTAG_SLEEP:
			LOG_DEBUG_IO("sleep %" PRIu32, cmd->cmd.sleep->us);
			jtag_sleep(cmd->cmd.sleep->us);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type encountered");
			break;
		}

		cmd = cmd->next;
	}

	return angie_buffer_flush(device);
}

/**
 * Angie quit method
 *
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_quit(void)
{
	if (!angie_handle)
		return ERROR_OK;

	int ret = angie_usb_close(angie_handle);
	if (ret != ERROR_OK)
		return ret;

	angie_read_queue_clean(&angie_handle->read_queue);

	free(angie_handle);
	angie_handle = NULL;

	return ERROR_OK;
}

/**
 * Angie initialization method
 *
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_init(void)
{
	int ret;

	angie_handle = calloc(1, sizeof(*angie_handle));
	if (!angie_handle) {
		ret = ERROR_FAIL;
		goto exit;
	}

	angie_read_queue_init(&angie_handle->read_queue);

	ret = angie_usb_open(angie_handle);
	if (ret != ERROR_OK)
		goto exit;

	if (angie_is_firmware_needed(angie_handle)) {
		LOG_INFO("Loading ANGIE firmware. This is reversible by power-cycling ANGIE device.");

		ret = libusb_claim_interface(angie_handle->usbdev, 0);
		if (ret != LIBUSB_SUCCESS) {
			LOG_ERROR("Failed to claim interface 0");
			ret = ERROR_FAIL;
			goto exit;
		}

		ret = angie_load_firmware_and_renumerate(angie_handle,
										   ANGIE_FIRMWARE_FILE,
										   ANGIE_RENUMERATION_DELAY_US);
		if (ret != ERROR_OK) {
			LOG_ERROR("Could not download firmware and re-numerate ANGIE");
			angie_quit();
			return ret;
		}

		ret = angie_load_bitstream(angie_handle, ANGIE_BITSTREAM_FILE);
		if (ret != ERROR_OK) {
			LOG_ERROR("Could not download bitstream");
			angie_quit();
			return ret;
		}
	} else {
		LOG_INFO("ANGIE device is already running ANGIE firmware");
	}

	return ERROR_OK;
exit:
	angie_quit();
	return ret;
}

/**
 * Angie set speed method
 *
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_speed(int divisor)
{
	int baud = (divisor == 0) ? 3000000 :
		(divisor == 1) ? 2000000 :
		3000000 / divisor;
	LOG_DEBUG("angie speed(%d) rate %d bits/sec", divisor, baud);

	return ERROR_OK;
}

/**
 * Angie set khz method
 *
 * @param khz
 * @param divisor returned to caller
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_khz(int khz, int *divisor)
{
	if (khz == 0) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}

	// Calculate frequency divisor.
	if (khz > 2500) {
		*divisor = 0;		// Special case: 3 MHz
	} else if (khz > 1700) {
		*divisor = 1;		// Special case: 2 MHz
	} else {
		*divisor = (2 * 3000 / khz + 1) / 2;
		if (*divisor > 0x3FFF)
			*divisor = 0x3FFF;
	}
	return ERROR_OK;
}

/**
 * Angie set speed div
 *
 * @param divisor
 * @param khz returned to caller
 * @return ERROR_OK on success, negative error code otherwise
 */
static int angie_speed_div(int divisor, int *khz)
{
	// Maximum 3 Mbaud for bit bang mode
	if (divisor == 0)
		*khz = 30000;
	else if (divisor == 1)
		*khz = 20000;
	else
		*khz = 30000 / divisor;
	return ERROR_OK;
}

static struct jtag_interface angie_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = angie_jtag_execute_queue,
};

struct adapter_driver angie_adapter_driver = {
	.name = "angie",
	.transport_ids = TRANSPORT_JTAG,
	.transport_preferred_id = TRANSPORT_JTAG,

	.init = angie_init,
	.quit = angie_quit,
	.speed = angie_speed,
	.khz = angie_khz,
	.speed_div = angie_speed_div,

	.jtag_ops = &angie_interface,
};
