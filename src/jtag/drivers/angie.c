// SPDX-License-Identifier: GPL-2.0-or-later
/***************************************************************************
	File : angie.c															*
	Contents : OpenOCD driver code for NanoXplore USB-JTAG ANGIE			*
	adapter hardware.														*
	Based on openULINK driver code by: Martin Schmoelzer.					*
	Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.				*
	<aboudjelida@nanoxplore.com>											*
	<ahmederrachedbjld@gmail.com>											*
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "helper/system.h"
#include <helper/types.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <target/image.h>
#include <libusb.h>
#include "libusb_helper.h"
#include "angie/include/msgtypes.h"

/** USB Vendor ID of ANGIE device in unconfigured state (no firmware loaded
 *  yet) or with its firmware. */
#define ANGIE_VID				0x584e

/** USB Product ID of ANGIE device in unconfigured state (no firmware loaded
 *  yet) or with its firmware. */
#define ANGIE_PID 0x414F
#define ANGIE_PID_2 0x424e
#define ANGIE_PID_3 0x4255
#define ANGIE_PID_4 0x4355
#define ANGIE_PID_5 0x4a55

/** Address of EZ-USB ANGIE CPU Control & Status register. This register can be
 *  written by issuing a Control EP0 vendor request. */
#define CPUCS_REG				0xE600

/** USB Control EP0 bRequest: "Firmware Load". */
#define REQUEST_FIRMWARE_LOAD	0xA0

/** Value to write into CPUCS to put EZ-USB ANGIE into reset. */
#define CPU_RESET				0x01

/** Value to write into CPUCS to put EZ-USB ANGIE out of reset. */
#define CPU_START				0x00

/** Base address of firmware in EZ-USB ANGIE code space. */
#define FIRMWARE_ADDR			0x0000

/** USB interface number */
#define USB_INTERFACE			0

/** Delay (in microseconds) to wait while EZ-USB performs ReNumeration. */
#define ANGIE_RENUMERATION_DELAY_US	1500000

/** Default location of ANGIE firmware image. */
#define ANGIE_FIRMWARE_FILE		PKGDATADIR "/angie/angie_firmware.bin"

/** Default location of ANGIE firmware image. */
#define ANGIE_BITSTREAM_FILE		PKGDATADIR "/angie/angie_bitstream.bit"

/** Maximum size of a single firmware section. Entire EZ-USB ANGIE code space = 16kB */
#define SECTION_BUFFERSIZE		16384

/** Tuning of OpenOCD SCAN commands split into multiple ANGIE commands. */
#define SPLIT_SCAN_THRESHOLD	10

/** ANGIE hardware type */
enum angie_type {
	ANGIE,
};

enum angie_payload_direction {
	PAYLOAD_DIRECTION_OUT,
	PAYLOAD_DIRECTION_IN
};

enum angie_delay_type {
	DELAY_CLOCK_TCK,
	DELAY_CLOCK_TMS,
	DELAY_SCAN_IN,
	DELAY_SCAN_OUT,
	DELAY_SCAN_IO
};

/**
 * ANGIE command (ANGIE command queue element).
 *
 * For the OUT direction payload, things are quite easy: Payload is stored
 * in a rather small array (up to 63 bytes), the payload is always allocated
 * by the function generating the command and freed by angie_clear_queue().
 *
 * For the IN direction payload, things get a little bit more complicated:
 * The maximum IN payload size for a single command is 64 bytes. Assume that
 * a single OpenOCD command needs to scan 256 bytes. This results in the
 * generation of four ANGIE commands. The function generating these
 * commands shall allocate an uint8_t[256] array. Each command's #payload_in
 * pointer shall point to the corresponding offset where IN data shall be
 * placed, while #payload_in_start shall point to the first element of the 256
 * byte array.
 * - first command:  #payload_in_start + 0
 * - second command: #payload_in_start + 64
 * - third command:  #payload_in_start + 128
 * - fourth command: #payload_in_start + 192
 *
 * The last command sets #needs_postprocessing to true.
 */
struct angie_cmd {
	uint8_t id;			/**< ANGIE command ID */

	uint8_t *payload_out;		/**< Pointer where OUT payload shall be stored */
	uint8_t payload_out_size;	/**< OUT direction payload size for this command */

	uint8_t *payload_in_start;	/**< Pointer to first element of IN payload array */
	uint8_t *payload_in;		/**< Pointer where IN payload shall be stored */
	uint8_t payload_in_size;	/**< IN direction payload size for this command */

	/** Indicates if this command needs post-processing */
	bool needs_postprocessing;

	/** Indicates if angie_clear_queue() should free payload_in_start  */
	bool free_payload_in_start;

	/** Pointer to corresponding OpenOCD command for post-processing */
	struct jtag_command *cmd_origin;

	struct angie_cmd *next;		/**< Pointer to next command (linked list) */
};

/** Describes one driver instance */
struct angie {
	struct libusb_context *libusb_ctx;
	struct libusb_device_handle *usb_device_handle;
	enum angie_type type;

	unsigned int ep_in;		/**< IN endpoint number */
	unsigned int ep_out;		/**< OUT endpoint number */

	/* delay value for "SLOW_CLOCK commands" in [0:255] range in units of 4 us;
		-1 means no need for delay */
	int delay_scan_in;	/**< Delay value for SCAN_IN commands */
	int delay_scan_out;	/**< Delay value for SCAN_OUT commands */
	int delay_scan_io;	/**< Delay value for SCAN_IO commands */
	int delay_clock_tck;	/**< Delay value for CLOCK_TMS commands */
	int delay_clock_tms;	/**< Delay value for CLOCK_TCK commands */

	int commands_in_queue;		/**< Number of commands in queue */
	struct angie_cmd *queue_start;	/**< Pointer to first command in queue */
	struct angie_cmd *queue_end;	/**< Pointer to last command in queue */
};

/**************************** Function Prototypes *****************************/

/* USB helper functions */
static int angie_usb_open(struct angie *device);
static int angie_usb_close(struct angie *device);

/* ANGIE MCU (Cypress EZ-USB) specific functions */
static int angie_cpu_reset(struct angie *device, char reset_bit);
static int angie_load_firmware_and_renumerate(struct angie *device, const char *filename,
		uint32_t delay_us);
static int angie_load_firmware(struct angie *device, const char *filename);
static int angie_load_bitstream(struct angie *device, const char *filename);
static int angie_i2c_write(struct angie *device, uint8_t *i2c_data, uint8_t i2c_data_size);
static int angie_io_extender_config(struct angie *device, uint8_t i2c_adr, uint8_t cfg_value);
static int angie_write_firmware_section(struct angie *device,
		struct image *firmware_image, int section_index);

/* Generic helper functions */
static void angie_dump_signal_states(uint8_t input_signals, uint8_t output_signals);

/* ANGIE command generation helper functions */
static int angie_allocate_payload(struct angie_cmd *angie_cmd, int size,
		enum angie_payload_direction direction);

/* ANGIE command queue helper functions */
static int angie_get_queue_size(struct angie *device,
		enum angie_payload_direction direction);
static void angie_clear_queue(struct angie *device);
static int angie_append_queue(struct angie *device, struct angie_cmd *angie_cmd);
static int angie_execute_queued_commands(struct angie *device, int timeout_ms);

static void angie_dump_queue(struct angie *device);

static int angie_append_scan_cmd(struct angie *device,
		enum scan_type scan_type,
		int scan_size_bits,
		uint8_t *tdi,
		uint8_t *tdo_start,
		uint8_t *tdo,
		uint8_t tms_count_start,
		uint8_t tms_sequence_start,
		uint8_t tms_count_end,
		uint8_t tms_sequence_end,
		struct jtag_command *origin,
		bool postprocess);
static int angie_append_clock_tms_cmd(struct angie *device, uint8_t count,
		uint8_t sequence);
static int angie_append_clock_tck_cmd(struct angie *device, uint16_t count);
static int angie_append_get_signals_cmd(struct angie *device);
static int angie_append_set_signals_cmd(struct angie *device, uint8_t low,
		uint8_t high);
static int angie_append_sleep_cmd(struct angie *device, uint32_t us);
static int angie_append_configure_tck_cmd(struct angie *device,
		int delay_scan_in,
		int delay_scan_out,
		int delay_scan_io,
		int delay_tck,
		int delay_tms);
static int angie_append_test_cmd(struct angie *device);

/* ANGIE TCK frequency helper functions */
static int angie_calculate_delay(enum angie_delay_type type, long f, int *delay);

/* Interface between ANGIE and OpenOCD */
static void angie_set_end_state(enum tap_state endstate);
static int angie_queue_statemove(struct angie *device);

static int angie_queue_scan(struct angie *device, struct jtag_command *cmd);
static int angie_queue_tlr_reset(struct angie *device, struct jtag_command *cmd);
static int angie_queue_runtest(struct angie *device, struct jtag_command *cmd);
static int angie_queue_pathmove(struct angie *device, struct jtag_command *cmd);
static int angie_queue_sleep(struct angie *device, struct jtag_command *cmd);
static int angie_queue_stableclocks(struct angie *device, struct jtag_command *cmd);

static int angie_post_process_scan(struct angie_cmd *angie_cmd);
static int angie_post_process_queue(struct angie *device);

/* adapter driver functions */
static int angie_execute_queue(struct jtag_command *cmd_queue);
static int angie_khz(int khz, int *jtag_speed);
static int angie_speed(int speed);
static int angie_speed_div(int speed, int *khz);
static int angie_init(void);
static int angie_quit(void);
static int angie_reset(int trst, int srst);

/****************************** Global Variables ******************************/

static struct angie *angie_handle;

/**************************** USB helper functions ****************************/

/**
 * Opens the ANGIE device
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_usb_open(struct angie *device)
{
	struct libusb_device_handle *usb_device_handle;
	const uint16_t vids[] = {ANGIE_VID, ANGIE_VID, ANGIE_VID, ANGIE_VID, ANGIE_VID, 0};
	const uint16_t pids[] = {ANGIE_PID, ANGIE_PID_2, ANGIE_PID_3, ANGIE_PID_4, ANGIE_PID_5, 0};

	int ret = jtag_libusb_open(vids, pids, NULL, &usb_device_handle, NULL);

	if (ret != ERROR_OK) {
		LOG_ERROR("Could not find and open ANGIE");
		return ret;
	}

	device->usb_device_handle = usb_device_handle;
	device->type = ANGIE;

	return ERROR_OK;
}

/**
 * Releases the ANGIE interface and closes the USB device handle.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_usb_close(struct angie *device)
{
	if (device->usb_device_handle) {
		if (libusb_release_interface(device->usb_device_handle, 0) != 0) {
			LOG_ERROR("Could not release interface 0");
			return ERROR_FAIL;
		}

		jtag_libusb_close(device->usb_device_handle);
		device->usb_device_handle = NULL;
	}
	return ERROR_OK;
}

/******************* ANGIE CPU (EZ-USB) specific functions ********************/

/**
 * Writes '0' or '1' to the CPUCS register, putting the EZ-USB CPU into reset
 * or out of reset.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param reset_bit 0 to put CPU into reset, 1 to put CPU out of reset.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_cpu_reset(struct angie *device, char reset_bit)
{
	return jtag_libusb_control_transfer(device->usb_device_handle,
			(LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE),
			REQUEST_FIRMWARE_LOAD, CPUCS_REG, 0, &reset_bit, 1, LIBUSB_TIMEOUT_MS, NULL);
}

/**
 * Puts the ANGIE's EZ-USB microcontroller into reset state, downloads
 * the firmware image, resumes the microcontroller and re-enumerates
 * USB devices.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 *  The usb_handle member will be modified during re-enumeration.
 * @param filename path to the Intel HEX file containing the firmware image.
 * @param delay_us the delay to wait for the device to re-enumerate.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_load_firmware_and_renumerate(struct angie *device,
	const char *filename, uint32_t delay_us)
{
	int ret;

	/* Basic process: After downloading the firmware, the ANGIE will disconnect
	 * itself and re-connect after a short amount of time so we have to close
	 * the handle and re-enumerate USB devices */

	ret = angie_load_firmware(device, filename);
	if (ret != ERROR_OK)
		return ret;

	ret = angie_usb_close(device);
	if (ret != ERROR_OK)
		return ret;

	usleep(delay_us);

	ret = angie_usb_open(device);
	if (ret != ERROR_OK)
		return ret;

	ret = libusb_claim_interface(angie_handle->usb_device_handle, 0);
	if (ret != LIBUSB_SUCCESS)
		return ERROR_FAIL;

	return ERROR_OK;
}

/**
 * Downloads a firmware image to the ANGIE's EZ-USB microcontroller
 * over the USB bus.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param filename an absolute or relative path to the Intel HEX file
 *  containing the firmware image.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_load_firmware(struct angie *device, const char *filename)
{
	struct image angie_firmware_image;
	int ret;

	ret = angie_cpu_reset(device, CPU_RESET);
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

	/* Download all sections in the image to ANGIE */
	for (unsigned int i = 0; i < angie_firmware_image.num_sections; i++) {
		ret = angie_write_firmware_section(device, &angie_firmware_image, i);
		if (ret != ERROR_OK) {
			LOG_ERROR("Could not write firmware section");
			return ret;
		}
	}

	image_close(&angie_firmware_image);

	ret = angie_cpu_reset(device, CPU_START);
	if (ret != ERROR_OK) {
		LOG_ERROR("Could not restart ANGIE CPU");
		return ret;
	}

	return ERROR_OK;
}

/**
 * Downloads a bitstream file to the ANGIE's FPGA through the EZ-USB microcontroller
 * over the USB bus.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param filename an absolute or relative path to the Xilinx .bit file
 *  containing the bitstream data.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_load_bitstream(struct angie *device, const char *filename)
{
	int ret, transferred;
	const char *bitstream_file_path = filename;
	FILE *bitstream_file = NULL;
	char *bitstream_data = NULL;
	size_t bitstream_size = 0;
	uint8_t gpifcnt[4];

	/* Open the bitstream file */
	bitstream_file = fopen(bitstream_file_path, "rb");
	if (!bitstream_file) {
		LOG_ERROR("Failed to open bitstream file: %s\n", bitstream_file_path);
		return ERROR_FAIL;
	}

	/* Get the size of the bitstream file */
	fseek(bitstream_file, 0, SEEK_END);
	bitstream_size = ftell(bitstream_file);
	fseek(bitstream_file, 0, SEEK_SET);

	/* Allocate memory for the bitstream data */
	bitstream_data = malloc(bitstream_size);
	if (!bitstream_data) {
		LOG_ERROR("Failed to allocate memory for bitstream data.");
		fclose(bitstream_file);
		return ERROR_FAIL;
	}

	/* Read the bitstream data from the file */
	if (fread(bitstream_data, 1, bitstream_size, bitstream_file) != bitstream_size) {
		LOG_ERROR("Failed to read bitstream data.");
		free(bitstream_data);
		fclose(bitstream_file);
		return ERROR_FAIL;
	}

	h_u32_to_be(gpifcnt, bitstream_size);

	/* CFGopen */
	ret = jtag_libusb_control_transfer(device->usb_device_handle,
		0x00, 0xB0, 0, 0, (char *)gpifcnt, 4, LIBUSB_TIMEOUT_MS, &transferred);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed opencfg");
		/* Abort if libusb sent less data than requested */
		return ERROR_FAIL;
	}

	/* Send the bitstream data to the microcontroller */
	int actual_length = 0;
	ret = jtag_libusb_bulk_write(device->usb_device_handle, 0x02, bitstream_data, bitstream_size, 1000, &actual_length);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to send bitstream data: %s", libusb_strerror(ret));
		free(bitstream_data);
		fclose(bitstream_file);
		return ERROR_FAIL;
	}

	LOG_INFO("Bitstream sent successfully.");

	/* Clean up */
	free(bitstream_data);
	fclose(bitstream_file);

	/* CFGclose */
	transferred = 0;
	ret = jtag_libusb_control_transfer(device->usb_device_handle,
		0x00, 0xB1, 0, 0, NULL, 0, LIBUSB_TIMEOUT_MS, &transferred);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed cfgclose");
		/* Abort if libusb sent less data than requested */
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

/**
 * Send an i2c write operation to dev-board components.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param i2c_data table of i2c data that we want to write to slave device.
 * @param i2c_data_size the size of i2c data table.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_i2c_write(struct angie *device, uint8_t *i2c_data, uint8_t i2c_data_size)
{
	char i2c_data_buffer[i2c_data_size + 2];
	char buffer_received[1];
	int ret, transferred;
	i2c_data_buffer[0] = 0; // write = 0
	i2c_data_buffer[1] = i2c_data_size - 1; // i2c_data count (without address)

	for (uint8_t i = 0; i < i2c_data_size; i++)
		i2c_data_buffer[i + 2] = i2c_data[i];

	// Send i2c packet to Dev-board and configure its clock source /
	ret = jtag_libusb_bulk_write(device->usb_device_handle, 0x06, i2c_data_buffer,
								 i2c_data_size + 2, 1000, &transferred);
	if (ret != ERROR_OK) {
		LOG_ERROR("Error in i2c clock gen configuration : ret ERROR");
		return ret;
	}
	if (transferred != i2c_data_size + 2) {
		LOG_ERROR("Error in i2c clock gen configuration : bytes transferred");
		return ERROR_FAIL;
	}

	usleep(500);

	// Receive packet from ANGIE /
	ret = jtag_libusb_bulk_write(device->usb_device_handle, 0x88, buffer_received, 1, 1000, &transferred);
	if (ret != ERROR_OK) {
		LOG_ERROR("Error in i2c clock gen configuration : ret ERROR");
		return ret;
	}
	return ERROR_OK;
}

/**
 * Configure dev-board gpio extender modules by configuring their
 * register 3 and register 1 responsible for IO directions and values.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param i2c_adr i2c address of the gpio extender.
 * @param cfg_value IOs configuration to be written in register Number 3.
 * @param value the IOs value to be written in register Number 1.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_io_extender_config(struct angie *device, uint8_t i2c_adr, uint8_t cfg_value)
{
	uint8_t ioconfig[3] = {i2c_adr, 3, cfg_value};
	int ret = angie_i2c_write(device, ioconfig, 3);
	if (ret != ERROR_OK)
		return ret;

	usleep(500);
	return ret;
}

/**
 * Send one contiguous firmware section to the ANGIE's EZ-USB microcontroller
 * over the USB bus.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param firmware_image pointer to the firmware image that contains the section
 *  which should be sent to the ANGIE's EZ-USB microcontroller.
 * @param section_index index of the section within the firmware image.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_write_firmware_section(struct angie *device,
	struct image *firmware_image, int section_index)
{
	int addr, bytes_remaining, chunk_size;
	uint8_t data[SECTION_BUFFERSIZE];
	uint8_t *data_ptr = data;
	uint16_t size;
	size_t size_read;
	int ret, transferred;

	size = (uint16_t)firmware_image->sections[section_index].size;
	addr = (uint16_t)firmware_image->sections[section_index].base_address;

	LOG_DEBUG("section %02i at addr 0x%04x (size 0x%04" PRIx16 ")", section_index, addr,
		size);

	/* Copy section contents to local buffer */
	ret = image_read_section(firmware_image, section_index, 0, size, data,
			&size_read);

	if (ret != ERROR_OK)
		return ret;
	if (size_read != size)
		return ERROR_FAIL;

	bytes_remaining = size;

	/* Send section data in chunks of up to 64 bytes to ANGIE */
	while (bytes_remaining > 0) {
		if (bytes_remaining > 64)
			chunk_size = 64;
		else
			chunk_size = bytes_remaining;

		ret = jtag_libusb_control_transfer(device->usb_device_handle,
				(LIBUSB_ENDPOINT_OUT | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE),
				REQUEST_FIRMWARE_LOAD, addr, FIRMWARE_ADDR, (char *)data_ptr,
				chunk_size, LIBUSB_TIMEOUT_MS, &transferred);

		if (ret != ERROR_OK)
			return ret;

		if (transferred != chunk_size) {
			/* Abort if libusb sent less data than requested */
			return ERROR_FAIL;
		}

		bytes_remaining -= chunk_size;
		addr += chunk_size;
		data_ptr += chunk_size;
	}

	return ERROR_OK;
}

/************************** Generic helper functions **************************/

/**
 * Print state of interesting signals via LOG_INFO().
 *
 * @param input_signals input signal states as returned by CMD_GET_SIGNALS
 * @param output_signals output signal states as returned by CMD_GET_SIGNALS
 */
static void angie_dump_signal_states(uint8_t input_signals, uint8_t output_signals)
{
	LOG_INFO("ANGIE signal states: TDI: %i, TDO: %i, TMS: %i, TCK: %i, TRST: %i "
		"SRST: %i",
		(output_signals & SIGNAL_TDI   ? 1 : 0),
		(input_signals  & SIGNAL_TDO   ? 1 : 0),
		(output_signals & SIGNAL_TMS   ? 1 : 0),
		(output_signals & SIGNAL_TCK   ? 1 : 0),
		(output_signals & SIGNAL_TRST  ? 1 : 0),
		(output_signals & SIGNAL_SRST  ? 1 : 0));
}

/**************** ANGIE command generation helper functions ***************/

/**
 * Allocate and initialize space in memory for ANGIE command payload.
 *
 * @param angie_cmd pointer to command whose payload should be allocated.
 * @param size the amount of memory to allocate (bytes).
 * @param direction which payload to allocate.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_allocate_payload(struct angie_cmd *angie_cmd, int size,
	enum angie_payload_direction direction)
{
	uint8_t *payload;

	payload = calloc(size, sizeof(uint8_t));

	if (!payload) {
		LOG_ERROR("Could not allocate ANGIE command payload: out of memory");
		return ERROR_FAIL;
	}

	switch (direction) {
	    case PAYLOAD_DIRECTION_OUT:
		    if (angie_cmd->payload_out) {
			    LOG_ERROR("BUG: Duplicate payload allocation for ANGIE command");
			    free(payload);
			    return ERROR_FAIL;
		    }
			angie_cmd->payload_out = payload;
			angie_cmd->payload_out_size = size;
		    break;
	    case PAYLOAD_DIRECTION_IN:
		    if (angie_cmd->payload_in_start) {
			    LOG_ERROR("BUG: Duplicate payload allocation for ANGIE command");
			    free(payload);
			    return ERROR_FAIL;
		    }

			angie_cmd->payload_in_start = payload;
		    angie_cmd->payload_in = payload;
		    angie_cmd->payload_in_size = size;

			/* By default, free payload_in_start in angie_clear_queue(). Commands
			 * that do not want this behavior (e. g. split scans) must turn it off
			 * separately! */
		    angie_cmd->free_payload_in_start = true;

		    break;
	}

	return ERROR_OK;
}

/****************** ANGIE command queue helper functions ******************/

/**
 * Get the current number of bytes in the queue, including command IDs.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param direction the transfer direction for which to get byte count.
 * @return the number of bytes currently stored in the queue for the specified
 *  direction.
 */
static int angie_get_queue_size(struct angie *device,
	enum angie_payload_direction direction)
{
	struct angie_cmd *current = device->queue_start;
	int sum = 0;

	while (current) {
		switch (direction) {
		    case PAYLOAD_DIRECTION_OUT:
			    sum += current->payload_out_size + 1;	/* + 1 byte for Command ID */
			    break;
		    case PAYLOAD_DIRECTION_IN:
			    sum += current->payload_in_size;
			    break;
		}

		current = current->next;
	}

	return sum;
}

/**
 * Clear the ANGIE command queue.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 */
static void angie_clear_queue(struct angie *device)
{
	struct angie_cmd *current = device->queue_start;
	struct angie_cmd *next = NULL;

	while (current) {
		/* Save pointer to next element */
		next = current->next;

		/* Free payloads: OUT payload can be freed immediately */
		free(current->payload_out);
		current->payload_out = NULL;

		/* IN payload MUST be freed ONLY if no other commands use the
		 * payload_in_start buffer */
		if (current->free_payload_in_start) {
			free(current->payload_in_start);
			current->payload_in_start = NULL;
			current->payload_in = NULL;
		}

		/* Free queue element */
		free(current);

		/* Proceed with next element */
		current = next;
	}

	device->commands_in_queue = 0;
	device->queue_start = NULL;
	device->queue_end = NULL;
}

/**
 * Add a command to the ANGIE command queue.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param angie_cmd pointer to command that shall be appended to the ANGIE
 *  command queue.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_append_queue(struct angie *device, struct angie_cmd *angie_cmd)
{
	int newsize_out, newsize_in;
	int ret = ERROR_OK;

	newsize_out = angie_get_queue_size(device, PAYLOAD_DIRECTION_OUT) + 1
		+ angie_cmd->payload_out_size;

	newsize_in = angie_get_queue_size(device, PAYLOAD_DIRECTION_IN)
		+ angie_cmd->payload_in_size;

	/* Check if the current command can be appended to the queue */
	if (newsize_out > 64 || newsize_in > 64) {
		/* New command does not fit. Execute all commands in queue before starting
		 * new queue with the current command as first entry. */
		ret = angie_execute_queued_commands(device, LIBUSB_TIMEOUT_MS);

		if (ret == ERROR_OK)
			ret = angie_post_process_queue(device);

		if (ret == ERROR_OK)
			angie_clear_queue(device);
	}

	if (!device->queue_start) {
		/* Queue was empty */
		device->commands_in_queue = 1;

		device->queue_start = angie_cmd;
		device->queue_end = angie_cmd;
	} else {
		/* There are already commands in the queue */
		device->commands_in_queue++;

		device->queue_end->next = angie_cmd;
		device->queue_end = angie_cmd;
	}

	if (ret != ERROR_OK)
		angie_clear_queue(device);

	return ret;
}

/**
 * Sends all queued ANGIE commands to the ANGIE for execution.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param timeout_ms
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_execute_queued_commands(struct angie *device, int timeout_ms)
{
	struct angie_cmd *current;
	int ret, i, index_out, index_in, count_out, count_in, transferred;
	uint8_t buffer[64];

	if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO))
		angie_dump_queue(device);

	index_out = 0;
	count_out = 0;
	count_in = 0;

	for (current = device->queue_start; current; current = current->next) {
		/* Add command to packet */
		buffer[index_out] = current->id;
		index_out++;
		count_out++;

		for (i = 0; i < current->payload_out_size; i++)
			buffer[index_out + i] = current->payload_out[i];
		index_out += current->payload_out_size;
		count_in += current->payload_in_size;
		count_out += current->payload_out_size;
	}

	/* Send packet to ANGIE */
	ret = jtag_libusb_bulk_write(device->usb_device_handle, device->ep_out,
			(char *)buffer, count_out, timeout_ms, &transferred);
	if (ret != ERROR_OK) {
		LOG_ERROR("Libusb bulk write queued commands failed.");
		return ret;
	}
	if (transferred != count_out) {
		LOG_ERROR("Libusb bulk write queued commands failed: transferred byte count");
		return ERROR_FAIL;
	}

	/* Wait for response if commands contain IN payload data */
	if (count_in > 0) {
		ret = jtag_libusb_bulk_write(device->usb_device_handle, device->ep_in,
				(char *)buffer, count_in, timeout_ms, &transferred);
	if (ret != ERROR_OK) {
		LOG_ERROR("Libusb bulk write input payload data failed");
		return ret;
	}
	if (transferred != count_in) {
		LOG_ERROR("Libusb bulk write input payload data failed: transferred byte count");
		return ERROR_FAIL;
	}

		/* Write back IN payload data */
		index_in = 0;
		for (current = device->queue_start; current; current = current->next) {
			for (i = 0; i < current->payload_in_size; i++) {
				current->payload_in[i] = buffer[index_in];
				index_in++;
			}
		}
	}
	return ERROR_OK;
}

/**
 * Convert an ANGIE command ID (\a id) to a human-readable string.
 *
 * @param id the ANGIE command ID.
 * @return the corresponding human-readable string.
 */
static const char *angie_cmd_id_string(uint8_t id)
{
	switch (id) {
	case CMD_SCAN_IN:
		return "CMD_SCAN_IN";
	case CMD_SLOW_SCAN_IN:
		return "CMD_SLOW_SCAN_IN";
	case CMD_SCAN_OUT:
		return "CMD_SCAN_OUT";
	case CMD_SLOW_SCAN_OUT:
		return "CMD_SLOW_SCAN_OUT";
	case CMD_SCAN_IO:
		return "CMD_SCAN_IO";
	case CMD_SLOW_SCAN_IO:
		return "CMD_SLOW_SCAN_IO";
	case CMD_CLOCK_TMS:
		return "CMD_CLOCK_TMS";
	case CMD_SLOW_CLOCK_TMS:
		return "CMD_SLOW_CLOCK_TMS";
	case CMD_CLOCK_TCK:
		return "CMD_CLOCK_TCK";
	case CMD_SLOW_CLOCK_TCK:
		return "CMD_SLOW_CLOCK_TCK";
	case CMD_SLEEP_US:
		return "CMD_SLEEP_US";
	case CMD_SLEEP_MS:
		return "CMD_SLEEP_MS";
	case CMD_GET_SIGNALS:
		return "CMD_GET_SIGNALS";
	case CMD_SET_SIGNALS:
		return "CMD_SET_SIGNALS";
	case CMD_CONFIGURE_TCK_FREQ:
		return "CMD_CONFIGURE_TCK_FREQ";
	case CMD_SET_LEDS:
		return "CMD_SET_LEDS";
	case CMD_TEST:
		return "CMD_TEST";
	default:
		return "CMD_UNKNOWN";
	}
}

/**
 * Print one ANGIE command to stdout.
 *
 * @param angie_cmd pointer to ANGIE command.
 */
static void angie_dump_command(struct angie_cmd *angie_cmd)
{
	char hex[64 * 3];
	for (int i = 0; i < angie_cmd->payload_out_size; i++)
		sprintf(hex + 3 * i, "%02" PRIX8 " ", angie_cmd->payload_out[i]);

	hex[3 * angie_cmd->payload_out_size - 1] = 0;
	LOG_DEBUG_IO(" %-22s | OUT size = %" PRIi8 ", bytes = %s",
					angie_cmd_id_string(angie_cmd->id), angie_cmd->payload_out_size, hex);

	LOG_DEBUG_IO("\n                         | IN size  =  %" PRIi8 "\n", angie_cmd->payload_in_size);
}

/**
 * Print the ANGIE command queue to stdout.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 */
static void angie_dump_queue(struct angie *device)
{
	struct angie_cmd *current;

	LOG_DEBUG_IO("ANGIE command queue:\n");

	for (current = device->queue_start; current; current = current->next)
		angie_dump_command(current);
}

/**
 * Perform JTAG scan
 *
 * Creates and appends a JTAG scan command to the ANGIE command queue.
 * A JTAG scan consists of three steps:
 * - Move to the desired SHIFT state, depending on scan type (IR/DR scan).
 * - Shift TDI data into the JTAG chain, optionally reading the TDO pin.
 * - Move to the desired end state.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param scan_type the type of the scan (IN, OUT, IO (bidirectional)).
 * @param scan_size_bits number of bits to shift into the JTAG chain.
 * @param tdi pointer to array containing TDI data.
 * @param tdo_start pointer to first element of array where TDO data shall be
 *  stored. See #angie_cmd for details.
 * @param tdo pointer to array where TDO data shall be stored
 * @param tms_count_start number of TMS state transitions to perform BEFORE
 *  shifting data into the JTAG chain.
 * @param tms_sequence_start sequence of TMS state transitions that will be
 *  performed BEFORE shifting data into the JTAG chain.
 * @param tms_count_end number of TMS state transitions to perform AFTER
 *  shifting data into the JTAG chain.
 * @param tms_sequence_end sequence of TMS state transitions that will be
 *  performed AFTER shifting data into the JTAG chain.
 * @param origin pointer to OpenOCD command that generated this scan command.
 * @param postprocess whether this command needs to be post-processed after
 *  execution.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_append_scan_cmd(struct angie *device, enum scan_type scan_type,
	int scan_size_bits, uint8_t *tdi, uint8_t *tdo_start, uint8_t *tdo,
	uint8_t tms_count_start, uint8_t tms_sequence_start, uint8_t tms_count_end,
	uint8_t tms_sequence_end, struct jtag_command *origin, bool postprocess)
{
	struct angie_cmd *cmd = calloc(1, sizeof(struct angie_cmd));
	int ret, i, scan_size_bytes;
	uint8_t bits_last_byte;

	if (!cmd)
		return ERROR_FAIL;

	/* Check size of command. USB buffer can hold 64 bytes, 1 byte is command ID,
	 * 5 bytes are setup data -> 58 remaining payload bytes for TDI data */
	if (scan_size_bits > (58 * 8)) {
		LOG_ERROR("BUG: Tried to create CMD_SCAN_IO ANGIE command with too"
			" large payload");
		free(cmd);
		return ERROR_FAIL;
	}

	scan_size_bytes = DIV_ROUND_UP(scan_size_bits, 8);

	bits_last_byte = scan_size_bits % 8;
	if (bits_last_byte == 0)
		bits_last_byte = 8;

	/* Allocate out_payload depending on scan type */
	switch (scan_type) {
	    case SCAN_IN:
		    if (device->delay_scan_in < 0)
			    cmd->id = CMD_SCAN_IN;
		    else
			    cmd->id = CMD_SLOW_SCAN_IN;
		    ret = angie_allocate_payload(cmd, 5, PAYLOAD_DIRECTION_IN);
		    break;
	    case SCAN_OUT:
		    if (device->delay_scan_out < 0)
			    cmd->id = CMD_SCAN_OUT;
		    else
			    cmd->id = CMD_SLOW_SCAN_OUT;
		    ret = angie_allocate_payload(cmd, scan_size_bytes + 5, PAYLOAD_DIRECTION_OUT);
		    break;
	    case SCAN_IO:
		    if (device->delay_scan_io < 0)
			    cmd->id = CMD_SCAN_IO;
		    else
			    cmd->id = CMD_SLOW_SCAN_IO;
		    ret = angie_allocate_payload(cmd, scan_size_bytes + 5, PAYLOAD_DIRECTION_OUT);
		    break;
	    default:
		    LOG_ERROR("BUG: 'append scan cmd' encountered an unknown scan type");
		    ret = ERROR_FAIL;
		    break;
	}

	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	/* Build payload_out that is common to all scan types */
	cmd->payload_out[0] = scan_size_bytes & 0xFF;
	cmd->payload_out[1] = bits_last_byte & 0xFF;
	cmd->payload_out[2] = ((tms_count_start & 0x0F) << 4) | (tms_count_end & 0x0F);
	cmd->payload_out[3] = tms_sequence_start;
	cmd->payload_out[4] = tms_sequence_end;

	/* Setup payload_out for types with OUT transfer */
	if (scan_type == SCAN_OUT || scan_type == SCAN_IO) {
		for (i = 0; i < scan_size_bytes; i++)
			cmd->payload_out[i + 5] = tdi[i];
	}

	/* Setup payload_in pointers for types with IN transfer */
	if (scan_type == SCAN_IN || scan_type == SCAN_IO) {
		cmd->payload_in_start = tdo_start;
		cmd->payload_in = tdo;
		cmd->payload_in_size = scan_size_bytes;
	}

	cmd->needs_postprocessing = postprocess;
	cmd->cmd_origin = origin;

	/* For scan commands, we free payload_in_start only when the command is
	 * the last in a series of split commands or a stand-alone command */
	cmd->free_payload_in_start = postprocess;

	return angie_append_queue(device, cmd);
}

/**
 * Perform TAP state transitions
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param count defines the number of TCK clock cycles generated (up to 8).
 * @param sequence defines the TMS pin levels for each state transition. The
 *  Least-Significant Bit is read first.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_append_clock_tms_cmd(struct angie *device, uint8_t count,
	uint8_t sequence)
{
	struct angie_cmd *cmd = calloc(1, sizeof(struct angie_cmd));
	int ret;

	if (!cmd) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	if (device->delay_clock_tms < 0)
		cmd->id = CMD_CLOCK_TMS;
	else
		cmd->id = CMD_SLOW_CLOCK_TMS;

	/* CMD_CLOCK_TMS has two OUT payload bytes and zero IN payload bytes */
	ret = angie_allocate_payload(cmd, 2, PAYLOAD_DIRECTION_OUT);
	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	cmd->payload_out[0] = count;
	cmd->payload_out[1] = sequence;

	return angie_append_queue(device, cmd);
}

/**
 * Generate a defined amount of TCK clock cycles
 *
 * All other JTAG signals are left unchanged.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param count the number of TCK clock cycles to generate.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_append_clock_tck_cmd(struct angie *device, uint16_t count)
{
	struct angie_cmd *cmd = calloc(1, sizeof(struct angie_cmd));
	int ret;

	if (!cmd) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	if (device->delay_clock_tck < 0)
		cmd->id = CMD_CLOCK_TCK;
	else
		cmd->id = CMD_SLOW_CLOCK_TCK;

	/* CMD_CLOCK_TCK has two OUT payload bytes and zero IN payload bytes */
	ret = angie_allocate_payload(cmd, 2, PAYLOAD_DIRECTION_OUT);
	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	cmd->payload_out[0] = count & 0xff;
	cmd->payload_out[1] = (count >> 8) & 0xff;

	return angie_append_queue(device, cmd);
}

/**
 * Read JTAG signals.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_append_get_signals_cmd(struct angie *device)
{
	struct angie_cmd *cmd = calloc(1, sizeof(struct angie_cmd));
	int ret;

	if (!cmd) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	cmd->id = CMD_GET_SIGNALS;
	cmd->needs_postprocessing = true;

	/* CMD_GET_SIGNALS has two IN payload bytes */
	ret = angie_allocate_payload(cmd, 2, PAYLOAD_DIRECTION_IN);

	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	return angie_append_queue(device, cmd);
}

/**
 * Arbitrarily set JTAG output signals.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param low defines which signals will be de-asserted. Each bit corresponds
 *  to a JTAG signal:
 *  - SIGNAL_TDI
 *  - SIGNAL_TMS
 *  - SIGNAL_TCK
 *  - SIGNAL_TRST
 *  - SIGNAL_BRKIN
 *  - SIGNAL_RESET
 *  - SIGNAL_OCDSE
 * @param high defines which signals will be asserted.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_append_set_signals_cmd(struct angie *device, uint8_t low,
	uint8_t high)
{
	struct angie_cmd *cmd = calloc(1, sizeof(struct angie_cmd));
	int ret;

	if (!cmd) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	cmd->id = CMD_SET_SIGNALS;

	/* CMD_SET_SIGNALS has two OUT payload bytes and zero IN payload bytes */
	ret = angie_allocate_payload(cmd, 2, PAYLOAD_DIRECTION_OUT);

	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	cmd->payload_out[0] = low;
	cmd->payload_out[1] = high;

	return angie_append_queue(device, cmd);
}

/**
 * Sleep for a pre-defined number of microseconds
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param us the number microseconds to sleep.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_append_sleep_cmd(struct angie *device, uint32_t us)
{
	struct angie_cmd *cmd = calloc(1, sizeof(struct angie_cmd));
	int ret;

	if (!cmd) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	cmd->id = CMD_SLEEP_US;

	/* CMD_SLEEP_US has two OUT payload bytes and zero IN payload bytes */
	ret = angie_allocate_payload(cmd, 2, PAYLOAD_DIRECTION_OUT);

	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	cmd->payload_out[0] = us & 0x00ff;
	cmd->payload_out[1] = (us >> 8) & 0x00ff;

	return angie_append_queue(device, cmd);
}

/**
 * Set TCK delay counters
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param delay_scan_in delay count top value in jtag_slow_scan_in() function.
 * @param delay_scan_out delay count top value in jtag_slow_scan_out() function.
 * @param delay_scan_io delay count top value in jtag_slow_scan_io() function.
 * @param delay_tck delay count top value in jtag_clock_tck() function.
 * @param delay_tms delay count top value in jtag_slow_clock_tms() function.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_append_configure_tck_cmd(struct angie *device, int delay_scan_in,
	int delay_scan_out, int delay_scan_io, int delay_tck, int delay_tms)
{
	struct angie_cmd *cmd = calloc(1, sizeof(struct angie_cmd));
	int ret;

	if (!cmd) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	cmd->id = CMD_CONFIGURE_TCK_FREQ;

	/* CMD_CONFIGURE_TCK_FREQ has five OUT payload bytes and zero
	 * IN payload bytes */
	ret = angie_allocate_payload(cmd, 5, PAYLOAD_DIRECTION_OUT);
	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	if (delay_scan_in < 0)
		cmd->payload_out[0] = 0;
	else
		cmd->payload_out[0] = (uint8_t)delay_scan_in;

	if (delay_scan_out < 0)
		cmd->payload_out[1] = 0;
	else
		cmd->payload_out[1] = (uint8_t)delay_scan_out;

	if (delay_scan_io < 0)
		cmd->payload_out[2] = 0;
	else
		cmd->payload_out[2] = (uint8_t)delay_scan_io;

	if (delay_tck < 0)
		cmd->payload_out[3] = 0;
	else
		cmd->payload_out[3] = (uint8_t)delay_tck;

	if (delay_tms < 0)
		cmd->payload_out[4] = 0;
	else
		cmd->payload_out[4] = (uint8_t)delay_tms;

	return angie_append_queue(device, cmd);
}

/**
 * Test command. Used to check if the ANGIE device is ready to accept new
 * commands.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_append_test_cmd(struct angie *device)
{
	struct angie_cmd *cmd = calloc(1, sizeof(struct angie_cmd));
	int ret;

	if (!cmd) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	cmd->id = CMD_TEST;

	/* CMD_TEST has one OUT payload byte and zero IN payload bytes */
	ret = angie_allocate_payload(cmd, 1, PAYLOAD_DIRECTION_OUT);
	if (ret != ERROR_OK) {
		free(cmd);
		return ret;
	}

	cmd->payload_out[0] = 0xAA;

	return angie_append_queue(device, cmd);
}

/****************** ANGIE TCK frequency helper functions ******************/

/**
 * Calculate delay values for a given TCK frequency.
 *
 * The ANGIE firmware uses five different speed values for different
 * commands. These speed values are calculated in these functions.
 *
 * The five different commands which support variable TCK frequency are
 * implemented twice in the firmware:
 *   1. Maximum possible frequency without any artificial delay
 *   2. Variable frequency with artificial linear delay loop
 *
 * To set the ANGIE to maximum frequency, it is only necessary to use the
 * corresponding command IDs. To set the ANGIE to a lower frequency, the
 * delay loop top values have to be calculated first. Then, a
 * CMD_CONFIGURE_TCK_FREQ command needs to be sent to the ANGIE device.
 *
 * The delay values are described by linear equations:
 *    t = k * x + d
 *    (t = period, k = constant, x = delay value, d = constant)
 *
 * Thus, the delay can be calculated as in the following equation:
 *    x = (t - d) / k
 *
 * The constants in these equations have been determined and validated by
 * measuring the frequency resulting from different delay values.
 *
 * @param type for which command to calculate the delay value.
 * @param f TCK frequency for which to calculate the delay value in Hz.
 * @param delay where to store resulting delay value.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_calculate_delay(enum angie_delay_type type, long f, int *delay)
{
	float t_us, x, x_ceil;

	/* Calculate period of requested TCK frequency */
	t_us = 1000000.0 / f;

	switch (type) {
	    case DELAY_CLOCK_TCK:
		    x = (t_us - 6.0) / 4;
		    break;
	    case DELAY_CLOCK_TMS:
		    x = (t_us - 8.5) / 4;
		    break;
	    case DELAY_SCAN_IN:
		    x = (t_us - 8.8308) / 4;
		    break;
	    case DELAY_SCAN_OUT:
		    x = (t_us - 10.527) / 4;
		    break;
	    case DELAY_SCAN_IO:
		    x = (t_us - 13.132) / 4;
		    break;
	    default:
		    return ERROR_FAIL;
		    break;
	}

	/* Check if the delay value is negative. This happens when a frequency is
	 * requested that is too high for the delay loop implementation. In this
	 * case, set delay value to zero. */
	if (x < 0)
		x = 0;

	/* We need to convert the exact delay value to an integer. Therefore, we
	 * round the exact value UP to ensure that the resulting frequency is NOT
	 * higher than the requested frequency. */
	x_ceil = ceilf(x);

	/* Check if the value is within limits */
	if (x_ceil > 255)
		return ERROR_FAIL;

	*delay = (int)x_ceil;

	return ERROR_OK;
}

/**
 * Calculate frequency for a given delay value.
 *
 * Similar to the #angie_calculate_delay function, this function calculates the
 * TCK frequency for a given delay value by using linear equations of the form:
 *    t = k * x + d
 *    (t = period, k = constant, x = delay value, d = constant)
 *
 * @param type for which command to calculate the delay value.
 * @param delay value for which to calculate the resulting TCK frequency.
 * @return the resulting TCK frequency
 */
static long angie_calculate_frequency(enum angie_delay_type type, int delay)
{
	float t_us, f_float;

	if (delay > 255)
		return 0;

	switch (type) {
	    case DELAY_CLOCK_TCK:
		    if (delay < 0)
			    t_us = 2.666;
		    else
			    t_us = (4.0 * delay) + 6.0;
		    break;
	    case DELAY_CLOCK_TMS:
		    if (delay < 0)
			    t_us = 5.666;
		    else
			    t_us = (4.0 * delay) + 8.5;
		    break;
	    case DELAY_SCAN_IN:
		    if (delay < 0)
			    t_us = 5.5;
		    else
			    t_us = (4.0 * delay) + 8.8308;
		    break;
	    case DELAY_SCAN_OUT:
		    if (delay < 0)
			    t_us = 7.0;
		    else
			    t_us = (4.0 * delay) + 10.527;
		    break;
	    case DELAY_SCAN_IO:
		    if (delay < 0)
			    t_us = 9.926;
		    else
			    t_us = (4.0 * delay) + 13.132;
		    break;
	    default:
		    return 0;
	}

	f_float = 1000000.0 / t_us;
	return roundf(f_float);
}

/******************* Interface between ANGIE and OpenOCD ******************/

/**
 * Sets the end state follower (see interface.h) if \a endstate is a stable
 * state.
 *
 * @param endstate the state the end state follower should be set to.
 */
static void angie_set_end_state(enum tap_state endstate)
{
	if (tap_is_state_stable(endstate))
		tap_set_end_state(endstate);
	else
		LOG_ERROR("BUG: %s is not a valid end state", tap_state_name(endstate));
}

/**
 * Move from the current TAP state to the current TAP end state.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_queue_statemove(struct angie *device)
{
	uint8_t tms_sequence, tms_count;
	int ret;

	if (tap_get_state() == tap_get_end_state()) {
		/* Do nothing if we are already there */
		return ERROR_OK;
	}

	tms_sequence = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	ret = angie_append_clock_tms_cmd(device, tms_count, tms_sequence);

	if (ret == ERROR_OK)
		tap_set_state(tap_get_end_state());

	return ret;
}

/**
 * Perform a scan operation on a JTAG register.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param cmd pointer to the command that shall be executed.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_queue_scan(struct angie *device, struct jtag_command *cmd)
{
	uint32_t scan_size_bits, scan_size_bytes, bits_last_scan;
	uint32_t scans_max_payload, bytecount;
	uint8_t *tdi_buffer_start = NULL, *tdi_buffer = NULL;
	uint8_t *tdo_buffer_start = NULL, *tdo_buffer = NULL;

	uint8_t first_tms_count, first_tms_sequence;
	uint8_t last_tms_count, last_tms_sequence;

	uint8_t tms_count_pause, tms_sequence_pause;
	uint8_t tms_count_resume, tms_sequence_resume;

	uint8_t tms_count_start, tms_sequence_start;
	uint8_t tms_count_end, tms_sequence_end;

	enum scan_type type;
	int ret;

	/* Determine scan size */
	scan_size_bits = jtag_scan_size(cmd->cmd.scan);
	scan_size_bytes = DIV_ROUND_UP(scan_size_bits, 8);

	/* Determine scan type (IN/OUT/IO) */
	type = jtag_scan_type(cmd->cmd.scan);

	/* Determine number of scan commands with maximum payload */
	scans_max_payload = scan_size_bytes / 58;

	/* Determine size of last shift command */
	bits_last_scan = scan_size_bits - (scans_max_payload * 58 * 8);

	/* Allocate TDO buffer if required */
	if (type == SCAN_IN || type == SCAN_IO) {
		tdo_buffer_start = calloc(scan_size_bytes, sizeof(uint8_t));

		if (!tdo_buffer_start)
			return ERROR_FAIL;

		tdo_buffer = tdo_buffer_start;
	}

	/* Fill TDI buffer if required */
	if (type == SCAN_OUT || type == SCAN_IO) {
		jtag_build_buffer(cmd->cmd.scan, &tdi_buffer_start);
		tdi_buffer = tdi_buffer_start;
	}

	/* Get TAP state transitions */
	if (cmd->cmd.scan->ir_scan) {
		angie_set_end_state(TAP_IRSHIFT);
		first_tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
		first_tms_sequence = tap_get_tms_path(tap_get_state(), tap_get_end_state());

		tap_set_state(TAP_IRSHIFT);
		tap_set_end_state(cmd->cmd.scan->end_state);
		last_tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
		last_tms_sequence = tap_get_tms_path(tap_get_state(), tap_get_end_state());

		/* TAP state transitions for split scans */
		tms_count_pause = tap_get_tms_path_len(TAP_IRSHIFT, TAP_IRPAUSE);
		tms_sequence_pause = tap_get_tms_path(TAP_IRSHIFT, TAP_IRPAUSE);
		tms_count_resume = tap_get_tms_path_len(TAP_IRPAUSE, TAP_IRSHIFT);
		tms_sequence_resume = tap_get_tms_path(TAP_IRPAUSE, TAP_IRSHIFT);
	} else {
		angie_set_end_state(TAP_DRSHIFT);
		first_tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
		first_tms_sequence = tap_get_tms_path(tap_get_state(), tap_get_end_state());

		tap_set_state(TAP_DRSHIFT);
		tap_set_end_state(cmd->cmd.scan->end_state);
		last_tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
		last_tms_sequence = tap_get_tms_path(tap_get_state(), tap_get_end_state());

		/* TAP state transitions for split scans */
		tms_count_pause = tap_get_tms_path_len(TAP_DRSHIFT, TAP_DRPAUSE);
		tms_sequence_pause = tap_get_tms_path(TAP_DRSHIFT, TAP_DRPAUSE);
		tms_count_resume = tap_get_tms_path_len(TAP_DRPAUSE, TAP_DRSHIFT);
		tms_sequence_resume = tap_get_tms_path(TAP_DRPAUSE, TAP_DRSHIFT);
	}

	/* Generate scan commands */
	bytecount = scan_size_bytes;
	while (bytecount > 0) {
		if (bytecount == scan_size_bytes) {
			/* This is the first scan */
			tms_count_start = first_tms_count;
			tms_sequence_start = first_tms_sequence;
		} else {
			/* Resume from previous scan */
			tms_count_start = tms_count_resume;
			tms_sequence_start = tms_sequence_resume;
		}

		if (bytecount > 58) {	/* Full scan, at least one scan will follow */
			tms_count_end = tms_count_pause;
			tms_sequence_end = tms_sequence_pause;

			ret = angie_append_scan_cmd(device,
					type,
					58 * 8,
					tdi_buffer,
					tdo_buffer_start,
					tdo_buffer,
					tms_count_start,
					tms_sequence_start,
					tms_count_end,
					tms_sequence_end,
					cmd,
					false);

			bytecount -= 58;

			/* Update TDI and TDO buffer pointers */
			if (tdi_buffer_start)
				tdi_buffer += 58;
			if (tdo_buffer_start)
				tdo_buffer += 58;
		} else if (bytecount == 58) {	/* Full scan, no further scans */
			tms_count_end = last_tms_count;
			tms_sequence_end = last_tms_sequence;

			ret = angie_append_scan_cmd(device,
					type,
					58 * 8,
					tdi_buffer,
					tdo_buffer_start,
					tdo_buffer,
					tms_count_start,
					tms_sequence_start,
					tms_count_end,
					tms_sequence_end,
					cmd,
					true);

			bytecount = 0;
		} else {/* Scan with less than maximum payload, no further scans */
			tms_count_end = last_tms_count;
			tms_sequence_end = last_tms_sequence;

			ret = angie_append_scan_cmd(device,
					type,
					bits_last_scan,
					tdi_buffer,
					tdo_buffer_start,
					tdo_buffer,
					tms_count_start,
					tms_sequence_start,
					tms_count_end,
					tms_sequence_end,
					cmd,
					true);

			bytecount = 0;
		}

		if (ret != ERROR_OK) {
			free(tdi_buffer_start);
			free(tdo_buffer_start);
			return ret;
		}
	}

	free(tdi_buffer_start);

	/* Set current state to the end state requested by the command */
	tap_set_state(cmd->cmd.scan->end_state);

	return ERROR_OK;
}

/**
 * Move the TAP into the Test Logic Reset state.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param cmd pointer to the command that shall be executed.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_queue_tlr_reset(struct angie *device, struct jtag_command *cmd)
{
	int ret = angie_append_clock_tms_cmd(device, 5, 0xff);

	if (ret == ERROR_OK)
		tap_set_state(TAP_RESET);

	return ret;
}

/**
 * Run Test.
 *
 * Generate TCK clock cycles while remaining
 * in the Run-Test/Idle state.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param cmd pointer to the command that shall be executed.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_queue_runtest(struct angie *device, struct jtag_command *cmd)
{
	int ret;

	/* Only perform statemove if the TAP currently isn't in the TAP_IDLE state */
	if (tap_get_state() != TAP_IDLE) {
		angie_set_end_state(TAP_IDLE);
		angie_queue_statemove(device);
	}

	/* Generate the clock cycles */
	ret = angie_append_clock_tck_cmd(device, cmd->cmd.runtest->num_cycles);
	if (ret != ERROR_OK)
		return ret;

	/* Move to end state specified in command */
	if (cmd->cmd.runtest->end_state != tap_get_state()) {
		tap_set_end_state(cmd->cmd.runtest->end_state);
		angie_queue_statemove(device);
	}

	return ERROR_OK;
}

/**
 * Execute a JTAG_RESET command
 *
 * @param device
 * @param trst indicate if trst signal is activated.
 * @param srst indicate if srst signal is activated.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_reset(int trst, int srst)
{
	struct angie *device = angie_handle;
	uint8_t low = 0, high = 0;

	if (trst) {
		tap_set_state(TAP_RESET);
		low |= SIGNAL_TRST;
	} else {
		high |= SIGNAL_TRST;
	}

	if (srst)
		low |= SIGNAL_SRST;
	else
		high |= SIGNAL_SRST;

	int ret = angie_append_set_signals_cmd(device, low, high);
	if (ret != ERROR_OK)
		return ret;

	ret = angie_execute_queued_commands(device, LIBUSB_TIMEOUT_MS);
	if (ret != ERROR_OK)
		return ret;

	angie_clear_queue(device);

	return ERROR_OK;
}

/**
 * Move to one TAP state or several states in succession.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param cmd pointer to the command that shall be executed.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_queue_pathmove(struct angie *device, struct jtag_command *cmd)
{
	int ret, state_count;
	enum tap_state *path;
	uint8_t tms_sequence;

	unsigned int num_states = cmd->cmd.pathmove->num_states;
	path = cmd->cmd.pathmove->path;
	state_count = 0;

	while (num_states > 0) {
		unsigned int batch_size;

		tms_sequence = 0;

		/* Determine batch size */
		if (num_states >= 8)
			batch_size = 8;
		else
			batch_size = num_states;

		for (unsigned int i = 0; i < batch_size; i++) {
			if (tap_state_transition(tap_get_state(), false) == path[state_count]) {
				/* Append '0' transition: clear bit 'i' in tms_sequence */
				buf_set_u32(&tms_sequence, i, 1, 0x0);
			} else if (tap_state_transition(tap_get_state(), true)
				   == path[state_count]) {
				/* Append '1' transition: set bit 'i' in tms_sequence */
				buf_set_u32(&tms_sequence, i, 1, 0x1);
			} else {
				/* Invalid state transition */
				LOG_ERROR("BUG: %s -> %s isn't a valid TAP state transition",
					tap_state_name(tap_get_state()),
					tap_state_name(path[state_count]));
				return ERROR_FAIL;
			}

			tap_set_state(path[state_count]);
			state_count++;
			num_states--;
		}

		/* Append CLOCK_TMS command to ANGIE command queue */
		LOG_INFO("pathmove batch: count = %i, sequence = 0x%" PRIx8 "", batch_size, tms_sequence);
		ret = angie_append_clock_tms_cmd(angie_handle, batch_size, tms_sequence);
		if (ret != ERROR_OK)
			return ret;
	}

	return ERROR_OK;
}

/**
 * Sleep for a specific amount of time.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param cmd pointer to the command that shall be executed.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_queue_sleep(struct angie *device, struct jtag_command *cmd)
{
	/* IMPORTANT! Due to the time offset in command execution introduced by
	 * command queueing, this needs to be implemented in the ANGIE device */
	return angie_append_sleep_cmd(device, cmd->cmd.sleep->us);
}

/**
 * Generate TCK cycles while remaining in a stable state.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @param cmd pointer to the command that shall be executed.
 */
static int angie_queue_stableclocks(struct angie *device, struct jtag_command *cmd)
{
	int ret;

	if (!tap_is_state_stable(tap_get_state())) {
		LOG_ERROR("JTAG_STABLECLOCKS: state not stable");
		return ERROR_FAIL;
	}

	unsigned int num_cycles = cmd->cmd.stableclocks->num_cycles;

	/* TMS stays either high (Test Logic Reset state) or low (all other states) */
	if (tap_get_state() == TAP_RESET)
		ret = angie_append_set_signals_cmd(device, 0, SIGNAL_TMS);
	else
		ret = angie_append_set_signals_cmd(device, SIGNAL_TMS, 0);

	if (ret != ERROR_OK)
		return ret;

	while (num_cycles > 0) {
		if (num_cycles > 0xFFFF) {
			/* ANGIE CMD_CLOCK_TCK can generate up to 0xFFFF (uint16_t) cycles */
			ret = angie_append_clock_tck_cmd(device, 0xFFFF);
			num_cycles -= 0xFFFF;
		} else {
			ret = angie_append_clock_tck_cmd(device, num_cycles);
			num_cycles = 0;
		}

		if (ret != ERROR_OK)
			return ret;
	}

	return ERROR_OK;
}

/**
 * Post-process JTAG_SCAN command
 *
 * @param angie_cmd pointer to ANGIE command that shall be processed.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_post_process_scan(struct angie_cmd *angie_cmd)
{
	struct jtag_command *cmd = angie_cmd->cmd_origin;
	int ret;

	switch (jtag_scan_type(cmd->cmd.scan)) {
	    case SCAN_IN:
	    case SCAN_IO:
		    ret = jtag_read_buffer(angie_cmd->payload_in_start, cmd->cmd.scan);
		    break;
	    case SCAN_OUT:
			/* Nothing to do for OUT scans */
		    ret = ERROR_OK;
		    break;
	    default:
		    LOG_ERROR("BUG: angie post process scan encountered an unknown JTAG scan type");
		    ret = ERROR_FAIL;
		    break;
	}

	return ret;
}

/**
 * Perform post-processing of commands after ANGIE queue has been executed.
 *
 * @param device pointer to struct angie identifying ANGIE driver instance.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_post_process_queue(struct angie *device)
{
	struct angie_cmd *current;
	struct jtag_command *openocd_cmd;
	int ret;

	current = device->queue_start;

	while (current) {
		openocd_cmd = current->cmd_origin;

		/* Check if a corresponding OpenOCD command is stored for this
		 * ANGIE command */
		if (current->needs_postprocessing && openocd_cmd) {
			switch (openocd_cmd->type) {
			    case JTAG_SCAN:
				    ret = angie_post_process_scan(current);
				    break;
			    case JTAG_TLR_RESET:
			    case JTAG_RUNTEST:
			    case JTAG_PATHMOVE:
			    case JTAG_SLEEP:
			    case JTAG_STABLECLOCKS:
					/* Nothing to do for these commands */
				    ret = ERROR_OK;
				    break;
			    default:
				    ret = ERROR_FAIL;
				    LOG_ERROR("BUG: angie post process queue encountered unknown JTAG "
					"command type");
				    break;
			}

			if (ret != ERROR_OK)
				return ret;
		}

		current = current->next;
	}

	return ERROR_OK;
}

/**************************** JTAG driver functions ***************************/

/**
 * Executes the JTAG Command Queue.
 *
 * This is done in three stages: First, all OpenOCD commands are processed into
 * queued ANGIE commands. Next, the ANGIE command queue is sent to the
 * ANGIE device and data received from the ANGIE device is cached. Finally,
 * the post-processing function writes back data to the corresponding OpenOCD
 * commands.
 *
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_execute_queue(struct jtag_command *cmd_queue)
{
	struct jtag_command *cmd = cmd_queue;
	int ret;

	while (cmd) {
		switch (cmd->type) {
		    case JTAG_SCAN:
			    ret = angie_queue_scan(angie_handle, cmd);
			    break;
		    case JTAG_TLR_RESET:
			    ret = angie_queue_tlr_reset(angie_handle, cmd);
			    break;
		    case JTAG_RUNTEST:
			    ret = angie_queue_runtest(angie_handle, cmd);
			    break;
		    case JTAG_PATHMOVE:
			    ret = angie_queue_pathmove(angie_handle, cmd);
			    break;
		    case JTAG_SLEEP:
			    ret = angie_queue_sleep(angie_handle, cmd);
			    break;
		    case JTAG_STABLECLOCKS:
			    ret = angie_queue_stableclocks(angie_handle, cmd);
			    break;
		    default:
			    ret = ERROR_FAIL;
			    LOG_ERROR("BUG: encountered unknown JTAG command type");
			    break;
		}

		if (ret != ERROR_OK)
			return ret;

		cmd = cmd->next;
	}

	if (angie_handle->commands_in_queue > 0) {
		ret = angie_execute_queued_commands(angie_handle, LIBUSB_TIMEOUT_MS);
		if (ret != ERROR_OK)
			return ret;

		ret = angie_post_process_queue(angie_handle);
		if (ret != ERROR_OK)
			return ret;

		angie_clear_queue(angie_handle);
	}

	return ERROR_OK;
}

/**
 * Set the TCK frequency of the ANGIE adapter.
 *
 * @param khz desired JTAG TCK frequency.
 * @param jtag_speed where to store corresponding adapter-specific speed value.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_khz(int khz, int *jtag_speed)
{
	int ret;

	if (khz == 0) {
		LOG_ERROR("RCLK not supported");
		return ERROR_FAIL;
	}

	/* CLOCK_TCK commands are decoupled from others. Therefore, the frequency
	 * setting can be done independently from all other commands. */
	if (khz >= 375) {
		angie_handle->delay_clock_tck = -1;
	} else {
		ret = angie_calculate_delay(DELAY_CLOCK_TCK, khz * 1000,
				&angie_handle->delay_clock_tck);
		if (ret != ERROR_OK)
			return ret;
	}

	/* SCAN_{IN,OUT,IO} commands invoke CLOCK_TMS commands. Therefore, if the
	 * requested frequency goes below the maximum frequency for SLOW_CLOCK_TMS
	 * commands, all SCAN commands MUST also use the variable frequency
	 * implementation! */
	if (khz >= 176) {
		angie_handle->delay_clock_tms = -1;
		angie_handle->delay_scan_in = -1;
		angie_handle->delay_scan_out = -1;
		angie_handle->delay_scan_io = -1;
	} else {
		ret = angie_calculate_delay(DELAY_CLOCK_TMS, khz * 1000,
				&angie_handle->delay_clock_tms);
		if (ret != ERROR_OK)
			return ret;

		ret = angie_calculate_delay(DELAY_SCAN_IN, khz * 1000,
				&angie_handle->delay_scan_in);
		if (ret != ERROR_OK)
			return ret;

		ret = angie_calculate_delay(DELAY_SCAN_OUT, khz * 1000,
				&angie_handle->delay_scan_out);
		if (ret != ERROR_OK)
			return ret;

		ret = angie_calculate_delay(DELAY_SCAN_IO, khz * 1000,
				&angie_handle->delay_scan_io);
		if (ret != ERROR_OK)
			return ret;
	}

	LOG_DEBUG_IO("ANGIE TCK setup: delay_tck      = %i (%li Hz),",
		angie_handle->delay_clock_tck,
		angie_calculate_frequency(DELAY_CLOCK_TCK, angie_handle->delay_clock_tck));
	LOG_DEBUG_IO("                 delay_tms      = %i (%li Hz),",
		angie_handle->delay_clock_tms,
		angie_calculate_frequency(DELAY_CLOCK_TMS, angie_handle->delay_clock_tms));
	LOG_DEBUG_IO("                 delay_scan_in  = %i (%li Hz),",
		angie_handle->delay_scan_in,
		angie_calculate_frequency(DELAY_SCAN_IN, angie_handle->delay_scan_in));
	LOG_DEBUG_IO("                 delay_scan_out = %i (%li Hz),",
		angie_handle->delay_scan_out,
		angie_calculate_frequency(DELAY_SCAN_OUT, angie_handle->delay_scan_out));
	LOG_DEBUG_IO("                 delay_scan_io  = %i (%li Hz),",
		angie_handle->delay_scan_io,
		angie_calculate_frequency(DELAY_SCAN_IO, angie_handle->delay_scan_io));

	/* Configure the ANGIE device with the new delay values */
	ret = angie_append_configure_tck_cmd(angie_handle,
			angie_handle->delay_scan_in,
			angie_handle->delay_scan_out,
			angie_handle->delay_scan_io,
			angie_handle->delay_clock_tck,
			angie_handle->delay_clock_tms);

	if (ret != ERROR_OK)
		return ret;

	*jtag_speed = khz;

	return ERROR_OK;
}

/**
 * Set the TCK frequency of the ANGIE adapter.
 *
 * Because of the way the TCK frequency is set up in the ANGIE firmware,
 * there are five different speed settings. To simplify things, the
 * adapter-specific speed setting value is identical to the TCK frequency in
 * khz.
 *
 * @param speed desired adapter-specific speed value.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_speed(int speed)
{
	int dummy;

	return angie_khz(speed, &dummy);
}

/**
 * Convert adapter-specific speed value to corresponding TCK frequency in kHz.
 *
 * Because of the way the TCK frequency is set up in the ANGIE firmware,
 * there are five different speed settings. To simplify things, the
 * adapter-specific speed setting value is identical to the TCK frequency in
 * khz.
 *
 * @param speed adapter-specific speed value.
 * @param khz where to store corresponding TCK frequency in kHz.
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_speed_div(int speed, int *khz)
{
	*khz = speed;

	return ERROR_OK;
}

/**
 * Initiates the firmware download to the ANGIE adapter and prepares
 * the USB handle.
 *
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_init(void)
{
	int ret, transferred;
	char str_manufacturer[20];
	bool download_firmware = false;
	char dummy[64];
	uint8_t input_signals, output_signals;

	angie_handle = calloc(1, sizeof(struct angie));

	if (!angie_handle) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	ret = angie_usb_open(angie_handle);
	if (ret != ERROR_OK) {
		free(angie_handle);
		angie_handle = NULL;
		return ret;
	}

	/* Get String Descriptor to determine if firmware needs to be loaded */
	ret = libusb_get_string_descriptor_ascii(angie_handle->usb_device_handle, 1, (unsigned char *)str_manufacturer, 20);
	if (ret < 0) {
		/* Could not get descriptor -> Unconfigured or original Keil firmware */
		download_firmware = true;
	} else {
		/* We got a String Descriptor, check if it is the correct one */
		if (strncmp(str_manufacturer, "NanoXplore, SAS.", 16) != 0)
			download_firmware = true;
	}

	if (download_firmware) {
		LOG_INFO("Loading ANGIE firmware. This is reversible by power-cycling ANGIE device.");
		if (libusb_claim_interface(angie_handle->usb_device_handle, 0) != LIBUSB_SUCCESS) {
			LOG_ERROR("Could not claim interface 0");
			return ERROR_FAIL;
		}
		ret = angie_load_firmware_and_renumerate(angie_handle,
				ANGIE_FIRMWARE_FILE, ANGIE_RENUMERATION_DELAY_US);
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
		if (libusb_release_interface(angie_handle->usb_device_handle, 0) != LIBUSB_SUCCESS) {
			LOG_ERROR("Fail release interface 0");
			return ERROR_FAIL;
		}
		if (libusb_claim_interface(angie_handle->usb_device_handle, 1) != LIBUSB_SUCCESS) {
			LOG_ERROR("Could not claim interface 1");
			return ERROR_FAIL;
		}
		/* Configure io extender 23: all input */
		ret = angie_io_extender_config(angie_handle, 0x23, 0xFF);
		if (ret != ERROR_OK) {
			LOG_ERROR("Could not configure io extender 23");
			return ret;
		}
		if (libusb_release_interface(angie_handle->usb_device_handle, 1) != LIBUSB_SUCCESS) {
			LOG_ERROR("Fail release interface 1");
			return ERROR_FAIL;
		}
	} else {
		LOG_INFO("ANGIE device is already running ANGIE firmware");
	}

	/* Get ANGIE USB IN/OUT endpoints and claim the interface 0 */
	ret = jtag_libusb_choose_interface(angie_handle->usb_device_handle,
		&angie_handle->ep_in, &angie_handle->ep_out, 0xFF, 0, 0, -1);
	if (ret != ERROR_OK) {
		LOG_ERROR("Choose and claim interface failed");
		angie_quit();
		return ret;
	}

	/* Initialize ANGIE command queue */
	angie_clear_queue(angie_handle);

	/* Issue one test command with short timeout */
	ret = angie_append_test_cmd(angie_handle);
	if (ret != ERROR_OK) {
		LOG_ERROR("Append test command failed.");
		angie_quit();
		return ret;
	}

	ret = angie_execute_queued_commands(angie_handle, 200);
	if (ret != ERROR_OK) {
		/* Sending test command failed. The ANGIE device may be forever waiting for
		 * the host to fetch an USB Bulk IN packet (e. g. OpenOCD crashed or was
		 * shut down by the user via Ctrl-C. Try to retrieve this Bulk IN packet. */

		ret = jtag_libusb_bulk_write(angie_handle->usb_device_handle, angie_handle->ep_in,
				dummy, 64, 200, &transferred);

		if (ret != ERROR_OK || transferred == 0) {
			/* Bulk IN transfer failed -> unrecoverable error condition */
			LOG_ERROR("Cannot communicate with ANGIE device. Disconnect ANGIE from "
				"the USB port and re-connect, then re-run OpenOCD");
			angie_quit();
			return ERROR_FAIL;
		}
		/* Successfully received Bulk IN packet -> continue */
		LOG_INFO("Recovered from lost Bulk IN packet");
	}

	angie_clear_queue(angie_handle);

	/* Execute get signals command */
	ret = angie_append_get_signals_cmd(angie_handle);
	if (ret != ERROR_OK) {
		LOG_ERROR("Append get signals command failed");
		angie_quit();
		return ret;
	}
	ret = angie_execute_queued_commands(angie_handle, 200);
	if (ret != ERROR_OK) {
		LOG_ERROR("Execute get signals command failed");
		angie_quit();
		return ret;
	}

	/* Post-process the single CMD_GET_SIGNALS command */
	input_signals = angie_handle->queue_start->payload_in[0];
	output_signals = angie_handle->queue_start->payload_in[1];
	angie_dump_signal_states(input_signals, output_signals);

	angie_clear_queue(angie_handle);

	return ERROR_OK;
}

/**
 * Closes the USB handle for the ANGIE device.
 *
 * @return on success: ERROR_OK
 * @return on failure: ERROR_FAIL
 */
static int angie_quit(void)
{
	int ret = angie_usb_close(angie_handle);
	free(angie_handle);
	angie_handle = NULL;

	return ret;
}

static struct jtag_interface angie_interface = {
	.execute_queue = angie_execute_queue,
};

struct adapter_driver angie_adapter_driver = {
	.name = "angie",
	.transports = jtag_only,

	.init = angie_init,
	.quit = angie_quit,
	.reset = angie_reset,
	.speed = angie_speed,
	.khz = angie_khz,
	.speed_div = angie_speed_div,

	.jtag_ops = &angie_interface,
};
