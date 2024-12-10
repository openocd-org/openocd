// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2018 by Mickaël Thomas                                  *
 *   mickael9@gmail.com                                                    *
 *                                                                         *
 *   Copyright (C) 2016 by Maksym Hilliaka                                 *
 *   oter@frozen-team.com                                                  *
 *                                                                         *
 *   Copyright (C) 2016 by Phillip Pearson                                 *
 *   pp@myelin.co.nz                                                       *
 *                                                                         *
 *   Copyright (C) 2014 by Paul Fertser                                    *
 *   fercerpav@gmail.com                                                   *
 *                                                                         *
 *   Copyright (C) 2013 by mike brown                                      *
 *   mike@theshedworks.org.uk                                              *
 *                                                                         *
 *   Copyright (C) 2013 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/system.h>
#include <libusb.h>
#include <helper/log.h>
#include <helper/replacements.h>
#include <jtag/jtag.h>	/* ERROR_JTAG_DEVICE_ERROR only */

#include "cmsis_dap.h"
#include "libusb_helper.h"

enum {
	CMSIS_DAP_TRANSFER_PENDING = 0,	/* must be 0, used in libusb_handle_events_completed */
	CMSIS_DAP_TRANSFER_IDLE,
	CMSIS_DAP_TRANSFER_COMPLETED
};

struct cmsis_dap_bulk_transfer {
	struct libusb_transfer *transfer;
	uint8_t *buffer;
	int status;		/* either CMSIS_DAP_TRANSFER_ enum or error code */
	int transferred;
};

struct cmsis_dap_backend_data {
	struct libusb_context *usb_ctx;
	struct libusb_device_handle *dev_handle;
	unsigned int ep_out;
	unsigned int ep_in;
	int interface;

	struct cmsis_dap_bulk_transfer command_transfers[MAX_PENDING_REQUESTS];
	struct cmsis_dap_bulk_transfer response_transfers[MAX_PENDING_REQUESTS];
};

static int cmsis_dap_usb_interface = -1;

static void cmsis_dap_usb_close(struct cmsis_dap *dap);
static int cmsis_dap_usb_alloc(struct cmsis_dap *dap, unsigned int pkt_sz);
static void cmsis_dap_usb_free(struct cmsis_dap *dap);

static int cmsis_dap_usb_open(struct cmsis_dap *dap, uint16_t vids[], uint16_t pids[], const char *serial)
{
	int err;
	struct libusb_context *ctx;
	struct libusb_device **device_list;

	err = libusb_init(&ctx);
	if (err) {
		LOG_ERROR("libusb initialization failed: %s", libusb_strerror(err));
		return ERROR_FAIL;
	}

	int num_devices = libusb_get_device_list(ctx, &device_list);
	if (num_devices < 0) {
		LOG_ERROR("could not enumerate USB devices: %s", libusb_strerror(num_devices));
		libusb_exit(ctx);
		return ERROR_FAIL;
	}

	for (int i = 0; i < num_devices; i++) {
		struct libusb_device *dev = device_list[i];
		struct libusb_device_descriptor dev_desc;

		err = libusb_get_device_descriptor(dev, &dev_desc);
		if (err) {
			LOG_ERROR("could not get device descriptor for device %d: %s", i, libusb_strerror(err));
			continue;
		}

		/* Match VID/PID */

		bool id_match = false;
		bool id_filter = vids[0] || pids[0];
		for (int id = 0; vids[id] || pids[id]; id++) {
			id_match = !vids[id] || dev_desc.idVendor == vids[id];
			id_match &= !pids[id] || dev_desc.idProduct == pids[id];

			if (id_match)
				break;
		}

		if (id_filter && !id_match)
			continue;

		/* Don't continue if we asked for a serial number and the device doesn't have one */
		if (dev_desc.iSerialNumber == 0 && serial && serial[0])
			continue;

		struct libusb_device_handle *dev_handle = NULL;
		err = libusb_open(dev, &dev_handle);
		if (err) {
			/* It's to be expected that most USB devices can't be opened
			 * so only report an error if it was explicitly selected
			 */
			if (id_filter) {
				LOG_ERROR("could not open device 0x%04x:0x%04x: %s",
						dev_desc.idVendor, dev_desc.idProduct, libusb_strerror(err));
			} else {
				LOG_DEBUG("could not open device 0x%04x:0x%04x: %s",
						dev_desc.idVendor, dev_desc.idProduct, libusb_strerror(err));
			}
			continue;
		}

		/* Match serial number */

		bool serial_match = false;
		char dev_serial[256] = {0};
		if (dev_desc.iSerialNumber > 0) {
			err = libusb_get_string_descriptor_ascii(
					dev_handle, dev_desc.iSerialNumber,
					(uint8_t *)dev_serial, sizeof(dev_serial));

			if (err < 0) {
				const char *msg = "could not read serial number for device 0x%04x:0x%04x: %s";
				if (serial)
					LOG_WARNING(msg, dev_desc.idVendor, dev_desc.idProduct,
								libusb_strerror(err));
				else
					LOG_DEBUG(msg, dev_desc.idVendor, dev_desc.idProduct,
								libusb_strerror(err));
			} else if (serial && strncmp(dev_serial, serial, sizeof(dev_serial)) == 0) {
				serial_match = true;
			}
		}

		if (serial && !serial_match) {
			libusb_close(dev_handle);
			continue;
		}

		/* Find the CMSIS-DAP string in product string */

		bool cmsis_dap_in_product_str = false;
		char product_string[256] = {0};
		if (dev_desc.iProduct > 0) {
			err = libusb_get_string_descriptor_ascii(
					dev_handle, dev_desc.iProduct,
					(uint8_t *)product_string, sizeof(product_string));
			if (err < 0) {
				LOG_WARNING("could not read product string for device 0x%04x:0x%04x: %s",
						dev_desc.idVendor, dev_desc.idProduct, libusb_strerror(err));
			} else if (strstr(product_string, "CMSIS-DAP")) {
				LOG_DEBUG("found product string of 0x%04x:0x%04x '%s'",
						  dev_desc.idVendor, dev_desc.idProduct, product_string);
				cmsis_dap_in_product_str = true;
			}
		}

		bool device_identified_reliably = cmsis_dap_in_product_str
											|| serial_match || id_match;

		/* Find the CMSIS-DAP interface */

		for (int config = 0; config < dev_desc.bNumConfigurations; config++) {
			struct libusb_config_descriptor *config_desc;
			err = libusb_get_config_descriptor(dev, config, &config_desc);
			if (err) {
				LOG_ERROR("could not get configuration descriptor %d for device 0x%04x:0x%04x: %s",
						config, dev_desc.idVendor, dev_desc.idProduct, libusb_strerror(err));
				continue;
			}

			LOG_DEBUG("enumerating interfaces of 0x%04x:0x%04x",
					  dev_desc.idVendor, dev_desc.idProduct);
			int config_num = config_desc->bConfigurationValue;
			const struct libusb_interface_descriptor *intf_desc_candidate = NULL;
			const struct libusb_interface_descriptor *intf_desc_found = NULL;

			for (int interface = 0; interface < config_desc->bNumInterfaces; interface++) {
				const struct libusb_interface_descriptor *intf_desc = &config_desc->interface[interface].altsetting[0];
				int interface_num = intf_desc->bInterfaceNumber;

				/* Skip this interface if another one was requested explicitly */
				if (cmsis_dap_usb_interface != -1 && cmsis_dap_usb_interface != interface_num)
					continue;

				/* CMSIS-DAP v2 spec says:
				 *
				 * CMSIS-DAP with default V2 configuration uses WinUSB and is therefore faster.
				 * Optionally support for streaming SWO trace is provided via an additional USB endpoint.
				 *
				 * The WinUSB configuration requires custom class support with the interface setting
				 *     Class Code: 0xFF (Vendor specific)
				 *     Subclass: 0x00
				 *     Protocol code: 0x00
				 *
				 * Depending on the configuration it uses the following USB endpoints which should be configured
				 * in the interface descriptor in this order:
				 *  - Endpoint 1: Bulk Out – used for commands received from host PC.
				 *  - Endpoint 2: Bulk In – used for responses send to host PC.
				 *  - Endpoint 3: Bulk In (optional) – used for streaming SWO trace (if enabled with SWO_STREAM).
				 */

				/* Search for "CMSIS-DAP" in the interface string */
				bool cmsis_dap_in_interface_str = false;
				if (intf_desc->iInterface != 0) {

					char interface_str[256] = {0};

					err = libusb_get_string_descriptor_ascii(
							dev_handle, intf_desc->iInterface,
							(uint8_t *)interface_str, sizeof(interface_str));
					if (err < 0) {
						LOG_DEBUG("could not read interface string %d for device 0x%04x:0x%04x: %s",
								  intf_desc->iInterface,
								  dev_desc.idVendor, dev_desc.idProduct,
								  libusb_strerror(err));
					} else if (strstr(interface_str, "CMSIS-DAP")) {
						cmsis_dap_in_interface_str = true;
						LOG_DEBUG("found interface %d string '%s'",
								  interface_num, interface_str);
					}
				}

				/* Bypass the following check if this interface was explicitly requested. */
				if (cmsis_dap_usb_interface == -1) {
					if (!cmsis_dap_in_product_str && !cmsis_dap_in_interface_str)
						continue;
				}

				/* check endpoints */
				if (intf_desc->bNumEndpoints < 2) {
					LOG_DEBUG("skipping interface %d, has only %d endpoints",
							  interface_num, intf_desc->bNumEndpoints);
					continue;
				}

				if ((intf_desc->endpoint[0].bmAttributes & 3) != LIBUSB_TRANSFER_TYPE_BULK ||
						(intf_desc->endpoint[0].bEndpointAddress & 0x80) != LIBUSB_ENDPOINT_OUT) {
					LOG_DEBUG("skipping interface %d, endpoint[0] is not bulk out",
							  interface_num);
					continue;
				}

				if ((intf_desc->endpoint[1].bmAttributes & 3) != LIBUSB_TRANSFER_TYPE_BULK ||
						(intf_desc->endpoint[1].bEndpointAddress & 0x80) != LIBUSB_ENDPOINT_IN) {
					LOG_DEBUG("skipping interface %d, endpoint[1] is not bulk in",
							  interface_num);
					continue;
				}

				/* We can rely on the interface is really CMSIS-DAP if
				 * - we've seen CMSIS-DAP in the interface string
				 * - config asked explicitly for an interface number
				 * - the device has only one interface
				 * The later two cases should be honored only if we know
				 * we are on the right device */
				bool intf_identified_reliably = cmsis_dap_in_interface_str
							|| (device_identified_reliably &&
									(cmsis_dap_usb_interface != -1
									 || config_desc->bNumInterfaces == 1));

				if (intf_desc->bInterfaceClass != LIBUSB_CLASS_VENDOR_SPEC ||
						intf_desc->bInterfaceSubClass != 0 || intf_desc->bInterfaceProtocol != 0) {
					/* If the interface is reliably identified
					 * then we need not insist on setting USB class, subclass and protocol
					 * exactly as the specification requires.
					 * Just filter out the well known classes, mainly CDC and MSC.
					 * At least KitProg3 uses class 0 contrary to the specification */
					if (intf_identified_reliably &&
							(intf_desc->bInterfaceClass == 0 || intf_desc->bInterfaceClass > 0x12)) {
						LOG_WARNING("Using CMSIS-DAPv2 interface %d with wrong class %" PRId8
								  " subclass %" PRId8 " or protocol %" PRId8,
								  interface_num,
								  intf_desc->bInterfaceClass,
								  intf_desc->bInterfaceSubClass,
								  intf_desc->bInterfaceProtocol);
					} else {
						LOG_DEBUG("skipping interface %d, class %" PRId8
								  " subclass %" PRId8 " protocol %" PRId8,
								  interface_num,
								  intf_desc->bInterfaceClass,
								  intf_desc->bInterfaceSubClass,
								  intf_desc->bInterfaceProtocol);
						continue;

					}
				}

				if (intf_identified_reliably) {
					/* That's the one! */
					intf_desc_found = intf_desc;
					break;
				}

				if (!intf_desc_candidate && device_identified_reliably) {
					/* This interface looks suitable for CMSIS-DAP. Store the pointer to it
					 * and keep searching for another one with CMSIS-DAP in interface string */
					intf_desc_candidate = intf_desc;
				}
			}

			if (!intf_desc_found) {
				/* We were not able to identify reliably which interface is CMSIS-DAP.
				 * Let's use the first suitable if we found one */
				intf_desc_found = intf_desc_candidate;
			}

			if (!intf_desc_found) {
				libusb_free_config_descriptor(config_desc);
				continue;
			}

			/* We've chosen an interface, connect to it */
			int interface_num = intf_desc_found->bInterfaceNumber;
			int packet_size = intf_desc_found->endpoint[0].wMaxPacketSize;
			int ep_out = intf_desc_found->endpoint[0].bEndpointAddress;
			int ep_in = intf_desc_found->endpoint[1].bEndpointAddress;

			libusb_free_config_descriptor(config_desc);
			libusb_free_device_list(device_list, true);

			LOG_INFO("Using CMSIS-DAPv2 interface with VID:PID=0x%04x:0x%04x, serial=%s",
					dev_desc.idVendor, dev_desc.idProduct, dev_serial);

			int current_config;
			err = libusb_get_configuration(dev_handle, &current_config);
			if (err) {
				LOG_ERROR("could not find current configuration: %s", libusb_strerror(err));
				libusb_close(dev_handle);
				libusb_exit(ctx);
				return ERROR_FAIL;
			}

			if (config_num != current_config) {
				err = libusb_set_configuration(dev_handle, config_num);
				if (err) {
					LOG_ERROR("could not set configuration: %s", libusb_strerror(err));
					libusb_close(dev_handle);
					libusb_exit(ctx);
					return ERROR_FAIL;
				}
			}

			err = libusb_claim_interface(dev_handle, interface_num);
			if (err)
				LOG_WARNING("could not claim interface: %s", libusb_strerror(err));

			dap->bdata = calloc(1, sizeof(struct cmsis_dap_backend_data));
			if (!dap->bdata) {
				LOG_ERROR("unable to allocate memory");
				libusb_release_interface(dev_handle, interface_num);
				libusb_close(dev_handle);
				libusb_exit(ctx);
				return ERROR_FAIL;
			}

			dap->bdata->usb_ctx = ctx;
			dap->bdata->dev_handle = dev_handle;
			dap->bdata->ep_out = ep_out;
			dap->bdata->ep_in = ep_in;
			dap->bdata->interface = interface_num;

			for (unsigned int idx = 0; idx < MAX_PENDING_REQUESTS; idx++) {
				dap->bdata->command_transfers[idx].status = CMSIS_DAP_TRANSFER_IDLE;
				dap->bdata->command_transfers[idx].transfer = libusb_alloc_transfer(0);
				if (!dap->bdata->command_transfers[idx].transfer) {
					LOG_ERROR("unable to allocate USB transfer");
					cmsis_dap_usb_close(dap);
					return ERROR_FAIL;
				}

				dap->bdata->response_transfers[idx].status = CMSIS_DAP_TRANSFER_IDLE;
				dap->bdata->response_transfers[idx].transfer = libusb_alloc_transfer(0);
				if (!dap->bdata->response_transfers[idx].transfer) {
					LOG_ERROR("unable to allocate USB transfer");
					cmsis_dap_usb_close(dap);
					return ERROR_FAIL;
				}
			}

			err = cmsis_dap_usb_alloc(dap, packet_size);
			if (err != ERROR_OK)
				cmsis_dap_usb_close(dap);

			return err;
		}

		libusb_close(dev_handle);
	}

	libusb_free_device_list(device_list, true);

	libusb_exit(ctx);
	return ERROR_FAIL;
}

static void cmsis_dap_usb_close(struct cmsis_dap *dap)
{
	for (unsigned int i = 0; i < MAX_PENDING_REQUESTS; i++) {
		libusb_free_transfer(dap->bdata->command_transfers[i].transfer);
		libusb_free_transfer(dap->bdata->response_transfers[i].transfer);
	}
	cmsis_dap_usb_free(dap);
	libusb_release_interface(dap->bdata->dev_handle, dap->bdata->interface);
	libusb_close(dap->bdata->dev_handle);
	libusb_exit(dap->bdata->usb_ctx);
	free(dap->bdata);
	dap->bdata = NULL;
}

static void LIBUSB_CALL cmsis_dap_usb_callback(struct libusb_transfer *transfer)
{
	struct cmsis_dap_bulk_transfer *tr;

	tr = (struct cmsis_dap_bulk_transfer *)transfer->user_data;
	if (transfer->status == LIBUSB_TRANSFER_COMPLETED) {
		tr->status = CMSIS_DAP_TRANSFER_COMPLETED;
		tr->transferred = transfer->actual_length;
	} else if (transfer->status == LIBUSB_TRANSFER_TIMED_OUT) {
		tr->status = ERROR_TIMEOUT_REACHED;
	} else {
		tr->status = ERROR_JTAG_DEVICE_ERROR;
	}
}

static int cmsis_dap_usb_read(struct cmsis_dap *dap, int transfer_timeout_ms,
							  enum cmsis_dap_blocking blocking)
{
	int transferred = 0;
	int err;
	struct cmsis_dap_bulk_transfer *tr;
	tr = &dap->bdata->response_transfers[dap->pending_fifo_get_idx];

	if (tr->status == CMSIS_DAP_TRANSFER_IDLE) {
		libusb_fill_bulk_transfer(tr->transfer,
								  dap->bdata->dev_handle, dap->bdata->ep_in,
								  tr->buffer, dap->packet_size,
								  &cmsis_dap_usb_callback, tr,
								  transfer_timeout_ms);
		LOG_DEBUG_IO("submit read @ %u", dap->pending_fifo_get_idx);
		tr->status = CMSIS_DAP_TRANSFER_PENDING;
		err = libusb_submit_transfer(tr->transfer);
		if (err) {
			tr->status = CMSIS_DAP_TRANSFER_IDLE;
			LOG_ERROR("error submitting USB read: %s", libusb_strerror(err));
			return ERROR_FAIL;
		}
	}

	struct timeval tv;
	if (blocking == CMSIS_DAP_NON_BLOCKING) {
		tv.tv_sec = 0;
		tv.tv_usec = 0;
	} else {
		tv.tv_sec = transfer_timeout_ms / 1000;
		tv.tv_usec = transfer_timeout_ms % 1000 * 1000;
	}

	while (tr->status == CMSIS_DAP_TRANSFER_PENDING) {
		err = libusb_handle_events_timeout_completed(dap->bdata->usb_ctx, &tv,
												 &tr->status);
		if (err) {
			LOG_ERROR("error handling USB events: %s", libusb_strerror(err));
			return ERROR_FAIL;
		}
		if (tv.tv_sec == 0 && tv.tv_usec == 0)
			break;
	}

	if (tr->status < 0 || tr->status == CMSIS_DAP_TRANSFER_COMPLETED) {
		/* Check related command request for an error */
		struct cmsis_dap_bulk_transfer *tr_cmd;
		tr_cmd = &dap->bdata->command_transfers[dap->pending_fifo_get_idx];
		if (tr_cmd->status < 0) {
			err = tr_cmd->status;
			tr_cmd->status = CMSIS_DAP_TRANSFER_IDLE;
			if (err != ERROR_TIMEOUT_REACHED)
				LOG_ERROR("error writing USB data");
			else
				LOG_DEBUG("command write USB timeout @ %u", dap->pending_fifo_get_idx);

			return err;
		}
		if (tr_cmd->status == CMSIS_DAP_TRANSFER_COMPLETED)
			tr_cmd->status = CMSIS_DAP_TRANSFER_IDLE;
	}

	if (tr->status < 0) {
		err = tr->status;
		tr->status = CMSIS_DAP_TRANSFER_IDLE;
		if (err != ERROR_TIMEOUT_REACHED)
			LOG_ERROR("error reading USB data");
		else
			LOG_DEBUG("USB timeout @ %u", dap->pending_fifo_get_idx);

		return err;
	}

	if (tr->status == CMSIS_DAP_TRANSFER_COMPLETED) {
		transferred = tr->transferred;
		LOG_DEBUG_IO("completed read @ %u, transferred %i",
					 dap->pending_fifo_get_idx, transferred);
		memcpy(dap->packet_buffer, tr->buffer, transferred);
		memset(&dap->packet_buffer[transferred], 0, dap->packet_buffer_size - transferred);
		tr->status = CMSIS_DAP_TRANSFER_IDLE;
	}

	return transferred;
}

static int cmsis_dap_usb_write(struct cmsis_dap *dap, int txlen, int timeout_ms)
{
	int err;
	struct cmsis_dap_bulk_transfer *tr;
	tr = &dap->bdata->command_transfers[dap->pending_fifo_put_idx];

	if (tr->status == CMSIS_DAP_TRANSFER_PENDING) {
		LOG_ERROR("busy command USB transfer at %u", dap->pending_fifo_put_idx);
		struct timeval tv = {
			.tv_sec = timeout_ms / 1000,
			.tv_usec = timeout_ms % 1000 * 1000
		};
		libusb_handle_events_timeout_completed(dap->bdata->usb_ctx, &tv, &tr->status);
	}
	if (tr->status < 0) {
		if (tr->status != ERROR_TIMEOUT_REACHED)
			LOG_ERROR("error writing USB data, late detect");
		else
			LOG_DEBUG("USB write timeout @ %u, late detect", dap->pending_fifo_get_idx);
		tr->status = CMSIS_DAP_TRANSFER_IDLE;
	}
	if (tr->status == CMSIS_DAP_TRANSFER_COMPLETED) {
		LOG_ERROR("USB write: late transfer competed");
		tr->status = CMSIS_DAP_TRANSFER_IDLE;
	}
	if (tr->status != CMSIS_DAP_TRANSFER_IDLE) {
		libusb_cancel_transfer(tr->transfer);
		/* TODO: switch to less verbose errors and wait for USB working again */
		return ERROR_JTAG_DEVICE_ERROR;
	}

	memcpy(tr->buffer, dap->packet_buffer, txlen);

	libusb_fill_bulk_transfer(tr->transfer,
							  dap->bdata->dev_handle, dap->bdata->ep_out,
							  tr->buffer, txlen,
							  &cmsis_dap_usb_callback, tr,
							  timeout_ms);

	LOG_DEBUG_IO("submit write @ %u", dap->pending_fifo_put_idx);
	tr->status = CMSIS_DAP_TRANSFER_PENDING;
	err = libusb_submit_transfer(tr->transfer);
	if (err) {
		if (err == LIBUSB_ERROR_BUSY)
			libusb_cancel_transfer(tr->transfer);
		else
			tr->status = CMSIS_DAP_TRANSFER_IDLE;

		LOG_ERROR("error submitting USB write: %s", libusb_strerror(err));
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int cmsis_dap_usb_alloc(struct cmsis_dap *dap, unsigned int pkt_sz)
{
	dap->packet_buffer = malloc(pkt_sz);
	if (!dap->packet_buffer) {
		LOG_ERROR("unable to allocate CMSIS-DAP packet buffer");
		return ERROR_FAIL;
	}

	dap->packet_size = pkt_sz;
	dap->packet_buffer_size = pkt_sz;
	/* Prevent sending zero size USB packets */
	dap->packet_usable_size = pkt_sz - 1;

	dap->command = dap->packet_buffer;
	dap->response = dap->packet_buffer;

	struct cmsis_dap_backend_data *bdata = dap->bdata;
	for (unsigned int i = 0; i < MAX_PENDING_REQUESTS; i++) {
		bdata->command_transfers[i].buffer =
			oocd_libusb_dev_mem_alloc(bdata->dev_handle, pkt_sz);

		bdata->response_transfers[i].buffer =
			oocd_libusb_dev_mem_alloc(bdata->dev_handle, pkt_sz);

		if (!bdata->command_transfers[i].buffer
			|| !bdata->response_transfers[i].buffer) {
			LOG_ERROR("unable to allocate CMSIS-DAP pending packet buffer");
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static void cmsis_dap_usb_free(struct cmsis_dap *dap)
{
	struct cmsis_dap_backend_data *bdata = dap->bdata;

	for (unsigned int i = 0; i < MAX_PENDING_REQUESTS; i++) {
		oocd_libusb_dev_mem_free(bdata->dev_handle,
			bdata->command_transfers[i].buffer, dap->packet_size);
		oocd_libusb_dev_mem_free(bdata->dev_handle,
			bdata->response_transfers[i].buffer, dap->packet_size);
		bdata->command_transfers[i].buffer = NULL;
		bdata->response_transfers[i].buffer = NULL;
	}

	free(dap->packet_buffer);
	dap->packet_buffer = NULL;
	dap->command = NULL;
	dap->response = NULL;
}

static void cmsis_dap_usb_cancel_all(struct cmsis_dap *dap)
{
	for (unsigned int i = 0; i < MAX_PENDING_REQUESTS; i++) {
		if (dap->bdata->command_transfers[i].status == CMSIS_DAP_TRANSFER_PENDING)
			libusb_cancel_transfer(dap->bdata->command_transfers[i].transfer);
		if (dap->bdata->response_transfers[i].status == CMSIS_DAP_TRANSFER_PENDING)
			libusb_cancel_transfer(dap->bdata->response_transfers[i].transfer);

		dap->bdata->command_transfers[i].status = CMSIS_DAP_TRANSFER_IDLE;
		dap->bdata->response_transfers[i].status = CMSIS_DAP_TRANSFER_IDLE;
	}
}

COMMAND_HANDLER(cmsis_dap_handle_usb_interface_command)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], cmsis_dap_usb_interface);
	else
		LOG_ERROR("expected exactly one argument to cmsis_dap_usb_interface <interface_number>");

	return ERROR_OK;
}

const struct command_registration cmsis_dap_usb_subcommand_handlers[] = {
	{
		.name = "interface",
		.handler = &cmsis_dap_handle_usb_interface_command,
		.mode = COMMAND_CONFIG,
		.help = "set the USB interface number to use (for USB bulk backend only)",
		.usage = "<interface_number>",
	},
	COMMAND_REGISTRATION_DONE
};

const struct cmsis_dap_backend cmsis_dap_usb_backend = {
	.name = "usb_bulk",
	.open = cmsis_dap_usb_open,
	.close = cmsis_dap_usb_close,
	.read = cmsis_dap_usb_read,
	.write = cmsis_dap_usb_write,
	.packet_buffer_alloc = cmsis_dap_usb_alloc,
	.packet_buffer_free = cmsis_dap_usb_free,
	.cancel_all = cmsis_dap_usb_cancel_all,
};
