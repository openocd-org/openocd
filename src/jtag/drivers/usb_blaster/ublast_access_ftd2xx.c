/*
 *   Driver for USB-JTAG, Altera USB-Blaster and compatibles
 *
 *   Inspired from original code from Kolja Waschk's USB-JTAG project
 *   (http://www.ixo.de/info/usb_jtag/), and from openocd project.
 *
 *   Copyright (C) 2012 Robert Jarzmik robert.jarzmik@free.fr
 *   Copyright (C) 2011 Ali Lown ali@lown.me.uk
 *   Copyright (C) 2009 Catalin Patulea cat@vv.carleton.ca
 *   Copyright (C) 2006 Kolja Waschk usbjtag@ixo.de
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 */

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include <jtag/interface.h>
#include <jtag/commands.h>

#include "ublast_access.h"

#include <ftd2xx.h>
#include "jtag/drivers/ftd2xx_common.h"

static FT_HANDLE *ublast_getftdih(struct ublast_lowlevel *low)
{
	return low->priv;
}

static int ublast_ftd2xx_write(struct ublast_lowlevel *low, uint8_t *buf, int size,
			      uint32_t *bytes_written)
{
	FT_STATUS status;
	DWORD dw_bytes_written;
	FT_HANDLE *ftdih = ublast_getftdih(low);

	status = FT_Write(*ftdih, buf, size, &dw_bytes_written);
	if (status != FT_OK) {
		*bytes_written = dw_bytes_written;
		LOG_ERROR("FT_Write returned: %s", ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}
	*bytes_written = dw_bytes_written;
	return ERROR_OK;
}

static int ublast_ftd2xx_read(struct ublast_lowlevel *low, uint8_t *buf,
			     unsigned size, uint32_t *bytes_read)
{
	DWORD dw_bytes_read;
	FT_STATUS status;
	FT_HANDLE *ftdih = ublast_getftdih(low);

	status = FT_Read(*ftdih, buf, size, &dw_bytes_read);
	if (status != FT_OK) {
		*bytes_read = dw_bytes_read;
		LOG_ERROR("FT_Read returned: %s", ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}
	*bytes_read = dw_bytes_read;
	return ERROR_OK;
}

static int ublast_ftd2xx_init(struct ublast_lowlevel *low)
{
	FT_STATUS status;
	FT_HANDLE *ftdih = ublast_getftdih(low);
	uint8_t latency_timer;

	LOG_INFO("usb blaster interface using FTD2XX");
	/* Open by device description */
	if (low->ublast_device_desc == NULL) {
		LOG_WARNING("no usb blaster device description specified, "
			    "using default 'USB-Blaster'");
		low->ublast_device_desc = "USB-Blaster";
	}

#if IS_WIN32 == 0
	/* Add non-standard Vid/Pid to the linux driver */
	status = FT_SetVIDPID(low->ublast_vid, low->ublast_pid);
	if (status != FT_OK) {
		LOG_WARNING("couldn't add %4.4x:%4.4x",
			    low->ublast_vid, low->ublast_pid);
	}
#endif
	status = FT_OpenEx(low->ublast_device_desc, FT_OPEN_BY_DESCRIPTION,
			   ftdih);
	if (status != FT_OK) {
		DWORD num_devices;

		LOG_ERROR("unable to open ftdi device: %s",
			  ftd2xx_status_string(status));
		status = FT_ListDevices(&num_devices, NULL, FT_LIST_NUMBER_ONLY);
		if (status == FT_OK) {
			char **desc_array =
				malloc(sizeof(char *) * (num_devices + 1));
			unsigned int i;

			for (i = 0; i < num_devices; i++)
				desc_array[i] = malloc(64);
			desc_array[num_devices] = NULL;

			status = FT_ListDevices(desc_array, &num_devices,
						FT_LIST_ALL | FT_OPEN_BY_DESCRIPTION);

			if (status == FT_OK) {
				LOG_ERROR("ListDevices: %" PRIu32, (uint32_t)num_devices);
				for (i = 0; i < num_devices; i++)
					LOG_ERROR("%i: %s", i, desc_array[i]);
			}

			for (i = 0; i < num_devices; i++)
				free(desc_array[i]);
			free(desc_array);
		} else {
			printf("ListDevices: NONE\n");
		}
		return ERROR_JTAG_INIT_FAILED;
	}

	status = FT_SetLatencyTimer(*ftdih, 2);
	if (status != FT_OK) {
		LOG_ERROR("unable to set latency timer: %s",
				ftd2xx_status_string(status));
		return ERROR_JTAG_INIT_FAILED;
	}

	status = FT_GetLatencyTimer(*ftdih, &latency_timer);
	if (status != FT_OK)
		LOG_ERROR("unable to get latency timer: %s",
				ftd2xx_status_string(status));
	else
		LOG_DEBUG("current latency timer: %i", latency_timer);

	status = FT_SetBitMode(*ftdih, 0x00, 0);
	if (status != FT_OK) {
		LOG_ERROR("unable to disable bit i/o mode: %s",
				ftd2xx_status_string(status));
		return ERROR_JTAG_INIT_FAILED;
	}
	return ERROR_OK;
}

static int ublast_ftd2xx_quit(struct ublast_lowlevel *low)
{
	FT_HANDLE *ftdih = ublast_getftdih(low);

	FT_Close(*ftdih);
	return ERROR_OK;
}

static struct ublast_lowlevel_priv {
	FT_HANDLE ftdih;
} info;

static struct ublast_lowlevel low = {
	.open = ublast_ftd2xx_init,
	.close = ublast_ftd2xx_quit,
	.read = ublast_ftd2xx_read,
	.write = ublast_ftd2xx_write,
	.priv = &info,
};

struct ublast_lowlevel *ublast_register_ftd2xx(void)
{
	return &low;
}
