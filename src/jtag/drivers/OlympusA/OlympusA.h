/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
Simplified version of JTAG Control build in to Zeus in the Olympus Driver
Based on usb bluster
*/

#ifndef OPENOCD_JTAG_OLYMPUSA_H
#define OPENOCD_JTAG_OLYMPUSA_H


struct olympusa_lowlevel {
	HANDLE DeviceHandle;

	int (*write)(HANDLE DeviceHandle, uint8_t *buf, int size, uint32_t *bytes_written);
	int (*read)(HANDLE DeviceHandle, uint8_t *buf, unsigned size, uint32_t *bytes_read);
	int (*open)(HANDLE DeviceHandle);
	int (*close)(HANDLE DeviceHandle);
	int (*speed)(HANDLE DeviceHandle, int speed);

	void *priv;
	int flags;
};

extern struct olympusa_lowlevel *olympusa_lowlevel_register(void);

#endif
