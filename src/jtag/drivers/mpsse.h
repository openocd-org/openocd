/**************************************************************************
 *   Copyright (C) 2012 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_DRIVERS_MPSSE_H
#define OPENOCD_JTAG_DRIVERS_MPSSE_H

#include <stdbool.h>
#include "helper/binarybuffer.h"

/* Mode flags */
#define POS_EDGE_OUT 0x00
#define NEG_EDGE_OUT 0x01
#define POS_EDGE_IN 0x00
#define NEG_EDGE_IN 0x04
#define MSB_FIRST 0x00
#define LSB_FIRST 0x08

enum ftdi_chip_type {
	TYPE_FT2232C,
	TYPE_FT2232H,
	TYPE_FT4232H,
	TYPE_FT232H,
};

struct mpsse_ctx;

/* Device handling */
struct mpsse_ctx *mpsse_open(const uint16_t *vid, const uint16_t *pid, const char *description,
	const char *serial, const char *location, int channel);
void mpsse_close(struct mpsse_ctx *ctx);
bool mpsse_is_high_speed(struct mpsse_ctx *ctx);

/* Command queuing. These correspond to the MPSSE commands with the same names, but no need to care
 * about bit/byte transfer or data length limitation. Read data is guaranteed to be available only
 * after the following mpsse_flush(). */
void mpsse_clock_data_out(struct mpsse_ctx *ctx, const uint8_t *out, unsigned out_offset,
			 unsigned length, uint8_t mode);
void mpsse_clock_data_in(struct mpsse_ctx *ctx, uint8_t *in, unsigned in_offset, unsigned length,
			uint8_t mode);
void mpsse_clock_data(struct mpsse_ctx *ctx, const uint8_t *out, unsigned out_offset, uint8_t *in,
		     unsigned in_offset, unsigned length, uint8_t mode);
void mpsse_clock_tms_cs_out(struct mpsse_ctx *ctx, const uint8_t *out, unsigned out_offset,
			   unsigned length, bool tdi, uint8_t mode);
void mpsse_clock_tms_cs(struct mpsse_ctx *ctx, const uint8_t *out, unsigned out_offset, uint8_t *in,
		       unsigned in_offset, unsigned length, bool tdi, uint8_t mode);
void mpsse_set_data_bits_low_byte(struct mpsse_ctx *ctx, uint8_t data, uint8_t dir);
void mpsse_set_data_bits_high_byte(struct mpsse_ctx *ctx, uint8_t data, uint8_t dir);
void mpsse_read_data_bits_low_byte(struct mpsse_ctx *ctx, uint8_t *data);
void mpsse_read_data_bits_high_byte(struct mpsse_ctx *ctx, uint8_t *data);
void mpsse_loopback_config(struct mpsse_ctx *ctx, bool enable);
void mpsse_set_divisor(struct mpsse_ctx *ctx, uint16_t divisor);
int mpsse_divide_by_5_config(struct mpsse_ctx *ctx, bool enable);
int mpsse_rtck_config(struct mpsse_ctx *ctx, bool enable);

/* Helper to set frequency in Hertz. Returns actual realizable frequency or negative error.
 * Frequency 0 means RTCK. */
int mpsse_set_frequency(struct mpsse_ctx *ctx, int frequency);

/* Queue handling */
int mpsse_flush(struct mpsse_ctx *ctx);
void mpsse_purge(struct mpsse_ctx *ctx);

#endif /* OPENOCD_JTAG_DRIVERS_MPSSE_H */
