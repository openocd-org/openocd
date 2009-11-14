/***************************************************************************
 *   Copyright (C) 2004, 2005 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef BINARYBUFFER_H
#define BINARYBUFFER_H

#include "types.h"

/* support functions to access arbitrary bits in a byte array
 * flip_u32 inverses the bit order inside a 32-bit word (31..0 -> 0..31)
 */

/* inlining this will help show what fn that is taking time during profiling. */
static inline void buf_set_u32(uint8_t* buffer,
		unsigned int first, unsigned int num, uint32_t value)
{
	if ((num == 32) && (first == 0))
	{
		buffer[3]=(value >> 24)&0xff;
		buffer[2]=(value >> 16)&0xff;
		buffer[1]=(value >> 8)&0xff;
		buffer[0]=(value >> 0)&0xff;
	} else
	{
		unsigned int i;

		for (i = first; i < first + num; i++)
		{
			if (((value >> (i-first))&1) == 1)
				buffer[i/8] |= 1 << (i%8);
			else
				buffer[i/8] &= ~(1 << (i%8));
		}
	}
}
static inline uint32_t buf_get_u32(const uint8_t* buffer,
		unsigned int first, unsigned int num)
{
	if ((num == 32) && (first == 0))
	{
		return (((uint32_t)buffer[3]) << 24) |
			(((uint32_t)buffer[2]) << 16) |
			(((uint32_t)buffer[1]) << 8) |
			(((uint32_t)buffer[0]) << 0);
	} else {
		uint32_t result = 0;
		unsigned int i;

		for (i = first; i < first + num; i++)
		{
			if (((buffer[i/8]>>(i%8))&1) == 1)
				result |= 1 << (i-first);
		}

		return result;
	}
}

uint32_t flip_u32(uint32_t value, unsigned int num);

int buf_cmp(const uint8_t *buf1, const uint8_t *buf2, int size);
int buf_cmp_mask(const uint8_t *buf1, const uint8_t *buf2,
		const uint8_t *mask, int size);
uint8_t* buf_cpy(const uint8_t *from, uint8_t *to, int size);

uint8_t* buf_set_ones(uint8_t *buf, int count);
uint8_t* buf_set_buf(const uint8_t *src, int src_start,
		uint8_t *dst, int dst_start, int len);

int str_to_buf(const char *str, int len,
		uint8_t *bin_buf, int buf_size, int radix);
char* buf_to_str(const uint8_t *buf, int size, int radix);

struct scan_field;
int buf_to_u32_handler(uint8_t *in_buf, void *priv, struct scan_field *field);

#define CEIL(m, n)	(((m) + (n) - 1) / (n))

/* read a uint32_t from a buffer in target memory endianness */
static inline uint32_t fast_target_buffer_get_u32(const uint8_t *p, int le)
{
	return le ? le_to_h_u32(p) : be_to_h_u32(p);
}

#endif /* BINARYBUFFER_H */
