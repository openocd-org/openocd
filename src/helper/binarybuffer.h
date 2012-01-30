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

#include <helper/types.h>

/** @file
 * Support functions to access arbitrary bits in a byte array
 */

/**
 * Sets @c num bits in @c _buffer, starting at the @c first bit,
 * using the bits in @c value.  This routine fast-paths writes
 * of little-endian, byte-aligned, 32-bit words.
 * @param _buffer The buffer whose bits will be set.
 * @param first The bit offset in @c _buffer to start writing (0-31).
 * @param num The number of bits from @c value to copy (1-32).
 * @param value Up to 32 bits that will be copied to _buffer.
 */
static inline void buf_set_u32(void *_buffer,
	unsigned first, unsigned num, uint32_t value)
{
	uint8_t *buffer = (uint8_t *)_buffer;

	if ((num == 32) && (first == 0)) {
		buffer[3] = (value >> 24) & 0xff;
		buffer[2] = (value >> 16) & 0xff;
		buffer[1] = (value >> 8) & 0xff;
		buffer[0] = (value >> 0) & 0xff;
	} else {
		for (unsigned i = first; i < first + num; i++) {
			if (((value >> (i - first)) & 1) == 1)
				buffer[i / 8] |= 1 << (i % 8);
			else
				buffer[i / 8] &= ~(1 << (i % 8));
		}
	}
}
/**
 * Retrieves @c num bits from @c _buffer, starting at the @c first bit,
 * returning the bits in a 32-bit word.  This routine fast-paths reads
 * of little-endian, byte-aligned, 32-bit words.
 * @param _buffer The buffer whose bits will be read.
 * @param first The bit offset in @c _buffer to start reading (0-31).
 * @param num The number of bits from @c _buffer to read (1-32).
 * @returns Up to 32-bits that were read from @c _buffer.
 */
static inline uint32_t buf_get_u32(const void *_buffer,
	unsigned first, unsigned num)
{
	uint8_t *buffer = (uint8_t *)_buffer;

	if ((num == 32) && (first == 0)) {
		return (((uint32_t)buffer[3]) << 24) |
				(((uint32_t)buffer[2]) << 16) |
				(((uint32_t)buffer[1]) << 8) |
				(((uint32_t)buffer[0]) << 0);
	} else {
		uint32_t result = 0;
		for (unsigned i = first; i < first + num; i++) {
			if (((buffer[i / 8] >> (i % 8)) & 1) == 1)
				result |= 1 << (i - first);
		}
		return result;
	}
}

/**
 * Inverts the ordering of bits inside a 32-bit word (e.g. 31..0 -> 0..31).
 * This routine can be used to flip smaller data types by using smaller
 * values for @c width.
 * @param value The word to flip.
 * @param width The number of bits in value (2-32).
 * @returns A 32-bit word with @c value in reversed bit-order.
 */
uint32_t flip_u32(uint32_t value, unsigned width);

bool buf_cmp(const void *buf1, const void *buf2, unsigned size);
bool buf_cmp_mask(const void *buf1, const void *buf2,
		const void *mask, unsigned size);

/**
 * Copies @c size bits out of @c from and into @c to.  Any extra
 * bits in the final byte will be set to zero.
 * @param from The buffer to copy into @c to.
 * @param to The buffer that will receive the copy of @c from.
 * @param size The number of bits to copy.
 */
void *buf_cpy(const void *from, void *to, unsigned size);

/**
 * Set the contents of @c buf with @c count bits, all set to 1.
 * @param buf The buffer to fill with ones.
 * @param size The number of bits.
 * @returns The original buffer (@c buf).
 */
void *buf_set_ones(void *buf, unsigned size);

void *buf_set_buf(const void *src, unsigned src_start,
		  void *dst, unsigned dst_start, unsigned len);

int str_to_buf(const char *str, unsigned len,
		void *bin_buf, unsigned buf_size, unsigned radix);
char *buf_to_str(const void *buf, unsigned size, unsigned radix);

/* read a uint32_t from a buffer in target memory endianness */
static inline uint32_t fast_target_buffer_get_u32(const void *p, bool le)
{
	return le ? le_to_h_u32(p) : be_to_h_u32(p);
}

#endif	/* BINARYBUFFER_H */
