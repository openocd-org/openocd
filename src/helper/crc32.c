// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2013-2014 by Franck Jullien                             *
 *   elec4fun@gmail.com                                                    *
 *                                                                         *
 *   Copyright (C) 2022 Otto-von-Guericke-Universität Magdeburg            *
 *   marian.buschsieweke@ovgu.de                                           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "crc32.h"
#include <stdint.h>
#include <stddef.h>

static uint32_t crc_le_step(uint32_t poly, uint32_t crc, uint32_t data_in,
		unsigned int data_bits)
{
	for (unsigned int i = 0; i < data_bits; i++) {
		uint32_t d, c;
		d = ((data_in >> i) & 0x1) ? 0xffffffff : 0;
		c = (crc & 0x1) ? 0xffffffff : 0;
		crc = crc >> 1;
		crc = crc ^ ((d ^ c) & poly);
	}

	return crc;
}

uint32_t crc32_le(uint32_t poly, uint32_t seed, const void *_data,
		size_t data_len)
{
	if (((uintptr_t)_data & 0x3) || (data_len & 0x3)) {
		/* data is unaligned, processing data one byte at a time */
		const uint8_t *data = _data;
		for (size_t i = 0; i < data_len; i++)
			seed = crc_le_step(poly, seed, data[i], 8);
	} else {
		/* data is aligned, processing 32 bit at a time */
		data_len >>= 2;
		const uint32_t *data = _data;
		for (size_t i = 0; i < data_len; i++)
			seed = crc_le_step(poly, seed, data[i], 32);
	}

	return seed;
}
