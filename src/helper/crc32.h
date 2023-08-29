/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2022 Otto-von-Guericke-Universität Magdeburg            *
 *   marian.buschsieweke@ovgu.de                                           *
 ***************************************************************************/

#ifndef OPENOCD_HELPER_CRC32_H
#define OPENOCD_HELPER_CRC32_H

#include <stdint.h>
#include <stddef.h>

/** @file
 * A generic CRC32 implementation
 */

/**
 * CRC32 polynomial commonly used for little endian CRC32
 */
#define CRC32_POLY_LE	0xedb88320

/**
 * Calculate the CRC32 value of the given data
 * @param	poly		The polynomial of the CRC
 * @param	seed		The seed to use (mostly either `0` or `0xffffffff`)
 * @param	data		The data to calculate the CRC32 of
 * @param	data_len	The length of the data in @p data in bytes
 * @return	The CRC value of the first @p data_len bytes at @p data
 * @note	This function can be used to incrementally compute the CRC one
 *			chunk of data at a time by using the CRC32 of the previous chunk
 *			as @p seed for the next chunk.
 */
uint32_t crc32_le(uint32_t poly, uint32_t seed, const void *data,
		size_t data_len);

#endif /* OPENOCD_HELPER_CRC32_H */
