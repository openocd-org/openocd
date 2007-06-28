/***************************************************************************
 *   Copyright (C) 2004, 2005 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
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
#ifndef TYPES_H
#define TYPES_H

#include "config.h"

#ifndef u8
typedef unsigned char u8;
#endif

#ifndef u16
typedef unsigned short u16;
#endif

#ifndef u32
typedef unsigned int u32;
#endif

#ifndef u64
typedef unsigned long long u64;
#endif

#ifdef WORDS_BIGENDIAN /* big endian host */

#define le_to_h_u32(x) (u32)((x)[0] | (x)[1] << 8 | (x)[2] << 16 | (x)[3] << 24)
#define le_to_h_u16(x) (u16)((x)[0] | (x)[1] << 8)
#define be_to_h_u32(x) (*(u32*)(x))
#define be_to_h_u16(x) (*(u16*)(x))

#define h_u32_to_le(buf, val) \
	do { \
		(buf)[3] = ((val) & 0xff000000) >> 24; \
		(buf)[2] = ((val) & 0x00ff0000) >> 16; \
		(buf)[1] = ((val) & 0x0000ff00) >> 8; \
		(buf)[0] = ((val) & 0x000000ff); \
	} while (0)

#define h_u16_to_le(buf, val)  \
	do { \
		(buf)[0] = ((val) & 0xff000) >> 8; \
		(buf)[1] = ((val) & 0x00ff); \
	} while (0)

#define h_u32_to_be(buf, val) do { *(u32*)(buf) = (val); } while (0)
#define h_u16_to_be(buf, val) do { *(u16*)(buf) = (val); } while (0)

#else /* little endian host */
#define le_to_h_u32(x) (*(u32*)(x))
#define le_to_h_u16(x) (*(u16*)(x))
#define be_to_h_u32(x) (u32)((x)[3] | (x)[2] << 8 | (x)[1] << 16 | (x)[0] << 24)
#define be_to_h_u16(x) (u16)((x)[1] | (x)[0] << 8)

#define h_u32_to_le(buf, val) do { *(u32*)(buf) = (val); } while (0)
#define h_u16_to_le(buf, val) do { *(u16*)(buf) = (val); } while (0)

#define h_u32_to_be(buf, val) \
	do { \
		(buf)[0] = ((val) & 0xff000000) >> 24; \
		(buf)[1] = ((val) & 0x00ff0000) >> 16; \
		(buf)[2] = ((val) & 0x0000ff00) >> 8; \
		(buf)[3] = ((val) & 0x000000ff); \
	} while (0)

#define h_u16_to_be(buf, val) \
	do { \
		(buf)[0] = ((val) & 0xff000) >> 8; \
		(buf)[1] = ((val) & 0x00ff); \
	} while (0)
#endif

#endif /* TYPES_H */
