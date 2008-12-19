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
#ifndef TYPES_H
#define TYPES_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

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

typedef struct jtag_tap_s jtag_tap_t;


/* DANGER!!!! here be dragons! 
 * 
 * Leave these fn's as byte accesses because it is safe
 * across architectures. Clever usage of 32 bit access
 * will create problems on some hosts.
 * 
 * Note that the pointer in memory might be unaligned. 
 * 
 * On some CPU's, i.e. ARM7, the 2 lsb are ignored for 32 
 * bit access, on others it will cause an exception and 
 * on e.g. x86, it works the same as if aligned.
 * 
 */


static inline u32 le_to_h_u32(const u8* buf)
{
	return (u32)(buf[0] | buf[1] << 8 | buf[2] << 16 | buf[3] << 24);
}

static inline u16 le_to_h_u16(const u8* buf)
{
	return (u16)(buf[0] | buf[1] << 8);
}

static inline u32 be_to_h_u32(const u8* buf)
{
	return (u32)(buf[3] | buf[2] << 8 | buf[1] << 16 | buf[0] << 24);
}

static inline u16 be_to_h_u16(const u8* buf)
{
	return (u16)(buf[1] | buf[0] << 8);
}

static inline void h_u32_to_le(u8* buf, int val)
{
	buf[3] = (u8) (val >> 24);
	buf[2] = (u8) (val >> 16);
	buf[1] = (u8) (val >> 8);
	buf[0] = (u8) (val >> 0);
}

static inline void h_u32_to_be(u8* buf, int val)
{
	buf[0] = (u8) (val >> 24);
	buf[1] = (u8) (val >> 16);
	buf[2] = (u8) (val >> 8);
	buf[3] = (u8) (val >> 0);
}

static inline void h_u16_to_le(u8* buf, int val)
{
	buf[1] = (u8) (val >> 8);
	buf[0] = (u8) (val >> 0);
}

static inline void h_u16_to_be(u8* buf, int val)
{
	buf[0] = (u8) (val >> 8);
	buf[1] = (u8) (val >> 0);
}

#endif /* TYPES_H */
