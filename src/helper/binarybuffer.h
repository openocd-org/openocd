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
static __inline void buf_set_u32(u8* buffer, unsigned int first, unsigned int num, u32 value)
{
	if ((num==32)&&(first==0))
	{
		buffer[3]=(value>>24)&0xff;
		buffer[2]=(value>>16)&0xff;
		buffer[1]=(value>>8)&0xff;
		buffer[0]=(value>>0)&0xff;
	} else
	{
		unsigned int i;
		
		for (i=first; i<first+num; i++)
		{
			if (((value >> (i-first))&1) == 1)
				buffer[i/8] |= 1 << (i%8);
			else
				buffer[i/8] &= ~(1 << (i%8));
		}
	}
}
static __inline u32 buf_get_u32(u8* buffer, unsigned int first, unsigned int num)
{
	if ((num==32)&&(first==0))
	{
		return (((u32)buffer[3])<<24)|(((u32)buffer[2])<<16)|(((u32)buffer[1])<<8)|(((u32)buffer[0])<<0);
	} else
	{
		u32 result = 0;
		unsigned int i;
		
		for (i=first; i<first+num; i++)
		{
			if (((buffer[i/8]>>(i%8))&1) == 1)
				result |= 1 << (i-first);
		}
	
		return result;
	}
}

extern u32 flip_u32(u32 value, unsigned int num);

extern int buf_cmp(u8 *buf1, u8 *buf2, int size);
extern int buf_cmp_mask(u8 *buf1, u8 *buf2, u8 *mask, int size);
extern u8* buf_cpy(u8 *from, u8 *to, int size);

extern u8* buf_set_ones(u8 *buf, int count);
extern u8* buf_set_buf(u8 *src, int src_start, u8 *dst, int dst_start, int len);

extern int str_to_buf(const char *str, int len, u8 *bin_buf, int buf_size, int radix);
extern char* buf_to_str(const u8 *buf, int size, int radix);

struct scan_field_s;
extern int buf_to_u32_handler(u8 *in_buf, void *priv, struct scan_field_s *field);

#define CEIL(m, n)	((m + n - 1) / n)

/* read a u32 from a buffer in target memory endianness */
static __inline u32 fast_target_buffer_get_u32(u8 *buffer, int little)
{
	if (little)
		return le_to_h_u32(buffer);
	else
		return be_to_h_u32(buffer);
}

#endif /* BINARYBUFFER_H */
