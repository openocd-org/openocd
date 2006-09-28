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
#ifndef BINARYBUFFER_H
#define BINARYBUFFER_H

#include "types.h"

/* support functions to access arbitrary bits in a byte array
 * flip_u32 inverses the bit order inside a 32-bit word (31..0 -> 0..31)
 */

extern int buf_set_u32(u8* buffer, unsigned int first, unsigned int num, u32 value);
extern u32 buf_get_u32(u8* buffer, unsigned int first, unsigned int num);

extern u32 flip_u32(u32 value, unsigned int num);

extern int buf_cmp(u8 *buf1, u8 *buf2, int size);
extern int buf_cmp_mask(u8 *buf1, u8 *buf2, u8 *mask, int size);
extern u8* buf_cpy(u8 *from, u8 *to, int size);

extern u8* buf_set_ones(u8 *buf, int count);
extern u8* buf_set_buf(u8 *src, int src_start, u8 *dst, int dst_start, int len);

extern int str_to_buf(char* str, int len, u8 *bin_buf, int buf_size, int radix);
extern char* buf_to_str(u8 *buf, int size, int radix);

extern int buf_to_u32_handler(u8 *in_buf, void *priv);

#define CEIL(m, n)	((m + n - 1) / n)

#endif /* BINARYBUFFER_H */
