/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2008 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifndef DCC_STDIO_H
#define DCC_STDIO_H

void dbg_trace_point(unsigned long number);

void dbg_write_u32(const unsigned long *val, long len);
void dbg_write_u16(const unsigned short *val, long len);
void dbg_write_u8(const unsigned char *val, long len);

void dbg_write_str(const char *msg);
void dbg_write_char(char msg);

#endif	/* DCC_STDIO_H */
