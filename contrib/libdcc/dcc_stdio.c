/***************************************************************************
 *   Copyright (C) 2008 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

#include "dcc_stdio.h"

#if defined(__ARM_ARCH_7M__)

/* we use the cortex_m3 DCRDR reg to simulate a arm7_9 dcc channel
 * DCRDR[7:0] is used by target for status
 * DCRDR[15:8] is used by target for write buffer
 * DCRDR[23:16] is used for by host for status
 * DCRDR[31:24] is used for by host for write buffer */

#define DCRDR_WRSTS	*((volatile u8*)0xE000EDF8)
#define DCRDR_WRDAT	*((volatile u8*)0xE000EDF9)

#define	BUSY	1

void dbg_write(u32 dcc_data)
{
	int len = 4;
	
	while (len--)
	{
		/* wait for data ready */
		while (DCRDR_WRSTS & BUSY);
		
		/* write our data */
		DCRDR_WRDAT = (u8)(dcc_data & 0xff);
		/* set write flag - tell host there is data */
		DCRDR_WRSTS = BUSY;
		dcc_data >>= 8;
	}
}

#elif defined(__ARM_ARCH_4T__) || defined(__ARM_ARCH_5TE__)
					
void dbg_write(u32 dcc_data)
{
	u32 dcc_status;
	
	do {
		asm volatile("mrc p14, 0, %0, c0, c0" : "=r" (dcc_status));
	} while (dcc_status & 0x2);
	
	asm volatile("mcr p14, 0, %0, c1, c0" : : "r" (dcc_data));
}

#else
 #error unsupported target
#endif


void dbg_write_u32(u32 *val, u32 len)
{	
	dbg_write(0x01 | 0x0400 | ((len & 0xffff) << 16));

	while (len > 0)
	{
		dbg_write(*val);
		
		val++;
		len--;
	}
}

void dbg_write_u16(u16 *val, u32 len)
{
	u32 dcc_data;
		
	dbg_write(0x01 | 0x0200 | ((len & 0xffff) << 16));

	while (len > 0)
	{
		dcc_data = val[0] | (val[1] << 8)
			| ((len > 1) ? (val[2] | (val[3] << 8)) << 16 : 0x00);
		
		dbg_write(dcc_data);
		
		val += 2;
		len -= 2;
	}
}

void dbg_write_u8(u8 *val, u32 len)
{	
	u32 dcc_data;

	dbg_write(0x01 | 0x0100 | ((len & 0xffff) << 16));

	while (len > 0)
	{
		dcc_data = val[0]
			| ((len > 1) ? val[1] << 8 : 0x00)
			| ((len > 2) ? val[2] << 16 : 0x00)
			| ((len > 3) ? val[3] << 24 : 0x00);
		
		dbg_write(dcc_data);
		
		val += 2;
		len -= 2;
	}
}

void dbg_write_str(u8 *msg)
{
	int len;
	u32 dcc_data;
	
	for (len = 0; msg[len] && (len < 65536); len++);
	
	dbg_write(0x01 | ((len & 0xffff) << 16));
	
	while (len > 0)
	{		
		dcc_data = msg[0]
			| ((len > 1) ? msg[1] << 8 : 0x00)
			| ((len > 2) ? msg[2] << 16 : 0x00)
			| ((len > 3) ? msg[3] << 24 : 0x00);
		dbg_write(dcc_data);
		
		msg += 4;
		len -= 4;
	}
}

void dbg_write_char(u8 msg)
{	
	dbg_write(0x02 | ((msg & 0xff) << 16));
}
