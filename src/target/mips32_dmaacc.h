/***************************************************************************
 *   Copyright (C) 2008 by John McCarthy                                   *
 *   jgmcc@magma.ca                                                        *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
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
#ifndef MIPS32_DMAACC_H
#define MIPS32_DMAACC_H

#include "mips_ejtag.h"

#define EJTAG_CTRL_DMA_BYTE			0x00000000
#define EJTAG_CTRL_DMA_HALFWORD		0x00000080
#define EJTAG_CTRL_DMA_WORD			0x00000100
#define EJTAG_CTRL_DMA_TRIPLEBYTE	0x00000180

#define RETRY_ATTEMPTS	0

extern int mips32_dmaacc_read_mem(mips_ejtag_t *ejtag_info, u32 addr, int size, int count, void *buf);
extern int mips32_dmaacc_write_mem(mips_ejtag_t *ejtag_info, u32 addr, int size, int count, void *buf);

extern int mips32_dmaacc_read_mem8(mips_ejtag_t *ejtag_info, u32 addr, int count, u8 *buf);
extern int mips32_dmaacc_read_mem16(mips_ejtag_t *ejtag_info, u32 addr, int count, u16 *buf);
extern int mips32_dmaacc_read_mem32(mips_ejtag_t *ejtag_info, u32 addr, int count, u32 *buf);

extern int mips32_dmaacc_write_mem8(mips_ejtag_t *ejtag_info, u32 addr, int count, u8 *buf);
extern int mips32_dmaacc_write_mem16(mips_ejtag_t *ejtag_info, u32 addr, int count, u16 *buf);
extern int mips32_dmaacc_write_mem32(mips_ejtag_t *ejtag_info, u32 addr, int count, u32 *buf);

#endif
