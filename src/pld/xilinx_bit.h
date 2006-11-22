/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
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
#ifndef XILINX_BIT_H
#define XILINX_BIT_H

#include "types.h"

typedef struct xilinx_bit_file_s
{
	u8 unknown_header[13];
	u8 *source_file;
	u8 *part_name;
	u8 *date;
	u8 *time;
	u32 length;
	u8 *data;
} xilinx_bit_file_t;

int xilinx_read_bit_file(xilinx_bit_file_t *bit_file, char *filename);

#endif /* XILINX_BIT_H */
