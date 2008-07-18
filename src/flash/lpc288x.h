/***************************************************************************
 *   Copyright (C) 2008 by			                                       *
 *   Karl RobinSod <karl.robinsod@gmail.com>                               *
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

#ifndef lpc288x_H
#define lpc288x_H

#include "flash.h"
#include "target.h"

typedef struct lpc288x_flash_bank_s
{
	u32 working_area;
	u32 working_area_size;

	/* chip id register */
	u32 cidr;
	char * target_name;
	u32 cclk;

	u32 sector_size_break;
} lpc288x_flash_bank_t;

#endif /* lpc288x_H */
