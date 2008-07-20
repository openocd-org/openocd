/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
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
#ifndef NON_CFI_H
#define NON_CFI_H

#include "types.h"

typedef struct non_cfi_s
{
	u16 mfr;
	u16 id;
	u16 pri_id;
	u8 dev_size;
	u16 interface_desc;
	u16 max_buf_write_size;
	u8 num_erase_regions;
	u32 erase_region_info[6];
} non_cfi_t;

extern non_cfi_t non_cfi_flashes[];
extern void cfi_fixup_non_cfi(flash_bank_t *bank, void *param);

#endif /* NON_CFI_H */
