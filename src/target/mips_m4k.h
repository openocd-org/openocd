/***************************************************************************
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

#ifndef MIPS_M4K_H
#define MIPS_M4K_H

#include <helper/types.h>

struct target;

#define MIPSM4K_COMMON_MAGIC	0xB321B321

struct mips_m4k_common
{
	int common_magic;
	bool is_pic32mx;
	struct mips32_common mips32;
};

static inline struct mips_m4k_common *
target_to_m4k(struct target *target)
{
	return container_of(target->arch_info,
			struct mips_m4k_common, mips32);
}

#endif	/*MIPS_M4K_H*/
