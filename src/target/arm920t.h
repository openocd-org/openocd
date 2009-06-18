/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
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
#ifndef ARM920T_H
#define ARM920T_H

#include "arm9tdmi.h"
#include "armv4_5_mmu.h"

#define	ARM920T_COMMON_MAGIC 0xa920a920

typedef struct arm920t_common_s
{
	uint32_t common_magic;
	armv4_5_mmu_common_t armv4_5_mmu;
	arm9tdmi_common_t arm9tdmi_common;
	uint32_t cp15_control_reg;
	uint32_t d_fsr;
	uint32_t i_fsr;
	uint32_t d_far;
	uint32_t i_far;
	int preserve_cache;
} arm920t_common_t;

typedef struct arm920t_cache_line_s
{
	uint32_t cam;
	uint32_t data[8];
} arm920t_cache_line_t;

typedef struct arm920t_tlb_entry_s
{
	uint32_t cam;
	uint32_t ram1;
	uint32_t ram2;
} arm920t_tlb_entry_t;

#endif /* ARM920T_H */
