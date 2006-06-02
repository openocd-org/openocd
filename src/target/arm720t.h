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
#ifndef ARM720T_H
#define ARM720T_H

#include "target.h"
#include "register.h"
#include "embeddedice.h"
#include "arm_jtag.h"
#include "arm7tdmi.h"
#include "armv4_5_mmu.h"
#include "armv4_5_cache.h"

#define	ARM720T_COMMON_MAGIC 0xa720a720

typedef struct arm720t_common_s
{
	int common_magic;
	armv4_5_mmu_common_t armv4_5_mmu;
	arm7tdmi_common_t arm7tdmi_common;
	u32 cp15_control_reg;
	u32 fsr;
	u32 far;
} arm720t_common_t;

#endif /* ARM720T_H */
