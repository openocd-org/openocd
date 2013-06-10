/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef INCLUDED_RTOS_STANDARD_STACKINGS_H_
#define INCLUDED_RTOS_STANDARD_STACKINGS_H_

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"

extern const struct rtos_register_stacking rtos_standard_Cortex_M3_stacking;
extern const struct rtos_register_stacking rtos_standard_Cortex_R4_stacking;
extern const struct rtos_register_stacking rtos_standard_NDS32_N1068_stacking;

#endif	/* ifndef INCLUDED_RTOS_STANDARD_STACKINGS_H_ */
