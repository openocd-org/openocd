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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#define REG_R0 0
#define REG_R1 1
#define REG_R2 2
#define REG_R3 3
#define REG_R4 4
#define REG_R5 5
#define REG_R6 6
#define REG_R7 7
#define REG_R8 8
#define REG_R9 9
#define REG_R10 10
#define REG_R11 11
#define REG_R12 12
#define REG_R13 13
#define REG_R14 14
#define REG_R15 15
#define REG_CPSR 16
#define REG_SPSR 17

#define MODE_USR 0x10
#define MODE_FIQ 0x11
#define MODE_IRQ 0x12
#define MODE_SVC 0x13
#define MODE_ABT 0x17
#define MODE_UND 0x1b
#define MODE_SYS 0x1f

#define MODE_ANY 0x40
#define MODE_CURRENT 0x80

#define MODE_MASK 0x1f
#define PSR_I 0x80
#define PSR_F 0x40
#define PSR_T 0x20

#define XSCALE_DBG_MAINID 0x0
#define XSCALE_DBG_CACHETYPE 0x1
#define XSCALE_DBG_CTRL 0x2
#define XSCALE_DBG_AUXCTRL 0x3
#define XSCALE_DBG_TTB 0x4
#define XSCALE_DBG_DAC 0x5
#define XSCALE_DBG_FSR 0x6
#define XSCALE_DBG_FAR 0x7
#define XSCALE_DBG_PID 0x8
#define XSCALE_DBG_CPACCESS 0x9
#define XSCALE_DBG_IBCR0 0xa
#define XSCALE_DBG_IBCR1 0xb
#define XSCALE_DBG_DBR0 0xc
#define XSCALE_DBG_DBR1 0xd
#define XSCALE_DBG_DBCON 0xe
