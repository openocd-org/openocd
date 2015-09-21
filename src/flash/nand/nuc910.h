/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

/*
 * NAND controller interface for Nuvoton NUC910
 */

#ifndef OPENOCD_FLASH_NAND_NUC910_H
#define OPENOCD_FLASH_NAND_NUC910_H

#define NUC910_FMICSR	0xB000D000
#define NUC910_SMCSR	0xB000D0A0
#define NUC910_SMTCR	0xB000D0A4
#define NUC910_SMIER	0xB000D0A8
#define NUC910_SMISR	0xB000D0AC
#define NUC910_SMCMD	0xB000D0B0
#define NUC910_SMADDR	0xB000D0B4
#define NUC910_SMDATA	0xB000D0B8

#define NUC910_SMECC0	0xB000D0BC
#define NUC910_SMECC1	0xB000D0C0
#define NUC910_SMECC2	0xB000D0C4
#define NUC910_SMECC3	0xB000D0C8
#define NUC910_ECC4ST	0xB000D114

/* Global Control and Status Register (FMICSR) */
#define NUC910_FMICSR_SM_EN	(1<<3)

/* NAND Flash Address Port Register (SMADDR) */
#define NUC910_SMADDR_EOA (1<<31)

/* NAND Flash Control and Status Register (SMCSR) */
#define NUC910_SMCSR_PSIZE	(1<<3)
#define NUC910_SMCSR_DBW	(1<<4)

/* NAND Flash Interrupt Status Register (SMISR) */
#define NUC910_SMISR_ECC_IF	(1<<2)
#define NUC910_SMISR_RB_	(1<<18)

/* ECC4 Correction Status (ECC4ST) */

#endif /* OPENOCD_FLASH_NAND_NUC910_H */
