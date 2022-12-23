/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2010 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
