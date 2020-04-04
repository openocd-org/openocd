/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2019, Ampere Computing LLC                              *
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

#ifndef OPENOCD_TARGET_ARM_ADI_V6_H
#define OPENOCD_TARGET_ARM_ADI_V6_H

/**
 * @file
 * This defines formats and data structures used to talk to ADIv6 entities.
 * Those include a DAP, different types of Debug Port (DP), and memory mapped
 * resources accessed through a MEM-AP.
 */

#include <helper/list.h>
/*#include "arm_jtag.h"*/

/* A[3:0] for DP registers; A[1:0] are always zero.
 * - JTAG accesses all of these via JTAG_DP_DPACC, except for
 *   IDCODE (JTAG_DP_IDCODE) and ABORT (JTAG_DP_ABORT).
 * - SWD accesses these directly, sometimes needing SELECT.DPBANKSEL
 *   See arm_adi.h for remaining DP regs
 */
#define DP_BASEPTR0     BANK_REG(0x2, 0x0) /* DPv3: ro */
#define DP_BASEPTR1     BANK_REG(0x3, 0x0) /* DPv3: ro */
#define DP_SELECT1      BANK_REG(0x5, 0x4) /* DPv3: ro */


/* MEM-AP register addresses */
#define MEM_AP_REG_DARx		0x000
#define MEM_AP_REG_CSW		0xD00
#define MEM_AP_REG_TAR		0xD04
#define MEM_AP_REG_TAR_UPPER	0xD08		/* RW: Large Physical Address Extension */
#define MEM_AP_REG_DRW		0xD0C		/* RW: Data Read/Write register */
#define MEM_AP_REG_BD0		0xD10		/* RW: Banked Data register 0-3 */
#define MEM_AP_REG_BD1		0xD14
#define MEM_AP_REG_BD2		0xD18
#define MEM_AP_REG_BD3		0xD1C
#define MEM_AP_REG_MBT		0xD20		/* --: Memory Barrier Transfer register */
#define MEM_AP_REG_BASE_UPPER	0xDF0		/* RO: Debug Base Address (LA) register */
#define MEM_AP_REG_CFG		0xDF4		/* RO: Configuration register */
#define MEM_AP_REG_BASE		0xDF8		/* RO: Debug Base Address register */
/* Generic AP register address */
#define AP_REG_IDR		0xDFC		/* RO: Identification Register */
#define AP_REG_DEVARCH		0xFBC		/* RO: Present bit 20 should be on and ARCHID (bits 15:0) */
#define AP_REG_DEVID		0xFC8		/* RO: Device ID Format (bits 3:0) indicates ROM table entry size  */
#define AP_REG_CIDR0		0xFF0		/* RO: Component ID0 */
#define AP_REG_CIDR1		0xFF4		/* RO: Component ID1 with Class identification */
#define AP_REG_CIDR2		0xFF8		/* RO: Component ID2 */
#define AP_REG_CIDR3		0xFFC		/* RO: Component ID3 */


#endif /* OPENOCD_TARGET_ARM_ADI_V6_H */
