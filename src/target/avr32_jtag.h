/***************************************************************************
 *   Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>       *
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

#ifndef OPENOCD_TARGET_AVR32_JTAG_H
#define OPENOCD_TARGET_AVR32_JTAG_H

#define	AVR32NUMCOREREGS	17

/* tap instructions */
#define AVR32_INST_IDCODE		0x01
#define AVR32_INST_NEXUS_ACCESS	0x10
#define AVR32_INST_MW_ACCESS	0x11
#define AVR32_INST_MB_ACCESS	0x12

#define	SLAVE_OCD				0x01
#define	SLAVE_HSB_CACHED		0x04
#define	SLAVE_HSB_UNCACHED		0x05

/*
 * Registers
 */

#define AVR32_OCDREG_DID		0x00
#define AVR32_OCDREG_DC			0x02
#define		OCDREG_DC_SS			(1 <<  8)
#define		OCDREG_DC_DBR			(1 << 12)
#define		OCDREG_DC_DBE			(1 << 13)
#define		OCDREG_DC_SQA			(1 << 22)
#define		OCDREG_DC_RES			(1 << 30)
#define		OCDREG_DC_ABORT			(1 << 31)
#define AVR32_OCDREG_DS			0x04
#define		OCDREG_DS_SSS			(1 <<  0)
#define		OCDREG_DS_SWB			(1 <<  1)
#define		OCDREG_DS_HWB			(1 <<  2)
#define		OCDREG_DS_STP			(1 <<  4)
#define		OCDREG_DS_DBS			(1 <<  5)
#define		OCDREG_DS_BP_SHIFT		8
#define		OCDREG_DS_BP_MASK		0xff
#define		OCDREG_DS_INC			(1 << 24)
#define		OCDREG_DS_BOZ			(1 << 25)
#define		OCDREG_DS_DBA			(1 << 26)
#define		OCDREG_DS_EXB			(1 << 27)
#define		OCDREG_DS_NTBF			(1 << 28)

#define AVR32_OCDREG_DINST		0x41
#define AVR32_OCDREG_DPC		0x42
#define AVR32_OCDREG_DCCPU		0x44
#define AVR32_OCDREG_DCEMU		0x45
#define AVR32_OCDREG_DCSR		0x46
#define		OCDREG_DCSR_CPUD		(1 <<  0)
#define		OCDREG_DCSR_EMUD		(1 <<  1)

/*
 * Direction bit
 */
#define	MODE_WRITE				0x00
#define	MODE_READ				0x01

/*
 * Some instructions
 */

#define	RETD					0xd703d623
#define	MTDR(dreg, reg)			(0xe7b00044 | ((reg) << 16) | dreg)
#define	MFDR(reg, dreg)			(0xe5b00044 | ((reg) << 16) | dreg)
#define	MTSR(sysreg, reg)		(0xe3b00002 | ((reg) << 16) | sysreg)
#define	MFSR(reg, sysreg)		(0xe1b00002 | ((reg) << 16) | sysreg)

struct avr32_jtag {
	struct jtag_tap *tap;
	uint32_t dpc; /* Debug PC value */
};

int avr32_jtag_nexus_read(struct avr32_jtag *jtag_info,
		uint32_t addr, uint32_t *value);
int avr32_jtag_nexus_write(struct avr32_jtag *jtag_info,
		uint32_t addr, uint32_t value);

int avr32_jtag_mwa_read(struct avr32_jtag *jtag_info, int slave,
		uint32_t addr, uint32_t *value);
int avr32_jtag_mwa_write(struct avr32_jtag *jtag_info, int slave,
		uint32_t addr, uint32_t value);

int avr32_ocd_setbits(struct avr32_jtag *jtag, int reg, uint32_t bits);
int avr32_ocd_clearbits(struct avr32_jtag *jtag, int reg, uint32_t bits);

int avr32_jtag_exec(struct avr32_jtag *jtag_info, uint32_t inst);

#endif /* OPENOCD_TARGET_AVR32_JTAG_H */
