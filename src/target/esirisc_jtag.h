/***************************************************************************
 *   Copyright (C) 2018 by Square, Inc.                                    *
 *   Steven Stallion <stallion@squareup.com>                               *
 *   James Zhao <hjz@squareup.com>                                         *
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

#ifndef OPENOCD_TARGET_ESIRISC_JTAG_H
#define OPENOCD_TARGET_ESIRISC_JTAG_H

#include <helper/types.h>
#include <jtag/jtag.h>

/* TAP Instructions */
#define INSTR_IDCODE			0x8
#define INSTR_DEBUG				0x9
#define INSTR_BYPASS			0xf
#define INSTR_LENGTH			4

/* eSi-Debug Commands */
#define DEBUG_NOP				0x00
#define DEBUG_READ_BYTE			0x10
#define DEBUG_READ_HWORD		0x20
#define DEBUG_READ_WORD			0x30
#define DEBUG_WRITE_BYTE		0x60
#define DEBUG_WRITE_HWORD		0x70
#define DEBUG_WRITE_WORD		0x80
#define DEBUG_READ_REG			0xb0
#define DEBUG_WRITE_REG			0xc0
#define DEBUG_READ_CSR			0xd0
#define DEBUG_WRITE_CSR			0xe0
#define DEBUG_ENABLE_DEBUG		0xf0
#define DEBUG_DISABLE_DEBUG		0xf2
#define DEBUG_ASSERT_RESET		0xf4
#define DEBUG_DEASSERT_RESET	0xf6
#define DEBUG_BREAK				0xf8
#define DEBUG_CONTINUE			0xfa
#define DEBUG_FLUSH_CACHES		0xfc

/* Exception IDs */
#define EID_OVERFLOW			0x3d
#define EID_CANT_DEBUG			0x3e
#define EID_NONE				0x3f

/* Byte Stuffing */
#define STUFF_MARKER			0x55
#define PAD_BYTE				0xaa

struct esirisc_jtag {
	struct jtag_tap *tap;
	uint8_t status;
};

bool esirisc_jtag_is_debug_active(struct esirisc_jtag *jtag_info);
bool esirisc_jtag_is_stopped(struct esirisc_jtag *jtag_info);
uint8_t esirisc_jtag_get_eid(struct esirisc_jtag *jtag_info);

int esirisc_jtag_read_byte(struct esirisc_jtag *jtag_info,
		uint32_t address, uint8_t *data);
int esirisc_jtag_read_hword(struct esirisc_jtag *jtag_info,
		uint32_t address, uint16_t *data);
int esirisc_jtag_read_word(struct esirisc_jtag *jtag_info,
		uint32_t address, uint32_t *data);

int esirisc_jtag_write_byte(struct esirisc_jtag *jtag_info,
		uint32_t address, uint8_t data);
int esirisc_jtag_write_hword(struct esirisc_jtag *jtag_info,
		uint32_t address, uint16_t data);
int esirisc_jtag_write_word(struct esirisc_jtag *jtag_info,
		uint32_t address, uint32_t data);

int esirisc_jtag_read_reg(struct esirisc_jtag *jtag_info,
		uint8_t reg, uint32_t *data);
int esirisc_jtag_write_reg(struct esirisc_jtag *jtag_info,
		uint8_t reg, uint32_t data);

int esirisc_jtag_read_csr(struct esirisc_jtag *jtag_info,
		uint8_t bank, uint8_t csr, uint32_t *data);
int esirisc_jtag_write_csr(struct esirisc_jtag *jtag_info,
		uint8_t bank, uint8_t csr, uint32_t data);

int esirisc_jtag_enable_debug(struct esirisc_jtag *jtag_info);
int esirisc_jtag_disable_debug(struct esirisc_jtag *jtag_info);

int esirisc_jtag_assert_reset(struct esirisc_jtag *jtag_info);
int esirisc_jtag_deassert_reset(struct esirisc_jtag *jtag_info);

int esirisc_jtag_break(struct esirisc_jtag *jtag_info);
int esirisc_jtag_continue(struct esirisc_jtag *jtag_info);

int esirisc_jtag_flush_caches(struct esirisc_jtag *jtag_info);

#endif /* OPENOCD_TARGET_ESIRISC_JTAG_H */
