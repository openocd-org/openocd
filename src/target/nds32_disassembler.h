/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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

#ifndef __NDS32_DISASSEMBLER_H__
#define __NDS32_DISASSEMBLER_H__

#include <target/nds32.h>

enum nds32_instruction_type {
	NDS32_INSN_DATA_PROC = 0,
	NDS32_INSN_LOAD_STORE,
	NDS32_INSN_JUMP_BRANCH,
	NDS32_INSN_RESOURCE_ACCESS,
	NDS32_INSN_MISC,
};

struct nds32_instruction {
	enum nds32_instruction_type type;
	char text[128];
	uint32_t opcode;
	uint8_t instruction_size;
	uint32_t access_start;
	uint32_t access_end;

	struct {
		uint8_t opc_6;
		uint8_t rt;
		uint8_t ra;
		uint8_t rb;
		uint8_t rd;
		uint8_t sub_opc;
		int32_t imm;
	} info;

};

int nds32_read_opcode(struct nds32 *nds32, uint32_t address, uint32_t *value);
int nds32_evaluate_opcode(struct nds32 *nds32, uint32_t opcode, uint32_t address,
		struct nds32_instruction *instruction);

#endif /* __NDS32_DISASSEMBLER_H__ */
