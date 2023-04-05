/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_NDS32_DISASSEMBLER_H
#define OPENOCD_TARGET_NDS32_DISASSEMBLER_H

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

#endif /* OPENOCD_TARGET_NDS32_DISASSEMBLER_H */
