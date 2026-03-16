// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * This file wraps the functions in capstone library.
 * It also takes care of API changes across versions.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <capstone.h>
#include <stdint.h>
#include <stdio.h>

#include <helper/command.h>
#include <helper/log.h>
#include <target/oocd_capstone.h>
#include <target/target.h>

/*
 * Extracted from Capstone tool 'cstool', file 'cstool/cstool.c'.
 * Big rework expected for next Capstone v6.
 */
static struct {
	const char *name;
	cs_arch arch;
	cs_mode mode;
} all_archs[] = {
	{ "arm", CS_ARCH_ARM, CS_MODE_ARM },
	{ "armbe", CS_ARCH_ARM, CS_MODE_ARM | CS_MODE_BIG_ENDIAN },
	{ "arm64", CS_ARCH_ARM64, CS_MODE_LITTLE_ENDIAN },
	{ "cortexm", CS_ARCH_ARM, CS_MODE_ARM | CS_MODE_THUMB | CS_MODE_MCLASS },
	{ "thumb", CS_ARCH_ARM, CS_MODE_ARM | CS_MODE_THUMB },
};

int oocd_cs_list_insn_types(struct command_invocation *cmd)
{
	for (size_t i = 0; i < ARRAY_SIZE(all_archs); i++)
		command_print_sameline(cmd, (i == 0) ? "%s" : " %s",
							   all_archs[i].name);

	return ERROR_OK;
}

static void print_opcode(struct command_invocation *cmd, const cs_insn *insn)
{
	char opcode[3 * ARRAY_SIZE(insn->bytes) + 1];

	for (uint16_t i = 0; i < insn->size; i++)
		sprintf(&opcode[3 * i], " %02" PRIx8, insn->bytes[i]);

	command_print(cmd, "0x%08" PRIx64 " %s\t%s%s%s",
				  insn->address, opcode, insn->mnemonic,
				  insn->op_str[0] ? "\t" : "", insn->op_str);
}

int oocd_cs_disassemble(struct command_invocation *cmd, struct target *target,
	uint64_t address, unsigned int count, const char *insn_set)
{
	size_t i;

	for (i = 0; i < ARRAY_SIZE(all_archs); i++)
		if (!strcmp(insn_set, all_archs[i].name))
			break;

	if (i == ARRAY_SIZE(all_archs)) {
		command_print(cmd, "Instruction set \"%s\" not supported by Capstone", insn_set);
		return ERROR_FAIL;
	}

	csh handle;
	cs_err csret = cs_open(all_archs[i].arch, all_archs[i].mode, &handle);
	if (csret != CS_ERR_OK) {
		command_print(cmd, "Capstone cs_open() failed: %s", cs_strerror(csret));
		return ERROR_FAIL;
	}

	csret = cs_option(handle, CS_OPT_SKIPDATA, CS_OPT_ON);
	if (csret != CS_ERR_OK) {
		command_print(cmd, "Capstone cs_option() failed: %s", cs_strerror(csret));
		cs_close(&handle);
		return ERROR_FAIL;
	}

	cs_insn *insn = cs_malloc(handle);
	if (!insn) {
		command_print(cmd, "Capstone cs_malloc() failed: %s", cs_strerror(csret));
		cs_close(&handle);
		return ERROR_FAIL;
	}

	while (count > 0) {
		uint8_t buf[4];

		int retval = target_read_buffer(target, address, sizeof(buf), buf);
		if (retval != ERROR_OK) {
			cs_free(insn, 1);
			cs_close(&handle);
			return retval;
		}

		size_t size = sizeof(buf);
		const uint8_t *tmp = buf;
		bool csbool = cs_disasm_iter(handle, &tmp, &size, &address, insn);
		if (!csbool) {
			command_print(cmd, "Capstone cs_disasm_iter() failed: %s", cs_strerror(csret));
			cs_free(insn, 1);
			cs_close(&handle);
			return ERROR_FAIL;
		}

		print_opcode(cmd, insn);
		count--;
	}

	cs_free(insn, 1);
	cs_close(&handle);

	return ERROR_OK;
}
