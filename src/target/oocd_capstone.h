/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_WRAP_CAPSTONE_H
#define OPENOCD_TARGET_WRAP_CAPSTONE_H

#include <stdint.h>

struct command_invocation;
struct target;

#ifdef HAVE_CAPSTONE

int oocd_cs_list_insn_types(struct command_invocation *cmd);
int oocd_cs_disassemble(struct command_invocation *cmd, struct target *target,
	uint64_t address, unsigned int count, const char *insn_set);

#else /* HAVE_CAPSTONE */

#include <helper/command.h>
#include <helper/log.h>

static inline
int oocd_cs_list_insn_types(struct command_invocation *cmd)
{
	command_print(cmd, "Capstone library not present");
	return ERROR_NOT_IMPLEMENTED;
}

static inline
int oocd_cs_disassemble(struct command_invocation *cmd, struct target *target,
	uint64_t address, unsigned int count, const char *insn_set)
{
	command_print(cmd, "Capstone library not present");
	return ERROR_NOT_IMPLEMENTED;
}

#endif /* HAVE_CAPSTONE */

#endif /* OPENOCD_TARGET_WRAP_CAPSTONE_H */
