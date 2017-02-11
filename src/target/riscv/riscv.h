#ifndef RISCV_H
#define RISCV_H

#include "opcodes.h"

extern struct target_type riscv011_target;
extern struct target_type riscv013_target;

/*
 * Definitions shared by code supporting all RISC-V versions.
 */

typedef struct {
	unsigned dtm_version;
	struct command_context *cmd_ctx;
	void *version_specific;
	/* Width of a GPR (and many other things) in bits. */
	uint8_t xlen;
} riscv_info_t;

extern uint8_t ir_dtmcontrol[1];
extern struct scan_field select_dtmcontrol;
extern uint8_t ir_dbus[1];
extern struct scan_field select_dbus;
extern uint8_t ir_idcode[1];
extern struct scan_field select_idcode;

/*** Version-independent functions that we don't want in the main address space. ***/

static uint32_t load(const struct target *target, unsigned int rd,
		unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t load(const struct target *target, unsigned int rd,
		unsigned int base, uint16_t offset)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	switch (info->xlen) {
		case 32:
			return lw(rd, base, offset);
		case 64:
			return ld(rd, base, offset);
	}
	assert(0);
}

static uint32_t store(const struct target *target, unsigned int src,
		unsigned int base, uint16_t offset) __attribute__ ((unused));
static uint32_t store(const struct target *target, unsigned int src,
		unsigned int base, uint16_t offset)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	switch (info->xlen) {
		case 32:
			return sw(src, base, offset);
		case 64:
			return sd(src, base, offset);
	}
	assert(0);
}

static unsigned xlen(const struct target *target) __attribute__ ((unused));
static unsigned xlen(const struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	return info->xlen;
}

#endif
