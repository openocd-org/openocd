#include <assert.h>
#include <stdlib.h>
#include <time.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "target_type.h"
#include "log.h"
#include "jtag/jtag.h"
#include "opcodes.h"
#include "register.h"
#include "breakpoints.h"

/**
 * Since almost everything can be accomplish by scanning the dbus register, all
 * functions here assume dbus is already selected. The exception are functions
 * called directly by OpenOCD, which can't assume anything about what's
 * currently in IR. They should set IR to dbus explicitly.
 */

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

#define DEBUG_ROM_START         0x800
#define DEBUG_ROM_RESUME        (DEBUG_ROM_START + 4)
#define DEBUG_ROM_EXCEPTION     (DEBUG_ROM_START + 8)
#define DEBUG_RAM_START         0x400

#define SETHALTNOT				0x10c

/*** JTAG registers. ***/

#define DTMINFO					0x10
#define DTMINFO_ADDRBITS		(0xf<<4)
#define DTMINFO_VERSION			(0xf)

#define DBUS						0x11
#define DBUS_OP_START				0
#define DBUS_OP_SIZE				2
typedef enum {
	DBUS_OP_NOP = 0,
	DBUS_OP_READ = 1,
	DBUS_OP_WRITE = 2,
	DBUS_OP_CONDITIONAL_WRITE = 3
} dbus_op_t;
typedef enum {
	DBUS_STATUS_SUCCESS = 0,
	DBUS_STATUS_NO_WRITE = 1,
	DBUS_STATUS_FAILED = 2,
	DBUS_STATUS_BUSY = 3
} dbus_status_t;
#define DBUS_DATA_START				2
#define DBUS_DATA_SIZE				34
#define DBUS_ADDRESS_START			36

/*** Debug Bus registers. ***/

#define DMCONTROL				0x10
#define DMCONTROL_INTERRUPT		(1L<<33)
#define DMCONTROL_HALTNOT		(1L<<32)
#define DMCONTROL_BUSERROR		(7<<19)
#define DMCONTROL_SERIAL		(3<<16)
#define DMCONTROL_AUTOINCREMENT	(1<<15)
#define DMCONTROL_ACCESS		(7<<12)
#define DMCONTROL_HARTID		(0x3ff<<2)
#define DMCONTROL_NDRESET		(1<<1)
#define DMCONTROL_FULLRESET		1

#define DMINFO					0x11
#define DMINFO_ABUSSIZE			(0x7f<<25)
#define DMINFO_SERIALCOUNT		(0xf<<21)
#define DMINFO_ACCESS128		(1<<20)
#define DMINFO_ACCESS64			(1<<19)
#define DMINFO_ACCESS32			(1<<18)
#define DMINFO_ACCESS16			(1<<17)
#define DMINFO_ACCESS8			(1<<16)
#define DMINFO_DRAMSIZE			(0x3f<<10)
#define DMINFO_AUTHENTICATED	(1<<5)
#define DMINFO_AUTHBUSY			(1<<4)
#define DMINFO_AUTHTYPE			(3<<2)
#define DMINFO_VERSION			3

/*** Info about the core being debugged. ***/

#define DBUS_ADDRESS_UNKNOWN	0xffff

// gdb's register list is defined in riscv_gdb_reg_names gdb/riscv-tdep.c in
// its source tree. We must interpret the numbers the same here.
enum {
	REG_XPR0 = 0,
	REG_XPR31 = 31,
	REG_PC = 32,
	REG_FPR0 = 33,
	REG_FPR31 = 64,
	REG_CSR0 = 65,
	REG_CSR4095 = 4160,
	REG_END = 4161,
	REG_COUNT
};

typedef struct {
	/* Number of address bits in the dbus register. */
	uint8_t addrbits;
	/* Width of a GPR (and many other things) in bits. */
	uint8_t xlen;
	/* Last value we wrote to DBUS_ADDRESS (eg. the address of the register
	 * whose value will be read the next time we scan dbus). */
	uint16_t dbus_address;
	/* Last op we wrote to dbus. */
	dbus_op_t dbus_op;
	/* Number of words in Debug RAM. */
	unsigned int dramsize;
	/* Our local copy of Debug RAM. */
	uint32_t *dram;
	/* One bit for every word in dram. If the bit is set, then we're
	 * confident that the value we have matches the one in actual Debug
	 * RAM. */
	uint64_t dram_valid;
	uint32_t dcsr;

	struct reg *reg_list;
	/* Single buffer that contains all register names, instead of calling
	 * malloc for each register. Needs to be freed when reg_list is freed. */
	char *reg_names;
	/* Single buffer that contains all register values. */
	void *reg_values;
} riscv_info_t;

typedef struct {
	bool haltnot;
	bool interrupt;
} bits_t;

/*** Necessary prototypes. ***/

static int riscv_poll(struct target *target);

/*** Utility functions. ***/

static uint8_t ir_dtminfo[1] = {DTMINFO};
static struct scan_field select_dtminfo = {
	.in_value = NULL,
	.out_value = ir_dtminfo
};
static uint8_t ir_dbus[1] = {DBUS};
static struct scan_field select_dbus = {
	.in_value = NULL,
	.out_value = ir_dbus
};
static uint8_t ir_debug[1] = {0x5};
static struct scan_field select_debug = {
	.in_value = NULL,
	.out_value = ir_debug
};
#define DEBUG_LENGTH	264

static uint16_t dram_address(unsigned int index)
{
	if (index < 0x10)
		return index;
	else
		return 0x40 + index - 0x10;
}

#if 0
static int debug_scan(struct target *target)
{
	uint8_t in[DIV_ROUND_UP(DEBUG_LENGTH, 8)];
	jtag_add_ir_scan(target->tap, &select_debug, TAP_IDLE);

	struct scan_field field;
	field.num_bits = DEBUG_LENGTH;
	field.out_value = NULL;
	field.check_value = NULL;
	field.check_mask = NULL;
	field.in_value = in;
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	/* Always return to dbus. */
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	jtag_execute_queue();

	LOG_DEBUG("  debug_pc=0x%x", buf_get_u32(in, 0, 32));
	LOG_DEBUG("  last_returned_data=0x%x", buf_get_u32(in, 32, 32));
	LOG_DEBUG("  last_returned_pc=0x%x", buf_get_u32(in, 64, 32));
	LOG_DEBUG("  last_requested_pc=0x%x", buf_get_u32(in, 96, 32));
	LOG_DEBUG("  last_committed_instruction=0x%x", buf_get_u32(in, 128, 32));
	LOG_DEBUG("  last_committed_pc=0x%x", buf_get_u32(in, 160, 32));
	LOG_DEBUG("  last_committed_time=0x%lx", buf_get_u64(in, 192, 64));
	LOG_DEBUG("  3bits=0x%x", buf_get_u32(in, 256, 3));

	return 0;
}
#endif

static dbus_status_t dbus_scan(struct target *target, uint64_t *data_in,
		dbus_op_t op, uint16_t address, uint64_t data_out)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	uint8_t in[8] = {0};
	uint8_t out[8];
	struct scan_field field = {
		.num_bits = info->addrbits + DBUS_OP_SIZE + DBUS_DATA_SIZE,
		.out_value = out,
		.in_value = in
	};

	assert(info->addrbits != 0);

	buf_set_u64(out, DBUS_OP_START, DBUS_OP_SIZE, op);
	buf_set_u64(out, DBUS_DATA_START, DBUS_DATA_SIZE, data_out);
	buf_set_u64(out, DBUS_ADDRESS_START, info->addrbits, address);

	/* Assume dbus is already selected. */
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
	jtag_add_runtest(1, TAP_IDLE);
	info->dbus_address = address;
	info->dbus_op = op;

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("dbus_scan failed jtag scan");
		return retval;
	}

	if (data_in) {
		*data_in = buf_get_u64(in, DBUS_DATA_START, DBUS_DATA_SIZE);
	}

	static const char *op_string[] = {"nop", "r", "w", "cw"};
	static const char *status_string[] = {"+", "nw", "F", "b"};
	LOG_DEBUG("vvv $display(\"hardware: dbus scan %db %s %01x:%08x @%02x -> %s %01x:%08x @%02x\");",
			field.num_bits,
			op_string[buf_get_u32(out, 0, 2)],
			buf_get_u32(out, 34, 2), buf_get_u32(out, 2, 32),
			buf_get_u32(out, 36, info->addrbits),
			status_string[buf_get_u32(in, 0, 2)],
			buf_get_u32(in, 34, 2), buf_get_u32(in, 2, 32),
			buf_get_u32(in, 36, info->addrbits));
	/*
	LOG_DEBUG("dbus scan %db %s %01x:%08x @%02x -> %s %01x:%08x @%02x",
			field.num_bits,
			op_string[buf_get_u32(out, 0, 2)],
			buf_get_u32(out, 34, 2), buf_get_u32(out, 2, 32),
			buf_get_u32(out, 36, info->addrbits),
			status_string[buf_get_u32(in, 0, 2)],
			buf_get_u32(in, 34, 2), buf_get_u32(in, 2, 32),
			buf_get_u32(in, 36, info->addrbits));
			*/

	//debug_scan(target);

	return buf_get_u64(in, DBUS_OP_START, DBUS_OP_SIZE);
}

static uint64_t dbus_read(struct target *target, uint16_t address, uint16_t next_address)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	uint64_t value;

	dbus_status_t status = DBUS_STATUS_BUSY;
	if (address != info->dbus_address || info->dbus_op == DBUS_OP_NOP) {
		while (status == DBUS_STATUS_BUSY) {
			status = dbus_scan(target, NULL, DBUS_OP_READ, address, 0);
		}
	}
	status = DBUS_STATUS_BUSY;
	while (status == DBUS_STATUS_BUSY) {
		status = dbus_scan(target, &value, DBUS_OP_READ, next_address, 0);
	}
	if (status != DBUS_STATUS_SUCCESS) {
		LOG_ERROR("dbus_read failed read at 0x%x; status=%d\n", address, status);
	}
	return value;
}

static void dbus_write(struct target *target, uint16_t address, uint64_t value)
{
	dbus_status_t status = DBUS_STATUS_BUSY;
	while (status == DBUS_STATUS_BUSY) {
		status = dbus_scan(target, NULL, DBUS_OP_WRITE, address, value);
	}
	if (status != DBUS_STATUS_SUCCESS) {
		LOG_ERROR("dbus_write failed write 0x%lx to 0x%x; status=%d\n", value,
				address, status);
	}
}

static uint32_t dtminfo_read(struct target *target)
{
	struct scan_field field;
	uint8_t in[4];

	jtag_add_ir_scan(target->tap, &select_dtminfo, TAP_IDLE);

	field.num_bits = 32;
	field.out_value = NULL;
	field.in_value = in;
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("dtminfo_read failed jtag scan");
		return retval;
	}

	/* Always return to dbus. */
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	return buf_get_u32(field.in_value, 0, 32);
}

static uint32_t dram_read32(struct target *target, unsigned int index)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	// TODO: check cache to see if this even needs doing.
	uint16_t address = dram_address(index);
	uint32_t value = dbus_read(target, address, address);
	info->dram_valid |= (1<<index);
	info->dram[index] = value;
	return value;
}

static void dram_write32(struct target *target, unsigned int index, uint32_t value,
		bool set_interrupt)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

#if 1
	if (!set_interrupt &&
			info->dram_valid & (1<<index) &&
			info->dram[index] == value) {
		LOG_DEBUG("DRAM cache hit: 0x%x @%d", value, index);
		return;
	}
#endif

	uint64_t dbus_value = DMCONTROL_HALTNOT | value;
	if (set_interrupt)
		dbus_value |= DMCONTROL_INTERRUPT;
	dbus_write(target, dram_address(index), dbus_value);
	info->dram_valid |= (1<<index);
	info->dram[index] = value;
}

#if 1
static int dram_check32(struct target *target, unsigned int index,
		uint32_t expected)
{
	uint16_t address = dram_address(index);
	uint32_t actual = dbus_read(target, address, address + 1);
	if (expected != actual) {
		LOG_ERROR("Wrote 0x%x to Debug RAM at %d, but read back 0x%x",
				expected, index, actual);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}
#endif

/* Read the haltnot and interrupt bits. */
static bits_t read_bits(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	static int next_address = 0;

	uint64_t value;
	if (info->dbus_address < 0x10 || info->dbus_address == DMCONTROL) {
		value = dbus_read(target, info->dbus_address, next_address);
	} else {
		value = dbus_read(target, 0, next_address);
	}

	if (info->dram_valid) {
		// Cycle through addresses, so we have more debug info. Only look at
		// ones that we've written, to reduce data mismatch between real life
		// and simulation.
		do {
			next_address = (next_address + 1) % 64;
		} while (!(info->dram_valid & (1<<next_address)));
	}

	bits_t result = {
		.haltnot = get_field(value, DMCONTROL_HALTNOT),
		.interrupt = get_field(value, DMCONTROL_INTERRUPT)
	};
	return result;
}

/* Write instruction that jumps from the specified word in Debug RAM to resume
 * in Debug ROM. */
static void dram_write_jump(struct target *target, unsigned int index, bool set_interrupt)
{
	dram_write32(target, index,
			jal(0, (uint32_t) (DEBUG_ROM_RESUME - (DEBUG_RAM_START + 4*index))),
			set_interrupt);
}

static int wait_for_state(struct target *target, enum target_state state)
{
	time_t start = time(NULL);
	while (1) {
		int result = riscv_poll(target);
		if (result != ERROR_OK) {
			return result;
		}
		if (target->state == state) {
			return ERROR_OK;
		}
		if (time(NULL) - start > 2) {
			LOG_ERROR("Timed out waiting for state %d.", state);
			return ERROR_FAIL;
		}
	}
}

static int wait_for_debugint_clear(struct target *target)
{
	time_t start = time(NULL);
	// Throw away the results of the first read, since they'll contain the
	// result of the read that happened just before debugint was set. (Assuming
	// the last scan before calling this function was one that sets debugint.)
	read_bits(target);
	while (1) {
		bits_t bits = read_bits(target);
		if (!bits.interrupt) {
			return ERROR_OK;
		}
		if (time(NULL) - start > 2) {
			LOG_ERROR("Timed out waiting for debug int to clear.");
			return ERROR_FAIL;
		}
	}
}

static int wait_and_read(struct target *target, uint32_t *data, uint16_t address)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	time_t start = time(NULL);
	// Throw away the results of the first read, since they'll contain the
	// result of the read that happened just before debugint was set. (Assuming
	// the last scan before calling this function was one that sets debugint.)
	dbus_read(target, info->dbus_address, address);

	while (1) {
		uint64_t dbus_value = dbus_read(target, info->dbus_address, address);
		*data = dbus_value;
		if (!get_field(dbus_value, DMCONTROL_INTERRUPT)) {
			return ERROR_OK;
		}
		if (time(NULL) - start > 2) {
			LOG_ERROR("Timed out waiting for debug int to clear.");
			return ERROR_FAIL;
		}
	}
}

static int read_csr(struct target *target, uint32_t *value, uint32_t csr)
{
	dram_write32(target, 0, csrr(S0, csr), false);
	dram_write32(target, 1, sw(S0, ZERO, DEBUG_RAM_START + 16), false);
	dram_write_jump(target, 2, true);

	if (wait_for_debugint_clear(target) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
		return ERROR_FAIL;
	}
	*value = dram_read32(target, 4);

	return ERROR_OK;
}

static int resume(struct target *target, int current, uint32_t address,
		int handle_breakpoints, int debug_execution, bool step)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	if (!current) {
		if (info->xlen > 32) {
			LOG_WARNING("Asked to resume at 32-bit PC on %d-bit target.",
					info->xlen);
		}
		LOG_ERROR("TODO: current is false");
		return ERROR_FAIL;
	}

	if (handle_breakpoints) {
		LOG_ERROR("TODO: handle_breakpoints is true");
		return ERROR_FAIL;
	}

	if (debug_execution) {
		LOG_ERROR("TODO: debug_execution is true");
		return ERROR_FAIL;
	}

	info->dcsr |= DCSR_EBREAKM | DCSR_EBREAKH | DCSR_EBREAKS | DCSR_EBREAKU;
	info->dcsr &= ~DCSR_HALT;

	if (step) {
		info->dcsr |= DCSR_STEP;
	} else {
		info->dcsr &= ~DCSR_STEP;
	}

	dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16), false);
	dram_write32(target, 1, csrw(S0, CSR_DCSR), false);
	dram_write_jump(target, 2, false);

	// Write DCSR value, set interrupt and clear haltnot.
	uint64_t dbus_value = DMCONTROL_INTERRUPT | info->dcsr;
	dbus_write(target, dram_address(4), dbus_value);
	info->dram_valid |= (1<<4);
	info->dram[4] = info->dcsr;

	if (wait_for_debugint_clear(target) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
		return ERROR_FAIL;
	}

	target->state = TARGET_RUNNING;

	return ERROR_OK;
}

/** Update register sizes based on xlen. */
static void update_reg_list(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	if (info->reg_values) {
		free(info->reg_values);
	}
	info->reg_values = malloc(REG_COUNT * info->xlen / 4);

	for (unsigned int i = 0; i < REG_COUNT; i++) {
		struct reg *r = &info->reg_list[i];
		r->value = info->reg_values + i * info->xlen / 4;
		r->size = info->xlen;
		if (r->dirty) {
			LOG_ERROR("Register %d was dirty. Its value is lost.", i);
		}
		r->valid = false;
	}
}

/*** OpenOCD target functions. ***/

static int register_get(struct reg *reg)
{
	struct target *target = (struct target *) reg->arch_info;
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	uint32_t value = 0;

	if (reg->number == S0) {
		dram_write32(target, 0, csrr(S0, CSR_DSCRATCH), false);
		dram_write32(target, 1, sw(S0, ZERO, DEBUG_RAM_START), false);
		dram_write_jump(target, 2, true);
	} else if (reg->number == S1) {
		dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 4 * info->dramsize - 4), false);
		dram_write32(target, 1, sw(S0, ZERO, DEBUG_RAM_START), false);
		dram_write_jump(target, 2, true);
	} else if (reg->number == ZERO) {
		buf_set_u64(reg->value, 0, info->xlen, value);
		LOG_DEBUG("%s=0x%x", reg->name, value);
		return ERROR_OK;
	} else if (reg->number <= REG_XPR31) {
		dram_write_jump(target, 1, false);
		dram_write32(target, 0, sw(reg->number - REG_XPR0, ZERO, DEBUG_RAM_START), true);
	} else if (reg->number == REG_PC) {
		dram_write32(target, 0, csrr(S0, CSR_DPC), false);
		dram_write32(target, 1, sw(S0, ZERO, DEBUG_RAM_START), false);
		dram_write_jump(target, 2, true);
	} else if (reg->number >= REG_FPR0 && reg->number <= REG_FPR31) {
		dram_write32(target, 0, fsw(reg->number - REG_FPR0, 0, DEBUG_RAM_START), false);
		dram_write_jump(target, 1, true);
	} else if (reg->number >= REG_CSR0 && reg->number <= REG_CSR4095) {
		dram_write32(target, 0, csrr(S0, reg->number - REG_CSR0), false);
		dram_write32(target, 1, sw(S0, ZERO, DEBUG_RAM_START), false);
		dram_write_jump(target, 2, true);
	} else {
		LOG_ERROR("Don't know how to read register %d (%s)", reg->number, reg->name);
		return ERROR_FAIL;
	}

	if (wait_and_read(target, &value, 0) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
		return ERROR_FAIL;
	}

	LOG_DEBUG("%s=0x%x", reg->name, value);
	buf_set_u32(reg->value, 0, 32, value);

	info->dram_valid &= ~1;

	return ERROR_OK;
}

static int register_write(struct target *target, unsigned int number,
		uint32_t value)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	if (number == S0) {
		dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16), false);
		dram_write32(target, 1, csrw(S0, CSR_DSCRATCH), false);
		dram_write_jump(target, 2, false);
	} else if (number == S1) {
		dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16), false);
		dram_write32(target, 1, sw(S0, ZERO, DEBUG_RAM_START + 4 * info->dramsize - 4), false);
		dram_write_jump(target, 2, false);
	} else if (number <= REG_XPR31) {
		dram_write32(target, 0, lw(number - REG_XPR0, ZERO, DEBUG_RAM_START + 16), false);
		dram_write_jump(target, 1, false);
	} else if (number == REG_PC) {
		dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16), false);
		dram_write32(target, 1, csrw(S0, CSR_DPC), false);
		dram_write_jump(target, 2, false);
	} else if (number >= REG_FPR0 && number <= REG_FPR31) {
		dram_write32(target, 0, flw(number - REG_FPR0, 0, DEBUG_RAM_START + 16), false);
		dram_write_jump(target, 1, false);
	} else if (number >= REG_CSR0 && number <= REG_CSR4095) {
		dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16), false);
		dram_write32(target, 1, csrw(S0, number - REG_CSR0), false);
		dram_write_jump(target, 2, false);
	} else {
		LOG_ERROR("Don't know how to read register %d", number);
		return ERROR_FAIL;
	}

	dram_write32(target, 4, value, true);

	if (wait_for_debugint_clear(target) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int register_set(struct reg *reg, uint8_t *buf)
{
	struct target *target = (struct target *) reg->arch_info;

	uint32_t value = buf_get_u32(buf, 0, 32);

	LOG_DEBUG("write 0x%x to %s", value, reg->name);

	return register_write(target, reg->number, value);
}

static struct reg_arch_type riscv_reg_arch_type = {
	.get = register_get,
	.set = register_set
};

static int riscv_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_DEBUG("riscv_init_target()");
	target->arch_info = calloc(1, sizeof(riscv_info_t));
	if (!target->arch_info)
		return ERROR_FAIL;
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	info->dbus_address = DBUS_ADDRESS_UNKNOWN;
	info->dbus_op = DBUS_OP_NOP;

	select_dtminfo.num_bits = target->tap->ir_length;
	select_dbus.num_bits = target->tap->ir_length;
	select_debug.num_bits = target->tap->ir_length;

	const unsigned int max_reg_name_len = 12;
	info->reg_list = calloc(REG_COUNT, sizeof(struct reg));

	info->reg_names = calloc(1, REG_COUNT * max_reg_name_len);
	char *reg_name = info->reg_names;
	info->reg_values = NULL;

	for (unsigned int i = 0; i < REG_COUNT; i++) {
		struct reg *r = &info->reg_list[i];
		r->number = i;
		r->caller_save = true;
		r->dirty = false;
		r->valid = false;
		r->exist = true;
		r->type = &riscv_reg_arch_type;
		r->arch_info = target;
		if (i <= REG_XPR31) {
			sprintf(reg_name, "x%d", i);
		} else if (i == REG_PC) {
			sprintf(reg_name, "pc");
		} else if (i >= REG_FPR0 && i <= REG_FPR31) {
			sprintf(reg_name, "f%d", i - REG_FPR0);
		} else if (i >= REG_CSR0 && i <= REG_CSR4095) {
			sprintf(reg_name, "csr%d", i - REG_CSR0);
		}
		if (reg_name[0]) {
			r->name = reg_name;
		}
		reg_name += strlen(reg_name) + 1;
		assert(reg_name < info->reg_names + REG_COUNT * max_reg_name_len);
	}
	update_reg_list(target);

	info->dram_valid = 0;

	return ERROR_OK;
}

static void riscv_deinit_target(struct target *target)
{
	LOG_DEBUG("riscv_deinit_target()");
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	if (info->dram) {
		free(info->dram);
	}
	free(info);
	target->arch_info = NULL;
}

static int riscv_halt(struct target *target)
{
	LOG_DEBUG("riscv_halt()");
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	dram_write32(target, 0, csrsi(CSR_DCSR, DCSR_HALT), false);
	dram_write32(target, 1, csrr(S0, CSR_MHARTID), false);
	dram_write32(target, 2, sw(S0, ZERO, SETHALTNOT), false);
	dram_write_jump(target, 3, true);

	if (wait_for_debugint_clear(target) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int riscv_step(struct target *target, int current, uint32_t address,
		int handle_breakpoints)
{
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
	// Hardware single step doesn't exist yet.
#if 0
	return resume(target, current, address, handle_breakpoints, 0, true);
#else
	uint32_t dpc;
	if (read_csr(target, &dpc, CSR_DPC) != ERROR_OK) {
		return ERROR_FAIL;
	}
	uint32_t next_pc = dpc + 4;
	// TODO: write better next pc prediction code
	if (breakpoint_add(target, next_pc, 4, BKPT_SOFT) != ERROR_OK) {
		return ERROR_FAIL;
	}
	if (resume(target, current, address, handle_breakpoints, 0, false) != ERROR_OK) {
		return ERROR_FAIL;
	}
	while (target->state == TARGET_RUNNING) {
		riscv_poll(target);
	}
	breakpoint_remove(target, next_pc);

	return ERROR_OK;
#endif
}

#if 0
static void dram_test(struct target *target)
{
	uint32_t shadow[16];

	for (int j = 0; j < 100; j++) {
		LOG_DEBUG("Round %d", j);
		for (int i = 0; i < 16; i++) {
			shadow[i] = random();
			dram_write32(target, i, shadow[i], false);
		}
		for (int i = 0; i < 16; i++) {
			if (dram_check32(target, i, shadow[i]) != ERROR_OK) {
				LOG_ERROR("Mismatch! j=%d i=%d", j, i);
			}
		}
	}
}
#endif

#if 0
static void light_leds(struct target *target)
{
	dram_write32(target, 0, lui(S0, 0x70002), false);
	dram_write32(target, 1, lui(S1, 0xccccc), false);
	dram_write32(target, 2, sw(S1, S0, 0xa0), false);
	dram_write32(target, 3, jal(ZERO, 0), true);
}
#endif

static void dump_debug_ram(struct target *target)
{
	for (unsigned int i = 0; i < 16; i++) {
		uint32_t value = dram_read32(target, i);
		LOG_ERROR("Debug RAM 0x%x: 0x%08x", i, value);
	}
}

#if 0
static void write_constants(struct target *target)
{
	dram_write32(target, 0, lui(S0, 0x70002), false);
	dram_write32(target, 1, lui(S1, 0xccccc), false);
	dram_write32(target, 2, sw(S0, ZERO, DEBUG_RAM_START + 0), false);
	dram_write32(target, 3, sw(S1, ZERO, DEBUG_RAM_START + 4), false);
	dram_write_jump(target, 4, true);

	if (wait_for_debugint_clear(target) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
	}

	uint32_t word0 = dram_read32(target, 0);
	uint32_t word1 = dram_read32(target, 1);
	if (word0 != 0x70002000) {
		LOG_ERROR("Value at word 0 should be 0x%x but is 0x%x",
				0x70002000, word0);
	}
	if (word1 != 0x70002000) {
		LOG_ERROR("Value at word 1 should be 0x%x but is 0x%x",
				0x70002000, word1);
	}
}
#endif

#if 0
static void test_s1(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	riscv_halt(target);
	riscv_poll(target);

	dram_write32(target, info->dramsize - 1, 0xdeadbed, false);
	dram_read32(target, info->dramsize - 1);
	riscv_step(target, true, 0, 0);
	dram_read32(target, info->dramsize - 1);

	exit(0);
}
#endif 

static int riscv_examine(struct target *target)
{
	LOG_DEBUG("riscv_examine()");
	if (target_was_examined(target)) {
		return ERROR_OK;
	}

	// Don't need to select dbus, since the first thing we do is read dtminfo.

	uint32_t dtminfo = dtminfo_read(target);
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	info->addrbits = get_field(dtminfo, DTMINFO_ADDRBITS);

	uint32_t dminfo = dbus_read(target, DMINFO, 0);
	// TODO: need to read dminfo twice to get the correct value.
	dminfo = dbus_read(target, DMINFO, 0);
	LOG_DEBUG("dminfo: 0x%08x", dminfo);
	LOG_DEBUG("  abussize=0x%x", get_field(dminfo, DMINFO_ABUSSIZE));
	LOG_DEBUG("  serialcount=0x%x", get_field(dminfo, DMINFO_SERIALCOUNT));
	LOG_DEBUG("  access128=%d", get_field(dminfo, DMINFO_ACCESS128));
	LOG_DEBUG("  access64=%d", get_field(dminfo, DMINFO_ACCESS64));
	LOG_DEBUG("  access32=%d", get_field(dminfo, DMINFO_ACCESS32));
	LOG_DEBUG("  access16=%d", get_field(dminfo, DMINFO_ACCESS16));
	LOG_DEBUG("  access8=%d", get_field(dminfo, DMINFO_ACCESS8));
	LOG_DEBUG("  dramsize=0x%x", get_field(dminfo, DMINFO_DRAMSIZE));
	LOG_DEBUG("  authenticated=0x%x", get_field(dminfo, DMINFO_AUTHENTICATED));
	LOG_DEBUG("  authbusy=0x%x", get_field(dminfo, DMINFO_AUTHBUSY));
	LOG_DEBUG("  authtype=0x%x", get_field(dminfo, DMINFO_AUTHTYPE));
	LOG_DEBUG("  version=0x%x", get_field(dminfo, DMINFO_VERSION));

	if (get_field(dminfo, DMINFO_VERSION) != 1) {
		LOG_ERROR("OpenOCD only supports Debug Module version 1, not %d",
				get_field(dminfo, DMINFO_VERSION));
		return ERROR_FAIL;
	}

	info->dramsize = get_field(dminfo, DMINFO_DRAMSIZE) + 1;
	info->dram = malloc(info->dramsize * 4);
	if (!info->dram)
		return ERROR_FAIL;

	if (get_field(dminfo, DMINFO_AUTHTYPE) != 0) {
		LOG_ERROR("Authentication required by RISC-V core but not "
				"supported by OpenOCD. dminfo=0x%x", dminfo);
		return ERROR_FAIL;
	}

	// Figure out XLEN.
	dram_write32(target, 0, xori(S1, ZERO, -1), false);
	// 0xffffffff  0xffffffff:ffffffff  0xffffffff:ffffffff:ffffffff:ffffffff
	dram_write32(target, 1, srli(S1, S1, 31), false);
	// 0x00000001  0x00000001:ffffffff  0x00000001:ffffffff:ffffffff:ffffffff
	dram_write32(target, 2, sw(S1, ZERO, DEBUG_RAM_START), false);
	dram_write32(target, 3, srli(S1, S1, 31), false);
	// 0x00000000  0x00000000:00000003  0x00000000:00000003:ffffffff:ffffffff
	dram_write32(target, 4, sw(S1, ZERO, DEBUG_RAM_START + 4), false);

	// Check that we can actually read/write dram.
	int error = 0;
	error += dram_check32(target, 0, xori(S1, ZERO, -1));
	error += dram_check32(target, 1, srli(S1, S1, 31));
	error += dram_check32(target, 2, sw(S1, ZERO, DEBUG_RAM_START));
	error += dram_check32(target, 3, srli(S1, S1, 31));
	error += dram_check32(target, 4, sw(S1, ZERO, DEBUG_RAM_START + 4));
	if (error != 5 * ERROR_OK) {
		dump_debug_ram(target);
		return ERROR_FAIL;
	}

	// Execute.
	dram_write_jump(target, 5, true);

	if (wait_for_debugint_clear(target) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
		return ERROR_FAIL;
	}

	uint32_t word0 = dram_read32(target, 0);
	uint32_t word1 = dram_read32(target, 1);
	if (word0 == 1 && word1 == 0) {
		info->xlen = 32;
	} else if (word0 == 0xffffffff && word1 == 3) {
		info->xlen = 64;
	} else if (word0 == 0xffffffff && word1 == 0xffffffff) {
		info->xlen = 128;
	} else {
		uint32_t exception = dram_read32(target, info->dramsize-1);
		LOG_ERROR("Failed to discover xlen; word0=0x%x, word1=0x%x, exception=0x%x",
				word0, word1, exception);
		return ERROR_FAIL;
	}
	LOG_DEBUG("Discovered XLEN is %d", info->xlen);

	// Update register list to match discovered XLEN.
	update_reg_list(target);

	target_set_examined(target);

	//write_constants(target);
	//light_leds(target);
	//dram_test(target);
	//test_s1(target);

	return ERROR_OK;
}

static int handle_halt(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	target->state = TARGET_HALTED;

	uint32_t dpc;
	if (read_csr(target, &dpc, CSR_DPC) != ERROR_OK) {
		return ERROR_FAIL;
	}

	if (read_csr(target, &info->dcsr, CSR_DCSR) != ERROR_OK) {
		return ERROR_FAIL;
	}
	int cause = get_field(info->dcsr, DCSR_CAUSE);
	LOG_DEBUG("halt cause is %d; dcsr=0x%x", cause, info->dcsr);
	switch (cause) {
		case DCSR_CAUSE_SWBP:
		case DCSR_CAUSE_HWBP:
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case DCSR_CAUSE_DEBUGINT:
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case DCSR_CAUSE_STEP:
			target->debug_reason = DBG_REASON_SINGLESTEP;
			break;
		case DCSR_CAUSE_HALT:
		default:
			LOG_ERROR("Invalid halt cause %d in DCSR (0x%x)",
					cause, info->dcsr);
	}

	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	LOG_DEBUG("halted at 0x%x", dpc);

	return ERROR_OK;
}

static int riscv_poll(struct target *target)
{
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
	bits_t bits = read_bits(target);

	if (bits.haltnot && bits.interrupt) {
		target->state = TARGET_DEBUG_RUNNING;
		LOG_DEBUG("debug running");
	} else if (bits.haltnot && !bits.interrupt) {
		if (target->state != TARGET_HALTED) {
			return handle_halt(target);
		}
	} else if (!bits.haltnot && bits.interrupt) {
		// Target is halting. There is no state for that, so don't change anything.
		LOG_DEBUG("halting");
	} else if (!bits.haltnot && !bits.interrupt) {
		target->state = TARGET_RUNNING;
		LOG_DEBUG("running");
	}

	return ERROR_OK;
}

static int riscv_resume(struct target *target, int current, uint32_t address,
		int handle_breakpoints, int debug_execution)
{
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
	return resume(target, current, address, handle_breakpoints,
			debug_execution, false);
}

static int riscv_assert_reset(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	// TODO: Maybe what I implemented here is more like soft_reset_halt()?

	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	// The only assumption we can make is that the TAP was reset.
	if (wait_for_debugint_clear(target) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
		return ERROR_FAIL;
	}

	// Not sure what we should do when there are multiple cores.
	// Here just reset the single hart we're talking to.
	info->dcsr |= DCSR_EBREAKM | DCSR_EBREAKH | DCSR_EBREAKS |
		DCSR_EBREAKU | DCSR_HALT;
	if (target->reset_halt) {
		info->dcsr |= DCSR_NDRESET;
	} else {
		info->dcsr |= DCSR_FULLRESET;
	}
	dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16), false);
	dram_write32(target, 1, csrw(S0, CSR_DCSR), false);
	// We shouldn't actually need the jump because a reset should happen.
	dram_write_jump(target, 2, false);
	dram_write32(target, 4, info->dcsr, true);

	target->state = TARGET_RESET;

	return ERROR_OK;
}

static int riscv_deassert_reset(struct target *target)
{
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
	if (target->reset_halt) {
		return wait_for_state(target, TARGET_HALTED);
	} else {
		return wait_for_state(target, TARGET_RUNNING);
	}
}

static int riscv_read_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

#if 0
	// Plain implementation, where we write the address each time.
	dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16), false);
	switch (size) {
		case 1:
			dram_write32(target, 1, lb(S1, S0, 0), false);
			dram_write32(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16), false);
			break;
		case 2:
			dram_write32(target, 1, lh(S1, S0, 0), false);
			dram_write32(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16), false);
			break;
		case 4:
			dram_write32(target, 1, lw(S1, S0, 0), false);
			dram_write32(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16), false);
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}
	dram_write_jump(target, 3, false);

	uint32_t i = 0;
	while (i <= count) {
		uint64_t scan_result;
		// Write the next address, set interrupt, and read the previous value.
		uint64_t interrupt = 0;
		if (i < count) {
			interrupt = DMCONTROL_INTERRUPT;
		}
		dbus_status_t status = dbus_scan(target, &scan_result, DBUS_OP_CONDITIONAL_WRITE,
				4, DMCONTROL_HALTNOT | interrupt | (address + i * size));
		if (status == DBUS_STATUS_SUCCESS) {
			if (i > 0) {
				uint32_t offset = size * (i-1);
				switch (size) {
					case 1:
						buffer[offset] = scan_result & 0xff;
						break;
					case 2:
						buffer[offset] = scan_result & 0xff;
						buffer[offset + 1] = (scan_result >> 8) & 0xff;
						break;
					case 4:
						buffer[offset] = scan_result & 0xff;
						buffer[offset + 1] = (scan_result >> 8) & 0xff;
						buffer[offset + 2] = (scan_result >> 16) & 0xff;
						buffer[offset + 3] = (scan_result >> 24) & 0xff;
						break;
				}
			}
			i++;
		} else if (status == DBUS_STATUS_NO_WRITE || status == DBUS_STATUS_BUSY) {
			// Need to retry the access that failed, which was the previous one.
		} else if (status == DBUS_STATUS_FAILED) {
			LOG_ERROR("dbus write failed!");
			return ERROR_FAIL;
		}
	}
#else
	dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16), false);
	switch (size) {
		case 1:
			dram_write32(target, 1, lb(S1, S0, 0), false);
			dram_write32(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16), false);
			break;
		case 2:
			dram_write32(target, 1, lh(S1, S0, 0), false);
			dram_write32(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16), false);
			break;
		case 4:
			dram_write32(target, 1, lw(S1, S0, 0), false);
			dram_write32(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16), false);
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}
	dram_write_jump(target, 3, false);

	for (unsigned int i = 0; i < count; i++) {
		dram_write32(target, 4, address + i * size, true);

                uint32_t value;
		if (wait_and_read(target, &value, 4) != ERROR_OK) {
			LOG_ERROR("Debug interrupt didn't clear.");
			return ERROR_FAIL;
		}

		unsigned int offset = i * size;
		switch (size) {
			case 1:
				buffer[offset] = value & 0xff;
				break;
			case 2:
				buffer[offset] = value & 0xff;
				buffer[offset + 1] = (value >> 8) & 0xff;
				break;
			case 4:
				buffer[offset] = value & 0xff;
				buffer[offset + 1] = (value >> 8) & 0xff;
				buffer[offset + 2] = (value >> 16) & 0xff;
				buffer[offset + 3] = (value >> 24) & 0xff;
				break;
		}
	}
#endif

	return ERROR_OK;
}

#if 1
static int riscv_write_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	// Set up the address.
	dram_write32(target, 0, sw(T0, ZERO, DEBUG_RAM_START + 20), false);
	dram_write32(target, 1, lw(T0, ZERO, DEBUG_RAM_START + 16), false);
	dram_write_jump(target, 2, false);
	dram_write32(target, 4, address, true);

	uint32_t t0;
	if (wait_and_read(target, &t0, 5) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
		return ERROR_FAIL;
	}

	switch (size) {
		case 1:
			dram_write32(target, 0, lb(S0, ZERO, DEBUG_RAM_START + 16), false);
			dram_write32(target, 1, sb(S0, T0, 0), false);
			break;
		case 2:
			dram_write32(target, 0, lh(S0, ZERO, DEBUG_RAM_START + 16), false);
			dram_write32(target, 1, sh(S0, T0, 0), false);
			break;
		case 4:
			dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16), false);
			dram_write32(target, 1, sw(S0, T0, 0), false);
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}
	dram_write32(target, 2, addi(T0, T0, size), false);
	dram_write_jump(target, 3, false);

	uint32_t i = 0;
	while (i < count) {
		// Write the next value and set interrupt.
		uint32_t value;
		uint32_t offset = size * i;
		switch (size) {
			case 1:
				value = buffer[offset];
				break;
			case 2:
				value = buffer[offset] |
					(buffer[offset+1] << 8);
				break;
			case 4:
				value = buffer[offset] |
					((uint32_t) buffer[offset+1] << 8) |
					((uint32_t) buffer[offset+2] << 16) |
					((uint32_t) buffer[offset+3] << 24);
				break;
			default:
				return ERROR_FAIL;
		}

		dbus_status_t status = dbus_scan(target, NULL, DBUS_OP_CONDITIONAL_WRITE,
				4, DMCONTROL_HALTNOT | DMCONTROL_INTERRUPT | value);
		if (status == DBUS_STATUS_SUCCESS) {
			i++;
		} else if (status == DBUS_STATUS_NO_WRITE) {
			// Need to retry the access that failed, which was the previous one.
			i--;
		} else if (status == DBUS_STATUS_BUSY) {
			// This operation may still complete. Retry the current access.
		} else if (status == DBUS_STATUS_FAILED) {
			LOG_ERROR("dbus write failed!");
			return ERROR_FAIL;
		}
		info->dram_valid |= (1<<4);
		info->dram[4] = value;
	}

	return register_write(target, T0, t0);
}
#else
/** Inefficient implementation that doesn't require conditional writes. */
static int riscv_write_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	// Write program.
	dram_write32(target, 0, lw(S1, ZERO, DEBUG_RAM_START + 16), false);
	switch (size) {
		case 1:
			dram_write32(target, 1, lb(S0, ZERO, DEBUG_RAM_START + 20), false);
			dram_write32(target, 2, sb(S0, S1, 0), false);
			break;
		case 2:
			dram_write32(target, 1, lh(S0, ZERO, DEBUG_RAM_START + 20), false);
			dram_write32(target, 2, sh(S0, S1, 0), false);
			break;
		case 4:
			dram_write32(target, 1, lw(S0, ZERO, DEBUG_RAM_START + 20), false);
			dram_write32(target, 2, sw(S0, S1, 0), false);
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}
	dram_write_jump(target, 3, false);

	for (uint32_t i = 0; i < count; i++) {
		// Write the next value and set interrupt.
		uint32_t value;
		uint32_t offset = size * i;
		switch (size) {
			case 1:
				value = buffer[offset];
				break;
			case 2:
				value = buffer[offset] |
					(buffer[offset+1] << 8);
				break;
			case 4:
				value = buffer[offset] |
					((uint32_t) buffer[offset+1] << 8) |
					((uint32_t) buffer[offset+2] << 16) |
					((uint32_t) buffer[offset+3] << 24);
				break;
			default:
				return ERROR_FAIL;
		}

		dram_write32(target, 4, address + offset, false);
		dram_write32(target, 5, value, true);

		if (wait_for_debugint_clear(target) != ERROR_OK) {
			LOG_ERROR("Debug interrupt didn't clear.");
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}
#endif

static int riscv_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	LOG_DEBUG("reg_class=%d", reg_class);

	switch (reg_class) {
		case REG_CLASS_GENERAL:
			*reg_list_size = 32;
			break;
		case REG_CLASS_ALL:
			*reg_list_size = REG_COUNT;
			break;
		default:
			LOG_ERROR("Unsupported reg_class: %d", reg_class);
			return ERROR_FAIL;
	}

	*reg_list = calloc(*reg_list_size, sizeof(struct reg *));
	if (!*reg_list) {
		return ERROR_FAIL;
	}
	for (int i = 0; i < *reg_list_size; i++) {
		(*reg_list)[i] = &info->reg_list[i];
	}

	return ERROR_OK;
}

int riscv_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
    if (breakpoint->type != BKPT_SOFT) {
        LOG_INFO("OpenOCD only supports software breakpoints.");
        return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
    }

    if (target_read_memory(target, breakpoint->address, breakpoint->length, 1,
                breakpoint->orig_instr) != ERROR_OK) {
        LOG_ERROR("Failed to read original instruction at 0x%x",
                breakpoint->address);
        return ERROR_FAIL;
    }

    int retval;
    if (breakpoint->length == 4) {
        retval = target_write_u32(target, breakpoint->address, ebreak());
    } else {
        retval = target_write_u16(target, breakpoint->address, ebreak_c());
    }
    if (retval != ERROR_OK) {
        LOG_ERROR("Failed to write %d-byte breakpoint instruction at 0x%x",
                breakpoint->length, breakpoint->address);
        return ERROR_FAIL;
    }

    breakpoint->set = true;

    return ERROR_OK;
}

static int riscv_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
    if (breakpoint->type != BKPT_SOFT) {
        LOG_INFO("OpenOCD only supports software breakpoints.");
        return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
    }

	if (target_write_memory(target, breakpoint->address, breakpoint->length, 1,
				breakpoint->orig_instr) != ERROR_OK) {
		LOG_ERROR("Failed to restore instruction for %d-byte breakpoint at 0x%x",
				breakpoint->length, breakpoint->address);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

struct target_type riscv_target =
{
	.name = "riscv",

	.init_target = riscv_init_target,
	.deinit_target = riscv_deinit_target,
	.examine = riscv_examine,

	/* poll current target status */
	.poll = riscv_poll,

	.halt = riscv_halt,
	.resume = riscv_resume,
	.step = riscv_step,

	.assert_reset = riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,

	.read_memory = riscv_read_memory,
	.write_memory = riscv_write_memory,

	.get_gdb_reg_list = riscv_get_gdb_reg_list,

	.add_breakpoint = riscv_add_breakpoint,
	.remove_breakpoint = riscv_remove_breakpoint,
};
