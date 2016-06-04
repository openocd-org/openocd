#include <assert.h>
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

static uint16_t dram_address(unsigned int index)
{
	if (index < 0x10)
		return index;
	else
		return 0x40 + index - 0x10;
}

static dbus_status_t dbus_scan(struct target *target, uint64_t *data_in,
		dbus_op_t op, uint16_t address, uint64_t data_out)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	struct scan_field field;
	uint8_t in[8] = {0};
	uint8_t out[8];

	assert(info->addrbits != 0);

	field.num_bits = info->addrbits + DBUS_OP_SIZE + DBUS_DATA_SIZE;
	field.out_value = out;
	field.in_value = in;
	buf_set_u64(out, DBUS_OP_START, DBUS_OP_SIZE, op);
	buf_set_u64(out, DBUS_DATA_START, DBUS_DATA_SIZE, data_out);
	buf_set_u64(out, DBUS_ADDRESS_START, info->addrbits, address);

	/* Assume dbus is already selected. */
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
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
	LOG_DEBUG("dbus scan %db %s %01x:%08x @%02x -> %s %01x:%08x @%02x",
			field.num_bits,
			op_string[buf_get_u32(out, 0, 2)],
			buf_get_u32(out, 34, 2), buf_get_u32(out, 2, 32),
			buf_get_u32(out, 36, info->addrbits),
			status_string[buf_get_u32(in, 0, 2)],
			buf_get_u32(in, 34, 2), buf_get_u32(in, 2, 32),
			buf_get_u32(in, 36, info->addrbits));
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
	LOG_DEBUG("address=0x%x, value=0x%01x:%08x", address,
			(uint32_t) (value >> 32), (uint32_t) value);
	return value;
}

static void dbus_write(struct target *target, uint16_t address, uint64_t value)
{
	LOG_DEBUG("address=0x%x, value=0x%01x:%08x", address,
			(uint32_t) (value >> 32), (uint32_t) value);
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
	// TODO: check cache to see if this even needs doing.
	uint16_t address = dram_address(index);
	return dbus_read(target, address, address);
}

static void dram_write32(struct target *target, unsigned int index, uint32_t value,
		bool set_interrupt)
{
	// TODO: check cache to see if this even needs doing.
	uint64_t dbus_value = DMCONTROL_HALTNOT | value;
	if (set_interrupt)
		dbus_value |= DMCONTROL_INTERRUPT;
	dbus_write(target, dram_address(index), dbus_value);
}

static int dram_check32(struct target *target, unsigned int index,
		uint32_t expected)
{
	uint32_t actual = dram_read32(target, index);
	if (expected != actual) {
		LOG_ERROR("Wrote 0x%x to Debug RAM at %d, but read back 0x%x",
				expected, index, actual);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

/* Read the haltnot and interrupt bits. */
static bits_t read_bits(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	uint64_t value;
	if (info->dbus_address < 0x10 || info->dbus_address == DMCONTROL) {
		value = dbus_read(target, info->dbus_address, 0);
	} else {
		value = dbus_read(target, 0, 0);
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

	dram_write32(target, 0, csrsi(CSR_DCSR, DCSR_HALT), false);
	if (step) {
		dram_write32(target, 1, csrsi(CSR_DCSR, DCSR_STEP), false);
	} else {
		dram_write32(target, 1, csrci(CSR_DCSR, DCSR_STEP), false);
	}
	dram_write_jump(target, 2, true);

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

	if (reg->number == REG_PC) {
		dram_write32(target, 0, csrr(S0, CSR_DPC), false);
		dram_write32(target, 1, sw(S0, ZERO, DEBUG_RAM_START), false);
		dram_write_jump(target, 2, true);
	} else {
		return ERROR_FAIL;
	}

	if (wait_for_debugint_clear(target) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
		return ERROR_FAIL;
	}

	buf_set_u32(reg->value, 0, 32, dram_read32(target, 0));

	return ERROR_OK;
}

static int register_set(struct reg *reg, uint8_t *buf)
{
	return ERROR_FAIL;
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

	const unsigned int max_reg_name_len = 12;
	info->reg_list = calloc(REG_COUNT, sizeof(struct reg));

	info->reg_names = malloc(REG_COUNT * max_reg_name_len);
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

#if 0
static void megan_sequence(struct target *target)
{
	uint64_t data;
	// [RISCV]: DBUS Input = (Addr: 11, Data: 3aaaaaaaa Op: 1)
	// [RISCV]:        DBUS Result = 00xxxxxxxxx  (Addr: 00, Data: xxxxxxxxx Res: x)
	dbus_scan(target, &data, DBUS_OP_READ, 0x11, 0x3aaaaaaaa);
	// [RISCV]: Reading DMINFO
	// [RISCV]: DBUS Input = (Addr: 11, Data: 3aaaaaaaa Op: 1)
	// [RISCV]:        DBUS Result = 1100000f084  (Addr: 11, Data: 000003c21 Res: 0)
	dbus_scan(target, &data, DBUS_OP_READ, 0x11, 0x3aaaaaaaa);

	// extra, to actually read dminfo
	dbus_scan(target, &data, DBUS_OP_READ, 0x11, 0x3aaaaaaaa);

	// [RISCV]:WRITING Debug RAM (interrupt is not set)
	// [RISCV]: DBUS Input = (Addr: 00, Data: 0fff04493 Op: 2)
	// [RISCV]:        DBUS Result = 1100000f084  (Addr: 11, Data: 000003c21 Res: 0)
	dbus_scan(target, &data, DBUS_OP_WRITE, 0, 0x0fff04493);
	// [RISCV]: DBUS Input = (Addr: 01, Data: 001f4d493 Op: 2)
	// [RISCV]:        DBUS Result = 003ee1643dc  (Addr: 00, Data: 0fb8590f7 Res: 0)
	dbus_scan(target, &data, DBUS_OP_WRITE, 1, 0x001f4d493);
	// [RISCV]: DBUS Input = (Addr: 02, Data: 040902023 Op: 2)
	// [RISCV]:        DBUS Result = 013ee1643dc  (Addr: 01, Data: 0fb8590f7 Res: 0)
	dbus_scan(target, &data, DBUS_OP_WRITE, 2, 0x040902023);
	// [RISCV]: DBUS Input = (Addr: 03, Data: 001f4d493 Op: 2)
	// [RISCV]:        DBUS Result = 023ee1643dc  (Addr: 02, Data: 0fb8590f7 Res: 0)
	dbus_scan(target, &data, DBUS_OP_WRITE, 3, 0x001f4d493);
	// [RISCV]: DBUS Input = (Addr: 04, Data: 040902223 Op: 2)
	// [RISCV]:        DBUS Result = 033ee1643dc  (Addr: 03, Data: 0fb8590f7 Res: 0)
	dbus_scan(target, &data, DBUS_OP_WRITE, 4, 0x040902223);
	// [RISCV]:On last write to  Debug RAM, interrupt is set.
	// [RISCV]: DBUS Input = (Addr: 05, Data: 23f00006f Op: 2)
	// [RISCV]:        DBUS Result = 043ee1643dc  (Addr: 04, Data: 0fb8590f7 Res: 0)
	dbus_scan(target, &data, DBUS_OP_WRITE, 5, 0x23f00006f);
	// [RISCV]: Performing "priming" read from Debug RAM
	// [RISCV]: DBUS Input = (Addr: 00, Data: 000000000 Op: 1)
	// [RISCV]:        DBUS Result = 053ee1643dc  (Addr: 05, Data: 0fb8590f7 Res: 0)
	dbus_scan(target, &data, DBUS_OP_READ, 0, 0);
	// [RISCV]: Polling until INTERRUPT is not set
	// [RISCV]: DBUS Input = (Addr: 00, Data: 000000000 Op: 1)
	// [RISCV]:        DBUS Result = 00fffc1124c  (Addr: 00, Data: 3fff04493 Res: 0)
	dbus_scan(target, &data, DBUS_OP_READ, 0, 0);
	// [RISCV]: DBUS Input = (Addr: 00, Data: 000000000 Op: 1)
	// [RISCV]:        DBUS Result = 00fffc1124c  (Addr: 00, Data: 3fff04493 Res: 0)
	dbus_scan(target, &data, DBUS_OP_READ, 0, 0);
	// [RISCV]: DBUS Input = (Addr: 00, Data: 000000000 Op: 1)
	// [RISCV]:        DBUS Result = 00c00000004  (Addr: 00, Data: 300000001 Res: 0)
	dbus_scan(target, &data, DBUS_OP_READ, 0, 0);
	// [RISCV]: DBUS Input = (Addr: 00, Data: 000000000 Op: 1)
	// [RISCV]:        DBUS Result = 00400000004  (Addr: 00, Data: 100000001 Res: 0)
	dbus_scan(target, &data, DBUS_OP_READ, 0, 0);
	// [RISCV]:READING Debug RAM (interrupt is not set)
	// [RISCV]: DBUS Input = (Addr: 00, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 00400000004  (Addr: 00, Data: 100000001 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 01, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 00400000004  (Addr: 00, Data: 100000001 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 02, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 01400000000  (Addr: 01, Data: 100000000 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 03, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 0250240808c  (Addr: 02, Data: 140902023 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 04, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 03407d3524c  (Addr: 03, Data: 101f4d493 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 05, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 0450240888c  (Addr: 04, Data: 140902223 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 06, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 054fc0001bc  (Addr: 05, Data: 13f00006f Res: 0)
	// [RISCV]: DBUS Input = (Addr: 07, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 0642a1643dc  (Addr: 06, Data: 10a8590f7 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 08, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 077ee17e7dc  (Addr: 07, Data: 1fb85f9f7 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 09, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 087ee1643dc  (Addr: 08, Data: 1fb8590f7 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 0a, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 097ee1643dc  (Addr: 09, Data: 1fb8590f7 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 0b, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 0a7ee1643dc  (Addr: 0a, Data: 1fb8590f7 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 0c, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 0b7ee1643dc  (Addr: 0b, Data: 1fb8590f7 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 0d, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 0c7ee1643dc  (Addr: 0c, Data: 1fb8590f7 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 0e, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 0d7ee1643dc  (Addr: 0d, Data: 1fb8590f7 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 0f, Data: 0babecafe Op: 1)
	// [RISCV]:        DBUS Result = 0e7ee1643dc  (Addr: 0e, Data: 1fb8590f7 Res: 0)
	// [RISCV]: DBUS Input = (Addr: 00, Data: 000000000 Op: 1)
	// [RISCV]:        DBUS Result = 0f56cfc76d8  (Addr: 0f, Data: 15b3f1db6 Res: 0)
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
	info->dram_valid = 0;

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
		return ERROR_FAIL;
	}

	// TODO: Doesn't work with this extra nop.
	dram_write32(target, 5, nop(), false);
	// Execute.
	dram_write_jump(target, 6, true);

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
		LOG_ERROR("Failed to discover xlen; word0=0x%x, word1=0x%x",
				word0, word1);
		return ERROR_FAIL;
	}
	LOG_DEBUG("Discovered XLEN is %d", info->xlen);

	// Update register list to match discovered XLEN.
	update_reg_list(target);

	target_set_examined(target);

	return ERROR_OK;
}

static int riscv_poll(struct target *target)
{
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
	bits_t bits = read_bits(target);

	if (bits.haltnot && bits.interrupt) {
		target->state = TARGET_DEBUG_RUNNING;
	} else if (bits.haltnot && !bits.interrupt) {
		target->state = TARGET_HALTED;
	} else if (!bits.haltnot && bits.interrupt) {
		// Target is halting. There is no state for that, so don't change anything.
	} else if (!bits.haltnot && !bits.interrupt) {
		target->state = TARGET_RUNNING;
	}

	return ERROR_OK;
}

static int riscv_halt(struct target *target)
{
	LOG_DEBUG("riscv_halt()");
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
	dram_write32(target, 0, csrsi(CSR_DCSR, DCSR_HALT), false);
	dram_write_jump(target, 1, true);

	return ERROR_OK;
}

static int riscv_resume(struct target *target, int current, uint32_t address,
		int handle_breakpoints, int debug_execution)
{
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
	return resume(target, current, address, handle_breakpoints,
			debug_execution, false);
}

static int riscv_step(struct target *target, int current, uint32_t address,
		int handle_breakpoints)
{
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
	return resume(target, current, address, handle_breakpoints, 0, true);
}

static int riscv_assert_reset(struct target *target)
{
	// TODO: Maybe what I implemented here is more like soft_reset_halt()?

	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	// The only assumption we can make is that the TAP was reset.
	if (wait_for_debugint_clear(target) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
		return ERROR_FAIL;
	}

	// Not sure what we should do when there are multiple cores.
	// Here just reset the single hart we're talking to.
	uint32_t dcsr = DCSR_EBREAKM | DCSR_EBREAKH | DCSR_EBREAKS |
		DCSR_EBREAKU | DCSR_HALT;
	if (target->reset_halt) {
		dcsr |= DCSR_NDRESET;
	} else {
		dcsr |= DCSR_FULLRESET;
	}
	dram_write32(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16), false);
	dram_write32(target, 1, csrw(S0, CSR_DCSR), false);
	// We shouldn't actually need the jump because a reset should happen.
	dram_write_jump(target, 2, false);
	dram_write32(target, 4, dcsr, true);

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

	return ERROR_OK;
}

static int riscv_write_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
	// TODO: save/restore T0

	// Set up the address.
	dram_write32(target, 0, lw(T0, ZERO, DEBUG_RAM_START + 16), false);
	dram_write_jump(target, 1, false);
	dram_write32(target, 4, address, true);

	if (wait_for_debugint_clear(target) != ERROR_OK) {
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
	}

	return ERROR_OK;
}

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

struct target_type riscv_target = {
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
};
