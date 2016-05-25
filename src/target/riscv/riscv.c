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
	LOG_DEBUG("address=0x%x, value=0x%lx", address, value);
	return value;
}

static void dbus_write(struct target *target, uint16_t address, uint64_t value)
{
	LOG_DEBUG("address=0x%x, value=0x%lx", address, value);
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
	/* TODO: Can we rely on IR not being messed with between calls into
	 * RISCV code?  Eg. what happens if there are multiple cores and some
	 * other core is accessed? */
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	return buf_get_u32(field.in_value, 0, 32);
}

static uint32_t dram_read32(struct target *target, unsigned int index,
		bool set_interrupt)
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

static void dram_check32(struct target *target, unsigned int index,
		uint32_t expected)
{
	uint32_t actual = dram_read32(target, index, false);
	if (expected != actual) {
		LOG_ERROR("Wrote 0x%x to Debug RAM at %d, but read back 0x%x",
				expected, index, actual);
	}
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

/*** OpenOCD target functions. ***/

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

static int riscv_examine(struct target *target)
{
	LOG_DEBUG("riscv_examine()");
	if (target_was_examined(target)) {
		return ERROR_OK;
	}

	uint32_t dtminfo = dtminfo_read(target);
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	info->addrbits = get_field(dtminfo, DTMINFO_ADDRBITS);

	uint32_t dminfo = dbus_read(target, DMINFO, 0);
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
	dram_check32(target, 0, xori(S1, ZERO, -1));
	dram_check32(target, 1, srli(S1, S1, 31));
	dram_check32(target, 2, sw(S1, ZERO, DEBUG_RAM_START));
	dram_check32(target, 3, srli(S1, S1, 31));
	dram_check32(target, 4, sw(S1, ZERO, DEBUG_RAM_START + 4));

	// Execute.
	dram_write_jump(target, 5, true);

	if (wait_for_debugint_clear(target) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
		return ERROR_FAIL;
	}

	uint32_t word0 = dram_read32(target, 0, false);
	uint32_t word1 = dram_read32(target, 1, false);
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

	target_set_examined(target);

	return ERROR_OK;
}

static int riscv_poll(struct target *target)
{
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
	dram_write32(target, 0, csrsi(CSR_DCSR, DCSR_HALT), false);
	dram_write_jump(target, 1, true);

	return ERROR_OK;
}

static int riscv_resume(struct target *target, int current, uint32_t address,
		int handle_breakpoints, int debug_execution)
{
	return resume(target, current, address, handle_breakpoints,
			debug_execution, false);
}

static int riscv_step(struct target *target, int current, uint32_t address,
		int handle_breakpoints)
{
	return resume(target, current, address, handle_breakpoints, 0, true);
}

static int riscv_assert_reset(struct target *target)
{
	// TODO: Maybe what I implemented here is more like soft_reset_halt()?

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
	if (target->reset_halt) {
		return wait_for_state(target, TARGET_HALTED);
	} else {
		return wait_for_state(target, TARGET_RUNNING);
	}
}

static int riscv_read_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
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
			LOG_ERROR("Unsupported size for read_memory: %d", size);
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

	/* TODO: */
	/* .virt2phys = riscv_virt2phys, */
};
