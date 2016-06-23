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

#define DIM(x)		(sizeof(x)/sizeof(*x))

#define CSR_TDRSELECT			0x7a0
#define CSR_TDRDATA1			0x7a1
#define CSR_TDRDATA2			0x7a2
#define CSR_TDRDATA3			0x7a3

#define CSR_BPCONTROL_X			(1<<0)
#define CSR_BPCONTROL_W			(1<<1)
#define CSR_BPCONTROL_R			(1<<2)
#define CSR_BPCONTROL_U			(1<<3)
#define CSR_BPCONTROL_S			(1<<4)
#define CSR_BPCONTROL_H			(1<<5)
#define CSR_BPCONTROL_M			(1<<6)
#define CSR_BPCONTROL_BPMATCH	(0xf<<7)
#define CSR_BPCONTROL_BPACTION	(0xff<<11)

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

typedef enum {
	RE_OK,
	RE_FAIL,
	RE_AGAIN
} riscv_error_t;

/*** Debug Bus registers. ***/

#define DMCONTROL				0x10
#define DMCONTROL_INTERRUPT		(((uint64_t)1)<<33)
#define DMCONTROL_HALTNOT		(((uint64_t)1)<<32)
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

#define MAX_HWBPS			16
#define DRAM_CACHE_SIZE		16

struct memory_cache_line {
	uint32_t data;
	bool valid;
	bool dirty;
};

typedef struct {
	/* Number of address bits in the dbus register. */
	uint8_t addrbits;
	/* Width of a GPR (and many other things) in bits. */
	uint8_t xlen;
	/* Number of words in Debug RAM. */
	unsigned int dramsize;
	uint32_t dcsr;
	uint32_t dpc;

	struct memory_cache_line dram_cache[DRAM_CACHE_SIZE];

	struct reg *reg_list;
	/* Single buffer that contains all register names, instead of calling
	 * malloc for each register. Needs to be freed when reg_list is freed. */
	char *reg_names;
	/* Single buffer that contains all register values. */
	void *reg_values;

	// For each physical hwbp, contains ~0 if the hwbp is available, or the
	// unique_id of the breakpoint that is using it.
	uint32_t hwbp_unique_id[MAX_HWBPS];

	// This value is incremented every time a dbus access comes back as "busy".
	// It's used to determine how many run-test/idle cycles to feed the target
	// in between accesses.
	unsigned int dbus_busy_delay;

	// This value is incremented every time we read the debug interrupt as
	// high.  It's used to add extra run-test/idle cycles after setting debug
	// interrupt high, so ideally we never have to perform a whole extra scan
	// before the interrupt is cleared.
	unsigned int interrupt_high_delay;

	// This cache is write-through, and always valid when the target is halted.
	uint32_t gpr_cache[32];
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
	LOG_DEBUG("  last_committed_time=0x%" PRIx64, buf_get_u64(in, 192, 64));
	LOG_DEBUG("  3bits=0x%x", buf_get_u32(in, 256, 3));

	return 0;
}
#endif

static void increase_dbus_busy_delay(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	info->dbus_busy_delay++;
	LOG_INFO("Increment dbus_busy_delay to %d", info->dbus_busy_delay);
}

static void increase_interrupt_high_delay(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	info->interrupt_high_delay++;
	LOG_INFO("Increment interrupt_high_delay to %d", info->interrupt_high_delay);
}

static void add_dbus_scan(struct target *target, struct scan_field *field,
		uint8_t *out_value, uint8_t *in_value, dbus_op_t op, uint16_t address, 
		uint64_t data)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	LOG_DEBUG("op=%d address=0x%02x data=0x%09" PRIx64, op, address, data);

	field->num_bits = info->addrbits + DBUS_OP_SIZE + DBUS_DATA_SIZE;
	field->in_value = in_value;
	field->out_value = out_value;

	buf_set_u64(out_value, DBUS_OP_START, DBUS_OP_SIZE, op);
	buf_set_u64(out_value, DBUS_DATA_START, DBUS_DATA_SIZE, data);
	buf_set_u64(out_value, DBUS_ADDRESS_START, info->addrbits, address);

	jtag_add_dr_scan(target->tap, 1, field, TAP_IDLE);

	// TODO: 1 should come from the dtminfo register
	int idle_count = 1 + info->dbus_busy_delay;
	if (data & DMCONTROL_INTERRUPT) {
		idle_count += info->interrupt_high_delay;
	}

	jtag_add_runtest(idle_count, TAP_IDLE);
}

static dbus_status_t dbus_scan(struct target *target, uint16_t *address_in,
		uint64_t *data_in, dbus_op_t op, uint16_t address_out, uint64_t data_out)
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
	buf_set_u64(out, DBUS_ADDRESS_START, info->addrbits, address_out);

	/* Assume dbus is already selected. */
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);
	jtag_add_runtest(1, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("dbus_scan failed jtag scan");
		return retval;
	}

	if (data_in) {
		*data_in = buf_get_u64(in, DBUS_DATA_START, DBUS_DATA_SIZE);
	}

	if (address_in) {
		*address_in = buf_get_u32(in, DBUS_ADDRESS_START, info->addrbits);
	}

	static const char *op_string[] = {"nop", "r", "w", "cw"};
	static const char *status_string[] = {"+", "nw", "F", "b"};
	/*
	LOG_DEBUG("vvv $display(\"hardware: dbus scan %db %s %01x:%08x @%02x -> %s %01x:%08x @%02x\");",
			field.num_bits,
			op_string[buf_get_u32(out, 0, 2)],
			buf_get_u32(out, 34, 2), buf_get_u32(out, 2, 32),
			buf_get_u32(out, 36, info->addrbits),
			status_string[buf_get_u32(in, 0, 2)],
			buf_get_u32(in, 34, 2), buf_get_u32(in, 2, 32),
			buf_get_u32(in, 36, info->addrbits));
			*/
	LOG_DEBUG("dbus scan %db %s %01x:%08x @%02x -> %s %01x:%08x @%02x",
			field.num_bits,
			op_string[buf_get_u32(out, 0, 2)],
			buf_get_u32(out, 34, 2), buf_get_u32(out, 2, 32),
			buf_get_u32(out, 36, info->addrbits),
			status_string[buf_get_u32(in, 0, 2)],
			buf_get_u32(in, 34, 2), buf_get_u32(in, 2, 32),
			buf_get_u32(in, 36, info->addrbits));

	//debug_scan(target);

	return buf_get_u32(in, DBUS_OP_START, DBUS_OP_SIZE);
}

static uint64_t dbus_read(struct target *target, uint16_t address)
{
	uint64_t value;
	dbus_status_t status;
	uint16_t address_in;

	do {
		do {
			status = dbus_scan(target, &address_in, &value, DBUS_OP_READ, address, 0);
		} while (status == DBUS_STATUS_BUSY);
	} while (address_in != address);

	return value;
}

static void dbus_write(struct target *target, uint16_t address, uint64_t value)
{
	dbus_status_t status = DBUS_STATUS_BUSY;
	while (status == DBUS_STATUS_BUSY) {
		status = dbus_scan(target, NULL, NULL, DBUS_OP_WRITE, address, value);
	}
	if (status != DBUS_STATUS_SUCCESS) {
		LOG_ERROR("dbus_write failed write 0x%" PRIx64 " to 0x%x; status=%d\n",
				value, address, status);
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
	uint16_t address = dram_address(index);
	uint32_t value = dbus_read(target, address);
	return value;
}

static void dram_write32(struct target *target, unsigned int index, uint32_t value,
		bool set_interrupt)
{
	uint64_t dbus_value = DMCONTROL_HALTNOT | value;
	if (set_interrupt)
		dbus_value |= DMCONTROL_INTERRUPT;
	dbus_write(target, dram_address(index), dbus_value);
}

/** Read the haltnot and interrupt bits. */
static bits_t read_bits(struct target *target)
{
	uint64_t value;
	dbus_status_t status;
	uint16_t address_in;

	do {
		do {
			status = dbus_scan(target, &address_in, &value, DBUS_OP_READ, 0, 0);
		} while (status == DBUS_STATUS_BUSY);
	} while (address_in > 0x10 && address_in != DMCONTROL);

	bits_t result = {
		.haltnot = get_field(value, DMCONTROL_HALTNOT),
		.interrupt = get_field(value, DMCONTROL_INTERRUPT)
	};
	return result;
}

static int wait_for_debugint_clear(struct target *target, bool ignore_first)
{
	time_t start = time(NULL);
	if (ignore_first) {
		// Throw away the results of the first read, since they'll contain the
		// result of the read that happened just before debugint was set.
		// (Assuming the last scan before calling this function was one that
		// sets debugint.)
		read_bits(target);
	}
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

static void cache_set(struct target *target, unsigned int index, uint32_t data)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	if (info->dram_cache[index].valid &&
			info->dram_cache[index].data == data) {
		// This is already preset on the target.
		LOG_DEBUG("Cache hit at 0x%x for data 0x%x", index, data);
		return;
	}
	info->dram_cache[index].data = data;
	info->dram_cache[index].valid = true;
	info->dram_cache[index].dirty = true;
}

static void cache_set_jump(struct target *target, unsigned int index)
{
	cache_set(target, index,
			jal(0, (uint32_t) (DEBUG_ROM_RESUME - (DEBUG_RAM_START + 4*index))));
}

static void dump_debug_ram(struct target *target)
{
	for (unsigned int i = 0; i < 16; i++) {
		uint32_t value = dram_read32(target, i);
		LOG_ERROR("Debug RAM 0x%x: 0x%08x", i, value);
	}
}

/* Call this if the code you just ran writes to debug RAM entries 0 through 3. */
static void cache_invalidate(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	for (unsigned int i = 0; i < DRAM_CACHE_SIZE; i++) {
		info->dram_cache[i].valid = false;
		info->dram_cache[i].dirty = false;
	}
}

/* Called by cache_write() after the program has run. Also call this if you're
 * running programs without calling cache_write(). */
static void cache_clean(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	for (unsigned int i = 0; i < DRAM_CACHE_SIZE; i++) {
		if (i >= 4) {
			info->dram_cache[i].valid = false;
		} else {
			info->dram_cache[i].dirty = false;
		}
	}
}

/** Write cache to the target, and optionally run the program. */
static int cache_write(struct target *target, unsigned int address, bool run)
{
	LOG_DEBUG("enter");
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	uint8_t in[(DRAM_CACHE_SIZE + 2) * 8] = {0};
	uint8_t out[(DRAM_CACHE_SIZE + 2) * 8];
	struct scan_field field[DRAM_CACHE_SIZE + 2];

	unsigned int last = DRAM_CACHE_SIZE;
	for (unsigned int i = 0; i < DRAM_CACHE_SIZE; i++) {
		if (info->dram_cache[i].dirty) {
			assert(i < info->dramsize);
			last = i;
		}
	}

	unsigned int scan = 0;

	if (last == DRAM_CACHE_SIZE) {
		// Nothing needs to be written to RAM.
		dbus_write(target, DMCONTROL, DMCONTROL_HALTNOT | DMCONTROL_INTERRUPT);

	} else {
		for (unsigned int i = 0; i < DRAM_CACHE_SIZE; i++) {
			if (info->dram_cache[i].dirty) {
				uint64_t data = DMCONTROL_HALTNOT | info->dram_cache[i].data;
				if (i == last && run) {
					data |= DMCONTROL_INTERRUPT;
				}
				add_dbus_scan(target, &field[scan], out + 8*scan, in + 8*scan,
						DBUS_OP_WRITE, i, data);

				scan++;
			}
		}
	}

	// Throw away the results of the first read, since it'll contain the result
	// of the read that happened just before debugint was set.
	add_dbus_scan(target, &field[scan], out + 8*scan, NULL, DBUS_OP_READ,
			address, DMCONTROL_HALTNOT);
	scan++;

	// This scan contains the results of the read the caller requested, as well
	// as an interrupt bit worth looking at.
	add_dbus_scan(target, &field[scan], out + 8*scan, in + 8*scan, DBUS_OP_READ,
			address, DMCONTROL_HALTNOT);
	scan++;

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("JTAG execute failed.");
		return retval;
	}

	int errors = 0;
	for (unsigned int i = 0; i < scan; i++) {
		dbus_status_t status = buf_get_u32(in + 8*i, DBUS_OP_START, DBUS_OP_SIZE);
		switch (status) {
			case DBUS_STATUS_SUCCESS:
				break;
			case DBUS_STATUS_NO_WRITE:
				LOG_ERROR("Got no-write response to unconditional write. Hardware error?");
				return ERROR_FAIL;
			case DBUS_STATUS_FAILED:
				LOG_ERROR("Debug RAM write failed. Hardware error?");
				return ERROR_FAIL;
			case DBUS_STATUS_BUSY:
				errors++;
				break;
		}
		LOG_DEBUG("read scan=%d result=%d data=%09" PRIx64 " address=%02x",
				i,
				buf_get_u32(in + 8*i, DBUS_OP_START, DBUS_OP_SIZE),
				buf_get_u64(in + 8*i, DBUS_DATA_START, DBUS_DATA_SIZE),
				buf_get_u32(in + 8*i, DBUS_ADDRESS_START, info->addrbits));
	}

	if (errors) {
		increase_dbus_busy_delay(target);

		// Try again, using the slow careful code.
		for (unsigned int i = 0; i < DRAM_CACHE_SIZE; i++) {
			if (i == last && run) {
				dram_write32(target, last, info->dram_cache[last].data, true);
			} else {
				dram_write32(target, i, info->dram_cache[i].data, false);
			}
			info->dram_cache[i].dirty = false;
		}
		cache_clean(target);

		if (wait_for_debugint_clear(target, true) != ERROR_OK) {
			LOG_ERROR("Debug interrupt didn't clear.");
			dump_debug_ram(target);
			return ERROR_FAIL;
		}

	} else {
		cache_clean(target);

		int interrupt = buf_get_u32(in + 8*(scan-1), DBUS_DATA_START + 33, 1);
		if (interrupt) {
			increase_interrupt_high_delay(target);
			// Slow path wait for it to clear.
			if (wait_for_debugint_clear(target, false) != ERROR_OK) {
				LOG_ERROR("Debug interrupt didn't clear.");
				dump_debug_ram(target);
				return ERROR_FAIL;
			}
		} else {
			// We read a useful value in that last scan.
			unsigned int read_addr = buf_get_u32(in + 8*(scan-1), DBUS_ADDRESS_START, info->addrbits);
			if (read_addr != address) {
				LOG_INFO("Got data from 0x%x but expected it from 0x%x",
						read_addr, address);
			}
			info->dram_cache[read_addr].data =
				buf_get_u64(in + 8*(scan-1), DBUS_DATA_START, DBUS_DATA_SIZE);
			info->dram_cache[read_addr].valid = true;
		}
	}

	LOG_DEBUG("exit");

	return ERROR_OK;
}

uint32_t cache_get32(struct target *target, unsigned int address)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	if (!info->dram_cache[address].valid) {
		info->dram_cache[address].data = dram_read32(target, address);
		info->dram_cache[address].valid = true;
	}
	return info->dram_cache[address].data;
}

#if 0
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

static int wait_and_read(struct target *target, uint32_t *data, uint16_t address)
{
	time_t start = time(NULL);
	// Throw away the results of the first read, since they'll contain the
	// result of the read that happened just before debugint was set. (Assuming
	// the last scan before calling this function was one that sets debugint.)
	dbus_scan(target, NULL, NULL, DBUS_OP_READ, address, 0);

	while (1) {
		uint64_t dbus_value = dbus_read(target, address);
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
	cache_set(target, 0, csrr(S0, csr));
	cache_set(target, 1, sw(S0, ZERO, DEBUG_RAM_START + 16));
	cache_set_jump(target, 2);
	if (cache_write(target, 4, true) != ERROR_OK) {
		return ERROR_FAIL;
	}
	*value = cache_get32(target, 4);

	return ERROR_OK;
}

static int write_csr(struct target *target, uint32_t csr, uint32_t value)
{
	cache_set(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16));
	cache_set(target, 1, csrw(S0, csr));
	cache_set_jump(target, 2);
	cache_set(target, 4, value);
	if (cache_write(target, 4, true) != ERROR_OK) {
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int write_gpr(struct target *target, unsigned int gpr, uint32_t value)
{
	cache_set(target, 0, lw(gpr, ZERO, DEBUG_RAM_START + 16));
	cache_set_jump(target, 1);
	cache_set(target, 4, value);
	if (cache_write(target, 4, true) != ERROR_OK) {
		return ERROR_FAIL;
	}
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

	// TODO: check if dpc is dirty (which also is true if an exception was hit
	// at any time)
	cache_set(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16));
	cache_set(target, 1, csrw(S0, CSR_DPC));
	cache_set_jump(target, 2);
	cache_set(target, 4, info->dpc);
	if (cache_write(target, 4, true) != ERROR_OK) {
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

	cache_invalidate(target);

	if (wait_for_debugint_clear(target, true) != ERROR_OK) {
		LOG_ERROR("Debug interrupt didn't clear.");
		return ERROR_FAIL;
	}

	target->state = TARGET_RUNNING;
	for (unsigned int i = 0; i < 32; i++) {
		info->gpr_cache[i] = 0xbadbad;
	}

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

	if (reg->number <= REG_XPR31) {
		buf_set_u64(reg->value, 0, info->xlen, info->gpr_cache[reg->number]);
		LOG_DEBUG("%s=0x%x", reg->name, info->gpr_cache[reg->number]);
		return ERROR_OK;
	} else if (reg->number == REG_PC) {
		buf_set_u32(reg->value, 0, 32, info->dpc);
		LOG_DEBUG("%s=0x%x (cached)", reg->name, info->dpc);
		return ERROR_OK;
	} else if (reg->number >= REG_FPR0 && reg->number <= REG_FPR31) {
		cache_set(target, 0, fsw(reg->number - REG_FPR0, 0, DEBUG_RAM_START + 16));
		cache_set_jump(target, 1);
	} else if (reg->number >= REG_CSR0 && reg->number <= REG_CSR4095) {
		cache_set(target, 0, csrr(S0, reg->number - REG_CSR0));
		cache_set(target, 1, sw(S0, ZERO, DEBUG_RAM_START + 16));
		cache_set_jump(target, 2);
	} else {
		LOG_ERROR("Don't know how to read register %d (%s)", reg->number, reg->name);
		return ERROR_FAIL;
	}

	if (cache_write(target, 4, true) != ERROR_OK) {
		return ERROR_FAIL;
	}

	uint32_t value = cache_get32(target, 4);
	if (reg->number < 32 && info->gpr_cache[reg->number] != value) {
		LOG_ERROR("cached value for %s is 0x%x but just read 0x%x",
				reg->name, info->gpr_cache[reg->number], value);
		assert(info->gpr_cache[reg->number] == value);
	}

	uint32_t exception = cache_get32(target, info->dramsize-1);
	if (exception) {
		LOG_ERROR("Got exception 0x%x when reading register %d", exception,
				reg->number);
		return ERROR_FAIL;
	}

	LOG_DEBUG("%s=0x%x", reg->name, value);
	buf_set_u32(reg->value, 0, 32, value);

	return ERROR_OK;
}

static int register_write(struct target *target, unsigned int number,
		uint32_t value)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	if (number == S0) {
		cache_set(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16));
		cache_set(target, 1, csrw(S0, CSR_DSCRATCH));
		cache_set_jump(target, 2);
	} else if (number == S1) {
		cache_set(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16));
		cache_set(target, 1, sw(S0, ZERO, DEBUG_RAM_START + 4 * info->dramsize - 4));
		cache_set_jump(target, 2);
	} else if (number <= REG_XPR31) {
		cache_set(target, 0, lw(number - REG_XPR0, ZERO, DEBUG_RAM_START + 16));
		cache_set_jump(target, 1);
	} else if (number == REG_PC) {
		info->dpc = value;
		return ERROR_OK;
	} else if (number >= REG_FPR0 && number <= REG_FPR31) {
		cache_set(target, 0, flw(number - REG_FPR0, 0, DEBUG_RAM_START + 16));
		cache_set_jump(target, 1);
	} else if (number >= REG_CSR0 && number <= REG_CSR4095) {
		cache_set(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16));
		cache_set(target, 1, csrw(S0, number - REG_CSR0));
		cache_set_jump(target, 2);
	} else {
		LOG_ERROR("Don't know how to read register %d", number);
		return ERROR_FAIL;
	}

	cache_set(target, 4, value);
	if (cache_write(target, 4, true) != ERROR_OK) {
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int register_set(struct reg *reg, uint8_t *buf)
{
	struct target *target = (struct target *) reg->arch_info;
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	uint32_t value = buf_get_u32(buf, 0, 32);

	LOG_DEBUG("write 0x%x to %s", value, reg->name);
	info->gpr_cache[reg->number] = value;

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

	memset(info->hwbp_unique_id, 0xff, sizeof(info->hwbp_unique_id));

	return ERROR_OK;
}

static void riscv_deinit_target(struct target *target)
{
	LOG_DEBUG("riscv_deinit_target()");
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	free(info);
	target->arch_info = NULL;
}

static int riscv_halt(struct target *target)
{
	LOG_DEBUG("riscv_halt()");
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	cache_set(target, 0, csrsi(CSR_DCSR, DCSR_HALT));
	cache_set(target, 1, csrr(S0, CSR_MHARTID));
	cache_set(target, 2, sw(S0, ZERO, SETHALTNOT));
	cache_set_jump(target, 3);

	if (cache_write(target, 4, true) != ERROR_OK) {
		LOG_ERROR("cache_write() failed.");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int riscv_step(struct target *target, int current, uint32_t address,
		int handle_breakpoints)
{
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);
	// Hardware single step doesn't exist yet.
#if 1
	return resume(target, current, address, handle_breakpoints, 0, true);
#else
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	uint32_t next_pc = info->dpc + 4;
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

#if 0
static void write_constants(struct target *target)
{
	dram_write32(target, 0, lui(S0, 0x70002), false);
	dram_write32(target, 1, lui(S1, 0xccccc), false);
	dram_write32(target, 2, sw(S0, ZERO, DEBUG_RAM_START + 0), false);
	dram_write32(target, 3, sw(S1, ZERO, DEBUG_RAM_START + 4), false);
	dram_write_jump(target, 4, true);

	if (wait_for_debugint_clear(target, true) != ERROR_OK) {
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
	LOG_DEBUG("dtminfo=0x%x", dtminfo);
	LOG_DEBUG("  addrbits=%d", get_field(dtminfo, DTMINFO_ADDRBITS));
	LOG_DEBUG("  version=%d", get_field(dtminfo, DTMINFO_VERSION));
	// TODO: Add support for the idle field, once it's implemented in the FPGA
	// image.
	if (dtminfo == 0) {
		LOG_ERROR("dtminfo is 0. Check JTAG connectivity/board power.");
		return ERROR_FAIL;
	}
	if (get_field(dtminfo, DTMINFO_VERSION) != 0) {
		LOG_ERROR("Unsupported DTM version %d. (dtminfo=0x%x)",
				get_field(dtminfo, DTMINFO_VERSION), dtminfo);
		return ERROR_FAIL;
	}

	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	info->addrbits = get_field(dtminfo, DTMINFO_ADDRBITS);

	uint32_t dminfo = dbus_read(target, DMINFO);
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
		LOG_ERROR("OpenOCD only supports Debug Module version 1, not %d "
				"(dminfo=0x%x)", get_field(dminfo, DMINFO_VERSION), dminfo);
		return ERROR_FAIL;
	}

	info->dramsize = get_field(dminfo, DMINFO_DRAMSIZE) + 1;

	if (get_field(dminfo, DMINFO_AUTHTYPE) != 0) {
		LOG_ERROR("Authentication required by RISC-V core but not "
				"supported by OpenOCD. dminfo=0x%x", dminfo);
		return ERROR_FAIL;
	}

	// Figure out XLEN.
	cache_set(target, 0, xori(S1, ZERO, -1));
	// 0xffffffff  0xffffffff:ffffffff  0xffffffff:ffffffff:ffffffff:ffffffff
	cache_set(target, 1, srli(S1, S1, 31));
	// 0x00000001  0x00000001:ffffffff  0x00000001:ffffffff:ffffffff:ffffffff
	cache_set(target, 2, sw(S1, ZERO, DEBUG_RAM_START));
	cache_set(target, 3, srli(S1, S1, 31));
	// 0x00000000  0x00000000:00000003  0x00000000:00000003:ffffffff:ffffffff
	cache_set(target, 4, sw(S1, ZERO, DEBUG_RAM_START + 4));
	cache_set_jump(target, 5);

	cache_write(target, 0, true);
	cache_invalidate(target);

#if 0
	// TODO
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
#endif

	uint32_t word0 = cache_get32(target, 0);
	uint32_t word1 = cache_get32(target, 1);
	if (word0 == 1 && word1 == 0) {
		info->xlen = 32;
	} else if (word0 == 0xffffffff && word1 == 3) {
		info->xlen = 64;
	} else if (word0 == 0xffffffff && word1 == 0xffffffff) {
		info->xlen = 128;
	} else {
		uint32_t exception = cache_get32(target, info->dramsize-1);
		LOG_ERROR("Failed to discover xlen; word0=0x%x, word1=0x%x, exception=0x%x",
				word0, word1, exception);
		dump_debug_ram(target);
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

static riscv_error_t handle_halt_routine(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	const unsigned int max_scan = 256;
	uint8_t *in = malloc(max_scan * 8);
	uint8_t *out = malloc(max_scan * 8);
	struct scan_field *field = calloc(max_scan, sizeof(struct scan_field));
	unsigned int scan = 0;

	// Read all GPRs as fast as we can, because gdb is going to ask for them
	// anyway. Reading them one at a time is much slower.
	// TODO

	// Write the jump back to address 1.
	add_dbus_scan(target, &field[scan], out + 8*scan, in + 8*scan,
			DBUS_OP_WRITE, 1, DMCONTROL_HALTNOT |
			jal(0, (uint32_t) (DEBUG_ROM_RESUME - (DEBUG_RAM_START + 4*1))));
	scan++;
	for (int reg = 1; reg < 32; reg++) {
		if (reg == S0 || reg == S1) {
			continue;
		}

		// Write store instruction.
		add_dbus_scan(target, &field[scan], out + 8*scan, in + 8*scan,
				DBUS_OP_WRITE, 0, DMCONTROL_INTERRUPT | DMCONTROL_HALTNOT |
				sw(reg, ZERO, DEBUG_RAM_START + 16));
		scan++;

		// Read value.
		add_dbus_scan(target, &field[scan], out + 8*scan, in + 8*scan,
				DBUS_OP_READ, 4, DMCONTROL_HALTNOT);
		scan++;
		assert(scan < max_scan);
	}

	// Write store of s0 at index 1.
	add_dbus_scan(target, &field[scan], out + 8*scan, in + 8*scan,
			DBUS_OP_WRITE, 1, DMCONTROL_HALTNOT |
			sw(S0, ZERO, DEBUG_RAM_START + 16));
	scan++;
	// Write jump at index 2.
	add_dbus_scan(target, &field[scan], out + 8*scan, in + 8*scan,
			DBUS_OP_WRITE, 2, DMCONTROL_HALTNOT |
			jal(0, (uint32_t) (DEBUG_ROM_RESUME - (DEBUG_RAM_START + 4*2))));
		cache_set(target, 0, csrr(S0, CSR_DSCRATCH));
	scan++;

	// Read S1 from debug RAM
	add_dbus_scan(target, &field[scan], out + 8*scan, in + 8*scan,
			DBUS_OP_WRITE, 0, DMCONTROL_INTERRUPT | DMCONTROL_HALTNOT |
			lw(S0, ZERO, DEBUG_RAM_START + 4 * info->dramsize - 4));
	scan++;
	// Read value.
	add_dbus_scan(target, &field[scan], out + 8*scan, in + 8*scan,
			DBUS_OP_READ, 4, DMCONTROL_HALTNOT);
	scan++;

	// Read S0 from dscratch
	unsigned int csr[] = {CSR_DSCRATCH, CSR_DPC, CSR_DCSR};
	for (unsigned int i = 0; i < DIM(csr); i++) {
		add_dbus_scan(target, &field[scan], out + 8*scan, in + 8*scan,
				DBUS_OP_WRITE, 0, DMCONTROL_INTERRUPT | DMCONTROL_HALTNOT |
				csrr(S0, csr[i]));
		scan++;
		// Read value.
		add_dbus_scan(target, &field[scan], out + 8*scan, in + 8*scan,
				DBUS_OP_READ, 4, DMCONTROL_HALTNOT);
		scan++;
		assert(scan < max_scan);
	}

	// Final read to get the last value out.
	add_dbus_scan(target, &field[scan], out + 8*scan, in + 8*scan,
			DBUS_OP_READ, 4, DMCONTROL_HALTNOT);
	scan++;
	assert(scan < max_scan);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("JTAG execute failed: %d", retval);
		goto error;
	}

	unsigned int dbus_busy = 0;
	unsigned int interrupt_set = 0;
	unsigned result = 0;
	info->gpr_cache[0] = 0;
	// The first scan result is the result from something old we don't care
	// about.
	for (unsigned int i = 1; i < scan && dbus_busy == 0; i++) {
		dbus_status_t status = buf_get_u32(in + 8*i, DBUS_OP_START, DBUS_OP_SIZE);
		uint64_t data = buf_get_u64(in + 8*i, DBUS_DATA_START, DBUS_DATA_SIZE);
		uint32_t address = buf_get_u32(in + 8*i, DBUS_ADDRESS_START, info->addrbits);
		LOG_DEBUG("read scan=%d result=%d data=%09" PRIx64 " address=%02x",
				i, status, data, address);
		switch (status) {
			case DBUS_STATUS_SUCCESS:
				break;
			case DBUS_STATUS_NO_WRITE:
				LOG_ERROR("Got no-write response without conditional write. Hardware error?");
				goto error;
			case DBUS_STATUS_FAILED:
				LOG_ERROR("Debug access failed. Hardware error?");
				goto error;
			case DBUS_STATUS_BUSY:
				dbus_busy++;
				break;
		}
		if (data & DMCONTROL_INTERRUPT) {
			interrupt_set++;
			break;
		}
		if (address == 4) {
			switch (result) {
				case 0: info->gpr_cache[1] = data; break;
				case 1: info->gpr_cache[2] = data; break;
				case 2: info->gpr_cache[3] = data; break;
				case 3: info->gpr_cache[4] = data; break;
				case 4: info->gpr_cache[5] = data; break;
				case 5: info->gpr_cache[6] = data; break;
				case 6: info->gpr_cache[7] = data; break;
						// S0
						// S1
				case 7: info->gpr_cache[10] = data; break;
				case 8: info->gpr_cache[11] = data; break;
				case 9: info->gpr_cache[12] = data; break;
				case 10: info->gpr_cache[13] = data; break;
				case 11: info->gpr_cache[14] = data; break;
				case 12: info->gpr_cache[15] = data; break;
				case 13: info->gpr_cache[16] = data; break;
				case 14: info->gpr_cache[17] = data; break;
				case 15: info->gpr_cache[18] = data; break;
				case 16: info->gpr_cache[19] = data; break;
				case 17: info->gpr_cache[20] = data; break;
				case 18: info->gpr_cache[21] = data; break;
				case 19: info->gpr_cache[22] = data; break;
				case 20: info->gpr_cache[23] = data; break;
				case 21: info->gpr_cache[24] = data; break;
				case 22: info->gpr_cache[25] = data; break;
				case 23: info->gpr_cache[26] = data; break;
				case 24: info->gpr_cache[27] = data; break;
				case 25: info->gpr_cache[28] = data; break;
				case 26: info->gpr_cache[29] = data; break;
				case 27: info->gpr_cache[30] = data; break;
				case 28: info->gpr_cache[31] = data; break;
				case 29: info->gpr_cache[S1] = data; break;
				case 30: info->gpr_cache[S0] = data; break;
				case 31: info->dpc = data; break;
				case 32: info->dcsr = data; break;
				default:
						 assert(0);
			}
			result++;
		}
	}

	free(in);
	free(out);
	free(field);

	cache_invalidate(target);

	if (dbus_busy) {
		increase_dbus_busy_delay(target);
		return RE_AGAIN;
	}
	if (interrupt_set) {
		increase_interrupt_high_delay(target);
		return RE_AGAIN;
	}

	return RE_OK;

error:
	free(in);
	free(out);
	free(field);
	return RE_FAIL;
}

static int handle_halt(struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	target->state = TARGET_HALTED;

	riscv_error_t re;
	do {
		re = handle_halt_routine(target);
	} while (re == RE_AGAIN);
	if (re != RE_OK) {
		LOG_ERROR("handle_halt_routine failed");
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

	LOG_DEBUG("halted at 0x%x", info->dpc);

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
	if (wait_for_debugint_clear(target, true) != ERROR_OK) {
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
	cache_invalidate(target);

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
	cache_set(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16));
	switch (size) {
		case 1:
			cache_set(target, 1, lb(S1, S0, 0));
			cache_set(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16));
			break;
		case 2:
			cache_set(target, 1, lh(S1, S0, 0));
			cache_set(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16));
			break;
		case 4:
			cache_set(target, 1, lw(S1, S0, 0));
			cache_set(target, 2, sw(S1, ZERO, DEBUG_RAM_START + 16));
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}
	cache_set_jump(target, 3);
	cache_write(target, 4, false);

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
static int setup_write_memory(struct target *target, uint32_t size)
{
	switch (size) {
		case 1:
			cache_set(target, 0, lb(S0, ZERO, DEBUG_RAM_START + 16));
			cache_set(target, 1, sb(S0, T0, 0));
			break;
		case 2:
			cache_set(target, 0, lh(S0, ZERO, DEBUG_RAM_START + 16));
			cache_set(target, 1, sh(S0, T0, 0));
			break;
		case 4:
			cache_set(target, 0, lw(S0, ZERO, DEBUG_RAM_START + 16));
			cache_set(target, 1, sw(S0, T0, 0));
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}
	cache_set(target, 2, addi(T0, T0, size));
	cache_set_jump(target, 3);
	cache_write(target, 4, false);

	return ERROR_OK;
}

static int riscv_write_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	jtag_add_ir_scan(target->tap, &select_dbus, TAP_IDLE);

	// Set up the address.
	cache_set(target, 0, sw(T0, ZERO, DEBUG_RAM_START + 20));
	cache_set(target, 1, lw(T0, ZERO, DEBUG_RAM_START + 16));
	cache_set_jump(target, 2);
	cache_set(target, 4, address);
	if (cache_write(target, 5, true) != ERROR_OK) {
		return ERROR_FAIL;
	}

	uint32_t t0 = cache_get32(target, 5);

	if (setup_write_memory(target, size) != ERROR_OK) {
		return ERROR_FAIL;
	}

	const int max_batch_size = 256;
	uint8_t *in = malloc(max_batch_size * 8);
	uint8_t *out = malloc(max_batch_size * 8);
	struct scan_field *field = calloc(max_batch_size, sizeof(struct scan_field));

	uint32_t i = 0;
	while (i < count + 1) {
		unsigned int batch_size = MIN(count + 1 - i, max_batch_size);

		for (unsigned int j = 0; j < batch_size; j++) {
			if (i + j == count) {
				// Just insert a read so we can confirm that the last scan
				// succeeded.

				add_dbus_scan(target, &field[j], out + 8*j, in + 8*j,
						DBUS_OP_READ, info->dramsize-1, DMCONTROL_HALTNOT | 0);
			} else {
				// Write the next value and set interrupt.
				uint32_t value;
				uint32_t offset = size * (i + j);
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
						goto error;
				}

				add_dbus_scan(target, &field[j], out + 8*j, in + 8*j,
						DBUS_OP_CONDITIONAL_WRITE, 4, DMCONTROL_HALTNOT | DMCONTROL_INTERRUPT | value);
			}
		}

		int retval = jtag_execute_queue();
		if (retval != ERROR_OK) {
			LOG_ERROR("JTAG execute failed: %d", retval);
			goto error;
		}

		int dbus_busy = 0;
		int execute_busy = 0;
		for (unsigned int j = 0; j < batch_size; j++) {
			dbus_status_t status = buf_get_u32(in + 8*j, DBUS_OP_START, DBUS_OP_SIZE);
			switch (status) {
				case DBUS_STATUS_SUCCESS:
					break;
				case DBUS_STATUS_NO_WRITE:
					execute_busy++;
					break;
				case DBUS_STATUS_FAILED:
					LOG_ERROR("Debug RAM write failed. Hardware error?");
					goto error;
				case DBUS_STATUS_BUSY:
					dbus_busy++;
					break;
			}
			LOG_DEBUG("j=%d data=%09" PRIx64, j, buf_get_u64(in + 8*j, DBUS_DATA_START, DBUS_DATA_SIZE));
		}
		if (dbus_busy) {
			increase_dbus_busy_delay(target);
		}
		if (execute_busy) {
			increase_interrupt_high_delay(target);
		}
		if (dbus_busy || execute_busy) {
			wait_for_debugint_clear(target, false);

			// Retry.
			// Set t0 back to what it should have been at the beginning of this
			// batch.
			LOG_INFO("Retrying memory write starting from 0x%x with more delays", address + size * i);

			cache_clean(target);

			if (write_gpr(target, T0, address + size * i) != ERROR_OK) {
				goto error;
			}

			if (setup_write_memory(target, size) != ERROR_OK) {
				goto error;
			}
		} else {
			i += batch_size;
		}
	}

	free(in);
	free(out);
	free(field);

	cache_clean(target);
	return register_write(target, T0, t0);

error:
	free(in);
	free(out);
	free(field);
	return ERROR_FAIL;
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

		if (wait_for_debugint_clear(target, true) != ERROR_OK) {
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
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	if (breakpoint->type == BKPT_SOFT) {
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

	} else if (breakpoint->type == BKPT_HARD) {
		int i;
		uint32_t tdrdata1;
		uint32_t tdrselect, tdrselect_rb;
		for (i = 0; i < MAX_HWBPS; i++) {
			if (info->hwbp_unique_id[i] == ~0U) {
				// TODO 0x80000000 is a hack until the core supports proper
				// debug hwbps.
				tdrselect = 0x80000000 | i;
				write_csr(target, CSR_TDRSELECT, tdrselect);
				read_csr(target, &tdrselect_rb, CSR_TDRSELECT);
				if (tdrselect_rb != tdrselect) {
					// We've run out of breakpoints.
					LOG_ERROR("Couldn't find an available hardware breakpoint. "
							"(0x%x != 0x%x)", tdrselect, tdrselect_rb);
					return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
				}
				read_csr(target, &tdrdata1, CSR_TDRDATA1);
				if ((tdrdata1 >> (info->xlen - 4)) == 1) {
					break;
				}
			}
		}
		if (i >= MAX_HWBPS) {
			LOG_ERROR("Couldn't find an available hardware breakpoint.");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		LOG_DEBUG("Start using resource %d for bp %d", i, breakpoint->unique_id);

		tdrdata1 |= CSR_BPCONTROL_X;
		tdrdata1 |= CSR_BPCONTROL_U;
		tdrdata1 |= CSR_BPCONTROL_S;
		tdrdata1 |= CSR_BPCONTROL_H;
		tdrdata1 |= CSR_BPCONTROL_M;
		write_csr(target, CSR_TDRDATA1, tdrdata1);
		write_csr(target, CSR_TDRDATA2, breakpoint->address);

		uint32_t tdrdata1_rb;
		read_csr(target, &tdrdata1_rb, CSR_TDRDATA1);
		LOG_DEBUG("tdrdata1=0x%x", tdrdata1_rb);

		if (!(tdrdata1_rb & CSR_BPCONTROL_X)) {
			LOG_ERROR("Breakpoint %d doesn't support execute", i);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		info->hwbp_unique_id[i] = breakpoint->unique_id;
	} else {
        LOG_INFO("OpenOCD only supports hardware and software breakpoints.");
        return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
    }

    breakpoint->set = true;

    return ERROR_OK;
}

static int riscv_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

    if (breakpoint->type == BKPT_SOFT) {
		if (target_write_memory(target, breakpoint->address, breakpoint->length, 1,
					breakpoint->orig_instr) != ERROR_OK) {
			LOG_ERROR("Failed to restore instruction for %d-byte breakpoint at 0x%x",
					breakpoint->length, breakpoint->address);
			return ERROR_FAIL;
		}

	} else if (breakpoint->type == BKPT_HARD) {
		int i;
		for (i = 0; i < MAX_HWBPS; i++) {
			if (info->hwbp_unique_id[i] == breakpoint->unique_id) {
				break;
			}
		}
		if (i >= MAX_HWBPS) {
			LOG_ERROR("Couldn't find the hardware resources used by hardware breakpoint.");
			return ERROR_FAIL;
		}
		LOG_DEBUG("Stop using resource %d for bp %d", i, breakpoint->unique_id);
		write_csr(target, CSR_TDRSELECT, 0x80000000 | i);
		write_csr(target, CSR_TDRDATA1, 0);
		info->hwbp_unique_id[i] = ~0U;

	} else {
		LOG_INFO("OpenOCD only supports hardware and software breakpoints.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->set = false;

	return ERROR_OK;
}

int riscv_arch_state(struct target *target)
{
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

	.arch_state = riscv_arch_state,
};
