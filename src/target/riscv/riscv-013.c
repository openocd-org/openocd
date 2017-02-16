/*
 * Support for RISC-V, debug version 0.13, which is currently (2/4/17) the
 * latest draft.
 */

#include <assert.h>
#include <stdlib.h>
#include <time.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "target/algorithm.h"
#include "target_type.h"
#include "log.h"
#include "jtag/jtag.h"
#include "register.h"
#include "breakpoints.h"
#include "helper/time_support.h"
#include "riscv.h"
#include "debug_defines.h"

/**
 * Since almost everything can be accomplish by scanning the dbus register, all
 * functions here assume dbus is already selected. The exception are functions
 * called directly by OpenOCD, which can't assume anything about what's
 * currently in IR. They should set IR to dbus explicitly.
 */

/**
 * Code structure
 *
 * At the bottom of the stack are the OpenOCD JTAG functions:
 * 		jtag_add_[id]r_scan
 * 		jtag_execute_query
 * 		jtag_add_runtest
 *
 * There are a few functions to just instantly shift a register and get its
 * value:
 * 		dtmcontrol_scan
 * 		idcode_scan
 * 		dmi_scan
 *
 * Because doing one scan and waiting for the result is slow, most functions
 * batch up a bunch of dbus writes and then execute them all at once. They use
 * the scans "class" for this:
 * 		scans_new
 * 		scans_delete
 * 		scans_execute
 * 		scans_add_...
 * Usually you new(), call a bunch of add functions, then execute() and look
 * at the results by calling scans_get...()
 *
 * Optimized functions will directly use the scans class above, but slightly
 * lazier code will use the cache functions that in turn use the scans
 * functions:
 * 		cache_get...
 * 		cache_set...
 * 		cache_write
 * cache_set... update a local structure, which is then synced to the target
 * with cache_write(). Only Debug RAM words that are actually changed are sent
 * to the target. Afterwards use cache_get... to read results.
 */

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

#define DIM(x)		(sizeof(x)/sizeof(*x))

// Constants for legacy SiFive hardware breakpoints.
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

#define DBUS						0x11
#define DMI_OP_START				0
#define DMI_OP_SIZE				2
typedef enum {
	DMI_OP_NOP = 0,
	DMI_OP_READ = 1,
	DMI_OP_WRITE = 2
} dmi_op_t;
typedef enum {
	DMI_STATUS_SUCCESS = 0,
	DMI_STATUS_FAILED = 2,
	DMI_STATUS_BUSY = 3
} dmi_status_t;
#define DMI_DATA_START				2
#define DMI_DATA_SIZE				32
#define DMI_ADDRESS_START			34

typedef enum {
	RE_OK,
	RE_FAIL,
	RE_AGAIN
} riscv_error_t;

typedef enum slot {
	SLOT0,
	SLOT1,
	SLOT_LAST,
} slot_t;

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

/*** Info about the core being debugged. ***/

#define DMI_ADDRESS_UNKNOWN	0xffff
#define WALL_CLOCK_TIMEOUT		2

// gdb's register list is defined in riscv_gdb_reg_names gdb/riscv-tdep.c in
// its source tree. We must interpret the numbers the same here.
enum {
	REG_XPR0 = 0,
	REG_XPR31 = 31,
	REG_PC = 32,
	REG_FPR0 = 33,
	REG_FPR31 = 64,
	REG_CSR0 = 65,
	REG_MSTATUS = CSR_MSTATUS + REG_CSR0,
	REG_CSR4095 = 4160,
	REG_PRIV = 4161,
	REG_COUNT
};

#define MAX_HWBPS			16
#define DRAM_CACHE_SIZE		16

struct trigger {
	uint64_t address;
	uint32_t length;
	uint64_t mask;
	uint64_t value;
	bool read, write, execute;
	int unique_id;
};

struct memory_cache_line {
	uint32_t data;
	bool valid;
	bool dirty;
};

typedef struct {
	/* Number of address bits in the dbus register. */
	unsigned abits;
	/* Number of abstract command data registers. */
	unsigned datacount;
	/* Number of words in the Program Buffer. */
	unsigned progsize;
	/* Number of Program Buffer registers. */
	/* Number of words in Debug RAM. */
	unsigned int dramsize;	// TODO: remove
	uint64_t dcsr;
	uint64_t dpc;
	uint64_t misa;
	uint64_t tselect;
	bool tselect_dirty;
	/* The value that mstatus actually has on the target right now. This is not
	 * the value we present to the user. That one may be stored in the
	 * reg_cache. */
	uint64_t mstatus_actual;

	struct memory_cache_line dram_cache[DRAM_CACHE_SIZE];

	/* Single buffer that contains all register names, instead of calling
	 * malloc for each register. Needs to be freed when reg_list is freed. */
	char *reg_names;
	/* Single buffer that contains all register values. */
	void *reg_values;

	// For each physical trigger, contains -1 if the hwbp is available, or the
	// unique_id of the breakpoint/watchpoint that is using it.
	int trigger_unique_id[MAX_HWBPS];

	unsigned int trigger_count;

	// Number of run-test/idle cycles the target requests we do after each dbus
	// access.
	unsigned int dtmcontrol_idle;

	// This value is incremented every time a dbus access comes back as "busy".
	// It's used to determine how many run-test/idle cycles to feed the target
	// in between accesses.
	unsigned int dmi_busy_delay;

	// This value is incremented every time we read the debug interrupt as
	// high.  It's used to add extra run-test/idle cycles after setting debug
	// interrupt high, so ideally we never have to perform a whole extra scan
	// before the interrupt is cleared.
	unsigned int interrupt_high_delay;

	bool need_strict_step;
	bool never_halted;
} riscv013_info_t;

typedef struct {
	bool haltnot;
	bool interrupt;
} bits_t;

/*** Necessary prototypes. ***/

static int poll_target(struct target *target, bool announce);
static int riscv013_poll(struct target *target);
static int register_get(struct reg *reg);

/*** Utility functions. ***/

#define DEBUG_LENGTH	264

static riscv013_info_t *get_info(const struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	return (riscv013_info_t *) info->version_specific;
}

static unsigned int slot_offset(const struct target *target, slot_t slot)
{
	riscv013_info_t *info = get_info(target);
	switch (xlen(target)) {
		case 32:
			switch (slot) {
				case SLOT0: return 4;
				case SLOT1: return 5;
				case SLOT_LAST: return info->dramsize-1;
			}
		case 64:
			switch (slot) {
				case SLOT0: return 4;
				case SLOT1: return 6;
				case SLOT_LAST: return info->dramsize-2;
			}
	}
	LOG_ERROR("slot_offset called with xlen=%d, slot=%d",
			xlen(target), slot);
	assert(0);
}

static uint16_t dram_address(unsigned int index)
{
	if (index < 0x10)
		return index;
	else
		return 0x40 + index - 0x10;
}

static void select_dmi(struct target *target)
{
	static uint8_t ir_dmi[1] = {DTM_DMI};
	struct scan_field field = {
		.num_bits = target->tap->ir_length,
		.out_value = ir_dmi,
		.in_value = NULL,
		.check_value = NULL,
		.check_mask = NULL
	};

	jtag_add_ir_scan(target->tap, &field, TAP_IDLE);
}

static uint32_t dtmcontrol_scan(struct target *target, uint32_t out)
{
	struct scan_field field;
	uint8_t in_value[4];
	uint8_t out_value[4];

	buf_set_u32(out_value, 0, 32, out);

	jtag_add_ir_scan(target->tap, &select_dtmcontrol, TAP_IDLE);

	field.num_bits = 32;
	field.out_value = out_value;
	field.in_value = in_value;
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	/* Always return to dmi. */
	select_dmi(target);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	uint32_t in = buf_get_u32(field.in_value, 0, 32);
	LOG_DEBUG("DTMCONTROL: 0x%x -> 0x%x", out, in);

	return in;
}

static uint32_t idcode_scan(struct target *target)
{
	struct scan_field field;
	uint8_t in_value[4];

	jtag_add_ir_scan(target->tap, &select_idcode, TAP_IDLE);

	field.num_bits = 32;
	field.out_value = NULL;
	field.in_value = in_value;
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	/* Always return to dmi. */
	select_dmi(target);

	uint32_t in = buf_get_u32(field.in_value, 0, 32);
	LOG_DEBUG("IDCODE: 0x0 -> 0x%x", in);

	return in;
}

static void increase_dmi_busy_delay(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	info->dmi_busy_delay += info->dmi_busy_delay / 10 + 1;
	LOG_INFO("dtmcontrol_idle=%d, dmi_busy_delay=%d, interrupt_high_delay=%d",
			info->dtmcontrol_idle, info->dmi_busy_delay,
			info->interrupt_high_delay);

	dtmcontrol_scan(target, DTM_DTMCONTROL_DMIRESET);
}

static void increase_interrupt_high_delay(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	info->interrupt_high_delay += info->interrupt_high_delay / 10 + 1;
	LOG_INFO("dtmcontrol_idle=%d, dmi_busy_delay=%d, interrupt_high_delay=%d",
			info->dtmcontrol_idle, info->dmi_busy_delay,
			info->interrupt_high_delay);
}

static void add_dmi_scan(const struct target *target, struct scan_field *field,
		uint8_t *out_value, uint8_t *in_value, dmi_op_t op,
		uint16_t address, uint64_t data)
{
	riscv013_info_t *info = get_info(target);

	field->num_bits = info->abits + DMI_OP_SIZE + DMI_DATA_SIZE;
	field->in_value = in_value;
	field->out_value = out_value;

	buf_set_u64(out_value, DMI_OP_START, DMI_OP_SIZE, op);
	buf_set_u64(out_value, DMI_DATA_START, DMI_DATA_SIZE, data);
	buf_set_u64(out_value, DMI_ADDRESS_START, info->abits, address);

	jtag_add_dr_scan(target->tap, 1, field, TAP_IDLE);

	int idle_count = info->dtmcontrol_idle + info->dmi_busy_delay;
	if (data & DMCONTROL_INTERRUPT) {
		idle_count += info->interrupt_high_delay;
	}

	if (idle_count) {
		jtag_add_runtest(idle_count, TAP_IDLE);
	}
}

static void dump_field(const struct scan_field *field)
{
	static const char *op_string[] = {"-", "r", "w", "?"};
	static const char *status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_DEBUG)
		return;

	uint64_t out = buf_get_u64(field->out_value, 0, field->num_bits);
	unsigned int out_op = get_field(out, DTM_DMI_OP);
	unsigned int out_data = get_field(out, DTM_DMI_DATA);
	unsigned int out_address = out >> DTM_DMI_ADDRESS_OFFSET;
	uint64_t in = buf_get_u64(field->in_value, 0, field->num_bits);
	unsigned int in_op = get_field(in, DTM_DMI_OP);
	unsigned int in_data = get_field(in, DTM_DMI_DATA);
	unsigned int in_address = in >> DTM_DMI_ADDRESS_OFFSET;

	log_printf_lf(LOG_LVL_DEBUG,
			__FILE__, __LINE__, "scan",
			"%db %s %08x @%02x -> %s %08x @%02x",
			field->num_bits,
			op_string[out_op], out_data, out_address,
			status_string[in_op], in_data, in_address);
}

static dmi_status_t dmi_scan(struct target *target, uint16_t *address_in,
		uint64_t *data_in, dmi_op_t op, uint16_t address_out, uint64_t data_out)
{
	riscv013_info_t *info = get_info(target);
	uint8_t in[8] = {0};
	uint8_t out[8];
	struct scan_field field = {
		.num_bits = info->abits + DMI_OP_SIZE + DMI_DATA_SIZE,
		.out_value = out,
		.in_value = in
	};

	assert(info->abits != 0);

	buf_set_u64(out, DMI_OP_START, DMI_OP_SIZE, op);
	buf_set_u64(out, DMI_DATA_START, DMI_DATA_SIZE, data_out);
	buf_set_u64(out, DMI_ADDRESS_START, info->abits, address_out);

	/* Assume dbus is already selected. */
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	int idle_count = info->dtmcontrol_idle + info->dmi_busy_delay;

	if (idle_count) {
		jtag_add_runtest(idle_count, TAP_IDLE);
	}

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("dmi_scan failed jtag scan");
		return retval;
	}

	if (data_in) {
		*data_in = buf_get_u64(in, DMI_DATA_START, DMI_DATA_SIZE);
	}

	if (address_in) {
		*address_in = buf_get_u32(in, DMI_ADDRESS_START, info->abits);
	}

	dump_field(&field);

	return buf_get_u32(in, DMI_OP_START, DMI_OP_SIZE);
}

static uint64_t dmi_read(struct target *target, uint16_t address)
{
	uint64_t value;
	dmi_status_t status;
	uint16_t address_in;

	unsigned i = 0;
	dmi_scan(target, &address_in, &value, DMI_OP_READ, address, 0);
	do {
		status = dmi_scan(target, &address_in, &value, DMI_OP_READ, address, 0);
		if (status == DMI_STATUS_BUSY) {
			increase_dmi_busy_delay(target);
		}
	} while (((status != DMI_STATUS_SUCCESS) || (address_in != address)) &&
			i++ < 256);

	if (status != DMI_STATUS_SUCCESS) {
		LOG_ERROR("failed read from 0x%x; value=0x%" PRIx64 ", status=%d\n",
				address, value, status);
	}

	return value;
}

static void dmi_write(struct target *target, uint16_t address, uint64_t value)
{
	dmi_status_t status = DMI_STATUS_BUSY;
	unsigned i = 0;
	while (status == DMI_STATUS_BUSY && i++ < 256) {
		dmi_scan(target, NULL, NULL, DMI_OP_WRITE, address, value);
		status = dmi_scan(target, NULL, NULL, DMI_OP_NOP, 0, 0);
		if (status == DMI_STATUS_BUSY) {
			increase_dmi_busy_delay(target);
		}
	}
	if (status != DMI_STATUS_SUCCESS) {
		LOG_ERROR("failed to write 0x%" PRIx64 " to 0x%x; status=%d\n", value, address, status);
	}
}

/*** scans "class" ***/

typedef struct {
	// Number of scans that space is reserved for.
	unsigned int scan_count;
	// Size reserved in memory for each scan, in bytes.
	unsigned int scan_size;
	unsigned int next_scan;
	uint8_t *in;
	uint8_t *out;
	struct scan_field *field;
	const struct target *target;
} scans_t;

static scans_t *scans_new(struct target *target, unsigned int scan_count)
{
	scans_t *scans = malloc(sizeof(scans_t));
	scans->scan_count = scan_count;
	// This code also gets called before xlen is detected.
	if (xlen(target))
		scans->scan_size = 2 + xlen(target) / 8;
	else
		scans->scan_size = 2 + 128 / 8;
	scans->next_scan = 0;
	scans->in = calloc(scans->scan_size, scans->scan_count);
	scans->out = calloc(scans->scan_size, scans->scan_count);
	scans->field = calloc(scans->scan_count, sizeof(struct scan_field));
	scans->target = target;
	return scans;
}

static scans_t *scans_delete(scans_t *scans)
{
	assert(scans);
	free(scans->field);
	free(scans->out);
	free(scans->in);
	free(scans);
	return NULL;
}

static void scans_dump(scans_t *scans)
{
	for (unsigned int i = 0; i < scans->next_scan; i++) {
		dump_field(&scans->field[i]);
	}
}

static int scans_execute(scans_t *scans)
{
	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("failed jtag scan: %d", retval);
		return retval;
	}

	scans_dump(scans);

	return ERROR_OK;
}

/** Add a 32-bit dbus write to the scans structure. */
static void scans_add_write32(scans_t *scans, uint16_t address, uint32_t data,
		bool set_interrupt)
{
	const unsigned int i = scans->next_scan;
	int data_offset = scans->scan_size * i;
	add_dmi_scan(scans->target, &scans->field[i], scans->out + data_offset,
			scans->in + data_offset, DMI_OP_WRITE, address,
			(set_interrupt ? DMCONTROL_INTERRUPT : 0) | DMCONTROL_HALTNOT | data);
	scans->next_scan++;
	assert(scans->next_scan <= scans->scan_count);
}

/** Add a 32-bit dbus read. */
static void scans_add_read32(scans_t *scans, uint16_t address, bool set_interrupt)
{
	assert(scans->next_scan < scans->scan_count);
	const unsigned int i = scans->next_scan;
	int data_offset = scans->scan_size * i;
	add_dmi_scan(scans->target, &scans->field[i], scans->out + data_offset,
			scans->in + data_offset, DMI_OP_READ, address,
			(set_interrupt ? DMCONTROL_INTERRUPT : 0) | DMCONTROL_HALTNOT);
	scans->next_scan++;
}

static uint32_t scans_get_u32(scans_t *scans, unsigned int index,
		unsigned first, unsigned num)
{
	return buf_get_u32(scans->in + scans->scan_size * index, first, num);
}

/*** end of scans class ***/

/** Convert register number (internal OpenOCD number) to the number expected by
 * the abstract command interface. */
static unsigned reg_number_to_no(unsigned reg_num)
{
	if (reg_num <= REG_XPR31) {
		return reg_num + 0x1000 - REG_XPR0;
	} else if (reg_num >= REG_CSR0 && reg_num <= REG_CSR4095) {
		return reg_num - REG_CSR0;
	} else if (reg_num >= REG_FPR0 && reg_num <= REG_FPR31) {
		return reg_num + 0x1020 - REG_FPR0;
	} else {
		return ~0;
	}
}

uint32_t abstract_register_size(unsigned width)
{
	switch (width) {
		case 32:
			return set_field(0, AC_ACCESS_REGISTER_SIZE, 2);
		case 64:
			return set_field(0, AC_ACCESS_REGISTER_SIZE, 3);
			break;
		case 128:
			return set_field(0, AC_ACCESS_REGISTER_SIZE, 4);
			break;
		default:
			LOG_ERROR("Unsupported register width: %d", width);
			return 0;
	}
}

static int execute_abstract_command(struct target *target, uint32_t command)
{
	dmi_write(target, DMI_COMMAND, command);

	uint32_t abstractcs;
	for (unsigned i = 0; i < 256; i++) {
		abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		if (get_field(abstractcs, DMI_ABSTRACTCS_BUSY) == 0)
			break;
	}
	if (get_field(abstractcs, DMI_ABSTRACTCS_BUSY)) {
		LOG_ERROR("Abstract command 0x%x never completed (abstractcs=0x%x)",
				command, abstractcs);
		return ERROR_FAIL;
	}
	if (get_field(abstractcs, DMI_ABSTRACTCS_CMDERR)) {
		const char *errors[8] = {
			"none",
			"busy",
			"not supported",
			"exception",
			"halt/resume",
			"reserved",
			"reserved",
			"other" };
		LOG_DEBUG("Abstract command 0x%x ended in error '%s' (abstractcs=0x%x)",
				command, errors[get_field(abstractcs, DMI_ABSTRACTCS_CMDERR)],
				abstractcs);
		// Clear the error.
		dmi_write(target, DMI_ABSTRACTCS, 0);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/*** program "class" ***/
/* This class allows a debug program to be built up piecemeal, and then be
 * executed. If necessary, the program is split up to fit in the program
 * buffer. */

typedef struct {
	uint8_t code[12 * 4];
	unsigned length;
	bool write;
	unsigned regno;
	uint64_t write_value;
} program_t;

static program_t *program_new(void)
{
	program_t *program = malloc(sizeof(program_t));
	if (program) {
		program->length = 0;
		// Default to read zero.
		program->write = false;
		program->regno = 0x1000;
	}
	return program;
}

static void program_delete(program_t *program)
{
	free(program);
}

static void program_add32(program_t *program, uint32_t instruction)
{
	assert(program->length + 4 < sizeof(program->code));
	program->code[program->length++] = instruction & 0xff;
	program->code[program->length++] = (instruction >> 8) & 0xff;
	program->code[program->length++] = (instruction >> 16) & 0xff;
	program->code[program->length++] = (instruction >> 24) & 0xff;
}

static void program_set_read(program_t *program, unsigned reg_num)
{
	program->write = false;
	program->regno = reg_number_to_no(reg_num);
}

static void program_set_write(program_t *program, unsigned reg_num, uint64_t value)
{
	program->write = true;
	program->regno = reg_number_to_no(reg_num);
	program->write_value = value;
}

/*** end of program class ***/

static uint32_t dram_read32(struct target *target, unsigned int index)
{
	uint16_t address = dram_address(index);
	uint32_t value = dmi_read(target, address);
	return value;
}

static void dram_write32(struct target *target, unsigned int index, uint32_t value,
		bool set_interrupt)
{
	uint64_t dmi_value = DMCONTROL_HALTNOT | value;
	if (set_interrupt)
		dmi_value |= DMCONTROL_INTERRUPT;
	dmi_write(target, dram_address(index), dmi_value);
}

/** Read the haltnot and interrupt bits. */
static bits_t read_bits(struct target *target)
{
	uint64_t value;
	dmi_status_t status;
	uint16_t address_in;
	riscv013_info_t *info = get_info(target);

	bits_t err_result = {
		.haltnot = 0,
		.interrupt = 0
	};

	do {
		unsigned i = 0;
		do {
			status = dmi_scan(target, &address_in, &value, DMI_OP_READ, 0, 0);
			if (status == DMI_STATUS_BUSY) {
				if (address_in == (1<<info->abits) - 1 &&
						value == (1ULL<<DMI_DATA_SIZE) - 1) {
					LOG_ERROR("TDO seems to be stuck high.");
					return err_result;
				}
				increase_dmi_busy_delay(target);
			}
		} while (status == DMI_STATUS_BUSY && i++ < 256);

		if (i >= 256) {
			LOG_ERROR("Failed to read from 0x%x; status=%d", address_in, status);
			return err_result;
		}
	} while (address_in > 0x10 && address_in != DMCONTROL);

	bits_t result = {
		.haltnot = get_field(value, DMCONTROL_HALTNOT),
		.interrupt = get_field(value, DMCONTROL_INTERRUPT)
	};
	return result;
}

static int wait_for_haltstatus(struct target *target, unsigned status)
{
	time_t start = time(NULL);
	while (1) {
		uint32_t dmcontrol = dmi_read(target, DMI_DMCONTROL);
		unsigned s = get_field(dmcontrol, DMI_DMCONTROL_HARTSTATUS);
		if (s == status)
			return ERROR_OK;
		if (time(NULL) - start > WALL_CLOCK_TIMEOUT) {
			LOG_ERROR("Timed out waiting for hart status to be %d (dmcontrol=0x%x)",
					status, dmcontrol);
			return ERROR_FAIL;
		}
	}
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
		if (time(NULL) - start > WALL_CLOCK_TIMEOUT) {
			LOG_ERROR("Timed out waiting for debug int to clear.");
			return ERROR_FAIL;
		}
	}
}

static void cache_set32(struct target *target, unsigned int index, uint32_t data)
{
	riscv013_info_t *info = get_info(target);
	if (info->dram_cache[index].valid &&
			info->dram_cache[index].data == data) {
		// This is already preset on the target.
		LOG_DEBUG("cache[0x%x] = 0x%x (hit)", index, data);
		return;
	}
	LOG_DEBUG("cache[0x%x] = 0x%x", index, data);
	info->dram_cache[index].data = data;
	info->dram_cache[index].valid = true;
	info->dram_cache[index].dirty = true;
}

static void cache_set(struct target *target, slot_t slot, uint64_t data)
{
	unsigned int offset = slot_offset(target, slot);
	cache_set32(target, offset, data);
	if (xlen(target) > 32) {
		cache_set32(target, offset + 1, data >> 32);
	}
}

static void cache_set_jump(struct target *target, unsigned int index)
{
	cache_set32(target, index,
			jal(0, (uint32_t) (DEBUG_ROM_RESUME - (DEBUG_RAM_START + 4*index))));
}

static void cache_set_load(struct target *target, unsigned int index,
		unsigned int reg, slot_t slot)
{
	uint16_t offset = DEBUG_RAM_START + 4 * slot_offset(target, slot);
	cache_set32(target, index, load(target, reg, ZERO, offset));
}

static void cache_set_store(struct target *target, unsigned int index,
		unsigned int reg, slot_t slot)
{
	uint16_t offset = DEBUG_RAM_START + 4 * slot_offset(target, slot);
	cache_set32(target, index, store(target, reg, ZERO, offset));
}

static void dump_debug_ram(struct target *target)
{
	for (unsigned int i = 0; i < DRAM_CACHE_SIZE; i++) {
		uint32_t value = dram_read32(target, i);
		LOG_ERROR("Debug RAM 0x%x: 0x%08x", i, value);
	}
}

/* Call this if the code you just ran writes to debug RAM entries 0 through 3. */
static void cache_invalidate(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	for (unsigned int i = 0; i < info->dramsize; i++) {
		info->dram_cache[i].valid = false;
		info->dram_cache[i].dirty = false;
	}
}

/* Called by cache_write() after the program has run. Also call this if you're
 * running programs without calling cache_write(). */
static void cache_clean(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	for (unsigned int i = 0; i < info->dramsize; i++) {
		if (i >= 4) {
			info->dram_cache[i].valid = false;
		}
		info->dram_cache[i].dirty = false;
	}
}

/** Write cache to the target, and optionally run the program.
 * Then read the value at address into the cache, assuming address < 128. */
#define CACHE_NO_READ	128
static int cache_write(struct target *target, unsigned int address, bool run)
{
	LOG_DEBUG("enter");
	riscv013_info_t *info = get_info(target);
	scans_t *scans = scans_new(target, info->dramsize + 2);

	unsigned int last = info->dramsize;
	for (unsigned int i = 0; i < info->dramsize; i++) {
		if (info->dram_cache[i].dirty) {
			last = i;
		}
	}

	if (last == info->dramsize) {
		// Nothing needs to be written to RAM.
		dmi_write(target, DMCONTROL, DMCONTROL_HALTNOT | DMCONTROL_INTERRUPT);

	} else {
		for (unsigned int i = 0; i < info->dramsize; i++) {
			if (info->dram_cache[i].dirty) {
				bool set_interrupt = (i == last && run);
				scans_add_write32(scans, i, info->dram_cache[i].data,
						set_interrupt);
			}
		}
	}

	if (run || address < CACHE_NO_READ) {
		// Throw away the results of the first read, since it'll contain the
		// result of the read that happened just before debugint was set.
		scans_add_read32(scans, address, false);

		// This scan contains the results of the read the caller requested, as
		// well as an interrupt bit worth looking at.
		scans_add_read32(scans, address, false);
	}

	int retval = scans_execute(scans);
	if (retval != ERROR_OK) {
		LOG_ERROR("JTAG execute failed.");
		return retval;
	}

	int errors = 0;
	for (unsigned int i = 0; i < scans->next_scan; i++) {
		dmi_status_t status = scans_get_u32(scans, i, DMI_OP_START,
				DMI_OP_SIZE);
		switch (status) {
			case DMI_STATUS_SUCCESS:
				break;
			case DMI_STATUS_FAILED:
				LOG_ERROR("Debug RAM write failed. Hardware error?");
				return ERROR_FAIL;
			case DMI_STATUS_BUSY:
				errors++;
				break;
			default:
				LOG_ERROR("Got invalid bus access status: %d", status);
				return ERROR_FAIL;
		}
	}

	if (errors) {
		increase_dmi_busy_delay(target);

		// Try again, using the slow careful code.
		// Write all RAM, just to be extra cautious.
		for (unsigned int i = 0; i < info->dramsize; i++) {
			if (i == last && run) {
				dram_write32(target, last, info->dram_cache[last].data, true);
			} else {
				dram_write32(target, i, info->dram_cache[i].data, false);
			}
			info->dram_cache[i].dirty = false;
		}
		if (run) {
			cache_clean(target);
		}

		if (wait_for_debugint_clear(target, true) != ERROR_OK) {
			LOG_ERROR("Debug interrupt didn't clear.");
			dump_debug_ram(target);
			return ERROR_FAIL;
		}

	} else {
		if (run) {
			cache_clean(target);
		} else {
			for (unsigned int i = 0; i < info->dramsize; i++) {
				info->dram_cache[i].dirty = false;
			}
		}

		if (run || address < CACHE_NO_READ) {
			int interrupt = scans_get_u32(scans, scans->next_scan-1,
					DMI_DATA_START + 33, 1);
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
				unsigned int read_addr = scans_get_u32(scans, scans->next_scan-1,
						DMI_ADDRESS_START, info->abits);
				if (read_addr != address) {
					LOG_INFO("Got data from 0x%x but expected it from 0x%x",
							read_addr, address);
				}
				info->dram_cache[read_addr].data =
					scans_get_u32(scans, scans->next_scan-1, DMI_DATA_START, 32);
				info->dram_cache[read_addr].valid = true;
			}
		}
	}

	scans_delete(scans);
	LOG_DEBUG("exit");

	return ERROR_OK;
}

static uint32_t cache_get32(struct target *target, unsigned int address)
{
	riscv013_info_t *info = get_info(target);
	if (!info->dram_cache[address].valid) {
		info->dram_cache[address].data = dram_read32(target, address);
		info->dram_cache[address].valid = true;
	}
	return info->dram_cache[address].data;
}

/* Write instruction that jumps from the specified word in Debug RAM to resume
 * in Debug ROM. */
static void dram_write_jump(struct target *target, unsigned int index,
		bool set_interrupt)
{
	dram_write32(target, index,
			jal(0, (uint32_t) (DEBUG_ROM_RESUME - (DEBUG_RAM_START + 4*index))),
			set_interrupt);
}

static int wait_for_state(struct target *target, enum target_state state)
{
	time_t start = time(NULL);
	while (1) {
		int result = riscv013_poll(target);
		if (result != ERROR_OK) {
			return result;
		}
		if (target->state == state) {
			return ERROR_OK;
		}
		if (time(NULL) - start > WALL_CLOCK_TIMEOUT) {
			LOG_ERROR("Timed out waiting for state %d.", state);
			return ERROR_FAIL;
		}
	}
}

static void write_program(struct target *target, const program_t *program)
{
	riscv013_info_t *info = get_info(target);

	assert(program->length <= info->progsize * 4);
	for (unsigned i = 0; i < program->length; i += 4) {
		uint32_t value =
			program->code[i] |
			((uint32_t) program->code[i+1] << 8) |
			((uint32_t) program->code[i+2] << 16) |
			((uint32_t) program->code[i+3] << 24);
		dmi_write(target, DMI_IBUF0 + i / 4, value);
	}
}

static int execute_program(struct target *target, const program_t *program)
{
	write_program(target, program);

	uint32_t command = 0;
	if (program->write) {
		if (get_field(command, AC_ACCESS_REGISTER_SIZE) > 2) {
			dmi_write(target, DMI_DATA1, program->write_value >> 32);
		}
		dmi_write(target, DMI_DATA0, program->write_value);
		command |= AC_ACCESS_REGISTER_WRITE | AC_ACCESS_REGISTER_POSTEXEC;
	} else {
		command |= AC_ACCESS_REGISTER_PREEXEC;
	}
	command |= abstract_register_size(xlen(target));
	command |= program->regno;

	return execute_abstract_command(target, command);
}

static int abstract_read_register(struct target *target,
		unsigned reg_number, 
		unsigned width,
		uint64_t *value)
{
	uint32_t command = abstract_register_size(width);

	command |= reg_number_to_no(reg_number);

	int result = execute_abstract_command(target, command);
	if (result != ERROR_OK) {
		return result;
	}

	if (value) {
		*value = 0;
		switch (width) {
			case 128:
				LOG_ERROR("Ignoring top 64 bits from 128-bit register read.");
			case 64:
				*value |= ((uint64_t) dmi_read(target, DMI_DATA1)) << 32;
			case 32:
				*value |= dmi_read(target, DMI_DATA0);
				break;
		}
	}

	return ERROR_OK;
}

static int abstract_write_register(struct target *target,
		unsigned reg_number, 
		unsigned width,
		uint64_t value)
{
	uint32_t command = abstract_register_size(width);

	command |= reg_number_to_no(reg_number);
	command |= AC_ACCESS_REGISTER_WRITE;

	switch (width) {
		case 128:
			LOG_ERROR("Ignoring top 64 bits from 128-bit register write.");
		case 64:
			dmi_write(target, DMI_DATA1, value >> 32);
		case 32:
			dmi_write(target, DMI_DATA0, value);
			break;
	}

	int result = execute_abstract_command(target, command);
	if (result != ERROR_OK) {
		return result;
	}

	return ERROR_OK;
}

/** csr is the CSR index between 0 and 4096. */
static int read_csr(struct target *target, uint64_t *value, uint32_t csr)
{
	int result = abstract_read_register(target, csr, xlen(target), value);
	if (result == ERROR_OK)
		return result;

	// Fall back to program buffer.
	program_t *program = program_new();
	program_add32(program, csrr(S0, csr));
	program_add32(program, ebreak());
	program_set_read(program, S0);
	execute_program(target, program);
	program_delete(program);

	result = abstract_read_register(target, S0, xlen(target), value);
	if (result != ERROR_OK)
		return result;

	LOG_DEBUG("csr 0x%x = 0x%" PRIx64, csr, *value);

	return ERROR_OK;
}

static int write_csr(struct target *target, uint32_t csr, uint64_t value)
{
	LOG_DEBUG("csr 0x%x <- 0x%" PRIx64, csr, value);
	int result = abstract_write_register(target, csr, xlen(target), value);
	if (result == ERROR_OK)
		return result;

	// Fall back to program buffer.
	program_t *program = program_new();
	program_add32(program, csrw(S0, csr));
	program_add32(program, ebreak());
	program_set_write(program, S0, value);
	result = execute_program(target, program);
	program_delete(program);

	return result;
}

static int maybe_read_tselect(struct target *target)
{
	riscv013_info_t *info = get_info(target);

	if (info->tselect_dirty) {
		int result = read_csr(target, &info->tselect, CSR_TSELECT);
		if (result != ERROR_OK)
			return result;
		info->tselect_dirty = false;
	}

	return ERROR_OK;
}

static int maybe_write_tselect(struct target *target)
{
	riscv013_info_t *info = get_info(target);

	if (!info->tselect_dirty) {
		int result = write_csr(target, CSR_TSELECT, info->tselect);
		if (result != ERROR_OK)
			return result;
		info->tselect_dirty = true;
	}

	return ERROR_OK;
}

static int execute_resume(struct target *target, bool step)
{
	riscv013_info_t *info = get_info(target);

	LOG_DEBUG("step=%d", step);

	maybe_write_tselect(target);

	// TODO: check if dpc is dirty (which also is true if an exception was hit
	// at any time)
	if (write_csr(target, CSR_DPC, info->dpc) != ERROR_OK) {
		return ERROR_FAIL;
	}

	struct reg *mstatus_reg = &target->reg_cache->reg_list[REG_MSTATUS];
	if (mstatus_reg->valid) {
		uint64_t mstatus_user = buf_get_u64(mstatus_reg->value, 0, xlen(target));
		if (mstatus_user != info->mstatus_actual) {
			if (write_csr(target, CSR_MSTATUS, mstatus_user) != ERROR_OK) {
				return ERROR_FAIL;
			}
		}
	}

	info->dcsr |= DCSR_EBREAKM | DCSR_EBREAKH | DCSR_EBREAKS | DCSR_EBREAKU;
	info->dcsr &= ~DCSR_HALT;

	if (step) {
		info->dcsr |= DCSR_STEP;
	} else {
		info->dcsr &= ~DCSR_STEP;
	}

	if (write_csr(target, CSR_DCSR, info->dcsr) != ERROR_OK) {
		return ERROR_FAIL;
	}

	program_t *program = program_new();
	program_add32(program, fence_i());
	program_add32(program, ebreak());
	if (execute_program(target, program) != ERROR_OK) {
		return ERROR_FAIL;
	}
	program_delete(program);

	dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE |
			DMI_DMCONTROL_RESUMEREQ);

	if (wait_for_haltstatus(target, 1) != ERROR_OK) {
		return ERROR_FAIL;
	}

	target->state = TARGET_RUNNING;
	register_cache_invalidate(target->reg_cache);

	return ERROR_OK;
}

// Execute a step, and wait for reentry into Debug Mode.
static int full_step(struct target *target, bool announce)
{
	int result = execute_resume(target, true);
	if (result != ERROR_OK)
		return result;
	time_t start = time(NULL);
	while (1) {
		result = poll_target(target, announce);
		if (result != ERROR_OK)
			return result;
		if (target->state != TARGET_DEBUG_RUNNING)
			break;
		if (time(NULL) - start > WALL_CLOCK_TIMEOUT) {
			LOG_ERROR("Timed out waiting for step to complete.");
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static int resume(struct target *target, int debug_execution, bool step)
{
	if (debug_execution) {
		LOG_ERROR("TODO: debug_execution is true");
		return ERROR_FAIL;
	}

	return execute_resume(target, step);
}

static void reg_cache_set(struct target *target, unsigned int number,
		uint64_t value)
{
	struct reg *r = &target->reg_cache->reg_list[number];
	LOG_DEBUG("%s <= 0x%" PRIx64, r->name, value);
	r->valid = true;
	buf_set_u64(r->value, 0, r->size, value);
}

/** Update register sizes based on xlen. */
static void update_reg_list(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	if (info->reg_values) {
		free(info->reg_values);
	}
	info->reg_values = malloc(REG_COUNT * xlen(target) / 4);

	for (unsigned int i = 0; i < REG_COUNT; i++) {
		struct reg *r = &target->reg_cache->reg_list[i];
		r->value = info->reg_values + i * xlen(target) / 4;
		if (r->dirty) {
			LOG_ERROR("Register %d was dirty. Its value is lost.", i);
		}
		if (i == REG_PRIV) {
			r->size = 8;
		} else {
			r->size = xlen(target);
		}
		r->valid = false;
	}

	reg_cache_set(target, ZERO, 0);
}

static uint64_t reg_cache_get(struct target *target, unsigned int number)
{
	struct reg *r = &target->reg_cache->reg_list[number];
	if (!r->valid) {
		LOG_ERROR("Register cache entry for %d is invalid!", number);
		assert(r->valid);
	}
	uint64_t value = buf_get_u64(r->value, 0, r->size);
	LOG_DEBUG("%s = 0x%" PRIx64, r->name, value);
	return value;
}

static int update_mstatus_actual(struct target *target)
{
	struct reg *mstatus_reg = &target->reg_cache->reg_list[REG_MSTATUS];
	if (mstatus_reg->valid) {
		// We previously made it valid.
		return ERROR_OK;
	}

	// Force reading the register. In that process mstatus_actual will be
	// updated.
	return register_get(&target->reg_cache->reg_list[REG_MSTATUS]);
}

/*** OpenOCD target functions. ***/

static int register_get(struct reg *reg)
{
	struct target *target = (struct target *) reg->arch_info;
	riscv013_info_t *info = get_info(target);

	maybe_write_tselect(target);

	int result = ERROR_OK;
	uint64_t value;
	if (reg->number <= REG_XPR31) {
		buf_set_u64(reg->value, 0, xlen(target), reg_cache_get(target, reg->number));
		LOG_DEBUG("%s=0x%" PRIx64, reg->name, reg_cache_get(target, reg->number));
		return ERROR_OK;
	} else if (reg->number == REG_PC) {
		buf_set_u32(reg->value, 0, 32, info->dpc);
		reg->valid = true;
		LOG_DEBUG("%s=0x%" PRIx64 " (cached)", reg->name, info->dpc);
		return ERROR_OK;
	} else if (reg->number >= REG_FPR0 && reg->number <= REG_FPR31) {
		result = update_mstatus_actual(target);
		if (result != ERROR_OK) {
			return result;
		}
		unsigned i = 0;
		if ((info->mstatus_actual & MSTATUS_FS) == 0) {
			info->mstatus_actual = set_field(info->mstatus_actual, MSTATUS_FS, 1);
			cache_set_load(target, i++, S0, SLOT1);
			cache_set32(target, i++, csrw(S0, CSR_MSTATUS));
			cache_set(target, SLOT1, info->mstatus_actual);
		}

		if (xlen(target) == 32) {
			cache_set32(target, i++, fsw(reg->number - REG_FPR0, 0, DEBUG_RAM_START + 16));
		} else {
			cache_set32(target, i++, fsd(reg->number - REG_FPR0, 0, DEBUG_RAM_START + 16));
		}
		cache_set_jump(target, i++);
	} else if (reg->number >= REG_CSR0 && reg->number <= REG_CSR4095) {
		result = read_csr(target, &value, reg->number - REG_CSR0);
	} else if (reg->number == REG_PRIV) {
		buf_set_u64(reg->value, 0, 8, get_field(info->dcsr, DCSR_PRV));
		LOG_DEBUG("%s=%d (cached)", reg->name,
				(int) get_field(info->dcsr, DCSR_PRV));
		return ERROR_OK;
	} else {
		LOG_ERROR("Don't know how to read register %d (%s)", reg->number, reg->name);
		return ERROR_FAIL;
	}

	if (result != ERROR_OK)
		return result;

	LOG_DEBUG("%s=0x%" PRIx64, reg->name, value);
	buf_set_u64(reg->value, 0, xlen(target), value);

	if (reg->number == REG_MSTATUS) {
		info->mstatus_actual = value;
		reg->valid = true;
	}

	return ERROR_OK;
}

static int register_write(struct target *target, unsigned int number,
		uint64_t value)
{
	riscv013_info_t *info = get_info(target);

	maybe_write_tselect(target);

	if (number == S0) {
		cache_set_load(target, 0, S0, SLOT0);
		cache_set32(target, 1, csrw(S0, CSR_DSCRATCH));
		cache_set_jump(target, 2);
	} else if (number == S1) {
		cache_set_load(target, 0, S0, SLOT0);
		cache_set_store(target, 1, S0, SLOT_LAST);
		cache_set_jump(target, 2);
	} else if (number <= REG_XPR31) {
		cache_set_load(target, 0, number - REG_XPR0, SLOT0);
		cache_set_jump(target, 1);
	} else if (number == REG_PC) {
		info->dpc = value;
		return ERROR_OK;
	} else if (number >= REG_FPR0 && number <= REG_FPR31) {
		int result = update_mstatus_actual(target);
		if (result != ERROR_OK) {
			return result;
		}
		unsigned i = 0;
		if ((info->mstatus_actual & MSTATUS_FS) == 0) {
			info->mstatus_actual = set_field(info->mstatus_actual, MSTATUS_FS, 1);
			cache_set_load(target, i++, S0, SLOT1);
			cache_set32(target, i++, csrw(S0, CSR_MSTATUS));
			cache_set(target, SLOT1, info->mstatus_actual);
		}

		if (xlen(target) == 32) {
			cache_set32(target, i++, flw(number - REG_FPR0, 0, DEBUG_RAM_START + 16));
		} else {
			cache_set32(target, i++, fld(number - REG_FPR0, 0, DEBUG_RAM_START + 16));
		}
		cache_set_jump(target, i++);
	} else if (number >= REG_CSR0 && number <= REG_CSR4095) {
		cache_set_load(target, 0, S0, SLOT0);
		cache_set32(target, 1, csrw(S0, number - REG_CSR0));
		cache_set_jump(target, 2);

		if (number == REG_MSTATUS) {
			info->mstatus_actual = value;
		}
	} else if (number == REG_PRIV) {
		info->dcsr = set_field(info->dcsr, DCSR_PRV, value);
		return ERROR_OK;
	} else {
		LOG_ERROR("Don't know how to write register %d", number);
		return ERROR_FAIL;
	}

	cache_set(target, SLOT0, value);
	if (cache_write(target, info->dramsize - 1, true) != ERROR_OK) {
		return ERROR_FAIL;
	}

	uint32_t exception = cache_get32(target, info->dramsize-1);
	if (exception) {
		LOG_ERROR("Got exception 0x%x when writing register %d", exception,
				number);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int register_set(struct reg *reg, uint8_t *buf)
{
	struct target *target = (struct target *) reg->arch_info;

	uint64_t value = buf_get_u64(buf, 0, xlen(target));

	LOG_DEBUG("write 0x%" PRIx64 " to %s", value, reg->name);
	struct reg *r = &target->reg_cache->reg_list[reg->number];
	r->valid = true;
	memcpy(r->value, buf, (r->size + 7) / 8);

	return register_write(target, reg->number, value);
}

static struct reg_arch_type riscv_reg_arch_type = {
	.get = register_get,
	.set = register_set
};

static int halt(struct target *target)
{
	LOG_DEBUG("riscv_halt()");
	select_dmi(target);

	dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_HALTREQ | DMI_DMCONTROL_DMACTIVE);

	return ERROR_OK;
}

static int init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_DEBUG("init");
	riscv_info_t *generic_info = (riscv_info_t *) target->arch_info;
	generic_info->version_specific = calloc(1, sizeof(riscv013_info_t));
	if (!generic_info->version_specific)
		return ERROR_FAIL;
	riscv013_info_t *info = get_info(target);

	target->reg_cache = calloc(1, sizeof(*target->reg_cache));
	target->reg_cache->name = "RISC-V registers";
	target->reg_cache->num_regs = REG_COUNT;

	target->reg_cache->reg_list = calloc(REG_COUNT, sizeof(struct reg));

	const unsigned int max_reg_name_len = 12;
	info->reg_names = calloc(1, REG_COUNT * max_reg_name_len);
	char *reg_name = info->reg_names;
	info->reg_values = NULL;

	for (unsigned int i = 0; i < REG_COUNT; i++) {
		struct reg *r = &target->reg_cache->reg_list[i];
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
		} else if (i == REG_PRIV) {
			sprintf(reg_name, "priv");
		}
		if (reg_name[0]) {
			r->name = reg_name;
		}
		reg_name += strlen(reg_name) + 1;
		assert(reg_name < info->reg_names + REG_COUNT * max_reg_name_len);
	}
	update_reg_list(target);

	memset(info->trigger_unique_id, 0xff, sizeof(info->trigger_unique_id));

	return ERROR_OK;
}

static void deinit_target(struct target *target)
{
	LOG_DEBUG("riscv_deinit_target()");
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	free(info->version_specific);
	info->version_specific = NULL;
}

static int add_trigger(struct target *target, struct trigger *trigger)
{
	riscv013_info_t *info = get_info(target);

	maybe_read_tselect(target);

	unsigned int i;
	for (i = 0; i < info->trigger_count; i++) {
		if (info->trigger_unique_id[i] != -1) {
			continue;
		}

		write_csr(target, CSR_TSELECT, i);

		uint64_t tdata1;
		read_csr(target, &tdata1, CSR_TDATA1);
		int type = get_field(tdata1, MCONTROL_TYPE(xlen(target)));

		if (type != 2) {
			continue;
		}

		if (tdata1 & (MCONTROL_EXECUTE | MCONTROL_STORE | MCONTROL_LOAD)) {
			// Trigger is already in use, presumably by user code.
			continue;
		}

		// address/data match trigger
		tdata1 |= MCONTROL_DMODE(xlen(target));
		tdata1 = set_field(tdata1, MCONTROL_ACTION,
				MCONTROL_ACTION_DEBUG_MODE);
		tdata1 = set_field(tdata1, MCONTROL_MATCH, MCONTROL_MATCH_EQUAL);
		tdata1 |= MCONTROL_M;
		if (info->misa & (1 << ('H' - 'A')))
			tdata1 |= MCONTROL_H;
		if (info->misa & (1 << ('S' - 'A')))
			tdata1 |= MCONTROL_S;
		if (info->misa & (1 << ('U' - 'A')))
			tdata1 |= MCONTROL_U;

		if (trigger->execute)
			tdata1 |= MCONTROL_EXECUTE;
		if (trigger->read)
			tdata1 |= MCONTROL_LOAD;
		if (trigger->write)
			tdata1 |= MCONTROL_STORE;

		write_csr(target, CSR_TDATA1, tdata1);

		uint64_t tdata1_rb;
		read_csr(target, &tdata1_rb, CSR_TDATA1);
		LOG_DEBUG("tdata1=0x%" PRIx64, tdata1_rb);

		if (tdata1 != tdata1_rb) {
			LOG_DEBUG("Trigger %d doesn't support what we need; After writing 0x%"
					PRIx64 " to tdata1 it contains 0x%" PRIx64,
					i, tdata1, tdata1_rb);
			write_csr(target, CSR_TDATA1, 0);
			continue;
		}

		write_csr(target, CSR_TDATA2, trigger->address);

		LOG_DEBUG("Using resource %d for bp %d", i,
				trigger->unique_id);
		info->trigger_unique_id[i] = trigger->unique_id;
		break;
	}
	if (i >= info->trigger_count) {
		LOG_ERROR("Couldn't find an available hardware trigger.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	return ERROR_OK;
}

static int remove_trigger(struct target *target, struct trigger *trigger)
{
	riscv013_info_t *info = get_info(target);

	maybe_read_tselect(target);

	unsigned int i;
	for (i = 0; i < info->trigger_count; i++) {
		if (info->trigger_unique_id[i] == trigger->unique_id) {
			break;
		}
	}
	if (i >= info->trigger_count) {
		LOG_ERROR("Couldn't find the hardware resources used by hardware "
				"trigger.");
		return ERROR_FAIL;
	}
	LOG_DEBUG("Stop using resource %d for bp %d", i, trigger->unique_id);
	write_csr(target, CSR_TSELECT, i);
	write_csr(target, CSR_TDATA1, 0);
	info->trigger_unique_id[i] = -1;

	return ERROR_OK;
}

static void trigger_from_breakpoint(struct trigger *trigger,
		const struct breakpoint *breakpoint)
{
	trigger->address = breakpoint->address;
	trigger->length = breakpoint->length;
	trigger->mask = ~0LL;
	trigger->read = false;
	trigger->write = false;
	trigger->execute = true;
	// unique_id is unique across both breakpoints and watchpoints.
	trigger->unique_id = breakpoint->unique_id;
}

static void trigger_from_watchpoint(struct trigger *trigger,
		const struct watchpoint *watchpoint)
{
	trigger->address = watchpoint->address;
	trigger->length = watchpoint->length;
	trigger->mask = watchpoint->mask;
	trigger->value = watchpoint->value;
	trigger->read = (watchpoint->rw == WPT_READ || watchpoint->rw == WPT_ACCESS);
	trigger->write = (watchpoint->rw == WPT_WRITE || watchpoint->rw == WPT_ACCESS);
	trigger->execute = false;
	// unique_id is unique across both breakpoints and watchpoints.
	trigger->unique_id = watchpoint->unique_id;
}

static int add_breakpoint(struct target *target,
	struct breakpoint *breakpoint)
{
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
		struct trigger trigger;
		trigger_from_breakpoint(&trigger, breakpoint);
		int result = add_trigger(target, &trigger);
		if (result != ERROR_OK) {
			return result;
		}

	} else {
		LOG_INFO("OpenOCD only supports hardware and software breakpoints.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->set = true;

	return ERROR_OK;
}

static int remove_breakpoint(struct target *target,
		struct breakpoint *breakpoint)
{
	if (breakpoint->type == BKPT_SOFT) {
		if (target_write_memory(target, breakpoint->address, breakpoint->length, 1,
					breakpoint->orig_instr) != ERROR_OK) {
			LOG_ERROR("Failed to restore instruction for %d-byte breakpoint at "
					"0x%x", breakpoint->length, breakpoint->address);
			return ERROR_FAIL;
		}

	} else if (breakpoint->type == BKPT_HARD) {
		struct trigger trigger;
		trigger_from_breakpoint(&trigger, breakpoint);
		int result = remove_trigger(target, &trigger);
		if (result != ERROR_OK) {
			return result;
		}

	} else {
		LOG_INFO("OpenOCD only supports hardware and software breakpoints.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	breakpoint->set = false;

	return ERROR_OK;
}

static int add_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	int result = add_trigger(target, &trigger);
	if (result != ERROR_OK) {
		return result;
	}
	watchpoint->set = true;

	return ERROR_OK;
}

static int remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint)
{
	struct trigger trigger;
	trigger_from_watchpoint(&trigger, watchpoint);

	int result = remove_trigger(target, &trigger);
	if (result != ERROR_OK) {
		return result;
	}
	watchpoint->set = false;

	return ERROR_OK;
}

static int strict_step(struct target *target, bool announce)
{
	riscv013_info_t *info = get_info(target);

	LOG_DEBUG("enter");

	struct breakpoint *breakpoint = target->breakpoints;
	while (breakpoint) {
		remove_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}

	struct watchpoint *watchpoint = target->watchpoints;
	while (watchpoint) {
		remove_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}

	int result = full_step(target, announce);
	if (result != ERROR_OK)
		return result;

	breakpoint = target->breakpoints;
	while (breakpoint) {
		add_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}

	watchpoint = target->watchpoints;
	while (watchpoint) {
		add_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}

	info->need_strict_step = false;

	return ERROR_OK;
}

static int step(struct target *target, int current, uint32_t address,
		int handle_breakpoints)
{
	riscv013_info_t *info = get_info(target);

	select_dmi(target);

	if (!current) {
		if (xlen(target) > 32) {
			LOG_WARNING("Asked to resume at 32-bit PC on %d-bit target.",
					xlen(target));
		}
		int result = register_write(target, REG_PC, address);
		if (result != ERROR_OK)
			return result;
	}

	if (info->need_strict_step || handle_breakpoints) {
		int result = strict_step(target, true);
		if (result != ERROR_OK)
			return result;
	} else {
		return resume(target, 0, true);
	}

	return ERROR_OK;
}

static int examine(struct target *target)
{
	// Don't need to select dbus, since the first thing we do is read dtmcontrol.

	uint32_t dtmcontrol = dtmcontrol_scan(target, 0);
	LOG_DEBUG("dtmcontrol=0x%x", dtmcontrol);
	LOG_DEBUG("  dmireset=%d", get_field(dtmcontrol, DTM_DTMCONTROL_DMIRESET));
	LOG_DEBUG("  idle=%d", get_field(dtmcontrol, DTM_DTMCONTROL_IDLE));
	LOG_DEBUG("  dmistat=%d", get_field(dtmcontrol, DTM_DTMCONTROL_DMISTAT));
	LOG_DEBUG("  abits=%d", get_field(dtmcontrol, DTM_DTMCONTROL_ABITS));
	LOG_DEBUG("  version=%d", get_field(dtmcontrol, DTM_DTMCONTROL_VERSION));
	if (dtmcontrol == 0) {
		LOG_ERROR("dtmcontrol is 0. Check JTAG connectivity/board power.");
		return ERROR_FAIL;
	}
	if (get_field(dtmcontrol, DTM_DTMCONTROL_VERSION) != 1) {
		LOG_ERROR("Unsupported DTM version %d. (dtmcontrol=0x%x)",
				get_field(dtmcontrol, DTM_DTMCONTROL_VERSION), dtmcontrol);
		return ERROR_FAIL;
	}

	riscv013_info_t *info = get_info(target);
	info->abits = get_field(dtmcontrol, DTM_DTMCONTROL_ABITS);
	info->dtmcontrol_idle = get_field(dtmcontrol, DTM_DTMCONTROL_IDLE);
	if (info->dtmcontrol_idle == 0) {
		// Some old SiFive cores don't set idle but need it to be 1.
		uint32_t idcode = idcode_scan(target);
		if (idcode == 0x10e31913)
			info->dtmcontrol_idle = 1;
	}

	uint32_t dmcontrol = dmi_read(target, DMI_DMCONTROL);
	if (get_field(dmcontrol, DMI_DMCONTROL_VERSION) != 1) {
		LOG_ERROR("OpenOCD only supports Debug Module version 1, not %d "
				"(dmcontrol=0x%x)", get_field(dmcontrol, DMI_DMCONTROL_VERSION), dmcontrol);
		return ERROR_FAIL;
	}

	// Reset the Debug Module.
	dmi_write(target, DMI_DMCONTROL, 0);
	dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE);
	dmcontrol = dmi_read(target, DMI_DMCONTROL);

	LOG_DEBUG("dmcontrol: 0x%08x", dmcontrol);
	LOG_DEBUG("  haltreq=%d", get_field(dmcontrol, DMI_DMCONTROL_HALTREQ));
	LOG_DEBUG("  reset=%d", get_field(dmcontrol, DMI_DMCONTROL_RESET));
	LOG_DEBUG("  dmactive=%d", get_field(dmcontrol, DMI_DMCONTROL_DMACTIVE));
	LOG_DEBUG("  hartstatus=%d", get_field(dmcontrol, DMI_DMCONTROL_HARTSTATUS));
	LOG_DEBUG("  hartsel=0x%x", get_field(dmcontrol, DMI_DMCONTROL_HARTSEL));
	LOG_DEBUG("  authenticated=%d", get_field(dmcontrol, DMI_DMCONTROL_AUTHENTICATED));
	LOG_DEBUG("  authbusy=%d", get_field(dmcontrol, DMI_DMCONTROL_AUTHBUSY));
	LOG_DEBUG("  authtype=%d", get_field(dmcontrol, DMI_DMCONTROL_AUTHTYPE));
	LOG_DEBUG("  version=%d", get_field(dmcontrol, DMI_DMCONTROL_VERSION));

	unsigned hartstatus = DMI_DMCONTROL_HARTSTATUS;

	if (!get_field(dmcontrol, DMI_DMCONTROL_DMACTIVE)) {
		LOG_ERROR("Debug Module did not become active. dmcontrol=0x%x",
				dmcontrol);
		return ERROR_FAIL;
	}

	if (!get_field(dmcontrol, DMI_DMCONTROL_AUTHENTICATED)) {
		LOG_ERROR("Authentication required by RISC-V core but not "
				"supported by OpenOCD. dmcontrol=0x%x", dmcontrol);
		return ERROR_FAIL;
	}

	if (hartstatus == 2) {
		LOG_ERROR("The hart is unavailable.");
		return ERROR_FAIL;
	}

	if (hartstatus == 3) {
		LOG_ERROR("The hart doesn't exist.");
		return ERROR_FAIL;
	}

	// Check that abstract data registers are accessible.
	uint32_t abstractcs = dmi_read(target, DMI_ABSTRACTCS);
	info->datacount = get_field(abstractcs, DMI_ABSTRACTCS_DATACOUNT);
	LOG_DEBUG("abstractcs=0x%x", abstractcs);
	LOG_DEBUG("  datacount=%d", info->datacount);

	uint32_t accesscs = dmi_read(target, DMI_ACCESSCS);
	info->progsize = get_field(abstractcs, DMI_ACCESSCS_PROGSIZE);
	LOG_DEBUG("accesscs=0x%x", accesscs);
	LOG_DEBUG("  progsize=%d", info->progsize);

	uint32_t value = 0x53467665;
	for (unsigned i = 0; i < info->datacount; i++) {
		dmi_write(target, DMI_DATA0 + i, value);
		value += 0x52534335;
	}

	for (unsigned i = 0; i < info->progsize; i++) {
		dmi_write(target, DMI_IBUF0 + i, value);
		value += 0x52534335;
	}

	value = 0x53467665;
	for (unsigned i = 0; i < info->datacount; i++) {
		uint32_t check = dmi_read(target, DMI_DATA0 + i);
		if (check != value) {
			LOG_ERROR("Wrote 0x%x to dbus address 0x%x but got back 0x%x",
					value, DMI_DATA0 + i, check);
			return ERROR_FAIL;
		}
		value += 0x52534335;
	}
	for (unsigned i = 0; i < info->progsize; i++) {
		uint32_t check = dmi_read(target, DMI_IBUF0 + i);
		if (check != value) {
			LOG_ERROR("Wrote 0x%x to dbus address 0x%x but got back 0x%x",
					value, DMI_IBUF0 + i, check);
			return ERROR_FAIL;
		}
		value += 0x52534335;
	}

	if (hartstatus == 1) {
		dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_HALTREQ | DMI_DMCONTROL_DMACTIVE);
		for (unsigned i = 0; i < 256; i++) {
			dmcontrol = dmi_read(target, DMI_DMCONTROL);
			if (get_field(dmcontrol, DMI_DMCONTROL_HARTSTATUS) == 0)
				break;
		}
		if (get_field(dmcontrol, DMI_DMCONTROL_HARTSTATUS) != 0) {
			LOG_ERROR("hart didn't halt; dmcontrol=0x%x", dmcontrol);
			return ERROR_FAIL;
		}
	}

	// TODO: do this using Quick Access, if supported.

	riscv_info_t *generic_info = (riscv_info_t *) target->arch_info;
	if (abstract_read_register(target, 15, 128, NULL) == ERROR_OK) {
		generic_info->xlen = 128;
	} else if (abstract_read_register(target, 15, 64, NULL) == ERROR_OK) {
		generic_info->xlen = 64;
	} else if (abstract_read_register(target, 15, 32, NULL) == ERROR_OK) {
		generic_info->xlen = 32;
	} else {
		LOG_ERROR("Failed to discover size using abstract register reads.");
		return ERROR_FAIL;
	}

	LOG_DEBUG("Discovered XLEN is %d", xlen(target));

	// Update register list to match discovered XLEN.
	update_reg_list(target);

	if (read_csr(target, &info->misa, CSR_MISA) != ERROR_OK) {
		LOG_ERROR("Failed to read misa.");
		return ERROR_FAIL;
	}

	if (hartstatus == 1) {
		// Resume if the hart had been running.
		dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE |
				DMI_DMCONTROL_RESUMEREQ);
		for (unsigned i = 0; i < 256; i++) {
			dmcontrol = dmi_read(target, DMI_DMCONTROL);
			if (get_field(dmcontrol, DMI_DMCONTROL_HARTSTATUS) == 1)
				break;
		}
		if (get_field(dmcontrol, DMI_DMCONTROL_HARTSTATUS) != 1) {
			LOG_ERROR("hart didn't resume; dmcontrol=0x%x", dmcontrol);
			return ERROR_FAIL;
		}
	}

	info->never_halted = true;

	int result = riscv013_poll(target);
	if (result != ERROR_OK) {
		return result;
	}

	target_set_examined(target);
	LOG_INFO("Examined RISCV core; XLEN=%d, misa=0x%" PRIx64, xlen(target), info->misa);

	return ERROR_OK;
}

static riscv_error_t handle_halt_routine(struct target *target)
{
	riscv013_info_t *info = get_info(target);

	// Read all GPRs as fast as we can, because gdb is going to ask for them
	// anyway. Reading them one at a time is much slower.

	for (int reg = 1; reg < 32; reg++) {
		uint64_t value;
		int result = abstract_read_register(target, reg, xlen(target), &value);
		if (result != ERROR_OK)
			return result;
		reg_cache_set(target, reg, value);
	}

	unsigned int csr[] = {CSR_DPC, CSR_DCSR};
	for (unsigned int i = 0; i < DIM(csr); i++) {
		uint64_t value;
		int reg = csr[i];
		int result = read_csr(target, &value, reg);
		if (result != ERROR_OK)
			return result;
		reg_cache_set(target, reg, value);
	}

	// TODO: get rid of those 2 variables and talk to the cache directly.
	info->dpc = reg_cache_get(target, CSR_DPC);
	info->dcsr = reg_cache_get(target, CSR_DCSR);

	return RE_OK;
}

static int handle_halt(struct target *target, bool announce)
{
	riscv013_info_t *info = get_info(target);
	target->state = TARGET_HALTED;

	riscv_error_t re;
	do {
		re = handle_halt_routine(target);
	} while (re == RE_AGAIN);
	if (re != RE_OK) {
		LOG_ERROR("handle_halt_routine failed");
		return ERROR_FAIL;
	}

	int cause = get_field(info->dcsr, CSR_DCSR_CAUSE);
	switch (cause) {
		case DCSR_CAUSE_SWBP:
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case DCSR_CAUSE_HWBP:
			target->debug_reason = DBG_REASON_WPTANDBKPT;
			// If we halted because of a data trigger, gdb doesn't know to do
			// the disable-breakpoints-step-enable-breakpoints dance.
			info->need_strict_step = true;
			break;
		case DCSR_CAUSE_DEBUGINT:
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case DCSR_CAUSE_STEP:
			target->debug_reason = DBG_REASON_SINGLESTEP;
			break;
		case DCSR_CAUSE_HALT:
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		default:
			LOG_ERROR("Invalid halt cause %d in DCSR (0x%" PRIx64 ")",
					cause, info->dcsr);
	}

	if (info->never_halted) {
		info->never_halted = false;

		// Disable any hardware triggers that have dmode set. We can't have set
		// them ourselves. Maybe they're left over from some killed debug
		// session.
		// Count the number of triggers while we're at it.

		int result = maybe_read_tselect(target);
		if (result != ERROR_OK)
			return result;
		for (info->trigger_count = 0; info->trigger_count < MAX_HWBPS; info->trigger_count++) {
			write_csr(target, CSR_TSELECT, info->trigger_count);
			uint64_t tselect_rb;
			read_csr(target, &tselect_rb, CSR_TSELECT);
			if (info->trigger_count != tselect_rb)
				break;
			uint64_t tdata1;
			read_csr(target, &tdata1, CSR_TDATA1);
			if ((tdata1 & MCONTROL_DMODE(xlen(target))) &&
					(tdata1 & (MCONTROL_EXECUTE | MCONTROL_STORE | MCONTROL_LOAD))) {
				write_csr(target, CSR_TDATA1, 0);
			}
		}
	}

	if (announce) {
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	}

	const char *cause_string[] = {
		"none",
		"software breakpoint",
		"hardware trigger",
		"debug interrupt",
		"step",
		"halt"
	};
	// This is logged to the user so that gdb will show it when a user types
	// 'monitor reset init'. At that time gdb appears to have the pc cached
	// still so if a user manually inspects the pc it will still have the old
	// value.
	LOG_USER("halted at 0x%" PRIx64 " due to %s", info->dpc, cause_string[cause]);

	return ERROR_OK;
}

static int poll_target(struct target *target, bool announce)
{
	select_dmi(target);

	// Inhibit debug logging during poll(), which isn't usually interesting and
	// just fills up the screen/logs with clutter.
	int old_debug_level = debug_level;
	if (debug_level >= LOG_LVL_DEBUG) {
		debug_level = LOG_LVL_INFO;
	}
	uint32_t dmcontrol = dmi_read(target, DMI_DMCONTROL);
	debug_level = old_debug_level;

	switch (get_field(dmcontrol, DMI_DMCONTROL_HARTSTATUS)) {
		case 0:
			if (target->state != TARGET_HALTED) {
				return handle_halt(target, announce);
			}
			break;
		case 1:
			target->state = TARGET_RUNNING;
			break;
		case 2:
			// Could be unavailable for other reasons.
			target->state = TARGET_RESET;
			break;
		case 3:
			LOG_ERROR("Hart disappeared!");
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int riscv013_poll(struct target *target)
{
	return poll_target(target, true);
}

static int riscv013_resume(struct target *target, int current, uint32_t address,
		int handle_breakpoints, int debug_execution)
{
	riscv013_info_t *info = get_info(target);

	select_dmi(target);

	if (!current) {
		if (xlen(target) > 32) {
			LOG_WARNING("Asked to resume at 32-bit PC on %d-bit target.",
					xlen(target));
		}
		int result = register_write(target, REG_PC, address);
		if (result != ERROR_OK)
			return result;
	}

	if (info->need_strict_step || handle_breakpoints) {
		int result = strict_step(target, false);
		if (result != ERROR_OK)
			return result;
	}

	return resume(target, debug_execution, false);
}

static int assert_reset(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	// TODO: Maybe what I implemented here is more like soft_reset_halt()?

	select_dmi(target);

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

static int deassert_reset(struct target *target)
{
	select_dmi(target);
	if (target->reset_halt) {
		return wait_for_state(target, TARGET_HALTED);
	} else {
		return wait_for_state(target, TARGET_RUNNING);
	}
}

static int read_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	select_dmi(target);

	abstract_write_register(target, S0, xlen(target), address);

	program_t *program = program_new();
	switch (size) {
		case 1:
			program_add32(program, lb(S1, S0, 0));
			break;
		case 2:
			program_add32(program, lh(S1, S0, 0));
			break;
		case 4:
			program_add32(program, lw(S1, S0, 0));
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}
	program_add32(program, addi(S0, S0, size));
	program_add32(program, ebreak());
	write_program(target, program);
	program_delete(program);

	execute_abstract_command(target,
			AC_ACCESS_REGISTER_PREEXEC |
			abstract_register_size(xlen(target)) | reg_number_to_no(S1));
	dmi_write(target, DMI_ABSTRACTCS, DMI_ABSTRACTCS_AUTOEXEC0);

	for (uint32_t i = 0; i < count; i++) {
		uint32_t value = dmi_read(target, DMI_DATA0);
		switch (size) {
			case 1:
				buffer[i] = value;
				break;
			case 2:
				buffer[2*i] = value;
				buffer[2*i+1] = value >> 8;
				break;
			case 4:
				buffer[4*i] = value;
				buffer[4*i+1] = value >> 8;
				buffer[4*i+2] = value >> 16;
				buffer[4*i+3] = value >> 24;
				break;
			default:
				return ERROR_FAIL;
		}
	}
	dmi_write(target, DMI_ABSTRACTCS, 0);
	// TODO: Check for errors.

	return ERROR_OK;
}

static int write_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	select_dmi(target);

	abstract_write_register(target, S0, xlen(target), address);

	program_t *program = program_new();
	switch (size) {
		case 1:
			program_add32(program, sb(S1, S0, 0));
			break;
		case 2:
			program_add32(program, sh(S1, S0, 0));
			break;
		case 4:
			program_add32(program, sw(S1, S0, 0));
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}
	program_add32(program, addi(S0, S0, size));
	program_add32(program, ebreak());
	write_program(target, program);
	program_delete(program);

	for (uint32_t i = 0; i < count; i++) {
		uint32_t value;
		switch (size) {
			case 1:
				value = buffer[i];
				break;
			case 2:
				value = buffer[2*i] | ((uint32_t) buffer[2*i+1] << 8);
				break;
			case 4:
				value = buffer[4*i] |
					((uint32_t) buffer[4*i+1] << 8) |
					((uint32_t) buffer[4*i+2] << 16) |
					((uint32_t) buffer[4*i+3] << 24);
				break;
			default:
				return ERROR_FAIL;
		}
		dmi_write(target, DMI_DATA0, value);

		if (i == 0) {
			execute_abstract_command(target,
					AC_ACCESS_REGISTER_WRITE | AC_ACCESS_REGISTER_POSTEXEC |
					abstract_register_size(xlen(target)) | reg_number_to_no(S1));
			dmi_write(target, DMI_ABSTRACTCS, DMI_ABSTRACTCS_AUTOEXEC0);
		}
	}
	dmi_write(target, DMI_ABSTRACTCS, 0);
	// TODO: Check for errors.

	return ERROR_OK;
}

static int arch_state(struct target *target)
{
	return ERROR_OK;
}

struct target_type riscv013_target =
{
	.name = "riscv",

	.init_target = init_target,
	.deinit_target = deinit_target,
	.examine = examine,

	/* poll current target status */
	.poll = riscv013_poll,

	.halt = halt,
	.resume = riscv013_resume,
	.step = step,

	.assert_reset = assert_reset,
	.deassert_reset = deassert_reset,

	.read_memory = read_memory,
	.write_memory = write_memory,

	.add_breakpoint = add_breakpoint,
	.remove_breakpoint = remove_breakpoint,

	.add_watchpoint = add_watchpoint,
	.remove_watchpoint = remove_watchpoint,

	.arch_state = arch_state,
};
