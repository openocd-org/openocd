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

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

#define DIM(x)		(sizeof(x)/sizeof(*x))

#define CSR_DCSR_CAUSE_SWBP		1
#define CSR_DCSR_CAUSE_TRIGGER	2
#define CSR_DCSR_CAUSE_DEBUGINT	3
#define CSR_DCSR_CAUSE_STEP		4
#define CSR_DCSR_CAUSE_HALT		5

/*** JTAG registers. ***/

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

#define CMDERR_NONE				0
#define CMDERR_BUSY				1
#define CMDERR_NOT_SUPPORTED	2
#define CMDERR_EXCEPTION		3
#define CMDERR_HALT_RESUME		4
#define CMDERR_OTHER			7

/*** Info about the core being debugged. ***/

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
	REG_TSELECT = CSR_TSELECT + REG_CSR0,
	REG_TDATA1 = CSR_TDATA1 + REG_CSR0,
	REG_TDATA2 = CSR_TDATA2 + REG_CSR0,
	REG_MISA = CSR_MISA + REG_CSR0,
	REG_DPC = CSR_DPC + REG_CSR0,
	REG_DCSR = CSR_DCSR + REG_CSR0,
	REG_MSTATUS = CSR_MSTATUS + REG_CSR0,
	REG_CSR4095 = 4160,
	REG_PRIV = 4161,
	REG_COUNT
};

#define MAX_HWBPS			16

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
	uint64_t dcsr;
	uint64_t dpc;
	uint64_t misa;
	uint64_t tselect;
	bool tselect_dirty;
	/* The value that mstatus actually has on the target right now. This is not
	 * the value we present to the user. That one may be stored in the
	 * reg_cache. */
	uint64_t mstatus_actual;

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

	// This value is increased every time we tried to execute two commands
	// consecutively, and the second one failed because the previous hadn't
	// completed yet.  It's used to add extra run-test/idle cycles after
	// starting a command, so we don't have to waste time checking for busy to
	// go low.
	unsigned int ac_busy_delay;

	bool need_strict_step;
	bool never_halted;
} riscv013_info_t;

typedef struct {
	bool haltnot;
	bool interrupt;
} bits_t;

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

	if (field->in_value) {
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
	} else {
		log_printf_lf(LOG_LVL_DEBUG,
				__FILE__, __LINE__, "scan", "%db %s %08x @%02x -> ?",
				field->num_bits, op_string[out_op], out_data, out_address);
	}
}

static riscv013_info_t *get_info(const struct target *target)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	return (riscv013_info_t *) info->version_specific;
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

static void scans_add_dmi_write(scans_t *scans, unsigned address,
		uint32_t value, bool exec)
{
	riscv013_info_t *info = get_info(scans->target);
	assert(scans->next_scan < scans->scan_count);
	const unsigned int i = scans->next_scan;
	int data_offset = scans->scan_size * i;
	struct scan_field *field = scans->field + i;

	uint8_t *out = scans->out + data_offset;
	field->num_bits = info->abits + DTM_DMI_OP_LENGTH + DTM_DMI_DATA_LENGTH;
	// We gain a lot of speed in remote bitbang by not looking at the return
	// value.
	field->in_value = NULL;
	field->out_value = out;

	buf_set_u64(out, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_WRITE);
	buf_set_u64(out, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, value);
	buf_set_u64(out, DTM_DMI_ADDRESS_OFFSET, info->abits, address);

	/* Assume dbus is already selected. */
	jtag_add_dr_scan(scans->target->tap, 1, field, TAP_IDLE);

	int idle_count = info->dtmcontrol_idle + info->dmi_busy_delay;
	if (exec)
		idle_count += info->ac_busy_delay;

	if (idle_count) {
		jtag_add_runtest(idle_count, TAP_IDLE);
	}

	scans->next_scan++;
}

/*** end of scans class ***/
/*** Necessary prototypes. ***/

static int poll_target(struct target *target, bool announce);
static int riscv013_poll(struct target *target);
static int register_get(struct reg *reg);

/*** Utility functions. ***/

bool supports_extension(struct target *target, char letter)
{
	riscv013_info_t *info = get_info(target);
	unsigned num;
	if (letter >= 'a' && letter <= 'z') {
		num = letter - 'a';
	} else if (letter >= 'A' && letter <= 'Z') {
		num = letter - 'A';
	} else {
		return false;
	}
	return info->misa & (1 << num);
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
	LOG_DEBUG("DTMCS: 0x%x -> 0x%x", out, in);

	return in;
}

static void increase_ac_busy_delay(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	info->ac_busy_delay += info->ac_busy_delay / 10 + 1;
	LOG_INFO("dtmcontrol_idle=%d, dmi_busy_delay=%d, ac_busy_delay=%d",
			info->dtmcontrol_idle, info->dmi_busy_delay,
			info->ac_busy_delay);
}

static void increase_dmi_busy_delay(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	info->dmi_busy_delay += info->dmi_busy_delay / 10 + 1;
	LOG_INFO("dtmcontrol_idle=%d, dmi_busy_delay=%d, ac_busy_delay=%d",
			info->dtmcontrol_idle, info->dmi_busy_delay,
			info->ac_busy_delay);

	dtmcontrol_scan(target, DTM_DTMCS_DMIRESET);
}

/**
 * exec: If this is set, assume the scan results in an execution, so more
 * run-test/idle cycles may be required.
 */
static dmi_status_t dmi_scan(struct target *target, uint16_t *address_in,
		uint64_t *data_in, dmi_op_t op, uint16_t address_out, uint64_t data_out,
		bool exec)
{
	riscv013_info_t *info = get_info(target);
	uint8_t in[8] = {0};
	uint8_t out[8];
	struct scan_field field = {
		.num_bits = info->abits + DTM_DMI_OP_LENGTH + DTM_DMI_DATA_LENGTH,
		.out_value = out,
	};

	if (address_in || data_in) {
		field.in_value = in;
	}

	assert(info->abits != 0);

	buf_set_u64(out, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, op);
	buf_set_u64(out, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, data_out);
	buf_set_u64(out, DTM_DMI_ADDRESS_OFFSET, info->abits, address_out);

	/* Assume dbus is already selected. */
	jtag_add_dr_scan(target->tap, 1, &field, TAP_IDLE);

	int idle_count = info->dtmcontrol_idle + info->dmi_busy_delay;
	if (exec)
		idle_count += info->ac_busy_delay;

	if (idle_count) {
		jtag_add_runtest(idle_count, TAP_IDLE);
	}

	int retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("dmi_scan failed jtag scan");
		return DMI_STATUS_FAILED;
	}

	if (data_in) {
		*data_in = buf_get_u64(in, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);
	}

	if (address_in) {
		*address_in = buf_get_u32(in, DTM_DMI_ADDRESS_OFFSET, info->abits);
	}

	dump_field(&field);

	return buf_get_u32(in, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
}

static uint64_t dmi_read(struct target *target, uint16_t address)
{
	uint64_t value;
	dmi_status_t status;
	uint16_t address_in;

	unsigned i = 0;
	for (i = 0; i < 256; i++) {
		status = dmi_scan(target, NULL, NULL, DMI_OP_READ, address, 0,
				false);
		if (status == DMI_STATUS_BUSY) {
			increase_dmi_busy_delay(target);
		} else {
			break;
		}
	}

	status = dmi_scan(target, &address_in, &value, DMI_OP_NOP, address, 0,
			false);

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
		dmi_scan(target, NULL, NULL, DMI_OP_WRITE, address, value,
				address == DMI_COMMAND);
		status = dmi_scan(target, NULL, NULL, DMI_OP_NOP, 0, 0, false);
		if (status == DMI_STATUS_BUSY) {
			increase_dmi_busy_delay(target);
		}
	}
	if (status != DMI_STATUS_SUCCESS) {
		LOG_ERROR("failed to write 0x%" PRIx64 " to 0x%x; status=%d\n", value, address, status);
	}
}

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

static int wait_for_idle(struct target *target, uint32_t *abstractcs)
{
	time_t start = time(NULL);
	while (1) {
		*abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		if (get_field(*abstractcs, DMI_ABSTRACTCS_BUSY) == 0) {
			return ERROR_OK;
		}
		if (time(NULL) - start > WALL_CLOCK_TIMEOUT) {
			LOG_ERROR("Timed out waiting for busy to go low. (abstractcs=0x%x)",
					*abstractcs);
			return ERROR_FAIL;
		}
	}
}

static int execute_abstract_command(struct target *target, uint32_t command)
{
	dmi_write(target, DMI_COMMAND, command);

	uint32_t abstractcs;
	if (wait_for_idle(target, &abstractcs) != ERROR_OK)
		return ERROR_FAIL;

	if (get_field(abstractcs, DMI_ABSTRACTCS_CMDERR) != CMDERR_NONE) {
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
		dmi_write(target, DMI_ABSTRACTCS, DMI_ABSTRACTCS_CMDERR);
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
		dmi_write(target, DMI_PROGBUF0 + i / 4, value);
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
		uint64_t *value,
		uint32_t reg_number, 
		unsigned width)
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

static int update_mstatus_actual(struct target *target)
{
	struct reg *mstatus_reg = &target->reg_cache->reg_list[REG_MSTATUS];
	if (mstatus_reg->valid) {
		// We previously made it valid.
		return ERROR_OK;
	}

	LOG_DEBUG("Reading mstatus");

	// Force reading the register. In that process mstatus_actual will be
	// updated.
	return register_get(&target->reg_cache->reg_list[REG_MSTATUS]);
}

static int register_write_direct(struct target *target, unsigned number,
		uint64_t value)
{
	riscv013_info_t *info = get_info(target);
	LOG_DEBUG("register 0x%x <- 0x%" PRIx64, number, value);

	if (number == REG_MSTATUS) {
		info->mstatus_actual = value;
	}

	int result = abstract_write_register(target, number, xlen(target), value);
	if (result == ERROR_OK)
		return result;

	// Fall back to program buffer.
	if (number >= REG_FPR0 && number <= REG_FPR31) {
		result = update_mstatus_actual(target);
		if (result != ERROR_OK) {
			return result;
		}
		if ((info->mstatus_actual & MSTATUS_FS) == 0) {
			result = register_write_direct(target, REG_MSTATUS, 
					set_field(info->mstatus_actual, MSTATUS_FS, 1));
			if (result != ERROR_OK)
				return result;
		}

		program_t *program = program_new();
		// TODO: Fully support D extension on RV32.
		if (supports_extension(target, 'D') && xlen(target) >= 64) {
			program_add32(program, fmv_d_x(number - REG_FPR0, S0));
		} else {
			program_add32(program, fmv_s_x(number - REG_FPR0, S0));
		}
		program_add32(program, ebreak());
		program_set_write(program, S0, value);
		result = execute_program(target, program);
		program_delete(program);
	} else if (number >= REG_CSR0 && number <= REG_CSR4095) {
		program_t *program = program_new();
		program_add32(program, csrw(S0, number - REG_CSR0));
		program_add32(program, ebreak());
		program_set_write(program, S0, value);
		result = execute_program(target, program);
		program_delete(program);
	} else {
		return result;
	}

	return result;
}

/** Actually read registers from the target right now. */
static int register_read_direct(struct target *target, uint64_t *value, uint32_t number)
{
	riscv013_info_t *info = get_info(target);
	int result = abstract_read_register(target, value, number, xlen(target));
	if (result == ERROR_OK)
		return result;

	// Fall back to program buffer.
	if (number >= REG_FPR0 && number <= REG_FPR31) {
		result = update_mstatus_actual(target);
		if (result != ERROR_OK) {
			return result;
		}
		if ((info->mstatus_actual & MSTATUS_FS) == 0) {
			result = register_write_direct(target, REG_MSTATUS,
					set_field(info->mstatus_actual, MSTATUS_FS, 1));
			if (result != ERROR_OK)
				return result;
		}
		LOG_DEBUG("mstatus_actual=0x%lx", info->mstatus_actual);

		program_t *program = program_new();
		if (supports_extension(target, 'D') && xlen(target) >= 64) {
			program_add32(program, fmv_x_d(S0, number - REG_FPR0));
		} else {
			program_add32(program, fmv_x_s(S0, number - REG_FPR0));
		}
		program_add32(program, ebreak());
		program_set_read(program, S0);
		result = execute_program(target, program);
		program_delete(program);
	} else if (number >= REG_CSR0 && number <= REG_CSR4095) {
		program_t *program = program_new();
		program_add32(program, csrr(S0, number - REG_CSR0));
		program_add32(program, ebreak());
		program_set_read(program, S0);
		result = execute_program(target, program);
		program_delete(program);
	} else {
		return result;
	}

	if (result != ERROR_OK)
		return result;

	result = register_read_direct(target, value, S0);
	if (result != ERROR_OK)
		return result;

	LOG_DEBUG("register 0x%x = 0x%" PRIx64, number, *value);

	return ERROR_OK;
}

static int maybe_read_tselect(struct target *target)
{
	riscv013_info_t *info = get_info(target);

	if (info->tselect_dirty) {
		int result = register_read_direct(target, &info->tselect, REG_TSELECT);
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
		int result = register_write_direct(target, REG_TSELECT, info->tselect);
		if (result != ERROR_OK)
			return result;
		info->tselect_dirty = true;
	}

	return ERROR_OK;
}

static void reg_cache_set(struct target *target, unsigned int number,
		uint64_t value)
{
	struct reg *r = &target->reg_cache->reg_list[number];
	LOG_DEBUG("%s <= 0x%" PRIx64, r->name, value);
	r->valid = true;
	buf_set_u64(r->value, 0, r->size, value);
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

static int execute_resume(struct target *target, bool step)
{
	riscv013_info_t *info = get_info(target);

	LOG_DEBUG("step=%d", step);

	maybe_write_tselect(target);

	// TODO: check if dpc is dirty (which also is true if an exception was hit
	// at any time)
	if (register_write_direct(target, REG_DPC, info->dpc) != ERROR_OK) {
		return ERROR_FAIL;
	}

	struct reg *mstatus_reg = &target->reg_cache->reg_list[REG_MSTATUS];
	if (mstatus_reg->valid) {
		uint64_t mstatus_user = buf_get_u64(mstatus_reg->value, 0, xlen(target));
		if (mstatus_user != info->mstatus_actual) {
			if (register_write_direct(target, REG_MSTATUS, mstatus_user) != ERROR_OK) {
				return ERROR_FAIL;
			}
		}
	}

	info->dcsr |= CSR_DCSR_EBREAKM | CSR_DCSR_EBREAKH | CSR_DCSR_EBREAKS |
		CSR_DCSR_EBREAKU;

	if (step) {
		info->dcsr |= CSR_DCSR_STEP;
	} else {
		info->dcsr &= ~CSR_DCSR_STEP;
	}

	if (register_write_direct(target, REG_DCSR, info->dcsr) != ERROR_OK) {
		return ERROR_FAIL;
	}

	// Restore GPRs
	if (register_write_direct(target, S0, reg_cache_get(target, S0)) != ERROR_OK) {
		return ERROR_FAIL;
	}
	if (register_write_direct(target, S1, reg_cache_get(target, S1)) != ERROR_OK) {
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

	target->state = TARGET_RUNNING;
	register_cache_invalidate(target->reg_cache);
	reg_cache_set(target, ZERO, 0);

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
		if (i == ZERO) {
			r->valid = true;
		} else {
			r->valid = false;
		}
	}
}

/*** OpenOCD target functions. ***/

static int register_get(struct reg *reg)
{
	struct target *target = (struct target *) reg->arch_info;
	riscv013_info_t *info = get_info(target);

	maybe_write_tselect(target);

	if (reg->number <= REG_XPR31) {
		buf_set_u64(reg->value, 0, xlen(target), reg_cache_get(target, reg->number));
		LOG_DEBUG("%s=0x%" PRIx64, reg->name, reg_cache_get(target, reg->number));
		return ERROR_OK;
	} else if (reg->number == REG_PC) {
		buf_set_u32(reg->value, 0, 32, info->dpc);
		reg->valid = true;
		LOG_DEBUG("%s=0x%" PRIx64 " (cached)", reg->name, info->dpc);
		return ERROR_OK;
	} else if (reg->number == REG_PRIV) {
		buf_set_u64(reg->value, 0, 8, get_field(info->dcsr, CSR_DCSR_PRV));
		LOG_DEBUG("%s=%d (cached)", reg->name,
				(int) get_field(info->dcsr, CSR_DCSR_PRV));
		return ERROR_OK;
	} else {
		uint64_t value;
		int result = register_read_direct(target, &value, reg->number);
		if (result != ERROR_OK) {
			return result;
		}
		LOG_DEBUG("%s=0x%" PRIx64, reg->name, value);
		buf_set_u64(reg->value, 0, xlen(target), value);

		if (reg->number == REG_MSTATUS) {
			info->mstatus_actual = value;
			reg->valid = true;
		}
	}

	return ERROR_OK;
}

static int register_write(struct target *target, unsigned int number,
		uint64_t value)
{
	riscv013_info_t *info = get_info(target);

	maybe_write_tselect(target);

	if (number == REG_PC) {
		info->dpc = value;
	} else if (number == REG_PRIV) {
		info->dcsr = set_field(info->dcsr, CSR_DCSR_PRV, value);
	} else {
		return register_write_direct(target, number, value);
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

	dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_HALTREQ |
			DMI_DMCONTROL_DMACTIVE);

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

		register_write_direct(target, REG_TSELECT, i);

		uint64_t tdata1;
		register_read_direct(target, &tdata1, REG_TDATA1);
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

		register_write_direct(target, REG_TDATA1, tdata1);

		uint64_t tdata1_rb;
		register_read_direct(target, &tdata1_rb, REG_TDATA1);
		LOG_DEBUG("tdata1=0x%" PRIx64, tdata1_rb);

		if (tdata1 != tdata1_rb) {
			LOG_DEBUG("Trigger %d doesn't support what we need; After writing 0x%"
					PRIx64 " to tdata1 it contains 0x%" PRIx64,
					i, tdata1, tdata1_rb);
			register_write_direct(target, REG_TDATA1, 0);
			continue;
		}

		register_write_direct(target, REG_TDATA2, trigger->address);

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
	register_write_direct(target, REG_TSELECT, i);
	register_write_direct(target, REG_TDATA1, 0);
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
	LOG_DEBUG("  dmireset=%d", get_field(dtmcontrol, DTM_DTMCS_DMIRESET));
	LOG_DEBUG("  idle=%d", get_field(dtmcontrol, DTM_DTMCS_IDLE));
	LOG_DEBUG("  dmistat=%d", get_field(dtmcontrol, DTM_DTMCS_DMISTAT));
	LOG_DEBUG("  abits=%d", get_field(dtmcontrol, DTM_DTMCS_ABITS));
	LOG_DEBUG("  version=%d", get_field(dtmcontrol, DTM_DTMCS_VERSION));
	if (dtmcontrol == 0) {
		LOG_ERROR("dtmcontrol is 0. Check JTAG connectivity/board power.");
		return ERROR_FAIL;
	}
	if (get_field(dtmcontrol, DTM_DTMCS_VERSION) != 1) {
		LOG_ERROR("Unsupported DTM version %d. (dtmcontrol=0x%x)",
				get_field(dtmcontrol, DTM_DTMCS_VERSION), dtmcontrol);
		return ERROR_FAIL;
	}

	riscv013_info_t *info = get_info(target);
	info->abits = get_field(dtmcontrol, DTM_DTMCS_ABITS);
	info->dtmcontrol_idle = get_field(dtmcontrol, DTM_DTMCS_IDLE);

	uint32_t dmcontrol = dmi_read(target, DMI_DMCONTROL);
	uint32_t dmstatus = dmi_read(target, DMI_DMSTATUS);
	if (get_field(dmstatus, DMI_DMSTATUS_VERSIONLO) != 2) {
		LOG_ERROR("OpenOCD only supports Debug Module version 2, not %d "
				"(dmstatus=0x%x)", get_field(dmstatus, DMI_DMSTATUS_VERSIONLO), dmstatus);
		return ERROR_FAIL;
	}

	// Reset the Debug Module.
	dmi_write(target, DMI_DMCONTROL, 0);
	dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE);
	dmcontrol = dmi_read(target, DMI_DMCONTROL);

	LOG_DEBUG("dmcontrol: 0x%08x", dmcontrol);
	LOG_DEBUG("dmstatus:  0x%08x", dmstatus);

	if (!get_field(dmcontrol, DMI_DMCONTROL_DMACTIVE)) {
		LOG_ERROR("Debug Module did not become active. dmcontrol=0x%x",
				dmcontrol);
		return ERROR_FAIL;
	}

	if (!get_field(dmstatus, DMI_DMSTATUS_AUTHENTICATED)) {
		LOG_ERROR("Authentication required by RISC-V core but not "
				"supported by OpenOCD. dmcontrol=0x%x", dmcontrol);
		return ERROR_FAIL;
	}

	if (get_field(dmstatus, DMI_DMSTATUS_ANYUNAVAIL)) {
		LOG_ERROR("The hart is unavailable.");
		return ERROR_FAIL;
	}

	if (get_field(dmstatus, DMI_DMSTATUS_ANYNONEXISTENT)) {
		LOG_ERROR("The hart doesn't exist.");
		return ERROR_FAIL;
	}

	// Check that abstract data registers are accessible.
	uint32_t abstractcs = dmi_read(target, DMI_ABSTRACTCS);
	LOG_DEBUG("abstractcs=0x%x", abstractcs);
	info->datacount = get_field(abstractcs, DMI_ABSTRACTCS_DATACOUNT);
	info->progsize = get_field(abstractcs, DMI_ABSTRACTCS_PROGSIZE);

	uint32_t value = 0x53467665;
	for (unsigned i = 0; i < info->datacount; i++) {
		dmi_write(target, DMI_DATA0 + i, value);
		value += 0x52534335;
	}

	for (unsigned i = 0; i < info->progsize; i++) {
		dmi_write(target, DMI_PROGBUF0 + i, value);
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
		uint32_t check = dmi_read(target, DMI_PROGBUF0 + i);
		if (check != value) {
			LOG_ERROR("Wrote 0x%x to dbus address 0x%x but got back 0x%x",
					value, DMI_PROGBUF0 + i, check);
			return ERROR_FAIL;
		}
		value += 0x52534335;
	}

	bool should_attempt_resume = false;
	if (get_field(dmstatus, DMI_DMSTATUS_ANYRUNNING)) {
		should_attempt_resume = true;
		LOG_DEBUG("Hart currently running. Requesting halt.\n");
		dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_HALTREQ |
				DMI_DMCONTROL_DMACTIVE);
		for (unsigned i = 0; i < 256; i++) {
			dmcontrol = dmi_read(target, DMI_DMCONTROL);
			dmstatus = dmi_read(target, DMI_DMSTATUS);
			if (get_field(dmstatus, DMI_DMSTATUS_ALLHALTED))
				break;
		}
		if (!get_field(dmstatus, DMI_DMSTATUS_ALLHALTED)) {
			LOG_ERROR("hart didn't halt; dmstatus=0x%x", dmstatus);
			return ERROR_FAIL;
		}
	}

	// TODO: do this using Quick Access, if supported.

	riscv_info_t *generic_info = (riscv_info_t *) target->arch_info;
	if (abstract_read_register(target, NULL, S0, 128) == ERROR_OK) {
		generic_info->xlen = 128;
	} else if (abstract_read_register(target, NULL, S0, 64) == ERROR_OK) {
		generic_info->xlen = 64;
	} else if (abstract_read_register(target, NULL, S0, 32) == ERROR_OK) {
		generic_info->xlen = 32;
	} else {
		LOG_ERROR("Failed to discover size using abstract register reads.");
		return ERROR_FAIL;
	}

	LOG_DEBUG("Discovered XLEN is %d", xlen(target));

	// Update register list to match discovered XLEN.
	update_reg_list(target);

	if (register_read_direct(target, &info->misa, REG_MISA) != ERROR_OK) {
		LOG_ERROR("Failed to read misa.");
		return ERROR_FAIL;
	}

	if (should_attempt_resume) {
		LOG_DEBUG("Resuming hart.\n");
		// Resume if the hart had been running.
		dmi_write(target, DMI_DMCONTROL, DMI_DMCONTROL_DMACTIVE |
				DMI_DMCONTROL_RESUMEREQ);
		for (unsigned i = 0; i < 256; i++) {
			dmcontrol = dmi_read(target, DMI_DMCONTROL);
			dmstatus = dmi_read(target, DMI_DMSTATUS);
			if (get_field(dmstatus, DMI_DMSTATUS_ALLRUNNING))
				break;
		}
		if (!get_field(dmstatus, DMI_DMSTATUS_ALLRUNNING)) {
			LOG_ERROR("hart didn't resume; dmstatus=0x%x", dmstatus);
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
		int result = abstract_read_register(target, &value, reg, xlen(target));
		if (result != ERROR_OK)
			return result;
		reg_cache_set(target, reg, value);
	}

	unsigned int csr[] = {REG_DPC, REG_DCSR};
	for (unsigned int i = 0; i < DIM(csr); i++) {
		uint64_t value;
		int reg = csr[i];
		int result = register_read_direct(target, &value, reg);
		if (result != ERROR_OK)
			return result;
		reg_cache_set(target, reg, value);
	}

	// TODO: get rid of those 2 variables and talk to the cache directly.
	info->dpc = reg_cache_get(target, REG_DPC);
	info->dcsr = reg_cache_get(target, REG_DCSR);

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
		case CSR_DCSR_CAUSE_SWBP:
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case CSR_DCSR_CAUSE_TRIGGER:
			target->debug_reason = DBG_REASON_WPTANDBKPT;
			// If we halted because of a data trigger, gdb doesn't know to do
			// the disable-breakpoints-step-enable-breakpoints dance.
			info->need_strict_step = true;
			break;
		case CSR_DCSR_CAUSE_DEBUGINT:
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case CSR_DCSR_CAUSE_STEP:
			target->debug_reason = DBG_REASON_SINGLESTEP;
			break;
		case CSR_DCSR_CAUSE_HALT:
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		default:
			LOG_ERROR("Invalid halt cause %d in CSR_DCSR (0x%" PRIx64 ")",
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
			register_write_direct(target, REG_TSELECT, info->trigger_count);
			uint64_t tselect_rb;
			register_read_direct(target, &tselect_rb, REG_TSELECT);
			if (info->trigger_count != tselect_rb)
				break;
			uint64_t tdata1;
			register_read_direct(target, &tdata1, REG_TDATA1);
			if ((tdata1 & MCONTROL_DMODE(xlen(target))) &&
					(tdata1 & (MCONTROL_EXECUTE | MCONTROL_STORE | MCONTROL_LOAD))) {
				register_write_direct(target, REG_TDATA1, 0);
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
	debug_level = old_debug_level;

	uint32_t dmstatus = dmi_read(target, DMI_DMSTATUS);
	if (get_field(dmstatus, DMI_DMSTATUS_ALLHALTED)) {
		if (target->state != TARGET_HALTED) {
			return handle_halt(target, announce);
		}
		return ERROR_OK;
	}

	if (get_field(dmstatus, DMI_DMSTATUS_ALLRUNNING)) {
		target->state = TARGET_RUNNING;
		return ERROR_OK;
	}

	if (get_field(dmstatus, DMI_DMSTATUS_ANYUNAVAIL)) {
		target->state = TARGET_RESET;
		return ERROR_OK;
	}

	if (get_field(dmstatus, DMI_DMSTATUS_ANYNONEXISTENT)) {
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
  select_dmi(target);
  dmi_write(target, DMI_DMCONTROL,
	    DMI_DMCONTROL_DMACTIVE | DMI_DMCONTROL_NDMRESET);
  return ERROR_OK;
}

static int deassert_reset(struct target *target)
{
  select_dmi(target);
  dmi_write(target, DMI_DMCONTROL,
	    DMI_DMCONTROL_DMACTIVE);
  return ERROR_OK;
}

static int read_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	select_dmi(target);

        while (1) {
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

          if (execute_abstract_command(target,
                                       AC_ACCESS_REGISTER_PREEXEC |
                                       abstract_register_size(xlen(target)) | reg_number_to_no(S1)) != ERROR_OK) {
            return ERROR_FAIL;
          }

          // Set up autoexec s.t. each read of the the result that was in S1
          // will start another run of reading the address pointed to by S0,
          // copying it to S1, and storing S1 into Data 0.
          if (count > 1) {
            dmi_write(target, DMI_ABSTRACTAUTO, 0x1 << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET);
          }
          
          uint32_t abstractcs;
          for (uint32_t i = 0; i < count; i++) {

            // On last iteration, turn off autoexec before reading the value
            // so that we don't inadvertently read too far into memory.
            if ((count > 1) && ((i + 1) == count)) {
              dmi_write(target, DMI_ABSTRACTAUTO, 0);
            }

            uint32_t value = dmi_read(target, DMI_DATA0);
            // If autoexec was set, the above dmi_read started an abstract command.
            // If we just immediately loop and do another read here,
            // we'll probably get a busy error. Wait for idle first,
            // or otherwise take ac_command_busy into account (this defeats the purpose
            // of autoexec, this whole code needs optimization).
            if ((count > 1) && ((i + 1) < count)) {
              if (wait_for_idle(target, &abstractcs) != ERROR_OK) {
                dmi_write(target, DMI_ABSTRACTAUTO, 0);
                return ERROR_FAIL;
              }
            }
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
          
          abstractcs = dmi_read(target, DMI_ABSTRACTCS);
          unsigned cmderr = get_field(abstractcs, DMI_ABSTRACTCS_CMDERR);
          if (cmderr == CMDERR_BUSY) {
	    // Clear the error and wait longer.
            dmi_write(target, DMI_ABSTRACTCS, DMI_ABSTRACTCS_CMDERR);
            increase_ac_busy_delay(target);
          } else if (cmderr) {
            LOG_ERROR("cmderr=%d", get_field(abstractcs, DMI_ABSTRACTCS_CMDERR));
            return ERROR_FAIL;
          } else {
            return ERROR_OK;
          }
        }
        // Should not get here.
        assert(0);
        return ERROR_OK;
}

/**
 * If there was a DMI error, clear that error and return 1.
 * Otherwise return 0.
 */
static int check_dmi_error(struct target *target)
{
	dmi_status_t status = dmi_scan(target, NULL, NULL, DMI_OP_NOP, 0, 0,
			false);
	if (status != DMI_STATUS_SUCCESS) {
		// Clear errors.
		dtmcontrol_scan(target, DTM_DTMCS_DMIRESET);
		increase_dmi_busy_delay(target);
		return 1;
	}
	return 0;
}

static int write_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	select_dmi(target);

	while (1) {
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

		scans_t *scans = scans_new(target, count + 10);
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
			scans_add_dmi_write(scans, DMI_DATA0, value, true);

			if (i == 0) {
				scans_add_dmi_write(scans, DMI_COMMAND,
						AC_ACCESS_REGISTER_WRITE | AC_ACCESS_REGISTER_POSTEXEC
						| abstract_register_size(xlen(target)) |
						reg_number_to_no(S1), true);
				scans_add_dmi_write(scans, DMI_ABSTRACTAUTO,
						0x1 << DMI_ABSTRACTAUTO_AUTOEXECDATA_OFFSET,
						false);
			}
		}
		int result = scans_execute(scans);
		scans_delete(scans);
		scans = NULL;
		if (result != ERROR_OK)
			return result;

		int dmi_error = check_dmi_error(target);

		// Clear autoexec.
		dmi_write(target, DMI_ABSTRACTAUTO, 0);
		uint32_t abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		unsigned cmderr = get_field(abstractcs, DMI_ABSTRACTCS_CMDERR);
		if (cmderr == CMDERR_BUSY) {
			dmi_write(target, DMI_ABSTRACTCS, DMI_ABSTRACTCS_CMDERR);
			increase_ac_busy_delay(target);
		} else if (cmderr) {
			LOG_ERROR("cmderr=%d", get_field(abstractcs, DMI_ABSTRACTCS_CMDERR));
			return ERROR_FAIL;
		} else if (!dmi_error) {
			break;
		}
	}

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
