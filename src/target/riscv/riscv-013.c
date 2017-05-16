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
#include "rtos/rtos.h"
#include "program.h"
#include "asm.h"
#include "batch.h"

#define DMI_DATA1 (DMI_DATA0 + 1)

static void riscv013_on_step_or_resume(struct target *target, bool step);
static void riscv013_step_or_resume_current_hart(struct target *target, bool step);
static riscv_addr_t riscv013_progbuf_addr(struct target *target);
static riscv_addr_t riscv013_progbuf_size(struct target *target);
static riscv_addr_t riscv013_data_size(struct target *target);
static riscv_addr_t riscv013_data_addr(struct target *target);
static void riscv013_set_autoexec(struct target *target, int offset, bool enabled);
static int riscv013_debug_buffer_register(struct target *target, riscv_addr_t addr);
static void riscv013_clear_abstract_error(struct target *target);

/* Implementations of the functions in riscv_info_t. */
static riscv_reg_t riscv013_get_register(struct target *target, int hartid, int regid);
static void riscv013_set_register(struct target *target, int hartid, int regid, uint64_t value);
static void riscv013_select_current_hart(struct target *target);
static void riscv013_halt_current_hart(struct target *target);
static void riscv013_resume_current_hart(struct target *target);
static void riscv013_step_current_hart(struct target *target);
static void riscv013_on_halt(struct target *target);
static void riscv013_on_step(struct target *target);
static void riscv013_on_resume(struct target *target);
static bool riscv013_is_halted(struct target *target);
static enum riscv_halt_reason riscv013_halt_reason(struct target *target);
static void riscv013_debug_buffer_enter(struct target *target, struct riscv_program *p);
static void riscv013_debug_buffer_leave(struct target *target, struct riscv_program *p);
static void riscv013_write_debug_buffer(struct target *target, int i, riscv_insn_t d);
static riscv_insn_t riscv013_read_debug_buffer(struct target *target, int i);
static int riscv013_execute_debug_buffer(struct target *target);
static void riscv013_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d);
static void riscv013_fill_dmi_read_u64(struct target *target, char *buf, int a);
static int riscv013_dmi_write_u64_bits(struct target *target);
static void riscv013_fill_dmi_nop_u64(struct target *target, char *buf);
static void riscv013_reset_current_hart(struct target *target);

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

#define RISCV013_INFO(r) riscv013_info_t *r = get_info(target)

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

	// Some memoized values
	int progbuf_size, progbuf_addr, data_addr, data_size;
} riscv013_info_t;

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

/*** Necessary prototypes. ***/

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

static void increase_dmi_busy_delay(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	info->dmi_busy_delay += info->dmi_busy_delay / 10 + 1;
	LOG_INFO("dtmcontrol_idle=%d, dmi_busy_delay=%d, ac_busy_delay=%d",
			info->dtmcontrol_idle, info->dmi_busy_delay,
			info->ac_busy_delay);

	dtmcontrol_scan(target, DTM_DTMCS_DMIRESET);
}

static void increase_ac_busy_delay(struct target *target)
{
	riscv013_info_t *info = get_info(target);
	info->ac_busy_delay += info->ac_busy_delay / 10 + 1;
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

	int idle_count = info->dmi_busy_delay;
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
	select_dmi(target);

	uint64_t value;
	dmi_status_t status;
	uint16_t address_in;

	unsigned i = 0;

        // This first loop ensures that the read request was actually sent
        // to the target. Note that if for some reason this stays busy,
        // it is actually due to the Previous dmi_read or dmi_write.
	for (i = 0; i < 256; i++) {
		status = dmi_scan(target, NULL, NULL, DMI_OP_READ, address, 0,
				false);
		if (status == DMI_STATUS_BUSY) {
			increase_dmi_busy_delay(target);
		} else if (status == DMI_STATUS_SUCCESS) {
			break;
		} else {
			LOG_ERROR("failed read from 0x%x, status=%d\n", address, status);
			break;
		}
	}

        if (status != DMI_STATUS_SUCCESS) {
                LOG_ERROR("Failed read from 0x%x; value=0x%" PRIx64 ", status=%d\n",
                                address, value, status);
                abort();
        }

         // This second loop ensures that we got the read
         // data back. Note that NOP can result in a 'busy' result as well, but
         // that would be noticed on the next DMI access we do.
         for (i = 0; i < 256; i++) {
           status = dmi_scan(target, &address_in, &value, DMI_OP_NOP, address, 0,
                             false);
           if (status == DMI_STATUS_BUSY) {
             increase_dmi_busy_delay(target);
           } else if (status == DMI_STATUS_SUCCESS) {
             break;
           } else {
             LOG_ERROR("failed read (NOP) at 0x%x, status=%d\n", address, status);
             break;
           }
        }

	if (status != DMI_STATUS_SUCCESS) {
		LOG_ERROR("Failed read (NOP) from 0x%x; value=0x%" PRIx64 ", status=%d\n",
				address, value, status);
		abort();
	}

	return value;
}

static void dmi_write(struct target *target, uint16_t address, uint64_t value)
{
	select_dmi(target);
	dmi_status_t status = DMI_STATUS_BUSY;
	unsigned i = 0;

         // The first loop ensures that we successfully sent the write request.
        for (i = 0; i < 256; i++) {
           status = dmi_scan(target, NULL, NULL, DMI_OP_WRITE, address, value,
                             address == DMI_COMMAND);
           if (status == DMI_STATUS_BUSY) {
             increase_dmi_busy_delay(target);
           } else if (status == DMI_STATUS_SUCCESS) {
             break;
           } else {
             LOG_ERROR("failed write to 0x%x, status=%d\n", address, status);
             break;
           }
        }

         if (status != DMI_STATUS_SUCCESS) {
           LOG_ERROR("Failed write to 0x%x;, status=%d\n",
                     address, status);
           abort();
        }

         // The second loop isn't strictly necessary, but would ensure that
         // the write is complete/ has no non-busy errors before returning from this function.
         for (i = 0; i < 256; i++) {
           status = dmi_scan(target, NULL, NULL, DMI_OP_NOP, address, 0,
                             false);
           if (status == DMI_STATUS_BUSY) {
             increase_dmi_busy_delay(target);
           } else if (status == DMI_STATUS_SUCCESS) {
             break;
           } else {
             LOG_ERROR("failed write (NOP) at 0x%x, status=%d\n", address, status);
             break;
           }
	}
	if (status != DMI_STATUS_SUCCESS) {
		LOG_ERROR("failed to write (NOP) 0x%" PRIx64 " to 0x%x; status=%d\n", value, address, status);
		abort();
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
			if (get_field(*abstractcs, DMI_ABSTRACTCS_CMDERR) != CMDERR_NONE) {
				const char *errors[8] = {
					"none",
					"busy",
					"not supported",
					"exception",
					"halt/resume",
					"reserved",
					"reserved",
					"other" };

				LOG_ERROR("Abstract command ended in error '%s' (abstractcs=0x%x)",
						errors[get_field(*abstractcs, DMI_ABSTRACTCS_CMDERR)],
						*abstractcs);
			}

			LOG_ERROR("Timed out waiting for busy to go low. (abstractcs=0x%x)",
					*abstractcs);
			return ERROR_FAIL;
		}
	}
}

static int register_read_direct(struct target *target, uint64_t *value, uint32_t number);

static int register_write_direct(struct target *target, unsigned number,
		uint64_t value)
{
	struct riscv_program program;
	riscv_program_init(&program, target);

	riscv_addr_t input = riscv_program_alloc_d(&program);
	riscv_program_write_ram(&program, input + 4, value >> 32);
	riscv_program_write_ram(&program, input, value);

	assert(GDB_REGNO_XPR0 == 0);
	if (number <= GDB_REGNO_XPR31) {
		riscv_program_lx(&program, number, input);
	} else if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
		riscv_program_fld(&program, number, input);
	} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
		enum gdb_regno temp = riscv_program_gettemp(&program);
		riscv_program_lx(&program, temp, input);
		riscv_program_csrw(&program, temp, number);
	} else {
		LOG_ERROR("Unsupported register (enum gdb_regno)(%d)", number);
		abort();
	}

	int exec_out = riscv_program_exec(&program, target);
	if (exec_out != ERROR_OK) {
		LOG_ERROR("Unable to execute program");
		return exec_out;
	}

	return ERROR_OK;
}

/** Actually read registers from the target right now. */
static int register_read_direct(struct target *target, uint64_t *value, uint32_t number)
{
	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_addr_t output = riscv_program_alloc_d(&program);
	riscv_program_write_ram(&program, output + 4, 0);
	riscv_program_write_ram(&program, output, 0);

	assert(GDB_REGNO_XPR0 == 0);
	if (number <= GDB_REGNO_XPR31) {
		riscv_program_sx(&program, number, output);
	} else if (number >= GDB_REGNO_FPR0 && number <= GDB_REGNO_FPR31) {
		riscv_program_fsd(&program, number, output);
	} else if (number >= GDB_REGNO_CSR0 && number <= GDB_REGNO_CSR4095) {
		LOG_DEBUG("reading CSR index=0x%03x", number - GDB_REGNO_CSR0);
		enum gdb_regno temp = riscv_program_gettemp(&program);
		riscv_program_csrr(&program, temp, number);
		riscv_program_sx(&program, temp, output);
	} else {
		LOG_ERROR("Unsupported register (enum gdb_regno)(%d)", number);
		abort();
	}

	int exec_out = riscv_program_exec(&program, target);
	if (exec_out != ERROR_OK) {
		LOG_ERROR("Unable to execute program");
		return exec_out;
	}

	*value = 0;
	*value |= ((uint64_t)(riscv_program_read_ram(&program, output + 4))) << 32;
	*value |= riscv_program_read_ram(&program, output);
	LOG_DEBUG("register 0x%x = 0x%" PRIx64, number, *value);
	return ERROR_OK;
}

static int maybe_read_tselect(struct target *target)
{
	riscv013_info_t *info = get_info(target);

	if (info->tselect_dirty) {
		int result = register_read_direct(target, &info->tselect, GDB_REGNO_TSELECT);
		if (result != ERROR_OK)
			return result;
		info->tselect_dirty = false;
	}

	return ERROR_OK;
}

/*** OpenOCD target functions. ***/

static int register_get(struct reg *reg)
{
	struct target *target = (struct target *) reg->arch_info;
	uint64_t value = riscv_get_register(target, reg->number);
	buf_set_u64(reg->value, 0, 64, value);
	return ERROR_OK;
}

static int register_write(struct target *target, unsigned int number,
		uint64_t value)
{
	riscv_set_register(target, number, value);
	return ERROR_OK;
}

static int register_set(struct reg *reg, uint8_t *buf)
{
	struct target *target = (struct target *) reg->arch_info;

	uint64_t value = buf_get_u64(buf, 0, riscv_xlen(target));

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

static int init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	LOG_DEBUG("init");
	riscv_info_t *generic_info = (riscv_info_t *) target->arch_info;

	riscv_info_init(generic_info);
	generic_info->get_register = &riscv013_get_register;
	generic_info->set_register = &riscv013_set_register;
	generic_info->select_current_hart = &riscv013_select_current_hart;
	generic_info->is_halted = &riscv013_is_halted;
	generic_info->halt_current_hart = &riscv013_halt_current_hart;
	generic_info->resume_current_hart = &riscv013_resume_current_hart;
	generic_info->step_current_hart = &riscv013_step_current_hart;
	generic_info->on_halt = &riscv013_on_halt;
	generic_info->on_resume = &riscv013_on_resume;
	generic_info->on_step = &riscv013_on_step;
	generic_info->halt_reason = &riscv013_halt_reason;
	generic_info->debug_buffer_enter = &riscv013_debug_buffer_enter;
	generic_info->debug_buffer_leave = &riscv013_debug_buffer_leave;
	generic_info->read_debug_buffer = &riscv013_read_debug_buffer;
	generic_info->write_debug_buffer = &riscv013_write_debug_buffer;
	generic_info->execute_debug_buffer = &riscv013_execute_debug_buffer;
	generic_info->fill_dmi_write_u64 = &riscv013_fill_dmi_write_u64;
	generic_info->fill_dmi_read_u64 = &riscv013_fill_dmi_read_u64;
	generic_info->fill_dmi_nop_u64 = &riscv013_fill_dmi_nop_u64;
	generic_info->dmi_write_u64_bits = &riscv013_dmi_write_u64_bits;
	generic_info->reset_current_hart = &riscv013_reset_current_hart;

	generic_info->version_specific = calloc(1, sizeof(riscv013_info_t));
	if (!generic_info->version_specific)
		return ERROR_FAIL;
	riscv013_info_t *info = get_info(target);

	info->progbuf_size = -1;
	info->progbuf_addr = -1;
	info->data_size = -1;
	info->data_addr = -1;

	info->dmi_busy_delay = 0;
	info->ac_busy_delay = 0;

	target->reg_cache = calloc(1, sizeof(*target->reg_cache));
	target->reg_cache->name = "RISC-V registers";
	target->reg_cache->num_regs = GDB_REGNO_COUNT;

	target->reg_cache->reg_list = calloc(GDB_REGNO_COUNT, sizeof(struct reg));

	const unsigned int max_reg_name_len = 12;
	info->reg_names = calloc(1, GDB_REGNO_COUNT * max_reg_name_len);
	char *reg_name = info->reg_names;
	info->reg_values = NULL;

	for (unsigned int i = 0; i < GDB_REGNO_COUNT; i++) {
		struct reg *r = &target->reg_cache->reg_list[i];
		r->number = i;
		r->caller_save = true;
		r->dirty = false;
		r->valid = false;
		r->exist = true;
		r->type = &riscv_reg_arch_type;
		r->arch_info = target;
		if (i <= GDB_REGNO_XPR31) {
			sprintf(reg_name, "x%d", i);
		} else if (i == GDB_REGNO_PC) {
			sprintf(reg_name, "pc");
		} else if (i >= GDB_REGNO_FPR0 && i <= GDB_REGNO_FPR31) {
			sprintf(reg_name, "f%d", i - GDB_REGNO_FPR0);
		} else if (i >= GDB_REGNO_CSR0 && i <= GDB_REGNO_CSR4095) {
			sprintf(reg_name, "csr%d", i - GDB_REGNO_CSR0);
		} else if (i == GDB_REGNO_PRIV) {
			sprintf(reg_name, "priv");
		}
		if (reg_name[0]) {
			r->name = reg_name;
		}
		reg_name += strlen(reg_name) + 1;
		assert(reg_name < info->reg_names + GDB_REGNO_COUNT * max_reg_name_len);
	}
#if 0
	update_reg_list(target);
#endif

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

	int i;
	for (i = 0; i < riscv_count_triggers(target); i++) {
		if (info->trigger_unique_id[i] != -1) {
			continue;
		}

		register_write_direct(target, GDB_REGNO_TSELECT, i);

		uint64_t tdata1;
		register_read_direct(target, &tdata1, GDB_REGNO_TDATA1);
		int type = get_field(tdata1, MCONTROL_TYPE(riscv_xlen(target)));

		if (type != 2) {
			continue;
		}

		if (tdata1 & (MCONTROL_EXECUTE | MCONTROL_STORE | MCONTROL_LOAD)) {
			// Trigger is already in use, presumably by user code.
			continue;
		}

		// address/data match trigger
		tdata1 |= MCONTROL_DMODE(riscv_xlen(target));
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

		register_write_direct(target, GDB_REGNO_TDATA1, tdata1);

		uint64_t tdata1_rb;
		register_read_direct(target, &tdata1_rb, GDB_REGNO_TDATA1);
		LOG_DEBUG("tdata1=0x%" PRIx64, tdata1_rb);

		if (tdata1 != tdata1_rb) {
			LOG_DEBUG("Trigger %d doesn't support what we need; After writing 0x%"
					PRIx64 " to tdata1 it contains 0x%" PRIx64,
					i, tdata1, tdata1_rb);
			register_write_direct(target, GDB_REGNO_TDATA1, 0);
			continue;
		}

		register_write_direct(target, GDB_REGNO_TDATA2, trigger->address);

		LOG_DEBUG("Using resource %d for bp %d", i,
				trigger->unique_id);
		info->trigger_unique_id[i] = trigger->unique_id;
		break;
	}
	if (i >= riscv_count_triggers(target)) {
		LOG_ERROR("Couldn't find an available hardware trigger.");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	return ERROR_OK;
}

static int remove_trigger(struct target *target, struct trigger *trigger)
{
	riscv013_info_t *info = get_info(target);

	maybe_read_tselect(target);

	int i;
	for (i = 0; i < riscv_count_triggers(target); i++) {
		if (info->trigger_unique_id[i] == trigger->unique_id) {
			break;
		}
	}
	if (i >= riscv_count_triggers(target)) {
		LOG_ERROR("Couldn't find the hardware resources used by hardware "
				"trigger.");
		return ERROR_FAIL;
	}
	LOG_DEBUG("Stop using resource %d for bp %d", i, trigger->unique_id);
	register_write_direct(target, GDB_REGNO_TSELECT, i);
	register_write_direct(target, GDB_REGNO_TDATA1, 0);
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
	info->datacount = get_field(abstractcs, DMI_ABSTRACTCS_DATACOUNT);
	info->progsize = get_field(abstractcs, DMI_ABSTRACTCS_PROGSIZE);

	/* Before doing anything else we must first enumerate the harts. */
	RISCV_INFO(r);
	if (riscv_rtos_enabled(target)) {
		for (int i = 0; i < RISCV_MAX_HARTS; ++i) {
			riscv_set_current_hartid(target, i);
			uint32_t s = dmi_read(target, DMI_DMSTATUS);
			if (get_field(s, DMI_DMSTATUS_ANYNONEXISTENT))
				break;
			r->hart_count = i + 1;
		}
	} else {
		r->hart_count = 1;
	}

	LOG_DEBUG("Enumerated %d harts", r->hart_count);

	/* Halt every hart so we can probe them. */
	riscv_halt_all_harts(target);

	/* Find the address of the program buffer, which must be done without
	 * knowing anything about the target. */
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		riscv_set_current_hartid(target, i);

		/* Without knowing anything else we can at least mess with the
		 * program buffer. */
		r->debug_buffer_size[i] = riscv013_progbuf_size(target);

		/* Guess this is a 32-bit system, we're probing it. */
		r->xlen[i] = 32;

		/* First find the low 32 bits of the program buffer.  This is
		 * used to check for alignment. */
		struct riscv_program program32;
		riscv_program_init(&program32, target);
		riscv_program_csrrw(&program32, GDB_REGNO_S0, GDB_REGNO_S0, GDB_REGNO_DSCRATCH);
		riscv_program_insert(&program32, auipc(GDB_REGNO_S0));
		riscv_program_insert(&program32, sw(GDB_REGNO_S0, GDB_REGNO_S0, -4));
		riscv_program_csrrw(&program32, GDB_REGNO_S0, GDB_REGNO_S0, GDB_REGNO_DSCRATCH);
		riscv_program_fence(&program32);
		riscv_program_exec(&program32, target);

		riscv_addr_t progbuf_addr = dmi_read(target, DMI_PROGBUF0) - 4;
		if (get_field(dmi_read(target, DMI_ABSTRACTCS), DMI_ABSTRACTCS_CMDERR) != 0) {
			LOG_ERROR("Unable to find the address of the program buffer on hart %d", i);
			r->xlen[i] = -1;
			continue;
		}
		r->debug_buffer_addr[i] = progbuf_addr;

		/* Check to see if the core can execute 64 bit instructions.
		 * In order to make this work we first need to */
		int offset = (progbuf_addr % 8 == 0) ? -4 : 0;

		struct riscv_program program64;
		riscv_program_init(&program64, target);
		riscv_program_csrrw(&program64, GDB_REGNO_S0, GDB_REGNO_S0, GDB_REGNO_DSCRATCH);
		riscv_program_insert(&program64, auipc(GDB_REGNO_S0));
		riscv_program_insert(&program64, sd(GDB_REGNO_S0, GDB_REGNO_S0, offset));
		riscv_program_csrrw(&program64, GDB_REGNO_S0, GDB_REGNO_S0, GDB_REGNO_DSCRATCH);
		riscv_program_fence(&program64);
		riscv_program_exec(&program64, target);

		if (get_field(dmi_read(target, DMI_ABSTRACTCS), DMI_ABSTRACTCS_CMDERR) == 0) {
			r->debug_buffer_addr[i] =
				(dmi_read(target, DMI_PROGBUF0 + (8 + offset) / 4) << 32)
				+ dmi_read(target, DMI_PROGBUF0 + (4 + offset) / 4)
				- 4;
			r->xlen[i] = 64;
		}

		LOG_DEBUG("hart %d has XLEN=%d", i, r->xlen[i]);
		LOG_DEBUG("found program buffer at 0x%08lx", (long)(r->debug_buffer_addr[i]));

		if (riscv_program_gah(&program64, r->debug_buffer_addr[i])) {
                	LOG_ERROR("This implementation will not work with hart %d with debug_buffer_addr of 0x%lx\n", i, 
                            (long)r->debug_buffer_addr[i]);
			abort();
                }
		
		/* Check to see if we can use the data words as an extended
		 * program buffer or not. */
		if (r->debug_buffer_addr[i] + (4 * r->debug_buffer_size[i]) == riscv013_data_addr(target)) {
			r->debug_buffer_size[i] += riscv013_data_size(target);
			LOG_DEBUG("extending the debug buffer using data words, total size %d", r->debug_buffer_size[i]);
		}
	}

	/* Then we check the number of triggers availiable to each hart. */
	for (int i = 0; i < riscv_count_harts(target); ++i) {
		for (uint32_t t = 0; t < RISCV_MAX_TRIGGERS; ++t) {
			riscv_set_current_hartid(target, i);

			r->trigger_count[i] = t;
			register_write_direct(target, GDB_REGNO_TSELECT, t);
			uint64_t tselect = t+1;
			register_read_direct(target, &tselect, GDB_REGNO_TSELECT);
			if (tselect != t)
				break;
		}
	}

	/* Resumes all the harts, so the debugger can later pause them. */
	riscv_resume_all_harts(target);
	target_set_examined(target);
        
        // This print is used by some regression suites to know when
        // they can connect with gdb/telnet.
        // We will need to update those suites if we want to remove this line.
        LOG_INFO("Examined RISC-V core");
	return ERROR_OK;
}

static int assert_reset(struct target *target)
{
  /*FIXME -- this only works for single-hart.*/
  RISCV_INFO(r);
  assert(r->current_hartid == 0);

  select_dmi(target);
  LOG_DEBUG("ASSERTING NDRESET");
  uint32_t control = dmi_read(target, DMI_DMCONTROL);
  control = set_field(control, DMI_DMCONTROL_NDMRESET, 1);
  if (target->reset_halt) {
    LOG_DEBUG("TARGET RESET HALT SET, ensuring halt is set during reset.");
    control = set_field(control, DMI_DMCONTROL_HALTREQ, 1);
  } else {
    LOG_DEBUG("TARGET RESET HALT NOT SET");
    control = set_field(control, DMI_DMCONTROL_HALTREQ, 0);
  }

  dmi_write(target, DMI_DMCONTROL,
	    control);

  return ERROR_OK;
}

static int deassert_reset(struct target *target)
{
  RISCV_INFO(r);
  RISCV013_INFO(info);

  select_dmi(target);

  /*FIXME -- this only works for Single Hart*/
  assert(r->current_hartid == 0);

  /*FIXME -- is there bookkeeping we need to do here*/
  
  uint32_t control = dmi_read(target, DMI_DMCONTROL);

  // Clear the reset, but make sure haltreq is still set
  if (target->reset_halt) {
    control = set_field(control, DMI_DMCONTROL_HALTREQ, 1);
  }

  control = set_field(control, DMI_DMCONTROL_NDMRESET, 0);
  dmi_write(target, DMI_DMCONTROL, control);

  uint32_t status;
  int dmi_busy_delay = info->dmi_busy_delay;
  if (target->reset_halt) {
    LOG_DEBUG("DEASSERTING RESET, waiting for hart to be halted.");
    do {
      status = dmi_read(target, DMI_DMSTATUS);
    } while (get_field(status, DMI_DMSTATUS_ALLHALTED) == 0);
  } else {
    LOG_DEBUG("DEASSERTING RESET, waiting for hart to be running.");
    do {
      status = dmi_read(target, DMI_DMSTATUS);
      if (get_field(status, DMI_DMSTATUS_ANYHALTED) ||
	  get_field(status, DMI_DMSTATUS_ANYUNAVAIL)) {
	LOG_ERROR("Unexpected hart status during reset.");
	abort();
      }
    } while (get_field(status, DMI_DMSTATUS_ALLRUNNING) == 0);
  }
  info->dmi_busy_delay = dmi_busy_delay;
  return ERROR_OK;
}

static int read_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	RISCV013_INFO(info);

	LOG_DEBUG("reading %d words of %d bytes from 0x%08lx", count, size, (long)address);

	select_dmi(target);
	riscv_set_current_hartid(target, 0);

	/* This program uses two temporary registers.  A word of data and the
	 * associated address are stored at some location in memory.  The
	 * program loads the word from that address and then increments the
	 * address.  The debugger is expected to pull the memory word-by-word
	 * from the chip with AUTOEXEC set in order to trigger program
	 * execution on every word. */
	uint64_t s0 = riscv_get_register(target, GDB_REGNO_S0);
	uint64_t s1 = riscv_get_register(target, GDB_REGNO_S1);

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_addr_t r_data = riscv_program_alloc_w(&program);
	riscv_addr_t r_addr = riscv_program_alloc_x(&program);
	riscv_program_fence(&program);
	riscv_program_lx(&program, GDB_REGNO_S0, r_addr);
	riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size);
	switch (size) {
		case 1:
			riscv_program_lbr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 2:
			riscv_program_lhr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 4:
			riscv_program_lwr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}
	riscv_program_sw(&program, GDB_REGNO_S1, r_data);
	riscv_program_sx(&program, GDB_REGNO_S0, r_addr);

	/* The first round through the program's execution we use the regular
	 * program execution mechanism. */
	switch (riscv_xlen(target)) {
	case 64:
		riscv_program_write_ram(&program, r_addr + 4, (((riscv_addr_t) address) - size) >> 32);
	case 32:
		riscv_program_write_ram(&program, r_addr, ((riscv_addr_t) address) - size);
		break;
	default:
		LOG_ERROR("unknown XLEN %d", riscv_xlen(target));
		return ERROR_FAIL;
	}

	if (riscv_program_exec(&program, target) != ERROR_OK) {
		uint32_t acs = dmi_read(target, DMI_ABSTRACTCS);
		LOG_ERROR("failed to execute program, abstractcs=0x%08x", acs);
		riscv013_clear_abstract_error(target);
		riscv_set_register(target, GDB_REGNO_S0, s0);
		riscv_set_register(target, GDB_REGNO_S1, s1);
		LOG_ERROR("  exiting with ERROR_FAIL");
		return ERROR_FAIL;
	}

	uint32_t value = riscv_program_read_ram(&program, r_data);
	LOG_DEBUG("M[0x%08lx] reads 0x%08lx", (long)address, (long)value);
	switch (size) {
	case 1:
		buffer[0] = value;
		break;
	case 2:
		buffer[0] = value;
		buffer[1] = value >> 8;
		break;
	case 4:
		buffer[0] = value;
		buffer[1] = value >> 8;
		buffer[2] = value >> 16;
		buffer[3] = value >> 24;
		break;
	default:
		LOG_ERROR("unsupported access size: %d", size);
		return ERROR_FAIL;
	}

	/* The rest of this program is designed to be fast so it reads various
	 * DMI registers directly. */
	int d_data = (r_data - riscv_debug_buffer_addr(target)) / 4;
	int d_addr = (r_addr - riscv_debug_buffer_addr(target)) / 4;

	riscv013_set_autoexec(target, d_data, 1);

	/* Copying memory might fail because we're going too quickly, in which
	 * case we need to back off a bit and try again.  There's two
	 * termination conditions to this loop: a non-BUSY error message, or
	 * the data was all copied. */
	riscv_addr_t cur_addr = 0xbadbeef;
	riscv_addr_t fin_addr = address + (count * size);
	riscv_addr_t prev_addr = ((riscv_addr_t) address) - size;
	LOG_DEBUG("writing until final address 0x%016lx", fin_addr);
	while (count > 1 && (cur_addr = riscv_read_debug_buffer_x(target, d_addr)) < fin_addr) {
		LOG_DEBUG("transferring burst starting at address 0x%016lx (previous burst was 0x%016lx)", cur_addr, prev_addr);
		assert(prev_addr < cur_addr);
		prev_addr = cur_addr;
		riscv_addr_t start = (cur_addr - address) / size;
                assert (cur_addr >= address);
                struct riscv_batch *batch = riscv_batch_alloc(
			target,
			32,
			info->dmi_busy_delay + info->ac_busy_delay);

		size_t reads = 0;
		size_t rereads = reads;
		for (riscv_addr_t i = start; i < count; ++i) {
			size_t index = 
				riscv_batch_add_dmi_read(
					batch,
					riscv013_debug_buffer_register(target, r_data));
			assert(index == reads);
			reads++;
			if (riscv_batch_full(batch))
				break;
		}

		riscv_batch_run(batch);

		for (size_t i = start; i < start + reads; ++i) {
			riscv_addr_t offset = size*i;
			riscv_addr_t t_addr = address + offset;
			uint8_t *t_buffer = buffer + offset;

			uint64_t dmi_out = riscv_batch_get_dmi_read(batch, rereads);
			value = get_field(dmi_out, DTM_DMI_DATA);
			rereads++;

			switch (size) {
			case 1:
				t_buffer[0] = value;
				break;
			case 2:
				t_buffer[0] = value;
				t_buffer[1] = value >> 8;
				break;
			case 4:
				t_buffer[0] = value;
				t_buffer[1] = value >> 8;
				t_buffer[2] = value >> 16;
				t_buffer[3] = value >> 24;
				break;
			default:
				LOG_ERROR("unsupported access size: %d", size);
				return ERROR_FAIL;
			}

			LOG_DEBUG("M[0x%08lx] reads 0x%08lx", (long)t_addr, (long)value);
		}

		riscv_batch_free(batch);

                // Note that if the scan resulted in a Busy DMI response, it
                // is this read to abstractcs that will cause the dmi_busy_delay
                // to be incremented if necessary. The loop condition above
                // catches the case where no writes went through at all.

		uint32_t abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		while (get_field(abstractcs, DMI_ABSTRACTCS_BUSY))
			abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		switch (get_field(abstractcs, DMI_ABSTRACTCS_CMDERR)) {
		case CMDERR_NONE:
			LOG_DEBUG("successful (partial?) memory write");
			break;
		case CMDERR_BUSY:
			LOG_DEBUG("memory write resulted in busy response");
			riscv013_clear_abstract_error(target);
			increase_ac_busy_delay(target);
			break;
		default:
			LOG_ERROR("error when writing memory, abstractcs=0x%08lx", (long)abstractcs);
			riscv013_set_autoexec(target, d_data, 0);
			riscv_set_register(target, GDB_REGNO_S0, s0);
			riscv_set_register(target, GDB_REGNO_S1, s1);
			riscv013_clear_abstract_error(target);
			return ERROR_FAIL;
		}
	}

	riscv013_set_autoexec(target, d_data, 0);
	riscv_set_register(target, GDB_REGNO_S0, s0);
	riscv_set_register(target, GDB_REGNO_S1, s1);
	return ERROR_OK;
}

static int write_memory(struct target *target, uint32_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV013_INFO(info);

	LOG_DEBUG("writing %d words of %d bytes to 0x%08lx", count, size, (long)address);

	select_dmi(target);
	riscv_set_current_hartid(target, 0);

	/* This program uses two temporary registers.  A word of data and the
	 * associated address are stored at some location in memory.  The
	 * program stores the word to that address and then increments the
	 * address.  The debugger is expected to feed the memory word-by-word
	 * into the chip with AUTOEXEC set in order to trigger program
	 * execution on every word. */
	uint64_t s0 = riscv_get_register(target, GDB_REGNO_S0);
	uint64_t s1 = riscv_get_register(target, GDB_REGNO_S1);

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_addr_t r_data = riscv_program_alloc_w(&program);
	riscv_addr_t r_addr = riscv_program_alloc_x(&program);
	riscv_program_fence(&program);
	riscv_program_lx(&program, GDB_REGNO_S0, r_addr);
	riscv_program_lw(&program, GDB_REGNO_S1, r_data);

	switch (size) {
		case 1:
			riscv_program_sbr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 2:
			riscv_program_shr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		case 4:
			riscv_program_swr(&program, GDB_REGNO_S1, GDB_REGNO_S0, 0);
			break;
		default:
			LOG_ERROR("Unsupported size: %d", size);
			return ERROR_FAIL;
	}

	riscv_program_addi(&program, GDB_REGNO_S0, GDB_REGNO_S0, size);
	riscv_program_sx(&program, GDB_REGNO_S0, r_addr);

	/* The first round through the program's execution we use the regular
	 * program execution mechanism. */
	uint32_t value;
	switch (size) {
		case 1:
			value = buffer[0];
			break;
		case 2:
			value = buffer[0]
				| ((uint32_t) buffer[1] << 8);
			break;
		case 4:
			value = buffer[0]
				| ((uint32_t) buffer[1] << 8)
				| ((uint32_t) buffer[2] << 16)
				| ((uint32_t) buffer[3] << 24);
			break;
		default:
			LOG_ERROR("unsupported access size: %d", size);
			return ERROR_FAIL;
	}

	switch (riscv_xlen(target)) {
	case 64:
		riscv_program_write_ram(&program, r_addr + 4, (uint64_t)address >> 32);
	case 32:
		riscv_program_write_ram(&program, r_addr, address);
		break;
	default:
		LOG_ERROR("unknown XLEN %d", riscv_xlen(target));
		return ERROR_FAIL;
	}
	riscv_program_write_ram(&program, r_data, value);

	LOG_DEBUG("M[0x%08lx] writes 0x%08lx", (long)address, (long)value);

	if (riscv_program_exec(&program, target) != ERROR_OK) {
		uint32_t acs = dmi_read(target, DMI_ABSTRACTCS);
		LOG_ERROR("failed to execute program, abstractcs=0x%08x", acs);
		riscv013_clear_abstract_error(target);
		riscv_set_register(target, GDB_REGNO_S0, s0);
		riscv_set_register(target, GDB_REGNO_S1, s1);
		LOG_ERROR("  exiting with ERROR_FAIL");
		return ERROR_FAIL;
	}

	/* The rest of this program is designed to be fast so it reads various
	 * DMI registers directly. */
	int d_data = (r_data - riscv_debug_buffer_addr(target)) / 4;
	int d_addr = (r_addr - riscv_debug_buffer_addr(target)) / 4;

	riscv013_set_autoexec(target, d_data, 1);

	/* Copying memory might fail because we're going too quickly, in which
	 * case we need to back off a bit and try again.  There's two
	 * termination conditions to this loop: a non-BUSY error message, or
	 * the data was all copied. */
	riscv_addr_t cur_addr = 0xbadbeef;
	riscv_addr_t fin_addr = address + (count * size);
	LOG_DEBUG("writing until final address 0x%016lx", fin_addr);
	while ((cur_addr = riscv_read_debug_buffer_x(target, d_addr)) < fin_addr) {
		LOG_DEBUG("transferring burst starting at address 0x%016lx", cur_addr);
		riscv_addr_t start = (cur_addr - address) / size;
                assert (cur_addr > address);
                struct riscv_batch *batch = riscv_batch_alloc(
			target,
			32,
			info->dmi_busy_delay + info->ac_busy_delay);

		for (riscv_addr_t i = start; i < count; ++i) {
			riscv_addr_t offset = size*i;
			riscv_addr_t t_addr = address + offset;
			const uint8_t *t_buffer = buffer + offset;

			switch (size) {
				case 1:
					value = t_buffer[0];
					break;
				case 2:
					value = t_buffer[0]
						| ((uint32_t) t_buffer[1] << 8);
					break;
				case 4:
					value = t_buffer[0]
						| ((uint32_t) t_buffer[1] << 8)
						| ((uint32_t) t_buffer[2] << 16)
						| ((uint32_t) t_buffer[3] << 24);
					break;
				default:
					LOG_ERROR("unsupported access size: %d", size);
					return ERROR_FAIL;
			}

			LOG_DEBUG("M[0x%08lx] writes 0x%08lx", (long)t_addr, (long)value);

			riscv_batch_add_dmi_write(
				batch,
				riscv013_debug_buffer_register(target, r_data),
				value);
			if (riscv_batch_full(batch))
				break;
		}

		riscv_batch_run(batch);
		riscv_batch_free(batch);

                // Note that if the scan resulted in a Busy DMI response, it
                // is this read to abstractcs that will cause the dmi_busy_delay
                // to be incremented if necessary. The loop condition above
                // catches the case where no writes went through at all.

		uint32_t abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		while (get_field(abstractcs, DMI_ABSTRACTCS_BUSY))
			abstractcs = dmi_read(target, DMI_ABSTRACTCS);
		switch (get_field(abstractcs, DMI_ABSTRACTCS_CMDERR)) {
		case CMDERR_NONE:
			LOG_DEBUG("successful (partial?) memory write");
			break;
		case CMDERR_BUSY:
			LOG_DEBUG("memory write resulted in busy response");
			riscv013_clear_abstract_error(target);
			increase_ac_busy_delay(target);
			break;
		default:
			LOG_ERROR("error when writing memory, abstractcs=0x%08lx", (long)abstractcs);
			riscv013_set_autoexec(target, d_data, 0);
			riscv013_clear_abstract_error(target);
			riscv_set_register(target, GDB_REGNO_S0, s0);
			riscv_set_register(target, GDB_REGNO_S1, s1);
			return ERROR_FAIL;
		}
	}

	riscv013_set_autoexec(target, d_data, 0);
	riscv_set_register(target, GDB_REGNO_S0, s0);
	riscv_set_register(target, GDB_REGNO_S1, s1);
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

	.poll = &riscv_openocd_poll,
	.halt = &riscv_openocd_halt,
	.resume = &riscv_openocd_resume,
	.step = &riscv_openocd_step,

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

/*** 0.13-specific implementations of various RISC-V hepler functions. ***/
static riscv_reg_t riscv013_get_register(struct target *target, int hid, int rid)
{
	LOG_DEBUG("reading register 0x%08x on hart %d", rid, hid);

	riscv_set_current_hartid(target, hid);

	uint64_t out;
	riscv013_info_t *info = get_info(target);

	if (rid <= GDB_REGNO_XPR31) {
		register_read_direct(target, &out, rid);
	} else if (rid == GDB_REGNO_PC) {
		register_read_direct(target, &out, GDB_REGNO_DPC);
		LOG_DEBUG("read PC from DPC: 0x%016lx", out);
	} else if (rid == GDB_REGNO_PRIV) {
		uint64_t dcsr;
		register_read_direct(target, &dcsr, CSR_DCSR);
		buf_set_u64((unsigned char *)&out, 0, 8, get_field(dcsr, CSR_DCSR_PRV));
	} else {
		int result = register_read_direct(target, &out, rid);
		if (result != ERROR_OK) {
			LOG_ERROR("Unable to read register %d", rid);
			out = -1;
		}

		if (rid == GDB_REGNO_MSTATUS)
			info->mstatus_actual = out;
	}

	return out;
}

static void riscv013_set_register(struct target *target, int hid, int rid, uint64_t value)
{
	LOG_DEBUG("writing register 0x%08x on hart %d", rid, hid);

	riscv_set_current_hartid(target, hid);

	if (rid <= GDB_REGNO_XPR31) {
		register_write_direct(target, rid, value);
	} else if (rid == GDB_REGNO_PC) {
		LOG_DEBUG("writing PC to DPC: 0x%016lx", value);
		register_write_direct(target, GDB_REGNO_DPC, value);
		uint64_t actual_value;
		register_read_direct(target, &actual_value, GDB_REGNO_DPC);
		LOG_DEBUG("  actual DPC written: 0x%016lx", actual_value);
		assert(value == actual_value);
	} else if (rid == GDB_REGNO_PRIV) {
		uint64_t dcsr;
		register_read_direct(target, &dcsr, CSR_DCSR);
		dcsr = set_field(dcsr, CSR_DCSR_PRV, value);
		register_write_direct(target, CSR_DCSR, dcsr);
	} else {
		register_write_direct(target, rid, value);
	}
}

static void riscv013_select_current_hart(struct target *target)
{
	RISCV_INFO(r);

	uint64_t dmcontrol = dmi_read(target, DMI_DMCONTROL);
	dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_HARTSEL, r->current_hartid);
	dmi_write(target, DMI_DMCONTROL, dmcontrol);
}

static void riscv013_halt_current_hart(struct target *target)
{
	RISCV_INFO(r);
	LOG_DEBUG("halting hart %d", r->current_hartid);
	assert(!riscv_is_halted(target));

	/* Issue the halt command, and then wait for the current hart to halt. */
	uint32_t dmcontrol = dmi_read(target, DMI_DMCONTROL);
	dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_HALTREQ, 1);
	dmi_write(target, DMI_DMCONTROL, dmcontrol);
	for (size_t i = 0; i < 256; ++i)
		if (riscv_is_halted(target))
			break;

	if (!riscv_is_halted(target)) {
		uint32_t dmstatus = dmi_read(target, DMI_DMSTATUS);
		dmcontrol = dmi_read(target, DMI_DMCONTROL);

		LOG_ERROR("unable to halt hart %d", r->current_hartid);
		LOG_ERROR("  dmcontrol=0x%08x", dmcontrol);
		LOG_ERROR("  dmstatus =0x%08x", dmstatus);
		abort();
	}

	dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_HALTREQ, 0);
	dmi_write(target, DMI_DMCONTROL, dmcontrol);
}

static void riscv013_resume_current_hart(struct target *target)
{
	return riscv013_step_or_resume_current_hart(target, false);
}

static void riscv013_step_current_hart(struct target *target)
{
	return riscv013_step_or_resume_current_hart(target, true);
}

static void riscv013_on_resume(struct target *target)
{
	return riscv013_on_step_or_resume(target, false);
}

static void riscv013_on_step(struct target *target)
{
	return riscv013_on_step_or_resume(target, true);
}

static void riscv013_on_halt(struct target *target)
{
}

static bool riscv013_is_halted(struct target *target)
{
	uint32_t dmstatus = dmi_read(target, DMI_DMSTATUS);
	if (get_field(dmstatus, DMI_DMSTATUS_ANYUNAVAIL))
		LOG_ERROR("hart %d is unavailiable", riscv_current_hartid(target));
	if (get_field(dmstatus, DMI_DMSTATUS_ANYNONEXISTENT))
		LOG_ERROR("hart %d doesn't exist", riscv_current_hartid(target));
	return get_field(dmstatus, DMI_DMSTATUS_ALLHALTED);
}

static enum riscv_halt_reason riscv013_halt_reason(struct target *target)
{
	uint64_t dcsr = riscv_get_register(target, GDB_REGNO_DCSR);
	switch (get_field(dcsr, CSR_DCSR_CAUSE)) {
	case CSR_DCSR_CAUSE_SWBP:
	case CSR_DCSR_CAUSE_TRIGGER:
		return RISCV_HALT_BREAKPOINT;
	case CSR_DCSR_CAUSE_STEP:
		return RISCV_HALT_SINGLESTEP;
	case CSR_DCSR_CAUSE_DEBUGINT:
	case CSR_DCSR_CAUSE_HALT:
		return RISCV_HALT_INTERRUPT;
	}

	LOG_ERROR("Unknown DCSR cause field: %x", (int)get_field(dcsr, CSR_DCSR_CAUSE));
	LOG_ERROR("  dcsr=0x%016lx", (long)dcsr);
	abort();
}

void riscv013_debug_buffer_enter(struct target *target, struct riscv_program *program)
{
}

void riscv013_debug_buffer_leave(struct target *target, struct riscv_program *program)
{
}

void riscv013_write_debug_buffer(struct target *target, int index, riscv_insn_t data)
{
	if (index >= riscv013_progbuf_size(target))
		return dmi_write(target, DMI_DATA0 + index - riscv013_progbuf_size(target), data);
	return dmi_write(target, DMI_PROGBUF0 + index, data);
}

riscv_insn_t riscv013_read_debug_buffer(struct target *target, int index)
{
	if (index >= riscv013_progbuf_size(target))
		return dmi_read(target, DMI_DATA0 + index - riscv013_progbuf_size(target));
	return dmi_read(target, DMI_PROGBUF0 + index);
}

int riscv013_execute_debug_buffer(struct target *target)
{
	riscv013_clear_abstract_error(target);

	uint32_t run_program = 0;
	run_program = set_field(run_program, AC_ACCESS_REGISTER_SIZE, 2);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_POSTEXEC, 1);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_TRANSFER, 0);
	run_program = set_field(run_program, AC_ACCESS_REGISTER_REGNO, 0x1000);
	dmi_write(target, DMI_COMMAND, run_program);

	{
		uint32_t dmstatus = 0;
		wait_for_idle(target, &dmstatus);
	}

	uint32_t cs = dmi_read(target, DMI_ABSTRACTCS);
	if (get_field(cs, DMI_ABSTRACTCS_CMDERR) != 0) {
		LOG_ERROR("unable to execute program: (abstractcs=0x%08x)", cs);
		dmi_read(target, DMI_DMSTATUS);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

void riscv013_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d)
{
        RISCV013_INFO(info);
        buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_WRITE);
        buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, d);
        buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, a);
}

void riscv013_fill_dmi_read_u64(struct target *target, char *buf, int a)
{
        RISCV013_INFO(info);
        buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_READ);
        buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, 0);
        buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, a);
}

void riscv013_fill_dmi_nop_u64(struct target *target, char *buf)
{
        RISCV013_INFO(info);
        buf_set_u64((unsigned char *)buf, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH, DMI_OP_NOP);
        buf_set_u64((unsigned char *)buf, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH, 0);
        buf_set_u64((unsigned char *)buf, DTM_DMI_ADDRESS_OFFSET, info->abits, 0);
}

int riscv013_dmi_write_u64_bits(struct target *target)
{
	RISCV013_INFO(info);
	return info->abits + DTM_DMI_DATA_LENGTH + DTM_DMI_OP_LENGTH;
}

void riscv013_reset_current_hart(struct target *target)
{
  select_dmi(target);
  uint32_t control = dmi_read(target, DMI_DMCONTROL);
  control = set_field(control, DMI_DMCONTROL_NDMRESET, 1);
  control = set_field(control, DMI_DMCONTROL_HALTREQ, 1);
  dmi_write(target, DMI_DMCONTROL, control);

  control = set_field(control, DMI_DMCONTROL_NDMRESET, 0);
  dmi_write(target, DMI_DMCONTROL, control);

  while (get_field(dmi_read(target, DMI_DMSTATUS), DMI_DMSTATUS_ALLHALTED) == 0);

  control = set_field(control, DMI_DMCONTROL_HALTREQ, 0);
  dmi_write(target, DMI_DMCONTROL, control);
}

/* Helper Functions. */
static void riscv013_on_step_or_resume(struct target *target, bool step)
{
	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_fence_i(&program);
	if (riscv_program_exec(&program, target) != ERROR_OK)
		LOG_ERROR("Unable to execute fence.i");

	/* We want to twiddle some bits in the debug CSR so debugging works. */
	uint64_t dcsr = riscv_get_register(target, GDB_REGNO_DCSR);
	dcsr = set_field(dcsr, CSR_DCSR_STEP, step);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKM, 1);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKH, 1);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKS, 1);
	dcsr = set_field(dcsr, CSR_DCSR_EBREAKU, 1);
	riscv_set_register(target, GDB_REGNO_DCSR, dcsr);
}

static void riscv013_step_or_resume_current_hart(struct target *target, bool step)
{
	RISCV_INFO(r);
	LOG_DEBUG("resuming hart %d (for step?=%d)", r->current_hartid, step);
	assert(riscv_is_halted(target));

	struct riscv_program program;
	riscv_program_init(&program, target);
	riscv_program_fence_i(&program);
	if (riscv_program_exec(&program, target) != ERROR_OK)
		abort();

	/* Issue the halt command, and then wait for the current hart to halt. */
	uint32_t dmcontrol = dmi_read(target, DMI_DMCONTROL);
	dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_RESUMEREQ, 1);
	dmi_write(target, DMI_DMCONTROL, dmcontrol);

	for (size_t i = 0; i < 256; ++i) {
		usleep(10);
		uint32_t dmstatus = dmi_read(target, DMI_DMSTATUS);
		if (get_field(dmstatus, DMI_DMSTATUS_ALLRESUMEACK) == 0)
			continue;
		if (step && get_field(dmstatus, DMI_DMSTATUS_ALLHALTED) == 0)
			continue;

		dmcontrol = set_field(dmcontrol, DMI_DMCONTROL_RESUMEREQ, 0);
		dmi_write(target, DMI_DMCONTROL, dmcontrol);
		return;
	}

	uint32_t dmstatus = dmi_read(target, DMI_DMSTATUS);
	dmcontrol = dmi_read(target, DMI_DMCONTROL);
	LOG_ERROR("unable to resume hart %d", r->current_hartid);
	LOG_ERROR("  dmcontrol=0x%08x", dmcontrol);
	LOG_ERROR("  dmstatus =0x%08x", dmstatus);

	if (step) {
		LOG_ERROR("  was stepping, halting");
		riscv013_halt_current_hart(target);
		return;
	}

	abort();
}

riscv_addr_t riscv013_progbuf_addr(struct target *target)
{
	RISCV013_INFO(info);
	assert(info->progbuf_addr != -1);
	return info->progbuf_addr;
}

riscv_addr_t riscv013_progbuf_size(struct target *target)
{
	RISCV013_INFO(info);
	if (info->progbuf_size == -1) {
		uint32_t acs = dmi_read(target, DMI_ABSTRACTCS);
		info->progbuf_size = get_field(acs, DMI_ABSTRACTCS_PROGSIZE);
	}
	return info->progbuf_size;
}

riscv_addr_t riscv013_data_size(struct target *target)
{
	RISCV013_INFO(info);
	if (info->data_size == -1) {
		uint32_t acs = dmi_read(target, DMI_HARTINFO);
		info->data_size = get_field(acs, DMI_HARTINFO_DATASIZE);
	}
	return info->data_size;
}

riscv_addr_t riscv013_data_addr(struct target *target)
{
	RISCV013_INFO(info);
	if (info->data_addr == -1) {
		uint32_t acs = dmi_read(target, DMI_HARTINFO);
		info->data_addr = get_field(acs, DMI_HARTINFO_DATAACCESS) ? get_field(acs, DMI_HARTINFO_DATAADDR) : 0;
	}
	return info->data_addr;
}

void riscv013_set_autoexec(struct target *target, int offset, bool enabled)
{
	if (offset >= riscv013_progbuf_size(target)) {
		LOG_DEBUG("setting bit %d in AUTOEXECDATA to %d", offset, enabled);
		uint32_t aa = dmi_read(target, DMI_ABSTRACTAUTO);
		uint32_t aa_aed = get_field(aa, DMI_ABSTRACTAUTO_AUTOEXECDATA);
		aa_aed &= ~(1 << (offset - riscv013_progbuf_size(target)));
		aa_aed |= (enabled << (offset - riscv013_progbuf_size(target)));
		aa = set_field(aa, DMI_ABSTRACTAUTO_AUTOEXECDATA, aa_aed);
		dmi_write(target, DMI_ABSTRACTAUTO, aa);
	} else {
		LOG_DEBUG("setting bit %d in AUTOEXECPROGBUF to %d", offset, enabled);
		uint32_t aa = dmi_read(target, DMI_ABSTRACTAUTO);
		uint32_t aa_aed = get_field(aa, DMI_ABSTRACTAUTO_AUTOEXECPROGBUF);
		aa_aed &= ~(1 << offset);
		aa_aed |= (enabled << offset);
		aa = set_field(aa, DMI_ABSTRACTAUTO_AUTOEXECPROGBUF, aa_aed);
		dmi_write(target, DMI_ABSTRACTAUTO, aa);
	}
}

int riscv013_debug_buffer_register(struct target *target, riscv_addr_t addr)
{
	if (addr >= riscv013_data_addr(target))
		return DMI_DATA0 + (addr - riscv013_data_addr(target)) / 4;
	else
		return DMI_PROGBUF0 + (addr - riscv013_progbuf_addr(target)) / 4;
}

void riscv013_clear_abstract_error(struct target *target)
{
	uint32_t acs = dmi_read(target, DMI_ABSTRACTCS);
	dmi_write(target, DMI_ABSTRACTCS, acs);
}
