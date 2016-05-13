#include <assert.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target.h"
#include "target_type.h"
#include "log.h"
#include "jtag/jtag.h"

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

/*** JTAG registers. ***/

#define DTMINFO			0x10
#define DTMINFO_ADDRBITS	(0xf<<4)
#define DTMINFO_VERSION		(0xf)

#define DBUS			0x11

/*** Debug Bus registers. ***/

#define DMCONTROL		0x10
#define DMCONTROL_HALTNOT	(1<<33)
#define DMCONTROL_INTERRUPT	(1<<32)
#define DMCONTROL_BUSERROR	(7<<19)
#define DMCONTROL_SERIAL	(3<<16)
#define DMCONTROL_AUTOINCREMENT	(1<<15)
#define DMCONTROL_ACCESS	(7<<12)
#define DMCONTROL_HARTID	(0x3ff<<2)
#define DMCONTROL_NDRESET	(1<<1)
#define DMCONTROL_FULLRESET	1

#define DMINFO			0x11
#define DMINFO_ABUSSIZE		(0x7f<<25)
#define DMINFO_SERIALCOUNT	(0xf<<21)
#define DMINFO_ACCESS128	(1<<20)
#define DMINFO_ACCESS64		(1<<19)
#define DMINFO_ACCESS32		(1<<18)
#define DMINFO_ACCESS16		(1<<17)
#define DMINFO_ACCESS8		(1<<16)
#define DMINFO_DRAMSIZE		(0x3f<<10)
#define DMINFO_AUTHENTICATED	(1<<5)
#define DMINFO_AUTHBUSY		(1<<4)
#define DMINFO_AUTHTYPE		(3<<2)
#define DMINFO_VERSION		3

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
	/* Number of words in Debug RAM. */
	unsigned int dramsize;
} riscv_info_t;

static uint64_t dbus_scan(struct target *target, uint16_t address,
		uint64_t data_out, bool read, bool write)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	struct scan_field field;
	uint8_t in[8];
	uint8_t out[8];

	assert(info->addrbits != 0);

	// TODO: max bits is 32?
	field.num_bits = info->addrbits + 35;
	field.out_value = out;
	if (read) {
		field.in_value = in;
	}
	buf_set_u64(out, 0, 34, data_out);
	buf_set_u64(out, 34, info->addrbits, address);
	buf_set_u64(out, info->addrbits + 34, 1, write);

	/* Assume dbus is already selected. */
	jtag_add_dr_scan(target->tap, 1, &field, TAP_DRUPDATE);
	info->dbus_address = address;

	return buf_get_u64(in, 0, 34);
}

static uint64_t dbus_read(struct target *target, uint16_t address, uint16_t next_address)
{
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	if (address != info->dbus_address) {
		dbus_scan(target, address, 0, false, false);
	}
	return dbus_scan(target, next_address, 0, true, false);
}

static uint32_t dtminfo_read(struct target *target)
{
	struct scan_field field;
	uint8_t in[4];
	uint8_t out[4];

	field.num_bits = target->tap->ir_length;
	field.out_value = out;
	field.in_value = NULL;
	buf_set_u32(out, 0, field.num_bits, DTMINFO);
	jtag_add_ir_scan(target->tap, &field, TAP_DRSELECT);

	field.num_bits = 32;
	field.out_value = NULL;
	field.in_value = in;
	jtag_add_dr_scan(target->tap, 1, &field, TAP_DRUPDATE);

	/* Always return to dbus. */
	/* TODO: Can we rely on IR not being messed with between calls into
	 * RISCV code?  Eg. what happens if there are multiple cores and some
	 * other core is accessed? */
	field.num_bits = target->tap->ir_length;
	field.out_value = out;
	field.in_value = NULL;
	buf_set_u32(out, 0, field.num_bits, DBUS);
	jtag_add_ir_scan(target->tap, &field, TAP_DRSELECT);

	return buf_get_u32(field.in_value, 0, 32);
}

static int riscv_init_target(struct command_context *cmd_ctx,
		struct target *target)
{
	target->arch_info = calloc(1, sizeof(riscv_info_t));
	riscv_info_t *info = (riscv_info_t *) target->arch_info;
	info->dbus_address = DBUS_ADDRESS_UNKNOWN;

	return ERROR_OK;
}

int riscv_examine(struct target *target)
{
	if (target_was_examined(target)) {
		return ERROR_OK;
	}

	uint32_t dtminfo = dtminfo_read(target);
	riscv_info_t *info = (riscv_info_t *) target->arch_info;

	info->addrbits = get_field(dtminfo, DTMINFO_ADDRBITS);

	/* TODO: Figure out size of debug RAM, and allocate it. */
	uint64_t dminfo = dbus_read(target, DMINFO, 0);
	info->dramsize = get_field(dminfo, DMINFO_DRAMSIZE) + 1;

	target_set_examined(target);

	return ERROR_OK;
}

struct target_type riscv_target = {
	.name = "riscv",

	.init_target = riscv_init_target,
	.examine = riscv_examine,
};
