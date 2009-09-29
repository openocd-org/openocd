/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "etm.h"
#include "etb.h"
#include "image.h"
#include "arm7_9_common.h"
#include "arm_disassembler.h"


/*
 * ARM "Embedded Trace Macrocell" (ETM) support -- direct JTAG access.
 *
 * ETM modules collect instruction and/or data trace information, compress
 * it, and transfer it to a debugging host through either a (buffered) trace
 * port (often a 38-pin Mictor connector) or an Embedded Trace Buffer (ETB).
 *
 * There are several generations of these modules.  Original versions have
 * JTAG access through a dedicated scan chain.  Recent versions have added
 * access via coprocessor instructions, memory addressing, and the ARM Debug
 * Interface v5 (ADIv5); and phased out direct JTAG access.
 *
 * This code supports up to the ETMv1.3 architecture, as seen in ETM9 and
 * most common ARM9 systems.  Note: "CoreSight ETM9" implements ETMv3.2,
 * implying non-JTAG connectivity options.
 *
 * Relevant documentation includes:
 *  ARM DDI 0157G ... ETM9 (r2p2) Technical Reference Manual
 *  ARM DDI 0315B ... CoreSight ETM9 (r0p1) Technical Reference Manual
 *  ARM IHI 0014O ... Embedded Trace Macrocell, Architecture Specification
 */

#define ARRAY_SIZE(x)	((int)(sizeof(x)/sizeof((x)[0])))

enum {
	RO,				/* read/only */
	WO,				/* write/only */
	RW,				/* read/write */
};

struct etm_reg_info {
	uint8_t		addr;
	uint8_t		size;		/* low-N of 32 bits */
	uint8_t		mode;		/* RO, WO, RW */
	uint8_t		bcd_vers;	/* 1.0, 2.0, etc */
	char		*name;
};

/*
 * Registers 0..0x7f are JTAG-addressable using scanchain 6.
 * (Or on some processors, through coprocessor operations.)
 * Newer versions of ETM make some W/O registers R/W, and
 * provide definitions for some previously-unused bits.
 */

/* basic registers that are always there given the right ETM version */
static const struct etm_reg_info etm_core[] = {
	/* NOTE: we "know" ETM_CONFIG is listed first */
	{ ETM_CONFIG, 32, RO, 0x10, "ETM_CONFIG", },

	/* ETM Trace Registers */
	{ ETM_CTRL, 32, RW, 0x10, "ETM_CTRL", },
	{ ETM_TRIG_EVENT, 17, WO, 0x10, "ETM_TRIG_EVENT", },
	{ ETM_ASIC_CTRL,  8, WO, 0x10, "ETM_ASIC_CTRL", },
	{ ETM_STATUS,  3, RO, 0x11, "ETM_STATUS", },
	{ ETM_SYS_CONFIG,  9, RO, 0x12, "ETM_SYS_CONFIG", },

	/* TraceEnable configuration */
	{ ETM_TRACE_RESOURCE_CTRL, 32, WO, 0x12, "ETM_TRACE_RESOURCE_CTRL", },
	{ ETM_TRACE_EN_CTRL2, 16, WO, 0x12, "ETM_TRACE_EN_CTRL2", },
	{ ETM_TRACE_EN_EVENT, 17, WO, 0x10, "ETM_TRACE_EN_EVENT", },
	{ ETM_TRACE_EN_CTRL1, 26, WO, 0x10, "ETM_TRACE_EN_CTRL1", },

	/* ViewData configuration (data trace) */
	{ ETM_VIEWDATA_EVENT, 17, WO, 0x10, "ETM_VIEWDATA_EVENT", },
	{ ETM_VIEWDATA_CTRL1, 32, WO, 0x10, "ETM_VIEWDATA_CTRL1", },
	{ ETM_VIEWDATA_CTRL2, 32, WO, 0x10, "ETM_VIEWDATA_CTRL2", },
	{ ETM_VIEWDATA_CTRL3, 17, WO, 0x10, "ETM_VIEWDATA_CTRL3", },

	/* REVISIT exclude VIEWDATA_CTRL2 when it's not there */

	{ 0x78, 12, WO, 0x20, "ETM_SYNC_FREQ", },
	{ 0x79, 32, RO, 0x20, "ETM_ID", },
};

static const struct etm_reg_info etm_fifofull[] = {
	/* FIFOFULL configuration */
	{ ETM_FIFOFULL_REGION, 25, WO, 0x10, "ETM_FIFOFULL_REGION", },
	{ ETM_FIFOFULL_LEVEL,  8, WO, 0x10, "ETM_FIFOFULL_LEVEL", },
};

static const struct etm_reg_info etm_addr_comp[] = {
	/* Address comparator register pairs */
#define ADDR_COMPARATOR(i) \
		{ ETM_ADDR_COMPARATOR_VALUE + (i), 32, WO, 0x10, \
				"ETM_ADDR_COMPARATOR_VALUE" #i, }, \
		{ ETM_ADDR_ACCESS_TYPE + (i),  7, WO, 0x10, \
				"ETM_ADDR_ACCESS_TYPE" #i, }
	ADDR_COMPARATOR(0),
	ADDR_COMPARATOR(1),
	ADDR_COMPARATOR(2),
	ADDR_COMPARATOR(3),
	ADDR_COMPARATOR(4),
	ADDR_COMPARATOR(5),
	ADDR_COMPARATOR(6),
	ADDR_COMPARATOR(7),

	ADDR_COMPARATOR(8),
	ADDR_COMPARATOR(9),
	ADDR_COMPARATOR(10),
	ADDR_COMPARATOR(11),
	ADDR_COMPARATOR(12),
	ADDR_COMPARATOR(13),
	ADDR_COMPARATOR(14),
	ADDR_COMPARATOR(15),
#undef ADDR_COMPARATOR
};

static const struct etm_reg_info etm_data_comp[] = {
	/* Data Value Comparators (NOTE: odd addresses are reserved) */
#define DATA_COMPARATOR(i) \
		{ ETM_DATA_COMPARATOR_VALUE + 2*(i), 32, WO, 0x10, \
				"ETM_DATA_COMPARATOR_VALUE" #i, }, \
		{ ETM_DATA_COMPARATOR_MASK + 2*(i), 32, WO, 0x10, \
				"ETM_DATA_COMPARATOR_MASK" #i, }
	DATA_COMPARATOR(0),
	DATA_COMPARATOR(1),
	DATA_COMPARATOR(2),
	DATA_COMPARATOR(3),
	DATA_COMPARATOR(4),
	DATA_COMPARATOR(5),
	DATA_COMPARATOR(6),
	DATA_COMPARATOR(7),
#undef DATA_COMPARATOR
};

static const struct etm_reg_info etm_counters[] = {
#define ETM_COUNTER(i) \
		{ ETM_COUNTER_RELOAD_VALUE + (i), 16, WO, 0x10, \
				"ETM_COUNTER_RELOAD_VALUE" #i, }, \
		{ ETM_COUNTER_ENABLE + (i), 18, WO, 0x10, \
				"ETM_COUNTER_ENABLE" #i, }, \
		{ ETM_COUNTER_RELOAD_EVENT + (i), 17, WO, 0x10, \
				"ETM_COUNTER_RELOAD_EVENT" #i, }, \
		{ ETM_COUNTER_VALUE + (i), 16, RO, 0x10, \
				"ETM_COUNTER_VALUE" #i, }
	ETM_COUNTER(0),
	ETM_COUNTER(1),
	ETM_COUNTER(2),
	ETM_COUNTER(3),
#undef ETM_COUNTER
};

static const struct etm_reg_info etm_sequencer[] = {
#define ETM_SEQ(i) \
		{ ETM_SEQUENCER_EVENT + (i), 17, WO, 0x10, \
				"ETM_SEQUENCER_EVENT" #i, }
	ETM_SEQ(0),				/* 1->2 */
	ETM_SEQ(1),				/* 2->1 */
	ETM_SEQ(2),				/* 2->3 */
	ETM_SEQ(3),				/* 3->1 */
	ETM_SEQ(4),				/* 3->2 */
	ETM_SEQ(5),				/* 1->3 */
#undef ETM_SEQ
	/* 0x66 reserved */
	{ ETM_SEQUENCER_STATE,  2, RO, 0x10, "ETM_SEQUENCER_STATE", },
};

static const struct etm_reg_info etm_outputs[] = {
#define ETM_OUTPUT(i) \
		{ ETM_EXTERNAL_OUTPUT + (i), 17, WO, 0x10, \
				"ETM_EXTERNAL_OUTPUT" #i, }

	ETM_OUTPUT(0),
	ETM_OUTPUT(1),
	ETM_OUTPUT(2),
	ETM_OUTPUT(3),
#undef ETM_OUTPUT
};

#if 0
	/* registers from 0x6c..0x7f were added after ETMv1.3 */

	/* Context ID Comparators */
	{ 0x6c, 32, RO, 0x20, "ETM_CONTEXTID_COMPARATOR_VALUE1", }
	{ 0x6d, 32, RO, 0x20, "ETM_CONTEXTID_COMPARATOR_VALUE1", }
	{ 0x6e, 32, RO, 0x20, "ETM_CONTEXTID_COMPARATOR_VALUE1", }
	{ 0x6f, 32, RO, 0x20, "ETM_CONTEXTID_COMPARATOR_MASK", }
#endif

static int etm_reg_arch_type = -1;

static int etm_get_reg(reg_t *reg);
static int etm_read_reg_w_check(reg_t *reg,
		uint8_t* check_value, uint8_t* check_mask);
static int etm_register_user_commands(struct command_context_s *cmd_ctx);
static int etm_set_reg_w_exec(reg_t *reg, uint8_t *buf);
static int etm_write_reg(reg_t *reg, uint32_t value);

static command_t *etm_cmd;


/* Look up register by ID ... most ETM instances only
 * support a subset of the possible registers.
 */
static reg_t *etm_reg_lookup(etm_context_t *etm_ctx, unsigned id)
{
	reg_cache_t *cache = etm_ctx->reg_cache;
	int i;

	for (i = 0; i < cache->num_regs; i++) {
		struct etm_reg_s *reg = cache->reg_list[i].arch_info;

		if (reg->reg_info->addr == id)
			return &cache->reg_list[i];
	}

	/* caller asking for nonexistent register is a bug! */
	/* REVISIT say which of the N targets was involved */
	LOG_ERROR("ETM: register 0x%02x not available", id);
	return NULL;
}

static void etm_reg_add(unsigned bcd_vers, arm_jtag_t *jtag_info,
		reg_cache_t *cache, etm_reg_t *ereg,
		const struct etm_reg_info *r, unsigned nreg)
{
	reg_t *reg = cache->reg_list;

	reg += cache->num_regs;
	ereg += cache->num_regs;

	/* add up to "nreg" registers from "r", if supported by this
	 * version of the ETM, to the specified cache.
	 */
	for (; nreg--; r++) {

		/* this ETM may be too old to have some registers */
		if (r->bcd_vers > bcd_vers)
			continue;

		reg->name = r->name;
		reg->size = r->size;
		reg->value = &ereg->value;
		reg->arch_info = ereg;
		reg->arch_type = etm_reg_arch_type;
		reg++;
		cache->num_regs++;

		ereg->reg_info = r;
		ereg->jtag_info = jtag_info;
		ereg++;
	}
}

reg_cache_t *etm_build_reg_cache(target_t *target,
		arm_jtag_t *jtag_info, etm_context_t *etm_ctx)
{
	reg_cache_t *reg_cache = malloc(sizeof(reg_cache_t));
	reg_t *reg_list = NULL;
	etm_reg_t *arch_info = NULL;
	unsigned bcd_vers, config;

	/* register a register arch-type for etm registers only once */
	if (etm_reg_arch_type == -1)
		etm_reg_arch_type = register_reg_arch_type(etm_get_reg,
				etm_set_reg_w_exec);

	/* the actual registers are kept in two arrays */
	reg_list = calloc(128, sizeof(reg_t));
	arch_info = calloc(128, sizeof(etm_reg_t));

	/* fill in values for the reg cache */
	reg_cache->name = "etm registers";
	reg_cache->next = NULL;
	reg_cache->reg_list = reg_list;
	reg_cache->num_regs = 0;

	/* add ETM_CONFIG, then parse its values to see
	 * which other registers exist in this ETM
	 */
	etm_reg_add(0x10, jtag_info, reg_cache, arch_info,
			etm_core, 1);

	etm_get_reg(reg_list);
	etm_ctx->config = buf_get_u32((void *)&arch_info->value, 0, 32);
	config = etm_ctx->config;

	/* figure ETM version then add base registers */
	if (config & (1 << 31)) {
		bcd_vers = 0x20;
		LOG_WARNING("ETMv2+ support is incomplete");

		/* REVISIT read ID register, distinguish ETMv3.3 etc;
		 * don't presume trace start/stop support is present;
		 * and include any context ID comparator registers.
		 */
	} else {
		switch (config >> 28) {
		case 7:
		case 5:
		case 3:
			bcd_vers = 0x13;
			break;
		case 4:
		case 2:
			bcd_vers = 0x12;
			break;
		case 1:
			bcd_vers = 0x11;
			break;
		case 0:
			bcd_vers = 0x10;
			break;
		default:
			LOG_WARNING("Bad ETMv1 protocol %d", config >> 28);
			free(reg_cache);
			free(reg_list);
			free(arch_info);
			return ERROR_OK;
		}
	}
	etm_ctx->bcd_vers = bcd_vers;
	LOG_INFO("ETM v%d.%d", bcd_vers >> 4, bcd_vers & 0xf);

	etm_reg_add(bcd_vers, jtag_info, reg_cache, arch_info,
			etm_core + 1, ARRAY_SIZE(etm_core) - 1);

	/* address and data comparators; counters; outputs */
	etm_reg_add(bcd_vers, jtag_info, reg_cache, arch_info,
			etm_addr_comp, 4 * (0x0f & (config >> 0)));
	etm_reg_add(bcd_vers, jtag_info, reg_cache, arch_info,
			etm_data_comp, 2 * (0x0f & (config >> 4)));
	etm_reg_add(bcd_vers, jtag_info, reg_cache, arch_info,
			etm_counters, 4 * (0x07 & (config >> 13)));
	etm_reg_add(bcd_vers, jtag_info, reg_cache, arch_info,
			etm_outputs, (0x07 & (config >> 20)));

	/* FIFOFULL presence is optional
	 * REVISIT for ETMv1.2 and later, don't bother adding this
	 * unless ETM_SYS_CONFIG says it's also *supported* ...
	 */
	if (config & (1 << 23))
		etm_reg_add(bcd_vers, jtag_info, reg_cache, arch_info,
				etm_fifofull, ARRAY_SIZE(etm_fifofull));

	/* sequencer is optional (for state-dependant triggering) */
	if (config & (1 << 16))
		etm_reg_add(bcd_vers, jtag_info, reg_cache, arch_info,
				etm_sequencer, ARRAY_SIZE(etm_sequencer));

	/* REVISIT could realloc and likely save half the memory
	 * in the two chunks we allocated...
	 */

	/* the ETM might have an ETB connected */
	if (strcmp(etm_ctx->capture_driver->name, "etb") == 0)
	{
		etb_t *etb = etm_ctx->capture_driver_priv;

		if (!etb)
		{
			LOG_ERROR("etb selected as etm capture driver, but no ETB configured");
			free(reg_cache);
			free(reg_list);
			free(arch_info);
			return ERROR_OK;
		}

		reg_cache->next = etb_build_reg_cache(etb);

		etb->reg_cache = reg_cache->next;
	}


	return reg_cache;
}

static int etm_read_reg(reg_t *reg)
{
	return etm_read_reg_w_check(reg, NULL, NULL);
}

static int etm_store_reg(reg_t *reg)
{
	return etm_write_reg(reg, buf_get_u32(reg->value, 0, reg->size));
}

int etm_setup(target_t *target)
{
	int retval;
	uint32_t etm_ctrl_value;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	etm_context_t *etm_ctx = arm7_9->etm_ctx;
	reg_t *etm_ctrl_reg;

	etm_ctrl_reg = etm_reg_lookup(etm_ctx, ETM_CTRL);
	if (!etm_ctrl_reg)
		return ERROR_OK;

	/* initialize some ETM control register settings */
	etm_get_reg(etm_ctrl_reg);
	etm_ctrl_value = buf_get_u32(etm_ctrl_reg->value, 0, etm_ctrl_reg->size);

	/* clear the ETM powerdown bit (0) */
	etm_ctrl_value &= ~0x1;

	/* configure port width (6:4), mode (17:16) and clocking (13) */
	etm_ctrl_value = (etm_ctrl_value &
		~ETM_PORT_WIDTH_MASK & ~ETM_PORT_MODE_MASK & ~ETM_PORT_CLOCK_MASK)
		| etm_ctx->portmode;

	buf_set_u32(etm_ctrl_reg->value, 0, etm_ctrl_reg->size, etm_ctrl_value);
	etm_store_reg(etm_ctrl_reg);

	if ((retval = jtag_execute_queue()) != ERROR_OK)
		return retval;

	if ((retval = etm_ctx->capture_driver->init(etm_ctx)) != ERROR_OK)
	{
		LOG_ERROR("ETM capture driver initialization failed");
		return retval;
	}
	return ERROR_OK;
}

static int etm_get_reg(reg_t *reg)
{
	int retval;

	if ((retval = etm_read_reg(reg)) != ERROR_OK)
	{
		LOG_ERROR("BUG: error scheduling etm register read");
		return retval;
	}

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("register read failed");
		return retval;
	}

	return ERROR_OK;
}

static int etm_read_reg_w_check(reg_t *reg,
		uint8_t* check_value, uint8_t* check_mask)
{
	etm_reg_t *etm_reg = reg->arch_info;
	const struct etm_reg_info *r = etm_reg->reg_info;
	uint8_t reg_addr = r->addr & 0x7f;
	scan_field_t fields[3];

	if (etm_reg->reg_info->mode == WO) {
		LOG_ERROR("BUG: can't read write-only register %s", r->name);
		return ERROR_INVALID_ARGUMENTS;
	}

	LOG_DEBUG("%s (%u)", r->name, reg_addr);

	jtag_set_end_state(TAP_IDLE);
	arm_jtag_scann(etm_reg->jtag_info, 0x6);
	arm_jtag_set_instr(etm_reg->jtag_info, etm_reg->jtag_info->intest_instr, NULL);

	fields[0].tap = etm_reg->jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = reg->value;
	fields[0].in_value = NULL;
	fields[0].check_value = NULL;
	fields[0].check_mask = NULL;

	fields[1].tap = etm_reg->jtag_info->tap;
	fields[1].num_bits = 7;
	fields[1].out_value = malloc(1);
	buf_set_u32(fields[1].out_value, 0, 7, reg_addr);
	fields[1].in_value = NULL;
	fields[1].check_value = NULL;
	fields[1].check_mask = NULL;

	fields[2].tap = etm_reg->jtag_info->tap;
	fields[2].num_bits = 1;
	fields[2].out_value = malloc(1);
	buf_set_u32(fields[2].out_value, 0, 1, 0);
	fields[2].in_value = NULL;
	fields[2].check_value = NULL;
	fields[2].check_mask = NULL;

	jtag_add_dr_scan(3, fields, jtag_get_end_state());

	fields[0].in_value = reg->value;
	fields[0].check_value = check_value;
	fields[0].check_mask = check_mask;

	jtag_add_dr_scan_check(3, fields, jtag_get_end_state());

	free(fields[1].out_value);
	free(fields[2].out_value);

	return ERROR_OK;
}

static int etm_set_reg(reg_t *reg, uint32_t value)
{
	int retval;

	if ((retval = etm_write_reg(reg, value)) != ERROR_OK)
	{
		LOG_ERROR("BUG: error scheduling etm register write");
		return retval;
	}

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->valid = 1;
	reg->dirty = 0;

	return ERROR_OK;
}

static int etm_set_reg_w_exec(reg_t *reg, uint8_t *buf)
{
	int retval;

	etm_set_reg(reg, buf_get_u32(buf, 0, reg->size));

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("register write failed");
		return retval;
	}
	return ERROR_OK;
}

static int etm_write_reg(reg_t *reg, uint32_t value)
{
	etm_reg_t *etm_reg = reg->arch_info;
	const struct etm_reg_info *r = etm_reg->reg_info;
	uint8_t reg_addr = r->addr & 0x7f;
	scan_field_t fields[3];

	if (etm_reg->reg_info->mode == RO) {
		LOG_ERROR("BUG: can't write read--only register %s", r->name);
		return ERROR_INVALID_ARGUMENTS;
	}

	LOG_DEBUG("%s (%u): 0x%8.8" PRIx32 "", r->name, reg_addr, value);

	jtag_set_end_state(TAP_IDLE);
	arm_jtag_scann(etm_reg->jtag_info, 0x6);
	arm_jtag_set_instr(etm_reg->jtag_info, etm_reg->jtag_info->intest_instr, NULL);

	fields[0].tap = etm_reg->jtag_info->tap;
	fields[0].num_bits = 32;
	uint8_t tmp1[4];
	fields[0].out_value = tmp1;
	buf_set_u32(fields[0].out_value, 0, 32, value);
	fields[0].in_value = NULL;

	fields[1].tap = etm_reg->jtag_info->tap;
	fields[1].num_bits = 7;
	uint8_t tmp2;
	fields[1].out_value = &tmp2;
	buf_set_u32(fields[1].out_value, 0, 7, reg_addr);
	fields[1].in_value = NULL;

	fields[2].tap = etm_reg->jtag_info->tap;
	fields[2].num_bits = 1;
	uint8_t tmp3;
	fields[2].out_value = &tmp3;
	buf_set_u32(fields[2].out_value, 0, 1, 1);
	fields[2].in_value = NULL;

	jtag_add_dr_scan(3, fields, jtag_get_end_state());

	return ERROR_OK;
}


/* ETM trace analysis functionality
 *
 */
extern etm_capture_driver_t etm_dummy_capture_driver;
#if BUILD_OOCD_TRACE == 1
extern etm_capture_driver_t oocd_trace_capture_driver;
#endif

static etm_capture_driver_t *etm_capture_drivers[] =
{
	&etb_capture_driver,
	&etm_dummy_capture_driver,
#if BUILD_OOCD_TRACE == 1
	&oocd_trace_capture_driver,
#endif
	NULL
};

static int etm_read_instruction(etm_context_t *ctx, arm_instruction_t *instruction)
{
	int i;
	int section = -1;
	uint32_t size_read;
	uint32_t opcode;
	int retval;

	if (!ctx->image)
		return ERROR_TRACE_IMAGE_UNAVAILABLE;

	/* search for the section the current instruction belongs to */
	for (i = 0; i < ctx->image->num_sections; i++)
	{
		if ((ctx->image->sections[i].base_address <= ctx->current_pc) &&
			(ctx->image->sections[i].base_address + ctx->image->sections[i].size > ctx->current_pc))
		{
			section = i;
			break;
		}
	}

	if (section == -1)
	{
		/* current instruction couldn't be found in the image */
		return ERROR_TRACE_INSTRUCTION_UNAVAILABLE;
	}

	if (ctx->core_state == ARMV4_5_STATE_ARM)
	{
		uint8_t buf[4];
		if ((retval = image_read_section(ctx->image, section,
			ctx->current_pc - ctx->image->sections[section].base_address,
			4, buf, &size_read)) != ERROR_OK)
		{
			LOG_ERROR("error while reading instruction: %i", retval);
			return ERROR_TRACE_INSTRUCTION_UNAVAILABLE;
		}
		opcode = target_buffer_get_u32(ctx->target, buf);
		arm_evaluate_opcode(opcode, ctx->current_pc, instruction);
	}
	else if (ctx->core_state == ARMV4_5_STATE_THUMB)
	{
		uint8_t buf[2];
		if ((retval = image_read_section(ctx->image, section,
			ctx->current_pc - ctx->image->sections[section].base_address,
			2, buf, &size_read)) != ERROR_OK)
		{
			LOG_ERROR("error while reading instruction: %i", retval);
			return ERROR_TRACE_INSTRUCTION_UNAVAILABLE;
		}
		opcode = target_buffer_get_u16(ctx->target, buf);
		thumb_evaluate_opcode(opcode, ctx->current_pc, instruction);
	}
	else if (ctx->core_state == ARMV4_5_STATE_JAZELLE)
	{
		LOG_ERROR("BUG: tracing of jazelle code not supported");
		exit(-1);
	}
	else
	{
		LOG_ERROR("BUG: unknown core state encountered");
		exit(-1);
	}

	return ERROR_OK;
}

static int etmv1_next_packet(etm_context_t *ctx, uint8_t *packet, int apo)
{
	while (ctx->data_index < ctx->trace_depth)
	{
		/* if the caller specified an address packet offset, skip until the
		 * we reach the n-th cycle marked with tracesync */
		if (apo > 0)
		{
			if (ctx->trace_data[ctx->data_index].flags & ETMV1_TRACESYNC_CYCLE)
				apo--;

			if (apo > 0)
			{
				ctx->data_index++;
				ctx->data_half = 0;
			}
			continue;
		}

		/* no tracedata output during a TD cycle
		 * or in a trigger cycle */
		if ((ctx->trace_data[ctx->data_index].pipestat == STAT_TD)
			|| (ctx->trace_data[ctx->data_index].flags & ETMV1_TRIGGER_CYCLE))
		{
			ctx->data_index++;
			ctx->data_half = 0;
			continue;
		}

		if ((ctx->portmode & ETM_PORT_WIDTH_MASK) == ETM_PORT_16BIT)
		{
			if (ctx->data_half == 0)
			{
				*packet = ctx->trace_data[ctx->data_index].packet & 0xff;
				ctx->data_half = 1;
			}
			else
			{
				*packet = (ctx->trace_data[ctx->data_index].packet & 0xff00) >> 8;
				ctx->data_half = 0;
				ctx->data_index++;
			}
		}
		else if ((ctx->portmode & ETM_PORT_WIDTH_MASK) == ETM_PORT_8BIT)
		{
			*packet = ctx->trace_data[ctx->data_index].packet & 0xff;
			ctx->data_index++;
		}
		else
		{
			/* on a 4-bit port, a packet will be output during two consecutive cycles */
			if (ctx->data_index > (ctx->trace_depth - 2))
				return -1;

			*packet = ctx->trace_data[ctx->data_index].packet & 0xf;
			*packet |= (ctx->trace_data[ctx->data_index + 1].packet & 0xf) << 4;
			ctx->data_index += 2;
		}

		return 0;
	}

	return -1;
}

static int etmv1_branch_address(etm_context_t *ctx)
{
	int retval;
	uint8_t packet;
	int shift = 0;
	int apo;
	uint32_t i;

	/* quit analysis if less than two cycles are left in the trace
	 * because we can't extract the APO */
	if (ctx->data_index > (ctx->trace_depth - 2))
		return -1;

	/* a BE could be output during an APO cycle, skip the current
	 * and continue with the new one */
	if (ctx->trace_data[ctx->pipe_index + 1].pipestat & 0x4)
		return 1;
	if (ctx->trace_data[ctx->pipe_index + 2].pipestat & 0x4)
		return 2;

	/* address packet offset encoded in the next two cycles' pipestat bits */
	apo = ctx->trace_data[ctx->pipe_index + 1].pipestat & 0x3;
	apo |= (ctx->trace_data[ctx->pipe_index + 2].pipestat & 0x3) << 2;

	/* count number of tracesync cycles between current pipe_index and data_index
	 * i.e. the number of tracesyncs that data_index already passed by
	 * to subtract them from the APO */
	for (i = ctx->pipe_index; i < ctx->data_index; i++)
	{
		if (ctx->trace_data[ctx->pipe_index + 1].pipestat & ETMV1_TRACESYNC_CYCLE)
			apo--;
	}

	/* extract up to four 7-bit packets */
	do {
		if ((retval = etmv1_next_packet(ctx, &packet, (shift == 0) ? apo + 1 : 0)) != 0)
			return -1;
		ctx->last_branch &= ~(0x7f << shift);
		ctx->last_branch |= (packet & 0x7f) << shift;
		shift += 7;
	} while ((packet & 0x80) && (shift < 28));

	/* one last packet holding 4 bits of the address, plus the branch reason code */
	if ((shift == 28) && (packet & 0x80))
	{
		if ((retval = etmv1_next_packet(ctx, &packet, 0)) != 0)
			return -1;
		ctx->last_branch &= 0x0fffffff;
		ctx->last_branch |= (packet & 0x0f) << 28;
		ctx->last_branch_reason = (packet & 0x70) >> 4;
		shift += 4;
	}
	else
	{
		ctx->last_branch_reason = 0;
	}

	if (shift == 32)
	{
		ctx->pc_ok = 1;
	}

	/* if a full address was output, we might have branched into Jazelle state */
	if ((shift == 32) && (packet & 0x80))
	{
		ctx->core_state = ARMV4_5_STATE_JAZELLE;
	}
	else
	{
		/* if we didn't branch into Jazelle state, the current processor state is
		 * encoded in bit 0 of the branch target address */
		if (ctx->last_branch & 0x1)
		{
			ctx->core_state = ARMV4_5_STATE_THUMB;
			ctx->last_branch &= ~0x1;
		}
		else
		{
			ctx->core_state = ARMV4_5_STATE_ARM;
			ctx->last_branch &= ~0x3;
		}
	}

	return 0;
}

static int etmv1_data(etm_context_t *ctx, int size, uint32_t *data)
{
	int j;
	uint8_t buf[4];
	int retval;

	for (j = 0; j < size; j++)
	{
		if ((retval = etmv1_next_packet(ctx, &buf[j], 0)) != 0)
			return -1;
	}

	if (size == 8)
	{
		LOG_ERROR("TODO: add support for 64-bit values");
		return -1;
	}
	else if (size == 4)
		*data = target_buffer_get_u32(ctx->target, buf);
	else if (size == 2)
		*data = target_buffer_get_u16(ctx->target, buf);
	else if (size == 1)
		*data = buf[0];
	else
		return -1;

	return 0;
}

static int etmv1_analyze_trace(etm_context_t *ctx, struct command_context_s *cmd_ctx)
{
	int retval;
	arm_instruction_t instruction;

	/* read the trace data if it wasn't read already */
	if (ctx->trace_depth == 0)
		ctx->capture_driver->read_trace(ctx);

	/* start at the beginning of the captured trace */
	ctx->pipe_index = 0;
	ctx->data_index = 0;
	ctx->data_half = 0;

	/* neither the PC nor the data pointer are valid */
	ctx->pc_ok = 0;
	ctx->ptr_ok = 0;

	while (ctx->pipe_index < ctx->trace_depth)
	{
		uint8_t pipestat = ctx->trace_data[ctx->pipe_index].pipestat;
		uint32_t next_pc = ctx->current_pc;
		uint32_t old_data_index = ctx->data_index;
		uint32_t old_data_half = ctx->data_half;
		uint32_t old_index = ctx->pipe_index;
		uint32_t last_instruction = ctx->last_instruction;
		uint32_t cycles = 0;
		int current_pc_ok = ctx->pc_ok;

		if (ctx->trace_data[ctx->pipe_index].flags & ETMV1_TRIGGER_CYCLE)
		{
			command_print(cmd_ctx, "--- trigger ---");
		}

		/* instructions execute in IE/D or BE/D cycles */
		if ((pipestat == STAT_IE) || (pipestat == STAT_ID))
			ctx->last_instruction = ctx->pipe_index;

		/* if we don't have a valid pc skip until we reach an indirect branch */
		if ((!ctx->pc_ok) && (pipestat != STAT_BE))
		{
			ctx->pipe_index++;
			continue;
		}

		/* any indirect branch could have interrupted instruction flow
		 * - the branch reason code could indicate a trace discontinuity
		 * - a branch to the exception vectors indicates an exception
		 */
		if ((pipestat == STAT_BE) || (pipestat == STAT_BD))
		{
			/* backup current data index, to be able to consume the branch address
			 * before examining data address and values
			 */
			old_data_index = ctx->data_index;
			old_data_half = ctx->data_half;

			ctx->last_instruction = ctx->pipe_index;

			if ((retval = etmv1_branch_address(ctx)) != 0)
			{
				/* negative return value from etmv1_branch_address means we ran out of packets,
				 * quit analysing the trace */
				if (retval < 0)
					break;

				/* a positive return values means the current branch was abandoned,
				 * and a new branch was encountered in cycle ctx->pipe_index + retval;
				 */
				LOG_WARNING("abandoned branch encountered, correctnes of analysis uncertain");
				ctx->pipe_index += retval;
				continue;
			}

			/* skip over APO cycles */
			ctx->pipe_index += 2;

			switch (ctx->last_branch_reason)
			{
				case 0x0:	/* normal PC change */
					next_pc = ctx->last_branch;
					break;
				case 0x1:	/* tracing enabled */
					command_print(cmd_ctx, "--- tracing enabled at 0x%8.8" PRIx32 " ---", ctx->last_branch);
					ctx->current_pc = ctx->last_branch;
					ctx->pipe_index++;
					continue;
					break;
				case 0x2:	/* trace restarted after FIFO overflow */
					command_print(cmd_ctx, "--- trace restarted after FIFO overflow at 0x%8.8" PRIx32 " ---", ctx->last_branch);
					ctx->current_pc = ctx->last_branch;
					ctx->pipe_index++;
					continue;
					break;
				case 0x3:	/* exit from debug state */
					command_print(cmd_ctx, "--- exit from debug state at 0x%8.8" PRIx32 " ---", ctx->last_branch);
					ctx->current_pc = ctx->last_branch;
					ctx->pipe_index++;
					continue;
					break;
				case 0x4:	/* periodic synchronization point */
					next_pc = ctx->last_branch;
					/* if we had no valid PC prior to this synchronization point,
					 * we have to move on with the next trace cycle
					 */
					if (!current_pc_ok)
					{
						command_print(cmd_ctx, "--- periodic synchronization point at 0x%8.8" PRIx32 " ---", next_pc);
						ctx->current_pc = next_pc;
						ctx->pipe_index++;
						continue;
					}
					break;
				default:	/* reserved */
					LOG_ERROR("BUG: branch reason code 0x%" PRIx32 " is reserved", ctx->last_branch_reason);
					exit(-1);
					break;
			}

			/* if we got here the branch was a normal PC change
			 * (or a periodic synchronization point, which means the same for that matter)
			 * if we didn't accquire a complete PC continue with the next cycle
			 */
			if (!ctx->pc_ok)
				continue;

			/* indirect branch to the exception vector means an exception occured */
			if ((ctx->last_branch <= 0x20)
				|| ((ctx->last_branch >= 0xffff0000) && (ctx->last_branch <= 0xffff0020)))
			{
				if ((ctx->last_branch & 0xff) == 0x10)
				{
					command_print(cmd_ctx, "data abort");
				}
				else
				{
					command_print(cmd_ctx, "exception vector 0x%2.2" PRIx32 "", ctx->last_branch);
					ctx->current_pc = ctx->last_branch;
					ctx->pipe_index++;
					continue;
				}
			}
		}

		/* an instruction was executed (or not, depending on the condition flags)
		 * retrieve it from the image for displaying */
		if (ctx->pc_ok && (pipestat != STAT_WT) && (pipestat != STAT_TD) &&
			!(((pipestat == STAT_BE) || (pipestat == STAT_BD)) &&
				((ctx->last_branch_reason != 0x0) && (ctx->last_branch_reason != 0x4))))
		{
			if ((retval = etm_read_instruction(ctx, &instruction)) != ERROR_OK)
			{
				/* can't continue tracing with no image available */
				if (retval == ERROR_TRACE_IMAGE_UNAVAILABLE)
				{
					return retval;
				}
				else if (retval == ERROR_TRACE_INSTRUCTION_UNAVAILABLE)
				{
					/* TODO: handle incomplete images
					 * for now we just quit the analsysis*/
					return retval;
				}
			}

			cycles = old_index - last_instruction;
		}

		if ((pipestat == STAT_ID) || (pipestat == STAT_BD))
		{
			uint32_t new_data_index = ctx->data_index;
			uint32_t new_data_half = ctx->data_half;

			/* in case of a branch with data, the branch target address was consumed before
			 * we temporarily go back to the saved data index */
			if (pipestat == STAT_BD)
			{
				ctx->data_index = old_data_index;
				ctx->data_half = old_data_half;
			}

			if (ctx->tracemode & ETMV1_TRACE_ADDR)
			{
				uint8_t packet;
				int shift = 0;

				do {
					if ((retval = etmv1_next_packet(ctx, &packet, 0)) != 0)
						return ERROR_ETM_ANALYSIS_FAILED;
					ctx->last_ptr &= ~(0x7f << shift);
					ctx->last_ptr |= (packet & 0x7f) << shift;
					shift += 7;
				} while ((packet & 0x80) && (shift < 32));

				if (shift >= 32)
					ctx->ptr_ok = 1;

				if (ctx->ptr_ok)
				{
					command_print(cmd_ctx, "address: 0x%8.8" PRIx32 "", ctx->last_ptr);
				}
			}

			if (ctx->tracemode & ETMV1_TRACE_DATA)
			{
				if ((instruction.type == ARM_LDM) || (instruction.type == ARM_STM))
				{
					int i;
					for (i = 0; i < 16; i++)
					{
						if (instruction.info.load_store_multiple.register_list & (1 << i))
						{
							uint32_t data;
							if (etmv1_data(ctx, 4, &data) != 0)
								return ERROR_ETM_ANALYSIS_FAILED;
							command_print(cmd_ctx, "data: 0x%8.8" PRIx32 "", data);
						}
					}
				}
				else if ((instruction.type >= ARM_LDR) && (instruction.type <= ARM_STRH))
				{
					uint32_t data;
					if (etmv1_data(ctx, arm_access_size(&instruction), &data) != 0)
						return ERROR_ETM_ANALYSIS_FAILED;
					command_print(cmd_ctx, "data: 0x%8.8" PRIx32 "", data);
				}
			}

			/* restore data index after consuming BD address and data */
			if (pipestat == STAT_BD)
			{
				ctx->data_index = new_data_index;
				ctx->data_half = new_data_half;
			}
		}

		/* adjust PC */
		if ((pipestat == STAT_IE) || (pipestat == STAT_ID))
		{
			if (((instruction.type == ARM_B) ||
			     (instruction.type == ARM_BL) ||
			     (instruction.type == ARM_BLX)) &&
			    (instruction.info.b_bl_bx_blx.target_address != 0xffffffff))
			{
				next_pc = instruction.info.b_bl_bx_blx.target_address;
			}
			else
			{
				next_pc += (ctx->core_state == ARMV4_5_STATE_ARM) ? 4 : 2;
			}
		}
		else if (pipestat == STAT_IN)
		{
			next_pc += (ctx->core_state == ARMV4_5_STATE_ARM) ? 4 : 2;
		}

		if ((pipestat != STAT_TD) && (pipestat != STAT_WT))
		{
			char cycles_text[32] = "";

			/* if the trace was captured with cycle accurate tracing enabled,
			 * output the number of cycles since the last executed instruction
			 */
			if (ctx->tracemode & ETMV1_CYCLE_ACCURATE)
			{
				snprintf(cycles_text, 32, " (%i %s)",
					 (int)cycles,
					(cycles == 1) ? "cycle" : "cycles");
			}

			command_print(cmd_ctx, "%s%s%s",
				instruction.text,
				(pipestat == STAT_IN) ? " (not executed)" : "",
				cycles_text);

			ctx->current_pc = next_pc;

			/* packets for an instruction don't start on or before the preceding
			 * functional pipestat (i.e. other than WT or TD)
			 */
			if (ctx->data_index <= ctx->pipe_index)
			{
				ctx->data_index = ctx->pipe_index + 1;
				ctx->data_half = 0;
			}
		}

		ctx->pipe_index += 1;
	}

	return ERROR_OK;
}

static int handle_etm_tracemode_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etmv1_tracemode_t tracemode;

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!arm7_9->etm_ctx)
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	tracemode = arm7_9->etm_ctx->tracemode;

	if (argc == 4)
	{
		if (strcmp(args[0], "none") == 0)
		{
			tracemode = ETMV1_TRACE_NONE;
		}
		else if (strcmp(args[0], "data") == 0)
		{
			tracemode = ETMV1_TRACE_DATA;
		}
		else if (strcmp(args[0], "address") == 0)
		{
			tracemode = ETMV1_TRACE_ADDR;
		}
		else if (strcmp(args[0], "all") == 0)
		{
			tracemode = ETMV1_TRACE_DATA | ETMV1_TRACE_ADDR;
		}
		else
		{
			command_print(cmd_ctx, "invalid option '%s'", args[0]);
			return ERROR_OK;
		}

		switch (strtol(args[1], NULL, 0))
		{
			case 0:
				tracemode |= ETMV1_CONTEXTID_NONE;
				break;
			case 8:
				tracemode |= ETMV1_CONTEXTID_8;
				break;
			case 16:
				tracemode |= ETMV1_CONTEXTID_16;
				break;
			case 32:
				tracemode |= ETMV1_CONTEXTID_32;
				break;
			default:
				command_print(cmd_ctx, "invalid option '%s'", args[1]);
				return ERROR_OK;
		}

		if (strcmp(args[2], "enable") == 0)
		{
			tracemode |= ETMV1_CYCLE_ACCURATE;
		}
		else if (strcmp(args[2], "disable") == 0)
		{
			tracemode |= 0;
		}
		else
		{
			command_print(cmd_ctx, "invalid option '%s'", args[2]);
			return ERROR_OK;
		}

		if (strcmp(args[3], "enable") == 0)
		{
			tracemode |= ETMV1_BRANCH_OUTPUT;
		}
		else if (strcmp(args[3], "disable") == 0)
		{
			tracemode |= 0;
		}
		else
		{
			command_print(cmd_ctx, "invalid option '%s'", args[2]);
			return ERROR_OK;
		}
	}
	else if (argc != 0)
	{
		command_print(cmd_ctx, "usage: configure trace mode <none | data | address | all> <context id bits> <cycle accurate> <branch output>");
		return ERROR_OK;
	}

	command_print(cmd_ctx, "current tracemode configuration:");

	switch (tracemode & ETMV1_TRACE_MASK)
	{
		case ETMV1_TRACE_NONE:
			command_print(cmd_ctx, "data tracing: none");
			break;
		case ETMV1_TRACE_DATA:
			command_print(cmd_ctx, "data tracing: data only");
			break;
		case ETMV1_TRACE_ADDR:
			command_print(cmd_ctx, "data tracing: address only");
			break;
		case ETMV1_TRACE_DATA | ETMV1_TRACE_ADDR:
			command_print(cmd_ctx, "data tracing: address and data");
			break;
	}

	switch (tracemode & ETMV1_CONTEXTID_MASK)
	{
		case ETMV1_CONTEXTID_NONE:
			command_print(cmd_ctx, "contextid tracing: none");
			break;
		case ETMV1_CONTEXTID_8:
			command_print(cmd_ctx, "contextid tracing: 8 bit");
			break;
		case ETMV1_CONTEXTID_16:
			command_print(cmd_ctx, "contextid tracing: 16 bit");
			break;
		case ETMV1_CONTEXTID_32:
			command_print(cmd_ctx, "contextid tracing: 32 bit");
			break;
	}

	if (tracemode & ETMV1_CYCLE_ACCURATE)
	{
		command_print(cmd_ctx, "cycle-accurate tracing enabled");
	}
	else
	{
		command_print(cmd_ctx, "cycle-accurate tracing disabled");
	}

	if (tracemode & ETMV1_BRANCH_OUTPUT)
	{
		command_print(cmd_ctx, "full branch address output enabled");
	}
	else
	{
		command_print(cmd_ctx, "full branch address output disabled");
	}

	/* only update ETM_CTRL register if tracemode changed */
	if (arm7_9->etm_ctx->tracemode != tracemode)
	{
		reg_t *etm_ctrl_reg;

		etm_ctrl_reg = etm_reg_lookup(arm7_9->etm_ctx, ETM_CTRL);
		if (!etm_ctrl_reg)
			return ERROR_OK;

		etm_get_reg(etm_ctrl_reg);

		buf_set_u32(etm_ctrl_reg->value, 2, 2, tracemode & ETMV1_TRACE_MASK);
		buf_set_u32(etm_ctrl_reg->value, 14, 2, (tracemode & ETMV1_CONTEXTID_MASK) >> 4);
		buf_set_u32(etm_ctrl_reg->value, 12, 1, (tracemode & ETMV1_CYCLE_ACCURATE) >> 8);
		buf_set_u32(etm_ctrl_reg->value, 8, 1, (tracemode & ETMV1_BRANCH_OUTPUT) >> 9);
		etm_store_reg(etm_ctrl_reg);

		arm7_9->etm_ctx->tracemode = tracemode;

		/* invalidate old trace data */
		arm7_9->etm_ctx->capture_status = TRACE_IDLE;
		if (arm7_9->etm_ctx->trace_depth > 0)
		{
			free(arm7_9->etm_ctx->trace_data);
			arm7_9->etm_ctx->trace_data = NULL;
		}
		arm7_9->etm_ctx->trace_depth = 0;
	}

	return ERROR_OK;
}

static int handle_etm_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_portmode_t portmode = 0x0;
	etm_context_t *etm_ctx = malloc(sizeof(etm_context_t));
	int i;

	if (argc != 5)
	{
		free(etm_ctx);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	target = get_target(args[0]);
	if (!target)
	{
		LOG_ERROR("target '%s' not defined", args[0]);
		free(etm_ctx);
		return ERROR_FAIL;
	}

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		free(etm_ctx);
		return ERROR_FAIL;
	}

	switch (strtoul(args[1], NULL, 0))
	{
		case 4:
			portmode |= ETM_PORT_4BIT;
			break;
		case 8:
			portmode |= ETM_PORT_8BIT;
			break;
		case 16:
			portmode |= ETM_PORT_16BIT;
			break;
		default:
			command_print(cmd_ctx, "unsupported ETM port width '%s', must be 4, 8 or 16", args[1]);
			free(etm_ctx);
			return ERROR_FAIL;
	}

	if (strcmp("normal", args[2]) == 0)
	{
		portmode |= ETM_PORT_NORMAL;
	}
	else if (strcmp("multiplexed", args[2]) == 0)
	{
		portmode |= ETM_PORT_MUXED;
	}
	else if (strcmp("demultiplexed", args[2]) == 0)
	{
		portmode |= ETM_PORT_DEMUXED;
	}
	else
	{
		command_print(cmd_ctx, "unsupported ETM port mode '%s', must be 'normal', 'multiplexed' or 'demultiplexed'", args[2]);
		free(etm_ctx);
		return ERROR_FAIL;
	}

	if (strcmp("half", args[3]) == 0)
	{
		portmode |= ETM_PORT_HALF_CLOCK;
	}
	else if (strcmp("full", args[3]) == 0)
	{
		portmode |= ETM_PORT_FULL_CLOCK;
	}
	else
	{
		command_print(cmd_ctx, "unsupported ETM port clocking '%s', must be 'full' or 'half'", args[3]);
		free(etm_ctx);
		return ERROR_FAIL;
	}

	for (i = 0; etm_capture_drivers[i]; i++)
	{
		if (strcmp(args[4], etm_capture_drivers[i]->name) == 0)
		{
			int retval;
			if ((retval = etm_capture_drivers[i]->register_commands(cmd_ctx)) != ERROR_OK)
			{
				free(etm_ctx);
				return retval;
			}

			etm_ctx->capture_driver = etm_capture_drivers[i];

			break;
		}
	}

	if (!etm_capture_drivers[i])
	{
		/* no supported capture driver found, don't register an ETM */
		free(etm_ctx);
		LOG_ERROR("trace capture driver '%s' not found", args[4]);
		return ERROR_FAIL;
	}

	etm_ctx->target = target;
	etm_ctx->trigger_percent = 50;
	etm_ctx->trace_data = NULL;
	etm_ctx->trace_depth = 0;
	etm_ctx->portmode = portmode;
	etm_ctx->tracemode = 0x0;
	etm_ctx->core_state = ARMV4_5_STATE_ARM;
	etm_ctx->image = NULL;
	etm_ctx->pipe_index = 0;
	etm_ctx->data_index = 0;
	etm_ctx->current_pc = 0x0;
	etm_ctx->pc_ok = 0;
	etm_ctx->last_branch = 0x0;
	etm_ctx->last_branch_reason = 0x0;
	etm_ctx->last_ptr = 0x0;
	etm_ctx->ptr_ok = 0x0;
	etm_ctx->last_instruction = 0;

	arm7_9->etm_ctx = etm_ctx;

	return etm_register_user_commands(cmd_ctx);
}

static int handle_etm_info_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm;
	reg_t *etm_sys_config_reg;

	int max_port_size;

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	etm = arm7_9->etm_ctx;
	if (!etm)
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	command_print(cmd_ctx, "ETM v%d.%d",
			etm->bcd_vers >> 4, etm->bcd_vers & 0xf);
	command_print(cmd_ctx, "pairs of address comparators: %i",
			(etm->config >> 0) & 0x0f);
	command_print(cmd_ctx, "data comparators: %i",
			(etm->config >> 4) & 0x0f);
	command_print(cmd_ctx, "memory map decoders: %i",
			(etm->config >> 8) & 0x1f);
	command_print(cmd_ctx, "number of counters: %i",
			(etm->config >> 13) & 0x07);
	command_print(cmd_ctx, "sequencer %spresent",
			(etm->config & (1 << 16)) ? "" : "not ");
	command_print(cmd_ctx, "number of ext. inputs: %i",
			(etm->config >> 17) & 0x07);
	command_print(cmd_ctx, "number of ext. outputs: %i",
			(etm->config >> 20) & 0x07);
	command_print(cmd_ctx, "FIFO full %spresent",
			(etm->config & (1 << 23)) ? "" : "not ");
	if (etm->bcd_vers < 0x20)
		command_print(cmd_ctx, "protocol version: %i",
				(etm->config >> 28) & 0x07);
	else {
		command_print(cmd_ctx, "trace start/stop %spresent",
				(etm->config & (1 << 26)) ? "" : "not ");
		command_print(cmd_ctx, "number of context comparators: %i",
				(etm->config >> 24) & 0x03);
	}

	/* SYS_CONFIG isn't present before ETMv1.2 */
	etm_sys_config_reg = etm_reg_lookup(etm, ETM_SYS_CONFIG);
	if (!etm_sys_config_reg)
		return ERROR_OK;

	etm_get_reg(etm_sys_config_reg);

	switch (buf_get_u32(etm_sys_config_reg->value, 0, 3))
	{
		case 0:
			max_port_size = 4;
			break;
		case 1:
			max_port_size = 8;
			break;
		case 2:
			max_port_size = 16;
			break;
		default:
			LOG_ERROR("Illegal max_port_size");
			exit(-1);
	}
	command_print(cmd_ctx, "max. port size: %i", max_port_size);

	command_print(cmd_ctx, "half-rate clocking %ssupported",
			(buf_get_u32(etm_sys_config_reg->value, 3, 1) == 1) ? "" : "not ");
	command_print(cmd_ctx, "full-rate clocking %ssupported",
			(buf_get_u32(etm_sys_config_reg->value, 4, 1) == 1) ? "" : "not ");
	command_print(cmd_ctx, "normal trace format %ssupported",
			(buf_get_u32(etm_sys_config_reg->value, 5, 1) == 1) ? "" : "not ");
	command_print(cmd_ctx, "multiplex trace format %ssupported",
			(buf_get_u32(etm_sys_config_reg->value, 6, 1) == 1) ? "" : "not ");
	command_print(cmd_ctx, "demultiplex trace format %ssupported",
			(buf_get_u32(etm_sys_config_reg->value, 7, 1) == 1) ? "" : "not ");
	command_print(cmd_ctx, "FIFO full %ssupported",
			(buf_get_u32(etm_sys_config_reg->value, 8, 1) == 1) ? "" : "not ");

	return ERROR_OK;
}

static int handle_etm_status_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	trace_status_t trace_status;

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!arm7_9->etm_ctx)
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	trace_status = arm7_9->etm_ctx->capture_driver->status(arm7_9->etm_ctx);

	if (trace_status == TRACE_IDLE)
	{
		command_print(cmd_ctx, "tracing is idle");
	}
	else
	{
		static char *completed = " completed";
		static char *running = " is running";
		static char *overflowed = ", trace overflowed";
		static char *triggered = ", trace triggered";

		command_print(cmd_ctx, "trace collection%s%s%s",
			(trace_status & TRACE_RUNNING) ? running : completed,
			(trace_status & TRACE_OVERFLOWED) ? overflowed : "",
			(trace_status & TRACE_TRIGGERED) ? triggered : "");

		if (arm7_9->etm_ctx->trace_depth > 0)
		{
			command_print(cmd_ctx, "%i frames of trace data read", (int)(arm7_9->etm_ctx->trace_depth));
		}
	}

	return ERROR_OK;
}

static int handle_etm_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm_ctx;

	if (argc < 1)
	{
		command_print(cmd_ctx, "usage: etm image <file> [base address] [type]");
		return ERROR_OK;
	}

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!(etm_ctx = arm7_9->etm_ctx))
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	if (etm_ctx->image)
	{
		image_close(etm_ctx->image);
		free(etm_ctx->image);
		command_print(cmd_ctx, "previously loaded image found and closed");
	}

	etm_ctx->image = malloc(sizeof(image_t));
	etm_ctx->image->base_address_set = 0;
	etm_ctx->image->start_address_set = 0;

	/* a base address isn't always necessary, default to 0x0 (i.e. don't relocate) */
	if (argc >= 2)
	{
		etm_ctx->image->base_address_set = 1;
		etm_ctx->image->base_address = strtoul(args[1], NULL, 0);
	}
	else
	{
		etm_ctx->image->base_address_set = 0;
	}

	if (image_open(etm_ctx->image, args[0], (argc >= 3) ? args[2] : NULL) != ERROR_OK)
	{
		free(etm_ctx->image);
		etm_ctx->image = NULL;
		return ERROR_OK;
	}

	return ERROR_OK;
}

static int handle_etm_dump_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	fileio_t file;
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm_ctx;
	uint32_t i;

	if (argc != 1)
	{
		command_print(cmd_ctx, "usage: etm dump <file>");
		return ERROR_OK;
	}

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!(etm_ctx = arm7_9->etm_ctx))
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	if (etm_ctx->capture_driver->status == TRACE_IDLE)
	{
		command_print(cmd_ctx, "trace capture wasn't enabled, no trace data captured");
		return ERROR_OK;
	}

	if (etm_ctx->capture_driver->status(etm_ctx) & TRACE_RUNNING)
	{
		/* TODO: if on-the-fly capture is to be supported, this needs to be changed */
		command_print(cmd_ctx, "trace capture not completed");
		return ERROR_OK;
	}

	/* read the trace data if it wasn't read already */
	if (etm_ctx->trace_depth == 0)
		etm_ctx->capture_driver->read_trace(etm_ctx);

	if (fileio_open(&file, args[0], FILEIO_WRITE, FILEIO_BINARY) != ERROR_OK)
	{
		return ERROR_OK;
	}

	fileio_write_u32(&file, etm_ctx->capture_status);
	fileio_write_u32(&file, etm_ctx->portmode);
	fileio_write_u32(&file, etm_ctx->tracemode);
	fileio_write_u32(&file, etm_ctx->trace_depth);

	for (i = 0; i < etm_ctx->trace_depth; i++)
	{
		fileio_write_u32(&file, etm_ctx->trace_data[i].pipestat);
		fileio_write_u32(&file, etm_ctx->trace_data[i].packet);
		fileio_write_u32(&file, etm_ctx->trace_data[i].flags);
	}

	fileio_close(&file);

	return ERROR_OK;
}

static int handle_etm_load_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	fileio_t file;
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm_ctx;
	uint32_t i;

	if (argc != 1)
	{
		command_print(cmd_ctx, "usage: etm load <file>");
		return ERROR_OK;
	}

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!(etm_ctx = arm7_9->etm_ctx))
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	if (etm_ctx->capture_driver->status(etm_ctx) & TRACE_RUNNING)
	{
		command_print(cmd_ctx, "trace capture running, stop first");
		return ERROR_OK;
	}

	if (fileio_open(&file, args[0], FILEIO_READ, FILEIO_BINARY) != ERROR_OK)
	{
		return ERROR_OK;
	}

	if (file.size % 4)
	{
		command_print(cmd_ctx, "size isn't a multiple of 4, no valid trace data");
		fileio_close(&file);
		return ERROR_OK;
	}

	if (etm_ctx->trace_depth > 0)
	{
		free(etm_ctx->trace_data);
		etm_ctx->trace_data = NULL;
	}

	{
	  uint32_t tmp;
	  fileio_read_u32(&file, &tmp); etm_ctx->capture_status = tmp;
	  fileio_read_u32(&file, &tmp); etm_ctx->portmode = tmp;
	  fileio_read_u32(&file, &tmp); etm_ctx->tracemode = tmp;
	  fileio_read_u32(&file, &etm_ctx->trace_depth);
	}
	etm_ctx->trace_data = malloc(sizeof(etmv1_trace_data_t) * etm_ctx->trace_depth);
	if (etm_ctx->trace_data == NULL)
	{
		command_print(cmd_ctx, "not enough memory to perform operation");
		fileio_close(&file);
		return ERROR_OK;
	}

	for (i = 0; i < etm_ctx->trace_depth; i++)
	{
		uint32_t pipestat, packet, flags;
		fileio_read_u32(&file, &pipestat);
		fileio_read_u32(&file, &packet);
		fileio_read_u32(&file, &flags);
		etm_ctx->trace_data[i].pipestat = pipestat & 0xff;
		etm_ctx->trace_data[i].packet = packet & 0xffff;
		etm_ctx->trace_data[i].flags = flags;
	}

	fileio_close(&file);

	return ERROR_OK;
}

static int handle_etm_trigger_percent_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm_ctx;

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!(etm_ctx = arm7_9->etm_ctx))
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	if (argc > 0)
	{
		uint32_t new_value = strtoul(args[0], NULL, 0);

		if ((new_value < 2) || (new_value > 100))
		{
			command_print(cmd_ctx, "valid settings are 2%% to 100%%");
		}
		else
		{
			etm_ctx->trigger_percent = new_value;
		}
	}

	command_print(cmd_ctx, "%i percent of the tracebuffer reserved for after the trigger", ((int)(etm_ctx->trigger_percent)));

	return ERROR_OK;
}

static int handle_etm_start_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm_ctx;
	reg_t *etm_ctrl_reg;

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!(etm_ctx = arm7_9->etm_ctx))
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	/* invalidate old tracing data */
	arm7_9->etm_ctx->capture_status = TRACE_IDLE;
	if (arm7_9->etm_ctx->trace_depth > 0)
	{
		free(arm7_9->etm_ctx->trace_data);
		arm7_9->etm_ctx->trace_data = NULL;
	}
	arm7_9->etm_ctx->trace_depth = 0;

	etm_ctrl_reg = etm_reg_lookup(etm_ctx, ETM_CTRL);
	if (!etm_ctrl_reg)
		return ERROR_OK;

	etm_get_reg(etm_ctrl_reg);

	/* Clear programming bit (10), set port selection bit (11) */
	buf_set_u32(etm_ctrl_reg->value, 10, 2, 0x2);

	etm_store_reg(etm_ctrl_reg);
	jtag_execute_queue();

	etm_ctx->capture_driver->start_capture(etm_ctx);

	return ERROR_OK;
}

static int handle_etm_stop_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm_ctx;
	reg_t *etm_ctrl_reg;

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!(etm_ctx = arm7_9->etm_ctx))
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	etm_ctrl_reg = etm_reg_lookup(etm_ctx, ETM_CTRL);
	if (!etm_ctrl_reg)
		return ERROR_OK;

	etm_get_reg(etm_ctrl_reg);

	/* Set programming bit (10), clear port selection bit (11) */
	buf_set_u32(etm_ctrl_reg->value, 10, 2, 0x1);

	etm_store_reg(etm_ctrl_reg);
	jtag_execute_queue();

	etm_ctx->capture_driver->stop_capture(etm_ctx);

	return ERROR_OK;
}

static int handle_etm_analyze_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	etm_context_t *etm_ctx;
	int retval;

	target = get_current_target(cmd_ctx);

	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}

	if (!(etm_ctx = arm7_9->etm_ctx))
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}

	if ((retval = etmv1_analyze_trace(etm_ctx, cmd_ctx)) != ERROR_OK)
	{
		switch (retval)
		{
			case ERROR_ETM_ANALYSIS_FAILED:
				command_print(cmd_ctx, "further analysis failed (corrupted trace data or just end of data");
				break;
			case ERROR_TRACE_INSTRUCTION_UNAVAILABLE:
				command_print(cmd_ctx, "no instruction for current address available, analysis aborted");
				break;
			case ERROR_TRACE_IMAGE_UNAVAILABLE:
				command_print(cmd_ctx, "no image available for trace analysis");
				break;
			default:
				command_print(cmd_ctx, "unknown error: %i", retval);
		}
	}

	return ERROR_OK;
}

int etm_register_commands(struct command_context_s *cmd_ctx)
{
	etm_cmd = register_command(cmd_ctx, NULL, "etm", NULL, COMMAND_ANY, "Embedded Trace Macrocell");

	register_command(cmd_ctx, etm_cmd, "config", handle_etm_config_command,
		COMMAND_CONFIG, "etm config <target> <port_width> <port_mode> <clocking> <capture_driver>");

	return ERROR_OK;
}

static int etm_register_user_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, etm_cmd, "tracemode", handle_etm_tracemode_command,
		COMMAND_EXEC, "configure/display trace mode: "
			"<none | data | address | all> "
			"<context_id_bits> <cycle_accurate> <branch_output>");

	register_command(cmd_ctx, etm_cmd, "info", handle_etm_info_command,
		COMMAND_EXEC, "display info about the current target's ETM");

	register_command(cmd_ctx, etm_cmd, "trigger_percent", handle_etm_trigger_percent_command,
		COMMAND_EXEC, "amount (<percent>) of trace buffer to be filled after the trigger occured");
	register_command(cmd_ctx, etm_cmd, "status", handle_etm_status_command,
		COMMAND_EXEC, "display current target's ETM status");
	register_command(cmd_ctx, etm_cmd, "start", handle_etm_start_command,
		COMMAND_EXEC, "start ETM trace collection");
	register_command(cmd_ctx, etm_cmd, "stop", handle_etm_stop_command,
		COMMAND_EXEC, "stop ETM trace collection");

	register_command(cmd_ctx, etm_cmd, "analyze", handle_etm_analyze_command,
		COMMAND_EXEC, "anaylze collected ETM trace");

	register_command(cmd_ctx, etm_cmd, "image", handle_etm_image_command,
		COMMAND_EXEC, "load image from <file> [base address]");

	register_command(cmd_ctx, etm_cmd, "dump", handle_etm_dump_command,
		COMMAND_EXEC, "dump captured trace data <file>");
	register_command(cmd_ctx, etm_cmd, "load", handle_etm_load_command,
		COMMAND_EXEC, "load trace data for analysis <file>");

	return ERROR_OK;
}
