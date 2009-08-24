/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

#include "embeddedice.h"

#define ARRAY_SIZE(x)	((int)(sizeof(x)/sizeof((x)[0])))

#if 0
static bitfield_desc_t embeddedice_comms_ctrl_bitfield_desc[] =
{
	{"R", 1},
	{"W", 1},
	{"reserved", 26},
	{"version", 4}
};
#endif

/*
 * From:  ARM9E-S TRM, DDI 0165, table C-4 (and similar, for other cores)
 */
static const struct {
	char		*name;
	unsigned short	addr;
	unsigned short	width;
} eice_regs[] = {
	[EICE_DBG_CTRL] = {
		.name =		"debug_ctrl",
		.addr =		0,
		/* width is assigned based on EICE version */
	},
	[EICE_DBG_STAT] = {
		.name =		"debug_status",
		.addr =		1,
		/* width is assigned based on EICE version */
	},
	[EICE_COMMS_CTRL] = {
		.name =		"comms_ctrl",
		.addr =		4,
		.width =	6,
	},
	[EICE_COMMS_DATA] = {
		.name =		"comms_data",
		.addr =		5,
		.width =	32,
	},
	[EICE_W0_ADDR_VALUE] = {
		.name =		"watch_0_addr_value",
		.addr =		8,
		.width =	32,
	},
	[EICE_W0_ADDR_MASK] = {
		.name =		"watch_0_addr_mask",
		.addr =		9,
		.width =	32,
	},
	[EICE_W0_DATA_VALUE ] = {
		.name =		"watch_0_data_value",
		.addr =		10,
		.width =	32,
	},
	[EICE_W0_DATA_MASK] = {
		.name =		"watch_0_data_mask",
		.addr =		11,
		.width =	32,
	},
	[EICE_W0_CONTROL_VALUE] = {
		.name =		"watch_0_control_value",
		.addr =		12,
		.width =	9,
	},
	[EICE_W0_CONTROL_MASK] = {
		.name =		"watch_0_control_mask",
		.addr =		13,
		.width =	8,
	},
	[EICE_W1_ADDR_VALUE] = {
		.name =		"watch_1_addr_value",
		.addr =		16,
		.width =	32,
	},
	[EICE_W1_ADDR_MASK] = {
		.name =		"watch_1_addr_mask",
		.addr =		17,
		.width =	32,
	},
	[EICE_W1_DATA_VALUE] = {
		.name =		"watch_1_data_value",
		.addr =		18,
		.width =	32,
	},
	[EICE_W1_DATA_MASK] = {
		.name =		"watch_1_data_mask",
		.addr =		19,
		.width =	32,
	},
	[EICE_W1_CONTROL_VALUE] = {
		.name =		"watch_1_control_value",
		.addr =		20,
		.width =	9,
	},
	[EICE_W1_CONTROL_MASK] = {
		.name =		"watch_1_control_mask",
		.addr =		21,
		.width =	8,
	},
	/* vector_catch isn't always present */
	[EICE_VEC_CATCH] = {
		.name =		"vector_catch",
		.addr =		2,
		.width =	8,
	},
};


static int embeddedice_reg_arch_type = -1;

static int embeddedice_get_reg(reg_t *reg);

reg_cache_t* embeddedice_build_reg_cache(target_t *target, arm7_9_common_t *arm7_9)
{
	int retval;
	reg_cache_t *reg_cache = malloc(sizeof(reg_cache_t));
	reg_t *reg_list = NULL;
	embeddedice_reg_t *arch_info = NULL;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	int num_regs = ARRAY_SIZE(eice_regs);
	int i;
	int eice_version = 0;

	/* register a register arch-type for EmbeddedICE registers only once */
	if (embeddedice_reg_arch_type == -1)
		embeddedice_reg_arch_type = register_reg_arch_type(
				embeddedice_get_reg, embeddedice_set_reg_w_exec);

	/* vector_catch isn't always present */
	if (!arm7_9->has_vector_catch)
		num_regs--;

	/* the actual registers are kept in two arrays */
	reg_list = calloc(num_regs, sizeof(reg_t));
	arch_info = calloc(num_regs, sizeof(embeddedice_reg_t));

	/* fill in values for the reg cache */
	reg_cache->name = "EmbeddedICE registers";
	reg_cache->next = NULL;
	reg_cache->reg_list = reg_list;
	reg_cache->num_regs = num_regs;

	/* set up registers */
	for (i = 0; i < num_regs; i++)
	{
		reg_list[i].name = eice_regs[i].name;
		reg_list[i].size = eice_regs[i].width;
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].bitfield_desc = NULL;
		reg_list[i].num_bitfields = 0;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].arch_info = &arch_info[i];
		reg_list[i].arch_type = embeddedice_reg_arch_type;
		arch_info[i].addr = eice_regs[i].addr;
		arch_info[i].jtag_info = jtag_info;
	}

	/* identify EmbeddedICE version by reading DCC control register */
	embeddedice_read_reg(&reg_list[EICE_COMMS_CTRL]);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		for (i = 0; i < num_regs; i++)
		{
			free(reg_list[i].value);
		}
		free(reg_list);
		free(reg_cache);
		free(arch_info);
		return NULL;
	}

	eice_version = buf_get_u32(reg_list[EICE_COMMS_CTRL].value, 28, 4);
	LOG_DEBUG("Embedded ICE version %d", eice_version);

	switch (eice_version)
	{
		case 1:
			/* ARM7TDMI r3, ARM7TDMI-S r3
			 *
			 * REVISIT docs say ARM7TDMI-S r4 uses version 1 but
			 * that it has 6-bit CTRL and 5-bit STAT... doc bug?
			 * ARM7TDMI r4 docs say EICE v4.
			 */
			reg_list[EICE_DBG_CTRL].size = 3;
			reg_list[EICE_DBG_STAT].size = 5;
			break;
		case 2:
			/* ARM9TDMI */
			reg_list[EICE_DBG_CTRL].size = 4;
			reg_list[EICE_DBG_STAT].size = 5;
			arm7_9->has_single_step = 1;
			break;
		case 3:
			LOG_ERROR("EmbeddedICE v%d handling might be broken",
					eice_version);
			reg_list[EICE_DBG_CTRL].size = 6;
			reg_list[EICE_DBG_STAT].size = 5;
			arm7_9->has_single_step = 1;
			arm7_9->has_monitor_mode = 1;
			break;
		case 4:
			/* ARM7TDMI r4 */
			reg_list[EICE_DBG_CTRL].size = 6;
			reg_list[EICE_DBG_STAT].size = 5;
			arm7_9->has_monitor_mode = 1;
			break;
		case 5:
			/* ARM9E-S rev 1 */
			reg_list[EICE_DBG_CTRL].size = 6;
			reg_list[EICE_DBG_STAT].size = 5;
			arm7_9->has_single_step = 1;
			arm7_9->has_monitor_mode = 1;
			break;
		case 6:
			/* ARM7EJ-S, ARM9E-S rev 2, ARM9EJ-S */
			reg_list[EICE_DBG_CTRL].size = 6;
			reg_list[EICE_DBG_STAT].size = 10;
			/* DBG_STAT has MOE bits */
			arm7_9->has_monitor_mode = 1;
			break;
		case 7:
			LOG_ERROR("EmbeddedICE v%d handling might be broken",
					eice_version);
			reg_list[EICE_DBG_CTRL].size = 6;
			reg_list[EICE_DBG_STAT].size = 5;
			arm7_9->has_monitor_mode = 1;
			break;
		default:
			/*
			 * The Feroceon implementation has the version number
			 * in some unusual bits.  Let feroceon.c validate it
			 * and do the appropriate setup itself.
			 */
			if (strcmp(target_get_name(target), "feroceon") == 0)
				break;
			LOG_ERROR("unknown EmbeddedICE version (comms ctrl: 0x%8.8" PRIx32 ")", buf_get_u32(reg_list[EICE_COMMS_CTRL].value, 0, 32));
	}

	return reg_cache;
}

int embeddedice_setup(target_t *target)
{
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	/* explicitly disable monitor mode */
	if (arm7_9->has_monitor_mode)
	{
		reg_t *dbg_ctrl = &arm7_9->eice_cache->reg_list[EICE_DBG_CTRL];

		embeddedice_read_reg(dbg_ctrl);
		if ((retval = jtag_execute_queue()) != ERROR_OK)
			return retval;
		buf_set_u32(dbg_ctrl->value, 4, 1, 0);
		embeddedice_set_reg_w_exec(dbg_ctrl, dbg_ctrl->value);
	}
	return jtag_execute_queue();
}

static int embeddedice_get_reg(reg_t *reg)
{
	int retval;
	if ((retval = embeddedice_read_reg(reg)) != ERROR_OK)
	{
		LOG_ERROR("BUG: error scheduling EmbeddedICE register read");
		return retval;
	}

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("register read failed");
		return retval;
	}

	return ERROR_OK;
}

int embeddedice_read_reg_w_check(reg_t *reg, uint8_t* check_value, uint8_t* check_mask)
{
	embeddedice_reg_t *ice_reg = reg->arch_info;
	uint8_t reg_addr = ice_reg->addr & 0x1f;
	scan_field_t fields[3];
	uint8_t field1_out[1];
	uint8_t field2_out[1];

	jtag_set_end_state(TAP_IDLE);
	arm_jtag_scann(ice_reg->jtag_info, 0x2);

	arm_jtag_set_instr(ice_reg->jtag_info, ice_reg->jtag_info->intest_instr, NULL);

	fields[0].tap = ice_reg->jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = reg->value;
	fields[0].in_value = NULL;
	fields[0].check_value = NULL;
	fields[0].check_mask = NULL;

	fields[1].tap = ice_reg->jtag_info->tap;
	fields[1].num_bits = 5;
	fields[1].out_value = field1_out;
	buf_set_u32(fields[1].out_value, 0, 5, reg_addr);
	fields[1].in_value = NULL;
	fields[1].check_value = NULL;
	fields[1].check_mask = NULL;

	fields[2].tap = ice_reg->jtag_info->tap;
	fields[2].num_bits = 1;
	fields[2].out_value = field2_out;
	buf_set_u32(fields[2].out_value, 0, 1, 0);
	fields[2].in_value = NULL;
	fields[2].check_value = NULL;
	fields[2].check_mask = NULL;

	jtag_add_dr_scan(3, fields, jtag_get_end_state());

	fields[0].in_value = reg->value;
	fields[0].check_value = check_value;
	fields[0].check_mask = check_mask;

	/* when reading the DCC data register, leaving the address field set to
	 * EICE_COMMS_DATA would read the register twice
	 * reading the control register is safe
	 */
	buf_set_u32(fields[1].out_value, 0, 5, eice_regs[EICE_COMMS_CTRL].addr);

	jtag_add_dr_scan_check(3, fields, jtag_get_end_state());

	return ERROR_OK;
}

/* receive <size> words of 32 bit from the DCC
 * we pretend the target is always going to be fast enough
 * (relative to the JTAG clock), so we don't need to handshake
 */
int embeddedice_receive(arm_jtag_t *jtag_info, uint32_t *data, uint32_t size)
{
	scan_field_t fields[3];
	uint8_t field1_out[1];
	uint8_t field2_out[1];

	jtag_set_end_state(TAP_IDLE);
	arm_jtag_scann(jtag_info, 0x2);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].in_value = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 5;
	fields[1].out_value = field1_out;
	buf_set_u32(fields[1].out_value, 0, 5, eice_regs[EICE_COMMS_DATA].addr);
	fields[1].in_value = NULL;

	fields[2].tap = jtag_info->tap;
	fields[2].num_bits = 1;
	fields[2].out_value = field2_out;
	buf_set_u32(fields[2].out_value, 0, 1, 0);
	fields[2].in_value = NULL;

	jtag_add_dr_scan(3, fields, jtag_get_end_state());

	while (size > 0)
	{
		/* when reading the last item, set the register address to the DCC control reg,
		 * to avoid reading additional data from the DCC data reg
		 */
		if (size == 1)
			buf_set_u32(fields[1].out_value, 0, 5,
					eice_regs[EICE_COMMS_CTRL].addr);

		fields[0].in_value = (uint8_t *)data;
		jtag_add_dr_scan(3, fields, jtag_get_end_state());
		jtag_add_callback(arm_le_to_h_u32, (jtag_callback_data_t)data);

		data++;
		size--;
	}

	return jtag_execute_queue();
}

int embeddedice_read_reg(reg_t *reg)
{
	return embeddedice_read_reg_w_check(reg, NULL, NULL);
}

void embeddedice_set_reg(reg_t *reg, uint32_t value)
{
	embeddedice_write_reg(reg, value);

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->valid = 1;
	reg->dirty = 0;

}

int embeddedice_set_reg_w_exec(reg_t *reg, uint8_t *buf)
{
	int retval;
	embeddedice_set_reg(reg, buf_get_u32(buf, 0, reg->size));

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("register write failed");
		return retval;
	}
	return ERROR_OK;
}

void embeddedice_write_reg(reg_t *reg, uint32_t value)
{
	embeddedice_reg_t *ice_reg = reg->arch_info;

	LOG_DEBUG("%i: 0x%8.8" PRIx32 "", ice_reg->addr, value);

	jtag_set_end_state(TAP_IDLE);
	arm_jtag_scann(ice_reg->jtag_info, 0x2);

	arm_jtag_set_instr(ice_reg->jtag_info, ice_reg->jtag_info->intest_instr, NULL);

	uint8_t reg_addr = ice_reg->addr & 0x1f;
	embeddedice_write_reg_inner(ice_reg->jtag_info->tap, reg_addr, value);

}

void embeddedice_store_reg(reg_t *reg)
{
	embeddedice_write_reg(reg, buf_get_u32(reg->value, 0, reg->size));
}

/* send <size> words of 32 bit to the DCC
 * we pretend the target is always going to be fast enough
 * (relative to the JTAG clock), so we don't need to handshake
 */
int embeddedice_send(arm_jtag_t *jtag_info, uint32_t *data, uint32_t size)
{
	scan_field_t fields[3];
	uint8_t field0_out[4];
	uint8_t field1_out[1];
	uint8_t field2_out[1];

	jtag_set_end_state(TAP_IDLE);
	arm_jtag_scann(jtag_info, 0x2);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = field0_out;
	fields[0].in_value = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 5;
	fields[1].out_value = field1_out;
	buf_set_u32(fields[1].out_value, 0, 5, eice_regs[EICE_COMMS_DATA].addr);
	fields[1].in_value = NULL;

	fields[2].tap = jtag_info->tap;
	fields[2].num_bits = 1;
	fields[2].out_value = field2_out;
	buf_set_u32(fields[2].out_value, 0, 1, 1);

	fields[2].in_value = NULL;

	while (size > 0)
	{
		buf_set_u32(fields[0].out_value, 0, 32, *data);
		jtag_add_dr_scan(3, fields, jtag_get_end_state());

		data++;
		size--;
	}

	/* call to jtag_execute_queue() intentionally omitted */
	return ERROR_OK;
}

/* wait for DCC control register R/W handshake bit to become active
 */
int embeddedice_handshake(arm_jtag_t *jtag_info, int hsbit, uint32_t timeout)
{
	scan_field_t fields[3];
	uint8_t field0_in[4];
	uint8_t field1_out[1];
	uint8_t field2_out[1];
	int retval;
	uint32_t hsact;
	struct timeval lap;
	struct timeval now;

	if (hsbit == EICE_COMM_CTRL_WBIT)
		hsact = 1;
	else if (hsbit == EICE_COMM_CTRL_RBIT)
		hsact = 0;
	else
		return ERROR_INVALID_ARGUMENTS;

	jtag_set_end_state(TAP_IDLE);
	arm_jtag_scann(jtag_info, 0x2);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].in_value = field0_in;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 5;
	fields[1].out_value = field1_out;
	buf_set_u32(fields[1].out_value, 0, 5, eice_regs[EICE_COMMS_DATA].addr);
	fields[1].in_value = NULL;

	fields[2].tap = jtag_info->tap;
	fields[2].num_bits = 1;
	fields[2].out_value = field2_out;
	buf_set_u32(fields[2].out_value, 0, 1, 0);
	fields[2].in_value = NULL;

	jtag_add_dr_scan(3, fields, jtag_get_end_state());
	gettimeofday(&lap, NULL);
	do
	{
		jtag_add_dr_scan(3, fields, jtag_get_end_state());
		if ((retval = jtag_execute_queue()) != ERROR_OK)
			return retval;

		if (buf_get_u32(field0_in, hsbit, 1) == hsact)
			return ERROR_OK;

		gettimeofday(&now, NULL);
	}
	while ((uint32_t)((now.tv_sec-lap.tv_sec)*1000 + (now.tv_usec-lap.tv_usec)/1000) <= timeout);

	return ERROR_TARGET_TIMEOUT;
}

#ifndef HAVE_JTAG_MINIDRIVER_H
/* this is the inner loop of the open loop DCC write of data to target */
void embeddedice_write_dcc(jtag_tap_t *tap, int reg_addr, uint8_t *buffer, int little, int count)
{
	int i;
	for (i = 0; i < count; i++)
	{
		embeddedice_write_reg_inner(tap, reg_addr, fast_target_buffer_get_u32(buffer, little));
		buffer += 4;
	}
}
#else
/* provided by minidriver */
#endif
