/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
 *                                                                         *
 *   Copyright (C) 2009 by David N. Claffey <dnclaffey@gmail.com>          *
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

#include "mips32.h"
#include "mips_ejtag.h"

void mips_ejtag_set_instr(struct mips_ejtag *ejtag_info, int new_instr)
{
	struct jtag_tap *tap;

	tap = ejtag_info->tap;
	assert(tap != NULL);

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != (uint32_t)new_instr) {
		struct scan_field field;
		uint8_t t[4];

		field.num_bits = tap->ir_length;
		field.out_value = t;
		buf_set_u32(t, 0, field.num_bits, new_instr);
		field.in_value = NULL;

		jtag_add_ir_scan(tap, &field, TAP_IDLE);
	}
}

int mips_ejtag_get_idcode(struct mips_ejtag *ejtag_info, uint32_t *idcode)
{
	struct scan_field field;
	uint8_t r[4];

	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_IDCODE);

	field.num_bits = 32;
	field.out_value = NULL;
	field.in_value = r;

	jtag_add_dr_scan(ejtag_info->tap, 1, &field, TAP_IDLE);

	int retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	*idcode = buf_get_u32(field.in_value, 0, 32);

	return ERROR_OK;
}

static int mips_ejtag_get_impcode(struct mips_ejtag *ejtag_info, uint32_t *impcode)
{
	struct scan_field field;
	uint8_t r[4];

	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_IMPCODE);

	field.num_bits = 32;
	field.out_value = NULL;
	field.in_value = r;

	jtag_add_dr_scan(ejtag_info->tap, 1, &field, TAP_IDLE);

	int retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	*impcode = buf_get_u32(field.in_value, 0, 32);

	return ERROR_OK;
}

int mips_ejtag_drscan_32(struct mips_ejtag *ejtag_info, uint32_t *data)
{
	struct jtag_tap *tap;
	tap  = ejtag_info->tap;
	assert(tap != NULL);

	struct scan_field field;
	uint8_t t[4], r[4];
	int retval;

	field.num_bits = 32;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, *data);
	field.in_value = r;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	*data = buf_get_u32(field.in_value, 0, 32);

	keep_alive();

	return ERROR_OK;
}

void mips_ejtag_drscan_32_out(struct mips_ejtag *ejtag_info, uint32_t data)
{
	uint8_t t[4];
	struct jtag_tap *tap;
	tap  = ejtag_info->tap;
	assert(tap != NULL);

	struct scan_field field;

	field.num_bits = 32;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, data);

	field.in_value = NULL;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
}

int mips_ejtag_drscan_8(struct mips_ejtag *ejtag_info, uint32_t *data)
{
	struct jtag_tap *tap;
	tap  = ejtag_info->tap;
	assert(tap != NULL);

	struct scan_field field;
	uint8_t t[4] = {0, 0, 0, 0}, r[4];
	int retval;

	field.num_bits = 8;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, *data);
	field.in_value = r;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("register read failed");
		return retval;
	}

	*data = buf_get_u32(field.in_value, 0, 32);

	return ERROR_OK;
}

void mips_ejtag_drscan_8_out(struct mips_ejtag *ejtag_info, uint8_t data)
{
	struct jtag_tap *tap;
	tap  = ejtag_info->tap;
	assert(tap != NULL);

	struct scan_field field;

	field.num_bits = 8;
	field.out_value = &data;
	field.in_value = NULL;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
}

static int mips_ejtag_step_enable(struct mips_ejtag *ejtag_info)
{
	static const uint32_t code[] = {
			MIPS32_MTC0(1, 31, 0),			/* move $1 to COP0 DeSave */
			MIPS32_MFC0(1, 23, 0),			/* move COP0 Debug to $1 */
			MIPS32_ORI(1, 1, 0x0100),		/* set SSt bit in debug reg */
			MIPS32_MTC0(1, 23, 0),			/* move $1 to COP0 Debug */
			MIPS32_B(NEG16(5)),
			MIPS32_MFC0(1, 31, 0),			/* move COP0 DeSave to $1 */
	};

	return mips32_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
			0, NULL, 0, NULL, 1);
}

static int mips_ejtag_step_disable(struct mips_ejtag *ejtag_info)
{
	static const uint32_t code[] = {
			MIPS32_MTC0(15, 31, 0),							/* move $15 to COP0 DeSave */
			MIPS32_LUI(15, UPPER16(MIPS32_PRACC_STACK)),	/* $15 = MIPS32_PRACC_STACK */
			MIPS32_ORI(15, 15, LOWER16(MIPS32_PRACC_STACK)),
			MIPS32_SW(1, 0, 15),							/* sw $1,($15) */
			MIPS32_SW(2, 0, 15),							/* sw $2,($15) */
			MIPS32_MFC0(1, 23, 0),							/* move COP0 Debug to $1 */
			MIPS32_LUI(2, 0xFFFF),							/* $2 = 0xfffffeff */
			MIPS32_ORI(2, 2, 0xFEFF),
			MIPS32_AND(1, 1, 2),
			MIPS32_MTC0(1, 23, 0),							/* move $1 to COP0 Debug */
			MIPS32_LW(2, 0, 15),
			MIPS32_LW(1, 0, 15),
			MIPS32_B(NEG16(13)),
			MIPS32_MFC0(15, 31, 0),							/* move COP0 DeSave to $15 */
	};

	return mips32_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		0, NULL, 0, NULL, 1);
}

int mips_ejtag_config_step(struct mips_ejtag *ejtag_info, int enable_step)
{
	if (enable_step)
		return mips_ejtag_step_enable(ejtag_info);
	return mips_ejtag_step_disable(ejtag_info);
}

int mips_ejtag_enter_debug(struct mips_ejtag *ejtag_info)
{
	uint32_t ejtag_ctrl;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);

	/* set debug break bit */
	ejtag_ctrl = ejtag_info->ejtag_ctrl | EJTAG_CTRL_JTAGBRK;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* break bit will be cleared by hardware */
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	LOG_DEBUG("ejtag_ctrl: 0x%8.8" PRIx32 "", ejtag_ctrl);
	if ((ejtag_ctrl & EJTAG_CTRL_BRKST) == 0) {
		LOG_ERROR("Failed to enter Debug Mode!");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int mips_ejtag_exit_debug(struct mips_ejtag *ejtag_info)
{
	uint32_t inst;
	inst = MIPS32_DRET;

	/* execute our dret instruction */
	return mips32_pracc_exec(ejtag_info, 1, &inst, 0, NULL, 0, NULL, 0);
}

int mips_ejtag_read_debug(struct mips_ejtag *ejtag_info, uint32_t* debug_reg)
{
	/* read ejtag ECR */
	static const uint32_t code[] = {
			MIPS32_MTC0(15, 31, 0),							/* move $15 to COP0 DeSave */
			MIPS32_LUI(15, UPPER16(MIPS32_PRACC_STACK)),	/* $15 = MIPS32_PRACC_STACK */
			MIPS32_ORI(15, 15, LOWER16(MIPS32_PRACC_STACK)),
			MIPS32_SW(1, 0, 15),							/* sw $1,($15) */
			MIPS32_SW(2, 0, 15),							/* sw $2,($15) */
			MIPS32_LUI(1, UPPER16(MIPS32_PRACC_PARAM_OUT)),	/* $1 = MIPS32_PRACC_PARAM_OUT */
			MIPS32_ORI(1, 1, LOWER16(MIPS32_PRACC_PARAM_OUT)),
			MIPS32_MFC0(2, 23, 0),							/* move COP0 Debug to $2 */
			MIPS32_SW(2, 0, 1),
			MIPS32_LW(2, 0, 15),
			MIPS32_LW(1, 0, 15),
			MIPS32_B(NEG16(12)),
			MIPS32_MFC0(15, 31, 0),							/* move COP0 DeSave to $15 */
	};

	return mips32_pracc_exec(ejtag_info, ARRAY_SIZE(code), code,
		0, NULL, 1, debug_reg, 1);
}

int mips_ejtag_init(struct mips_ejtag *ejtag_info)
{
	uint32_t ejtag_version;
	int retval;

	retval = mips_ejtag_get_impcode(ejtag_info, &ejtag_info->impcode);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("impcode: 0x%8.8" PRIx32 "", ejtag_info->impcode);

	/* get ejtag version */
	ejtag_version = ((ejtag_info->impcode >> 29) & 0x07);

	switch (ejtag_version) {
		case 0:
			LOG_DEBUG("EJTAG: Version 1 or 2.0 Detected");
			break;
		case 1:
			LOG_DEBUG("EJTAG: Version 2.5 Detected");
			break;
		case 2:
			LOG_DEBUG("EJTAG: Version 2.6 Detected");
			break;
		case 3:
			LOG_DEBUG("EJTAG: Version 3.1 Detected");
			break;
		default:
			LOG_DEBUG("EJTAG: Unknown Version Detected");
			break;
	}
	LOG_DEBUG("EJTAG: features:%s%s%s%s%s%s%s",
		ejtag_info->impcode & EJTAG_IMP_R3K ? " R3k" : " R4k",
		ejtag_info->impcode & EJTAG_IMP_DINT ? " DINT" : "",
		ejtag_info->impcode & (1 << 22) ? " ASID_8" : "",
		ejtag_info->impcode & (1 << 21) ? " ASID_6" : "",
		ejtag_info->impcode & EJTAG_IMP_MIPS16 ? " MIPS16" : "",
		ejtag_info->impcode & EJTAG_IMP_NODMA ? " noDMA" : " DMA",
		ejtag_info->impcode & EJTAG_DCR_MIPS64  ? " MIPS64" : " MIPS32");

	if ((ejtag_info->impcode & EJTAG_IMP_NODMA) == 0)
		LOG_DEBUG("EJTAG: DMA Access Mode Support Enabled");

	/* set initial state for ejtag control reg */
	ejtag_info->ejtag_ctrl = EJTAG_CTRL_ROCC | EJTAG_CTRL_PRACC | EJTAG_CTRL_PROBEN | EJTAG_CTRL_SETDEV;
	ejtag_info->fast_access_save = -1;

	return ERROR_OK;
}

int mips_ejtag_fastdata_scan(struct mips_ejtag *ejtag_info, int write_t, uint32_t *data)
{
	struct jtag_tap *tap;

	tap = ejtag_info->tap;
	assert(tap != NULL);

	struct scan_field fields[2];
	uint8_t spracc = 0;
	uint8_t t[4] = {0, 0, 0, 0};

	/* fastdata 1-bit register */
	fields[0].num_bits = 1;
	fields[0].out_value = &spracc;
	fields[0].in_value = NULL;

	/* processor access data register 32 bit */
	fields[1].num_bits = 32;
	fields[1].out_value = t;

	if (write_t) {
		fields[1].in_value = NULL;
		buf_set_u32(t, 0, 32, *data);
	} else
		fields[1].in_value = (void *) data;

	jtag_add_dr_scan(tap, 2, fields, TAP_IDLE);

	if (!write_t && data)
		jtag_add_callback(mips_le_to_h_u32,
			(jtag_callback_data_t) data);

	keep_alive();

	return ERROR_OK;
}
