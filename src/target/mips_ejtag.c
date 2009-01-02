/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
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

#include "binarybuffer.h"
#include "log.h"
#include "jtag.h"

#include <stdlib.h>

int mips_ejtag_set_instr(mips_ejtag_t *ejtag_info, int new_instr, in_handler_t handler)
{
	jtag_tap_t *tap;

	tap = ejtag_info->tap;
	if (tap==NULL)
		return ERROR_FAIL;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != new_instr)
	{
		scan_field_t field;
		u8 t[4];

		field.tap = tap;
		field.num_bits = tap->ir_length;
		field.out_value = t;
		buf_set_u32(field.out_value, 0, field.num_bits, new_instr);
		field.out_mask = NULL;
		field.in_value = NULL;
		field.in_check_value = NULL;
		field.in_check_mask = NULL;
		field.in_handler = handler;
		field.in_handler_priv = NULL;
		jtag_add_ir_scan(1, &field, -1);
	}

	return ERROR_OK;
}

int mips_ejtag_get_idcode(mips_ejtag_t *ejtag_info, u32 *idcode, in_handler_t handler)
{
	scan_field_t field;

	jtag_add_end_state(TAP_IDLE);

	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_IDCODE, NULL);

	field.tap = ejtag_info->tap;
	field.num_bits = 32;
	field.out_value = NULL;
	field.out_mask = NULL;
	field.in_value = (void*)idcode;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;
	jtag_add_dr_scan(1, &field, -1);

	if (jtag_execute_queue() != ERROR_OK)
	{
		LOG_ERROR("register read failed");
	}

	return ERROR_OK;
}

int mips_ejtag_get_impcode(mips_ejtag_t *ejtag_info, u32 *impcode, in_handler_t handler)
{
	scan_field_t field;

	jtag_add_end_state(TAP_IDLE);

	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_IMPCODE, NULL);

	field.tap = ejtag_info->tap;
	field.num_bits = 32;
	field.out_value = NULL;
	field.out_mask = NULL;
	field.in_value = (void*)impcode;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;
	jtag_add_dr_scan(1, &field, -1);

	if (jtag_execute_queue() != ERROR_OK)
	{
		LOG_ERROR("register read failed");
	}

	return ERROR_OK;
}

int mips_ejtag_drscan_32(mips_ejtag_t *ejtag_info, u32 *data)
{
	jtag_tap_t *tap;
	tap  = ejtag_info->tap;

	if (tap==NULL)
		return ERROR_FAIL;
	scan_field_t field;
	u8 t[4];
	int retval;

	field.tap = tap;
	field.num_bits = 32;
	field.out_value = t;
	buf_set_u32(field.out_value, 0, field.num_bits, *data);
	field.out_mask = NULL;
	field.in_value = (u8*)data;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;
	jtag_add_dr_scan(1, &field, -1);

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("register read failed");
		return retval;
	}

	keep_alive();

	return ERROR_OK;
}

int mips_ejtag_step_enable(mips_ejtag_t *ejtag_info)
{
	u32 code[] = {
			MIPS32_MTC0(1,31,0),			/* move $1 to COP0 DeSave */
			MIPS32_MFC0(1,23,0),			/* move COP0 Debug to $1 */
			MIPS32_ORI(1,1,0x0100),			/* set SSt bit in debug reg */
			MIPS32_MTC0(1,23,0),			/* move $1 to COP0 Debug */
			MIPS32_MFC0(1,31,0),			/* move COP0 DeSave to $1 */
			MIPS32_NOP,
			MIPS32_B(NEG16(7)),
			MIPS32_NOP,
	};

	mips32_pracc_exec(ejtag_info, sizeof(code)/sizeof(code[0]), code, \
		0, NULL, 0, NULL, 1);

	return ERROR_OK;
}
int mips_ejtag_step_disable(mips_ejtag_t *ejtag_info)
{
	u32 code[] = {
			MIPS32_MTC0(15,31,0),							/* move $15 to COP0 DeSave */
			MIPS32_LUI(15,UPPER16(MIPS32_PRACC_STACK)), 	/* $15 = MIPS32_PRACC_STACK */
			MIPS32_ORI(15,15,LOWER16(MIPS32_PRACC_STACK)),
			MIPS32_SW(1,0,15), 								/* sw $1,($15) */
			MIPS32_SW(2,0,15), 								/* sw $2,($15) */
			MIPS32_MFC0(1,23,0),							/* move COP0 Debug to $1 */
			MIPS32_LUI(2,0xFFFF), 							/* $2 = 0xfffffeff */
			MIPS32_ORI(2,2,0xFEFF),
			MIPS32_AND(1,1,2),
			MIPS32_MTC0(1,23,0),							/* move $1 to COP0 Debug */
			MIPS32_LW(2,0,15),
			MIPS32_LW(1,0,15),
			MIPS32_MFC0(15,31,0),							/* move COP0 DeSave to $15 */
			MIPS32_NOP,
			MIPS32_B(NEG16(15)),
			MIPS32_NOP,
	};

	mips32_pracc_exec(ejtag_info, sizeof(code)/sizeof(code[0]), code, \
		0, NULL, 0, NULL, 1);

	return ERROR_OK;
}

int mips_ejtag_config_step(mips_ejtag_t *ejtag_info, int enable_step)
{
	if (enable_step)
		return mips_ejtag_step_enable(ejtag_info);
	return mips_ejtag_step_disable(ejtag_info);
}

int mips_ejtag_enter_debug(mips_ejtag_t *ejtag_info)
{
	u32 ejtag_ctrl;
	jtag_add_end_state(TAP_IDLE);
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL, NULL);

	/* set debug break bit */
	ejtag_ctrl = ejtag_info->ejtag_ctrl | EJTAG_CTRL_JTAGBRK;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);

	/* break bit will be cleared by hardware */
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	LOG_DEBUG("ejtag_ctrl: 0x%8.8x", ejtag_ctrl);
	if((ejtag_ctrl & EJTAG_CTRL_BRKST) == 0)
		LOG_DEBUG("Failed to enter Debug Mode!");

	return ERROR_OK;
}

int mips_ejtag_exit_debug(mips_ejtag_t *ejtag_info, int enable_interrupts)
{
	u32 inst;
	inst = MIPS32_DRET;

	/* TODO : enable/disable interrrupts */

	/* execute our dret instruction */
	mips32_pracc_exec(ejtag_info, 1, &inst, 0, NULL, 0, NULL, 0);

	return ERROR_OK;
}

int mips_ejtag_read_debug(mips_ejtag_t *ejtag_info, u32* debug_reg)
{
	/* read ejtag ECR */
	u32 code[] = {
			MIPS32_MTC0(15,31,0),							/* move $15 to COP0 DeSave */
			MIPS32_LUI(15,UPPER16(MIPS32_PRACC_STACK)), 	/* $15 = MIPS32_PRACC_STACK */
			MIPS32_ORI(15,15,LOWER16(MIPS32_PRACC_STACK)),
			MIPS32_SW(1,0,15), 								/* sw $1,($15) */
			MIPS32_SW(2,0,15), 								/* sw $2,($15) */
			MIPS32_LUI(1,UPPER16(MIPS32_PRACC_PARAM_OUT)), 	/* $1 = MIPS32_PRACC_PARAM_OUT */
			MIPS32_ORI(1,1,LOWER16(MIPS32_PRACC_PARAM_OUT)),
			MIPS32_MFC0(2,23,0),							/* move COP0 Debug to $2 */
			MIPS32_SW(2,0,1),
			MIPS32_LW(2,0,15),
			MIPS32_LW(1,0,15),
			MIPS32_MFC0(15,31,0),							/* move COP0 DeSave to $15 */
			MIPS32_NOP,
			MIPS32_B(NEG16(14)),
			MIPS32_NOP,
	};

	mips32_pracc_exec(ejtag_info, sizeof(code)/sizeof(code[0]), code, \
		0, NULL, 1, debug_reg, 1);

	return ERROR_OK;
}

int mips_ejtag_init(mips_ejtag_t *ejtag_info)
{
	u32 ejtag_version;

	mips_ejtag_get_impcode(ejtag_info, &ejtag_info->impcode, NULL);
	LOG_DEBUG("impcode: 0x%8.8x", ejtag_info->impcode);

	/* get ejtag version */
	ejtag_version = ((ejtag_info->impcode >> 29) & 0x07);

	switch (ejtag_version)
	{
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
		ejtag_info->impcode & (1<<28) ? " R3k":    " R4k",
		ejtag_info->impcode & (1<<24) ? " DINT":   "",
		ejtag_info->impcode & (1<<22) ? " ASID_8": "",
		ejtag_info->impcode & (1<<21) ? " ASID_6": "",
		ejtag_info->impcode & (1<<16) ? " MIPS16": "",
		ejtag_info->impcode & (1<<14) ? " noDMA":  " DMA",
		ejtag_info->impcode & (1<<0)  ? " MIPS64": " MIPS32"
	);

	if((ejtag_info->impcode & (1<<14)) == 0)
		LOG_DEBUG("EJTAG: DMA Access Mode Support Enabled");

	/* set initial state for ejtag control reg */
	ejtag_info->ejtag_ctrl = EJTAG_CTRL_ROCC | EJTAG_CTRL_PRACC | EJTAG_CTRL_PROBEN | EJTAG_CTRL_SETDEV;

	return ERROR_OK;
}
