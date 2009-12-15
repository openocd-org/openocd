/***************************************************************************
 *   Copyright (C) 2009 by Mathias Kuester                                 *
 *   mkdorg@users.sourceforge.net                                          *
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

#include <helper/jim.h>

#include "target.h"
#include "target_type.h"
#include "register.h"
#include "dsp563xx.h"
#include "dsp563xx_once.h"

/** single word instruction */
int dsp563xx_once_ir_exec(struct jtag_tap *tap, uint8_t instr, uint8_t rw,
			  uint8_t go, uint8_t ex)
{
	dsp563xx_write_dr_u8(tap, 0,
			     instr | (ex << 5) | (go << 6) | (rw << 7), 8, 0);
	dsp563xx_execute_queue();

	return ERROR_OK;
}

/** single word instruction */
int dsp563xx_once_ir_exec_nq(struct jtag_tap *tap, uint8_t instr, uint8_t rw,
			     uint8_t go, uint8_t ex)
{
	dsp563xx_write_dr_u8(tap, 0,
			     instr | (ex << 5) | (go << 6) | (rw << 7), 8, 0);

	return ERROR_OK;
}

/** once read register */
int dsp563xx_once_reg_read(struct jtag_tap *tap, uint8_t reg, uint32_t * data)
{
	uint32_t dr_in;

	dr_in = 0;

	dsp563xx_once_ir_exec(tap, reg, 1, 0, 0);
	dsp563xx_write_dr_u32(tap, &dr_in, 0x00, 24, 0);
	dsp563xx_execute_queue();

	*data = dr_in;

	return ERROR_OK;
}

/** once write register */
int dsp563xx_once_reg_write(struct jtag_tap *tap, uint8_t reg, uint32_t data)
{
	dsp563xx_once_ir_exec(tap, reg, 0, 0, 0);
	dsp563xx_write_dr_u32(tap, 0x00, data, 24, 0);
	dsp563xx_execute_queue();

	return ERROR_OK;
}

/** single word instruction */
int dsp563xx_once_execute_sw_ir(struct jtag_tap *tap, uint32_t opcode)
{
	dsp563xx_once_ir_exec(tap, DSP563XX_ONCE_OPDBR, 0, 1, 0);
	dsp563xx_write_dr_u32(tap, 0, opcode, 24, 0);
	dsp563xx_execute_queue();

	return ERROR_OK;
}

/** double word instruction */
int dsp563xx_once_execute_dw_ir(struct jtag_tap *tap, uint32_t opcode,
				uint32_t operand)
{
	dsp563xx_once_ir_exec(tap, DSP563XX_ONCE_OPDBR, 0, 0, 0);
	dsp563xx_write_dr_u32(tap, 0, opcode, 24, 0);
	dsp563xx_execute_queue();

	dsp563xx_once_ir_exec(tap, DSP563XX_ONCE_OPDBR, 0, 1, 0);
	dsp563xx_write_dr_u32(tap, 0, operand, 24, 0);
	dsp563xx_execute_queue();

	return ERROR_OK;
}

/** single word instruction */
int dsp563xx_once_execute_sw_ir_nq(struct jtag_tap *tap, uint32_t opcode)
{
	dsp563xx_once_ir_exec_nq(tap, DSP563XX_ONCE_OPDBR, 0, 1, 0);
	dsp563xx_write_dr_u32(tap, 0, opcode, 24, 0);

	return ERROR_OK;
}

/** double word instruction */
int dsp563xx_once_execute_dw_ir_nq(struct jtag_tap *tap, uint32_t opcode,
				   uint32_t operand)
{
	dsp563xx_once_ir_exec_nq(tap, DSP563XX_ONCE_OPDBR, 0, 0, 0);
	dsp563xx_write_dr_u32(tap, 0, opcode, 24, 0);

	dsp563xx_once_ir_exec_nq(tap, DSP563XX_ONCE_OPDBR, 0, 1, 0);
	dsp563xx_write_dr_u32(tap, 0, operand, 24, 0);

	return ERROR_OK;
}
