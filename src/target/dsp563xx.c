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

#define DSP563XX_JTAG_INS_LEN		4

#define JTAG_STATUS_NORMAL		0x01
#define JTAG_STATUS_STOPWAIT		0x05
#define JTAG_STATUS_BUSY		0x09
#define JTAG_STATUS_DEBUG		0x0d

#define JTAG_INSTR_EXTEST		0x00
#define JTAG_INSTR_SAMPLE_PRELOAD	0x01
#define JTAG_INSTR_IDCODE		0x02
#define JTAG_INSTR_CLAMP		0x03
#define JTAG_INSTR_HIZ			0x04
#define JTAG_INSTR_ENABLE_ONCE		0x06
#define JTAG_INSTR_DEBUG_REQUEST	0x07
#define JTAG_INSTR_BYPASS		0x0F

/* forward declarations */
int dsp563xx_target_create(struct target *target, Jim_Interp * interp);
int dsp563xx_init_target(struct command_context *cmd_ctx, struct target *target);

int dsp563xx_arch_state(struct target *target);
int dsp563xx_poll(struct target *target);
int dsp563xx_halt(struct target *target);
int dsp563xx_resume(struct target *target, int current, uint32_t address,
		    int handle_breakpoints, int debug_execution);
int dsp563xx_step(struct target *target, int current, uint32_t address,
		  int handle_breakpoints);

int dsp563xx_assert_reset(struct target *target);
int dsp563xx_deassert_reset(struct target *target);
int dsp563xx_soft_reset_halt(struct target *target);

/* IR and DR functions */
int dsp563xx_jtag_sendinstr(struct jtag_tap *tap, uint8_t * ir_in, uint8_t ir_out);
int dsp563xx_jtag_senddat(struct jtag_tap *tap, uint32_t * dr_in, uint32_t dr_out,
			  int len);

int dsp563xx_read_memory_p(struct target *target, uint32_t address, uint32_t size,
			   uint32_t count, uint8_t * buffer);
int dsp563xx_write_memory_p(struct target *target, uint32_t address, uint32_t size,
			    uint32_t count, uint8_t * buffer);

#define ASM_REG_R_R0	0x607000
#define ASM_REG_R_R1	0x617000
#define ASM_REG_R_R2	0x627000
#define ASM_REG_R_R3	0x637000
#define ASM_REG_R_R4	0x647000
#define ASM_REG_R_R5	0x657000
#define ASM_REG_R_R6	0x667000
#define ASM_REG_R_R7	0x677000

#define ASM_REG_W_R0	0x60F400
#define ASM_REG_W_R1	0x61F400
#define ASM_REG_W_R2	0x62F400
#define ASM_REG_W_R3	0x63F400
#define ASM_REG_W_R4	0x64F400
#define ASM_REG_W_R5	0x65F400
#define ASM_REG_W_R6	0x66F400
#define ASM_REG_W_R7	0x67F400

#define ASM_REG_R_N0	0x707000
#define ASM_REG_R_N1	0x717000
#define ASM_REG_R_N2	0x727000
#define ASM_REG_R_N3	0x737000
#define ASM_REG_R_N4	0x747000
#define ASM_REG_R_N5	0x757000
#define ASM_REG_R_N6	0x767000
#define ASM_REG_R_N7	0x777000

#define ASM_REG_W_N0	0x70F400
#define ASM_REG_W_N1	0x71F400
#define ASM_REG_W_N2	0x72F400
#define ASM_REG_W_N3	0x73F400
#define ASM_REG_W_N4	0x74F400
#define ASM_REG_W_N5	0x75F400
#define ASM_REG_W_N6	0x76F400
#define ASM_REG_W_N7	0x77F400

#define ASM_REG_R_M0	0x057020	/* control register m[0..7] */
#define ASM_REG_R_M1	0x057021
#define ASM_REG_R_M2	0x057022
#define ASM_REG_R_M3	0x057023
#define ASM_REG_R_M4	0x057024
#define ASM_REG_R_M5	0x057025
#define ASM_REG_R_M6	0x057026
#define ASM_REG_R_M7	0x057027

#define ASM_REG_W_M0	0x05F420
#define ASM_REG_W_M1	0x05F421
#define ASM_REG_W_M2	0x05F422
#define ASM_REG_W_M3	0x05F423
#define ASM_REG_W_M4	0x05F424
#define ASM_REG_W_M5	0x05F425
#define ASM_REG_W_M6	0x05F426
#define ASM_REG_W_M7	0x05F427

#define ASM_REG_R_X0	0x447000
#define ASM_REG_R_X1	0x457000

#define ASM_REG_W_X0	0x44F400
#define ASM_REG_W_X1	0x45F400

#define ASM_REG_R_Y0	0x467000
#define ASM_REG_R_Y1	0x477000

#define ASM_REG_W_Y0	0x46F400
#define ASM_REG_W_Y1	0x47F400

#define ASM_REG_R_A0	0x507000
#define ASM_REG_R_A1	0x547000
#define ASM_REG_R_A2	0x527000

#define ASM_REG_W_A0	0x50F400
#define ASM_REG_W_A1	0x54F400
#define ASM_REG_W_A2	0x52F400

#define ASM_REG_R_B0	0x517000
#define ASM_REG_R_B1	0x557000
#define ASM_REG_R_B2	0x537000

#define ASM_REG_W_B0	0x51F400
#define ASM_REG_W_B1	0x55F400
#define ASM_REG_W_B2	0x53F400

#define ASM_REG_R_VBA	0x057030	/* control register */
#define ASM_REG_W_VBA	0x05F430

#define ASM_REG_R_OMR	0x05703A	/* control register */
#define ASM_REG_W_OMR	0x05F43A

#define ASM_REG_R_EP	0x05702A
#define ASM_REG_W_EP	0x05F42A

#define ASM_REG_R_SC	0x057031	/* stack counter */
#define ASM_REG_W_SC	0x05F431

#define ASM_REG_R_SZ	0x057038	/* stack size */
#define ASM_REG_W_SZ	0x05F438

#define ASM_REG_R_SR	0x057039	/* control register, status register */
#define ASM_REG_W_SR	0x05F439

#define ASM_REG_R_SP	0x05703B	/* control register, stack pointer */
#define ASM_REG_W_SP	0x05F43B

#define ASM_REG_R_SSH	0x05703C	/* control register, system stack high */
#define ASM_REG_W_SSH	0x05743C

#define ASM_REG_R_SSL	0x05703D	/* control register, system stack low */
#define ASM_REG_W_SSL	0x05F43D

#define ASM_REG_R_LA	0x05703E	/* control register, loop address */
#define ASM_REG_W_LA	0x05F43E

#define ASM_REG_R_LC	0x05703F	/* control register, loop count */
#define ASM_REG_W_LC	0x05F43F

#define ASM_REG_R_PC	0x000000
#define ASM_REG_W_PC	0x000000

static const struct
{
	unsigned id;
	char *name;
	unsigned bits;
	uint32_t r_cmd;
	uint32_t w_cmd;
} dsp563xx_regs[] =
{
	/* *INDENT-OFF* */
	{0, "r0", 24, ASM_REG_R_R0, ASM_REG_W_R0},
	{1, "r1", 24, ASM_REG_R_R1, ASM_REG_W_R1},
	{2, "r2", 24, ASM_REG_R_R2, ASM_REG_W_R2},
	{3, "r3", 24, ASM_REG_R_R3, ASM_REG_W_R3},
	{4, "r4", 24, ASM_REG_R_R4, ASM_REG_W_R4},
	{5, "r5", 24, ASM_REG_R_R5, ASM_REG_W_R5},
	{6, "r6", 24, ASM_REG_R_R6, ASM_REG_W_R6},
	{7, "r7", 24, ASM_REG_R_R7, ASM_REG_W_R7},
	{8, "n0", 24, ASM_REG_R_N0, ASM_REG_W_N0},
	{9, "n1", 24, ASM_REG_R_N1, ASM_REG_W_N1},
	{10, "n2", 24, ASM_REG_R_N2, ASM_REG_W_N2},
	{11, "n3", 24, ASM_REG_R_N3, ASM_REG_W_N3},
	{12, "n4", 24, ASM_REG_R_N4, ASM_REG_W_N4},
	{13, "n5", 24, ASM_REG_R_N5, ASM_REG_W_N5},
	{14, "n6", 24, ASM_REG_R_N6, ASM_REG_W_N6},
	{15, "n7", 24, ASM_REG_R_N7, ASM_REG_W_N7},
	{16, "m0", 24, ASM_REG_R_M0, ASM_REG_W_M0},
	{17, "m1", 24, ASM_REG_R_M1, ASM_REG_W_M1},
	{18, "m2", 24, ASM_REG_R_M2, ASM_REG_W_M2},
	{19, "m3", 24, ASM_REG_R_M3, ASM_REG_W_M3},
	{20, "m4", 24, ASM_REG_R_M4, ASM_REG_W_M4},
	{21, "m5", 24, ASM_REG_R_M5, ASM_REG_W_M5},
	{22, "m6", 24, ASM_REG_R_M6, ASM_REG_W_M6},
	{23, "m7", 24, ASM_REG_R_M7, ASM_REG_W_M7},
	{24, "x0", 24, ASM_REG_R_X0, ASM_REG_W_X0},
	{25, "x1", 24, ASM_REG_R_X1, ASM_REG_W_X1},
	{26, "y0", 24, ASM_REG_R_Y0, ASM_REG_W_Y0},
	{27, "y1", 24, ASM_REG_R_Y1, ASM_REG_W_Y1},
	{28, "a0", 24, ASM_REG_R_A0, ASM_REG_W_A0},
	{29, "a1", 24, ASM_REG_R_A1, ASM_REG_W_A1},
	{30, "a2", 8, ASM_REG_R_A2, ASM_REG_W_A2},
	{31, "b0", 24, ASM_REG_R_B0, ASM_REG_W_B0},
	{32, "b1", 24, ASM_REG_R_B1, ASM_REG_W_B1},
	{33, "b2", 8, ASM_REG_R_B2, ASM_REG_W_B2},
	{34, "omr", 24, ASM_REG_R_OMR, ASM_REG_W_OMR},
	{35, "vba", 24, ASM_REG_R_VBA, ASM_REG_W_VBA},
	{36, "ep", 24, ASM_REG_R_EP, ASM_REG_W_EP},
	{37, "sc", 24, ASM_REG_R_SC, ASM_REG_W_SC},
	{38, "sz", 24, ASM_REG_R_SZ, ASM_REG_W_SZ},
	{39, "sr", 24, ASM_REG_R_SR, ASM_REG_W_SR},
	{40, "sp", 24, ASM_REG_R_SP, ASM_REG_W_SP},
	{41, "la", 24, ASM_REG_R_LA, ASM_REG_W_LA},
	{42, "lc", 24, ASM_REG_R_LC, ASM_REG_W_LC},
	{43, "pc", 24, ASM_REG_R_PC, ASM_REG_W_PC}
	/* *INDENT-ON* */
};

static int dsp563xx_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
			      int *reg_list_size)
{
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	int i;

	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}

	*reg_list_size = DSP563XX_NUMCOREREGS;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	for (i = 0; i < DSP563XX_NUMCOREREGS; i++)
	{
		(*reg_list)[i] = &dsp563xx->core_cache->reg_list[i];
	}

	return ERROR_OK;

}

int dsp563xx_read_core_reg(struct target *target, int num)
{
	uint32_t reg_value;
	struct dsp563xx_core_reg *dsp563xx_core_reg;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if ((num < 0) || (num >= DSP563XX_NUMCOREREGS))
		return ERROR_INVALID_ARGUMENTS;

	dsp563xx_core_reg = dsp563xx->core_cache->reg_list[num].arch_info;
	reg_value = dsp563xx->core_regs[num];
	buf_set_u32(dsp563xx->core_cache->reg_list[num].value, 0, 32, reg_value);
	dsp563xx->core_cache->reg_list[num].valid = 1;
	dsp563xx->core_cache->reg_list[num].dirty = 0;

	return ERROR_OK;
}

int dsp563xx_write_core_reg(struct target *target, int num)
{
	uint32_t reg_value;
	struct dsp563xx_core_reg *dsp563xx_core_reg;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if ((num < 0) || (num >= DSP563XX_NUMCOREREGS))
		return ERROR_INVALID_ARGUMENTS;

	reg_value = buf_get_u32(dsp563xx->core_cache->reg_list[num].value, 0, 32);
	dsp563xx_core_reg = dsp563xx->core_cache->reg_list[num].arch_info;
	dsp563xx->core_regs[num] = reg_value;
	dsp563xx->core_cache->reg_list[num].valid = 1;
	dsp563xx->core_cache->reg_list[num].dirty = 0;

	return ERROR_OK;
}

int dsp563xx_target_create(struct target *target, Jim_Interp * interp)
{
	struct dsp563xx_common *dsp563xx = calloc(1, sizeof(struct dsp563xx_common));

	dsp563xx->jtag_info.tap = target->tap;
	target->arch_info = dsp563xx;
	dsp563xx->read_core_reg = dsp563xx_read_core_reg;
	dsp563xx->write_core_reg = dsp563xx_write_core_reg;

	return ERROR_OK;
}

int dsp563xx_get_core_reg(struct reg *reg)
{
	int retval = 0;

	LOG_DEBUG("%s", __FUNCTION__);

	struct dsp563xx_core_reg *dsp563xx_reg = reg->arch_info;
	struct target *target = dsp563xx_reg->target;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = dsp563xx->read_core_reg(target, dsp563xx_reg->num);

	return retval;
}

int dsp563xx_set_core_reg(struct reg *reg, uint8_t * buf)
{
	LOG_DEBUG("%s", __FUNCTION__);

	struct dsp563xx_core_reg *dsp563xx_reg = reg->arch_info;
	struct target *target = dsp563xx_reg->target;
	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}

	buf_set_u32(reg->value, 0, reg->size, value);
	reg->dirty = 1;
	reg->valid = 1;

	return ERROR_OK;
}

int dsp563xx_save_context(struct target *target)
{
	int i;
	uint32_t data = 0;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	struct dsp563xx_core_reg *arch_info;

	for (i = 0; i < DSP563XX_NUMCOREREGS - 1; i++)
	{

//              if (!dsp563xx->core_cache->reg_list[i].valid)
		{
			arch_info = dsp563xx->core_cache->reg_list[i].arch_info;
			dsp563xx_once_execute_dw_ir(target->tap, arch_info->r_cmd,
						    0xfffffc);
			dsp563xx_once_execute_sw_ir(target->tap, 0x000000);
			dsp563xx_once_reg_read(target->tap, DSP563XX_ONCE_OGDBR,
					       &data);
			dsp563xx->core_regs[i] = data;
			dsp563xx->read_core_reg(target, i);
		}
	}

	/* read pc */
	dsp563xx_once_reg_read(target->tap, DSP563XX_ONCE_OPABEX, &data);
	dsp563xx->core_regs[i] = data;
	dsp563xx->read_core_reg(target, i);

	return ERROR_OK;
}

int dsp563xx_restore_context(struct target *target)
{
	int i;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	struct dsp563xx_core_reg *arch_info;

	for (i = 0; i < DSP563XX_NUMCOREREGS - 1; i++)
	{
		if (dsp563xx->core_cache->reg_list[i].dirty)
		{
			arch_info = dsp563xx->core_cache->reg_list[i].arch_info;

			dsp563xx->write_core_reg(target, i);

			dsp563xx_once_execute_dw_ir(target->tap, arch_info->w_cmd,
						    dsp563xx->core_regs[i]);
			dsp563xx_once_execute_sw_ir(target->tap, 0x000000);
		}
	}

	return ERROR_OK;
}

static const struct reg_arch_type dsp563xx_reg_type = {
	.get = dsp563xx_get_core_reg,
	.set = dsp563xx_set_core_reg,
};

int dsp563xx_init_target(struct command_context *cmd_ctx, struct target *target)
{
	/* get pointers to arch-specific information */
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = malloc(sizeof(struct reg) * DSP563XX_NUMCOREREGS);
	struct dsp563xx_core_reg *arch_info =
		malloc(sizeof(struct dsp563xx_core_reg) * DSP563XX_NUMCOREREGS);
	int i;

	LOG_DEBUG("%s", __FUNCTION__);

	/* Build the process context cache */
	cache->name = "dsp563xx registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = DSP563XX_NUMCOREREGS;
	(*cache_p) = cache;
	dsp563xx->core_cache = cache;

	for (i = 0; i < DSP563XX_NUMCOREREGS; i++)
	{
		arch_info[i].num = dsp563xx_regs[i].id;
		arch_info[i].name = dsp563xx_regs[i].name;
		arch_info[i].size = dsp563xx_regs[i].bits;
		arch_info[i].r_cmd = dsp563xx_regs[i].r_cmd;
		arch_info[i].w_cmd = dsp563xx_regs[i].w_cmd;
		arch_info[i].target = target;
		arch_info[i].dsp563xx_common = dsp563xx;
		reg_list[i].name = dsp563xx_regs[i].name;
		reg_list[i].size = dsp563xx_regs[i].bits;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].type = &dsp563xx_reg_type;
		reg_list[i].arch_info = &arch_info[i];
	}

	return ERROR_OK;
}

int dsp563xx_arch_state(struct target *target)
{
	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

int dsp563xx_jtag_status(struct target *target, uint8_t * status)
{
	uint8_t ir_in;

	ir_in = 0;

	dsp563xx_jtag_sendinstr(target->tap, &ir_in, JTAG_INSTR_ENABLE_ONCE);
	dsp563xx_execute_queue();

	*status = ir_in;

	return ERROR_OK;
}

int dsp563xx_jtag_debug_request(struct target *target)
{
	uint8_t ir_in = 0;
	uint32_t retry = 0;

	while (ir_in != JTAG_STATUS_DEBUG)
	{
		dsp563xx_jtag_sendinstr(target->tap, &ir_in,
					JTAG_INSTR_DEBUG_REQUEST);
		dsp563xx_execute_queue();
		LOG_DEBUG("JTAG CMD 7 res: %02X", ir_in);
		dsp563xx_jtag_sendinstr(target->tap, &ir_in, JTAG_INSTR_ENABLE_ONCE);
		dsp563xx_execute_queue();
		LOG_DEBUG("JTAG CMD 6 res: %02X", ir_in);

		if (retry++ == 100)
			return ERROR_TARGET_FAILURE;
	}

	if (ir_in != JTAG_STATUS_DEBUG)
	{
		return ERROR_TARGET_FAILURE;
	}

	return ERROR_OK;
}

int dsp563xx_poll(struct target *target)
{
	uint8_t jtag_status;
	uint32_t once_status;

	dsp563xx_jtag_status(target, &jtag_status);

	if ((jtag_status & 1) != 1)
	{
		target->state = TARGET_UNKNOWN;
		LOG_ERROR
			("jtag status contains invalid mode value - communication failure");
		return ERROR_TARGET_FAILURE;
	}

	if (jtag_status != JTAG_STATUS_DEBUG)
	{
		target->state = TARGET_RUNNING;
	}

	dsp563xx_once_reg_read(target->tap, DSP563XX_ONCE_OSCR, &once_status);

	if ((once_status & DSP563XX_ONCE_OSCR_DEBUG_M) == DSP563XX_ONCE_OSCR_DEBUG_M)
	{
		target->state = TARGET_HALTED;

	}

	return ERROR_OK;
}

int dsp563xx_halt(struct target *target)
{
	uint8_t jtag_status;
	uint32_t once_status;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if (target->state == TARGET_HALTED)
	{
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
	{
		LOG_WARNING("target was in unknown state when halt was requested");
	}

//      if ( jtag_status != 0x0d )
	{
		dsp563xx_jtag_debug_request(target);

		/* store pipeline register */
		dsp563xx_once_reg_read(target->tap, DSP563XX_ONCE_OPILR,
				       &dsp563xx->pipeline_context.once_opilr);
		dsp563xx_once_reg_read(target->tap, DSP563XX_ONCE_OPDBR,
				       &dsp563xx->pipeline_context.once_opdbr);

		dsp563xx_save_context(target);

		dsp563xx_jtag_status(target, &jtag_status);
		LOG_DEBUG("%02X", jtag_status);
		dsp563xx_once_reg_read(target->tap, DSP563XX_ONCE_OSCR,
				       &once_status);
		LOG_DEBUG("%02X", (unsigned) once_status);
	}

	LOG_DEBUG("target->state: %s", target_state_name(target));

	LOG_DEBUG("%s", __FUNCTION__);

	return ERROR_OK;
}

#define DSP563XX_ASM_CMD_JUMP	0x0AF080

int dsp563xx_resume(struct target *target, int current, uint32_t address,
		    int handle_breakpoints, int debug_execution)
{
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	LOG_DEBUG("%s", __FUNCTION__);

	dsp563xx_restore_context(target);

	if (current)
	{
		/* restore pipeline registers and go */
		dsp563xx_once_reg_write(target->tap, DSP563XX_ONCE_OPILR,
					dsp563xx->pipeline_context.once_opilr);
		dsp563xx_once_reg_write(target->tap,
					DSP563XX_ONCE_OPDBR | DSP563XX_ONCE_OCR_EX |
					DSP563XX_ONCE_OCR_GO,
					dsp563xx->pipeline_context.once_opdbr);
	}
	else
	{
		/* set to go register and jump */
		dsp563xx_once_reg_write(target->tap, DSP563XX_ONCE_OPDBR,
					DSP563XX_ASM_CMD_JUMP);
		dsp563xx_once_reg_write(target->tap,
					DSP563XX_ONCE_PDBGOTO | DSP563XX_ONCE_OCR_EX
					| DSP563XX_ONCE_OCR_GO, address);
	}

	target->state = TARGET_RUNNING;

	return ERROR_OK;
}

int dsp563xx_step(struct target *target, int current, uint32_t address,
		  int handle_breakpoints)
{
	uint32_t once_status;
	uint32_t dr_in, cnt;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if (target->state != TARGET_HALTED)
	{
		LOG_DEBUG("target was not halted");
		return ERROR_OK;
	}

	LOG_DEBUG("%s %08X %08X", __FUNCTION__, current, (unsigned) address);

	dsp563xx_jtag_debug_request(target);

	dsp563xx_restore_context(target);

	/* reset trace mode */
	dsp563xx_once_reg_write(target->tap, DSP563XX_ONCE_OSCR, 0x000000);
	/* enable trace mode */
	dsp563xx_once_reg_write(target->tap, DSP563XX_ONCE_OSCR,
				DSP563XX_ONCE_OSCR_TME);

	cnt = 0;

	/* on JUMP we need one extra cycle */
	if (!current)
		cnt++;

	/* load step counter with N-1 */
	dsp563xx_once_reg_write(target->tap, DSP563XX_ONCE_OTC, cnt);

	if (current)
	{
		/* restore pipeline registers and go */
		dsp563xx_once_reg_write(target->tap, DSP563XX_ONCE_OPILR,
					dsp563xx->pipeline_context.once_opilr);
		dsp563xx_once_reg_write(target->tap,
					DSP563XX_ONCE_OPDBR | DSP563XX_ONCE_OCR_EX |
					DSP563XX_ONCE_OCR_GO,
					dsp563xx->pipeline_context.once_opdbr);
	}
	else
	{
		/* set to go register and jump */
		dsp563xx_once_reg_write(target->tap, DSP563XX_ONCE_OPDBR,
					DSP563XX_ASM_CMD_JUMP);
		dsp563xx_once_reg_write(target->tap,
					DSP563XX_ONCE_PDBGOTO | DSP563XX_ONCE_OCR_EX
					| DSP563XX_ONCE_OCR_GO, address);
	}

	while (1)
	{
		dsp563xx_once_reg_read(target->tap, DSP563XX_ONCE_OSCR,
				       &once_status);

		if (once_status & DSP563XX_ONCE_OSCR_TO)
		{
			/* store pipeline register */
			dsp563xx_once_reg_read(target->tap, DSP563XX_ONCE_OPILR,
					       &dsp563xx->pipeline_context.
					       once_opilr);
			dsp563xx_once_reg_read(target->tap, DSP563XX_ONCE_OPDBR,
					       &dsp563xx->pipeline_context.
					       once_opdbr);

			dsp563xx_save_context(target);

			dsp563xx_once_reg_read(target->tap, DSP563XX_ONCE_OPABFR,
					       &dr_in);
			LOG_DEBUG("%08X", (unsigned) dr_in);
			dsp563xx_once_reg_read(target->tap, DSP563XX_ONCE_OPABDR,
					       &dr_in);
			LOG_DEBUG("%08X", (unsigned) dr_in);
			dsp563xx_once_reg_read(target->tap, DSP563XX_ONCE_OPABEX,
					       &dr_in);
			LOG_DEBUG("%08X", (unsigned) dr_in);

			/* reset trace mode */
			dsp563xx_once_reg_write(target->tap, DSP563XX_ONCE_OSCR,
						0x000000);

			break;
		}
	}

	return ERROR_OK;
}

int dsp563xx_assert_reset(struct target *target)
{
	target->state = TARGET_RESET;

	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

int dsp563xx_deassert_reset(struct target *target)
{
	target->state = TARGET_RUNNING;

	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

int dsp563xx_soft_reset_halt(struct target *target)
{
	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

/*
* 000000			nop
* 46F400 AABBCC		move              #$aabbcc,y0
* 60F400 AABBCC		move              #$aabbcc,r0
* 467000 AABBCC		move              y0,x:AABBCC
* 607000 AABBCC		move              r0,x:AABBCC

* 46E000		move              x:(r0),y0
* 4EE000		move              y:(r0),y0
* 07E086		move              p:(r0),y0

* 0450B9		move              sr,r0
* 0446BA		move              omr,y0
* 0446BC		move              ssh,y0
* 0446BD		move              ssl,y0
* 0446BE		move              la,y0
* 0446BF		move              lc,y0
* 
* 61F000 AABBCC		move              x:AABBCC,r1
* 076190		movem             r0,p:(r1)
*
*/
int dsp563xx_read_memory_p(struct target *target, uint32_t address,
			   uint32_t size, uint32_t count, uint8_t * buffer)
{
	uint32_t i, x;
	uint32_t data;
	uint8_t *b;

	LOG_DEBUG("address: 0x%8.8" PRIx32 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8"
		  PRIx32, address, size, count);

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	x = count;

	for (i = 0; i < x; i++)
	{
		dsp563xx_once_execute_dw_ir_nq(target->tap, 0x60F400, address + i);
		dsp563xx_once_execute_sw_ir_nq(target->tap, 0x07E086);
		dsp563xx_once_execute_dw_ir_nq(target->tap, 0x467000, 0xfffffc);
		dsp563xx_execute_queue();

		dsp563xx_once_reg_read(target->tap, DSP563XX_ONCE_OGDBR, &data);

		b = buffer + 4 * i;
		if (size > 0)
			*b++ = data >> 0;
		if (size > 1)
			*b++ = data >> 8;
		if (size > 2)
			*b++ = data >> 16;
		if (size > 3)
			*b++ = 0x00;
	}

	return ERROR_OK;
}

int dsp563xx_write_memory_p(struct target *target, uint32_t address, uint32_t size,
			    uint32_t count, uint8_t * buffer)
{
	uint32_t i, x;
	uint32_t data;
	uint8_t *b;

	LOG_DEBUG("address: 0x%8.8" PRIx32 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8"
		  PRIx32 "", address, size, count);

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	x = count;

	for (i = 0; i < x; i++)
	{
		b = buffer + 4 * i;

		data = 0;
		if (size > 0)
			data = *buffer++;
		if (size > 1)
			data |= (*buffer++) << 8;
		if (size > 2)
			data |= (*buffer++) << 16;
		if (size > 3)
			data |= (*buffer++) << 24;

//              LOG_DEBUG("%08X", data);

		dsp563xx_once_execute_dw_ir_nq(target->tap, 0x61F400, address + i);
		dsp563xx_once_execute_dw_ir_nq(target->tap, 0x60F400, data);
		dsp563xx_once_execute_sw_ir_nq(target->tap, 0x076190);
		dsp563xx_execute_queue();
	}

	return ERROR_OK;
}

int dsp563xx_jtag_senddat(struct jtag_tap *tap, uint32_t * dr_in, uint32_t dr_out,
			  int len)
{
	return dsp563xx_write_dr_u32(tap, dr_in, dr_out, len, 1);
}

int dsp563xx_jtag_sendinstr(struct jtag_tap *tap, uint8_t * ir_in, uint8_t ir_out)
{
	return dsp563xx_write_ir_u8(tap, ir_in, ir_out, DSP563XX_JTAG_INS_LEN, 1);
}

/* IR and DR functions */
int dsp563xx_write_ir(struct jtag_tap *tap, uint8_t * ir_in, uint8_t * ir_out,
		      int ir_len, int rti)
{
	if (NULL == tap)
	{
		LOG_ERROR("invalid tap");
		return ERROR_FAIL;
	}
	if (ir_len != tap->ir_length)
	{
		LOG_ERROR("invalid ir_len");
		return ERROR_FAIL;
	}

	{
		struct scan_field field[1];

		field[0].tap = tap;
		field[0].num_bits = tap->ir_length;
		field[0].out_value = ir_out;
		field[0].in_value = ir_in;
		jtag_add_plain_ir_scan(ARRAY_SIZE(field), field,
				       jtag_set_end_state(TAP_IDLE));
	}

	return ERROR_OK;
}

int dsp563xx_write_dr(struct jtag_tap *tap, uint8_t * dr_in, uint8_t * dr_out,
		      int dr_len, int rti)
{
	if (NULL == tap)
	{
		LOG_ERROR("invalid tap");
		return ERROR_FAIL;
	}

	{
		struct scan_field field[1];

		field[0].tap = tap;
		field[0].num_bits = dr_len;
		field[0].out_value = dr_out;
		field[0].in_value = dr_in;
		jtag_add_plain_dr_scan(ARRAY_SIZE(field), field,
				       jtag_set_end_state(TAP_IDLE));
	}

	return ERROR_OK;
}

int dsp563xx_write_ir_u8(struct jtag_tap *tap, uint8_t * ir_in, uint8_t ir_out,
			 int ir_len, int rti)
{
	if (ir_len > 8)
	{
		LOG_ERROR("ir_len overflow, maxium is 8");
		return ERROR_FAIL;
	}

	dsp563xx_write_ir(tap, ir_in, &ir_out, ir_len, rti);

	return ERROR_OK;
}

int dsp563xx_write_dr_u8(struct jtag_tap *tap, uint8_t * dr_in, uint8_t dr_out,
			 int dr_len, int rti)
{
	if (dr_len > 8)
	{
		LOG_ERROR("dr_len overflow, maxium is 8");
		return ERROR_FAIL;
	}

	dsp563xx_write_dr(tap, dr_in, &dr_out, dr_len, rti);

	return ERROR_OK;
}

int dsp563xx_write_ir_u16(struct jtag_tap *tap, uint16_t * ir_in, uint16_t ir_out,
			  int ir_len, int rti)
{
	if (ir_len > 16)
	{
		LOG_ERROR("ir_len overflow, maxium is 16");
		return ERROR_FAIL;
	}

	dsp563xx_write_ir(tap, (uint8_t *) ir_in, (uint8_t *) & ir_out, ir_len, rti);

	return ERROR_OK;
}

int dsp563xx_write_dr_u16(struct jtag_tap *tap, uint16_t * dr_in, uint16_t dr_out,
			  int dr_len, int rti)
{
	if (dr_len > 16)
	{
		LOG_ERROR("dr_len overflow, maxium is 16");
		return ERROR_FAIL;
	}

	dsp563xx_write_dr(tap, (uint8_t *) dr_in, (uint8_t *) & dr_out, dr_len, rti);

	return ERROR_OK;
}

int dsp563xx_write_ir_u32(struct jtag_tap *tap, uint32_t * ir_in, uint32_t ir_out,
			  int ir_len, int rti)
{
	if (ir_len > 32)
	{
		LOG_ERROR("ir_len overflow, maxium is 32");
		return ERROR_FAIL;
	}

	dsp563xx_write_ir(tap, (uint8_t *) ir_in, (uint8_t *) & ir_out, ir_len, rti);

	return ERROR_OK;
}

int dsp563xx_write_dr_u32(struct jtag_tap *tap, uint32_t * dr_in, uint32_t dr_out,
			  int dr_len, int rti)
{
	if (dr_len > 32)
	{
		LOG_ERROR("dr_len overflow, maxium is 32");
		return ERROR_FAIL;
	}

	dsp563xx_write_dr(tap, (uint8_t *) dr_in, (uint8_t *) & dr_out, dr_len, rti);

	return ERROR_OK;
}

int dsp563xx_execute_queue(void)
{
	return jtag_execute_queue();
}

/** Holds methods for DSP563XX targets. */
struct target_type dsp563xx_target = {
	.name = "dsp563xx",

	.poll = dsp563xx_poll,
	.arch_state = dsp563xx_arch_state,

	.target_request_data = NULL,

	.get_gdb_reg_list = dsp563xx_get_gdb_reg_list,

	.halt = dsp563xx_halt,
	.resume = dsp563xx_resume,
	.step = dsp563xx_step,

	.assert_reset = dsp563xx_assert_reset,
	.deassert_reset = dsp563xx_deassert_reset,
	.soft_reset_halt = dsp563xx_soft_reset_halt,

	.read_memory = dsp563xx_read_memory_p,
	.write_memory = dsp563xx_write_memory_p,

	.target_create = dsp563xx_target_create,
	.init_target = dsp563xx_init_target,
};
