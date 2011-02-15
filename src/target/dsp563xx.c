/***************************************************************************
 *   Copyright (C) 2009-2011 by Mathias Kuester                            *
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

#include <jim.h>

#include "target.h"
#include "target_type.h"
#include "register.h"
#include "dsp563xx.h"
#include "dsp563xx_once.h"

#define ASM_REG_W_R0	0x60F400
#define ASM_REG_W_R1	0x61F400
#define ASM_REG_W_R2	0x62F400
#define ASM_REG_W_R3	0x63F400
#define ASM_REG_W_R4	0x64F400
#define ASM_REG_W_R5	0x65F400
#define ASM_REG_W_R6	0x66F400
#define ASM_REG_W_R7	0x67F400

#define ASM_REG_W_N0	0x70F400
#define ASM_REG_W_N1	0x71F400
#define ASM_REG_W_N2	0x72F400
#define ASM_REG_W_N3	0x73F400
#define ASM_REG_W_N4	0x74F400
#define ASM_REG_W_N5	0x75F400
#define ASM_REG_W_N6	0x76F400
#define ASM_REG_W_N7	0x77F400

#define ASM_REG_W_M0	0x05F420
#define ASM_REG_W_M1	0x05F421
#define ASM_REG_W_M2	0x05F422
#define ASM_REG_W_M3	0x05F423
#define ASM_REG_W_M4	0x05F424
#define ASM_REG_W_M5	0x05F425
#define ASM_REG_W_M6	0x05F426
#define ASM_REG_W_M7	0x05F427

#define ASM_REG_W_X0	0x44F400
#define ASM_REG_W_X1	0x45F400

#define ASM_REG_W_Y0	0x46F400
#define ASM_REG_W_Y1	0x47F400

#define ASM_REG_W_A0	0x50F400
#define ASM_REG_W_A1	0x54F400
#define ASM_REG_W_A2	0x52F400

#define ASM_REG_W_B0	0x51F400
#define ASM_REG_W_B1	0x55F400
#define ASM_REG_W_B2	0x53F400

#define ASM_REG_W_VBA	0x05F430
#define ASM_REG_W_OMR	0x05F43A
#define ASM_REG_W_EP	0x05F42A
#define ASM_REG_W_SC	0x05F431
#define ASM_REG_W_SZ	0x05F438
#define ASM_REG_W_SR	0x05F439
#define ASM_REG_W_SP	0x05F43B
#define ASM_REG_W_SSH	0x05F43C
#define ASM_REG_W_SSL	0x05F43D
#define ASM_REG_W_LA	0x05F43E
#define ASM_REG_W_LC	0x05F43F
#define ASM_REG_W_PC	0x000000
#define ASM_REG_W_IPRC	0xFFFFFF
#define ASM_REG_W_IPRP	0xFFFFFE

#define ASM_REG_W_BCR	0xFFFFFB
#define ASM_REG_W_DCR	0xFFFFFA
#define ASM_REG_W_AAR0	0xFFFFF9
#define ASM_REG_W_AAR1	0xFFFFF8
#define ASM_REG_W_AAR2	0xFFFFF7
#define ASM_REG_W_AAR3	0xFFFFF6

static struct once_reg once_regs[] = {
	{0, 0x00, 24, "OSCR", 0},
	{1, 0x01, 24, "OMBC", 0},
	{2, 0x02, 24, "OBCR", 0},
	{3, 0x05, 24, "OMLR0", 0},
	{4, 0x06, 24, "OMLR1", 0},
	{5, 0x09, 24, "OGDBR", 0},
	{6, 0x0a, 24, "OPDBR", 0},
	{7, 0x0b, 24, "OPILR", 0},
	{8, 0x0c, 24, "PDB", 0},
	{9, 0x0d, 24, "OTC", 0},
	{10, 0x0f, 24, "OPABFR", 0},
	{11, 0x10, 24, "OPABDR", 0},
	{12, 0x11, 24, "OPABEX", 0},
	{13, 0x12, 25, "OPABF0", 0},
	{14, 0x12, 25, "OPABF1", 0},
	{15, 0x12, 25, "OPABF2", 0},
	{16, 0x12, 25, "OPABF3", 0},
	{17, 0x12, 25, "OPABF4", 0},
	{18, 0x12, 25, "OPABF5", 0},
	{19, 0x12, 25, "OPABF6", 0},
	{20, 0x12, 25, "OPABF7", 0},
	{21, 0x12, 25, "OPABF8", 0},
	{22, 0x12, 25, "OPABF9", 0},
	{23, 0x12, 25, "OPABF10", 0},
	{24, 0x12, 25, "OPABF11", 0},
//      {25,0x1f,24,"NRSEL",0},
};

static const struct
{
	unsigned id;
	const char *name;
	unsigned bits;
	/* effective addressing mode encoding */
	uint8_t eame;
	uint32_t instr_mask;
} dsp563xx_regs[] =
{
	/* *INDENT-OFF* */
	/* address registers */
	{ 0, "r0", 24, 0x10, ASM_REG_W_R0},
	{ 1, "r1", 24, 0x11, ASM_REG_W_R1},
	{ 2, "r2", 24, 0x12, ASM_REG_W_R2},
	{ 3, "r3", 24, 0x13, ASM_REG_W_R3},
	{ 4, "r4", 24, 0x14, ASM_REG_W_R4},
	{ 5, "r5", 24, 0x15, ASM_REG_W_R5},
	{ 6, "r6", 24, 0x16, ASM_REG_W_R6},
	{ 7, "r7", 24, 0x17, ASM_REG_W_R7},
	/* offset registers */
	{ 8, "n0", 24, 0x18, ASM_REG_W_N0},
	{ 9, "n1", 24, 0x19, ASM_REG_W_N1},
	{10, "n2", 24, 0x1a, ASM_REG_W_N2},
	{11, "n3", 24, 0x1b, ASM_REG_W_N3},
	{12, "n4", 24, 0x1c, ASM_REG_W_N4},
	{13, "n5", 24, 0x1d, ASM_REG_W_N5},
	{14, "n6", 24, 0x1e, ASM_REG_W_N6},
	{15, "n7", 24, 0x1f, ASM_REG_W_N7},
	/* modifier registers */
	{16, "m0", 24, 0x20, ASM_REG_W_M0},
	{17, "m1", 24, 0x21, ASM_REG_W_M1},
	{18, "m2", 24, 0x22, ASM_REG_W_M2},
	{19, "m3", 24, 0x23, ASM_REG_W_M3},
	{20, "m4", 24, 0x24, ASM_REG_W_M4},
	{21, "m5", 24, 0x25, ASM_REG_W_M5},
	{22, "m6", 24, 0x26, ASM_REG_W_M6},
	{23, "m7", 24, 0x27, ASM_REG_W_M7},
	/* data alu input register */
	{24, "x0", 24, 0x04, ASM_REG_W_X0},
	{25, "x1", 24, 0x05, ASM_REG_W_X1},
	{26, "y0", 24, 0x06, ASM_REG_W_Y0},
	{27, "y1", 24, 0x07, ASM_REG_W_Y1},
	/* data alu accumulator register */
	{28, "a0", 24, 0x08, ASM_REG_W_A0},
	{29, "a1", 24, 0x0c, ASM_REG_W_A1},
	{30, "a2",  8, 0x0a, ASM_REG_W_A2},
	{31, "b0", 24, 0x09, ASM_REG_W_B0},
	{32, "b1", 24, 0x0d, ASM_REG_W_B1},
	{33, "b2",  8, 0x0b, ASM_REG_W_B2},
	/* stack */
	{34, "ssh",24, 0x3c, ASM_REG_W_SSH},
	{35, "ssl",24, 0x3d, ASM_REG_W_SSL},
	{36, "sp", 24, 0x3b, ASM_REG_W_SP},
	{37, "ep", 24, 0x2a, ASM_REG_W_EP},
	{38, "sz", 24, 0x38, ASM_REG_W_SZ},
	{39, "sc", 24, 0x31, ASM_REG_W_SC},
	/* system */
	{40, "pc", 24, 0x00, ASM_REG_W_PC},
	{41, "sr", 24, 0x39, ASM_REG_W_SR},
	{42, "omr",24, 0x3a, ASM_REG_W_OMR},
	{43, "la", 24, 0x3e, ASM_REG_W_LA},
	{44, "lc", 24, 0x3f, ASM_REG_W_LC},
	/* interrupt */
	{45, "vba", 24, 0x30, ASM_REG_W_VBA},
	{46, "iprc",24, 0x00, ASM_REG_W_IPRC},
	{47, "iprp",24, 0x00, ASM_REG_W_IPRP},
	/* port a */
	{48, "bcr", 24, 0x00, ASM_REG_W_BCR},
	{49, "dcr", 24, 0x00, ASM_REG_W_DCR},
	{50, "aar0",24, 0x00, ASM_REG_W_AAR0},
	{51, "aar1",24, 0x00, ASM_REG_W_AAR1},
	{52, "aar2",24, 0x00, ASM_REG_W_AAR2},
	{53, "aar3",24, 0x00, ASM_REG_W_AAR3},
	/* *INDENT-ON* */
};

#define REG_NUM_R0	0
#define REG_NUM_R1	1
#define REG_NUM_N0	8
#define REG_NUM_N1	9
#define REG_NUM_M0	16
#define REG_NUM_M1	17
#define REG_NUM_SSH	34
#define REG_NUM_SSL	35
#define REG_NUM_SP	36
#define REG_NUM_EP	37
#define REG_NUM_SC	39
#define REG_NUM_PC	40
#define REG_NUM_SR	41
#define REG_NUM_IPRC	46
#define REG_NUM_IPRP	47
#define REG_NUM_BCR	48
#define REG_NUM_DCR	49
#define REG_NUM_AAR0	50
#define REG_NUM_AAR1	51
#define REG_NUM_AAR2	52
#define REG_NUM_AAR3	53

enum memory_type
{
	MEM_X = 0,
	MEM_Y = 1,
	MEM_P = 2,
};

#define INSTR_JUMP	0x0AF080
/* Effective Addressing Mode Encoding */
#define EAME_R0		0x10
/* instrcution encoder */
/* movep
 * s - peripheral space X/Y (X=0,Y=1)
 * w - write/read
 * d - source/destination register
 * p - IO short address
 */
#define INSTR_MOVEP_REG_HIO(s,w,d,p)   (0x084000 | ((s & 1)<<16) | ((w&1)<<15) | ((d & 0x3f)<<8) | (p & 0x3f))

static int dsp563xx_get_gdb_reg_list(struct target *target, struct reg **reg_list[], int *reg_list_size)
{
	int i;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}

	*reg_list_size = DSP563XX_NUMCOREREGS;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	if (!*reg_list)
		return ERROR_INVALID_ARGUMENTS;

	for (i = 0; i < DSP563XX_NUMCOREREGS; i++)
	{
		(*reg_list)[i] = &dsp563xx->core_cache->reg_list[i];
	}

	return ERROR_OK;

}

static int dsp563xx_read_core_reg(struct target *target, int num)
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

static int dsp563xx_write_core_reg(struct target *target, int num)
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

static int dsp563xx_target_create(struct target *target, Jim_Interp * interp)
{
	struct dsp563xx_common *dsp563xx = calloc(1, sizeof(struct dsp563xx_common));

	if (!dsp563xx)
		return ERROR_INVALID_ARGUMENTS;

	dsp563xx->jtag_info.tap = target->tap;
	target->arch_info = dsp563xx;
	dsp563xx->read_core_reg = dsp563xx_read_core_reg;
	dsp563xx->write_core_reg = dsp563xx_write_core_reg;

	return ERROR_OK;
}

static int dsp563xx_get_core_reg(struct reg *reg)
{
	struct dsp563xx_core_reg *dsp563xx_reg = reg->arch_info;
	struct target *target = dsp563xx_reg->target;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	LOG_DEBUG("%s", __FUNCTION__);

	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}

	return dsp563xx->read_core_reg(target, dsp563xx_reg->num);
}

static int dsp563xx_set_core_reg(struct reg *reg, uint8_t * buf)
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

static int dsp563xx_read_register(struct target *target, int num, int force);
static int dsp563xx_write_register(struct target *target, int num, int force);

static int dsp563xx_reg_read_high_io(struct target *target, uint32_t instr_mask, uint32_t * data)
{
	int err;
	uint32_t instr;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	/* we use r0 to store temporary data */
	if (!dsp563xx->core_cache->reg_list[REG_NUM_R0].valid)
		dsp563xx->read_core_reg(target, REG_NUM_R0);

	/* move source memory to r0 */
	instr = INSTR_MOVEP_REG_HIO(MEM_X, 0, EAME_R0, instr_mask);
	if ((err = dsp563xx_once_execute_sw_ir(target->tap, 0, instr)) != ERROR_OK)
		return err;
	/* move r0 to debug register */
	instr = INSTR_MOVEP_REG_HIO(MEM_X, 1, EAME_R0, 0xfffffc);
	if ((err = dsp563xx_once_execute_sw_ir(target->tap, 1, instr)) != ERROR_OK)
		return err;
	/* read debug register */
	if ((err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OGDBR, data)) != ERROR_OK)
		return err;
	/* r0 is no longer valid on target */
	dsp563xx->core_cache->reg_list[REG_NUM_R0].dirty = 1;

	return ERROR_OK;
}

static int dsp563xx_reg_write_high_io(struct target *target, uint32_t instr_mask, uint32_t data)
{
	int err;
	uint32_t instr;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	/* we use r0 to store temporary data */
	if (!dsp563xx->core_cache->reg_list[REG_NUM_R0].valid)
		dsp563xx->read_core_reg(target, REG_NUM_R0);

	/* move data to r0 */
	if ((err = dsp563xx_once_execute_dw_ir(target->tap, 0, 0x60F400, data)) != ERROR_OK)
		return err;
	/* move r0 to destination memory */
	instr = INSTR_MOVEP_REG_HIO(MEM_X, 1, EAME_R0, instr_mask);
	if ((err = dsp563xx_once_execute_sw_ir(target->tap, 1, instr)) != ERROR_OK)
		return err;

	/* r0 is no longer valid on target */
	dsp563xx->core_cache->reg_list[REG_NUM_R0].dirty = 1;

	return ERROR_OK;
}

static int dsp563xx_reg_read(struct target *target, uint32_t eame, uint32_t * data)
{
	int err;
	uint32_t instr;

	instr = INSTR_MOVEP_REG_HIO(MEM_X, 1, eame, 0xfffffc);
	if ((err = dsp563xx_once_execute_sw_ir(target->tap, 0, instr)) != ERROR_OK)
		return err;
	/* nop */
	if ((err = dsp563xx_once_execute_sw_ir(target->tap, 1, 0x000000)) != ERROR_OK)
		return err;
	/* read debug register */
	return dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OGDBR, data);
}

static int dsp563xx_reg_write(struct target *target, uint32_t instr_mask, uint32_t data)
{
	int err;

	if ((err = dsp563xx_once_execute_dw_ir(target->tap, 0, instr_mask, data)) != ERROR_OK)
		return err;
	/* nop */
	return dsp563xx_once_execute_sw_ir(target->tap, 1, 0x000000);
}

static int dsp563xx_reg_pc_read(struct target *target)
{
	int err;
	uint32_t opabdr, opabex;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	/* pc was changed, nothing todo */
	if (dsp563xx->core_cache->reg_list[REG_NUM_PC].dirty)
		return ERROR_OK;

	if ((err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OPABDR, &opabdr)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OPABEX, &opabex)) != ERROR_OK)
		return err;

	/* conditional branch check */
	if (opabdr == opabex)
	{
		/* TODO: check the trace buffer and if a
		 * conditional branch is detected then decode
		 * the branch command and add the relative
		 * address to the current pc
		 */
		LOG_DEBUG("%s conditional branch not supported yet", __FUNCTION__);
	}
	else
	{
		dsp563xx->core_regs[REG_NUM_PC] = opabex;
		dsp563xx->read_core_reg(target, REG_NUM_PC);
	}

	return ERROR_OK;
}

static int dsp563xx_reg_ssh_read(struct target *target)
{
	int err;
	uint32_t sp, sc, ep;
	struct dsp563xx_core_reg *arch_info;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	arch_info = dsp563xx->core_cache->reg_list[REG_NUM_SSH].arch_info;

	/* get a valid stack pointer */
	if ((err = dsp563xx_read_register(target, REG_NUM_SP, 0)) != ERROR_OK)
		return err;
	sp = dsp563xx->core_regs[REG_NUM_SP];
	if ((err = dsp563xx_write_register(target, REG_NUM_SP, 0)) != ERROR_OK)
		return err;

	/* get a valid stack count */
	if ((err = dsp563xx_read_register(target, REG_NUM_SC, 0)) != ERROR_OK)
		return err;
	sc = dsp563xx->core_regs[REG_NUM_SC];
	if ((err = dsp563xx_write_register(target, REG_NUM_SC, 0)) != ERROR_OK)
		return err;

	/* get a valid extended pointer */
	if ((err = dsp563xx_read_register(target, REG_NUM_EP, 0)) != ERROR_OK)
		return err;
	ep = dsp563xx->core_regs[REG_NUM_EP];
	if ((err = dsp563xx_write_register(target, REG_NUM_EP, 0)) != ERROR_OK)
		return err;

	if (!sp)
	{
		sp = 0x00FFFFFF;
	}
	else
	{
		if ((err = dsp563xx_reg_read(target, arch_info->eame, &sp)) != ERROR_OK)
			return err;

		if ((err = dsp563xx_write_register(target, REG_NUM_SC, 1)) != ERROR_OK)
			return err;
		if ((err = dsp563xx_write_register(target, REG_NUM_SP, 1)) != ERROR_OK)
			return err;
		if ((err = dsp563xx_write_register(target, REG_NUM_EP, 1)) != ERROR_OK)
			return err;
	}

	dsp563xx->core_regs[REG_NUM_SSH] = sp;
	dsp563xx->read_core_reg(target, REG_NUM_SSH);

	return ERROR_OK;
}

static int dsp563xx_reg_ssh_write(struct target *target)
{
	int err;
	uint32_t sp;
	struct dsp563xx_core_reg *arch_info;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	arch_info = dsp563xx->core_cache->reg_list[REG_NUM_SSH].arch_info;

	/* get a valid stack pointer */
	if ((err = dsp563xx_read_register(target, REG_NUM_SP, 0)) != ERROR_OK)
		return err;
	sp = dsp563xx->core_regs[REG_NUM_SP];

	if (sp)
	{
		sp--;
		/* write new stackpointer */
		dsp563xx->core_regs[REG_NUM_SP] = sp;
		if ((err = dsp563xx->read_core_reg(target, REG_NUM_SP)) != ERROR_OK)
			return err;
		if ((err = dsp563xx_write_register(target, REG_NUM_SP, 1)) != ERROR_OK)
			return err;

		if ((err = dsp563xx_reg_write(target, arch_info->instr_mask, dsp563xx->core_regs[REG_NUM_SSH])) != ERROR_OK)
			return err;

		if ((err = dsp563xx_read_register(target, REG_NUM_SP, 1)) != ERROR_OK)
			return err;
		if ((err = dsp563xx_read_register(target, REG_NUM_SSH, 1)) != ERROR_OK)
			return err;
	}

	return ERROR_OK;
}

static int dsp563xx_reg_ssl_read(struct target *target)
{
	int err;
	uint32_t sp;
	struct dsp563xx_core_reg *arch_info;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	arch_info = dsp563xx->core_cache->reg_list[REG_NUM_SSL].arch_info;

	/* get a valid stack pointer */
	if ((err = dsp563xx_read_register(target, REG_NUM_SP, 0)) != ERROR_OK)
		return err;
	sp = dsp563xx->core_regs[REG_NUM_SP];

	if (!sp)
	{
		sp = 0x00FFFFFF;
	}
	else
	{
		if ((err = dsp563xx_reg_read(target, arch_info->eame, &sp)) != ERROR_OK)
			return err;
	}

	dsp563xx->core_regs[REG_NUM_SSL] = sp;
	dsp563xx->read_core_reg(target, REG_NUM_SSL);

	return ERROR_OK;
}

static int dsp563xx_read_register(struct target *target, int num, int force)
{
	int err = ERROR_OK;
	uint32_t data = 0;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	struct dsp563xx_core_reg *arch_info;

	if (force)
		dsp563xx->core_cache->reg_list[num].valid = 0;

	if (!dsp563xx->core_cache->reg_list[num].valid)
	{
		arch_info = dsp563xx->core_cache->reg_list[num].arch_info;

		switch (arch_info->num)
		{
			case REG_NUM_SSH:
				err = dsp563xx_reg_ssh_read(target);
				break;
			case REG_NUM_SSL:
				err = dsp563xx_reg_ssl_read(target);
				break;
			case REG_NUM_PC:
				err = dsp563xx_reg_pc_read(target);
				break;
			case REG_NUM_IPRC:
			case REG_NUM_IPRP:
			case REG_NUM_BCR:
			case REG_NUM_DCR:
			case REG_NUM_AAR0:
			case REG_NUM_AAR1:
			case REG_NUM_AAR2:
			case REG_NUM_AAR3:
				err = dsp563xx_reg_read_high_io(target, arch_info->instr_mask, &data);
				if (err == ERROR_OK)
				{
					dsp563xx->core_regs[num] = data;
					dsp563xx->read_core_reg(target, num);
				}
				break;
			default:
				err = dsp563xx_reg_read(target, arch_info->eame, &data);
				if (err == ERROR_OK)
				{
					dsp563xx->core_regs[num] = data;
					dsp563xx->read_core_reg(target, num);
				}
				break;
		}

	}

	return err;
}

static int dsp563xx_write_register(struct target *target, int num, int force)
{
	int err = ERROR_OK;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	struct dsp563xx_core_reg *arch_info;

	if (force)
		dsp563xx->core_cache->reg_list[num].dirty = 1;

	if (dsp563xx->core_cache->reg_list[num].dirty)
	{
		arch_info = dsp563xx->core_cache->reg_list[num].arch_info;

		dsp563xx->write_core_reg(target, num);

		switch (arch_info->num)
		{
			case REG_NUM_SSH:
				err = dsp563xx_reg_ssh_write(target);
				break;
			case REG_NUM_PC:
				/* pc is updated on resume, no need to write it here */
				break;
			case REG_NUM_IPRC:
			case REG_NUM_IPRP:
			case REG_NUM_BCR:
			case REG_NUM_DCR:
			case REG_NUM_AAR0:
			case REG_NUM_AAR1:
			case REG_NUM_AAR2:
			case REG_NUM_AAR3:
				err = dsp563xx_reg_write_high_io(target, arch_info->instr_mask, dsp563xx->core_regs[num]);
				break;
			default:
				err = dsp563xx_reg_write(target, arch_info->instr_mask, dsp563xx->core_regs[num]);

				if ((err == ERROR_OK) && (arch_info->num == REG_NUM_SP))
				{
					dsp563xx->core_cache->reg_list[REG_NUM_SSH].valid = 0;
					dsp563xx->core_cache->reg_list[REG_NUM_SSL].valid = 0;
				}

				break;
		}
	}

	return err;
}

static int dsp563xx_save_context(struct target *target)
{
	int i, err = ERROR_OK;

	for (i = 0; i < DSP563XX_NUMCOREREGS; i++)
	{
		if ((err = dsp563xx_read_register(target, i, 0)) != ERROR_OK)
			break;
	}

	return err;
}

static int dsp563xx_restore_context(struct target *target)
{
	int i, err = ERROR_OK;

	for (i = 0; i < DSP563XX_NUMCOREREGS; i++)
	{
		if ((err = dsp563xx_write_register(target, i, 0)) != ERROR_OK)
			break;
	}

	return err;
}

static const struct reg_arch_type dsp563xx_reg_type = {
	.get = dsp563xx_get_core_reg,
	.set = dsp563xx_set_core_reg,
};

static int dsp563xx_init_target(struct command_context *cmd_ctx, struct target *target)
{
	/* get pointers to arch-specific information */
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = malloc(sizeof(struct reg) * DSP563XX_NUMCOREREGS);
	struct dsp563xx_core_reg *arch_info = malloc(sizeof(struct dsp563xx_core_reg) * DSP563XX_NUMCOREREGS);
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
		arch_info[i].eame = dsp563xx_regs[i].eame;
		arch_info[i].instr_mask = dsp563xx_regs[i].instr_mask;
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

static int dsp563xx_arch_state(struct target *target)
{
	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

#define DSP563XX_SR_SA	(1<<17)
#define DSP563XX_SR_SC	(1<<13)

static int dsp563xx_debug_once_init(struct target *target)
{
	return dsp563xx_once_read_register(target->tap, 1, once_regs, DSP563XX_NUMONCEREGS);
}

static int dsp563xx_debug_init(struct target *target)
{
	int err;
	uint32_t sr;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	struct dsp563xx_core_reg *arch_info;

	if ((err = dsp563xx_debug_once_init(target)) != ERROR_OK)
		return err;

	arch_info = dsp563xx->core_cache->reg_list[REG_NUM_SR].arch_info;

	/* check 24bit mode */
	if ((err = dsp563xx_read_register(target, REG_NUM_SR, 0)) != ERROR_OK)
		return err;

	sr = dsp563xx->core_regs[REG_NUM_SR];

	if (sr & (DSP563XX_SR_SA | DSP563XX_SR_SC))
	{
		sr &= ~(DSP563XX_SR_SA | DSP563XX_SR_SC);

		if ((err = dsp563xx_once_execute_dw_ir(target->tap, 1, arch_info->instr_mask, sr)) != ERROR_OK)
			return err;
		dsp563xx->core_cache->reg_list[REG_NUM_SR].dirty = 1;
	}

	if ((err = dsp563xx_read_register(target, REG_NUM_N0, 0)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_read_register(target, REG_NUM_N1, 0)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_read_register(target, REG_NUM_M0, 0)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_read_register(target, REG_NUM_M1, 0)) != ERROR_OK)
		return err;

	if (dsp563xx->core_regs[REG_NUM_N0] != 0x000000)
	{
		arch_info = dsp563xx->core_cache->reg_list[REG_NUM_N0].arch_info;
		if ((err = dsp563xx_reg_write(target, arch_info->instr_mask, 0x000000)) != ERROR_OK)
			return err;
	}
	dsp563xx->core_cache->reg_list[REG_NUM_N0].dirty = 1;

	if (dsp563xx->core_regs[REG_NUM_N1] != 0x000000)
	{
		arch_info = dsp563xx->core_cache->reg_list[REG_NUM_N1].arch_info;
		if ((err = dsp563xx_reg_write(target, arch_info->instr_mask, 0x000000)) != ERROR_OK)
			return err;
	}
	dsp563xx->core_cache->reg_list[REG_NUM_N1].dirty = 1;

	if (dsp563xx->core_regs[REG_NUM_M0] != 0xffffff)
	{
		arch_info = dsp563xx->core_cache->reg_list[REG_NUM_M0].arch_info;
		if ((err = dsp563xx_reg_write(target, arch_info->instr_mask, 0xffffff)) != ERROR_OK)
			return err;
	}
	dsp563xx->core_cache->reg_list[REG_NUM_M0].dirty = 1;

	if (dsp563xx->core_regs[REG_NUM_M1] != 0xffffff)
	{
		arch_info = dsp563xx->core_cache->reg_list[REG_NUM_M1].arch_info;
		if ((err = dsp563xx_reg_write(target, arch_info->instr_mask, 0xffffff)) != ERROR_OK)
			return err;
	}
	dsp563xx->core_cache->reg_list[REG_NUM_M1].dirty = 1;

	if ((err = dsp563xx_save_context(target)) != ERROR_OK)
		return err;

	return ERROR_OK;
}

static int dsp563xx_jtag_debug_request(struct target *target)
{
	return dsp563xx_once_request_debug(target->tap, target->state == TARGET_RESET);
}

static int dsp563xx_poll(struct target *target)
{
	int err;
	uint32_t once_status;
	int state;

	state = dsp563xx_once_target_status(target->tap);

	if (state == TARGET_UNKNOWN)
	{
		target->state = state;
		LOG_ERROR("jtag status contains invalid mode value - communication failure");
		return ERROR_TARGET_FAILURE;
	}

	if ((err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OSCR, &once_status)) != ERROR_OK)
		return err;

	if ((once_status & DSP563XX_ONCE_OSCR_DEBUG_M) == DSP563XX_ONCE_OSCR_DEBUG_M)
	{
		if (target->state != TARGET_HALTED)
		{
			target->state = TARGET_HALTED;
			if ((err = dsp563xx_debug_init(target)) != ERROR_OK)
				return err;

			LOG_DEBUG("target->state: %s", target_state_name(target));
		}
	}

	return ERROR_OK;
}

static int dsp563xx_halt(struct target *target)
{
	int err;
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

	if ((err = dsp563xx_jtag_debug_request(target)) != ERROR_OK)
		return err;

	/* store pipeline register */
	if ((err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OPILR, &dsp563xx->pipeline_context.once_opilr)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OPDBR, &dsp563xx->pipeline_context.once_opdbr)) != ERROR_OK)
		return err;

	LOG_DEBUG("%s", __FUNCTION__);

	return ERROR_OK;
}

static int dsp563xx_resume(struct target *target, int current, uint32_t address, int handle_breakpoints, int debug_execution)
{
	int err;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	LOG_DEBUG("%s", __FUNCTION__);

	if ((err = dsp563xx_restore_context(target)) != ERROR_OK)
		return err;
	register_cache_invalidate(dsp563xx->core_cache);

	if (current)
	{
		/* restore pipeline registers and go */
		if ((err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OPILR, dsp563xx->pipeline_context.once_opilr)) != ERROR_OK)
			return err;
		if ((err =
		     dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OPDBR | DSP563XX_ONCE_OCR_EX | DSP563XX_ONCE_OCR_GO,
					     dsp563xx->pipeline_context.once_opdbr)) != ERROR_OK)
			return err;
	}
	else
	{
		/* set to go register and jump */
		if ((err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OPDBR, INSTR_JUMP)) != ERROR_OK)
			return err;
		if ((err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_PDBGOTO | DSP563XX_ONCE_OCR_EX | DSP563XX_ONCE_OCR_GO, address)) != ERROR_OK)
			return err;
	}

	target->state = TARGET_RUNNING;

	return ERROR_OK;
}

static int dsp563xx_step_ex(struct target *target, int current, uint32_t address, int handle_breakpoints, int steps)
{
	int err;
	uint32_t once_status;
	uint32_t dr_in, cnt;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);

	if (target->state != TARGET_HALTED)
	{
		LOG_DEBUG("target was not halted");
		return ERROR_OK;
	}

	LOG_DEBUG("%s %08X %08X", __FUNCTION__, current, (unsigned) address);

	if ((err = dsp563xx_jtag_debug_request(target)) != ERROR_OK)
		return err;
	if ((err = dsp563xx_restore_context(target)) != ERROR_OK)
		return err;

	/* reset trace mode */
	if ((err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OSCR, 0x000000)) != ERROR_OK)
		return err;
	/* enable trace mode */
	if ((err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OSCR, DSP563XX_ONCE_OSCR_TME)) != ERROR_OK)
		return err;

	cnt = steps;

	/* on JUMP we need one extra cycle */
	if (!current)
		cnt++;

	/* load step counter with N-1 */
	if ((err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OTC, cnt)) != ERROR_OK)
		return err;

	if (current)
	{
		/* restore pipeline registers and go */
		if ((err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OPILR, dsp563xx->pipeline_context.once_opilr)) != ERROR_OK)
			return err;
		if ((err =
		     dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OPDBR | DSP563XX_ONCE_OCR_EX | DSP563XX_ONCE_OCR_GO,
					     dsp563xx->pipeline_context.once_opdbr)) != ERROR_OK)
			return err;
	}
	else
	{
		/* set to go register and jump */
		if ((err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OPDBR, INSTR_JUMP)) != ERROR_OK)
			return err;
		if ((err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_PDBGOTO | DSP563XX_ONCE_OCR_EX | DSP563XX_ONCE_OCR_GO, address)) != ERROR_OK)
			return err;
	}

	while (1)
	{
		if ((err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OSCR, &once_status)) != ERROR_OK)
			return err;

		if (once_status & DSP563XX_ONCE_OSCR_TO)
		{
			/* store pipeline register */
			if ((err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OPILR, &dsp563xx->pipeline_context.once_opilr)) != ERROR_OK)
				return err;
			if ((err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OPDBR, &dsp563xx->pipeline_context.once_opdbr)) != ERROR_OK)
				return err;

			if ((err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OPABFR, &dr_in)) != ERROR_OK)
				return err;
			LOG_DEBUG("fetch: %08X", (unsigned) dr_in);
			if ((err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OPABDR, &dr_in)) != ERROR_OK)
				return err;
			LOG_DEBUG("decode: %08X", (unsigned) dr_in);
			if ((err = dsp563xx_once_reg_read(target->tap, 1, DSP563XX_ONCE_OPABEX, &dr_in)) != ERROR_OK)
				return err;
			LOG_DEBUG("execute: %08X", (unsigned) dr_in);

			/* reset trace mode */
			if ((err = dsp563xx_once_reg_write(target->tap, 1, DSP563XX_ONCE_OSCR, 0x000000)) != ERROR_OK)
				return err;

			register_cache_invalidate(dsp563xx->core_cache);
			if ((err = dsp563xx_debug_init(target)) != ERROR_OK)
				return err;

			break;
		}
	}

	return ERROR_OK;
}

static int dsp563xx_step(struct target *target, int current, uint32_t address, int handle_breakpoints)
{
	return dsp563xx_step_ex(target, current, address, handle_breakpoints, 0);
}

static int dsp563xx_assert_reset(struct target *target)
{
	int retval = 0;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (jtag_reset_config & RESET_HAS_SRST)
	{
		/* default to asserting srst */
		if (jtag_reset_config & RESET_SRST_PULLS_TRST)
		{
			jtag_add_reset(1, 1);
		}
		else
		{
			jtag_add_reset(0, 1);
		}
	}

	target->state = TARGET_RESET;
	jtag_add_sleep(5000);

	/* registers are now invalid */
	register_cache_invalidate(dsp563xx->core_cache);

	if (target->reset_halt)
	{
		if ((retval = target_halt(target)) != ERROR_OK)
			return retval;
	}

	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

static int dsp563xx_deassert_reset(struct target *target)
{
	int err;

	/* deassert reset lines */
	jtag_add_reset(0, 0);

	if ((err = dsp563xx_poll(target)) != ERROR_OK)
		return err;

	if (target->reset_halt)
	{
		if (target->state == TARGET_HALTED)
		{
			/* after a reset the cpu jmp to the
			 * reset vector and need 2 cycles to fill
			 * the cache (fetch,decode,excecute)
			 */
			if ((err = dsp563xx_step_ex(target, 1, 0, 1, 1)) != ERROR_OK)
				return err;
		}
	}

//      target->state = TARGET_RUNNING;

	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

static int dsp563xx_soft_reset_halt(struct target *target)
{
	LOG_DEBUG("%s", __FUNCTION__);
	return ERROR_OK;
}

static int dsp563xx_read_memory(struct target *target, int mem_type, uint32_t address, uint32_t size, uint32_t count, uint8_t * buffer)
{
	int err;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	uint32_t i, x;
	uint32_t data, move_cmd;
	uint8_t *b;

	LOG_DEBUG("address: 0x%8.8" PRIx32 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32, address, size, count);

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* we only support 4 byte aligned data */
	if ( size != 4 )
	{
		return ERROR_INVALID_ARGUMENTS;
	}

	switch (mem_type)
	{
		case MEM_X:
			/* TODO: mark effected queued registers */
			move_cmd = 0x61d800;
			break;
		case MEM_Y:
			move_cmd = 0x69d800;
			break;
		case MEM_P:
			move_cmd = 0x07d891;
			break;
		default:
			return ERROR_INVALID_ARGUMENTS;
	}

	/* we use r0 to store temporary data */
	if (!dsp563xx->core_cache->reg_list[REG_NUM_R0].valid)
		dsp563xx->read_core_reg(target, REG_NUM_R0);
	/* we use r1 to store temporary data */
	if (!dsp563xx->core_cache->reg_list[REG_NUM_R1].valid)
		dsp563xx->read_core_reg(target, REG_NUM_R1);

	/* r0 is no longer valid on target */
	dsp563xx->core_cache->reg_list[REG_NUM_R0].dirty = 1;
	/* r1 is no longer valid on target */
	dsp563xx->core_cache->reg_list[REG_NUM_R1].dirty = 1;

	x = count;
	b = buffer;

	if ((err = dsp563xx_once_execute_dw_ir(target->tap, 1, 0x60F400, address)) != ERROR_OK)
		return err;

	for (i = 0; i < x; i++)
	{
		if ((err = dsp563xx_once_execute_sw_ir(target->tap, 0, move_cmd)) != ERROR_OK)
			return err;
		if ((err = dsp563xx_once_execute_sw_ir(target->tap, 0, 0x08D13C)) != ERROR_OK)
			return err;
		if ((err = dsp563xx_once_reg_read(target->tap, 0, DSP563XX_ONCE_OGDBR, (uint32_t*)b)) != ERROR_OK)
			return err;
		b += 4;
	}

	/* flush the jtag queue */
	if ((err = jtag_execute_queue()) != ERROR_OK)
	{
		return err;
	}

	/* walk over the buffer and fix target endianness */
	b = buffer;

	for (i = 0; i < x; i++)
	{
		data = *((uint32_t*)b) & 0x00FFFFFF;
//		LOG_DEBUG("R: %08X", *((uint32_t*)b));
		target_buffer_set_u32(target, b, data);
		b += 4;
	}

	return ERROR_OK;
}

static int dsp563xx_read_memory_p(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t * buffer)
{
	return dsp563xx_read_memory(target, MEM_P, address, size, count, buffer);
}

static int dsp563xx_write_memory(struct target *target, int mem_type, uint32_t address, uint32_t size, uint32_t count, uint8_t * buffer)
{
	int err;
	struct dsp563xx_common *dsp563xx = target_to_dsp563xx(target);
	uint32_t i, x;
	uint32_t data, move_cmd;
	uint8_t *b;

	LOG_DEBUG("address: 0x%8.8" PRIx32 ", size: 0x%8.8" PRIx32 ", count: 0x%8.8" PRIx32 "", address, size, count);

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* we only support 4 byte aligned data */
	if ( size != 4 )
	{
		return ERROR_INVALID_ARGUMENTS;
	}

	switch (mem_type)
	{
		case MEM_X:
			move_cmd = 0x615800;
			break;
		case MEM_Y:
			move_cmd = 0x695800;
			break;
		case MEM_P:
			move_cmd = 0x075891;
			break;
		default:
			return ERROR_INVALID_ARGUMENTS;
	}

	/* we use r0 to store temporary data */
	if (!dsp563xx->core_cache->reg_list[REG_NUM_R0].valid)
		dsp563xx->read_core_reg(target, REG_NUM_R0);
	/* we use r1 to store temporary data */
	if (!dsp563xx->core_cache->reg_list[REG_NUM_R1].valid)
		dsp563xx->read_core_reg(target, REG_NUM_R1);

	/* r0 is no longer valid on target */
	dsp563xx->core_cache->reg_list[REG_NUM_R0].dirty = 1;
	/* r1 is no longer valid on target */
	dsp563xx->core_cache->reg_list[REG_NUM_R1].dirty = 1;

	x = count;
	b = buffer;

	if ((err = dsp563xx_once_execute_dw_ir(target->tap, 1, 0x60F400, address)) != ERROR_OK)
		return err;

	for (i = 0; i < x; i++)
	{
		data = target_buffer_get_u32(target, b);

//		LOG_DEBUG("W: %08X", data);

		data &= 0x00ffffff;

		if ((err = dsp563xx_once_execute_dw_ir(target->tap, 0, 0x61F400, data)) != ERROR_OK)
			return err;
		if ((err = dsp563xx_once_execute_sw_ir(target->tap, 0, move_cmd)) != ERROR_OK)
			return err;
		b += 4;
	}

	/* flush the jtag queue */
	if ((err = jtag_execute_queue()) != ERROR_OK)
	{
		return err;
	}

	return ERROR_OK;
}

static int dsp563xx_write_memory_p(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t * buffer)
{
	return dsp563xx_write_memory(target, MEM_P, address, size, count, buffer);
}

static int dsp563xx_bulk_write_memory_p(struct target *target, uint32_t address, uint32_t count, uint8_t *buffer)
{
	return dsp563xx_write_memory(target, MEM_P, address, 4, count, buffer);
}

static void handle_md_output(struct command_context *cmd_ctx, struct target *target, uint32_t address, unsigned size, unsigned count, const uint8_t * buffer)
{
	const unsigned line_bytecnt = 32;
	unsigned line_modulo = line_bytecnt / size;

	char output[line_bytecnt * 4 + 1];
	unsigned output_len = 0;

	const char *value_fmt;
	switch (size)
	{
		case 4:
			value_fmt = "%8.8x ";
			break;
		case 2:
			value_fmt = "%4.4x ";
			break;
		case 1:
			value_fmt = "%2.2x ";
			break;
		default:
			/* "can't happen", caller checked */
			LOG_ERROR("invalid memory read size: %u", size);
			return;
	}

	for (unsigned i = 0; i < count; i++)
	{
		if (i % line_modulo == 0)
		{
			output_len += snprintf(output + output_len, sizeof(output) - output_len, "0x%8.8x: ", (unsigned) (address + (i * size)));
		}

		uint32_t value = 0;
		const uint8_t *value_ptr = buffer + i * size;
		switch (size)
		{
			case 4:
				value = target_buffer_get_u32(target, value_ptr);
				break;
			case 2:
				value = target_buffer_get_u16(target, value_ptr);
				break;
			case 1:
				value = *value_ptr;
		}
		output_len += snprintf(output + output_len, sizeof(output) - output_len, value_fmt, value);

		if ((i % line_modulo == line_modulo - 1) || (i == count - 1))
		{
			command_print(cmd_ctx, "%s", output);
			output_len = 0;
		}
	}
}

COMMAND_HANDLER(dsp563xx_mem_command)
{
	struct target *target = get_current_target(CMD_CTX);
	int err = ERROR_OK;
	int read_mem;
	uint32_t address = 0;
	uint32_t count = 1, i;
	uint32_t pattern = 0;
	uint32_t mem_type;
	uint8_t *buffer, *b;

	switch (CMD_NAME[1])
	{
		case 'w':
			read_mem = 0;
			break;
		case 'd':
			read_mem = 1;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	switch (CMD_NAME[3])
	{
		case 'x':
			mem_type = MEM_X;
			break;
		case 'y':
			mem_type = MEM_Y;
			break;
		case 'p':
			mem_type = MEM_P;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (CMD_ARGC > 0)
	{
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);
	}

	if (read_mem == 0)
	{
		if (CMD_ARGC < 2)
		{
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (CMD_ARGC > 1)
		{
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], pattern);
		}
		if (CMD_ARGC > 2)
		{
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], count);
		}
	}

	if (read_mem == 1)
	{
		if (CMD_ARGC < 1)
		{
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (CMD_ARGC > 1)
		{
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], count);
		}
	}

	buffer = calloc(count, sizeof(uint32_t));

	if (read_mem == 1)
	{
		if ((err = dsp563xx_read_memory(target, mem_type, address, sizeof(uint32_t), count, buffer)) == ERROR_OK)
			handle_md_output(CMD_CTX, target, address, sizeof(uint32_t), count, buffer);
	}
	else
	{
		b = buffer;

		for (i = 0; i < count; i++)
		{
			target_buffer_set_u32(target, b, pattern);
			b += 4;
		}

		err = dsp563xx_write_memory(target, mem_type, address, sizeof(uint32_t), count, buffer);
	}

	free(buffer);

	return err;
}

static const struct command_registration dsp563xx_command_handlers[] = {
	{
	 .name = "mwwx",
	 .handler = dsp563xx_mem_command,
	 .mode = COMMAND_EXEC,
	 .help = "write x memory words",
	 .usage = "mwwx address value [count]",
	 },
	{
	 .name = "mwwy",
	 .handler = dsp563xx_mem_command,
	 .mode = COMMAND_EXEC,
	 .help = "write y memory words",
	 .usage = "mwwy address value [count]",
	 },
	{
	 .name = "mwwp",
	 .handler = dsp563xx_mem_command,
	 .mode = COMMAND_EXEC,
	 .help = "write p memory words",
	 .usage = "mwwp address value [count]",
	 },
	{
	 .name = "mdwx",
	 .handler = dsp563xx_mem_command,
	 .mode = COMMAND_EXEC,
	 .help = "display x memory words",
	 .usage = "mdwx address [count]",
	 },
	{
	 .name = "mdwy",
	 .handler = dsp563xx_mem_command,
	 .mode = COMMAND_EXEC,
	 .help = "display y memory words",
	 .usage = "mdwy address [count]",
	 },
	{
	 .name = "mdwp",
	 .handler = dsp563xx_mem_command,
	 .mode = COMMAND_EXEC,
	 .help = "display p memory words",
	 .usage = "mdwp address [count]",
	 },
	COMMAND_REGISTRATION_DONE
};

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
	.bulk_write_memory = dsp563xx_bulk_write_memory_p,

	.commands = dsp563xx_command_handlers,
	.target_create = dsp563xx_target_create,
	.init_target = dsp563xx_init_target,
};
