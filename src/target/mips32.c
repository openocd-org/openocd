// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2011 by Drasko DRASKOVIC                                *
 *   drasko.draskovic@gmail.com                                            *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "mips32.h"
#include "mips_cpu.h"
#include "breakpoints.h"
#include "algorithm.h"
#include "register.h"

static const char *mips_isa_strings[] = {
	"MIPS32", "MIPS16", "", "MICRO MIPS32",
};

#define MIPS32_GDB_FP_REG 1

/*
 * GDB registers
 * based on gdb-7.6.2/gdb/features/mips-{fpu,cp0,cpu,dsp}.xml
 */
static const struct {
	unsigned int id;
	const char *name;
	enum reg_type type;
	const char *group;
	const char *feature;
	int size;
} mips32_regs[] = {
	{  0,  "r0", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  1,  "r1", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  2,  "r2", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  3,  "r3", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  4,  "r4", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  5,  "r5", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  6,  "r6", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  7,  "r7", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  8,  "r8", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{  9,  "r9", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 10, "r10", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 11, "r11", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 12, "r12", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 13, "r13", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 14, "r14", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 15, "r15", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 16, "r16", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 17, "r17", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 18, "r18", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 19, "r19", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 20, "r20", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 21, "r21", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 22, "r22", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 23, "r23", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 24, "r24", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 25, "r25", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 26, "r26", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 27, "r27", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 28, "r28", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 29, "r29", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 30, "r30", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 31, "r31", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 32,  "lo", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },
	{ 33,  "hi", REG_TYPE_INT, NULL, "org.gnu.gdb.mips.cpu", 0 },

	{ MIPS32_REGLIST_FP_INDEX + 0,  "f0", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 1,  "f1", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 2,  "f2", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 3,  "f3", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 4,  "f4", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 5,  "f5", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 6,  "f6", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 7,  "f7", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 8,  "f8", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 9,  "f9", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 10, "f10", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 11, "f11", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 12, "f12", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 13, "f13", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 14, "f14", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 15, "f15", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 16, "f16", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 17, "f17", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 18, "f18", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 19, "f19", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 20, "f20", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 21, "f21", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 22, "f22", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 23, "f23", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 24, "f24", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 25, "f25", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 26, "f26", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 27, "f27", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 28, "f28", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 29, "f29", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 30, "f30", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },
	{ MIPS32_REGLIST_FP_INDEX + 31, "f31", REG_TYPE_IEEE_DOUBLE, NULL,
		"org.gnu.gdb.mips.fpu", MIPS32_GDB_FP_REG },

	{ MIPS32_REGLIST_FPC_INDEX + 0, "fcsr", REG_TYPE_INT, "float",
		"org.gnu.gdb.mips.fpu", 0 },
	{ MIPS32_REGLIST_FPC_INDEX + 1, "fir", REG_TYPE_INT, "float",
		"org.gnu.gdb.mips.fpu", 0 },

	{ MIPS32_REGLIST_C0_STATUS_INDEX,	"status", REG_TYPE_INT, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS32_REGLIST_C0_BADVADDR_INDEX,	"badvaddr", REG_TYPE_INT, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS32_REGLIST_C0_CAUSE_INDEX,	"cause", REG_TYPE_INT, NULL,
		"org.gnu.gdb.mips.cp0", 0 },
	{ MIPS32_REGLIST_C0_PC_INDEX,		"pc", REG_TYPE_INT, NULL,
		"org.gnu.gdb.mips.cpu", 0 },
	{ MIPS32_REGLIST_C0_GUESTCTL1_INDEX, "guestCtl1", REG_TYPE_INT, NULL,
		"org.gnu.gdb.mips.cp0", 0 },

	{ MIPS32_REGLIST_DSP_INDEX + 0, "hi1", REG_TYPE_INT, NULL,
		"org.gnu.gdb.mips.dsp", 0 },
	{ MIPS32_REGLIST_DSP_INDEX + 1, "lo1", REG_TYPE_INT, NULL,
		"org.gnu.gdb.mips.dsp", 0 },
	{ MIPS32_REGLIST_DSP_INDEX + 2, "hi2", REG_TYPE_INT, NULL,
		"org.gnu.gdb.mips.dsp", 0 },
	{ MIPS32_REGLIST_DSP_INDEX + 3, "lo2", REG_TYPE_INT, NULL,
		"org.gnu.gdb.mips.dsp", 0 },
	{ MIPS32_REGLIST_DSP_INDEX + 4, "hi3", REG_TYPE_INT, NULL,
		"org.gnu.gdb.mips.dsp", 0 },
	{ MIPS32_REGLIST_DSP_INDEX + 5, "lo3", REG_TYPE_INT, NULL,
		"org.gnu.gdb.mips.dsp", 0 },

	{ MIPS32_REGLIST_DSP_DSPCTL_INDEX, "dspctl", REG_TYPE_INT, NULL,
		"org.gnu.gdb.mips.dsp", 0 },
};

#define MIPS32_NUM_REGS ARRAY_SIZE(mips32_regs)



#define zero	0

#define AT	1

#define v0	2
#define v1	3

#define a0	4
#define a1	5
#define a2	6
#define	a3	7
#define t0	8
#define t1	9
#define t2	10
#define t3	11
#define t4	12
#define t5	13
#define t6	14
#define t7	15
#define ta0	12	/* alias for $t4 */
#define ta1	13	/* alias for $t5 */
#define ta2	14	/* alias for $t6 */
#define ta3	15	/* alias for $t7 */

#define s0	16
#define s1	17
#define s2	18
#define s3	19
#define s4	20
#define s5	21
#define s6	22
#define s7	23
#define s8	30		/* == fp */

#define t8	24
#define t9	25
#define k0	26
#define k1	27

#define gp	28

#define sp	29
#define fp	30
#define ra	31


static const struct {
	const char *name;
} mips32_dsp_regs[MIPS32NUMDSPREGS] = {
	{ "hi1"},
	{ "lo1"},
	{ "hi2"},
	{ "lo2"},
	{ "hi3"},
	{ "lo3"},
	{ "control"},
};

static int mips32_get_core_reg(struct reg *reg)
{
	int retval;
	struct mips32_core_reg *mips32_reg = reg->arch_info;
	struct target *target = mips32_reg->target;
	struct mips32_common *mips32_target = target_to_mips32(target);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = mips32_target->read_core_reg(target, mips32_reg->num);

	return retval;
}

static int mips32_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct mips32_core_reg *mips32_reg = reg->arch_info;
	struct target *target = mips32_reg->target;
	uint64_t value;

	if (reg->size == 64)
		value = buf_get_u64(buf, 0, 64);
	else
		value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (reg->size == 64)
		buf_set_u64(reg->value, 0, 64, value);
	else
		buf_set_u32(reg->value, 0, 32, value);

	reg->dirty = true;
	reg->valid = true;

	return ERROR_OK;
}

/**
 * mips32_set_all_fpr_width - Set the width of all floating-point registers
 * @param[in] mips32: MIPS32 common structure
 * @param[in] fp64: Flag indicating whether to set the width to 64 bits (double precision)
 *
 * @brief Sets the width of all floating-point registers based on the specified flag.
 */
static void mips32_set_all_fpr_width(struct mips32_common *mips32, bool fp64)
{
	struct reg_cache *cache = mips32->core_cache;
	struct reg *reg_list = cache->reg_list;
	int i;

	for (i = MIPS32_REGLIST_FP_INDEX; i < (MIPS32_REGLIST_FP_INDEX + MIPS32_REG_FP_COUNT); i++) {
		reg_list[i].size = fp64 ? 64 : 32;
		reg_list[i].reg_data_type->type = fp64 ? REG_TYPE_IEEE_DOUBLE : REG_TYPE_IEEE_SINGLE;
	}
}

/**
 * mips32_detect_fpr_mode_change - Detect changes in floating-point register mode
 * @param[in] mips32: MIPS32 common structure
 * @param[in] cp0_status: Value of the CP0 status register
 *
 * @brief Detects changes in the floating-point register mode based on the CP0 status register.
 * If changes are detected, it updates the internal state
 * and logs a warning message indicating the mode change.
 */
static void mips32_detect_fpr_mode_change(struct mips32_common *mips32, uint32_t cp0_status)
{
	if (!mips32->fp_imp)
		return;

	/* CP0.Status.FR indicates the working mode of floating-point register.
	 * When FP = 0, fpr can contain any 32bit data type,
	 * 64bit data types are stored in even-odd register pairs.
	 * When FP = 1, fpr can contain any data types.*/
	bool fpu_in_64bit = ((cp0_status & BIT(MIPS32_CP0_STATUS_FR_SHIFT)) != 0);

	/* CP0.Status.CU1 indicated whether CoProcessor1(which is FPU) is present. */
	bool fp_enabled = ((cp0_status & BIT(MIPS32_CP0_STATUS_CU1_SHIFT)) != 0);

	if (mips32->fpu_in_64bit != fpu_in_64bit) {
		mips32->fpu_in_64bit = fpu_in_64bit;
		mips32_set_all_fpr_width(mips32, fpu_in_64bit);
		LOG_WARNING("** FP mode changed to %sbit, you must reconnect GDB **", fpu_in_64bit ? "64" : "32");
	}

	if (mips32->fpu_enabled != fp_enabled) {
		mips32->fpu_enabled = fp_enabled;
		const char *s = fp_enabled ? "enabled" : "disabled";
		LOG_WARNING("** FP is %s, register update %s **", s, s);
	}
}

static int mips32_read_core_reg(struct target *target, unsigned int num)
{
	unsigned int cnum;
	uint64_t reg_value = 0;

	/* get pointers to arch-specific information */
	struct mips32_common *mips32 = target_to_mips32(target);

	if (num >= MIPS32_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (num >= MIPS32_REGLIST_DSP_INDEX) {
		/* DSP */
		cnum = num - MIPS32_REGLIST_DSP_INDEX;
		reg_value = mips32->core_regs.dsp[cnum];
		buf_set_u32(mips32->core_cache->reg_list[num].value, 0, 32, reg_value);
	} else if (num >= MIPS32_REGLIST_C0_INDEX) {
		/* CP0 */
		cnum = num - MIPS32_REGLIST_C0_INDEX;
		reg_value = mips32->core_regs.cp0[cnum];
		buf_set_u32(mips32->core_cache->reg_list[num].value, 0, 32, reg_value);
		if (cnum == MIPS32_REG_C0_STATUS_INDEX)
			mips32_detect_fpr_mode_change(mips32, reg_value);
	} else if (num >= MIPS32_REGLIST_FPC_INDEX) {
		/* FPCR */
		cnum = num - MIPS32_REGLIST_FPC_INDEX;
		reg_value = mips32->core_regs.fpcr[cnum];
		buf_set_u32(mips32->core_cache->reg_list[num].value, 0, 32, reg_value);
	} else if (num >= MIPS32_REGLIST_FP_INDEX) {
		/* FPR */
		cnum = num - MIPS32_REGLIST_FP_INDEX;
		reg_value = mips32->core_regs.fpr[cnum];
		buf_set_u64(mips32->core_cache->reg_list[num].value, 0, 64, reg_value);
	} else {
		/* GPR */
		cnum = num - MIPS32_REGLIST_GP_INDEX;
		reg_value = mips32->core_regs.gpr[cnum];
		buf_set_u32(mips32->core_cache->reg_list[num].value, 0, 32, reg_value);
	}

	mips32->core_cache->reg_list[num].valid = true;
	mips32->core_cache->reg_list[num].dirty = false;

	LOG_DEBUG("read core reg %i value 0x%" PRIx64 "", num, reg_value);

	return ERROR_OK;
}

static int mips32_write_core_reg(struct target *target, unsigned int num)
{
	unsigned int cnum;
	uint64_t reg_value;

	/* get pointers to arch-specific information */
	struct mips32_common *mips32 = target_to_mips32(target);

	if (num >= MIPS32_NUM_REGS)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (num >= MIPS32_REGLIST_DSP_INDEX) {
		/* DSP */
		cnum = num - MIPS32_REGLIST_DSP_INDEX;
		reg_value = buf_get_u32(mips32->core_cache->reg_list[num].value, 0, 32);
		mips32->core_regs.dsp[cnum] = (uint32_t)reg_value;
	} else if (num >= MIPS32_REGLIST_C0_INDEX) {
		/* CP0 */
		cnum = num - MIPS32_REGLIST_C0_INDEX;
		reg_value = buf_get_u32(mips32->core_cache->reg_list[num].value, 0, 32);
		mips32->core_regs.cp0[cnum] = (uint32_t)reg_value;
		if (cnum == MIPS32_REG_C0_STATUS_INDEX)
			mips32_detect_fpr_mode_change(mips32, reg_value);
	} else if (num >= MIPS32_REGLIST_FPC_INDEX) {
		/* FPCR */
		cnum = num - MIPS32_REGLIST_FPC_INDEX;
		reg_value = buf_get_u32(mips32->core_cache->reg_list[num].value, 0, 32);
		mips32->core_regs.fpcr[cnum] = (uint32_t)reg_value;
	} else if (num >= MIPS32_REGLIST_FP_INDEX) {
		/* FPR */
		cnum = num - MIPS32_REGLIST_FP_INDEX;
		reg_value = buf_get_u64(mips32->core_cache->reg_list[num].value, 0, 64);
		mips32->core_regs.fpr[cnum] = reg_value;
	} else {
		/* GPR */
		cnum = num - MIPS32_REGLIST_GP_INDEX;
		reg_value = buf_get_u32(mips32->core_cache->reg_list[num].value, 0, 32);
		mips32->core_regs.gpr[cnum] = (uint32_t)reg_value;
	}

	LOG_DEBUG("write core reg %i value 0x%" PRIx64 "", num, reg_value);
	mips32->core_cache->reg_list[num].valid = true;
	mips32->core_cache->reg_list[num].dirty = false;

	return ERROR_OK;
}

int mips32_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
		int *reg_list_size, enum target_register_class reg_class)
{
	/* get pointers to arch-specific information */
	struct mips32_common *mips32 = target_to_mips32(target);
	unsigned int i;

	/* include floating point registers */
	*reg_list_size = MIPS32_NUM_REGS;
	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

	for (i = 0; i < MIPS32_NUM_REGS; i++)
		(*reg_list)[i] = &mips32->core_cache->reg_list[i];

	return ERROR_OK;
}

int mips32_save_context(struct target *target)
{
	unsigned int i;

	/* get pointers to arch-specific information */
	struct mips32_common *mips32 = target_to_mips32(target);

	/* read core registers */
	int retval = mips32_pracc_read_regs(mips32);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not read core registers from target");
		return retval;
	}

	for (i = 0; i < MIPS32_NUM_REGS; i++) {
		if (!mips32->core_cache->reg_list[i].valid)
			mips32->read_core_reg(target, i);
	}

	return ERROR_OK;
}

int mips32_restore_context(struct target *target)
{
	unsigned int i;

	/* get pointers to arch-specific information */
	struct mips32_common *mips32 = target_to_mips32(target);

	for (i = 0; i < MIPS32_NUM_REGS; i++) {
		if (mips32->core_cache->reg_list[i].dirty)
			mips32->write_core_reg(target, i);
	}

	/* write core regs */
	return mips32_pracc_write_regs(mips32);
}

int mips32_arch_state(struct target *target)
{
	struct mips32_common *mips32 = target_to_mips32(target);

	LOG_USER("target halted in %s mode due to %s, pc: 0x%8.8" PRIx32 "",
		mips_isa_strings[mips32->isa_mode],
		debug_reason_name(target),
		buf_get_u32(mips32->core_cache->reg_list[MIPS32_REGLIST_C0_PC_INDEX].value, 0, 32));

	return ERROR_OK;
}

static const struct reg_arch_type mips32_reg_type = {
	.get = mips32_get_core_reg,
	.set = mips32_set_core_reg,
};

struct reg_cache *mips32_build_reg_cache(struct target *target)
{
	/* get pointers to arch-specific information */
	struct mips32_common *mips32 = target_to_mips32(target);

	int num_regs = MIPS32_NUM_REGS;
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
	struct mips32_core_reg *arch_info = malloc(sizeof(struct mips32_core_reg) * num_regs);
	struct reg_feature *feature;
	int i;

	/* Build the process context cache */
	cache->name = "mips32 registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	(*cache_p) = cache;
	mips32->core_cache = cache;

	for (i = 0; i < num_regs; i++) {
		arch_info[i].num = mips32_regs[i].id;
		arch_info[i].target = target;
		arch_info[i].mips32_common = mips32;

		reg_list[i].name = mips32_regs[i].name;
		reg_list[i].size = mips32_regs[i].size ? 64 : 32;

		reg_list[i].value = mips32_regs[i].size ? calloc(1, 8) : calloc(1, 4);
		reg_list[i].valid = false;
		reg_list[i].type = &mips32_reg_type;
		reg_list[i].arch_info = &arch_info[i];

		reg_list[i].reg_data_type = calloc(1, sizeof(struct reg_data_type));
		if (reg_list[i].reg_data_type)
			reg_list[i].reg_data_type->type = mips32_regs[i].type;
		else
			LOG_ERROR("unable to allocate reg type list");


		reg_list[i].dirty = false;

		reg_list[i].group = mips32_regs[i].group;
		reg_list[i].number = i;
		reg_list[i].exist = true;
		reg_list[i].caller_save = true;	/* gdb defaults to true */

		feature = calloc(1, sizeof(struct reg_feature));
		if (feature) {
			feature->name = mips32_regs[i].feature;
			reg_list[i].feature = feature;
		} else
			LOG_ERROR("unable to allocate feature list");
	}

	return cache;
}

int mips32_init_arch_info(struct target *target, struct mips32_common *mips32, struct jtag_tap *tap)
{
	target->arch_info = mips32;
	mips32->common_magic = MIPS32_COMMON_MAGIC;
	mips32->fast_data_area = NULL;
	mips32->isa_imp = MIPS32_ONLY;	/* default */

	/* has breakpoint/watchpoint unit been scanned */
	mips32->bp_scanned = 0;
	mips32->data_break_list = NULL;

	mips32->ejtag_info.tap = tap;
	mips32->read_core_reg = mips32_read_core_reg;
	mips32->write_core_reg = mips32_write_core_reg;
	/* if unknown endianness defaults to little endian, 1 */
	mips32->ejtag_info.endianness = target->endianness == TARGET_BIG_ENDIAN ? 0 : 1;
	mips32->ejtag_info.scan_delay = MIPS32_SCAN_DELAY_LEGACY_MODE;
	mips32->ejtag_info.mode = 0;			/* Initial default value */
	mips32->ejtag_info.isa = 0;	/* isa on debug mips32, updated by poll function */
	mips32->ejtag_info.config_regs = 0;	/* no config register read */
	return ERROR_OK;
}

/* run to exit point. return error if exit point was not reached. */
static int mips32_run_and_wait(struct target *target, target_addr_t entry_point,
		unsigned int timeout_ms, target_addr_t exit_point, struct mips32_common *mips32)
{
	uint32_t pc;
	int retval;
	/* This code relies on the target specific  resume() and  poll()->debug_entry()
	 * sequence to write register values to the processor and the read them back */
	retval = target_resume(target, false, entry_point, false, true);
	if (retval != ERROR_OK)
		return retval;

	retval = target_wait_state(target, TARGET_HALTED, timeout_ms);
	/* If the target fails to halt due to the breakpoint, force a halt */
	if (retval != ERROR_OK || target->state != TARGET_HALTED) {
		retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;
		retval = target_wait_state(target, TARGET_HALTED, 500);
		if (retval != ERROR_OK)
			return retval;
		return ERROR_TARGET_TIMEOUT;
	}

	pc = buf_get_u32(mips32->core_cache->reg_list[MIPS32_REGLIST_C0_PC_INDEX].value, 0, 32);
	if (exit_point && (pc != exit_point)) {
		LOG_DEBUG("failed algorithm halted at 0x%" PRIx32 " ", pc);
		return ERROR_TARGET_TIMEOUT;
	}

	return ERROR_OK;
}

int mips32_run_algorithm(struct target *target, int num_mem_params,
		struct mem_param *mem_params, int num_reg_params,
		struct reg_param *reg_params, target_addr_t entry_point,
		target_addr_t exit_point, unsigned int timeout_ms, void *arch_info)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips32_algorithm *mips32_algorithm_info = arch_info;
	enum mips32_isa_mode isa_mode = mips32->isa_mode;

	uint32_t context[MIPS32_NUM_REGS];
	int retval = ERROR_OK;

	LOG_DEBUG("Running algorithm");

	/* NOTE: mips32_run_algorithm requires that each algorithm uses a software breakpoint
	 * at the exit point */

	if (mips32->common_magic != MIPS32_COMMON_MAGIC) {
		LOG_ERROR("current target isn't a MIPS32 target");
		return ERROR_TARGET_INVALID;
	}

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "not halted (run target algo)");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* refresh core register cache */
	for (unsigned int i = 0; i < MIPS32_NUM_REGS; i++) {
		if (!mips32->core_cache->reg_list[i].valid)
			mips32->read_core_reg(target, i);
		context[i] = buf_get_u32(mips32->core_cache->reg_list[i].value, 0, 32);
	}

	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction == PARAM_IN)
			continue;
		retval = target_write_buffer(target, mem_params[i].address,
				mem_params[i].size, mem_params[i].value);
		if (retval != ERROR_OK)
			return retval;
	}

	for (int i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction == PARAM_IN)
			continue;

		struct reg *reg = register_get_by_name(mips32->core_cache, reg_params[i].reg_name, false);

		if (!reg) {
			LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (reg->size != reg_params[i].size) {
			LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size",
					reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		mips32_set_core_reg(reg, reg_params[i].value);
	}

	mips32->isa_mode = mips32_algorithm_info->isa_mode;

	retval = mips32_run_and_wait(target, entry_point, timeout_ms, exit_point, mips32);

	if (retval != ERROR_OK)
		return retval;

	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction != PARAM_OUT) {
			retval = target_read_buffer(target, mem_params[i].address, mem_params[i].size,
					mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	for (int i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction != PARAM_OUT) {
			struct reg *reg = register_get_by_name(mips32->core_cache, reg_params[i].reg_name, false);
			if (!reg) {
				LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			if (reg->size != reg_params[i].size) {
				LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size",
						reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			buf_set_u32(reg_params[i].value, 0, 32, buf_get_u32(reg->value, 0, 32));
		}
	}

	/* restore everything we saved before */
	for (unsigned int i = 0; i < MIPS32_NUM_REGS; i++) {
		uint32_t regvalue;
		regvalue = buf_get_u32(mips32->core_cache->reg_list[i].value, 0, 32);
		if (regvalue != context[i]) {
			LOG_DEBUG("restoring register %s with value 0x%8.8" PRIx32,
				mips32->core_cache->reg_list[i].name, context[i]);
			buf_set_u32(mips32->core_cache->reg_list[i].value,
					0, 32, context[i]);
			mips32->core_cache->reg_list[i].valid = true;
			mips32->core_cache->reg_list[i].dirty = true;
		}
	}

	mips32->isa_mode = isa_mode;

	return ERROR_OK;
}

int mips32_examine(struct target *target)
{
	struct mips32_common *mips32 = target_to_mips32(target);

	if (!target_was_examined(target)) {
		target_set_examined(target);

		/* we will configure later */
		mips32->bp_scanned = 0;
		mips32->num_inst_bpoints = 0;
		mips32->num_data_bpoints = 0;
		mips32->num_inst_bpoints_avail = 0;
		mips32->num_data_bpoints_avail = 0;
	}

	return ERROR_OK;
}

static int mips32_configure_ibs(struct target *target)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	int retval, i;
	uint32_t bpinfo;

	/* get number of inst breakpoints */
	retval = target_read_u32(target, ejtag_info->ejtag_ibs_addr, &bpinfo);
	if (retval != ERROR_OK)
		return retval;

	mips32->num_inst_bpoints = (bpinfo >> 24) & 0x0F;
	mips32->num_inst_bpoints_avail = mips32->num_inst_bpoints;
	mips32->inst_break_list = calloc(mips32->num_inst_bpoints,
		sizeof(struct mips32_comparator));

	for (i = 0; i < mips32->num_inst_bpoints; i++)
		mips32->inst_break_list[i].reg_address =
			ejtag_info->ejtag_iba0_addr +
			(ejtag_info->ejtag_iba_step_size * i);

	/* clear IBIS reg */
	retval = target_write_u32(target, ejtag_info->ejtag_ibs_addr, 0);
	return retval;
}

static int mips32_configure_dbs(struct target *target)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	int retval, i;
	uint32_t bpinfo;

	/* get number of data breakpoints */
	retval = target_read_u32(target, ejtag_info->ejtag_dbs_addr, &bpinfo);
	if (retval != ERROR_OK)
		return retval;

	mips32->num_data_bpoints = (bpinfo >> 24) & 0x0F;
	mips32->num_data_bpoints_avail = mips32->num_data_bpoints;
	mips32->data_break_list = calloc(mips32->num_data_bpoints,
		sizeof(struct mips32_comparator));

	for (i = 0; i < mips32->num_data_bpoints; i++)
		mips32->data_break_list[i].reg_address =
			ejtag_info->ejtag_dba0_addr +
			(ejtag_info->ejtag_dba_step_size * i);

	/* clear DBIS reg */
	retval = target_write_u32(target, ejtag_info->ejtag_dbs_addr, 0);
	return retval;
}

int mips32_configure_break_unit(struct target *target)
{
	/* get pointers to arch-specific information */
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	int retval;
	uint32_t dcr;

	if (mips32->bp_scanned)
		return ERROR_OK;

	/* get info about breakpoint support */
	retval = target_read_u32(target, EJTAG_DCR, &dcr);
	if (retval != ERROR_OK)
		return retval;

	/* EJTAG 2.0 defines IB and DB bits in IMP instead of DCR. */
	if (ejtag_info->ejtag_version == EJTAG_VERSION_20) {
		ejtag_info->debug_caps = dcr & EJTAG_DCR_ENM;
		if (!(ejtag_info->impcode & EJTAG_V20_IMP_NOIB))
			ejtag_info->debug_caps |= EJTAG_DCR_IB;
		if (!(ejtag_info->impcode & EJTAG_V20_IMP_NODB))
			ejtag_info->debug_caps |= EJTAG_DCR_DB;
	} else
		/* keep  debug caps for later use */
		ejtag_info->debug_caps = dcr & (EJTAG_DCR_ENM
				| EJTAG_DCR_IB | EJTAG_DCR_DB);


	if (ejtag_info->debug_caps & EJTAG_DCR_IB) {
		retval = mips32_configure_ibs(target);
		if (retval != ERROR_OK)
			return retval;
	}

	if (ejtag_info->debug_caps & EJTAG_DCR_DB) {
		retval = mips32_configure_dbs(target);
		if (retval != ERROR_OK)
			return retval;
	}

	/* check if target endianness settings matches debug control register */
	if (((ejtag_info->debug_caps & EJTAG_DCR_ENM)
			&& (target->endianness == TARGET_LITTLE_ENDIAN)) ||
			(!(ejtag_info->debug_caps & EJTAG_DCR_ENM)
			 && (target->endianness == TARGET_BIG_ENDIAN)))
		LOG_WARNING("DCR endianness settings does not match target settings");

	LOG_DEBUG("DCR 0x%" PRIx32 " numinst %i numdata %i", dcr, mips32->num_inst_bpoints,
			mips32->num_data_bpoints);

	mips32->bp_scanned = 1;

	return ERROR_OK;
}

int mips32_enable_interrupts(struct target *target, int enable)
{
	int retval;
	int update = 0;
	uint32_t dcr;

	/* read debug control register */
	retval = target_read_u32(target, EJTAG_DCR, &dcr);
	if (retval != ERROR_OK)
		return retval;

	if (enable) {
		if (!(dcr & EJTAG_DCR_INTE)) {
			/* enable interrupts */
			dcr |= EJTAG_DCR_INTE;
			update = 1;
		}
	} else {
		if (dcr & EJTAG_DCR_INTE) {
			/* disable interrupts */
			dcr &= ~EJTAG_DCR_INTE;
			update = 1;
		}
	}

	if (update) {
		retval = target_write_u32(target, EJTAG_DCR, dcr);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

/* read processor identification cp0 register */
static int mips32_read_c0_prid(struct target *target)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	int retval;

	retval = mips32_cp0_read(ejtag_info, &mips32->prid, 15, 0);
	if (retval != ERROR_OK) {
		LOG_ERROR("processor id not available, failed to read cp0 PRId register");
		mips32->prid = 0;
	}

	return retval;
}

/**
 * mips32_find_cpu_by_prid - Find CPU information by processor ID.
 * @param[in] prid: Processor ID of the CPU.
 *
 * @brief This function looks up the CPU entry in the mips32_cpu_entry array based on the provided
 * processor ID. It also handles special cases like AMD/Alchemy CPUs that use Company Options
 * instead of Processor IDs.
 *
 * @return Pointer to the corresponding cpu_entry struct, or the 'unknown' entry if not found.
 */
static const struct cpu_entry *mips32_find_cpu_by_prid(uint32_t prid)
{
	/* AMD/Alchemy CPU uses Company Options instead of Processor ID.
	 * Therefore an extra transform step for prid to map it to an assigned ID,
	 */
	if ((prid & PRID_COMP_MASK) == PRID_COMP_ALCHEMY) {
		/* Clears Processor ID field, then put Company Option field to its place */
		prid = (prid & 0xFFFF00FF) | ((prid & 0xFF000000) >> 16);
	}

	/* Mask out Company Option */
	prid &= 0x00FFFFFF;

	for (unsigned int i = 0; i < MIPS32_NUM_CPU_ENTRIES; i++) {
		const struct cpu_entry *entry = &mips32_cpu_entry[i];
		if ((entry->prid & MIPS32_CORE_MASK) <= prid && prid <= entry->prid)
			return entry;
	}

	/* If nothing matched, then return unknown entry */
	return &mips32_cpu_entry[MIPS32_NUM_CPU_ENTRIES - 1];
}

static bool mips32_cpu_is_lexra(struct mips_ejtag *ejtag_info)
{
	return (ejtag_info->prid & PRID_COMP_MASK) == PRID_COMP_LEXRA;
}

static int mips32_cpu_get_release(struct mips_ejtag *ejtag_info)
{
	return (ejtag_info->config[0] & MIPS32_CONFIG0_AR_MASK) >> MIPS32_CONFIG0_AR_SHIFT;
}

/**
 * mips32_cpu_support_sync - Checks CPU supports ordering
 * @param[in] ejtag_info: MIPS EJTAG information structure.
 *
 * @brief MIPS ISA implemented on Lexra CPUs is MIPS-I, similar to R3000,
 * which does not have the SYNC instruction alone with unaligned
 * load/store instructions.
 *
 * @returns true if current CPU supports sync instruction(CPU is not Lexra)
*/
bool mips32_cpu_support_sync(struct mips_ejtag *ejtag_info)
{
	return !mips32_cpu_is_lexra(ejtag_info);
}

/**
 * mips32_cpu_support_hazard_barrier - Checks CPU supports hazard barrier
 * @param[in] ejtag_info: MIPS EJTAG information structure.
 *
 * @brief hazard barrier instructions EHB and *.HB was introduced to MIPS from release 2.
 *
 * @returns true if current CPU supports hazard barrier(release > 1)
*/
bool mips32_cpu_support_hazard_barrier(struct mips_ejtag *ejtag_info)
{
	return mips32_cpu_get_release(ejtag_info) > MIPS32_RELEASE_1;
}

/**
 * mips32_cpu_probe - Detects processor type and applies necessary quirks.
 * @param[in] target: The target CPU to probe.
 *
 * @brief This function probes the CPU, reads its PRID (Processor ID), and determines the CPU type.
 * It applies any quirks necessary for specific processor types.
 *
 * NOTE: The proper detection of certain CPUs can become quite complicated.
 * Please consult the following Linux kernel code when adding new CPUs:
 *  arch/mips/include/asm/cpu.h
 *  arch/mips/kernel/cpu-probe.c
 *
 * @return ERROR_OK on success; error code on failure.
 */
int mips32_cpu_probe(struct target *target)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	int retval;

	if (mips32->prid)
		return ERROR_OK; /* Already probed once, return early. */

	retval = mips32_read_c0_prid(target);
	if (retval != ERROR_OK)
		return retval;

	const struct cpu_entry *entry = mips32_find_cpu_by_prid(mips32->prid);

	switch (mips32->prid & PRID_COMP_MASK) {
	case PRID_COMP_INGENIC_E1:
		switch (mips32->prid & PRID_IMP_MASK) {
		case PRID_IMP_XBURST_REV1:
			mips32->cpu_quirks |= EJTAG_QUIRK_PAD_DRET;
			break;
		default:
			break;
		}
		break;

	/* Determine which CP0 registers are available in the current processor core */
	case PRID_COMP_MTI:
		switch (entry->prid & PRID_IMP_MASK) {
		case PRID_IMP_MAPTIV_UC:
			mips32->cp0_mask = MIPS_CP0_MAPTIV_UC;
			break;
		case PRID_IMP_MAPTIV_UP:
		case PRID_IMP_M5150:
			mips32->cp0_mask = MIPS_CP0_MAPTIV_UP;
			break;
		case PRID_IMP_IAPTIV:
		case PRID_IMP_IAPTIV_CM:
			mips32->cp0_mask = MIPS_CP0_IAPTIV;
			break;
		default:
			/* CP0 mask should be the same as MK4 by default */
			mips32->cp0_mask = MIPS_CP0_MK4;
			break;
		}

	default:
		break;
	}

	mips32->cpu_info = entry;
	LOG_DEBUG("CPU: %s (PRId %08x)", entry->cpu_name, mips32->prid);

	return ERROR_OK;
}

/* reads dsp implementation info from CP0 Config3 register {DSPP, DSPREV}*/
static void mips32_read_config_dsp(struct mips32_common *mips32, struct mips_ejtag *ejtag_info)
{
	uint32_t retval, status_value, dsp_present;
	bool dsp_enabled;

	retval = mips32_cp0_read(ejtag_info, &status_value, MIPS32_C0_STATUS, 0);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read cp0 status register");
		return;
	}

	dsp_present = ((ejtag_info->config[3] & MIPS32_CONFIG3_DSPP_MASK) >> MIPS32_CONFIG3_DSPP_SHIFT);
	dsp_enabled = (status_value & BIT(MIPS32_CP0_STATUS_MX_SHIFT)) != 0;
	if (dsp_present) {
		mips32->dsp_imp = ((ejtag_info->config[3] & MIPS32_CONFIG3_DSPREV_MASK) >> MIPS32_CONFIG3_DSPREV_SHIFT) + 1;
		LOG_USER("DSP implemented: rev %d, %s", mips32->dsp_imp, dsp_enabled ? "enabled" : "disabled");
	} else {
		LOG_USER("DSP implemented: %s", "no");
	}
}

/* read fpu implementation info from CP0 Config1 register {CU1, FP}*/
static int mips32_read_config_fpu(struct mips32_common *mips32, struct mips_ejtag *ejtag_info)
{
	int retval;
	uint32_t fp_imp = (ejtag_info->config[1] & MIPS32_CONFIG1_FP_MASK) >> MIPS32_CONFIG1_FP_SHIFT;
	char buf[60] = {0};
	if (!fp_imp) {
		LOG_USER("FPU implemented: %s", "no");
		mips32->fp_imp = MIPS32_FP_IMP_NONE;
		return ERROR_OK;
	}
	uint32_t fir_value, status_value;
	bool fpu_in_64bit, fp_enabled;

	retval = mips32_cp0_read(ejtag_info, &status_value, MIPS32_C0_STATUS, 0);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read cp0 status register");
		return retval;
	}

	fpu_in_64bit = (status_value & BIT(MIPS32_CP0_STATUS_FR_SHIFT)) != 0;
	fp_enabled = (status_value & BIT(MIPS32_CP0_STATUS_CU1_SHIFT)) != 0;
	if (fp_enabled) {
		retval = mips32_cp1_control_read(ejtag_info, &fir_value, 0);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read cp1 FIR register");
			return retval;
		}

		if ((fir_value >> MIPS32_CP1_FIR_F64_SHIFT) & 0x1)
			fp_imp++;
	} else {
		/* This is the only condition that writes to buf */
		snprintf(buf, sizeof(buf), "yes, disabled");
		fp_imp = MIPS32_FP_IMP_UNKNOWN;
	}

	mips32->fpu_in_64bit = fpu_in_64bit;
	mips32->fpu_enabled = fp_enabled;

	mips32_set_all_fpr_width(mips32, fpu_in_64bit);

	/* If fpu is not disabled, print out more information */
	if (!buf[0])
		snprintf(buf, sizeof(buf), "yes, %sbit (%s, working in %sbit)",
			fp_imp == MIPS32_FP_IMP_64 ? "64" : "32",
			fp_enabled ? "enabled" : "disabled",
			fpu_in_64bit ? "64" : "32");

	LOG_USER("FPU implemented: %s", buf);
	mips32->fp_imp = fp_imp;

	return ERROR_OK;
}

/**
 * mips32_read_config_fdc - Read Fast Debug Channel configuration
 * @param[in,out] mips32: MIPS32 common structure
 * @param[in] ejtag_info: EJTAG information structure
 * @param[in] dcr: Device Configuration Register value
 *
 * @brief Checks if the current target implements the Common Device Memory Map (CDMM) and Fast Debug Channel (FDC).
 *
 * This function examines the configuration registers and the Device Configuration Register (DCR) to determine
 * if the current MIPS32 target supports the Common Device Memory Map (CDMM) and the Fast Debug Channel (FDC).
 * If supported, it sets the corresponding flags in the MIPS32 common structure. \n
 *
 * NOTE:These are defined on MD00090, page 67 and MD00047F, page 82, respectively.
 * MIPS Documents are pretty much all available online,
 * it should pop up first when you search "MDxxxxx"
 */
static void mips32_read_config_fdc(struct mips32_common *mips32, struct mips_ejtag *ejtag_info, uint32_t dcr)
{
	if (((ejtag_info->config[3] & MIPS32_CONFIG3_CDMM_MASK) != 0) && ((dcr & EJTAG_DCR_FDC) != 0)) {
		mips32->fdc = 1;
		mips32->semihosting = 1;
	} else {
		mips32->fdc = 0;
		mips32->semihosting = 0;
	}
}

/* read config to config3 cp0 registers and log isa implementation */
int mips32_read_config_regs(struct target *target)
{
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	char buf[60] = {0};
	int retval;

	if (ejtag_info->config_regs == 0)
		for (int i = 0; i != 4; i++) {
			retval = mips32_cp0_read(ejtag_info, &ejtag_info->config[i], 16, i);
			if (retval != ERROR_OK) {
				LOG_ERROR("isa info not available, failed to read cp0 config register: %" PRId32, i);
				ejtag_info->config_regs = 0;
				return retval;
			}
			ejtag_info->config_regs = i + 1;
			if ((ejtag_info->config[i] & (1 << 31)) == 0)
				break;	/* no more config registers implemented */
		}
	else
		return ERROR_OK;	/* already successfully read */

	LOG_DEBUG("read  %"PRIu32" config registers", ejtag_info->config_regs);

	mips32->isa_rel = (ejtag_info->config[0] & MIPS32_CONFIG0_AR_MASK) >> MIPS32_CONFIG0_AR_SHIFT;
	snprintf(buf, sizeof(buf), ", release %s(AR=%d)",
			mips32->isa_rel == MIPS32_RELEASE_1 ? "1"
			: mips32->isa_rel == MIPS32_RELEASE_2 ? "2"
			: mips32->isa_rel == MIPS32_RELEASE_6 ? "6"
			: "unknown", mips32->isa_rel);

	if (ejtag_info->impcode & EJTAG_IMP_MIPS16) {
		mips32->isa_imp = MIPS32_MIPS16;
		LOG_USER("ISA implemented: %s%s", "MIPS32, MIPS16", buf);
	} else if (ejtag_info->config_regs >= 4) {	/* config3 implemented */
		unsigned int isa_imp = (ejtag_info->config[3] & MIPS32_CONFIG3_ISA_MASK) >> MIPS32_CONFIG3_ISA_SHIFT;
		if (isa_imp == 1) {
			mips32->isa_imp = MMIPS32_ONLY;
			LOG_USER("ISA implemented: %s%s", "microMIPS32", buf);

		} else if (isa_imp != 0) {
			mips32->isa_imp = MIPS32_MMIPS32;
			LOG_USER("ISA implemented: %s%s", "MIPS32, microMIPS32", buf);
		}
	} else if (mips32->isa_imp == MIPS32_ONLY)	{
		/* initial default value */
		LOG_USER("ISA implemented: %s%s", "MIPS32", buf);
	}

	/* Retrieve DSP info */
	mips32_read_config_dsp(mips32, ejtag_info);

	/* Retrieve if Float Point CoProcessor Implemented */
	retval = mips32_read_config_fpu(mips32, ejtag_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("fpu info is not available, error while reading cp0 status");
		mips32->fp_imp = MIPS32_FP_IMP_NONE;
		return retval;
	}

	uint32_t dcr;

	retval = target_read_u32(target, EJTAG_DCR, &dcr);
	if (retval != ERROR_OK) {
		LOG_ERROR("failed to read EJTAG_DCR register");
		return retval;
	}

	/* Determine if FDC and CDMM are implemented for this core */
	mips32_read_config_fdc(mips32, ejtag_info, dcr);

	return ERROR_OK;
}

int mips32_checksum_memory(struct target *target, target_addr_t address,
		uint32_t count, uint32_t *checksum)
{
	struct working_area *crc_algorithm;
	struct reg_param reg_params[2];
	struct mips32_algorithm mips32_info;

	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

	/* see contrib/loaders/checksum/mips32.s for src */
	uint32_t isa = ejtag_info->isa ? 1 : 0;

	uint32_t mips_crc_code[] = {
		MIPS32_ADDIU(isa, 12, 4, 0),			/* addiu	$t4, $a0, 0 */
		MIPS32_ADDIU(isa, 10, 5, 0),			/* addiu	$t2, $a1, 0 */
		MIPS32_ADDIU(isa, 4, 0, 0xFFFF),		/* addiu	$a0, $zero, 0xffff */
		MIPS32_BEQ(isa, 0, 0, 0x10 << isa),		/* beq		$zero, $zero, ncomp */
		MIPS32_ADDIU(isa, 11, 0, 0),			/* addiu	$t3, $zero, 0 */
						/* nbyte: */
		MIPS32_LB(isa, 5, 0, 12),			/* lb		$a1, ($t4) */
		MIPS32_ADDI(isa, 12, 12, 1),			/* addi		$t4, $t4, 1 */
		MIPS32_SLL(isa, 5, 5, 24),			/* sll		$a1, $a1, 24 */
		MIPS32_LUI(isa, 2, 0x04c1),			/* lui		$v0, 0x04c1 */
		MIPS32_XOR(isa, 4, 4, 5),			/* xor		$a0, $a0, $a1 */
		MIPS32_ORI(isa, 7, 2, 0x1db7),			/* ori		$a3, $v0, 0x1db7 */
		MIPS32_ADDU(isa, 6, 0, 0),			/* addu		$a2, $zero, $zero */
						/* loop */
		MIPS32_SLL(isa, 8, 4, 1),			/* sll		$t0, $a0, 1 */
		MIPS32_ADDIU(isa, 6, 6, 1),			/* addiu	$a2, $a2, 1 */
		MIPS32_SLTI(isa, 4, 4, 0),			/* slti		$a0, $a0, 0 */
		MIPS32_XOR(isa, 9, 8, 7),			/* xor		$t1, $t0, $a3 */
		MIPS32_MOVN(isa, 8, 9, 4),			/* movn		$t0, $t1, $a0 */
		MIPS32_SLTI(isa, 3, 6, 8),			/* slti		$v1, $a2, 8 */
		MIPS32_BNE(isa, 3, 0, NEG16(7 << isa)),		/* bne		$v1, $zero, loop */
		MIPS32_ADDU(isa, 4, 8, 0),			/* addu		$a0, $t0, $zero */
						/* ncomp */
		MIPS32_BNE(isa, 10, 11, NEG16(16 << isa)),	/* bne		$t2, $t3, nbyte */
		MIPS32_ADDIU(isa, 11, 11, 1),			/* addiu	$t3, $t3, 1 */
		MIPS32_SDBBP(isa),
	};

	/* make sure we have a working area */
	if (target_alloc_working_area(target, sizeof(mips_crc_code), &crc_algorithm) != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	pracc_swap16_array(ejtag_info, mips_crc_code, ARRAY_SIZE(mips_crc_code));

	/* convert mips crc code into a buffer in target endianness */
	uint8_t mips_crc_code_8[sizeof(mips_crc_code)];
	target_buffer_set_u32_array(target, mips_crc_code_8,
					ARRAY_SIZE(mips_crc_code), mips_crc_code);

	int retval = target_write_buffer(target, crc_algorithm->address, sizeof(mips_crc_code), mips_crc_code_8);
	if (retval != ERROR_OK)
		return retval;

	mips32_info.common_magic = MIPS32_COMMON_MAGIC;
	mips32_info.isa_mode = isa ? MIPS32_ISA_MMIPS32 : MIPS32_ISA_MIPS32;	/* run isa as in debug mode */

	init_reg_param(&reg_params[0], "r4", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, address);

	init_reg_param(&reg_params[1], "r5", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, count);

	unsigned int timeout = 20000 * (1 + (count / (1024 * 1024)));

	retval = target_run_algorithm(target, 0, NULL, 2, reg_params, crc_algorithm->address,
				      crc_algorithm->address + (sizeof(mips_crc_code) - 4), timeout, &mips32_info);

	if (retval == ERROR_OK)
		*checksum = buf_get_u32(reg_params[0].value, 0, 32);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);

	target_free_working_area(target, crc_algorithm);

	return retval;
}

/** Checks whether a memory region is erased. */
int mips32_blank_check_memory(struct target *target,
		struct target_memory_check_block *blocks, int num_blocks,
		uint8_t erased_value)
{
	struct working_area *erase_check_algorithm;
	struct reg_param reg_params[3];
	struct mips32_algorithm mips32_info;

	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

	if (erased_value != 0xff) {
		LOG_ERROR("Erase value 0x%02" PRIx8 " not yet supported for MIPS32",
			erased_value);
		return ERROR_FAIL;
	}
	uint32_t isa = ejtag_info->isa ? 1 : 0;
	uint32_t erase_check_code[] = {
						/* nbyte: */
		MIPS32_LB(isa, 8, 0, 4),			/* lb		$t0, ($a0) */
		MIPS32_AND(isa, 6, 6, 8),			/* and		$a2, $a2, $t0 */
		MIPS32_ADDIU(isa, 5, 5, NEG16(1)),		/* addiu	$a1, $a1, -1 */
		MIPS32_BNE(isa, 5, 0, NEG16(4 << isa)),		/* bne		$a1, $zero, nbyte */
		MIPS32_ADDIU(isa, 4, 4, 1),			/* addiu	$a0, $a0, 1 */
		MIPS32_SDBBP(isa)				/* sdbbp */
	};

	/* make sure we have a working area */
	if (target_alloc_working_area(target, sizeof(erase_check_code), &erase_check_algorithm) != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	pracc_swap16_array(ejtag_info, erase_check_code, ARRAY_SIZE(erase_check_code));

	/* convert erase check code into a buffer in target endianness */
	uint8_t erase_check_code_8[sizeof(erase_check_code)];
	target_buffer_set_u32_array(target, erase_check_code_8,
					ARRAY_SIZE(erase_check_code), erase_check_code);

	int retval = target_write_buffer(target, erase_check_algorithm->address,
						sizeof(erase_check_code), erase_check_code_8);
	if (retval != ERROR_OK)
		goto cleanup;

	mips32_info.common_magic = MIPS32_COMMON_MAGIC;
	mips32_info.isa_mode = isa ? MIPS32_ISA_MMIPS32 : MIPS32_ISA_MIPS32;

	init_reg_param(&reg_params[0], "r4", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, blocks[0].address);

	init_reg_param(&reg_params[1], "r5", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, blocks[0].size);

	init_reg_param(&reg_params[2], "r6", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, erased_value);

	retval = target_run_algorithm(target, 0, NULL, 3, reg_params, erase_check_algorithm->address,
			erase_check_algorithm->address + (sizeof(erase_check_code) - 4), 10000, &mips32_info);

	if (retval == ERROR_OK)
		blocks[0].result = buf_get_u32(reg_params[2].value, 0, 32);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

cleanup:
	target_free_working_area(target, erase_check_algorithm);

	if (retval != ERROR_OK)
		return retval;

	return 1;       /* only one block has been checked */
}

static int mips32_verify_pointer(struct command_invocation *cmd,
		struct mips32_common *mips32)
{
	if (mips32->common_magic != MIPS32_COMMON_MAGIC) {
		command_print(cmd, "target is not an MIPS32");
		return ERROR_TARGET_INVALID;
	}
	return ERROR_OK;
}

/**
 * mips32_read_config_mmu - Reads MMU configuration and logs relevant information.
 * @param[in] ejtag_info: EJTAG interface information.
 *
 * @brief Reads the MMU configuration from the CP0 register and calculates the number of TLB entries,
 * ways, and sets. Handles different MMU types like VTLB only, root RPU/Fixed, and VTLB and FTLB.
 *
 * @return ERROR_OK on success; error code on failure.
 */
static int mips32_read_config_mmu(struct mips_ejtag *ejtag_info)
{
	uint32_t config4, tlb_entries = 0, ways = 0, sets = 0;
	uint32_t config0 = ejtag_info->config[0];
	uint32_t config1 = ejtag_info->config[1];
	uint32_t config3 = ejtag_info->config[3];
	uint32_t mmu_type = (config0 >> 7) & 7;
	uint32_t vz_present = (config3 & BIT(23));

	int retval = mips32_cp0_read(ejtag_info, &config4, 16, 4);
	if (retval != ERROR_OK)
		return retval;

	/* mmu type = 1: VTLB only (Note: Does not account for Config4.ExtVTLB)
	 * mmu type = 3: root RPU/Fixed (Note: Only valid with VZ ASE)
	 * mmu type = 4: VTLB and FTLB
	 */
	if ((mmu_type == 1 || mmu_type == 4) || (mmu_type == 3 && vz_present)) {
		tlb_entries = (uint32_t)(((config1 >> 25) & 0x3f) + 1);
		if (mmu_type == 4) {
			/* Release 6 definition for Config4[0:15] (MD01251, page 243) */
			/* The FTLB ways field is defined as [2, 3, 4, 5, 6, 7, 8, ...0 (reserved)] */
			int index = ((config4 >> 4) & 0xf);
			ways = index > 6 ? 0 : index + 2;

			/* The FTLB sets field is defined as [1, 2, 4, 8, ..., 16384, 32768] (powers of 2) */
			index = (config4 & 0xf);
			sets = 1 << index;
			tlb_entries = tlb_entries + (ways * sets);
		}
	}
	LOG_USER("TLB Entries: %d (%d ways, %d sets per way)", tlb_entries, ways, sets);

	return ERROR_OK;
}

/**
 * mips32_cp0_find_register_by_name - Find CP0 register by its name.
 * @param[in] cp0_mask: Mask to filter out irrelevant registers.
 * @param[in] reg_name: Name of the register to find.
 *
 * @brief This function iterates through mips32_cp0_regs to find a register
 * matching reg_name, considering cp0_mask to filter out registers
 * not relevant for the current core.
 *
 * @return Pointer to the found register, or NULL if not found.
 */
static const struct mips32_cp0 *mips32_cp0_find_register_by_name(uint32_t cp0_mask, const char *reg_name)
{
	if (reg_name)
		for (unsigned int i = 0; i < MIPS32NUMCP0REGS; i++) {
			if ((mips32_cp0_regs[i].core & cp0_mask) == 0)
				continue;

			if (strcmp(mips32_cp0_regs[i].name, reg_name) == 0)
				return &mips32_cp0_regs[i];
		}
	return NULL;
}

/**
 * mips32_cp0_get_all_regs - Print all CP0 registers and their values.
 * @param[in] cmd: Command invocation context.
 * @param[in] ejtag_info: EJTAG interface information.
 * @param[in] cp0_mask: Mask to identify relevant registers.
 *
 * @brief Iterates over all CP0 registers, reads their values, and prints them.
 * Only considers registers relevant to the current core, as defined by cp0_mask.
 *
 * @return ERROR_OK on success; error code on failure.
 */
static int mips32_cp0_get_all_regs(struct command_invocation *cmd, struct mips_ejtag *ejtag_info, uint32_t cp0_mask)
{
	uint32_t value;

	for (unsigned int i = 0; i < MIPS32NUMCP0REGS; i++) {
		/* Register name not valid for this core */
		if ((mips32_cp0_regs[i].core & cp0_mask) == 0)
			continue;

		int retval = mips32_cp0_read(ejtag_info, &value, mips32_cp0_regs[i].reg, mips32_cp0_regs[i].sel);
		if (retval != ERROR_OK) {
			command_print(CMD, "Error: couldn't access reg %s", mips32_cp0_regs[i].name);
			return retval;
		}

		command_print(CMD, "%*s: 0x%8.8" PRIx32, 14, mips32_cp0_regs[i].name, value);
	}
	return ERROR_OK;
}

/**
 * mips32_cp0_get_reg_by_name - Read and print a CP0 register's value by name.
 * @param[in] cmd: Command invocation context.
 * @param[in] ejtag_info: EJTAG interface information.
 * @param[in] cp0_mask: Mask to identify relevant registers.
 *
 * @brief Finds a CP0 register by name, reads its value, and prints it.
 * Handles error scenarios like register not found or read failure.
 *
 * @return ERROR_OK on success; error code on failure.
 */
static int mips32_cp0_get_reg_by_name(struct command_invocation *cmd, struct mips_ejtag *ejtag_info, uint32_t cp0_mask)
{
	const struct mips32_cp0 *cp0_regs = mips32_cp0_find_register_by_name(cp0_mask, CMD_ARGV[0]);
	if (!cp0_regs) {
		command_print(CMD, "Error: Register '%s' not found", CMD_ARGV[0]);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	uint32_t value;
	int retval = mips32_cp0_read(ejtag_info, &value, cp0_regs->reg, cp0_regs->sel);
	if (retval != ERROR_OK) {
		command_print(CMD, "Error: Encounter an Error while reading cp0 reg %d sel %d",
					cp0_regs->reg, cp0_regs->sel);
		return retval;
	}

	command_print(CMD, "0x%8.8" PRIx32, value);
	return ERROR_OK;
}

/**
 * mips32_cp0_get_reg_by_number - Read and print a CP0 register's value by number.
 * @param[in] cmd: Command invocation context.
 * @param[in] ejtag_info: EJTAG interface information.
 *
 * @brief Reads a specific CP0 register (identified by number and selection) and prints its value.
 * The register number and selection are parsed from the command arguments.
 *
 * @return ERROR_OK on success; error code on failure.
 */
static int mips32_cp0_get_reg_by_number(struct command_invocation *cmd, struct mips_ejtag *ejtag_info)
{
	uint32_t cp0_reg, cp0_sel, value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], cp0_reg);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], cp0_sel);

	int retval = mips32_cp0_read(ejtag_info, &value, cp0_reg, cp0_sel);
	if (retval != ERROR_OK) {
		command_print(CMD,
				"Error: couldn't access reg %" PRIu32,
				cp0_reg);
		return retval;
	}

	command_print(CMD, "cp0 reg %" PRIu32 ", select %" PRIu32 ": %8.8" PRIx32,
			cp0_reg, cp0_sel, value);
	return ERROR_OK;
}

/**
 * mips32_cp0_set_reg_by_name - Write to a CP0 register identified by name.
 * @param[in] cmd: Command invocation context.
 * @param[in] mips32: Common MIPS32 data structure.
 * @param[in] ejtag_info: EJTAG interface information.
 *
 * @brief Writes a value to a CP0 register specified by name. Updates internal
 * cache if specific registers (STATUS, CAUSE, DEPC, GUESTCTL1) are modified.
 *
 * @return ERROR_OK on success; error code on failure.
 */
static int mips32_cp0_set_reg_by_name(struct command_invocation *cmd,
		struct mips32_common *mips32, struct mips_ejtag *ejtag_info)
{
	const struct mips32_cp0 *cp0_regs = mips32_cp0_find_register_by_name(mips32->cp0_mask, CMD_ARGV[0]);
	if (!cp0_regs) {
		command_print(CMD, "Error: Register '%s' not found", CMD_ARGV[0]);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}


	uint32_t value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	if (cp0_regs->reg == MIPS32_C0_STATUS && cp0_regs->sel == 0) {
		/* Update cached Status register if user is writing to Status */
		mips32->core_regs.cp0[MIPS32_REG_C0_STATUS_INDEX] = value;
		mips32->core_cache->reg_list[MIPS32_REGLIST_C0_STATUS_INDEX].dirty = 1;
	} else if (cp0_regs->reg == MIPS32_C0_CAUSE && cp0_regs->sel == 0) {
		/* Update register cache with new value if its Cause */
		mips32->core_regs.cp0[MIPS32_REG_C0_CAUSE_INDEX] = value;
		mips32->core_cache->reg_list[MIPS32_REGLIST_C0_CAUSE_INDEX].dirty = 1;
	} else if (cp0_regs->reg == MIPS32_C0_DEPC && cp0_regs->sel == 0) {
		/* Update cached PC if its DEPC */
		mips32->core_regs.cp0[MIPS32_REG_C0_PC_INDEX] = value;
		mips32->core_cache->reg_list[MIPS32_REGLIST_C0_PC_INDEX].dirty = 1;
	} else if (cp0_regs->reg == MIPS32_C0_GUESTCTL1 && cp0_regs->sel == 4) {
		/* Update cached guestCtl1 */
		mips32->core_regs.cp0[MIPS32_REG_C0_GUESTCTL1_INDEX] = value;
		mips32->core_cache->reg_list[MIPS32_REGLIST_C0_GUESTCTL1_INDEX].dirty = 1;
	}

	int retval = mips32_cp0_write(ejtag_info, value,
								cp0_regs->reg,
								cp0_regs->sel);
	if (retval != ERROR_OK) {
		command_print(CMD, "Error: Encounter an Error while writing to cp0 reg %d, sel %d",
					cp0_regs->reg, cp0_regs->sel);
		return retval;
	}

	command_print(CMD, "cp0 reg %s (%u, select %u: %8.8" PRIx32 ")",
			CMD_ARGV[0], cp0_regs->reg, cp0_regs->sel, value);
	return ERROR_OK;
}

/**
 * mips32_cp0_set_reg_by_number - Write to a CP0 register identified by number.
 * @param[in] cmd: Command invocation context.
 * @param[in] mips32: Common MIPS32 data structure.
 * @param[in] ejtag_info: EJTAG interface information.
 *
 * @brief Writes a value to a CP0 register specified by number and selection.
 * Handles special cases like updating the internal cache for certain registers.
 *
 * @return ERROR_OK on success; error code on failure.
 */
static int mips32_cp0_set_reg_by_number(struct command_invocation *cmd,
		struct mips32_common *mips32, struct mips_ejtag *ejtag_info)
{
	uint32_t cp0_reg, cp0_sel, value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], cp0_reg);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], cp0_sel);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], value);

	if (cp0_reg == MIPS32_C0_STATUS && cp0_sel == 0) {
		/* Update cached status register if user is writing to Status register */
		mips32->core_regs.cp0[MIPS32_REG_C0_STATUS_INDEX] = value;
		mips32->core_cache->reg_list[MIPS32_REGLIST_C0_STATUS_INDEX].dirty = 1;
	} else if (cp0_reg == MIPS32_C0_CAUSE && cp0_sel == 0) {
		/* Update register cache with new value if its Cause register */
		mips32->core_regs.cp0[MIPS32_REG_C0_CAUSE_INDEX] = value;
		mips32->core_cache->reg_list[MIPS32_REGLIST_C0_CAUSE_INDEX].dirty = 1;
	} else if (cp0_reg == MIPS32_C0_DEPC && cp0_sel == 0) {
		/* Update cached PC if its DEPC */
		mips32->core_regs.cp0[MIPS32_REG_C0_PC_INDEX] = value;
		mips32->core_cache->reg_list[MIPS32_REGLIST_C0_PC_INDEX].dirty = 1;
	} else if (cp0_reg == MIPS32_C0_GUESTCTL1 && cp0_sel == 4) {
		/* Update cached guestCtl1, too */
		mips32->core_regs.cp0[MIPS32_REG_C0_GUESTCTL1_INDEX] = value;
		mips32->core_cache->reg_list[MIPS32_REGLIST_C0_GUESTCTL1_INDEX].dirty = 1;
	}

	int retval = mips32_cp0_write(ejtag_info, value, cp0_reg, cp0_sel);
	if (retval != ERROR_OK) {
		command_print(CMD,
				"Error: couldn't access cp0 reg %" PRIu32 ", select %" PRIu32,
				cp0_reg,  cp0_sel);
		return retval;
	}

	command_print(CMD, "cp0 reg %" PRIu32 ", select %" PRIu32 ": %8.8" PRIx32,
			cp0_reg, cp0_sel, value);
	return ERROR_OK;
}

/**
 * mips32_handle_cp0_command - Handle commands related to CP0 registers.
 * @cmd: Command invocation context.
 *
 * Orchestrates different operations on CP0 registers based on the command arguments.
 * Supports operations like reading all registers, reading/writing a specific register
 * by name or number.
 *
 * Return: ERROR_OK on success; error code on failure.
 */
COMMAND_HANDLER(mips32_handle_cp0_command)
{
	int retval, tmp;
	struct target *target = get_current_target(CMD_CTX);
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;


	retval = mips32_verify_pointer(CMD, mips32);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED) {
		command_print(CMD, "Error: target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_TARGET_NOT_HALTED;
	}

	switch (CMD_ARGC) {
		case 0: /* No arg => print out all cp0 regs */
			retval = mips32_cp0_get_all_regs(CMD, ejtag_info, mips32->cp0_mask);
			break;
		case 1: /* 1 arg => get cp0 #reg/#sel value by name */
			retval = mips32_cp0_get_reg_by_name(CMD, ejtag_info, mips32->cp0_mask);
			break;
		case 2: /* 2 args => get cp0 reg/sel value or set value by name */
			tmp = *CMD_ARGV[0];
			if (isdigit(tmp)) /* starts from number then args are #reg and #sel */
				retval = mips32_cp0_get_reg_by_number(CMD, ejtag_info);
			else /* or set value by register name */
				retval = mips32_cp0_set_reg_by_name(CMD, mips32, ejtag_info);

			break;
		case 3: /* 3 args => set cp0 reg/sel value*/
			retval = mips32_cp0_set_reg_by_number(CMD, mips32, ejtag_info);
			break;
		default: /* Other argc => err */
			retval = ERROR_COMMAND_SYNTAX_ERROR;
			break;
	}

	return retval;
}

/**
 * mips32_dsp_enable - Enable access to DSP registers
 * @param[in] ctx: Context information for the pracc queue
 * @param[in] isa: Instruction Set Architecture identifier
 *
 * @brief Enables access to DSP registers by modifying the status register.
 *
 * This function adds instructions to the context queue for enabling
 * access to DSP registers by modifying the status register.
 */
static void mips32_dsp_enable(struct pracc_queue_info *ctx, int isa)
{
	/* Save Status Register */
	/* move status to $9 (t1) 2*/
	pracc_add(ctx, 0, MIPS32_MFC0(isa, 9, 12, 0));

	/* Read it again in order to modify it */
	/* move status to $0 (t0) 3*/
	pracc_add(ctx, 0, MIPS32_MFC0(isa, 8, 12, 0));

	/* Enable access to DSP registers by setting MX bit in status register */
	/* $15 = MIPS32_PRACC_STACK 4/5/6*/
	pracc_add(ctx, 0, MIPS32_LUI(isa, 15, UPPER16(MIPS32_DSP_ENABLE)));
	pracc_add(ctx, 0, MIPS32_ORI(isa, 15, 15, LOWER16(MIPS32_DSP_ENABLE)));
	pracc_add(ctx, 0, MIPS32_ISA_OR(8, 8, 15));
	/* Enable DSP - update status registers 7*/
	pracc_add(ctx, 0, MIPS32_MTC0(isa, 8, 12, 0));
}

/**
 * mips32_dsp_restore - Restore DSP status registers to the previous setting
 * @param[in] ctx: Context information pracc queue
 * @param[in] isa: isa identifier
 *
 * @brief Restores the DSP status registers to their previous setting.
 *
 * This function adds instructions to the context queue for restoring the DSP
 * status registers to their values before the operation.
 */
static void mips32_dsp_restore(struct pracc_queue_info *ctx, int isa)
{
	pracc_add(ctx, 0, MIPS32_MTC0(isa, 9, 12, 0)); /* Restore status registers to previous setting */
	pracc_add(ctx, 0, MIPS32_NOP); /* nop */
}

/**
 * mips32_pracc_read_dsp_reg - Read a value from a MIPS32 DSP register
 * @param[in] ejtag_info: EJTAG information structure
 * @param[out] val: Pointer to store the read value
 * @param[in] reg: Index of the DSP register to read
 *
 * @brief Reads the value from the specified MIPS32 DSP register using EJTAG access.
 *
 * This function initiates a sequence of instructions to read the value from the
 * specified DSP register. It will enable dsp module if its not enabled
 * and restoring the status registers after the read operation.
 *
 * @return ERROR_OK on success; error code on failure.
 */
static int mips32_pracc_read_dsp_reg(struct mips_ejtag *ejtag_info, uint32_t *val, uint32_t reg)
{
	int isa = 0;

	struct pracc_queue_info ctx = {
		.max_code = 48,
		.ejtag_info = ejtag_info
	};

	uint32_t dsp_read_code[] = {
		MIPS32_DSP_MFHI(t0, 1),		/* mfhi	t0,$ac1 - OPCODE - 0x00204010 */
		MIPS32_DSP_MFLO(t0, 1),		/* mflo	t0,$ac1 - OPCODE - 0x00204012 */
		MIPS32_DSP_MFHI(t0, 2),		/* mfhi	t0,$ac2 - OPCODE - 0x00404010 */
		MIPS32_DSP_MFLO(t0, 2),		/* mflo	t0,$ac2 - OPCODE - 0x00404012 */
		MIPS32_DSP_MFHI(t0, 3),		/* mfhi	t0,$ac3 - OPCODE - 0x00604010*/
		MIPS32_DSP_MFLO(t0, 3),		/* mflo	t0,$ac3 - OPCODE - 0x00604012 */
		MIPS32_DSP_RDDSP(t0, 0x3F),	/* rddsp t0, 0x3f (DSPCtl) - OPCODE - 0x7c3f44b8 */
	};

	/* Check status register to determine if dsp register access is enabled */
	/* Get status register so it can be restored later */

	ctx.pracc_list = NULL;

	/* Init context queue */
	pracc_queue_init(&ctx);

	if (ctx.retval != ERROR_OK)
		goto exit;

	/* Enables DSP whether its already enabled or not */
	mips32_dsp_enable(&ctx, isa);

	/* move AC or Control to $8 (t0) 8*/
	pracc_add(&ctx, 0, dsp_read_code[reg]);
	/* Restore status registers to previous setting */
	mips32_dsp_restore(&ctx, isa);

	/* $15 = MIPS32_PRACC_BASE_ADDR 1*/
	pracc_add(&ctx, 0, MIPS32_LUI(isa, 15, PRACC_UPPER_BASE_ADDR));
	/* store $8 to pracc_out 10*/
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT, MIPS32_SW(isa, 8, PRACC_OUT_OFFSET, 15));
	/* move COP0 DeSave to $15 11*/
	pracc_add(&ctx, 0, MIPS32_MFC0(isa, 15, 31, 0));
	/* restore upper 16 of $8 12*/
	pracc_add(&ctx, 0, MIPS32_LUI(isa, 8, UPPER16(ejtag_info->reg8)));
	/* restore lower 16 of $8 13*/
	pracc_add(&ctx, 0, MIPS32_ORI(isa, 8, 8, LOWER16(ejtag_info->reg8)));
	/* restore upper 16 of $9 14*/
	pracc_add(&ctx, 0, MIPS32_LUI(isa, 9, UPPER16(ejtag_info->reg9)));
	pracc_add(&ctx, 0, MIPS32_SYNC(isa));
	/* jump to start 18*/
	pracc_add(&ctx, 0, MIPS32_B(isa, NEG16(ctx.code_count + 1)));
	/* restore lower 16 of $9 15*/
	pracc_add(&ctx, 0, MIPS32_ORI(isa, 9, 9, LOWER16(ejtag_info->reg9)));

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, val, 1);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

/**
 * mips32_pracc_write_dsp_reg - Write a value to a MIPS32 DSP register
 * @param[in] ejtag_info: EJTAG information structure
 * @param[in] val: Value to be written to the register
 * @param[in] reg: Index of the DSP register to write
 *
 * @brief Writes the specified value to the specified MIPS32 DSP register.
 *
 * This function initiates a sequence of instructions to write the given value to the
 * specified DSP register.
 *
 * @return ERROR_OK on success; error code on failure.
 */
static int mips32_pracc_write_dsp_reg(struct mips_ejtag *ejtag_info, uint32_t val, uint32_t reg)
{
	int isa = 0;

	struct pracc_queue_info ctx = {
		.max_code = 48,
		.ejtag_info = ejtag_info
	};

	uint32_t dsp_write_code[] = {
		MIPS32_DSP_MTHI(t0, 1),		/* mthi t0, $ac1 - OPCODE - 0x01000811 */
		MIPS32_DSP_MTLO(t0, 1),		/* mtlo t0, $ac1 - OPCODE - 0x01000813 */
		MIPS32_DSP_MTHI(t0, 2),		/* mthi t0, $ac2 - OPCODE - 0x01001011 */
		MIPS32_DSP_MTLO(t0, 2),		/* mtlo t0, $ac2 - OPCODE - 0x01001013 */
		MIPS32_DSP_MTHI(t0, 3),		/* mthi t0, $ac3 - OPCODE - 0x01001811 */
		MIPS32_DSP_MTLO(t0, 3),		/* mtlo t0, $ac3 - OPCODE - 0x01001813 */
		MIPS32_DSP_WRDSP(t0, 0x1F), /* wrdsp t0, 0x1f (DSPCtl) - OPCODE - 0x7d00fcf8*/
	};

	/* Init context queue */
	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	/* Enables DSP whether its already enabled or not */
	mips32_dsp_enable(&ctx, isa);

	/* Load val to $8 (t0) */
	pracc_add(&ctx, 0, MIPS32_LUI(isa, 8, UPPER16(val)));
	pracc_add(&ctx, 0, MIPS32_ORI(isa, 8, 8, LOWER16(val)));

	/* move AC or Control to $8 (t0) */
	pracc_add(&ctx, 0, dsp_write_code[reg]);

	/* nop, delay in order to ensure write */
	pracc_add(&ctx, 0, MIPS32_NOP);
	/* Restore status registers to previous setting */
	mips32_dsp_restore(&ctx, isa);

	/* move COP0 DeSave to $15 */
	pracc_add(&ctx, 0, MIPS32_MFC0(isa, 15, 31, 0));

	/* restore $8 */
	pracc_add(&ctx, 0, MIPS32_LUI(isa, 8, UPPER16(ejtag_info->reg8)));
	pracc_add(&ctx, 0, MIPS32_ORI(isa, 8, 8, LOWER16(ejtag_info->reg8)));

	/* restore upper 16 of $9 */
	pracc_add(&ctx, 0, MIPS32_LUI(isa, 9, UPPER16(ejtag_info->reg9)));

	/* jump to start */
	pracc_add(&ctx, 0, MIPS32_B(isa, NEG16(ctx.code_count + 1)));
	/* restore lower 16 of $9  */
	pracc_add(&ctx, 0, MIPS32_ORI(isa, 9, 9, LOWER16(ejtag_info->reg9)));

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 1);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

/**
 * mips32_handle_cpuinfo_command - Handles the 'cpuinfo' command.
 * @param[in] cmd: Command invocation context.
 *
 * @brief Executes the 'cpuinfo' command which displays detailed information about the current CPU core.
 * This includes core type, vendor, instruction set, cache size, and other relevant details.
 *
 * @return ERROR_OK on success; error code on failure.
 */
COMMAND_HANDLER(mips32_handle_cpuinfo_command)
{
	int retval;
	struct target *target = get_current_target(CMD_CTX);
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

	uint32_t prid = mips32->prid; /* cp0 PRID - 15, 0 */
	uint32_t config0 = ejtag_info->config[0]; /*	cp0 config - 16, 0 */
	uint32_t config1 = ejtag_info->config[1]; /*	cp0 config - 16, 1 */
	uint32_t config3 = ejtag_info->config[3]; /*	cp0 config - 16, 3 */

	/* Following configs are not read during probe */
	uint32_t config5; /*	cp0 config - 16, 5 */

	/* No args for now */
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (target->state != TARGET_HALTED) {
		command_print(CMD, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = mips32_cp0_read(ejtag_info, &config5, 16, 5);
	if (retval != ERROR_OK)
		return retval;

	/* Determine Core info */
	const struct cpu_entry *entry = mips32->cpu_info;
	/* Display Core Type info */
	command_print(CMD, "CPU Core: %s", entry->cpu_name);

	/* Display Core Vendor ID if it's unknown */
	if (entry == &mips32_cpu_entry[MIPS32_NUM_CPU_ENTRIES - 1])
		command_print(CMD, "Vendor: Unknown CPU vendor code %x.", ((prid & 0x00ffff00) >> 16));
	else
		command_print(CMD, "Vendor: %s", entry->vendor);

	/* If MIPS release 2 or above, then get exception base info */
	enum mips32_isa_rel ar = mips32->isa_rel;
	if (ar > MIPS32_RELEASE_1) {	/* release 2 and above */
		uint32_t ebase;
		retval = mips32_cp0_read(ejtag_info, &ebase, 15, 1);
		if (retval != ERROR_OK)
			return retval;

		command_print(CMD, "Current CPU ID: %d", (ebase & 0x1ff));
	} else {
		command_print(CMD, "Current CPU ID: 0");
	}

	char *instr;
	switch ((config3 & MIPS32_CONFIG3_ISA_MASK) >> MIPS32_CONFIG3_ISA_SHIFT) {
	case 0:
		instr = "MIPS32";
		break;
	case 1:
		instr = "microMIPS";
		break;
	case 2:
		instr = "MIPS32 (at reset) and microMIPS";
		break;
	case 3:
	default:
		instr = "microMIPS (at reset) and MIPS32";
		break;
	}

	/* Display Instruction Set Info */
	command_print(CMD, "Instr set: %s", instr);
	command_print(CMD, "Instr rel: %s",
			ar == MIPS32_RELEASE_1 ? "1"
			: ar == MIPS32_RELEASE_2 ? "2"
			: ar == MIPS32_RELEASE_6 ? "6"
			: "unknown");
	command_print(CMD, "PRId: %x", prid);
	/* Some of MIPS CPU Revisions(for M74K) can be seen on MD00541, page 26 */
	uint32_t rev = prid & 0x000000ff;
	command_print(CMD, "RTL Rev: %d.%d.%d", (rev & 0xE0), (rev & 0x1C), (rev & 0x3));

	command_print(CMD, "Max Number of Instr Breakpoints: %d", mips32->num_inst_bpoints);
	command_print(CMD, "Max Number of  Data Breakpoints: %d", mips32->num_data_bpoints);

	/* MMU Support */
	uint32_t mmu_type = (config0 >> 7) & 7; /* MMU Type Info */
	char *mmu;
	switch (mmu_type) {
		case MIPS32_MMU_TLB:
			mmu = "TLB";
		break;
		case MIPS32_MMU_BAT:
			mmu = "BAT";
		break;
		case MIPS32_MMU_FIXED:
			mmu = "FIXED";
		break;
		case MIPS32_MMU_DUAL_VTLB_FTLB:
			mmu = "DUAL VAR/FIXED";
		break;
		default:
			mmu = "Unknown";
	}
	command_print(CMD, "MMU Type: %s", mmu);

	retval = mips32_read_config_mmu(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	/* Definitions of I/D Cache Sizes are available on MD01251, page 224~226 */
	int index;
	uint32_t ways, sets, bpl;

	/* Determine Instr Cache Size */
	/* Ways mapping = [1, 2, 3, 4, 5, 6, 7, 8] */
	ways = ((config1 >> MIPS32_CFG1_IASHIFT) & 7);

	/* Sets per way = [64, 128, 256, 512, 1024, 2048, 4096, 32] */
	index = ((config1 >> MIPS32_CFG1_ISSHIFT) & 7);
	sets = index == 7 ? 32 : 32 << (index + 1);

	/* Bytes per line = [0, 4, 8, 16, 32, 64, 128, Reserved] */
	index = ((config1 >> MIPS32_CFG1_ILSHIFT) & 7);
	bpl = index == 0 ? 0 : 4 << (index - 1);
	command_print(CMD, "Instr Cache: %d (%d ways, %d lines, %d byte per line)", ways * sets * bpl, ways, sets, bpl);

	/* Determine data cache size, same as above */
	ways = ((config1 >>  MIPS32_CFG1_DASHIFT) & 7);

	index = ((config1 >> MIPS32_CFG1_DSSHIFT) & 7);
	sets = index == 7 ? 32 : 32 << (index + 1);

	index = ((config1 >> MIPS32_CFG1_DLSHIFT) & 7);
	bpl = index == 0 ? 0 : 4 << (index - 1);
	command_print(CMD, " Data Cache: %d (%d ways, %d lines, %d byte per line)", ways * sets * bpl, ways, sets, bpl);

	/* does the core hava FPU*/
	mips32_read_config_fpu(mips32, ejtag_info);

	/* does the core support a DSP */
	mips32_read_config_dsp(mips32, ejtag_info);

	/* VZ module */
	uint32_t vzase = (config3 & BIT(23));
	if (vzase)
		command_print(CMD, "VZ implemented: yes");
	else
		command_print(CMD, "VZ implemented: no");

	/* multithreading */
	uint32_t mtase  = (config3 & BIT(2));
	if (mtase) {
		command_print(CMD, "MT  implemented: yes");

		/* Get VPE and Thread info */
		uint32_t tcbind;
		uint32_t mvpconf0;

		/* Read tcbind register */
		retval = mips32_cp0_read(ejtag_info, &tcbind, 2, 2);
		if (retval != ERROR_OK)
			return retval;

		command_print(CMD, " | Current VPE: %d", (tcbind & 0xf));
		command_print(CMD, " | Current  TC: %d", ((tcbind >> 21) & 0xff));

		/* Read mvpconf0 register */
		retval = mips32_cp0_read(ejtag_info, &mvpconf0, 0, 2);
		if (retval != ERROR_OK)
			return retval;

		command_print(CMD, " | Total  TC: %d", (mvpconf0 & 0xf) + 1);
		command_print(CMD, " | Total VPE: %d", ((mvpconf0 >> 10) & 0xf) + 1);
	} else {
		command_print(CMD, "MT  implemented: no");
	}

	/* MIPS SIMD Architecture (MSA) */
	uint32_t msa = (config3 & BIT(28));
	command_print(CMD, "MSA implemented: %s", msa ? "yes" : "no");

	/* Move To/From High COP0 (MTHC0/MFHC0) instructions are implemented.
	 * Implicates current ISA release >= 5.*/
	uint32_t mvh = (config5 & BIT(5));
	command_print(CMD, "MVH implemented: %s", mvh ? "yes" : "no");

	/* Common Device Memory Map implemented? */
	uint32_t cdmm = (config3 & BIT(3));
	command_print(CMD, "CDMM implemented: %s", cdmm ? "yes" : "no");

	return ERROR_OK;
}

/**
 * mips32_dsp_find_register_by_name - Find DSP register index by name
 * @param[in] reg_name: Name of the DSP register to find
 *
 * @brief Searches for a DSP register by name and returns its index.
 * If no match is found, it returns MIPS32NUMDSPREGS.
 *
 * @return Index of the found register or MIPS32NUMDSPREGS if not found.
 */
static int mips32_dsp_find_register_by_name(const char *reg_name)
{
	if (reg_name)
		for (int i = 0; i < MIPS32NUMDSPREGS; i++) {
			if (strcmp(mips32_dsp_regs[i].name, reg_name) == 0)
				return i;
		}
	return MIPS32NUMDSPREGS;
}

/**
 * mips32_dsp_get_all_regs - Get values of all MIPS32 DSP registers
 * @param[in] cmd: Command invocation context
 * @param[in] ejtag_info: EJTAG information structure
 *
 * @brief This function iterates through all DSP registers, reads their values,
 * and prints each register name along with its corresponding value.
 *
 * @return ERROR_OK on success; error code on failure.
 */
static int mips32_dsp_get_all_regs(struct command_invocation *cmd, struct mips32_common *mips32)
{
	uint32_t value = 0;
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	for (int i = 0; i < MIPS32NUMDSPREGS; i++) {
		int retval = mips32_pracc_read_dsp_reg(ejtag_info, &value, i);
		if (retval != ERROR_OK) {
			command_print(CMD, "couldn't access reg %s", mips32_dsp_regs[i].name);
			return retval;
		}
		mips32->core_regs.dsp[i] = value;
		mips32->core_cache->reg_list[MIPS32_REGLIST_DSP_INDEX + i].dirty = 1;
		command_print(CMD, "%*s: 0x%8.8x", 7, mips32_dsp_regs[i].name, value);
	}
	return ERROR_OK;
}

/**
 * mips32_dsp_get_register - Get the value of a MIPS32 DSP register
 * @param[in] cmd: Command invocation context
 * @param[in] ejtag_info: EJTAG information structure
 *
 * @brief Retrieves the value of a specified MIPS32 DSP register.
 * If the register is found, it reads the register value and prints the result.
 * If the register is not found, it prints an error message.
 *
 * @return ERROR_OK on success; error code on failure.
 */
static int mips32_dsp_get_register(struct command_invocation *cmd, struct mips32_common *mips32)
{
	uint32_t value = 0;
	int index = mips32_dsp_find_register_by_name(CMD_ARGV[0]);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	if (index == MIPS32NUMDSPREGS) {
		command_print(CMD, "ERROR: register '%s' not found", CMD_ARGV[0]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	int retval = mips32_pracc_read_dsp_reg(ejtag_info, &value, index);
	if (retval != ERROR_OK) {
		command_print(CMD, "ERROR: Could not access dsp register %s", CMD_ARGV[0]);
		return retval;
	}

	command_print(CMD, "0x%8.8x", value);

	if (mips32->core_regs.dsp[index] != value) {
		mips32->core_regs.dsp[index] = value;
		mips32->core_cache->reg_list[MIPS32_REGLIST_DSP_INDEX + index].dirty = 1;
	}

	return retval;
}

/**
 * mips32_dsp_set_register - Set the value of a MIPS32 DSP register
 * @param[in] cmd: Command invocation context
 * @param[in] ejtag_info: EJTAG information structure
 *
 * @brief Sets the value of a specified MIPS32 DSP register.
 * If the register is found, it writes provided value to the register.
 * If the register is not found or there is an error in writing the value,
 * it prints an error message.
 *
 * @return ERROR_OK on success; error code on failure.
 */
static int mips32_dsp_set_register(struct command_invocation *cmd, struct mips32_common *mips32)
{
	uint32_t value;
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	int index = mips32_dsp_find_register_by_name(CMD_ARGV[0]);
	if (index == MIPS32NUMDSPREGS) {
		command_print(CMD, "ERROR: register '%s' not found", CMD_ARGV[0]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	int retval = mips32_pracc_write_dsp_reg(ejtag_info, value, index);
	if (retval != ERROR_OK) {
		command_print(CMD, "Error: could not write to dsp register %s", CMD_ARGV[0]);
		return retval;
	}

	mips32->core_regs.dsp[index] = value;
	mips32->core_cache->reg_list[MIPS32_REGLIST_DSP_INDEX + index].dirty = 1;

	return retval;
}

/**
 * mips32_handle_dsp_command - Handles mips dsp related command
 * @param[in] cmd: Command invocation context
 *
 * @brief Reads or sets the content of each dsp register.
 *
 * @return ERROR_OK on success; error code on failure.
*/
COMMAND_HANDLER(mips32_handle_dsp_command)
{
	int retval, tmp;
	struct target *target = get_current_target(CMD_CTX);
	struct mips32_common *mips32 = target_to_mips32(target);

	retval = mips32_verify_pointer(CMD, mips32);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED) {
		command_print(CMD, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

	/* Check for too many command args */
	if (CMD_ARGC >= 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Check if DSP access supported or not */
	if (!mips32->dsp_imp) {
		/* Issue Error Message */
		command_print(CMD, "DSP not implemented by this processor");
		return ERROR_OK;
	}

	switch (CMD_ARGC) {
		case 0:
			retval = mips32_dsp_get_all_regs(CMD, mips32);
			break;
		case 1:
			retval = mips32_dsp_get_register(CMD, mips32);
			break;
		case 2:
			tmp = *CMD_ARGV[0];
			if (isdigit(tmp)) {
				command_print(CMD, "Error: invalid dsp command format");
				retval = ERROR_COMMAND_ARGUMENT_INVALID;
			} else {
				retval = mips32_dsp_set_register(CMD, mips32);
			}
			break;
		default:
			command_print(CMD, "Error: invalid argument format, required 0-2, given %d", CMD_ARGC);
			retval = ERROR_COMMAND_ARGUMENT_INVALID;
			break;
	}
	return retval;
}

/**
 * mips32_handle_ejtag_reg_command - Handler commands related to EJTAG
 * @param[in] cmd: Command invocation context
 *
 * @brief Prints all EJTAG Registers including DCR features.
 *
 * @return ERROR_OK on success; error code on failure.
 */
COMMAND_HANDLER(mips32_handle_ejtag_reg_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

	uint32_t ejtag_ctrl;
	uint32_t dcr;
	int retval;

	retval = mips_ejtag_get_idcode(ejtag_info);
	if (retval != ERROR_OK)
		command_print(CMD, "Error: Encounter an Error while getting idcode");
	else
		command_print(CMD, "       idcode: 0x%8.8" PRIx32, ejtag_info->idcode);

	retval = mips_ejtag_get_impcode(ejtag_info);
	if (retval != ERROR_OK)
		command_print(CMD, "Error: Encounter an Error while getting impcode");
	else
		command_print(CMD, "      impcode: 0x%8.8" PRIx32, ejtag_info->impcode);

	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	ejtag_ctrl = ejtag_info->ejtag_ctrl;
	retval = mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
	if (retval != ERROR_OK)
		command_print(CMD, "Error: Encounter an Error while executing drscan reading EJTAG Control register");
	else
		command_print(CMD, "ejtag control: 0x%8.8" PRIx32, ejtag_ctrl);

	ejtag_main_print_imp(ejtag_info);

	/* Display current DCR */
	retval = target_read_u32(target, EJTAG_DCR, &dcr);
	if (retval != ERROR_OK)
		command_print(CMD, "Error: Encounter an Error while reading Debug Control Register");
	else
		command_print(CMD, "          DCR: 0x%8.8" PRIx32, dcr);

	for (unsigned int i = 0; i < EJTAG_DCR_ENTRIES; i++) {
		if (dcr & BIT(dcr_features[i].bit))
			command_print(CMD, "%s supported", dcr_features[i].name);
	}

	return ERROR_OK;
}

/**
 * mips32_handle_scan_delay_command - Handler command for changing scan delay
 * @param[in] cmd: Command invocation context
 *
 * @brief Changes current scan mode between legacy and fast queued mode.
 *
 * @return ERROR_OK on success; error code on failure.
 */
COMMAND_HANDLER(mips32_handle_scan_delay_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct mips32_common *mips32 = target_to_mips32(target);
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;

	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], ejtag_info->scan_delay);
	else if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(CMD, "scan delay: %d nsec", ejtag_info->scan_delay);
	if (ejtag_info->scan_delay >= MIPS32_SCAN_DELAY_LEGACY_MODE) {
		ejtag_info->mode = 0;
		command_print(CMD, "running in legacy mode");
	} else {
		ejtag_info->mode = 1;
		command_print(CMD, "running in fast queued mode");
	}

	return ERROR_OK;
}

static const struct command_registration mips32_exec_command_handlers[] = {
	{
		.name = "cp0",
		.handler = mips32_handle_cp0_command,
		.mode = COMMAND_EXEC,
		.usage = "[[reg_name|regnum select] [value]]",
		.help = "display/modify cp0 register",
	},
	{
		.name = "cpuinfo",
		.handler = mips32_handle_cpuinfo_command,
		.mode = COMMAND_EXEC,
		.help = "display CPU information",
		.usage = "",
	},
	{
		.name = "dsp",
		.handler = mips32_handle_dsp_command,
		.mode = COMMAND_EXEC,
		.help = "display or set DSP register; "
			"with no arguments, displays all registers and their values",
		.usage = "[[register_name] [value]]",
	},
	{
		.name = "scan_delay",
		.handler = mips32_handle_scan_delay_command,
		.mode = COMMAND_ANY,
		.help = "display/set scan delay in nano seconds",
		.usage = "[value]",
	},
	{
		.name = "ejtag_reg",
		.handler = mips32_handle_ejtag_reg_command,
		.mode = COMMAND_ANY,
		.help = "read ejtag registers",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration mips32_command_handlers[] = {
	{
		.name = "mips32",
		.mode = COMMAND_ANY,
		.help = "mips32 command group",
		.usage = "",
		.chain = mips32_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
