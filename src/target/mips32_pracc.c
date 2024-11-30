// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
 *                                                                         *
 *   Copyright (C) 2009 by David N. Claffey <dnclaffey@gmail.com>          *
 *                                                                         *
 *   Copyright (C) 2011 by Drasko DRASKOVIC                                *
 *   drasko.draskovic@gmail.com                                            *
 ***************************************************************************/

/*
 * This version has optimized assembly routines for 32 bit operations:
 * - read word
 * - write word
 * - write array of words
 *
 * One thing to be aware of is that the MIPS32 cpu will execute the
 * instruction after a branch instruction (one delay slot).
 *
 * For example:
 *  LW $2, ($5 +10)
 *  B foo
 *  LW $1, ($2 +100)
 *
 * The LW $1, ($2 +100) instruction is also executed. If this is
 * not wanted a NOP can be inserted:
 *
 *  LW $2, ($5 +10)
 *  B foo
 *  NOP
 *  LW $1, ($2 +100)
 *
 * or the code can be changed to:
 *
 *  B foo
 *  LW $2, ($5 +10)
 *  LW $1, ($2 +100)
 *
 * The original code contained NOPs. I have removed these and moved
 * the branches.
 *
 * These changes result in a 35% speed increase when programming an
 * external flash.
 *
 * More improvement could be gained if the registers do no need
 * to be preserved but in that case the routines should be aware
 * OpenOCD is used as a flash programmer or as a debug tool.
 *
 * Nico Coesel
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/align.h>
#include <helper/time_support.h>
#include <jtag/adapter.h>

#include "mips_cpu.h"
#include "mips32.h"
#include "mips32_pracc.h"

static int wait_for_pracc_rw(struct mips_ejtag *ejtag_info)
{
	int64_t then = timeval_ms();

	/* wait for the PrAcc to become "1" */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);

	while (1) {
		ejtag_info->pa_ctrl = ejtag_info->ejtag_ctrl;
		int retval = mips_ejtag_drscan_32(ejtag_info, &ejtag_info->pa_ctrl);
		if (retval != ERROR_OK)
			return retval;

		if (ejtag_info->pa_ctrl & EJTAG_CTRL_PRACC)
			break;

		int64_t timeout = timeval_ms() - then;
		if (timeout > 1000) {
			LOG_DEBUG("DEBUGMODULE: No memory access in progress!");
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	return ERROR_OK;
}

/* Shift in control and address for a new processor access, save them in ejtag_info */
static int mips32_pracc_read_ctrl_addr(struct mips_ejtag *ejtag_info)
{
	int retval = wait_for_pracc_rw(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);

	ejtag_info->pa_addr = 0;
	return  mips_ejtag_drscan_32(ejtag_info, &ejtag_info->pa_addr);
}

/* Finish processor access */
static void mips32_pracc_finish(struct mips_ejtag *ejtag_info)
{
	uint32_t ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
	mips_ejtag_drscan_32_out(ejtag_info, ctrl);
}

static int mips32_pracc_clean_text_jump(struct mips_ejtag *ejtag_info)
{
	uint32_t jt_code = MIPS32_J(ejtag_info->isa, MIPS32_PRACC_TEXT);
	pracc_swap16_array(ejtag_info, &jt_code, 1);
	/* do 3 0/nops to clean pipeline before a jump to pracc text, NOP in delay slot */
	for (int i = 0; i != 5; i++) {
		/* Wait for pracc */
		int retval = wait_for_pracc_rw(ejtag_info);
		if (retval != ERROR_OK)
			return retval;

		/* Data or instruction out */
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		uint32_t data = (i == 3) ? jt_code : MIPS32_NOP;
		mips_ejtag_drscan_32_out(ejtag_info, data);

		/* finish pa */
		mips32_pracc_finish(ejtag_info);
	}

	if (ejtag_info->mode != 0)	/* async mode support only for MIPS ... */
		return ERROR_OK;

	for (int i = 0; i != 2; i++) {
		int retval = mips32_pracc_read_ctrl_addr(ejtag_info);
		if (retval != ERROR_OK)
			return retval;

		if (ejtag_info->pa_addr != MIPS32_PRACC_TEXT) {	/* LEXRA/BMIPS ?, shift out another NOP, max 2 */
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
			mips_ejtag_drscan_32_out(ejtag_info, MIPS32_NOP);
			mips32_pracc_finish(ejtag_info);
		} else
			break;
	}

	return ERROR_OK;
}

static int mips32_pracc_exec(struct mips_ejtag *ejtag_info, struct pracc_queue_info *ctx,
					uint32_t *param_out, bool check_last)
{
	int code_count = 0;
	int store_pending = 0;		/* increases with every store instr at dmseg, decreases with every store pa */
	uint32_t max_store_addr = 0;	/* for store pa address testing */
	bool restart = 0;		/* restarting control */
	int restart_count = 0;
	uint32_t instr = 0;
	bool final_check = 0;		/* set to 1 if in final checks after function code shifted out */
	bool pass = 0;			/* to check the pass through pracc text after function code sent */
	int retval;

	while (1) {
		if (restart) {
			if (restart_count < 3) {					/* max 3 restarts allowed */
				retval = mips32_pracc_clean_text_jump(ejtag_info);
				if (retval != ERROR_OK)
					return retval;
			} else
				return ERROR_JTAG_DEVICE_ERROR;
			restart_count++;
			restart = 0;
			code_count = 0;
			LOG_DEBUG("restarting code");
		}

		retval = mips32_pracc_read_ctrl_addr(ejtag_info); /* update current pa info: control and address */
		if (retval != ERROR_OK)
			return retval;

		/* Check for read or write access */
		if (ejtag_info->pa_ctrl & EJTAG_CTRL_PRNW) {				/* write/store access */
			/* Check for pending store from a previous store instruction at dmseg */
			if (store_pending == 0) {
				LOG_DEBUG("unexpected write at address %" PRIx32, ejtag_info->pa_addr);
				if (code_count < 2) {	/* allow for restart */
					restart = 1;
					continue;
				} else
					return ERROR_JTAG_DEVICE_ERROR;
			} else {
				/* check address */
				if (ejtag_info->pa_addr < MIPS32_PRACC_PARAM_OUT ||
						ejtag_info->pa_addr > max_store_addr) {
					LOG_DEBUG("writing at unexpected address %" PRIx32, ejtag_info->pa_addr);
					return ERROR_JTAG_DEVICE_ERROR;
				}
			}
			/* read data */
			uint32_t data = 0;
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
			retval = mips_ejtag_drscan_32(ejtag_info, &data);
			if (retval != ERROR_OK)
				return retval;

			/* store data at param out, address based offset */
			param_out[(ejtag_info->pa_addr - MIPS32_PRACC_PARAM_OUT) / 4] = data;
			store_pending--;

		} else {					/* read/fetch access */
			if (!final_check) {			/* executing function code */
				/* check address */
				if (ejtag_info->pa_addr != (MIPS32_PRACC_TEXT + code_count * 4)) {
					LOG_DEBUG("reading at unexpected address %" PRIx32 ", expected %x",
							ejtag_info->pa_addr, MIPS32_PRACC_TEXT + code_count * 4);

					/* restart code execution only in some cases */
					if (code_count == 1 && ejtag_info->pa_addr == MIPS32_PRACC_TEXT &&
										restart_count == 0) {
						LOG_DEBUG("restarting, without clean jump");
						restart_count++;
						code_count = 0;
						continue;
					} else if (code_count < 2) {
						restart = 1;
						continue;
					}
					return ERROR_JTAG_DEVICE_ERROR;
				}
				/* check for store instruction at dmseg */
				uint32_t store_addr = ctx->pracc_list[code_count].addr;
				if (store_addr != 0) {
					if (store_addr > max_store_addr)
						max_store_addr = store_addr;
					store_pending++;
				}

				instr = ctx->pracc_list[code_count++].instr;
				if (code_count == ctx->code_count)	/* last instruction, start final check */
					final_check = 1;

			} else {	/* final check after function code shifted out */
					/* check address */
				if (ejtag_info->pa_addr == MIPS32_PRACC_TEXT) {
					if (!pass) {	/* first pass through pracc text */
						if (store_pending == 0)		/* done, normal exit */
							return ERROR_OK;
						pass = 1;		/* pracc text passed */
						code_count = 0;		/* restart code count */
					} else {
						LOG_DEBUG("unexpected second pass through pracc text");
						return ERROR_JTAG_DEVICE_ERROR;
					}
				} else {
					if (ejtag_info->pa_addr != (MIPS32_PRACC_TEXT + code_count * 4)) {
						LOG_DEBUG("unexpected read address in final check: %"
							PRIx32 ", expected: %x", ejtag_info->pa_addr,
							MIPS32_PRACC_TEXT + code_count * 4);
						return ERROR_JTAG_DEVICE_ERROR;
					}
				}
				if (!pass) {
					if ((code_count - ctx->code_count) > 1) { /* allow max 2 instr delay slot */
						LOG_DEBUG("failed to jump back to pracc text");
						return ERROR_JTAG_DEVICE_ERROR;
					}
				} else
					if (code_count > 10) {		/* enough, abandon */
						LOG_DEBUG("execution abandoned, store pending: %d", store_pending);
						return ERROR_JTAG_DEVICE_ERROR;
					}
				instr = MIPS32_NOP;	/* shift out NOPs instructions */
				code_count++;
			}

			/* Send instruction out */
			mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
			mips_ejtag_drscan_32_out(ejtag_info, instr);
		}
		/* finish processor access, let the processor eat! */
		mips32_pracc_finish(ejtag_info);

		if (final_check && !check_last)			/* last instr, don't check, execute and exit */
			return jtag_execute_queue();

		if (store_pending == 0 && pass) {	/* store access done, but after passing pracc text */
			LOG_DEBUG("warning: store access pass pracc text");
			return ERROR_OK;
		}
	}
}

inline void pracc_queue_init(struct pracc_queue_info *ctx)
{
	ctx->retval = ERROR_OK;
	ctx->code_count = 0;
	ctx->store_count = 0;
	ctx->max_code = 0;
	ctx->pracc_list = NULL;
	ctx->isa = ctx->ejtag_info->isa ? 1 : 0;
}

void pracc_add(struct pracc_queue_info *ctx, uint32_t addr, uint32_t instr)
{
	if (ctx->retval != ERROR_OK)	/* On previous out of memory, return */
		return;
	if (ctx->code_count == ctx->max_code) {
		void *p = realloc(ctx->pracc_list, sizeof(struct pa_list) * (ctx->max_code + PRACC_BLOCK));
		if (p) {
			ctx->max_code += PRACC_BLOCK;
			ctx->pracc_list = p;
		} else {
			ctx->retval = ERROR_FAIL;	/* Out of memory */
			return;
		}
	}
	ctx->pracc_list[ctx->code_count].instr = instr;
	ctx->pracc_list[ctx->code_count++].addr = addr;
	if (addr)
		ctx->store_count++;
}

static void pracc_add_li32(struct pracc_queue_info *ctx, uint32_t reg_num, uint32_t data, bool optimize)
{
	if (LOWER16(data) == 0 && optimize)
		pracc_add(ctx, 0, MIPS32_LUI(ctx->isa, reg_num, UPPER16(data)));	/* load only upper value */
	else if (UPPER16(data) == 0 && optimize)
		pracc_add(ctx, 0, MIPS32_ORI(ctx->isa, reg_num, 0, LOWER16(data)));	/* load only lower */
	else {
		pracc_add(ctx, 0, MIPS32_LUI(ctx->isa, reg_num, UPPER16(data)));	/* load upper and lower */
		pracc_add(ctx, 0, MIPS32_ORI(ctx->isa, reg_num, reg_num, LOWER16(data)));
	}
}

inline void pracc_queue_free(struct pracc_queue_info *ctx)
{
	free(ctx->pracc_list);
}

int mips32_pracc_queue_exec(struct mips_ejtag *ejtag_info, struct pracc_queue_info *ctx,
					uint32_t *buf, bool check_last)
{
	if (ctx->retval != ERROR_OK) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	if (ejtag_info->isa && ejtag_info->endianness)
		for (int i = 0; i != ctx->code_count; i++)
			ctx->pracc_list[i].instr = SWAP16(ctx->pracc_list[i].instr);

	if (ejtag_info->mode == 0)
		return mips32_pracc_exec(ejtag_info, ctx, buf, check_last);

	union scan_in {
		uint8_t scan_96[12];
		struct {
			uint8_t ctrl[4];
			uint8_t data[4];
			uint8_t addr[4];
		} scan_32;

	} *scan_in = malloc(sizeof(union scan_in) * (ctx->code_count + ctx->store_count));
	if (!scan_in) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	unsigned int num_clocks =
		((uint64_t)(ejtag_info->scan_delay) * adapter_get_speed_khz() + 500000) / 1000000;

	uint32_t ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ALL);

	int scan_count = 0;
	for (int i = 0; i != ctx->code_count; i++) {
		jtag_add_clocks(num_clocks);
		mips_ejtag_add_scan_96(ejtag_info, ejtag_ctrl, ctx->pracc_list[i].instr,
				       scan_in[scan_count++].scan_96);

		/* Check store address from previous instruction, if not the first */
		if (i > 0 && ctx->pracc_list[i - 1].addr) {
			jtag_add_clocks(num_clocks);
			mips_ejtag_add_scan_96(ejtag_info, ejtag_ctrl, 0, scan_in[scan_count++].scan_96);
		}
	}

	int retval = jtag_execute_queue();		/* execute queued scans */
	if (retval != ERROR_OK)
		goto exit;

	uint32_t fetch_addr = MIPS32_PRACC_TEXT;		/* start address */
	scan_count = 0;
	for (int i = 0; i != ctx->code_count; i++) {				/* verify every pracc access */
		/* check pracc bit */
		ejtag_ctrl = buf_get_u32(scan_in[scan_count].scan_32.ctrl, 0, 32);
		uint32_t addr = buf_get_u32(scan_in[scan_count].scan_32.addr, 0, 32);
		if (!(ejtag_ctrl & EJTAG_CTRL_PRACC)) {
			LOG_ERROR("Access not pending, count: %d", scan_count);
			retval = ERROR_FAIL;
			goto exit;
		}
		if (ejtag_ctrl & EJTAG_CTRL_PRNW) {
			LOG_ERROR("Not a fetch/read access, count: %d", scan_count);
			retval = ERROR_FAIL;
			goto exit;
		}
		if (addr != fetch_addr) {
			LOG_ERROR("Fetch addr mismatch, read: %" PRIx32 " expected: %" PRIx32 " count: %d",
					  addr, fetch_addr, scan_count);
			retval = ERROR_FAIL;
			goto exit;
		}
		fetch_addr += 4;
		scan_count++;

		/* check if previous instruction is a store instruction at dmesg */
		if (i > 0 && ctx->pracc_list[i - 1].addr) {
			uint32_t store_addr = ctx->pracc_list[i - 1].addr;
			ejtag_ctrl = buf_get_u32(scan_in[scan_count].scan_32.ctrl, 0, 32);
			addr = buf_get_u32(scan_in[scan_count].scan_32.addr, 0, 32);

			if (!(ejtag_ctrl & EJTAG_CTRL_PRNW)) {
				LOG_ERROR("Not a store/write access, count: %d", scan_count);
				retval = ERROR_FAIL;
				goto exit;
			}
			if (addr != store_addr) {
				LOG_ERROR("Store address mismatch, read: %" PRIx32 " expected: %" PRIx32 " count: %d",
							      addr, store_addr, scan_count);
				retval = ERROR_FAIL;
				goto exit;
			}
			int buf_index = (addr - MIPS32_PRACC_PARAM_OUT) / 4;
			buf[buf_index] = buf_get_u32(scan_in[scan_count].scan_32.data, 0, 32);
			scan_count++;
		}
	}
exit:
	free(scan_in);
	return retval;
}

static int mips32_pracc_read_u32(struct mips_ejtag *ejtag_info, uint32_t addr, uint32_t *buf)
{
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, PRACC_UPPER_BASE_ADDR));	/* $15 = MIPS32_PRACC_BASE_ADDR */
	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 8, UPPER16((addr + 0x8000)))); /* load  $8 with modified upper addr */
	pracc_add(&ctx, 0, MIPS32_LW(ctx.isa, 8, LOWER16(addr), 8));			/* lw $8, LOWER16(addr)($8) */
	if (mips32_cpu_support_sync(ejtag_info))
		pracc_add(&ctx, 0, MIPS32_SYNC(ctx.isa));
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET, 15));	/* sw $8,PRACC_OUT_OFFSET($15) */
	pracc_add_li32(&ctx, 8, ejtag_info->reg8, 0);				/* restore $8 */
	pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));		/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));				/* move COP0 DeSave to $15 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, buf, 1);
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_read_mem(struct mips_ejtag *ejtag_info, uint32_t addr, int size, int count, void *buf)
{
	if (count == 1 && size == 4)
		return mips32_pracc_read_u32(ejtag_info, addr, (uint32_t *)buf);

	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	uint32_t *data = NULL;
	if (size != 4) {
		data = malloc(256 * sizeof(uint32_t));
		if (!data) {
			LOG_ERROR("Out of memory");
			goto exit;
		}
	}

	uint32_t *buf32 = buf;
	uint16_t *buf16 = buf;
	uint8_t *buf8 = buf;

	while (count) {
		ctx.code_count = 0;
		ctx.store_count = 0;

		int this_round_count = (count > 256) ? 256 : count;
		uint32_t last_upper_base_addr = UPPER16((addr + 0x8000));

		pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, PRACC_UPPER_BASE_ADDR)); /* $15 = MIPS32_PRACC_BASE_ADDR */
		pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 9, last_upper_base_addr));	/* upper memory addr to $9 */

		for (int i = 0; i != this_round_count; i++) {			/* Main code loop */
			uint32_t upper_base_addr = UPPER16((addr + 0x8000));
			if (last_upper_base_addr != upper_base_addr) {	/* if needed, change upper addr in $9 */
				pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 9, upper_base_addr));
				last_upper_base_addr = upper_base_addr;
			}

			if (size == 4)				/* load from memory to $8 */
				pracc_add(&ctx, 0, MIPS32_LW(ctx.isa, 8, LOWER16(addr), 9));
			else if (size == 2)
				pracc_add(&ctx, 0, MIPS32_LHU(ctx.isa, 8, LOWER16(addr), 9));
			else
				pracc_add(&ctx, 0, MIPS32_LBU(ctx.isa, 8, LOWER16(addr), 9));

			if (mips32_cpu_support_sync(ejtag_info))
				pracc_add(&ctx, 0, MIPS32_SYNC(ctx.isa));
			pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + i * 4,			/* store $8 at param out */
					  MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET + i * 4, 15));
			addr += size;
		}
		pracc_add_li32(&ctx, 8, ejtag_info->reg8, 0);				/* restore $8 */
		pracc_add_li32(&ctx, 9, ejtag_info->reg9, 0);				/* restore $9 */

		pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));	/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));			/* restore $15 from DeSave */

		if (size == 4) {
			ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, buf32, 1);
			if (ctx.retval != ERROR_OK)
				goto exit;
			buf32 += this_round_count;
		} else {
			ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, data, 1);
			if (ctx.retval != ERROR_OK)
				goto exit;

			uint32_t *data_p = data;
			for (int i = 0; i != this_round_count; i++) {
				if (size == 2)
					*buf16++ = *data_p++;
				else
					*buf8++ = *data_p++;
			}
		}
		count -= this_round_count;
	}
exit:
	pracc_queue_free(&ctx);
	free(data);
	return ctx.retval;
}

int mips32_cp0_read(struct mips_ejtag *ejtag_info, uint32_t *val, uint32_t cp0_reg, uint32_t cp0_sel)
{
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, PRACC_UPPER_BASE_ADDR));	/* $15 = MIPS32_PRACC_BASE_ADDR */
	if (mips32_cpu_support_hazard_barrier(ejtag_info))
		pracc_add(&ctx, 0, MIPS32_EHB(ctx.isa));
	pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 8, cp0_reg, cp0_sel));		/* move cp0 reg / sel to $8 */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET, 15));	/* store $8 to pracc_out */
	pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));				/* restore $15 from DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 8, UPPER16(ejtag_info->reg8)));	/* restore upper 16 bits  of $8 */
	pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));		/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(ctx.isa, 8, 8, LOWER16(ejtag_info->reg8))); /* restore lower 16 bits of $8 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, val, 1);
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_cp0_write(struct mips_ejtag *ejtag_info, uint32_t val, uint32_t cp0_reg, uint32_t cp0_sel)
{
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	pracc_add_li32(&ctx, 15, val, 0);				/* Load val to $15 */

	pracc_add(&ctx, 0, MIPS32_MTC0(ctx.isa, 15, cp0_reg, cp0_sel));		/* write $15 to cp0 reg / sel */
	if (mips32_cpu_support_hazard_barrier(ejtag_info))
		pracc_add(&ctx, 0, MIPS32_EHB(ctx.isa));
	pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));		/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));			/* restore $15 from DeSave */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 1);
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_cp1_control_read(struct mips_ejtag *ejtag_info, uint32_t *val, uint32_t cp1_c_reg)
{
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, PRACC_UPPER_BASE_ADDR));	/* $15 = MIPS32_PRACC_BASE_ADDR */
	pracc_add(&ctx, 0, MIPS32_EHB(ctx.isa));
	pracc_add(&ctx, 0, MIPS32_CFC1(ctx.isa, 8, cp1_c_reg));			/* move cp1c reg to $8 */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET, 15));	/* store $8 to pracc_out */
	pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));				/* restore $15 from DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 8, UPPER16(ejtag_info->reg8)));	/* restore upper 16 bits  of $8 */
	pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));		/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(ctx.isa, 8, 8, LOWER16(ejtag_info->reg8))); /* restore lower 16 bits of $8 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, val, 1);
	pracc_queue_free(&ctx);
	return ctx.retval;
}

/**
 * \b mips32_pracc_sync_cache
 *
 * Synchronize Caches to Make Instruction Writes Effective
 * (ref. doc. MIPS32 Architecture For Programmers Volume II: The MIPS32 Instruction Set,
 *  Document Number: MD00086, Revision 2.00, June 9, 2003)
 *
 * When the instruction stream is written, the SYNCI instruction should be used
 * in conjunction with other instructions to make the newly-written instructions effective.
 *
 * Explanation :
 * A program that loads another program into memory is actually writing the D- side cache.
 * The instructions it has loaded can't be executed until they reach the I-cache.
 *
 * After the instructions have been written, the loader should arrange
 * to write back any containing D-cache line and invalidate any locations
 * already in the I-cache.
 *
 * If the cache coherency attribute (CCA) is set to zero, it's a write through cache, there is no need
 * to write back.
 *
 * In the latest MIPS32/64 CPUs, MIPS provides the synci instruction,
 * which does the whole job for a cache-line-sized chunk of the memory you just loaded:
 * That is, it arranges a D-cache write-back (if CCA = 3) and an I-cache invalidate.
 *
 * The line size is obtained with the rdhwr SYNCI_Step in release 2 or from cp0 config 1 register in release 1.
 */
static int mips32_pracc_synchronize_cache(struct mips_ejtag *ejtag_info,
					 uint32_t start_addr, uint32_t end_addr, int cached, int rel)
{
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	/** Find cache line size in bytes */
	uint32_t clsiz;
	if (rel) {	/* Release 2 (rel = 1) */
		pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, PRACC_UPPER_BASE_ADDR)); /* $15 = MIPS32_PRACC_BASE_ADDR */

		pracc_add(&ctx, 0, MIPS32_RDHWR(ctx.isa, 8, MIPS32_SYNCI_STEP)); /* load synci_step value to $8 */

		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET, 15));		/* store $8 to pracc_out */

		pracc_add_li32(&ctx, 8, ejtag_info->reg8, 0);				/* restore $8 */

		pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));	/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));			/* restore $15 from DeSave */

		ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, &clsiz, 1);
		if (ctx.retval != ERROR_OK)
			goto exit;

	} else {			/* Release 1 (rel = 0) */
		uint32_t conf;
		ctx.retval = mips32_cp0_read(ejtag_info, &conf, 16, 1);
		if (ctx.retval != ERROR_OK)
			goto exit;

		uint32_t dl = (conf & MIPS32_CONFIG1_DL_MASK) >> MIPS32_CONFIG1_DL_SHIFT;

		/* dl encoding : dl=1 => 4 bytes, dl=2 => 8 bytes, etc... max dl=6 => 128 bytes cache line size */
		clsiz = 0x2 << dl;
		if (dl == 0)
			clsiz = 0;
	}

	if (clsiz == 0)
		goto exit;  /* Nothing to do */

	/* make sure clsiz is power of 2 */
	if (!IS_PWR_OF_2(clsiz)) {
		LOG_DEBUG("clsiz must be power of 2");
		ctx.retval = ERROR_FAIL;
		goto exit;
	}

	/* make sure start_addr and end_addr have the same offset inside de cache line */
	start_addr |= clsiz - 1;
	end_addr |= clsiz - 1;

	ctx.code_count = 0;
	ctx.store_count = 0;

	int count = 0;
	uint32_t last_upper_base_addr = UPPER16((start_addr + 0x8000));

	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, last_upper_base_addr)); /* load upper memory base addr to $15 */

	while (start_addr <= end_addr) {						/* main loop */
		uint32_t upper_base_addr = UPPER16((start_addr + 0x8000));
		if (last_upper_base_addr != upper_base_addr) {		/* if needed, change upper addr in $15 */
			pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, upper_base_addr));
			last_upper_base_addr = upper_base_addr;
		}
		if (rel)			/* synci instruction, offset($15) */
			pracc_add(&ctx, 0, MIPS32_SYNCI(ctx.isa, LOWER16(start_addr), 15));

		else {
			if (cached == 3)	/* cache Hit_Writeback_D, offset($15) */
				pracc_add(&ctx, 0, MIPS32_CACHE(ctx.isa, MIPS32_CACHE_D_HIT_WRITEBACK,
							LOWER16(start_addr), 15));
			/* cache Hit_Invalidate_I, offset($15) */
			pracc_add(&ctx, 0, MIPS32_CACHE(ctx.isa, MIPS32_CACHE_I_HIT_INVALIDATE,
							LOWER16(start_addr), 15));
		}
		start_addr += clsiz;
		count++;
		if (count == 256 && start_addr <= end_addr) {			/* more ?, then execute code list */
			pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));	/* to start */
			pracc_add(&ctx, 0, MIPS32_NOP);					/* nop in delay slot */

			ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 1);
			if (ctx.retval != ERROR_OK)
				goto exit;

			ctx.code_count = 0;	/* reset counters for another loop */
			ctx.store_count = 0;
			count = 0;
		}
	}
	pracc_add(&ctx, 0, MIPS32_SYNC(ctx.isa));
	pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));		/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));				/* restore $15 from DeSave*/

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 1);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

static int mips32_pracc_write_mem_generic(struct mips_ejtag *ejtag_info,
		uint32_t addr, int size, int count, const void *buf)
{
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	pracc_queue_init(&ctx);

	const uint32_t *buf32 = buf;
	const uint16_t *buf16 = buf;
	const uint8_t *buf8 = buf;

	while (count) {
		ctx.code_count = 0;
		ctx.store_count = 0;

		int this_round_count = (count > 128) ? 128 : count;
		uint32_t last_upper_base_addr = UPPER16((addr + 0x8000));
			      /* load $15 with memory base address */
		pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, last_upper_base_addr));

		for (int i = 0; i != this_round_count; i++) {
			uint32_t upper_base_addr = UPPER16((addr + 0x8000));
			if (last_upper_base_addr != upper_base_addr) {	/* if needed, change upper address in $15*/
				pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 15, upper_base_addr));
				last_upper_base_addr = upper_base_addr;
			}

			if (size == 4) {
				pracc_add_li32(&ctx, 8, *buf32, 1);		/* load with li32, optimize */
				pracc_add(&ctx, 0, MIPS32_SW(ctx.isa, 8, LOWER16(addr), 15)); /* store word to mem */
				buf32++;

			} else if (size == 2) {
				pracc_add(&ctx, 0, MIPS32_ORI(ctx.isa, 8, 0, *buf16));		/* load lower value */
				pracc_add(&ctx, 0, MIPS32_SH(ctx.isa, 8, LOWER16(addr), 15)); /* store half word */
				buf16++;

			} else {
				pracc_add(&ctx, 0, MIPS32_ORI(ctx.isa, 8, 0, *buf8));		/* load lower value */
				pracc_add(&ctx, 0, MIPS32_SB(ctx.isa, 8, LOWER16(addr), 15));	/* store byte */
				buf8++;
			}
			addr += size;
		}

		pracc_add_li32(&ctx, 8, ejtag_info->reg8, 0);				/* restore $8 */

		pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));	/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(ctx.isa, 15, 31, 0));			/* restore $15 from DeSave */

		ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 1);
		if (ctx.retval != ERROR_OK)
			goto exit;
		count -= this_round_count;
	}
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_write_mem(struct mips_ejtag *ejtag_info, uint32_t addr, int size, int count, const void *buf)
{
	int retval = mips32_pracc_write_mem_generic(ejtag_info, addr, size, count, buf);
	if (retval != ERROR_OK)
		return retval;

	/**
	 * If we are in the cacheable region and cache is activated,
	 * we must clean D$ (if Cache Coherency Attribute is set to 3) + invalidate I$ after we did the write,
	 * so that changes do not continue to live only in D$ (if CCA = 3), but to be
	 * replicated in I$ also (maybe we wrote the instructions)
	 */
	uint32_t conf = 0;
	int cached = 0;

	if ((KSEGX(addr) == KSEG1) || ((addr >= 0xff200000) && (addr <= 0xff3fffff)))
		return retval; /*Nothing to do*/

	/* Reads Config0 */
	mips32_cp0_read(ejtag_info, &conf, 16, 0);

	switch (KSEGX(addr)) {
		case KUSEG:
			cached = (conf & MIPS32_CONFIG0_KU_MASK) >> MIPS32_CONFIG0_KU_SHIFT;
			break;
		case KSEG0:
			cached = (conf & MIPS32_CONFIG0_K0_MASK) >> MIPS32_CONFIG0_K0_SHIFT;
			break;
		case KSEG2:
		case KSEG3:
			cached = (conf & MIPS32_CONFIG0_K23_MASK) >> MIPS32_CONFIG0_K23_SHIFT;
			break;
		default:
			/* what ? */
			break;
	}

	/**
	 * Check cacheability bits coherency algorithm
	 * is the region cacheable or uncached.
	 * If cacheable we have to synchronize the cache
	 */
	if (cached == 3 || cached == 0) {		/* Write back cache or write through cache */
		uint32_t start_addr = addr;
		uint32_t end_addr = addr + count * size;
		uint32_t rel = (conf & MIPS32_CONFIG0_AR_MASK) >> MIPS32_CONFIG0_AR_SHIFT;
		/* FIXME: In MIPS Release 6, the encoding of CACHE instr has changed */
		if (rel > MIPS32_RELEASE_2) {
			LOG_DEBUG("Unsupported MIPS Release ( > 5)");
			return ERROR_FAIL;
		}
		retval = mips32_pracc_synchronize_cache(ejtag_info, start_addr, end_addr, cached, rel);
	} else {
		struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};

		pracc_queue_init(&ctx);
		if (mips32_cpu_support_sync(ejtag_info))
			pracc_add(&ctx, 0, MIPS32_SYNC(ctx.isa));
		if (mips32_cpu_support_hazard_barrier(ejtag_info))
			pracc_add(&ctx, 0, MIPS32_EHB(ctx.isa));
		pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));	/* jump to start */
		pracc_add(&ctx, 0, MIPS32_NOP);
		ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 1);
		if (ctx.retval != ERROR_OK) {
			LOG_ERROR("Unable to barrier");
			retval = ctx.retval;
		}
		pracc_queue_free(&ctx);
	}

	return retval;
}

int mips32_pracc_write_regs(struct mips32_common *mips32)
{
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	uint32_t *gprs = mips32->core_regs.gpr;
	uint32_t *c0rs = mips32->core_regs.cp0;
	bool fpu_in_64bit = ((c0rs[0] & BIT(MIPS32_CP0_STATUS_FR_SHIFT)) != 0);
	bool fp_enabled = ((c0rs[0] & BIT(MIPS32_CP0_STATUS_CU1_SHIFT)) != 0);
	bool dsp_enabled = ((c0rs[0] & BIT(MIPS32_CP0_STATUS_MX_SHIFT)) != 0);
	uint32_t rel = (ejtag_info->config[0] & MIPS32_CONFIG0_AR_MASK) >> MIPS32_CONFIG0_AR_SHIFT;

	pracc_queue_init(&ctx);

	uint32_t cp0_write_code[] = {
		MIPS32_MTC0(ctx.isa, 1, 12, 0),					/* move $1 to status */
		MIPS32_MTLO(ctx.isa, 1),						/* move $1 to lo */
		MIPS32_MTHI(ctx.isa, 1),						/* move $1 to hi */
		MIPS32_MTC0(ctx.isa, 1, 8, 0),					/* move $1 to badvaddr */
		MIPS32_MTC0(ctx.isa, 1, 13, 0),					/* move $1 to cause*/
		MIPS32_MTC0(ctx.isa, 1, 24, 0),					/* move $1 to depc (pc) */
	};

	uint32_t cp0_write_data[] = {
		/* status */
		c0rs[0],
		/* lo */
		gprs[32],
		/* hi */
		gprs[33],
		/* badvaddr */
		c0rs[1],
		/* cause */
		c0rs[2],
		/* depc (pc) */
		c0rs[3],
	};

	/* Write CP0 Status Register first, changes on EXL or ERL bits
	 * may lead to different behaviour on writing to other CP0 registers.
	 */
	for (size_t i = 0; i < ARRAY_SIZE(cp0_write_code); i++) {
		/* load CP0 value in $1 */
		pracc_add_li32(&ctx, 1, cp0_write_data[i], 0);
		/* write value from $1 to CP0 register */
		pracc_add(&ctx, 0, cp0_write_code[i]);
	}

	if (mips32_cpu_support_hazard_barrier(ejtag_info))
		pracc_add(&ctx, 0, MIPS32_EHB(ctx.isa));

	/* store FPRs */
	if (mips32->fp_imp && fp_enabled) {
		uint64_t *fprs = mips32->core_regs.fpr;
		if (fpu_in_64bit) {
			for (int i = 0; i != MIPS32_REG_FP_COUNT; i++) {
				uint32_t fp_lo = fprs[i] & 0xffffffff;
				uint32_t fp_hi = (fprs[i] >> 32) & 0xffffffff;
				pracc_add_li32(&ctx, 2, fp_lo, 0);
				pracc_add_li32(&ctx, 3, fp_hi, 0);
				pracc_add(&ctx, 0, MIPS32_MTC1(ctx.isa, 2, i));
				pracc_add(&ctx, 0, MIPS32_MTHC1(ctx.isa, 3, i));
			}
		} else {
			for (int i = 0; i != MIPS32_REG_FP_COUNT; i++) {
				uint32_t fp_lo = fprs[i] & 0xffffffff;
				pracc_add_li32(&ctx, 2, fp_lo, 0);
				pracc_add(&ctx, 0, MIPS32_MTC1(ctx.isa, 2, i));
			}
		}

		if (rel > MIPS32_RELEASE_1)
			pracc_add(&ctx, 0, MIPS32_EHB(ctx.isa));
	}

	/* Store DSP Accumulators */
	if (mips32->dsp_imp && dsp_enabled) {
		/* Struct of mips32_dsp_regs: {ac{hi, lo}1-3, dspctl} */
		uint32_t *dspr = mips32->core_regs.dsp;
		size_t dsp_regs = ARRAY_SIZE(mips32->core_regs.dsp);

		/* Starts from ac1, core_regs.dsp contains dspctl register, therefore - 1 */
		for (size_t index = 0; index != ((dsp_regs - 1) / 2); index++) {
			/* Every accumulator have 2 registers, hi and lo, and core_regs.dsp stores ac[1~3] */
			/* reads hi[ac] from core_regs array */
			pracc_add_li32(&ctx, 2, dspr[index * 2], 0);
			/* reads lo[ac] from core_regs array */
			pracc_add_li32(&ctx, 3, dspr[(index * 2) + 1], 0);

			/* Write to accumulator 1~3 and index starts from 0, therefore ac = index + 1 */
			size_t ac = index + 1;
			pracc_add(&ctx, 0, MIPS32_DSP_MTHI(2, ac));
			pracc_add(&ctx, 0, MIPS32_DSP_MTLO(3, ac));
		}

		/* DSPCTL is the last element of register store */
		pracc_add_li32(&ctx, 2, dspr[6], 0);
		pracc_add(&ctx, 0, MIPS32_DSP_WRDSP(2, 0x1F));

		if (rel > MIPS32_RELEASE_1)
			pracc_add(&ctx, 0, MIPS32_EHB(ctx.isa));
	}

	/* load registers 2 to 31 with li32, optimize */
	for (int i = 2; i < 32; i++)
		pracc_add_li32(&ctx, i, gprs[i], 1);

	/* load $15 in DeSave */
	pracc_add(&ctx, 0, MIPS32_MTC0(ctx.isa, 15, 31, 0));
	/* load upper half word in $1 */
	pracc_add(&ctx, 0, MIPS32_LUI(ctx.isa, 1, UPPER16((gprs[1]))));
	/* jump to start */
	pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));
	/* load lower half word in $1 */
	pracc_add(&ctx, 0, MIPS32_ORI(ctx.isa, 1, 1, LOWER16((gprs[1]))));

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL, 1);

	ejtag_info->reg8 = gprs[8];
	ejtag_info->reg9 = gprs[9];
	pracc_queue_free(&ctx);
	return ctx.retval;
}

/* Saves content in `$1` to `DeSave(cp0.31.0)` and loads `MIPS32_PRACC_BASE_ADDR` into `$1` */
static void mips32_pracc_store_regs_set_base_addr(struct pracc_queue_info *ctx)
{
	/* move $1 to COP0 DeSave */
	pracc_add(ctx, 0, MIPS32_MTC0(ctx->isa, 1, 31, 0));
	/* $1 = MIP32_PRACC_BASE_ADDR */
	pracc_add(ctx, 0, MIPS32_LUI(ctx->isa, 1, PRACC_UPPER_BASE_ADDR));
}

/* This function assumes the address for saving is stored in `$1`.
 * And that action is performed in `mips32_pracc_set_save_base_addr`.
 */
static void mips32_pracc_store_regs_gpr(struct pracc_queue_info *ctx, unsigned int offset_gpr)
{
	for (int i = 2; i != 32; i++)
		pracc_add(ctx, MIPS32_PRACC_PARAM_OUT + offset_gpr + (i * 4),
				MIPS32_SW(ctx->isa, i, PRACC_OUT_OFFSET + offset_gpr + (i * 4), 1));
}

static void mips32_pracc_store_regs_lohi(struct pracc_queue_info *ctx)
{
	uint32_t lohi_read_code[] = {
		MIPS32_MFLO(ctx->isa, 8),	/* move lo to $8 */
		MIPS32_MFHI(ctx->isa, 8),	/* move hi to $8 */
	};

	/* store lo & hi */
	for (int i = 0; i < 2; i++) {
		/* load COP0 needed registers to $8 */
		pracc_add(ctx, 0, lohi_read_code[i]);
		/* store $8 at PARAM OUT */
		pracc_add(ctx, MIPS32_PRACC_PARAM_OUT + (i + 32) * 4,
					MIPS32_SW(ctx->isa, 8, PRACC_OUT_OFFSET + (i + 32) * 4, 1));
	}
}

/* Saves CP0 registers [status, badvaddr, cause, depc] */
static void mips32_pracc_store_regs_cp0_context(struct pracc_queue_info *ctx, unsigned int offset_cp0)
{
	uint32_t cp0_read_code[] = {
		MIPS32_MFC0(ctx->isa, 8, 12, 0),	/* move status to $8 */
		MIPS32_MFC0(ctx->isa, 8, 8, 0),	/* move badvaddr to $8 */
		MIPS32_MFC0(ctx->isa, 8, 13, 0),	/* move cause to $8 */
		MIPS32_MFC0(ctx->isa, 8, 24, 0),	/* move depc (pc) to $8 */
	};

	/* store cp0 */
	for (size_t i = 0; i < ARRAY_SIZE(cp0_read_code); i++) {
		size_t offset = offset_cp0 + (i * 4);

		/* load COP0 needed registers to $8 */
		pracc_add(ctx, 0, cp0_read_code[i]);
		/* store $8 at PARAM OUT */
		pracc_add(ctx, MIPS32_PRACC_PARAM_OUT + offset,
					MIPS32_SW(ctx->isa, 8, PRACC_OUT_OFFSET + offset, 1));
	}
}

/* Loads original content of $1 into $8,
 * then store it to the batch data access address.
 * Finally it restores $1 from DeSave.
 */
static void mips32_pracc_store_regs_restore(struct pracc_queue_info *ctx)
{
	/* move DeSave to $8, reg1 value */
	pracc_add(ctx, 0, MIPS32_MFC0(ctx->isa, 8, 31, 0));
	/* store reg1 value from $8 to param out */
	pracc_add(ctx, MIPS32_PRACC_PARAM_OUT + 4,
			  MIPS32_SW(ctx->isa, 8, PRACC_OUT_OFFSET + 4, 1));

	/* move COP0 DeSave to $1, restore reg1 */
	pracc_add(ctx, 0, MIPS32_MFC0(ctx->isa, 1, 31, 0));
}

/* This function performs following actions:
 * Saves `$1` to `DeSave`,
 * then load `PRACC_UPPER_BASE_ADDR` for saving the register data structure into `$1`,
 * Saves `$2` ~ `$31` to `PRACC_UPPER_BASE_ADDR + offset_gpr`
 * Saves HI and LO,
 * Saves necessary cp0 registers.
*/
static void mips32_pracc_store_regs(struct pracc_queue_info *ctx,
					unsigned int offset_gpr, unsigned int offset_cp0)
{
	mips32_pracc_store_regs_set_base_addr(ctx);
	mips32_pracc_store_regs_gpr(ctx, offset_gpr);
	mips32_pracc_store_regs_lohi(ctx);
	mips32_pracc_store_regs_cp0_context(ctx, offset_cp0);
	mips32_pracc_store_regs_restore(ctx);
}

int mips32_pracc_read_regs(struct mips32_common *mips32)
{
	struct mips_ejtag *ejtag_info = &mips32->ejtag_info;
	struct pracc_queue_info ctx = {.ejtag_info = ejtag_info};
	struct mips32_core_regs *core_regs = &mips32->core_regs;
	unsigned int offset_gpr = ((uint8_t *)&core_regs->gpr[0]) - (uint8_t *)core_regs;
	unsigned int offset_cp0 = ((uint8_t *)&core_regs->cp0[0]) - (uint8_t *)core_regs;
	unsigned int offset_fpr = ((uint8_t *)&core_regs->fpr[0]) - (uint8_t *)core_regs;
	unsigned int offset_fpcr = ((uint8_t *)&core_regs->fpcr[0]) - (uint8_t *)core_regs;
	unsigned int offset_dsp = ((uint8_t *)&core_regs->dsp[0]) - (uint8_t *)core_regs;
	bool fp_enabled, dsp_enabled;

	/*
	 * This procedure has to be in 3 distinctive steps, because we can
	 * only know whether FP and DSP are enabled after reading CP0.
	 *
	 * Step 1: Read everything except CP1 and DSP stuff
	 * Step 2: Read CP1 stuff if FP is implemented
	 * Step 3: Read DSP registers if dsp is implemented
	 */

	pracc_queue_init(&ctx);

	mips32_pracc_store_regs(&ctx, offset_gpr, offset_cp0);

	/* jump to start */
	pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));
	/* load $15 in DeSave */
	pracc_add(&ctx, 0, MIPS32_MTC0(ctx.isa, 15, 31, 0));

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, (uint32_t *)&mips32->core_regs, 1);

	pracc_queue_free(&ctx);

	/* reg8 is saved but not restored, next called function should restore it */
	ejtag_info->reg8 = mips32->core_regs.gpr[8];
	ejtag_info->reg9 = mips32->core_regs.gpr[9];

	if (ctx.retval != ERROR_OK)
		return ctx.retval;

	/* we only care if FP is actually impl'd and if cp1 is enabled */
	/* since we already read cp0 in the prev step */
	/* now we know what's in cp0.status */
	fp_enabled = (mips32->core_regs.cp0[0] & BIT(MIPS32_CP0_STATUS_CU1_SHIFT)) != 0;
	if (mips32->fp_imp && fp_enabled) {
		pracc_queue_init(&ctx);

		mips32_pracc_store_regs_set_base_addr(&ctx);

		/* FCSR */
		pracc_add(&ctx, 0, MIPS32_CFC1(ctx.isa, 8, 31));
		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + offset_fpcr,
				  MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET + offset_fpcr, 1));

		/* FIR */
		pracc_add(&ctx, 0, MIPS32_CFC1(ctx.isa, 8, 0));
		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + offset_fpcr + 4,
				  MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET + offset_fpcr + 4, 1));

		/* f0 to f31 */
		if (mips32->fpu_in_64bit) {
			for (int i = 0; i != 32; i++) {
				size_t offset = offset_fpr + (i * 8);
				/* current pracc implementation (or EJTAG itself) only supports 32b access */
				/* so there is no way to use SDC1 */

				/* lower half */
				pracc_add(&ctx, 0, MIPS32_MFC1(ctx.isa, 8, i));
				pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + offset,
						  MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET + offset, 1));

				/* upper half */
				pracc_add(&ctx, 0, MIPS32_MFHC1(ctx.isa, 8, i));
				pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + offset + 4,
						  MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET + offset + 4, 1));
			}
		} else {
			for (int i = 0; i != 32; i++) {
				size_t offset = offset_fpr + (i * 8);
				pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + offset,
						  MIPS32_SWC1(ctx.isa, i, PRACC_OUT_OFFSET + offset, 1));
			}
		}

		mips32_pracc_store_regs_restore(&ctx);

		/* jump to start */
		pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));
		/* load $15 in DeSave */
		pracc_add(&ctx, 0, MIPS32_MTC0(ctx.isa, 15, 31, 0));

		ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, (uint32_t *)&mips32->core_regs, 1);

		pracc_queue_free(&ctx);
	}

	dsp_enabled = (mips32->core_regs.cp0[MIPS32_REG_C0_STATUS_INDEX] & BIT(MIPS32_CP0_STATUS_MX_SHIFT)) != 0;
	if (mips32->dsp_imp && dsp_enabled) {
		pracc_queue_init(&ctx);

		mips32_pracc_store_regs_set_base_addr(&ctx);

		/* Struct of mips32_dsp_regs[7]: {ac{hi, lo}1-3, dspctl} */
		size_t dsp_regs = ARRAY_SIZE(mips32->core_regs.dsp);
		/* Starts from ac1, core_regs.dsp have dspctl register, therefore - 1 */
		for (size_t index = 0; index != ((dsp_regs - 1) / 2); index++) {
			/* Every accumulator have 2 registers, hi&lo, and core_regs.dsp stores ac[1~3] */
			/* Reads offset of hi[ac] from core_regs array */
			size_t offset_hi = offset_dsp + ((index * 2) * sizeof(uint32_t));
			/* Reads offset of lo[ac] from core_regs array */
			size_t offset_lo = offset_dsp + (((index * 2) + 1) * sizeof(uint32_t));

			/* DSP Ac registers starts from 1 and index starts from 0, therefore ac = index + 1 */
			size_t ac = index + 1;
			pracc_add(&ctx, 0, MIPS32_DSP_MFHI(8, ac));
			pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + offset_hi,
					MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET + offset_hi, 1));
			pracc_add(&ctx, 0, MIPS32_DSP_MFLO(8, ac));
			pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + offset_lo,
					MIPS32_SW(ctx.isa, 8, PRACC_OUT_OFFSET + offset_lo, 1));
		}

		/* DSPCTL is the last element of register store */
		pracc_add(&ctx, 0, MIPS32_DSP_RDDSP(8, 0x3F));
		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + offset_dsp + ((dsp_regs - 1) * sizeof(uint32_t)),
				  MIPS32_SW(ctx.isa, 8,
							PRACC_OUT_OFFSET + offset_dsp + ((dsp_regs - 1) * sizeof(uint32_t)), 1));

		mips32_pracc_store_regs_restore(&ctx);

		/* jump to start */
		pracc_add(&ctx, 0, MIPS32_B(ctx.isa, NEG16((ctx.code_count + 1) << ctx.isa)));
		/* load $15 in DeSave */
		pracc_add(&ctx, 0, MIPS32_MTC0(ctx.isa, 15, 31, 0));

		ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, (uint32_t *)&mips32->core_regs, 1);

		pracc_queue_free(&ctx);
	}
	return ctx.retval;
}

/**
 * mips32_pracc_fastdata_xfer_synchronize_cache - Synchronize cache for fast data transfer
 * @param[in] ejtag_info: EJTAG information structure
 * @param[in] addr: Starting address for cache synchronization
 * @param[in] size: Size of each data element
 * @param[in] count: Number of data elements
 *
 * @brief Synchronizes the cache for fast data transfer based on
 * the specified address and cache configuration.
 * If the region is cacheable (write-back cache or write-through cache),
 * it synchronizes the cache for the specified range.
 * The synchronization is performed using the MIPS32 cache synchronization function.
 *
 * @return ERROR_OK on success; error code on failure.
 */
static int mips32_pracc_fastdata_xfer_synchronize_cache(struct mips_ejtag *ejtag_info,
													uint32_t addr, int size, int count)
{
	int retval = ERROR_OK;

	if ((KSEGX(addr) == KSEG1) || (addr >= 0xff200000 && addr <= 0xff3fffff)) // DESEG?
		return retval; /*Nothing to do*/

	int cached = 0;
	uint32_t conf = 0;

	mips32_cp0_read(ejtag_info, &conf, 16, 0);

	switch (KSEGX(addr)) {
		case KUSEG:
			cached = (conf & MIPS32_CONFIG0_KU_MASK) >> MIPS32_CONFIG0_KU_SHIFT;
			break;
		case KSEG0:
			cached = (conf & MIPS32_CONFIG0_K0_MASK) >> MIPS32_CONFIG0_K0_SHIFT;
			break;
		case KSEG2:
		case KSEG3:
			cached = (conf & MIPS32_CONFIG0_K23_MASK) >> MIPS32_CONFIG0_K23_SHIFT;
			break;
		default:
			/* what ? */
			break;
	}

    /**
	 * Check cacheability bits coherency algorithm
	 * is the region cacheable or uncached.
	 * If cacheable we have to synchronize the cache
	 */
	if (cached == 3 || cached == 0) {		/* Write back cache or write through cache */
		uint32_t start_addr = addr;
		uint32_t end_addr = addr + count * size;
		uint32_t rel = (conf & MIPS32_CONFIG0_AR_MASK) >> MIPS32_CONFIG0_AR_SHIFT;
		/* FIXME: In MIPS Release 6, the encoding of CACHE instr has changed */
		if (rel > MIPS32_RELEASE_2) {
			LOG_DEBUG("Unsupported MIPS Release ( > 5)");
			return ERROR_FAIL;
		}
		retval = mips32_pracc_synchronize_cache(ejtag_info, start_addr, end_addr, cached, rel);
	}

	return retval;
}

/* fastdata upload/download requires an initialized working area
 * to load the download code; it should not be called otherwise
 * fetch order from the fastdata area
 * 1. start addr
 * 2. end addr
 * 3. data ...
 */
int mips32_pracc_fastdata_xfer(struct mips_ejtag *ejtag_info, struct working_area *source,
		int write_t, uint32_t addr, int count, uint32_t *buf)
{
	uint32_t isa = ejtag_info->isa ? 1 : 0;
	uint32_t handler_code[] = {
		/* r15 points to the start of this code */
		MIPS32_SW(isa, 8, MIPS32_FASTDATA_HANDLER_SIZE - 4, 15),
		MIPS32_SW(isa, 9, MIPS32_FASTDATA_HANDLER_SIZE - 8, 15),
		MIPS32_SW(isa, 10, MIPS32_FASTDATA_HANDLER_SIZE - 12, 15),
		MIPS32_SW(isa, 11, MIPS32_FASTDATA_HANDLER_SIZE - 16, 15),
		/* start of fastdata area in t0 */
		MIPS32_LUI(isa, 8, UPPER16(MIPS32_PRACC_FASTDATA_AREA)),
		MIPS32_ORI(isa, 8, 8, LOWER16(MIPS32_PRACC_FASTDATA_AREA)),
		MIPS32_LW(isa, 9, 0, 8),					/* start addr in t1 */
		mips32_cpu_support_sync(ejtag_info) ? MIPS32_SYNC(isa) : MIPS32_NOP,				/* barrier for ordering */
		MIPS32_LW(isa, 10, 0, 8),					/* end addr to t2 */
		mips32_cpu_support_sync(ejtag_info) ? MIPS32_SYNC(isa) : MIPS32_NOP,				/* barrier for ordering */
		/* loop: */
		write_t ? MIPS32_LW(isa, 11, 0, 8) : MIPS32_LW(isa, 11, 0, 9),	/* from xfer area : from memory */
		write_t ? MIPS32_SW(isa, 11, 0, 9) : MIPS32_SW(isa, 11, 0, 8),	/* to memory      : to xfer area */

		mips32_cpu_support_sync(ejtag_info) ? MIPS32_SYNC(isa) : MIPS32_NOP,				/* barrier for ordering */

		MIPS32_BNE(isa, 10, 9, NEG16(4 << isa)),			/* bne $t2,t1,loop */
		MIPS32_ADDI(isa, 9, 9, 4),					/* addi t1,t1,4 */

		MIPS32_LW(isa, 8, MIPS32_FASTDATA_HANDLER_SIZE - 4, 15),
		MIPS32_LW(isa, 9, MIPS32_FASTDATA_HANDLER_SIZE - 8, 15),
		MIPS32_LW(isa, 10, MIPS32_FASTDATA_HANDLER_SIZE - 12, 15),
		MIPS32_LW(isa, 11, MIPS32_FASTDATA_HANDLER_SIZE - 16, 15),

		MIPS32_LUI(isa, 15, UPPER16(MIPS32_PRACC_TEXT)),
		MIPS32_ORI(isa, 15, 15, LOWER16(MIPS32_PRACC_TEXT) | isa),	/* isa bit for JR instr */
		mips32_cpu_support_hazard_barrier(ejtag_info)
			? MIPS32_JRHB(isa, 15)
			: MIPS32_JR(isa, 15),								/* jr start */
		MIPS32_MFC0(isa, 15, 31, 0),					/* move COP0 DeSave to $15 */
	};

	if (source->size < MIPS32_FASTDATA_HANDLER_SIZE)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	pracc_swap16_array(ejtag_info, handler_code, ARRAY_SIZE(handler_code));
		/* write program into RAM */
	if (write_t != ejtag_info->fast_access_save) {
		mips32_pracc_write_mem(ejtag_info, source->address, 4, ARRAY_SIZE(handler_code), handler_code);
		/* save previous operation to speed to any consecutive read/writes */
		ejtag_info->fast_access_save = write_t;
	}

	LOG_DEBUG("%s using 0x%.8" TARGET_PRIxADDR " for write handler", __func__, source->address);

	uint32_t jmp_code[] = {
		MIPS32_LUI(isa, 15, UPPER16(source->address)),			/* load addr of jump in $15 */
		MIPS32_ORI(isa, 15, 15, LOWER16(source->address) | isa),	/* isa bit for JR instr */
		mips32_cpu_support_hazard_barrier(ejtag_info)
			? MIPS32_JRHB(isa, 15)
			: MIPS32_JR(isa, 15),	/* jump to ram program */
		isa ? MIPS32_XORI(isa, 15, 15, 1) : MIPS32_NOP,	/* drop isa bit, needed for LW/SW instructions */
	};

	pracc_swap16_array(ejtag_info, jmp_code, ARRAY_SIZE(jmp_code));

	/* execute jump code, with no address check */
	for (unsigned int i = 0; i < ARRAY_SIZE(jmp_code); i++) {
		int retval = wait_for_pracc_rw(ejtag_info);
		if (retval != ERROR_OK)
			return retval;

		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		mips_ejtag_drscan_32_out(ejtag_info, jmp_code[i]);

		/* Clear the access pending bit (let the processor eat!) */
		mips32_pracc_finish(ejtag_info);
	}

	/* wait PrAcc pending bit for FASTDATA write, read address */
	int retval = mips32_pracc_read_ctrl_addr(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	/* next fetch to dmseg should be in FASTDATA_AREA, check */
	if (ejtag_info->pa_addr != MIPS32_PRACC_FASTDATA_AREA)
		return ERROR_FAIL;

	/* Send the load start address */
	uint32_t val = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_FASTDATA);
	mips_ejtag_fastdata_scan(ejtag_info, 1, &val);

	retval = wait_for_pracc_rw(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	/* Send the load end address */
	val = addr + (count - 1) * 4;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_FASTDATA);
	mips_ejtag_fastdata_scan(ejtag_info, 1, &val);

	unsigned int num_clocks = 0;	/* like in legacy code */
	if (ejtag_info->mode != 0)
		num_clocks = ((uint64_t)(ejtag_info->scan_delay) * adapter_get_speed_khz() + 500000) / 1000000;

	for (int i = 0; i < count; i++) {
		jtag_add_clocks(num_clocks);
		mips_ejtag_fastdata_scan(ejtag_info, write_t, buf++);
	}

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("fastdata load failed");
		return retval;
	}

	retval = mips32_pracc_read_ctrl_addr(ejtag_info);
	if (retval != ERROR_OK)
		return retval;

	if (ejtag_info->pa_addr != MIPS32_PRACC_TEXT)
		LOG_ERROR("mini program did not return to start");

	return mips32_pracc_fastdata_xfer_synchronize_cache(ejtag_info, addr, 4, count);
}
