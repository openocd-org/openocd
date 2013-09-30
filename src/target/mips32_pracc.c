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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
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
 * I also moved the PRACC_STACK to 0xFF204000. This allows
 * the use of 16 bits offsets to get pointers to the input
 * and output area relative to the stack. Note that the stack
 * isn't really a stack (the stack pointer is not 'moving')
 * but a FIFO simulated in software.
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

#include <helper/time_support.h>

#include "mips32.h"
#include "mips32_pracc.h"

struct mips32_pracc_context {
	uint32_t *local_iparam;
	int num_iparam;
	uint32_t *local_oparam;
	int num_oparam;
	const uint32_t *code;
	int code_len;
	uint32_t stack[32];
	int stack_offset;
	struct mips_ejtag *ejtag_info;
};

static int mips32_pracc_sync_cache(struct mips_ejtag *ejtag_info,
		uint32_t start_addr, uint32_t end_addr);
static int mips32_pracc_clean_invalidate_cache(struct mips_ejtag *ejtag_info,
		uint32_t start_addr, uint32_t end_addr);

static int wait_for_pracc_rw(struct mips_ejtag *ejtag_info, uint32_t *ctrl)
{
	uint32_t ejtag_ctrl;
	long long then = timeval_ms();
	int timeout;
	int retval;

	/* wait for the PrAcc to become "1" */
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);

	while (1) {
		ejtag_ctrl = ejtag_info->ejtag_ctrl;
		retval = mips_ejtag_drscan_32(ejtag_info, &ejtag_ctrl);
		if (retval != ERROR_OK)
			return retval;

		if (ejtag_ctrl & EJTAG_CTRL_PRACC)
			break;

		timeout = timeval_ms() - then;
		if (timeout > 1000) {
			LOG_DEBUG("DEBUGMODULE: No memory access in progress!");
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	*ctrl = ejtag_ctrl;
	return ERROR_OK;
}

static int mips32_pracc_exec_read(struct mips32_pracc_context *ctx, uint32_t address)
{
	struct mips_ejtag *ejtag_info = ctx->ejtag_info;
	int offset;
	uint32_t ejtag_ctrl, data;

	if ((address >= MIPS32_PRACC_PARAM_IN)
		&& (address < MIPS32_PRACC_PARAM_IN + ctx->num_iparam * 4)) {
		offset = (address - MIPS32_PRACC_PARAM_IN) / 4;
		data = ctx->local_iparam[offset];
	} else if ((address >= MIPS32_PRACC_PARAM_OUT)
		&& (address < MIPS32_PRACC_PARAM_OUT + ctx->num_oparam * 4)) {
		offset = (address - MIPS32_PRACC_PARAM_OUT) / 4;
		data = ctx->local_oparam[offset];
	} else if ((address >= MIPS32_PRACC_TEXT)
		&& (address < MIPS32_PRACC_TEXT + ctx->code_len * 4)) {
		offset = (address - MIPS32_PRACC_TEXT) / 4;
		data = ctx->code[offset];
	} else if (address == MIPS32_PRACC_STACK) {
		if (ctx->stack_offset <= 0) {
			LOG_ERROR("Error: Pracc stack out of bounds");
			return ERROR_JTAG_DEVICE_ERROR;
		}
		/* save to our debug stack */
		data = ctx->stack[--ctx->stack_offset];
	} else if (address >= 0xFF200000) {
		/* CPU keeps reading at the end of execution.
		 * If we after 0xF0000000  address range, we can use
		 * one shot jump instruction.
		 * Since this instruction is limited to
		 * 26bit, we need to do some magic to fit it to our needs. */
		LOG_DEBUG("Reading unexpected address. Jump to 0xFF200200\n");
		data = MIPS32_J((0x0FFFFFFF & 0xFF200200) >> 2);
	} else {
		LOG_ERROR("Error reading unexpected address 0x%8.8" PRIx32 "", address);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	/* Send the data out */
	mips_ejtag_set_instr(ctx->ejtag_info, EJTAG_INST_DATA);
	mips_ejtag_drscan_32_out(ctx->ejtag_info, data);

	/* Clear the access pending bit (let the processor eat!) */
	ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	mips_ejtag_set_instr(ctx->ejtag_info, EJTAG_INST_CONTROL);
	mips_ejtag_drscan_32_out(ctx->ejtag_info, ejtag_ctrl);

	return jtag_execute_queue();
}

static int mips32_pracc_exec_write(struct mips32_pracc_context *ctx, uint32_t address)
{
	uint32_t ejtag_ctrl, data;
	int offset;
	struct mips_ejtag *ejtag_info = ctx->ejtag_info;
	int retval;

	mips_ejtag_set_instr(ctx->ejtag_info, EJTAG_INST_DATA);
	retval = mips_ejtag_drscan_32(ctx->ejtag_info, &data);
	if (retval != ERROR_OK)
		return retval;

	/* Clear access pending bit */
	ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	mips_ejtag_set_instr(ctx->ejtag_info, EJTAG_INST_CONTROL);
	mips_ejtag_drscan_32_out(ctx->ejtag_info, ejtag_ctrl);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	if ((address >= MIPS32_PRACC_PARAM_OUT)
		&& (address < MIPS32_PRACC_PARAM_OUT + ctx->num_oparam * 4)) {
		offset = (address - MIPS32_PRACC_PARAM_OUT) / 4;
		ctx->local_oparam[offset] = data;
	} else if (address == MIPS32_PRACC_STACK) {
		if (ctx->stack_offset >= 32) {
			LOG_ERROR("Error: Pracc stack out of bounds");
			return ERROR_JTAG_DEVICE_ERROR;
		}
		/* save data onto our stack */
		ctx->stack[ctx->stack_offset++] = data;
	} else {
		LOG_ERROR("Error writing unexpected address 0x%8.8" PRIx32 "", address);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

int mips32_pracc_exec(struct mips_ejtag *ejtag_info, int code_len, const uint32_t *code,
		int num_param_in, uint32_t *param_in, int num_param_out, uint32_t *param_out, int cycle)
{
	uint32_t ejtag_ctrl;
	uint32_t address;
	struct mips32_pracc_context ctx;
	int retval;
	int pass = 0;

	ctx.local_iparam = param_in;
	ctx.local_oparam = param_out;
	ctx.num_iparam = num_param_in;
	ctx.num_oparam = num_param_out;
	ctx.code = code;
	ctx.code_len = code_len;
	ctx.ejtag_info = ejtag_info;
	ctx.stack_offset = 0;

	while (1) {
		retval = wait_for_pracc_rw(ejtag_info, &ejtag_ctrl);
		if (retval != ERROR_OK)
			return retval;

		address = 0;
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
		retval = mips_ejtag_drscan_32(ejtag_info, &address);
		if (retval != ERROR_OK)
			return retval;

		/* Check for read or write */
		if (ejtag_ctrl & EJTAG_CTRL_PRNW) {
			retval = mips32_pracc_exec_write(&ctx, address);
			if (retval != ERROR_OK)
				return retval;
		} else {
			/* Check to see if its reading at the debug vector. The first pass through
			 * the module is always read at the vector, so the first one we allow.  When
			 * the second read from the vector occurs we are done and just exit. */
			if ((address == MIPS32_PRACC_TEXT) && (pass++))
				break;

			retval = mips32_pracc_exec_read(&ctx, address);
			if (retval != ERROR_OK)
				return retval;
		}

		if (cycle == 0)
			break;
	}

	/* stack sanity check */
	if (ctx.stack_offset != 0)
		LOG_DEBUG("Pracc Stack not zero");

	return ERROR_OK;
}

inline void pracc_queue_init(struct pracc_queue_info *ctx)
{
	ctx->retval = ERROR_OK;
	ctx->code_count = 0;
	ctx->store_count = 0;

	ctx->pracc_list = malloc(2 * ctx->max_code * sizeof(uint32_t));
	if (ctx->pracc_list == NULL) {
		LOG_ERROR("Out of memory");
		ctx->retval = ERROR_FAIL;
	}
}

inline void pracc_add(struct pracc_queue_info *ctx, uint32_t addr, uint32_t instr)
{
	ctx->pracc_list[ctx->max_code + ctx->code_count] = addr;
	ctx->pracc_list[ctx->code_count++] = instr;
	if (addr)
		ctx->store_count++;
}

inline void pracc_queue_free(struct pracc_queue_info *ctx)
{
	if (ctx->code_count > ctx->max_code)	/* Only for internal check, will be erased */
		LOG_ERROR("Internal error, code count: %d > max code: %d", ctx->code_count, ctx->max_code);
	if (ctx->pracc_list != NULL)
		free(ctx->pracc_list);
}

int mips32_pracc_queue_exec(struct mips_ejtag *ejtag_info, struct pracc_queue_info *ctx, uint32_t *buf)
{
	if (ejtag_info->mode == 0)
		return mips32_pracc_exec(ejtag_info, ctx->code_count, ctx->pracc_list, 0, NULL,
				  ctx->store_count, buf, ctx->code_count - 1);

	union scan_in {
		uint8_t scan_96[12];
		struct {
			uint8_t ctrl[4];
			uint8_t data[4];
			uint8_t addr[4];
		} scan_32;

	} *scan_in = malloc(sizeof(union scan_in) * (ctx->code_count + ctx->store_count));
	if (scan_in == NULL) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	unsigned num_clocks =
		((uint64_t)(ejtag_info->scan_delay) * jtag_get_speed_khz() + 500000) / 1000000;

	uint32_t ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ALL);

	int scan_count = 0;
	for (int i = 0; i != 2 * ctx->code_count; i++) {
		uint32_t data = 0;
		if (i & 1u) {			/* Check store address from previous instruction, if not the first */
			if (i < 2 || 0 == ctx->pracc_list[ctx->max_code + (i / 2) - 1])
				continue;
		} else
			data = ctx->pracc_list[i / 2];

		jtag_add_clocks(num_clocks);
		mips_ejtag_add_scan_96(ejtag_info, ejtag_ctrl, data, scan_in[scan_count++].scan_96);
	}

	int retval = jtag_execute_queue();		/* execute queued scans */
	if (retval != ERROR_OK)
		goto exit;

	uint32_t fetch_addr = MIPS32_PRACC_TEXT;		/* start address */
	scan_count = 0;
	for (int i = 0; i != 2 * ctx->code_count; i++) {				/* verify every pracc access */
		uint32_t store_addr = 0;
		if (i & 1u) {			/* Read store addres from previous instruction, if not the first */
			store_addr = ctx->pracc_list[ctx->max_code + (i / 2) - 1];
			if (i < 2 || 0 == store_addr)
				continue;
		}

		ejtag_ctrl = buf_get_u32(scan_in[scan_count].scan_32.ctrl, 0, 32);
		if (!(ejtag_ctrl & EJTAG_CTRL_PRACC)) {
			LOG_ERROR("Error: access not pending  count: %d", scan_count);
			retval = ERROR_FAIL;
			goto exit;
		}

		uint32_t addr = buf_get_u32(scan_in[scan_count].scan_32.addr, 0, 32);

		if (store_addr != 0) {
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

		} else {
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
		}
		scan_count++;
	}
exit:
	free(scan_in);
	return retval;
}

int mips32_pracc_read_u32(struct mips_ejtag *ejtag_info, uint32_t addr, uint32_t *buf)
{
	struct pracc_queue_info ctx = {.max_code = 9};
	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));					/* move $15 to COP0 DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(15, PRACC_UPPER_BASE_ADDR));			/* $15 = MIPS32_PRACC_BASE_ADDR */
	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16((addr + 0x8000))));		/* load  $8 with modified upper address */
	pracc_add(&ctx, 0, MIPS32_LW(8, LOWER16(addr), 8));				/* lw $8, LOWER16(addr)($8) */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(8, PRACC_OUT_OFFSET, 15));			/* sw $8,PRACC_OUT_OFFSET($15) */
	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 of $8 */
	pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));		/* restore lower 16 of $8 */
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* move COP0 DeSave to $15 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, buf);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_read_mem(struct mips_ejtag *ejtag_info, uint32_t addr, int size, int count, void *buf)
{
	if (count == 1 && size == 4)
		return mips32_pracc_read_u32(ejtag_info, addr, (uint32_t *)buf);

	uint32_t *data = NULL;
	struct pracc_queue_info ctx = {.max_code = 256 * 3 + 9 + 1};	/* alloc memory for the worst case */
	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	if (size != 4) {
		data = malloc(256 * sizeof(uint32_t));
		if (data == NULL) {
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

		pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));					/* save $15 in DeSave */
		pracc_add(&ctx, 0, MIPS32_LUI(15, PRACC_UPPER_BASE_ADDR));			/* $15 = MIPS32_PRACC_BASE_ADDR */
		pracc_add(&ctx, 0, MIPS32_LUI(9, last_upper_base_addr));		/* load the upper memory address in $9 */

		for (int i = 0; i != this_round_count; i++) {			/* Main code loop */
			uint32_t upper_base_addr = UPPER16((addr + 0x8000));
			if (last_upper_base_addr != upper_base_addr) {			/* if needed, change upper address in $9 */
				pracc_add(&ctx, 0, MIPS32_LUI(9, upper_base_addr));
				last_upper_base_addr = upper_base_addr;
			}

			if (size == 4)
				pracc_add(&ctx, 0, MIPS32_LW(8, LOWER16(addr), 9));		/* load from memory to $8 */
			else if (size == 2)
				pracc_add(&ctx, 0, MIPS32_LHU(8, LOWER16(addr), 9));
			else
				pracc_add(&ctx, 0, MIPS32_LBU(8, LOWER16(addr), 9));

			pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + i * 4,
					  MIPS32_SW(8, PRACC_OUT_OFFSET + i * 4, 15));		/* store $8 at param out */
			addr += size;
		}
		pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 bits of reg 8 */
		pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));	/* restore lower 16 bits of reg 8 */
		pracc_add(&ctx, 0, MIPS32_LUI(9, UPPER16(ejtag_info->reg9)));		/* restore upper 16 bits of reg 9 */
		pracc_add(&ctx, 0, MIPS32_ORI(9, 9, LOWER16(ejtag_info->reg9)));	/* restore lower 16 bits of reg 9 */

		pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));				/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* restore $15 from DeSave */

		if (size == 4) {
			ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, buf32);
			if (ctx.retval != ERROR_OK)
				goto exit;
			buf32 += this_round_count;
		} else {
			ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, data);
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
	if (data != NULL)
		free(data);
	return ctx.retval;
}

int mips32_cp0_read(struct mips_ejtag *ejtag_info, uint32_t *val, uint32_t cp0_reg, uint32_t cp0_sel)
{
	struct pracc_queue_info ctx = {.max_code = 8};
	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));					/* move $15 to COP0 DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(15, PRACC_UPPER_BASE_ADDR));			/* $15 = MIPS32_PRACC_BASE_ADDR */
	pracc_add(&ctx, 0, MIPS32_MFC0(8, 0, 0) | (cp0_reg << 11) | cp0_sel);	/* move COP0 [cp0_reg select] to $8 */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT,
				MIPS32_SW(8, PRACC_OUT_OFFSET, 15));			/* store $8 to pracc_out */
	pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* move COP0 DeSave to $15 */
	pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 bits  of $8 */
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));		/* restore lower 16 bits of $8 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, val);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;

	/**
	 * Note that our input parametes cp0_reg and cp0_sel
	 * are numbers (not gprs) which make part of mfc0 instruction opcode.
	 *
	 * These are not fix, but can be different for each mips32_cp0_read() function call,
	 * and that is why we must insert them directly into opcode,
	 * i.e. we can not pass it on EJTAG microprogram stack (via param_in),
	 * and put them into the gprs later from MIPS32_PRACC_STACK
	 * because mfc0 do not use gpr as a parameter for the cp0_reg and select part,
	 * but plain (immediate) number.
	 *
	 * MIPS32_MTC0 is implemented via MIPS32_R_INST macro.
	 * In order to insert our parameters, we must change rd and funct fields.
	 *
	 * code[2] |= (cp0_reg << 11) | cp0_sel;   change rd and funct of MIPS32_R_INST macro
	 **/
}

int mips32_cp0_write(struct mips_ejtag *ejtag_info, uint32_t val, uint32_t cp0_reg, uint32_t cp0_sel)
{
	struct pracc_queue_info ctx = {.max_code = 6};
	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));					/* move $15 to COP0 DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(15, UPPER16(val)));				/* Load val to $15 */
	pracc_add(&ctx, 0, MIPS32_ORI(15, 15, LOWER16(val)));

	pracc_add(&ctx, 0, MIPS32_MTC0(15, 0, 0) | (cp0_reg << 11) | cp0_sel);	/* write cp0 reg / sel */

	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));					/* move COP0 DeSave to $15 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;

	/**
	 * Note that MIPS32_MTC0 macro is implemented via MIPS32_R_INST macro.
	 * In order to insert our parameters, we must change rd and funct fields.
	 * code[3] |= (cp0_reg << 11) | cp0_sel;   change rd and funct fields of MIPS32_R_INST macro
	 **/
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
 * You can do that with cache instructions, but those instructions are only available in kernel mode,
 * and a loader writing instructions for the use of its own process need not be privileged software.
 *
 * In the latest MIPS32/64 CPUs, MIPS provides the synci instruction,
 * which does the whole job for a cache-line-sized chunk of the memory you just loaded:
 * That is, it arranges a D-cache write-back and an I-cache invalidate.
 *
 * To employ synci at user level, you need to know the size of a cache line,
 * and that can be obtained with a rdhwr SYNCI_Step
 * from one of the standard “hardware registers”.
 */
static int mips32_pracc_sync_cache(struct mips_ejtag *ejtag_info,
		uint32_t start_addr, uint32_t end_addr)
{
	static const uint32_t code[] = {
															/* start: */
		MIPS32_MTC0(15, 31, 0),								/* move $15 to COP0 DeSave */
		MIPS32_LUI(15, UPPER16(MIPS32_PRACC_STACK)),		/* $15 = MIPS32_PRACC_STACK */
		MIPS32_ORI(15, 15, LOWER16(MIPS32_PRACC_STACK)),
		MIPS32_SW(8, 0, 15),								/* sw $8,($15) */
		MIPS32_SW(9, 0, 15),								/* sw $9,($15) */
		MIPS32_SW(10, 0, 15),								/* sw $10,($15) */
		MIPS32_SW(11, 0, 15),								/* sw $11,($15) */

		MIPS32_LUI(8, UPPER16(MIPS32_PRACC_PARAM_IN)),		/* $8 = MIPS32_PRACC_PARAM_IN */
		MIPS32_ORI(8, 8, LOWER16(MIPS32_PRACC_PARAM_IN)),
		MIPS32_LW(9, 0, 8),									/* Load write start_addr to $9 */
		MIPS32_LW(10, 4, 8),								/* Load write end_addr to $10 */

		MIPS32_RDHWR(11, MIPS32_SYNCI_STEP),				/* $11 = MIPS32_SYNCI_STEP */
		MIPS32_BEQ(11, 0, 6),								/* beq $11, $0, end */
		MIPS32_NOP,
															/* synci_loop : */
		MIPS32_SYNCI(0, 9),									/* synci 0($9) */
		MIPS32_SLTU(8, 10, 9),								/* sltu $8, $10, $9  # $8 = $10 < $9 ? 1 : 0 */
		MIPS32_BNE(8, 0, NEG16(3)),							/* bne $8, $0, synci_loop */
		MIPS32_ADDU(9, 9, 11),								/* $9 += MIPS32_SYNCI_STEP */
		MIPS32_SYNC,
															/* end: */
		MIPS32_LW(11, 0, 15),								/* lw $11,($15) */
		MIPS32_LW(10, 0, 15),								/* lw $10,($15) */
		MIPS32_LW(9, 0, 15),								/* lw $9,($15) */
		MIPS32_LW(8, 0, 15),								/* lw $8,($15) */
		MIPS32_B(NEG16(24)),								/* b start */
		MIPS32_MFC0(15, 31, 0),								/* move COP0 DeSave to $15 */
	};

	/* TODO remove array */
	uint32_t *param_in = malloc(2 * sizeof(uint32_t));
	int retval;
	param_in[0] = start_addr;
	param_in[1] = end_addr;

	retval = mips32_pracc_exec(ejtag_info, ARRAY_SIZE(code), code, 2, param_in, 0, NULL, 1);

	free(param_in);

	return retval;
}

/**
 * \b mips32_pracc_clean_invalidate_cache
 *
 * Writeback D$ and Invalidate I$
 * so that the instructions written can be visible to CPU
 */
static int mips32_pracc_clean_invalidate_cache(struct mips_ejtag *ejtag_info,
													uint32_t start_addr, uint32_t end_addr)
{
	static const uint32_t code[] = {
															/* start: */
		MIPS32_MTC0(15, 31, 0),								/* move $15 to COP0 DeSave */
		MIPS32_LUI(15, UPPER16(MIPS32_PRACC_STACK)),		/* $15 = MIPS32_PRACC_STACK */
		MIPS32_ORI(15, 15, LOWER16(MIPS32_PRACC_STACK)),
		MIPS32_SW(8, 0, 15),								/* sw $8,($15) */
		MIPS32_SW(9, 0, 15),								/* sw $9,($15) */
		MIPS32_SW(10, 0, 15),								/* sw $10,($15) */
		MIPS32_SW(11, 0, 15),								/* sw $11,($15) */

		MIPS32_LUI(8, UPPER16(MIPS32_PRACC_PARAM_IN)),		/* $8 = MIPS32_PRACC_PARAM_IN */
		MIPS32_ORI(8, 8, LOWER16(MIPS32_PRACC_PARAM_IN)),
		MIPS32_LW(9, 0, 8),									/* Load write start_addr to $9 */
		MIPS32_LW(10, 4, 8),								/* Load write end_addr to $10 */
		MIPS32_LW(11, 8, 8),								/* Load write clsiz to $11 */

															/* cache_loop: */
		MIPS32_SLTU(8, 10, 9),								/* sltu $8, $10, $9  :  $8 <- $10 < $9 ? */
		MIPS32_BGTZ(8, 6),									/* bgtz $8, end */
		MIPS32_NOP,

		MIPS32_CACHE(MIPS32_CACHE_D_HIT_WRITEBACK, 0, 9),		/* cache Hit_Writeback_D, 0($9) */
		MIPS32_CACHE(MIPS32_CACHE_I_HIT_INVALIDATE, 0, 9),	/* cache Hit_Invalidate_I, 0($9) */

		MIPS32_ADDU(9, 9, 11),								/* $9 += $11 */

		MIPS32_B(NEG16(7)),									/* b cache_loop */
		MIPS32_NOP,
															/* end: */
		MIPS32_LW(11, 0, 15),								/* lw $11,($15) */
		MIPS32_LW(10, 0, 15),								/* lw $10,($15) */
		MIPS32_LW(9, 0, 15),								/* lw $9,($15) */
		MIPS32_LW(8, 0, 15),								/* lw $8,($15) */
		MIPS32_B(NEG16(25)),								/* b start */
		MIPS32_MFC0(15, 31, 0),								/* move COP0 DeSave to $15 */
	};

	/**
	 * Find cache line size in bytes
	 */
	uint32_t conf;
	uint32_t dl, clsiz;

	mips32_cp0_read(ejtag_info, &conf, 16, 1);
	dl = (conf & MIPS32_CONFIG1_DL_MASK) >> MIPS32_CONFIG1_DL_SHIFT;

	/* dl encoding : dl=1 => 4 bytes, dl=2 => 8 bytes, etc... */
	clsiz = 0x2 << dl;

	/* TODO remove array */
	uint32_t *param_in = malloc(3 * sizeof(uint32_t));
	int retval;
	param_in[0] = start_addr;
	param_in[1] = end_addr;
	param_in[2] = clsiz;

	retval = mips32_pracc_exec(ejtag_info, ARRAY_SIZE(code), code, 3, param_in, 0, NULL, 1);

	free(param_in);

	return retval;
}

static int mips32_pracc_write_mem_generic(struct mips_ejtag *ejtag_info,
		uint32_t addr, int size, int count, const void *buf)
{
	struct pracc_queue_info ctx = {.max_code = 128 * 3 + 6 + 1};	/* alloc memory for the worst case */
	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	const uint32_t *buf32 = buf;
	const uint16_t *buf16 = buf;
	const uint8_t *buf8 = buf;

	while (count) {
		ctx.code_count = 0;
		ctx.store_count = 0;
		int this_round_count = (count > 128) ? 128 : count;
		uint32_t last_upper_base_addr = UPPER16((addr + 0x8000));

		pracc_add(&ctx, 0, MIPS32_MTC0(15, 31, 0));				/* save $15 in DeSave */
		pracc_add(&ctx, 0, MIPS32_LUI(15, last_upper_base_addr));		/* load $15 with memory base address */

		for (int i = 0; i != this_round_count; i++) {
			uint32_t upper_base_addr = UPPER16((addr + 0x8000));
			if (last_upper_base_addr != upper_base_addr) {
				pracc_add(&ctx, 0, MIPS32_LUI(15, upper_base_addr));	/* if needed, change upper address in $15*/
				last_upper_base_addr = upper_base_addr;
			}

			if (size == 4) {			/* for word writes check if one half word is 0 and load it accordingly */
				if (LOWER16(*buf32) == 0)
					pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(*buf32)));		/* load only upper value */
				else if (UPPER16(*buf32) == 0)
						pracc_add(&ctx, 0, MIPS32_ORI(8, 0, LOWER16(*buf32)));	/* load only lower */
				else {
					pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(*buf32)));		/* load upper and lower */
					pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(*buf32)));
				}
				pracc_add(&ctx, 0, MIPS32_SW(8, LOWER16(addr), 15));		/* store word to memory */
				buf32++;

			} else if (size == 2) {
				pracc_add(&ctx, 0, MIPS32_ORI(8, 0, *buf16));		/* load lower value */
				pracc_add(&ctx, 0, MIPS32_SH(8, LOWER16(addr), 15));	/* store half word to memory */
				buf16++;

			} else {
				pracc_add(&ctx, 0, MIPS32_ORI(8, 0, *buf8));		/* load lower value */
				pracc_add(&ctx, 0, MIPS32_SB(8, LOWER16(addr), 15));	/* store byte to memory */
				buf8++;
			}
			addr += size;
		}

		pracc_add(&ctx, 0, MIPS32_LUI(8, UPPER16(ejtag_info->reg8)));		/* restore upper 16 bits of reg 8 */
		pracc_add(&ctx, 0, MIPS32_ORI(8, 8, LOWER16(ejtag_info->reg8)));	/* restore lower 16 bits of reg 8 */

		pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));				/* jump to start */
		pracc_add(&ctx, 0, MIPS32_MFC0(15, 31, 0));				/* restore $15 from DeSave */

		ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);
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
	 * If we are in the cachable regoion and cache is activated,
	 * we must clean D$ + invalidate I$ after we did the write,
	 * so that changes do not continue to live only in D$, but to be
	 * replicated in I$ also (maybe we wrote the istructions)
	 */
	uint32_t conf = 0;
	int cached = 0;

	if ((KSEGX(addr) == KSEG1) || ((addr >= 0xff200000) && (addr <= 0xff3fffff)))
		return retval; /*Nothing to do*/

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
	 * Check cachablitiy bits coherency algorithm -
	 * is the region cacheable or uncached.
	 * If cacheable we have to synchronize the cache
	 */
	if (cached == 0x3) {
		uint32_t start_addr, end_addr;
		uint32_t rel;

		start_addr = addr;
		end_addr = addr + count * size;

		/** select cache synchronisation mechanism based on Architecture Release */
		rel = (conf & MIPS32_CONFIG0_AR_MASK) >> MIPS32_CONFIG0_AR_SHIFT;
		switch (rel) {
			case MIPS32_ARCH_REL1:
				/* MIPS32/64 Release 1 - we must use cache instruction */
				mips32_pracc_clean_invalidate_cache(ejtag_info, start_addr, end_addr);
				break;
			case MIPS32_ARCH_REL2:
				/* MIPS32/64 Release 2 - we can use synci instruction */
				mips32_pracc_sync_cache(ejtag_info, start_addr, end_addr);
				break;
			default:
				/* what ? */
				break;
		}
	}

	return retval;
}

int mips32_pracc_write_regs(struct mips_ejtag *ejtag_info, uint32_t *regs)
{
	static const uint32_t cp0_write_code[] = {
		MIPS32_MTC0(1, 12, 0),							/* move $1 to status */
		MIPS32_MTLO(1),									/* move $1 to lo */
		MIPS32_MTHI(1),									/* move $1 to hi */
		MIPS32_MTC0(1, 8, 0),							/* move $1 to badvaddr */
		MIPS32_MTC0(1, 13, 0),							/* move $1 to cause*/
		MIPS32_MTC0(1, 24, 0),							/* move $1 to depc (pc) */
	};

	struct pracc_queue_info ctx = {.max_code = 37 * 2 + 6 + 1};
	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	/* load registers 2 to 31 with lui and ori instructions, check if some instructions can be saved */
	for (int i = 2; i < 32; i++) {
		if (LOWER16((regs[i])) == 0)					/* if lower half word is 0, lui instruction only */
			pracc_add(&ctx, 0, MIPS32_LUI(i, UPPER16((regs[i]))));
		else if (UPPER16((regs[i])) == 0)					/* if upper half word is 0, ori with $0 only*/
			pracc_add(&ctx, 0, MIPS32_ORI(i, 0, LOWER16((regs[i]))));
		else {									/* default, load with lui and ori instructions */
			pracc_add(&ctx, 0, MIPS32_LUI(i, UPPER16((regs[i]))));
			pracc_add(&ctx, 0, MIPS32_ORI(i, i, LOWER16((regs[i]))));
		}
	}

	for (int i = 0; i != 6; i++) {
		pracc_add(&ctx, 0, MIPS32_LUI(1, UPPER16((regs[i + 32]))));		/* load CPO value in $1, with lui and ori */
		pracc_add(&ctx, 0, MIPS32_ORI(1, 1, LOWER16((regs[i + 32]))));
		pracc_add(&ctx, 0, cp0_write_code[i]);					/* write value from $1 to CPO register */
	}

	pracc_add(&ctx, 0, MIPS32_LUI(1, UPPER16((regs[1]))));			/* load upper half word in $1 */
	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_ORI(1, 1, LOWER16((regs[1]))));		/* load lower half word in $1 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, NULL);

	ejtag_info->reg8 = regs[8];
	ejtag_info->reg9 = regs[9];
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
}

int mips32_pracc_read_regs(struct mips_ejtag *ejtag_info, uint32_t *regs)
{
	static int cp0_read_code[] = {
		MIPS32_MFC0(8, 12, 0),							/* move status to $8 */
		MIPS32_MFLO(8),									/* move lo to $8 */
		MIPS32_MFHI(8),									/* move hi to $8 */
		MIPS32_MFC0(8, 8, 0),							/* move badvaddr to $8 */
		MIPS32_MFC0(8, 13, 0),							/* move cause to $8 */
		MIPS32_MFC0(8, 24, 0),							/* move depc (pc) to $8 */
	};

	struct pracc_queue_info ctx = {.max_code = 48};
	pracc_queue_init(&ctx);
	if (ctx.retval != ERROR_OK)
		goto exit;

	pracc_add(&ctx, 0, MIPS32_MTC0(1, 31, 0));						/* move $1 to COP0 DeSave */
	pracc_add(&ctx, 0, MIPS32_LUI(1, PRACC_UPPER_BASE_ADDR));				/* $1 = MIP32_PRACC_BASE_ADDR */

	for (int i = 2; i != 32; i++)					/* store GPR's 2 to 31 */
		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + (i * 4),
				  MIPS32_SW(i, PRACC_OUT_OFFSET + (i * 4), 1));

	for (int i = 0; i != 6; i++) {
		pracc_add(&ctx, 0, cp0_read_code[i]);				/* load COP0 needed registers to $8 */
		pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + (i + 32) * 4,			/* store $8 at PARAM OUT */
				  MIPS32_SW(8, PRACC_OUT_OFFSET + (i + 32) * 4, 1));
	}
	pracc_add(&ctx, 0, MIPS32_MFC0(8, 31, 0));					/* move DeSave to $8, reg1 value */
	pracc_add(&ctx, MIPS32_PRACC_PARAM_OUT + 4,					/* store reg1 value from $8 to param out */
			  MIPS32_SW(8, PRACC_OUT_OFFSET + 4, 1));

	pracc_add(&ctx, 0, MIPS32_B(NEG16(ctx.code_count + 1)));					/* jump to start */
	pracc_add(&ctx, 0, MIPS32_MFC0(1, 31, 0));					/* move COP0 DeSave to $1, restore reg1 */

	if (ejtag_info->mode == 0)
		ctx.store_count++;	/* Needed by legacy code, due to offset from reg0 */

	ctx.retval = mips32_pracc_queue_exec(ejtag_info, &ctx, regs);

	ejtag_info->reg8 = regs[8];	/* reg8 is saved but not restored, next called function should restore it */
	ejtag_info->reg9 = regs[9];
exit:
	pracc_queue_free(&ctx);
	return ctx.retval;
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
	uint32_t handler_code[] = {
		/* caution when editing, table is modified below */
		/* r15 points to the start of this code */
		MIPS32_SW(8, MIPS32_FASTDATA_HANDLER_SIZE - 4, 15),
		MIPS32_SW(9, MIPS32_FASTDATA_HANDLER_SIZE - 8, 15),
		MIPS32_SW(10, MIPS32_FASTDATA_HANDLER_SIZE - 12, 15),
		MIPS32_SW(11, MIPS32_FASTDATA_HANDLER_SIZE - 16, 15),
		/* start of fastdata area in t0 */
		MIPS32_LUI(8, UPPER16(MIPS32_PRACC_FASTDATA_AREA)),
		MIPS32_ORI(8, 8, LOWER16(MIPS32_PRACC_FASTDATA_AREA)),
		MIPS32_LW(9, 0, 8),								/* start addr in t1 */
		MIPS32_LW(10, 0, 8),							/* end addr to t2 */
														/* loop: */
		/* 8 */ MIPS32_LW(11, 0, 0),					/* lw t3,[t8 | r9] */
		/* 9 */ MIPS32_SW(11, 0, 0),					/* sw t3,[r9 | r8] */
		MIPS32_BNE(10, 9, NEG16(3)),					/* bne $t2,t1,loop */
		MIPS32_ADDI(9, 9, 4),							/* addi t1,t1,4 */

		MIPS32_LW(8, MIPS32_FASTDATA_HANDLER_SIZE - 4, 15),
		MIPS32_LW(9, MIPS32_FASTDATA_HANDLER_SIZE - 8, 15),
		MIPS32_LW(10, MIPS32_FASTDATA_HANDLER_SIZE - 12, 15),
		MIPS32_LW(11, MIPS32_FASTDATA_HANDLER_SIZE - 16, 15),

		MIPS32_LUI(15, UPPER16(MIPS32_PRACC_TEXT)),
		MIPS32_ORI(15, 15, LOWER16(MIPS32_PRACC_TEXT)),
		MIPS32_JR(15),								/* jr start */
		MIPS32_MFC0(15, 31, 0),						/* move COP0 DeSave to $15 */
	};

	uint32_t jmp_code[] = {
		MIPS32_MTC0(15, 31, 0),			/* move $15 to COP0 DeSave */
		/* 1 */ MIPS32_LUI(15, 0),		/* addr of working area added below */
		/* 2 */ MIPS32_ORI(15, 15, 0),	/* addr of working area added below */
		MIPS32_JR(15),					/* jump to ram program */
		MIPS32_NOP,
	};

	int retval, i;
	uint32_t val, ejtag_ctrl, address;

	if (source->size < MIPS32_FASTDATA_HANDLER_SIZE)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	if (write_t) {
		handler_code[8] = MIPS32_LW(11, 0, 8);	/* load data from probe at fastdata area */
		handler_code[9] = MIPS32_SW(11, 0, 9);	/* store data to RAM @ r9 */
	} else {
		handler_code[8] = MIPS32_LW(11, 0, 9);	/* load data from RAM @ r9 */
		handler_code[9] = MIPS32_SW(11, 0, 8);	/* store data to probe at fastdata area */
	}

	/* write program into RAM */
	if (write_t != ejtag_info->fast_access_save) {
		mips32_pracc_write_mem_generic(ejtag_info, source->address, 4, ARRAY_SIZE(handler_code), handler_code);
		/* save previous operation to speed to any consecutive read/writes */
		ejtag_info->fast_access_save = write_t;
	}

	LOG_DEBUG("%s using 0x%.8" PRIx32 " for write handler", __func__, source->address);

	jmp_code[1] |= UPPER16(source->address);
	jmp_code[2] |= LOWER16(source->address);

	for (i = 0; i < (int) ARRAY_SIZE(jmp_code); i++) {
		retval = wait_for_pracc_rw(ejtag_info, &ejtag_ctrl);
		if (retval != ERROR_OK)
			return retval;

		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_DATA);
		mips_ejtag_drscan_32_out(ejtag_info, jmp_code[i]);

		/* Clear the access pending bit (let the processor eat!) */
		ejtag_ctrl = ejtag_info->ejtag_ctrl & ~EJTAG_CTRL_PRACC;
		mips_ejtag_set_instr(ejtag_info, EJTAG_INST_CONTROL);
		mips_ejtag_drscan_32_out(ejtag_info, ejtag_ctrl);
	}

	/* wait PrAcc pending bit for FASTDATA write */
	retval = wait_for_pracc_rw(ejtag_info, &ejtag_ctrl);
	if (retval != ERROR_OK)
		return retval;

	/* next fetch to dmseg should be in FASTDATA_AREA, check */
	address = 0;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
	retval = mips_ejtag_drscan_32(ejtag_info, &address);
	if (retval != ERROR_OK)
		return retval;

	if (address != MIPS32_PRACC_FASTDATA_AREA)
		return ERROR_FAIL;

	/* Send the load start address */
	val = addr;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_FASTDATA);
	mips_ejtag_fastdata_scan(ejtag_info, 1, &val);

	retval = wait_for_pracc_rw(ejtag_info, &ejtag_ctrl);
	if (retval != ERROR_OK)
		return retval;

	/* Send the load end address */
	val = addr + (count - 1) * 4;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_FASTDATA);
	mips_ejtag_fastdata_scan(ejtag_info, 1, &val);

	unsigned num_clocks = 0;	/* like in legacy code */
	if (ejtag_info->mode != 0)
		num_clocks = ((uint64_t)(ejtag_info->scan_delay) * jtag_get_speed_khz() + 500000) / 1000000;

	for (i = 0; i < count; i++) {
		jtag_add_clocks(num_clocks);
		retval = mips_ejtag_fastdata_scan(ejtag_info, write_t, buf++);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		LOG_ERROR("fastdata load failed");
		return retval;
	}

	retval = wait_for_pracc_rw(ejtag_info, &ejtag_ctrl);
	if (retval != ERROR_OK)
		return retval;

	address = 0;
	mips_ejtag_set_instr(ejtag_info, EJTAG_INST_ADDRESS);
	retval = mips_ejtag_drscan_32(ejtag_info, &address);
	if (retval != ERROR_OK)
		return retval;

	if (address != MIPS32_PRACC_TEXT)
		LOG_ERROR("mini program did not return to start");

	return retval;
}
