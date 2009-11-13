/***************************************************************************
 *   Copyright (C) 2008 digenius technology GmbH.                          *
 *   Michael Bruck                                                         *
 *                                                                         *
 *   Copyright (C) 2008,2009 Oyvind Harboe oyvind.harboe@zylin.com         *
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

#include "arm11_dbgtap.h"

#include "time_support.h"

#if 0
#define JTAG_DEBUG(expr ...)	DEBUG(expr)
#else
#define JTAG_DEBUG(expr ...)	do {} while (0)
#endif

/*
This pathmove goes from Pause-IR to Shift-IR while avoiding RTI. The
behavior of the FTDI driver IIRC was to go via RTI.

Conversely there may be other places in this code where the ARM11 code relies
on the driver to hit through RTI when coming from Update-?R.
*/
static const tap_state_t arm11_move_pi_to_si_via_ci[] =
{
    TAP_IREXIT2, TAP_IRUPDATE, TAP_DRSELECT, TAP_IRSELECT, TAP_IRCAPTURE, TAP_IRSHIFT
};


int arm11_add_ir_scan_vc(int num_fields, struct scan_field *fields, tap_state_t state)
{
	if (cmd_queue_cur_state == TAP_IRPAUSE)
		jtag_add_pathmove(asizeof(arm11_move_pi_to_si_via_ci), arm11_move_pi_to_si_via_ci);

	jtag_add_ir_scan(num_fields, fields, state);
	return ERROR_OK;
}

static const tap_state_t arm11_move_pd_to_sd_via_cd[] =
{
	TAP_DREXIT2, TAP_DRUPDATE, TAP_DRSELECT, TAP_DRCAPTURE, TAP_DRSHIFT
};

int arm11_add_dr_scan_vc(int num_fields, struct scan_field *fields, tap_state_t state)
{
	if (cmd_queue_cur_state == TAP_DRPAUSE)
		jtag_add_pathmove(asizeof(arm11_move_pd_to_sd_via_cd), arm11_move_pd_to_sd_via_cd);

	jtag_add_dr_scan(num_fields, fields, state);
	return ERROR_OK;
}


/** Code de-clutter: Construct struct scan_field to write out a value
 *
 * \param arm11			Target state variable.
 * \param num_bits		Length of the data field
 * \param out_data		pointer to the data that will be sent out
 *						<em > (data is read when it is added to the JTAG queue)</em>
 * \param in_data		pointer to the memory that will receive data that was clocked in
 *						<em > (data is written when the JTAG queue is executed)</em>
 * \param field			target data structure that will be initialized
 */
void arm11_setup_field(arm11_common_t * arm11, int num_bits, void * out_data, void * in_data, struct scan_field * field)
{
	field->tap   			= arm11->target->tap;
	field->num_bits			= num_bits;
	field->out_value		= out_data;
	field->in_value			= in_data;
}


/** Write JTAG instruction register
 *
 * \param arm11		Target state variable.
 * \param instr		An ARM11 DBGTAP instruction. Use enum #arm11_instructions.
 * \param state		Pass the final TAP state or ARM11_TAP_DEFAULT for the default value (Pause-IR).
 *
 * \remarks			This adds to the JTAG command queue but does \em not execute it.
 */
void arm11_add_IR(arm11_common_t * arm11, uint8_t instr, tap_state_t state)
{
	struct jtag_tap *tap;
	tap = arm11->target->tap;

	if (buf_get_u32(tap->cur_instr, 0, 5) == instr)
	{
		JTAG_DEBUG("IR <= 0x%02x SKIPPED", instr);
		return;
	}

	JTAG_DEBUG("IR <= 0x%02x", instr);

	struct scan_field field;

	arm11_setup_field(arm11, 5, &instr, NULL, &field);

	arm11_add_ir_scan_vc(1, &field, state == ARM11_TAP_DEFAULT ? TAP_IRPAUSE : state);
}

/** Verify shifted out data from Scan Chain Register (SCREG)
 *  Used as parameter to struct scan_field::in_handler in
 *  arm11_add_debug_SCAN_N().
 *
 */
static void arm11_in_handler_SCAN_N(uint8_t *in_value)
{
	/** \todo TODO: clarify why this isnt properly masked in core.c jtag_read_buffer() */
	uint8_t v = *in_value & 0x1F;

	if (v != 0x10)
	{
		LOG_ERROR("'arm11 target' JTAG communication error SCREG SCAN OUT 0x%02x (expected 0x10)", v);
		jtag_set_error(ERROR_FAIL);
	}

	JTAG_DEBUG("SCREG SCAN OUT 0x%02x", v);
}

/** Select and write to Scan Chain Register (SCREG)
 *
 * This function sets the instruction register to SCAN_N and writes
 * the data register with the selected chain number.
 *
 * http://infocenter.arm.com/help/topic/com.arm.doc.ddi0301f/Cacbjhfg.html
 *
 * \param arm11	    Target state variable.
 * \param chain	    Scan chain that will be selected.
 * \param state	    Pass the final TAP state or ARM11_TAP_DEFAULT for the default
 *					value (Pause-DR).
 *
 * The chain takes effect when Update-DR is passed (usually when subsequently
 * the INTEXT/EXTEST instructions are written).
 *
 * \warning			(Obsolete) Using this twice in a row will \em fail. The first
 *					call will end in Pause-DR. The second call, due to the IR
 *					caching, will not go through Capture-DR when shifting in the
 *					new scan chain number. As a result the verification in
 *					arm11_in_handler_SCAN_N() must fail.
 *
 * \remarks			This adds to the JTAG command queue but does \em not execute it.
 */

int arm11_add_debug_SCAN_N(arm11_common_t * arm11, uint8_t chain, tap_state_t state)
{
	JTAG_DEBUG("SCREG <= 0x%02x", chain);

	arm11_add_IR(arm11, ARM11_SCAN_N, ARM11_TAP_DEFAULT);

	struct scan_field		field;

	uint8_t tmp[1];
	arm11_setup_field(arm11, 5, &chain, &tmp, &field);

	arm11_add_dr_scan_vc(1, &field, state == ARM11_TAP_DEFAULT ? TAP_DRPAUSE : state);

	jtag_execute_queue_noclear();

	arm11_in_handler_SCAN_N(tmp);

	return jtag_execute_queue();
}

/** Write an instruction into the ITR register
 *
 * \param arm11		Target state variable.
 * \param inst		An ARM11 processor instruction/opcode.
 * \param flag		Optional parameter to retrieve the InstCompl flag
 *					(this will be written when the JTAG chain is executed).
 * \param state		Pass the final TAP state or ARM11_TAP_DEFAULT for the default
 *					value (Run-Test/Idle).
 *
 * \remarks			By default this ends with Run-Test/Idle state
 *					and causes the instruction to be executed. If
 *					a subsequent write to DTR is needed before
 *					executing the instruction then TAP_DRPAUSE should be
 *					passed to \p state.
 *
 * \remarks			This adds to the JTAG command queue but does \em not execute it.
 */
void arm11_add_debug_INST(arm11_common_t * arm11, uint32_t inst, uint8_t * flag, tap_state_t state)
{
	JTAG_DEBUG("INST <= 0x%08x", inst);

	struct scan_field		itr[2];

	arm11_setup_field(arm11, 32,    &inst,	NULL, itr + 0);
	arm11_setup_field(arm11, 1,	    NULL,	flag, itr + 1);

	arm11_add_dr_scan_vc(asizeof(itr), itr, state == ARM11_TAP_DEFAULT ? TAP_IDLE : state);
}

/** Read the Debug Status and Control Register (DSCR)
 *
 * same as CP14 c1
 *
 * \param arm11		Target state variable.
 * \param value		DSCR content
 * \return			Error status
 *
 * \remarks			This is a stand-alone function that executes the JTAG command queue.
 */
int arm11_read_DSCR(arm11_common_t * arm11, uint32_t *value)
{
	int retval;
	retval = arm11_add_debug_SCAN_N(arm11, 0x01, ARM11_TAP_DEFAULT);
	if (retval != ERROR_OK)
		return retval;

	arm11_add_IR(arm11, ARM11_INTEST, ARM11_TAP_DEFAULT);

	uint32_t				dscr;
	struct scan_field	chain1_field;

	arm11_setup_field(arm11, 32, NULL, &dscr, &chain1_field);

	arm11_add_dr_scan_vc(1, &chain1_field, TAP_DRPAUSE);

	CHECK_RETVAL(jtag_execute_queue());

	if (arm11->last_dscr != dscr)
		JTAG_DEBUG("DSCR  = %08x (OLD %08x)", dscr, arm11->last_dscr);

	arm11->last_dscr = dscr;

	*value = dscr;

	return ERROR_OK;
}

/** Write the Debug Status and Control Register (DSCR)
 *
 * same as CP14 c1
 *
 * \param arm11		Target state variable.
 * \param dscr		DSCR content
 *
 * \remarks			This is a stand-alone function that executes the JTAG command queue.
 */
int arm11_write_DSCR(arm11_common_t * arm11, uint32_t dscr)
{
	int retval;
	retval = arm11_add_debug_SCAN_N(arm11, 0x01, ARM11_TAP_DEFAULT);
	if (retval != ERROR_OK)
		return retval;

	arm11_add_IR(arm11, ARM11_EXTEST, ARM11_TAP_DEFAULT);

	struct scan_field		    chain1_field;

	arm11_setup_field(arm11, 32, &dscr, NULL, &chain1_field);

	arm11_add_dr_scan_vc(1, &chain1_field, TAP_DRPAUSE);

	CHECK_RETVAL(jtag_execute_queue());

	JTAG_DEBUG("DSCR <= %08x (OLD %08x)", dscr, arm11->last_dscr);

	arm11->last_dscr = dscr;

	return ERROR_OK;
}



/** Get the debug reason from Debug Status and Control Register (DSCR)
 *
 * \param dscr		DSCR value to analyze
 * \return			Debug reason
 *
 */
enum target_debug_reason arm11_get_DSCR_debug_reason(uint32_t dscr)
{
	switch (dscr & ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_MASK)
	{
	case ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_HALT:
		LOG_INFO("Debug entry: JTAG HALT");
		return DBG_REASON_DBGRQ;

	case ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_BREAKPOINT:
		LOG_INFO("Debug entry: breakpoint");
		return DBG_REASON_BREAKPOINT;

	case ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_WATCHPOINT:
		LOG_INFO("Debug entry: watchpoint");
		return DBG_REASON_WATCHPOINT;

	case ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_BKPT_INSTRUCTION:
		LOG_INFO("Debug entry: BKPT instruction");
		return DBG_REASON_BREAKPOINT;

	case ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_EDBGRQ:
		LOG_INFO("Debug entry: EDBGRQ signal");
		return DBG_REASON_DBGRQ;

	case ARM11_DSCR_METHOD_OF_DEBUG_ENTRY_VECTOR_CATCH:
		LOG_INFO("Debug entry: VCR vector catch");
		return DBG_REASON_BREAKPOINT;

	default:
		LOG_INFO("Debug entry: unknown");
		return DBG_REASON_DBGRQ;
	}
};



/** Prepare the stage for ITR/DTR operations
 * from the arm11_run_instr... group of functions.
 *
 * Put arm11_run_instr_data_prepare() and arm11_run_instr_data_finish()
 * around a block of arm11_run_instr_... calls.
 *
 * Select scan chain 5 to allow quick access to DTR. When scan
 * chain 4 is needed to put in a register the ITRSel instruction
 * shortcut is used instead of actually changing the Scan_N
 * register.
 *
 * \param arm11		Target state variable.
 *
 */
int arm11_run_instr_data_prepare(arm11_common_t * arm11)
{
	return arm11_add_debug_SCAN_N(arm11, 0x05, ARM11_TAP_DEFAULT);
}

/** Cleanup after ITR/DTR operations
 * from the arm11_run_instr... group of functions
 *
 * Put arm11_run_instr_data_prepare() and arm11_run_instr_data_finish()
 * around a block of arm11_run_instr_... calls.
 *
 * Any IDLE can lead to an instruction execution when
 * scan chains 4 or 5 are selected and the IR holds
 * INTEST or EXTEST. So we must disable that before
 * any following activities lead to an IDLE.
 *
 * \param arm11		Target state variable.
 *
 */
int arm11_run_instr_data_finish(arm11_common_t * arm11)
{
	return arm11_add_debug_SCAN_N(arm11, 0x00, ARM11_TAP_DEFAULT);
}



/** Execute one or multiple instructions via ITR
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	Pointer to sequence of ARM opcodes
 * \param count		Number of opcodes to execute
 *
 */
int arm11_run_instr_no_data(arm11_common_t * arm11, uint32_t * opcode, size_t count)
{
	arm11_add_IR(arm11, ARM11_ITRSEL, ARM11_TAP_DEFAULT);

	while (count--)
	{
		arm11_add_debug_INST(arm11, *opcode++, NULL, TAP_IDLE);

		int i = 0;
		while (1)
		{
			uint8_t flag;

			arm11_add_debug_INST(arm11, 0, &flag, count ? TAP_IDLE : TAP_DRPAUSE);

			CHECK_RETVAL(jtag_execute_queue());

			if (flag)
				break;

			long long then = 0;

			if (i == 1000)
			{
				then = timeval_ms();
			}
			if (i >= 1000)
			{
				if ((timeval_ms()-then) > 1000)
				{
					LOG_WARNING("Timeout (1000ms) waiting for instructions to complete");
					return ERROR_FAIL;
				}
			}

			i++;
		}
	}

	return ERROR_OK;
}

/** Execute one instruction via ITR
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	ARM opcode
 *
 */
int arm11_run_instr_no_data1(arm11_common_t * arm11, uint32_t opcode)
{
	return arm11_run_instr_no_data(arm11, &opcode, 1);
}


/** Execute one instruction via ITR repeatedly while
 *  passing data to the core via DTR on each execution.
 *
 *  The executed instruction \em must read data from DTR.
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	ARM opcode
 * \param data		Pointer to the data words to be passed to the core
 * \param count		Number of data words and instruction repetitions
 *
 */
int arm11_run_instr_data_to_core(arm11_common_t * arm11, uint32_t opcode, uint32_t * data, size_t count)
{
	arm11_add_IR(arm11, ARM11_ITRSEL, ARM11_TAP_DEFAULT);

	arm11_add_debug_INST(arm11, opcode, NULL, TAP_DRPAUSE);

	arm11_add_IR(arm11, ARM11_EXTEST, ARM11_TAP_DEFAULT);

	struct scan_field	chain5_fields[3];

	uint32_t				Data;
	uint8_t				Ready;
	uint8_t				nRetry;

	arm11_setup_field(arm11, 32,    &Data,  NULL,		chain5_fields + 0);
	arm11_setup_field(arm11,  1,    NULL,   &Ready,		chain5_fields + 1);
	arm11_setup_field(arm11,  1,    NULL,   &nRetry,	chain5_fields + 2);

	while (count--)
	{
		int i = 0;
		do
		{
			Data	    = *data;

			arm11_add_dr_scan_vc(asizeof(chain5_fields), chain5_fields, jtag_set_end_state(TAP_IDLE));

			CHECK_RETVAL(jtag_execute_queue());

			JTAG_DEBUG("DTR  Ready %d  nRetry %d", Ready, nRetry);

			long long then = 0;

			if (i == 1000)
			{
				then = timeval_ms();
			}
			if (i >= 1000)
			{
				if ((timeval_ms()-then) > 1000)
				{
					LOG_WARNING("Timeout (1000ms) waiting for instructions to complete");
					return ERROR_FAIL;
				}
			}

			i++;
		}
		while (!Ready);

		data++;
	}

	arm11_add_IR(arm11, ARM11_INTEST, ARM11_TAP_DEFAULT);

	int i = 0;
	do
	{
		Data	    = 0;

		arm11_add_dr_scan_vc(asizeof(chain5_fields), chain5_fields, TAP_DRPAUSE);

		CHECK_RETVAL(jtag_execute_queue());

		JTAG_DEBUG("DTR  Data %08x  Ready %d  nRetry %d", Data, Ready, nRetry);

		long long then = 0;

		if (i == 1000)
		{
			then = timeval_ms();
		}
		if (i >= 1000)
		{
			if ((timeval_ms()-then) > 1000)
			{
				LOG_WARNING("Timeout (1000ms) waiting for instructions to complete");
				return ERROR_FAIL;
			}
		}

		i++;
	}
	while (!Ready);

	return ERROR_OK;
}

/** JTAG path for arm11_run_instr_data_to_core_noack
 *
 *  The repeated TAP_IDLE's do not cause a repeated execution
 *  if passed without leaving the state.
 *
 *  Since this is more than 7 bits (adjustable via adding more
 *  TAP_IDLE's) it produces an artificial delay in the lower
 *  layer (FT2232) that is long enough to finish execution on
 *  the core but still shorter than any manually inducible delays.
 *
 *  To disable this code, try "memwrite burst false"
 *
 *  FIX!!! should we use multiple TAP_IDLE here or not???
 *
 *  https://lists.berlios.de/pipermail/openocd-development/2009-July/009698.html
 *  https://lists.berlios.de/pipermail/openocd-development/2009-August/009865.html
 */
static const tap_state_t arm11_MOVE_DRPAUSE_IDLE_DRPAUSE_with_delay[] =
{
	TAP_DREXIT2, TAP_DRUPDATE, TAP_IDLE, TAP_IDLE, TAP_IDLE, TAP_DRSELECT, TAP_DRCAPTURE, TAP_DRSHIFT
};



/** Execute one instruction via ITR repeatedly while
 *  passing data to the core via DTR on each execution.
 *
 *  No Ready check during transmission.
 *
 *  The executed instruction \em must read data from DTR.
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	ARM opcode
 * \param data		Pointer to the data words to be passed to the core
 * \param count		Number of data words and instruction repetitions
 *
 */
int arm11_run_instr_data_to_core_noack(arm11_common_t * arm11, uint32_t opcode, uint32_t * data, size_t count)
{
	arm11_add_IR(arm11, ARM11_ITRSEL, ARM11_TAP_DEFAULT);

	arm11_add_debug_INST(arm11, opcode, NULL, TAP_DRPAUSE);

	arm11_add_IR(arm11, ARM11_EXTEST, ARM11_TAP_DEFAULT);

	struct scan_field	chain5_fields[3];

	arm11_setup_field(arm11, 32,    NULL/*&Data*/,  NULL,				chain5_fields + 0);
	arm11_setup_field(arm11,  1,    NULL,			NULL /*&Ready*/,	chain5_fields + 1);
	arm11_setup_field(arm11,  1,    NULL,			NULL,				chain5_fields + 2);

	uint8_t			*Readies;
	size_t readiesNum = (count + 1);
	size_t bytes = sizeof(*Readies)*readiesNum;
	Readies = (uint8_t *) malloc(bytes);
	if (Readies == NULL)
	{
		LOG_ERROR("Out of memory allocating " ZU " bytes", bytes);
		return ERROR_FAIL;
	}

	uint8_t	*		ReadyPos			= Readies;

	while (count--)
	{
		chain5_fields[0].out_value	= (void *)(data++);
		chain5_fields[1].in_value	= ReadyPos++;

		if (count)
		{
			jtag_add_dr_scan(asizeof(chain5_fields), chain5_fields, jtag_set_end_state(TAP_DRPAUSE));
			jtag_add_pathmove(asizeof(arm11_MOVE_DRPAUSE_IDLE_DRPAUSE_with_delay),
				arm11_MOVE_DRPAUSE_IDLE_DRPAUSE_with_delay);
		}
		else
		{
			jtag_add_dr_scan(asizeof(chain5_fields), chain5_fields, jtag_set_end_state(TAP_IDLE));
		}
	}

	arm11_add_IR(arm11, ARM11_INTEST, ARM11_TAP_DEFAULT);

	chain5_fields[0].out_value	= 0;
	chain5_fields[1].in_value   = ReadyPos++;

	arm11_add_dr_scan_vc(asizeof(chain5_fields), chain5_fields, TAP_DRPAUSE);

	int retval = jtag_execute_queue();
	if (retval == ERROR_OK)
	{
		size_t error_count = 0;

		for (size_t i = 0; i < readiesNum; i++)
		{
			if (Readies[i] != 1)
			{
				error_count++;
			}
		}

		if (error_count > 0 )
			LOG_ERROR(ZU " words out of " ZU " not transferred", error_count, readiesNum);

	}

	free(Readies);

	return retval;
}


/** Execute an instruction via ITR while handing data into the core via DTR.
 *
 *  The executed instruction \em must read data from DTR.
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	ARM opcode
 * \param data		Data word to be passed to the core via DTR
 *
 */
int arm11_run_instr_data_to_core1(arm11_common_t * arm11, uint32_t opcode, uint32_t data)
{
	return arm11_run_instr_data_to_core(arm11, opcode, &data, 1);
}


/** Execute one instruction via ITR repeatedly while
 *  reading data from the core via DTR on each execution.
 *
 *  The executed instruction \em must write data to DTR.
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	ARM opcode
 * \param data		Pointer to an array that receives the data words from the core
 * \param count		Number of data words and instruction repetitions
 *
 */
int arm11_run_instr_data_from_core(arm11_common_t * arm11, uint32_t opcode, uint32_t * data, size_t count)
{
	arm11_add_IR(arm11, ARM11_ITRSEL, ARM11_TAP_DEFAULT);

	arm11_add_debug_INST(arm11, opcode, NULL, TAP_IDLE);

	arm11_add_IR(arm11, ARM11_INTEST, ARM11_TAP_DEFAULT);

	struct scan_field	chain5_fields[3];

	uint32_t			Data;
	uint8_t			Ready;
	uint8_t			nRetry;

	arm11_setup_field(arm11, 32,    NULL,	&Data,	    chain5_fields + 0);
	arm11_setup_field(arm11,  1,    NULL,	&Ready,	    chain5_fields + 1);
	arm11_setup_field(arm11,  1,    NULL,	&nRetry,    chain5_fields + 2);

	while (count--)
	{
		int i = 0;
		do
		{
			arm11_add_dr_scan_vc(asizeof(chain5_fields), chain5_fields, count ? TAP_IDLE : TAP_DRPAUSE);

			CHECK_RETVAL(jtag_execute_queue());

			JTAG_DEBUG("DTR  Data %08x  Ready %d  nRetry %d", Data, Ready, nRetry);

			long long then = 0;

			if (i == 1000)
			{
				then = timeval_ms();
			}
			if (i >= 1000)
			{
				if ((timeval_ms()-then) > 1000)
				{
					LOG_WARNING("Timeout (1000ms) waiting for instructions to complete");
					return ERROR_FAIL;
				}
			}

			i++;
		}
		while (!Ready);

		*data++ = Data;
	}

	return ERROR_OK;
}

/** Execute one instruction via ITR
 *  then load r0 into DTR and read DTR from core.
 *
 *  The first executed instruction (\p opcode) should write data to r0.
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	ARM opcode to write r0 with the value of interest
 * \param data		Pointer to a data word that receives the value from r0 after \p opcode was executed.
 *
 */
int arm11_run_instr_data_from_core_via_r0(arm11_common_t * arm11, uint32_t opcode, uint32_t * data)
{
	int retval;
	retval = arm11_run_instr_no_data1(arm11, opcode);
	if (retval != ERROR_OK)
		return retval;

	/* MCR p14,0,R0,c0,c5,0 (move r0 -> wDTR -> local var) */
	arm11_run_instr_data_from_core(arm11, 0xEE000E15, data, 1);

	return ERROR_OK;
}

/** Load data into core via DTR then move it to r0 then
 *  execute one instruction via ITR
 *
 *  The final executed instruction (\p opcode) should read data from r0.
 *
 * \pre arm11_run_instr_data_prepare() /  arm11_run_instr_data_finish() block
 *
 * \param arm11		Target state variable.
 * \param opcode	ARM opcode to read r0 act upon it
 * \param data		Data word that will be written to r0 before \p opcode is executed
 *
 */
int arm11_run_instr_data_to_core_via_r0(arm11_common_t * arm11, uint32_t opcode, uint32_t data)
{
	int retval;
	/* MRC p14,0,r0,c0,c5,0 */
	retval = arm11_run_instr_data_to_core1(arm11, 0xEE100E15, data);
	if (retval != ERROR_OK)
		return retval;

	retval = arm11_run_instr_no_data1(arm11, opcode);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

/** Apply reads and writes to scan chain 7
 *
 * \see arm11_sc7_action_t
 *
 * \param arm11		Target state variable.
 * \param actions	A list of read and/or write instructions
 * \param count		Number of instructions in the list.
 *
 */
int arm11_sc7_run(arm11_common_t * arm11, arm11_sc7_action_t * actions, size_t count)
{
	int retval;

	retval = arm11_add_debug_SCAN_N(arm11, 0x07, ARM11_TAP_DEFAULT);
	if (retval != ERROR_OK)
		return retval;

	arm11_add_IR(arm11, ARM11_EXTEST, ARM11_TAP_DEFAULT);

	struct scan_field	chain7_fields[3];

	uint8_t				nRW;
	uint32_t				DataOut;
	uint8_t				AddressOut;
	uint8_t				Ready;
	uint32_t				DataIn;
	uint8_t				AddressIn;

	arm11_setup_field(arm11,  1, &nRW,			&Ready,		chain7_fields + 0);
	arm11_setup_field(arm11, 32, &DataOut,		&DataIn,	chain7_fields + 1);
	arm11_setup_field(arm11,  7, &AddressOut,	&AddressIn,	chain7_fields + 2);

	for (size_t i = 0; i < count + 1; i++)
	{
		if (i < count)
		{
			nRW			= actions[i].write ? 1 : 0;
			DataOut		= actions[i].value;
			AddressOut	= actions[i].address;
		}
		else
		{
			nRW			= 0;
			DataOut		= 0;
			AddressOut	= 0;
		}

		do
		{
			JTAG_DEBUG("SC7 <= Address %02x  Data %08x    nRW %d", AddressOut, DataOut, nRW);

			arm11_add_dr_scan_vc(asizeof(chain7_fields), chain7_fields, TAP_DRPAUSE);

			CHECK_RETVAL(jtag_execute_queue());

			JTAG_DEBUG("SC7 => Address %02x  Data %08x  Ready %d", AddressIn, DataIn, Ready);
		}
		while (!Ready); /* 'nRW' is 'Ready' on read out */

		if (i > 0)
		{
			if (actions[i - 1].address != AddressIn)
			{
				LOG_WARNING("Scan chain 7 shifted out unexpected address");
			}

			if (!actions[i - 1].write)
			{
				actions[i - 1].value = DataIn;
			}
			else
			{
				if (actions[i - 1].value != DataIn)
				{
					LOG_WARNING("Scan chain 7 shifted out unexpected data");
				}
			}
		}
	}

	for (size_t i = 0; i < count; i++)
	{
		JTAG_DEBUG("SC7 %02d: %02x %s %08x", i, actions[i].address, actions[i].write ? "<=" : "=>", actions[i].value);
	}

	return ERROR_OK;
}

/** Clear VCR and all breakpoints and watchpoints via scan chain 7
 *
 * \param arm11		Target state variable.
 *
 */
void arm11_sc7_clear_vbw(arm11_common_t * arm11)
{
	arm11_sc7_action_t		clear_bw[arm11->brp + arm11->wrp + 1];
	arm11_sc7_action_t *	pos = clear_bw;

	for (size_t i = 0; i < asizeof(clear_bw); i++)
	{
		clear_bw[i].write	= true;
		clear_bw[i].value	= 0;
	}

	for (size_t i = 0; i < arm11->brp; i++)
		(pos++)->address = ARM11_SC7_BCR0 + i;


	for (size_t i = 0; i < arm11->wrp; i++)
		(pos++)->address = ARM11_SC7_WCR0 + i;


	(pos++)->address = ARM11_SC7_VCR;

	arm11_sc7_run(arm11, clear_bw, asizeof(clear_bw));
}

/** Write VCR register
 *
 * \param arm11		Target state variable.
 * \param value		Value to be written
 */
void arm11_sc7_set_vcr(arm11_common_t * arm11, uint32_t value)
{
	arm11_sc7_action_t		set_vcr;

	set_vcr.write		= true;
	set_vcr.address		= ARM11_SC7_VCR;
	set_vcr.value		= value;


	arm11_sc7_run(arm11, &set_vcr, 1);
}



/** Read word from address
 *
 * \param arm11		Target state variable.
 * \param address	Memory address to be read
 * \param result	Pointer where to store result
 *
 */
int arm11_read_memory_word(arm11_common_t * arm11, uint32_t address, uint32_t * result)
{
	int retval;
	retval = arm11_run_instr_data_prepare(arm11);
	if (retval != ERROR_OK)
		return retval;

	/* MRC p14,0,r0,c0,c5,0 (r0 = address) */
	CHECK_RETVAL(arm11_run_instr_data_to_core1(arm11, 0xee100e15, address));

	/* LDC p14,c5,[R0],#4 (DTR = [r0]) */
	CHECK_RETVAL(arm11_run_instr_data_from_core(arm11, 0xecb05e01, result, 1));

	return arm11_run_instr_data_finish(arm11);
}


/** Write Embedded Trace Macrocell (ETM) via Scan chain 6
 *
 * http://infocenter.arm.com/help/topic/com.arm.doc.ddi0318e/Bcfddjeh.html#Bcfggcbe
 *
 * \param arm11		Target state variable.
 * \param address	7 bit ETM register address
 * \param value		Value to be written
 *					
 * \return			Error status
 *
 * \remarks			This is a stand-alone function that executes the JTAG command queue.
 */
int arm11_write_etm(arm11_common_t * arm11, uint8_t address, uint32_t value)
{
	CHECK_RETVAL(arm11_add_debug_SCAN_N(arm11, 0x06, ARM11_TAP_DEFAULT));

	/* Uses INTEST for read and write */
	arm11_add_IR(arm11, ARM11_INTEST, ARM11_TAP_DEFAULT);

	struct scan_field		chain6_fields[3];

	uint8_t				nRW			= 1;

	arm11_setup_field(arm11, 32, &value,		NULL,		chain6_fields + 0);
	arm11_setup_field(arm11,  7, &address,		NULL,		chain6_fields + 1);
	arm11_setup_field(arm11,  1, &nRW,			NULL,		chain6_fields + 2);

	arm11_add_dr_scan_vc(asizeof(chain6_fields), chain6_fields, TAP_IDLE);

	CHECK_RETVAL(jtag_execute_queue());

	return ERROR_OK;
}

/** Read Embedded Trace Macrocell (ETM) via Scan chain 6
 *
 * http://infocenter.arm.com/help/topic/com.arm.doc.ddi0318e/Bcfddjeh.html#Bcfggcbe
 *
 * \param arm11		Target state variable.
 * \param address	7 bit ETM register address
 * \param value		Pointer that receives value that was read
 *					
 * \return			Error status
 *
 * \remarks			This is a stand-alone function that executes the JTAG command queue.
 */
int arm11_read_etm(arm11_common_t * arm11, uint8_t address, uint32_t * value)
{
	CHECK_RETVAL(arm11_add_debug_SCAN_N(arm11, 0x06, ARM11_TAP_DEFAULT));

	/* Uses INTEST for read and write */
	arm11_add_IR(arm11, ARM11_INTEST, ARM11_TAP_DEFAULT);

	struct scan_field		chain6_fields[3];

	uint8_t				nRW			= 0;

	arm11_setup_field(arm11, 32, NULL,			NULL,		chain6_fields + 0);
	arm11_setup_field(arm11,  7, &address,		NULL,		chain6_fields + 1);
	arm11_setup_field(arm11,  1, &nRW,			NULL,		chain6_fields + 2);

	arm11_add_dr_scan_vc(asizeof(chain6_fields), chain6_fields, TAP_IDLE);

	/* Data is made available in Capture-DR and shifted out on the next access */

	arm11_setup_field(arm11, 32, NULL,			value,		chain6_fields + 0);
	arm11_setup_field(arm11,  7, &address,		NULL,		chain6_fields + 1);
	arm11_setup_field(arm11,  1, &nRW,			NULL,		chain6_fields + 2);

	arm11_add_dr_scan_vc(asizeof(chain6_fields), chain6_fields, TAP_IDLE);

	CHECK_RETVAL(jtag_execute_queue());

	return ERROR_OK;
}

