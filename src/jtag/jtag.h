/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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
#ifndef JTAG_H
#define JTAG_H

#include "types.h"
#include "binarybuffer.h"

#include "command.h"

#if 0
#define _DEBUG_JTAG_IO_
#endif

/* Tap States
 * TLR - Test-Logic-Reset, RTI - Run-Test/Idle, 
 * SDS - Select-DR-Scan, CD - Capture-DR, SD - Shift-DR, E1D - Exit1-DR,
 * PD - Pause-DR, E2D - Exit2-DR, UD - Update-DR,
 * SIS - Select-IR-Scan, CI - Capture-IR, SI - Shift-IR, E1I - Exit1-IR,
 * PI - Pause-IR, E2I - Exit2-IR, UI - Update-IR 
 */
enum tap_state
{
	TAP_TLR = 0x0, TAP_RTI = 0x8, 
	TAP_SDS = 0x1, TAP_CD = 0x2, TAP_SD = 0x3, TAP_E1D = 0x4, 
	TAP_PD = 0x5, TAP_E2D = 0x6, TAP_UD = 0x7,
	TAP_SIS = 0x9, TAP_CI = 0xa, TAP_SI = 0xb, TAP_E1I = 0xc,
	TAP_PI = 0xd, TAP_E2I = 0xe, TAP_UI = 0xf
};

typedef struct tap_transition_s
{
	enum tap_state high;
	enum tap_state low;
} tap_transition_t;

extern char* tap_state_strings[16];
extern int tap_move_map[16];	/* map 16 TAP states to 6 stable states */
extern u8 tap_move[6][6];		/* value scanned to TMS to move from one of six stable states to another */
extern tap_transition_t tap_transitions[16];	/* describe the TAP state diagram */

extern enum tap_state end_state;		/* finish DR scans in dr_end_state */
extern enum tap_state cur_state;		/* current TAP state */

extern enum tap_state cmd_queue_end_state;		/* finish DR scans in dr_end_state */
extern enum tap_state cmd_queue_cur_state;		/* current TAP state */

#define TAP_MOVE(from, to) tap_move[tap_move_map[from]][tap_move_map[to]]

typedef void * error_handler_t; /* Later on we can delete error_handler_t, but keep it for now to make patches more readable */

struct scan_field_s;
typedef int (*in_handler_t)(u8 *in_value, void *priv, struct scan_field_s *field);

typedef struct scan_field_s
{
	int device;			/* ordinal device number this instruction refers to */
	int num_bits;		/* number of bits this field specifies (up to 32) */
	u8 *out_value;		/* value to be scanned into the device */
	u8 *out_mask;		/* only masked bits care */
	u8 *in_value;		/* pointer to a 32-bit memory location to take data scanned out */
	/* in_check_value/mask, in_handler_error_handler, in_handler_priv can be used by the in handler, otherwise they contain garbage  */
	u8 *in_check_value;		/* used to validate scan results */ 
	u8 *in_check_mask;		/* check specified bits against check_value */
	in_handler_t in_handler;	    /* process received buffer using this handler */
	void *in_handler_priv;	/* additional information for the in_handler */
} scan_field_t;

enum scan_type
{
	/* IN: from device to host, OUT: from host to device */
	SCAN_IN = 1, SCAN_OUT = 2, SCAN_IO = 3
};

typedef struct scan_command_s
{
	int ir_scan;	/* instruction/not data scan */
	int num_fields;		/* number of fields in *fields array */
	scan_field_t *fields;	/* pointer to an array of data scan fields */
	enum tap_state end_state;	/* TAP state in which JTAG commands should finish */
} scan_command_t;

typedef struct statemove_command_s
{
	enum tap_state end_state;	/* TAP state in which JTAG commands should finish */
} statemove_command_t;

typedef struct pathmove_command_s
{
	int num_states;				/* number of states in *path */
	enum tap_state *path;		/* states that have to be passed */
} pathmove_command_t;

typedef struct runtest_command_s
{
	int num_cycles;		/* number of cycles that should be spent in Run-Test/Idle */
	enum tap_state end_state;	/* TAP state in which JTAG commands should finish */
} runtest_command_t;

typedef struct reset_command_s
{
	int trst;			/* trst/srst 0: deassert, 1: assert, -1: don't change */
	int srst;
} reset_command_t;

typedef struct end_state_command_s
{
	enum tap_state end_state;	/* TAP state in which JTAG commands should finish */
} end_state_command_t;

typedef struct sleep_command_s
{
	u32 us;		/* number of microseconds to sleep */
} sleep_command_t;

typedef union jtag_command_container_u
{
	scan_command_t *scan;
	statemove_command_t *statemove;
	pathmove_command_t *pathmove;
	runtest_command_t *runtest;
	reset_command_t *reset;
	end_state_command_t *end_state;
	sleep_command_t *sleep;
} jtag_command_container_t;

enum jtag_command_type
{
	JTAG_SCAN = 1,
	JTAG_STATEMOVE = 2, JTAG_RUNTEST = 3,
	JTAG_RESET = 4, JTAG_END_STATE = 5,
	JTAG_PATHMOVE = 6, JTAG_SLEEP = 7
};

typedef struct jtag_command_s
{
	jtag_command_container_t cmd;
	enum jtag_command_type type;
	struct jtag_command_s *next;
} jtag_command_t;

extern jtag_command_t *jtag_command_queue;

typedef struct jtag_device_s
{
	int ir_length;		/* size of instruction register */
	u8 *expected;		/* Capture-IR expected value */
	u8 *expected_mask;	/* Capture-IR expected mask */
	u32 idcode;			/* device identification code */
	u8 *cur_instr;		/* current instruction */
	int bypass;			/* bypass register selected */
	struct jtag_device_s *next;
} jtag_device_t;

extern jtag_device_t *jtag_devices;
extern int jtag_num_devices;
extern int jtag_ir_scan_size;

enum reset_line_mode
{
	LINE_OPEN_DRAIN = 0x0,
	LINE_PUSH_PULL = 0x1,
};

typedef struct jtag_interface_s
{
	char* name;
	
	/* queued command execution
	 */
	int (*execute_queue)(void);
	
	/* interface initalization
	 */
	int (*speed)(int speed);
	int (*register_commands)(struct command_context_s *cmd_ctx);
	int (*init)(void);
	int (*quit)(void);
	
} jtag_interface_t;

enum jtag_event
{
	JTAG_SRST_ASSERTED,
	JTAG_TRST_ASSERTED,
	JTAG_SRST_RELEASED,
	JTAG_TRST_RELEASED,
};

extern char* jtag_event_strings[];

extern int jtag_trst;
extern int jtag_srst;

typedef struct jtag_event_callback_s
{
	int (*callback)(enum jtag_event event, void *priv);
	void *priv;
	struct jtag_event_callback_s *next;
} jtag_event_callback_t;

extern jtag_event_callback_t *jtag_event_callbacks;

extern jtag_interface_t *jtag;	/* global pointer to configured JTAG interface */
extern enum tap_state end_state;
extern enum tap_state cur_state;

extern int jtag_speed;

enum reset_types
{
	RESET_NONE = 0x0, 
	RESET_HAS_TRST = 0x1, 
	RESET_HAS_SRST = 0x2, 
	RESET_TRST_AND_SRST = 0x3, 
	RESET_SRST_PULLS_TRST = 0x4,
	RESET_TRST_PULLS_SRST = 0x8,
	RESET_TRST_OPEN_DRAIN = 0x10,
	RESET_SRST_PUSH_PULL = 0x20,
};

extern enum reset_types jtag_reset_config;

/* JTAG subsystem */
extern int jtag_init(struct command_context_s *cmd_ctx);
extern int jtag_register_commands(struct command_context_s *cmd_ctx);

/* JTAG interface, can be implemented with a software or hardware fifo */
extern int jtag_add_ir_scan(int num_fields, scan_field_t *fields, enum tap_state endstate);
extern int jtag_add_dr_scan(int num_fields, scan_field_t *fields, enum tap_state endstate);
extern int jtag_add_plain_ir_scan(int num_fields, scan_field_t *fields, enum tap_state endstate);
extern int jtag_add_plain_dr_scan(int num_fields, scan_field_t *fields, enum tap_state endstate);
/* execute a state transition within the JTAG standard, but the exact path
 * path that is taken is undefined. Many implementations use precisely
 * 7 clocks to perform a transition, but it could be more or less
 * than that.
 *
 * The following assertions are made about certain common state moves:
 *
 * - A state move from Pause-[ID]R to Pause-[ID]R should always go through 
 *   Update-[ID]R and Capture-[ID]R before returning to Pause-[ID]R, otherwise 
 *   there's no way force a register update, if you can't go to Run-Test/Idle for 
 *   some reason.
 *
 *   - A state move from Pause-[ID]R to Shift-[ID]R must not go through 
 *   Update-[ID]R.
 *
 *   - Run-Test/Idle must not be entered unless requested, because R-T/I may have 
 *   side effects.
 */
extern int jtag_add_statemove(enum tap_state endstate);
/* A list of unambigious single clock state transitions, not
 * all drivers can support this, but it is required for e.g.
 * XScale and Xilinx support
 */
extern int jtag_add_pathmove(int num_states, enum tap_state *path);
/* cycle precisely num_cycles in the TAP_RTI state */
extern int jtag_add_runtest(int num_cycles, enum tap_state endstate);
extern int jtag_add_reset(int trst, int srst);
extern int jtag_add_end_state(enum tap_state endstate);
extern int jtag_add_sleep(u32 us);
/*
 * For software FIFO implementations, the queued commands can be executed 
 * during this call or earlier. A sw queue might decide to push out
 * some of the jtag_add_xxx() operations once the queue is "big enough".
 * 
 * This fn will return an error code if any of the prior jtag_add_xxx() 
 * calls caused a failure, e.g. check failure. Note that it does not
 * matter if the operation was executed *before* jtag_execute_queue(),
 * jtag_execute_queue() will still return an error code. 
 * 
 * All jtag_add_xxx() calls that have in_handler!=NULL will have been
 * executed when this fn returns, but if what has been queued only 
 * clocks data out, without reading anything back, then JTAG could 
 * be running *after* jtag_execute_queue() returns. The API does 
 * not define a way to flush a hw FIFO that runs *after* 
 * jtag_execute_queue() returns. 
 * 
 * jtag_add_xxx() commands can either be executed immediately or 
 * at some time between the jtag_add_xxx() fn call and jtag_execute_queue().  
 */
extern int jtag_execute_queue(void);

/* JTAG support functions */
extern void jtag_set_check_value(scan_field_t *field, u8 *value,  u8 *mask, error_handler_t *in_error_handler);
extern enum scan_type jtag_scan_type(scan_command_t *cmd);
extern int jtag_scan_size(scan_command_t *cmd);
extern int jtag_read_buffer(u8 *buffer, scan_command_t *cmd);
extern int jtag_build_buffer(scan_command_t *cmd, u8 **buffer);
extern jtag_device_t* jtag_get_device(int num);
extern void jtag_sleep(u32 us);
extern int jtag_call_event_callbacks(enum jtag_event event);
extern int jtag_register_event_callback(int (*callback)(enum jtag_event event, void *priv), void *priv);

extern int jtag_verify_capture_ir;


/* error codes
 * JTAG subsystem uses codes between -100 and -199 */

#define ERROR_JTAG_INIT_FAILED			(-100)
#define ERROR_JTAG_INVALID_INTERFACE	(-101)
#define ERROR_JTAG_NOT_IMPLEMENTED		(-102)
#define ERROR_JTAG_TRST_ASSERTED		(-103)
#define ERROR_JTAG_QUEUE_FAILED			(-104)
#define ERROR_JTAG_RESET_WOULD_ASSERT_TRST		(-105)
#define ERROR_JTAG_RESET_CANT_SRST				(-106)
#define ERROR_JTAG_DEVICE_ERROR			(-107)
#endif /* JTAG_H */
