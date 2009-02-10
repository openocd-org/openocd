/***************************************************************************
*   Copyright (C) 2005 by Dominic Rath                                    *
*   Dominic.Rath@gmx.de                                                   *
*                                                                         *
*   Copyright (C) 2007,2008 Øyvind Harboe                                 *
*   oyvind.harboe@zylin.com                                               *
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
#include "log.h"

#include "command.h"


#if 0
#define _DEBUG_JTAG_IO_
#endif

#ifndef DEBUG_JTAG_IOZ
#define DEBUG_JTAG_IOZ 64
#endif


/* 16 Tap States, from page 21 of ASSET InterTech, Inc.'s svf.pdf
 */
enum tap_state {
	TAP_RESET    = 0, TAP_IDLE = 8,
	TAP_DRSELECT = 1, TAP_DRCAPTURE = 2, TAP_DRSHIFT = 3, TAP_DREXIT1 = 4,
	TAP_DRPAUSE  = 5, TAP_DREXIT2 = 6, TAP_DRUPDATE = 7,
	TAP_IRSELECT = 9, TAP_IRCAPTURE = 10, TAP_IRSHIFT = 11, TAP_IREXIT1 = 12,
	TAP_IRPAUSE  = 13, TAP_IREXIT2 = 14, TAP_IRUPDATE = 15
};

typedef enum tap_state tap_state_t;

typedef struct tap_transition_s
{
	tap_state_t high;
	tap_state_t low;
} tap_transition_t;

//extern tap_transition_t tap_transitions[16];    /* describe the TAP state diagram */


/*-----<Cable Helper API>-------------------------------------------*/

/* The "Cable Helper API" is what the cable drivers can use to help implement
 * their "Cable API".  So a Cable Helper API is a set of helper functions used by
 * cable drivers, and this is different from a Cable API.  A "Cable API" is what
 * higher level code used to talk to a cable.
 */


/** implementation of wrapper function tap_set_state() */
void tap_set_state_impl(tap_state_t new_state);

/**
 * Function tap_set_state
 * sets the state of a "state follower" which tracks the state of the TAPs connected to the
 * cable.  The state follower is hopefully always in the same state as the actual
 * TAPs in the jtag chain, and will be so if there are no bugs in the tracking logic within that
 * cable driver. All the cable drivers call this function to indicate the state they think
 * the TAPs attached to their cables are in.  Because this function can also log transitions,
 * it will be helpful to call this function with every transition that the TAPs being manipulated
 * are expected to traverse, not just end points of a multi-step state path.
 * @param new_state is the state we think the TAPs are currently in or are about to enter.
 */
#if defined(_DEBUG_JTAG_IO_)
#define tap_set_state(new_state) \
	do { \
		LOG_DEBUG( "tap_set_state(%s)", tap_state_name(new_state) ); \
		tap_set_state_impl(new_state); \
	} while (0)
#else
static inline void tap_set_state(tap_state_t new_state)
{
	tap_set_state_impl(new_state);
}

#endif

/**
 * Function tap_get_state
 * gets the state of the "state follower" which tracks the state of the TAPs connected to
 * the cable.
 * @see tap_set_state
 * @return tap_state_t - The state the TAPs are in now.
 */
tap_state_t tap_get_state(void);

/**
 * Function tap_set_end_state
 * sets the state of an "end state follower" which tracks the state that any cable driver
 * thinks will be the end (resultant) state of the current TAP SIR or SDR operation.  At completion
 * of that TAP operation this value is copied into the state follower via tap_set_state().
 * @param new_end_state is that state the TAPs should enter at completion of a pending TAP operation.
 */
void        tap_set_end_state(tap_state_t new_end_state);

/**
 * Function tap_get_end_state
 * @see tap_set_end_state
 * @return tap_state_t - The state the TAPs should be in at completion of the current TAP operation.
 */
tap_state_t tap_get_end_state(void);

/**
 * Function tap_get_tms_path
 * returns a 7 bit long "bit sequence" indicating what has to be done with TMS
 * during a sequence of seven TAP clock cycles in order to get from
 * state \a "from" to state \a "to".
 * @param from is the starting state
 * @param to is the resultant or final state
 * @return int - a 7 bit sequence, with the first bit in the sequence at bit 0.
 */
int tap_get_tms_path(tap_state_t from, tap_state_t to);

/**
 * Function tap_move_ndx
 * when given a stable state, returns an index from 0-5.  The index corresponds to a
 * sequence of stable states which are given in this order: <p>
 * { TAP_RESET, TAP_IDLE, TAP_DRSHIFT, TAP_DRPAUSE, TAP_IRSHIFT, TAP_IRPAUSE }
 * <p>
 * This sequence corresponds to look up tables which are used in some of the
 * cable drivers.
 * @param astate is the stable state to find in the sequence.  If a non stable
 *  state is passed, this may cause the program to output an error message
 *  and terminate.
 * @return int - the array (or sequence) index as described above
 */
int tap_move_ndx(tap_state_t astate);

/**
 * Function tap_is_state_stable
 * returns true if the \a astate is stable.
 */
bool tap_is_state_stable(tap_state_t astate);

/**
 * Function tap_state_transition
 * takes a current TAP state and returns the next state according to the tms value.
 * @param current_state is the state of a TAP currently.
 * @param tms is either zero or non-zero, just like a real TMS line in a jtag interface.
 * @return tap_state_t - the next state a TAP would enter.
 */
tap_state_t tap_state_transition(tap_state_t current_state, bool tms);

/**
 * Function tap_state_name
 * Returns a string suitable for display representing the JTAG tap_state
 */
const char* tap_state_name(tap_state_t state);

/*-----</Cable Helper API>------------------------------------------*/


extern tap_state_t cmd_queue_end_state;         /* finish DR scans in dr_end_state */
extern tap_state_t cmd_queue_cur_state;         /* current TAP state */

typedef void* error_handler_t;  /* Later on we can delete error_handler_t, but keep it for now to make patches more readable */

struct scan_field_s;
typedef int (*in_handler_t)(u8* in_value, void* priv, struct scan_field_s* field);

typedef struct scan_field_s
{
	jtag_tap_t* tap;                /* tap pointer this instruction refers to */
	int         num_bits;           /* number of bits this field specifies (up to 32) */
	u8*         out_value;          /* value to be scanned into the device */
	u8*         out_mask;           /* only masked bits care */
	u8*         in_value;           /* pointer to a 32-bit memory location to take data scanned out */
	/* in_check_value/mask, in_handler_error_handler, in_handler_priv can be used by the in handler, otherwise they contain garbage  */
	u8*          in_check_value;    /* used to validate scan results */
	u8*          in_check_mask;     /* check specified bits against check_value */
	in_handler_t in_handler;        /* process received buffer using this handler */
	void*        in_handler_priv;   /* additional information for the in_handler */
} scan_field_t;

enum scan_type {
	/* IN: from device to host, OUT: from host to device */
	SCAN_IN = 1, SCAN_OUT = 2, SCAN_IO = 3
};

typedef struct scan_command_s
{
	int           ir_scan;      /* instruction/not data scan */
	int           num_fields;   /* number of fields in *fields array */
	scan_field_t* fields;       /* pointer to an array of data scan fields */
	tap_state_t   end_state;    /* TAP state in which JTAG commands should finish */
} scan_command_t;

typedef struct statemove_command_s
{
	tap_state_t end_state;   /* TAP state in which JTAG commands should finish */
} statemove_command_t;

typedef struct pathmove_command_s
{
	int          num_states;    /* number of states in *path */
	tap_state_t* path;          /* states that have to be passed */
} pathmove_command_t;

typedef struct runtest_command_s
{
	int         num_cycles;     /* number of cycles that should be spent in Run-Test/Idle */
	tap_state_t end_state;      /* TAP state in which JTAG commands should finish */
} runtest_command_t;


typedef struct stableclocks_command_s
{
	int num_cycles;             /* number of clock cycles that should be sent */
} stableclocks_command_t;


typedef struct reset_command_s
{
	int trst;           /* trst/srst 0: deassert, 1: assert, -1: don't change */
	int srst;
} reset_command_t;

typedef struct end_state_command_s
{
	tap_state_t end_state;   /* TAP state in which JTAG commands should finish */
} end_state_command_t;

typedef struct sleep_command_s
{
	u32 us;     /* number of microseconds to sleep */
} sleep_command_t;

typedef union jtag_command_container_u
{
	scan_command_t*         scan;
	statemove_command_t*    statemove;
	pathmove_command_t*     pathmove;
	runtest_command_t*      runtest;
	stableclocks_command_t* stableclocks;
	reset_command_t*        reset;
	end_state_command_t*    end_state;
	sleep_command_t* sleep;
} jtag_command_container_t;

enum jtag_command_type {
	JTAG_SCAN         = 1,
	JTAG_STATEMOVE    = 2,
	JTAG_RUNTEST      = 3,
	JTAG_RESET        = 4,
	JTAG_END_STATE    = 5,
	JTAG_PATHMOVE     = 6,
	JTAG_SLEEP        = 7,
	JTAG_STABLECLOCKS = 8
};

typedef struct jtag_command_s
{
	jtag_command_container_t cmd;
	enum jtag_command_type   type;
	struct jtag_command_s*   next;
} jtag_command_t;

extern jtag_command_t* jtag_command_queue;

/* forward declaration */
typedef struct jtag_tap_event_action_s jtag_tap_event_action_t;

/* this is really: typedef jtag_tap_t */
/* But - the typedef is done in "types.h" */
/* due to "forward decloration reasons" */
struct jtag_tap_s
{
	const char* chip;
	const char* tapname;
	const char* dotted_name;
	int         abs_chain_position;
	int         enabled;
	int         ir_length;          /* size of instruction register */
	u32         ir_capture_value;
	u8*         expected;           /* Capture-IR expected value */
	u32         ir_capture_mask;
	u8*         expected_mask;      /* Capture-IR expected mask */
	u32         idcode;             /* device identification code */
	u32*        expected_ids;       /* Array of expected identification codes */
	u8          expected_ids_cnt;   /* Number of expected identification codes */
	u8*         cur_instr;          /* current instruction */
	int         bypass;             /* bypass register selected */

	jtag_tap_event_action_t* event_action;

	jtag_tap_t* next_tap;
};
extern jtag_tap_t* jtag_AllTaps(void);
extern jtag_tap_t* jtag_TapByPosition(int n);
extern jtag_tap_t* jtag_TapByPosition(int n);
extern jtag_tap_t* jtag_TapByString(const char* dotted_name);
extern jtag_tap_t* jtag_TapByJimObj(Jim_Interp* interp, Jim_Obj* obj);
extern jtag_tap_t* jtag_TapByAbsPosition(int abs_position);
extern int         jtag_NumEnabledTaps(void);
extern int         jtag_NumTotalTaps(void);

static __inline__ jtag_tap_t* jtag_NextEnabledTap(jtag_tap_t* p)
{
	if (p == NULL)
	{
		/* start at the head of list */
		p = jtag_AllTaps();
	}
	else
	{
		/* start *after* this one */
		p = p->next_tap;
	}
	while (p)
	{
		if (p->enabled)
		{
			break;
		}
		else
		{
			p = p->next_tap;
		}
	}

	return p;
}


enum reset_line_mode {
	LINE_OPEN_DRAIN = 0x0,
	LINE_PUSH_PULL  = 0x1,
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
	int (*register_commands)(struct command_context_s* cmd_ctx);
	int (*init)(void);
	int (*quit)(void);

	/* returns JTAG maxium speed for KHz. 0=RTCK. The function returns
	 *  a failure if it can't support the KHz/RTCK.
	 *
	 *  WARNING!!!! if RTCK is *slow* then think carefully about
	 *  whether you actually want to support this in the driver.
	 *  Many target scripts are written to handle the absence of RTCK
	 *  and use a fallback kHz TCK.
	 */
	int (*khz)(int khz, int* jtag_speed);

	/* returns the KHz for the provided JTAG speed. 0=RTCK. The function returns
	 *  a failure if it can't support the KHz/RTCK. */
	int (*speed_div)(int speed, int* khz);

	/* Read and clear the power dropout flag. Note that a power dropout
	 *  can be transitionary, easily much less than a ms.
	 *
	 *  So to find out if the power is *currently* on, you must invoke
	 *  this method twice. Once to clear the power dropout flag and a
	 *  second time to read the current state.
	 *
	 *  Currently the default implementation is never to detect power dropout.
	 */
	int (*power_dropout)(int* power_dropout);

	/* Read and clear the srst asserted detection flag.
	 *
	 * NB!!!! like power_dropout this does *not* read the current
	 * state. srst assertion is transitionary and *can* be much
	 * less than 1ms.
	 */
	int (*srst_asserted)(int* srst_asserted);
} jtag_interface_t;

enum jtag_event {
	JTAG_TRST_ASSERTED
};

extern char* jtag_event_strings[];

enum jtag_tap_event {
	JTAG_TAP_EVENT_ENABLE,
	JTAG_TAP_EVENT_DISABLE
};

extern const Jim_Nvp nvp_jtag_tap_event[];

struct jtag_tap_event_action_s
{
	enum jtag_tap_event      event;
	Jim_Obj*                 body;
	jtag_tap_event_action_t* next;
};

extern int jtag_trst;
extern int jtag_srst;

typedef struct jtag_event_callback_s
{
	int (*callback)(enum jtag_event event, void* priv);
	void*                         priv;
	struct jtag_event_callback_s* next;
} jtag_event_callback_t;

extern jtag_event_callback_t* jtag_event_callbacks;

extern jtag_interface_t*      jtag; /* global pointer to configured JTAG interface */

extern int jtag_speed;
extern int jtag_speed_post_reset;

enum reset_types {
	RESET_NONE            = 0x0,
	RESET_HAS_TRST        = 0x1,
	RESET_HAS_SRST        = 0x2,
	RESET_TRST_AND_SRST   = 0x3,
	RESET_SRST_PULLS_TRST = 0x4,
	RESET_TRST_PULLS_SRST = 0x8,
	RESET_TRST_OPEN_DRAIN = 0x10,
	RESET_SRST_PUSH_PULL  = 0x20,
};

extern enum reset_types jtag_reset_config;

/* initialize interface upon startup. A successful no-op
 * upon subsequent invocations
 */
extern int  jtag_interface_init(struct command_context_s* cmd_ctx);

/* initialize JTAG chain using only a RESET reset. If init fails,
 * try reset + init.
 */
extern int  jtag_init(struct command_context_s* cmd_ctx);

/* reset, then initialize JTAG chain */
extern int  jtag_init_reset(struct command_context_s* cmd_ctx);
extern int  jtag_register_commands(struct command_context_s* cmd_ctx);

/* JTAG interface, can be implemented with a software or hardware fifo
 *
 * TAP_DRSHIFT and TAP_IRSHIFT are illegal end states. TAP_DRSHIFT/IRSHIFT as end states
 * can be emulated by using a larger scan.
 *
 * Code that is relatively insensitive to the path(as long
 * as it is JTAG compliant) taken through state machine can use
 * endstate for jtag_add_xxx_scan(). Otherwise the pause state must be
 * specified as end state and a subsequent jtag_add_pathmove() must
 * be issued.
 *
 */
extern void jtag_add_ir_scan(int num_fields, scan_field_t* fields, tap_state_t endstate);
extern int  interface_jtag_add_ir_scan(int num_fields, scan_field_t* fields, tap_state_t endstate);
extern void jtag_add_dr_scan(int num_fields, scan_field_t* fields, tap_state_t endstate);
extern int  interface_jtag_add_dr_scan(int num_fields, scan_field_t* fields, tap_state_t endstate);
extern void jtag_add_plain_ir_scan(int num_fields, scan_field_t* fields, tap_state_t endstate);
extern int  interface_jtag_add_plain_ir_scan(int num_fields, scan_field_t* fields, tap_state_t endstate);
extern void jtag_add_plain_dr_scan(int num_fields, scan_field_t* fields, tap_state_t endstate);
extern int  interface_jtag_add_plain_dr_scan(int num_fields, scan_field_t* fields, tap_state_t endstate);

/* run a TAP_RESET reset. End state is TAP_RESET, regardless
 * of start state.
 */
extern void jtag_add_tlr(void);
extern int  interface_jtag_add_tlr(void);

/* Do not use jtag_add_pathmove() unless you need to, but do use it
 * if you have to.
 *
 * DANGER! If the target is dependent upon a particular sequence
 * of transitions for things to work correctly(e.g. as a workaround
 * for an errata that contradicts the JTAG standard), then pathmove
 * must be used, even if some jtag interfaces happen to use the
 * desired path. Worse, the jtag interface used for testing a
 * particular implementation, could happen to use the "desired"
 * path when transitioning to/from end
 * state.
 *
 * A list of unambigious single clock state transitions, not
 * all drivers can support this, but it is required for e.g.
 * XScale and Xilinx support
 *
 * Note! TAP_RESET must not be used in the path!
 *
 * Note that the first on the list must be reachable
 * via a single transition from the current state.
 *
 * All drivers are required to implement jtag_add_pathmove().
 * However, if the pathmove sequence can not be precisely
 * executed, an interface_jtag_add_pathmove() or jtag_execute_queue()
 * must return an error. It is legal, but not recommended, that
 * a driver returns an error in all cases for a pathmove if it
 * can only implement a few transitions and therefore
 * a partial implementation of pathmove would have little practical
 * application.
 */
extern void jtag_add_pathmove(int num_states, tap_state_t* path);
extern int  interface_jtag_add_pathmove(int num_states, tap_state_t* path);

/* go to TAP_IDLE, if we're not already there and cycle
 * precisely num_cycles in the TAP_IDLE after which move
 * to the end state, if it is != TAP_IDLE
 *
 * nb! num_cycles can be 0, in which case the fn will navigate
 * to endstate via TAP_IDLE
 */
extern void jtag_add_runtest(int num_cycles, tap_state_t endstate);
extern int  interface_jtag_add_runtest(int num_cycles, tap_state_t endstate);

/* A reset of the TAP state machine can be requested.
 *
 * Whether tms or trst reset is used depends on the capabilities of
 * the target and jtag interface(reset_config  command configures this).
 *
 * srst can driver a reset of the TAP state machine and vice
 * versa
 *
 * Application code may need to examine value of jtag_reset_config
 * to determine the proper codepath
 *
 * DANGER! Even though srst drives trst, trst might not be connected to
 * the interface, and it might actually be *harmful* to assert trst in this case.
 *
 * This is why combinations such as "reset_config srst_only srst_pulls_trst"
 * are supported.
 *
 * only req_tlr_or_trst and srst can have a transition for a
 * call as the effects of transitioning both at the "same time"
 * are undefined, but when srst_pulls_trst or vice versa,
 * then trst & srst *must* be asserted together.
 */
extern void jtag_add_reset(int req_tlr_or_trst, int srst);

/* this drives the actual srst and trst pins. srst will always be 0
 * if jtag_reset_config & RESET_SRST_PULLS_TRST != 0 and ditto for
 * trst.
 *
 * the higher level jtag_add_reset will invoke jtag_add_tlr() if
 * approperiate
 */
extern int  interface_jtag_add_reset(int trst, int srst);
extern void jtag_add_end_state(tap_state_t endstate);
extern int  interface_jtag_add_end_state(tap_state_t endstate);
extern void jtag_add_sleep(u32 us);
extern int  interface_jtag_add_sleep(u32 us);


/**
 * Function jtag_add_stable_clocks
 * first checks that the state in which the clocks are to be issued is
 * stable, then queues up clock_count clocks for transmission.
 */
void jtag_add_clocks(int num_cycles);
int  interface_jtag_add_clocks(int num_cycles);


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
extern int            jtag_execute_queue(void);

/* can be implemented by hw+sw */
extern int            interface_jtag_execute_queue(void);
extern int            jtag_power_dropout(int* dropout);
extern int            jtag_srst_asserted(int* srst_asserted);

/* JTAG support functions */
extern void           jtag_set_check_value(scan_field_t* field, u8* value, u8* mask, error_handler_t* in_error_handler);
extern enum scan_type jtag_scan_type(scan_command_t* cmd);
extern int            jtag_scan_size(scan_command_t* cmd);
extern int            jtag_read_buffer(u8* buffer, scan_command_t* cmd);
extern int            jtag_build_buffer(scan_command_t* cmd, u8** buffer);

extern void           jtag_sleep(u32 us);
extern int            jtag_call_event_callbacks(enum jtag_event event);
extern int            jtag_register_event_callback(int (* callback)(enum jtag_event event, void* priv), void* priv);

extern int jtag_verify_capture_ir;

void jtag_tap_handle_event(jtag_tap_t* tap, enum jtag_tap_event e);

/* error codes
 * JTAG subsystem uses codes between -100 and -199 */

#define ERROR_JTAG_INIT_FAILED       (-100)
#define ERROR_JTAG_INVALID_INTERFACE (-101)
#define ERROR_JTAG_NOT_IMPLEMENTED   (-102)
#define ERROR_JTAG_TRST_ASSERTED     (-103)
#define ERROR_JTAG_QUEUE_FAILED      (-104)
#define ERROR_JTAG_NOT_STABLE_STATE  (-105)
#define ERROR_JTAG_DEVICE_ERROR      (-107)


/* this allows JTAG devices to implement the entire jtag_xxx() layer in hw/sw */
#ifdef HAVE_JTAG_MINIDRIVER_H
/* Here a #define MINIDRIVER() and an inline version of hw fifo interface_jtag_add_dr_out can be defined */
#include "jtag_minidriver.h"
#define MINIDRIVER(a) notused ## a
#else
#define MINIDRIVER(a) a

/* jtag_add_dr_out() is a faster version of jtag_add_dr_scan()
 *
 * Current or end_state can not be TAP_RESET. end_state can be -1
 *
 * num_bits[i] is the number of bits to clock out from value[i] LSB first.
 *
 * If the device is in bypass, then that is an error condition in
 * the caller code that is not detected by this fn, whereas jtag_add_dr_scan()
 * does detect it. Similarly if the device is not in bypass, data must
 * be passed to it.
 *
 * If anything fails, then jtag_error will be set and jtag_execute() will
 * return an error. There is no way to determine if there was a failure
 * during this function call.
 *
 * Note that this jtag_add_dr_out can be defined as an inline function.
 */
extern void interface_jtag_add_dr_out(jtag_tap_t* tap, int num_fields, const int* num_bits, const u32* value,
		tap_state_t end_state);

#endif

static __inline__ void jtag_add_dr_out(jtag_tap_t* tap, int num_fields, const int* num_bits, const u32* value,
		tap_state_t end_state)
{
	if (end_state != -1)
		cmd_queue_end_state = end_state;
	cmd_queue_cur_state = cmd_queue_end_state;
	interface_jtag_add_dr_out(tap, num_fields, num_bits, value, cmd_queue_end_state);
}


#endif /* JTAG_H */
