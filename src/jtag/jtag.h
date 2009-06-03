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

#include "binarybuffer.h"
#include "log.h"


#ifdef _DEBUG_JTAG_IO_
#define DEBUG_JTAG_IO(expr ...)		LOG_DEBUG(expr)
#else
#define DEBUG_JTAG_IO(expr ...)
#endif

#ifndef DEBUG_JTAG_IOZ
#define DEBUG_JTAG_IOZ 64
#endif

/*-----<Macros>--------------------------------------------------*/

/** When given an array, compute its DIMension, i.e. number of elements in the array */
#define DIM(x)					(sizeof(x)/sizeof((x)[0]))

/** Calculate the number of bytes required to hold @a n TAP scan bits */
#define TAP_SCAN_BYTES(n)		CEIL(n, 8)

/*-----</Macros>-------------------------------------------------*/



/*
 * Tap states from ARM7TDMI-S Technical reference manual.
 * Also, validated against several other ARM core technical manuals.
 *
 * N.B. tap_get_tms_path() was changed to reflect this corrected
 * numbering and ordering of the TAP states.
 *
 * DANGER!!!! some interfaces care about the actual numbers used
 * as they are handed off directly to hardware implementations.
 */

typedef enum tap_state
{
#if BUILD_ECOSBOARD
/* These are the old numbers. Leave as-is for now... */
	TAP_RESET    = 0, TAP_IDLE = 8,
	TAP_DRSELECT = 1, TAP_DRCAPTURE = 2, TAP_DRSHIFT = 3, TAP_DREXIT1 = 4,
	TAP_DRPAUSE  = 5, TAP_DREXIT2 = 6, TAP_DRUPDATE = 7,
	TAP_IRSELECT = 9, TAP_IRCAPTURE = 10, TAP_IRSHIFT = 11, TAP_IREXIT1 = 12,
	TAP_IRPAUSE  = 13, TAP_IREXIT2 = 14, TAP_IRUPDATE = 15,

	TAP_NUM_STATES = 16, TAP_INVALID = -1,
#else
	/* Proper ARM recommended numbers */
	TAP_DREXIT2 = 0x0,
	TAP_DREXIT1 = 0x1,
	TAP_DRSHIFT = 0x2,
	TAP_DRPAUSE = 0x3,
	TAP_IRSELECT = 0x4,
	TAP_DRUPDATE = 0x5,
	TAP_DRCAPTURE = 0x6,
	TAP_DRSELECT = 0x7,
	TAP_IREXIT2 = 0x8,
	TAP_IREXIT1 = 0x9,
	TAP_IRSHIFT = 0xa,
	TAP_IRPAUSE = 0xb,
	TAP_IDLE = 0xc,
	TAP_IRUPDATE = 0xd,
	TAP_IRCAPTURE = 0xe,
	TAP_RESET = 0x0f,

	TAP_NUM_STATES = 0x10,

	TAP_INVALID = -1,
#endif
} tap_state_t;

/**
 * Function tap_state_name
 * Returns a string suitable for display representing the JTAG tap_state
 */
const char* tap_state_name(tap_state_t state);

typedef struct tap_transition_s
{
	tap_state_t high;
	tap_state_t low;
} tap_transition_t;

//extern tap_transition_t tap_transitions[16];    /* describe the TAP state diagram */



extern tap_state_t cmd_queue_end_state;         /* finish DR scans in dr_end_state */
extern tap_state_t cmd_queue_cur_state;         /* current TAP state */

typedef struct scan_field_s
{
	jtag_tap_t* tap;                /* tap pointer this instruction refers to */
	int         num_bits;           /* number of bits this field specifies (up to 32) */
	u8*         out_value;          /* value to be scanned into the device */
	u8*         in_value;           /* pointer to a 32-bit memory location to take data scanned out */

	u8*         check_value;        /* Used together with jtag_add_dr_scan_check() to check data clocked
	                                   in */
	u8*         check_mask;         /* mask to go with check_value */

	/* internal work space */
	int			allocated;			/* in_value has been allocated for the queue */
	int			modified;			/* did we modify the in_value? */
	u8			intmp[4];			/* temporary storage for checking synchronously */
} scan_field_t;

#ifdef INCLUDE_JTAG_INTERFACE_H

enum scan_type {
	/* IN: from device to host, OUT: from host to device */
	SCAN_IN = 1, SCAN_OUT = 2, SCAN_IO = 3
};

typedef struct scan_command_s
{
	bool          ir_scan;      /* instruction/not data scan */
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

extern void* cmd_queue_alloc(size_t size);
extern void cmd_queue_free(void);

extern void jtag_queue_command(jtag_command_t *cmd);
extern void jtag_command_queue_reset(void);

#endif // INCLUDE_JTAG_INTERFACE_H

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

/// Shutdown the JTAG interface upon program exit.
extern int  jtag_interface_quit(void);

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
/* same as jtag_add_ir_scan except no verify is performed */
extern void jtag_add_ir_scan_noverify(int num_fields, const scan_field_t *fields, tap_state_t state);
extern void jtag_add_dr_scan(int num_fields, const scan_field_t* fields, tap_state_t endstate);

/* set in_value to point to 32 bits of memory to scan into. This function
 * is a way to handle the case of synchronous and asynchronous
 * JTAG queues.
 *
 * In the event of an asynchronous queue execution the queue buffer
 * allocation method is used, for the synchronous case the temporary 32 bits come
 * from the input field itself.
 */
extern void jtag_alloc_in_value32(scan_field_t *field);

/* This version of jtag_add_dr_scan() uses the check_value/mask fields */
extern void jtag_add_dr_scan_check(int num_fields, scan_field_t* fields, tap_state_t endstate);
extern void jtag_add_plain_ir_scan(int num_fields, const scan_field_t* fields, tap_state_t endstate);
extern void jtag_add_plain_dr_scan(int num_fields, const scan_field_t* fields, tap_state_t endstate);


/* Simplest/typical callback - do some conversion on the data clocked in.
 * This callback is for such conversion that can not fail.
 * For conversion types or checks that can
 * fail, use the jtag_callback_t variant */
typedef void (*jtag_callback1_t)(u8 *in);

/* A simpler version of jtag_add_callback4 */
extern void jtag_add_callback(jtag_callback1_t, u8 *in);


/* This type can store an integer safely by a normal cast on 64 and
 * 32 bit systems. */
typedef intptr_t jtag_callback_data_t;

/* The generic callback mechanism.
 *
 * The callback is invoked with three arguments. The first argument is
 * the pointer to the data clocked in.
 */
typedef int (*jtag_callback_t)(u8 *in, jtag_callback_data_t data1, jtag_callback_data_t data2, jtag_callback_data_t data3);


/* This callback can be executed immediately the queue has been flushed. Note that
 * the JTAG queue can either be executed synchronously or asynchronously. Typically
 * for USB the queue is executed asynchronously. For low latency interfaces, the
 * queue may be executed synchronously.
 *
 * These callbacks are typically executed *after* the *entire* JTAG queue has been
 * executed for e.g. USB interfaces.
 *
 * The callbacks are guaranteeed to be invoked in the order that they were queued.
 *
 * The strange name is due to C's lack of overloading using function arguments
 *
 * The callback mechansim is very general and does not really make any assumptions
 * about what the callback does and what the arguments are.
 *
 * in - typically used to point to the data to operate on. More often than not
 * this will be the data clocked in during a shift operation
 *
 * data1 - an integer that is big enough to be used either as an 'int' or
 * cast to/from a pointer
 *
 * data2 - an integer that is big enough to be used either as an 'int' or
 * cast to/from a pointer
 *
 * Why stop at 'data2' for arguments? Somewhat historical reasons. This is
 * sufficient to implement the jtag_check_value_mask(), besides the
 * line is best drawn somewhere...
 *
 * If the execution of the queue fails before the callbacks, then the
 * callbacks may or may not be invoked depending on driver implementation.
 */
extern void jtag_add_callback4(jtag_callback_t, u8 *in,
		jtag_callback_data_t data1, jtag_callback_data_t data2,
		jtag_callback_data_t data3);


/* run a TAP_RESET reset. End state is TAP_RESET, regardless
 * of start state.
 */
extern void jtag_add_tlr(void);

/* Application code *must* assume that interfaces will
 * implement transitions between states with different
 * paths and path lengths through the state diagram. The
 * path will vary across interface and also across versions
 * of the same interface over time. Even if the OpenOCD code
 * is unchanged, the actual path taken may vary over time
 * and versions of interface firmware or PCB revisions.
 *
 * Use jtag_add_pathmove() when specific transition sequences
 * are required.
 *
 * Do not use jtag_add_pathmove() unless you need to, but do use it
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
extern void jtag_add_pathmove(int num_states, const tap_state_t* path);

/* go to TAP_IDLE, if we're not already there and cycle
 * precisely num_cycles in the TAP_IDLE after which move
 * to the end state, if it is != TAP_IDLE
 *
 * nb! num_cycles can be 0, in which case the fn will navigate
 * to endstate via TAP_IDLE
 */
extern void jtag_add_runtest(int num_cycles, tap_state_t endstate);

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

extern void jtag_add_end_state(tap_state_t endstate);
extern void jtag_add_sleep(u32 us);


/**
 * Function jtag_add_stable_clocks
 * first checks that the state in which the clocks are to be issued is
 * stable, then queues up clock_count clocks for transmission.
 */
void jtag_add_clocks(int num_cycles);


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

/* same as jtag_execute_queue() but does not clear the error flag */
extern void jtag_execute_queue_noclear(void);

/* this flag is set when an error occurs while executing the queue. cleared
 * by jtag_execute_queue()
 *
 * this flag can also be set from application code if some error happens
 * during processing that should be reported during jtag_execute_queue().
 */
extern int jtag_error;

static __inline__ void jtag_set_error(int error)
{
	if ((error==ERROR_OK)||(jtag_error!=ERROR_OK))
	{
		/* keep first error */
		return;
	}
	jtag_error=error;
}



/* can be implemented by hw+sw */
extern int            jtag_power_dropout(int* dropout);
extern int            jtag_srst_asserted(int* srst_asserted);

/* JTAG support functions */

/* execute jtag queue and check value and use mask if mask is != NULL. invokes
 * jtag_set_error() with any error. */
extern void jtag_check_value_mask(scan_field_t *field, u8 *value, u8 *mask);

#ifdef INCLUDE_JTAG_INTERFACE_H
extern enum scan_type jtag_scan_type(const scan_command_t* cmd);
extern int            jtag_scan_size(const scan_command_t* cmd);
extern int            jtag_read_buffer(u8* buffer, const scan_command_t* cmd);
extern int            jtag_build_buffer(const scan_command_t* cmd, u8** buffer);
#endif // INCLUDE_JTAG_INTERFACE_H

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

/* jtag_add_dr_out() is a version of jtag_add_dr_scan() which
 * only scans data out. It operates on 32 bit integers instead
 * of 8 bit, which makes it a better impedance match with
 * the calling code which often operate on 32 bit integers.
 *
 * Current or end_state can not be TAP_RESET. end_state can be TAP_INVALID
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
 * This is an inline fn to speed up embedded hosts. Also note that
 * interface_jtag_add_dr_out() can be a *small* inline function for
 * embedded hosts.
 *
 * There is no jtag_add_dr_outin() version of this fn that also allows
 * clocking data back in. Patches gladly accepted!
 */
extern void jtag_add_dr_out(jtag_tap_t* tap,
		int num_fields, const int* num_bits, const u32* value,
		tap_state_t end_state);


/**
 * Function jtag_add_statemove
 * moves from the current state to the goal \a state. This needs
 * to be handled according to the xsvf spec, see the XSTATE command
 * description.
 */
extern int jtag_add_statemove(tap_state_t goal_state);

#endif /* JTAG_H */
