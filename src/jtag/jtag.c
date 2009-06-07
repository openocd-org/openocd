/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 SoftPLC Corporation                                *
 *       http://softplc.com                                                *
 *   dick@softplc.com                                                      *
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

#include "jtag.h"
#include "minidriver.h"
#include "interface.h"

#ifdef HAVE_STRINGS_H
#include <strings.h>
#endif


/// The number of JTAG queue flushes (for profiling and debugging purposes).
static int jtag_flush_queue_count;

static void jtag_add_scan_check(void (*jtag_add_scan)(int in_num_fields, const scan_field_t *in_fields, tap_state_t state),
		int in_num_fields, scan_field_t *in_fields, tap_state_t state);

/* note that this is not marked as static as it must be available from outside jtag.c for those
   that implement the jtag_xxx() minidriver layer
*/
int jtag_error=ERROR_OK;

char* jtag_event_strings[] =
{
	"JTAG controller reset (RESET or TRST)"
};

const Jim_Nvp nvp_jtag_tap_event[] = {
	{ .value = JTAG_TAP_EVENT_ENABLE,       .name = "tap-enable" },
	{ .value = JTAG_TAP_EVENT_DISABLE,      .name = "tap-disable" },

	{ .name = NULL, .value = -1 }
};

int jtag_trst = 0;
int jtag_srst = 0;

/**
 * List all TAPs that have been created.
 */
static jtag_tap_t *__jtag_all_taps = NULL;
/**
 * The number of TAPs in the __jtag_all_taps list, used to track the
 * assigned chain position to new TAPs
 */
static int jtag_num_taps = 0;

enum reset_types jtag_reset_config = RESET_NONE;
tap_state_t cmd_queue_end_state = TAP_RESET;
tap_state_t cmd_queue_cur_state = TAP_RESET;

int jtag_verify_capture_ir = 1;
int jtag_verify = 1;

/* how long the OpenOCD should wait before attempting JTAG communication after reset lines deasserted (in ms) */
static int jtag_nsrst_delay = 0; /* default to no nSRST delay */
static int jtag_ntrst_delay = 0; /* default to no nTRST delay */

/* callbacks to inform high-level handlers about JTAG state changes */
jtag_event_callback_t *jtag_event_callbacks;

/* speed in kHz*/
static int speed_khz = 0;
/* flag if the kHz speed was defined */
static bool hasKHz = false;

/* jtag interfaces (parport, FTDI-USB, TI-USB, ...)
 */

#if BUILD_ECOSBOARD == 1
	extern jtag_interface_t zy1000_interface;
#endif

#if BUILD_MINIDUMMY == 1
	extern jtag_interface_t minidummy_interface;
#endif
#if BUILD_PARPORT == 1
	extern jtag_interface_t parport_interface;
#endif

#if BUILD_DUMMY == 1
	extern jtag_interface_t dummy_interface;
#endif

#if BUILD_FT2232_FTD2XX == 1
	extern jtag_interface_t ft2232_interface;
#endif

#if BUILD_FT2232_LIBFTDI == 1
	extern jtag_interface_t ft2232_interface;
#endif

#if BUILD_AMTJTAGACCEL == 1
	extern jtag_interface_t amt_jtagaccel_interface;
#endif

#if BUILD_EP93XX == 1
	extern jtag_interface_t ep93xx_interface;
#endif

#if BUILD_AT91RM9200 == 1
	extern jtag_interface_t at91rm9200_interface;
#endif

#if BUILD_GW16012 == 1
	extern jtag_interface_t gw16012_interface;
#endif

#if BUILD_PRESTO_LIBFTDI == 1 || BUILD_PRESTO_FTD2XX == 1
	extern jtag_interface_t presto_interface;
#endif

#if BUILD_USBPROG == 1
	extern jtag_interface_t usbprog_interface;
#endif

#if BUILD_JLINK == 1
	extern jtag_interface_t jlink_interface;
#endif

#if BUILD_VSLLINK == 1
	extern jtag_interface_t vsllink_interface;
#endif

#if BUILD_RLINK == 1
	extern jtag_interface_t rlink_interface;
#endif

#if BUILD_ARMJTAGEW == 1
	extern jtag_interface_t armjtagew_interface;
#endif

jtag_interface_t *jtag_interfaces[] = {
#if BUILD_ECOSBOARD == 1
	&zy1000_interface,
#endif
#if BUILD_MINIDUMMY == 1
	&minidummy_interface,
#endif
#if BUILD_PARPORT == 1
	&parport_interface,
#endif
#if BUILD_DUMMY == 1
	&dummy_interface,
#endif
#if BUILD_FT2232_FTD2XX == 1
	&ft2232_interface,
#endif
#if BUILD_FT2232_LIBFTDI == 1
	&ft2232_interface,
#endif
#if BUILD_AMTJTAGACCEL == 1
	&amt_jtagaccel_interface,
#endif
#if BUILD_EP93XX == 1
	&ep93xx_interface,
#endif
#if BUILD_AT91RM9200 == 1
	&at91rm9200_interface,
#endif
#if BUILD_GW16012 == 1
	&gw16012_interface,
#endif
#if BUILD_PRESTO_LIBFTDI == 1 || BUILD_PRESTO_FTD2XX == 1
	&presto_interface,
#endif
#if BUILD_USBPROG == 1
	&usbprog_interface,
#endif
#if BUILD_JLINK == 1
	&jlink_interface,
#endif
#if BUILD_VSLLINK == 1
	&vsllink_interface,
#endif
#if BUILD_RLINK == 1
	&rlink_interface,
#endif
#if BUILD_ARMJTAGEW == 1
	&armjtagew_interface,
#endif
	NULL,
};

struct jtag_interface_s *jtag = NULL;

/* configuration */
static jtag_interface_t *jtag_interface = NULL;
int jtag_speed = 0;

/* jtag commands */
static int handle_interface_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_jtag_speed_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_jtag_khz_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_jtag_device_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_reset_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_jtag_nsrst_delay_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_jtag_ntrst_delay_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

static int handle_scan_chain_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

static int handle_endstate_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_jtag_reset_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_runtest_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_irscan_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int Jim_Command_drscan(Jim_Interp *interp, int argc, Jim_Obj *const *argv);
static int Jim_Command_flush_count(Jim_Interp *interp, int argc, Jim_Obj *const *args);

static int handle_verify_ircapture_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_verify_jtag_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_tms_sequence_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

jtag_tap_t *jtag_all_taps(void)
{
	return __jtag_all_taps;
};

int jtag_tap_count(void)
{
	return jtag_num_taps;
	}

unsigned jtag_tap_count_enabled(void)
{
	jtag_tap_t *t;
	unsigned n;

	n = 0;
	t = jtag_all_taps();
	while(t){
		if( t->enabled ){
			n++;
		}
		t = t->next_tap;
	}
	return n;
}

/// Append a new TAP to the chain of all taps.
static void jtag_tap_add(struct jtag_tap_s *t)
{
	t->abs_chain_position = jtag_num_taps++;

	jtag_tap_t **tap = &__jtag_all_taps;
	while(*tap != NULL)
		tap = &(*tap)->next_tap;
	*tap = t;
}

jtag_tap_t *jtag_tap_by_string( const char *s )
{
	jtag_tap_t *t;
	char *cp;

	t = jtag_all_taps();
	/* try name first */
	while(t){
		if( 0 == strcmp( t->dotted_name, s ) ){
			break;
		} else {
			t = t->next_tap;
		}
	}
	/* backup plan is by number */
	if( t == NULL ){
		/* ok - is "s" a number? */
		int n;
		n = strtol( s, &cp, 0 );
		if( (s != cp) && (*cp == 0) ){
			/* Then it is... */
			t = jtag_tap_by_abs_position(n);
		}
	}
	return t;
}

jtag_tap_t * jtag_tap_by_jim_obj( Jim_Interp *interp, Jim_Obj *o )
{
	jtag_tap_t *t;
	const char *cp;

	cp = Jim_GetString( o, NULL );
	if(cp == NULL){
		cp = "(unknown)";
		t = NULL;
	}  else {
		t = jtag_tap_by_string( cp );
	}
	if( t == NULL ){
		Jim_SetResult_sprintf(interp,"Tap: %s is unknown", cp );
	}
	return t;
}

/* returns a pointer to the n-th device in the scan chain */
jtag_tap_t * jtag_tap_by_abs_position( int n )
{
	int orig_n;
	jtag_tap_t *t;

	orig_n = n;
	t = jtag_all_taps();

	while( t && (n > 0)) {
		n--;
		t = t->next_tap;
	}
	return t;
}

const char *jtag_tap_name(const jtag_tap_t *tap)
{
	return (tap == NULL) ? "(unknown)" : tap->dotted_name;
}


int jtag_register_event_callback(int (*callback)(enum jtag_event event, void *priv), void *priv)
{
	jtag_event_callback_t **callbacks_p = &jtag_event_callbacks;

	if (callback == NULL)
	{
		return ERROR_INVALID_ARGUMENTS;
	}

	if (*callbacks_p)
	{
		while ((*callbacks_p)->next)
			callbacks_p = &((*callbacks_p)->next);
		callbacks_p = &((*callbacks_p)->next);
	}

	(*callbacks_p) = malloc(sizeof(jtag_event_callback_t));
	(*callbacks_p)->callback = callback;
	(*callbacks_p)->priv = priv;
	(*callbacks_p)->next = NULL;

	return ERROR_OK;
}

int jtag_unregister_event_callback(int (*callback)(enum jtag_event event, void *priv))
{
	jtag_event_callback_t **callbacks_p = &jtag_event_callbacks;

	if (callback == NULL)
	{
		return ERROR_INVALID_ARGUMENTS;
	}

	while (*callbacks_p)
	{
		jtag_event_callback_t **next = &((*callbacks_p)->next);
		if ((*callbacks_p)->callback == callback)
		{
			free(*callbacks_p);
			*callbacks_p = *next;
		}
		callbacks_p = next;
	}

	return ERROR_OK;
}

int jtag_call_event_callbacks(enum jtag_event event)
{
	jtag_event_callback_t *callback = jtag_event_callbacks;

	LOG_DEBUG("jtag event: %s", jtag_event_strings[event]);

	while (callback)
	{
		callback->callback(event, callback->priv);
		callback = callback->next;
	}

	return ERROR_OK;
}

static void jtag_checks(void)
{
	assert(jtag_trst == 0);
}

static void jtag_prelude(tap_state_t state)
{
	jtag_checks();

	assert(state!=TAP_INVALID);

	cmd_queue_cur_state = state;
}

void jtag_alloc_in_value32(scan_field_t *field)
{
	interface_jtag_alloc_in_value32(field);
}

void jtag_add_ir_scan_noverify(int in_num_fields, const scan_field_t *in_fields, tap_state_t state)
{
	int retval;
	jtag_prelude(state);

	retval=interface_jtag_add_ir_scan(in_num_fields, in_fields, state);
	if (retval!=ERROR_OK)
		jtag_error=retval;

}


/**
 * Generate an IR SCAN with a list of scan fields with one entry for each enabled TAP.
 *
 * If the input field list contains an instruction value for a TAP then that is used
 * otherwise the TAP is set to bypass.
 *
 * TAPs for which no fields are passed are marked as bypassed for subsequent DR SCANs.
 *
 */
void jtag_add_ir_scan(int in_num_fields, scan_field_t *in_fields, tap_state_t state)
{
	if (jtag_verify&&jtag_verify_capture_ir)
	{
		/* 8 x 32 bit id's is enough for all invocations */

		for (int j = 0; j < in_num_fields; j++)
		{
			/* if we are to run a verification of the ir scan, we need to get the input back.
			 * We may have to allocate space if the caller didn't ask for the input back.
			 */
			in_fields[j].check_value=in_fields[j].tap->expected;
			in_fields[j].check_mask=in_fields[j].tap->expected_mask;
		}
		jtag_add_scan_check(jtag_add_ir_scan_noverify, in_num_fields, in_fields, state);
	} else
	{
		jtag_add_ir_scan_noverify(in_num_fields, in_fields, state);
	}
}

/**
 * Duplicate the scan fields passed into the function into an IR SCAN command
 *
 * This function assumes that the caller handles extra fields for bypassed TAPs
 *
 */
void jtag_add_plain_ir_scan(int in_num_fields, const scan_field_t *in_fields, tap_state_t state)
{
	int retval;

	jtag_prelude(state);

	retval=interface_jtag_add_plain_ir_scan(in_num_fields, in_fields, state);
	if (retval!=ERROR_OK)
		jtag_error=retval;
}

void jtag_add_callback(jtag_callback1_t f, u8 *in)
{
	interface_jtag_add_callback(f, in);
}

void jtag_add_callback4(jtag_callback_t f, u8 *in,
		jtag_callback_data_t data1, jtag_callback_data_t data2,
		jtag_callback_data_t data3)
{
	interface_jtag_add_callback4(f, in, data1, data2, data3);
}

int jtag_check_value_inner(u8 *captured, u8 *in_check_value, u8 *in_check_mask, int num_bits);

static int jtag_check_value_mask_callback(u8 *in, jtag_callback_data_t data1, jtag_callback_data_t data2, jtag_callback_data_t data3)
{
	return jtag_check_value_inner(in, (u8 *)data1, (u8 *)data2, (int)data3);
}

static void jtag_add_scan_check(void (*jtag_add_scan)(int in_num_fields, const scan_field_t *in_fields, tap_state_t state),
		int in_num_fields, scan_field_t *in_fields, tap_state_t state)
{
	for (int i = 0; i < in_num_fields; i++)
	{
		struct scan_field_s *field = &in_fields[i];
		field->allocated = 0;
		field->modified = 0;
		if (field->check_value || field->in_value)
			continue;
		interface_jtag_add_scan_check_alloc(field);
		field->modified = 1;
	}

	jtag_add_scan(in_num_fields, in_fields, state);

	for (int i = 0; i < in_num_fields; i++)
	{
		if ((in_fields[i].check_value != NULL) && (in_fields[i].in_value != NULL))
		{
			/* this is synchronous for a minidriver */
			jtag_add_callback4(jtag_check_value_mask_callback, in_fields[i].in_value,
				(jtag_callback_data_t)in_fields[i].check_value,
				(jtag_callback_data_t)in_fields[i].check_mask,
				(jtag_callback_data_t)in_fields[i].num_bits);
		}
		if (in_fields[i].allocated)
		{
			free(in_fields[i].in_value);
		}
		if (in_fields[i].modified)
		{
			in_fields[i].in_value = NULL;
		}
	}
}

void jtag_add_dr_scan_check(int in_num_fields, scan_field_t *in_fields, tap_state_t state)
{
	if (jtag_verify)
	{
		jtag_add_scan_check(jtag_add_dr_scan, in_num_fields, in_fields, state);
	} else
	{
		jtag_add_dr_scan(in_num_fields, in_fields, state);
	}
}


/**
 * Generate a DR SCAN using the fields passed to the function
 *
 * For not bypassed TAPs the function checks in_fields and uses fields specified there.
 * For bypassed TAPs the function generates a dummy 1bit field.
 *
 * The bypass status of TAPs is set by jtag_add_ir_scan().
 *
 */
void jtag_add_dr_scan(int in_num_fields, const scan_field_t *in_fields, tap_state_t state)
{
	int retval;

	jtag_prelude(state);

	retval=interface_jtag_add_dr_scan(in_num_fields, in_fields, state);
	if (retval!=ERROR_OK)
		jtag_error=retval;
}



/**
 * Duplicate the scan fields passed into the function into a DR SCAN command
 *
 * This function assumes that the caller handles extra fields for bypassed TAPs
 *
 */
void jtag_add_plain_dr_scan(int in_num_fields, const scan_field_t *in_fields, tap_state_t state)
{
	int retval;

	jtag_prelude(state);

	retval=interface_jtag_add_plain_dr_scan(in_num_fields, in_fields, state);
	if (retval!=ERROR_OK)
		jtag_error=retval;
}

void jtag_add_dr_out(jtag_tap_t* tap,
		int num_fields, const int* num_bits, const u32* value,
		tap_state_t end_state)
{
	assert(end_state != TAP_INVALID);

	cmd_queue_cur_state = end_state;

	interface_jtag_add_dr_out(tap,
			num_fields, num_bits, value,
			end_state);
}

void jtag_add_tlr(void)
{
	jtag_prelude(TAP_RESET);

	int retval;
	retval=interface_jtag_add_tlr();
	if (retval!=ERROR_OK)
		jtag_error=retval;
	
	jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
}

void jtag_add_pathmove(int num_states, const tap_state_t *path)
{
	tap_state_t cur_state = cmd_queue_cur_state;
	int i;
	int retval;

	/* the last state has to be a stable state */
	if (!tap_is_state_stable(path[num_states - 1]))
	{
		LOG_ERROR("BUG: TAP path doesn't finish in a stable state");
		exit(-1);
	}

	for (i=0; i<num_states; i++)
	{
		if (path[i] == TAP_RESET)
		{
			LOG_ERROR("BUG: TAP_RESET is not a valid state for pathmove sequences");
			exit(-1);
		}

		if ( tap_state_transition(cur_state, true)  != path[i]
		  && tap_state_transition(cur_state, false) != path[i])
		{
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition", tap_state_name(cur_state), tap_state_name(path[i]));
			exit(-1);
		}
		cur_state = path[i];
	}

	jtag_checks();

	retval = interface_jtag_add_pathmove(num_states, path);
	cmd_queue_cur_state = path[num_states - 1];
	if (retval!=ERROR_OK)
		jtag_error=retval;
}

void jtag_add_runtest(int num_cycles, tap_state_t state)
{
	int retval;

	jtag_prelude(state);

	/* executed by sw or hw fifo */
	retval=interface_jtag_add_runtest(num_cycles, state);
	if (retval!=ERROR_OK)
		jtag_error=retval;
}


void jtag_add_clocks( int num_cycles )
{
	int retval;

	if( !tap_is_state_stable(cmd_queue_cur_state) )
	{
		 LOG_ERROR( "jtag_add_clocks() was called with TAP in non-stable state \"%s\"",
				 tap_state_name(cmd_queue_cur_state) );
		 jtag_error = ERROR_JTAG_NOT_STABLE_STATE;
		 return;
	}

	if( num_cycles > 0 )
	{
		jtag_checks();

		retval = interface_jtag_add_clocks(num_cycles);
		if (retval != ERROR_OK)
			jtag_error=retval;
	}
}

void jtag_add_reset(int req_tlr_or_trst, int req_srst)
{
	int trst_with_tlr = 0;
	int retval;

	/* FIX!!! there are *many* different cases here. A better
	 * approach is needed for legal combinations of transitions...
	 */
	if ((jtag_reset_config & RESET_HAS_SRST)&&
			(jtag_reset_config & RESET_HAS_TRST)&&
			((jtag_reset_config & RESET_SRST_PULLS_TRST)==0))
	{
		if (((req_tlr_or_trst&&!jtag_trst)||
				(!req_tlr_or_trst&&jtag_trst))&&
				((req_srst&&!jtag_srst)||
						(!req_srst&&jtag_srst)))
		{
			/* FIX!!! srst_pulls_trst allows 1,1 => 0,0 transition.... */
			//LOG_ERROR("BUG: transition of req_tlr_or_trst and req_srst in the same jtag_add_reset() call is undefined");
		}
	}

	/* Make sure that jtag_reset_config allows the requested reset */
	/* if SRST pulls TRST, we can't fulfill srst == 1 with trst == 0 */
	if (((jtag_reset_config & RESET_SRST_PULLS_TRST) && (req_srst == 1)) && (!req_tlr_or_trst))
	{
		LOG_ERROR("BUG: requested reset would assert trst");
		jtag_error=ERROR_FAIL;
		return;
	}

	/* if TRST pulls SRST, we reset with TAP T-L-R */
	if (((jtag_reset_config & RESET_TRST_PULLS_SRST) && (req_tlr_or_trst)) && (req_srst == 0))
	{
		trst_with_tlr = 1;
	}

	if (req_srst && !(jtag_reset_config & RESET_HAS_SRST))
	{
		LOG_ERROR("BUG: requested SRST assertion, but the current configuration doesn't support this");
		jtag_error=ERROR_FAIL;
		return;
	}

	if (req_tlr_or_trst)
	{
		if (!trst_with_tlr && (jtag_reset_config & RESET_HAS_TRST))
		{
			jtag_trst = 1;
		} else
		{
			trst_with_tlr = 1;
		}
	} else
	{
		jtag_trst = 0;
	}

	jtag_srst = req_srst;

	retval = interface_jtag_add_reset(jtag_trst, jtag_srst);
	if (retval!=ERROR_OK)
	{
		jtag_error=retval;
		return;
	}
	jtag_execute_queue();

	if (jtag_srst)
	{
		LOG_DEBUG("SRST line asserted");
	}
	else
	{
		LOG_DEBUG("SRST line released");
		if (jtag_nsrst_delay)
			jtag_add_sleep(jtag_nsrst_delay * 1000);
	}

	if (trst_with_tlr)
	{
		LOG_DEBUG("JTAG reset with RESET instead of TRST");
		jtag_set_end_state(TAP_RESET);
		jtag_add_tlr();
		return;
	}

	if (jtag_trst)
	{
		/* we just asserted nTRST, so we're now in Test-Logic-Reset,
		 * and inform possible listeners about this
		 */
		LOG_DEBUG("TRST line asserted");
		tap_set_state(TAP_RESET);
		jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
	}
	else
	{
		if (jtag_ntrst_delay)
			jtag_add_sleep(jtag_ntrst_delay * 1000);
	}
}

tap_state_t jtag_set_end_state(tap_state_t state)
{
	if ((state == TAP_DRSHIFT)||(state == TAP_IRSHIFT))
	{
		LOG_ERROR("BUG: TAP_DRSHIFT/IRSHIFT can't be end state. Calling code should use a larger scan field");
	}

	if (state!=TAP_INVALID)
		cmd_queue_end_state = state;
	return cmd_queue_end_state;
}

tap_state_t jtag_get_end_state(void)
{
	return cmd_queue_end_state;
}

void jtag_add_sleep(u32 us)
{
	keep_alive(); /* we might be running on a very slow JTAG clk */
	int retval=interface_jtag_add_sleep(us);
	if (retval!=ERROR_OK)
		jtag_error=retval;
	return;
}

int jtag_check_value_inner(u8 *captured, u8 *in_check_value, u8 *in_check_mask, int num_bits)
{
	int retval = ERROR_OK;

	int compare_failed = 0;

	if (in_check_mask)
		compare_failed = buf_cmp_mask(captured, in_check_value, in_check_mask, num_bits);
	else
		compare_failed = buf_cmp(captured, in_check_value, num_bits);

	if (compare_failed){
		/* An error handler could have caught the failing check
		 * only report a problem when there wasn't a handler, or if the handler
		 * acknowledged the error
		 */
		/*
		LOG_WARNING("TAP %s:",
					jtag_tap_name(field->tap));
					*/
		if (compare_failed)
		{
			char *captured_char = buf_to_str(captured, (num_bits > DEBUG_JTAG_IOZ) ? DEBUG_JTAG_IOZ : num_bits, 16);
			char *in_check_value_char = buf_to_str(in_check_value, (num_bits > DEBUG_JTAG_IOZ) ? DEBUG_JTAG_IOZ : num_bits, 16);

			if (in_check_mask)
			{
				char *in_check_mask_char;
				in_check_mask_char = buf_to_str(in_check_mask, (num_bits > DEBUG_JTAG_IOZ) ? DEBUG_JTAG_IOZ : num_bits, 16);
				LOG_WARNING("value captured during scan didn't pass the requested check:");
				LOG_WARNING("captured: 0x%s check_value: 0x%s check_mask: 0x%s",
							captured_char, in_check_value_char, in_check_mask_char);
				free(in_check_mask_char);
			}
			else
			{
				LOG_WARNING("value captured during scan didn't pass the requested check: captured: 0x%s check_value: 0x%s", captured_char, in_check_value_char);
			}

			free(captured_char);
			free(in_check_value_char);

			retval = ERROR_JTAG_QUEUE_FAILED;
		}

	}
	return retval;
}

void jtag_check_value_mask(scan_field_t *field, u8 *value, u8 *mask)
{
	assert(field->in_value != NULL);

	if (value==NULL)
	{
		/* no checking to do */
		return;
	}

	jtag_execute_queue_noclear();

	int retval=jtag_check_value_inner(field->in_value, value, mask, field->num_bits);
	jtag_set_error(retval);
}



int default_interface_jtag_execute_queue(void)
{
	if (NULL == jtag)
	{
		LOG_ERROR("No JTAG interface configured yet.  "
			"Issue 'init' command in startup scripts "
			"before communicating with targets.");
		return ERROR_FAIL;
	}

	return jtag->execute_queue();
}

void jtag_execute_queue_noclear(void)
{
	/* each flush can take as much as 1-2ms on high bandwidth low latency interfaces.
	 * E.g. a JTAG over TCP/IP or USB....
	 */
	jtag_flush_queue_count++;

	int retval=interface_jtag_execute_queue();
	/* we keep the first error */
	if ((jtag_error==ERROR_OK)&&(retval!=ERROR_OK))
	{
		jtag_error=retval;
	}
}

int jtag_get_flush_queue_count(void)
{
	return jtag_flush_queue_count;
}

int jtag_execute_queue(void)
{
	int retval;
	jtag_execute_queue_noclear();
	retval=jtag_error;
	jtag_error=ERROR_OK;
	return retval;
}

static int jtag_reset_callback(enum jtag_event event, void *priv)
{
	jtag_tap_t *tap = priv;

	LOG_DEBUG("-");

	if (event == JTAG_TRST_ASSERTED)
	{
		buf_set_ones(tap->cur_instr, tap->ir_length);
		tap->bypass = 1;
	}

	return ERROR_OK;
}

void jtag_sleep(u32 us)
{
	alive_sleep(us/1000);
}

/// maximum number of JTAG devices expected in the chain
#define JTAG_MAX_CHAIN_SIZE 20

#define EXTRACT_MFG(X)  (((X) & 0xffe) >> 1)
#define EXTRACT_PART(X) (((X) & 0xffff000) >> 12)
#define EXTRACT_VER(X)  (((X) & 0xf0000000) >> 28)

static int jtag_examine_chain_execute(u8 *idcode_buffer, unsigned num_idcode)
{
	scan_field_t field = {
			.tap = NULL,
			.num_bits = num_idcode * 32,
			.out_value = idcode_buffer,
			.in_value = idcode_buffer,
		};

	// initialize to the end of chain ID value
	for (unsigned i = 0; i < JTAG_MAX_CHAIN_SIZE; i++)
		buf_set_u32(idcode_buffer, i * 32, 32, 0x000000FF);

	jtag_add_plain_dr_scan(1, &field, TAP_RESET);
	return jtag_execute_queue();
}

static bool jtag_examine_chain_check(u8 *idcodes, unsigned count)
{
	u8 zero_check = 0x0;
	u8 one_check = 0xff;

	for (unsigned i = 0; i < count * 4; i++)
	{
		zero_check |= idcodes[i];
		one_check &= idcodes[i];
	}

	/* if there wasn't a single non-zero bit or if all bits were one,
	 * the scan is not valid */
	if (zero_check == 0x00 || one_check == 0xff)
	{
		LOG_ERROR("JTAG communication failure: check connection, "
			"JTAG interface, target power etc.");
		return false;
	}
	return true;
}

static void jtag_examine_chain_display(enum log_levels level, const char *msg,
		const char *name, u32 idcode)
{
	log_printf_lf(level, __FILE__, __LINE__, __FUNCTION__,
			"JTAG tap: %s %16.16s: 0x%08x "
			"(mfg: 0x%3.3x, part: 0x%4.4x, ver: 0x%1.1x)",
		name, msg, idcode,
		EXTRACT_MFG(idcode), EXTRACT_PART(idcode), EXTRACT_VER(idcode) );
}

static bool jtag_idcode_is_final(u32 idcode)
{
		return idcode == 0x000000FF || idcode == 0xFFFFFFFF;
}

/**
 * This helper checks that remaining bits in the examined chain data are
 * all as expected, but a single JTAG device requires only 64 bits to be
 * read back correctly.  This can help identify and diagnose problems
 * with the JTAG chain earlier, gives more helpful/explicit error messages.
 */
static void jtag_examine_chain_end(u8 *idcodes, unsigned count, unsigned max)
{
	bool triggered = false;
	for ( ; count < max - 31; count += 32)
	{
		u32 idcode = buf_get_u32(idcodes, count, 32);
		// do not trigger the warning if the data looks good
		if (!triggered && jtag_idcode_is_final(idcode))
			continue;
		LOG_WARNING("Unexpected idcode after end of chain: %d 0x%08x",
				count, idcode);
		triggered = true;
	}
}

static bool jtag_examine_chain_match_tap(const struct jtag_tap_s *tap)
{
	if (0 == tap->expected_ids_cnt)
	{
		/// @todo Enable LOG_INFO to ask for reports about unknown TAP IDs.
#if 0
		LOG_INFO("Uknown JTAG TAP ID: 0x%08x", tap->idcode)
		LOG_INFO("Please report the chip name and reported ID code to the openocd project");
#endif
		return true;
	}

	/* Loop over the expected identification codes and test for a match */
	u8 ii;
	for (ii = 0; ii < tap->expected_ids_cnt; ii++)
	{
		if (tap->idcode == tap->expected_ids[ii])
			break;
	}

	/* If none of the expected ids matched, log an error */
	if (ii != tap->expected_ids_cnt)
	{
		LOG_INFO("JTAG Tap/device matched");
		return true;
	}
	jtag_examine_chain_display(LOG_LVL_ERROR, "got",
			tap->dotted_name, tap->idcode);
	for (ii = 0; ii < tap->expected_ids_cnt; ii++)
	{
		char msg[32];
		snprintf(msg, sizeof(msg), "expected %hhu of %hhu",
				ii + 1, tap->expected_ids_cnt);
		jtag_examine_chain_display(LOG_LVL_ERROR, msg,
				tap->dotted_name, tap->expected_ids[ii]);
	}
	return false;
	}

/* Try to examine chain layout according to IEEE 1149.1 Â§12
 */
static int jtag_examine_chain(void)
	{
	u8 idcode_buffer[JTAG_MAX_CHAIN_SIZE * 4];
	unsigned device_count = 0;

	jtag_examine_chain_execute(idcode_buffer, JTAG_MAX_CHAIN_SIZE);

	if (!jtag_examine_chain_check(idcode_buffer, JTAG_MAX_CHAIN_SIZE))
		return ERROR_JTAG_INIT_FAILED;

	/* point at the 1st tap */
	jtag_tap_t *tap = jtag_tap_next_enabled(NULL);
	if (tap == NULL)
	{
		LOG_ERROR("JTAG: No taps enabled?");
		return ERROR_JTAG_INIT_FAILED;
	}

	for (unsigned bit_count = 0; bit_count < (JTAG_MAX_CHAIN_SIZE * 32) - 31;)
	{
		u32 idcode = buf_get_u32(idcode_buffer, bit_count, 32);
		if ((idcode & 1) == 0)
		{
			/* LSB must not be 0, this indicates a device in bypass */
			LOG_WARNING("Tap/Device does not have IDCODE");
			idcode=0;

			bit_count += 1;
		}
		else
		{
	 		/*
			 * End of chain (invalid manufacturer ID) some devices, such
			 * as AVR will output all 1's instead of TDI input value at
			 * end of chain.
				 */
			if (jtag_idcode_is_final(idcode))
				{
				jtag_examine_chain_end(idcode_buffer,
						bit_count + 32, JTAG_MAX_CHAIN_SIZE * 32);
				break;
			}

			jtag_examine_chain_display(LOG_LVL_INFO, "tap/device found",
					tap ? tap->dotted_name : "(not-named)",
					idcode);

			bit_count += 32;
		}
		device_count++;
		if (!tap)
			continue;

			tap->idcode = idcode;

		// ensure the TAP ID does matches what was expected
 		if (!jtag_examine_chain_match_tap(tap))
			return ERROR_JTAG_INIT_FAILED;

		tap = jtag_tap_next_enabled(tap);
	}

	/* see if number of discovered devices matches configuration */
	if (device_count != jtag_tap_count_enabled())
	{
		LOG_ERROR("number of discovered devices in JTAG chain (%i) "
				"does not match (enabled) configuration (%i), total taps: %d",
				device_count, jtag_tap_count_enabled(), jtag_tap_count());
		LOG_ERROR("check the config file and ensure proper JTAG communication"
				" (connections, speed, ...)");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int jtag_validate_chain(void)
{
	jtag_tap_t *tap;
	int total_ir_length = 0;
	u8 *ir_test = NULL;
	scan_field_t field;
	int chain_pos = 0;

	tap = NULL;
	total_ir_length = 0;
	for(;;){
		tap = jtag_tap_next_enabled(tap);
		if( tap == NULL ){
			break;
		}
		total_ir_length += tap->ir_length;
	}

	total_ir_length += 2;
	ir_test = malloc(CEIL(total_ir_length, 8));
	buf_set_ones(ir_test, total_ir_length);

	field.tap = NULL;
	field.num_bits = total_ir_length;
	field.out_value = ir_test;
	field.in_value = ir_test;


	jtag_add_plain_ir_scan(1, &field, TAP_RESET);
	jtag_execute_queue();

	tap = NULL;
	chain_pos = 0;
	int val;
	for(;;){
		tap = jtag_tap_next_enabled(tap);
		if( tap == NULL ){
			break;
		}

		val = buf_get_u32(ir_test, chain_pos, 2);
		if (val != 0x1)
		{
			char *cbuf = buf_to_str(ir_test, total_ir_length, 16);
			LOG_ERROR("Could not validate JTAG scan chain, IR mismatch, scan returned 0x%s. tap=%s pos=%d expected 0x1 got %0x", cbuf, jtag_tap_name(tap), chain_pos, val);
			free(cbuf);
			free(ir_test);
			return ERROR_JTAG_INIT_FAILED;
		}
		chain_pos += tap->ir_length;
	}

	val = buf_get_u32(ir_test, chain_pos, 2);
	if (val != 0x3)
	{
		char *cbuf = buf_to_str(ir_test, total_ir_length, 16);
		LOG_ERROR("Could not validate end of JTAG scan chain, IR mismatch, scan returned 0x%s. pos=%d expected 0x3 got %0x", cbuf, chain_pos, val);
		free(cbuf);
		free(ir_test);
		return ERROR_JTAG_INIT_FAILED;
	}

	free(ir_test);

	return ERROR_OK;
}

enum jtag_tap_cfg_param {
	JCFG_EVENT
};

static Jim_Nvp nvp_config_opts[] = {
	{ .name = "-event",      .value = JCFG_EVENT },

	{ .name = NULL,          .value = -1 }
};

static int jtag_tap_configure_cmd( Jim_GetOptInfo *goi, jtag_tap_t * tap)
{
	Jim_Nvp *n;
	Jim_Obj *o;
	int e;

	/* parse config or cget options */
	while (goi->argc > 0) {
		Jim_SetEmptyResult (goi->interp);

		e = Jim_GetOpt_Nvp(goi, nvp_config_opts, &n);
		if (e != JIM_OK) {
			Jim_GetOpt_NvpUnknown(goi, nvp_config_opts, 0);
			return e;
		}

		switch (n->value) {
			case JCFG_EVENT:
				if (goi->argc == 0) {
					Jim_WrongNumArgs( goi->interp, goi->argc, goi->argv, "-event ?event-name? ..." );
					return JIM_ERR;
				}

				e = Jim_GetOpt_Nvp( goi, nvp_jtag_tap_event, &n );
				if (e != JIM_OK) {
					Jim_GetOpt_NvpUnknown(goi, nvp_jtag_tap_event, 1);
					return e;
				}

				if (goi->isconfigure) {
					if (goi->argc != 1) {
						Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "-event ?event-name? ?EVENT-BODY?");
						return JIM_ERR;
					}
				} else {
					if (goi->argc != 0) {
						Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "-event ?event-name?");
						return JIM_ERR;
					}
				}

				{
					jtag_tap_event_action_t *jteap;

					jteap = tap->event_action;
					/* replace existing? */
					while (jteap) {
						if (jteap->event == (enum jtag_tap_event)n->value) {
							break;
						}
						jteap = jteap->next;
					}

					if (goi->isconfigure) {
						if (jteap == NULL) {
							/* create new */
							jteap = calloc(1, sizeof (*jteap));
						}
						jteap->event = n->value;
						Jim_GetOpt_Obj( goi, &o);
						if (jteap->body) {
							Jim_DecrRefCount(interp, jteap->body);
						}
						jteap->body = Jim_DuplicateObj(goi->interp, o);
						Jim_IncrRefCount(jteap->body);

						/* add to head of event list */
						jteap->next = tap->event_action;
						tap->event_action = jteap;
						Jim_SetEmptyResult(goi->interp);
					} else {
						/* get */
						if (jteap == NULL) {
							Jim_SetEmptyResult(goi->interp);
						} else {
							Jim_SetResult(goi->interp, Jim_DuplicateObj(goi->interp, jteap->body));
						}
					}
				}
				/* loop for more */
				break;
		}
	} /* while (goi->argc) */

	return JIM_OK;
}


static void jtag_tap_init(jtag_tap_t *tap)
{
	assert(0 != tap->ir_length);

	tap->expected = malloc(tap->ir_length);
	tap->expected_mask = malloc(tap->ir_length);
	tap->cur_instr = malloc(tap->ir_length);

	buf_set_u32(tap->expected, 0, tap->ir_length, tap->ir_capture_value);
	buf_set_u32(tap->expected_mask, 0, tap->ir_length, tap->ir_capture_mask);
	buf_set_ones(tap->cur_instr, tap->ir_length);

	// place TAP in bypass mode
	tap->bypass = 1;
	// register the reset callback for the TAP
	jtag_register_event_callback(&jtag_reset_callback, tap);

	LOG_DEBUG("Created Tap: %s @ abs position %d, "
			"irlen %d, capture: 0x%x mask: 0x%x", tap->dotted_name,
				tap->abs_chain_position, tap->ir_length,
				tap->ir_capture_value, tap->ir_capture_mask);
	jtag_tap_add(tap);
}

static void jtag_tap_free(jtag_tap_t *tap)
{
	/// @todo is anything missing? no memory leaks please 
	free((void *)tap->expected_ids);
	free((void *)tap->chip);
	free((void *)tap->tapname);
	free((void *)tap->dotted_name);
	free(tap);
}

static int jim_newtap_cmd( Jim_GetOptInfo *goi )
{
	jtag_tap_t *pTap;
	jim_wide w;
	int x;
	int e;
	int reqbits;
	Jim_Nvp *n;
	char *cp;
	const Jim_Nvp opts[] = {
#define NTAP_OPT_IRLEN     0
		{ .name = "-irlen"			,	.value = NTAP_OPT_IRLEN },
#define NTAP_OPT_IRMASK    1
		{ .name = "-irmask"			,	.value = NTAP_OPT_IRMASK },
#define NTAP_OPT_IRCAPTURE 2
		{ .name = "-ircapture"		,	.value = NTAP_OPT_IRCAPTURE },
#define NTAP_OPT_ENABLED   3
		{ .name = "-enable"			,	.value = NTAP_OPT_ENABLED },
#define NTAP_OPT_DISABLED  4
		{ .name = "-disable"		,	.value = NTAP_OPT_DISABLED },
#define NTAP_OPT_EXPECTED_ID 5
		{ .name = "-expected-id"	,	.value = NTAP_OPT_EXPECTED_ID },
		{ .name = NULL				,	.value = -1 },
	};

	pTap = malloc( sizeof(jtag_tap_t) );
	memset( pTap, 0, sizeof(*pTap) );
	if( !pTap ){
		Jim_SetResult_sprintf( goi->interp, "no memory");
		return JIM_ERR;
	}
	/*
	 * we expect CHIP + TAP + OPTIONS
	 * */
	if( goi->argc < 3 ){
		Jim_SetResult_sprintf(goi->interp, "Missing CHIP TAP OPTIONS ....");
		return JIM_ERR;
	}
	Jim_GetOpt_String( goi, &cp, NULL );
	pTap->chip = strdup(cp);

	Jim_GetOpt_String( goi, &cp, NULL );
	pTap->tapname = strdup(cp);

	/* name + dot + name + null */
	x = strlen(pTap->chip) + 1 + strlen(pTap->tapname) + 1;
	cp = malloc( x );
	sprintf( cp, "%s.%s", pTap->chip, pTap->tapname );
	pTap->dotted_name = cp;

	LOG_DEBUG("Creating New Tap, Chip: %s, Tap: %s, Dotted: %s, %d params",
			  pTap->chip, pTap->tapname, pTap->dotted_name, goi->argc);

	/* default is enabled */
	pTap->enabled = 1;

	/* deal with options */
#define NTREQ_IRLEN      1
#define NTREQ_IRCAPTURE  2
#define NTREQ_IRMASK     4

	/* clear them as we find them */
	reqbits = (NTREQ_IRLEN | NTREQ_IRCAPTURE | NTREQ_IRMASK);

	while( goi->argc ){
		e = Jim_GetOpt_Nvp( goi, opts, &n );
		if( e != JIM_OK ){
			Jim_GetOpt_NvpUnknown( goi, opts, 0 );
			return e;
		}
		LOG_DEBUG("Processing option: %s", n->name );
		switch( n->value ){
		case NTAP_OPT_ENABLED:
			pTap->enabled = 1;
			break;
		case NTAP_OPT_DISABLED:
			pTap->enabled = 0;
			break;
		case NTAP_OPT_EXPECTED_ID:
		{
			u32 *new_expected_ids;

			e = Jim_GetOpt_Wide( goi, &w );
			if( e != JIM_OK) {
				Jim_SetResult_sprintf(goi->interp, "option: %s bad parameter", n->name);
				return e;
			}

			new_expected_ids = malloc(sizeof(u32) * (pTap->expected_ids_cnt + 1));
			if (new_expected_ids == NULL) {
				Jim_SetResult_sprintf( goi->interp, "no memory");
				return JIM_ERR;
			}

			memcpy(new_expected_ids, pTap->expected_ids, sizeof(u32) * pTap->expected_ids_cnt);

			new_expected_ids[pTap->expected_ids_cnt] = w;

			free(pTap->expected_ids);
			pTap->expected_ids = new_expected_ids;
			pTap->expected_ids_cnt++;
			break;
		}
		case NTAP_OPT_IRLEN:
		case NTAP_OPT_IRMASK:
		case NTAP_OPT_IRCAPTURE:
			e = Jim_GetOpt_Wide( goi, &w );
			if( e != JIM_OK ){
				Jim_SetResult_sprintf( goi->interp, "option: %s bad parameter", n->name );
				return e;
			}
			if( (w < 0) || (w > 0xffff) ){
				/* wacky value */
				Jim_SetResult_sprintf( goi->interp, "option: %s - wacky value: %d (0x%x)",
									   n->name, (int)(w), (int)(w));
				return JIM_ERR;
			}
			switch(n->value){
			case NTAP_OPT_IRLEN:
				pTap->ir_length = w;
				reqbits &= (~(NTREQ_IRLEN));
				break;
			case NTAP_OPT_IRMASK:
				pTap->ir_capture_mask = w;
				reqbits &= (~(NTREQ_IRMASK));
				break;
			case NTAP_OPT_IRCAPTURE:
				pTap->ir_capture_value = w;
				reqbits &= (~(NTREQ_IRCAPTURE));
				break;
			}
		} /* switch(n->value) */
	} /* while( goi->argc ) */

	/* Did all the required option bits get cleared? */
	if (0 == reqbits)
	{
		jtag_tap_init(pTap);
		return ERROR_OK;
	}

		Jim_SetResult_sprintf( goi->interp,
							   "newtap: %s missing required parameters",
							   pTap->dotted_name);
	jtag_tap_free(pTap);
		return JIM_ERR;
	}

static int jim_jtag_command( Jim_Interp *interp, int argc, Jim_Obj *const *argv )
{
	Jim_GetOptInfo goi;
	int e;
	Jim_Nvp *n;
	Jim_Obj *o;
	struct command_context_s *context;

	enum {
		JTAG_CMD_INTERFACE,
		JTAG_CMD_INIT_RESET,
		JTAG_CMD_NEWTAP,
		JTAG_CMD_TAPENABLE,
		JTAG_CMD_TAPDISABLE,
		JTAG_CMD_TAPISENABLED,
		JTAG_CMD_CONFIGURE,
		JTAG_CMD_CGET
	};

	const Jim_Nvp jtag_cmds[] = {
		{ .name = "interface"     , .value = JTAG_CMD_INTERFACE },
		{ .name = "arp_init-reset", .value = JTAG_CMD_INIT_RESET },
		{ .name = "newtap"        , .value = JTAG_CMD_NEWTAP },
		{ .name = "tapisenabled"     , .value = JTAG_CMD_TAPISENABLED },
		{ .name = "tapenable"     , .value = JTAG_CMD_TAPENABLE },
		{ .name = "tapdisable"    , .value = JTAG_CMD_TAPDISABLE },
		{ .name = "configure"     , .value = JTAG_CMD_CONFIGURE },
		{ .name = "cget"          , .value = JTAG_CMD_CGET },

		{ .name = NULL, .value = -1 },
	};

	context = Jim_GetAssocData(interp, "context");
	/* go past the command */
	Jim_GetOpt_Setup( &goi, interp, argc-1, argv+1 );

	e = Jim_GetOpt_Nvp( &goi, jtag_cmds, &n );
	if( e != JIM_OK ){
		Jim_GetOpt_NvpUnknown( &goi, jtag_cmds, 0 );
		return e;
	}
		Jim_SetEmptyResult( goi.interp );
	switch( n->value ){
	case JTAG_CMD_INTERFACE:
		/* return the name of the interface */
		/* TCL code might need to know the exact type... */
		/* FUTURE: we allow this as a means to "set" the interface. */
		if( goi.argc != 0 ){
			Jim_WrongNumArgs( goi.interp, 1, goi.argv-1, "(no params)");
			return JIM_ERR;
		}
		Jim_SetResultString( goi.interp, jtag_interface->name, -1 );
		return JIM_OK;
	case JTAG_CMD_INIT_RESET:
		if( goi.argc != 0 ){
			Jim_WrongNumArgs( goi.interp, 1, goi.argv-1, "(no params)");
			return JIM_ERR;
		}
		e = jtag_init_reset(context);
		if( e != ERROR_OK ){
			Jim_SetResult_sprintf( goi.interp, "error: %d", e);
			return JIM_ERR;
		}
		return JIM_OK;
	case JTAG_CMD_NEWTAP:
		return jim_newtap_cmd( &goi );
		break;
	case JTAG_CMD_TAPISENABLED:
	case JTAG_CMD_TAPENABLE:
	case JTAG_CMD_TAPDISABLE:
		if( goi.argc != 1 ){
			Jim_SetResultString( goi.interp, "Too many parameters",-1 );
			return JIM_ERR;
		}

		{
			jtag_tap_t *t;
			t = jtag_tap_by_jim_obj( goi.interp, goi.argv[0] );
			if( t == NULL ){
				return JIM_ERR;
			}
			switch( n->value ){
			case JTAG_CMD_TAPISENABLED:
				e = t->enabled;
				break;
			case JTAG_CMD_TAPENABLE:
				jtag_tap_handle_event( t, JTAG_TAP_EVENT_ENABLE);
				e = 1;
				t->enabled = e;
				break;
			case JTAG_CMD_TAPDISABLE:
				jtag_tap_handle_event( t, JTAG_TAP_EVENT_DISABLE);
				e = 0;
				t->enabled = e;
				break;
			}
			Jim_SetResult( goi.interp, Jim_NewIntObj( goi.interp, e ) );
			return JIM_OK;
		}
		break;

	case JTAG_CMD_CGET:
		if( goi.argc < 2 ){
			Jim_WrongNumArgs( goi.interp, 0, NULL, "?tap-name? -option ...");
			return JIM_ERR;
		}

		{
			jtag_tap_t *t;

			Jim_GetOpt_Obj(&goi, &o);
			t = jtag_tap_by_jim_obj( goi.interp, o );
			if( t == NULL ){
				return JIM_ERR;
			}

			goi.isconfigure = 0;
			return jtag_tap_configure_cmd( &goi, t);
		}
		break;

	case JTAG_CMD_CONFIGURE:
		if( goi.argc < 3 ){
			Jim_WrongNumArgs( goi.interp, 0, NULL, "?tap-name? -option ?VALUE? ...");
			return JIM_ERR;
		}

		{
			jtag_tap_t *t;

			Jim_GetOpt_Obj(&goi, &o);
			t = jtag_tap_by_jim_obj( goi.interp, o );
			if( t == NULL ){
				return JIM_ERR;
			}

			goi.isconfigure = 1;
			return jtag_tap_configure_cmd( &goi, t);
		}
	}

	return JIM_ERR;
}

int jtag_register_commands(struct command_context_s *cmd_ctx)
{
	register_jim( cmd_ctx, "jtag", jim_jtag_command, "perform jtag tap actions");

	register_command(cmd_ctx, NULL, "interface", handle_interface_command,
		COMMAND_CONFIG, "try to configure interface");
	register_command(cmd_ctx, NULL, "jtag_speed", handle_jtag_speed_command,
		COMMAND_ANY, "(DEPRECATED) set jtag speed (if supported)");
	register_command(cmd_ctx, NULL, "jtag_khz", handle_jtag_khz_command,
		COMMAND_ANY, "set maximum jtag speed (if supported); "
		"parameter is maximum khz, or 0 for adaptive clocking (RTCK).");
	register_command(cmd_ctx, NULL, "jtag_device", handle_jtag_device_command,
		COMMAND_CONFIG, "(DEPRECATED) jtag_device <ir_length> <ir_expected> <ir_mask>");
	register_command(cmd_ctx, NULL, "reset_config", handle_reset_config_command,
		COMMAND_ANY,
		"[none/trst_only/srst_only/trst_and_srst] [srst_pulls_trst/trst_pulls_srst] [combined/separate] [trst_push_pull/trst_open_drain] [srst_push_pull/srst_open_drain]");
	register_command(cmd_ctx, NULL, "jtag_nsrst_delay", handle_jtag_nsrst_delay_command,
		COMMAND_ANY, "jtag_nsrst_delay <ms> - delay after deasserting srst in ms");
	register_command(cmd_ctx, NULL, "jtag_ntrst_delay", handle_jtag_ntrst_delay_command,
		COMMAND_ANY, "jtag_ntrst_delay <ms> - delay after deasserting trst in ms");

	register_command(cmd_ctx, NULL, "scan_chain", handle_scan_chain_command,
		COMMAND_EXEC, "print current scan chain configuration");

	register_command(cmd_ctx, NULL, "endstate", handle_endstate_command,
		COMMAND_EXEC, "finish JTAG operations in <tap_state>");
	register_command(cmd_ctx, NULL, "jtag_reset", handle_jtag_reset_command,
		COMMAND_EXEC, "toggle reset lines <trst> <srst>");
	register_command(cmd_ctx, NULL, "runtest", handle_runtest_command,
		COMMAND_EXEC, "move to Run-Test/Idle, and execute <num_cycles>");
	register_command(cmd_ctx, NULL, "irscan", handle_irscan_command,
		COMMAND_EXEC, "execute IR scan <device> <instr> [dev2] [instr2] ...");
	register_jim(cmd_ctx, "drscan", Jim_Command_drscan, "execute DR scan <device> <num_bits> <value> <num_bits1> <value2> ...");
	register_jim(cmd_ctx, "flush_count", Jim_Command_flush_count, "returns number of times the JTAG queue has been flushed");

	register_command(cmd_ctx, NULL, "verify_ircapture", handle_verify_ircapture_command,
		COMMAND_ANY, "verify value captured during Capture-IR <enable|disable>");
	register_command(cmd_ctx, NULL, "verify_jtag", handle_verify_jtag_command,
		COMMAND_ANY, "verify value capture <enable|disable>");
	register_command(cmd_ctx, NULL, "tms_sequence", handle_tms_sequence_command,
		COMMAND_ANY, "choose short(default) or long tms_sequence <short|long>");
	return ERROR_OK;
}

int jtag_interface_init(struct command_context_s *cmd_ctx)
{
	if (jtag)
		return ERROR_OK;

	if (!jtag_interface)
	{
		/* nothing was previously specified by "interface" command */
		LOG_ERROR("JTAG interface has to be specified, see \"interface\" command");
		return ERROR_JTAG_INVALID_INTERFACE;
	}
	if(hasKHz)
	{
		jtag_interface->khz(jtag_get_speed_khz(), &jtag_speed);
		hasKHz = false;
	}

	if (jtag_interface->init() != ERROR_OK)
		return ERROR_JTAG_INIT_FAILED;

	jtag = jtag_interface;
	return ERROR_OK;
}

static int jtag_init_inner(struct command_context_s *cmd_ctx)
{
	jtag_tap_t *tap;
	int retval;

	LOG_DEBUG("Init JTAG chain");

	tap = jtag_tap_next_enabled(NULL);
	if( tap == NULL ){
		LOG_ERROR("There are no enabled taps?");
		return ERROR_JTAG_INIT_FAILED;
	}

	jtag_add_tlr();
	if ((retval=jtag_execute_queue())!=ERROR_OK)
		return retval;

	/* examine chain first, as this could discover the real chain layout */
	if (jtag_examine_chain() != ERROR_OK)
	{
		LOG_ERROR("trying to validate configured JTAG chain anyway...");
	}

	if (jtag_validate_chain() != ERROR_OK)
	{
		LOG_WARNING("Could not validate JTAG chain, continuing anyway...");
	}

	return ERROR_OK;
}

int jtag_interface_quit(void)
{
	if (!jtag || !jtag->quit)
		return ERROR_OK;

	// close the JTAG interface
	int result = jtag->quit();
	if (ERROR_OK != result)
		LOG_ERROR("failed: %d", result);

	return ERROR_OK;
}


int jtag_init_reset(struct command_context_s *cmd_ctx)
{
	int retval;

	if ((retval=jtag_interface_init(cmd_ctx)) != ERROR_OK)
		return retval;

	LOG_DEBUG("Trying to bring the JTAG controller to life by asserting TRST / RESET");

	/* Reset can happen after a power cycle.
	 *
	 * Ideally we would only assert TRST or run RESET before the target reset.
	 *
	 * However w/srst_pulls_trst, trst is asserted together with the target
	 * reset whether we want it or not.
	 *
	 * NB! Some targets have JTAG circuitry disabled until a
	 * trst & srst has been asserted.
	 *
	 * NB! here we assume nsrst/ntrst delay are sufficient!
	 *
	 * NB! order matters!!!! srst *can* disconnect JTAG circuitry
	 *
	 */
	jtag_add_reset(1, 0); /* RESET or TRST */
	if (jtag_reset_config & RESET_HAS_SRST)
	{
		jtag_add_reset(1, 1);
		if ((jtag_reset_config & RESET_SRST_PULLS_TRST)==0)
			jtag_add_reset(0, 1);
	}
	jtag_add_reset(0, 0);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
		return retval;

	/* Check that we can communication on the JTAG chain + eventually we want to
	 * be able to perform enumeration only after OpenOCD has started
	 * telnet and GDB server
	 *
	 * That would allow users to more easily perform any magic they need to before
	 * reset happens.
	 */
	return jtag_init_inner(cmd_ctx);
}

int jtag_init(struct command_context_s *cmd_ctx)
{
	int retval;
	if ((retval=jtag_interface_init(cmd_ctx)) != ERROR_OK)
		return retval;
	if (jtag_init_inner(cmd_ctx)==ERROR_OK)
	{
		return ERROR_OK;
	}
	return jtag_init_reset(cmd_ctx);
}

void jtag_set_speed_khz(unsigned khz)
{
	speed_khz = khz;
}
unsigned jtag_get_speed_khz(void)
{
	return speed_khz;
}

static int default_khz(int khz, int *jtag_speed)
{
	LOG_ERROR("Translation from khz to jtag_speed not implemented");
	return ERROR_FAIL;
}

static int default_speed_div(int speed, int *khz)
{
	LOG_ERROR("Translation from jtag_speed to khz not implemented");
	return ERROR_FAIL;
}

static int default_power_dropout(int *dropout)
{
	*dropout=0; /* by default we can't detect power dropout */
	return ERROR_OK;
}

static int default_srst_asserted(int *srst_asserted)
{
	*srst_asserted=0; /* by default we can't detect srst asserted */
	return ERROR_OK;
}

static int handle_interface_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int i;
	int retval;

	/* check whether the interface is already configured */
	if (jtag_interface)
	{
		LOG_WARNING("Interface already configured, ignoring");
		return ERROR_OK;
	}

	/* interface name is a mandatory argument */
	if (argc < 1 || args[0][0] == '\0')
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	for (i=0; jtag_interfaces[i]; i++)
	{
		if (strcmp(args[0], jtag_interfaces[i]->name) == 0)
		{
			if ((retval = jtag_interfaces[i]->register_commands(cmd_ctx)) != ERROR_OK)
			{
				return retval;
			}

			jtag_interface = jtag_interfaces[i];

			if (jtag_interface->khz == NULL)
			{
				jtag_interface->khz = default_khz;
			}
			if (jtag_interface->speed_div == NULL)
			{
				jtag_interface->speed_div = default_speed_div;
			}
			if (jtag_interface->power_dropout == NULL)
			{
				jtag_interface->power_dropout = default_power_dropout;
			}
			if (jtag_interface->srst_asserted == NULL)
			{
				jtag_interface->srst_asserted = default_srst_asserted;
			}

			return ERROR_OK;
		}
	}

	/* no valid interface was found (i.e. the configuration option,
	 * didn't match one of the compiled-in interfaces
	 */
	LOG_ERROR("No valid jtag interface found (%s)", args[0]);
	LOG_ERROR("compiled-in jtag interfaces:");
	for (i = 0; jtag_interfaces[i]; i++)
	{
		LOG_ERROR("%i: %s", i, jtag_interfaces[i]->name);
	}

	return ERROR_JTAG_INVALID_INTERFACE;
}

static int handle_jtag_device_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int e;
	char buf[1024];
	Jim_Obj *newargs[ 10 ];
	/*
	 * CONVERT SYNTAX
	 * argv[-1] = command
	 * argv[ 0] = ir length
	 * argv[ 1] = ir capture
	 * argv[ 2] = ir mask
	 * argv[ 3] = not actually used by anything but in the docs
	 */

	if( argc < 4 ){
		command_print( cmd_ctx, "OLD DEPRECATED SYNTAX: Please use the NEW syntax");
		return ERROR_OK;
	}
	command_print( cmd_ctx, "OLD SYNTAX: DEPRECATED - translating to new syntax");
	command_print( cmd_ctx, "jtag newtap CHIP TAP -irlen %s -ircapture %s -irvalue %s",
				   args[0],
				   args[1],
				   args[2] );
	command_print( cmd_ctx, "Example: STM32 has 2 taps, the cortexM3(len4) + boundaryscan(len5)");
	command_print( cmd_ctx, "jtag newtap stm32 cortexm3 ....., thus creating the tap: \"stm32.cortexm3\"");
	command_print( cmd_ctx, "jtag newtap stm32 boundary ....., and the tap: \"stm32.boundary\"");
	command_print( cmd_ctx, "And then refer to the taps by the dotted name.");

	newargs[0] = Jim_NewStringObj( interp, "jtag", -1   );
	newargs[1] = Jim_NewStringObj( interp, "newtap", -1 );
	sprintf( buf, "chip%d", jtag_tap_count() );
	newargs[2] = Jim_NewStringObj( interp, buf, -1 );
	sprintf( buf, "tap%d", jtag_tap_count() );
	newargs[3] = Jim_NewStringObj( interp, buf, -1  );
	newargs[4] = Jim_NewStringObj( interp, "-irlen", -1  );
	newargs[5] = Jim_NewStringObj( interp, args[0], -1  );
	newargs[6] = Jim_NewStringObj( interp, "-ircapture", -1  );
	newargs[7] = Jim_NewStringObj( interp, args[1], -1  );
	newargs[8] = Jim_NewStringObj( interp, "-irmask", -1  );
	newargs[9] = Jim_NewStringObj( interp, args[2], -1  );

	command_print( cmd_ctx, "NEW COMMAND:");
	sprintf( buf, "%s %s %s %s %s %s %s %s %s %s",
			 Jim_GetString( newargs[0], NULL ),
			 Jim_GetString( newargs[1], NULL ),
			 Jim_GetString( newargs[2], NULL ),
			 Jim_GetString( newargs[3], NULL ),
			 Jim_GetString( newargs[4], NULL ),
			 Jim_GetString( newargs[5], NULL ),
			 Jim_GetString( newargs[6], NULL ),
			 Jim_GetString( newargs[7], NULL ),
			 Jim_GetString( newargs[8], NULL ),
			 Jim_GetString( newargs[9], NULL ) );

	e = jim_jtag_command( interp, 10, newargs );
	if( e != JIM_OK ){
		command_print( cmd_ctx, "%s", Jim_GetString( Jim_GetResult(interp), NULL ) );
	}
	return e;
}

static int handle_scan_chain_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	jtag_tap_t *tap;

	tap = jtag_all_taps();
	command_print(cmd_ctx, "     TapName            | Enabled |   IdCode      Expected    IrLen IrCap  IrMask Instr     ");
	command_print(cmd_ctx, "---|--------------------|---------|------------|------------|------|------|------|---------");

	while( tap ){
		u32 expected, expected_mask, cur_instr, ii;
		expected = buf_get_u32(tap->expected, 0, tap->ir_length);
		expected_mask = buf_get_u32(tap->expected_mask, 0, tap->ir_length);
		cur_instr = buf_get_u32(tap->cur_instr, 0, tap->ir_length);

		command_print(cmd_ctx,
					  "%2d | %-18s |    %c    | 0x%08x | 0x%08x | 0x%02x | 0x%02x | 0x%02x | 0x%02x",
					  tap->abs_chain_position,
					  tap->dotted_name,
					  tap->enabled ? 'Y' : 'n',
					  tap->idcode,
					  (tap->expected_ids_cnt > 0 ? tap->expected_ids[0] : 0),
					  tap->ir_length,
					  expected,
					  expected_mask,
					  cur_instr);

		for (ii = 1; ii < tap->expected_ids_cnt; ii++) {
			command_print(cmd_ctx, "   |                    |         |            | 0x%08x |      |      |      |         ",
						  tap->expected_ids[ii]);
		}

		tap = tap->next_tap;
	}

	return ERROR_OK;
}

static int handle_reset_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int new_cfg = 0;
	int mask = 0;

	if (argc < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Original versions cared about the order of these tokens:
	 *   reset_config signals [combination [trst_type [srst_type]]]
	 * They also clobbered the previous configuration even on error.
	 *
	 * Here we don't care about the order, and only change values
	 * which have been explicitly specified.
	 */
	for (; argc; argc--, args++) {
		int tmp = 0;
		int m;

		/* signals */
		m = RESET_HAS_TRST | RESET_HAS_SRST;
		if (strcmp(*args, "none") == 0)
			tmp = RESET_NONE;
		else if (strcmp(*args, "trst_only") == 0)
			tmp = RESET_HAS_TRST;
		else if (strcmp(*args, "srst_only") == 0)
			tmp = RESET_HAS_SRST;
		else if (strcmp(*args, "trst_and_srst") == 0)
			tmp = RESET_HAS_TRST | RESET_HAS_SRST;
		else
			m = 0;
		if (mask & m) {
			LOG_ERROR("extra reset_config %s spec (%s)",
					"signal", *args);
			return ERROR_INVALID_ARGUMENTS;
		}
		if (m)
			goto next;

		/* combination (options for broken wiring) */
		m = RESET_SRST_PULLS_TRST | RESET_TRST_PULLS_SRST;
		if (strcmp(*args, "separate") == 0)
			/* separate reset lines - default */;
		else if (strcmp(*args, "srst_pulls_trst") == 0)
			tmp |= RESET_SRST_PULLS_TRST;
		else if (strcmp(*args, "trst_pulls_srst") == 0)
			tmp |= RESET_TRST_PULLS_SRST;
		else if (strcmp(*args, "combined") == 0)
			tmp |= RESET_SRST_PULLS_TRST | RESET_TRST_PULLS_SRST;
		else
			m = 0;
		if (mask & m) {
			LOG_ERROR("extra reset_config %s spec (%s)",
					"combination", *args);
			return ERROR_INVALID_ARGUMENTS;
		}
		if (m)
			goto next;

		/* trst_type (NOP without HAS_TRST) */
		m = RESET_TRST_OPEN_DRAIN;
		if (strcmp(*args, "trst_open_drain") == 0)
			tmp |= RESET_TRST_OPEN_DRAIN;
		else if (strcmp(*args, "trst_push_pull") == 0)
			/* push/pull from adapter - default */;
		else
			m = 0;
		if (mask & m) {
			LOG_ERROR("extra reset_config %s spec (%s)",
					"trst_type", *args);
			return ERROR_INVALID_ARGUMENTS;
		}
		if (m)
			goto next;

		/* srst_type (NOP without HAS_SRST) */
		m |= RESET_SRST_PUSH_PULL;
		if (strcmp(*args, "srst_push_pull") == 0)
			tmp |= RESET_SRST_PUSH_PULL;
		else if (strcmp(*args, "srst_open_drain") == 0)
			/* open drain from adapter - default */;
		else
			m = 0;
		if (mask & m) {
			LOG_ERROR("extra reset_config %s spec (%s)",
					"srst_type", *args);
			return ERROR_INVALID_ARGUMENTS;
		}
		if (m)
			goto next;

		/* caller provided nonsense; fail */
		LOG_ERROR("unknown reset_config flag (%s)", *args);
		return ERROR_INVALID_ARGUMENTS;

next:
		/* Remember the bits which were specified (mask)
		 * and their new values (new_cfg).
		 */
		mask |= m;
		new_cfg |= tmp;
	}

	/* clear previous values of those bits, save new values */
	jtag_reset_config &= ~mask;
	jtag_reset_config |= new_cfg;

	return ERROR_OK;
}

static int handle_jtag_nsrst_delay_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc < 1)
	{
		LOG_ERROR("jtag_nsrst_delay <ms> command takes one required argument");
		exit(-1);
	}
	else
	{
		jtag_set_nsrst_delay(strtoul(args[0], NULL, 0));
	}

	return ERROR_OK;
}

static int handle_jtag_ntrst_delay_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc < 1)
	{
		LOG_ERROR("jtag_ntrst_delay <ms> command takes one required argument");
		exit(-1);
	}
	else
	{
		jtag_set_ntrst_delay(strtoul(args[0], NULL, 0));
	}

	return ERROR_OK;
}

static int handle_jtag_speed_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval=ERROR_OK;

	if (argc == 1)
	{
		LOG_DEBUG("handle jtag speed");

		int cur_speed = 0;
		cur_speed = jtag_speed = strtoul(args[0], NULL, 0);

		/* this command can be called during CONFIG,
		 * in which case jtag isn't initialized */
		if (jtag)
		{
			retval=jtag->speed(cur_speed);
		}
	} else if (argc == 0)
	{
	} else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	command_print(cmd_ctx, "jtag_speed: %d", jtag_speed);

	return retval;
}

static int handle_jtag_khz_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval=ERROR_OK;
	LOG_DEBUG("handle jtag khz");

	int cur_speed = 0;
	if(argc == 1)
	{
		jtag_set_speed_khz(strtoul(args[0], NULL, 0));
		if (jtag != NULL)
		{
			LOG_DEBUG("have interface set up");
			int speed_div1;
			if ((retval=jtag->khz(jtag_get_speed_khz(), &speed_div1))!=ERROR_OK)
			{
				jtag_set_speed_khz(0);
				return retval;
			}

			cur_speed = jtag_speed = speed_div1;

			retval=jtag->speed(cur_speed);
		} else
		{
			hasKHz = true;
		}
	} else if (argc==0)
	{
	} else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	cur_speed = jtag_get_speed_khz();

	if (jtag!=NULL)
	{
		if ((retval=jtag->speed_div(jtag_speed, &cur_speed))!=ERROR_OK)
			return retval;
	}

	if (cur_speed)
		command_print(cmd_ctx, "%d kHz", cur_speed);
	else
		command_print(cmd_ctx, "RCLK - adaptive");
	return retval;

}

static int handle_endstate_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	tap_state_t state = tap_state_by_name(args[0]);
	if (state < 0)
	{
			command_print( cmd_ctx, "Invalid state name: %s\n", args[0] );
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		jtag_set_end_state(state);
		jtag_execute_queue();

	command_print(cmd_ctx, "current endstate: %s",
			tap_state_name(cmd_queue_end_state));

	return ERROR_OK;
}

static int handle_jtag_reset_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int trst = -1;
	int srst = -1;

	if (argc < 2)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (args[0][0] == '1')
		trst = 1;
	else if (args[0][0] == '0')
		trst = 0;
	else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (args[1][0] == '1')
		srst = 1;
	else if (args[1][0] == '0')
		srst = 0;
	else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (jtag_interface_init(cmd_ctx) != ERROR_OK)
		return ERROR_JTAG_INIT_FAILED;

	jtag_add_reset(trst, srst);
	jtag_execute_queue();

	return ERROR_OK;
}

static int handle_runtest_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc < 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	jtag_add_runtest(strtol(args[0], NULL, 0), jtag_get_end_state());
	jtag_execute_queue();

	return ERROR_OK;

}

/*
 * For "irscan" or "drscan" commands, the "end" (really, "next") state
 * should be stable ... and *NOT* a shift state, otherwise free-running
 * jtag clocks could change the values latched by the update state.
 */
static bool scan_is_safe(tap_state_t state)
{
	switch (state)
	{
	case TAP_RESET:
	case TAP_IDLE:
	case TAP_DRPAUSE:
	case TAP_IRPAUSE:
		return true;
	default:
		return false;
	}
}


static int handle_irscan_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int i;
	scan_field_t *fields;
	jtag_tap_t *tap;
	tap_state_t endstate;

	if ((argc < 2) || (argc % 2))
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* optional "-endstate" "statename" at the end of the arguments,
	 * so that e.g. IRPAUSE can let us load the data register before
	 * entering RUN/IDLE to execute the instruction we load here.
	 */
	endstate = TAP_IDLE;

	if( argc >= 4 ){
		/* have at least one pair of numbers. */
		/* is last pair the magic text? */
		if( 0 == strcmp( "-endstate", args[ argc - 2 ] ) ){
			const char *cpA;
			const char *cpS;
			cpA = args[ argc-1 ];
			for( endstate = 0 ; endstate < TAP_NUM_STATES ; endstate++ ){
				cpS = tap_state_name( endstate );
				if( 0 == strcmp( cpA, cpS ) ){
					break;
				}
			}
			if( endstate >= TAP_NUM_STATES ){
				return ERROR_COMMAND_SYNTAX_ERROR;
			} else {
				if (!scan_is_safe(endstate))
					LOG_WARNING("irscan with unsafe "
							"endstate \"%s\"", cpA);
				/* found - remove the last 2 args */
				argc -= 2;
			}
		}
	}

	int num_fields = argc / 2;

	fields = malloc(sizeof(scan_field_t) * num_fields);

	for (i = 0; i < num_fields; i++)
	{
		tap = jtag_tap_by_string( args[i*2] );
		if (tap==NULL)
		{
			command_print( cmd_ctx, "Tap: %s unknown", args[i*2] );
			return ERROR_FAIL;
		}
		int field_size = tap->ir_length;
		fields[i].tap = tap;
		fields[i].num_bits = field_size;
		fields[i].out_value = malloc(CEIL(field_size, 8));
		buf_set_u32(fields[i].out_value, 0, field_size, strtoul(args[i*2+1], NULL, 0));
		fields[i].in_value = NULL;
	}

	/* did we have an endstate? */
	jtag_add_ir_scan(num_fields, fields, endstate);

	int retval=jtag_execute_queue();

	for (i = 0; i < num_fields; i++)
		free(fields[i].out_value);

	free (fields);

	return retval;
}

static int Jim_Command_drscan(Jim_Interp *interp, int argc, Jim_Obj *const *args)
{
	int retval;
	scan_field_t *fields;
	int num_fields;
	int field_count = 0;
	int i, e;
	jtag_tap_t *tap;
	tap_state_t endstate;

	/* args[1] = device
	 * args[2] = num_bits
	 * args[3] = hex string
	 * ... repeat num bits and hex string ...
	 *
	 * .. optionally:
	*     args[N-2] = "-endstate"
	 *     args[N-1] = statename
	 */
	if ((argc < 4) || ((argc % 2)!=0))
	{
		Jim_WrongNumArgs(interp, 1, args, "wrong arguments");
		return JIM_ERR;
	}

	endstate = TAP_IDLE;

	/* validate arguments as numbers */
	e = JIM_OK;
	for (i = 2; i < argc; i+=2)
	{
		long bits;
		const char *cp;

		e = Jim_GetLong(interp, args[i], &bits);
		/* If valid - try next arg */
		if( e == JIM_OK ){
			continue;
		}

		/* Not valid.. are we at the end? */
		if ( ((i+2) != argc) ){
			/* nope, then error */
			return e;
		}

		/* it could be: "-endstate FOO"
		 * e.g. DRPAUSE so we can issue more instructions
		 * before entering RUN/IDLE and executing them.
		 */

		/* get arg as a string. */
		cp = Jim_GetString( args[i], NULL );
		/* is it the magic? */
		if( 0 == strcmp( "-endstate", cp ) ){
			/* is the statename valid? */
			cp = Jim_GetString( args[i+1], NULL );

			/* see if it is a valid state name */
			endstate = tap_state_by_name(cp);
			if( endstate < 0 ){
				/* update the error message */
				Jim_SetResult_sprintf(interp,"endstate: %s invalid", cp );
			} else {
				if (!scan_is_safe(endstate))
					LOG_WARNING("drscan with unsafe "
							"endstate \"%s\"", cp);

				/* valid - so clear the error */
				e = JIM_OK;
				/* and remove the last 2 args */
				argc -= 2;
			}
		}

		/* Still an error? */
		if( e != JIM_OK ){
			return e; /* too bad */
		}
	} /* validate args */

	tap = jtag_tap_by_jim_obj( interp, args[1] );
	if( tap == NULL ){
		return JIM_ERR;
	}

	num_fields=(argc-2)/2;
	fields = malloc(sizeof(scan_field_t) * num_fields);
	for (i = 2; i < argc; i+=2)
	{
		long bits;
		int len;
		const char *str;

		Jim_GetLong(interp, args[i], &bits);
		str = Jim_GetString(args[i+1], &len);

		fields[field_count].tap = tap;
		fields[field_count].num_bits = bits;
		fields[field_count].out_value = malloc(CEIL(bits, 8));
		str_to_buf(str, len, fields[field_count].out_value, bits, 0);
		fields[field_count].in_value = fields[field_count].out_value;
		field_count++;
	}

	jtag_add_dr_scan(num_fields, fields, endstate);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
	{
		Jim_SetResultString(interp, "drscan: jtag execute failed",-1);
		return JIM_ERR;
	}

	field_count=0;
	Jim_Obj *list = Jim_NewListObj(interp, NULL, 0);
	for (i = 2; i < argc; i+=2)
	{
		long bits;
		char *str;

		Jim_GetLong(interp, args[i], &bits);
		str = buf_to_str(fields[field_count].in_value, bits, 16);
		free(fields[field_count].out_value);

		Jim_ListAppendElement(interp, list, Jim_NewStringObj(interp, str, strlen(str)));
		free(str);
		field_count++;
	}

	Jim_SetResult(interp, list);

	free(fields);

	return JIM_OK;
}


static int Jim_Command_flush_count(Jim_Interp *interp, int argc, Jim_Obj *const *args)
{
	Jim_SetResult(interp, Jim_NewIntObj(interp, jtag_get_flush_queue_count()));

	return JIM_OK;
}


static int handle_verify_ircapture_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 1)
	{
		if (strcmp(args[0], "enable") == 0)
		{
			jtag_verify_capture_ir = 1;
		}
		else if (strcmp(args[0], "disable") == 0)
		{
			jtag_verify_capture_ir = 0;
		} else
		{
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	} else if (argc != 0)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(cmd_ctx, "verify Capture-IR is %s", (jtag_verify_capture_ir) ? "enabled": "disabled");

	return ERROR_OK;
}

void jtag_set_verify(bool enable)
{
	jtag_verify = enable;
}

bool jtag_will_verify()
{
	return jtag_verify;
}

static int handle_verify_jtag_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (argc == 1)
	{
		if (strcmp(args[0], "enable") == 0)
			jtag_set_verify(true);
		else if (strcmp(args[0], "disable") == 0)
			jtag_set_verify(false);
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

	const char *status = jtag_will_verify() ? "enabled": "disabled";
	command_print(cmd_ctx, "verify jtag capture is %s", status);

	return ERROR_OK;
}


int jtag_power_dropout(int *dropout)
{
	return jtag->power_dropout(dropout);
}

int jtag_srst_asserted(int *srst_asserted)
{
	return jtag->srst_asserted(srst_asserted);
}

void jtag_tap_handle_event( jtag_tap_t * tap, enum jtag_tap_event e)
{
	jtag_tap_event_action_t * jteap;
	int done;

	jteap = tap->event_action;

	done = 0;
	while (jteap) {
		if (jteap->event == e) {
			done = 1;
			LOG_DEBUG( "JTAG tap: %s event: %d (%s) action: %s\n",
					tap->dotted_name,
					e,
					Jim_Nvp_value2name_simple(nvp_jtag_tap_event, e)->name,
					Jim_GetString(jteap->body, NULL) );
			if (Jim_EvalObj(interp, jteap->body) != JIM_OK) {
				Jim_PrintErrorMessage(interp);
			}
		}

		jteap = jteap->next;
	}

	if (!done) {
		LOG_DEBUG( "event %d %s - no action",
				e,
				Jim_Nvp_value2name_simple( nvp_jtag_tap_event, e)->name);
	}
}

static int handle_tms_sequence_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (argc == 1)
	{
		bool use_new_table;
		if (strcmp(args[0], "short") == 0)
			use_new_table = true;
		else if (strcmp(args[0], "long") == 0)
			use_new_table = false;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;

		tap_use_new_tms_table(use_new_table);
	}

	command_print(cmd_ctx, "tms sequence is  %s",
			tap_uses_new_tms_table() ? "short": "long");

	return ERROR_OK;
}

/**
 * Function jtag_add_statemove
 * moves from the current state to the goal \a state. This needs
 * to be handled according to the xsvf spec, see the XSTATE command
 * description.
 */
int jtag_add_statemove(tap_state_t goal_state)
{
	int retval = ERROR_OK;

	tap_state_t moves[8];
	tap_state_t cur_state = cmd_queue_cur_state;
	int i;
	int tms_bits;
	int	tms_count;

	LOG_DEBUG( "cur_state=%s goal_state=%s",
		tap_state_name(cur_state),
		tap_state_name(goal_state) );


	/*	From the XSVF spec, pertaining to XSTATE:

		For special states known as stable states (Test-Logic-Reset,
		Run-Test/Idle, Pause-DR, Pause- IR), an XSVF interpreter follows
		predefined TAP state paths when the starting state is a stable state and
		when the XSTATE specifies a new stable state (see the STATE command in
		the [Ref 5] for the TAP state paths between stable states). For
		non-stable states, XSTATE should specify a state that is only one TAP
		state transition distance from the current TAP state to avoid undefined
		TAP state paths. A sequence of multiple XSTATE commands can be issued to
		transition the TAP through a specific state path.
	*/

	if (goal_state==cur_state )
		;	/* nothing to do */

	else if( goal_state==TAP_RESET )
	{
		jtag_add_tlr();
	}

	else if( tap_is_state_stable(cur_state) && tap_is_state_stable(goal_state) )
	{
		/* 	note: unless tms_bits holds a path that agrees with [Ref 5] in above
			spec, then this code is not fully conformant to the xsvf spec.  This
			puts a burden on tap_get_tms_path() function from the xsvf spec.
			If in doubt, you should confirm that that burden is being met.
		*/

		tms_bits  = tap_get_tms_path(cur_state, goal_state);
		tms_count = tap_get_tms_path_len(cur_state, goal_state);

		assert( (unsigned) tms_count < DIM(moves) );

		for (i=0;   i<tms_count;   i++, tms_bits>>=1)
		{
			bool bit = tms_bits & 1;

			cur_state = tap_state_transition(cur_state, bit);
			moves[i] = cur_state;
		}

		jtag_add_pathmove(tms_count, moves);
	}

	/*	else state must be immediately reachable in one clock cycle, and does not
		need to be a stable state.
	*/
	else if( tap_state_transition(cur_state, true)  == goal_state
		||   tap_state_transition(cur_state, false) == goal_state )
	{
		/* move a single state */
		moves[0] = goal_state;
		jtag_add_pathmove( 1, moves );
	}

	else
	{
		retval = ERROR_FAIL;
	}

	return retval;
}

void jtag_set_nsrst_delay(unsigned delay)
{
	jtag_nsrst_delay = delay;
}
void jtag_set_ntrst_delay(unsigned delay)
{
	jtag_ntrst_delay = delay;
}


