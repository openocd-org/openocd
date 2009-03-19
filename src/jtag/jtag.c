/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 SoftPLC Corporation                                *
 * 	 http://softplc.com                                                    *
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

#include "replacements.h"

#include "jtag.h"

#include "command.h"
#include "log.h"

#include "stdlib.h"
#include "string.h"
#include <unistd.h>

/* note that this is not marked as static as it must be available from outside jtag.c for those
   that implement the jtag_xxx() minidriver layer
*/
int jtag_error=ERROR_OK;

typedef struct cmd_queue_page_s
{
	void *address;
	size_t used;
	struct cmd_queue_page_s *next;
} cmd_queue_page_t;

#define CMD_QUEUE_PAGE_SIZE (1024 * 1024)
static cmd_queue_page_t *cmd_queue_pages = NULL;

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

jtag_command_t *jtag_command_queue = NULL;
jtag_command_t **last_comand_pointer = &jtag_command_queue;
static jtag_tap_t *jtag_all_taps = NULL;

enum reset_types jtag_reset_config = RESET_NONE;
tap_state_t cmd_queue_end_state = TAP_RESET;
tap_state_t cmd_queue_cur_state = TAP_RESET;

int jtag_verify_capture_ir = 1;

/* how long the OpenOCD should wait before attempting JTAG communication after reset lines deasserted (in ms) */
int jtag_nsrst_delay = 0; /* default to no nSRST delay */
int jtag_ntrst_delay = 0; /* default to no nTRST delay */

/* maximum number of JTAG devices expected in the chain
 */
#define JTAG_MAX_CHAIN_SIZE 20

/* callbacks to inform high-level handlers about JTAG state changes */
jtag_event_callback_t *jtag_event_callbacks;

/* speed in kHz*/
static int speed_khz = 0;
/* flag if the kHz speed was defined */
static int hasKHz = 0;

/* jtag interfaces (parport, FTDI-USB, TI-USB, ...)
 */

#if BUILD_ECOSBOARD == 1
	extern jtag_interface_t zy1000_interface;
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

jtag_interface_t *jtag = NULL;

/* configuration */
jtag_interface_t *jtag_interface = NULL;
int jtag_speed = 0;

/* forward declarations */
void jtag_add_pathmove(int num_states, tap_state_t *path);
void jtag_add_runtest(int num_cycles, tap_state_t endstate);
void jtag_add_end_state(tap_state_t endstate);
void jtag_add_sleep(u32 us);
int jtag_execute_queue(void);
int tap_state_by_name(const char *name);

/* jtag commands */
int handle_interface_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_jtag_speed_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_jtag_khz_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_jtag_device_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_reset_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_jtag_nsrst_delay_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_jtag_ntrst_delay_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int handle_scan_chain_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int handle_endstate_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_jtag_reset_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_runtest_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_irscan_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int Jim_Command_drscan(Jim_Interp *interp, int argc, Jim_Obj *const *argv);

int handle_verify_ircapture_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

jtag_tap_t *jtag_AllTaps(void)
{
	return jtag_all_taps;
};

int jtag_NumTotalTaps(void)
{
	jtag_tap_t *t;
	int n;

	n = 0;
	t = jtag_AllTaps();
	while(t){
		n++;
		t = t->next_tap;
	}
	return n;
}

int jtag_NumEnabledTaps(void)
{
	jtag_tap_t *t;
	int n;

	n = 0;
	t = jtag_AllTaps();
	while(t){
		if( t->enabled ){
			n++;
		}
		t = t->next_tap;
	}
	return n;
}

jtag_tap_t *jtag_TapByString( const char *s )
{
	jtag_tap_t *t;
	char *cp;

	t = jtag_AllTaps();
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
			t = jtag_TapByAbsPosition(n);
		}
	}
	return t;
}

jtag_tap_t * jtag_TapByJimObj( Jim_Interp *interp, Jim_Obj *o )
{
	jtag_tap_t *t;
	const char *cp;

	cp = Jim_GetString( o, NULL );
	if(cp == NULL){
		cp = "(unknown)";
		t = NULL;
	}  else {
		t = jtag_TapByString( cp );
	}
	if( t == NULL ){
		Jim_SetResult_sprintf(interp,"Tap: %s is unknown", cp );
	}
	return t;
}

/* returns a pointer to the n-th device in the scan chain */
jtag_tap_t * jtag_TapByAbsPosition( int n )
{
	int orig_n;
	jtag_tap_t *t;

	orig_n = n;
	t = jtag_AllTaps();

	while( t && (n > 0)) {
		n--;
		t = t->next_tap;
	}
	return t;
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

/* returns a pointer to the pointer of the last command in queue
 * this may be a pointer to the root pointer (jtag_command_queue)
 * or to the next member of the last but one command
 */
jtag_command_t** jtag_get_last_command_p(void)
{
/*	jtag_command_t *cmd = jtag_command_queue;

	if (cmd)
		while (cmd->next)
			cmd = cmd->next;
	else
		return &jtag_command_queue;

	return &cmd->next;*/

	return last_comand_pointer;
}

void* cmd_queue_alloc(size_t size)
{
	cmd_queue_page_t **p_page = &cmd_queue_pages;
	int offset;
	u8 *t;

	/*
	 * WARNING:
	 *    We align/round the *SIZE* per below
	 *    so that all pointers returned by
	 *    this function are reasonably well
	 *    aligned.
	 *
	 * If we did not, then an "odd-length" request would cause the
	 * *next* allocation to be at an *odd* address, and because
	 * this function has the same type of api as malloc() - we
	 * must also return pointers that have the same type of
	 * alignment.
	 *
	 * What I do not/have is a reasonable portable means
	 * to align by...
	 *
	 * The solution here, is based on these suggestions.
	 * http://gcc.gnu.org/ml/gcc-help/2008-12/msg00041.html
	 *
	 */
	union worse_case_align {
		int i;
		long l;
		float f;
		void *v;
	};
#define ALIGN_SIZE  (sizeof(union worse_case_align))

	/* The alignment process. */
	size = (size + ALIGN_SIZE -1) & (~(ALIGN_SIZE-1));
	/* Done... */

	if (*p_page)
	{
		while ((*p_page)->next)
			p_page = &((*p_page)->next);
		if (CMD_QUEUE_PAGE_SIZE - (*p_page)->used < size)
			p_page = &((*p_page)->next);
	}

	if (!*p_page)
	{
		*p_page = malloc(sizeof(cmd_queue_page_t));
		(*p_page)->used = 0;
		(*p_page)->address = malloc(CMD_QUEUE_PAGE_SIZE);
		(*p_page)->next = NULL;
	}

	offset = (*p_page)->used;
	(*p_page)->used += size;

	t=(u8 *)((*p_page)->address);
	return t + offset;
}

void cmd_queue_free(void)
{
	cmd_queue_page_t *page = cmd_queue_pages;

	while (page)
	{
		cmd_queue_page_t *last = page;
		free(page->address);
		page = page->next;
		free(last);
	}

	cmd_queue_pages = NULL;
}

static void jtag_prelude1(void)
{
	if (jtag_trst == 1)
	{
		LOG_WARNING("JTAG command queued, while TRST is low (TAP in reset)");
		jtag_error=ERROR_JTAG_TRST_ASSERTED;
		return;
	}

	if (cmd_queue_end_state == TAP_RESET)
		jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
}

static void jtag_prelude(tap_state_t state)
{
	jtag_prelude1();

	if (state != -1)
		jtag_add_end_state(state);

	cmd_queue_cur_state = cmd_queue_end_state;
}

void jtag_add_ir_scan(int num_fields, scan_field_t *fields, tap_state_t state)
{
	int retval;

	jtag_prelude(state);

	retval=interface_jtag_add_ir_scan(num_fields, fields, cmd_queue_end_state);
	if (retval!=ERROR_OK)
		jtag_error=retval;
}

int MINIDRIVER(interface_jtag_add_ir_scan)(int num_fields, scan_field_t *fields, tap_state_t state)
{
	jtag_command_t **last_cmd;
	jtag_tap_t *tap;
	int j;
	int x;
	int nth_tap;
	int scan_size = 0;

	last_cmd = jtag_get_last_command_p();

	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	(*last_cmd)->next = NULL;
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->type = JTAG_SCAN;

	/* allocate memory for ir scan command */
	(*last_cmd)->cmd.scan = cmd_queue_alloc(sizeof(scan_command_t));
	(*last_cmd)->cmd.scan->ir_scan = 1;
	x = jtag_NumEnabledTaps();
	(*last_cmd)->cmd.scan->num_fields = x;	/* one field per device */
	(*last_cmd)->cmd.scan->fields = cmd_queue_alloc(x  * sizeof(scan_field_t));
	(*last_cmd)->cmd.scan->end_state = state;

	nth_tap = -1;
	tap = NULL;
	for(;;){
		int found = 0;

		/* do this here so it is not forgotten */
		tap = jtag_NextEnabledTap(tap);
		if( tap == NULL ){
			break;
		}
		nth_tap++;
		scan_size = tap->ir_length;
		(*last_cmd)->cmd.scan->fields[nth_tap].tap = tap;
		(*last_cmd)->cmd.scan->fields[nth_tap].num_bits = scan_size;
		(*last_cmd)->cmd.scan->fields[nth_tap].in_value = NULL;
		(*last_cmd)->cmd.scan->fields[nth_tap].in_handler = NULL;	/* disable verification by default */

		/* search the list */
		for (j = 0; j < num_fields; j++)
		{
			if (tap == fields[j].tap)
			{
				found = 1;
				(*last_cmd)->cmd.scan->fields[nth_tap].out_value = buf_cpy(fields[j].out_value, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
				(*last_cmd)->cmd.scan->fields[nth_tap].out_mask = buf_cpy(fields[j].out_mask, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);

				if (jtag_verify_capture_ir)
				{
					if (fields[j].in_handler==NULL)
					{
						jtag_set_check_value((*last_cmd)->cmd.scan->fields+nth_tap, tap->expected, tap->expected_mask, NULL);
					} else
					{
						(*last_cmd)->cmd.scan->fields[nth_tap].in_handler = fields[j].in_handler;
						(*last_cmd)->cmd.scan->fields[nth_tap].in_handler_priv = fields[j].in_handler_priv;
						(*last_cmd)->cmd.scan->fields[nth_tap].in_check_value = tap->expected;
						(*last_cmd)->cmd.scan->fields[nth_tap].in_check_mask = tap->expected_mask;
					}
				}

				tap->bypass = 0;
				break;
			}
		}

		if (!found)
		{
			/* if a tap isn't listed, set it to BYPASS */
			(*last_cmd)->cmd.scan->fields[nth_tap].out_value = buf_set_ones(cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
			(*last_cmd)->cmd.scan->fields[nth_tap].out_mask = NULL;
			tap->bypass = 1;
		}

		/* update device information */
		buf_cpy((*last_cmd)->cmd.scan->fields[nth_tap].out_value, tap->cur_instr, scan_size);
	}

	return ERROR_OK;
}

void jtag_add_plain_ir_scan(int num_fields, scan_field_t *fields, tap_state_t state)
{
	int retval;

	jtag_prelude(state);

	retval=interface_jtag_add_plain_ir_scan(num_fields, fields, cmd_queue_end_state);
	if (retval!=ERROR_OK)
		jtag_error=retval;
}

int MINIDRIVER(interface_jtag_add_plain_ir_scan)(int num_fields, scan_field_t *fields, tap_state_t state)
{
	int i;
	jtag_command_t **last_cmd;

	last_cmd = jtag_get_last_command_p();

	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	(*last_cmd)->next = NULL;
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->type = JTAG_SCAN;

	/* allocate memory for ir scan command */
	(*last_cmd)->cmd.scan = cmd_queue_alloc(sizeof(scan_command_t));
	(*last_cmd)->cmd.scan->ir_scan = 1;
	(*last_cmd)->cmd.scan->num_fields = num_fields;
	(*last_cmd)->cmd.scan->fields = cmd_queue_alloc(num_fields * sizeof(scan_field_t));
	(*last_cmd)->cmd.scan->end_state = state;

	for( i = 0 ; i < num_fields ; i++ ){
		int num_bits = fields[i].num_bits;
		int num_bytes = CEIL(fields[i].num_bits, 8);
		(*last_cmd)->cmd.scan->fields[i].tap = fields[i].tap;
		(*last_cmd)->cmd.scan->fields[i].num_bits = num_bits;
		(*last_cmd)->cmd.scan->fields[i].out_value = buf_cpy(fields[i].out_value, cmd_queue_alloc(num_bytes), num_bits);
		(*last_cmd)->cmd.scan->fields[i].out_mask = buf_cpy(fields[i].out_mask, cmd_queue_alloc(num_bytes), num_bits);
		(*last_cmd)->cmd.scan->fields[i].in_value = fields[i].in_value;
		(*last_cmd)->cmd.scan->fields[i].in_check_value = fields[i].in_check_value;
		(*last_cmd)->cmd.scan->fields[i].in_check_mask = fields[i].in_check_mask;
		(*last_cmd)->cmd.scan->fields[i].in_handler = NULL;
		(*last_cmd)->cmd.scan->fields[i].in_handler_priv = NULL;
	}
	return ERROR_OK;
}

void jtag_add_dr_scan(int num_fields, scan_field_t *fields, tap_state_t state)
{
	int retval;

	jtag_prelude(state);

	retval=interface_jtag_add_dr_scan(num_fields, fields, cmd_queue_end_state);
	if (retval!=ERROR_OK)
		jtag_error=retval;
}

int MINIDRIVER(interface_jtag_add_dr_scan)(int num_fields, scan_field_t *fields, tap_state_t state)
{
	int j;
	int nth_tap;
	int bypass_devices = 0;
	int field_count = 0;
	int scan_size;

	jtag_command_t **last_cmd = jtag_get_last_command_p();
	jtag_tap_t *tap;

	/* count devices in bypass */
	tap = NULL;
	bypass_devices = 0;
	for(;;){
		tap = jtag_NextEnabledTap(tap);
		if( tap == NULL ){
			break;
		}
		if( tap->bypass ){
			bypass_devices++;
		}
	}

	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->next = NULL;
	(*last_cmd)->type = JTAG_SCAN;

	/* allocate memory for dr scan command */
	(*last_cmd)->cmd.scan = cmd_queue_alloc(sizeof(scan_command_t));
	(*last_cmd)->cmd.scan->ir_scan = 0;
	(*last_cmd)->cmd.scan->num_fields = num_fields + bypass_devices;
	(*last_cmd)->cmd.scan->fields = cmd_queue_alloc((num_fields + bypass_devices) * sizeof(scan_field_t));
	(*last_cmd)->cmd.scan->end_state = state;

	tap = NULL;
	nth_tap = -1;
	for(;;){
		nth_tap++;
		tap = jtag_NextEnabledTap(tap);
		if( tap == NULL ){
			break;
		}
		int found = 0;
		(*last_cmd)->cmd.scan->fields[field_count].tap = tap;

		for (j = 0; j < num_fields; j++)
		{
			if (tap == fields[j].tap)
			{
				found = 1;
				scan_size = fields[j].num_bits;
				(*last_cmd)->cmd.scan->fields[field_count].num_bits = scan_size;
				(*last_cmd)->cmd.scan->fields[field_count].out_value = buf_cpy(fields[j].out_value, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
				(*last_cmd)->cmd.scan->fields[field_count].out_mask = buf_cpy(fields[j].out_mask, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
				(*last_cmd)->cmd.scan->fields[field_count].in_value = fields[j].in_value;
				(*last_cmd)->cmd.scan->fields[field_count].in_check_value = fields[j].in_check_value;
				(*last_cmd)->cmd.scan->fields[field_count].in_check_mask = fields[j].in_check_mask;
				(*last_cmd)->cmd.scan->fields[field_count].in_handler = fields[j].in_handler;
				(*last_cmd)->cmd.scan->fields[field_count++].in_handler_priv = fields[j].in_handler_priv;
			}
		}
		if (!found)
		{
#ifdef _DEBUG_JTAG_IO_
			/* if a device isn't listed, the BYPASS register should be selected */
			if (! tap->bypass)
			{
				LOG_ERROR("BUG: no scan data for a device not in BYPASS");
				exit(-1);
			}
#endif
			/* program the scan field to 1 bit length, and ignore it's value */
			(*last_cmd)->cmd.scan->fields[field_count].num_bits = 1;
			(*last_cmd)->cmd.scan->fields[field_count].out_value = NULL;
			(*last_cmd)->cmd.scan->fields[field_count].out_mask = NULL;
			(*last_cmd)->cmd.scan->fields[field_count].in_value = NULL;
			(*last_cmd)->cmd.scan->fields[field_count].in_check_value = NULL;
			(*last_cmd)->cmd.scan->fields[field_count].in_check_mask = NULL;
			(*last_cmd)->cmd.scan->fields[field_count].in_handler = NULL;
			(*last_cmd)->cmd.scan->fields[field_count++].in_handler_priv = NULL;
		}
		else
		{
#ifdef _DEBUG_JTAG_IO_
			/* if a device is listed, the BYPASS register must not be selected */
			if (tap->bypass)
			{
				LOG_ERROR("BUG: scan data for a device in BYPASS");
				exit(-1);
			}
#endif
		}
	}
	return ERROR_OK;
}

void MINIDRIVER(interface_jtag_add_dr_out)(jtag_tap_t *target_tap,
		int num_fields,
		const int *num_bits,
		const u32 *value,
		tap_state_t end_state)
{
	int nth_tap;
	int field_count = 0;
	int scan_size;
	int bypass_devices = 0;

	jtag_command_t **last_cmd = jtag_get_last_command_p();
	jtag_tap_t *tap;

	/* count devices in bypass */
	tap = NULL;
	bypass_devices = 0;
	for(;;){
		tap = jtag_NextEnabledTap(tap);
		if( tap == NULL ){
			break;
		}
		if( tap->bypass ){
			bypass_devices++;
		}
	}

	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->next = NULL;
	(*last_cmd)->type = JTAG_SCAN;

	/* allocate memory for dr scan command */
	(*last_cmd)->cmd.scan = cmd_queue_alloc(sizeof(scan_command_t));
	(*last_cmd)->cmd.scan->ir_scan = 0;
	(*last_cmd)->cmd.scan->num_fields = num_fields + bypass_devices;
	(*last_cmd)->cmd.scan->fields = cmd_queue_alloc((num_fields + bypass_devices) * sizeof(scan_field_t));
	(*last_cmd)->cmd.scan->end_state = end_state;

	tap = NULL;
	nth_tap = -1;
	for(;;){
		tap = jtag_NextEnabledTap(tap);
		if( tap == NULL ){
			break;
		}
		nth_tap++;
		(*last_cmd)->cmd.scan->fields[field_count].tap = tap;

		if (tap == target_tap)
		{
			int j;
#ifdef _DEBUG_JTAG_IO_
			/* if a device is listed, the BYPASS register must not be selected */
			if (tap->bypass)
			{
				LOG_ERROR("BUG: scan data for a device in BYPASS");
				exit(-1);
			}
#endif
			for (j = 0; j < num_fields; j++)
			{
				u8 out_value[4];
				scan_size = num_bits[j];
				buf_set_u32(out_value, 0, scan_size, value[j]);
				(*last_cmd)->cmd.scan->fields[field_count].num_bits = scan_size;
				(*last_cmd)->cmd.scan->fields[field_count].out_value = buf_cpy(out_value, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
				(*last_cmd)->cmd.scan->fields[field_count].out_mask = NULL;
				(*last_cmd)->cmd.scan->fields[field_count].in_value = NULL;
				(*last_cmd)->cmd.scan->fields[field_count].in_check_value = NULL;
				(*last_cmd)->cmd.scan->fields[field_count].in_check_mask = NULL;
				(*last_cmd)->cmd.scan->fields[field_count].in_handler = NULL;
				(*last_cmd)->cmd.scan->fields[field_count++].in_handler_priv = NULL;
			}
		} else
		{
#ifdef _DEBUG_JTAG_IO_
			/* if a device isn't listed, the BYPASS register should be selected */
			if (! tap->bypass)
			{
				LOG_ERROR("BUG: no scan data for a device not in BYPASS");
				exit(-1);
			}
#endif
			/* program the scan field to 1 bit length, and ignore it's value */
			(*last_cmd)->cmd.scan->fields[field_count].num_bits = 1;
			(*last_cmd)->cmd.scan->fields[field_count].out_value = NULL;
			(*last_cmd)->cmd.scan->fields[field_count].out_mask = NULL;
			(*last_cmd)->cmd.scan->fields[field_count].in_value = NULL;
			(*last_cmd)->cmd.scan->fields[field_count].in_check_value = NULL;
			(*last_cmd)->cmd.scan->fields[field_count].in_check_mask = NULL;
			(*last_cmd)->cmd.scan->fields[field_count].in_handler = NULL;
			(*last_cmd)->cmd.scan->fields[field_count++].in_handler_priv = NULL;
		}
	}
}

void jtag_add_plain_dr_scan(int num_fields, scan_field_t *fields, tap_state_t state)
{
	int retval;

	jtag_prelude(state);

	retval=interface_jtag_add_plain_dr_scan(num_fields, fields, cmd_queue_end_state);
	if (retval!=ERROR_OK)
		jtag_error=retval;
}

int MINIDRIVER(interface_jtag_add_plain_dr_scan)(int num_fields, scan_field_t *fields, tap_state_t state)
{
	int i;
	jtag_command_t **last_cmd = jtag_get_last_command_p();

	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->next = NULL;
	(*last_cmd)->type = JTAG_SCAN;

	/* allocate memory for scan command */
	(*last_cmd)->cmd.scan = cmd_queue_alloc(sizeof(scan_command_t));
	(*last_cmd)->cmd.scan->ir_scan = 0;
	(*last_cmd)->cmd.scan->num_fields = num_fields;
	(*last_cmd)->cmd.scan->fields = cmd_queue_alloc(num_fields * sizeof(scan_field_t));
	(*last_cmd)->cmd.scan->end_state = state;

	for (i = 0; i < num_fields; i++)
	{
		int num_bits = fields[i].num_bits;
		int num_bytes = CEIL(fields[i].num_bits, 8);
		(*last_cmd)->cmd.scan->fields[i].tap = fields[i].tap;
		(*last_cmd)->cmd.scan->fields[i].num_bits = num_bits;
		(*last_cmd)->cmd.scan->fields[i].out_value = buf_cpy(fields[i].out_value, cmd_queue_alloc(num_bytes), num_bits);
		(*last_cmd)->cmd.scan->fields[i].out_mask = buf_cpy(fields[i].out_mask, cmd_queue_alloc(num_bytes), num_bits);
		(*last_cmd)->cmd.scan->fields[i].in_value = fields[i].in_value;
		(*last_cmd)->cmd.scan->fields[i].in_check_value = fields[i].in_check_value;
		(*last_cmd)->cmd.scan->fields[i].in_check_mask = fields[i].in_check_mask;
		(*last_cmd)->cmd.scan->fields[i].in_handler = fields[i].in_handler;
		(*last_cmd)->cmd.scan->fields[i].in_handler_priv = fields[i].in_handler_priv;
	}

	return ERROR_OK;
}

void jtag_add_tlr(void)
{
	jtag_prelude(TAP_RESET);

	int retval;
	retval=interface_jtag_add_tlr();
	if (retval!=ERROR_OK)
		jtag_error=retval;
}

int MINIDRIVER(interface_jtag_add_tlr)(void)
{
	tap_state_t state = TAP_RESET;
	jtag_command_t **last_cmd = jtag_get_last_command_p();

	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->next = NULL;
	(*last_cmd)->type = JTAG_STATEMOVE;

	(*last_cmd)->cmd.statemove = cmd_queue_alloc(sizeof(statemove_command_t));
	(*last_cmd)->cmd.statemove->end_state = state;

	return ERROR_OK;
}

void jtag_add_pathmove(int num_states, tap_state_t *path)
{
	tap_state_t cur_state=cmd_queue_cur_state;
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

	jtag_prelude1();

	retval=interface_jtag_add_pathmove(num_states, path);
	cmd_queue_cur_state = path[num_states - 1];
	if (retval!=ERROR_OK)
		jtag_error=retval;
}

int MINIDRIVER(interface_jtag_add_pathmove)(int num_states, tap_state_t *path)
{
	jtag_command_t **last_cmd = jtag_get_last_command_p();
	int i;

	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->next = NULL;
	(*last_cmd)->type = JTAG_PATHMOVE;

	(*last_cmd)->cmd.pathmove = cmd_queue_alloc(sizeof(pathmove_command_t));
	(*last_cmd)->cmd.pathmove->num_states = num_states;
	(*last_cmd)->cmd.pathmove->path = cmd_queue_alloc(sizeof(tap_state_t) * num_states);

	for (i = 0; i < num_states; i++)
		(*last_cmd)->cmd.pathmove->path[i] = path[i];

	return ERROR_OK;
}

int MINIDRIVER(interface_jtag_add_runtest)(int num_cycles, tap_state_t state)
{
	jtag_command_t **last_cmd = jtag_get_last_command_p();

	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	(*last_cmd)->next = NULL;
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->type = JTAG_RUNTEST;

	(*last_cmd)->cmd.runtest = cmd_queue_alloc(sizeof(runtest_command_t));
	(*last_cmd)->cmd.runtest->num_cycles = num_cycles;
	(*last_cmd)->cmd.runtest->end_state = state;

	return ERROR_OK;
}

void jtag_add_runtest(int num_cycles, tap_state_t state)
{
	int retval;

	jtag_prelude(state);

	/* executed by sw or hw fifo */
	retval=interface_jtag_add_runtest(num_cycles, cmd_queue_end_state);
	if (retval!=ERROR_OK)
		jtag_error=retval;
}


int MINIDRIVER(interface_jtag_add_clocks)( int num_cycles )
{
	jtag_command_t **last_cmd = jtag_get_last_command_p();

	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	(*last_cmd)->next = NULL;
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->type = JTAG_STABLECLOCKS;

	(*last_cmd)->cmd.stableclocks = cmd_queue_alloc(sizeof(stableclocks_command_t));
	(*last_cmd)->cmd.stableclocks->num_cycles = num_cycles;
	return ERROR_OK;
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
		jtag_prelude1();

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
		jtag_add_end_state(TAP_RESET);
		jtag_add_tlr();
		jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
		return;
	}

	if (jtag_trst)
	{
		/* we just asserted nTRST, so we're now in Test-Logic-Reset,
		 * and inform possible listeners about this
		 */
		LOG_DEBUG("TRST line asserted");
		cmd_queue_cur_state = TAP_RESET;
		jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
	}
	else
	{
		if (jtag_ntrst_delay)
			jtag_add_sleep(jtag_ntrst_delay * 1000);
	}
}

int MINIDRIVER(interface_jtag_add_reset)(int req_trst, int req_srst)
{
	jtag_command_t **last_cmd = jtag_get_last_command_p();

	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	(*last_cmd)->next = NULL;
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->type = JTAG_RESET;

	(*last_cmd)->cmd.reset = cmd_queue_alloc(sizeof(reset_command_t));
	(*last_cmd)->cmd.reset->trst = req_trst;
	(*last_cmd)->cmd.reset->srst = req_srst;

	return ERROR_OK;
}

void jtag_add_end_state(tap_state_t state)
{
	cmd_queue_end_state = state;
	if ((cmd_queue_end_state == TAP_DRSHIFT)||(cmd_queue_end_state == TAP_IRSHIFT))
	{
		LOG_ERROR("BUG: TAP_DRSHIFT/IRSHIFT can't be end state. Calling code should use a larger scan field");
	}
}

int MINIDRIVER(interface_jtag_add_sleep)(u32 us)
{
	jtag_command_t **last_cmd = jtag_get_last_command_p();

	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	(*last_cmd)->next = NULL;
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->type = JTAG_SLEEP;

	(*last_cmd)->cmd.sleep = cmd_queue_alloc(sizeof(sleep_command_t));
	(*last_cmd)->cmd.sleep->us = us;

	return ERROR_OK;
}

void jtag_add_sleep(u32 us)
{
	keep_alive(); /* we might be running on a very slow JTAG clk */
	int retval=interface_jtag_add_sleep(us);
	if (retval!=ERROR_OK)
		jtag_error=retval;
	return;
}

int jtag_scan_size(scan_command_t *cmd)
{
	int bit_count = 0;
	int i;

	/* count bits in scan command */
	for (i = 0; i < cmd->num_fields; i++)
	{
		bit_count += cmd->fields[i].num_bits;
	}

	return bit_count;
}

int jtag_build_buffer(scan_command_t *cmd, u8 **buffer)
{
	int bit_count = 0;
	int i;

	bit_count = jtag_scan_size(cmd);
	*buffer = malloc(CEIL(bit_count, 8));

	bit_count = 0;

#ifdef _DEBUG_JTAG_IO_
	LOG_DEBUG("num_fields: %i",cmd->num_fields);
#endif

	for (i = 0; i < cmd->num_fields; i++)
	{
		if (cmd->fields[i].out_value)
		{
#ifdef _DEBUG_JTAG_IO_
			char* char_buf = buf_to_str(cmd->fields[i].out_value, (cmd->fields[i].num_bits > DEBUG_JTAG_IOZ) ? DEBUG_JTAG_IOZ : cmd->fields[i].num_bits, 16);
#endif
			buf_set_buf(cmd->fields[i].out_value, 0, *buffer, bit_count, cmd->fields[i].num_bits);
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG("fields[%i].out_value[%i]: 0x%s", i, cmd->fields[i].num_bits, char_buf);
			free(char_buf);
#endif
		}

		bit_count += cmd->fields[i].num_bits;
	}

	return bit_count;
}

int jtag_read_buffer(u8 *buffer, scan_command_t *cmd)
{
	int i;
	int bit_count = 0;
	int retval;

	/* we return ERROR_OK, unless a check fails, or a handler reports a problem */
	retval = ERROR_OK;

	for (i = 0; i < cmd->num_fields; i++)
	{
		/* if neither in_value nor in_handler
		 * are specified we don't have to examine this field
		 */
		if (cmd->fields[i].in_value || cmd->fields[i].in_handler)
		{
			int num_bits = cmd->fields[i].num_bits;
			u8 *captured = buf_set_buf(buffer, bit_count, malloc(CEIL(num_bits, 8)), 0, num_bits);

#ifdef _DEBUG_JTAG_IO_
			char *char_buf = buf_to_str(captured, (num_bits > DEBUG_JTAG_IOZ) ? DEBUG_JTAG_IOZ : num_bits, 16);
			LOG_DEBUG("fields[%i].in_value[%i]: 0x%s", i, num_bits, char_buf);
			free(char_buf);
#endif

			if (cmd->fields[i].in_value)
			{
				buf_cpy(captured, cmd->fields[i].in_value, num_bits);

				if (cmd->fields[i].in_handler)
				{
					if (cmd->fields[i].in_handler(cmd->fields[i].in_value, cmd->fields[i].in_handler_priv, cmd->fields+i) != ERROR_OK)
					{
						LOG_WARNING("in_handler: with \"in_value\", mismatch in %s", cmd->ir_scan ? "SIR" : "SDR" );
						retval = ERROR_JTAG_QUEUE_FAILED;
					}
				}
			}

			/* no in_value specified, but a handler takes care of the scanned data */
			if (cmd->fields[i].in_handler && (!cmd->fields[i].in_value))
			{
				if (cmd->fields[i].in_handler(captured, cmd->fields[i].in_handler_priv, cmd->fields+i) != ERROR_OK)
				{
					/* We're going to call the error:handler later, but if the in_handler
					 * reported an error we report this failure upstream
					 */
					LOG_WARNING("in_handler: w/o \"in_value\", mismatch in %s",  cmd->ir_scan ? "SIR" : "SDR" );
					retval = ERROR_JTAG_QUEUE_FAILED;
				}
			}

			free(captured);
		}
		bit_count += cmd->fields[i].num_bits;
	}

	return retval;
}

static const char *jtag_tap_name(jtag_tap_t *tap)
{
	return (tap == NULL) ? "(unknown)" : tap->dotted_name;
}

int jtag_check_value(u8 *captured, void *priv, scan_field_t *field)
{
	int retval = ERROR_OK;
	int num_bits = field->num_bits;

	int compare_failed = 0;

	if (field->in_check_mask)
		compare_failed = buf_cmp_mask(captured, field->in_check_value, field->in_check_mask, num_bits);
	else
		compare_failed = buf_cmp(captured, field->in_check_value, num_bits);

	if (compare_failed){
		/* An error handler could have caught the failing check
		 * only report a problem when there wasn't a handler, or if the handler
		 * acknowledged the error
		 */
		LOG_WARNING("TAP %s:",
					jtag_tap_name(field->tap));
		if (compare_failed)
		{
			char *captured_char = buf_to_str(captured, (num_bits > DEBUG_JTAG_IOZ) ? DEBUG_JTAG_IOZ : num_bits, 16);
			char *in_check_value_char = buf_to_str(field->in_check_value, (num_bits > DEBUG_JTAG_IOZ) ? DEBUG_JTAG_IOZ : num_bits, 16);

			if (field->in_check_mask)
			{
				char *in_check_mask_char;
				in_check_mask_char = buf_to_str(field->in_check_mask, (num_bits > DEBUG_JTAG_IOZ) ? DEBUG_JTAG_IOZ : num_bits, 16);
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

/*
  set up checking of this field using the in_handler. The values passed in must be valid until
  after jtag_execute() has completed.
 */
void jtag_set_check_value(scan_field_t *field, u8 *value, u8 *mask, error_handler_t *in_error_handler)
{
	if (value)
		field->in_handler = jtag_check_value;
	else
		field->in_handler = NULL;	/* No check, e.g. embeddedice uses value==NULL to indicate no check */
	field->in_handler_priv = NULL;
	field->in_check_value = value;
	field->in_check_mask = mask;
}

enum scan_type jtag_scan_type(scan_command_t *cmd)
{
	int i;
	int type = 0;

	for (i = 0; i < cmd->num_fields; i++)
	{
		if (cmd->fields[i].in_value || cmd->fields[i].in_handler)
			type |= SCAN_IN;
		if (cmd->fields[i].out_value)
			type |= SCAN_OUT;
	}

	return type;
}

int MINIDRIVER(interface_jtag_execute_queue)(void)
{
	int retval;

	if (jtag==NULL)
	{
		LOG_ERROR("No JTAG interface configured yet. Issue 'init' command in startup scripts before communicating with targets.");
		return ERROR_FAIL;
	}

	retval = jtag->execute_queue();

	cmd_queue_free();

	jtag_command_queue = NULL;
	last_comand_pointer = &jtag_command_queue;

	return retval;
}

int jtag_execute_queue(void)
{
	int retval=interface_jtag_execute_queue();
	if (retval==ERROR_OK)
	{
		retval=jtag_error;
	}
	jtag_error=ERROR_OK;
	return retval;
}

int jtag_reset_callback(enum jtag_event event, void *priv)
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

/* Try to examine chain layout according to IEEE 1149.1 Â§12
 */
int jtag_examine_chain(void)
{
	jtag_tap_t *tap;
	scan_field_t field;
	u8 idcode_buffer[JTAG_MAX_CHAIN_SIZE * 4];
	int i;
	int bit_count;
	int device_count = 0;
	u8 zero_check = 0x0;
	u8 one_check = 0xff;

	field.tap = NULL;
	field.num_bits = sizeof(idcode_buffer) * 8;
	field.out_value = idcode_buffer;
	field.out_mask = NULL;
	field.in_value = idcode_buffer;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;

	for (i = 0; i < JTAG_MAX_CHAIN_SIZE; i++)
	{
		buf_set_u32(idcode_buffer, i * 32, 32, 0x000000FF);
	}

	jtag_add_plain_dr_scan(1, &field, TAP_RESET);
	jtag_execute_queue();

	for (i = 0; i < JTAG_MAX_CHAIN_SIZE * 4; i++)
	{
		zero_check |= idcode_buffer[i];
		one_check &= idcode_buffer[i];
	}

	/* if there wasn't a single non-zero bit or if all bits were one, the scan isn't valid */
	if ((zero_check == 0x00) || (one_check == 0xff))
	{
		LOG_ERROR("JTAG communication failure, check connection, JTAG interface, target power etc.");
		return ERROR_JTAG_INIT_FAILED;
	}

	/* point at the 1st tap */
	tap = jtag_NextEnabledTap(NULL);
	if( tap == NULL ){
		LOG_ERROR("JTAG: No taps enabled?");
		return ERROR_JTAG_INIT_FAILED;
	}

	for (bit_count = 0; bit_count < (JTAG_MAX_CHAIN_SIZE * 32) - 31;)
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
			u32 manufacturer;
			u32 part;
			u32 version;

			if (idcode == 0x000000FF)
			{
				int unexpected=0;
				/* End of chain (invalid manufacturer ID)
				 *
				 * The JTAG examine is the very first thing that happens
				 *
				 * A single JTAG device requires only 64 bits to be read back correctly.
				 *
				 * The code below adds a check that the rest of the data scanned (640 bits)
				 * are all as expected. This helps diagnose/catch problems with the JTAG chain
				 *
				 * earlier and gives more helpful/explicit error messages.
				 */
				for (bit_count += 32; bit_count < (JTAG_MAX_CHAIN_SIZE * 32) - 31;bit_count += 32)
				{
					idcode = buf_get_u32(idcode_buffer, bit_count, 32);
					if (unexpected||(idcode != 0x000000FF))
					{
						LOG_WARNING("Unexpected idcode after end of chain! %d 0x%08x", bit_count, idcode);
						unexpected = 1;
					}
				}

				break;
			}

#define EXTRACT_MFG(X)  (((X) & 0xffe) >> 1)
			manufacturer = EXTRACT_MFG(idcode);
#define EXTRACT_PART(X) (((X) & 0xffff000) >> 12)
			part = EXTRACT_PART(idcode);
#define EXTRACT_VER(X)  (((X) & 0xf0000000) >> 28)
			version = EXTRACT_VER(idcode);

			LOG_INFO("JTAG tap: %s tap/device found: 0x%8.8x (Manufacturer: 0x%3.3x, Part: 0x%4.4x, Version: 0x%1.1x)",
					 ((tap != NULL) ? (tap->dotted_name) : "(not-named)"),
				idcode, manufacturer, part, version);

			bit_count += 32;
		}
		if (tap)
		{
			tap->idcode = idcode;

			if (tap->expected_ids_cnt > 0) {
				/* Loop over the expected identification codes and test for a match */
				u8 ii;
				for (ii = 0; ii < tap->expected_ids_cnt; ii++) {
					if( tap->idcode == tap->expected_ids[ii] ){
						break;
					}
				}

				/* If none of the expected ids matched, log an error */
				if (ii == tap->expected_ids_cnt) {
					LOG_ERROR("JTAG tap: %s             got: 0x%08x (mfg: 0x%3.3x, part: 0x%4.4x, ver: 0x%1.1x)",
							  tap->dotted_name,
							  idcode,
							  EXTRACT_MFG( tap->idcode ),
							  EXTRACT_PART( tap->idcode ),
							  EXTRACT_VER( tap->idcode ) );
					for (ii = 0; ii < tap->expected_ids_cnt; ii++) {
						LOG_ERROR("JTAG tap: %s expected %hhu of %hhu: 0x%08x (mfg: 0x%3.3x, part: 0x%4.4x, ver: 0x%1.1x)",
								  tap->dotted_name,
								  ii + 1,
								  tap->expected_ids_cnt,
								  tap->expected_ids[ii],
								  EXTRACT_MFG( tap->expected_ids[ii] ),
								  EXTRACT_PART( tap->expected_ids[ii] ),
								  EXTRACT_VER( tap->expected_ids[ii] ) );
					}

					return ERROR_JTAG_INIT_FAILED;
				} else {
					LOG_INFO("JTAG Tap/device matched");
				}
			} else {
#if 0
				LOG_INFO("JTAG TAP ID: 0x%08x - Unknown - please report (A) chipname and (B) idcode to the openocd project",
						 tap->idcode);
#endif
			}
			tap = jtag_NextEnabledTap(tap);
		}
		device_count++;
	}

	/* see if number of discovered devices matches configuration */
	if (device_count != jtag_NumEnabledTaps())
	{
		LOG_ERROR("number of discovered devices in JTAG chain (%i) doesn't match (enabled) configuration (%i), total taps: %d",
				  device_count, jtag_NumEnabledTaps(), jtag_NumTotalTaps());
		LOG_ERROR("check the config file and ensure proper JTAG communication (connections, speed, ...)");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

int jtag_validate_chain(void)
{
	jtag_tap_t *tap;
	int total_ir_length = 0;
	u8 *ir_test = NULL;
	scan_field_t field;
	int chain_pos = 0;

	tap = NULL;
	total_ir_length = 0;
	for(;;){
		tap = jtag_NextEnabledTap(tap);
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
	field.out_mask = NULL;
	field.in_value = ir_test;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;

	jtag_add_plain_ir_scan(1, &field, TAP_RESET);
	jtag_execute_queue();

	tap = NULL;
	chain_pos = 0;
	int val;
	for(;;){
		tap = jtag_NextEnabledTap(tap);
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
						if (jteap->event == n->value) {
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

static int jim_newtap_cmd( Jim_GetOptInfo *goi )
{
	jtag_tap_t *pTap;
	jtag_tap_t **ppTap;
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

	/* Did we get all the options? */
	if( reqbits ){
		// no
		Jim_SetResult_sprintf( goi->interp,
							   "newtap: %s missing required parameters",
							   pTap->dotted_name);
		/* TODO: Tell user what is missing :-( */
		/* no memory leaks pelase */
		free(((void *)(pTap->expected_ids)));
		free(((void *)(pTap->chip)));
		free(((void *)(pTap->tapname)));
		free(((void *)(pTap->dotted_name)));
		free(((void *)(pTap)));
		return JIM_ERR;
	}

	pTap->expected      = malloc( pTap->ir_length );
	pTap->expected_mask = malloc( pTap->ir_length );
	pTap->cur_instr     = malloc( pTap->ir_length );

	buf_set_u32( pTap->expected,
				 0,
				 pTap->ir_length,
				 pTap->ir_capture_value );
	buf_set_u32( pTap->expected_mask,
				 0,
				 pTap->ir_length,
				 pTap->ir_capture_mask );
	buf_set_ones( pTap->cur_instr,
				  pTap->ir_length );

	pTap->bypass = 1;

	jtag_register_event_callback(jtag_reset_callback, pTap );

	ppTap = &(jtag_all_taps);
	while( (*ppTap) != NULL ){
		ppTap = &((*ppTap)->next_tap);
	}
	*ppTap = pTap;
	{
		static int n_taps = 0;
		pTap->abs_chain_position = n_taps++;
	}
	LOG_DEBUG( "Created Tap: %s @ abs position %d, irlen %d, capture: 0x%x mask: 0x%x",
				(*ppTap)->dotted_name,
				(*ppTap)->abs_chain_position,
				(*ppTap)->ir_length,
				(*ppTap)->ir_capture_value,
				(*ppTap)->ir_capture_mask );

	return ERROR_OK;
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
			t = jtag_TapByJimObj( goi.interp, goi.argv[0] );
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
			t = jtag_TapByJimObj( goi.interp, o );
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
			t = jtag_TapByJimObj( goi.interp, o );
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
		COMMAND_ANY, "set jtag speed (if supported)");
	register_command(cmd_ctx, NULL, "jtag_khz", handle_jtag_khz_command,
		COMMAND_ANY, "same as jtag_speed, except it takes maximum khz as arguments. 0 KHz = RTCK.");
	register_command(cmd_ctx, NULL, "jtag_device", handle_jtag_device_command,
		COMMAND_CONFIG, "jtag_device <ir_length> <ir_expected> <ir_mask>");
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

	register_command(cmd_ctx, NULL, "verify_ircapture", handle_verify_ircapture_command,
		COMMAND_ANY, "verify value captured during Capture-IR <enable|disable>");
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
		jtag_interface->khz(speed_khz, &jtag_speed);
		hasKHz = 0;
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

	tap = jtag_NextEnabledTap(NULL);
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

int handle_interface_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
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

int handle_jtag_device_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
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
	sprintf( buf, "chip%d", jtag_NumTotalTaps() );
	newargs[2] = Jim_NewStringObj( interp, buf, -1 );
	sprintf( buf, "tap%d", jtag_NumTotalTaps() );
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

int handle_scan_chain_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	jtag_tap_t *tap;

	tap = jtag_all_taps;
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

int handle_reset_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (argc >= 1)
	{
		if (strcmp(args[0], "none") == 0)
			jtag_reset_config = RESET_NONE;
		else if (strcmp(args[0], "trst_only") == 0)
			jtag_reset_config = RESET_HAS_TRST;
		else if (strcmp(args[0], "srst_only") == 0)
			jtag_reset_config = RESET_HAS_SRST;
		else if (strcmp(args[0], "trst_and_srst") == 0)
			jtag_reset_config = RESET_TRST_AND_SRST;
		else
		{
			LOG_ERROR("(1) invalid reset_config argument (%s), defaulting to none", args[0]);
			jtag_reset_config = RESET_NONE;
			return ERROR_INVALID_ARGUMENTS;
		}
	}

	if (argc >= 2)
	{
		if (strcmp(args[1], "separate") == 0)
		{
			/* seperate reset lines - default */
		} else
		{
			if (strcmp(args[1], "srst_pulls_trst") == 0)
				jtag_reset_config |= RESET_SRST_PULLS_TRST;
			else if (strcmp(args[1], "trst_pulls_srst") == 0)
				jtag_reset_config |= RESET_TRST_PULLS_SRST;
			else if (strcmp(args[1], "combined") == 0)
				jtag_reset_config |= RESET_SRST_PULLS_TRST | RESET_TRST_PULLS_SRST;
			else
			{
				LOG_ERROR("(2) invalid reset_config argument (%s), defaulting to none", args[1]);
				jtag_reset_config = RESET_NONE;
				return ERROR_INVALID_ARGUMENTS;
			}
		}
	}

	if (argc >= 3)
	{
		if (strcmp(args[2], "trst_open_drain") == 0)
			jtag_reset_config |= RESET_TRST_OPEN_DRAIN;
		else if (strcmp(args[2], "trst_push_pull") == 0)
			jtag_reset_config &= ~RESET_TRST_OPEN_DRAIN;
		else
		{
			LOG_ERROR("(3) invalid reset_config argument (%s) defaulting to none", args[2] );
			jtag_reset_config = RESET_NONE;
			return ERROR_INVALID_ARGUMENTS;
		}
	}

	if (argc >= 4)
	{
		if (strcmp(args[3], "srst_push_pull") == 0)
			jtag_reset_config |= RESET_SRST_PUSH_PULL;
		else if (strcmp(args[3], "srst_open_drain") == 0)
			jtag_reset_config &= ~RESET_SRST_PUSH_PULL;
		else
		{
			LOG_ERROR("(4) invalid reset_config argument (%s), defaulting to none", args[3]);
			jtag_reset_config = RESET_NONE;
			return ERROR_INVALID_ARGUMENTS;
		}
	}

	return ERROR_OK;
}

int handle_jtag_nsrst_delay_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc < 1)
	{
		LOG_ERROR("jtag_nsrst_delay <ms> command takes one required argument");
		exit(-1);
	}
	else
	{
		jtag_nsrst_delay = strtoul(args[0], NULL, 0);
	}

	return ERROR_OK;
}

int handle_jtag_ntrst_delay_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc < 1)
	{
		LOG_ERROR("jtag_ntrst_delay <ms> command takes one required argument");
		exit(-1);
	}
	else
	{
		jtag_ntrst_delay = strtoul(args[0], NULL, 0);
	}

	return ERROR_OK;
}

int handle_jtag_speed_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
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

int handle_jtag_khz_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval=ERROR_OK;
	LOG_DEBUG("handle jtag khz");

	if(argc == 1)
	{
		speed_khz = strtoul(args[0], NULL, 0);
		if (jtag != NULL)
		{
			int cur_speed = 0;
			LOG_DEBUG("have interface set up");
			int speed_div1;
			if ((retval=jtag->khz(speed_khz, &speed_div1))!=ERROR_OK)
			{
				speed_khz = 0;
				return retval;
			}

			cur_speed = jtag_speed = speed_div1;

			retval=jtag->speed(cur_speed);
		} else
		{
			hasKHz = 1;
		}
	} else if (argc==0)
	{
	} else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (jtag!=NULL)
	{
		if ((retval=jtag->speed_div(jtag_speed, &speed_khz))!=ERROR_OK)
			return retval;
	}

	if (speed_khz==0)
	{
		command_print(cmd_ctx, "RCLK - adaptive");
	} else
	{
		command_print(cmd_ctx, "%d kHz", speed_khz);
	}
	return retval;

}

int handle_endstate_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int state;

	if (argc < 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	else
	{
		state = tap_state_by_name( args[0] );
		if( state < 0 ){
			command_print( cmd_ctx, "Invalid state name: %s\n", args[0] );
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		jtag_add_end_state( (tap_state_t)(state));
		jtag_execute_queue();
	}
	command_print(cmd_ctx, "current endstate: %s", tap_state_name(cmd_queue_end_state));

	return ERROR_OK;
}

int handle_jtag_reset_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
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

int handle_runtest_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc < 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	jtag_add_runtest(strtol(args[0], NULL, 0), -1);
	jtag_execute_queue();

	return ERROR_OK;

}

int handle_irscan_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int i;
	scan_field_t *fields;
	jtag_tap_t *tap;
	int endstate;

	if ((argc < 2) || (argc % 2))
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* optional "-endstate" */
	/*          "statename" */
	/* at the end of the arguments. */
	/* assume none. */
	endstate = -1;
	if( argc >= 4 ){
		/* have at least one pair of numbers. */
		/* is last pair the magic text? */
		if( 0 == strcmp( "-endstate", args[ argc - 2 ] ) ){
			const char *cpA;
			const char *cpS;
			cpA = args[ argc-1 ];
			for( endstate = 0 ; endstate < 16 ; endstate++ ){
				cpS = tap_state_name( endstate );
				if( 0 == strcmp( cpA, cpS ) ){
					break;
				}
			}
			if( endstate >= 16 ){
				return ERROR_COMMAND_SYNTAX_ERROR;
			} else {
				/* found - remove the last 2 args */
				argc -= 2;
			}
		}
	}

	fields = malloc(sizeof(scan_field_t) * argc / 2);

	for (i = 0; i < argc / 2; i++)
	{
		tap = jtag_TapByString( args[i*2] );
		if (tap==NULL)
		{
			command_print( cmd_ctx, "Tap: %s unknown", args[i*2] );
			return ERROR_FAIL;
		}
		int field_size = tap->ir_length;
		fields[i].tap = tap;
		fields[i].out_value = malloc(CEIL(field_size, 8));
		buf_set_u32(fields[i].out_value, 0, field_size, strtoul(args[i*2+1], NULL, 0));
		fields[i].out_mask = NULL;
		fields[i].in_value = NULL;
		fields[i].in_check_mask = NULL;
		fields[i].in_handler = NULL;
		fields[i].in_handler_priv = NULL;
	}

	jtag_add_ir_scan(argc / 2, fields, -1);
	/* did we have an endstate? */
	if( endstate >= 0 ){
		jtag_add_end_state(endstate);
	}
	jtag_execute_queue();

	for (i = 0; i < argc / 2; i++)
		free(fields[i].out_value);

	free (fields);

	return ERROR_OK;
}

int Jim_Command_drscan(Jim_Interp *interp, int argc, Jim_Obj *const *args)
{
	int retval;
	scan_field_t *fields;
	int num_fields;
	int field_count = 0;
	int i, e;
	jtag_tap_t *tap;
	int endstate;

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

	/* assume no endstate */
	endstate = -1;
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

		/* it could be: "-endstate FOO" */

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

	tap = jtag_TapByJimObj( interp, args[1] );
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
		fields[field_count].out_mask = NULL;
		fields[field_count].in_value = fields[field_count].out_value;
		fields[field_count].in_check_mask = NULL;
		fields[field_count].in_check_value = NULL;
		fields[field_count].in_handler = NULL;
		fields[field_count++].in_handler_priv = NULL;
	}

	jtag_add_dr_scan(num_fields, fields, -1);
	/* did we get an end state? */
	if( endstate >= 0 ){
		jtag_add_end_state( (tap_state_t)endstate );
	}
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

int handle_verify_ircapture_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
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

/*-----<Cable Helper API>---------------------------------------*/

/*  these Cable Helper API functions are all documented in the jtag.h header file,
	using a Doxygen format.  And since Doxygen's configuration file "Doxyfile",
	is setup to prefer its docs in the header file, no documentation is here, for
	if it were, it would have to be doubly maintained.
*/

/**
 * @see tap_set_state() and tap_get_state() accessors.
 * Actual name is not important since accessors hide it.
 */
static tap_state_t state_follower = TAP_RESET;

void tap_set_state_impl( tap_state_t new_state )
{
	/* this is the state we think the TAPs are in now, was cur_state */
	state_follower = new_state;
}

tap_state_t tap_get_state()
{
	return state_follower;
}

/**
 * @see tap_set_end_state() and tap_get_end_state() accessors.
 * Actual name is not important because accessors hide it.
 */
static tap_state_t end_state_follower = TAP_RESET;

void tap_set_end_state( tap_state_t new_end_state )
{
	/* this is the state we think the TAPs will be in at completion of the
	   current TAP operation, was end_state
	*/
	end_state_follower = new_end_state;
}

tap_state_t tap_get_end_state()
{
	return end_state_follower;
}


int tap_move_ndx( tap_state_t astate )
{
	/* given a stable state, return the index into the tms_seqs[] array within tap_get_tms_path() */

	/* old version
	const static int move_map[16] =
	{
		0, -1, -1,  2, -1,  3, -1, -1,
		1, -1, -1,  4, -1,  5, -1, -1
	};
	*/

	int ndx;

	switch( astate )
	{
	case TAP_RESET:		ndx = 0;			break;
	case TAP_DRSHIFT:	ndx = 2;			break;
	case TAP_DRPAUSE:	ndx = 3;			break;
	case TAP_IDLE:		ndx = 1;			break;
	case TAP_IRSHIFT:	ndx = 4;			break;
	case TAP_IRPAUSE:	ndx = 5;			break;
	default:
		LOG_ERROR( "fatal: unstable state \"%s\" used in tap_move_ndx()", tap_state_name(astate) );
		exit(1);
	}

	return ndx;
}

int tap_get_tms_path( tap_state_t from, tap_state_t to )
{
	/* tap_move[i][j]: tap movement command to go from state i to state j
	 * 0: Test-Logic-Reset
	 * 1: Run-Test/Idle
	 * 2: Shift-DR
	 * 3: Pause-DR
	 * 4: Shift-IR
	 * 5: Pause-IR
	 *
	 * DRSHIFT->DRSHIFT and IRSHIFT->IRSHIFT have to be caught in interface specific code
	 */
	const static u8 tms_seqs[6][6] =
	{
		/* value clocked to TMS to move from one of six stable states to another */

		/* RESET  IDLE  DRSHIFT  DRPAUSE  IRSHIFT  IRPAUSE */
		{  0x7f, 0x00,    0x17,    0x0a,    0x1b,    0x16 },	/* RESET */
		{  0x7f, 0x00,    0x25,    0x05,    0x2b,    0x0b },	/* IDLE */
		{  0x7f, 0x31,    0x00,    0x01,    0x0f,    0x2f },	/* DRSHIFT  */
		{  0x7f, 0x30,    0x20,    0x17,    0x1e,    0x2f },	/* DRPAUSE  */
		{  0x7f, 0x31,    0x07,    0x17,    0x00,    0x01 },	/* IRSHIFT  */
		{  0x7f, 0x30,    0x1c,    0x17,    0x20,    0x2f }	/* IRPAUSE  */
	};

	if( !tap_is_state_stable(from) )
	{
		LOG_ERROR( "fatal: tap_state \"from\" (=%s) is not stable", tap_state_name(from) );
		exit(1);
	}

	if( !tap_is_state_stable(to) )
	{
		LOG_ERROR( "fatal: tap_state \"to\" (=%s) is not stable", tap_state_name(to) );
		exit(1);
	}

	/* @todo: support other than 7 clocks ? */
	return tms_seqs[tap_move_ndx(from)][tap_move_ndx(to)];
}


bool tap_is_state_stable(tap_state_t astate)
{
	bool is_stable;

	/* 	A switch() is used because it is symbol dependent
		(not value dependent like an array), and can also check bounds.
	*/
	switch( astate )
	{
	case TAP_RESET:
	case TAP_IDLE:
	case TAP_DRSHIFT:
	case TAP_DRPAUSE:
	case TAP_IRSHIFT:
	case TAP_IRPAUSE:
		is_stable = true;
		break;
	default:
		is_stable = false;
	}

	return is_stable;
}

tap_state_t tap_state_transition(tap_state_t cur_state, bool tms)
{
	tap_state_t new_state;

	/* 	A switch is used because it is symbol dependent and not value dependent
		like an array.  Also it can check for out of range conditions.
	*/

	if (tms)
	{
		switch (cur_state)
		{
		case TAP_RESET:
			new_state = cur_state;
			break;
		case TAP_IDLE:
		case TAP_DRUPDATE:
		case TAP_IRUPDATE:
			new_state = TAP_DRSELECT;
			break;
		case TAP_DRSELECT:
			new_state = TAP_IRSELECT;
			break;
		case TAP_DRCAPTURE:
		case TAP_DRSHIFT:
			new_state = TAP_DREXIT1;
			break;
		case TAP_DREXIT1:
		case TAP_DREXIT2:
			new_state = TAP_DRUPDATE;
			break;
		case TAP_DRPAUSE:
			new_state = TAP_DREXIT2;
			break;
		case TAP_IRSELECT:
			new_state = TAP_RESET;
			break;
		case TAP_IRCAPTURE:
		case TAP_IRSHIFT:
			new_state = TAP_IREXIT1;
			break;
		case TAP_IREXIT1:
		case TAP_IREXIT2:
			new_state = TAP_IRUPDATE;
			break;
		case TAP_IRPAUSE:
			new_state = TAP_IREXIT2;
			break;
		default:
			LOG_ERROR( "fatal: invalid argument cur_state=%d", cur_state );
			exit(1);
			break;
		}
	}
	else
	{
		switch (cur_state)
		{
		case TAP_RESET:
		case TAP_IDLE:
		case TAP_DRUPDATE:
		case TAP_IRUPDATE:
			new_state = TAP_IDLE;
			break;
		case TAP_DRSELECT:
			new_state = TAP_DRCAPTURE;
			break;
		case TAP_DRCAPTURE:
		case TAP_DRSHIFT:
		case TAP_DREXIT2:
			new_state = TAP_DRSHIFT;
			break;
		case TAP_DREXIT1:
		case TAP_DRPAUSE:
			new_state = TAP_DRPAUSE;
			break;
		case TAP_IRSELECT:
			new_state = TAP_IRCAPTURE;
			break;
		case TAP_IRCAPTURE:
		case TAP_IRSHIFT:
		case TAP_IREXIT2:
			new_state = TAP_IRSHIFT;
			break;
		case TAP_IREXIT1:
		case TAP_IRPAUSE:
			new_state = TAP_IRPAUSE;
			break;
		default:
			LOG_ERROR( "fatal: invalid argument cur_state=%d", cur_state );
			exit(1);
			break;
		}
	}

	return new_state;
}

const char* tap_state_name(tap_state_t state)
{
	const char* ret;

	switch( state )
	{
	case TAP_RESET:		ret = "RESET";			break;
	case TAP_IDLE:		ret = "IDLE";			break;
	case TAP_DRSELECT:	ret = "DRSELECT";		break;
	case TAP_DRCAPTURE: ret = "DRCAPTURE";		break;
	case TAP_DRSHIFT:	ret = "DRSHIFT";			break;
	case TAP_DREXIT1:	ret = "DREXIT1";			break;
	case TAP_DRPAUSE:	ret = "DRPAUSE";			break;
	case TAP_DREXIT2:	ret = "DREXIT2";			break;
	case TAP_DRUPDATE:	ret = "DRUPDATE";		break;
	case TAP_IRSELECT:	ret = "IRSELECT";		break;
	case TAP_IRCAPTURE: ret = "IRCAPTURE";		break;
	case TAP_IRSHIFT:	ret = "IRSHIFT";			break;
	case TAP_IREXIT1:	ret = "IREXIT1";			break;
	case TAP_IRPAUSE:	ret = "IRPAUSE";			break;
	case TAP_IREXIT2:	ret = "IREXIT2";			break;
	case TAP_IRUPDATE:	ret = "IRUPDATE";		break;
	default:				ret = "???";
	}

	return ret;
}

int tap_state_by_name( const char *name )
{
	int x;

	for( x = 0 ; x < 16 ; x++ ){
		/* be nice to the human */
		if( 0 == strcasecmp( name, tap_state_name(x) ) ){
			return x;
		}
	}
	/* not found */
	return -1;
}

/*-----</Cable Helper API>--------------------------------------*/
