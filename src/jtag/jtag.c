/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                      *
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


char* tap_state_strings[16] =
{
	"tlr", 
	"sds", "cd", "sd", "e1d", "pd", "e2d", "ud",
	"rti",
	"sis", "ci", "si", "e1i", "pi", "e2i", "ui"
};

typedef struct cmd_queue_page_s
{
	void *address;
	size_t used;
	struct cmd_queue_page_s *next;
} cmd_queue_page_t;

#define CMD_QUEUE_PAGE_SIZE (1024 * 1024)
static cmd_queue_page_t *cmd_queue_pages = NULL;

/* tap_move[i][j]: tap movement command to go from state i to state j
 * 0: Test-Logic-Reset
 * 1: Run-Test/Idle
 * 2: Shift-DR
 * 3: Pause-DR
 * 4: Shift-IR
 * 5: Pause-IR
 * 
 * SD->SD and SI->SI have to be caught in interface specific code
 */
u8 tap_move[6][6] =
{
/*	  TLR   RTI   SD    PD    SI    PI             */
	{0x7f, 0x00, 0x17, 0x0a, 0x1b, 0x16},	/* TLR */
	{0x7f, 0x00, 0x25, 0x05, 0x2b, 0x0b},	/* RTI */
	{0x7f, 0x31, 0x00, 0x01, 0x0f, 0x2f},	/* SD  */
	{0x7f, 0x30, 0x20, 0x17, 0x1e, 0x2f},	/* PD  */
	{0x7f, 0x31, 0x07, 0x17, 0x00, 0x01},	/* SI  */
	{0x7f, 0x30, 0x1c, 0x17, 0x20, 0x2f}	/* PI  */
};

int tap_move_map[16] = {
	0, -1, -1,  2, -1,  3, -1, -1,
	1, -1, -1,  4, -1,  5, -1, -1
};

tap_transition_t tap_transitions[16] =
{
	{TAP_TLR, TAP_RTI},		/* TLR */
	{TAP_SIS, TAP_CD},		/* SDS */
	{TAP_E1D, TAP_SD},		/* CD  */
	{TAP_E1D, TAP_SD},		/* SD  */
	{TAP_UD,  TAP_PD}, 		/* E1D */
	{TAP_E2D, TAP_PD},		/* PD  */
	{TAP_UD,  TAP_SD},		/* E2D */
	{TAP_SDS, TAP_RTI},		/* UD  */
	{TAP_SDS, TAP_RTI},		/* RTI */
	{TAP_TLR, TAP_CI},		/* SIS */
	{TAP_E1I, TAP_SI},		/* CI  */
	{TAP_E1I, TAP_SI},		/* SI  */
	{TAP_UI,  TAP_PI}, 		/* E1I */
	{TAP_E2I, TAP_PI},		/* PI  */
	{TAP_UI,  TAP_SI},		/* E2I */
	{TAP_SDS, TAP_RTI}		/* UI  */
};

char* jtag_event_strings[] =
{
	"JTAG controller reset (TLR or TRST)"
};

/* kludge!!!! these are just global variables that the
 * interface use internally. They really belong
 * inside the drivers, but we don't want to break
 * linking the drivers!!!!
 */
enum tap_state end_state = TAP_TLR;
enum tap_state cur_state = TAP_TLR;
int jtag_trst = 0;
int jtag_srst = 0;

jtag_command_t *jtag_command_queue = NULL;
jtag_command_t **last_comand_pointer = &jtag_command_queue;
jtag_device_t *jtag_devices = NULL;
int jtag_num_devices = 0;
int jtag_ir_scan_size = 0;
enum reset_types jtag_reset_config = RESET_NONE;
enum tap_state cmd_queue_end_state = TAP_TLR;
enum tap_state cmd_queue_cur_state = TAP_TLR;

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
	extern jtag_interface_t eCosBoard_interface;
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

jtag_interface_t *jtag_interfaces[] = {
#if BUILD_ECOSBOARD == 1
	&eCosBoard_interface,
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
	NULL,
};

jtag_interface_t *jtag = NULL;

/* configuration */
jtag_interface_t *jtag_interface = NULL;
int jtag_speed = 0;



/* forward declarations */
void jtag_add_pathmove(int num_states, enum tap_state *path);
void jtag_add_runtest(int num_cycles, enum tap_state endstate);
void jtag_add_end_state(enum tap_state endstate);
void jtag_add_sleep(u32 us);
int jtag_execute_queue(void);
int jtag_cancel_queue(void);

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

/* returns a pointer to the n-th device in the scan chain */
jtag_device_t* jtag_get_device(int num)
{
	jtag_device_t *device = jtag_devices;
	int i = 0;

	while (device)
	{
		if (num == i)
			return device;
		device = device->next;
		i++;
	}
	
	LOG_ERROR("jtag device number %d not defined", num);
	exit(-1);
}

void* cmd_queue_alloc(size_t size)
{
	cmd_queue_page_t **p_page = &cmd_queue_pages;
	int offset;
	u8 *t;

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

void cmd_queue_free()
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

static void jtag_prelude1()
{
	if (jtag_trst == 1)
	{
		LOG_WARNING("JTAG command queued, while TRST is low (TAP in reset)");
		jtag_error=ERROR_JTAG_TRST_ASSERTED;
		return;
	}

	if (cmd_queue_end_state == TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
}

static void jtag_prelude(enum tap_state state)
{
	jtag_prelude1();
	
	if (state != -1)
		jtag_add_end_state(state);

	cmd_queue_cur_state = cmd_queue_end_state;
}

void jtag_add_ir_scan(int num_fields, scan_field_t *fields, enum tap_state state)
{
	int retval;
	
	jtag_prelude(state);
	
	retval=interface_jtag_add_ir_scan(num_fields, fields, cmd_queue_end_state);
	if (retval!=ERROR_OK)
		jtag_error=retval;
}

int MINIDRIVER(interface_jtag_add_ir_scan)(int num_fields, scan_field_t *fields, enum tap_state state)
{	
	jtag_command_t **last_cmd;
	jtag_device_t *device;
	int i, j;
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
	(*last_cmd)->cmd.scan->num_fields = jtag_num_devices;	/* one field per device */
	(*last_cmd)->cmd.scan->fields = cmd_queue_alloc(jtag_num_devices * sizeof(scan_field_t));
	(*last_cmd)->cmd.scan->end_state = state;

	for (i = 0; i < jtag_num_devices; i++)
	{
		int found = 0;
		device = jtag_get_device(i);
		scan_size = device->ir_length;
		(*last_cmd)->cmd.scan->fields[i].device = i;
		(*last_cmd)->cmd.scan->fields[i].num_bits = scan_size;
		(*last_cmd)->cmd.scan->fields[i].in_value = NULL;
		(*last_cmd)->cmd.scan->fields[i].in_handler = NULL;	/* disable verification by default */

		/* search the list */
		for (j = 0; j < num_fields; j++)
		{
			if (i == fields[j].device)
			{
				found = 1;
				(*last_cmd)->cmd.scan->fields[i].out_value = buf_cpy(fields[j].out_value, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
				(*last_cmd)->cmd.scan->fields[i].out_mask = buf_cpy(fields[j].out_mask, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
			
				if (jtag_verify_capture_ir)
				{
					if (fields[j].in_handler==NULL)
					{
						jtag_set_check_value((*last_cmd)->cmd.scan->fields+i, device->expected, device->expected_mask, NULL);
					} else
					{
						(*last_cmd)->cmd.scan->fields[i].in_handler = fields[j].in_handler;
						(*last_cmd)->cmd.scan->fields[i].in_handler_priv = fields[j].in_handler_priv;
						(*last_cmd)->cmd.scan->fields[i].in_check_value = device->expected; 
						(*last_cmd)->cmd.scan->fields[i].in_check_mask = device->expected_mask;
					}
				}
				
				device->bypass = 0;
				break;
			}
		}
	
		if (!found)
		{
			/* if a device isn't listed, set it to BYPASS */
			(*last_cmd)->cmd.scan->fields[i].out_value = buf_set_ones(cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
			(*last_cmd)->cmd.scan->fields[i].out_mask = NULL;
			device->bypass = 1;
			
		}
		
		/* update device information */
		buf_cpy((*last_cmd)->cmd.scan->fields[i].out_value, jtag_get_device(i)->cur_instr, scan_size);
	}
	
	return ERROR_OK;
}

void jtag_add_plain_ir_scan(int num_fields, scan_field_t *fields, enum tap_state state)
{
	int retval;
	
	jtag_prelude(state);
	
	retval=interface_jtag_add_plain_ir_scan(num_fields, fields, cmd_queue_end_state);
	if (retval!=ERROR_OK)
		jtag_error=retval;
}

int MINIDRIVER(interface_jtag_add_plain_ir_scan)(int num_fields, scan_field_t *fields, enum tap_state state)
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

	for (i = 0; i < num_fields; i++)
	{
		int num_bits = fields[i].num_bits;
		int num_bytes = CEIL(fields[i].num_bits, 8);
		(*last_cmd)->cmd.scan->fields[i].device = fields[i].device;
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

void jtag_add_dr_scan(int num_fields, scan_field_t *fields, enum tap_state state)
{
	int retval;
	
	jtag_prelude(state);

	retval=interface_jtag_add_dr_scan(num_fields, fields, cmd_queue_end_state);
	if (retval!=ERROR_OK)
		jtag_error=retval;
}

int MINIDRIVER(interface_jtag_add_dr_scan)(int num_fields, scan_field_t *fields, enum tap_state state)
{
	int i, j;
	int bypass_devices = 0;
	int field_count = 0;
	int scan_size;

	jtag_command_t **last_cmd = jtag_get_last_command_p();
	jtag_device_t *device = jtag_devices;

	/* count devices in bypass */
	while (device)
	{
		if (device->bypass)
			bypass_devices++;
		device = device->next;
	}
	if (bypass_devices >= jtag_num_devices)
	{
		LOG_ERROR("all devices in bypass");
		return ERROR_JTAG_DEVICE_ERROR;
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
	
	for (i = 0; i < jtag_num_devices; i++)
	{
		int found = 0;
		(*last_cmd)->cmd.scan->fields[field_count].device = i;
	
		for (j = 0; j < num_fields; j++)
		{
			if (i == fields[j].device)
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
			if (!jtag_get_device(i)->bypass)
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
			if (jtag_get_device(i)->bypass)
			{
				LOG_ERROR("BUG: scan data for a device in BYPASS");
				exit(-1);
			}
#endif
		}
	}
	return ERROR_OK;
}

void MINIDRIVER(interface_jtag_add_dr_out)(int device_num, 
		int num_fields,
		const int *num_bits,
		const u32 *value,
		enum tap_state end_state)
{
	int i;
	int field_count = 0;
	int scan_size;
	int bypass_devices = 0;

	jtag_command_t **last_cmd = jtag_get_last_command_p();
	jtag_device_t *device = jtag_devices;
	/* count devices in bypass */
	while (device)
	{
		if (device->bypass)
			bypass_devices++;
		device = device->next;
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
	
	for (i = 0; i < jtag_num_devices; i++)
	{
		(*last_cmd)->cmd.scan->fields[field_count].device = i;
	
		if (i == device_num)
		{
			int j;
#ifdef _DEBUG_JTAG_IO_
			/* if a device is listed, the BYPASS register must not be selected */
			if (jtag_get_device(i)->bypass)
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
			if (!jtag_get_device(i)->bypass)
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

void jtag_add_plain_dr_scan(int num_fields, scan_field_t *fields, enum tap_state state)
{
	int retval;
	
	jtag_prelude(state);

	retval=interface_jtag_add_plain_dr_scan(num_fields, fields, cmd_queue_end_state);
	if (retval!=ERROR_OK)
		jtag_error=retval;
}

int MINIDRIVER(interface_jtag_add_plain_dr_scan)(int num_fields, scan_field_t *fields, enum tap_state state)
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
		(*last_cmd)->cmd.scan->fields[i].device = fields[i].device;
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

void jtag_add_tlr()
{
	jtag_prelude(TAP_TLR);
	
	int retval;
	retval=interface_jtag_add_tlr();
	if (retval!=ERROR_OK)
		jtag_error=retval;
}

int MINIDRIVER(interface_jtag_add_tlr)()
{
	enum tap_state state = TAP_TLR;
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

void jtag_add_pathmove(int num_states, enum tap_state *path)
{
	enum tap_state cur_state=cmd_queue_cur_state;
	int i;
	int retval;

	/* the last state has to be a stable state */
	if (tap_move_map[path[num_states - 1]] == -1)
	{
		LOG_ERROR("BUG: TAP path doesn't finish in a stable state");
		exit(-1);
	}

	for (i=0; i<num_states; i++)
	{
		if ((tap_transitions[cur_state].low != path[i])&&
				(tap_transitions[cur_state].high != path[i]))
		{
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition", tap_state_strings[cur_state], tap_state_strings[path[i]]);
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

int MINIDRIVER(interface_jtag_add_pathmove)(int num_states, enum tap_state *path)
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
	(*last_cmd)->cmd.pathmove->path = cmd_queue_alloc(sizeof(enum tap_state) * num_states);
	
	for (i = 0; i < num_states; i++)
		(*last_cmd)->cmd.pathmove->path[i] = path[i];
	
	return ERROR_OK;
}

int MINIDRIVER(interface_jtag_add_runtest)(int num_cycles, enum tap_state state)
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

void jtag_add_runtest(int num_cycles, enum tap_state state)
{
	int retval;
	
	jtag_prelude(state);
	
	/* executed by sw or hw fifo */
	retval=interface_jtag_add_runtest(num_cycles, cmd_queue_end_state);
	if (retval!=ERROR_OK)
		jtag_error=retval;
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
		LOG_DEBUG("JTAG reset with TLR instead of TRST");
		jtag_add_end_state(TAP_TLR);
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
		cmd_queue_cur_state = TAP_TLR;
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

void jtag_add_end_state(enum tap_state state)
{
	cmd_queue_end_state = state;
	if ((cmd_queue_end_state == TAP_SD)||(cmd_queue_end_state == TAP_SI))
	{
		LOG_ERROR("BUG: TAP_SD/SI can't be end state. Calling code should use a larger scan field");
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

	for (i = 0; i < cmd->num_fields; i++)
	{
		if (cmd->fields[i].out_value)
		{
#ifdef _DEBUG_JTAG_IO_
			char* char_buf = buf_to_str(cmd->fields[i].out_value, (cmd->fields[i].num_bits > 64) ? 64 : cmd->fields[i].num_bits, 16);
#endif
			buf_set_buf(cmd->fields[i].out_value, 0, *buffer, bit_count, cmd->fields[i].num_bits);
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG("fields[%i].out_value: 0x%s", i, char_buf);
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
			char *char_buf;

			char_buf = buf_to_str(captured, (num_bits > 64) ? 64 : num_bits, 16);
			LOG_DEBUG("fields[%i].in_value: 0x%s", i, char_buf);
			free(char_buf);
#endif
			
			if (cmd->fields[i].in_value)
			{
				buf_cpy(captured, cmd->fields[i].in_value, num_bits);
				
				if (cmd->fields[i].in_handler)
				{
					if (cmd->fields[i].in_handler(cmd->fields[i].in_value, cmd->fields[i].in_handler_priv, cmd->fields+i) != ERROR_OK)
					{
						LOG_WARNING("in_handler reported a failed check");
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
					LOG_WARNING("in_handler reported a failed check");
					retval = ERROR_JTAG_QUEUE_FAILED;
				}
			}

			free(captured);
		}
		bit_count += cmd->fields[i].num_bits;
	}

	return retval;
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
	
	if (compare_failed)
		{
		/* An error handler could have caught the failing check
		 * only report a problem when there wasn't a handler, or if the handler
		 * acknowledged the error
		 */ 
		if (compare_failed)
		{
			char *captured_char = buf_to_str(captured, (num_bits > 64) ? 64 : num_bits, 16);
			char *in_check_value_char = buf_to_str(field->in_check_value, (num_bits > 64) ? 64 : num_bits, 16);

			if (field->in_check_mask)
			{
				char *in_check_mask_char;
				in_check_mask_char = buf_to_str(field->in_check_mask, (num_bits > 64) ? 64 : num_bits, 16);
				LOG_WARNING("value captured during scan didn't pass the requested check: captured: 0x%s check_value: 0x%s check_mask: 0x%s", captured_char, in_check_value_char, in_check_mask_char);
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
	jtag_device_t *device = priv;

	LOG_DEBUG("-");
	
	if (event == JTAG_TRST_ASSERTED)
	{
		buf_set_ones(device->cur_instr, device->ir_length);
		device->bypass = 1;
	}
	
	return ERROR_OK;
}

void jtag_sleep(u32 us)
{
	usleep(us);
}

/* Try to examine chain layout according to IEEE 1149.1 Â§12
 */
int jtag_examine_chain()
{
	jtag_device_t *device = jtag_devices;
	scan_field_t field;
	u8 idcode_buffer[JTAG_MAX_CHAIN_SIZE * 4];
	int i;
	int bit_count;
	int device_count = 0;
	u8 zero_check = 0x0;
	u8 one_check = 0xff;
	
	field.device = 0;
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
	
	jtag_add_plain_dr_scan(1, &field, TAP_TLR);
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
	
	for (bit_count = 0; bit_count < (JTAG_MAX_CHAIN_SIZE * 32) - 31;)
	{
		u32 idcode = buf_get_u32(idcode_buffer, bit_count, 32);
		if ((idcode & 1) == 0)
		{
			/* LSB must not be 0, this indicates a device in bypass */
			device_count++;
			
			bit_count += 1;
		}
		else
		{
			u32 manufacturer;
			u32 part;
			u32 version;
			
			if (idcode == 0x000000FF)
			{
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
					if (idcode != 0x000000FF)
					{
						LOG_WARNING("Unexpected idcode after end of chain! 0x%08x", idcode);
					}
				}
				
				break;
			}
			
			if (device)
			{
				device->idcode = idcode;
				device = device->next;
			}
			device_count++;
			
			manufacturer = (idcode & 0xffe) >> 1;
			part = (idcode & 0xffff000) >> 12;
			version = (idcode & 0xf0000000) >> 28;

			LOG_INFO("JTAG device found: 0x%8.8x (Manufacturer: 0x%3.3x, Part: 0x%4.4x, Version: 0x%1.1x)", 
				idcode, manufacturer, part, version);
			
			bit_count += 32;
		}
	}
	
	/* see if number of discovered devices matches configuration */
	if (device_count != jtag_num_devices)
	{
		LOG_ERROR("number of discovered devices in JTAG chain (%i) doesn't match configuration (%i)", 
				device_count, jtag_num_devices);
		LOG_ERROR("check the config file and ensure proper JTAG communication (connections, speed, ...)");
		return ERROR_JTAG_INIT_FAILED;
	}
	
	return ERROR_OK;
}

int jtag_validate_chain()
{
	jtag_device_t *device = jtag_devices;
	int total_ir_length = 0;
	u8 *ir_test = NULL;
	scan_field_t field;
	int chain_pos = 0;
	
	while (device)
	{
		total_ir_length += device->ir_length;
		device = device->next;
	}
	
	total_ir_length += 2;
	ir_test = malloc(CEIL(total_ir_length, 8));
	buf_set_ones(ir_test, total_ir_length);
	
	field.device = 0;
	field.num_bits = total_ir_length;
	field.out_value = ir_test;
	field.out_mask = NULL;
	field.in_value = ir_test;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;
	
	jtag_add_plain_ir_scan(1, &field, TAP_TLR);
	jtag_execute_queue();
	
	device = jtag_devices;
	while (device)
	{
		if (buf_get_u32(ir_test, chain_pos, 2) != 0x1)
		{
			char *cbuf = buf_to_str(ir_test, total_ir_length, 16);
			LOG_ERROR("Error validating JTAG scan chain, IR mismatch, scan returned 0x%s", cbuf);
			free(cbuf);
			free(ir_test);
			return ERROR_JTAG_INIT_FAILED;
		}
		chain_pos += device->ir_length;
		device = device->next;
	}
	
	if (buf_get_u32(ir_test, chain_pos, 2) != 0x3)
	{
		char *cbuf = buf_to_str(ir_test, total_ir_length, 16);
		LOG_ERROR("Error validating JTAG scan chain, IR mismatch, scan returned 0x%s", cbuf);
		free(cbuf);
		free(ir_test);
		return ERROR_JTAG_INIT_FAILED;
	}
	
	free(ir_test);
	
	return ERROR_OK;
}

int jtag_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "interface", handle_interface_command,
		COMMAND_CONFIG, NULL);
	register_command(cmd_ctx, NULL, "jtag_speed", handle_jtag_speed_command,
		COMMAND_ANY, "set jtag speed (if supported)");
	register_command(cmd_ctx, NULL, "jtag_khz", handle_jtag_khz_command,
		COMMAND_ANY, "same as jtag_speed, except it takes maximum khz as arguments. 0 KHz = RTCK.");
	register_command(cmd_ctx, NULL, "jtag_device", handle_jtag_device_command,
		COMMAND_CONFIG, "jtag_device <ir_length> <ir_expected> <ir_mask>");
	register_command(cmd_ctx, NULL, "reset_config", handle_reset_config_command,
		COMMAND_CONFIG, NULL);
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
	int validate_tries = 0;
	jtag_device_t *device;
	int retval;

	LOG_DEBUG("Init JTAG chain");
	
	device = jtag_devices;
	jtag_ir_scan_size = 0;
	jtag_num_devices = 0;
	while (device != NULL)
	{
		jtag_ir_scan_size += device->ir_length;
		jtag_num_devices++;
		device = device->next;
	}
	
	jtag_add_tlr();
	if ((retval=jtag_execute_queue())!=ERROR_OK)
		return retval;

	/* examine chain first, as this could discover the real chain layout */
	if (jtag_examine_chain() != ERROR_OK)
	{
		LOG_ERROR("trying to validate configured JTAG chain anyway...");
	}
	
	while (jtag_validate_chain() != ERROR_OK)
	{
		validate_tries++;
		
		if (validate_tries > 5)
		{
			LOG_ERROR("Could not validate JTAG chain, exit");
			return ERROR_JTAG_INVALID_INTERFACE;
		}
		usleep(10000);
	}
	
	return ERROR_OK;
}

int jtag_init_reset(struct command_context_s *cmd_ctx)
{
	int retval;

	if ((retval=jtag_interface_init(cmd_ctx)) != ERROR_OK)
		return retval;

	LOG_DEBUG("Trying to bring the JTAG controller to life by asserting TRST / TLR");

	/* Reset can happen after a power cycle.
	 * 
	 * Ideally we would only assert TRST or run TLR before the target reset.
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
	jtag_add_reset(1, 0); /* TLR or TRST */
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
	return ERROR_FAIL;	
}

int handle_interface_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int i;

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
			if (jtag_interfaces[i]->register_commands(cmd_ctx) != ERROR_OK)
				exit(-1);

			jtag_interface = jtag_interfaces[i];
			
			if (jtag_interface->khz == NULL)
			{
				jtag_interface->khz = default_khz;
			}
			if (jtag_interface->speed_div == NULL)
			{
				jtag_interface->speed_div = default_speed_div;
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
	jtag_device_t **last_device_p = &jtag_devices;

	if (*last_device_p)
	{
		while ((*last_device_p)->next)
			last_device_p = &((*last_device_p)->next);
		last_device_p = &((*last_device_p)->next);
	}

	if (argc < 3)
		return ERROR_OK;

	*last_device_p = malloc(sizeof(jtag_device_t));
	(*last_device_p)->ir_length = strtoul(args[0], NULL, 0);

	(*last_device_p)->expected = malloc((*last_device_p)->ir_length);
	buf_set_u32((*last_device_p)->expected, 0, (*last_device_p)->ir_length, strtoul(args[1], NULL, 0));
	(*last_device_p)->expected_mask = malloc((*last_device_p)->ir_length);
	buf_set_u32((*last_device_p)->expected_mask, 0, (*last_device_p)->ir_length, strtoul(args[2], NULL, 0));

	(*last_device_p)->cur_instr = malloc((*last_device_p)->ir_length);
	(*last_device_p)->bypass = 1;
	buf_set_ones((*last_device_p)->cur_instr, (*last_device_p)->ir_length);
	
	(*last_device_p)->next = NULL;
	
	jtag_register_event_callback(jtag_reset_callback, (*last_device_p));
	
	jtag_num_devices++;
	
	return ERROR_OK;
}

int handle_scan_chain_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	jtag_device_t *device = jtag_devices;
	int device_count = 0;
	
	while (device)
	{
		u32 expected, expected_mask, cur_instr;
		expected = buf_get_u32(device->expected, 0, device->ir_length);
		expected_mask = buf_get_u32(device->expected_mask, 0, device->ir_length);
		cur_instr = buf_get_u32(device->cur_instr, 0, device->ir_length);
		command_print(cmd_ctx, "%i: idcode: 0x%8.8x ir length %i, ir capture 0x%x, ir mask 0x%x, current instruction 0x%x", device_count, device->idcode, device->ir_length, expected, expected_mask, cur_instr);
		device = device->next;
		device_count++;
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
			LOG_ERROR("invalid reset_config argument, defaulting to none");
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
				LOG_ERROR("invalid reset_config argument, defaulting to none");
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
			LOG_ERROR("invalid reset_config argument, defaulting to none");
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
			LOG_ERROR("invalid reset_config argument, defaulting to none");
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
		retval=ERROR_COMMAND_SYNTAX_ERROR;
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
		retval=ERROR_COMMAND_SYNTAX_ERROR;
	}
	command_print(cmd_ctx, "jtag_khz: %d", speed_khz);
	return retval;

}

int handle_endstate_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	enum tap_state state;

	if (argc < 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	else
	{
		for (state = 0; state < 16; state++)
		{
			if (strcmp(args[0], tap_state_strings[state]) == 0)
			{
				jtag_add_end_state(state);
				jtag_execute_queue();
			}
		}
	}
	command_print(cmd_ctx, "current endstate: %s", tap_state_strings[cmd_queue_end_state]);
	
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
	
	if ((argc < 2) || (argc % 2))
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	fields = malloc(sizeof(scan_field_t) * argc / 2);
	
	for (i = 0; i < argc / 2; i++)
	{
		int device = strtoul(args[i*2], NULL, 0);
		int field_size = jtag_get_device(device)->ir_length;
		fields[i].device = device;
		fields[i].out_value = malloc(CEIL(field_size, 8));
		buf_set_u32(fields[i].out_value, 0, field_size, strtoul(args[i*2+1], NULL, 0));
		fields[i].out_mask = NULL;
		fields[i].in_value = NULL;
		fields[i].in_check_mask = NULL;
		fields[i].in_handler = NULL;
		fields[i].in_handler_priv = NULL;
	}

	jtag_add_ir_scan(argc / 2, fields, -1);
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
	long device;

	/* args[1] = device
	 * args[2] = num_bits
	 * args[3] = hex string
	 * ... repeat num bits and hex string ...
	 */
	if ((argc < 4) || ((argc % 2)!=0))
	{
		Jim_WrongNumArgs(interp, 1, args, "wrong arguments");
		return JIM_ERR;
	}

	for (i = 2; i < argc; i+=2)
	{
		long bits;

		e = Jim_GetLong(interp, args[i], &bits);
		if (e != JIM_OK)
			return e;
	}

	e = Jim_GetLong(interp, args[1], &device);
	if (e != JIM_OK)
		return e;

	num_fields=(argc-2)/2;
	fields = malloc(sizeof(scan_field_t) * num_fields);
	for (i = 2; i < argc; i+=2)
	{
		long bits;
		int len;
		const char *str;

		Jim_GetLong(interp, args[i], &bits);
		str = Jim_GetString(args[i+1], &len);
		
		fields[field_count].device = device;
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
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
	{
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "drscan: jtag execute failed", NULL);
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
