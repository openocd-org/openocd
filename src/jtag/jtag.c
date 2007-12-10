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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "replacements.h"

#include "jtag.h"

#include "command.h"
#include "log.h"
#include "interpreter.h"

#include "stdlib.h"
#include "string.h"
#include <unistd.h>

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
	"SRST asserted",
	"TRST asserted",
	"SRST released",
	"TRST released"
};

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

/* jtag interfaces (parport, FTDI-USB, TI-USB, ...)
 */
#if BUILD_PARPORT == 1
	extern jtag_interface_t parport_interface;
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

jtag_interface_t *jtag_interfaces[] = {
#if BUILD_PARPORT == 1
	&parport_interface,
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
	NULL,
};

jtag_interface_t *jtag = NULL;

/* configuration */
char* jtag_interface = NULL;
int jtag_speed = -1;


/* forward declarations */
int jtag_add_ir_scan(int num_fields, scan_field_t *fields, enum tap_state endstate, error_handler_t *error_handler);
int jtag_add_dr_scan(int num_fields, scan_field_t *fields, enum tap_state endstate, error_handler_t *error_handler);
int jtag_add_plain_ir_scan(int num_fields, scan_field_t *fields, enum tap_state endstate, error_handler_t *error_handler);
int jtag_add_plain_dr_scan(int num_fields, scan_field_t *fields, enum tap_state endstate, error_handler_t *error_handler);
int jtag_add_statemove(enum tap_state endstate);
int jtag_add_pathmove(int num_states, enum tap_state *path);
int jtag_add_runtest(int num_cycles, enum tap_state endstate);
int jtag_add_reset(int trst, int srst);
int jtag_add_end_state(enum tap_state endstate);
int jtag_add_sleep(u32 us);
int jtag_execute_queue(void);
int jtag_cancel_queue(void);

/* jtag commands */
int handle_interface_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_jtag_speed_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_jtag_device_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_reset_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_jtag_nsrst_delay_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_jtag_ntrst_delay_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int handle_scan_chain_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int handle_endstate_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_jtag_reset_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_runtest_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_statemove_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_irscan_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_drscan_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

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
	
	DEBUG("jtag event: %s", jtag_event_strings[event]);
	
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

	return NULL;
}

void* cmd_queue_alloc(size_t size)
{
	cmd_queue_page_t **p_page = &cmd_queue_pages;
	int offset;

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
	
	return ((*p_page)->address) + offset;
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

int jtag_add_ir_scan(int num_fields, scan_field_t *fields, enum tap_state state, error_handler_t *error_handler)
{
	jtag_command_t **last_cmd;
	jtag_device_t *device;
	int i, j;
	int scan_size = 0;

	if (jtag_trst == 1)
	{
		WARNING("JTAG command queued, while TRST is low (TAP in reset)");
		return ERROR_JTAG_TRST_ASSERTED;
	}

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
	if (error_handler)
	{
		(*last_cmd)->cmd.scan->error_handler = cmd_queue_alloc(sizeof(error_handler_t));
		*(*last_cmd)->cmd.scan->error_handler = *error_handler;
	}
	else
	{
		(*last_cmd)->cmd.scan->error_handler = NULL;
	}
		
	if (state != -1)
		cmd_queue_end_state = state;

	if (cmd_queue_cur_state == TAP_TLR && cmd_queue_end_state != TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_RELEASED);
	
	if (cmd_queue_end_state == TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
	
	cmd_queue_cur_state = cmd_queue_end_state;
		
	for (i=0; i < jtag_num_devices; i++)
	{
		int found = 0;
		device = jtag_get_device(i);
		scan_size = device->ir_length;
		(*last_cmd)->cmd.scan->fields[i].device = i;
		(*last_cmd)->cmd.scan->fields[i].num_bits = scan_size;
		(*last_cmd)->cmd.scan->fields[i].in_value = NULL;
		if (jtag_verify_capture_ir)
		{
			(*last_cmd)->cmd.scan->fields[i].in_check_value = buf_cpy(device->expected, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
			(*last_cmd)->cmd.scan->fields[i].in_check_mask = buf_cpy(device->expected_mask, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
		}
		else
		{
			(*last_cmd)->cmd.scan->fields[i].in_check_value = NULL;
			(*last_cmd)->cmd.scan->fields[i].in_check_mask = NULL;
		}
		(*last_cmd)->cmd.scan->fields[i].in_handler = NULL;
		(*last_cmd)->cmd.scan->fields[i].in_handler_priv = NULL;

		/* search the list */
		for (j=0; j < num_fields; j++)
		{
			if (i == fields[j].device)
			{
				found = 1;
				(*last_cmd)->cmd.scan->fields[i].out_value = buf_cpy(fields[j].out_value, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
				(*last_cmd)->cmd.scan->fields[i].out_mask = buf_cpy(fields[j].out_mask, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
				
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

int jtag_add_plain_ir_scan(int num_fields, scan_field_t *fields, enum tap_state state, error_handler_t *error_handler)
{
	jtag_command_t **last_cmd;
	int i;

	if (jtag_trst == 1)
	{
		WARNING("JTAG command queued, while TRST is low (TAP in reset)");
		return ERROR_JTAG_TRST_ASSERTED;
	}

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
	if (error_handler)
	{
		(*last_cmd)->cmd.scan->error_handler = cmd_queue_alloc(sizeof(error_handler_t));
		*(*last_cmd)->cmd.scan->error_handler = *error_handler;
	}
	else
	{
		(*last_cmd)->cmd.scan->error_handler = NULL;
	}

	if (state != -1)
		cmd_queue_end_state = state;

	if (cmd_queue_cur_state == TAP_TLR && cmd_queue_end_state != TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_RELEASED);
	
	if (cmd_queue_end_state == TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
		
	cmd_queue_cur_state = cmd_queue_end_state;
	
	for (i = 0; i < num_fields; i++)
	{
		int num_bits = fields[i].num_bits;
		int num_bytes = CEIL(fields[i].num_bits, 8);
		(*last_cmd)->cmd.scan->fields[i].device = fields[i].device;
		(*last_cmd)->cmd.scan->fields[i].num_bits = num_bits;
		(*last_cmd)->cmd.scan->fields[i].out_value = buf_cpy(fields[i].out_value, cmd_queue_alloc(num_bytes), num_bits);
		(*last_cmd)->cmd.scan->fields[i].out_mask = buf_cpy(fields[i].out_mask, cmd_queue_alloc(num_bytes), num_bits);
		(*last_cmd)->cmd.scan->fields[i].in_value = fields[i].in_value;
		(*last_cmd)->cmd.scan->fields[i].in_check_value = buf_cpy(fields[i].in_check_value, cmd_queue_alloc(num_bytes), num_bits);
		(*last_cmd)->cmd.scan->fields[i].in_check_mask = buf_cpy(fields[i].in_check_mask, cmd_queue_alloc(num_bytes), num_bits);
		(*last_cmd)->cmd.scan->fields[i].in_handler = NULL;
		(*last_cmd)->cmd.scan->fields[i].in_handler_priv = NULL;
	}
	return ERROR_OK;
}

int jtag_add_dr_scan(int num_fields, scan_field_t *fields, enum tap_state state, error_handler_t *error_handler)
{
	int i, j;
	int bypass_devices = 0;
	int field_count = 0;
	jtag_command_t **last_cmd = jtag_get_last_command_p();
	jtag_device_t *device = jtag_devices;
	int scan_size;

	if (jtag_trst == 1)
	{
		WARNING("JTAG command queued, while TRST is low (TAP in reset)");
		return ERROR_JTAG_TRST_ASSERTED;
	}

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
	(*last_cmd)->cmd.scan->end_state = state;
	if (error_handler)
	{
		(*last_cmd)->cmd.scan->error_handler = cmd_queue_alloc(sizeof(error_handler_t));
		*(*last_cmd)->cmd.scan->error_handler = *error_handler;
	}
	else
	{
		(*last_cmd)->cmd.scan->error_handler = NULL;
	}
	
	if (state != -1)
		cmd_queue_end_state = state;

	if (cmd_queue_cur_state == TAP_TLR && cmd_queue_end_state != TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_RELEASED);
	
	if (cmd_queue_end_state == TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
			
	cmd_queue_cur_state = cmd_queue_end_state;
	
	for (i=0; i < jtag_num_devices; i++)
	{
		int found = 0;
		(*last_cmd)->cmd.scan->fields[field_count].device = i;
	
		for (j=0; j < num_fields; j++)
		{
			if (i == fields[j].device)
			{
				found = 1;
				scan_size = fields[j].num_bits;
				(*last_cmd)->cmd.scan->fields[field_count].num_bits = scan_size;
				(*last_cmd)->cmd.scan->fields[field_count].out_value = buf_cpy(fields[j].out_value, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
				(*last_cmd)->cmd.scan->fields[field_count].out_mask = buf_cpy(fields[j].out_mask, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
				(*last_cmd)->cmd.scan->fields[field_count].in_value = fields[j].in_value;
				(*last_cmd)->cmd.scan->fields[field_count].in_check_value = buf_cpy(fields[j].in_check_value, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
				(*last_cmd)->cmd.scan->fields[field_count].in_check_mask = buf_cpy(fields[j].in_check_mask, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
				(*last_cmd)->cmd.scan->fields[field_count].in_handler = fields[j].in_handler;
				(*last_cmd)->cmd.scan->fields[field_count++].in_handler_priv = fields[j].in_handler_priv;
			}
		}
		if (!found)
		{
			/* if a device isn't listed, the BYPASS register should be selected */
			if (!jtag_get_device(i)->bypass)
			{
				ERROR("BUG: no scan data for a device not in BYPASS");
				exit(-1);
			}
	
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
			/* if a device is listed, the BYPASS register must not be selected */
			if (jtag_get_device(i)->bypass)
			{
				WARNING("scan data for a device in BYPASS");
			}
		}
	}
	return ERROR_OK;
}

int jtag_add_plain_dr_scan(int num_fields, scan_field_t *fields, enum tap_state state, error_handler_t *error_handler)
{
	int i;
	jtag_command_t **last_cmd = jtag_get_last_command_p();
	
	if (jtag_trst == 1)
	{
		WARNING("JTAG command queued, while TRST is low (TAP in reset)");
		return ERROR_JTAG_TRST_ASSERTED;
	}

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
	if (error_handler)
	{
		(*last_cmd)->cmd.scan->error_handler = cmd_queue_alloc(sizeof(error_handler_t));
		*(*last_cmd)->cmd.scan->error_handler = *error_handler;
	}
	else
	{
		(*last_cmd)->cmd.scan->error_handler = NULL;
	}
		
	if (state != -1)
		cmd_queue_end_state = state;

	if (cmd_queue_cur_state == TAP_TLR && cmd_queue_end_state != TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_RELEASED);
	
	if (cmd_queue_end_state == TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
			
	cmd_queue_cur_state = cmd_queue_end_state;
	
	for (i = 0; i < num_fields; i++)
	{
		int num_bits = fields[i].num_bits;
		int num_bytes = CEIL(fields[i].num_bits, 8);
		(*last_cmd)->cmd.scan->fields[i].device = fields[i].device;
		(*last_cmd)->cmd.scan->fields[i].num_bits = num_bits;
		(*last_cmd)->cmd.scan->fields[i].out_value = buf_cpy(fields[i].out_value, cmd_queue_alloc(num_bytes), num_bits);
		(*last_cmd)->cmd.scan->fields[i].out_mask = buf_cpy(fields[i].out_mask, cmd_queue_alloc(num_bytes), num_bits);
		(*last_cmd)->cmd.scan->fields[i].in_value = fields[i].in_value;
		(*last_cmd)->cmd.scan->fields[i].in_check_value = buf_cpy(fields[i].in_check_value, cmd_queue_alloc(num_bytes), num_bits);
		(*last_cmd)->cmd.scan->fields[i].in_check_mask = buf_cpy(fields[i].in_check_mask, cmd_queue_alloc(num_bytes), num_bits);
		(*last_cmd)->cmd.scan->fields[i].in_handler = fields[i].in_handler;
		(*last_cmd)->cmd.scan->fields[i].in_handler_priv = fields[i].in_handler_priv;
	}

	return ERROR_OK;
}
int jtag_add_statemove(enum tap_state state)
{
	jtag_command_t **last_cmd = jtag_get_last_command_p();
	
	if (jtag_trst == 1)
	{
		WARNING("JTAG command queued, while TRST is low (TAP in reset)");
		return ERROR_JTAG_TRST_ASSERTED;
	}

	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->next = NULL;
	(*last_cmd)->type = JTAG_STATEMOVE;

	(*last_cmd)->cmd.statemove = cmd_queue_alloc(sizeof(statemove_command_t));
	(*last_cmd)->cmd.statemove->end_state = state;
	
	if (state != -1)
		cmd_queue_end_state = state;

	if (cmd_queue_cur_state == TAP_TLR && cmd_queue_end_state != TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_RELEASED);
	
	if (cmd_queue_end_state == TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
			
	cmd_queue_cur_state = cmd_queue_end_state;
	
	return ERROR_OK;
}

int jtag_add_pathmove(int num_states, enum tap_state *path)
{
	jtag_command_t **last_cmd = jtag_get_last_command_p();
	int i;
	
	if (jtag_trst == 1)
	{
		WARNING("JTAG command queued, while TRST is low (TAP in reset)");
		return ERROR_JTAG_TRST_ASSERTED;
	}
	
	/* the last state has to be a stable state */
	if (tap_move_map[path[num_states - 1]] == -1)
	{
		ERROR("TAP path doesn't finish in a stable state");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}
	
	if (jtag->support_pathmove)
	{
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
	}
	else
	{
		/* validate the desired path, and see if it fits a default path */
		int begin = 0;
		int end = 0;
		int j;
		
		for (i = 0; i < num_states; i++)
		{
			for (j = i; j < num_states; j++)
			{
				if (tap_move_map[path[j]] != -1)
				{	
					end = j;
					break;
				}
			}
			
			if (begin - end <= 7) 	/* a default path spans no more than 7 states */
			{
				jtag_add_statemove(path[end]);
			}
			else
			{
				ERROR("encountered a TAP path that can't be fulfilled by default paths");	
				return ERROR_JTAG_NOT_IMPLEMENTED;
			}
			
			i = end;
		}
	}

	if (cmd_queue_cur_state == TAP_TLR && cmd_queue_end_state != TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_RELEASED);
	
	if (cmd_queue_end_state == TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
	
	cmd_queue_cur_state = path[num_states - 1];
	
	return ERROR_OK;
}

int jtag_add_runtest(int num_cycles, enum tap_state state)
{
	jtag_command_t **last_cmd = jtag_get_last_command_p();
	
	if (jtag_trst == 1)
	{
		WARNING("JTAG command queued, while TRST is low (TAP in reset)");
		return ERROR_JTAG_TRST_ASSERTED;
	}

	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	(*last_cmd)->next = NULL;
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->type = JTAG_RUNTEST;

	(*last_cmd)->cmd.runtest = cmd_queue_alloc(sizeof(runtest_command_t));
	(*last_cmd)->cmd.runtest->num_cycles = num_cycles;
	(*last_cmd)->cmd.runtest->end_state = state;
	
	if (state != -1)
		cmd_queue_end_state = state;

	if (cmd_queue_cur_state == TAP_TLR && cmd_queue_end_state != TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_RELEASED);
	
	if (cmd_queue_end_state == TAP_TLR)
		jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
			
	cmd_queue_cur_state = cmd_queue_end_state;
	
	return ERROR_OK;
}

int jtag_add_reset(int req_trst, int req_srst)
{
	int trst_with_tms = 0;
	
	jtag_command_t **last_cmd = jtag_get_last_command_p();
	
	if (req_trst == -1)
		req_trst = jtag_trst;
	
	if (req_srst == -1)
		req_srst = jtag_srst;

	/* Make sure that jtag_reset_config allows the requested reset */
	/* if SRST pulls TRST, we can't fulfill srst == 1 with trst == 0 */
	if (((jtag_reset_config & RESET_SRST_PULLS_TRST) && (req_srst == 1)) && (req_trst == 0))
		return ERROR_JTAG_RESET_WOULD_ASSERT_TRST;
		
	/* if TRST pulls SRST, we reset with TAP T-L-R */
	if (((jtag_reset_config & RESET_TRST_PULLS_SRST) && (req_trst == 1)) && (req_srst == 0))
	{
		req_trst = 0;
		trst_with_tms = 1;
	}
	
	if (req_srst && !(jtag_reset_config & RESET_HAS_SRST))
	{
		ERROR("requested nSRST assertion, but the current configuration doesn't support this");
		return ERROR_JTAG_RESET_CANT_SRST;
	}
	
	if (req_trst && !(jtag_reset_config & RESET_HAS_TRST))
	{
		req_trst = 0;
		trst_with_tms = 1;
	}
	
	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	(*last_cmd)->next = NULL;
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->type = JTAG_RESET;

	(*last_cmd)->cmd.reset = cmd_queue_alloc(sizeof(reset_command_t));
	(*last_cmd)->cmd.reset->trst = req_trst;
	(*last_cmd)->cmd.reset->srst = req_srst;

	jtag_trst = req_trst;
	jtag_srst = req_srst;

	if (jtag_srst)
	{
		jtag_call_event_callbacks(JTAG_SRST_ASSERTED);
	}
	else
	{
		jtag_call_event_callbacks(JTAG_SRST_RELEASED);
		if (jtag_nsrst_delay)
			jtag_add_sleep(jtag_nsrst_delay * 1000);
	}
	
	if (trst_with_tms)
	{
		last_cmd = &((*last_cmd)->next);
		
		/* allocate memory for a new list member */
		*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
		(*last_cmd)->next = NULL;
		last_comand_pointer = &((*last_cmd)->next);
		(*last_cmd)->type = JTAG_STATEMOVE;

		(*last_cmd)->cmd.statemove = cmd_queue_alloc(sizeof(statemove_command_t));
		(*last_cmd)->cmd.statemove->end_state = TAP_TLR;
		
		jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
		cmd_queue_cur_state = TAP_TLR;
		cmd_queue_end_state = TAP_TLR;
		
		return ERROR_OK;
	}
	else
	{
		if (jtag_trst)
		{
			/* we just asserted nTRST, so we're now in Test-Logic-Reset,
			 * and inform possible listeners about this
			 */
			cmd_queue_cur_state = TAP_TLR;
			jtag_call_event_callbacks(JTAG_TRST_ASSERTED);
		}
		else
		{
			/* the nTRST line got deasserted, so we're still in Test-Logic-Reset,
			 * but we might want to add a delay to give the TAP time to settle
			 */
			if (jtag_ntrst_delay)
				jtag_add_sleep(jtag_ntrst_delay * 1000);
		}
	}

	return ERROR_OK;
}

int jtag_add_end_state(enum tap_state state)
{
	jtag_command_t **last_cmd = jtag_get_last_command_p();
	
	/* allocate memory for a new list member */
	*last_cmd = cmd_queue_alloc(sizeof(jtag_command_t));
	(*last_cmd)->next = NULL;
	last_comand_pointer = &((*last_cmd)->next);
	(*last_cmd)->type = JTAG_END_STATE;

	(*last_cmd)->cmd.end_state = cmd_queue_alloc(sizeof(end_state_command_t));
	(*last_cmd)->cmd.end_state->end_state = state;

	if (state != -1)
		cmd_queue_end_state = state;
	
	return ERROR_OK;
}

int jtag_add_sleep(u32 us)
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

int jtag_scan_size(scan_command_t *cmd)
{
	int bit_count = 0;
	int i;

	/* count bits in scan command */
	for (i=0; i<cmd->num_fields; i++)
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
			DEBUG("fields[%i].out_value: 0x%s", i, char_buf);
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
	
	for (i=0; i < cmd->num_fields; i++)
	{
		/* if neither in_value, in_check_value nor in_handler
		 * are specified we don't have to examine this field
		 */
		if (cmd->fields[i].in_value || cmd->fields[i].in_check_value || cmd->fields[i].in_handler)
		{
			int num_bits = cmd->fields[i].num_bits;
			u8 *captured = buf_set_buf(buffer, bit_count, malloc(CEIL(num_bits, 8)), 0, num_bits);
			
			#ifdef _DEBUG_JTAG_IO_
				char *char_buf;

				char_buf = buf_to_str(captured, (num_bits > 64) ? 64 : num_bits, 16);
				DEBUG("fields[%i].in_value: 0x%s", i, char_buf);
				free(char_buf);
			#endif
			
			if (cmd->fields[i].in_value)
			{
				buf_cpy(captured, cmd->fields[i].in_value, num_bits);
				
				if (cmd->fields[i].in_handler)
				{
					if (cmd->fields[i].in_handler(cmd->fields[i].in_value, cmd->fields[i].in_handler_priv) != ERROR_OK)
					{
						WARNING("in_handler reported a failed check");
						retval = ERROR_JTAG_QUEUE_FAILED;
					}
				}
			}
			
			/* no in_value specified, but a handler takes care of the scanned data */
			if (cmd->fields[i].in_handler && (!cmd->fields[i].in_value))
			{
				if (cmd->fields[i].in_handler(captured, cmd->fields[i].in_handler_priv) != ERROR_OK)
				{
					/* We're going to call the error:handler later, but if the in_handler
					 * reported an error we report this failure upstream
					 */
					WARNING("in_handler reported a failed check");
					retval = ERROR_JTAG_QUEUE_FAILED;
				}
			}

			if (cmd->fields[i].in_check_value)
			{
				int compare_failed = 0;
				
				if (cmd->fields[i].in_check_mask)
					compare_failed = buf_cmp_mask(captured, cmd->fields[i].in_check_value, cmd->fields[i].in_check_mask, num_bits);
				else
					compare_failed = buf_cmp(captured, cmd->fields[i].in_check_value, num_bits);
				
				if (compare_failed)
				{
					if (cmd->error_handler)
					{
						/* ask the error handler if once has been specified if this is a real problem */ 
						if (cmd->error_handler->error_handler(captured, cmd->error_handler->error_handler_priv) != ERROR_OK)
							retval = ERROR_JTAG_QUEUE_FAILED;
						else
							compare_failed = 0;
					}
					else
					{
						/* if there wasn't a handler specified, we report a failure */
						retval = ERROR_JTAG_QUEUE_FAILED;
					}
					
					/* An error handler could have caught the failing check
					 * only report a problem when there wasn't a handler, or if the handler
					 * acknowledged the error
					 */ 
					if (compare_failed)
					{
						char *captured_char = buf_to_str(captured, (num_bits > 64) ? 64 : num_bits, 16);
						char *in_check_value_char = buf_to_str(cmd->fields[i].in_check_value, (num_bits > 64) ? 64 : num_bits, 16);

						if (cmd->fields[i].in_check_mask)
						{
							char *in_check_mask_char;
							in_check_mask_char = buf_to_str(cmd->fields[i].in_check_mask, (num_bits > 64) ? 64 : num_bits, 16);
							WARNING("value captured during scan didn't pass the requested check: captured: 0x%s check_value: 0x%s check_mask: 0x%s", captured_char, in_check_value_char, in_check_mask_char);
							free(in_check_mask_char);
						}
						else
						{
							WARNING("value captured during scan didn't pass the requested check: captured: 0x%s check_value: 0x%s", captured_char, in_check_value_char);
						}

						free(captured_char);
						free(in_check_value_char);
					}
					
				}
			}
			free(captured);
		}
		bit_count += cmd->fields[i].num_bits;
	}

	return retval;
}

enum scan_type jtag_scan_type(scan_command_t *cmd)
{
	int i;
	int type = 0;
	
	for (i=0; i < cmd->num_fields; i++)
	{
		if (cmd->fields[i].in_check_value || cmd->fields[i].in_value || cmd->fields[i].in_handler)
			type |= SCAN_IN;
		if (cmd->fields[i].out_value)
			type |= SCAN_OUT;
	}

	return type;
}

int jtag_execute_queue(void)
{
	int retval;

	retval = jtag->execute_queue();
	
	cmd_queue_free();

	jtag_command_queue = NULL;
	last_comand_pointer = &jtag_command_queue;

	return retval;
}

int jtag_cancel_queue(void)
{
	cmd_queue_free();
	jtag_command_queue = NULL;
	last_comand_pointer = &jtag_command_queue;

	return ERROR_OK;
}

int jtag_reset_callback(enum jtag_event event, void *priv)
{
	jtag_device_t *device = priv;

	DEBUG("-");
	
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

/* Try to examine chain layout according to IEEE 1149.1 §12
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
	
	jtag_add_plain_dr_scan(1, &field, TAP_TLR, NULL);
	jtag_execute_queue();
	
	for (i = 0; i < JTAG_MAX_CHAIN_SIZE * 4; i++)
	{
		zero_check |= idcode_buffer[i];
		one_check &= idcode_buffer[i];
	}
	
	/* if there wasn't a single non-zero bit or if all bits were one, the scan isn't valid */
	if ((zero_check == 0x00) || (one_check == 0xff))
	{
		ERROR("JTAG communication failure, check connection, JTAG interface, target power etc.");
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
				/* End of chain (invalid manufacturer ID) */
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

			INFO("JTAG device found: 0x%8.8x (Manufacturer: 0x%3.3x, Part: 0x%4.4x, Version: 0x%1.1x)", 
				idcode, manufacturer, part, version);
			
			bit_count += 32;
		}
	}
	
	/* see if number of discovered devices matches configuration */
	if (device_count != jtag_num_devices)
	{
		ERROR("number of discovered devices in JTAG chain (%i) doesn't match configuration (%i)", 
			device_count, jtag_num_devices);
		ERROR("check the config file and ensure proper JTAG communication (connections, speed, ...)");
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
	
	jtag_add_plain_ir_scan(1, &field, TAP_TLR, NULL);
	jtag_execute_queue();
	
	device = jtag_devices;
	while (device)
	{
		if (buf_get_u32(ir_test, chain_pos, 2) != 0x1)
		{
			char *cbuf = buf_to_str(ir_test, total_ir_length, 16);
			ERROR("Error validating JTAG scan chain, IR mismatch, scan returned 0x%s", cbuf);
			free(cbuf);
			return ERROR_JTAG_INIT_FAILED;
		}
		chain_pos += device->ir_length;
		device = device->next;
	}
	
	if (buf_get_u32(ir_test, chain_pos, 2) != 0x3)
	{
		char *cbuf = buf_to_str(ir_test, total_ir_length, 16);
		ERROR("Error validating JTAG scan chain, IR mismatch, scan returned 0x%s", cbuf);
		free(cbuf);
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
		COMMAND_ANY, "set jtag speed (if supported) <speed>");
	register_command(cmd_ctx, NULL, "jtag_device", handle_jtag_device_command,
		COMMAND_CONFIG, NULL);
	register_command(cmd_ctx, NULL, "reset_config", handle_reset_config_command,
		COMMAND_CONFIG, NULL);
	register_command(cmd_ctx, NULL, "jtag_nsrst_delay", handle_jtag_nsrst_delay_command,
		COMMAND_CONFIG, NULL);
	register_command(cmd_ctx, NULL, "jtag_ntrst_delay", handle_jtag_ntrst_delay_command,
		COMMAND_CONFIG, NULL);
		
	register_command(cmd_ctx, NULL, "scan_chain", handle_scan_chain_command,
		COMMAND_EXEC, "print current scan chain configuration");

	register_command(cmd_ctx, NULL, "endstate", handle_endstate_command,
		COMMAND_EXEC, "finish JTAG operations in <tap_state>");
	register_command(cmd_ctx, NULL, "jtag_reset", handle_jtag_reset_command,
		COMMAND_EXEC, "toggle reset lines <trst> <srst>");
	register_command(cmd_ctx, NULL, "runtest", handle_runtest_command,
		COMMAND_EXEC, "move to Run-Test/Idle, and execute <num_cycles>");
	register_command(cmd_ctx, NULL, "statemove", handle_statemove_command,
		COMMAND_EXEC, "move to current endstate or [tap_state]");
	register_command(cmd_ctx, NULL, "irscan", handle_irscan_command,
		COMMAND_EXEC, "execute IR scan <device> <instr> [dev2] [instr2] ...");
	register_command(cmd_ctx, NULL, "drscan", handle_drscan_command,
		COMMAND_EXEC, "execute DR scan <device> <var> [dev2] [var2] ...");

	register_command(cmd_ctx, NULL, "verify_ircapture", handle_verify_ircapture_command,
		COMMAND_ANY, "verify value captured during Capture-IR <enable|disable>");
	return ERROR_OK;
}

int jtag_init(struct command_context_s *cmd_ctx)
{
	int i, validate_tries = 0;
	
	DEBUG("-");

	if (jtag_speed == -1)
		jtag_speed = 0;
	
	if (jtag_interface && (jtag_interface[0] != 0))
		/* configuration var 'jtag_interface' is set, and not empty */
		for (i = 0; jtag_interfaces[i]; i++)
		{
			if (strcmp(jtag_interface, jtag_interfaces[i]->name) == 0)
			{
				jtag_device_t *device;
				device = jtag_devices;
	
				if (jtag_interfaces[i]->init() != ERROR_OK)
					return ERROR_JTAG_INIT_FAILED;
				jtag = jtag_interfaces[i];

				jtag_ir_scan_size = 0;
				jtag_num_devices = 0;
				while (device != NULL)
				{
					jtag_ir_scan_size += device->ir_length;
					jtag_num_devices++;
					device = device->next;
				}
				
				jtag_add_statemove(TAP_TLR);
				jtag_execute_queue();

				/* examine chain first, as this could discover the real chain layout */
				if (jtag_examine_chain() != ERROR_OK)
				{
					ERROR("trying to validate configured JTAG chain anyway...");
				}
				
				while (jtag_validate_chain() != ERROR_OK)
				{
					validate_tries++;
					if (validate_tries > 5)
					{
						ERROR("Could not validate JTAG chain, exit");
						jtag = NULL;
						return ERROR_JTAG_INVALID_INTERFACE;
					}
					usleep(10000);
				}

				return ERROR_OK;
			}
		}
	
	/* no valid interface was found (i.e. the configuration option,
	 * didn't match one of the compiled-in interfaces
	 */
	ERROR("No valid jtag interface found (%s)", jtag_interface);
	ERROR("compiled-in jtag interfaces:");
	for (i = 0; jtag_interfaces[i]; i++)
	{
		ERROR("%i: %s", i, jtag_interfaces[i]->name);
	}
	
	jtag = NULL;
	return ERROR_JTAG_INVALID_INTERFACE;
}

int handle_interface_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int i;
	
	/* only if the configuration var isn't overwritten from cmdline */
	if (!jtag_interface)
	{
		if (args[0] && (args[0][0] != 0))
		{
			for (i=0; jtag_interfaces[i]; i++)
			{
				if (strcmp(args[0], jtag_interfaces[i]->name) == 0)
				{
					if (jtag_interfaces[i]->register_commands(cmd_ctx) != ERROR_OK)
						exit(-1);
				
					jtag_interface = jtag_interfaces[i]->name;
		
					return ERROR_OK;
				}
			}
		}
		
		/* remember the requested interface name, so we can complain about it later */
		jtag_interface = strdup(args[0]);
		DEBUG("'interface' command didn't specify a valid interface");
	}
	
	return ERROR_OK;
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
			ERROR("invalid reset_config argument, defaulting to none");
			jtag_reset_config = RESET_NONE;
			return ERROR_INVALID_ARGUMENTS;
		}
	}
	
	if (argc >= 2)
	{
		if (strcmp(args[1], "srst_pulls_trst") == 0)
			jtag_reset_config |= RESET_SRST_PULLS_TRST;
		else if (strcmp(args[1], "trst_pulls_srst") == 0)
			jtag_reset_config |= RESET_TRST_PULLS_SRST;
		else if (strcmp(args[1], "combined") == 0)
			jtag_reset_config |= RESET_SRST_PULLS_TRST | RESET_TRST_PULLS_SRST;
		else if (strcmp(args[1], "separate") == 0)
			jtag_reset_config &= ~(RESET_SRST_PULLS_TRST | RESET_TRST_PULLS_SRST);
		else
		{
			ERROR("invalid reset_config argument, defaulting to none");
			jtag_reset_config = RESET_NONE;
			return ERROR_INVALID_ARGUMENTS;
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
			ERROR("invalid reset_config argument, defaulting to none");
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
			ERROR("invalid reset_config argument, defaulting to none");
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
		ERROR("jtag_nsrst_delay <ms> command takes one required argument");
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
		ERROR("jtag_ntrst_delay <ms> command takes one required argument");
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
	if (argc == 0)
		command_print(cmd_ctx, "jtag_speed: %i", jtag_speed);

	if (argc > 0)
	{
		/* this command can be called during CONFIG, 
		 * in which case jtag isn't initialized */
		if (jtag)
			jtag->speed(strtoul(args[0], NULL, 0));
		else
			jtag_speed = strtoul(args[0], NULL, 0);
	}

	return ERROR_OK;
}

int handle_endstate_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	enum tap_state state;

	if (argc < 1)
	{
		command_print(cmd_ctx, "usage: endstate <tap_state>");
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
	command_print(cmd_ctx, "current endstate: %s", tap_state_strings[end_state]);
	
	return ERROR_OK;
}

int handle_jtag_reset_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int trst = -1;
	int srst = -1;
	char *usage = "usage: jtag_reset <trst> <srst>";
	int retval;
	
	if (argc < 1)
	{
		command_print(cmd_ctx, usage);
		return ERROR_OK;
	}

	if (args[0][0] == '1')
		trst = 1;
	else if (args[0][0] == '0')
		trst = 0;
	else
	{
		command_print(cmd_ctx, usage);
		return ERROR_OK;
	}

	if (args[1][0] == '1')
		srst = 1;
	else if (args[1][0] == '0')
		srst = 0;
	else
	{
		command_print(cmd_ctx, usage);
		return ERROR_OK;
	}

	if ((retval = jtag_add_reset(trst, srst)) != ERROR_OK)
	{
		switch (retval)
		{
			case ERROR_JTAG_RESET_WOULD_ASSERT_TRST:
				command_print(cmd_ctx, "requested reset would assert trst\nif this is acceptable, use jtag_reset 1 %c", args[1][0]);
				break;
			case ERROR_JTAG_RESET_CANT_SRST:
				command_print(cmd_ctx, "can't assert srst because the current reset_config doesn't support it");
				break;
			default:
				command_print(cmd_ctx, "unknown error");
		}
	}
	jtag_execute_queue();

	return ERROR_OK;
}

int handle_runtest_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc < 1)
	{
		command_print(cmd_ctx, "usage: runtest <num_cycles>");
		return ERROR_OK;
	}

	jtag_add_runtest(strtol(args[0], NULL, 0), -1);
	jtag_execute_queue();

	return ERROR_OK;

}

int handle_statemove_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	enum tap_state state;

	state = -1;
	if (argc == 1)
	{
		for (state = 0; state < 16; state++)
		{
			if (strcmp(args[0], tap_state_strings[state]) == 0)
			{
				break;
			}
		}
	}

	jtag_add_statemove(state);
	jtag_execute_queue();

	return ERROR_OK;

}

int handle_irscan_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int i;
	scan_field_t *fields;
	
	if ((argc < 2) || (argc % 2))
	{
		command_print(cmd_ctx, "usage: irscan <device> <instr> [dev2] [instr2] ...");
		return ERROR_OK;
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

	jtag_add_ir_scan(argc / 2, fields, -1, NULL);
	jtag_execute_queue();

	for (i = 0; i < argc / 2; i++)
		free(fields[i].out_value);

	free (fields);

	return ERROR_OK;
}

int handle_drscan_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	scan_field_t *fields;
	int num_fields = 0;
	int field_count = 0;
	var_t *var;
	int i, j;
	
	if ((argc < 2) || (argc % 2))
	{
		command_print(cmd_ctx, "usage: drscan <device> <var> [dev2] [var2]");
		return ERROR_OK;
	}

	for (i = 0; i < argc; i+=2)
	{
		var = get_var_by_namenum(args[i+1]);
		if (var)
		{
			num_fields += var->num_fields;
		}
		else
		{
			command_print(cmd_ctx, "variable %s doesn't exist", args[i+1]);
			return ERROR_OK;
		}
	}

	fields = malloc(sizeof(scan_field_t) * num_fields);

	for (i = 0; i < argc; i+=2)
	{
		var = get_var_by_namenum(args[i+1]);
	
		for (j = 0; j < var->num_fields; j++)
		{
			fields[field_count].device = strtol(args[i], NULL, 0);
			fields[field_count].num_bits = var->fields[j].num_bits;
			fields[field_count].out_value = malloc(CEIL(var->fields[j].num_bits, 8));
			buf_set_u32(fields[field_count].out_value, 0, var->fields[j].num_bits, var->fields[j].value);
			fields[field_count].out_mask = NULL;
			fields[field_count].in_value = fields[field_count].out_value;
			fields[field_count].in_check_mask = NULL;
			fields[field_count].in_check_value = NULL;
			fields[field_count].in_handler = field_le_to_host;
			fields[field_count++].in_handler_priv = &(var->fields[j]);
		}
	}

	jtag_add_dr_scan(num_fields, fields, -1, NULL);
	jtag_execute_queue();
	
	for (i = 0; i < argc / 2; i++)
		free(fields[i].out_value);

	free(fields);

	return ERROR_OK;
}

int handle_verify_ircapture_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 0)
	{
		command_print(cmd_ctx, "verify Capture-IR is %s", (jtag_verify_capture_ir) ? "enabled": "disabled");
		return ERROR_OK;
	}
	
	if (strcmp(args[0], "enable") == 0)
	{
		jtag_verify_capture_ir = 1;
	}
	else if (strcmp(args[0], "disable") == 0)
	{
		jtag_verify_capture_ir = 0;
	}
	
	return ERROR_OK;
}
