/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
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

#include "rtos.h"
#include "target/target.h"
#include "helper/log.h"
#include "server/gdb_server.h"

static void hex_to_str(char *dst, char *hex_src);

/* RTOSs */
extern struct rtos_type FreeRTOS_rtos;
extern struct rtos_type ThreadX_rtos;
extern struct rtos_type eCos_rtos;
extern struct rtos_type Linux_os;

static struct rtos_type *rtos_types[] = {
	&ThreadX_rtos,
	&FreeRTOS_rtos,
	&eCos_rtos,
	&Linux_os,
	NULL
};

int rtos_thread_packet(struct connection *connection, char *packet, int packet_size);

int rtos_smp_init(struct target *target)
{
	if (target->rtos->type->smp_init)
		return target->rtos->type->smp_init(target);
	return ERROR_TARGET_INIT_FAILED;
}

int rtos_create(Jim_GetOptInfo *goi, struct target *target)
{
	int x;
	char *cp;
	if (!goi->isconfigure) {
		if (goi->argc != 0) {
			if (goi->argc != 0) {
				Jim_WrongNumArgs(goi->interp,
					goi->argc, goi->argv,
					"NO PARAMS");
				return JIM_ERR;
			}

			Jim_SetResultString(goi->interp,
				target_type_name(target), -1);
		}
	}

	if (target->rtos)
		free((void *)(target->rtos));
						/*			e = Jim_GetOpt_String(goi,
						 * &cp, NULL); */
/*			target->rtos = strdup(cp); */

	Jim_GetOpt_String(goi, &cp, NULL);
	/* now does target type exist */

	if (0 == strcmp(cp, "auto")) {
		/* auto detection of RTOS */
		target->rtos_auto_detect = true;
		x = 0;
	} else {

		for (x = 0; rtos_types[x]; x++) {
			if (0 == strcmp(cp, rtos_types[x]->name)) {
				/* found */
				break;
			}
		}
		if (rtos_types[x] == NULL) {
			Jim_SetResultFormatted(goi->interp, "Unknown rtos type %s, try one of ",
				cp);
			for (x = 0; rtos_types[x]; x++) {
				if (rtos_types[x + 1]) {
					Jim_AppendStrings(goi->interp,
						Jim_GetResult(goi->interp),
						rtos_types[x]->name,
						", ", NULL);
				} else {
					Jim_AppendStrings(goi->interp,
						Jim_GetResult(goi->interp),
						" or ",
						rtos_types[x]->name, NULL);
				}
			}
			return JIM_ERR;
		}
	}
	/* Create it */
	target->rtos = calloc(1, sizeof(struct rtos));
	target->rtos->type = rtos_types[x];
	target->rtos->current_threadid = -1;
	target->rtos->current_thread = 0;
	target->rtos->symbols = NULL;
	target->rtos->target = target;
	/* put default thread handler in linux usecase it is overloaded*/
	target->rtos->gdb_thread_packet = rtos_thread_packet;

	if (0 != strcmp(cp, "auto"))
		target->rtos->type->create(target);

	return JIM_OK;
}

int gdb_thread_packet(struct connection *connection, char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	if (target->rtos == NULL)
		return rtos_thread_packet(connection, packet, packet_size);	/* thread not
										 *found*/
	return target->rtos->gdb_thread_packet(connection, packet, packet_size);
}
/* return -1 if no rtos defined, 0 if rtos and symbol to be asked, 1 if all
 * symbol have been asked*/
int rtos_qsymbol(struct connection *connection, char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	if (target->rtos != NULL) {
		int next_symbol_num = -1;
		if (target->rtos->symbols == NULL)
			target->rtos->type->get_symbol_list_to_lookup(&target->rtos->symbols);
		if (0 == strcmp("qSymbol::", packet))
			/* first query - */
			next_symbol_num = 0;
		else {
			int64_t value = 0;
			char *hex_name_str = malloc(strlen(packet));
			char *name_str;
			int symbol_num;

			char *found = strstr(packet, "qSymbol::");
			if (0 == found)
				sscanf(packet, "qSymbol:%" SCNx64 ":%s", &value, hex_name_str);
			else
				/* No value returned by GDB - symbol was not found*/
				sscanf(packet, "qSymbol::%s", hex_name_str);
			name_str = (char *) malloc(1 + strlen(hex_name_str) / 2);

			hex_to_str(name_str, hex_name_str);
			symbol_num = 0;
			while ((target->rtos->symbols[symbol_num].symbol_name != NULL) &&
					(0 != strcmp(target->rtos->symbols[symbol_num].symbol_name, name_str)))
				symbol_num++;

			if (target->rtos->symbols[symbol_num].symbol_name == NULL) {
				LOG_OUTPUT("ERROR: unknown symbol\r\n");
				gdb_put_packet(connection, "OK", 2);
				return ERROR_OK;
			}

			target->rtos->symbols[symbol_num].address = value;

			next_symbol_num = symbol_num+1;
			free(hex_name_str);
			free(name_str);
		}

		int symbols_done = 0;
		if (target->rtos->symbols[next_symbol_num].symbol_name == NULL) {
			if ((target->rtos_auto_detect == false) ||
					(1 == target->rtos->type->detect_rtos(target))) {
				/* Found correct RTOS or not autodetecting */
				if (target->rtos_auto_detect == true)
					LOG_OUTPUT("Auto-detected RTOS: %s\r\n",
						target->rtos->type->name);
				symbols_done = 1;
			} else {
				/* Auto detecting RTOS and currently not found */
				if (1 != rtos_try_next(target))
					/* No more RTOS's to try */
					symbols_done = 1;
				else {
					next_symbol_num = 0;
					target->rtos->type->get_symbol_list_to_lookup(
						&target->rtos->symbols);
				}
			}
		}
		if (symbols_done == 1)
			return symbols_done;
		else {
			char *symname = target->rtos->symbols[next_symbol_num].symbol_name;
			char qsymstr[] = "qSymbol:";
			char *opstring = (char *)malloc(sizeof(qsymstr)+strlen(symname)*2+1);
			char *posptr = opstring;
			posptr += sprintf(posptr, "%s", qsymstr);
			str_to_hex(posptr, symname);
			gdb_put_packet(connection, opstring, strlen(opstring));
			free(opstring);
			return symbols_done;
		}
	}
	gdb_put_packet(connection, "OK", 2);
	return -1;
}

int rtos_thread_packet(struct connection *connection, char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);

	if (strstr(packet, "qThreadExtraInfo,")) {
		if ((target->rtos != NULL) && (target->rtos->thread_details != NULL) &&
				(target->rtos->thread_count != 0)) {
			threadid_t threadid = 0;
			int found = -1;
			sscanf(packet, "qThreadExtraInfo,%" SCNx64, &threadid);

			if ((target->rtos != NULL) && (target->rtos->thread_details != NULL)) {
				int thread_num;
				for (thread_num = 0; thread_num < target->rtos->thread_count; thread_num++) {
					if (target->rtos->thread_details[thread_num].threadid == threadid) {
						if (target->rtos->thread_details[thread_num].exists)
							found = thread_num;
					}
				}
			}
			if (found == -1) {
				gdb_put_packet(connection, "E01", 3);	/* thread not found */
				return ERROR_OK;
			}

			struct thread_detail *detail = &target->rtos->thread_details[found];

			int str_size = 0;
			if (detail->display_str != NULL)
				str_size += strlen(detail->display_str);
			if (detail->thread_name_str != NULL)
				str_size += strlen(detail->thread_name_str);
			if (detail->extra_info_str != NULL)
				str_size += strlen(detail->extra_info_str);

			char *tmp_str = (char *) malloc(str_size + 7);
			char *tmp_str_ptr = tmp_str;

			if (detail->display_str != NULL)
				tmp_str_ptr += sprintf(tmp_str_ptr, "%s", detail->display_str);
			if (detail->thread_name_str != NULL) {
				if (tmp_str_ptr != tmp_str)
					tmp_str_ptr += sprintf(tmp_str_ptr, " : ");
				tmp_str_ptr += sprintf(tmp_str_ptr, "%s", detail->thread_name_str);
			}
			if (detail->extra_info_str != NULL) {
				if (tmp_str_ptr != tmp_str)
					tmp_str_ptr += sprintf(tmp_str_ptr, " : ");
				tmp_str_ptr +=
					sprintf(tmp_str_ptr, " : %s", detail->extra_info_str);
			}

			assert(strlen(tmp_str) ==
				(size_t) (tmp_str_ptr - tmp_str));

			char *hex_str = (char *) malloc(strlen(tmp_str)*2 + 1);
			str_to_hex(hex_str, tmp_str);

			gdb_put_packet(connection, hex_str, strlen(hex_str));
			free(hex_str);
			free(tmp_str);
			return ERROR_OK;

		}
		gdb_put_packet(connection, "", 0);
		return ERROR_OK;
	} else if (strstr(packet, "qSymbol")) {
		if (rtos_qsymbol(connection, packet, packet_size) == 1) {
			target->rtos_auto_detect = false;
			target->rtos->type->create(target);
			target->rtos->type->update_threads(target->rtos);
			/* No more symbols needed */
			gdb_put_packet(connection, "OK", 2);
		}
		return ERROR_OK;
	} else if (strstr(packet, "qfThreadInfo")) {
		int i;
		if ((target->rtos != NULL) && (target->rtos->thread_count != 0)) {

			char *out_str = (char *) malloc(17 * target->rtos->thread_count + 5);
			char *tmp_str = out_str;
			tmp_str += sprintf(tmp_str, "m");
			for (i = 0; i < target->rtos->thread_count; i++) {
				if (i != 0)
					tmp_str += sprintf(tmp_str, ",");
				tmp_str += sprintf(tmp_str, "%016" PRIx64,
						target->rtos->thread_details[i].threadid);
			}
			tmp_str[0] = 0;
			gdb_put_packet(connection, out_str, strlen(out_str));
		} else
			gdb_put_packet(connection, "", 0);

		return ERROR_OK;
	} else if (strstr(packet, "qsThreadInfo")) {
		gdb_put_packet(connection, "l", 1);
		return ERROR_OK;
	} else if (strstr(packet, "qAttached")) {
		gdb_put_packet(connection, "1", 1);
		return ERROR_OK;
	} else if (strstr(packet, "qOffsets")) {
		char offsets[] = "Text=0;Data=0;Bss=0";
		gdb_put_packet(connection, offsets, sizeof(offsets)-1);
		return ERROR_OK;
	} else if (strstr(packet, "qC")) {
		if (target->rtos != NULL) {
			char buffer[15];
			int size;
			size = snprintf(buffer, 15, "QC%08X", (int)target->rtos->current_thread);
			gdb_put_packet(connection, buffer, size);
		} else
			gdb_put_packet(connection, "QC0", 3);
		return ERROR_OK;
	} else if (packet[0] == 'T') {	/* Is thread alive? */
		threadid_t threadid;
		int found = -1;
		sscanf(packet, "T%" SCNx64, &threadid);
		if ((target->rtos != NULL) && (target->rtos->thread_details != NULL)) {
			int thread_num;
			for (thread_num = 0; thread_num < target->rtos->thread_count; thread_num++) {
				if (target->rtos->thread_details[thread_num].threadid == threadid) {
					if (target->rtos->thread_details[thread_num].exists)
						found = thread_num;
				}
			}
		}
		if (found != -1)
			gdb_put_packet(connection, "OK", 2);	/* thread alive */
		else
			gdb_put_packet(connection, "E01", 3);	/* thread not found */
		return ERROR_OK;
	} else if (packet[0] == 'H') {	/* Set current thread ( 'c' for step and continue, 'g' for
					 * all other operations ) */
		if ((packet[1] == 'g') && (target->rtos != NULL))
			sscanf(packet, "Hg%16" SCNx64, &target->rtos->current_threadid);
		gdb_put_packet(connection, "OK", 2);
		return ERROR_OK;
	}

	return GDB_THREAD_PACKET_NOT_CONSUMED;
}

int rtos_get_gdb_reg_list(struct connection *connection)
{
	struct target *target = get_target_from_connection(connection);
	int64_t current_threadid = target->rtos->current_threadid;
	if ((target->rtos != NULL) && (current_threadid != -1) &&
			(current_threadid != 0) &&
			((current_threadid != target->rtos->current_thread) ||
			(target->smp))) {	/* in smp several current thread are possible */
		char *hex_reg_list;
		target->rtos->type->get_thread_reg_list(target->rtos,
			current_threadid,
			&hex_reg_list);

		if (hex_reg_list != NULL) {
			gdb_put_packet(connection, hex_reg_list, strlen(hex_reg_list));
			free(hex_reg_list);
			return ERROR_OK;
		}
	}
	return ERROR_FAIL;
}

int rtos_generic_stack_read(struct target *target,
	const struct rtos_register_stacking *stacking,
	int64_t stack_ptr,
	char **hex_reg_list)
{
	int list_size = 0;
	char *tmp_str_ptr;
	int64_t new_stack_ptr;
	int i;
	int retval;

	if (stack_ptr == 0) {
		LOG_OUTPUT("Error: null stack pointer in thread\r\n");
		return -5;
	}
	/* Read the stack */
	uint8_t *stack_data = (uint8_t *) malloc(stacking->stack_registers_size);
	uint32_t address = stack_ptr;

	if (stacking->stack_growth_direction == 1)
		address -= stacking->stack_registers_size;
	retval = target_read_buffer(target, address, stacking->stack_registers_size, stack_data);
	if (retval != ERROR_OK) {
		LOG_OUTPUT("Error reading stack frame from FreeRTOS thread\r\n");
		return retval;
	}
#if 0
		LOG_OUTPUT("Stack Data :");
		for (i = 0; i < stacking->stack_registers_size; i++)
			LOG_OUTPUT("%02X", stack_data[i]);
		LOG_OUTPUT("\r\n");
#endif
	for (i = 0; i < stacking->num_output_registers; i++)
		list_size += stacking->register_offsets[i].width_bits/8;
	*hex_reg_list = (char *)malloc(list_size*2 + 1);
	tmp_str_ptr = *hex_reg_list;
	new_stack_ptr = stack_ptr - stacking->stack_growth_direction *
		stacking->stack_registers_size;
	if (stacking->stack_alignment != 0) {
		/* Align new stack pointer to x byte boundary */
		new_stack_ptr =
			(new_stack_ptr & (~((int64_t) stacking->stack_alignment - 1))) +
			((stacking->stack_growth_direction == -1) ? stacking->stack_alignment : 0);
	}
	for (i = 0; i < stacking->num_output_registers; i++) {
		int j;
		for (j = 0; j < stacking->register_offsets[i].width_bits/8; j++) {
			if (stacking->register_offsets[i].offset == -1)
				tmp_str_ptr += sprintf(tmp_str_ptr, "%02x", 0);
			else if (stacking->register_offsets[i].offset == -2)
				tmp_str_ptr += sprintf(tmp_str_ptr, "%02x",
						((uint8_t *)&new_stack_ptr)[j]);
			else
				tmp_str_ptr += sprintf(tmp_str_ptr, "%02x",
						stack_data[stacking->register_offsets[i].offset + j]);
		}
	}
/*	LOG_OUTPUT("Output register string: %s\r\n", *hex_reg_list); */
	return ERROR_OK;
}

int rtos_try_next(struct target *target)
{
	int x;

	if (target->rtos == NULL)
		return -1;

	for (x = 0; rtos_types[x]; x++) {
		if (target->rtos->type == rtos_types[x]) {
			/* found */
			if (rtos_types[x+1] != NULL) {
				target->rtos->type = rtos_types[x+1];
				if (target->rtos->symbols != NULL)
					free(target->rtos->symbols);
				return 1;
			} else {
				/* No more rtos types */
				return 0;
			}

		}
	}
	return 0;

}

static void hex_to_str(char *dst, char *hex_src)
{
	int src_pos = 0;
	int dst_pos = 0;

	while (hex_src[src_pos] != '\x00') {
		char hex_char = hex_src[src_pos];
		char hex_digit_val =
			(hex_char >=
			 'a') ? hex_char-'a'+
			10 : (hex_char >= 'A') ? hex_char-'A'+10 : hex_char-'0';
		if (0 == (src_pos & 0x01)) {
			dst[dst_pos] = hex_digit_val;
			dst[dst_pos+1] = 0;
		} else {
			((unsigned char *)dst)[dst_pos] <<= 4;
			((unsigned char *)dst)[dst_pos] += hex_digit_val;
			dst_pos++;
		}
		src_pos++;
	}

}

int str_to_hex(char *hex_dst, char *src)
{
	char *posptr = hex_dst;
	unsigned i;
	for (i = 0; i < strlen(src); i++)
		posptr += sprintf(posptr, "%02x", (unsigned char)src[i]);
	return posptr - hex_dst;
}

int rtos_update_threads(struct target *target)
{
	if ((target->rtos != NULL) && (target->rtos->type != NULL))
		target->rtos->type->update_threads(target->rtos);
	return ERROR_OK;
}
