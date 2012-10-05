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
extern struct rtos_type ChibiOS_rtos;

static struct rtos_type *rtos_types[] = {
	&ThreadX_rtos,
	&FreeRTOS_rtos,
	&eCos_rtos,
	&Linux_os,
	&ChibiOS_rtos,
	NULL
};

int rtos_thread_packet(struct connection *connection, char *packet, int packet_size);

int rtos_smp_init(struct target *target)
{
	if (target->rtos->type->smp_init)
		return target->rtos->type->smp_init(target);
	return ERROR_TARGET_INIT_FAILED;
}

static int os_alloc(struct target *target, struct rtos_type *ostype)
{
	struct rtos *os = target->rtos = calloc(1, sizeof(struct rtos));

	if (!os)
		return JIM_ERR;

	os->type = ostype;
	os->current_threadid = -1;
	os->current_thread = 0;
	os->symbols = NULL;
	os->target = target;

	/* RTOS drivers can override the packet handler in _create(). */
	os->gdb_thread_packet = rtos_thread_packet;

	return JIM_OK;
}

static void os_free(struct target *target)
{
	if (!target->rtos)
		return;

	if (target->rtos->symbols)
		free(target->rtos->symbols);

	free(target->rtos);
	target->rtos = NULL;
}

static int os_alloc_create(struct target *target, struct rtos_type *ostype)
{
	int ret = os_alloc(target, ostype);

	if (JIM_OK == ret) {
		ret = target->rtos->type->create(target);
		if (ret != JIM_OK)
			os_free(target);
	}

	return ret;
}

int rtos_create(Jim_GetOptInfo *goi, struct target *target)
{
	int x;
	char *cp;
	struct Jim_Obj *res;

	if (!goi->isconfigure && goi->argc != 0) {
		Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "NO PARAMS");
		return JIM_ERR;
	}

	os_free(target);

	Jim_GetOpt_String(goi, &cp, NULL);

	if (0 == strcmp(cp, "auto")) {
		/* Auto detect tries to look up all symbols for each RTOS,
		 * and runs the RTOS driver's _detect() function when GDB
		 * finds all symbols for any RTOS. See rtos_qsymbol(). */
		target->rtos_auto_detect = true;

		/* rtos_qsymbol() will iterate over all RTOSes. Allocate
		 * target->rtos here, and set it to the first RTOS type. */
		return os_alloc(target, rtos_types[0]);
	}

	for (x = 0; rtos_types[x]; x++)
		if (0 == strcmp(cp, rtos_types[x]->name))
			return os_alloc_create(target, rtos_types[x]);

	Jim_SetResultFormatted(goi->interp, "Unknown RTOS type %s, try one of: ", cp);
	res = Jim_GetResult(goi->interp);
	for (x = 0; rtos_types[x]; x++)
		Jim_AppendStrings(goi->interp, res, rtos_types[x]->name, ", ", NULL);
	Jim_AppendStrings(goi->interp, res, " or auto", NULL);

	return JIM_ERR;
}

int gdb_thread_packet(struct connection *connection, char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	if (target->rtos == NULL)
		return rtos_thread_packet(connection, packet, packet_size);	/* thread not
										 *found*/
	return target->rtos->gdb_thread_packet(connection, packet, packet_size);
}

static char *next_symbol(struct rtos *os, char *cur_symbol, uint64_t cur_addr)
{
	symbol_table_elem_t *s;

	if (!os->symbols)
		os->type->get_symbol_list_to_lookup(&os->symbols);

	if (!cur_symbol[0])
		return os->symbols[0].symbol_name;

	for (s = os->symbols; s->symbol_name; s++)
		if (!strcmp(s->symbol_name, cur_symbol)) {
			s->address = cur_addr;
			s++;
			return s->symbol_name;
		}

	return NULL;
}

/* rtos_qsymbol() processes and replies to all qSymbol packets from GDB.
 *
 * GDB sends a qSymbol:: packet (empty address, empty name) to notify
 * that it can now answer qSymbol::hexcodedname queries, to look up symbols.
 *
 * If the qSymbol packet has no address that means GDB did not find the
 * symbol, in which case auto-detect will move on to try the next RTOS.
 *
 * rtos_qsymbol() then calls the next_symbol() helper function, which
 * iterates over symbol names for the current RTOS until it finds the
 * symbol in the received GDB packet, and then returns the next entry
 * in the list of symbols.
 *
 * If GDB replied about the last symbol for the RTOS and the RTOS was
 * specified explicitly, then no further symbol lookup is done. When
 * auto-detecting, the RTOS driver _detect() function must return success.
 *
 * rtos_qsymbol() returns 1 if an RTOS has been detected, or 0 otherwise.
 */
int rtos_qsymbol(struct connection *connection, char *packet, int packet_size)
{
	int rtos_detected = 0;
	uint64_t addr;
	size_t reply_len;
	char reply[GDB_BUFFER_SIZE], cur_sym[GDB_BUFFER_SIZE / 2] = "", *next_sym;
	struct target *target = get_target_from_connection(connection);
	struct rtos *os = target->rtos;

	reply_len = sprintf(reply, "OK");

	if (!os)
		goto done;

	if (sscanf(packet, "qSymbol:%" SCNx64 ":", &addr))
		hex_to_str(cur_sym, strchr(packet + 8, ':') + 1);
	else if (target->rtos_auto_detect && !rtos_try_next(target))
		goto done;

	next_sym = next_symbol(os, cur_sym, addr);
	if (!next_sym) {
		if (!target->rtos_auto_detect) {
			rtos_detected = 1;
			goto done;
		}

		if (os->type->detect_rtos(target)) {
			LOG_OUTPUT("Auto-detected RTOS: %s\r\n", os->type->name);
			rtos_detected = 1;
			goto done;
		}

		if (!rtos_try_next(target))
			goto done;

		os->type->get_symbol_list_to_lookup(&os->symbols);

		next_sym = os->symbols[0].symbol_name;
		if (!next_sym)
			goto done;
	}

	if (8 + (strlen(next_sym) * 2) + 1 > sizeof(reply)) {
		LOG_OUTPUT("ERROR: RTOS symbol '%s' name is too long for GDB!", next_sym);
		goto done;
	}

	reply_len = sprintf(reply, "qSymbol:");
	reply_len += str_to_hex(reply + reply_len, next_sym);

done:
	gdb_put_packet(connection, reply, reply_len);
	return rtos_detected;
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
	struct rtos *os = target->rtos;
	struct rtos_type **type = rtos_types;

	if (!os)
		return 0;

	while (*type && os->type != *type)
		type++;

	if (!*type || !*(++type))
		return 0;

	os->type = *type;
	if (os->symbols) {
		free(os->symbols);
		os->symbols = NULL;
	}

	return 1;
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
