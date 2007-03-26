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

#include "gdb_server.h"

#include "server.h"
#include "log.h"
#include "binarybuffer.h"
#include "breakpoints.h"

#define __USE_GNU
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>

#if 0
#define _DEBUG_GDB_IO_
#endif

static unsigned short gdb_port;

int gdb_last_signal(target_t *target)
{
	switch (target->debug_reason)
	{
		case DBG_REASON_DBGRQ:
			return 0x2; /* SIGINT */
		case DBG_REASON_BREAKPOINT:
		case DBG_REASON_WATCHPOINT:
		case DBG_REASON_WPTANDBKPT:
			return 0x05; /* SIGTRAP */
		case DBG_REASON_SINGLESTEP:
			return 0x05; /* SIGTRAP */
		case DBG_REASON_NOTHALTED:
			return 0x0; /* no signal... shouldn't happen */
		default:
			ERROR("BUG: undefined debug reason");
			exit(-1);
	}
}

int gdb_get_char(connection_t *connection, int* next_char)
{
	gdb_connection_t *gdb_con = connection->priv;
	char *debug_buffer;
	
	if (gdb_con->buf_cnt-- > 0)
	{
		*next_char = *(gdb_con->buf_p++);
		if (gdb_con->buf_cnt > 0)
			connection->input_pending = 1;
		else
			connection->input_pending = 0;
		
#ifdef _DEBUG_GDB_IO_
		DEBUG("returned char '%c' (0x%2.2x)", *next_char, *next_char);
#endif
		
		return ERROR_OK;
	}

	while ((gdb_con->buf_cnt = read_socket(connection->fd, gdb_con->buffer, GDB_BUFFER_SIZE)) <= 0)
	{
		if (gdb_con->buf_cnt == 0)
			return ERROR_SERVER_REMOTE_CLOSED;
		
#ifdef _WIN32
		errno = WSAGetLastError();

		switch(errno)
		{
			case WSAEWOULDBLOCK:
				usleep(1000);
				break;
			case WSAECONNABORTED:
				return ERROR_SERVER_REMOTE_CLOSED;
			default:
				ERROR("read: %d", errno);
				exit(-1);
		}
#else
		switch(errno)
		{
			case EAGAIN:
				usleep(1000);
				break;
			case ECONNABORTED:
				return ERROR_SERVER_REMOTE_CLOSED;
			case ECONNRESET:
				return ERROR_SERVER_REMOTE_CLOSED;
			default:
				ERROR("read: %s", strerror(errno));
				exit(-1);
		}
#endif
	}
	
	debug_buffer = malloc(gdb_con->buf_cnt + 1);
	memcpy(debug_buffer, gdb_con->buffer, gdb_con->buf_cnt);
	debug_buffer[gdb_con->buf_cnt] = 0;
	DEBUG("received '%s'", debug_buffer);
	free(debug_buffer);

	gdb_con->buf_p = gdb_con->buffer;
	gdb_con->buf_cnt--;
	*next_char = *(gdb_con->buf_p++);
	if (gdb_con->buf_cnt > 0)
		connection->input_pending = 1;
	else
		connection->input_pending = 0;	
#ifdef _DEBUG_GDB_IO_
		DEBUG("returned char '%c' (0x%2.2x)", *next_char, *next_char);
#endif
	
	return ERROR_OK;
}

int gdb_putback_char(connection_t *connection, int last_char)
{
	gdb_connection_t *gdb_con = connection->priv;
	
	if (gdb_con->buf_p > gdb_con->buffer)
	{
		*(--gdb_con->buf_p) = last_char;
		gdb_con->buf_cnt++;
	}
	else
	{
		ERROR("BUG: couldn't put character back"); 	
	}
	
	return ERROR_OK;
}

int gdb_put_packet(connection_t *connection, char *buffer, int len)
{
	int i;
	unsigned char my_checksum = 0;
	char checksum[3];
	char *debug_buffer;
	int reply;
	int retval;
	gdb_connection_t *gdb_con = connection->priv;

	for (i = 0; i < len; i++)
		my_checksum += buffer[i];
	
	while (1)
	{
			
		debug_buffer = malloc(len + 1);
		memcpy(debug_buffer, buffer, len);
		debug_buffer[len] = 0;
		DEBUG("sending packet '$%s#%2.2x'", debug_buffer, my_checksum);
		free(debug_buffer);
		
		write_socket(connection->fd, "$", 1);
		if (len > 0)
			write_socket(connection->fd, buffer, len);
		write_socket(connection->fd, "#", 1);
	
		snprintf(checksum, 3, "%2.2x", my_checksum);
	
		write_socket(connection->fd, checksum, 2);

		if ((retval = gdb_get_char(connection, &reply)) != ERROR_OK)
			return retval;

		if (reply == '+')
			break;
		else if (reply == '-')
			WARNING("negative reply, retrying");
		else if (reply == 0x3)
		{
			gdb_con->ctrl_c = 1;
			if ((retval = gdb_get_char(connection, &reply)) != ERROR_OK)
				return retval;
			if (reply == '+')
				break;
			else if (reply == '-')
				WARNING("negative reply, retrying");
			else
			{
				ERROR("unknown character 0x%2.2x in reply, dropping connection", reply);
				return ERROR_SERVER_REMOTE_CLOSED;
			}
		}
		else
		{
			ERROR("unknown character 0x%2.2x in reply, dropping connection", reply);
			return ERROR_SERVER_REMOTE_CLOSED;
		}
	}
	
	return ERROR_OK;
}

int gdb_get_packet(connection_t *connection, char *buffer, int *len)
{
	int character;
	int count = 0;
	int retval;
	int first_char = 0;
	int packet_type = '\0';
	char checksum[3];
	unsigned char my_checksum = 0;
	gdb_connection_t *gdb_con = connection->priv;

	while (1)
	{
		do
		{
			if ((retval = gdb_get_char(connection, &character)) != ERROR_OK)
				return retval;

			DEBUG("character: '%c'", character);

			switch (character)
			{
				case '$':
					break;
				case '+':
					WARNING("acknowledgment received, but no packet pending");
					break;
				case '-':
					WARNING("negative acknowledgment, but no packet pending");
					break;
				case 0x3:
					gdb_con->ctrl_c = 1;
					*len = 0;
					return ERROR_OK;
				default:
					WARNING("ignoring character 0x%x", character);
					break;
			}
		} while (character != '$');

		my_checksum = 0;
			
		do
		{
			if ((retval = gdb_get_char(connection, &character)) != ERROR_OK)
				return retval;
			
			if( !first_char ) {
				packet_type = character;
				first_char = 1;	
			}
			
			if( packet_type == 'X' )
			{
				switch (character)
				{
					case '#':
						break;
					case 0x7d:
						/* data transmitted in binary mode (X packet)
						* uses 0x7d as escape character */
						my_checksum += character & 0xff;
						gdb_get_char(connection, &character);
						my_checksum += character & 0xff;
						buffer[count++] = (character ^ 0x20) & 0xff;
						if (count > *len)
						{
							ERROR("packet buffer too small");
							return ERROR_GDB_BUFFER_TOO_SMALL;
						}
						break;
					default:
						buffer[count++] = character & 0xff;
						my_checksum += character & 0xff;
						if (count > *len)
						{
							ERROR("packet buffer too small");
							return ERROR_GDB_BUFFER_TOO_SMALL;
						}
						break;
				}
			}
			else
			{
				switch (character)
				{
					case '#':
						break;
					case 0x3:
						gdb_con->ctrl_c = 1;
						break;
					default:
						buffer[count++] = character & 0xff;
						my_checksum += character & 0xff;
						if (count > *len)
						{
							ERROR("packet buffer too small");
							return ERROR_GDB_BUFFER_TOO_SMALL;
						}
						break;
				}
			}
		} while (character != '#');

		*len = count;
		
		if ((retval = gdb_get_char(connection, &character)) != ERROR_OK)
			return retval;
		checksum[0] = character;
		if ((retval = gdb_get_char(connection, &character)) != ERROR_OK)
			return retval;
		checksum[1] = character;
		checksum[2] = 0;
		
		if (my_checksum == strtoul(checksum, NULL, 16))
		{
			write_socket(connection->fd, "+", 1);
			break;
		}

		WARNING("checksum error, requesting retransmission");
		write_socket(connection->fd, "-", 1);
	}

	return ERROR_OK;
}

int gdb_output(struct command_context_s *context, char* line)
{
	connection_t *connection = context->output_handler_priv;
	char *hex_buffer;
	int i, bin_size;

	bin_size = strlen(line);
	
	hex_buffer = malloc(bin_size*2 + 4);

	hex_buffer[0] = 'O';
	for (i=0; i<bin_size; i++)
		snprintf(hex_buffer + 1 + i*2, 3, "%2.2x", line[i]);
	hex_buffer[bin_size*2+1] = '0';
	hex_buffer[bin_size*2+2] = 'a';
	hex_buffer[bin_size*2+3] = 0x0;

	gdb_put_packet(connection, hex_buffer, bin_size*2 + 3);

	free(hex_buffer);
	return ERROR_OK;
}

int gdb_target_callback_event_handler(struct target_s *target, enum target_event event, void *priv)
{
	connection_t *connection = priv;
	gdb_connection_t *gdb_connection = connection->priv;
	char sig_reply[4];
	int signal;
	
	switch (event)
	{
		case TARGET_EVENT_HALTED:
			if (gdb_connection->frontend_state == TARGET_RUNNING)
			{
				if (gdb_connection->ctrl_c)
				{
					signal = 0x2;
					gdb_connection->ctrl_c = 0;
				}
				else
				{
					signal = gdb_last_signal(target);
				}
				
				snprintf(sig_reply, 4, "T%2.2x", signal);
				gdb_put_packet(connection, sig_reply, 3);
				gdb_connection->frontend_state = TARGET_HALTED;
			}
			break;
		case TARGET_EVENT_RESUMED:
			if (gdb_connection->frontend_state == TARGET_HALTED)
			{
				gdb_connection->frontend_state = TARGET_RUNNING;
			}
			break;
		default:
			break;
	}

	return ERROR_OK;
}

int gdb_new_connection(connection_t *connection)
{
	gdb_connection_t *gdb_connection = malloc(sizeof(gdb_connection_t));
	gdb_service_t *gdb_service = connection->service->priv;
	int retval;
	int initial_ack;
	
	connection->priv = gdb_connection;
	
	/* initialize gdb connection information */
	gdb_connection->buf_p = gdb_connection->buffer;
	gdb_connection->buf_cnt = 0;
	gdb_connection->ctrl_c = 0;
	gdb_connection->frontend_state = TARGET_HALTED;
	
	/* output goes through gdb connection */
	command_set_output_handler(connection->cmd_ctx, gdb_output, connection);
	
	/* register callback to be informed about target events */
	target_register_event_callback(gdb_target_callback_event_handler, connection);	
	
	/* a gdb session just attached, put the target in halt mode */
	if (((retval = gdb_service->target->type->halt(gdb_service->target)) != ERROR_OK) &&
			 (retval != ERROR_TARGET_ALREADY_HALTED))
	{
		ERROR("error when trying to halt target");
		exit(-1);
	}
	
	while (gdb_service->target->state != TARGET_HALTED)
	{
		gdb_service->target->type->poll(gdb_service->target);
	}
	
	/* remove the initial ACK from the incoming buffer */
	if ((retval = gdb_get_char(connection, &initial_ack)) != ERROR_OK)
		return retval;
		
	if (initial_ack != '+')
		gdb_putback_char(connection, initial_ack);
		
	return ERROR_OK;
}

int gdb_connection_closed(connection_t *connection)
{
	if (connection->priv)
		free(connection->priv);
	else
		ERROR("BUG: connection->priv == NULL");
	
	target_unregister_event_callback(gdb_target_callback_event_handler, connection);

	return ERROR_OK;
}

void gdb_send_error(connection_t *connection, u8 the_error)
{
    char err[4];
    snprintf(err, 4, "E%2.2X", the_error );
    gdb_put_packet(connection, err, 3);
}

int gdb_last_signal_packet(connection_t *connection, target_t *target, char* packet, int packet_size)
{
	char sig_reply[4];
	int signal;
	
	signal = gdb_last_signal(target);

	snprintf(sig_reply, 4, "S%2.2x", signal);
	gdb_put_packet(connection, sig_reply, 3);
	
	return ERROR_OK;
}

void gdb_str_to_target(target_t *target, char *str, char *tstr)
{
	int str_len = strlen(str);
	int i;
	
	if (str_len % 2)
	{
		ERROR("BUG: gdb value with uneven number of characters encountered: %s", str);
		exit(-1);
	}
	
	if (target->endianness == TARGET_LITTLE_ENDIAN)
	{
		for (i = 0; i < str_len; i+=2)
		{
			tstr[str_len - i - 1] = str[i + 1];
			tstr[str_len - i - 2] = str[i];
		}
	}
	else
	{
		for (i = 0; i < str_len; i++)
		{
			tstr[i] = str[i];
		}
	}	
}

void gdb_target_to_str(target_t *target, char *tstr, char *str)
{
	int str_len = strlen(tstr);
	int i;

	if (str_len % 2)
	{
		ERROR("BUG: gdb value with uneven number of characters encountered");
		exit(-1);
	}
	
	if (target->endianness == TARGET_LITTLE_ENDIAN)
	{
		for (i = 0; i < str_len; i+=2)
		{
			str[str_len - i - 1] = tstr[i + 1];
			str[str_len - i - 2] = tstr[i];
		}
	}
	else
	{
		for (i = 0; i < str_len; i++)
		{
			str[i] = tstr[i];
		}
	}	
}

int gdb_get_registers_packet(connection_t *connection, target_t *target, char* packet, int packet_size)
{
	reg_t **reg_list;
	int reg_list_size;
	int retval;
	int reg_packet_size = 0;
	char *reg_packet;
	char *reg_packet_p;
	int i;
	
	DEBUG("-");

	if ((retval = target->type->get_gdb_reg_list(target, &reg_list, &reg_list_size)) != ERROR_OK)
	{
		switch (retval)
		{
			case ERROR_TARGET_NOT_HALTED:
				ERROR("gdb requested registers but we're not halted, dropping connection");
				return ERROR_SERVER_REMOTE_CLOSED;
			default:
				/* this is a bug condition - get_gdb_reg_list() may not return any other error */
				ERROR("BUG: unexpected error returned by get_gdb_reg_list()");
				exit(-1);
		}
	}

	for (i = 0; i < reg_list_size; i++)
	{
		reg_packet_size += reg_list[i]->size;
	}
	
	reg_packet = malloc(CEIL(reg_packet_size, 8) * 2);
	reg_packet_p = reg_packet;
	
	for (i = 0; i < reg_list_size; i++)
	{
		char *hex_buf = buf_to_str(reg_list[i]->value, reg_list[i]->size, 16);
		DEBUG("hex_buf: %s", hex_buf);
		gdb_str_to_target(target, hex_buf, reg_packet_p);
		reg_packet_p += CEIL(reg_list[i]->size, 8) * 2;
		free(hex_buf);
	}

	reg_packet_p = strndup(reg_packet, CEIL(reg_packet_size, 8) * 2);
	DEBUG("reg_packet: %s", reg_packet_p);
	free(reg_packet_p);
	
	gdb_put_packet(connection, reg_packet, CEIL(reg_packet_size, 8) * 2);
	free(reg_packet);
	
	free(reg_list);
	
	return ERROR_OK;
}

int gdb_set_registers_packet(connection_t *connection, target_t *target, char *packet, int packet_size)
{
	int i;
	reg_t **reg_list;
	int reg_list_size;
	int retval;
	char *packet_p;
	
	DEBUG("-");

	/* skip command character */
	packet++;
	packet_size--;

	if (packet_size % 2)
	{
		WARNING("GDB set_registers packet with uneven characters received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	if ((retval = target->type->get_gdb_reg_list(target, &reg_list, &reg_list_size)) != ERROR_OK)
	{
		switch (retval)
		{
			case ERROR_TARGET_NOT_HALTED:
				ERROR("gdb tried to registers but we're not halted, dropping connection");
				return ERROR_SERVER_REMOTE_CLOSED;
			default:
				/* this is a bug condition - get_gdb_reg_list() may not return any other error */
				ERROR("BUG: unexpected error returned by get_gdb_reg_list()");
				exit(-1);
		}
	}

	packet_p = packet;
	for (i = 0; i < reg_list_size; i++)
	{
		u8 *bin_buf;
		char *hex_buf;
		reg_arch_type_t *arch_type;
		
		/* convert from GDB-string (target-endian) to hex-string (big-endian) */
		hex_buf = malloc(CEIL(reg_list[i]->size, 8) * 2);
		gdb_target_to_str(target, packet_p, hex_buf);
		
		/* convert hex-string to binary buffer */
		bin_buf = malloc(CEIL(reg_list[i]->size, 8));
		str_to_buf(hex_buf, CEIL(reg_list[i]->size, 8) * 2, bin_buf, reg_list[i]->size, 16);

		/* get register arch_type, and call set method */	
		arch_type = register_get_arch_type(reg_list[i]->arch_type);
		if (arch_type == NULL)
		{
			ERROR("BUG: encountered unregistered arch type");
			exit(-1);
		}
		arch_type->set(reg_list[i], bin_buf);

		/* advance packet pointer */		
		packet_p += (CEIL(reg_list[i]->size, 8) * 2);
		
		free(bin_buf);
		free(hex_buf);
	}
	
	/* free reg_t *reg_list[] array allocated by get_gdb_reg_list */ 
	free(reg_list);

	gdb_put_packet(connection, "OK", 2);
	
	return ERROR_OK;
}

int gdb_get_register_packet(connection_t *connection, target_t *target, char *packet, int packet_size)
{
	char *reg_packet;
	int reg_num = strtoul(packet + 1, NULL, 16);
	reg_t **reg_list;
	int reg_list_size;
	int retval;
	char *hex_buf;
	
	DEBUG("-");
	
	if ((retval = target->type->get_gdb_reg_list(target, &reg_list, &reg_list_size)) != ERROR_OK)
	{
		switch (retval)
		{
			case ERROR_TARGET_NOT_HALTED:
				ERROR("gdb requested registers but we're not halted, dropping connection");
				return ERROR_SERVER_REMOTE_CLOSED;
			default:
				/* this is a bug condition - get_gdb_reg_list() may not return any other error */
				ERROR("BUG: unexpected error returned by get_gdb_reg_list()");
				exit(-1);
		}
	}
	
	if (reg_list_size <= reg_num)
	{
		ERROR("gdb requested a non-existing register");
		exit(-1);
	}

	reg_packet = malloc(CEIL(reg_list[reg_num]->size, 8) * 2);

	hex_buf = buf_to_str(reg_list[reg_num]->value, reg_list[reg_num]->size, 16);
	
	gdb_str_to_target(target, hex_buf, reg_packet);
	
	gdb_put_packet(connection, reg_packet, CEIL(reg_list[reg_num]->size, 8) * 2);
	
	free(reg_list);
	free(reg_packet);
	free(hex_buf);
	
	return ERROR_OK;
}

int gdb_set_register_packet(connection_t *connection, target_t *target, char *packet, int packet_size)
{
	char *separator;
	char *hex_buf;
	u8 *bin_buf;
	int reg_num = strtoul(packet + 1, &separator, 16);
	reg_t **reg_list;
	int reg_list_size;
	int retval;
	reg_arch_type_t *arch_type;
	
	DEBUG("-");
	
	if ((retval = target->type->get_gdb_reg_list(target, &reg_list, &reg_list_size)) != ERROR_OK)
	{
		switch (retval)
		{
			case ERROR_TARGET_NOT_HALTED:
				ERROR("gdb tried to set a register but we're not halted, dropping connection");
				return ERROR_SERVER_REMOTE_CLOSED;
			default:
				/* this is a bug condition - get_gdb_reg_list() may not return any other error */
				ERROR("BUG: unexpected error returned by get_gdb_reg_list()");
				exit(-1);
		}
	}
	
	if (reg_list_size < reg_num)
	{
		ERROR("gdb requested a non-existing register");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	if (*separator != '=')
	{
		ERROR("GDB 'set register packet', but no '=' following the register number");
		return ERROR_SERVER_REMOTE_CLOSED;
	}
	
	/* convert from GDB-string (target-endian) to hex-string (big-endian) */
	hex_buf = malloc(CEIL(reg_list[reg_num]->size, 8) * 2);
	gdb_target_to_str(target, separator + 1, hex_buf);
	
	/* convert hex-string to binary buffer */
	bin_buf = malloc(CEIL(reg_list[reg_num]->size, 8));
	str_to_buf(hex_buf, CEIL(reg_list[reg_num]->size, 8) * 2, bin_buf, reg_list[reg_num]->size, 16);

	/* get register arch_type, and call set method */	
	arch_type = register_get_arch_type(reg_list[reg_num]->arch_type);
	if (arch_type == NULL)
	{
		ERROR("BUG: encountered unregistered arch type");
		exit(-1);
	}
	arch_type->set(reg_list[reg_num], bin_buf);

	gdb_put_packet(connection, "OK", 2);

	free(bin_buf);
	free(hex_buf);
	free(reg_list);

	return ERROR_OK;
}

int gdb_memory_packet_error(connection_t *connection, int retval)
{
	switch (retval)
	{
		case ERROR_TARGET_NOT_HALTED:
			ERROR("gdb tried to read memory but we're not halted, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
			break;
		case ERROR_TARGET_DATA_ABORT:
			gdb_send_error(connection, EIO);
			break;
		case ERROR_TARGET_TRANSLATION_FAULT:
			gdb_send_error(connection, EFAULT);
			break;
		case ERROR_TARGET_UNALIGNED_ACCESS:
			gdb_send_error(connection, EFAULT);
			break;
		default:
			ERROR("BUG: unexpected error %i", retval);
			exit(-1);
	}
	
	return ERROR_OK;
}

int gdb_read_memory_packet(connection_t *connection, target_t *target, char *packet, int packet_size)
{
	char *separator;
	u32 addr = 0;
	u32 len = 0;

	u8 *buffer;
	char *hex_buffer;

	int i;
	int retval;

	/* skip command character */
	packet++;

	addr = strtoul(packet, &separator, 16);
	
	if (*separator != ',')
	{
		ERROR("incomplete read memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator+1, NULL, 16);

	buffer = malloc(len);

	DEBUG("addr: 0x%8.8x, len: 0x%8.8x", addr, len);

	switch (len)
	{
		case 4:
			if ((addr % 4) == 0)
				retval = target->type->read_memory(target, addr, 4, 1, buffer);
			else
				retval = target->type->read_memory(target, addr, 1, len, buffer);
			break;
		case 2:
			if ((addr % 2) == 0)
				retval = target->type->read_memory(target, addr, 2, 1, buffer);
			else
				retval = target->type->read_memory(target, addr, 1, len, buffer);
			break;
		default:
			if (((addr % 4) == 0) && ((len % 4) == 0))
				retval = target->type->read_memory(target, addr, 4, len / 4, buffer);
			else
				retval = target->type->read_memory(target, addr, 1, len, buffer);
	}

	if (retval == ERROR_OK)
	{
		hex_buffer = malloc(len * 2 + 1);
		
		for (i=0; i<len; i++)
			snprintf(hex_buffer + 2*i, 3, "%2.2x", buffer[i]);
	
		gdb_put_packet(connection, hex_buffer, len * 2);
	
		free(hex_buffer);
	}
	else
	{
		if ((retval = gdb_memory_packet_error(connection, retval)) != ERROR_OK)
			return retval; 
	}

	free(buffer);
	
	return ERROR_OK;
}

int gdb_write_memory_packet(connection_t *connection, target_t *target, char *packet, int packet_size)
{
	char *separator;
	u32 addr = 0;
	u32 len = 0;

	u8 *buffer;

	int i;
	int retval;

	/* skip command character */
	packet++;

	addr = strtoul(packet, &separator, 16);
	
	if (*separator != ',')
	{
		ERROR("incomplete write memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator+1, &separator, 16);

	if (*(separator++) != ':')
	{
		ERROR("incomplete write memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	buffer = malloc(len);

	DEBUG("addr: 0x%8.8x, len: 0x%8.8x", addr, len);

	for (i=0; i<len; i++)
	{
		u32 tmp;
		sscanf(separator + 2*i, "%2x", &tmp);
		buffer[i] = tmp;
	}

	retval = ERROR_OK;
	switch (len)
	{
		/* handle sized writes */
		case 4:
			if ((addr % 4) == 0)
				retval = target->type->write_memory(target, addr, 4, 1, buffer);
			else
				retval = target->type->write_memory(target, addr, 1, len, buffer);
			break;
		case 2:
			if ((addr % 2) == 0)
				retval = target->type->write_memory(target, addr, 2, 1, buffer);
			else
				retval = target->type->write_memory(target, addr, 1, len, buffer);
			break;
		case 3:
		case 1:
			retval = target->type->write_memory(target, addr, 1, len, buffer);
			break;
		/* handle bulk writes */
		default:
			retval = target_write_buffer(target, addr, len, buffer);
			break;
	}

	if (retval == ERROR_OK)
	{
		gdb_put_packet(connection, "OK", 2);
	}
	else
	{
		if ((retval = gdb_memory_packet_error(connection, retval)) != ERROR_OK)
			return retval; 
	}
	
	free(buffer);
	
	return ERROR_OK;
}

int gdb_write_memory_binary_packet(connection_t *connection, target_t *target, char *packet, int packet_size)
{
	char *separator;
	u32 addr = 0;
	u32 len = 0;

	u8 *buffer;
	int retval;

	/* skip command character */
	packet++;

	addr = strtoul(packet, &separator, 16);
	
	if (*separator != ',')
	{
		ERROR("incomplete write memory binary packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator+1, &separator, 16);

	if (*(separator++) != ':')
	{
		ERROR("incomplete write memory binary packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	retval = ERROR_OK;
	if( len ) {
		
		buffer = malloc(len);
	
		DEBUG("addr: 0x%8.8x, len: 0x%8.8x", addr, len);
		
		memcpy( buffer, separator, len );
	
		switch (len)
		{
			case 4:
				if ((addr % 4) == 0)
					retval = target->type->write_memory(target, addr, 4, 1, buffer);
				else
					retval = target->type->write_memory(target, addr, 1, len, buffer);
				break;
			case 2:
				if ((addr % 2) == 0)
					retval = target->type->write_memory(target, addr, 2, 1, buffer);
				else
					retval = target->type->write_memory(target, addr, 1, len, buffer);
				break;
			case 3:
			case 1:
				retval = target->type->write_memory(target, addr, 1, len, buffer);
				break;
			default:
				retval = target_write_buffer(target, addr, len, buffer);
				break;
		}
		
		free(buffer);
	}

	if (retval == ERROR_OK)
	{
		gdb_put_packet(connection, "OK", 2);
	}
	else
	{
		if ((retval = gdb_memory_packet_error(connection, retval)) != ERROR_OK)
			return retval; 
	}
	
	return ERROR_OK;
}

void gdb_step_continue_packet(connection_t *connection, target_t *target, char *packet, int packet_size)
{
	int current = 0;
	u32 address = 0x0;

	DEBUG("-");

	if (packet_size > 1)
	{
		packet[packet_size] = 0;
		address = strtoul(packet + 1, NULL, 16);
	}
	else
	{
		current = 1;
	}

	if (packet[0] == 'c')
	{
		DEBUG("continue");
		target->type->resume(target, current, address, 0, 0); /* resume at current address, don't handle breakpoints, not debugging */
	}
	else if (packet[0] == 's')
	{
		DEBUG("step");
		target->type->step(target, current, address, 0); /* step at current or address, don't handle breakpoints */
	}
}

int gdb_bp_wp_packet_error(connection_t *connection, int retval)
{
	switch (retval)
	{
		case ERROR_TARGET_NOT_HALTED:
			ERROR("gdb tried to set a breakpoint but we're not halted, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
			break;
		case ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
			gdb_send_error(connection, EBUSY);
			break;
		default:
			ERROR("BUG: unexpected error %i", retval);
			exit(-1);
	}
	
	return ERROR_OK;
}

int gdb_breakpoint_watchpoint_packet(connection_t *connection, target_t *target, char *packet, int packet_size)
{
	int type;
	enum breakpoint_type bp_type = BKPT_SOFT /* dummy init to avoid warning */;
	enum watchpoint_rw wp_type;
	u32 address;
	u32 size;
	char *separator;
	int retval;

	DEBUG("-");

	type = strtoul(packet + 1, &separator, 16);
	
	if (type == 0)	/* memory breakpoint */
		bp_type = BKPT_SOFT;
	else if (type == 1) /* hardware breakpoint */
		bp_type = BKPT_HARD;
	else if (type == 2) /* write watchpoint */
		wp_type = WPT_WRITE;
	else if (type == 3) /* read watchpoint */
		wp_type = WPT_READ;
	else if (type == 4) /* access watchpoint */
		wp_type = WPT_ACCESS;
		
	if (*separator != ',')
	{
		ERROR("incomplete breakpoint/watchpoint packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	address = strtoul(separator+1, &separator, 16);

	if (*separator != ',')
	{
		ERROR("incomplete breakpoint/watchpoint packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	size = strtoul(separator+1, &separator, 16);

	switch (type)
	{
		case 0:
		case 1:
			if (packet[0] == 'Z')
			{
				if ((retval = breakpoint_add(target, address, size, bp_type)) != ERROR_OK)
				{
					if ((retval = gdb_bp_wp_packet_error(connection, retval)) != ERROR_OK)
						return retval;
				}
				else
				{
					gdb_put_packet(connection, "OK", 2);
				}
			}
			else
			{
				breakpoint_remove(target, address);
				gdb_put_packet(connection, "OK", 2);
			}
			break;
		case 2:
		case 3:
		case 4:
		{
			if (packet[0] == 'Z')
			{
				if ((retval = watchpoint_add(target, address, size, type-2, 0, 0xffffffffu)) != ERROR_OK)
				{
					if ((retval = gdb_bp_wp_packet_error(connection, retval)) != ERROR_OK)
						return retval;
				}
				else
				{
					gdb_put_packet(connection, "OK", 2);
				}
			}
			else
			{
				watchpoint_remove(target, address);
				gdb_put_packet(connection, "OK", 2);
			}
			break;
		}
		default:
			break;
	}

	return ERROR_OK;
}

void gdb_query_packet(connection_t *connection, char *packet, int packet_size)
{
	command_context_t *cmd_ctx = connection->cmd_ctx;

	if (strstr(packet, "qRcmd,"))
	{
		if (packet_size > 6)
		{
			char *cmd;
			int i;
			cmd = malloc((packet_size - 6)/2 + 1);
			for (i=0; i < (packet_size - 6)/2; i++)
			{
				u32 tmp;
				sscanf(packet + 6 + 2*i, "%2x", &tmp);
				cmd[i] = tmp;
			}
			cmd[(packet_size - 6)/2] = 0x0;
			command_run_line(cmd_ctx, cmd);
			free(cmd);
		}
		gdb_put_packet(connection, "OK", 2);
		return;
	}
	
	gdb_put_packet(connection, "", 0);
}

int gdb_input(connection_t *connection)
{
	gdb_service_t *gdb_service = connection->service->priv;
	target_t *target = gdb_service->target;
	char packet[GDB_BUFFER_SIZE];
	int packet_size;
	int retval;
	gdb_connection_t *gdb_con = connection->priv;

	/* drain input buffer */
	do
	{
		packet_size = GDB_BUFFER_SIZE-1;
		if ((retval = gdb_get_packet(connection, packet, &packet_size)) != ERROR_OK)
		{
			switch (retval)
			{
				case ERROR_GDB_BUFFER_TOO_SMALL:
					ERROR("BUG: buffer supplied for gdb packet was too small");
					exit(-1);
				case ERROR_SERVER_REMOTE_CLOSED:
					return ERROR_SERVER_REMOTE_CLOSED;
				default:
					ERROR("BUG: unexpected error");
					exit(-1);
			}
		}
		
		/* terminate with zero */
		packet[packet_size] = 0;
		
		DEBUG("recevied packet: '%s'", packet);
		
		if (packet_size > 0)
		{
			retval = ERROR_OK;
			switch (packet[0])
			{
				case 'H':
					/* Hct... -- set thread 
					* we don't have threads, send empty reply */
					gdb_put_packet(connection, NULL, 0);
					break;
				case 'q':
					gdb_query_packet(connection, packet, packet_size);
					break;
				case 'g':
					retval = gdb_get_registers_packet(connection, target, packet, packet_size);
					break;
				case 'G':
					retval = gdb_set_registers_packet(connection, target, packet, packet_size);
					break;
				case 'p':
					retval = gdb_get_register_packet(connection, target, packet, packet_size);
					break;
				case 'P':
					retval = gdb_set_register_packet(connection, target, packet, packet_size);
					break;
				case 'm':
					retval = gdb_read_memory_packet(connection, target, packet, packet_size);
					break;
				case 'M':
					retval = gdb_write_memory_packet(connection, target, packet, packet_size);
					break;
				case 'z':
				case 'Z':
					retval = gdb_breakpoint_watchpoint_packet(connection, target, packet, packet_size);
					break;
				case '?':
					gdb_last_signal_packet(connection, target, packet, packet_size);
				break;
				case 'c':
				case 's':
					gdb_step_continue_packet(connection, target, packet, packet_size);
					break;
				case 'D':
					target->type->resume(target, 1, 0, 1, 0);
					gdb_put_packet(connection, "OK", 2);
					break;
				case 'X':
					if ((retval = gdb_write_memory_binary_packet(connection, target, packet, packet_size)) != ERROR_OK)
						return retval;
					break;
				case 'k':
					gdb_put_packet(connection, "OK", 2);
					return ERROR_SERVER_REMOTE_CLOSED;
				default:
					/* ignore unkown packets */
					DEBUG("ignoring 0x%2.2x packet", packet[0]);
					gdb_put_packet(connection, NULL, 0);
					break;
			}
			
			/* if a packet handler returned an error, exit input loop */
			if (retval != ERROR_OK)
				return retval;
		}
				
		if (gdb_con->ctrl_c)
		{
			if (target->state == TARGET_RUNNING)
			{
				target->type->halt(target);
				gdb_con->ctrl_c = 0;
			}
		}
		
	} while (gdb_con->buf_cnt > 0);

	return ERROR_OK;
}

int gdb_init()
{
	gdb_service_t *gdb_service;
	target_t *target = targets;
	int i = 0;
	
	if (!target)
	{
		WARNING("no gdb ports allocated as no target has been specified");
		return ERROR_OK;
	}
		
	if (gdb_port == 0)
	{
		WARNING("no gdb port specified, using default port 3333");
		gdb_port = 3333;
	}
	
	while (target)
	{
		char service_name[8];
		
		snprintf(service_name, 8, "gdb-%2.2i", i);
		
		gdb_service = malloc(sizeof(gdb_service_t));
		gdb_service->target = target;
	
		add_service("gdb", CONNECTION_GDB, gdb_port + i, 1, gdb_new_connection, gdb_input, gdb_connection_closed, gdb_service);
		
		DEBUG("gdb service for target %s at port %i", target->type->name, gdb_port + i);
		
		i++;
		target = target->next;
	}
	
	return ERROR_OK;
}

/* daemon configuration command gdb_port */
int handle_gdb_port_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 0)
		return ERROR_OK;

	/* only if the port wasn't overwritten by cmdline */
	if (gdb_port == 0)
		gdb_port = strtoul(args[0], NULL, 0);

	return ERROR_OK;
}

int gdb_register_commands(command_context_t *command_context)
{
	register_command(command_context, NULL, "gdb_port", handle_gdb_port_command,
					 COMMAND_CONFIG, "");
	
	return ERROR_OK;
}
