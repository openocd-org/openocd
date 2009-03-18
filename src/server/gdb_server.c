/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
#include "jtag.h"
#include "breakpoints.h"
#include "flash.h"
#include "target.h"
#include "target_request.h"
#include "configuration.h"

#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <stdlib.h>

#if 0
#define _DEBUG_GDB_IO_
#endif

static int gdb_breakpoint_override;
static enum breakpoint_type gdb_breakpoint_override_type;

extern int gdb_error(connection_t *connection, int retval);
static unsigned short gdb_port;
static const char *DIGITS = "0123456789abcdef";

static void gdb_log_callback(void *priv, const char *file, int line,
		const char *function, const char *string);

enum gdb_detach_mode
{
	GDB_DETACH_RESUME,
	GDB_DETACH_RESET,
	GDB_DETACH_HALT,
	GDB_DETACH_NOTHING
};

/* target behaviour on gdb detach */
enum gdb_detach_mode detach_mode = GDB_DETACH_RESUME;

/* set if we are sending a memory map to gdb
 * via qXfer:memory-map:read packet */
/* enabled by default*/
int gdb_use_memory_map = 1;
/* enabled by default*/
int gdb_flash_program = 1;

/* if set, data aborts cause an error to be reported in memory read packets
 * see the code in gdb_read_memory_packet() for further explanations */
int gdb_report_data_abort = 0;

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
			LOG_USER("undefined debug reason %d - target needs reset", target->debug_reason);
			return 0x0;
	}
}

int check_pending(connection_t *connection, int timeout_s, int *got_data)
{
	/* a non-blocking socket will block if there is 0 bytes available on the socket,
	 * but return with as many bytes as are available immediately
	 */
	struct timeval tv;
	fd_set read_fds;
	gdb_connection_t *gdb_con = connection->priv;
	int t;
	if (got_data==NULL)
		got_data=&t;
	*got_data=0;

	if (gdb_con->buf_cnt>0)
	{
		*got_data = 1;
		return ERROR_OK;
	}

	FD_ZERO(&read_fds);
	FD_SET(connection->fd, &read_fds);

	tv.tv_sec = timeout_s;
	tv.tv_usec = 0;
	if (socket_select(connection->fd + 1, &read_fds, NULL, NULL, &tv) == 0)
	{
		/* This can typically be because a "monitor" command took too long
		 * before printing any progress messages
		 */
		if (timeout_s>0)
		{
			return ERROR_GDB_TIMEOUT;
		} else
		{
			return ERROR_OK;
		}
	}
	*got_data=FD_ISSET(connection->fd, &read_fds)!=0;
	return ERROR_OK;
}

int gdb_get_char(connection_t *connection, int* next_char)
{
	gdb_connection_t *gdb_con = connection->priv;
	int retval=ERROR_OK;

#ifdef _DEBUG_GDB_IO_
	char *debug_buffer;
#endif

	if (gdb_con->buf_cnt-- > 0)
	{
		*next_char = *(gdb_con->buf_p++);
		if (gdb_con->buf_cnt > 0)
			connection->input_pending = 1;
		else
			connection->input_pending = 0;

#ifdef _DEBUG_GDB_IO_
		LOG_DEBUG("returned char '%c' (0x%2.2x)", *next_char, *next_char);
#endif

		return ERROR_OK;
	}

	for (;;)
	{
		if (connection->service->type == CONNECTION_PIPE)
		{
			gdb_con->buf_cnt = read(connection->fd, gdb_con->buffer, GDB_BUFFER_SIZE);
		}
		else
		{
			retval = check_pending(connection, 1, NULL);
			if (retval != ERROR_OK)
				return retval;
			gdb_con->buf_cnt = read_socket(connection->fd, gdb_con->buffer, GDB_BUFFER_SIZE);
		}

		if (gdb_con->buf_cnt > 0)
		{
			break;
		}
		if (gdb_con->buf_cnt == 0)
		{
			gdb_con->closed = 1;
			return ERROR_SERVER_REMOTE_CLOSED;
		}

#ifdef _WIN32
		errno = WSAGetLastError();

		switch(errno)
		{
			case WSAEWOULDBLOCK:
				usleep(1000);
				break;
			case WSAECONNABORTED:
				gdb_con->closed = 1;
				return ERROR_SERVER_REMOTE_CLOSED;
			case WSAECONNRESET:
				gdb_con->closed = 1;
				return ERROR_SERVER_REMOTE_CLOSED;
			default:
				LOG_ERROR("read: %d", errno);
				exit(-1);
		}
#else
		switch(errno)
		{
			case EAGAIN:
				usleep(1000);
				break;
			case ECONNABORTED:
				gdb_con->closed = 1;
				return ERROR_SERVER_REMOTE_CLOSED;
			case ECONNRESET:
				gdb_con->closed = 1;
				return ERROR_SERVER_REMOTE_CLOSED;
			default:
				LOG_ERROR("read: %s", strerror(errno));
				gdb_con->closed = 1;
				return ERROR_SERVER_REMOTE_CLOSED;
		}
#endif
	}

#ifdef _DEBUG_GDB_IO_
	debug_buffer = malloc(gdb_con->buf_cnt + 1);
	memcpy(debug_buffer, gdb_con->buffer, gdb_con->buf_cnt);
	debug_buffer[gdb_con->buf_cnt] = 0;
	LOG_DEBUG("received '%s'", debug_buffer);
	free(debug_buffer);
#endif

	gdb_con->buf_p = gdb_con->buffer;
	gdb_con->buf_cnt--;
	*next_char = *(gdb_con->buf_p++);
	if (gdb_con->buf_cnt > 0)
		connection->input_pending = 1;
	else
		connection->input_pending = 0;
#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("returned char '%c' (0x%2.2x)", *next_char, *next_char);
#endif

	return retval;
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
		LOG_ERROR("BUG: couldn't put character back");
	}

	return ERROR_OK;
}

/* The only way we can detect that the socket is closed is the first time
 * we write to it, we will fail. Subsequent write operations will
 * succeed. Shudder! */
int gdb_write(connection_t *connection, void *data, int len)
{
	gdb_connection_t *gdb_con = connection->priv;
	if (gdb_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	if (connection->service->type == CONNECTION_PIPE)
	{
		/* write to stdout */
		if (write(STDOUT_FILENO, data, len) == len)
		{
			return ERROR_OK;
		}
	}
	else
	{
		if (write_socket(connection->fd, data, len) == len)
		{
			return ERROR_OK;
		}
	}
	gdb_con->closed = 1;
	return ERROR_SERVER_REMOTE_CLOSED;
}

int gdb_put_packet_inner(connection_t *connection, char *buffer, int len)
{
	int i;
	unsigned char my_checksum = 0;
#ifdef _DEBUG_GDB_IO_
	char *debug_buffer;
#endif
	int reply;
	int retval;
	gdb_connection_t *gdb_con = connection->priv;

	for (i = 0; i < len; i++)
		my_checksum += buffer[i];

#ifdef _DEBUG_GDB_IO_
	/*
	 * At this point we should have nothing in the input queue from GDB,
	 * however sometimes '-' is sent even though we've already received
	 * an ACK (+) for everything we've sent off.
	 */
	int gotdata;
	for (;;)
	{
		if ((retval=check_pending(connection, 0, &gotdata))!=ERROR_OK)
			return retval;
		if (!gotdata)
			break;
		if ((retval = gdb_get_char(connection, &reply)) != ERROR_OK)
			return retval;
		if( reply == '$' ){
			/* fix a problem with some IAR tools */
			gdb_putback_char( connection, reply );
			LOG_DEBUG("Unexpected start of new packet");
			break;
		}

		LOG_WARNING("Discard unexpected char %c", reply);
	}
#endif

	while (1)
	{
#ifdef _DEBUG_GDB_IO_
		debug_buffer = malloc(len + 1);
		memcpy(debug_buffer, buffer, len);
		debug_buffer[len] = 0;
		LOG_DEBUG("sending packet '$%s#%2.2x'", debug_buffer, my_checksum);
		free(debug_buffer);
#endif

		char local_buffer[1024];
		local_buffer[0] = '$';
		if (len+4 <= sizeof(local_buffer))
		{
			/* performance gain on smaller packets by only a single call to gdb_write() */
			memcpy(local_buffer+1, buffer, len++);
			local_buffer[len++] = '#';
			local_buffer[len++] = DIGITS[(my_checksum >> 4) & 0xf];
			local_buffer[len++] = DIGITS[my_checksum & 0xf];
			if((retval = gdb_write(connection, local_buffer, len)) != ERROR_OK)
			{
				return retval;
			}
		}
		else
		{
			/* larger packets are transmitted directly from caller supplied buffer
			   by several calls to gdb_write() to avoid dynamic allocation */
			local_buffer[1] = '#';
			local_buffer[2] = DIGITS[(my_checksum >> 4) & 0xf];
			local_buffer[3] = DIGITS[my_checksum & 0xf];
			if((retval = gdb_write(connection, local_buffer, 1)) != ERROR_OK)
			{
				return retval;
			}
			if((retval = gdb_write(connection, buffer, len)) != ERROR_OK)
			{
				return retval;
			}
			if((retval = gdb_write(connection, local_buffer+1, 3)) != ERROR_OK)
			{
				return retval;
			}
		}

		if (gdb_con->noack_mode)
			break;

		if ((retval = gdb_get_char(connection, &reply)) != ERROR_OK)
			return retval;

		if (reply == '+')
			break;
		else if (reply == '-')
		{
			/* Stop sending output packets for now */
			log_remove_callback(gdb_log_callback, connection);
			LOG_WARNING("negative reply, retrying");
		}
		else if (reply == 0x3)
		{
			gdb_con->ctrl_c = 1;
			if ((retval = gdb_get_char(connection, &reply)) != ERROR_OK)
				return retval;
			if (reply == '+')
				break;
			else if (reply == '-')
			{
				/* Stop sending output packets for now */
				log_remove_callback(gdb_log_callback, connection);
				LOG_WARNING("negative reply, retrying");
			}
			else if( reply == '$' ){
				LOG_ERROR("GDB missing ack(1) - assumed good");
				gdb_putback_char( connection, reply );
				return ERROR_OK;
			} else {

				LOG_ERROR("unknown character(1) 0x%2.2x in reply, dropping connection", reply);
				gdb_con->closed=1;
				return ERROR_SERVER_REMOTE_CLOSED;
			}
		}
		else if( reply == '$' ){
			LOG_ERROR("GDB missing ack(2) - assumed good");
			gdb_putback_char( connection, reply );
			return ERROR_OK;
		}
		else
		{
			LOG_ERROR("unknown character(2) 0x%2.2x in reply, dropping connection", reply);
			gdb_con->closed=1;
			return ERROR_SERVER_REMOTE_CLOSED;
		}
	}
	if (gdb_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	return ERROR_OK;
}

int gdb_put_packet(connection_t *connection, char *buffer, int len)
{
	gdb_connection_t *gdb_con = connection->priv;
	gdb_con->busy = 1;
	int retval = gdb_put_packet_inner(connection, buffer, len);
	gdb_con->busy = 0;

	/* we sent some data, reset timer for keep alive messages */
	kept_alive();

	return retval;
}

static __inline__ int fetch_packet(connection_t *connection, int *checksum_ok, int noack, int *len, char *buffer)
{
	unsigned char my_checksum = 0;
	char checksum[3];
	int character;
	int retval;

	gdb_connection_t *gdb_con = connection->priv;
	my_checksum = 0;
	int count = 0;
	count = 0;
	for (;;)
	{
		/* The common case is that we have an entire packet with no escape chars.
		 * We need to leave at least 2 bytes in the buffer to have
		 * gdb_get_char() update various bits and bobs correctly.
		 */
		if ((gdb_con->buf_cnt > 2) && ((gdb_con->buf_cnt+count) < *len))
		{
			/* The compiler will struggle a bit with constant propagation and
			 * aliasing, so we help it by showing that these values do not
			 * change inside the loop
			 */
			int i;
			char *buf = gdb_con->buf_p;
			int run = gdb_con->buf_cnt - 2;
			i = 0;
			int done = 0;
			while (i < run)
			{
				character = *buf++;
				i++;
				if (character == '#')
				{
					/* Danger! character can be '#' when esc is
					 * used so we need an explicit boolean for done here.
					 */
					done = 1;
					break;
				}

				if (character == '}')
				{
					/* data transmitted in binary mode (X packet)
					 * uses 0x7d as escape character */
					my_checksum += character & 0xff;
					character = *buf++;
					i++;
					my_checksum += character & 0xff;
					buffer[count++] = (character ^ 0x20) & 0xff;
				}
				else
				{
					my_checksum += character & 0xff;
					buffer[count++] = character & 0xff;
				}
			}
			gdb_con->buf_p += i;
			gdb_con->buf_cnt -= i;
			if (done)
				break;
		}
		if (count > *len)
		{
			LOG_ERROR("packet buffer too small");
			return ERROR_GDB_BUFFER_TOO_SMALL;
		}

		if ((retval = gdb_get_char(connection, &character)) != ERROR_OK)
			return retval;

		if (character == '#')
			break;

		if (character == '}')
		{
			/* data transmitted in binary mode (X packet)
			 * uses 0x7d as escape character */
			my_checksum += character & 0xff;
			if ((retval = gdb_get_char(connection, &character)) != ERROR_OK)
				return retval;
			my_checksum += character & 0xff;
			buffer[count++] = (character ^ 0x20) & 0xff;
		}
		else
		{
			my_checksum += character & 0xff;
			buffer[count++] = character & 0xff;
		}
	}

	*len = count;

	if ((retval = gdb_get_char(connection, &character)) != ERROR_OK)
		return retval;
	checksum[0] = character;
	if ((retval = gdb_get_char(connection, &character)) != ERROR_OK)
		return retval;
	checksum[1] = character;
	checksum[2] = 0;

	if (!noack)
	{
		*checksum_ok=(my_checksum == strtoul(checksum, NULL, 16));
	}

	return ERROR_OK;
}

int gdb_get_packet_inner(connection_t *connection, char *buffer, int *len)
{
	int character;
	int retval;
	gdb_connection_t *gdb_con = connection->priv;

	while (1)
	{
		do
		{
			if ((retval = gdb_get_char(connection, &character)) != ERROR_OK)
				return retval;

#ifdef _DEBUG_GDB_IO_
			LOG_DEBUG("character: '%c'", character);
#endif

			switch (character)
			{
				case '$':
					break;
				case '+':
					/* gdb sends a dummy ack '+' at every remote connect - see remote_start_remote (remote.c)
					 * incase anyone tries to debug why they receive this warning every time */
					LOG_WARNING("acknowledgment received, but no packet pending");
					break;
				case '-':
					LOG_WARNING("negative acknowledgment, but no packet pending");
					break;
				case 0x3:
					gdb_con->ctrl_c = 1;
					*len = 0;
					return ERROR_OK;
				default:
					LOG_WARNING("ignoring character 0x%x", character);
					break;
			}
		} while (character != '$');



		int checksum_ok = 0;
		/* explicit code expansion here to get faster inlined code in -O3 by not
		 * calculating checksum
		 */
		if (gdb_con->noack_mode)
		{
			if ((retval=fetch_packet(connection, &checksum_ok, 1, len, buffer))!=ERROR_OK)
				return retval;
		} else
		{
			if ((retval=fetch_packet(connection, &checksum_ok, 0, len, buffer))!=ERROR_OK)
				return retval;
		}

		if (gdb_con->noack_mode)
		{
			/* checksum is not checked in noack mode */
			break;
		}
		if (checksum_ok)
		{
			if ((retval = gdb_write(connection, "+", 1)) != ERROR_OK)
			{
				return retval;
			}
			break;
		}
	}
	if (gdb_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	return ERROR_OK;
}

int gdb_get_packet(connection_t *connection, char *buffer, int *len)
{
	gdb_connection_t *gdb_con = connection->priv;
	gdb_con->busy = 1;
	int retval = gdb_get_packet_inner(connection, buffer, len);
	gdb_con->busy = 0;
	return retval;
}

int gdb_output_con(connection_t *connection, const char* line)
{
	char *hex_buffer;
	int i, bin_size;

	bin_size = strlen(line);

	hex_buffer = malloc(bin_size*2 + 2);
	if (hex_buffer == NULL)
		return ERROR_GDB_BUFFER_TOO_SMALL;

	hex_buffer[0] = 'O';
	for (i=0; i<bin_size; i++)
		snprintf(hex_buffer + 1 + i*2, 3, "%2.2x", line[i]);
	hex_buffer[bin_size*2+1] = 0;

	int retval = gdb_put_packet(connection, hex_buffer, bin_size*2 + 1);

	free(hex_buffer);
	return retval;
}

int gdb_output(struct command_context_s *context, const char* line)
{
	/* this will be dumped to the log and also sent as an O packet if possible */
	LOG_USER_N("%s", line);
	return ERROR_OK;
}


static void gdb_frontend_halted(struct target_s *target, connection_t *connection)
{
	gdb_connection_t *gdb_connection = connection->priv;

	/* In the GDB protocol when we are stepping or coninuing execution,
	 * we have a lingering reply. Upon receiving a halted event
	 * when we have that lingering packet, we reply to the original
	 * step or continue packet.
	 *
	 * Executing monitor commands can bring the target in and
	 * out of the running state so we'll see lots of TARGET_EVENT_XXX
	 * that are to be ignored.
	 */
	if (gdb_connection->frontend_state == TARGET_RUNNING)
	{
		char sig_reply[4];
		int signal;

		/* stop forwarding log packets! */
		log_remove_callback(gdb_log_callback, connection);

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
}

int gdb_target_callback_event_handler(struct target_s *target, enum target_event event, void *priv)
{
	int retval;
	connection_t *connection = priv;

	target_handle_event( target, event );
	switch (event)
	{
		case TARGET_EVENT_EARLY_HALTED:
			gdb_frontend_halted(target, connection);
			break;
		case TARGET_EVENT_HALTED:
			target_call_event_callbacks(target, TARGET_EVENT_GDB_END);
			break;
		case TARGET_EVENT_GDB_FLASH_ERASE_START:
			target_handle_event( target, TARGET_EVENT_OLD_gdb_program_config );
			if((retval = jtag_execute_queue()) != ERROR_OK)
			{
				return retval;
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
	gdb_connection->vflash_image = NULL;
	gdb_connection->closed = 0;
	gdb_connection->busy = 0;
	gdb_connection->noack_mode = 0;

	/* send ACK to GDB for debug request */
	gdb_write(connection, "+", 1);

	/* output goes through gdb connection */
	command_set_output_handler(connection->cmd_ctx, gdb_output, connection);

	/* we must remove all breakpoints registered to the target as a previous
	 * GDB session could leave dangling breakpoints if e.g. communication
	 * timed out.
	 */
	breakpoint_clear_target(gdb_service->target);
	watchpoint_clear_target(gdb_service->target);

	/* register callback to be informed about target events */
	target_register_event_callback(gdb_target_callback_event_handler, connection);

	/* a gdb session just attached, try to put the target in halt mode.
	 *
	 * DANGER!!!!
	 *
	 * If the halt fails(e.g. target needs a reset, JTAG communication not
	 * working, etc.), then the GDB connect will succeed as
	 * the get_gdb_reg_list() will lie and return a register list with
	 * dummy values.
	 *
	 * This allows GDB monitor commands to be run from a GDB init script to
	 * initialize the target
	 *
	 * Also, since the halt() is asynchronous target connect will be
	 * instantaneous and thus avoiding annoying timeout problems during
	 * connect.
	 */
	target_halt(gdb_service->target);
	/* FIX!!!! could extended-remote work better here?
	 *
	 *  wait a tiny bit for halted state or we just continue. The
	 * GDB register packet will then contain garbage
	 */
	target_wait_state(gdb_service->target, TARGET_HALTED, 500);

	/* remove the initial ACK from the incoming buffer */
	if ((retval = gdb_get_char(connection, &initial_ack)) != ERROR_OK)
		return retval;

	/* FIX!!!??? would we actually ever receive a + here???
	 * Not observed.
	 */
	if (initial_ack != '+')
		gdb_putback_char(connection, initial_ack);
	target_call_event_callbacks(gdb_service->target, TARGET_EVENT_GDB_ATTACH );
	return ERROR_OK;
}

int gdb_connection_closed(connection_t *connection)
{
	gdb_service_t *gdb_service = connection->service->priv;
	gdb_connection_t *gdb_connection = connection->priv;

	/* see if an image built with vFlash commands is left */
	if (gdb_connection->vflash_image)
	{
		image_close(gdb_connection->vflash_image);
		free(gdb_connection->vflash_image);
		gdb_connection->vflash_image = NULL;
	}

	/* if this connection registered a debug-message receiver delete it */
	delete_debug_msg_receiver(connection->cmd_ctx, gdb_service->target);

	if (connection->priv)
	{
		free(connection->priv);
		connection->priv = NULL;
	}
	else
	{
		LOG_ERROR("BUG: connection->priv == NULL");
	}

	target_unregister_event_callback(gdb_target_callback_event_handler, connection);
	target_call_event_callbacks(gdb_service->target, TARGET_EVENT_GDB_END);
	log_remove_callback(gdb_log_callback, connection);

	target_call_event_callbacks(gdb_service->target, TARGET_EVENT_GDB_DETACH );

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

static int gdb_reg_pos(target_t *target, int pos, int len)
{
	if (target->endianness == TARGET_LITTLE_ENDIAN)
		return pos;
	else
		return len - 1 - pos;
}

/* Convert register to string of bytes. NB! The # of bits in the
 * register might be non-divisible by 8(a byte), in which
 * case an entire byte is shown.
 *
 * NB! the format on the wire is the target endianess
 *
 * The format of reg->value is little endian
 *
 */
void gdb_str_to_target(target_t *target, char *tstr, reg_t *reg)
{
	int i;

	u8 *buf;
	int buf_len;
	buf = reg->value;
	buf_len = CEIL(reg->size, 8);

	for (i = 0; i < buf_len; i++)
	{
		int j = gdb_reg_pos(target, i, buf_len);
		tstr[i*2]   = DIGITS[(buf[j]>>4) & 0xf];
		tstr[i*2+1] = DIGITS[buf[j]&0xf];
	}
}

static int hextoint(char c)
{
	if (c>='0'&&c<='9')
	{
		return c-'0';
	}
	c=toupper(c);
	if (c>='A'&&c<='F')
	{
		return c-'A'+10;
	}
	LOG_ERROR("BUG: invalid register value %08x", c);
	return 0;
}

/* copy over in register buffer */
void gdb_target_to_reg(target_t *target, char *tstr, int str_len, u8 *bin)
{
	if (str_len % 2)
	{
		LOG_ERROR("BUG: gdb value with uneven number of characters encountered");
		exit(-1);
	}

	int i;
	for (i = 0; i < str_len; i+=2)
	{
		u8 t = hextoint(tstr[i])<<4;
		t |= hextoint(tstr[i+1]);

		int j = gdb_reg_pos(target, i/2, str_len/2);
		bin[j] = t;
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

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("-");
#endif

	if ((retval = target->type->get_gdb_reg_list(target, &reg_list, &reg_list_size)) != ERROR_OK)
	{
		return gdb_error(connection, retval);
	}

	for (i = 0; i < reg_list_size; i++)
	{
		reg_packet_size += reg_list[i]->size;
	}

	reg_packet = malloc(CEIL(reg_packet_size, 8) * 2);
	reg_packet_p = reg_packet;

	for (i = 0; i < reg_list_size; i++)
	{
		gdb_str_to_target(target, reg_packet_p, reg_list[i]);
		reg_packet_p += CEIL(reg_list[i]->size, 8) * 2;
	}

#ifdef _DEBUG_GDB_IO_
	{
		char *reg_packet_p;
		reg_packet_p = strndup(reg_packet, CEIL(reg_packet_size, 8) * 2);
		LOG_DEBUG("reg_packet: %s", reg_packet_p);
		free(reg_packet_p);
	}
#endif

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

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("-");
#endif

	/* skip command character */
	packet++;
	packet_size--;

	if (packet_size % 2)
	{
		LOG_WARNING("GDB set_registers packet with uneven characters received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	if ((retval = target->type->get_gdb_reg_list(target, &reg_list, &reg_list_size)) != ERROR_OK)
	{
		return gdb_error(connection, retval);
	}

	packet_p = packet;
	for (i = 0; i < reg_list_size; i++)
	{
		u8 *bin_buf;
		int chars = (CEIL(reg_list[i]->size, 8) * 2);

		if (packet_p + chars > packet + packet_size)
		{
			LOG_ERROR("BUG: register packet is too small for registers");
		}

		reg_arch_type_t *arch_type;
		bin_buf = malloc(CEIL(reg_list[i]->size, 8));
		gdb_target_to_reg(target, packet_p, chars, bin_buf);

		/* get register arch_type, and call set method */
		arch_type = register_get_arch_type(reg_list[i]->arch_type);

		arch_type->set(reg_list[i], bin_buf);

		/* advance packet pointer */
		packet_p += chars;


		free(bin_buf);
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

#ifdef _DEBUG_GDB_IO_
	LOG_DEBUG("-");
#endif

	if ((retval = target->type->get_gdb_reg_list(target, &reg_list, &reg_list_size)) != ERROR_OK)
	{
		return gdb_error(connection, retval);
	}

	if (reg_list_size <= reg_num)
	{
		LOG_ERROR("gdb requested a non-existing register");
		exit(-1);
	}

	reg_packet = malloc(CEIL(reg_list[reg_num]->size, 8) * 2);

	gdb_str_to_target(target, reg_packet, reg_list[reg_num]);

	gdb_put_packet(connection, reg_packet, CEIL(reg_list[reg_num]->size, 8) * 2);

	free(reg_list);
	free(reg_packet);

	return ERROR_OK;
}

int gdb_set_register_packet(connection_t *connection, target_t *target, char *packet, int packet_size)
{
	char *separator;
	u8 *bin_buf;
	int reg_num = strtoul(packet + 1, &separator, 16);
	reg_t **reg_list;
	int reg_list_size;
	int retval;
	reg_arch_type_t *arch_type;

	LOG_DEBUG("-");

	if ((retval = target->type->get_gdb_reg_list(target, &reg_list, &reg_list_size)) != ERROR_OK)
	{
		return gdb_error(connection, retval);
	}

	if (reg_list_size < reg_num)
	{
		LOG_ERROR("gdb requested a non-existing register");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	if (*separator != '=')
	{
		LOG_ERROR("GDB 'set register packet', but no '=' following the register number");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	/* convert from GDB-string (target-endian) to hex-string (big-endian) */
	bin_buf = malloc(CEIL(reg_list[reg_num]->size, 8));
	int chars = (CEIL(reg_list[reg_num]->size, 8) * 2);

	/* fix!!! add some sanity checks on packet size here */

	gdb_target_to_reg(target, separator + 1, chars, bin_buf);

		/* get register arch_type, and call set method */
	arch_type = register_get_arch_type(reg_list[reg_num]->arch_type);
	arch_type->set(reg_list[reg_num], bin_buf);

	gdb_put_packet(connection, "OK", 2);

	free(bin_buf);
	free(reg_list);

	return ERROR_OK;
}

int gdb_error(connection_t *connection, int retval)
{
	switch (retval)
	{
		case ERROR_TARGET_DATA_ABORT:
			gdb_send_error(connection, EIO);
			break;
		case ERROR_TARGET_TRANSLATION_FAULT:
			gdb_send_error(connection, EFAULT);
			break;
		case ERROR_TARGET_UNALIGNED_ACCESS:
			gdb_send_error(connection, EFAULT);
			break;
		case ERROR_TARGET_NOT_HALTED:
			gdb_send_error(connection, EFAULT);
			break;
		default:
			/* This could be that the target reset itself. */
			LOG_ERROR("unexpected error %i", retval);
			gdb_send_error(connection, EFAULT);
			break;
	}

	return ERROR_OK;
}

/* We don't have to worry about the default 2 second timeout for GDB packets,
 * because GDB breaks up large memory reads into smaller reads.
 *
 * 8191 bytes by the looks of it. Why 8191 bytes instead of 8192?????
 */
int gdb_read_memory_packet(connection_t *connection, target_t *target, char *packet, int packet_size)
{
	char *separator;
	u32 addr = 0;
	u32 len = 0;

	u8 *buffer;
	char *hex_buffer;

	int retval = ERROR_OK;

	/* skip command character */
	packet++;

	addr = strtoul(packet, &separator, 16);

	if (*separator != ',')
	{
		LOG_ERROR("incomplete read memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator+1, NULL, 16);

	buffer = malloc(len);

	LOG_DEBUG("addr: 0x%8.8x, len: 0x%8.8x", addr, len);

	retval = target_read_buffer(target, addr, len, buffer);

	if ((retval!=ERROR_OK)&&!gdb_report_data_abort)
	{
		/* TODO : Here we have to lie and send back all zero's lest stack traces won't work.
		 * At some point this might be fixed in GDB, in which case this code can be removed.
		 *
		 * OpenOCD developers are acutely aware of this problem, but there is nothing
		 * gained by involving the user in this problem that hopefully will get resolved
		 * eventually
		 *
		 * http://sourceware.org/cgi-bin/gnatsweb.pl?cmd=view%20audit-trail&database=gdb&pr=2395
		 *
		 * For now, the default is to fix up things to make current GDB versions work.
		 * This can be overwritten using the gdb_report_data_abort <'enable'|'disable'> command.
		 */
		memset(buffer, 0, len);
		retval = ERROR_OK;
	}

	if (retval == ERROR_OK)
	{
		hex_buffer = malloc(len * 2 + 1);

		int i;
		for (i = 0; i < len; i++)
		{
			u8 t = buffer[i];
			hex_buffer[2 * i] = DIGITS[(t >> 4) & 0xf];
			hex_buffer[2 * i + 1] = DIGITS[t & 0xf];
		}

		gdb_put_packet(connection, hex_buffer, len * 2);

		free(hex_buffer);
	}
	else
	{
		retval = gdb_error(connection, retval);
	}

	free(buffer);

	return retval;
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
		LOG_ERROR("incomplete write memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator+1, &separator, 16);

	if (*(separator++) != ':')
	{
		LOG_ERROR("incomplete write memory packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	buffer = malloc(len);

	LOG_DEBUG("addr: 0x%8.8x, len: 0x%8.8x", addr, len);

	for (i=0; i<len; i++)
	{
		u32 tmp;
		sscanf(separator + 2*i, "%2x", &tmp);
		buffer[i] = tmp;
	}

	retval = target_write_buffer(target, addr, len, buffer);

	if (retval == ERROR_OK)
	{
		gdb_put_packet(connection, "OK", 2);
	}
	else
	{
		retval = gdb_error(connection, retval);
	}

	free(buffer);

	return retval;
}

int gdb_write_memory_binary_packet(connection_t *connection, target_t *target, char *packet, int packet_size)
{
	char *separator;
	u32 addr = 0;
	u32 len = 0;

	int retval;

	/* skip command character */
	packet++;

	addr = strtoul(packet, &separator, 16);

	if (*separator != ',')
	{
		LOG_ERROR("incomplete write memory binary packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	len = strtoul(separator+1, &separator, 16);

	if (*(separator++) != ':')
	{
		LOG_ERROR("incomplete write memory binary packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	retval = ERROR_OK;
	if (len)
	{
		LOG_DEBUG("addr: 0x%8.8x, len: 0x%8.8x", addr, len);

		retval = target_write_buffer(target, addr, len, (u8*)separator);
	}

	if (retval == ERROR_OK)
	{
		gdb_put_packet(connection, "OK", 2);
	}
	else
	{
		if ((retval = gdb_error(connection, retval)) != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int gdb_step_continue_packet(connection_t *connection, target_t *target, char *packet, int packet_size)
{
	int current = 0;
	u32 address = 0x0;
	int retval=ERROR_OK;

	LOG_DEBUG("-");

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
		LOG_DEBUG("continue");
		target_handle_event( target, TARGET_EVENT_OLD_pre_resume );
		retval=target_resume(target, current, address, 0, 0); /* resume at current address, don't handle breakpoints, not debugging */
	}
	else if (packet[0] == 's')
	{
		LOG_DEBUG("step");
		retval=target->type->step(target, current, address, 0); /* step at current or address, don't handle breakpoints */
	}
	return retval;
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

	LOG_DEBUG("-");

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

	if (gdb_breakpoint_override&&((bp_type==BKPT_SOFT)||(bp_type==BKPT_HARD)))
	{
		bp_type=gdb_breakpoint_override_type;
	}

	if (*separator != ',')
	{
		LOG_ERROR("incomplete breakpoint/watchpoint packet received, dropping connection");
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	address = strtoul(separator+1, &separator, 16);

	if (*separator != ',')
	{
		LOG_ERROR("incomplete breakpoint/watchpoint packet received, dropping connection");
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
					if ((retval = gdb_error(connection, retval)) != ERROR_OK)
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
					if ((retval = gdb_error(connection, retval)) != ERROR_OK)
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

/* print out a string and allocate more space as needed, mainly used for XML at this point */
void xml_printf(int *retval, char **xml, int *pos, int *size, const char *fmt, ...)
{
	if (*retval != ERROR_OK)
	{
		return;
	}
	int first = 1;

	for (;;)
	{
		if ((*xml == NULL) || (!first))
		{
			/* start by 0 to exercise all the code paths.
			 * Need minimum 2 bytes to fit 1 char and 0 terminator. */

			*size = *size * 2 + 2;
			char *t = *xml;
			*xml = realloc(*xml, *size);
			if (*xml == NULL)
			{
				if (t)
					free(t);
				*retval = ERROR_SERVER_REMOTE_CLOSED;
				return;
			}
		}

		va_list ap;
		int ret;
		va_start(ap, fmt);
		ret = vsnprintf(*xml + *pos, *size - *pos, fmt, ap);
		va_end(ap);
		if ((ret > 0) && ((ret + 1) < *size - *pos))
		{
			*pos += ret;
			return;
		}
		/* there was just enough or not enough space, allocate more. */
		first = 0;
	}
}

static int decode_xfer_read(char *buf, char **annex, int *ofs, unsigned int *len)
{
	char *separator;

	/* Extract and NUL-terminate the annex. */
	*annex = buf;
	while (*buf && *buf != ':')
		buf++;
	if (*buf == '\0')
		return -1;
	*buf++ = 0;

	/* After the read marker and annex, qXfer looks like a
	 * traditional 'm' packet. */

	*ofs = strtoul(buf, &separator, 16);

	if (*separator != ',')
		return -1;

	*len = strtoul(separator+1, NULL, 16);

	return 0;
}

int gdb_calc_blocksize(flash_bank_t *bank)
{
	int i;
	int block_size = 0xffffffff;

	/* loop through all sectors and return smallest sector size */

	for (i = 0; i < bank->num_sectors; i++)
	{
		if (bank->sectors[i].size < block_size)
			block_size = bank->sectors[i].size;
	}

	return block_size;
}

static int compare_bank (const void * a, const void * b)
{
	flash_bank_t *b1, *b2;
	b1=*((flash_bank_t **)a);
	b2=*((flash_bank_t **)b);

	if (b1->base==b2->base)
	{
		return 0;
	} else if (b1->base>b2->base)
	{
		return 1;
	} else
	{
		return -1;
	}
}

int gdb_query_packet(connection_t *connection, target_t *target, char *packet, int packet_size)
{
	command_context_t *cmd_ctx = connection->cmd_ctx;
	gdb_connection_t *gdb_connection = connection->priv;

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

			/* We want to print all debug output to GDB connection */
			log_add_callback(gdb_log_callback, connection);
			target_call_timer_callbacks_now();
			command_run_line(cmd_ctx, cmd);
			target_call_timer_callbacks_now();
			log_remove_callback(gdb_log_callback, connection);
			free(cmd);
		}
		gdb_put_packet(connection, "OK", 2);
		return ERROR_OK;
	}
	else if (strstr(packet, "qCRC:"))
	{
		if (packet_size > 5)
		{
			int retval;
			char gdb_reply[10];
			char *separator;
			u32 checksum;
			u32 addr = 0;
			u32 len = 0;

			/* skip command character */
			packet += 5;

			addr = strtoul(packet, &separator, 16);

			if (*separator != ',')
			{
				LOG_ERROR("incomplete read memory packet received, dropping connection");
				return ERROR_SERVER_REMOTE_CLOSED;
			}

			len = strtoul(separator + 1, NULL, 16);

			retval = target_checksum_memory(target, addr, len, &checksum);

			if (retval == ERROR_OK)
			{
				snprintf(gdb_reply, 10, "C%8.8x", checksum);
				gdb_put_packet(connection, gdb_reply, 9);
			}
			else
			{
				if ((retval = gdb_error(connection, retval)) != ERROR_OK)
					return retval;
			}

			return ERROR_OK;
		}
	}
	else if (strstr(packet, "qSupported"))
	{
		/* we currently support packet size and qXfer:memory-map:read (if enabled)
		 * disable qXfer:features:read for the moment */
		int retval = ERROR_OK;
		char *buffer = NULL;
		int pos = 0;
		int size = 0;

		xml_printf(&retval, &buffer, &pos, &size,
				"PacketSize=%x;qXfer:memory-map:read%c;qXfer:features:read-;QStartNoAckMode+",
				(GDB_BUFFER_SIZE - 1), ((gdb_use_memory_map == 1)&&(flash_get_bank_count()>0)) ? '+' : '-');

		if (retval != ERROR_OK)
		{
			gdb_send_error(connection, 01);
			return ERROR_OK;
		}

		gdb_put_packet(connection, buffer, strlen(buffer));
		free(buffer);

		return ERROR_OK;
	}
	else if (strstr(packet, "qXfer:memory-map:read::")&&(flash_get_bank_count()>0))
	{
		/* We get away with only specifying flash here. Regions that are not
		 * specified are treated as if we provided no memory map(if not we
		 * could detect the holes and mark them as RAM).
		 * Normally we only execute this code once, but no big deal if we
		 * have to regenerate it a couple of times. */

		flash_bank_t *p;
		char *xml = NULL;
		int size = 0;
		int pos = 0;
		int retval = ERROR_OK;

		int offset;
		int length;
		char *separator;
		int blocksize;

		/* skip command character */
		packet += 23;

		offset = strtoul(packet, &separator, 16);
		length = strtoul(separator + 1, &separator, 16);

		xml_printf(&retval, &xml, &pos, &size, "<memory-map>\n");

		/*
		sort banks in ascending order, we need to make non-flash memory be ram(or rather
		read/write) by default for GDB.
		GDB does not have a concept of non-cacheable read/write memory.
		 */
		flash_bank_t **banks=malloc(sizeof(flash_bank_t *)*flash_get_bank_count());
		int i;

		for (i=0; i<flash_get_bank_count(); i++)
		{
			p = get_flash_bank_by_num(i);
			if (p == NULL)
			{
				free(banks);
				retval = ERROR_FAIL;
				gdb_send_error(connection, retval);
				return retval;
			}
			banks[i]=p;
		}

		qsort(banks, flash_get_bank_count(), sizeof(flash_bank_t *), compare_bank);

		u32 ram_start=0;
		for (i=0; i<flash_get_bank_count(); i++)
		{
			p = banks[i];

			if (ram_start<p->base)
			{
				xml_printf(&retval, &xml, &pos, &size, "<memory type=\"ram\" start=\"0x%x\" length=\"0x%x\"/>\n",
					ram_start, p->base-ram_start);
			}

			/* if device has uneven sector sizes, eg. str7, lpc
			 * we pass the smallest sector size to gdb memory map */
			blocksize = gdb_calc_blocksize(p);

			xml_printf(&retval, &xml, &pos, &size, "<memory type=\"flash\" start=\"0x%x\" length=\"0x%x\">\n" \
				"<property name=\"blocksize\">0x%x</property>\n" \
				"</memory>\n", \
				p->base, p->size, blocksize);
			ram_start=p->base+p->size;
		}
		if (ram_start!=0)
		{
			xml_printf(&retval, &xml, &pos, &size, "<memory type=\"ram\" start=\"0x%x\" length=\"0x%x\"/>\n",
				ram_start, 0-ram_start);
		} else
		{
			/* a flash chip could be at the very end of the 32 bit address space, in which case
			ram_start will be precisely 0 */
		}

		free(banks);
		banks = NULL;

		xml_printf(&retval, &xml, &pos, &size, "</memory-map>\n");

		if (retval != ERROR_OK)
		{
			gdb_send_error(connection, retval);
			return retval;
		}

		if (offset + length > pos)
		{
			length = pos - offset;
		}

		char *t = malloc(length + 1);
		t[0] = 'l';
		memcpy(t + 1, xml + offset, length);
		gdb_put_packet(connection, t, length + 1);

		free(t);
		free(xml);
		return ERROR_OK;
	}
	else if (strstr(packet, "qXfer:features:read:"))
	{
		char *xml = NULL;
		int size = 0;
		int pos = 0;
		int retval = ERROR_OK;

		int offset;
		unsigned int length;
		char *annex;

		/* skip command character */
		packet += 20;

		if (decode_xfer_read(packet, &annex, &offset, &length) < 0)
		{
			gdb_send_error(connection, 01);
			return ERROR_OK;
		}

		if (strcmp(annex, "target.xml") != 0)
		{
			gdb_send_error(connection, 01);
			return ERROR_OK;
		}

		xml_printf(&retval, &xml, &pos, &size, \
			"l<target version=\"1.0\">\n<architecture>arm</architecture>\n</target>\n");

		if (retval != ERROR_OK)
		{
			gdb_send_error(connection, retval);
			return retval;
		}

		gdb_put_packet(connection, xml, strlen(xml));

		free(xml);
		return ERROR_OK;
	}
	else if (strstr(packet, "QStartNoAckMode"))
	{
		gdb_connection->noack_mode = 1;
		gdb_put_packet(connection, "OK", 2);
		return ERROR_OK;
	}

	gdb_put_packet(connection, "", 0);
	return ERROR_OK;
}

int gdb_v_packet(connection_t *connection, target_t *target, char *packet, int packet_size)
{
	gdb_connection_t *gdb_connection = connection->priv;
	gdb_service_t *gdb_service = connection->service->priv;
	int result;

	/* if flash programming disabled - send a empty reply */

	if (gdb_flash_program == 0)
	{
		gdb_put_packet(connection, "", 0);
		return ERROR_OK;
	}

	if (strstr(packet, "vFlashErase:"))
	{
		unsigned long addr;
		unsigned long length;

		char *parse = packet + 12;
		if (*parse == '\0')
		{
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}

		addr = strtoul(parse, &parse, 16);

		if (*(parse++) != ',' || *parse == '\0')
		{
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}

		length = strtoul(parse, &parse, 16);

		if (*parse != '\0')
		{
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}

		/* assume all sectors need erasing - stops any problems
		 * when flash_write is called multiple times */
		flash_set_dirty();

		/* perform any target specific operations before the erase */
		target_call_event_callbacks(gdb_service->target, TARGET_EVENT_GDB_FLASH_ERASE_START);
		result = flash_erase_address_range(gdb_service->target, addr, length );
		target_call_event_callbacks(gdb_service->target, TARGET_EVENT_GDB_FLASH_ERASE_END);

		/* perform erase */
		if (result != ERROR_OK)
		{
			/* GDB doesn't evaluate the actual error number returned,
			 * treat a failed erase as an I/O error
			 */
			gdb_send_error(connection, EIO);
			LOG_ERROR("flash_erase returned %i", result);
		}
		else
			gdb_put_packet(connection, "OK", 2);

		return ERROR_OK;
	}

	if (strstr(packet, "vFlashWrite:"))
	{
		int retval;
		unsigned long addr;
		unsigned long length;
		char *parse = packet + 12;

		if (*parse == '\0')
		{
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}
		addr = strtoul(parse, &parse, 16);
		if (*(parse++) != ':')
		{
			LOG_ERROR("incomplete vFlashErase packet received, dropping connection");
			return ERROR_SERVER_REMOTE_CLOSED;
		}
		length = packet_size - (parse - packet);

		/* create a new image if there isn't already one */
		if (gdb_connection->vflash_image == NULL)
		{
			gdb_connection->vflash_image = malloc(sizeof(image_t));
			image_open(gdb_connection->vflash_image, "", "build");
		}

		/* create new section with content from packet buffer */
		if((retval = image_add_section(gdb_connection->vflash_image, addr, length, 0x0, (u8*)parse)) != ERROR_OK)
		{
			return retval;
		}

		gdb_put_packet(connection, "OK", 2);

		return ERROR_OK;
	}

	if (!strcmp(packet, "vFlashDone"))
	{
		u32 written;

		/* process the flashing buffer. No need to erase as GDB
		 * always issues a vFlashErase first. */
		target_call_event_callbacks(gdb_service->target, TARGET_EVENT_GDB_FLASH_WRITE_START);
		result = flash_write(gdb_service->target, gdb_connection->vflash_image, &written, 0);
		target_call_event_callbacks(gdb_service->target, TARGET_EVENT_GDB_FLASH_WRITE_END);
		if ( result != ERROR_OK)
		{
			if (result == ERROR_FLASH_DST_OUT_OF_BANK)
				gdb_put_packet(connection, "E.memtype", 9);
			else
				gdb_send_error(connection, EIO);
			}
		else
		{
			LOG_DEBUG("wrote %u bytes from vFlash image to flash", written);
			gdb_put_packet(connection, "OK", 2);
		}

		image_close(gdb_connection->vflash_image);
		free(gdb_connection->vflash_image);
		gdb_connection->vflash_image = NULL;

		return ERROR_OK;
	}

	gdb_put_packet(connection, "", 0);
	return ERROR_OK;
}

int gdb_detach(connection_t *connection, target_t *target)
{

	switch( detach_mode )
	{
		case GDB_DETACH_RESUME:
			target_handle_event( target, TARGET_EVENT_OLD_pre_resume );
			target_resume(target, 1, 0, 1, 0);
			break;

		case GDB_DETACH_RESET:
			/* FIX?? make this configurable?? */
			target_process_reset(connection->cmd_ctx, RESET_HALT);
			break;

		case GDB_DETACH_HALT:
			target_halt(target);
			break;

		case GDB_DETACH_NOTHING:
			break;
	}

	gdb_put_packet(connection, "OK", 2);
	return ERROR_OK;
}

static void gdb_log_callback(void *priv, const char *file, int line,
		const char *function, const char *string)
{
	connection_t *connection = priv;
	gdb_connection_t *gdb_con = connection->priv;

	if (gdb_con->busy)
	{
		/* do not reply this using the O packet */
		return;
	}

	gdb_output_con(connection, string);
}

/* Do not allocate this on the stack */
char gdb_packet_buffer[GDB_BUFFER_SIZE];

static void gdb_sig_halted(connection_t *connection)
{
	char sig_reply[4];
	snprintf(sig_reply, 4, "T%2.2x", 2);
	gdb_put_packet(connection, sig_reply, 3);

}

int gdb_input_inner(connection_t *connection)
{
	gdb_service_t *gdb_service = connection->service->priv;
	target_t *target = gdb_service->target;
	char *packet=gdb_packet_buffer;
	int packet_size;
	int retval;
	gdb_connection_t *gdb_con = connection->priv;
	static int extended_protocol = 0;

	/* drain input buffer */
	do
	{
		packet_size = GDB_BUFFER_SIZE-1;
		if ((retval = gdb_get_packet(connection, packet, &packet_size)) != ERROR_OK)
		{
			return retval;
		}

		/* terminate with zero */
		packet[packet_size] = 0;

		if( LOG_LEVEL_IS( LOG_LVL_DEBUG ) ){
			if( packet[0] == 'X' ){
				// binary packets spew junk into the debug log stream
				char buf[ 50 ];
				int x;
				for( x = 0 ; (x < 49) && (packet[x] != ':') ; x++ ){
					buf[x] = packet[x];
				}
				buf[x] = 0;
				LOG_DEBUG("received packet: '%s:<binary-data>'", buf );
			} else {
				LOG_DEBUG("received packet: '%s'", packet );
			}
		}

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
				case 'Q':
					retval = gdb_query_packet(connection, target, packet, packet_size);
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
					{
						if (target->state != TARGET_HALTED)
						{
							/* If the target isn't in the halted state, then we can't
							 * step/continue. This might be early setup, etc.
							 */
							gdb_sig_halted(connection);
						} else
						{
							/* We're running/stepping, in which case we can
							 * forward log output until the target is halted
							 */
							gdb_connection_t *gdb_con = connection->priv;
							gdb_con->frontend_state = TARGET_RUNNING;
							log_add_callback(gdb_log_callback, connection);
							target_call_event_callbacks(target, TARGET_EVENT_GDB_START);
							int retval=gdb_step_continue_packet(connection, target, packet, packet_size);
							if (retval!=ERROR_OK)
							{
								/* we'll never receive a halted condition... issue a false one.. */
								gdb_frontend_halted(target, connection);
							}
						}
					}
					break;
				case 'v':
					retval = gdb_v_packet(connection, target, packet, packet_size);
					break;
				case 'D':
					retval = gdb_detach(connection, target);
					extended_protocol = 0;
					break;
				case 'X':
					if ((retval = gdb_write_memory_binary_packet(connection, target, packet, packet_size)) != ERROR_OK)
						return retval;
					break;
				case 'k':
					if (extended_protocol != 0)
						break;
					gdb_put_packet(connection, "OK", 2);
					return ERROR_SERVER_REMOTE_CLOSED;
				case '!':
					/* handle extended remote protocol */
					extended_protocol = 1;
					gdb_put_packet(connection, "OK", 2);
					break;
				case 'R':
					/* handle extended restart packet */
					breakpoint_clear_target(gdb_service->target);
					watchpoint_clear_target(gdb_service->target);
					command_run_linef(connection->cmd_ctx, "ocd_gdb_restart %d", get_num_by_target(target));
					break;
				default:
					/* ignore unkown packets */
					LOG_DEBUG("ignoring 0x%2.2x packet", packet[0]);
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
				target_halt(target);
				gdb_con->ctrl_c = 0;
			}
		}

	} while (gdb_con->buf_cnt > 0);

	return ERROR_OK;
}

int gdb_input(connection_t *connection)
{
	int retval = gdb_input_inner(connection);
	gdb_connection_t *gdb_con = connection->priv;
	if (retval == ERROR_SERVER_REMOTE_CLOSED)
		return retval;

	/* logging does not propagate the error, yet can set the gdb_con->closed flag */
	if (gdb_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	/* we'll recover from any other errors(e.g. temporary timeouts, etc.) */
	return ERROR_OK;
}

int gdb_init(void)
{
	gdb_service_t *gdb_service;
	target_t *target = all_targets;

	if (!target)
	{
		LOG_WARNING("no gdb ports allocated as no target has been specified");
		return ERROR_OK;
	}

	if (gdb_port == 0 && server_use_pipes == 0)
	{
		LOG_WARNING("no gdb port specified, using default port 3333");
		gdb_port = 3333;
	}

	if (server_use_pipes)
	{
		/* only a single gdb connection when using a pipe */

		gdb_service = malloc(sizeof(gdb_service_t));
		gdb_service->target = target;

		add_service("gdb", CONNECTION_PIPE, 0, 1, gdb_new_connection, gdb_input, gdb_connection_closed, gdb_service);

		LOG_DEBUG("gdb service for target %s using pipes", target->type->name);
	}
	else
	{
		while (target)
		{
			gdb_service = malloc(sizeof(gdb_service_t));
			gdb_service->target = target;

			add_service("gdb", CONNECTION_TCP, gdb_port + target->target_number, 1, gdb_new_connection, gdb_input, gdb_connection_closed, gdb_service);

			LOG_DEBUG("gdb service for target %s at port %i", target->type->name, gdb_port + target->target_number);
			target = target->next;
		}
	}

	return ERROR_OK;
}

/* daemon configuration command gdb_port */
int handle_gdb_port_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 0)
	{
		command_print(cmd_ctx, "%d", gdb_port);
		return ERROR_OK;
	}

	/* only if the port wasn't overwritten by cmdline */
	if (gdb_port == 0)
		gdb_port = strtoul(args[0], NULL, 0);

	return ERROR_OK;
}

int handle_gdb_detach_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 1)
	{
		if (strcmp(args[0], "resume") == 0)
		{
			detach_mode = GDB_DETACH_RESUME;
			return ERROR_OK;
		}
		else if (strcmp(args[0], "reset") == 0)
		{
			detach_mode = GDB_DETACH_RESET;
			return ERROR_OK;
		}
		else if (strcmp(args[0], "halt") == 0)
		{
			detach_mode = GDB_DETACH_HALT;
			return ERROR_OK;
		}
		else if (strcmp(args[0], "nothing") == 0)
		{
			detach_mode = GDB_DETACH_NOTHING;
			return ERROR_OK;
		}
		else
			LOG_WARNING("invalid gdb_detach configuration directive: %s", args[0]);
	}

	return ERROR_COMMAND_SYNTAX_ERROR;
}

int handle_gdb_memory_map_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 1)
	{
		if (strcmp(args[0], "enable") == 0)
		{
			gdb_use_memory_map = 1;
			return ERROR_OK;
		}
		else if (strcmp(args[0], "disable") == 0)
		{
			gdb_use_memory_map = 0;
			return ERROR_OK;
		}
		else
			LOG_WARNING("invalid gdb_memory_map configuration directive %s", args[0]);
	}

	return ERROR_COMMAND_SYNTAX_ERROR;
}

int handle_gdb_flash_program_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 1)
	{
		if (strcmp(args[0], "enable") == 0)
		{
			gdb_flash_program = 1;
			return ERROR_OK;
		}
		else if (strcmp(args[0], "disable") == 0)
		{
			gdb_flash_program = 0;
			return ERROR_OK;
		}
		else
			LOG_WARNING("invalid gdb_flash_program configuration directive: %s", args[0]);
	}

	return ERROR_COMMAND_SYNTAX_ERROR;
}

int handle_gdb_report_data_abort_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 1)
	{
		if (strcmp(args[0], "enable") == 0)
		{
			gdb_report_data_abort = 1;
			return ERROR_OK;
		}
		else if (strcmp(args[0], "disable") == 0)
		{
			gdb_report_data_abort = 0;
			return ERROR_OK;
		}
		else
			LOG_WARNING("invalid gdb_report_data_abort configuration directive: %s", args[0]);
	}

	return ERROR_COMMAND_SYNTAX_ERROR;
}

/* gdb_breakpoint_override */
int handle_gdb_breakpoint_override_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 0)
	{

	} else if (argc==1)
	{
		gdb_breakpoint_override = 1;
		if (strcmp(args[0], "hard")==0)
		{
			gdb_breakpoint_override_type=BKPT_HARD;
		} else if (strcmp(args[0], "soft")==0)
		{
			gdb_breakpoint_override_type=BKPT_SOFT;
		} else if (strcmp(args[0], "disable") == 0)
		{
			gdb_breakpoint_override = 0;
		}
	} else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	if (gdb_breakpoint_override)
	{
		LOG_USER("force %s breakpoints", (gdb_breakpoint_override_type==BKPT_HARD)?"hard":"soft");
	} else
	{
		LOG_USER("breakpoint type is not overriden");
	}

	return ERROR_OK;
}

int gdb_register_commands(command_context_t *command_context)
{
	register_command(command_context, NULL, "gdb_port", handle_gdb_port_command,
			COMMAND_ANY, "daemon configuration command gdb_port");
	register_command(command_context, NULL, "gdb_detach", handle_gdb_detach_command,
			COMMAND_CONFIG, "");
	register_command(command_context, NULL, "gdb_memory_map", handle_gdb_memory_map_command,
			COMMAND_CONFIG, "enable or disable memory map");
	register_command(command_context, NULL, "gdb_flash_program", handle_gdb_flash_program_command,
			COMMAND_CONFIG, "enable or disable flash program");
	register_command(command_context, NULL, "gdb_report_data_abort", handle_gdb_report_data_abort_command,
			COMMAND_CONFIG, "enable or disable report data");
	register_command(command_context, NULL, "gdb_breakpoint_override", handle_gdb_breakpoint_override_command,
			COMMAND_EXEC, "hard/soft/disable - force breakpoint type for gdb 'break' commands."
			"The raison d'etre for this option is to support GDB GUI's without "
			"a hard/soft breakpoint concept where the default OpenOCD behaviour "
			"is not sufficient");
	return ERROR_OK;
}
