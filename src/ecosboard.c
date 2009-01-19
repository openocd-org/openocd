/***************************************************************************
 *   Copyright (C) 2007-2008 by Øyvind Harboe                              *
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

#include "log.h"
#include "types.h"
#include "jtag.h"
#include "configuration.h"
#include "xsvf.h"
#include "svf.h"
#include "target.h"
#include "flash.h"
#include "nand.h"
#include "pld.h"

#include "command.h"
#include "server.h"
#include "telnet_server.h"
#include "gdb_server.h"

#include <time_support.h>
#include <sys/time.h>
#include <sys/types.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <cyg/io/flash.h>
#include <pkgconf/fs_jffs2.h>	// Address of JFFS2
#include <network.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <cyg/fileio/fileio.h>
#include <dirent.h>
#include <cyg/athttpd/http.h>
#include <cyg/athttpd/socket.h>
#include <cyg/athttpd/handler.h>
#include <cyg/athttpd/cgi.h>
#include <cyg/athttpd/forms.h>
#include <cyg/discover/discover.h>
#include <cyg/hal/hal_diag.h>
#include <cyg/kernel/kapi.h>
#include <cyg/io/serialio.h>
#include <cyg/io/io.h>
#include <netinet/tcp.h>
#include "rom.h"
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <stdio.h>
#include <ifaddrs.h>
#include <string.h>

#include <unistd.h>
#include <stdio.h>
#define MAX_IFS 64
#if defined(CYGPKG_NET_FREEBSD_STACK)
#include <tftp_support.h>
/* posix compatibility broken*/
struct tftpd_fileops fileops =
{
	(int (*)(const char *, int))open,
	close,
	(int (*)(int, const void *, int))write,
	( int (*)(int, void *, int))read
};

#endif


void diag_write(char *buf, int len)
{
	int j;
	for (j = 0; j < len; j++)
	{
		diag_printf("%c", buf[j]);
	}
}

static bool serialLog = true;
static bool writeLog = true;

char hwaddr[512];


extern flash_driver_t *flash_drivers[];
extern target_type_t *target_types[];

#ifdef CYGPKG_PROFILE_GPROF
#include <cyg/profile/profile.h>

extern char _stext, _etext; // Defined by the linker

static char *start_of_code=&_stext;
static char *end_of_code=&_etext;

void start_profile(void)
{
	// This starts up the system-wide profiling, gathering
	// profile information on all of the code, with a 16 byte
	// "bucket" size, at a rate of 100us/profile hit.
	// Note: a bucket size of 16 will give pretty good function
	//       resolution.  Much smaller and the buffer becomes
	//       much too large for very little gain.
	// Note: a timer period of 100us is also a reasonable
	//       compromise.  Any smaller and the overhead of
	//       handling the timter (profile) interrupt could
	//       swamp the system.  A fast processor might get
	//       by with a smaller value, but a slow one could
	//       even be swamped by this value.  If the value is
	//       too large, the usefulness of the profile is reduced.

	// no more interrupts than 1/10ms.
	//profile_on((void *)0, (void *)0x40000, 16, 10000); // SRAM
	//	profile_on(0, &_etext, 16, 10000); // SRAM & DRAM
	profile_on(start_of_code, end_of_code, 16, 10000); // Nios DRAM
}
#endif

extern int eth0_up;
static FILE *log;

static char reboot_stack[2048];

static void zylinjtag_reboot(cyg_addrword_t data)
{
	serialLog = true;
	diag_printf("Rebooting in 100 ticks..\n");
	cyg_thread_delay(100);
	diag_printf("Unmounting /config..\n");
	umount("/config");
	diag_printf("Rebooting..\n");
	HAL_PLATFORM_RESET();
}
static cyg_thread zylinjtag_thread_object;
static cyg_handle_t zylinjtag_thread_handle;

void reboot(void)
{
	cyg_thread_create(1, zylinjtag_reboot, (cyg_addrword_t) 0, "reboot Thread",
			(void *) reboot_stack, sizeof(reboot_stack),
			&zylinjtag_thread_handle, &zylinjtag_thread_object);
	cyg_thread_resume(zylinjtag_thread_handle);
}

int configuration_output_handler(struct command_context_s *context,
		const char* line)
{
	diag_printf("%s", line);

	return ERROR_OK;
}

int zy1000_configuration_output_handler_log(struct command_context_s *context,
		const char* line)
{
	LOG_USER_N("%s", line);

	return ERROR_OK;
}

#ifdef CYGPKG_PROFILE_GPROF
extern void start_profile(void);

int eCosBoard_handle_eCosBoard_profile_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	command_print(cmd_ctx, "Profiling started");
	start_profile();
	return ERROR_OK;
}

#endif

externC void phi_init_all_network_interfaces(void);

command_context_t *cmd_ctx;

static bool webRunning = false;

void keep_webserver(void)
{
	// Target initialisation is only attempted at startup, so we sleep forever and
	// let the http server bail us out(i.e. get config files set up).
	diag_printf("OpenOCD has invoked exit().\n"
		"Use web server to correct any configuration settings and reboot.\n");
	if (!webRunning)
		reboot();

	// exit() will terminate the current thread and we we'll then sleep eternally or
	// we'll have a reboot scheduled.
}

extern void printDccChar(char c);

static char logBuffer[128 * 1024];
static const int logSize = sizeof(logBuffer);
int writePtr = 0;
int logCount = 0;

void _zylinjtag_diag_write_char(char c, void **param)
{
	if (writeLog)
	{
		logBuffer[writePtr] = c;
		writePtr = (writePtr + 1) % logSize;
		logCount++;
	}
	if (serialLog)
	{
		if (c == '\n')
		{
			HAL_DIAG_WRITE_CHAR('\r');
		}
		HAL_DIAG_WRITE_CHAR(c);
	}

#ifdef CYGPKG_HAL_ZYLIN_PHI
	printDccChar(c);
#endif
}

void copyfile(char *name2, char *name1);

void copydir(char *name, char *destdir);

#if 0
MTAB_ENTRY( romfs_mte1,
		"/rom",
		"romfs",
		"",
		(CYG_ADDRWORD) &filedata[0] );
#endif

void openocd_sleep_prelude(void)
{
	cyg_mutex_unlock(&httpstate.jim_lock);
}

void openocd_sleep_postlude(void)
{
	cyg_mutex_lock(&httpstate.jim_lock);
}

void format(void)
{
	diag_printf("Formatting JFFS2...\n");

	cyg_io_handle_t handle;

	Cyg_ErrNo err;
	err = cyg_io_lookup(CYGDAT_IO_FLASH_BLOCK_DEVICE_NAME_1, &handle);
	if (err != ENOERR)
	{
		diag_printf("Flash Error cyg_io_lookup: %d\n", err);
		reboot();
	}

	cyg_uint32 len;
	cyg_io_flash_getconfig_devsize_t ds;
	len = sizeof(ds);
	err = cyg_io_get_config(handle, CYG_IO_GET_CONFIG_FLASH_DEVSIZE, &ds, &len);
	if (err != ENOERR)
	{
		diag_printf("Flash error cyg_io_get_config %d\n", err);
		reboot();
	}

	cyg_io_flash_getconfig_erase_t e;
	void *err_addr;
	len = sizeof(e);

	e.offset = 0;
	e.len = ds.dev_size;
	e.err_address = &err_addr;

	diag_printf("Formatting 0x%08x bytes\n", ds.dev_size);
	err = cyg_io_get_config(handle, CYG_IO_GET_CONFIG_FLASH_ERASE, &e, &len);
	if (err != ENOERR)
	{
		diag_printf("Flash erase error %d offset 0x%p\n", err, err_addr);
		reboot();
	}

	diag_printf("Flash formatted successfully\n");

	reboot();
}

static int zylinjtag_Jim_Command_format_jffs2(Jim_Interp *interp, int argc,
		Jim_Obj * const *argv)
{
	if (argc != 1)
	{
		return JIM_ERR;
	}

	format();
	for (;;)
		;
}

static int zylinjtag_Jim_Command_threads(Jim_Interp *interp, int argc,
		Jim_Obj * const *argv)
{
	cyg_handle_t thread = 0;
	cyg_uint16 id = 0;
	Jim_Obj *threads = Jim_NewListObj(interp, NULL, 0);

	/* Loop over the threads, and generate a table row for
	 * each.
	 */
	while (cyg_thread_get_next(&thread, &id))
	{
		Jim_Obj *threadObj = Jim_NewListObj(interp, NULL, 0);

		cyg_thread_info info;
		char *state_string;

		cyg_thread_get_info(thread, id, &info);

		if (info.name == NULL)
			info.name = "<no name>";

		Jim_ListAppendElement(interp, threadObj, Jim_NewStringObj(interp,
				info.name, strlen(info.name)));

		/* Translate the state into a string.
		 */
		if (info.state == 0)
			state_string = "RUN";
		else if (info.state & 0x04)
			state_string = "SUSP";
		else
			switch (info.state & 0x1b)
			{
			case 0x01:
				state_string = "SLEEP";
				break;
			case 0x02:
				state_string = "CNTSLEEP";
				break;
			case 0x08:
				state_string = "CREATE";
				break;
			case 0x10:
				state_string = "EXIT";
				break;
			default:
				state_string = "????";
				break;
			}

		Jim_ListAppendElement(interp, threadObj, Jim_NewStringObj(interp,
				state_string, strlen(state_string)));

		Jim_ListAppendElement(interp, threadObj, Jim_NewIntObj(interp, id));
		Jim_ListAppendElement(interp, threadObj, Jim_NewIntObj(interp,
				info.set_pri));
		Jim_ListAppendElement(interp, threadObj, Jim_NewIntObj(interp,
				info.cur_pri));

		Jim_ListAppendElement(interp, threads, threadObj);
	}
	Jim_SetResult(interp, threads);

	return JIM_OK;
}

static int zylinjtag_Jim_Command_log(Jim_Interp *interp, int argc,
		Jim_Obj * const *argv)
{
	Jim_Obj *tclOutput = Jim_NewStringObj(interp, "", 0);

	if (logCount >= logSize)
	{
		Jim_AppendString(httpstate.jim_interp, tclOutput, logBuffer + logCount
				% logSize, logSize - logCount % logSize);
	}
	Jim_AppendString(httpstate.jim_interp, tclOutput, logBuffer, writePtr);

	Jim_SetResult(interp, tclOutput);
	return JIM_OK;
}

static int zylinjtag_Jim_Command_reboot(Jim_Interp *interp, int argc,
		Jim_Obj * const *argv)
{
	reboot();
	return JIM_OK;
}


extern Jim_Interp *interp;

static void zylinjtag_startNetwork(void)
{
	// Bring TCP/IP up immediately before we're ready to accept commands.
	//
	// That is as soon as a PING responds, we're accepting telnet sessions.
#if defined(CYGPKG_NET_FREEBSD_STACK)
	phi_init_all_network_interfaces();
#else
	lwip_init();
#endif
	if (!eth0_up)
	{
		diag_printf("Network not up and running\n");
		exit(-1);
	}
#if defined(CYGPKG_NET_FREEBSD_STACK)
	/*start TFTP*/
	tftpd_start(69, &fileops);
#endif

	cyg_httpd_init_tcl_interpreter();

	interp = httpstate.jim_interp;

	Jim_CreateCommand(httpstate.jim_interp, "log", zylinjtag_Jim_Command_log,
			NULL, NULL);
	Jim_CreateCommand(httpstate.jim_interp, "reboot",
			zylinjtag_Jim_Command_reboot, NULL, NULL);
	Jim_CreateCommand(httpstate.jim_interp, "threads",
			zylinjtag_Jim_Command_threads, NULL, NULL);
	Jim_CreateCommand(httpstate.jim_interp, "format_jffs2",
			zylinjtag_Jim_Command_format_jffs2, NULL, NULL);

	cyg_httpd_start();

	webRunning = true;

	diag_printf("Web server running\n");

	int s;
	struct ifreq ifr;
	s = socket(AF_INET, SOCK_DGRAM, 0);
	if (s >= 0)
	{
		strcpy(ifr.ifr_name, "eth0");
		int res;
		res = ioctl(s, SIOCGIFHWADDR, &ifr);
		close(s);

		if (res < 0)
		{
			diag_printf("Can't obtain MAC address\n");
			reboot();
		}
	}

	sprintf(hwaddr, "%02x:%02x:%02x:%02x:%02x:%02x",
			(int) ((unsigned char *) &ifr.ifr_hwaddr.sa_data)[0],
			(int) ((unsigned char *) &ifr.ifr_hwaddr.sa_data)[1],
			(int) ((unsigned char *) &ifr.ifr_hwaddr.sa_data)[2],
			(int) ((unsigned char *) &ifr.ifr_hwaddr.sa_data)[3],
			(int) ((unsigned char *) &ifr.ifr_hwaddr.sa_data)[4],
			(int) ((unsigned char *) &ifr.ifr_hwaddr.sa_data)[5]);

	discover_message
			= alloc_printf("ZY1000 Zylin JTAG debugger MAC %s", hwaddr);

	discover_launch();
}

static void print_exception_handler(cyg_addrword_t data, cyg_code_t exception,
		cyg_addrword_t info)
{
	writeLog = false;
	serialLog = true;
	char *infoStr = "unknown";
	switch (exception)
	{
#ifdef CYGNUM_HAL_VECTOR_UNDEF_INSTRUCTION
	case CYGNUM_HAL_VECTOR_UNDEF_INSTRUCTION:
	infoStr = "undefined instruction";
	break;
	case CYGNUM_HAL_VECTOR_SOFTWARE_INTERRUPT:
	infoStr = "software interrupt";
	break;
	case CYGNUM_HAL_VECTOR_ABORT_PREFETCH:
	infoStr = "abort prefetch";
	break;
	case CYGNUM_HAL_VECTOR_ABORT_DATA:
	infoStr = "abort data";
	break;
#endif
	default:
		break;
	}

	diag_printf("Exception: %08x(%s) %08x\n", exception, infoStr, info);

	diag_printf("Dumping log\n---\n");
	if (logCount >= logSize)
	{
		diag_write(logBuffer + logCount % logSize, logSize - logCount % logSize);
	}
	diag_write(logBuffer, writePtr);

	diag_printf("---\nLogdump complete.\n");
	diag_printf("Exception: %08x(%s) %08x\n", exception, infoStr, info);
	diag_printf("\n---\nRebooting\n");
	HAL_PLATFORM_RESET();

}

static void setHandler(cyg_code_t exception)
{
	cyg_exception_handler_t *old_handler;
	cyg_addrword_t old_data;

	cyg_exception_set_handler(exception, print_exception_handler, 0,
			&old_handler, &old_data);
}

static cyg_thread zylinjtag_uart_thread_object;
static cyg_handle_t zylinjtag_uart_thread_handle;
static char uart_stack[4096];

static char forwardBuffer[1024]; // NB! must be smaller than a TCP/IP packet!!!!!
static char backwardBuffer[1024];

void setNoDelay(int session, int flag)
{
#if 1
	// This decreases latency dramatically for e.g. GDB load which
	// does not have a sliding window protocol
	//
	// Can cause *lots* of TCP/IP packets to be sent and it would have
	// to be enabled/disabled on the fly to avoid the CPU being
	// overloaded...
	setsockopt(session, /* socket affected */
	IPPROTO_TCP, /* set option at TCP level */
	TCP_NODELAY, /* name of option */
	(char *) &flag, /* the cast is historical
	 cruft */
	sizeof(int)); /* length of option value */
#endif
}

struct
{
	int req;
	int actual;
	int req2;
	int actual2;
} tcpipSent[512 * 1024];
int cur;

static void zylinjtag_uart(cyg_addrword_t data)
{
	int so_reuseaddr_option = 1;

	int fd;
	if ((fd = socket(AF_INET, SOCK_STREAM, 0)) == -1)
	{
		LOG_ERROR("error creating socket: %s", strerror(errno));
		exit(-1);
	}

	setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, (void*) &so_reuseaddr_option,
			sizeof(int));

	struct sockaddr_in sin;
	unsigned int address_size;
	address_size = sizeof(sin);
	memset(&sin, 0, sizeof(sin));
	sin.sin_family = AF_INET;
	sin.sin_addr.s_addr = INADDR_ANY;
	sin.sin_port = htons(5555);

	if (bind(fd, (struct sockaddr *) &sin, sizeof(sin)) == -1)
	{
		LOG_ERROR("couldn't bind to socket: %s", strerror(errno));
		exit(-1);
	}

	if (listen(fd, 1) == -1)
	{
		LOG_ERROR("couldn't listen on socket: %s", strerror(errno));
		exit(-1);
	}
	//	socket_nonblock(fd);


	for (;;)
	{
		int session = accept(fd, (struct sockaddr *) &sin, &address_size);
		if (session < 0)
		{
			continue;
		}

		setNoDelay(session, 1);
		int oldopts = fcntl(session, F_GETFL, 0);
		fcntl(session, F_SETFL, oldopts | O_NONBLOCK); //

		int serHandle = open("/dev/ser0", O_RDWR | O_NONBLOCK);
		if (serHandle < 0)
		{
			close(session);
			continue;
		}

#ifdef CYGPKG_PROFILE_GPROF
		start_profile();
#endif
		int actual = 0;
		int actual2 = 0;
		int pos, pos2;
		pos = 0;
		pos2 = 0;
		cur = 0;
		for (;;)
		{
			fd_set write_fds;
			fd_set read_fds;
			FD_ZERO(&write_fds);
			FD_ZERO(&read_fds);
			int fd_max = -1;
			FD_SET(session, &read_fds);
			fd_max = session;
			FD_SET(serHandle, &read_fds);
			if (serHandle > fd_max)
			{
				fd_max = serHandle;
			}
			/* Wait... */

			cyg_thread_delay(5); // 50ms fixed delay to wait for data to be sent/received
			if ((actual == 0) && (actual2 == 0))
			{
				int retval = select(fd_max + 1, &read_fds, NULL, NULL, NULL);
				if (retval <= 0)
				{
					break;
				}
			}

			if (actual2 <= 0)
			{
				memset(backwardBuffer, 's', sizeof(backwardBuffer));
				actual2 = read(serHandle, backwardBuffer,
						sizeof(backwardBuffer));
				if (actual2 < 0)
				{
					if (errno != EAGAIN)
					{
						goto closeSession;
					}
					actual2 = 0;
				}
				pos2 = 0;
			}

			int x = actual2;
			int y = 0;
			if (actual2 > 0)
			{
				int written = write(session, backwardBuffer + pos2, actual2);
				if (written <= 0)
					goto closeSession;
				actual2 -= written;
				pos2 += written;
				y = written;
			}

			if (FD_ISSET(session, &read_fds)
					&& (sizeof(forwardBuffer) > actual))
			{
				// NB! Here it is important that we empty the TCP/IP read buffer
				// to make transmission tick right
				memmove(forwardBuffer, forwardBuffer + pos, actual);
				pos = 0;
				int t;
				// this will block if there is no data at all
				t = read_socket(session, forwardBuffer + actual,
						sizeof(forwardBuffer) - actual);
				if (t <= 0)
				{
					goto closeSession;
				}
				actual += t;
			}

			int x2 = actual;
			int y2 = 0;
			if (actual > 0)
			{
				/* Do not put things into the serial buffer if it has something to send
				 * as that can cause a single byte to be sent at the time.
				 *
				 *
				 */
				int written = write(serHandle, forwardBuffer + pos, actual);
				if (written < 0)
				{
					if (errno != EAGAIN)
					{
						goto closeSession;
					}
					// The serial buffer is full
					written = 0;
				}
				else
				{
					actual -= written;
					pos += written;
				}
				y2 = written;
			}
			if (cur < 1024)
			{
				tcpipSent[cur].req = x;
				tcpipSent[cur].actual = y;
				tcpipSent[cur].req2 = x2;
				tcpipSent[cur].actual2 = y2;
				cur++;
			}

		}
		closeSession: close(session);
		close(serHandle);

		int i;
		for (i = 0; i < 1024; i++)
		{
			diag_printf("%d %d %d %d\n", tcpipSent[i].req, tcpipSent[i].actual,
					tcpipSent[i].req2, tcpipSent[i].actual2);

		}
	}
	close(fd);

}

void startUart(void)
{
	cyg_thread_create(1, zylinjtag_uart, (cyg_addrword_t) 0, "uart thread",
			(void *) uart_stack, sizeof(uart_stack),
			&zylinjtag_uart_thread_handle, &zylinjtag_uart_thread_object);
	cyg_thread_set_priority(zylinjtag_uart_thread_handle, 1); // low priority as it sits in a busy loop
	cyg_thread_resume(zylinjtag_uart_thread_handle);
}

int handle_uart_command(struct command_context_s *cmd_ctx, char *cmd,
		char **args, int argc)
{
	static int current_baud = 38400;
	if (argc == 0)
	{
		command_print(cmd_ctx, "%d", current_baud);
		return ERROR_OK;
	}
	else if (argc != 1)
	{
		return ERROR_INVALID_ARGUMENTS;
	}

	current_baud = atol(args[0]);

	int baud;
	switch (current_baud)
	{
	case 9600:
		baud = CYGNUM_SERIAL_BAUD_9600;
		break;
	case 19200:
		baud = CYGNUM_SERIAL_BAUD_19200;
		break;
	case 38400:
		baud = CYGNUM_SERIAL_BAUD_38400;
		break;
	case 57600:
		baud = CYGNUM_SERIAL_BAUD_57600;
		break;
	case 115200:
		baud = CYGNUM_SERIAL_BAUD_115200;
		break;
	case 230400:
		baud = CYGNUM_SERIAL_BAUD_230400;
		break;
	default:
		command_print(cmd_ctx, "unsupported baudrate");
		return ERROR_INVALID_ARGUMENTS;
	}

	cyg_serial_info_t buf;
	cyg_uint32 len = 1;
	//get existing serial configuration
	len = sizeof(cyg_serial_info_t);
	int err;
	cyg_io_handle_t serial_handle;

	err = cyg_io_lookup("/dev/ser0", &serial_handle);
	if (err != ENOERR)
	{
		LOG_ERROR("/dev/ser0 not found\n");
		return ERROR_FAIL;
	}

	err = cyg_io_get_config(serial_handle,
			CYG_IO_GET_CONFIG_SERIAL_OUTPUT_DRAIN, &buf, &len);
	err = cyg_io_get_config(serial_handle, CYG_IO_GET_CONFIG_SERIAL_INFO, &buf,
			&len);
	if (err != ENOERR)
	{
		command_print(cmd_ctx, "Failed to get serial port settings %d", err);
		return ERROR_OK;
	}
	buf.baud = baud;

	err = cyg_io_set_config(serial_handle, CYG_IO_SET_CONFIG_SERIAL_INFO, &buf,
			&len);
	if (err != ENOERR)
	{
		command_print(cmd_ctx, "Failed to set serial port settings %d", err);
		return ERROR_OK;
	}

	return ERROR_OK;
}

bool logAllToSerial = false;


int boolParam(char *var);


command_context_t *setup_command_handler(void);

extern const char *zylin_config_dir;

int add_default_dirs(void)
{
	add_script_search_dir(zylin_config_dir);
	add_script_search_dir("/rom/lib/openocd");
	add_script_search_dir("/rom");
	return ERROR_OK;
}

int ioutil_init(struct command_context_s *cmd_ctx);

int main(int argc, char *argv[])
{
	/* ramblockdevice will be the same address every time. The deflate app uses a buffer 16mBytes out, so we
	 * need to allocate towards the end of the heap.  */

#ifdef CYGNUM_HAL_VECTOR_UNDEF_INSTRUCTION
	setHandler(CYGNUM_HAL_VECTOR_UNDEF_INSTRUCTION);
	setHandler(CYGNUM_HAL_VECTOR_ABORT_PREFETCH);
	setHandler(CYGNUM_HAL_VECTOR_ABORT_DATA);
#endif

	int err;

	atexit(keep_webserver);

	diag_init_putc(_zylinjtag_diag_write_char);
	// We want this in the log.
	diag_printf("Zylin ZY1000.\n");

	err = mount("", "/ram", "ramfs");
	if (err < 0)
	{
		diag_printf("unable to mount ramfs\n");
	}
	chdir("/ram");

	char address[16];
	sprintf(address, "%p", &filedata[0]);
	err = mount(address, "/rom", "romfs");
	if (err < 0)
	{
		diag_printf("unable to mount /rom\n");
	}

	err = mount("", "/log", "logfs");
	if (err < 0)
	{
		diag_printf("unable to mount logfs\n");
	}

	err = mount("", "/tftp", "tftpfs");
	if (err < 0)
	{
		diag_printf("unable to mount logfs\n");
	}

	log = fopen("/log/log", "w");
	if (log == NULL)
	{
		diag_printf("Could not open log file /ram/log\n");
		exit(-1);
	}


	copydir("/rom", "/ram/cgi");

	err = mount("/dev/flash1", "/config", "jffs2");
	if (err < 0)
	{
		diag_printf("unable to mount jffs2, falling back to ram disk..\n");
		err = mount("", "/config", "ramfs");
		if (err < 0)
		{
			diag_printf("unable to mount /config as ramdisk.\n");
			reboot();
		}
	}
	else
	{
		/* are we using a ram disk instead of a flash disk? This is used
		 * for ZY1000 live demo...
		 *
		 * copy over flash disk to ram block device
		 */
		if (boolParam("ramdisk"))
		{
			diag_printf("Unmounting /config from flash and using ram instead\n");
			err = umount("/config");
			if (err < 0)
			{
				diag_printf("unable to unmount jffs\n");
				reboot();
			}

			err = mount("/dev/flash1", "/config2", "jffs2");
			if (err < 0)
			{
				diag_printf("unable to mount jffs\n");
				reboot();
			}

			err = mount("", "/config", "ramfs");
			if (err < 0)
			{
				diag_printf("unable to mount ram block device\n");
				reboot();
			}

			//		copydir("/config2", "/config");
			copyfile("/config2/ip", "/config/ip");
			copydir("/config2/settings", "/config/settings");

			umount("/config2");
		}
	}

	mkdir(zylin_config_dir, 0777);
	char *dirname=alloc_printf("%s/target", zylin_config_dir);
	mkdir(dirname, 0777);
	free(dirname);
	dirname=alloc_printf("%s/event", zylin_config_dir);
	mkdir(dirname, 0777);
	free(dirname);

	logAllToSerial = boolParam("logserial");

	// We need the network & web server in case there is something wrong with
	// the config files that invoke exit()
	zylinjtag_startNetwork();

	/* we're going to access the jim interpreter from here on... */
	openocd_sleep_postlude();
	startUart();

	add_default_dirs();

	/* initialize commandline interface */
	command_context_t * cmd_ctx;
	cmd_ctx = setup_command_handler();
	command_set_output_handler(cmd_ctx, configuration_output_handler, NULL);
	command_context_mode(cmd_ctx, COMMAND_CONFIG);

#if BUILD_IOUTIL
	if (ioutil_init(cmd_ctx) != ERROR_OK)
	{
		return EXIT_FAILURE;
	}
#endif


#ifdef CYGPKG_PROFILE_GPROF
	register_command(cmd_ctx, NULL, "ecosboard_profile", eCosBoard_handle_eCosBoard_profile_command,
			COMMAND_ANY, NULL);
#endif

	register_command(cmd_ctx, NULL, "uart", handle_uart_command, COMMAND_ANY,
			"uart <baud>  - forward uart on port 5555");

	int errVal;
	errVal = log_init(cmd_ctx);
	if (errVal != ERROR_OK)
	{
		diag_printf("log_init() failed %d\n", errVal);
		exit(-1);
	}

	set_log_output(cmd_ctx, log);

	LOG_DEBUG("log init complete");

	//	diag_printf("Executing config files\n");

	if (logAllToSerial)
	{
		diag_printf(
				 "%s/logserial=1 => sending log output to serial port using \"debug_level 3\" as default.\n", zylin_config_dir);
		command_run_line(cmd_ctx, "debug_level 3");
	}

	command_run_linef(cmd_ctx, "script /rom/openocd.cfg");

	// FIX!!!  Yuk!
	// diag_printf() is really invoked from many more places than we trust it
	// not to cause instabilities(e.g. invoking fputc() from an interrupt is *BAD*).
	//
	// Disabling it here is safe and gives us enough logged debug output for now. Crossing
	// fingers that it doesn't cause any crashes.
	diag_printf("Init complete, GDB & telnet servers launched.\n");
	command_set_output_handler(cmd_ctx,
			zy1000_configuration_output_handler_log, NULL);
	if (!logAllToSerial)
	{
		serialLog = false;
	}

	/* handle network connections */
	server_loop(cmd_ctx);
	openocd_sleep_prelude();

	/* shut server down */
	server_quit();

	/* free commandline interface */
	command_done(cmd_ctx);
	umount("/config");

	exit(0);
	for (;;)
		;
}

cyg_int32 cyg_httpd_exec_cgi_tcl(char *file_name);
cyg_int32 homeForm(CYG_HTTPD_STATE *p)
{
	cyg_httpd_exec_cgi_tcl("/ram/cgi/index.tcl");
	return 0;
}

CYG_HTTPD_HANDLER_TABLE_ENTRY(root_label, "/", homeForm);

CYG_HTTPD_MIME_TABLE_ENTRY(text_mime_label, "text", "text/plain");
CYG_HTTPD_MIME_TABLE_ENTRY(bin_mime_label, "bin", "application/octet-stream");

#include <pkgconf/system.h>
#include <pkgconf/hal.h>
#include <pkgconf/kernel.h>
#include <pkgconf/io_fileio.h>
#include <pkgconf/fs_rom.h>

#include <cyg/kernel/ktypes.h>         // base kernel types
#include <cyg/infra/cyg_trac.h>        // tracing macros
#include <cyg/infra/cyg_ass.h>         // assertion macros
#include <unistd.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/stat.h>
#include <errno.h>
#include <dirent.h>

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <cyg/fileio/fileio.h>

#include <cyg/kernel/kapi.h>
#include <cyg/infra/diag.h>

//==========================================================================
// Eventually we want to eXecute In Place from the ROM in a protected
// environment, so we'll need executables to be aligned to a boundary
// suitable for MMU protection. A suitable boundary would be the 4k
// boundary in all the CPU architectures I am currently aware of.

// Forward definitions

// Filesystem operations
static int tftpfs_mount(cyg_fstab_entry *fste, cyg_mtab_entry *mte);
static int tftpfs_umount(cyg_mtab_entry *mte);
static int tftpfs_open(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
		int mode, cyg_file *fte);
static int tftpfs_fo_read(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);
static int tftpfs_fo_write(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);

// File operations
static int tftpfs_fo_fsync(struct CYG_FILE_TAG *fp, int mode);
static int tftpfs_fo_close(struct CYG_FILE_TAG *fp);
static int tftpfs_fo_lseek(struct CYG_FILE_TAG *fp, off_t *apos, int whence);

//==========================================================================
// Filesystem table entries

// -------------------------------------------------------------------------
// Fstab entry.
// This defines the entry in the filesystem table.
// For simplicity we use _FILESYSTEM synchronization for all accesses since
// we should never block in any filesystem operations.
#if 1
FSTAB_ENTRY( tftpfs_fste, "tftpfs", 0,
		CYG_SYNCMODE_NONE,
		tftpfs_mount,
		tftpfs_umount,
		tftpfs_open,
		(cyg_fsop_unlink *)cyg_fileio_erofs,
		(cyg_fsop_mkdir *)cyg_fileio_erofs,
		(cyg_fsop_rmdir *)cyg_fileio_erofs,
		(cyg_fsop_rename *)cyg_fileio_erofs,
		(cyg_fsop_link *)cyg_fileio_erofs,
		(cyg_fsop_opendir *)cyg_fileio_erofs,
		(cyg_fsop_chdir *)cyg_fileio_erofs,
		(cyg_fsop_stat *)cyg_fileio_erofs,
		(cyg_fsop_getinfo *)cyg_fileio_erofs,
		(cyg_fsop_setinfo *)cyg_fileio_erofs);
#endif

// -------------------------------------------------------------------------
// mtab entry.
// This defines a single ROMFS loaded into ROM at the configured address
//
// MTAB_ENTRY(	rom_mte,	// structure name
// 		"/rom",		// mount point
// 		"romfs",	// FIlesystem type
// 		"",		// hardware device
//  (CYG_ADDRWORD) CYGNUM_FS_ROM_BASE_ADDRESS	// Address in ROM
//           );


// -------------------------------------------------------------------------
// File operations.
// This set of file operations are used for normal open files.

static cyg_fileops tftpfs_fileops =
{ tftpfs_fo_read, tftpfs_fo_write, tftpfs_fo_lseek,
		(cyg_fileop_ioctl *) cyg_fileio_erofs, cyg_fileio_seltrue,
		tftpfs_fo_fsync, tftpfs_fo_close,
		(cyg_fileop_fstat *) cyg_fileio_erofs,
		(cyg_fileop_getinfo *) cyg_fileio_erofs,
		(cyg_fileop_setinfo *) cyg_fileio_erofs, };

// -------------------------------------------------------------------------
// tftpfs_mount()
// Process a mount request. This mainly finds root for the
// filesystem.

static int tftpfs_mount(cyg_fstab_entry *fste, cyg_mtab_entry *mte)
{
	return ENOERR;
}

static int tftpfs_umount(cyg_mtab_entry *mte)
{
	return ENOERR;
}

struct Tftp
{
	int write;
	int readFile;
	cyg_uint8 *mem;
	int actual;
	char *server;
	char *file;
};

static void freeTftp(struct Tftp *t)
{
	if (t == NULL)
		return;
	if (t->mem)
		free(t->mem);
	if (t->server)
		free(t->server);
	if (t->file)
		free(t->file);
	free(t);
}

static const int tftpMaxSize = 8192 * 1024;
static int tftpfs_open(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
		int mode, cyg_file *file)
{
	struct Tftp *tftp;
	tftp = malloc(sizeof(struct Tftp));
	if (tftp == NULL)
		return EMFILE;
	memset(tftp, 0, sizeof(struct Tftp));

	file->f_flag |= mode & CYG_FILE_MODE_MASK;
	file->f_type = CYG_FILE_TYPE_FILE;
	file->f_ops = &tftpfs_fileops;
	file->f_offset = 0;
	file->f_data = 0;
	file->f_xops = 0;

	tftp->mem = malloc(tftpMaxSize);
	if (tftp->mem == NULL)
	{
		freeTftp(tftp);
		return EMFILE;
	}

	char *server = strchr(name, '/');
	if (server == NULL)
	{
		freeTftp(tftp);
		return EMFILE;
	}

	tftp->server = malloc(server - name + 1);
	if (tftp->server == NULL)
	{
		freeTftp(tftp);
		return EMFILE;
	}
	strncpy(tftp->server, name, server - name);
	tftp->server[server - name] = 0;

	tftp->file = strdup(server + 1);
	if (tftp->file == NULL)
	{
		freeTftp(tftp);
		return EMFILE;
	}

	file->f_data = (CYG_ADDRWORD) tftp;

	return ENOERR;
}

static int fetchTftp(struct Tftp *tftp)
{
	if (!tftp->readFile)
	{
		int err;
		tftp->actual = tftp_client_get(tftp->file, tftp->server, 0, tftp->mem,
				tftpMaxSize, TFTP_OCTET, &err);

		if (tftp->actual < 0)
		{
			return EMFILE;
		}
		tftp->readFile = 1;
	}
	return ENOERR;
}

// -------------------------------------------------------------------------
// tftpfs_fo_write()
// Read data from file.

static int tftpfs_fo_read(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
	struct Tftp *tftp = (struct Tftp *) fp->f_data;

	if (fetchTftp(tftp) != ENOERR)
		return EMFILE;

	int i;
	off_t pos = fp->f_offset;
	int resid = 0;
	for (i = 0; i < uio->uio_iovcnt; i++)
	{
		cyg_iovec *iov = &uio->uio_iov[i];
		char *buf = (char *) iov->iov_base;
		off_t len = iov->iov_len;

		if (len + pos > tftp->actual)
		{
			len = tftp->actual - pos;
		}
		resid += iov->iov_len - len;

		memcpy(buf, tftp->mem + pos, len);
		pos += len;

	}
	uio->uio_resid = resid;
	fp->f_offset = pos;

	return ENOERR;
}

static int tftpfs_fo_write(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
	struct Tftp *tftp = (struct Tftp *) fp->f_data;

	int i;
	off_t pos = fp->f_offset;
	int resid = 0;
	for (i = 0; i < uio->uio_iovcnt; i++)
	{
		cyg_iovec *iov = &uio->uio_iov[i];
		char *buf = (char *) iov->iov_base;
		off_t len = iov->iov_len;

		if (len + pos > tftpMaxSize)
		{
			len = tftpMaxSize - pos;
		}
		resid += iov->iov_len - len;

		memcpy(tftp->mem + pos, buf, len);
		pos += len;

	}
	uio->uio_resid = resid;
	fp->f_offset = pos;

	tftp->write = 1;

	return ENOERR;
}

static int tftpfs_fo_fsync(struct CYG_FILE_TAG *fp, int mode)
{
	int error = ENOERR;
	return error;
}

// -------------------------------------------------------------------------
// romfs_fo_close()
// Close a file. We just clear out the data pointer.

static int tftpfs_fo_close(struct CYG_FILE_TAG *fp)
{
	struct Tftp *tftp = (struct Tftp *) fp->f_data;
	int error = ENOERR;

	if (tftp->write)
	{
		tftp_client_put(tftp->file, tftp->server, 0, tftp->mem, fp->f_offset,
				TFTP_OCTET, &error);
	}

	freeTftp(tftp);
	fp->f_data = 0;
	return error;
}

// -------------------------------------------------------------------------
// romfs_fo_lseek()
// Seek to a new file position.

static int tftpfs_fo_lseek(struct CYG_FILE_TAG *fp, off_t *apos, int whence)
{
	struct Tftp *tftp = (struct Tftp *) fp->f_data;
	off_t pos = *apos;

	if (fetchTftp(tftp) != ENOERR)
		return EMFILE;

	switch (whence)
	{
	case SEEK_SET:
		// Pos is already where we want to be.
		break;

	case SEEK_CUR:
		// Add pos to current offset.
		pos += fp->f_offset;
		break;

	case SEEK_END:
		// Add pos to file size.
		pos += tftp->actual;
		break;

	default:
		return EINVAL;
	}

	// Check that pos is still within current file size, or at the
	// very end.
	if (pos < 0 || pos > tftp->actual)
		return EINVAL;

	// All OK, set fp offset and return new position.
	*apos = fp->f_offset = pos;

	return ENOERR;
}

void usleep(int us)
{
	if (us > 10000)
		cyg_thread_delay(us / 10000 + 1);
	else
		HAL_DELAY_US(us);
}

// Chunked version.
cyg_int32 show_log_entry(CYG_HTTPD_STATE *phttpstate)
{
	cyg_httpd_start_chunked("text");
	if (logCount >= logSize)
	{
		cyg_httpd_write_chunked(logBuffer + logCount % logSize, logSize
				- logCount % logSize);
	}
	cyg_httpd_write_chunked(logBuffer, writePtr);
	cyg_httpd_end_chunked();
	return -1;
}

CYG_HTTPD_HANDLER_TABLE_ENTRY(show_log, "/ram/log", show_log_entry);

// Filesystem operations
static int logfs_mount(cyg_fstab_entry *fste, cyg_mtab_entry *mte);
static int logfs_umount(cyg_mtab_entry *mte);
static int logfs_open(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
		int mode, cyg_file *fte);
static int logfs_fo_write(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio);

// File operations
static int logfs_fo_fsync(struct CYG_FILE_TAG *fp, int mode);
static int logfs_fo_close(struct CYG_FILE_TAG *fp);

#include <cyg/io/devtab.h>

//==========================================================================
// Filesystem table entries

// -------------------------------------------------------------------------
// Fstab entry.
// This defines the entry in the filesystem table.
// For simplicity we use _FILESYSTEM synchronization for all accesses since
// we should never block in any filesystem operations.
FSTAB_ENTRY( logfs_fste, "logfs", 0,
		CYG_SYNCMODE_FILE_FILESYSTEM|CYG_SYNCMODE_IO_FILESYSTEM,
		logfs_mount,
		logfs_umount,
		logfs_open,
		(cyg_fsop_unlink *)cyg_fileio_erofs,
		(cyg_fsop_mkdir *)cyg_fileio_erofs,
		(cyg_fsop_rmdir *)cyg_fileio_erofs,
		(cyg_fsop_rename *)cyg_fileio_erofs,
		(cyg_fsop_link *)cyg_fileio_erofs,
		(cyg_fsop_opendir *)cyg_fileio_erofs,
		(cyg_fsop_chdir *)cyg_fileio_erofs,
		(cyg_fsop_stat *)cyg_fileio_erofs,
		(cyg_fsop_getinfo *)cyg_fileio_erofs,
		(cyg_fsop_setinfo *)cyg_fileio_erofs);

// -------------------------------------------------------------------------
// File operations.
// This set of file operations are used for normal open files.

static cyg_fileops logfs_fileops =
{ (cyg_fileop_read *) cyg_fileio_erofs, (cyg_fileop_write *) logfs_fo_write,
		(cyg_fileop_lseek *) cyg_fileio_erofs,
		(cyg_fileop_ioctl *) cyg_fileio_erofs, cyg_fileio_seltrue,
		logfs_fo_fsync, logfs_fo_close, (cyg_fileop_fstat *) cyg_fileio_erofs,
		(cyg_fileop_getinfo *) cyg_fileio_erofs,
		(cyg_fileop_setinfo *) cyg_fileio_erofs, };

// -------------------------------------------------------------------------
// logfs_mount()
// Process a mount request. This mainly finds root for the
// filesystem.

static int logfs_mount(cyg_fstab_entry *fste, cyg_mtab_entry *mte)
{
	return ENOERR;
}

static int logfs_umount(cyg_mtab_entry *mte)
{
	return ENOERR;
}

static int logfs_open(cyg_mtab_entry *mte, cyg_dir dir, const char *name,
		int mode, cyg_file *file)
{
	file->f_flag |= mode & CYG_FILE_MODE_MASK;
	file->f_type = CYG_FILE_TYPE_FILE;
	file->f_ops = &logfs_fileops;
	file->f_offset = 0;
	file->f_data = 0;
	file->f_xops = 0;
	return ENOERR;
}

// -------------------------------------------------------------------------
// logfs_fo_write()
// Write data to file.

static int logfs_fo_write(struct CYG_FILE_TAG *fp, struct CYG_UIO_TAG *uio)
{
	int i;
	for (i = 0; i < uio->uio_iovcnt; i++)
	{
		cyg_iovec *iov = &uio->uio_iov[i];
		char *buf = (char *) iov->iov_base;
		off_t len = iov->iov_len;

		diag_write(buf, len);
	}
	uio->uio_resid = 0;

	return ENOERR;
}
static int logfs_fo_fsync(struct CYG_FILE_TAG *fp, int mode)
{
	return ENOERR;
}

// -------------------------------------------------------------------------
// romfs_fo_close()
// Close a file. We just clear out the data pointer.

static int logfs_fo_close(struct CYG_FILE_TAG *fp)
{
	return ENOERR;
}

