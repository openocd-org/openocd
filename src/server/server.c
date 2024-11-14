// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2010 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "server.h"
#include <helper/time_support.h>
#include <target/target.h>
#include <target/target_request.h>
#include <target/openrisc/jsp_server.h>
#include "openocd.h"
#include "tcl_server.h"
#include "telnet_server.h"

#include <signal.h>

#ifdef HAVE_NETDB_H
#include <netdb.h>
#endif

#ifndef _WIN32
#include <netinet/tcp.h>
#endif

static struct service *services;

enum shutdown_reason {
	CONTINUE_MAIN_LOOP,			/* stay in main event loop */
	SHUTDOWN_REQUESTED,			/* set by shutdown command; exit the event loop and quit the debugger */
	SHUTDOWN_WITH_ERROR_CODE,	/* set by shutdown command; quit with non-zero return code */
	SHUTDOWN_WITH_SIGNAL_CODE	/* set by sig_handler; exec shutdown then exit with signal as return code */
};
static enum shutdown_reason shutdown_openocd = CONTINUE_MAIN_LOOP;

/* store received signal to exit application by killing ourselves */
static int last_signal;

/* set the polling period to 100ms */
static int polling_period = 100;

/* address by name on which to listen for incoming TCP/IP connections */
static char *bindto_name;

static int add_connection(struct service *service, struct command_context *cmd_ctx)
{
	socklen_t address_size;
	struct connection *c, **p;
	int retval;
	int flag = 1;

	c = malloc(sizeof(struct connection));
	c->fd = -1;
	c->fd_out = -1;
	memset(&c->sin, 0, sizeof(c->sin));
	c->cmd_ctx = copy_command_context(cmd_ctx);
	c->service = service;
	c->input_pending = false;
	c->priv = NULL;
	c->next = NULL;

	if (service->type == CONNECTION_TCP) {
		address_size = sizeof(c->sin);

		c->fd = accept(service->fd, (struct sockaddr *)&service->sin, &address_size);
		c->fd_out = c->fd;

		/* This increases performance dramatically for e.g. GDB load which
		 * does not have a sliding window protocol.
		 *
		 * Ignore errors from this fn as it probably just means less performance
		 */
		setsockopt(c->fd,	/* socket affected */
			IPPROTO_TCP,			/* set option at TCP level */
			TCP_NODELAY,			/* name of option */
			(char *)&flag,			/* the cast is historical cruft */
			sizeof(int));			/* length of option value */

		LOG_INFO("accepting '%s' connection on tcp/%s", service->name, service->port);
		retval = service->new_connection(c);
		if (retval != ERROR_OK) {
			close_socket(c->fd);
			LOG_ERROR("attempted '%s' connection rejected", service->name);
			command_done(c->cmd_ctx);
			free(c);
			return retval;
		}
	} else if (service->type == CONNECTION_STDINOUT) {
		c->fd = service->fd;
		c->fd_out = fileno(stdout);

#ifdef _WIN32
		/* we are using stdin/out so ignore ctrl-c under windoze */
		SetConsoleCtrlHandler(NULL, TRUE);
#endif

		/* do not check for new connections again on stdin */
		service->fd = -1;

		LOG_INFO("accepting '%s' connection from pipe", service->name);
		retval = service->new_connection(c);
		if (retval != ERROR_OK) {
			LOG_ERROR("attempted '%s' connection rejected", service->name);
			command_done(c->cmd_ctx);
			free(c);
			return retval;
		}
	} else if (service->type == CONNECTION_PIPE) {
		c->fd = service->fd;
		/* do not check for new connections again on stdin */
		service->fd = -1;

		char *out_file = alloc_printf("%so", service->port);
		c->fd_out = open(out_file, O_WRONLY);
		free(out_file);
		if (c->fd_out == -1) {
			LOG_ERROR("could not open %s", service->port);
			command_done(c->cmd_ctx);
			free(c);
			return ERROR_FAIL;
		}

		LOG_INFO("accepting '%s' connection from pipe %s", service->name, service->port);
		retval = service->new_connection(c);
		if (retval != ERROR_OK) {
			LOG_ERROR("attempted '%s' connection rejected", service->name);
			command_done(c->cmd_ctx);
			free(c);
			return retval;
		}
	}

	/* add to the end of linked list */
	for (p = &service->connections; *p; p = &(*p)->next)
		;
	*p = c;

	if (service->max_connections != CONNECTION_LIMIT_UNLIMITED)
		service->max_connections--;

	return ERROR_OK;
}

static int remove_connection(struct service *service, struct connection *connection)
{
	struct connection **p = &service->connections;
	struct connection *c;

	/* find connection */
	while ((c = *p)) {
		if (c->fd == connection->fd) {
			service->connection_closed(c);
			if (service->type == CONNECTION_TCP)
				close_socket(c->fd);
			else if (service->type == CONNECTION_PIPE) {
				/* The service will listen to the pipe again */
				c->service->fd = c->fd;
			}

			command_done(c->cmd_ctx);

			/* delete connection */
			*p = c->next;
			free(c);

			if (service->max_connections != CONNECTION_LIMIT_UNLIMITED)
				service->max_connections++;

			break;
		}

		/* redirect p to next list pointer */
		p = &(*p)->next;
	}

	return ERROR_OK;
}

static void free_service(struct service *c)
{
	free(c->name);
	free(c->port);
	free(c);
}

int add_service(const struct service_driver *driver, const char *port,
		int max_connections, void *priv)
{
	struct service *c, **p;
	struct hostent *hp;
	int so_reuseaddr_option = 1;

	c = malloc(sizeof(struct service));

	c->name = strdup(driver->name);
	c->port = strdup(port);
	c->max_connections = 1;	/* Only TCP/IP ports can support more than one connection */
	c->fd = -1;
	c->connections = NULL;
	c->new_connection_during_keep_alive = driver->new_connection_during_keep_alive_handler;
	c->new_connection = driver->new_connection_handler;
	c->input = driver->input_handler;
	c->connection_closed = driver->connection_closed_handler;
	c->keep_client_alive = driver->keep_client_alive_handler;
	c->priv = priv;
	c->next = NULL;
	long portnumber;
	if (strcmp(c->port, "pipe") == 0)
		c->type = CONNECTION_STDINOUT;
	else {
		char *end;
		portnumber = strtol(c->port, &end, 0);
		if (!*end && (parse_long(c->port, &portnumber) == ERROR_OK)) {
			c->portnumber = portnumber;
			c->type = CONNECTION_TCP;
		} else
			c->type = CONNECTION_PIPE;
	}

	if (c->type == CONNECTION_TCP) {
		c->max_connections = max_connections;

		c->fd = socket(AF_INET, SOCK_STREAM, 0);
		if (c->fd == -1) {
			LOG_ERROR("error creating socket: %s", strerror(errno));
			free_service(c);
			return ERROR_FAIL;
		}

		setsockopt(c->fd,
			SOL_SOCKET,
			SO_REUSEADDR,
			(void *)&so_reuseaddr_option,
			sizeof(int));

		socket_nonblock(c->fd);

		memset(&c->sin, 0, sizeof(c->sin));
		c->sin.sin_family = AF_INET;

		if (!bindto_name)
			c->sin.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		else {
			hp = gethostbyname(bindto_name);
			if (!hp) {
				LOG_ERROR("couldn't resolve bindto address: %s", bindto_name);
				close_socket(c->fd);
				free_service(c);
				return ERROR_FAIL;
			}
			memcpy(&c->sin.sin_addr, hp->h_addr_list[0], hp->h_length);
		}
		c->sin.sin_port = htons(c->portnumber);

		if (bind(c->fd, (struct sockaddr *)&c->sin, sizeof(c->sin)) == -1) {
			LOG_ERROR("couldn't bind %s to socket on port %d: %s", c->name, c->portnumber, strerror(errno));
			close_socket(c->fd);
			free_service(c);
			return ERROR_FAIL;
		}

#ifndef _WIN32
		int segsize = 65536;
		setsockopt(c->fd, IPPROTO_TCP, TCP_MAXSEG,  &segsize, sizeof(int));
#endif
		int window_size = 128 * 1024;

		/* These setsockopt()s must happen before the listen() */

		setsockopt(c->fd, SOL_SOCKET, SO_SNDBUF,
			(char *)&window_size, sizeof(window_size));
		setsockopt(c->fd, SOL_SOCKET, SO_RCVBUF,
			(char *)&window_size, sizeof(window_size));

		if (listen(c->fd, 1) == -1) {
			LOG_ERROR("couldn't listen on socket: %s", strerror(errno));
			close_socket(c->fd);
			free_service(c);
			return ERROR_FAIL;
		}

		struct sockaddr_in addr_in;
		addr_in.sin_port = 0;
		socklen_t addr_in_size = sizeof(addr_in);
		if (getsockname(c->fd, (struct sockaddr *)&addr_in, &addr_in_size) == 0)
			LOG_INFO("Listening on port %hu for %s connections",
				 ntohs(addr_in.sin_port), c->name);
	} else if (c->type == CONNECTION_STDINOUT) {
		c->fd = fileno(stdin);

#ifdef _WIN32
		/* for win32 set stdin/stdout to binary mode */
		if (_setmode(_fileno(stdout), _O_BINARY) < 0)
			LOG_WARNING("cannot change stdout mode to binary");
		if (_setmode(_fileno(stdin), _O_BINARY) < 0)
			LOG_WARNING("cannot change stdin mode to binary");
		if (_setmode(_fileno(stderr), _O_BINARY) < 0)
			LOG_WARNING("cannot change stderr mode to binary");
#else
		socket_nonblock(c->fd);
#endif
	} else if (c->type == CONNECTION_PIPE) {
#ifdef _WIN32
		/* we currently do not support named pipes under win32
		 * so exit openocd for now */
		LOG_ERROR("Named pipes currently not supported under this os");
		free_service(c);
		return ERROR_FAIL;
#else
		/* Pipe we're reading from */
		c->fd = open(c->port, O_RDONLY | O_NONBLOCK);
		if (c->fd == -1) {
			LOG_ERROR("could not open %s", c->port);
			free_service(c);
			return ERROR_FAIL;
		}
#endif
	}

	/* add to the end of linked list */
	for (p = &services; *p; p = &(*p)->next)
		;
	*p = c;

	return ERROR_OK;
}

static void remove_connections(struct service *service)
{
	struct connection *connection;

	connection = service->connections;

	while (connection) {
		struct connection *tmp;

		tmp = connection->next;
		remove_connection(service, connection);
		connection = tmp;
	}
}

int remove_service(const char *name, const char *port)
{
	struct service *tmp;
	struct service *prev;

	prev = services;

	for (tmp = services; tmp; prev = tmp, tmp = tmp->next) {
		if (!strcmp(tmp->name, name) && !strcmp(tmp->port, port)) {
			remove_connections(tmp);

			if (tmp == services)
				services = tmp->next;
			else
				prev->next = tmp->next;

			if (tmp->type != CONNECTION_STDINOUT)
				close_socket(tmp->fd);

			free(tmp->priv);
			free_service(tmp);

			return ERROR_OK;
		}
	}

	return ERROR_OK;
}

static int remove_services(void)
{
	struct service *c = services;

	/* loop service */
	while (c) {
		struct service *next = c->next;

		remove_connections(c);

		free(c->name);

		if (c->type == CONNECTION_PIPE) {
			if (c->fd != -1)
				close(c->fd);
		}
		free(c->port);
		free(c->priv);
		/* delete service */
		free(c);

		/* remember the last service for unlinking */
		c = next;
	}

	services = NULL;

	return ERROR_OK;
}

void server_keep_clients_alive(void)
{
	for (struct service *s = services; s; s = s->next)
		if (s->keep_client_alive)
			for (struct connection *c = s->connections; c; c = c->next)
				s->keep_client_alive(c);
}

/* server_loop：负责gdb server的主循环，监听服务的文件描述符并处理连接请求，
 * 直到gdb server关闭，用于管理服务和连接的I/O操作
 */
int server_loop(struct command_context *command_context)
{
	struct service *service; //指向服务的指针，用于遍历和操作不同的服务

	bool poll_ok = true; //布尔值，表示轮询状态

	/* used in select() */
	fd_set read_fds; //fd_set类型，用于select系统调用，表示监听的文件描述符集合
	int fd_max; //表示文件描述符的最大值，用于select系统调用

	/* used in accept() */
	int retval; //用于存储函数调用的返回值

	//下一次时间的时间戳(当前时间加上轮询周期)，用于定时时间处理
	int64_t next_event = timeval_ms() + polling_period; 

/* 如果不是win32，将SIGPIPE信号设置为忽略
 * 当尝试向一个关闭的socket连接写数据时，会触发SIGPIPE，将其忽略避免程序异常退出
 */
#ifndef _WIN32
	if (signal(SIGPIPE, SIG_IGN) == SIG_ERR)
		LOG_ERROR("couldn't set SIGPIPE to SIG_IGN");
#endif

	//服务器进入主循环
	while (shutdown_openocd == CONTINUE_MAIN_LOOP) {
		/* monitor sockets for activity */
		//清空read_fds，并将fd_max设置为0，准备重新添加需要监听的文件描述符
		fd_max = 0;
		FD_ZERO(&read_fds);

		/* add service and connection fds to read_fds */
		/* 遍历所有服务，将它们的文件描述符service->next添加到read_fds集合，以便监听
		 * 更新fd_max以确保它是当前文件描述符的最大值
		 * 检查服务的connect,如果存在连接，则进一步处理连接的文件描述符
		*/
		for (service = services; service; service = service->next) {
			if (service->fd != -1) {
				/* listen for new connections */
				FD_SET(service->fd, &read_fds);

				if (service->fd > fd_max)
					fd_max = service->fd;
			}

			if (service->connections) {
				struct connection *c;
                
				//遍历所有连接，并将每个连接的文件描述符c->fd添加到read_fds集合中，以便监听
				//更新fd_max，确保fd_max为文件描述符最大值
				for (c = service->connections; c; c = c->next) {
					/* check for activity on the connection */
					FD_SET(c->fd, &read_fds);
					if (c->fd > fd_max)
						fd_max = c->fd;
				}
			}
		}

		struct timeval tv;
		tv.tv_sec = 0;
		if (poll_ok) {
			/* we're just polling this iteration, this is faster on embedded
			 * hosts */
			tv.tv_usec = 0;//tv.tv_sec = 0和tv.tv_usec = 0表示0超时，即立即返回，相当于非阻塞轮询
			
			//调用socket_select监听文件描述符集合中的活动，并将结果存储在reval中
			retval = socket_select(fd_max + 1, &read_fds, NULL, NULL, &tv);
		} else {
			/* Timeout socket_select() when a target timer expires or every polling_period */
			//如果poll_ok为false，则根据next_event和当前时间计算超时时间timeout_ms，用于控制select的阻塞时长
			int timeout_ms = next_event - timeval_ms();
			if (timeout_ms < 0)
				timeout_ms = 0;
			else if (timeout_ms > polling_period)  
				timeout_ms = polling_period;   //最大等待时间
			tv.tv_usec = timeout_ms * 1000; //微秒数
			/* Only while we're sleeping we'll let others run */
			//调用socket_select以阻塞模式等待，直到有文件描述符准备好或超时时间到达
			retval = socket_select(fd_max + 1, &read_fds, NULL, NULL, &tv);
		}
        /* 错误处理：检查retval是否为-1，表示select调用失败
		 * 在select失败的情况下，检查错误类型，确保了在非严重错误时服务器能够正常运行，
		 * 而在严重错误时，停止循环并返回错误
		 */
		if (retval == -1) {
#ifdef _WIN32

			errno = WSAGetLastError();  //获取select错误代码

			if (errno == WSAEINTR)  //错误是WSAEINTR，清空read_fds文件描述符集合
				FD_ZERO(&read_fds);
			else {  //记录日志并返回ERROR_FAIL，表示不可恢复的错误
				LOG_ERROR("error during select: %s", strerror(errno)); 
				return ERROR_FAIL;
			}
#else

			if (errno == EINTR)
				FD_ZERO(&read_fds);
			else {
				LOG_ERROR("error during select: %s", strerror(errno));
				return ERROR_FAIL;
			}
#endif
		}
        
		/* 超时检测：retval == 0表示select 超时，没有任何文件描述符变为可用
		 * 超时处理：
		 * target_call_timer_callbacks()执行已到期的定时器回调函数
		 * target_timer_next_event()获取下一次事件的时间戳，更新next_event
		 * process_jim_events(command_context)处理Jim TCL事件
		 * FD_ZERO(&read_fds)清空read_fds,因为在某些操作系统中，read_fds可能在超时后保持不变
		 * 设置轮询标志：将poll_ok设置为false，表示下次循环使用阻塞等待模式
		 * 如果 retval不为0，表示有事件发生，则设置为true，下次循环使用非阻塞轮询模式
		*/
		if (retval == 0) {
			/* Execute callbacks of expired timers when
			 * - there was nothing to do if poll_ok was true
			 * - socket_select() timed out if poll_ok was false, now one or more
			 *   timers expired or the polling period elapsed
			 */
			target_call_timer_callbacks();
			next_event = target_timer_next_event();
			process_jim_events(command_context);

			FD_ZERO(&read_fds);	/* eCos leaves read_fds unchanged in this case!  */

			/* We timed out/there was nothing to do, timeout rather than poll next time
			 **/
			poll_ok = false;
		} else {
			/* There was something to do, next time we'll just poll */
			poll_ok = true;
		}

		/* This is a simple back-off algorithm where we immediately
		 * re-poll if we did something this time around.
		 *
		 * This greatly improves performance of DCC.
		 * 如果target_got_message()返回true，表示有未处理的目标消息，则将poll_OK设置为true,以便尽快处理
		 */
		poll_ok = poll_ok || target_got_message();
        
		/* 遍历所有服务,处理监听上的新连接请求
		 * 如果服务的max_connections不是-1，调用add_connection函数增加新连接
		 * 如果服务类型为CONNECTION_TCP，拒绝新连接(通过accept获取连接后立即关闭)，并记录日志
		*/
		for (service = services; service; service = service->next) {
			/* handle new connections on listeners */
			if ((service->fd != -1)
				&& (FD_ISSET(service->fd, &read_fds))) {
				if (service->max_connections != 0)
					add_connection(service, command_context);
				else {
					if (service->type == CONNECTION_TCP) {
						struct sockaddr_in sin;
						socklen_t address_size = sizeof(sin);
						int tmp_fd;
						tmp_fd = accept(service->fd,
								(struct sockaddr *)&service->sin,
								&address_size);
						close_socket(tmp_fd);
					}
					LOG_INFO(
						"rejected '%s' connection, no more connections allowed",
						service->name);
				}
			}

			/* handle activity on connections */
			/* 遍历服务的所有活动连接，检查连线是否在read_fds中设置了活动标志或存在input_pending
			 * 对每个活跃的连接调用service->input函数处理输入数据
			 *
			*/
			if (service->connections) {
				struct connection *c;

				for (c = service->connections; c; ) {
					if ((c->fd >= 0 && FD_ISSET(c->fd, &read_fds)) || c->input_pending) {
						retval = service->input(c);
						if (retval != ERROR_OK) {
							struct connection *next = c->next;
							if (service->type == CONNECTION_PIPE ||
									service->type == CONNECTION_STDINOUT) {
								/* if connection uses a pipe then
								 * shutdown openocd on error */
								shutdown_openocd = SHUTDOWN_REQUESTED;  //触发服务器关闭
							}
							remove_connection(service, c);
							LOG_INFO("dropped '%s' connection",
								service->name);
							c = next;  //继续处理
							continue;
						}
					}
					c = c->next;
				}
			}
		}

/* WIN32平台消息处理:
 * 使用PeekMessage检查消息队列中的消息
*/
#ifdef _WIN32
		MSG msg;
		while (PeekMessage(&msg, NULL, 0, 0, PM_REMOVE)) {
			if (msg.message == WM_QUIT)
				shutdown_openocd = SHUTDOWN_WITH_SIGNAL_CODE; //请求服务器关闭
		}
#endif
	}

	/* when quit for signal or CTRL-C, run (eventually user implemented) "shutdown" */
	//处理退出信号
	if (shutdown_openocd == SHUTDOWN_WITH_SIGNAL_CODE)
		command_run_line(command_context, "shutdown"); //触发服务器关闭流程
    
	//返回退出状态:异常退出/正常退出
	return shutdown_openocd == SHUTDOWN_WITH_ERROR_CODE ? ERROR_FAIL : ERROR_OK;  
}

static void sig_handler(int sig)
{
	/* store only first signal that hits us */
	if (shutdown_openocd == CONTINUE_MAIN_LOOP) {
		shutdown_openocd = SHUTDOWN_WITH_SIGNAL_CODE;
		last_signal = sig;
		LOG_DEBUG("Terminating on Signal %d", sig);
	} else
		LOG_DEBUG("Ignored extra Signal %d", sig);
}


#ifdef _WIN32
BOOL WINAPI control_handler(DWORD ctrl_type)
{
	shutdown_openocd = SHUTDOWN_WITH_SIGNAL_CODE;
	return TRUE;
}
#else
static void sigkey_handler(int sig)
{
	/* ignore keystroke generated signals if not in foreground process group */

	if (tcgetpgrp(STDIN_FILENO) > 0)
		sig_handler(sig);
	else
		LOG_DEBUG("Ignored Signal %d", sig);
}
#endif


int server_host_os_entry(void)
{
	/* this currently only calls WSAStartup on native win32 systems
	 * before any socket operations are performed.
	 * This is an issue if you call init in your config script */

#ifdef _WIN32
	WORD version_requested;
	WSADATA wsadata;

	version_requested = MAKEWORD(2, 2);

	if (WSAStartup(version_requested, &wsadata) != 0) {
		LOG_ERROR("Failed to Open Winsock");
		return ERROR_FAIL;
	}
#endif
	return ERROR_OK;
}

int server_host_os_close(void)
{
#ifdef _WIN32
	WSACleanup();
#endif
	return ERROR_OK;
}

int server_preinit(void)
{
#ifdef _WIN32
	/* register ctrl-c handler */
	SetConsoleCtrlHandler(control_handler, TRUE);

	signal(SIGBREAK, sig_handler);
	signal(SIGINT, sig_handler);
#else
	signal(SIGHUP, sig_handler);
	signal(SIGPIPE, sig_handler);
	signal(SIGQUIT, sigkey_handler);
	signal(SIGINT, sigkey_handler);
#endif
	signal(SIGTERM, sig_handler);
	signal(SIGABRT, sig_handler);

	return ERROR_OK;
}

int server_init(struct command_context *cmd_ctx)
{
	int ret = tcl_init();

	if (ret != ERROR_OK)
		return ret;

	ret = telnet_init("Open On-Chip Debugger");

	if (ret != ERROR_OK) {
		remove_services();
		return ret;
	}

	return ERROR_OK;
}

int server_quit(void)
{
	remove_services();
	target_quit();

#ifdef _WIN32
	SetConsoleCtrlHandler(control_handler, FALSE);

	return ERROR_OK;
#endif

	/* return signal number so we can kill ourselves */
	return last_signal;
}

void server_free(void)
{
	tcl_service_free();
	telnet_service_free();
	jsp_service_free();

	free(bindto_name);
}

void exit_on_signal(int sig)
{
#ifndef _WIN32
	/* bring back default system handler and kill yourself */
	signal(sig, SIG_DFL);
	kill(getpid(), sig);
#endif
}

int connection_write(struct connection *connection, const void *data, int len)
{
	if (len == 0) {
		/* successful no-op. Sockets and pipes behave differently here... */
		return 0;
	}
	if (connection->service->type == CONNECTION_TCP)
		return write_socket(connection->fd_out, data, len);
	else
		return write(connection->fd_out, data, len);
}

int connection_read(struct connection *connection, void *data, int len)
{
	if (connection->service->type == CONNECTION_TCP)
		return read_socket(connection->fd, data, len);
	else
		return read(connection->fd, data, len);
}

bool openocd_is_shutdown_pending(void)
{
	return shutdown_openocd != CONTINUE_MAIN_LOOP;
}

/* tell the server we want to shut down */
COMMAND_HANDLER(handle_shutdown_command)
{
	LOG_USER("shutdown command invoked");

	shutdown_openocd = SHUTDOWN_REQUESTED;

	command_run_line(CMD_CTX, "_run_pre_shutdown_commands");

	if (CMD_ARGC == 1) {
		if (!strcmp(CMD_ARGV[0], "error")) {
			shutdown_openocd = SHUTDOWN_WITH_ERROR_CODE;
			return ERROR_FAIL;
		}
	}

	return ERROR_COMMAND_CLOSE_CONNECTION;
}

COMMAND_HANDLER(handle_poll_period_command)
{
	if (CMD_ARGC == 0)
		LOG_WARNING("You need to set a period value");
	else
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], polling_period);

	LOG_INFO("set servers polling period to %ums", polling_period);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_bindto_command)
{
	switch (CMD_ARGC) {
		case 0:
			command_print(CMD, "bindto name: %s", bindto_name);
			break;
		case 1:
			free(bindto_name);
			bindto_name = strdup(CMD_ARGV[0]);
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return ERROR_OK;
}

static const struct command_registration server_command_handlers[] = {
	{
		.name = "shutdown",
		.handler = &handle_shutdown_command,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "shut the server down",
	},
	{
		.name = "poll_period",
		.handler = &handle_poll_period_command,
		.mode = COMMAND_ANY,
		.usage = "",
		.help = "set the servers polling period",
	},
	{
		.name = "bindto",
		.handler = &handle_bindto_command,
		.mode = COMMAND_CONFIG,
		.usage = "[name]",
		.help = "Specify address by name on which to listen for "
			"incoming TCP/IP connections",
	},
	COMMAND_REGISTRATION_DONE
};

int server_register_commands(struct command_context *cmd_ctx)
{
	int retval = telnet_register_commands(cmd_ctx);
	if (retval != ERROR_OK)
		return retval;

	retval = tcl_register_commands(cmd_ctx);
	if (retval != ERROR_OK)
		return retval;

	retval = jsp_register_commands(cmd_ctx);
	if (retval != ERROR_OK)
		return retval;

	return register_commands(cmd_ctx, NULL, server_command_handlers);
}

COMMAND_HELPER(server_port_command, unsigned short *out)
{
	switch (CMD_ARGC) {
		case 0:
			command_print(CMD, "%d", *out);
			break;
		case 1:
		{
			uint16_t port;
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], port);
			*out = port;
			break;
		}
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return ERROR_OK;
}

COMMAND_HELPER(server_pipe_command, char **out)
{
	switch (CMD_ARGC) {
		case 0:
			command_print(CMD, "%s", *out);
			break;
		case 1:
		{
			if (CMD_CTX->mode == COMMAND_EXEC) {
				LOG_WARNING("unable to change server port after init");
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}
			free(*out);
			*out = strdup(CMD_ARGV[0]);
			break;
		}
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return ERROR_OK;
}
