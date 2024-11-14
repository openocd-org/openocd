// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2010 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 Richard Missenden                                  *
 *   richard.missenden@googlemail.com                                      *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif//项目配置文件

//openocd中所需的各种模块和组件，如JTAG接口、传输层、工具函数等
#include "openocd.h"
#include <jtag/adapter.h>
#include <jtag/jtag.h>
#include <transport/transport.h>
#include <helper/util.h>
#include <helper/configuration.h>
#include <flash/nor/core.h>
#include <flash/nand/core.h>
#include <pld/pld.h>
#include <target/arm_cti.h>
#include <target/arm_adi_v5.h>
#include <target/arm_tpiu_swo.h>
#include <rtt/rtt.h>

#include <server/server.h>
#include <server/gdb_server.h>
#include <server/rtt_server.h>

#ifdef HAVE_STRINGS_H
#include <strings.h>
#endif

//版本信息定义
#ifdef PKGBLDDATE
#define OPENOCD_VERSION	\
	"Open On-Chip Debugger " VERSION RELSTR " (" PKGBLDDATE ")"
#else
#define OPENOCD_VERSION	\
	"Open On-Chip Debugger " VERSION RELSTR
#endif

/* 启动TCL脚本定义
 * 将外部文件startup_tcl.inc包含到一个const char数组中，
 * 表示为OpenOCD的启动TCL脚本内容 */
static const char openocd_startup_tcl[] = {
#include "startup_tcl.inc"
0 /* Terminate with zero */
};

// handler和jim的区别：
// 	用途：handler类用于OpenOCD内部命令处理器，Jim类用于OpenOCD的Jim TCL 脚本命令处理器
//      接口类型：前者是COMMAND_HANDLE宏定义的命令处理器，后者是Jim TCL 命令接口函数
//      适用环境：handler类用于OpenOCD的CLI命令系统，通过TELNET或控制台访问；Jim类用于OpenOCD的TCL脚本解释器环境
//
//jim_version_command 是一个命令处理函数，返回 OpenOCD 的版本信息
//接收三个参数：interp 是解释器对象，argc 是参数个数，argv 是参数数组
/* Give scripts and TELNET a way to find out what version this is */
static int jim_version_command(Jim_Interp *interp, int argc,
	Jim_Obj * const *argv)
{       
	// 参数检查：如果传入的参数数量大于2，则返回错误 JIM_ERR
	if (argc > 2)
		return JIM_ERR;
	// 默认版本信息：OPENOCD_VERSION
	const char *str = "";
	char *version_str;
	version_str = OPENOCD_VERSION;
        
	//如果传入了一个参数，则提取该参数
        //如果参数为 "git"，则将 version_str 设置为 GITVERSION，即 Git 版本信息
	if (argc == 2)
		str = Jim_GetString(argv[1], NULL);

	if (strcmp("git", str) == 0)
		version_str = GITVERSION;
        
	//使用 Jim_SetResult 设置结果，将 version_str 作为输出返回给解释器
	Jim_SetResult(interp, Jim_NewStringObj(interp, version_str, -1));

	return JIM_OK;
}

/* 事件回调处理函数，用于处理与目标状态相关的事件（如GDB连接、断开和目标暂停）
 * 它通过设置或检查target->verbose_halt_msg来决定是否显示详细的暂停信息
 * 函数声明：
 * 这个函数被声明为静态（static），意味着它仅在当前文件范围内可见。它接收三个参数：
 * target: 指向目标（target）结构体的指针，代表当前正在调试的目标。
 * event: 一个枚举类型的值，表示发生的事件类型。
 * priv: 一个通用的指针，可以传递附加的上下文信息。 */
static int log_target_callback_event_handler(struct target *target,
	enum target_event event,
	void *priv)
{
	switch (event) {
		/* 当GDB连接到目标并启动时，将target->verbose_halt_msg设置为false，表示不显示详细的暂停信息 */
		case TARGET_EVENT_GDB_START:
			target->verbose_halt_msg = false;
			break;
                /* 当GDB断开连接时，将target->verbose_halt_msg设置为true，表示允许显示详细的暂停信息 */
		case TARGET_EVENT_GDB_END:
			target->verbose_halt_msg = true;
			break;
		/* 当目标被暂停时，检查target->verbose_halt_msg的值。
		* 如果为true，则调用target_arch_state(target)，以便获取和显示目标架构的状态信息
		* 注释提到“当调试器导致暂停时，不显示信息”。 */
		case TARGET_EVENT_HALTED:
			if (target->verbose_halt_msg) {
				/* do not display information when debugger caused the halt */
				target_arch_state(target);
			}
			break;
		default:
			break;
	}

	return ERROR_OK;//返回ERROR_OK，表示处理成功
}

//初始化标志位：该布尔标志用于控制程序是否在启动时初始化
static bool init_at_startup = true;

/*禁用启动时的初始化，将init_at_startup设置为false*/
COMMAND_HANDLER(handle_noinit_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;
	init_at_startup = false;
	return ERROR_OK;
}

/*
 *命令处理函数，用于执行初始化
*/
/* OpenOCD can't really handle failure of this command. Patches welcome! :-) */
COMMAND_HANDLER(handle_init_command)
{
        //检查参数数量
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int retval;
	//静态变量，用于记录是否已完成初始化。若已初始化，则直接返回成功
	static int initialized;
	if (initialized)
		return ERROR_OK;

	initialized = 1;
        
	//保存当前的jtag_poll_mask状态，以备后续恢复
	bool save_poll_mask = jtag_poll_mask();
       
	//运行“target init”命令来初始化目标。如果失败，返回ERROR_FAIL
	retval = command_run_line(CMD_CTX, "target init");
	if (retval != ERROR_OK)
		return ERROR_FAIL;
        
	//调用adapter_init来初始化调试适配器，并记录调试日志信息
	retval = adapter_init(CMD_CTX);
	if (retval != ERROR_OK) {
		/* we must be able to set up the debug adapter */
		return retval;
	}

	LOG_DEBUG("Debug Adapter init complete");

	/* "transport init" verifies the expected devices are present;
	 * for JTAG, it checks the list of configured TAPs against
	 * what's discoverable, possibly with help from the platform's
	 * JTAG event handlers.  (which require COMMAND_EXEC)
          * 切换命令上下文模式为COMMAND_EXEC，然后运行“transport init”命令，确保传输层初始化成功
	 */
	command_context_mode(CMD_CTX, COMMAND_EXEC);

	retval = command_run_line(CMD_CTX, "transport init");
	if (retval != ERROR_OK)
		return ERROR_FAIL;
        
	/* 初始化DAP（调试访问端口），检查所有连接的目标状态
         * 如果target_examine检查失败，返回错误 */
	retval = command_run_line(CMD_CTX, "dap init");
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	LOG_DEBUG("Examining targets...");
	if (target_examine() != ERROR_OK)
		LOG_DEBUG("target examination failed");
        
	/* 切换命令模式为COMMAND_CONFIG并依次初始化flash、nand、pld等存储器组件，
         * 确保每个组件正确配置 */
	command_context_mode(CMD_CTX, COMMAND_CONFIG);

	if (command_run_line(CMD_CTX, "flash init") != ERROR_OK)
		return ERROR_FAIL;

	if (command_run_line(CMD_CTX, "nand init") != ERROR_OK)
		return ERROR_FAIL;

	if (command_run_line(CMD_CTX, "pld init") != ERROR_OK)
		return ERROR_FAIL;
	command_context_mode(CMD_CTX, COMMAND_EXEC);

	/* in COMMAND_EXEC, after target_examine(), only tpiu or only swo 
         * 切换回COMMAND_EXEC模式，运行tpiu init命令
         * 恢复之前的jtag_poll_mask状态，确保不影响后续操作 */
	if (command_run_line(CMD_CTX, "tpiu init") != ERROR_OK)
		return ERROR_FAIL;

	jtag_poll_unmask(save_poll_mask);

	/* initialize telnet subsystem 
         * 初始化Telnet系统：将所有目标（all_targets）添加到GDB调试会话 */
	gdb_target_add_all(all_targets);
        
	/* 注册log_target_callback_event_handler事件回调函数
         * 每当目标发生特定事件（如暂停、继续运行等）时，OpenOCD会调用这个回调函数
	 * 这在调试过程中有助于记录和管理调试事件
         */
	target_register_event_callback(log_target_callback_event_handler, CMD_CTX);
       
	/* 调用了command_run_line函数，执行名为_run_post_init_commands的命令
         * 该命令可能包含一些自定义的后置初始化指令，用于在初始化完成后进一步配置或执行一些额外操作
         * 如果命令执行失败，返回ERROR_FAIL */
	if (command_run_line(CMD_CTX, "_run_post_init_commands") != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

/* 命令处理器，用于处理add_script_search_dir命令
 * 将一个用户指定的目录添加到 OpenOCD 的脚本搜索路径中，
 * 以便在执行脚本时可以在该目录中查找文件 */
COMMAND_HANDLER(handle_add_script_search_dir_command)
{       
	//检查参数数量，确保只有一个参数
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	//add_script_search_dir将 CMD_ARGV[0] 指定的路径添加到脚本搜索路径中
	add_script_search_dir(CMD_ARGV[0]);

	//返回成功状态 ERROR_OK，表示操作完成
	return ERROR_OK;
}

/* 定义名为 openocd_command_handlers 的静态常量结构体数组，类型为 command_registration
 * 列出了 OpenOCD 的一些命令及其对应的处理函数
 * 每个数组元素代表一个命令，包含名称(命令的名称字符串)、处理函数(用于处理命令的函数指针)、
 * 模式(表示该命令在哪种模式下有效，如COMMAND_ANY表示在任意模式下有效)、
 * 帮助信息(描述命令的作用)和用法(描述命令的参数要求) */
static const struct command_registration openocd_command_handlers[] = {
	 /* version */
        {
		.name = "version",  //表示显示版本信息的命令
		.jim_handler = jim_version_command, //用于处理 version 命令
		.mode = COMMAND_ANY, //表示该命令可以在任何模式下执行
		.help = "show program version", //说明该命令用于显示程序版本
	},
         /* noinit */
	{
		.name = "noinit", //用于在启动时阻止调用 init
		.handler = &handle_noinit_command, 
		.mode = COMMAND_CONFIG, //表示该命令仅在配置模式下有效
		.help = "Prevent 'init' from being called at startup.",
		.usage = "" //表示没有参数
	},
        /* init */
	{
		.name = "init", //用于初始化已配置的目标和服务器
		.handler = &handle_init_command,
		.mode = COMMAND_ANY,
		.help = "Initializes configured targets and servers.  "
			"Changes command mode from CONFIG to EXEC.  "
			"Unless 'noinit' is called, this command is "
			"called automatically at the end of startup.",
                        //描述初始化过程的详细说明，解释在启动时该命令会自动调用，除非调用了 noinit
		.usage = ""
	},
        /* add_script_search_dir */
	{
		.name = "add_script_search_dir", //用于添加一个目录到脚本搜索路径中
		.handler = &handle_add_script_search_dir_command,
		.mode = COMMAND_ANY,
		.help = "dir to search for config files and scripts",
		.usage = "<directory>" //要求提供一个参数，指定要添加的目录
	},
	COMMAND_REGISTRATION_DONE //结束标志，标识数组的结束，通常用于数组终止符
};

/* openocd_register_commands函数：用于在 OpenOCD 的命令系统中注册命令
 * 这是一个静态函数 openocd_register_commands，它在当前文件范围内可见，不能在其他文件中访问
 * 参数 cmd_ctx 是一个指向 command_context 结构的指针，用于表示命令上下文 */
static int openocd_register_commands(struct command_context *cmd_ctx)
{       
	/* 调用register_commands函数将 openocd_command_handlers中定义的命令注册到指定的命令上下文cmd_ctx中
         * NULL 作为第二个参数传递，表示没有特定的父命令名称，即这些命令是顶层命令
         * register_commands的返回值将作为openocd_register_commands的返回值，
	 * 这样可以用于判断命令注册是否成功 */
	return register_commands(cmd_ctx, NULL, openocd_command_handlers);
}

/*
 * TODO: to be removed after v0.12.0
 * workaround for syntax change of "expr" in jimtcl 0.81
 * replace "expr" with openocd version that prints the deprecated msg
 */
struct jim_scriptobj {
	void *token;
	Jim_Obj *filename_obj;
	int len;
	int subst_flags;
	int in_use;
	int firstline;
	int linenr;
	int missing;
};

static int jim_expr_command(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	if (argc == 2)
		return Jim_EvalExpression(interp, argv[1]);

	if (argc > 2) {
		Jim_Obj *obj = Jim_ConcatObj(interp, argc - 1, argv + 1);
		Jim_IncrRefCount(obj);
		const char *s = Jim_String(obj);
		struct jim_scriptobj *script = Jim_GetIntRepPtr(interp->currentScriptObj);
		if (interp->currentScriptObj == interp->emptyObj ||
				strcmp(interp->currentScriptObj->typePtr->name, "script") ||
				script->subst_flags ||
				script->filename_obj == interp->emptyObj)
			LOG_WARNING("DEPRECATED! use 'expr { %s }' not 'expr %s'", s, s);
		else
			LOG_WARNING("DEPRECATED! (%s:%d) use 'expr { %s }' not 'expr %s'",
						Jim_String(script->filename_obj), script->linenr, s, s);
		int retcode = Jim_EvalExpression(interp, obj);
		Jim_DecrRefCount(interp, obj);
		return retcode;
	}

	Jim_WrongNumArgs(interp, 1, argv, "expression ?...?");
	return JIM_ERR;
}

static const struct command_registration expr_handler[] = {
	{
		.name = "expr",
		.jim_handler = jim_expr_command,
		.mode = COMMAND_ANY,
		.help = "",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static int workaround_for_jimtcl_expr(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, expr_handler);
}

/* 声明了一个全局指针变量 global_cmd_ctx，类型为 struct command_context *，用于指向全局的命令上下文
 * 用于存储和访问全局的命令上下文 */
struct command_context *global_cmd_ctx;

/* setup_command_handler 函数的主要作用是初始化命令上下文 cmd_ctx，
 * 然后逐个注册子系统命令，
 * 包括 OpenOCD 核心、GDB、日志、传输、适配器、目标、闪存等模块
 * 通过这些命令注册，OpenOCD 实现了对各种功能和子系统的管理和控制 
 * 函数 setup_command_handler 是一个静态函数，它在当前文件范围内可见
 * 接收一个 Jim_Interp 类型的参数 interp，这是 OpenOCD 中的脚本解释器，用于解释和执行 TCL 脚本 */
static struct command_context *setup_command_handler(Jim_Interp *interp)
{
	/* 1、初始化日志系统
         * 首先调用 log_init() 初始化日志系统，然后输出调试信息，表示日志初始化完成 */
	log_init();
	LOG_DEBUG("log_init: complete");
	
	/* 2、初始化命令上下文
         * 调用 command_init 函数初始化命令上下文 cmd_ctx，
	 * 并传入启动 TCL 脚本 openocd_startup_tcl 和解释器 interp
         * cmd_ctx 是 struct command_context 类型的指针，用于存储命令的上下文信息，
	 * 初始化后可以用来注册和处理命令*/
	struct command_context *cmd_ctx = command_init(openocd_startup_tcl, interp);

	
	/* register subsystem commands */
	/* 3、依次调用每个子系统的命令注册函数，注册命令
         * 定义一个函数指针类型command_registrant_t，用于指向命令注册函数，接收struct command_context *参数
         * command_registrants是一个常量数组，包含多个命令注册函数的地址。
	 * 这些函数用于注册不同的子系统命令，如 OpenOCD 核心命令、服务器命令、GDB 命令、日志命令、
         * 传输层命令、适配器命令、目标命令、闪存命令等。
	 * 数组最后一个元素为 NULL，用于标记数组的结束 */
	typedef int (*command_registrant_t)(struct command_context *cmd_ctx_value);
	static const command_registrant_t command_registrants[] = {
		&workaround_for_jimtcl_expr,
		&openocd_register_commands,
		&server_register_commands,
		&gdb_register_commands,
		&log_register_commands,
		&rtt_server_register_commands,
		&transport_register_commands,
		&adapter_register_commands,
		&target_register_commands,
		&flash_register_commands,
		&nand_register_commands,
		&pld_register_commands,
		&cti_register_commands,
		&dap_register_commands,
		&arm_tpiu_swo_register_commands,
		NULL
	};
	/* 通过 for 循环遍历 command_registrants 数组，逐一调用每个注册函数，并将 cmd_ctx 作为参数传递
         * 如果某个注册函数返回的值不等于 ERROR_OK，则表示注册失败，
         * 此时调用 command_done 清理命令上下文 cmd_ctx，并返回 NULL 以终止初始化过程 */
	for (unsigned i = 0; command_registrants[i]; i++) {
		int retval = (*command_registrants[i])(cmd_ctx);
		if (retval != ERROR_OK) {
			command_done(cmd_ctx);
			return NULL;
		}
	}
	/* 4、注册成功后，输出调试信息和版本信息
         *输出调试信息，表示命令注册已完成
         * 打印 OpenOCD 的版本信息和 GPL v2 许可证声明，提示用户当前的程序版本和使用协议 */
	LOG_DEBUG("command registration: complete");

	LOG_OUTPUT(OPENOCD_VERSION "\n"
		"Licensed under GNU GPL v2\n");
        
	/* 5、将命令上下文赋值给全局变量 global_cmd_ctx，并返回上下文指针 cmd_ctx
         * 将 cmd_ctx 指向的命令上下文赋值给全局变量 global_cmd_ctx，以便其他地方可以访问全局的命令上下文
         */
	global_cmd_ctx = cmd_ctx;
       
	//返回初始化完成的 cmd_ctx，表示命令处理器设置完成
	return cmd_ctx;
}

/** OpenOCD runtime meat that can become single-thread in future. It parse
 * commandline, reads configuration, sets up the target and starts server loop.
 * Commandline arguments are passed into this function from openocd_main().
 * 定义一个函数 openocd_thread，它是 OpenOCD 的核心运行函数。
 * 它解析命令行参数，读取配置，设置目标，初始化目标系统和启动服务器循环
 * 这个函数是 openocd_main 函数的一个辅助函数，接收命令行参数并执行主要的初始化和运行步骤 
 * 函数 openocd_thread 是一个静态函数，仅在当前文件范围内可见
 * 参数：argc 和 argv 是从命令行接收的参数数量和参数值
 * cmd_ctx 是指向 command_context 结构的指针，用于存储和管理命令的上下文信息 
 */
static int openocd_thread(int argc, char *argv[], struct command_context *cmd_ctx)
{
	int ret;
	
        /* 1、解析命令行参数
 	 *调用 parse_cmdline_args 函数来解析命令行参数，这个函数将参数解析结果存储在 cmd_ctx 中
         * 如果解析失败，函数立即返回 ERROR_FAIL */
	if (parse_cmdline_args(cmd_ctx, argc, argv) != ERROR_OK)
		return ERROR_FAIL;
        
	/* 2、执行服务器预初始化
 	 * 调用 server_preinit 进行服务器的预初始化。预初始化阶段通常用于设置一些必要的环境或检查。
         * 如果预初始化失败，则返回 ERROR_FAIL */
	if (server_preinit() != ERROR_OK)
		return ERROR_FAIL;

	/* 3、解析配置文件，初始化服务器
 	 * 调用 parse_config_file 函数来解析配置文件，并将结果存储在 ret 中
         * 如果返回值为 ERROR_COMMAND_CLOSE_CONNECTION，则调用 server_quit 退出服务器，
         * 并返回 ERROR_OK，表示成功退出
         * 如果解析失败（即 ret 不等于 ERROR_OK），也会调用 server_quit 并返回 ERROR_FAIL */
	ret = parse_config_file(cmd_ctx);
	if (ret == ERROR_COMMAND_CLOSE_CONNECTION) {
		server_quit(); /* gdb server may be initialized by -c init */
		return ERROR_OK;
	} else if (ret != ERROR_OK) {
		server_quit(); /* gdb server may be initialized by -c init */
		return ERROR_FAIL;
	}

	/* 4、根据设置，在启动时执行初始化
 	 * 调用 server_init 初始化服务器，将命令上下文 cmd_ctx 传递给服务器
         * 如果初始化失败，返回 ERROR_FAIL */
	ret = server_init(cmd_ctx);
	if (ret != ERROR_OK)
		return ERROR_FAIL;

	/* 检查 init_at_startup 是否为 true，如果是，则运行 init 命令(即 command_run_line(cmd_ctx, "init"))，
	 * 用于执行启动时的初始化。如果 init 命令执行失败，则调用 server_quit 并返回 ERROR_FAIL */
	if (init_at_startup) {
		ret = command_run_line(cmd_ctx, "init");
		if (ret != ERROR_OK) {
			server_quit();
			return ERROR_FAIL;
		}
	}

	/* 5、进入服务器循环，处理调试客户端的请求
 	 * 调用 server_loop，进入服务器的主循环，等待和处理调试客户端的连接和请求
         * 服务器循环会持续运行，直到接收到终止信号或出现错误 */
	ret = server_loop(cmd_ctx);

	/* 6、退出服务器循环，关闭服务器并返回退出状态
 	 * 退出服务器主循环后，调用 server_quit 来停止服务器，返回最后的退出信号
         * 如果 server_quit 的返回值不是 ERROR_OK，则返回 last_signal，表示退出状态 */
	int last_signal = server_quit();
	if (last_signal != ERROR_OK)
		return last_signal;

	if (ret != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

/* normally this is the main() function entry, but if OpenOCD is linked
 * into application, then this fn will not be invoked, but rather that
 * application will have it's own implementation of main(). 
 * openocd_main 函数:负责初始化 OpenOCD 的各个子系统，运行主线程并在退出时进行清理
 * 这是 OpenOCD 的主要入口函数
 * 如果 OpenOCD 被作为库链接到另一个应用程序中，那么该应用程序会实现自己的 main 函数，而不是调用 openocd_main */
int openocd_main(int argc, char *argv[])
{
	int ret;

	/* initialize commandline interface */
	//1、初始化命令行接口、实用工具和 RTT 子系统
	/* initialize commandline interface 
         * 定义并初始化 cmd_ctx（命令上下文）用于管理命令系统。
         * 调用 setup_command_handler 初始化命令接口和命令处理程序 */
	struct command_context *cmd_ctx;

	cmd_ctx = setup_command_handler(NULL);	

	/* 调用 util_init 初始化实用工具和 rtt_init 初始化 RTT（实时跟踪）子系统。
         * 如果任一初始化失败，返回 EXIT_FAILURE */
	if (util_init(cmd_ctx) != ERROR_OK)
		return EXIT_FAILURE;

	if (rtt_init() != ERROR_OK)
		return EXIT_FAILURE;

	//输出日志信息，显示如何提交错误报告的链接
	LOG_OUTPUT("For bug reports, read\n\t"
		"http://openocd.org/doc/doxygen/bugs.html"
		"\n");

	//2、设置命令上下文模式并启动服务器所需的主机 OS 设置
        /* 设置命令上下文模式为 COMMAND_CONFIG（配置模式），以便进行配置
         * 设置命令输出处理程序，将输出处理委托给 configuration_output_handler 函数 */
	command_context_mode(cmd_ctx, COMMAND_CONFIG);
	command_set_output_handler(cmd_ctx, configuration_output_handler, NULL);

	//调用 server_host_os_entry，用于在主机操作系统上初始化服务器所需的设置
	server_host_os_entry();

	/* Start the executable meat that can evolve into thread in future. */
	//3、调用 openocd_thread 启动核心执行逻辑，包括解析命令行、读取配置、初始化目标和进入服务器循环
	/* Start the executable meat that can evolve into thread in future. 
         * 调用 openocd_thread 函数，启动 OpenOCD 的主要执行过程，
	 * 包括命令行解析、配置读取、目标设置和服务器循环，该函数的返回值存储在 ret 中 */
	ret = openocd_thread(argc, argv, cmd_ctx);

	//4、清理各个子系统（如闪存、GDB 服务、TPIU/SWO、服务器、命令系统、DAP、适配器等）
        /* 清理资源，依次释放：
         *       flash_free_all_banks() 释放所有闪存资源。
         *       gdb_service_free() 释放 GDB 服务资源。
         *       arm_tpiu_swo_cleanup_all() 清理 ARM 的 TPIU/SWO 资源。
         *       server_free() 释放服务器资源 */
	flash_free_all_banks();
	gdb_service_free();
	arm_tpiu_swo_cleanup_all();
	server_free();

	//注销所有注册的命令，并删除帮助命令，释放命令系统的相关资源
	unregister_all_commands(cmd_ctx, NULL);
	help_del_all_commands(cmd_ctx);

	/* free all DAP and CTI objects 
         * 清理所有 DAP（调试访问端口）和 CTI（CoreSight 跟踪接口）相关对象，确保调试相关资源完全释放 */
	arm_cti_cleanup_all();
	dap_cleanup_all();

	/* 调用 adapter_quit 退出适配器，释放调试适配器的资源。
         * server_host_os_close 关闭主机操作系统相关的服务器设置，进行必要的清理操作 */
	adapter_quit();
	server_host_os_close();

	/* Shutdown commandline interface */
	//关闭命令行接口，释放命令上下文 cmd_ctx 及相关资源
	command_exit(cmd_ctx);

	//调用 rtt_exit 清理 RTT 子系统，free_config 释放配置资源，确保所有加载的配置都被释放
	rtt_exit();
	free_config();

	//退出日志系统，清理日志资源
	log_exit();

	//5、根据 openocd_thread 的返回状态决定退出状态
	//检查 openocd_thread 的返回值
	if (ret == ERROR_FAIL)
		return EXIT_FAILURE;
	else if (ret != ERROR_OK)
		exit_on_signal(ret);

	return ret;
}
