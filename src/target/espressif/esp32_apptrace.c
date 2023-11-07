// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32xx application tracing module for OpenOCD                        *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_ARPA_INET_H
#include <arpa/inet.h>
#endif

#ifdef HAVE_NETDB_H
#include <netdb.h>
#endif

#ifndef _WIN32
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#endif

#include <helper/list.h>
#include <helper/time_support.h>
#include <target/target.h>
#include <target/target_type.h>
#include <target/smp.h>
#include <server/server.h>
#include "esp_xtensa.h"
#include "esp_xtensa_smp.h"
#include "esp_xtensa_apptrace.h"
#include "esp32_apptrace.h"
#include "esp32_sysview.h"
#include "segger_sysview.h"

#define ESP32_APPTRACE_USER_BLOCK_CORE(_v_)     ((_v_) >> 15)
#define ESP32_APPTRACE_USER_BLOCK_LEN(_v_)      ((_v_) & ~BIT(15))

#define ESP32_APPTRACE_USER_BLOCK_HDR_SZ        4

#define ESP_APPTRACE_CMD_MODE_GEN               0
#define ESP_APPTRACE_CMD_MODE_SYSVIEW           1
#define ESP_APPTRACE_CMD_MODE_SYSVIEW_MCORE     2
#define ESP_APPTRACE_CMD_MODE_SYNC              3

#define ESP32_APPTRACE_TGT_STATE_TMO            5000
#define ESP_APPTRACE_BLOCKS_POOL_SZ             10

struct esp32_apptrace_dest_file_data {
	int fout;
};

struct esp32_apptrace_dest_tcp_data {
	int sockfd;
};

struct esp32_apptrace_target_state {
	int running;
	uint32_t block_id;
	uint32_t data_len;
};

struct esp_apptrace_target2host_hdr {
	uint16_t block_sz;
	uint16_t wr_sz;
};
#define APPTRACE_BLOCK_SIZE_OFFSET      0
#define APPTRACE_WR_SIZE_OFFSET         2

struct esp32_apptrace_block {
	struct list_head node;
	uint8_t *data;
	uint32_t data_len;
};

static int esp32_apptrace_data_processor(void *priv);
static int esp32_apptrace_get_data_info(struct esp32_apptrace_cmd_ctx *ctx,
	struct esp32_apptrace_target_state *target_state,
	uint32_t *fired_target_num);
static int esp32_apptrace_safe_halt_targets(struct esp32_apptrace_cmd_ctx *ctx,
	struct esp32_apptrace_target_state *targets);
static struct esp32_apptrace_block *esp32_apptrace_free_block_get(struct esp32_apptrace_cmd_ctx *ctx);
static int esp32_apptrace_handle_trace_block(struct esp32_apptrace_cmd_ctx *ctx,
	struct esp32_apptrace_block *block);
static int esp32_sysview_start(struct esp32_apptrace_cmd_ctx *ctx);
static int esp32_sysview_stop(struct esp32_apptrace_cmd_ctx *ctx);

static const bool s_time_stats_enable = true;

/*********************************************************************
*                       Trace destination API
**********************************************************************/

static int esp32_apptrace_file_dest_write(void *priv, uint8_t *data, int size)
{
	struct esp32_apptrace_dest_file_data *dest_data = (struct esp32_apptrace_dest_file_data *)priv;

	int wr_sz = write(dest_data->fout, data, size);
	if (wr_sz != size) {
		LOG_ERROR("Failed to write %d bytes to out file (%d)! Written %d.", size, errno, wr_sz);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int esp32_apptrace_file_dest_cleanup(void *priv)
{
	struct esp32_apptrace_dest_file_data *dest_data = (struct esp32_apptrace_dest_file_data *)priv;

	if (dest_data->fout > 0)
		close(dest_data->fout);
	free(dest_data);
	return ERROR_OK;
}

static int esp32_apptrace_file_dest_init(struct esp32_apptrace_dest *dest, const char *dest_name)
{
	struct esp32_apptrace_dest_file_data *dest_data = calloc(1, sizeof(*dest_data));
	if (!dest_data) {
		LOG_ERROR("Failed to alloc mem for file dest!");
		return ERROR_FAIL;
	}

	LOG_INFO("Open file %s", dest_name);
	dest_data->fout = open(dest_name, O_WRONLY | O_CREAT | O_TRUNC | O_BINARY, 0666);
	if (dest_data->fout <= 0) {
		LOG_ERROR("Failed to open file %s", dest_name);
		free(dest_data);
		return ERROR_FAIL;
	}

	dest->priv = dest_data;
	dest->write = esp32_apptrace_file_dest_write;
	dest->clean = esp32_apptrace_file_dest_cleanup;
	dest->log_progress = true;

	return ERROR_OK;
}

static int esp32_apptrace_console_dest_write(void *priv, uint8_t *data, int size)
{
	LOG_USER_N("%.*s", size, data);
	return ERROR_OK;
}

static int esp32_apptrace_console_dest_cleanup(void *priv)
{
	return ERROR_OK;
}

static int esp32_apptrace_console_dest_init(struct esp32_apptrace_dest *dest, const char *dest_name)
{
	dest->priv = NULL;
	dest->write = esp32_apptrace_console_dest_write;
	dest->clean = esp32_apptrace_console_dest_cleanup;
	dest->log_progress = false;

	return ERROR_OK;
}

static int esp32_apptrace_tcp_dest_write(void *priv, uint8_t *data, int size)
{
	struct esp32_apptrace_dest_tcp_data *dest_data = (struct esp32_apptrace_dest_tcp_data *)priv;
	int wr_sz = write_socket(dest_data->sockfd, data, size);
	if (wr_sz != size) {
		LOG_ERROR("Failed to write %u bytes to out socket (%d)! Written %d.", size, errno, wr_sz);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int esp32_apptrace_tcp_dest_cleanup(void *priv)
{
	struct esp32_apptrace_dest_tcp_data *dest_data = (struct esp32_apptrace_dest_tcp_data *)priv;

	if (dest_data->sockfd > 0)
		close_socket(dest_data->sockfd);
	free(dest_data);
	return ERROR_OK;
}

static int esp32_apptrace_tcp_dest_init(struct esp32_apptrace_dest *dest, const char *dest_name)
{
	const char *port_sep = strchr(dest_name, ':');
	/* separator not found, or was the first or the last character */
	if (!port_sep || port_sep == dest_name || port_sep == dest_name + strlen(dest_name) - 1) {
		LOG_ERROR("apptrace: Invalid connection URI, format should be tcp://host:port");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	size_t hostname_len = port_sep - dest_name;

	char hostname[64] = { 0 };
	if (hostname_len >= sizeof(hostname)) {
		LOG_ERROR("apptrace: Hostname too long");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	memcpy(hostname, dest_name, hostname_len);

	const char *port_str = port_sep + 1;
	struct addrinfo *ai;
	int flags = 0;
#ifdef AI_NUMERICSERV
	flags |= AI_NUMERICSERV;
#endif	/* AI_NUMERICSERV */
	struct addrinfo hint = {
		.ai_family = AF_UNSPEC,
		.ai_socktype = SOCK_STREAM,
		.ai_protocol = 0,
		.ai_flags = flags
	};
	int res = getaddrinfo(hostname, port_str, &hint, &ai);
	if (res != 0) {
		LOG_ERROR("apptrace: Failed to resolve host name: %s", hostname);
		return ERROR_FAIL;
	}
	int sockfd = -1;
	for (struct addrinfo *ai_it = ai; ai_it; ai_it = ai_it->ai_next) {
		sockfd = socket(ai_it->ai_family, ai_it->ai_socktype, ai_it->ai_protocol);
		if (sockfd < 0) {
			LOG_DEBUG("apptrace: Failed to create socket (%d, %d, %d) (%s)",
				ai_it->ai_family,
				ai_it->ai_socktype,
				ai_it->ai_protocol,
				strerror(errno));
			continue;
		}

		char cur_hostname[NI_MAXHOST];
		char cur_portname[NI_MAXSERV];
		res =
			getnameinfo(ai_it->ai_addr, ai_it->ai_addrlen, cur_hostname,
			sizeof(cur_hostname),
			cur_portname, sizeof(cur_portname),
			NI_NUMERICHOST | NI_NUMERICSERV);
		if (res != 0)
			continue;

		LOG_INFO("apptrace: Trying to connect to %s:%s", cur_hostname, cur_portname);
		if (connect(sockfd, ai_it->ai_addr, ai_it->ai_addrlen) < 0) {
			close_socket(sockfd);
			sockfd = -1;
			LOG_WARNING("apptrace: Connection failed (%s)", strerror(errno));
			continue;
		}
		break;
	}
	freeaddrinfo(ai);
	if (sockfd < 0) {
		LOG_ERROR("apptrace: Could not connect to %s:%s", hostname, port_str);
		return ERROR_FAIL;
	}
	LOG_INFO("apptrace: Connected!");

	struct esp32_apptrace_dest_tcp_data *dest_data = calloc(1, sizeof(struct esp32_apptrace_dest_tcp_data));
	if (!dest_data) {
		LOG_ERROR("apptrace: Failed to alloc mem for tcp dest!");
		close_socket(sockfd);
		return ERROR_FAIL;
	}

	dest_data->sockfd = sockfd;
	dest->priv = dest_data;
	dest->write = esp32_apptrace_tcp_dest_write;
	dest->clean = esp32_apptrace_tcp_dest_cleanup;
	dest->log_progress = true;

	return ERROR_OK;
}

int esp32_apptrace_dest_init(struct esp32_apptrace_dest dest[], const char *dest_paths[], unsigned int max_dests)
{
	int res;
	unsigned int i;

	for (i = 0; i < max_dests; i++) {
		if (strncmp(dest_paths[i], "file://", 7) == 0)
			res = esp32_apptrace_file_dest_init(&dest[i], &dest_paths[i][7]);
		else if (strncmp(dest_paths[i], "con:", 4) == 0)
			res = esp32_apptrace_console_dest_init(&dest[i], NULL);
		else if (strncmp(dest_paths[i], "tcp://", 6) == 0)
			res = esp32_apptrace_tcp_dest_init(&dest[i], &dest_paths[i][6]);
		else
			break;

		if (res != ERROR_OK) {
			LOG_ERROR("apptrace: Failed to init trace data destination '%s'!", dest_paths[i]);
			return 0;
		}
	}

	return i;
}

int esp32_apptrace_dest_cleanup(struct esp32_apptrace_dest dest[], unsigned int max_dests)
{
	for (unsigned int i = 0; i < max_dests; i++) {
		if (dest[i].clean && dest[i].priv) {
			int res = dest[i].clean(dest[i].priv);
			dest[i].priv = NULL;
			return res;
		}
	}
	return ERROR_OK;
}

/*********************************************************************
*                 Trace data blocks management API
**********************************************************************/
static void esp32_apptrace_blocks_pool_cleanup(struct esp32_apptrace_cmd_ctx *ctx)
{
	struct esp32_apptrace_block *cur;
	struct list_head *head = &ctx->free_trace_blocks;
	struct list_head *tmp, *pos;

	list_for_each_safe(pos, tmp, head) {
		cur = list_entry(pos, struct esp32_apptrace_block, node);
		if (cur) {
			list_del(&cur->node);
			free(cur->data);
			free(cur);
		}
	}

	head = &ctx->ready_trace_blocks;

	list_for_each_safe(pos, tmp, head) {
		cur = list_entry(pos, struct esp32_apptrace_block, node);
		if (cur) {
			list_del(&cur->node);
			free(cur->data);
			free(cur);
		}
	}
}

struct esp32_apptrace_block *esp32_apptrace_free_block_get(struct esp32_apptrace_cmd_ctx *ctx)
{
	struct esp32_apptrace_block *block = NULL;

	if (!list_empty(&ctx->free_trace_blocks)) {
		/*get first */
		block = list_first_entry(&ctx->free_trace_blocks, struct esp32_apptrace_block, node);
		list_del(&block->node);
	}

	return block;
}

static int esp32_apptrace_ready_block_put(struct esp32_apptrace_cmd_ctx *ctx, struct esp32_apptrace_block *block)
{
	LOG_DEBUG("esp32_apptrace_ready_block_put");
	/* add to ready blocks list */
	INIT_LIST_HEAD(&block->node);
	list_add(&block->node, &ctx->ready_trace_blocks);

	return ERROR_OK;
}

static struct esp32_apptrace_block *esp32_apptrace_ready_block_get(struct esp32_apptrace_cmd_ctx *ctx)
{
	if (list_empty(&ctx->ready_trace_blocks))
		return NULL;

	struct esp32_apptrace_block *block =
		list_last_entry(&ctx->ready_trace_blocks, struct esp32_apptrace_block, node);

	/* remove it from ready list */
	list_del(&block->node);

	return block;
}

static int esp32_apptrace_block_free(struct esp32_apptrace_cmd_ctx *ctx, struct esp32_apptrace_block *block)
{
	/* add to free blocks list */
	INIT_LIST_HEAD(&block->node);
	list_add(&block->node, &ctx->free_trace_blocks);

	return ERROR_OK;
}

static int esp32_apptrace_wait_tracing_finished(struct esp32_apptrace_cmd_ctx *ctx)
{
	int64_t timeout = timeval_ms() + (LOG_LEVEL_IS(LOG_LVL_DEBUG) ? 70000 : 5000);
	while (!list_empty(&ctx->ready_trace_blocks)) {
		alive_sleep(100);
		if (timeval_ms() >= timeout) {
			LOG_ERROR("Failed to wait for pended trace blocks!");
			return ERROR_FAIL;
		}
	}
	/* signal timer callback to stop */
	ctx->running = 0;
	target_unregister_timer_callback(esp32_apptrace_data_processor, ctx);
	return ERROR_OK;
}

/*********************************************************************
*                          Trace commands
**********************************************************************/

int esp32_apptrace_cmd_ctx_init(struct esp32_apptrace_cmd_ctx *cmd_ctx, struct command_invocation *cmd, int mode)
{
	struct target *target = get_current_target(CMD_CTX);

	memset(cmd_ctx, 0, sizeof(struct esp32_apptrace_cmd_ctx));
	cmd_ctx->target = target;
	cmd_ctx->mode = mode;
	cmd_ctx->target_state = target->state;
	cmd_ctx->cmd = cmd;

	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		unsigned int i = 0;
		cmd_ctx->cores_num = 0;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			if (i == ESP32_APPTRACE_MAX_CORES_NUM) {
				command_print(cmd, "Too many cores configured! Max %d cores are supported.",
					ESP32_APPTRACE_MAX_CORES_NUM);
				return ERROR_FAIL;
			}
			if (!target_was_examined(curr))
				continue;
			cmd_ctx->cores_num++;
			cmd_ctx->cpus[i++] = curr;
		}
	} else {
		cmd_ctx->cores_num = 1;
		cmd_ctx->cpus[0] = target;
	}
	/* some relies on ESP32_APPTRACE_MAX_CORES_NUM
	 * TODO: remove that dependency */
	assert(cmd_ctx->cores_num <= ESP32_APPTRACE_MAX_CORES_NUM && "Too many cores number!");

	struct xtensa *xtensa = target->arch_info;
	if (xtensa->common_magic == XTENSA_COMMON_MAGIC) {
		cmd_ctx->hw = target_to_esp_xtensa(target)->apptrace.hw;
	} else { /* TODO: riscv is not supported yet */
		command_print(cmd, "Unsupported target arch 0x%X", xtensa->common_magic);
		return ERROR_FAIL;
	}

	cmd_ctx->max_trace_block_sz = cmd_ctx->hw->max_block_size_get(cmd_ctx->cpus[0]);
	if (cmd_ctx->max_trace_block_sz == 0) {
		command_print(cmd, "Failed to get max trace block size!");
		return ERROR_FAIL;
	}
	LOG_INFO("Total trace memory: %" PRIu32 " bytes", cmd_ctx->max_trace_block_sz);

	INIT_LIST_HEAD(&cmd_ctx->ready_trace_blocks);
	INIT_LIST_HEAD(&cmd_ctx->free_trace_blocks);
	for (unsigned int i = 0; i < ESP_APPTRACE_BLOCKS_POOL_SZ; i++) {
		struct esp32_apptrace_block *block = calloc(1, sizeof(struct esp32_apptrace_block));
		if (!block) {
			command_print(cmd, "Failed to alloc trace buffer entry!");
			esp32_apptrace_blocks_pool_cleanup(cmd_ctx);
			return ERROR_FAIL;
		}
		block->data = malloc(cmd_ctx->max_trace_block_sz);
		if (!block->data) {
			free(block);
			command_print(cmd, "Failed to alloc trace buffer %" PRIu32 " bytes!", cmd_ctx->max_trace_block_sz);
			esp32_apptrace_blocks_pool_cleanup(cmd_ctx);
			return ERROR_FAIL;
		}
		INIT_LIST_HEAD(&block->node);
		list_add(&block->node, &cmd_ctx->free_trace_blocks);
	}

	cmd_ctx->running = 1;
	if (cmd_ctx->mode != ESP_APPTRACE_CMD_MODE_SYNC) {
		int res = target_register_timer_callback(esp32_apptrace_data_processor,
			0,
			TARGET_TIMER_TYPE_PERIODIC,
			cmd_ctx);
		if (res != ERROR_OK) {
			command_print(cmd, "Failed to start trace data timer callback (%d)!", res);
			esp32_apptrace_blocks_pool_cleanup(cmd_ctx);
			return ERROR_FAIL;
		}
	}

	if (s_time_stats_enable) {
		cmd_ctx->stats.min_blk_read_time = 1000000.0;
		cmd_ctx->stats.min_blk_proc_time = 1000000.0;
	}
	if (duration_start(&cmd_ctx->idle_time) != 0) {
		command_print(cmd, "Failed to start idle time measurement!");
		esp32_apptrace_cmd_ctx_cleanup(cmd_ctx);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int esp32_apptrace_cmd_ctx_cleanup(struct esp32_apptrace_cmd_ctx *cmd_ctx)
{
	esp32_apptrace_blocks_pool_cleanup(cmd_ctx);
	return ERROR_OK;
}

#define ESP32_APPTRACE_CMD_NUM_ARG_CHECK(_cmd_, _arg_, _start_, _end_)	   \
	do { \
		if ((_arg_) == 0 && (_start_) == (_end_)) { \
			command_print(_cmd_, "Invalid '" # _arg_ "' arg!"); \
			return;	\
		} \
	} while (0)

void esp32_apptrace_cmd_args_parse(struct esp32_apptrace_cmd_ctx *cmd_ctx,
	struct esp32_apptrace_cmd_data *cmd_data,
	const char **argv,
	int argc)
{
	char *end;

	cmd_data->poll_period = strtoul(argv[0], &end, 10);
	ESP32_APPTRACE_CMD_NUM_ARG_CHECK(cmd_ctx->cmd, cmd_data->poll_period, argv[0], end);
	if (argc > 1) {
		cmd_data->max_len = strtoul(argv[1], &end, 10);
		ESP32_APPTRACE_CMD_NUM_ARG_CHECK(cmd_ctx->cmd, cmd_data->max_len, argv[1], end);
		if (argc > 2) {
			int32_t tmo = strtol(argv[2], &end, 10);
			ESP32_APPTRACE_CMD_NUM_ARG_CHECK(cmd_ctx->cmd, tmo, argv[2], end);
			cmd_ctx->stop_tmo = 1.0 * tmo;
			if (argc > 3) {
				cmd_data->wait4halt = strtoul(argv[3], &end, 10);
				ESP32_APPTRACE_CMD_NUM_ARG_CHECK(cmd_ctx->cmd, cmd_data->wait4halt, argv[3], end);
				if (argc > 4) {
					cmd_data->skip_len = strtoul(argv[4], &end, 10);
					ESP32_APPTRACE_CMD_NUM_ARG_CHECK(cmd_ctx->cmd, cmd_data->skip_len, argv[4], end);
				}
			}
		}
	}
}

static int esp32_apptrace_core_id_get(struct target *target, uint8_t *hdr_buf)
{
	return ESP32_APPTRACE_USER_BLOCK_CORE(target_buffer_get_u16(target, hdr_buf + APPTRACE_BLOCK_SIZE_OFFSET));
}

static uint32_t esp32_apptrace_usr_block_len_get(struct target *target, uint8_t *hdr_buf, uint32_t *wr_len)
{
	*wr_len = ESP32_APPTRACE_USER_BLOCK_LEN(target_buffer_get_u16(target, hdr_buf + APPTRACE_WR_SIZE_OFFSET));
	return ESP32_APPTRACE_USER_BLOCK_LEN(target_buffer_get_u16(target, hdr_buf + APPTRACE_BLOCK_SIZE_OFFSET));
}

static int esp32_apptrace_cmd_init(struct esp32_apptrace_cmd_ctx *cmd_ctx,
	struct command_invocation *cmd,
	int mode,
	const char **argv,
	int argc)
{
	struct esp32_apptrace_cmd_data *cmd_data;

	if (argc < 1) {
		command_print(cmd, "Not enough args! Need trace data destination!");
		return ERROR_FAIL;
	}

	int res = esp32_apptrace_cmd_ctx_init(cmd_ctx, cmd, mode);
	if (res != ERROR_OK)
		return res;

	cmd_data = calloc(1, sizeof(*cmd_data));
	assert(cmd_data && "No memory for command data!");
	cmd_ctx->cmd_priv = cmd_data;

	/*outfile1 [poll_period [trace_size [stop_tmo [wait4halt [skip_size]]]]] */
	res = esp32_apptrace_dest_init(&cmd_data->data_dest, argv, 1);
	if (res != 1) {	/* only one destination needs to be initialized */
		command_print(cmd, "Wrong args! Needs a trace data destination!");
		free(cmd_data);
		goto on_error;
	}
	cmd_ctx->stop_tmo = -1.0;	/* infinite */
	cmd_data->max_len = UINT32_MAX;
	cmd_data->poll_period = 0 /*ms*/;
	if (argc > 1)
		/* parse remaining args */
		esp32_apptrace_cmd_args_parse(cmd_ctx, cmd_data, &argv[1], argc - 1);

	LOG_USER("App trace params: from %d cores, size %" PRId32 " bytes, stop_tmo %g s, poll period %" PRId32
		" ms, wait_rst %d, skip %" PRId32 " bytes", cmd_ctx->cores_num,
		cmd_data->max_len,
		cmd_ctx->stop_tmo,
		cmd_data->poll_period,
		cmd_data->wait4halt,
		cmd_data->skip_len);

	cmd_ctx->trace_format.hdr_sz = ESP32_APPTRACE_USER_BLOCK_HDR_SZ;
	cmd_ctx->trace_format.core_id_get = esp32_apptrace_core_id_get;
	cmd_ctx->trace_format.usr_block_len_get = esp32_apptrace_usr_block_len_get;
	return ERROR_OK;
on_error:
	command_print(cmd, "Not enough args! Need %d trace data destinations!", cmd_ctx->cores_num);
	cmd_ctx->running = 0;
	esp32_apptrace_cmd_ctx_cleanup(cmd_ctx);
	return res;
}

static int esp32_apptrace_cmd_cleanup(struct esp32_apptrace_cmd_ctx *cmd_ctx)
{
	struct esp32_apptrace_cmd_data *cmd_data = cmd_ctx->cmd_priv;

	esp32_apptrace_dest_cleanup(&cmd_data->data_dest, 1);
	free(cmd_data);
	cmd_ctx->cmd_priv = NULL;
	esp32_apptrace_cmd_ctx_cleanup(cmd_ctx);
	return ERROR_OK;
}

static void esp32_apptrace_print_stats(struct esp32_apptrace_cmd_ctx *ctx)
{
	struct esp32_apptrace_cmd_data *cmd_data = ctx->cmd_priv;
	uint32_t trace_sz = 0;

	if (cmd_data)
		trace_sz = ctx->tot_len > cmd_data->skip_len ? ctx->tot_len - cmd_data->skip_len : 0;
	LOG_USER("Tracing is %s. Size is %" PRId32 " of %" PRId32 " @ %f (%f) KiB/s",
		!ctx->running ? "STOPPED" : "RUNNING",
		trace_sz,
		cmd_data ? cmd_data->max_len : 0,
		duration_kbps(&ctx->read_time, ctx->tot_len),
		duration_kbps(&ctx->read_time, ctx->raw_tot_len));
	LOG_USER("Data: blocks incomplete %" PRId32 ", lost bytes: %" PRId32,
		ctx->stats.incompl_blocks,
		ctx->stats.lost_bytes);
	if (s_time_stats_enable) {
		LOG_USER("Block read time [%f..%f] ms",
			1000 * ctx->stats.min_blk_read_time,
			1000 * ctx->stats.max_blk_read_time);
		LOG_USER("Block proc time [%f..%f] ms",
			1000 * ctx->stats.min_blk_proc_time,
			1000 * ctx->stats.max_blk_proc_time);
	}
}

static int esp32_apptrace_wait4halt(struct esp32_apptrace_cmd_ctx *ctx, struct target *target)
{
	LOG_USER("Wait for halt...");
	while (!openocd_is_shutdown_pending()) {
		int res = target_poll(target);
		if (res != ERROR_OK)
			return res;
		if (target->state == TARGET_HALTED) {
			LOG_USER("%s: HALTED", target->cmd_name);
			break;
		}
		alive_sleep(500);
	}
	return ERROR_OK;
}

int esp32_apptrace_safe_halt_targets(struct esp32_apptrace_cmd_ctx *ctx,
	struct esp32_apptrace_target_state *targets)
{
	int res = ERROR_OK;

	memset(targets, 0, ctx->cores_num * sizeof(struct esp32_apptrace_target_state));
	/* halt all CPUs */
	LOG_DEBUG("Halt all targets!");
	for (unsigned int k = 0; k < ctx->cores_num; k++) {
		if (!target_was_examined(ctx->cpus[k]))
			continue;
		if (ctx->cpus[k]->state == TARGET_HALTED)
			continue;
		res = target_halt(ctx->cpus[k]);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to halt target (%d)!", res);
			return res;
		}
		res = target_wait_state(ctx->cpus[k], TARGET_HALTED, ESP32_APPTRACE_TGT_STATE_TMO);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to wait halt target %s / %d (%d)!",
				target_name(ctx->cpus[k]),
				ctx->cpus[k]->state,
				res);
			return res;
		}
	}
	/* read current block statuses from CPUs */
	LOG_DEBUG("Read current block statuses");
	for (unsigned int k = 0; k < ctx->cores_num; k++) {
		uint32_t stat;
		res = ctx->hw->status_reg_read(ctx->cpus[k], &stat);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read trace status (%d)!", res);
			return res;
		}
		/* check if some CPU stopped inside tracing regs update critical section */
		if (stat) {
			if (ctx->hw->leave_trace_crit_section_start) {
				res = ctx->hw->leave_trace_crit_section_start(ctx->cpus[k]);
				if (res != ERROR_OK)
					return res;
			}
			uint32_t bp_addr = stat;
			res = breakpoint_add(ctx->cpus[k], bp_addr, 1, BKPT_HARD);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to set breakpoint (%d)!", res);
				return res;
			}
			while (stat) {
				/* allow this CPU to leave ERI write critical section */
				res = target_resume(ctx->cpus[k], 1, 0, 1, 0);
				if (res != ERROR_OK) {
					LOG_ERROR("Failed to resume target (%d)!", res);
					breakpoint_remove(ctx->cpus[k], bp_addr);
					return res;
				}
				/* wait for CPU to be halted on BP */
				enum target_debug_reason debug_reason = DBG_REASON_UNDEFINED;
				while (debug_reason != DBG_REASON_BREAKPOINT) {
					res = target_wait_state(ctx->cpus[k], TARGET_HALTED,
						ESP32_APPTRACE_TGT_STATE_TMO);
					if (res != ERROR_OK) {
						LOG_ERROR("Failed to wait halt on bp (%d)!", res);
						breakpoint_remove(ctx->cpus[k], bp_addr);
						return res;
					}
					debug_reason = ctx->cpus[k]->debug_reason;
				}
				res = ctx->hw->status_reg_read(ctx->cpus[k], &stat);
				if (res != ERROR_OK) {
					LOG_ERROR("Failed to read trace status (%d)!", res);
					breakpoint_remove(ctx->cpus[k], bp_addr);
					return res;
				}
			}
			breakpoint_remove(ctx->cpus[k], bp_addr);
			if (ctx->hw->leave_trace_crit_section_stop) {
				res = ctx->hw->leave_trace_crit_section_stop(ctx->cpus[k]);
				if (res != ERROR_OK)
					return res;
			}
		}
		res = ctx->hw->data_len_read(ctx->cpus[k], &targets[k].block_id, &targets[k].data_len);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read trace status (%d)!", res);
			return res;
		}
	}

	return ERROR_OK;
}

static int esp32_apptrace_connect_targets(struct esp32_apptrace_cmd_ctx *ctx,
	bool conn,
	bool resume_target)
{
	struct esp32_apptrace_target_state target_to_connect[ESP32_APPTRACE_MAX_CORES_NUM];

	if (conn)
		LOG_USER("Connect targets...");
	else
		LOG_USER("Disconnect targets...");

	int res = esp32_apptrace_safe_halt_targets(ctx, target_to_connect);
	if (res != ERROR_OK) {
		command_print(ctx->cmd, "Failed to halt targets (%d)!", res);
		return res;
	}
	if (ctx->cores_num > 1) {
		/* set block ids to the highest value */
		uint32_t max_id = 0;
		for (unsigned int k = 0; k < ctx->cores_num; k++) {
			if (target_to_connect[k].block_id > max_id)
				max_id = target_to_connect[k].block_id;
		}
		for (unsigned int k = 0; k < ctx->cores_num; k++)
			target_to_connect[k].block_id = max_id;
	}
	for (unsigned int k = 0; k < ctx->cores_num; k++) {
		/* update host connected status */
		res = ctx->hw->ctrl_reg_write(ctx->cpus[k],
			target_to_connect[k].block_id,
			0 /*ack target data*/,
			conn,
			false /*no host data*/);
		if (res != ERROR_OK) {
			command_print(ctx->cmd, "Failed to read trace status (%d)!", res);
			return res;
		}
	}
	if (resume_target) {
		LOG_DEBUG("Resume targets");
		bool smp_resumed = false;
		for (unsigned int k = 0; k < ctx->cores_num; k++) {
			if (smp_resumed && ctx->cpus[k]->smp) {
				/* in SMP mode we need to call target_resume for one core only */
				continue;
			}
			res = target_resume(ctx->cpus[k], 1, 0, 1, 0);
			if (res != ERROR_OK) {
				command_print(ctx->cmd, "Failed to resume target (%d)!", res);
				return res;
			}
			if (ctx->cpus[k]->smp)
				smp_resumed = true;
		}
	}
	if (conn)
		LOG_INFO("Targets connected.");
	else
		LOG_INFO("Targets disconnected.");
	return ERROR_OK;
}

int esp_apptrace_usr_block_write(const struct esp32_apptrace_hw *hw, struct target *target,
	uint32_t block_id,
	const uint8_t *data,
	uint32_t size)
{
	struct esp_apptrace_host2target_hdr hdr = { .block_sz = size };
	uint32_t buf_sz[2] = { sizeof(hdr), size };
	const uint8_t *bufs[2] = { (const uint8_t *)&hdr, data };

	if (size > hw->usr_block_max_size_get(target)) {
		LOG_ERROR("Too large user block %" PRId32, size);
		return ERROR_FAIL;
	}

	return hw->buffs_write(target,
		ARRAY_SIZE(buf_sz),
		buf_sz,
		bufs,
		block_id,
		true /*ack target data*/,
		true /*host data*/);
}

static uint32_t esp32_apptrace_usr_block_check(struct esp32_apptrace_cmd_ctx *ctx, uint8_t *hdr_buf)
{
	uint32_t wr_len = 0;
	uint32_t usr_len = ctx->trace_format.usr_block_len_get(ctx->target, hdr_buf, &wr_len);
	if (usr_len != wr_len) {
		LOG_ERROR("Incomplete block sz %" PRId32 ", wr %" PRId32, usr_len, wr_len);
		ctx->stats.incompl_blocks++;
		ctx->stats.lost_bytes += usr_len - wr_len;
	}
	return usr_len;
}

int esp32_apptrace_get_data_info(struct esp32_apptrace_cmd_ctx *ctx,
	struct esp32_apptrace_target_state *target_state,
	uint32_t *fired_target_num)
{
	if (fired_target_num)
		*fired_target_num = UINT32_MAX;

	for (unsigned int i = 0; i < ctx->cores_num; i++) {
		int res = ctx->hw->data_len_read(ctx->cpus[i], &target_state[i].block_id, &target_state[i].data_len);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read data len on (%s)!", target_name(ctx->cpus[i]));
			return res;
		}
		if (target_state[i].data_len) {
			LOG_TARGET_DEBUG(ctx->cpus[i], "Block %" PRId32 ", len %" PRId32 " bytes on fired",
				target_state[i].block_id, target_state[i].data_len);
			if (fired_target_num)
				*fired_target_num = i;
			break;
		}
	}
	return ERROR_OK;
}

static int esp32_apptrace_process_data(struct esp32_apptrace_cmd_ctx *ctx,
	unsigned int core_id,
	uint8_t *data,
	uint32_t data_len)
{
	struct esp32_apptrace_cmd_data *cmd_data = ctx->cmd_priv;

	LOG_DEBUG("Got block %" PRId32 " bytes [%x %x...%x %x]", data_len, data[12], data[13],
		data[data_len - 2], data[data_len - 1]);
	if (ctx->tot_len + data_len > cmd_data->skip_len) {
		uint32_t wr_idx = 0, wr_chunk_len = data_len;
		if (ctx->tot_len < cmd_data->skip_len) {
			wr_chunk_len = (ctx->tot_len + wr_chunk_len) - cmd_data->skip_len;
			wr_idx = cmd_data->skip_len - ctx->tot_len;
		}
		if (ctx->tot_len + wr_chunk_len > cmd_data->max_len)
			wr_chunk_len -= (ctx->tot_len + wr_chunk_len - cmd_data->skip_len) - cmd_data->max_len;
		if (wr_chunk_len > 0) {
			int res = cmd_data->data_dest.write(cmd_data->data_dest.priv, data + wr_idx, wr_chunk_len);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to write %" PRId32 " bytes to dest 0!", data_len);
				return res;
			}
		}
		ctx->tot_len += wr_chunk_len;
	} else {
		ctx->tot_len += data_len;
	}

	if (cmd_data->data_dest.log_progress)
		LOG_USER("%" PRId32 " ", ctx->tot_len);
	/* check for stop condition */
	if (ctx->tot_len > cmd_data->skip_len && (ctx->tot_len - cmd_data->skip_len >= cmd_data->max_len)) {
		ctx->running = 0;
		if (duration_measure(&ctx->read_time) != 0) {
			LOG_ERROR("Failed to stop trace read time measure!");
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static int esp32_apptrace_handle_trace_block(struct esp32_apptrace_cmd_ctx *ctx,
	struct esp32_apptrace_block *block)
{
	uint32_t processed = 0;
	uint32_t hdr_sz = ctx->trace_format.hdr_sz;

	LOG_DEBUG("Got block %" PRId32 " bytes", block->data_len);
	/* process user blocks one by one */
	while (processed < block->data_len) {
		LOG_DEBUG("Process usr block %" PRId32 "/%" PRId32, processed, block->data_len);
		/* process user block */
		uint32_t usr_len = esp32_apptrace_usr_block_check(ctx, block->data + processed);
		int core_id = ctx->trace_format.core_id_get(ctx->target, block->data + processed);
		/* process user data */
		int res = ctx->process_data(ctx, core_id, block->data + processed + hdr_sz, usr_len);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to process %" PRId32 " bytes!", usr_len);
			return res;
		}
		processed += usr_len + hdr_sz;
	}
	return ERROR_OK;
}

static int esp32_apptrace_data_processor(void *priv)
{
	struct esp32_apptrace_cmd_ctx *ctx = (struct esp32_apptrace_cmd_ctx *)priv;

	if (!ctx->running)
		return ERROR_OK;

	struct esp32_apptrace_block *block = esp32_apptrace_ready_block_get(ctx);
	if (!block)
		return ERROR_OK;

	int res = esp32_apptrace_handle_trace_block(ctx, block);
	if (res != ERROR_OK) {
		ctx->running = 0;
		LOG_ERROR("Failed to process trace block %" PRId32 " bytes!", block->data_len);
		return res;
	}
	res = esp32_apptrace_block_free(ctx, block);
	if (res != ERROR_OK) {
		ctx->running = 0;
		LOG_ERROR("Failed to free ready block!");
		return res;
	}

	return ERROR_OK;
}

static int esp32_apptrace_check_connection(struct esp32_apptrace_cmd_ctx *ctx)
{
	if (!ctx)
		return ERROR_FAIL;

	unsigned int busy_target_num = 0;

	for (unsigned int i = 0; i < ctx->cores_num; i++) {
		bool conn = true;
		int res = ctx->hw->ctrl_reg_read(ctx->cpus[i], NULL, NULL, &conn);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read apptrace control reg for cpu(%d) res(%d)!", i, res);
			return res;
		}
		if (!conn) {
			uint32_t stat = 0;
			LOG_TARGET_WARNING(ctx->cpus[i], "apptrace connection is lost. Re-connect.");
			res = ctx->hw->status_reg_read(ctx->cpus[i], &stat);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to read trace status (%d)!", res);
				return res;
			}
			if (stat) {
				LOG_TARGET_WARNING(ctx->cpus[i], "in critical state. Retry in next poll");
				if (++busy_target_num == ctx->cores_num) {
					LOG_WARNING("No available core");
					return ERROR_WAIT;
				}
				continue;
			}
			res = ctx->hw->ctrl_reg_write(ctx->cpus[i],
				0,
				0,
				true /*host connected*/,
				false /*no host data*/);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to write apptrace control reg for cpu(%d) res(%d)!", i, res);
				return res;
			}
			if (ctx->stop_tmo != -1.0) {
				/* re-start idle time measurement */
				if (duration_start(&ctx->idle_time) != 0) {
					LOG_ERROR("Failed to re-start idle time measure!");
					return ERROR_FAIL;
				}
			}
		}
	}

	return ERROR_OK;
}

static int esp32_apptrace_poll(void *priv)
{
	struct esp32_apptrace_cmd_ctx *ctx = (struct esp32_apptrace_cmd_ctx *)priv;
	int res;
	uint32_t fired_target_num = 0;
	struct esp32_apptrace_target_state target_state[ESP32_APPTRACE_MAX_CORES_NUM];
	struct duration blk_proc_time;

	if (!ctx->running) {
		if (ctx->auto_clean)
			ctx->auto_clean(ctx);
		return ERROR_FAIL;
	}

	/*  Check for connection is alive.For some reason target and therefore host_connected flag
	 *  might have been reset */
	res = esp32_apptrace_check_connection(ctx);
	if (res != ERROR_OK) {
		if (res != ERROR_WAIT)
			ctx->running = 0;
		return res;
	}

	/* check for data from target */
	res = esp32_apptrace_get_data_info(ctx, target_state, &fired_target_num);
	if (res != ERROR_OK) {
		ctx->running = 0;
		LOG_ERROR("Failed to read data len!");
		return res;
	}
	/* LOG_DEBUG("Block %d (%d bytes) on target (%s)!", target_state[0].block_id,
	 * target_state[0].data_len, target_name(ctx->cpus[0])); */
	if (fired_target_num == UINT32_MAX) {
		/* no data has been received, but block could be switched due to the data transferred
		 * from host to target */
		if (ctx->cores_num > 1) {
			uint32_t max_block_id = 0, min_block_id = ctx->hw->max_block_id;
			/* find maximum block ID and set the same ID in control reg for both cores
			 * */
			for (unsigned int i = 0; i < ctx->cores_num; i++) {
				if (max_block_id < target_state[i].block_id)
					max_block_id = target_state[i].block_id;
				if (min_block_id > target_state[i].block_id)
					min_block_id = target_state[i].block_id;
			}
			/* handle block ID overflow */
			if (max_block_id == ctx->hw->max_block_id && min_block_id == 0)
				max_block_id = 0;
			for (unsigned int i = 0; i < ctx->cores_num; i++) {
				if (max_block_id != target_state[i].block_id) {
					LOG_TARGET_DEBUG(ctx->cpus[i], "Ack empty block %" PRId32 "!", max_block_id);
					res = ctx->hw->ctrl_reg_write(ctx->cpus[i],
						max_block_id,
						0 /*all read*/,
						true /*host connected*/,
						false /*no host data*/);
					if (res != ERROR_OK) {
						ctx->running = 0;
						LOG_TARGET_ERROR(ctx->cpus[i], "Failed to ack empty data block!");
						return res;
					}
				}
			}
			ctx->last_blk_id = max_block_id;
		}
		if (ctx->stop_tmo != -1.0) {
			if (duration_measure(&ctx->idle_time) != 0) {
				ctx->running = 0;
				LOG_ERROR("Failed to measure idle time!");
				return ERROR_FAIL;
			}
			if (duration_elapsed(&ctx->idle_time) >= ctx->stop_tmo) {
				ctx->running = 0;
				LOG_ERROR("Data timeout!");
				return ERROR_FAIL;
			}
		}
		return ERROR_OK;/* no data */
	}
	/* sanity check */
	if (target_state[fired_target_num].data_len > ctx->max_trace_block_sz) {
		ctx->running = 0;
		LOG_ERROR("Too large block size %" PRId32 "!", target_state[fired_target_num].data_len);
		return ERROR_FAIL;
	}
	if (ctx->tot_len == 0) {
		if (duration_start(&ctx->read_time) != 0) {
			ctx->running = 0;
			LOG_ERROR("Failed to start trace read time measurement!");
			return ERROR_FAIL;
		}
	}
	struct esp32_apptrace_block *block = esp32_apptrace_free_block_get(ctx);
	if (!block) {
		ctx->running = 0;
		LOG_TARGET_ERROR(ctx->cpus[fired_target_num], "Failed to get free block for data!");
		return ERROR_FAIL;
	}
	if (s_time_stats_enable) {
		/* read block */
		if (duration_start(&blk_proc_time) != 0) {
			ctx->running = 0;
			LOG_ERROR("Failed to start block read time measurement!");
			return ERROR_FAIL;
		}
	}
	res = ctx->hw->data_read(ctx->cpus[fired_target_num], target_state[fired_target_num].data_len, block->data,
		target_state[fired_target_num].block_id,
		/* do not ack target data in sync mode,
		   esp32_apptrace_handle_trace_block() can write response data and will do ack thereafter */
		ctx->mode != ESP_APPTRACE_CMD_MODE_SYNC);
	if (res != ERROR_OK) {
		ctx->running = 0;
		LOG_TARGET_ERROR(ctx->cpus[fired_target_num], "Failed to read data!");
		return res;
	}
	ctx->last_blk_id = target_state[fired_target_num].block_id;
	block->data_len = target_state[fired_target_num].data_len;
	ctx->raw_tot_len += block->data_len;
	if (s_time_stats_enable) {
		if (duration_measure(&blk_proc_time) != 0) {
			ctx->running = 0;
			LOG_ERROR("Failed to measure block read time!");
			return ERROR_FAIL;
		}
		/* update stats */
		float brt = duration_elapsed(&blk_proc_time);
		if (brt > ctx->stats.max_blk_read_time)
			ctx->stats.max_blk_read_time = brt;
		if (brt < ctx->stats.min_blk_read_time)
			ctx->stats.min_blk_read_time = brt;

		if (duration_start(&blk_proc_time) != 0) {
			ctx->running = 0;
			LOG_ERROR("Failed to start block proc time measurement!");
			return ERROR_FAIL;
		}
	}
	/* in sync mode do not ack target data on other cores, esp32_apptrace_handle_trace_block() can write response
	 * data and will do ack thereafter */
	if (ctx->mode != ESP_APPTRACE_CMD_MODE_SYNC) {
		for (unsigned int i = 0; i < ctx->cores_num; i++) {
			if (i == fired_target_num)
				continue;
			res = ctx->hw->ctrl_reg_write(ctx->cpus[i],
				ctx->last_blk_id,
				0 /*all read*/,
				true /*host connected*/,
				false /*no host data*/);
			if (res != ERROR_OK) {
				ctx->running = 0;
				LOG_TARGET_ERROR(ctx->cpus[i], "Failed to ack data!");
				return res;
			}
			LOG_TARGET_DEBUG(ctx->cpus[i], "Ack block %" PRId32, ctx->last_blk_id);
		}
		res = esp32_apptrace_ready_block_put(ctx, block);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_TARGET_ERROR(ctx->cpus[fired_target_num], "Failed to put ready block of data!");
			return res;
		}
	} else {
		res = esp32_apptrace_handle_trace_block(ctx, block);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_ERROR("Failed to process trace block %" PRId32 " bytes!", block->data_len);
			return res;
		}
		res = esp32_apptrace_block_free(ctx, block);
		if (res != ERROR_OK) {
			ctx->running = 0;
			LOG_ERROR("Failed to free ready block!");
			return res;
		}
	}
	if (ctx->stop_tmo != -1.0) {
		/* start idle time measurement */
		if (duration_start(&ctx->idle_time) != 0) {
			ctx->running = 0;
			LOG_ERROR("Failed to start idle time measure!");
			return ERROR_FAIL;
		}
	}
	if (s_time_stats_enable) {
		if (duration_measure(&blk_proc_time) != 0) {
			ctx->running = 0;
			LOG_ERROR("Failed to stop block proc time measure!");
			return ERROR_FAIL;
		}
		/* update stats */
		float bt = duration_elapsed(&blk_proc_time);
		if (bt > ctx->stats.max_blk_proc_time)
			ctx->stats.max_blk_proc_time = bt;
		if (bt < ctx->stats.min_blk_proc_time)
			ctx->stats.min_blk_proc_time = bt;
	}
	return ERROR_OK;
}

static inline bool is_sysview_mode(int mode)
{
	return mode == ESP_APPTRACE_CMD_MODE_SYSVIEW || mode ==	ESP_APPTRACE_CMD_MODE_SYSVIEW_MCORE;
}

static void esp32_apptrace_cmd_stop(struct esp32_apptrace_cmd_ctx *ctx)
{
	if (duration_measure(&ctx->read_time) != 0)
		LOG_ERROR("Failed to stop trace read time measurement!");
	int res = target_unregister_timer_callback(esp32_apptrace_poll, ctx);
	if (res != ERROR_OK)
		LOG_ERROR("Failed to unregister target timer handler (%d)!", res);
	if (is_sysview_mode(ctx->mode)) {
		/* stop tracing */
		res = esp32_sysview_stop(ctx);
		if (res != ERROR_OK)
			LOG_ERROR("sysview: Failed to stop tracing!");
	}
	/* data processor is alive, so wait for all received blocks to be processed */
	res = esp32_apptrace_wait_tracing_finished(ctx);
	if (res != ERROR_OK)
		LOG_ERROR("Failed to wait for pended blocks (%d)!", res);
	res = esp32_apptrace_connect_targets(ctx, false, ctx->target_state == TARGET_RUNNING);
	if (res != ERROR_OK)
		LOG_ERROR("Failed to disconnect targets (%d)!", res);
	esp32_apptrace_print_stats(ctx);
	res = esp32_apptrace_cmd_cleanup(ctx);
	if (res != ERROR_OK)
		LOG_ERROR("Failed to cleanup cmd ctx (%d)!", res);
}

/* this function must be called after connecting to targets */
static int esp32_sysview_start(struct esp32_apptrace_cmd_ctx *ctx)
{
	uint8_t cmds[] = { SEGGER_SYSVIEW_COMMAND_ID_START };
	uint32_t fired_target_num = 0;
	struct esp32_apptrace_target_state target_state[ESP32_APPTRACE_MAX_CORES_NUM] = {{0}};
	struct esp32_sysview_cmd_data *cmd_data = ctx->cmd_priv;

	/* get current block id */
	int res = esp32_apptrace_get_data_info(ctx, target_state, &fired_target_num);
	if (res != ERROR_OK) {
		LOG_ERROR("sysview: Failed to read target data info!");
		return res;
	}
	if (fired_target_num == UINT32_MAX) {
		/* it can happen that there is no pending target data, but block was switched
		 * in this case block_ids on both CPUs are equal, so select the first one */
		fired_target_num = 0;
	}
	/* start tracing */
	res = esp_apptrace_usr_block_write(ctx->hw, ctx->cpus[fired_target_num], target_state[fired_target_num].block_id,
		cmds, sizeof(cmds));
	if (res != ERROR_OK) {
		LOG_ERROR("sysview: Failed to start tracing!");
		return res;
	}
	cmd_data->sv_trace_running = 1;
	return res;
}

static int esp32_sysview_stop(struct esp32_apptrace_cmd_ctx *ctx)
{
	uint32_t old_block_id, fired_target_num = 0, empty_target_num = 0;
	struct esp32_apptrace_target_state target_state[ESP32_APPTRACE_MAX_CORES_NUM];
	struct esp32_sysview_cmd_data *cmd_data = ctx->cmd_priv;
	uint8_t cmds[] = { SEGGER_SYSVIEW_COMMAND_ID_STOP };
	struct duration wait_time;

	struct esp32_apptrace_block *block = esp32_apptrace_free_block_get(ctx);
	if (!block) {
		LOG_ERROR("Failed to get free block for data on (%s)!", target_name(ctx->cpus[fired_target_num]));
		return ERROR_FAIL;
	}

	/* halt all CPUs (not only one), otherwise it can happen that there is no target data and
	 * while we are queueing commands another CPU switches tracing block */
	int res = esp32_apptrace_safe_halt_targets(ctx, target_state);
	if (res != ERROR_OK) {
		LOG_ERROR("sysview: Failed to halt targets (%d)!", res);
		return res;
	}
	/* it can happen that there is no pending target data
	 * in this case block_ids on both CPUs are equal, so the first one will be selected */
	for (unsigned int k = 0; k < ctx->cores_num; k++) {
		if (target_state[k].data_len) {
			fired_target_num = k;
			break;
		}
	}
	if (target_state[fired_target_num].data_len) {
		/* read pending data without ack, they will be acked when stop command is queued */
		res = ctx->hw->data_read(ctx->cpus[fired_target_num], target_state[fired_target_num].data_len, block->data,
			target_state[fired_target_num].block_id,
			false /*no ack target data*/);
		if (res != ERROR_OK) {
			LOG_ERROR("sysview: Failed to read data on (%s)!", target_name(ctx->cpus[fired_target_num]));
			return res;
		}
		/* process data */
		block->data_len = target_state[fired_target_num].data_len;
		res = esp32_apptrace_handle_trace_block(ctx, block);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to process trace block %" PRId32 " bytes!", block->data_len);
			return res;
		}
	}
	/* stop tracing and ack target data */
	res = esp_apptrace_usr_block_write(ctx->hw, ctx->cpus[fired_target_num], target_state[fired_target_num].block_id,
		cmds,
		sizeof(cmds));
	if (res != ERROR_OK) {
		LOG_ERROR("sysview: Failed to stop tracing!");
		return res;
	}
	if (ctx->cores_num > 1) {
		empty_target_num = fired_target_num ? 0 : 1;
		/* ack target data on another CPU */
		res = ctx->hw->ctrl_reg_write(ctx->cpus[empty_target_num], target_state[fired_target_num].block_id,
			0 /*target data ack*/,
			true /*host connected*/,
			false /*no host data*/);
		if (res != ERROR_OK) {
			LOG_ERROR("sysview: Failed to ack data on target '%s' (%d)!",
				target_name(ctx->cpus[empty_target_num]), res);
			return res;
		}
	}
	/* resume targets to allow command processing */
	LOG_INFO("Resume targets");
	bool smp_resumed = false;
	for (unsigned int k = 0; k < ctx->cores_num; k++) {
		if (smp_resumed && ctx->cpus[k]->smp) {
			/* in SMP mode we need to call target_resume for one core only */
			continue;
		}
		res = target_resume(ctx->cpus[k], 1, 0, 1, 0);
		if (res != ERROR_OK) {
			LOG_ERROR("sysview: Failed to resume target '%s' (%d)!", target_name(ctx->cpus[k]), res);
			return res;
		}
		if (ctx->cpus[k]->smp)
			smp_resumed = true;
	}
	/* wait for block switch (command sent), so we can disconnect from targets */
	old_block_id = target_state[fired_target_num].block_id;
	if (duration_start(&wait_time) != 0) {
		LOG_ERROR("Failed to start trace stop timeout measurement!");
		return ERROR_FAIL;
	}
	/* we are waiting for the last data from tracing block and also there can be data in the pended
	 * data buffer */
	/* so we are expecting two TRX block switches at most or stopping due to timeout */
	while (cmd_data->sv_trace_running) {
		res = esp32_apptrace_get_data_info(ctx, target_state, &fired_target_num);
		if (res != ERROR_OK) {
			LOG_ERROR("sysview: Failed to read targets data info!");
			return res;
		}
		if (fired_target_num == UINT32_MAX) {
			/* it can happen that there is no pending (last) target data, but block was
			 * switched */
			/* in this case block_ids on both CPUs are equal, so select the first one */
			fired_target_num = 0;
		}
		if (target_state[fired_target_num].block_id != old_block_id) {
			if (target_state[fired_target_num].data_len) {
				/* read last data and ack them */
				res = ctx->hw->data_read(ctx->cpus[fired_target_num],
					target_state[fired_target_num].data_len,
					block->data,
					target_state[fired_target_num].block_id,
					true /*ack target data*/);
				if (res != ERROR_OK) {
					LOG_ERROR("sysview: Failed to read last data on (%s)!", target_name(ctx->cpus[fired_target_num]));
				} else {
					if (ctx->cores_num > 1) {
						/* ack target data on another CPU */
						empty_target_num = fired_target_num ? 0 : 1;
						res = ctx->hw->ctrl_reg_write(ctx->cpus[empty_target_num],
							target_state[fired_target_num].block_id,
							0 /*all read*/,
							true /*host connected*/,
							false /*no host data*/);
						if (res != ERROR_OK) {
							LOG_ERROR("sysview: Failed to ack data on target '%s' (%d)!",
								target_name(ctx->cpus[empty_target_num]), res);
							return res;
						}
					}
					/* process data */
					block->data_len = target_state[fired_target_num].data_len;
					res = esp32_apptrace_handle_trace_block(ctx, block);
					if (res != ERROR_OK) {
						LOG_ERROR("Failed to process trace block %" PRId32 " bytes!",
							block->data_len);
						return res;
					}
				}
				old_block_id = target_state[fired_target_num].block_id;
			}
		}
		if (duration_measure(&wait_time) != 0) {
			LOG_ERROR("Failed to start trace stop timeout measurement!");
			return ERROR_FAIL;
		}
		const float stop_tmo = LOG_LEVEL_IS(LOG_LVL_DEBUG) ? 30.0 : 0.5;
		if (duration_elapsed(&wait_time) >= stop_tmo) {
			LOG_INFO("Stop waiting for the last data due to timeout.");
			break;
		}
	}
	return res;
}

static int esp32_cmd_apptrace_generic(struct command_invocation *cmd, int mode, const char **argv, int argc)
{
	static struct esp32_apptrace_cmd_ctx s_at_cmd_ctx;
	struct esp32_apptrace_cmd_data *cmd_data;
	int res = ERROR_FAIL;
	enum target_state old_state;
	struct target *target = get_current_target(CMD_CTX);

	if (argc < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* command can be invoked on unexamined core, if so find examined one */
	if (target->smp && !target_was_examined(target)) {
		struct target_list *head;
		struct target *curr;
		LOG_WARNING("Current target '%s' was not examined!", target_name(target));
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			if (target_was_examined(curr)) {
				target = curr;
				LOG_WARNING("Run command on target '%s'", target_name(target));
				break;
			}
		}
	}
	old_state = target->state;

	if (strcmp(argv[0], "start") == 0) {
		if (is_sysview_mode(mode)) {
			/* init cmd context */
			res = esp32_sysview_cmd_init(&s_at_cmd_ctx,
				cmd,
				mode,
				mode == ESP_APPTRACE_CMD_MODE_SYSVIEW_MCORE,
				&argv[1],
				argc - 1);
			if (res != ERROR_OK) {
				command_print(cmd, "Failed to init cmd ctx (%d)!", res);
				return res;
			}
			cmd_data = s_at_cmd_ctx.cmd_priv;
			if (cmd_data->skip_len != 0) {
				s_at_cmd_ctx.running = 0;
				esp32_sysview_cmd_cleanup(&s_at_cmd_ctx);
				command_print(cmd, "Data skipping not supported!");
				return ERROR_FAIL;
			}
			s_at_cmd_ctx.process_data = esp32_sysview_process_data;
		} else {
			res = esp32_apptrace_cmd_init(&s_at_cmd_ctx,
				cmd,
				mode,
				&argv[1],
				argc - 1);
			if (res != ERROR_OK) {
				command_print(cmd, "Failed to init cmd ctx (%d)!", res);
				return res;
			}
			cmd_data = s_at_cmd_ctx.cmd_priv;
			s_at_cmd_ctx.process_data = esp32_apptrace_process_data;
		}
		s_at_cmd_ctx.auto_clean = esp32_apptrace_cmd_stop;
		if (cmd_data->wait4halt) {
			res = esp32_apptrace_wait4halt(&s_at_cmd_ctx, target);
			if (res != ERROR_OK) {
				command_print(cmd, "Failed to wait for halt target (%d)!", res);
				goto _on_start_error;
			}
		}
		res = esp32_apptrace_connect_targets(&s_at_cmd_ctx, true, old_state == TARGET_RUNNING);
		if (res != ERROR_OK) {
			command_print(cmd, "Failed to connect to targets (%d)!", res);
			goto _on_start_error;
		}
		if (is_sysview_mode(mode)) {
			/* start tracing */
			res = esp32_sysview_start(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				esp32_apptrace_connect_targets(&s_at_cmd_ctx, false, old_state == TARGET_RUNNING);
				s_at_cmd_ctx.running = 0;
				esp32_apptrace_cmd_cleanup(&s_at_cmd_ctx);
				command_print(cmd, "sysview: Failed to start tracing!");
				return res;
			}
		}
		res = target_register_timer_callback(esp32_apptrace_poll,
			cmd_data->poll_period,
			TARGET_TIMER_TYPE_PERIODIC,
			&s_at_cmd_ctx);
		if (res != ERROR_OK) {
			command_print(cmd, "Failed to register target timer handler (%d)!", res);
			goto _on_start_error;
		}
	} else if (strcmp(argv[0], "stop") == 0) {
		if (!s_at_cmd_ctx.running) {
			command_print(cmd, "Tracing is not running!");
			return ERROR_FAIL;
		}
		esp32_apptrace_cmd_stop(&s_at_cmd_ctx);
		return ERROR_OK;
	} else if (strcmp(argv[0], "status") == 0) {
		if (s_at_cmd_ctx.running && duration_measure(&s_at_cmd_ctx.read_time) != 0)
			LOG_ERROR("Failed to measure trace read time!");
		esp32_apptrace_print_stats(&s_at_cmd_ctx);
		return ERROR_OK;
	} else if (strcmp(argv[0], "dump") == 0) {
		if (is_sysview_mode(mode)) {
			command_print(cmd, "Not supported!");
			return ERROR_FAIL;
		}
		/* [dump outfile] - post-mortem dump without connection to targets */
		res = esp32_apptrace_cmd_init(&s_at_cmd_ctx,
			cmd,
			mode,
			&argv[1],
			argc - 1);
		if (res != ERROR_OK) {
			command_print(cmd, "Failed to init cmd ctx (%d)!", res);
			return res;
		}
		s_at_cmd_ctx.stop_tmo = 0.01;	/* use small stop tmo */
		s_at_cmd_ctx.process_data = esp32_apptrace_process_data;
		/* check for exit signal and command completion */
		while (!openocd_is_shutdown_pending() && s_at_cmd_ctx.running) {
			res = esp32_apptrace_poll(&s_at_cmd_ctx);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to poll target for trace data (%d)!", res);
				break;
			}
			/* let registered timer callbacks to run */
			target_call_timer_callbacks();
		}
		if (s_at_cmd_ctx.running) {
			/* data processor is alive, so wait for all received blocks to be processed */
			res = esp32_apptrace_wait_tracing_finished(&s_at_cmd_ctx);
			if (res != ERROR_OK)
				LOG_ERROR("Failed to wait for pended blocks (%d)!", res);
		}
		esp32_apptrace_print_stats(&s_at_cmd_ctx);
		res = esp32_apptrace_cmd_cleanup(&s_at_cmd_ctx);
		if (res != ERROR_OK)
			command_print(cmd, "Failed to cleanup cmd ctx (%d)!", res);
	} else {
		command_print(cmd, "Invalid action '%s'!", argv[0]);
	}

	return res;

_on_start_error:
	s_at_cmd_ctx.running = 0;
	if (is_sysview_mode(mode))
		esp32_sysview_cmd_cleanup(&s_at_cmd_ctx);
	else
		esp32_apptrace_cmd_cleanup(&s_at_cmd_ctx);
	return res;
}

COMMAND_HANDLER(esp32_cmd_apptrace)
{
	return esp32_cmd_apptrace_generic(CMD, ESP_APPTRACE_CMD_MODE_GEN, CMD_ARGV, CMD_ARGC);
}

COMMAND_HANDLER(esp32_cmd_sysview)
{
	return esp32_cmd_apptrace_generic(CMD, ESP_APPTRACE_CMD_MODE_SYSVIEW, CMD_ARGV, CMD_ARGC);
}

COMMAND_HANDLER(esp32_cmd_sysview_mcore)
{
	return esp32_cmd_apptrace_generic(CMD, ESP_APPTRACE_CMD_MODE_SYSVIEW_MCORE, CMD_ARGV, CMD_ARGC);
}

const struct command_registration esp32_apptrace_command_handlers[] = {
	{
		.name = "apptrace",
		.handler = esp32_cmd_apptrace,
		.mode = COMMAND_EXEC,
		.help =
			"App Tracing: application level trace control. Starts, stops or queries tracing process status.",
		.usage =
			"(start <destination> [poll_period [trace_size [stop_tmo [wait4halt [skip_size]]]]) | (stop) | (status) | (dump <destination>)",
	},
	{
		.name = "sysview",
		.handler = esp32_cmd_sysview,
		.mode = COMMAND_EXEC,
		.help =
			"App Tracing: SEGGER SystemView compatible trace control. Starts, stops or queries tracing process status.",
		.usage =
			"(start file://<outfile1> [file://<outfile2>] [poll_period [trace_size [stop_tmo [wait4halt [skip_size]]]]) | (stop) | (status)",
	},
	{
		.name = "sysview_mcore",
		.handler = esp32_cmd_sysview_mcore,
		.mode = COMMAND_EXEC,
		.help =
			"App Tracing: Espressif multi-core SystemView trace control. Starts, stops or queries tracing process status.",
		.usage =
			"(start file://<outfile> [poll_period [trace_size [stop_tmo [wait4halt [skip_size]]]]) | (stop) | (status)",
	},
	COMMAND_REGISTRATION_DONE
};
