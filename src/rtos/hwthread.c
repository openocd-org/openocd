// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include <jtag/jtag.h>
#include "target/target.h"
#include "target/register.h"
#include <target/smp.h>
#include "rtos.h"
#include "helper/log.h"
#include "helper/types.h"
#include "server/gdb_server.h"

static bool hwthread_detect_rtos(struct target *target);
static int hwthread_create(struct target *target);
static int hwthread_update_threads(struct rtos *rtos);
static int hwthread_get_thread_reg(struct rtos *rtos, int64_t thread_id,
		uint32_t reg_num, struct rtos_reg *rtos_reg);
static int hwthread_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs);
static int hwthread_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[]);
static int hwthread_smp_init(struct target *target);
static int hwthread_set_reg(struct rtos *rtos, uint32_t reg_num, uint8_t *reg_value);
static int hwthread_read_buffer(struct rtos *rtos, target_addr_t address,
		uint32_t size, uint8_t *buffer);
static int hwthread_write_buffer(struct rtos *rtos, target_addr_t address,
		uint32_t size, const uint8_t *buffer);

#define HW_THREAD_NAME_STR_SIZE (32)

static inline threadid_t threadid_from_target(const struct target *target)
{
	if (!target->smp)
		return 1;

	threadid_t threadid = 1;
	struct target_list *head;
	foreach_smp_target(head, target->smp_targets) {
		if (target == head->target)
			return threadid;
		++threadid;
	}
	assert(0 && "Target is not found in it's own SMP group!");
	return -1;
}

const struct rtos_type hwthread_rtos = {
	.name = "hwthread",
	.detect_rtos = hwthread_detect_rtos,
	.create = hwthread_create,
	.update_threads = hwthread_update_threads,
	.get_thread_reg_list = hwthread_get_thread_reg_list,
	.get_thread_reg = hwthread_get_thread_reg,
	.get_symbol_list_to_lookup = hwthread_get_symbol_list_to_lookup,
	.smp_init = hwthread_smp_init,
	.set_reg = hwthread_set_reg,
	.read_buffer = hwthread_read_buffer,
	.write_buffer = hwthread_write_buffer,
};

struct hwthread_params {
	int dummy_param;
};

static int hwthread_fill_thread(struct rtos *rtos, struct target *curr, int thread_num, threadid_t tid)
{
	char tmp_str[HW_THREAD_NAME_STR_SIZE];

	memset(tmp_str, 0, HW_THREAD_NAME_STR_SIZE);

	/* thread-id is the index of this core inside the SMP group plus 1 */
	rtos->thread_details[thread_num].threadid = tid;
	/* create the thread name */
	rtos->thread_details[thread_num].exists = true;
	rtos->thread_details[thread_num].thread_name_str = strdup(target_name(curr));
	snprintf(tmp_str, HW_THREAD_NAME_STR_SIZE-1, "state: %s", debug_reason_name(curr));
	rtos->thread_details[thread_num].extra_info_str = strdup(tmp_str);

	return ERROR_OK;
}

static int hwthread_update_threads(struct rtos *rtos)
{
	int threads_found = 0;
	int thread_list_size = 0;
	struct target_list *head;
	struct target *target;
	int64_t current_thread = 0;
	int64_t current_threadid = rtos->current_threadid; /* thread selected by GDB */
	enum target_debug_reason current_reason = DBG_REASON_UNDEFINED;

	if (!rtos)
		return -1;

	target = rtos->target;

	/* wipe out previous thread details if any */
	rtos_free_threadlist(rtos);

	/* determine the number of "threads" */
	if (target->smp) {
		foreach_smp_target(head, target->smp_targets) {
			struct target *curr = head->target;

			if (!target_was_examined(curr))
				continue;

			++thread_list_size;
		}
	} else
		thread_list_size = 1;

	/* restore the threadid which is currently selected by GDB
	 * because rtos_free_threadlist() wipes out it
	 * (GDB thread id is 1-based indexing) */
	if (current_threadid <= thread_list_size)
		rtos->current_threadid = current_threadid;
	else
		LOG_TARGET_WARNING(target, "SMP node change, disconnect GDB from core/thread %" PRId64,
			    current_threadid);

	/* create space for new thread details */
	rtos->thread_details = malloc(sizeof(struct thread_detail) * thread_list_size);

	if (target->smp) {
		/* loop over all threads */
		foreach_smp_target(head, target->smp_targets) {
			struct target *curr = head->target;

			if (!target_was_examined(curr))
				continue;

			threadid_t tid = threadid_from_target(curr);
			hwthread_fill_thread(rtos, curr, threads_found, tid);

			/* find an interesting thread to set as current */
			switch (current_reason) {
			case DBG_REASON_UNDEFINED:
				current_reason = curr->debug_reason;
				current_thread = tid;
				break;
			case DBG_REASON_SINGLESTEP:
				/* single-step can only be overridden by itself */
				if (curr->debug_reason == DBG_REASON_SINGLESTEP) {
					if (tid == rtos->current_threadid)
						current_thread = tid;
				}
				break;
			case DBG_REASON_BREAKPOINT:
				/* single-step overrides breakpoint */
				if (curr->debug_reason == DBG_REASON_SINGLESTEP) {
					current_reason = curr->debug_reason;
					current_thread = tid;
				} else if (curr->debug_reason == DBG_REASON_BREAKPOINT) {
					/* multiple breakpoints, prefer gdbs' threadid */
					if (tid == rtos->current_threadid)
						current_thread = tid;
				}
				break;
			case DBG_REASON_WATCHPOINT:
				/* breakpoint and single-step override watchpoint */
				if (curr->debug_reason == DBG_REASON_SINGLESTEP ||
						curr->debug_reason == DBG_REASON_BREAKPOINT) {
					current_reason = curr->debug_reason;
					current_thread = tid;
				}
				break;
			case DBG_REASON_DBGRQ:
				/* all other reasons override debug-request */
				if (curr->debug_reason == DBG_REASON_SINGLESTEP ||
						curr->debug_reason == DBG_REASON_WATCHPOINT ||
						curr->debug_reason == DBG_REASON_BREAKPOINT) {
					current_reason = curr->debug_reason;
					current_thread = tid;
				} else if (curr->debug_reason == DBG_REASON_DBGRQ) {
					if (tid == rtos->current_threadid)
						current_thread = tid;
				}

				break;

			default:
				break;
			}

			threads_found++;
		}
	} else {
		current_thread = 1;
		hwthread_fill_thread(rtos, target, threads_found, current_thread);
		threads_found++;
	}

	rtos->thread_count = threads_found;

	/* we found an interesting thread, set it as current */
	if (current_thread != 0)
		rtos->current_thread = current_thread;
	else if (rtos->current_threadid != 0)
		rtos->current_thread = rtos->current_threadid;
	else
		rtos->current_thread = threadid_from_target(target);

	LOG_TARGET_DEBUG(target, "%s current_thread=%i", __func__,
		(int)rtos->current_thread);
	return 0;
}

static int hwthread_smp_init(struct target *target)
{
	return hwthread_update_threads(target->rtos);
}

static struct target *hwthread_find_thread(struct target *target, threadid_t thread_id)
{
	/* Find the thread with that thread_id (index in SMP group plus 1)*/
	if (!(target && target->smp))
		return target;
	struct target_list *head;
	threadid_t tid = 1;
	foreach_smp_target(head, target->smp_targets) {
		if (thread_id == tid)
			return head->target;
		++tid;
	}
	return NULL;
}

static int hwthread_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **rtos_reg_list, int *rtos_reg_list_size)
{
	if (!rtos)
		return ERROR_FAIL;

	struct target *target = rtos->target;

	struct target *curr = hwthread_find_thread(target, thread_id);
	if (!curr)
		return ERROR_FAIL;

	if (!target_was_examined(curr))
		return ERROR_FAIL;

	int reg_list_size;
	struct reg **reg_list;
	int retval = target_get_gdb_reg_list(curr, &reg_list, &reg_list_size,
			REG_CLASS_GENERAL);
	if (retval != ERROR_OK)
		return retval;

	int j = 0;
	for (int i = 0; i < reg_list_size; i++) {
		if (!reg_list[i] || !reg_list[i]->exist || reg_list[i]->hidden)
			continue;
		j++;
	}
	*rtos_reg_list_size = j;
	*rtos_reg_list = calloc(*rtos_reg_list_size, sizeof(struct rtos_reg));
	if (!*rtos_reg_list) {
		free(reg_list);
		return ERROR_FAIL;
	}

	j = 0;
	for (int i = 0; i < reg_list_size; i++) {
		if (!reg_list[i] || !reg_list[i]->exist || reg_list[i]->hidden)
			continue;
		if (!reg_list[i]->valid) {
			retval = reg_list[i]->type->get(reg_list[i]);
			if (retval != ERROR_OK) {
				LOG_TARGET_ERROR(curr, "Couldn't get register %s",
					reg_list[i]->name);
				free(reg_list);
				free(*rtos_reg_list);
				return retval;
			}
		}
		(*rtos_reg_list)[j].number = reg_list[i]->number;
		(*rtos_reg_list)[j].size = reg_list[i]->size;
		memcpy((*rtos_reg_list)[j].value, reg_list[i]->value,
				DIV_ROUND_UP(reg_list[i]->size, 8));
		j++;
	}
	free(reg_list);

	return ERROR_OK;
}

static int hwthread_get_thread_reg(struct rtos *rtos, int64_t thread_id,
		uint32_t reg_num, struct rtos_reg *rtos_reg)
{
	if (!rtos)
		return ERROR_FAIL;

	struct target *target = rtos->target;

	struct target *curr = hwthread_find_thread(target, thread_id);
	if (!curr) {
		LOG_TARGET_ERROR(target, "Couldn't find RTOS thread for id %" PRId64,
			thread_id);
		return ERROR_FAIL;
	}

	if (!target_was_examined(curr)) {
		LOG_TARGET_ERROR(curr, "Target hasn't been examined yet.");
		return ERROR_FAIL;
	}

	struct reg *reg = register_get_by_number(curr->reg_cache, reg_num, true);
	if (!reg) {
		LOG_TARGET_ERROR(curr, "Couldn't find register %" PRIu32 " in thread %" PRId64,
			reg_num, thread_id);
		return ERROR_FAIL;
	}

	if (reg->type->get(reg) != ERROR_OK)
		return ERROR_FAIL;

	rtos_reg->number = reg->number;
	rtos_reg->size = reg->size;
	unsigned int bytes = (reg->size + 7) / 8;
	assert(bytes <= sizeof(rtos_reg->value));
	memcpy(rtos_reg->value, reg->value, bytes);

	return ERROR_OK;
}

static int hwthread_set_reg(struct rtos *rtos, uint32_t reg_num, uint8_t *reg_value)
{
	if (!rtos)
		return ERROR_FAIL;

	struct target *target = rtos->target;

	struct target *curr = hwthread_find_thread(target, rtos->current_thread);
	if (!curr)
		return ERROR_FAIL;

	struct reg *reg = register_get_by_number(curr->reg_cache, reg_num, true);
	if (!reg)
		return ERROR_FAIL;

	return reg->type->set(reg, reg_value);
}

static int hwthread_get_symbol_list_to_lookup(struct symbol_table_elem *symbol_list[])
{
	/* return an empty list, we don't have any symbols to look up */
	*symbol_list = calloc(1, sizeof(struct symbol_table_elem));
	(*symbol_list)[0].symbol_name = NULL;
	return 0;
}

static int hwthread_target_for_threadid(struct connection *connection, int64_t thread_id, struct target **p_target)
{
	struct target *target = get_target_from_connection(connection);

	struct target *curr = hwthread_find_thread(target, thread_id);
	if (!curr)
		return ERROR_FAIL;

	*p_target = curr;

	return ERROR_OK;
}

static bool hwthread_detect_rtos(struct target *target)
{
	/* always return 0, avoid auto-detection */
	return false;
}

static int hwthread_thread_packet(struct connection *connection, const char *packet, int packet_size)
{
	if (packet[0] == 'H' && packet[1] == 'g') {
		int64_t current_threadid;
		sscanf(packet, "Hg%16" SCNx64, &current_threadid);

		struct target *target = get_target_from_connection(connection);

		if (current_threadid > 0) {
			struct target *curr = NULL;
			if (hwthread_target_for_threadid(connection, current_threadid, &curr) != ERROR_OK) {
				LOG_TARGET_ERROR(target, "hwthread: cannot find thread id %" PRId64,
					current_threadid);
				gdb_put_packet(connection, "E01", 3);
				return ERROR_FAIL;
			}
			target->rtos->current_thread = current_threadid;
		} else if (current_threadid == 0 || current_threadid == -1) {
			target->rtos->current_thread = threadid_from_target(target);
		}

		target->rtos->current_threadid = current_threadid;

		gdb_put_packet(connection, "OK", 2);
		return ERROR_OK;
	}

	return rtos_thread_packet(connection, packet, packet_size);
}

static int hwthread_create(struct target *target)
{
	LOG_TARGET_INFO(target, "Hardware thread awareness created");

	target->rtos->rtos_specific_params = NULL;
	target->rtos->current_thread = 0;
	target->rtos->thread_details = NULL;
	target->rtos->gdb_target_for_threadid = hwthread_target_for_threadid;
	target->rtos->gdb_thread_packet = hwthread_thread_packet;
	return 0;
}

static int hwthread_read_buffer(struct rtos *rtos, target_addr_t address,
		uint32_t size, uint8_t *buffer)
{
	if (!rtos)
		return ERROR_FAIL;

	struct target *target = rtos->target;

	struct target *curr = hwthread_find_thread(target, rtos->current_thread);
	if (!curr)
		return ERROR_FAIL;

	return target_read_buffer(curr, address, size, buffer);
}

static int hwthread_write_buffer(struct rtos *rtos, target_addr_t address,
		uint32_t size, const uint8_t *buffer)
{
	if (!rtos)
		return ERROR_FAIL;

	struct target *target = rtos->target;

	struct target *curr = hwthread_find_thread(target, rtos->current_thread);
	if (!curr)
		return ERROR_FAIL;

	return target_write_buffer(curr, address, size, buffer);
}
