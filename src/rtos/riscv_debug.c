#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "riscv_debug.h"
#include "target/register.h"
#include "target/target.h"
#include "target/riscv/riscv.h"
#include "server/gdb_server.h"
#include "helper/binarybuffer.h"

static int riscv_gdb_thread_packet(struct connection *connection, const char *packet, int packet_size);
static int riscv_gdb_v_packet(struct connection *connection, const char *packet, int packet_size);

static bool riscv_detect_rtos(struct target *target)
{
	LOG_ERROR("riscv_detect_rtos() unimplemented");
	return -1;
}

static int riscv_create_rtos(struct target *target)
{
	LOG_DEBUG("RISC-V Debug 'RTOS' created: this doesn't mean you're running an RTOS, just that you have multi-hart support on RISC-V");
	LOG_WARNING("`-rtos riscv` is deprecated! Please change your configuration to use `-rtos");
	LOG_WARNING("hwthread` instead. To do that, you will have to explicitly list every hart in");
	LOG_WARNING("the system as a separate target. See");
	LOG_WARNING("https://github.com/riscv/riscv-tests/blob/ec6537fc4a527ca88be2f045e01c460e640ab9c5/debug/targets/SiFive/HiFiveUnleashed.cfg#L11");
	LOG_WARNING("for an example.");
	LOG_WARNING("You will have to change your configuration file in any OpenOCD newer than June");
	LOG_WARNING("2020.");

	struct riscv_rtos *r = calloc(1, sizeof(*r));
	target->rtos->rtos_specific_params = r;

	target->rtos->current_threadid = 1;
	target->rtos->current_thread = 1;

	target->rtos->gdb_thread_packet = riscv_gdb_thread_packet;
	target->rtos->gdb_v_packet = riscv_gdb_v_packet;

	return JIM_OK;
}

int riscv_update_threads(struct rtos *rtos)
{
	LOG_DEBUG("Updating the RISC-V Hart List");

	struct target *target = rtos->target;

	/* Figures out how many harts there are on the system. */
	int hart_count = riscv_count_harts(rtos->target);
	if (rtos->thread_count != hart_count) {
		rtos_free_threadlist(rtos);
		rtos->thread_count = hart_count;
		rtos->thread_details = calloc(rtos->thread_count, sizeof(*rtos->thread_details));
		for (int i = 0; i < rtos->thread_count; ++i) {
			LOG_DEBUG("  Setting up Hart %d", i);
			rtos->thread_details[i].threadid = i + 1;
			rtos->thread_details[i].exists = true;
			if (asprintf(&rtos->thread_details[i].thread_name_str, "Hart %d", i) < 0)
				LOG_ERROR("riscv_update_threads() failed asprintf");
			if (asprintf(&rtos->thread_details[i].extra_info_str, "RV%d",
						riscv_xlen_of_hart(target, i)) < 0)
				LOG_ERROR("riscv_update_threads() failed asprintf");
		}
	}
	return JIM_OK;
}

static int riscv_gdb_thread_packet(struct connection *connection, const char *packet, int packet_size)
{
	struct target *target = get_target_from_connection(connection);
	struct rtos *rtos = target->rtos;
	struct riscv_rtos *r = (struct riscv_rtos *)(target->rtos->rtos_specific_params);

	char *packet_stttrr = malloc(packet_size + 1);
	memset(packet_stttrr, '\0', packet_size + 1);
	memcpy(packet_stttrr, packet, packet_size);
	LOG_DEBUG("handling packet '%s'", packet_stttrr);

	switch (packet[0]) {
	case 'q':
		if (strncmp(packet, "qfThreadInfo", 12) == 0) {
			riscv_update_threads(target->rtos);
			r->qs_thread_info_offset = 1;

			char m[16];
			snprintf(m, 16, "m%08x", (int)rtos->thread_details[0].threadid);
			gdb_put_packet(connection, m, strlen(m));
			return ERROR_OK;
		}

		if (strncmp(packet, "qsThreadInfo", 12) == 0) {
			if (r->qs_thread_info_offset >= rtos->thread_count) {
				gdb_put_packet(connection, "l", 1);
				return ERROR_OK;
			}

			int tid = r->qs_thread_info_offset++;
			char m[16];
			snprintf(m, 16, "m%08x", (int)rtos->thread_details[tid].threadid);
			gdb_put_packet(connection, m, strlen(m));
			return ERROR_OK;
		}

		if (strncmp(packet, "qAttached", 9) == 0) {
			gdb_put_packet(connection, "1", 1);
			return ERROR_OK;
		}

		if (strncmp(packet, "qThreadExtraInfo", 16) == 0) {
			char tid_str[32];
			memcpy(tid_str, packet + 17, packet_size - 17);
			tid_str[packet_size - 17] = '\0';
			char *end;
			int tid = strtol(tid_str, &end, 16);
			if (*end != '\0') {
				LOG_ERROR("Got qThreadExtraInfo with non-numeric TID: '%s'", tid_str);
				gdb_put_packet(connection, NULL, 0);
				return ERROR_FAIL;
			}

			char m[16];
			snprintf(m, 16, "hart %d", tid);
			char h[33];
			h[0] = '\0';
			for (size_t i = 0; i < strlen(m); ++i) {
				char byte[3];
				snprintf(byte, 3, "%02x", m[i]);
				strncat(h, byte, 32);
			}
			gdb_put_packet(connection, h, strlen(h));
			return ERROR_OK;
		}

		if (strcmp(packet, "qTStatus") == 0) {
			gdb_put_packet(connection, "T0", 2);
			return ERROR_OK;
		}

		if (strcmp(packet, "qC") == 0) {
			char rep_str[32];
			snprintf(rep_str, 32, "QC%" PRIx64, rtos->current_threadid);
			gdb_put_packet(connection, rep_str, strlen(rep_str));
			return ERROR_OK;
		}

		return GDB_THREAD_PACKET_NOT_CONSUMED;

	case 'Q':
		return GDB_THREAD_PACKET_NOT_CONSUMED;

	case 'H':
		/* ‘H op thread-id’
		 *
		 * Set thread for subsequent operations (‘m’, ‘M’, ‘g’, ‘G’,
		 * et.al.). Depending on the operation to be performed, op
		 * should be ‘c’ for step and continue operations (note that
		 * this is deprecated, supporting the ‘vCont’ command is a
		 * better option), and ‘g’ for other operations. The thread
		 * designator thread-id has the format and interpretation
		 * described in thread-id syntax.
		 *
		 * Reply:
		 * ‘OK’ for success
		 * ‘E NN’ for an error
		 */
	{
		char tid_str[32];
		memcpy(tid_str, packet + 2, packet_size - 2);
		tid_str[packet_size - 2] = '\0';
		char *entptr;
		int tid = strtol(tid_str, &entptr, 16);
		if (*entptr != '\0') {
			LOG_ERROR("Got H packet, but without integer: %s", tid_str);
			return GDB_THREAD_PACKET_NOT_CONSUMED;
		}

		switch (tid) {
		case 0:
		case -1:
			riscv_set_all_rtos_harts(target);
			break;
		default:
			riscv_set_rtos_hartid(target, tid - 1);
			rtos->current_threadid = tid;
			break;
		}

		switch (packet[1]) {
		case 'g':
		case 'c':
			gdb_put_packet(connection, "OK", 2);
			return ERROR_OK;
		default:
			LOG_ERROR("Unknown H packet subtype %2x\n", packet[1]);
			gdb_put_packet(connection, NULL, 0);
			return ERROR_FAIL;
		}
	}

	case 'T':
	{
		char tid_str[32];
		memcpy(tid_str, packet + 1, packet_size - 1);
		tid_str[packet_size - 1] = '\0';
		char *end;
		int tid = strtol(tid_str, &end, 16);
		if (*end != '\0') {
			LOG_ERROR("T packet with non-numeric tid %s", tid_str);
			gdb_put_packet(connection, NULL, 0);
			return ERROR_FAIL;
		}

		riscv_update_threads(target->rtos);
		if (tid <= target->rtos->thread_count) {
			gdb_put_packet(connection, "OK", 2);
			return ERROR_OK;
		} else {
			gdb_put_packet(connection, "E00", 3);
			return ERROR_OK;
		}
	}

	case 'c':
	case 's':
		target->state = TARGET_HALTED;
		return JIM_OK;

	case 'R':
		gdb_put_packet(connection, "E00", 3);
		return JIM_OK;

	default:
		LOG_ERROR("Unknown packet of type 0x%2.2x", packet[0]);
		gdb_put_packet(connection, NULL, 0);
		return JIM_OK;
	}
}

static int riscv_gdb_v_packet(struct connection *connection, const char *packet, int packet_size)
{
	char *packet_stttrr = malloc(packet_size + 1);
	memset(packet_stttrr, '\0', packet_size + 1);
	memcpy(packet_stttrr, packet, packet_size);
	LOG_DEBUG("handling packet '%s'", packet_stttrr);

	struct target *target = get_target_from_connection(connection);

	if (strcmp(packet_stttrr, "vCont?") == 0) {
		static const char *message = "OK";
		gdb_put_packet(connection, (char *)message, strlen(message));
		return JIM_OK;
	}

	int threadid;
	if (sscanf(packet_stttrr, "vCont;s:%d;c", &threadid) == 1) {
		riscv_set_rtos_hartid(target, threadid - 1);
		riscv_step_rtos_hart(target);
		/* Stepping changes the current thread to whichever thread was stepped. */
		target->rtos->current_threadid = threadid;

		gdb_put_packet(connection, "S05", 3);
		return JIM_OK;

	} else if (strcmp(packet_stttrr, "vCont;c") == 0) {
		target_call_event_callbacks(target, TARGET_EVENT_GDB_START);
		target_call_event_callbacks(target, TARGET_EVENT_RESUME_START);
		riscv_set_all_rtos_harts(target);
		riscv_resume(target, 1, 0, 0, 0, false);
		target->state = TARGET_RUNNING;
		gdb_set_frontend_state_running(connection);
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		target_call_event_callbacks(target, TARGET_EVENT_RESUME_END);
		return JIM_OK;

	} else if (strncmp(packet_stttrr, "vCont", 5) == 0) {
		LOG_ERROR("Got unknown vCont-type packet");
	}

	return GDB_THREAD_PACKET_NOT_CONSUMED;
}

static int riscv_get_thread_reg(struct rtos *rtos, int64_t thread_id,
		uint32_t reg_num, struct rtos_reg *rtos_reg)
{
	LOG_DEBUG("thread_id=%" PRId64 ", reg_num=%d", thread_id, reg_num);

	struct target *target = rtos->target;
	struct reg *reg = register_get_by_number(target->reg_cache, reg_num, true);
	if (!reg)
		return ERROR_FAIL;

	uint64_t reg_value = 0;
	if (riscv_get_register_on_hart(rtos->target, &reg_value, thread_id - 1,
				reg_num) != ERROR_OK)
		return ERROR_FAIL;

	buf_set_u64(rtos_reg->value, 0, 64, reg_value);
	rtos_reg->number = reg->number;
	rtos_reg->size = reg->size;
	return ERROR_OK;
}

static int riscv_get_thread_reg_list(struct rtos *rtos, int64_t thread_id,
		struct rtos_reg **reg_list, int *num_regs)
{
	LOG_DEBUG("Updating RISC-V register list for hart %d", (int)(thread_id - 1));

	/* We return just the GPRs here. */

	*num_regs = 33;
	int xlen = riscv_xlen_of_hart(rtos->target, thread_id - 1);

	*reg_list = calloc(*num_regs, sizeof(struct rtos_reg));
	for (int i = 0; i < *num_regs; ++i) {
		uint64_t reg_value;
		if (riscv_get_register_on_hart(rtos->target, &reg_value, thread_id - 1,
					i) != ERROR_OK)
			return JIM_ERR;

		(*reg_list)[i].number = i;
		(*reg_list)[i].size = xlen;
		buf_set_u64((*reg_list)[i].value, 0, 64, reg_value);
	}
	return JIM_OK;
}

static int riscv_set_reg(struct rtos *rtos, uint32_t reg_num,
		uint8_t *reg_value)
{
	struct target *target = rtos->target;
	struct reg *reg = register_get_by_number(target->reg_cache, reg_num, true);
	if (!reg)
		return ERROR_FAIL;

	int hartid = rtos->current_threadid - 1;
	uint64_t value = buf_get_u64(reg_value, 0, reg->size);

	return riscv_set_register_on_hart(target, hartid, reg_num, value);
}

static int riscv_get_symbol_list_to_lookup(symbol_table_elem_t *symbol_list[])
{
	*symbol_list = calloc(1, sizeof(symbol_table_elem_t));
	(*symbol_list)[0].symbol_name = NULL;
	(*symbol_list)[0].optional = false;
	return JIM_OK;
}

const struct rtos_type riscv_rtos = {
	.name = "riscv",
	.detect_rtos = riscv_detect_rtos,
	.create = riscv_create_rtos,
	.update_threads = riscv_update_threads,
	.get_thread_reg = riscv_get_thread_reg,
	.get_thread_reg_list = riscv_get_thread_reg_list,
	.get_symbol_list_to_lookup = riscv_get_symbol_list_to_lookup,
	.set_reg = riscv_set_reg,
};
