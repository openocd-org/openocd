// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "batch.h"
#include "debug_defines.h"
#include "riscv.h"
#include "field_helpers.h"

#define DTM_DMI_MAX_ADDRESS_LENGTH	((1<<DTM_DTMCS_ABITS_LENGTH)-1)
#define DMI_SCAN_MAX_BIT_LENGTH (DTM_DMI_MAX_ADDRESS_LENGTH + DTM_DMI_DATA_LENGTH + DTM_DMI_OP_LENGTH)
#define DMI_SCAN_BUF_SIZE (DIV_ROUND_UP(DMI_SCAN_MAX_BIT_LENGTH, 8))

/* Reserve extra room in the batch (needed for the last NOP operation) */
#define BATCH_RESERVED_SCANS 1

struct riscv_batch *riscv_batch_alloc(struct target *target, size_t scans, size_t idle)
{
	scans += BATCH_RESERVED_SCANS;
	struct riscv_batch *out = calloc(1, sizeof(*out));
	if (!out) {
		LOG_ERROR("Failed to allocate struct riscv_batch");
		return NULL;
	}

	out->target = target;
	out->allocated_scans = scans;
	out->idle_count = idle;
	out->last_scan = RISCV_SCAN_TYPE_INVALID;

	out->data_out = NULL;
	out->data_in = NULL;
	out->fields = NULL;
	out->bscan_ctxt = NULL;
	out->read_keys = NULL;

	/* FIXME: There is potential for memory usage reduction. We could allocate
	 * smaller buffers than DMI_SCAN_BUF_SIZE (that is, buffers that correspond to
	 * the real DR scan length on the given target) */
	out->data_out = malloc(sizeof(*out->data_out) * scans * DMI_SCAN_BUF_SIZE);
	if (!out->data_out) {
		LOG_ERROR("Failed to allocate data_out in RISC-V batch.");
		goto alloc_error;
	};
	out->data_in = malloc(sizeof(*out->data_in) * scans * DMI_SCAN_BUF_SIZE);
	if (!out->data_in) {
		LOG_ERROR("Failed to allocate data_in in RISC-V batch.");
		goto alloc_error;
	}
	out->fields = malloc(sizeof(*out->fields) * scans);
	if (!out->fields) {
		LOG_ERROR("Failed to allocate fields in RISC-V batch.");
		goto alloc_error;
	}
	if (bscan_tunnel_ir_width != 0) {
		out->bscan_ctxt = malloc(sizeof(*out->bscan_ctxt) * scans);
		if (!out->bscan_ctxt) {
			LOG_ERROR("Failed to allocate bscan_ctxt in RISC-V batch.");
			goto alloc_error;
		}
	}
	out->read_keys = malloc(sizeof(*out->read_keys) * scans);
	if (!out->read_keys) {
		LOG_ERROR("Failed to allocate read_keys in RISC-V batch.");
		goto alloc_error;
	}

	return out;

alloc_error:
	riscv_batch_free(out);
	return NULL;
}

void riscv_batch_free(struct riscv_batch *batch)
{
	free(batch->data_in);
	free(batch->data_out);
	free(batch->fields);
	free(batch->bscan_ctxt);
	free(batch->read_keys);
	free(batch);
}

bool riscv_batch_full(struct riscv_batch *batch)
{
	return riscv_batch_available_scans(batch) == 0;
}

int riscv_batch_run(struct riscv_batch *batch, bool resets_delays,
		size_t reset_delays_after)
{
	if (batch->used_scans == 0) {
		LOG_TARGET_DEBUG(batch->target, "Ignoring empty batch.");
		return ERROR_OK;
	}

	riscv_batch_add_nop(batch);

	for (size_t i = 0; i < batch->used_scans; ++i) {
		if (bscan_tunnel_ir_width != 0)
			riscv_add_bscan_tunneled_scan(batch->target, batch->fields + i, batch->bscan_ctxt + i);
		else
			jtag_add_dr_scan(batch->target->tap, 1, batch->fields + i, TAP_IDLE);

		const bool delays_were_reset = resets_delays
			&& (i >= reset_delays_after);
		if (batch->idle_count > 0 && !delays_were_reset)
			jtag_add_runtest(batch->idle_count, TAP_IDLE);
	}

	keep_alive();

	if (jtag_execute_queue() != ERROR_OK) {
		LOG_TARGET_ERROR(batch->target, "Unable to execute JTAG queue");
		return ERROR_FAIL;
	}

	keep_alive();

	if (bscan_tunnel_ir_width != 0) {
		/* need to right-shift "in" by one bit, because of clock skew between BSCAN TAP and DM TAP */
		for (size_t i = 0; i < batch->used_scans; ++i) {
			if ((batch->fields + i)->in_value)
				buffer_shr((batch->fields + i)->in_value, DMI_SCAN_BUF_SIZE, 1);
		}
	}

	for (size_t i = 0; i < batch->used_scans; ++i)
		riscv_decode_dmi_scan(batch->target, batch->idle_count, batch->fields + i,
				/*discard_in*/ false);

	return ERROR_OK;
}

void riscv_batch_add_dm_write(struct riscv_batch *batch, uint64_t address, uint32_t data,
	bool read_back)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_get_dmi_scan_length(batch->target);
	field->out_value = (void *)(batch->data_out + batch->used_scans * DMI_SCAN_BUF_SIZE);
	riscv_fill_dm_write(batch->target, (char *)field->out_value, address, data);
	if (read_back) {
		field->in_value = (void *)(batch->data_in + batch->used_scans * DMI_SCAN_BUF_SIZE);
		riscv_fill_dm_nop(batch->target, (char *)field->in_value);
	} else {
		field->in_value = NULL;
	}
	batch->last_scan = RISCV_SCAN_TYPE_WRITE;
	batch->used_scans++;
}

size_t riscv_batch_add_dm_read(struct riscv_batch *batch, uint64_t address)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_get_dmi_scan_length(batch->target);
	field->out_value = (void *)(batch->data_out + batch->used_scans * DMI_SCAN_BUF_SIZE);
	field->in_value  = (void *)(batch->data_in  + batch->used_scans * DMI_SCAN_BUF_SIZE);
	riscv_fill_dm_read(batch->target, (char *)field->out_value, address);
	riscv_fill_dm_nop(batch->target, (char *)field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_READ;
	batch->used_scans++;

	batch->read_keys[batch->read_keys_used] = batch->used_scans;
	return batch->read_keys_used++;
}

unsigned int riscv_batch_get_dmi_read_op(const struct riscv_batch *batch, size_t key)
{
	assert(key < batch->read_keys_used);
	size_t index = batch->read_keys[key];
	assert(index < batch->used_scans);
	uint8_t *base = batch->data_in + DMI_SCAN_BUF_SIZE * index;
	/* extract "op" field from the DMI read result */
	return (unsigned int)buf_get_u32(base, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
}

uint32_t riscv_batch_get_dmi_read_data(const struct riscv_batch *batch, size_t key)
{
	assert(key < batch->read_keys_used);
	size_t index = batch->read_keys[key];
	assert(index < batch->used_scans);
	uint8_t *base = batch->data_in + DMI_SCAN_BUF_SIZE * index;
	/* extract "data" field from the DMI read result */
	return buf_get_u32(base, DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);
}

void riscv_batch_add_nop(struct riscv_batch *batch)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_get_dmi_scan_length(batch->target);
	field->out_value = (void *)(batch->data_out + batch->used_scans * DMI_SCAN_BUF_SIZE);
	field->in_value  = (void *)(batch->data_in  + batch->used_scans * DMI_SCAN_BUF_SIZE);
	riscv_fill_dm_nop(batch->target, (char *)field->out_value);
	riscv_fill_dm_nop(batch->target, (char *)field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_NOP;
	batch->used_scans++;
}

size_t riscv_batch_available_scans(struct riscv_batch *batch)
{
	assert(batch->allocated_scans >= (batch->used_scans + BATCH_RESERVED_SCANS));
	return batch->allocated_scans - batch->used_scans - BATCH_RESERVED_SCANS;
}

bool riscv_batch_dmi_busy_encountered(const struct riscv_batch *batch)
{
	if (batch->used_scans == 0)
		/* Empty batch */
		return false;

	assert(batch->last_scan == RISCV_SCAN_TYPE_NOP);
	const struct scan_field *field = batch->fields + batch->used_scans - 1;
	const uint64_t in = buf_get_u64(field->in_value, 0, field->num_bits);
	return get_field(in, DTM_DMI_OP) == DTM_DMI_OP_BUSY;
}
