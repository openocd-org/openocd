// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "batch.h"
#include "debug_defines.h"
#include "debug_reg_printer.h"
#include "riscv.h"
#include "field_helpers.h"

// TODO: DTM_DMI_MAX_ADDRESS_LENGTH should be reduced to 32 (per the debug spec)
#define DTM_DMI_MAX_ADDRESS_LENGTH	((1<<DTM_DTMCS_ABITS_LENGTH)-1)
#define DMI_SCAN_MAX_BIT_LENGTH (DTM_DMI_MAX_ADDRESS_LENGTH + DTM_DMI_DATA_LENGTH + DTM_DMI_OP_LENGTH)

#define DMI_SCAN_BUF_SIZE (DIV_ROUND_UP(DMI_SCAN_MAX_BIT_LENGTH, 8))

/* Reserve extra room in the batch (needed for the last NOP operation) */
#define BATCH_RESERVED_SCANS 1

static unsigned int get_dmi_scan_length(const struct target *target)
{
	const unsigned int abits = riscv_get_dmi_address_bits(target);
	assert(abits > 0);
	assert(abits <= DTM_DMI_MAX_ADDRESS_LENGTH);

	return abits + DTM_DMI_DATA_LENGTH + DTM_DMI_OP_LENGTH;
}

struct riscv_batch *riscv_batch_alloc(struct target *target, size_t scans)
{
	scans += BATCH_RESERVED_SCANS;
	struct riscv_batch *out = calloc(1, sizeof(*out));
	if (!out) {
		LOG_ERROR("Failed to allocate struct riscv_batch");
		return NULL;
	}

	out->target = target;
	out->allocated_scans = scans;
	out->last_scan = RISCV_SCAN_TYPE_INVALID;
	out->was_run = false;
	out->last_scan_delay = 0;

	out->data_out = NULL;
	out->data_in = NULL;
	out->fields = NULL;
	out->delay_classes = NULL;
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
	out->delay_classes = malloc(sizeof(*out->delay_classes) * scans);
	if (!out->delay_classes) {
		LOG_ERROR("Failed to allocate delay_classes in RISC-V batch.");
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
	free(batch->delay_classes);
	free(batch->bscan_ctxt);
	free(batch->read_keys);
	free(batch);
}

bool riscv_batch_full(struct riscv_batch *batch)
{
	return riscv_batch_available_scans(batch) == 0;
}

static bool riscv_batch_was_scan_busy(const struct riscv_batch *batch,
		size_t scan_idx)
{
	assert(batch->was_run);
	assert(scan_idx < batch->used_scans);
	const struct scan_field *field = batch->fields + scan_idx;
	assert(field->in_value);
	const uint64_t in = buf_get_u64(field->in_value, 0, field->num_bits);
	return get_field(in, DTM_DMI_OP) == DTM_DMI_OP_BUSY;
}

static void add_idle_before_batch(const struct riscv_batch *batch, size_t start_idx,
		const struct riscv_scan_delays *delays)
{
	if (!batch->was_run)
		return;
	/* Get the delay type of the scan that resulted in the busy response.
	 * Since DMI interactions always end with a NOP, if "start_idx" is zero
	 * the base delay value is used.
	 */
	const enum riscv_scan_delay_class delay_class = start_idx > 0
		? batch->delay_classes[start_idx - 1]
		: RISCV_DELAY_BASE;
	const unsigned int new_delay = riscv_scan_get_delay(delays, delay_class);
	if (new_delay <= batch->last_scan_delay)
		return;
	const unsigned int idle_change = new_delay - batch->last_scan_delay;
	LOG_TARGET_DEBUG(batch->target, "Adding %u idle cycles before the batch.",
			idle_change);
	jtag_add_runtest(idle_change, TAP_IDLE);
}

static unsigned int get_delay(const struct riscv_batch *batch, size_t scan_idx,
		const struct riscv_scan_delays *delays, bool resets_delays,
		size_t reset_delays_after)
{
	assert(batch);
	assert(scan_idx < batch->used_scans);
	const bool delays_were_reset = resets_delays
		&& (scan_idx >= reset_delays_after);
	const enum riscv_scan_delay_class delay_class =
		batch->delay_classes[scan_idx];
	const unsigned int delay = riscv_scan_get_delay(delays, delay_class);
	return delays_were_reset ? 0 : delay;
}

static unsigned int decode_dmi(const struct riscv_batch *batch, char *text,
		uint32_t address, uint32_t data)
{
	static const struct {
		uint32_t address;
		enum riscv_debug_reg_ordinal ordinal;
	} description[] = {
		{DM_DMCONTROL, DM_DMCONTROL_ORDINAL},
		{DM_DMSTATUS, DM_DMSTATUS_ORDINAL},
		{DM_ABSTRACTCS, DM_ABSTRACTCS_ORDINAL},
		{DM_COMMAND, DM_COMMAND_ORDINAL},
		{DM_SBCS, DM_SBCS_ORDINAL}
	};

	for (unsigned int i = 0; i < ARRAY_SIZE(description); i++) {
		if (riscv_get_dmi_address(batch->target, description[i].address)
				== address) {
			const riscv_debug_reg_ctx_t context = {
				.XLEN = { .value = 0, .is_set = false },
				.DXLEN = { .value = 0, .is_set = false },
				.abits = { .value = 0, .is_set = false },
			};
			return riscv_debug_reg_to_s(text, description[i].ordinal,
					context, data, RISCV_DEBUG_REG_HIDE_ALL_0);
		}
	}
	if (text)
		text[0] = '\0';
	return 0;
}

static void log_dmi_decoded(const struct riscv_batch *batch, bool write,
		uint32_t address, uint32_t data)
{
	const size_t size = decode_dmi(batch, /* text */ NULL, address, data) + 1;
	char * const decoded = malloc(size);
	if (!decoded) {
		LOG_ERROR("Not enough memory to allocate %zu bytes.", size);
		return;
	}
	decode_dmi(batch, decoded, address, data);
	LOG_DEBUG("%s: %s", write ? "write" : "read", decoded);
	free(decoded);
}

static void log_batch(const struct riscv_batch *batch, size_t start_idx,
		const struct riscv_scan_delays *delays, bool resets_delays,
		size_t reset_delays_after)
{
	if (debug_level < LOG_LVL_DEBUG)
		return;

	const unsigned int abits = riscv_get_dmi_address_bits(batch->target);

	/* Determine the "op" and "address" of the scan that preceded the first
	 * executed scan.
	 * FIXME: The code here assumes that there were no DMI operations between
	 * the last execution of the batch and the current one.
	 * Caching the info about the last executed DMI scan in "dm013_info_t"
	 * would be a more robust solution.
	 */
	bool last_scan_was_read = false;
	uint32_t last_scan_address = (uint32_t)(-1) /* to silence maybe-uninitialized */;
	if (start_idx > 0) {
		const struct scan_field * const field = &batch->fields[start_idx - 1];
		assert(field->out_value);
		last_scan_was_read = buf_get_u32(field->out_value, DTM_DMI_OP_OFFSET,
				DTM_DMI_OP_LENGTH) == DTM_DMI_OP_READ;
		last_scan_address = buf_get_u32(field->out_value,
				DTM_DMI_ADDRESS_OFFSET, abits);
	}

	/* Decode and log every executed scan */
	for (size_t i = start_idx; i < batch->used_scans; ++i) {
		static const char * const op_string[] = {"-", "r", "w", "?"};
		const unsigned int delay = get_delay(batch, i, delays, resets_delays,
				reset_delays_after);
		const struct scan_field * const field = &batch->fields[i];

		assert(field->out_value);
		const unsigned int out_op = buf_get_u32(field->out_value,
				DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
		const uint32_t out_data = buf_get_u32(field->out_value,
				DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);
		const uint32_t out_address = buf_get_u32(field->out_value,
				DTM_DMI_ADDRESS_OFFSET, abits);
		if (field->in_value) {
			static const char * const status_string[] = {
				"+", "?", "F", "b"
			};
			const unsigned int in_op = buf_get_u32(field->in_value,
					DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
			const uint32_t in_data = buf_get_u32(field->in_value,
					DTM_DMI_DATA_OFFSET, DTM_DMI_DATA_LENGTH);
			const uint32_t in_address = buf_get_u32(field->in_value,
					DTM_DMI_ADDRESS_OFFSET, abits);

			LOG_DEBUG("%db %s %08" PRIx32 " @%02" PRIx32
					" -> %s %08" PRIx32 " @%02" PRIx32 "; %ui",
					field->num_bits, op_string[out_op], out_data, out_address,
					status_string[in_op], in_data, in_address, delay);

			if (last_scan_was_read && in_op == DTM_DMI_OP_SUCCESS)
				log_dmi_decoded(batch, /*write*/ false,
						last_scan_address, in_data);
		} else {
			LOG_DEBUG("%db %s %08" PRIx32 " @%02" PRIx32 " -> ?; %ui",
					field->num_bits, op_string[out_op], out_data, out_address,
					delay);
		}

		if (out_op == DTM_DMI_OP_WRITE)
			log_dmi_decoded(batch, /*write*/ true, out_address,
					out_data);

		last_scan_was_read = out_op == DTM_DMI_OP_READ;
		last_scan_address = out_address;
	}
}

int riscv_batch_run_from(struct riscv_batch *batch, size_t start_idx,
		const struct riscv_scan_delays *delays, bool resets_delays,
		size_t reset_delays_after)
{
	assert(batch->used_scans);
	assert(start_idx < batch->used_scans);
	assert(batch->last_scan == RISCV_SCAN_TYPE_NOP);
	assert(!batch->was_run || riscv_batch_was_scan_busy(batch, start_idx));
	assert(start_idx == 0 || !riscv_batch_was_scan_busy(batch, start_idx - 1));

	if (batch->was_run)
		add_idle_before_batch(batch, start_idx, delays);

	LOG_TARGET_DEBUG(batch->target, "Running batch of scans [%zu, %zu)",
			start_idx, batch->used_scans);

	unsigned int delay = 0 /* to silence maybe-uninitialized */;
	for (size_t i = start_idx; i < batch->used_scans; ++i) {
		if (bscan_tunnel_ir_width != 0)
			riscv_add_bscan_tunneled_scan(batch->target->tap, batch->fields + i,
							batch->bscan_ctxt + i);
		else
			jtag_add_dr_scan(batch->target->tap, 1, batch->fields + i, TAP_IDLE);

		delay = get_delay(batch, i, delays, resets_delays,
				reset_delays_after);
		if (delay > 0)
			jtag_add_runtest(delay, TAP_IDLE);
	}

	keep_alive();

	if (jtag_execute_queue() != ERROR_OK) {
		LOG_TARGET_ERROR(batch->target, "Unable to execute JTAG queue");
		return ERROR_FAIL;
	}

	keep_alive();

	if (bscan_tunnel_ir_width != 0) {
		/* need to right-shift "in" by one bit, because of clock skew between BSCAN TAP and DM TAP */
		for (size_t i = start_idx; i < batch->used_scans; ++i) {
			if ((batch->fields + i)->in_value)
				buffer_shr((batch->fields + i)->in_value, DMI_SCAN_BUF_SIZE, 1);
		}
	}

	log_batch(batch, start_idx, delays, resets_delays, reset_delays_after);
	batch->was_run = true;
	batch->last_scan_delay = delay;
	return ERROR_OK;
}

void riscv_batch_add_dmi_write(struct riscv_batch *batch, uint32_t address, uint32_t data,
		bool read_back, enum riscv_scan_delay_class delay_class)
{
	// TODO: Check that the bit width of "address" is no more than dtmcs.abits,
	// otherwise return an error (during batch creation or when the batch is executed).

	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;

	field->num_bits = get_dmi_scan_length(batch->target);
	assert(field->num_bits <= DMI_SCAN_MAX_BIT_LENGTH);

	uint8_t *out_value = batch->data_out + batch->used_scans * DMI_SCAN_BUF_SIZE;
	uint8_t *in_value = batch->data_in + batch->used_scans * DMI_SCAN_BUF_SIZE;

	field->out_value = out_value;
	riscv_fill_dmi_write(batch->target, out_value, address, data);

	if (read_back) {
		field->in_value = in_value;
		riscv_fill_dm_nop(batch->target, in_value);
	} else {
		field->in_value = NULL;
	}

	batch->delay_classes[batch->used_scans] = delay_class;
	batch->last_scan = RISCV_SCAN_TYPE_WRITE;
	batch->used_scans++;
}

size_t riscv_batch_add_dmi_read(struct riscv_batch *batch, uint32_t address,
		enum riscv_scan_delay_class delay_class)
{
	// TODO: Check that the bit width of "address" is no more than dtmcs.abits,
	// otherwise return an error (during batch creation or when the batch is executed).

	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;

	field->num_bits = get_dmi_scan_length(batch->target);
	assert(field->num_bits <= DMI_SCAN_MAX_BIT_LENGTH);

	uint8_t *out_value = batch->data_out + batch->used_scans * DMI_SCAN_BUF_SIZE;
	uint8_t *in_value = batch->data_in + batch->used_scans * DMI_SCAN_BUF_SIZE;

	field->out_value = out_value;
	field->in_value = in_value;
	riscv_fill_dmi_read(batch->target, out_value, address);
	riscv_fill_dm_nop(batch->target, in_value);

	batch->delay_classes[batch->used_scans] = delay_class;
	batch->last_scan = RISCV_SCAN_TYPE_READ;
	batch->used_scans++;

	batch->read_keys[batch->read_keys_used] = batch->used_scans;
	return batch->read_keys_used++;
}

uint32_t riscv_batch_get_dmi_read_op(const struct riscv_batch *batch, size_t key)
{
	assert(key < batch->read_keys_used);
	size_t index = batch->read_keys[key];
	assert(index < batch->used_scans);
	uint8_t *base = batch->data_in + DMI_SCAN_BUF_SIZE * index;
	/* extract "op" field from the DMI read result */
	return buf_get_u32(base, DTM_DMI_OP_OFFSET, DTM_DMI_OP_LENGTH);
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

	field->num_bits = get_dmi_scan_length(batch->target);
	assert(field->num_bits <= DMI_SCAN_MAX_BIT_LENGTH);

	uint8_t *out_value = batch->data_out + batch->used_scans * DMI_SCAN_BUF_SIZE;
	uint8_t *in_value = batch->data_in + batch->used_scans * DMI_SCAN_BUF_SIZE;

	field->out_value = out_value;
	field->in_value = in_value;
	riscv_fill_dm_nop(batch->target, out_value);
	riscv_fill_dm_nop(batch->target, in_value);

	/* DMI NOP never triggers any debug module operation,
	 * so the shortest (base) delay can be used. */
	batch->delay_classes[batch->used_scans] = RISCV_DELAY_BASE;
	batch->last_scan = RISCV_SCAN_TYPE_NOP;
	batch->used_scans++;
}

size_t riscv_batch_available_scans(struct riscv_batch *batch)
{
	assert(batch->allocated_scans >= (batch->used_scans + BATCH_RESERVED_SCANS));
	return batch->allocated_scans - batch->used_scans - BATCH_RESERVED_SCANS;
}

bool riscv_batch_was_batch_busy(const struct riscv_batch *batch)
{
	assert(batch->was_run);
	assert(batch->used_scans);
	assert(batch->last_scan == RISCV_SCAN_TYPE_NOP);
	return riscv_batch_was_scan_busy(batch, batch->used_scans - 1);
}

size_t riscv_batch_finished_scans(const struct riscv_batch *batch)
{
	if (!riscv_batch_was_batch_busy(batch)) {
		/* Whole batch succeeded. */
		return batch->used_scans;
	}
	assert(batch->used_scans);
	size_t first_busy = 0;
	while (!riscv_batch_was_scan_busy(batch, first_busy))
		++first_busy;
	return first_busy;
}
