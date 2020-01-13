#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "batch.h"
#include "debug_defines.h"
#include "riscv.h"

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

static void dump_field(int idle, const struct scan_field *field);

struct riscv_batch *riscv_batch_alloc(struct target *target, size_t scans, size_t idle)
{
	scans += 4;
	struct riscv_batch *out = calloc(1, sizeof(*out));
	out->target = target;
	out->allocated_scans = scans;
	out->idle_count = idle;
	out->data_out = malloc(sizeof(*out->data_out) * (scans) * sizeof(uint64_t));
	out->data_in  = malloc(sizeof(*out->data_in)  * (scans) * sizeof(uint64_t));
	out->fields = malloc(sizeof(*out->fields) * (scans));
	if (bscan_tunnel_ir_width != 0)
		out->bscan_ctxt = malloc(sizeof(*out->bscan_ctxt) * (scans));
	out->last_scan = RISCV_SCAN_TYPE_INVALID;
	out->read_keys = malloc(sizeof(*out->read_keys) * (scans));
	return out;
}

void riscv_batch_free(struct riscv_batch *batch)
{
	free(batch->data_in);
	free(batch->data_out);
	free(batch->fields);
	if (batch->bscan_ctxt)
		free(batch->bscan_ctxt);
	free(batch->read_keys);
	free(batch);
}

bool riscv_batch_full(struct riscv_batch *batch)
{
	return batch->used_scans > (batch->allocated_scans - 4);
}

int riscv_batch_run(struct riscv_batch *batch)
{
	if (batch->used_scans == 0) {
		LOG_DEBUG("Ignoring empty batch.");
		return ERROR_OK;
	}

	keep_alive();

	riscv_batch_add_nop(batch);

	for (size_t i = 0; i < batch->used_scans; ++i) {
		if (bscan_tunnel_ir_width != 0)
			riscv_add_bscan_tunneled_scan(batch->target, batch->fields+i, batch->bscan_ctxt+i);
		else
			jtag_add_dr_scan(batch->target->tap, 1, batch->fields + i, TAP_IDLE);

		if (batch->idle_count > 0)
			jtag_add_runtest(batch->idle_count, TAP_IDLE);
	}

	if (jtag_execute_queue() != ERROR_OK) {
		LOG_ERROR("Unable to execute JTAG queue");
		return ERROR_FAIL;
	}

	if (bscan_tunnel_ir_width != 0) {
		/* need to right-shift "in" by one bit, because of clock skew between BSCAN TAP and DM TAP */
		for (size_t i = 0; i < batch->used_scans; ++i)
			buffer_shr((batch->fields + i)->in_value, sizeof(uint64_t), 1);
	}

	for (size_t i = 0; i < batch->used_scans; ++i)
		dump_field(batch->idle_count, batch->fields + i);

	return ERROR_OK;
}

void riscv_batch_add_dmi_write(struct riscv_batch *batch, unsigned address, uint64_t data)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	field->out_value = (void *)(batch->data_out + batch->used_scans * sizeof(uint64_t));
	field->in_value  = (void *)(batch->data_in  + batch->used_scans * sizeof(uint64_t));
	riscv_fill_dmi_write_u64(batch->target, (char *)field->out_value, address, data);
	riscv_fill_dmi_nop_u64(batch->target, (char *)field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_WRITE;
	batch->used_scans++;
}

size_t riscv_batch_add_dmi_read(struct riscv_batch *batch, unsigned address)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	field->out_value = (void *)(batch->data_out + batch->used_scans * sizeof(uint64_t));
	field->in_value  = (void *)(batch->data_in  + batch->used_scans * sizeof(uint64_t));
	riscv_fill_dmi_read_u64(batch->target, (char *)field->out_value, address);
	riscv_fill_dmi_nop_u64(batch->target, (char *)field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_READ;
	batch->used_scans++;

	batch->read_keys[batch->read_keys_used] = batch->used_scans;
	return batch->read_keys_used++;
}

uint64_t riscv_batch_get_dmi_read(struct riscv_batch *batch, size_t key)
{
	assert(key < batch->read_keys_used);
	size_t index = batch->read_keys[key];
	assert(index <= batch->used_scans);
	uint8_t *base = batch->data_in + 8 * index;
	return base[0] |
		((uint64_t) base[1]) << 8 |
		((uint64_t) base[2]) << 16 |
		((uint64_t) base[3]) << 24 |
		((uint64_t) base[4]) << 32 |
		((uint64_t) base[5]) << 40 |
		((uint64_t) base[6]) << 48 |
		((uint64_t) base[7]) << 56;
}

void riscv_batch_add_nop(struct riscv_batch *batch)
{
	assert(batch->used_scans < batch->allocated_scans);
	struct scan_field *field = batch->fields + batch->used_scans;
	field->num_bits = riscv_dmi_write_u64_bits(batch->target);
	field->out_value = (void *)(batch->data_out + batch->used_scans * sizeof(uint64_t));
	field->in_value  = (void *)(batch->data_in  + batch->used_scans * sizeof(uint64_t));
	riscv_fill_dmi_nop_u64(batch->target, (char *)field->out_value);
	riscv_fill_dmi_nop_u64(batch->target, (char *)field->in_value);
	batch->last_scan = RISCV_SCAN_TYPE_NOP;
	batch->used_scans++;
}

void dump_field(int idle, const struct scan_field *field)
{
	static const char * const op_string[] = {"-", "r", "w", "?"};
	static const char * const status_string[] = {"+", "?", "F", "b"};

	if (debug_level < LOG_LVL_DEBUG)
		return;

	assert(field->out_value != NULL);
	uint64_t out = buf_get_u64(field->out_value, 0, field->num_bits);
	unsigned int out_op = get_field(out, DTM_DMI_OP);
	unsigned int out_data = get_field(out, DTM_DMI_DATA);
	unsigned int out_address = out >> DTM_DMI_ADDRESS_OFFSET;

	if (field->in_value) {
		uint64_t in = buf_get_u64(field->in_value, 0, field->num_bits);
		unsigned int in_op = get_field(in, DTM_DMI_OP);
		unsigned int in_data = get_field(in, DTM_DMI_DATA);
		unsigned int in_address = in >> DTM_DMI_ADDRESS_OFFSET;

		log_printf_lf(LOG_LVL_DEBUG,
				__FILE__, __LINE__, __PRETTY_FUNCTION__,
				"%db %s %08x @%02x -> %s %08x @%02x; %di",
				field->num_bits, op_string[out_op], out_data, out_address,
				status_string[in_op], in_data, in_address, idle);
	} else {
		log_printf_lf(LOG_LVL_DEBUG,
				__FILE__, __LINE__, __PRETTY_FUNCTION__, "%db %s %08x @%02x -> ?; %di",
				field->num_bits, op_string[out_op], out_data, out_address, idle);
	}
}

size_t riscv_batch_available_scans(struct riscv_batch *batch)
{
	return batch->allocated_scans - batch->used_scans - 4;
}
