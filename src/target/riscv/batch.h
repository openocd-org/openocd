/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_BATCH_H
#define OPENOCD_TARGET_RISCV_BATCH_H

#include "target/target.h"
#include "jtag/jtag.h"
#include "riscv.h"

enum riscv_scan_type {
	RISCV_SCAN_TYPE_INVALID,
	RISCV_SCAN_TYPE_NOP,
	RISCV_SCAN_TYPE_READ,
	RISCV_SCAN_TYPE_WRITE,
};

/* These types are used to specify how many JTAG RTI cycles to add after a
 * scan.
 */
enum riscv_scan_delay_class {
	/* Delay needed for accessing debug module registers: */
	RISCV_DELAY_BASE,
	/* Delay for execution of an abstract command: */
	RISCV_DELAY_ABSTRACT_COMMAND,
	/* Delay for System Bus read operation: */
	RISCV_DELAY_SYSBUS_READ,
	/* Delay for System Bus write operation: */
	RISCV_DELAY_SYSBUS_WRITE
};

static inline const char *
riscv_scan_delay_class_name(enum riscv_scan_delay_class delay_class)
{
	switch (delay_class) {
	case RISCV_DELAY_BASE:
		return "DM access";
	case RISCV_DELAY_ABSTRACT_COMMAND:
		return "Abstract Command";
	case RISCV_DELAY_SYSBUS_READ:
		return "System Bus read";
	case RISCV_DELAY_SYSBUS_WRITE:
		return "System Bus write";
	}
	assert(0);
	return NULL;
}

/* The scan delay values are passed to "jtag_add_runtest()", which accepts an
 * "int".  Therefore, the passed value should be no greater than "INT_MAX".
 *
 * Since the resulting delay value can be a sum of two individual delays,
 * individual delays are limited to "INT_MAX / 2" to prevent overflow of the
 * final sum.
 */
#define RISCV_SCAN_DELAY_MAX (INT_MAX / 2)

struct riscv_scan_delays {
	unsigned int base_delay;
	unsigned int ac_delay;
	unsigned int sb_read_delay;
	unsigned int sb_write_delay;
};

static inline unsigned int
riscv_scan_get_delay(const struct riscv_scan_delays *delays,
		enum riscv_scan_delay_class delay_class)
{
	switch (delay_class) {
	case RISCV_DELAY_BASE:
		return delays->base_delay;
	case RISCV_DELAY_ABSTRACT_COMMAND:
		return delays->base_delay + delays->ac_delay;
	case RISCV_DELAY_SYSBUS_READ:
		return delays->base_delay + delays->sb_read_delay;
	case RISCV_DELAY_SYSBUS_WRITE:
		return delays->base_delay + delays->sb_write_delay;
	}
	assert(0);
	return 0;
}

static inline void riscv_scan_set_delay(struct riscv_scan_delays *delays,
		enum riscv_scan_delay_class delay_class, unsigned int delay)
{
	assert(delay <= RISCV_SCAN_DELAY_MAX);
	LOG_DEBUG("%s delay is set to %u.",
			riscv_scan_delay_class_name(delay_class), delay);
	switch (delay_class) {
	case RISCV_DELAY_BASE:
		delays->base_delay = delay;
		return;
	case RISCV_DELAY_ABSTRACT_COMMAND:
		delays->ac_delay = delay;
		return;
	case RISCV_DELAY_SYSBUS_READ:
		delays->sb_read_delay = delay;
		return;
	case RISCV_DELAY_SYSBUS_WRITE:
		delays->sb_write_delay = delay;
		return;
	}
	assert(0);
}

static inline int riscv_scan_increase_delay(struct riscv_scan_delays *delays,
		enum riscv_scan_delay_class delay_class)
{
	const unsigned int delay = riscv_scan_get_delay(delays, delay_class);
	const unsigned int delay_step = delay / 10 + 1;
	if (delay > RISCV_SCAN_DELAY_MAX - delay_step) {
		/* It's not clear if this issue actually occurs in real
		 * use-cases, so stick with a simple solution until the
		 * first bug report.
		 */
		LOG_ERROR("Delay for %s (%d) is not increased anymore (maximum was reached).",
				riscv_scan_delay_class_name(delay_class), delay);
		return ERROR_FAIL;
	}
	riscv_scan_set_delay(delays, delay_class, delay + delay_step);
	return ERROR_OK;
}

/* A batch of multiple JTAG scans, which are grouped together to avoid the
 * overhead of some JTAG adapters when sending single commands.  This is
 * designed to support block copies, as that's what we actually need to go
 * fast. */
struct riscv_batch {
	struct target *target;

	size_t allocated_scans;
	size_t used_scans;

	uint8_t *data_out;
	uint8_t *data_in;
	struct scan_field *fields;
	enum riscv_scan_delay_class *delay_classes;

	/* If in BSCAN mode, this field will be allocated (one per scan),
	   and utilized to tunnel all the scans in the batch.  If not in
	   BSCAN mode, this field is unallocated and stays NULL */
	riscv_bscan_tunneled_scan_context_t *bscan_ctxt;

	/* In JTAG we scan out the previous value's output when performing a
	 * scan.  This is a pain for users, so we just provide them the
	 * illusion of not having to do this by eliding all but the last NOP.
	 * */
	enum riscv_scan_type last_scan;

	/* The read keys. */
	size_t *read_keys;
	size_t read_keys_used;

	/* Flag indicating that the last run of the batch finished without an error
	 * from the underlying JTAG layer of OpenOCD - all scans were performed.
	 * However, RISC-V DMI "busy" condition could still have occurred.
	 */
	bool was_run;
	/* Number of RTI cycles used by the last scan on the last run.
	 * Only valid when `was_run` is set.
	 */
	unsigned int last_scan_delay;
};

/* Allocates (or frees) a new scan set.  "scans" is the maximum number of JTAG
 * scans that can be issued to this object. */
struct riscv_batch *riscv_batch_alloc(struct target *target, size_t scans);
void riscv_batch_free(struct riscv_batch *batch);

/* Checks to see if this batch is full. */
bool riscv_batch_full(struct riscv_batch *batch);

/* Executes this batch of JTAG DTM DMI scans, starting form "start" scan.
 *
 * If batch is run for the first time, it is expected that "start" is zero.
 * It is expected that the batch ends with a DMI NOP operation.
 *
 * "idle_counts" specifies the number of JTAG Run-Test-Idle cycles to add
 * after each scan depending on the delay class of the scan.
 *
 * If "resets_delays" is true, the algorithm will stop inserting idle cycles
 * (JTAG Run-Test-Idle) after "reset_delays_after" number of scans is
 * performed.  This is useful for stress-testing of RISC-V algorithms in
 * OpenOCD that are based on batches.
 */
int riscv_batch_run_from(struct riscv_batch *batch, size_t start_idx,
		const struct riscv_scan_delays *delays, bool resets_delays,
		size_t reset_delays_after);

/* Get the number of scans successfully executed form this batch. */
size_t riscv_batch_finished_scans(const struct riscv_batch *batch);

/* Adds a DM register write to this batch. */
void riscv_batch_add_dmi_write(struct riscv_batch *batch, uint32_t address, uint32_t data,
	bool read_back, enum riscv_scan_delay_class delay_class);

static inline void
riscv_batch_add_dm_write(struct riscv_batch *batch, uint32_t address, uint32_t data,
	bool read_back, enum riscv_scan_delay_class delay_type)
{
	return riscv_batch_add_dmi_write(batch,
			riscv_get_dmi_address(batch->target, address), data,
			read_back, delay_type);
}

/* DM register reads must be handled in two parts: the first one schedules a read and
 * provides a key, the second one actually obtains the result of the read -
 * status (op) and the actual data. */
size_t riscv_batch_add_dmi_read(struct riscv_batch *batch, uint32_t address,
		enum riscv_scan_delay_class delay_class);

static inline size_t
riscv_batch_add_dm_read(struct riscv_batch *batch, uint32_t address,
		enum riscv_scan_delay_class delay_type)
{
	return riscv_batch_add_dmi_read(batch,
			riscv_get_dmi_address(batch->target, address), delay_type);
}

uint32_t riscv_batch_get_dmi_read_op(const struct riscv_batch *batch, size_t key);
uint32_t riscv_batch_get_dmi_read_data(const struct riscv_batch *batch, size_t key);

/* Scans in a NOP. */
void riscv_batch_add_nop(struct riscv_batch *batch);

/* Returns the number of available scans. */
size_t riscv_batch_available_scans(struct riscv_batch *batch);

/* Return true iff the last scan in the batch returned DMI_OP_BUSY. */
bool riscv_batch_was_batch_busy(const struct riscv_batch *batch);

#endif /* OPENOCD_TARGET_RISCV_BATCH_H */
