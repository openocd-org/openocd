/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef TARGET__RISCV__SCANS_H
#define TARGET__RISCV__SCANS_H

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
	RISCV_DELAY_SYSBUS_WRITE,
};

struct riscv_scan_delays {
	/* The purpose of these delays is to be passed to "jtag_add_runtest()",
	 * which accepts an "int".
	 * Therefore, they should be no greater then "INT_MAX".
	 */
	unsigned int base_delay;
	unsigned int ac_delay;
	unsigned int sb_read_delay;
	unsigned int sb_write_delay;
};

static inline unsigned int riscv_scan_get_delay(struct riscv_scan_delays delays,
		enum riscv_scan_delay_class delay_class)
{
	switch (delay_class) {
	case RISCV_DELAY_BASE:
		return delays.base_delay;
	case RISCV_DELAY_ABSTRACT_COMMAND:
		return delays.ac_delay;
	case RISCV_DELAY_SYSBUS_READ:
		return delays.sb_read_delay;
	case RISCV_DELAY_SYSBUS_WRITE:
		return delays.sb_write_delay;
	}
	return 0;
}

static inline void riscv_scan_set_delay(struct riscv_scan_delays *delays,
		enum riscv_scan_delay_class delay_class, unsigned int delay)
{
	assert(delay <= INT_MAX);
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
	unsigned int used_delay;
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
		struct riscv_scan_delays delays, bool resets_delays,
		size_t reset_delays_after);

/* Get the number of scans successfully executed form this batch. */
size_t riscv_batch_finished_scans(const struct riscv_batch *batch);

/* Adds a DM register write to this batch. */
void riscv_batch_add_dm_write(struct riscv_batch *batch, uint64_t address, uint32_t data,
	bool read_back, enum riscv_scan_delay_class delay_class);

/* DM register reads must be handled in two parts: the first one schedules a read and
 * provides a key, the second one actually obtains the result of the read -
 * status (op) and the actual data. */
size_t riscv_batch_add_dm_read(struct riscv_batch *batch, uint64_t address,
		enum riscv_scan_delay_class delay_class);
unsigned int riscv_batch_get_dmi_read_op(const struct riscv_batch *batch, size_t key);
uint32_t riscv_batch_get_dmi_read_data(const struct riscv_batch *batch, size_t key);

/* Scans in a NOP. */
void riscv_batch_add_nop(struct riscv_batch *batch);

/* Returns the number of available scans. */
size_t riscv_batch_available_scans(struct riscv_batch *batch);

/* Return true iff the last scan in the batch returned DMI_OP_BUSY. */
bool riscv_batch_was_batch_busy(const struct riscv_batch *batch);

/* TODO: The function is defined in `riscv-013.c`. This is done to reduce the
 * diff of the commit. The intention is to move the function definition to
 * a separate module (e.g. `riscv013-jtag-dtm.c/h`) in another commit. */
void riscv_log_dmi_scan(const struct target *target, int idle, const struct scan_field *field,
		bool discard_in);

#endif
