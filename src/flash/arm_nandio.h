#ifndef __ARM_NANDIO_H
#define __ARM_NANDIO_H

#include <flash/nand.h>
#include <helper/binarybuffer.h>

/**
 * The arm_nand_data struct is used for defining NAND I/O operations on an ARM
 * core.
 */
struct arm_nand_data {
	/** target is proxy for some ARM core */
	struct target		*target;

	/** copy_area holds write-to-NAND loop and data to write */
	struct working_area	*copy_area;

	/** chunk_size == page or ECC unit */
	unsigned		chunk_size;

	/** data == where to write the data */
	uint32_t		data;

	/* currently implicit:  data width == 8 bits (not 16) */
};

int arm_nandwrite(struct arm_nand_data *nand, uint8_t *data, int size);
int arm_nandread(struct arm_nand_data *nand, uint8_t *data, uint32_t size);

#endif /* __ARM_NANDIO_H */
