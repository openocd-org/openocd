/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Copyright (C) 2009 by David Brownell
 */
#ifndef OPENOCD_FLASH_NAND_ARM_IO_H
#define OPENOCD_FLASH_NAND_ARM_IO_H

/**
 * Available operational states the arm_nand_data struct can be in.
 */
enum arm_nand_op {
	ARM_NAND_NONE,	/**< No operation performed. */
	ARM_NAND_READ,	/**< Read operation performed. */
	ARM_NAND_WRITE,	/**< Write operation performed. */
};

/**
 * The arm_nand_data struct is used for defining NAND I/O operations on an ARM
 * core.
 */
struct arm_nand_data {
	/** Target is proxy for some ARM core. */
	struct target *target;

	/** The copy area holds code loop and data for I/O operations. */
	struct working_area *copy_area;

	/** The chunk size is the page size or ECC chunk. */
	unsigned chunk_size;

	/** Where data is read from or written to. */
	uint32_t data;

	/** Last operation executed using this struct. */
	enum arm_nand_op op;

	/* currently implicit:  data width == 8 bits (not 16) */
};

int arm_nandwrite(struct arm_nand_data *nand, uint8_t *data, int size);
int arm_nandread(struct arm_nand_data *nand, uint8_t *data, uint32_t size);

#endif /* OPENOCD_FLASH_NAND_ARM_IO_H */
