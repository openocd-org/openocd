/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_FIELD_HELPERS_H
#define OPENOCD_TARGET_RISCV_FIELD_HELPERS_H

#include <stdint.h>
#include <assert.h>

static inline uint64_t get_field(uint64_t reg, uint64_t mask)
{
	return (reg & mask) / (mask & ~(mask << 1));
}

static inline uint32_t get_field32(uint64_t reg, uint64_t mask)
{
	uint64_t value = get_field(reg, mask);
	assert(value <= UINT32_MAX);
	return value;
}

static inline uint64_t set_field(uint64_t reg, uint64_t mask, uint64_t val)
{
	/* Clear current value from field. */
	reg &= ~mask;
	uint64_t low_field_bit = mask & ~(mask << 1);
	/* Assert if the value doesn't fit in the field. */
	assert(((val * low_field_bit) & ~mask) == 0);
	reg |= (val * low_field_bit) & mask;
	return reg;
}

static inline uint32_t set_field32(uint32_t reg, uint32_t mask, uint32_t val)
{
	return (uint32_t)set_field(reg, mask, val);
}

static inline uint64_t field_value(uint64_t mask, uint64_t val)
{
	return set_field(0, mask, val);
}

static inline uint32_t field_value32(uint32_t mask, uint32_t val)
{
	return set_field32(0, mask, val);
}

#endif /* OPENOCD_TARGET_RISCV_FIELD_HELPERS_H */
