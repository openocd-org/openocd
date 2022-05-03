/* SPDX-License-Identifier: GPL-2.0-or-later */

#include <stdint.h>

#define FLASH_BSY	(1 << 0)
#define FLASH_PGERR	(1 << 2)
#define FLASH_WRPRTERR	(1 << 4)

void flash_write(volatile uint32_t *flash_sr,
		uint32_t hwords_count,
		uint16_t *buffer,
		uint16_t *target_addr) __attribute__((naked));

void flash_write(volatile uint32_t *flash_sr,
		uint32_t hwords_count,
		uint16_t *buffer,
		uint16_t *target_addr)
{
	do {
		*target_addr = *buffer++;

		register uint32_t sr;
		do {
			sr = *flash_sr;
		} while (sr & FLASH_BSY);

		if (sr & (FLASH_PGERR | FLASH_WRPRTERR))
			break;

		target_addr++;
	} while (--hwords_count);
	asm("ebreak");
}
