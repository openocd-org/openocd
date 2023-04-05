/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NOR_IMP_H
#define OPENOCD_FLASH_NOR_IMP_H

#include <stdbool.h>

/* this is an internal header */
#include "core.h"
#include "driver.h"
/* almost all drivers will need this file */
#include <target/target.h>

/**
 * Adds a new NOR bank to the global list of banks.
 * @param bank The bank that should be added.
 */
void flash_bank_add(struct flash_bank *bank);

/**
 * @return The first bank in the global list.
 */
struct flash_bank *flash_bank_list(void);

int flash_driver_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last);
int flash_driver_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last);
int flash_driver_write(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t count);
int flash_driver_read(struct flash_bank *bank,
		uint8_t *buffer, uint32_t offset, uint32_t count);
int flash_driver_verify(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t count);

/* write (optional verify) an image to flash memory of the given target */
int flash_write_unlock_verify(struct target *target, struct image *image,
		uint32_t *written, bool erase, bool unlock, bool write, bool verify);

#endif /* OPENOCD_FLASH_NOR_IMP_H */
