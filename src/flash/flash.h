/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef FLASH_H
#define FLASH_H

#include <flash/common.h>

struct image;

#define FLASH_MAX_ERROR_STR	(128)

/**
 * Describes the geometry and status of a single flash sector
 * within a flash bank.  A single bank typically consists of multiple
 * sectors, each of which can be erased and protected independently.
 */
struct flash_sector
{
	/// Bus offset from start of the flash chip (in bytes).
	uint32_t offset;
	/// Number of bytes in this flash sector.
	uint32_t size;
	/**
	 * Indication of erasure status: 0 = not erased, 1 = erased,
	 * other = unknown.  Set by @c flash_driver_s::erase_check.
	 */
	int is_erased;
	/**
	 * Indication of protection status: 0 = unprotected/unlocked,
	 * 1 = protected/locked, other = unknown.  Set by
	 * @c flash_driver_s::protect_check.
	 */
	int is_protected;
};

struct flash_bank;

#define __FLASH_BANK_COMMAND(name) \
		COMMAND_HELPER(name, struct flash_bank *bank)

/**
 * @brief Provides the implementation-independent structure that defines
 * all of the callbacks required by OpenOCD flash drivers.
 *
 * Driver authors must implement the routines defined here, providing an
 * instance with the fields filled out.  After that, the instance must
 * be registered in flash.c, so it can be used by the driver lookup system.
 *
 * Specifically, the user can issue the command: @par
 * @code
 * flash bank DRIVERNAME ...parameters...
 * @endcode
 *
 * OpenOCD will search for the driver with a @c flash_driver_s::name
 * that matches @c DRIVERNAME.
 *
 * The flash subsystem calls some of the other drivers routines a using
 * corresponding static <code>flash_driver_<i>callback</i>()</code>
 * routine in flash.c.
 */
struct flash_driver
{
	/**
	 * Gives a human-readable name of this flash driver,
	 * This field is used to select and initialize the driver.
	 */
	char *name;

	/**
	 * An array of driver-specific commands to register.  When called
	 * during the "flash bank" command, the driver can register addition
	 * commands to support new flash chip functions.
	 */
	const struct command_registration *commands;

	/**
	 * Finish the "flash bank" command for @a bank.  The
	 * @a bank parameter will have been filled in by the core flash
	 * layer when this routine is called, and the driver can store
	 * additional information in its struct flash_bank::driver_priv field.
	 *
	 * The CMD_ARGV are: @par
	 * @code
	 * CMD_ARGV[0] = bank
	 * CMD_ARGV[1] = drivername {name above}
	 * CMD_ARGV[2] = baseaddress
	 * CMD_ARGV[3] = lengthbytes
	 * CMD_ARGV[4] = chip_width_in bytes
	 * CMD_ARGV[5] = bus_width_bytes
	 * CMD_ARGV[6] = driver-specific parameters
	 * @endcode
	 *
	 * For example, CMD_ARGV[4] = 16 bit flash, CMD_ARGV[5] = 32bit bus.
	 *
	 * If extra arguments are provided (@a CMD_ARGC > 6), they will
	 * start in @a CMD_ARGV[6].  These can be used to implement
	 * driver-specific extensions.
	 *
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	__FLASH_BANK_COMMAND((*flash_bank_command));

	/**
	 * Bank/sector erase routine (target-specific).  When
	 * called, the flash driver should erase the specified sectors
	 * using whatever means are at its disposal.
	 *
	 * @param bank The bank of flash to be erased.
	 * @param first The number of the first sector to erase, typically 0.
	 * @param last The number of the last sector to erase, typically N-1.
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*erase)(struct flash_bank *bank, int first, int last);

	/**
	 * Bank/sector protection routine (target-specific).
	 * When called, the driver should disable 'flash write' bits (or
	 * enable 'erase protection' bits) for the given @a bank and @a
	 * sectors.
	 *
	 * @param bank The bank to protect or unprotect.
	 * @param set If non-zero, enable protection; if 0, disable it.
	 * @param first The first sector to (un)protect, typicaly 0.
	 * @param last The last sector to (un)project, typically N-1.
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*protect)(struct flash_bank *bank, int set, int first, int last);

	/**
	 * Program data into the flash.  Note CPU address will be
	 * "bank->base + offset", while the physical address is
	 * dependent upon current target MMU mappings.
	 *
	 * @param bank The bank to program
	 * @param buffer The data bytes to write.
	 * @param offset The offset into the chip to program.
	 * @param count The number of bytes to write.
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*write)(struct flash_bank *bank,
			uint8_t *buffer, uint32_t offset, uint32_t count);

	/**
	 * Probe to determine what kind of flash is present.
	 * This is invoked by the "probe" script command.
	 *
	 * @param bank The bank to probe
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*probe)(struct flash_bank *bank);

	/**
	 * Check the erasure status of a flash bank.
	 * When called, the driver routine must perform the required
	 * checks and then set the @c flash_sector_s::is_erased field
	 * for each of the flash banks's sectors.
	 *
	 * @param bank The bank to check
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*erase_check)(struct flash_bank *bank);

	/**
	 * Determine if the specific bank is "protected" or not.
	 * When called, the driver routine must must perform the
	 * required protection check(s) and then set the @c
	 * flash_sector_s::is_protected field for each of the flash
	 * bank's sectors.
	 *
	 * @param bank - the bank to check
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*protect_check)(struct flash_bank *bank);

	/**
	 * Display human-readable information about the flash
	 * bank into the given buffer.  Drivers must be careful to avoid
	 * overflowing the buffer.
	 *
	 * @param bank - the bank to get info about
	 * @param char - where to put the text for the human to read
	 * @param buf_size - the size of the human buffer.
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*info)(struct flash_bank *bank, char *buf, int buf_size);

	/**
	 * A more gentle flavor of filash_driver_s::probe, performing
	 * setup with less noise.  Generally, driver routines should test
	 * to seee if the bank has already been probed; if it has, the
	 * driver probably should not perform its probe a second time.
	 *
	 * This callback is often called from the inside of other
	 * routines (e.g. GDB flash downloads) to autoprobe the flash as
	 * it is programing the flash.
	 *
	 * @param bank - the bank to probe
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*auto_probe)(struct flash_bank *bank);
};

#define FLASH_BANK_COMMAND_HANDLER(name) static __FLASH_BANK_COMMAND(name)

/**
 * Provides details of a flash bank, available either on-chip or through
 * a major interface.
 *
 * This structure will be passed as a parameter to the callbacks in the
 * flash_driver_s structure, some of which may modify the contents of
 * this structure of the area of flash that it defines.  Driver writers
 * may use the @c driver_priv member to store additional data on a
 * per-bank basis, if required.
 */
struct flash_bank
{
	char *name;

	struct target *target; /**< Target to which this bank belongs. */

	struct flash_driver *driver; /**< Driver for this bank. */
	void *driver_priv; /**< Private driver storage pointer */

	int bank_number; /**< The 'bank' (or chip number) of this instance. */
	uint32_t base; /**< The base address of this bank */
	uint32_t size; /**< The size of this chip bank, in bytes */

	int chip_width; /**< Width of the chip in bytes (1,2,4 bytes) */
	int bus_width; /**< Maximum bus width, in bytes (1,2,4 bytes) */

	/**
	 * The number of sectors on this chip.  This value will
	 * be set intially to 0, and the flash driver must set this to
	 * some non-zero value during "probe()" or "auto_probe()".
	 */
	int num_sectors;
	/// Array of sectors, allocated and initilized by the flash driver
	struct flash_sector *sectors;

	struct flash_bank *next; /**< The next flash bank on this chip */
};

/// Registers the 'flash' subsystem commands
int flash_register_commands(struct command_context *cmd_ctx);
/// Initializes the 'flash' subsystem drivers
int flash_init_drivers(struct command_context *cmd_ctx);

/**
 * Erases @a length bytes in the @a target flash, starting at @a addr.
 * @returns ERROR_OK if successful; otherwise, an error code.
 */
int flash_erase_address_range(struct target *target,
		uint32_t addr, uint32_t length);
/**
 * Writes @a image into the @a target flash.  The @a written parameter
 * will contain the
 * @param target The target with the flash to be programmed.
 * @param image The image that will be programmed to flash.
 * @param written On return, contains the number of bytes written.
 * @param erase If non-zero, indicates the flash driver should first
 * erase the corresponding banks or sectors before programming.
 * @returns ERROR_OK if successful; otherwise, an error code.
 */
int flash_write(struct target *target,
		struct image *image, uint32_t *written, int erase);
/**
 * Forces targets to re-examine their erase/protection state.
 * This routine must be called when the system may modify the status.
 */
void flash_set_dirty(void);
/// @returns The number of flash banks currently defined.
int flash_get_bank_count(void);
/**
 * Provides default erased-bank check handling. Checks to see if
 * the flash driver knows they are erased; if things look uncertain,
 * this routine will call default_flash_mem_blank_check() to confirm.
 * @returns ERROR_OK if successful; otherwise, an error code.
 */
int default_flash_blank_check(struct flash_bank *bank);
/**
 * Provides a default blank flash memory check.  Ensures the contents
 * of the given bank have truly been erased.
 * @param bank The flash bank.
 * @returns ERROR_OK if successful; otherwise, an error code.
 */
int default_flash_mem_blank_check(struct flash_bank *bank);

/**
 * Returns the flash bank specified by @a name, which matches the
 * driver name and a suffix (option) specify the driver-specific
 * bank number. The suffix consists of the '.' and the driver-specific
 * bank number: when two str9x banks are defined, then 'str9x.1' refers
 * to the second.
 */
struct flash_bank *get_flash_bank_by_name(const char *name);
/**
 * Returns a flash bank by the specified flash_bank_s bank_number, @a num.
 * @param num The flash bank number.
 * @returns A struct flash_bank for flash bank @a num, or NULL
 */
struct flash_bank *get_flash_bank_by_num(int num);
/**
 * Retreives @a bank from a command argument, reporting errors parsing
 * the bank identifier or retreiving the specified bank.  The bank
 * may be identified by its bank number or by @c name.instance, where
 * @a instance is driver-specific.
 * @param name_index The index to the string in args containing the
 * bank identifier.
 * @param bank On output, contians a pointer to the bank or NULL.
 * @returns ERROR_OK on success, or an error indicating the problem.
 */
COMMAND_HELPER(flash_command_get_bank, unsigned name_index,
		struct flash_bank **bank);
/**
 * Returns the flash bank like get_flash_bank_by_num(), without probing.
 * @param num The flash bank number.
 * @returns A struct flash_bank for flash bank @a num, or NULL.
 */
struct flash_bank *get_flash_bank_by_num_noprobe(int num);
/**
 * Returns the flash bank located at a specified address.
 * @param target The target, presumed to contain one or more banks.
 * @param addr An address that is within the range of the bank.
 * @returns The struct flash_bank located at @a addr, or NULL.
 */
struct flash_bank *get_flash_bank_by_addr(struct target *target, uint32_t addr);

#endif /* FLASH_H */
