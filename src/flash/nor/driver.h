/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2007,2008 Øyvind Harboe <oyvind.harboe@zylin.com>       *
 *   Copyright (C) 2008 by Spencer Oliver <spen@spen-soft.co.uk>           *
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 *   Copyright (C) 2010 by Antonio Borneo <borneo.antonio@gmail.com>       *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NOR_DRIVER_H
#define OPENOCD_FLASH_NOR_DRIVER_H

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
 * OpenOCD will search for the driver with a @c flash_driver::name
 * that matches @c DRIVERNAME.
 *
 * The flash subsystem calls some of the other drivers routines a using
 * corresponding static <code>flash_driver_<i>callback</i>()</code>
 * routine in flash.c.
 */
struct flash_driver {
	/**
	 * Gives a human-readable name of this flash driver,
	 * This field is used to select and initialize the driver.
	 */
	const char *name;

	/**
	 * Gives a human-readable description of arguments.
	 */
	const char *usage;

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
	 * CMD_ARGV[5] = bus_width_in_bytes
	 * CMD_ARGV[6] = driver-specific parameters
	 * @endcode
	 *
	 * For example, CMD_ARGV[4] = 2 (for 16 bit flash),
	 *	CMD_ARGV[5] = 4 (for 32 bit bus).
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
	int (*erase)(struct flash_bank *bank, unsigned int first,
		unsigned int last);

	/**
	 * Bank/sector protection routine (target-specific).
	 *
	 * If protection is not implemented, set method to NULL
	 *
	 * When called, the driver should enable/disable protection
	 * for MINIMUM the range covered by first..last sectors
	 * inclusive. Some chips have alignment requirements will
	 * cause the actual range to be protected / unprotected to
	 * be larger than the first..last range.
	 *
	 * @param bank The bank to protect or unprotect.
	 * @param set If non-zero, enable protection; if 0, disable it.
	 * @param first The first sector to (un)protect, typically 0.
	 * @param last The last sector to (un)project, typically N-1.
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*protect)(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last);

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
			const uint8_t *buffer, uint32_t offset, uint32_t count);

	/**
	 * Read data from the flash. Note CPU address will be
	 * "bank->base + offset", while the physical address is
	 * dependent upon current target MMU mappings.
	 *
	 * @param bank The bank to read.
	 * @param buffer The data bytes read.
	 * @param offset The offset into the chip to read.
	 * @param count The number of bytes to read.
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	 int (*read)(struct flash_bank *bank,
			uint8_t *buffer, uint32_t offset, uint32_t count);

	/**
	 * Verify data in flash.  Note CPU address will be
	 * "bank->base + offset", while the physical address is
	 * dependent upon current target MMU mappings.
	 *
	 * @param bank The bank to verify
	 * @param buffer The data bytes to verify against.
	 * @param offset The offset into the chip to verify.
	 * @param count The number of bytes to verify.
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*verify)(struct flash_bank *bank,
			const uint8_t *buffer, uint32_t offset, uint32_t count);

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
	 * checks and then set the @c flash_sector::is_erased field
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
	 * flash_sector::is_protected field for each of the flash
	 * bank's sectors.
	 *
	 * If protection is not implemented, set method to NULL
	 *
	 * @param bank - the bank to check
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*protect_check)(struct flash_bank *bank);

	/**
	 * Display human-readable information about the flash
	 * bank.
	 *
	 * @param bank - the bank to get info about
	 * @param cmd - command invocation instance for which to generate
	 *              the textual output
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*info)(struct flash_bank *bank, struct command_invocation *cmd);

	/**
	 * A more gentle flavor of flash_driver::probe, performing
	 * setup with less noise.  Generally, driver routines should test
	 * to see if the bank has already been probed; if it has, the
	 * driver probably should not perform its probe a second time.
	 *
	 * This callback is often called from the inside of other
	 * routines (e.g. GDB flash downloads) to autoprobe the flash as
	 * it is programming the flash.
	 *
	 * @param bank - the bank to probe
	 * @returns ERROR_OK if successful; otherwise, an error code.
	 */
	int (*auto_probe)(struct flash_bank *bank);

	/**
	 * Deallocates private driver structures.
	 * Use default_flash_free_driver_priv() to simply free(bank->driver_priv)
	 *
	 * @param bank - the bank being destroyed
	 */
	void (*free_driver_priv)(struct flash_bank *bank);
};

#define FLASH_BANK_COMMAND_HANDLER(name) \
	static __FLASH_BANK_COMMAND(name)

/**
 * Find a NOR flash driver by its name.
 * @param name The name of the requested driver.
 * @returns The flash_driver called @c name, or NULL if not found.
 */
const struct flash_driver *flash_driver_find_by_name(const char *name);

extern const struct flash_driver aduc702x_flash;
extern const struct flash_driver aducm360_flash;
extern const struct flash_driver ambiqmicro_flash;
extern const struct flash_driver at91sam3_flash;
extern const struct flash_driver at91sam4_flash;
extern const struct flash_driver at91sam4l_flash;
extern const struct flash_driver at91sam7_flash;
extern const struct flash_driver at91samd_flash;
extern const struct flash_driver ath79_flash;
extern const struct flash_driver atsame5_flash;
extern const struct flash_driver atsamv_flash;
extern const struct flash_driver avr_flash;
extern const struct flash_driver bluenrgx_flash;
extern const struct flash_driver cc26xx_flash;
extern const struct flash_driver cc3220sf_flash;
extern const struct flash_driver cfi_flash;
extern const struct flash_driver dsp5680xx_flash;
extern const struct flash_driver dw_spi_flash;
extern const struct flash_driver efm32_flash;
extern const struct flash_driver em357_flash;
extern const struct flash_driver eneispif_flash;
extern const struct flash_driver esirisc_flash;
extern const struct flash_driver faux_flash;
extern const struct flash_driver fespi_flash;
extern const struct flash_driver fm3_flash;
extern const struct flash_driver fm4_flash;
extern const struct flash_driver jtagspi_flash;
extern const struct flash_driver kinetis_flash;
extern const struct flash_driver kinetis_ke_flash;
extern const struct flash_driver lpc2000_flash;
extern const struct flash_driver lpc288x_flash;
extern const struct flash_driver lpc2900_flash;
extern const struct flash_driver lpcspifi_flash;
extern const struct flash_driver max32xxx_flash;
extern const struct flash_driver mdr_flash;
extern const struct flash_driver mrvlqspi_flash;
extern const struct flash_driver msp432_flash;
extern const struct flash_driver mspm0_flash;
extern const struct flash_driver niietcm4_flash;
extern const struct flash_driver npcx_flash;
extern const struct flash_driver nrf51_flash;
extern const struct flash_driver nrf5_flash;
extern const struct flash_driver numicro_flash;
extern const struct flash_driver ocl_flash;
extern const struct flash_driver pic32mx_flash;
extern const struct flash_driver psoc4_flash;
extern const struct flash_driver psoc5lp_eeprom_flash;
extern const struct flash_driver psoc5lp_flash;
extern const struct flash_driver psoc5lp_nvl_flash;
extern const struct flash_driver psoc6_flash;
extern const struct flash_driver qn908x_flash;
extern const struct flash_driver renesas_rpchf_flash;
extern const struct flash_driver rp2040_flash;
extern const struct flash_driver rsl10_flash;
extern const struct flash_driver sh_qspi_flash;
extern const struct flash_driver sim3x_flash;
extern const struct flash_driver stellaris_flash;
extern const struct flash_driver stm32f1x_flash;
extern const struct flash_driver stm32f2x_flash;
extern const struct flash_driver stm32h7x_flash;
extern const struct flash_driver stm32l4x_flash;
extern const struct flash_driver stm32lx_flash;
extern const struct flash_driver stmqspi_flash;
extern const struct flash_driver stmsmi_flash;
extern const struct flash_driver str7x_flash;
extern const struct flash_driver str9x_flash;
extern const struct flash_driver str9xpec_flash;
extern const struct flash_driver swm050_flash;
extern const struct flash_driver tms470_flash;
extern const struct flash_driver virtual_flash;
extern const struct flash_driver w600_flash;
extern const struct flash_driver xcf_flash;
extern const struct flash_driver xmc1xxx_flash;
extern const struct flash_driver xmc4xxx_flash;

#endif /* OPENOCD_FLASH_NOR_DRIVER_H */
