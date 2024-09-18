/* SPDX-License-Identifier: GPL-2.0-or-later */
/**
 * @file
 * Driver for SPI NOR flash chips connected via DesignWare SPI Core.
 *
 * In order to avoid using stack, all helper function arguments are packed
 * into a single struct, passed by pointer.
 *
 * Pointers are represented by 64 bit integers to make structs compatible
 * with 64 bit targets.
 *
 * This file contains helper function argument structures.
 */

#ifndef OPENOCD_FLASH_NOR_DW_SPI_HELPER_H
#define OPENOCD_FLASH_NOR_DW_SPI_HELPER_H

#include <stdint.h>

/**
 * @brief Arguments for transaction helper function.
 */
struct dw_spi_transaction {
	uint64_t buffer;
	///< Pointer to data buffer to send over SPI.
	///< Return values are stored in place of output data when
	///< dw_spi_transaction::read_flag is 1.
	uint32_t size; ///< Size of dw_spi_transaction::buffer.
	uint64_t status_reg; ///< Pointer to SR register.
	uint64_t data_reg; ///< Pointer to DR register.
	uint8_t read_flag;
	///< When 1, store RX FIFO data to dw_spi_transaction::buffer.
} __attribute__((packed));

/**
 * @brief Arguments for check_fill helper function.
 */
struct dw_spi_check_fill {
	uint32_t address; ///< Starting address. Sector aligned.
	uint32_t sector_size; ///< Sector size.
	uint32_t sector_count; ///< Number of sectors to check.
	uint64_t status_reg; ///< Pointer to SR register.
	uint64_t data_reg; ///< Pointer to DR register.
	uint64_t fill_status_array;
	///< Pointer to array describing sectors fill status.
	///< 1 if filled, 0 if not filled.
	uint8_t pattern; ///< Fill pattern.
	uint8_t read_cmd; ///< Read data command.
	uint8_t four_byte_mode; ///< Four byte addressing mode flag.
} __attribute__((packed));

/**
 * @brief Arguments for erase helper function.
 */
struct dw_spi_erase {
	uint32_t address; ///< First sector address. Sector aligned.
	uint32_t sector_size; ///< Sector size.
	uint32_t sector_count; ///< Number of sectors to erase.
	uint64_t status_reg; ///< Pointer to SR register.
	uint64_t data_reg; ///< Pointer to DR register.
	uint8_t read_status_cmd; ///< Read status command.
	uint8_t write_enable_cmd; ///< Write enable command.
	uint8_t erase_sector_cmd; ///< Erase sector command.
	uint8_t write_enable_mask; ///< Write enable mask.
	uint8_t busy_mask; ///< Busy mask.
	uint8_t four_byte_mode; ///< Four byte addressing mode flag.
} __attribute__((packed));

/**
 * @brief Arguments for program helper function.
 */
struct dw_spi_program {
	uint32_t address;
	///< First page address. Page aligned when write is crossing
	///< the page boundary.
	uint32_t page_size; ///< Page size.
	uint64_t buffer; ///< Data buffer pointer.
	uint32_t buffer_size; ///< Size of dw_spi_program::buffer.
	uint64_t status_reg; ///< Pointer to SR register.
	uint64_t data_reg; ///< Pointer to DR register.
	uint8_t read_status_cmd; ///< Read status command.
	uint8_t write_enable_cmd; ///< Write enable command.
	uint8_t program_cmd; ///< Program command.
	uint8_t write_enable_mask; ///< Write enable mask.
	uint8_t busy_mask; ///< Busy mask.
	uint8_t four_byte_mode; ///< Four byte addressing mode flag.
} __attribute__((packed));

/**
 * @brief Arguments for read helper function.
 */
struct dw_spi_read {
	uint32_t address; ///< First sector address.
	uint64_t buffer; ///< Data buffer pointer.
	uint32_t buffer_size; ///< Size of dw_spi_read::buffer.
	uint64_t status_reg; ///< Pointer to SR register.
	uint64_t data_reg; ///< Pointer to DR register.
	uint8_t read_cmd; ///< Read data command.
	uint8_t four_byte_mode; ///< Four byte addressing mode flag.
} __attribute__((packed));

#endif /* OPENOCD_FLASH_NOR_DW_SPI_HELPER_H */
