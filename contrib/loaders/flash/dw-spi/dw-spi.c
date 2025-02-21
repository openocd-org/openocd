// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * @file
 * Helper functions for DesignWare SPI Core driver.
 * These helpers are loaded into CPU and execute Flash manipulation algorithms
 * at full CPU speed. Due to inability to control nCS pin, this is the only way
 * to communicate with Flash chips connected via DW SPI serial interface.
 *
 * In order to avoid using stack, all functions used in helpers are inlined.
 * Software breakpoints are used to terminate helpers.
 *
 * Pushing byte to TX FIFO does not make byte immediately available in RX FIFO
 * and nCS is only asserted when TX FIFO is not empty. General approach is to
 * fill TX FIFO with as many bytes as possible, at the same time reading
 * available bytes from RX FIFO.
 *
 * This file contains helper functions.
 */

#include "dw-spi.h"

#include "../../../../src/flash/nor/dw-spi-helper.h"

/**
 * @brief Generic flash transaction.
 *
 * @param[in] arg: Function arguments.
 */
__attribute__((section(".transaction"))) void
transaction(struct dw_spi_transaction *arg)
{
	register uint8_t *buffer_tx = (uint8_t *)arg->buffer;
	register uint8_t *buffer_rx = buffer_tx;
	register uint32_t size = arg->size;
	register volatile uint8_t *status = (uint8_t *)arg->status_reg;
	register volatile uint8_t *data = (uint8_t *)arg->data_reg;

	wait_tx_finish(status);
	flush_rx(status, data);

	for (; size > 0; size--) {
		send_u8(status, data, *buffer_tx++);
		if (arg->read_flag && rx_available(status))
			*buffer_rx++ = rcv_byte(data);
	}

	// Pushed all data to TX FIFO. Read bytes left in RX FIFO.
	if (arg->read_flag) {
		while (buffer_rx < buffer_tx) {
			wait_rx_available(status);
			*buffer_rx++ = rcv_byte(data);
		}
	}

	RETURN;
}

/**
 * @brief Check flash sectors are filled with pattern. Primary use for
 * checking sector erase state.
 *
 * @param[in] arg: Function arguments.
 */
__attribute__((section(".check_fill"))) void
check_fill(struct dw_spi_check_fill *arg)
{
	register uint32_t tx_size;
	register uint32_t rx_size;
	register uint32_t dummy_count;
	register uint8_t filled;
	register uint8_t *fill_status_array = (uint8_t *)arg->fill_status_array;
	register volatile uint8_t *status = (uint8_t *)arg->status_reg;
	register volatile uint8_t *data = (uint8_t *)arg->data_reg;

	for (; arg->sector_count > 0; arg->sector_count--,
								  arg->address += arg->sector_size,
								  fill_status_array++) {
		wait_tx_finish(status);
		flush_rx(status, data);

		/*
		 * Command byte and address bytes make up for dummy_count number of
		 * bytes, that must be skipped in RX FIFO before actual data arrives.
		 */
		send_u8(status, data, arg->read_cmd);
		if (arg->four_byte_mode) {
			dummy_count = 1 + 4; // Command byte + 4 address bytes
			send_u32(status, data, arg->address);
		} else {
			dummy_count = 1 + 3; // Command byte + 3 address bytes
			send_u24(status, data, arg->address);
		}

		for (tx_size = arg->sector_size, rx_size = arg->sector_size, filled = 1;
			 tx_size > 0; tx_size--) {
			send_u8(status, data, 0); // Dummy write to push out read data.
			if (rx_available(status)) {
				if (dummy_count > 0) {
					// Read data not arrived yet.
					rcv_byte(data);
					dummy_count--;
				} else {
					if (rcv_byte(data) != arg->pattern) {
						filled = 0;
						break;
					}
					rx_size--;
				}
			}
		}
		if (filled) {
			for (; rx_size > 0; rx_size--) {
				wait_rx_available(status);
				if (rcv_byte(data) != arg->pattern) {
					filled = 0;
					break;
				}
			}
		}
		*fill_status_array = filled;
	}

	RETURN;
}

/**
 * @brief Erase flash sectors.
 *
 * @param[in] arg: Function arguments.
 */
__attribute__((section(".erase"))) void
erase(struct dw_spi_erase *arg)
{
	register uint32_t address = arg->address;
	register uint32_t count = arg->sector_count;
	register volatile uint8_t *status = (uint8_t *)arg->status_reg;
	register volatile uint8_t *data = (uint8_t *)arg->data_reg;

	for (; count > 0; count--, address += arg->sector_size) {
		write_enable(status, data, arg->write_enable_cmd);
		wait_write_enable(status, data, arg->read_status_cmd,
						  arg->write_enable_mask);

		erase_sector(status, data, arg->erase_sector_cmd, address,
					 arg->four_byte_mode);
		wait_busy(status, data, arg->read_status_cmd, arg->busy_mask);
	}

	RETURN;
}

/**
 * @brief Flash program.
 *
 * @param[in] arg: Function arguments.
 */
__attribute__((section(".program"))) void
program(struct dw_spi_program *arg)
{
	register uint8_t *buffer = (uint8_t *)arg->buffer;
	register uint32_t buffer_size = arg->buffer_size;
	register volatile uint8_t *status = (uint8_t *)arg->status_reg;
	register volatile uint8_t *data = (uint8_t *)arg->data_reg;
	register uint32_t page_size;

	while (buffer_size > 0) {
		write_enable(status, data, arg->write_enable_cmd);
		wait_write_enable(status, data, arg->read_status_cmd,
						  arg->write_enable_mask);

		wait_tx_finish(status);

		send_u8(status, data, arg->program_cmd);
		if (arg->four_byte_mode)
			send_u32(status, data, arg->address);
		else
			send_u24(status, data, arg->address);

		for (page_size = MIN(arg->page_size, buffer_size); page_size > 0;
			 page_size--, buffer_size--) {
			send_u8(status, data, *buffer++);
		}
		arg->address += arg->page_size;
		wait_busy(status, data, arg->read_status_cmd, arg->busy_mask);
	}

	RETURN;
}

/**
 * @brief Read data from flash.
 *
 * @param[in] arg: Function arguments.
 */
__attribute__((section(".read"))) void
read(struct dw_spi_read *arg)
{
	register uint32_t tx_size = arg->buffer_size;
	register uint32_t rx_size = arg->buffer_size;
	register uint32_t dummy_count;
	register uint8_t *buffer = (uint8_t *)arg->buffer;
	register volatile uint8_t *status = (uint8_t *)arg->status_reg;
	register volatile uint8_t *data = (uint8_t *)arg->data_reg;

	wait_tx_finish(status);
	flush_rx(status, data);

	/*
	 * Command byte and address bytes make up for dummy_count number of
	 * bytes, that must be skipped in RX FIFO before actual data arrives.
	 */
	send_u8(status, data, arg->read_cmd);
	if (arg->four_byte_mode) {
		dummy_count = 1 + 4; // Command byte + 4 address bytes
		send_u32(status, data, arg->address);
	} else {
		dummy_count = 1 + 3; // Command byte + 3 address bytes
		send_u24(status, data, arg->address);
	}

	for (; tx_size > 0; tx_size--) {
		send_u8(status, data, 0); // Dummy write to push out read data.
		if (rx_available(status)) {
			if (dummy_count > 0) {
				rcv_byte(data);
				dummy_count--;
			} else {
				*buffer++ = rcv_byte(data);
				rx_size--;
			}
		}
	}
	while (rx_size > 0) {
		wait_rx_available(status);
		if (dummy_count > 0) {
			// Read data not arrived yet.
			rcv_byte(data);
			dummy_count--;
		} else {
			*buffer++ = rcv_byte(data);
			rx_size--;
		}
	}

	RETURN;
}
