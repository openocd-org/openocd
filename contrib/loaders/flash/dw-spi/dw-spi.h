/* SPDX-License-Identifier: GPL-2.0-or-later */
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
 * This file contains functions, common to helpers.
 */

#ifndef _DW_SPI_H_
#define _DW_SPI_H_

#include <stdint.h>
#include <sys/param.h>

#include "../../../../src/helper/types.h"

/**
 * @brief SI busy status bit.
 *
 * Set when serial transfer is in progress, cleared when master is idle or
 * disabled.
 */
#define DW_SPI_STATUS_BUSY 0x01

/**
 * @brief SI TX FIFO not full status bit.
 *
 * Set when TX FIFO has room for one or more data-word.
 */
#define DW_SPI_STATUS_TFNF 0x02

/**
 * @brief SI TX FIFO empty status bit.
 */
#define DW_SPI_STATUS_TFE 0x04

/**
 * @brief SI RX FIFO not empty status bit.
 */
#define DW_SPI_STATUS_RFNE 0x08

/**
 * @brief Return from helper function.
 */
#define RETURN            \
	do {                  \
		asm("sdbbp\n\t"); \
		return;           \
	} while (0)

/**
 * @brief Append byte to TX FIFO.
 *
 * For each transferred byte, DW SPI controller receives a byte into RX FIFO.
 * Slave data are read by pushing dummy bytes to TX FIFO.
 *
 * @param[in] dr: Pointer to DR register.
 * @param[in] byte: Data to push.
 */
__attribute__((always_inline)) static inline void
_send_byte(volatile uint8_t *dr, uint8_t byte)
{
	*dr = byte;
}

/**
 * @brief Get byte from RX FIFO.
 *
 * Reading RX byte removes it from RX FIFO.
 *
 * @param[in] dr: Pointer to DR register.
 * @return RX FIFO byte.
 */
__attribute__((always_inline)) static inline uint8_t
rcv_byte(volatile uint8_t *dr)
{
	return *dr;
}

/**
 * @brief Check transmission is currently in progress.
 *
 * @param[in] sr: Pointer to SR register.
 * @retval 1: Transmission is in progress.
 * @retval 0: Controller is idle or off.
 */
__attribute__((always_inline)) static inline int
tx_in_progress(volatile uint8_t *sr)
{
	return (*sr ^ DW_SPI_STATUS_TFE) & (DW_SPI_STATUS_BUSY | DW_SPI_STATUS_TFE);
}

/**
 * @brief Wait for controller to finish previous transaction.
 *
 * @param[in] sr: Pointer to SR register.
 */
__attribute__((always_inline)) static inline void
wait_tx_finish(volatile uint8_t *sr)
{
	while (tx_in_progress(sr))
		;
}

/**
 * @brief Wait for room in TX FIFO.
 *
 * @param[in] sr: Pointer to SR register.
 */
__attribute__((always_inline)) static inline void
wait_tx_available(volatile uint8_t *sr)
{
	while (!(*sr & DW_SPI_STATUS_TFNF))
		;
}

/**
 * @brief Check for data available in RX FIFO.
 *
 * @param[in] sr: Pointer to SR register.
 * @retval 1: Data available.
 * @retval 0: No data available.
 */
__attribute__((always_inline)) static inline int
rx_available(volatile uint8_t *sr)
{
	return *sr & DW_SPI_STATUS_RFNE;
}

/**
 * @brief Wait for data in RX FIFO.
 *
 * @param[in] sr: Pointer to SR register.
 */
__attribute__((always_inline)) static inline void
wait_rx_available(volatile uint8_t *sr)
{
	while (!rx_available(sr))
		;
}

/**
 * @brief Flush RX FIFO.
 *
 * @param[in] sr: Pointer to SR register.
 * @param[in] dr: Pointer to DR register.
 */
__attribute__((always_inline)) static inline void
flush_rx(volatile uint8_t *sr, volatile uint8_t *dr)
{
	while (*sr & DW_SPI_STATUS_RFNE)
		*dr;
}

/**
 * @brief Append variable number of bytes to TX FIFO.
 *
 * @param[in] sr: Pointer to SR register.
 * @param[in] dr: Pointer to DR register.
 * @param[in] word: Data to append.
 * @param[in] bytes: Number of bytes to append.
 */
__attribute__((always_inline)) static inline void
_send_bytes(volatile uint8_t *sr, volatile uint8_t *dr, uint32_t word,
			int bytes)
{
	for (register int i = bytes - 1; i >= 0; i--) {
		wait_tx_available(sr);
		_send_byte(dr, (word >> (i * 8)) & 0xff);
	}
}

/**
 * @brief Append 8 bit value to TX FIFO.
 *
 * @param[in] sr: Pointer to SR register.
 * @param[in] dr: Pointer to DR register.
 * @param[in] word: Data to push.
 */
__attribute__((always_inline)) static inline void
send_u8(volatile uint8_t *sr, volatile uint8_t *dr, uint8_t byte)
{
	wait_tx_available(sr);
	_send_byte(dr, byte);
}

/**
 * @brief Append 24 bit value to TX FIFO.
 *
 * Used to send Flash addresses in 24 bit mode.
 *
 * @param[in] sr: Pointer to SR register.
 * @param[in] dr: Pointer to DR register.
 * @param[in] word: Data to push.
 */
__attribute__((always_inline)) static inline void
send_u24(volatile uint8_t *sr, volatile uint8_t *dr, uint32_t word)
{
	_send_bytes(sr, dr, word, 3);
}

/**
 * @brief Append 32 bit value to TX FIFO.
 *
 * @param[in] sr: Pointer to SR register.
 * @param[in] dr: Pointer to DR register.
 * @param[in] word: Data to push.
 */
__attribute__((always_inline)) static inline void
send_u32(volatile uint8_t *sr, volatile uint8_t *dr, uint32_t word)
{
	_send_bytes(sr, dr, word, 4);
}

/**
 * @brief Read chip status register.
 *
 * @param[in] sr: Pointer to SR register.
 * @param[in] dr: Pointer to DR register.
 * @param[in] stat_cmd: Read status command.
 * @return Chip status.
 */
__attribute__((always_inline)) static inline uint8_t
read_status(volatile uint8_t *sr, volatile uint8_t *dr, uint8_t stat_cmd)
{
	wait_tx_finish(sr);
	flush_rx(sr, dr);
	/*
	 * Don't bother with wait_tx_available() as TX FIFO is empty
	 * and we only send two bytes.
	 */
	_send_byte(dr, stat_cmd);
	_send_byte(dr, 0); // Dummy write to push out read data.
	wait_rx_available(sr);
	rcv_byte(dr); // Dummy read to skip command byte.
	wait_rx_available(sr);
	return rcv_byte(dr);
}

/**
 * @brief Enable Flash chip write.
 *
 * @param[in] sr: Pointer to SR register.
 * @param[in] dr: Pointer to DR register.
 * @param[in] we_cmd: Write enable command.
 */
__attribute__((always_inline)) static inline void
write_enable(volatile uint8_t *sr, volatile uint8_t *dr, uint8_t we_cmd)
{
	wait_tx_finish(sr);
	_send_byte(dr, we_cmd);
}

/**
 * @brief Erase Flash sector.
 *
 * @param[in] sr: Pointer to SR register.
 * @param[in] dr: Pointer to DR register.
 * @param[in] erase_cmd: Erase sector cmd.
 * @param[in] address: Sector address.
 * @param[in] four_byte_mode: Device is in 32 bit mode flag.
 */
__attribute__((always_inline)) static inline void
erase_sector(volatile uint8_t *sr, volatile uint8_t *dr, uint8_t erase_cmd,
			 uint32_t address, uint8_t four_byte_mode)
{
	wait_tx_finish(sr);
	_send_byte(dr, erase_cmd);
	if (four_byte_mode)
		send_u32(sr, dr, address);
	else
		send_u24(sr, dr, address);
}

/**
 * @brief Wait for write enable flag.
 *
 * @param[in] sr: Pointer to SR register.
 * @param[in] dr: Pointer to DR register.
 * @param[in] stat_cmd: Read status command.
 * @param[in] we_mask: Write enable status mask.
 */
__attribute__((always_inline)) static inline void
wait_write_enable(volatile uint8_t *sr, volatile uint8_t *dr, uint8_t stat_cmd,
				  uint8_t we_mask)
{
	while (!(read_status(sr, dr, stat_cmd) & we_mask))
		;
}

/**
 * @brief Wait while flash is busy.
 *
 * @param[in] sr: Pointer to SR register.
 * @param[in] dr: Pointer to DR register.
 * @param[in] stat_cmd: Read status command.
 * @param[in] busy_mask: Flash busy mask.
 */
__attribute__((always_inline)) static inline void
wait_busy(volatile uint8_t *sr, volatile uint8_t *dr, uint8_t stat_cmd,
		  uint8_t busy_mask)
{
	while (read_status(sr, dr, stat_cmd) & busy_mask)
		;
}

#endif // _DW_SPI_H_
