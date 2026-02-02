// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2015 by Maxim Integrated                                *
 *   Copyright (C) 2025-2026 Analog Devices, Inc.                          *
 ***************************************************************************/

// **** Includes ****

#include <string.h>

#ifdef ALGO_TEST
#include "mxc_device.h"
#endif

#include "../../../../src/flash/nor/max32xxx_options.h"
#include "../../../../src/flash/nor/max32xxx_qspi_common_defs.h"
#include "ctb_regs.h"
#include "../max32xxx/flc_regs.h"
#include "../max32xxx/gcr_regs.h"

#ifdef ALGO_TEST
#include <stdio.h>
#else
#define printf(...)
#endif

#define MXC_BASE_CTB ((uint32_t)0x40001000UL)
#define MXC_CTB ((struct mxc_ctb_regs *)MXC_BASE_CTB)
#define MXC_BASE_GCR ((uint32_t)0x40000000UL)
#define MXC_GCR ((struct mxc_gcr_regs *)MXC_BASE_GCR)

#define ERROR_OK 0

#define SPIFLASH_WRITE_ENABLE 0x06
#define SPIFLASH_READ_STATUS 0x05
#define SPIFLASH_BSY 0
#define SPIFLASH_BSY_BIT (0x1 << SPIFLASH_BSY)

#define SPIXFC_FIFO_DEPTH (16)
#define SPIXFC_HEADER_NULL                                                                                             \
	0xF000 /* 16-bit filler magic word indicating this                                                                 \
			* isn't a header */

#define getbyte(temp8)                                                          \
	/* Wait for the Read FIFO to not equal the Write FIFO */                    \
	do {while (*read_ptr == *write_ptr);                                        \
	temp8 = **read_ptr;                                                         \
	/* Increment and wrap around the read pointer */                            \
	if ((*read_ptr + 1) >= (uint8_t *)(fifo_end)) {                             \
		*read_ptr = fifo_start;                                                  \
	} else {                                                                    \
		(*read_ptr)++;                                                          \
	}                                                                           \
	len--;                                                                      \
	addr++; } while (0)

void algo_write(uint8_t *work_start, uint8_t *work_end, uint32_t len, uint32_t addr);

#ifndef ALGO_TEST
__attribute__ ((naked, noreturn, section(".algo_entry"), used))
void algo_entry(uint8_t *work_start, uint8_t *work_end, uint32_t len, uint32_t addr)
{
	(void)work_start;
	(void)work_end;
	(void)len;
	(void)addr;
	__asm volatile ("b algo_write");
}

__attribute__ ((naked, noreturn, section(".algo_exit"), used))
void algo_exit(void)
{
	__asm volatile ("bkpt #0");
}
#endif

static void memcpy32(uint32_t *dst, const uint32_t *src, unsigned int len)
{
	while (len) {
		*dst = *src;
		dst++;
		src++;
		len -= 4;
	}
}

static inline void target_read_u32(uint32_t addr, uint32_t *data)
{
	volatile uint32_t *data_ptr = (uint32_t *)addr;

	*data = *data_ptr;
}

static inline void target_read_u8(uint32_t addr, uint8_t *data)
{
	volatile uint8_t *data_ptr = (uint8_t *)addr;

	*data = *data_ptr;
}

static inline void target_write_u16(uint32_t addr, uint16_t data)
{
	volatile uint16_t *data_ptr = (uint16_t *)addr;

	*data_ptr = data;
}

int max32xxx_qspi_write_txfifo(const uint8_t *data, unsigned int len)
{
	unsigned int data_i = 0;

	while (len - data_i) {
		unsigned int write_len;

		// Calculate how many bytes we can write on this round
		if ((len - data_i) > SPIXFC_FIFO_DEPTH)
			write_len = SPIXFC_FIFO_DEPTH;
		else
			write_len = (len - data_i);

		unsigned int tx_fifo_avail;
		// Wait for there to be room in the TX FIFO
		do {
			uint32_t temp32 = 0;
			target_read_u32(SPIXFC_FIFO_CTRL, &temp32);
			tx_fifo_avail =
				SPIXFC_FIFO_DEPTH - ((temp32 & SPIXFC_FIFO_CTRL_TX_FIFO_CNT) >> SPIXFC_FIFO_CTRL_TX_FIFO_CNT_POS);
		} while (tx_fifo_avail < write_len);

		while (write_len) {
			uint16_t write_data = data[data_i++];

			if (data_i < len) {
				write_data |= (data[data_i++] << 8);
				write_len -= 2;
			} else {
				write_data |= SPIXFC_HEADER_NULL;
				write_len -= 1;
			}
			target_write_u16(SPIXFC_FIFO_TX, write_data);
		}
	}

	return ERROR_OK;
}

int max32xxx_qspi_read_rxfifo(uint8_t *data, unsigned int len)
{
	unsigned int data_i = 0;

	while (len - data_i) {
		unsigned int rx_fifo_avail;

		// Wait for there to be data in the RX FIFO
		do {
			uint32_t temp32;
			target_read_u32(SPIXFC_FIFO_CTRL, &temp32);
			rx_fifo_avail = (temp32 & SPIXFC_FIFO_CTRL_RX_FIFO_CNT) >> SPIXFC_FIFO_CTRL_RX_FIFO_CNT_POS;
		} while (!rx_fifo_avail);

		unsigned int read_len;
		// Calculate how many bytes we can write on this round
		if ((len - data_i) > rx_fifo_avail)
			read_len = rx_fifo_avail;
		else
			read_len = (len - data_i);

		while (read_len) {
			target_read_u8(SPIXFC_FIFO_RX, &data[data_i++]);
			read_len--;
		}
	}

	return ERROR_OK;
}

int max32xxx_qspi_write_bytes(const uint8_t *data, unsigned int len, int deass)
{
	int retval = ERROR_OK;
	unsigned int data_i = 0;

	// Wrap the length
	while (len - data_i) {
		uint16_t header;
		unsigned int chunk_len;
		// Max transaction length is 32 units
		if ((len - data_i) > 32)
			chunk_len = 32;
		else
			chunk_len = (len - data_i);

		// Setup the SPI header, 32 maps to 0 in the size field
		header = (SPIXFC_HEADER_TX | SPIXFC_HEADER_BYTE);
		if (chunk_len == 32)
			header |= (0 << SPIXFC_HEADER_SIZE_POS);
		else
			header |= (chunk_len << SPIXFC_HEADER_SIZE_POS);

		// If we de-asserting and this is the final chunk
		if (deass && ((len - data_i - chunk_len) == 0))
			header |= SPIXFC_HEADER_SS_DEASS;

		// Write the header to the TX FIFO
		retval = max32xxx_qspi_write_txfifo((uint8_t *)&header, sizeof(header));
		if (retval != ERROR_OK)
			return retval;

		// Write the data to the TX FIFO
		retval = max32xxx_qspi_write_txfifo(&data[data_i], chunk_len);
		if (retval != ERROR_OK)
			return retval;
		data_i += chunk_len;
	}

	return retval;
}

int max32xxx_qspi_read_bytes(uint8_t *data, unsigned int len, int deass)
{
	int retval = ERROR_OK;
	unsigned int data_i = 0;

	// Wrap the length
	while (len - data_i) {
		uint16_t header;
		unsigned int chunk_len;
		// Max transaction length is 32 units
		if ((len - data_i) > 32)
			chunk_len = 32;
		else
			chunk_len = (len - data_i);

		// Setup the SPI header, 32 maps to 0 in the size field
		header = SPIXFC_HEADER_RX | SPIXFC_HEADER_BYTE;
		if (chunk_len == 32)
			header |= (0 << SPIXFC_HEADER_SIZE_POS);
		else
			header |= (chunk_len << SPIXFC_HEADER_SIZE_POS);

		// If we de-asserting and this is the final chunk
		if (deass && ((len - data_i - chunk_len) == 0))
			header |= SPIXFC_HEADER_SS_DEASS;

		// Write the header to the TX FIFO
		retval = max32xxx_qspi_write_txfifo((uint8_t *)&header, sizeof(header));
		if (retval != ERROR_OK)
			return retval;

		// Read the data to the TX FIFO, convert to number of bytes
		retval = max32xxx_qspi_read_rxfifo((uint8_t *)&data[data_i], chunk_len);
		if (retval != ERROR_OK)
			return retval;
		data_i += chunk_len;
	}

	return retval;
}

int max32xxx_qspi_poll_wip(void)
{
	uint8_t read_data;
	int retval;

	do {
		uint8_t cmd_data = SPIFLASH_READ_STATUS;
		retval = max32xxx_qspi_write_bytes(&cmd_data, 1, 0);
		if (retval != ERROR_OK)
			return retval;

		read_data = SPIFLASH_BSY_BIT;
		retval = max32xxx_qspi_read_bytes(&read_data, 1, 1);
		if (retval != ERROR_OK)
			return retval;

	} while (read_data & SPIFLASH_BSY_BIT);

	return ERROR_OK;
}

int max32xxx_qspi_set_we(void)
{
	uint8_t cmd_data = SPIFLASH_WRITE_ENABLE;

	return max32xxx_qspi_write_bytes(&cmd_data, 1, 1);
}

int max32xxx_qspi_write(const uint8_t *buffer, uint32_t offset, uint32_t count, uint32_t spi_cmd)
{
	int retval = ERROR_OK;
	unsigned int buffer_i = 0;
	uint8_t cmd_data[5];
	// Send the page program command
	cmd_data[0] = spi_cmd & 0xFF;

	while (count - buffer_i) {
		retval = max32xxx_qspi_poll_wip();
		if (retval != ERROR_OK)
			goto exit;

		// Set the write enable
		retval = max32xxx_qspi_set_we();
		if (retval != ERROR_OK)
			goto exit;

		unsigned int write_len = SPI_WRITE_BOUNDARY - (offset % SPI_WRITE_BOUNDARY);

		if (write_len > (count - buffer_i))
			write_len = (count - buffer_i);

		if (spi_cmd == 0x12) {
			cmd_data[4] = (offset & 0x000000FF) >> 0;
			cmd_data[3] = (offset & 0x0000FF00) >> 8;
			cmd_data[2] = (offset & 0x00FF0000) >> 16;
			cmd_data[1] = (offset & 0xFF000000) >> 24;
			// Write the command
			retval = max32xxx_qspi_write_bytes(cmd_data, 5, 0);
		} else {
			// Address is MSB first
			cmd_data[3] = (offset & 0x0000FF) >> 0;
			cmd_data[2] = (offset & 0x00FF00) >> 8;
			cmd_data[1] = (offset & 0xFF0000) >> 16;
			// Write the command
			retval = max32xxx_qspi_write_bytes(cmd_data, 4, 0);
		}

		if (retval != ERROR_OK)
			goto exit;

		// Write the data
		retval = max32xxx_qspi_write_bytes(&buffer[buffer_i], write_len, 1);
		if (retval != ERROR_OK)
			goto exit;

		// Increment the pointers
		buffer_i += write_len;
		offset += write_len;
	}

exit:
	return retval;
}

void aes_gcm(uint32_t *plain, uint32_t *cipher, uint32_t addr, uint8_t *auth_buffer)
{
	// Reset the CTB
	MXC_CTB->crypto_ctrl = MXC_F_CTB_CRYPTO_CTRL_RST;

	// Set the legacy bit
	MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_FLAG_MODE;

	// Byte swap the input and output
	MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_BSO;
	MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_BSI;

	// Clear interrupt flags
	MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_CPH_DONE;

	// Setup the key source
	MXC_CTB->cipher_ctrl = MXC_S_CTB_CIPHER_CTRL_SRC_QSPIKEY_REGFILE;

	// IV[95:0] = {IV_FIXED_VALUE[47:0], address[31:0], counter[15:0]}
	uint8_t iv[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	uint16_t counter_value = (addr & 0xFFFF0) >> 4;

	// Byte swapping within the 32-bit word, in reverse order
	iv[8] = counter_value & 0xFF;
	iv[9] = (counter_value & 0xFF00) >> 8;
	iv[10] = (addr & 0x000000FF) >> 0;
	iv[11] = (addr & 0x0000FF00) >> 8;
	iv[4] = (addr & 0x00FF0000) >> 16;
	iv[5] = (addr & 0xFF000000) >> 24;

	// Copy in the IV
	memcpy32((uint32_t *)MXC_CTB->cipher_init, (uint32_t *)iv, sizeof(iv));

	// Computer H
	MXC_CTB->cipher_ctrl |= MXC_F_CTB_CIPHER_CTRL_HVC;

	// Wait for and clear the done flag
	do {
	} while (!(MXC_CTB->crypto_ctrl & MXC_F_CTB_CRYPTO_CTRL_CPH_DONE));

	MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_CPH_DONE;

	// Setup the CT calculation
	MXC_CTB->cipher_ctrl |=
		MXC_S_CTB_CIPHER_CTRL_MODE_GCM | MXC_S_CTB_CIPHER_CTRL_CIPHER_AES128 | MXC_F_CTB_CIPHER_CTRL_DTYPE;

	// Clear the aad and setup the payload length
	MXC_CTB->aad_length_0 = 0;
	MXC_CTB->aad_length_1 = 0;
	MXC_CTB->pld_length_0 = 16;
	MXC_CTB->pld_length_1 = 0;

	// Copy in the data
	memcpy32((uint32_t *)MXC_CTB->crypto_din, (uint32_t *)plain, 16);

	// Wait for and clear the done flag
	do {
	} while (!(MXC_CTB->crypto_ctrl & MXC_F_CTB_CRYPTO_CTRL_CPH_DONE));

	MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_CPH_DONE;

	// Copy out the CT
	memcpy32(cipher, (uint32_t *)MXC_CTB->crypto_dout, 16);

	// Copy out the auth data
	auth_buffer[2 * ((addr % 0x80) >> 4) + 0] = (MXC_CTB->tagmic[3] & 0x00FF) >> 0;
	auth_buffer[2 * ((addr % 0x80) >> 4) + 1] = (MXC_CTB->tagmic[3] & 0xFF00) >> 8;
}

void aes_ecb(uint32_t *plain, uint32_t *cipher)
{
	// Reset the CTB
	MXC_CTB->crypto_ctrl = MXC_F_CTB_CRYPTO_CTRL_RST;

	// Set the legacy bit
	MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_FLAG_MODE;

	// Byte swap the input and output
	MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_BSO;
	MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_BSI;

	// Clear interrupt flags
	MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_CPH_DONE;

	// Setup the key source
	MXC_CTB->cipher_ctrl = MXC_S_CTB_CIPHER_CTRL_SRC_QSPIKEY_REGFILE;

	// Setup the CT calculation
	MXC_CTB->cipher_ctrl |= MXC_S_CTB_CIPHER_CTRL_CIPHER_AES128;

	// Copy data to start the operation
	memcpy32((uint32_t *)MXC_CTB->crypto_din, (uint32_t *)plain, 16);

	// Wait for and clear the done flag
	do {
	} while (!(MXC_CTB->crypto_ctrl & MXC_F_CTB_CRYPTO_CTRL_CPH_DONE));

	MXC_CTB->crypto_ctrl |= MXC_F_CTB_CRYPTO_CTRL_CPH_DONE;

	// Copy the data out
	memcpy32(cipher, (uint32_t *)MXC_CTB->crypto_dout, 16);
}

void algo_write(uint8_t *work_start, uint8_t *work_end, uint32_t len, uint32_t addr)
{
	printf(" > %s starting\n", __func__);

	// Setup the pointers
	uint8_t *volatile *write_ptr = (uint8_t **)(work_start + SPIX_ALGO_WRITE_PTR_OFFSET);
	uint8_t *volatile *read_ptr = (uint8_t **)(work_start + SPIX_ALGO_READ_PTR_OFFSET);
	uint8_t *fifo_start = work_start + SPIX_ALGO_HEADER_SIZE;
	uint8_t *fifo_end = work_end - SPIX_ALGO_TAIL_SIZE;
	struct max32xxx_qspi_algo_tail *tail = (struct max32xxx_qspi_algo_tail *)(work_end - SPIX_ALGO_TAIL_SIZE);
	uint32_t *spi_write_cmd = &tail->spi_write_cmd;
	uint32_t *options = &tail->options;
	uint32_t pt_buffer[4];

	printf(" > w:%08x r:%08x o:%08x enc:%08x s:%08x e:%08x spi:%08x\n", (uint32_t)write_ptr, (uint32_t)read_ptr,
		   (uint32_t)*options, (uint32_t)pt_buffer, (uint32_t)fifo_start, (uint32_t)fifo_end, (uint32_t)*spi_write_cmd);

	if (*options & OPTIONS_ENC) {
		// Setup the AES

		// Enable CRYPTO clock
		if ((MXC_GCR->clkcn & MXC_F_GCR_CLKCN_HIRC_EN) == 0)
			MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_HIRC_EN;

		// Disable CRYPTO clock gate
		if (MXC_GCR->perckcn0 & MXC_F_GCR_PERCKCN0_CRYPTOD)
			MXC_GCR->perckcn0 &= ~(MXC_F_GCR_PERCKCN0_CRYPTOD);
	}

	uint32_t addr_logic = addr;
	uint32_t addr_physic = addr;

	if (*options & OPTIONS_AUTH) {
		/* Scale the physical address to match the starting logical address
		 * Account for the 0x20 bytes of authentication data for each 0x80 byte block
		 */
		addr_physic = addr_logic + ((addr_logic / 0x80) * 0x20);
	}

	printf(" > addr        = 0x%x\n", addr);
	printf(" > addr_logic  = 0x%x\n", addr_logic);
	printf(" > addr_physic = 0x%x\n", addr_physic);
	printf(" > len         = 0x%x\n", len);

	int i;
	while (len > 0) {
		// Fill the buffer with the plain text data from the working area
		for (i = 0; i < 4; i++) {
			uint8_t temp8;
			// Data is already aligned/padded by the host flash framework.
			pt_buffer[i] = 0;
			getbyte(temp8);
			pt_buffer[i] |= (temp8 << (0));
			getbyte(temp8);
			pt_buffer[i] |= (temp8 << (8));
			getbyte(temp8);
			pt_buffer[i] |= (temp8 << (16));
			getbyte(temp8);
			pt_buffer[i] |= (temp8 << (24));
		}

		uint32_t ct_buffer[4];
		uint8_t auth_buffer[16];
		if (*options & OPTIONS_ENC) {
			if (*options & OPTIONS_AUTH) {
				uint32_t addr_logic_tmp;

				if (*options & OPTIONS_RELATIVE_XOR)
					addr_logic_tmp = (addr_logic & 0x00FFFFF0);
				else
					addr_logic_tmp = (addr_logic & 0xFFFFFFF0);

				aes_gcm(pt_buffer, ct_buffer, addr_logic_tmp, auth_buffer);
			} else {
				// XOR data with the address
				for (i = 0; i < 4; i++) {
					if (*options & OPTIONS_RELATIVE_XOR)
						pt_buffer[i] ^= ((addr_logic & 0x00FFFFF0) + i * 4);

					else
						pt_buffer[i] ^= (((addr_logic & 0xFFFFFFF0) | 0x08000000) + i * 4);
				}
				aes_ecb(pt_buffer, ct_buffer);
			}
		} else {
			memcpy(ct_buffer, pt_buffer, 16);
		}

		// Write the data to the flash
		if (max32xxx_qspi_write((const uint8_t *)ct_buffer, addr_physic, 16, *spi_write_cmd) != ERROR_OK) {
#ifndef ALGO_TEST
			__asm("bkpt\n");
#else
			printf(" > %s error\n", __func__);
			return;
#endif
		}
		// Increment the physical address
		addr_physic += 16;

		if ((*options & OPTIONS_AUTH) && (*options & OPTIONS_ENC) && ((addr_logic % 0x80) == 0x70)) {
			// Write the authentication information to the flash
			if (max32xxx_qspi_write((const uint8_t *)auth_buffer, addr_physic, 16, *spi_write_cmd) != ERROR_OK) {
#ifndef ALGO_TEST
				__asm("bkpt\n");
#else
				printf(" > %s error\n", __func__);
				return;
#endif
			}
			/* Only increment the physical addresses since this was authentication data
			 * */
			addr_physic += 16;

			// Write the counter values to the flash
			uint16_t counters[8];
			uint16_t counter_base;
			if (*options & OPTIONS_RELATIVE_XOR)
				counter_base = ((addr_logic & 0x00FFFFF0) >> 4) - 0x7;
			else
				counter_base = ((addr_logic & 0xFFFFFF0) >> 4) - 0x7;
			for (i = 0; i < 8; i++)
				counters[i] = counter_base + i;
			if (max32xxx_qspi_write((const uint8_t *)counters, addr_physic, 16, *spi_write_cmd) != ERROR_OK) {
#ifndef ALGO_TEST
				__asm("bkpt\n");
#else
				printf(" > %s error\n", __func__);
				return;
#endif
			}
			/* Only increment the physical addresses since this was authentication data
			 * */
			addr_physic += 16;
		}

		// Increment the logical address
		addr_logic += 16;
	}
#ifndef ALGO_TEST
	algo_exit();
#else
	printf(" > %s returning\n", __func__);
	return;
#endif
}
