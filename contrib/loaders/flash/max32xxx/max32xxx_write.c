// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2015 by Maxim Integrated                                *
 *   Copyright (C) 2025 Analog Devices, Inc.                               *
 ***************************************************************************/

/***** Includes *****/
#ifdef ALGO_TEST
#include "mxc_device.h"
#endif

#include "tpu_regs.h"
#include "gcr_regs.h"
#include "flc_regs.h"
#include "algo_options.h"

#ifdef ALGO_TEST
#include <stdio.h>
#else
#define printf(...)
#endif

/***** Definitions *****/
#define MXC_BASE_TPU                    ((uint32_t)0x40001000UL)
#define MXC_TPU                         ((struct mxc_tpu_regs *)MXC_BASE_TPU)
#define MXC_BASE_GCR                    ((uint32_t)0x40000000UL)
#define MXC_GCR                         ((struct mxc_gcr_regs *)MXC_BASE_GCR)

/******************************************************************************/
#define getbyte(temp8)                                                  \
/* Wait for the Read FIFO to not equal the Write FIFO */                \
	do { while (*read_ptr == *write_ptr);                               \
		temp8 = **read_ptr;                                             \
/* Increment and wrap around the read pointer */                        \
		if ((*read_ptr + 1) >= (uint8_t *)(work_end - 8 - 256)) {       \
			*read_ptr = (uint8_t *)(work_start + 8);                    \
		} else {                                                        \
			(*read_ptr)++;                                              \
		}                                                               \
		len--;                                                          \
		addr++; } while (0)

/******************************************************************************/
#ifndef ALGO_TEST
__attribute__ ((naked, section(".algo")))
#endif
void algo_write(uint8_t *work_start, uint8_t *work_end, uint32_t len, uint32_t addr)
{
	printf(" > %s starting\n", __func__);

	volatile uint8_t * (*write_ptr) = (volatile uint8_t **)work_start;
	volatile uint8_t * (*read_ptr) = (volatile uint8_t **)(work_start + 4);
	uint32_t *flc_base = (uint32_t *)(work_end - 4 - 128);
	uint32_t *options = (uint32_t *)(work_end - 8 - 128);
	uint32_t *enc_buffer = (uint32_t *)(work_end - 8 - 256);
	uint8_t temp8;
	uint32_t addr_save;
	int i;
	struct mxc_flc_regs *MXC_FLC = (struct mxc_flc_regs *)*flc_base;

	printf(" > w%08x r%08x o%08x f%08x b%08x b%08x\n",
		(uint32_t)write_ptr, (uint32_t)read_ptr, (uint32_t)*options, (uint32_t)*flc_base,
		(uint32_t)enc_buffer, (uint32_t)(enc_buffer + 256));

	if (*options & OPTIONS_ENC) {
		/* Enable Memory Protection */
		MXC_GCR->scon |= MXC_F_GCR_SCON_MEMPROT_EN;

		/* Set the keysize */
		if (*options & OPTIONS_KEYSIZE)
			MXC_GCR->scon |= MXC_F_GCR_SCON_MEMPROT_KEYSZ;
		else
			MXC_GCR->scon &= ~(MXC_F_GCR_SCON_MEMPROT_KEYSZ);
	} else {
		/* Disable memory protection */
		MXC_GCR->scon &= ~MXC_F_GCR_SCON_MEMPROT_EN;
	}

	if (*options & OPTIONS_ENC) {
		/* Setup the AES */

		/* Enable CRYPTO clock */
		if ((MXC_GCR->clkcn & MXC_F_GCR_CLKCN_HIRC_EN) == 0)
			MXC_GCR->clkcn |= MXC_F_GCR_CLKCN_HIRC_EN;

		/* Disable CRYPTO clock gate */
		if (MXC_GCR->perckcn0 & MXC_F_GCR_PERCKCN0_CRYPTOD)
			MXC_GCR->perckcn0 &= ~(MXC_F_GCR_PERCKCN0_CRYPTOD);

		/* Reset Crypto block and clear state */
		MXC_TPU->ctrl = MXC_F_TPU_CTRL_RST;

		/* Set the legacy bit */
		MXC_TPU->ctrl |= MXC_F_TPU_CTRL_FLAG_MODE;

		/* Byte swap the input and output */
		MXC_TPU->ctrl |= MXC_F_TPU_CTRL_BSO;
		MXC_TPU->ctrl |= MXC_F_TPU_CTRL_BSI;
	}

	while (len) {
		if ((*options & OPTIONS_128) == 0) {
			/* Save the current address before we read from the working area */
			addr_save = addr;

			/* 32-bit write */
			MXC_FLC->cn |= MXC_F_FLC_CN_WDTH;

			enc_buffer[0] = 0;
			for (i = 0; i < 4; i++) {
				/* Get data from the working area, pad with 0xFF */
				if (len) {
					getbyte(temp8);
					__asm("nop\n");
				} else {
					temp8 = 0xFF;
					__asm("nop\n");
				}
				enc_buffer[0] |= (temp8 << (i * 8));
			}

			/* 32-bit write */
			MXC_FLC->cn |= MXC_F_FLC_CN_WDTH;

			MXC_FLC->addr = addr_save;
			MXC_FLC->data[0] = enc_buffer[0];

			/* Enable the write */
			MXC_FLC->cn |= MXC_F_FLC_CN_WR;

			/* Wait for the operation to complete */
			do {} while (MXC_FLC->cn & MXC_F_FLC_CN_WR);

			/* Check access violations */
			if (MXC_FLC->intr & MXC_F_FLC_INTR_AF) {
				MXC_FLC->intr &= ~MXC_F_FLC_INTR_AF;
				#ifndef ALGO_TEST
				#ifdef __riscv
				__asm("ebreak\n");
				#else
				__asm("bkpt\n");
				#endif
				#else
				printf(" > Error writing to flash\n");
				return;
				#endif
			}
		} else {
			/* Save the current address before we read from the working area */
			addr_save = addr;

			/* Fill the buffer with the plain text data from the working area */
			for (i = 0; i < 4; i++) {
				/* Get data from the working area, pad with 0xFF */
				enc_buffer[i] = 0;
				if (len) {
					getbyte(temp8);
					__asm("nop\n");
				} else {
					temp8 = 0xFF;
					__asm("nop\n");
				}
				enc_buffer[i] |= (temp8 << (0));
				/* Get data from the working area, pad with 0xFF */
				if (len) {
					getbyte(temp8);
					__asm("nop\n");
				} else {
					temp8 = 0xFF;
					__asm("nop\n");
				}
				enc_buffer[i] |= (temp8 << (8));
				/* Get data from the working area, pad with 0xFF */
				if (len) {
					getbyte(temp8);
					__asm("nop\n");
				} else {
					temp8 = 0xFF;
					__asm("nop\n");
				}
				enc_buffer[i] |= (temp8 << (16));
				/* Get data from the working area, pad with 0xFF */
				if (len) {
					getbyte(temp8);
					__asm("nop\n");
				} else {
					temp8 = 0xFF;
					__asm("nop\n");
				}
				enc_buffer[i] |= (temp8 << (24));
			}

			if (*options & OPTIONS_ENC) {
				/* XOR data with the address */
				for (i = 0; i < 4; i++) {
					if (*options & OPTIONS_RELATIVE_XOR)
						enc_buffer[i] ^= ((addr_save & 0x00FFFFFF) + i * 4);
					else
						enc_buffer[i] ^= (addr_save + i * 4);
				}

				/* Encrypt the plain text
				 * Clear interrupt flags*/
				MXC_TPU->ctrl |= MXC_F_TPU_CTRL_CPH_DONE;

				MXC_TPU->cipher_ctrl = ((0x0 << MXC_F_TPU_CIPHER_CTRL_MODE_POS) |
					(0x0 << MXC_F_TPU_CIPHER_CTRL_ENC_POS));

				if (*options & OPTIONS_KEYSIZE) {
					/* ECB, AES-256, encrypt */
					MXC_TPU->cipher_ctrl |=
						(0x3 << MXC_F_TPU_CIPHER_CTRL_CIPHER_POS);
				} else {
					/* ECB, AES-128, encrypt */
					MXC_TPU->cipher_ctrl |=
						(0x1 << MXC_F_TPU_CIPHER_CTRL_CIPHER_POS);
				}

				/* Set the key source */
				MXC_TPU->cipher_ctrl =
					((MXC_TPU->cipher_ctrl & ~MXC_F_TPU_CIPHER_CTRL_SRC) |
					(0x3 << MXC_F_TPU_CIPHER_CTRL_SRC_POS));

				/* Copy data to start the operation */
				MXC_TPU->din[0] = enc_buffer[0];
				MXC_TPU->din[1] = enc_buffer[1];
				MXC_TPU->din[2] = enc_buffer[2];
				MXC_TPU->din[3] = enc_buffer[3];

				/* Wait until operation is complete */
				do {} while (!(MXC_TPU->ctrl & MXC_F_TPU_CTRL_CPH_DONE));

				/* Copy the data out */
				enc_buffer[0] = MXC_TPU->dout[0];
				enc_buffer[1] = MXC_TPU->dout[1];
				enc_buffer[2] = MXC_TPU->dout[2];
				enc_buffer[3] = MXC_TPU->dout[3];
			}

			/* 128-bit write */
			MXC_FLC->cn &= ~MXC_F_FLC_CN_WDTH;

			MXC_FLC->addr = addr_save;
			MXC_FLC->data[0] = enc_buffer[0];
			MXC_FLC->data[1] = enc_buffer[1];
			MXC_FLC->data[2] = enc_buffer[2];
			MXC_FLC->data[3] = enc_buffer[3];

			/* Enable the write */
			MXC_FLC->cn |= MXC_F_FLC_CN_WR;

			/* Wait for the operation to complete */
			do {} while (MXC_FLC->cn & MXC_F_FLC_CN_WR);

			/* Check access violations */
			if (MXC_FLC->intr & MXC_F_FLC_INTR_AF) {
				MXC_FLC->intr &= ~MXC_F_FLC_INTR_AF;
				#ifndef ALGO_TEST
				#ifdef __riscv
				__asm("ebreak\n");
				#else
				__asm("bkpt\n");
				#endif
				printf(" > Error writing to flash\n");
				return;
				#endif
			}
		}
	}

	#ifndef ALGO_TEST
	#ifdef __riscv
	__asm("ebreak\n");
	#else
	__asm("bkpt\n");
	#endif
	#else
	printf(" > %s returning\n", __func__);
	return;
	#endif
}
