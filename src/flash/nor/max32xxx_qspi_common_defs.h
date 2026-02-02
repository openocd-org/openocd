/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2026 Analog Devices, Inc.                               *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NOR_MAX32XXX_QSPI_COMMON_DEFS_H
#define OPENOCD_FLASH_NOR_MAX32XXX_QSPI_COMMON_DEFS_H

#include <stddef.h>
#include <stdint.h>

#define SPIXFC_BASE 0x40027000
#define SPIXFC_CFG (SPIXFC_BASE | 0x00)
#define SPIXFC_SS_POL (SPIXFC_BASE | 0x04)
#define SPIXFC_GEN_CTRL (SPIXFC_BASE | 0x08)
#define SPIXFC_FIFO_CTRL (SPIXFC_BASE | 0x0C)

#define SPIXFC_CONFIG_PAGE_SIZE_POS 6
#define SPIXFC_CONFIG_PAGE_SIZE (0x3 << SPIXFC_CONFIG_PAGE_SIZE_POS)
#define SPIXFC_CONFIG_PAGE_SIZE_4_BYTES (0x0 << SPIXFC_CONFIG_PAGE_SIZE_POS)
#define SPIXFC_CONFIG_PAGE_SIZE_8_BYTES (0x1 << SPIXFC_CONFIG_PAGE_SIZE_POS)
#define SPIXFC_CONFIG_PAGE_SIZE_16_BYTES (0x2 << SPIXFC_CONFIG_PAGE_SIZE_POS)
#define SPIXFC_CONFIG_PAGE_SIZE_32_BYTES (0x3 << SPIXFC_CONFIG_PAGE_SIZE_POS)

#define SPIXFC_FIFO_CTRL_TX_FIFO_CNT_POS 8
#define SPIXFC_FIFO_CTRL_TX_FIFO_CNT (0x1FUL << SPIXFC_FIFO_CTRL_TX_FIFO_CNT_POS)

#define SPIXFC_FIFO_CTRL_RX_FIFO_CNT_POS 24
#define SPIXFC_FIFO_CTRL_RX_FIFO_CNT (0x3FUL << SPIXFC_FIFO_CTRL_RX_FIFO_CNT_POS)

#define SPIXFC_FIFO_TX 0x400BC000
#define SPIXFC_FIFO_RX 0x400BC004

#define SPIXFC_HEADER_TX 0x1
#define SPIXFC_HEADER_RX 0x2
#define SPIXFC_HEADER_BIT (0x0 << 2)
#define SPIXFC_HEADER_BYTE (0x1 << 2)
#define SPIXFC_HEADER_PAGE (0x2 << 2)
#define SPIXFC_HEADER_SIZE_POS 4
#define SPIXFC_HEADER_WIDTH_POS 9
#define SPIXFC_HEADER_SS_DEASS (0x1 << 13)

#define SPI_WRITE_BOUNDARY 256

#define SPIX_ALGO_STACK_SIZE 256U

/*
 * Shared host/loader working-area layout:
 *   [header | fifo payload ... | tail]
 *
 * header:
 *   write_ptr (ring write pointer)
 *   read_ptr  (ring read pointer)
 *
 * tail:
 *   spi_write_cmd
 *   options
 *   stack (reserved downward-growing stack space for loader code)
 */

struct max32xxx_qspi_algo_header {
	uint32_t write_ptr;
	uint32_t read_ptr;
};

struct max32xxx_qspi_algo_tail {
	uint32_t spi_write_cmd;
	uint32_t options;
	uint8_t stack[SPIX_ALGO_STACK_SIZE];
};

#define SPIX_ALGO_HEADER_SIZE ((uint32_t)sizeof(struct max32xxx_qspi_algo_header))
#define SPIX_ALGO_TAIL_SIZE ((uint32_t)sizeof(struct max32xxx_qspi_algo_tail))

#define SPIX_ALGO_WRITE_PTR_OFFSET ((uint32_t)offsetof(struct max32xxx_qspi_algo_header, write_ptr))
#define SPIX_ALGO_READ_PTR_OFFSET ((uint32_t)offsetof(struct max32xxx_qspi_algo_header, read_ptr))

#define SPIX_ALGO_SPI_WRITE_CMD_OFFSET ((uint32_t)offsetof(struct max32xxx_qspi_algo_tail, spi_write_cmd))
#define SPIX_ALGO_OPTIONS_OFFSET ((uint32_t)offsetof(struct max32xxx_qspi_algo_tail, options))

#endif /* OPENOCD_FLASH_NOR_MAX32XXX_QSPI_COMMON_DEFS_H */
