/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Copyright (C) 2020 by Nuvoton Technology Corporation
 * Mulin Chao <mlchao@nuvoton.com>
 * Wealian Liao <WHLIAO@nuvoton.com>
 */

#ifndef OPENOCD_LOADERS_FLASH_NPCX_NPCX_FLASH_H
#define OPENOCD_LOADERS_FLASH_NPCX_NPCX_FLASH_H

#include "npcx_flash_config.h"

/* Bit functions */
#define NPCX_SET_BIT(reg, bit)           ((reg) |= (0x1 << (bit)))
#define NPCX_CLEAR_BIT(reg, bit)         ((reg) &= (~(0x1 << (bit))))
#define NPCX_IS_BIT_SET(reg, bit)        (((reg) >> (bit)) & (0x1))

/* Field functions */
#define NPCX_GET_POS_FIELD(pos, size)    (pos)
#define NPCX_GET_SIZE_FIELD(pos, size)   (size)
#define NPCX_FIELD_POS(field)            NPCX_GET_POS_##field
#define NPCX_FIELD_SIZE(field)           NPCX_GET_SIZE_##field
/* Read field functions */
#define NPCX_GET_FIELD(reg, field) \
	_NPCX_GET_FIELD_((reg), NPCX_FIELD_POS(field), NPCX_FIELD_SIZE(field))
#define _NPCX_GET_FIELD_(reg, f_pos, f_size) \
	(((reg) >> (f_pos)) & ((1 << (f_size)) - 1))
/* Write field functions */
#define NPCX_SET_FIELD(reg, field, value) \
	_NPCX_SET_FIELD_((reg), NPCX_FIELD_POS(field), NPCX_FIELD_SIZE(field), (value))
#define _NPCX_SET_FIELD_(reg, f_pos, f_size, value) \
	((reg) = ((reg) & (~(((1 << (f_size)) - 1) << (f_pos)))) | ((value) << (f_pos)))

/* Register definitions */
#define NPCX_REG32_ADDR(addr)            ((volatile uint32_t *)(addr))
#define NPCX_REG16_ADDR(addr)            ((volatile uint16_t *)(addr))
#define NPCX_REG8_ADDR(addr)             ((volatile uint8_t  *)(addr))

#define NPCX_HW_BYTE(addr)               (*NPCX_REG8_ADDR(addr))
#define NPCX_HW_WORD(addr)               (*NPCX_REG16_ADDR(addr))
#define NPCX_HW_DWORD(addr)              (*NPCX_REG32_ADDR(addr))

/* Devalt */
#define NPCX_SCFG_BASE_ADDR  0x400C3000
#define NPCX_DEVCNT          NPCX_HW_BYTE(NPCX_SCFG_BASE_ADDR + 0x000)
#define NPCX_DEVALT(n)	     NPCX_HW_BYTE(NPCX_SCFG_BASE_ADDR + 0x010 + (n))

#define NPCX_DEVCNT_HIF_TYP_SEL_FIELD    FIELD(2, 2)
#define NPCX_DEVCNT_JEN0_HEN             4
#define NPCX_DEVCNT_JEN1_HEN             5
#define NPCX_DEVCNT_F_SPI_TRIS           6

/* Pin-mux for SPI/FIU */
#define NPCX_DEVALT0_SPIP_SL             0
#define NPCX_DEVALT0_GPIO_NO_SPIP        3
#define NPCX_DEVALT0_F_SPI_CS1_2         4
#define NPCX_DEVALT0_F_SPI_CS1_1         5
#define NPCX_DEVALT0_F_SPI_QUAD          6
#define NPCX_DEVALT0_NO_F_SPI            7

/* Flash Interface Unit (FIU) registers */
#define NPCX_FIU_BASE_ADDR   0x40020000
#define NPCX_FIU_CFG         NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x000)
#define NPCX_BURST_CFG       NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x001)
#define NPCX_RESP_CFG        NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x002)
#define NPCX_SPI_FL_CFG      NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x014)
#define NPCX_UMA_CODE        NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x016)
#define NPCX_UMA_AB0         NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x017)
#define NPCX_UMA_AB1         NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x018)
#define NPCX_UMA_AB2         NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x019)
#define NPCX_UMA_DB0         NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x01A)
#define NPCX_UMA_DB1         NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x01B)
#define NPCX_UMA_DB2         NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x01C)
#define NPCX_UMA_DB3         NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x01D)
#define NPCX_UMA_CTS         NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x01E)
#define NPCX_UMA_ECTS        NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x01F)
#define NPCX_UMA_DB0_3      NPCX_HW_DWORD(NPCX_FIU_BASE_ADDR + 0x020)
#define NPCX_FIU_RD_CMD      NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x030)
#define NPCX_FIU_DMM_CYC     NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x032)
#define NPCX_FIU_EXT_CFG     NPCX_HW_BYTE(NPCX_FIU_BASE_ADDR + 0x033)
#define NPCX_FIU_UMA_AB0_3  NPCX_HW_DWORD(NPCX_FIU_BASE_ADDR + 0x034)

/* FIU register fields */
#define NPCX_RESP_CFG_IAD_EN             0
#define NPCX_RESP_CFG_DEV_SIZE_EX        2
#define NPCX_UMA_CTS_A_SIZE              3
#define NPCX_UMA_CTS_C_SIZE              4
#define NPCX_UMA_CTS_RD_WR               5
#define NPCX_UMA_CTS_DEV_NUM             6
#define NPCX_UMA_CTS_EXEC_DONE           7
#define NPCX_UMA_ECTS_SW_CS0             0
#define NPCX_UMA_ECTS_SW_CS1             1
#define NPCX_UMA_ECTS_SEC_CS             2
#define NPCX_UMA_ECTS_UMA_LOCK           3

/* Flash UMA commands for npcx internal SPI flash */
#define NPCX_CMD_READ_ID                 0x9F
#define NPCX_CMD_READ_MAN_DEV_ID         0x90
#define NPCX_CMD_WRITE_EN                0x06
#define NPCX_CMD_WRITE_STATUS            0x50
#define NPCX_CMD_READ_STATUS_REG         0x05
#define NPCX_CMD_READ_STATUS_REG2        0x35
#define NPCX_CMD_WRITE_STATUS_REG        0x01
#define NPCX_CMD_FLASH_PROGRAM           0x02
#define NPCX_CMD_SECTOR_ERASE            0x20
#define NPCX_CMD_PROGRAM_UINT_SIZE       0x08
#define NPCX_CMD_PAGE_SIZE               0x00
#define NPCX_CMD_READ_ID_TYPE            0x47
#define NPCX_CMD_FAST_READ               0x0B
#define NPCX_CMD_CHIP_ERASE              0xC7

/*
 * Status registers for SPI flash
 */
#define NPCX_SPI_FLASH_SR2_SUS           (1 << 7)
#define NPCX_SPI_FLASH_SR2_CMP           (1 << 6)
#define NPCX_SPI_FLASH_SR2_LB3           (1 << 5)
#define NPCX_SPI_FLASH_SR2_LB2           (1 << 4)
#define NPCX_SPI_FLASH_SR2_LB1           (1 << 3)
#define NPCX_SPI_FLASH_SR2_QE            (1 << 1)
#define NPCX_SPI_FLASH_SR2_SRP1          (1 << 0)
#define NPCX_SPI_FLASH_SR1_SRP0          (1 << 7)
#define NPCX_SPI_FLASH_SR1_SEC           (1 << 6)
#define NPCX_SPI_FLASH_SR1_TB            (1 << 5)
#define NPCX_SPI_FLASH_SR1_BP2           (1 << 4)
#define NPCX_SPI_FLASH_SR1_BP1           (1 << 3)
#define NPCX_SPI_FLASH_SR1_BP0           (1 << 2)
#define NPCX_SPI_FLASH_SR1_WEL           (1 << 1)
#define NPCX_SPI_FLASH_SR1_BUSY          (1 << 0)

#define NPCX_MASK_CMD_ONLY               (0xC0)
#define NPCX_MASK_CMD_ADR                (0xC0 | 0x08)
#define NPCX_MASK_CMD_ADR_WR             (0xC0 | 0x20 | 0x08 | 0x01)
#define NPCX_MASK_RD_1BYTE               (0xC0 | 0x10 | 0x01)
#define NPCX_MASK_RD_2BYTE               (0xC0 | 0x10 | 0x02)
#define NPCX_MASK_RD_3BYTE               (0xC0 | 0x10 | 0x03)
#define NPCX_MASK_RD_4BYTE               (0xC0 | 0x10 | 0x04)
#define NPCX_MASK_CMD_RD_1BYTE           (0xC0 | 0x01)
#define NPCX_MASK_CMD_RD_2BYTE           (0xC0 | 0x02)
#define NPCX_MASK_CMD_RD_3BYTE           (0xC0 | 0x03)
#define NPCX_MASK_CMD_RD_4BYTE           (0xC0 | 0x04)
#define NPCX_MASK_CMD_WR_ONLY            (0xC0 | 0x20)
#define NPCX_MASK_CMD_WR_1BYTE           (0xC0 | 0x20 | 0x10 | 0x01)
#define NPCX_MASK_CMD_WR_2BYTE           (0xC0 | 0x20 | 0x10 | 0x02)
#define NPCX_MASK_CMD_WR_ADR             (0xC0 | 0x20 | 0x08)

/* Flash loader parameters */
struct __attribute__((__packed__)) npcx_flash_params {
	uint32_t addr; /* Address in flash */
	uint32_t len;  /* Number of bytes */
	uint32_t cmd;  /* Command */
	uint32_t sync; /* Handshake signal */
};

/* Flash trigger signal */
enum npcx_flash_handshake {
	NPCX_FLASH_LOADER_WAIT    = 0x0,       /* Idle */
	NPCX_FLASH_LOADER_EXECUTE = 0xFFFFFFFF /* Execute Command */
};

/* Flash loader command */
enum npcx_flash_commands {
	NPCX_FLASH_CMD_NO_ACTION = 0, /* No action, default value */
	NPCX_FLASH_CMD_GET_FLASH_ID,  /* Get the internal flash ID */
	NPCX_FLASH_CMD_ERASE_SECTORS, /* Erase unprotected sectors */
	NPCX_FLASH_CMD_ERASE_ALL,     /* Erase all */
	NPCX_FLASH_CMD_PROGRAM,       /* Program data */
};

/* Status */
enum npcx_flash_status {
	NPCX_FLASH_STATUS_OK = 0,
	NPCX_FLASH_STATUS_FAILED_UNKNOWN_COMMAND,
	NPCX_FLASH_STATUS_FAILED,
	NPCX_FLASH_STATUS_FAILED_TIMEOUT,
};

#endif /* OPENOCD_LOADERS_FLASH_NPCX_NPCX_FLASH_H */
