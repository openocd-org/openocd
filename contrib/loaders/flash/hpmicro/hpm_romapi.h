/* SPDX-License-Identifier: BSD-3-Clause */

/*
 * Copyright (c) 2021-2023 HPMicro
 */

#ifndef HPM_ROMAPI_H
#define HPM_ROMAPI_H

/**
 * @brief ROM APIs
 * @defgroup romapi_interface ROM APIs
 * @{
 */

#include "hpm_common.h"
#include "hpm_romapi_xpi_def.h"
#include "hpm_romapi_xpi_soc_def.h"
#include "hpm_romapi_xpi_nor_def.h"

/***********************************************************************************************************************
 *
 *
 *	  Definitions
 *
 *
 **********************************************************************************************************************/

/**
 * @brief Bootloader API table
 */
struct bootloader_api_table_t {
	/**< Bootloader API table: version */
	const uint32_t version;
	/**< Bootloader API table: copyright string address */
	const char *copyright;
	/**< Bootloader API table: run_bootloader API */
	const uint32_t reserved0;
	/**< Bootloader API table: otp driver interface address */
	const uint32_t reserved1;
	/**< Bootloader API table: xpi driver interface address */
	const struct xpi_driver_interface_t *xpi_driver_if;
	/**< Bootloader API table: xpi nor driver interface address */
	const struct xpi_nor_driver_interface_t *xpi_nor_driver_if;
	/**< Bootloader API table: xpi ram driver interface address */
	const uint32_t reserved2;
};

/**< Bootloader API table Root */
#define ROM_API_TABLE_ROOT ((const struct bootloader_api_table_t *)0x2001FF00U)

#endif /* HPM_ROMAPI_H */
