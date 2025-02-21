// SPDX-License-Identifier: GPL-2.0-or-later
/**
 * @file
 * Driver for SPI NOR flash chips connected via DesignWare SPI Core.
 * Controller's Linux driver is located at drivers/spi/spi-dw-mmio.c.
 * DW-SPI is used in a number of processors, including Microsemi Jaguar2 and
 * Ocelot switch chips.
 *
 * Serial interface (SI) nCS0 pin, which is usually connected to the external
 * flash, cannot be controlled via GPIO controller: it is asserted only when
 * TX FIFO is not empty. Since JTAG is not fast enough to fill TX FIFO and
 * collect data from RX FIFO at the same time even on the slowest SPI clock
 * speeds, driver can only operate using functions, loaded in target's memory.
 * Therefore, it requires the user to set up DRAM controller and provide
 * work-area.
 *
 * In Microsemi devices, serial interface pins may be governed either
 * by Boot or Master controller. For these devices, additional configuration of
 * spi_mst address is required to switch between the two.
 *
 * Currently supported devices typically have much more RAM then NOR Flash
 * (Jaguar2 reference design has 256MB RAM and 32MB NOR Flash), so supporting
 * work-area sizes smaller then transfer buffer seems like the unnecessary
 * complication.
 *
 * This code was tested on Jaguar2 VSC7448 connected to Macronix MX25L25635F.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "dw-spi-helper.h"
#include "imp.h"
#include "spi.h"

#include <helper/bits.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/breakpoints.h>
#include <target/mips32.h>
#include <target/target_type.h>

/**
 * @brief IP block placement map.
 *
 * Used for dynamic definition of the register map.
 *
 * IP block is used on different chips and placed in different locations.
 * This structure defines some implementation specific variables.
 */
struct dw_spi_regmap {
	uint32_t freq; ///< Clock frequency.
	target_addr_t simc; ///< Absolute offset of SIMC register block.
	target_addr_t spi_mst;
	///< Absolute offset of ICPU_CFG:SPI_MST register. 0 if not available.
	uint8_t si_if_owner_offset;
	///< Offset of \ref si_mode bits in ICPU_CFG:SPI_MST.
};

/**
 * @brief Register map for Jaguar2 switch devices.
 */
static const struct dw_spi_regmap jaguar2_regmap = {
	.freq = 250000000UL,
	.simc = 0x70101000UL,
	.spi_mst = 0x70000024UL,
	.si_if_owner_offset = 6,
};

/**
 * @brief Register map for Ocelot switch devices.
 */
static const struct dw_spi_regmap ocelot_regmap = {
	.freq = 250000000UL,
	.simc = 0x70101000UL,
	.spi_mst = 0x70000024UL,
	.si_if_owner_offset = 4,
};

#define DW_SPI_IF_OWNER_WIDTH 2 ///< IF owner register field width.

/**
 * @brief Owner of the SI interface.
 */
enum dw_spi_si_mode {
	DW_SPI_SI_MODE_BOOT = 1,
	///< Boot controller maps contents of SPI Flash to memory in read-only mode.
	DW_SPI_SI_MODE_MASTER = 2,
	///< SPI controller mode for reading/writing SPI Flash.
};

#define DW_SPI_REG_CTRLR0 0x00 ///< General configuration register.
#define DW_SPI_REG_SIMCEN 0x08 ///< Master controller enable register.
#define DW_SPI_REG_SER 0x10 ///< Slave select register.
#define DW_SPI_REG_BAUDR 0x14 ///< Baud rate configuration register.
#define DW_SPI_REG_SR 0x28 ///< Status register.
#define DW_SPI_REG_IMR 0x2c ///< Interrupt configuration register.
#define DW_SPI_REG_DR 0x60 ///< Data register.

#define DW_SPI_REG_CTRLR0_DFS(x) ((x) & GENMASK(3, 0)) ///< Data frame size.
#define DW_SPI_REG_CTRLR0_FRF(x) (((x) << 4) & GENMASK(5, 4)) ///< SI protocol.
#define DW_SPI_REG_CTRLR0_SCPH(x) ((!!(x)) << 6) ///< Probe position.
#define DW_SPI_REG_CTRLR0_SCPOL(x) ((!!(x)) << 7) ///< Probe polarity.
#define DW_SPI_REG_CTRLR0_TMOD(x) (((x) << 8) & GENMASK(9, 8)) ///< SI mode.
#define DW_SPI_REG_SIMCEN_SIMCEN(x) (!!(x)) ///< Controller enable.
#define DW_SPI_REG_SER_SER(x) ((x) & GENMASK(15, 0)) ///< Slave select bitmask.
#define DW_SPI_REG_BAUDR_SCKDV(x) ((x) & GENMASK(15, 0)) ///< Clock divisor.

/**
 * @brief Driver private state.
 */
struct dw_spi_driver {
	bool probed; ///< Bank is probed.
	uint32_t id; ///< Chip ID.
	unsigned int speed; ///< Flash speed.
	unsigned int timeout; ///< Flash timeout in milliseconds.
	uint8_t chip_select_bitmask; ///< Chip select bitmask.
	bool four_byte_mode; ///< Flash chip is in 32bit address mode.
	enum dw_spi_si_mode saved_ctrl_mode;
	///< Previously selected controller mode.
	struct dw_spi_regmap regmap; ///< SI controller regmap.
	const struct flash_device *spi_flash; ///< SPI flash device info.
};

/**
 * @brief Register used to pass argument struct to helper functions.
 */
#define DW_SPI_ARG_REG "r4"

/**
 * @brief Default timeout value in ms for flash transaction jobs.
 */
#define DW_SPI_TIMEOUT_DEFAULT (600 * 1000)

/**
 * @brief Timeout value in ms for short flash transactions,
 * e.g. reading flash ID and status register.
 */
#define DW_SPI_TIMEOUT_TRANSACTION 1000

/**
 * @brief Select SI interface owner.
 *
 * Mode selection is skipped if Boot controller not available on target
 * (i.e. spi_mst command argument is not configured).
 *
 * @param[in] bank: Flash bank.
 * @param[in] mode: New controller mode.
 * @return Command execution status.
 */
static int
dw_spi_ctrl_mode(const struct flash_bank *const bank, enum dw_spi_si_mode mode)
{
	struct target *const target = bank->target;
	const struct dw_spi_driver *const driver = bank->driver_priv;
	const struct dw_spi_regmap *const regmap = &driver->regmap;

	if (!regmap->spi_mst)
		return ERROR_OK;

	uint32_t ctrl;
	int ret = target_read_u32(target, regmap->spi_mst, &ctrl);
	if (ret) {
		LOG_ERROR("DW SPI SPI:MST register read error");
		return ret;
	}
	ctrl &= ~GENMASK(DW_SPI_IF_OWNER_WIDTH + driver->regmap.si_if_owner_offset,
					 driver->regmap.si_if_owner_offset);
	ctrl |= mode << driver->regmap.si_if_owner_offset;

	ret = target_write_u32(target, regmap->spi_mst, ctrl);
	if (ret)
		LOG_ERROR("DW SPI controller mode configuration error");

	return ret;
}

/**
 * @brief Select master controller as SI interface owner.
 *
 * Previous interface owner is restored via dw_spi_ctrl_mode_restore() function.
 * Mode selection is skipped if Boot controller not available on target
 * (i.e. spi_mst command argument is not configured).
 *
 * @param[in] bank: Flash bank.
 * @param[in] mode: New controller mode.
 * @return Command execution status.
 */
static int
dw_spi_ctrl_mode_configure(const struct flash_bank *const bank,
						   enum dw_spi_si_mode mode)
{
	struct target *const target = bank->target;
	struct dw_spi_driver *const driver = bank->driver_priv;
	const struct dw_spi_regmap *const regmap = &driver->regmap;

	if (!regmap->spi_mst)
		return ERROR_OK;

	uint32_t ctrl;
	int ret = target_read_u32(target, regmap->spi_mst, &ctrl);
	if (ret) {
		LOG_ERROR("DW SPI controller mode query error");
		return ret;
	}
	driver->saved_ctrl_mode =
		(enum dw_spi_si_mode)((ctrl >> driver->regmap.si_if_owner_offset) &
							  GENMASK(DW_SPI_IF_OWNER_WIDTH, 0));

	return dw_spi_ctrl_mode(bank, mode);
}

/**
 * @brief Restore SI controller mode.
 *
 * Restore initially configured SI controller mode. Undo configuration done by
 * dw_spi_ctrl_mode_configure() function.
 * Mode selection is skipped if Boot controller not available on target
 * (i.e. spi_mst command argument is not configured).
 *
 * @param[in] bank: Flash bank.
 * @return Command execution status.
 */
static int
dw_spi_ctrl_mode_restore(const struct flash_bank *const bank)
{
	const struct dw_spi_driver *const driver = bank->driver_priv;

	return dw_spi_ctrl_mode(bank, driver->saved_ctrl_mode);
}

/**
 * @brief Enable master controller.
 *
 * Configuration of the master controller must be done when it is disabled.
 *
 * @param[in] bank: Flash bank.
 * @param[in] value: New enable state.
 * @return Command execution status.
 */
static int
dw_spi_master_ctrl_enable(const struct flash_bank *const bank, bool value)
{
	struct target *const target = bank->target;
	const struct dw_spi_driver *const driver = bank->driver_priv;
	const struct dw_spi_regmap *const regmap = &driver->regmap;

	int ret = target_write_u32(target, regmap->simc + DW_SPI_REG_SIMCEN,
							   DW_SPI_REG_SIMCEN_SIMCEN(value));
	if (ret)
		LOG_ERROR("DW SPI master controller enable flag configuration error");

	return ret;
}

/**
 * @brief Configure SI transfer mode.
 *
 * @param[in] bank: Flash bank.
 * @return Command execution status.
 */
static int
dw_spi_ctrl_configure_si(const struct flash_bank *const bank)
{
	struct target *const target = bank->target;
	const struct dw_spi_driver *const driver = bank->driver_priv;
	const struct dw_spi_regmap *const regmap = &driver->regmap;

	// 8 bit frame; Motorola protocol; middle lo probe; TX RX mode
	const uint32_t mode = DW_SPI_REG_CTRLR0_DFS(0x7) |
						  DW_SPI_REG_CTRLR0_FRF(0) |
						  DW_SPI_REG_CTRLR0_SCPH(0) |
						  DW_SPI_REG_CTRLR0_SCPOL(0) |
						  DW_SPI_REG_CTRLR0_TMOD(0);

	int ret = target_write_u32(target, regmap->simc + DW_SPI_REG_CTRLR0, mode);
	if (ret) {
		LOG_ERROR("DW SPI master controller configuration query error");
		return ret;
	}

	ret = target_write_u32(target, regmap->simc + DW_SPI_REG_SER,
						   DW_SPI_REG_SER_SER(driver->chip_select_bitmask));
	if (ret)
		LOG_ERROR("DW SPI slave select configuration error");

	return ret;
}

/**
 * @brief Configure SI transfer speed.
 *
 * @param[in] bank: Flash bank.
 * @return Command execution status.
 */
static int
dw_spi_ctrl_configure_speed(const struct flash_bank *const bank)
{
	struct target *const target = bank->target;
	const struct dw_spi_driver *const driver = bank->driver_priv;
	const struct dw_spi_regmap *const regmap = &driver->regmap;

	// divisor LSB must be zero
	const uint16_t div = MIN((regmap->freq / driver->speed), 0xfffe) & 0xfffe;

	int ret = target_write_u32(target, regmap->simc + DW_SPI_REG_BAUDR,
							   DW_SPI_REG_BAUDR_SCKDV(div));
	if (ret) {
		LOG_ERROR("DW SPI speed configuration error");
		return ret;
	}

	unsigned int speed = regmap->freq / div;
	LOG_DEBUG("DW SPI setting NOR controller speed to %u kHz", speed / 1000);

	return ret;
}

/**
 * @brief Disable SI master controller interrupts.
 *
 * @param[in] bank: Flash bank.
 * @return Command execution status.
 */
static int
dw_spi_ctrl_disable_interrupts(const struct flash_bank *const bank)
{
	struct target *const target = bank->target;
	const struct dw_spi_driver *const driver = bank->driver_priv;
	const struct dw_spi_regmap *const regmap = &driver->regmap;

	int ret = target_write_u32(target, regmap->simc + DW_SPI_REG_IMR, 0);
	if (ret)
		LOG_ERROR("DW SPI disable interrupts error");

	return ret;
}

/**
 * @brief Do data transaction.
 *
 * Buffer data are sent and replaced with received data. Total buffer size
 * is a sum of TX and RX messages. RX portion of the buffer is initially
 * filled with dummy values, which get replaced by the message.
 *
 * @param[in] bank: Flash bank.
 * @param[in,out] buffer: Data buffer. If \p read flag is set, buffer is
 * filled with received data.
 * @param[in] size: \p buffer size.
 * @param[in] read: The read flag. If set to true, read values will override
 * \p buffer.
 * @return Command execution status.
 */
static int
dw_spi_ctrl_transaction(const struct flash_bank *const bank,
						uint8_t *const buffer, size_t size, bool read)
{
	struct target *const target = bank->target;
	const struct dw_spi_driver *const driver = bank->driver_priv;
	const struct dw_spi_regmap *const regmap = &driver->regmap;

	static const uint8_t target_code[] = {
#include "../../../contrib/loaders/flash/dw-spi/mipsel-linux-gnu-transaction.inc"
	};
	const size_t target_code_size = sizeof(target_code);
	const size_t total_working_area_size =
		target_code_size + sizeof(struct dw_spi_transaction) + size;

	// allocate working area, memory args and data buffer
	struct working_area *helper;
	int ret = target_alloc_working_area(target, target_code_size, &helper);
	if (ret) {
		LOG_ERROR("DW SPI could not allocate working area. Need %zx",
				  total_working_area_size);
		goto err_helper;
	}

	struct working_area *helper_args;
	ret = target_alloc_working_area(target, sizeof(struct dw_spi_transaction),
									&helper_args);
	if (ret) {
		LOG_ERROR("DW SPI could not allocate working area. Need %zx",
				  total_working_area_size);
		goto err_helper_args;
	}

	struct working_area *target_buffer;
	ret = target_alloc_working_area(target, size, &target_buffer);
	if (ret) {
		LOG_ERROR("DW SPI could not allocate working area. Need %zx",
				  total_working_area_size);
		goto err_target_buffer;
	}

	// write algorithm code and buffer to working areas
	ret = target_write_buffer(target, helper->address, target_code_size,
							  target_code);
	if (ret) {
		LOG_ERROR("DW SPI writing to working area error");
		goto err_write_buffer;
	}

	ret = target_write_buffer(target, target_buffer->address, size, buffer);
	if (ret) {
		LOG_ERROR("DW SPI writing to working area error");
		goto err_write_buffer;
	}

	// prepare helper execution
	struct mips32_algorithm mips32_algo = { .common_magic = MIPS32_COMMON_MAGIC,
											.isa_mode = MIPS32_ISA_MIPS32 };

	struct reg_param reg_param;
	init_reg_param(&reg_param, DW_SPI_ARG_REG, 32, PARAM_OUT);
	struct mem_param mem_param;
	init_mem_param(&mem_param, helper_args->address, helper_args->size,
				   PARAM_OUT);

	// Set the arguments for the helper
	buf_set_u32(reg_param.value, 0, 32, helper_args->address);

	struct dw_spi_transaction *helper_args_val =
		(struct dw_spi_transaction *)mem_param.value;
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->buffer,
						  target_buffer->address);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->size, size);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->status_reg,
						  regmap->simc + DW_SPI_REG_SR);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->data_reg,
						  regmap->simc + DW_SPI_REG_DR);
	helper_args_val->read_flag = read;

	ret = target_run_algorithm(target, 1, &mem_param, 1, &reg_param,
							   helper->address, 0, DW_SPI_TIMEOUT_TRANSACTION,
							   &mips32_algo);

	if (ret) {
		LOG_ERROR("DW SPI flash algorithm error");
		goto cleanup;
	}

	if (read) {
		ret = target_read_buffer(target, target_buffer->address, size, buffer);
		if (ret)
			LOG_ERROR("DW SPI target buffer read error");
	}

cleanup:
	destroy_reg_param(&reg_param);
	destroy_mem_param(&mem_param);

err_write_buffer:
	target_free_working_area(target, target_buffer);
err_target_buffer:
	target_free_working_area(target, helper_args);
err_helper_args:
	target_free_working_area(target, helper);
err_helper:

	return ret;
}

/**
 * @brief Check that selected region is filled with pattern.
 *
 * This function is used for Flash erase checking.
 *
 * @param[in] bank: Flash bank.
 * @param[in] address: Starting address. Sector aligned.
 * @param[in] sector_size: Size of sector.
 * @param[in] sector_count: Number of sectors.
 * @param[in] pattern: Fill pattern.
 * @param[in] read_cmd: Flash read command.
 * @param[out] buffer: Filled flag array. Must hold \p sector_count number
 * of entries.
 * @return Command execution status.
 */
static int
dw_spi_ctrl_check_sectors_fill(const struct flash_bank *const bank,
							   uint32_t address, size_t sector_size,
							   size_t sector_count, uint8_t pattern,
							   uint8_t read_cmd, uint8_t *buffer)
{
	struct target *const target = bank->target;
	const struct dw_spi_driver *const driver = bank->driver_priv;
	const struct dw_spi_regmap *const regmap = &driver->regmap;

	static const uint8_t target_code[] = {
#include "../../../contrib/loaders/flash/dw-spi/mipsel-linux-gnu-check_fill.inc"
	};
	const size_t target_code_size = sizeof(target_code);
	const size_t total_working_area_size =
		target_code_size + sizeof(struct dw_spi_check_fill) + sector_count;

	// allocate working area, memory args and data buffer
	struct working_area *helper;
	int ret = target_alloc_working_area(target, target_code_size, &helper);
	if (ret) {
		LOG_ERROR("DW SPI could not allocate working area. Need %zx",
				  total_working_area_size);
		goto err_helper;
	}

	struct working_area *helper_args;
	ret = target_alloc_working_area(target, sizeof(struct dw_spi_check_fill),
									&helper_args);
	if (ret) {
		LOG_ERROR("DW SPI could not allocate working area. Need %zx",
				  total_working_area_size);
		goto err_helper_args;
	}

	struct working_area *target_buffer;
	ret = target_alloc_working_area(target, sector_count, &target_buffer);
	if (ret) {
		LOG_ERROR("DW SPI could not allocate working area. Need %zx",
				  total_working_area_size);
		goto err_target_buffer;
	}

	// write algorithm code and buffer to working areas
	ret = target_write_buffer(target, helper->address, target_code_size,
							  target_code);
	if (ret) {
		LOG_ERROR("DW SPI writing to working area error");
		goto err_write_buffer;
	}

	// prepare helper execution
	struct mips32_algorithm mips32_algo = { .common_magic = MIPS32_COMMON_MAGIC,
											.isa_mode = MIPS32_ISA_MIPS32 };

	struct reg_param reg_param;
	init_reg_param(&reg_param, DW_SPI_ARG_REG, 32, PARAM_OUT);
	struct mem_param mem_param;
	init_mem_param(&mem_param, helper_args->address, helper_args->size,
				   PARAM_OUT);

	// Set the arguments for the helper
	buf_set_u32(reg_param.value, 0, 32, helper_args->address);

	struct dw_spi_check_fill *helper_args_val =
		(struct dw_spi_check_fill *)mem_param.value;
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->address,
						  address);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->sector_size,
						  sector_size);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->sector_count,
						  sector_count);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->status_reg,
						  regmap->simc + DW_SPI_REG_SR);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->data_reg,
						  regmap->simc + DW_SPI_REG_DR);
	target_buffer_set_u32(target,
						  (uint8_t *)&helper_args_val->fill_status_array,
						  target_buffer->address);
	helper_args_val->pattern = pattern;
	helper_args_val->read_cmd = read_cmd;
	helper_args_val->four_byte_mode = driver->four_byte_mode;

	ret = target_run_algorithm(target, 1, &mem_param, 1, &reg_param,
							   helper->address, 0, driver->timeout,
							   &mips32_algo);

	if (ret) {
		LOG_ERROR("DW SPI flash algorithm error");
		goto cleanup;
	}

	ret = target_read_buffer(target, target_buffer->address, sector_count,
							 buffer);
	if (ret)
		LOG_ERROR("DW SPI target buffer read error");

cleanup:
	destroy_reg_param(&reg_param);
	destroy_mem_param(&mem_param);

err_write_buffer:
	target_free_working_area(target, target_buffer);
err_target_buffer:
	target_free_working_area(target, helper_args);
err_helper_args:
	target_free_working_area(target, helper);
err_helper:

	return ret;
}

/**
 * @brief Write flash region.
 *
 * @param[in] bank: Flash bank.
 * @param[in] address: First page address. Page aligned when write is crossing
 *                     the page boundary.
 * @param[in] buffer: Data buffer.
 * @param[in] buffer_size: \p buffer size.
 * @param[in] page_size: Size of flash page.
 * @param[in] stat_cmd: Flash command to read chip status.
 * @param[in] we_cmd: Flash command to enable write.
 * @param[in] program_cmd: Flash command to program chip.
 * @param[in] we_mask: Status byte write enable mask.
 * @param[in] busy_mask: Status byte write busy mask.
 * @return Command execution status.
 */
static int
dw_spi_ctrl_program(const struct flash_bank *const bank, uint32_t address,
					const uint8_t *const buffer, size_t buffer_size,
					uint32_t page_size, uint8_t stat_cmd, uint8_t we_cmd,
					uint8_t program_cmd, uint8_t we_mask, uint8_t busy_mask)
{
	struct target *const target = bank->target;
	const struct dw_spi_driver *const driver = bank->driver_priv;
	const struct dw_spi_regmap *const regmap = &driver->regmap;

	static const uint8_t target_code[] = {
#include "../../../contrib/loaders/flash/dw-spi/mipsel-linux-gnu-program.inc"
	};
	const size_t target_code_size = sizeof(target_code);
	const size_t total_working_area_size =
		target_code_size + sizeof(struct dw_spi_program) + buffer_size;

	// allocate working area, memory args and data buffer
	struct working_area *helper;
	int ret = target_alloc_working_area(target, target_code_size, &helper);
	if (ret) {
		LOG_ERROR("DW SPI could not allocate working area. Need %zx",
				  total_working_area_size);
		goto err_helper;
	}

	struct working_area *helper_args;
	ret = target_alloc_working_area(target, sizeof(struct dw_spi_program),
									&helper_args);
	if (ret) {
		LOG_ERROR("DW SPI could not allocate working area. Need %zx",
				  total_working_area_size);
		goto err_helper_args;
	}

	struct working_area *target_buffer;
	ret = target_alloc_working_area(target, buffer_size, &target_buffer);
	if (ret) {
		LOG_ERROR("DW SPI could not allocate working area. Need %zx",
				  total_working_area_size);
		goto err_target_buffer;
	}

	// write algorithm code and buffer to working areas
	ret = target_write_buffer(target, helper->address, target_code_size,
							  target_code);
	if (ret) {
		LOG_ERROR("DW SPI writing to working area error");
		goto err_write_buffer;
	}

	ret = target_write_buffer(target, target_buffer->address, buffer_size,
							  buffer);
	if (ret) {
		LOG_ERROR("DW SPI writing to working area error");
		goto err_write_buffer;
	}

	// prepare helper execution
	struct mips32_algorithm mips32_algo = { .common_magic = MIPS32_COMMON_MAGIC,
											.isa_mode = MIPS32_ISA_MIPS32 };

	struct reg_param reg_param;
	init_reg_param(&reg_param, DW_SPI_ARG_REG, 32, PARAM_OUT);
	struct mem_param mem_param;
	init_mem_param(&mem_param, helper_args->address, helper_args->size,
				   PARAM_OUT);

	// Set the arguments for the helper
	buf_set_u32(reg_param.value, 0, 32, helper_args->address);
	struct dw_spi_program *helper_args_val =
		(struct dw_spi_program *)mem_param.value;
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->address,
						  address);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->page_size,
						  page_size);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->buffer,
						  target_buffer->address);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->buffer_size,
						  buffer_size);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->status_reg,
						  regmap->simc + DW_SPI_REG_SR);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->data_reg,
						  regmap->simc + DW_SPI_REG_DR);
	helper_args_val->read_status_cmd = stat_cmd;
	helper_args_val->write_enable_cmd = we_cmd;
	helper_args_val->program_cmd = program_cmd;
	helper_args_val->write_enable_mask = we_mask;
	helper_args_val->busy_mask = busy_mask;
	helper_args_val->four_byte_mode = driver->four_byte_mode;

	ret = target_run_algorithm(target, 1, &mem_param, 1, &reg_param,
							   helper->address, 0, driver->timeout,
							   &mips32_algo);
	if (ret)
		LOG_ERROR("DW SPI flash algorithm error");

	destroy_reg_param(&reg_param);
	destroy_mem_param(&mem_param);

err_write_buffer:
	target_free_working_area(target, target_buffer);
err_target_buffer:
	target_free_working_area(target, helper_args);
err_helper_args:
	target_free_working_area(target, helper);
err_helper:

	return ret;
}

/**
 * @brief Erase sectors.
 *
 * @param[in] bank: Flash bank.
 * @param[in] address: Flash address. Must be sector aligned.
 * @param[in] sector_size: Size of flash sector.
 * @param[in] sector_count: Number of sectors to erase.
 * @param[in] stat_cmd: Flash command to read chip status.
 * @param[in] we_cmd: Flash command to set enable write.
 * @param[in] erase_sector_cmd: Flash command to set erase sector.
 * @param[in] we_mask: Status byte write enable mask.
 * @param[in] busy_mask: Status byte write busy mask.
 * @return Command execution status.
 */
static int
dw_spi_ctrl_erase_sectors(const struct flash_bank *const bank, uint32_t address,
						  uint32_t sector_size, size_t sector_count,
						  uint8_t stat_cmd, uint8_t we_cmd,
						  uint8_t erase_sector_cmd, uint8_t we_mask,
						  uint8_t busy_mask)
{
	struct target *const target = bank->target;
	const struct dw_spi_driver *const driver = bank->driver_priv;
	const struct dw_spi_regmap *const regmap = &driver->regmap;

	static const uint8_t target_code[] = {
#include "../../../contrib/loaders/flash/dw-spi/mipsel-linux-gnu-erase.inc"
	};
	const size_t target_code_size = sizeof(target_code);
	const size_t total_working_area_size =
		target_code_size + sizeof(struct dw_spi_erase);

	// allocate working area and memory args
	struct working_area *helper;
	int ret = target_alloc_working_area(target, target_code_size, &helper);
	if (ret) {
		LOG_ERROR("DW SPI could not allocate working area. Need %zx",
				  total_working_area_size);
		goto err_helper;
	}

	struct working_area *helper_args;
	ret = target_alloc_working_area(target, sizeof(struct dw_spi_erase),
									&helper_args);
	if (ret) {
		LOG_ERROR("DW SPI could not allocate working area. Need %zx",
				  total_working_area_size);
		goto err_helper_args;
	}

	// write algorithm code to working area
	ret = target_write_buffer(target, helper->address, target_code_size,
							  target_code);
	if (ret) {
		LOG_ERROR("DW SPI writing to working area error");
		goto err_write_buffer;
	}

	// prepare helper execution
	struct mips32_algorithm mips32_algo = { .common_magic = MIPS32_COMMON_MAGIC,
											.isa_mode = MIPS32_ISA_MIPS32 };

	struct reg_param reg_param;
	init_reg_param(&reg_param, DW_SPI_ARG_REG, 32, PARAM_OUT);
	struct mem_param mem_param;
	init_mem_param(&mem_param, helper_args->address, helper_args->size,
				   PARAM_OUT);

	// Set the arguments for the helper
	buf_set_u32(reg_param.value, 0, 32, helper_args->address);
	struct dw_spi_erase *helper_args_val =
		(struct dw_spi_erase *)mem_param.value;
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->address,
						  address);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->sector_size,
						  sector_size);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->sector_count,
						  sector_count);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->status_reg,
						  regmap->simc + DW_SPI_REG_SR);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->data_reg,
						  regmap->simc + DW_SPI_REG_DR);
	helper_args_val->read_status_cmd = stat_cmd;
	helper_args_val->write_enable_cmd = we_cmd;
	helper_args_val->erase_sector_cmd = erase_sector_cmd;
	helper_args_val->write_enable_mask = we_mask;
	helper_args_val->busy_mask = busy_mask;
	helper_args_val->four_byte_mode = driver->four_byte_mode;

	ret = target_run_algorithm(target, 1, &mem_param, 1, &reg_param,
							   helper->address, 0, driver->timeout,
							   &mips32_algo);
	if (ret)
		LOG_ERROR("DW SPI flash algorithm error");

	destroy_reg_param(&reg_param);
	destroy_mem_param(&mem_param);

err_write_buffer:
	target_free_working_area(target, helper_args);
err_helper_args:
	target_free_working_area(target, helper);
err_helper:

	return ret;
}

/**
 * @brief Read flash data.
 *
 * @param[in] bank: Flash bank.
 * @param[in] address: Flash address.
 * @param[out] buffer: Data buffer.
 * @param[in] buffer_size: \p buffer size.
 * @param[in] read_cmd: Flash command to read data from flash.
 * @return Command execution status.
 */
static int
dw_spi_ctrl_read(const struct flash_bank *const bank, uint32_t address,
				 uint8_t *buffer, size_t buffer_size, uint8_t read_cmd)
{
	struct target *const target = bank->target;
	const struct dw_spi_driver *const driver = bank->driver_priv;
	const struct dw_spi_regmap *const regmap = &driver->regmap;

	static const uint8_t target_code[] = {
#include "../../../contrib/loaders/flash/dw-spi/mipsel-linux-gnu-read.inc"
	};
	const size_t target_code_size = sizeof(target_code);
	const size_t total_working_area_size =
		target_code_size + sizeof(struct dw_spi_read) + buffer_size;

	// allocate working area and memory args
	struct working_area *helper;
	int ret = target_alloc_working_area(target, target_code_size, &helper);
	if (ret) {
		LOG_ERROR("DW SPI could not allocate working area. Need %zx",
				  total_working_area_size);
		goto err_helper;
	}

	struct working_area *helper_args;
	ret = target_alloc_working_area(target, sizeof(struct dw_spi_read),
									&helper_args);
	if (ret) {
		LOG_ERROR("DW SPI could not allocate working area. Need %zx",
				  total_working_area_size);
		goto err_helper_args;
	}

	struct working_area *target_buffer;
	ret = target_alloc_working_area(target, buffer_size, &target_buffer);
	if (ret) {
		LOG_ERROR("DW SPI could not allocate working area. Need %zx",
				  total_working_area_size);
		goto err_target_buffer;
	}

	// write algorithm code to working area
	ret = target_write_buffer(target, helper->address, target_code_size,
							  target_code);
	if (ret) {
		LOG_ERROR("DW SPI writing to working area error");
		goto err_write_buffer;
	}

	// prepare helper execution
	struct mips32_algorithm mips32_algo = { .common_magic = MIPS32_COMMON_MAGIC,
											.isa_mode = MIPS32_ISA_MIPS32 };

	struct reg_param reg_param;
	init_reg_param(&reg_param, DW_SPI_ARG_REG, 32, PARAM_OUT);
	struct mem_param mem_param;
	init_mem_param(&mem_param, helper_args->address, helper_args->size,
				   PARAM_OUT);

	// Set the arguments for the helper
	buf_set_u32(reg_param.value, 0, 32, helper_args->address);
	struct dw_spi_read *helper_args_val = (struct dw_spi_read *)mem_param.value;
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->address,
						  address);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->buffer,
						  target_buffer->address);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->buffer_size,
						  buffer_size);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->status_reg,
						  regmap->simc + DW_SPI_REG_SR);
	target_buffer_set_u32(target, (uint8_t *)&helper_args_val->data_reg,
						  regmap->simc + DW_SPI_REG_DR);
	helper_args_val->read_cmd = read_cmd;
	helper_args_val->four_byte_mode = driver->four_byte_mode;

	ret = target_run_algorithm(target, 1, &mem_param, 1, &reg_param,
							   helper->address, 0, driver->timeout,
							   &mips32_algo);
	if (ret) {
		LOG_ERROR("DW SPI flash algorithm error");
		goto cleanup;
	}

	ret =
		target_read_buffer(target, target_buffer->address, buffer_size, buffer);
	if (ret)
		LOG_ERROR("DW SPI target buffer read error");

cleanup:
	destroy_reg_param(&reg_param);
	destroy_mem_param(&mem_param);

err_write_buffer:
	target_free_working_area(target, target_buffer);
err_target_buffer:
	target_free_working_area(target, helper_args);
err_helper_args:
	target_free_working_area(target, helper);
err_helper:

	return ret;
}

/**
 * @brief Read Flash device ID.
 *
 * @param[in] bank: Flash bank handle.
 * @return Command execution status.
 */
static int
dw_spi_read_id(const struct flash_bank *const bank)
{
	struct dw_spi_driver *const driver = bank->driver_priv;

	const size_t buffer_size = 1 + 3 + 1;
	uint8_t buffer[buffer_size];

	memset(buffer, 0, buffer_size);
	buffer[0] = SPIFLASH_READ_ID;

	int ret = dw_spi_ctrl_transaction(bank, buffer, buffer_size, true);
	if (ret) {
		LOG_ERROR("DW SPI flash ID read error");
		return ret;
	}

	buffer[buffer_size - 1] = 0;
	// use le_to_h_u32 to decode flash ID as per JEDEC SFDP
	driver->id = le_to_h_u32(buffer + 1);
	LOG_DEBUG("DW SPI read flash ID %" PRIx32, driver->id);

	return ERROR_OK;
}

/**
 * @brief Read Flash device status.
 *
 * @param[in] bank: Flash bank handle.
 * @param[out] status: The status byte.
 * @return Command execution status.
 */
static int
dw_spi_read_status(const struct flash_bank *const bank, uint8_t *const status)
{
	const int buffer_size = 2;
	uint8_t buffer[buffer_size];

	memset(buffer, 0, buffer_size);
	buffer[0] = SPIFLASH_READ_STATUS;

	int ret = dw_spi_ctrl_transaction(bank, buffer, buffer_size, true);
	if (ret) {
		LOG_ERROR("DW SPI flash status read error");
		return ret;
	}

	*status = buffer[1];

	return ERROR_OK;
}

/**
 * @brief Wait for Flash command to finish.
 *
 * @param[in] bank: Flash bank handle.
 * @param[in] timeout: The timeout in ms.
 * @return Command execution status.
 */
static int
dw_spi_wait_finish(const struct flash_bank *const bank, unsigned int timeout)
{
	const int64_t end_time = timeval_ms() + timeout;
	while (timeval_ms() <= end_time) {
		uint8_t status;
		int ret = dw_spi_read_status(bank, &status);
		if (ret) {
			LOG_ERROR("DW SPI status query error");
			return ret;
		}
		if (!(status & SPIFLASH_BSY_BIT))
			return ERROR_OK;

		alive_sleep(1);
	}

	LOG_ERROR("DW SPI process timeout");
	return ERROR_TIMEOUT_REACHED;
}

/**
 * @brief Flash device write enable.
 *
 * @param[in] bank: Flash bank handle.
 * @return Command execution status.
 */
static int
dw_spi_write_enable(const struct flash_bank *const bank)
{
	const int buffer_size = 1;
	uint8_t buffer[buffer_size];

	memset(buffer, 0, buffer_size);
	buffer[0] = SPIFLASH_WRITE_ENABLE;

	int ret = dw_spi_ctrl_transaction(bank, buffer, buffer_size, false);
	if (ret) {
		LOG_ERROR("DW SPI flash write enable error");
		return ret;
	}

	uint8_t status;
	ret = dw_spi_read_status(bank, &status);
	if (ret)
		return ret;

	return status & SPIFLASH_WE_BIT ? ERROR_OK : ERROR_FAIL;
}

/**
 * @brief Erase Flash chip.
 *
 * @param[in] bank: Flash bank handle.
 * @return Command execution status.
 */
static int
dw_spi_erase_chip(const struct flash_bank *const bank)
{
	const struct dw_spi_driver *const driver = bank->driver_priv;

	const int buffer_size = 1;
	uint8_t buffer[buffer_size];

	int ret = dw_spi_write_enable(bank);
	if (ret)
		return ret;

	memset(buffer, 0, buffer_size);
	buffer[0] = driver->spi_flash->chip_erase_cmd;

	ret = dw_spi_ctrl_transaction(bank, buffer, buffer_size, false);
	if (ret) {
		LOG_ERROR("DW SPI erase flash error");
		return ret;
	}

	ret = dw_spi_wait_finish(bank, driver->timeout);
	if (ret) {
		LOG_ERROR("DW SPI erase flash timeout");
		return ret;
	}

	return ERROR_OK;
}

/**
 * @brief Flash device erase sectors.
 *
 * @param[in] bank: Flash bank handle.
 * @param[in] first: The first sector to erase.
 * @param[in] last: The last sector to erase.
 * @return Command execution status.
 */
static int
dw_spi_erase_sectors(const struct flash_bank *const bank, unsigned int first,
					 unsigned int last)
{
	const struct dw_spi_driver *const driver = bank->driver_priv;

	if (first == 0 && last >= (bank->num_sectors - 1)) {
		// full erase
		int ret = dw_spi_erase_chip(bank);
		if (ret)
			return ret;
	} else {
		// partial erase
		int ret = dw_spi_ctrl_erase_sectors(bank, bank->sectors[first].offset,
											driver->spi_flash->sectorsize,
											last - first + 1,
											SPIFLASH_READ_STATUS,
											SPIFLASH_WRITE_ENABLE,
											driver->spi_flash->erase_cmd,
											SPIFLASH_WE_BIT, SPIFLASH_BSY_BIT);
		if (ret) {
			LOG_ERROR("DW SPI flash erase sectors error");
			return ret;
		}
	}

	return ERROR_OK;
}

/**
 * @brief Flash bank blank check.
 */
static int
dw_spi_blank_check(struct flash_bank *bank, size_t sector_count,
				   uint8_t pattern)
{
	const struct dw_spi_driver *const driver = bank->driver_priv;

	uint8_t *erased = malloc(sector_count);
	if (!erased) {
		LOG_ERROR("could not allocate memory");
		return ERROR_FAIL;
	}

	// set initial erased value to unknown
	memset(erased, 2, sector_count);
	for (unsigned int sector_idx = 0; sector_idx < sector_count; sector_idx++)
		bank->sectors[sector_idx].is_erased = 2;

	int ret = dw_spi_ctrl_check_sectors_fill(bank, 0, bank->sectors[0].size,
											 sector_count, pattern,
											 driver->spi_flash->read_cmd,
											 erased);
	if (!ret) {
		for (unsigned int sector_idx = 0; sector_idx < sector_count;
			 sector_idx++)
			bank->sectors[sector_idx].is_erased = erased[sector_idx];
	} else {
		LOG_ERROR("DW SPI flash erase check error");
	}

	free(erased);

	return ret;
}

/**
 * @brief Write buffer to Flash.
 *
 * @param[in] bank: Flash bank handle.
 * @param[in] buffer: Data buffer.
 * @param[in] offset: Flash address offset.
 * @param[in] count: \p buffer size.
 * @return Command execution status.
 */
static int
dw_spi_write_buffer(const struct flash_bank *const bank, const uint8_t *buffer,
					uint32_t offset, uint32_t count)
{
	const struct dw_spi_driver *const driver = bank->driver_priv;
	const size_t page_size = driver->spi_flash->pagesize;

	// Write unaligned first sector separately as helper function does
	// not handle this case well.
	struct {
		uint32_t address;
		const uint8_t *buffer;
		size_t count;
	} chunks[2] = {
		{ .address = offset, .buffer = buffer, .count = 0 },
		{ .address = offset, .buffer = buffer, .count = count },
	};

	if (offset % page_size) {
		// start is not aligned
		chunks[0].count = MIN(page_size - (offset % page_size), count);
		chunks[1].count -= chunks[0].count;
		chunks[1].address += chunks[0].count;
		chunks[1].buffer += chunks[0].count;
	}

	for (unsigned int chunk_idx = 0; chunk_idx < ARRAY_SIZE(chunks);
		 chunk_idx++) {
		if (chunks[chunk_idx].count > 0) {
			int ret = dw_spi_ctrl_program(bank, chunks[chunk_idx].address,
										  chunks[chunk_idx].buffer,
										  chunks[chunk_idx].count, page_size,
										  SPIFLASH_READ_STATUS,
										  SPIFLASH_WRITE_ENABLE,
										  driver->spi_flash->pprog_cmd,
										  SPIFLASH_WE_BIT, SPIFLASH_BSY_BIT);
			if (ret) {
				LOG_ERROR("DW SPI flash write error");
				return ret;
			}
		}
	}

	return ERROR_OK;
}

/**
 * @brief Search for Flash chip info.
 *
 * @param[in] bank: Flash bank handle.
 * @return Command execution status.
 */
static int
dw_spi_spiflash_search(const struct flash_bank *const bank)
{
	struct dw_spi_driver *const driver = bank->driver_priv;

	int ret = dw_spi_read_id(bank);
	if (ret)
		return ret;

	unsigned int idx = 0;
	while (flash_devices[idx].name) {
		if (flash_devices[idx].device_id == driver->id) {
			driver->spi_flash = &flash_devices[idx];
			return ERROR_OK;
		}
		idx++;
	}

	LOG_ERROR("DW SPI could not find Flash with ID %" PRIx32
			  " in SPI Flash table: either Flash device is not supported "
			  "or communication speed is too high",
			  driver->id);
	return ERROR_FAIL;
}

/**
 * @brief Handle flash bank command.
 *
 * @param[in] CMD_ARGC: Number of arguments.
 * @param[in] CMD_ARGV: Command arguments.
 * @return Command execution status.
 */
FLASH_BANK_COMMAND_HANDLER(dw_spi_flash_bank_command)
{
	unsigned int speed = 1000000;
	unsigned int timeout = DW_SPI_TIMEOUT_DEFAULT;
	uint8_t chip_select_bitmask = BIT(0);
	struct dw_spi_regmap regmap = { 0 };

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	for (unsigned int idx = 6; idx < CMD_ARGC; idx++) {
		if (strcmp(CMD_ARGV[idx], "-jaguar2") == 0) {
			// Fast config for Jaguar2 chips.
			memcpy(&regmap, &jaguar2_regmap, sizeof(jaguar2_regmap));
		} else if (strcmp(CMD_ARGV[idx], "-ocelot") == 0) {
			// Fast config for Ocelot chips.
			memcpy(&regmap, &ocelot_regmap, sizeof(ocelot_regmap));
		} else if (strcmp(CMD_ARGV[idx], "-freq") == 0) {
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[++idx], regmap.freq);
		} else if (strcmp(CMD_ARGV[idx], "-simc") == 0) {
			COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[++idx], regmap.simc);
		} else if (strcmp(CMD_ARGV[idx], "-spi_mst") == 0) {
			COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[++idx], regmap.spi_mst);
		} else if (strcmp(CMD_ARGV[idx], "-if_owner_offset") == 0) {
			COMMAND_PARSE_NUMBER(u8, CMD_ARGV[++idx],
								 regmap.si_if_owner_offset);
		} else if (strcmp(CMD_ARGV[idx], "-speed") == 0) {
			COMMAND_PARSE_NUMBER(uint, CMD_ARGV[++idx], speed);
		} else if (strcmp(CMD_ARGV[idx], "-timeout") == 0) {
			COMMAND_PARSE_NUMBER(uint, CMD_ARGV[++idx], timeout);
			timeout *= 1000; // convert to ms
		} else if (strcmp(CMD_ARGV[idx], "-chip_select") == 0) {
			unsigned int cs_bit;
			COMMAND_PARSE_NUMBER(uint, CMD_ARGV[++idx], cs_bit);
			chip_select_bitmask = BIT(cs_bit);
		} else {
			LOG_WARNING("DW SPI unknown argument %s", CMD_ARGV[idx]);
		}
	}

	if (!regmap.simc) {
		LOG_ERROR("DW SPI cannot use boot controller with unconfigured simc");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	bank->driver_priv = malloc(sizeof(struct dw_spi_driver));
	if (!bank->driver_priv) {
		LOG_ERROR("could not allocate memory");
		return ERROR_FAIL;
	}

	struct dw_spi_driver *driver = bank->driver_priv;
	memset(driver, 0, sizeof(struct dw_spi_driver));
	driver->speed = speed;
	driver->timeout = timeout;
	driver->chip_select_bitmask = chip_select_bitmask;
	driver->four_byte_mode = true; // 24bit commands not provided by spi.h
	memcpy(&driver->regmap, &regmap, sizeof(regmap));

	return ERROR_OK;
}

/**
 * @brief Assert target is halted.
 *
 * @param[in] bank: Flash bank handle.
 * @return Command execution status.
 */
static int
dw_spi_assert_halted(const struct flash_bank *const bank)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	return ERROR_OK;
}

/**
 * @brief Prepare master controller for transaction.
 *
 * @param[in] bank: Flash bank handle.
 * @return Command execution status.
 */
static int
dw_spi_master_ctrl_configure(struct flash_bank *bank)
{
	int ret = dw_spi_assert_halted(bank);
	if (ret)
		return ret;
	ret = dw_spi_ctrl_mode_configure(bank, DW_SPI_SI_MODE_MASTER);
	if (ret) {
		LOG_ERROR("DW SPI switch to master controller mode error");
		return ret;
	}
	ret = dw_spi_master_ctrl_enable(bank, false);
	if (ret) {
		LOG_ERROR("DW SPI disable master controller error");
		return ret;
	}
	ret = dw_spi_ctrl_configure_speed(bank);
	if (ret) {
		LOG_ERROR("DW SPI speed configuration error");
		return ret;
	}
	ret = dw_spi_ctrl_disable_interrupts(bank);
	if (ret) {
		LOG_ERROR("DW SPI disable SPI interrupts error");
		return ret;
	}
	ret = dw_spi_ctrl_configure_si(bank);
	if (ret) {
		LOG_ERROR("DW SPI controller configuration error");
		return ret;
	}
	ret = dw_spi_master_ctrl_enable(bank, true);
	if (ret) {
		LOG_ERROR("DW SPI enable master controller error");
		return ret;
	}

	return ERROR_OK;
}

/**
 * @brief Restore SI controller selection.
 *
 * @param[in] bank: Flash bank handle.
 * @return Command execution status.
 */
static int
dw_spi_master_ctrl_restore(struct flash_bank *bank)
{
	int ret = dw_spi_master_ctrl_enable(bank, false);
	if (ret) {
		LOG_ERROR("DW SPI disable master controller error");
		return ret;
	}
	ret = dw_spi_ctrl_mode_restore(bank);
	if (ret) {
		LOG_ERROR("DW SPI controller restore error");
		return ret;
	}

	return ret;
}

/**
 * @brief Flash bank probe.
 *
 * @param[in] bank: Flash bank handle.
 * @return Command execution status.
 */
static int
dw_spi_probe(struct flash_bank *bank)
{
	struct dw_spi_driver *const driver = bank->driver_priv;

	if (!driver)
		return ERROR_FAIL;

	if (strcmp(bank->target->type->name, mips_m4k_target.name) != 0 ||
		bank->target->endianness != TARGET_LITTLE_ENDIAN) {
		LOG_ERROR("DW SPI currently only supports "
				  "little endian mips_m4k target");
		return ERROR_TARGET_INVALID;
	}

	int ret = dw_spi_master_ctrl_configure(bank);
	if (ret)
		return ret;

	ret = dw_spi_spiflash_search(bank);
	if (ret)
		goto err;

	bank->write_start_alignment = 0;
	bank->write_end_alignment = 0;

	uint32_t flash_size = driver->spi_flash->size_in_bytes;
	if (!bank->size) {
		bank->size = flash_size;
		LOG_INFO("DW SPI probed flash size 0x%" PRIx32, flash_size);
	} else {
		if (flash_size > bank->size)
			LOG_WARNING("DW SPI probed flash size 0x%" PRIx32
						" is greater then declared 0x%" PRIx32,
						flash_size, bank->size);
		if (flash_size < bank->size) {
			LOG_ERROR("DW SPI probed flash size 0x%" PRIx32
					  " is smaller then declared 0x%" PRIx32,
					  flash_size, bank->size);
			ret = ERROR_FLASH_BANK_INVALID;
			goto err;
		}
	}
	bank->num_sectors = bank->size / driver->spi_flash->sectorsize;

	// free previously allocated in case of reprobing
	free(bank->sectors);

	bank->sectors =
		alloc_block_array(0, driver->spi_flash->sectorsize, bank->num_sectors);

	if (!bank->sectors) {
		LOG_ERROR("could not allocate memory");
		ret = ERROR_FAIL;
		goto err;
	}

	driver->probed = true;

err:
	dw_spi_master_ctrl_restore(bank);
	return ret;
}

/**
 * @brief Flash bank erase sectors.
 *
 * @param[in] bank: Flash bank handle.
 * @param[in] first: The first sector to erase.
 * @param[in] last: The last sector to erase.
 * @return Command execution status.
 */
static int
dw_spi_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	int ret = dw_spi_master_ctrl_configure(bank);
	if (ret)
		return ret;

	ret = dw_spi_erase_sectors(bank, first, last);
	dw_spi_master_ctrl_restore(bank);
	return ret;
}

/**
 * @brief Flash bank write data.
 *
 * @param[in] bank: Flash bank handle.
 * @param[in] buffer: Data buffer.
 * @param[in] offset: Flash address offset.
 * @param[in] count: \p buffer size.
 * @return Command execution status.
 */
static int
dw_spi_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset,
			 uint32_t count)
{
	int ret = dw_spi_master_ctrl_configure(bank);
	if (ret)
		return ret;

	ret = dw_spi_write_buffer(bank, buffer, offset, count);
	dw_spi_master_ctrl_restore(bank);
	return ret;
}

/**
 * @brief Flash bank read data using master controller.
 *
 * @param[in] bank: Flash bank handle.
 * @param[out] buffer: Data buffer.
 * @param[in] offset: Flash address offset.
 * @param[in] count: \p buffer size.
 * @return Command execution status.
 */
static int
dw_spi_read(struct flash_bank *bank, uint8_t *buffer, uint32_t offset,
			uint32_t count)
{
	struct dw_spi_driver *const driver = bank->driver_priv;

	int ret = dw_spi_master_ctrl_configure(bank);
	if (ret)
		return ret;

	ret = dw_spi_ctrl_read(bank, offset, buffer, count,
						   driver->spi_flash->read_cmd);
	dw_spi_master_ctrl_restore(bank);
	return ret;
}

/**
 * @brief Flash bank erase check.
 *
 * @param[in] bank: Flash bank handle.
 * @return Command execution status.
 */
static int
dw_spi_erase_check(struct flash_bank *bank)
{
	int ret = dw_spi_master_ctrl_configure(bank);
	if (ret)
		return ret;

	ret = dw_spi_blank_check(bank, bank->num_sectors, bank->erased_value);

	dw_spi_master_ctrl_restore(bank);
	return ret;
}

/**
 * @brief Flash bank info.
 *
 * @param[in] bank: Flash bank handle.
 * @param[in,out] cmd Command invocation.
 * @return Command execution status.
 */
static int
dw_spi_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	const struct dw_spi_driver *const driver = bank->driver_priv;
	command_print(cmd, "model %s", driver->spi_flash->name);
	command_print(cmd, "ID 0x%" PRIx32, driver->id);
	command_print_sameline(cmd, "size 0x%" PRIx32, bank->size);
	return ERROR_OK;
}

/**
 * @brief Autoprobe driver.
 *
 * @param[in] bank: Flash bank handle.
 * @return Command execution status.
 */
static int
dw_spi_auto_probe(struct flash_bank *bank)
{
	struct dw_spi_driver *driver = bank->driver_priv;
	if (!driver)
		return ERROR_FAIL;
	if (!driver->probed)
		return dw_spi_probe(bank);
	return ERROR_OK;
}

/**
 * @brief DW-SPI NOR flash functions.
 */
const struct flash_driver dw_spi_flash = {
	.name = "dw-spi",
	.flash_bank_command = dw_spi_flash_bank_command,
	.erase = dw_spi_erase,
	.write = dw_spi_write,
	.read = dw_spi_read,
	.probe = dw_spi_probe,
	.auto_probe = dw_spi_auto_probe,
	.erase_check = dw_spi_erase_check,
	.info = dw_spi_info,
	.verify = default_flash_verify,
	.free_driver_priv = default_flash_free_driver_priv,
};
