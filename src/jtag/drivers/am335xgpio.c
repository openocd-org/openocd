// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022 by Steve Marple, stevemarple@googlemail.com        *
 *                                                                         *
 *   Based on bcm2835gpio.c and linuxgpiod.c                               *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <transport/transport.h>
#include "bitbang.h"

#include <sys/mman.h>

/* GPIO register base addresses. Values taken from "AM335x and AMIC110 Sitara
 * Processors Technical Reference Manual", Chapter 2 Memory Map.
 */
#define AM335XGPIO_NUM_GPIO_PER_CHIP 32
#define AM335XGPIO_NUM_GPIO_CHIPS 4
#define AM335XGPIO_GPIO0_HW_ADDR 0x44E07000
#define AM335XGPIO_GPIO1_HW_ADDR 0x4804C000
#define AM335XGPIO_GPIO2_HW_ADDR 0x481AC000
#define AM335XGPIO_GPIO3_HW_ADDR 0x481AE000

/* 32-bit offsets from GPIO chip base address. Values taken from "AM335x and
 * AMIC110 Sitara Processors Technical Reference Manual", Chapter 25
 * General-Purpose Input/Output.
 */
#define AM335XGPIO_GPIO_OE_OFFSET (0x134 / 4)
#define AM335XGPIO_GPIO_DATAIN_OFFSET (0x138 / 4)
#define AM335XGPIO_GPIO_DATAOUT_OFFSET (0x13C / 4)  /* DATAOUT register uses 0 for output, 1 for input */
#define AM335XGPIO_GPIO_CLEARDATAOUT_OFFSET (0x190 / 4)
#define AM335XGPIO_GPIO_SETDATAOUT_OFFSET (0x194 / 4)

#define AM335XGPIO_READ_REG(chip_num, offset) \
	(*(am335xgpio_gpio_chip_mmap_addr[(chip_num)] + (offset)))

#define AM335XGPIO_WRITE_REG(chip_num, offset, value) \
	(*(am335xgpio_gpio_chip_mmap_addr[(chip_num)] + (offset)) = (value))

#define AM335XGPIO_SET_REG_BITS(chip_num, offset, bit_mask) \
	(*(am335xgpio_gpio_chip_mmap_addr[(chip_num)] + (offset)) |= (bit_mask))

#define AM335XGPIO_CLEAR_REG_BITS(chip_num, offset, bit_mask) \
	(*(am335xgpio_gpio_chip_mmap_addr[(chip_num)] + (offset)) &= ~(bit_mask))

#define AM335XGPIO_SET_INPUT(gpio_config) \
	AM335XGPIO_SET_REG_BITS((gpio_config)->chip_num, AM335XGPIO_GPIO_OE_OFFSET, BIT((gpio_config)->gpio_num))
#define AM335XGPIO_SET_OUTPUT(gpio_config) \
	AM335XGPIO_CLEAR_REG_BITS((gpio_config)->chip_num, AM335XGPIO_GPIO_OE_OFFSET, BIT((gpio_config)->gpio_num))
#define AM335XGPIO_SET_HIGH(gpio_config) \
	AM335XGPIO_WRITE_REG((gpio_config)->chip_num, AM335XGPIO_GPIO_SETDATAOUT_OFFSET, BIT((gpio_config)->gpio_num))
#define AM335XGPIO_SET_LOW(gpio_config) \
	AM335XGPIO_WRITE_REG((gpio_config)->chip_num, AM335XGPIO_GPIO_CLEARDATAOUT_OFFSET, BIT((gpio_config)->gpio_num))

enum amx335gpio_initial_gpio_mode {
	AM335XGPIO_GPIO_MODE_INPUT,
	AM335XGPIO_GPIO_MODE_OUTPUT_LOW,
	AM335XGPIO_GPIO_MODE_OUTPUT_HIGH,
};

static const uint32_t am335xgpio_gpio_chip_hw_addr[AM335XGPIO_NUM_GPIO_CHIPS] = {
	AM335XGPIO_GPIO0_HW_ADDR,
	AM335XGPIO_GPIO1_HW_ADDR,
	AM335XGPIO_GPIO2_HW_ADDR,
	AM335XGPIO_GPIO3_HW_ADDR,
};

/* Memory-mapped address pointers */
static volatile uint32_t *am335xgpio_gpio_chip_mmap_addr[AM335XGPIO_NUM_GPIO_CHIPS];

static int dev_mem_fd;
static enum amx335gpio_initial_gpio_mode initial_gpio_mode[ADAPTER_GPIO_IDX_NUM];

/* Transition delay coefficients */
static int speed_coeff = 600000;
static int speed_offset = 575;
static unsigned int jtag_delay;

static const struct adapter_gpio_config *adapter_gpio_config;

static bool is_gpio_config_valid(const struct adapter_gpio_config *gpio_config)
{
	return gpio_config->chip_num < AM335XGPIO_NUM_GPIO_CHIPS
		&& gpio_config->gpio_num < AM335XGPIO_NUM_GPIO_PER_CHIP;
}

static int get_gpio_value(const struct adapter_gpio_config *gpio_config)
{
	unsigned int shift = gpio_config->gpio_num;
	uint32_t value = AM335XGPIO_READ_REG(gpio_config->chip_num, AM335XGPIO_GPIO_DATAIN_OFFSET);
	value = (value >> shift) & 1;
	return value ^ (gpio_config->active_low ? 1 : 0);
}

static void set_gpio_value(const struct adapter_gpio_config *gpio_config, int value)
{
	value = value ^ (gpio_config->active_low ? 1 : 0);
	switch (gpio_config->drive) {
	case ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL:
		if (value)
			AM335XGPIO_SET_HIGH(gpio_config);
		else
			AM335XGPIO_SET_LOW(gpio_config);
		/* For performance reasons assume the GPIO is already set as an output
		 * and therefore the call can be omitted here.
		 */
		break;
	case ADAPTER_GPIO_DRIVE_MODE_OPEN_DRAIN:
		if (value) {
			AM335XGPIO_SET_INPUT(gpio_config);
		} else {
			AM335XGPIO_SET_LOW(gpio_config);
			AM335XGPIO_SET_OUTPUT(gpio_config);
		}
		break;
	case ADAPTER_GPIO_DRIVE_MODE_OPEN_SOURCE:
		if (value) {
			AM335XGPIO_SET_HIGH(gpio_config);
			AM335XGPIO_SET_OUTPUT(gpio_config);
		} else {
			AM335XGPIO_SET_INPUT(gpio_config);
		}
		break;
	}
}

static enum amx335gpio_initial_gpio_mode get_gpio_mode(const struct adapter_gpio_config *gpio_config)
{
	if (AM335XGPIO_READ_REG(gpio_config->chip_num, AM335XGPIO_GPIO_OE_OFFSET) & BIT(gpio_config->gpio_num))
		return AM335XGPIO_GPIO_MODE_INPUT;

	/* Return output level too so that pin mode can be fully restored */
	if (AM335XGPIO_READ_REG(gpio_config->chip_num, AM335XGPIO_GPIO_DATAOUT_OFFSET) & BIT(gpio_config->gpio_num))
		return AM335XGPIO_GPIO_MODE_OUTPUT_HIGH;
	return AM335XGPIO_GPIO_MODE_OUTPUT_LOW;
}

static const char *get_gpio_mode_name(enum amx335gpio_initial_gpio_mode gpio_mode)
{
	switch (gpio_mode) {
	case AM335XGPIO_GPIO_MODE_INPUT:
		return "input";
	case AM335XGPIO_GPIO_MODE_OUTPUT_LOW:
		return "output (low)";
	case AM335XGPIO_GPIO_MODE_OUTPUT_HIGH:
		return "output (high)";
	}
	return "unknown";
}

static void initialize_gpio(enum adapter_gpio_config_index idx)
{
	if (!is_gpio_config_valid(&adapter_gpio_config[idx]))
		return;

	initial_gpio_mode[idx] = get_gpio_mode(&adapter_gpio_config[idx]);
	LOG_DEBUG("saved GPIO mode for %s (GPIO %d %d): %s",
			adapter_gpio_get_name(idx), adapter_gpio_config[idx].chip_num, adapter_gpio_config[idx].gpio_num,
			get_gpio_mode_name(initial_gpio_mode[idx]));

	if (adapter_gpio_config[idx].pull != ADAPTER_GPIO_PULL_NONE) {
		LOG_WARNING("am335xgpio does not support pull-up or pull-down settings (signal %s)",
			adapter_gpio_get_name(idx));
	}

	switch (adapter_gpio_config[idx].init_state) {
	case ADAPTER_GPIO_INIT_STATE_INACTIVE:
		set_gpio_value(&adapter_gpio_config[idx], 0);
		break;
	case ADAPTER_GPIO_INIT_STATE_ACTIVE:
		set_gpio_value(&adapter_gpio_config[idx], 1);
		break;
	case ADAPTER_GPIO_INIT_STATE_INPUT:
		AM335XGPIO_SET_INPUT(&adapter_gpio_config[idx]);
		break;
	}

	/* Direction for non push-pull is already set by set_gpio_value() */
	if (adapter_gpio_config[idx].drive == ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL
		&& adapter_gpio_config[idx].init_state != ADAPTER_GPIO_INIT_STATE_INPUT)
		AM335XGPIO_SET_OUTPUT(&adapter_gpio_config[idx]);
}

static void restore_gpio(enum adapter_gpio_config_index idx)
{
	if (is_gpio_config_valid(&adapter_gpio_config[idx])) {
		switch (initial_gpio_mode[idx]) {
		case AM335XGPIO_GPIO_MODE_INPUT:
			AM335XGPIO_SET_INPUT(&adapter_gpio_config[idx]);
			break;
		case AM335XGPIO_GPIO_MODE_OUTPUT_LOW:
			AM335XGPIO_SET_LOW(&adapter_gpio_config[idx]);
			AM335XGPIO_SET_OUTPUT(&adapter_gpio_config[idx]);
			break;
		case AM335XGPIO_GPIO_MODE_OUTPUT_HIGH:
			AM335XGPIO_SET_HIGH(&adapter_gpio_config[idx]);
			AM335XGPIO_SET_OUTPUT(&adapter_gpio_config[idx]);
			break;
		}
	}
}

static enum bb_value am335xgpio_read(void)
{
	return get_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_TDO]) ? BB_HIGH : BB_LOW;
}

static int am335xgpio_write(int tck, int tms, int tdi)
{
	set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_TDI], tdi);
	set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_TMS], tms);
	set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_TCK], tck); /* Write clock last */

	for (unsigned int i = 0; i < jtag_delay; ++i)
		asm volatile ("");

	return ERROR_OK;
}

static int am335xgpio_swd_write(int swclk, int swdio)
{
	set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO], swdio);
	set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWCLK], swclk); /* Write clock last */

	for (unsigned int i = 0; i < jtag_delay; ++i)
		asm volatile ("");

	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int am335xgpio_reset(int trst, int srst)
{
	/* As the "adapter reset_config" command keeps the srst and trst gpio drive
	 * mode settings in sync we can use our standard set_gpio_value() function
	 * that honours drive mode and active low.
	 */
	if (is_gpio_config_valid(&adapter_gpio_config[ADAPTER_GPIO_IDX_SRST]))
		set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_SRST], srst);

	if (is_gpio_config_valid(&adapter_gpio_config[ADAPTER_GPIO_IDX_TRST]))
		set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_TRST], trst);

	LOG_DEBUG("trst %d gpio: %d %d, srst %d gpio: %d %d",
		trst,
		(int)adapter_gpio_config[ADAPTER_GPIO_IDX_TRST].chip_num,
		(int)adapter_gpio_config[ADAPTER_GPIO_IDX_TRST].gpio_num,
		srst,
		(int)adapter_gpio_config[ADAPTER_GPIO_IDX_SRST].chip_num,
		(int)adapter_gpio_config[ADAPTER_GPIO_IDX_SRST].gpio_num);
	return ERROR_OK;
}

static void am335xgpio_swdio_drive(bool is_output)
{
	if (is_output) {
		if (is_gpio_config_valid(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO_DIR]))
			set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO_DIR], 1);
		AM335XGPIO_SET_OUTPUT(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO]);
	} else {
		AM335XGPIO_SET_INPUT(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO]);
		if (is_gpio_config_valid(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO_DIR]))
			set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO_DIR], 0);
	}
}

static int am335xgpio_swdio_read(void)
{
	return get_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO]);
}

static int am335xgpio_blink(bool on)
{
	if (is_gpio_config_valid(&adapter_gpio_config[ADAPTER_GPIO_IDX_LED]))
		set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_LED], on ? 1 : 0);

	return ERROR_OK;
}

static const struct bitbang_interface am335xgpio_bitbang = {
	.read = am335xgpio_read,
	.write = am335xgpio_write,
	.swdio_read = am335xgpio_swdio_read,
	.swdio_drive = am335xgpio_swdio_drive,
	.swd_write = am335xgpio_swd_write,
	.blink = am335xgpio_blink
};

static int am335xgpio_khz(int khz, int *jtag_speed)
{
	if (!khz) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}
	*jtag_speed = speed_coeff / khz - speed_offset;
	if (*jtag_speed < 0)
		*jtag_speed = 0;
	return ERROR_OK;
}

static int am335xgpio_speed_div(int speed, int *khz)
{
	*khz = speed_coeff / (speed + speed_offset);
	return ERROR_OK;
}

static int am335xgpio_speed(int speed)
{
	jtag_delay = speed;
	return ERROR_OK;
}

COMMAND_HANDLER(am335xgpio_handle_speed_coeffs)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], speed_coeff);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], speed_offset);
	}

	command_print(CMD, "AM335x GPIO config: speed_coeffs = %d, speed_offset = %d",
				  speed_coeff, speed_offset);
	return ERROR_OK;
}

static const struct command_registration am335xgpio_subcommand_handlers[] = {
	{
		.name = "speed_coeffs",
		.handler = am335xgpio_handle_speed_coeffs,
		.mode = COMMAND_CONFIG,
		.help = "SPEED_COEFF and SPEED_OFFSET for delay calculations.",
		.usage = "[SPEED_COEFF SPEED_OFFSET]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration am335xgpio_command_handlers[] = {
	{
		.name = "am335xgpio",
		.mode = COMMAND_ANY,
		.help = "perform am335xgpio management",
		.chain = am335xgpio_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static const char * const am335xgpio_transports[] = { "jtag", "swd", NULL };

static struct jtag_interface am335xgpio_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

static bool am335xgpio_jtag_mode_possible(void)
{
	if (!is_gpio_config_valid(&adapter_gpio_config[ADAPTER_GPIO_IDX_TCK]))
		return false;
	if (!is_gpio_config_valid(&adapter_gpio_config[ADAPTER_GPIO_IDX_TMS]))
		return false;
	if (!is_gpio_config_valid(&adapter_gpio_config[ADAPTER_GPIO_IDX_TDI]))
		return false;
	if (!is_gpio_config_valid(&adapter_gpio_config[ADAPTER_GPIO_IDX_TDO]))
		return false;
	return true;
}

static bool am335xgpio_swd_mode_possible(void)
{
	if (!is_gpio_config_valid(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWCLK]))
		return false;
	if (!is_gpio_config_valid(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO]))
		return false;
	return true;
}

static void am335xgpio_munmap(void)
{
	for (unsigned int i = 0; i < AM335XGPIO_NUM_GPIO_CHIPS && am335xgpio_gpio_chip_mmap_addr[i] != MAP_FAILED; ++i)
		if (munmap((void *)am335xgpio_gpio_chip_mmap_addr[i], sysconf(_SC_PAGE_SIZE)) < 0)
			LOG_ERROR("Cannot unmap GPIO memory for chip %d: %s", i, strerror(errno));
}

static int am335xgpio_init(void)
{
	LOG_INFO("AM335x GPIO JTAG/SWD bitbang driver");

	bitbang_interface = &am335xgpio_bitbang;
	adapter_gpio_config = adapter_gpio_get_config();

	if (transport_is_jtag() && !am335xgpio_jtag_mode_possible()) {
		LOG_ERROR("Require tck, tms, tdi and tdo gpios for JTAG mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (transport_is_swd() && !am335xgpio_swd_mode_possible()) {
		LOG_ERROR("Require swclk and swdio gpio for SWD mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	dev_mem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC);
	if (dev_mem_fd < 0) {
		LOG_DEBUG("Cannot open /dev/gpiomem, fallback to /dev/mem");
		dev_mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
	}
	if (dev_mem_fd < 0) {
		LOG_ERROR("open: %s", strerror(errno));
		return ERROR_JTAG_INIT_FAILED;
	}

	for (unsigned int i = 0; i < AM335XGPIO_NUM_GPIO_CHIPS; ++i) {
		am335xgpio_gpio_chip_mmap_addr[i] = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, am335xgpio_gpio_chip_hw_addr[i]);

		if (am335xgpio_gpio_chip_mmap_addr[i] == MAP_FAILED) {
			LOG_ERROR("mmap: %s", strerror(errno));
			am335xgpio_munmap();
			close(dev_mem_fd);
			return ERROR_JTAG_INIT_FAILED;
		}
	}
	close(dev_mem_fd);

	/* Configure JTAG/SWD signals. Default directions and initial states are handled
	 * by adapter.c and "adapter gpio" command.
	 */
	if (transport_is_jtag()) {
		initialize_gpio(ADAPTER_GPIO_IDX_TDO);
		initialize_gpio(ADAPTER_GPIO_IDX_TDI);
		initialize_gpio(ADAPTER_GPIO_IDX_TMS);
		initialize_gpio(ADAPTER_GPIO_IDX_TCK);
		initialize_gpio(ADAPTER_GPIO_IDX_TRST);
	}

	if (transport_is_swd()) {
		/* swdio and its buffer should be initialized in the order that prevents
		 * two outputs from being connected together. This will occur if the
		 * swdio GPIO of the AM335x is configured as an output while its
		 * external buffer is configured to send the swdio signal from the
		 * target to the AM335x.
		 */
		if (adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].init_state == ADAPTER_GPIO_INIT_STATE_INPUT) {
			initialize_gpio(ADAPTER_GPIO_IDX_SWDIO);
			initialize_gpio(ADAPTER_GPIO_IDX_SWDIO_DIR);
		} else {
			initialize_gpio(ADAPTER_GPIO_IDX_SWDIO_DIR);
			initialize_gpio(ADAPTER_GPIO_IDX_SWDIO);
		}

		initialize_gpio(ADAPTER_GPIO_IDX_SWCLK);
	}

	initialize_gpio(ADAPTER_GPIO_IDX_SRST);
	initialize_gpio(ADAPTER_GPIO_IDX_LED);

	return ERROR_OK;
}

static int am335xgpio_quit(void)
{
	if (transport_is_jtag()) {
		restore_gpio(ADAPTER_GPIO_IDX_TDO);
		restore_gpio(ADAPTER_GPIO_IDX_TDI);
		restore_gpio(ADAPTER_GPIO_IDX_TMS);
		restore_gpio(ADAPTER_GPIO_IDX_TCK);
		restore_gpio(ADAPTER_GPIO_IDX_TRST);
	}

	if (transport_is_swd()) {
		/* Restore swdio/swdio_dir to their initial modes, even if that means
		 * connecting two outputs. Begin by making swdio an input so that the
		 * current and final states of swdio and swdio_dir do not have to be
		 * considered to calculate the safe restoration order.
		 */
		AM335XGPIO_SET_INPUT(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO]);
		restore_gpio(ADAPTER_GPIO_IDX_SWDIO_DIR);
		restore_gpio(ADAPTER_GPIO_IDX_SWDIO);

		restore_gpio(ADAPTER_GPIO_IDX_SWCLK);
	}

	restore_gpio(ADAPTER_GPIO_IDX_SRST);
	restore_gpio(ADAPTER_GPIO_IDX_LED);

	am335xgpio_munmap();

	return ERROR_OK;
}

struct adapter_driver am335xgpio_adapter_driver = {
	.name = "am335xgpio",
	.transports = am335xgpio_transports,
	.commands = am335xgpio_command_handlers,

	.init = am335xgpio_init,
	.quit = am335xgpio_quit,
	.reset = am335xgpio_reset,
	.speed = am335xgpio_speed,
	.khz = am335xgpio_khz,
	.speed_div = am335xgpio_speed_div,

	.jtag_ops = &am335xgpio_interface,
	.swd_ops = &bitbang_swd,
};
