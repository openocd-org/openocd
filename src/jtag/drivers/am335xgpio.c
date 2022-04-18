/***************************************************************************
 *   Copyright (C) 2022 by Steve Marple, stevemarple@googlemail.com        *
 *                                                                         *
 *   Based on bcm2835gpio.c and linuxgpiod.c                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <transport/transport.h>
#include "bitbang.h"

#include <sys/mman.h>

/*
 * GPIO register base addresses. Values taken from "AM335x and AMIC110 Sitara
 * Processors Technical Reference Manual", Chapter 2 Memory Map.
 */
#define AM335XGPIO_NUM_GPIO_PORTS 4
#define AM335XGPIO_GPIO0_HW_ADDR 0x44E07000
#define AM335XGPIO_GPIO1_HW_ADDR 0x4804C000
#define AM335XGPIO_GPIO2_HW_ADDR 0x481AC000
#define AM335XGPIO_GPIO3_HW_ADDR 0x481AE000

/* 32-bit offsets from GPIO port base address. Values taken from "AM335x and
 * AMIC110 Sitara Processors Technical Reference Manual", Chapter 25
 * General-Purpose Input/Output.
 */
#define AM335XGPIO_GPIO_OE_OFFSET (0x134 / 4)
#define AM335XGPIO_GPIO_DATAIN_OFFSET (0x138 / 4)
#define AM335XGPIO_GPIO_DATAOUT_OFFSET (0x13C / 4)  /* DATAOUT register uses 0 for output, 1 for input */
#define AM335XGPIO_GPIO_CLEARDATAOUT_OFFSET (0x190 / 4)
#define AM335XGPIO_GPIO_SETDATAOUT_OFFSET (0x194 / 4)

/* GPIOs are integer values; need to map to a port module, and the pin within
 * that module. GPIOs 0 to 31 map to GPIO0, 32 to 63 to GPIO1 etc. This scheme
 * matches that used by Linux on the BeagleBone.
 */
#define AM335XGPIO_PORT_NUM(gpio_num) ((gpio_num) / 32)
#define AM335XGPIO_BIT_NUM(gpio_num) ((gpio_num) % 32)
#define AM335XGPIO_BIT_MASK(gpio_num) BIT(AM335XGPIO_BIT_NUM(gpio_num))

#define AM335XGPIO_READ_REG(gpio_num, offset) \
	(*(am335xgpio_gpio_port_mmap_addr[AM335XGPIO_PORT_NUM(gpio_num)] + (offset)))

#define AM335XGPIO_WRITE_REG(gpio_num, offset, value) \
	(*(am335xgpio_gpio_port_mmap_addr[AM335XGPIO_PORT_NUM(gpio_num)] + (offset)) = (value))

#define AM335XGPIO_SET_REG_BITS(gpio_num, offset, bit_mask) \
	(*(am335xgpio_gpio_port_mmap_addr[AM335XGPIO_PORT_NUM(gpio_num)] + (offset)) |= (bit_mask))

#define AM335XGPIO_CLEAR_REG_BITS(gpio_num, offset, bit_mask) \
	(*(am335xgpio_gpio_port_mmap_addr[AM335XGPIO_PORT_NUM(gpio_num)] + (offset)) &= ~(bit_mask))

enum amx335gpio_gpio_mode {
	AM335XGPIO_GPIO_MODE_INPUT,
	AM335XGPIO_GPIO_MODE_OUTPUT, /* To set output mode but not state */
	AM335XGPIO_GPIO_MODE_OUTPUT_LOW,
	AM335XGPIO_GPIO_MODE_OUTPUT_HIGH,
};

static const uint32_t am335xgpio_gpio_port_hw_addr[AM335XGPIO_NUM_GPIO_PORTS] = {
	AM335XGPIO_GPIO0_HW_ADDR,
	AM335XGPIO_GPIO1_HW_ADDR,
	AM335XGPIO_GPIO2_HW_ADDR,
	AM335XGPIO_GPIO3_HW_ADDR,
};

/* Memory-mapped address pointers */
static volatile uint32_t *am335xgpio_gpio_port_mmap_addr[AM335XGPIO_NUM_GPIO_PORTS];

static int dev_mem_fd;

/* GPIO numbers for each signal. Negative values are invalid */
static int tck_gpio = -1;
static enum amx335gpio_gpio_mode tck_gpio_mode;
static int tms_gpio = -1;
static enum amx335gpio_gpio_mode tms_gpio_mode;
static int tdi_gpio = -1;
static enum amx335gpio_gpio_mode tdi_gpio_mode;
static int tdo_gpio = -1;
static enum amx335gpio_gpio_mode tdo_gpio_mode;
static int trst_gpio = -1;
static enum amx335gpio_gpio_mode trst_gpio_mode;
static int srst_gpio = -1;
static enum amx335gpio_gpio_mode srst_gpio_mode;
static int swclk_gpio = -1;
static enum amx335gpio_gpio_mode swclk_gpio_mode;
static int swdio_gpio = -1;
static enum amx335gpio_gpio_mode swdio_gpio_mode;
static int swdio_dir_gpio = -1;
static enum amx335gpio_gpio_mode swdio_dir_gpio_mode;
static int led_gpio = -1;
static enum amx335gpio_gpio_mode led_gpio_mode = -1;

static bool swdio_dir_is_active_high = true; /* Active state means output */
static bool led_is_active_high = true;

/* Transition delay coefficients */
static int speed_coeff = 600000;
static int speed_offset = 575;
static unsigned int jtag_delay;

static int is_gpio_valid(int gpio_num)
{
	return gpio_num >= 0 && gpio_num < (32 * AM335XGPIO_NUM_GPIO_PORTS);
}

static int get_gpio_value(int gpio_num)
{
	unsigned int shift = AM335XGPIO_BIT_NUM(gpio_num);
	return (AM335XGPIO_READ_REG(gpio_num, AM335XGPIO_GPIO_DATAIN_OFFSET) >> shift) & 1;
}

static void set_gpio_value(int gpio_num, int value)
{
	if (value)
		AM335XGPIO_WRITE_REG(gpio_num, AM335XGPIO_GPIO_SETDATAOUT_OFFSET, AM335XGPIO_BIT_MASK(gpio_num));
	else
		AM335XGPIO_WRITE_REG(gpio_num, AM335XGPIO_GPIO_CLEARDATAOUT_OFFSET, AM335XGPIO_BIT_MASK(gpio_num));
}

static enum amx335gpio_gpio_mode get_gpio_mode(int gpio_num)
{
	if (AM335XGPIO_READ_REG(gpio_num, AM335XGPIO_GPIO_OE_OFFSET) & AM335XGPIO_BIT_MASK(gpio_num)) {
		return AM335XGPIO_GPIO_MODE_INPUT;
	} else {
		/* Return output level too so that pin mode can be fully restored */
		if (AM335XGPIO_READ_REG(gpio_num, AM335XGPIO_GPIO_DATAOUT_OFFSET) & AM335XGPIO_BIT_MASK(gpio_num))
			return AM335XGPIO_GPIO_MODE_OUTPUT_HIGH;
		else
			return AM335XGPIO_GPIO_MODE_OUTPUT_LOW;
	}
}

static void set_gpio_mode(int gpio_num, enum amx335gpio_gpio_mode gpio_mode)
{
	if (gpio_mode == AM335XGPIO_GPIO_MODE_INPUT) {
		AM335XGPIO_SET_REG_BITS(gpio_num, AM335XGPIO_GPIO_OE_OFFSET, AM335XGPIO_BIT_MASK(gpio_num));
		return;
	}

	if (gpio_mode == AM335XGPIO_GPIO_MODE_OUTPUT_LOW)
		set_gpio_value(gpio_num, 0);
	if (gpio_mode == AM335XGPIO_GPIO_MODE_OUTPUT_HIGH)
		set_gpio_value(gpio_num, 1);

	if (gpio_mode == AM335XGPIO_GPIO_MODE_OUTPUT ||
		gpio_mode == AM335XGPIO_GPIO_MODE_OUTPUT_LOW ||
		gpio_mode == AM335XGPIO_GPIO_MODE_OUTPUT_HIGH) {
			AM335XGPIO_CLEAR_REG_BITS(gpio_num, AM335XGPIO_GPIO_OE_OFFSET, AM335XGPIO_BIT_MASK(gpio_num));
	}
}

static const char *get_gpio_mode_name(enum amx335gpio_gpio_mode gpio_mode)
{
	switch (gpio_mode) {
	case AM335XGPIO_GPIO_MODE_INPUT:
		return "input";
	case AM335XGPIO_GPIO_MODE_OUTPUT:
		return "output";
	case AM335XGPIO_GPIO_MODE_OUTPUT_LOW:
		return "output (low)";
	case AM335XGPIO_GPIO_MODE_OUTPUT_HIGH:
		return "output (high)";
	default:
		return "unknown";
	}
}

static bb_value_t am335xgpio_read(void)
{
	return get_gpio_value(tdo_gpio) ? BB_HIGH : BB_LOW;
}

static int am335xgpio_write(int tck, int tms, int tdi)
{
	set_gpio_value(tdi_gpio, tdi);
	set_gpio_value(tms_gpio, tms);
	set_gpio_value(tck_gpio, tck); /* Write clock last */

	for (unsigned int i = 0; i < jtag_delay; ++i)
		asm volatile ("");

	return ERROR_OK;
}

static int am335xgpio_swd_write(int swclk, int swdio)
{
	set_gpio_value(swdio_gpio, swdio);
	set_gpio_value(swclk_gpio, swclk); /* Write clock last */

	for (unsigned int i = 0; i < jtag_delay; ++i)
		asm volatile ("");

	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int am335xgpio_reset(int trst, int srst)
{
	/* assume active low */
	if (is_gpio_valid(srst_gpio)) {
		if (jtag_get_reset_config() & RESET_SRST_PUSH_PULL)
			set_gpio_mode(srst_gpio, srst ? AM335XGPIO_GPIO_MODE_OUTPUT_LOW : AM335XGPIO_GPIO_MODE_OUTPUT_HIGH);
		else
			set_gpio_mode(srst_gpio, srst ? AM335XGPIO_GPIO_MODE_OUTPUT_LOW : AM335XGPIO_GPIO_MODE_INPUT);
	}

	/* assume active low */
	if (is_gpio_valid(trst_gpio)) {
		if (jtag_get_reset_config() & RESET_TRST_OPEN_DRAIN)
			set_gpio_mode(trst_gpio, trst ? AM335XGPIO_GPIO_MODE_OUTPUT_LOW : AM335XGPIO_GPIO_MODE_INPUT);
		else
			set_gpio_mode(trst_gpio, trst ? AM335XGPIO_GPIO_MODE_OUTPUT_LOW : AM335XGPIO_GPIO_MODE_OUTPUT_HIGH);
	}

	LOG_DEBUG("am335xgpio_reset(%d, %d), trst_gpio: %d (%s), srst_gpio: %d (%s)",
		trst, srst,
		trst_gpio, get_gpio_mode_name(get_gpio_mode(trst_gpio)),
		srst_gpio, get_gpio_mode_name(get_gpio_mode(srst_gpio)));
	return ERROR_OK;
}

static void am335xgpio_swdio_drive(bool is_output)
{
	if (is_output) {
		set_gpio_value(swdio_dir_gpio, swdio_dir_is_active_high ? 1 : 0);
		set_gpio_mode(swdio_gpio, AM335XGPIO_GPIO_MODE_OUTPUT);
	} else {
		set_gpio_mode(swdio_gpio, AM335XGPIO_GPIO_MODE_INPUT);
		set_gpio_value(swdio_dir_gpio, swdio_dir_is_active_high ? 0 : 1);
	}
}

static int am335xgpio_swdio_read(void)
{
	return get_gpio_value(swdio_gpio);
}

static int am335xgpio_blink(int on)
{
	if (is_gpio_valid(led_gpio))
		set_gpio_value(led_gpio, (!on ^ led_is_active_high) ? 1 : 0);

	return ERROR_OK;
}

static struct bitbang_interface am335xgpio_bitbang = {
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

COMMAND_HANDLER(am335xgpio_handle_jtag_gpionums)
{
	if (CMD_ARGC == 4) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], tms_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], tdi_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[3], tdo_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD, "AM335x GPIO config: tck = %d, tms = %d, tdi = %d, tdo = %d",
			tck_gpio, tms_gpio, tdi_gpio, tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(am335xgpio_handle_jtag_gpionum_tck)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tck_gpio);

	command_print(CMD, "AM335x GPIO config: tck = %d", tck_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(am335xgpio_handle_jtag_gpionum_tms)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tms_gpio);

	command_print(CMD, "AM335x GPIO config: tms = %d", tms_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(am335xgpio_handle_jtag_gpionum_tdo)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdo_gpio);

	command_print(CMD, "AM335x GPIO config: tdo = %d", tdo_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(am335xgpio_handle_jtag_gpionum_tdi)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], tdi_gpio);

	command_print(CMD, "AM335x GPIO config: tdi = %d", tdi_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(am335xgpio_handle_jtag_gpionum_srst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], srst_gpio);

	command_print(CMD, "AM335x GPIO config: srst = %d", srst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(am335xgpio_handle_jtag_gpionum_trst)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], trst_gpio);

	command_print(CMD, "AM335x GPIO config: trst = %d", trst_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(am335xgpio_handle_swd_gpionums)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], swdio_gpio);
	} else if (CMD_ARGC != 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	command_print(CMD, "AM335x GPIO config: swclk = %d, swdio = %d", swclk_gpio, swdio_gpio);

	return ERROR_OK;
}

COMMAND_HANDLER(am335xgpio_handle_swd_gpionum_swclk)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swclk_gpio);

	command_print(CMD, "AM335x GPIO config: swclk = %d", swclk_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(am335xgpio_handle_swd_gpionum_swdio)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swdio_gpio);

	command_print(CMD, "AM335x GPIO config: swdio = %d", swdio_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(am335xgpio_handle_swd_gpionum_swdio_dir)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], swdio_dir_gpio);

	command_print(CMD, "AM335x GPIO config: swdio_dir = %d", swdio_dir_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(am335xgpio_handle_swd_dir_output_state)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_BOOL(CMD_ARGV[0], swdio_dir_is_active_high, "high", "low");

	command_print(CMD, "AM335x GPIO config: swdio_dir_output_state = %s", swdio_dir_is_active_high ? "high" : "low");
	return ERROR_OK;
}

COMMAND_HANDLER(am335xgpio_handle_gpionum_led)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], led_gpio);

	command_print(CMD, "AM335x GPIO config: led = %d", led_gpio);
	return ERROR_OK;
}

COMMAND_HANDLER(am335xgpio_handle_led_on_state)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_BOOL(CMD_ARGV[0], led_is_active_high, "high", "low");

	command_print(CMD, "AM335x GPIO config: led_on_state = %s", led_is_active_high ? "high" : "low");
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
		.name = "jtag_nums",
		.handler = am335xgpio_handle_jtag_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for tck, tms, tdi, tdo (in that order).",
		.usage = "[tck tms tdi tdo]",
	},
	{
		.name = "tck_num",
		.handler = am335xgpio_handle_jtag_gpionum_tck,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tck.",
		.usage = "[tck]",
	},
	{
		.name = "tms_num",
		.handler = am335xgpio_handle_jtag_gpionum_tms,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tms.",
		.usage = "[tms]",
	},
	{
		.name = "tdo_num",
		.handler = am335xgpio_handle_jtag_gpionum_tdo,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdo.",
		.usage = "[tdo]",
	},
	{
		.name = "tdi_num",
		.handler = am335xgpio_handle_jtag_gpionum_tdi,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for tdi.",
		.usage = "[tdi]",
	},
	{
		.name = "swd_nums",
		.handler = am335xgpio_handle_swd_gpionums,
		.mode = COMMAND_CONFIG,
		.help = "gpio numbers for swclk, swdio (in that order).",
		.usage = "[swclk swdio]",
	},
	{
		.name = "swclk_num",
		.handler = am335xgpio_handle_swd_gpionum_swclk,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swclk.",
		.usage = "[swclk]",
	},
	{
		.name = "swdio_num",
		.handler = am335xgpio_handle_swd_gpionum_swdio,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio.",
		.usage = "[swdio]",
	},
	{
		.name = "swdio_dir_num",
		.handler = am335xgpio_handle_swd_gpionum_swdio_dir,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for swdio direction control pin.",
		.usage = "[swdio_dir]",
	},
	{
		.name = "swdio_dir_output_state",
		.handler = am335xgpio_handle_swd_dir_output_state,
		.mode = COMMAND_CONFIG,
		.help = "required state for swdio_dir pin to select SWDIO buffer to be output.",
		.usage = "['off'|'on']",
	},
	{
		.name = "srst_num",
		.handler = am335xgpio_handle_jtag_gpionum_srst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for srst.",
		.usage = "[srst]",
	},
	{
		.name = "trst_num",
		.handler = am335xgpio_handle_jtag_gpionum_trst,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for trst.",
		.usage = "[trst]",
	},
	{
		.name = "led_num",
		.handler = am335xgpio_handle_gpionum_led,
		.mode = COMMAND_CONFIG,
		.help = "gpio number for led.",
		.usage = "[led]",
	},
	{
		.name = "led_on_state",
		.handler = am335xgpio_handle_led_on_state,
		.mode = COMMAND_CONFIG,
		.help = "required state for led pin to turn on LED.",
		.usage = "['off'|'on']",
	},
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
	if (!is_gpio_valid(tck_gpio))
		return false;
	if (!is_gpio_valid(tms_gpio))
		return false;
	if (!is_gpio_valid(tdi_gpio))
		return false;
	if (!is_gpio_valid(tdo_gpio))
		return false;
	return true;
}

static bool am335xgpio_swd_mode_possible(void)
{
	if (!is_gpio_valid(swclk_gpio))
		return false;
	if (!is_gpio_valid(swdio_gpio))
		return false;
	return true;
}

static int am335xgpio_init(void)
{
	bitbang_interface = &am335xgpio_bitbang;

	LOG_INFO("AM335x GPIO JTAG/SWD bitbang driver");

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

	for (unsigned int i = 0; i < AM335XGPIO_NUM_GPIO_PORTS; ++i) {
		am335xgpio_gpio_port_mmap_addr[i] = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, am335xgpio_gpio_port_hw_addr[i]);

		if (am335xgpio_gpio_port_mmap_addr[i] == MAP_FAILED) {
			LOG_ERROR("mmap: %s", strerror(errno));
			close(dev_mem_fd);
			return ERROR_JTAG_INIT_FAILED;
		}
	}

	/*
	 * Configure TDO as an input, and TDI, TCK, TMS, TRST, SRST as outputs.
	 * Drive TDI and TCK low, and TMS high.
	 */
	if (transport_is_jtag()) {
		tdo_gpio_mode = get_gpio_mode(tdo_gpio);
		tdi_gpio_mode = get_gpio_mode(tdi_gpio);
		tck_gpio_mode = get_gpio_mode(tck_gpio);
		tms_gpio_mode = get_gpio_mode(tms_gpio);
		LOG_DEBUG("saved GPIO mode for tdo (GPIO #%d): %s", tdo_gpio, get_gpio_mode_name(tdo_gpio_mode));
		LOG_DEBUG("saved GPIO mode for tdi (GPIO #%d): %s", tdi_gpio, get_gpio_mode_name(tdi_gpio_mode));
		LOG_DEBUG("saved GPIO mode for tck (GPIO #%d): %s", tck_gpio, get_gpio_mode_name(tck_gpio_mode));
		LOG_DEBUG("saved GPIO mode for tms (GPIO #%d): %s", tms_gpio, get_gpio_mode_name(tms_gpio_mode));

		set_gpio_mode(tdo_gpio, AM335XGPIO_GPIO_MODE_INPUT);
		set_gpio_mode(tdi_gpio, AM335XGPIO_GPIO_MODE_OUTPUT_LOW);
		set_gpio_mode(tms_gpio, AM335XGPIO_GPIO_MODE_OUTPUT_HIGH);
		set_gpio_mode(tck_gpio, AM335XGPIO_GPIO_MODE_OUTPUT_LOW);

		if (is_gpio_valid(trst_gpio)) {
			trst_gpio_mode = get_gpio_mode(trst_gpio);
			LOG_DEBUG("saved GPIO mode for trst (GPIO #%d): %s", trst_gpio, get_gpio_mode_name(trst_gpio_mode));
		}
	}

	if (transport_is_swd()) {
		swclk_gpio_mode = get_gpio_mode(swclk_gpio);
		swdio_gpio_mode = get_gpio_mode(swdio_gpio);
		LOG_DEBUG("saved GPIO mode for swclk (GPIO #%d): %s", swclk_gpio, get_gpio_mode_name(swclk_gpio_mode));
		LOG_DEBUG("saved GPIO mode for swdio (GPIO #%d): %s", swdio_gpio, get_gpio_mode_name(swdio_gpio_mode));
		if (is_gpio_valid(swdio_dir_gpio)) {
			swdio_dir_gpio_mode = get_gpio_mode(swdio_dir_gpio);
			LOG_DEBUG("saved GPIO mode for swdio_dir (GPIO #%d): %s",
					swdio_dir_gpio, get_gpio_mode_name(swdio_dir_gpio_mode));
			set_gpio_mode(swdio_dir_gpio,
					swdio_dir_is_active_high ? AM335XGPIO_GPIO_MODE_OUTPUT_HIGH : AM335XGPIO_GPIO_MODE_OUTPUT_LOW);

		}
		set_gpio_mode(swdio_gpio, AM335XGPIO_GPIO_MODE_OUTPUT_LOW);
		set_gpio_mode(swclk_gpio, AM335XGPIO_GPIO_MODE_OUTPUT_LOW);
	}

	if (is_gpio_valid(srst_gpio)) {
		srst_gpio_mode = get_gpio_mode(srst_gpio);
		LOG_DEBUG("saved GPIO mode for srst (GPIO #%d): %s", srst_gpio, get_gpio_mode_name(srst_gpio_mode));
	}

	if (is_gpio_valid(led_gpio)) {
		led_gpio_mode = get_gpio_mode(led_gpio);
		LOG_DEBUG("saved GPIO mode for led (GPIO #%d): %s", led_gpio, get_gpio_mode_name(led_gpio_mode));
		set_gpio_mode(led_gpio,
				led_is_active_high ? AM335XGPIO_GPIO_MODE_OUTPUT_LOW : AM335XGPIO_GPIO_MODE_OUTPUT_HIGH);
	}

	/* Set GPIO modes for TRST and SRST and make both inactive */
	am335xgpio_reset(0, 0);
	return ERROR_OK;
}

static int am335xgpio_quit(void)
{
	if (transport_is_jtag()) {
		set_gpio_mode(tdo_gpio, tdo_gpio_mode);
		set_gpio_mode(tdi_gpio, tdi_gpio_mode);
		set_gpio_mode(tck_gpio, tck_gpio_mode);
		set_gpio_mode(tms_gpio, tms_gpio_mode);
		if (is_gpio_valid(trst_gpio))
			set_gpio_mode(trst_gpio, trst_gpio_mode);
	}

	if (transport_is_swd()) {
		set_gpio_mode(swclk_gpio, swclk_gpio_mode);
		set_gpio_mode(swdio_gpio, swdio_gpio_mode);
		if (is_gpio_valid(swdio_dir_gpio))
			set_gpio_mode(swdio_dir_gpio, swdio_dir_gpio_mode);
	}

	if (is_gpio_valid(srst_gpio))
		set_gpio_mode(srst_gpio, srst_gpio_mode);

	if (is_gpio_valid(led_gpio))
		set_gpio_mode(led_gpio, led_gpio_mode);

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
