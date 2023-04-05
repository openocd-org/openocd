// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2013 by Paul Fertser, fercerpav@gmail.com               *
 *                                                                         *
 *   Copyright (C) 2012 by Creative Product Design, marc @ cpdesign.com.au *
 *   Based on at91rm9200.c (c) Anders Larsen                               *
 *   and RPi GPIO examples by Gert van Loo & Dom                           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <transport/transport.h>
#include "bitbang.h"

#include <sys/mman.h>

uint32_t bcm2835_peri_base = 0x20000000;
#define BCM2835_GPIO_BASE	(bcm2835_peri_base + 0x200000) /* GPIO controller */

#define BCM2835_PADS_GPIO_0_27		(bcm2835_peri_base + 0x100000)
#define BCM2835_PADS_GPIO_0_27_OFFSET	(0x2c / 4)

/* See "GPIO Function Select Registers (GPFSELn)" in "Broadcom BCM2835 ARM Peripherals" datasheet. */
#define BCM2835_GPIO_MODE_INPUT 0
#define BCM2835_GPIO_MODE_OUTPUT 1

/* GPIO setup macros */
#define MODE_GPIO(g) (*(pio_base+((g)/10))>>(((g)%10)*3) & 7)
#define INP_GPIO(g) do { *(pio_base+((g)/10)) &= ~(7<<(((g)%10)*3)); } while (0)
#define SET_MODE_GPIO(g, m) do { /* clear the mode bits first, then set as necessary */ \
		INP_GPIO(g);						\
		*(pio_base+((g)/10)) |=  ((m)<<(((g)%10)*3)); } while (0)
#define OUT_GPIO(g) SET_MODE_GPIO(g, BCM2835_GPIO_MODE_OUTPUT)

#define GPIO_SET (*(pio_base+7))  /* sets   bits which are 1, ignores bits which are 0 */
#define GPIO_CLR (*(pio_base+10)) /* clears bits which are 1, ignores bits which are 0 */
#define GPIO_LEV (*(pio_base+13)) /* current level of the pin */

static int dev_mem_fd;
static volatile uint32_t *pio_base = MAP_FAILED;
static volatile uint32_t *pads_base = MAP_FAILED;

/* Transition delay coefficients */
static int speed_coeff = 113714;
static int speed_offset = 28;
static unsigned int jtag_delay;

static const struct adapter_gpio_config *adapter_gpio_config;
static struct initial_gpio_state {
	unsigned int mode;
	unsigned int output_level;
} initial_gpio_state[ADAPTER_GPIO_IDX_NUM];
static uint32_t initial_drive_strength_etc;

static bool is_gpio_config_valid(enum adapter_gpio_config_index idx)
{
	/* Only chip 0 is supported, accept unset value (-1) too */
	return adapter_gpio_config[idx].chip_num >= -1
		&& adapter_gpio_config[idx].chip_num <= 0
		&& adapter_gpio_config[idx].gpio_num >= 0
		&& adapter_gpio_config[idx].gpio_num <= 31;
}

static void set_gpio_value(const struct adapter_gpio_config *gpio_config, int value)
{
	value = value ^ (gpio_config->active_low ? 1 : 0);
	switch (gpio_config->drive) {
	case ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL:
		if (value)
			GPIO_SET = 1 << gpio_config->gpio_num;
		else
			GPIO_CLR = 1 << gpio_config->gpio_num;
		/* For performance reasons assume the GPIO is already set as an output
		 * and therefore the call can be omitted here.
		 */
		break;
	case ADAPTER_GPIO_DRIVE_MODE_OPEN_DRAIN:
		if (value) {
			INP_GPIO(gpio_config->gpio_num);
		} else {
			GPIO_CLR = 1 << gpio_config->gpio_num;
			OUT_GPIO(gpio_config->gpio_num);
		}
		break;
	case ADAPTER_GPIO_DRIVE_MODE_OPEN_SOURCE:
		if (value) {
			GPIO_SET = 1 << gpio_config->gpio_num;
			OUT_GPIO(gpio_config->gpio_num);
		} else {
			INP_GPIO(gpio_config->gpio_num);
		}
		break;
	}
}

static void restore_gpio(enum adapter_gpio_config_index idx)
{
	if (is_gpio_config_valid(idx)) {
		SET_MODE_GPIO(adapter_gpio_config[idx].gpio_num, initial_gpio_state[idx].mode);
		if (initial_gpio_state[idx].mode == BCM2835_GPIO_MODE_OUTPUT) {
			if (initial_gpio_state[idx].output_level)
				GPIO_SET = 1 << adapter_gpio_config[idx].gpio_num;
			else
				GPIO_CLR = 1 << adapter_gpio_config[idx].gpio_num;
		}
	}
}

static void initialize_gpio(enum adapter_gpio_config_index idx)
{
	if (!is_gpio_config_valid(idx))
		return;

	initial_gpio_state[idx].mode = MODE_GPIO(adapter_gpio_config[idx].gpio_num);
	unsigned int shift = adapter_gpio_config[idx].gpio_num;
	initial_gpio_state[idx].output_level = (GPIO_LEV >> shift) & 1;
	LOG_DEBUG("saved GPIO mode for %s (GPIO %d %d): %d",
			adapter_gpio_get_name(idx), adapter_gpio_config[idx].chip_num, adapter_gpio_config[idx].gpio_num,
			initial_gpio_state[idx].mode);

	if (adapter_gpio_config[idx].pull != ADAPTER_GPIO_PULL_NONE) {
		LOG_WARNING("BCM2835 GPIO does not support pull-up or pull-down settings (signal %s)",
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
		INP_GPIO(adapter_gpio_config[idx].gpio_num);
		break;
	}

	/* Direction for non push-pull is already set by set_gpio_value() */
	if (adapter_gpio_config[idx].drive == ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL)
		OUT_GPIO(adapter_gpio_config[idx].gpio_num);
}

static bb_value_t bcm2835gpio_read(void)
{
	unsigned int shift = adapter_gpio_config[ADAPTER_GPIO_IDX_TDO].gpio_num;
	uint32_t value = (GPIO_LEV >> shift) & 1;
	return value ^ (adapter_gpio_config[ADAPTER_GPIO_IDX_TDO].active_low ? BB_HIGH : BB_LOW);

}

static int bcm2835gpio_write(int tck, int tms, int tdi)
{
	uint32_t set = tck << adapter_gpio_config[ADAPTER_GPIO_IDX_TCK].gpio_num |
			tms << adapter_gpio_config[ADAPTER_GPIO_IDX_TMS].gpio_num |
			tdi << adapter_gpio_config[ADAPTER_GPIO_IDX_TDI].gpio_num;
	uint32_t clear = !tck << adapter_gpio_config[ADAPTER_GPIO_IDX_TCK].gpio_num |
			!tms << adapter_gpio_config[ADAPTER_GPIO_IDX_TMS].gpio_num |
			!tdi << adapter_gpio_config[ADAPTER_GPIO_IDX_TDI].gpio_num;

	GPIO_SET = set;
	GPIO_CLR = clear;

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

/* Requires push-pull drive mode for swclk and swdio */
static int bcm2835gpio_swd_write_fast(int swclk, int swdio)
{
	swclk = swclk ^ (adapter_gpio_config[ADAPTER_GPIO_IDX_SWCLK].active_low ? 1 : 0);
	swdio = swdio ^ (adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].active_low ? 1 : 0);

	uint32_t set = swclk << adapter_gpio_config[ADAPTER_GPIO_IDX_SWCLK].gpio_num |
					swdio << adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].gpio_num;
	uint32_t clear = !swclk << adapter_gpio_config[ADAPTER_GPIO_IDX_SWCLK].gpio_num |
					!swdio << adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].gpio_num;

	GPIO_SET = set;
	GPIO_CLR = clear;

	for (unsigned int i = 0; i < jtag_delay; i++)
		asm volatile ("");

	return ERROR_OK;
}

/* Generic mode that works for open-drain/open-source drive modes, but slower */
static int bcm2835gpio_swd_write_generic(int swclk, int swdio)
{
	set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO], swdio);
	set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWCLK], swclk); /* Write clock last */

	for (unsigned int i = 0; i < jtag_delay; ++i)
		asm volatile ("");

	return ERROR_OK;
}

/* (1) assert or (0) deassert reset lines */
static int bcm2835gpio_reset(int trst, int srst)
{
	/* As the "adapter reset_config" command keeps the srst and trst gpio drive
	 * mode settings in sync we can use our standard set_gpio_value() function
	 * that honours drive mode and active low.
	 */
	if (is_gpio_config_valid(ADAPTER_GPIO_IDX_SRST))
		set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_SRST], srst);

	if (is_gpio_config_valid(ADAPTER_GPIO_IDX_TRST))
		set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_TRST], trst);

	LOG_DEBUG("BCM2835 GPIO: bcm2835gpio_reset(%d, %d), trst_gpio: %d %d, srst_gpio: %d %d",
		trst, srst,
		adapter_gpio_config[ADAPTER_GPIO_IDX_TRST].chip_num, adapter_gpio_config[ADAPTER_GPIO_IDX_TRST].gpio_num,
		adapter_gpio_config[ADAPTER_GPIO_IDX_SRST].chip_num, adapter_gpio_config[ADAPTER_GPIO_IDX_SRST].gpio_num);
	return ERROR_OK;
}

static void bcm2835_swdio_drive(bool is_output)
{
	if (is_output) {
		if (is_gpio_config_valid(ADAPTER_GPIO_IDX_SWDIO_DIR))
			set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO_DIR], 1);
		OUT_GPIO(adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].gpio_num);
	} else {
		INP_GPIO(adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].gpio_num);
		if (is_gpio_config_valid(ADAPTER_GPIO_IDX_SWDIO_DIR))
			set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO_DIR], 0);
	}
}

static int bcm2835_swdio_read(void)
{
	unsigned int shift = adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].gpio_num;
	uint32_t value = (GPIO_LEV >> shift) & 1;
	return value ^ (adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].active_low ? 1 : 0);
}

static int bcm2835gpio_khz(int khz, int *jtag_speed)
{
	if (!khz) {
		LOG_DEBUG("BCM2835 GPIO: RCLK not supported");
		return ERROR_FAIL;
	}
	*jtag_speed = speed_coeff/khz - speed_offset;
	if (*jtag_speed < 0)
		*jtag_speed = 0;
	return ERROR_OK;
}

static int bcm2835gpio_speed_div(int speed, int *khz)
{
	*khz = speed_coeff/(speed + speed_offset);
	return ERROR_OK;
}

static int bcm2835gpio_speed(int speed)
{
	jtag_delay = speed;
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_speed_coeffs)
{
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], speed_coeff);
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], speed_offset);
	}

	command_print(CMD, "BCM2835 GPIO: speed_coeffs = %d, speed_offset = %d",
				  speed_coeff, speed_offset);
	return ERROR_OK;
}

COMMAND_HANDLER(bcm2835gpio_handle_peripheral_base)
{
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], bcm2835_peri_base);

	command_print(CMD, "BCM2835 GPIO: peripheral_base = 0x%08x",
				  bcm2835_peri_base);
	return ERROR_OK;
}

static const struct command_registration bcm2835gpio_subcommand_handlers[] = {
	{
		.name = "speed_coeffs",
		.handler = &bcm2835gpio_handle_speed_coeffs,
		.mode = COMMAND_CONFIG,
		.help = "SPEED_COEFF and SPEED_OFFSET for delay calculations.",
		.usage = "[SPEED_COEFF SPEED_OFFSET]",
	},
	{
		.name = "peripheral_base",
		.handler = &bcm2835gpio_handle_peripheral_base,
		.mode = COMMAND_CONFIG,
		.help = "peripheral base to access GPIOs (RPi1 0x20000000, RPi2 0x3F000000).",
		.usage = "[base]",
	},

	COMMAND_REGISTRATION_DONE
};

static const struct command_registration bcm2835gpio_command_handlers[] = {
	{
		.name = "bcm2835gpio",
		.mode = COMMAND_ANY,
		.help = "perform bcm2835gpio management",
		.chain = bcm2835gpio_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static bool bcm2835gpio_jtag_mode_possible(void)
{
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TCK))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TMS))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TDI))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_TDO))
		return false;
	return true;
}

static bool bcm2835gpio_swd_mode_possible(void)
{
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_SWCLK))
		return false;
	if (!is_gpio_config_valid(ADAPTER_GPIO_IDX_SWDIO))
		return false;
	return true;
}

static void bcm2835gpio_munmap(void)
{
	if (pio_base != MAP_FAILED) {
		munmap((void *)pio_base, sysconf(_SC_PAGE_SIZE));
		pio_base = MAP_FAILED;
	}

	if (pads_base != MAP_FAILED) {
		munmap((void *)pads_base, sysconf(_SC_PAGE_SIZE));
		pads_base = MAP_FAILED;
	}
}

static int bcm2835gpio_blink(int on)
{
	if (is_gpio_config_valid(ADAPTER_GPIO_IDX_LED))
		set_gpio_value(&adapter_gpio_config[ADAPTER_GPIO_IDX_LED], on);

	return ERROR_OK;
}

static struct bitbang_interface bcm2835gpio_bitbang = {
	.read = bcm2835gpio_read,
	.write = bcm2835gpio_write,
	.swdio_read = bcm2835_swdio_read,
	.swdio_drive = bcm2835_swdio_drive,
	.swd_write = bcm2835gpio_swd_write_generic,
	.blink = bcm2835gpio_blink,
};

static int bcm2835gpio_init(void)
{
	LOG_INFO("BCM2835 GPIO JTAG/SWD bitbang driver");

	bitbang_interface = &bcm2835gpio_bitbang;
	adapter_gpio_config = adapter_gpio_get_config();

	if (transport_is_jtag() && !bcm2835gpio_jtag_mode_possible()) {
		LOG_ERROR("Require tck, tms, tdi and tdo gpios for JTAG mode");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (transport_is_swd() && !bcm2835gpio_swd_mode_possible()) {
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

	pio_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, BCM2835_GPIO_BASE);

	if (pio_base == MAP_FAILED) {
		LOG_ERROR("mmap: %s", strerror(errno));
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	pads_base = mmap(NULL, sysconf(_SC_PAGE_SIZE), PROT_READ | PROT_WRITE,
				MAP_SHARED, dev_mem_fd, BCM2835_PADS_GPIO_0_27);

	if (pads_base == MAP_FAILED) {
		LOG_ERROR("mmap: %s", strerror(errno));
		bcm2835gpio_munmap();
		close(dev_mem_fd);
		return ERROR_JTAG_INIT_FAILED;
	}

	close(dev_mem_fd);

	/* set 4mA drive strength, slew rate limited, hysteresis on */
	initial_drive_strength_etc = pads_base[BCM2835_PADS_GPIO_0_27_OFFSET] & 0x1f;
	pads_base[BCM2835_PADS_GPIO_0_27_OFFSET] = 0x5a000008 + 1;

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

		if (adapter_gpio_config[ADAPTER_GPIO_IDX_SWCLK].drive == ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL &&
				adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].drive == ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL) {
			LOG_DEBUG("BCM2835 GPIO using fast mode for SWD write");
			bcm2835gpio_bitbang.swd_write = bcm2835gpio_swd_write_fast;
		} else {
			LOG_DEBUG("BCM2835 GPIO using generic mode for SWD write");
			bcm2835gpio_bitbang.swd_write = bcm2835gpio_swd_write_generic;
		}
	}

	initialize_gpio(ADAPTER_GPIO_IDX_SRST);
	initialize_gpio(ADAPTER_GPIO_IDX_LED);

	return ERROR_OK;
}

static int bcm2835gpio_quit(void)
{
	if (transport_is_jtag()) {
		restore_gpio(ADAPTER_GPIO_IDX_TDO);
		restore_gpio(ADAPTER_GPIO_IDX_TDI);
		restore_gpio(ADAPTER_GPIO_IDX_TCK);
		restore_gpio(ADAPTER_GPIO_IDX_TMS);
		restore_gpio(ADAPTER_GPIO_IDX_TRST);
	}

	if (transport_is_swd()) {
		/* Restore swdio/swdio_dir to their initial modes, even if that means
		 * connecting two outputs. Begin by making swdio an input so that the
		 * current and final states of swdio and swdio_dir do not have to be
		 * considered to calculate the safe restoration order.
		 */
		INP_GPIO(adapter_gpio_config[ADAPTER_GPIO_IDX_SWDIO].gpio_num);
		restore_gpio(ADAPTER_GPIO_IDX_SWDIO_DIR);
		restore_gpio(ADAPTER_GPIO_IDX_SWDIO);
		restore_gpio(ADAPTER_GPIO_IDX_SWCLK);
	}

	restore_gpio(ADAPTER_GPIO_IDX_SRST);
	restore_gpio(ADAPTER_GPIO_IDX_LED);

	/* Restore drive strength. MSB is password ("5A") */
	pads_base[BCM2835_PADS_GPIO_0_27_OFFSET] = 0x5A000000 | initial_drive_strength_etc;
	bcm2835gpio_munmap();

	return ERROR_OK;
}


static const char * const bcm2835_transports[] = { "jtag", "swd", NULL };

static struct jtag_interface bcm2835gpio_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};
struct adapter_driver bcm2835gpio_adapter_driver = {
	.name = "bcm2835gpio",
	.transports = bcm2835_transports,
	.commands = bcm2835gpio_command_handlers,

	.init = bcm2835gpio_init,
	.quit = bcm2835gpio_quit,
	.reset = bcm2835gpio_reset,
	.speed = bcm2835gpio_speed,
	.khz = bcm2835gpio_khz,
	.speed_div = bcm2835gpio_speed_div,

	.jtag_ops = &bcm2835gpio_interface,
	.swd_ops = &bitbang_swd,
};
