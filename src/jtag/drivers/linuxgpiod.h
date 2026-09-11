/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_JTAG_DRIVERS_LINUXGPIOD_H
#define OPENOCD_JTAG_DRIVERS_LINUXGPIOD_H

int linuxgpiod_reset(int trst, int srst);
int linuxgpiod_quit(void);
int linuxgpiod_helper_get_line(enum adapter_gpio_config_index idx);

#endif
