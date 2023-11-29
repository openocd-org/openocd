/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_PLD_PLD_H
#define OPENOCD_PLD_PLD_H

#include <helper/command.h>

struct pld_device;

#define __PLD_CREATE_COMMAND(name) \
	COMMAND_HELPER(name, struct pld_device *pld)

struct pld_ipdbg_hub {
	struct jtag_tap *tap;
	unsigned int user_ir_code;
};

int pld_has_jtagspi_instruction(struct pld_device *device, bool *has_instruction);
int pld_get_jtagspi_userircode(struct pld_device *pld_device, unsigned int *ir);

int pld_get_jtagspi_stuff_bits(struct pld_device *pld_device, unsigned int *facing_read_bits,
							unsigned int *trailing_write_bits);
int pld_connect_spi_to_jtag(struct pld_device *pld_device);
int pld_disconnect_spi_from_jtag(struct pld_device *pld_device);

struct pld_driver {
	const char *name;
	__PLD_CREATE_COMMAND((*pld_create_command));
	const struct command_registration *commands;
	int (*load)(struct pld_device *pld_device, const char *filename);
	int (*get_ipdbg_hub)(int user_num, struct pld_device *pld_device, struct pld_ipdbg_hub *hub);
	int (*has_jtagspi_instruction)(struct pld_device *device, bool *has_instruction);
	int (*get_jtagspi_userircode)(struct pld_device *pld_device, unsigned int *ir);
	int (*connect_spi_to_jtag)(struct pld_device *pld_device);
	int (*disconnect_spi_from_jtag)(struct pld_device *pld_device);
	int (*get_stuff_bits)(struct pld_device *pld_device, unsigned int *facing_read_bits,
		unsigned int *trailing_write_bits);
};

#define PLD_CREATE_COMMAND_HANDLER(name) \
	static __PLD_CREATE_COMMAND(name)

struct pld_device {
	struct pld_driver *driver;
	void *driver_priv;
	struct pld_device *next;
	char *name;
};

int pld_register_commands(struct command_context *cmd_ctx);

struct pld_device *get_pld_device_by_num(int num);
struct pld_device *get_pld_device_by_name(const char *name);
struct pld_device *get_pld_device_by_name_or_numstr(const char *str);

#define ERROR_PLD_DEVICE_INVALID        (-1000)
#define ERROR_PLD_FILE_LOAD_FAILED      (-1001)

extern struct pld_driver efinix_pld;
extern struct pld_driver gatemate_pld;
extern struct pld_driver gowin_pld;
extern struct pld_driver intel_pld;
extern struct pld_driver lattice_pld;
extern struct pld_driver virtex2_pld;

#endif /* OPENOCD_PLD_PLD_H */
