// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2007,2008 Øyvind Harboe <oyvind.harboe@zylin.com>       *
 *   Copyright (C) 2008 by Spencer Oliver <spen@spen-soft.co.uk>           *
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <helper/types.h>
#include "core.h"
#include "driver.h"

static struct nand_flash_controller *nand_flash_controllers[] = {
	// Keep in alphabetic order the list of drivers
	&at91sam9_nand_controller,
	&davinci_nand_controller,
	&imx31_nand_flash_controller,
	&lpc3180_nand_controller,
	&lpc32xx_nand_controller,
	&mxc_nand_flash_controller,
	&nonce_nand_controller,
	&nuc910_nand_controller,
	&orion_nand_controller,
	&s3c2410_nand_controller,
	&s3c2412_nand_controller,
	&s3c2440_nand_controller,
	&s3c2443_nand_controller,
	&s3c6400_nand_controller,
};

struct nand_flash_controller *nand_driver_find_by_name(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(nand_flash_controllers); i++) {
		struct nand_flash_controller *controller = nand_flash_controllers[i];
		if (strcmp(name, controller->name) == 0)
			return controller;
	}
	return NULL;
}
int nand_driver_walk(nand_driver_walker_t f, void *x)
{
	for (size_t i = 0; i < ARRAY_SIZE(nand_flash_controllers); i++) {
		int retval = (*f)(nand_flash_controllers[i], x);
		if (retval != ERROR_OK)
			return retval;
	}
	return ERROR_OK;
}
