// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "imp.h"

/**
 * The list of built-in flash drivers.
 * @todo Make this dynamically extendable with loadable modules.
 */
static const struct flash_driver * const flash_drivers[] = {
	&aduc702x_flash,
	&aducm360_flash,
	&ambiqmicro_flash,
	&at91sam3_flash,
	&at91sam4_flash,
	&at91sam4l_flash,
	&at91sam7_flash,
	&at91samd_flash,
	&ath79_flash,
	&atsame5_flash,
	&atsamv_flash,
	&avr_flash,
	&bluenrgx_flash,
	&cc3220sf_flash,
	&cc26xx_flash,
	&cfi_flash,
	&dsp5680xx_flash,
	&dw_spi_flash,
	&efm32_flash,
	&em357_flash,
	&eneispif_flash,
	&esirisc_flash,
	&faux_flash,
	&fm3_flash,
	&fm4_flash,
	&fespi_flash,
	&jtagspi_flash,
	&kinetis_flash,
	&kinetis_ke_flash,
	&lpc2000_flash,
	&lpc288x_flash,
	&lpc2900_flash,
	&lpcspifi_flash,
	&max32xxx_flash,
	&mdr_flash,
	&mrvlqspi_flash,
	&msp432_flash,
	&mspm0_flash,
	&niietcm4_flash,
	&npcx_flash,
	&nrf5_flash,
	&nrf51_flash,
	&numicro_flash,
	&ocl_flash,
	&pic32mx_flash,
	&psoc4_flash,
	&psoc5lp_flash,
	&psoc5lp_eeprom_flash,
	&psoc5lp_nvl_flash,
	&psoc6_flash,
	&qn908x_flash,
	&renesas_rpchf_flash,
	&rp2040_flash,
	&sh_qspi_flash,
	&sim3x_flash,
	&stellaris_flash,
	&stm32f1x_flash,
	&stm32f2x_flash,
	&stm32lx_flash,
	&stm32l4x_flash,
	&stm32h7x_flash,
	&stmsmi_flash,
	&stmqspi_flash,
	&str7x_flash,
	&str9x_flash,
	&str9xpec_flash,
	&swm050_flash,
	&tms470_flash,
	&virtual_flash,
	&xcf_flash,
	&xmc1xxx_flash,
	&xmc4xxx_flash,
	&w600_flash,
	&rsl10_flash,
	NULL,
};

const struct flash_driver *flash_driver_find_by_name(const char *name)
{
	for (unsigned int i = 0; flash_drivers[i]; i++) {
		if (strcmp(name, flash_drivers[i]->name) == 0)
			return flash_drivers[i];
	}
	return NULL;
}
