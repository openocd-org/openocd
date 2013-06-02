/***************************************************************************
 *   Copyright (C) 2010 by Peter Korsgaard <jacmet@sunsite.dk>             *
 *   Heavily based on s3c2412.c by Ben Dooks <ben@fluff.org>               *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "s3c24xx.h"
/* s3c64xx uses another base address for the nand controller than 24xx */
#undef S3C2410_NFREG
#define S3C2410_NFREG(x) ((x) + 0x70200000)

NAND_DEVICE_COMMAND_HANDLER(s3c6400_nand_device_command)
{
	struct s3c24xx_nand_controller *info;
	CALL_S3C24XX_DEVICE_COMMAND(nand, &info);

	/* fill in the address fields for the core device */
	info->cmd = S3C2440_NFCMD;
	info->addr = S3C2440_NFADDR;
	info->data = S3C2440_NFDATA;
	info->nfstat = S3C2412_NFSTAT;

	return ERROR_OK;
}

static int s3c6400_init(struct nand_device *nand)
{
	struct target *target = nand->target;

	target_write_u32(target, S3C2410_NFCONF,
			 S3C2440_NFCONF_TACLS(3) |
			 S3C2440_NFCONF_TWRPH0(7) |
			 S3C2440_NFCONF_TWRPH1(7) | 4);

	target_write_u32(target, S3C2440_NFCONT,
			 S3C2412_NFCONT_INIT_MAIN_ECC |
			 S3C2440_NFCONT_ENABLE);

	return ERROR_OK;
}

struct nand_flash_controller s3c6400_nand_controller = {
	.name = "s3c6400",
	.nand_device_command = &s3c6400_nand_device_command,
	.init = &s3c6400_init,
	.reset = &s3c24xx_reset,
	.command = &s3c24xx_command,
	.address = &s3c24xx_address,
	.write_data = &s3c24xx_write_data,
	.read_data = &s3c24xx_read_data,
	.write_page = s3c24xx_write_page,
	.read_page = s3c24xx_read_page,
	.write_block_data = &s3c2440_write_block_data,
	.read_block_data = &s3c2440_read_block_data,
	.nand_ready = &s3c2440_nand_ready,
};
