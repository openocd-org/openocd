/***************************************************************************
 *   Copyright (C) 2007, 2008 by Ben Dooks                                 *
 *   ben@fluff.org                                                         *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef S3C24xx_NAND_H
#define S3C24xx_NAND_H

/*
 * S3C24XX Series OpenOCD NAND Flash controller support.
 *
 * Many thanks to Simtec Electronics for sponsoring this work.
 */

#include "nand.h"
#include "s3c24xx_regs_nand.h"

typedef struct s3c24xx_nand_controller_s
{
	struct target_s *target;

	/* register addresses */
	uint32_t		 cmd;
	uint32_t		 addr;
	uint32_t		 data;
	uint32_t		 nfstat;
} s3c24xx_nand_controller_t;

/* Default to using the un-translated NAND register based address */
#undef S3C2410_NFREG
#define S3C2410_NFREG(x) ((x) + 0x4e000000)

s3c24xx_nand_controller_t *s3c24xx_nand_device_command(
			struct command_context_s *cmd_ctx, char *cmd,
			char **args, int argc, struct nand_device_s *nand);

int s3c24xx_register_commands(struct command_context_s *cmd_ctx);

int s3c24xx_reset(struct nand_device_s *nand);

int s3c24xx_command(struct nand_device_s *nand, uint8_t command);
int s3c24xx_address(struct nand_device_s *nand, uint8_t address);

int s3c24xx_write_data(struct nand_device_s *nand, uint16_t data);
int s3c24xx_read_data(struct nand_device_s *nand, void *data);

int s3c24xx_controller_ready(struct nand_device_s *nand, int tout);

#define s3c24xx_write_page NULL
#define s3c24xx_read_page NULL

/* code shared between different controllers */

int s3c2440_nand_ready(struct nand_device_s *nand, int timeout);

int s3c2440_read_block_data(struct nand_device_s *nand,
		uint8_t *data, int data_size);
int s3c2440_write_block_data(struct nand_device_s *nand,
		uint8_t *data, int data_size);

#endif // S3C24xx_NAND_H
