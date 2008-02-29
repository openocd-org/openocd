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

/*
 * S3C24XX Series OpenOCD NAND Flash controller support.
 *
 * Many thanks to Simtec Electronics for sponsoring this work.
 */

#include "target.h"
#include "s3c24xx_regs_nand.h"

typedef struct s3c24xx_nand_controller_s
{
	struct target_s *target;
	
	/* register addresses */
	u32		 cmd;
	u32		 addr;
	u32		 data;
	u32		 nfstat; 
} s3c24xx_nand_controller_t;

/* Default to using the un-translated NAND register based address */
#undef S3C2410_NFREG
#define S3C2410_NFREG(x) ((x) + 0x4e000000)

extern s3c24xx_nand_controller_t *s3c24xx_nand_device_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct nand_device_s *device);

extern int s3c24xx_register_commands(struct command_context_s *cmd_ctx);
extern int s3c24xx_reset(struct nand_device_s *device);
extern int s3c24xx_command(struct nand_device_s *device, u8 command);
extern int s3c24xx_address(struct nand_device_s *device, u8 address);
extern int s3c24xx_write_data(struct nand_device_s *device, u16 data);
extern int s3c24xx_read_data(struct nand_device_s *device, void *data);
extern int s3c24xx_controller_ready(struct nand_device_s *device, int tout);

#define s3c24xx_write_page NULL
#define s3c24xx_read_page NULL

/* code shared between different controllers */

extern int s3c2440_nand_ready(struct nand_device_s *device, int timeout);

extern int s3c2440_read_block_data(struct nand_device_s *, u8 *data, int data_size);
extern int s3c2440_write_block_data(struct nand_device_s *, u8 *data, int data_size);
