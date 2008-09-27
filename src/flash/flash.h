/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
#ifndef FLASH_H
#define FLASH_H

#include "target.h"
#include "image.h"

#define FLASH_MAX_ERROR_STR	(128)

typedef struct flash_sector_s
{
	u32 offset;
	u32 size;
	int is_erased;
	int is_protected;
} flash_sector_t;

struct flash_bank_s;

typedef struct flash_driver_s
{
	char *name;
	int (*register_commands)(struct command_context_s *cmd_ctx);
	int (*flash_bank_command)(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);

	/* use flash_driver_erase() wrapper to invoke */
	int (*erase)(struct flash_bank_s *bank, int first, int last);

	/* use flash_driver_protect() wrapper to invoke */
	int (*protect)(struct flash_bank_s *bank, int set, int first, int last);

	/* use the flash_driver_write() wrapper to invoke. */
	int (*write)(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);

	int (*probe)(struct flash_bank_s *bank);
	int (*erase_check)(struct flash_bank_s *bank);
	int (*protect_check)(struct flash_bank_s *bank);
	int (*info)(struct flash_bank_s *bank, char *buf, int buf_size);
	int (*auto_probe)(struct flash_bank_s *bank);
} flash_driver_t;

typedef struct flash_bank_s
{
	target_t *target;
	flash_driver_t *driver;
	void *driver_priv;
	int bank_number;
	u32 base;
	u32 size;
	int chip_width;
	int bus_width;
	int num_sectors;
	flash_sector_t *sectors;
	struct flash_bank_s *next;
} flash_bank_t;

extern int flash_register_commands(struct command_context_s *cmd_ctx);
extern int flash_init_drivers(struct command_context_s *cmd_ctx);

extern int flash_erase_address_range(target_t *target, u32 addr, u32 length);
extern int flash_write(target_t *target, image_t *image, u32 *written, int erase);
extern void flash_set_dirty(void);
extern int flash_get_bank_count(void);
extern int default_flash_blank_check(struct flash_bank_s *bank);
extern int default_flash_mem_blank_check(struct flash_bank_s *bank);

extern flash_bank_t *get_flash_bank_by_num(int num);
extern flash_bank_t *get_flash_bank_by_num_noprobe(int num);
extern flash_bank_t *get_flash_bank_by_addr(target_t *target, u32 addr);

#define ERROR_FLASH_BANK_INVALID			(-900)
#define ERROR_FLASH_SECTOR_INVALID			(-901)
#define ERROR_FLASH_OPERATION_FAILED		(-902)
#define ERROR_FLASH_DST_OUT_OF_BANK			(-903)
#define ERROR_FLASH_DST_BREAKS_ALIGNMENT	(-904)
#define ERROR_FLASH_BUSY					(-905)
#define ERROR_FLASH_SECTOR_NOT_ERASED		(-906)
#define ERROR_FLASH_BANK_NOT_PROBED			(-907)

#endif /* FLASH_H */
