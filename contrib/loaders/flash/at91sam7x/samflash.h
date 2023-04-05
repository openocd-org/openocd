/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2007 by Pavel Chromy                                    *
 *   chromy@asix.cz                                                        *
 ***************************************************************************/
#ifndef samflashH
#define samflashH

#include "platform.h"

#define FLASH_AREA_ADDR 0x100000

#define FLASH_STAT_OK 0
#define FLASH_STAT_PROGE 1
#define FLASH_STAT_LOCKE 2
#define FLASH_STAT_VERIFE 3
#define FLASH_STAT_INITE 4

extern unsigned int flash_page_count;
extern unsigned int flash_page_size; /* words */

/* detect chip and set loader parameters */
int flash_init(void);

/* program single flash page */
int flash_page_program(uint32 *data, int page_num);

/* erase whole chip */
int flash_erase_all(void);

/* verify written data */
int flash_verify(uint32 adr, unsigned int len, uint8 *src);

#endif
