/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2007 by Pavel Chromy                                    *
 *   chromy@asix.cz                                                        *
 ***************************************************************************/
#ifndef OCL_H
#define OCL_H

/* command/response mask */
#define OCL_CMD_MASK 0xFFFF0000L

/* commands */
#define OCL_FLASH_BLOCK 0x0CFB0000L
#define OCL_ERASE_BLOCK 0x0CEB0000L
#define OCL_ERASE_ALL 0x0CEA0000L
#define OCL_PROBE 0x0CBE0000L

/* responses */
#define OCL_CMD_DONE 0x0ACD0000L
#define OCL_CMD_ERR 0x0ACE0000L
#define OCL_CHKS_FAIL 0x0ACF0000L
#define OCL_BUFF_OVER 0x0AB00000L

#define OCL_CHKS_INIT 0xC100CD0CL

#endif /* OCL_H */
