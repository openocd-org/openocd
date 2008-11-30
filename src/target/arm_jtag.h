/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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
#ifndef ARM_JTAG
#define ARM_JTAG

#include "types.h"
#include "jtag.h"

typedef struct arm_jtag_s
{
	jtag_tap_t *tap;
	
	int scann_size;
	u32 scann_instr;
	int cur_scan_chain;
	
	u32 intest_instr;
} arm_jtag_t;

extern int arm_jtag_set_instr(arm_jtag_t *jtag_info, u32 new_instr, in_handler_t handler);
extern int arm_jtag_scann(arm_jtag_t *jtag_info, u32 new_scan_chain);
extern int arm_jtag_setup_connection(arm_jtag_t *jtag_info);

/* JTAG buffers to host, be and le buffers, flipping variants */
int arm_jtag_buf_to_u32_flip(u8 *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_le32_flip(u8 *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_le16_flip(u8 *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_be32_flip(u8 *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_be16_flip(u8 *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_8_flip(u8 *in_buf, void *priv, struct scan_field_s *field);

/* JTAG buffers to host, be and le buffers */
int arm_jtag_buf_to_u32(u8 *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_le32(u8 *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_le16(u8 *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_be32(u8 *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_be16(u8 *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_8(u8 *in_buf, void *priv, struct scan_field_s *field);

#endif /* ARM_JTAG */

