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

#include "jtag.h"

typedef struct arm_jtag_s
{
	jtag_tap_t *tap;

	uint32_t scann_size;
	uint32_t scann_instr;
	uint32_t cur_scan_chain;

	uint32_t intest_instr;
} arm_jtag_t;

int arm_jtag_set_instr(arm_jtag_t *jtag_info,
		uint32_t new_instr, void *verify_capture);
int arm_jtag_scann(arm_jtag_t *jtag_info, uint32_t new_scan_chain);
int arm_jtag_setup_connection(arm_jtag_t *jtag_info);

/* JTAG buffers to host, be and le buffers, flipping variants */
int arm_jtag_buf_to_u32_flip(uint8_t *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_le32_flip(uint8_t *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_le16_flip(uint8_t *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_be32_flip(uint8_t *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_be16_flip(uint8_t *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_8_flip(uint8_t *in_buf, void *priv, struct scan_field_s *field);

/* JTAG buffers to host, be and le buffers */
int arm_jtag_buf_to_u32(uint8_t *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_le32(uint8_t *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_le16(uint8_t *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_be32(uint8_t *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_be16(uint8_t *in_buf, void *priv, struct scan_field_s *field);
int arm_jtag_buf_to_8(uint8_t *in_buf, void *priv, struct scan_field_s *field);


/* use this as a static so we can inline it in -O3 and refer to it via a pointer  */
static __inline__ void arm7flip32(jtag_callback_data_t arg)
{
  uint8_t *in = (uint8_t *)arg;
  *((uint32_t *)in) = flip_u32(le_to_h_u32(in), 32);
}

static __inline__ void arm_le_to_h_u32(jtag_callback_data_t arg)
{
  uint8_t *in = (uint8_t *)arg;
  *((uint32_t *)in) = le_to_h_u32(in);
}


#endif /* ARM_JTAG */

