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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm_jtag.h"

#include "binarybuffer.h"
#include "log.h"
#include "jtag.h"

#include <stdlib.h>

#if 0
#define _ARM_JTAG_SCAN_N_CHECK_
#endif

int arm_jtag_set_instr(arm_jtag_t *jtag_info, u32 new_instr,  in_handler_t handler)
{
	jtag_tap_t *tap;
	tap = jtag_info->tap;
	if (tap==NULL)
		return ERROR_FAIL;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != new_instr)
	{
		scan_field_t field;
		u8 t[4];

		field.tap = tap;
		field.num_bits = tap->ir_length;
		field.out_value = t;
		buf_set_u32(field.out_value, 0, field.num_bits, new_instr);
		field.out_mask = NULL;
		field.in_value = NULL;
		field.in_check_value = NULL;
		field.in_check_mask = NULL;
		field.in_handler = handler;
		field.in_handler_priv = NULL;
		jtag_add_ir_scan(1, &field, -1);
	}

	return ERROR_OK;
}

int arm_jtag_scann(arm_jtag_t *jtag_info, u32 new_scan_chain)
{
	int retval = ERROR_OK;
	if(jtag_info->cur_scan_chain != new_scan_chain)
	{
		u32 values[1];
		int num_bits[1];

		values[0]=new_scan_chain;
		num_bits[0]=jtag_info->scann_size;

		if((retval = arm_jtag_set_instr(jtag_info, jtag_info->scann_instr, NULL)) != ERROR_OK)
		{
			return retval;
		}

		jtag_add_dr_out(jtag_info->tap,
				1,
				num_bits,
				values,
				-1);

		jtag_info->cur_scan_chain = new_scan_chain;
	}

	return retval;
}

int arm_jtag_reset_callback(enum jtag_event event, void *priv)
{
	arm_jtag_t *jtag_info = priv;

	if (event == JTAG_TRST_ASSERTED)
	{
		jtag_info->cur_scan_chain = 0;
	}

	return ERROR_OK;
}

int arm_jtag_setup_connection(arm_jtag_t *jtag_info)
{
	jtag_info->scann_instr = 0x2;
	jtag_info->cur_scan_chain = 0;
	jtag_info->intest_instr = 0xc;

	return jtag_register_event_callback(arm_jtag_reset_callback, jtag_info);
}

/* read JTAG buffer into host-endian u32, flipping bit-order */
int arm_jtag_buf_to_u32_flip(u8 *in_buf, void *priv, struct scan_field_s *field)
{
	u32 *dest = priv;
	*dest = flip_u32(le_to_h_u32(in_buf), 32);
	return ERROR_OK;
}

/* read JTAG buffer into little-endian u32, flipping bit-order */
int arm_jtag_buf_to_le32_flip(u8 *in_buf, void *priv, struct scan_field_s *field)
{
	h_u32_to_le(((u8*)priv), flip_u32(le_to_h_u32(in_buf), 32));
	return ERROR_OK;
}

/* read JTAG buffer into little-endian u16, flipping bit-order */
int arm_jtag_buf_to_le16_flip(u8 *in_buf, void *priv, struct scan_field_s *field)
{
	h_u16_to_le(((u8*)priv), flip_u32(le_to_h_u32(in_buf), 32) & 0xffff);
	return ERROR_OK;
}

/* read JTAG buffer into big-endian u32, flipping bit-order */
int arm_jtag_buf_to_be32_flip(u8 *in_buf, void *priv, struct scan_field_s *field)
{
	h_u32_to_be(((u8*)priv), flip_u32(le_to_h_u32(in_buf), 32));
	return ERROR_OK;
}

/* read JTAG buffer into big-endian u16, flipping bit-order */
int arm_jtag_buf_to_be16_flip(u8 *in_buf, void *priv, struct scan_field_s *field)
{
	h_u16_to_be(((u8*)priv), flip_u32(le_to_h_u32(in_buf), 32) & 0xffff);
	return ERROR_OK;
}

/* read JTAG buffer into u8, flipping bit-order */
int arm_jtag_buf_to_8_flip(u8 *in_buf, void *priv, struct scan_field_s *field)
{
	u8 *dest = priv;
	*dest = flip_u32(le_to_h_u32(in_buf), 32) & 0xff;
	return ERROR_OK;
}

/* not-flipping variants */
/* read JTAG buffer into host-endian u32 */
int arm_jtag_buf_to_u32(u8 *in_buf, void *priv, struct scan_field_s *field)
{
	u32 *dest = priv;
	*dest = le_to_h_u32(in_buf);
	return ERROR_OK;
}

/* read JTAG buffer into little-endian u32 */
int arm_jtag_buf_to_le32(u8 *in_buf, void *priv, struct scan_field_s *field)
{
	h_u32_to_le(((u8*)priv), le_to_h_u32(in_buf));
	return ERROR_OK;
}

/* read JTAG buffer into little-endian u16 */
int arm_jtag_buf_to_le16(u8 *in_buf, void *priv, struct scan_field_s *field)
{
	h_u16_to_le(((u8*)priv), le_to_h_u32(in_buf) & 0xffff);
	return ERROR_OK;
}

/* read JTAG buffer into big-endian u32 */
int arm_jtag_buf_to_be32(u8 *in_buf, void *priv, struct scan_field_s *field)
{
	h_u32_to_be(((u8*)priv), le_to_h_u32(in_buf));
	return ERROR_OK;
}

/* read JTAG buffer into big-endian u16 */
int arm_jtag_buf_to_be16(u8 *in_buf, void *priv, struct scan_field_s *field)
{
	h_u16_to_be(((u8*)priv), le_to_h_u32(in_buf) & 0xffff);
	return ERROR_OK;
}

/* read JTAG buffer into u8 */
int arm_jtag_buf_to_8(u8 *in_buf, void *priv, struct scan_field_s *field)
{
	u8 *dest = priv;
	*dest = le_to_h_u32(in_buf) & 0xff;
	return ERROR_OK;
}
