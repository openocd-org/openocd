/***************************************************************************
 *   Copyright (C) 2009 by Mathias Kuester                                 *
 *   mkdorg@users.sourceforge.net                                          *
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
#ifndef DSP563XX_H
#define DSP563XX_H

#include <jtag/jtag.h>

#define DSP563XX_NUMCOREREGS	44

struct mcu_jtag
{
	struct jtag_tap *tap;
};

struct dsp563xx_pipeline_context
{
	/* PIL Register */
	uint32_t once_opilr;
	/* PDB Register */
	uint32_t once_opdbr;
};

struct dsp563xx_common
{
	struct mcu_jtag jtag_info;
	struct reg_cache *core_cache;
	uint32_t core_regs[DSP563XX_NUMCOREREGS];

	struct dsp563xx_pipeline_context pipeline_context;

	/* register cache to processor synchronization */
	int (*read_core_reg) (struct target * target, int num);
	int (*write_core_reg) (struct target * target, int num);
};

struct dsp563xx_core_reg
{
	uint32_t num;
	char *name;
	uint32_t size;
	uint32_t r_cmd;
	uint32_t w_cmd;
	struct target *target;
	struct dsp563xx_common *dsp563xx_common;
};

static inline struct dsp563xx_common *target_to_dsp563xx(struct target *target)
{
	return target->arch_info;
}

int dsp563xx_write_ir(struct jtag_tap *tap, uint8_t * ir_in, uint8_t * ir_out,
		      int ir_len, int rti);
int dsp563xx_write_dr(struct jtag_tap *tap, uint8_t * dr_in, uint8_t * dr_out,
		      int dr_len, int rti);
int dsp563xx_write_ir_u8(struct jtag_tap *tap, uint8_t * ir_in, uint8_t ir_out,
			 int ir_len, int rti);
int dsp563xx_write_dr_u8(struct jtag_tap *tap, uint8_t * ir_in, uint8_t ir_out,
			 int dr_len, int rti);
int dsp563xx_write_ir_u16(struct jtag_tap *tap, uint16_t * ir_in, uint16_t ir_out,
			  int ir_len, int rti);
int dsp563xx_write_dr_u16(struct jtag_tap *tap, uint16_t * ir_in, uint16_t ir_out,
			  int dr_len, int rti);
int dsp563xx_write_ir_u32(struct jtag_tap *tap, uint32_t * ir_in, uint32_t ir_out,
			  int ir_len, int rti);
int dsp563xx_write_dr_u32(struct jtag_tap *tap, uint32_t * ir_in, uint32_t ir_out,
			  int dr_len, int rti);

int dsp563xx_execute_queue(void);

#endif /* DSP563XX_H */
