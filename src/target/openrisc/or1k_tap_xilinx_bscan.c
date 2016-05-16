/***************************************************************************
 *   Copyright (C) 2013 by Sergio Chico                                    *
 *   sergio.chico@gmail.com                                                *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "or1k_tap.h"
#include "or1k.h"

#include <jtag/jtag.h>

#define OR1K_XILINX_TAP_INST_USER1	0x02

static int or1k_tap_xilinx_bscan_init(struct or1k_jtag *jtag_info)
{
	LOG_DEBUG("Initialising Xilinx Internal JTAG TAP");

	/* Put TAP into state where it can talk to the debug interface
	 * by shifting in correct value to IR.
	 */

	/* Ensure TAP is reset - maybe not necessary*/
	jtag_add_tlr();

	struct jtag_tap *tap = jtag_info->tap;
	struct scan_field field;
	uint8_t ir_value = OR1K_XILINX_TAP_INST_USER1;

	field.num_bits = tap->ir_length;
	field.out_value = &ir_value;
	field.in_value = NULL;

	jtag_add_ir_scan(tap, &field, TAP_IDLE);

	return jtag_execute_queue();
}

static struct or1k_tap_ip xilinx_bscan_tap = {
	.name = "xilinx_bscan",
	.init = or1k_tap_xilinx_bscan_init,
};

int or1k_tap_xilinx_bscan_register(void)
{
	list_add_tail(&xilinx_bscan_tap.list, &tap_list);
	return 0;
}
