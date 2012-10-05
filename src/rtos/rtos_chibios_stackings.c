/***************************************************************************
 *   Copyright (C) 2012 by Matthias Blaicher                               *
 *   Matthias Blaicher - matthias@blaicher.com                             *
 *                                                                         *
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
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

#include "rtos.h"

static const struct stack_register_offset rtos_chibios_arm_v7m_stack_offsets[] = {
	{ -1,   32 },		/* r0   */
	{ -1,   32 },		/* r1   */
	{ -1,   32 },		/* r2   */
	{ -1,   32 },		/* r3   */
	{ 0x00, 32 },		/* r4   */
	{ 0x04, 32 },		/* r5   */
	{ 0x08, 32 },		/* r6   */
	{ 0x0c, 32 },		/* r7   */
	{ 0x10, 32 },		/* r8   */
	{ 0x14, 32 },		/* r9   */
	{ 0x18, 32 },		/* r10  */
	{ 0x1c, 32 },		/* r11  */
	{ -1,   32 },		/* r12  */
	{ -2,   32 },		/* sp   */
	{ -1,   32 },		/* lr   */
	{ 0x20, 32 },		/* pc   */
	{ -1,   96 },		/* FPA1 */
	{ -1,   96 },		/* FPA2 */
	{ -1,   96 },		/* FPA3 */
	{ -1,   96 },		/* FPA4 */
	{ -1,   96 },		/* FPA5 */
	{ -1,   96 },		/* FPA6 */
	{ -1,   96 },		/* FPA7 */
	{ -1,   96 },		/* FPA8 */
	{ -1,   32 },		/* FPS  */
	{ -1,   32 },		/* xPSR */
};

const struct rtos_register_stacking rtos_chibios_arm_v7m_stacking = {
	0x24,					/* stack_registers_size */
	-1,					/* stack_growth_direction */
	26,					/* num_output_registers */
	0,					/* stack_alignment */
	rtos_chibios_arm_v7m_stack_offsets	/* register_offsets */
};
