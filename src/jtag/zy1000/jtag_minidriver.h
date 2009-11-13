/***************************************************************************
 *   Copyright (C) 2007-2008 by Øyvind Harboe                              *
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

#include <cyg/hal/hal_io.h>             // low level i/o

//#define VERBOSE(a) a
#define VERBOSE(a)

/* used to test manual mode */
#define TEST_MANUAL() 0

#if 0
int  diag_printf(const char *fmt, ...);
#define ZY1000_POKE(a, b) HAL_WRITE_UINT32(a, b); diag_printf("poke 0x%08x,0x%08x\n", a, b)
#define ZY1000_PEEK(a, b) HAL_READ_UINT32(a, b); diag_printf("peek 0x%08x = 0x%08x\n", a, b)
#else
#define ZY1000_POKE(a, b) HAL_WRITE_UINT32(a, b)
#define ZY1000_PEEK(a, b) HAL_READ_UINT32(a, b)
#endif

// FIFO empty?
static __inline__ void waitIdle(void)
{
	cyg_uint32 empty;
	do
	{
		ZY1000_PEEK(ZY1000_JTAG_BASE + 0x10, empty);
	} while ((empty & 0x100) == 0);
}

static __inline__ void waitQueue(void)
{
//	waitIdle();
}

static void sampleShiftRegister(void)
{
#if 0
	cyg_uint32 dummy;
	waitIdle();
	ZY1000_PEEK(ZY1000_JTAG_BASE + 0xc, dummy);
#endif
}

/* -O3 will inline this for us */
static void setCurrentState(enum tap_state state)
{
	cyg_uint32 a;
	a = state;
	int repeat = 0;
	if (state == TAP_RESET)
	{
		// The FPGA nor we know the current state of the CPU TAP
		// controller. This will move it to TAP for sure.
		//
		// 5 should be enough here, 7 is what OpenOCD uses
		repeat = 7;
	}
	waitQueue();
	sampleShiftRegister();
	ZY1000_POKE(ZY1000_JTAG_BASE + 0x8, (repeat << 8) | (a << 4) | a);

}

/*
 * Enter state and cause repeat transitions *out* of that state. So if the endState != state, then
 * the transition from state to endState counts as a transition out of state.
 */
static __inline__ void shiftValueInner(const enum tap_state state, const enum tap_state endState, int repeat, cyg_uint32 value)
{
	cyg_uint32 a,b;
	a = state;
	b = endState;
	waitQueue();
	sampleShiftRegister();
	ZY1000_POKE(ZY1000_JTAG_BASE + 0xc, value);
#if 1
#if TEST_MANUAL()
	if ((state == TAP_DRSHIFT) && (endState != TAP_DRSHIFT))
	{
		int i;
		setCurrentState(state);
		for (i = 0; i < repeat; i++)
		{
			int tms;
			tms = 0;
			if ((i == repeat-1) && (state != endState))
			{
				tms = 1;
			}
			/* shift out value */
			waitIdle();
			ZY1000_POKE(ZY1000_JTAG_BASE + 0x28, (((value >> i)&1) << 1) | tms);
		}
		waitIdle();
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x28, 0);
		waitIdle();
		//ZY1000_POKE(ZY1000_JTAG_BASE + 0x20, TAP_DRSHIFT); // set this state and things break => expected
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x20, TAP_DRPAUSE); // set this and things will work => expected. Not setting this is not sufficient to make things break.
		setCurrentState(endState);
	} else
	{
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x8, (repeat << 8) | (a << 4) | b);
	}
#else
	/* fast version */
	ZY1000_POKE(ZY1000_JTAG_BASE + 0x8, (repeat << 8) | (a << 4) | b);
#endif
#else
	/* maximum debug version */
	if ((repeat > 0) && ((state == TAP_DRSHIFT)||(state == TAP_SI)))
	{
		int i;
		/* sample shift register for every bit. */
		for (i = 0; i < repeat-1; i++)
		{
			sampleShiftRegister();
			ZY1000_POKE(ZY1000_JTAG_BASE + 0xc, value >> i);
			ZY1000_POKE(ZY1000_JTAG_BASE + 0x8, (1 << 8) | (a << 4) | a);
		}
		sampleShiftRegister();
		ZY1000_POKE(ZY1000_JTAG_BASE + 0xc, value >> (repeat-1));
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x8, (1 << 8) | (a << 4) | b);
	} else
	{
		sampleShiftRegister();
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x8, (repeat << 8) | (a << 4) | b);
	}
	sampleShiftRegister();
#endif
}



static __inline__ void interface_jtag_add_dr_out_core(struct jtag_tap *target_tap,
		int num_fields,
		const int *num_bits,
		const uint32_t *value,
		enum tap_state end_state)
{
	enum tap_state pause_state = TAP_DRSHIFT;

	struct jtag_tap *tap, *nextTap;
	for (tap = jtag_tap_next_enabled(NULL); tap!= NULL; tap = nextTap)
	{
		nextTap = jtag_tap_next_enabled(tap);
		if (nextTap == NULL)
		{
			pause_state = end_state;
		}
		if (tap == target_tap)
		{
			int j;
			for (j = 0; j < (num_fields-1); j++)
			{
				shiftValueInner(TAP_DRSHIFT, TAP_DRSHIFT, num_bits[j], value[j]);
			}
			shiftValueInner(TAP_DRSHIFT, pause_state, num_bits[j], value[j]);
		} else
		{
			/* program the scan field to 1 bit length, and ignore it's value */
			shiftValueInner(TAP_DRSHIFT, pause_state, 1, 0);
		}
	}
}

static __inline__ void interface_jtag_add_dr_out(struct jtag_tap *target_tap,
		int num_fields,
		const int *num_bits,
		const uint32_t *value,
		enum tap_state end_state)
{

	int singletap = (jtag_tap_next_enabled(jtag_tap_next_enabled(NULL)) == NULL);
	if ((singletap) && (num_fields == 3))
	{
		/* used by embeddedice_write_reg_inner() */
		shiftValueInner(TAP_DRSHIFT, TAP_DRSHIFT, num_bits[0], value[0]);
		shiftValueInner(TAP_DRSHIFT, TAP_DRSHIFT, num_bits[1], value[1]);
		shiftValueInner(TAP_DRSHIFT, end_state, num_bits[2], value[2]);
	} else if ((singletap) && (num_fields == 2))
	{
		/* used by arm7 code */
		shiftValueInner(TAP_DRSHIFT, TAP_DRSHIFT, num_bits[0], value[0]);
		shiftValueInner(TAP_DRSHIFT, end_state, num_bits[1], value[1]);
	} else
	{
		interface_jtag_add_dr_out_core(target_tap, num_fields, num_bits, value, end_state);
	}
}

#define interface_jtag_add_callback(callback, in) callback(in)

#define interface_jtag_add_callback4(callback, in, data1, data2, data3) jtag_set_error(callback(in, data1, data2, data3))
