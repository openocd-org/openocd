/***************************************************************************
 *   Copyright (C) 2007-2010 by Øyvind Harboe                              *
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

/* used to test manual mode */
#define TEST_MANUAL() 0
#define VERBOSE(a)

#if BUILD_ZY1000_MASTER

#define ZY1000_PEEK(a, b) do {b = *((volatile uint32_t *)(a)); } while (0)
#define ZY1000_POKE(a, b) do {*((volatile uint32_t *)(a)) = b; } while (0)
extern volatile void *zy1000_jtag_master;
#define ZY1000_JTAG_BASE ((unsigned long)zy1000_jtag_master)

#else

/* redirect this to TCP/IP */
#define ZY1000_JTAG_BASE 0
extern void zy1000_tcpout(uint32_t address, uint32_t data);
extern uint32_t zy1000_tcpin(uint32_t address);
#define ZY1000_PEEK(a, b) b = zy1000_tcpin(a)
#define ZY1000_POKE(a, b) zy1000_tcpout(a, b)

#endif

#if BUILD_ZY1000_MASTER
/* FIFO empty? */
static inline void waitIdle(void)
{
	uint32_t empty;
	do {
		ZY1000_PEEK(ZY1000_JTAG_BASE + 0x10, empty);
	} while ((empty & 0x100) == 0);
}

static inline void zy1000_flush_readqueue(void)
{
	/* Not used w/hardware fifo */
}
static inline void zy1000_flush_callbackqueue(void)
{
	/* Not used w/hardware fifo */
}
#else
extern void waitIdle(void);
void zy1000_flush_readqueue(void);
void zy1000_flush_callbackqueue(void);
void zy1000_jtag_add_callback4(jtag_callback_t callback,
		jtag_callback_data_t data0,
		jtag_callback_data_t data1,
		jtag_callback_data_t data2,
		jtag_callback_data_t data3);
void zy1000_jtag_add_callback(jtag_callback1_t callback, jtag_callback_data_t data0);
#endif

static inline void waitQueue(void)
{
/*	waitIdle(); */
}

static inline void sampleShiftRegister(void)
{
#if 0
	uint32_t dummy;
	waitIdle();
	ZY1000_PEEK(ZY1000_JTAG_BASE + 0xc, dummy);
#endif
}

static inline void setCurrentState(enum tap_state state)
{
	uint32_t a;
	a = state;
	int repeat = 0;
	if (state == TAP_RESET) {
		/* The FPGA nor we know the current state of the CPU TAP */
		/* controller. This will move it to TAP for sure. */
		/*  */
		/* 5 should be enough here, 7 is what OpenOCD uses */
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
static inline void shiftValueInner(const enum tap_state state,
	const enum tap_state endState,
	int repeat,
	uint32_t value)
{
	uint32_t a, b;
	a = state;
	b = endState;
	waitQueue();
	sampleShiftRegister();
	ZY1000_POKE(ZY1000_JTAG_BASE + 0xc, value);
#if 1
#if TEST_MANUAL()
	if ((state == TAP_DRSHIFT) && (endState != TAP_DRSHIFT)) {
		int i;
		setCurrentState(state);
		for (i = 0; i < repeat; i++) {
			int tms;
			tms = 0;
			if ((i == repeat-1) && (state != endState))
				tms = 1;
					/* shift out value */
			waitIdle();
			ZY1000_POKE(ZY1000_JTAG_BASE + 0x28, (((value >> i)&1) << 1) | tms);
		}
		waitIdle();
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x28, 0);
		waitIdle();
		/* ZY1000_POKE(ZY1000_JTAG_BASE + 0x20, TAP_DRSHIFT); // set this state and things
		 * break => expected */
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x20, TAP_DRPAUSE);	/* set this and things will
									 * work => expected. Not
									 * setting this is not
									 * sufficient to make things
									 * break. */
		setCurrentState(endState);
	} else
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x8, (repeat << 8) | (a << 4) | b);

#else
	/* fast version */
	ZY1000_POKE(ZY1000_JTAG_BASE + 0x8, (repeat << 8) | (a << 4) | b);
#endif
#else
	/* maximum debug version */
	if ((repeat > 0) && ((state == TAP_DRSHIFT) || (state == TAP_SI))) {
		int i;
		/* sample shift register for every bit. */
		for (i = 0; i < repeat-1; i++) {
			sampleShiftRegister();
			ZY1000_POKE(ZY1000_JTAG_BASE + 0xc, value >> i);
			ZY1000_POKE(ZY1000_JTAG_BASE + 0x8, (1 << 8) | (a << 4) | a);
		}
		sampleShiftRegister();
		ZY1000_POKE(ZY1000_JTAG_BASE + 0xc, value >> (repeat-1));
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x8, (1 << 8) | (a << 4) | b);
	} else {
		sampleShiftRegister();
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x8, (repeat << 8) | (a << 4) | b);
	}
	sampleShiftRegister();
#endif
}

#if BUILD_ZY1000_MASTER
#define interface_jtag_add_callback(callback, in) callback(in)
#define interface_jtag_add_callback4(callback, in, data1, data2, \
		data3) jtag_set_error(callback(in, data1, data2, data3))
#else
#define interface_jtag_add_callback(callback, in) zy1000_jtag_add_callback(callback, in)
#define interface_jtag_add_callback4(callback, in, data1, data2, data3) zy1000_jtag_add_callback4( \
	callback, \
	in, \
	data1, \
	data2, \
	data3)
#endif
