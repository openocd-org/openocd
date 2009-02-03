/***************************************************************************
 *   Copyright (C) 2008 by Øyvind Harboe                                   *
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

#include "replacements.h"

#include "jtag.h"
#include "bitbang.h"


/* my private tap controller state, which tracks state for calling code */
static tap_state_t dummy_state = TAP_RESET;

static int dummy_clock;         /* edge detector */

static int clock_count;         /* count clocks in any stable state, only stable states */

static u32 dummy_data;


int dummy_speed(int speed);
int dummy_register_commands(struct command_context_s *cmd_ctx);
int dummy_init(void);
int dummy_quit(void);
static int dummy_khz(int khz, int *jtag_speed);
static int dummy_speed_div(int speed, int *khz);


/* The dummy driver is used to easily check the code path
 * where the target is unresponsive.
 */
jtag_interface_t dummy_interface =
{
	.name = "dummy",

	.execute_queue = bitbang_execute_queue,

	.speed = dummy_speed,
	.register_commands = dummy_register_commands,
	.khz = dummy_khz,
	.speed_div = dummy_speed_div,

	.init = dummy_init,
	.quit = dummy_quit,
};

int dummy_read(void);
void dummy_write(int tck, int tms, int tdi);
void dummy_reset(int trst, int srst);
void dummy_led(int on);

bitbang_interface_t dummy_bitbang =
{
	.read = dummy_read,
	.write = dummy_write,
	.reset = dummy_reset,
	.blink = dummy_led
};

int dummy_read(void)
{
	int data = 1 & dummy_data;
	dummy_data = (dummy_data >> 1) | (1<<31);
	return data;
}


void dummy_write(int tck, int tms, int tdi)
{
	/* TAP standard: "state transitions occur on rising edge of clock" */
	if( tck != dummy_clock )
	{
		if( tck )
		{
			int old_state = dummy_state;
			dummy_state = tap_state_transition( old_state, tms );

			if( old_state != dummy_state )
			{
				if( clock_count )
				{
					LOG_DEBUG("dummy_tap: %d stable clocks", clock_count);
					clock_count = 0;
				}

				LOG_DEBUG("dummy_tap: %s", tap_state_name(dummy_state) );

#if defined(DEBUG)
				if(dummy_state == TAP_DRCAPTURE)
					dummy_data = 0x01255043;
#endif
			}
			else
			{
				/* this is a stable state clock edge, no change of state here,
				 * simply increment clock_count for subsequent logging
				 */
				++clock_count;
			}
		}
		dummy_clock = tck;
	}
}

void dummy_reset(int trst, int srst)
{
	dummy_clock = 0;

	if (trst || (srst && (jtag_reset_config & RESET_SRST_PULLS_TRST)))
		dummy_state = TAP_RESET;

	LOG_DEBUG("reset to: %s", tap_state_name(dummy_state) );
}

static int dummy_khz(int khz, int *jtag_speed)
{
	if (khz==0)
	{
		*jtag_speed=0;
	}
	else
	{
		*jtag_speed=64000/khz;
	}
	return ERROR_OK;
}

static int dummy_speed_div(int speed, int *khz)
{
	if (speed==0)
	{
		*khz = 0;
	}
	else
	{
		*khz=64000/speed;
	}

	return ERROR_OK;
}

int dummy_speed(int speed)
{
	return ERROR_OK;
}

int dummy_register_commands(struct command_context_s *cmd_ctx)
{
	return ERROR_OK;
}

int dummy_init(void)
{
	bitbang_interface = &dummy_bitbang;

	return ERROR_OK;
}

int dummy_quit(void)
{
	return ERROR_OK;
}

void dummy_led(int on)
{
}

