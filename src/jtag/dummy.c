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

static tap_state_t tap_state_transition(tap_state_t cur_state, int tms);


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
	return 1;
}


void dummy_write(int tck, int tms, int tdi)
{
	/* TAP standard: "state transitions occur on rising edge of clock" */
	if( tck != dummy_clock )
	{
		if( tck )
		{
			int old_state = dummy_state;
			dummy_state = tap_state_transition( dummy_state, tms );
			if( old_state != dummy_state )
				LOG_INFO( "dummy_tap=%s", jtag_state_name(dummy_state) );
		}
		dummy_clock = tck;
	}
}

void dummy_reset(int trst, int srst)
{
	dummy_clock = 0;
	dummy_state = TAP_RESET;
	LOG_DEBUG( "reset to %s", jtag_state_name(dummy_state) );
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


/**
 * Function tap_state_transition
 * takes a current TAP state and returns the next state according to the tms value.
 *
 * Even though there is code to duplicate this elsewhere, we do it here a little
 * differently just to get a second opinion, i.e. a verification, on state tracking
 * in that other logic. Plus array lookups without index checking are no favorite thing.
 * This is educational for developers new to TAP controllers.
 */
static tap_state_t tap_state_transition(tap_state_t cur_state, int tms)
{
	tap_state_t new_state;

	if (tms)
	{
		switch (cur_state)
		{
		case TAP_RESET:
			new_state = cur_state;
			break;
		case TAP_IDLE:
		case TAP_DRUPDATE:
		case TAP_IRUPDATE:
			new_state = TAP_DRSELECT;
			break;
		case TAP_DRSELECT:
			new_state = TAP_IRSELECT;
			break;
		case TAP_DRCAPTURE:
		case TAP_DRSHIFT:
			new_state = TAP_DREXIT1;
			break;
		case TAP_DREXIT1:
		case TAP_DREXIT2:
			new_state = TAP_DRUPDATE;
			break;
		case TAP_DRPAUSE:
			new_state = TAP_DREXIT2;
			break;
		case TAP_IRSELECT:
			new_state = TAP_RESET;
			break;
		case TAP_IRCAPTURE:
		case TAP_IRSHIFT:
			new_state = TAP_IREXIT1;
			break;
		case TAP_IREXIT1:
		case TAP_IREXIT2:
			new_state = TAP_IRUPDATE;
			break;
		case TAP_IRPAUSE:
			new_state = TAP_IREXIT2;
			break;
		default:
			LOG_ERROR( "fatal: invalid argument cur_state=%d", cur_state );
			exit(1);
			break;
		}
	}
	else
	{
		switch (cur_state)
		{
		case TAP_RESET:
		case TAP_IDLE:
		case TAP_DRUPDATE:
		case TAP_IRUPDATE:
			new_state = TAP_IDLE;
			break;
		case TAP_DRSELECT:
			new_state = TAP_DRCAPTURE;
			break;
		case TAP_DRCAPTURE:
		case TAP_DRSHIFT:
		case TAP_DREXIT2:
			new_state = TAP_DRSHIFT;
			break;
		case TAP_DREXIT1:
		case TAP_DRPAUSE:
			new_state = TAP_DRPAUSE;
			break;
		case TAP_IRSELECT:
			new_state = TAP_IRCAPTURE;
			break;
		case TAP_IRCAPTURE:
		case TAP_IRSHIFT:
		case TAP_IREXIT2:
			new_state = TAP_IRSHIFT;
			break;
		case TAP_IREXIT1:
		case TAP_IRPAUSE:
			new_state = TAP_IRPAUSE;
			break;
		default:
			LOG_ERROR( "fatal: invalid argument cur_state=%d", cur_state );
			exit(1);
			break;
		}
	}

	return new_state;
}
