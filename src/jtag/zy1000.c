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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "embeddedice.h"
#include "minidriver.h"
#include "interface.h"

#include <cyg/hal/hal_io.h>             // low level i/o
#include <cyg/hal/hal_diag.h>


#define ZYLIN_VERSION "1.52"
#define ZYLIN_DATE __DATE__
#define ZYLIN_TIME __TIME__
#define ZYLIN_OPENOCD "$Revision$"
#define ZYLIN_OPENOCD_VERSION "Zylin JTAG ZY1000 " ZYLIN_VERSION " " ZYLIN_DATE " " ZYLIN_TIME
const char *zylin_config_dir="/config/settings";

/* low level command set
 */
int zy1000_read(void);
static void zy1000_write(int tck, int tms, int tdi);
void zy1000_reset(int trst, int srst);


int zy1000_speed(int speed);
int zy1000_register_commands(struct command_context_s *cmd_ctx);
int zy1000_init(void);
int zy1000_quit(void);

/* interface commands */
int zy1000_handle_zy1000_port_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

static int zy1000_khz(int khz, int *jtag_speed)
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

static int zy1000_speed_div(int speed, int *khz)
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

static bool readPowerDropout(void)
{
	cyg_uint32 state;
	// sample and clear power dropout
	HAL_WRITE_UINT32(ZY1000_JTAG_BASE+0x10, 0x80);
	HAL_READ_UINT32(ZY1000_JTAG_BASE+0x10, state);
	bool powerDropout;
	powerDropout = (state & 0x80) != 0;
	return powerDropout;
}


static bool readSRST(void)
{
	cyg_uint32 state;
	// sample and clear SRST sensing
	HAL_WRITE_UINT32(ZY1000_JTAG_BASE+0x10, 0x00000040);
	HAL_READ_UINT32(ZY1000_JTAG_BASE+0x10, state);
	bool srstAsserted;
	srstAsserted = (state & 0x40) != 0;
	return srstAsserted;
}

static int zy1000_srst_asserted(int *srst_asserted)
{
	*srst_asserted=readSRST();
	return ERROR_OK;
}

static int zy1000_power_dropout(int *dropout)
{
	*dropout=readPowerDropout();
	return ERROR_OK;
}


jtag_interface_t zy1000_interface =
{
	.name = "ZY1000",
	.execute_queue = NULL,
	.speed = zy1000_speed,
	.register_commands = zy1000_register_commands,
	.init = zy1000_init,
	.quit = zy1000_quit,
	.khz = zy1000_khz,
	.speed_div = zy1000_speed_div,
	.power_dropout = zy1000_power_dropout,
	.srst_asserted = zy1000_srst_asserted,
};

static void zy1000_write(int tck, int tms, int tdi)
{

}

int zy1000_read(void)
{
	return -1;
}

extern bool readSRST(void);

void zy1000_reset(int trst, int srst)
{
	LOG_DEBUG("zy1000 trst=%d, srst=%d", trst, srst);
	if(!srst)
	{
		ZY1000_POKE(ZY1000_JTAG_BASE+0x14, 0x00000001);
	}
	else
	{
		/* Danger!!! if clk!=0 when in
		 * idle in TAP_IDLE, reset halt on str912 will fail.
		 */
		ZY1000_POKE(ZY1000_JTAG_BASE+0x10, 0x00000001);
	}

	if(!trst)
	{
		ZY1000_POKE(ZY1000_JTAG_BASE+0x14, 0x00000002);
	}
	else
	{
		/* assert reset */
		ZY1000_POKE(ZY1000_JTAG_BASE+0x10, 0x00000002);
	}

	if (trst||(srst&&(jtag_reset_config & RESET_SRST_PULLS_TRST)))
	{
		waitIdle();
		/* we're now in the RESET state until trst is deasserted */
		ZY1000_POKE(ZY1000_JTAG_BASE+0x20, TAP_RESET);
	} else
	{
		/* We'll get RCLK failure when we assert TRST, so clear any false positives here */
		ZY1000_POKE(ZY1000_JTAG_BASE+0x14, 0x400);
	}

	/* wait for srst to float back up */
	if (!srst)
	{
		int i;
		for (i=0; i<1000; i++)
		{
			// We don't want to sense our own reset, so we clear here.
			// There is of course a timing hole where we could loose
			// a "real" reset.
			if (!readSRST())
				break;

			/* wait 1ms */
			alive_sleep(1);
		}

		if (i==1000)
		{
			LOG_USER("SRST didn't deassert after %dms", i);
		} else if (i>1)
		{
			LOG_USER("SRST took %dms to deassert", i);
		}
	}
}

int zy1000_speed(int speed)
{
	if(speed == 0)
	{
		/*0 means RCLK*/
		speed = 0;
		ZY1000_POKE(ZY1000_JTAG_BASE+0x10, 0x100);
		LOG_DEBUG("jtag_speed using RCLK");
	}
	else
	{
		if(speed > 8190 || speed < 2)
		{
			LOG_USER("valid ZY1000 jtag_speed=[8190,2]. Divisor is 64MHz / even values between 8190-2, i.e. min 7814Hz, max 32MHz");
			return ERROR_INVALID_ARGUMENTS;
		}

		LOG_USER("jtag_speed %d => JTAG clk=%f", speed, 64.0/(float)speed);
		ZY1000_POKE(ZY1000_JTAG_BASE+0x14, 0x100);
		ZY1000_POKE(ZY1000_JTAG_BASE+0x1c, speed&~1);
	}
	return ERROR_OK;
}

static bool savePower;


static void setPower(bool power)
{
	savePower = power;
	if (power)
	{
		HAL_WRITE_UINT32(ZY1000_JTAG_BASE+0x14, 0x8);
	} else
	{
		HAL_WRITE_UINT32(ZY1000_JTAG_BASE+0x10, 0x8);
	}
}

int handle_power_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc > 1)
	{
		return ERROR_INVALID_ARGUMENTS;
	}

	if (argc == 1)
	{
		if (strcmp(args[0], "on") == 0)
		{
			setPower(1);
		}
		else if (strcmp(args[0], "off") == 0)
		{
			setPower(0);
		} else
		{
			command_print(cmd_ctx, "arg is \"on\" or \"off\"");
			return ERROR_INVALID_ARGUMENTS;
		}
	}

	command_print(cmd_ctx, "Target power %s", savePower ? "on" : "off");

	return ERROR_OK;
}


/* Give TELNET a way to find out what version this is */
static int jim_zy1000_version(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	if ((argc < 1) || (argc > 2))
		return JIM_ERR;
	char buff[128];
	const char *version_str=NULL;

	if (argc == 1)
	{
		version_str=ZYLIN_OPENOCD_VERSION;
	} else
	{
		const char *str = Jim_GetString(argv[1], NULL);
		if (strcmp("openocd", str) == 0)
		{
			int revision;
			revision = atol(ZYLIN_OPENOCD+strlen("XRevision: "));
			sprintf(buff, "%d", revision);
			version_str=buff;
		}
		else if (strcmp("zy1000", str) == 0)
		{
			version_str=ZYLIN_VERSION;
		}
		else if (strcmp("date", str) == 0)
		{
			version_str=ZYLIN_DATE;
		}
		else
		{
			return JIM_ERR;
		}
	}

	Jim_SetResult(interp, Jim_NewStringObj(interp, version_str, -1));

	return JIM_OK;
}


static int
zylinjtag_Jim_Command_powerstatus(Jim_Interp *interp,
								   int argc,
		Jim_Obj * const *argv)
{
	if (argc != 1)
	{
		Jim_WrongNumArgs(interp, 1, argv, "powerstatus");
		return JIM_ERR;
	}

	cyg_uint32 status;
	ZY1000_PEEK(ZY1000_JTAG_BASE+0x10, status);

	Jim_SetResult(interp, Jim_NewIntObj(interp, (status&0x80)!=0));

	return JIM_OK;
}

int zy1000_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "power", handle_power_command, COMMAND_ANY,
			"power <on/off> - turn power switch to target on/off. No arguments - print status.");

	Jim_CreateCommand(interp, "zy1000_version", jim_zy1000_version, NULL, NULL);


	Jim_CreateCommand(interp, "powerstatus", zylinjtag_Jim_Command_powerstatus, NULL, NULL);

	return ERROR_OK;
}




int zy1000_init(void)
{
	LOG_USER("%s", ZYLIN_OPENOCD_VERSION);

	ZY1000_POKE(ZY1000_JTAG_BASE+0x10, 0x30); // Turn on LED1 & LED2

	setPower(true); // on by default


	 /* deassert resets. Important to avoid infinite loop waiting for SRST to deassert */
	zy1000_reset(0, 0);
	zy1000_speed(jtag_speed);

	return ERROR_OK;
}

int zy1000_quit(void)
{

	return ERROR_OK;
}



int interface_jtag_execute_queue(void)
{
	cyg_uint32 empty;

	waitIdle();
	ZY1000_PEEK(ZY1000_JTAG_BASE+0x10, empty);
	/* clear JTAG error register */
	ZY1000_POKE(ZY1000_JTAG_BASE+0x14, 0x400);

	if ((empty&0x400)!=0)
	{
		LOG_WARNING("RCLK timeout");
		/* the error is informative only as we don't want to break the firmware if there
		 * is a false positive.
		 */
//		return ERROR_FAIL;
	}
	return ERROR_OK;
}





static cyg_uint32 getShiftValue(void)
{
	cyg_uint32 value;
	waitIdle();
	ZY1000_PEEK(ZY1000_JTAG_BASE+0xc, value);
	VERBOSE(LOG_INFO("getShiftValue %08x", value));
	return value;
}
#if 0
static cyg_uint32 getShiftValueFlip(void)
{
	cyg_uint32 value;
	waitIdle();
	ZY1000_PEEK(ZY1000_JTAG_BASE+0x18, value);
	VERBOSE(LOG_INFO("getShiftValue %08x (flipped)", value));
	return value;
}
#endif

#if 0
static void shiftValueInnerFlip(const tap_state_t state, const tap_state_t endState, int repeat, cyg_uint32 value)
{
	VERBOSE(LOG_INFO("shiftValueInner %s %s %d %08x (flipped)", tap_state_name(state), tap_state_name(endState), repeat, value));
	cyg_uint32 a,b;
	a=state;
	b=endState;
	ZY1000_POKE(ZY1000_JTAG_BASE+0xc, value);
	ZY1000_POKE(ZY1000_JTAG_BASE+0x8, (1<<15)|(repeat<<8)|(a<<4)|b);
	VERBOSE(getShiftValueFlip());
}
#endif

extern int jtag_check_value(u8 *captured, void *priv);

static __inline void scanFields(int num_fields, scan_field_t *fields, tap_state_t shiftState, tap_state_t end_state)
{
	int i;
	int j;
	int k;

	for (i = 0; i < num_fields; i++)
	{
		cyg_uint32 value;

		static u8 *in_buff=NULL; /* pointer to buffer for scanned data */
		static int in_buff_size=0;
		u8 *inBuffer=NULL;


		// figure out where to store the input data
		int num_bits=fields[i].num_bits;
		if (fields[i].in_value!=NULL)
		{
			inBuffer=fields[i].in_value;
		}

		// here we shuffle N bits out/in
		j=0;
		while (j<num_bits)
		{
			tap_state_t pause_state;
			int l;
			k=num_bits-j;
			pause_state=(shiftState==TAP_DRSHIFT)?TAP_DRSHIFT:TAP_IRSHIFT;
			if (k>32)
			{
				k=32;
				/* we have more to shift out */
			} else if (i == num_fields-1)
			{
				/* this was the last to shift out this time */
				pause_state=end_state;
			}

			// we have (num_bits+7)/8 bytes of bits to toggle out.
			// bits are pushed out LSB to MSB
			value=0;
			if (fields[i].out_value!=NULL)
			{
				for (l=0; l<k; l+=8)
				{
					value|=fields[i].out_value[(j+l)/8]<<l;
				}
			}
			/* mask away unused bits for easier debugging */
			value&=~(((u32)0xffffffff)<<k);

			shiftValueInner(shiftState, pause_state, k, value);

			if (inBuffer!=NULL)
			{
				// data in, LSB to MSB
				value=getShiftValue();
				// we're shifting in data to MSB, shift data to be aligned for returning the value
				value >>= 32-k;

				for (l=0; l<k; l+=8)
				{
					inBuffer[(j+l)/8]=(value>>l)&0xff;
				}
			}
			j+=k;
		}
	}
}

int interface_jtag_add_end_state(tap_state_t state)
{
	return ERROR_OK;
}


int interface_jtag_add_ir_scan(int num_fields, const scan_field_t *fields, tap_state_t state)
{

	int j;
	int scan_size = 0;
	jtag_tap_t *tap, *nextTap;
	for(tap = jtag_NextEnabledTap(NULL); tap!= NULL; tap=nextTap)
	{
		nextTap=jtag_NextEnabledTap(tap);
		tap_state_t end_state;
		if (nextTap==NULL)
		{
			end_state = state;
		} else
		{
			end_state = TAP_IRSHIFT;
		}

		int found = 0;

		scan_size = tap->ir_length;

		/* search the list */
		for (j=0; j < num_fields; j++)
		{
			if (tap == fields[j].tap)
			{
				found = 1;

				scanFields(1, fields+j, TAP_IRSHIFT, end_state);
				/* update device information */
				buf_cpy(fields[j].out_value, tap->cur_instr, scan_size);

				tap->bypass = 0;
				break;
			}
		}

		if (!found)
		{
			/* if a device isn't listed, set it to BYPASS */
			u8 ones[]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

			scan_field_t tmp;
			memset(&tmp, 0, sizeof(tmp));
			tmp.out_value = ones;
			tmp.num_bits = scan_size;
			scanFields(1, &tmp, TAP_IRSHIFT, end_state);
			/* update device information */
			buf_cpy(tmp.out_value, tap->cur_instr, scan_size);
			tap->bypass = 1;
		}
	}

	return ERROR_OK;
}





int interface_jtag_add_plain_ir_scan(int num_fields, const scan_field_t *fields, tap_state_t state)
{
	scanFields(num_fields, fields, TAP_IRSHIFT, state);

	return ERROR_OK;
}

/*extern jtag_command_t **jtag_get_last_command_p(void);*/

int interface_jtag_add_dr_scan(int num_fields, const scan_field_t *fields, tap_state_t state)
{

	int j;
	jtag_tap_t *tap, *nextTap;
	for(tap = jtag_NextEnabledTap(NULL); tap!= NULL; tap=nextTap)
	{
		nextTap=jtag_NextEnabledTap(tap);
		int found=0;
		tap_state_t end_state;
		if (nextTap==NULL)
		{
			end_state = state;
		} else
		{
			end_state = TAP_DRSHIFT;
		}

		for (j=0; j < num_fields; j++)
		{
			if (tap == fields[j].tap)
			{
				found = 1;

				scanFields(1, fields+j, TAP_DRSHIFT, end_state);
			}
		}
		if (!found)
		{
			scan_field_t tmp;
			/* program the scan field to 1 bit length, and ignore it's value */
			tmp.num_bits = 1;
			tmp.out_value = NULL;
			tmp.in_value = NULL;

			scanFields(1, &tmp, TAP_DRSHIFT, end_state);
		}
		else
		{
		}
	}
	return ERROR_OK;
}

int interface_jtag_add_plain_dr_scan(int num_fields, const scan_field_t *fields, tap_state_t state)
{
	scanFields(num_fields, fields, TAP_DRSHIFT, state);
	return ERROR_OK;
}


int interface_jtag_add_tlr()
{
	setCurrentState(TAP_RESET);
	return ERROR_OK;
}




extern int jtag_nsrst_delay;
extern int jtag_ntrst_delay;

int interface_jtag_add_reset(int req_trst, int req_srst)
{
	zy1000_reset(req_trst, req_srst);
	return ERROR_OK;
}

static int zy1000_jtag_add_clocks(int num_cycles, tap_state_t state, tap_state_t clockstate)
{
	/* num_cycles can be 0 */
	setCurrentState(clockstate);

	/* execute num_cycles, 32 at the time. */
	int i;
	for (i=0; i<num_cycles; i+=32)
	{
		int num;
		num=32;
		if (num_cycles-i<num)
		{
			num=num_cycles-i;
		}
		shiftValueInner(clockstate, clockstate, num, 0);
	}

#if !TEST_MANUAL()
	/* finish in end_state */
	setCurrentState(state);
#else
	tap_state_t t=TAP_IDLE;
	/* test manual drive code on any target */
	int tms;
	u8 tms_scan = tap_get_tms_path(t, state);
	int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	for (i = 0; i < tms_count; i++)
	{
		tms = (tms_scan >> i) & 1;
		waitIdle();
		ZY1000_POKE(ZY1000_JTAG_BASE+0x28,  tms);
	}
	waitIdle();
	ZY1000_POKE(ZY1000_JTAG_BASE+0x20, state);
#endif


	return ERROR_OK;
}

int interface_jtag_add_runtest(int num_cycles, tap_state_t state)
{
	return zy1000_jtag_add_clocks(num_cycles, state, TAP_IDLE);
}

int interface_jtag_add_clocks(int num_cycles)
{
	return zy1000_jtag_add_clocks(num_cycles, cmd_queue_cur_state, cmd_queue_cur_state);
}

int interface_jtag_add_sleep(u32 us)
{
	jtag_sleep(us);
	return ERROR_OK;
}

int interface_jtag_add_pathmove(int num_states, const tap_state_t *path)
{
	int state_count;
	int tms = 0;

	/*wait for the fifo to be empty*/
	waitIdle();

	state_count = 0;

	tap_state_t cur_state=cmd_queue_cur_state;

	while (num_states)
	{
		if (tap_state_transition(cur_state, false) == path[state_count])
		{
			tms = 0;
		}
		else if (tap_state_transition(cur_state, true) == path[state_count])
		{
			tms = 1;
		}
		else
		{
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition", tap_state_name(cur_state), tap_state_name(path[state_count]));
			exit(-1);
		}

		waitIdle();
		ZY1000_POKE(ZY1000_JTAG_BASE+0x28,  tms);

		cur_state = path[state_count];
		state_count++;
		num_states--;
	}

	waitIdle();
	ZY1000_POKE(ZY1000_JTAG_BASE+0x20,  cur_state);
	return ERROR_OK;
}



void embeddedice_write_dcc(jtag_tap_t *tap, int reg_addr, u8 *buffer, int little, int count)
{
//	static int const reg_addr=0x5;
	tap_state_t end_state=jtag_get_end_state();
	if (jtag_NextEnabledTap(jtag_NextEnabledTap(NULL))==NULL)
	{
		/* better performance via code duplication */
		if (little)
		{
			int i;
			for (i = 0; i < count; i++)
			{
				shiftValueInner(TAP_DRSHIFT, TAP_DRSHIFT, 32, fast_target_buffer_get_u32(buffer, 1));
				shiftValueInner(TAP_DRSHIFT, end_state, 6, reg_addr|(1<<5));
				buffer+=4;
			}
		} else
		{
			int i;
			for (i = 0; i < count; i++)
			{
				shiftValueInner(TAP_DRSHIFT, TAP_DRSHIFT, 32, fast_target_buffer_get_u32(buffer, 0));
				shiftValueInner(TAP_DRSHIFT, end_state, 6, reg_addr|(1<<5));
				buffer+=4;
			}
		}
	}
	else
	{
		int i;
		for (i = 0; i < count; i++)
		{
			embeddedice_write_reg_inner(tap, reg_addr, fast_target_buffer_get_u32(buffer, little));
			buffer += 4;
		}
	}
}

int loadFile(const char *fileName, void **data, int *len);

/* boolean parameter stored on config */
int boolParam(char *var)
{
	bool result = false;
	char *name = alloc_printf("%s/%s", zylin_config_dir, var);
	if (name == NULL)
		return result;

	void *data;
	int len;
	if (loadFile(name, &data, &len) == ERROR_OK)
	{
		if (len > 1)
			len = 1;
		result = strncmp((char *) data, "1", len) == 0;
		free(data);
	}
	free(name);
	return result;
}


