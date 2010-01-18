/***************************************************************************
 *   Copyright (C) 2007-2009 by Øyvind Harboe                              *
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

/* This file supports the zy1000 debugger: http://www.zylin.com/zy1000.html
 *
 * The zy1000 is a standalone debugger that has a web interface and
 * requires no drivers on the developer host as all communication
 * is via TCP/IP. The zy1000 gets it performance(~400-700kBytes/s
 * DCC downloads @ 16MHz target) as it has an FPGA to hardware
 * accelerate the JTAG commands, while offering *very* low latency
 * between OpenOCD and the FPGA registers.
 *
 * The disadvantage of the zy1000 is that it has a feeble CPU compared to
 * a PC(ca. 50-500 DMIPS depending on how one counts it), whereas a PC
 * is on the order of 10000 DMIPS(i.e. at a factor of 20-200).
 *
 * The zy1000 revc hardware is using an Altera Nios CPU, whereas the
 * revb is using ARM7 + Xilinx.
 *
 * See Zylin web pages or contact Zylin for more information.
 *
 * The reason this code is in OpenOCD rather than OpenOCD linked with the
 * ZY1000 code is that OpenOCD is the long road towards getting
 * libopenocd into place. libopenocd will support both low performance,
 * low latency systems(embedded) and high performance high latency
 * systems(PCs).
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/embeddedice.h>
#include <jtag/minidriver.h>
#include <jtag/interface.h>
#include "zy1000_version.h"

#include <cyg/hal/hal_io.h>             // low level i/o
#include <cyg/hal/hal_diag.h>

#include <time.h>

#ifdef CYGPKG_HAL_NIOS2
#include <cyg/hal/io.h>
#include <cyg/firmwareutil/firmwareutil.h>
#endif

#define ZYLIN_VERSION GIT_ZY1000_VERSION
#define ZYLIN_DATE __DATE__
#define ZYLIN_TIME __TIME__
#define ZYLIN_OPENOCD GIT_OPENOCD_VERSION
#define ZYLIN_OPENOCD_VERSION "ZY1000 " ZYLIN_VERSION " " ZYLIN_DATE


static int zy1000_khz(int khz, int *jtag_speed)
{
	if (khz == 0)
	{
		*jtag_speed = 0;
	}
	else
	{
		*jtag_speed = 64000/khz;
	}
	return ERROR_OK;
}

static int zy1000_speed_div(int speed, int *khz)
{
	if (speed == 0)
	{
		*khz = 0;
	}
	else
	{
		*khz = 64000/speed;
	}

	return ERROR_OK;
}

static bool readPowerDropout(void)
{
	cyg_uint32 state;
	// sample and clear power dropout
	ZY1000_POKE(ZY1000_JTAG_BASE + 0x10, 0x80);
	ZY1000_PEEK(ZY1000_JTAG_BASE + 0x10, state);
	bool powerDropout;
	powerDropout = (state & 0x80) != 0;
	return powerDropout;
}


static bool readSRST(void)
{
	cyg_uint32 state;
	// sample and clear SRST sensing
	ZY1000_POKE(ZY1000_JTAG_BASE + 0x10, 0x00000040);
	ZY1000_PEEK(ZY1000_JTAG_BASE + 0x10, state);
	bool srstAsserted;
	srstAsserted = (state & 0x40) != 0;
	return srstAsserted;
}

static int zy1000_srst_asserted(int *srst_asserted)
{
	*srst_asserted = readSRST();
	return ERROR_OK;
}

static int zy1000_power_dropout(int *dropout)
{
	*dropout = readPowerDropout();
	return ERROR_OK;
}

void zy1000_reset(int trst, int srst)
{
	LOG_DEBUG("zy1000 trst=%d, srst=%d", trst, srst);

	/* flush the JTAG FIFO. Not flushing the queue before messing with
	 * reset has such interesting bugs as causing hard to reproduce
	 * RCLK bugs as RCLK will stop responding when TRST is asserted
	 */
	waitIdle();

	if (!srst)
	{
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x14, 0x00000001);
	}
	else
	{
		/* Danger!!! if clk != 0 when in
		 * idle in TAP_IDLE, reset halt on str912 will fail.
		 */
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x10, 0x00000001);
	}

	if (!trst)
	{
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x14, 0x00000002);
	}
	else
	{
		/* assert reset */
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x10, 0x00000002);
	}

	if (trst||(srst && (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
	{
		/* we're now in the RESET state until trst is deasserted */
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x20, TAP_RESET);
	} else
	{
		/* We'll get RCLK failure when we assert TRST, so clear any false positives here */
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x14, 0x400);
	}

	/* wait for srst to float back up */
	if (!srst)
	{
		int i;
		for (i = 0; i < 1000; i++)
		{
			// We don't want to sense our own reset, so we clear here.
			// There is of course a timing hole where we could loose
			// a "real" reset.
			if (!readSRST())
				break;

			/* wait 1ms */
			alive_sleep(1);
		}

		if (i == 1000)
		{
			LOG_USER("SRST didn't deassert after %dms", i);
		} else if (i > 1)
		{
			LOG_USER("SRST took %dms to deassert", i);
		}
	}
}

int zy1000_speed(int speed)
{
	/* flush JTAG master FIFO before setting speed */
	waitIdle();

	if (speed == 0)
	{
		/*0 means RCLK*/
		speed = 0;
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x10, 0x100);
		LOG_DEBUG("jtag_speed using RCLK");
	}
	else
	{
		if (speed > 8190 || speed < 2)
		{
			LOG_USER("valid ZY1000 jtag_speed=[8190,2]. Divisor is 64MHz / even values between 8190-2, i.e. min 7814Hz, max 32MHz");
			return ERROR_INVALID_ARGUMENTS;
		}

		LOG_USER("jtag_speed %d => JTAG clk=%f", speed, 64.0/(float)speed);
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x14, 0x100);
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x1c, speed&~1);
	}
	return ERROR_OK;
}

static bool savePower;


static void setPower(bool power)
{
	savePower = power;
	if (power)
	{
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x14, 0x8);
	} else
	{
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x10, 0x8);
	}
}

COMMAND_HANDLER(handle_power_command)
{
	switch (CMD_ARGC)
	{
	case 1: {
		bool enable;
		COMMAND_PARSE_ON_OFF(CMD_ARGV[0], enable);
		setPower(enable);
		// fall through
	}
	case 0:
		LOG_INFO("Target power %s", savePower ? "on" : "off");
		break;
	default:
		return ERROR_INVALID_ARGUMENTS;
	}

	return ERROR_OK;
}


/* Give TELNET a way to find out what version this is */
static int jim_zy1000_version(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	if ((argc < 1) || (argc > 3))
		return JIM_ERR;
	const char *version_str = NULL;

	if (argc == 1)
	{
		version_str = ZYLIN_OPENOCD_VERSION;
	} else
	{
		const char *str = Jim_GetString(argv[1], NULL);
		const char *str2 = NULL;
		if (argc > 2)
			str2 = Jim_GetString(argv[2], NULL);
		if (strcmp("openocd", str) == 0)
		{
			version_str = ZYLIN_OPENOCD;
		}
		else if (strcmp("zy1000", str) == 0)
		{
			version_str = ZYLIN_VERSION;
		}
		else if (strcmp("date", str) == 0)
		{
			version_str = ZYLIN_DATE;
		}
		else if (strcmp("time", str) == 0)
		{
			version_str = ZYLIN_TIME;
		}
		else if (strcmp("pcb", str) == 0)
		{
#ifdef CYGPKG_HAL_NIOS2
			version_str="c";
#else
			version_str="b";
#endif
		}
#ifdef CYGPKG_HAL_NIOS2
		else if (strcmp("fpga", str) == 0)
		{

			/* return a list of 32 bit integers to describe the expected
			 * and actual FPGA
			 */
			static char *fpga_id = "0x12345678 0x12345678 0x12345678 0x12345678";
			cyg_uint32 id, timestamp;
			HAL_READ_UINT32(SYSID_BASE, id);
			HAL_READ_UINT32(SYSID_BASE+4, timestamp);
			sprintf(fpga_id, "0x%08x 0x%08x 0x%08x 0x%08x", id, timestamp, SYSID_ID, SYSID_TIMESTAMP);
			version_str = fpga_id;
			if ((argc>2) && (strcmp("time", str2) == 0))
			{
			    time_t last_mod = timestamp;
			    char * t = ctime (&last_mod) ;
			    t[strlen(t)-1] = 0;
			    version_str = t;
			}
		}
#endif

		else
		{
			return JIM_ERR;
		}
	}

	Jim_SetResult(interp, Jim_NewStringObj(interp, version_str, -1));

	return JIM_OK;
}


#ifdef CYGPKG_HAL_NIOS2


struct info_forward
{
	void *data;
	struct cyg_upgrade_info *upgraded_file;
};

static void report_info(void *data, const char * format, va_list args)
{
	char *s = alloc_vprintf(format, args);
	LOG_USER_N("%s", s);
	free(s);
}

struct cyg_upgrade_info firmware_info =
{
		(cyg_uint8 *)0x84000000,
		"/ram/firmware.phi",
		"Firmware",
		0x0300000,
		0x1f00000 -
		0x0300000,
		"ZylinNiosFirmware\n",
		report_info,
};

static int jim_zy1000_writefirmware(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	if (argc != 2)
		return JIM_ERR;

	int length;
	const char *str = Jim_GetString(argv[1], &length);

	/* */
	int tmpFile;
	if ((tmpFile = open(firmware_info.file, O_RDWR | O_CREAT | O_TRUNC)) <= 0)
	{
		return JIM_ERR;
	}
	bool success;
	success = write(tmpFile, str, length) == length;
	close(tmpFile);
	if (!success)
		return JIM_ERR;

	if (!cyg_firmware_upgrade(NULL, firmware_info))
		return JIM_ERR;

	return JIM_OK;
}
#endif

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
	ZY1000_PEEK(ZY1000_JTAG_BASE + 0x10, status);

	Jim_SetResult(interp, Jim_NewIntObj(interp, (status&0x80) != 0));

	return JIM_OK;
}




int zy1000_init(void)
{
	LOG_USER("%s", ZYLIN_OPENOCD_VERSION);

	ZY1000_POKE(ZY1000_JTAG_BASE + 0x10, 0x30); // Turn on LED1 & LED2

	setPower(true); // on by default


	 /* deassert resets. Important to avoid infinite loop waiting for SRST to deassert */
	zy1000_reset(0, 0);
	zy1000_speed(jtag_get_speed());

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
	ZY1000_PEEK(ZY1000_JTAG_BASE + 0x10, empty);
	/* clear JTAG error register */
	ZY1000_POKE(ZY1000_JTAG_BASE + 0x14, 0x400);

	if ((empty&0x400) != 0)
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
	ZY1000_PEEK(ZY1000_JTAG_BASE + 0xc, value);
	VERBOSE(LOG_INFO("getShiftValue %08x", value));
	return value;
}
#if 0
static cyg_uint32 getShiftValueFlip(void)
{
	cyg_uint32 value;
	waitIdle();
	ZY1000_PEEK(ZY1000_JTAG_BASE + 0x18, value);
	VERBOSE(LOG_INFO("getShiftValue %08x (flipped)", value));
	return value;
}
#endif

#if 0
static void shiftValueInnerFlip(const tap_state_t state, const tap_state_t endState, int repeat, cyg_uint32 value)
{
	VERBOSE(LOG_INFO("shiftValueInner %s %s %d %08x (flipped)", tap_state_name(state), tap_state_name(endState), repeat, value));
	cyg_uint32 a,b;
	a = state;
	b = endState;
	ZY1000_POKE(ZY1000_JTAG_BASE + 0xc, value);
	ZY1000_POKE(ZY1000_JTAG_BASE + 0x8, (1 << 15) | (repeat << 8) | (a << 4) | b);
	VERBOSE(getShiftValueFlip());
}
#endif

static void gotoEndState(tap_state_t end_state)
{
	setCurrentState(end_state);
}

static __inline void scanFields(int num_fields, const struct scan_field *fields, tap_state_t shiftState, int pause)
{
	int i;
	int j;
	int k;

	for (i = 0; i < num_fields; i++)
	{
		cyg_uint32 value;

		uint8_t *inBuffer = NULL;


		// figure out where to store the input data
		int num_bits = fields[i].num_bits;
		if (fields[i].in_value != NULL)
		{
			inBuffer = fields[i].in_value;
		}

		// here we shuffle N bits out/in
		j = 0;
		while (j < num_bits)
		{
			tap_state_t pause_state;
			int l;
			k = num_bits-j;
			pause_state = (shiftState == TAP_DRSHIFT)?TAP_DRSHIFT:TAP_IRSHIFT;
			if (k > 32)
			{
				k = 32;
				/* we have more to shift out */
			} else if (pause&&(i == num_fields-1))
			{
				/* this was the last to shift out this time */
				pause_state = (shiftState==TAP_DRSHIFT)?TAP_DRPAUSE:TAP_IRPAUSE;
			}

			// we have (num_bits + 7)/8 bytes of bits to toggle out.
			// bits are pushed out LSB to MSB
			value = 0;
			if (fields[i].out_value != NULL)
			{
				for (l = 0; l < k; l += 8)
				{
					value|=fields[i].out_value[(j + l)/8]<<l;
				}
			}
			/* mask away unused bits for easier debugging */
			if (k < 32)
			{
				value&=~(((uint32_t)0xffffffff) << k);
			} else
			{
				/* Shifting by >= 32 is not defined by the C standard
				 * and will in fact shift by &0x1f bits on nios */
			}

			shiftValueInner(shiftState, pause_state, k, value);

			if (inBuffer != NULL)
			{
				// data in, LSB to MSB
				value = getShiftValue();
				// we're shifting in data to MSB, shift data to be aligned for returning the value
				value >>= 32-k;

				for (l = 0; l < k; l += 8)
				{
					inBuffer[(j + l)/8]=(value >> l)&0xff;
				}
			}
			j += k;
		}
	}
}

int interface_jtag_add_ir_scan(int num_fields, const struct scan_field *fields, tap_state_t state)
{

	int j;
	int scan_size = 0;
	struct jtag_tap *tap, *nextTap;
	for (tap = jtag_tap_next_enabled(NULL); tap!= NULL; tap = nextTap)
	{
		nextTap = jtag_tap_next_enabled(tap);
		int pause = (nextTap==NULL);

		int found = 0;

		scan_size = tap->ir_length;

		/* search the list */
		for (j = 0; j < num_fields; j++)
		{
			if (tap == fields[j].tap)
			{
				found = 1;

				scanFields(1, fields + j, TAP_IRSHIFT, pause);
				/* update device information */
				buf_cpy(fields[j].out_value, tap->cur_instr, scan_size);

				tap->bypass = 0;
				break;
			}
		}

		if (!found)
		{
			/* if a device isn't listed, set it to BYPASS */
			uint8_t ones[]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

			struct scan_field tmp;
			memset(&tmp, 0, sizeof(tmp));
			tmp.out_value = ones;
			tmp.num_bits = scan_size;
			scanFields(1, &tmp, TAP_IRSHIFT, pause);
			/* update device information */
			buf_cpy(tmp.out_value, tap->cur_instr, scan_size);
			tap->bypass = 1;
		}
	}
	gotoEndState(state);

	return ERROR_OK;
}





int interface_jtag_add_plain_ir_scan(int num_fields, const struct scan_field *fields, tap_state_t state)
{
	scanFields(num_fields, fields, TAP_IRSHIFT, 1);
	gotoEndState(state);

	return ERROR_OK;
}

int interface_jtag_add_dr_scan(int num_fields, const struct scan_field *fields, tap_state_t state)
{

	int j;
	struct jtag_tap *tap, *nextTap;
	for (tap = jtag_tap_next_enabled(NULL); tap!= NULL; tap = nextTap)
	{
		nextTap = jtag_tap_next_enabled(tap);
		int found = 0;
		int pause = (nextTap==NULL);

		for (j = 0; j < num_fields; j++)
		{
			if (tap == fields[j].tap)
			{
				found = 1;

				scanFields(1, fields+j, TAP_DRSHIFT, pause);
			}
		}
		if (!found)
		{
			struct scan_field tmp;
			/* program the scan field to 1 bit length, and ignore it's value */
			tmp.num_bits = 1;
			tmp.out_value = NULL;
			tmp.in_value = NULL;

			scanFields(1, &tmp, TAP_DRSHIFT, pause);
		}
		else
		{
		}
	}
	gotoEndState(state);
	return ERROR_OK;
}

int interface_jtag_add_plain_dr_scan(int num_fields, const struct scan_field *fields, tap_state_t state)
{
	scanFields(num_fields, fields, TAP_DRSHIFT, 1);
	gotoEndState(state);
	return ERROR_OK;
}


int interface_jtag_add_tlr()
{
	setCurrentState(TAP_RESET);
	return ERROR_OK;
}




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
	for (i = 0; i < num_cycles; i += 32)
	{
		int num;
		num = 32;
		if (num_cycles-i < num)
		{
			num = num_cycles-i;
		}
		shiftValueInner(clockstate, clockstate, num, 0);
	}

#if !TEST_MANUAL()
	/* finish in end_state */
	setCurrentState(state);
#else
	tap_state_t t = TAP_IDLE;
	/* test manual drive code on any target */
	int tms;
	uint8_t tms_scan = tap_get_tms_path(t, state);
	int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	for (i = 0; i < tms_count; i++)
	{
		tms = (tms_scan >> i) & 1;
		waitIdle();
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x28,  tms);
	}
	waitIdle();
	ZY1000_POKE(ZY1000_JTAG_BASE + 0x20, state);
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

int interface_jtag_add_sleep(uint32_t us)
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

	tap_state_t cur_state = cmd_queue_cur_state;

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
		ZY1000_POKE(ZY1000_JTAG_BASE + 0x28,  tms);

		cur_state = path[state_count];
		state_count++;
		num_states--;
	}

	waitIdle();
	ZY1000_POKE(ZY1000_JTAG_BASE + 0x20,  cur_state);
	return ERROR_OK;
}



void embeddedice_write_dcc(struct jtag_tap *tap, int reg_addr, uint8_t *buffer, int little, int count)
{
//	static int const reg_addr = 0x5;
	tap_state_t end_state = jtag_get_end_state();
	if (jtag_tap_next_enabled(jtag_tap_next_enabled(NULL)) == NULL)
	{
		/* better performance via code duplication */
		if (little)
		{
			int i;
			for (i = 0; i < count; i++)
			{
				shiftValueInner(TAP_DRSHIFT, TAP_DRSHIFT, 32, fast_target_buffer_get_u32(buffer, 1));
				shiftValueInner(TAP_DRSHIFT, end_state, 6, reg_addr | (1 << 5));
				buffer += 4;
			}
		} else
		{
			int i;
			for (i = 0; i < count; i++)
			{
				shiftValueInner(TAP_DRSHIFT, TAP_DRSHIFT, 32, fast_target_buffer_get_u32(buffer, 0));
				shiftValueInner(TAP_DRSHIFT, end_state, 6, reg_addr | (1 << 5));
				buffer += 4;
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


static const struct command_registration zy1000_commands[] = {
	{
		.name = "power",
		.handler = handle_power_command,
		.mode = COMMAND_ANY,
		.help = "Turn power switch to target on/off. "
			"With no arguments, prints status.",
		.usage = "('on'|'off)",
	},
	{
		.name = "zy1000_version",
		.mode = COMMAND_ANY,
		.jim_handler = jim_zy1000_version,
		.help = "Print version info for zy1000.",
		.usage = "['openocd'|'zy1000'|'date'|'time'|'pcb'|'fpga']",
	},
	{
		.name = "powerstatus",
		.mode = COMMAND_ANY,
		.jim_handler = zylinjtag_Jim_Command_powerstatus,
		.help = "Returns power status of target",
	},
#ifdef CYGPKG_HAL_NIOS2
	{
		.name = "updatezy1000firmware",
		.mode = COMMAND_ANY,
		.jim_handler = jim_zy1000_writefirmware,
		.help = "writes firmware to flash",
		/* .usage = "some_string", */
	},
#endif
	COMMAND_REGISTRATION_DONE
};



struct jtag_interface zy1000_interface =
{
	.name = "ZY1000",
	.execute_queue = NULL,
	.speed = zy1000_speed,
	.commands = zy1000_commands,
	.init = zy1000_init,
	.quit = zy1000_quit,
	.khz = zy1000_khz,
	.speed_div = zy1000_speed_div,
	.power_dropout = zy1000_power_dropout,
	.srst_asserted = zy1000_srst_asserted,
};

