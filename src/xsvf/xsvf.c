/***************************************************************************
 *	 Copyright (C) 2005 by Dominic Rath									   *
 *	 Dominic.Rath@gmx.de													   *
 *																		   *
 *	 Copyright (C) 2007,2008 Øyvind Harboe								   *
 *	 oyvind.harboe@zylin.com												   *
 *																		   *
 *	 Copyright (C) 2008 Peter Hettkamp									   *
 *	 peter.hettkamp@htp-tel.de											   *
 *																		   *
 *	 Copyright (C) 2009 SoftPLC Corporation. http://softplc.com             *
 *	 dick@softplc.com											           *
 *                                                                          *
 *	 This program is free software; you can redistribute it and/or modify   *
 *	 it under the terms of the GNU General Public License as published by   *
 *	 the Free Software Foundation; either version 2 of the License, or	   *
 *	 (at your option) any later version.									   *
 *																		   *
 *	 This program is distributed in the hope that it will be useful,		   *
 *	 but WITHOUT ANY WARRANTY; without even the implied warranty of		   *
 *	 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the		   *
 *	 GNU General Public License for more details.						   *
 *																		   *
 *	 You should have received a copy of the GNU General Public License	   *
 *	 along with this program; if not, write to the						   *
 *	 Free Software Foundation, Inc.,										   *
 *	 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.			   *
 ***************************************************************************/


/* The specification for SVF is available here:
 * http://www.asset-intertech.com/support/svf.pdf
 * Below, this document is refered to as the "SVF spec".
 *
 * The specification for XSVF is available here:
 * http://www.xilinx.com/support/documentation/application_notes/xapp503.pdf
 * Below, this document is refered to as the "XSVF spec".
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "xsvf.h"

#include "jtag.h"
#include "command.h"
#include "log.h"

#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include <sys/time.h>
#include <time.h>


/* XSVF commands, from appendix B of xapp503.pdf  */
#define XCOMPLETE		0x00
#define XTDOMASK			0x01
#define XSIR				0x02
#define XSDR				0x03
#define XRUNTEST			0x04
#define XREPEAT			0x07
#define XSDRSIZE			0x08
#define XSDRTDO			0x09
#define XSETSDRMASKS		0x0A
#define XSDRINC			0x0B
#define XSDRB			0x0C
#define XSDRC			0x0D
#define XSDRE			0x0E
#define XSDRTDOB			0x0F
#define XSDRTDOC			0x10
#define XSDRTDOE			0x11
#define XSTATE			0x12
#define XENDIR			0x13
#define XENDDR			0x14
#define XSIR2			0x15
#define XCOMMENT			0x16
#define XWAIT			0x17

/* XWAITSTATE is not in the xilinx XSVF spec, but the svf2xsvf.py translator
 * generates this.  Arguably it is needed because the XSVF XRUNTEST command
 * was ill conceived and does not directly flow out of the SVF RUNTEST command.
 * This XWAITSTATE does map directly from the SVF RUNTEST command.
 */
#define XWAITSTATE		0x18

/* Lattice has extended the SVF file format, and Dick Hollenbeck's python based
 * SVF2XSVF converter supports these 3 additional XSVF opcodes, LCOUNT, LDELAY, LSDR.
 * Here is an example of usage of the 3 lattice opcode extensions:

! Set the maximum loop count to 25.
LCOUNT	25;
! Step to DRPAUSE give 5 clocks and wait for 1.00e+000 SEC.
LDELAY	DRPAUSE	5 TCK	1.00E-003 SEC;
! Test for the completed status. Match means pass.
! Loop back to LDELAY line if not match and loop count less than 25.

LSDR 1  TDI  (0)
		TDO  (1);
*/

#define LCOUNT			0x19
#define LDELAY			0x1A
#define LSDR				0x1B
#define XTRST			0x1C


/* XSVF valid state values for the XSTATE command, from appendix B of xapp503.pdf */
#define XSV_RESET		0x00
#define XSV_IDLE			0x01
#define XSV_DRSELECT		0x02
#define XSV_DRCAPTURE	0x03
#define XSV_DRSHIFT		0x04
#define XSV_DREXIT1		0x05
#define XSV_DRPAUSE		0x06
#define XSV_DREXIT2		0x07
#define XSV_DRUPDATE		0x08
#define XSV_IRSELECT		0x09
#define XSV_IRCAPTURE	0x0A
#define XSV_IRSHIFT		0x0B
#define XSV_IREXIT1		0x0C
#define XSV_IRPAUSE		0x0D
#define XSV_IREXIT2		0x0E
#define XSV_IRUPDATE		0x0F

/* arguments to XTRST */
#define XTRST_ON			0
#define XTRST_OFF		1
#define XTRST_Z			2
#define XTRST_ABSENT		3

#define XSTATE_MAX_PATH 12

static int handle_xsvf_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

static int xsvf_fd = 0;


/* map xsvf tap state to an openocd "tap_state_t" */
static tap_state_t xsvf_to_tap( int xsvf_state )
{
	tap_state_t	ret;

	switch( xsvf_state )
	{
	case XSV_RESET:			ret = TAP_RESET;			break;
	case XSV_IDLE:			ret = TAP_IDLE;			break;
	case XSV_DRSELECT:		ret = TAP_DRSELECT;		break;
	case XSV_DRCAPTURE:		ret = TAP_DRCAPTURE;		break;
	case XSV_DRSHIFT:		ret = TAP_DRSHIFT;		break;
	case XSV_DREXIT1:		ret = TAP_DREXIT1;		break;
	case XSV_DRPAUSE:		ret = TAP_DRPAUSE;		break;
	case XSV_DREXIT2:		ret = TAP_DREXIT2;		break;
	case XSV_DRUPDATE:		ret = TAP_DRUPDATE;		break;
	case XSV_IRSELECT:		ret = TAP_IRSELECT;		break;
	case XSV_IRCAPTURE:		ret = TAP_IRCAPTURE;		break;
	case XSV_IRSHIFT:		ret = TAP_IRSHIFT;		break;
	case XSV_IREXIT1:		ret = TAP_IREXIT1;		break;
	case XSV_IRPAUSE:		ret = TAP_IRPAUSE;		break;
	case XSV_IREXIT2:		ret = TAP_IREXIT2;		break;
	case XSV_IRUPDATE:		ret = TAP_IRUPDATE;		break;
	default:
		LOG_ERROR( "UNKNOWN XSVF STATE 0x%02X", xsvf_state );
		exit(1);
	}

	return ret;
}


/* xsvf has it's own definition of a statemove. This needs
 * to be handled according to the xsvf spec, which has nothing
 * to do with the JTAG spec or OpenOCD as such.
 *
 * Implemented via jtag_add_pathmove().
 */
static void xsvf_add_statemove(tap_state_t state)
{
	tap_state_t moves[7]; 	/* max # of transitions */
	tap_state_t curstate = cmd_queue_cur_state;
	int i;

	u8 move = tap_get_tms_path(cmd_queue_cur_state, state);

	if (state != TAP_RESET  &&  state==cmd_queue_cur_state)
		return;

	if(state==TAP_RESET)
	{
		jtag_add_tlr();
		return;
	}

	for (i=0; i<7; i++)
	{
		int j = (move >> i) & 1;
		if (j)
		{
			curstate = tap_state_transition(curstate, true);
		}
		else
		{
			curstate = tap_state_transition(curstate, false);
		}
		moves[i] = curstate;
	}

	jtag_add_pathmove(7, moves);
}

int xsvf_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "xsvf", handle_xsvf_command,
		COMMAND_EXEC, "run xsvf <file> [virt2] [quiet]");

	return ERROR_OK;
}

static int xsvf_read_buffer(int num_bits, int fd, u8* buf)
{
	int num_bytes;

	for (num_bytes = (num_bits + 7) / 8; num_bytes > 0; num_bytes--)
	{
		/* reverse the order of bytes as they are read sequentially from file */
		if (read(fd, buf + num_bytes - 1, 1) < 0)
			return ERROR_XSVF_EOF;
	}

	return ERROR_OK;
}


static int xsvf_read_xstates(int fd, tap_state_t *path, int max_path, int *path_len)
{
	char c;
	u8   uc;

	while ((read(fd, &c, 1) > 0) && (c == XSTATE))
	{
		tap_state_t	mystate;

		if (*path_len > max_path)
		{
			LOG_WARNING("XSTATE path longer than max_path");
			break;
		}
		if (read(fd, &uc, 1) < 0)
		{
			return ERROR_XSVF_EOF;
		}

		mystate = xsvf_to_tap(uc);

		LOG_DEBUG("XSTATE %02X %s", uc, tap_state_name(mystate) );

		path[(*path_len)++] = mystate;
	}

	lseek(fd, -1, SEEK_CUR);

	return ERROR_OK;
}


static int handle_xsvf_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u8 *dr_out_buf = NULL; 				/* from host to device (TDI) */
	u8 *dr_in_buf = NULL;				/* from device to host (TDO) */
	u8 *dr_in_mask = NULL;

	int xsdrsize = 0;
	int xruntest = 0;					/* number of TCK cycles OR microseconds */
	int xrepeat	 = 0;					/* number of retries */

	tap_state_t	xendir = TAP_IDLE;		/* see page 8 of the SVF spec, initial xendir to be TAP_IDLE */
	tap_state_t xenddr = TAP_IDLE;

	u8  		opcode;
	u8		uc;
	long		file_offset = 0;

	int		loop_count = 0;
	tap_state_t	loop_state = TAP_IDLE;
	int		loop_clocks = 0;
	int		loop_usecs = 0;

	int 		do_abort = 0;
	int 		unsupported = 0;
	int 		tdo_mismatch = 0;
	int 		result;
	int		verbose = 1;
	char*	filename;

	int 		runtest_requires_tck = 0;	/* a flag telling whether to clock TCK during waits, or simply sleep, controled by virt2 */


	/* use NULL to indicate a "plain" xsvf file which accounts for
	   additional devices in the scan chain, otherwise the device
	   that should be affected
	*/
	jtag_tap_t *tap = NULL;

	if (argc < 2)
	{
		command_print(cmd_ctx, "usage: xsvf <device#|plain> <file> [<variant>] [quiet]");
		return ERROR_FAIL;
	}

	filename = args[1];		/* we mess with args starting point below, snapshot filename here */

	if (strcmp(args[0], "plain") != 0)
	{
		tap = jtag_TapByString( args[0] );
		if (!tap )
		{
			command_print( cmd_ctx, "Tap: %s unknown", args[0] );
			return ERROR_FAIL;
		}
	}

	if ((xsvf_fd = open(filename, O_RDONLY)) < 0)
	{
		command_print(cmd_ctx, "file \"%s\" not found", filename);
		return ERROR_FAIL;
	}

	/* if this argument is present, then interpret xruntest counts as TCK cycles rather than as usecs */
	if ((argc > 2) && (strcmp(args[2], "virt2") == 0))
	{
		runtest_requires_tck = 1;
		--argc;
		++args;
	}

	if ((argc > 2) && (strcmp(args[2], "quiet") == 0))
	{
		verbose = 0;
	}

	LOG_USER("xsvf processing file: \"%s\"", filename);

	while( read(xsvf_fd, &opcode, 1) > 0 )
	{
		/* record the position of the just read opcode within the file */
		file_offset = lseek(xsvf_fd, 0, SEEK_CUR) - 1;

		switch (opcode)
		{
			case XCOMPLETE:
				LOG_DEBUG("XCOMPLETE");

				result = jtag_execute_queue();
				if (result != ERROR_OK)
				{
					tdo_mismatch = 1;
					break;
				}
				break;

			case XTDOMASK:
				LOG_DEBUG("XTDOMASK");
				if (dr_in_mask && (xsvf_read_buffer(xsdrsize, xsvf_fd, dr_in_mask) != ERROR_OK))
					do_abort = 1;
				break;

			case XRUNTEST:
				{
					u8	xruntest_buf[4];

					if (read(xsvf_fd, xruntest_buf, 4) < 0)
					{
						do_abort = 1;
						break;
					}

					xruntest = be_to_h_u32(xruntest_buf);
					LOG_DEBUG("XRUNTEST %d 0x%08X", xruntest, xruntest);
				}
				break;

			case XREPEAT:
				{
					u8 myrepeat;

					if (read(xsvf_fd, &myrepeat, 1) < 0)
						do_abort = 1;
					else
					{
						xrepeat = myrepeat;
						LOG_DEBUG("XREPEAT %d", xrepeat );
					}
				}
				break;

			case XSDRSIZE:
				{
					u8	xsdrsize_buf[4];

					if (read(xsvf_fd, xsdrsize_buf, 4) < 0)
					{
						do_abort = 1;
						break;
					}

					xsdrsize = be_to_h_u32(xsdrsize_buf);
					LOG_DEBUG("XSDRSIZE %d", xsdrsize);

					if( dr_out_buf ) free(dr_out_buf);
					if( dr_in_buf)   free(dr_in_buf);
					if( dr_in_mask)  free(dr_in_mask);

					dr_out_buf = malloc((xsdrsize + 7) / 8);
					dr_in_buf = malloc((xsdrsize + 7) / 8);
					dr_in_mask = malloc((xsdrsize + 7) / 8);
				}
				break;

			case XSDR:		/* these two are identical except for the dr_in_buf */
			case XSDRTDO:
				{
					int limit = xrepeat;
					int	matched = 0;
					int attempt;

					const char* op_name = (opcode == XSDR ? "XSDR" : "XSDRTDO");

					if (xsvf_read_buffer(xsdrsize, xsvf_fd, dr_out_buf) != ERROR_OK)
					{
						do_abort = 1;
						break;
					}

					if (opcode == XSDRTDO)
					{
						if(xsvf_read_buffer(xsdrsize, xsvf_fd, dr_in_buf)  != ERROR_OK )
						{
							do_abort = 1;
							break;
						}
					}

					if (limit < 1)
						limit = 1;

					LOG_DEBUG("%s %d", op_name, xsdrsize);

					for( attempt=0; attempt<limit;  ++attempt )
					{
						scan_field_t field;

						if( attempt>0 )
						{
							/* perform the XC9500 exception handling sequence shown in xapp067.pdf and
							   illustrated in psuedo code at end of this file.  We start from state
							   DRPAUSE:
							   go to Exit2-DR
							   go to Shift-DR
							   go to Exit1-DR
							   go to Update-DR
							   go to Run-Test/Idle

							   This sequence should be harmless for other devices, and it
							   will be skipped entirely if xrepeat is set to zero.
							*/

							static tap_state_t exception_path[] = {
								TAP_DREXIT2,
								TAP_DRSHIFT,
								TAP_DREXIT1,
								TAP_DRUPDATE,
								TAP_IDLE,
							};

							jtag_add_pathmove( sizeof(exception_path)/sizeof(exception_path[0]), exception_path);

							if (verbose)
								LOG_USER("%s %d retry %d", op_name, xsdrsize, attempt);
						}

						field.tap = tap;
						field.num_bits = xsdrsize;
						field.out_value = dr_out_buf;
						field.out_mask = NULL;
						field.in_value = NULL;

						jtag_set_check_value(&field, dr_in_buf, dr_in_mask, NULL);

						if (tap == NULL)
							jtag_add_plain_dr_scan(1, &field, TAP_DRPAUSE);
						else
							jtag_add_dr_scan(1, &field, TAP_DRPAUSE);

						/* LOG_DEBUG("FLUSHING QUEUE"); */
						result = jtag_execute_queue();
						if (result == ERROR_OK)
						{
							matched = 1;
							break;
						}
					}

					if (!matched)
					{
						LOG_USER( "%s mismatch", op_name);
						tdo_mismatch = 1;
						break;
					}

					/* See page 19 of XSVF spec regarding opcode "XSDR" */
					if (xruntest)
					{
						xsvf_add_statemove(TAP_IDLE);

						if (runtest_requires_tck)
							jtag_add_clocks(xruntest);
						else
							jtag_add_sleep(xruntest);
					}
					else if (xendir != TAP_DRPAUSE)	/* we are already in TAP_DRPAUSE */
						xsvf_add_statemove(xenddr);
				}
				break;

			case XSETSDRMASKS:
				LOG_ERROR("unsupported XSETSDRMASKS\n");
				unsupported = 1;
				break;

			case XSDRINC:
				LOG_ERROR("unsupported XSDRINC\n");
				unsupported = 1;
				break;

			case XSDRB:
				LOG_ERROR("unsupported XSDRB\n");
				unsupported = 1;
				break;

			case XSDRC:
				LOG_ERROR("unsupported XSDRC\n");
				unsupported = 1;
				break;

			case XSDRE:
				LOG_ERROR("unsupported XSDRE\n");
				unsupported = 1;
				break;

			case XSDRTDOB:
				LOG_ERROR("unsupported XSDRTDOB\n");
				unsupported = 1;
				break;

			case XSDRTDOC:
				LOG_ERROR("unsupported XSDRTDOC\n");
				unsupported = 1;
				break;

			case XSDRTDOE:
				LOG_ERROR("unsupported XSDRTDOE\n");
				unsupported = 1;
				break;

			case XSTATE:
				{
					tap_state_t	mystate;
					tap_state_t *path;
					int path_len;

					if (read(xsvf_fd, &uc, 1) < 0)
					{
						do_abort = 1;
						break;
					}

					mystate = xsvf_to_tap(uc);

					LOG_DEBUG("XSTATE 0x%02X %s", uc, tap_state_name(mystate) );

					path = calloc(XSTATE_MAX_PATH, 4);
					path_len = 1;

					path[0] = mystate;
					if (xsvf_read_xstates(xsvf_fd, path, XSTATE_MAX_PATH, &path_len) != ERROR_OK)
						do_abort = 1;
					else
					{
						int i,lasti;

						/* here the trick is that jtag_add_pathmove() must end in a stable
						 * state, so we must only invoke jtag_add_tlr() when we absolutely
						 * have to
						 */
						for(i=0,lasti=0;  i<path_len;  i++)
						{
							if(path[i]==TAP_RESET)
							{
								if(i>lasti)
								{
									jtag_add_pathmove(i-lasti,path+lasti);
								}
								lasti=i+1;
								jtag_add_tlr();
							}
						}
						if(i>=lasti)
						{
							jtag_add_pathmove(i-lasti, path+lasti);
						}
					}
					free(path);
				}
				break;

			case XENDIR:
				{
					tap_state_t	 mystate;

					if (read(xsvf_fd, &uc, 1) < 0)
					{
						do_abort = 1;
						break;
					}

					/* see page 22 of XSVF spec */
					mystate = uc == 1 ? TAP_IRPAUSE : TAP_IDLE;

					LOG_DEBUG("XENDIR 0x%02X %s", uc, tap_state_name(mystate));

					/* assuming that the XRUNTEST comes from SVF RUNTEST, then only these states
					 * should come here because the SVF spec only allows these with a RUNTEST
					 */
					if (mystate != TAP_IRPAUSE && mystate != TAP_DRPAUSE && mystate != TAP_RESET && mystate != TAP_IDLE )
					{
						LOG_ERROR("illegal XENDIR endstate: \"%s\"", tap_state_name(mystate));
						unsupported = 1;
						break;
					}
					xendir = mystate;
				}
				break;

			case XENDDR:
				{
					tap_state_t	 mystate;

					if (read(xsvf_fd, &uc, 1) < 0)
					{
						do_abort = 1;
						break;
					}

					/* see page 22 of XSVF spec */
					mystate = uc == 1 ? TAP_DRPAUSE : TAP_IDLE;

					LOG_DEBUG("XENDDR %02X %s", uc, tap_state_name(mystate));

					if (mystate != TAP_IRPAUSE && mystate != TAP_DRPAUSE && mystate != TAP_RESET && mystate != TAP_IDLE )
					{
						LOG_ERROR("illegal XENDDR endstate: \"%s\"", tap_state_name( mystate ));
						unsupported = 1;
						break;
					}
					xenddr = mystate;
				}
				break;

			case XSIR:
			case XSIR2:
				{
					u8	short_buf[2];
					u8*	ir_buf;
					int bitcount;
					tap_state_t my_end_state = xruntest ? TAP_IDLE : xendir;

					if( opcode == XSIR )
					{
						/* one byte bitcount */
						if (read(xsvf_fd, short_buf, 1) < 0)
						{
							do_abort = 1;
							break;
						}
						bitcount = short_buf[0];
						LOG_DEBUG("XSIR %d", bitcount);
					}
					else
					{
						if (read(xsvf_fd, short_buf, 2) < 0)
						{
							do_abort = 1;
							break;
						}
						bitcount = be_to_h_u16(short_buf);
						LOG_DEBUG("XSIR2 %d", bitcount);
					}

					ir_buf = malloc((bitcount+7) / 8);

					if (xsvf_read_buffer(bitcount, xsvf_fd, ir_buf) != ERROR_OK)
						do_abort = 1;
					else
					{
						scan_field_t field;

						field.tap = tap;
						field.num_bits = bitcount;
						field.out_value = ir_buf;
						field.out_mask = NULL;
						field.in_value = NULL;
						field.in_check_value = NULL;
						field.in_check_mask = NULL;
						field.in_handler = NULL;
						field.in_handler_priv = NULL;

						if (tap == NULL)
							jtag_add_plain_ir_scan(1, &field, my_end_state);
						else
							jtag_add_ir_scan(1, &field, my_end_state);

						if (xruntest)
						{
							if (runtest_requires_tck)
								jtag_add_clocks(xruntest);
							else
								jtag_add_sleep(xruntest);
						}

						/* Note that an -irmask of non-zero in your config file
						 * can cause this to fail.  Setting -irmask to zero cand work
						 * around the problem.
						 */

						/* LOG_DEBUG("FLUSHING QUEUE"); */
						result = jtag_execute_queue();
						if(result != ERROR_OK)
						{
							tdo_mismatch = 1;
						}
					}
					free(ir_buf);
				}
				break;

			case XCOMMENT:
				{
					int		ndx = 0;
					char 	comment[128];

					do
					{
						if (read(xsvf_fd, &uc, 1) < 0)
						{
							do_abort = 1;
							break;
						}

						if ( ndx < sizeof(comment)-1 )
							comment[ndx++] = uc;

					} while (uc != 0);

					comment[sizeof(comment)-1] = 0;		/* regardless, terminate */
					if (verbose)
						LOG_USER(comment);
				}
				break;

			case XWAIT:
				{
					/* expected in stream:
					   XWAIT <u8 wait_state> <u8 end_state> <u32 usecs>
					*/

					u8	wait;
					u8	end;
					u8	delay_buf[4];

					tap_state_t wait_state;
					tap_state_t end_state;
					int 	delay;

					if ( read(xsvf_fd, &wait, 1) < 0
					  || read(xsvf_fd, &end, 1) < 0
					  || read(xsvf_fd, delay_buf, 4) < 0)
					{
						do_abort = 1;
						break;
					}

					wait_state = xsvf_to_tap(wait);
					end_state  = xsvf_to_tap(end);
					delay      = be_to_h_u32(delay_buf);

					LOG_DEBUG("XWAIT %s %s usecs:%d", tap_state_name(wait_state), tap_state_name(end_state), delay);

					if (runtest_requires_tck && wait_state == TAP_IDLE )
					{
						jtag_add_runtest(delay, end_state);
					}
					else
					{
						xsvf_add_statemove( wait_state );
						jtag_add_sleep(delay);
						xsvf_add_statemove( end_state );
					}
				}
				break;

			case XWAITSTATE:
				{
					/* expected in stream:
					   XWAITSTATE <u8 wait_state> <u8 end_state> <u32 clock_count> <u32 usecs>
					*/

					u8  clock_buf[4];
					u8  	usecs_buf[4];
					u8	wait;
					u8	end;
					tap_state_t wait_state;
					tap_state_t end_state;
					int clock_count;
					int usecs;

					if ( read(xsvf_fd, &wait, 1) < 0
					 ||  read(xsvf_fd, &end, 1) < 0
					 ||  read(xsvf_fd, clock_buf, 4) < 0
					 ||  read(xsvf_fd, usecs_buf, 4) < 0 )
					{
						do_abort = 1;
						break;
					}

					wait_state = xsvf_to_tap( wait );
					end_state  = xsvf_to_tap( end );

					clock_count = be_to_h_u32(clock_buf);
					usecs       = be_to_h_u32(usecs_buf);

					LOG_DEBUG("XWAITSTATE %s %s clocks:%i usecs:%i",
						tap_state_name(wait_state),
						tap_state_name(end_state),
						clock_count, usecs);

					/* the following states are 'stable', meaning that they have a transition
					 * in the state diagram back to themselves.  This is necessary because we will
					 * be issuing a number of clocks in this state.  This set of allowed states is also
					 * determined by the SVF RUNTEST command's allowed states.
					 */
					if (wait_state != TAP_IRPAUSE && wait_state != TAP_DRPAUSE && wait_state != TAP_RESET && wait_state != TAP_IDLE)
					{
						LOG_ERROR("illegal XWAITSTATE wait_state: \"%s\"", tap_state_name( wait_state ));
						unsupported = 1;
					}

					xsvf_add_statemove( wait_state );

					jtag_add_clocks( clock_count );

					jtag_add_sleep( usecs );

					xsvf_add_statemove( end_state );
				}
				break;

			case LCOUNT:
				{
					/* expected in stream:
					   LCOUNT <u32 loop_count>
					*/
					u8  count_buf[4];

					if ( read(xsvf_fd, count_buf, 4) < 0 )
					{
						do_abort = 1;
						break;
					}

					loop_count = be_to_h_u32(count_buf);
					LOG_DEBUG("LCOUNT %d", loop_count);
				}
				break;

			case LDELAY:
				{
					/* expected in stream:
					   LDELAY <u8 wait_state> <u32 clock_count> <u32 usecs_to_sleep>
					*/
					u8	state;
					u8  clock_buf[4];
					u8  usecs_buf[4];

					if ( read(xsvf_fd, &state, 1) < 0
					  || read(xsvf_fd, clock_buf, 4) < 0
					  ||	 read(xsvf_fd, usecs_buf, 4) < 0 )
					{
						do_abort = 1;
						break;
					}

					loop_state  = xsvf_to_tap(state);
					loop_clocks = be_to_h_u32(clock_buf);
					loop_usecs  = be_to_h_u32(usecs_buf);

					LOG_DEBUG("LDELAY %s clocks:%d usecs:%d", tap_state_name(loop_state), loop_clocks, loop_usecs);
				}
				break;

			/* LSDR is more like XSDRTDO than it is like XSDR.  It uses LDELAY which
			 * comes with clocks !AND! sleep requirements.
			 */
			case LSDR:
				{
					int limit = loop_count;
					int matched = 0;
					int attempt;

					LOG_DEBUG("LSDR");

					if ( xsvf_read_buffer(xsdrsize, xsvf_fd, dr_out_buf) != ERROR_OK
					  || xsvf_read_buffer(xsdrsize, xsvf_fd, dr_in_buf) != ERROR_OK )
					{
						do_abort = 1;
						break;
					}

					if (limit < 1)
						limit = 1;

					for( attempt=0; attempt<limit;  ++attempt )
					{
						scan_field_t field;

						xsvf_add_statemove( loop_state );
						jtag_add_clocks(loop_clocks);
						jtag_add_sleep(loop_usecs);

						field.tap = tap;
						field.num_bits = xsdrsize;
						field.out_value = dr_out_buf;
						field.out_mask = NULL;
						field.in_value = NULL;

						if (attempt > 0 && verbose)
							LOG_USER("LSDR retry %d", attempt);

						jtag_set_check_value(&field, dr_in_buf, dr_in_mask, NULL);
						if (tap == NULL)
							jtag_add_plain_dr_scan(1, &field, TAP_DRPAUSE);
						else
							jtag_add_dr_scan(1, &field, TAP_DRPAUSE);

						/* LOG_DEBUG("FLUSHING QUEUE"); */
						result = jtag_execute_queue();
						if(result == ERROR_OK)
						{
							matched = 1;
							break;
						}
					}

					if (!matched )
					{
						LOG_USER( "LSDR mismatch" );
						tdo_mismatch = 1;
						break;
					}
				}
				break;

			case XTRST:
				{
					u8	trst_mode;

					if (read(xsvf_fd, &trst_mode, 1) < 0)
					{
						do_abort = 1;
						break;
					}

					switch( trst_mode )
					{
					case XTRST_ON:
						jtag_add_reset(1, 0);
						break;
					case XTRST_OFF:
					case XTRST_Z:
						jtag_add_reset(0, 0);
						break;
					case XTRST_ABSENT:
						break;
					default:
						LOG_ERROR( "XTRST mode argument (0x%02X) out of range", trst_mode );
						do_abort = 1;
					}
				}
				break;

			default:
				LOG_ERROR("unknown xsvf command (0x%02X)\n", uc);
				unsupported = 1;
		}

		if (do_abort || unsupported || tdo_mismatch)
		{
			LOG_DEBUG("xsvf failed, setting taps to reasonable state");

			/* upon error, return the TAPs to a reasonable state */
			xsvf_add_statemove( TAP_IDLE );
			jtag_execute_queue();
			break;
		}
	}

	if (tdo_mismatch)
	{
		command_print(cmd_ctx, "TDO mismatch, somewhere near offset %lu in xsvf file, aborting",
					  file_offset );


		return ERROR_FAIL;
	}

	if (unsupported)
	{
		command_print(cmd_ctx,
			 "unsupported xsvf command: 0x%02X in xsvf file at offset %ld, aborting",
					  uc,  lseek(xsvf_fd, 0, SEEK_CUR)-1 );
		return ERROR_FAIL;
	}

	if (do_abort)
	{
		command_print(cmd_ctx, "premature end of xsvf file detected, aborting");
		return ERROR_FAIL;
	}

	if (dr_out_buf)
		free(dr_out_buf);

	if (dr_in_buf)
		free(dr_in_buf);

	if (dr_in_mask)
		free(dr_in_mask);

	close(xsvf_fd);

	command_print(cmd_ctx, "XSVF file programmed successfully");

	return ERROR_OK;
}


/* PSUEDO-Code from Xilinx Appnote XAPP067.pdf:

the following pseudo code clarifies the intent of the xrepeat support.  The
flow given is for the entire processing of an SVF file, not an XSVF file.
No idea if this is just for the XC9500/XL/XV devices or all Xilinx parts.

"Pseudo-Code Algorithm for SVF-Based ISP"

1. Go to Test-Logic-Reset state
2. Go to Run-Test Idle state
3. Read SVF record

4. if SIR record then
	   go to Shift-IR state
	   Scan in <TDI value>

5. else if SDR record then
	   set <repeat count> to 0
	   store <TDI value> as <current TDI value>
	   store <TDO value> as <current TDO value>
6. go to Shift-DR state
	   scan in <current TDI value>
	   if <current TDO value> is specified then
		   if <current TDO value> does not equal <actual TDO value> then
			   if <repeat count> > 32 then
				   LOG ERROR
				   go to Run-Test Idle state
				   go to Step 3
			   end if
			   go to Pause-DR
			   go to Exit2-DR
			   go to Shift-DR
			   go to Exit1-DR
			   go to Update-DR
			   go to Run-Test/Idle
			   increment <repeat count> by 1
			   pause <current pause time> microseconds
			   go to Step 6)
		   end if
	   else
		   go to Run-Test Idle state
		   go to Step 3
	   endif
else if RUNTEST record then
   pause tester for <TCK value> microseconds
   store <TCK value> as <current pause time>
end if

*/
