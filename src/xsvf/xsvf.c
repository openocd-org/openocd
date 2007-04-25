/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#define XSTATE_MAX_PATH (12)

int handle_xsvf_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int xsvf_fd = 0;

u8 *dr_out_buf;	/* from host to device (TDI) */
u8 *dr_in_buf;	/* from device to host (TDO) */
u8 *dr_in_mask;

int xsdrsize = 0;
int xruntest = 0;	/* number of TCK cycles / microseconds */
int xrepeat = 0x20; /* number of XC9500 retries */

int xendir = 0;
int xenddr = 0;

enum tap_state xsvf_to_tap[] =
{
	TAP_TLR, TAP_RTI,
	TAP_SDS, TAP_CD, TAP_SD, TAP_E1D, TAP_PD, TAP_E2D, TAP_UD,
	TAP_SIS, TAP_CI, TAP_SI, TAP_E1I, TAP_PI, TAP_E2I, TAP_UI,
};

int tap_to_xsvf[] =
{
	0x0, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x1, 0x9, 0xa, 0xb, 0xc, 0xe, 0xf
};

int xsvf_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "xsvf", handle_xsvf_command,
		COMMAND_EXEC, "run xsvf <file>");

	return ERROR_OK;
}

int xsvf_read_buffer(int num_bits, int fd, u8* buf)
{
	int num_bytes;

	for (num_bytes = (num_bits + 7) / 8; num_bytes > 0; num_bytes--)
	{
		if (read(fd, buf + num_bytes - 1, 1) < 0)
			return ERROR_XSVF_EOF;
	}

	return ERROR_OK;
}

int xsvf_read_xstates(int fd, enum tap_state *path, int max_path, int *path_len)
{
	char c;
	unsigned char uc;
	
	while ((read(fd, &c, 1) > 0) && (c == 0x12))
	{
		if (*path_len > max_path)
		{
			WARNING("XSTATE path longer than max_path");
			break;
		}
		if (read(fd, &uc, 1) < 0)
		{
			return ERROR_XSVF_EOF;
		}
		path[(*path_len)++] = xsvf_to_tap[uc];
	}
	
	lseek(fd, -1, SEEK_CUR);
	
	return ERROR_OK;
}

int handle_xsvf_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	char c;
	u8 buf4[4], buf2[2];
	unsigned char uc, uc2;
	unsigned int ui;
	unsigned short us;

	int do_abort = 0;
	int unsupported = 0;
	int tdo_mismatch = 0;
	
	int runtest_requires_tck = 0;
	
	int device = -1;	/* use -1 to indicate a "plain" xsvf file which accounts for additional devices in the scan chain, otherwise the device that should be affected */

	if (argc < 2)
	{
		command_print(cmd_ctx, "usage: xsvf <device#|plain> <file> <variant>");
		return ERROR_OK;
	}

	if (strcmp(args[0], "plain") != 0)
	{
		device = strtoul(args[0], NULL, 0);
	}

	if ((xsvf_fd = open(args[1], O_RDONLY)) < 0)
	{
		command_print(cmd_ctx, "file %s not found", args[0]);
		return ERROR_OK;
	}
	
	if ((argc > 2) && (strcmp(args[2], "virt2") == 0))
	{
		runtest_requires_tck = 1;
	}

	while (read(xsvf_fd, &c, 1) > 0)
	{
		switch (c)
		{
			case 0x00:	/* XCOMPLETE */
				DEBUG("XCOMPLETE");
				if (jtag_execute_queue() != ERROR_OK)
				{
					tdo_mismatch = 1;
					break;	
				}
				break;
			case 0x01:	/* XTDOMASK */
				DEBUG("XTDOMASK");
				if (dr_in_mask && (xsvf_read_buffer(xsdrsize, xsvf_fd, dr_in_mask) != ERROR_OK))
					do_abort = 1;
				break;
			case 0x02:	/* XSIR */
				DEBUG("XSIR");
				if (read(xsvf_fd, &c, 1) < 0)
					do_abort = 1;
				else
				{
					u8 *ir_buf = malloc((c + 7) / 8);
					if (xsvf_read_buffer(c, xsvf_fd, ir_buf) != ERROR_OK)
						do_abort = 1;
					else
					{
						scan_field_t field;
						field.device = device;
						field.num_bits = c;
						field.out_value = ir_buf;
						field.out_mask = NULL;
						field.in_value = NULL;
						field.in_check_value = NULL;
						field.in_check_mask = NULL;
						field.in_handler = NULL;
						field.in_handler_priv = NULL;
						if (device == -1)
							jtag_add_plain_ir_scan(1, &field, TAP_PI, NULL);
						else
							jtag_add_ir_scan(1, &field, TAP_PI, NULL);
						if (jtag_execute_queue() != ERROR_OK)
						{
							tdo_mismatch = 1;
							free(ir_buf);
							break;
						}
						if (xruntest)
						{
							if (runtest_requires_tck)
								jtag_add_runtest(xruntest, xsvf_to_tap[xendir]);
							else
							{
								jtag_add_statemove(TAP_RTI);
								jtag_add_sleep(xruntest);
								jtag_add_statemove(xsvf_to_tap[xendir]);
							}
						}
						else if (xendir != 0xd)	/* Pause-IR */
							jtag_add_statemove(xsvf_to_tap[xendir]);
					}
					free(ir_buf);
				}
				break;
			case 0x03:	/* XSDR */
				DEBUG("XSDR");
				if (xsvf_read_buffer(xsdrsize, xsvf_fd, dr_out_buf) != ERROR_OK)
					do_abort = 1;
				else
				{
					scan_field_t field;
					field.device = device;
					field.num_bits = xsdrsize;
					field.out_value = dr_out_buf;
					field.out_mask = NULL;
					field.in_value = NULL;
					field.in_check_value = dr_in_buf;
					field.in_check_mask = dr_in_mask;
					field.in_handler = NULL;
					field.in_handler_priv = NULL;
					if (device == -1)
						jtag_add_plain_dr_scan(1, &field, TAP_PD, NULL);
					else
						jtag_add_dr_scan(1, &field, TAP_PD, NULL);
					if (jtag_execute_queue() != ERROR_OK)
					{
						tdo_mismatch = 1;
						break;	
					}
					if (xruntest)
					{
						if (runtest_requires_tck)
							jtag_add_runtest(xruntest, xsvf_to_tap[xenddr]);
						else
						{
							jtag_add_statemove(TAP_RTI);
							jtag_add_sleep(xruntest);
							jtag_add_statemove(xsvf_to_tap[xenddr]);
						}
					}
					else if (xendir != 0x6)	/* Pause-DR */
						jtag_add_statemove(xsvf_to_tap[xenddr]);
				}
				break;
			case 0x04:	/* XRUNTEST */
				DEBUG("XRUNTEST");
				if (read(xsvf_fd, buf4, 4) < 0)
					do_abort = 1;
				else
				{
					xruntest = be_to_h_u32(buf4);
				}
				break;
			case 0x07:	/* XREPEAT */
				DEBUG("XREPEAT");
				if (read(xsvf_fd, &c, 1) < 0)
					do_abort = 1;
				else
				{
					xrepeat = c;
				}
				break;
			case 0x08:	/* XSDRSIZE */
				DEBUG("XSDRSIZE");
				if (read(xsvf_fd, buf4, 4) < 0)
					do_abort = 1;
				else
				{
					xsdrsize = be_to_h_u32(buf4);
					free(dr_out_buf);
					free(dr_in_buf);
					free(dr_in_mask);
					dr_out_buf = malloc((xsdrsize + 7) / 8);
					dr_in_buf = malloc((xsdrsize + 7) / 8);
					dr_in_mask = malloc((xsdrsize + 7) / 8);
				}
				break;
			case 0x09:	/* XSDRTDO */
				DEBUG("XSDRTDO");
				if (xsvf_read_buffer(xsdrsize, xsvf_fd, dr_out_buf) != ERROR_OK)
					do_abort = 1;
				else
				{
					if (xsvf_read_buffer(xsdrsize, xsvf_fd, dr_in_buf) != ERROR_OK)
						do_abort = 1;
					else
					{
						scan_field_t field;
						field.device = device;
						field.num_bits = xsdrsize;
						field.out_value = dr_out_buf;
						field.out_mask = NULL;
						field.in_value = NULL;
						field.in_check_value = dr_in_buf;
						field.in_check_mask = dr_in_mask;
						field.in_handler = NULL;
						field.in_handler_priv = NULL;
						if (device == -1)
							jtag_add_plain_dr_scan(1, &field, TAP_PD, NULL);
						else
							jtag_add_dr_scan(1, &field, TAP_PD, NULL);
						if (jtag_execute_queue() != ERROR_OK)
						{
							tdo_mismatch = 1;
							break;	
						}
						if (xruntest)
						{
							if (runtest_requires_tck)
								jtag_add_runtest(xruntest, xsvf_to_tap[xenddr]);
							else
							{
								jtag_add_statemove(TAP_RTI);
								jtag_add_sleep(xruntest);
								jtag_add_statemove(xsvf_to_tap[xenddr]);
							}
						}
						else if (xendir != 0x6)	/* Pause-DR */
							jtag_add_statemove(xsvf_to_tap[xenddr]);
					}
				}
				break;
			case 0x0a:	/* XSETDRMASKS */
				printf("unsupported XSETSDRMASKS\n");
				unsupported = 1;
				break;
			case 0x0b:	/* XSDRINC */
				printf("unsupported XSDRINC\n");
				unsupported = 1;
				break;
			case 0x0c:	/* XSDRB */
				unsupported = 1;
				break;
			case 0x0d:	/* XSDRC */
				unsupported = 1;
				break;
			case 0x0e:	/* XSDRE */
				unsupported = 1;
				break;
			case 0x0f:	/* XSDRTDOB */
				unsupported = 1;
				break;
			case 0x10:	/* XSDRTDOB */
				unsupported = 1;
				break;
			case 0x11:	/* XSDRTDOB */
				unsupported = 1;
				break;
			case 0x12:	/* XSTATE */
				DEBUG("XSTATE");
				if (read(xsvf_fd, &uc, 1) < 0)
					do_abort = 1;
				else
				{
					enum tap_state *path = calloc(XSTATE_MAX_PATH, 4);
					int path_len = 1;
					path[0] = xsvf_to_tap[uc];
					if (xsvf_read_xstates(xsvf_fd, path, XSTATE_MAX_PATH, &path_len) != ERROR_OK)
						do_abort = 1;
					else
					{
						jtag_add_pathmove(path_len, path);
					}
					free(path);
				}
				break;
			case 0x13:	/* XENDIR */
				DEBUG("XENDIR");
				if (read(xsvf_fd, &c, 1) < 0)
					do_abort = 1;
				else
				{
					if (c == 0)
						xendir = 1;
					else if (c == 1)
						xendir = 0xd;
					else
					{
						ERROR("unknown XENDIR endstate");
						unsupported = 1;
					}
				}
				break;
			case 0x14:	/* XENDDR */
				DEBUG("XENDDR");
				if (read(xsvf_fd, &c, 1) < 0)
					do_abort = 1;
				else
				{
					if (c == 0)
						xenddr = 1;
					else if (c == 1)
						xenddr = 0x6;
					else
					{
						ERROR("unknown XENDDR endstate");
						unsupported = 1;
					}
				}
				break;
			case 0x15:	/* XSIR2 */
				DEBUG("XSIR2");
				if (read(xsvf_fd, buf2, 2) < 0)
					do_abort = 1;
				else
				{
					u8 *ir_buf;
					us = be_to_h_u16(buf2);
					ir_buf = malloc((us + 7) / 8);
					if (xsvf_read_buffer(us, xsvf_fd, ir_buf) != ERROR_OK)
						do_abort = 1;
					else
					{
						scan_field_t field;
						field.device = device;
						field.num_bits = us;
						field.out_value = ir_buf;
						field.out_mask = NULL;
						field.in_value = NULL;
						field.in_check_value = NULL;
						field.in_check_mask = NULL;
						field.in_handler = NULL;
						field.in_handler_priv = NULL;
						if (device == -1)
							jtag_add_plain_ir_scan(1, &field, xsvf_to_tap[xendir], NULL);
						else
							jtag_add_ir_scan(1, &field, xsvf_to_tap[xendir], NULL);
					}
					free(ir_buf);
				}
				break;
			case 0x16:	/* XCOMMENT */
				do
				{
					if (read(xsvf_fd, &c, 1) < 0)
					{
						do_abort = 1;
						break;
					}
				} while (c != 0);
				break;
			case 0x17:	/* XWAIT */
				DEBUG("XWAIT");
				if ((read(xsvf_fd, &uc, 1) < 0) || (read(xsvf_fd, &uc2, 1) < 0) || (read(xsvf_fd, buf4, 4) < 0))
					do_abort = 1;
				else
				{
					jtag_add_statemove(xsvf_to_tap[uc]);
					ui = be_to_h_u32(buf4);
					jtag_add_sleep(ui);
					jtag_add_statemove(xsvf_to_tap[uc2]);
				}
				break;
			default:
				printf("unknown xsvf command (0x%2.2x)\n", c);
				unsupported = 1;
		}

		if (do_abort || unsupported || tdo_mismatch)
			break;
	}
	
	if (tdo_mismatch)
	{
		command_print(cmd_ctx, "TDO mismatch, aborting");
		jtag_cancel_queue();
		return ERROR_OK;
	}

	if (unsupported)
	{
		command_print(cmd_ctx, "unsupported xsvf command encountered, aborting");
		jtag_cancel_queue();
		return ERROR_OK;
	}

	if (do_abort)
	{
		command_print(cmd_ctx, "premature end detected, aborting");
		jtag_cancel_queue();
		return ERROR_OK;
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
