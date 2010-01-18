/***************************************************************************
 *   Copyright (C) 2009 by Simon Qian <SimonQian@SimonQian.com>            *
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

/* Versaloon is a programming tool for multiple MCUs.
 * OpenOCD and MSP430 supports are distributed under GPLv2.
 * You can find it at http://www.SimonQian.com/en/Versaloon.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/commands.h>
#include "usb_common.h"

//#define _VSLLINK_IN_DEBUG_MODE_

#define VSLLINK_MODE_NORMAL			0
#define VSLLINK_MODE_DMA			1

static uint16_t vsllink_usb_vid;
static uint16_t vsllink_usb_pid;
static uint8_t vsllink_usb_bulkout;
static uint8_t vsllink_usb_bulkin;
static uint8_t vsllink_usb_interface;
static uint8_t vsllink_mode = VSLLINK_MODE_NORMAL;
static int VSLLINK_USB_TIMEOUT = 10000;

static int VSLLINK_BufferSize = 1024;

/* Global USB buffers */
static int vsllink_usb_out_buffer_idx;
static int vsllink_usb_in_want_length;
static uint8_t* vsllink_usb_in_buffer = NULL;
static uint8_t* vsllink_usb_out_buffer = NULL;

/* Constants for VSLLink command */
#define VSLLINK_CMD_CONN			0x80
#define VSLLINK_CMD_DISCONN			0x81
#define VSLLINK_CMD_SET_SPEED		0x82
#define VSLLINK_CMD_SET_PORT		0x90
#define VSLLINK_CMD_GET_PORT		0x91
#define VSLLINK_CMD_SET_PORTDIR		0x92
#define VSLLINK_CMD_HW_JTAGSEQCMD	0xA0
#define VSLLINK_CMD_HW_JTAGHLCMD	0xA1
#define VSLLINK_CMD_HW_SWDCMD		0xA2
#define VSLLINK_CMD_HW_JTAGRAWCMD	0xA3

#define VSLLINK_CMDJTAGSEQ_TMSBYTE	0x00
#define VSLLINK_CMDJTAGSEQ_TMSCLOCK	0x40
#define VSLLINK_CMDJTAGSEQ_SCAN		0x80

#define VSLLINK_CMDJTAGSEQ_CMDMSK	0xC0
#define VSLLINK_CMDJTAGSEQ_LENMSK	0x3F

#define JTAG_PINMSK_SRST			(1 << 0)
#define JTAG_PINMSK_TRST			(1 << 1)
#define JTAG_PINMSK_USR1			(1 << 2)
#define JTAG_PINMSK_USR2			(1 << 3)
#define JTAG_PINMSK_TCK				(1 << 4)
#define JTAG_PINMSK_TMS				(1 << 5)
#define JTAG_PINMSK_TDI				(1 << 6)
#define JTAG_PINMSK_TDO				(1 << 7)


#define VSLLINK_TAP_MOVE(from, to)	VSLLINK_tap_move[tap_move_ndx(from)][tap_move_ndx(to)]

/* VSLLINK_tap_move[i][j]: tap movement command to go from state i to state j
 * 0: Test-Logic-Reset
 * 1: Run-Test/Idle
 * 2: Shift-DR
 * 3: Pause-DR
 * 4: Shift-IR
 * 5: Pause-IR
 *
 * SD->SD and SI->SI have to be caught in interface specific code
 */
static uint8_t VSLLINK_tap_move[6][6] =
{
/*	  TLR   RTI   SD    PD    SI    PI             */
	{0xff, 0x7f, 0x2f, 0x0a, 0x37, 0x16},	/* TLR */
	{0xff, 0x00, 0x45, 0x05, 0x4b, 0x0b},	/* RTI */
	{0xff, 0x61, 0x00, 0x01, 0x0f, 0x2f},	/* SD  */
	{0xfe, 0x60, 0x40, 0x5c, 0x3c, 0x5e},	/* PD  */
	{0xff, 0x61, 0x07, 0x17, 0x00, 0x01},	/* SI  */
	{0xfe, 0x60, 0x38, 0x5c, 0x40, 0x5e}	/* PI  */
};

struct insert_insignificant_operation {
	unsigned char insert_value;
	unsigned char insert_position;
};

static struct insert_insignificant_operation VSLLINK_TAP_MOVE_INSERT_INSIGNIFICANT[6][6] =
{
/*	 stuff	offset   */
	{/*	TLR	*/
	{1,		0,},	/* TLR */
	{1,		0,},	/* RTI */
	{1,		0,},	/* SD  */
	{1,		0,},	/* PD  */
	{1,		0,},	/* SI  */
	{1,		0,}},	/* PI  */
	{/*	RTI	*/
	{1,		0,},	/* TLR */
	{0,		0,},	/* RTI */
	{0,		4,},	/* SD  */
	{0,		7,},	/* PD  */
	{0,		5,},	/* SI  */
	{0,		7,}},	/* PI  */
	{/*	SD	*/
	{0,		0,},	/* TLR */
	{0,		0,},	/* RTI */
	{0,		0,},	/* SD  */
	{0,		0,},	/* PD  */
	{0,		0,},	/* SI  */
	{0,		0,}},	/* PI  */
	{/*	PD	*/
	{0,		0,},	/* TLR */
	{0,		0,},	/* RTI */
	{0,		0,},	/* SD  */
	{0,		0,},	/* PD  */
	{0,		0,},	/* SI  */
	{0,		0,}},	/* PI  */
	{/*	SI	*/
	{0,		0,},	/* TLR */
	{0,		0,},	/* RTI */
	{0,		0,},	/* SD  */
	{0,		0,},	/* PD  */
	{0,		0,},	/* SI  */
	{0,		0,}},	/* PI  */
	{/*	PI	*/
	{0,		0,},	/* TLR */
	{0,		0,},	/* RTI */
	{0,		0,},	/* SD  */
	{0,		0,},	/* PD  */
	{0,		0,},	/* SI  */
	{0,		0,}},	/* PI  */
};

static uint8_t VSLLINK_BIT_MSK[8] =
{
	0x00, 0x01, 0x03, 0x07, 0x0f, 0x1f, 0x3f, 0x7f
};

struct pending_scan_result {
	int offset;
	int length; /* Number of bits to read */
	struct scan_command *command; /* Corresponding scan command */
	uint8_t *buffer;
};

#define MAX_PENDING_SCAN_RESULTS 256

static int pending_scan_results_length;
static struct pending_scan_result pending_scan_results_buffer[MAX_PENDING_SCAN_RESULTS];

/* Queue command functions */
static void vsllink_end_state(tap_state_t state);
static void vsllink_state_move_dma(void);
static void vsllink_state_move_normal(void);
static void (*vsllink_state_move)(void);
static void vsllink_path_move_dma(int num_states, tap_state_t *path);
static void vsllink_path_move_normal(int num_states, tap_state_t *path);
static void (*vsllink_path_move)(int num_states, tap_state_t *path);
static void vsllink_runtest(int num_cycles);
static void vsllink_stableclocks_dma(int num_cycles, int tms);
static void vsllink_stableclocks_normal(int num_cycles, int tms);
static void (*vsllink_stableclocks)(int num_cycles, int tms);
static void vsllink_scan_dma(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size, struct scan_command *command);
static void vsllink_scan_normal(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size, struct scan_command *command);
static void (*vsllink_scan)(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size, struct scan_command *command);
static void vsllink_reset(int trst, int srst);
static void vsllink_simple_command(uint8_t command);
static int vsllink_connect(void);
static int vsllink_disconnect(void);

/* VSLLink tap buffer functions */
static void vsllink_tap_append_step(int tms, int tdi);
static void vsllink_tap_init_dma(void);
static void vsllink_tap_init_normal(void);
static void (*vsllink_tap_init)(void);
static int vsllink_tap_execute_dma(void);
static int vsllink_tap_execute_normal(void);
static int (*vsllink_tap_execute)(void);
static void vsllink_tap_ensure_space_dma(int scans, int length);
static void vsllink_tap_ensure_space_normal(int scans, int length);
static void (*vsllink_tap_ensure_space)(int scans, int length);
static void vsllink_tap_append_scan_dma(int length, uint8_t *buffer, struct scan_command *command);
static void vsllink_tap_append_scan_normal(int length, uint8_t *buffer, struct scan_command *command, int offset);

/* VSLLink lowlevel functions */
struct vsllink {
	struct usb_dev_handle* usb_handle;
};

static struct vsllink *vsllink_usb_open(void);
static void vsllink_usb_close(struct vsllink *vsllink);
static int vsllink_usb_message(struct vsllink *vsllink, int out_length, int in_length);
static int vsllink_usb_write(struct vsllink *vsllink, int out_length);
static int vsllink_usb_read(struct vsllink *vsllink);

#if defined _DEBUG_USB_COMMS_ || defined _DEBUG_JTAG_IO_
static void vsllink_debug_buffer(uint8_t *buffer, int length);
#endif

static int vsllink_tms_data_len = 0;
static uint8_t* vsllink_tms_cmd_pos;

static int tap_length = 0;
static int tap_buffer_size = 0;
static uint8_t *tms_buffer = NULL;
static uint8_t *tdi_buffer = NULL;
static uint8_t *tdo_buffer = NULL;
static int last_tms;

static struct vsllink* vsllink_handle = NULL;

static void reset_command_pointer(void)
{
	if (vsllink_mode == VSLLINK_MODE_NORMAL)
	{
		vsllink_usb_out_buffer[0] = VSLLINK_CMD_HW_JTAGSEQCMD;
		vsllink_usb_out_buffer_idx = 3;
	}
	else
	{
		tap_length = 0;
	}
}

static int vsllink_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;

	DEBUG_JTAG_IO("--------------------------------- vsllink -------------------------------------");

	reset_command_pointer();
	while (cmd != NULL)
	{
		switch (cmd->type)
		{
			case JTAG_RUNTEST:
				DEBUG_JTAG_IO("runtest %i cycles, end in %s", cmd->cmd.runtest->num_cycles, \
					tap_state_name(cmd->cmd.runtest->end_state));

				vsllink_end_state(cmd->cmd.runtest->end_state);
				vsllink_runtest(cmd->cmd.runtest->num_cycles);
				break;

			case JTAG_STATEMOVE:
				DEBUG_JTAG_IO("statemove end in %s", tap_state_name(cmd->cmd.statemove->end_state));

				vsllink_end_state(cmd->cmd.statemove->end_state);
				vsllink_state_move();
				break;

			case JTAG_PATHMOVE:
				DEBUG_JTAG_IO("pathmove: %i states, end in %s", \
					cmd->cmd.pathmove->num_states, \
					tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]));

				vsllink_path_move(cmd->cmd.pathmove->num_states, cmd->cmd.pathmove->path);
				break;

			case JTAG_SCAN:
				vsllink_end_state(cmd->cmd.scan->end_state);

				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				if (cmd->cmd.scan->ir_scan)
				{
					DEBUG_JTAG_IO("JTAG Scan write IR(%d bits), end in %s:", scan_size, tap_state_name(cmd->cmd.scan->end_state));
				}
				else
				{
					DEBUG_JTAG_IO("JTAG Scan write DR(%d bits), end in %s:", scan_size, tap_state_name(cmd->cmd.scan->end_state));
				}

#ifdef _DEBUG_JTAG_IO_
				vsllink_debug_buffer(buffer, (scan_size + 7) >> 3);
#endif

				type = jtag_scan_type(cmd->cmd.scan);

				vsllink_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size, cmd->cmd.scan);
				break;

			case JTAG_RESET:
				DEBUG_JTAG_IO("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);

				vsllink_tap_execute();

				if (cmd->cmd.reset->trst == 1)
				{
					tap_set_state(TAP_RESET);
				}
				vsllink_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;

			case JTAG_SLEEP:
				DEBUG_JTAG_IO("sleep %i", cmd->cmd.sleep->us);
				vsllink_tap_execute();
				jtag_sleep(cmd->cmd.sleep->us);
				break;

			case JTAG_STABLECLOCKS:
				DEBUG_JTAG_IO("add %d clocks", cmd->cmd.stableclocks->num_cycles);
				switch (tap_get_state())
				{
				case TAP_RESET:
					// tms should be '1' to stay in TAP_RESET mode
					scan_size = 1;
					break;
				case TAP_DRSHIFT:
				case TAP_IDLE:
				case TAP_DRPAUSE:
				case TAP_IRSHIFT:
				case TAP_IRPAUSE:
					// in other mode, tms should be '0'
					scan_size = 0;
					break;			/* above stable states are OK */
				default:
					 LOG_ERROR("jtag_add_clocks() was called with TAP in non-stable state \"%s\"",
							 tap_state_name(tap_get_state()));
					 exit(-1);
				}
				vsllink_stableclocks(cmd->cmd.stableclocks->num_cycles, scan_size);
				break;

			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered: %d", cmd->type);
				exit(-1);
		}
		cmd = cmd->next;
	}

	return vsllink_tap_execute();
}

static int vsllink_speed(int speed)
{
	int result;

	vsllink_usb_out_buffer[0] = VSLLINK_CMD_SET_SPEED;
	vsllink_usb_out_buffer[1] = (speed >> 0) & 0xff;
	vsllink_usb_out_buffer[2] = (speed >> 8) & 0xFF;

	result = vsllink_usb_write(vsllink_handle, 3);

	if (result == 3)
	{
		return ERROR_OK;
	}
	else
	{
		LOG_ERROR("VSLLink setting speed failed (%d)", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int vsllink_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;

	return ERROR_OK;
}

static int vsllink_speed_div(int jtag_speed, int *khz)
{
	*khz = jtag_speed;

	return ERROR_OK;
}

static int vsllink_init(void)
{
	int check_cnt, to_tmp;
	int result;
	char version_str[100];

	vsllink_usb_in_buffer = malloc(VSLLINK_BufferSize);
	vsllink_usb_out_buffer = malloc(VSLLINK_BufferSize);
	if ((vsllink_usb_in_buffer == NULL) || (vsllink_usb_out_buffer == NULL))
	{
		LOG_ERROR("Not enough memory");
		exit(-1);
	}

	vsllink_handle = vsllink_usb_open();

	if (vsllink_handle == 0)
	{
		LOG_ERROR("Can't find USB JTAG Interface! Please check connection and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}
	LOG_DEBUG("vsllink found on %04X:%04X", vsllink_usb_vid, vsllink_usb_pid);

	to_tmp = VSLLINK_USB_TIMEOUT;
	VSLLINK_USB_TIMEOUT = 100;
	check_cnt = 0;
	while (check_cnt < 5)
	{
		vsllink_simple_command(0x00);
		result = vsllink_usb_read(vsllink_handle);

		if (result > 2)
		{
			vsllink_usb_in_buffer[result] = 0;
			VSLLINK_BufferSize = vsllink_usb_in_buffer[0] + (vsllink_usb_in_buffer[1] << 8);
			strncpy(version_str, (char *)vsllink_usb_in_buffer + 2, sizeof(version_str));
			LOG_INFO("%s", version_str);

			// free the pre-alloc memroy
			free(vsllink_usb_in_buffer);
			free(vsllink_usb_out_buffer);
			vsllink_usb_in_buffer = NULL;
			vsllink_usb_out_buffer = NULL;

			// alloc new memory
			vsllink_usb_in_buffer = malloc(VSLLINK_BufferSize);
			vsllink_usb_out_buffer = malloc(VSLLINK_BufferSize);
			if ((vsllink_usb_in_buffer == NULL) || (vsllink_usb_out_buffer == NULL))
			{
				LOG_ERROR("Not enough memory");
				exit(-1);
			}
			else
			{
				LOG_INFO("buffer size for USB is %d bytes", VSLLINK_BufferSize);
			}
			// alloc memory for dma mode
			if (vsllink_mode == VSLLINK_MODE_DMA)
			{
				tap_buffer_size = (VSLLINK_BufferSize - 3) / 2;
				tms_buffer = (uint8_t*)malloc(tap_buffer_size);
				tdi_buffer = (uint8_t*)malloc(tap_buffer_size);
				tdo_buffer = (uint8_t*)malloc(tap_buffer_size);
				if ((tms_buffer == NULL) || (tdi_buffer == NULL) || (tdo_buffer == NULL))
				{
					LOG_ERROR("Not enough memory");
					exit(-1);
				}
			}
			break;
		}
		vsllink_simple_command(VSLLINK_CMD_DISCONN);
		check_cnt++;
	}
	if (check_cnt == 3)
	{
		// It's dangerout to proced
		LOG_ERROR("VSLLink initial failed");
		exit(-1);
	}
	VSLLINK_USB_TIMEOUT = to_tmp;

	/* Some older firmware versions sometimes fail if the
	 * voltage isn't read first.
	 */
	vsllink_simple_command(0x01);
	result = vsllink_usb_read(vsllink_handle);
	if (result != 2)
		LOG_WARNING("Fail to get target voltage");
	else
		LOG_INFO("Target runs at %d mV", vsllink_usb_in_buffer[0]
				+ (vsllink_usb_in_buffer[1] << 8));

	// connect to vsllink
	vsllink_connect();
	// initialize function pointers
	if (vsllink_mode == VSLLINK_MODE_NORMAL)
	{
		// normal mode
		vsllink_state_move = vsllink_state_move_normal;
		vsllink_path_move = vsllink_path_move_normal;
		vsllink_stableclocks = vsllink_stableclocks_normal;
		vsllink_scan = vsllink_scan_normal;

		vsllink_tap_init = vsllink_tap_init_normal;
		vsllink_tap_execute = vsllink_tap_execute_normal;
		vsllink_tap_ensure_space = vsllink_tap_ensure_space_normal;

		LOG_INFO("vsllink run in NORMAL mode");
	}
	else
	{
		// dma mode
		vsllink_state_move = vsllink_state_move_dma;
		vsllink_path_move = vsllink_path_move_dma;
		vsllink_stableclocks = vsllink_stableclocks_dma;
		vsllink_scan = vsllink_scan_dma;

		vsllink_tap_init = vsllink_tap_init_dma;
		vsllink_tap_execute = vsllink_tap_execute_dma;
		vsllink_tap_ensure_space = vsllink_tap_ensure_space_dma;

		LOG_INFO("vsllink run in DMA mode");
	}

	// Set SRST and TRST to output, Set USR1 and USR2 to input
	vsllink_usb_out_buffer[0] = VSLLINK_CMD_SET_PORTDIR;
	vsllink_usb_out_buffer[1] = JTAG_PINMSK_SRST | JTAG_PINMSK_TRST | JTAG_PINMSK_USR1 | JTAG_PINMSK_USR2;
	vsllink_usb_out_buffer[2] = JTAG_PINMSK_SRST | JTAG_PINMSK_TRST;
	if (vsllink_usb_write(vsllink_handle, 3) != 3)
	{
		LOG_ERROR("VSLLink USB send data error");
		exit(-1);
	}

	vsllink_reset(0, 0);

	LOG_INFO("VSLLink JTAG Interface ready");

	vsllink_tap_init();

	return ERROR_OK;
}

static int vsllink_quit(void)
{
	if ((vsllink_usb_in_buffer != NULL) && (vsllink_usb_out_buffer != NULL))
	{
		// Set all pins to input
		vsllink_usb_out_buffer[0] = VSLLINK_CMD_SET_PORTDIR;
		vsllink_usb_out_buffer[1] = JTAG_PINMSK_SRST | JTAG_PINMSK_TRST | JTAG_PINMSK_USR1 | JTAG_PINMSK_USR2;
		vsllink_usb_out_buffer[2] = 0;
		if (vsllink_usb_write(vsllink_handle, 3) != 3)
		{
			LOG_ERROR("VSLLink USB send data error");
			exit(-1);
		}

		// disconnect
		vsllink_disconnect();
		vsllink_usb_close(vsllink_handle);
		vsllink_handle = NULL;
	}

	if (vsllink_usb_in_buffer != NULL)
	{
		free(vsllink_usb_in_buffer);
		vsllink_usb_in_buffer = NULL;
	}
	if (vsllink_usb_out_buffer != NULL)
	{
		free(vsllink_usb_out_buffer);
		vsllink_usb_out_buffer = NULL;
	}

	return ERROR_OK;
}

/***************************************************************************/
/* Queue command implementations */
static int vsllink_disconnect(void)
{
	vsllink_simple_command(VSLLINK_CMD_DISCONN);
	return ERROR_OK;
}

static int vsllink_connect(void)
{
	char vsllink_str[100];

	vsllink_usb_out_buffer[0] = VSLLINK_CMD_CONN;
	vsllink_usb_out_buffer[1] = vsllink_mode;
	vsllink_usb_message(vsllink_handle, 2, 0);
	if (vsllink_usb_read(vsllink_handle) > 2)
	{
		strncpy(vsllink_str, (char *)vsllink_usb_in_buffer + 2, sizeof(vsllink_str));
		LOG_INFO("%s", vsllink_str);
	}

	return ERROR_OK;
}

// when vsllink_tms_data_len > 0, vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx] is the byte that need to be appended.
// length of VSLLINK_CMDJTAGSEQ_TMSBYTE has been set, no need to set it here.
static void vsllink_append_tms(void)
{
	uint8_t tms_scan = VSLLINK_TAP_MOVE(tap_get_state(), tap_get_end_state());
	uint16_t tms2;
	struct insert_insignificant_operation *insert = \
		&VSLLINK_TAP_MOVE_INSERT_INSIGNIFICANT[tap_move_ndx(tap_get_state())][tap_move_ndx(tap_get_end_state())];

	if (((tap_get_state() != TAP_RESET) && (tap_get_state() != TAP_IDLE) && (tap_get_state() != TAP_DRPAUSE) && (tap_get_state() != TAP_IRPAUSE)) || \
			(vsllink_tms_data_len <= 0) || (vsllink_tms_data_len >= 8) || \
			(vsllink_tms_cmd_pos == NULL))
	{
		LOG_ERROR("There MUST be some bugs in the driver");
		exit(-1);
	}

	tms2 = (tms_scan & VSLLINK_BIT_MSK[insert->insert_position]) << \
				vsllink_tms_data_len;
	if (insert->insert_value == 1)
	{
		tms2 |= VSLLINK_BIT_MSK[8 - vsllink_tms_data_len] << \
				(vsllink_tms_data_len + insert->insert_position);
	}
	tms2 |= (tms_scan >> insert->insert_position) << \
				(8 + insert->insert_position);

	vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] |= (tms2 >> 0) & 0xff;
	vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = (tms2 >> 8) & 0xff;

	vsllink_tms_data_len = 0;
	vsllink_tms_cmd_pos = NULL;
}

static void vsllink_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
	{
		tap_set_end_state(state);
	}
	else
	{
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

/* Goes to the end state. */
static void vsllink_state_move_normal(void)
{
	if (vsllink_tms_data_len > 0)
	{
		vsllink_append_tms();
	}
	else
	{
		vsllink_tap_ensure_space(0, 2);

		vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = VSLLINK_CMDJTAGSEQ_TMSBYTE;
		vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = VSLLINK_TAP_MOVE(tap_get_state(), tap_get_end_state());
	}

	tap_set_state(tap_get_end_state());
}
static void vsllink_state_move_dma(void)
{
	int i, insert_length = (tap_length % 8) ? (8 - (tap_length % 8)) : 0;
	struct insert_insignificant_operation *insert = \
		&VSLLINK_TAP_MOVE_INSERT_INSIGNIFICANT[tap_move_ndx(tap_get_state())][tap_move_ndx(tap_get_end_state())];
	uint8_t tms_scan = VSLLINK_TAP_MOVE(tap_get_state(), tap_get_end_state());

	if (tap_get_state() == TAP_RESET)
	{
		vsllink_tap_ensure_space(0, 8);

		for (i = 0; i < 8; i++)
		{
			vsllink_tap_append_step(1, 0);
		}
	}

	if (insert_length > 0)
	{
		vsllink_tap_ensure_space(0, 16);

		for (i = 0; i < insert->insert_position; i++)
		{
			vsllink_tap_append_step((tms_scan >> i) & 1, 0);
		}
		for (i = 0; i < insert_length; i++)
		{
			vsllink_tap_append_step(insert->insert_value, 0);
		}
		for (i = insert->insert_position; i < 8; i++)
		{
			vsllink_tap_append_step((tms_scan >> i) & 1, 0);
		}
	}
	else
	{
		vsllink_tap_ensure_space(0, 8);

		for (i = 0; i < 8; i++)
		{
			vsllink_tap_append_step((tms_scan >> i) & 1, 0);
		}
	}

	tap_set_state(tap_get_end_state());
}

// write tms from current vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx]
static void vsllink_add_path(int start, int num, tap_state_t *path)
{
	int i;

	for (i = start; i < (start + num); i++)
	{
		if ((i & 7) == 0)
		{
			if (i > 0)
			{
				vsllink_usb_out_buffer_idx++;
			}
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx] = 0;
		}

		if (path[i - start] == tap_state_transition(tap_get_state(), true))
		{
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx] |= 1 << (i & 7);
		}
		else if (path[i - start] == tap_state_transition(tap_get_state(), false))
		{
			// nothing to do
		}
		else
		{
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition", tap_state_name(tap_get_state()), tap_state_name(path[i]));
			exit(-1);
		}
		tap_set_state(path[i - start]);
	}
	if ((i > 0) && ((i & 7) == 0))
	{
		vsllink_usb_out_buffer_idx++;
		vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx] = 0;
	}

	tap_set_end_state(tap_get_state());
}

static void vsllink_path_move_normal(int num_states, tap_state_t *path)
{
	int i, tms_len, tms_cmd_pos, path_idx = 0;

	if (vsllink_tms_data_len > 0)
	{
		// there are vsllink_tms_data_len more tms bits to be shifted
		// so there are vsllink_tms_data_len + num_states tms bits in all
		tms_len = vsllink_tms_data_len + num_states;
		if (tms_len <= 16)
		{
			// merge into last tms shift
			if (tms_len < 8)
			{
				// just append tms data to the last tms byte
				vsllink_add_path(vsllink_tms_data_len, num_states, path);
			}
			else if (tms_len == 8)
			{
				// end last tms shift command
				(*vsllink_tms_cmd_pos)--;
				vsllink_add_path(vsllink_tms_data_len, num_states, path);
			}
			else if (tms_len < 16)
			{
				if ((*vsllink_tms_cmd_pos & VSLLINK_CMDJTAGSEQ_LENMSK) < VSLLINK_CMDJTAGSEQ_LENMSK)
				{
					// every tms shift command can contain VSLLINK_CMDJTAGSEQ_LENMSK + 1 bytes in most
					// there is enought tms length in the current tms shift command
					(*vsllink_tms_cmd_pos)++;
					vsllink_add_path(vsllink_tms_data_len, num_states, path);
				}
				else
				{
					// every tms shift command can contain VSLLINK_CMDJTAGSEQ_LENMSK + 1 bytes in most
					// not enough tms length in the current tms shift command
					// so a new command should be added
					// first decrease byte length of last tms shift command
					(*vsllink_tms_cmd_pos)--;
					// append tms data to the last tms byte
					vsllink_add_path(vsllink_tms_data_len, 8 - vsllink_tms_data_len, path);
					path += 8 - vsllink_tms_data_len;
					// add new command(3 bytes)
					vsllink_tap_ensure_space(0, 3);
					vsllink_tms_cmd_pos = &vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx];
					vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = VSLLINK_CMDJTAGSEQ_TMSBYTE | 1;
					vsllink_add_path(0, num_states - (8 - vsllink_tms_data_len), path);
				}
			}
			else if (tms_len == 16)
			{
				// end last tms shift command
				vsllink_add_path(vsllink_tms_data_len, num_states, path);
			}

			vsllink_tms_data_len = (vsllink_tms_data_len + num_states) & 7;
			if (vsllink_tms_data_len == 0)
			{
				vsllink_tms_cmd_pos = NULL;
			}
			num_states = 0;
		}
		else
		{
			vsllink_add_path(vsllink_tms_data_len, 16 - vsllink_tms_data_len, path);

			path += 16 - vsllink_tms_data_len;
			num_states -= 16 - vsllink_tms_data_len;
			vsllink_tms_data_len = 0;
			vsllink_tms_cmd_pos = NULL;
		}
	}

	if (num_states > 0)
	{
		// Normal operation, don't need to append tms data
		vsllink_tms_data_len = num_states & 7;

		while (num_states > 0)
		{
			if (num_states > ((VSLLINK_CMDJTAGSEQ_LENMSK + 1) * 8))
			{
				i = (VSLLINK_CMDJTAGSEQ_LENMSK + 1) * 8;
			}
			else
			{
				i = num_states;
			}
			tms_len = (i + 7) >> 3;
			vsllink_tap_ensure_space(0, tms_len + 2);
			tms_cmd_pos = vsllink_usb_out_buffer_idx;
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = VSLLINK_CMDJTAGSEQ_TMSBYTE | (tms_len - 1);

			vsllink_add_path(0, i, path + path_idx);

			path_idx += i;
			num_states -= i;
		}

		if (vsllink_tms_data_len > 0)
		{
			if (tms_len < (VSLLINK_CMDJTAGSEQ_LENMSK + 1))
			{
				vsllink_tms_cmd_pos = &vsllink_usb_out_buffer[tms_cmd_pos];
				(*vsllink_tms_cmd_pos)++;
			}
			else
			{
				vsllink_usb_out_buffer[tms_cmd_pos]--;

				tms_len = vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx];
				vsllink_tap_ensure_space(0, 3);
				vsllink_tms_cmd_pos = &vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx];
				vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = VSLLINK_CMDJTAGSEQ_TMSBYTE | 1;
				vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx] = tms_len;
			}
		}
	}
}
static void vsllink_path_move_dma(int num_states, tap_state_t *path)
{
	int i, j = 0;

	if (tap_length & 7)
	{
		if ((8 - (tap_length & 7)) < num_states)
		{
			j = 8 - (tap_length & 7);
		}
		else
		{
			j = num_states;
		}
		for (i = 0; i < j; i++)
		{
			if (path[i] == tap_state_transition(tap_get_state(), false))
			{
				vsllink_tap_append_step(0, 0);
			}
			else if (path[i] == tap_state_transition(tap_get_state(), true))
			{
				vsllink_tap_append_step(1, 0);
			}
			else
			{
				LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition", tap_state_name(tap_get_state()), tap_state_name(path[i]));
				exit(-1);
			}
			tap_set_state(path[i]);
		}
		num_states -= j;
	}

	if (num_states > 0)
	{
		vsllink_tap_ensure_space(0, num_states);

		for (i = 0; i < num_states; i++)
		{
			if (path[j + i] == tap_state_transition(tap_get_state(), false))
			{
				vsllink_tap_append_step(0, 0);
			}
			else if (path[j + i] == tap_state_transition(tap_get_state(), true))
			{
				vsllink_tap_append_step(1, 0);
			}
			else
			{
				LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition", tap_state_name(tap_get_state()), tap_state_name(path[i]));
				exit(-1);
			}
			tap_set_state(path[j + i]);
		}
	}

	tap_set_end_state(tap_get_state());
}

static void vsllink_stableclocks_normal(int num_cycles, int tms)
{
	int tms_len;
	uint16_t tms_append_byte;

	if (vsllink_tms_data_len > 0)
	{
		// there are vsllink_tms_data_len more tms bits to be shifted
		// so there are vsllink_tms_data_len + num_cycles tms bits in all
		tms_len = vsllink_tms_data_len + num_cycles;
		if (tms > 0)
		{
			// append '1' for tms
			tms_append_byte = (uint16_t)((((1 << num_cycles) - 1) << vsllink_tms_data_len) & 0xFFFF);
		}
		else
		{
			// append '0' for tms
			tms_append_byte = 0;
		}
		if (tms_len <= 16)
		{
			// merge into last tms shift
			if (tms_len < 8)
			{
				// just add to vsllink_tms_data_len
				// same result if tun through
				//vsllink_tms_data_len += num_cycles;
				vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx] |= (uint8_t)(tms_append_byte & 0xFF);
			}
			else if (tms_len == 8)
			{
				// end last tms shift command
				// just reduce it, and append last tms byte
				(*vsllink_tms_cmd_pos)--;
				vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] |= (uint8_t)(tms_append_byte & 0xFF);
			}
			else if (tms_len < 16)
			{
				if ((*vsllink_tms_cmd_pos & VSLLINK_CMDJTAGSEQ_LENMSK) < VSLLINK_CMDJTAGSEQ_LENMSK)
				{
					// every tms shift command can contain VSLLINK_CMDJTAGSEQ_LENMSK + 1 bytes in most
					// there is enought tms length in the current tms shift command
					// increase the tms byte length by 1 and set the last byte to 0
					(*vsllink_tms_cmd_pos)++;
					vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] |= (uint8_t)(tms_append_byte & 0xFF);
					vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx] = (uint8_t)(tms_append_byte >> 8);
				}
				else
				{
					// every tms shift command can contain VSLLINK_CMDJTAGSEQ_LENMSK + 1 bytes in most
					// not enough tms length in the current tms shift command
					// so a new command should be added
					// first decrease byte length of last tms shift command
					(*vsllink_tms_cmd_pos)--;
					// append last tms byte and move the command pointer to the next empty position
					vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] |= (uint8_t)(tms_append_byte & 0xFF);
					// add new command(3 bytes)
					vsllink_tap_ensure_space(0, 3);
					vsllink_tms_cmd_pos = &vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx];
					vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = VSLLINK_CMDJTAGSEQ_TMSBYTE | 1;
					vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx] = (uint8_t)(tms_append_byte >> 8);
				}
			}
			else if (tms_len == 16)
			{
				// end last tms shift command
				vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] |= (uint8_t)(tms_append_byte & 0xFF);
				vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = (uint8_t)(tms_append_byte >> 8);
			}

			vsllink_tms_data_len = tms_len & 7;
			if (vsllink_tms_data_len == 0)
			{
				vsllink_tms_cmd_pos = NULL;
			}
			num_cycles = 0;
		}
		else
		{
			// more shifts will be needed
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] |= (uint8_t)(tms_append_byte & 0xFF);
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = (uint8_t)(tms_append_byte >> 8);

			num_cycles -= 16 - vsllink_tms_data_len;
			vsllink_tms_data_len = 0;
			vsllink_tms_cmd_pos = NULL;
		}
	}
	// from here vsllink_tms_data_len == 0 or num_cycles == 0

	if (vsllink_tms_data_len > 0)
	{
		// num_cycles == 0
		// no need to shift
		if (num_cycles > 0)
		{
			LOG_ERROR("There MUST be some bugs in the driver");
			exit(-1);
		}
	}
	else
	{
		// get number of bytes left to be sent
		tms_len = num_cycles >> 3;
		if (tms_len > 0)
		{
			vsllink_tap_ensure_space(1, 5);
			// if tms_len > 0, vsllink_tms_data_len == 0
			// so just add new command
			// LSB of the command byte is the tms value when do the shifting
			if (tms > 0)
			{
				vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = VSLLINK_CMDJTAGSEQ_TMSCLOCK | 1;
			}
			else
			{
				vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = VSLLINK_CMDJTAGSEQ_TMSCLOCK;
			}
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = (tms_len >> 0) & 0xff;
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = (tms_len >> 8) & 0xff;
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = (tms_len >> 16) & 0xff;
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = (tms_len >> 24) & 0xff;

			vsllink_usb_in_want_length += 1;
			pending_scan_results_buffer[pending_scan_results_length].buffer = NULL;
			pending_scan_results_length++;

			if (tms_len > 0xFFFF)
			{
				vsllink_tap_execute();
			}
		}

		// post-process
		vsllink_tms_data_len = num_cycles & 7;
		if (vsllink_tms_data_len > 0)
		{
			vsllink_tap_ensure_space(0, 3);
			vsllink_tms_cmd_pos = &vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx];
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = VSLLINK_CMDJTAGSEQ_TMSBYTE | 1;
			if (tms > 0)
			{
				// append '1' for tms
				vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx] = (1 << vsllink_tms_data_len) - 1;
			}
			else
			{
				// append '0' for tms
				vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx] = 0x00;
			}
		}
	}
}
static void vsllink_stableclocks_dma(int num_cycles, int tms)
{
	int i, cur_cycles;

	if (tap_length & 7)
	{
		if ((8 - (tap_length & 7)) < num_cycles)
		{
			cur_cycles = 8 - (tap_length & 7);
		}
		else
		{
			cur_cycles = num_cycles;
		}
		for (i = 0; i < cur_cycles; i++)
		{
			vsllink_tap_append_step(tms, 0);
		}
		num_cycles -= cur_cycles;
	}

	while (num_cycles > 0)
	{
		if (num_cycles > 8 * tap_buffer_size)
		{
			cur_cycles = 8 * tap_buffer_size;
		}
		else
		{
			cur_cycles = num_cycles;
		}

		vsllink_tap_ensure_space(0, cur_cycles);

		for (i = 0; i < cur_cycles; i++)
		{
			vsllink_tap_append_step(tms, 0);
		}

		num_cycles -= cur_cycles;
	}
}

static void vsllink_runtest(int num_cycles)
{
	tap_state_t saved_end_state = tap_get_end_state();

	if (tap_get_state() != TAP_IDLE)
	{
		// enter into IDLE state
		vsllink_end_state(TAP_IDLE);
		vsllink_state_move();
	}

	vsllink_stableclocks(num_cycles, 0);

	// post-process
	// set end_state
	vsllink_end_state(saved_end_state);
	tap_set_state(TAP_IDLE);
	if (tap_get_end_state() != TAP_IDLE)
	{
		vsllink_state_move();
	}
}

static void vsllink_scan_normal(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size, struct scan_command *command)
{
	tap_state_t saved_end_state;
	uint8_t bits_left, tms_tmp, tdi_len;
	int i;

	if (0 == scan_size)
	{
		return;
	}

	tdi_len = ((scan_size + 7) >> 3);
	if ((tdi_len + 7) > VSLLINK_BufferSize)
	{
		LOG_ERROR("Your implementation of VSLLink has not enough buffer");
		exit(-1);
	}

	saved_end_state = tap_get_end_state();

	/* Move to appropriate scan state */
	vsllink_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

	if (vsllink_tms_data_len > 0)
	{
		if (tap_get_state() == tap_get_end_state())
		{
			// already in IRSHIFT or DRSHIFT state
			// merge tms data in the last tms shift command into next scan command
			if (*vsllink_tms_cmd_pos < 1)
			{
				LOG_ERROR("There MUST be some bugs in the driver");
				exit(-1);
			}
			else if (*vsllink_tms_cmd_pos < 2)
			{
				tms_tmp = vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx];
				vsllink_usb_out_buffer_idx--;
			}
			else
			{
				tms_tmp = vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx];
				*vsllink_tms_cmd_pos -= 2;
			}

			vsllink_tap_ensure_space(1, tdi_len + 7);
			// VSLLINK_CMDJTAGSEQ_SCAN ored by 1 means that tms_before is valid
			// which is merged from the last tms shift command
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = VSLLINK_CMDJTAGSEQ_SCAN | 1;
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = ((tdi_len + 1) >> 0) & 0xff;
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = ((tdi_len + 1) >> 8) & 0xff;
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = tms_tmp;
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = buffer[0] << (8 - vsllink_tms_data_len);

			for (i = 0; i < tdi_len; i++)
			{
				buffer[i] >>= 8 - vsllink_tms_data_len;
				if (i != tdi_len)
				{
					buffer[i] += buffer[i + 1] << vsllink_tms_data_len;
				}
			}

			vsllink_tap_append_scan_normal(scan_size - vsllink_tms_data_len, buffer, command, vsllink_tms_data_len);
			scan_size -= 8 - vsllink_tms_data_len;
			vsllink_tms_data_len = 0;
		}
		else
		{
			vsllink_state_move();
			vsllink_tap_ensure_space(1, tdi_len + 5);

			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = VSLLINK_CMDJTAGSEQ_SCAN;
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = (tdi_len >> 0) & 0xff;
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = (tdi_len >> 8) & 0xff;

			vsllink_tap_append_scan_normal(scan_size, buffer, command, 0);
		}
	}
	else
	{
		vsllink_tap_ensure_space(1, tdi_len + 7);

		vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = VSLLINK_CMDJTAGSEQ_SCAN | 1;
		vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = ((tdi_len + 1) >> 0) & 0xff;
		vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = ((tdi_len + 1)>> 8) & 0xff;
		vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = VSLLINK_TAP_MOVE(tap_get_state(), tap_get_end_state());
		vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = 0;

		vsllink_tap_append_scan_normal(scan_size, buffer, command, 8);
	}
	vsllink_end_state(saved_end_state);

	bits_left = scan_size & 0x07;
	tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (bits_left > 0)
	{
		vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = 1 << (bits_left - 1);
	}
	else
	{
		vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = 1 << 7;
	}

	if (tap_get_state() != tap_get_end_state())
	{
		vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = VSLLINK_TAP_MOVE(tap_get_state(), tap_get_end_state());
	}
	else
	{
		vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = 0;
	}

	tap_set_state(tap_get_end_state());
}
static void vsllink_scan_dma(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size, struct scan_command *command)
{
	tap_state_t saved_end_state;

	saved_end_state = tap_get_end_state();

	/* Move to appropriate scan state */
	vsllink_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

	vsllink_state_move();
	vsllink_end_state(saved_end_state);

	/* Scan */
	vsllink_tap_append_scan_dma(scan_size, buffer, command);

	tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);
	while (tap_length % 8 != 0)
	{
		// more 0s in Pause
		vsllink_tap_append_step(0, 0);
	}

	if (tap_get_state() != tap_get_end_state())
	{
		vsllink_state_move();
	}
}

static void vsllink_reset(int trst, int srst)
{
	int result;

	LOG_DEBUG("trst: %i, srst: %i", trst, srst);

	/* Signals are active low */
	vsllink_usb_out_buffer[0] = VSLLINK_CMD_SET_PORT;
	vsllink_usb_out_buffer[1] = JTAG_PINMSK_SRST | JTAG_PINMSK_TRST;
	vsllink_usb_out_buffer[2] = 0;
	if (srst == 0)
	{
		vsllink_usb_out_buffer[2] |= JTAG_PINMSK_SRST;
	}
	if (trst == 0)
	{
		vsllink_usb_out_buffer[2] |= JTAG_PINMSK_TRST;
	}

	result = vsllink_usb_write(vsllink_handle, 3);
	if (result != 3)
	{
		LOG_ERROR("VSLLink command VSLLINK_CMD_SET_PORT failed (%d)", result);
	}
}

static void vsllink_simple_command(uint8_t command)
{
	int result;

	DEBUG_JTAG_IO("0x%02x", command);

	vsllink_usb_out_buffer[0] = command;
	result = vsllink_usb_write(vsllink_handle, 1);

	if (result != 1)
	{
		LOG_ERROR("VSLLink command 0x%02x failed (%d)", command, result);
	}
}

COMMAND_HANDLER(vsllink_handle_mode_command)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("parameter error, should be one parameter for VID");
		return ERROR_FAIL;
	}

	if (!strcmp(CMD_ARGV[0], "normal"))
	{
		vsllink_mode = VSLLINK_MODE_NORMAL;
	}
	else if (!strcmp(CMD_ARGV[0], "dma"))
	{
		vsllink_mode = VSLLINK_MODE_DMA;
	}
	else
	{
		LOG_ERROR("invalid vsllink_mode: %s", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_vid_command)
{
	if (CMD_ARGC != 1)
	{
		LOG_ERROR("parameter error, should be one parameter for VID");
		return ERROR_OK;
	}

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], vsllink_usb_vid);
	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_pid_command)
{
	if (CMD_ARGC != 1)
	{
		LOG_ERROR("parameter error, should be one parameter for PID");
		return ERROR_OK;
	}
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], vsllink_usb_pid);
	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_bulkin_command)
{
	if (CMD_ARGC != 1)
	{
		LOG_ERROR("parameter error, should be one parameter for BULKIN endpoint");
		return ERROR_OK;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], vsllink_usb_bulkin);

	vsllink_usb_bulkin |= 0x80;

	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_bulkout_command)
{
	if (CMD_ARGC != 1)
	{
		LOG_ERROR("parameter error, should be one parameter for BULKOUT endpoint");
		return ERROR_OK;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], vsllink_usb_bulkout);

	vsllink_usb_bulkout &= ~0x80;

	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_interface_command)
{
	if (CMD_ARGC != 1)
	{
		LOG_ERROR("parameter error, should be one parameter for interface number");
		return ERROR_OK;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], vsllink_usb_interface);
	return ERROR_OK;
}

/***************************************************************************/
/* VSLLink tap functions */

static void vsllink_tap_init_normal(void)
{
	vsllink_usb_out_buffer_idx = 0;
	vsllink_usb_in_want_length = 0;
	pending_scan_results_length = 0;
}
static void vsllink_tap_init_dma(void)
{
	tap_length = 0;
	pending_scan_results_length = 0;
}

static void vsllink_tap_ensure_space_normal(int scans, int length)
{
	int available_scans = MAX_PENDING_SCAN_RESULTS - pending_scan_results_length;
	int available_bytes = VSLLINK_BufferSize - vsllink_usb_out_buffer_idx;

	if (scans > available_scans || length > available_bytes)
	{
		vsllink_tap_execute();
	}
}
static void vsllink_tap_ensure_space_dma(int scans, int length)
{
	int available_scans = MAX_PENDING_SCAN_RESULTS - pending_scan_results_length;
	int available_bytes = tap_buffer_size * 8 - tap_length;

	if (scans > available_scans || length > available_bytes)
	{
		vsllink_tap_execute();
	}
}

static void vsllink_tap_append_step(int tms, int tdi)
{
	last_tms = tms;
	int index = tap_length / 8;

	if (index < tap_buffer_size)
	{
		int bit_index = tap_length % 8;
		uint8_t bit = 1 << bit_index;

		if (tms)
		{
			tms_buffer[index] |= bit;
		}
		else
		{
			tms_buffer[index] &= ~bit;
		}

		if (tdi)
		{
			tdi_buffer[index] |= bit;
		}
		else
		{
			tdi_buffer[index] &= ~bit;
		}

		tap_length++;
	}
	else
	{
		LOG_ERROR("buffer overflow, tap_length=%d", tap_length);
	}
}

static void vsllink_tap_append_scan_normal(int length, uint8_t *buffer, struct scan_command *command, int offset)
{
	struct pending_scan_result *pending_scan_result = &pending_scan_results_buffer[pending_scan_results_length];
	int i;

	if (offset > 0)
	{
		vsllink_usb_in_want_length += ((length + 7) >> 3) + 1;
	}
	else
	{
		vsllink_usb_in_want_length += (length + 7) >> 3;
	}
	pending_scan_result->length = length;
	pending_scan_result->offset = offset;
	pending_scan_result->command = command;
	pending_scan_result->buffer = buffer;

	for (i = 0; i < ((length + 7) >> 3); i++)
	{
		vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = buffer[i];
	}

	pending_scan_results_length++;
}
static void vsllink_tap_append_scan_dma(int length, uint8_t *buffer, struct scan_command *command)
{
	struct pending_scan_result *pending_scan_result;
	int len_tmp, len_all, i;

	len_all = 0;
	while (len_all < length)
	{
		if ((length - len_all) > tap_buffer_size * 8)
		{
			len_tmp = tap_buffer_size * 8;
		}
		else
		{
			len_tmp = length - len_all;
		}

		vsllink_tap_ensure_space(1, (len_tmp + 7) & ~7);

		pending_scan_result = &pending_scan_results_buffer[pending_scan_results_length];
		pending_scan_result->offset = tap_length;
		pending_scan_result->length = len_tmp;
		pending_scan_result->command = command;
		pending_scan_result->buffer = buffer + len_all / 8;

		for (i = 0; i < len_tmp; i++)
		{
			vsllink_tap_append_step(((len_all + i) < length-1 ? 0 : 1), (buffer[(len_all + i)/8] >> ((len_all + i)%8)) & 1);
		}

		pending_scan_results_length++;
		len_all += len_tmp;
	}
}

/* Pad and send a tap sequence to the device, and receive the answer.
 * For the purpose of padding we assume that we are in reset or idle or pause state. */
static int vsllink_tap_execute_normal(void)
{
	int i;
	int result;
	int first = 0;

	if (vsllink_tms_data_len > 0)
	{
		if ((tap_get_state() != TAP_RESET) && (tap_get_state() != TAP_IDLE) && (tap_get_state() != TAP_IRPAUSE) && (tap_get_state() != TAP_DRPAUSE))
		{
			LOG_WARNING("%s is not in RESET or IDLE or PAUSR state", tap_state_name(tap_get_state()));
		}

		if (vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx] & (1 << (vsllink_tms_data_len - 1)))
		{
			// last tms bit is '1'
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] |= 0xFF << vsllink_tms_data_len;
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = 0xFF;
			vsllink_tms_data_len = 0;
		}
		else
		{
			// last tms bit is '0'
			vsllink_usb_out_buffer_idx++;
			vsllink_usb_out_buffer[vsllink_usb_out_buffer_idx++] = 0;
			vsllink_tms_data_len = 0;
		}
	}

	if (vsllink_usb_out_buffer_idx > 3)
	{
		if (vsllink_usb_out_buffer[0] == VSLLINK_CMD_HW_JTAGSEQCMD)
		{
			vsllink_usb_out_buffer[1] = (vsllink_usb_out_buffer_idx >> 0) & 0xff;
			vsllink_usb_out_buffer[2] = (vsllink_usb_out_buffer_idx >> 8) & 0xff;
		}

		result = vsllink_usb_message(vsllink_handle, vsllink_usb_out_buffer_idx, vsllink_usb_in_want_length);

		if (result == vsllink_usb_in_want_length)
		{
			for (i = 0; i < pending_scan_results_length; i++)
			{
				struct pending_scan_result *pending_scan_result = &pending_scan_results_buffer[i];
				uint8_t *buffer = pending_scan_result->buffer;
				int length = pending_scan_result->length;
				int offset = pending_scan_result->offset;
				struct scan_command *command = pending_scan_result->command;

				if (buffer != NULL)
				{
					// IRSHIFT or DRSHIFT
					buf_set_buf(vsllink_usb_in_buffer, first * 8 + offset, buffer, 0, length);
					first += (length + offset + 7) >> 3;

					DEBUG_JTAG_IO("JTAG scan read(%d bits):", length);
#ifdef _DEBUG_JTAG_IO_
					vsllink_debug_buffer(buffer, (length + 7) >> 3);
#endif

					if (jtag_read_buffer(buffer, command) != ERROR_OK)
					{
						vsllink_tap_init();
						return ERROR_JTAG_QUEUE_FAILED;
					}

					free(pending_scan_result->buffer);
					pending_scan_result->buffer = NULL;
				}
				else
				{
					first++;
				}
			}
		}
		else
		{
			LOG_ERROR("vsllink_tap_execute, wrong result %d, expected %d", result, vsllink_usb_in_want_length);
			return ERROR_JTAG_QUEUE_FAILED;
		}

		vsllink_tap_init();
	}
	reset_command_pointer();

	return ERROR_OK;
}
static int vsllink_tap_execute_dma(void)
{
	int byte_length;
	int i;
	int result;

	if (tap_length > 0)
	{
		/* Pad last byte so that tap_length is divisible by 8 */
		while (tap_length % 8 != 0)
		{
			/* More of the last TMS value keeps us in the same state,
			 * analogous to free-running JTAG interfaces. */
			vsllink_tap_append_step(last_tms, 0);
		}
		byte_length = tap_length / 8;

		vsllink_usb_out_buffer[0] = VSLLINK_CMD_HW_JTAGRAWCMD;
		vsllink_usb_out_buffer[1] = ((byte_length * 2 + 3) >> 0) & 0xff;		// package size
		vsllink_usb_out_buffer[2] = ((byte_length * 2 + 3) >> 8) & 0xff;

		memcpy(&vsllink_usb_out_buffer[3], tdi_buffer, byte_length);
		memcpy(&vsllink_usb_out_buffer[3 + byte_length], tms_buffer, byte_length);

		result = vsllink_usb_message(vsllink_handle, 3 + 2 * byte_length, byte_length);
		if (result == byte_length)
		{
			for (i = 0; i < pending_scan_results_length; i++)
			{
				struct pending_scan_result *pending_scan_result = &pending_scan_results_buffer[i];
				uint8_t *buffer = pending_scan_result->buffer;
				int length = pending_scan_result->length;
				int first = pending_scan_result->offset;

				struct scan_command *command = pending_scan_result->command;
				buf_set_buf(vsllink_usb_in_buffer, first, buffer, 0, length);

				DEBUG_JTAG_IO("JTAG scan read(%d bits, from %d bits):", length, first);
#ifdef _DEBUG_JTAG_IO_
				vsllink_debug_buffer(buffer, (length + 7) >> 3);
#endif

				if (jtag_read_buffer(buffer, command) != ERROR_OK)
				{
					vsllink_tap_init();
					return ERROR_JTAG_QUEUE_FAILED;
				}

				if (pending_scan_result->buffer != NULL)
				{
					free(pending_scan_result->buffer);
				}
			}
		}
		else
		{
			LOG_ERROR("vsllink_tap_execute, wrong result %d, expected %d", result, byte_length);
			return ERROR_JTAG_QUEUE_FAILED;
		}

		vsllink_tap_init();
	}

	return ERROR_OK;
}

/*****************************************************************************/
/* VSLLink USB low-level functions */

static struct vsllink* vsllink_usb_open(void)
{
	usb_init();

	const uint16_t vids[] = { vsllink_usb_vid, 0 };
	const uint16_t pids[] = { vsllink_usb_pid, 0 };
	struct usb_dev_handle *dev;
	if (jtag_usb_open(vids, pids, &dev) != ERROR_OK)
		return NULL;

	/* usb_set_configuration required under win32 */
	struct usb_device *udev = usb_device(dev);
	int ret = usb_set_configuration(dev, udev->config[0].bConfigurationValue);
	if (ret != 0)
	{
		LOG_ERROR("fail to set configuration to %d (error %d)."
				"Not enough permissions for the device?",
				udev->config[0].bConfigurationValue, ret);
		return NULL;
	}
	ret = usb_claim_interface(dev, vsllink_usb_interface);
	if (ret != 0)
	{
		LOG_ERROR("fail to claim interface %d, %d returned",
				vsllink_usb_interface, ret);
		return NULL;
	}
#if 0
	/*
	* This makes problems under Mac OS X. And is not needed
	* under Windows. Hopefully this will not break a linux build
	*/
	usb_set_altinterface(dev, 0);
#endif

	struct vsllink *result = malloc(sizeof(struct vsllink));
	result->usb_handle = dev;
	return result;
}

static void vsllink_usb_close(struct vsllink *vsllink)
{
	int ret;

	ret = usb_release_interface(vsllink->usb_handle, vsllink_usb_interface);
	if (ret != 0)
	{
		LOG_ERROR("fail to release interface %d, %d returned", vsllink_usb_interface, ret);
		exit(-1);
	}

	ret = usb_close(vsllink->usb_handle);
	if (ret != 0)
	{
		LOG_ERROR("fail to close usb, %d returned", ret);
		exit(-1);
	}

	free(vsllink);
}

/* Send a message and receive the reply. */
static int vsllink_usb_message(struct vsllink *vsllink, int out_length, int in_length)
{
	int result;

	result = vsllink_usb_write(vsllink, out_length);
	if (result == out_length)
	{
		if (in_length > 0)
		{
			result = vsllink_usb_read(vsllink);
			if (result == in_length)
			{
				return result;
			}
			else
			{
				LOG_ERROR("usb_bulk_read failed (requested=%d, result=%d)", in_length, result);
				return -1;
			}
		}
		return 0;
	}
	else
	{
		LOG_ERROR("usb_bulk_write failed (requested=%d, result=%d)", out_length, result);
		return -1;
	}
}

/* Write data from out_buffer to USB. */
static int vsllink_usb_write(struct vsllink *vsllink, int out_length)
{
	int result;

	if (out_length > VSLLINK_BufferSize)
	{
		LOG_ERROR("vsllink_write illegal out_length=%d (max=%d)", out_length, VSLLINK_BufferSize);
		return -1;
	}

	result = usb_bulk_write(vsllink->usb_handle, vsllink_usb_bulkout, \
		(char *)vsllink_usb_out_buffer, out_length, VSLLINK_USB_TIMEOUT);

	DEBUG_JTAG_IO("vsllink_usb_write, out_length = %d, result = %d", out_length, result);

#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("USB out:");
	vsllink_debug_buffer(vsllink_usb_out_buffer, out_length);
#endif

#ifdef _VSLLINK_IN_DEBUG_MODE_
	usleep(100000);
#endif

	return result;
}

/* Read data from USB into in_buffer. */
static int vsllink_usb_read(struct vsllink *vsllink)
{
	int result = usb_bulk_read(vsllink->usb_handle, vsllink_usb_bulkin, \
		(char *)vsllink_usb_in_buffer, VSLLINK_BufferSize, VSLLINK_USB_TIMEOUT);

	DEBUG_JTAG_IO("vsllink_usb_read, result = %d", result);

#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("USB in:");
	vsllink_debug_buffer(vsllink_usb_in_buffer, result);
#endif
	return result;
}

#define BYTES_PER_LINE  16

#if defined _DEBUG_USB_COMMS_ || defined _DEBUG_JTAG_IO_
static void vsllink_debug_buffer(uint8_t *buffer, int length)
{
	char line[81];
	char s[4];
	int i;
	int j;

	for (i = 0; i < length; i += BYTES_PER_LINE)
	{
		snprintf(line, 5, "%04x", i);
		for (j = i; j < i + BYTES_PER_LINE && j < length; j++)
		{
			snprintf(s, 4, " %02x", buffer[j]);
			strcat(line, s);
		}
		LOG_DEBUG("%s", line);
	}
}
#endif // _DEBUG_USB_COMMS_ || _DEBUG_JTAG_IO_

static const struct command_registration vsllink_command_handlers[] = {
	{
		.name = "vsllink_usb_vid",
		.handler = &vsllink_handle_usb_vid_command,
		.mode = COMMAND_CONFIG,
	},
	{
		.name = "vsllink_usb_pid",
		.handler = &vsllink_handle_usb_pid_command,
		.mode = COMMAND_CONFIG,
	},
	{
		.name = "vsllink_usb_bulkin",
		.handler = &vsllink_handle_usb_bulkin_command,
		.mode = COMMAND_CONFIG,
	},
	{
		.name = "vsllink_usb_bulkout",
		.handler = &vsllink_handle_usb_bulkout_command,
		.mode = COMMAND_CONFIG,
	},
	{
		.name = "vsllink_usb_interface",
		.handler = &vsllink_handle_usb_interface_command,
		.mode = COMMAND_CONFIG,
	},
	{
		.name = "vsllink_mode",
		.handler = &vsllink_handle_mode_command,
		.mode = COMMAND_CONFIG,
	},
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface vsllink_interface = {
	.name = "vsllink",
	.commands = vsllink_command_handlers,

	.init = vsllink_init,
	.quit = vsllink_quit,
	.khz = vsllink_khz,
	.speed = vsllink_speed,
	.speed_div = vsllink_speed_div,
	.execute_queue = vsllink_execute_queue,
};
