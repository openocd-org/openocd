/***************************************************************************
 *   Copyright (C) 2007 by Juergen Stuber <juergen@jstuber.net>            *
 *   based on Dominic Rath's and Benedikt Sauter's usbprog.c               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Jean-Christophe PLAGNIOL-VIILARD                *
 *   plagnioj@jcrosoft.com                                                 *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/commands.h>
#include "libusb_common.h"

/* See Segger's public documentation:
 * Reference manual for J-Link USB Protocol
 * Document RM08001-R6 Date: June 16, 2009
 * (Or newer, with some SWD information).
 * http://www.segger.com/cms/admin/uploads/productDocs/RM08001_JLinkUSBProtocol.pdf
 */

/*
 * The default pid of the segger is 0x0101
 * But when you change the USB Address it will also
 *
 * pid = ( usb_address > 0x4) ? 0x0101 : (0x101 + usb_address)
 */

#define JLINK_OB_PID  0x0105

#define JLINK_WRITE_ENDPOINT	0x02
#define JLINK_READ_ENDPOINT		0x81

#define JLINK_OB_WRITE_ENDPOINT	0x06
#define JLINK_OB_READ_ENDPOINT	0x85

static unsigned int jlink_write_ep = JLINK_WRITE_ENDPOINT;
static unsigned int jlink_read_ep = JLINK_READ_ENDPOINT;
static unsigned int jlink_hw_jtag_version = 2;

#define JLINK_USB_TIMEOUT 1000

/* See Section 3.3.2 of the Segger JLink USB protocol manual */
/* 2048 is the max value we can use here */
#define JLINK_TAP_BUFFER_SIZE 2048
/*#define JLINK_TAP_BUFFER_SIZE 256*/
/*#define JLINK_TAP_BUFFER_SIZE 384*/

#define JLINK_IN_BUFFER_SIZE			(2048 + 1)
#define JLINK_OUT_BUFFER_SIZE			(2*2048 + 4)

/* Global USB buffers */
static uint8_t usb_in_buffer[JLINK_IN_BUFFER_SIZE];
static uint8_t usb_out_buffer[JLINK_OUT_BUFFER_SIZE];

/* Constants for JLink command */
#define EMU_CMD_VERSION			0x01
#define EMU_CMD_RESET_TRST		0x02
#define EMU_CMD_RESET_TARGET	0x03
#define EMU_CMD_SET_SPEED		0x05
#define EMU_CMD_GET_STATE		0x07
#define EMU_CMD_SET_KS_POWER	0x08
#define EMU_CMD_GET_SPEEDS		0xc0
#define EMU_CMD_GET_HW_INFO		0xc1
#define EMU_CMD_GET_COUNTERS	0xc2
#define EMU_CMD_SELECT_IF		0xc7
#define EMU_CMD_HW_CLOCK		0xc8
#define EMU_CMD_HW_TMS0			0xc9
#define EMU_CMD_HW_TMS1			0xca
#define EMU_CMD_HW_DATA0		0xcb
#define EMU_CMD_HW_DATA1		0xcc
#define EMU_CMD_HW_JTAG			0xcd
#define EMU_CMD_HW_JTAG2		0xce
#define EMU_CMD_HW_JTAG3		0xcf
#define EMU_CMD_HW_RELEASE_RESET_STOP_EX 0xd0
#define EMU_CMD_HW_RELEASE_RESET_STOP_TIMED 0xd1
#define EMU_CMD_GET_MAX_MEM_BLOCK	0xd4
#define EMU_CMD_HW_JTAG_WRITE		0xd5
#define EMU_CMD_HW_JTAG_GET_RESULT	0xd6
#define EMU_CMD_HW_RESET0		0xdc
#define EMU_CMD_HW_RESET1		0xdd
#define EMU_CMD_HW_TRST0		0xde
#define EMU_CMD_HW_TRST1		0xdf
#define EMU_CMD_GET_CAPS		0xe8
#define EMU_CMD_GET_CPU_CAPS	0xe9
#define EMU_CMD_EXEC_CPU_CMD	0xea
#define EMU_CMD_GET_CAPS_EX		0xed
#define EMU_CMD_GET_HW_VERSION	0xf0
#define EMU_CMD_WRITE_DCC		0xf1
#define EMU_CMD_READ_CONFIG		0xf2
#define EMU_CMD_WRITE_CONFIG		0xf3
#define EMU_CMD_WRITE_MEM			0xf4
#define EMU_CMD_READ_MEM			0xf5
#define EMU_CMD_MEASURE_RTCK_REACT	0xf6
#define EMU_CMD_WRITE_MEM_ARM79		0xf7
#define EMU_CMD_READ_MEM_ARM79		0xf8

/* bits return from EMU_CMD_GET_CAPS */
#define EMU_CAP_RESERVED_1		0
#define EMU_CAP_GET_HW_VERSION		1
#define EMU_CAP_WRITE_DCC		2
#define EMU_CAP_ADAPTIVE_CLOCKING	3
#define EMU_CAP_READ_CONFIG		4
#define EMU_CAP_WRITE_CONFIG		5
#define EMU_CAP_TRACE			6
#define EMU_CAP_WRITE_MEM		7
#define EMU_CAP_READ_MEM		8
#define EMU_CAP_SPEED_INFO		9
#define EMU_CAP_EXEC_CODE		10
#define EMU_CAP_GET_MAX_BLOCK_SIZE	11
#define EMU_CAP_GET_HW_INFO		12
#define EMU_CAP_SET_KS_POWER		13
#define EMU_CAP_RESET_STOP_TIMED	14
#define EMU_CAP_RESERVED_2		15
#define EMU_CAP_MEASURE_RTCK_REACT	16
#define EMU_CAP_SELECT_IF		17
#define EMU_CAP_RW_MEM_ARM79		18
#define EMU_CAP_GET_COUNTERS		19
#define EMU_CAP_READ_DCC		20
#define EMU_CAP_GET_CPU_CAPS		21
#define EMU_CAP_EXEC_CPU_CMD		22
#define EMU_CAP_SWO			23
#define EMU_CAP_WRITE_DCC_EX		24
#define EMU_CAP_UPDATE_FIRMWARE_EX	25
#define EMU_CAP_FILE_IO			26
#define EMU_CAP_REGISTER		27
#define EMU_CAP_INDICATORS		28
#define EMU_CAP_TEST_NET_SPEED		29
#define EMU_CAP_RAWTRACE		30
#define EMU_CAP_RESERVED_3		31

static char *jlink_cap_str[] = {
	"Always 1.",
	"Supports command EMU_CMD_GET_HARDWARE_VERSION",
	"Supports command EMU_CMD_WRITE_DCC",
	"Supports adaptive clocking",
	"Supports command EMU_CMD_READ_CONFIG",
	"Supports command EMU_CMD_WRITE_CONFIG",
	"Supports trace commands",
	"Supports command EMU_CMD_WRITE_MEM",
	"Supports command EMU_CMD_READ_MEM",
	"Supports command EMU_CMD_GET_SPEED",
	"Supports command EMU_CMD_CODE_...",
	"Supports command EMU_CMD_GET_MAX_BLOCK_SIZE",
	"Supports command EMU_CMD_GET_HW_INFO",
	"Supports command EMU_CMD_SET_KS_POWER",
	"Supports command EMU_CMD_HW_RELEASE_RESET_STOP_TIMED",
	"Reserved",
	"Supports command EMU_CMD_MEASURE_RTCK_REACT",
	"Supports command EMU_CMD_HW_SELECT_IF",
	"Supports command EMU_CMD_READ/WRITE_MEM_ARM79",
	"Supports command EMU_CMD_GET_COUNTERS",
	"Supports command EMU_CMD_READ_DCC",
	"Supports command EMU_CMD_GET_CPU_CAPS",
	"Supports command EMU_CMD_EXEC_CPU_CMD",
	"Supports command EMU_CMD_SWO",
	"Supports command EMU_CMD_WRITE_DCC_EX",
	"Supports command EMU_CMD_UPDATE_FIRMWARE_EX",
	"Supports command EMU_CMD_FILE_IO",
	"Supports command EMU_CMD_REGISTER",
	"Supports command EMU_CMD_INDICATORS",
	"Supports command EMU_CMD_TEST_NET_SPEED",
	"Supports command EMU_CMD_RAWTRACE",
	"Reserved",
};

/* max speed 12MHz v5.0 jlink */
#define JLINK_MAX_SPEED 12000

/* J-Link hardware versions */
#define JLINK_HW_TYPE_JLINK	0
#define JLINK_HW_TYPE_JTRACE	1
#define JLINK_HW_TYPE_FLASHER	2
#define JLINK_HW_TYPE_JLINK_PRO	3
#define JLINK_HW_TYPE_MAX	4

static char *jlink_hw_type_str[] = {
	"J-Link",
	"J-Trace",
	"Flasher",
	"J-Link Pro",
};

/* Queue command functions */
static void jlink_end_state(tap_state_t state);
static void jlink_state_move(void);
static void jlink_path_move(int num_states, tap_state_t *path);
static void jlink_runtest(int num_cycles);
static void jlink_scan(bool ir_scan, enum scan_type type, uint8_t *buffer,
		int scan_size, struct scan_command *command);
static void jlink_reset(int trst, int srst);
static void jlink_simple_command(uint8_t command);
static int jlink_get_status(void);

/* J-Link tap buffer functions */
static void jlink_tap_init(void);
static int jlink_tap_execute(void);
static void jlink_tap_ensure_space(int scans, int bits);
static void jlink_tap_append_step(int tms, int tdi);
static void jlink_tap_append_scan(int length, uint8_t *buffer,
		struct scan_command *command);

/* Jlink lowlevel functions */
struct jlink {
	struct jtag_libusb_device_handle *usb_handle;
};

static struct jlink *jlink_usb_open(void);
static void jlink_usb_close(struct jlink *jlink);
static int jlink_usb_message(struct jlink *jlink, int out_length, int in_length);
static int jlink_usb_io(struct jlink *jlink, int out_length, int in_length);
static int jlink_usb_write(struct jlink *jlink, int out_length);
static int jlink_usb_read(struct jlink *jlink, int expected_size);

/* helper functions */
static int jlink_get_version_info(void);

#ifdef _DEBUG_USB_COMMS_
static void jlink_debug_buffer(uint8_t *buffer, int length);
#else
static inline void jlink_debug_buffer(uint8_t *buffer, int length)
{
}
#endif

static enum tap_state jlink_last_state = TAP_RESET;

static struct jlink *jlink_handle;

/* pid could be specified at runtime */
static uint16_t vids[] = { 0x1366, 0x1366, 0x1366, 0x1366, 0x1366, 0 };
static uint16_t pids[] = { 0x0101, 0x0102, 0x0103, 0x0104, 0x0105, 0 };

static uint32_t jlink_caps;
static uint32_t jlink_hw_type;

/* 256 byte non-volatile memory */
struct jlink_config {
	uint8_t	usb_address;
	/* 0ffset 0x01 to 0x03 */
	uint8_t	reserved_1[3];
	uint32_t kickstart_power_on_jtag_pin_19;
	/* 0ffset 0x08 to 0x1f */
	uint8_t reserved_2[24];
	/* IP only for J-Link Pro */
	uint8_t ip_address[4];
	uint8_t subnet_mask[4];
	/* 0ffset 0x28 to 0x2f */
	uint8_t reserved_3[8];
	uint8_t mac_address[6];
	/* 0ffset 0x36 to 0xff */
	uint8_t reserved_4[202];
} __attribute__ ((packed));
struct jlink_config jlink_cfg;

/***************************************************************************/
/* External interface implementation */

static void jlink_execute_runtest(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("runtest %i cycles, end in %i",
			cmd->cmd.runtest->num_cycles,
			cmd->cmd.runtest->end_state);

	jlink_end_state(cmd->cmd.runtest->end_state);

	jlink_runtest(cmd->cmd.runtest->num_cycles);
}

static void jlink_execute_statemove(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("statemove end in %i", cmd->cmd.statemove->end_state);

	jlink_end_state(cmd->cmd.statemove->end_state);
	jlink_state_move();
}

static void jlink_execute_pathmove(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("pathmove: %i states, end in %i",
		cmd->cmd.pathmove->num_states,
		cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

	jlink_path_move(cmd->cmd.pathmove->num_states,
			cmd->cmd.pathmove->path);
}

static void jlink_execute_scan(struct jtag_command *cmd)
{
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;

	DEBUG_JTAG_IO("scan end in %s", tap_state_name(cmd->cmd.scan->end_state));

	jlink_end_state(cmd->cmd.scan->end_state);

	scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
	DEBUG_JTAG_IO("scan input, length = %d", scan_size);

	jlink_debug_buffer(buffer, (scan_size + 7) / 8);
	type = jtag_scan_type(cmd->cmd.scan);
	jlink_scan(cmd->cmd.scan->ir_scan,
			type, buffer, scan_size, cmd->cmd.scan);
}

static void jlink_execute_reset(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("reset trst: %i srst %i",
			cmd->cmd.reset->trst, cmd->cmd.reset->srst);

	jlink_tap_execute();
	jlink_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
	jlink_tap_execute();
}

static void jlink_execute_sleep(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("sleep %" PRIi32 "", cmd->cmd.sleep->us);
	jlink_tap_execute();
	jtag_sleep(cmd->cmd.sleep->us);
}

static void jlink_execute_command(struct jtag_command *cmd)
{
	switch (cmd->type) {
		case JTAG_RUNTEST:
			jlink_execute_runtest(cmd);
			break;
		case JTAG_TLR_RESET:
			jlink_execute_statemove(cmd);
			break;
		case JTAG_PATHMOVE:
			jlink_execute_pathmove(cmd);
			break;
		case JTAG_SCAN:
			jlink_execute_scan(cmd);
			break;
		case JTAG_RESET:
			jlink_execute_reset(cmd);
			break;
		case JTAG_SLEEP:
			jlink_execute_sleep(cmd);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type encountered");
			exit(-1);
	}
}

static int jlink_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;

	while (cmd != NULL) {
		jlink_execute_command(cmd);
		cmd = cmd->next;
	}

	return jlink_tap_execute();
}

/* Sets speed in kHz. */
static int jlink_speed(int speed)
{
	int result;

	if (speed > JLINK_MAX_SPEED) {
		LOG_INFO("reduce speed request: %dkHz to %dkHz maximum",
				speed, JLINK_MAX_SPEED);
		speed = JLINK_MAX_SPEED;
	}

	/* check for RTCK setting */
	if (speed == 0)
		speed = -1;

	usb_out_buffer[0] = EMU_CMD_SET_SPEED;
	usb_out_buffer[1] = (speed >> 0) & 0xff;
	usb_out_buffer[2] = (speed >> 8) & 0xff;

	result = jlink_usb_write(jlink_handle, 3);
	if (result != 3) {
		LOG_ERROR("J-Link setting speed failed (%d)", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int jlink_speed_div(int speed, int *khz)
{
	*khz = speed;

	return ERROR_OK;
}

static int jlink_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;

	return ERROR_OK;
}

/*
 * select transport interface
 *
 * @param	iface [0..31] currently: 0=JTAG, 1=SWD
 * @returns	ERROR_OK or ERROR_ code
 *
 * @pre jlink_handle must be opened
 * @pre function may be called only for devices, that have
 *		EMU_CAP_SELECT_IF capability enabled
 */
static int jlink_select_interface(int iface)
{
	/* According to Segger's document RM08001-R7 Date: October 8, 2010,
	 * http://www.segger.com/admin/uploads/productDocs/RM08001_JLinkUSBProtocol.pdf
	 * section 5.5.3 EMU_CMD_SELECT_IF
	 * > SubCmd 1..31 to select interface (0..31)
	 *
	 * The table below states:
	 *  0 TIF_JTAG
	 *  1 TIF_SWD
	 *
	 * This obviosly means that to select TIF_JTAG one should write SubCmd = 1.
	 *
	 * In fact, JTAG interface operates when SubCmd=0
	 *
	 * It looks like a typo in documentation, because interfaces 0..31 could not
	 * be selected by 1..31 range command.
	 */
	assert(iface >= 0 && iface < 32);
	int result;

	/* get available interfaces */
	usb_out_buffer[0] = EMU_CMD_SELECT_IF;
	usb_out_buffer[1] = 0xff;

	result = jlink_usb_io(jlink_handle, 2, 4);
	if (result != ERROR_OK) {
		LOG_ERROR("J-Link query interface failed (%d)", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	uint32_t iface_mask = buf_get_u32(usb_in_buffer, 0, 32);

	if (!(iface_mask & (1<<iface))) {
		LOG_ERROR("J-Link requesting to select unsupported interface (%" PRIx32 ")", iface_mask);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	/* Select interface */
	usb_out_buffer[0] = EMU_CMD_SELECT_IF;
	usb_out_buffer[1] = iface;

	result = jlink_usb_io(jlink_handle, 2, 4);
	if (result != ERROR_OK) {
		LOG_ERROR("J-Link interface select failed (%d)", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int jlink_init(void)
{
	int i;

	jlink_handle = jlink_usb_open();

	if (jlink_handle == 0) {
		LOG_ERROR("Cannot find jlink Interface! Please check "
				"connection and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}

	/*
	 * The next three instructions were added after discovering a problem
	 * while using an oscilloscope.
	 * For the V8 SAM-ICE dongle (and likely other j-link device variants),
	 * the reset line to the target microprocessor was found to cycle only
	 * intermittently during emulator startup (even after encountering the
	 * downstream reset instruction later in the code).
	 * This was found to create two issues:
	 * 1) In general it is a bad practice to not reset a CPU to a known
	 * state when starting an emulator and
	 * 2) something critical happens inside the dongle when it does the
	 * first read following a new USB session.
	 * Keeping the processor in reset during the first read collecting
	 * version information seems to prevent errant
	 * "J-Link command EMU_CMD_VERSION failed" issues.
	 */

	LOG_INFO("J-Link initialization started / target CPU reset initiated");
	jlink_simple_command(EMU_CMD_HW_TRST0);
	jlink_simple_command(EMU_CMD_HW_RESET0);
	usleep(1000);

	jlink_hw_jtag_version = 2;

	if (jlink_get_version_info() == ERROR_OK) {
		/* attempt to get status */
		jlink_get_status();
	}

	/*
	 * Some versions of Segger's software do not select JTAG interface by default.
	 *
	 * Segger recommends to select interface necessarily as a part of init process,
	 * in case any previous session leaves improper interface selected.
	 *
	 * Until SWD implemented, select only JTAG interface here.
	 */
	if (jlink_caps & (1<<EMU_CAP_SELECT_IF))
		jlink_select_interface(0);

	LOG_INFO("J-Link JTAG Interface ready");

	jlink_reset(0, 0);
	jtag_sleep(3000);
	jlink_tap_init();

	/* v5/6 jlink seems to have an issue if the first tap move
	 * is not divisible by 8, so we send a TLR on first power up */
	for (i = 0; i < 8; i++)
		jlink_tap_append_step(1, 0);
	jlink_tap_execute();

	return ERROR_OK;
}

static int jlink_quit(void)
{
	jlink_usb_close(jlink_handle);
	return ERROR_OK;
}

/***************************************************************************/
/* Queue command implementations */

static void jlink_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

/* Goes to the end state. */
static void jlink_state_move(void)
{
	int i;
	int tms = 0;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	uint8_t tms_scan_bits = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	for (i = 0; i < tms_scan_bits; i++) {
		tms = (tms_scan >> i) & 1;
		jlink_tap_append_step(tms, 0);
	}

	tap_set_state(tap_get_end_state());
}

static void jlink_path_move(int num_states, tap_state_t *path)
{
	int i;

	for (i = 0; i < num_states; i++) {
		if (path[i] == tap_state_transition(tap_get_state(), false))
			jlink_tap_append_step(0, 0);
		else if (path[i] == tap_state_transition(tap_get_state(), true))
			jlink_tap_append_step(1, 0);
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
					tap_state_name(tap_get_state()), tap_state_name(path[i]));
			exit(-1);
		}

		tap_set_state(path[i]);
	}

	tap_set_end_state(tap_get_state());
}

static void jlink_runtest(int num_cycles)
{
	int i;

	tap_state_t saved_end_state = tap_get_end_state();

	jlink_tap_ensure_space(1, num_cycles + 16);

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		jlink_end_state(TAP_IDLE);
		jlink_state_move();
		/* num_cycles--; */
	}

	/* execute num_cycles */
	for (i = 0; i < num_cycles; i++)
		jlink_tap_append_step(0, 0);

	/* finish in end_state */
	jlink_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		jlink_state_move();
}

static void jlink_scan(bool ir_scan, enum scan_type type, uint8_t *buffer,
		int scan_size, struct scan_command *command)
{
	tap_state_t saved_end_state;

	jlink_tap_ensure_space(1, scan_size + 16);

	saved_end_state = tap_get_end_state();

	/* Move to appropriate scan state */
	jlink_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

	/* Only move if we're not already there */
	if (tap_get_state() != tap_get_end_state())
		jlink_state_move();

	jlink_end_state(saved_end_state);

	/* Scan */
	jlink_tap_append_scan(scan_size, buffer, command);

	/* We are in Exit1, go to Pause */
	jlink_tap_append_step(0, 0);

	tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state())
		jlink_state_move();
}

static void jlink_reset(int trst, int srst)
{
	LOG_DEBUG("trst: %i, srst: %i", trst, srst);

	/* Signals are active low */
	if (srst == 0)
		jlink_simple_command(EMU_CMD_HW_RESET1);

	if (srst == 1)
		jlink_simple_command(EMU_CMD_HW_RESET0);

	if (trst == 1)
		jlink_simple_command(EMU_CMD_HW_TRST0);

	if (trst == 0)
		jlink_simple_command(EMU_CMD_HW_TRST1);
}

static void jlink_simple_command(uint8_t command)
{
	int result;

	DEBUG_JTAG_IO("0x%02x", command);

	usb_out_buffer[0] = command;
	result = jlink_usb_write(jlink_handle, 1);

	if (result != 1)
		LOG_ERROR("J-Link command 0x%02x failed (%d)", command, result);
}

static int jlink_get_status(void)
{
	int result;

	jlink_simple_command(EMU_CMD_GET_STATE);

	result = jlink_usb_read(jlink_handle, 8);
	if (result != 8) {
		LOG_ERROR("J-Link command EMU_CMD_GET_STATE failed (%d)", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	int vref = usb_in_buffer[0] + (usb_in_buffer[1] << 8);
	LOG_INFO("Vref = %d.%d TCK = %d TDI = %d TDO = %d TMS = %d SRST = %d TRST = %d", \
		vref / 1000, vref % 1000, \
		usb_in_buffer[2], usb_in_buffer[3], usb_in_buffer[4], \
		usb_in_buffer[5], usb_in_buffer[6], usb_in_buffer[7]);

	if (vref < 1500)
		LOG_ERROR("Vref too low. Check Target Power");

	return ERROR_OK;
}

#define jlink_dump_printf(context, expr ...) \
	do { \
		if (context) \
			command_print(context, expr); \
			else \
			LOG_INFO(expr); \
	} while (0);

static void jlink_caps_dump(struct command_context *ctx)
{
	int i;

	jlink_dump_printf(ctx, "J-Link Capabilities");

	for (i = 1; i < 31; i++)
		if (jlink_caps & (1 << i))
			jlink_dump_printf(ctx, "%s", jlink_cap_str[i]);
}

static void jlink_config_usb_address_dump(struct command_context *ctx, struct jlink_config *cfg)
{
	if (!cfg)
		return;

	jlink_dump_printf(ctx, "USB-Address: 0x%x", cfg->usb_address);
}

static void jlink_config_kickstart_dump(struct command_context *ctx, struct jlink_config *cfg)
{
	if (!cfg)
		return;

	jlink_dump_printf(ctx, "Kickstart power on JTAG-pin 19: 0x%" PRIx32,
		cfg->kickstart_power_on_jtag_pin_19);
}

static void jlink_config_mac_address_dump(struct command_context *ctx, struct jlink_config *cfg)
{
	if (!cfg)
		return;

	jlink_dump_printf(ctx, "MAC Address: %.02x:%.02x:%.02x:%.02x:%.02x:%.02x",
		cfg->mac_address[5], cfg->mac_address[4],
		cfg->mac_address[3], cfg->mac_address[2],
		cfg->mac_address[1], cfg->mac_address[0]);
}

static void jlink_config_ip_dump(struct command_context *ctx, struct jlink_config *cfg)
{
	if (!cfg)
		return;

	jlink_dump_printf(ctx, "IP Address: %d.%d.%d.%d",
		cfg->ip_address[3], cfg->ip_address[2],
		cfg->ip_address[1], cfg->ip_address[0]);
	jlink_dump_printf(ctx, "Subnet Mask: %d.%d.%d.%d",
		cfg->subnet_mask[3], cfg->subnet_mask[2],
		cfg->subnet_mask[1], cfg->subnet_mask[0]);
}

static void jlink_config_dump(struct command_context *ctx, struct jlink_config *cfg)
{
	if (!cfg)
		return;

	jlink_dump_printf(ctx, "J-Link configuration");
	jlink_config_usb_address_dump(ctx, cfg);
	jlink_config_kickstart_dump(ctx, cfg);

	if (jlink_hw_type == JLINK_HW_TYPE_JLINK_PRO) {
		jlink_config_ip_dump(ctx, cfg);
		jlink_config_mac_address_dump(ctx, cfg);
	}
}

static int jlink_get_config(struct jlink_config *cfg)
{
	int result;
	int size = sizeof(struct jlink_config);

	usb_out_buffer[0] = EMU_CMD_READ_CONFIG;
	result = jlink_usb_io(jlink_handle, 1, size);

	if (result != ERROR_OK) {
		LOG_ERROR("jlink_usb_read failed (requested=%d, result=%d)", size, result);
		return ERROR_FAIL;
	}

	memcpy(cfg, usb_in_buffer, size);
	return ERROR_OK;
}

static int jlink_set_config(struct jlink_config *cfg)
{
	int result;
	int size = sizeof(struct jlink_config);

	jlink_simple_command(EMU_CMD_WRITE_CONFIG);

	memcpy(usb_out_buffer, cfg, size);

	result = jlink_usb_write(jlink_handle, size);
	if (result != size) {
		LOG_ERROR("jlink_usb_write failed (requested=%d, result=%d)", 256, result);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/*
 * List of unsupported version string markers.
 *
 * The firmware versions does not correspond directly with
 * "Software and documentation pack for Windows", it may be
 * distinguished by the "compile" date in the information string.
 *
 * For example, version string is:
 *   "J-Link ARM V8 compiled May  3 2012 18:36:22"
 * Marker sould be:
 *   "May  3 2012"
 *
 * The list must be terminated by NULL string.
 */
static const char * const unsupported_versions[] = {
	"Jan 31 2011",
	"JAN 31 2011",
	NULL			/* End of list */
};

static void jlink_check_supported(const char *str)
{
	const char * const *p = unsupported_versions;
	while (*p) {
		if (NULL != strstr(str, *p)) {
			LOG_WARNING(
			"Unsupported J-Link firmware version.\n"
			"       Please check http://www.segger.com/j-link-older-versions.html for updates");
			return;
		}
		p++;
	}
}

static int jlink_get_version_info(void)
{
	int result;
	int len;
	uint32_t jlink_max_size;

	/* query hardware version */
	jlink_simple_command(EMU_CMD_VERSION);

	result = jlink_usb_read(jlink_handle, 2);
	if (2 != result) {
		LOG_ERROR("J-Link command EMU_CMD_VERSION failed (%d)", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	len = buf_get_u32(usb_in_buffer, 0, 16);
	if (len > JLINK_IN_BUFFER_SIZE) {
		LOG_ERROR("J-Link command EMU_CMD_VERSION impossible return length 0x%0x", len);
		len = JLINK_IN_BUFFER_SIZE;
	}

	result = jlink_usb_read(jlink_handle, len);
	if (result != len) {
		LOG_ERROR("J-Link command EMU_CMD_VERSION failed (%d)", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	usb_in_buffer[result] = 0;
	LOG_INFO("%s", (char *)usb_in_buffer);
	jlink_check_supported((char *)usb_in_buffer);

	/* query hardware capabilities */
	jlink_simple_command(EMU_CMD_GET_CAPS);

	result = jlink_usb_read(jlink_handle, 4);
	if (4 != result) {
		LOG_ERROR("J-Link command EMU_CMD_GET_CAPS failed (%d)", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	jlink_caps = buf_get_u32(usb_in_buffer, 0, 32);
	LOG_INFO("J-Link caps 0x%x", (unsigned)jlink_caps);

	if (jlink_caps & (1 << EMU_CAP_GET_HW_VERSION)) {
		/* query hardware version */
		jlink_simple_command(EMU_CMD_GET_HW_VERSION);

		result = jlink_usb_read(jlink_handle, 4);
		if (4 != result) {
			LOG_ERROR("J-Link command EMU_CMD_GET_HW_VERSION failed (%d)", result);
			return ERROR_JTAG_DEVICE_ERROR;
		}

		uint32_t jlink_hw_version = buf_get_u32(usb_in_buffer, 0, 32);
		uint32_t major_revision = (jlink_hw_version / 10000) % 100;
		jlink_hw_type = (jlink_hw_version / 1000000) % 100;
		if (major_revision >= 5)
			jlink_hw_jtag_version = 3;

		LOG_INFO("J-Link hw version %i", (int)jlink_hw_version);

		if (jlink_hw_type >= JLINK_HW_TYPE_MAX)
			LOG_INFO("J-Link hw type uknown 0x%" PRIx32, jlink_hw_type);
		else
			LOG_INFO("J-Link hw type %s", jlink_hw_type_str[jlink_hw_type]);
	}

	if (jlink_caps & (1 << EMU_CAP_GET_MAX_BLOCK_SIZE)) {
		/* query hardware maximum memory block */
		jlink_simple_command(EMU_CMD_GET_MAX_MEM_BLOCK);

		result = jlink_usb_read(jlink_handle, 4);
		if (4 != result) {
			LOG_ERROR("J-Link command EMU_CMD_GET_MAX_MEM_BLOCK failed (%d)", result);
			return ERROR_JTAG_DEVICE_ERROR;
		}

		jlink_max_size = buf_get_u32(usb_in_buffer, 0, 32);
		LOG_INFO("J-Link max mem block %i", (int)jlink_max_size);
	}

	if (jlink_caps & (1 << EMU_CAP_READ_CONFIG)) {
		if (jlink_get_config(&jlink_cfg) != ERROR_OK)
			return ERROR_JTAG_DEVICE_ERROR;

		jlink_config_dump(NULL, &jlink_cfg);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_pid_command)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("Need exactly one argument to jlink_pid");
		return ERROR_FAIL;
	}

	pids[0] = strtoul(CMD_ARGV[0], NULL, 16);
	pids[1] = 0;
	vids[1] = 0;

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_jlink_info_command)
{
	if (jlink_get_version_info() == ERROR_OK) {
		/* attempt to get status */
		jlink_get_status();
	}

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_jlink_caps_command)
{
	jlink_caps_dump(CMD_CTX);

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_jlink_hw_jtag_command)
{
	switch (CMD_ARGC) {
		case 0:
			command_print(CMD_CTX, "J-Link hw jtag  %i", jlink_hw_jtag_version);
			break;
		case 1: {
			int request_version = atoi(CMD_ARGV[0]);
			switch (request_version) {
				case 2:
				case 3:
					jlink_hw_jtag_version = request_version;
					break;
				default:
					return ERROR_COMMAND_SYNTAX_ERROR;
			}
			break;
		}
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_jlink_kickstart_command)
{
	uint32_t kickstart;

	if (CMD_ARGC < 1) {
		jlink_config_kickstart_dump(CMD_CTX, &jlink_cfg);
		return ERROR_OK;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], kickstart);

	jlink_cfg.kickstart_power_on_jtag_pin_19 = kickstart;
	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_jlink_mac_address_command)
{
	uint8_t addr[6];
	int i;
	char *e;
	const char *str;

	if (CMD_ARGC < 1) {
		jlink_config_mac_address_dump(CMD_CTX, &jlink_cfg);
		return ERROR_OK;
	}

	str = CMD_ARGV[0];

	if ((strlen(str) != 17) || (str[2] != ':' || str[5] != ':' || str[8] != ':' ||
		str[11] != ':' || str[14] != ':')) {
		command_print(CMD_CTX, "ethaddr miss format ff:ff:ff:ff:ff:ff");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	for (i = 5; i >= 0; i--) {
		addr[i] = strtoul(str, &e, 16);
		str = e + 1;
	}

	if (!(addr[0] | addr[1] | addr[2] | addr[3] | addr[4] | addr[5])) {
		command_print(CMD_CTX, "invalid it's zero mac_address");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (!(0x01 & addr[0])) {
		command_print(CMD_CTX, "invalid it's a multicat mac_address");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	memcpy(jlink_cfg.mac_address, addr, sizeof(addr));

	return ERROR_OK;
}

static int string_to_ip(const char *s, uint8_t *ip, int *pos)
{
	uint8_t lip[4];
	char *e;
	const char *s_save = s;
	int i;

	if (!s)
		return -EINVAL;

	for (i = 0; i < 4; i++) {
		lip[i] = strtoul(s, &e, 10);

		if (*e != '.' && i != 3)
			return -EINVAL;

		s = e + 1;
	}

	*pos = e - s_save;

	memcpy(ip, lip, sizeof(lip));
	return ERROR_OK;
}

static void cpy_ip(uint8_t *dst, uint8_t *src)
{
	int i, j;

	for (i = 0, j = 3; i < 4; i++, j--)
		dst[i] = src[j];
}

COMMAND_HANDLER(jlink_handle_jlink_ip_command)
{
	uint32_t ip_address;
	uint32_t subnet_mask = 0;
	int i, len;
	int ret;
	uint8_t subnet_bits = 24;

	if (CMD_ARGC < 1) {
		jlink_config_ip_dump(CMD_CTX, &jlink_cfg);
		return ERROR_OK;
	}

	ret = string_to_ip(CMD_ARGV[0], (uint8_t *)&ip_address, &i);
	if (ret != ERROR_OK)
		return ret;

	len = strlen(CMD_ARGV[0]);

	/* check for this format A.B.C.D/E */

	if (i < len) {
		if (CMD_ARGV[0][i] != '/')
			return ERROR_COMMAND_SYNTAX_ERROR;

		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0] + i + 1, subnet_bits);
	} else {
		if (CMD_ARGC > 1) {
			ret = string_to_ip(CMD_ARGV[1], (uint8_t *)&subnet_mask, &i);
			if (ret != ERROR_OK)
				return ret;
		}
	}

	if (!subnet_mask)
		subnet_mask = (uint32_t)(subnet_bits < 32 ?
				((1ULL << subnet_bits) - 1) : 0xffffffff);

	cpy_ip(jlink_cfg.ip_address, (uint8_t *)&ip_address);
	cpy_ip(jlink_cfg.subnet_mask, (uint8_t *)&subnet_mask);

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_jlink_reset_command)
{
	memset(&jlink_cfg, 0xff, sizeof(jlink_cfg));
	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_jlink_save_command)
{
	if (!(jlink_caps & (1 << EMU_CAP_WRITE_CONFIG))) {
		command_print(CMD_CTX, "J-Link write emulator configuration not supported");
		return ERROR_OK;
	}

	command_print(CMD_CTX, "The J-Link need to be unpluged and repluged ta have the config effective");
	return jlink_set_config(&jlink_cfg);
}

COMMAND_HANDLER(jlink_handle_jlink_usb_address_command)
{
	uint32_t address;

	if (CMD_ARGC < 1) {
		jlink_config_usb_address_dump(CMD_CTX, &jlink_cfg);
		return ERROR_OK;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);

	if (address > 0x3 && address != 0xff) {
		command_print(CMD_CTX, "USB Address must be between 0x00 and 0x03 or 0xff");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	jlink_cfg.usb_address = address;
	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_jlink_config_command)
{
	struct jlink_config cfg;
	int ret = ERROR_OK;

	if (CMD_ARGC == 0) {
		if (!(jlink_caps & (1 << EMU_CAP_READ_CONFIG))) {
			command_print(CMD_CTX, "J-Link read emulator configuration not supported");
			goto exit;
		}

		ret = jlink_get_config(&cfg);

		if (ret != ERROR_OK)
			command_print(CMD_CTX, "J-Link read emulator configuration failled");
		else
			jlink_config_dump(CMD_CTX, &jlink_cfg);
	}

exit:
	return ret;
}

static const struct command_registration jlink_config_subcommand_handlers[] = {
	{
		.name = "kickstart",
		.handler = &jlink_handle_jlink_kickstart_command,
		.mode = COMMAND_EXEC,
		.help = "set Kickstart power on JTAG-pin 19.",
		.usage = "[val]",
	},
	{
		.name = "mac_address",
		.handler = &jlink_handle_jlink_mac_address_command,
		.mode = COMMAND_EXEC,
		.help = "set the MAC Address",
		.usage = "[ff:ff:ff:ff:ff:ff]",
	},
	{
		.name = "ip",
		.handler = &jlink_handle_jlink_ip_command,
		.mode = COMMAND_EXEC,
		.help = "set the ip address of the J-Link Pro, "
			"where A.B.C.D is the ip, "
			"E the bit of the subnet mask, "
			"F.G.H.I the subnet mask",
		.usage = "[A.B.C.D[/E] [F.G.H.I]]",
	},
	{
		.name = "reset",
		.handler = &jlink_handle_jlink_reset_command,
		.mode = COMMAND_EXEC,
		.help = "reset the current config",
	},
	{
		.name = "save",
		.handler = &jlink_handle_jlink_save_command,
		.mode = COMMAND_EXEC,
		.help = "save the current config",
	},
	{
		.name = "usb_address",
		.handler = &jlink_handle_jlink_usb_address_command,
		.mode = COMMAND_EXEC,
		.help = "set the USB-Address, "
			"This will change the product id",
		.usage = "[0x00 to 0x03 or 0xff]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration jlink_subcommand_handlers[] = {
	{
		.name = "caps",
		.handler = &jlink_handle_jlink_caps_command,
		.mode = COMMAND_EXEC,
		.help = "show jlink capabilities",
	},
	{
		.name = "info",
		.handler = &jlink_handle_jlink_info_command,
		.mode = COMMAND_EXEC,
		.help = "show jlink info",
	},
	{
		.name = "hw_jtag",
		.handler = &jlink_handle_jlink_hw_jtag_command,
		.mode = COMMAND_EXEC,
		.help = "access J-Link HW JTAG command version",
		.usage = "[2|3]",
	},
	{
		.name = "config",
		.handler = &jlink_handle_jlink_config_command,
		.mode = COMMAND_EXEC,
		.help = "access J-Link configuration, "
			"if no argument this will dump the config",
		.chain = jlink_config_subcommand_handlers,
	},
	{
		.name = "pid",
		.handler = &jlink_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "set the pid of the interface we want to use",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration jlink_command_handlers[] = {
	{
		.name = "jlink",
		.mode = COMMAND_ANY,
		.help = "perform jlink management",
		.chain = jlink_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface jlink_interface = {
	.name = "jlink",
	.commands = jlink_command_handlers,
	.transports = jtag_only,

	.execute_queue = jlink_execute_queue,
	.speed = jlink_speed,
	.speed_div = jlink_speed_div,
	.khz = jlink_khz,
	.init = jlink_init,
	.quit = jlink_quit,
};

/***************************************************************************/
/* J-Link tap functions */


static unsigned tap_length;
static uint8_t tms_buffer[JLINK_TAP_BUFFER_SIZE];
static uint8_t tdi_buffer[JLINK_TAP_BUFFER_SIZE];
static uint8_t tdo_buffer[JLINK_TAP_BUFFER_SIZE];

struct pending_scan_result {
	int first;	/* First bit position in tdo_buffer to read */
	int length; /* Number of bits to read */
	struct scan_command *command; /* Corresponding scan command */
	uint8_t *buffer;
};

#define MAX_PENDING_SCAN_RESULTS 256

static int pending_scan_results_length;
static struct pending_scan_result pending_scan_results_buffer[MAX_PENDING_SCAN_RESULTS];

static void jlink_tap_init(void)
{
	tap_length = 0;
	pending_scan_results_length = 0;
}

static void jlink_tap_ensure_space(int scans, int bits)
{
	int available_scans = MAX_PENDING_SCAN_RESULTS - pending_scan_results_length;
	int available_bits = JLINK_TAP_BUFFER_SIZE * 8 - tap_length - 32;

	if (scans > available_scans || bits > available_bits)
		jlink_tap_execute();
}

static void jlink_tap_append_step(int tms, int tdi)
{
	int index_var = tap_length / 8;

	assert(index_var < JLINK_TAP_BUFFER_SIZE);

	int bit_index = tap_length % 8;
	uint8_t bit = 1 << bit_index;

	/* we do not pad TMS, so be sure to initialize all bits */
	if (0 == bit_index)
		tms_buffer[index_var] = tdi_buffer[index_var] = 0;

	if (tms)
		tms_buffer[index_var] |= bit;
	else
		tms_buffer[index_var] &= ~bit;

	if (tdi)
		tdi_buffer[index_var] |= bit;
	else
		tdi_buffer[index_var] &= ~bit;

	tap_length++;
}

static void jlink_tap_append_scan(int length, uint8_t *buffer,
		struct scan_command *command)
{
	struct pending_scan_result *pending_scan_result =
		&pending_scan_results_buffer[pending_scan_results_length];
	int i;

	pending_scan_result->first = tap_length;
	pending_scan_result->length = length;
	pending_scan_result->command = command;
	pending_scan_result->buffer = buffer;

	for (i = 0; i < length; i++) {
		int tms = (i < (length - 1)) ? 0 : 1;
		int tdi = (buffer[i / 8] & (1 << (i % 8))) != 0;
		jlink_tap_append_step(tms, tdi);
	}
	pending_scan_results_length++;
}

/* Pad and send a tap sequence to the device, and receive the answer.
 * For the purpose of padding we assume that we are in idle or pause state. */
static int jlink_tap_execute(void)
{
	int byte_length;
	int i;
	int result;

	if (!tap_length)
		return ERROR_OK;

	/* JLink returns an extra NULL in packet when size of incoming
	 * message is a multiple of 64, creates problems with USB comms.
	 * WARNING: This will interfere with tap state counting. */
	while ((DIV_ROUND_UP(tap_length, 8) % 64) == 0)
		jlink_tap_append_step((tap_get_state() == TAP_RESET) ? 1 : 0, 0);

	/* number of full bytes (plus one if some would be left over) */
	byte_length = DIV_ROUND_UP(tap_length, 8);

	bool use_jtag3 = jlink_hw_jtag_version >= 3;
	usb_out_buffer[0] = use_jtag3 ? EMU_CMD_HW_JTAG3 : EMU_CMD_HW_JTAG2;
	usb_out_buffer[1] = 0;
	usb_out_buffer[2] = (tap_length >> 0) & 0xff;
	usb_out_buffer[3] = (tap_length >> 8) & 0xff;
	memcpy(usb_out_buffer + 4, tms_buffer, byte_length);
	memcpy(usb_out_buffer + 4 + byte_length, tdi_buffer, byte_length);

	jlink_last_state = jtag_debug_state_machine(tms_buffer, tdi_buffer,
			tap_length, jlink_last_state);

	result = jlink_usb_message(jlink_handle, 4 + 2 * byte_length,
			use_jtag3 ? byte_length + 1 : byte_length);
	if (result != ERROR_OK) {
		LOG_ERROR("jlink_tap_execute failed USB io (%d)", result);
		jlink_tap_init();
		return ERROR_JTAG_QUEUE_FAILED;
	}

	result = use_jtag3 ? usb_in_buffer[byte_length] : 0;
	if (result != 0) {
		LOG_ERROR("jlink_tap_execute failed, result %d", result);
		jlink_tap_init();
		return ERROR_JTAG_QUEUE_FAILED;
	}

	memcpy(tdo_buffer, usb_in_buffer, byte_length);

	for (i = 0; i < pending_scan_results_length; i++) {
		struct pending_scan_result *pending_scan_result = &pending_scan_results_buffer[i];
		uint8_t *buffer = pending_scan_result->buffer;
		int length = pending_scan_result->length;
		int first = pending_scan_result->first;
		struct scan_command *command = pending_scan_result->command;

		/* Copy to buffer */
		buf_set_buf(tdo_buffer, first, buffer, 0, length);

		DEBUG_JTAG_IO("pending scan result, length = %d", length);

		jlink_debug_buffer(buffer, DIV_ROUND_UP(length, 8));

		if (jtag_read_buffer(buffer, command) != ERROR_OK) {
			jlink_tap_init();
			return ERROR_JTAG_QUEUE_FAILED;
		}

		if (pending_scan_result->buffer != NULL)
			free(pending_scan_result->buffer);
	}

	jlink_tap_init();
	return ERROR_OK;
}

/*****************************************************************************/
/* JLink USB low-level functions */

static struct jlink *jlink_usb_open()
{
	struct jtag_libusb_device_handle *devh;
	if (jtag_libusb_open(vids, pids, &devh) != ERROR_OK)
		return NULL;

	/* BE ***VERY CAREFUL*** ABOUT MAKING CHANGES IN THIS
	 * AREA!!!!!!!!!!!  The behavior of libusb is not completely
	 * consistent across Windows, Linux, and Mac OS X platforms.
	 * The actions taken in the following compiler conditionals may
	 * not agree with published documentation for libusb, but were
	 * found to be necessary through trials and tribulations.  Even
	 * little tweaks can break one or more platforms, so if you do
	 * make changes test them carefully on all platforms before
	 * committing them!
	 */

#if IS_WIN32 == 0

	jtag_libusb_reset_device(devh);

#if IS_DARWIN == 0

	int timeout = 5;
	/* reopen jlink after usb_reset
	 * on win32 this may take a second or two to re-enumerate */
	int retval;
	while ((retval = jtag_libusb_open(vids, pids, &devh)) != ERROR_OK) {
		usleep(1000);
		timeout--;
		if (!timeout)
			break;
	}
	if (ERROR_OK != retval)
		return NULL;
#endif

#endif

	/* usb_set_configuration required under win32 */
	struct jtag_libusb_device *udev = jtag_libusb_get_device(devh);
	jtag_libusb_set_configuration(devh, 0);
	jtag_libusb_claim_interface(devh, 0);

#if 0
	/*
	 * This makes problems under Mac OS X. And is not needed
	 * under Windows. Hopefully this will not break a linux build
	 */
	usb_set_altinterface(result->usb_handle, 0);
#endif

	/* Use the OB endpoints if the JLink we matched is a Jlink-OB adapter */
	uint16_t matched_pid;
	if (jtag_libusb_get_pid(udev, &matched_pid) == ERROR_OK) {
		if (matched_pid == JLINK_OB_PID) {
			jlink_read_ep = JLINK_OB_WRITE_ENDPOINT;
			jlink_write_ep = JLINK_OB_READ_ENDPOINT;
		}
	}

	jtag_libusb_get_endpoints(udev, &jlink_read_ep, &jlink_write_ep);

	struct jlink *result = malloc(sizeof(struct jlink));
	result->usb_handle = devh;
	return result;
}

static void jlink_usb_close(struct jlink *jlink)
{
	jtag_libusb_close(jlink->usb_handle);
	free(jlink);
}

/* Send a message and receive the reply. */
static int jlink_usb_message(struct jlink *jlink, int out_length, int in_length)
{
	int result;

	result = jlink_usb_write(jlink, out_length);
	if (result != out_length) {
		LOG_ERROR("usb_bulk_write failed (requested=%d, result=%d)",
				out_length, result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	result = jlink_usb_read(jlink, in_length);
	if (result != in_length) {
		LOG_ERROR("usb_bulk_read failed (requested=%d, result=%d)",
				in_length, result);
		return ERROR_JTAG_DEVICE_ERROR;
	}
	return ERROR_OK;
}

/* calls the given usb_bulk_* function, allowing for the data to
 * trickle in with some timeouts  */
static int usb_bulk_with_retries(
		int (*f)(jtag_libusb_device_handle *, int, char *, int, int),
		jtag_libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout)
{
	int tries = 3, count = 0;

	while (tries && (count < size)) {
		int result = f(dev, ep, bytes + count, size - count, timeout);
		if (result > 0)
			count += result;
		else if ((-ETIMEDOUT != result) || !--tries)
			return result;
	}
	return count;
}

static int wrap_usb_bulk_write(jtag_libusb_device_handle *dev, int ep,
		char *buff, int size, int timeout)
{
	/* usb_bulk_write() takes const char *buff */
	return jtag_libusb_bulk_write(dev, ep, buff, size, timeout);
}

static inline int usb_bulk_write_ex(jtag_libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout)
{
	return usb_bulk_with_retries(&wrap_usb_bulk_write,
			dev, ep, bytes, size, timeout);
}

static inline int usb_bulk_read_ex(jtag_libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout)
{
	return usb_bulk_with_retries(&jtag_libusb_bulk_read,
			dev, ep, bytes, size, timeout);
}

/* Write data from out_buffer to USB. */
static int jlink_usb_write(struct jlink *jlink, int out_length)
{
	int result;

	if (out_length > JLINK_OUT_BUFFER_SIZE) {
		LOG_ERROR("jlink_write illegal out_length=%d (max=%d)",
				out_length, JLINK_OUT_BUFFER_SIZE);
		return -1;
	}

	result = usb_bulk_write_ex(jlink->usb_handle, jlink_write_ep,
		(char *)usb_out_buffer, out_length, JLINK_USB_TIMEOUT);

	DEBUG_JTAG_IO("jlink_usb_write, out_length = %d, result = %d",
			out_length, result);

	jlink_debug_buffer(usb_out_buffer, out_length);
	return result;
}

/* Read data from USB into in_buffer. */
static int jlink_usb_read(struct jlink *jlink, int expected_size)
{
	int result = usb_bulk_read_ex(jlink->usb_handle, jlink_read_ep,
		(char *)usb_in_buffer, expected_size, JLINK_USB_TIMEOUT);

	DEBUG_JTAG_IO("jlink_usb_read, result = %d", result);

	jlink_debug_buffer(usb_in_buffer, result);
	return result;
}

/*
 * Send a message and receive the reply - simple messages.
 *
 * @param jlink pointer to driver data
 * @param out_length data length in @c usb_out_buffer
 * @param in_length data length to be read to @c usb_in_buffer
 */
static int jlink_usb_io(struct jlink *jlink, int out_length, int in_length)
{
	int result;

	result = jlink_usb_write(jlink, out_length);
	if (result != out_length) {
		LOG_ERROR("usb_bulk_write failed (requested=%d, result=%d)",
				out_length, result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	result = jlink_usb_read(jlink, in_length);
	if (result != in_length) {
		LOG_ERROR("usb_bulk_read failed (requested=%d, result=%d)",
				in_length, result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	/*
	 * Section 4.2.4 IN-transaction:
	 * read dummy 0-byte packet if transaction size is
	 * multiple of 64 bytes but not max. size of 0x8000
	 */
	if ((in_length % 64) == 0 && in_length != 0x8000) {
		char dummy_buffer;
		result = usb_bulk_read_ex(jlink->usb_handle, jlink_read_ep,
			&dummy_buffer, 1, JLINK_USB_TIMEOUT);
		if (result != 0) {
			LOG_ERROR("dummy byte read failed");
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}
	return ERROR_OK;
}

#ifdef _DEBUG_USB_COMMS_
#define BYTES_PER_LINE  16

static void jlink_debug_buffer(uint8_t *buffer, int length)
{
	char line[81];
	char s[4];
	int i;
	int j;

	for (i = 0; i < length; i += BYTES_PER_LINE) {
		snprintf(line, 5, "%04x", i);
		for (j = i; j < i + BYTES_PER_LINE && j < length; j++) {
			snprintf(s, 4, " %02x", buffer[j]);
			strcat(line, s);
		}
		LOG_DEBUG("%s", line);
	}
}
#endif
