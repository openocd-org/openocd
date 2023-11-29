// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "ecp5.h"
#include "lattice.h"
#include "lattice_cmd.h"

#define ISC_PROGRAM_USERCODE 0xC2

#define STATUS_DONE_BIT        0x00000100
#define STATUS_ERROR_BITS      0x00020040
#define STATUS_FEA_OTP         0x00004000
#define STATUS_FAIL_FLAG       0x00002000
#define STATUS_BUSY_FLAG       0x00001000
#define REGISTER_ALL_BITS_1    0xffffffff

int lattice_ecp5_read_status(struct jtag_tap *tap, uint32_t *status, uint32_t out, bool do_idle)
{
	return lattice_read_u32_register(tap, LSC_READ_STATUS, status, out, do_idle);
}

int lattice_ecp5_read_usercode(struct jtag_tap *tap, uint32_t *usercode, uint32_t out)
{
	return lattice_read_u32_register(tap, READ_USERCODE, usercode, out, true);
}

int lattice_ecp5_write_usercode(struct lattice_pld_device *lattice_device, uint32_t usercode)
{
	struct jtag_tap *tap = lattice_device->tap;
	if (!tap)
		return ERROR_FAIL;

	int retval = lattice_set_instr(tap, ISC_ENABLE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(5, TAP_IDLE);
	jtag_add_sleep(20000);

	retval = lattice_set_instr(tap, ISC_PROGRAM_USERCODE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	uint8_t buffer[4];
	struct scan_field field;
	h_u32_to_le(buffer, usercode);
	field.num_bits = 32;
	field.out_value = buffer;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_add_runtest(5, TAP_IDLE);
	jtag_add_sleep(2000);

	retval = lattice_set_instr(tap, ISC_DISABLE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(5, TAP_IDLE);
	jtag_add_sleep(200000);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;
	return lattice_verify_usercode(lattice_device, 0x0, usercode, REGISTER_ALL_BITS_1);
}

static int lattice_ecp5_enable_sram_programming(struct jtag_tap *tap)
{
	int retval = lattice_set_instr(tap, ISC_ENABLE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	struct scan_field field;
	uint8_t buffer = 0x0;
	field.num_bits = 8;
	field.out_value = &buffer;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_add_runtest(2, TAP_IDLE);
	jtag_add_sleep(10000);

	return jtag_execute_queue();
}

static int lattice_ecp5_erase_sram(struct jtag_tap *tap)
{
	int retval = lattice_set_instr(tap, ISC_ERASE, TAP_IRPAUSE);
	if (retval != ERROR_OK)
		return retval;

	struct scan_field field;
	uint8_t buffer = 1;
	field.num_bits = 8;
	field.out_value = &buffer;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_add_runtest(2, TAP_IDLE);
	jtag_add_sleep(200000);
	return jtag_execute_queue();
}

static int lattice_ecp5_init_address(struct jtag_tap *tap)
{
	int retval = lattice_set_instr(tap, LSC_INIT_ADDRESS, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	struct scan_field field;
	uint8_t buffer = 1;
	field.num_bits = 8;
	field.out_value = &buffer;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_add_runtest(2, TAP_IDLE);
	jtag_add_sleep(10000);
	return jtag_execute_queue();
}

static int lattice_ecp5_program_config_map(struct jtag_tap *tap, struct lattice_bit_file *bit_file)
{
	int retval = lattice_set_instr(tap, LSC_BITSTREAM_BURST, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(2, TAP_IDLE);
	jtag_add_sleep(10000);

	struct scan_field field;
	field.num_bits = (bit_file->raw_bit.length - bit_file->offset) * 8;
	field.out_value = bit_file->raw_bit.data + bit_file->offset;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	retval = lattice_set_instr(tap, BYPASS, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(100, TAP_IDLE);
	jtag_add_sleep(10000);

	return jtag_execute_queue();
}

static int lattice_ecp5_exit_programming_mode(struct jtag_tap *tap)
{
	int retval = lattice_set_instr(tap, ISC_DISABLE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(2, TAP_IDLE);
	jtag_add_sleep(200000);
	retval = lattice_set_instr(tap, BYPASS, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(2, TAP_IDLE);
	jtag_add_sleep(1000);
	return jtag_execute_queue();
}

int lattice_ecp5_load(struct lattice_pld_device *lattice_device, struct lattice_bit_file *bit_file)
{
	struct jtag_tap *tap = lattice_device->tap;
	if (!tap)
		return ERROR_FAIL;

	int retval = lattice_preload(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_ecp5_enable_sram_programming(tap);
	if (retval != ERROR_OK)
		return retval;

	const uint32_t out = 0x0;
	const uint32_t expected1 = 0x0;
	const uint32_t mask1 = STATUS_ERROR_BITS | STATUS_FEA_OTP;
	retval = lattice_verify_status_register_u32(lattice_device, out, expected1, mask1, true);
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_ecp5_erase_sram(tap);
	if (retval != ERROR_OK)
		return retval;

	const uint32_t mask2 = STATUS_FAIL_FLAG | STATUS_BUSY_FLAG;
	retval = lattice_verify_status_register_u32(lattice_device, out, expected1, mask2, false);
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_ecp5_init_address(tap);
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_ecp5_program_config_map(tap, bit_file);
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_ecp5_exit_programming_mode(tap);
	if (retval != ERROR_OK)
		return retval;

	const uint32_t expected2 = STATUS_DONE_BIT;
	const uint32_t mask3 = STATUS_DONE_BIT | STATUS_FAIL_FLAG;
	return lattice_verify_status_register_u32(lattice_device, out, expected2, mask3, false);
}

int lattice_ecp5_connect_spi_to_jtag(struct lattice_pld_device *pld_device_info)
{
	if (!pld_device_info)
		return ERROR_FAIL;

	struct jtag_tap *tap = pld_device_info->tap;
	if (!tap)
		return ERROR_FAIL;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) == PROGRAM_SPI)
		return ERROR_OK;

	// erase configuration
	int retval = lattice_preload(pld_device_info);
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_ecp5_enable_sram_programming(tap);
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_ecp5_erase_sram(tap);
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_ecp5_exit_programming_mode(tap);
	if (retval != ERROR_OK)
		return retval;

	// connect jtag to spi pins
	retval = lattice_set_instr(tap, PROGRAM_SPI, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	struct scan_field field;
	uint8_t buffer[2] = {0xfe, 0x68};
	field.num_bits = 16;
	field.out_value = buffer;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	return jtag_execute_queue();
}

int lattice_ecp5_disconnect_spi_from_jtag(struct lattice_pld_device *pld_device_info)
{
	if (!pld_device_info)
		return ERROR_FAIL;

	struct jtag_tap *tap = pld_device_info->tap;
	if (!tap)
		return ERROR_FAIL;

	/* Connecting it again takes way too long to do it multiple times for writing
	   a bitstream (ca. 0.4s each access).
	   We just leave it connected since SCS will not be active when not in shift_dr state.
	   So there is no need to change instruction, just make sure we are not in shift dr state. */
	jtag_add_runtest(2, TAP_IDLE);
	return jtag_execute_queue();
}

int lattice_ecp5_get_facing_read_bits(struct lattice_pld_device *pld_device_info, unsigned int *facing_read_bits)
{
	if (!pld_device_info)
		return ERROR_FAIL;

	*facing_read_bits = 0;

	return ERROR_OK;
}

int lattice_ecp5_refresh(struct lattice_pld_device *lattice_device)
{
	struct jtag_tap *tap = lattice_device->tap;
	if (!tap)
		return ERROR_FAIL;

	int retval = lattice_preload(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_set_instr(tap, LSC_REFRESH, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(2, TAP_IDLE);
	jtag_add_sleep(200000);
	retval = lattice_set_instr(tap, BYPASS, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(100, TAP_IDLE);
	jtag_add_sleep(1000);

	return jtag_execute_queue();
}
