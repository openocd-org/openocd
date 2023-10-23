// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "certus.h"
#include "lattice.h"
#include "lattice_cmd.h"

#define LSC_ENABLE_X    0x74
#define LSC_REFRESH     0x79
#define LSC_DEVICE_CTRL 0x7D

int lattice_certus_read_status(struct jtag_tap *tap, uint64_t *status, uint64_t out)
{
	return lattice_read_u64_register(tap, LSC_READ_STATUS, status, out);
}

int lattice_certus_read_usercode(struct jtag_tap *tap, uint32_t *usercode, uint32_t out)
{
	return lattice_read_u32_register(tap, READ_USERCODE, usercode, out, false);
}

int lattice_certus_write_usercode(struct lattice_pld_device *lattice_device, uint32_t usercode)
{
	LOG_ERROR("Not supported to write usercode on certus devices");
	return ERROR_FAIL;
}

static int lattice_certus_enable_transparent_mode(struct jtag_tap *tap)
{
	struct scan_field field;

	int retval = lattice_set_instr(tap, LSC_ENABLE_X, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	uint8_t buffer = 0x0;
	field.num_bits = 8;
	field.out_value = &buffer;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_add_runtest(2, TAP_IDLE);

	return jtag_execute_queue();
}

static int lattice_certus_erase_device(struct lattice_pld_device *lattice_device)
{
	struct jtag_tap *tap = lattice_device->tap;
	if (!tap)
		return ERROR_FAIL;

	int retval = lattice_set_instr(tap, LSC_DEVICE_CTRL, TAP_IRPAUSE);
	if (retval != ERROR_OK)
		return retval;

	struct scan_field field;
	uint8_t buffer = 8;
	field.num_bits = 8;
	field.out_value = &buffer;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_add_runtest(2, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_set_instr(tap, LSC_DEVICE_CTRL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	buffer = 0;
	field.num_bits = 8;
	field.out_value = &buffer;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_add_runtest(2, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_set_instr(tap, ISC_ERASE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	buffer = 0;
	field.num_bits = 8;
	field.out_value = &buffer;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_add_runtest(100, TAP_IDLE);
	jtag_add_sleep(5000);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	/* check done is cleared and fail is cleared */
	const uint64_t status_done_flag =  0x100;
	const uint64_t status_fail_flag = 0x2000;
	return lattice_verify_status_register_u64(lattice_device, 0x0, 0x0, status_done_flag | status_fail_flag);
}

static int lattice_certus_enable_programming(struct jtag_tap *tap)
{
	struct scan_field field;

	int retval = lattice_set_instr(tap, LSC_REFRESH, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(2, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_set_instr(tap, ISC_ENABLE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	uint8_t buffer = 0;
	field.num_bits = 8;
	field.out_value = &buffer;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_add_runtest(2, TAP_IDLE);
	return jtag_execute_queue();
}

static int lattice_certus_init_address(struct jtag_tap *tap)
{
	int retval = lattice_set_instr(tap, LSC_INIT_ADDRESS, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(2, TAP_IDLE);
	return jtag_execute_queue();
}

static int lattice_certus_exit_programming_mode(struct jtag_tap *tap)
{
	int retval = lattice_set_instr(tap, ISC_DISABLE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(2, TAP_IDLE);
	retval = lattice_set_instr(tap, BYPASS, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(100, TAP_IDLE);
	return jtag_execute_queue();
}

static int lattice_certus_program_config_map(struct jtag_tap *tap, struct lattice_bit_file *bit_file)
{
	struct scan_field field;

	int retval = lattice_set_instr(tap, LSC_BITSTREAM_BURST, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	field.num_bits = (bit_file->raw_bit.length - bit_file->offset) * 8;
	field.out_value = bit_file->raw_bit.data + bit_file->offset;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	return jtag_execute_queue();
}

int lattice_certus_load(struct lattice_pld_device *lattice_device, struct lattice_bit_file *bit_file)
{
	struct jtag_tap *tap = lattice_device->tap;
	if (!tap)
		return ERROR_FAIL;

	int retval = lattice_preload(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	/*  check password protection is disabled */
	const uint64_t status_pwd_protection = 0x20000;
	retval = lattice_verify_status_register_u64(lattice_device, 0x0, 0x0, status_pwd_protection);
	if (retval != ERROR_OK) {
		LOG_ERROR("Password protection is set");
		return retval;
	}

	retval = lattice_certus_enable_transparent_mode(tap);
	if (retval != ERROR_OK)
		return retval;

	/* Check the SRAM Erase Lock */
	const uint64_t status_otp = 0x40;
	retval = lattice_verify_status_register_u64(lattice_device, 0x0, status_otp, status_otp);
	if (retval != ERROR_OK) {
		LOG_ERROR("NV User Feature Sector OTP is Set");
		return retval;
	}

	/* Check the SRAM Lock */
	const uint64_t status_write_protected = 0x400;
	retval = lattice_verify_status_register_u64(lattice_device, 0x0, 0x0, status_write_protected);
	if (retval != ERROR_OK) {
		LOG_ERROR("NV User Feature Sector OTP is Set");
		return retval;
	}

	retval = lattice_certus_enable_programming(tap);
	if (retval != ERROR_OK) {
		LOG_ERROR("failed to enable programming mode");
		return retval;
	}

	retval = lattice_certus_erase_device(lattice_device);
	if (retval != ERROR_OK) {
		LOG_ERROR("erasing device failed");
		return retval;
	}

	retval = lattice_certus_init_address(tap);
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_certus_program_config_map(tap, bit_file);
	if (retval != ERROR_OK)
		return retval;
	const uint32_t expected = 0x100; // done
	const uint32_t mask = expected |
			0x3000 | // Busy Flag and Fail Flag
			0xf000000; // BSE Error
	retval = lattice_verify_status_register_u64(lattice_device, 0x0, 0x100, mask);
	if (retval != ERROR_OK)
		return retval;

	return lattice_certus_exit_programming_mode(tap);
}
