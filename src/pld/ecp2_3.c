// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "ecp2_3.h"
#include "lattice.h"

#define LSCC_REFRESH         0x23
#define ISC_ENABLE           0x15
#define LSCC_RESET_ADDRESS   0x21
#define ISC_PROGRAM_USERCODE 0x1A
#define ISC_ERASE            0x03
#define READ_USERCODE        0x17
#define ISC_DISABLE          0x1E
#define LSCC_READ_STATUS     0x53
#define LSCC_BITSTREAM_BURST 0x02
#define PROGRAM_SPI          0x3A

#define STATUS_DONE_BIT        0x00020000
#define STATUS_ERROR_BITS_ECP2 0x00040003
#define STATUS_ERROR_BITS_ECP3 0x00040007
#define REGISTER_ALL_BITS_1    0xffffffff
#define REGISTER_ALL_BITS_0    0x00000000

int lattice_ecp2_3_read_status(struct jtag_tap *tap, uint32_t *status, uint32_t out, bool do_idle)
{
	return lattice_read_u32_register(tap, LSCC_READ_STATUS, status, out, do_idle);
}

int lattice_ecp2_3_read_usercode(struct jtag_tap *tap, uint32_t *usercode, uint32_t out)
{
	return lattice_read_u32_register(tap, READ_USERCODE, usercode, out, false);
}

int lattice_ecp2_3_write_usercode(struct lattice_pld_device *lattice_device, uint32_t usercode)
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

	struct scan_field field;
	uint8_t buffer[4];
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

static int lattice_ecp2_3_erase_device(struct lattice_pld_device *lattice_device)
{
	struct jtag_tap *tap = lattice_device->tap;
	if (!tap)
		return ERROR_FAIL;

	/* program user code with all bits set */
	int retval = lattice_set_instr(tap, ISC_PROGRAM_USERCODE, TAP_IRPAUSE);
	if (retval != ERROR_OK)
		return retval;
	struct scan_field field;
	uint8_t buffer[4] = {0xff, 0xff, 0xff, 0xff};
	field.num_bits = 32;
	field.out_value = buffer;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_add_runtest(5, TAP_IDLE);
	jtag_add_sleep(2000);

	/* verify every bit is set */
	const uint32_t out = REGISTER_ALL_BITS_1;
	const uint32_t mask = REGISTER_ALL_BITS_1;
	const uint32_t expected_pre = REGISTER_ALL_BITS_1;
	retval = lattice_verify_usercode(lattice_device, out, expected_pre, mask);
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_set_instr(tap, ISC_ERASE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(5, TAP_IDLE);
	if (lattice_device->family == LATTICE_ECP2)
		jtag_add_sleep(100000);
	else
		jtag_add_sleep(2000000);

	retval = lattice_set_instr(tap, LSCC_RESET_ADDRESS, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(5, TAP_IDLE);
	jtag_add_sleep(2000);

	/* after erasing check all bits in user register are cleared */
	const uint32_t expected_post = REGISTER_ALL_BITS_0;
	return lattice_verify_usercode(lattice_device, out, expected_post, mask);
}

static int lattice_ecp2_3_program_config_map(struct lattice_pld_device *lattice_device,
											struct lattice_bit_file *bit_file)
{
	struct jtag_tap *tap = lattice_device->tap;
	if (!tap)
		return ERROR_FAIL;

	int retval = lattice_set_instr(tap, LSCC_RESET_ADDRESS, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(5, TAP_IDLE);
	jtag_add_sleep(2000);

	struct scan_field field;
	retval = lattice_set_instr(tap, LSCC_BITSTREAM_BURST, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	field.num_bits = (bit_file->raw_bit.length - bit_file->offset) * 8;
	field.out_value = bit_file->raw_bit.data + bit_file->offset;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	jtag_add_runtest(256, TAP_IDLE);
	jtag_add_sleep(2000);
	return jtag_execute_queue();
}

static int lattice_ecp2_3_exit_programming_mode(struct lattice_pld_device *lattice_device)
{
	struct jtag_tap *tap = lattice_device->tap;
	if (!tap)
		return ERROR_FAIL;

	int retval = lattice_set_instr(tap, ISC_DISABLE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(5, TAP_IDLE);
	jtag_add_sleep(200000);
	retval = lattice_set_instr(tap, BYPASS, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(100, TAP_IDLE);
	jtag_add_sleep(1000);
	return jtag_execute_queue();
}

int lattice_ecp2_load(struct lattice_pld_device *lattice_device, struct lattice_bit_file *bit_file)
{
	struct jtag_tap *tap = lattice_device->tap;
	if (!tap)
		return ERROR_FAIL;

	int retval = lattice_preload(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	/* Enable the programming mode */
	retval = lattice_set_instr(tap, LSCC_REFRESH, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = lattice_set_instr(tap, ISC_ENABLE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(5, TAP_IDLE);
	jtag_add_sleep(20000);

	/* Erase the device */
	retval = lattice_ecp2_3_erase_device(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	/* Program Fuse Map */
	retval = lattice_ecp2_3_program_config_map(lattice_device, bit_file);
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_ecp2_3_exit_programming_mode(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	const uint32_t out = REGISTER_ALL_BITS_1;
	const uint32_t mask = STATUS_DONE_BIT | STATUS_ERROR_BITS_ECP2;
	const uint32_t expected = STATUS_DONE_BIT;
	return lattice_verify_status_register_u32(lattice_device, out, expected, mask, false);
}

int lattice_ecp3_load(struct lattice_pld_device *lattice_device, struct lattice_bit_file *bit_file)
{
	struct jtag_tap *tap = lattice_device->tap;
	if (!tap)
		return ERROR_FAIL;

	/* Program Bscan register */
	int retval = lattice_preload(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	/* Enable the programming mode */
	retval = lattice_set_instr(tap, LSCC_REFRESH, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(5, TAP_IDLE);
	jtag_add_sleep(500000);
	retval = lattice_set_instr(tap, ISC_ENABLE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(5, TAP_IDLE);
	jtag_add_sleep(20000);

	retval = lattice_ecp2_3_erase_device(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	/* Program Fuse Map */
	retval = lattice_ecp2_3_program_config_map(lattice_device, bit_file);
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_ecp2_3_exit_programming_mode(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	const uint32_t out = REGISTER_ALL_BITS_1;
	const uint32_t mask = STATUS_DONE_BIT | STATUS_ERROR_BITS_ECP3;
	const uint32_t expected = STATUS_DONE_BIT;
	return lattice_verify_status_register_u32(lattice_device, out, expected, mask, false);
}

int lattice_ecp2_3_connect_spi_to_jtag(struct lattice_pld_device *pld_device_info)
{
	if (!pld_device_info)
		return ERROR_FAIL;

	struct jtag_tap *tap = pld_device_info->tap;
	if (!tap)
		return ERROR_FAIL;

	// erase configuration
	int retval = lattice_set_instr(tap, ISC_ENABLE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = lattice_set_instr(tap, ISC_ERASE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	retval = lattice_set_instr(tap, ISC_DISABLE, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	// connect jtag to spi pins
	retval = lattice_set_instr(tap, PROGRAM_SPI, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	return jtag_execute_queue();
}

int lattice_ecp2_3_disconnect_spi_from_jtag(struct lattice_pld_device *pld_device_info)
{
	if (!pld_device_info)
		return ERROR_FAIL;

	struct jtag_tap *tap = pld_device_info->tap;
	if (!tap)
		return ERROR_FAIL;

	int retval = lattice_set_instr(tap, BYPASS, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	return jtag_execute_queue();
}

int lattice_ecp2_3_get_facing_read_bits(struct lattice_pld_device *pld_device_info, unsigned int *facing_read_bits)
{
	if (!pld_device_info)
		return ERROR_FAIL;

	*facing_read_bits = 1;

	return ERROR_OK;
}

int lattice_ecp2_3_refresh(struct lattice_pld_device *lattice_device)
{
	if (!lattice_device || !lattice_device->tap)
		return ERROR_FAIL;

	int retval = lattice_set_instr(lattice_device->tap, LSCC_REFRESH, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	return jtag_execute_queue();
}
