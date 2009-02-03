/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

#include "str9xpec.h"
#include "flash.h"
#include "target.h"
#include "log.h"
#include "armv4_5.h"
#include "arm7_9_common.h"
#include "jtag.h"
#include "binarybuffer.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <getopt.h>

int str9xpec_register_commands(struct command_context_s *cmd_ctx);
int str9xpec_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);
int str9xpec_erase(struct flash_bank_s *bank, int first, int last);
int str9xpec_protect(struct flash_bank_s *bank, int set, int first, int last);
int str9xpec_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int str9xpec_probe(struct flash_bank_s *bank);
int str9xpec_handle_part_id_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int str9xpec_protect_check(struct flash_bank_s *bank);
int str9xpec_erase_check(struct flash_bank_s *bank);
int str9xpec_info(struct flash_bank_s *bank, char *buf, int buf_size);

int str9xpec_erase_area(struct flash_bank_s *bank, int first, int last);
int str9xpec_set_address(struct flash_bank_s *bank, u8 sector);
int str9xpec_write_options(struct flash_bank_s *bank);

int str9xpec_handle_flash_options_cmap_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int str9xpec_handle_flash_options_lvdthd_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int str9xpec_handle_flash_options_lvdsel_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int str9xpec_handle_flash_options_lvdwarn_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int str9xpec_handle_flash_options_read_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int str9xpec_handle_flash_options_write_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int str9xpec_handle_flash_lock_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int str9xpec_handle_flash_unlock_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int str9xpec_handle_flash_enable_turbo_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int str9xpec_handle_flash_disable_turbo_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

flash_driver_t str9xpec_flash =
{
	.name = "str9xpec",
	.register_commands = str9xpec_register_commands,
	.flash_bank_command = str9xpec_flash_bank_command,
	.erase = str9xpec_erase,
	.protect = str9xpec_protect,
	.write = str9xpec_write,
	.probe = str9xpec_probe,
	.auto_probe = str9xpec_probe,
	.erase_check = str9xpec_erase_check,
	.protect_check = str9xpec_protect_check,
	.info = str9xpec_info
};

int str9xpec_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *str9xpec_cmd = register_command(cmd_ctx, NULL, "str9xpec", NULL, COMMAND_ANY, "str9xpec flash specific commands");

	register_command(cmd_ctx, str9xpec_cmd, "enable_turbo", str9xpec_handle_flash_enable_turbo_command, COMMAND_EXEC,
					 "enable str9xpec turbo mode");
	register_command(cmd_ctx, str9xpec_cmd, "disable_turbo", str9xpec_handle_flash_disable_turbo_command, COMMAND_EXEC,
					 "disable str9xpec turbo mode");
	register_command(cmd_ctx, str9xpec_cmd, "options_cmap", str9xpec_handle_flash_options_cmap_command, COMMAND_EXEC,
					 "configure str9xpec boot sector");
	register_command(cmd_ctx, str9xpec_cmd, "options_lvdthd", str9xpec_handle_flash_options_lvdthd_command, COMMAND_EXEC,
					 "configure str9xpec lvd threshold");
	register_command(cmd_ctx, str9xpec_cmd, "options_lvdsel", str9xpec_handle_flash_options_lvdsel_command, COMMAND_EXEC,
					 "configure str9xpec lvd selection");
	register_command(cmd_ctx, str9xpec_cmd, "options_lvdwarn", str9xpec_handle_flash_options_lvdwarn_command, COMMAND_EXEC,
					 "configure str9xpec lvd warning");
	register_command(cmd_ctx, str9xpec_cmd, "options_read", str9xpec_handle_flash_options_read_command, COMMAND_EXEC,
					 "read str9xpec options");
	register_command(cmd_ctx, str9xpec_cmd, "options_write", str9xpec_handle_flash_options_write_command, COMMAND_EXEC,
					 "write str9xpec options");
	register_command(cmd_ctx, str9xpec_cmd, "lock", str9xpec_handle_flash_lock_command, COMMAND_EXEC,
					 "lock str9xpec device");
	register_command(cmd_ctx, str9xpec_cmd, "unlock", str9xpec_handle_flash_unlock_command, COMMAND_EXEC,
					 "unlock str9xpec device");
	register_command(cmd_ctx, str9xpec_cmd, "part_id", str9xpec_handle_part_id_command, COMMAND_EXEC,
					 "print part id of str9xpec flash bank <num>");

	return ERROR_OK;
}

int str9xpec_set_instr(jtag_tap_t *tap, u32 new_instr, tap_state_t end_state)
{
	if( tap == NULL ){
		return ERROR_TARGET_INVALID;
	}

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != new_instr)
	{
		scan_field_t field;

		field.tap = tap;
		field.num_bits = tap->ir_length;
		field.out_value = calloc(CEIL(field.num_bits, 8), 1);
		buf_set_u32(field.out_value, 0, field.num_bits, new_instr);
		field.out_mask = NULL;
		field.in_value = NULL;
		field.in_check_value = NULL;
		field.in_check_mask = NULL;
		field.in_handler = NULL;
		field.in_handler_priv = NULL;

		jtag_add_ir_scan(1, &field, end_state);

		free(field.out_value);
	}

	return ERROR_OK;
}

u8 str9xpec_isc_status(jtag_tap_t *tap)
{
	scan_field_t field;
	u8 status;

	if (str9xpec_set_instr(tap, ISC_NOOP, TAP_IRPAUSE) != ERROR_OK)
		return ISC_STATUS_ERROR;

	field.tap = tap;
	field.num_bits = 8;
	field.out_value = NULL;
	field.out_mask = NULL;
	field.in_value = &status;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;

	jtag_add_dr_scan(1, &field, TAP_IDLE);
	jtag_execute_queue();

	LOG_DEBUG("status: 0x%2.2x", status);

	if (status & ISC_STATUS_SECURITY)
		LOG_INFO("Device Security Bit Set");

	return status;
}

int str9xpec_isc_enable(struct flash_bank_s *bank)
{
	u8 status;
	jtag_tap_t *tap;
	str9xpec_flash_controller_t *str9xpec_info = bank->driver_priv;

	tap = str9xpec_info->tap;

	if (str9xpec_info->isc_enable)
		return ERROR_OK;

	/* enter isc mode */
	if (str9xpec_set_instr(tap, ISC_ENABLE, TAP_IDLE) != ERROR_OK)
		return ERROR_TARGET_INVALID;

	/* check ISC status */
	status = str9xpec_isc_status(tap);
	if (status & ISC_STATUS_MODE)
	{
		/* we have entered isc mode */
		str9xpec_info->isc_enable = 1;
		LOG_DEBUG("ISC_MODE Enabled");
	}

	return ERROR_OK;
}

int str9xpec_isc_disable(struct flash_bank_s *bank)
{
	u8 status;
	jtag_tap_t *tap;
	str9xpec_flash_controller_t *str9xpec_info = bank->driver_priv;

	tap = str9xpec_info->tap;

	if (!str9xpec_info->isc_enable)
		return ERROR_OK;

	if (str9xpec_set_instr(tap, ISC_DISABLE, TAP_IDLE) != ERROR_OK)
		return ERROR_TARGET_INVALID;

	/* delay to handle aborts */
	jtag_add_sleep(50);

	/* check ISC status */
	status = str9xpec_isc_status(tap);
	if (!(status & ISC_STATUS_MODE))
	{
		/* we have left isc mode */
		str9xpec_info->isc_enable = 0;
		LOG_DEBUG("ISC_MODE Disabled");
	}

	return ERROR_OK;
}

int str9xpec_read_config(struct flash_bank_s *bank)
{
	scan_field_t field;
	u8 status;
	jtag_tap_t *tap;

	str9xpec_flash_controller_t *str9xpec_info = bank->driver_priv;

	tap = str9xpec_info->tap;

	LOG_DEBUG("ISC_CONFIGURATION");

	/* execute ISC_CONFIGURATION command */
	str9xpec_set_instr(tap, ISC_CONFIGURATION, TAP_IRPAUSE);

	field.tap = tap;
	field.num_bits = 64;
	field.out_value = NULL;
	field.out_mask = NULL;
	field.in_value = str9xpec_info->options;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;

	jtag_add_dr_scan(1, &field, TAP_IDLE);
	jtag_execute_queue();

	status = str9xpec_isc_status(tap);

	return status;
}

int str9xpec_build_block_list(struct flash_bank_s *bank)
{
	str9xpec_flash_controller_t *str9xpec_info = bank->driver_priv;

	int i;
	int num_sectors;
	int b0_sectors = 0, b1_sectors = 0;
	u32 offset = 0;
	int b1_size = 0x2000;

	switch (bank->size)
	{
		case (256 * 1024):
			b0_sectors = 4;
			break;
		case (512 * 1024):
			b0_sectors = 8;
			break;
		case (1024 * 1024):
			b0_sectors = 16;
			break;
		case (2048 * 1024):
			b0_sectors = 32;
			break;
		case (128 * 1024):
			b1_size = 0x4000;
			b1_sectors = 8;
			break;
		case (32 * 1024):
			b1_sectors = 4;
			break;
		default:
			LOG_ERROR("BUG: unknown bank->size encountered");
			exit(-1);
	}

	num_sectors = b0_sectors + b1_sectors;

	bank->num_sectors = num_sectors;
	bank->sectors = malloc(sizeof(flash_sector_t) * num_sectors);
	str9xpec_info->sector_bits = malloc(sizeof(u32) * num_sectors);

	num_sectors = 0;

	for (i = 0; i < b0_sectors; i++)
	{
		bank->sectors[num_sectors].offset = offset;
		bank->sectors[num_sectors].size = 0x10000;
		offset += bank->sectors[i].size;
		bank->sectors[num_sectors].is_erased = -1;
		bank->sectors[num_sectors].is_protected = 1;
		str9xpec_info->sector_bits[num_sectors++] = i;
	}

	for (i = 0; i < b1_sectors; i++)
	{
		bank->sectors[num_sectors].offset = offset;
		bank->sectors[num_sectors].size = b1_size;
		offset += bank->sectors[i].size;
		bank->sectors[num_sectors].is_erased = -1;
		bank->sectors[num_sectors].is_protected = 1;
		str9xpec_info->sector_bits[num_sectors++] = i + 32;
	}

	return ERROR_OK;
}

/* flash bank str9x <base> <size> 0 0 <target#>
 */
int str9xpec_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	str9xpec_flash_controller_t *str9xpec_info;
	armv4_5_common_t *armv4_5 = NULL;
	arm7_9_common_t *arm7_9 = NULL;
	arm_jtag_t *jtag_info = NULL;

	if (argc < 6)
	{
		LOG_WARNING("incomplete flash_bank str9x configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	str9xpec_info = malloc(sizeof(str9xpec_flash_controller_t));
	bank->driver_priv = str9xpec_info;

	/* find out jtag position of flash controller
	 * it is always after the arm966 core */

	armv4_5 = bank->target->arch_info;
	arm7_9 = armv4_5->arch_info;
	jtag_info = &arm7_9->jtag_info;

	str9xpec_info->tap = jtag_TapByAbsPosition( jtag_info->tap->abs_chain_position - 1);
	str9xpec_info->isc_enable = 0;

	str9xpec_build_block_list(bank);

	/* clear option byte register */
	buf_set_u32(str9xpec_info->options, 0, 64, 0);

	return ERROR_OK;
}

int str9xpec_blank_check(struct flash_bank_s *bank, int first, int last)
{
	scan_field_t field;
	u8 status;
	jtag_tap_t *tap;
	int i;
	u8 *buffer = NULL;

	str9xpec_flash_controller_t *str9xpec_info = bank->driver_priv;

	tap = str9xpec_info->tap;

	if (!str9xpec_info->isc_enable) {
		str9xpec_isc_enable( bank );
	}

	if (!str9xpec_info->isc_enable) {
		return ERROR_FLASH_OPERATION_FAILED;
	}

	buffer = calloc(CEIL(64, 8), 1);

	LOG_DEBUG("blank check: first_bank: %i, last_bank: %i", first, last);

	for (i = first; i <= last; i++) {
		buf_set_u32(buffer, str9xpec_info->sector_bits[i], 1, 1);
	}

	/* execute ISC_BLANK_CHECK command */
	str9xpec_set_instr(tap, ISC_BLANK_CHECK, TAP_IRPAUSE);

	field.tap = tap;
	field.num_bits = 64;
	field.out_value = buffer;
	field.out_mask = NULL;
	field.in_value = NULL;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;

	jtag_add_dr_scan(1, &field, TAP_IDLE);
	jtag_add_sleep(40000);

	/* read blank check result */
	field.tap = tap;
	field.num_bits = 64;
	field.out_value = NULL;
	field.out_mask = NULL;
	field.in_value = buffer;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;

	jtag_add_dr_scan(1, &field, TAP_IRPAUSE);
	jtag_execute_queue();

	status = str9xpec_isc_status(tap);

	for (i = first; i <= last; i++)
	{
		if (buf_get_u32(buffer, str9xpec_info->sector_bits[i], 1))
			bank->sectors[i].is_erased = 0;
		else
			bank->sectors[i].is_erased = 1;
	}

	free(buffer);

	str9xpec_isc_disable(bank);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;
	return ERROR_OK;
}

int str9xpec_protect_check(struct flash_bank_s *bank)
{
	u8 status;
	int i;

	str9xpec_flash_controller_t *str9xpec_info = bank->driver_priv;

	status = str9xpec_read_config(bank);

	for (i = 0; i < bank->num_sectors; i++)
	{
		if (buf_get_u32(str9xpec_info->options, str9xpec_info->sector_bits[i], 1))
			bank->sectors[i].is_protected = 1;
		else
			bank->sectors[i].is_protected = 0;
	}

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;
	return ERROR_OK;
}

int str9xpec_erase_area(struct flash_bank_s *bank, int first, int last)
{
	scan_field_t field;
	u8 status;
	jtag_tap_t *tap;
	int i;
	u8 *buffer = NULL;

	str9xpec_flash_controller_t *str9xpec_info = bank->driver_priv;

	tap = str9xpec_info->tap;

	if (!str9xpec_info->isc_enable) {
		str9xpec_isc_enable( bank );
	}

	if (!str9xpec_info->isc_enable) {
		return ISC_STATUS_ERROR;
	}

	buffer = calloc(CEIL(64, 8), 1);

	LOG_DEBUG("erase: first_bank: %i, last_bank: %i", first, last);

	/* last bank: 0xFF signals a full erase (unlock complete device) */
	/* last bank: 0xFE signals a option byte erase */
	if (last == 0xFF)
	{
		for (i = 0; i < 64; i++) {
			buf_set_u32(buffer, i, 1, 1);
		}
	}
	else if (last == 0xFE)
	{
		buf_set_u32(buffer, 49, 1, 1);
	}
	else
	{
		for (i = first; i <= last; i++) {
			buf_set_u32(buffer, str9xpec_info->sector_bits[i], 1, 1);
		}
	}

	LOG_DEBUG("ISC_ERASE");

	/* execute ISC_ERASE command */
	str9xpec_set_instr(tap, ISC_ERASE, TAP_IRPAUSE);

	field.tap = tap;
	field.num_bits = 64;
	field.out_value = buffer;
	field.out_mask = NULL;
	field.in_value = NULL;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;

	jtag_add_dr_scan(1, &field, TAP_IDLE);
	jtag_execute_queue();

	jtag_add_sleep(10);

	/* wait for erase completion */
	while (!((status = str9xpec_isc_status(tap)) & ISC_STATUS_BUSY)) {
		alive_sleep(1);
	}

	free(buffer);

	str9xpec_isc_disable(bank);

	return status;
}

int str9xpec_erase(struct flash_bank_s *bank, int first, int last)
{
	int status;

	status = str9xpec_erase_area(bank, first, last);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

int str9xpec_lock_device(struct flash_bank_s *bank)
{
	scan_field_t field;
	u8 status;
	jtag_tap_t *tap;
	str9xpec_flash_controller_t *str9xpec_info = NULL;

	str9xpec_info = bank->driver_priv;
	tap = str9xpec_info->tap;

	if (!str9xpec_info->isc_enable) {
		str9xpec_isc_enable( bank );
	}

	if (!str9xpec_info->isc_enable) {
		return ISC_STATUS_ERROR;
	}

	/* set security address */
	str9xpec_set_address(bank, 0x80);

	/* execute ISC_PROGRAM command */
	str9xpec_set_instr(tap, ISC_PROGRAM_SECURITY, TAP_IDLE);

	str9xpec_set_instr(tap, ISC_NOOP, TAP_IRPAUSE);

	do {
		field.tap = tap;
		field.num_bits = 8;
		field.out_value = NULL;
		field.out_mask = NULL;
		field.in_value = &status;
		field.in_check_value = NULL;
		field.in_check_mask = NULL;
		field.in_handler = NULL;
		field.in_handler_priv = NULL;

		jtag_add_dr_scan(1, &field, -1);
		jtag_execute_queue();

	} while(!(status & ISC_STATUS_BUSY));

	str9xpec_isc_disable(bank);

	return status;
}

int str9xpec_unlock_device(struct flash_bank_s *bank)
{
	u8 status;

	status = str9xpec_erase_area(bank, 0, 255);

	return status;
}

int str9xpec_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	u8 status;
	int i;

	str9xpec_flash_controller_t *str9xpec_info = bank->driver_priv;

	status = str9xpec_read_config(bank);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	LOG_DEBUG("protect: first_bank: %i, last_bank: %i", first, last);

	/* last bank: 0xFF signals a full device protect */
	if (last == 0xFF)
	{
		if( set )
		{
			status = str9xpec_lock_device(bank);
		}
		else
		{
			/* perform full erase to unlock device */
			status = str9xpec_unlock_device(bank);
		}
	}
	else
	{
		for (i = first; i <= last; i++)
		{
			if( set )
				buf_set_u32(str9xpec_info->options, str9xpec_info->sector_bits[i], 1, 1);
			else
				buf_set_u32(str9xpec_info->options, str9xpec_info->sector_bits[i], 1, 0);
		}

		status = str9xpec_write_options(bank);
	}

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

int str9xpec_set_address(struct flash_bank_s *bank, u8 sector)
{
	jtag_tap_t *tap;
	scan_field_t field;
	str9xpec_flash_controller_t *str9xpec_info = bank->driver_priv;

	tap = str9xpec_info->tap;

	/* set flash controller address */
	str9xpec_set_instr(tap, ISC_ADDRESS_SHIFT, TAP_IRPAUSE);

	field.tap = tap;
	field.num_bits = 8;
	field.out_value = &sector;
	field.out_mask = NULL;
	field.in_value = NULL;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;

	jtag_add_dr_scan(1, &field, -1);

	return ERROR_OK;
}

int str9xpec_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	str9xpec_flash_controller_t *str9xpec_info = bank->driver_priv;
	u32 dwords_remaining = (count / 8);
	u32 bytes_remaining = (count & 0x00000007);
	u32 bytes_written = 0;
	u8 status;
	u32 check_address = offset;
	jtag_tap_t *tap;
	scan_field_t field;
	u8 *scanbuf;
	int i;
	u32 first_sector = 0;
	u32 last_sector = 0;

	tap = str9xpec_info->tap;

	if (!str9xpec_info->isc_enable) {
		str9xpec_isc_enable(bank);
	}

	if (!str9xpec_info->isc_enable) {
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (offset & 0x7)
	{
		LOG_WARNING("offset 0x%x breaks required 8-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	for (i = 0; i < bank->num_sectors; i++)
	{
		u32 sec_start = bank->sectors[i].offset;
		u32 sec_end = sec_start + bank->sectors[i].size;

		/* check if destination falls within the current sector */
		if ((check_address >= sec_start) && (check_address < sec_end))
		{
			/* check if destination ends in the current sector */
			if (offset + count < sec_end)
				check_address = offset + count;
			else
				check_address = sec_end;
		}

		if ((offset >= sec_start) && (offset < sec_end)){
			first_sector = i;
		}

		if ((offset + count >= sec_start) && (offset + count < sec_end)){
			last_sector = i;
		}
	}

	if (check_address != offset + count)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	LOG_DEBUG("first_sector: %i, last_sector: %i", first_sector, last_sector);

	scanbuf = calloc(CEIL(64, 8), 1);

	LOG_DEBUG("ISC_PROGRAM");

	for (i = first_sector; i <= last_sector; i++)
	{
		str9xpec_set_address(bank, str9xpec_info->sector_bits[i]);

		dwords_remaining = dwords_remaining < (bank->sectors[i].size/8) ? dwords_remaining : (bank->sectors[i].size/8);

		while (dwords_remaining > 0)
		{
			str9xpec_set_instr(tap, ISC_PROGRAM, TAP_IRPAUSE);

			field.tap = tap;
			field.num_bits = 64;
			field.out_value = (buffer + bytes_written);
			field.out_mask = NULL;
			field.in_value = NULL;
			field.in_check_value = NULL;
			field.in_check_mask = NULL;
			field.in_handler = NULL;
			field.in_handler_priv = NULL;

			jtag_add_dr_scan(1, &field, TAP_IDLE);

			/* small delay before polling */
			jtag_add_sleep(50);

			str9xpec_set_instr(tap, ISC_NOOP, TAP_IRPAUSE);

			do {
				field.tap = tap;
				field.num_bits = 8;
				field.out_value = NULL;
				field.out_mask = NULL;
				field.in_value = scanbuf;
				field.in_check_value = NULL;
				field.in_check_mask = NULL;
				field.in_handler = NULL;
				field.in_handler_priv = NULL;

				jtag_add_dr_scan(1, &field, -1);
				jtag_execute_queue();

				status = buf_get_u32(scanbuf, 0, 8);

			} while(!(status & ISC_STATUS_BUSY));

			if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
				return ERROR_FLASH_OPERATION_FAILED;

			/* if ((status & ISC_STATUS_INT_ERROR) != STR9XPEC_ISC_INTFAIL)
				return ERROR_FLASH_OPERATION_FAILED; */

			dwords_remaining--;
			bytes_written += 8;
		}
	}

	if (bytes_remaining)
	{
		u8 last_dword[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
		int i = 0;

		while(bytes_remaining > 0)
		{
			last_dword[i++] = *(buffer + bytes_written);
			bytes_remaining--;
			bytes_written++;
		}

		str9xpec_set_instr(tap, ISC_PROGRAM, TAP_IRPAUSE);

		field.tap = tap;
		field.num_bits = 64;
		field.out_value = last_dword;
		field.out_mask = NULL;
		field.in_value = NULL;
		field.in_check_value = NULL;
		field.in_check_mask = NULL;
		field.in_handler = NULL;
		field.in_handler_priv = NULL;

		jtag_add_dr_scan(1, &field, TAP_IDLE);

		/* small delay before polling */
		jtag_add_sleep(50);

		str9xpec_set_instr(tap, ISC_NOOP, TAP_IRPAUSE);

		do {
			field.tap = tap;
			field.num_bits = 8;
			field.out_value = NULL;
			field.out_mask = NULL;
			field.in_value = scanbuf;
			field.in_check_value = NULL;
			field.in_check_mask = NULL;
			field.in_handler = NULL;
			field.in_handler_priv = NULL;

			jtag_add_dr_scan(1, &field, -1);
			jtag_execute_queue();

			status = buf_get_u32(scanbuf, 0, 8);

		} while(!(status & ISC_STATUS_BUSY));

		if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
			return ERROR_FLASH_OPERATION_FAILED;

		/* if ((status & ISC_STATUS_INT_ERROR) != STR9XPEC_ISC_INTFAIL)
			return ERROR_FLASH_OPERATION_FAILED; */
	}

	free(scanbuf);

	str9xpec_isc_disable(bank);

	return ERROR_OK;
}

int str9xpec_probe(struct flash_bank_s *bank)
{
	return ERROR_OK;
}

int str9xpec_handle_part_id_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	scan_field_t field;
	u8 *buffer = NULL;
	jtag_tap_t *tap;
	u32 idcode;
	str9xpec_flash_controller_t *str9xpec_info = NULL;

	if (argc < 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	str9xpec_info = bank->driver_priv;
	tap = str9xpec_info->tap;

	buffer = calloc(CEIL(32, 8), 1);

	str9xpec_set_instr(tap, ISC_IDCODE, TAP_IRPAUSE);

	field.tap = tap;
	field.num_bits = 32;
	field.out_value = NULL;
	field.out_mask = NULL;
	field.in_value = buffer;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;

	jtag_add_dr_scan(1, &field, TAP_IDLE);
	jtag_execute_queue();

	idcode = buf_get_u32(buffer, 0, 32);

	command_print(cmd_ctx, "str9xpec part id: 0x%8.8x", idcode);

	free(buffer);

	return ERROR_OK;
}

int str9xpec_erase_check(struct flash_bank_s *bank)
{
	return str9xpec_blank_check(bank, 0, bank->num_sectors - 1);
}

int str9xpec_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "str9xpec flash driver info" );
	return ERROR_OK;
}

int str9xpec_handle_flash_options_read_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	u8 status;
	str9xpec_flash_controller_t *str9xpec_info = NULL;

	if (argc < 1)
	{
		command_print(cmd_ctx, "str9xpec options_read <bank>");
		return ERROR_OK;
	}

	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	str9xpec_info = bank->driver_priv;

	status = str9xpec_read_config(bank);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	/* boot bank */
	if (buf_get_u32(str9xpec_info->options, STR9XPEC_OPT_CSMAPBIT, 1))
		command_print(cmd_ctx, "CS Map: bank1");
	else
		command_print(cmd_ctx, "CS Map: bank0");

	/* OTP lock */
	if (buf_get_u32(str9xpec_info->options, STR9XPEC_OPT_OTPBIT, 1))
		command_print(cmd_ctx, "OTP Lock: OTP Locked");
	else
		command_print(cmd_ctx, "OTP Lock: OTP Unlocked");

	/* LVD Threshold */
	if (buf_get_u32(str9xpec_info->options, STR9XPEC_OPT_LVDTHRESBIT, 1))
		command_print(cmd_ctx, "LVD Threshold: 2.7v");
	else
		command_print(cmd_ctx, "LVD Threshold: 2.4v");

	/* LVD reset warning */
	if (buf_get_u32(str9xpec_info->options, STR9XPEC_OPT_LVDWARNBIT, 1))
		command_print(cmd_ctx, "LVD Reset Warning: VDD or VDDQ Inputs");
	else
		command_print(cmd_ctx, "LVD Reset Warning: VDD Input Only");

	/* LVD reset select */
	if (buf_get_u32(str9xpec_info->options, STR9XPEC_OPT_LVDSELBIT, 1))
		command_print(cmd_ctx, "LVD Reset Selection: VDD or VDDQ Inputs");
	else
		command_print(cmd_ctx, "LVD Reset Selection: VDD Input Only");

	return ERROR_OK;
}

int str9xpec_write_options(struct flash_bank_s *bank)
{
	scan_field_t field;
	u8 status;
	jtag_tap_t *tap;
	str9xpec_flash_controller_t *str9xpec_info = NULL;

	str9xpec_info = bank->driver_priv;
	tap = str9xpec_info->tap;

	/* erase config options first */
	status = str9xpec_erase_area( bank, 0xFE, 0xFE );

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return status;

	if (!str9xpec_info->isc_enable) {
		str9xpec_isc_enable( bank );
	}

	if (!str9xpec_info->isc_enable) {
		return ISC_STATUS_ERROR;
	}

	/* according to data 64th bit has to be set */
	buf_set_u32(str9xpec_info->options, 63, 1, 1);

	/* set option byte address */
	str9xpec_set_address(bank, 0x50);

	/* execute ISC_PROGRAM command */
	str9xpec_set_instr(tap, ISC_PROGRAM, TAP_IRPAUSE);

	field.tap = tap;
	field.num_bits = 64;
	field.out_value = str9xpec_info->options;
	field.out_mask = NULL;
	field.in_value = NULL;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;

	jtag_add_dr_scan(1, &field, TAP_IDLE);

	/* small delay before polling */
	jtag_add_sleep(50);

	str9xpec_set_instr(tap, ISC_NOOP, TAP_IRPAUSE);

	do {
		field.tap = tap;
		field.num_bits = 8;
		field.out_value = NULL;
		field.out_mask = NULL;
		field.in_value = &status;
		field.in_check_value = NULL;
		field.in_check_mask = NULL;
		field.in_handler = NULL;
		field.in_handler_priv = NULL;

		jtag_add_dr_scan(1, &field, -1);
		jtag_execute_queue();

	} while(!(status & ISC_STATUS_BUSY));

	str9xpec_isc_disable(bank);

	return status;
}

int str9xpec_handle_flash_options_write_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	u8 status;

	if (argc < 1)
	{
		command_print(cmd_ctx, "str9xpec options_write <bank>");
		return ERROR_OK;
	}

	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	status = str9xpec_write_options(bank);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

int str9xpec_handle_flash_options_cmap_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	str9xpec_flash_controller_t *str9xpec_info = NULL;

	if (argc < 2)
	{
		command_print(cmd_ctx, "str9xpec options_cmap <bank> <bank0|bank1>");
		return ERROR_OK;
	}

	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	str9xpec_info = bank->driver_priv;

	if (strcmp(args[1], "bank1") == 0)
	{
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_CSMAPBIT, 1, 1);
	}
	else
	{
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_CSMAPBIT, 1, 0);
	}

	return ERROR_OK;
}

int str9xpec_handle_flash_options_lvdthd_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	str9xpec_flash_controller_t *str9xpec_info = NULL;

	if (argc < 2)
	{
		command_print(cmd_ctx, "str9xpec options_lvdthd <bank> <2.4v|2.7v>");
		return ERROR_OK;
	}

	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	str9xpec_info = bank->driver_priv;

	if (strcmp(args[1], "2.7v") == 0)
	{
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_LVDTHRESBIT, 1, 1);
	}
	else
	{
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_LVDTHRESBIT, 1, 0);
	}

	return ERROR_OK;
}

int str9xpec_handle_flash_options_lvdsel_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	str9xpec_flash_controller_t *str9xpec_info = NULL;

	if (argc < 2)
	{
		command_print(cmd_ctx, "str9xpec options_lvdsel <bank> <vdd|vdd_vddq>");
		return ERROR_OK;
	}

	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	str9xpec_info = bank->driver_priv;

	if (strcmp(args[1], "vdd_vddq") == 0)
	{
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_LVDSELBIT, 1, 1);
	}
	else
	{
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_LVDSELBIT, 1, 0);
	}

	return ERROR_OK;
}

int str9xpec_handle_flash_options_lvdwarn_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	str9xpec_flash_controller_t *str9xpec_info = NULL;

	if (argc < 2)
	{
		command_print(cmd_ctx, "str9xpec options_lvdwarn <bank> <vdd|vdd_vddq>");
		return ERROR_OK;
	}

	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	str9xpec_info = bank->driver_priv;

	if (strcmp(args[1], "vdd_vddq") == 0)
	{
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_LVDWARNBIT, 1, 1);
	}
	else
	{
		buf_set_u32(str9xpec_info->options, STR9XPEC_OPT_LVDWARNBIT, 1, 0);
	}

	return ERROR_OK;
}

int str9xpec_handle_flash_lock_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u8 status;
	flash_bank_t *bank;

	if (argc < 1)
	{
		command_print(cmd_ctx, "str9xpec lock <bank>");
		return ERROR_OK;
	}

	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	status = str9xpec_lock_device(bank);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

int str9xpec_handle_flash_unlock_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u8 status;
	flash_bank_t *bank;

	if (argc < 1)
	{
		command_print(cmd_ctx, "str9xpec unlock <bank>");
		return ERROR_OK;
	}

	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	status = str9xpec_unlock_device(bank);

	if ((status & ISC_STATUS_ERROR) != STR9XPEC_ISC_SUCCESS)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

int str9xpec_handle_flash_enable_turbo_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	flash_bank_t *bank;
	jtag_tap_t *tap0;
	jtag_tap_t *tap1;
	jtag_tap_t *tap2;
	str9xpec_flash_controller_t *str9xpec_info = NULL;

	if (argc < 1)
	{
		command_print(cmd_ctx, "str9xpec enable_turbo <bank>");
		return ERROR_OK;
	}

	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	str9xpec_info = bank->driver_priv;

	tap0 = str9xpec_info->tap;

	/* remove arm core from chain - enter turbo mode */
	tap1 = tap0->next_tap;
	if (tap1 == NULL)
	{
		/* things are *WRONG* */
		command_print(cmd_ctx,"**STR9FLASH** (tap1) invalid chain?");
		return ERROR_OK;
	}
	tap2 = tap1->next_tap;
	if (tap2 == NULL)
	{
		/* things are *WRONG* */
		command_print(cmd_ctx,"**STR9FLASH** (tap2) invalid chain?");
		return ERROR_OK;
	}

	/* enable turbo mode - TURBO-PROG-ENABLE */
	str9xpec_set_instr(tap2, 0xD, TAP_IDLE);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
		return retval;

	/* modify scan chain - str9 core has been removed */
	tap1->enabled = 0;

	return ERROR_OK;
}

int str9xpec_handle_flash_disable_turbo_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	jtag_tap_t *tap;
	str9xpec_flash_controller_t *str9xpec_info = NULL;

	if (argc < 1)
	{
		command_print(cmd_ctx, "str9xpec disable_turbo <bank>");
		return ERROR_OK;
	}

	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	str9xpec_info = bank->driver_priv;
	tap = str9xpec_info->tap;

	if (tap == NULL)
		return ERROR_FAIL;

	/* exit turbo mode via RESET */
	str9xpec_set_instr(tap, ISC_NOOP, TAP_RESET);
	jtag_execute_queue();

	/* restore previous scan chain */
	if (tap->next_tap) {
		tap->next_tap->enabled = 1;
	}

	return ERROR_OK;
}
