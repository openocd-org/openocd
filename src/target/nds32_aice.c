/***************************************************************************
 *   Copyright (C) 2013 Andes technology.                                  *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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

#include <helper/log.h>
#include "nds32_aice.h"

int aice_read_reg_64(struct aice_port_s *aice, uint32_t num, uint64_t *val)
{
	if (aice->port->api->read_reg_64 == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice->port->api->read_reg_64(num, val);
}

int aice_write_reg_64(struct aice_port_s *aice, uint32_t num, uint64_t val)
{
	if (aice->port->api->write_reg_64 == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice->port->api->write_reg_64(num, val);
}

int aice_select_target(struct aice_port_s *aice, uint32_t target_id)
{
	if (aice->port->api->select_target == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice->port->api->select_target(target_id);
}

int aice_read_tlb(struct aice_port_s *aice, uint32_t virtual_address,
		uint32_t *physical_address)
{
	if (aice->port->api->read_tlb == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice->port->api->read_tlb(virtual_address, physical_address);
}

int aice_cache_ctl(struct aice_port_s *aice, uint32_t subtype, uint32_t address)
{
	if (aice->port->api->cache_ctl == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice->port->api->cache_ctl(subtype, address);
}

int aice_set_retry_times(struct aice_port_s *aice, uint32_t a_retry_times)
{
	if (aice->port->api->set_retry_times == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice->port->api->set_retry_times(a_retry_times);
}

int aice_program_edm(struct aice_port_s *aice, char *command_sequence)
{
	if (aice->port->api->program_edm == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice->port->api->program_edm(command_sequence);
}

int aice_pack_command(struct aice_port_s *aice, bool enable_pack_command)
{
	if (aice->port->api->pack_command == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice->port->api->pack_command(enable_pack_command);
}

int aice_execute(struct aice_port_s *aice, uint32_t *instructions,
		uint32_t instruction_num)
{
	if (aice->port->api->execute == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice->port->api->execute(instructions, instruction_num);
}

int aice_set_custom_srst_script(struct aice_port_s *aice, const char *script)
{
	if (aice->port->api->set_custom_srst_script == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice->port->api->set_custom_srst_script(script);
}

int aice_set_custom_trst_script(struct aice_port_s *aice, const char *script)
{
	if (aice->port->api->set_custom_trst_script == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice->port->api->set_custom_trst_script(script);
}

int aice_set_custom_restart_script(struct aice_port_s *aice, const char *script)
{
	if (aice->port->api->set_custom_restart_script == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice->port->api->set_custom_restart_script(script);
}

int aice_set_count_to_check_dbger(struct aice_port_s *aice, uint32_t count_to_check)
{
	if (aice->port->api->set_count_to_check_dbger == NULL) {
		LOG_WARNING("Not implemented: %s", __func__);
		return ERROR_FAIL;
	}

	return aice->port->api->set_count_to_check_dbger(count_to_check);
}
