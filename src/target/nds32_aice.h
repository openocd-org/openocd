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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_NDS32_AICE_H
#define OPENOCD_TARGET_NDS32_AICE_H

#include <jtag/aice/aice_port.h>

int aice_read_reg_64(struct aice_port_s *aice, uint32_t num, uint64_t *val);
int aice_write_reg_64(struct aice_port_s *aice, uint32_t num, uint64_t val);
int aice_read_tlb(struct aice_port_s *aice, target_addr_t virtual_address,
		target_addr_t *physical_address);
int aice_cache_ctl(struct aice_port_s *aice, uint32_t subtype, uint32_t address);
int aice_set_retry_times(struct aice_port_s *aice, uint32_t a_retry_times);
int aice_program_edm(struct aice_port_s *aice, char *command_sequence);
int aice_set_command_mode(struct aice_port_s *aice,
		enum aice_command_mode command_mode);
int aice_execute(struct aice_port_s *aice, uint32_t *instructions,
		uint32_t instruction_num);
int aice_set_custom_srst_script(struct aice_port_s *aice, const char *script);
int aice_set_custom_trst_script(struct aice_port_s *aice, const char *script);
int aice_set_custom_restart_script(struct aice_port_s *aice, const char *script);
int aice_set_count_to_check_dbger(struct aice_port_s *aice, uint32_t count_to_check);
int aice_profiling(struct aice_port_s *aice, uint32_t interval, uint32_t iteration,
		uint32_t reg_no, uint32_t *samples, uint32_t *num_samples);

static inline int aice_open(struct aice_port_s *aice, struct aice_port_param_s *param)
{
	return aice->port->api->open(param);
}

static inline int aice_close(struct aice_port_s *aice)
{
	return aice->port->api->close();
}

static inline int aice_reset(struct aice_port_s *aice)
{
	return aice->port->api->reset();
}

static inline int aice_assert_srst(struct aice_port_s *aice,
		enum aice_srst_type_s srst)
{
	return aice->port->api->assert_srst(aice->coreid, srst);
}

static inline int aice_run(struct aice_port_s *aice)
{
	return aice->port->api->run(aice->coreid);
}

static inline int aice_halt(struct aice_port_s *aice)
{
	return aice->port->api->halt(aice->coreid);
}

static inline int aice_step(struct aice_port_s *aice)
{
	return aice->port->api->step(aice->coreid);
}

static inline int aice_read_register(struct aice_port_s *aice, uint32_t num,
		uint32_t *val)
{
	return aice->port->api->read_reg(aice->coreid, num, val);
}

static inline int aice_write_register(struct aice_port_s *aice, uint32_t num,
		uint32_t val)
{
	return aice->port->api->write_reg(aice->coreid, num, val);
}

static inline int aice_read_debug_reg(struct aice_port_s *aice, uint32_t addr,
		uint32_t *val)
{
	return aice->port->api->read_debug_reg(aice->coreid, addr, val);
}

static inline int aice_write_debug_reg(struct aice_port_s *aice, uint32_t addr,
		const uint32_t val)
{
	return aice->port->api->write_debug_reg(aice->coreid, addr, val);
}

static inline int aice_read_mem_unit(struct aice_port_s *aice, uint32_t addr,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	return aice->port->api->read_mem_unit(aice->coreid, addr, size, count, buffer);
}

static inline int aice_write_mem_unit(struct aice_port_s *aice, uint32_t addr,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	return aice->port->api->write_mem_unit(aice->coreid, addr, size, count, buffer);
}

static inline int aice_read_mem_bulk(struct aice_port_s *aice, uint32_t addr,
		uint32_t length, uint8_t *buffer)
{
	return aice->port->api->read_mem_bulk(aice->coreid, addr, length, buffer);
}

static inline int aice_write_mem_bulk(struct aice_port_s *aice, uint32_t addr,
		uint32_t length, const uint8_t *buffer)
{
	return aice->port->api->write_mem_bulk(aice->coreid, addr, length, buffer);
}

static inline int aice_idcode(struct aice_port_s *aice, uint32_t *idcode,
		uint8_t *num_of_idcode)
{
	return aice->port->api->idcode(idcode, num_of_idcode);
}

static inline int aice_state(struct aice_port_s *aice,
		enum aice_target_state_s *state)
{
	return aice->port->api->state(aice->coreid, state);
}

static inline int aice_set_jtag_clock(struct aice_port_s *aice, uint32_t a_clock)
{
	return aice->port->api->set_jtag_clock(a_clock);
}

static inline int aice_memory_access(struct aice_port_s *aice,
		enum nds_memory_access a_access)
{
	return aice->port->api->memory_access(aice->coreid, a_access);
}

static inline int aice_memory_mode(struct aice_port_s *aice,
		enum nds_memory_select mem_select)
{
	return aice->port->api->memory_mode(aice->coreid, mem_select);
}

static inline int aice_set_data_endian(struct aice_port_s *aice,
		enum aice_target_endian target_data_endian)
{
	return aice->port->api->set_data_endian(aice->coreid, target_data_endian);
}

#endif /* OPENOCD_TARGET_NDS32_AICE_H */
