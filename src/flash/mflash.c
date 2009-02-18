/***************************************************************************
 *   Copyright (C) 2007-2008 by unsik Kim <donari75@gmail.com>             *
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

#include <ctype.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <inttypes.h>

#include "command.h"
#include "log.h"
#include "target.h"
#include "time_support.h"
#include "fileio.h"
#include "mflash.h"

static int s3c2440_set_gpio_to_output (mflash_gpio_num_t gpio);
static int s3c2440_set_gpio_output_val (mflash_gpio_num_t gpio, u8 val);
static int pxa270_set_gpio_to_output (mflash_gpio_num_t gpio);
static int pxa270_set_gpio_output_val (mflash_gpio_num_t gpio, u8 val);

static command_t *mflash_cmd;

static mflash_bank_t *mflash_bank;

static mflash_gpio_drv_t pxa270_gpio = {
	.name = "pxa270",
	.set_gpio_to_output = pxa270_set_gpio_to_output,
	.set_gpio_output_val = pxa270_set_gpio_output_val
};

static mflash_gpio_drv_t s3c2440_gpio = {
	.name = "s3c2440",
	.set_gpio_to_output = s3c2440_set_gpio_to_output,
	.set_gpio_output_val = s3c2440_set_gpio_output_val
};

static mflash_gpio_drv_t *mflash_gpio[] =
{
		&pxa270_gpio,
		&s3c2440_gpio,
		NULL
};

#define PXA270_GAFR0_L 0x40E00054
#define PXA270_GAFR3_U 0x40E00070
#define PXA270_GAFR3_U_RESERVED_BITS  0xfffc0000u
#define PXA270_GPDR0 0x40E0000C
#define PXA270_GPDR3 0x40E0010C
#define PXA270_GPDR3_RESERVED_BITS  0xfe000000u
#define PXA270_GPSR0 0x40E00018
#define PXA270_GPCR0 0x40E00024

static int pxa270_set_gpio_to_output (mflash_gpio_num_t gpio)
{
	u32 addr, value, mask;
	target_t *target = mflash_bank->target;
	int ret;

	/* remove alternate function. */
	mask = 0x3u << (gpio.num & 0xF)*2;

	addr = PXA270_GAFR0_L + (gpio.num >> 4) * 4;

	if ((ret = target_read_u32(target, addr, &value)) != ERROR_OK)
		return ret;

	value &= ~mask;
	if (addr == PXA270_GAFR3_U)
		value &= ~PXA270_GAFR3_U_RESERVED_BITS;

	if ((ret = target_write_u32(target, addr, value)) != ERROR_OK)
		return ret;

	/* set direction to output */
	mask = 0x1u << (gpio.num & 0x1F);

	addr = PXA270_GPDR0 + (gpio.num >> 5) * 4;

	if ((ret = target_read_u32(target, addr, &value)) != ERROR_OK)
		return ret;

	value |= mask;
	if (addr == PXA270_GPDR3)
		value &= ~PXA270_GPDR3_RESERVED_BITS;

	ret = target_write_u32(target, addr, value);
	return ret;
}

static int pxa270_set_gpio_output_val (mflash_gpio_num_t gpio, u8 val)
{
	u32 addr, value, mask;
	target_t *target = mflash_bank->target;
	int ret;

	mask = 0x1u << (gpio.num & 0x1F);

	if (val) {
		addr = PXA270_GPSR0 + (gpio.num >> 5) * 4;
	} else {
		addr = PXA270_GPCR0 + (gpio.num >> 5) * 4;
	}

	if ((ret = target_read_u32(target, addr, &value)) != ERROR_OK)
		return ret;

	value |= mask;

	ret = target_write_u32(target, addr, value);

	return ret;
}

#define S3C2440_GPACON 0x56000000
#define S3C2440_GPADAT 0x56000004
#define S3C2440_GPJCON 0x560000d0
#define S3C2440_GPJDAT 0x560000d4

static int s3c2440_set_gpio_to_output (mflash_gpio_num_t gpio)
{
	u32 data, mask, gpio_con;
	target_t *target = mflash_bank->target;
	int ret;

	if (gpio.port[0] >= 'a' && gpio.port[0] <= 'h') {
		gpio_con = S3C2440_GPACON + (gpio.port[0] - 'a') * 0x10;
	} else if (gpio.port[0] == 'j') {
		gpio_con = S3C2440_GPJCON;
	} else {
		LOG_ERROR("invalid port %d%s", gpio.num, gpio.port);
		return ERROR_INVALID_ARGUMENTS;
	}

	ret = target_read_u32(target, gpio_con, &data);

	if (ret == ERROR_OK) {
		if (gpio.port[0] == 'a') {
			mask = 1 << gpio.num;
			data &= ~mask;
		} else {
			mask = 3 << gpio.num * 2;
			data &= ~mask;
			data |= (1 << gpio.num * 2);
		}

		ret = target_write_u32(target, gpio_con, data);
	}
	return ret;
}

static int s3c2440_set_gpio_output_val (mflash_gpio_num_t gpio, u8 val)
{
	u32 data, mask, gpio_dat;
	target_t *target = mflash_bank->target;
	int ret;

	if (gpio.port[0] >= 'a' && gpio.port[0] <= 'h') {
		gpio_dat = S3C2440_GPADAT + (gpio.port[0] - 'a') * 0x10;
	} else if (gpio.port[0] == 'j') {
		gpio_dat = S3C2440_GPJDAT;
	} else {
		LOG_ERROR("invalid port %d%s", gpio.num, gpio.port);
		return ERROR_INVALID_ARGUMENTS;
	}

	ret = target_read_u32(target, gpio_dat, &data);

	if (ret == ERROR_OK) {
		mask = 1 << gpio.num;
		if (val)
			data |= mask;
		else
			data &= ~mask;

		ret = target_write_u32(target, gpio_dat, data);
	}
	return ret;
}

static int mflash_rst(u8 level)
{
	return mflash_bank->gpio_drv->set_gpio_output_val(mflash_bank->rst_pin, level);
}

static int mflash_init_gpio (void)
{
	mflash_gpio_drv_t *gpio_drv = mflash_bank->gpio_drv;

	gpio_drv->set_gpio_to_output(mflash_bank->rst_pin);
	gpio_drv->set_gpio_output_val(mflash_bank->rst_pin, 1);

	if (mflash_bank->wp_pin.num != -1) {
		gpio_drv->set_gpio_to_output(mflash_bank->wp_pin);
		gpio_drv->set_gpio_output_val(mflash_bank->wp_pin, 1);
	}

	if (mflash_bank->dpd_pin.num != -1) {
		gpio_drv->set_gpio_to_output(mflash_bank->dpd_pin);
		gpio_drv->set_gpio_output_val(mflash_bank->dpd_pin, 1);
	}

	return ERROR_OK;
}

static int mg_dsk_wait(mg_io_type_wait wait, u32 time)
{
	u8 status, error;
	target_t *target = mflash_bank->target;
	u32 mg_task_reg = mflash_bank->base + MG_REG_OFFSET;
	duration_t duration;
	long long t=0;

	duration_start_measure(&duration);

	while (time) {

		target_read_u8(target, mg_task_reg + MG_REG_STATUS, &status);

		if (status & mg_io_rbit_status_busy)
		{
			if (wait == mg_io_wait_bsy)
				return ERROR_OK;
		} else {
			switch(wait)
			{
				case mg_io_wait_not_bsy:
					return ERROR_OK;
				case mg_io_wait_rdy_noerr:
					if (status & mg_io_rbit_status_ready)
						return ERROR_OK;
					break;
				case mg_io_wait_drq_noerr:
					if (status & mg_io_rbit_status_data_req)
						return ERROR_OK;
					break;
				default:
					break;
			}

			/* Now we check the error condition! */
			if (status & mg_io_rbit_status_error)
			{
				target_read_u8(target, mg_task_reg + MG_REG_ERROR, &error);

				if (error & mg_io_rbit_err_bad_sect_num) {
					LOG_ERROR("sector not found");
					return ERROR_FAIL;
				}
				else if (error & (mg_io_rbit_err_bad_block | mg_io_rbit_err_uncorrectable)) {
					LOG_ERROR("bad block");
					return ERROR_FAIL;
				} else {
					LOG_ERROR("disk operation fail");
					return ERROR_FAIL;
				}
			}

			switch (wait)
			{
				case mg_io_wait_rdy:
					if (status & mg_io_rbit_status_ready)
						return ERROR_OK;

				case mg_io_wait_drq:
					if (status & mg_io_rbit_status_data_req)
						return ERROR_OK;

				default:
					break;
			}
		}

		duration_stop_measure(&duration, NULL);

		t=duration.duration.tv_usec/1000;
		t+=duration.duration.tv_sec*1000;

		if (t > time)
			break;
	}

	LOG_ERROR("timeout occured");
	return ERROR_FAIL;
}

static int mg_dsk_srst(u8 on)
{
	target_t *target = mflash_bank->target;
	u32 mg_task_reg = mflash_bank->base + MG_REG_OFFSET;
	u8 value;
	int ret;

	if ((ret = target_read_u8(target, mg_task_reg + MG_REG_DRV_CTRL, &value)) != ERROR_OK)
		return ret;

	if(on) {
		value |= (mg_io_rbit_devc_srst);
	} else {
		value &= ~mg_io_rbit_devc_srst;
	}

	ret = target_write_u8(target, mg_task_reg + MG_REG_DRV_CTRL, value);
	return ret;
}

static int mg_dsk_io_cmd(u32 sect_num, u32 cnt, u8 cmd)
{
	target_t *target = mflash_bank->target;
	u32 mg_task_reg = mflash_bank->base + MG_REG_OFFSET;
	u8 value;

	if (mg_dsk_wait(mg_io_wait_rdy_noerr, MG_OEM_DISK_WAIT_TIME_NORMAL) != ERROR_OK)
		return ERROR_FAIL;

	value = mg_io_rval_dev_drv_master | mg_io_rval_dev_lba_mode |((sect_num >> 24) & 0xf);

	target_write_u8(target, mg_task_reg + MG_REG_DRV_HEAD, value);
	target_write_u8(target, mg_task_reg + MG_REG_SECT_CNT, (u8)cnt);
	target_write_u8(target, mg_task_reg + MG_REG_SECT_NUM, (u8)sect_num);
	target_write_u8(target, mg_task_reg + MG_REG_CYL_LOW, (u8)(sect_num >> 8));
	target_write_u8(target, mg_task_reg + MG_REG_CYL_HIGH, (u8)(sect_num >> 16));

	target_write_u8(target, mg_task_reg + MG_REG_COMMAND, cmd);

	return ERROR_OK;
}

static int mg_dsk_drv_info(void)
{
	target_t *target = mflash_bank->target;
	u32 mg_buff = mflash_bank->base + MG_BUFFER_OFFSET;

	if ( mg_dsk_io_cmd(0, 1, mg_io_cmd_identify) != ERROR_OK)
		return ERROR_FAIL;

	if ( mg_dsk_wait(mg_io_wait_drq, MG_OEM_DISK_WAIT_TIME_NORMAL) != ERROR_OK)
		return ERROR_FAIL;

	LOG_INFO("read drive info.");

	if (! mflash_bank->drv_info)
		mflash_bank->drv_info = malloc(sizeof(mg_drv_info_t));

	target->type->read_memory(target, mg_buff, 2, sizeof(mg_io_type_drv_info) >> 1,
			(u8 *)&mflash_bank->drv_info->drv_id);

	mflash_bank->drv_info->tot_sects = (u32)(mflash_bank->drv_info->drv_id.total_user_addressable_sectors_hi << 16)
									+ mflash_bank->drv_info->drv_id.total_user_addressable_sectors_lo;

	target_write_u8(target, mflash_bank->base + MG_REG_OFFSET + MG_REG_COMMAND, mg_io_cmd_confirm_read);

	return ERROR_OK;
}

static int mg_mflash_probe(void)
{
	mflash_bank->proved = 0;

	mflash_init_gpio();

	LOG_INFO("reset mflash");

	mflash_rst(0);

	if (mg_dsk_wait(mg_io_wait_bsy, MG_OEM_DISK_WAIT_TIME_LONG) != ERROR_OK)
		return ERROR_FAIL;

	mflash_rst(1);

	if (mg_dsk_wait(mg_io_wait_not_bsy, MG_OEM_DISK_WAIT_TIME_LONG) != ERROR_OK)
		return ERROR_FAIL;

	mg_dsk_srst(1);

	if (mg_dsk_wait(mg_io_wait_bsy, MG_OEM_DISK_WAIT_TIME_LONG) != ERROR_OK)
		return ERROR_FAIL;

	mg_dsk_srst(0);

	if (mg_dsk_wait(mg_io_wait_not_bsy, MG_OEM_DISK_WAIT_TIME_LONG) != ERROR_OK)
		return ERROR_FAIL;

	if (mg_dsk_drv_info() != ERROR_OK)
		return ERROR_FAIL;

	mflash_bank->proved = 1;

	return ERROR_OK;
}

static int mflash_probe_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int ret;

	ret = mg_mflash_probe();

	if (ret == ERROR_OK) {
		command_print(cmd_ctx, "mflash (total %u sectors) found at 0x%8.8x",
				mflash_bank->drv_info->tot_sects, mflash_bank->base );
	}

	return ret;
}

static int mg_mflash_do_read_sects(void *buff, u32 sect_num, u32 sect_cnt)
{
	u32 i, address;
	int ret;
	target_t *target = mflash_bank->target;
	u8 *buff_ptr = buff;
	duration_t duration;

	if ( mg_dsk_io_cmd(sect_num, sect_cnt, mg_io_cmd_read) != ERROR_OK )
		return ERROR_FAIL;

	address = mflash_bank->base + MG_BUFFER_OFFSET;

	duration_start_measure(&duration);

	for (i = 0; i < sect_cnt; i++) {
		mg_dsk_wait(mg_io_wait_drq, MG_OEM_DISK_WAIT_TIME_NORMAL);

		target->type->read_memory(target, address, 2, MG_MFLASH_SECTOR_SIZE / 2, buff_ptr);
		buff_ptr += MG_MFLASH_SECTOR_SIZE;

		target_write_u8(target, mflash_bank->base + MG_REG_OFFSET + MG_REG_COMMAND, mg_io_cmd_confirm_read);

		LOG_DEBUG("%u (0x%8.8x) sector read", sect_num + i, (sect_num + i) * MG_MFLASH_SECTOR_SIZE);

		duration_stop_measure(&duration, NULL);

		if ((duration.duration.tv_sec * 1000 + duration.duration.tv_usec / 1000) > 3000) {
			LOG_INFO("read %u'th sectors", sect_num + i);
			duration_start_measure(&duration);
		}
	}

	ret = mg_dsk_wait(mg_io_wait_rdy, MG_OEM_DISK_WAIT_TIME_NORMAL);

	return ret;
}

static int mg_mflash_read_sects(void *buff, u32 sect_num, u32 sect_cnt)
{
	u32 quotient, residue, i;
	u8 *buff_ptr = buff;

	quotient = sect_cnt >> 8;
	residue = sect_cnt % 256;

	for (i = 0; i < quotient; i++) {
		LOG_DEBUG("sect num : %u buff : 0x%8.8x", sect_num, (u32)buff_ptr);
		mg_mflash_do_read_sects(buff_ptr, sect_num, 256);
		sect_num += 256;
		buff_ptr += 256 * MG_MFLASH_SECTOR_SIZE;
	}

	if (residue) {
		LOG_DEBUG("sect num : %u buff : %8.8x", sect_num, (u32)buff_ptr);
		mg_mflash_do_read_sects(buff_ptr, sect_num, residue);
	}

	return ERROR_OK;
}

static int mg_mflash_do_write_sects(void *buff, u32 sect_num, u32 sect_cnt)
{
	u32 i, address;
	int ret;
	target_t *target = mflash_bank->target;
	u8 *buff_ptr = buff;
	duration_t duration;

	if ( mg_dsk_io_cmd(sect_num, sect_cnt, mg_io_cmd_write) != ERROR_OK ) {
		LOG_ERROR("mg_io_cmd_write fail");
		return ERROR_FAIL;
	}

	address = mflash_bank->base + MG_BUFFER_OFFSET;

	duration_start_measure(&duration);

	for (i = 0; i < sect_cnt; i++) {
		ret = mg_dsk_wait(mg_io_wait_drq, MG_OEM_DISK_WAIT_TIME_NORMAL);
		if (ret != ERROR_OK)
			LOG_ERROR("mg_io_wait_drq time out");

		ret = target->type->write_memory(target, address, 2, MG_MFLASH_SECTOR_SIZE / 2, buff_ptr);
		if (ret != ERROR_OK)
			LOG_ERROR("mem write error");
		buff_ptr += MG_MFLASH_SECTOR_SIZE;

		ret = target_write_u8(target, mflash_bank->base + MG_REG_OFFSET + MG_REG_COMMAND, mg_io_cmd_confirm_write);
		if (ret != ERROR_OK)
			LOG_ERROR("mg_io_cmd_confirm_write error");

		LOG_DEBUG("%u (0x%8.8x) sector write", sect_num + i, (sect_num + i) * MG_MFLASH_SECTOR_SIZE);

		duration_stop_measure(&duration, NULL);

		if ((duration.duration.tv_sec * 1000 + duration.duration.tv_usec / 1000) > 3000) {
			LOG_INFO("wrote %u'th sectors", sect_num + i);
			duration_start_measure(&duration);
		}
	}

	ret = mg_dsk_wait(mg_io_wait_rdy, MG_OEM_DISK_WAIT_TIME_NORMAL);

	return ret;
}

static int mg_mflash_write_sects(void *buff, u32 sect_num, u32 sect_cnt)
{
	u32 quotient, residue, i;
	u8 *buff_ptr = buff;

	quotient = sect_cnt >> 8;
	residue = sect_cnt % 256;

	for (i = 0; i < quotient; i++) {
		LOG_DEBUG("sect num : %u buff : %8.8x", sect_num, (u32)buff_ptr);
		mg_mflash_do_write_sects(buff_ptr, sect_num, 256);
		sect_num += 256;
		buff_ptr += 256 * MG_MFLASH_SECTOR_SIZE;
	}

	if (residue) {
		LOG_DEBUG("sect num : %u buff : %8.8x", sect_num, (u32)buff_ptr);
		mg_mflash_do_write_sects(buff_ptr, sect_num, residue);
	}

	return ERROR_OK;
}

static int mg_mflash_read (u32 addr, u8 *buff, u32 len)
{
	u8 *sect_buff, *buff_ptr = buff;
	u32 cur_addr, next_sec_addr, end_addr, cnt, sect_num;

	cnt = 0;
	cur_addr = addr;
	end_addr = addr + len;

	sect_buff = malloc(MG_MFLASH_SECTOR_SIZE);

	if (cur_addr & MG_MFLASH_SECTOR_SIZE_MASK) {

		next_sec_addr = (cur_addr + MG_MFLASH_SECTOR_SIZE) & ~MG_MFLASH_SECTOR_SIZE_MASK;
		sect_num = cur_addr >> MG_MFLASH_SECTOR_SIZE_SHIFT;
		mg_mflash_read_sects(sect_buff, sect_num, 1);

		if (end_addr < next_sec_addr) {
			memcpy(buff_ptr, sect_buff + (cur_addr & MG_MFLASH_SECTOR_SIZE_MASK), end_addr - cur_addr);
			LOG_DEBUG("copies %u byte from sector offset 0x%8.8x", end_addr - cur_addr, cur_addr);
			cur_addr = end_addr;
		} else {
			memcpy(buff_ptr, sect_buff + (cur_addr & MG_MFLASH_SECTOR_SIZE_MASK), next_sec_addr - cur_addr);
			LOG_DEBUG("copies %u byte from sector offset 0x%8.8x", next_sec_addr - cur_addr, cur_addr);
			buff_ptr += (next_sec_addr - cur_addr);
			cur_addr = next_sec_addr;
		}
	}

	if (cur_addr < end_addr) {

		sect_num = cur_addr >> MG_MFLASH_SECTOR_SIZE_SHIFT;
		next_sec_addr = cur_addr + MG_MFLASH_SECTOR_SIZE;

		while (next_sec_addr <= end_addr) {
			cnt++;
			next_sec_addr += MG_MFLASH_SECTOR_SIZE;
		}

		if (cnt)
			mg_mflash_read_sects(buff_ptr, sect_num, cnt);

		buff_ptr += cnt * MG_MFLASH_SECTOR_SIZE;
		cur_addr += cnt * MG_MFLASH_SECTOR_SIZE;

		if (cur_addr < end_addr) {

			sect_num = cur_addr >> MG_MFLASH_SECTOR_SIZE_SHIFT;
			mg_mflash_read_sects(sect_buff, sect_num, 1);
			memcpy(buff_ptr, sect_buff, end_addr - cur_addr);
			LOG_DEBUG("copies %u byte", end_addr - cur_addr);

		}
	}

	free(sect_buff);

	return ERROR_OK;
}

static int mg_mflash_write(u32 addr, u8 *buff, u32 len)
{
	u8 *sect_buff, *buff_ptr = buff;
	u32 cur_addr, next_sec_addr, end_addr, cnt, sect_num;

	cnt = 0;
	cur_addr = addr;
	end_addr = addr + len;

	sect_buff = malloc(MG_MFLASH_SECTOR_SIZE);

	if (cur_addr & MG_MFLASH_SECTOR_SIZE_MASK) {

		next_sec_addr = (cur_addr + MG_MFLASH_SECTOR_SIZE) & ~MG_MFLASH_SECTOR_SIZE_MASK;
		sect_num = cur_addr >> MG_MFLASH_SECTOR_SIZE_SHIFT;
		mg_mflash_read_sects(sect_buff, sect_num, 1);

		if (end_addr < next_sec_addr) {
			memcpy(sect_buff + (cur_addr & MG_MFLASH_SECTOR_SIZE_MASK), buff_ptr, end_addr - cur_addr);
			LOG_DEBUG("copies %u byte to sector offset 0x%8.8x", end_addr - cur_addr, cur_addr);
			cur_addr = end_addr;
		} else {
			memcpy(sect_buff + (cur_addr & MG_MFLASH_SECTOR_SIZE_MASK), buff_ptr, next_sec_addr - cur_addr);
			LOG_DEBUG("copies %u byte to sector offset 0x%8.8x", next_sec_addr - cur_addr, cur_addr);
			buff_ptr += (next_sec_addr - cur_addr);
			cur_addr = next_sec_addr;
		}

		mg_mflash_write_sects(sect_buff, sect_num, 1);
	}

	if (cur_addr < end_addr) {

		sect_num = cur_addr >> MG_MFLASH_SECTOR_SIZE_SHIFT;
		next_sec_addr = cur_addr + MG_MFLASH_SECTOR_SIZE;

		while (next_sec_addr <= end_addr) {
			cnt++;
			next_sec_addr += MG_MFLASH_SECTOR_SIZE;
		}

		if (cnt)
			mg_mflash_write_sects(buff_ptr, sect_num, cnt);

		buff_ptr += cnt * MG_MFLASH_SECTOR_SIZE;
		cur_addr += cnt * MG_MFLASH_SECTOR_SIZE;

		if (cur_addr < end_addr) {

			sect_num = cur_addr >> MG_MFLASH_SECTOR_SIZE_SHIFT;
			mg_mflash_read_sects(sect_buff, sect_num, 1);
			memcpy(sect_buff, buff_ptr, end_addr - cur_addr);
			LOG_DEBUG("copies %u byte", end_addr - cur_addr);
			mg_mflash_write_sects(sect_buff, sect_num, 1);
		}
	}

	free(sect_buff);

	return ERROR_OK;
}

static int mflash_write_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u32 address, buf_cnt;
	u8 *buffer;
	/* TODO : multi-bank support, large file support */
	fileio_t fileio;
	duration_t duration;
	char *duration_text;
	int ret;

	if (argc != 3) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	address = strtoul(args[2], NULL, 0);

	if (! mflash_bank->proved ) {
		mg_mflash_probe();
	}

	if (fileio_open(&fileio, args[1], FILEIO_READ, FILEIO_BINARY) != ERROR_OK) {
		return ERROR_FAIL;
	}

	buffer = malloc(fileio.size);

	if (fileio_read(&fileio, fileio.size, buffer, &buf_cnt) != ERROR_OK)
	{
		free(buffer);
		fileio_close(&fileio);
		return ERROR_FAIL;
	}

	duration_start_measure(&duration);

	ret = mg_mflash_write(address, buffer, (u32)fileio.size);

	duration_stop_measure(&duration, &duration_text);

	command_print(cmd_ctx, "wrote %lli byte from file %s in %s (%f kB/s)",
		fileio.size, args[1], duration_text,
		(float)fileio.size / 1024.0 / ((float)duration.duration.tv_sec + ((float)duration.duration.tv_usec / 1000000.0)));

	free(duration_text);

	fileio_close(&fileio);

	free(buffer);

	return ERROR_OK;
}

static int mflash_dump_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u32 address, size_written, size;
	u8 *buffer;
	/* TODO : multi-bank support */
	fileio_t fileio;
	duration_t duration;
	char *duration_text;

	if (argc != 4) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	address = strtoul(args[2], NULL, 0);
	size = strtoul(args[3], NULL, 0);

	if (! mflash_bank->proved ) {
			mg_mflash_probe();
	}

	if (fileio_open(&fileio, args[1], FILEIO_WRITE, FILEIO_BINARY) != ERROR_OK) {
		return ERROR_FAIL;
	}

	buffer = malloc(size);

	duration_start_measure(&duration);

	mg_mflash_read(address, buffer, size);

	duration_stop_measure(&duration, &duration_text);

	fileio_write(&fileio, size, buffer, &size_written);

	command_print(cmd_ctx, "dump image (address 0x%8.8x size %u) to file %s in %s (%f kB/s)",
				address, size, args[1], duration_text,
				(float)size / 1024.0 / ((float)duration.duration.tv_sec + ((float)duration.duration.tv_usec / 1000000.0)));

	free(duration_text);

	fileio_close(&fileio);

	free(buffer);

	return ERROR_OK;
}

int mflash_init_drivers(struct command_context_s *cmd_ctx)
{
	if (mflash_bank) {
		register_command(cmd_ctx, mflash_cmd, "probe", mflash_probe_command, COMMAND_EXEC, NULL);
		register_command(cmd_ctx, mflash_cmd, "write", mflash_write_command, COMMAND_EXEC,
				"mflash write <num> <file> <address>");
		register_command(cmd_ctx, mflash_cmd, "dump", mflash_dump_command, COMMAND_EXEC,
						"mflash dump <num> <file> <address> <size>");
	}

	return ERROR_OK;
}

static int mflash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	char *str;
	int i;

	if (argc < 8)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if ((target = get_target_by_num(strtoul(args[7], NULL, 0))) == NULL)
	{
		LOG_ERROR("target %lu not defined", strtoul(args[7], NULL, 0));
		return ERROR_FAIL;
	}

	mflash_bank = calloc(sizeof(mflash_bank_t), 1);
	mflash_bank->base = strtoul(args[1], NULL, 0);
	mflash_bank->chip_width = strtoul(args[2], NULL, 0);
	mflash_bank->bus_width = strtoul(args[3], NULL, 0);
	mflash_bank->rst_pin.num = strtoul(args[4], &str, 0);
	if (*str)
		mflash_bank->rst_pin.port[0] = (u16)tolower(str[0]);
	mflash_bank->wp_pin.num = strtol(args[5], &str, 0);
	if (*str)
		mflash_bank->wp_pin.port[0] = (u16)tolower(str[0]);
	mflash_bank->dpd_pin.num = strtol(args[6], &str, 0);
	if (*str)
		mflash_bank->dpd_pin.port[0] = (u16)tolower(str[0]);

	mflash_bank->target = target;

	for (i = 0; mflash_gpio[i] ; i++) {
		if (! strcmp(mflash_gpio[i]->name, args[0])) {
			mflash_bank->gpio_drv = mflash_gpio[i];
		}
	}

	if (! mflash_bank->gpio_drv) {
		LOG_ERROR("%s is unsupported soc", args[0]);
		return ERROR_INVALID_ARGUMENTS;
	}

	return ERROR_OK;
}

int mflash_register_commands(struct command_context_s *cmd_ctx)
{
	mflash_cmd = register_command(cmd_ctx, NULL, "mflash", NULL, COMMAND_ANY, NULL);
	register_command(cmd_ctx, mflash_cmd, "bank", mflash_bank_command, COMMAND_CONFIG,
			"mflash bank <soc> <base> <chip_width> <bus_width> <RST pin> <WP pin> <DPD pin> <target #>");
	return ERROR_OK;
}
