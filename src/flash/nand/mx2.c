/***************************************************************************
 *   Copyright (C) 2009 by Alexei Babich                                   *
 *   Rezonans plc., Chelyabinsk, Russia                                    *
 *   impatt@mail.ru                                                        *
 *                                                                         *
 *   Copyright (C) 2010 by Gaetan CARLIER                                  *
 *   Trump s.a., Belgium                                                   *
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

/*
 * Freescale iMX OpenOCD NAND Flash controller support.
 * based on Freescale iMX2* and iMX3* OpenOCD NAND Flash controller support.
 */

/*
 * driver tested with Samsung K9F2G08UXA and Numonyx/ST NAND02G-B2D @mxc
 * tested "nand probe #", "nand erase # 0 #", "nand dump # file 0 #",
 * "nand write # file 0", "nand verify"
 *
 * get_next_halfword_from_sram_buffer() not tested
 * !! all function only tested with 2k page nand device; mxc_write_page
 *    writes the 4 MAIN_BUFFER's and is not compatible with < 2k page
 * !! oob must be be used due to NFS bug
*/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "mx2.h"
#include <target/target.h>

/* This permits to print (in LOG_INFO) how much bytes
 * has been written after a page read or write.
 * This is useful when OpenOCD is used with a graphical
 * front-end to estimate progression of the global read/write
 */
#undef _MXC_PRINT_STAT
/* #define _MXC_PRINT_STAT */

static const char target_not_halted_err_msg[] =
	"target must be halted to use mxc NAND flash controller";
static const char data_block_size_err_msg[] =
	"minimal granularity is one half-word, %" PRId32 " is incorrect";
static const char sram_buffer_bounds_err_msg[] =
	"trying to access out of SRAM buffer bound (addr=0x%" PRIx32 ")";
static const char get_status_register_err_msg[] = "can't get NAND status";
static uint32_t in_sram_address;
static unsigned char sign_of_sequental_byte_read;

static int initialize_nf_controller(struct nand_device *nand);
static int get_next_byte_from_sram_buffer(struct nand_device *nand, uint8_t *value);
static int get_next_halfword_from_sram_buffer(struct nand_device *nand, uint16_t *value);
static int poll_for_complete_op(struct nand_device *nand, const char *text);
static int validate_target_state(struct nand_device *nand);
static int do_data_output(struct nand_device *nand);

static int mxc_command(struct nand_device *nand, uint8_t command);
static int mxc_address(struct nand_device *nand, uint8_t address);

NAND_DEVICE_COMMAND_HANDLER(mxc_nand_device_command)
{
	struct mxc_nf_controller *mxc_nf_info;
	int hwecc_needed;
	int x;

	mxc_nf_info = malloc(sizeof(struct mxc_nf_controller));
	if (mxc_nf_info == NULL) {
		LOG_ERROR("no memory for nand controller");
		return ERROR_FAIL;
	}
	nand->controller_priv = mxc_nf_info;

	if (CMD_ARGC < 4) {
		LOG_ERROR("use \"nand device mxc target mx27|mx31|mx35 noecc|hwecc [biswap]\"");
		return ERROR_FAIL;
	}

	/*
	 * check board type
	 */
	if (strcmp(CMD_ARGV[2], "mx27") == 0)
		mxc_nf_info->mxc_base_addr = 0xD8000000;
	else if (strcmp(CMD_ARGV[2], "mx31") == 0)
		mxc_nf_info->mxc_base_addr = 0xB8000000;
	else if (strcmp(CMD_ARGV[2], "mx35") == 0)
		mxc_nf_info->mxc_base_addr = 0xBB000000;

	/*
	 * check hwecc requirements
	 */
	hwecc_needed = strcmp(CMD_ARGV[3], "hwecc");
	if (hwecc_needed == 0)
		mxc_nf_info->flags.hw_ecc_enabled = 1;
	else
		mxc_nf_info->flags.hw_ecc_enabled = 0;

	mxc_nf_info->optype = MXC_NF_DATAOUT_PAGE;
	mxc_nf_info->fin = MXC_NF_FIN_NONE;
	mxc_nf_info->flags.target_little_endian =
	(nand->target->endianness == TARGET_LITTLE_ENDIAN);

	/*
	 * should factory bad block indicator be swaped
	 * as a workaround for how the nfc handles pages.
	 */
	if (CMD_ARGC > 4 && strcmp(CMD_ARGV[4], "biswap") == 0) {
		LOG_DEBUG("BI-swap enabled");
		mxc_nf_info->flags.biswap_enabled = 1;
	}

	/*
	 * testing host endianness
	 */
	x = 1;
	if (*(char *) &x == 1)
		mxc_nf_info->flags.host_little_endian = 1;
	else
		mxc_nf_info->flags.host_little_endian = 0;
	return ERROR_OK;
}

static int mxc_init(struct nand_device *nand)
{
	struct mxc_nf_controller *mxc_nf_info = nand->controller_priv;
	struct target *target = nand->target;

	int validate_target_result;
	uint16_t buffsize_register_content;
	uint32_t pcsr_register_content;
	int retval;
	uint16_t nand_status_content;
	/*
	 * validate target state
	 */
	validate_target_result = validate_target_state(nand);
	if (validate_target_result != ERROR_OK)
		return validate_target_result;

	target_read_u16(target, MXC_NF_BUFSIZ, &buffsize_register_content);
	mxc_nf_info->flags.one_kb_sram = !(buffsize_register_content & 0x000f);

	target_read_u32(target, MXC_FMCR, &pcsr_register_content);
	if (!nand->bus_width) {
		/* bus_width not yet defined. Read it from MXC_FMCR */
		nand->bus_width =
			(pcsr_register_content & MXC_FMCR_NF_16BIT_SEL) ? 16 : 8;
	} else {
		/* bus_width forced in soft. Sync it to MXC_FMCR */
		pcsr_register_content |=
			((nand->bus_width == 16) ? MXC_FMCR_NF_16BIT_SEL : 0x00000000);
		target_write_u32(target, MXC_FMCR, pcsr_register_content);
	}
	if (nand->bus_width == 16)
		LOG_DEBUG("MXC_NF : bus is 16-bit width");
	else
		LOG_DEBUG("MXC_NF : bus is 8-bit width");

	if (!nand->page_size) {
		nand->page_size = (pcsr_register_content & MXC_FMCR_NF_FMS) ? 2048 : 512;
	} else {
		pcsr_register_content |=
			((nand->page_size == 2048) ? MXC_FMCR_NF_FMS : 0x00000000);
		target_write_u32(target, MXC_FMCR, pcsr_register_content);
	}
	if (mxc_nf_info->flags.one_kb_sram && (nand->page_size == 2048)) {
		LOG_ERROR("NAND controller have only 1 kb SRAM, so "
		          "pagesize 2048 is incompatible with it");
	} else {
		LOG_DEBUG("MXC_NF : NAND controller can handle pagesize of 2048");
	}

	initialize_nf_controller(nand);

	retval = ERROR_OK;
	retval |= mxc_command(nand, NAND_CMD_STATUS);
	retval |= mxc_address(nand, 0x00);
	retval |= do_data_output(nand);
	if (retval != ERROR_OK) {
		LOG_ERROR(get_status_register_err_msg);
		return ERROR_FAIL;
	}
	target_read_u16(target, MXC_NF_MAIN_BUFFER0, &nand_status_content);
	if (!(nand_status_content & 0x0080)) {
		LOG_INFO("NAND read-only");
		mxc_nf_info->flags.nand_readonly = 1;
	} else {
		mxc_nf_info->flags.nand_readonly = 0;
	}
	return ERROR_OK;
}

static int mxc_read_data(struct nand_device *nand, void *data)
{
	int validate_target_result;
	int try_data_output_from_nand_chip;
	/*
	 * validate target state
	 */
	validate_target_result = validate_target_state(nand);
	if (validate_target_result != ERROR_OK)
		return validate_target_result;

	/*
	 * get data from nand chip
	 */
	try_data_output_from_nand_chip = do_data_output(nand);
	if (try_data_output_from_nand_chip != ERROR_OK) {
		LOG_ERROR("mxc_read_data : read data failed : '%x'",
				  try_data_output_from_nand_chip);
		return try_data_output_from_nand_chip;
	}

	if (nand->bus_width == 16)
		get_next_halfword_from_sram_buffer(nand, data);
	else
		get_next_byte_from_sram_buffer(nand, data);

	return ERROR_OK;
}

static int mxc_write_data(struct nand_device *nand, uint16_t data)
{
	LOG_ERROR("write_data() not implemented");
	return ERROR_NAND_OPERATION_FAILED;
}

static int mxc_reset(struct nand_device *nand)
{
	/*
	 * validate target state
	 */
	int validate_target_result;
	validate_target_result = validate_target_state(nand);
	if (validate_target_result != ERROR_OK)
		return validate_target_result;
	initialize_nf_controller(nand);
	return ERROR_OK;
}

static int mxc_command(struct nand_device *nand, uint8_t command)
{
	struct mxc_nf_controller *mxc_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	int validate_target_result;
	int poll_result;
	/*
	 * validate target state
	 */
	validate_target_result = validate_target_state(nand);
	if (validate_target_result != ERROR_OK)
		return validate_target_result;

	switch (command) {
	case NAND_CMD_READOOB:
		command = NAND_CMD_READ0;
		/* set read point for data_read() and read_block_data() to
		 * spare area in SRAM buffer
		 */
		in_sram_address = MXC_NF_SPARE_BUFFER0;
		break;
	case NAND_CMD_READ1:
		command = NAND_CMD_READ0;
		/*
		 * offset == one half of page size
		 */
		in_sram_address = MXC_NF_MAIN_BUFFER0 + (nand->page_size >> 1);
		break;
	default:
		in_sram_address = MXC_NF_MAIN_BUFFER0;
		break;
	}

	target_write_u16(target, MXC_NF_FCMD, command);
	/*
	 * start command input operation (set MXC_NF_BIT_OP_DONE==0)
	 */
	target_write_u16(target, MXC_NF_CFG2, MXC_NF_BIT_OP_FCI);
	poll_result = poll_for_complete_op(nand, "command");
	if (poll_result != ERROR_OK)
		return poll_result;
	/*
	 * reset cursor to begin of the buffer
	 */
	sign_of_sequental_byte_read = 0;
	/* Handle special read command and adjust NF_CFG2(FDO) */
	switch (command) {
	case NAND_CMD_READID:
		mxc_nf_info->optype = MXC_NF_DATAOUT_NANDID;
		mxc_nf_info->fin = MXC_NF_FIN_DATAOUT;
		break;
	case NAND_CMD_STATUS:
		mxc_nf_info->optype = MXC_NF_DATAOUT_NANDSTATUS;
		mxc_nf_info->fin = MXC_NF_FIN_DATAOUT;
		target_write_u16 (target, MXC_NF_BUFADDR, 0);
		in_sram_address = 0;
		break;
	case NAND_CMD_READ0:
		mxc_nf_info->fin = MXC_NF_FIN_DATAOUT;
		mxc_nf_info->optype = MXC_NF_DATAOUT_PAGE;
		break;
	default:
		/* Ohter command use the default 'One page data out' FDO */
		mxc_nf_info->optype = MXC_NF_DATAOUT_PAGE;
		break;
	}
	return ERROR_OK;
}

static int mxc_address(struct nand_device *nand, uint8_t address)
{
	struct mxc_nf_controller *mxc_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	int validate_target_result;
	int poll_result;
	/*
	 * validate target state
	 */
	validate_target_result = validate_target_state(nand);
	if (validate_target_result != ERROR_OK)
		return validate_target_result;

	target_write_u16(target, MXC_NF_FADDR, address);
	/*
	 * start address input operation (set MXC_NF_BIT_OP_DONE==0)
	 */
	target_write_u16(target, MXC_NF_CFG2, MXC_NF_BIT_OP_FAI);
	poll_result = poll_for_complete_op(nand, "address");
	if (poll_result != ERROR_OK)
		return poll_result;

	return ERROR_OK;
}

static int mxc_nand_ready(struct nand_device *nand, int tout)
{
	struct mxc_nf_controller *mxc_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	uint16_t poll_complete_status;
	int validate_target_result;

	/*
	 * validate target state
	 */
	validate_target_result = validate_target_state(nand);
	if (validate_target_result != ERROR_OK)
		return validate_target_result;

	do {
		target_read_u16(target, MXC_NF_CFG2, &poll_complete_status);
		if (poll_complete_status & MXC_NF_BIT_OP_DONE)
			return tout;

		alive_sleep(1);
	}
	while (tout-- > 0);
	return tout;
}

static int mxc_write_page(struct nand_device *nand, uint32_t page,
							uint8_t *data, uint32_t data_size,
							uint8_t *oob, uint32_t oob_size)
{
	struct mxc_nf_controller *mxc_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	int retval;
	uint16_t nand_status_content;
	uint16_t swap1, swap2, new_swap1;
	int poll_result;
	if (data_size % 2) {
		LOG_ERROR(data_block_size_err_msg, data_size);
		return ERROR_NAND_OPERATION_FAILED;
	}
	if (oob_size % 2) {
		LOG_ERROR(data_block_size_err_msg, oob_size);
		return ERROR_NAND_OPERATION_FAILED;
	}
	if (!data) {
		LOG_ERROR("nothing to program");
		return ERROR_NAND_OPERATION_FAILED;
	}
	/*
	 * validate target state
	 */
	retval = validate_target_state(nand);
	if (retval != ERROR_OK)
		return retval;

	in_sram_address = MXC_NF_MAIN_BUFFER0;
	sign_of_sequental_byte_read = 0;
	retval = ERROR_OK;
	retval |= mxc_command(nand, NAND_CMD_SEQIN);
	retval |= mxc_address(nand, 0); /* col */
	retval |= mxc_address(nand, 0); /* col */
	retval |= mxc_address(nand, page & 0xff); /* page address */
	retval |= mxc_address(nand, (page >> 8) & 0xff); /* page address */
	retval |= mxc_address(nand, (page >> 16) & 0xff); /* page address */

	target_write_buffer(target, MXC_NF_MAIN_BUFFER0, data_size, data);
	if (oob) {
		if (mxc_nf_info->flags.hw_ecc_enabled) {
			/*
			 * part of spare block will be overrided by hardware
			 * ECC generator
			 */
			LOG_DEBUG("part of spare block will be overrided "
			          "by hardware ECC generator");
		}
		target_write_buffer(target, MXC_NF_SPARE_BUFFER0, oob_size,	oob);
	}

	if (nand->page_size > 512 && mxc_nf_info->flags.biswap_enabled) {
		/* BI-swap - work-around of i.MX NFC for NAND device with page == 2kb*/
		target_read_u16(target, MXC_NF_MAIN_BUFFER3 + 464, &swap1);
		if (oob) {
			LOG_ERROR("Due to NFC Bug, oob is not correctly implemented in mxc driver");
			return ERROR_NAND_OPERATION_FAILED;
		}
		swap2 = 0xffff;  /* Spare buffer unused forced to 0xffff */
		new_swap1 = (swap1 & 0xFF00) | (swap2 >> 8);
		swap2 = (swap1 << 8) | (swap2 & 0xFF);
		target_write_u16(target, MXC_NF_MAIN_BUFFER3 + 464, new_swap1);
		target_write_u16(target, MXC_NF_SPARE_BUFFER3 + 4, swap2);
	}

	/*
	 * start data input operation (set MXC_NF_BIT_OP_DONE==0)
	 */
	target_write_u16(target, MXC_NF_BUFADDR, 0);
	target_write_u16(target, MXC_NF_CFG2, MXC_NF_BIT_OP_FDI);
	poll_result = poll_for_complete_op(nand, "data input");
	if (poll_result != ERROR_OK)
		return poll_result;

	target_write_u16(target, MXC_NF_BUFADDR, 1);
	target_write_u16(target, MXC_NF_CFG2, MXC_NF_BIT_OP_FDI);
	poll_result = poll_for_complete_op(nand, "data input");
	if (poll_result != ERROR_OK)
		return poll_result;

	target_write_u16(target, MXC_NF_BUFADDR, 2);
	target_write_u16(target, MXC_NF_CFG2, MXC_NF_BIT_OP_FDI);
	poll_result = poll_for_complete_op(nand, "data input");
	if (poll_result != ERROR_OK)
		return poll_result;

	target_write_u16(target, MXC_NF_BUFADDR, 3);
	target_write_u16(target, MXC_NF_CFG2, MXC_NF_BIT_OP_FDI);
	poll_result = poll_for_complete_op(nand, "data input");
	if (poll_result != ERROR_OK)
		return poll_result;

	retval |= mxc_command(nand, NAND_CMD_PAGEPROG);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * check status register
	 */
	retval = ERROR_OK;
	retval |= mxc_command(nand, NAND_CMD_STATUS);
	target_write_u16 (target, MXC_NF_BUFADDR, 0);
	mxc_nf_info->optype = MXC_NF_DATAOUT_NANDSTATUS;
	mxc_nf_info->fin = MXC_NF_FIN_DATAOUT;
	retval |= do_data_output(nand);
	if (retval != ERROR_OK) {
		LOG_ERROR(get_status_register_err_msg);
		return retval;
	}
	target_read_u16(target, MXC_NF_MAIN_BUFFER0, &nand_status_content);
	if (nand_status_content & 0x0001) {
		/*
		 * page not correctly written
		 */
		return ERROR_NAND_OPERATION_FAILED;
	}
#ifdef _MXC_PRINT_STAT
	LOG_INFO("%d bytes newly written", data_size);
#endif
	return ERROR_OK;
}

static int mxc_read_page(struct nand_device *nand, uint32_t page,
						   uint8_t *data, uint32_t data_size,
						   uint8_t *oob, uint32_t oob_size)
{
	struct mxc_nf_controller *mxc_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	int retval;
	uint16_t swap1, swap2, new_swap1;

	if (data_size % 2) {
		LOG_ERROR(data_block_size_err_msg, data_size);
		return ERROR_NAND_OPERATION_FAILED;
	}
	if (oob_size % 2) {
		LOG_ERROR(data_block_size_err_msg, oob_size);
		return ERROR_NAND_OPERATION_FAILED;
	}

	/*
	 * validate target state
	 */
	retval = validate_target_state(nand);
	if (retval != ERROR_OK) {
		return retval;
	}
	/* Reset address_cycles before mxc_command ?? */
	retval = mxc_command(nand, NAND_CMD_READ0);
	if (retval != ERROR_OK) return retval;
	retval = mxc_address(nand, 0); /* col */
	if (retval != ERROR_OK) return retval;
	retval = mxc_address(nand, 0); /* col */
	if (retval != ERROR_OK) return retval;
	retval = mxc_address(nand, page & 0xff); /* page address */
	if (retval != ERROR_OK) return retval;
	retval = mxc_address(nand, (page >> 8) & 0xff); /* page address */
	if (retval != ERROR_OK) return retval;
	retval = mxc_address(nand, (page >> 16) & 0xff); /* page address */
	if (retval != ERROR_OK) return retval;
	retval = mxc_command(nand, NAND_CMD_READSTART);
	if (retval != ERROR_OK) return retval;

	target_write_u16(target, MXC_NF_BUFADDR, 0);
	mxc_nf_info->fin = MXC_NF_FIN_DATAOUT;
	retval = do_data_output(nand);
	if (retval != ERROR_OK) {
		LOG_ERROR("MXC_NF : Error reading page 0");
		return retval;
	}
	/* Test nand page size to know how much MAIN_BUFFER must be written */
	target_write_u16(target, MXC_NF_BUFADDR, 1);
	mxc_nf_info->fin = MXC_NF_FIN_DATAOUT;
	retval = do_data_output(nand);
	if (retval != ERROR_OK) {
		LOG_ERROR("MXC_NF : Error reading page 1");
		return retval;
	}
	target_write_u16(target, MXC_NF_BUFADDR, 2);
	mxc_nf_info->fin = MXC_NF_FIN_DATAOUT;
	retval = do_data_output(nand);
	if (retval != ERROR_OK) {
		LOG_ERROR("MXC_NF : Error reading page 2");
		return retval;
	}
	target_write_u16(target, MXC_NF_BUFADDR, 3);
	mxc_nf_info->fin = MXC_NF_FIN_DATAOUT;
	retval = do_data_output(nand);
	if (retval != ERROR_OK) {
		LOG_ERROR("MXC_NF : Error reading page 3");
		return retval;
	}

	if (nand->page_size > 512 && mxc_nf_info->flags.biswap_enabled) {
		/* BI-swap -  work-around of mxc NFC for NAND device with page == 2k */
		target_read_u16(target, MXC_NF_MAIN_BUFFER3 + 464, &swap1);
		target_read_u16(target, MXC_NF_SPARE_BUFFER3 + 4, &swap2);
		new_swap1 = (swap1 & 0xFF00) | (swap2 >> 8);
		swap2 = (swap1 << 8) | (swap2 & 0xFF);
		target_write_u16(target, MXC_NF_MAIN_BUFFER3 + 464, new_swap1);
		target_write_u16(target, MXC_NF_SPARE_BUFFER3 + 4, swap2);
	}

	if (data)
		target_read_buffer(target, MXC_NF_MAIN_BUFFER0, data_size, data);
	if (oob)
		target_read_buffer(target, MXC_NF_SPARE_BUFFER0, oob_size, oob);

#ifdef _MXC_PRINT_STAT
	if (data_size > 0) {
		/* When Operation Status is read (when page is erased),
		 * this function is used but data_size is null.
		 */
		LOG_INFO("%d bytes newly read", data_size);
	}
#endif
	return ERROR_OK;
}

static int initialize_nf_controller(struct nand_device *nand)
{
	struct mxc_nf_controller *mxc_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	uint16_t work_mode;
	uint16_t temp;
	/*
	 * resets NAND flash controller in zero time ? I dont know.
	 */
	target_write_u16(target, MXC_NF_CFG1, MXC_NF_BIT_RESET_EN);
	work_mode = MXC_NF_BIT_INT_DIS;	/* disable interrupt */
	if (target->endianness == TARGET_BIG_ENDIAN) {
		LOG_DEBUG("MXC_NF : work in Big Endian mode");
		work_mode |= MXC_NF_BIT_BE_EN;
	} else {
		LOG_DEBUG("MXC_NF : work in Little Endian mode");
	}
	if (mxc_nf_info->flags.hw_ecc_enabled) {
		LOG_DEBUG("MXC_NF : work with ECC mode");
		work_mode |= MXC_NF_BIT_ECC_EN;
	} else {
		LOG_DEBUG("MXC_NF : work without ECC mode");
	}
	target_write_u16(target, MXC_NF_CFG1, work_mode);
	/*
	 * unlock SRAM buffer for write; 2 mean "Unlock", other values means "Lock"
	 */
	target_write_u16(target, MXC_NF_BUFCFG, 2);
	target_read_u16(target, MXC_NF_FWP, &temp);
	if ((temp & 0x0007) == 1) {
		LOG_ERROR("NAND flash is tight-locked, reset needed");
		return ERROR_FAIL;
	}

	/*
	 * unlock NAND flash for write
	 */
	target_write_u16(target, MXC_NF_FWP, 4);
	target_write_u16(target, MXC_NF_LOCKSTART, 0x0000);
	target_write_u16(target, MXC_NF_LOCKEND, 0xFFFF);
	/*
	 * 0x0000 means that first SRAM buffer @0xD800_0000 will be used
	 */
	target_write_u16(target, MXC_NF_BUFADDR, 0x0000);
	/*
	 * address of SRAM buffer
	 */
	in_sram_address = MXC_NF_MAIN_BUFFER0;
	sign_of_sequental_byte_read = 0;
	return ERROR_OK;
}

static int get_next_byte_from_sram_buffer(struct nand_device *nand, uint8_t *value)
{
	struct mxc_nf_controller *mxc_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	static uint8_t even_byte = 0;
	uint16_t temp;
	/*
	 * host-big_endian ??
	 */
	if (sign_of_sequental_byte_read == 0)
		even_byte = 0;

	if (in_sram_address > MXC_NF_LAST_BUFFER_ADDR) {
		LOG_ERROR(sram_buffer_bounds_err_msg, in_sram_address);
		*value = 0;
		sign_of_sequental_byte_read = 0;
		even_byte = 0;
		return ERROR_NAND_OPERATION_FAILED;
	} else {
		target_read_u16(target, in_sram_address, &temp);
		if (even_byte) {
			*value = temp >> 8;
			even_byte = 0;
			in_sram_address += 2;
		} else {
			*value = temp & 0xff;
			even_byte = 1;
		}
	}
	sign_of_sequental_byte_read = 1;
	return ERROR_OK;
}

static int get_next_halfword_from_sram_buffer(struct nand_device *nand, uint16_t *value)
{
	struct mxc_nf_controller *mxc_nf_info = nand->controller_priv;
	struct target *target = nand->target;

	if (in_sram_address > MXC_NF_LAST_BUFFER_ADDR) {
		LOG_ERROR(sram_buffer_bounds_err_msg, in_sram_address);
		*value = 0;
		return ERROR_NAND_OPERATION_FAILED;
	} else {
		target_read_u16(target, in_sram_address, value);
		in_sram_address += 2;
	}
	return ERROR_OK;
}

static int poll_for_complete_op(struct nand_device *nand, const char *text)
{
	struct mxc_nf_controller *mxc_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	uint16_t poll_complete_status;

	for (int poll_cycle_count = 0; poll_cycle_count < 100; poll_cycle_count++) {
		target_read_u16(target, MXC_NF_CFG2, &poll_complete_status);
		if (poll_complete_status & MXC_NF_BIT_OP_DONE)
			break;

		usleep(10);
	}
	if (!(poll_complete_status & MXC_NF_BIT_OP_DONE)) {
		LOG_ERROR("%s sending timeout", text);
		return ERROR_NAND_OPERATION_FAILED;
	}
	return ERROR_OK;
}

static int validate_target_state(struct nand_device *nand)
{
	struct mxc_nf_controller *mxc_nf_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR(target_not_halted_err_msg);
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (mxc_nf_info->flags.target_little_endian !=
	    (target->endianness == TARGET_LITTLE_ENDIAN)) {
		/*
		 * endianness changed after NAND controller probed
		 */
		return ERROR_NAND_OPERATION_FAILED;
	}
	return ERROR_OK;
}

static int do_data_output(struct nand_device *nand)
{
	struct mxc_nf_controller *mxc_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	int poll_result;
	uint16_t ecc_status;
	switch (mxc_nf_info->fin) {
	case MXC_NF_FIN_DATAOUT:
		/*
		 * start data output operation (set MXC_NF_BIT_OP_DONE==0)
		 */
		target_write_u16(target, MXC_NF_CFG2, MXC_NF_BIT_DATAOUT_TYPE(mxc_nf_info->optype));
		poll_result = poll_for_complete_op(nand, "data output");
		if (poll_result != ERROR_OK)
			return poll_result;

		mxc_nf_info->fin = MXC_NF_FIN_NONE;
		/*
		 * ECC stuff
		 */
		if ((mxc_nf_info->optype == MXC_NF_DATAOUT_PAGE) &&
			mxc_nf_info->flags.hw_ecc_enabled) {
			target_read_u16(target, MXC_NF_ECCSTATUS, &ecc_status);
			switch (ecc_status & 0x000c) {
			case 1 << 2:
				LOG_INFO("main area readed with 1 (correctable) error");
				break;
			case 2 << 2:
				LOG_INFO("main area readed with more than 1 (incorrectable) error");
				return ERROR_NAND_OPERATION_FAILED;
					break;
			}
			switch (ecc_status & 0x0003) {
			case 1:
				LOG_INFO("spare area readed with 1 (correctable) error");
				break;
			case 2:
				LOG_INFO("main area readed with more than 1 (incorrectable) error");
				return ERROR_NAND_OPERATION_FAILED;
				break;
			}
		}
		break;
	case MXC_NF_FIN_NONE:
		break;
	}
	return ERROR_OK;
}

struct nand_flash_controller mxc_nand_flash_controller = {
	.name					= "mxc",
	.nand_device_command	= &mxc_nand_device_command,
	.init					= &mxc_init,
	.reset					= &mxc_reset,
	.command				= &mxc_command,
	.address				= &mxc_address,
	.write_data				= &mxc_write_data,
	.read_data				= &mxc_read_data,
	.write_page				= &mxc_write_page,
	.read_page				= &mxc_read_page,
	.nand_ready				= &mxc_nand_ready,
};
