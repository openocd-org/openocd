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
 * Freescale iMX2* OpenOCD NAND Flash controller support.
 * based on Freescale iMX3* OpenOCD NAND Flash controller support.
 */

/*
 * driver tested with Samsung K9F2G08UXA and Numonyx/ST NAND02G-B2D @imx27
 * tested "nand probe #", "nand erase # 0 #", "nand dump # file 0 #", 
 * "nand write # file 0", "nand verify"
 *
 * get_next_halfword_from_sram_buffer() not tested
 * !! all function only tested with 2k page nand device; imx27_write_page
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
#undef _MX2_PRINT_STAT
//#define _MX2_PRINT_STAT

static const char target_not_halted_err_msg[] =
	"target must be halted to use mx2 NAND flash controller";
static const char data_block_size_err_msg[] =
	"minimal granularity is one half-word, %" PRId32 " is incorrect";
static const char sram_buffer_bounds_err_msg[] =
	"trying to access out of SRAM buffer bound (addr=0x%" PRIx32 ")";
static const char get_status_register_err_msg[] = "can't get NAND status";
static uint32_t in_sram_address;
static unsigned char sign_of_sequental_byte_read;

static int initialize_nf_controller(struct nand_device *nand);
static int get_next_byte_from_sram_buffer(struct target * target, uint8_t * value);
static int get_next_halfword_from_sram_buffer(struct target * target,
					       uint16_t * value);
static int poll_for_complete_op(struct target * target, const char *text);
static int validate_target_state(struct nand_device *nand);
static int do_data_output(struct nand_device *nand);

static int imx27_command(struct nand_device *nand, uint8_t command);
static int imx27_address(struct nand_device *nand, uint8_t address);

NAND_DEVICE_COMMAND_HANDLER(imx27_nand_device_command)
{
	struct mx2_nf_controller *mx2_nf_info;
	int hwecc_needed;
	int x;
	mx2_nf_info = malloc(sizeof(struct mx2_nf_controller));
	if (mx2_nf_info == NULL) {
		LOG_ERROR("no memory for nand controller");
		return ERROR_FAIL;
	}

	nand->controller_priv = mx2_nf_info;
	if (CMD_ARGC < 3) {
		LOG_ERROR("use \"nand device imx27 target noecc|hwecc\"");
		return ERROR_FAIL;
	}
	/*
	 * check hwecc requirements
	 */

	hwecc_needed = strcmp(CMD_ARGV[2], "hwecc");
	if (hwecc_needed == 0) 
		mx2_nf_info->flags.hw_ecc_enabled = 1;
	else
		mx2_nf_info->flags.hw_ecc_enabled = 0;

	mx2_nf_info->optype = MX2_NF_DATAOUT_PAGE;
	mx2_nf_info->fin = MX2_NF_FIN_NONE;
	mx2_nf_info->flags.target_little_endian =
	(nand->target->endianness == TARGET_LITTLE_ENDIAN);
	/*
	 * testing host endianness
	 */
	x = 1;
	if (*(char *) &x == 1)
		mx2_nf_info->flags.host_little_endian = 1;
	else
		mx2_nf_info->flags.host_little_endian = 0;
	return ERROR_OK;
}

static int imx27_init(struct nand_device *nand)
{
	struct mx2_nf_controller *mx2_nf_info = nand->controller_priv;
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

	target_read_u16(target, MX2_NF_BUFSIZ, &buffsize_register_content);
	mx2_nf_info->flags.one_kb_sram = !(buffsize_register_content & 0x000f);

	target_read_u32(target, MX2_FMCR, &pcsr_register_content);
	if (!nand->bus_width) {
		/* bus_width not yet defined. Read it from MX2_FMCR */
		nand->bus_width =
		    (pcsr_register_content & MX2_FMCR_NF_16BIT_SEL) ? 16 : 8;
	} else {
		/* bus_width forced in soft. Sync it to MX2_FMCR */
		pcsr_register_content |=
		    ((nand->bus_width == 16) ? MX2_FMCR_NF_16BIT_SEL : 0x00000000);
		target_write_u32(target, MX2_FMCR, pcsr_register_content);
	}
	if (nand->bus_width == 16)
		LOG_DEBUG("MX2_NF : bus is 16-bit width");
	else
		LOG_DEBUG("MX2_NF : bus is 8-bit width");

	if (!nand->page_size) {
		nand->page_size =
		    (pcsr_register_content & MX2_FMCR_NF_FMS) ? 2048 : 512;
	} else {
		pcsr_register_content |=
		    ((nand->page_size == 2048) ? MX2_FMCR_NF_FMS : 0x00000000);
		target_write_u32(target, MX2_FMCR, pcsr_register_content);
	}
	if (mx2_nf_info->flags.one_kb_sram && (nand->page_size == 2048)) {
		LOG_ERROR("NAND controller have only 1 kb SRAM, so "
		          "pagesize 2048 is incompatible with it");
	} else {
		LOG_DEBUG("MX2_NF : NAND controller can handle pagesize of 2048");
	}

	initialize_nf_controller(nand);

	retval = ERROR_OK;
	retval |= imx27_command(nand, NAND_CMD_STATUS);
	retval |= imx27_address(nand, 0x00);
	retval |= do_data_output(nand);
	if (retval != ERROR_OK) {
		LOG_ERROR(get_status_register_err_msg);
		return ERROR_FAIL;
	}
	target_read_u16(target, MX2_NF_MAIN_BUFFER0, &nand_status_content);
	if (!(nand_status_content & 0x0080)) {
		LOG_INFO("NAND read-only");
		mx2_nf_info->flags.nand_readonly = 1;
	} else {
		mx2_nf_info->flags.nand_readonly = 0;
	}
	return ERROR_OK;
}

static int imx27_read_data(struct nand_device *nand, void *data)
{
	struct target *target = nand->target;
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
		LOG_ERROR("imx27_read_data : read data failed : '%x'",
		          try_data_output_from_nand_chip);
		return try_data_output_from_nand_chip;
	}

	if (nand->bus_width == 16)
	    get_next_halfword_from_sram_buffer(target, data);
	else
	    get_next_byte_from_sram_buffer(target, data);

	return ERROR_OK;
}

static int imx27_write_data(struct nand_device *nand, uint16_t data)
{
	LOG_ERROR("write_data() not implemented");
	return ERROR_NAND_OPERATION_FAILED;
}

static int imx27_reset(struct nand_device *nand)
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

static int imx27_command(struct nand_device *nand, uint8_t command)
{
	struct mx2_nf_controller *mx2_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	int validate_target_result;
	int poll_result;
	/*
	 * validate target state
	 */
	validate_target_result = validate_target_state(nand);
	if (validate_target_result != ERROR_OK)
		return validate_target_result;

	switch(command) {
	case NAND_CMD_READOOB:
		command = NAND_CMD_READ0;
		/* set read point for data_read() and read_block_data() to
		 * spare area in SRAM buffer
		 */
		in_sram_address = MX2_NF_SPARE_BUFFER0;	
		break;
	case NAND_CMD_READ1:
		command = NAND_CMD_READ0;
		/*
		 * offset == one half of page size
		 */
		in_sram_address =
		    MX2_NF_MAIN_BUFFER0 + (nand->page_size >> 1);
		break;
	default:
		in_sram_address = MX2_NF_MAIN_BUFFER0;
		break;
	}

	target_write_u16(target, MX2_NF_FCMD, command);
	/*
	 * start command input operation (set MX2_NF_BIT_OP_DONE==0)
	 */
	target_write_u16(target, MX2_NF_CFG2, MX2_NF_BIT_OP_FCI);
	poll_result = poll_for_complete_op(target, "command");
	if (poll_result != ERROR_OK)
		return poll_result;
	/*
	 * reset cursor to begin of the buffer
	 */
	sign_of_sequental_byte_read = 0;
	/* Handle special read command and adjust NF_CFG2(FDO) */
	switch(command) {
	case NAND_CMD_READID:
		mx2_nf_info->optype = MX2_NF_DATAOUT_NANDID;
		mx2_nf_info->fin = MX2_NF_FIN_DATAOUT;
		break;
	case NAND_CMD_STATUS:
		mx2_nf_info->optype = MX2_NF_DATAOUT_NANDSTATUS;
		mx2_nf_info->fin = MX2_NF_FIN_DATAOUT;
                target_write_u16 (target, MX2_NF_BUFADDR, 0);
                in_sram_address = 0;
		break;
	case NAND_CMD_READ0:
		mx2_nf_info->fin = MX2_NF_FIN_DATAOUT;
		mx2_nf_info->optype = MX2_NF_DATAOUT_PAGE;
		break;
	default:
		/* Ohter command use the default 'One page data out' FDO */
		mx2_nf_info->optype = MX2_NF_DATAOUT_PAGE;
		break;
	}
	return ERROR_OK;
}

static int imx27_address(struct nand_device *nand, uint8_t address)
{
	struct target *target = nand->target;
	int validate_target_result;
	int poll_result;
	/*
	 * validate target state
	 */
	validate_target_result = validate_target_state(nand);
	if (validate_target_result != ERROR_OK)
		return validate_target_result;

	target_write_u16(target, MX2_NF_FADDR, address);
	/*
	 * start address input operation (set MX2_NF_BIT_OP_DONE==0)
	 */
	target_write_u16(target, MX2_NF_CFG2, MX2_NF_BIT_OP_FAI);
	poll_result = poll_for_complete_op(target, "address");
	if (poll_result != ERROR_OK)
		return poll_result;

	return ERROR_OK;
}

static int imx27_nand_ready(struct nand_device *nand, int tout)
{
	uint16_t poll_complete_status;
	struct target *target = nand->target;
	int validate_target_result;

	/*
	 * validate target state
	 */
	validate_target_result = validate_target_state(nand);
	if (validate_target_result != ERROR_OK)
		return validate_target_result;

	do {
		target_read_u16(target, MX2_NF_CFG2, &poll_complete_status);
		if (poll_complete_status & MX2_NF_BIT_OP_DONE)
			return tout;

		alive_sleep(1);
	}
	while (tout-- > 0);
	return tout;
}

static int imx27_write_page(struct nand_device *nand, uint32_t page,
			     uint8_t * data, uint32_t data_size, uint8_t * oob,
			     uint32_t oob_size)
{
	struct mx2_nf_controller *mx2_nf_info = nand->controller_priv;
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

	in_sram_address = MX2_NF_MAIN_BUFFER0;
	sign_of_sequental_byte_read = 0;
	retval = ERROR_OK;
	retval |= imx27_command(nand, NAND_CMD_SEQIN);
	retval |= imx27_address(nand, 0); //col
	retval |= imx27_address(nand, 0); //col
	retval |= imx27_address(nand, page & 0xff); //page address
	retval |= imx27_address(nand, (page >> 8) & 0xff); //page address
	retval |= imx27_address(nand, (page >> 16) & 0xff); //page address
	
	target_write_buffer(target, MX2_NF_MAIN_BUFFER0, data_size, data);
	if (oob) {
		if (mx2_nf_info->flags.hw_ecc_enabled) {
			/*
			 * part of spare block will be overrided by hardware
			 * ECC generator
			 */
			LOG_DEBUG("part of spare block will be overrided "
			          "by hardware ECC generator");
		}
		target_write_buffer(target, MX2_NF_SPARE_BUFFER0, oob_size,
		                    oob);
	}
	//BI-swap -  work-around of imx27 NFC for NAND device with page == 2kb
	target_read_u16(target, MX2_NF_MAIN_BUFFER3 + 464, &swap1);
	if (oob) {
		LOG_ERROR("Due to NFC Bug, oob is not correctly implemented "
		          "in mx2 driver");
		return ERROR_NAND_OPERATION_FAILED;
	}
	//target_read_u16 (target, MX2_NF_SPARE_BUFFER3 + 4, &swap2);
	swap2 = 0xffff;  //Spare buffer unused forced to 0xffff
        new_swap1 = (swap1 & 0xFF00) | (swap2 >> 8);
        swap2 = (swap1 << 8) | (swap2 & 0xFF);

	target_write_u16(target, MX2_NF_MAIN_BUFFER3 + 464, new_swap1);
	target_write_u16(target, MX2_NF_SPARE_BUFFER3 + 4, swap2);
	/*
	 * start data input operation (set MX2_NF_BIT_OP_DONE==0)
	 */
	target_write_u16(target, MX2_NF_BUFADDR, 0);
	target_write_u16(target, MX2_NF_CFG2, MX2_NF_BIT_OP_FDI);
	poll_result = poll_for_complete_op(target, "data input");
	if (poll_result != ERROR_OK)
		return poll_result;
	
	target_write_u16(target, MX2_NF_BUFADDR, 1);
	target_write_u16(target, MX2_NF_CFG2, MX2_NF_BIT_OP_FDI);
	poll_result = poll_for_complete_op(target, "data input");
	if (poll_result != ERROR_OK)
		return poll_result;
	
	target_write_u16(target, MX2_NF_BUFADDR, 2);
	target_write_u16(target, MX2_NF_CFG2, MX2_NF_BIT_OP_FDI);
	poll_result = poll_for_complete_op(target, "data input");
	if (poll_result != ERROR_OK)
		return poll_result;
	
	target_write_u16(target, MX2_NF_BUFADDR, 3);
	target_write_u16(target, MX2_NF_CFG2, MX2_NF_BIT_OP_FDI);
	poll_result = poll_for_complete_op(target, "data input");
	if (poll_result != ERROR_OK)
		return poll_result;

	retval |= imx27_command(nand, NAND_CMD_PAGEPROG);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * check status register
	 */
        retval = ERROR_OK;
        retval |= imx27_command(nand, NAND_CMD_STATUS);
        target_write_u16 (target, MX2_NF_BUFADDR, 0);
        mx2_nf_info->optype = MX2_NF_DATAOUT_NANDSTATUS;
        mx2_nf_info->fin = MX2_NF_FIN_DATAOUT;
        retval |= do_data_output(nand);
        if (retval != ERROR_OK) {
                LOG_ERROR (get_status_register_err_msg);
                return retval;
        }
        target_read_u16 (target, MX2_NF_MAIN_BUFFER0, &nand_status_content);
        if (nand_status_content & 0x0001) {
                /*
                 * page not correctly written
                 */
                return ERROR_NAND_OPERATION_FAILED;
        }
#ifdef _MX2_PRINT_STAT
	LOG_INFO("%d bytes newly written", data_size);
#endif
	return ERROR_OK;
}

static int imx27_read_page(struct nand_device *nand, uint32_t page,
			    uint8_t * data, uint32_t data_size, uint8_t * oob,
			    uint32_t oob_size)
{
	struct mx2_nf_controller *mx2_nf_info = nand->controller_priv;
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
	/* Reset address_cycles before imx27_command ?? */
	retval = ERROR_OK;
	retval |= imx27_command(nand, NAND_CMD_READ0);

	retval |= imx27_address(nand, 0); //col
	retval |= imx27_address(nand, 0); //col
	retval |= imx27_address(nand, page & 0xff); //page address
	retval |= imx27_address(nand, (page >> 8) & 0xff); //page address
	retval |= imx27_address(nand, (page >> 16) & 0xff); //page address
	retval |= imx27_command(nand, NAND_CMD_READSTART);

	target_write_u16(target, MX2_NF_BUFADDR, 0);
	mx2_nf_info->fin = MX2_NF_FIN_DATAOUT;
	retval = do_data_output(nand);
	if (retval != ERROR_OK) {
		LOG_ERROR("MX2_NF : Error reading page 0");
		return retval;
	}
	//Test nand page size to know how much MAIN_BUFFER must be written
	target_write_u16(target, MX2_NF_BUFADDR, 1);
	mx2_nf_info->fin = MX2_NF_FIN_DATAOUT;
	retval = do_data_output(nand);
	if (retval != ERROR_OK) {
		LOG_ERROR("MX2_NF : Error reading page 1");
		return retval;
	}
	target_write_u16(target, MX2_NF_BUFADDR, 2);
	mx2_nf_info->fin = MX2_NF_FIN_DATAOUT;
	retval = do_data_output(nand);
	if (retval != ERROR_OK) {
		LOG_ERROR("MX2_NF : Error reading page 2");
		return retval;
	}
	target_write_u16(target, MX2_NF_BUFADDR, 3);
	mx2_nf_info->fin = MX2_NF_FIN_DATAOUT;
	retval = do_data_output(nand);
	if (retval != ERROR_OK) {
		LOG_ERROR("MX2_NF : Error reading page 3");
		return retval;
	}
	//BI-swap -  work-around of imx27 NFC for NAND device with page == 2k
	target_read_u16(target, MX2_NF_MAIN_BUFFER3 + 464, &swap1);
	target_read_u16(target, MX2_NF_SPARE_BUFFER3 + 4, &swap2);
        new_swap1 = (swap1 & 0xFF00) | (swap2 >> 8);
        swap2 = (swap1 << 8) | (swap2 & 0xFF);
	target_write_u16(target, MX2_NF_MAIN_BUFFER3 + 464, new_swap1);
	target_write_u16(target, MX2_NF_SPARE_BUFFER3 + 4, swap2);

	if (data)
		target_read_buffer(target, MX2_NF_MAIN_BUFFER0, data_size, data);
	if (oob)
		target_read_buffer(target, MX2_NF_SPARE_BUFFER0, oob_size,
		                    oob);
#ifdef _MX2_PRINT_STAT
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
	struct mx2_nf_controller *mx2_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	uint16_t work_mode;
	uint16_t temp;
	/*
	 * resets NAND flash controller in zero time ? I dont know.
	 */
	target_write_u16(target, MX2_NF_CFG1, MX2_NF_BIT_RESET_EN);
	work_mode = MX2_NF_BIT_INT_DIS;	/* disable interrupt */
	if (target->endianness == TARGET_BIG_ENDIAN) {
		LOG_DEBUG("MX2_NF : work in Big Endian mode");
		work_mode |= MX2_NF_BIT_BE_EN;
	} else {
		LOG_DEBUG("MX2_NF : work in Little Endian mode");
	}
	if (mx2_nf_info->flags.hw_ecc_enabled) {
		LOG_DEBUG("MX2_NF : work with ECC mode");
		work_mode |= MX2_NF_BIT_ECC_EN;
	} else {
		LOG_DEBUG("MX2_NF : work without ECC mode");
	}
	target_write_u16(target, MX2_NF_CFG1, work_mode);
	/*
	 * unlock SRAM buffer for write; 2 mean "Unlock", other values means "Lock"
	 */
	target_write_u16(target, MX2_NF_BUFCFG, 2);
	target_read_u16(target, MX2_NF_FWP, &temp);
	if ((temp & 0x0007) == 1) {
		LOG_ERROR("NAND flash is tight-locked, reset needed");
		return ERROR_FAIL;
	}

	/*
	 * unlock NAND flash for write
	 */
	target_write_u16(target, MX2_NF_FWP, 4);
	target_write_u16(target, MX2_NF_LOCKSTART, 0x0000);
	target_write_u16(target, MX2_NF_LOCKEND, 0xFFFF);
	/*
	 * 0x0000 means that first SRAM buffer @0xD800_0000 will be used
	 */
	target_write_u16(target, MX2_NF_BUFADDR, 0x0000);
	/*
	 * address of SRAM buffer
	 */
	in_sram_address = MX2_NF_MAIN_BUFFER0;
	sign_of_sequental_byte_read = 0;
	return ERROR_OK;
}

static int get_next_byte_from_sram_buffer(struct target * target, uint8_t * value)
{
	static uint8_t even_byte = 0;
	uint16_t temp;
	/*
	 * host-big_endian ??
	 */
	if (sign_of_sequental_byte_read == 0)
		even_byte = 0;
	
	if (in_sram_address > MX2_NF_LAST_BUFFER_ADDR) {
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

static int get_next_halfword_from_sram_buffer(struct target * target,
					       uint16_t * value)
{
	if (in_sram_address > MX2_NF_LAST_BUFFER_ADDR) {
		LOG_ERROR(sram_buffer_bounds_err_msg, in_sram_address);
		*value = 0;
		return ERROR_NAND_OPERATION_FAILED;
	} else {
		target_read_u16(target, in_sram_address, value);
		in_sram_address += 2;
	}
	return ERROR_OK;
}

static int poll_for_complete_op(struct target * target, const char *text)
{
	uint16_t poll_complete_status;
	for (int poll_cycle_count = 0; poll_cycle_count < 100; poll_cycle_count++) {
		target_read_u16(target, MX2_NF_CFG2, &poll_complete_status);
		if (poll_complete_status & MX2_NF_BIT_OP_DONE)
			break;

		usleep(10);
	}
	if (!(poll_complete_status & MX2_NF_BIT_OP_DONE)) {
		LOG_ERROR("%s sending timeout", text);
		return ERROR_NAND_OPERATION_FAILED;
	}
	return ERROR_OK;
}

static int validate_target_state(struct nand_device *nand)
{
	struct mx2_nf_controller *mx2_nf_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR(target_not_halted_err_msg);
		return ERROR_NAND_OPERATION_FAILED;
	}

	if (mx2_nf_info->flags.target_little_endian != 
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
	struct mx2_nf_controller *mx2_nf_info = nand->controller_priv;
	struct target *target = nand->target;
	int poll_result;
	uint16_t ecc_status;
	switch(mx2_nf_info->fin) {
	case MX2_NF_FIN_DATAOUT:
		/*
		 * start data output operation (set MX2_NF_BIT_OP_DONE==0)
		 */
		target_write_u16(target, MX2_NF_CFG2, MX2_NF_BIT_DATAOUT_TYPE(mx2_nf_info->optype));
		poll_result = poll_for_complete_op(target, "data output");
		if (poll_result != ERROR_OK)
			return poll_result;

		mx2_nf_info->fin = MX2_NF_FIN_NONE;
		/*
		 * ECC stuff
		 */
		if ((mx2_nf_info->optype == MX2_NF_DATAOUT_PAGE) && mx2_nf_info->flags.hw_ecc_enabled) {
			target_read_u16(target, MX2_NF_ECCSTATUS, &ecc_status);
			switch(ecc_status & 0x000c) {
			case 1 << 2:
				LOG_INFO("main area readed with 1 (correctable) error");
				break;
			case 2 << 2:
				LOG_INFO("main area readed with more than 1 (incorrectable) error");
				return ERROR_NAND_OPERATION_FAILED;
		    		break;
			}
			switch(ecc_status & 0x0003) {
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
	case MX2_NF_FIN_NONE:
		break;
	}
	return ERROR_OK;
}

struct nand_flash_controller imx27_nand_flash_controller = {
	.name			= "imx27",
	.nand_device_command 	= &imx27_nand_device_command,
	.init			= &imx27_init,
	.reset			= &imx27_reset,
	.command		= &imx27_command,
	.address		= &imx27_address,
	.write_data		= &imx27_write_data,
	.read_data		= &imx27_read_data,
	.write_page		= &imx27_write_page,
	.read_page		= &imx27_read_page,
	.nand_ready		= &imx27_nand_ready,
};
