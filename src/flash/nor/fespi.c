/***************************************************************************
 *   Copyright (C) 2010 by Antonio Borneo <borneo.antonio@gmail.com>       *
 *   Modified by Megan Wachs <megan@sifive.com> from the original stmsmi.c *
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

/* The Freedom E SPI controller is a SPI bus controller
 * specifically designed for SPI Flash Memories on Freedom E platforms.
 * 
 * Two working modes are available:
 * - SW mode: the SPI is controlled by SW. Any custom commands can be sent
 *   on the bus. Writes are only possible in this mode.
 * - HW mode: Memory content is directly
 *   accessible in CPU memory space. CPU can read, write and execute memory
 *   content. */

/* ATTENTION:
 * To have flash memory mapped in CPU memory space, the controller
 * must have "HW mode" enabled.
 * 1) The command "reset init" has to initialize the controller and put
 *    it in HW mode (this is actually the default out of reset for Freedom E systems).
 * 2) every command in this file have to return to prompt in HW mode. */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "spi.h"
#include <jtag/jtag.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include "target/riscv/riscv.h"

/* Register offsets */

#define FESPI_REG_SCKDIV          0x00
#define FESPI_REG_SCKMODE         0x04
#define FESPI_REG_CSID            0x10
#define FESPI_REG_CSDEF           0x14
#define FESPI_REG_CSMODE          0x18

#define FESPI_REG_DCSSCK          0x28
#define FESPI_REG_DSCKCS          0x2a
#define FESPI_REG_DINTERCS        0x2c
#define FESPI_REG_DINTERXFR       0x2e

#define FESPI_REG_FMT             0x40
#define FESPI_REG_TXFIFO          0x48
#define FESPI_REG_RXFIFO          0x4c
#define FESPI_REG_TXCTRL          0x50
#define FESPI_REG_RXCTRL          0x54

#define FESPI_REG_FCTRL           0x60
#define FESPI_REG_FFMT            0x64

#define FESPI_REG_IE              0x70
#define FESPI_REG_IP              0x74

/* Fields */

#define FESPI_SCK_POL             0x1
#define FESPI_SCK_PHA             0x2

#define FESPI_FMT_PROTO(x)        ((x) & 0x3)
#define FESPI_FMT_ENDIAN(x)       (((x) & 0x1) << 2)
#define FESPI_FMT_DIR(x)          (((x) & 0x1) << 3)
#define FESPI_FMT_LEN(x)          (((x) & 0xf) << 16)

/* TXCTRL register */
#define FESPI_TXWM(x)             ((x) & 0xffff)
/* RXCTRL register */
#define FESPI_RXWM(x)             ((x) & 0xffff)

#define FESPI_IP_TXWM             0x1
#define FESPI_IP_RXWM             0x2

#define FESPI_FCTRL_EN            0x1

#define FESPI_INSN_CMD_EN         0x1
#define FESPI_INSN_ADDR_LEN(x)    (((x) & 0x7) << 1)
#define FESPI_INSN_PAD_CNT(x)     (((x) & 0xf) << 4)
#define FESPI_INSN_CMD_PROTO(x)   (((x) & 0x3) << 8)
#define FESPI_INSN_ADDR_PROTO(x)  (((x) & 0x3) << 10)
#define FESPI_INSN_DATA_PROTO(x)  (((x) & 0x3) << 12)
#define FESPI_INSN_CMD_CODE(x)    (((x) & 0xff) << 16)
#define FESPI_INSN_PAD_CODE(x)    (((x) & 0xff) << 24)

/* Values */

#define FESPI_CSMODE_AUTO         0
#define FESPI_CSMODE_HOLD         2
#define FESPI_CSMODE_OFF          3

#define FESPI_DIR_RX              0
#define FESPI_DIR_TX              1

#define FESPI_PROTO_S             0
#define FESPI_PROTO_D             1
#define FESPI_PROTO_Q             2

#define FESPI_ENDIAN_MSB          0
#define FESPI_ENDIAN_LSB          1


/* Timeout in ms */
#define FESPI_CMD_TIMEOUT   (100)
#define FESPI_PROBE_TIMEOUT (100)
#define FESPI_MAX_TIMEOUT  (3000)


#define FESPI_READ_REG(a) (_FESPI_READ_REG(a))
#define _FESPI_READ_REG(a)					\
{								\
	int __a;						\
	uint32_t __v;						\
								\
	__a = target_read_u32(target, ctrl_base + (a), &__v); 	\
	if (__a != ERROR_OK) {					\
		LOG_ERROR("FESPI_READ_REG error");		\
		return __a;					\
	}							\
	__v;							\
}

#define FESPI_WRITE_REG(a, v)					\
{								\
	int __r;						\
								\
	__r = target_write_u32(target, ctrl_base + (a), (v)); 	\
	if (__r != ERROR_OK) {					\
		LOG_ERROR("FESPI_WRITE_REG error");		\
		return __r;					\
	}							\
}

#define FESPI_DISABLE_HW_MODE()	FESPI_WRITE_REG(FESPI_REG_FCTRL, \
		FESPI_READ_REG(FESPI_REG_FCTRL) & ~FESPI_FCTRL_EN)
#define FESPI_ENABLE_HW_MODE()	FESPI_WRITE_REG(FESPI_REG_FCTRL, \
	FESPI_READ_REG(FESPI_REG_FCTRL) | FESPI_FCTRL_EN)

struct fespi_flash_bank {
	int probed;
	uint32_t ctrl_base;
	const struct flash_device *dev;
};

struct fespi_target {
	char *name;
	uint32_t tap_idcode;
	uint32_t ctrl_base;
};

//TODO !!! What is the right naming convention here?
static const struct fespi_target target_devices[] = {
	/* name,   tap_idcode, ctrl_base */
	{ "Freedom E300 SPI Flash",  0x10e31913 , 0x10014000 },
	{ NULL,    0,           0          }
};

FLASH_BANK_COMMAND_HANDLER(fespi_flash_bank_command)
{
	struct fespi_flash_bank *fespi_info;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	fespi_info = malloc(sizeof(struct fespi_flash_bank));
	if (fespi_info == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = fespi_info;
	fespi_info->probed = 0;
	fespi_info->ctrl_base = 0;
	if (CMD_ARGC >= 7) {
          int temp;
	  COMMAND_PARSE_NUMBER(int, CMD_ARGV[6], temp);
          fespi_info->ctrl_base = (uint32_t) temp;
	  LOG_DEBUG("ASSUMING FESPI device at ctrl_base = 0x%x", fespi_info->ctrl_base);
	}

	return ERROR_OK;
}

static int fespi_set_dir (struct flash_bank * bank, bool dir) {
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t ctrl_base = fespi_info->ctrl_base;

	FESPI_WRITE_REG(FESPI_REG_FMT,
			(FESPI_READ_REG(FESPI_REG_FMT) & ~(FESPI_FMT_DIR(0xFFFFFFFF))) |
			FESPI_FMT_DIR(dir));

	return ERROR_OK;

}

static int fespi_txwm_wait(struct flash_bank *bank) {
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t ctrl_base = fespi_info->ctrl_base;

	int64_t start = timeval_ms();

	while (1) {
		if (FESPI_READ_REG(FESPI_REG_IP) & FESPI_IP_TXWM) {
			break;
		}
		int64_t now = timeval_ms();
		if (now - start > 1000) {
			LOG_ERROR("ip.txwm didn't get set.");
			return ERROR_TARGET_TIMEOUT;
		}
	}

	return ERROR_OK;

}

static int fespi_tx(struct flash_bank *bank, uint8_t in){
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t ctrl_base = fespi_info->ctrl_base;

	int64_t start = timeval_ms();

	while (1) {
		if ((int32_t) FESPI_READ_REG(FESPI_REG_TXFIFO) >= 0) {
			break;
		}
		int64_t now = timeval_ms();
		if (now - start > 1000) {
			LOG_ERROR("txfifo stayed negative.");
			return ERROR_TARGET_TIMEOUT;
		}
	}

	FESPI_WRITE_REG(FESPI_REG_TXFIFO, in);

	return ERROR_OK;
}

static int fespi_rx(struct flash_bank *bank, uint8_t *out)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t ctrl_base = fespi_info->ctrl_base;

	int64_t start = timeval_ms();
	int32_t value;

	while (1) {
		value = (int32_t) FESPI_READ_REG(FESPI_REG_RXFIFO);
		if (value >= 0)
			break;
		int64_t now = timeval_ms();
		if (now - start > 1000) {
			LOG_ERROR("rxfifo didn't go positive (value=0x%x).", value);
			return ERROR_TARGET_TIMEOUT;
		}
	}

	if (out) {
		*out = value & 0xff;
	}
	return ERROR_OK;
}

//TODO!!! Why don't we need to call this after writing?
static int fespi_wip (struct flash_bank * bank, int timeout)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t ctrl_base = fespi_info->ctrl_base;

	int64_t endtime;

	fespi_set_dir(bank, FESPI_DIR_RX);

	FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_HOLD);
	endtime = timeval_ms() + timeout;

	fespi_tx(bank, SPIFLASH_READ_STATUS);
	if (fespi_rx(bank, NULL) != ERROR_OK)
		return ERROR_FAIL;

	do {
		alive_sleep(1);

		fespi_tx(bank, 0);
		uint8_t rx;
		if (fespi_rx(bank, &rx) != ERROR_OK)
			return ERROR_FAIL;
		if ((rx & SPIFLASH_BSY_BIT) == 0) {
			FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_AUTO);
			fespi_set_dir(bank, FESPI_DIR_TX);
			return ERROR_OK;
		}
	} while (timeval_ms() < endtime);

	LOG_ERROR("timeout");
	return ERROR_FAIL;
}

static int fespi_erase_sector(struct flash_bank *bank, int sector)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t ctrl_base = fespi_info->ctrl_base;
	int retval;

	retval = fespi_tx(bank, SPIFLASH_WRITE_ENABLE);
	if (retval != ERROR_OK) {return retval;}
	retval = fespi_txwm_wait(bank);
	if (retval != ERROR_OK) {return retval;}
	  
	FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_HOLD);
	retval = fespi_tx(bank, fespi_info->dev->erase_cmd);
	if (retval != ERROR_OK) {return retval;}	
	sector = bank->sectors[sector].offset;
	retval = fespi_tx(bank, sector >> 16);
	if (retval != ERROR_OK) {return retval;}	
	retval = fespi_tx(bank, sector >> 8);
	if (retval != ERROR_OK) {return retval;}	
	retval = fespi_tx(bank, sector);
	if (retval != ERROR_OK) {return retval;}	
	retval = fespi_txwm_wait(bank);
	if (retval != ERROR_OK) {return retval;}	
	FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_AUTO);

	retval = fespi_wip(bank, FESPI_MAX_TIMEOUT);
	if (retval != ERROR_OK){return retval;}

	return ERROR_OK;
}

static int fespi_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t ctrl_base = fespi_info->ctrl_base;
	int retval = ERROR_OK;
	int sector;

	LOG_DEBUG("%s: from sector %d to sector %d", __func__, first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first < 0) || (last < first) || (last >= bank->num_sectors)) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(fespi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
	}

	FESPI_WRITE_REG(FESPI_REG_TXCTRL, FESPI_TXWM(1));
	retval = fespi_txwm_wait(bank);
	if (retval != ERROR_OK){
	  LOG_ERROR("WM Didn't go high before attempting.");
	  return retval;
	}

	/* Disable Hardware accesses*/
	FESPI_DISABLE_HW_MODE();

	/* poll WIP */
	retval = fespi_wip(bank, FESPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK) 
		return retval;

	for (sector = first; sector <= last; sector++) {
		retval = fespi_erase_sector(bank, sector);
		if (retval != ERROR_OK)
			break;
		keep_alive();
	}

	/* Switch to HW mode before return to prompt */
	FESPI_ENABLE_HW_MODE();
	return retval;
}

static int fespi_protect(struct flash_bank *bank, int set,
		int first, int last)
{
	int sector;

	for (sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;
	return ERROR_OK;
}

static int slow_fespi_write_buffer(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t len)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t ctrl_base = fespi_info->ctrl_base;
	uint32_t ii;

	//TODO!!! assert that len < page size

	fespi_tx(bank, SPIFLASH_WRITE_ENABLE);
	fespi_txwm_wait(bank);

	FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_HOLD);

	fespi_tx(bank, SPIFLASH_PAGE_PROGRAM);

	fespi_tx(bank, offset >> 16);
	fespi_tx(bank, offset >> 8);
	fespi_tx(bank, offset);

	for (ii = 0; ii < len; ii++) {
		fespi_tx(bank, buffer[ii]);
	}

	fespi_txwm_wait(bank);

	FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_AUTO);

	keep_alive();

	return ERROR_OK;
}

/*
 * Here's the source for the algorithm.
 * You can turn it into the array below using:
	sed -n '/ALGO_START$/,/ALGO_END/ p' fespi.c | \
		riscv32-unknown-elf-gcc -x assembler-with-cpp - -nostdlib -nostartfiles -o tmp.o && \
		riscv32-unknown-elf-objcopy -O binary tmp.o algorithm.bin && \
		xxd -i algorithm.bin

// ALGO_START
#define SPIFLASH_READ_STATUS	0x05 // Read Status Register
#define SPIFLASH_BSY_BIT		0x00000001 // WIP Bit of SPI SR on SMI SR

// Register offsets
#define FESPI_REG_FMT             0x40
#define FESPI_REG_TXFIFO          0x48
#define FESPI_REG_RXFIFO          0x4c
#define FESPI_REG_IP              0x74

// Fields
#define FESPI_IP_TXWM             0x1
#define FESPI_FMT_DIR(x)          (((x) & 0x1) << 3)

// To enter, jump to the start of command_table (ie. offset 0).
//      a0 - FESPI base address
//      a1 - start address of buffer

// The buffer contains a "program" in byte sequences. The first byte in a
// sequence determines the operation. Some operation will read more data from
// the program, while some will not. The operation byte is the offset into
// command_table, so eg. 4 means exit, 8 means transmit, and so on.

		.global _start
_start:
command_table:
        j       main            // 0
        ebreak                  // 4
        j       tx              // 8
        j       txwm_wait       // 12
        j       write_reg       // 16
		j		wip_wait		// 20
		j		set_dir 		// 24

// Execute the program.
main:
        lbu     t0, 0(a1)
        addi    a1, a1, 1
        la      t1, command_table
        add     t0, t0, t1
        jr      t0

// Read 1 byte the contains the number of bytes to transmit. Then read those
// bytes from the program and transmit them one by one.
tx:
        lbu     t1, 0(a1)       // read number of bytes to transmit
        addi    a1, a1, 1
1:      lw      t0, FESPI_REG_TXFIFO(a0)        // wait for FIFO clear
        bltz    t0, 1b
        lbu     t0, 0(a1)       // Load byte to write
        sw      t0, FESPI_REG_TXFIFO(a0)
        addi    a1, a1, 1
        addi    t1, t1, -1
        bgtz    t1, 1b
        j       main

// Wait until TXWM is set.
txwm_wait:
1:      lw      t0, FESPI_REG_IP(a0)
        andi    t0, t0, FESPI_IP_TXWM
        beqz    t0, 1b
        j       main

// Read 1 byte that contains the offset of the register to write, and 1 byte
// that contains the data to write.
write_reg:
        lbu     t0, 0(a1)       // read register to write
        add     t0, t0, a0
        lbu     t1, 1(a1)       // read value to write
        addi    a1, a1, 2
        sw      t1, 0(t0)
        j       main

wip_wait:
		li		a2, SPIFLASH_READ_STATUS
        jal		txrx_byte
		// discard first result
1:		li		a2, 0
        jal		txrx_byte
		andi	t0, a2, SPIFLASH_BSY_BIT
		bnez	t0, 1b
		j		main

txrx_byte:	// transmit the byte in a2, receive a bit into a2
		lw      t0, FESPI_REG_TXFIFO(a0)        // wait for FIFO clear
        bltz    t0, txrx_byte
        sw      a2, FESPI_REG_TXFIFO(a0)
1:		lw		a2, FESPI_REG_RXFIFO(a0)
		bltz	a2, 1b
		ret

set_dir:
		lw		t0, FESPI_REG_FMT(a0)
		li		t1, ~(FESPI_FMT_DIR(0xFFFFFFFF))
		and		t0, t0, t1
        lbu     t1, 0(a1)       // read value to OR in
        addi    a1, a1, 1
		or		t0, t0, t1
		sw		t0, FESPI_REG_FMT(a0)
		j		main

// ALGO_END
 */
static const uint8_t algorithm_bin[] = {
  0x6f, 0x00, 0xc0, 0x01, 0x73, 0x00, 0x10, 0x00, 0x6f, 0x00, 0xc0, 0x02,
  0x6f, 0x00, 0x00, 0x05, 0x6f, 0x00, 0xc0, 0x05, 0x6f, 0x00, 0x00, 0x07,
  0x6f, 0x00, 0x00, 0x0a, 0x83, 0xc2, 0x05, 0x00, 0x93, 0x85, 0x15, 0x00,
  0x17, 0x03, 0x00, 0x00, 0x13, 0x03, 0xc3, 0xfd, 0xb3, 0x82, 0x62, 0x00,
  0x67, 0x80, 0x02, 0x00, 0x03, 0xc3, 0x05, 0x00, 0x93, 0x85, 0x15, 0x00,
  0x83, 0x22, 0x85, 0x04, 0xe3, 0xce, 0x02, 0xfe, 0x83, 0xc2, 0x05, 0x00,
  0x23, 0x24, 0x55, 0x04, 0x93, 0x85, 0x15, 0x00, 0x13, 0x03, 0xf3, 0xff,
  0xe3, 0x44, 0x60, 0xfe, 0x6f, 0xf0, 0x5f, 0xfc, 0x83, 0x22, 0x45, 0x07,
  0x93, 0xf2, 0x12, 0x00, 0xe3, 0x8c, 0x02, 0xfe, 0x6f, 0xf0, 0x5f, 0xfb,
  0x83, 0xc2, 0x05, 0x00, 0xb3, 0x82, 0xa2, 0x00, 0x03, 0xc3, 0x15, 0x00,
  0x93, 0x85, 0x25, 0x00, 0x23, 0xa0, 0x62, 0x00, 0x6f, 0xf0, 0xdf, 0xf9,
  0x13, 0x06, 0x50, 0x00, 0xef, 0x00, 0x80, 0x01, 0x13, 0x06, 0x00, 0x00,
  0xef, 0x00, 0x00, 0x01, 0x93, 0x72, 0x16, 0x00, 0xe3, 0x9a, 0x02, 0xfe,
  0x6f, 0xf0, 0x1f, 0xf8, 0x83, 0x22, 0x85, 0x04, 0xe3, 0xce, 0x02, 0xfe,
  0x23, 0x24, 0xc5, 0x04, 0x03, 0x26, 0xc5, 0x04, 0xe3, 0x4e, 0x06, 0xfe,
  0x67, 0x80, 0x00, 0x00, 0x83, 0x22, 0x05, 0x04, 0x13, 0x03, 0x70, 0xff,
  0xb3, 0xf2, 0x62, 0x00, 0x03, 0xc3, 0x05, 0x00, 0x93, 0x85, 0x15, 0x00,
  0xb3, 0xe2, 0x62, 0x00, 0x23, 0x20, 0x55, 0x04, 0x6f, 0xf0, 0x9f, 0xf4
};
#define STEP_EXIT			4
#define STEP_TX				8
#define STEP_TXWM_WAIT		12
#define STEP_WRITE_REG		16
#define STEP_WIP_WAIT		20
#define STEP_SET_DIR		24
#define STEP_NOP			0xff

struct algorithm_steps {
	unsigned size;
	unsigned used;
	uint8_t **steps;
};

struct algorithm_steps *as_new(unsigned size)
{
	struct algorithm_steps *as = calloc(1, sizeof(struct algorithm_steps));
	as->size = size;
	as->steps = calloc(size, sizeof(as->steps[0]));
	return as;
}

struct algorithm_steps *as_delete(struct algorithm_steps *as)
{
	for (unsigned step = 0; step < as->used; step++) {
		free(as->steps[step]);
		as->steps[step] = NULL;
	}
	free(as);
	return NULL;
}

int as_empty(struct algorithm_steps *as)
{
	for (unsigned s = 0; s < as->used; s++) {
		if (as->steps[s][0] != STEP_NOP)
			return 0;
	}
	return 1;
}

// Return size of compiled program.
unsigned as_compile(struct algorithm_steps *as, uint8_t *target,
		unsigned target_size)
{
	unsigned offset = 0;
	bool finish_early = false;
	for (unsigned s = 0; s < as->used && !finish_early; s++) {
		unsigned bytes_left = target_size - offset;
		switch (as->steps[s][0]) {
			case STEP_NOP:
				break;
			case STEP_TX:
				{
					unsigned size = as->steps[s][1];
					if (size + 3 > bytes_left) {
						finish_early = true;
						break;
					}
					memcpy(target + offset, as->steps[s], size + 2);
					offset += size + 2;
					break;
				}
			case STEP_WRITE_REG:
				if (4 > bytes_left) {
					finish_early = true;
					break;
				}
				memcpy(target + offset, as->steps[s], 3);
				offset += 3;
				break;
			case STEP_SET_DIR:
				if (3 > bytes_left) {
					finish_early = true;
					break;
				}
				memcpy(target + offset, as->steps[s], 2);
				offset += 2;
				break;
			case STEP_TXWM_WAIT:
			case STEP_WIP_WAIT:
				if (2 > bytes_left) {
					finish_early = true;
					break;
				}
				memcpy(target + offset, as->steps[s], 1);
				offset += 1;
				break;
			default:
				assert(0);
		}
		if (!finish_early)
			as->steps[s][0] = STEP_NOP;
	}
	assert(offset + 1 <= target_size);
	target[offset++] = STEP_EXIT;

	LOG_DEBUG("%d-byte program:", offset);
	for (unsigned i = 0; i < offset;) {
		char buf[80];
		for (unsigned x = 0; i < offset && x < 16; x++, i++) {
			sprintf(buf + x*3, "%02x ", target[i]);
		}
		LOG_DEBUG("%s", buf);
	}

	return offset;
}

void as_add_tx(struct algorithm_steps *as, unsigned count, const uint8_t *data)
{
	LOG_DEBUG("count=%d", count);
	while (count > 0) {
		unsigned step_count = MIN(count, 255);
		assert(as->used < as->size);
		as->steps[as->used] = malloc(step_count + 2);
		as->steps[as->used][0] = STEP_TX;
		as->steps[as->used][1] = step_count;
		memcpy(as->steps[as->used] + 2, data, step_count);
		as->used++;
		data += step_count;
		count -= step_count;
	}
}

void as_add_tx1(struct algorithm_steps *as, uint8_t byte)
{
	uint8_t data[1];
	data[0] = byte;
	as_add_tx(as, 1, data);
}

void as_add_write_reg(struct algorithm_steps *as, uint8_t offset, uint8_t data)
{
	assert(as->used < as->size);
	as->steps[as->used] = malloc(3);
	as->steps[as->used][0] = STEP_WRITE_REG;
	as->steps[as->used][1] = offset;
	as->steps[as->used][2] = data;
	as->used++;
}

void as_add_txwm_wait(struct algorithm_steps *as)
{
	assert(as->used < as->size);
	as->steps[as->used] = malloc(1);
	as->steps[as->used][0] = STEP_TXWM_WAIT;
	as->used++;
}

void as_add_wip_wait(struct algorithm_steps *as)
{
	assert(as->used < as->size);
	as->steps[as->used] = malloc(1);
	as->steps[as->used][0] = STEP_WIP_WAIT;
	as->used++;
}

void as_add_set_dir(struct algorithm_steps *as, bool dir)
{
	assert(as->used < as->size);
	as->steps[as->used] = malloc(2);
	as->steps[as->used][0] = STEP_SET_DIR;
	as->steps[as->used][1] = FESPI_FMT_DIR(dir);
	as->used++;
}

/* This should write something less than or equal to a page.*/
static int steps_add_buffer_write(struct algorithm_steps *as,
		const uint8_t *buffer, uint32_t chip_offset, uint32_t len)
{
	as_add_tx1(as, SPIFLASH_WRITE_ENABLE);
	as_add_txwm_wait(as);
	as_add_write_reg(as, FESPI_REG_CSMODE, FESPI_CSMODE_HOLD);

	uint8_t setup[] = {
		SPIFLASH_PAGE_PROGRAM,
		chip_offset >> 16,
		chip_offset >> 8,
		chip_offset,
	};
	as_add_tx(as, sizeof(setup), setup);

	as_add_tx(as, len, buffer);
	as_add_txwm_wait(as);
	as_add_write_reg(as, FESPI_REG_CSMODE, FESPI_CSMODE_AUTO);

	// fespi_wip()
	as_add_set_dir(as, FESPI_DIR_RX);
	as_add_write_reg(as, FESPI_REG_CSMODE, FESPI_CSMODE_HOLD);
	as_add_wip_wait(as);
	as_add_write_reg(as, FESPI_REG_CSMODE, FESPI_CSMODE_AUTO);
	as_add_set_dir(as, FESPI_DIR_TX);

	return ERROR_OK;
}

static int steps_execute(struct algorithm_steps *as,
		struct flash_bank *bank, struct working_area *algorithm_wa,
		struct working_area *data_wa)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t ctrl_base = fespi_info->ctrl_base;
	uint8_t *data_buf = malloc(data_wa->size);
	int xlen = riscv_xlen(target);

	struct reg_param reg_params[2];
	init_reg_param(&reg_params[0], "x10", xlen, PARAM_OUT);
	init_reg_param(&reg_params[1], "x11", xlen, PARAM_OUT);
	buf_set_u64(reg_params[0].value, 0, xlen, ctrl_base);
	buf_set_u64(reg_params[1].value, 0, xlen, data_wa->address);
	while (!as_empty(as)) {
		keep_alive();
		unsigned bytes = as_compile(as, data_buf, data_wa->size);
		int retval = target_write_buffer(target, data_wa->address, bytes,
				data_buf);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write data to 0x%x: %d", data_wa->address,
					retval);
			return retval;
		}

		retval = target_run_algorithm(target, 0, NULL, 2, reg_params,
				algorithm_wa->address, algorithm_wa->address + 4,
				10000, NULL);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to execute algorithm at 0x%x: %d", algorithm_wa->address,
					retval);
			return retval;
		}
	}

	return ERROR_OK;
}

static int fespi_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t ctrl_base = fespi_info->ctrl_base;
	uint32_t cur_count, page_size, page_offset;
	int sector;
	int retval = ERROR_OK;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32,
			__func__, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > fespi_info->dev->size_in_bytes) {
		LOG_WARNING("Write past end of flash. Extra data discarded.");
		count = fespi_info->dev->size_in_bytes - offset;
	}

	/* Check sector protection */
	for (sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset <
					(bank->sectors[sector].offset + bank->sectors[sector].size))
				&& ((offset + count - 1) >= bank->sectors[sector].offset)
				&& bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %d protected", sector);
			return ERROR_FAIL;
		}
	}

	struct working_area *algorithm_wa;
	if (target_alloc_working_area(target, sizeof(algorithm_bin),
				&algorithm_wa) != ERROR_OK) {
		LOG_WARNING("Couldn't allocate %zd-byte working area.",
				sizeof(algorithm_bin));
		algorithm_wa = NULL;
	} else {
		retval = target_write_buffer(target, algorithm_wa->address,
				sizeof(algorithm_bin), algorithm_bin);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write code to 0x%x: %d", algorithm_wa->address,
					retval);
			target_free_working_area(target, algorithm_wa);
			algorithm_wa = NULL;
		}
	}

	struct working_area *data_wa = NULL;
	unsigned data_wa_size = 2 * count;
	while (1) {
		if (data_wa_size < 128) {
			LOG_WARNING("Couldn't allocate data working area.");
			target_free_working_area(target, algorithm_wa);
			algorithm_wa = NULL;
		}
		if (target_alloc_working_area_try(target, data_wa_size, &data_wa) ==
				ERROR_OK) {
			break;
		}

		data_wa_size /= 2;
	}

	page_size = fespi_info->dev->pagesize;

	fespi_txwm_wait(bank);

	/* Disable Hardware accesses*/
	FESPI_DISABLE_HW_MODE();

	/* poll WIP */
	retval = fespi_wip(bank, FESPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	struct algorithm_steps *as = as_new(count / 4);

	/* unaligned buffer head */
	if (count > 0 && (offset & 3) != 0) {
		cur_count = 4 - (offset & 3);
		if (cur_count > count)
			cur_count = count;
		if (algorithm_wa) {
			retval = steps_add_buffer_write(as, buffer, offset, cur_count);
		} else {
			retval = slow_fespi_write_buffer(bank, buffer, offset, cur_count);
		}
		if (retval != ERROR_OK)
			goto err;
		offset += cur_count;
		buffer += cur_count;
		count -= cur_count;
	}

	page_offset = offset % page_size;
	/* central part, aligned words */
	while (count >= 4) {
		/* clip block at page boundary */
		if (page_offset + count > page_size)
			cur_count = page_size - page_offset;
		else
			cur_count = count & ~3;

		if (algorithm_wa) {
			retval = steps_add_buffer_write(as, buffer, offset, cur_count);
		} else {
			retval = slow_fespi_write_buffer(bank, buffer, offset, cur_count);
		}
		if (retval != ERROR_OK)
			goto err;

		page_offset = 0;
		buffer += cur_count;
		offset += cur_count;
		count -= cur_count;
	}

	/* buffer tail */
	if (count > 0) {
		if (algorithm_wa) {
			retval = steps_add_buffer_write(as, buffer, offset, count);
		} else {
			retval = slow_fespi_write_buffer(bank, buffer, offset, count);
		}
		if (retval != ERROR_OK)
			goto err;
	}

	if (algorithm_wa) {
		retval = steps_execute(as, bank, algorithm_wa, data_wa);
	}

err:
	if (algorithm_wa) {
		target_free_working_area(target, data_wa);
		target_free_working_area(target, algorithm_wa);
	}

	/* Switch to HW mode before return to prompt */
	FESPI_ENABLE_HW_MODE();
	return retval;
}

/* Return ID of flash device */
/* On exit, SW mode is kept */
static int fespi_read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t ctrl_base = fespi_info->ctrl_base;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	fespi_txwm_wait(bank);

	/* Disable Hardware accesses*/
	FESPI_DISABLE_HW_MODE();

	/* poll WIP */
	retval = fespi_wip(bank, FESPI_PROBE_TIMEOUT);
	if (retval != ERROR_OK)
		return retval;

	fespi_set_dir(bank, FESPI_DIR_RX);

	/* Send SPI command "read ID" */
	FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_HOLD);

	fespi_tx(bank, SPIFLASH_READ_ID);
	/* Send dummy bytes to actually read the ID.*/
	fespi_tx(bank, 0);
	fespi_tx(bank, 0);
	fespi_tx(bank, 0);

	/* read ID from Receive Register */
	*id = 0;
	if (fespi_rx(bank, NULL) != ERROR_OK)
		return ERROR_FAIL;
	uint8_t rx;
	if (fespi_rx(bank, &rx) != ERROR_OK)
		return ERROR_FAIL;
	*id = rx;
	if (fespi_rx(bank, &rx) != ERROR_OK)
		return ERROR_FAIL;
	*id |= (rx << 8);
	if (fespi_rx(bank, &rx) != ERROR_OK)
		return ERROR_FAIL;
	*id |= (rx << 16);

	FESPI_WRITE_REG(FESPI_REG_CSMODE, FESPI_CSMODE_AUTO);

	fespi_set_dir(bank, FESPI_DIR_TX);

	return ERROR_OK;
}

static int fespi_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	uint32_t ctrl_base;
	struct flash_sector *sectors;
	uint32_t id = 0; /* silence uninitialized warning */
	const struct fespi_target *target_device;
	int retval;

	if (fespi_info->probed)
		free(bank->sectors);
	fespi_info->probed = 0;

	if (fespi_info->ctrl_base == 0) {
	  for (target_device = target_devices ; target_device->name ; ++target_device)
	    if (target_device->tap_idcode == target->tap->idcode)
	      break;

	  if (!target_device->name) {
	    LOG_ERROR("Device ID 0x%" PRIx32 " is not known as FESPI capable",
		      target->tap->idcode);
	    return ERROR_FAIL;
	  }

	  fespi_info->ctrl_base = target_device->ctrl_base;

	  LOG_DEBUG("Valid FESPI on device %s at address 0x%" PRIx32,
		    target_device->name, bank->base);

	} else {
	  LOG_DEBUG("Assuming FESPI as specified at address 0x%x with ctrl at 0x%x",
		    fespi_info->ctrl_base,
		    bank->base);
	}
        ctrl_base = fespi_info->ctrl_base;

	/* read and decode flash ID; returns in SW mode */
	FESPI_WRITE_REG(FESPI_REG_TXCTRL, FESPI_TXWM(1));
	fespi_set_dir(bank, FESPI_DIR_TX);

	retval = fespi_read_flash_id(bank, &id);

	FESPI_ENABLE_HW_MODE();
	if (retval != ERROR_OK)
		return retval;

	fespi_info->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name ; p++)
		if (p->device_id == id) {
			fespi_info->dev = p;
			break;
		}

	if (!fespi_info->dev) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", id);
		return ERROR_FAIL;
	}

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")",
			fespi_info->dev->name, fespi_info->dev->device_id);

	/* Set correct size value */
	bank->size = fespi_info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors =
		fespi_info->dev->size_in_bytes / fespi_info->dev->sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (sectors == NULL) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	for (int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * fespi_info->dev->sectorsize;
		sectors[sector].size = fespi_info->dev->sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 1;
	}

	bank->sectors = sectors;
	fespi_info->probed = 1;
	return ERROR_OK;
}

static int fespi_auto_probe(struct flash_bank *bank)
{
	struct fespi_flash_bank *fespi_info = bank->driver_priv;
	if (fespi_info->probed)
		return ERROR_OK;
	return fespi_probe(bank);
}

static int fespi_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

static int get_fespi_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct fespi_flash_bank *fespi_info = bank->driver_priv;

	if (!(fespi_info->probed)) {
		snprintf(buf, buf_size,
				"\nFESPI flash bank not probed yet\n");
		return ERROR_OK;
	}

	snprintf(buf, buf_size, "\nFESPI flash information:\n"
			"  Device \'%s\' (ID 0x%08" PRIx32 ")\n",
			fespi_info->dev->name, fespi_info->dev->device_id);

	return ERROR_OK;
}

struct flash_driver fespi_flash = {
	.name = "fespi",
	.flash_bank_command = fespi_flash_bank_command,
	.erase = fespi_erase,
	.protect = fespi_protect,
	.write = fespi_write,
	.read = default_flash_read,
	.probe = fespi_probe,
	.auto_probe = fespi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = fespi_protect_check,
	.info = get_fespi_info
};
