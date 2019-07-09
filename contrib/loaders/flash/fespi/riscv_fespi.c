#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "../../../../src/flash/nor/spi.h"

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
/* Timeout in target "loops" */
#define FESPI_CMD_TIMEOUT         1000
#define FESPI_PROBE_TIMEOUT       1000
#define FESPI_MAX_TIMEOUT         30000

enum {
	ERROR_OK,
	ERROR_FAIL
};

static int fespi_txwm_wait(volatile uint32_t *ctrl_base);
static void fespi_disable_hw_mode(volatile uint32_t *ctrl_base);
static void fespi_enable_hw_mode(volatile uint32_t *ctrl_base);
static int fespi_wip(volatile uint32_t *ctrl_base);
static int slow_fespi_write_buffer(volatile uint32_t *ctrl_base,
		const uint8_t *buffer, unsigned offset, unsigned len);

int main(volatile uint32_t *ctrl_base, uint32_t page_size,
		const uint8_t *buffer, unsigned offset, uint32_t count)
{
	fespi_txwm_wait(ctrl_base);

	/* Disable Hardware accesses*/
	fespi_disable_hw_mode(ctrl_base);

	/* poll WIP */
	int retval = fespi_wip(ctrl_base);
	if (retval != ERROR_OK)
		goto err;

	/* Assume page_size is a power of two so we don't need the modulus code. */
	uint32_t page_offset = offset & (page_size - 1);

	/* central part, aligned words */
	while (count > 0) {
		uint32_t cur_count;
		/* clip block at page boundary */
		if (page_offset + count > page_size)
			cur_count = page_size - page_offset;
		else
			cur_count = count;

		retval = slow_fespi_write_buffer(ctrl_base, buffer, offset, cur_count);
		if (retval != ERROR_OK)
			goto err;

		page_offset = 0;
		buffer += cur_count;
		offset += cur_count;
		count -= cur_count;
	}

err:
	/* Switch to HW mode before return to prompt */
	fespi_enable_hw_mode(ctrl_base);

	return retval;
}

static uint32_t fespi_read_reg(volatile uint32_t *ctrl_base, unsigned address)
{
	return ctrl_base[address / 4];
}

static void fespi_write_reg(volatile uint32_t *ctrl_base, unsigned address, uint32_t value)
{
	ctrl_base[address / 4] = value;
}

static void fespi_disable_hw_mode(volatile uint32_t *ctrl_base)
{
	uint32_t fctrl = fespi_read_reg(ctrl_base, FESPI_REG_FCTRL);
	fespi_write_reg(ctrl_base, FESPI_REG_FCTRL, fctrl & ~FESPI_FCTRL_EN);
}

static void fespi_enable_hw_mode(volatile uint32_t *ctrl_base)
{
	uint32_t fctrl = fespi_read_reg(ctrl_base, FESPI_REG_FCTRL);
	fespi_write_reg(ctrl_base, FESPI_REG_FCTRL, fctrl | FESPI_FCTRL_EN);
}

static int fespi_txwm_wait(volatile uint32_t *ctrl_base)
{
	unsigned timeout = 1000;

	while (timeout--) {
		uint32_t ip = fespi_read_reg(ctrl_base, FESPI_REG_IP);
		if (ip & FESPI_IP_TXWM)
			return ERROR_OK;
	}

	return ERROR_FAIL;
}

static void fespi_set_dir(volatile uint32_t *ctrl_base, bool dir)
{
	uint32_t fmt = fespi_read_reg(ctrl_base, FESPI_REG_FMT);
	fespi_write_reg(ctrl_base, FESPI_REG_FMT,
			(fmt & ~(FESPI_FMT_DIR(0xFFFFFFFF))) | FESPI_FMT_DIR(dir));
}

static int fespi_tx(volatile uint32_t *ctrl_base, uint8_t in)
{
	unsigned timeout = 1000;

	while (timeout--) {
		uint32_t txfifo = fespi_read_reg(ctrl_base, FESPI_REG_TXFIFO);
		if (!(txfifo >> 31)) {
			fespi_write_reg(ctrl_base, FESPI_REG_TXFIFO, in);
			return ERROR_OK;
		}
	}
	return ERROR_FAIL;
}

static int fespi_rx(volatile uint32_t *ctrl_base, uint8_t *out)
{
	unsigned timeout = 1000;

	while (timeout--) {
		uint32_t value = fespi_read_reg(ctrl_base, FESPI_REG_RXFIFO);
		if (!(value >> 31)) {
			if (out)
				*out = value & 0xff;
			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

static int fespi_wip(volatile uint32_t *ctrl_base)
{
	fespi_set_dir(ctrl_base, FESPI_DIR_RX);

	fespi_write_reg(ctrl_base, FESPI_REG_CSMODE, FESPI_CSMODE_HOLD);

	fespi_tx(ctrl_base, SPIFLASH_READ_STATUS);
	if (fespi_rx(ctrl_base, NULL) != ERROR_OK)
		return ERROR_FAIL;

	unsigned timeout = 1000;
	while (timeout--) {
		fespi_tx(ctrl_base, 0);
		uint8_t rx;
		if (fespi_rx(ctrl_base, &rx) != ERROR_OK)
			return ERROR_FAIL;
		if ((rx & SPIFLASH_BSY_BIT) == 0) {
			fespi_write_reg(ctrl_base, FESPI_REG_CSMODE, FESPI_CSMODE_AUTO);
			fespi_set_dir(ctrl_base, FESPI_DIR_TX);
			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

static int slow_fespi_write_buffer(volatile uint32_t *ctrl_base,
		const uint8_t *buffer, unsigned offset, unsigned len)
{
	fespi_tx(ctrl_base, SPIFLASH_WRITE_ENABLE);
	fespi_txwm_wait(ctrl_base);

	fespi_write_reg(ctrl_base, FESPI_REG_CSMODE, FESPI_CSMODE_HOLD);

	fespi_tx(ctrl_base, SPIFLASH_PAGE_PROGRAM);

	fespi_tx(ctrl_base, offset >> 16);
	fespi_tx(ctrl_base, offset >> 8);
	fespi_tx(ctrl_base, offset);

	for (unsigned i = 0; i < len; i++)
		fespi_tx(ctrl_base, buffer[i]);

	fespi_txwm_wait(ctrl_base);

	fespi_write_reg(ctrl_base, FESPI_REG_CSMODE, FESPI_CSMODE_AUTO);

	return fespi_wip(ctrl_base);
}
