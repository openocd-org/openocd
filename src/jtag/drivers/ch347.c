/***************************************************************************
 *   Driver for CH347-JTAG interface V1.2                                  *
 *                                                                         *
 *   Copyright (C) 2022 Nanjing Qinheng Microelectronics Co., Ltd.         *
 *   Web: http://wch.cn                                                    *
 *   Author: WCH@TECH53 <tech@wch.cn>                                      *
 *                                                                         *
 *   Copyright (C) 2022 by oidcat                                          *
 *   Author: oidcatiot@163.com                                             *
 *                                                                         *
 *   Copyright (C) 2023 by coflery                                         *
 *   Author: coflery@gmail.com                                             *
 *                                                                         *
 *   CH347 is a high-speed USB bus converter chip that provides UART, I2C  *
 *   and SPI synchronous serial ports and JTAG interface through USB bus.  *
 *                                                                         *
 *   The Jtag interface by CH347 can supports transmission frequency       *
 *   configuration up to 60MHz.                                            *
 *                                                                         *
 *   The USB2.0 to JTAG scheme based on CH347 can be used to build         *
 *   customized USB high-speed JTAG debugger and other products.           *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *            _____________                                                *
 *           |             |____JTAG(TDO,TDI,TMS,TCK,TRST)                 *
 *      USB__|    CH347T   |                                               *
 *           |_____________|____SWD(SWCLK/TCK,SWDIO/TMS)                   *
 *           |             |                                               *
 *           |_____________|____UART(TXD1,RXD1,RTS1,CTS1,DTR1)             *
 *            ______|______                                                *
 *           |             |                                               *
 *           | 8 MHz XTAL  |                                               *
 *           |_____________|                                               *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if IS_CYGWIN == 1
#include "windows.h"
#undef LOG_ERROR
#endif

/* project specific includes */
#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <jtag/swd.h>
#include <helper/time_support.h>
#include <helper/replacements.h>
#include <helper/list.h>
#include <helper/binarybuffer.h>
#include "libusb_helper.h"

/* system includes */
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/time.h>
#include <time.h>

#define KHZ(n)	((n)*UINT64_C(1000))
#define MHZ(n)	((n)*UINT64_C(1000000))
#define GHZ(n)	((n)*UINT64_C(1000000000))

#define JTAGIO_STA_OUT_TDI  (0x10)
#define JTAGIO_STA_OUT_TMS  (0x02)
#define JTAGIO_STA_OUT_TCK  (0x01)
#define JTAGIO_STA_OUT_TRST (0x20)
#define TDI_H				JTAGIO_STA_OUT_TDI
#define TMS_H				JTAGIO_STA_OUT_TMS
#define TCK_H				JTAGIO_STA_OUT_TCK
#define TRST_H				JTAGIO_STA_OUT_TRST
#define TDI_L				0
#define TMS_L				0
#define TCK_L				0
#define TRST_L				0

#define VENDOR_VERSION	0x5F

/* Format: CMD(1byte) + Length(2bytes) + Data(Nbytes) */
#define HW_TDO_BUF_SIZE					4096
#define SF_PACKET_BUF_SIZE				51200 /* Command packet length */
#define UCMDPKT_DATA_MAX_BYTES_USBHS	507   /* Maximum command length */
#define USBC_PACKET_USBHS				512   /* Maximum data length in packet */
#define USBC_PACKET_USBHS_SINGLE		510   /* Maximum package length */
#define CH347_CMD_HEADER				3     /* Protocol header length */

/* JTAG */
#define CH347_CMD_INFO_RD			 0xCA
#define CH347_CMD_JTAG_INIT			 0xD0 /* JTAG init command */
#define CH347_CMD_JTAG_BIT_OP		 0xD1 /* JTAG pin bit control */
#define CH347_CMD_JTAG_BIT_OP_RD	 0xD2 /* JTAG pin bit control and read */
#define CH347_CMD_JTAG_DATA_SHIFT	 0xD3 /* JTAG data shift */
#define CH347_CMD_JTAG_DATA_SHIFT_RD 0xD4 /* JTAG data shift and read */
/* SWD */
#define CH347_CMD_SWD_INIT	0xE5 /* SWD init command */
#define CH347_CMD_SWD		0xE8
#define CH347_CMD_SWD_REG_W	0xA0 /* SWD write reg */
#define CH347_CMD_SWD_SEQ_W	0xA1 /* SWD write spec seq */
#define CH347_CMD_SWD_REG_R	0xA2 /* SWD read reg */
#define CH347_MAX_SEND_CMD	0X20 /* max send cmd number */
#define CH347_MAX_SEND_BUF	512
#define CH347_MAX_RECV_BUF	512
#define BUILD_UINT16(loByte, hiByte) \
	((uint16_t)(((loByte)&0xFF) + (((hiByte)&0xFF) << 8)))

#pragma pack(1)
typedef unsigned char UCHAR;

typedef enum pack {
	STANDARD_PACK = 0,
	LARGER_PACK = 1,
} PACK_SIZE;

typedef struct _CH347_info /* Record the CH347 pin status */
{
	int TMS;
	int TDI;
	int TCK;
	int TRST;

	int buffer_idx;
	uint8_t buffer[SF_PACKET_BUF_SIZE];

	int len_idx;
	int len_value;
	uint8_t lastCmd;

	uint8_t read_buffer[SF_PACKET_BUF_SIZE];
	uint32_t read_idx;
	uint32_t read_count;
	struct bit_copy_queue read_queue;
	PACK_SIZE pack_size;
} _CH347_Info;

int DevIsOpened; /* Whether the device is turned on */
bool UsbHighDev = true;

typedef struct _CH347_SWD_IO {
	uint8_t usbcmd; /* 0xA0、0xA1、0xA2 */
	uint8_t cmd;
	uint32_t *dst;
	uint32_t value;
	struct list_head list_entry;
} CH347_SWD_IO, *PCH347_SWD_IO;

typedef struct _CH347_SWD_CONTEXT {
	uint8_t send_buf[CH347_MAX_SEND_BUF];
	uint8_t recv_buf[CH347_MAX_RECV_BUF];
	uint32_t send_len;
	uint32_t recv_len;
	uint32_t need_recv_len;
	int queued_retval;
	uint8_t sent_cmd_count;
	struct list_head send_cmd_head;
	struct list_head free_cmd_head;
	uint8_t *ch347_cmd_buf;
} CH347_SWD_CONTEXT;
static CH347_SWD_CONTEXT ch347_swd_context;
static bool swd_mode;
#pragma pack()

#include <jtag/drivers/libusb_helper.h>

#define CH347_EPOUT           0x06u
#define CH347_EPIN            0x86u
#define CH347_MPHSI_INTERFACE 2

bool ugOpen;
unsigned long ugIndex;
struct libusb_device_handle *ch347_handle;

uint32_t usb_write_timeout = 500;
uint32_t usb_read_timeout = 500;
static uint16_t ch347_vid = 0x1a86; /* WCH */
static uint16_t ch347_pid = 0x55dd; /* CH347 */

static uint32_t CH347OpenDevice(uint64_t iIndex)
{
	uint16_t vids[] = {ch347_vid, 0};
	uint16_t pids[] = {ch347_pid, 0};

	if (jtag_libusb_open(vids, pids, &ch347_handle, NULL))
	{
		LOG_ERROR("ch347 not found: vid=%04x, pid=%04x",
				  ch347_vid, ch347_pid);
		return false;
	}

	if (libusb_claim_interface(ch347_handle, CH347_MPHSI_INTERFACE))
	{
		LOG_ERROR("ch347 unable to claim interface");
		return false;
	}

	char ver;
	if (jtag_libusb_control_transfer(ch347_handle,
				LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
				VENDOR_VERSION, 0, 0, &ver, sizeof(ver), usb_write_timeout, NULL) != ERROR_OK)
	{
		LOG_ERROR("ch347 unable to get firmware version");
		return false;
	}
	LOG_INFO("CH347 found (Firmware=0x%02X)", ver);

	return true;
}

static bool CH347WriteData(uint64_t iIndex, uint8_t *data, uint32_t *length)
{
	int ret, tmp = 0;
	ret = jtag_libusb_bulk_write(ch347_handle,
								 CH347_EPOUT,
								 (char *)data,
								 (int)(*length),
								 usb_write_timeout, &tmp);
	*length = (uint32_t)tmp;

	if (!ret)
		return true;
	else
		return false;
}

static bool CH347ReadData(uint64_t iIndex, uint8_t *data, uint32_t *length)
{
	int ret, tmp = 0;
	ret = jtag_libusb_bulk_read(ch347_handle,
								CH347_EPIN,
								(char *)data,
								(int)(*length),
								usb_read_timeout, &tmp);

	*length = (uint32_t)tmp;

	if (!ret)
		return true;
	else
		return false;
}

static bool CH347CloseDevice(uint64_t iIndex)
{
	jtag_libusb_close(ch347_handle);
	return true;
}

_CH347_Info ch347;

static int ch347_swd_run_queue(void);
/* swd init func */
static bool CH347SWD_INIT(uint64_t iIndex, uint8_t iClockRate)
{
	uint8_t cmdBuf[128] = "";
	uint32_t i = 0;
	cmdBuf[i++] = CH347_CMD_SWD_INIT;
	cmdBuf[i++] = 8; /* Data length is 6 */
	cmdBuf[i++] = 0;
	cmdBuf[i++] = 0x40;
	cmdBuf[i++] = 0x42;

	cmdBuf[i++] = 0x0f;			/* Reserved Bytes */
	cmdBuf[i++] = 0x00;			/* Reserved Bytes */
	cmdBuf[i++] = iClockRate;	/* JTAG clock speed */
	i += 3;						/* Reserved Bytes */

	uint32_t mLength = i;
	if (!CH347WriteData(iIndex, cmdBuf, &mLength) || (mLength != i))
		return false;

	mLength = 4;
	memset(cmdBuf, 0, sizeof(cmdBuf));

	if (!CH347ReadData(iIndex, cmdBuf, &mLength) || (mLength != 4))
		return false;

	return true;
}

/**
 *  HexToString - Hex Conversion String Function
 *  @param buf    Point to a buffer to place the Hex data to be converted
 *  @param size   Pointing to the length unit where data needs to be converted
 *
 *  @return Returns a converted string
 */
static char *HexToString(uint8_t *buf, uint32_t size)
{
	uint32_t i;
	if (buf == NULL)
		return "NULL";

	char *str = calloc(size * 2 + 1, 1);

	for (i = 0; i < size; i++)
		sprintf(str + 2 * i, "%02x ", buf[i]);
	return str;
}

/**
 *  CH347_Write - CH347 Write
 *  @param oBuffer    Point to a buffer to place the data to be written out
 *  @param ioLength   Pointing to the length unit, the input is the length to be
 *                    written out, and the return is the actual written length
 *
 *  @return Write success returns 1, failure returns 0
 */
static int CH347_Write(void *oBuffer, uint32_t *ioLength)
{
	int ret = -1;
	uint32_t wlength = *ioLength, WI;
	if (*ioLength >= HW_TDO_BUF_SIZE)
		wlength = HW_TDO_BUF_SIZE;
	WI = 0;
	while (1) {
		ret = CH347WriteData(ugIndex, (uint8_t *)oBuffer + WI, &wlength);
		if (!ret) {
			*ioLength = 0;
			return false;
		}
		LOG_DEBUG_IO("(size=%u, buf=[%s]) -> %" PRIu32, wlength,
					 HexToString((uint8_t *)oBuffer, wlength),
					 wlength);
		WI += wlength;
		if (WI >= *ioLength)
			break;
		if ((*ioLength - WI) > HW_TDO_BUF_SIZE)
			wlength = HW_TDO_BUF_SIZE;
		else
			wlength = *ioLength - WI;
	}

	*ioLength = WI;
	return true;
}

/**
 * CH347_Read - CH347 Read
 * @param oBuffer  Point to a buffer to place the data to be read in
 * @param ioLength Pointing to the length unit, the input is the length to
 *                 be read, and the return is the actual read length
 *
 * @return Write success returns 1, failure returns 0
 */
static int CH347_Read(void *oBuffer, uint32_t *ioLength)
{
	uint32_t rlength = *ioLength, WI;
	/* The maximum allowable reading for a single read is 4096B of data.
	   If it exceeds the allowable reading limit, it will be calculated as
	   4096B */
	if (rlength > HW_TDO_BUF_SIZE)
		rlength = HW_TDO_BUF_SIZE;
	WI = 0;

	while (1) {
		if (!CH347ReadData(ugIndex, (uint8_t *)oBuffer + WI,
				   &rlength)) {
			LOG_ERROR("CH347 read data fail");
			return false;
		}

		WI += rlength;
		if (WI >= *ioLength)
			break;
		if ((*ioLength - WI) > HW_TDO_BUF_SIZE)
			rlength = HW_TDO_BUF_SIZE;
		else
			rlength = *ioLength - WI;
	}
	LOG_DEBUG_IO("(size=%u, buf=[%s]) -> %" PRIu32, WI,
				 HexToString((uint8_t *)oBuffer, WI), (uint32_t)WI);
	*ioLength = WI;
	return true;
}

static void CH347_Read_Scan(UCHAR *pBuffer, uint32_t length)
{
	uint32_t read_size = 0;
	uint32_t RxLen = 0;
	unsigned long index = 0;
	unsigned long read_buf_index = 0;
	unsigned char *read_buf = NULL;
	int dataLen = 0, i = 0; /*, this_bits = 0; */

	read_size = length;
	RxLen = read_size;
	index = 0;
	read_buf_index = 0;
	read_buf = calloc(sizeof(unsigned char), read_size);
	if (!CH347_Read(read_buf, &RxLen)) {
		LOG_ERROR("CH347 read fail");
		return;
	}
	while (index < read_size) { /* deal with the CH347_CMD_JTAG_BIT_OP_RD  or  CH347_CMD_JTAG_DATA_SHIFT_RD */
		if (read_buf[index] == CH347_CMD_JTAG_DATA_SHIFT_RD) {
			dataLen = read_buf[++index] & 0xFF;
			dataLen += (read_buf[++index] & 0xFF) << 8;
			memcpy(pBuffer + read_buf_index, &read_buf[index + 1], dataLen);
			read_buf_index += dataLen;
			index += dataLen + 1;
		} else if (read_buf[index] == CH347_CMD_JTAG_BIT_OP_RD) {
			dataLen = read_buf[++index] & 0xFF;
			dataLen += (read_buf[++index] & 0xFF) << 8;

			for (i = 0; i < dataLen; i++) {
				if (read_buf[index + 1 + i] & 1)
					*(pBuffer + read_buf_index) |= (1 << i);
				else
					*(pBuffer + read_buf_index) &= ~(1 << i);
			}
			read_buf_index += 1;
			index += dataLen + 1;
		} else {
			LOG_ERROR("CH347 read command fail");
			*(pBuffer + read_buf_index) = read_buf[index];
			read_buf_index++;
			index++;
		}
	}
	if (read_buf) {
		free(read_buf);
		read_buf = NULL;
	}
}

static void CH347_Flush_Buffer(void)
{
	uint32_t retlen = (uint32_t)ch347.buffer_idx;
	int nb = ch347.buffer_idx, ret = ERROR_OK;

	while (ret == ERROR_OK && nb > 0) {
		ret = CH347_Write(ch347.buffer, &retlen);
		nb -= retlen;
	}
	memset(&ch347.buffer, 0, sizeof(ch347.buffer));
	ch347.buffer_idx = 0;
	ch347.lastCmd = 0;
	ch347.len_idx = 0;
	ch347.len_value = 0;

	if (ch347.read_count == 0)
		return;
	if (ch347.pack_size == LARGER_PACK) {
		CH347_Read_Scan(&ch347.read_buffer[0], ch347.read_count);
		bit_copy_execute(&ch347.read_queue);
		memset(ch347.read_buffer, 0, SF_PACKET_BUF_SIZE);
		ch347.read_count = 0;
		ch347.read_idx = 0;
	}
}

static void CH347_In_Buffer(uint8_t byte)
{
	if ((SF_PACKET_BUF_SIZE - ch347.buffer_idx) < 1)
		CH347_Flush_Buffer();
	ch347.buffer[ch347.buffer_idx] = byte;
	ch347.buffer_idx++;
	if ((SF_PACKET_BUF_SIZE - ch347.buffer_idx) == 0)
		CH347_Flush_Buffer();
}

static void CH347_In_Buffer_bytes(uint8_t *bytes, unsigned long bytes_length)
{
	if ((ch347.buffer_idx + bytes_length) > SF_PACKET_BUF_SIZE)
		CH347_Flush_Buffer();
	memcpy(&ch347.buffer[ch347.buffer_idx], bytes, bytes_length);
	ch347.buffer_idx += bytes_length;
	if ((SF_PACKET_BUF_SIZE - ch347.buffer_idx) < 1)
		CH347_Flush_Buffer();
}

static void combinePackets(uint8_t cmd, int cur_idx, unsigned long int len)
{
	if (cmd != ch347.lastCmd) {
		ch347.buffer[cur_idx] = cmd;
		ch347.buffer[cur_idx + 1] =
			(uint8_t)(((len - CH347_CMD_HEADER) >> 0) & 0xFF);
		ch347.buffer[cur_idx + 2] =
			(uint8_t)(((len - CH347_CMD_HEADER) >> 8) & 0xFF);

		/* update the ch347 struct */
		ch347.lastCmd = cmd;
		ch347.len_idx = cur_idx + 1;
		ch347.len_value = (len - CH347_CMD_HEADER);
	} else {
		/* update the ch347 struct cmd data leng */
		ch347.len_value += (len - CH347_CMD_HEADER);

		/* update the cmd packet valid leng */
		ch347.buffer[ch347.len_idx] = (uint8_t)((ch347.len_value >> 0) \
							& 0xFF);
		ch347.buffer[ch347.len_idx + 1] = (uint8_t)(
			(ch347.len_value >> 8) & 0xFF);

		/* update the buffer data leng */
		memcpy(&ch347.buffer[cur_idx],
			   &ch347.buffer[cur_idx + CH347_CMD_HEADER],
			   (len - CH347_CMD_HEADER));

		/* update the ch347 buffer index */
		ch347.buffer_idx -= CH347_CMD_HEADER;
	}
}
/**
 * CH347_ClockTms - Function function used to change the TMS value at the
 * rising edge of TCK to switch its Tap state
 * @param BitBangPkt Protocol package
 * @param tms TMS value to be changed
 * @param BI Protocol packet length
 *
 * @return Return protocol packet length
 */
static unsigned long CH347_ClockTms(int tms, unsigned long BI)
{
	uint8_t data = 0;
	unsigned char cmd = 0;

	if (tms == 1)
		cmd = TMS_H;
	else
		cmd = TMS_L;

	BI += 2;

	data = cmd | TDI_L | TCK_L | TRST_H;
	CH347_In_Buffer(data);
	data = cmd | TDI_L | TCK_H | TRST_H;
	CH347_In_Buffer(data);
	ch347.TMS = cmd;
	ch347.TDI = TDI_L;
	ch347.TCK = TCK_H;
	ch347.TRST = TRST_H;

	return BI;
}

/**
 * CH347_IdleClock - Function function to ensure that the clock is in a low state
 * @param BitBangPkt Protocol package
 * @param BI Protocol packet length
 *
 * @return Return protocol packet length
 */
static unsigned long CH347_IdleClock(unsigned long BI)
{
	unsigned char byte = 0;
	byte |= ch347.TMS ? TMS_H : TMS_L;
	byte |= ch347.TDI ? TDI_H : TDI_L;
	byte |= ch347.TRST ? TRST_H : TRST_L;
	BI++;
	CH347_In_Buffer(byte);

	return BI;
}

/**
 * CH347_TmsChange - Function function that performs state switching by changing the value of TMS
 * @param tmsValue The TMS values that need to be switched form one byte of data in the switching order
 * @param step The number of bit values that need to be read from the tmsValue value
 * @param skip Count from the skip bit of tmsValue to step
 *
 */
static void CH347_TmsChange(const unsigned char *tmsValue, int step, int skip)
{
	int i;
	int index = ch347.buffer_idx;
	unsigned long BI, retlen, cmdLen;

	BI = CH347_CMD_HEADER;
	retlen = CH347_CMD_HEADER;
	LOG_DEBUG_IO("(TMS Value: %02x..., step = %d, skip = %d)", tmsValue[0],
				 step, skip);

	for (i = 0; i < 3; i++)
		CH347_In_Buffer(0);

	for (i = skip; i < step; i++) {
		retlen = CH347_ClockTms((tmsValue[i / 8] >> (i % 8)) & 0x01, BI);
		BI = retlen;
	}
	cmdLen = CH347_IdleClock(BI);

	combinePackets(CH347_CMD_JTAG_BIT_OP, index, cmdLen);
}

/**
 * CH347_TMS - By ch347_ execute_ Queue call
 * @param cmd Upper layer transfer command parameters
 *
 */
static void CH347_TMS(struct tms_command *cmd)
{
	LOG_DEBUG_IO("(step: %d)", cmd->num_bits);
	CH347_TmsChange(cmd->bits, cmd->num_bits, 0);
}

/**
 * CH347_Reset - CH347 Reset Tap Status Function
 * @brief If there are more than six consecutive TCKs and TMS is high, the state
 *         machine can be set to a Test-Logic-Reset state
 *
 */
static int ch347_reset(int trst, int srst)
{
	LOG_DEBUG_IO("reset trst: %i srst %i", trst, srst);
#if 1
	unsigned char BitBang[512] = "", BII, i;
	uint32_t TxLen;

	BII = CH347_CMD_HEADER;
	for (i = 0; i < 7; i++) {
		BitBang[BII++] = TMS_H | TDI_L | TCK_L;
		BitBang[BII++] = TMS_H | TDI_L | TCK_H;
	}
	BitBang[BII++] = TMS_H | TDI_L | TCK_L;

	ch347.TCK = TCK_L;
	ch347.TDI = TDI_L;
	ch347.TMS = 0;

	BitBang[0] = CH347_CMD_JTAG_BIT_OP;
	BitBang[1] = BII - CH347_CMD_HEADER;
	BitBang[2] = 0;

	TxLen = BII;

	if (!CH347_Write(BitBang, &TxLen) && (TxLen != BII)) {
		LOG_ERROR("CH347 send reset command fail");
		return false;
	}
#else
	if (!swd_mode && trst == 0) {

		unsigned long int BI = 0;

		CH347_In_Buffer(CH347_CMD_JTAG_BIT_OP);
		CH347_In_Buffer(0x01);
		CH347_In_Buffer(0);

		ch347.TRST = 0;
		CH347_IdleClock(BI);

		CH347_Flush_Buffer();

		Sleep(50);

		CH347_In_Buffer(CH347_CMD_JTAG_BIT_OP);
		CH347_In_Buffer(0x01);
		CH347_In_Buffer(0);

		ch347.TRST = 1;
		CH347_IdleClock(BI);

		CH347_Flush_Buffer();
		return ERROR_OK;
	}
#endif
	return ERROR_OK;
}

/**
 * CH347_MovePath - Obtain the current Tap status and switch to the status TMS
 *                  value passed down by cmd
 * @param cmd Upper layer transfer command parameters
 *
 */
static void CH347_MovePath(struct pathmove_command *cmd)
{
	unsigned int i;
	int index = ch347.buffer_idx;
	unsigned long BI, retlen = 0, cmdLen;

	BI = CH347_CMD_HEADER;

	for (i = 0; i < 3; i++)
		CH347_In_Buffer(0);
	LOG_DEBUG_IO("(num_states=%d, last_state=%d)",
				 cmd->num_states, cmd->path[cmd->num_states - 1]);

	for (i = 0; i < cmd->num_states; i++) {
		if (tap_state_transition(tap_get_state(), false) == cmd->path[i])
			retlen = CH347_ClockTms(0, BI);
		BI = retlen;
		if (tap_state_transition(tap_get_state(), true) == cmd->path[i])
			retlen = CH347_ClockTms(1, BI);
		BI = retlen;
		tap_set_state(cmd->path[i]);
	}

	cmdLen = CH347_IdleClock(BI);

	combinePackets(CH347_CMD_JTAG_BIT_OP, index, cmdLen);
}

/**
 * CH347_MoveState - Toggle the Tap state to the Target state stat
 * @param stat Pre switch target path
 * @param skip Number of digits to skip
 *
 */
static void CH347_MoveState(tap_state_t state, int skip)
{
	uint8_t tms_scan;
	int tms_len;

	LOG_DEBUG_IO("(from %s to %s)", tap_state_name(tap_get_state()),
				 tap_state_name(state));
	if (tap_get_state() == state)
		return;
	tms_scan = tap_get_tms_path(tap_get_state(), state);
	tms_len = tap_get_tms_path_len(tap_get_state(), state);
	CH347_TmsChange(&tms_scan, tms_len, skip);
	tap_set_state(state);
}

/**
 * CH347_WriteRead - CH347 Batch read/write function
 * @param bits     Read and write data this time
 * @param nb_bits  Incoming data length
 * @param scan     The transmission method of incoming data to determine whether
 *                 to perform data reading
 */
static void CH347_WriteRead(struct scan_command *cmd, uint8_t *bits,
							int nb_bits, enum scan_type scan)
{
	int nb8 = nb_bits / 8;
	int nb1 = nb_bits % 8;
	int num_bits = 0;
	bool IsRead = false; /*, isLastByte = false */
	uint8_t TMS_Bit = 0, TDI_Bit = 0, CMD_Bit;
	static uint8_t byte0[SF_PACKET_BUF_SIZE];
	uint8_t *readData = calloc(SF_PACKET_BUF_SIZE, 1);
	unsigned long readLen = 0;
	unsigned long BI = 0, DI, DII, PktDataLen, DLen = 0, tempIndex,
		totalReadLength = 0, tempLength = 0;
	if (ch347.pack_size == LARGER_PACK) {
		if ((ch347.read_count >= (USBC_PACKET_USBHS_SINGLE * 1)))
			CH347_Flush_Buffer();
	} else {
		CH347_Flush_Buffer();
	}

	if (nb8 > 0 && nb1 == 0) {
		nb8--;
		nb1 = 8;
	}

	IsRead = (scan == SCAN_IN || scan == SCAN_IO);
	DI = BI = 0;
	while (DI < (unsigned long)nb8) {
		if ((nb8 - DI) > UCMDPKT_DATA_MAX_BYTES_USBHS)
			PktDataLen = UCMDPKT_DATA_MAX_BYTES_USBHS;
		else
			PktDataLen = nb8 - DI;

		DII = PktDataLen;

		if (IsRead)
			CH347_In_Buffer(CH347_CMD_JTAG_DATA_SHIFT_RD);
		else
			CH347_In_Buffer(CH347_CMD_JTAG_DATA_SHIFT);

		/* packet data don't deal D3 & D4 */
		if ((CH347_CMD_JTAG_DATA_SHIFT_RD != ch347.lastCmd) |
			(CH347_CMD_JTAG_DATA_SHIFT != ch347.lastCmd)) {
			/* update the ch347 struct */
			ch347.lastCmd = 0;
			ch347.len_idx = 0;
			ch347.len_value = 0;
		}

		CH347_In_Buffer((uint8_t)(PktDataLen >> 0) & 0xFF);
		CH347_In_Buffer((uint8_t)(PktDataLen >> 8) & 0xFF);

		if (bits)
			CH347_In_Buffer_bytes(&bits[DI], PktDataLen);
		else
			CH347_In_Buffer_bytes(byte0, PktDataLen);
		DI += DII;

		tempLength += (DII + CH347_CMD_HEADER);
	}

	totalReadLength += tempLength;

	if (IsRead) {
		ch347.read_count += tempLength;
		readLen += tempLength;
	}

	if (bits) {
		CMD_Bit = IsRead ? CH347_CMD_JTAG_BIT_OP_RD :	\
			CH347_CMD_JTAG_BIT_OP;
		DLen = (nb1 * 2) + 1;

		if (CMD_Bit != ch347.lastCmd) {
			CH347_In_Buffer(CMD_Bit);
			CH347_In_Buffer((uint8_t)(DLen >> 0) & 0xFF);
			CH347_In_Buffer((uint8_t)(DLen >> 8) & 0xFF);
			ch347.lastCmd = CMD_Bit;
			ch347.len_idx = ch347.buffer_idx - 2;
			ch347.len_value = DLen;
		} else {
			/* update the ch347 struct cmd data leng */
			ch347.len_value += DLen;
			/* update the cmd packet valid leng */
			ch347.buffer[ch347.len_idx] =
				(uint8_t)(ch347.len_value >> 0) & 0xFF;
			ch347.buffer[ch347.len_idx + 1] =
				(uint8_t)(ch347.len_value >> 8) & 0xFF;
		}

		TMS_Bit = TMS_L;
		for (int i = 0; i < nb1; i++) {
			if ((bits[nb8] >> i) & 0x01)
				TDI_Bit = TDI_H;
			else
				TDI_Bit = TDI_L;

			if ((i + 1) == nb1)
				TMS_Bit = TMS_H;

			CH347_In_Buffer(TMS_Bit | TDI_Bit | TCK_L | TRST_H);
			CH347_In_Buffer(TMS_Bit | TDI_Bit | TCK_H | TRST_H);
		}
		CH347_In_Buffer(TMS_Bit | TDI_Bit | TCK_L | TRST_H);
	}

	ch347.TMS = TMS_Bit;
	ch347.TDI = TDI_Bit;
	ch347.TCK = TCK_L;

	if (IsRead) {
		tempLength = ((DLen / 2) + CH347_CMD_HEADER);
		totalReadLength += tempLength;
		ch347.read_count += tempLength;
		readLen += tempLength;
		DI = BI = 0;
	}
	int offset = 0;
	if (IsRead && totalReadLength > 0) {
		if (ch347.pack_size == STANDARD_PACK && bits && cmd) {
			CH347_Flush_Buffer();
			CH347_Read_Scan(readData, readLen);
		}

		for (unsigned int i = 0; i < cmd->num_fields; i++) {
			/* if neither in_value nor in_handler
			 * are specified we don't have to examine this field
			 */
			LOG_DEBUG("fields[%i].in_value[%i], offset: %d",
				  i, cmd->fields[i].num_bits, offset);
			num_bits = cmd->fields[i].num_bits;
			if (cmd->fields[i].in_value) {
				if (ch347.pack_size == LARGER_PACK) {
					if (cmd->fields[i].in_value)
						bit_copy_queued(
							&ch347.read_queue,
							cmd->fields[i].in_value,
							0,
							&ch347.read_buffer[ch347.read_idx],
							offset, num_bits);

					if (num_bits > 7)
						ch347.read_idx +=
							DIV_ROUND_UP(offset, 8);
				} else {
					uint8_t *captured = buf_set_buf(
						readData, offset,
						malloc(DIV_ROUND_UP(num_bits, 8)),
						0,
						num_bits);

					if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
						char *char_buf =
							buf_to_hex_str(
								captured,
								(num_bits >
								 DEBUG_JTAG_IOZ)
								? DEBUG_JTAG_IOZ
								: num_bits);
						free(char_buf);
					}
					if (cmd->fields[i].in_value)
						buf_cpy(captured,
							cmd->fields[i].in_value,
							num_bits);
					free(captured);
				}
			}
			offset += num_bits;
		}
	}

	tempIndex = ch347.buffer_idx;
	for (unsigned int i = 0; i < CH347_CMD_HEADER; i++)
		CH347_In_Buffer(0);
	BI = CH347_CMD_HEADER;
	BI = CH347_IdleClock(BI);

	combinePackets(CH347_CMD_JTAG_BIT_OP, tempIndex, BI);
	if (readData) {
		free(readData);
		readData = NULL;
	}
}

static void CH347_RunTest(int cycles, tap_state_t state)
{
	LOG_DEBUG_IO("%s(cycles=%i, end_state=%d)", __func__, cycles, state);
	if (tap_get_state() != TAP_IDLE)
		CH347_MoveState(TAP_IDLE, 0);

	uint8_t tmsValue = 0;
	CH347_TmsChange(&tmsValue, 7, 1);

	CH347_WriteRead(NULL, NULL, cycles, SCAN_OUT);
	CH347_MoveState(state, 0);
}

static void CH347_TableClocks(int cycles)
{
	LOG_DEBUG_IO("%s(cycles=%i)", __func__, cycles);
	CH347_WriteRead(NULL, NULL, cycles, SCAN_OUT);
}

/**
 * CH347_Scan - Switch to SHIFT-DR or SHIFT-IR status for scanning
 * @param cmd Upper layer transfer command parameters
 *
 * @return Success returns ERROR_OK
 */
static int CH347_Scan(struct scan_command *cmd)
{
	int scan_bits;
	uint8_t *buf = NULL;
	enum scan_type type;
	int ret = ERROR_OK;
	static const char *const type2str[] = {
		"", "SCAN_IN", "SCAN_OUT", "SCAN_IO"
	};
	char *log_buf = NULL;

	type = jtag_scan_type(cmd);
	scan_bits = jtag_build_buffer(cmd, &buf);

	if (cmd->ir_scan)
		CH347_MoveState(TAP_IRSHIFT, 0);
	else
		CH347_MoveState(TAP_DRSHIFT, 0);

	log_buf = HexToString(buf, DIV_ROUND_UP(scan_bits, 8));
	LOG_DEBUG_IO("Scan");
	LOG_DEBUG_IO("%s(scan=%s, type=%s, bits=%d, buf=[%s], end_state=%d)",
				__func__,
				cmd->ir_scan ? "IRSCAN" : "DRSCAN",
				type2str[type],
				scan_bits, log_buf, cmd->end_state);

	free(log_buf);

	CH347_WriteRead(cmd, buf, scan_bits, type);

	free(buf);

	CH347_MoveState(cmd->end_state, 1);

	return ret;
}

static void CH347_Sleep(int us)
{
	LOG_DEBUG_IO("%s(us=%d)", __func__, us);
	jtag_sleep(us);
}

static int ch347_execute_queue(void)
{
	struct jtag_command *cmd;
	int ret = ERROR_OK;

	for (cmd = jtag_command_queue; ret == ERROR_OK && cmd;
		 cmd = cmd->next) {
		switch (cmd->type) {
		case JTAG_RESET:
			LOG_DEBUG_IO("JTAG_RESET : %d %d.\n",
						 cmd->cmd.reset->trst,
						 cmd->cmd.reset->srst);
			ch347_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			break;
		case JTAG_RUNTEST:
			CH347_RunTest(cmd->cmd.runtest->num_cycles,
						  cmd->cmd.runtest->end_state);
			break;
		case JTAG_STABLECLOCKS:
			CH347_TableClocks(cmd->cmd.stableclocks->num_cycles);
			break;
		case JTAG_TLR_RESET:
			CH347_MoveState(cmd->cmd.statemove->end_state, 0);
			break;
		case JTAG_PATHMOVE:
			CH347_MovePath(cmd->cmd.pathmove);
			break;
		case JTAG_TMS:
			CH347_TMS(cmd->cmd.tms);
			break;
		case JTAG_SLEEP:
			CH347_Sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_SCAN:
			ret = CH347_Scan(cmd->cmd.scan);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type 0x%X",
				  cmd->type);
			ret = ERROR_FAIL;
			break;
		}
	}

	CH347_Flush_Buffer();
	return ret;
}

/**
 * ch347_init - CH347 Initialization function
 *
 *  Todo:
 *                Initialize dynamic library functions
 *                Open Device
 *  @return Success returns 0, failure returns ERROR_FAIL
 */
static int ch347_init(void)
{
	DevIsOpened = CH347OpenDevice(ugIndex);
	ugIndex = DevIsOpened;
	if (DevIsOpened < 0) {
		LOG_ERROR("CH347 open error");
		return ERROR_FAIL;
	}

	if (!swd_mode) {
		/* ch347 init */
		ch347.TCK = 0;
		ch347.TMS = 0;
		ch347.TDI = 0;
		ch347.TRST = TRST_H;
		ch347.buffer_idx = 0;

		memset(&ch347.buffer, 0, SF_PACKET_BUF_SIZE);
		ch347.len_idx = 0;
		ch347.len_value = 0;
		ch347.lastCmd = 0;

		memset(&ch347.read_buffer, 0, SF_PACKET_BUF_SIZE);
		ch347.read_count = 0;
		ch347.read_idx = 0;

		bit_copy_queue_init(&ch347.read_queue);


		tap_set_state(TAP_RESET);
	} else { /* swd init */
		CH347SWD_INIT(ugIndex, 0);
	}
	return ERROR_OK;
}

/**
 * ch347_quit - CH347 Device Release Function
 *
 * Todo:
 *              Reset JTAG pin signal
 *              Close
 *  @return always returns 0
 */
static int ch347_quit(void)
{
	uint32_t retlen = 4;
	uint8_t byte[4] = {CH347_CMD_JTAG_BIT_OP, 0x01, 0x00, ch347.TRST};
	if (!swd_mode) {
		CH347_Write(byte, &retlen);
		bit_copy_discard(&ch347.read_queue);
	}
	if (DevIsOpened) {
		CH347CloseDevice(ugIndex);
		LOG_INFO("CH347 close");
		DevIsOpened = false;
	}
	return 0;
}

static bool Check_Speed(uint64_t iIndex, uint8_t iClockRate)
{
	uint32_t i = 0, j;
	bool retVal;
	uint8_t cmdBuf[32] = "";
	cmdBuf[i++] = CH347_CMD_JTAG_INIT;
	cmdBuf[i++] = 6;
	cmdBuf[i++] = 0;

	cmdBuf[i++] = 0;
	cmdBuf[i++] = iClockRate;

	for (j = 0; j < 4; j++)
		cmdBuf[i++] = ch347.TCK | ch347.TDI | ch347.TMS | ch347.TRST;

	uint32_t mLength = i;
	if (!CH347WriteData(iIndex, cmdBuf, &mLength) || (mLength != i))
		return false;

	mLength = 4;
	memset(cmdBuf, 0, sizeof(cmdBuf));

	if (!CH347ReadData(iIndex, cmdBuf, &mLength) || (mLength != 4))
		return false;

	retVal = ((cmdBuf[0] == CH347_CMD_JTAG_INIT) && \
		  (cmdBuf[CH347_CMD_HEADER] == 0));
	return retVal;
}

static bool CH347Jtag_INIT(uint64_t iIndex, uint8_t iClockRate)
{
	ch347.pack_size = (Check_Speed(iIndex, 0x09) == true) ? \
		STANDARD_PACK : LARGER_PACK;
	if (ch347.pack_size == STANDARD_PACK) {
		if (iClockRate - 2 < 0)
			return Check_Speed(iIndex, 0);
		else
			return Check_Speed(iIndex, iClockRate - 2);
	}

	return Check_Speed(iIndex, iClockRate);
}

/**
 * ch347_speed - CH347 TCK frequency setting
 *  @param speed Frequency size set
 *  @return Success returns ERROR_OK，failed returns FALSE
 */
static int ch347_speed(int speed)
{
	unsigned long i = 0;
	uint8_t clockRate;
	int retval = -1;
	int speed_clock[8] = {
		KHZ(468.75), KHZ(937.5), MHZ(1.875),
		MHZ(3.75), MHZ(7.5), MHZ(15), MHZ(30), MHZ(60)
	};

	if (!swd_mode) {
		for (i = 0; i < (sizeof(speed_clock) / sizeof(int)); i++) {
			if ((speed >= speed_clock[i]) &&
				(speed <= speed_clock[i + 1])) {
				clockRate = i + 1;
				retval = CH347Jtag_INIT(ugIndex, clockRate);
				if (!retval) {
					LOG_ERROR("CH347 set TCK speed error");
					return retval;
				} else {
					break;
				}
			} else if (speed < speed_clock[0]) {
				retval = CH347Jtag_INIT(ugIndex, 0);
				if (!retval) {
					LOG_ERROR("CH347 set TCK speed error");
					return retval;
				} else {
					break;
				}
			}
		}
	}
	return ERROR_OK;
}

static int ch347_speed_div(int speed, int *khz)
{
	*khz = speed / 1000;
	return ERROR_OK;
}

static int ch347_khz(int khz, int *jtag_speed)
{
	if (khz == 0) {
		LOG_ERROR("CH347 not support the adapter speed");
		return ERROR_FAIL;
	}
	*jtag_speed = khz * 1000;
	return ERROR_OK;
}

static int ch347_trst_out(unsigned char status)
{
	uint32_t BI = 0;
	unsigned char byte = 0;
	unsigned char cmdPacket[4] = "";
	cmdPacket[BI++] = CH347_CMD_JTAG_BIT_OP;
	cmdPacket[BI++] = 0x01;
	cmdPacket[BI++] = 0;
	byte = ch347.TCK | ch347.TDI | ch347.TMS | (ch347.TRST =
							(status ? TRST_H : TRST_L));
	cmdPacket[BI++] = byte;

	if (!CH347_Write(cmdPacket, &BI)) {
		LOG_ERROR("CH347 set TRST error");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(ch347_handle_vid_pid_command)
{
	if (CMD_ARGC > 2) {
		LOG_WARNING("ignoring extra IDs in ch347_vid_pid "
					"(maximum is 1 pair)");
		CMD_ARGC = 2;
	}
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], ch347_vid);
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], ch347_pid);
	} else
		LOG_WARNING("incomplete ch347_vid_pid configuration");

	return ERROR_OK;
}

COMMAND_HANDLER(ch347_trst)
{
	ch347_trst_out(TRST_L);
	jtag_sleep(atoi(CMD_ARGV[0]) * 1000);
	ch347_trst_out(TRST_H);
	return ERROR_OK;
}

static const struct command_registration ch347_subcommand_handlers[] = {
	{
		.name = "vid_pid",
		.handler = ch347_handle_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the VID and PID of the adapter",
		.usage = "vid pid",
	},
	{
		.name = "jtag_ntrst_delay",
		.handler = ch347_trst,
		.mode = COMMAND_ANY,
		.help = "set the trst of the CH347 device that is used as JTAG",
		.usage = "[milliseconds]",
	},

	COMMAND_REGISTRATION_DONE};

static const struct command_registration ch347_command_handlers[] = {
	{
		.name = "ch347",
		.mode = COMMAND_ANY,
		.help = "perform ch347 management",
		.chain = ch347_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE};

static int ch347_swd_init(void)
{
	PCH347_SWD_IO pswd_io;
	LOG_INFO("CH347 SWD mode enabled");
	swd_mode = true;
	memset(&ch347_swd_context, 0, sizeof(ch347_swd_context));

	INIT_LIST_HEAD(&ch347_swd_context.send_cmd_head);
	INIT_LIST_HEAD(&ch347_swd_context.free_cmd_head);

	ch347_swd_context.queued_retval = ERROR_OK;
	/* 0XE8 + 2byte len + N byte cmds */
	ch347_swd_context.send_len = CH347_CMD_HEADER;
	/* 0XE8 + 2byte len + N byte ack + data */
	ch347_swd_context.need_recv_len = CH347_CMD_HEADER;
	ch347_swd_context.ch347_cmd_buf = malloc(128 * sizeof(CH347_SWD_IO));
	if (ch347_swd_context.ch347_cmd_buf) {
		pswd_io = (PCH347_SWD_IO)ch347_swd_context.ch347_cmd_buf;
		for (int i = 0; i < 128; i++, pswd_io++) {
			INIT_LIST_HEAD(&pswd_io->list_entry);
			list_add_tail(&pswd_io->list_entry,
						  &ch347_swd_context.free_cmd_head);
		}
	}
	return ch347_swd_context.ch347_cmd_buf ? ERROR_OK : ERROR_FAIL;
}

static PCH347_SWD_IO ch347_get_one_swd_io(void)
{
	PCH347_SWD_IO pswd_io;
	if (list_empty(&ch347_swd_context.free_cmd_head)) {
		return NULL;
	} else {
		pswd_io = list_first_entry(&ch347_swd_context.free_cmd_head,
					   CH347_SWD_IO, list_entry);
		list_del_init(&pswd_io->list_entry);
		pswd_io->cmd = 0;
		pswd_io->usbcmd = CH347_CMD_SWD_SEQ_W;
		pswd_io->dst = NULL;
		return pswd_io;
	}
}

static void ch347_swd_queue_flush(void)
{
	uint32_t mLength = ch347_swd_context.send_len;
	ch347_swd_context.send_buf[0] = (uint8_t)CH347_CMD_SWD;
	ch347_swd_context.send_buf[1] = (uint8_t)(ch347_swd_context.send_len -
						  CH347_CMD_HEADER);
	ch347_swd_context.send_buf[2] = (uint8_t)((ch347_swd_context.send_len -
						   CH347_CMD_HEADER) >> 8);
	if (!CH347WriteData(ugIndex, ch347_swd_context.send_buf, &mLength) ||
		(mLength != ch347_swd_context.send_len)) {
		ch347_swd_context.queued_retval = ERROR_FAIL;
		LOG_DEBUG("CH347WriteData error ");
		return;
	}
	ch347_swd_context.recv_len = 0;
	do {
		mLength = CH347_MAX_RECV_BUF - ch347_swd_context.recv_len;
		if (!CH347ReadData(ugIndex,
				   &ch347_swd_context.recv_buf[ch347_swd_context.recv_len],
				   &mLength)) {
			ch347_swd_context.queued_retval = ERROR_FAIL;
			LOG_DEBUG("CH347ReadData error ");
			return;
		}
		ch347_swd_context.recv_len += mLength;
	} while (ch347_swd_context.recv_len < ch347_swd_context.need_recv_len);

	if (ch347_swd_context.need_recv_len > ch347_swd_context.recv_len) {
		LOG_ERROR("CH347 flush write/read failed %d %d %d",
			  __LINE__,
			  ch347_swd_context.recv_len,
			  ch347_swd_context.need_recv_len);
	}
}
static void ch347_wrtie_swd_reg(uint8_t cmd, const uint8_t *out, uint8_t parity)
{
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] =
		CH347_CMD_SWD_REG_W;
	/* 8bit + 32bit +1bit */
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x29;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x00;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = cmd;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out[0];
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out[1];
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out[2];
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out[3];
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = parity;
	/* 0xA0 +  1 byte(3bit ACK) */
	ch347_swd_context.need_recv_len += (1 + 1);
}

static void ch347_wrtie_spec_seq(const uint8_t *out, uint8_t out_len)
{
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] =
		CH347_CMD_SWD_SEQ_W;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out_len;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x00;
	for (uint8_t i = 0; i < DIV_ROUND_UP(out_len, 8); i++) {
		if (out) {
			ch347_swd_context.send_buf[ch347_swd_context.send_len++]
				= out[i];
		} else {
			ch347_swd_context.send_buf[ch347_swd_context.send_len++]
				= 0x00;
		}
	}
	ch347_swd_context.need_recv_len += 1; /* 0xA1 */
}

static void ch347_read_swd_reg(uint8_t cmd)
{
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] =
		CH347_CMD_SWD_REG_R;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x22;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x00;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = cmd;
	/* 0xA2 + 1 byte(3bit ACK) + 4 byte(data) +
	   1 byte(1bit parity+1bit trn) */
	ch347_swd_context.need_recv_len += 1 + 1 + 4 + 1;
}

static int ch347_swd_switch_out(enum swd_special_seq seq, const uint8_t *out,
				unsigned out_len)
{
	PCH347_SWD_IO pswd_io;
	if ((ch347_swd_context.send_len +
		 (1 + 2 + DIV_ROUND_UP(out_len, 8))) > CH347_MAX_SEND_BUF) {
		return ERROR_FAIL;
	}
	if ((ch347_swd_context.need_recv_len + 2) > CH347_MAX_RECV_BUF)
		return ERROR_FAIL;

	pswd_io = ch347_get_one_swd_io();
	if (pswd_io) {
		ch347_wrtie_spec_seq(out, out_len);
		list_add_tail(&pswd_io->list_entry,
					  &ch347_swd_context.send_cmd_head);
		return ERROR_OK;
	} else {
		return ERROR_FAIL;
	}
}

/* check read/write REG can fill in remaining buff */
static bool ch347_chk_buf_size(uint8_t cmd, uint32_t ap_delay_clk)
{
	bool bflush;
	uint32_t send_len, recv_len, len;
	bflush = false;
	send_len = ch347_swd_context.send_len;
	recv_len = ch347_swd_context.need_recv_len;
	do {

		if (cmd & SWD_CMD_RNW) {
			len = 1 + 1 + 1 + 1; /* 0xA2 + len + rev + cmd */
			if (send_len + len > CH347_MAX_SEND_BUF)
				break;
			send_len += len;
			len = 1 + 1 + 4 + 1;
			/* 0xA2 + 1byte(3bit ack) +  4byte(data) +
			   1byte(1bit parity+1bit trn) */
			if (recv_len + len > CH347_MAX_RECV_BUF)
				break;
			recv_len += len;
		} else { /* write reg */
			len = 1 + 1 + 1 + 1 + 4 + 1;
			/* 0xA0 + len + rev  + cmd +data + parity */
			if (send_len + len > CH347_MAX_SEND_BUF)
				break;
			send_len += len;
			len = 1 + 1; /* 0xA0 + 1byte(3bit ack) */
			if (recv_len + len > CH347_MAX_RECV_BUF)
				break;
			recv_len += len;
		}
		if (cmd & SWD_CMD_APNDP) {
			len = 1 + 1 + 1 + DIV_ROUND_UP(ap_delay_clk, 8);
			/* 0xA1 + Len  + rev  + n byte(delay) */
			if (send_len + len > CH347_MAX_SEND_BUF)
				break;
			len = 1; /* 0xA1 */
			if ((recv_len + len) > CH347_MAX_RECV_BUF)
				break;
		}
		/* swd packet requests */
		bflush = true;
	} while (false);

	return bflush;
}

static void ch347_swd_send_idle(uint32_t ap_delay_clk)
{
	PCH347_SWD_IO pswd_io;

	pswd_io = ch347_get_one_swd_io();
	if (!pswd_io) {
		ch347_swd_run_queue();
		pswd_io = ch347_get_one_swd_io();
		if (!pswd_io) {
			LOG_DEBUG("ch347_swd_queue_cmd error ");
			ch347_swd_context.queued_retval = ERROR_FAIL;
			return;
		}
	}
	ch347_wrtie_spec_seq(NULL, ap_delay_clk);

	list_add_tail(&pswd_io->list_entry, &ch347_swd_context.send_cmd_head);
}

static void ch347_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data,
				uint32_t ap_delay_clk)
{
	PCH347_SWD_IO pswd_io;
	if (ap_delay_clk > 255)
		printf("ch347_swd_queue_cmd ap_delay_clk = %d\r\n",
			   ap_delay_clk);

	if (ch347_swd_context.sent_cmd_count >= CH347_MAX_SEND_CMD)
		ch347_swd_run_queue();

	if (!ch347_chk_buf_size(cmd, ap_delay_clk))
		ch347_swd_run_queue();

	pswd_io = ch347_get_one_swd_io();
	if (!pswd_io) {
		ch347_swd_run_queue();
		pswd_io = ch347_get_one_swd_io();
		if (!pswd_io) {
			LOG_DEBUG("ch347_swd_queue_cmd error ");
			ch347_swd_context.queued_retval = ERROR_FAIL;
			return;
		}
	}

	pswd_io->cmd = cmd | SWD_CMD_START | SWD_CMD_PARK;

	if (pswd_io->cmd & SWD_CMD_RNW) {
		pswd_io->usbcmd = CH347_CMD_SWD_REG_R;
		pswd_io->dst = dst;
		ch347_read_swd_reg(pswd_io->cmd);
	} else {
		pswd_io->usbcmd = CH347_CMD_SWD_REG_W;
		pswd_io->value = data;
		ch347_wrtie_swd_reg(pswd_io->cmd, (uint8_t *)&data,
							parity_u32(data));
	}

	ch347_swd_context.sent_cmd_count++;
	list_add_tail(&pswd_io->list_entry, &ch347_swd_context.send_cmd_head);
	/* Insert idle cycles after AP accesses to avoid WAIT */
	if (cmd & SWD_CMD_APNDP) {
		if (ap_delay_clk == 0)
			printf("ap_delay_clk == 0");
		ch347_swd_send_idle(ap_delay_clk);
	}
}

static int ch347_swd_switch_seq(enum swd_special_seq seq)
{
	printf("ch347_swd_switch_seq %d \r\n", seq);
	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		return ch347_swd_switch_out(seq, swd_seq_line_reset,
									swd_seq_line_reset_len);
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		return ch347_swd_switch_out(seq, swd_seq_jtag_to_swd,
									swd_seq_jtag_to_swd_len);
	case JTAG_TO_DORMANT:
		LOG_DEBUG("JTAG-to-DORMANT");
		return ch347_swd_switch_out(seq, swd_seq_jtag_to_dormant,
									swd_seq_jtag_to_dormant_len);
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		return ch347_swd_switch_out(seq, swd_seq_swd_to_jtag,
									swd_seq_swd_to_jtag_len);
	case SWD_TO_DORMANT:
		LOG_DEBUG("SWD-to-DORMANT");
		return ch347_swd_switch_out(seq, swd_seq_swd_to_dormant,
									swd_seq_swd_to_dormant_len);
	case DORMANT_TO_SWD:
		LOG_DEBUG("DORMANT-to-SWD");
		return ch347_swd_switch_out(seq, swd_seq_dormant_to_swd,
									swd_seq_dormant_to_swd_len);
	case DORMANT_TO_JTAG:
		LOG_DEBUG("DORMANT-to-JTAG");
		return ch347_swd_switch_out(seq, swd_seq_dormant_to_jtag,
									swd_seq_dormant_to_jtag_len);
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}
}

static void ch347_swd_read_reg(uint8_t cmd, uint32_t *value,
							   uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RNW);
	ch347_swd_queue_cmd(cmd, value, 0, ap_delay_clk);
}

static void ch347_swd_write_reg(uint8_t cmd, uint32_t value,
								uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RNW));
	ch347_swd_queue_cmd(cmd, NULL, value, ap_delay_clk);
}

static int ch347_swd_run_queue(void)
{
	LOG_DEBUG_IO("Executing %u queued transactions",
				 ch347_swd_context.sent_cmd_count);
	int retval, parity;
	struct list_head *tmp, *pos;
	uint8_t *recv_buf;
	uint32_t recv_len, cmds_len, data;
	PCH347_SWD_IO pswd_io;
	if (ch347_swd_context.queued_retval != ERROR_OK) {
		LOG_DEBUG_IO("Skipping due to previous errors: %d",
					 ch347_swd_context.queued_retval);
		goto skip;
	}

	/* A transaction must be followed by another transaction or at least 8
	   idle cycles to ensure that data is clocked through the AP. */
	if ((ch347_swd_context.send_len + (1 + 2 + 1)) > CH347_MAX_SEND_BUF)
		goto skip_idle;

	if ((ch347_swd_context.need_recv_len + 1) > CH347_MAX_RECV_BUF)
		goto skip_idle;

	ch347_swd_send_idle(8);

skip_idle:

	ch347_swd_queue_flush();

	if (ch347_swd_context.queued_retval != ERROR_OK) {
		LOG_ERROR("CH347 usb write/read failed %d", __LINE__);
		goto skip;
	}
	recv_buf = ch347_swd_context.recv_buf;
	recv_len = 0;
	if (recv_buf[recv_len++] != CH347_CMD_SWD) { /* 0XE8 */
		ch347_swd_context.queued_retval = ERROR_FAIL;
		LOG_ERROR("CH347 usb write/read failed %d", __LINE__);
		goto skip;
	}

	cmds_len = BUILD_UINT16(recv_buf[recv_len], recv_buf[recv_len + 1]);
	recv_len += 2; /* cmds_len */
	if ((cmds_len + CH347_CMD_HEADER) > ch347_swd_context.recv_len) {
		ch347_swd_context.queued_retval = ERROR_FAIL;
		LOG_ERROR("CH347 usb write/read failed %d", __LINE__);
		goto skip;
	}

	list_for_each_safe(pos, tmp, &ch347_swd_context.send_cmd_head) {
		pswd_io = list_entry(pos, CH347_SWD_IO, list_entry);
		if (pswd_io->usbcmd == CH347_CMD_SWD_SEQ_W) {
			if (recv_buf[recv_len++] != CH347_CMD_SWD_SEQ_W) {
				ch347_swd_context.queued_retval = ERROR_FAIL;
				LOG_ERROR("CH347 usb write/read failed %d", __LINE__);
				goto skip;
			}
		} else { /* read/write Reg */
			int ack;
			bool check_ack;
			/* read  Reg */
			if (recv_buf[recv_len] == CH347_CMD_SWD_REG_R) {
				recv_len++;
				ack = buf_get_u32(&recv_buf[recv_len++], 0, 3);
				/* Devices do not reply to DP_TARGETSEL write
				   cmd, ignore received ack */
				check_ack = swd_cmd_returns_ack(pswd_io->cmd);
				if (ack != SWD_ACK_OK && check_ack) {
					ch347_swd_context.queued_retval =
						swd_ack_to_error_code(ack);
					LOG_ERROR("CH347 ack != SWD_ACK_OK %d", __LINE__);
					goto skip;
				}
				if (pswd_io->cmd & SWD_CMD_RNW) {
					data = buf_get_u32(&recv_buf[recv_len],
							   0, 32);
					parity = buf_get_u32(
						&recv_buf[recv_len], 32, 1);
					if (parity != parity_u32(data)) {
						LOG_ERROR("CH347 SWD read data parity mismatch %d", __LINE__);
						ch347_swd_context.queued_retval = ERROR_FAIL;
						goto skip;
					}

					LOG_DEBUG_IO("%s%s %s %s reg %X = %08X\n" PRIx32,
								check_ack ? "" : "ack ignored ",
								ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : \
								ack == SWD_ACK_FAULT  ? "FAULT" : "JUNK",
								pswd_io->cmd & SWD_CMD_APNDP ? "AP" : "DP",
								pswd_io->cmd & SWD_CMD_RNW ? "read" : "write",
								(pswd_io->cmd & SWD_CMD_A32) >> 1,
								data);

					if (pswd_io->dst)
						*pswd_io->dst = data;
				} else {
					ch347_swd_context.queued_retval =
						ERROR_FAIL;
					LOG_ERROR("CH347 usb write/read failed %d", __LINE__);
					goto skip;
				}
				recv_len += 5;
			} else if (recv_buf[recv_len] == CH347_CMD_SWD_REG_W) {
				recv_len++;
				ack = buf_get_u32(&recv_buf[recv_len++], 0, 3);
				/* Devices do not reply to DP_TARGETSEL write
				   cmd, ignore received ack */
				check_ack = swd_cmd_returns_ack(pswd_io->cmd);
				if (ack != SWD_ACK_OK && check_ack) {
					ch347_swd_context.queued_retval =
						swd_ack_to_error_code(ack);
					LOG_ERROR("CH347 SWD read data parity mismatch %d", __LINE__);
					goto skip;
				}
				LOG_DEBUG_IO("%s%s %s %s reg %X = %08X\n" PRIx32,
							check_ack ? "" : "ack ignored ",
							ack == SWD_ACK_OK ? "OK" : ack == SWD_ACK_WAIT ? "WAIT" : \
							ack == SWD_ACK_FAULT  ? "FAULT" : "JUNK",
							pswd_io->cmd & SWD_CMD_APNDP ? "AP" : "DP",
							pswd_io->cmd & SWD_CMD_RNW ? "read" : "write",
							(pswd_io->cmd & SWD_CMD_A32) >> 1,
							pswd_io->value);
			} else {
				ch347_swd_context.queued_retval = ERROR_FAIL;
				LOG_ERROR("CH347 usb write/read failed %d recv_len = %d",
					  __LINE__, recv_len);
				goto skip;
			}
		}
		list_del_init(&pswd_io->list_entry);
		list_add_tail(&pswd_io->list_entry,
					  &ch347_swd_context.free_cmd_head);
	}

skip:
	if (!list_empty(&ch347_swd_context.send_cmd_head)) {
		list_for_each_safe(pos, tmp, &ch347_swd_context.send_cmd_head)
		{
			pswd_io = list_entry(pos, CH347_SWD_IO, list_entry);
			list_del_init(&pswd_io->list_entry);
			list_add_tail(&pswd_io->list_entry,
						  &ch347_swd_context.free_cmd_head);
		}
	}
	/* 0xE8 + 2byte len */
	ch347_swd_context.send_len = CH347_CMD_HEADER;
	/* 0xE8 + 2byte len */
	ch347_swd_context.need_recv_len = CH347_CMD_HEADER;
	ch347_swd_context.recv_len = 0;
	ch347_swd_context.sent_cmd_count = 0;
	retval = ch347_swd_context.queued_retval;
	ch347_swd_context.queued_retval = ERROR_OK;

	return retval;
}

static const struct swd_driver ch347_swd = {
	.init = ch347_swd_init,
	.switch_seq = ch347_swd_switch_seq,
	.read_reg = ch347_swd_read_reg,
	.write_reg = ch347_swd_write_reg,
	.run = ch347_swd_run_queue,
};

static const char *const ch347_transports[] = {"jtag", "swd", NULL};

static struct jtag_interface ch347_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = ch347_execute_queue,
};

struct adapter_driver ch347_adapter_driver = {
	.name = "ch347",
	.transports = ch347_transports,
	.commands = ch347_command_handlers,

	.init = ch347_init,
	.quit = ch347_quit,
	.reset = ch347_reset,
	.speed = ch347_speed,
	.khz = ch347_khz,
	.speed_div = ch347_speed_div,

	.jtag_ops = &ch347_interface,
	.swd_ops = &ch347_swd,
};
