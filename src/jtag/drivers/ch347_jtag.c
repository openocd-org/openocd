/***************************************************************************                                                                     *
 *  gpio - trst
 * Driver for CH347-JTAG interface V1.0                                  *
 *                                                                         *
 *   Copyright (C) 2022 Nanjing Qinheng Microelectronics Co., Ltd.         *
 *   Web: http://wch.cn                                                    *
 *   Author: WCH@TECH53 <tech@wch.cn>                                      *
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
 *           |             |                                               *
 *      USB__|    CH347T   |____JTAG (TDO,TDI,TMS,TCK)                     *
 *           |_____________|                                               *
 *            __|__________                                                *
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
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <helper/time_support.h>
#include <helper/replacements.h>

/* system includes */
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define JTAGIO_STA_OUT_TDI (0x10)
#define JTAGIO_STA_OUT_TMS (0x02)
#define JTAGIO_STA_OUT_TCK (0x01)

#define TDI_H JTAGIO_STA_OUT_TDI
#define TDI_L 0
#define TMS_H JTAGIO_STA_OUT_TMS
#define TMS_L 0
#define TCK_H JTAGIO_STA_OUT_TCK
#define TCK_L 0

#define KHZ(n) ((n)*UINT64_C(1000))
#define MHZ(n) ((n)*UINT64_C(1000000))
#define GHZ(n) ((n)*UINT64_C(1000000000))

#define HW_TDO_BUF_SIZE 4096
#define SF_PACKET_BUF_SIZE 51200         // Command packet length
#define UCMDPKT_DATA_MAX_BYTES_USBHS 507 // The data length contained in each command packet during USB high-speed operation
#define USBC_PACKET_USBHS 512            // Maximum data length per packet at USB high speed

#define CH347_CMD_HEADER 3 // Protocol header length

// Protocol transmission format: CMD (1 byte)+Length (2 bytes)+Data
#define CH347_CMD_INFO_RD 0xCA            // Parameter acquisition, used to obtain firmware version, JTAG interface related parameters, etc
#define CH347_CMD_JTAG_INIT 0xD0          // JTAG Interface Initialization Command
#define CH347_CMD_JTAG_BIT_OP 0xD1        // JTAG interface pin bit control command
#define CH347_CMD_JTAG_BIT_OP_RD 0xD2     // JTAG interface pin bit control and read commands
#define CH347_CMD_JTAG_DATA_SHIFT 0xD3    // JTAG interface data shift command
#define CH347_CMD_JTAG_DATA_SHIFT_RD 0xD4 // JTAG interface data shift and read command

#pragma pack(1)

typedef struct _CH347_info  // Record the CH347 pin status
{
    int TMS;
    int TDI;
    int TCK;
} _CH347_Info;

int DevIsOpened;            // Whether the device is turned on
bool UsbHighDev = true;
unsigned long USBC_PACKET;

_CH347_Info ch347 = {0, 0, 0}; // Initialize device structure status

#pragma pack()

#ifdef _WIN32
#include <windows.h>
typedef int(__stdcall *pCH347OpenDevice)(unsigned long iIndex);
typedef int(__stdcall *pCH347CloseDevice)(unsigned long iIndex);
typedef unsigned long(__stdcall *pCH347SetTimeout)(unsigned long iIndex,        // Specify equipment serial number
                                                   unsigned long iWriteTimeout, // Specifies the timeout period for USB write out data blocks, in milliseconds mS, and 0xFFFFFFFF specifies no timeout (default)
                                                   unsigned long iReadTimeout); // Specifies the timeout period for USB reading data blocks, in milliseconds mS, and 0xFFFFFFFF specifies no timeout (default)
typedef unsigned long(__stdcall *pCH347WriteData)(unsigned long iIndex,         // Specify equipment serial number
                                                  void *oBuffer,                // Point to a buffer large enough to hold the descriptor
                                                  unsigned long *ioLength);     // Pointing to the length unit, the input is the length to be read, and the return is the actual read length
typedef unsigned long(__stdcall *pCH347ReadData)(unsigned long iIndex,          // Specify equipment serial number
                                                 void *oBuffer,                 // Point to a buffer large enough to hold the descriptor
                                                 unsigned long *ioLength);      // Pointing to the length unit, the input is the length to be read, and the return is the actual read length
typedef unsigned long(__stdcall *pCH347Jtag_INIT)(unsigned long iIndex,         // Specify equipment serial number
                                                  unsigned char iClockRate);    // Pointing to the length unit, the input is the length to be read, and the return is the actual read length
HMODULE uhModule;
BOOL ugOpen;
unsigned long ugIndex;
pCH347OpenDevice CH347OpenDevice;
pCH347CloseDevice CH347CloseDevice;
pCH347SetTimeout CH347SetTimeout;
pCH347ReadData CH347ReadData;
pCH347WriteData CH347WriteData;
pCH347Jtag_INIT CH347Jtag_INIT;
#elif defined(__linux__)

#include <jtag/drivers/libusb_helper.h>

#define CH347_EPOUT 0x06u
#define CH347_EPIN  0x86u

bool ugOpen;
unsigned long ugIndex = 0;
struct libusb_device_handle *ch347_handle;

static const uint16_t ch347_vids[] = {0x1a86, 0};
static const uint16_t ch347_pids[] = {0x55dd, 0};

static uint32_t CH347OpenDevice(uint64_t iIndex)
{
    if (jtag_libusb_open(ch347_vids, ch347_pids, NULL, &ch347_handle, NULL) != ERROR_OK) {
        return false;
    } else {
        return true;
    }
}

static bool CH347WriteData(uint64_t iIndex, uint8_t *data, uint64_t *length)
{
	int ret, tmp = 0;
	ret = jtag_libusb_bulk_write(ch347_handle,
						CH347_EPOUT,
						(char *)data,
						*length,
						100, &tmp);
	*length = tmp;

	if (!ret) {
		return true;
	} else {
		return false;
	}
}

static bool CH347ReadData(uint64_t iIndex, uint8_t *data, uint64_t *length)
{
    int ret, tmp = 0;

    int size = *length;
    ret = jtag_libusb_bulk_read(ch347_handle,
                                CH347_EPIN,
                                (char *)data,
                                size,
                                100, &tmp);

    *length = tmp;
   if (!ret) {
		return true;
   } else {
		return false;
   }
}

static bool CH347Jtag_INIT(uint64_t iIndex, uint8_t iClockRate)
{
    if (iClockRate > 5)
        return false;

    uint64_t i = 0;
    bool retVal;
    uint8_t cmdBuf[128] = "";
    cmdBuf[i++] = CH347_CMD_JTAG_INIT;
    cmdBuf[i++] = 6; // Data length is 6
    cmdBuf[i++] = 0;

    cmdBuf[i++] = 0;          // Reserved Bytes
    cmdBuf[i++] = iClockRate; // JTAG clock speed
    i += 4;                   // Reserved Bytes

    uint64_t mLength = i;
    if (!CH347WriteData(iIndex, cmdBuf, &mLength) || (mLength != i)) {
        return false;
    }

    mLength = 4;
    memset(cmdBuf, 0, sizeof(cmdBuf));

    if (!CH347ReadData(iIndex, cmdBuf, &mLength) || (mLength != 4)) {
        return false;
    }

    retVal = ((cmdBuf[0] == CH347_CMD_JTAG_INIT) && (cmdBuf[CH347_CMD_HEADER] == 0));
    return retVal;
}

static bool CH347CloseDevice(uint64_t iIndex)
{
    jtag_libusb_close(ch347_handle);
    return true;
}

#endif

/**
 *  HexToString - Hex Conversion String Function
 *  @param buf    Point to a buffer to place the Hex data to be converted
 *  @param size   Pointing to the length unit where data needs to be converted
 *
 *  @return 	  Returns a converted string
 */
static char *HexToString(uint8_t *buf, uint32_t size)
{
    uint32_t i;
    char *str = calloc(size * 2 + 1, 1);

    for (i = 0; i < size; i++)
        sprintf(str + 2 * i, "%02x ", buf[i]);
    return str;
}

/**
 *  CH347_Write - CH347 Write
 *  @param oBuffer    Point to a buffer to place the data to be written out
 *  @param ioLength   Pointing to the length unit, the input is the length to be written out, and the return is the actual written length
 *
 *  @return 		  Write success returns 1, failure returns 0
 */
static int CH347_Write(void *oBuffer, unsigned long *ioLength)
{
    int ret = -1;
    unsigned long wlength = *ioLength, WI;

    if (*ioLength >= UCMDPKT_DATA_MAX_BYTES_USBHS)
        wlength = UCMDPKT_DATA_MAX_BYTES_USBHS;
    WI = 0;
    while (1)
    {
        ret = CH347WriteData(ugIndex, (uint8_t *)oBuffer + WI, &wlength);
        if (!ret) {
            *ioLength = 0;
            return false;
        }
        LOG_DEBUG_IO("(size=%lu, buf=[%s]) -> %" PRIu32, wlength, HexToString((uint8_t *)oBuffer, wlength), (uint32_t)wlength);
        WI += wlength;
        if (WI >= *ioLength)
            break;
        if ((*ioLength - WI) > UCMDPKT_DATA_MAX_BYTES_USBHS)
            wlength = UCMDPKT_DATA_MAX_BYTES_USBHS;
        else
            wlength = *ioLength - WI;
    }

    *ioLength = WI;
    return true;
}

/**
 * CH347_Read - CH347 Read
 * @param oBuffer  	Point to a buffer to place the data to be read in
 * @param ioLength 	Pointing to the length unit, the input is the length to be read, and the return is the actual read length
 *
 * @return 			Write success returns 1, failure returns 0
 */
static int CH347_Read(void *oBuffer, unsigned long *ioLength)
{
    unsigned long rlength = *ioLength, WI;
    // The maximum allowable reading for a single read is 4096B of data. If it exceeds the allowable reading limit, it will be calculated as 4096B
    if (rlength > UCMDPKT_DATA_MAX_BYTES_USBHS)
        rlength = UCMDPKT_DATA_MAX_BYTES_USBHS;
    WI = 0;

    while (1){
        if (!CH347ReadData(ugIndex, (uint8_t *)oBuffer + WI, &rlength))
        {
            LOG_ERROR("CH347_Read read data failure.");
            return false;
        }

        WI += rlength;
        if (WI >= *ioLength)
            break;
        if ((*ioLength - WI) > UCMDPKT_DATA_MAX_BYTES_USBHS)
            rlength = UCMDPKT_DATA_MAX_BYTES_USBHS;
        else
            rlength = *ioLength - WI;
    }

    LOG_DEBUG_IO("(size=%lu, buf=[%s]) -> %" PRIu32, WI, HexToString((uint8_t *)oBuffer, WI), (uint32_t)WI);
    *ioLength = WI;
    return true;
}

/**
 * CH347_ClockTms - Function function used to change the TMS value at the rising edge of TCK to switch its Tap state
 * @param BitBangPkt 	Protocol package
 * @param tms 		 	TMS value to be changed
 * @param BI		 	Protocol packet length
 *
 * @return 			 	Return protocol packet length
 */
static unsigned long CH347_ClockTms(unsigned char *BitBangPkt, int tms, unsigned long BI)
{
    unsigned char cmd = 0;

    if (tms == 1)
        cmd = TMS_H;
    else
        cmd = TMS_L;

    BitBangPkt[BI++] = cmd | TDI_L | TCK_L;
    BitBangPkt[BI++] = cmd | TDI_L | TCK_H;

    ch347.TMS = cmd;
    ch347.TDI = TDI_L;
    ch347.TCK = TCK_H;

    return BI;
}

/**
 * CH347_IdleClock - Function function to ensure that the clock is in a low state
 * @param BitBangPkt 	 Protocol package
 * @param BI  		 	 Protocol packet length
 *
 * @return 			 	 Return protocol packet length
 */
static unsigned long CH347_IdleClock(unsigned char *BitBangPkt, unsigned long BI)
{
    unsigned char byte = 0;
    byte |= ch347.TMS ? TMS_H : TMS_L;
    byte |= ch347.TDI ? TDI_H : TDI_L;
    BitBangPkt[BI++] = byte;

    return BI;
}

/**
 * CH347_TmsChange - Function function that performs state switching by changing the value of TMS
 * @param tmsValue 		 The TMS values that need to be switched form one byte of data in the switching order
 * @param step 	   		 The number of bit values that need to be read from the tmsValue value
 * @param skip 	   		 Count from the skip bit of tmsValue to step
 *
 */
static void CH347_TmsChange(const unsigned char *tmsValue, int step, int skip)
{
    int i;
    unsigned long BI, retlen, TxLen;
    unsigned char BitBangPkt[4096] = "";

    BI = CH347_CMD_HEADER;
    retlen = CH347_CMD_HEADER;
    LOG_DEBUG_IO("(TMS Value: %02x..., step = %d, skip = %d)", tmsValue[0], step, skip);

    for (i = skip; i < step; i++)
    {
        retlen = CH347_ClockTms(BitBangPkt, (tmsValue[i / 8] >> (i % 8)) & 0x01, BI);
        BI = retlen;
    }
    BI = CH347_IdleClock(BitBangPkt, BI);
    BitBangPkt[0] = CH347_CMD_JTAG_BIT_OP;
    BitBangPkt[1] = (unsigned char)BI - CH347_CMD_HEADER;
    BitBangPkt[2] = 0;

    TxLen = BI;

    if (!CH347_Write(BitBangPkt, &TxLen) && (TxLen != BI))
    {
        LOG_ERROR("JTAG Write send usb data failure.");
        return;
    }
}

/**
 * CH347_TMS - By ch347_ execute_ Queue call
 * @param cmd 	   Upper layer transfer command parameters
 *
 */
static void CH347_TMS(struct tms_command *cmd)
{
    LOG_DEBUG_IO("(step: %d)", cmd->num_bits);
    CH347_TmsChange(cmd->bits, cmd->num_bits, 0);
}

/**
 * CH347_Reset - CH347 Reset Tap Status Function
 * @brief 	If there are more than six consecutive TCKs and TMS is high, the state machine can be set to a Test-Logic-Reset state
 *
 */
static int CH347_Reset(void)
{
    unsigned char BitBang[512] = "", BI, i;
    unsigned long TxLen;

    BI = CH347_CMD_HEADER;
    for (i = 0; i < 7; i++)
    {
        BitBang[BI++] = TMS_H | TDI_L | TCK_L;
        BitBang[BI++] = TMS_H | TDI_L | TCK_H;
    }
    BitBang[BI++] = TMS_H | TDI_L | TCK_L;

    ch347.TCK = TCK_L;
    ch347.TDI = TDI_L;
    ch347.TMS = 0;

    BitBang[0] = CH347_CMD_JTAG_BIT_OP;
    BitBang[1] = BI - CH347_CMD_HEADER;
    BitBang[2] = 0;

    TxLen = BI;

    if (!CH347_Write(BitBang, &TxLen) && (TxLen != BI))
    {
        LOG_ERROR("JTAG_Init send usb data failure.");
        return false;
    }
    return true;
}

/**
 * CH347_MovePath - Obtain the current Tap status and switch to the status TMS value passed down by cmd
 * @param cmd Upper layer transfer command parameters
 *
 */
static void CH347_MovePath(struct pathmove_command *cmd)
{
    int i;
    unsigned long BI, retlen = 0, TxLen;
    unsigned char BitBangPkt[4096] = "";

    BI = CH347_CMD_HEADER;

    LOG_DEBUG_IO("(num_states=%d, last_state=%d)",
                 cmd->num_states, cmd->path[cmd->num_states - 1]);

    for (i = 0; i < cmd->num_states; i++)
    {
        if (tap_state_transition(tap_get_state(), false) == cmd->path[i])
            retlen = CH347_ClockTms(BitBangPkt, 0, BI);
        BI = retlen;
        if (tap_state_transition(tap_get_state(), true) == cmd->path[i])
            retlen = CH347_ClockTms(BitBangPkt, 1, BI);
        BI = retlen;
        tap_set_state(cmd->path[i]);
    }

    BI = CH347_IdleClock(BitBangPkt, BI);
    BitBangPkt[0] = CH347_CMD_JTAG_BIT_OP;
    BitBangPkt[1] = (unsigned char)BI - CH347_CMD_HEADER;
    BitBangPkt[2] = 0;

    TxLen = BI;
    if (!CH347_Write(BitBangPkt, &TxLen) && (TxLen != BI))
    {
        LOG_ERROR("JTAG Write send usb data failure.");
        return;
    }
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
 * @param bits 			 Read and write data this time
 * @param nb_bits 		 Incoming data length
 * @param scan			 The transmission method of incoming data to determine whether to perform data reading
 *
 */
static void CH347_WriteRead(uint8_t *bits, int nb_bits, enum scan_type scan)
{
    // uint32_t delay = 1000000;
    int nb8 = nb_bits / 8;
    int nb1 = nb_bits % 8;
    int i;
    bool IsRead = false;
    uint8_t TMS_Bit, TDI_Bit = 0;
    uint8_t *tdos = calloc(1, nb_bits / 8 + 32);
    static uint8_t BitBangPkt[SF_PACKET_BUF_SIZE];
    static uint8_t byte0[SF_PACKET_BUF_SIZE];
    unsigned char temp[512] = "";
    unsigned char temp_a[512] = "";
    unsigned long BI = 0, TxLen, RxLen, DI, DII, PktDataLen, DLen;
    int ret = ERROR_OK;

    // The last TDI bit will be output in a bit band mode, with nb1 guaranteed not to be 0, enabling it to output the last 1 bit of data when TMS changes
    if (nb8 > 0 && nb1 == 0)
    {
        nb8--;
        nb1 = 8;
    }
    IsRead = (scan == SCAN_IN || scan == SCAN_IO);
    DI = BI = 0;
    while (DI < (unsigned long)nb8)
    {
        // Build Package
        if ((nb8 - DI) > UCMDPKT_DATA_MAX_BYTES_USBHS)
            PktDataLen = UCMDPKT_DATA_MAX_BYTES_USBHS;
        else
            PktDataLen = nb8 - DI;

        DII = PktDataLen;

        if (IsRead)
            BitBangPkt[BI++] = CH347_CMD_JTAG_DATA_SHIFT_RD;
        else
            BitBangPkt[BI++] = CH347_CMD_JTAG_DATA_SHIFT;

        BitBangPkt[BI++] = (uint8_t)(PktDataLen >> 0) & 0xFF;
        BitBangPkt[BI++] = (uint8_t)(PktDataLen >> 8) & 0xFF;

        if (bits)
            memcpy(&BitBangPkt[BI], &bits[DI], PktDataLen);
        else
            memcpy(&BitBangPkt[BI], byte0, PktDataLen);
        BI += PktDataLen;

        // If it is necessary to read back data, judge the current BI value and issue the command
        if (IsRead)
        {
            TxLen = BI;

            if (!CH347_Write(BitBangPkt, &TxLen) && (TxLen != BI))
            {
                LOG_ERROR("CH347_WriteRead write usb data failure.");
                return;
            }
            BI = 0;

            ret = ERROR_OK;
            while (ret == ERROR_OK && PktDataLen > 0)
            {
                RxLen = PktDataLen + CH347_CMD_HEADER;
                if (!(ret = CH347_Read(temp, &RxLen)))
                {
                    LOG_ERROR("CH347_WriteRead read usb data failure.\n");
                    return;
                }

                if (RxLen != TxLen)
                {
                    if (!(ret = CH347_Read(temp_a, &TxLen)))
                    {
                        LOG_ERROR("CH347_WriteRead read usb data failure.\n");
                        return;
                    }
                    memcpy(&temp[RxLen], temp_a, TxLen);
                    RxLen += TxLen;
                }

                if (RxLen != 0)
                    memcpy(&tdos[DI], &temp[CH347_CMD_HEADER], (RxLen - CH347_CMD_HEADER));
                PktDataLen -= RxLen;
            }
        }

        DI += DII;

        // During transmission, if there is no read back, the command will be issued when the command packet length is about to reach saturation
        if (((SF_PACKET_BUF_SIZE - BI) < USBC_PACKET || (SF_PACKET_BUF_SIZE - BI) == USBC_PACKET))
        {
            TxLen = BI;

            if (!CH347_Write(BitBangPkt, &TxLen) && (TxLen != BI))
            {
                LOG_ERROR("CH347_WriteRead send usb data failure.");
                return;
            }
            BI = 0;
        }
    }

    // Clear the remaining commands in the while loop
    if (BI > 0)
    {
        TxLen = BI;
        if (!CH347_Write(BitBangPkt, &TxLen) && (TxLen != BI))
        {
            LOG_ERROR("CH347_WriteRead send usb data failure.");
            return;
        }
        BI = 0;
    }

    // Build a command package that outputs the last 1 bits of TDI data
    if (bits)
    {
        BitBangPkt[BI++] = IsRead ? CH347_CMD_JTAG_BIT_OP_RD : CH347_CMD_JTAG_BIT_OP;
        DLen = (nb1 * 2) + 1;
        // DLen = (nb1 * 2);
        BitBangPkt[BI++] = (uint8_t)(DLen >> 0) & 0xFF;
        BitBangPkt[BI++] = (uint8_t)(DLen >> 8) & 0xFF;
        TMS_Bit = TMS_L;

        for (i = 0; i < nb1; i++)
        {
            if ((bits[nb8] >> i) & 0x01)
                TDI_Bit = TDI_H;
            else
                TDI_Bit = TDI_L;

            if ((i + 1) == nb1) // The last bit is output in Exit1-DR state
                TMS_Bit = TMS_H;
            BitBangPkt[BI++] = TMS_Bit | TDI_Bit | TCK_L;
            BitBangPkt[BI++] = TMS_Bit | TDI_Bit | TCK_H;
        }
        BitBangPkt[BI++] = TMS_Bit | TDI_Bit | TCK_L;
        ch347.TCK = TCK_L;
        ch347.TDI = TDI_Bit;
        ch347.TMS = TMS_Bit;
    }

    // Read the last byte of data in Bit-Bang mode
    if (nb1 && IsRead)
    {
        TxLen = BI;

        if (!CH347_Write(BitBangPkt, &TxLen) && (TxLen != BI))
        {
            LOG_ERROR("CH347_WriteRead send usb data failure.");
            return;
        }
        BI = 0;

        RxLen = (TxLen + CH347_CMD_HEADER)/2;
        if (!(ret = CH347_Read(temp, &RxLen)))
        {
            LOG_ERROR("CH347_WriteRead read usb data failure.");
            return;
        }

        for (i = 0; ret == true && i < nb1; i++)
        {
            if (temp[CH347_CMD_HEADER + i] & 1)
                tdos[nb8] |= (1 << i);
            else
                tdos[nb8] &= ~(1 << i);
        }
    }

    // Clear unprocessed commands in this batch read/write function
    if (BI > 0)
    {
        TxLen = BI;
        if (!CH347_Write(BitBangPkt, &TxLen) && (TxLen != BI))
        {
            LOG_ERROR("CH347_WriteRead send usb data failure.");
            return;
        }
        BI = 0;
    }

    if (bits)
    {
        memcpy(bits, tdos, DIV_ROUND_UP(nb_bits, 8));
    }

    free(tdos);
    LOG_DEBUG_IO("bits %d str value: [%s].\n", DIV_ROUND_UP(nb_bits, 8), HexToString(bits, DIV_ROUND_UP(nb_bits, 8)));

    // Pull down TCK and TDI to a low level, as TDI sampling occurs at the rising edge of TCK. If the state does not change, TDI sampling may occur at the falling edge of TCK
    BI = CH347_CMD_HEADER;
    BI = CH347_IdleClock(BitBangPkt, BI);

    BitBangPkt[0] = CH347_CMD_JTAG_BIT_OP;
    BitBangPkt[1] = (unsigned char)BI - CH347_CMD_HEADER;
    BitBangPkt[2] = 0;

    TxLen = BI;

    if (!CH347_Write(BitBangPkt, &TxLen) && (TxLen != BI))
    {
        LOG_ERROR("JTAG Write send usb data failure.");
        return;
    }
}

static void CH347_RunTest(int cycles, tap_state_t state)
{
    LOG_DEBUG_IO("%s(cycles=%i, end_state=%d)", __func__, cycles, state);
    CH347_MoveState(TAP_IDLE, 0);

    CH347_WriteRead(NULL, cycles, SCAN_OUT);
    CH347_MoveState(state, 0);
}

static void CH347_TableClocks(int cycles)
{
    LOG_DEBUG_IO("%s(cycles=%i)", __func__, cycles);
    CH347_WriteRead(NULL, cycles, SCAN_OUT);
}

/**
 * CH347_Scan - Switch to SHIFT-DR or SHIFT-IR status for scanning
 * @param cmd 	    Upper layer transfer command parameters
 *
 * @return 	        Success returns ERROR_OK
 */
static int CH347_Scan(struct scan_command *cmd)
{
    int scan_bits;
    uint8_t *buf = NULL;
    enum scan_type type;
    int ret = ERROR_OK;
    static const char *const type2str[] = {"", "SCAN_IN", "SCAN_OUT", "SCAN_IO"};
    char *log_buf = NULL;

    type = jtag_scan_type(cmd);
    scan_bits = jtag_build_buffer(cmd, &buf);

    if (cmd->ir_scan)
        CH347_MoveState(TAP_IRSHIFT, 0);
    else
        CH347_MoveState(TAP_DRSHIFT, 0);

    log_buf = HexToString(buf, DIV_ROUND_UP(scan_bits, 8));
    LOG_DEBUG_IO("Scan");
    LOG_DEBUG_IO("%s(scan=%s, type=%s, bits=%d, buf=[%s], end_state=%d)", __func__,
                 cmd->ir_scan ? "IRSCAN" : "DRSCAN",
                 type2str[type],
                 scan_bits, log_buf, cmd->end_state);

    free(log_buf);

    CH347_WriteRead(buf, scan_bits, type);

    ret = jtag_read_buffer(buf, cmd);
    free(buf);

    CH347_MoveState(cmd->end_state, 1);

    return ret;
}

static void CH347_Sleep(int us)
{
    LOG_DEBUG_IO("%s(us=%d)", __func__, us);
    jtag_sleep(us);
}

static int ch347_execute_queue(struct jtag_command *cmd_queue)
{
    struct jtag_command *cmd = cmd_queue;
    static int first_call = 1;
    int ret = ERROR_OK;

    if (first_call)
    {
        first_call--;
        CH347_Reset();
    }

    while (cmd) {
        switch (cmd->type)
        {
        case JTAG_RESET:
            CH347_Reset();
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
        cmd = cmd->next;
    }
    return ret;
}

/**
 * ch347_init - CH347 Initialization function
 *
 *  Todo:
 *                Initialize dynamic library functions
 *                Open Device
 *  @return 	  Success returns 0, failure returns ERROR_FAIL
 */
static int ch347_init(void)
{
#ifdef _WIN32
    if (uhModule == 0)
    {
        uhModule = LoadLibrary("CH347DLL.DLL");
        if (uhModule)
        {
            CH347OpenDevice = (pCH347OpenDevice)GetProcAddress(uhModule, "CH347OpenDevice");
            CH347CloseDevice = (pCH347CloseDevice)GetProcAddress(uhModule, "CH347CloseDevice");
            CH347ReadData = (pCH347ReadData)GetProcAddress(uhModule, "CH347ReadData");
            CH347WriteData = (pCH347WriteData)GetProcAddress(uhModule, "CH347WriteData");
            CH347SetTimeout = (pCH347SetTimeout)GetProcAddress(uhModule, "CH347SetTimeout");
            CH347Jtag_INIT = (pCH347Jtag_INIT)GetProcAddress(uhModule, "CH347Jtag_INIT");
            if (CH347OpenDevice == NULL || CH347CloseDevice == NULL || CH347SetTimeout == NULL || CH347ReadData == NULL || CH347WriteData == NULL || CH347Jtag_INIT == NULL)
            {
                LOG_ERROR("Jtag_init error ");
                return ERROR_FAIL;
            }
        }
    }
    DevIsOpened = CH347OpenDevice(ugIndex);
#elif defined(__linux__)

    DevIsOpened = CH347OpenDevice(ugIndex);
    ugIndex = DevIsOpened;
#endif
    if (!DevIsOpened)
    {
        LOG_ERROR("CH347 Open Error.");
        return ERROR_FAIL;
    }

    USBC_PACKET = USBC_PACKET_USBHS; // The default is USB 2.0 high-speed, with a single transfer USB packet size of 512 bytes

    tap_set_state(TAP_RESET);
    return 0;
}

/**
 * ch347_quit - CH347 Device Release Function
 *
 * Todo:
 *              Reset JTAG pin signal
 *              Close
 *  @return 	always returns 0
 */
static int ch347_quit(void)
{
    // Set all signal lines to low level before exiting
    unsigned long retlen = 4;
    unsigned char byte[4] = {CH347_CMD_JTAG_BIT_OP, 0x01, 0x00, 0x00};

    CH347_Write(byte, &retlen);

    if (DevIsOpened)
    {
        CH347CloseDevice(ugIndex);
        LOG_INFO("Close the CH347.");
        DevIsOpened = false;
    }
    return 0;
}

/**
 * ch347_speed - CH347 TCK frequency setting
 *  @param speed Frequency size set
 *  @return 	 Success returns ERROR_OK，failed returns FALSE
 */
static int ch347_speed(int speed)
{
    unsigned long i = 0;
    int retval = -1;
    int speed_clock[6] = {MHZ(1.875), MHZ(3.75), MHZ(7.5), MHZ(15), MHZ(30), MHZ(60)};

    for (i = 0; i < (sizeof(speed_clock) / sizeof(int)); i++)
    {
        if ((speed >= speed_clock[i]) && (speed <= speed_clock[i + 1]))
        {
            retval = CH347Jtag_INIT(ugIndex, i + 1);
            if (!retval)
            {
                LOG_ERROR("Couldn't set CH347 TCK speed");
                return retval;
            }
            else
            {
                break;
            }
        }
        else if (speed < speed_clock[0])
        {
            retval = CH347Jtag_INIT(ugIndex, 0);
            if (!retval)
            {
                LOG_ERROR("Couldn't set CH347 TCK speed");
                return retval;
            }
            else
            {
                break;
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
    if (khz == 0)
    {
        LOG_ERROR("Couldn't support the adapter speed");
        return ERROR_FAIL;
    }
    *jtag_speed = khz * 1000;
    return ERROR_OK;
}

COMMAND_HANDLER(ch347_handle_vid_pid_command)
{
    // TODO
    return ERROR_OK;
}

static const struct command_registration ch347_subcommand_handlers[] = {
    {
        .name = "vid_pid",
        .handler = ch347_handle_vid_pid_command,
        .mode = COMMAND_CONFIG,
        .help = "",
        .usage = "",
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

static struct jtag_interface ch347_interface = {
    .supported = DEBUG_CAP_TMS_SEQ,
    .execute_queue = ch347_execute_queue,
};

struct adapter_driver ch347_adapter_driver = {
    .name = "ch347",
    .transports = jtag_only,
    .commands = ch347_command_handlers,

    .init = ch347_init,
    .quit = ch347_quit,
    .speed = ch347_speed,
    .khz = ch347_khz,
    .speed_div = ch347_speed_div,

    .jtag_ops = &ch347_interface,
};