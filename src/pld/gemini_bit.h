/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023 by Rapid Silicon                                   *
 *   chernyee.kok@rapidsilicon.com                                         *
 ***************************************************************************/

#ifndef OPENOCD_PLD_GEMINI_BIT_H
#define OPENOCD_PLD_GEMINI_BIT_H

#include "helper/types.h"

typedef struct ubi_header {
	uint16_t      ubiHeadrVersion;
	uint16_t      headerSize;  //ubi header size
	//External Image number (64bits) :
	// productId (b[63:48]) | customer build id (b[47:32]) | image version(b[31:0])
	uint32_t      einMsw;      //External Image version number msb
	uint32_t      einLsw;      //External Image version number lsb
	uint8_t       imageType;       
	uint8_t       packageCount;
	uint16_t      crc16;
} __attribute__((packed)) ubi_header_t;

typedef struct bop_header {
	uint32_t      bopId;        //byte [3:1] 'B','O','P', byte[0] : binary type
	uint16_t      bopHeaderVersion;    //bop header version 
	uint16_t      signToolVersion;     //signing tool version
	uint32_t      binaryVersion;
	uint32_t      binaryLen;
	uint32_t      loadAddr;
	uint32_t      entryAddr;
	uint32_t      offsetToBinary;
	uint32_t      offsetToNextHeader;
	uint8_t       sigAlgo;
	uint8_t       option;
	uint8_t       encAlgo;
	uint8_t       ivLen;     //lenth of iv. (current only supports 16 bytes)
	uint16_t      pubKeyLen; //
	uint16_t      encKeyLen; //16 or 32 bytes
	uint16_t      sigLen;
	uint8_t       compressionAlgo;
	uint8_t       binPadBytes; //number of padding bytes in the payload binary or bitstream
	uint16_t      xcbHeaderSize;
	uint8_t       padding[16]; // To make BOP header 64 bytes for scatter gather hash calculation
	uint16_t      crc16;
} __attribute__((packed)) bop_header_t;

typedef struct gemini_bit_file
{
	ubi_header_t *ubi_header;
	bop_header_t **bop_header_list;
	uint8_t *rawdata;
	long filesize;
} gemini_bit_file_t;

int gemini_create_helper_bitstream(gemini_bit_file_t *bit_file, uint8_t **bitstream, uint32_t *filesize);
int gemini_read_bit_file(gemini_bit_file_t *bit_file, const char *filename);
void gemini_free_bit_file(gemini_bit_file_t *bit_file);

#endif /* OPENOCD_PLD_GEMINI_BIT_H */
