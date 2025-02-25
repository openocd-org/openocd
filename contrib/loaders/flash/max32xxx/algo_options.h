/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2016 by Maxim Integrated                                *
 *   Copyright (C) 2025 Analog Devices, Inc.                               *
 ***************************************************************************/

#define OPTIONS_128                     0x01 /* Perform 128 bit flash writes */
#define OPTIONS_ENC                     0x02 /* Encrypt the flash contents */
#define OPTIONS_AUTH                    0x04 /* Authenticate the flash contents */
#define OPTIONS_COUNT                   0x08 /* Add counter values to authentication */
#define OPTIONS_INTER                   0x10 /* Interleave the authentication and count values*/
#define OPTIONS_RELATIVE_XOR            0x20 /* Only XOR the offset of the address when encrypting */
#define OPTIONS_KEYSIZE                 0x40 /* Use a 256 bit KEY */
