/* SPDX-License-Identifier: GPL-2.0-or-later */
/****************************************************************************
	File : i2c.h                                                            *
	Contents : i2c bit-bang library                                         *
	Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.              *
	<aboudjelida@nanoxplore.com>                                            *
	<ahmederrachedbjld@gmail.com>											*
*****************************************************************************/

#ifndef __I2C_H
#define __I2C_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

void start_cd(void);
void repeated_start(void);
void stop_cd(void);
void clock_cd(void);
void send_ack(void);
void send_nack(void);
bool get_ack(void);

uint8_t get_address(uint8_t adr, uint8_t rdwr);

void send_byte(uint8_t input);
uint8_t receive_byte(void);
#endif
