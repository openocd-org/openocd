// SPDX-License-Identifier: GPL-2.0-or-later
/****************************************************************************
	File : i2c.cpp                                                          *
	Contents : i2c bit-bang library                                         *
	Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.              *
	<aboudjelida@nanoxplore.com>                                            *
	<ahmederrachedbjld@gmail.com>											*
*****************************************************************************/

#include "i2c.h"
#include "io.h"
#include "delay.h"
#include "reg_ezusb.h"

bool get_status(void)
{
	PIN_SDA_DIR = 1;
	OEA = 0xF7;
	delay_us(1);
	bool sda_state = PIN_SDA;
	PIN_T0 = sda_state;
	delay_us(1);
	OEA = 0xFF;
	delay_us(1);
	return sda_state;
}

void start_cd(void)
{
	PIN_SDA_DIR = 0;    // SP6 SDA: OUT
	delay_us(10);
	PIN_SDA = 0;
	delay_us(1);
	PIN_SCL = 0;
	delay_us(1);
}

void repeated_start(void)
{
	PIN_SDA = 1;
	delay_us(1);
	PIN_SCL = 1;
	delay_us(1);
	PIN_SDA = 0;
	delay_us(1);
	PIN_SCL = 0;
	delay_us(1);
}

void stop_cd(void)
{
	PIN_SDA = 0;
	delay_us(1);
	PIN_SCL = 1;
	delay_us(1);
	PIN_SDA = 1;
	delay_us(1);
	PIN_SDA_DIR = 1;    // SP6 SDA: IN
	delay_us(1);
}

void clock_cd(void)
{
	PIN_SCL = 1;
	delay_us(1);
	PIN_SCL = 0;
	delay_us(1);
}

void send_ack(void)
{
	PIN_SDA = 0;
	delay_us(1);
	PIN_SCL = 1;
	delay_us(1);
	PIN_SCL = 0;
	delay_us(1);
}

void send_nack(void)
{
	PIN_SDA = 1;
	delay_us(1);
	PIN_SCL = 1;
	delay_us(1);
	PIN_SCL = 0;
	delay_us(1);
}

bool get_ack(void)
{
	PIN_SDA_DIR = 1;    // SP6 SDA: IN
	delay_us(1);
	OEA = 0xF7;         // FX2 SDA: IN
	PIN_SCL = 1;
	delay_us(1);
	bool ack = PIN_SDA;
	PIN_SCL = 0;
	delay_us(1);
	OEA = 0xFF;         // FX2 SDA: OUT
	PIN_SDA_DIR = 0;    // SP6 SDA: OUT
	delay_us(1);
	return ack;
}

/* here address(8 bits) = adr (7 bits) + type (1 bit) */
uint8_t get_address(uint8_t adr, uint8_t rdwr)
{
	adr &= 0x7F;
	adr = adr << 1;
	adr |= (rdwr & 0x01);
	return adr;
}

/* here send bit after bit and clocking scl with each bit */
void send_byte(uint8_t input)
{
	for (uint8_t i = 0; i < 8; i++) {
		if ((input & 0x80)) {
			PIN_SDA = 1;
			delay_us(1);
			clock_cd();
		} else {
			PIN_SDA = 0;
			delay_us(1);
			clock_cd();
		}
	input = input << 1;
	}
}

/* here receive bit after bit and clocking scl with each bit */

uint8_t receive_byte(void)
{
	PIN_SDA_DIR = 1;    // SP6 SDA: IN
	OEA = 0xF7;         // FX2 SDA: IN
	uint8_t input = 0x00;
	for (uint8_t i = 0; i < 8; i++) {
		PIN_SCL = 1;
		delay_us(1);
		input = input << 1;
		if (PIN_SDA == 1)
			input |= 0x01;
		else
			input |= 0X00;

		PIN_SCL = 0;
		delay_us(1);
	}
	OEA = 0xFF;         // FX2 SDA: OUT
	PIN_SDA_DIR = 0;    // SP6 SDA: OUT
	return input;
}
