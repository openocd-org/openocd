/***************************************************************************
 *   Copyright (C) 2007-2008 by Øyvind Harboe                              *
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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif


#include "log.h"
#include "jtag.h"
#include "bitbang.h"
#include "../target/embeddedice.h"


#include <cyg/hal/hal_io.h>             // low level i/o
#include <cyg/hal/hal_diag.h>

#include <stdlib.h>


extern int jtag_error;

/* low level command set
 */
int zy1000_read(void);
static void zy1000_write(int tck, int tms, int tdi);
void zy1000_reset(int trst, int srst);


int zy1000_speed(int speed);
int zy1000_register_commands(struct command_context_s *cmd_ctx);
int zy1000_init(void);
int zy1000_quit(void);

/* interface commands */
int zy1000_handle_zy1000_port_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

static int zy1000_khz(int khz, int *jtag_speed)
{
	if (khz==0)
	{
		*jtag_speed=0;
	}
	else
	{
		*jtag_speed=64000/khz;
	}
	return ERROR_OK;
}

static int zy1000_speed_div(int speed, int *khz)
{
	if (speed==0)
	{
		*khz = 0;
	}
	else
	{
		*khz=64000/speed;
	}

	return ERROR_OK;
}

static bool readPowerDropout()
{
	cyg_uint32 state;
	// sample and clear power dropout
	HAL_WRITE_UINT32(0x08000010, 0x80);
	HAL_READ_UINT32(0x08000010, state);
	bool powerDropout;
	powerDropout = (state & 0x80) != 0;
	return powerDropout;
}


static bool readSRST()
{
	cyg_uint32 state;
	// sample and clear SRST sensing
	HAL_WRITE_UINT32(0x08000010, 0x00000040);
	HAL_READ_UINT32(0x08000010, state);
	bool srstAsserted;
	srstAsserted = (state & 0x40) != 0;
	return srstAsserted;
}

static int zy1000_srst_asserted(int *srst_asserted)
{
	*srst_asserted=readSRST();
	return ERROR_OK;
}

static int zy1000_power_dropout(int *dropout)
{
	*dropout=readPowerDropout();
	return ERROR_OK;
}


jtag_interface_t zy1000_interface =
{
	.name = "ZY1000",
	.execute_queue = bitbang_execute_queue,
	.speed = zy1000_speed,
	.register_commands = zy1000_register_commands,
	.init = zy1000_init,
	.quit = zy1000_quit,
	.khz = zy1000_khz,
	.speed_div = zy1000_speed_div,
	.power_dropout = zy1000_power_dropout,
	.srst_asserted = zy1000_srst_asserted,
};

bitbang_interface_t zy1000_bitbang =
{
	.read = zy1000_read,
	.write = zy1000_write,
	.reset = zy1000_reset
};



static void zy1000_write(int tck, int tms, int tdi)
{

}

int zy1000_read(void)
{
	return -1;
}

extern bool readSRST();

void zy1000_reset(int trst, int srst)
{
	LOG_DEBUG("zy1000 trst=%d, srst=%d", trst, srst);
	if(!srst)
	{
		ZY1000_POKE(0x08000014, 0x00000001);
	}
	else
	{
		/* Danger!!! if clk!=0 when in
		 * idle in TAP_RTI, reset halt on str912 will fail.
		 */
		ZY1000_POKE(0x08000010, 0x00000001);
	}

	if(!trst)
	{
		ZY1000_POKE(0x08000014, 0x00000002);
	}
	else
	{
		/* assert reset */
		ZY1000_POKE(0x08000010, 0x00000002);
	}

	if (trst||(srst&&(jtag_reset_config & RESET_SRST_PULLS_TRST)))
	{
		waitIdle();
		/* we're now in the TLR state until trst is deasserted */
		ZY1000_POKE(0x08000020, TAP_TLR);
	} else
	{
		/* We'll get RCLK failure when we assert TRST, so clear any false positives here */
		ZY1000_POKE(0x08000014, 0x400);
	}

	/* wait for srst to float back up */
	if (!srst)
	{
		int i;
		for (i=0; i<1000; i++)
		{
			// We don't want to sense our own reset, so we clear here.
			// There is of course a timing hole where we could loose
			// a "real" reset.
			if (!readSRST())
				break;

			/* wait 1ms */
			alive_sleep(1);
		}

		if (i==1000)
		{
			LOG_USER("SRST didn't deassert after %dms", i);
		} else if (i>1)
		{
			LOG_USER("SRST took %dms to deassert", i);
		}
	}
}

int zy1000_speed(int speed)
{
	if(speed == 0)
	{
		/*0 means RCLK*/
		speed = 0;
		ZY1000_POKE(0x08000010, 0x100);
		LOG_DEBUG("jtag_speed using RCLK");
	}
	else
	{
		if(speed > 8190 || speed < 2)
		{
			LOG_ERROR("valid ZY1000 jtag_speed=[8190,2]. Divisor is 64MHz / even values between 8190-2, i.e. min 7814Hz, max 32MHz");
			return ERROR_INVALID_ARGUMENTS;
		}

		LOG_USER("jtag_speed %d => JTAG clk=%f", speed, 64.0/(float)speed);
		ZY1000_POKE(0x08000014, 0x100);
		ZY1000_POKE(0x0800001c, speed&~1);
	}
	return ERROR_OK;
}

int zy1000_register_commands(struct command_context_s *cmd_ctx)
{
	return ERROR_OK;
}


int zy1000_init(void)
{
	ZY1000_POKE(0x08000010, 0x30); // Turn on LED1 & LED2

	 /* deassert resets. Important to avoid infinite loop waiting for SRST to deassert */
	zy1000_reset(0, 0);
	zy1000_speed(jtag_speed);

	bitbang_interface = &zy1000_bitbang;

	return ERROR_OK;
}

int zy1000_quit(void)
{

	return ERROR_OK;
}



/* loads a file and returns a pointer to it in memory. The file contains
 * a 0 byte(sentinel) after len bytes - the length of the file. */
int loadFile(const char *fileName, void **data, int *len)
{
	FILE * pFile;
	pFile = fopen (fileName,"rb");
	if (pFile==NULL)
	{
		LOG_ERROR("Can't open %s\n", fileName);
		return ERROR_JTAG_DEVICE_ERROR;
	}
    if (fseek (pFile, 0, SEEK_END)!=0)
    {
		LOG_ERROR("Can't open %s\n", fileName);
		fclose(pFile);
		return ERROR_JTAG_DEVICE_ERROR;
    }
    *len=ftell (pFile);
    if (*len==-1)
    {
		LOG_ERROR("Can't open %s\n", fileName);
		fclose(pFile);
		return ERROR_JTAG_DEVICE_ERROR;
    }

    if (fseek (pFile, 0, SEEK_SET)!=0)
    {
		LOG_ERROR("Can't open %s\n", fileName);
		fclose(pFile);
		return ERROR_JTAG_DEVICE_ERROR;
    }
    *data=malloc(*len+1);
    if (*data==NULL)
    {
		LOG_ERROR("Can't open %s\n", fileName);
		fclose(pFile);
		return ERROR_JTAG_DEVICE_ERROR;
    }

    if (fread(*data, 1, *len, pFile)!=*len)
    {
		fclose(pFile);
    	free(*data);
		LOG_ERROR("Can't open %s\n", fileName);
		return ERROR_JTAG_DEVICE_ERROR;
    }
    fclose (pFile);
    *(((char *)(*data))+*len)=0; /* sentinel */

    return ERROR_OK;



}




int interface_jtag_execute_queue(void)
{
	cyg_uint32 empty;

	waitIdle();
	ZY1000_PEEK(0x08000010, empty);
	/* clear JTAG error register */
	ZY1000_POKE(0x08000014, 0x400);

	if ((empty&0x400)!=0)
	{
		LOG_WARNING("RCLK timeout");
		/* the error is informative only as we don't want to break the firmware if there
		 * is a false positive.
		 */
//		return ERROR_FAIL;
	}
	return ERROR_OK;
}





static cyg_uint32 getShiftValue()
{
	cyg_uint32 value;
	waitIdle();
	ZY1000_PEEK(0x0800000c, value);
	VERBOSE(LOG_INFO("getShiftValue %08x", value));
	return value;
}
#if 0
static cyg_uint32 getShiftValueFlip()
{
	cyg_uint32 value;
	waitIdle();
	ZY1000_PEEK(0x08000018, value);
	VERBOSE(LOG_INFO("getShiftValue %08x (flipped)", value));
	return value;
}
#endif

#if 0
static void shiftValueInnerFlip(const enum tap_state state, const enum tap_state endState, int repeat, cyg_uint32 value)
{
	VERBOSE(LOG_INFO("shiftValueInner %s %s %d %08x (flipped)", tap_state_strings[state], tap_state_strings[endState], repeat, value));
	cyg_uint32 a,b;
	a=state;
	b=endState;
	ZY1000_POKE(0x0800000c, value);
	ZY1000_POKE(0x08000008, (1<<15)|(repeat<<8)|(a<<4)|b);
	VERBOSE(getShiftValueFlip());
}
#endif

extern int jtag_check_value(u8 *captured, void *priv);

static void gotoEndState()
{
	setCurrentState(cmd_queue_end_state);
}

static __inline void scanFields(int num_fields, scan_field_t *fields, enum tap_state shiftState, int pause)
{
	int i;
	int j;
	int k;

	for (i = 0; i < num_fields; i++)
	{
		cyg_uint32 value;

		static u8 *in_buff=NULL; /* pointer to buffer for scanned data */
		static int in_buff_size=0;
		u8 *inBuffer=NULL;


		// figure out where to store the input data
		int num_bits=fields[i].num_bits;
		if (fields[i].in_value!=NULL)
		{
			inBuffer=fields[i].in_value;
		} else if (fields[i].in_handler!=NULL)
		{
			if (in_buff_size*8<num_bits)
			{
				// we need more space
				if (in_buff!=NULL)
					free(in_buff);
				in_buff=NULL;
				in_buff_size=(num_bits+7)/8;
				in_buff=malloc(in_buff_size);
				if (in_buff==NULL)
				{
					LOG_ERROR("Out of memory");
					jtag_error=ERROR_JTAG_QUEUE_FAILED;
					return;
				}
			}
			inBuffer=in_buff;
		}

		// here we shuffle N bits out/in
		j=0;
		while (j<num_bits)
		{
			enum tap_state pause_state;
			int l;
			k=num_bits-j;
			pause_state=(shiftState==TAP_SD)?TAP_SD:TAP_SI;
			if (k>32)
			{
				k=32;
				/* we have more to shift out */
			} else if (pause&&(i == num_fields-1))
			{
				/* this was the last to shift out this time */
				pause_state=(shiftState==TAP_SD)?TAP_PD:TAP_PI;
			}

			// we have (num_bits+7)/8 bytes of bits to toggle out.
			// bits are pushed out LSB to MSB
			value=0;
			if (fields[i].out_value!=NULL)
			{
				for (l=0; l<k; l+=8)
				{
					value|=fields[i].out_value[(j+l)/8]<<l;
				}
			}
			/* mask away unused bits for easier debugging */
			value&=~(((u32)0xffffffff)<<k);

			shiftValueInner(shiftState, pause_state, k, value);

			if (inBuffer!=NULL)
			{
				// data in, LSB to MSB
				value=getShiftValue();
				// we're shifting in data to MSB, shift data to be aligned for returning the value
				value >>= 32-k;

				for (l=0; l<k; l+=8)
				{
					inBuffer[(j+l)/8]=(value>>l)&0xff;
				}
			}
			j+=k;
		}

		if (fields[i].in_handler!=NULL)
		{
			// invoke callback
			int r=fields[i].in_handler(inBuffer, fields[i].in_handler_priv, fields+i);
			if (r!=ERROR_OK)
			{
			    /* this will cause jtag_execute_queue() to return an error */
				jtag_error=r;
			}
		}
	}
}

int interface_jtag_add_end_state(enum tap_state state)
{
	return ERROR_OK;
}


int interface_jtag_add_ir_scan(int num_fields, scan_field_t *fields, enum tap_state state)
{

	int i, j;
	int scan_size = 0;
	jtag_device_t *device;

	for (i=0; i < jtag_num_devices; i++)
	{
		int pause=i==(jtag_num_devices-1);
		int found = 0;
		device = jtag_get_device(i);
		if (device==NULL)
		{
			return ERROR_FAIL;
		}

		scan_size = device->ir_length;

		/* search the list */
		for (j=0; j < num_fields; j++)
		{
			if (i == fields[j].device)
			{
				found = 1;

				if ((jtag_verify_capture_ir)&&(fields[j].in_handler==NULL))
				{
					jtag_set_check_value(fields+j, device->expected, device->expected_mask, NULL);
				} else if (jtag_verify_capture_ir)
				{
					fields[j].in_check_value = device->expected;
					fields[j].in_check_mask = device->expected_mask;
				}

				scanFields(1, fields+j, TAP_SI, pause);
				/* update device information */
				buf_cpy(fields[j].out_value, device->cur_instr, scan_size);

				device->bypass = 0;
				break;
			}
		}

		if (!found)
		{
			/* if a device isn't listed, set it to BYPASS */
			u8 ones[]={0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff,0xff};

			scan_field_t tmp;
			memset(&tmp, 0, sizeof(tmp));
			tmp.out_value = ones;
			tmp.num_bits = scan_size;
			scanFields(1, &tmp, TAP_SI, pause);
			/* update device information */
			buf_cpy(tmp.out_value, device->cur_instr, scan_size);
			device->bypass = 1;
		}
	}
	gotoEndState();

	return ERROR_OK;
}





int interface_jtag_add_plain_ir_scan(int num_fields, scan_field_t *fields, enum tap_state state)
{
	scanFields(num_fields, fields, TAP_SI, 1);
	gotoEndState();

	return ERROR_OK;
}

/*extern jtag_command_t **jtag_get_last_command_p(void);*/

int interface_jtag_add_dr_scan(int num_fields, scan_field_t *fields, enum tap_state state)
{
	int i, j;
	for (i=0; i < jtag_num_devices; i++)
	{
		int found = 0;
		int pause = (i==(jtag_num_devices-1));

		for (j=0; j < num_fields; j++)
		{
			if (i == fields[j].device)
			{
				found = 1;

				scanFields(1, fields+j, TAP_SD, pause);
			}
		}
		if (!found)
		{
#ifdef _DEBUG_JTAG_IO_
			/* if a device isn't listed, the BYPASS register should be selected */
			if (!jtag_get_device(i)->bypass)
			{
				LOG_ERROR("BUG: no scan data for a device not in BYPASS");
				exit(-1);
			}
#endif

			scan_field_t tmp;
			/* program the scan field to 1 bit length, and ignore it's value */
			tmp.num_bits = 1;
			tmp.out_value = NULL;
			tmp.out_mask = NULL;
			tmp.in_value = NULL;
			tmp.in_check_value = NULL;
			tmp.in_check_mask = NULL;
			tmp.in_handler = NULL;
			tmp.in_handler_priv = NULL;

			scanFields(1, &tmp, TAP_SD, pause);
		}
		else
		{
#ifdef _DEBUG_JTAG_IO_
			/* if a device is listed, the BYPASS register must not be selected */
			if (jtag_get_device(i)->bypass)
			{
				LOG_WARNING("scan data for a device in BYPASS");
			}
#endif
		}
	}
	gotoEndState();
	return ERROR_OK;
}

int interface_jtag_add_plain_dr_scan(int num_fields, scan_field_t *fields, enum tap_state state)
{
	scanFields(num_fields, fields, TAP_SD, 1);
	gotoEndState();
	return ERROR_OK;
}


int interface_jtag_add_tlr()
{
	setCurrentState(TAP_TLR);
	return ERROR_OK;
}




extern int jtag_nsrst_delay;
extern int jtag_ntrst_delay;

int interface_jtag_add_reset(int req_trst, int req_srst)
{
	zy1000_reset(req_trst, req_srst);
	return ERROR_OK;
}

int interface_jtag_add_runtest(int num_cycles, enum tap_state state)
{
	/* num_cycles can be 0 */
	setCurrentState(TAP_RTI);

	/* execute num_cycles, 32 at the time. */
	int i;
	for (i=0; i<num_cycles; i+=32)
	{
		int num;
		num=32;
		if (num_cycles-i<num)
		{
			num=num_cycles-i;
		}
		shiftValueInner(TAP_RTI, TAP_RTI, num, 0);
	}

#if !TEST_MANUAL()
	/* finish in end_state */
	setCurrentState(state);
#else
	enum tap_state t=TAP_RTI;
	/* test manual drive code on any target */
	int tms;
	u8 tms_scan = TAP_MOVE(t, state);

	for (i = 0; i < 7; i++)
	{
		tms = (tms_scan >> i) & 1;
		waitIdle();
		ZY1000_POKE(0x08000028,  tms);
	}
	waitIdle();
	ZY1000_POKE(0x08000020, state);
#endif


	return ERROR_OK;
}

int interface_jtag_add_sleep(u32 us)
{
	jtag_sleep(us);
	return ERROR_OK;
}

int interface_jtag_add_pathmove(int num_states, enum tap_state *path)
{
	int state_count;
	int tms = 0;

	/*wait for the fifo to be empty*/
	waitIdle();

	state_count = 0;

	enum tap_state cur_state=cmd_queue_cur_state;

	while (num_states)
	{
		if (tap_transitions[cur_state].low == path[state_count])
		{
			tms = 0;
		}
		else if (tap_transitions[cur_state].high == path[state_count])
		{
			tms = 1;
		}
		else
		{
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition", tap_state_strings[cur_state], tap_state_strings[path[state_count]]);
			exit(-1);
		}

		waitIdle();
		ZY1000_POKE(0x08000028,  tms);

		cur_state = path[state_count];
		state_count++;
		num_states--;
	}

	waitIdle();
	ZY1000_POKE(0x08000020,  cur_state);
	return ERROR_OK;
}



void embeddedice_write_dcc(int chain_pos, int reg_addr, u8 *buffer, int little, int count)
{
//	static int const reg_addr=0x5;
	enum tap_state end_state=cmd_queue_end_state;
	if (jtag_num_devices==1)
	{
		/* better performance via code duplication */
		if (little)
		{
			int i;
			for (i = 0; i < count; i++)
			{
				shiftValueInner(TAP_SD, TAP_SD, 32, fast_target_buffer_get_u32(buffer, 1));
				shiftValueInner(TAP_SD, end_state, 6, reg_addr|(1<<5));
				buffer+=4;
			}
		} else
		{
			int i;
			for (i = 0; i < count; i++)
			{
				shiftValueInner(TAP_SD, TAP_SD, 32, fast_target_buffer_get_u32(buffer, 0));
				shiftValueInner(TAP_SD, end_state, 6, reg_addr|(1<<5));
				buffer+=4;
			}
		}
	}
	else
	{
		int i;
		for (i = 0; i < count; i++)
		{
			embeddedice_write_reg_inner(chain_pos, reg_addr, fast_target_buffer_get_u32(buffer, little));
			buffer += 4;
		}
	}
}

