/***************************************************************************
 *   Copyright (C) 2007 by Pavel Chromy                                    *
 *   chromy@asix.cz                                                        *
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

#if IS_CYGWIN == 1
#include "windows.h"
#undef ERROR
#endif

#include "replacements.h"

/* project specific includes */
#include "log.h"
#include "types.h"
#include "jtag.h"
#include "configuration.h"
#include "time_support.h"
#include "bitq.h"

/* system includes */
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include <sys/time.h>
#include <time.h>

#include "ftd2xx.h"


int presto_jtag_speed(int speed);
int presto_jtag_register_commands(struct command_context_s *cmd_ctx);
int presto_jtag_init(void);
int presto_jtag_quit(void);

jtag_interface_t presto_interface =
{
	.name = "presto",
	.execute_queue = bitq_execute_queue,
	.support_pathmove = 1,
	.speed = presto_jtag_speed,
	.register_commands = presto_jtag_register_commands,
	.init = presto_jtag_init,
	.quit = presto_jtag_quit,
};


int presto_bitq_out(int tms, int tdi, int tdo_req);
int presto_bitq_flush(void);
int presto_bitq_sleep(unsigned long us);
int presto_bitq_reset(int rst, int en);
int presto_bitq_in_rdy(void);
int presto_bitq_in(void);

bitq_interface_t presto_bitq =
{
	.out = presto_bitq_out,
	.flush = presto_bitq_flush,
	.sleep = presto_bitq_sleep,
	.reset = presto_bitq_reset,
	.in_rdy = presto_bitq_in_rdy,
	.in = presto_bitq_in,
};


/* -------------------------------------------------------------------------- */


#define FT_DEVICE_NAME_LEN 64
#define FT_DEVICE_SERNUM_LEN 64

#define PRESTO_VID_PID 0x0403f1a0

#define PRST_OK 0
#define PRST_ERR 1
#define PRST_TIMEOUT 2

#define BUFFER_SIZE (64*62)


typedef struct presto_s
{
	FT_HANDLE handle;
	FT_STATUS status;
	
	char serial[FT_DEVICE_SERNUM_LEN];

	BYTE buff_out[BUFFER_SIZE];
	int buff_out_pos;

	BYTE buff_in[BUFFER_SIZE];
	int buff_in_exp; /* expected in buffer length */
	int buff_in_len; /* length of data received */
	int buff_in_pos;

	unsigned long total_out;
	unsigned long total_in;

	int jtag_tms; /* last tms state */
	int jtag_tck; /* last tck state */

	int jtag_tdi_data;
	int jtag_tdi_count;

} presto_t;

presto_t presto_state;
presto_t *presto=&presto_state;

BYTE presto_init_seq[] =
{
	0x80,	0xA0,	0xA8,	0xB0,	0xC0,	0xE0
};


int presto_open(char *req_serial)
{
	int i;
	int result;
	DWORD numdevs;
	DWORD vidpid;
	char devname[FT_DEVICE_NAME_LEN];
	FT_DEVICE device;

	BYTE presto_data;
	unsigned long ftbytes;

	presto->handle=INVALID_HANDLE_VALUE;

	presto->buff_out_pos=0;
	presto->buff_in_pos=0;
	presto->buff_in_len=0;
	presto->buff_in_exp=0;

	presto->total_out=0;
	presto->total_in=0;

	presto->jtag_tms=0;
	presto->jtag_tck=0;
	presto->jtag_tdi_data=0;
	presto->jtag_tdi_count=0;

	if (FT_ListDevices(&numdevs, NULL, FT_LIST_NUMBER_ONLY)!=FT_OK) return PRST_ERR;
	for (i=0; i<numdevs; i++)
	{
		if (FT_Open(i, &(presto->handle))!=FT_OK) continue;
		if (FT_GetDeviceInfo(presto->handle,&device,&vidpid,presto->serial,devname,NULL)==FT_OK)
		{
			if (vidpid==PRESTO_VID_PID && (req_serial==NULL || !strcmp(presto->serial,req_serial)))
				break;
		}
		FT_Close(presto->handle);
		presto->handle=INVALID_HANDLE_VALUE;
	}

	if (presto->handle==INVALID_HANDLE_VALUE) return PRST_ERR;

	if ((presto->status=FT_SetLatencyTimer(presto->handle,1))!=FT_OK) return PRST_ERR;
	if ((presto->status=FT_SetTimeouts(presto->handle,100,0))!=FT_OK) return PRST_ERR;
	if ((presto->status=FT_Purge(presto->handle, FT_PURGE_TX|FT_PURGE_RX))!=FT_OK) return PRST_ERR;

	presto_data=0xD0;
	if ((presto->status=FT_Write(presto->handle,&presto_data,1,&ftbytes))!=FT_OK) return PRST_ERR;
	if ((presto->status=FT_Read(presto->handle,&presto_data,1,&ftbytes))!=FT_OK) return PRST_ERR;

	if (ftbytes!=1)
	{
		if ((presto->status=FT_SetBitMode(presto->handle,0x80,1))!=FT_OK) return PRST_ERR;
		if ((presto->status=FT_Purge(presto->handle, FT_PURGE_TX|FT_PURGE_RX))!=FT_OK) return PRST_ERR ;
		if ((presto->status=FT_SetBaudRate(presto->handle,9600))!=FT_OK) return PRST_ERR;

		presto_data=0;
		for (i=0; i<4*62; i++)
			if ((presto->status=FT_Write(presto->handle,&presto_data,1,&ftbytes))!=FT_OK) return PRST_ERR;

		usleep(100000);

		if ((presto->status=FT_SetBitMode(presto->handle,0x00,0))!=FT_OK) return PRST_ERR;
		if ((presto->status=FT_Purge(presto->handle, FT_PURGE_TX|FT_PURGE_RX))!=FT_OK) return PRST_ERR;

		presto_data=0xD0;
		if ((presto->status=FT_Write(presto->handle,&presto_data,1,&ftbytes))!=FT_OK) return PRST_ERR;
		if ((presto->status=FT_Read(presto->handle,&presto_data,1,&ftbytes))!=FT_OK) return PRST_ERR;
		if (ftbytes!=1) return PRST_ERR;
	}

	if ((presto->status=FT_SetTimeouts(presto->handle,0,0))!=FT_OK) return PRST_ERR;

	presto->status=FT_Write(presto->handle,&presto_init_seq,sizeof(presto_init_seq),&ftbytes);
	if (presto->status!=FT_OK) return PRST_ERR;
	if (ftbytes!=sizeof(presto_init_seq)) return PRST_TIMEOUT;

	return PRST_OK;
}


int presto_close(void)
{
	unsigned long ftbytes;

	int result=PRST_OK;

	if (presto->handle==INVALID_HANDLE_VALUE) return result;

	presto->status=FT_Write(presto->handle,&presto_init_seq,sizeof(presto_init_seq),&ftbytes);
	if (presto->status!=FT_OK) result=PRST_ERR;
	if (ftbytes!=sizeof(presto_init_seq)) result=PRST_TIMEOUT;

	if ((presto->status=FT_SetLatencyTimer(presto->handle,16))!=FT_OK) result=PRST_ERR;

	if ((presto->status=FT_Close(presto->handle))!=FT_OK) result=PRST_ERR;
	else presto->handle=INVALID_HANDLE_VALUE;

	return result;
}


int presto_flush(void)
{
	unsigned long ftbytes;

	if (presto->buff_out_pos==0) return PRST_OK;
	if (presto->status!=FT_OK) return PRST_ERR;

	if ((presto->status=FT_Write(presto->handle, presto->buff_out, presto->buff_out_pos, &ftbytes))!=FT_OK)
	{
		presto->buff_out_pos=0;
		return PRST_ERR;
	}

	presto->total_out+=ftbytes;

	if (presto->buff_out_pos!=ftbytes)
	{
		presto->buff_out_pos=0;
		return PRST_TIMEOUT;
	}

	presto->buff_out_pos=0;

	if (presto->buff_in_exp==0) return PRST_OK;

	presto->buff_in_pos=0;
	presto->buff_in_len=0;

	if ((presto->status=FT_Read(presto->handle, presto->buff_in, presto->buff_in_exp, &ftbytes))!=FT_OK)
	{
		presto->buff_in_exp=0;
		return PRST_ERR;
	}

	presto->total_in+=ftbytes;

	if (ftbytes!=presto->buff_in_exp)
	{
		presto->buff_in_exp=0;
		return PRST_TIMEOUT;
	}

	presto->buff_in_len=presto->buff_in_exp;
	presto->buff_in_exp=0;

	return PRST_OK;
}


int presto_sendbyte(int data)
{
	if (data==EOF) return presto_flush();

	if (presto->buff_out_pos < BUFFER_SIZE)
	{
		presto->buff_out[presto->buff_out_pos++]=(BYTE)data;
		if (((data&0xC0)==0x40) || ((data&0xD0)==0xD0))
			presto->buff_in_exp++;
	}
	else return PRST_ERR;

	if (presto->buff_out_pos >= BUFFER_SIZE) return presto_flush();
	return PRST_OK;
}


int presto_getbyte(void)
{
	if (presto->buff_in_pos<presto->buff_in_len)
		return presto->buff_in[presto->buff_in_pos++];

	if (presto->buff_in_exp==0) return -1;
	if (presto_flush()!=PRST_OK) return -1;

	if (presto->buff_in_pos<presto->buff_in_len)
		return presto->buff_in[presto->buff_in_pos++];

	return -1;
}


/* -------------------------------------------------------------------------- */


int presto_bitq_out(int tms, int tdi, int tdo_req)
{
	unsigned char cmdparam;

	if (presto->jtag_tck==0)
	{
		presto_sendbyte(0xA4);
		presto->jtag_tck=1;
	}

	else if (!tdo_req && tms==presto->jtag_tms)
	{
		if (presto->jtag_tdi_count==0) presto->jtag_tdi_data=(tdi!=0);
		else presto->jtag_tdi_data|=(tdi!=0)<<presto->jtag_tdi_count;
		if (++presto->jtag_tdi_count==4)
		{
			presto->jtag_tdi_data|=(presto->jtag_tdi_count-1)<<4;
			presto_sendbyte(presto->jtag_tdi_data);
			presto->jtag_tdi_count=0;
		}
		return 0;
	}

	if (presto->jtag_tdi_count)
	{
		presto->jtag_tdi_data|=(presto->jtag_tdi_count-1)<<4;
		presto_sendbyte(presto->jtag_tdi_data);
		presto->jtag_tdi_count=0;
	}

	if (tdi) cmdparam=0x0B;
	else cmdparam=0x0A;

	presto_sendbyte(0xC0|cmdparam);

	if (tms!=presto->jtag_tms)
	{
		if (tms) presto_sendbyte(0xEC);
		else presto_sendbyte(0xE8);
		presto->jtag_tms=tms;
	}

	if (tdo_req) presto_sendbyte(0xD4|cmdparam);
	else presto_sendbyte(0xC4|cmdparam);

	return 0;
}


int presto_bitq_flush(void)
{
	if (presto->jtag_tdi_count)
	{
		presto->jtag_tdi_data|=(presto->jtag_tdi_count-1)<<4;
		presto_sendbyte(presto->jtag_tdi_data);
		presto->jtag_tdi_count=0;
	}

	presto_sendbyte(0xCA);
	presto->jtag_tck=0;

	presto_sendbyte(0xA0);

	return presto_flush();
}


int presto_bitq_in_rdy(void)
{
	if (presto->buff_in_pos>=presto->buff_in_len) return 0;
	return presto->buff_in_len-presto->buff_in_pos;
}


int presto_bitq_in(void)
{
	if (presto->buff_in_pos>=presto->buff_in_len) return -1;
	if (presto->buff_in[presto->buff_in_pos++]&0x08) return 1;
	return 0;
}


int presto_bitq_sleep(unsigned long us)
{
	long waits;

	if (us>100000)
	{
		presto_bitq_flush();
		jtag_sleep(us);
		return 0;
	}

	waits=us/170+2;
	while (waits--) presto_sendbyte(0x80);

	return 0;
}


int presto_bitq_reset(int trst, int srst)
{
	unsigned char cmd;

	cmd=0xE8;
	if (presto->jtag_tms) cmd|=0x04;

	if (trst || srst) cmd|=0x02;

	presto_sendbyte(cmd);
	return 0;
}


/* -------------------------------------------------------------------------- */


int presto_jtag_speed(int speed)
{
	jtag_speed=speed;
	return ERROR_OK;
}


char *presto_serial;

int presto_handle_serial_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 1)
	{
		if (presto_serial) free(presto_serial);
		presto_serial = strdup(args[0]);
	}
	else
	{
		ERROR("expected exactly one argument to presto_serial <serial-number>");
	}

	return ERROR_OK;
}


int presto_jtag_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "presto_serial", presto_handle_serial_command,
		COMMAND_CONFIG, NULL);
	return ERROR_OK;
}


int presto_jtag_init(void)
{
	if (presto_open(presto_serial)!=0)
	{
		presto_close();
		if (presto_serial!=NULL) ERROR("Cannot open PRESTO, serial number %s", presto_serial);
		else ERROR("Cannot open PRESTO");
		return ERROR_JTAG_INIT_FAILED;
	}
	INFO("PRESTO open, serial number %s", presto->serial);
	
	bitq_interface=&presto_bitq;
	return ERROR_OK;
}


int presto_jtag_quit(void)
{
	bitq_cleanup();
	presto_close();
	INFO("PRESTO closed");

	if (presto_serial)
	{
		free(presto_serial);
		presto_serial=NULL;
	}

	return ERROR_OK;
}
