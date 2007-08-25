/***************************************************************************
 *   Copyright (C) 2007 by Benedikt Sauter sauter@ixbat.de		   *
 *   based on Dominic Rath's amt_jtagaccel.c			           *
 *									   * 
 *   usbprog is a free programming adapter. You can easily install	   *
 *   different firmware versions from an "online pool" over USB.	   * 
 *   The adapter can be used for programming and debugging AVR and ARM	   *
 *   processors, as USB to RS232 converter, as JTAG interface or as	   *
 *   simple I/O interface (5 lines).					   *
 *									   *
 *   http://www.embedded-projects.net/usbprog				   *
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

#include "replacements.h"

#include "jtag.h"
#include <usb.h>

/* system includes */

#include "log.h"

#define VID 0x1781
#define PID 0x0c63

// Pins at usbprog
#define TDO_BIT         0
#define TDI_BIT         3
#define TCK_BIT         2
#define TMS_BIT         1

int usbprog_execute_queue(void);
int usbprog_speed(int speed);
int usbprog_register_commands(struct command_context_s *cmd_ctx);
int usbprog_init(void);
int usbprog_quit(void);


void usbprog_end_state(enum tap_state state);
void usbprog_state_move(void);
void usbprog_path_move(pathmove_command_t *cmd);
void usbprog_runtest(int num_cycles);
void usbprog_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size);

jtag_interface_t usbprog_interface = 
{
	.name = "usbprog",
	.execute_queue = usbprog_execute_queue,
	.support_pathmove = 0,
	.speed = usbprog_speed,	
	.register_commands = usbprog_register_commands,
	.init = usbprog_init,
	.quit = usbprog_quit
};

#define UNKOWN_COMMAND  0x00
#define PORT_DIRECTION  0x01
#define PORT_SET        0x02
#define PORT_GET        0x03
#define PORT_SETBIT     0x04
#define PORT_GETBIT     0x05
#define WRITE_TDI     	0x06
#define READ_TDO     	0x07
#define WRITE_AND_READ 	0x08
#define WRITE_TMS     	0x09
#define WRITE_TMS_CHAIN 0x0A

struct usbprog_jtag 
{
	struct usb_dev_handle* usb_handle;
};

struct usbprog_jtag * usbprog_jtag_handle;

struct usbprog_jtag* usbprog_jtag_open();
void usbprog_jtag_close(struct usbprog_jtag *usbprog_jtag);
void usbprog_jtag_init(struct usbprog_jtag *usbprog_jtag);
unsigned char usbprog_jtag_message(struct usbprog_jtag *usbprog_jtag, char *msg, int msglen);


void usbprog_jtag_read_tdo(struct usbprog_jtag *usbprog_jtag, char * buffer, int size);
void usbprog_jtag_write_tdi(struct usbprog_jtag *usbprog_jtag, char * buffer, int size);
void usbprog_jtag_write_and_read(struct usbprog_jtag *usbprog_jtag, char * buffer, int size);
void usbprog_jtag_write_tms(struct usbprog_jtag *usbprog_jtag, char tms_scan);

char tms_chain[64];
int tms_chain_index;
void usbprog_jtag_tms_collect(char tms_scan);
void usbprog_jtag_tms_send(struct usbprog_jtag *usbprog_jtag);

void usbprog_write(int tck, int tms, int tdi);
void usbprog_reset(int trst, int srst);

void usbprog_jtag_set_direction(struct usbprog_jtag *usbprog_jtag, unsigned char direction);
void usbprog_jtag_write_slice(struct usbprog_jtag *usbprog_jtag,unsigned char value);
unsigned char usbprog_jtag_get_port(struct usbprog_jtag *usbprog_jtag);
void usbprog_jtag_set_bit(struct usbprog_jtag *usbprog_jtag,int bit, int value);
int usbprog_jtag_get_bit(struct usbprog_jtag *usbprog_jtag, int bit);

int usbprog_speed(int speed)
{
	return ERROR_OK;
}

int usbprog_register_commands(struct command_context_s *cmd_ctx)
{
	return ERROR_OK;
}


int usbprog_execute_queue(void)
{
        jtag_command_t *cmd = jtag_command_queue; /* currently processed command */
        int scan_size;
        enum scan_type type;
        u8 *buffer;

        while (cmd)
        {
                switch (cmd->type)
                {
                        case JTAG_END_STATE:
#ifdef _DEBUG_JTAG_IO_
                                DEBUG("end_state: %i", cmd->cmd.end_state->end_state);
#endif
                                if (cmd->cmd.end_state->end_state != -1)
                                        usbprog_end_state(cmd->cmd.end_state->end_state);
                                break;
                        case JTAG_RESET:
#ifdef _DEBUG_JTAG_IO_
                                DEBUG("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);
#endif
                                if (cmd->cmd.reset->trst == 1)
                                {
                                        cur_state = TAP_TLR;
                                }
                                usbprog_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
                                break;
                        case JTAG_RUNTEST:
#ifdef _DEBUG_JTAG_IO_
                                DEBUG("runtest %i cycles, end in %i", cmd->cmd.runtest->num_cycles, cmd->cmd.runtest->end_state);
#endif
                                if (cmd->cmd.runtest->end_state != -1)
                                        usbprog_end_state(cmd->cmd.runtest->end_state);
                                usbprog_runtest(cmd->cmd.runtest->num_cycles);
                                break;
                        case JTAG_STATEMOVE:
#ifdef _DEBUG_JTAG_IO_
                                DEBUG("statemove end in %i", cmd->cmd.statemove->end_state);
#endif
                                if (cmd->cmd.statemove->end_state != -1)
                                        usbprog_end_state(cmd->cmd.statemove->end_state);
                                usbprog_state_move();
                                break;
                        case JTAG_PATHMOVE:
#ifdef _DEBUG_JTAG_IO_
                                DEBUG("pathmove: %i states, end in %i", cmd->cmd.pathmove->num_states, 
				cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);
#endif
                                usbprog_path_move(cmd->cmd.pathmove);
                                break;
                        case JTAG_SCAN:
#ifdef _DEBUG_JTAG_IO_
                                DEBUG("scan end in %i", cmd->cmd.scan->end_state);
#endif
                                if (cmd->cmd.scan->end_state != -1)
                                        usbprog_end_state(cmd->cmd.scan->end_state);
                                scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
                                type = jtag_scan_type(cmd->cmd.scan);
                                usbprog_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
                                if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
                                        return ERROR_JTAG_QUEUE_FAILED;
                                if (buffer)
                                        free(buffer);
                                break;
                        case JTAG_SLEEP:
#ifdef _DEBUG_JTAG_IO_
                                DEBUG("sleep %i", cmd->cmd.sleep->us);
#endif
                                jtag_sleep(cmd->cmd.sleep->us);
                                break;
                        default:
                                ERROR("BUG: unknown JTAG command type encountered");
                                exit(-1);
                }
                cmd = cmd->next;
        }

        return ERROR_OK;
}


int usbprog_init(void)
{
	usbprog_jtag_handle = usbprog_jtag_open();

	tms_chain_index=0;
	if(usbprog_jtag_handle==0){
		ERROR("Can't find USB JTAG Interface! Please check connection and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}
				
	INFO("USB JTAG Interface ready!");
				    
	usbprog_jtag_init(usbprog_jtag_handle);
	usbprog_reset(0, 0);
	usbprog_write(0, 0, 0);

	return ERROR_OK;
}

int usbprog_quit(void)
{
	
	return ERROR_OK;
}

/*************** jtag execute commands **********************/
void usbprog_end_state(enum tap_state state)
{
	if (tap_move_map[state] != -1)
		end_state = state;
	else
	{
		ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}


void usbprog_state_move(void) {

        int i=0, tms=0;
        u8 tms_scan = TAP_MOVE(cur_state, end_state);

        usbprog_jtag_write_tms(usbprog_jtag_handle,(char)tms_scan);
        for (i = 0; i < 7; i++)
        {
                tms = (tms_scan >> i) & 1;
        }
	
        cur_state = end_state;
}


void usbprog_path_move(pathmove_command_t *cmd)
{
        int num_states = cmd->num_states;
        int state_count;

        state_count = 0;
        while (num_states)
        {
                if (tap_transitions[cur_state].low == cmd->path[state_count])
                {
			//INFO("1");
                        usbprog_write(0, 0, 0);
                        usbprog_write(1, 0, 0);
                }
                else if (tap_transitions[cur_state].high == cmd->path[state_count])
                {
			//INFO("2");
                        usbprog_write(0, 1, 0);
                        usbprog_write(1, 1, 0);
                }
                else
                {
                        ERROR("BUG: %s -> %s isn't a valid TAP transition", tap_state_strings[cur_state], tap_state_strings[cmd->path[state_count]]);
                        exit(-1);
                }

                cur_state = cmd->path[state_count];
                state_count++;
                num_states--;
        }

        end_state = cur_state;
}


void usbprog_runtest(int num_cycles)
{
        int i;

        enum tap_state saved_end_state = end_state;


	/* only do a state_move when we're not already in RTI */
        if (cur_state != TAP_RTI)
        {
                usbprog_end_state(TAP_RTI);
                usbprog_state_move();
        }

        /* execute num_cycles */
	if(num_cycles>0)
	{
		usbprog_jtag_tms_send(usbprog_jtag_handle);
		usbprog_write(0, 0, 0);
	}
	else {
		usbprog_jtag_tms_send(usbprog_jtag_handle);
		//INFO("NUM CYCLES %i",num_cycles);
	}

        for (i = 0; i < num_cycles; i++)
        {
                usbprog_write(1, 0, 0);
                usbprog_write(0, 0, 0);
        }

        /* finish in end_state */
	/*
        usbprog_end_state(saved_end_state);
        if (cur_state != end_state)
                usbprog_state_move();
	*/
}



void usbprog_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size)
{
        enum tap_state saved_end_state = end_state;
        int bit_cnt;

        if (ir_scan)
                usbprog_end_state(TAP_SI);
        else
                usbprog_end_state(TAP_SD);

	//usbprog_jtag_tms_send(usbprog_jtag_handle);

        usbprog_state_move();
        usbprog_end_state(saved_end_state);

	usbprog_jtag_tms_send(usbprog_jtag_handle);

        if (type == SCAN_OUT) {
                usbprog_jtag_write_tdi(usbprog_jtag_handle,buffer, scan_size);
        }
        if (type == SCAN_IN) {
                usbprog_jtag_read_tdo(usbprog_jtag_handle,buffer, scan_size);
        }
        if (type == SCAN_IO) {
                usbprog_jtag_write_and_read(usbprog_jtag_handle,buffer, scan_size);
        }
        
        if (ir_scan)
                cur_state = TAP_PI;
        else
                cur_state = TAP_PD;

        if (cur_state != end_state)
                usbprog_state_move();
}

/*************** jtag wrapper functions *********************/

void usbprog_write(int tck, int tms, int tdi)
{
	 unsigned char output_value=0x00;

	 if (tms)
                output_value |= (1<<TMS_BIT);
	 if (tdi)
                output_value |= (1<<TDI_BIT);
	 if (tck)
                output_value |= (1<<TCK_BIT);

	 usbprog_jtag_write_slice(usbprog_jtag_handle,output_value);
}

/* (1) assert or (0) deassert reset lines */
void usbprog_reset(int trst, int srst)
{
        DEBUG("trst: %i, srst: %i", trst, srst);

        if(trst)
                usbprog_jtag_set_bit(usbprog_jtag_handle,5,0);
        else
                usbprog_jtag_set_bit(usbprog_jtag_handle,5,1);

        if(srst)
                usbprog_jtag_set_bit(usbprog_jtag_handle,4,0);
        else
                usbprog_jtag_set_bit(usbprog_jtag_handle,4,1);
}



/*************** jtag lowlevel functions ********************/


	struct usb_bus *busses;
struct usbprog_jtag* usbprog_jtag_open()
{
	struct usb_dev_handle* usb_handle;
	struct usb_bus *bus;
	struct usb_device *dev;

	struct usbprog_jtag * tmp;

	tmp = (struct usbprog_jtag*)malloc(sizeof(struct usbprog_jtag));

usb_set_debug(10);	
	usb_init();
	usb_find_busses();
	usb_find_devices();

	
	busses = usb_get_busses();

	/* find usbprog_jtag device in usb bus */

	for (bus = busses; bus; bus = bus->next){
		for (dev = bus->devices; dev; dev = dev->next){
			/* condition for sucessfully hit (too bad, I only check the vendor id)*/
			if (dev->descriptor.idVendor == VID && dev->descriptor.idProduct == PID) {
				tmp->usb_handle = usb_open(dev);
				usb_set_configuration (tmp->usb_handle,1);
				usb_claim_interface(tmp->usb_handle, 0);
				usb_set_altinterface(tmp->usb_handle,0);
				return tmp;
			}
		} 
	}
	return 0;
}


void usbprog_jtag_close(struct usbprog_jtag *usbprog_jtag)
{
	usb_close(usbprog_jtag->usb_handle);
	free(usbprog_jtag);
}


unsigned char usbprog_jtag_message(struct usbprog_jtag *usbprog_jtag, char *msg, int msglen)
{
	int res = usb_bulk_write(usbprog_jtag->usb_handle,3,msg,msglen,100);
	if(msg[0]==2||msg[0]==1||msg[0]==4||msg[0]==0||msg[0]==6||msg[0]==0x0A||msg[0]==9)
		return 1;  
	if(res == msglen) {
		//INFO("HALLLLOOO %i",(int)msg[0]);
		res =  usb_bulk_read(usbprog_jtag->usb_handle,0x82, msg, 2, 100);
		if (res > 0)
			return (unsigned char)msg[1];
		else 
			return -1;
	}
	else
		return -1;
	return 0;
}

void usbprog_jtag_init(struct usbprog_jtag *usbprog_jtag)
{
	usbprog_jtag_set_direction(usbprog_jtag, 0xFE);
}


void usbprog_jtag_write_and_read(struct usbprog_jtag *usbprog_jtag, char * buffer, int size)
{
	char tmp[64];	// fastes packet size for usb controller
	int send_bits,bufindex=0,fillindex=0,i,j,complete=size,loops;

	char swap;
	// 61 byte can be transfered (488 bit)
    
	while(size > 0) {  
		if(size > 488) {
			send_bits = 488;
			size = size - 488;
			loops = 61;
		} else {
			send_bits = size;
			loops = size/8;
			loops++;
			size = 0;
		}
		tmp[0] = WRITE_AND_READ;
		tmp[1] = (char)(send_bits>>8); // high 
		tmp[2] = (char)(send_bits);    // low
		i=0; 

		for(i=0;i < loops ;i++) {
			tmp[3+i]=buffer[bufindex];
			bufindex++;
		}
    
		if(usb_bulk_write(usbprog_jtag->usb_handle,3,tmp,64,1000)==64)
		{
			//INFO("HALLLLOOO2 %i",(int)tmp[0]);
			usleep(1);
			int timeout=0;
			while(usb_bulk_read(usbprog_jtag->usb_handle,0x82, tmp, 64, 1000) < 1){
				timeout++;
				if(timeout>10)
					break;
			}	

			for(i=0;i<loops ;i++) {
				swap =  tmp[3+i];
				buffer[fillindex++] = swap;
			} 
		}
	}
}


void usbprog_jtag_read_tdo(struct usbprog_jtag *usbprog_jtag, char * buffer, int size)
{
	char tmp[64];	// fastes packet size for usb controller
	int send_bits,bufindex=0,fillindex=0,i,j,complete=size,loops;

	char swap;
	// 61 byte can be transfered (488 bit)
    
	while(size > 0) {  
		if(size > 488) {
			send_bits = 488;
			size = size - 488;
			loops = 61;
		} else {
			send_bits = size;
			loops = size/8;
			loops++;
			size = 0;
		}
		tmp[0] = WRITE_AND_READ;
		tmp[1] = (char)(send_bits>>8); // high 
		tmp[2] = (char)(send_bits);    // low
    
		usb_bulk_write(usbprog_jtag->usb_handle,3,tmp,3,1000);
    
		//INFO("HALLLLOOO3 %i",(int)tmp[0]);
		int timeout=0;
		usleep(1);
		while(usb_bulk_read(usbprog_jtag->usb_handle,0x82, tmp, 64, 10) < 1){
			timeout++;
			if(timeout>10)
				break;
		}

		for(i=0;i<loops ;i++) {
			swap =  tmp[3+i];
			buffer[fillindex++] = swap;
		} 
	}
}

void usbprog_jtag_write_tdi(struct usbprog_jtag *usbprog_jtag, char * buffer, int size)
{
	char tmp[64];	// fastes packet size for usb controller
	int send_bits,bufindex=0,fillindex=0,i,j,complete=size,loops;
	char swap;
	// 61 byte can be transfered (488 bit)
	while(size > 0) {  
		if(size > 488) {
			send_bits = 488;
			size = size - 488;
			loops = 61;
		} else {
			send_bits = size;
			loops = size/8;
			//if(loops==0)
			loops++;
			size = 0;
		}
		tmp[0] = WRITE_TDI;
		tmp[1] = (char)(send_bits>>8); // high 
		tmp[2] = (char)(send_bits);    // low
		i=0; 

		for(i=0;i < loops ;i++) {
			tmp[3+i]=buffer[bufindex];
			bufindex++;
		}
		usb_bulk_write(usbprog_jtag->usb_handle,3,tmp,64,1000);
	}
}


void usbprog_jtag_write_tms(struct usbprog_jtag *usbprog_jtag, char tms_scan)
{
	usbprog_jtag_tms_collect(tms_scan);
}


void usbprog_jtag_set_direction(struct usbprog_jtag *usbprog_jtag, unsigned char direction)
{
	char tmp[2];
	tmp[0] = PORT_DIRECTION;
	tmp[1] = (char)direction;
	usbprog_jtag_message(usbprog_jtag,tmp,2);
}

void usbprog_jtag_write_slice(struct usbprog_jtag *usbprog_jtag,unsigned char value)
{
	char tmp[2];
	tmp[0] = PORT_SET;
	tmp[1] = (char)value;
	usbprog_jtag_message(usbprog_jtag,tmp,2);
}

unsigned char usbprog_jtag_get_port(struct usbprog_jtag *usbprog_jtag)
{
	char tmp[2];
	tmp[0] = PORT_GET;
	tmp[1] = 0x00;
	return usbprog_jtag_message(usbprog_jtag,tmp,2);
}


void usbprog_jtag_set_bit(struct usbprog_jtag *usbprog_jtag,int bit, int value)
{
	char tmp[3];
	tmp[0] = PORT_SETBIT;
	tmp[1] = (char)bit;
	if(value==1)  
		tmp[2] = 0x01;
	else
		tmp[2] = 0x00;
	usbprog_jtag_message(usbprog_jtag,tmp,3);
}

int usbprog_jtag_get_bit(struct usbprog_jtag *usbprog_jtag, int bit)
{
	char tmp[2];
	tmp[0] = PORT_GETBIT;
	tmp[1] = (char)bit;

	if(usbprog_jtag_message(usbprog_jtag,tmp,2)>0)
		return 1;
	else
		return 0;
}

void usbprog_jtag_tms_collect(char tms_scan){
	tms_chain[tms_chain_index]=tms_scan;
	tms_chain_index++;
}

void usbprog_jtag_tms_send(struct usbprog_jtag *usbprog_jtag){
	int i;
	//INFO("TMS SEND");
	if(tms_chain_index>0) {
		char tmp[tms_chain_index+2];
		tmp[0] = WRITE_TMS_CHAIN;
		tmp[1] = (char)(tms_chain_index);
		for(i=0;i<tms_chain_index+1;i++)
			tmp[2+i] = tms_chain[i];
		usb_bulk_write(usbprog_jtag->usb_handle,3,tmp,tms_chain_index+2,1000);
		tms_chain_index=0;
	}
}

