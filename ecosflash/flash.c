/*
#####ECOSGPLCOPYRIGHTBEGIN####
## -------------------------------------------
## This file is part of eCos, the Embedded Configurable Operating System.
## Copyright (C) 2008 Øyvind Harboe
##
## eCos is free software; you can redistribute it and/or modify it under
## the terms of the GNU General Public License as published by the Free
## Software Foundation; either version 2 or (at your option) any later version.
##
## eCos is distributed in the hope that it will be useful, but WITHOUT ANY
## WARRANTY; without even the implied warranty of MERCHANTABILITY or
## FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
## for more details.
##
## You should have received a copy of the GNU General Public License along
## with eCos; if not, write to the Free Software Foundation, Inc.,
## 51 Franklin St, Fifth Floor, Boston, MA 02110-1301, USA.
##
## As a special exception, if other files instantiate templates or use macros
## or inline functions from this file, or you compile this file and link it
## with other works to produce a work based on this file, this file does not
## by itself cause the resulting work to be covered by the GNU General Public
## License. However the source code for this file must still be made available
## in accordance with section (3) of the GNU General Public License.
##
## This exception does not invalidate any other reasons why a work based on
## this file might be covered by the GNU General Public License.
## -------------------------------------------
#####ECOSGPLCOPYRIGHTEND####
*/

#include <string.h>
#define _FLASH_PRIVATE_
#include <cyg/io/flash.h>



int myprintf(char *format, ...)
{
  return 0;
}

extern char _start_bss_clear;
extern char __bss_end__;

int init()
{
	// set up runtime environment
	char *t;
	for (t=&_start_bss_clear; t<&__bss_end__; t++)
	{
		*t=0;
	}
	return flash_init((_printf *)&myprintf);
	
}


int checkFlash(void *addr, int len)
{
    // Return error for illegal addresses
    if ((addr<flash_info.start)||(addr>flash_info.end))
    	return FLASH_ERR_INVALID;
    if ((((cyg_uint8 *)addr)+len)>(cyg_uint8 *)flash_info.end)
    	return FLASH_ERR_INVALID;
    return FLASH_ERR_OK;
}


int erase(void *address, int len)	
{
	int retval;
	void *failAddress;
	
	retval=checkFlash(address, len);
	if (retval!=0)
		return retval;
	
	retval=init();
	if (retval!=0)
		return retval;
	return flash_erase(address, len, &failAddress);
}



extern char _end;

// Data follows immediately after program, long word aligned.
int program(void *buffer, void *address, int len)	
{
	int retval;
	void *failAddress;
	retval=checkFlash(address, len);
	if (retval!=0)
		return retval;
	
	retval=init();
	if (retval!=0)
		return retval;
	//int flash_program(void *_addr, void *_data, int len, void **err_addr)
	return flash_program(address, buffer, len, &failAddress);
}
