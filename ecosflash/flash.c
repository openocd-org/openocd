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
