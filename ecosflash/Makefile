#*************************************************************************
#*   Copyright (C) 2008 Øyvind Harboe                                     #*
#*   oyvind.harboe@zylin.com                                              #*
#*                                                                        #*
#*   This program is free software; you can redistribute it and/or modify #*
#*   it under the terms of the GNU General Public License as published by #*
#*   the Free Software Foundation; either version 2 of the License, or    #*
#*   (at your option) any later version.                                  #*
#*                                                                        #*
#*   This program is distributed in the hope that it will be useful,      #*
#*   but WITHOUT ANY WARRANTY; without even the implied warranty of       #*
#*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        #*
#*   GNU General Public License for more details.                         #*
#*                                                                        #*
#*   You should have received a copy of the GNU General Public License    #*
#*   along with this program; if not, write to the                        #*
#*   Free Software Foundation, Inc.,                                      #*
#*   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.            #*
#***************************************************************************
# Create OpenOCD eCos flash driver
# Syntax: make INSTALL_DIR=ecosinstalldir OUTPUT=outputname

include $(INSTALL_DIR)/include/pkgconf/ecos.mak

all:
	$(ECOS_COMMAND_PREFIX)gcc $(ECOS_GLOBAL_CFLAGS) $(ECOS_GLOBAL_LDFLAGS) -g -o debug_$(OUTPUT).elf -nostdlib  flash.S flash.c -Wl,--gc-sections -I$(INSTALL_DIR)/include -Wl,$(INSTALL_DIR)/lib/libtarget.a -Wl,-Map,flash.map   
	cp debug_$(OUTPUT).elf $(OUTPUT).elf
	$(ECOS_COMMAND_PREFIX)strip $(OUTPUT).elf
	echo Flash driver $(OUTPUT).elf

