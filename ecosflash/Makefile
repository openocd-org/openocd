# Create OpenOCD eCos flash driver
# Syntax: make INSTALL_DIR=ecosinstalldir OUTPUT=outputname

include $(INSTALL_DIR)/include/pkgconf/ecos.mak

all:
	$(ECOS_COMMAND_PREFIX)gcc $(ECOS_GLOBAL_CFLAGS) $(ECOS_GLOBAL_LDFLAGS) -g -o debug_$(OUTPUT).elf -nostdlib  flash.S flash.c -Wl,--gc-sections -I$(INSTALL_DIR)/include -Wl,$(INSTALL_DIR)/lib/libtarget.a -Wl,-Map,flash.map   
	cp debug_$(OUTPUT).elf $(OUTPUT).elf
	$(ECOS_COMMAND_PREFIX)strip $(OUTPUT).elf
	echo Flash driver $(OUTPUT).elf

