# SPDX-License-Identifier: BSD-3-Clause
# Copyright (C) 2023 by NanoXplore, France - all rights reserved

# Needed by timing test
export PROJECT := angie_bitstream
TARGET_PART := xc6slx9-2tqg144
export TOPLEVEL := S609

# Detects the ROOT dir from the .git marker
sp :=
sp +=
_walk = $(if $1,$(wildcard /$(subst $(sp),/,$1)/$2) $(call _walk,$(wordlist 2,$(words $1),x $1),$2))
_find = $(firstword $(call _walk,$(strip $(subst /, ,$1)),$2))
_ROOT := $(patsubst %/.git,%,$(call _find,$(CURDIR),.git))

SHELL := /bin/bash
TOP_DIR := $(realpath $(_ROOT))
HDL_DIR := $(CURDIR)
SRC_DIR := $(HDL_DIR)/src
TOOLS_DIR := $(TOP_DIR)/tools/build
COMMON_DIR := $(TOP_DIR)/common/hdl
COMMON_HDL_DIR := $(COMMON_DIR)/src
COMMON_LIBS := $(COMMON_DIR)/libs
HDL_BUILD_DIR := $(HDL_DIR)/build
OUTPUT_DIR ?= $(HDL_BUILD_DIR)/output
FINAL_OUTPUT_DIR := $(OUTPUT_DIR)/$(PROJECT)

# Tools
MKDIR := mkdir -p
CP := cp -f

HDL_SRC_PATH := $(addprefix $(COMMON_DIR)/ips/, $(HDL_IPS)) $(HDL_DIR)
VHDSOURCE += $(foreach ip,$(HDL_SRC_PATH),$(wildcard $(ip)/src/*.vhd))
VSOURCE += $(foreach ip,$(HDL_SRC_PATH),$(wildcard $(ip)/src/*.v))
VSOURCE += $(foreach ip,$(HDL_SRC_PATH),$(wildcard $(ip)/src/*.vh))

CONSTRAINTS ?= $(SRC_DIR)/$(PROJECT).ucf

COMMON_OPTS := -intstyle xflow
XST_OPTS :=
NGDBUILD_OPTS :=
MAP_OPTS := -mt 2
PAR_OPTS := -mt 4
BITGEN_OPTS := -g Binary:Yes

XILINX_PLATFORM := lin64
PATH := $(PATH):$(XILINX_HOME)/bin/$(XILINX_PLATFORM)

RUN = @echo -ne "\n\n\e[1;33m======== $(1) ========\e[m\n\n"; \
	cd $(HDL_BUILD_DIR) && $(XILINX_HOME)/bin/$(XILINX_PLATFORM)/$(1)

compile: $(HDL_BUILD_DIR)/$(PROJECT).bin

install: $(HDL_BUILD_DIR)/$(PROJECT).bin
	$(MKDIR) $(FINAL_OUTPUT_DIR)
	$(CP) $(HDL_BUILD_DIR)/$(PROJECT).bin $(FINAL_OUTPUT_DIR)

clean:
	rm -rf $(HDL_BUILD_DIR)

$(HDL_BUILD_DIR)/$(PROJECT).bin: $(HDL_BUILD_DIR)/$(PROJECT).ncd
	$(call RUN,bitgen) $(COMMON_OPTS) $(BITGEN_OPTS) \
	-w $(PROJECT).ncd $(PROJECT).bit

$(HDL_BUILD_DIR)/$(PROJECT).ncd: $(HDL_BUILD_DIR)/$(PROJECT).map.ncd
	$(call RUN,par) $(COMMON_OPTS) $(PAR_OPTS) \
		-w $(PROJECT).map.ncd $(PROJECT).ncd $(PROJECT).pcf

$(HDL_BUILD_DIR)/$(PROJECT).map.ncd: $(HDL_BUILD_DIR)/$(PROJECT).ngd
	$(call RUN,map) $(COMMON_OPTS) $(MAP_OPTS) \
		-p $(TARGET_PART) \
		-w $(PROJECT).ngd -o $(PROJECT).map.ncd $(PROJECT).pcf

$(HDL_BUILD_DIR)/$(PROJECT).ngd: $(HDL_BUILD_DIR)/$(PROJECT).ngc
	$(call RUN,ngdbuild) $(COMMON_OPTS) $(NGDBUILD_OPTS) \
		-p $(TARGET_PART) -uc $(CONSTRAINTS) \
		$(PROJECT).ngc $(PROJECT).ngd

$(HDL_BUILD_DIR)/$(PROJECT).ngc: $(HDL_BUILD_DIR)/$(PROJECT).prj $(HDL_BUILD_DIR)/$(PROJECT).scr
	$(call RUN,xst) $(COMMON_OPTS) -ifn $(PROJECT).scr

$(HDL_BUILD_DIR)/$(PROJECT).scr: | $(HDL_BUILD_DIR)
	@echo "Updating $@"
	@mkdir -p $(HDL_BUILD_DIR)
	@rm -f $@
	@echo "run" \
	    "-ifn $(PROJECT).prj" \
	    "-ofn $(PROJECT).ngc" \
	    "-ifmt mixed" \
	    "$(XST_OPTS)" \
	    "-top $(TOPLEVEL)" \
	    "-ofmt NGC" \
	    "-p $(TARGET_PART)" \
	> $(HDL_BUILD_DIR)/$(PROJECT).scr

$(HDL_BUILD_DIR)/$(PROJECT).prj: | $(HDL_BUILD_DIR)
	@echo "Updating $@"
	@rm -f $@
	@$(foreach file,$(VSOURCE),echo "verilog work \"$(file)\"" >> $@;)
	@$(foreach file,$(VHDSOURCE),echo "vhdl work \"$(file)\"" >> $@;)
	@$(foreach lib,$(HDL_LIBS),$(foreach file,$(wildcard $(COMMON_LIBS)/$(lib)/src/*.vhd),echo "vhdl $(lib) \"$(file)\"" >> $@;))
	@$(foreach lib,$(HDL_LIBS),$(foreach file,$(wildcard $(COMMON_LIBS)/$(lib)/src/*.v),echo "verilog $(lib) \"$(file)\"" >> $@;))
	@$(foreach lib,$(HDL_LIBS),$(foreach file,$(wildcard $(COMMON_LIBS)/$(lib)/src/*.vh),echo "verilog $(lib) \"$(file)\"" >> $@;))

$(HDL_BUILD_DIR):
	$(MKDIR) $(HDL_BUILD_DIR)

.PHONY: clean compile install

