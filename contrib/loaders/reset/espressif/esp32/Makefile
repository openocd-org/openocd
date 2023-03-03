# SPDX-License-Identifier: GPL-2.0-or-later

# ESP32 Makefile to compile the SoC reset program
# Copyright (C) 2022 Espressif Systems Ltd.

# Prefix for ESP32 cross compilers (can include a directory path)
CROSS ?= xtensa-esp32-elf-

APP_ARCH := xtensa
APP_CHIP := ESP32
APP_CHIP_PATH := $(shell pwd)
SRCS := $(APP_CHIP_PATH)/esp32_cpu_reset_handler.S

CFLAGS :=
LDFLAGS :=

INCLUDES :=
DEFINES :=

include ../common.mk
