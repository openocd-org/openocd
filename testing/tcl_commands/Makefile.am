# SPDX-License-Identifier: GPL-2.0-or-later

TESTS =

if DUMMY
TESTS += \
	test-target-create-command.cfg \
	test-target-configure-cget-command.cfg
endif

EXTRA_DIST = utils.tcl $(TESTS)

TEST_EXTENSIONS = .cfg
CFG_LOG_COMPILER = $(top_builddir)/src/openocd
AM_CFG_LOG_FLAGS = -f $(abs_srcdir)/utils.tcl -f
