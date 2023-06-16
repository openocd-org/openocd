#!/bin/bash
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (C) 2023 by NanoXplore, France - all rights reserved

[ -z "${XILINX_HOME}" ] && export XILINX_HOME=/home/software/Xilinx/ISE/14.7/ISE_DS/ISE
export PATH="$XILINX_HOME:$PATH"
echo "SET XILINX_HOME to ${XILINX_HOME}"
# This is needed for isim
XILINX_HOME_BASE=${XILINX_HOME}/..
for part in common EDK PlanAhead ISE
do
	el=${XILINX_HOME_BASE}/${part}
	.  ${el}/.settings64.sh ${el}
done
