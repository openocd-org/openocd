/***************************************************************************
 *   Copyright 2016,2017 Sony Video & Sound Products Inc.                  *
 *   Masatoshi Tateishi - Masatoshi.Tateishi@jp.sony.com                   *
 *   Masayuki Ishikawa - Masayuki.Ishikawa@jp.sony.com                     *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_RTOS_NUTTX_HEADER_H
#define OPENOCD_RTOS_NUTTX_HEADER_H

/*  gdb script to update the header file
  according to kernel version and build option
  before executing function awareness
  kernel symbol must be loaded : symbol nuttx

define awareness
 set logging off
 set logging file nuttx_header.h
 set logging on

 printf "#define PID  %p\n",&((struct tcb_s *)(0))->pid
 printf "#define XCPREG  %p\n",&((struct tcb_s *)(0))->xcp.regs
 printf "#define STATE %p\n",&((struct tcb_s *)(0))->task_state
 printf "#define NAME %p\n",&((struct tcb_s *)(0))->name
 printf "#define NAME_SIZE %d\n",sizeof(((struct tcb_s *)(0))->name)
 end


 OR ~/.gdbinit


define hookpost-file

 if &g_readytorun != 0
  eval "monitor nuttx.pid_offset %d", &((struct tcb_s *)(0))->pid
  eval "monitor nuttx.xcpreg_offset %d", &((struct tcb_s *)(0))->xcp.regs
  eval "monitor nuttx.state_offset %d", &((struct tcb_s *)(0))->task_state
  eval "monitor nuttx.name_offset %d", &((struct tcb_s *)(0))->name
  eval "monitor nuttx.name_size %d", sizeof(((struct tcb_s *)(0))->name)
 end

end

*/

/* default offset */
#define PID  0xc
#define XCPREG  0x70
#define STATE 0x19
#define NAME 0xb8
#define NAME_SIZE 32

/* defconfig of nuttx */
/* #define CONFIG_DISABLE_SIGNALS */
#define CONFIG_DISABLE_MQUEUE
/* #define CONFIG_PAGING */


#endif /* OPENOCD_RTOS_NUTTX_HEADER_H */
